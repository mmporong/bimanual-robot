import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from bench_relative_ik import BenchJointMap, audit_self_collision, plan_delta
from ik_pick_place import DEFAULT_CALIBRATION, FixedContactChain, JOINTS
from plan_body_side_grasp import URDF_PATH, base_positions, grasp_axes


@pytest.fixture
def mapping_input():
    calibration = DEFAULT_CALIBRATION.read_bytes()
    mapping = {
        "schema": "bench_provisional_joint_map_v1",
        "scope": "left_stock_cup_bench_only", "joint_order": JOINTS.copy(),
        "offset_deg": [0, 0, 0, 0, 90],
        "calibration_sha256": hashlib.sha256(calibration).hexdigest(),
        "urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
        "physical_mapping_precision_verified": False,
        "physical_grasp_verified": False,
        "provenance": {"observed_at": "2026-09-30", "evidence": "synthetic test fixture",
                       "assumptions": "not a measured robot mapping"},
    }
    return calibration, mapping


def test_nominal_midpoint_and_inverse_share_offset(mapping_input):
    calibration, mapping = mapping_input
    joint_map = BenchJointMap(calibration, mapping)
    cal = json.loads(calibration)
    raw = [round((cal[n]["range_min"] + cal[n]["range_max"]) / 2) for n in JOINTS]
    q = joint_map.to_model_deg(raw)
    assert abs(q[4] - 90) <= 180 / 4095
    assert joint_map.to_raw(q.tolist()) == raw
    assert joint_map.to_raw((q + [0, 0, 0, 0, -90]).tolist())[4] < raw[4]


def test_result_mapping_cannot_mutate_future_plans(mapping_input):
    joint_map = BenchJointMap(*mapping_input)
    raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    first = plan_delta(raw, joint_map, [-.035, 0, -.020], [0, 0, .01])
    first["mapping"]["physical_mapping_precision_verified"] = True
    first["mapping"]["offset_deg"][4] = 0
    second = plan_delta(raw, joint_map, [-.035, 0, -.020], [0, 0, .01])
    assert second["mapping"]["physical_mapping_precision_verified"] is False
    assert second["mapping"]["offset_deg"][4] == 90
    assert second["target_raw"] == first["target_raw"]


def test_stock_wrist_observation_is_provisional_and_round_trips(mapping_input):
    _, mapping = mapping_input
    calibration = (Path(__file__).parent / "fixtures/so101_left_20260929.json").read_bytes()
    mapping = {**mapping, "calibration_sha256": hashlib.sha256(calibration).hexdigest()}
    joint_map = BenchJointMap(calibration, mapping)
    cal = json.loads(calibration)
    raw = [round((cal[n]["range_min"] + cal[n]["range_max"]) / 2) for n in JOINTS]
    raw[4] = 2076
    q = joint_map.to_model_deg(raw)
    assert q[4]-90 == pytest.approx(2.5054945054945)
    assert q[4] == pytest.approx(92.5054945054945)
    assert joint_map.to_raw(q.tolist())[4] == 2076
    assert joint_map.mapping["physical_mapping_precision_verified"] is False
    assert joint_map.mapping["physical_grasp_verified"] is False


@pytest.mark.parametrize("key,value", [
    ("calibration_sha256", "0" * 64), ("urdf_sha256", "0" * 64),
    ("physical_mapping_precision_verified", True), ("scope", "all_robots"),
    ("physical_grasp_verified", True), ("physical_execution_ready", True),
    ("offset_deg", [0, 0, 0, 0, float("nan")]),
    ("offset_deg", [0, 0, 0, 0, 181]), ("offset_deg", [0, 0]),
    ("offset_deg", [5, 0, 0, 0, 90]),
    ("provenance", {}),
])
def test_invalid_mapping_rejected(mapping_input, key, value):
    calibration, mapping = mapping_input
    with pytest.raises(ValueError):
        BenchJointMap(calibration, {**mapping, key: value})


def test_offset_applies_to_hardware_limits(mapping_input):
    calibration, mapping = mapping_input
    joint_map = BenchJointMap(calibration, mapping)
    chain = FixedContactChain({"contact_center_tool_m": [0, 0, 0]})
    joint_map.apply_limits(chain)
    cal = json.loads(calibration)["wrist_roll"]
    span = (cal["range_max"] - cal["range_min"]) * 180 / 4095
    assert np.degrees(chain.limits["left_wrist_roll"][0]) == pytest.approx(max(-157.21, 90-span), abs=.02)


def test_relative_lift_preserves_model_axes_and_quantized_fk(mapping_input):
    calibration, mapping = mapping_input
    joint_map = BenchJointMap(calibration, mapping)
    start_raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    contact = [-.035, 0, -.020]
    plan = plan_delta(start_raw, joint_map, contact, [0, 0, .01])
    assert plan["endpoint_measurement"]["position_error_mm"] < 2
    assert plan["endpoint_measurement"]["closing_horizontal_error_deg"] < 2
    assert plan["physical_mapping_precision_verified"] is False
    assert plan["physical_grasp_verified"] is False
    assert plan["motion_command_emitted"] is False
    assert plan["hardware_accessed"] is False
    chain = FixedContactChain({"contact_center_tool_m": contact})
    for sample in plan["samples"]:
        q = joint_map.to_model_deg(sample["raw_ticks"])
        assert q.tolist() == pytest.approx(sample["model_joint_deg"])
        tool = chain.transforms(base_positions("left", np.radians(q)))["left_tool0"]
        assert tool[:3, 3] == pytest.approx(sample["contact_center_m"])
    q_final = joint_map.to_model_deg(plan["target_raw"])
    axes = grasp_axes(chain.transforms(base_positions("left", np.radians(q_final)))["left_tool0"], "left")
    assert max(abs(axis[2]) for axis in axes) < np.sin(np.radians(2))
    for first, second in zip([start_raw] + plan["waypoints_raw"][:-1], plan["waypoints_raw"]):
        assert max(abs(a-b) for a, b in zip(first, second)) <= 5


@pytest.mark.parametrize("raw", [[0]*5, [float("nan")]*5, [1.5]*5, [1000]*4])
def test_invalid_raw_is_not_clipped(mapping_input, raw):
    with pytest.raises(ValueError):
        BenchJointMap(*mapping_input).to_model_deg(raw)


def test_large_relative_command_rejected(mapping_input):
    joint_map = BenchJointMap(*mapping_input)
    start_raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    with pytest.raises(ValueError):
        plan_delta(start_raw, joint_map, [0, 0, 0], [.1, 0, 0])


def test_start_limit_violation_is_not_hidden_by_seed_clipping(mapping_input):
    joint_map = BenchJointMap(*mapping_input)
    raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    raw[3] = joint_map.calibration["wrist_flex"]["range_min"]
    with pytest.raises(ValueError):
        plan_delta(raw, joint_map, [-.035, 0, -.020], [0, 0, .01])


def test_collision_audit_uses_same_mapped_raw_without_claiming_world_clear(mapping_input):
    joint_map = BenchJointMap(*mapping_input)
    raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    plan = plan_delta(raw, joint_map, [-.035, 0, -.020], [0, 0, .01])
    result = audit_self_collision(plan, joint_map, 1.45)
    assert result["sampled_self_collision_clear"]
    assert result["sample_count"] == len(plan["samples"])
    assert result["continuous_collision_verified"] is False
    assert result["external_world_collision_verified"] is False
    assert result["physical_execution_ready"] is False


@pytest.mark.parametrize("tamper", ["empty", "raw", "mapping", "fk", "claimed_success"])
def test_invalid_collision_evidence_is_rejected(mapping_input, tamper):
    joint_map = BenchJointMap(*mapping_input)
    raw = joint_map.to_raw([0, 30, 30, -60, 92.7893])
    plan = plan_delta(raw, joint_map, [-.035, 0, -.020], [0, 0, .01])
    if tamper == "empty":
        plan["samples"] = []
    elif tamper == "raw":
        plan["samples"][-1]["raw_ticks"] = [1000] * 5
    elif tamper == "mapping":
        plan["mapping"] = {}
    elif tamper == "fk":
        plan["samples"][-1]["contact_center_m"][0] += .01
    else:
        plan["physical_grasp_verified"] = True
    with pytest.raises(ValueError):
        audit_self_collision(plan, joint_map, 1.45)
