import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

import prepare_bench_cup_pregrasp as pregrasp
from test_ik_reset_clear_step import CALIBRATION_BYTES, snapshot
from test_ik_pick_place import reference_fixture
from servo.joint_reference import raw_to_joint_deg, build_reference
import ik_pick_place


MEASUREMENT = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             "calibration/workcells/bench_cup_20260929.yaml").read_text())


def test_current_calibration_produces_bounded_nominal_endpoints():
    calibration_bytes = ik_pick_place.DEFAULT_CALIBRATION.read_bytes()
    cal = json.loads(calibration_bytes)
    raw = ik_pick_place.raw_for([0, -80, 80, 60, 0], 0, calibration_bytes)
    start = snapshot()
    start.update(raw_ticks=raw, calibration_sha256=hashlib.sha256(calibration_bytes).hexdigest(),
                 joint_deg=pregrasp._snapshot_joint_deg(raw[:5], cal))
    result = pregrasp.build(MEASUREMENT, start, calibration_bytes)
    assert result["endpoints_ready_for_path_review"]
    for stage in result["stages"]:
        assert stage["quantized_endpoint_check"]["joint_margin_deg"] >= 3
        for name, value in zip(pregrasp.JOINTS, stage["arm_raw_ticks"]):
            assert cal[name]["range_min"] <= value <= cal[name]["range_max"]


def test_pregrasp_has_no_grip_or_execution_claim():
    result = pregrasp.build(MEASUREMENT, snapshot(), CALIBRATION_BYTES)
    assert not result["hardware_accessed"] and not result["motion_command_emitted"]
    assert not result["physical_execution_ready"]
    assert not result["gripper_raw_command_available"] and not result["close_command_available"]
    assert not result["start_to_endpoint_path_verified"]
    assert [stage["phase"] for stage in result["stages"]] == ["ALIGN_ABOVE_BEHIND_CUP", "PREGRASP_ABOVE_CUP"]
    assert result["endpoints_ready_for_path_review"]
    assert abs(result["stages"][0]["joint_deg"][4] - result["stages"][1]["joint_deg"][4]) < 5
    base = pregrasp.FixedContactChain({"contact_center_tool_m": MEASUREMENT["nominal_pregrasp"]["contact_center_tool_m"]}).transforms(
        pregrasp.base_positions("left", np.zeros(5)))["left_base_link"][:3, 3]
    assert np.allclose(np.asarray(result["stages"][0]["contact_center_target_m"]) - base, [.270, 0, .170])
    assert np.allclose(np.asarray(result["stages"][1]["contact_center_target_m"]) - base, [.350, 0, .170])
    assert result["midbody_behind_cup_candidate"]["endpoint_violations"]
    json.dumps(result, allow_nan=False)


def test_aligned_snapshot_and_endpoint_round_trip_use_same_reference():
    start = snapshot()
    reference = reference_fixture(CALIBRATION_BYTES)
    start["joint_reference"] = reference
    start["joint_deg"] = raw_to_joint_deg(start["raw_ticks"][:5], CALIBRATION_BYTES, reference)
    result = pregrasp.build(MEASUREMENT, start, CALIBRATION_BYTES)
    assert result["joint_reference"] == reference
    for stage in result["stages"]:
        actual_deg = raw_to_joint_deg(stage["arm_raw_ticks"], CALIBRATION_BYTES, reference)
        assert max(abs(a-b) for a, b in zip(actual_deg, stage["joint_deg"])) < .05
    assert not result["physical_execution_ready"]
    start["joint_deg"][0] += 2
    with pytest.raises(ValueError, match="각도 불일치"):
        pregrasp.build(MEASUREMENT, start, CALIBRATION_BYTES)


def test_asymmetric_reference_limits_select_feasible_roll_branch():
    # 회귀용 비대칭 영점: -87도 손목 해는 raw 하한 밖, +93도 해는 범위 안.
    start = snapshot()
    fixture = reference_fixture(CALIBRATION_BYTES)
    reference_raw = [2000, 1900, 1500, 2800, 980, start["raw_ticks"][5]]
    reference = build_reference(CALIBRATION_BYTES, reference_raw, [0]*5,
                                fixture["urdf_sha256"], fixture["provenance"])
    start["joint_reference"] = reference
    start["joint_deg"] = raw_to_joint_deg(start["raw_ticks"][:5], CALIBRATION_BYTES, reference)
    result = pregrasp.build(MEASUREMENT, start, CALIBRATION_BYTES)
    assert result["endpoints_ready_for_path_review"]
    cal = json.loads(CALIBRATION_BYTES)
    for stage in result["stages"]:
        assert 85 < stage["joint_deg"][4] < 100
        for name, raw in zip(pregrasp.JOINTS, stage["arm_raw_ticks"]):
            assert cal[name]["range_min"] <= raw <= cal[name]["range_max"]
        assert stage["quantized_endpoint_check"]["position_error_mm"] < 2
    assert abs(result["stages"][0]["joint_deg"][4] - result["stages"][1]["joint_deg"][4]) < 5
    assert not result["physical_execution_ready"]


@pytest.mark.parametrize("changed", ["unit", "diameter", "backoff", "snapshot", "execution_ready"])
def test_unverified_or_mismatched_inputs_are_rejected(changed):
    data, start = copy.deepcopy(MEASUREMENT), snapshot()
    if changed == "unit":
        data["cup_forward"]["unit"] = "mm"
    elif changed == "diameter":
        data["cup_diameter"]["value"] = .070
    elif changed == "backoff":
        data["nominal_pregrasp"]["selected_backoff_m"] = .060
    elif changed == "execution_ready":
        data["physical_execution_ready"] = True
    else:
        start["calibration_sha256"] = "different"
    with pytest.raises(ValueError):
        pregrasp.build(data, start, CALIBRATION_BYTES)
