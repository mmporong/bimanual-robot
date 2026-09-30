import copy
import hashlib
import json

import pytest

import review_bench_pregrasp_path as review
from ik_pick_place import digest
from plan_body_side_grasp import URDF_PATH
from servo.joint_reference import build_reference, raw_to_joint_deg
from test_ik_reset_clear_step import CALIBRATION_BYTES


def fixture_inputs():
    reference = build_reference(
        CALIBRATION_BYTES,
        [2004, 1952, 1518, 2858, 983, 1414],
        [0.0] * 5,
        hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
        {"status": "user_aligned_reference", "observed_at": "2026-09-29T17:50:03+09:00",
         "description": "synthetic regression fixture"},
    )
    current_raw = [2006, 1957, 2120, 2827, 981, 1414]
    snapshot = {
        "raw_ticks": current_raw,
        "torque": [0] * 6,
        "joint_deg": raw_to_joint_deg(current_raw[:5], CALIBRATION_BYTES, reference),
        "joint_reference": reference,
        "hardware_accessed": True,
        "motion_command_emitted": False,
        "calibration_sha256": hashlib.sha256(CALIBRATION_BYTES).hexdigest(),
    }
    stage_raw = [[2095, 1372, 2316, 2639, 2038], [2072, 1904, 1900, 2524, 2038]]
    phases = ["ALIGN_ABOVE_BEHIND_CUP", "PREGRASP_ABOVE_CUP"]
    stages = [{
        "phase": phase,
        "joint_deg": raw_to_joint_deg(raw, CALIBRATION_BYTES, reference),
        "arm_raw_ticks": raw,
        "endpoint_violations": [],
    } for phase, raw in zip(phases, stage_raw)]
    candidates = {
        "schema": "bench_cup_pregrasp_candidates_v1",
        "hardware_accessed": False,
        "joint_reference": reference,
        "motion_command_emitted": False,
        "physical_execution_ready": False,
        "start_to_endpoint_path_verified": False,
        "calibration_sha256": hashlib.sha256(CALIBRATION_BYTES).hexdigest(),
        "urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
        "snapshot_sha256": digest(snapshot),
        "endpoints_ready_for_path_review": True,
        "measurement": {
            "frame": "left_arm_base_center",
            "same_table_surface": True,
            "physical_execution_ready": False,
            "cup_forward": {"value": 0.35, "unit": "m"},
            "cup_lateral": {"value": 0.0, "unit": "m"},
            "cup_diameter": {"value": 0.08, "unit": "m"},
            "cup_height": {"value": 0.1, "unit": "m"},
        },
        "stages": stages,
    }
    return snapshot, candidates


def test_raw_interpolation_includes_start_and_limits_each_step():
    samples = review.sampled_raw_path([
        ("CURRENT", [0, 10, 20]),
        ("CLEARANCE_LIFT", [21, -1, 20]),
        ("ENDPOINT", [25, 4, 9]),
    ])
    assert samples[0]["raw_ticks"] == [0, 10, 20]
    assert samples[-1]["raw_ticks"] == [25, 4, 9]
    assert all(max(abs(a - b) for a, b in zip(first["raw_ticks"], second["raw_ticks"])) <= 10
               for first, second in zip(samples, samples[1:]))


def test_build_is_sampled_offline_non_executable_review():
    snapshot, candidates = fixture_inputs()
    result = review.build(snapshot, candidates, CALIBRATION_BYTES)
    assert result["schema"] == "bench_pregrasp_path_review_v1"
    assert result["sampled_only"] and result["offline_review_complete"]
    assert not result["hardware_accessed"] and not result["motion_command_emitted"]
    assert not result["executable"] and not result["physical_execution_ready"]
    assert not result["rendering_performed"]
    assert result["method"]["maximum_observed_raw_step_ticks"] <= 10
    assert result["method"]["minimum_observed_joint_limit_margin_deg"] >= 0
    assert result["method"]["start_sample_included"]
    assert result["method"]["clearance_profile"] == "retract_and_lift"
    assert len(result["gripper_assumption_reports"]) == 4
    assert result["waypoints"][1]["joint_deg"][1] == pytest.approx(
        result["waypoints"][0]["joint_deg"][1] - 30, abs=0.05)
    assert result["waypoints"][1]["joint_deg"][2] == pytest.approx(
        result["waypoints"][0]["joint_deg"][2] + 15, abs=0.05)
    for assumption in result["gripper_assumption_reports"]:
        assert assumption["candidate_connection_aggregate"]["sample_count"] > 0
        assert assumption["start_floor_report"]["reported_separately_from_candidate_connections"]
    json.dumps(result, allow_nan=False)


def test_shoulder_only_profile_holds_other_joints():
    snapshot, candidates = fixture_inputs()
    result = review.build(snapshot, candidates, CALIBRATION_BYTES, "shoulder_only")
    start, clearance = result["waypoints"][:2]
    assert clearance["joint_deg"][1] == pytest.approx(start["joint_deg"][1] - 30, abs=0.05)
    for index in (0, 2, 3, 4):
        assert clearance["raw_ticks"][index] == start["raw_ticks"][index]


def test_high_clearance_profile_uses_relative_joint_deltas():
    snapshot, candidates = fixture_inputs()
    result = review.build(snapshot, candidates, CALIBRATION_BYTES, "retract_and_raise_high")
    start, clearance = result["waypoints"][:2]
    assert clearance["joint_deg"][1] == pytest.approx(start["joint_deg"][1] - 45, abs=0.05)
    assert clearance["joint_deg"][2] == pytest.approx(start["joint_deg"][2] + 15, abs=0.05)


@pytest.mark.parametrize("changed", ["snapshot_hash", "calibration_hash", "urdf_hash", "reference", "cup_frame"])
def test_hash_or_reference_mismatch_is_rejected(changed):
    snapshot, candidates = fixture_inputs()
    candidates = copy.deepcopy(candidates)
    if changed == "snapshot_hash":
        candidates["snapshot_sha256"] = "0" * 64
    elif changed == "calibration_hash":
        candidates["calibration_sha256"] = "0" * 64
    elif changed == "urdf_hash":
        candidates["urdf_sha256"] = "0" * 64
    elif changed == "cup_frame":
        candidates["measurement"]["frame"] = "base_footprint"
    else:
        candidates["joint_reference"]["reference_raw"][0] += 1
    with pytest.raises(ValueError):
        review.build(snapshot, candidates, CALIBRATION_BYTES)
