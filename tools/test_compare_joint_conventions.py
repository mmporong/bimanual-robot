import copy
import hashlib
import json

import pytest

from compare_joint_conventions import compare
from servo.joint_reference import build_reference, URDF_PATH
from test_joint_reference import calibration_bytes, provenance


def inputs():
    calibration = calibration_bytes()
    reference = build_reference(calibration, [2000, 1900, 1800, 2800, 980, 1400],
                                [0]*5, hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(), provenance())
    snapshot = {"raw_ticks": [2000, 1900, 1800, 2800, 980, 1400], "torque": [0]*6,
                "hardware_accessed": True, "motion_command_emitted": False,
                "joint_reference": reference, "calibration_sha256": hashlib.sha256(calibration).hexdigest()}
    return snapshot, calibration


def test_comparison_preserves_both_interpretations_without_claiming_a_valid_mapping():
    snapshot, calibration = inputs()
    original = copy.deepcopy(snapshot)
    result = compare(snapshot, calibration)
    assert snapshot == original
    assert result["interpretations"][0]["joint_deg"] == [0]*5
    cal = json.loads(calibration)
    for row in result["rows"]:
        c = cal[row["joint"]]
        expected = (row["raw_tick"] - (c["range_min"] + c["range_max"]) / 2) * 360 / 4095
        assert row["lerobot_normalized_deg"] == pytest.approx(expected)
        assert row["manual_minus_normalized_deg"] == pytest.approx(-expected)
    assert not result["physical_execution_ready"] and not result["physical_mapping_verified"]
    assert not result["motion_command_emitted"] and not result["hardware_accessed"]


def test_wrong_calibration_is_not_silently_used_for_comparison():
    snapshot, calibration = inputs()
    with pytest.raises(ValueError, match="해시"):
        compare(snapshot, calibration + b"\n")
