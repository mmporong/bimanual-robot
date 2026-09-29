import copy
import hashlib
import json
import sys

import pytest

from servo import joint_reference as joint


URDF_SHA = hashlib.sha256(joint.URDF_PATH.read_bytes()).hexdigest()


def calibration_bytes(ranges=None, homing_delta=0):
    ranges = ranges or [(800, 3200), (700, 3300), (300, 2700), (600, 3000), (0, 4095), (1300, 2900)]
    data = {}
    for index, (name, limits) in enumerate(zip(joint.ALL_NAMES, ranges), start=1):
        data[name] = {"id": index, "drive_mode": 0, "homing_offset": homing_delta,
                      "range_min": limits[0], "range_max": limits[1]}
    return json.dumps(data, sort_keys=True).encode()


def provenance():
    return {"status": "user_aligned_reference", "observed_at": "2026-09-29T14:10:00+09:00",
            "description": "사용자가 URDF 기준 자세에 정렬함"}


def build(calibration=None):
    calibration = calibration or calibration_bytes()
    return joint.build_reference(calibration, [2000, 1900, 1800, 1700, 1600, 1400],
                                 [0]*5, URDF_SHA, provenance())


def test_explicit_zero_is_independent_of_range_midpoint():
    first_cal = calibration_bytes()
    shifted_cal = calibration_bytes([(600, 3200), (500, 3300), (100, 2700),
                                     (400, 3000), (0, 4095), (1200, 3000)])
    first = build(first_cal)
    shifted = build(shifted_cal)
    assert shifted["zero_raw"] == pytest.approx(first["zero_raw"])
    assert joint.raw_to_joint_deg(first["reference_raw"], first_cal, first) == pytest.approx(
        [0]*5)
    assert joint.raw_to_joint_deg(shifted["reference_raw"], shifted_cal, shifted) == pytest.approx(
        [0]*5)


def test_calibration_change_invalidates_reference():
    original = calibration_bytes()
    reference = build(original)
    changed_encoder_basis = calibration_bytes(homing_delta=1)
    with pytest.raises(ValueError, match="calibration 해시"):
        joint.validate_reference(reference, changed_encoder_basis, URDF_SHA)
    with pytest.raises(ValueError, match="calibration 해시"):
        joint.raw_to_joint_deg(reference["reference_raw"], changed_encoder_basis, reference)


def test_joint_raw_roundtrip_with_quantization():
    calibration = calibration_bytes()
    reference = build(calibration)
    requested = [12.3, -18.7, 27.1, -35.4, 44.8]
    raw = joint.joint_deg_to_raw(requested, calibration, reference)
    restored = joint.raw_to_joint_deg(raw, calibration, reference)
    assert restored == pytest.approx(requested, abs=360 / 4095 / 2)


def test_cli_requires_pose_confirmation_and_valid_snapshot(tmp_path, monkeypatch):
    calibration = calibration_bytes()
    cal_path, snapshot_path, output = (tmp_path / "cal.json", tmp_path / "snapshot.json",
                                       tmp_path / "reference.json")
    cal_path.write_bytes(calibration)
    snapshot_path.write_text(json.dumps({
        "raw_ticks": [2000, 1900, 1800, 1700, 1600, 1400], "torque": [0] * 6,
        "hardware_accessed": True, "motion_command_emitted": False,
        "calibration_sha256": hashlib.sha256(calibration).hexdigest(),
    }))
    argv = ["joint_reference", "--snapshot", str(snapshot_path), "--calibration", str(cal_path),
            "--reference-joint-deg", "0", "0", "0", "0", "0",
            "--description", "aligned", "--observed-at", "2026-09-29T14:10:00+09:00",
            "--output", str(output)]
    monkeypatch.setattr(sys, "argv", argv)
    with pytest.raises(SystemExit):
        joint.main()
    assert not output.exists()
    monkeypatch.setattr(sys, "argv", [*argv, "--pose-confirmed"])
    joint.main()
    assert json.loads(output.read_text())["schema"] == "joint_reference_v1"


@pytest.mark.parametrize("angles", [[200]*5, [0, 0, 1, 0, 0]])
def test_reference_must_match_rendered_zero_pose(angles):
    with pytest.raises(ValueError, match="q=0 기준"):
        joint.build_reference(calibration_bytes(), [2000, 1900, 1800, 1700, 1600, 1400],
                              angles, URDF_SHA, provenance())
    reference = build()
    reference["reference_joint_deg"] = angles
    with pytest.raises(ValueError, match="q=0 기준"):
        joint.validate_reference(reference, calibration_bytes(), URDF_SHA)


@pytest.mark.parametrize("field,value,match", [
    ("schema", "bad", "schema"),
    ("signs", [1, 1, -1, 1, 1], "signs"),
    ("zero_raw", [1, 2, 3, 4, float("nan")], "유한한"),
    ("urdf_sha256", "b" * 64, "URDF 해시"),
    ("pose_alignment_precision_verified", True, "측정 정밀도"),
])
def test_bad_reference_fields_are_rejected(field, value, match):
    calibration = calibration_bytes()
    reference = build(calibration)
    reference[field] = value
    with pytest.raises(ValueError, match=match):
        joint.validate_reference(reference, calibration, URDF_SHA)


def test_bad_types_ranges_hashes_and_provenance_are_rejected():
    calibration = calibration_bytes()
    with pytest.raises(ValueError, match="raw_ticks6는 정수"):
        joint.build_reference(calibration, [2000.5, 1900, 1800, 1700, 1600, 1400],
                              [0] * 5, URDF_SHA, provenance())
    with pytest.raises(ValueError, match="기준 raw"):
        joint.build_reference(calibration, [100, 1900, 1800, 1700, 1600, 1400],
                              [0] * 5, URDF_SHA, provenance())
    with pytest.raises(ValueError, match="SHA-256"):
        joint.build_reference(calibration, [2000, 1900, 1800, 1700, 1600, 1400],
                              [0] * 5, "not-a-hash", provenance())
    bad_provenance = copy.deepcopy(provenance())
    bad_provenance["status"] = "estimated"
    with pytest.raises(ValueError, match="user_aligned_reference"):
        joint.build_reference(calibration, [2000, 1900, 1800, 1700, 1600, 1400],
                              [0] * 5, URDF_SHA, bad_provenance)
    reference = build(calibration)
    with pytest.raises(ValueError, match="목표 raw"):
        joint.joint_deg_to_raw([500, 0, 0, 0, 0], calibration, reference)
