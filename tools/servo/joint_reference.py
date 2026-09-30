#!/usr/bin/env python3
"""사용자가 정렬한 자세를 URDF 관절 영점에 묶는 오프라인 도구.

이 파일은 서보 포트를 열거나 calibration 레지스터를 쓰지 않는다. 저장된
기준은 자세 정렬의 출처만 기록하며 측정 정밀도를 보증하지 않는다.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from plan_body_side_grasp import URDF_PATH


SCHEMA = "joint_reference_v1"
TICKS_PER_TURN = 4095.0
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_NAMES = [*JOINT_ORDER, "gripper"]
SIGNS = [1, 1, 1, 1, 1]


def _validate_reference_pose(values):
    # 현재 제공하는 정렬 그림은 q=0 한 자세다. 다른 자세를 임의로 등록하지 않는다.
    if any(value != 0.0 for value in values):
        raise ValueError("현재 정렬 도구는 그림과 같은 q=0 기준 자세만 지원합니다")
    root = ET.parse(URDF_PATH).getroot()
    for name, value in zip(JOINT_ORDER, values):
        limit = root.find(f"./joint[@name='left_{name}']/limit")
        if limit is None or not float(limit.attrib["lower"]) <= math.radians(value) <= float(limit.attrib["upper"]):
            raise ValueError(f"{name}: 기준 자세가 URDF 관절 한계 밖입니다")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _validate_sha256(value, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
            char not in "0123456789abcdef" for char in value):
        raise ValueError(f"{label}은 소문자 SHA-256이어야 합니다")
    return value


def _finite_vector(value, length: int, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label}: 유한한 숫자 {length}개가 필요합니다")
    if any(type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise ValueError(f"{label}: 유한한 숫자 {length}개가 필요합니다")
    return [float(item) for item in value]


def _load_calibration(calibration_bytes: bytes) -> dict:
    if not isinstance(calibration_bytes, bytes):
        raise ValueError("calibration_bytes는 bytes여야 합니다")
    try:
        calibration = json.loads(calibration_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("calibration JSON 오류") from exc
    if not isinstance(calibration, dict) or set(calibration) != set(ALL_NAMES):
        raise ValueError("calibration 관절 구성이 다릅니다")
    ids = []
    for name in ALL_NAMES:
        item = calibration[name]
        if not isinstance(item, dict):
            raise ValueError(f"{name}: calibration 항목 오류")
        for key in ("id", "drive_mode", "homing_offset", "range_min", "range_max"):
            if type(item.get(key)) is not int:
                raise ValueError(f"{name}.{key}: 정수가 필요합니다")
        if not 1 <= item["id"] <= 253 or item["drive_mode"] not in (0, 1):
            raise ValueError(f"{name}: ID 또는 drive_mode 오류")
        if not 0 <= item["range_min"] < item["range_max"] <= 4095:
            raise ValueError(f"{name}: calibration 범위 오류")
        ids.append(item["id"])
    if len(set(ids)) != len(ids):
        raise ValueError("calibration 서보 ID가 중복됩니다")
    return calibration


def _validate_provenance(provenance) -> dict:
    if not isinstance(provenance, dict):
        raise ValueError("provenance 객체가 필요합니다")
    if provenance.get("status") != "user_aligned_reference":
        raise ValueError("provenance.status는 user_aligned_reference여야 합니다")
    observed_at, description = provenance.get("observed_at"), provenance.get("description")
    if not isinstance(observed_at, str) or not observed_at.strip():
        raise ValueError("provenance.observed_at이 필요합니다")
    try:
        datetime.fromisoformat(observed_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("provenance.observed_at은 ISO 8601 형식이어야 합니다") from exc
    if not isinstance(description, str) or not description.strip():
        raise ValueError("provenance.description이 필요합니다")
    return {
        "status": "user_aligned_reference",
        "observed_at": observed_at,
        "description": description.strip(),
    }


def build_reference(calibration_bytes, raw_ticks6, reference_joint_deg5,
                    urdf_sha256, provenance):
    """명시적으로 관측한 raw와 URDF 각도의 대응점을 만든다."""
    calibration = _load_calibration(calibration_bytes)
    raw_values = _finite_vector(raw_ticks6, 6, "raw_ticks6")
    if any(not value.is_integer() for value in raw_values):
        raise ValueError("raw_ticks6는 정수여야 합니다")
    raw_ticks = [int(value) for value in raw_values]
    reference_joint_deg = _finite_vector(reference_joint_deg5, 5, "reference_joint_deg5")
    _validate_reference_pose(reference_joint_deg)
    _validate_sha256(urdf_sha256, "urdf_sha256")
    for name, raw in zip(ALL_NAMES, raw_ticks):
        item = calibration[name]
        if not item["range_min"] <= raw <= item["range_max"]:
            raise ValueError(f"{name}: 기준 raw가 calibration 범위 밖입니다")
    zero_raw = [
        raw - degrees * TICKS_PER_TURN / 360.0
        for raw, degrees in zip(raw_ticks[:5], reference_joint_deg)
    ]
    reference = {
        "schema": SCHEMA,
        "joint_order": JOINT_ORDER.copy(),
        "servo_ids": [calibration[name]["id"] for name in JOINT_ORDER],
        "signs": SIGNS.copy(),
        "reference_raw": raw_ticks[:5],
        "reference_joint_deg": reference_joint_deg,
        "zero_raw": zero_raw,
        "calibration_sha256": _sha(calibration_bytes),
        "urdf_sha256": urdf_sha256,
        "provenance": _validate_provenance(provenance),
        "pose_alignment_precision_verified": False,
        "hardware_accessed": False,
        "motion_command_emitted": False,
    }
    return validate_reference(reference, calibration_bytes, urdf_sha256)


def validate_reference(reference, calibration_bytes, urdf_sha256):
    """기준 파일의 구조와 calibration/URDF 결합을 검증한다."""
    calibration = _load_calibration(calibration_bytes)
    if not isinstance(reference, dict) or reference.get("schema") != SCHEMA:
        raise ValueError("joint reference schema 오류")
    if reference.get("joint_order") != JOINT_ORDER:
        raise ValueError("joint_order 오류")
    if reference.get("servo_ids") != [calibration[name]["id"] for name in JOINT_ORDER]:
        raise ValueError("servo_ids와 calibration이 다릅니다")
    if reference.get("signs") != SIGNS:
        raise ValueError("signs는 명시적으로 [1, 1, 1, 1, 1]이어야 합니다")
    if reference.get("calibration_sha256") != _sha(calibration_bytes):
        raise ValueError("joint reference와 calibration 해시가 다릅니다")
    _validate_sha256(urdf_sha256, "urdf_sha256")
    if reference.get("urdf_sha256") != urdf_sha256 or urdf_sha256 != _sha(URDF_PATH.read_bytes()):
        raise ValueError("joint reference와 URDF 해시가 다릅니다")
    reference_raw_values = _finite_vector(reference.get("reference_raw"), 5, "reference_raw")
    if any(not value.is_integer() for value in reference_raw_values):
        raise ValueError("reference_raw는 정수여야 합니다")
    reference_raw = [int(value) for value in reference_raw_values]
    reference_joint_deg = _finite_vector(
        reference.get("reference_joint_deg"), 5, "reference_joint_deg")
    _validate_reference_pose(reference_joint_deg)
    zero_raw = _finite_vector(reference.get("zero_raw"), 5, "zero_raw")
    for index, (name, raw) in enumerate(zip(JOINT_ORDER, reference_raw)):
        item = calibration[name]
        if not item["range_min"] <= raw <= item["range_max"]:
            raise ValueError(f"{name}: reference_raw가 calibration 범위 밖입니다")
        expected_zero = raw - reference_joint_deg[index] * TICKS_PER_TURN / 360.0
        if not math.isclose(zero_raw[index], expected_zero, rel_tol=0, abs_tol=1e-9):
            raise ValueError(f"{name}: zero_raw 대응이 다릅니다")
    _validate_provenance(reference.get("provenance"))
    if reference.get("pose_alignment_precision_verified") is not False:
        raise ValueError("사용자 정렬은 측정 정밀도 검증으로 표시할 수 없습니다")
    if reference.get("hardware_accessed") is not False or reference.get("motion_command_emitted") is not False:
        raise ValueError("joint reference는 오프라인 산출물이어야 합니다")
    return reference


def raw_to_joint_deg(raw5, calibration_bytes, reference):
    calibration = _load_calibration(calibration_bytes)
    validate_reference(reference, calibration_bytes, _sha(URDF_PATH.read_bytes()))
    raw_values = _finite_vector(raw5, 5, "raw5")
    if any(not value.is_integer() for value in raw_values):
        raise ValueError("raw5는 정수여야 합니다")
    raw = [int(value) for value in raw_values]
    for name, value in zip(JOINT_ORDER, raw):
        item = calibration[name]
        if not item["range_min"] <= value <= item["range_max"]:
            raise ValueError(f"{name}: raw가 calibration 범위 밖입니다")
    return [
        sign * (value - zero) * 360.0 / TICKS_PER_TURN
        for value, zero, sign in zip(raw, reference["zero_raw"], reference["signs"])
    ]


def joint_deg_to_raw(q5, calibration_bytes, reference):
    calibration = _load_calibration(calibration_bytes)
    validate_reference(reference, calibration_bytes, _sha(URDF_PATH.read_bytes()))
    q_deg = _finite_vector(q5, 5, "q5")
    raw = [
        int(round(zero + degrees * TICKS_PER_TURN / (360.0 * sign)))
        for degrees, zero, sign in zip(q_deg, reference["zero_raw"], reference["signs"])
    ]
    for name, value in zip(JOINT_ORDER, raw):
        item = calibration[name]
        if not item["range_min"] <= value <= item["range_max"]:
            raise ValueError(f"{name}: 목표 raw가 calibration 범위 밖입니다")
    return raw


def joint_limits_deg(calibration_bytes, reference):
    """저장된 raw 가동 범위를 같은 정렬 기준의 각도 범위로 변환한다."""
    calibration = _load_calibration(calibration_bytes)
    lower = raw_to_joint_deg(
        [calibration[name]["range_min"] for name in JOINT_ORDER], calibration_bytes, reference)
    upper = raw_to_joint_deg(
        [calibration[name]["range_max"] for name in JOINT_ORDER], calibration_bytes, reference)
    return list(zip(lower, upper))


def _snapshot_for_build(snapshot: dict, calibration_bytes: bytes) -> list[int]:
    if not isinstance(snapshot, dict):
        raise ValueError("snapshot 객체가 필요합니다")
    if snapshot.get("hardware_accessed") is not True or snapshot.get("motion_command_emitted") is not False:
        raise ValueError("읽기 전용 실물 snapshot만 허용합니다")
    if snapshot.get("calibration_sha256") != _sha(calibration_bytes):
        raise ValueError("snapshot과 calibration 해시가 다릅니다")
    torque = snapshot.get("torque")
    if (not isinstance(torque, list) or len(torque) != 6
            or any(type(value) is not int for value in torque) or torque != [0] * 6):
        raise ValueError("snapshot의 모든 토크가 OFF여야 합니다")
    raw = snapshot.get("raw_ticks")
    if not isinstance(raw, list) or len(raw) != 6 or any(type(value) is not int for value in raw):
        raise ValueError("snapshot raw_ticks는 정수 6개여야 합니다")
    return raw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--reference-joint-deg", type=float, nargs=5, required=True)
    parser.add_argument("--description", required=True)
    parser.add_argument("--observed-at", required=True)
    parser.add_argument("--pose-confirmed", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.pose_confirmed:
        parser.error("--pose-confirmed 없이 기준 파일을 만들 수 없습니다")

    calibration_bytes = args.calibration.read_bytes()
    snapshot = json.loads(args.snapshot.read_text(encoding="utf-8"))
    raw = _snapshot_for_build(snapshot, calibration_bytes)
    reference = build_reference(
        calibration_bytes,
        raw,
        args.reference_joint_deg,
        _sha(URDF_PATH.read_bytes()),
        {"status": "user_aligned_reference", "observed_at": args.observed_at,
         "description": args.description},
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(reference, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with args.output.open("x", encoding="utf-8") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    print(args.output)


if __name__ == "__main__":
    main()
