#!/usr/bin/env python3
"""저장된 raw의 두 각도 해석을 비교한다. 실물 영점을 선택하거나 모터를 제어하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from plan_body_side_grasp import URDF_PATH, base_positions
from servo.joint_reference import JOINT_ORDER, raw_to_joint_deg, _snapshot_for_build
from ik_pick_place import digest


def compare(snapshot, calibration_bytes):
    raw = _snapshot_for_build(snapshot, calibration_bytes)
    reference = snapshot.get("joint_reference")
    explicit = raw_to_joint_deg(raw[:5], calibration_bytes, reference)
    cal = json.loads(calibration_bytes)
    # 로컬 LeRobot MotorNormMode.DEGREES의 정의를 재현한다. URDF 영점 보정이 아니다.
    midpoint = [(cal[n]["range_min"] + cal[n]["range_max"]) / 2 for n in JOINT_ORDER]
    normalized = [(v - mid) * 360 / 4095 for v, mid in zip(raw, midpoint)]
    return {
        "schema": "joint_convention_comparison_v1",
        "hardware_accessed": False, "motion_command_emitted": False,
        "physical_mapping_verified": False, "physical_execution_ready": False,
        "snapshot_sha256": digest(snapshot),
        "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
        "urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
        "rows": [{"joint": name, "raw_tick": raw[i], "range_midpoint_raw": midpoint[i],
                  "manual_zero_raw": reference["zero_raw"][i],
                  "manual_reference_deg": explicit[i], "lerobot_normalized_deg": normalized[i],
                  "manual_minus_normalized_deg": explicit[i] - normalized[i]}
                 for i, name in enumerate(JOINT_ORDER)],
        "interpretations": [
            {"name": "manual_reference", "joint_deg": explicit,
             "status": "unverified_user_alignment"},
            {"name": "lerobot_degrees", "joint_deg": normalized,
             "status": "range_normalization_not_a_verified_urdf_mapping"},
        ],
        "limitations": ["두 계산 중 하나를 자동으로 정답으로 선택하지 않는다",
                        "raw/각도 왕복과 모델 충돌 검사는 실물 영점 일치의 증거가 아니다",
                        "책상·카메라·장착 기준과 그리퍼 개구는 이 비교에서 측정하지 않는다"],
    }


def render_comparison(result, path):
    from render_joint_reference import render
    titles = ["A · 수동 영점 해석 / 미검증", "B · LeRobot 각도 해석 / 미검증"]
    views = [(title, base_positions("left", np.radians(item["joint_deg"])),
              np.array([850, -950, 600]))
             for title, item in zip(titles, result["interpretations"])]
    render(path, comparisons=views)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", type=Path)
    args = parser.parse_args()
    if args.output.exists() or (args.render and args.render.exists()):
        parser.error("기존 출력은 덮어쓰지 않습니다")
    if args.render and args.render.resolve() == args.output.resolve():
        parser.error("JSON과 렌더 경로는 달라야 합니다")
    result = compare(json.loads(args.snapshot.read_text()), args.calibration.read_bytes())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    if args.render:
        render_comparison(result, args.render)
    print(args.output)


if __name__ == "__main__":
    main()
