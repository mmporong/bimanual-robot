#!/usr/bin/env python3
"""사용자 컵 실측과 모델 가정을 분리한 무접촉 IK 끝점 후보를 생성한다.

포트·카메라·모터에 접근하지 않는다. 시작부터 끝점까지의 경로 승인을
제공하지 않으며, 그리퍼 raw 개방 명령이나 닫기 명령을 생성하지 않는다.
"""
from __future__ import annotations

import argparse
from datetime import date
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from ik_pick_place import FixedContactChain, JOINTS, digest
from plan_body_side_grasp import (URDF_PATH, base_positions, measure_stage,
                                 solve_horizontal_endpoint, validate_measurement)
from servo.execute_safe_recovery import degrees_to_raw
from servo.ik_reset_clear_step import prepare


def _json_date(value):
    if isinstance(value, date):
        return value.isoformat()
    raise TypeError(f"JSON을 지원하지 않는 값: {type(value).__name__}")


def build(measurement: dict, snapshot: dict, calibration_bytes: bytes) -> dict:
    measurement = json.loads(json.dumps(measurement, default=_json_date, allow_nan=False))
    if measurement.get("frame") != "left_arm_base_center" or measurement.get("same_table_surface") is not True:
        raise ValueError("동일 책상의 왼팔 베이스 상대 실측만 허용합니다")
    if measurement.get("physical_execution_ready") is not False:
        raise ValueError("이 도구는 미검증 실물 배치의 끝점 후보만 생성합니다")
    for key in ("cup_forward", "cup_lateral", "cup_diameter", "cup_height"):
        item = measurement[key]
        if item["unit"] != "m" or type(item["value"]) not in (int, float) or not np.isfinite(item["value"]):
            raise ValueError(f"{key}: 유한한 m 단위 값이 필요합니다")
    model = measurement["nominal_pregrasp"]
    if model["status"] != "mesh_based_candidate_not_physical_calibration":
        raise ValueError("명목 모델 가정과 실측을 분리해야 합니다")
    if any(measurement[key]["value"] <= 0 for key in ("cup_forward", "cup_diameter", "cup_height")):
        raise ValueError("컵 전방 거리와 치수는 양수여야 합니다")
    if not np.isclose(measurement["cup_diameter"]["value"], model["reference_cup_diameter_m"], rtol=0, atol=1e-9):
        raise ValueError("컵 지름 변경 시 메시 접촉 중심·후퇴 검토가 필요합니다")
    offset_m = np.asarray(model["contact_center_tool_m"], dtype=float)
    if offset_m.shape != (3,) or not np.all(np.isfinite(offset_m)):
        raise ValueError("접촉 중심은 유한한 모델 3벡터여야 합니다")
    backoff_m = float(model["selected_backoff_m"])
    if not np.isfinite(backoff_m) or backoff_m < float(model["minimum_geometric_backoff_m"]) + 0.005:
        raise ValueError("후퇴 거리는 기하 최솟값보다 5 mm 이상 커야 합니다")
    # 기존 스냅샷 검증을 재사용한다. 반환된 제한 동작 후보는 실행하지 않는다.
    prepare(snapshot, calibration_bytes)
    cal = json.loads(calibration_bytes)
    chain = FixedContactChain({"contact_center_tool_m": offset_m.tolist()})
    transforms = chain.transforms(base_positions("left", np.zeros(5)))
    base_m = transforms["left_base_link"][:3, 3]
    local_cup_m = np.array([measurement["cup_forward"]["value"],
                            measurement["cup_lateral"]["value"], measurement["cup_height"]["value"] / 2])
    # 같은 원점이라는 가정만 적용하며 physical_execution_ready를 참으로 바꾸지 않는다.
    cup_m = base_m + local_cup_m
    before_m = cup_m - np.array([backoff_m, 0.0, 0.0])
    above_m = before_m.copy()
    above_m[2] = base_m[2] + float(model["high_clearance_above_table_m"])
    if not np.all(np.isfinite(above_m)) or above_m[2] <= cup_m[2] + measurement["cup_height"]["value"] / 2:
        raise ValueError("상부 정렬 높이가 컵 입구보다 높아야 합니다")
    over_cup_m = above_m.copy()
    over_cup_m[:2] = cup_m[:2]
    seed = np.radians(snapshot["joint_deg"])
    stages = []
    for index, (phase, target_m) in enumerate((("ALIGN_ABOVE_BEHIND_CUP", above_m),
                                             ("PREGRASP_ABOVE_CUP", over_cup_m),
                                             ("MIDBODY_BEHIND_CUP_CANDIDATE", before_m))):
        # 첫 자세 이후에는 현재 해에서 이어 풀어 등가 손목 180도 분기 전환을 피한다.
        q_rad = solve_horizontal_endpoint(chain, "left", target_m, seed_q=seed,
                                         restarts=12 if index == 0 else 1)
        measured = measure_stage(chain, "left", q_rad, target_m, np.array([1.0, 0.0, 0.0]))
        q_deg = np.degrees(q_rad)
        raw = [degrees_to_raw(float(q), cal[name]["range_min"], cal[name]["range_max"])
               for name, q in zip(JOINTS, q_deg)]
        quantized_deg = [(value - (cal[name]["range_min"] + cal[name]["range_max"]) / 2) * 360 / 4095
                         for name, value in zip(JOINTS, raw)]
        quantized = measure_stage(chain, "left", np.radians(quantized_deg), target_m, np.array([1.0, 0.0, 0.0]))
        stages.append({"phase": phase, "contact_center_target_m": target_m.tolist(),
                       "joint_deg": q_deg.tolist(), "arm_raw_ticks": raw, "endpoint_check": measured,
                       "quantized_endpoint_check": quantized,
                       "endpoint_violations": validate_measurement(measured) + validate_measurement(quantized)})
        seed = q_rad
    low_candidate = stages.pop()
    return {"schema": "bench_cup_pregrasp_candidates_v1", "hardware_accessed": False,
            "motion_command_emitted": False, "physical_execution_ready": False,
            "start_to_endpoint_path_verified": False, "gripper_raw_command_available": False,
            "close_command_available": False, "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
            "endpoints_ready_for_path_review": all(not stage["endpoint_violations"] for stage in stages),
            "midbody_behind_cup_candidate": low_candidate,
            "urdf_sha256": hashlib.sha256(URDF_PATH.read_bytes()).hexdigest(),
            "snapshot_sha256": digest(snapshot), "measurement": measurement,
            "assumptions": ["left_arm_base_center와 left_base_link 원점 대응은 미검증",
                            "동일 책상면을 모델 left_base_link 높이에 잠정 대응",
                            "접촉 중심과 개방각은 메시 후보이며 실측이 아님"],
            "stages": stages}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measurement", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build(yaml.safe_load(args.measurement.read_text()), json.loads(args.snapshot.read_text()),
                   args.calibration.read_bytes())
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(encoded)
    print(args.output)


if __name__ == "__main__":
    main()
