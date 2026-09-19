"""가정 강체 컵의 제한된 후퇴·장면 좌표 재관측. RGB/실물 복구가 아니다."""
from __future__ import annotations

import math
import numpy as np


def validate_recovery(config):
    required = {"max_retries", "reverse_sample_interval_s", "reverse_time_scale",
                "observation_duration_s", "maximum_observation_motion_m", "maximum_recovery_displacement_m",
                "maximum_recovery_force_n", "maximum_observation_tilt_deg", "maximum_observation_height_error_m"}
    if not isinstance(config, dict) or required - config.keys():
        raise ValueError("recovery 설정의 필수 항목 누락")
    if type(config["max_retries"]) is not int or not 0 <= config["max_retries"] <= 3:
        raise ValueError("max_retries는 0~3 정수")
    for key in ("reverse_sample_interval_s", "reverse_time_scale", "observation_duration_s",
                "maximum_observation_motion_m", "maximum_recovery_displacement_m",
                "maximum_recovery_force_n", "maximum_observation_tilt_deg",
                "maximum_observation_height_error_m"):
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"recovery.{key}: 유한한 양수 필요")
    if config["reverse_time_scale"] < 1:
        raise ValueError("후퇴 속도는 원래 접근보다 빠를 수 없습니다")


def retreat_plan(samples, config):
    """실행한 열린 손의 관절 이력을 역순 추종한다. 전역 충돌 보장은 아니다."""
    anchors = [i for i, s in enumerate(samples) if s["phase"] == "PREGRASP_ABOVE"]
    if not anchors or samples[-1]["phase"] not in {"ALIGN_MIDDLE", "APPROACH"}:
        raise ValueError("검증할 후퇴 이력 없음")
    history = samples[anchors[-1]:]
    stride = max(1, round(config["recovery"]["reverse_sample_interval_s"] / config["physics_dt_s"]))
    indices = list(range(len(history)-1, -1, -stride))
    if indices[-1] != 0:
        indices.append(0)
    poses = []
    for index, previous in zip(indices, [indices[0], *indices[:-1]]):
        duration_s = max(config["physics_dt_s"],
                         (previous-index)*config["physics_dt_s"]*config["recovery"]["reverse_time_scale"])
        poses.append({"name": "RETREAT", "joint_deg": np.degrees(history[index]["left_joint_actual_rad"]).tolist(),
                      "duration_s": duration_s, "gripper_rad": config["gripper_open_rad"]})
    poses.append({**poses[-1], "name": "REOBSERVE",
                  "duration_s": config["recovery"]["observation_duration_s"]})
    return {"poses": poses}


def recovery_failure(sample, original_center_m, config):
    limits = config["recovery"]
    if max(sample["contact_force_n"]) > limits["maximum_recovery_force_n"]:
        return "recovery_contact_force_exceeded"
    if np.linalg.norm(np.asarray(sample["cup_position_m"])[:2]-original_center_m[:2]) > limits["maximum_recovery_displacement_m"]:
        return "recovery_cup_displacement_exceeded"
    if sample["cup_tilt_deg"] > config["maximum_cup_tilt_deg"]:
        return "recovery_cup_tilt_exceeded"
    return None


def observe_stationary_cup(samples, config):
    """후퇴 완료 후 연속 물리 샘플을 관측한다. 정답 좌표임을 호출부에 기록한다."""
    count = max(2, math.ceil(config["recovery"]["observation_duration_s"] / config["physics_dt_s"]))
    window = samples[-count:]
    if len(window) != count or any(s["phase"] != "REOBSERVE" for s in window):
        raise ValueError("observation_window_incomplete")
    positions = np.asarray([s["cup_position_m"] for s in window])
    if (not np.isfinite(positions).all()
            or not np.isfinite([s["contact_force_n"] for s in window]).all()
            or not np.isfinite([s["cup_tilt_deg"] for s in window]).all()):
        raise ValueError("observation_nonfinite")
    if max(max(s["contact_force_n"]) for s in window) >= config["minimum_contact_force_n"]:
        raise ValueError("observation_contact_not_released")
    if np.max(np.linalg.norm(positions-positions[0], axis=1)) > config["recovery"]["maximum_observation_motion_m"]:
        raise ValueError("observation_cup_not_stationary")
    if max(s["cup_tilt_deg"] for s in window) > config["recovery"]["maximum_observation_tilt_deg"]:
        raise ValueError("observation_cup_not_upright")
    center = positions.mean(axis=0)
    if abs(center[2]-config["cup_center_m"][2]) > config["recovery"]["maximum_observation_height_error_m"]:
        raise ValueError("observation_cup_not_on_table")
    return center.tolist()
