"""강체 컵의 책상 지지·놓기·손 분리 판정. 실제 TPU/컵 변형 모델이 아니다."""
import math

import numpy as np


def validate_placement(config, gripper_unit="rad"):
    required = {"lowering_offset_m", "maximum_height_error_m", "maximum_tilt_deg",
                "withdraw_distance_m", "clearance_height_m",
                "maximum_lateral_error_m", "maximum_window_motion_m", "minimum_support_weight_ratio",
                "minimum_hand_clearance_m", f"gripper_open_tolerance_{gripper_unit}", "gravity_m_s2"}
    if not isinstance(config, dict) or required - config.keys():
        raise ValueError("placement 설정의 필수 항목 누락")
    for key in required:
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"placement.{key}: 유한한 양수 필요")
    if config["minimum_support_weight_ratio"] > 1:
        raise ValueError("지지력 비율은 1 이하여야 합니다")


def supported_window(samples, config, phase, duration_s, released=False, clear=False):
    """마지막 연속 구간의 위치·지지 접촉·정지를 모두 검사한다."""
    count = max(2, math.ceil(duration_s / config["physics_dt_s"]))
    window = samples[-count:]
    if len(window) != count or any(s["phase"] != phase for s in window):
        return False
    limits = config["placement"]
    radius = config.get("edge_support_radius_m")
    if radius is not None and (not isinstance(radius, (float, int)) or not math.isfinite(radius) or radius <= 0):
        return False
    minimum_support_n = config["cup_mass_kg"]*limits["gravity_m_s2"]*limits["minimum_support_weight_ratio"]
    positions = np.asarray([s["cup_position_m"] for s in window])
    if not np.isfinite(positions).all():
        return False
    if np.max(np.linalg.norm(positions-positions[0], axis=1)) > limits["maximum_window_motion_m"]:
        return False
    center = np.asarray(config["cup_center_m"])
    unit = config.get("gripper_unit", "rad")
    for sample in window:
        numbers = [sample["table_support_force_n"], sample["cup_tilt_deg"],
                   *sample["contact_force_n"], sample["contact_center_error_m"],
                   sample[f"gripper_actual_{unit}"], sample["arm_error_rad"]]
        if not np.isfinite(numbers).all():
            return False
        position = np.asarray(sample["cup_position_m"])
        expected_z_m = config["table_surface_z_m"]+config["cup_height_m"]/2
        if radius is not None:
            tilt = math.radians(sample["cup_tilt_deg"])
            expected_z_m = (config["table_surface_z_m"]+config["cup_height_m"]/2*math.cos(tilt)
                            +radius*math.sin(tilt))
        if (abs(position[2]-expected_z_m) > limits["maximum_height_error_m"]
                or np.linalg.norm(position[:2]-center[:2]) > limits["maximum_lateral_error_m"]
                or sample["cup_tilt_deg"] > limits["maximum_tilt_deg"]
                or sample["table_support_force_n"] < minimum_support_n
                or sample["arm_error_rad"] > config["maximum_tracking_error_rad"]):
            return False
        if released and (max(sample["contact_force_n"]) >= config["minimum_contact_force_n"]
                         or sample[f"gripper_actual_{unit}"] < config[f"gripper_open_{unit}"]-limits[f"gripper_open_tolerance_{unit}"]):
            return False
        if clear and sample["contact_center_error_m"] < limits["minimum_hand_clearance_m"]:
            return False
    return True


def evaluate_placement(samples, config, completed, lift_pass, touchdown_observed=False):
    return bool(completed and lift_pass and touchdown_observed and supported_window(
        samples, config, "PLACE_HOLD", 1.0, released=True, clear=True))


def placement_gate_failure(phase, samples, config, lift_pass, touchdown_observed):
    if phase == "LOWER" and not lift_pass:
        return "lift_not_verified_before_lowering"
    if phase == "TABLE_SETTLE" and not touchdown_observed:
        return "table_contact_not_observed_before_lower_limit"
    if phase == "OPEN" and (not touchdown_observed or not supported_window(samples, config, "TABLE_SETTLE", .5)):
        return "table_support_not_observed_before_open"
    if phase == "WITHDRAW" and (not touchdown_observed or not supported_window(
            samples, config, "RELEASE_HOLD", .5, released=True)):
        return "release_not_observed_before_withdraw"
    return None


def lowering_failure(sample, config):
    """지지 전 파지 상실을 낙하 후 내려놓기 성공으로 처리하지 않는다."""
    if sample["phase"] != "LOWER":
        return None
    support_n = config["cup_mass_kg"]*config["placement"]["gravity_m_s2"]*config["placement"]["minimum_support_weight_ratio"]
    if (sample["table_support_force_n"] < support_n
            and min(sample["contact_force_n"]) < config["minimum_contact_force_n"]):
        return "grasp_contact_lost_during_lowering"
    if (sample["contact_center_error_m"] > config["maximum_contact_center_error_m"]
            or abs(sample["midbody_height_error_m"]) > config["maximum_midbody_height_error_m"]):
        return "cup_separated_from_hand_during_lowering"
    if sample["cup_tilt_deg"] > config["maximum_cup_tilt_deg"]:
        return "cup_tilt_exceeded_during_lowering"
    return None


def touchdown_plan(plan, joint_actual_rad):
    """계획 하한까지 계속 누르지 않고 접촉 시점 자세에서 지지 확인으로 넘어간다."""
    # 접촉 시점의 관절값을 사용하며 물체 위치는 변경하지 않는다.
    joints = np.asarray(joint_actual_rad, dtype=float)
    if joints.shape != (5,) or not np.isfinite(joints).all():
        raise ValueError("접지 관절값은 유한한 5축 값이어야 합니다")
    poses = plan["poses"]
    start = next(i for i, p in enumerate(poses) if p["name"] == "TABLE_SETTLE")
    result = []
    for pose in poses[start:]:
        if pose["name"] in {"TABLE_SETTLE", "OPEN", "RELEASE_HOLD"}:
            pose = {k: v for k, v in pose.items() if k not in {"measurement", "target_m"}}
            pose = {**pose, "joint_deg": np.degrees(joints).tolist(), "source": "observed_touchdown"}
        result.append(pose)
    return {**plan, "poses": result}
