"""CPU-only bimanual pick/pour/place timeline for the contact proxy simulation."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import cup_contact_model as cup
from pour_geometry import load_experiment
from plan_body_side_grasp import _limits, base_positions, measure_stage, solve_horizontal_endpoint, validate_measurement


JOINT_MARGIN_DEG = 3.0
POUR_POSITION_LIMIT_MM = 2.0
POUR_AXIS_LIMIT_DEG = 4.0
EXPERIMENT = load_experiment(cup.ROOT / "config/simulation/bimanual_pour_experiment.json")
WORK_CUP_CENTER_M = np.array(EXPERIMENT["cup_work_center_m"])
BOTTLE_MOUTH_TARGET_M = np.array(EXPERIMENT["bottle_mouth_target_m"])
POUR_TILT_DEG = EXPERIMENT["pour_tilt_deg"]


class BimanualContactChain(cup.Chain):
    """Expose both configured contact centers even in a one-sided proxy URDF."""

    def __init__(self, path: Path, left_config, right_config):
        super().__init__(path)
        self._offsets = {
            "left": np.asarray(left_config["contact_center_tool_m"], dtype=float),
            "right": np.asarray(right_config["contact_center_tool_m"], dtype=float),
        }

    def transforms(self, positions):
        transforms = super().transforms(positions)
        for side, offset in self._offsets.items():
            contact_name = f"{side}_contact_center"
            if contact_name in transforms:
                transforms[f"{side}_tool0"] = transforms[contact_name]
            else:
                local = np.eye(4)
                local[:3, 3] = offset
                transforms[f"{side}_tool0"] = transforms[f"{side}_tool0"] @ local
        return transforms


def _pose(name, duration_s, left, right, left_gripper, right_gripper, **extra):
    return {
        "name": name,
        "duration_s": float(duration_s),
        "left_joint_deg": np.asarray(left, dtype=float).tolist(),
        "right_joint_deg": np.asarray(right, dtype=float).tolist(),
        "left_gripper_rad": float(left_gripper),
        "right_gripper_m": float(right_gripper),
        **extra,
    }


def _axis_error_deg(actual, desired):
    return math.degrees(math.acos(float(np.clip(np.dot(actual, desired), -1.0, 1.0))))


def _solve_pour_mouth(chain, mouth_target_m, tilt_deg, local_axis, seed_q, half_height_m):
    """입구 xyz·기울기·왼쪽 방향 범위를 구속하고 범위 안에서 연속성을 유도한다."""
    target = np.asarray(mouth_target_m, dtype=float)
    angle = math.radians(tilt_deg)
    azimuth = math.radians(EXPERIMENT["pour_azimuth_deg"])
    desired = np.array([math.sin(angle)*math.cos(azimuth), math.sin(angle)*math.sin(azimuth), math.cos(angle)])
    lower, upper = _limits(chain, "right")
    lower += math.radians(JOINT_MARGIN_DEG)
    upper -= math.radians(JOINT_MARGIN_DEG)

    def residual(q):
        transform = chain.transforms(base_positions("right", q))["right_tool0"]
        actual_axis = transform[:3, :3] @ local_axis
        mouth = transform[:3, 3] + half_height_m*actual_axis
        inward_violation = max(0., abs(actual_axis[0])-actual_axis[1]*math.tan(math.radians(30)))
        return np.r_[(mouth-target)*10., actual_axis[2]-desired[2],
                     inward_violation*5., (actual_axis[:2]-desired[:2])*.01, (q-seed_q)*.01]

    q = np.clip(seed_q, lower, upper)
    damping = 1e-3
    for _ in range(220):
        raw = residual(q)
        jacobian = np.empty((len(raw), len(q)))
        for index in range(len(q)):
            probe = q.copy()
            probe[index] = min(q[index]+1e-5, upper[index])
            if probe[index] == q[index]:
                probe[index] = max(q[index]-1e-5, lower[index])
            jacobian[:, index] = (residual(probe)-raw)/(probe[index]-q[index])
        delta = np.linalg.solve(jacobian.T @ jacobian+damping*np.eye(len(q)),jacobian.T @ raw)
        candidate = np.clip(q-delta,lower,upper)
        if np.sum(residual(candidate)**2) < raw @ raw:
            q = candidate
            damping = max(1e-8,damping*.4)
            if np.linalg.norm(delta) < 1e-7:
                break
        else:
            damping = min(1e5,damping*5.)
    transform = chain.transforms(base_positions("right",q))["right_tool0"]
    actual_axis = transform[:3,:3] @ local_axis
    mouth = transform[:3,3]+half_height_m*actual_axis
    position_error_mm = float(np.linalg.norm(mouth-target)*1000)
    tilt_error_deg = abs(math.degrees(math.acos(float(np.clip(actual_axis[2],-1.,1.))))-tilt_deg)
    axis_error_deg = _axis_error_deg(actual_axis, desired)
    inward = (tilt_deg == 0 or (actual_axis[1] > 0 and
              abs(actual_axis[0]) <= actual_axis[1]*math.tan(math.radians(30))))
    if position_error_mm > POUR_POSITION_LIMIT_MM or tilt_error_deg > POUR_AXIS_LIMIT_DEG or not inward:
        raise ValueError(f"병 입구/방향 IK 미통과: {position_error_mm} mm, {axis_error_deg} deg")
    return q, {"target_m":target.tolist(),"desired_bottle_axis":desired.tolist(),
               "actual_bottle_axis":actual_axis.tolist(),"position_error_mm":position_error_mm,
               "tilt_error_deg":tilt_error_deg,"axis_error_deg":axis_error_deg,
               "azimuth_is_soft_preference":True,"inward_direction_required":True}


def _prefix_pickup(poses, prefix, fixed_other, left_open, right_open):
    result = []
    for source in poses:
        if prefix == "LEFT":
            result.append(_pose(f"LEFT_{source['name']}", source["duration_s"], source["joint_deg"],
                                fixed_other, source["gripper_rad"], right_open))
        else:
            result.append(_pose(f"RIGHT_{source['name']}", source["duration_s"], fixed_other,
                                source["joint_deg"], left_open, source["gripper_m"]))
    return result


def build_plan(model, left_config, right_config):
    """Build a complete left-cup/right-bottle timeline without moving hardware."""
    chain = BimanualContactChain(Path(model), left_config, right_config)
    factory = lambda _path: chain
    left_plan = cup.make_plan(model, left_config, place=True, side="left", chain_factory=factory)
    right_plan = cup.make_plan(model, right_config, place=True, side="right", chain_factory=factory,
                               ik_seed_joint_deg=EXPERIMENT["right_pick_ik_seed_joint_deg"])
    # 시뮬레이션 초기 주차 자세도 같은 손목 분기로 둔다. 실행 중 순간이동은 하지 않는다.
    right_plan["poses"][0]["joint_deg"][-1] = EXPERIMENT["right_pick_ik_seed_joint_deg"][-1]
    for previous, pose in zip(right_plan["poses"], right_plan["poses"][1:]):
        travel = np.max(np.abs(np.radians(np.asarray(pose["joint_deg"])-previous["joint_deg"])))
        pose["duration_s"] = max(pose["duration_s"], 1.5*travel/EXPERIMENT["pour_joint_speed_rad_s"])
    left_lift_index = next(i for i, pose in enumerate(left_plan["poses"]) if pose["name"] == "LIFT_HOLD")
    right_lift_index = next(i for i, pose in enumerate(right_plan["poses"]) if pose["name"] == "LIFT_HOLD")
    left_pick = left_plan["poses"][:left_lift_index + 1]
    right_pick = right_plan["poses"][:right_lift_index + 1]
    left_lift = np.asarray(left_pick[-1]["joint_deg"], dtype=float)
    right_lift = np.asarray(right_pick[-1]["joint_deg"], dtype=float)
    left_open = left_config["gripper_open_rad"]
    left_close = left_config["gripper_close_rad"]
    right_open = right_config["gripper_open_m"]
    right_close = right_config["gripper_close_m"]

    poses = _prefix_pickup(left_pick, "LEFT", right_plan["poses"][0]["joint_deg"], left_open, right_open)
    poses.extend(_prefix_pickup(right_pick, "RIGHT", left_lift, left_close, right_open))

    left_seed = np.radians(left_lift)
    left_work_q = solve_horizontal_endpoint(chain, "left", WORK_CUP_CENTER_M, left_seed,
                                             restarts=1, iterations=240, random_seed=71)
    initial_rotation = chain.transforms(base_positions("left", left_seed))["left_tool0"][:3, :3]
    cup_axis_local = initial_rotation.T @ np.array([0., 0., 1.])
    for fraction in np.linspace(0., 1., 21):
        q = left_seed*(1-fraction)+left_work_q*fraction
        rotation = chain.transforms(base_positions("left", q))["left_tool0"][:3, :3]
        if _axis_error_deg(rotation @ cup_axis_local, np.array([0., 0., 1.])) > 2.:
            raise ValueError("컵 운반 중 상하 방향 반전 또는 기울기")
    left_measurement = measure_stage(chain, "left", left_work_q, WORK_CUP_CENTER_M,
                                     np.array([1., 0., 0.]))
    reasons = validate_measurement(left_measurement)
    if reasons:
        raise ValueError(f"컵 붓기 위치 IK 미통과: {reasons}")
    left_work = np.degrees(left_work_q)
    poses.append(_pose("POUR_MOVE_CUP", 3., left_work, right_lift, left_close, right_close,
                       measurement=left_measurement))

    right_seed = np.radians(right_lift)
    right_lift_transform = chain.transforms(base_positions("right", right_seed))["right_tool0"]
    bottle_axis_local = right_lift_transform[:3, :3].T @ np.array([0., 0., 1.])
    pour_stages = []
    bottle_half_height = float(right_config["cup_height_m"]) / 2.
    clear_mouth = np.asarray(right_config["cup_center_m"]).copy()
    clear_mouth[2] = EXPERIMENT["clear_bottle_mouth_z_m"]
    right_seed, clear_measurement = _solve_pour_mouth(chain,clear_mouth,0.,bottle_axis_local,right_seed,bottle_half_height)
    clear_right = np.degrees(right_seed)
    poses.append(_pose("POUR_CLEAR_BOTTLE",3.,left_work,clear_right,left_close,right_close,measurement=clear_measurement))
    for index, angle_deg in enumerate(np.arange(0.,POUR_TILT_DEG+1,5.)):
        mouth_target = BOTTLE_MOUTH_TARGET_M.copy()
        # X 전방 / Y 왼쪽: 병 입구는 오른쪽 대기점에서 컵으로 접근한다.
        mouth_target[1] -= EXPERIMENT["bottle_approach_right_offset_m"]*(1-angle_deg/POUR_TILT_DEG)
        schedule = np.asarray(EXPERIMENT["mouth_height_schedule"])
        mouth_target[2] = np.interp(angle_deg, schedule[:,0], schedule[:,1])
        right_seed, measurement = _solve_pour_mouth(
            chain, mouth_target, angle_deg, bottle_axis_local, right_seed, bottle_half_height)
        right_deg = np.degrees(right_seed)
        name = "POUR_MOVE" if index == 0 else ("POUR_TILT" if angle_deg == POUR_TILT_DEG
                                                else f"POUR_TILT_{int(angle_deg)}")
        # Smoothstep 최대 미분값 1.5를 반영해 관절별 최고 속도를 제한한다.
        travel = np.max(np.abs(np.radians(right_deg-np.asarray(poses[-1]["right_joint_deg"]))))
        duration = max(3. if index == 0 else .6, 1.5*travel/EXPERIMENT["pour_joint_speed_rad_s"])
        pose = _pose(name, duration, left_work, right_deg, left_close, right_close,
                     bottle_tilt_deg=angle_deg, measurement=measurement)
        poses.append(pose)
        pour_stages.append(pose)
    poses.append({**pour_stages[-1], "name": "POUR_HOLD", "duration_s": EXPERIMENT["pour_hold_s"]})
    for stage in reversed(pour_stages[:-1]):
        travel = np.max(np.abs(np.radians(np.asarray(stage["right_joint_deg"])-np.asarray(poses[-1]["right_joint_deg"]))))
        poses.append({**stage, "name": "POUR_RETURN", "duration_s":max(.6,1.5*travel/EXPERIMENT["pour_joint_speed_rad_s"])})
    poses.append(_pose("POUR_RETURN_CLEAR",3.,left_work,clear_right,left_close,right_close))
    poses.append(_pose("POUR_RETURN_LIFT", 3., left_work, right_lift, left_close, right_close))
    poses.append(_pose("POUR_RETURN_CUP", 3., left_lift, right_lift, left_close, right_close))

    for source in right_plan["poses"][right_lift_index + 1:]:
        poses.append(_pose(f"RIGHT_{source['name']}", source["duration_s"], left_lift,
                           source["joint_deg"], left_close, source["gripper_m"]))
    right_final = np.asarray(right_plan["poses"][-1]["joint_deg"], dtype=float)
    for source in left_plan["poses"][left_lift_index + 1:]:
        poses.append(_pose(f"LEFT_{source['name']}", source["duration_s"], source["joint_deg"],
                           right_final, source["gripper_rad"], right_open))
    poses.append(_pose("FINAL_HOLD", 2., left_plan["poses"][-1]["joint_deg"], right_final,
                       left_open, right_open))
    return {
        "schema": "bimanual_pour_plan_v1",
        "poses": poses,
        "task_geometry": {
            "cup_work_center_m": WORK_CUP_CENTER_M.tolist(),
            "bottle_mouth_target_m": BOTTLE_MOUTH_TARGET_M.tolist(),
            "cup_rim_to_bottle_mouth_clearance_m": float(
                BOTTLE_MOUTH_TARGET_M[2] - (WORK_CUP_CENTER_M[2] + left_config["cup_height_m"] / 2.)),
            "maximum_bottle_tilt_deg": POUR_TILT_DEG,
        },
        "hardware_accessed": False,
        "object_attachment_used": False,
        "continuous_collision_validated": False,
        "liquid_transfer_validated": False,
    }


def sample_plan(plan, elapsed_s):
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("경과 시간 오류")
    poses = plan["poses"]
    previous = poses[0]
    for pose in poses:
        if elapsed_s < pose["duration_s"]:
            fraction = elapsed_s / pose["duration_s"]
            weight = fraction * fraction * (3. - 2. * fraction)
            result = {"phase": pose["name"], "done": False}
            for key in ("left_joint_deg", "right_joint_deg"):
                result[key] = (np.asarray(previous[key]) * (1. - weight) +
                               np.asarray(pose[key]) * weight).tolist()
            for key in ("left_gripper_rad", "right_gripper_m"):
                result[key] = previous[key] * (1. - weight) + pose[key] * weight
            return result
        elapsed_s -= pose["duration_s"]
        previous = pose
    return {"phase": previous["name"], "done": True,
            **{key: previous[key] for key in ("left_joint_deg", "right_joint_deg",
                                               "left_gripper_rad", "right_gripper_m")}}
