"""CPU-only tray deposit and regrasp endpoints for the water-service mission."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import (
    _limits,
    base_positions,
    measure_stage,
    solve_horizontal_endpoint,
    validate_measurement,
)


TRAY_SURFACE_Z_M = 0.72
CUP_HEIGHT_M = 0.12
DEFAULT_TRAY_TARGET_XY_M = (0.0, 0.0)
EXACT_CENTER_XY_M = (0.0, 0.0)
MAX_UPRIGHT_TILT_DEG = 2.0
MAX_TRANSFER_TILT_DEG = 15.0
MAX_CENTER_REGION_RADIUS_M = 0.020
LOWER_SOLVER_TARGET_Z_M = 0.788
LOWER_BOTTOM_MIN_M = 0.718
LOWER_BOTTOM_MAX_M = 0.721
UPPER_BOTTOM_TARGET_M = 0.740
MIN_BOTTOM_TOLERANCE_M = 0.001


def _source_parts(source: dict[str, Any]):
    plan = source.get("plan", source)
    poses = plan.get("poses")
    left = source.get("left_config")
    right = source.get("right_config")
    if not isinstance(poses, list) or not poses or left is None or right is None:
        raise ValueError("source에는 plan.poses, left_config, right_config가 필요합니다")
    return plan, poses, left, right


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


def _named_pose(poses, name):
    try:
        return next(pose for pose in poses if pose.get("name") == name)
    except StopIteration as exc:
        raise ValueError(f"source plan pose 누락: {name}") from exc


def _upright_measurement(chain, q, target, cup_axis_local):
    transform = chain.transforms(base_positions("left", q))["left_tool0"]
    cup_up = transform[:3, :3] @ cup_axis_local
    tilt = math.degrees(math.acos(float(np.clip(cup_up[2], -1.0, 1.0))))
    heading = transform[:3, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    measured = measure_stage(chain, "left", q, target, heading)
    measured["cup_up_axis"] = cup_up.tolist()
    measured["cup_upright_tilt_deg"] = tilt
    return measured




def _cup_pose_metrics(chain, q, cup_axis_local, cup_height_m, cup_bottom_radius_m):
    transform = chain.transforms(base_positions("left", q))["left_tool0"]
    cup_up = transform[:3, :3] @ cup_axis_local
    tilt = math.degrees(math.acos(float(np.clip(cup_up[2], -1.0, 1.0))))
    radial = math.sqrt(max(0.0, 1.0 - float(cup_up[2]) ** 2))
    bottom = float(
        transform[2, 3] - cup_height_m / 2.0 * cup_up[2] - cup_bottom_radius_m * radial
    )
    return transform[:3, 3].copy(), cup_up, tilt, bottom


def _solve_center_region_soft_endpoint(
    chain, goal_xy, seed, cup_axis_local, cup_height_m, cup_bottom_radius_m,
):
    """Search the source wrist-roll branch for a <=15 degree center-region contact pose."""
    target = np.array([goal_xy[0], goal_xy[1], LOWER_SOLVER_TARGET_Z_M])
    lower, upper = _limits(chain, "left")
    lower += math.radians(3.0)
    upper -= math.radians(3.0)
    lower[4] = max(lower[4], 0.0)
    rng = np.random.default_rng(1506)
    starts = [np.clip(np.asarray(seed, dtype=float), lower, upper)]
    starts.extend(rng.uniform(lower, upper) for _ in range(15))

    def residual(q):
        position, cup_up, _, _ = _cup_pose_metrics(
            chain, q, cup_axis_local, cup_height_m, cup_bottom_radius_m,
        )
        return np.r_[(target - position) * 10.0, cup_up[:2] * 0.08]

    candidates = []
    for start in starts:
        q = start.copy()
        damping = 1e-3
        for _ in range(260):
            raw = residual(q)
            jacobian = np.empty((5, 5))
            for index in range(5):
                probe = q.copy()
                probe[index] = min(q[index] + 1e-5, upper[index])
                if probe[index] == q[index]:
                    probe[index] = max(q[index] - 1e-5, lower[index])
                jacobian[:, index] = (residual(probe) - raw) / (probe[index] - q[index])
            delta = np.linalg.solve(
                jacobian.T @ jacobian + damping * np.eye(5), jacobian.T @ raw,
            )
            candidate = np.clip(q - delta, lower, upper)
            if residual(candidate) @ residual(candidate) < raw @ raw:
                q = candidate
                damping = max(1e-8, damping * 0.4)
            else:
                damping = min(1e5, damping * 5.0)
        position, cup_up, tilt, bottom = _cup_pose_metrics(
            chain, q, cup_axis_local, cup_height_m, cup_bottom_radius_m,
        )
        center_error = float(np.linalg.norm(position[:2] - goal_xy))
        if (
            q[4] >= 0.0
            and cup_up[2] > 0.0
            and center_error <= MAX_CENTER_REGION_RADIUS_M
            and tilt <= MAX_TRANSFER_TILT_DEG
            and LOWER_BOTTOM_MIN_M <= bottom <= LOWER_BOTTOM_MAX_M
        ):
            candidates.append((center_error, abs(bottom - TRAY_SURFACE_Z_M), tilt, q.copy()))
    if not candidates:
        raise ValueError("source wrist-roll branch의 중앙 20mm·15도 접촉 해가 없습니다")
    return min(candidates, key=lambda item: item[:3])[3]


def _upper_on_same_joint_path(
    chain, source_q, lower_q, cup_axis_local, cup_height_m, cup_bottom_radius_m,
):
    candidates = []
    for fraction in np.linspace(0.0, 1.0, 1001):
        q = source_q * (1.0 - fraction) + lower_q * fraction
        _, _, tilt, bottom = _cup_pose_metrics(
            chain, q, cup_axis_local, cup_height_m, cup_bottom_radius_m,
        )
        if tilt <= MAX_TRANSFER_TILT_DEG:
            candidates.append((abs(bottom - UPPER_BOTTOM_TARGET_M), fraction, q.copy()))
    if not candidates:
        raise ValueError("같은 관절 분기에서 15도 이내 접근 자세가 없습니다")
    _, fraction, q = min(candidates, key=lambda item: item[0])
    return q, fraction


def _exact_center_probe(chain, seed, cup_axis_local):
    target = np.array([*EXACT_CENTER_XY_M, TRAY_SURFACE_Z_M + CUP_HEIGHT_M / 2.0])
    q = solve_horizontal_endpoint(
        chain, "left", target, seed, restarts=32, iterations=500, random_seed=991,
    )
    measured = _upright_measurement(chain, q, target, cup_axis_local)
    reasons = validate_measurement(measured)
    if measured["cup_upright_tilt_deg"] > MAX_UPRIGHT_TILT_DEG:
        reasons.append(
            f"cup_upright_tilt_deg={measured['cup_upright_tilt_deg']:.4f} > {MAX_UPRIGHT_TILT_DEG:.4f}"
        )
    return {
        "target_m": target.tolist(),
        "accepted": not reasons,
        "measurement": measured,
        "rejection_reasons": reasons,
        "multistart_count": 32,
    }


def audit_carry_path(
    chain,
    source_left_joint_deg,
    deposit_plan,
    regrasp_plan,
    cup_axis_local,
    *,
    cup_height_m=CUP_HEIGHT_M,
    cup_bottom_radius_m=0.028,
    support_surface_z_m=TRAY_SURFACE_Z_M,
    maximum_cup_tilt_deg=MAX_TRANSFER_TILT_DEG,
    samples_per_segment=101,
):
    """Audit closed-gripper joint interpolation; endpoint IK alone is insufficient."""
    if isinstance(samples_per_segment, bool) or not isinstance(samples_per_segment, int) or samples_per_segment < 2:
        raise ValueError("samples_per_segment는 2 이상의 정수여야 합니다")
    source_q = np.radians(np.asarray(source_left_joint_deg, dtype=float))
    cup_axis_local = np.asarray(cup_axis_local, dtype=float)
    if source_q.shape != (5,) or cup_axis_local.shape != (3,) or not np.isfinite(
        np.r_[source_q, cup_axis_local]
    ).all():
        raise ValueError("carry audit 입력 자세가 유효하지 않습니다")

    groups = (
        ("deposit_closed_holding", deposit_plan["poses"], {
            "TRAY_START", "TRAY_MOVE_ABOVE", "TRAY_LOWER", "TRAY_TABLE_SETTLE",
        }),
        ("regrasp_closed_holding", regrasp_plan["poses"], {
            "TRAY_CONTACT_HOLD", "TRAY_RELIFT", "TRAY_RETURN_SERVICE", "TRAY_LIFT_HOLD",
        }),
    )
    result = {"sampling": {"samples_per_segment": samples_per_segment}, "groups": {}}
    previous = source_q
    all_segments = []
    for group_name, poses, audited_names in groups:
        segments = []
        for pose in poses:
            target = np.radians(np.asarray(pose["left_joint_deg"], dtype=float))
            if pose["name"] in audited_names:
                maximum_tilt = 0.0
                minimum_bottom = float("inf")
                for fraction in np.linspace(0.0, 1.0, samples_per_segment):
                    q = previous * (1.0 - fraction) + target * fraction
                    transform = chain.transforms(base_positions("left", q))["left_tool0"]
                    cup_up = transform[:3, :3] @ cup_axis_local
                    tilt = math.degrees(math.acos(float(np.clip(cup_up[2], -1.0, 1.0))))
                    radial = math.sqrt(max(0.0, 1.0 - float(cup_up[2]) ** 2))
                    bottom = float(
                        transform[2, 3]
                        - cup_height_m / 2.0 * cup_up[2]
                        - cup_bottom_radius_m * radial
                    )
                    maximum_tilt = max(maximum_tilt, tilt)
                    minimum_bottom = min(minimum_bottom, bottom)
                segment = {
                    "name": pose["name"],
                    "maximum_cup_tilt_deg": maximum_tilt,
                    "minimum_cup_bottom_z_m": minimum_bottom,
                    "upright_accepted": maximum_tilt <= maximum_cup_tilt_deg,
                    "support_clearance_accepted": minimum_bottom >= (
                        support_surface_z_m - MIN_BOTTOM_TOLERANCE_M
                    ),
                }
                segment["accepted"] = (
                    segment["upright_accepted"] and segment["support_clearance_accepted"]
                )
                segments.append(segment)
                all_segments.append(segment)
            previous = target
        result["groups"][group_name] = {
            "segments": segments,
            "accepted": bool(segments) and all(segment["accepted"] for segment in segments),
        }
    result["maximum_cup_tilt_deg"] = max(
        segment["maximum_cup_tilt_deg"] for segment in all_segments
    )
    result["minimum_cup_bottom_z_m"] = min(
        segment["minimum_cup_bottom_z_m"] for segment in all_segments
    )
    result["accepted"] = all(group["accepted"] for group in result["groups"].values())
    result["acceptance"] = {
        "maximum_cup_tilt_deg": maximum_cup_tilt_deg,
        "minimum_cup_bottom_z_m": support_surface_z_m - MIN_BOTTOM_TOLERANCE_M,
    }
    return result


def build_tray_transfer(
    urdf_path: str | Path,
    source: dict[str, Any],
    tray_target_xy_m=DEFAULT_TRAY_TARGET_XY_M,
) -> dict[str, Any]:
    """Return deposit/regrasp plans; no simulator, ROS, camera, or hardware access."""
    _, poses, left_config, right_config = _source_parts(source)
    xy = np.asarray(tray_target_xy_m, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError("tray_target_xy_m은 유한한 2벡터여야 합니다")
    cup_radius = max(float(radius) for _, radius in source["experiment"]["cup_radius_profile_m"])
    # 실제 450x340 상판의 충돌 박스 합집합은 x=+-0.170 m, y=+-0.225 m이다.
    if abs(xy[0]) + cup_radius > 0.170 or abs(xy[1]) + cup_radius > 0.225:
        raise ValueError("컵 바닥이 450x340 상판 안전영역을 벗어납니다")

    chain = BimanualContactChain(Path(urdf_path), left_config, right_config)
    held = _named_pose(poses, "POUR_RETURN_CUP")
    parked = _named_pose(poses, "RIGHT_PLACE_HOLD")
    source_q = np.radians(held["left_joint_deg"])
    source_transform = chain.transforms(base_positions("left", source_q))["left_tool0"]
    cup_axis_local = source_transform[:3, :3].T @ np.array([0.0, 0.0, 1.0])

    cup_bottom_radius = float(source["experiment"]["cup_radius_profile_m"][0][1])
    center_goal = np.array([xy[0], xy[1], TRAY_SURFACE_Z_M + CUP_HEIGHT_M / 2.0])
    lower_q = _solve_center_region_soft_endpoint(
        chain, xy, source_q, cup_axis_local, CUP_HEIGHT_M, cup_bottom_radius,
    )
    upper_q, upper_fraction = _upper_on_same_joint_path(
        chain, source_q, lower_q, cup_axis_local, CUP_HEIGHT_M, cup_bottom_radius,
    )
    center, lower_up, lower_tilt, lower_bottom = _cup_pose_metrics(
        chain, lower_q, cup_axis_local, CUP_HEIGHT_M, cup_bottom_radius,
    )
    above, upper_up, upper_tilt, upper_bottom = _cup_pose_metrics(
        chain, upper_q, cup_axis_local, CUP_HEIGHT_M, cup_bottom_radius,
    )
    lower_measurement = {
        "position_m": center.tolist(), "cup_up_axis": lower_up.tolist(),
        "cup_upright_tilt_deg": lower_tilt, "cup_bottom_z_m": lower_bottom,
        "center_goal_xy_error_m": float(np.linalg.norm(center[:2] - xy)),
    }
    upper_measurement = {
        "position_m": above.tolist(), "cup_up_axis": upper_up.tolist(),
        "cup_upright_tilt_deg": upper_tilt, "cup_bottom_z_m": upper_bottom,
        "joint_path_fraction": upper_fraction,
    }
    # Release 뒤에는 같은 관절 분기를 역으로 따라가 패드가 컵에서 멀어진다.
    withdraw_q, withdraw_measurement = upper_q, upper_measurement
    withdraw = above.copy()
    clear_q = source_q.copy()
    clear, clear_up, clear_tilt, clear_bottom = _cup_pose_metrics(
        chain, clear_q, cup_axis_local, CUP_HEIGHT_M, cup_bottom_radius,
    )
    clear_measurement = {
        "position_m": clear.tolist(), "cup_up_axis": clear_up.tolist(),
        "cup_upright_tilt_deg": clear_tilt, "cup_bottom_z_m": clear_bottom,
    }

    left_open = left_config["gripper_open_rad"]
    left_close = left_config["gripper_close_rad"]
    right_open = right_config["gripper_open_m"]
    right_park = parked["right_joint_deg"]
    lower_deg = np.degrees(lower_q)
    upper_deg = np.degrees(upper_q)
    withdraw_deg = np.degrees(withdraw_q)
    clear_deg = np.degrees(clear_q)
    source_deg = np.asarray(held["left_joint_deg"], dtype=float)

    deposit = [
        _pose("TRAY_START", 0.5, source_deg, right_park, left_close, right_open),
        _pose("TRAY_MOVE_ABOVE", 3.0, upper_deg, right_park, left_close, right_open,
              target_m=above.tolist(), measurement=upper_measurement),
        _pose("TRAY_LOWER", 2.0, lower_deg, right_park, left_close, right_open,
              target_m=center.tolist(), measurement=lower_measurement),
        _pose("TRAY_TABLE_SETTLE", 1.0, lower_deg, right_park, left_close, right_open),
        _pose("TRAY_OPEN", 2.0, lower_deg, right_park, left_open, right_open),
        _pose("TRAY_RELEASE_HOLD", 1.0, lower_deg, right_park, left_open, right_open),
        _pose("TRAY_WITHDRAW", 2.0, withdraw_deg, right_park, left_open, right_open,
              target_m=withdraw.tolist(), measurement=withdraw_measurement),
        _pose("TRAY_CLEAR_ABOVE", 2.0, clear_deg, right_park, left_open, right_open,
              target_m=clear.tolist(), measurement=clear_measurement),
        _pose("TRAY_PLACE_HOLD", 2.0, clear_deg, right_park, left_open, right_open),
    ]
    regrasp = [
        _pose("TRAY_REGRASP_START", 0.5, clear_deg, right_park, left_open, right_open),
        _pose("TRAY_PREGRASP", 2.0, withdraw_deg, right_park, left_open, right_open),
        _pose("TRAY_REAPPROACH", 2.0, lower_deg, right_park, left_open, right_open),
        _pose("TRAY_CLOSE", 2.0, lower_deg, right_park, left_close, right_open),
        _pose("TRAY_CONTACT_HOLD", 1.0, lower_deg, right_park, left_close, right_open),
        _pose("TRAY_RELIFT", 2.0, upper_deg, right_park, left_close, right_open),
        _pose("TRAY_RETURN_SERVICE", 3.0, source_deg, right_park, left_close, right_open),
        _pose("TRAY_LIFT_HOLD", 2.0, source_deg, right_park, left_close, right_open),
    ]

    right_q = np.radians(right_park)
    right_tcp = chain.transforms(base_positions("right", right_q))["right_tool0"][:3, 3]
    tcp_distances = []
    for q in (upper_q, lower_q, withdraw_q, clear_q):
        left_tcp = chain.transforms(base_positions("left", q))["left_tool0"][:3, 3]
        tcp_distances.append(float(np.linalg.norm(left_tcp - right_tcp)))
    deposit_plan = {"poses": deposit}
    regrasp_plan = {"poses": regrasp}
    path_audit = audit_carry_path(
        chain, held["left_joint_deg"], deposit_plan, regrasp_plan, cup_axis_local,
        cup_height_m=CUP_HEIGHT_M,
        cup_bottom_radius_m=cup_bottom_radius,
        support_surface_z_m=TRAY_SURFACE_Z_M,
    )
    return {
        "schema": "tray_transfer_plan_v1",
        "deposit_plan": deposit_plan,
        "regrasp_plan": regrasp_plan,
        "tray_center_m": center.tolist(),
        "tray_center_goal_m": center_goal.tolist(),
        "edge_supported_release": True,
        "tray_surface_z_m": TRAY_SURFACE_Z_M,
        "tray_target_xy_m": xy.tolist(),
        "exact_center_probe": _exact_center_probe(chain, source_q, cup_axis_local),
        "minimum_tcp_separation_m": min(tcp_distances),
        "right_arm_parked": True,
        "endpoint_ik_validated": True,
        "endpoint_ik_contract": "source_roll_branch_center_region_20mm_tilt_15deg_bottom_edge_contact",
        "path_audit": path_audit,
        "executable": path_audit["accepted"],
        "execution_blockers": [] if path_audit["accepted"] else [
            "closed_holding_joint_interpolation_failed_cup_upright_or_support_clearance",
        ],
        "continuous_collision_validated": False,
        "continuous_upright_validated": path_audit["accepted"],
        "physics_validated": False,
        "hardware_accessed": False,
    }
