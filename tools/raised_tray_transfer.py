"""Experimental raised-center transfer, with geometric clearance audits."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions, _limits, solve_horizontal_endpoint
from tray_transfer_plan import _pose, _named_pose, _cup_pose_metrics
from search_tray_mounts import _solve_stage


def build_raised_transfer(model, source):
    spec = source["raised_tray"]
    height = float(spec["height_m"])
    if not .04 <= height <= .20:
        raise ValueError("raised tray height outside experimental range")
    chain = BimanualContactChain(Path(model), source["left_config"], source["right_config"])
    held = _named_pose(source["plan"]["poses"], "POUR_RETURN_CUP")
    parked = _named_pose(source["plan"]["poses"], "RIGHT_PLACE_HOLD")
    start = np.asarray(held["left_joint_deg"])
    upper = np.asarray(spec["above_joint_deg"], dtype=float)
    lower = np.asarray(spec["contact_joint_deg"], dtype=float)
    limit_low, limit_high = _limits(chain, "left")
    for q in (start, upper, lower):
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError("invalid raised tray joint vector")
        angles = np.radians(q)
        if np.min(np.minimum(angles-limit_low, limit_high-angles)) < math.radians(3.-1e-6):
            raise ValueError("raised tray joint margin below three degrees")
    transform = chain.transforms(base_positions("left", np.radians(start)))["left_tool0"]
    axis = transform[:3, :3].T @ np.array([0., 0., 1.])
    surface = .72+height
    center, _, tilt, bottom = _cup_pose_metrics(chain, np.radians(lower), axis, .12, .028)
    if np.linalg.norm(center[:2]) > .02 or abs(bottom-surface) > .002 or tilt > 2.:
        raise ValueError("raised contact endpoint is not central/upright/on support")
    left, right = source["left_config"], source["right_config"]
    contact_deg = lower.copy()
    contact_transform = chain.transforms(base_positions("left", np.radians(lower)))["left_tool0"]
    heading = contact_transform[:3, 2].copy()
    heading[2] = 0.
    heading /= np.linalg.norm(heading)
    backoff_m = float(source["experiment"]["cup_pregrasp_backoff_m"])
    back_target = center-heading*backoff_m
    back = _solve_stage(chain, back_target, np.radians(lower), axis, center_region=False,
                        restarts=8, iterations=320, random_seed=721)
    if not back["accepted"]:
        raise ValueError("same-height rim-clear backoff is not reachable")
    back_above_target = back_target+np.array([0., 0., backoff_m])
    back_above = _solve_stage(chain, back_above_target, back["q"], axis, center_region=False,
                              restarts=8, iterations=320, random_seed=722)
    if not back_above["accepted"]:
        raise ValueError("rim-clear pregrasp above is not reachable")
    back_deg, back_above_deg = np.degrees(back["q"]), np.degrees(back_above["q"])
    front_clear = _solve_stage(chain, np.array([.20, .17, 1.08]), back_above["q"], axis,
                               center_region=False, restarts=6, iterations=240, random_seed=723)
    if not front_clear["accepted"]:
        raise ValueError("front clear waypoint is not reachable")
    front_clear_deg = np.degrees(front_clear["q"])
    if (abs(left["cup_height_m"]-.12) > 1e-9
            or abs(left["table_surface_z_m"]-.72) > 1e-9
            or abs(source["experiment"]["cup_radius_profile_m"][0][1]-.028) > 1e-9):
        raise ValueError("raised tray geometry requires the validated 120mm flared cup")
    overtravel = float(left["placement"]["lowering_offset_m"])
    if not 0 < overtravel <= .003:
        raise ValueError("invalid guarded lowering offset")
    lowering_target = center.copy()
    lowering_target[2] -= overtravel
    lower = np.degrees(solve_horizontal_endpoint(
        chain, "left", lowering_target, np.radians(lower), restarts=1,
        iterations=240, random_seed=731,
    ))
    def pose(name, duration, q, closed):
        return _pose("TRAY_"+name, duration, q, parked["right_joint_deg"],
                     left["gripper_close_rad"] if closed else left["gripper_open_rad"],
                     right["gripper_open_m"])
    deposit = [pose("START", .5, start, True), pose("MOVE_ABOVE", 5., upper, True),
               pose("LOWER", 3., lower, True), pose("TABLE_SETTLE", 1., lower, True),
               pose("OPEN", 2., lower, False), pose("RELEASE_HOLD", 1., lower, False),
               pose("WITHDRAW", 3., back_deg, False), pose("CLEAR_ABOVE", 3., back_above_deg, False),
               pose("FRONT_CLEAR", 5., front_clear_deg, False),
               pose("PARK", 5., start, False),
               pose("PLACE_HOLD", 2., start, False)]
    regrasp = [pose("REGRASP_START", .5, start, False), pose("RETURN_ABOVE", 5., front_clear_deg, False),
               pose("PREGRASP_ABOVE", 5., back_above_deg, False),
               pose("PREGRASP", 3., back_deg, False), pose("REAPPROACH", 3., contact_deg, False),
               pose("CLOSE", 2., contact_deg, True), pose("CONTACT_HOLD", 1., contact_deg, True),
               pose("RELIFT", 3., upper, True),
               pose("RETURN_SERVICE", 5., start, True), pose("LIFT_HOLD", 2., start, True)]
    minimum = math.inf
    maximum_tilt = 0.
    for a, b in ((start, upper), (upper, lower)):
        for fraction in np.linspace(0., 1., 201):
            q = np.radians(a*(1-fraction)+b*fraction)
            position, _, cup_tilt, cup_bottom = _cup_pose_metrics(chain, q, axis, .12, .028)
            # Conservative XY extent: square support half-width plus full cup radius.
            over_stand = np.all(np.abs(position[:2]) <= .055+.045)
            clearance = cup_bottom-(surface if over_stand else .72)
            minimum = min(minimum, clearance)
            maximum_tilt = max(maximum_tilt, cup_tilt)
    accepted = minimum >= -overtravel-1e-5 and maximum_tilt <= 2.
    return {"schema": "raised_tray_transfer_v1", "deposit_plan": {"poses": deposit},
            "regrasp_plan": {"poses": regrasp}, "tray_center_m": center.tolist(),
            "tray_center_goal_m": [0., 0., surface+.06], "tray_surface_z_m": surface,
            "edge_supported_release": False, "executable": accepted,
            "guarded_lowering_offset_m": overtravel,
            "rim_backoff_m": backoff_m,
            "backoff_position_error_mm": back["position_error_mm"],
            "path_audit": {"accepted": accepted, "minimum_support_clearance_m": minimum,
                           "maximum_cup_tilt_deg": maximum_tilt, "samples_per_segment": 201},
            "physics_validated": False, "continuous_collision_validated": False,
            "hardware_accessed": False}
