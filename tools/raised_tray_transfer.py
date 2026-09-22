"""Experimental raised-center transfer, with geometric clearance audits."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions, _limits, solve_horizontal_endpoint
from tray_transfer_plan import _pose, _named_pose, _cup_pose_metrics
from search_tray_mounts import _solve_stage


def _solve_unloaded_stage(chain, target_m, seed_rad):
    """Position IK for an empty, open hand; cup-upright is not an empty-hand constraint."""
    low, high = _limits(chain, "left")
    low += math.radians(3.)
    high -= math.radians(3.)
    seed = np.asarray(seed_rad).copy()
    q = np.clip(seed, low, high)
    damping = 1e-3

    def residual(angles):
        tcp = chain.transforms(base_positions("left", angles))["left_tool0"][:3, 3]
        return np.r_[(tcp-target_m)*10., (angles-seed)*.01]

    for _ in range(240):
        raw = residual(q)
        jacobian = np.empty((8, 5))
        for i in range(5):
            probe = q.copy()
            probe[i] += 1e-5
            jacobian[:, i] = (residual(probe)-raw)/1e-5
        candidate = np.clip(q-np.linalg.solve(jacobian.T@jacobian+damping*np.eye(5), jacobian.T@raw), low, high)
        if np.linalg.norm(residual(candidate)) < np.linalg.norm(raw):
            q = candidate
            damping = max(1e-8, damping*.4)
        else:
            damping = min(1e5, damping*5.)
    error_mm = float(np.linalg.norm(residual(q)[:3])*100.)
    return {"q": q, "accepted": error_mm <= 2., "position_error_mm": error_mm}


def build_raised_transfer(model, source):
    return _build_support_transfer(model, source, plate=False)


def build_plate_transfer(model, source):
    return _build_support_transfer(model, source, plate=True)


def _build_support_transfer(model, source, *, plate):
    if "raised_tray" in source and "plate_transfer" in source:
        raise ValueError("plate and raised support are mutually exclusive")
    spec = source["plate_transfer" if plate else "raised_tray"]
    height = 0. if plate else float(spec["height_m"])
    if not plate and not .04 <= height <= .20:
        raise ValueError("raised tray height outside experimental range")
    target_xy = np.asarray(spec["target_xy_m"] if plate else [0., 0.], dtype=float)
    radius = max(r for _, r in source["experiment"]["cup_radius_profile_m"])
    if (target_xy.shape != (2,) or not np.isfinite(target_xy).all()
            or abs(target_xy[0])+radius > .170 or abs(target_xy[1])+radius > .225):
        raise ValueError("support target outside plate")
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
    target_tolerance_m = .002 if plate else .02
    if np.linalg.norm(center[:2]-target_xy) > target_tolerance_m:
        raise ValueError("contact endpoint outside target region")
    if abs(bottom-surface) > .002 or tilt > 2.:
        raise ValueError("raised contact endpoint is not central/upright/on support")
    loaded_ascent = []
    if plate:
        above_target = chain.transforms(base_positions("left", np.radians(upper)))["left_tool0"][:3, 3]
        previous = np.radians(lower)
        for fraction in np.linspace(1/6, 1., 6):
            target = center*(1-fraction)+above_target*fraction
            step = _solve_stage(chain, target, previous, axis, center_region=False,
                                restarts=2, iterations=180, random_seed=4600+len(loaded_ascent))
            if not step["accepted"]:
                raise ValueError("loaded vertical waypoint unreachable")
            loaded_ascent.append(np.degrees(step["q"]))
            previous = step["q"]
        upper = loaded_ascent[-1]
    left, right = source["left_config"], source["right_config"]
    right_park = spec.get("right_park_joint_deg", parked["right_joint_deg"]) if plate else parked["right_joint_deg"]
    right_low, right_high = _limits(chain, "right")
    right_angles = np.radians(right_park)
    if (right_angles.shape != (5,) or not np.isfinite(right_angles).all()
            or np.min(np.minimum(right_angles-right_low, right_high-right_angles)) < math.radians(3.)):
        raise ValueError("invalid right clearance pose")
    contact_deg = lower.copy()
    contact_transform = chain.transforms(base_positions("left", np.radians(lower)))["left_tool0"]
    heading = contact_transform[:3, 2].copy()
    heading[2] = 0.
    heading /= np.linalg.norm(heading)
    backoff_m = float(source["experiment"]["cup_pregrasp_backoff_m"])
    if plate:
        backoff_m = float(spec.get("empty_hand_backoff_m", backoff_m))
    if not np.isfinite(backoff_m) or not 0 < backoff_m <= .15:
        raise ValueError("invalid empty hand backoff")
    back_target = center-heading*backoff_m
    back = (_solve_unloaded_stage(chain, back_target, np.radians(lower)) if plate else
            _solve_stage(chain, back_target, np.radians(lower), axis, center_region=False,
                         restarts=8, iterations=320, random_seed=721))
    if not back["accepted"]:
        raise ValueError("same-height rim-clear backoff is not reachable")
    unloaded_lift_m = float(spec.get("empty_hand_lift_m", backoff_m)) if plate else backoff_m
    if not np.isfinite(unloaded_lift_m) or not 0 < unloaded_lift_m <= .20:
        raise ValueError("invalid empty hand lift")
    back_above_target = back_target+np.array([0., 0., unloaded_lift_m])
    back_above = (_solve_unloaded_stage(chain, back_above_target, np.radians(lower)) if plate else
                  _solve_stage(chain, back_above_target, back["q"], axis, center_region=False,
                               restarts=8, iterations=320, random_seed=722))
    if not back_above["accepted"]:
        raise ValueError("rim-clear pregrasp above is not reachable")
    back_deg, back_above_deg = np.degrees(back["q"]), np.degrees(back_above["q"])
    unloaded_ascent = []
    if plate:
        previous = back["q"]
        for fraction in np.linspace(1/6, 1., 6):
            step = _solve_unloaded_stage(chain, back_target+[0., 0., unloaded_lift_m*fraction], previous)
            if not step["accepted"]:
                raise ValueError("empty hand vertical waypoint unreachable")
            unloaded_ascent.append(np.degrees(step["q"]))
            previous = step["q"]
        back_above_deg = unloaded_ascent[-1]
    front_clear = _solve_stage(chain, np.array([.20, .17, 1.08]), np.radians(start) if plate else back_above["q"], axis,
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
    if plate:
        lowered = _solve_stage(chain, lowering_target, np.radians(lower), axis, center_region=False,
                               restarts=2, iterations=240, random_seed=731)
        if not lowered["accepted"]:
            raise ValueError("guarded lowering pose unreachable")
        lower = np.degrees(lowered["q"])
    else:
        lower = np.degrees(solve_horizontal_endpoint(
            chain, "left", lowering_target, np.radians(lower), restarts=1,
            iterations=240, random_seed=731,
        ))
    def pose(name, duration, q, closed):
        return _pose("TRAY_"+name, duration, q, right_park,
                     left["gripper_close_rad"] if closed else left["gripper_open_rad"],
                     right["gripper_open_m"])
    deposit = [pose("START", .5, start, True), pose("MOVE_ABOVE", 5., upper, True),
               pose("LOWER", 3., lower, True), pose("TABLE_SETTLE", 1., lower, True),
               pose("OPEN", 2., lower, False), pose("RELEASE_HOLD", 1., lower, False),
               pose("WITHDRAW", 3., back_deg, False), pose("CLEAR_ABOVE", 3., back_above_deg, False),
               pose("FRONT_CLEAR", 5., front_clear_deg, False),
               pose("PARK", 5., start, False),
               pose("PLACE_HOLD", 2., start, False)]
    if plate and "right_park_joint_deg" in spec:
        deposit[0]["right_joint_deg"] = list(parked["right_joint_deg"])
        previous = np.radians(parked["right_joint_deg"])
        right_origin = chain.transforms(base_positions("right", previous))["right_tool0"][:3, 3]
        right_clearance = []
        # Simulation candidate: lift the open hand 120 mm before folding it away.
        for index, height_offset in enumerate(np.linspace(.02, .12, 6)):
            target = right_origin+np.array([0., 0., height_offset])
            angles = solve_horizontal_endpoint(chain, "right", target, previous,
                restarts=4, iterations=240, random_seed=487+index)
            actual = chain.transforms(base_positions("right", angles))["right_tool0"][:3, 3]
            if (not np.isfinite(angles).all() or np.linalg.norm(actual-target) > .002
                    or np.min(np.minimum(angles-right_low, right_high-angles)) < math.radians(3.)):
                raise ValueError("right clearance lift unreachable")
            waypoint = pose("RIGHT_CLEAR_LIFT", .75, start, True)
            waypoint["right_joint_deg"] = np.degrees(angles).tolist()
            right_clearance.append(waypoint)
            previous = angles
        deposit[1:1] = [*right_clearance, pose("RIGHT_CLEAR", 5., start, True)]
    regrasp = [pose("REGRASP_START", .5, start, False), pose("RETURN_ABOVE", 5., front_clear_deg, False),
               pose("PREGRASP_ABOVE", 5., back_above_deg, False),
               pose("PREGRASP", 3., back_deg, False), pose("REAPPROACH", 3., contact_deg, False),
               pose("CLOSE", 2., contact_deg, True), pose("CONTACT_HOLD", 1., contact_deg, True),
               pose("RELIFT", 3., upper, True),
               pose("RETURN_SERVICE", 5., start, True), pose("LIFT_HOLD", 2., start, True)]
    if plate:
        lower_index = next(i for i, p in enumerate(deposit) if p["name"] == "TRAY_LOWER")
        deposit[lower_index:lower_index+1] = [pose("LOWER", .5, q, True)
            for q in [*reversed(loaded_ascent[:-1]), lower]]
        lift_index = next(i for i, p in enumerate(regrasp) if p["name"] == "TRAY_RELIFT")
        regrasp[lift_index:lift_index+1] = [pose("RELIFT", .5, q, True) for q in loaded_ascent]
        clear_index = next(i for i, p in enumerate(deposit) if p["name"] == "TRAY_CLEAR_ABOVE")
        deposit[clear_index:clear_index+1] = [pose("CLEAR_ABOVE", .5, q, False) for q in unloaded_ascent]
        descend_index = next(i for i, p in enumerate(regrasp) if p["name"] == "TRAY_PREGRASP")
        regrasp[descend_index:descend_index+1] = [pose("PREGRASP", .5, q, False)
            for q in [*reversed(unloaded_ascent[:-1]), back_deg]]
    minimum = math.inf
    maximum_tilt = 0.
    audit_waypoints = [start, upper, *reversed(loaded_ascent[:-1]), lower] if plate else [start, upper, lower]
    for a, b in zip(audit_waypoints, audit_waypoints[1:]):
        for fraction in np.linspace(0., 1., 201):
            q = np.radians(a*(1-fraction)+b*fraction)
            position, _, cup_tilt, cup_bottom = _cup_pose_metrics(chain, q, axis, .12, .028)
            # Conservative XY extent: square support half-width plus full cup radius.
            over_stand = plate or np.all(np.abs(position[:2]) <= .055+.045)
            clearance = cup_bottom-(surface if over_stand else .72)
            minimum = min(minimum, clearance)
            maximum_tilt = max(maximum_tilt, cup_tilt)
    accepted = minimum >= -overtravel-1e-5 and maximum_tilt <= 2.
    return {"schema": "plate_transfer_v1" if plate else "raised_tray_transfer_v1", "deposit_plan": {"poses": deposit},
            "regrasp_plan": {"poses": regrasp}, "tray_center_m": center.tolist(),
            "tray_center_goal_m": [*target_xy, surface+.06], "tray_surface_z_m": surface,
            "tray_target_xy_m": target_xy.tolist(),
            "edge_supported_release": False, "executable": accepted,
            "guarded_lowering_offset_m": overtravel,
            "rim_backoff_m": backoff_m,
            "empty_hand_lift_m": unloaded_lift_m,
            "backoff_position_error_mm": back["position_error_mm"],
            "path_audit": {"accepted": accepted, "minimum_support_clearance_m": minimum,
                           "maximum_cup_tilt_deg": maximum_tilt, "samples_per_segment": 201,
                           "scope": "loaded_left_cup_kinematics_only_not_collision_certification"},
            "physics_validated": False, "continuous_collision_validated": False,
            "hardware_accessed": False}
