#!/usr/bin/env python3
"""CPU-only differential-drive path follower for the mobile service base.

The measured pose is the chassis-centre pose.  Wheel commands, however, act at
the differential axle, which is ``axle_offset`` metres ahead of that pose.
This module therefore performs path following and final-pose regulation in the
axle frame while reporting errors for the requested chassis-centre goal.
"""

from __future__ import annotations

import math

import numpy as np


MAX_LINEAR_M_S = 0.08
MAX_ANGULAR_RAD_S = 0.3
FINAL_POSITION_TOLERANCE_M = 0.015
FINAL_YAW_TOLERANCE_RAD = 0.02


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_vector(value: object, size: int, name: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if array.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _finite_path(path_xy: object) -> np.ndarray:
    try:
        path = np.asarray(path_xy, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("path_xy must contain numeric values") from exc
    if path.ndim != 2 or path.shape[1:] != (2,) or len(path) == 0:
        raise ValueError("path_xy must be a non-empty array with shape (N, 2)")
    if not np.all(np.isfinite(path)):
        raise ValueError("path_xy must contain only finite values")
    return path


def _path_headings(path: np.ndarray, goal_yaw: float) -> np.ndarray:
    """Return a stable forward heading for each chassis-centre waypoint."""
    headings = np.full(len(path), goal_yaw, dtype=float)
    for index in range(len(path) - 1):
        for next_index in range(index + 1, len(path)):
            delta = path[next_index] - path[index]
            if float(np.linalg.norm(delta)) > 1e-9:
                headings[index] = math.atan2(float(delta[1]), float(delta[0]))
                break
    return headings


def _axle_reference(path: np.ndarray, goal_yaw: float, axle_offset: float) -> np.ndarray:
    headings = _path_headings(path, goal_yaw)
    offsets = axle_offset * np.column_stack((np.cos(headings), np.sin(headings)))
    return path + offsets


def _lookahead_target(polyline: np.ndarray, point: np.ndarray,
                      lookahead: float) -> tuple[np.ndarray, float]:
    """Project onto a polyline and advance by lookahead; return target/remain."""
    if len(polyline) == 1:
        return polyline[0].copy(), float(np.linalg.norm(polyline[0] - point))

    lengths = np.linalg.norm(np.diff(polyline, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    best_distance = math.inf
    best_progress = 0.0

    for index, length in enumerate(lengths):
        start = polyline[index]
        if length <= 1e-12:
            fraction = 0.0
            projection = start
        else:
            segment = polyline[index + 1] - start
            fraction = float(np.clip(np.dot(point - start, segment) / (length * length), 0.0, 1.0))
            projection = start + fraction * segment
        distance = float(np.dot(point - projection, point - projection))
        progress = float(cumulative[index] + fraction * length)
        if distance < best_distance or (math.isclose(distance, best_distance) and progress > best_progress):
            best_distance = distance
            best_progress = progress

    total = float(cumulative[-1])
    target_progress = min(total, best_progress + lookahead)
    segment_index = min(int(np.searchsorted(cumulative, target_progress, side="right") - 1), len(lengths) - 1)
    segment_length = float(lengths[segment_index])
    if segment_length <= 1e-12:
        target = polyline[segment_index + 1].copy()
    else:
        fraction = (target_progress - cumulative[segment_index]) / segment_length
        target = polyline[segment_index] + fraction * (
            polyline[segment_index + 1] - polyline[segment_index]
        )
    return target, total - best_progress


def follow_path(
    pose_xyyaw: object,
    path_xy: object,
    goal_yaw: float,
    wheel_radius: float = 0.0329,
    wheel_track: float = 0.510,
    axle_offset: float = 0.105,
) -> dict[str, float | bool]:
    """Compute one bounded wheel-velocity command for a static waypoint path.

    Positive angular velocity is counter-clockwise; consequently a left turn
    commands the right wheel faster than the left wheel.  The function is
    stateless and never modifies the supplied pose or path.
    """
    pose = _finite_vector(pose_xyyaw, 3, "pose_xyyaw")
    path = _finite_path(path_xy)
    try:
        goal_yaw = float(goal_yaw)
        wheel_radius = float(wheel_radius)
        wheel_track = float(wheel_track)
        axle_offset = float(axle_offset)
    except (TypeError, ValueError) as exc:
        raise ValueError("controller parameters must be numeric") from exc
    parameters = np.array([goal_yaw, wheel_radius, wheel_track, axle_offset])
    if not np.all(np.isfinite(parameters)):
        raise ValueError("controller parameters must be finite")
    if wheel_radius <= 0.0 or wheel_track <= 0.0 or axle_offset < 0.0:
        raise ValueError("wheel dimensions must be positive and axle_offset non-negative")

    chassis_xy = pose[:2]
    yaw = float(pose[2])
    goal_xy = path[-1]
    position_error = float(np.linalg.norm(goal_xy - chassis_xy))
    yaw_error = _wrap_angle(goal_yaw - yaw)
    arrived = (
        position_error <= FINAL_POSITION_TOLERANCE_M
        and abs(yaw_error) <= FINAL_YAW_TOLERANCE_RAD
    )

    linear = 0.0
    angular = 0.0
    if not arrived:
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        axle_xy = chassis_xy + axle_offset * heading
        axle_path = _axle_reference(path, goal_yaw, axle_offset)
        final_axle = axle_path[-1]
        final_axle_error = float(np.linalg.norm(final_axle - axle_xy))

        # A pose regulator completes the manoeuvre without leaving an offset
        # equal to axle_offset at the requested chassis-centre goal.
        if position_error < 0.18 or final_axle_error < 0.18 or len(path) == 1:
            delta = final_axle - axle_xy
            bearing = math.atan2(float(delta[1]), float(delta[0]))
            alpha = _wrap_angle(bearing - yaw)
            beta = _wrap_angle(goal_yaw - yaw - alpha)
            linear = 0.9 * float(delta @ heading)
            # Fade the ill-conditioned bearing term near the axle target.
            # A hard 5 mm switch can alternate opposite turns under wheel lag.
            bearing_weight = final_axle_error**2 / (final_axle_error**2 + .01**2)
            angular = (bearing_weight * (1.8 * alpha - 0.55 * beta)
                       + (1.0 - bearing_weight) * 1.5 * yaw_error)
        else:
            lookahead = min(0.15, max(0.08, 0.08 + 0.20 * position_error))
            target, _ = _lookahead_target(axle_path, axle_xy, lookahead)
            target_delta = target - axle_xy
            target_distance = max(float(np.linalg.norm(target_delta)), 1e-9)
            alpha = _wrap_angle(math.atan2(float(target_delta[1]), float(target_delta[0])) - yaw)
            # Slow before sharp turns and when the axle is not facing forward.
            linear = MAX_LINEAR_M_S * max(0.15, math.cos(alpha))
            curvature = 2.0 * math.sin(alpha) / target_distance
            angular = linear * curvature

        linear = float(np.clip(linear, -MAX_LINEAR_M_S, MAX_LINEAR_M_S))
        angular = float(np.clip(angular, -MAX_ANGULAR_RAD_S, MAX_ANGULAR_RAD_S))

    left = (linear - 0.5 * wheel_track * angular) / wheel_radius
    right = (linear + 0.5 * wheel_track * angular) / wheel_radius
    return {
        "left_rad_s": float(left),
        "right_rad_s": float(right),
        "linear_m_s": linear,
        "angular_rad_s": angular,
        "arrived": bool(arrived),
        "position_error_m": position_error,
        "yaw_error_rad": yaw_error,
    }


__all__ = ["follow_path"]
