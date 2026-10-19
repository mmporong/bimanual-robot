"""Plane-relative approach commands and their nominal footprint corridor."""

import math


FACE_HEADING_TOLERANCE_RAD = math.radians(0.3)
FACE_HEADING_DEADBAND_RAD = math.radians(0.1)
FACE_ALIGNMENT_GAP_M = 0.05
FACE_HEADING_FILTER_S = 0.35
FACE_APPROACH_TIME_S = 0.2


def filtered_face_heading(previous, observed, elapsed_s):
    """Filter a circular heading with the same state law in control and preview."""
    if not math.isfinite(observed) or not math.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError('invalid table heading filter input')
    if previous is None:
        return observed
    if not math.isfinite(previous):
        raise ValueError('invalid previous table heading')
    delta = math.atan2(math.sin(observed - previous), math.cos(observed - previous))
    return previous + (1.0 - math.exp(-elapsed_s / FACE_HEADING_FILTER_S)) * delta


def face_approach_command(remaining_m, error_rad, target_gap_m=0.01, *,
                          approach_time_s=FACE_APPROACH_TIME_S):
    """Reduce heading error before entering the existing final precision band."""
    if not all(math.isfinite(v) for v in (
            remaining_m, error_rad, target_gap_m, approach_time_s)) or approach_time_s <= 0.0:
        raise ValueError('nonfinite table approach command')
    angular = max(-0.02, min(0.02, error_rad))
    if abs(error_rad) <= FACE_HEADING_DEADBAND_RAD:
        angular = 0.0
    linear = min(0.05, max(0.01, remaining_m / approach_time_s)) if remaining_m > 0.0 else 0.0
    if abs(error_rad) > FACE_HEADING_TOLERANCE_RAD:
        clearance = remaining_m - max(0.0, FACE_ALIGNMENT_GAP_M - target_gap_m)
        if clearance <= 0.0:
            raise ValueError('table alignment requires more clearance')
        # [unit-mismatch: paradigmatic] angle [rad] / rate [rad/s] gives time [s].
        # Remaining alignment distance [m] / time [s] bounds linear speed [m/s].
        linear = min(linear, clearance / max(
            approach_time_s, 2.0 * abs(error_rad) / abs(angular)))
    return linear, angular


def face_approach_waypoints(start, face, normal, front_extent_m, half_width_m,
                            target_gap_m=0.01, *, command_heading_rad=None,
                            approach_time_s=FACE_APPROACH_TIME_S):
    """Preview the same command law up to the existing final precision band.

    This ideal odometry rollout is for static corridor validation, not evidence
    of real traction or physical parking accuracy.
    """
    values = (*start, *face, *normal, front_extent_m, half_width_m,
              target_gap_m, approach_time_s)
    if (len(start) != 3 or len(face) != 2 or len(normal) != 2
            or not all(math.isfinite(v) for v in values)
            or not math.isclose(math.hypot(*normal), 1.0, abs_tol=1e-6)
            or front_extent_m <= 0.0 or half_width_m <= 0.0 or target_gap_m <= 0.0
            or approach_time_s <= 0.0):
        raise ValueError('invalid table approach preview geometry')
    heading = math.atan2(-normal[1], -normal[0])
    command_heading = filtered_face_heading(command_heading_rad, heading, 0.0)
    pose = tuple(start)
    poses = [pose]
    for _step in range(1600):
        error = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        if abs(error) > math.radians(30.0) + 1e-9:
            raise ValueError('table approach preview heading out of range')
        lead = front_extent_m * math.cos(error) + half_width_m * abs(math.sin(error))
        gap = sum((pose[i] - face[i]) * normal[i] for i in (0, 1)) - lead
        if gap <= 0.0:
            raise ValueError('table approach preview crosses surface')
        command_error = math.atan2(math.sin(command_heading - pose[2]),
                                   math.cos(command_heading - pose[2]))
        if gap <= max(FACE_ALIGNMENT_GAP_M, target_gap_m) + 0.0005:
            if (target_gap_m <= FACE_ALIGNMENT_GAP_M and
                    max(abs(error), abs(command_error)) > FACE_HEADING_TOLERANCE_RAD):
                raise ValueError('table alignment requires more clearance')
            return poses
        remaining = (gap - target_gap_m) / math.cos(error)
        if math.dist(start[:2], pose[:2]) + remaining > 0.6 + 1e-9:
            raise ValueError('table approach preview exceeds bounded travel')
        linear, angular = face_approach_command(
            remaining, command_error, target_gap_m, approach_time_s=approach_time_s)
        closing_speed_bound = linear + abs(angular) * (front_extent_m + half_width_m)
        step_s = min(0.05, (gap - max(FACE_ALIGNMENT_GAP_M, target_gap_m))
                     / max(closing_speed_bound, 1e-9))
        pose = (pose[0] + linear * math.cos(pose[2]) * step_s,
                pose[1] + linear * math.sin(pose[2]) * step_s,
                pose[2] + angular * step_s)
        poses.append(pose)
        command_heading = filtered_face_heading(command_heading, heading, step_s)
    raise ValueError('table approach preview did not converge')
