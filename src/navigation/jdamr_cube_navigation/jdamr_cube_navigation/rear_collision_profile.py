"""Scoped Collision Monitor override for a live rear docking target.

The dock face is an intentional obstacle.  During the bounded rear approach,
``FootprintApproach`` would therefore predict the requested one-centimetre stop
as a collision.  This profile leaves the directional ``StopZone`` enabled and
replaces only that predictive action with the same live scan, footprint and
remembered blind-range points.
"""

from contextlib import contextmanager
import math
import time

from jdamr_cube_navigation.corridor_route import spin_node
from jdamr_cube_navigation.reverse_parking import path_sweep_hit, scan_points
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


SCAN_MAX_AGE_S = 0.5
PARAMETER_TIMEOUT_S = 2.0
PREDICTION_STEP_S = 0.05
# Keep these synchronized with the direct-motion guard and stop latency.  This
# module cannot import restaurant_service: rear_box_service is loaded before it.
DIRECT_GUARD_PERIOD_S = 0.1
DIRECT_STOP_LATENCY_S = 0.05
EXPECTED_LINEAR_DECEL_MPS2 = 0.5


def _wait_future(node, future, timeout_s=PARAMETER_TIMEOUT_S):
    """Wait through operator stop so a parameter restoration can still finish."""
    if not hasattr(future, 'done'):
        return future
    deadline_s = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline_s:
        spin_node(node, timeout_sec=0.05)
    if not future.done():
        try:
            future.cancel()
        except (AttributeError, RuntimeError):
            pass
        raise RuntimeError('rear collision parameter request timed out')
    if future.exception() is not None:
        raise RuntimeError(
            f'rear collision parameter request failed: {future.exception()}')
    return future.result()


def _parameter_client(node, attribute, remote_node):
    client = getattr(node, attribute, None)
    if client is None:
        client = AsyncParameterClient(node, remote_node)
        setattr(node, attribute, client)
    if not client.wait_for_services(timeout_sec=PARAMETER_TIMEOUT_S):
        raise RuntimeError(f'{remote_node} parameter service unavailable')
    return client


def _read_parameters(node, client, names):
    response = _wait_future(node, client.get_parameters(list(names)))
    values = getattr(response, 'values', None)
    if values is None or len(values) != len(names):
        raise RuntimeError('rear collision parameter response is incomplete')
    return {
        name: parameter_value_to_python(value)
        for name, value in zip(names, values)
    }


def _set_footprint_approach(node, client, enabled, expected_stop_zone=True):
    response = _wait_future(node, client.set_parameters_atomically([
        Parameter('FootprintApproach.enabled', value=bool(enabled)),
    ]))
    result = getattr(response, 'result', None)
    if result is None or result.successful is not True:
        reason = getattr(result, 'reason', '') if result is not None else ''
        raise RuntimeError(reason or 'FootprintApproach parameter change rejected')
    confirmed = _read_parameters(
        node, client, ('StopZone.enabled', 'FootprintApproach.enabled'))
    if confirmed['StopZone.enabled'] is not expected_stop_zone:
        raise RuntimeError('rear collision profile lost StopZone')
    if confirmed['FootprintApproach.enabled'] is not bool(enabled):
        raise RuntimeError('FootprintApproach parameter change not confirmed')


def _fail_closed(node, reason):
    """Latch the existing stop paths without hiding the parameter failure."""
    try:
        node.engage_emergency_stop(reason)
    except Exception:  # The original restoration error remains the caller's evidence.
        pass
    try:
        node.request_stop()
    except Exception:
        pass


def _stamp_s(scan):
    stamp = scan.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _prediction_poses(linear_mps, angular_radps, horizon_s):
    count = max(1, math.ceil(horizon_s / PREDICTION_STEP_S))
    poses = [(0.0, 0.0, 0.0)]
    for index in range(1, count + 1):
        elapsed_s = horizon_s * index / count
        yaw = angular_radps * elapsed_s
        if abs(angular_radps) <= 1e-9:
            poses.append((linear_mps * elapsed_s, 0.0, yaw))
            continue
        radius_m = linear_mps / angular_radps
        poses.append((radius_m * math.sin(yaw),
                      radius_m * (1.0 - math.cos(yaw)), yaw))
    return poses


def _larger_live_velocity(command, latest_motion, now_s):
    linear = float(command.linear.x)
    angular = float(command.angular.z)
    if not all(math.isfinite(value) for value in (linear, angular)):
        raise ValueError('rear collision command is not finite')
    if latest_motion is None or len(latest_motion) != 3:
        return linear, angular
    received_s, actual_linear, actual_angular = latest_motion
    if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value)
                for value in (received_s, actual_linear, actual_angular))
            or not 0.0 <= now_s - received_s <= SCAN_MAX_AGE_S):
        return linear, angular
    if abs(actual_linear) > abs(linear):
        # CorridorRoute stores planar odometry speed as a magnitude.  Preserve
        # the requested direction while accounting for the larger live speed.
        linear = math.copysign(abs(actual_linear), linear or 1.0)
    if abs(actual_angular) > abs(angular):
        angular = actual_angular
    return linear, angular


def _watch(node, command, state, linear_decel_mps2, angular_decel_radps2):
    requested = (float(command.linear.x), float(command.angular.z))
    if not all(math.isfinite(value) for value in requested):
        return 'rear collision command invalid'
    # A zero command must always pass so _direct_motion can stop in finally.
    if abs(requested[0]) <= 1e-12 and abs(requested[1]) <= 1e-12:
        return None

    scan = getattr(node, 'last_scan', None)
    received_s = (getattr(node, 'samples', None) or {}).get('scan')
    if scan is None or received_s is None:
        return 'rear box scan unavailable'
    now_s = time.monotonic()
    receive_age_s = now_s - received_s
    try:
        stamp_s = _stamp_s(scan)
        ros_age_s = node.get_clock().now().nanoseconds * 1e-9 - stamp_s
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 'rear collision scan timestamp invalid'
    if (not all(math.isfinite(value) for value in (receive_age_s, ros_age_s))
            or not 0.0 <= receive_age_s <= SCAN_MAX_AGE_S
            or not 0.0 <= ros_age_s <= SCAN_MAX_AGE_S):
        return 'rear box scan stale or future-dated'
    if scan.header.frame_id != state['frame_id']:
        return 'rear collision scan frame changed'

    scan_key = (id(scan), stamp_s)
    if scan_key != state['scan_key']:
        try:
            state['memory'] = tuple(node._remembered_blind_points(state['laser'], scan))
        except (AttributeError, RuntimeError, ValueError):
            return 'rear collision blind-range memory unavailable'
        state['scan_key'] = scan_key
    try:
        linear_mps, angular_radps = _larger_live_velocity(
            command, getattr(node, 'latest_motion', None), now_s)
        reaction_s = (max(receive_age_s, ros_age_s)
                      + DIRECT_GUARD_PERIOD_S + DIRECT_STOP_LATENCY_S)
        horizon_s = reaction_s + max(
            abs(linear_mps) / linear_decel_mps2,
            abs(angular_radps) / angular_decel_radps2)
        points = [*scan_points(scan, state['laser']), *state['memory']]
        poses = _prediction_poses(linear_mps, angular_radps, horizon_s)
        if path_sweep_hit(
                points, state['outline'], poses, push_tolerance_m=0.0):
            return 'rear docking swept footprint occupied'
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 'rear collision scan geometry invalid'
    return None


@contextmanager
def rear_collision_profile(node):
    """Disable only FootprintApproach while preserving an equivalent live guard."""
    snapshot = node._scan_in_base()
    if snapshot is None:
        raise RuntimeError('rear box scan unavailable')
    scan, laser, outline, memory = snapshot
    if not outline:
        raise RuntimeError('rear collision footprint unavailable')

    monitor = _parameter_client(
        node, 'precision_parameters', 'collision_monitor')
    smoother = _parameter_client(
        node, 'rear_collision_smoother_parameters', 'velocity_smoother')
    enabled = _read_parameters(
        node, monitor, ('StopZone.enabled', 'FootprintApproach.enabled'))
    if enabled['StopZone.enabled'] is not True:
        raise RuntimeError('rear collision profile requires StopZone enabled')
    if type(enabled['FootprintApproach.enabled']) is not bool:
        raise RuntimeError('FootprintApproach enabled state is invalid')
    if enabled['FootprintApproach.enabled'] is not True:
        try:
            _set_footprint_approach(node, monitor, True)
        except RuntimeError:
            _fail_closed(
                node, 'FootprintApproach could not be restored before rear docking')
            raise
        node.emit('rear_collision_profile_recovered',
                  footprint_approach_enabled=True, stop_zone_enabled=True)
    if getattr(node, 'stop_requested', False):
        raise RuntimeError('operator_stop')

    max_decel = _read_parameters(node, smoother, ('max_decel',))['max_decel']
    if (not isinstance(max_decel, (list, tuple)) or len(max_decel) < 3
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in (max_decel[0], max_decel[2]))):
        raise RuntimeError('velocity_smoother max_decel is invalid')
    linear_decel_mps2, angular_decel_radps2 = abs(max_decel[0]), abs(max_decel[2])
    if (not math.isclose(linear_decel_mps2, EXPECTED_LINEAR_DECEL_MPS2,
                         rel_tol=0.0, abs_tol=1e-9)
            or angular_decel_radps2 <= 0.0):
        raise RuntimeError('velocity_smoother deceleration does not match direct motion')

    state = {
        'laser': tuple(laser), 'outline': tuple(tuple(point) for point in outline),
        'memory': tuple(memory), 'frame_id': scan.header.frame_id,
        'scan_key': (id(scan), _stamp_s(scan)),
    }
    try:
        _set_footprint_approach(node, monitor, False)
        yield lambda command: _watch(
            node, command, state, linear_decel_mps2, angular_decel_radps2)
    finally:
        try:
            _set_footprint_approach(node, monitor, True)
        except RuntimeError:
            _fail_closed(
                node, 'FootprintApproach could not be restored after rear docking')
            raise
