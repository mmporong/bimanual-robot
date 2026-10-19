"""Rear docking keeps StopZone while replacing only predictive approach."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Twist
from jdamr_cube_navigation import rear_collision_profile as module
import pytest
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.task import Future


OUTLINE = ((-.28, -.23), (-.28, .23), (.12, .23), (.12, -.23))


def _value(value):
    return Parameter('p', value=value).get_parameter_value()


class ParameterClient:
    def __init__(self, values, restore_fails=False, restore_times_out=False):
        self.values = dict(values)
        self.restore_fails = restore_fails
        self.restore_times_out = restore_times_out
        self.sets = []

    def wait_for_services(self, timeout_sec=None):
        return True

    @staticmethod
    def _future(response):
        future = Future()
        future.set_result(response)
        return future

    def get_parameters(self, names):
        response = GetParameters.Response()
        response.values = [_value(self.values[name]) for name in names]
        return self._future(response)

    def set_parameters_atomically(self, parameters):
        requested = {parameter.name: parameter.value for parameter in parameters}
        self.sets.append(requested)
        enabled = requested['FootprintApproach.enabled']
        if self.restore_times_out and enabled is True:
            return Future()
        successful = not (self.restore_fails and enabled is True)
        if successful:
            self.values.update(requested)
        response = SetParametersAtomically.Response()
        response.result = SetParametersResult(
            successful=successful, reason='' if successful else 'restore rejected')
        return self._future(response)


def scan(points=(), stamp_s=10.0):
    ranges = [math.inf] * 721
    for x_m, y_m in points:
        angle = math.atan2(y_m, x_m)
        index = int(round((angle + math.pi) / (2.0 * math.pi / 720)))
        ranges[index] = math.hypot(x_m, y_m)
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='laser_link', stamp=SimpleNamespace(
                sec=int(stamp_s), nanosec=int((stamp_s % 1.0) * 1e9))),
        ranges=ranges, angle_min=-math.pi, angle_increment=2.0 * math.pi / 720,
        range_min=.05, range_max=12.0)


def command(linear=0.0, angular=0.0):
    result = Twist()
    result.linear.x, result.angular.z = linear, angular
    return result


def node(monkeypatch, *, stop_zone=True, approach=True, restore_fails=False,
         restore_times_out=False, points=(), memory=()):
    clock = {'mono': 10.0, 'ros': 10.0}
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['mono'])
    message = scan(points)
    monitor = ParameterClient({
        'StopZone.enabled': stop_zone,
        'FootprintApproach.enabled': approach,
    }, restore_fails=restore_fails, restore_times_out=restore_times_out)
    smoother = ParameterClient({'max_decel': [-.5, 0.0, -1.5]})
    result = SimpleNamespace(
        last_scan=message, samples={'scan': 10.0}, latest_motion=None,
        precision_parameters=monitor,
        rear_collision_smoother_parameters=smoother,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(clock['ros'] * 1e9))),
        _remembered_blind_points=Mock(return_value=list(memory)),
        engage_emergency_stop=Mock(), request_stop=Mock(), emit=Mock(),
    )
    result._scan_in_base = Mock(return_value=(message, (0., 0., 0.), OUTLINE, list(memory)))
    return result, monitor, clock


@pytest.mark.parametrize('exit_mode', ['normal', 'exception', 'stop'])
def test_profile_restores_approach_for_every_exit(monkeypatch, exit_mode):
    route, monitor, _clock = node(monkeypatch)
    route.stop_requested = False
    with pytest.raises(RuntimeError) if exit_mode == 'exception' else _does_not_raise():
        with module.rear_collision_profile(route) as watch:
            assert monitor.values['StopZone.enabled'] is True
            assert monitor.values['FootprintApproach.enabled'] is False
            assert watch(command()) is None
            if exit_mode == 'exception':
                raise RuntimeError('body failed')
            if exit_mode == 'stop':
                route.stop_requested = True
    assert monitor.values == {
        'StopZone.enabled': True, 'FootprintApproach.enabled': True}
    assert monitor.sets == [
        {'FootprintApproach.enabled': False},
        {'FootprintApproach.enabled': True},
    ]


class _does_not_raise:
    def __enter__(self):
        return None

    def __exit__(self, *_args):
        return False


def test_restore_failure_is_propagated(monkeypatch):
    route, monitor, _clock = node(monkeypatch, restore_fails=True)
    with pytest.raises(RuntimeError, match='restore rejected'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored after rear docking')
    route.request_stop.assert_called_once_with()


def test_restore_timeout_is_fail_closed(monkeypatch):
    route, monitor, clock = node(monkeypatch, restore_times_out=True)

    def advance(_node, timeout_sec):
        clock['mono'] += timeout_sec

    monkeypatch.setattr(module, 'spin_node', advance)
    with pytest.raises(RuntimeError, match='timed out'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored after rear docking')
    route.request_stop.assert_called_once_with()


def test_stale_disabled_profile_is_repaired_then_scoped_run_continues(monkeypatch):
    route, monitor, _clock = node(monkeypatch, approach=False)
    route.stop_requested = False
    with module.rear_collision_profile(route) as watch:
        assert monitor.values['FootprintApproach.enabled'] is False
        assert watch(command()) is None
    assert monitor.values == {
        'StopZone.enabled': True, 'FootprintApproach.enabled': True}
    assert monitor.sets == [
        {'FootprintApproach.enabled': True},
        {'FootprintApproach.enabled': False},
        {'FootprintApproach.enabled': True},
    ]
    route.emit.assert_called_once_with(
        'rear_collision_profile_recovered',
        footprint_approach_enabled=True, stop_zone_enabled=True)
    route.engage_emergency_stop.assert_not_called()
    route.request_stop.assert_not_called()


def test_stale_disabled_profile_repair_failure_is_fail_closed(monkeypatch):
    route, monitor, _clock = node(
        monkeypatch, approach=False, restore_fails=True)
    with pytest.raises(RuntimeError, match='restore rejected'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is False
    route.engage_emergency_stop.assert_called_once_with(
        'FootprintApproach could not be restored before rear docking')
    route.request_stop.assert_called_once_with()


def test_operator_stop_after_baseline_recovery_prevents_profile_disable(monkeypatch):
    route, monitor, _clock = node(monkeypatch, approach=False)
    route.stop_requested = True
    with pytest.raises(RuntimeError, match='operator_stop'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.values['FootprintApproach.enabled'] is True
    assert monitor.sets == [{'FootprintApproach.enabled': True}]


def test_profile_refuses_disabled_stop_zone(monkeypatch):
    route, monitor, _clock = node(monkeypatch, stop_zone=False)
    with pytest.raises(RuntimeError, match='requires StopZone'):
        with module.rear_collision_profile(route):
            pass
    assert monitor.sets == []


@pytest.mark.parametrize('condition,reason', [
    ('missing', 'unavailable'),
    ('receive_stale', 'stale or future-dated'),
    ('header_stale', 'stale or future-dated'),
    ('header_future', 'stale or future-dated'),
])
def test_watch_rejects_unknown_or_stale_scan(monkeypatch, condition, reason):
    route, _monitor, clock = node(monkeypatch)
    with module.rear_collision_profile(route) as watch:
        if condition == 'missing':
            route.last_scan = None
        elif condition == 'receive_stale':
            clock['mono'] = 10.6
        elif condition == 'header_stale':
            clock['ros'] = 10.6
        else:
            clock['ros'] = 9.9
        assert watch(command(-.05)) == f'rear box scan {reason}'


def test_reverse_path_entry_is_detected(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_rotation_corner_collision_is_detected(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, .15)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01, .3)) == 'rear docking swept footprint occupied'


def test_touching_point_is_allowed_only_when_moving_away(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.28, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(.05)) is None
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'


def test_one_centimetre_target_is_allowed_at_fresh_low_speed(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.285, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) is None


def test_larger_live_reverse_speed_sets_prediction_horizon(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, 0.0)])
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) is None
        route.latest_motion = (10.0, .05, 0.0)
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'


def test_larger_live_turn_rate_sets_prediction_arc(monkeypatch):
    route, _monitor, _clock = node(monkeypatch, points=[(-.29, .15)])
    route.latest_motion = (10.0, .01, .3)
    with module.rear_collision_profile(route) as watch:
        assert watch(command(-.01)) == 'rear docking swept footprint occupied'


def test_zero_command_can_stop_with_a_stale_scan(monkeypatch):
    route, _monitor, clock = node(monkeypatch)
    with module.rear_collision_profile(route) as watch:
        clock['mono'] = clock['ros'] = 20.0
        assert watch(command()) is None


def test_new_scan_refreshes_blind_range_memory(monkeypatch):
    route, _monitor, _clock = node(monkeypatch)
    route._remembered_blind_points.return_value = [(-.29, 0.0)]
    with module.rear_collision_profile(route) as watch:
        route.last_scan = scan(stamp_s=10.1)
        route.samples['scan'] = 10.0
        route.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10100000000))
        assert watch(command(-.05)) == 'rear docking swept footprint occupied'
        route._remembered_blind_points.assert_called_once()


def test_runtime_deceleration_must_match_direct_motion(monkeypatch):
    route, _monitor, _clock = node(monkeypatch)
    route.rear_collision_smoother_parameters.values['max_decel'] = [-.4, 0., -1.5]
    with pytest.raises(RuntimeError, match='does not match'):
        with module.rear_collision_profile(route):
            pass
