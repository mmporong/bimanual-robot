"""Mid-drive stop paths: late depth frames, gyro spikes and the Nav2 102 retry."""

import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped, Twist
from jdamr_cube_navigation import box_service, corridor_route, restaurant_service
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.corridor_route import CorridorRoute
from jdamr_cube_navigation.dedicated_tf import DedicatedTfListener
from jdamr_cube_navigation.restaurant_service import ServiceRoute
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException


def _clock(stamp_s):
    return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(stamp_s * 1e9)))


def _transform():
    return SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))


_TABLE_TARGET = {
    'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
    'face_center_map_xy_m': [.50, 0.], 'outward_normal_map_xy': [-1., 0.],
    'observed_face_span_m': .6,
}


def _table_node(monkeypatch, now_s, camera_alive=True):
    node = object.__new__(BoxServiceRoute)
    node.parking_contract = {'robot_base_frame': 'base_link'}
    node.station_reference = None
    node.station_reference_mode = None
    node._table_observation_context = ({}, (.50, 0.), .6)
    node._table_approach_validator = Mock(return_value=Mock())
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node.box_status = {
        'stamp_s': 9.0, 'detected': True, 'stable': True,
        'surface_kind': 'table_edge', 'frame_id': 'camera_color_optical_frame',
        'control_ready': False, 'width_m': .6,
    }
    node.box_status_received_s = time.monotonic()
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(transform=_transform())
    node.parking_tf.can_transform.return_value = True
    node.get_clock = Mock(return_value=_clock(now_s))
    node._fresh_depth_frame_after = Mock(return_value=camera_alive)
    node._final_gap_m = Mock(return_value=.01)
    node.box_front_half_width_m = .225
    node.observer_stable_frames = 8
    node.stop_requested = False
    node.emit = Mock()
    monkeypatch.setattr(box_service, 'spin_node', lambda *_a, **_k: None)
    monkeypatch.setattr(box_service, 'compute_table_edge_docking_target', Mock())
    target = dict(_TABLE_TARGET)
    update = node._table_approach_update(target, .065, lambda x, y, yaw: (x, y, yaw))
    # A newer frame whose processing finished 0.55 s after its stamp.
    node.box_status = {**node.box_status, 'stamp_s': now_s - .55}
    node.box_status_received_s = time.monotonic()
    return node, update


def test_late_new_depth_frame_carries_the_held_face_instead_of_stopping(monkeypatch):
    node, update = _table_node(monkeypatch, now_s=10.0)
    control = update((0., 0., 0.))
    assert control['remaining_m'] == pytest.approx(.425)
    # The late frame is never consumed: no new target, no face update.
    box_service.compute_table_edge_docking_target.assert_not_called()
    events = [call.args[0] for call in node.emit.call_args_list]
    assert 'table_approach_late_frame_skipped' in events
    assert 'table_approach_face_updated' not in events


def test_late_frame_still_stops_once_the_held_input_exceeds_observer_silence(monkeypatch):
    _node, update = _table_node(monkeypatch, now_s=10.6)
    with pytest.raises(RuntimeError, match='table approach depth stale'):
        update((0., 0., 0.))


def test_late_frame_still_stops_without_a_live_depth_stream(monkeypatch):
    _node, update = _table_node(monkeypatch, now_s=10.0, camera_alive=False)
    with pytest.raises(RuntimeError, match='table approach depth stale'):
        update((0., 0., 0.))


def _clocked_table_node(monkeypatch, clock):
    node, _update = _table_node(monkeypatch, now_s=clock[0])
    node.get_clock = lambda: _clock(clock[0])
    target = dict(_TABLE_TARGET)
    node.box_status = {**node.box_status, 'stamp_s': 9.0}
    update = node._table_approach_update(target, .065, lambda x, y, yaw: (x, y, yaw))
    return node, update


def test_one_event_per_distinct_late_frame(monkeypatch):
    clock = [10.0]
    node, update = _clocked_table_node(monkeypatch, clock)
    node.box_status = {**node.box_status, 'stamp_s': 9.45}
    update((0., 0., 0.))
    clock[0] = 10.05
    update((0., 0., 0.))
    events = [call for call in node.emit.call_args_list
              if call.args == ('table_approach_late_frame_skipped',)]
    assert len(events) == 1 and events[0].kwargs['skipped'] == 1


def test_late_frame_is_not_carried_on_a_held_face_older_than_observer_silence(monkeypatch):
    clock = [10.0]
    node, update = _clocked_table_node(monkeypatch, clock)
    # A fresh frame without a face is consumed; the held face keeps stamp 9.0.
    node.box_status = {'stamp_s': 9.9, 'detected': False, 'stable': False,
                       'reason': 'no_table_edge_candidate',
                       'frame_id': 'camera_color_optical_frame', 'control_ready': False}
    update((0., 0., 0.))
    clock[0] = 10.7
    node.box_status = {**node.box_status, 'stamp_s': 10.15, 'detected': True, 'stable': True,
                       'surface_kind': 'table_edge', 'reason': 'stable_detection'}
    with pytest.raises(RuntimeError, match='table approach depth stale'):
        update((0., 0., 0.))


def test_frame_that_ages_out_while_its_tf_arrives_is_skipped_as_late(monkeypatch):
    clock = [10.0]
    node, update = _clocked_table_node(monkeypatch, clock)
    node.box_status = {**node.box_status, 'stamp_s': 9.55}   # 0.45 s old: fresh on read
    node.parking_tf.can_transform.return_value = False

    def lagging_spin(*_args, **_kwargs):
        clock[0] = 10.06   # the frame is 0.51 s old once the TF wait ends

    monkeypatch.setattr(box_service, 'spin_node', lagging_spin)
    control = update((0., 0., 0.))
    assert control['remaining_m'] == pytest.approx(.425)
    box_service.compute_table_edge_docking_target.assert_not_called()
    assert any(call.args == ('table_approach_late_frame_skipped',)
               for call in node.emit.call_args_list)


def _odom_route():
    route = object.__new__(CorridorRoute)
    route.parking_contract = {'stopped_linear_mps': .01, 'stopped_angular_radps': .02}
    route.parking_motion_revision = 0
    route.samples = {}
    route.odom_last_pose = None
    route.odom_total_distance_m = 0.0
    route.amcl_motion_distance_m = 0.0
    route.amcl_motion_rotation_rad = 0.0
    return route


def _odom(angular_radps=0.0, linear_mps=0.0):
    message = Odometry()
    message.twist.twist.linear.x = linear_mps
    message.twist.twist.angular.z = angular_radps
    return message


def _feed(monkeypatch, route, samples, period_s=1.0 / 30.0):
    clock = {'now_s': 100.0}
    monkeypatch.setattr(
        'jdamr_cube_navigation.corridor_route.time.monotonic', lambda: clock['now_s'])
    for message in samples:
        route._odom_callback(message)
        clock['now_s'] += period_s


def test_single_gyro_sample_above_stop_limit_is_not_motion(monkeypatch):
    route = _odom_route()
    _feed(monkeypatch, route, [_odom(), _odom(.021), _odom(), _odom(-.021), _odom()])
    assert route.parking_motion_revision == 0


def test_sustained_yaw_rate_is_motion_after_confirm_window(monkeypatch):
    route = _odom_route()
    _feed(monkeypatch, route, [_odom(.03)] * 4)  # 0, 33, 67, 100 ms
    assert route.parking_motion_revision >= 1


def test_fast_yaw_spike_still_counts_at_once(monkeypatch):
    route = _odom_route()
    _feed(monkeypatch, route, [_odom(), _odom(.05), _odom()])
    assert route.parking_motion_revision == 1


def test_translation_and_non_finite_still_count_at_once(monkeypatch):
    route = _odom_route()
    _feed(monkeypatch, route, [_odom(linear_mps=.02)])
    assert route.parking_motion_revision == 1
    _feed(monkeypatch, route, [_odom(math.nan)])
    assert route.parking_motion_revision == 2


def test_nonzero_command_still_counts_at_once():
    route = _odom_route()
    command = Twist()
    command.angular.z = .001
    route._parking_command_callback(command)
    assert route.parking_motion_revision == 1


def test_controller_tf_error_is_a_bounded_input_gap():
    route = object.__new__(ServiceRoute)
    route.stop_requested = False
    route._retry_blocked_reason = route._retry_blocked_code = route._retry_guard_reason = None
    route._tag_blocked(102, False)
    assert route._retry_blocked_reason is None
    assert route._retry_guard_reason == 'nav2 transform error 102'
    assert ServiceRoute._input_gap_recoverable(route._retry_guard_reason)
    route._retry_guard_reason = None
    route._tag_blocked(102, True)
    assert route._retry_guard_reason == 'nav2 transform error 102'
    route.stop_requested = True
    route._retry_guard_reason = None
    route._tag_blocked(102, False)
    assert route._retry_guard_reason is None


def test_retry_loop_resends_once_after_controller_tf_error():
    route = object.__new__(ServiceRoute)
    route.stop_requested = False
    attempts = []

    def attempt():
        attempts.append(1)
        route._tag_blocked(102, False)
        return False

    route._wait_for_input_recovery = Mock(return_value=True)
    route._escape_blocked = Mock()
    assert route._run_with_input_recovery(attempt) is False
    assert len(attempts) == 2
    route._wait_for_input_recovery.assert_called_once_with('nav2 transform error 102')


def _recovery_route(monkeypatch, stamps):
    route = object.__new__(ServiceRoute)
    route.stop_requested = False
    route.emit = Mock()
    route.get_clock = Mock(return_value=_clock(50.0))
    monkeypatch.setattr(CorridorRoute, '_wait_for_input_recovery',
                        lambda self, reason, timeout_s=8.0: True)
    monkeypatch.setattr(restaurant_service, 'spin_node', lambda *_a, **_k: None)
    replies = iter(stamps)

    def lookup(*_args):
        stamp_s = next(replies)
        if stamp_s is None:
            raise TransformException('no map->odom yet')
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
            sec=int(stamp_s), nanosec=int(round((stamp_s % 1) * 1e9)))))

    route.parking_tf = SimpleNamespace(lookup_transform=Mock(side_effect=lookup))
    return route


def test_controller_tf_retry_waits_for_a_map_to_odom_publish_after_the_stall(monkeypatch):
    # AMCL future-dates map->odom; stamps behind the ROS time are the stalled ones.
    route = _recovery_route(monkeypatch, [49.2, None, 49.9, 50.8])
    assert route._wait_for_input_recovery('nav2 transform error 102', timeout_s=5.0)
    assert route.parking_tf.lookup_transform.call_count == 4


def test_controller_tf_retry_gives_up_while_map_to_odom_stays_stalled(monkeypatch):
    route = _recovery_route(monkeypatch, [49.2] * 10_000)
    clock = {'now_s': 0.0}

    def monotonic():
        clock['now_s'] += .05
        return clock['now_s']

    monkeypatch.setattr(restaurant_service.time, 'monotonic', monotonic)
    assert not route._wait_for_input_recovery('nav2 transform error 102', timeout_s=1.0)
    assert route.emit.call_args.args == ('nav2_transform_recovery_timeout',)


def test_other_input_gaps_do_not_wait_for_map_to_odom(monkeypatch):
    route = _recovery_route(monkeypatch, [])
    assert route._wait_for_input_recovery('scan stale: 0.9s', timeout_s=1.0)
    route.parking_tf.lookup_transform.assert_not_called()


def test_route_tf_runs_on_a_dedicated_listener(monkeypatch):
    created = []

    class Listener:
        def __init__(self, buffer, owner, name):
            created.append((buffer, owner, name))
            self.shutdown = Mock()

    monkeypatch.setattr(corridor_route, 'DedicatedTfListener', Listener)
    route = object.__new__(CorridorRoute)
    route._start_parking_tf()
    assert created == [(route.parking_tf, route, 'jdamr_corridor_route_tf')]
    listener = route.parking_tf_listener
    route._stop_parking_tf()
    listener.shutdown.assert_called_once()
    route._stop_parking_tf()   # idempotent
    listener.shutdown.assert_called_once()


def test_dedicated_tf_listener_fills_the_buffer_without_spinning_the_owner():
    context = rclpy.Context()
    # A launch-wide __node remap renames the owner, never the TF node.
    rclpy.init(context=context, args=['test', '--ros-args', '-r', '__node:=owner_renamed'])
    try:
        owner = rclpy.create_node('owner', context=context)
        publisher = rclpy.create_node('tf_publisher_probe', context=context,
                                      cli_args=['--ros-args', '-r', '__node:=tf_publisher_probe'])
        buffer = Buffer()
        listener = DedicatedTfListener(buffer, owner, 'owner_tf_probe')
        assert owner.get_name() == 'owner_renamed'
        assert listener.node.get_name() == 'owner_tf_probe'
        assert listener.thread.daemon is True
        broadcaster = TransformBroadcaster(publisher)
        delivered = False
        for _ in range(100):
            message = TransformStamped()
            message.header.stamp = publisher.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
            message.transform.rotation.w = 1.0
            broadcaster.sendTransform(message)
            time.sleep(.02)    # the owner is never spun
            if buffer.can_transform('odom', 'base_link', Time()):
                delivered = True
                break
        assert delivered
        listener.shutdown()
        assert not listener.thread.is_alive()
        publisher.destroy_node()
        owner.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_direct_motion_stop_event_reports_the_longest_command_gap(monkeypatch):
    route = object.__new__(ServiceRoute)
    route.stop_requested = False
    published = []
    route._escape_publisher = lambda: SimpleNamespace(publish=published.append)
    guards = iter([None, 'scan stale: 1.0s'])
    route._guard_failure = lambda _amcl: next(guards, 'scan stale: 1.0s')
    route._operator_link_ready = lambda: True
    route.emit = Mock()
    route.parking_contract = {'stopped_linear_mps': .01, 'stopped_angular_radps': .02}
    route.latest_motion = None
    monkeypatch.setattr(restaurant_service, 'spin_node', lambda *_a, **_k: None)
    command = Twist()
    command.linear.x = .05
    route._direct_motion(command, lambda _value: False, lambda: 0.0,
                         time.monotonic() + 5.0, 'straight_drive_stopped')
    stopped = next(call for call in route.emit.call_args_list
                   if call.args == ('straight_drive_stopped',))
    assert stopped.kwargs['reason'] == 'scan stale: 1.0s'
    # Commands are republished every 50 ms; the smoother stops after 0.5 s.
    assert .04 <= stopped.kwargs['max_publish_gap_s'] < .5
