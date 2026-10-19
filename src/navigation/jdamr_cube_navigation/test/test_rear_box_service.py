"""Rear-box control integration without a camera or hardware motion."""

from contextlib import nullcontext
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import rear_box_service as module
from jdamr_cube_navigation.box_service import BoxServiceRoute
import pytest


GEOMETRY = {
    'frame_length': {'unit': 'm', 'value': 0.34},
    'front_to_wheel_axis': {'unit': 'm', 'value': 0.065},
    'frame_width': {'unit': 'm', 'value': 0.45},
}


@pytest.fixture(autouse=True)
def isolate_collision_profile(monkeypatch):
    monkeypatch.setattr(module, 'rear_collision_profile', lambda _node: nullcontext(None))


def route_node():
    node = object.__new__(BoxServiceRoute)
    node.stop_requested = False
    node._approach_rear_corridor = Mock(return_value=True)
    return node


def face(gap=0.01, angle=0.0, stamp=5.0):
    return {'gap_m': gap, 'normal_yaw_error_rad': angle, 'stamp_s': stamp,
            'face_center_base_xy_m': (-0.275-gap, 0.),
            'normal_base_xy': (math.cos(angle), math.sin(angle)),
            'face_odom': (-0.275-gap, 0.),
            'normal_odom': (math.cos(angle), math.sin(angle))}


@pytest.mark.parametrize('moved', [None, -0.2])
def test_near_skewed_box_uses_straight_clearance_then_rear_feedback(monkeypatch, moved):
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0., 0., 0.), {}))
    observations = [face(.004, -.035), face(.2, -.035), face(.01, 0.)]
    for item in observations:
        item['face_odom'], item['normal_odom'] = (-.29, 0.), (1., 0.)
    node._observe_rear_box = Mock(side_effect=observations)
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._operator_link_ready = Mock(return_value=True)
    node._depart_dock_straight = Mock(return_value=True)
    update = Mock(return_value={'remaining_m': .2})
    node._rear_box_update = Mock(return_value=update)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=moved)
    node._camera = Mock()
    node.emit = Mock()
    monkeypatch.setattr(module, 'load_route', lambda _p: {'waypoints': [
        {'id': 'home_exit'}, {'id': 'observation'}]})
    assert node.park_rear_box(GEOMETRY, 'route', execute=True, timeout_s=90) is (moved is not None)
    node._depart_dock_straight.assert_called_once_with(.2)
    node._drive_zero_turn.assert_called_once_with(
        -.2, precision=True, hold_heading=True, target_update=update, approach_time_s=.2)
    node._camera.assert_not_called()
    node.capture_stationary_pose.assert_not_called()
    assert node._observe_rear_box.call_count == (2 if moved is None else 3)


def test_terminal_corner_miss_gets_one_clearance_and_same_target_reapproach(monkeypatch):
    node = route_node()
    node._final_gap_m = Mock(return_value=.05)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0., 0., 0.), {}))
    initial = face(.58, 0., stamp=1.)
    missed = face(.01085, math.radians(.548), stamp=2.)
    cleared = face(.21, math.radians(.1), stamp=3.)
    confirmed = face(.01, 0., stamp=4.)
    for item in (initial, missed, cleared, confirmed):
        item['face_odom'], item['normal_odom'] = (-.855, 0.), (1., 0.)
    node._observe_rear_box = Mock(side_effect=[initial, missed, cleared, confirmed])
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._operator_link_ready = Mock(return_value=True)
    node._depart_dock_straight = Mock(return_value=True)
    updates = [Mock(return_value={'remaining_m': .57}),
               Mock(return_value={'remaining_m': .20})]
    node._rear_box_update = Mock(side_effect=updates)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(side_effect=[-.57, -.20])
    node.emit = Mock()
    monkeypatch.setattr(module, 'load_route', lambda _p: {'waypoints': [
        {'id': 'home_exit'}, {'id': 'observation'}]})

    assert node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90, target_gap_m=.01)
    node._final_gap_m.assert_not_called()
    node._depart_dock_straight.assert_called_once_with(.2)
    assert node._drive_zero_turn.call_count == 2
    assert node._rear_box_update.call_args_list[1].kwargs['reference'] is initial
    assert node.confirmation == {
        'confirmed': True, 'reason': 'confirmed',
        'reference': 'live_rear_lidar_face',
        'nearest_gap_m': pytest.approx(.01),
        'farthest_gap_m': pytest.approx(.01),
        'internal_estimate_only': True,
        'external_accuracy_verified': False,
    }
    retry = [call for call in node.emit.call_args_list
             if call.args and call.args[0] == 'rear_box_terminal_retry_started']
    assert len(retry) == 1


def test_terminal_miss_does_not_retry_after_operator_link_loss():
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    node.verify_live_maps = Mock()
    node.wait_until_ready = Mock(return_value=True)
    node.capture_stationary_pose = Mock(return_value=((0., 0., 0.), {}))
    initial = face(.2, 0., stamp=1.)
    missed = face(.01085, math.radians(.548), stamp=2.)
    for item in (initial, missed):
        item['face_odom'], item['normal_odom'] = (-.475, 0.), (1., 0.)
    node._observe_rear_box = Mock(side_effect=[initial, missed])
    node._reverse_smoother_ready = Mock(return_value=True)
    node._parking_parameters_ready = Mock(return_value=True)
    node._confirm_collision_monitor_on = Mock()
    node._operator_link_ready = Mock(side_effect=[True, True, False])
    node._depart_dock_straight = Mock(return_value=True)
    update = Mock(return_value={'remaining_m': .19})
    node._rear_box_update = Mock(return_value=update)
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._drive_zero_turn = Mock(return_value=-.19)
    node.emit = Mock()

    assert not node.park_rear_box(
        GEOMETRY, 'route', execute=True, timeout_s=90)
    node._depart_dock_straight.assert_not_called()
    node._rear_box_update.assert_called_once()
    assert node._observe_rear_box.call_count == 2
    assert node.confirmation['reason'] == 'rear_box_gap_out_of_tolerance'


def test_far_corridor_moves_before_any_precision_face_is_required():
    node = route_node()
    del node._approach_rear_corridor
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(side_effect=[
        {'gap_m': 1.45}, {'gap_m': .95}, {'gap_m': .5}])
    node._rear_coarse_step = Mock(return_value=True)
    node._rear_box_observation = Mock(side_effect=AssertionError('fine face not required yet'))
    node.run_deadline_s = time.monotonic() + 10.
    node.emit = Mock()
    assert node._approach_rear_corridor(GEOMETRY, .275, .225)
    assert [call.args[3] for call in node._rear_coarse_step.call_args_list] == (
        pytest.approx([.5, .46]))
    assert all(call.args[2] == pytest.approx(.275)
               for call in node._rear_coarse_step.call_args_list)
    node._rear_box_observation.assert_not_called()


@pytest.mark.parametrize('clear', [True, False])
def test_coarse_reverse_validates_static_path_before_command(monkeypatch, clear):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._drive_zero_turn = Mock(return_value=-.49)
    node.emit = Mock()
    check = Mock(return_value=clear)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock(return_value=check))
    if not clear:
        with pytest.raises(RuntimeError, match='static corridor blocked'):
            node._rear_coarse_step(GEOMETRY, .275, .275, .5)
        node._drive_zero_turn.assert_not_called()
        return
    assert node._rear_coarse_step(GEOMETRY, .275, .275, .5)
    path = check.call_args.args[0]
    assert path[0] == pytest.approx((0., 0., 0.))
    assert path[-1] == pytest.approx((-.5, 0., 0.))
    assert node._drive_zero_turn.call_args.args[0] == -.5


def test_coarse_range_reaches_handoff_without_a_plane(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[(-.28, -.275), (.07, .275)])
    node._rear_corridor_observation = Mock(return_value={'gap_m': .49})
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator',
                        Mock(return_value=Mock(return_value=True)))
    def drive(_distance, **kwargs):
        reason = kwargs['watch']()
        assert reason == 'rear box coarse handoff ready'
        node._last_direct_motion_failure = reason
        return None
    node._drive_zero_turn = Mock(side_effect=drive)
    assert node._rear_coarse_step(GEOMETRY, .275, .275, .5)


def test_temporary_face_loss_waits_in_same_task_then_recovers():
    node = route_node()
    node.run_deadline_s = time.monotonic() + 10.
    node._operator_link_ready = Mock(return_value=True)
    observed = face(.4)
    node._observe_rear_box = Mock(side_effect=[
        RuntimeError('rear box observation unavailable: rear box unstable'), observed])
    node.emit = Mock()
    assert node._observe_rear_box_recovering(GEOMETRY, .275, .225) is observed
    assert node._observe_rear_box.call_count == 2
    assert node.emit.call_args.args[0] == 'rear_box_waiting_for_observation'


@pytest.mark.parametrize('target_gap', [True, 0., .009, .101, math.inf])
def test_explicit_rear_target_gap_reuses_parking_contract_bounds(target_gap):
    node = route_node()
    node._final_gap_m = Mock(return_value=.01)
    with pytest.raises(ValueError, match=r'\[0.01, 0.10\]'):
        node.park_rear_box(
            GEOMETRY, 'route', execute=False, timeout_s=90,
            target_gap_m=target_gap)
    node._final_gap_m.assert_not_called()


def test_reverse_preview_rotates_heading_without_rotating_actual_footprint(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    outline = [(-.28, -.23), (-.248, -.275), (.038, -.275), (.12, -.23),
               (.12, .23), (.038, .275), (-.248, .275), (-.28, .23)]
    node._footprint_outline = Mock(return_value=outline)
    node._reverse_footprint = Mock(side_effect=AssertionError('bounding box overstates corners'))
    node._rear_box_observation = Mock(return_value=face(.2))
    node.emit = Mock()
    check = Mock(return_value=True)
    factory = Mock(return_value=check)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', factory)
    update = node._rear_box_update(face(.2), GEOMETRY, .275, .225, .01)
    result = update((0., 0., 0.))
    result['validate_control']((0., 0., 0.), 0.)
    poses = check.call_args.args[0]
    assert poses[0] == pytest.approx((0., 0., 0.))
    assert poses[-1][0] < -.1
    assert all(abs(p[2]) < 1e-8 for p in poses)
    assert result['remaining_m'] == pytest.approx(.19)
    assert factory.call_args.args[2] == outline
    node._reverse_footprint.assert_not_called()


@pytest.mark.parametrize('age,frame', [(1., 'laser_link'), (0., 'wrong_frame')])
def test_bad_scan_stops_before_fitting_or_any_motion(monkeypatch, age, frame):
    node = route_node()
    node.last_scan = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=5, nanosec=0), frame_id=frame))
    node.samples = {'scan': 5.0}
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5e9))
    monkeypatch.setattr(module.time, 'monotonic', lambda: 5.0+age)
    fit = Mock()
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    with pytest.raises(RuntimeError):
        node._rear_box_observation(GEOMETRY, .275, .225)
    fit.assert_not_called()


@pytest.mark.parametrize('change', ['position', 'normal', 'clock'])
def test_live_scan_cannot_switch_target_or_regress_time(monkeypatch, change):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    next_face = face(.2, stamp=6.)
    if change == 'position':
        next_face['face_odom'] = (-1., 0.)
    elif change == 'normal':
        next_face['normal_odom'] = (0., 1.)
    else:
        next_face['stamp_s'] = 4.
    node._rear_box_observation = Mock(return_value=next_face)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(face(.2), GEOMETRY, .275, .225, .01)
    with pytest.raises(RuntimeError):
        update((0., 0., 0.))


@pytest.mark.parametrize('change', ['position', 'normal'])
def test_small_cumulative_changes_cannot_move_the_latched_target(monkeypatch, change):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    node.emit = Mock()
    observations = []
    for i in (1, 2):
        item = face(.2, stamp=5.+i)
        if change == 'position':
            item['face_odom'] = (-.475 - .039*i, 0.)
        else:
            angle = math.radians(7.*i)
            item['normal_odom'] = (math.cos(angle), math.sin(angle))
        observations.append(item)
    node._rear_box_observation = Mock(side_effect=observations)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(face(.2), GEOMETRY, .275, .225, .01)
    update((0., 0., 0.))
    with pytest.raises(RuntimeError, match='target changed'):
        update((0., 0., 0.))


def test_final_confirmation_rejects_a_different_box_at_the_right_gap():
    with pytest.raises(RuntimeError, match='target changed'):
        module._same_target(face(.2), {**face(.01), 'face_odom': (-1., 0.)})


def test_one_noisy_close_range_scan_does_not_interrupt_an_aligned_approach(monkeypatch):
    node = route_node()
    node.registry = {'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/mask.yaml'}}
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node._frozen_in_odom = Mock(return_value=(None, lambda x, y, yaw: (x, y, yaw)))
    node._footprint_outline = Mock(return_value=[])
    node._rear_box_observation = Mock(return_value=face(
        .0274, -.0053988349885327755, stamp=5.1))
    node.emit = Mock()
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock())
    update = node._rear_box_update(face(.0274), GEOMETRY, .275, .225, .01)
    result = update((0., 0., 0.))
    result['validate_control']((0., 0., 0.), result['heading_rad'])
    assert abs(result['heading_rad']) < math.radians(.1)
    assert 0.016 < result['remaining_m'] < 0.018
    repeated = update((0., 0., 0.))
    assert repeated['heading_rad'] == result['heading_rad']
    assert repeated['remaining_m'] == result['remaining_m']


def test_stopped_confirmation_uses_all_eight_scans_not_the_last_angle_outlier():
    observations = [face(.01, 0., stamp=5.+i*.1) for i in range(7)]
    observations.append(face(.01, math.radians(.31), stamp=5.7))
    result = module._stationary_face(observations, .275, .225)
    assert result['averaged_frames'] == 8
    assert math.degrees(result['normal_yaw_error_rad']) == pytest.approx(.03875, abs=1e-6)
    far_gap = result['gap_m'] + .45 * abs(math.sin(result['normal_yaw_error_rad']))
    assert .007 <= result['gap_m'] <= far_gap <= .013
    normal = result['normal_base_xy']
    point = result['face_center_base_xy_m']
    assert sum(normal[i]*point[i] for i in (0, 1)) == pytest.approx(result['line_offset_m'])
