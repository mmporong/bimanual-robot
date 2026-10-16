"""Rear-box control integration without a camera or hardware motion."""

import math
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


def face(gap=0.01, angle=0.0, stamp=5.0):
    return {'gap_m': gap, 'normal_yaw_error_rad': angle, 'stamp_s': stamp,
            'face_center_base_xy_m': (-0.275-gap, 0.),
            'normal_base_xy': (math.cos(angle), math.sin(angle)),
            'face_odom': (-0.275-gap, 0.),
            'normal_odom': (math.cos(angle), math.sin(angle))}


@pytest.mark.parametrize('moved', [None, -0.2])
def test_near_skewed_box_uses_straight_clearance_then_rear_feedback(monkeypatch, moved):
    node = object.__new__(BoxServiceRoute)
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
        -.2, precision=True, hold_heading=True, target_update=update)
    node._camera.assert_not_called()
    node.capture_stationary_pose.assert_called_once()
    assert node._observe_rear_box.call_count == (2 if moved is None else 3)


def test_reverse_preview_rotates_heading_without_rotating_actual_footprint(monkeypatch):
    node = object.__new__(BoxServiceRoute)
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
    node = object.__new__(BoxServiceRoute)
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
    node = object.__new__(BoxServiceRoute)
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
    node = object.__new__(BoxServiceRoute)
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
    node = object.__new__(BoxServiceRoute)
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
