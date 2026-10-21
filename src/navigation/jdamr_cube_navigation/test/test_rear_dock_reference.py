"""Reproduce return drift against a landmark learned during the same dock exit."""

from copy import deepcopy
import math
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import activate_navigation
from jdamr_cube_navigation import rear_box_service as module
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.restaurant_service import ServiceRoute
import pytest


GEOMETRY = {
    'frame_length': {'unit': 'm', 'value': .34},
    'front_to_wheel_axis': {'unit': 'm', 'value': .065},
    'frame_width': {'unit': 'm', 'value': .45},
}


def observed_at(pose, *, face_map=(-.285, .02), normal_yaw=0.):
    x_m, y_m, yaw_rad = pose
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    dx, dy = face_map[0] - x_m, face_map[1] - y_m
    midpoint = cosine * dx + sine * dy, -sine * dx + cosine * dy
    angle = normal_yaw - yaw_rad
    normal = math.cos(angle), math.sin(angle)
    return {'face_midpoint_base_xy_m': midpoint, 'normal_base_xy': normal,
            'normal_yaw_error_rad': angle, 'span_m': .35,
            'observation_roi_clipped': False, 'support_count': 40,
            'base_pose_odom': pose}


def node_with_reference(*, start=(0., 0., 0.), exit_pose=(.7, 0., 0.), normal_yaw=0.):
    node = object.__new__(BoxServiceRoute)
    node._rear_dock_reference = {
        'observation': observed_at(exit_pose, normal_yaw=normal_yaw),
        'pose_map': dict(zip(('x_m', 'y_m', 'yaw_rad'), exit_pose)),
        'start_pose_map': list(start), 'exit_pose_map': list(exit_pose),
    }
    node.home_rear_geometry = GEOMETRY
    node._odom_pose = Mock(return_value=exit_pose)
    node.stop_requested = False
    node._operator_link_ready = Mock(return_value=True)
    node._guard_failure = Mock(return_value=None)
    node.registry = {'frame_id': 'map', 'home': {'rear_parking_reference': 'lidar_box'},
                     'map': {'yaml_path': '/map.yaml'},
                     'keepout': {'yaml_path': '/keepout.yaml'}}
    node.emit = Mock()
    return node


def test_stage_retains_outbound_origin_tangent_and_observed_face_normal():
    node = node_with_reference(start=(.1, -.08, .2), exit_pose=(.8, -.08, .2),
                               normal_yaw=.1)
    stage = node._rear_reference_stage({'id': 'home_approach', 'x': 9., 'y': 9., 'yaw': 2.})
    normal = math.cos(.1), math.sin(.1)
    tangent = -normal[1], normal[0]
    displacement = stage['x'] - .1, stage['y'] + .08
    assert sum(displacement[i] * tangent[i] for i in (0, 1)) == pytest.approx(0.)
    assert math.hypot(*displacement) == pytest.approx(.7 * math.cos(.1))
    assert stage['yaw'] == pytest.approx(.1)


def test_departure_teaches_once_and_clearance_cannot_replace_reference(monkeypatch):
    node = node_with_reference()
    del node._rear_dock_reference
    node.config = {'dock_departure_distance_m': .7}
    node._dock_departure = {'start': (0., 0., 0.), 'end': (.7, 0., 0.)}
    observation = observed_at((.7, 0., 0.))
    node._rear_box_observation = Mock(return_value=observation)
    parent = Mock(return_value=True)
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', parent)
    assert node._depart_dock_straight(.7)
    saved = deepcopy(node._rear_dock_reference)
    assert node._depart_dock_straight(.2)
    assert node._depart_dock_straight(.7)
    assert node._rear_dock_reference == saved
    assert node._rear_box_observation.call_count == 2
    assert parent.call_count == 3


def test_unavailable_reference_does_not_delay_or_repeat_a_completed_exit(monkeypatch):
    node = node_with_reference()
    del node._rear_dock_reference
    node.config = {'dock_departure_distance_m': .7}
    node._rear_box_observation = Mock(side_effect=RuntimeError('rear box observation unavailable'))
    node._observe_rear_box = Mock(side_effect=AssertionError('no extra stopped observation dwell'))
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', Mock(return_value=True))
    assert node._depart_dock_straight(.7)
    assert node._rear_dock_reference is None
    assert node.emit.call_args.args[0] == 'rear_dock_reference_unavailable'
    node._observe_rear_box.assert_not_called()


def test_reference_uses_scan_stamped_pose_instead_of_later_stopped_exit(monkeypatch):
    node = node_with_reference()
    del node._rear_dock_reference
    node.config = {'dock_departure_distance_m': .7}
    node._dock_departure = {'start': (0., 0., 0.), 'end': (.7, 0., 0.)}
    node._rear_box_observation = Mock(return_value=observed_at((.66, 0., 0.)))
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', Mock(return_value=True))
    assert node._depart_dock_straight(.7)
    reference = node._rear_dock_reference
    assert reference['pose_map'] == pytest.approx({'x_m': .66, 'y_m': 0., 'yaw_rad': 0.})
    assert reference['exit_pose_map'] == [.7, 0., 0.]
    node._rear_dock_reference = reference
    measured = node._rear_reference_pose(observed_at((.4, -.25, .2)))
    assert tuple(measured[k] for k in ('x_m', 'y_m', 'yaw_rad')) == pytest.approx((.4, -.25, .2))


def test_clipped_exit_view_preserves_fresh_same_run_dock_observation(monkeypatch):
    node = node_with_reference()
    del node._rear_dock_reference
    node.config = {'dock_departure_distance_m': .7}
    node._dock_departure = {'start': (0., 0., 0.), 'end': (.7, 0., 0.)}
    clipped = observed_at((.7, 0., 0.))
    clipped['observation_roi_clipped'] = True
    node._rear_box_observation = Mock(side_effect=[observed_at((0., 0., 0.)), clipped])
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', Mock(return_value=True))
    assert node._depart_dock_straight(.7)
    assert node._rear_dock_reference['observation_phase'] == 'before_straight_departure'
    assert node._rear_dock_reference['pose_map'] == pytest.approx(
        {'x_m': 0., 'y_m': 0., 'yaw_rad': 0.})


def test_small_central_fragment_is_not_taught_as_a_dock_landmark(monkeypatch):
    node = node_with_reference()
    del node._rear_dock_reference
    node.config = {'dock_departure_distance_m': .7}
    node._dock_departure = {'start': (0., 0., 0.), 'end': (.7, 0., 0.)}
    fragment = observed_at((.7, 0., 0.))
    fragment.update(support_count=5, span_m=.09)
    node._rear_box_observation = Mock(return_value=fragment)
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', Mock(return_value=True))
    assert node._depart_dock_straight(.7)
    assert getattr(node, '_rear_dock_reference', None) is None
    assert node.emit.call_args.kwargs['reason'] == 'rear dock reference support is too small'


def test_new_outbound_exit_discards_prior_reference_even_when_new_observation_fails(monkeypatch):
    node = node_with_reference()
    node.config = {'dock_departure_distance_m': .7}
    node._dock_departure = None
    node._rear_box_observation = Mock(side_effect=RuntimeError('rear box observation unavailable'))
    monkeypatch.setattr(ServiceRoute, '_depart_dock_straight', Mock(return_value=True))
    assert node._depart_dock_straight(.7)
    assert node._rear_dock_reference is None


@pytest.mark.parametrize('clipped,span', [(True, .35), (False, .9)])
def test_cropped_or_other_width_face_never_seeds_localization(clipped, span):
    node = node_with_reference()
    observation = observed_at((.6, .3, -.2))
    observation.update(observation_roi_clipped=clipped, span_m=span)
    with pytest.raises(RuntimeError, match='clipped|width changed'):
        node._rear_reference_pose(observation)


def test_return_drift_corrects_measured_pose_then_replans_same_stage(monkeypatch):
    node = node_with_reference()
    stage = node._rear_reference_stage({'id': 'home_approach'})
    physical = (.4, -.25, math.radians(17.))
    node._observe_rear_box = Mock(side_effect=[observed_at(physical), observed_at((.7, 0., 0.))])
    node.capture_stationary_pose = Mock(side_effect=[((.7, 0., 0.), {}), (physical, {})])
    initialize = Mock()
    monkeypatch.setattr(activate_navigation, 'initialize_localization', initialize)
    node._plan_with_input_recovery = Mock(return_value={'ok': True})
    node._reach_staging = Mock(return_value=(.7, 0., 0.))
    node._center_rear_reference = Mock(return_value=True)
    assert node._align_rear_reference(stage, {'id': 'home', 'x_m': 0., 'y_m': 0., 'yaw_rad': 0.})
    seed = initialize.call_args.args[1]
    assert tuple(seed[k] for k in ('x_m', 'y_m', 'yaw_rad')) == pytest.approx(physical)
    # The inferred robot pose is not replaced with the registered dock origin.
    assert seed['x_m'] != 0.
    goal = node._plan_with_input_recovery.call_args.args[0]
    assert (goal['x_m'], goal['y_m'], goal['yaw_rad']) == pytest.approx((.7, 0., 0.))
    node._center_rear_reference.assert_called_once()


def test_other_same_width_face_after_replanning_cannot_start_centering(monkeypatch):
    node = node_with_reference()
    stage = node._rear_reference_stage({'id': 'home_approach'})
    physical = (.4, -.25, math.radians(17.))
    node._observe_rear_box = Mock(side_effect=[observed_at(physical), observed_at((2.2, 0., 0.))])
    node.capture_stationary_pose = Mock(side_effect=[((.7, 0., 0.), {}), (physical, {})])
    monkeypatch.setattr(activate_navigation, 'initialize_localization', Mock())
    node._plan_with_input_recovery = Mock(return_value={'ok': True})
    node._reach_staging = Mock(return_value=(.7, 0., 0.))
    node._make_reverse_path = Mock()
    node._execute_reverse_once = Mock()
    with pytest.raises(RuntimeError, match='disagrees with return region'):
        node._align_rear_reference(stage, {'id': 'home', 'x_m': 0., 'y_m': 0., 'yaw_rad': 0.})
    node._make_reverse_path.assert_not_called()
    node._execute_reverse_once.assert_not_called()


def test_near_landmark_localizes_and_centers_without_backtracking_to_far_stage(monkeypatch):
    node = node_with_reference()
    physical = (.4, -.08, math.radians(4.))
    observed = observed_at(physical)
    node._observe_rear_box = Mock(side_effect=AssertionError('reuse the matched face'))
    node.capture_stationary_pose = Mock(side_effect=[((.45, 0., 0.), {}), (physical, {})])
    initialize = Mock()
    monkeypatch.setattr(activate_navigation, 'initialize_localization', initialize)
    node._plan_with_input_recovery = Mock()
    node._reach_staging = Mock()
    node._center_rear_reference = Mock(return_value=True)
    assert node._align_rear_reference(
        node._rear_reference_stage({'id': 'home_approach'}), {'id': 'home'},
        observed=observed, return_to_stage=False)
    seed = initialize.call_args.args[1]
    assert tuple(seed[k] for k in ('x_m', 'y_m', 'yaw_rad')) == pytest.approx(physical)
    node._plan_with_input_recovery.assert_not_called()
    node._reach_staging.assert_not_called()
    node._observe_rear_box.assert_not_called()
    node._center_rear_reference.assert_called_once_with(
        observed, pytest.approx(.275), actual_pose_map=physical)


@pytest.mark.parametrize('case', ['operator_stop', 'link_lost', 'sensor_lost'])
def test_reference_alignment_respects_abort_conditions(monkeypatch, case):
    node = node_with_reference()
    node.stop_requested = case == 'operator_stop'
    node._operator_link_ready = Mock(return_value=case != 'link_lost')
    node._guard_failure = Mock(return_value='scan_stale' if case == 'sensor_lost' else None)
    node._observe_rear_box = Mock()
    with pytest.raises(RuntimeError, match='operator_stop|operator_link_lost|scan_stale'):
        node._align_rear_reference({'id': 'home_approach', 'x': .7, 'y': 0., 'yaw': 0.}, {})
    node._observe_rear_box.assert_not_called()


@pytest.mark.parametrize('clear', [True, False])
def test_lateral_drift_uses_one_static_checked_curve_to_twenty_cm_handoff(monkeypatch, clear):
    node = node_with_reference()
    physical = (.7, .08, 0.)
    observation = observed_at(physical)
    node._make_reverse_path = Mock(return_value=SimpleNamespace(poses=[
        SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(z=0., w=1.)))
        for x, y in (physical[:2], (.19, 0.))]))
    node._footprint_outline = Mock(return_value=[(-.275, -.225), (.065, .225)])
    check = Mock(return_value=clear)
    monkeypatch.setattr(module, 'CachedStaticCorridorValidator', Mock(return_value=check))
    node._odom_pose = Mock(return_value=(10., 20., math.pi / 2.))
    node._odom_path = Mock(return_value='frozen landmark curve')
    node._execute_reverse_once = Mock(return_value=True)
    if not clear:
        with pytest.raises(RuntimeError, match='static corridor blocked'):
            node._center_rear_reference(observation, .275, actual_pose_map=physical)
        node._execute_reverse_once.assert_not_called()
        return
    assert node._center_rear_reference(observation, .275, actual_pose_map=physical)
    start, target = node._make_reverse_path.call_args.args
    assert start == pytest.approx(physical)
    assert target == pytest.approx((.19, 0., 0.))
    assert node._make_reverse_path.call_args.kwargs == {'curve': True}
    frozen_points = node._odom_path.call_args.args[0]
    assert frozen_points[0] == pytest.approx((10., 20., math.pi / 2.))
    assert frozen_points[-1] == pytest.approx((10.08, 19.49, math.pi / 2.))
    node._execute_reverse_once.assert_called_once()
    assert node._execute_reverse_once.call_args.kwargs['final'] is False


def test_already_centered_return_does_not_add_reverse_or_turn():
    node = node_with_reference()
    node._make_reverse_path = Mock()
    assert node._center_rear_reference(
        observed_at((.7, 0., 0.)), .275, actual_pose_map=(.7, 0., 0.))
    node._make_reverse_path.assert_not_called()


def test_rear_roi_stops_at_unclipped_matching_face_before_side_furniture(monkeypatch):
    fit = Mock(return_value={'span_m': .35, 'observation_roi_clipped': False})
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    result = module._observe_rear_face_roi({}, GEOMETRY, .275, .225, .35)
    assert result['span_m'] == .35
    fit.assert_called_once()
    assert fit.call_args.kwargs['rear_half_width_m'] == .225
    assert fit.call_args.kwargs['observation_half_width_m'] == .6


def test_rear_roi_expands_for_clipped_box_and_skips_wrong_width(monkeypatch):
    fit = Mock(side_effect=[
        {'span_m': .17, 'observation_roi_clipped': True},
        {'span_m': .8, 'observation_roi_clipped': False},
        {'span_m': .35, 'observation_roi_clipped': False},
    ])
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    result = module._observe_rear_face_roi({}, GEOMETRY, .275, .225, .35)
    assert result == {'span_m': .35, 'observation_roi_clipped': False}
    assert fit.call_count == 3


def test_known_reference_never_falls_back_to_a_different_or_clipped_face(monkeypatch):
    fit = Mock(return_value={'span_m': .8, 'observation_roi_clipped': False})
    monkeypatch.setattr(module, 'estimate_rear_box_face', fit)
    with pytest.raises(ValueError, match='width or visibility changed'):
        module._observe_rear_face_roi({}, GEOMETRY, .275, .225, .35)
    assert fit.call_count == 3
