"""Regression tests for taught-station control integration without robot motion."""

import json
import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from jdamr_cube_navigation import box_service
from jdamr_cube_navigation.box_service import BoxServiceRoute
from jdamr_cube_navigation.restaurant_service import ServiceRoute
import pytest


def _pose(x=0., y=0., yaw=0.):
    rotation = SimpleNamespace(x=0., y=0., z=math.sin(yaw / 2.),
                               w=math.cos(yaw / 2.))
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y), rotation=rotation)


def _clock(stamp_s=10.):
    return SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(stamp_s * 1e9)))


def _node():
    node = object.__new__(BoxServiceRoute)
    node.registry = {
        'map': {'yaml_sha256': 'a' * 64, 'image_sha256': 'b' * 64},
        'keepout': {'yaml_sha256': 'c' * 64, 'image_sha256': 'd' * 64},
        'tables': [{
            'table_id': 'water_station',
            'service_poses': [{
                'priority': 1, 'x_m': 1.2, 'y_m': -.4, 'yaw_rad': .1,
            }],
        }],
    }
    node.parking_contract = {'robot_base_frame': 'base_link'}
    node.station_reference = None
    node.station_reference_profile = None
    node.station_reference_rgb_info = None
    node.station_reference_result = None
    node.station_reference_result_received_s = None
    node.station_reference_image_subscription = None
    node.station_reference_info_subscription = None
    node._station_reference_pending = None
    node._station_reference_lock = threading.Lock()
    node._station_reference_wake = threading.Event()
    node._station_reference_stop = threading.Event()
    node._station_reference_thread = None
    node.emit = Mock()
    node.create_subscription = Mock(side_effect=['info-sub', 'image-sub'])
    node.get_clock = Mock(return_value=_clock())
    return node


def _profiles(tmp_path):
    path = tmp_path / 'station_profiles.json'
    path.write_text(json.dumps({
        'schema_version': 1,
        'map': {'yaml_sha256': 'a' * 64, 'image_sha256': 'b' * 64},
        'keepout': {'yaml_sha256': 'c' * 64, 'image_sha256': 'd' * 64},
        'stations': {
            'water_station': {
                'target_priority': 'operator_taught_map_pose_and_heading',
                'lateral_tolerance_m': .005,
                'lateral_precision_status': 'NOT_VALIDATED',
                'rgbd_reference': {'immutable': 'reference'},
            },
        },
    }), encoding='utf-8')
    return path


def test_profile_loads_hash_bound_registered_pose_and_one_rgb_pair(
        tmp_path, monkeypatch):
    node = _node()
    model = Mock()
    monkeypatch.setattr(box_service, 'TaughtStationReference', Mock(return_value=model))

    node.configure_station_reference(_profiles(tmp_path), 'water_station')

    assert node.station_reference is model
    assert node.station_reference_taught_map_pose == pytest.approx((1.2, -.4, .1))
    assert node.create_subscription.call_count == 2
    topics = [call.args[1] for call in node.create_subscription.call_args_list]
    assert topics == ['/camera/color/camera_info', '/camera/color/image_raw']
    assert node._station_reference_thread.is_alive()
    node._station_reference_stop.set()
    node._station_reference_wake.set()
    node._station_reference_thread.join(timeout=1.)
    assert not node._station_reference_thread.is_alive()


def test_profile_rejects_different_map_before_subscribing(tmp_path, monkeypatch):
    node = _node()
    profile_path = _profiles(tmp_path)
    document = json.loads(profile_path.read_text())
    document['map']['image_sha256'] = '0' * 64
    profile_path.write_text(json.dumps(document), encoding='utf-8')
    constructor = Mock()
    monkeypatch.setattr(box_service, 'TaughtStationReference', constructor)

    with pytest.raises(ValueError, match='map image_sha256 mismatch'):
        node.configure_station_reference(profile_path, 'water_station')

    constructor.assert_not_called()
    node.create_subscription.assert_not_called()


def test_worker_uses_image_stamp_tf_and_keeps_only_registered_pose_correction(monkeypatch):
    # A fresh Jazzy process need not expose duration on the rclpy package.
    monkeypatch.delattr(box_service.rclpy, 'duration', raising=False)
    node = _node()
    model = Mock(return_value=None)
    model.observe.return_value = {
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'goal_odom': [1., .25, .2], 'stamp_s': 10.,
    }
    node.station_reference = model
    node.parking_tf = Mock()

    def lookup(target, source, stamp, *, timeout):
        assert stamp.nanoseconds == int(10e9)
        assert 0 < timeout.nanoseconds <= int(.2e9)
        if (target, source) == ('odom', 'base_link'):
            return SimpleNamespace(transform=_pose(.8, .1, .05))
        assert (target, source) == ('map', 'odom')
        return SimpleNamespace(transform=_pose(2., 3., math.pi / 2))

    node.parking_tf.lookup_transform.side_effect = lookup
    node.station_reference_rgb_info = SimpleNamespace(
        header=SimpleNamespace(frame_id='camera_color_optical_frame'))
    image = SimpleNamespace(
        encoding='rgb8', height=2, width=2, step=6,
        data=bytes(range(12)),
        header=SimpleNamespace(
            frame_id='camera_color_optical_frame',
            stamp=SimpleNamespace(sec=10, nanosec=0)))
    worker = threading.Thread(target=node._station_reference_loop)
    worker.start()
    node._station_image_callback(image)
    deadline = time.monotonic() + 1.
    while node.station_reference_result is None and time.monotonic() < deadline:
        time.sleep(.005)
    node._station_reference_stop.set()
    node._station_reference_wake.set()
    worker.join(timeout=1.)

    assert model.observe.call_count == 1
    observed_image, observed_info, observed_pose, stamp_s = model.observe.call_args.args
    assert observed_image.flags['C_CONTIGUOUS']
    assert observed_image[0, 0].tolist() == [2, 1, 0]
    assert observed_info is node.station_reference_rgb_info
    assert observed_pose == pytest.approx((.8, .1, .05))
    assert stamp_s == 10.
    assert node.station_reference_result['goal_map'] == pytest.approx(
        (1.75, 4., math.pi / 2 + .2))


def test_rgb_callback_is_nonblocking_and_replaces_pending_frame():
    node = _node()
    model = Mock()
    node.station_reference = model
    node.station_reference_rgb_info = SimpleNamespace(
        header=SimpleNamespace(frame_id='camera_color_optical_frame'))

    def image(stamp):
        return SimpleNamespace(
            encoding='bgr8', height=1, width=1, step=3,
            data=bytes((stamp, 0, 0)),
            header=SimpleNamespace(
                frame_id='camera_color_optical_frame',
                stamp=SimpleNamespace(sec=stamp, nanosec=0)))

    node._station_image_callback(image(10))
    node._station_image_callback(image(11))

    model.observe.assert_not_called()
    pending_image, _pending_info, pending_stamp = node._station_reference_pending
    assert pending_stamp == 11.
    assert pending_image[0, 0].tolist() == [11, 0, 0]


def test_stationary_target_anchors_tangent_to_registered_pose_by_default():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_taught_map_pose = (1.2, -.4, .1)
    node._fresh_station_reference = Mock(return_value={
        'valid': False, 'reason': 'station_reference_not_observed'})
    target = {
        'x_m': .40, 'y_m': .20,
        'face_center_map_xy_m': [.50, .20],
        'outward_normal_map_xy': [-1., 0.],
        'estimate_gap_m': .01,
    }

    anchored = node._station_anchor_target(target)

    assert anchored['x_m'] == pytest.approx(.40)
    assert anchored['y_m'] == pytest.approx(-.40)
    assert anchored['face_center_map_xy_m'] == pytest.approx((.50, -.40))
    assert anchored['outward_normal_map_xy'] == [-1., 0.]
    assert anchored['estimate_gap_m'] == .01
    assert anchored['rgbd_reference_valid'] is False
    assert anchored['millimeter_accuracy_validated'] is False


def test_fresh_rgbd_goal_replaces_only_taught_tangent():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_taught_map_pose = (1.2, -.4, .1)
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'goal_map': [1.1, .08, .02],
    })
    target = {
        'x_m': .40, 'y_m': .20,
        'face_center_map_xy_m': [.50, .20],
        'outward_normal_map_xy': [-1., 0.],
    }

    anchored = node._station_anchor_target(target)

    assert anchored['x_m'] == pytest.approx(.40)
    assert anchored['y_m'] == pytest.approx(.08)
    assert anchored['face_center_map_xy_m'] == pytest.approx((.50, .08))
    assert anchored['rgbd_reference_valid'] is True


def test_invalid_or_stale_reference_never_claims_lateral_precision():
    node = _node()
    node.station_reference = Mock()
    node._fresh_station_reference = Mock(return_value={
        'valid': False, 'reason': 'station_reference_stale'})

    status = node._station_lateral_status()

    assert status == {
        'lateral_reference_valid': False,
        'lateral_reference_reason': 'station_reference_stale',
        'lateral_tolerance_m': .005,
        'lateral_precision_status': 'NOT_VALIDATED',
        'millimeter_accuracy_validated': False,
        'internal_lateral_error_m': None,
        'internal_lateral_within_tolerance': False,
        'lateral_goal_valid': False,
        'lateral_goal_basis': None,
        'lateral_goal_observation_stamp_s': None,
    }


def test_rejected_rgb_frame_retains_accepted_odom_goal_only_for_steering():
    node = _node()
    node.station_reference = Mock()
    accepted = {
        'valid': True, 'goal_odom': [.4, .03, 0.],
        'goal_map': [1.4, .13, 0.], 'stamp_s': 9.9,
    }
    rejected = {'valid': False, 'reason': 'camera_pose_has_excess_pitch_roll'}
    node._fresh_station_reference = Mock(side_effect=[accepted, rejected, rejected])
    first, current = node._station_goal_reference()
    carried, current = node._station_goal_reference()

    assert first == carried == accepted
    assert current == rejected
    status = node._station_lateral_status()
    assert status['lateral_goal_valid'] is True
    assert status['lateral_reference_valid'] is False
    assert status['internal_lateral_within_tolerance'] is False
    assert status['lateral_goal_observation_stamp_s'] == 9.9


def test_final_lateral_status_uses_current_observation_even_after_goal_freezes():
    node = _node()
    node.station_reference = Mock()
    node.station_reference_goal_frozen = True
    node.station_reference_goal = {
        'valid': True, 'goal_odom': [.4, 0., 0.], 'stamp_s': 9.8,
    }
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'goal_odom': [.4, .028, 0.], 'stamp_s': 10.,
    })
    node._odom_pose = Mock(return_value=(.4, 0., 0.))

    status = node._station_lateral_status()

    assert status['internal_lateral_error_m'] == pytest.approx(.028)
    assert status['internal_lateral_within_tolerance'] is False


def test_moving_table_update_anchors_face_and_steers_on_existing_heading_path(
        monkeypatch):
    node = _node()
    node.station_reference = Mock()
    node.station_reference_profile = {'lateral_tolerance_m': .005}
    node.station_reference_taught_map_pose = (.40, .20, 0.)
    node._fresh_station_reference = Mock(return_value={
        'valid': True, 'reason': 'observed_registered_reference_pose',
        'goal_odom': [.40, .10, 0.], 'stamp_s': 10.,
    })
    node._table_observation_context = ({}, (.50, 0.), .6)
    node._table_approach_validator = Mock(return_value=Mock())
    node._odom_pose = Mock(return_value=(0., 0., 0.))
    node.box_status = {
        'stamp_s': 9., 'detected': True, 'stable': True,
        'surface_kind': 'table_edge',
        'frame_id': 'camera_color_optical_frame',
        'control_ready': False, 'width_m': .6,
    }
    node.box_status_received_s = time.monotonic()
    node.parking_tf = Mock()
    node.parking_tf.lookup_transform.return_value = SimpleNamespace(
        transform=_pose())
    node.get_clock = Mock(return_value=_clock())
    node._fresh_depth_frame_after = Mock(return_value=True)
    node._final_gap_m = Mock(return_value=.01)
    node.box_front_half_width_m = .225
    node.observer_stable_frames = 8
    node.emit = Mock()
    measured = {
        'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
        'face_center_map_xy_m': [.50, 0.],
        'outward_normal_map_xy': [-1., 0.],
    }
    monkeypatch.setattr(
        box_service, 'compute_table_edge_docking_target',
        Mock(return_value=measured))
    target = {
        'x_m': .425, 'y_m': 0., 'yaw_rad': 0.,
        'face_center_map_xy_m': [.50, 0.],
        'outward_normal_map_xy': [-1., 0.],
        'observed_face_span_m': .6,
    }

    update = node._table_approach_update(
        target, .065, lambda x, y, yaw: (x, y, yaw))
    node.box_status = {**node.box_status, 'stamp_s': 10.}
    control = update((0., 0., 0.))

    assert control['remaining_m'] == pytest.approx(.425)
    assert control['heading_rad'] > 0.
    stop = node._continuous_table_stop
    assert stop['target']['face_center_map_xy_m'] == pytest.approx((.50, .10))
    assert stop['target']['outward_normal_map_xy'] == pytest.approx((-1., 0.))
    assert stop['target']['x_m'] == pytest.approx(.425)
    assert stop['target']['y_m'] == pytest.approx(.10)
    assert stop['lateral_reference_valid'] is True

    # Entering the old 5 cm precision band must not discard a remaining
    # lateral error simply because the chassis is currently face-parallel.
    near = update((.39, 0., 0.))
    assert near['heading_rad'] > 0.
    assert node._continuous_table_stop['basis'] != 'aligned_face_frozen_for_precision_band'

    aligned = update((.39, .10, 0.))
    assert aligned['heading_rad'] == pytest.approx(0.)
    assert node.station_reference_goal_frozen is True


def test_destroy_stops_worker_before_ros_node(monkeypatch):
    node = _node()
    worker = threading.Thread(target=node._station_reference_loop)
    node._station_reference_thread = worker
    worker.start()
    destroyed = Mock(return_value='destroyed')
    monkeypatch.setattr(ServiceRoute, 'destroy_node', destroyed)

    assert node.destroy_node() == 'destroyed'

    assert not worker.is_alive()
    destroyed.assert_called_once_with()
