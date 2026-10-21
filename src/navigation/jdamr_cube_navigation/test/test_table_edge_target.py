"""Tests for table-edge docking target geometry."""

from copy import deepcopy
import math
from pathlib import Path

from jdamr_cube_navigation.table_edge_target import (
    compute_table_edge_docking_target,
    PROVENANCE,
)

import pytest
import yaml


def mount():
    """Return a tilted rear-camera mount document."""
    return {
        'camera_mount': {
            'status': 'derived',
            'parent_frame': 'base_link',
            'child_frame': 'camera_link',
            'transform': {
                'x_m': -0.275,
                'y_m': 0.0,
                'z_m': 1.0,
                'roll_rad': math.radians(2.0),
                'pitch_rad': math.radians(27.0),
                'yaw_rad': math.pi,
            },
        },
    }


def _base_to_optical(vector, roll_rad, pitch_rad, yaw_rad):
    yaw_cosine = math.cos(yaw_rad)
    yaw_sine = math.sin(yaw_rad)
    pitch_cosine = math.cos(pitch_rad)
    pitch_sine = math.sin(pitch_rad)
    roll_cosine = math.cos(roll_rad)
    roll_sine = math.sin(roll_rad)
    base_x, base_y, base_z = vector
    yaw_x = yaw_cosine * base_x + yaw_sine * base_y
    yaw_y = -yaw_sine * base_x + yaw_cosine * base_y
    pitch_x = pitch_cosine * yaw_x - pitch_sine * base_z
    pitch_z = pitch_sine * yaw_x + pitch_cosine * base_z
    camera_y = roll_cosine * yaw_y + roll_sine * pitch_z
    camera_z = -roll_sine * yaw_y + roll_cosine * pitch_z
    return -camera_y, -camera_z, pitch_x


def observation(mounted=None):
    """Project one physical horizontal edge into the configured camera."""
    mounted = mount() if mounted is None else mounted
    transform = mounted['camera_mount']['transform']
    rotation = tuple(transform[key] for key in (
        'roll_rad', 'pitch_rad', 'yaw_rad'))
    endpoints_base = ((-1.0, -0.4, 0.75), (-1.0, 0.4, 0.75))
    camera_position = tuple(transform[key] for key in ('x_m', 'y_m', 'z_m'))
    endpoints_optical = [
        _base_to_optical(tuple(
            point[index] - camera_position[index] for index in range(3)),
            *rotation)
        for point in endpoints_base
    ]
    normal_optical = _base_to_optical((0.0, 0.0, 1.0), *rotation)
    return {
        'stamp_s': 10.0,
        'detected': True,
        'stable': True,
        'surface_kind': 'table_edge',
        'edge_endpoints_optical_m': endpoints_optical,
        'plane_normal': normal_optical,
        'table_height_m': 0.75,
        'edge_rms_m': 0.003,
        'confidence': 0.9,
    }


def target(observed=None, mounted=None, robot=None, gap=0.05, extent=0.25):
    """Compute a target from default valid fixtures."""
    mounted = mount() if mounted is None else mounted
    return compute_table_edge_docking_target(
        observation(mounted) if observed is None else observed,
        mounted,
        ({'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
         if robot is None else robot),
        requested_gap_m=gap,
        front_extent_m=extent,
        now_s=10.2,
    )


def test_full_mount_geometry_produces_five_centimeter_table_gap():
    """Full roll, pitch and yaw recover the physical five-centimetre gap."""
    result = target()
    assert result['face_center_map_xy_m'] == pytest.approx((-1.0, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((1.0, 0.0))
    assert result['x_m'] == pytest.approx(-0.70)
    assert result['y_m'] == pytest.approx(0.0)
    assert abs(result['yaw_rad']) == pytest.approx(math.pi)
    assert result['estimate_gap_m'] == pytest.approx(0.05)
    assert result['provenance'] == PROVENANCE
    assert result['table_height_m'] == pytest.approx(0.75)
    assert result['surface_kind'] == 'table_edge'
    assert abs(result['edge_basis_map_xy'][1]) == pytest.approx(1.0)


def test_both_edge_ends_share_the_requested_line_gap():
    """The parallel robot-front line is five centimetres from both ends."""
    result = target()
    outward = result['outward_normal_map_xy']
    heading = (-outward[0], -outward[1])
    robot_front_center = (
        result['x_m'] + heading[0] * 0.25,
        result['y_m'] + heading[1] * 0.25,
    )
    for endpoint in ((-1.0, -0.4), (-1.0, 0.4)):
        signed_gap = (
            (robot_front_center[0] - endpoint[0]) * outward[0]
            + (robot_front_center[1] - endpoint[1]) * outward[1]
        )
        assert signed_gap == pytest.approx(0.05)


def test_robot_map_pose_rotates_edge_target_and_basis():
    """Robot map yaw rotates the target, normal and edge basis together."""
    result = target(robot={
        'x_m': 2.0, 'y_m': 1.0, 'yaw_rad': math.pi / 2.0})
    assert result['x_m'] == pytest.approx(2.0)
    assert result['y_m'] == pytest.approx(0.30)
    assert result['face_center_map_xy_m'] == pytest.approx((2.0, 0.0))
    assert result['outward_normal_map_xy'] == pytest.approx((0.0, 1.0))
    assert abs(result['edge_basis_map_xy'][0]) == pytest.approx(1.0)
    assert result['yaw_rad'] == pytest.approx(-math.pi / 2.0)


@pytest.mark.parametrize(
    'mutate, message', [
        (lambda item: item.update(stamp_s=9.0), 'stale'),
        (lambda item: item.update(stable=False), 'stable detected'),
        (lambda item: item.update(surface_kind='front'), 'table_edge'),
        (lambda item: item.update(
            edge_endpoints_optical_m=[[0.0, 0.0, 1.0]]), 'two points'),
        (lambda item: item.update(
            edge_endpoints_optical_m=[
                [0.0, 0.0, 1.0], [math.nan, 0.0, 1.0]]), 'finite'),
        (lambda item: item.update(plane_normal=[0.0, 0.0, 2.0]),
         'unit optical'),
    ],
)
def test_rejects_invalid_or_stale_edge_observations(mutate, message):
    """Malformed, stale, unstable or wrongly typed observations fail closed."""
    observed = observation()
    mutate(observed)
    with pytest.raises(ValueError, match=message):
        target(observed=observed)


def test_rejects_short_edge_and_nonhorizontal_plane():
    """A short line or non-upward top normal cannot define table docking."""
    mounted = mount()
    observed = observation(mounted)
    first = observed['edge_endpoints_optical_m'][0]
    observed['edge_endpoints_optical_m'] = [first, first]
    with pytest.raises(ValueError, match='too short'):
        target(observed=observed, mounted=mounted)

    observed = observation(mounted)
    transform = mounted['camera_mount']['transform']
    observed['plane_normal'] = _base_to_optical(
        (1.0, 0.0, 0.0),
        transform['roll_rad'], transform['pitch_rad'], transform['yaw_rad'])
    with pytest.raises(ValueError, match='point upward'):
        target(observed=observed, mounted=mounted)


def test_camera_height_or_angle_change_is_consumed_from_mount():
    """Updated camera height and angles preserve the physical target."""
    adjusted = deepcopy(mount())
    transform = adjusted['camera_mount']['transform']
    transform.update({
        'z_m': 1.12,
        'roll_rad': math.radians(-4.0),
        'pitch_rad': math.radians(33.0),
        'yaw_rad': math.pi - 0.15,
    })
    result = target(mounted=adjusted)
    assert result['face_center_map_xy_m'] == pytest.approx((-1.0, 0.0))
    assert result['x_m'] == pytest.approx(-0.70)
    assert result['estimate_gap_m'] == pytest.approx(0.05)


def test_rejects_table_height_inconsistent_with_transformed_edge():
    """Reported and transformed table heights must describe one edge."""
    observed = observation()
    observed['table_height_m'] = 0.90
    with pytest.raises(ValueError, match='height disagrees'):
        target(observed=observed)


@pytest.mark.parametrize(
    'field, value, message', [
        ('confidence', 0.0, 'confidence'),
        ('edge_rms_m', 0.05, 'edge_rms_m'),
    ],
)
def test_rejects_observations_below_minimum_edge_quality(
        field, value, message):
    """A target is not produced from zero confidence or a poor line fit."""
    observed = observation()
    observed[field] = value
    with pytest.raises(ValueError, match=message):
        target(observed=observed)


def calibrated_mount(yaw=0.10, bias=0.02):
    mounted = mount()
    mounted['camera_mount']['table_edge_calibration'] = {
        'enabled': True, 'status': 'derived',
        'yaw_offset_rad': yaw, 'plane_offset_bias_m': bias,
        'reference_transform': deepcopy(mounted['camera_mount']['transform']),
    }
    return mounted


def test_task_calibration_rotates_and_offsets_one_face_before_map_projection():
    mounted = calibrated_mount()
    unchanged = deepcopy(mounted)
    robot = {'x_m': 2.0, 'y_m': -1.0, 'yaw_rad': 0.7}
    result = target(mounted=mounted, robot=robot)
    angle = robot['yaw_rad'] + 0.10
    normal = (math.cos(angle), math.sin(angle))
    face = (2.0 - 1.02 * normal[0], -1.0 - 1.02 * normal[1])
    assert result['outward_normal_map_xy'] == pytest.approx(normal)
    assert result['face_center_map_xy_m'] == pytest.approx(face)
    assert (result['x_m'], result['y_m']) == pytest.approx((
        face[0] + 0.30 * normal[0], face[1] + 0.30 * normal[1]))
    assert result['yaw_rad'] == pytest.approx(angle - math.pi)
    tangent = result['edge_basis_map_xy']
    assert sum(a * b for a, b in zip(normal, tangent)) == pytest.approx(0.0)
    calibration = result['table_edge_calibration']
    assert calibration['raw_plane_offset_m'] == pytest.approx(1.0)
    assert calibration['corrected_plane_offset_m'] == pytest.approx(1.02)
    # Observation calibration must not rewrite physical extrinsics or top height.
    assert mounted == unchanged
    assert result['table_height_m'] == pytest.approx(0.75)


def test_disabled_task_calibration_leaves_target_unchanged():
    mounted = calibrated_mount()
    mounted['camera_mount']['table_edge_calibration']['enabled'] = False
    assert target(mounted=mounted) == target()


@pytest.mark.parametrize('field', (
    'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad'))
def test_task_calibration_rejects_changed_physical_mount(field):
    mounted = calibrated_mount()
    mounted['camera_mount']['transform'][field] += 0.001
    with pytest.raises(ValueError, match='mount has changed'):
        target(mounted=mounted)


@pytest.mark.parametrize('calibration, message', [
    (None, 'mapping'),
    ({'enabled': 1, 'status': 'derived'}, 'enabled'),
    ({'enabled': True, 'status': 'candidate'}, 'derived or measured'),
    ({'enabled': True, 'status': 'derived', 'yaw_offset_rad': math.nan,
      'plane_offset_bias_m': 0.0}, 'finite'),
])
def test_task_calibration_rejects_malformed_input(calibration, message):
    mounted = mount()
    mounted['camera_mount']['table_edge_calibration'] = calibration
    with pytest.raises(ValueError, match=message):
        target(mounted=mounted)


def test_current_task_calibration_replays_its_reference_faces():
    """Replay recorded fit/reference geometry; this is not new physical evidence."""
    mounted = yaml.safe_load((Path(__file__).resolve().parents[1]
                              / 'config/camera_mount_current.yaml').read_text())
    transform = mounted['camera_mount']['transform']
    provenance = mounted['camera_mount']['table_edge_calibration']['provenance']
    rotation = tuple(transform[key] for key in (
        'roll_rad', 'pitch_rad', 'yaw_rad'))
    front = provenance['reference_front_extent_m']
    half_width = provenance['reference_width_m'] / 2.0
    for view in provenance['reference_views']:
        normal = view['raw_normal_xy']
        center = tuple(-view['raw_offset_m'] * value for value in normal)
        tangent = (-normal[1], normal[0])
        observed = observation(mounted)
        observed['table_height_m'] = 0.78
        observed['edge_endpoints_optical_m'] = [
            _base_to_optical((
                center[0] + side * tangent[0] - transform['x_m'],
                center[1] + side * tangent[1] - transform['y_m'],
                observed['table_height_m'] - transform['z_m']), *rotation)
            for side in (-0.20, 0.20)]
        result = target(observed=observed, mounted=mounted, gap=0.01, extent=front)
        actual_normal = result['outward_normal_map_xy']
        actual_face = result['face_center_map_xy_m']
        actual_offset = -sum(a * b for a, b in zip(actual_normal, actual_face))
        for side in (-half_width, half_width):
            actual_gap = actual_normal[0] * front + actual_normal[1] * side + actual_offset
            expected_gap = (view['reference_normal_xy'][0] * front
                            + view['reference_normal_xy'][1] * side
                            + view['reference_offset_m'])
            assert actual_gap == pytest.approx(expected_gap, abs=0.002)
        # At the generated final pose, both front corners have the requested gap.
        heading = result['yaw_rad']
        for side in (-half_width, half_width):
            corner = (result['x_m'] + front * math.cos(heading) - side * math.sin(heading),
                      result['y_m'] + front * math.sin(heading) + side * math.cos(heading))
            assert sum(a * b for a, b in zip(actual_normal, corner)) + actual_offset == (
                pytest.approx(0.01))
