"""Regression tests for rear LiDAR docking against the low box."""

import base64
import gzip
import json
import math
from pathlib import Path

from jdamr_cube_navigation.rear_box_docking import (
    estimate_rear_box_face,
    PROVENANCE,
    rear_corridor_clearance,
    RearBoxFaceTracker,
)
import numpy as np
import pytest


GEOMETRY = {
    'laser_translation': {'unit': 'm', 'value': [-0.01, 0.0, 0.15]},
    'laser_yaw': {'unit': 'rad', 'value': math.pi},
}
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / 1440.0
RANGE_MIN = 0.28
RANGE_MAX = 16.0
REAR_EXTENT_M = 0.275
REAR_HALF_WIDTH_M = 0.225
FIXTURE_PATH = (
    Path(__file__).parent / 'fixtures/rear_box_scan_20261016.json')


def scan_from_base_points(points):
    ranges = [None] * 1441
    for base_x, base_y in points:
        relative_x = base_x - GEOMETRY['laser_translation']['value'][0]
        relative_y = base_y - GEOMETRY['laser_translation']['value'][1]
        scan_x = -relative_x
        scan_y = -relative_y
        distance = math.hypot(scan_x, scan_y)
        if distance <= RANGE_MIN:
            continue
        angle = math.atan2(scan_y, scan_x)
        index = round((angle - ANGLE_MIN) / ANGLE_INCREMENT)
        if 0 <= index < len(ranges):
            ranges[index] = distance
    return {
        'angle_min': ANGLE_MIN,
        'angle_increment': ANGLE_INCREMENT,
        'range_min': RANGE_MIN,
        'range_max': RANGE_MAX,
        'ranges': ranges,
    }


def estimate(scan, diagnostics=None):
    return estimate_rear_box_face(
        scan, geometry=GEOMETRY, rear_extent_m=REAR_EXTENT_M,
        rear_half_width_m=REAR_HALF_WIDTH_M, diagnostics=diagnostics)


def clearance(scan):
    return rear_corridor_clearance(
        scan, geometry=GEOMETRY, rear_extent_m=REAR_EXTENT_M,
        rear_half_width_m=REAR_HALF_WIDTH_M)


def recorded_scans():
    document = json.loads(FIXTURE_PATH.read_text(encoding='utf-8'))
    assert document['provenance']['source_sha256'] == (
        '786c93c69f10d9d105fc0e95b7daef8a38582690a68f6479773b8648560307ea')
    assert document['provenance']['scope'] == 'rear_roi_ranges_only'
    assert document['encoding'] == 'gzip+base64+json'
    packed = json.loads(gzip.decompress(base64.b64decode(document['payload'])))
    scans = []
    for item in packed:
        ranges = [None] * item['size']
        for index, value in item['values']:
            ranges[index] = value
        scans.append({
            'angle_min': item['angle_min'],
            'angle_increment': item['angle_increment'],
            'range_min': item['range_min'],
            'range_max': item['range_max'],
            'ranges': ranges,
        })
    return scans


def test_rear_corridor_clearance_reports_nearest_raw_return():
    result = clearance(scan_from_base_points([
        (-0.80, -0.10), (-0.80, 0.10),
    ]))

    assert result == {
        'gap_m': pytest.approx(0.525, abs=0.002),
        'support_count': 2,
        'nearest_base_xy_m': pytest.approx((-0.80, 0.10), abs=0.002),
    }


def test_rear_corridor_clearance_prefers_close_obstacle_over_far_wall():
    far_wall = [(-1.20, lateral)
                for lateral in np.linspace(-0.20, 0.20, 31)]
    result = clearance(scan_from_base_points(
        far_wall + [(-0.47, 0.04)]))

    assert result['gap_m'] == pytest.approx(0.195, abs=0.002)
    assert result['nearest_base_xy_m'] == pytest.approx(
        (-0.47, 0.04), abs=0.002)


def test_rear_corridor_clearance_excludes_returns_outside_swept_width():
    result = clearance(scan_from_base_points([
        (-0.45, REAR_HALF_WIDTH_M + 0.02),
        (-0.75, 0.0),
    ]))

    assert result['gap_m'] == pytest.approx(0.475, abs=0.002)
    assert result['support_count'] == 1


def test_rear_corridor_clearance_accepts_a_single_return():
    result = clearance(scan_from_base_points([(-0.55, 0.08)]))

    assert result['gap_m'] == pytest.approx(0.275, abs=0.002)
    assert result['support_count'] == 1


def test_rear_corridor_clearance_preserves_negative_body_intrusion():
    result = rear_corridor_clearance(
        scan_from_base_points([(-0.30, 0.03)]), geometry=GEOMETRY,
        rear_extent_m=0.35, rear_half_width_m=REAR_HALF_WIDTH_M)

    assert result['gap_m'] == pytest.approx(-0.05, abs=0.002)
    assert result['nearest_base_xy_m'] == pytest.approx(
        (-0.30, 0.03), abs=0.002)


def test_rear_corridor_clearance_rejects_empty_rear_roi():
    with pytest.raises(ValueError, match='rear corridor has no valid points'):
        clearance(scan_from_base_points([
            (0.60, 0.0),
            (-0.60, REAR_HALF_WIDTH_M + 0.05),
        ]))


def test_rear_corridor_clearance_does_not_change_face_estimation():
    scan = recorded_scans()[0]
    before = estimate(scan)

    result = clearance(scan)

    assert result['support_count'] >= before['support_count']
    assert estimate(scan) == before


def test_recorded_box_is_visible_in_all_twelve_frames_and_stable():
    tracker = RearBoxFaceTracker()
    results = [estimate(scan) for scan in recorded_scans()]
    stable = [tracker.update(result) for result in results]

    assert [result['support_count'] for result in results] == [
        58, 60, 58, 59, 59, 60, 58, 61, 62, 61, 61, 59]
    assert all(0.002 <= result['gap_m'] <= 0.006 for result in results)
    assert all(0.007 <= result['center_gap_m'] <= 0.014 for result in results)
    assert all(result['span_m'] >= 0.19 for result in results)
    assert all(result['residual_rms_m'] < 0.003 for result in results)
    assert all(abs(math.degrees(result['normal_yaw_error_rad'])) < 3.0
               for result in results)
    assert stable[:7] == [False] * 7
    assert stable[7:] == [True] * 5
    assert results[0]['provenance'] == PROVENANCE


def test_mount_transform_and_rotated_face_produce_corner_clearance():
    angle = math.radians(6.0)
    normal = np.array((math.cos(angle), math.sin(angle)))
    tangent = np.array((-normal[1], normal[0]))
    offset = -0.34
    points = [normal * offset + tangent * lateral
              for lateral in np.linspace(-0.16, 0.16, 81)]
    result = estimate(scan_from_base_points(points))

    expected_body_projection = (
        -REAR_EXTENT_M * normal[0]
        - REAR_HALF_WIDTH_M * abs(normal[1]))
    assert result['normal_base_xy'] == pytest.approx(normal, abs=0.004)
    assert result['gap_m'] == pytest.approx(
        expected_body_projection - offset, abs=0.002)
    assert result['face_center_base_xy_m'][1] == 0.0


def test_central_minimum_range_hole_still_uses_both_return_sides():
    points = [(-0.29, lateral) for lateral in np.linspace(-0.13, 0.13, 101)]
    result = estimate(scan_from_base_points(points))
    assert result['support_count'] >= 40
    assert result['span_m'] >= 0.24
    assert result['gap_m'] == pytest.approx(0.015, abs=0.002)


def test_longitudinal_side_wall_is_not_a_rear_box_face():
    points = [(-0.30 - distance, 0.16)
              for distance in np.linspace(0.0, 0.35, 100)]
    with pytest.raises(ValueError, match='line fit failed'):
        estimate(scan_from_base_points(points))


def test_occlusion_and_narrow_support_do_not_make_a_face():
    four_points = [(-0.34, lateral) for lateral in (-0.06, -0.02, 0.02, 0.06)]
    with pytest.raises(ValueError, match='below five'):
        estimate(scan_from_base_points(four_points))

    narrow = [(-0.34, lateral) for lateral in np.linspace(-0.025, 0.025, 30)]
    with pytest.raises(ValueError, match='spread is too narrow'):
        estimate(scan_from_base_points(narrow))


def test_sparse_outliers_are_ignored_but_curved_cluster_is_rejected():
    face = [(-0.35, lateral) for lateral in np.linspace(-0.16, 0.16, 81)]
    outliers = [(-0.40 + 0.02 * index, -0.20 + 0.04 * index)
                for index in range(9)]
    result = estimate(scan_from_base_points(face + outliers))
    assert result['gap_m'] == pytest.approx(0.075, abs=0.003)
    assert result['support_count'] >= 70

    curved = [(-0.30 - 2.5 * lateral * lateral, lateral)
              for lateral in np.linspace(-0.20, 0.20, 121)]
    with pytest.raises(ValueError, match='majority support'):
        estimate(scan_from_base_points(curved))


def test_nearest_face_wins_over_a_larger_parallel_background():
    # Disjoint visible spans model a partial foreground box beside a larger
    # background wall without asking one laser ray to return both surfaces.
    near = [(-0.35, lateral) for lateral in np.linspace(-0.20, -0.10, 30)]
    far = [(-0.60, lateral) for lateral in np.linspace(0.02, 0.20, 80)]
    diagnostics = {}
    result = estimate(scan_from_base_points(near + far), diagnostics)
    assert result['face_center_base_xy_m'][0] == pytest.approx(-0.35, abs=0.002)
    assert result['center_gap_m'] == pytest.approx(0.075, abs=0.002)
    assert result['support_count'] == 30
    assert diagnostics['foreground_layer_points'] == 30
    assert diagnostics['rear_roi_points'] > diagnostics['foreground_layer_points']
    assert diagnostics['rear_depth_layer_count'] == 2


def test_invalid_foreground_obstacle_is_not_skipped_for_a_far_face():
    foreground = [(-0.34 - distance, 0.16)
                  for distance in np.linspace(0.0, 0.03, 20)]
    background = [(-0.60, lateral)
                  for lateral in np.linspace(-0.20, 0.20, 80)]
    with pytest.raises(ValueError, match='line fit failed'):
        estimate(scan_from_base_points(foreground + background))


def test_tracker_treats_endpoint_direction_as_undirected():
    result = estimate(recorded_scans()[0])
    tracker = RearBoxFaceTracker(required_frames=3)
    reversed_line = dict(result)
    reversed_line['normal_base_xy'] = tuple(-value for value in result['normal_base_xy'])
    reversed_line['tangent_base_xy'] = tuple(-value for value in result['tangent_base_xy'])
    reversed_line['line_offset_m'] = -result['line_offset_m']
    assert tracker.update(result) is False
    assert tracker.update(reversed_line) is False
    assert tracker.update(result) is True


def test_tracker_slides_on_relative_line_change_without_accepting_it():
    result = estimate(recorded_scans()[0])
    tracker = RearBoxFaceTracker(required_frames=3)
    assert tracker.update(result) is False
    assert tracker.update(result) is False
    moved = dict(result, line_offset_m=result['line_offset_m'] - 0.04)
    assert tracker.update(moved) is False
    assert tracker.frame_count == 2


def test_tracker_recovers_after_single_outlier_with_sliding_window():
    result = estimate(recorded_scans()[0])
    outlier = dict(result, line_offset_m=result['line_offset_m'] - 0.04)
    tracker = RearBoxFaceTracker(required_frames=3)

    assert tracker.update(result) is False
    assert tracker.update(result) is False
    assert tracker.update(outlier) is False
    assert tracker.frame_count == 2
    assert tracker.update(result) is False
    assert tracker.frame_count == 2
    assert tracker.update(result) is False
    assert tracker.frame_count == 2
    assert tracker.update(result) is True


def test_tracker_clears_history_when_observation_is_missing():
    result = estimate(recorded_scans()[0])
    tracker = RearBoxFaceTracker(required_frames=2)
    assert tracker.update(result) is False
    assert tracker.frame_count == 1
    assert tracker.update(None) is False
    assert tracker.frame_count == 0


def test_diagnostics_expose_selection_without_changing_result():
    diagnostics = {}
    scan = recorded_scans()[0]
    baseline = estimate(scan)
    instrumented = estimate(scan, diagnostics=diagnostics)
    assert instrumented == baseline
    assert diagnostics['rear_roi_points'] == 58
    assert diagnostics['support_count'] == 58
