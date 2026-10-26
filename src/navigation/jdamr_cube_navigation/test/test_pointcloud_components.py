"""Geometric separation and input-contract tests for organized XYZ samples."""

import math

from jdamr_cube_navigation.pointcloud_components import organized_components
from jdamr_cube_navigation.table_edge_detection import (
    _optical_to_level_rotation,
)
from jdamr_cube_navigation.table_surface_model import detect_table_scene
from jdamr_cube_navigation.table_surface_tracking import TargetRegion
import numpy as np
import pytest
from test_table_edge_detection import INTRINSICS, _render_table, config


def test_adjacent_pixels_with_a_metric_jump_remain_separate():
    rows, columns = np.mgrid[:2, :4]
    points = np.column_stack((columns.ravel() * .01, rows.ravel() * .01,
                              np.where(columns.ravel() < 2, 1., 2.)))
    before = points.copy()
    groups = organized_components(points, rows.ravel(), columns.ravel(), (2, 4), 1,
                                  distance_m=.03, minimum_points=2)
    assert [group.tolist() for group in groups] == [[0, 1, 4, 5], [2, 3, 6, 7]]
    np.testing.assert_array_equal(points, before)


def test_a_missing_pixel_does_not_bridge_two_coplanar_objects():
    points = np.array([[0., 0., 1.], [.01, 0., 1.], [.02, 0., 1.], [.03, 0., 1.]])
    groups = organized_components(points, np.zeros(4, dtype=int),
                                  np.array([0, 1, 3, 4]), (1, 5), 1,
                                  minimum_points=2)
    assert [group.tolist() for group in groups] == [[0, 1], [2, 3]]


@pytest.mark.parametrize('mutate', [
    lambda p, r, c: (p * np.nan, r, c),
    lambda p, r, c: (p, r.astype(float), c),
    lambda p, r, c: (p, r, np.array([0, 0])),
    lambda p, r, c: (p, r, np.array([0, 9])),
])
def test_malformed_point_pixel_correspondences_are_rejected(mutate):
    points, rows, columns = mutate(np.ones((2, 3)), np.array([0, 0]), np.array([0, 1]))
    with pytest.raises(ValueError):
        organized_components(points, rows, columns, (1, 2), 1, minimum_points=1)


def test_empty_cloud_is_a_valid_empty_segmentation():
    assert organized_components(np.empty((0, 3)), np.array([], dtype=int),
                                np.array([], dtype=int), (240, 320), 2) == ()


def test_scene_separates_two_coplanar_tabletops_without_joining_their_edges():
    configured = config()
    one = _render_table(configured, front_x_m=.8, width_m=.32,
                        background_depth_m=1.8)[0]
    depth = np.minimum(np.roll(one, -72, axis=1), np.roll(one, 72, axis=1))
    scene = detect_table_scene(depth, INTRINSICS, configured)
    assert len(scene.candidates) == 2
    assert scene.evaluated_components >= 2
    assert scene.candidates[0].center_x_m * scene.candidates[1].center_x_m < 0.
    assert all(.25 <= candidate.width_m < .40 for candidate in scene.candidates)


def test_original_background_is_available_for_edge_discontinuity_evidence():
    configured = config()
    depth = _render_table(configured, front_x_m=.8, background_depth_m=1.8)[0]
    scene = detect_table_scene(depth, INTRINSICS, configured)
    assert len(scene.candidates) == 1
    assert scene.height_point_count < scene.input_point_count
    assert abs(scene.candidates[0].table_height_m - .75) < .002


def _rotated_odom_from_optical(configured):
    yaw = math.radians(31.)
    planar = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.],
        [math.sin(yaw), math.cos(yaw), 0.],
        [0., 0., 1.],
    ])
    transform = np.eye(4)
    transform[:3, :3] = planar @ _optical_to_level_rotation(configured)
    transform[:3, 3] = (1.2, -.4, .3)
    return transform


def _two_unequal_tables(configured):
    large = _render_table(configured, front_x_m=.8, width_m=.48,
                          background_depth_m=1.8)[0]
    small = _render_table(configured, front_x_m=.8, width_m=.28,
                          background_depth_m=1.8)[0]
    return np.minimum(np.roll(large, -68, axis=1), np.roll(small, 72, axis=1))


def test_target_component_is_fitted_before_larger_outside_component():
    configured = config()
    depth = _two_unequal_tables(configured)
    transform = _rotated_odom_from_optical(configured)
    complete = detect_table_scene(depth, INTRINSICS, configured)
    target = min(complete.candidates, key=lambda candidate: candidate.width_m)
    endpoints = np.asarray(target.edge_endpoints_optical_m)
    center_odom = (endpoints.mean(axis=0) @ transform[:3, :3].T +
                   transform[:3, 3])
    region = TargetRegion(tuple(center_odom[:2]), .04)

    baseline = detect_table_scene(
        depth, INTRINSICS, configured, maximum_components=2)
    prioritized = detect_table_scene(
        depth, INTRINSICS, configured, maximum_components=2,
        target_region_odom=region, odom_from_optical=transform)

    assert baseline.candidates[0].width_m > .4
    assert prioritized.candidates[0].width_m < .3
    assert prioritized.evaluated_components == 2
    assert prioritized.region_prioritized_components == 1


def test_region_priority_preserves_candidates_for_ambiguity_handling():
    configured = config()
    depth = _two_unequal_tables(configured)
    transform = _rotated_odom_from_optical(configured)
    region = TargetRegion((1.85, -.02), .35)

    scene = detect_table_scene(
        depth, INTRINSICS, configured, maximum_components=3,
        target_region_odom=region, odom_from_optical=transform)

    assert len(scene.candidates) == 2
    assert scene.evaluated_components == 3
    assert scene.region_prioritized_components >= 2


@pytest.mark.parametrize('kwargs', [
    {'target_region_odom': TargetRegion((0., 0.), .1)},
    {'odom_from_optical': np.eye(4)},
])
def test_region_and_stamped_transform_must_be_supplied_together(kwargs):
    configured = config()
    depth = _render_table(configured, background_depth_m=1.8)[0]
    with pytest.raises(ValueError, match='together'):
        detect_table_scene(depth, INTRINSICS, configured, **kwargs)


def test_region_priority_rejects_nonrigid_stamped_transform():
    configured = config()
    depth = _render_table(configured, background_depth_m=1.8)[0]
    transform = np.eye(4)
    transform[0, 0] = 2.
    with pytest.raises(ValueError, match='rigid'):
        detect_table_scene(
            depth, INTRINSICS, configured,
            target_region_odom=TargetRegion((0., 0.), .1),
            odom_from_optical=transform)


def test_absent_region_prior_preserves_the_existing_scene_result():
    configured = config()
    depth = _two_unequal_tables(configured)
    existing = detect_table_scene(
        depth, INTRINSICS, configured, maximum_components=2)

    explicit_absence = detect_table_scene(
        depth, INTRINSICS, configured, maximum_components=2,
        target_region_odom=None, odom_from_optical=None)

    assert explicit_absence == existing
    assert existing.region_prioritized_components == 0
