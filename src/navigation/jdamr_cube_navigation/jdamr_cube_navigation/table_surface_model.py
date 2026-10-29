"""Detect separate tabletop candidates in an organized depth point cloud."""

import math
from dataclasses import dataclass

import numpy as np

from .box_top_detection import depth_points
from .pointcloud_components import organized_components
from .table_edge_detection import (
    _detect_table_edge_points,
    _optical_to_level_rotation,
    _validate_table_config,
)
from .table_surface_tracking import TargetRegion, rigid_transform


@dataclass(frozen=True)
class SceneConfig:
    """Bound component connectivity and the number of surface fits."""

    distance_m: float = 0.05
    maximum_components: int = 8


@dataclass(frozen=True)
class TableScene:
    """Component counts and fresh candidates; no object-class probability.

    ``region_prioritized_components`` is the total number of segmented
    components whose odom XY bounding box intersects the prior disk, including
    components beyond the fit budget.
    """

    candidates: tuple
    component_sizes: tuple[int, ...]
    input_point_count: int
    height_point_count: int
    evaluated_components: int
    region_prioritized_components: int = 0


def _component_intersects_region(points, transform, region):
    """Return whether an odom XY component box intersects the target disk."""
    odom_xy = points @ transform[:2, :3].T + transform[:2, 3]
    lower = np.min(odom_xy, axis=0)
    upper = np.max(odom_xy, axis=0)
    nearest = np.clip(np.asarray(region.center_xy_m), lower, upper)
    return (np.linalg.norm(np.asarray(region.center_xy_m) - nearest)
            <= region.radius_m)


def detect_table_scene(depth_mm, intrinsics, config, *, distance_m=0.05,
                       maximum_components=8, target_region_odom=None,
                       odom_from_optical=None):
    """Fit each bounded component instead of joining coplanar tables."""
    _validate_table_config(config)
    if (isinstance(maximum_components, bool)
            or not isinstance(maximum_components, int)
            or maximum_components <= 0):
        raise ValueError('maximum_components must be a positive integer')
    if (isinstance(distance_m, bool) or not math.isfinite(distance_m)
            or distance_m <= 0):
        raise ValueError('component distance must be finite and positive')
    if (target_region_odom is None) != (odom_from_optical is None):
        raise ValueError(
            'target_region_odom and odom_from_optical '
            'must be supplied together')
    transform = None
    if target_region_odom is not None:
        if not isinstance(target_region_odom, TargetRegion):
            raise ValueError('target_region_odom must be a TargetRegion')
        transform = rigid_transform(odom_from_optical)
    points, columns, rows = depth_points(depth_mm, intrinsics, config)
    level = points @ _optical_to_level_rotation(config).T
    heights = level[:, 2] + config.camera_height_m
    indices = np.flatnonzero((heights >= config.minimum_table_height_m)
                             & (heights <= config.maximum_table_height_m))
    components = organized_components(
        points[indices], rows[indices].astype(np.int64),
        columns[indices].astype(np.int64), depth_mm.shape, config.pixel_step,
        distance_m=distance_m, minimum_points=config.minimum_inliers)
    prioritized_count = 0
    ordered_components = components
    if transform is not None:
        intersections = tuple(
            _component_intersects_region(points[indices[group]], transform,
                                         target_region_odom)
            for group in components)
        prioritized = tuple(group for group, intersects
                            in zip(components, intersections) if intersects)
        prioritized_count = len(prioritized)
        ordered_components = prioritized + tuple(
            group for group, intersects in zip(components, intersections)
            if not intersects)
    candidates = []
    for group in ordered_components[:maximum_components]:
        subset = indices[group]
        detection = _detect_table_edge_points(
            depth_mm, intrinsics, config, points[subset], columns[subset],
            rows[subset])
        if detection is not None:
            candidates.append(detection)
    return TableScene(
        tuple(candidates), tuple(len(group) for group in components),
        len(points), len(indices), min(len(components), maximum_components),
        prioritized_count)
