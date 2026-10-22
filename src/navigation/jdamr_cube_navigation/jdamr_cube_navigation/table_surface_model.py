"""Detect separate tabletop candidates in an organized depth point cloud."""

from dataclasses import dataclass
import math

import numpy as np

from .box_top_detection import depth_points
from .pointcloud_components import organized_components
from .table_edge_detection import (
    _detect_table_edge_points,
    _optical_to_level_rotation,
    _validate_table_config,
)


@dataclass(frozen=True)
class SceneConfig:
    """Bound component connectivity and the number of surface fits."""

    distance_m: float = 0.05
    maximum_components: int = 8


@dataclass(frozen=True)
class TableScene:
    """Component counts and fresh candidates; no object-class probability."""

    candidates: tuple
    component_sizes: tuple[int, ...]
    input_point_count: int
    height_point_count: int
    evaluated_components: int


def detect_table_scene(depth_mm, intrinsics, config, *, distance_m=0.05,
                       maximum_components=8):
    """Fit each bounded component instead of joining coplanar tables."""
    _validate_table_config(config)
    if (isinstance(maximum_components, bool)
            or not isinstance(maximum_components, int) or maximum_components <= 0):
        raise ValueError('maximum_components must be a positive integer')
    if isinstance(distance_m, bool) or not math.isfinite(distance_m) or distance_m <= 0:
        raise ValueError('component distance must be finite and positive')
    points, columns, rows = depth_points(depth_mm, intrinsics, config)
    level = points @ _optical_to_level_rotation(config).T
    heights = level[:, 2] + config.camera_height_m
    indices = np.flatnonzero((heights >= config.minimum_table_height_m)
                             & (heights <= config.maximum_table_height_m))
    components = organized_components(
        points[indices], rows[indices].astype(np.int64),
        columns[indices].astype(np.int64), depth_mm.shape, config.pixel_step,
        distance_m=distance_m, minimum_points=config.minimum_inliers)
    candidates = []
    for group in components[:maximum_components]:
        subset = indices[group]
        detection = _detect_table_edge_points(
            depth_mm, intrinsics, config, points[subset], columns[subset], rows[subset])
        if detection is not None:
            candidates.append(detection)
    return TableScene(tuple(candidates), tuple(len(group) for group in components),
                      len(points), len(indices), min(len(components), maximum_components))
