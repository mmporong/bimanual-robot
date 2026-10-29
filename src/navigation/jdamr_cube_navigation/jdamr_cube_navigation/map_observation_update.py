"""Offline occupancy-map updates from stationary planar LiDAR captures."""

from __future__ import annotations

from collections import deque
import math
from typing import Iterable, Mapping, Sequence

import numpy as np


def update_map_from_stationary_capture(
        image: np.ndarray,
        metadata: Mapping[str, object],
        capture: Mapping[str, object],
        map_to_base: Sequence[float] | Mapping[str, float],
        roi_mask: np.ndarray,
        *,
        base_frame: str = 'base_link',
        min_observations: int = 3,
        occupied_fraction: float = 0.6,
        free_fraction: float = 0.9,
        occupied_value: int = 0,
        free_value: int = 254,
        max_range_m: float | None = None) -> np.ndarray:
    """Fuse repeat observations into a copy of an existing occupancy image.

    ``capture`` contains a ``scans`` sequence and ``tf_static`` transforms.
    Each scan provides ``frame``, ``angle_min``, ``angle_increment``,
    ``range_min``, ``range_max``, and ``ranges``. Static transforms use
    ``parent``, ``child``, translation ``t`` (x, y, z), and quaternion ``q``
    (x, y, z, w). ``map_to_base`` is the fixed base pose ``(x, y, yaw)``.

    Only finite ranges within both scan limits (and ``max_range_m`` when
    supplied) are observations. A hit overrides free evidence for the same
    cell within one scan frame. The caller is responsible for establishing
    that the robot and scene region were stationary during the capture.
    """
    source = np.asarray(image)
    roi = np.asarray(roi_mask)
    if source.ndim != 2:
        raise ValueError('image must be a two-dimensional occupancy image')
    if roi.shape != source.shape or roi.dtype.kind != 'b':
        raise ValueError('roi_mask must be boolean and match image shape')

    resolution, origin = _map_geometry(metadata, source.shape)
    base_pose = _base_pose(map_to_base)
    _validate_fusion_settings(
        min_observations, occupied_fraction, free_fraction, max_range_m)

    hit_votes = np.zeros(source.shape, dtype=np.uint32)
    free_votes = np.zeros(source.shape, dtype=np.uint32)
    transforms = _transform_graph(capture.get('tf_static', ()))
    scans = capture.get('scans', ())
    if scans is None:
        scans = ()
    if not isinstance(scans, (list, tuple)):
        raise ValueError('capture scans must be a sequence')

    for scan in scans:
        frame_hits, frame_free = _scan_cells(
            scan, transforms, base_frame, base_pose, source.shape,
            resolution, origin, max_range_m)
        frame_free.difference_update(frame_hits)
        for row, column in frame_hits:
            if roi[row, column]:
                hit_votes[row, column] += 1
        for row, column in frame_free:
            if roi[row, column]:
                free_votes[row, column] += 1

    result = source.copy()
    evidence = hit_votes + free_votes
    enough = evidence >= min_observations
    denominator = np.maximum(evidence, 1)
    hit_fraction = hit_votes / denominator
    free_fraction_values = free_votes / denominator
    occupied = enough & (hit_fraction >= occupied_fraction) & roi
    free = enough & (free_fraction_values >= free_fraction) & roi & ~occupied
    result[occupied] = occupied_value
    result[free] = free_value
    return result


def _map_geometry(
        metadata: Mapping[str, object],
        shape: tuple[int, int]) -> tuple[float, tuple[float, float, float]]:
    try:
        resolution = float(metadata['resolution'])
        origin_values = metadata['origin']
        origin = tuple(float(value) for value in origin_values)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('metadata needs numeric resolution and origin') from error
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError('resolution must be finite and positive')
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError('origin must contain finite x, y, and yaw')
    height, width = shape
    for key, expected in (('width', width), ('height', height)):
        if key in metadata and int(metadata[key]) != expected:
            raise ValueError(f'metadata {key} does not match image shape')
    return resolution, origin


def _base_pose(
        pose: Sequence[float] | Mapping[str, float],
        ) -> tuple[float, float, float]:
    if isinstance(pose, Mapping):
        values = (pose.get('x'), pose.get('y'), pose.get('yaw'))
    else:
        values = tuple(pose)
    if len(values) != 3:
        raise ValueError('map_to_base must contain x, y, and yaw')
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError('map_to_base must contain x, y, and yaw') from error
    if not all(math.isfinite(value) for value in result):
        raise ValueError('map_to_base values must be finite')
    return result


def _validate_fusion_settings(
        minimum: int, occupied_fraction: float, free_fraction: float,
        max_range_m: float | None) -> None:
    if isinstance(minimum, bool) or int(minimum) != minimum or minimum < 1:
        raise ValueError('min_observations must be a positive integer')
    for name, value in (
            ('occupied_fraction', occupied_fraction),
            ('free_fraction', free_fraction)):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f'{name} must be between zero and one')
    if max_range_m is not None and (
            not math.isfinite(max_range_m) or max_range_m <= 0.0):
        raise ValueError('max_range_m must be finite and positive')


def _scan_cells(
        scan: object,
        transforms: Mapping[str, list[tuple[str, np.ndarray]]],
        base_frame: str,
        base_pose: tuple[float, float, float],
        shape: tuple[int, int],
        resolution: float,
        origin: tuple[float, float, float],
        max_range_m: float | None,
        ) -> tuple[set[tuple[int, int]], set[tuple[int, int]]]:
    if not isinstance(scan, Mapping):
        raise ValueError('each scan must be a mapping')
    ranges = scan.get('ranges')
    if ranges is None:
        return set(), set()
    if not isinstance(ranges, (list, tuple, np.ndarray)):
        raise ValueError('scan ranges must be a sequence or None')
    try:
        angle_min = float(scan['angle_min'])
        angle_increment = float(scan['angle_increment'])
        range_min = float(scan['range_min'])
        range_max = float(scan['range_max'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('scan angle and range limits must be numeric') from error
    limits = (angle_min, angle_increment, range_min, range_max)
    if not all(math.isfinite(value) for value in limits):
        raise ValueError('scan angle and range limits must be finite')
    if range_min < 0.0 or range_max < range_min:
        raise ValueError('scan range limits are invalid')

    laser_frame = _frame_name(scan.get('frame', scan.get('frame_id', '')))
    if not laser_frame:
        raise ValueError('each scan needs a frame')
    laser_to_base = _resolve_transform(transforms, laser_frame, base_frame)
    laser_origin = laser_to_base @ np.array([0.0, 0.0, 0.0, 1.0])
    origin_world = _base_to_map(laser_origin[:2], base_pose)
    hit_cells: set[tuple[int, int]] = set()
    free_cells: set[tuple[int, int]] = set()

    effective_max = range_max
    if max_range_m is not None:
        effective_max = min(effective_max, max_range_m)
    for index, raw_range in enumerate(ranges):
        if raw_range is None:
            continue
        try:
            distance = float(raw_range)
        except (TypeError, ValueError):
            continue
        if (not math.isfinite(distance) or distance < range_min or
                distance > range_max or distance > effective_max):
            continue
        angle = angle_min + index * angle_increment
        endpoint_laser = np.array([
            distance * math.cos(angle),
            distance * math.sin(angle),
            0.0,
            1.0,
        ])
        endpoint_base = laser_to_base @ endpoint_laser
        endpoint_world = _base_to_map(endpoint_base[:2], base_pose)
        ray_free, ray_hit = _ray_cells(
            origin_world, endpoint_world, shape, resolution, origin)
        free_cells.update(ray_free)
        if ray_hit is not None:
            hit_cells.add(ray_hit)
    return hit_cells, free_cells


def _frame_name(value: object) -> str:
    return str(value).strip().lstrip('/')


def _transform_graph(
        raw_transforms: object,
        ) -> dict[str, list[tuple[str, np.ndarray]]]:
    if raw_transforms is None:
        raw_transforms = ()
    if isinstance(raw_transforms, Mapping):
        transforms: Iterable[object] = (
            raw_transforms.get('transforms', [raw_transforms]))
    else:
        transforms = raw_transforms
    if not isinstance(transforms, (list, tuple)):
        raise ValueError('tf_static must be a transform or sequence')
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for transform in transforms:
        if not isinstance(transform, Mapping):
            raise ValueError('each static transform must be a mapping')
        parent = _frame_name(transform.get('parent', ''))
        child = _frame_name(transform.get('child', ''))
        if not parent or not child or parent == child:
            raise ValueError('static transform needs distinct parent and child')
        matrix = _rigid_transform(transform.get('t'), transform.get('q'))
        graph.setdefault(child, []).append((parent, matrix))
        graph.setdefault(parent, []).append((child, np.linalg.inv(matrix)))
    return graph


def _rigid_transform(translation: object, quaternion: object) -> np.ndarray:
    try:
        t = np.asarray(tuple(translation), dtype=float)
        q = np.asarray(tuple(quaternion), dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError('transform t and q must be numeric sequences') from error
    if t.shape != (3,) or q.shape != (4,) or not (
            np.isfinite(t).all() and np.isfinite(q).all()):
        raise ValueError('transform t and q have invalid values')
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError('transform quaternion cannot be zero')
    x_value, y_value, z_value, w_value = q / norm
    rotation = np.array([
        [1 - 2 * (y_value**2 + z_value**2),
         2 * (x_value * y_value - z_value * w_value),
         2 * (x_value * z_value + y_value * w_value)],
        [2 * (x_value * y_value + z_value * w_value),
         1 - 2 * (x_value**2 + z_value**2),
         2 * (y_value * z_value - x_value * w_value)],
        [2 * (x_value * z_value - y_value * w_value),
         2 * (y_value * z_value + x_value * w_value),
         1 - 2 * (x_value**2 + y_value**2)],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = t
    return matrix


def _resolve_transform(
        graph: Mapping[str, list[tuple[str, np.ndarray]]],
        source: str,
        target: str) -> np.ndarray:
    source = _frame_name(source)
    target = _frame_name(target)
    if source == target:
        return np.eye(4)
    queue = deque([(source, np.eye(4))])
    visited = {source}
    while queue:
        frame, frame_from_source = queue.popleft()
        for neighbor, neighbor_from_frame in graph.get(frame, ()):
            if neighbor in visited:
                continue
            neighbor_from_source = neighbor_from_frame @ frame_from_source
            if neighbor == target:
                return neighbor_from_source
            visited.add(neighbor)
            queue.append((neighbor, neighbor_from_source))
    raise ValueError(f'no static transform from {source} to {target}')


def _base_to_map(
        point: Sequence[float],
        pose: tuple[float, float, float]) -> tuple[float, float]:
    x_value, y_value, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        x_value + cosine * point[0] - sine * point[1],
        y_value + sine * point[0] + cosine * point[1],
    )


def _ray_cells(
        start_world: tuple[float, float],
        end_world: tuple[float, float],
        shape: tuple[int, int],
        resolution: float,
        origin: tuple[float, float, float],
        ) -> tuple[set[tuple[int, int]], tuple[int, int] | None]:
    start = _world_to_local(start_world, origin)
    end = _world_to_local(end_world, origin)
    height, width = shape
    bounds = (width * resolution, height * resolution)
    end_inside = _inside(end, bounds)
    clipped = _clip_segment(start, end, bounds)
    if clipped is None:
        return set(), None
    clipped_start, clipped_end = clipped
    start_cell = _local_to_cell(clipped_start, shape, resolution)
    end_cell = _local_to_cell(clipped_end, shape, resolution)
    cells = list(_bresenham(start_cell, end_cell))
    if end_inside:
        return set(cells[:-1]), cells[-1]
    return set(cells), None


def _world_to_local(
        point: tuple[float, float],
        origin: tuple[float, float, float]) -> tuple[float, float]:
    delta_x = point[0] - origin[0]
    delta_y = point[1] - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )


def _inside(point: tuple[float, float], bounds: tuple[float, float]) -> bool:
    return 0.0 <= point[0] < bounds[0] and 0.0 <= point[1] < bounds[1]


def _clip_segment(
        start: tuple[float, float],
        end: tuple[float, float],
        bounds: tuple[float, float],
        ) -> tuple[tuple[float, float], tuple[float, float]] | None:
    """Clip a segment to the half-open map rectangle with Liang-Barsky."""
    high_x = math.nextafter(bounds[0], 0.0)
    high_y = math.nextafter(bounds[1], 0.0)
    delta = (end[0] - start[0], end[1] - start[1])
    lower = 0.0
    upper = 1.0
    limits = (
        (-delta[0], start[0]),
        (delta[0], high_x - start[0]),
        (-delta[1], start[1]),
        (delta[1], high_y - start[1]),
    )
    for coefficient, distance in limits:
        if coefficient == 0.0:
            if distance < 0.0:
                return None
            continue
        ratio = distance / coefficient
        if coefficient < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return None
    def point_at(parameter: float) -> tuple[float, float]:
        return (
            min(max(start[0] + parameter * delta[0], 0.0), high_x),
            min(max(start[1] + parameter * delta[1], 0.0), high_y),
        )

    return point_at(lower), point_at(upper)


def _local_to_cell(
        point: tuple[float, float],
        shape: tuple[int, int],
        resolution: float) -> tuple[int, int]:
    height, _width = shape
    column = math.floor(point[0] / resolution)
    map_row = math.floor(point[1] / resolution)
    return height - 1 - map_row, column


def _bresenham(
        start: tuple[int, int],
        end: tuple[int, int]) -> Iterable[tuple[int, int]]:
    row, column = start
    end_row, end_column = end
    delta_column = abs(end_column - column)
    step_column = 1 if column < end_column else -1
    delta_row = -abs(end_row - row)
    step_row = 1 if row < end_row else -1
    error = delta_column + delta_row
    while True:
        yield row, column
        if row == end_row and column == end_column:
            return
        doubled = 2 * error
        if doubled >= delta_row:
            error += delta_row
            column += step_column
        if doubled <= delta_column:
            error += delta_column
            row += step_row
