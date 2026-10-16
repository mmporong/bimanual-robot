"""Pure rear-facing LiDAR geometry for a low docking box."""

from collections import deque
import math

from jdamr_cube_navigation.box_lidar_witness import (
    MAX_CANDIDATE_PLANE_DISTANCE_M,
    MAXIMUM_RESIDUAL_RMS_M,
    MINIMUM_SUPPORT,
    MINIMUM_TANGENT_SPREAD_M,
)
import numpy as np


PROVENANCE = 'REAR_LIDAR_FACE_RELATIVE_GEOMETRY'
RANSAC_ITERATIONS = 160
RANSAC_REFINEMENT_CANDIDATES = 8


def _finite(value, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f'{label} must be finite')
    return float(value)


def _positive(value, label):
    value = _finite(value, label)
    if value <= 0.0:
        raise ValueError(f'{label} must be positive')
    return value


def _laser_mount(geometry):
    if not isinstance(geometry, dict):
        raise ValueError('geometry must be a mapping')
    try:
        translation = geometry['laser_translation']
        yaw = geometry['laser_yaw']
    except KeyError as error:
        raise ValueError('geometry is missing laser mounting data') from error
    if (not isinstance(translation, dict) or translation.get('unit') != 'm'
            or not isinstance(yaw, dict) or yaw.get('unit') != 'rad'):
        raise ValueError('laser mounting data must use meters and radians')
    xyz = translation.get('value')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise ValueError('laser translation must contain XYZ')
    return (
        _finite(xyz[0], 'laser_translation[0]'),
        _finite(xyz[1], 'laser_translation[1]'),
        _finite(yaw.get('value'), 'laser_yaw'),
    )


def _scan_points_base(scan, geometry):
    if not isinstance(scan, dict):
        raise ValueError('scan must be a mapping')
    angle_min = _finite(scan.get('angle_min'), 'angle_min')
    angle_increment = _finite(scan.get('angle_increment'), 'angle_increment')
    range_min = _finite(scan.get('range_min'), 'range_min')
    range_max = _finite(scan.get('range_max'), 'range_max')
    if angle_increment == 0.0:
        raise ValueError('angle_increment must be nonzero')
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError('laser range bounds are invalid')
    ranges = scan.get('ranges')
    if not isinstance(ranges, (list, tuple)) or not ranges:
        raise ValueError('ranges must be a nonempty sequence')

    laser_x, laser_y, laser_yaw = _laser_mount(geometry)
    cosine = math.cos(laser_yaw)
    sine = math.sin(laser_yaw)
    points = []
    for index, raw_range in enumerate(ranges):
        if (isinstance(raw_range, bool)
                or not isinstance(raw_range, (int, float))
                or not math.isfinite(raw_range)
                or raw_range < range_min or raw_range > range_max):
            continue
        angle = angle_min + index * angle_increment
        scan_x = raw_range * math.cos(angle)
        scan_y = raw_range * math.sin(angle)
        points.append((
            laser_x + cosine * scan_x - sine * scan_y,
            laser_y + sine * scan_x + cosine * scan_y,
        ))
    if not points:
        raise ValueError('scan has no valid ranges')
    return np.asarray(points, dtype=np.float64)


def _canonical_line(points):
    center = np.mean(points, axis=0)
    _left, _singular, vectors = np.linalg.svd(
        points - center, full_matrices=False)
    tangent = vectors[0]
    normal = np.array((tangent[1], -tangent[0]), dtype=np.float64)
    if normal[0] < 0.0:
        normal = -normal
        tangent = -tangent
    # A rear docking face crosses the robot centreline. A longitudinal wall is
    # a different object even when it happens to fall inside the rear ROI.
    if normal[0] <= abs(normal[1]):
        raise ValueError('rear candidate is not a transverse face')
    offset = float(np.median(points @ normal))
    residuals = np.abs(points @ normal - offset)
    tangent_positions = points @ tangent
    return (
        tangent,
        normal,
        offset,
        float(np.sqrt(np.mean(residuals ** 2))),
        float(np.ptp(tangent_positions)),
    )


def _fit_rear_face(points):
    if len(points) < MINIMUM_SUPPORT:
        raise ValueError('rear face support is below five points')
    random = np.random.default_rng(0)
    first_indices = random.integers(0, len(points), size=RANSAC_ITERATIONS)
    second_indices = random.integers(
        0, len(points) - 1, size=RANSAC_ITERATIONS)
    second_indices += second_indices >= first_indices
    deltas = points[second_indices] - points[first_indices]
    lengths = np.linalg.norm(deltas, axis=1)
    valid_lengths = lengths > 1e-9
    tangents = np.zeros_like(deltas)
    tangents[valid_lengths] = (
        deltas[valid_lengths] / lengths[valid_lengths, None])
    normals = np.column_stack((tangents[:, 1], -tangents[:, 0]))
    normals[normals[:, 0] < 0.0] *= -1.0
    transverse = normals[:, 0] > np.abs(normals[:, 1])
    offsets = np.einsum('ij,ij->i', points[first_indices], normals)
    residuals = np.abs(points @ normals.T - offsets)
    masks = residuals <= MAXIMUM_RESIDUAL_RMS_M
    support_counts = np.count_nonzero(masks, axis=0)
    candidate_indices = np.flatnonzero(
        valid_lengths & transverse & (support_counts >= MINIMUM_SUPPORT))
    candidate_indices = candidate_indices[
        np.argsort(support_counts[candidate_indices])[::-1]
    ][:RANSAC_REFINEMENT_CANDIDATES]

    best = None
    for index in candidate_indices:
        candidate_points = points[masks[:, index]]
        try:
            fitted = _canonical_line(candidate_points)
        except ValueError:
            continue
        candidate = (
            int(support_counts[index]), -fitted[3], fitted, candidate_points)
        if best is None or candidate[:2] > best[:2]:
            best = candidate
    if best is None:
        raise ValueError('rear face line fit failed')

    _support, _negative_residual, fitted, selected = best
    # A robust face must explain a clear majority of the rear ROI. This still
    # permits sparse reflections while preventing one tangent of a curved or
    # multi-object cluster from being mistaken for the complete dock face.
    if len(selected) * 5 < len(points) * 3:
        raise ValueError('rear face line has no majority support')
    tangent, normal, offset, residual_rms_m, span_m = fitted
    if residual_rms_m > MAXIMUM_RESIDUAL_RMS_M:
        raise ValueError('rear face line residual is too large')
    if span_m < MINIMUM_TANGENT_SPREAD_M:
        raise ValueError('rear face tangent spread is too narrow')
    return tangent, normal, offset, residual_rms_m, span_m, selected


def _nearest_rear_layer(points):
    """Keep the foreground layer so a larger rear wall cannot win RANSAC."""
    if len(points) == 0:
        return points, 0
    ordered = points[np.argsort(points[:, 0])[::-1]]
    depth_steps = ordered[:-1, 0] - ordered[1:, 0]
    splits = np.flatnonzero(
        depth_steps > MAX_CANDIDATE_PLANE_DISTANCE_M)
    if len(splits) == 0:
        return ordered, 1
    return ordered[:int(splits[0]) + 1], int(len(splits) + 1)


def estimate_rear_box_face(
        scan, *, geometry, rear_extent_m, rear_half_width_m,
        diagnostics=None):
    """Estimate the nearest transverse face behind the rectangular base.

    Coordinates are in ``base_footprint``. The normal always points from the
    box toward the robot (+X), so ``gap_m`` is the minimum separation between
    the fitted line and the rear edge/corners of the base rectangle.
    """
    rear_extent_m = _positive(rear_extent_m, 'rear_extent_m')
    rear_half_width_m = _positive(rear_half_width_m, 'rear_half_width_m')
    if diagnostics is not None and not isinstance(diagnostics, dict):
        raise ValueError('diagnostics must be a mapping')

    points = _scan_points_base(scan, geometry)
    roi = points[
        (points[:, 0] <= -rear_extent_m)
        & (np.abs(points[:, 1]) <= rear_half_width_m)
    ]
    foreground, layer_count = _nearest_rear_layer(roi)
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update({
            'valid_scan_points': int(len(points)),
            'rear_roi_points': int(len(roi)),
            'foreground_layer_points': int(len(foreground)),
            'rear_depth_layer_count': layer_count,
            'minimum_support': MINIMUM_SUPPORT,
            'minimum_span_m': MINIMUM_TANGENT_SPREAD_M,
            'maximum_residual_rms_m': MAXIMUM_RESIDUAL_RMS_M,
        })
    tangent, normal, offset, residual_rms_m, span_m, selected = _fit_rear_face(
        foreground)

    robot_minimum_projection = (
        -rear_extent_m * normal[0]
        - rear_half_width_m * abs(normal[1])
    )
    gap_m = robot_minimum_projection - offset
    face_center = np.array((offset / normal[0], 0.0))
    center_gap_m = -rear_extent_m - face_center[0]
    support_center = np.mean(selected, axis=0)
    yaw_error_rad = math.atan2(normal[1], normal[0])
    line_anchor = normal * offset
    result = {
        'face_center_base_xy_m': tuple(float(value) for value in face_center),
        'support_center_base_xy_m': tuple(
            float(value) for value in support_center),
        'normal_base_xy': tuple(float(value) for value in normal),
        'tangent_base_xy': tuple(float(value) for value in tangent),
        'line_anchor_base_xy_m': tuple(float(value) for value in line_anchor),
        'line_offset_m': offset,
        'normal_yaw_error_rad': yaw_error_rad,
        'gap_m': gap_m,
        'center_gap_m': center_gap_m,
        'span_m': span_m,
        'support_count': int(len(selected)),
        'residual_rms_m': residual_rms_m,
        'provenance': PROVENANCE,
    }
    if diagnostics is not None:
        diagnostics.update(result)
    return result


class RearBoxFaceTracker:
    """Require a stable relative line across consecutive stopped scans."""

    def __init__(self, required_frames=8, maximum_offset_spread_m=0.03,
                 maximum_angle_spread_deg=3.0):
        if (isinstance(required_frames, bool)
                or not isinstance(required_frames, int)
                or required_frames <= 0):
            raise ValueError('required_frames must be a positive integer')
        self.required_frames = required_frames
        self.maximum_offset_spread_m = _positive(
            maximum_offset_spread_m, 'maximum_offset_spread_m')
        self.maximum_angle_spread_rad = math.radians(_positive(
            maximum_angle_spread_deg, 'maximum_angle_spread_deg'))
        self._history = deque(maxlen=required_frames)

    def update(self, face):
        if face is None:
            self._history.clear()
            return False
        if not isinstance(face, dict):
            raise ValueError('face must be a mapping')
        normal = np.asarray(face.get('normal_base_xy'), dtype=np.float64)
        tangent = np.asarray(face.get('tangent_base_xy'), dtype=np.float64)
        if (normal.shape != (2,) or tangent.shape != (2,)
                or not np.all(np.isfinite(normal))
                or not np.all(np.isfinite(tangent))
                or not math.isclose(float(np.linalg.norm(normal)), 1.0, abs_tol=1e-3)
                or not math.isclose(float(np.linalg.norm(tangent)), 1.0, abs_tol=1e-3)):
            raise ValueError('face line directions must be finite unit vectors')
        offset = _finite(face.get('line_offset_m'), 'line_offset_m')
        if abs(float(normal @ tangent)) > 1e-3:
            raise ValueError('face line directions must be perpendicular')
        if normal[0] < 0.0:
            normal = -normal
            offset = -offset
        self._history.append((normal, tangent, offset))
        if len(self._history) < self.required_frames:
            return False

        normals = np.asarray([item[0] for item in self._history])
        tangents = np.asarray([item[1] for item in self._history])
        offsets = np.asarray([item[2] for item in self._history])
        normal_angles = np.arccos(np.clip(
            np.abs(normals @ normals.T), 0.0, 1.0))
        tangent_angles = np.arccos(np.clip(
            np.abs(tangents @ tangents.T), 0.0, 1.0))
        stable = (
            float(np.ptp(offsets)) <= self.maximum_offset_spread_m
            and float(np.max(normal_angles)) <= self.maximum_angle_spread_rad
            and float(np.max(tangent_angles)) <= self.maximum_angle_spread_rad
        )
        if not stable:
            latest = self._history[-1]
            self._history.clear()
            self._history.append(latest)
        return stable

    @property
    def frame_count(self):
        return len(self._history)
