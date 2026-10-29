"""Associate fresh tabletop edges in odom, with bounded diagnostic prediction."""

from dataclasses import dataclass
import math

import numpy as np


def rigid_transform(value):
    """Validate an SE(3) transform rather than accepting scale or reflection."""
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], (0., 0., 0., 1.), atol=1e-8, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3],
                               np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(np.linalg.det(matrix[:3, :3]), 1., abs_tol=1e-6)):
        raise ValueError('a finite rigid odom-from-optical transform is required')
    return matrix.copy()


@dataclass(frozen=True)
class TrackConfig:
    """Association bounds; prediction is never an observed control input."""

    maximum_line_error_m: float = 0.06
    maximum_angle_rad: float = math.radians(5.)
    maximum_height_error_m: float = 0.03
    minimum_segment_overlap_m: float = 0.02
    prediction_horizon_s: float = 0.25
    maximum_prediction_translation_m: float = 0.03
    maximum_prediction_rotation_rad: float = math.radians(5.)
    stable_line_error_m: float = 0.03
    stable_angle_rad: float = math.radians(3.)
    required_frames: int = 8

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if name == 'required_frames':
                if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                    raise ValueError('required_frames must be a positive integer')
            elif isinstance(value, bool) or not math.isfinite(value) or value <= 0.:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class TargetRegion:
    """A selected table's finite search disk in odom, never a distance truth."""

    center_xy_m: tuple[float, float]
    radius_m: float

    def __post_init__(self):
        if (len(self.center_xy_m) != 2
                or any(isinstance(v, bool) or not math.isfinite(v) for v in self.center_xy_m)
                or isinstance(self.radius_m, bool) or not math.isfinite(self.radius_m)
                or self.radius_m <= 0.):
            raise ValueError('target region must have a finite center and positive radius')


@dataclass(frozen=True)
class TableTrack:
    """Keep measurement and frame timestamps separate even during occlusion."""

    state: str
    track_id: int | None
    measurement_stamp_s: float | None
    frame_stamp_s: float
    stable_frames: int
    detection: object | None
    endpoints_odom_m: tuple | None


def _geometry(detection, transform):
    endpoints = np.asarray(detection.edge_endpoints_optical_m, dtype=np.float64)
    if endpoints.shape != (2, 3) or not np.all(np.isfinite(endpoints)):
        raise ValueError('edge endpoints must be finite 2x3')
    world = endpoints @ transform[:3, :3].T + transform[:3, 3]
    tangent = world[1, :2] - world[0, :2]
    length = np.linalg.norm(tangent)
    if length <= 1e-6:
        raise ValueError('edge must have positive horizontal extent')
    tangent /= length
    normal = np.array([-tangent[1], tangent[0]])
    return world, tangent, normal


def candidates_in_region(candidates, transform, region):
    """Use distance to the finite observed segment, not its cropped midpoint."""
    transform = rigid_transform(transform)
    result = []
    center = np.asarray(region.center_xy_m)
    for item in candidates:
        world, tangent, _ = _geometry(item, transform)
        length = np.linalg.norm(world[1, :2] - world[0, :2])
        along = np.clip(np.dot(center - world[0, :2], tangent), 0., length)
        nearest = world[0, :2] + along * tangent
        if np.linalg.norm(center - nearest) <= region.radius_m:
            result.append(item)
    return tuple(result)


class TableSurfaceTracker:
    """Preserve an identity across cropping; do not silently switch targets."""

    def __init__(self, config=TrackConfig()):
        self.config = config
        self._next_id = 1
        self.reset()

    def reset(self):
        """Forget observations on mount/clock/selection changes."""
        self._last_stamp = None
        self._calibration_id = None
        self._world = None
        self._normal = None
        self._tangent = None
        self._seen_at = None
        self._seen_transform = None
        self._id = None
        self._count = 0
        self._region = None

    def update(self, candidates, stamp_s, odom_from_optical, calibration_id, region=None):
        """Select only an unambiguous current observation of the same edge."""
        if isinstance(stamp_s, bool) or not math.isfinite(stamp_s) or stamp_s <= 0:
            raise ValueError('frame stamp must be finite and positive')
        if not isinstance(calibration_id, str) or not calibration_id:
            raise ValueError('calibration identity is required')
        transform = rigid_transform(odom_from_optical)
        if self._calibration_id not in (None, calibration_id):
            self.reset()
        if region != self._region:
            self.reset()
            self._region = region
        self._calibration_id = calibration_id
        if self._last_stamp is not None and stamp_s <= self._last_stamp:
            return self._result('nonmonotonic_input', stamp_s)
        self._last_stamp = stamp_s
        if region is not None:
            candidates = candidates_in_region(candidates, transform, region)
        geometries = [(item, *_geometry(item, transform)) for item in candidates]
        if self._world is None:
            if len(geometries) != 1:
                return self._result('ambiguous' if geometries else 'unobservable', stamp_s)
            selected = geometries[0]
            self._id, self._next_id = self._next_id, self._next_id + 1
        else:
            matches = []
            for candidate in geometries:
                _, world, tangent, normal = candidate
                dot = float(np.clip(abs(np.dot(normal, self._normal)), 0., 1.))
                angle = math.acos(dot)
                line_error = float(np.max(np.abs(
                    (world[:, :2] - self._world[0, :2]) @ self._normal)))
                old_interval = np.sort(self._world[:, :2] @ self._tangent)
                new_interval = np.sort(world[:, :2] @ self._tangent)
                overlap = (min(old_interval[1], new_interval[1])
                           - max(old_interval[0], new_interval[0]))
                height_error = abs(float(np.mean(world[:, 2]) - np.mean(self._world[:, 2])))
                if (angle <= self.config.maximum_angle_rad
                        and line_error <= self.config.maximum_line_error_m
                        and overlap >= self.config.minimum_segment_overlap_m
                        and height_error <= self.config.maximum_height_error_m):
                    matches.append((candidate, line_error, angle))
            if len(matches) != 1:
                self._count = 0
                if len(matches) > 1:
                    return self._result('ambiguous', stamp_s)
                if geometries:
                    return self._result('target_not_observed', stamp_s)
                return self._prediction(stamp_s, transform)
            selected, line_error, angle = matches[0]
            if (line_error > self.config.stable_line_error_m
                    or angle > self.config.stable_angle_rad):
                self._count = 0
        detection, self._world, self._tangent, self._normal = selected
        self._seen_at, self._seen_transform = stamp_s, transform
        self._count += 1
        return self._result('observed', stamp_s, detection)

    def _prediction(self, stamp_s, transform):
        translation = np.linalg.norm(transform[:3, 3] - self._seen_transform[:3, 3])
        relative = self._seen_transform[:3, :3].T @ transform[:3, :3]
        angle = math.acos(float(np.clip((np.trace(relative) - 1.) / 2., -1., 1.)))
        allowed = (stamp_s - self._seen_at <= self.config.prediction_horizon_s
                   and translation <= self.config.maximum_prediction_translation_m
                   and angle <= self.config.maximum_prediction_rotation_rad)
        return self._result('predicted' if allowed else 'expired', stamp_s)

    def _result(self, state, stamp_s, detection=None):
        endpoints = None if self._world is None else tuple(
            tuple(float(v) for v in point) for point in self._world)
        return TableTrack(state, self._id, self._seen_at, stamp_s, self._count,
                          detection, endpoints)
