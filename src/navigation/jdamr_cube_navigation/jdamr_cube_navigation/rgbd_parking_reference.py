"""Estimate a camera pose delta from a fixed RGB-D parking reference.

The registered map pose remains the destination.  Features inside the named
reference ROIs only provide a local pose observation; this module never moves
the target to an object centroid.

``T_current_from_reference`` maps a 3-D point expressed in the reference
optical frame into the current optical frame::

    point_current = R @ point_reference + t

Consequently, with ``world_from_current`` as the live camera pose,
``world_from_reference = world_from_current @ T_current_from_reference``.
The same transform is therefore the local camera correction from the current
pose toward the recorded pose.  Its inverse, ``T_reference_from_current``, is
the observed current-camera pose in reference coordinates.  Keeping both in
the result prevents the common translation-sign mistake.
"""

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import cv2

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Rectified pinhole calibration used by the RGB-D reference estimator."""

    matrix: np.ndarray
    width: int | None = None
    height: int | None = None
    distortion: np.ndarray | None = None
    frame_id: str = ''

    @classmethod
    def from_value(cls, value):
        """Accept a 3x3 matrix or a CameraInfo-like mapping/object."""
        if isinstance(value, cls):
            return value.validated()
        if isinstance(value, Mapping):
            get = value.get
        elif hasattr(value, 'k') or hasattr(value, 'p'):
            def get(name, default=None):
                return getattr(value, name, default)
        else:
            array = np.asarray(value, dtype=np.float64)
            return cls(array).validated()

        projection = get('p', get('P'))
        intrinsic = get('k', get('K'))
        source = (projection
                  if projection is not None and len(projection)
                  else intrinsic)
        if source is None:
            raise ValueError('intrinsics_missing_matrix')
        array = np.asarray(source, dtype=np.float64)
        if array.size == 12:
            projection = array.reshape(3, 4)
            if not np.allclose(projection[:, 3], 0., atol=1e-9, rtol=0):
                raise ValueError('intrinsics_projection_offset_unsupported')
            array = projection[:, :3]
        distortion = get('d', get('D'))
        if distortion is not None:
            distortion = np.asarray(
                distortion, dtype=np.float64).reshape(-1)
        frame_id = get('frame', get('frame_id', '')) or ''
        header = get('header')
        if not frame_id and header is not None:
            frame_id = getattr(header, 'frame_id', '') or ''
        return cls(
            array.reshape(3, 3),
            _optional_positive_int(get('width')),
            _optional_positive_int(get('height')),
            distortion,
            str(frame_id),
        ).validated()

    def validated(self):
        """Return a defensive, validated calibration copy."""
        matrix = np.asarray(self.matrix, dtype=np.float64)
        distortion = (
            np.zeros(5, dtype=np.float64)
            if self.distortion is None
            else np.asarray(self.distortion, dtype=np.float64).reshape(-1)
        )
        if (matrix.shape != (3, 3) or not np.all(np.isfinite(matrix))
                or matrix[0, 0] <= 0. or matrix[1, 1] <= 0.
                or not np.allclose(
                    matrix[2], (0., 0., 1.), atol=1e-8, rtol=0)
                or abs(np.linalg.det(matrix)) <= 1e-12
                or not np.all(np.isfinite(distortion))):
            raise ValueError('invalid_intrinsics')
        if not np.allclose(distortion, 0., atol=1e-12, rtol=0):
            raise ValueError(
                'nonzero_distortion_requires_rectified_images')
        if self.width is not None and self.width <= 0:
            raise ValueError('invalid_intrinsics_dimensions')
        if self.height is not None and self.height <= 0:
            raise ValueError('invalid_intrinsics_dimensions')
        return CameraIntrinsics(
            matrix.copy(), self.width, self.height, distortion.copy(),
            self.frame_id)


@dataclass(frozen=True)
class ReferencePoseConfig:
    """Quality gates for a near-destination reference observation."""

    orb_features: int = 2000
    ratio_threshold: float = 0.75
    depth_scale_m: float = 0.001
    depth_neighborhood_radius_px: int = 2
    minimum_depth_samples: int = 3
    minimum_depth_m: float = 0.20
    maximum_depth_m: float = 4.0
    minimum_correspondences: int = 12
    minimum_inliers: int = 10
    minimum_inliers_per_roi: int = 3
    minimum_inlier_ratio: float = 0.45
    ransac_reprojection_error_px: float = 3.0
    maximum_reprojection_rmse_px: float = 2.5
    minimum_object_spread_m: float = 0.015
    minimum_object_thickness_m: float = 0.008
    minimum_object_thickness_ratio: float = 0.02
    minimum_image_spread_px: float = 18.0
    maximum_translation_m: float = 0.75
    maximum_rotation_rad: float = math.radians(45.)

    def __post_init__(self):
        """Reject configurations that would disable a quality gate."""
        integers = ('orb_features', 'depth_neighborhood_radius_px',
                    'minimum_depth_samples', 'minimum_correspondences',
                    'minimum_inliers', 'minimum_inliers_per_roi')
        for name in integers:
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, int)
                    or value <= 0):
                raise ValueError(f'{name}_must_be_positive_integer')
        for name, value in self.__dict__.items():
            if name in integers:
                continue
            if (isinstance(value, bool) or not math.isfinite(value)
                    or value <= 0.):
                raise ValueError(f'{name}_must_be_finite_and_positive')
        if self.ratio_threshold >= 1. or self.minimum_inlier_ratio > 1.:
            raise ValueError('ratio_thresholds_must_not_exceed_one')
        if self.minimum_depth_m >= self.maximum_depth_m:
            raise ValueError('depth_range_is_empty')


@dataclass(frozen=True)
class ReferencePoseEstimate:
    """A valid local correction or an explicit invalidity reason."""

    valid: bool
    reason: str
    T_current_from_reference: np.ndarray | None = None
    T_reference_from_current: np.ndarray | None = None
    match_count: int = 0
    inlier_count: int = 0
    reprojection_rmse_px: float | None = None
    roi_inliers: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True)
class TabletopPlaneConfig:
    """Bounds for a reference-derived tabletop plane candidate."""

    depth_scale_m: float = 0.001
    pixel_stride: int = 2
    minimum_depth_m: float = 0.20
    maximum_depth_m: float = 4.0
    ransac_iterations: int = 300
    ransac_distance_m: float = 0.008
    minimum_points: int = 200
    minimum_inliers: int = 150
    minimum_inlier_ratio: float = 0.45
    maximum_plane_rms_m: float = 0.006
    minimum_forward_projection_norm: float = 0.15
    maximum_planar_residual_rad: float = math.radians(5.)
    random_seed: int = 0

    def __post_init__(self):
        """Reject disabled or internally inconsistent plane gates."""
        positive_integers = (
            'pixel_stride', 'ransac_iterations', 'minimum_points',
            'minimum_inliers',
        )
        for name in positive_integers:
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, int)
                    or value <= 0):
                raise ValueError(f'{name}_must_be_positive_integer')
        if (isinstance(self.random_seed, bool)
                or not isinstance(self.random_seed, int)):
            raise ValueError('random_seed_must_be_integer')
        for name in (
                'depth_scale_m', 'minimum_depth_m', 'maximum_depth_m',
                'ransac_distance_m', 'minimum_inlier_ratio',
                'maximum_plane_rms_m', 'minimum_forward_projection_norm',
                'maximum_planar_residual_rad'):
            value = getattr(self, name)
            if (isinstance(value, bool) or not math.isfinite(value)
                    or value <= 0.):
                raise ValueError(f'{name}_must_be_finite_and_positive')
        if self.minimum_depth_m >= self.maximum_depth_m:
            raise ValueError('depth_range_is_empty')
        if self.minimum_inlier_ratio > 1.:
            raise ValueError('minimum_inlier_ratio_must_not_exceed_one')


@dataclass(frozen=True)
class PlanarBasisEstimate:
    """Candidate base/optical basis inferred from one reference tabletop."""

    valid: bool
    reason: str
    R_base_from_optical: np.ndarray | None = None
    plane_normal_up_optical: np.ndarray | None = None
    forward_optical: np.ndarray | None = None
    left_optical: np.ndarray | None = None
    plane_offset_m: float | None = None
    point_count: int = 0
    inlier_count: int = 0
    inlier_ratio: float = 0.
    plane_rms_m: float | None = None
    camera_yaw_rad: float | None = None
    camera_lever_xy_m: tuple[float, float] | None = None
    basis_source: str = 'reference_tabletop_plane_candidate'
    mount_calibration_complete: bool = False
    millimeter_accuracy_validated: bool = False


@dataclass(frozen=True)
class PlanarBaseErrorEstimate:
    """Planar base correction derived from a valid local camera correction."""

    valid: bool
    reason: str
    forward_m: float | None = None
    left_m: float | None = None
    yaw_rad: float | None = None
    pitch_roll_residual_rad: float | None = None
    basis_source: str = 'reference_tabletop_plane_candidate'
    camera_yaw_rad: float | None = None
    camera_lever_xy_m: tuple[float, float] | None = None
    mount_calibration_complete: bool = False
    millimeter_accuracy_validated: bool = False


def _optional_positive_int(value):
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError('invalid_intrinsics_dimensions')
    return int(value)


def _invalid(reason, *, matches=0, inliers=0, rmse=None, roi_inliers=()):
    return ReferencePoseEstimate(
        False, reason, match_count=int(matches), inlier_count=int(inliers),
        reprojection_rmse_px=rmse, roi_inliers=tuple(roi_inliers))


def _invalid_basis(reason, *, points=0, inliers=0, ratio=0., rms=None,
                   camera_yaw=None, lever=None):
    return PlanarBasisEstimate(
        False, reason, point_count=int(points), inlier_count=int(inliers),
        inlier_ratio=float(ratio), plane_rms_m=rms,
        camera_yaw_rad=camera_yaw, camera_lever_xy_m=lever)


def _invalid_base_error(reason, basis, residual=None):
    return PlanarBaseErrorEstimate(
        False, reason, pitch_roll_residual_rad=residual,
        basis_source=basis.basis_source,
        camera_yaw_rad=basis.camera_yaw_rad,
        camera_lever_xy_m=basis.camera_lever_xy_m,
        mount_calibration_complete=basis.mount_calibration_complete,
        millimeter_accuracy_validated=basis.millimeter_accuracy_validated,
    )


def project_rgb_pixel_to_depth(pixel_xy, rgb_intrinsics, depth_intrinsics):
    """Map a rectified RGB pixel to the registered depth image by its ray.

    This intentionally uses the two calibrations rather than an image-size
    ratio.  Both streams must already describe the same optical frame.
    """
    rgb = CameraIntrinsics.from_value(rgb_intrinsics)
    depth = CameraIntrinsics.from_value(depth_intrinsics)
    if rgb.frame_id and depth.frame_id and rgb.frame_id != depth.frame_id:
        raise ValueError('rgb_depth_optical_frame_mismatch')
    pixel = np.asarray(pixel_xy, dtype=np.float64)
    if pixel.shape != (2,) or not np.all(np.isfinite(pixel)):
        raise ValueError('invalid_rgb_pixel')
    ray = np.linalg.solve(rgb.matrix, np.array((pixel[0], pixel[1], 1.)))
    projected = depth.matrix @ ray
    if not np.all(np.isfinite(projected)) or projected[2] <= 0.:
        raise ValueError('invalid_calibration_ray')
    return projected[:2] / projected[2]


def _image_roi(bounds, width, height, reason='invalid_tabletop_roi'):
    try:
        values = tuple(bounds)
    except TypeError as error:
        raise ValueError(reason) from error
    if len(values) != 4 or any(isinstance(value, bool) for value in values):
        raise ValueError(reason)
    x0, y0, x1, y1 = (int(value) for value in values)
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError(reason)
    return x0, y0, x1, y1


def _camera_assumptions(camera_yaw_rad, camera_lever_xy_m):
    if (isinstance(camera_yaw_rad, bool)
            or not math.isfinite(camera_yaw_rad)):
        raise ValueError('camera_yaw_must_be_explicit_and_finite')
    lever = np.asarray(camera_lever_xy_m, dtype=np.float64)
    if lever.shape != (2,) or not np.all(np.isfinite(lever)):
        raise ValueError('camera_lever_xy_m_must_be_finite_pair')
    return float(camera_yaw_rad), (float(lever[0]), float(lever[1]))


def _depth_roi_points(depth_image, intrinsics, roi, exclusions, config):
    height, width = depth_image.shape
    x0, y0, x1, y1 = _image_roi(roi, width, height)
    selected = np.zeros((height, width), dtype=bool)
    selected[y0:y1:config.pixel_stride,
             x0:x1:config.pixel_stride] = True
    for bounds in exclusions:
        ex0, ey0, ex1, ey1 = _image_roi(
            bounds, width, height, 'invalid_tabletop_exclusion_roi')
        selected[ey0:ey1, ex0:ex1] = False
    rows, columns = np.nonzero(selected)
    values = np.asarray(depth_image[rows, columns], dtype=np.float64)
    depths = values * config.depth_scale_m
    valid = (
        np.isfinite(depths)
        & (depths >= config.minimum_depth_m)
        & (depths <= config.maximum_depth_m)
    )
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)
    pixels = np.column_stack((columns[valid], rows[valid],
                              np.ones(np.count_nonzero(valid))))
    rays = np.linalg.solve(intrinsics.matrix, pixels.T).T
    points = rays * (depths[valid] / rays[:, 2])[:, None]
    return points[np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0.)]


def _fit_dominant_plane(points, config):
    random = np.random.default_rng(config.random_seed)
    best_indices = None
    best_error = math.inf
    for _ in range(config.ransac_iterations):
        sample = points[random.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm <= 1e-9:
            continue
        normal /= norm
        distances = np.abs((points - sample[0]) @ normal)
        indices = np.flatnonzero(distances <= config.ransac_distance_m)
        if len(indices) == 0:
            continue
        median = float(np.median(distances[indices]))
        if (best_indices is None or len(indices) > len(best_indices)
                or (len(indices) == len(best_indices)
                    and median < best_error)):
            best_indices = indices
            best_error = median
    if best_indices is None:
        return None
    indices = best_indices
    for _ in range(2):
        centroid = np.mean(points[indices], axis=0)
        _, _, vectors = np.linalg.svd(points[indices] - centroid,
                                      full_matrices=False)
        normal = vectors[-1]
        distances = np.abs((points - centroid) @ normal)
        indices = np.flatnonzero(distances <= config.ransac_distance_m)
        if len(indices) < 3:
            return None
    centroid = np.mean(points[indices], axis=0)
    _, _, vectors = np.linalg.svd(points[indices] - centroid,
                                  full_matrices=False)
    normal = vectors[-1]
    if np.dot(normal, centroid) > 0.:
        normal = -normal
    distances = (points[indices] - centroid) @ normal
    rms = float(np.sqrt(np.mean(distances ** 2)))
    offset = -float(np.dot(normal, centroid))
    return normal, offset, indices, rms


def estimate_reference_planar_basis(
        reference_depth, depth_intrinsics, tabletop_depth_roi,
        *, camera_yaw_rad, camera_lever_xy_m,
        exclusion_depth_rois=(), config=TabletopPlaneConfig()):
    """Infer a candidate planar base basis from a reference tabletop.

    The ROI is semantic input supplied for the known tabletop.  The plane is
    not a general geometry detector and the result is not a completed mount
    calibration.  ``camera_yaw_rad`` and the camera lever arm are explicit
    external assumptions; camera height cancels from the planar conversion.
    """
    try:
        yaw, lever = _camera_assumptions(
            camera_yaw_rad, camera_lever_xy_m)
        intrinsics = CameraIntrinsics.from_value(depth_intrinsics)
        depth = np.asarray(reference_depth)
        if (depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number)
                or (intrinsics.width is not None
                    and intrinsics.width != depth.shape[1])
                or (intrinsics.height is not None
                    and intrinsics.height != depth.shape[0])):
            raise ValueError('invalid_reference_depth')
        points = _depth_roi_points(
            depth, intrinsics, tabletop_depth_roi, exclusion_depth_rois,
            config)
    except (TypeError, ValueError, np.linalg.LinAlgError) as error:
        return _invalid_basis(str(error))
    if len(points) < config.minimum_points:
        return _invalid_basis(
            'insufficient_tabletop_points', points=len(points),
            camera_yaw=yaw, lever=lever)
    fitted = _fit_dominant_plane(points, config)
    if fitted is None:
        return _invalid_basis(
            'tabletop_plane_fit_failed', points=len(points),
            camera_yaw=yaw, lever=lever)
    up, offset, inliers, rms = fitted
    ratio = len(inliers) / len(points)
    if (len(inliers) < config.minimum_inliers
            or ratio < config.minimum_inlier_ratio):
        return _invalid_basis(
            'insufficient_tabletop_plane_inliers', points=len(points),
            inliers=len(inliers), ratio=ratio, rms=rms,
            camera_yaw=yaw, lever=lever)
    if not math.isfinite(rms) or rms > config.maximum_plane_rms_m:
        return _invalid_basis(
            'tabletop_plane_rms_too_large', points=len(points),
            inliers=len(inliers), ratio=ratio, rms=rms,
            camera_yaw=yaw, lever=lever)
    optical_forward = np.array((0., 0., 1.))
    forward = optical_forward - np.dot(optical_forward, up) * up
    forward_norm = np.linalg.norm(forward)
    if (not math.isfinite(forward_norm)
            or forward_norm < config.minimum_forward_projection_norm):
        return _invalid_basis(
            'tabletop_forward_axis_unobservable', points=len(points),
            inliers=len(inliers), ratio=ratio, rms=rms,
            camera_yaw=yaw, lever=lever)
    forward /= forward_norm
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)
    level_from_optical = np.vstack((forward, left, up))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    yaw_rotation = np.array([
        [cosine, -sine, 0.],
        [sine, cosine, 0.],
        [0., 0., 1.],
    ])
    base_from_optical = yaw_rotation @ level_from_optical
    rigid = (
        np.all(np.isfinite(base_from_optical))
        and np.allclose(base_from_optical.T @ base_from_optical,
                        np.eye(3), atol=1e-6, rtol=0)
        and math.isclose(np.linalg.det(base_from_optical), 1., abs_tol=1e-6)
    )
    if not rigid:
        return _invalid_basis(
            'invalid_planar_basis', points=len(points), inliers=len(inliers),
            ratio=ratio, rms=rms, camera_yaw=yaw, lever=lever)
    return PlanarBasisEstimate(
        True, 'reference_tabletop_plane_candidate', base_from_optical,
        up.copy(), forward.copy(), left.copy(), offset, len(points),
        len(inliers), ratio, rms, yaw, lever,
    )


def _rigid_transform(value):
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], (0., 0., 0., 1.),
                               atol=1e-8, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3],
                               np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(
                np.linalg.det(matrix[:3, :3]), 1., abs_tol=1e-6)):
        raise ValueError('invalid_rigid_camera_transform')
    return matrix.copy()


def camera_reference_pose_to_planar_base_error(
        T_current_from_reference, basis,
        *, config=TabletopPlaneConfig()):
    """Convert a camera correction into forward, left and yaw base error.

    If ``R_BC`` is the candidate base-from-optical rotation, ``t`` and ``R``
    come from ``T_current_from_reference``, and ``lever`` is the camera origin
    in base coordinates, the desired reference-base pose in current-base is::

        yaw_R = planar_yaw(R_BC @ R @ R_BC.T)
        delta = R_BC @ t + lever - yaw_R @ lever

    Height is not needed for the planar result.  A large residual pitch/roll
    invalidates the observation instead of being silently flattened.
    """
    if not isinstance(basis, PlanarBasisEstimate) or not basis.valid:
        fallback = (basis if isinstance(basis, PlanarBasisEstimate)
                    else PlanarBasisEstimate(False, 'invalid_basis'))
        return _invalid_base_error('invalid_planar_basis', fallback)
    try:
        transform = _rigid_transform(T_current_from_reference)
        base_from_optical = np.asarray(
            basis.R_base_from_optical, dtype=np.float64)
        if (base_from_optical.shape != (3, 3)
                or not np.all(np.isfinite(base_from_optical))):
            raise ValueError('invalid_planar_basis')
        _, lever_xy = _camera_assumptions(
            basis.camera_yaw_rad, basis.camera_lever_xy_m)
    except (TypeError, ValueError, np.linalg.LinAlgError) as error:
        return _invalid_base_error(str(error), basis)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    relative_base_rotation = (
        base_from_optical @ rotation @ base_from_optical.T)
    yaw = math.atan2(relative_base_rotation[1, 0],
                     relative_base_rotation[0, 0])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    yaw_rotation = np.array([
        [cosine, -sine, 0.],
        [sine, cosine, 0.],
        [0., 0., 1.],
    ])
    residual_rotation = yaw_rotation.T @ relative_base_rotation
    residual = _rotation_angle(residual_rotation)
    if (not math.isfinite(residual)
            or residual > config.maximum_planar_residual_rad):
        return _invalid_base_error(
            'camera_pose_has_excess_pitch_roll', basis, residual)
    lever = np.array((lever_xy[0], lever_xy[1], 0.))
    delta = (base_from_optical @ translation
             + lever - yaw_rotation @ lever)
    if not np.all(np.isfinite(delta)) or not math.isfinite(yaw):
        return _invalid_base_error('nonfinite_planar_base_error', basis)
    return PlanarBaseErrorEstimate(
        True, 'observed', float(delta[0]), float(delta[1]), float(yaw),
        float(residual), basis.basis_source, basis.camera_yaw_rad,
        basis.camera_lever_xy_m, basis.mount_calibration_complete,
        basis.millimeter_accuracy_validated,
    )


def _validate_image(image, intrinsics, name):
    array = np.asarray(image)
    valid_shape = (
        array.ndim == 2
        or (array.ndim == 3 and array.shape[2] in (3, 4))
    )
    if (not valid_shape or array.dtype != np.uint8
            or not np.all(np.isfinite(array))):
        raise ValueError(f'invalid_{name}')
    if ((intrinsics.width is not None and intrinsics.width != array.shape[1])
            or (intrinsics.height is not None
                and intrinsics.height != array.shape[0])):
        raise ValueError(f'{name}_intrinsics_size_mismatch')
    if array.ndim == 2:
        return array
    conversion = (cv2.COLOR_BGRA2GRAY
                  if array.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(array, conversion)


def _validate_rois(rois, width, height):
    if not isinstance(rois, Mapping) or len(rois) < 2:
        raise ValueError('at_least_two_named_reference_rois_required')
    normalized = []
    occupancy = np.zeros((height, width), dtype=np.uint8)
    for name, bounds in rois.items():
        if not isinstance(name, str) or not name:
            raise ValueError('invalid_roi_name')
        values = tuple(bounds)
        if len(values) != 4 or any(isinstance(v, bool) for v in values):
            raise ValueError('invalid_roi_bounds')
        x0, y0, x1, y1 = (int(v) for v in values)
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise ValueError('invalid_roi_bounds')
        if np.any(occupancy[y0:y1, x0:x1]):
            raise ValueError('overlapping_reference_rois')
        occupancy[y0:y1, x0:x1] = 1
        normalized.append((name, (x0, y0, x1, y1)))
    return tuple(normalized), occupancy * 255


def _depth_point(pixel_xy, rgb, depth, depth_image, config):
    depth_xy = project_rgb_pixel_to_depth(pixel_xy, rgb, depth)
    u, v = (int(round(float(value))) for value in depth_xy)
    radius = config.depth_neighborhood_radius_px
    y0 = max(0, v - radius)
    y1 = min(depth_image.shape[0], v + radius + 1)
    x0 = max(0, u - radius)
    x1 = min(depth_image.shape[1], u + radius + 1)
    if x0 >= x1 or y0 >= y1:
        return None
    values = np.asarray(
        depth_image[y0:y1, x0:x1], dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values) & (values > 0.)] * config.depth_scale_m
    values = values[(values >= config.minimum_depth_m)
                    & (values <= config.maximum_depth_m)]
    if values.size < config.minimum_depth_samples:
        return None
    z = float(np.median(values))
    ray = np.linalg.solve(
        depth.matrix, np.array((depth_xy[0], depth_xy[1], 1.)))
    point = ray * (z / ray[2])
    if not np.all(np.isfinite(point)) or point[2] <= 0.:
        return None
    return point


def _rotation_angle(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.) / 2., -1., 1.))
    return math.acos(cosine)


def _has_3d_thickness(points, config):
    centered = points - np.mean(points, axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if len(singular) < 3 or singular[0] <= 0.:
        return False
    thickness_m = singular[2] / math.sqrt(len(points))
    thickness_ratio = singular[2] / singular[0]
    return (thickness_m >= config.minimum_object_thickness_m
            and thickness_ratio >= config.minimum_object_thickness_ratio)


def estimate_pose_from_correspondences(
        reference_points_m, current_pixels, roi_labels, current_intrinsics,
        *, config=ReferencePoseConfig()):
    """Estimate ``T_current_from_reference`` from prepared 3-D/2-D matches."""
    try:
        intrinsics = CameraIntrinsics.from_value(current_intrinsics)
    except (TypeError, ValueError, np.linalg.LinAlgError) as error:
        return _invalid(str(error))
    points = np.asarray(reference_points_m, dtype=np.float64)
    pixels = np.asarray(current_pixels, dtype=np.float64)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float64)
    if pixels.size == 0:
        pixels = np.empty((0, 2), dtype=np.float64)
    labels = tuple(str(label) for label in roi_labels)
    count = len(points) if points.ndim else 0
    if (points.shape != (count, 3) or pixels.shape != (count, 2)
            or len(labels) != count or not np.all(np.isfinite(points))
            or not np.all(np.isfinite(pixels)) or np.any(points[:, 2] <= 0.)):
        return _invalid('invalid_correspondences')
    if count < config.minimum_correspondences:
        return _invalid('insufficient_correspondences', matches=count)
    if len(set(labels)) < 2:
        return _invalid('ambiguous_roi_support', matches=count)
    centered = points - np.mean(points, axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    pixel_span = np.ptp(pixels, axis=0)
    if (len(singular) < 2 or singular[1] < config.minimum_object_spread_m
            or np.linalg.norm(pixel_span) < config.minimum_image_spread_px):
        return _invalid('ambiguous_geometry', matches=count)
    if not _has_3d_thickness(points, config):
        return _invalid('ambiguous_coplanar_geometry', matches=count)
    try:
        ok, rvec, tvec, inlier_array = cv2.solvePnPRansac(
            points, pixels, intrinsics.matrix, intrinsics.distortion,
            flags=cv2.SOLVEPNP_EPNP,
            iterationsCount=200,
            reprojectionError=config.ransac_reprojection_error_px,
            confidence=0.999,
        )
    except cv2.error:
        return _invalid('pnp_failed', matches=count)
    if not ok or inlier_array is None:
        return _invalid('pnp_failed', matches=count)
    inliers = np.asarray(inlier_array, dtype=np.int32).reshape(-1)
    roi_counts = tuple(sorted(
        (label, sum(labels[index] == label for index in inliers))
        for label in set(labels)))
    if (len(inliers) < config.minimum_inliers
            or len(inliers) / count < config.minimum_inlier_ratio):
        return _invalid('insufficient_pnp_inliers', matches=count,
                        inliers=len(inliers), roi_inliers=roi_counts)
    if (len(roi_counts) < 2
            or any(value < config.minimum_inliers_per_roi
                   for _, value in roi_counts)):
        return _invalid('ambiguous_roi_support', matches=count,
                        inliers=len(inliers), roi_inliers=roi_counts)
    if not _has_3d_thickness(points[inliers], config):
        return _invalid('ambiguous_coplanar_inlier_geometry', matches=count,
                        inliers=len(inliers), roi_inliers=roi_counts)
    try:
        rvec, tvec = cv2.solvePnPRefineLM(
            points[inliers], pixels[inliers], intrinsics.matrix,
            intrinsics.distortion, rvec, tvec)
        rotation, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(
            points[inliers], rvec, tvec, intrinsics.matrix,
            intrinsics.distortion)
    except cv2.error:
        return _invalid('pnp_refinement_failed', matches=count,
                        inliers=len(inliers), roi_inliers=roi_counts)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    projected = projected.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(
        (projected - pixels[inliers]) ** 2, axis=1))))
    transformed = points[inliers] @ rotation.T + translation
    rigid = (
        np.all(np.isfinite(rotation))
        and np.all(np.isfinite(translation))
        and np.allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
        and math.isclose(np.linalg.det(rotation), 1., abs_tol=1e-5)
    )
    if (not rigid or not math.isfinite(rmse)
            or np.any(transformed[:, 2] <= 0.)):
        return _invalid('nonfinite_or_nonforward_pose', matches=count,
                        inliers=len(inliers), rmse=rmse,
                        roi_inliers=roi_counts)
    if rmse > config.maximum_reprojection_rmse_px:
        return _invalid('reprojection_error_too_large', matches=count,
                        inliers=len(inliers), rmse=rmse,
                        roi_inliers=roi_counts)
    if (np.linalg.norm(translation) > config.maximum_translation_m
            or _rotation_angle(rotation) > config.maximum_rotation_rad):
        return _invalid('pose_out_of_bounds', matches=count,
                        inliers=len(inliers), rmse=rmse,
                        roi_inliers=roi_counts)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return ReferencePoseEstimate(
        True, 'observed', transform, inverse, count, len(inliers), rmse,
        roi_counts)


def _detect_orb_features(gray, mask, config):
    orb = cv2.ORB_create(nfeatures=config.orb_features, fastThreshold=7)
    return orb.detectAndCompute(gray, mask)


def _prepare_reference_features(
        reference_gray, reference_depth, reference_rgb_k,
        reference_depth_k, rois, mask, config):
    keypoints, descriptors = _detect_orb_features(
        reference_gray, mask, config)
    if descriptors is None:
        return None, np.empty((0, 3)), ()
    kept_descriptors, object_points, labels = [], [], []
    for index, keypoint in enumerate(keypoints):
        reference_xy = keypoint.pt
        label = next(
            (name for name, (x0, y0, x1, y1) in rois
             if (x0 <= reference_xy[0] < x1
                 and y0 <= reference_xy[1] < y1)),
            None,
        )
        if label is None:
            continue
        point = _depth_point(
            reference_xy, reference_rgb_k, reference_depth_k,
            reference_depth, config)
        if point is None:
            continue
        kept_descriptors.append(descriptors[index])
        object_points.append(point)
        labels.append(label)
    if not kept_descriptors:
        return np.empty((0, descriptors.shape[1]), dtype=descriptors.dtype), \
            np.empty((0, 3), dtype=np.float64), ()
    return (np.asarray(kept_descriptors), np.asarray(object_points),
            tuple(labels))


def _match_prepared_reference(
        reference_descriptors, reference_points, reference_labels,
        current_keypoints, current_descriptors, config):
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    pairs = matcher.knnMatch(reference_descriptors, current_descriptors, k=2)
    candidates = []
    for pair in pairs:
        if (len(pair) != 2
                or pair[0].distance
                >= config.ratio_threshold * pair[1].distance):
            continue
        match = pair[0]
        candidates.append(match)
    candidates.sort(key=lambda item: item.distance)
    unique = []
    used_current = set()
    for match in candidates:
        if match.trainIdx not in used_current:
            unique.append(match)
            used_current.add(match.trainIdx)

    object_points, image_points, labels = [], [], []
    for match in unique:
        object_points.append(reference_points[match.queryIdx])
        image_points.append(current_keypoints[match.trainIdx].pt)
        labels.append(reference_labels[match.queryIdx])
    return object_points, image_points, labels


def _readonly_copy(value):
    copied = np.array(value, copy=True)
    copied.setflags(write=False)
    return copied


class PreparedRGBDReference:
    """Own immutable reference features and reuse them for live observations.

    Construction performs reference ORB extraction and RGB-to-depth lifting
    once.  ``observe`` only processes the current RGB image and performs
    matching/PnP.  This class is pure computation and owns defensive,
    read-only copies, so later caller mutations cannot change the target.
    """

    def __init__(
            self, reference_rgb, reference_depth, reference_rgb_intrinsics,
            reference_depth_intrinsics,
            reference_rois: Mapping[str, Sequence[int]],
            *, config=ReferencePoseConfig()):
        """Copy and prepare one immutable RGB-D destination reference."""
        self.config = config
        self.reference_rgb = _readonly_copy(reference_rgb)
        self.reference_depth = _readonly_copy(reference_depth)
        self.reference_rois = ()
        self.reference_feature_count = 0
        self._validation_error = None
        self._feature_error = None
        self._descriptors = np.empty((0, 32), dtype=np.uint8)
        self._points = np.empty((0, 3), dtype=np.float64)
        self._labels = ()
        try:
            reference_rgb_k = CameraIntrinsics.from_value(
                reference_rgb_intrinsics)
            reference_depth_k = CameraIntrinsics.from_value(
                reference_depth_intrinsics)
            reference_gray = _validate_image(
                self.reference_rgb, reference_rgb_k, 'reference_rgb')
            depth = self.reference_depth
            if (depth.ndim != 2
                    or not np.issubdtype(depth.dtype, np.number)
                    or (reference_depth_k.width is not None
                        and reference_depth_k.width != depth.shape[1])
                    or (reference_depth_k.height is not None
                        and reference_depth_k.height != depth.shape[0])):
                raise ValueError('invalid_reference_depth')
            rois, mask = _validate_rois(
                reference_rois, reference_gray.shape[1],
                reference_gray.shape[0])
            if (reference_rgb_k.frame_id and reference_depth_k.frame_id
                    and (reference_rgb_k.frame_id
                         != reference_depth_k.frame_id)):
                raise ValueError('rgb_depth_optical_frame_mismatch')
            descriptors, points, labels = _prepare_reference_features(
                reference_gray, depth, reference_rgb_k,
                reference_depth_k, rois, mask, config)
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            self._validation_error = str(error)
            return
        self.reference_rois = rois
        if descriptors is None:
            self._feature_error = 'descriptors_unavailable'
            return
        if len(points) == 0:
            self._feature_error = 'insufficient_correspondences'
            return
        self._descriptors = _readonly_copy(descriptors)
        self._points = _readonly_copy(points)
        self._labels = labels
        self.reference_feature_count = len(points)

    def observe(self, current_rgb, current_rgb_intrinsics):
        """Estimate one live camera delta using the prepared reference."""
        if self._validation_error is not None:
            return _invalid(self._validation_error)
        try:
            current_k = CameraIntrinsics.from_value(current_rgb_intrinsics)
            current_gray = _validate_image(
                current_rgb, current_k, 'current_rgb')
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            return _invalid(str(error))
        if self._feature_error is not None:
            return _invalid(self._feature_error)
        current_keypoints, current_descriptors = _detect_orb_features(
            current_gray, None, self.config)
        if current_descriptors is None:
            return _invalid('descriptors_unavailable')
        object_points, image_points, labels = _match_prepared_reference(
            self._descriptors, self._points, self._labels,
            current_keypoints, current_descriptors, self.config)
        return estimate_pose_from_correspondences(
            object_points, image_points, labels, current_k,
            config=self.config)


def estimate_rgbd_reference_pose(
        reference_rgb, reference_depth, reference_rgb_intrinsics,
        reference_depth_intrinsics, current_rgb, current_rgb_intrinsics,
        reference_rois: Mapping[str, Sequence[int]],
        *, config=ReferencePoseConfig()):
    """Prepare once and estimate one local camera delta.

    Repeated callers should retain :class:`PreparedRGBDReference` and call
    ``observe`` instead.  This compatibility API intentionally constructs a
    fresh prepared reference each call and therefore does not provide the
    repeated-call performance benefit.
    """
    prepared = PreparedRGBDReference(
        reference_rgb, reference_depth, reference_rgb_intrinsics,
        reference_depth_intrinsics, reference_rois, config=config)
    return prepared.observe(current_rgb, current_rgb_intrinsics)
