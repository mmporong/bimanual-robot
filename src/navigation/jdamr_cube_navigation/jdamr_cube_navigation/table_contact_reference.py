"""Observe a taught pose from RGB contact bands projected onto its tabletop.

The fixed tabletop supplies depth for image rays. Reflective object depth is
never used. The two taught contact patches retain their physical separation;
their rigid planar registration provides the base correction.
"""

import math
import time

import cv2
import numpy as np

from jdamr_cube_navigation.rgbd_parking_reference import (
    CameraIntrinsics, PlanarBaseErrorEstimate)


def _number(value, label, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (float, int))
            or not math.isfinite(value) or (positive and value <= 0.)):
        raise ValueError(f'invalid_contact_{label}')
    return float(value)


def _subpixel_peak(score, x_px, y_px):
    offsets = []
    for axis in (1, 0):
        coordinate = x_px if axis == 1 else y_px
        if coordinate <= 0 or coordinate >= score.shape[axis] - 1:
            offsets.append(0.)
            continue
        values = (score[y_px, x_px - 1:x_px + 2] if axis == 1
                  else score[y_px - 1:y_px + 2, x_px])
        denominator = float(values[0] - 2 * values[1] + values[2])
        offset = (0.5 * float(values[0] - values[2]) / denominator
                  if np.all(np.isfinite(values)) and denominator < -1e-9 else 0.)
        offsets.append(offset if abs(offset) <= .75 else 0.)
    return x_px + offsets[0], y_px + offsets[1]


class TableContactReference:
    """Metric contact-gradient templates for an unchanged camera and table height."""

    def __init__(self, reference_rgb, rgb_info, basis, config):
        self.basis = basis
        self.intrinsics = CameraIntrinsics.from_value(rgb_info)
        self.shape = reference_rgb.shape
        self.resolution_m = _number(config['resolution_m_per_px'], 'resolution', positive=True)
        self.forward_min_m, self.forward_max_m = (
            _number(v, 'forward_range') for v in config['forward_range_m'])
        self.left_min_m, self.left_max_m = (
            _number(v, 'left_range') for v in config['left_range_m'])
        self.search_back_m, self.search_ahead_m = (
            _number(v, 'search_forward') for v in config['search_forward_offset_m'])
        self.search_lateral_m = _number(
            config['search_lateral_radius_m'], 'search_lateral', positive=True)
        self.minimum_score = _number(config['minimum_match_score'], 'minimum_score', positive=True)
        self.maximum_baseline_error_m = _number(
            config['maximum_baseline_error_m'], 'baseline_error', positive=True)
        self.maximum_yaw_rad = _number(config['maximum_yaw_rad'], 'yaw', positive=True)
        self.candidate_spacing_m = _number(
            config['candidate_spacing_m'], 'candidate_spacing', positive=True)
        self.tracking_radius_m = _number(
            config['tracking_radius_m'], 'tracking_radius', positive=True)
        self._last_centers_px = None
        self.candidate_count = config['candidate_count']
        if (isinstance(self.candidate_count, bool) or not isinstance(self.candidate_count, int)
                or not 1 <= self.candidate_count <= 16):
            raise ValueError('invalid_contact_candidate_count')
        if (self.forward_min_m >= self.forward_max_m or self.left_min_m >= self.left_max_m
                or not self.search_back_m <= 0. < self.search_ahead_m
                or self.minimum_score > 1. or self.maximum_yaw_rad > math.pi):
            raise ValueError('invalid_contact_search_geometry')
        regions = config['reference_regions_rgb']
        if len(regions) != 2:
            raise ValueError('two_named_contact_regions_required')
        left_m, forward_m = np.meshgrid(
            np.arange(self.left_max_m, self.left_min_m, -self.resolution_m),
            np.arange(self.forward_max_m, self.forward_min_m, -self.resolution_m))
        if not 16 <= left_m.size <= 2_000_000:
            raise ValueError('invalid_contact_raster_size')
        lever = basis.camera_lever_xy_m
        points_base = np.stack((forward_m - lever[0], left_m - lever[1],
                                np.full_like(left_m, -basis.plane_offset_m)), axis=-1)
        optical = points_base @ basis.R_base_from_optical
        pixels = optical @ self.intrinsics.matrix.T
        with np.errstate(divide='ignore', invalid='ignore'):
            pixels = pixels[:, :, :2] / pixels[:, :, 2:]
        self.map_x_px = pixels[:, :, 0].astype(np.float32)
        self.map_y_px = pixels[:, :, 1].astype(np.float32)
        height, width = self.shape[:2]
        self.valid_raster = ((optical[:, :, 2] > 0.)
                             & (self.map_x_px >= 2) & (self.map_x_px < width - 2)
                             & (self.map_y_px >= 2) & (self.map_y_px < height - 2)).astype(np.float32)
        self.map_x_px[~np.isfinite(self.map_x_px)] = -1.
        self.map_y_px[~np.isfinite(self.map_y_px)] = -1.
        gradients = self._gradients(reference_rgb)
        angles_deg = [_number(v, 'template_rotation') for v in config['rotation_candidates_deg']]
        if not angles_deg or len(angles_deg) > 61 or any(abs(v) > 45. for v in angles_deg):
            raise ValueError('invalid_contact_template_rotations')
        self.templates = {}
        for name, region in regions.items():
            if len(region) != 4:
                raise ValueError('invalid_contact_region')
            x0, y0, x1, y1 = [_number(v, 'region') for v in region]
            if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
                raise ValueError('contact_region_outside_image')
            mask = ((self.map_x_px >= x0) & (self.map_x_px < x1)
                    & (self.map_y_px >= y0) & (self.map_y_px < y1)
                    & (self.valid_raster > 0.)).astype(np.float32)
            rows_px, columns_px = np.nonzero(mask)
            if len(rows_px) < 16:
                raise ValueError('insufficient_contact_region_support')
            width_px = int(np.ptp(columns_px)) + 1
            height_px = int(np.ptp(rows_px)) + 1
            side_px = int(math.ceil(math.hypot(width_px, height_px))) + 4
            side_px += side_px % 2
            x_px = round((columns_px.min() + columns_px.max() - side_px) / 2)
            y_px = round((rows_px.min() + rows_px.max() - side_px) / 2)
            if not (0 <= x_px < x_px + side_px <= mask.shape[1]
                    and 0 <= y_px < y_px + side_px <= mask.shape[0]):
                raise ValueError('contact_template_outside_raster')
            patch = gradients[y_px:y_px + side_px, x_px:x_px + side_px]
            patch_mask = mask[y_px:y_px + side_px, x_px:x_px + side_px]
            if np.sum(patch ** 2 * patch_mask[:, :, None]) <= 1e-6:
                raise ValueError('contact_template_without_gradient')
            bounds = self._search_bounds(x_px, y_px, side_px)
            sx, sy, ex, ey = bounds
            rotations = []
            for angle_deg in angles_deg:
                matrix = cv2.getRotationMatrix2D((side_px / 2, side_px / 2), angle_deg, 1.)
                rotated = cv2.warpAffine(patch, matrix, (side_px, side_px))
                rotated = (rotated @ matrix[:, :2].T).astype(np.float32)
                support = cv2.warpAffine(
                    patch_mask, matrix, (side_px, side_px), flags=cv2.INTER_NEAREST)
                rotated *= support[:, :, None]
                template_energy = float(np.sum(rotated ** 2))
                coverage = cv2.matchTemplate(
                    self.valid_raster[sy:ey, sx:ex], support, cv2.TM_CCORR) / max(
                        float(support.sum()), 1.)
                rotations.append((rotated, support, template_energy, coverage >= .98))
            self.templates[name] = {
                'bounds_px': bounds, 'side_px': side_px,
                'reference_xy_m': self._base_xy(x_px + side_px / 2, y_px + side_px / 2),
                'rotations': rotations}
        names = tuple(self.templates)
        self.reference_vector_m = (self.templates[names[1]]['reference_xy_m']
                                   - self.templates[names[0]]['reference_xy_m'])
        self.reference_baseline_m = float(np.linalg.norm(self.reference_vector_m))
        if self.reference_baseline_m <= 2 * self.maximum_baseline_error_m:
            raise ValueError('contact_reference_baseline_too_short')

    def _base_xy(self, x_px, y_px):
        return np.array((self.forward_max_m - y_px * self.resolution_m,
                         self.left_max_m - x_px * self.resolution_m))

    def _search_bounds(self, x_px, y_px, side_px):
        return (max(0, x_px - math.ceil(self.search_lateral_m / self.resolution_m)),
                max(0, y_px - math.ceil(self.search_ahead_m / self.resolution_m)),
                min(self.valid_raster.shape[1], x_px + side_px
                    + math.ceil(self.search_lateral_m / self.resolution_m)),
                min(self.valid_raster.shape[0], y_px + side_px
                    - math.floor(self.search_back_m / self.resolution_m)))

    def _gradients(self, image):
        if image.shape != self.shape or image.dtype != np.uint8:
            raise ValueError('contact_image_shape_or_type_changed')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), .6)
        planar = cv2.remap(gray, self.map_x_px, self.map_y_px, cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT)
        return np.stack((cv2.Sobel(planar, cv2.CV_32F, 1, 0, ksize=3),
                         cv2.Sobel(planar, cv2.CV_32F, 0, 1, ksize=3)), axis=-1)

    def observe(self, image):
        started_s = time.perf_counter()
        quality = {'observation_method': 'table_contact_edges', 'contact_matches': {},
                   'contact_tracking': self._last_centers_px is not None}

        def invalid(reason):
            if self._last_centers_px is not None:
                self._last_centers_px = None
                estimate, retry_quality = self.observe(image)
                retry_quality['contact_reacquisition'] = True
                retry_quality['processing_s'] = time.perf_counter() - started_s
                return estimate, retry_quality
            quality['processing_s'] = time.perf_counter() - started_s
            return PlanarBaseErrorEstimate(False, reason), quality

        gradients = self._gradients(image)
        gradient_energy = np.sum(gradients ** 2, axis=2)
        candidates = []
        for name, template in self.templates.items():
            side_px = template['side_px']
            sx, sy, ex, ey = template['bounds_px']
            original_sx, original_sy = sx, sy
            if self._last_centers_px is not None:
                center_x, center_y = self._last_centers_px[name]
                radius_px = math.ceil(self.tracking_radius_m / self.resolution_m)
                sx = max(sx, math.floor(center_x - side_px / 2) - radius_px)
                sy = max(sy, math.floor(center_y - side_px / 2) - radius_px)
                ex = min(ex, math.ceil(center_x + side_px / 2) + radius_px)
                ey = min(ey, math.ceil(center_y + side_px / 2) + radius_px)
            search = gradients[sy:ey, sx:ex]
            search_energy = gradient_energy[sy:ey, sx:ex]
            aggregate = None
            for patch, support, template_energy, covered in template['rotations']:
                covered = covered[sy - original_sy:ey - original_sy - side_px + 1,
                                  sx - original_sx:ex - original_sx - side_px + 1]
                numerator = cv2.matchTemplate(search, patch, cv2.TM_CCORR)
                energy = cv2.matchTemplate(search_energy, support, cv2.TM_CCORR)
                denominator = np.sqrt(np.maximum(energy, 0.) * template_energy)
                scores = np.full_like(numerator, -1.)
                np.divide(numerator, denominator, out=scores,
                          where=covered & (denominator > 1e-6))
                scores[~np.isfinite(scores)] = -1.
                aggregate = scores if aggregate is None else np.maximum(aggregate, scores)
            remaining = aggregate.copy()
            peaks = []
            radius_px = math.ceil(self.candidate_spacing_m / self.resolution_m)
            for _ in range(self.candidate_count):
                _minimum, score, _minimum_at, (px, py) = cv2.minMaxLoc(remaining)
                if score < self.minimum_score:
                    break
                sub_x, sub_y = _subpixel_peak(aggregate, px, py)
                point_m = self._base_xy(sx + sub_x + side_px / 2, sy + sub_y + side_px / 2)
                peaks.append((score, point_m))
                cv2.circle(remaining, (px, py), radius_px, -1., -1)
            quality['contact_matches'][name] = {
                'score': peaks[0][0] if peaks else float(aggregate.max()),
                'candidate_count': len(peaks)}
            if not peaks:
                return invalid('table_contact_match_unavailable')
            candidates.append(peaks)
        best_pair = None
        for first in candidates[0]:
            for second in candidates[1]:
                distance_m = float(np.linalg.norm(second[1] - first[1]))
                if abs(distance_m - self.reference_baseline_m) > self.maximum_baseline_error_m:
                    continue
                score = (first[0] + second[0]) / 2.
                if best_pair is None or score > best_pair[0]:
                    best_pair = score, first, second
        if best_pair is None:
            return invalid('table_contact_pair_not_rigid')
        quality['contact_pair_score'] = best_pair[0]
        current = []
        for (name, match), (score, point_m) in zip(
                quality['contact_matches'].items(), best_pair[1:]):
            match.update(score=score, base_xy_m=point_m.tolist())
            current.append(point_m)
        vector_m = current[1] - current[0]
        baseline_error_m = float(np.linalg.norm(vector_m)) - self.reference_baseline_m
        quality['contact_baseline_error_m'] = baseline_error_m
        if abs(baseline_error_m) > self.maximum_baseline_error_m:
            return invalid('table_contact_pair_not_rigid')
        yaw_rad = math.atan2(vector_m[1], vector_m[0]) - math.atan2(
            self.reference_vector_m[1], self.reference_vector_m[0])
        yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
        if abs(yaw_rad) > self.maximum_yaw_rad:
            return invalid('table_contact_heading_out_of_range')
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        rotation = np.array(((cosine, -sine), (sine, cosine)))
        reference = [t['reference_xy_m'] for t in self.templates.values()]
        translation_m = np.mean([p - rotation @ r for p, r in zip(current, reference)], axis=0)
        self._last_centers_px = {
            name: ((self.left_max_m - point[1]) / self.resolution_m,
                   (self.forward_max_m - point[0]) / self.resolution_m)
            for name, point in zip(self.templates, current)}
        quality['processing_s'] = time.perf_counter() - started_s
        return PlanarBaseErrorEstimate(
            True, 'observed_table_contact_edges', forward_m=float(translation_m[0]),
            left_m=float(translation_m[1]), yaw_rad=yaw_rad,
            basis_source=self.basis.basis_source, camera_yaw_rad=self.basis.camera_yaw_rad,
            camera_lever_xy_m=self.basis.camera_lever_xy_m), quality
