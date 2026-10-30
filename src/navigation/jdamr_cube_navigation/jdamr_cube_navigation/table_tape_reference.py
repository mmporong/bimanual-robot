"""Locate two dark strips whose upper ends meet the tabletop front edge."""

import math
import time
from itertools import combinations

import cv2
import numpy as np

from jdamr_cube_navigation.rgbd_parking_reference import CameraIntrinsics, PlanarBaseErrorEstimate


class TableTapeReference:
    """Project strip upper ends onto the calibrated tabletop, not strip centres."""

    def __init__(self, rgb_info, basis, config):
        cv2.setNumThreads(1)
        self.intrinsics = CameraIntrinsics.from_value(rgb_info)
        self.rays_from_pixels = basis.R_base_from_optical @ np.linalg.inv(self.intrinsics.matrix)
        self.basis = basis
        self.config = dict(config)
        for key in ('spacing_m', 'width_m', 'goal_front_from_base_m',
                    'spacing_tolerance_m', 'width_tolerance_m', 'maximum_yaw_rad'):
            value = config[key]
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_tape_' + key)

    def tracking_checkpoint(self):
        return None

    def restore_tracking(self, checkpoint):
        pass  # Stateless detector; rejected observations leave no tracking state.

    def _point(self, u, v):
        ray = self.rays_from_pixels @ np.array([u, v, 1.])
        if ray[2] >= -1e-6:
            return None
        point = ray * (-self.basis.plane_offset_m / ray[2])
        return point[:2] + np.asarray(self.basis.camera_lever_xy_m)

    def observe(self, image):
        started = time.perf_counter()
        quality = {'observation_method': 'table_front_tape', 'candidates': []}

        def invalid(reason):
            quality['processing_s'] = time.perf_counter() - started
            return PlanarBaseErrorEstimate(False, reason), quality

        if image.ndim != 3 or image.dtype != np.uint8:
            return invalid('invalid_tape_image')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Isolate the upper transition: near contact the bottom of a strip can
        # merge with the dark chassis, so full-component height is not stable.
        mask = np.zeros_like(gray, dtype=np.uint8)
        mask[4:] = ((gray[4:] < 65) & (gray[:-4] > 120)).astype(np.uint8)
        candidates = []
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        full_mask = (gray < 65).astype(np.uint8)
        contours = list(contours) + list(cv2.findContours(
            full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
        for contour in contours:
            _, y, component_width, _ = cv2.boundingRect(contour)
            # The upper edge cannot be wider than its bounding box. Reject
            # tiny components before allocating arrays for every image speck.
            if component_width < 4 or y < 3 or y + 2 >= gray.shape[0]:
                continue
            upper = contour[:, 0, :]
            upper = upper[upper[:, 1] <= y + 1]
            x = int(upper[:, 0].min())
            w = int(upper[:, 0].max()) - x + 1
            h = 2
            # Reject clipped components, arm silhouettes and isolated pixels.
            if not (4 <= w <= 90
                    and x >= w and x + 2 * w < gray.shape[1] and y >= 3
                    and y + h < gray.shape[0]
                    and np.mean(full_mask[y:y+3, x:x+w]) >= .60):
                continue
            above = gray[y-3:y, x:x+w]
            sides = np.concatenate((gray[y:y+h, x-w:x].ravel(),
                                    gray[y:y+h, x+w:x+2*w].ravel()))
            if np.median(above) < 100 or np.median(sides) < 95:
                continue
            top = contour[:, 0, :]
            top = top[top[:, 1] <= y + 1]
            u = float((top[:, 0].min() + top[:, 0].max()) / 2)
            point = self._point(u, float(y))
            ends = [self._point(x, y), self._point(x+w, y)]
            if point is None or any(p is None for p in ends):
                continue
            width = float(np.linalg.norm(ends[1] - ends[0]))
            if abs(width - self.config['width_m']) > self.config['width_tolerance_m']:
                continue
            if abs(point[1]) > self.config.get('maximum_lateral_m', .20):
                continue
            if any(np.linalg.norm(point-c[2]) < .005 for c in candidates):
                continue
            candidates.append((u, float(y), point, width))
            quality['candidates'].append({'pixel': [u, y], 'base_xy_m': point.tolist(),
                                          'width_m': width})
        pairs = []
        for a, b in combinations(sorted(candidates, key=lambda c: c[0]), 2):
            vector = a[2] - b[2]  # image-left is base-left
            spacing = float(np.linalg.norm(vector))
            if abs(spacing - self.config['spacing_m']) > self.config['spacing_tolerance_m']:
                continue
            yaw = math.atan2(-vector[0], vector[1])
            if abs(yaw) > self.config['maximum_yaw_rad']:
                continue
            pairs.append((a, b, spacing, yaw))
        if len(pairs) != 1:
            return invalid('tape_pair_missing' if not pairs else 'tape_pair_ambiguous')
        a, b, spacing, yaw = pairs[0]
        center = (a[2] + b[2]) / 2
        goal = center - self.config['goal_front_from_base_m'] * np.array(
            [math.cos(yaw), math.sin(yaw)])
        quality.update(tape_pixels=[[a[0], a[1]], [b[0], b[1]]],
                       tape_spacing_m=spacing, tape_center_base_xy_m=center.tolist(),
                       processing_s=time.perf_counter()-started)
        return PlanarBaseErrorEstimate(
            True, 'observed_table_front_tape', forward_m=float(goal[0]),
            left_m=float(goal[1]), yaw_rad=yaw, basis_source=self.basis.basis_source,
            camera_yaw_rad=self.basis.camera_yaw_rad,
            camera_lever_xy_m=self.basis.camera_lever_xy_m), quality
