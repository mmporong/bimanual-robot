"""Tests for fixed-destination RGB-D reference pose estimation."""

import cv2

import jdamr_cube_navigation.rgbd_parking_reference as reference_module
from jdamr_cube_navigation.rgbd_parking_reference import (
    PreparedRGBDReference,
    ReferencePoseConfig,
    camera_reference_pose_to_planar_base_error,
    estimate_pose_from_correspondences,
    estimate_reference_planar_basis,
    estimate_rgbd_reference_pose,
    project_rgb_pixel_to_depth,
)

import numpy as np

import pytest


K = np.array([
    [520., 0., 320.],
    [0., 515., 240.],
    [0., 0., 1.],
])


def _synthetic_correspondences():
    left = np.array([
        [-.35, -.16, .75], [-.28, -.14, .82], [-.20, -.11, .78],
        [-.34, -.03, .86], [-.26, .02, .80], [-.18, .06, .90],
        [-.30, .13, .77], [-.21, .16, .85],
    ])
    right = left.copy()
    right[:, 0] += .58
    right[:, 2] += np.array([.05, -.02, .04, -.03, .02, -.04, .03, -.01])
    points = np.vstack((left, right))
    rvec = np.array([.025, -.04, .018])
    translation = np.array([.032, -.014, .021])
    pixels, _ = cv2.projectPoints(points, rvec, translation, K, None)
    labels = ('cup',) * len(left) + ('bottle',) * len(right)
    return points, pixels.reshape(-1, 2), labels, rvec, translation


def _tabletop_depth(pitch_rad, height_m=1.0):
    width, height = 320, 240
    intrinsic = np.array([
        [258., 0., 160.],
        [0., 258., 120.],
        [0., 0., 1.],
    ])
    sine, cosine = np.sin(pitch_rad), np.cos(pitch_rad)
    up_optical = np.array((0., -cosine, -sine))
    columns, rows = np.meshgrid(np.arange(width), np.arange(height))
    pixels = np.column_stack((columns.reshape(-1), rows.reshape(-1),
                              np.ones(width * height)))
    rays = np.linalg.solve(intrinsic, pixels.T).T
    scales = -height_m / (rays @ up_optical)
    valid = np.isfinite(scales) & (scales > 0.)
    depth = np.zeros(width * height, dtype=np.uint16)
    depth[valid] = np.round(scales[valid] * 1000.).astype(np.uint16)
    info = {
        'k': intrinsic.reshape(-1), 'width': width, 'height': height,
        'frame': 'camera_color_optical_frame',
    }
    return depth.reshape(height, width), info, up_optical


def test_correspondence_pose_defines_current_from_reference_and_inverse_sign():
    """The PnP output and inverse retain the documented translation sign."""
    points, pixels, labels, rvec, translation = _synthetic_correspondences()

    result = estimate_pose_from_correspondences(
        points, pixels, labels, K,
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=8,
            minimum_inliers_per_roi=3,
        ),
    )

    expected_rotation, _ = cv2.Rodrigues(rvec)
    assert result.valid, result.reason
    assert result.reason == 'observed'
    assert result.T_current_from_reference[:3, :3] == pytest.approx(
        expected_rotation, abs=1e-5)
    assert result.T_current_from_reference[:3, 3] == pytest.approx(
        translation, abs=1e-5)
    assert result.T_reference_from_current[:3, 3] == pytest.approx(
        -expected_rotation.T @ translation, abs=1e-5)
    assert (result.T_current_from_reference @ result.T_reference_from_current
            == pytest.approx(np.eye(4), abs=1e-8))


def test_pose_is_invalid_when_only_one_fixed_reference_roi_has_inliers():
    """A geometrically good match from just one object remains ambiguous."""
    points, pixels, labels, _, _ = _synthetic_correspondences()
    pixels[8:] += np.array([170., -130.])

    result = estimate_pose_from_correspondences(
        points, pixels, labels, K,
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=6,
            minimum_inliers_per_roi=3, minimum_inlier_ratio=.3,
        ),
    )

    assert not result.valid
    assert result.reason == 'ambiguous_roi_support'
    assert dict(result.roi_inliers)['bottle'] < 3


def test_coplanar_two_roi_correspondences_are_rejected_before_pnp():
    """Low reprojection error cannot validate planar EPNP ambiguity."""
    random = np.random.default_rng(91)
    y = random.uniform(-.035, .035, 12)
    x = np.concatenate((random.uniform(-.07, -.025, 6),
                        random.uniform(.025, .07, 6)))
    z = (.95 + random.uniform(-.03, .03) * x
         + random.uniform(-.03, .03) * y)
    points = np.column_stack((x, y, z))
    rvec = random.normal(0., .05, 3)
    translation = random.normal(0., .025, 3)
    review_k = np.array([
        [520., 0., 320.],
        [0., 515., 240.],
        [0., 0., 1.],
    ])
    pixels, _ = cv2.projectPoints(
        points, rvec, translation, review_k, None)
    pixels = pixels.reshape(-1, 2) + random.normal(0., .25, (len(points), 2))
    labels = ('cup',) * 6 + ('bottle',) * 6

    result = estimate_pose_from_correspondences(
        points, pixels, labels, review_k,
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=8,
            minimum_inliers_per_roi=3, minimum_inlier_ratio=.4,
            maximum_reprojection_rmse_px=2.5,
            minimum_object_spread_m=.015,
            minimum_image_spread_px=18.,
        ),
    )

    assert not result.valid
    assert result.reason == 'ambiguous_coplanar_geometry'
    assert result.inlier_count == 0


def test_noisy_near_planar_two_roi_correspondences_are_rejected():
    """Small depth noise does not turn a near-plane into 3-D support."""
    random = np.random.default_rng(91)
    xy = random.uniform((-.35, -.2), (.35, .2), (20, 2))
    points = np.column_stack((
        xy, .85 + random.normal(0., .0015, len(xy))))
    rvec = np.array((.025, -.045, .02))
    translation = np.array((.035, -.012, .055))
    pixels, _ = cv2.projectPoints(points, rvec, translation, K, None)
    pixels = pixels.reshape(-1, 2) + random.normal(0., .25, (len(points), 2))
    labels = ('cup',) * 10 + ('bottle',) * 10

    result = estimate_pose_from_correspondences(
        points, pixels, labels, K,
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=8,
            minimum_inliers_per_roi=3,
        ),
    )

    assert not result.valid
    assert result.reason == 'ambiguous_coplanar_geometry'


def test_nonplanar_outliers_cannot_hide_coplanar_pnp_inliers():
    """The accepted RANSAC inliers must independently retain 3-D thickness."""
    random = np.random.default_rng(37)
    xy = random.uniform((-.3, -.18), (.3, .18), (12, 2))
    plane = np.column_stack((xy, np.full(12, .85)))
    outliers = np.array([
        [-.28, -.15, .62], [-.18, .14, 1.08],
        [.19, -.13, .64], [.29, .16, 1.10],
    ])
    points = np.vstack((plane, outliers))
    rvec = np.array((.03, -.04, .02))
    translation = np.array((.025, -.012, .045))
    pixels, _ = cv2.projectPoints(points, rvec, translation, K, None)
    pixels = pixels.reshape(-1, 2)
    pixels[:12] += random.normal(0., .1, (12, 2))
    pixels[12:] += np.array([
        [130., -90.], [-125., 95.], [110., 105.], [-115., -100.],
    ])
    labels = (('cup',) * 6 + ('bottle',) * 6
              + ('cup',) * 2 + ('bottle',) * 2)

    result = estimate_pose_from_correspondences(
        points, pixels, labels, K,
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=8,
            minimum_inliers_per_roi=3, minimum_inlier_ratio=.5,
        ),
    )

    assert not result.valid
    assert result.reason == 'ambiguous_coplanar_inlier_geometry'
    assert result.inlier_count >= 8


def test_rgb_depth_mapping_uses_intrinsics_rays_instead_of_two_x_indexing():
    """Different principal points disprove simple resolution-ratio mapping."""
    rgb = {'k': [500., 0., 310., 0., 510., 245., 0., 0., 1.],
           'width': 640, 'height': 480, 'frame': 'optical'}
    depth = {'p': [260., 0., 151., 0., 0., 250., 119., 0., 0., 0., 1., 0.],
             'width': 320, 'height': 240, 'frame': 'optical'}

    mapped = project_rgb_pixel_to_depth((410., 296.), rgb, depth)

    assert mapped == pytest.approx((203., 144.))
    assert mapped != pytest.approx((205., 148.))


def test_rgb_depth_mapping_rejects_different_optical_frames():
    """Registered depth must share the RGB optical frame."""
    rgb = {'k': K.reshape(-1), 'frame': 'color_optical'}
    depth = {'k': K.reshape(-1), 'frame': 'depth_optical'}

    with pytest.raises(ValueError, match='rgb_depth_optical_frame_mismatch'):
        project_rgb_pixel_to_depth((320., 240.), rgb, depth)


def test_rgb_depth_mapping_rejects_unrectified_intrinsics():
    """Ray mapping cannot silently ignore nonzero lens distortion."""
    rgb = {'k': K.reshape(-1), 'd': [0.1, 0., 0., 0., 0.],
           'frame': 'optical'}
    depth = {'k': K.reshape(-1), 'd': [0., 0., 0., 0., 0.],
             'frame': 'optical'}

    with pytest.raises(
            ValueError,
            match='nonzero_distortion_requires_rectified_images'):
        project_rgb_pixel_to_depth((320., 240.), rgb, depth)


def test_full_estimator_reports_distortion_before_missing_features():
    """Blank images still expose the invalid unrectified-image contract."""
    reference = np.zeros((240, 320), dtype=np.uint8)
    depth = np.full((120, 160), 800, dtype=np.uint16)
    rgb_k = {'k': [260., 0., 160., 0., 260., 120., 0., 0., 1.],
             'd': [0.05, 0., 0., 0., 0.],
             'width': 320, 'height': 240, 'frame': 'optical'}
    depth_k = {'k': [130., 0., 80., 0., 130., 60., 0., 0., 1.],
               'd': [0., 0., 0., 0., 0.],
               'width': 160, 'height': 120, 'frame': 'optical'}

    result = estimate_rgbd_reference_pose(
        reference, depth, rgb_k, depth_k, reference.copy(), rgb_k,
        {'cup': (20, 35, 140, 205), 'bottle': (180, 30, 305, 210)},
    )

    assert not result.valid
    assert result.reason == 'nonzero_distortion_requires_rectified_images'


def test_full_estimator_rejects_missing_second_reference_group():
    """One textured ROI cannot stand in for the two-object reference."""
    rng = np.random.default_rng(7)
    reference = np.zeros((240, 320), dtype=np.uint8)
    reference[40:200, 25:135] = rng.integers(
        0, 256, (160, 110), dtype=np.uint8)
    current = reference.copy()
    depth = np.full((120, 160), 800, dtype=np.uint16)
    rgb_k = {'k': [260., 0., 160., 0., 260., 120., 0., 0., 1.],
             'width': 320, 'height': 240, 'frame': 'optical'}
    depth_k = {'k': [130., 0., 80., 0., 130., 60., 0., 0., 1.],
               'width': 160, 'height': 120, 'frame': 'optical'}

    result = estimate_rgbd_reference_pose(
        reference, depth, rgb_k, depth_k, current, rgb_k,
        {'cup': (20, 35, 140, 205), 'bottle': (190, 35, 300, 205)},
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=6,
            minimum_inliers_per_roi=2,
        ),
    )

    assert not result.valid
    assert result.reason in {
        'ambiguous_roi_support', 'insufficient_correspondences'}


def test_full_estimator_replays_two_fixed_rois_at_identity():
    """Two visible fixed groups recover an identity reference observation."""
    rng = np.random.default_rng(19)
    reference = np.zeros((240, 320), dtype=np.uint8)
    reference[40:200, 25:135] = rng.integers(
        0, 256, (160, 110), dtype=np.uint8)
    reference[35:205, 185:300] = rng.integers(
        0, 256, (170, 115), dtype=np.uint8)
    depth_rows, depth_columns = np.indices((120, 160))
    depth = (800 + 65 * (
        (depth_columns // 13 + depth_rows // 11) % 3 - 1)).astype(np.uint16)
    rgb_k = {'k': [260., 0., 160., 0., 260., 120., 0., 0., 1.],
             'width': 320, 'height': 240, 'frame': 'optical'}
    depth_k = {'k': [130., 0., 80., 0., 130., 60., 0., 0., 1.],
               'width': 160, 'height': 120, 'frame': 'optical'}

    result = estimate_rgbd_reference_pose(
        reference, depth, rgb_k, depth_k, reference.copy(), rgb_k,
        {'cup': (20, 35, 140, 205), 'bottle': (180, 30, 305, 210)},
        config=ReferencePoseConfig(
            minimum_correspondences=8, minimum_inliers=6,
            minimum_inliers_per_roi=2,
        ),
    )

    assert result.valid, result.reason
    assert dict(result.roi_inliers)['cup'] >= 2
    assert dict(result.roi_inliers)['bottle'] >= 2
    assert result.T_current_from_reference == pytest.approx(
        np.eye(4), abs=1e-5)


def test_prepared_reference_reuses_orb_and_depth_lift(monkeypatch):
    """Repeated observations prepare reference ORB and 3-D points once."""
    calls = {'reference_orb': 0, 'current_orb': 0, 'depth_lift': 0}
    original_detect = reference_module._detect_orb_features
    original_prepare = reference_module._prepare_reference_features

    def counted_detect(gray, mask, config):
        key = 'current_orb' if mask is None else 'reference_orb'
        calls[key] += 1
        return original_detect(gray, mask, config)

    def counted_prepare(*args, **kwargs):
        calls['depth_lift'] += 1
        return original_prepare(*args, **kwargs)

    monkeypatch.setattr(
        reference_module, '_detect_orb_features', counted_detect)
    monkeypatch.setattr(
        reference_module, '_prepare_reference_features', counted_prepare)
    random = np.random.default_rng(101)
    reference = np.zeros((240, 320), dtype=np.uint8)
    reference[40:200, 25:135] = random.integers(
        0, 256, (160, 110), dtype=np.uint8)
    reference[35:205, 185:300] = random.integers(
        0, 256, (170, 115), dtype=np.uint8)
    current = reference.copy()
    depth_rows, depth_columns = np.indices((120, 160))
    depth = (800 + 65 * (
        (depth_columns // 13 + depth_rows // 11) % 3 - 1)).astype(np.uint16)
    rgb_k = {'k': [260., 0., 160., 0., 260., 120., 0., 0., 1.],
             'width': 320, 'height': 240, 'frame': 'optical'}
    depth_k = {'k': [130., 0., 80., 0., 130., 60., 0., 0., 1.],
               'width': 160, 'height': 120, 'frame': 'optical'}
    config = ReferencePoseConfig(
        minimum_correspondences=8, minimum_inliers=6,
        minimum_inliers_per_roi=2)

    prepared = PreparedRGBDReference(
        reference, depth, rgb_k, depth_k,
        {'cup': (20, 35, 140, 205), 'bottle': (180, 30, 305, 210)},
        config=config)
    assert calls == {
        'reference_orb': 1, 'current_orb': 0, 'depth_lift': 1}
    assert not np.shares_memory(prepared.reference_rgb, reference)
    assert not np.shares_memory(prepared.reference_depth, depth)
    reference[:] = 0
    depth[:] = 0

    first = prepared.observe(current, rgb_k)
    second = prepared.observe(current, rgb_k)

    assert first.valid, first.reason
    assert second.valid, second.reason
    assert first.T_current_from_reference == pytest.approx(
        second.T_current_from_reference, abs=1e-10)
    assert calls == {
        'reference_orb': 1, 'current_orb': 2, 'depth_lift': 1}
    with pytest.raises(ValueError):
        prepared.reference_rgb[0, 0] = 1
    with pytest.raises(ValueError):
        prepared.reference_depth[0, 0] = 1


def test_full_estimator_rejects_invalid_reference_depth():
    """Zero depth neighborhoods cannot create guessed 3-D feature points."""
    rng = np.random.default_rng(23)
    reference = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    depth = np.zeros((120, 160), dtype=np.uint16)
    rgb_k = {'k': [260., 0., 160., 0., 260., 120., 0., 0., 1.],
             'width': 320, 'height': 240, 'frame': 'optical'}
    depth_k = {'k': [130., 0., 80., 0., 130., 60., 0., 0., 1.],
               'width': 160, 'height': 120, 'frame': 'optical'}

    result = estimate_rgbd_reference_pose(
        reference, depth, rgb_k, depth_k, reference.copy(), rgb_k,
        {'cup': (20, 35, 140, 205), 'bottle': (180, 30, 305, 210)},
    )

    assert not result.valid
    assert result.reason == 'insufficient_correspondences'


def test_tabletop_plane_builds_candidate_basis_with_explicit_yaw():
    """The plane sign and projected optical forward axis form base axes."""
    pitch = np.deg2rad(38.)
    camera_yaw = np.deg2rad(7.)
    depth, info, expected_up = _tabletop_depth(pitch)

    result = estimate_reference_planar_basis(
        depth, info, (0, 0, 320, 167),
        camera_yaw_rad=camera_yaw,
        camera_lever_xy_m=(-.275, 0.),
        exclusion_depth_rois=((70, 50, 123, 118),
                              (222, 25, 265, 118)),
    )

    level_rotation = np.array([
        [0., -np.sin(pitch), np.cos(pitch)],
        [-1., 0., 0.],
        [0., -np.cos(pitch), -np.sin(pitch)],
    ])
    yaw_rotation = np.array([
        [np.cos(camera_yaw), -np.sin(camera_yaw), 0.],
        [np.sin(camera_yaw), np.cos(camera_yaw), 0.],
        [0., 0., 1.],
    ])
    assert result.valid, result.reason
    assert result.plane_normal_up_optical == pytest.approx(
        expected_up, abs=2e-4)
    assert result.R_base_from_optical == pytest.approx(
        yaw_rotation @ level_rotation, abs=2e-4)
    assert result.plane_offset_m == pytest.approx(1., abs=2e-4)
    assert result.camera_yaw_rad == pytest.approx(camera_yaw)
    assert result.camera_lever_xy_m == pytest.approx((-.275, 0.))
    assert not result.mount_calibration_complete
    assert not result.millimeter_accuracy_validated


def test_tabletop_basis_requires_external_camera_yaw():
    """A plane cannot supply the camera yaw assumption by itself."""
    depth, info, _ = _tabletop_depth(np.deg2rad(40.))

    with pytest.raises(TypeError, match='camera_yaw_rad'):
        estimate_reference_planar_basis(
            depth, info, (0, 0, 320, 167),
            camera_lever_xy_m=(-.275, 0.))


def test_camera_pose_converts_to_base_error_with_lever_arm_sign():
    """The camera correction yields the intended base delta and yaw sign."""
    pitch = np.deg2rad(40.)
    depth, info, _ = _tabletop_depth(pitch)
    basis = estimate_reference_planar_basis(
        depth, info, (0, 0, 320, 167), camera_yaw_rad=np.deg2rad(6.),
        camera_lever_xy_m=(-.275, .03))
    expected_delta = np.array((-.042, .026, 0.))
    expected_yaw = np.deg2rad(5.)
    cosine, sine = np.cos(expected_yaw), np.sin(expected_yaw)
    yaw_rotation = np.array([
        [cosine, -sine, 0.],
        [sine, cosine, 0.],
        [0., 0., 1.],
    ])
    lever = np.array((-.275, .03, 0.))
    translation_base = expected_delta - lever + yaw_rotation @ lever
    base_from_optical = basis.R_base_from_optical
    transform = np.eye(4)
    transform[:3, :3] = (
        base_from_optical.T @ yaw_rotation @ base_from_optical)
    transform[:3, 3] = base_from_optical.T @ translation_base

    result = camera_reference_pose_to_planar_base_error(transform, basis)

    assert result.valid, result.reason
    assert (result.forward_m, result.left_m) == pytest.approx(
        expected_delta[:2], abs=1e-8)
    assert result.yaw_rad == pytest.approx(expected_yaw, abs=1e-8)
    assert result.pitch_roll_residual_rad == pytest.approx(0., abs=1e-8)
    assert not result.mount_calibration_complete
    assert not result.millimeter_accuracy_validated


def test_camera_pose_rejects_large_pitch_roll_residual():
    """A nonplanar camera pose is invalid rather than flattened to yaw."""
    depth, info, _ = _tabletop_depth(np.deg2rad(40.))
    basis = estimate_reference_planar_basis(
        depth, info, (0, 0, 320, 167), camera_yaw_rad=0.,
        camera_lever_xy_m=(-.275, 0.))
    angle = np.deg2rad(9.)
    roll = np.array([
        [1., 0., 0.],
        [0., np.cos(angle), -np.sin(angle)],
        [0., np.sin(angle), np.cos(angle)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = (
        basis.R_base_from_optical.T @ roll @ basis.R_base_from_optical)

    result = camera_reference_pose_to_planar_base_error(transform, basis)

    assert not result.valid
    assert result.reason == 'camera_pose_has_excess_pitch_roll'
    assert result.pitch_roll_residual_rad == pytest.approx(angle)


def test_tabletop_plane_rejects_missing_depth_inliers():
    """An empty tabletop ROI cannot become a candidate mount basis."""
    depth = np.zeros((240, 320), dtype=np.uint16)
    info = {'k': K.reshape(-1), 'width': 320, 'height': 240}

    result = estimate_reference_planar_basis(
        depth, info, (0, 0, 320, 167), camera_yaw_rad=0.,
        camera_lever_xy_m=(-.275, 0.))

    assert not result.valid
    assert result.reason == 'insufficient_tabletop_points'


@pytest.mark.parametrize('matrix', [
    np.zeros((3, 3)),
    np.array([[520., 0., 320.], [0., np.nan, 240.], [0., 0., 1.]]),
])
def test_invalid_intrinsics_return_an_explicit_invalid_reason(matrix):
    """Malformed calibration never reaches OpenCV as a guessed camera."""
    points, pixels, labels, _, _ = _synthetic_correspondences()

    result = estimate_pose_from_correspondences(points, pixels, labels, matrix)

    assert not result.valid
    assert result.reason == 'invalid_intrinsics'
