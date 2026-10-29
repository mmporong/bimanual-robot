"""Tests for stationary offline occupancy-map observation fusion."""

import math

import numpy as np

from jdamr_cube_navigation.map_observation_update import (
    update_map_from_stationary_capture,
)


SHAPE = (11, 11)
METADATA = {'resolution': 1.0, 'origin': [0.0, 0.0, 0.0]}
IDENTITY_TF = [{
    'parent': 'base_link',
    'child': 'laser',
    't': [0.0, 0.0, 0.0],
    'q': [0.0, 0.0, 0.0, 1.0],
}]


def _cell(x_value, y_value, *, shape=SHAPE):
    return shape[0] - 1 - y_value, x_value


def _scan(ranges, *, angle_min=0.0, angle_increment=0.0,
          range_min=0.1, range_max=20.0):
    return {
        'frame': 'laser',
        'angle_min': angle_min,
        'angle_increment': angle_increment,
        'range_min': range_min,
        'range_max': range_max,
        'ranges': ranges,
    }


def _update(image, scans, *, roi=None, pose=(0.5, 5.5, 0.0),
            metadata=METADATA, tf_static=IDENTITY_TF, **kwargs):
    if roi is None:
        roi = np.ones(image.shape, dtype=bool)
    return update_map_from_stationary_capture(
        image,
        metadata,
        {'scans': scans, 'tf_static': tf_static},
        pose,
        roi,
        **kwargs,
    )


def test_moved_obstacle_clears_old_cell_and_marks_new_hit():
    image = np.full(SHAPE, 205, dtype=np.uint8)
    image[_cell(3, 5)] = 0

    result = _update(image, [_scan([6.0])] * 3)

    assert result[_cell(3, 5)] == 254
    assert result[_cell(6, 5)] == 0
    assert result.shape == image.shape
    assert result.dtype == image.dtype


def test_cells_outside_roi_and_unobserved_cells_remain_byte_identical():
    image = np.arange(121, dtype=np.uint8).reshape(SHAPE)
    roi = np.zeros(SHAPE, dtype=bool)
    row = _cell(1, 5)[0]
    roi[row:row + 1, 1:7] = True
    roi[_cell(3, 5)] = False

    result = _update(image, [_scan([6.0])] * 3, roi=roi)

    changed = result != image
    assert not changed[_cell(3, 5)]
    assert not changed[_cell(9, 9)]
    np.testing.assert_array_equal(result[~roi], image[~roi])


def test_invalid_and_no_return_ranges_do_not_clear_cells():
    image = np.full(SHAPE, 17, dtype=np.uint8)
    scans = [
        _scan(None),
        _scan([None, math.inf, math.nan, 0.05, 21.0, 5.0]),
        _scan([5.0]),
        _scan([5.0]),
    ]

    result = _update(
        image, scans, max_range_m=4.0, min_observations=1)

    np.testing.assert_array_equal(result, image)


def test_laser_extrinsic_and_base_yaw_are_applied_to_hit():
    image = np.full(SHAPE, 205, dtype=np.uint8)
    tf_static = [{
        'parent': 'base_link',
        'child': 'laser',
        't': [1.0, 0.0, 0.0],
        'q': [0.0, 0.0, 0.0, 1.0],
    }]

    result = _update(
        image,
        [_scan([2.0])] * 3,
        pose=(5.5, 5.5, math.pi / 2.0),
        tf_static=tf_static,
    )

    assert result[_cell(5, 8)] == 0
    assert result[_cell(8, 5)] == 205


def test_endpoint_outside_map_clears_to_boundary_without_false_hit():
    image = np.zeros(SHAPE, dtype=np.uint8)

    result = _update(image, [_scan([5.0])] * 3, pose=(8.5, 5.5, 0.0))

    assert result[_cell(10, 5)] == 254
    assert not np.any(result[_cell(8, 5)[0], 8:] == 0)


def test_hit_wins_over_free_traversal_within_each_frame():
    image = np.full(SHAPE, 205, dtype=np.uint8)
    scans = [_scan([2.0, 4.0])] * 3

    result = _update(image, scans)

    assert result[_cell(2, 5)] == 0
    assert result[_cell(4, 5)] == 0


def test_low_evidence_preserves_original_bytes():
    image = np.arange(121, dtype=np.uint8).reshape(SHAPE)

    result = _update(image, [_scan([6.0])] * 2)

    np.testing.assert_array_equal(result, image)


def test_rotated_map_origin_is_supported():
    image = np.full((5, 5), 205, dtype=np.uint8)
    metadata = {
        'resolution': 1.0,
        'origin': [10.0, 20.0, math.pi / 2.0],
        'width': 5,
        'height': 5,
    }

    result = _update(
        image,
        [_scan([2.0])] * 3,
        pose=(9.5, 20.5, math.pi / 2.0),
        metadata=metadata,
    )

    assert result[_cell(2, 0, shape=(5, 5))] == 0
