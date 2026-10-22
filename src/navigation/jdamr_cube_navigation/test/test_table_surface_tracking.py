"""Motion, ambiguity, cropping, calibration and time regressions."""

from dataclasses import replace
import math
from types import SimpleNamespace

from jdamr_cube_navigation.table_surface_tracking import (
    rigid_transform, TableSurfaceTracker, TargetRegion, TrackConfig,
)
import numpy as np
import pytest


def observation(world=None, camera=None):
    if world is None:
        world = np.array([[-.4, 1., .75], [.4, 1., .75]])
    if camera is None:
        camera = np.eye(4)
    optical = (world - camera[:3, 3]) @ camera[:3, :3]
    return SimpleNamespace(edge_endpoints_optical_m=optical)


def test_motion_compensation_tracks_the_same_world_line():
    tracker = TableSurfaceTracker(TrackConfig(required_frames=2))
    first = tracker.update([observation()], 1., np.eye(4), 'mount-a')
    camera = np.eye(4)
    angle = .15
    camera[:2, :2] = [[math.cos(angle), -math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]]
    camera[:2, 3] = [.1, .2]
    second = tracker.update([observation(camera=camera)], 1.1, camera, 'mount-a')
    assert first.track_id == second.track_id
    assert second.state == 'observed'
    assert second.stable_frames == 2
    np.testing.assert_allclose(second.endpoints_odom_m, first.endpoints_odom_m)


def test_cropping_and_reversed_endpoints_do_not_change_the_target():
    tracker = TableSurfaceTracker()
    first = tracker.update([observation()], 1., np.eye(4), 'a')
    crop = np.array([[.3, 1., .75], [-.1, 1., .75]])
    second = tracker.update([observation(crop)], 1.1, np.eye(4), 'a')
    assert second.track_id == first.track_id
    assert second.stable_frames == 2


def test_multiple_initial_candidates_require_a_target_selection():
    tracker = TableSurfaceTracker()
    result = tracker.update([observation(), observation()], 1., np.eye(4), 'a')
    assert result.state == 'ambiguous'
    assert result.track_id is None and result.detection is None


def test_registered_region_selects_one_of_several_initial_objects():
    tracker = TableSurfaceTracker()
    selected = observation()
    other = observation(np.array([[.5, 1., .75], [1.2, 1., .75]]))
    region = TargetRegion((0., 1.), .1)
    result = tracker.update([other, selected], 1., np.eye(4), 'a', region)
    assert result.state == 'observed' and result.detection is selected


def test_region_containing_two_candidates_is_still_ambiguous():
    tracker = TableSurfaceTracker()
    result = tracker.update([observation(), observation()], 1., np.eye(4), 'a',
                            TargetRegion((0., 1.), .1))
    assert result.state == 'ambiguous' and result.detection is None


def test_switching_the_selected_table_region_starts_a_new_identity():
    tracker = TableSurfaceTracker()
    first = tracker.update([observation()], 1., np.eye(4), 'a', TargetRegion((0., 1.), .1))
    other = observation(np.array([[.5, 1., .75], [1.2, 1., .75]]))
    second = tracker.update([other], 1.1, np.eye(4), 'a', TargetRegion((.8, 1.), .1))
    assert second.state == 'observed' and second.track_id != first.track_id
    assert second.stable_frames == 1


def test_a_different_table_does_not_silently_replace_the_selected_face():
    tracker = TableSurfaceTracker()
    first = tracker.update([observation()], 1., np.eye(4), 'a')
    other = np.array([[-.4, 1.4, .75], [.4, 1.4, .75]])
    result = tracker.update([observation(other)], 1.1, np.eye(4), 'a')
    assert result.state == 'target_not_observed'
    assert result.track_id == first.track_id and result.detection is None


def test_a_nearby_coplanar_but_nonoverlapping_table_is_not_the_same_target():
    tracker = TableSurfaceTracker()
    first = tracker.update([observation()], 1., np.eye(4), 'a')
    adjacent = np.array([[.45, 1., .75], [.8, 1., .75]])
    result = tracker.update([observation(adjacent)], 1.1, np.eye(4), 'a')
    assert result.state == 'target_not_observed'
    assert result.track_id == first.track_id
    assert result.detection is None and result.stable_frames == 0


def test_two_matching_segments_are_ambiguous_instead_of_arbitrarily_picked():
    tracker = TableSurfaceTracker()
    tracker.update([observation()], 1., np.eye(4), 'a')
    result = tracker.update([observation(), observation()], 1.1, np.eye(4), 'a')
    assert result.state == 'ambiguous' and result.detection is None


def test_prediction_keeps_measurement_age_and_is_not_a_control_observation():
    tracker = TableSurfaceTracker()
    tracker.update([observation()], 1., np.eye(4), 'a')
    result = tracker.update([], 1.1, np.eye(4), 'a')
    assert result.state == 'predicted'
    assert result.measurement_stamp_s == 1. and result.frame_stamp_s == 1.1
    assert result.detection is None and result.stable_frames == 0
    assert tracker.update([], 1.3, np.eye(4), 'a').state == 'expired'


@pytest.mark.parametrize('change', ['translation', 'rotation'])
def test_prediction_expires_when_motion_exceeds_the_verified_budget(change):
    tracker = TableSurfaceTracker()
    tracker.update([observation()], 1., np.eye(4), 'a')
    camera = np.eye(4)
    if change == 'translation':
        camera[0, 3] = .05
    else:
        camera[:2, :2] = [[0., -1.], [1., 0.]]
    assert tracker.update([], 1.1, camera, 'a').state == 'expired'


def test_calibration_change_starts_a_new_identity():
    tracker = TableSurfaceTracker()
    first = tracker.update([observation()], 1., np.eye(4), 'a')
    second = tracker.update([observation()], 1.1, np.eye(4), 'b')
    assert first.track_id != second.track_id and second.stable_frames == 1


def test_out_of_order_frame_does_not_modify_or_freshen_the_track():
    tracker = TableSurfaceTracker()
    tracker.update([observation()], 2., np.eye(4), 'a')
    stale = tracker.update([observation()], 1., np.eye(4), 'a')
    assert stale.state == 'nonmonotonic_input' and stale.detection is None
    assert stale.measurement_stamp_s == 2.
    fresh = tracker.update([observation()], 2.1, np.eye(4), 'a')
    assert fresh.stable_frames == 2


@pytest.mark.parametrize('matrix', [np.zeros((4, 4)), np.diag([2., 1., 1., 1.]),
                                    np.diag([-1., 1., 1., 1.]), np.eye(4) * np.nan])
def test_nonrigid_transforms_are_rejected(matrix):
    with pytest.raises(ValueError):
        rigid_transform(matrix)


def test_invalid_tracker_configuration_is_rejected():
    with pytest.raises(ValueError):
        TableSurfaceTracker(replace(TrackConfig(), required_frames=2.5))
