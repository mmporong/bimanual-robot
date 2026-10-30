from types import SimpleNamespace

import numpy as np
import pytest

from jdamr_cube_navigation.table_contact_reference import TableContactReference


def _tracked_reference():
    reference = TableContactReference.__new__(TableContactReference)
    reference.resolution_m = 1.0
    reference.forward_max_m = 20.0
    reference.left_max_m = 20.0
    reference.minimum_score = 0.5
    reference.candidate_count = 1
    reference._candidate_spacing_px = 1
    reference._tracking_radius_px = 1
    reference._last_centers_px = {'cup': (10.0, 10.0), 'bottle': (10.0, 10.0)}
    reference._last_yaw_rad = 0.0
    reference._reacquisition_pending = False
    reference.basis = SimpleNamespace(
        basis_source='test', camera_yaw_rad=0.0, camera_lever_xy_m=(0.0, 0.0))
    patch = np.zeros((2, 2, 2), dtype=np.float32)
    support = np.ones((2, 2), dtype=np.float32)
    covered = np.ones((19, 19), dtype=bool)
    rotation = (patch, support, 1.0, covered, 0.0)
    reference.templates = {
        name: {
            'bounds_px': (0, 0, 20, 20),
            'side_px': 2,
            'reference_xy_m': np.zeros(2),
            'rotations': (rotation,),
            'tracking_rotations': {0.0: (rotation,)},
        }
        for name in ('cup', 'bottle')
    }
    reference._gradients = lambda _image: np.ones((20, 20, 2), dtype=np.float32)
    return reference


def _pairing_reference():
    reference = _tracked_reference()
    reference.candidate_count = 3
    reference._last_centers_px = None
    reference.reference_baseline_m = 4.0
    reference.maximum_baseline_error_m = 0.25
    reference.maximum_yaw_rad = 0.2
    reference._reference_heading_rad = 0.0
    reference._reference_xy_m = np.array(((0.0, 0.0), (4.0, 0.0)))
    reference.templates['cup']['reference_xy_m'] = reference._reference_xy_m[0]
    reference.templates['bottle']['reference_xy_m'] = reference._reference_xy_m[1]
    return reference


def _observe_peaks(monkeypatch, cup_peaks, bottle_peaks):
    reference = _pairing_reference()
    score_maps = []
    for peaks in (cup_peaks, bottle_peaks):
        scores = np.full((19, 19), -1.0, dtype=np.float32)
        for x_px, y_px, score in peaks:
            scores[y_px, x_px] = score
        score_maps.append(scores)

    import jdamr_cube_navigation.table_contact_reference as contact_module
    match_calls = 0

    def match_template(_search, _template, _method):
        nonlocal match_calls
        object_index, is_energy = divmod(match_calls, 2)
        match_calls += 1
        return np.ones((19, 19), dtype=np.float32) if is_energy else score_maps[object_index]

    monkeypatch.setattr(contact_module.cv2, 'matchTemplate', match_template)
    return reference.observe(np.empty((1, 1, 3), dtype=np.uint8))


def test_tracking_failure_defers_full_reacquisition_to_next_frame(monkeypatch):
    reference = _tracked_reference()
    gradient_calls = 0
    search_shapes = []
    original_gradients = reference._gradients

    def gradients(image):
        nonlocal gradient_calls
        gradient_calls += 1
        return original_gradients(image)

    reference._gradients = gradients

    import jdamr_cube_navigation.table_contact_reference as contact_module
    original_match = contact_module.cv2.matchTemplate

    def match_template(search, template, method):
        search_shapes.append(search.shape[:2])
        return original_match(search, template, method)

    monkeypatch.setattr(contact_module.cv2, 'matchTemplate', match_template)
    image = np.empty((1, 1, 3), dtype=np.uint8)

    first, first_quality = reference.observe(image)

    assert first.valid is False
    assert gradient_calls == 1
    assert search_shapes[0] == (4, 4)
    assert first_quality['contact_tracking'] is True
    assert first_quality['contact_reacquisition_pending'] is True

    second, second_quality = reference.observe(image.copy())

    assert second.valid is False
    assert gradient_calls == 2
    assert search_shapes[2] == (20, 20)
    assert second_quality['contact_tracking'] is False
    assert second_quality['contact_reacquisition'] is True


def test_tracking_checkpoint_restores_deferred_reacquisition_state():
    reference = _tracked_reference()
    checkpoint = reference.tracking_checkpoint()

    reference._last_centers_px = None
    reference._last_yaw_rad = None
    reference._reacquisition_pending = True
    reference.restore_tracking(checkpoint)

    assert reference._last_centers_px == checkpoint[0]
    assert reference._last_yaw_rad == checkpoint[1]
    assert reference._reacquisition_pending is False


def test_pairing_uses_valid_heading_alternative(monkeypatch):
    result, quality = _observe_peaks(
        monkeypatch,
        cup_peaks=((5, 7, 0.90),),
        bottle_peaks=((5, 11, 0.95), (5, 3, 0.85)),
    )

    assert result.valid is True
    assert result.yaw_rad == pytest.approx(0.0)
    assert quality['contact_matches']['bottle']['score'] == pytest.approx(0.85)


def test_pairing_prefers_stronger_weak_match_before_mean(monkeypatch):
    result, quality = _observe_peaks(
        monkeypatch,
        cup_peaks=((5, 7, 0.99), (10, 7, 0.80)),
        bottle_peaks=((5, 3, 0.61), (10, 3, 0.79)),
    )

    assert result.valid is True
    assert quality['contact_pair_score'] == pytest.approx(0.795)
    assert quality['contact_pair_min_score'] == pytest.approx(0.79)
    assert quality['contact_pair_selection'] == 'maximum_minimum_score_then_mean'
    assert quality['contact_matches']['cup']['score'] == pytest.approx(0.80)
    assert quality['contact_matches']['bottle']['score'] == pytest.approx(0.79)


@pytest.mark.parametrize(
    ('cup_peaks', 'bottle_peaks', 'reason'),
    [
        (((5, 7, 0.90),), ((5, 3, 0.49),), 'table_contact_match_unavailable'),
        (((5, 7, 0.90),), ((5, 1, 0.90),), 'table_contact_pair_not_rigid'),
        (((5, 3, 0.90),), ((5, 7, 0.90),), 'table_contact_heading_out_of_range'),
    ],
    ids=('occluded', 'wrong_spacing', 'flipped_pair'),
)
def test_pairing_rejects_bounded_invalid_cases(
        monkeypatch, cup_peaks, bottle_peaks, reason):
    result, _quality = _observe_peaks(monkeypatch, cup_peaks, bottle_peaks)

    assert result.valid is False
    assert result.reason == reason
