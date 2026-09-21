import copy
import json
from itertools import product
from pathlib import Path

import numpy as np
import pytest

from service_camera_views import camera_views


def config():
    return json.loads((Path(__file__).resolve().parents[1] / "config/simulation/service_cameras.json").read_text())


def test_wide_is_fixed_and_detail_looks_from_front_above_cup():
    settings = config()
    before = copy.deepcopy(settings)
    cup = [.4, 0., .8]
    views = camera_views(settings, "POUR", "POUR_HOLD", cup)
    assert views["pour_detail"]["eye_m"][0] > cup[0]
    assert views["pour_detail"]["eye_m"][2] > cup[2] + .12
    assert views["wide"] == camera_views(settings, "NAVIGATE", "NAVIGATE", [-1., -2., 1.])["wide"]
    assert "pour_detail" not in camera_views(settings, "DEPOSIT", "TRAY_LOWER", cup)
    assert settings == before and cup == [.4, 0., .8]


def test_nonfinite_camera_is_rejected():
    settings = config()
    settings["wide"]["eye_m"][0] = float("nan")
    with pytest.raises(ValueError):
        camera_views(settings, "POUR", "POUR_HOLD", [.4, 0., .8])


def test_wide_keeps_robot_envelope_inside_frame_at_kitchen_and_guest():
    view = config()["wide"]
    eye = np.array(view["eye_m"])
    forward = np.array(view["target_m"]) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    # Conservative 16:9 crop of the default USD camera's horizontal aperture.
    half_width_ratio = 20.955 / (2 * view["focal_length_mm"])
    half_height_ratio = half_width_ratio * 9 / 16
    envelope = np.array(list(product([-.62, .62], [-.62, .62], [0., 1.4])))
    for base in ([0., 0., 0.], [-.7, 0., 0.], [-1.75, -2.25, 0.]):
        delta = envelope + base - eye
        depth = delta @ forward
        horizontal = delta @ right / depth / half_width_ratio
        vertical = delta @ up / depth / half_height_ratio
        assert depth.min() > 0
        assert max(np.abs(horizontal).max(), np.abs(vertical).max()) < .9
