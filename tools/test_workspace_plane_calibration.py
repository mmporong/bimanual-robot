import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("workspace_plane_calibration.py")
SPEC = importlib.util.spec_from_file_location("workspace_plane_calibration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_build_calibration_reconstructs_correspondence_points():
    image = [[40, 30], [600, 50], [580, 440], [55, 450]]
    base = [[0.25, 0.25], [0.25, -0.25], [0.65, -0.25], [0.65, 0.25]]
    result = MODULE.build_calibration(image, base, (640, 480), 0.72, 0.105, 0.55, 0.06)
    assert result["frame"] == "base_footprint"
    assert result["fit_error_mm"]["max"] < 3.0
    assert result["surface_z_m"] == 0.72


def test_parse_xy_rejects_wrong_shape():
    assert MODULE.parse_xy("0.31,-0.18") == [0.31, -0.18]
    try:
        MODULE.parse_xy("0.31")
    except Exception as exc:
        assert "X,Y" in str(exc)
    else:
        raise AssertionError("잘못된 좌표가 허용됨")
