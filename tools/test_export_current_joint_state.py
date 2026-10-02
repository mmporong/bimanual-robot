import importlib.util
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).parent / "servo" / "export_current_joint_state.py"
SPEC = importlib.util.spec_from_file_location("export_current_joint_state", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_midpoint_maps_to_zero_degrees():
    assert MODULE.raw_to_lerobot_degrees(2000, 1000, 3000) == 0.0


def test_conversion_matches_lerobot_degrees_contract():
    value = MODULE.raw_to_lerobot_degrees(3000, 1000, 3000)
    assert abs(value - (1000 * 360 / 4095)) < 1e-12


def test_invalid_calibration_range_is_rejected():
    try:
        MODULE.raw_to_lerobot_degrees(1000, 1000, 1000)
    except ValueError as exc:
        assert "range_min" in str(exc)
    else:
        raise AssertionError("0 폭 calibration이 허용됨")
