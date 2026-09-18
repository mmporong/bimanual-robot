import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


MODULE_PATH = Path(__file__).with_name("plan_body_side_grasp.py")
SPEC = importlib.util.spec_from_file_location("plan_body_side_grasp", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_body_center_is_table_plus_half_height():
    cup = MODULE.ObjectSpec("cup", "left", (0.32, 0.17), 0.12, 0.035)
    np.testing.assert_allclose(MODULE.object_body_center(cup, 0.6931), [0.32, 0.17, 0.7531])


def test_nominal_axes_follow_each_tool0_convention():
    transform = np.eye(4)
    left_approach, left_closing = MODULE.grasp_axes(transform, "left")
    right_approach, right_closing = MODULE.grasp_axes(transform, "right")
    np.testing.assert_array_equal(left_approach, [0, 0, 1])
    np.testing.assert_array_equal(left_closing, [1, 0, 0])
    np.testing.assert_array_equal(right_approach, [0, -1, 0])
    np.testing.assert_array_equal(right_closing, [1, 0, 0])


def test_residual_has_five_constraints_and_does_not_fix_yaw():
    class FakeChain:
        def transforms(self, _positions):
            transform = np.eye(4)
            transform[:3, 3] = [1.0, 2.0, 3.0]
            return {"left_tool0": transform}

    residual = MODULE.constraint_residual(FakeChain(), "left", np.zeros(5), np.array([1, 2, 3]))
    assert residual.shape == (5,)
    assert residual.tolist() == [0.0, 0.0, 0.0, -1.0, -0.0]


def valid_measurement():
    return {
        "position_m": [0.32, 0.17, 0.7531],
        "approach_axis": [1.0, 0.0, 0.0],
        "closing_axis": [0.0, 1.0, 0.0],
        "position_error_mm": 1.99,
        "approach_horizontal_error_deg": 1.99,
        "closing_horizontal_error_deg": 1.99,
        "approach_heading_error_deg": 29.99,
        "joint_margin_deg": 3.0,
    }


def test_strict_validator_rejects_bad_or_nonfinite_result():
    measured = valid_measurement()
    measured["position_error_mm"] = 2.01
    measured["approach_axis"] = [math.nan, 0.0, 0.0]
    reasons = MODULE.validate_measurement(measured)
    assert any("position_error_mm" in reason for reason in reasons)
    assert any("approach_axis" in reason for reason in reasons)


def test_strict_validator_accepts_only_all_thresholds():
    assert MODULE.validate_measurement(valid_measurement()) == []
    for key in ("approach_horizontal_error_deg", "closing_horizontal_error_deg"):
        measured = valid_measurement()
        measured[key] = 2.0001
        assert MODULE.validate_measurement(measured)
    measured = valid_measurement()
    measured["joint_margin_deg"] = 2.9999
    assert MODULE.validate_measurement(measured)


def test_invalid_object_and_solver_inputs_are_rejected():
    with pytest.raises(ValueError):
        MODULE.ObjectSpec("cup", "left", (math.nan, 0.0), 0.1, 0.03)
    with pytest.raises(ValueError):
        MODULE.ObjectSpec("cup", "middle", (0.3, 0.0), 0.1, 0.03)
    with pytest.raises(ValueError):
        MODULE.constraint_residual(object(), "left", np.zeros(4), np.zeros(3))


def test_plan_side_never_masquerades_failed_stage_as_success(monkeypatch):
    monkeypatch.setattr(
        MODULE,
        "solve_horizontal_endpoint",
        lambda *_args, **_kwargs: np.zeros(5),
    )
    bad = valid_measurement()
    bad["joint_margin_deg"] = 0.0
    monkeypatch.setattr(MODULE, "measure_stage", lambda *_args, **_kwargs: dict(bad))
    spec = MODULE.ObjectSpec("cup", "left", (0.32, 0.17), 0.12, 0.035)
    result = MODULE.plan_side(object(), spec, 0.6931)
    assert not any(stage["accepted"] for stage in result["stages"])
    assert result["blocked"] is True
    assert result["path_checked"] is False
    assert result["planned_blocked"] == ["close", "lift"]


def test_report_contract_uses_tool0_and_emits_no_motion(monkeypatch, tmp_path):
    monkeypatch.setattr(MODULE, "Chain", lambda _path: object())
    monkeypatch.setattr(
        MODULE,
        "plan_side",
        lambda _chain, spec, _table: {
            "tool_frame": f"{spec.side}_tool0",
            "stages": [],
            "blocked": True,
        },
    )
    source = tmp_path / "robot.urdf"
    source.write_text('<robot name="fixture"/>', encoding="utf-8")
    report = MODULE.build_report(source)
    assert report["schema"] == "body_side_grasp_v1"
    assert report["sides"]["right"]["tool_frame"] == "right_tool0"
    assert "right_bottle_tcp" not in str(report)
    assert report["motion_command_emitted"] is False
    assert report["object_attached"] is False
    assert report["path_checked"] is False
    assert any("FinRay" in limit for limit in report["modeling_limits"])


def test_cli_output_is_exclusive(monkeypatch, tmp_path):
    output = tmp_path / "plan.json"
    output.write_text("keep", encoding="utf-8")
    monkeypatch.setattr(MODULE, "build_report", lambda _path: {})
    with pytest.raises(FileExistsError):
        MODULE.main(["--output", str(output)])
    assert output.read_text(encoding="utf-8") == "keep"
