import importlib.util
import json
import math
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).with_name("build_pick_schedule.py")
TOOLS_DIR = str(MODULE_PATH.parent)
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)
SPEC = importlib.util.spec_from_file_location("build_pick_schedule", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _inputs():
    pregrasp = [10.0, 20.0, -10.0, 0.0, 5.0]
    grasp = [20.0, 10.0, -5.0, 8.0, 5.0]
    plan = {
        "motion_command_emitted": False,
        "stages": {
            "pregrasp": {"joint_deg": pregrasp.copy()},
            "grasp": {"joint_deg": grasp.copy()},
        },
    }
    audit = {
        "motion_command_emitted": False,
        "continuous_path_ready_for_preview": True,
        "trajectory_ready_for_preview": True,
        "failed_sample_count": 0,
        "start_recovery": {
            "required": False,
            "ready_for_explicit_motion_approval": False,
        },
        "start_joint_deg": [0.0] * 5,
        "target_stages_joint_deg": {
            "pregrasp": pregrasp.copy(),
            "grasp": grasp.copy(),
        },
    }
    return plan, audit


def test_build_schedule_preserves_waypoints_and_uses_common_scalar_progress():
    plan, audit = _inputs()
    result = MODULE.build_schedule(plan, audit, [10.0], sample_period_s=0.25)

    assert result["motion_command_emitted"] is False
    assert result["real_hardware_execution_approved"] is False
    assert result["waypoints"] == [
        {"name": "start", "joint_deg": audit["start_joint_deg"]},
        {"name": "pregrasp", "joint_deg": plan["stages"]["pregrasp"]["joint_deg"]},
        {"name": "grasp", "joint_deg": plan["stages"]["grasp"]["joint_deg"]},
    ]
    pregrasp_samples = [
        sample for sample in result["samples"]
        if sample["phase"] == "start_to_pregrasp"
    ]
    assert pregrasp_samples[-1]["joint_deg"] == plan["stages"]["pregrasp"]["joint_deg"]
    first_intermediate = result["samples"][1]
    progress = first_intermediate["scalar_progress"]
    assert first_intermediate["joint_deg"] == [
        target * progress
        for target in plan["stages"]["pregrasp"]["joint_deg"]
    ]
    assert all(
        following["time_s"] > previous["time_s"]
        for previous, following in zip(result["samples"], result["samples"][1:])
    )


def test_segment_duration_obeys_each_axis_speed_limit():
    plan, audit = _inputs()
    limits = [5.0, 20.0, 10.0, 2.0, 100.0]
    result = MODULE.build_schedule(plan, audit, limits, sample_period_s=0.3)

    for phase, source, target in (
        ("start_to_pregrasp", audit["start_joint_deg"], plan["stages"]["pregrasp"]["joint_deg"]),
        ("pregrasp_to_grasp", plan["stages"]["pregrasp"]["joint_deg"], plan["stages"]["grasp"]["joint_deg"]),
    ):
        duration = result["segment_duration_s"][phase]
        for start_value, end_value, limit in zip(source, target, limits):
            assert abs(end_value - start_value) / duration <= limit + 1e-9


def test_schedule_marks_initial_contact_recovery_separately():
    plan, audit = _inputs()
    audit["trajectory_ready_for_preview"] = False
    audit["failed_sample_count"] = 4
    audit["start_recovery"] = {
        "required": True,
        "ready_for_explicit_motion_approval": True,
    }
    result = MODULE.build_schedule(plan, audit, [10.0])
    assert result["path_review"]["classification"] == "start_recovery_only"
    assert result["path_review"]["collision_free"] is False


def test_schedule_rejects_invalid_numbers_and_excessive_sample_count():
    invalid_limits = ([True], ["1"], [0.0], [-1.0], [math.nan], [math.inf], [1.0, 2.0])
    for limits in invalid_limits:
        plan, audit = _inputs()
        try:
            MODULE.build_schedule(plan, audit, limits)
        except ValueError:
            pass
        else:
            raise AssertionError(f"잘못된 속도 제한이 허용됨: {limits!r}")

    plan, audit = _inputs()
    try:
        MODULE.build_schedule(plan, audit, [0.001], sample_period_s=0.001, max_samples=100)
    except ValueError as exc:
        assert "샘플 수" in str(exc)
    else:
        raise AssertionError("과도한 샘플 일정이 허용됨")


def test_schedule_rejects_nonfinite_waypoint():
    plan, audit = _inputs()
    audit["target_stages_joint_deg"]["grasp"][0] = math.inf
    try:
        MODULE.build_schedule(plan, audit, [10.0])
    except ValueError as exc:
        assert "유한한 숫자 5개" in str(exc)
    else:
        raise AssertionError("유효하지 않은 감사 waypoint가 허용됨")


def test_schedule_rejects_finite_waypoints_whose_delta_overflows():
    plan, audit = _inputs()
    plan["stages"]["pregrasp"]["joint_deg"][0] = 1e308
    audit["target_stages_joint_deg"]["pregrasp"][0] = 1e308
    audit["start_joint_deg"][0] = -1e308
    try:
        MODULE.build_schedule(plan, audit, [1.0])
    except ValueError as exc:
        assert "구간 시간" in str(exc)
    else:
        raise AssertionError("overflow가 발생하는 waypoint 차이가 허용됨")


def test_schedule_preserves_zero_length_waypoint_without_duplicate_time_sample():
    plan, audit = _inputs()
    plan["stages"]["pregrasp"]["joint_deg"] = audit["start_joint_deg"].copy()
    audit["target_stages_joint_deg"]["pregrasp"] = audit["start_joint_deg"].copy()
    result = MODULE.build_schedule(plan, audit, [10.0], sample_period_s=0.25)

    assert result["waypoints"][1] == {
        "name": "pregrasp",
        "joint_deg": audit["start_joint_deg"],
    }
    assert result["segment_duration_s"]["start_to_pregrasp"] == 0.0
    assert all(sample["phase"] != "start_to_pregrasp" for sample in result["samples"])
    assert all(
        following["time_s"] > previous["time_s"]
        for previous, following in zip(result["samples"], result["samples"][1:])
    )


def test_schedule_rejects_positive_duration_that_cannot_advance_elapsed_time():
    plan, audit = _inputs()
    pregrasp = [0.0, 1.0, 0.0, 0.0, 0.0]
    grasp = [math.nextafter(0.0, 1.0), 1.0, 0.0, 0.0, 0.0]
    plan["stages"]["pregrasp"]["joint_deg"] = pregrasp.copy()
    audit["target_stages_joint_deg"]["pregrasp"] = pregrasp.copy()
    plan["stages"]["grasp"]["joint_deg"] = grasp.copy()
    audit["target_stages_joint_deg"]["grasp"] = grasp.copy()

    try:
        MODULE.build_schedule(plan, audit, [1.0] * 5, sample_period_s=0.1)
    except ValueError as exc:
        assert "양수 간격" in str(exc)
    else:
        raise AssertionError("시간축에서 표현할 수 없는 양수 구간이 허용됨")


def test_schedule_cli_refuses_input_or_existing_output(monkeypatch, tmp_path):
    plan, audit = _inputs()
    plan_path = tmp_path / "plan.json"
    audit_path = tmp_path / "audit.json"
    existing_path = tmp_path / "existing.json"
    plan_text = json.dumps(plan)
    plan_path.write_text(plan_text, encoding="utf-8")
    audit_path.write_text(json.dumps(audit), encoding="utf-8")
    existing_path.write_text("preserve", encoding="utf-8")

    for output_path in (plan_path, audit_path, existing_path):
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "build_pick_schedule.py",
                "--plan", str(plan_path),
                "--audit", str(audit_path),
                "--max-joint-speed-deg-s", "10",
                "--output", str(output_path),
            ],
        )
        try:
            MODULE.main()
        except ValueError:
            pass
        else:
            raise AssertionError(f"위험한 출력 경로가 허용됨: {output_path}")

    assert plan_path.read_text(encoding="utf-8") == plan_text
    assert existing_path.read_text(encoding="utf-8") == "preserve"
