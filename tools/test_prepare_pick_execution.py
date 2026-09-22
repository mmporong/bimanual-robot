import importlib.util
import json
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).with_name("prepare_pick_execution.py")
SPEC = importlib.util.spec_from_file_location("prepare_pick_execution", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _inputs():
    target = [0.0, -25.0, 31.0, 49.0, 0.0]
    plan = {
        "motion_command_emitted": False,
        "stages": {
            "pregrasp": {"joint_deg": target},
            "grasp": {"joint_deg": [1.0, -2.0, 3.0, -4.0, 5.0]},
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
        "start_joint_deg": [0.0, -96.0, 96.0, 71.0, -77.0],
        "target_stages_joint_deg": {
            "pregrasp": target.copy(),
            "grasp": [1.0, -2.0, 3.0, -4.0, 5.0],
        },
    }
    return plan, audit


def test_prepare_accepts_only_the_audited_pregrasp_target():
    plan, audit = _inputs()
    result = MODULE.prepare(plan, audit)
    assert result["ready_for_explicit_motion_approval"] is True
    assert result["recovery"]["source_stage"] == "pregrasp"
    assert result["recovery"]["target_joint_deg"] == plan["stages"]["pregrasp"]["joint_deg"]


def test_prepare_rejects_target_changed_after_audit():
    plan, audit = _inputs()
    plan["stages"]["pregrasp"]["joint_deg"][1] += 1.0
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "목표가 다릅니다" in str(exc)
    else:
        raise AssertionError("감사 뒤 바뀐 목표가 허용됨")


def test_prepare_rejects_noncontinuous_path():
    plan, audit = _inputs()
    audit["continuous_path_ready_for_preview"] = False
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "통과하지 못했습니다" in str(exc)
    else:
        raise AssertionError("실패한 연속 경로가 허용됨")


def test_prepare_accepts_audited_grasp_stage():
    plan, audit = _inputs()
    result = MODULE.prepare(plan, audit, "grasp")
    assert result["recovery"]["source_stage"] == "grasp"
    assert result["recovery"]["target_joint_deg"] == plan["stages"]["grasp"]["joint_deg"]
    assert result["start_joint_deg"] == plan["stages"]["pregrasp"]["joint_deg"]


def test_prepare_rejects_pregrasp_changed_after_grasp_audit():
    plan, audit = _inputs()
    plan["stages"]["pregrasp"]["joint_deg"][0] = 99.0
    try:
        MODULE.prepare(plan, audit, "grasp")
    except ValueError as exc:
        assert "pregrasp 목표가 다릅니다" in str(exc)
    else:
        raise AssertionError("감사 후 변조된 pregrasp가 grasp 시작값으로 허용됨")


def test_prepare_rejects_unaudited_other_stage_even_for_pregrasp():
    plan, audit = _inputs()
    plan["stages"]["grasp"]["joint_deg"][2] += 0.1
    try:
        MODULE.prepare(plan, audit, "pregrasp")
    except ValueError as exc:
        assert "grasp 목표가 다릅니다" in str(exc)
    else:
        raise AssertionError("감사 후 바뀐 후속 단계가 허용됨")


def test_prepare_rejects_non_numeric_or_non_finite_joint_values():
    for invalid in (True, "0.0", float("nan"), float("inf"), float("-inf")):
        plan, audit = _inputs()
        plan["stages"]["pregrasp"]["joint_deg"][0] = invalid
        try:
            MODULE.prepare(plan, audit)
        except ValueError as exc:
            assert "유한한 숫자 5개" in str(exc)
        else:
            raise AssertionError(f"잘못된 관절값이 허용됨: {invalid!r}")


def test_prepare_rejects_invalid_audited_start_vector():
    plan, audit = _inputs()
    audit["start_joint_deg"] = [0.0, 1.0, 2.0, 3.0, float("nan")]
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "시작 관절값" in str(exc)
    else:
        raise AssertionError("유효하지 않은 감사 시작값이 허용됨")


def test_prepare_labels_initial_contact_recovery_as_not_collision_free():
    plan, audit = _inputs()
    audit["trajectory_ready_for_preview"] = False
    audit["failed_sample_count"] = 4
    audit["start_recovery"] = {
        "required": True,
        "ready_for_explicit_motion_approval": True,
    }
    result = MODULE.prepare(plan, audit)
    assert result["path_review"] == {
        "classification": "start_recovery_only",
        "collision_free": False,
        "start_recovery_required": True,
    }


def test_prepare_rejects_inconsistent_continuous_flag_without_recovery_evidence():
    plan, audit = _inputs()
    audit["trajectory_ready_for_preview"] = False
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "복구 근거" in str(exc)
    else:
        raise AssertionError("근거 없는 continuous=true 감사가 허용됨")


def test_prepare_recovery_requires_positive_integer_failed_sample_count():
    for invalid in (None, 0, True, -1, "1", 1.0):
        plan, audit = _inputs()
        audit["trajectory_ready_for_preview"] = False
        audit["failed_sample_count"] = invalid
        audit["start_recovery"] = {
            "required": True,
            "ready_for_explicit_motion_approval": True,
        }
        try:
            MODULE.prepare(plan, audit)
        except ValueError as exc:
            assert "양의 정수 실패 샘플 수" in str(exc)
        else:
            raise AssertionError(f"잘못된 실패 샘플 수가 허용됨: {invalid!r}")


def test_prepare_requires_literal_true_continuous_flag():
    for invalid in (1, "true", [True]):
        plan, audit = _inputs()
        audit["continuous_path_ready_for_preview"] = invalid
        try:
            MODULE.prepare(plan, audit)
        except ValueError as exc:
            assert "통과하지 못했습니다" in str(exc)
        else:
            raise AssertionError(f"bool이 아닌 연속 경로 표시가 허용됨: {invalid!r}")


def test_prepare_rejects_collision_free_with_failed_samples():
    plan, audit = _inputs()
    audit["failed_sample_count"] = 1
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "모순" in str(exc)
    else:
        raise AssertionError("실패 샘플이 있는 충돌 없음 표시가 허용됨")


def test_prepare_rejects_collision_free_with_recovery_required():
    plan, audit = _inputs()
    audit["start_recovery"] = {
        "required": True,
        "ready_for_explicit_motion_approval": True,
    }
    try:
        MODULE.prepare(plan, audit)
    except ValueError as exc:
        assert "모순" in str(exc)
    else:
        raise AssertionError("복구가 필요한 충돌 없음 표시가 허용됨")


def test_prepare_cli_refuses_input_or_existing_output(monkeypatch, tmp_path):
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
                "prepare_pick_execution.py",
                "--plan", str(plan_path),
                "--audit", str(audit_path),
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
