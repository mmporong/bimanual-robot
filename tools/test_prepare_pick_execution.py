import importlib.util
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
        assert "pregrasp 시작값" in str(exc)
    else:
        raise AssertionError("감사 후 변조된 pregrasp가 grasp 시작값으로 허용됨")
