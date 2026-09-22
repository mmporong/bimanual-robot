import importlib.util
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).with_name("plan_safe_recovery.py")
SPEC = importlib.util.spec_from_file_location("plan_safe_recovery", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_neutral_plan_is_motionless_preview():
    plan = MODULE.build_neutral_plan([0.0] * 5)
    assert plan["motion_command_emitted"] is False
    assert plan["ready_for_collision_review"] is True
    assert plan["stages"]["pregrasp"] == plan["stages"]["grasp"]
