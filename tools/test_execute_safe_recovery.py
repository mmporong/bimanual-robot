import importlib.util
import json
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).parent / "servo" / "execute_safe_recovery.py"
SPEC = importlib.util.spec_from_file_location("execute_safe_recovery", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_degree_raw_roundtrip_reference_values():
    assert MODULE.degrees_to_raw(0.0, 778, 3281) == 2030
    assert MODULE.degrees_to_raw(-93.643, 778, 3281) == 964


def test_interpolation_limits_every_synchronized_step():
    start = [2040, 783, 2516, 2956, 1036]
    target = [2041, 964, 2356, 2821, 1183]
    waypoints = MODULE.interpolate_raw(start, target, 23)
    previous = start
    for waypoint in waypoints:
        assert max(abs(b - a) for a, b in zip(previous, waypoint)) <= 23
        previous = waypoint
    assert waypoints[-1] == target


def test_mock_execution_reaches_target_and_releases_torque(tmp_path):
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 783, 2516, 2956, 1036]
    target = [2041, 964, 2356, 2821, 1183]
    bus = MODULE.MultiJointFakeBus(calibration, start)
    waypoints = MODULE.interpolate_raw(start, target, 23)
    result = MODULE.execute(bus, calibration, waypoints, 80, 5, 450, 55)
    assert result["completed"] is True
    assert max(abs(a - b) for a, b in zip(result["final_raw"], target)) <= 8
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)


def test_start_drift_is_rejected_before_torque_enable():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    plan = {
        "start_joint_deg": [0.0] * 5,
    }
    state = {
        "position": [2200, 2030, 1413, 2026, 2048],
        "torque": [0] * 5,
    }
    target = [2045, 2030, 1413, 2026, 2048]
    try:
        MODULE.validate_start(plan, calibration, state, target, 12, 220)
    except RuntimeError as exc:
        assert "자세가 바뀌었습니다" in str(exc)
    else:
        raise AssertionError("stale recovery plan이 허용됨")


def test_goal_seed_write_failure_never_leaves_torque_enabled():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 783, 2516, 2956, 1036]

    class FailingBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(calibration, start)
            self.failed = False

        def write(self, sid, address, value, size=1):
            if address == MODULE.A_GOAL and not self.failed:
                self.failed = True
                return False
            return super().write(sid, address, value, size)

    bus = FailingBus()
    try:
        MODULE.execute(bus, calibration, [start], 80, 5, 450, 55)
    except RuntimeError as exc:
        assert "쓰기 실패" in str(exc)
    else:
        raise AssertionError("goal seed 쓰기 실패가 무시됨")
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)
