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


def test_execution_defaults_to_one_internally_rate_limited_goal():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    plan = {"start_joint_deg": [-0.4396, -106.7692, 96.8791, 80.9231, -88.044]}
    state = {"position": [2040, 815, 2515, 2947, 1046], "torque": [0] * 5}
    target = [2041, 955, 2375, 2835, 1168]
    waypoints = MODULE.validate_start(plan, calibration, state, target, 12, 220)
    assert len(waypoints) == 1
    previous = state["position"]
    for waypoint in waypoints:
        assert max(abs(b - a) for a, b in zip(previous, waypoint)) <= 220
        previous = waypoint


def test_mock_execution_reaches_target_and_releases_torque(tmp_path):
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 783, 2516, 2956, 1036]
    target = [2041, 964, 2356, 2821, 1183]
    bus = MODULE.MultiJointFakeBus(calibration, start)
    waypoints = MODULE.interpolate_raw(start, target, 23)
    result = MODULE.execute(bus, calibration, waypoints, 80, 5, 450, 55)
    assert result["completed"] is True
    assert max(abs(a - b) for a, b in zip(result["released_final_raw"], target)) <= 8
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


def test_continuous_stage_accepts_all_torque_enabled():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    plan = {"start_joint_deg": [0.0] * 5}
    state = {
        "position": [2045, 2030, 1413, 2026, 2048],
        "torque": [1] * 5,
    }
    target = [2045, 2030, 1413, 2026, 2048]
    assert MODULE.validate_start(plan, calibration, state, target, 12, 220, 220, True)


def test_continuous_stage_rejects_partially_enabled_torque():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    plan = {"start_joint_deg": [0.0] * 5}
    state = {
        "position": [2045, 2030, 1413, 2026, 2048],
        "torque": [1, 1, 0, 1, 1],
    }
    try:
        MODULE.validate_start(plan, calibration, state, state["position"], 12, 220, 220, True)
    except RuntimeError as exc:
        assert "전부" in str(exc)
    else:
        raise AssertionError("일부 토크만 켜진 상태가 허용됨")


def test_position_only_start_read_skips_load_and_temperature():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class NoTelemetryBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if address in (MODULE.A_LOAD, MODULE.A_TEMP):
                raise AssertionError("position-only에서 telemetry를 읽음")
            return super().read(sid, address, size)

    state = MODULE.read_all(NoTelemetryBus(calibration, start), calibration, position_only=True)
    assert state["position"] == start
    assert state["temperature"] == []
    assert state["load"] == []


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


def test_single_temperature_spike_requires_confirmation():
    class TemperatureBus:
        def __init__(self):
            self.values = {1: [85, 35], 2: [34, 34]}

        def read(self, sid, address, size=1):
            return self.values[sid].pop(0)

    assert MODULE.confirmed_temperatures(TemperatureBus(), [1, 2], 55) == [35, 34]


def test_torque_release_sag_is_not_reported_as_completed():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]
    target = [2040, 980, 2420, 2820, 1180]

    class SaggingBus(MODULE.MultiJointFakeBus):
        def write(self, sid, address, value, size=1):
            result = super().write(sid, address, value, size)
            if address == MODULE.A_TORQUE and value == 0 and sid in (2, 3):
                self.positions[sid] += 40
            return result

    bus = SaggingBus(calibration, start)
    result = MODULE.execute(bus, calibration, [target], 80, 5, 450, 55)
    assert result["target_reached_with_torque"] is True
    assert result["persistent_after_torque_release"] is False
    assert result["completed"] is False


def test_success_can_hold_torque_for_the_next_continuous_stage():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]
    target = [2040, 980, 2420, 2820, 1180]
    bus = MODULE.MultiJointFakeBus(calibration, start)
    result = MODULE.execute(
        bus, calibration, [target], 80, 5, 450, 55,
        hold_torque_on_success=True,
    )
    assert result["completed"] is True
    assert result["torque_held"] is True
    assert result["persistent_after_torque_release"] is None
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 1 for name in MODULE.JOINTS)


def test_hold_mode_still_releases_torque_after_failure():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class OverloadBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if address == MODULE.A_LOAD and self.reg[sid][MODULE.A_TORQUE]:
                return 700
            return super().read(sid, address, size)

    bus = OverloadBus(calibration, start)
    try:
        MODULE.execute(
            bus, calibration, [[2040, 980, 2420, 2820, 1180]], 80, 5, 450, 55,
            hold_torque_on_success=True,
        )
    except RuntimeError as exc:
        assert "부하 상한" in str(exc)
    else:
        raise AssertionError("과부하가 무시됨")
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)


def test_hold_mode_releases_torque_if_profile_restore_fails():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class RestoreFailureBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(calibration, start)
            self.speed_writes = {}

        def write(self, sid, address, value, size=1):
            if address == MODULE.A_SPEED:
                count = self.speed_writes.get(sid, 0) + 1
                self.speed_writes[sid] = count
                if count == 2:
                    return False
            return super().write(sid, address, value, size)

    bus = RestoreFailureBus()
    try:
        MODULE.execute(
            bus, calibration, [start], 80, 5, 450, 55,
            hold_torque_on_success=True,
        )
    except RuntimeError as exc:
        assert "쓰기 실패" in str(exc)
    else:
        raise AssertionError("프로파일 복원 실패가 무시됨")
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)


def test_hold_mode_releases_all_torque_if_torque_verification_is_partial():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class PartialTorqueBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(calibration, start)
            self.verify_reads = 0

        def read(self, sid, address, size=1):
            if address == MODULE.A_TORQUE and all(self.reg[item][MODULE.A_TORQUE] for item in self.reg):
                self.verify_reads += 1
                if self.verify_reads == 8:
                    return 0
            return super().read(sid, address, size)

    bus = PartialTorqueBus()
    try:
        MODULE.execute(
            bus, calibration, [start], 80, 5, 450, 55,
            hold_torque_on_success=True,
        )
    except RuntimeError as exc:
        assert "토크 유지 확인 실패" in str(exc)
    else:
        raise AssertionError("부분 토크 유지가 성공으로 처리됨")
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)


def test_hold_mode_releases_torque_on_base_exception_during_restore():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class InterruptingRestoreBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(calibration, start)
            self.speed_writes = {}

        def write(self, sid, address, value, size=1):
            if address == MODULE.A_SPEED:
                count = self.speed_writes.get(sid, 0) + 1
                self.speed_writes[sid] = count
                if count == 2:
                    raise KeyboardInterrupt("restore interrupted")
            return super().write(sid, address, value, size)

    bus = InterruptingRestoreBus()
    try:
        MODULE.execute(
            bus, calibration, [start], 80, 5, 450, 55,
            hold_torque_on_success=True,
        )
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError("복원 중 인터럽트가 전달되지 않음")
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 0 for name in MODULE.JOINTS)
