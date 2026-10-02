import importlib.util
import json
from pathlib import Path
import sys

import pytest


MODULE_PATH = Path(__file__).parent / "servo" / "execute_safe_recovery.py"
SPEC = importlib.util.spec_from_file_location("execute_safe_recovery", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_degree_raw_roundtrip_reference_values():
    assert MODULE.degrees_to_raw(0.0, 778, 3281) == 2030
    assert MODULE.degrees_to_raw(-93.643, 778, 3281) == 964


def _bound_inputs(tmp_path):
    calibration_path = MODULE.DEFAULT_CALIBRATION
    plan = {
        "motion_command_emitted": False,
        "ready_for_explicit_motion_approval": True,
        "start_joint_deg": [0.0] * 5,
        "recovery": {"target_joint_deg": [1.0] * 5},
    }
    plan["calibration_binding"] = MODULE.calibration_binding(plan, calibration_path.read_bytes())
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(plan), encoding="utf-8")
    return path, calibration_path, plan


def test_bound_plan_uses_current_repository_calibration(tmp_path):
    path, calibration_path, plan = _bound_inputs(tmp_path)
    _, _, raw_ticks = MODULE.load_inputs(path, calibration_path)
    assert raw_ticks == plan["calibration_binding"]["target_raw"]
    assert plan["calibration_binding"]["servo_ids"] == [1, 2, 3, 4, 5]


def test_bound_plan_rejects_changed_calibration(tmp_path):
    path, calibration_path, _ = _bound_inputs(tmp_path)
    changed = tmp_path / "changed.json"
    changed.write_bytes(calibration_path.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="calibration"):
        MODULE.load_inputs(path, changed)


@pytest.mark.parametrize("key,value", [
    ("target_raw", [0] * 5), ("servo_ids", [5, 4, 3, 2, 1]),
    ("joint_order", list(reversed(MODULE.JOINTS))),
])
def test_bound_plan_rejects_inconsistent_commands(tmp_path, key, value):
    path, calibration_path, plan = _bound_inputs(tmp_path)
    plan["calibration_binding"][key] = value
    path.write_text(json.dumps(plan), encoding="utf-8")
    with pytest.raises(ValueError, match="raw"):
        MODULE.load_inputs(path, calibration_path)


@pytest.mark.parametrize("value_deg", [float("nan"), float("inf"), True, 500.0])
def test_binding_rejects_invalid_or_out_of_range_angle(tmp_path, value_deg):
    _, calibration_path, plan = _bound_inputs(tmp_path)
    plan["recovery"]["target_joint_deg"][0] = value_deg
    with pytest.raises(ValueError):
        MODULE.calibration_binding(plan, calibration_path.read_bytes())


def test_legacy_unbound_plan_remains_supported(tmp_path):
    path, calibration_path, plan = _bound_inputs(tmp_path)
    del plan["calibration_binding"]
    path.write_text(json.dumps(plan), encoding="utf-8")
    assert len(MODULE.load_inputs(path, calibration_path)[2]) == 5


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
    plan["start_joint_deg"] = [(raw - (calibration[n]["range_min"] + calibration[n]["range_max"]) / 2)
                               * 360 / 4095 for n, raw in zip(MODULE.JOINTS, state["position"])]
    target = [2041, 955, 2375, 2835, 1168]
    waypoints = MODULE.validate_start(plan, calibration, state, target, 12, 220)
    assert len(waypoints) == 1
    previous = state["position"]
    for waypoint in waypoints:
        assert max(abs(b - a) for a, b in zip(previous, waypoint)) <= 220
        previous = waypoint


def test_mock_execution_reaches_target_and_releases_torque(tmp_path):
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2516, 2900, 1036]
    target = [2041, 964, 2356, 2821, 1183]
    bus = MODULE.MultiJointFakeBus(calibration, start)
    waypoints = MODULE.interpolate_raw(start, target, 23)
    result = MODULE.execute(bus, calibration, waypoints, 80, 5, 450, 55,
                            hold_torque_on_success=False)
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
    plan["start_joint_deg"] = [(raw - (calibration[n]["range_min"] + calibration[n]["range_max"]) / 2)
                               * 360 / 4095 for n, raw in zip(MODULE.JOINTS, state["position"])]
    assert MODULE.validate_start(plan, calibration, state, target, 12, 220, 220, True)


def test_monitor_stop_freezes_goal_without_releasing_torque():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    start = [int((cal[n]["range_min"] + cal[n]["range_max"]) / 2) for n in MODULE.JOINTS]
    bus = MODULE.MultiJointFakeBus(cal, start)
    calls = 0

    def monitor():
        nonlocal calls
        calls += 1
        if calls == 3:
            raise RuntimeError("dashboard_stop_requested")

    with pytest.raises(RuntimeError, match="dashboard_stop_requested"):
        MODULE.execute(bus, cal, [[v + 30 for v in start]], 80, 5, 450, 55,
                       hold_torque_on_success=True, position_only=True, monitor=monitor)
    assert all(bus.reg[cal[n]["id"]][MODULE.A_TORQUE] == 1 for n in MODULE.JOINTS)
    assert bus.goals == bus.positions


def test_monitor_preflight_failure_never_enables_torque():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    bus = MODULE.MultiJointFakeBus(cal, [2000] * 5)

    def monitor():
        raise RuntimeError("camera_stale")

    with pytest.raises(RuntimeError, match="camera_stale"):
        MODULE.execute(bus, cal, [[2020] * 5], 80, 5, 450, 55, position_only=True, monitor=monitor)
    assert all(bus.reg[cal[n]["id"]][MODULE.A_TORQUE] == 0 for n in MODULE.JOINTS)


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
    start = [2040, 900, 2516, 2900, 1036]

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
    result = MODULE.execute(bus, calibration, [target], 80, 5, 450, 55,
                            hold_torque_on_success=False)
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


def test_failure_cancels_goal_without_dropping_arm():
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
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 1 for name in MODULE.JOINTS)
    assert bus.goals == bus.positions


def test_profile_restore_failure_does_not_release_torque():
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
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 1 for name in MODULE.JOINTS)


def test_torque_verification_failure_does_not_drop_other_joints():
    calibration = json.loads(MODULE.DEFAULT_CALIBRATION.read_text(encoding="utf-8"))
    start = [2040, 900, 2500, 2900, 1100]

    class PartialTorqueBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(calibration, start)
            self.verify_reads = 0

        def read(self, sid, address, size=1):
            if address == MODULE.A_TORQUE and all(self.reg[item][MODULE.A_TORQUE] for item in self.reg):
                self.verify_reads += 1
                if self.verify_reads == 13:
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
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 1 for name in MODULE.JOINTS)


def test_interrupt_during_restore_does_not_release_torque():
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
    assert all(bus.reg[int(calibration[name]["id"])][MODULE.A_TORQUE] == 1 for name in MODULE.JOINTS)


def test_stall_freezes_present_position_and_never_writes_torque_off(monkeypatch):
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    start = [2000] * 5

    class StuckBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if address == MODULE.A_POS:
                return self.positions[sid]
            if address in (MODULE.A_LOAD, MODULE.A_TEMP):
                raise AssertionError("금지된 telemetry 판독")
            return super().read(sid, address, size)

        def write(self, sid, address, value, size=1):
            if address == MODULE.A_TORQUE and value == 0:
                raise AssertionError("이동 실패에서 토크 해제")
            return super().write(sid, address, value, size)

    seconds = [0.0]
    def sleep(duration):
        seconds[0] += duration
    monkeypatch.setattr(MODULE.time, "sleep", sleep)
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: seconds[0])
    bus = StuckBus(cal, start)
    with pytest.raises(RuntimeError, match="스톨") as raised:
        MODULE.execute(bus, cal, [[2040] * 5], 80, 5, 450, 55, position_only=True)
    assert bus.goals == bus.positions
    assert raised.value.motion_stop_report["hold_command_verified"] is True
    assert all(bus.reg[sid][MODULE.A_TORQUE] == 1 for sid in bus.reg)


def test_hold_keeps_disabled_joint_off_and_continues_after_read_failure():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    class ReadFailureBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if sid == 2 and address == MODULE.A_POS:
                return None
            return super().read(sid, address, size)

        def write(self, sid, address, value, size=1):
            assert address != MODULE.A_TORQUE
            return super().write(sid, address, value, size)

    bus = ReadFailureBus(cal, [2000] * 5)
    for sid in bus.reg:
        bus.reg[sid][MODULE.A_TORQUE] = int(sid != 3)
        bus.goals[sid] = 2100
    report = MODULE.hold_current(bus, list(bus.reg))
    assert report["hold_command_verified"] is False
    assert len(report["errors"]) == 1
    assert bus.reg[3][MODULE.A_TORQUE] == 0
    assert bus.reg[1][MODULE.A_TORQUE] == 1
    assert bus.goals[5] == bus.positions[5]


def test_default_success_holds_torque():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    bus = MODULE.MultiJointFakeBus(cal, [2000] * 5)
    result = MODULE.execute(bus, cal, [[2020] * 5], 80, 5, 450, 55, position_only=True)
    assert result["torque_held"] is True


def test_preflight_error_cancels_previous_goal_when_already_enabled():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    bus = MODULE.MultiJointFakeBus(cal, [2000] * 5)
    for sid in bus.reg:
        bus.reg[sid][MODULE.A_TORQUE] = 1
        bus.goals[sid] = 2200
    def monitor():
        raise RuntimeError("camera_stale")
    with pytest.raises(RuntimeError, match="camera_stale") as raised:
        MODULE.execute(bus, cal, [[2200] * 5], 80, 5, 450, 55, monitor=monitor)
    assert raised.value.motion_stop_report["hold_command_verified"]
    assert bus.goals == bus.positions


def test_explicit_release_partial_failure_is_not_success():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    class PartialReleaseBus(MODULE.MultiJointFakeBus):
        def write(self, sid, address, value, size=1):
            if sid == 3 and address == MODULE.A_TORQUE and value == 0:
                return False
            return super().write(sid, address, value, size)
    bus = PartialReleaseBus(cal, [2000] * 5)
    with pytest.raises(RuntimeError, match="토크 해제 미확인"):
        MODULE.execute(bus, cal, [[2020] * 5], 80, 5, 450, 55,
                       hold_torque_on_success=False, position_only=True)


def test_one_tick_start_boundary_keeps_seed_inside_calibration():
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    start = [2000] * 5
    start[2] = cal["elbow_flex"]["range_max"] + 1
    class SeedBus(MODULE.MultiJointFakeBus):
        first_elbow_goal = None
        def write(self, sid, address, value, size=1):
            if sid == 3 and address == MODULE.A_GOAL and self.first_elbow_goal is None:
                self.first_elbow_goal = value
            return super().write(sid, address, value, size)
    bus = SeedBus(cal, start)
    target = start.copy()
    target[2] -= 40
    MODULE.execute(bus, cal, [target], 80, 5, 450, 55, position_only=True)
    assert bus.first_elbow_goal == cal["elbow_flex"]["range_max"]


def test_stream_reaches_final_goal_without_waiting_at_each_small_target(monkeypatch):
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    class DeadbandBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if address == MODULE.A_POS:
                delta = self.goals[sid] - self.positions[sid]
                if self.reg[sid][MODULE.A_TORQUE] and abs(delta) > 25:
                    self.positions[sid] += max(-8, min(8, delta))
                return self.positions[sid]
            return super().read(sid, address, size)
    seconds = [0.0]
    def sleep(duration):
        seconds[0] += duration
    monkeypatch.setattr(MODULE.time, "sleep", sleep)
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: seconds[0])
    bus = DeadbandBus(cal, [2000] * 5)
    result = MODULE.execute(bus, cal, MODULE.interpolate_raw([2000]*5, [2200]*5, 5),
                            80, 5, 450, 55, arrival_tolerance_ticks=30,
                            position_only=True, waypoint_period_s=.1)
    assert result["completed"]
    assert all(2170 <= value <= 2200 for value in result["loaded_final_raw"])


def test_stream_stuck_joint_stops_at_tracking_bound_without_torque_off(monkeypatch):
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    class StuckBus(MODULE.MultiJointFakeBus):
        def read(self, sid, address, size=1):
            if address == MODULE.A_POS:
                return self.positions[sid]
            return super().read(sid, address, size)
    seconds = [0.0]
    def sleep(duration):
        seconds[0] += duration
    monkeypatch.setattr(MODULE.time, "sleep", sleep)
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: seconds[0])
    bus = StuckBus(cal, [2000] * 5)
    with pytest.raises(RuntimeError, match="추종 상한") as raised:
        MODULE.execute(bus, cal, MODULE.interpolate_raw([2000]*5, [2200]*5, 5),
                       80, 5, 450, 55, position_only=True, waypoint_period_s=.1)
    assert raised.value.motion_stop_report["hold_command_verified"]
    assert bus.goals == bus.positions


def _feedback_fixture(deadband=37):
    cal = json.loads(MODULE.DEFAULT_CALIBRATION.read_text())
    class FeedbackBus(MODULE.MultiJointFakeBus):
        def __init__(self):
            super().__init__(cal, [2000] * 5)
            self.writes = []
            for reg in self.reg.values():
                reg[MODULE.A_TORQUE] = 1
        def read(self, sid, address, size=1):
            if address in (MODULE.A_LOAD, MODULE.A_TEMP):
                raise AssertionError("부하·온도 판독 금지")
            if address == MODULE.A_POS and sid == cal["shoulder_lift"]["id"]:
                delta = self.goals[sid] - self.positions[sid]
                if self.reg[sid][MODULE.A_TORQUE] and abs(delta) > deadband:
                    self.positions[sid] += (1 if delta > 0 else -1) * min(8, abs(delta)-deadband)
                return self.positions[sid]
            return super().read(sid, address, size)
        def write(self, sid, address, value, size=1):
            self.writes.append((sid, address, value))
            return super().write(sid, address, value, size)
    seconds = [0.0]
    def sleep(duration):
        seconds[0] += duration
    return cal, FeedbackBus(), sleep, lambda: seconds[0]


def test_encoder_feedback_reaches_actual_target_without_changing_gain_or_torque():
    cal, bus, sleep, clock = _feedback_fixture()
    result = MODULE.execute_encoder_feedback(bus, cal, [2000, 1950, 2000, 2000, 2000], sleep=sleep, clock=clock)
    assert result["completed"]
    assert abs(result["actual_raw"][1]-1950) <= 10
    assert -40 <= result["compensation_ticks"][1] < 0
    assert result["command_raw"][1] < result["target_raw"][1]
    assert max(abs(a-b) for s in result["samples"] for a, b in zip(s["command_raw"], s["actual_raw"])) <= 45
    assert all(address in (MODULE.A_GOAL, MODULE.A_SPEED, MODULE.A_ACCEL) for _, address, _ in bus.writes)
    assert all(reg[MODULE.A_TORQUE] == 1 for reg in bus.reg.values())


def test_encoder_feedback_stuck_motor_freezes_without_torque_off():
    cal, bus, sleep, clock = _feedback_fixture(deadband=1000)
    with pytest.raises(RuntimeError, match="위치 변화 없음") as raised:
        MODULE.execute_encoder_feedback(bus, cal, [2000, 1950, 2000, 2000, 2000], sleep=sleep, clock=clock)
    assert raised.value.motion_stop_report["hold_command_verified"]
    assert bus.goals == bus.positions
    assert not any(a == MODULE.A_TORQUE for _, a, _ in bus.writes)


def test_encoder_feedback_without_compensation_does_not_claim_arrival():
    cal, bus, sleep, clock = _feedback_fixture()
    with pytest.raises(RuntimeError):
        MODULE.execute_encoder_feedback(bus, cal, [2000, 1950, 2000, 2000, 2000],
                                        compensation_limit_ticks=0, sleep=sleep, clock=clock)
    assert bus.goals == bus.positions


def test_encoder_feedback_monitor_interrupt_freezes_existing_goals():
    cal, bus, sleep, clock = _feedback_fixture()
    def interrupt():
        raise KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt) as raised:
        MODULE.execute_encoder_feedback(bus, cal, [2000, 1950, 2000, 2000, 2000],
                                        monitor=interrupt, sleep=sleep, clock=clock)
    assert raised.value.motion_stop_report["hold_command_verified"]
    assert not any(a == MODULE.A_TORQUE for _, a, _ in bus.writes)


def test_encoder_feedback_timeout_cannot_become_success_inside_arrival_band():
    cal, bus, sleep, clock = _feedback_fixture()
    calls = [0]
    def monitor():
        calls[0] += 1
        if calls[0] == 2:
            sleep(11)
    with pytest.raises(RuntimeError, match="도달하지 못했습니다"):
        MODULE.execute_encoder_feedback(bus, cal, [2000]*5, monitor=monitor, sleep=sleep, clock=clock)


def test_encoder_feedback_tracking_clamp_cannot_emit_large_goal_jump():
    cal, bus, sleep, clock = _feedback_fixture()
    original_read = bus.read
    calls = [0]
    sid2 = cal["shoulder_lift"]["id"]
    def read(sid, address, size=1):
        if sid == sid2 and address == MODULE.A_POS:
            calls[0] += 1
            if calls[0] == 2:
                bus.positions[sid] = 1900
        return original_read(sid, address, size)
    bus.read = read
    with pytest.raises(RuntimeError, match="5 tick 전달 제한"):
        MODULE.execute_encoder_feedback(bus, cal, [2000,1800,2000,2000,2000], sleep=sleep, clock=clock)
    assert not any(sid == sid2 and addr == MODULE.A_GOAL and value == 1945 for sid, addr, value in bus.writes)


def test_encoder_feedback_continuation_keeps_previous_compensated_goal():
    cal, bus, sleep, clock = _feedback_fixture()
    target = [2000,1950,2000,2000,2000]
    first = MODULE.execute_encoder_feedback(bus, cal, target, sleep=sleep, clock=clock)
    previous = first["command_raw"][1]
    bus.writes.clear()
    second = MODULE.execute_encoder_feedback(bus, cal, target, tolerance_ticks=3, sleep=sleep, clock=clock)
    assert abs(second["actual_raw"][1]-target[1]) <= 3
    sid2 = cal["shoulder_lift"]["id"]
    commands = [value for sid, addr, value in bus.writes if sid == sid2 and addr == MODULE.A_GOAL]
    assert commands
    assert max(abs(a-b) for a,b in zip([previous]+commands[:-1], commands)) <= 5


def test_encoder_feedback_refuses_unfinished_far_away_previous_goal():
    cal, bus, sleep, clock = _feedback_fixture()
    bus.goals[cal["shoulder_lift"]["id"]] = 1800
    with pytest.raises(RuntimeError, match="초기 유지 명령"):
        MODULE.execute_encoder_feedback(bus, cal, [2000]*5, sleep=sleep, clock=clock)


def test_encoder_feedback_default_bound_rejects_larger_deadband():
    cal, bus, sleep, clock = _feedback_fixture(deadband=45)
    with pytest.raises(RuntimeError, match="shoulder_lift:.*위치 변화 없음") as raised:
        MODULE.execute_encoder_feedback(bus, cal, [2000,1950,2000,2000,2000],
                                        tolerance_ticks=3, sleep=sleep, clock=clock)
    assert raised.value.encoder_feedback_report["actual_raw"][1] != 1950
    assert raised.value.encoder_feedback_report["samples"]
    assert raised.value.motion_stop_report["hold_command_verified"]


def test_encoder_feedback_explicit_bound_can_reach_same_actual_target():
    cal, bus, sleep, clock = _feedback_fixture(deadband=45)
    initial_goals = bus.goals.copy()
    result = MODULE.execute_encoder_feedback(bus, cal, [2000,1950,2000,2000,2000],
                                            compensation_limit_ticks=50, tracking_limit_ticks=60,
                                            tolerance_ticks=3, sleep=sleep, clock=clock)
    assert result["completed"]
    assert abs(result["actual_raw"][1]-1950) <= 3
    assert -50 <= result["compensation_ticks"][1] < 0
    assert max(abs(a-b) for s in result["samples"]
               for a,b in zip(s["command_raw"],s["actual_raw"])) <= 60
    for sid, initial in initial_goals.items():
        commands = [v for i,a,v in bus.writes if i == sid and a == MODULE.A_GOAL]
        assert max(abs(a-b) for a,b in zip([initial]+commands[:-1],commands)) <= 5
    assert all(a in (MODULE.A_GOAL,MODULE.A_SPEED,MODULE.A_ACCEL) for _,a,_ in bus.writes)


def test_encoder_feedback_larger_bound_still_stops_stuck_motor():
    cal, bus, sleep, clock = _feedback_fixture(deadband=1000)
    with pytest.raises(RuntimeError, match="위치 변화 없음") as raised:
        MODULE.execute_encoder_feedback(bus, cal, [2000,1950,2000,2000,2000],
                                        compensation_limit_ticks=50, tracking_limit_ticks=60,
                                        tolerance_ticks=3, sleep=sleep, clock=clock)
    assert raised.value.motion_stop_report["hold_command_verified"]
    assert not any(a == MODULE.A_TORQUE for _,a,_ in bus.writes)


@pytest.mark.parametrize("options", [{"compensation_limit_ticks":51},{"tracking_limit_ticks":61}])
def test_encoder_feedback_excessive_bound_rejected_before_writes(options):
    cal, bus, sleep, clock = _feedback_fixture()
    with pytest.raises(ValueError, match="피드백 보정 범위"):
        MODULE.execute_encoder_feedback(bus, cal, [2000]*5, sleep=sleep, clock=clock, **options)
    assert bus.writes == []


def test_encoder_feedback_final_monitor_position_change_is_not_stale_success():
    cal, bus, sleep, clock = _feedback_fixture()
    calls = [0]
    def monitor():
        calls[0] += 1
        if calls[0] == 5:
            bus.positions[cal["shoulder_lift"]["id"]] = 1980
    with pytest.raises(RuntimeError, match="최종 위치") as raised:
        MODULE.execute_encoder_feedback(bus, cal, [2000]*5, monitor=monitor, sleep=sleep, clock=clock)
    assert raised.value.encoder_feedback_report["actual_raw"][1] == 1980
    assert raised.value.motion_stop_report["hold_command_verified"]


def test_encoder_feedback_final_monitor_timeout_is_not_success():
    cal, bus, sleep, clock = _feedback_fixture()
    calls = [0]
    def monitor():
        calls[0] += 1
        if calls[0] == 5:
            sleep(11)
    with pytest.raises(RuntimeError, match="종료 확인 중"):
        MODULE.execute_encoder_feedback(bus, cal, [2000]*5, monitor=monitor, sleep=sleep, clock=clock)


def test_encoder_feedback_failure_restores_speed_and_acceleration_after_freeze():
    cal, bus, sleep, clock = _feedback_fixture(deadband=1000)
    prior = {sid:(reg[MODULE.A_SPEED],reg[MODULE.A_ACCEL]) for sid,reg in bus.reg.items()}
    with pytest.raises(RuntimeError):
        MODULE.execute_encoder_feedback(bus, cal, [2000,1950,2000,2000,2000], sleep=sleep, clock=clock)
    assert bus.goals == bus.positions
    assert {sid:(reg[MODULE.A_SPEED],reg[MODULE.A_ACCEL]) for sid,reg in bus.reg.items()} == prior
    assert not any(a == MODULE.A_TORQUE for _,a,_ in bus.writes)
