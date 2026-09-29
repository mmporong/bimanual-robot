import pytest

from servo.small_raw_jog import run, SID, OPERATING_MODE
from servo.sts_bus import (A_ACCEL, A_GOAL, A_MAX_ANGLE, A_MIN_ANGLE, A_OFFSET,
                          A_PHASE, A_POS, A_SPEED, A_TORQUE)
from servo.servo_record_ranges import SO101


class Clock:
    def __init__(self):
        self.now = 0.0

    def sleep(self, duration):
        self.now += duration

    def __call__(self):
        return self.now


class MockBus:
    def __init__(self, stalled=False):
        self.pos = {sid: 1000 for sid in range(1, 7)}
        self.torque = {sid: 0 for sid in range(1, 7)}
        self.reg = {A_GOAL: 3300, A_SPEED: 321, A_ACCEL: 12,
                    A_MIN_ANGLE: 0, A_MAX_ANGLE: 4095, OPERATING_MODE: 0}
        self.writes = []
        self.stalled = stalled

    def read(self, sid, address, size=1):
        assert address in {A_POS, A_TORQUE, *self.reg}
        if address == A_TORQUE:
            return self.torque[sid]
        if address == A_POS:
            if sid == SID and self.torque[sid] and not self.stalled:
                error = self.reg[A_GOAL] - self.pos[sid]
                self.pos[sid] += max(-8, min(8, error))
            return self.pos[sid]
        return self.reg[address]

    def write(self, sid, address, value, size=1):
        assert sid == SID
        assert address in {A_GOAL, A_SPEED, A_ACCEL, A_TORQUE}
        self.writes.append((sid, address, value))
        if address == A_TORQUE:
            self.torque[sid] = value
        else:
            self.reg[address] = value
        return True


def trial(bus, **kwargs):
    clock = Clock()
    return run(bus, sleep=clock.sleep, clock=clock, **kwargs)


def test_read_only_preflight_does_not_write():
    bus = MockBus()
    result = trial(bus)
    assert not bus.writes
    assert not result["motion_command_emitted"]
    assert result["start_raw"] == 1000 and result["target_raw"] == 1032


def test_roundtrip_only_wrist_and_stale_goal_overridden_before_torque():
    bus = MockBus()
    result = trial(bus, execute=True)
    assert result["roundtrip_reached"] and not result["stop_errors"]
    assert result["final_torque"] == [0] * 6
    assert abs(result["final_raw"][SID-1] - 1000) <= 4
    assert bus.writes[0] == (SID, A_GOAL, 1000)
    assert bus.reg[A_SPEED] == 321 and bus.reg[A_ACCEL] == 12
    assert all(sid == SID for sid, _, _ in bus.writes)


@pytest.mark.parametrize("delta", [0, 11, 12, 31, 33, -32, -33, 32.0])
def test_large_or_invalid_delta_rejected(delta):
    bus = MockBus()
    with pytest.raises(ValueError):
        trial(bus, delta=delta, execute=True)
    assert not bus.writes


@pytest.mark.parametrize("address,value", [(OPERATING_MODE, 1), (A_MAX_ANGLE, 1020)])
def test_mode_and_hardware_limits_reject_before_write(address, value):
    bus = MockBus()
    bus.reg[address] = value
    assert trial(bus, execute=True).get("error")
    assert not bus.writes


def test_other_torque_enabled_rejects_before_write():
    bus = MockBus()
    bus.torque[1] = 1
    assert trial(bus, execute=True).get("error")
    assert not bus.writes


def test_stall_releases_without_automatic_return():
    bus = MockBus(stalled=True)
    result = trial(bus, execute=True)
    assert "스톨" in result["error"]
    assert not result["roundtrip_reached"]
    assert bus.torque[SID] == 0
    assert not any(s["phase"] == "RETURN" for s in result["samples"])


def test_dashboard_stop_request_interrupts_and_releases():
    bus = MockBus()
    def monitor():
        if bus.torque[SID] and any(value == 1032 for _, addr, value in bus.writes if addr == A_GOAL):
            raise RuntimeError("대시보드 정지 요청")
    result = trial(bus, execute=True, monitor=monitor)
    assert "대시보드 정지" in result["error"]
    assert not result["roundtrip_reached"]
    assert bus.torque[SID] == 0
    assert not any(s["phase"] == "RETURN" for s in result["samples"])


def test_signal_interrupt_releases():
    bus = MockBus()
    def monitor():
        if bus.torque[SID]:
            raise KeyboardInterrupt()
    result = trial(bus, execute=True, monitor=monitor)
    assert result["error"] == "KeyboardInterrupt"
    assert bus.torque[SID] == 0


class GripperBus(MockBus):
    def __init__(self, stalled=False, phase=12):
        super().__init__(stalled)
        self.reg.update({A_OFFSET: 0, A_PHASE: phase})
        self.limits = {sid: (800, 3200) for sid in range(1, 7)}
        self.limits[5] = (0, 4095)

    def read(self, sid, address, size=1):
        if address == A_POS:
            if sid == 6 and self.torque[sid] and not self.stalled:
                error = self.reg[A_GOAL] - self.pos[sid]
                self.pos[sid] += max(-8, min(8, error))
            return self.pos[sid]
        if address in (A_MIN_ANGLE, A_MAX_ANGLE):
            return self.limits[sid][address == A_MAX_ANGLE]
        return super().read(sid, address, size)

    def write(self, sid, address, value, size=1):
        assert sid == 6
        assert address in {A_GOAL, A_SPEED, A_ACCEL, A_TORQUE}
        self.writes.append((sid, address, value))
        if address == A_TORQUE:
            self.torque[sid] = value
        else:
            self.reg[address] = value
        return True


def gripper_calibration(drive=0):
    return {name: {"id": sid, "drive_mode": drive if sid == 6 else 0,
                   "homing_offset": 0, "range_min": 0 if sid == 5 else 800,
                   "range_max": 4095 if sid == 5 else 3200}
            for sid, name in enumerate(SO101, 1)}


@pytest.mark.parametrize("drive,phase,sign", [(0, 12, 1), (1, 76, -1)])
def test_gripper_partial_cycle_uses_calibrated_open_direction(drive, phase, sign):
    bus = GripperBus(phase=phase)
    result = trial(bus, delta=128, calibration=gripper_calibration(drive),
                   expected_phase=phase, execute=True)
    assert result["partial_cycle_reached"]
    assert not result["roundtrip_reached"]
    assert not result["physical_jaw_motion_verified"]
    assert result["target_raw"] == 1000 + sign * 128
    assert result["partial_close_raw"] == 1000 + sign * 64
    opened = result["samples"]
    assert max(sign * (s["raw"] - 1000) for s in opened if s["phase"] == "OPEN_PROBE") >= 112
    assert result["final_torque"] == [0] * 6
    assert all(sid == 6 for sid, _, _ in bus.writes)
    assert bus.reg[A_SPEED] == 321 and bus.reg[A_ACCEL] == 12
    assert bus.writes[0] == (6, A_GOAL, 1000)


def test_gripper_start_at_closed_end_opens_but_does_not_return_to_end():
    bus = GripperBus()
    bus.pos[6] = 800
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert result["partial_cycle_reached"]
    assert result["partial_close_raw"] == 864
    assert not any(s["phase"] == "PARTIAL_CLOSE" and s["goal"] == 800 for s in result["samples"])


@pytest.mark.parametrize("issue", ["offset", "limits", "phase", "other_torque", "open_limit"])
def test_gripper_bad_preflight_does_not_write(issue):
    bus = GripperBus()
    if issue == "offset":
        bus.reg[A_OFFSET] = 3
    elif issue == "limits":
        bus.limits[1] = (810, 3200)
    elif issue == "phase":
        bus.reg[A_PHASE] = 76
    elif issue == "other_torque":
        bus.torque[1] = 1
    else:
        bus.pos[6] = 3180
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert result.get("error")
    assert not bus.writes


def test_gripper_read_only_preflight():
    bus = GripperBus()
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12)
    assert result["calibration_used"]
    assert result["target_raw"] == 1128
    assert not result["partial_cycle_reached"]
    assert not bus.writes


def test_gripper_stall_releases_and_never_closes():
    bus = GripperBus(stalled=True)
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert "스톨" in result["error"]
    assert not result["partial_cycle_reached"]
    assert bus.torque[6] == 0
    assert not any(s["phase"] == "PARTIAL_CLOSE" for s in result["samples"])


def test_gripper_dashboard_stop_releases_and_never_closes():
    bus = GripperBus()
    def monitor():
        if bus.torque[6]:
            raise RuntimeError("대시보드 정지 요청")
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12,
                   execute=True, monitor=monitor)
    assert "대시보드 정지" in result["error"]
    assert bus.torque[6] == 0
    assert not result["partial_cycle_reached"]


def test_gripper_wrong_delta_or_missing_phase_rejected():
    bus = GripperBus()
    with pytest.raises(ValueError):
        trial(bus, delta=32, calibration=gripper_calibration(), expected_phase=12, execute=True)
    with pytest.raises(ValueError):
        trial(bus, delta=128, calibration=gripper_calibration(), execute=True)
    assert not bus.writes


def test_gripper_cleanup_finishes_with_off_after_goal_and_profile_writes():
    class GoalReenableBus(GripperBus):
        enabled_once = False
        def write(self, sid, address, value, size=1):
            super().write(sid, address, value, size)
            if address == A_TORQUE and value == 1:
                self.enabled_once = True
            if address == A_GOAL and self.enabled_once and self.torque[sid] == 0:
                self.torque[sid] = 1
            return True
    bus = GoalReenableBus()
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert result["partial_cycle_reached"]
    assert result["final_torque"] == [0] * 6
    assert not result["stop_errors"]
    assert bus.writes[-1] == (6, A_TORQUE, 0)


def test_gripper_off_ack_without_effect_retries_and_reports_failure():
    class OffIgnoredBus(GripperBus):
        def write(self, sid, address, value, size=1):
            if address == A_TORQUE and value == 0:
                self.writes.append((sid, address, value))
                return True
            return super().write(sid, address, value, size)
    bus = OffIgnoredBus()
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert result["stop_errors"]
    assert result["final_torque"][-1] == 1
    assert len(result["release_attempts"]) == 3


def test_gripper_first_final_off_ignored_is_recovered_and_recorded():
    class FirstFinalOffIgnoredBus(GripperBus):
        count = 0
        def write(self, sid, address, value, size=1):
            if address == A_TORQUE and value == 0:
                self.count += 1
                if self.count == 2:
                    self.writes.append((sid, address, value))
                    self.torque[sid] = 1
                    return True
            return super().write(sid, address, value, size)
    bus = FirstFinalOffIgnoredBus()
    result = trial(bus, delta=128, calibration=gripper_calibration(), expected_phase=12, execute=True)
    assert result["final_torque"] == [0] * 6
    assert result["release_attempts"] == [{"torque": 1}, {"torque": 0}]
    assert not result["stop_errors"]
