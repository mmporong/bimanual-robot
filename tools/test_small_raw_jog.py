import pytest

from servo.small_raw_jog import run, SID, OPERATING_MODE
from servo.sts_bus import A_ACCEL, A_GOAL, A_MAX_ANGLE, A_MIN_ANGLE, A_POS, A_SPEED, A_TORQUE


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
