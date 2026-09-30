import copy

import pytest

from servo import servo_apply_ranges as apply
from servo.sts_bus import encode_offset


class RangeBus:
    def __init__(self):
        self.reg = {sid: {addr: 0 for addr, _ in apply.FIELDS.values()}
                    for sid in range(1, 7)}
        for sid, row in self.reg.items():
            row.update({apply.A_MIN_ANGLE: 800, apply.A_MAX_ANGLE: 3200,
                        apply.A_POS: 2000, apply.A_OFFSET: encode_offset(-123),
                        apply.A_PHASE: 76 if sid == 6 else 12,
                        apply.A_P: 16, apply.A_D: 32, apply.A_I: 0,
                        apply.A_LOCK: 1, apply.A_TORQUE: 0})
        self.writes = []
        self.fail_once = None

    def read(self, sid, addr, n=1):
        return self.reg[sid][addr]

    def write(self, sid, addr, value, n=1):
        self.writes.append((sid, addr, value))
        self.reg[sid][addr] = value
        if self.fail_once == (sid, addr):
            self.fail_once = None
            return False  # 실제 쓰기는 도착했으나 ACK만 유실.
        return True


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setattr(apply.time, "sleep", lambda _: None)
    bus = RangeBus()
    expected = {name: {"id": sid, "homing_offset": -123, "drive_mode": 0,
                       "range_min": 800, "range_max": 3200}
                for sid, name in enumerate(apply.SO101, 1)}
    candidate = copy.deepcopy(expected)
    for c in candidate.values():
        c.update(range_min=850, range_max=3150)
    candidate["wrist_roll"].update(range_min=0, range_max=4095)
    candidate["gripper"]["drive_mode"] = 1
    return bus, candidate, expected, [12] * 5 + [76]


def test_apply_only_limits_preserves_offset_phase_and_gripper_flag(rig):
    bus, candidate, expected, phases = rig
    before = copy.deepcopy(bus.reg)
    logs = []
    result = apply.apply_ranges(bus, candidate, expected, phases, logs.append)
    assert result["status"] == "applied_pending_power_cycle"
    assert logs[0]["status"] == "prepared"
    assert result["candidate"]["gripper"]["drive_mode"] == 1
    assert {addr for _, addr, _ in bus.writes} == {
        apply.A_MIN_ANGLE, apply.A_MAX_ANGLE, apply.A_LOCK}
    for sid, row in bus.reg.items():
        for key in ("offset_raw", "phase", "p", "d", "i", "lock", "torque", "pos"):
            addr = apply.FIELDS[key][0]
            assert row[addr] == before[sid][addr]


@pytest.mark.parametrize("key,value", [("phase", 12), ("offset_raw", 0),
                                     ("min", 700), ("max", 3300),
                                     ("torque", 1), ("lock", 0), ("pos", 840),
                                     ("pos", None)])
def test_bad_live_state_never_writes(rig, key, value):
    bus, candidate, expected, phases = rig
    bus.reg[6][apply.FIELDS[key][0]] = value
    with pytest.raises((RuntimeError, ValueError)):
        apply.apply_ranges(bus, candidate, expected, phases, lambda _: None)
    assert not bus.writes


def test_candidate_offset_change_never_writes(rig):
    bus, candidate, expected, phases = rig
    candidate["gripper"]["homing_offset"] = 0
    with pytest.raises(ValueError):
        apply.apply_ranges(bus, candidate, expected, phases, lambda _: None)
    assert not bus.writes


def test_backup_failure_never_writes(rig):
    bus, candidate, expected, phases = rig
    def failed_log(_):
        raise OSError("disk full")
    with pytest.raises(OSError):
        apply.apply_ranges(bus, candidate, expected, phases, failed_log)
    assert not bus.writes


def test_ambiguous_write_ack_failure_rolls_back_every_touched_limit(rig):
    bus, candidate, expected, phases = rig
    before = copy.deepcopy(bus.reg)
    bus.fail_once = (3, apply.A_MAX_ANGLE)
    result = apply.apply_ranges(bus, candidate, expected, phases, lambda _: None)
    assert result["status"] == "failed"
    assert result["rollback_verified"]
    assert bus.reg == before


def test_success_log_failure_rolls_back(rig):
    bus, candidate, expected, phases = rig
    before = copy.deepcopy(bus.reg)
    def failed_log(value):
        if value["status"] == "applied_pending_power_cycle":
            raise OSError("disk full")
    result = apply.apply_ranges(bus, candidate, expected, phases, failed_log)
    assert result["status"] == "failed"
    assert result["rollback_verified"]
    assert bus.reg == before


def test_cancellation_rolls_back(rig):
    bus, candidate, expected, phases = rig
    before = copy.deepcopy(bus.reg)
    result = apply.apply_ranges(bus, candidate, expected, phases, lambda _: None,
                                lambda: len(bus.writes) >= 6)
    assert result["status"] == "failed"
    assert result["rollback_verified"]
    assert bus.reg == before


def test_rollback_failure_is_reported_and_does_not_skip_other_ids(rig, monkeypatch):
    bus, candidate, expected, phases = rig
    original = apply.write_limit
    def fail_restore(bus, sid, addr, value):
        if sid == 3:
            raise SystemExit("disconnected motor")
        return original(bus, sid, addr, value)
    monkeypatch.setattr(apply, "write_limit", fail_restore)
    result = apply.apply_ranges(bus, candidate, expected, phases, lambda _: None)
    assert result["status"] == "failed"
    assert not result["rollback_verified"]
    assert result["rollback_errors"]
    assert bus.reg[1][apply.A_MIN_ANGLE] == expected["shoulder_pan"]["range_min"]
    assert bus.reg[2][apply.A_MAX_ANGLE] == expected["shoulder_lift"]["range_max"]


def test_failed_final_readback_rolls_back(rig, monkeypatch):
    bus, candidate, expected, phases = rig
    before = copy.deepcopy(bus.reg)
    def failed_verify(*_):
        raise RuntimeError("final mismatch")
    monkeypatch.setattr(apply, "verify_after", failed_verify)
    result = apply.apply_ranges(bus, candidate, expected, phases, lambda _: None)
    assert result["status"] == "failed"
    assert result["rollback_verified"]
    assert bus.reg == before
