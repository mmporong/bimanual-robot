import copy
import json
import sys
from pathlib import Path

import pytest

from servo import ik_reset_clear_step as step
from servo.sts_bus import (A_ACCEL, A_GOAL, A_LOCK, A_MAX_ANGLE, A_MIN_ANGLE, A_OFFSET,
                           A_P, A_POS, A_SPEED, A_TORQUE, encode_offset)


# 2026-09-29 저장 raw의 회귀 자료다. 최신 보정과 과거 raw를 섞지 않는다.
# 원본: e00092f의 calibration/bi_follower/arms_left.json. 실물 실행에 사용하지 않는다.
CALIBRATION_BYTES = (Path(__file__).parent / "fixtures/so101_left_20260929.json").read_bytes()
CALIBRATION = json.loads(CALIBRATION_BYTES)


def test_execution_record_failure_prevents_port_open(tmp_path, monkeypatch):
    source, calibration, review, output = [tmp_path / name for name in
                                          ("snapshot.json", "cal.json", "review.json", "trial.json")]
    source.write_text(json.dumps(snapshot()))
    calibration.write_bytes(CALIBRATION_BYTES)
    review.write_text("{}")
    monkeypatch.setattr(sys, "argv", ["clear", "--snapshot-file", str(source),
        "--calibration", str(calibration), "--output", str(output), "--execute",
        "--port", "/dev/mock", "--stop-status-url", "http://127.0.0.1:8770/api/stop/status",
        "--model-review", str(review)])
    monkeypatch.setattr(step, "open_stop_bus", lambda _: pytest.fail("기록 실패 시 포트 미접근"))
    def failed_sync(_fd):
        raise OSError("record unavailable")
    monkeypatch.setattr(step.os, "fsync", failed_sync)
    with pytest.raises(OSError, match="record unavailable"):
        step.main()
    pending = json.loads(output.read_text())
    assert pending["status"] == "execution_pending"
    assert pending["motion_command_emitted"] is None


def snapshot():
    raw = [1991, 780, 2516, 2775, 982, 1415]
    return {
        "raw_ticks": raw,
        "torque": [0] * 6,
        "joint_deg": step._snapshot_joint_deg(raw[:5], CALIBRATION),
        "gripper_percent": 5.02,
        "hardware_accessed": True,
        "motion_command_emitted": False,
        "calibration_sha256": step._sha(CALIBRATION_BYTES),
    }


def review_for(plan):
    samples = step.expected_review_samples(plan, CALIBRATION)
    for sample in samples:
        sample["tcp_z_relative_m"] -= 0.01981537634286812
        sample["finger_bottom_relative_m"] -= 0.029149414062500045
        sample["self_collision_pairs"] = [["a", "b"]]
    return {
        "schema": "reset_clear_model_review_v1",
        "start_raw": plan["start_raw"],
        "end_raw": plan["poses"][0]["raw_ticks"],
        "calibration_sha256": plan["calibration_sha256"],
        "urdf_sha256": plan["urdf_sha256"],
        "accepted": True,
        "max_joint_step_deg": 0.5,
        "maximum_arm_x_relative_m": 0.2,
        "cup_near_face_x_relative_m": 0.31,
        "physical_geometry_verified": False,
        "continuous_collision_validated": False,
        "initial_contact_escape_completed": False,
        "samples": samples,
    }


def test_candidate_changes_only_expected_id3_within_bound():
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    target = plan["poses"][0]["raw_ticks"]
    delta = [b - a for a, b in zip(plan["start_raw"], target)]
    assert plan["servo_ids"] == [1, 2, 3, 4, 5, 6]
    assert delta[2] < 0 and abs(delta[2]) <= 35
    assert delta[:2] + delta[3:] == [0] * 5
    assert plan["poses"] == [{"phase": "CLEAR_RESET_STEP", "raw_ticks": target}]
    assert not plan["physical_cup_grasp_verified"]


def test_model_review_tamper_and_new_collision_are_rejected():
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    review = review_for(plan)
    step.validate_model_review(plan, review, CALIBRATION_BYTES)
    bad = copy.deepcopy(review)
    bad["samples"][-1]["tcp_z_relative_m"] += 0.001
    with pytest.raises(ValueError, match="FK 상승량"):
        step.validate_model_review(plan, bad, CALIBRATION_BYTES)
    bad = copy.deepcopy(review)
    bad["samples"][-1]["self_collision_pairs"].append(["new", "pair"])
    with pytest.raises(ValueError, match="새로운"):
        step.validate_model_review(plan, bad, CALIBRATION_BYTES)


class HardwareBus:
    def __init__(self, plan):
        self.positions = dict(zip(plan["servo_ids"], plan["start_raw"]))
        self.reg = {}
        self.writes = []
        for name in step.NAMES:
            item, sid = CALIBRATION[name], CALIBRATION[name]["id"]
            self.reg.update({
                (sid, A_POS): self.positions[sid], (sid, A_GOAL): self.positions[sid],
                (sid, A_TORQUE): 0, (sid, A_SPEED): 300, (sid, A_ACCEL): 10,
                (sid, A_OFFSET): encode_offset(item["homing_offset"]),
                (sid, A_MIN_ANGLE): item["range_min"], (sid, A_MAX_ANGLE): item["range_max"],
                (sid, A_P): 16, (sid, A_LOCK): 1,
            })

    def read(self, sid, address, size=1):
        if address == A_POS and self.reg[sid, A_TORQUE]:
            goal = self.reg[sid, A_GOAL]
            self.positions[sid] += max(-8, min(8, goal - self.positions[sid]))
        return self.positions[sid] if address == A_POS else self.reg.get((sid, address))

    def write(self, sid, address, value, size=1):
        self.writes.append((sid, address, value))
        if address == A_P and self.reg[sid, A_LOCK] != 0:
            return True
        self.reg[sid, address] = value
        return True


def test_hardware_mismatch_prevents_any_write():
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    bus = HardwareBus(plan)
    bus.reg[3, A_MAX_ANGLE] -= 1
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None)
    assert "calibration/EEPROM" in result["failure"]
    assert not bus.writes
    assert not result["command_write_attempted"]


def test_mock_limited_run_restores_profiles_and_all_torque_off():
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    bus = HardwareBus(plan)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None)
    assert result["sequence_completed"]
    assert result["phases_completed"] == ["CLEAR_RESET_STEP"]
    assert result["final_torque"] == [0] * 6
    assert all(bus.reg[sid, A_SPEED] == 300 and bus.reg[sid, A_ACCEL] == 10 for sid in plan["servo_ids"])
    assert not result["temperature_load_read"] and not result["physical_cup_grasp_verified"]
    assert result["raw_step_ticks"] == 35
    assert result["arrival_tolerance_ticks"] == 4
    assert result["max_position_error_ticks"] <= 4
    targets = {value for sid, addr, value in bus.writes if sid == 3 and addr == A_GOAL}
    assert targets <= {plan["start_raw"][2], plan["poses"][0]["raw_ticks"][2], bus.positions[3]}
    assert not any(address in (A_P, A_LOCK) for _sid, address, _value in bus.writes)


def test_stationary_motor_is_not_accepted_and_last_error_is_recorded(monkeypatch):
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    bus = HardwareBus(plan)
    original_read = bus.read
    def stationary_read(sid, address, size=1):
        if address == A_POS:
            return bus.positions[sid]
        return original_read(sid, address, size)
    bus.read = stationary_read
    now = [0.0]
    def sleep(seconds):
        now[0] += seconds
    original_run = step.ik_pick_place.run_sequence
    def timed_run(*args, **kwargs):
        return original_run(*args, sleep=sleep, clock=lambda: now[0], **kwargs)
    monkeypatch.setattr(step.ik_pick_place, "run_sequence", timed_run)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None)
    assert "position_stall" in result["failure"]
    assert result["last_goal_raw"] == plan["poses"][0]["raw_ticks"]
    assert result["last_observed_raw"] == plan["start_raw"]
    assert result["max_position_error_ticks"] == 34
    assert not result["phases_completed"]
    assert result["final_torque"] == [0] * 6


@pytest.mark.parametrize("kwargs", [
    {"raw_step_ticks": 36}, {"raw_step_ticks": True},
    {"arrival_tolerance_ticks": 0}, {"arrival_tolerance_ticks": 16},
])
def test_bad_execution_settings_prevent_writes(kwargs):
    plan = step.prepare(snapshot(), CALIBRATION_BYTES)
    bus = HardwareBus(plan)
    with pytest.raises(ValueError):
        step.ik_pick_place.run_sequence(bus, plan, lambda _: True, **kwargs)
    assert not bus.writes


def test_default_cli_writes_candidate_without_opening_port(tmp_path, monkeypatch):
    snapshot_path, calibration_path, output_path = (
        tmp_path / "snapshot.json", tmp_path / "calibration.json", tmp_path / "candidate.json")
    snapshot_path.write_text(json.dumps(snapshot()))
    calibration_path.write_bytes(CALIBRATION_BYTES)
    monkeypatch.setattr(step, "open_stop_bus", lambda _port: pytest.fail("기본 후보 생성이 포트를 열었습니다"))
    monkeypatch.setattr(sys, "argv", ["ik_reset_clear_step", "--snapshot-file", str(snapshot_path),
        "--calibration", str(calibration_path), "--output", str(output_path)])
    assert step.main() == 0
    candidate = json.loads(output_path.read_text())
    assert candidate["hardware_accessed"] is False
    assert candidate["motion_command_emitted"] is False


def reserved_journal(tmp_path):
    path = tmp_path / "trial.gain.jsonl"
    step.reserve_gain_journal(path)
    return path


def test_p32_success_restores_original_gain_and_lock(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    gain = result["gain_test"]
    assert result["sequence_completed"]
    assert (gain["original_p"], gain["observed_test_p"]) == (16, 32)
    assert (gain["restored_p"], gain["restored_lock"]) == (16, 1)
    assert gain["errors"] == []
    assert bus.reg[3, A_P] == 16 and bus.reg[3, A_LOCK] == 1


def test_applied_p_write_with_failed_ack_still_restores(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    original_write, failed = bus.write, [False]
    def write(sid, address, value, size=1):
        applied = original_write(sid, address, value, size)
        if sid == 3 and address == A_P and value == 32 and not failed[0]:
            failed[0] = True
            return False
        return applied
    bus.write = write
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    assert not result["sequence_completed"]
    assert "ACK 실패" in result["failure"]
    assert bus.reg[3, A_P] == 16 and bus.reg[3, A_LOCK] == 1
    assert result["gain_test"]["observed_test_p"] == 32
    assert result["gain_test"]["restored_p"] == 16


def test_motion_failure_still_restores_gain(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(step.ik_pick_place, "run_sequence", lambda *_args, **_kwargs: {
        "sequence_completed": False, "failure": "simulated_end_failure", "stop_errors": [],
        "command_write_attempted": True, "phases_completed": [], "temperature_load_read": False})
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    assert not result["sequence_completed"] and result["failure"] == "simulated_end_failure"
    assert (bus.reg[3, A_P], bus.reg[3, A_LOCK]) == (16, 1)
    assert result["gain_test"]["final_torque"] == [0] * 6


def test_gain_journal_reservation_failure_does_not_open_port(tmp_path, monkeypatch):
    source, calibration, review, output = [tmp_path / name for name in
        ("snapshot.json", "cal.json", "review.json", "trial.json")]
    source.write_text(json.dumps(snapshot()))
    calibration.write_bytes(CALIBRATION_BYTES)
    review.write_text(json.dumps(review_for(step.prepare(snapshot(), CALIBRATION_BYTES))))
    monkeypatch.setattr(sys, "argv", ["clear", "--snapshot-file", str(source),
        "--calibration", str(calibration), "--output", str(output), "--execute",
        "--port", "/dev/mock", "--stop-status-url", "http://127.0.0.1:8770/api/stop/status",
        "--model-review", str(review), "--elbow-p-test", "32"])
    monkeypatch.setattr(step, "reserve_gain_journal", lambda _path: (_ for _ in ()).throw(OSError("journal unavailable")))
    monkeypatch.setattr(step, "open_stop_bus", lambda _port: pytest.fail("저널 실패 뒤 포트가 열렸습니다"))
    with pytest.raises(OSError, match="journal unavailable"):
        step.main()


def test_gain_restore_readback_failure_marks_sequence_failed(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    original_write, p_writes = bus.write, [0]
    def write(sid, address, value, size=1):
        if sid == 3 and address == A_P:
            p_writes[0] += 1
            if p_writes[0] == 2:  # P32는 적용하고 finally의 P16 복구만 무시한다.
                bus.writes.append((sid, address, value))
                return True
        return original_write(sid, address, value, size)
    bus.write = write
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    assert not result["sequence_completed"]
    assert "복구 readback 불일치" in " ".join(result["gain_test"]["errors"])
    assert result["gain_test"]["restored_p"] == 32


def test_journal_failure_after_p32_does_not_skip_restore(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus, journal = HardwareBus(plan), reserved_journal(tmp_path)
    original_journal, calls = step._journal, [0]
    def journal_write(path, event, mode="a"):
        calls[0] += 1
        if calls[0] == 2:  # backup은 기록됐고 P32 관측 기록에서 저장 장치가 실패한다.
            raise OSError("journal lost")
        return original_journal(path, event, mode)
    monkeypatch.setattr(step, "_journal", journal_write)
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=journal)
    assert not result["sequence_completed"] and "journal lost" in result["failure"]
    assert (bus.reg[3, A_P], bus.reg[3, A_LOCK]) == (16, 1)
    assert (result["gain_test"]["restored_p"], result["gain_test"]["restored_lock"]) == (16, 1)


def test_unexpected_motion_exception_still_restores_gain(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    def interrupted(*_args, **_kwargs):
        raise KeyboardInterrupt("simulated interrupt")
    monkeypatch.setattr(step.ik_pick_place, "run_sequence", interrupted)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    assert not result["sequence_completed"] and "KeyboardInterrupt" in result["failure"]
    assert (bus.reg[3, A_P], bus.reg[3, A_LOCK]) == (16, 1)
    assert not result["gain_test"]["restore_pending"]


def test_unproven_all_torque_off_defers_p_restore_but_relocks(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    original_write = bus.write
    def write(sid, address, value, size=1):
        if sid == 1 and address == A_TORQUE and value == 0:
            bus.writes.append((sid, address, value))
            return True  # ACK만 하고 ID1 토크를 끄지 않는 종료 실패를 모사한다.
        return original_write(sid, address, value, size)
    bus.write = write
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    def failed_end(*_args, **_kwargs):
        bus.reg[1, A_TORQUE] = 1
        return {"sequence_completed": False, "failure": "end failure", "stop_errors": [],
                "command_write_attempted": True, "phases_completed": [], "temperature_load_read": False}
    monkeypatch.setattr(step.ik_pick_place, "run_sequence", failed_end)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    gain = result["gain_test"]
    assert not result["sequence_completed"] and gain["restore_pending"]
    assert bus.reg[3, A_P] == 32 and bus.reg[3, A_LOCK] == 1
    assert not any(sid == 3 and address == A_P and value == 16 for sid, address, value in bus.writes)
    assert "토크 OFF 미증명" in " ".join(gain["errors"])


def test_unexpected_original_p_value_causes_no_servo_write(tmp_path, monkeypatch):
    plan, bus = step.prepare(snapshot(), CALIBRATION_BYTES), None
    bus = HardwareBus(plan)
    bus.reg[3, A_P] = 17
    monkeypatch.setattr(step.time, "sleep", lambda _seconds: None)
    result = step.execute(bus, plan, CALIBRATION_BYTES, lambda: None,
                          elbow_p_test=32, gain_journal=reserved_journal(tmp_path))
    assert not result["sequence_completed"] and "P=16" in result["failure"]
    assert bus.writes == []
    assert bus.reg[3, A_P] == 17 and bus.reg[3, A_LOCK] == 1
