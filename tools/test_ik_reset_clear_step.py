import copy
import json
import sys

import pytest

from servo import ik_reset_clear_step as step
from servo.sts_bus import A_ACCEL, A_GOAL, A_MAX_ANGLE, A_MIN_ANGLE, A_OFFSET, A_POS, A_SPEED, A_TORQUE, encode_offset


CALIBRATION_BYTES = step.ik_pick_place.DEFAULT_CALIBRATION.read_bytes()
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
            })

    def read(self, sid, address, size=1):
        if address == A_POS and self.reg[sid, A_TORQUE]:
            goal = self.reg[sid, A_GOAL]
            self.positions[sid] += max(-8, min(8, goal - self.positions[sid]))
        return self.positions[sid] if address == A_POS else self.reg.get((sid, address))

    def write(self, sid, address, value, size=1):
        self.writes.append((sid, address, value))
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
