import copy
import hashlib
import json
import signal
import subprocess
import sys
from pathlib import Path

import pytest

import ik_pick_place as p
from servo.joint_reference import build_reference


def fixture_config():
    # 모의 작업셀이다. 실측값이나 실행 가능한 실물 설정으로 사용하지 않는다.
    return {"schema": "fixed_ik_pick_place_v1", "frame": "base_footprint",
        "provenance": {"status": "simulation_fixture", "measured_at": "2026-09-29", "valid_for": "offline test only"},
        "cup_center_m": [.39, .17, .78], "place_center_m": [.40, .17, .78],
        "table_surface_z_m": .72, "cup_height_m": .12, "cup_radius_m": .035,
        "contact_center_tool_m": [-.035, 0, -.02], "start_joint_deg": [0, -55, 70, -15, 90],
        "right_parked_deg": [0, -55, 70, -15, 90], "right_gripper_m": .0325,
        "right_gripper_percent": 50,
        "start_gripper_percent": 100, "open_percent": 100, "close_percent": 50,
        "gripper_open_rad": 1.3, "gripper_close_rad": 0,
        "lift_distance_m": .06, "pregrasp_backoff_m": .06, "pregrasp_height_m": .08,
        "reorient_backoff_m": .1, "reorient_height_m": .13,
        "lowering_offset_m": .002,
        "withdraw_distance_m": .065}


def reference_fixture(calibration_bytes):
    cal = json.loads(calibration_bytes)
    raw = [int(round((cal[n]["range_min"] + cal[n]["range_max"]) / 2)) + 20 for n in p.NAMES]
    return build_reference(calibration_bytes, raw, [0]*5,
        hashlib.sha256(p.URDF_PATH.read_bytes()).hexdigest(),
        {"status": "user_aligned_reference", "observed_at": "2026-09-29T12:00:00+09:00",
         "description": "synthetic test fixture, not a physical alignment"})


def test_reference_changes_raw_but_preserves_ik_and_simulation_angles():
    from simulate_fixed_ik_pick_place import simulation_plan
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    config = fixture_config()
    baseline = p.prepare(config, calibration_bytes)
    config["joint_reference"] = reference_fixture(calibration_bytes)
    packet = p.prepare(config, calibration_bytes)
    assert packet["start_raw"][:5] != baseline["start_raw"][:5]
    assert p.verify_packet(packet, calibration_bytes)
    simulated = simulation_plan(packet, calibration_bytes)
    poses = [pose for pose in simulated["poses"] if pose["name"] in p.PHASES]
    for pose, source in zip(poses, packet["poses"]):
        assert pose["joint_deg"] == p.decode_raw(source["raw_ticks"],
            json.loads(calibration_bytes), config, calibration_bytes)[0]
        assert max(abs(a-b) for a, b in zip(pose["joint_deg"], source["joint_deg"])) < .05
    changed = copy.deepcopy(packet)
    changed["config"]["joint_reference"]["zero_raw"][0] += 1
    with pytest.raises(ValueError):
        p.verify_packet(changed, calibration_bytes)


def test_snapshot_uses_reference_without_writing_hardware():
    from servo.sts_bus import A_OFFSET, encode_offset
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    cal = json.loads(calibration_bytes)
    reference = reference_fixture(calibration_bytes)
    raw = reference["reference_raw"] + [cal["gripper"]["range_min"]]
    bus = p.MockBus(list(range(1, 7)), raw)
    for name in p.NAMES:
        bus.reg[cal[name]["id"], A_OFFSET] = encode_offset(cal[name]["homing_offset"])
    observed = p.snapshot(bus, calibration_bytes, reference)
    assert observed["joint_deg"] == [0]*5
    assert observed["joint_reference"] == reference
    assert not bus.writes


def test_reference_limits_intersect_urdf_without_changing_other_chain():
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    reference = reference_fixture(calibration_bytes)
    chain, untouched = p.FixedContactChain(fixture_config()), p.FixedContactChain(fixture_config())
    original = dict(chain.limits)
    p.apply_joint_reference_limits(chain, calibration_bytes, reference)
    for name, hardware in zip(p.JOINTS, p.joint_limits_deg(calibration_bytes, reference)):
        key = f"left_{name}"
        low, high = p.np.radians(hardware)
        assert chain.limits[key] == (max(original[key][0], low), min(original[key][1], high))
    assert untouched.limits == original
    p.apply_joint_reference_limits(untouched, calibration_bytes, None)
    assert untouched.limits == original


@pytest.fixture(scope="module")
def plan():
    return p.prepare(fixture_config(), p.DEFAULT_CALIBRATION.read_bytes())


def test_complete_sequence_has_move_and_release(plan):
    phases = list(dict.fromkeys(pose["phase"] for pose in plan["poses"]))
    assert phases == p.PHASES
    end = {pose["phase"]: pose for pose in plan["poses"]}
    assert end["TRANSFER"]["target_m"][0] == .40
    assert end["LOWER"]["target_m"] == [.40, .17, .778]
    assert end["CLOSE"]["gripper_percent"] == 50
    assert end["OPEN"]["gripper_percent"] == 100
    assert p.verify_packet(plan, p.DEFAULT_CALIBRATION.read_bytes())


@pytest.mark.parametrize("key,value", [("table_surface_z_m", None), ("cup_height_m", 0),
    ("open_percent", 101), ("close_percent", True), ("start_joint_deg", [0]*4),
    ("contact_center_tool_m", [0, 0, float("nan")]), ("right_gripper_m", 0),
    ("place_center_m", [.4, .17, .77]), ("frame", "camera"), ("provenance", {})])
def test_missing_or_invalid_inputs_rejected(key, value):
    config = fixture_config()
    config[key] = value
    with pytest.raises(ValueError):
        p.validate_config(config)


def test_lowering_allowance_is_not_an_unbounded_table_penetration():
    config = fixture_config()
    config["lowering_offset_m"] = .004
    with pytest.raises(ValueError, match="하강 여유"):
        p.validate_config(config)


@pytest.mark.parametrize("key,value", [("position_tolerance_mm", 0), ("axes_tolerance_deg", float("nan"))])
def test_solver_rejects_invalid_convergence_tolerances(key, value):
    config = fixture_config()
    with pytest.raises(ValueError, match="허용오차"):
        p.solve_horizontal_endpoint(p.FixedContactChain(config), "left", config["cup_center_m"], **{key: value})


def test_gripper_percentage_raw_and_drive_mode():
    cal_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    cal = json.loads(cal_bytes)
    assert p.raw_for([0]*5, 100, cal_bytes)[5] == cal["gripper"]["range_max"]
    assert p.raw_for([0]*5, 0, cal_bytes)[5] == cal["gripper"]["range_min"]
    cal["gripper"]["drive_mode"] = 1
    assert p.raw_for([0]*5, 100, json.dumps(cal).encode())[5] == cal["gripper"]["range_min"]
    cal["gripper"]["id"] = 1
    with pytest.raises(ValueError, match="중복"):
        p.raw_for([0]*5, 100, json.dumps(cal).encode())


def test_gripper_quantized_endpoints_do_not_extrapolate_negative_rad():
    config = fixture_config()
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    cal = json.loads(calibration_bytes)
    for percent, angle in ((50, 0), (100, 1.3)):
        raw = p.raw_for([0]*5, percent, calibration_bytes)
        assert p.decode_raw(raw, cal, config)[1] == angle


def test_committed_simulation_input_matches_test_fixture():
    path = p.DEFAULT_CALIBRATION.parents[2] / "config/ik/fixed_cup_pick_place_simulation.json"
    config = json.loads(path.read_text())
    config["provenance"] = fixture_config()["provenance"]
    assert config == fixture_config()


@pytest.mark.parametrize("key", ["calibration_sha256", "urdf_sha256", "config_sha256"])
def test_changed_binding_rejected(plan, key):
    bad = copy.deepcopy(plan)
    bad[key] = "changed"
    with pytest.raises(ValueError, match="변경"):
        p.verify_packet(bad, p.DEFAULT_CALIBRATION.read_bytes())


def test_tampered_stage_and_raw_rejected(plan):
    bad = copy.deepcopy(plan)
    bad["poses"].pop()
    with pytest.raises(ValueError):
        p.verify_packet(bad, p.DEFAULT_CALIBRATION.read_bytes())
    bad = copy.deepcopy(plan)
    bad["poses"][0]["raw_ticks"][5] = 0
    with pytest.raises(ValueError, match="raw"):
        p.verify_packet(bad, p.DEFAULT_CALIBRATION.read_bytes())


class Clock:
    value = 0
    def now(self):
        return self.value
    def sleep(self, seconds):
        self.value += seconds


def run(bus, plan, confirm=lambda _: True):
    clock = Clock()
    return p.run_sequence(bus, plan, confirm, sleep=clock.sleep, clock=clock.now)


def assert_restored(bus, plan):
    for sid in plan["servo_ids"]:
        assert bus.reg[sid, p.A_TORQUE] == 0
        assert bus.reg[sid, p.A_SPEED] == 300
        assert bus.reg[sid, p.A_ACCEL] == 10
        assert bus.reg[sid, p.A_GOAL] == bus.positions[sid]
    assert all(address in {p.A_GOAL, p.A_SPEED, p.A_ACCEL, p.A_TORQUE} for _, address, _ in bus.writes)


def test_full_mock_no_physical_success_or_eeprom(plan):
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    result = run(bus, plan)
    assert result["sequence_completed"]
    assert result["phases_completed"] == p.PHASES
    assert not result["physical_task_verified"] and not result["physical_grasp_verified"]
    assert not result["temperature_load_read"]
    assert result["plan_sha256"] == p.digest(plan)
    assert_restored(bus, plan)


def test_unverified_reference_still_allows_offline_mock():
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    config = fixture_config()
    config["joint_reference"] = reference_fixture(calibration_bytes)
    packet = p.prepare(config, calibration_bytes)
    bus = p.MockBus(packet["servo_ids"], packet["start_raw"])
    result = run(bus, packet)
    assert result["sequence_completed"]
    assert not result["physical_task_verified"]
    assert_restored(bus, packet)


def test_simulation_uses_same_quantized_goal_sequence_and_required_holds(plan):
    from simulate_fixed_ik_pick_place import simulation_plan
    result = simulation_plan(plan, p.DEFAULT_CALIBRATION.read_bytes())
    poses = [pose for pose in result["poses"] if pose["name"] in p.PHASES]
    assert len(poses) == len(plan["poses"])
    for simulated, source in zip(poses, plan["poses"]):
        assert simulated["name"] == source["phase"]
        assert simulated["joint_deg"] == p.decode_raw(source["raw_ticks"],
            json.loads(p.DEFAULT_CALIBRATION.read_bytes()), plan["config"])[0]
    assert {"CONTACT_HOLD", "LIFT_HOLD", "TABLE_SETTLE", "RELEASE_HOLD", "PLACE_HOLD"} <= {
        pose["name"] for pose in result["poses"]}
    assert result["command_source"] == "quantized_raw_targets"


def test_simulation_adapter_contract_reaches_existing_runner(plan, tmp_path, monkeypatch):
    import simulate_fixed_ik_pick_place as sim
    packet = copy.deepcopy(plan)
    packet["packet_sha256"] = p.digest(packet)
    source = tmp_path / "plan.json"
    source.write_text(json.dumps(packet))
    captured = []
    def inspect(options, backend):
        assert backend.CONTACT_FRAME == "left_contact_center"
        assert backend.GRIPPER_KEY == "gripper_rad"
        assert "ik_pick_place.py" in backend.EXTRA_TOOL_FILES
        config = backend.load_config(source)
        assert config["placement_center_m"] == plan["config"]["place_center_m"]
        converted = backend.make_plan(None, config, place=True)
        assert converted["right_gripper_m"] == plan["config"]["right_gripper_m"]
        assert options.place and not options.recover and not options.record
        captured.append(True)
    monkeypatch.setattr(sim, "run", inspect)
    monkeypatch.setattr(sys, "argv", ["sim", "--plan", str(source), "--output-dir", str(tmp_path / "sim"), "--headless"])
    sim.main()
    assert captured == [True]


@pytest.mark.parametrize("phase", p.PHASES)
def test_rejected_phase_stops_without_advancing(plan, phase):
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    result = run(bus, plan, lambda name: name != phase)
    assert not result["sequence_completed"]
    assert phase not in result["phases_completed"]
    assert "observation_rejected" in result["failure"]
    assert result["failure_phase"] == phase
    assert_restored(bus, plan)


def test_start_mismatch_does_not_write(plan):
    raw = plan["start_raw"].copy()
    raw[0] += 100
    bus = p.MockBus(plan["servo_ids"], raw)
    result = run(bus, plan)
    assert "start_state_changed" in result["failure"]
    assert not bus.writes
    assert not result["command_write_attempted"]


def test_opposite_arm_drift_stops_before_first_command(plan):
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    def changed():
        raise RuntimeError("right_parked_state_changed")
    clock = Clock()
    result = p.run_sequence(bus, plan, lambda _: True, sleep=clock.sleep, clock=clock.now, monitor=changed)
    assert "right_parked_state_changed" in result["failure"]
    assert not bus.writes


def test_offset_preflight_failure_does_not_enable_torque(plan):
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    def bad_offset():
        raise ValueError("offset mismatch")
    result = p.run_sequence(bus, plan, lambda _: True, preflight=bad_offset)
    assert "offset mismatch" in result["failure"] and not bus.writes


def test_snapshot_only_reads_and_detects_wrong_offset(plan):
    from servo.sts_bus import encode_offset
    cal_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    cal = json.loads(cal_bytes)
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    for name in p.NAMES:
        bus.reg[cal[name]["id"], p.A_OFFSET] = encode_offset(cal[name]["homing_offset"])
    state = p.snapshot(bus, cal_bytes)
    assert state["raw_ticks"] == plan["start_raw"]
    assert not state["motion_command_emitted"] and not bus.writes
    bus.reg[1, p.A_OFFSET] = 0
    with pytest.raises(ValueError, match="offset"):
        p.snapshot(bus, cal_bytes)
    assert not bus.writes


def test_readback_failure_and_temperature_load_are_never_read(plan):
    class Misreported(p.MockBus):
        def read(self, sid, address, size=1):
            assert address not in (60, 63)
            value = super().read(sid, address, size)
            return value+1 if address == p.A_GOAL else value
    bus = Misreported(plan["servo_ids"], plan["start_raw"])
    result = run(bus, plan)
    assert "goal_readback_failed" in result["failure"]
    assert_restored(bus, plan)


def test_failed_torque_off_is_reported(plan):
    class Ignored(p.MockBus):
        def write(self, sid, address, value, size=1):
            if address == p.A_TORQUE and value == 0:
                return True
            return super().write(sid, address, value, size)
    bus = Ignored(plan["servo_ids"], plan["start_raw"])
    result = run(bus, plan)
    assert not result["sequence_completed"] and result["stop_errors"]


def test_stop_restore_sends_all_off_before_goal_and_retries_final_off(plan):
    class Ordered(p.MockBus):
        def __init__(self, ids, raw):
            super().__init__(ids, raw)
            self.operations = []
            self.final_off_failures = 0

        def read(self, sid, address, size=1):
            self.operations.append(("read", sid, address))
            return super().read(sid, address, size)

        def write(self, sid, address, value, size=1):
            self.operations.append(("write", sid, address, value))
            if address == p.A_SPEED and sid == plan["servo_ids"][0]:
                self.reg[sid, p.A_TORQUE] = 1
            if address == p.A_TORQUE and value == 0 and self.reg[sid, p.A_TORQUE] == 1:
                self.final_off_failures += 1
                if self.final_off_failures < 3:
                    return True
            return super().write(sid, address, value, size)

    bus = Ordered(plan["servo_ids"], plan["start_raw"])
    errors = p.stop_restore(bus, plan["servo_ids"], [(300, 10)]*6)
    assert not errors
    assert bus.operations[:6] == [
        ("write", sid, p.A_TORQUE, 0) for sid in plan["servo_ids"]
    ]
    assert bus.final_off_failures == 3
    assert all(bus.reg[sid, p.A_TORQUE] == 0 for sid in plan["servo_ids"])


def test_wait_loop_monitors_and_detects_torque_release(plan):
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    calls = 0

    def monitor():
        nonlocal calls
        calls += 1
        if calls == 3:
            bus.reg[plan["servo_ids"][0], p.A_TORQUE] = 0

    clock = Clock()
    result = p.run_sequence(bus, plan, lambda _: True, sleep=clock.sleep,
                            clock=clock.now, monitor=monitor)
    assert calls == 3
    assert "torque_released" in result["failure"]
    assert_restored(bus, plan)


def test_stall_and_interrupt_restore(plan):
    class Stalled(p.MockBus):
        def read(self, sid, address, size=1):
            if address == p.A_POS:
                return self.positions[sid]
            return super().read(sid, address, size)
    bus = Stalled(plan["servo_ids"], plan["start_raw"])
    result = run(bus, plan)
    assert "position_stall" in result["failure"]
    assert_restored(bus, plan)
    bus = p.MockBus(plan["servo_ids"], plan["start_raw"])
    def interrupt(phase):
        raise KeyboardInterrupt()
    assert "KeyboardInterrupt" in run(bus, plan, interrupt)["failure"]
    assert_restored(bus, plan)


def test_stop_continues_when_one_servo_disappears(plan):
    class Missing(p.MockBus):
        def read(self, sid, address, size=1):
            if sid == 1 and address == p.A_POS:
                return None
            return super().read(sid, address, size)
    bus = Missing(plan["servo_ids"], plan["start_raw"])
    errors = p.stop_restore(bus, plan["servo_ids"], [(300, 10)]*6)
    assert errors
    assert all(bus.reg[sid, p.A_TORQUE] == 0 for sid in plan["servo_ids"])


def test_template_cli_never_opens_hardware(tmp_path):
    output = tmp_path / "config.json"
    result = subprocess.run([sys.executable, str(Path(p.__file__)), "template", "--output", str(output)], capture_output=True)
    assert result.returncode == 0
    assert json.loads(output.read_text())["cup_center_m"] is None
    result = subprocess.run([sys.executable, str(Path(p.__file__)), "template", "--output", str(output)], capture_output=True)
    assert result.returncode != 0


def test_simulation_fixture_cannot_open_real_bus_even_with_execute_flags(plan, tmp_path, monkeypatch):
    packet = copy.deepcopy(plan)
    packet["packet_sha256"] = p.digest(packet)
    source = tmp_path / "plan.json"
    source.write_text(json.dumps(packet))
    def forbidden(*args, **kwargs):
        pytest.fail("실측하지 않은 packet에서 버스를 열면 안 됩니다")
    monkeypatch.setattr(p, "open_stop_bus", forbidden)
    monkeypatch.setattr(sys, "argv", ["pick", "execute", "--plan", str(source), "--port", "/dev/left",
        "--right-port", "/dev/right", "--observed-workcell", "--output", str(tmp_path / "out.json")])
    with pytest.raises(SystemExit) as exc:
        p.main()
    assert exc.value.code == 2


def test_execute_uses_exclusive_buses_and_composes_dashboard_monitor(plan, tmp_path, monkeypatch):
    packet = copy.deepcopy(plan)
    packet["config"]["provenance"]["status"] = "user_measured"
    packet["packet_sha256"] = p.digest(packet)
    source = tmp_path / "plan.json"
    source.write_text(json.dumps(packet))
    opened, dashboard_urls = [], []

    class DummyBus:
        def close(self):
            pass

    monkeypatch.setattr(p, "verify_packet", lambda *_: None)
    # 포트/정지 감시 연결만 검사한다. 미검증 기준 거부는 별도 회귀에서 검사한다.
    monkeypatch.setattr(p, "require_verified_physical_mapping", lambda *_: None)
    monkeypatch.setattr(p, "audit", lambda *_: {"robot_sampled_collision_pass": True})
    monkeypatch.setattr(p, "open_stop_bus", lambda port: opened.append(port) or DummyBus())
    monkeypatch.setattr("serial.Serial", lambda *_: pytest.fail("execute가 검증된 포트 개방을 우회하면 안 됩니다"))
    monkeypatch.setattr(p, "dashboard_monitor", dashboard_urls.append)
    monkeypatch.setattr(p, "snapshot", lambda *_: {
        "raw_ticks": packet["right_expected_raw"], "torque": [0]*6})

    polls = 0
    def fake_select(_read, _write, _errors, timeout):
        nonlocal polls
        assert 0 < timeout <= .2
        polls += 1
        if polls == 2:
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        return [], [], []

    monkeypatch.setattr(p.select, "select", fake_select)

    def fake_run(_bus, _plan, _confirm, **kwargs):
        kwargs["monitor"]()
        with pytest.raises(KeyboardInterrupt):
            _confirm("REORIENT_ABOVE")
        with pytest.raises(KeyboardInterrupt):
            kwargs["monitor"]()
        return {"failure": "KeyboardInterrupt:실행 중단", "stop_errors": [],
                "command_write_attempted": False}

    monkeypatch.setattr(p, "run_sequence", fake_run)
    output = tmp_path / "result.json"
    monkeypatch.setattr(sys, "argv", ["pick", "execute", "--plan", str(source),
        "--port", "/dev/left", "--right-port", "/dev/right", "--observed-workcell",
        "--stop-status-url", "http://127.0.0.1:8770/api/stop/status", "--output", str(output)])
    assert p.main() == 2
    assert opened == ["/dev/right", "/dev/left"]
    assert polls == 2
    assert dashboard_urls == ["http://127.0.0.1:8770/api/stop/status"]*4


@pytest.mark.parametrize("with_reference", [False, True])
def test_unverified_mapping_cannot_open_ports_even_when_workspace_is_measured(tmp_path, monkeypatch, with_reference):
    config = fixture_config()
    config["provenance"]["status"] = "user_measured"
    calibration_bytes = p.DEFAULT_CALIBRATION.read_bytes()
    if with_reference:
        config["joint_reference"] = reference_fixture(calibration_bytes)
    packet = p.prepare(config, calibration_bytes)
    packet["packet_sha256"] = p.digest(packet)
    source = tmp_path / "plan.json"
    source.write_text(json.dumps(packet))
    monkeypatch.setattr(p, "audit", lambda *_: {"robot_sampled_collision_pass": True})
    monkeypatch.setattr(p, "open_stop_bus", lambda *_: pytest.fail("미검증 기준으로 포트를 열 수 없습니다"))
    monkeypatch.setattr(sys, "argv", ["pick", "execute", "--plan", str(source),
        "--port", "/dev/left", "--right-port", "/dev/right", "--observed-workcell",
        "--output", str(tmp_path / "result.json")])
    with pytest.raises(SystemExit) as exc:
        p.main()
    assert exc.value.code == 2
    assert not (tmp_path / "result.json").exists()
