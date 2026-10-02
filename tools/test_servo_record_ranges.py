import json
import pytest

from servo import servo_record_ranges as recorder


def fixture():
    return {name: {"id": i, "drive_mode": int(name == "gripper"),
                   "homing_offset": 10, "range_min": 0 if name == "wrist_roll" else 500,
                   "range_max": 4095 if name == "wrist_roll" else 2500}
            for i, name in enumerate(recorder.SO101, 1)}


class ReadOnlyBus:
    def __init__(self):
        self.closed = False

    def read(self, sid, address, size=1):
        if address == recorder.A_TORQUE:
            return 0
        if address == recorder.A_OFFSET:
            return 10
        raise AssertionError(f"unexpected read: {address}")

    def write_eeprom(self, *args):
        raise AssertionError("capture-only must not write EEPROM")

    def close(self):
        self.closed = True


@pytest.mark.parametrize("name,key,value", [
    ("gripper", "range_min", 2490), ("gripper", "range_min", 2600),
    ("shoulder_pan", "range_min", 0), ("shoulder_pan", "range_max", 4095),
    ("wrist_flex", "homing_offset", 2048), ("wrist_roll", "id", 6),
    ("elbow_flex", "range_min", 500.0), ("shoulder_lift", "drive_mode", 1),
])
def test_invalid_capture_rejected(name, key, value):
    c = fixture()
    c[name][key] = value
    with pytest.raises(ValueError):
        recorder.validate_capture(c)


def test_full_turn_and_reversed_parallel_gripper_supported():
    recorder.validate_capture(fixture())


def setup_capture(monkeypatch, tmp_path):
    bus = ReadOnlyBus()
    path = tmp_path / "candidate.json"
    monkeypatch.setattr(recorder, "open_stop_bus", lambda _: bus)
    monkeypatch.setattr(recorder, "record", lambda *_: (
        {i: 500 for i in range(1, 7)}, {i: 2500 for i in range(1, 7)}, {}))
    monkeypatch.setattr("sys.argv", ["recorder", "--port", "fake", "--capture-only", "--out", str(path),
                                     "--gripper-drive-mode", "1", "--gripper-margin", "40"])
    return bus, path


def test_capture_only_saves_candidate_without_hardware_write(monkeypatch, tmp_path):
    bus, path = setup_capture(monkeypatch, tmp_path)
    assert recorder.main() == 0
    c = json.loads(path.read_text())
    assert c["gripper"]["drive_mode"] == 1
    assert c["gripper"]["range_min"] == 540 and c["gripper"]["range_max"] == 2460
    assert c["wrist_roll"]["range_min"] == 0 and c["wrist_roll"]["range_max"] == 4095
    assert bus.closed


def test_existing_candidate_not_overwritten(monkeypatch, tmp_path):
    bus, path = setup_capture(monkeypatch, tmp_path)
    path.write_text("previous")
    with pytest.raises(FileExistsError):
        recorder.main()
    assert path.read_text() == "previous" and bus.closed


def test_capture_and_apply_cannot_be_combined(monkeypatch, tmp_path):
    setup_capture(monkeypatch, tmp_path)
    monkeypatch.setattr("sys.argv", ["recorder", "--out", "unused", "--capture-only", "--execute"])
    with pytest.raises(SystemExit):
        recorder.main()


def test_large_position_discontinuity_rejected(monkeypatch):
    entered = iter([False, False, True])
    samples = iter([1000, 3500])
    monkeypatch.setattr(recorder, "enter_pressed", lambda: next(entered))
    monkeypatch.setattr(recorder.time, "sleep", lambda _: None)
    bus = type("Bus", (), {"read": lambda *_: next(samples)})()
    with pytest.raises(RuntimeError, match="불연속"):
        recorder.record(bus, [1], 250)


def test_single_unconfirmed_glitch_not_accepted(monkeypatch):
    entered = iter([False, False, False, True])
    samples = iter([1000, 1400, 1000])
    monkeypatch.setattr(recorder, "enter_pressed", lambda: next(entered))
    monkeypatch.setattr(recorder.time, "sleep", lambda _: None)
    bus = type("Bus", (), {"read": lambda *_: next(samples)})()
    lo, hi, dropped = recorder.record(bus, [1], 250)
    assert lo == hi == {1: 1000}
    assert dropped == {1: 1}
