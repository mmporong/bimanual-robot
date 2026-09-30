from dataclasses import dataclass, replace
import json
from types import SimpleNamespace

import pytest

from servo.standard_so101_calibration import PRESERVED, calibrate_connected


@dataclass
class Calibration:
    id: int = 1
    drive_mode: int = 0
    homing_offset: int = 10
    range_min: int = 500
    range_max: int = 3500


class Bus:
    def __init__(self):
        self.motors = {"shoulder": None, "elbow": None}
        self.cal = {n: Calibration(id=i + 1) for i, n in enumerate(self.motors)}
        self.values = {n: {**dict.fromkeys(PRESERVED, 0), "Lock": 1,
                          "Torque_Enable": 0, "Present_Position": 2000} for n in self.motors}
        self.writes = []

    def read_calibration(self):
        return self.cal.copy()

    def read(self, reg, name, normalize=False):
        assert reg not in {"Present_Load", "Present_Temperature"}
        return self.values[name][reg]

    def write(self, reg, name, value):
        assert reg not in {"Goal_Position", "P_Coefficient", "I_Coefficient", "D_Coefficient", "Phase"}
        assert reg != "Torque_Enable" or value == 0
        self.writes.append((reg, name, value))
        self.values[name][reg] = value

    def write_calibration(self, data, cache=False):
        self.cal.update(data)


def robot_with(error=None, invalid=False):
    bus = Bus()
    robot = SimpleNamespace(bus=bus, calibration={})

    def official_calibrate():
        for n in bus.motors:
            bus.write("Lock", n, 0)
            bus.cal[n] = replace(bus.cal[n], homing_offset=200,
                                 range_min=2000 if invalid else 500,
                                 range_max=2000 if invalid else 3500)
        if error:
            raise error
        robot.calibration = bus.cal.copy()

    robot.calibrate = official_calibrate
    return robot


def test_success_keeps_new_calibration_without_motion(tmp_path):
    robot = robot_with()
    result = calibrate_connected(robot, tmp_path, b'{"original":true}')
    assert result["calibration_completed"]
    assert not result["motion_command_emitted"]
    assert not result["physical_mapping_verified"]
    assert all(v.homing_offset == 200 for v in robot.bus.cal.values())
    assert all(v["Lock"] == 1 and v["Torque_Enable"] == 0 for v in robot.bus.values.values())
    assert (tmp_path / "active_calibration.backup.json").read_bytes() == b'{"original":true}'


@pytest.mark.parametrize("error", [RuntimeError("read failure"), KeyboardInterrupt("cancel")])
def test_failure_restores_actual_board_calibration(tmp_path, error):
    robot = robot_with(error)
    original = robot.bus.cal.copy()
    with pytest.raises(type(error)):
        calibrate_connected(robot, tmp_path, b"{}")
    assert robot.bus.cal == original
    result = json.loads((tmp_path / "result.json").read_text())
    assert not result["calibration_completed"]
    assert result["restoration_errors"] == []


def test_invalid_candidate_restores_original(tmp_path):
    robot = robot_with(invalid=True)
    original = robot.bus.cal.copy()
    with pytest.raises(RuntimeError, match="잘못된 보정 범위"):
        calibrate_connected(robot, tmp_path, b"{}")
    assert robot.bus.cal == original


def test_torque_on_rejected_without_writes(tmp_path):
    robot = robot_with()
    robot.bus.values["elbow"]["Torque_Enable"] = 1
    with pytest.raises(ValueError, match="OFF"):
        calibrate_connected(robot, tmp_path, b"{}")
    assert robot.bus.writes == []


def test_backup_collision_rejected_before_official_calibration(tmp_path):
    robot = robot_with()
    (tmp_path / "before.json").write_text("existing")
    with pytest.raises(FileExistsError):
        calibrate_connected(robot, tmp_path, b"{}")
    assert robot.bus.writes == []
