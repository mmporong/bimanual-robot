import importlib.util
from pathlib import Path
import sys

import pytest


MODULE_PATH = Path(__file__).parent / "servo" / "servo_goto.py"
SPEC = importlib.util.spec_from_file_location("servo_goto", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC.loader.exec_module(MODULE)


def _argv():
    return [
        "servo_goto.py", "--mock", "--id", "6", "--goal", "3500",
        "--max-delta", "200", "--speed", "300", "--execute",
        "--hold-torque-on-success", "--position-only",
    ]


def test_hold_success_requires_confirmed_torque(monkeypatch):
    class RecordingBus(MODULE.FakeBus):
        instance = None

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            RecordingBus.instance = self

    monkeypatch.setattr(MODULE, "FakeBus", RecordingBus)
    monkeypatch.setattr(sys, "argv", _argv())
    assert MODULE.main() == 0
    assert RecordingBus.instance.reg[MODULE.A_TORQUE] == 1


def test_hold_torque_read_failure_releases_torque(monkeypatch):
    class TorqueReadFailureBus(MODULE.FakeBus):
        instance = None

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            TorqueReadFailureBus.instance = self

        def read(self, sid, address, n=1):
            if address == MODULE.A_TORQUE:
                return None
            return super().read(sid, address, n)

    monkeypatch.setattr(MODULE, "FakeBus", TorqueReadFailureBus)
    monkeypatch.setattr(sys, "argv", _argv())
    with pytest.raises(RuntimeError, match="토크 유지 확인 실패"):
        MODULE.main()
    assert TorqueReadFailureBus.instance.reg[MODULE.A_TORQUE] == 0


def test_hold_profile_restore_failure_releases_torque(monkeypatch):
    class RestoreFailureBus(MODULE.FakeBus):
        instance = None

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.speed_writes = 0
            RestoreFailureBus.instance = self

        def write(self, sid, address, value, n=1):
            if address == MODULE.A_SPEED:
                self.speed_writes += 1
                if self.speed_writes == 2:
                    return False
            return super().write(sid, address, value, n)

    monkeypatch.setattr(MODULE, "FakeBus", RestoreFailureBus)
    monkeypatch.setattr(sys, "argv", _argv())
    with pytest.raises(RuntimeError, match="속도 프로파일 복원 실패"):
        MODULE.main()
    assert RestoreFailureBus.instance.reg[MODULE.A_TORQUE] == 0
