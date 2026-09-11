import contextlib
import io
import sys
import unittest
from pathlib import Path


SERVO_DIR = Path(__file__).resolve().parents[1] / "tools" / "servo"
sys.path.insert(0, str(SERVO_DIR))

from servo_check_phase import check_phase  # noqa: E402
from sts_bus import FakeBus  # noqa: E402


class ServoPhaseCheckTest(unittest.TestCase):
    def test_expected_phase_passes(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(check_phase(FakeBus(phase=76), 6, 76))

    def test_wrong_phase_fails(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(check_phase(FakeBus(phase=12), 6, 76))

    def test_missing_servo_fails(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(check_phase(FakeBus(sid=5, phase=76), 6, 76))


if __name__ == "__main__":
    unittest.main()
