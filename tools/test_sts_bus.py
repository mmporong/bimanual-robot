import importlib.util
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).parent / "servo" / "sts_bus.py"
SPEC = importlib.util.spec_from_file_location("sts_bus_tested", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def status_packet(sid, params=b"", error=0):
    length = len(params) + 2
    body = bytes([error]) + params
    checksum = (~(sid + length + sum(body))) & 0xFF
    return b"\xff\xff" + bytes([sid, length]), body + bytes([checksum])


def test_status_packet_rejects_a_delayed_response_from_another_servo():
    head, rest = status_packet(4, b"\x23")
    assert MODULE.validate_status_packet(5, head, rest) is None


def test_status_packet_accepts_expected_servo_and_returns_parameters():
    head, rest = status_packet(5, b"\x23")
    assert MODULE.validate_status_packet(5, head, rest) == b"\x23"


def test_status_packet_rejects_error_and_bad_checksum():
    head, rest = status_packet(5, b"\x23", error=1)
    assert MODULE.validate_status_packet(5, head, rest) is None
    head, rest = status_packet(5, b"\x23")
    assert MODULE.validate_status_packet(5, head, rest[:-1] + b"\x00") is None
