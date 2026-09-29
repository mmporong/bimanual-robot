import importlib.util
import json
import re
import threading
from http.client import HTTPConnection
from pathlib import Path
import pytest

from dashboard_stop import StopController
from servo.sts_bus import A_TORQUE


MODULE_PATH = Path(__file__).with_name("camera_dashboard.py")
SPEC = importlib.util.spec_from_file_location("camera_dashboard", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_dashboard_html_has_three_streams_and_only_stop_control():
    html = MODULE.INDEX.read_text(encoding="utf-8")
    assert "/stream/top" in html
    assert "/stream/wrist" in html
    assert "/stream/wrist_other" in html
    assert "/api/stop" in html
    assert "/cmd" not in html


def test_default_camera_paths_are_stable_by_path():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "/dev/v4l/by-path/pci-0000:67:00.3-usb-0:1.1:1.0-video-index0" in source
    assert "/dev/v4l/by-path/pci-0000:67:00.4-usb-0:1:1.0-video-index0" in source


class MockBus:
    def __init__(self, port, events, fail=False):
        self.port, self.events, self.fail = port, events, fail

    def write(self, sid, addr, value):
        assert addr == A_TORQUE and value == 0
        self.events.append((self.port, "off", sid))
        if self.fail and sid == 1:
            raise SystemExit("USB lost")
        return True

    def read(self, sid, addr):
        assert addr == A_TORQUE
        self.events.append((self.port, "read", sid))
        return None if self.fail else 0

    def close(self):
        self.events.append((self.port, "close", 0))


@pytest.mark.parametrize("ports", [["A"], ["A", "A"], ["A", "B", "C"]])
def test_incomplete_or_duplicate_arm_configuration_rejected(ports):
    with pytest.raises(ValueError):
        StopController(ports)


def test_stop_all_before_readback_and_repeat(tmp_path):
    events = []
    control = StopController(["A", "B"], lambda p: MockBus(p, events), tmp_path / "stop.jsonl")
    assert control.stop()["verified_off"]
    assert [e[1] for e in events[:12]] == ["off"] * 12
    assert len(events) == 26
    control.stop()
    assert len(events) == 52
    assert len((tmp_path / "stop.jsonl").read_text().splitlines()) == 2


def test_disconnect_does_not_skip_other_arm():
    events = []
    control = StopController(["A", "B"], lambda p: MockBus(p, events, p == "A"))
    assert not control.stop()["verified_off"]
    assert len([e for e in events if e[1] == "off"]) == 12
    assert len([e for e in events if e[1] == "close"]) == 2


def test_open_failure_does_not_skip_other_arm():
    events = []
    def factory(port):
        if port == "A":
            raise OSError("busy")
        return MockBus(port, events)
    result = StopController(["A", "B"], factory).stop()
    assert not result["verified_off"]
    assert len([e for e in events if e[1] == "off"]) == 6


def test_stop_log_failure_keeps_hardware_result(tmp_path):
    result = StopController(["A", "B"], lambda p: MockBus(p, []), tmp_path).stop()
    assert result["verified_off"] and result["log_error"]


def test_http_get_and_unauthorized_post_do_not_touch_bus():
    events = []
    control = StopController(["A", "B"], lambda p: MockBus(p, events))
    server = MODULE.ThreadingHTTPServer(("127.0.0.1", 0), MODULE.make_handler({}, control))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    conn = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
    try:
        conn.request("GET", "/")
        response = conn.getresponse()
        html = response.read().decode()
        token = re.search(r"'X-Stop-Token':'([^']+)'", html).group(1)
        for url in ["/api/status", "/api/stop/status"]:
            conn.request("GET", url)
            assert conn.getresponse().read()
        conn.request("POST", "/api/stop")
        response = conn.getresponse()
        assert response.status == 403
        response.read()
        assert not events
        conn.request("POST", "/api/stop", headers={"Origin": f"http://127.0.0.1:{server.server_port}", "X-Stop-Token": token})
        response = conn.getresponse()
        assert response.status == 200
        assert json.loads(response.read())["verified_off"]
        assert len([e for e in events if e[1] == "off"]) == 12
    finally:
        conn.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
