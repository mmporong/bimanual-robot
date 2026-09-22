import json
from http.server import ThreadingHTTPServer
from pathlib import Path
import sqlite3
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from service_mission_control import MissionController
from service_execution_backend import BackendResult, ImmediateBackend, Ros2ManipulationBackend
from service_order_server import (
    BackendBusyError,
    DASHBOARD_JS,
    INDEX,
    ServiceApplication,
    default_state_dir,
    handler_factory,
)
from service_order_store import ServiceOrderStore


def post_json(url, payload):
    body = json.dumps(payload).encode()
    request = Request(url, data=body, headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=3) as response:
        return response.status, json.loads(response.read())


@pytest.fixture
def server(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield app, f"http://127.0.0.1:{httpd.server_port}", tmp_path / "state"
    httpd.shutdown()
    httpd.server_close()
    thread.join(timeout=3)
    app.close()


def test_dashboard_and_health_are_served(server):
    _, base, _ = server
    with urlopen(base + "/", timeout=3) as response:
        html = response.read().decode()
    assert "양팔 물 서빙 관제" in html
    assert "data-table=\"table_4\"" in html
    with urlopen(base + "/api/health", timeout=3) as response:
        health = json.loads(response.read())
    assert health == {
        "status": "ok",
        "mode": "simulation_dry_run",
        "manipulation": "immediate_success",
        "navigation": "immediate_success",
        "charge": "immediate_success",
        "active_goal": False,
        "hardware_accessed": False,
    }

    with urlopen(base + "/dashboard.js", timeout=3) as response:
        javascript = response.read().decode()
    assert "renderMap" in javascript
    assert "execution-backend" in javascript


def test_order_api_is_idempotent_and_persists_state(server):
    _, base, state_dir = server
    payload = {"order_id": "WEB-TEST-1", "table_id": "table_2", "drink": "HOT_WATER"}
    first_status, first = post_json(base + "/api/orders", payload)
    second_status, second = post_json(base + "/api/orders", payload)
    assert first_status == 201 and second_status == 200
    assert first["order"]["order_id"] == second["order"]["order_id"]
    assert first["state"]["active_order_id"] == "WEB-TEST-1"
    stored = json.loads((state_dir / "snapshot.json").read_text())
    assert stored["active_order_id"] == "WEB-TEST-1"
    assert (state_dir / "events.jsonl").is_file()
    database = state_dir / "service_mission_control.sqlite3"
    assert database.is_file()
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT COUNT(*) FROM orders").fetchone()[0] == 1
    with urlopen(base + "/api/orders/history?limit=1", timeout=3) as response:
        history = json.loads(response.read())
    assert [item["order_id"] for item in history["orders"]] == ["WEB-TEST-1"]


def test_step_failure_and_cancel_endpoints(server):
    _, base, state_dir = server
    post_json(base + "/api/orders", {
        "order_id": "ACTIVE", "table_id": "table_1", "drink": "COLD_WATER"
    })
    post_json(base + "/api/orders", {
        "order_id": "WAITING", "table_id": "table_3", "drink": "HOT_WATER"
    })
    status, canceled = post_json(base + "/api/orders/WAITING/cancel", {})
    assert status == 200 and canceled["order"]["state"] == "CANCELED"
    status, stepped = post_json(base + "/api/step", {"success": True})
    assert status == 200
    assert stepped["executed_command"]["destination"] == "kitchen"
    with sqlite3.connect(state_dir / "service_mission_control.sqlite3") as connection:
        stored = connection.execute(
            "SELECT order_id, phase, kind FROM commands ORDER BY command_sequence DESC LIMIT 1"
        ).fetchone()
    assert stored == ("ACTIVE", "NAVIGATE_KITCHEN", "navigate")


def test_invalid_order_returns_400(server):
    _, base, _ = server
    with pytest.raises(HTTPError) as error:
        post_json(base + "/api/orders", {
            "order_id": "BAD", "table_id": "table_99", "drink": "COLD_WATER"
        })
    assert error.value.code == 400
    with pytest.raises(HTTPError) as limit_error:
        urlopen(base + "/api/events?limit=0", timeout=3)
    assert limit_error.value.code == 400


def test_static_dashboard_has_no_external_runtime_dependencies():
    html = Path(INDEX).read_text()
    javascript = Path(DASHBOARD_JS).read_text()
    assert "https://" not in html and "http://" not in html
    assert "fetch(\"http" not in javascript and "request(\"http" not in javascript
    assert "request(\"/api/orders\"" in javascript
    assert ".innerHTML" not in javascript


def test_restart_restores_active_mission_and_history(tmp_path):
    state_dir = tmp_path / "persistent-state"
    first = ServiceApplication(MissionController(), state_dir)
    first.submit({"order_id": "RESTORE-1", "table_id": "table_4", "drink": "HOT_WATER"})
    first.step()
    before = first.state()
    assert before["phase"] == "ALIGN_KITCHEN"
    first.close()

    second = ServiceApplication(MissionController(), state_dir)
    after = second.state()
    assert after["active_order_id"] == "RESTORE-1"
    assert after["phase"] == "ALIGN_KITCHEN"
    assert after["battery_percent"] == before["battery_percent"]
    assert after["persistence"]["backend"] == "sqlite"
    assert any(event["kind"] == "CONTROLLER_RESTORED" for event in after["events"])
    assert second.history("orders", 10)[0]["order_id"] == "RESTORE-1"
    second.close()


def test_default_state_dir_uses_persistent_user_state(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "xdg-state"))
    assert default_state_dir() == tmp_path / "xdg-state/bimanual-robot/service-order-server"


def test_store_rejects_unknown_schema_version(tmp_path):
    state_dir = tmp_path / "future-schema"
    state_dir.mkdir()
    database = state_dir / "service_mission_control.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        connection.execute("INSERT INTO metadata(key, value) VALUES('schema_version', '99')")
    with pytest.raises(RuntimeError, match="unsupported service-order database schema"):
        ServiceOrderStore(state_dir)


@pytest.mark.parametrize("timeout_sec", [0.0, -1.0, float("nan"), float("inf")])
def test_ros2_backend_rejects_non_positive_or_non_finite_timeout(timeout_sec):
    with pytest.raises(ValueError, match="positive finite"):
        Ros2ManipulationBackend(timeout_sec=timeout_sec)


class FailureBackend(ImmediateBackend):
    mode = "test_failure"

    def execute(self, command, *, mission_id, phase_attempt):
        del mission_id, phase_attempt
        if command["kind"] == "manipulate":
            return BackendResult(False, "POSE_TOLERANCE", self.mode)
        return BackendResult(True, adapter=self.mode)


def test_persistent_executor_cannot_be_advanced_by_manual_step(tmp_path):
    backend = ImmediateBackend()
    backend.mode = 'ros2_planned_session'
    app = ServiceApplication(MissionController(), tmp_path, backend)
    try:
        with pytest.raises(BackendBusyError, match='manual phase advance'):
            app.step()
        assert app.controller.phase == 'IDLE_AT_DOCK'
    finally:
        app.close()


def test_executor_observation_is_retained_in_status_and_snapshot(tmp_path):
    class ObservedBackend(ImmediateBackend):
        def info(self):
            return {**super().info(), 'simulator_accessed': True, 'executor_kind': 'isaac_physics'}
    app = ServiceApplication(MissionController(), tmp_path, ObservedBackend())
    try:
        assert app.state()['simulator_accessed'] is True
        saved = json.loads((tmp_path / 'snapshot.json').read_text())
        assert saved['simulator_accessed'] is True
        assert saved['execution_backend']['executor_kind'] == 'isaac_physics'
    finally:
        app.close()


def test_backend_result_controls_phase_and_is_persisted(tmp_path):
    state_dir = tmp_path / "backend-result"
    app = ServiceApplication(MissionController(), state_dir, FailureBackend())
    app.submit({"order_id": "BACKEND-1", "table_id": "table_1", "drink": "COLD_WATER"})
    navigation = app.execute_next()
    assert navigation["backend_result"]["success"] is True
    assert app.controller.phase == "ALIGN_KITCHEN"

    manipulation = app.execute_next()
    assert manipulation["backend_result"]["failure_code"] == "POSE_TOLERANCE"
    assert app.controller.phase == "ALIGN_KITCHEN"
    assert app.controller.phase_attempt == 2
    with sqlite3.connect(state_dir / "service_mission_control.sqlite3") as connection:
        payload = json.loads(connection.execute(
            "SELECT payload_json FROM commands ORDER BY command_sequence DESC LIMIT 1"
        ).fetchone()[0])
    assert payload["backend_result"]["failure_code"] == "POSE_TOLERANCE"
    app.close()


class BlockingCancelBackend(ImmediateBackend):
    mode = "test_blocking_cancel"

    def __init__(self):
        self.started = threading.Event()
        self.canceled = threading.Event()

    def execute(self, command, *, mission_id, phase_attempt):
        del command, mission_id, phase_attempt
        self.started.set()
        assert self.canceled.wait(timeout=3)
        return BackendResult(False, "CANCELED", self.mode)

    def cancel_active(self):
        self.canceled.set()
        return True


def test_active_order_cancel_propagates_and_backend_result_is_superseded(tmp_path):
    backend = BlockingCancelBackend()
    app = ServiceApplication(MissionController(), tmp_path / "cancel", backend)
    app.submit({"order_id": "CANCEL-1", "table_id": "table_1", "drink": "COLD_WATER"})
    app.controller.advance()
    assert app.controller.phase == "ALIGN_KITCHEN"

    result = {}
    worker = threading.Thread(target=lambda: result.update(app.execute_next()))
    worker.start()
    assert backend.started.wait(timeout=2)
    canceled = app.cancel("CANCEL-1")
    worker.join(timeout=3)

    assert not worker.is_alive()
    assert canceled["backend_cancel_requested"] is True
    assert canceled["order"]["state"] == "CANCELED"
    assert result["superseded"] is True
    assert result["backend_result"]["failure_code"] == "CANCELED"
    assert app.controller.orders["CANCEL-1"].state == "CANCELED"
    app.close()
