import json
from http.server import ThreadingHTTPServer
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from service_mission_control import MissionController
from service_order_server import INDEX, ServiceApplication, handler_factory


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


def test_dashboard_and_health_are_served(server):
    _, base, _ = server
    with urlopen(base + "/", timeout=3) as response:
        html = response.read().decode()
    assert "HOLD THE FLOW" in html
    assert "data-table=\"table_4\"" in html
    with urlopen(base + "/api/health", timeout=3) as response:
        health = json.loads(response.read())
    assert health == {"status": "ok", "mode": "simulation_dry_run"}


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


def test_step_failure_and_cancel_endpoints(server):
    _, base, _ = server
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


def test_invalid_order_returns_400(server):
    _, base, _ = server
    with pytest.raises(HTTPError) as error:
        post_json(base + "/api/orders", {
            "order_id": "BAD", "table_id": "table_99", "drink": "COLD_WATER"
        })
    assert error.value.code == 400


def test_static_dashboard_has_no_external_runtime_dependencies():
    html = Path(INDEX).read_text()
    assert "https://" not in html and "http://" not in html
    assert "fetch(\"/api/orders\"" in html
