import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from apps.service_order.service_order_server import ServiceApplication, handler_factory
from src.mission.service_execution_backend import RolloutServiceBackend
from src.mission.service_mission_control import MissionController


class FakeRolloutService:
    """Stands in for tools/rollout/rollout_service.py: records calls, returns scripted results."""

    def __init__(self):
        self.calls: list[str] = []
        self.results = {"pour": {"task": "pour", "ok": True, "reason": "auto_done"},
                        "place": {"task": "place", "ok": True, "reason": "auto_done"}}
        self.loading_polls = 0
        self.stops = 0
        self.stow_pose = None  # e.g. "place_end" to behave like --stow.pose
        self.stow_ok = True
        self.arms = "unknown"
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                return None

            def _json(self, code, body):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self._json(200, {"state": "loading" if fake.loading_polls else "ready",
                                 "stow_pose": fake.stow_pose, "arms": fake.arms})

            def do_POST(self):
                if self.path == "/stop":
                    fake.stops += 1
                    self._json(200, {"stopped": True})
                    return
                task = self.path.rsplit("/", 1)[-1]
                if task == "stow" and fake.stow_pose is None:
                    self._json(404, {"error": "stow_not_configured"})
                    return
                if fake.loading_polls:
                    fake.loading_polls -= 1
                    self._json(409, {"error": "not_ready", "state": "loading"})
                    return
                fake.calls.append(task)
                if task == "stow":
                    fake.arms = "stowed" if fake.stow_ok else "hold"
                    self._json(200, {"task": "stow", "ok": fake.stow_ok,
                                     "reason": "stowed" if fake.stow_ok else "stow_failed"})
                    return
                fake.arms = "carrying_cup" if task == "pour" else "stowed"
                self._json(200, fake.results[task])

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.httpd.server_port}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


@pytest.fixture
def fake():
    service = FakeRolloutService()
    yield service
    service.close()


def manip(phase, order_id="ORD-1"):
    return {"kind": "manipulate", "phase": phase, "order_id": order_id}


def test_each_policy_runs_once_and_covers_its_phases(fake):
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    for phase in ("ALIGN_KITCHEN", "GRASP_CUP", "GRASP_BOTTLE", "POUR", "RETURN_BOTTLE", "PLACE_DECK",
                  "ALIGN_TABLE", "REGRASP_CUP", "SERVE"):
        result = backend.execute(manip(phase), mission_id="M1", phase_attempt=1)
        assert result.success, (phase, result)
    # a retried phase reports the cached result instead of moving the arms again
    assert backend.execute(manip("GRASP_CUP"), mission_id="M1", phase_attempt=2).success
    assert fake.calls == ["pour", "place"]
    assert backend.execute({"kind": "navigate"}, mission_id="M1", phase_attempt=1).success


def test_policy_failure_is_cached_and_mapped(fake):
    fake.results["pour"] = {"task": "pour", "ok": False, "reason": "duration"}
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    first = backend.execute(manip("GRASP_CUP"), mission_id="M1", phase_attempt=1)
    retry = backend.execute(manip("GRASP_CUP"), mission_id="M1", phase_attempt=2)
    assert (first.success, first.failure_code) == (False, "POUR_END_NOT_DETECTED")
    assert retry.failure_code == "POUR_END_NOT_DETECTED"
    assert fake.calls == ["pour"]
    accepting = RolloutServiceBackend(url=fake.url, require_alignment=False, accept_duration=True)
    assert accepting.execute(manip("GRASP_CUP"), mission_id="M2", phase_attempt=1).success


def test_later_phase_without_policy_run_fails(fake):
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    result = backend.execute(manip("SERVE"), mission_id="M1", phase_attempt=1)
    assert (result.success, result.failure_code) == (False, "PLACE_NOT_RUN")
    assert fake.calls == []


def test_alignment_gate_waits_for_signal_and_can_be_canceled(fake):
    backend = RolloutServiceBackend(url=fake.url)
    assert backend.confirm_alignment()["reason"] == "not_waiting"
    out = {}
    worker = threading.Thread(target=lambda: out.setdefault(
        "r", backend.execute(manip("ALIGN_KITCHEN"), mission_id="M1", phase_attempt=1)))
    worker.start()
    time.sleep(0.3)
    assert backend.info()["waiting_alignment"]["station"] == "kitchen"
    assert backend.confirm_alignment("table")["reason"] == "station_mismatch"
    assert backend.confirm_alignment("kitchen")["confirmed"]
    worker.join(timeout=3)
    assert out["r"].success
    assert backend.info()["waiting_alignment"] is None

    worker = threading.Thread(target=lambda: out.__setitem__(
        "c", backend.execute(manip("ALIGN_TABLE"), mission_id="M1", phase_attempt=1)))
    worker.start()
    time.sleep(0.3)
    assert backend.cancel_active()
    worker.join(timeout=3)
    assert out["c"].failure_code == "CANCELED_WHILE_WAITING_ALIGNMENT"
    assert fake.calls == []


def test_waits_while_service_is_loading_and_reports_unreachable(fake):
    fake.loading_polls = 2
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    assert backend.execute(manip("GRASP_CUP"), mission_id="M1", phase_attempt=1).success
    assert fake.calls == ["pour"]
    dead = RolloutServiceBackend(url="http://127.0.0.1:9", require_alignment=False)
    result = dead.execute(manip("GRASP_CUP"), mission_id="M1", phase_attempt=1)
    assert result.failure_code == "ROLLOUT_SERVICE_UNREACHABLE"
    assert dead.info()["manipulation_service_state"] == "unreachable"


def test_order_runs_end_to_end_with_alignment_button(fake, tmp_path):
    fake.stow_pose = "place_end"
    backend = RolloutServiceBackend(url=fake.url)
    app = ServiceApplication(MissionController(), tmp_path / "state", backend)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_port}"

    def post(path, payload):
        request = Request(base + path, data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"}, method="POST")
        with urlopen(request, timeout=5) as response:
            return json.loads(response.read())

    try:
        post("/api/orders", {"order_id": "ORD-1", "table_id": "table_2", "drink": "COLD_WATER"})
        done = threading.Event()

        def run():
            while app.state()["orders"][0]["state"] == "RUNNING":
                app.execute_next()
            done.set()

        threading.Thread(target=run, daemon=True).start()
        for station in ("kitchen", "table"):
            for _ in range(50):
                waiting = app.state()["execution_backend"]["waiting_alignment"]
                # right after a confirm the previous wait stays visible for up to one poll (0.2 s)
                if waiting and waiting["station"] == station:
                    break
                time.sleep(0.1)
            assert waiting["station"] == station
            assert post("/api/align/confirm", {"station": station})["confirmed"]
        assert done.wait(5)
        assert app.state()["orders"][0]["state"] == "SUCCEEDED"
        # arms fold before the first drive, then pour and place each run once
        assert fake.calls == ["stow", "pour", "place"]
    finally:
        httpd.shutdown()
        httpd.server_close()
        app.close()


def test_alignment_endpoint_rejects_backends_without_gate(tmp_path):
    app = ServiceApplication(MissionController(), tmp_path / "state")
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(app))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        request = Request(f"http://127.0.0.1:{httpd.server_port}/api/align/confirm", data=b"{}",
                          headers={"Content-Type": "application/json"}, method="POST")
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 409
    finally:
        httpd.shutdown()
        httpd.server_close()
        app.close()


def nav(destination):
    return {"kind": "navigate", "destination": destination}


def test_arms_fold_once_before_driving_to_the_kitchen(fake):
    fake.stow_pose = "place_end"
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    assert backend.execute(nav("kitchen"), mission_id="M1", phase_attempt=1).success
    # a retried kitchen drive does not move the arms again
    assert backend.execute(nav("kitchen"), mission_id="M1", phase_attempt=2).success
    assert fake.calls == ["stow"]
    info = backend.info()
    assert info["drive"] == {"next": "kitchen", "arms_ready": True}
    assert info["stow_pose"] == "place_end"
    # the table and dock legs do not fold the arms (the service already did it after pour / place)
    assert backend.execute(nav("table_2"), mission_id="M1", phase_attempt=1).success
    assert backend.execute(nav("dock"), mission_id="M1", phase_attempt=1).success
    assert fake.calls == ["stow"]
    # next mission folds again
    assert backend.execute(nav("kitchen"), mission_id="M2", phase_attempt=1).success
    assert fake.calls == ["stow", "stow"]


def test_failed_fold_blocks_the_drive(fake):
    fake.stow_pose = "place_end"
    fake.stow_ok = False
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    result = backend.execute(nav("kitchen"), mission_id="M1", phase_attempt=1)
    assert (result.success, result.failure_code) == (False, "STOW_FAILED")
    assert backend.info()["drive"]["arms_ready"] is False


def test_service_without_stow_pose_keeps_old_behavior(fake):
    backend = RolloutServiceBackend(url=fake.url, require_alignment=False)
    result = backend.execute(nav("kitchen"), mission_id="M1", phase_attempt=1)
    assert result.success and result.action_result["stow"] == "not_configured"
    assert fake.calls == []
    dead = RolloutServiceBackend(url="http://127.0.0.1:9", require_alignment=False)
    assert dead.execute(nav("kitchen"), mission_id="M1", phase_attempt=1).failure_code ==         "ROLLOUT_SERVICE_UNREACHABLE"


def test_drive_hint_follows_the_mission(fake):
    fake.stow_pose = "place_end"
    backend = RolloutServiceBackend(url=fake.url)
    seen = []
    done = threading.Event()

    def run():
        for phase in ("ALIGN_KITCHEN", "GRASP_CUP", "ALIGN_TABLE", "REGRASP_CUP"):
            backend.execute(manip(phase), mission_id="M1", phase_attempt=1)
        done.set()

    assert backend.execute(nav("kitchen"), mission_id="M1", phase_attempt=1).success
    threading.Thread(target=run, daemon=True).start()
    for station in ("kitchen", "table"):
        for _ in range(50):
            info = backend.info()
            if info["waiting_alignment"] and info["waiting_alignment"]["station"] == station:
                break
            time.sleep(0.1)
        seen.append(info["drive"])
        backend.confirm_alignment(station)
    assert done.wait(5)
    assert seen == [{"next": "kitchen", "arms_ready": True}, {"next": "table", "arms_ready": True}]
    assert fake.calls == ["stow", "pour", "place"]
