#!/usr/bin/env python3
"""Local web order API for the CPU-only service mission controller."""
from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
from typing import Any
from urllib.parse import urlparse

from service_mission_control import DryRunRuntime, MissionController


ROOT = Path(__file__).resolve().parents[1]
INDEX = Path(__file__).with_name("service_order_dashboard") / "index.html"
ORDER_PATH = re.compile(r"^/api/orders/([A-Za-z0-9._:-]{1,64})/cancel$")


class ServiceApplication:
    """Thread-safe application boundary shared by HTTP and the dry-run loop."""

    def __init__(self, controller: MissionController | None = None, state_dir: Path | None = None) -> None:
        self.controller = controller or MissionController()
        self.runtime = DryRunRuntime(self.controller)
        self.state_dir = Path(state_dir) if state_dir is not None else None
        self.lock = threading.RLock()
        self._persist()

    def _persist(self) -> None:
        if self.state_dir is None:
            return
        self.state_dir.mkdir(parents=True, exist_ok=True)
        snapshot = self.controller.snapshot()
        temporary = self.state_dir / "snapshot.json.tmp"
        temporary.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.state_dir / "snapshot.json")
        (self.state_dir / "events.jsonl").write_text(
            "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in self.controller.events),
            encoding="utf-8",
        )

    def state(self) -> dict:
        with self.lock:
            return self.controller.snapshot()

    def submit(self, payload: dict) -> tuple[dict, bool]:
        with self.lock:
            order, created = self.controller.submit(payload)
            self._persist()
            return {"order": order.__dict__, "state": self.controller.snapshot()}, created

    def cancel(self, order_id: str) -> dict:
        with self.lock:
            order = self.controller.cancel(order_id)
            self._persist()
            return {"order": order.__dict__, "state": self.controller.snapshot()}

    def step(self, *, success: bool = True, failure: str | None = None) -> dict:
        with self.lock:
            command = self.runtime.step(success=success, failure=failure)
            self._persist()
            return {"executed_command": command, "state": self.controller.snapshot()}


def handler_factory(app: ServiceApplication) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "HoldTheFlowOrderServer/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"{self.address_string()} - {fmt % args}")

        def _json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            length_text = self.headers.get("Content-Length")
            if length_text is None:
                raise ValueError("Content-Length is required")
            try:
                length = int(length_text)
            except ValueError as exc:
                raise ValueError("Content-Length must be an integer") from exc
            if not 0 < length <= 16_384:
                raise ValueError("request body must be 1-16384 bytes")
            try:
                payload = json.loads(self.rfile.read(length))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("request body must be UTF-8 JSON") from exc
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            return payload

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/api/health":
                self._json(HTTPStatus.OK, {"status": "ok", "mode": "simulation_dry_run"})
                return
            if path == "/api/state":
                self._json(HTTPStatus.OK, app.state())
                return
            if path in {"/", "/index.html"}:
                body = INDEX.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                if path == "/api/orders":
                    result, created = app.submit(self._read_json())
                    self._json(HTTPStatus.CREATED if created else HTTPStatus.OK, result)
                    return
                if path == "/api/step":
                    payload = self._read_json()
                    success = payload.get("success", True)
                    if not isinstance(success, bool):
                        raise ValueError("success must be boolean")
                    failure = payload.get("failure")
                    if failure is not None and (not isinstance(failure, str) or not failure):
                        raise ValueError("failure must be a nonempty string")
                    self._json(HTTPStatus.OK, app.step(success=success, failure=failure))
                    return
                match = ORDER_PATH.fullmatch(path)
                if match:
                    self._json(HTTPStatus.OK, app.cancel(match.group(1)))
                    return
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            except KeyError as exc:
                self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_order", "detail": str(exc)})
            except ValueError as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_request", "detail": str(exc)})

    return Handler


def auto_step(app: ServiceApplication, interval_s: float, stop: threading.Event) -> None:
    while not stop.wait(interval_s):
        state = app.state()
        if state["phase"] != "IDLE_AT_DOCK" or state["queue"] or state["active_order_id"]:
            app.step()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--state-dir", type=Path, default=Path("/tmp/bimanual-service-order-server"))
    parser.add_argument("--auto-step-s", type=float, default=0.75)
    parser.add_argument("--battery-percent", type=float, default=100.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not 0 < args.port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    if args.auto_step_s <= 0:
        raise ValueError("auto-step-s must be positive")
    app = ServiceApplication(MissionController(battery_percent=args.battery_percent), args.state_dir)
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(app))
    stop = threading.Event()
    worker = threading.Thread(target=auto_step, args=(app, args.auto_step_s, stop), daemon=True)
    worker.start()
    print(f"service order dashboard: http://{args.host}:{server.server_port}", flush=True)
    print("mode: simulation_dry_run (no robot, ROS 2, camera, or Isaac Sim access)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        worker.join(timeout=max(1.0, args.auto_step_s * 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
