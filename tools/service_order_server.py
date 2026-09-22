#!/usr/bin/env python3
"""Local web order API for the CPU-only service mission controller."""
from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse

from service_mission_control import DryRunRuntime, MissionController
from service_order_store import ServiceOrderStore


ROOT = Path(__file__).resolve().parents[1]
INDEX = Path(__file__).with_name("service_order_dashboard") / "index.html"
DASHBOARD_JS = INDEX.with_name("dashboard.js")
ORDER_PATH = re.compile(r"^/api/orders/([A-Za-z0-9._:-]{1,64})/cancel$")


def default_state_dir() -> Path:
    state_home = os.environ.get("XDG_STATE_HOME")
    base = Path(state_home).expanduser() if state_home else Path.home() / ".local/state"
    return base / "bimanual-robot/service-order-server"


class ServiceApplication:
    """Thread-safe application boundary shared by HTTP and the dry-run loop."""

    def __init__(self, controller: MissionController | None = None, state_dir: Path | None = None) -> None:
        self.state_dir = Path(state_dir) if state_dir is not None else None
        self.lock = threading.RLock()
        self.store = ServiceOrderStore(self.state_dir) if self.state_dir is not None else None
        persisted = self.store.load_snapshot() if self.store is not None else None
        if persisted is not None:
            source = controller or MissionController()
            self.controller = MissionController.from_snapshot(
                persisted,
                config=source.config,
                layout=source.layout,
                clock=source.clock,
            )
        else:
            self.controller = controller or MissionController()
        self.runtime = DryRunRuntime(self.controller)
        self._persist()

    def _persist(self, command: dict | None = None) -> None:
        if self.state_dir is None:
            return
        self.state_dir.mkdir(parents=True, exist_ok=True)
        snapshot = self.controller.snapshot()
        if self.store is not None:
            self.store.persist(snapshot, command)
        temporary = self.state_dir / "snapshot.json.tmp"
        temporary.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.state_dir / "snapshot.json")
        event_temporary = self.state_dir / "events.jsonl.tmp"
        event_temporary.write_text(
            "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in self.controller.events),
            encoding="utf-8",
        )
        event_temporary.replace(self.state_dir / "events.jsonl")

    def state(self) -> dict:
        with self.lock:
            snapshot = self.controller.snapshot()
            snapshot["stats"] = self._stats(snapshot)
            snapshot["persistence"] = (
                self.store.info() if self.store is not None else {"enabled": False, "backend": None}
            )
            snapshot["restaurant"] = {
                "room_bounds_m": self.controller.layout["room_bounds_m"],
                "kitchen": self.controller.layout["kitchen"],
                "tables": self.controller.layout["tables"],
                "waypoints": self.controller.layout["waypoints"],
            }
            return snapshot

    def _stats(self, snapshot: dict) -> dict:
        if self.store is not None:
            return self.store.stats()
        counts = {state: 0 for state in ("QUEUED", "RUNNING", "SUCCEEDED", "FAILED", "CANCELED")}
        for order in snapshot["orders"]:
            counts[order["state"]] += 1
        return {
            "total": len(snapshot["orders"]),
            "queued": counts["QUEUED"],
            "running": counts["RUNNING"],
            "succeeded": counts["SUCCEEDED"],
            "failed": counts["FAILED"],
            "canceled": counts["CANCELED"],
            "events": len(snapshot["events"]),
            "commands": len(self.runtime.commands),
        }

    def history(self, kind: str, limit: int) -> list[dict]:
        with self.lock:
            if self.store is None:
                if kind == "orders":
                    return list(reversed(self.controller.snapshot()["orders"]))[:limit]
                return self.controller.events[-limit:]
            return self.store.order_history(limit) if kind == "orders" else self.store.event_history(limit)

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
            phase_before = self.controller.phase
            order_before = self.controller.active_order_id
            command = self.runtime.step(success=success, failure=failure)
            stored_command = dict(command)
            stored_command.setdefault("phase", phase_before)
            stored_command.setdefault("order_id", order_before)
            self._persist(stored_command)
            return {"executed_command": command, "state": self.controller.snapshot()}

    def close(self) -> None:
        if self.store is not None:
            self.store.close()


def handler_factory(app: ServiceApplication) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "HoldTheFlowOrderServer/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            if urlparse(self.path).path == "/api/state":
                return
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
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/api/health":
                self._json(HTTPStatus.OK, {"status": "ok", "mode": "simulation_dry_run"})
                return
            if path == "/api/state":
                self._json(HTTPStatus.OK, app.state())
                return
            if path in {"/api/orders/history", "/api/events"}:
                try:
                    raw_limit = parse_qs(parsed.query).get("limit", ["100"])[0]
                    limit = int(raw_limit)
                    if not 1 <= limit <= 1000:
                        raise ValueError
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "limit_must_be_1_to_1000"})
                    return
                kind = "orders" if path == "/api/orders/history" else "events"
                self._json(HTTPStatus.OK, {kind: app.history(kind, limit)})
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
            if path == "/dashboard.js":
                body = DASHBOARD_JS.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/javascript; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
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
    parser.add_argument("--state-dir", type=Path, default=default_state_dir())
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
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
