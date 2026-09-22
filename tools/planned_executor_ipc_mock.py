#!/usr/bin/env python3
"""Stateful Unix-socket mock for the future Isaac Sim PLANNED executor."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import socketserver
import threading
import time

from hold_flow_mission.planned_ipc import MANIPULATION_PHASES, MAX_MESSAGE_BYTES, PROTOCOL


@dataclass
class Session:
    mission_id: str
    order_id: str
    table_id: str
    drink: str
    next_index: int = 0
    canceled: threading.Event = field(default_factory=threading.Event)
    responses: dict[str, dict] = field(default_factory=dict)
    inflight_request_id: str | None = None


class SessionCoordinator:
    """Enforce one ordered manipulation session per mission."""

    def __init__(self, *, phase_delay_sec: float = 0.02) -> None:
        if not math.isfinite(phase_delay_sec) or phase_delay_sec < 0.0:
            raise ValueError("phase_delay_sec must be a non-negative finite number")
        self.phase_delay_sec = phase_delay_sec
        self.sessions: dict[str, Session] = {}
        self.lock = threading.Lock()

    @staticmethod
    def _response(**values: object) -> dict:
        return {"protocol": PROTOCOL, "simulator_accessed": False,
                "hardware_accessed": False, **values}

    def _failure(self, code: str, message: str, *, state: str = "FAILED") -> dict:
        return self._response(
            success=False,
            failure_code=code,
            message=message,
            session_state=state,
        )

    def cancel(self, mission_id: object) -> dict:
        if not isinstance(mission_id, str) or not mission_id:
            return self._failure("MISSION_ID_INVALID", "mission_id is required")
        with self.lock:
            session = self.sessions.get(mission_id)
            if session is None:
                return self._response(canceled=False, success=False,
                                      failure_code="SESSION_NOT_FOUND",
                                      message="session does not exist",
                                      session_state="MISSING")
            session.canceled.set()
        return self._response(canceled=True, success=True, failure_code="",
                              message="session cancellation requested",
                              session_state="CANCELING")

    def execute(self, request: dict) -> dict:
        required = ("request_id", "mission_id", "order_id", "phase_id", "table_id", "drink")
        if any(not isinstance(request.get(key), str) or not request[key] for key in required):
            return self._failure("REQUEST_INVALID", "required identifiers must be nonempty strings")
        request_id = request["request_id"]
        mission_id = request["mission_id"]
        phase_id = request["phase_id"]
        timeout_sec = request.get("timeout_sec")
        if (
            isinstance(timeout_sec, bool)
            or not isinstance(timeout_sec, (int, float))
            or not math.isfinite(timeout_sec)
            or timeout_sec <= 0.0
        ):
            return self._failure("REQUEST_INVALID", "timeout_sec must be positive and finite")
        if phase_id not in MANIPULATION_PHASES:
            return self._failure("PHASE_UNSUPPORTED", f"unsupported phase: {phase_id}")
        if request["table_id"] != "table_1":
            return self._failure("TABLE_UNSUPPORTED", "mock session covers table_1 only")
        if request["drink"] != "COLD_WATER":
            return self._failure("DRINK_UNSUPPORTED", "mock session covers COLD_WATER only")

        with self.lock:
            session = self.sessions.get(mission_id)
            if session is None:
                if phase_id != MANIPULATION_PHASES[0]:
                    return self._failure(
                        "SESSION_NOT_STARTED",
                        f"first phase must be {MANIPULATION_PHASES[0]}",
                    )
                session = Session(
                    mission_id=mission_id,
                    order_id=request["order_id"],
                    table_id=request["table_id"],
                    drink=request["drink"],
                )
                self.sessions[mission_id] = session
            if request_id in session.responses:
                return session.responses[request_id]
            if session.inflight_request_id is not None:
                return self._failure(
                    "SESSION_BUSY",
                    f"request {session.inflight_request_id} is still running",
                    state="RUNNING",
                )
            if (
                session.order_id != request["order_id"]
                or session.table_id != request["table_id"]
                or session.drink != request["drink"]
            ):
                return self._failure("SESSION_SCOPE_MISMATCH", "session identity changed")
            if session.canceled.is_set():
                return self._failure("CANCELED", "session was canceled", state="CANCELED")
            if session.next_index >= len(MANIPULATION_PHASES):
                return self._failure("SESSION_ALREADY_COMPLETE", "session already completed")
            expected = MANIPULATION_PHASES[session.next_index]
            if phase_id != expected:
                return self._failure(
                    "PHASE_ORDER_MISMATCH",
                    f"expected {expected}, received {phase_id}",
                    state="WAITING",
                )
            session.inflight_request_id = request_id

        started = time.monotonic()
        phase_deadline = started + self.phase_delay_sec
        timeout_deadline = started + timeout_sec
        while time.monotonic() < phase_deadline:
            remaining = min(0.01, phase_deadline - time.monotonic(), timeout_deadline - time.monotonic())
            if remaining <= 0.0:
                response = self._failure("TIMEOUT", "phase exceeded timeout_sec", state="FAILED")
                with self.lock:
                    session.inflight_request_id = None
                    session.responses[request_id] = response
                return response
            if session.canceled.wait(timeout=remaining):
                response = self._failure("CANCELED", "session canceled during phase",
                                         state="CANCELED")
                with self.lock:
                    session.inflight_request_id = None
                    session.responses[request_id] = response
                return response

        with self.lock:
            session.inflight_request_id = None
            if session.canceled.is_set():
                response = self._failure("CANCELED", "session canceled during phase",
                                         state="CANCELED")
            else:
                session.next_index += 1
                complete = session.next_index == len(MANIPULATION_PHASES)
                response = self._response(
                    success=True,
                    failure_code="",
                    message="stateful executor IPC mock completed phase",
                    session_state="COMPLETE" if complete else "WAITING",
                    completed_phase=phase_id,
                    next_phase="" if complete else MANIPULATION_PHASES[session.next_index],
                )
            session.responses[request_id] = response
            return response

    def handle(self, request: object) -> dict:
        if not isinstance(request, dict) or request.get("protocol") != PROTOCOL:
            return self._failure("PROTOCOL_ERROR", "protocol mismatch")
        if request.get("op") == "execute_phase":
            return self.execute(request)
        if request.get("op") == "cancel_session":
            return self.cancel(request.get("mission_id"))
        if request.get("op") == "status":
            with self.lock:
                count = len(self.sessions)
            return self._response(success=True, failure_code="", message="ready",
                                  session_state="READY", session_count=count)
        return self._failure("OP_UNSUPPORTED", "unsupported operation")


class Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        raw = self.rfile.readline(MAX_MESSAGE_BYTES + 1)
        if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
            response = self.server.coordinator._failure(
                "MESSAGE_TOO_LARGE", "request exceeds limit"
            )
        else:
            try:
                request = json.loads(raw)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                response = self.server.coordinator._failure("JSON_INVALID", str(exc))
            else:
                response = self.server.coordinator.handle(request)
        self.wfile.write(json.dumps(response, separators=(",", ":")).encode() + b"\n")


class ThreadingUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, path: str, coordinator: SessionCoordinator) -> None:
        self.coordinator = coordinator
        super().__init__(path, Handler)


def serve(socket_path: Path, *, phase_delay_sec: float) -> None:
    socket_path = Path(socket_path).expanduser()
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    if socket_path.exists():
        if not socket_path.is_socket():
            raise RuntimeError(f"refusing to replace non-socket path: {socket_path}")
        socket_path.unlink()
    coordinator = SessionCoordinator(phase_delay_sec=phase_delay_sec)
    try:
        with ThreadingUnixServer(str(socket_path), coordinator) as server:
            print(f"planned executor IPC mock: {socket_path}", flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
    finally:
        if socket_path.exists() and socket_path.is_socket():
            socket_path.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--phase-delay-s", type=float, default=0.02)
    args = parser.parse_args()
    serve(args.socket, phase_delay_sec=args.phase_delay_s)


if __name__ == "__main__":
    main()
