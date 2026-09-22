"""Single-world phase gate. Only the simulation thread may advance physics.

Socket threads submit requests; they never touch USD, PhysX or an articulation.
Waiting freezes simulation time, not just joint targets. One process owns one
mission; a completed/failed world cannot be reused without explicit reset.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
from pathlib import Path
import socket
import socketserver
import threading
import time

from hold_flow_mission.planned_ipc import MANIPULATION_PHASES, MAX_MESSAGE_BYTES, PROTOCOL


def phase_for_tick(state: str, pose: str, time_s: float) -> str:
    if time_s < 2.0:
        return "ALIGN_KITCHEN"
    if state == "POUR":
        if pose.startswith("LEFT_"):
            return "GRASP_CUP"
        if pose.startswith("POUR_"):
            return "POUR"
        if pose in {"RIGHT_LOWER", "RIGHT_TABLE_SETTLE", "RIGHT_OPEN",
                    "RIGHT_RELEASE_HOLD", "RIGHT_WITHDRAW", "RIGHT_CLEAR_ABOVE",
                    "RIGHT_PLACE_HOLD"}:
            return "RETURN_BOTTLE"
        return "GRASP_BOTTLE"
    return {"DEPOSIT": "PLACE_DECK", "BACKOUT": "ALIGN_TABLE",
            "NAVIGATE": "ALIGN_TABLE", "DOCK": "ALIGN_TABLE",
            "SETTLE_BASE": "ALIGN_TABLE", "REGRASP": "REGRASP_CUP",
            "SERVE": "SERVE"}[state]


class PhaseExecutor:
    def __init__(self, output: Path, *, idle_timeout_sec: float = 120.0):
        if not math.isfinite(idle_timeout_sec) or idle_timeout_sec <= 0:
            raise ValueError("idle timeout must be positive and finite")
        self.output = output
        self.idle_timeout = idle_timeout_sec
        self.condition = threading.Condition()
        self.identity = None
        self.index = 0
        self.active = None
        self.running_phase = None
        self.deadline = None
        self.stop_reason = ""
        self.finished = False
        self.requests = {}
        self.responses = {}

    def _response(self, **values):
        return dict(protocol=PROTOCOL, hardware_accessed=False, simulator_accessed=True,
                    executor_kind="isaac_physics", **values)

    def _failure(self, code, message, **values):
        return self._response(success=False, failure_code=code, message=message,
                              session_state="FAILED", **values)

    def handle(self, request):
        if not isinstance(request, dict) or request.get("protocol") != PROTOCOL:
            return self._failure("PROTOCOL_ERROR", "protocol mismatch")
        with self.condition:
            if request.get("op") == "status":
                return self._response(success=True, failure_code="", message="Isaac world ready",
                    session_state="FINISHED" if self.finished else "RUNNING" if self.active else "WAITING",
                    phase=self.running_phase, next_phase=MANIPULATION_PHASES[self.index] if self.index < len(MANIPULATION_PHASES) else "",
                    stop_reason=self.stop_reason, mission_id=self.identity[0] if self.identity else None)
            if request.get("op") == "cancel_session":
                if self.identity is None or request.get("mission_id") != self.identity[0]:
                    return self._failure("SESSION_NOT_FOUND", "mission does not own this world", canceled=False)
                self.stop_reason = self.stop_reason or "CANCELED"
                self.condition.notify_all()
                # This acknowledges a request, not a physical stop.
                return self._response(success=True, canceled=True, failure_code="",
                    message="cancellation requested; await phase result for stopped world",
                    session_state="CANCELING", stop_confirmed=self.finished)
            if request.get("op") != "execute_phase":
                return self._failure("OP_UNSUPPORTED", "unsupported operation")
            keys = ("request_id", "mission_id", "order_id", "phase_id", "table_id", "drink")
            if any(not isinstance(request.get(k), str) or not request[k] for k in keys):
                return self._failure("REQUEST_INVALID", "identifiers must be nonempty strings")
            timeout = request.get("timeout_sec")
            if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
                return self._failure("REQUEST_INVALID", "timeout must be positive and finite")
            if request["table_id"] != "table_1" or request["drink"] != "COLD_WATER":
                return self._failure("SCOPE_UNSUPPORTED", "this scene supports table_1/COLD_WATER only")
            identity = tuple(request[k] for k in ("mission_id", "order_id", "table_id", "drink"))
            if self.identity is not None and identity != self.identity:
                return self._failure("WORLD_OWNED", "one mission per world; restart after completion")
            rid = request["request_id"]
            if rid in self.requests:
                if self.requests[rid] != request:
                    return self._failure("REQUEST_ID_CONFLICT", "request ID reused with a different payload")
                if rid in self.responses:
                    return self.responses[rid]
                return self._failure("SESSION_BUSY", "request still running")
            if self.finished or self.stop_reason:
                return self._failure(self.stop_reason or "SESSION_COMPLETE", "world cannot be resumed")
            if self.active is not None:
                return self._failure("SESSION_BUSY", "another phase is running")
            if request["phase_id"] != MANIPULATION_PHASES[self.index]:
                return self._failure("PHASE_ORDER_MISMATCH", f"expected {MANIPULATION_PHASES[self.index]}")
            self.identity = identity
            self.requests[rid] = dict(request)
            self.active = request
            self.deadline = time.monotonic() + timeout
            self.condition.notify_all()
            while rid not in self.responses:
                remaining = self.deadline - time.monotonic()
                if remaining <= 0:
                    self.stop_reason = self.stop_reason or "TIMEOUT"
                    self.condition.notify_all()
                    # Allow the simulation owner to stop and publish evidence.
                    self.condition.wait_for(lambda: rid in self.responses, timeout=0.5)
                    if rid not in self.responses:
                        return self._failure(self.stop_reason, "stop requested; owner acknowledgment pending",
                                             stop_confirmed=False)
                    break
                self.condition.wait(timeout=min(remaining, 0.1))
            return self.responses[rid]

    def _complete(self, *, failure="", observation=None):
        if self.active is None:
            return
        request = self.active
        response = self._response(success=not failure, failure_code=failure,
            message="Isaac physics phase completed" if not failure else "Isaac execution stopped: " + failure,
            session_state="FAILED" if failure else "COMPLETE" if self.finished else "WAITING",
            request_id=request["request_id"], mission_id=request["mission_id"], order_id=request["order_id"],
            completed_phase=request["phase_id"], observation=observation or {},
            stop_confirmed=self.finished, result_path=str(self.output / "result.json"))
        with (self.output / "executor_phases.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(response) + "\n")
        self.responses[request["request_id"]] = response
        self.active = None
        if not failure:
            self.index += 1
        self.condition.notify_all()

    def checkpoint(self, phase, observation, *, boundary_failure=""):
        """Called before a physics step. Returns a stop reason or an empty string."""
        with self.condition:
            if self.stop_reason:
                return self.stop_reason
            if self.active and time.monotonic() >= self.deadline:
                self.stop_reason = "TIMEOUT"
                return self.stop_reason
            if phase == self.running_phase:
                return ""
            if boundary_failure:
                self.stop_reason = boundary_failure
                return self.stop_reason
            if self.running_phase is not None:
                self._complete(observation=observation)
            waiting_until = time.monotonic() + self.idle_timeout
            while self.active is None and not self.stop_reason:
                remaining = waiting_until - time.monotonic()
                if remaining <= 0:
                    self.stop_reason = "EXECUTOR_IDLE_TIMEOUT"
                    break
                self.condition.wait(timeout=min(remaining, 0.1))
            if self.stop_reason:
                return self.stop_reason
            if self.active["phase_id"] != phase:
                self.stop_reason = "PHYSICS_PHASE_MISMATCH"
                return self.stop_reason
            self.running_phase = phase
            return ""

    def finish(self, success, reason, observation=None, *, persist_result=None):
        with self.condition:
            self.finished = True
            if self.active and time.monotonic() >= self.deadline:
                self.stop_reason = self.stop_reason or 'TIMEOUT'
            failure = self.stop_reason or ("" if success else reason)
            if failure:
                self.stop_reason = failure
            if persist_result is not None:
                if failure:
                    persist_result.update(task_pass=False, failure=failure)
                try:
                    (self.output / 'result.json').write_text(json.dumps(persist_result, indent=2) + '\n')
                except OSError:
                    failure = self.stop_reason = 'EVIDENCE_WRITE_FAILED'
                    persist_result.update(task_pass=False, failure=failure)
            self._complete(failure=failure, observation=observation)
            self.condition.notify_all()


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(5.0)
        try:
            raw = self.rfile.readline(MAX_MESSAGE_BYTES + 1)
            if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
                response = self.server.executor._failure("MESSAGE_TOO_LARGE", "invalid framing")
            else:
                try:
                    response = self.server.executor.handle(json.loads(raw))
                except (ValueError, UnicodeError):
                    response = self.server.executor._failure("JSON_INVALID", "invalid JSON")
            self.wfile.write(json.dumps(response).encode() + b"\n")
        except (socket.timeout, BrokenPipeError, ConnectionResetError):
            return


class _Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


@contextmanager
def serve_executor(path, output, *, idle_timeout_sec=120.0):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    executor = PhaseExecutor(output, idle_timeout_sec=idle_timeout_sec)
    # Binding is exclusive: never unlink another live or stale socket implicitly.
    server = _Server(str(path), _Handler)
    identity = path.stat().st_ino
    path.chmod(0o600)
    server.executor = executor
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield executor
    finally:
        if not executor.finished:
            executor.finish(False, "EXECUTOR_CLOSED")
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
        if path.exists() and path.stat().st_ino == identity:
            path.unlink()
