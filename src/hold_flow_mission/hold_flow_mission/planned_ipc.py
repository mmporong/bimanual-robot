"""Unix-socket protocol shared by ROS 2 and the Isaac Sim executor."""
from __future__ import annotations

import json
import math
from pathlib import Path
import socket


PROTOCOL = "planned_executor_ipc_v1"
MAX_MESSAGE_BYTES = 64 * 1024
MANIPULATION_PHASES = (
    "ALIGN_KITCHEN",
    "GRASP_CUP",
    "GRASP_BOTTLE",
    "POUR",
    "RETURN_BOTTLE",
    "PLACE_DECK",
    "ALIGN_TABLE",
    "REGRASP_CUP",
    "SERVE",
)


class PlannedIpcError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _read_message(stream: object) -> dict:
    raw = stream.readline(MAX_MESSAGE_BYTES + 1)
    if not raw:
        raise PlannedIpcError("IPC_EMPTY_RESPONSE", "executor returned no response")
    if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
        raise PlannedIpcError("IPC_MESSAGE_TOO_LARGE", "executor response exceeds limit")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PlannedIpcError("IPC_JSON_INVALID", f"invalid executor JSON: {exc}") from exc
    if not isinstance(value, dict) or value.get("protocol") != PROTOCOL:
        raise PlannedIpcError("IPC_PROTOCOL_ERROR", "executor protocol mismatch")
    return value


def call_executor(socket_path: Path, payload: dict, *, timeout_sec: float) -> dict:
    """Send one request and read one bounded response."""
    if (
        isinstance(timeout_sec, bool)
        or not isinstance(timeout_sec, (int, float))
        or not math.isfinite(timeout_sec)
        or timeout_sec <= 0.0
    ):
        raise ValueError("timeout_sec must be a positive finite number")
    request = {"protocol": PROTOCOL, **payload}
    encoded = json.dumps(request, separators=(",", ":")).encode() + b"\n"
    if len(encoded) > MAX_MESSAGE_BYTES:
        raise PlannedIpcError("IPC_MESSAGE_TOO_LARGE", "executor request exceeds limit")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_sec)
    try:
        client.connect(str(Path(socket_path).expanduser()))
        client.sendall(encoded)
        with client.makefile("rb") as stream:
            return _read_message(stream)
    except FileNotFoundError as exc:
        raise PlannedIpcError("IPC_UNAVAILABLE", f"executor socket is missing: {socket_path}") from exc
    except ConnectionRefusedError as exc:
        raise PlannedIpcError("IPC_UNAVAILABLE", f"executor refused connection: {socket_path}") from exc
    except socket.timeout as exc:
        raise PlannedIpcError("IPC_TIMEOUT", "executor response timed out") from exc
    except OSError as exc:
        raise PlannedIpcError("IPC_IO_ERROR", f"executor communication failed: {exc}") from exc
    finally:
        client.close()


def execute_phase(
    socket_path: Path,
    *,
    request_id: str,
    mission_id: str,
    order_id: str,
    phase_id: str,
    table_id: str,
    drink: str,
    timeout_sec: float,
) -> dict:
    response = call_executor(
        socket_path,
        {
            "op": "execute_phase",
            "request_id": request_id,
            "mission_id": mission_id,
            "order_id": order_id,
            "phase_id": phase_id,
            "table_id": table_id,
            "drink": drink,
            "timeout_sec": timeout_sec,
        },
        timeout_sec=timeout_sec + 1.0,
    )
    for key in ("success", "failure_code", "message", "session_state"):
        if key not in response:
            raise PlannedIpcError("IPC_PROTOCOL_ERROR", f"executor response lacks {key}")
    if not isinstance(response["success"], bool):
        raise PlannedIpcError("IPC_PROTOCOL_ERROR", "executor success must be boolean")
    return response


def cancel_session(socket_path: Path, mission_id: str, *, timeout_sec: float = 1.0) -> bool:
    response = call_executor(
        socket_path,
        {"op": "cancel_session", "mission_id": mission_id},
        timeout_sec=timeout_sec,
    )
    return response.get("canceled") is True
