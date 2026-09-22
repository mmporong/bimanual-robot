import threading
import time

import pytest

from planned_executor_ipc_mock import SessionCoordinator, ThreadingUnixServer
from hold_flow_mission.planned_ipc import (
    MANIPULATION_PHASES,
    PROTOCOL,
    PlannedIpcError,
    call_executor,
    execute_phase,
)


def request(phase, *, request_id=None, mission_id="MIS-1", table_id="table_1", drink="COLD_WATER"):
    return {
        "protocol": PROTOCOL,
        "op": "execute_phase",
        "request_id": request_id or f"REQ-{phase}",
        "mission_id": mission_id,
        "order_id": "ORD-1",
        "phase_id": phase,
        "table_id": table_id,
        "drink": drink,
        "timeout_sec": 1.0,
    }


def test_session_requires_ordered_phases_and_completes():
    coordinator = SessionCoordinator(phase_delay_sec=0.0)
    for index, phase in enumerate(MANIPULATION_PHASES):
        result = coordinator.handle(request(phase))
        assert result["success"] is True
        expected = "COMPLETE" if index == len(MANIPULATION_PHASES) - 1 else "WAITING"
        assert result["session_state"] == expected


def test_out_of_order_phase_is_rejected_without_advancing():
    coordinator = SessionCoordinator(phase_delay_sec=0.0)
    result = coordinator.handle(request("GRASP_CUP"))
    assert result["failure_code"] == "SESSION_NOT_STARTED"
    assert coordinator.handle(request("ALIGN_KITCHEN"))["success"] is True
    result = coordinator.handle(request("POUR"))
    assert result["failure_code"] == "PHASE_ORDER_MISMATCH"
    assert coordinator.handle(request("GRASP_CUP"))["success"] is True


def test_request_id_is_idempotent():
    coordinator = SessionCoordinator(phase_delay_sec=0.0)
    first = coordinator.handle(request("ALIGN_KITCHEN", request_id="REQ-SAME"))
    second = coordinator.handle(request("ALIGN_KITCHEN", request_id="REQ-SAME"))
    assert second == first
    assert coordinator.sessions["MIS-1"].next_index == 1


def test_cancel_interrupts_running_phase():
    coordinator = SessionCoordinator(phase_delay_sec=0.5)
    result = {}
    worker = threading.Thread(
        target=lambda: result.update(coordinator.handle(request("ALIGN_KITCHEN")))
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while "MIS-1" not in coordinator.sessions and time.monotonic() < deadline:
        time.sleep(0.005)
    canceled = coordinator.handle({
        "protocol": PROTOCOL,
        "op": "cancel_session",
        "mission_id": "MIS-1",
    })
    worker.join(timeout=1.0)
    assert canceled["canceled"] is True
    assert result["failure_code"] == "CANCELED"
    assert not worker.is_alive()


def test_executor_timeout_does_not_advance_phase():
    coordinator = SessionCoordinator(phase_delay_sec=0.2)
    payload = request("ALIGN_KITCHEN")
    payload["timeout_sec"] = 0.01
    result = coordinator.handle(payload)
    assert result["failure_code"] == "TIMEOUT"
    assert coordinator.sessions["MIS-1"].next_index == 0


def test_scope_is_explicitly_limited_until_isaac_executor_replaces_mock():
    coordinator = SessionCoordinator(phase_delay_sec=0.0)
    assert coordinator.handle(request("ALIGN_KITCHEN", table_id="table_2"))["failure_code"] == "TABLE_UNSUPPORTED"
    assert coordinator.handle(request("ALIGN_KITCHEN", drink="HOT_WATER"))["failure_code"] == "DRINK_UNSUPPORTED"


def test_concurrent_phase_request_is_rejected_as_session_busy():
    coordinator = SessionCoordinator(phase_delay_sec=0.3)
    first = {}
    worker = threading.Thread(
        target=lambda: first.update(coordinator.handle(request("ALIGN_KITCHEN")))
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while (
        "MIS-1" not in coordinator.sessions
        or coordinator.sessions["MIS-1"].inflight_request_id is None
    ) and time.monotonic() < deadline:
        time.sleep(0.005)
    second = coordinator.handle(request("ALIGN_KITCHEN", request_id="REQ-SECOND"))
    worker.join(timeout=1.0)
    assert second["failure_code"] == "SESSION_BUSY"
    assert first["success"] is True


def test_unix_socket_round_trip_crosses_process_protocol(tmp_path):
    socket_path = tmp_path / "executor.sock"
    coordinator = SessionCoordinator(phase_delay_sec=0.0)
    server = ThreadingUnixServer(str(socket_path), coordinator)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        result = execute_phase(
            socket_path,
            request_id="REQ-1",
            mission_id="MIS-1",
            order_id="ORD-1",
            phase_id="ALIGN_KITCHEN",
            table_id="table_1",
            drink="COLD_WATER",
            timeout_sec=1.0,
        )
        assert result["success"] is True
        assert result["next_phase"] == "GRASP_CUP"
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1.0)


def test_ipc_reports_missing_socket_and_rejects_invalid_timeout(tmp_path):
    with pytest.raises(PlannedIpcError) as missing:
        call_executor(tmp_path / "missing.sock", {"op": "status"}, timeout_sec=0.1)
    assert missing.value.code == "IPC_UNAVAILABLE"
    with pytest.raises(ValueError, match="positive finite"):
        call_executor(tmp_path / "missing.sock", {"op": "status"}, timeout_sec=float("nan"))
