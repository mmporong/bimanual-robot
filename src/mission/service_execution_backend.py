#!/usr/bin/env python3
"""Execution backends for service mission commands.

The default backend preserves the CPU-only dry-run.  The ROS 2 backend sends
only manipulation commands to ExecuteManipulationSkill; navigation and charge
remain explicit immediate mocks until their adapters are implemented.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import threading
from typing import Protocol


def _planned_ipc_module():
    """Load ROS-workspace IPC only when the roundtrip backend is selected."""
    from hold_flow_mission import planned_ipc

    return planned_ipc


def call_executor(*args, **kwargs):
    return _planned_ipc_module().call_executor(*args, **kwargs)


def execute_phase(*args, **kwargs):
    return _planned_ipc_module().execute_phase(*args, **kwargs)


def cancel_session(*args, **kwargs):
    return _planned_ipc_module().cancel_session(*args, **kwargs)


@dataclass(frozen=True)
class BackendResult:
    success: bool
    failure_code: str = ""
    adapter: str = "simulation_immediate"
    action_result: dict | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class CommandBackend(Protocol):
    mode: str

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult: ...

    def cancel_active(self) -> bool: ...

    def info(self) -> dict: ...

    def close(self) -> None: ...


class ImmediateBackend:
    """Marks every command successful without accessing ROS or hardware."""

    mode = "simulation_dry_run"

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult:
        del command, mission_id, phase_attempt
        return BackendResult(success=True)

    def cancel_active(self) -> bool:
        return False

    def info(self) -> dict:
        return {
            "mode": self.mode,
            "manipulation": "immediate_success",
            "navigation": "immediate_success",
            "charge": "immediate_success",
            "active_goal": False,
            "hardware_accessed": False,
        }

    def close(self) -> None:
        return None


class Ros2ManipulationBackend:
    """Routes manipulation to ROS 2 and keeps other commands as mocks."""

    mode = "ros2_manipulation_mock"

    def __init__(self, *, timeout_sec: float = 30.0) -> None:
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be a positive finite number")
        try:
            import rclpy
            from rclpy.context import Context

            from hold_flow_mission.action_client import ManipulationActionClient
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 manipulation backend requires sourced Jazzy and install/setup.bash"
            ) from exc
        self.timeout_sec = float(timeout_sec)
        self._rclpy = rclpy
        self._context = Context()
        rclpy.init(context=self._context)
        self._client = ManipulationActionClient(context=self._context)

    def execute(
        self,
        command: dict,
        *,
        mission_id: str | None,
        phase_attempt: int,
    ) -> BackendResult:
        if command.get("kind") != "manipulate":
            return BackendResult(
                success=True,
                adapter="simulation_immediate_non_manipulation",
            )
        if not mission_id:
            return BackendResult(False, "MISSION_ID_MISSING", self.mode)

        from hold_flow_mission.contract import ManipulationGoalSpec

        phase = str(command.get("phase", "UNKNOWN"))
        request_id = f"{mission_id}:{phase}:{phase_attempt}"
        try:
            spec = ManipulationGoalSpec.from_mission_command(
                command,
                request_id=request_id,
                mission_id=mission_id,
                timeout_sec=self.timeout_sec,
            )
            action_result = self._client.execute(spec)
        except RuntimeError as exc:
            return BackendResult(
                False,
                "ACTION_SERVER_UNAVAILABLE",
                self.mode,
                {"message": str(exc)},
            )
        except Exception as exc:
            return BackendResult(
                False,
                "BACKEND_EXCEPTION",
                self.mode,
                {"message": f"{type(exc).__name__}: {exc}"},
            )
        if not action_result.get("accepted"):
            return BackendResult(False, "GOAL_REJECTED", self.mode, action_result)
        if (
            action_result.get("mission_id") != mission_id
            or action_result.get("order_id") != command.get("order_id")
            or action_result.get("request_id") != request_id
        ):
            return BackendResult(False, "RESULT_ID_MISMATCH", self.mode, action_result)
        if action_result.get("success"):
            return BackendResult(True, adapter=self.mode, action_result=action_result)
        failure_code = str(action_result.get("failure_code") or "ACTION_ABORTED")
        return BackendResult(False, failure_code, self.mode, action_result)

    def cancel_active(self) -> bool:
        return self._client.cancel_active()

    def info(self) -> dict:
        return {
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill",
            "navigation": "immediate_success_mock",
            "charge": "immediate_success_mock",
            "timeout_sec": self.timeout_sec,
            "active_goal": self._client.has_active_goal(),
            "hardware_accessed": False,
        }

    def close(self) -> None:
        self._client.destroy_node()
        if self._context.ok():
            self._context.shutdown()


class Ros2PlannedArtifactBackend(Ros2ManipulationBackend):
    """Labels the shared Action client as verified artifact replay."""

    mode = "ros2_planned_artifact_replay"

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill (artifact replay)",
            "simulator_accessed": False,
        })
        return info


class Ros2PlannedSessionBackend(Ros2ManipulationBackend):
    """Labels the shared Action client as a persistent executor session."""

    mode = "ros2_planned_session"

    def execute(self, command: dict, *, mission_id: str | None, phase_attempt: int) -> BackendResult:
        result = super().execute(command, mission_id=mission_id, phase_attempt=phase_attempt)
        if result.action_result:
            try:
                evidence = json.loads(result.action_result.get('message', ''))
            except (TypeError, ValueError):
                evidence = None
            if isinstance(evidence, dict):
                self._executor_evidence = evidence
        return result

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "manipulation": "/execute_manipulation_skill (persistent IPC session)",
            "executor_transport": "unix_socket_json",
            "hardware_accessed": False,
            "simulator_accessed": getattr(self, '_executor_evidence', {}).get('simulator_accessed'),
            "executor_kind": getattr(self, '_executor_evidence', {}).get('executor_kind', 'unconfirmed'),
            "scope": "table_1 cold water; dock navigation and charging are mocks",
        })
        return info


class Ros2PlannedRoundtripBackend(Ros2PlannedSessionBackend):
    """Runs navigation in the same Isaac world as ROS Action manipulation."""

    mode = "ros2_planned_roundtrip"
    scope = "dock_roundtrip"

    def __init__(self, *, executor_socket: Path, timeout_sec: float = 30.0) -> None:
        self.executor_socket = Path(executor_socket).expanduser()
        self._identity: dict[str, str] | None = None
        self._identity_lock = threading.Lock()
        self._executor_evidence = self._validate_executor(timeout_sec)
        self._executor_id = self._executor_evidence["executor_id"]
        super().__init__(timeout_sec=timeout_sec)

    def _validate_executor(self, timeout_sec: float) -> dict:
        try:
            status = call_executor(
                self.executor_socket,
                {"op": "status"},
                timeout_sec=min(float(timeout_sec), 5.0),
            )
        except (OSError, RuntimeError) as exc:
            code = getattr(exc, "code", "IPC_UNAVAILABLE")
            raise RuntimeError(f"roundtrip executor status failed ({code}): {exc}") from exc
        expected = {
            "success": True,
            "executor_kind": "isaac_physics",
            "scope": self.scope,
            "simulator_accessed": True,
            "hardware_accessed": False,
        }
        mismatches = [key for key, value in expected.items() if status.get(key) != value]
        if mismatches:
            raise RuntimeError(
                "roundtrip executor provenance mismatch: " + ", ".join(mismatches)
            )
        if status.get("mission_id") is not None or status.get("session_state") != "WAITING":
            raise RuntimeError("roundtrip executor world is not fresh and waiting")
        if not isinstance(status.get("executor_id"), str) or not status["executor_id"]:
            raise RuntimeError("roundtrip executor provenance mismatch: executor_id")
        return status

    def validate_order(self, payload: dict, *, existing_order_ids: set[str]) -> None:
        order_id = payload.get("order_id")
        if payload.get("table_id") != "table_1" or payload.get("drink") != "COLD_WATER":
            raise ValueError("ros2-planned-roundtrip supports table_1/COLD_WATER only")
        if existing_order_ids and order_id not in existing_order_ids:
            raise ValueError("ros2-planned-roundtrip allows one order per Isaac world")

    def _remember_identity(self, command: dict, mission_id: str | None) -> dict[str, str] | None:
        with self._identity_lock:
            if mission_id:
                candidate = {
                    "mission_id": mission_id,
                    "order_id": str(command.get("order_id") or ""),
                    "table_id": str(command.get("table_id") or ""),
                    "drink": str(command.get("drink") or ""),
                }
                if all(candidate.values()):
                    if self._identity is not None and self._identity != candidate:
                        raise RuntimeError("roundtrip world identity changed")
                    self._identity = candidate
            return dict(self._identity) if self._identity is not None else None

    def execute(self, command: dict, *, mission_id: str | None, phase_attempt: int) -> BackendResult:
        try:
            identity = self._remember_identity(command, mission_id)
        except RuntimeError as exc:
            return BackendResult(False, "WORLD_IDENTITY_MISMATCH", self.mode, {"message": str(exc)})
        if command.get("kind") == "navigate":
            phase = str(command.get("phase") or "")
            if phase not in {"NAVIGATE_KITCHEN", "NAVIGATE_TABLE", "NAVIGATE_DOCK"}:
                return BackendResult(False, "NAVIGATION_PHASE_UNSUPPORTED", self.mode)
            if identity is None:
                return BackendResult(False, "MISSION_IDENTITY_MISSING", self.mode)
            request_id = f"{identity['mission_id']}:{phase}:{phase_attempt}"
            try:
                result = execute_phase(
                    self.executor_socket,
                    request_id=request_id,
                    mission_id=identity["mission_id"],
                    order_id=identity["order_id"],
                    phase_id=phase,
                    table_id=identity["table_id"],
                    drink=identity["drink"],
                    executor_id=self._executor_id,
                    timeout_sec=self.timeout_sec,
                )
            except Exception as exc:
                planned_ipc_error = _planned_ipc_module().PlannedIpcError
                if not isinstance(exc, planned_ipc_error):
                    raise
                detail = {"message": str(exc), "result_state": "unknown"}
                if exc.code == "IPC_TIMEOUT":
                    detail["cancel_requested"] = self._cancel_identity(identity)
                    return BackendResult(False, "IPC_TIMEOUT_RESULT_UNKNOWN", self.mode, detail)
                return BackendResult(False, exc.code, self.mode, detail)
            self._executor_evidence = result
            if result.get("success"):
                return BackendResult(True, adapter=self.mode, action_result=result)
            return BackendResult(
                False,
                str(result.get("failure_code") or "EXECUTOR_FAILED"),
                self.mode,
                result,
            )
        if command.get("kind") == "charge":
            return BackendResult(
                True,
                adapter="battery_model_only",
                action_result={"message": "charge is controller-model-only; no physical charging"},
            )
        if command.get("kind") == "manipulate":
            return super().execute(command, mission_id=mission_id, phase_attempt=phase_attempt)
        return BackendResult(False, "COMMAND_UNSUPPORTED", self.mode)

    def _cancel_identity(self, identity: dict[str, str]) -> bool:
        try:
            return cancel_session(
                self.executor_socket,
                identity["mission_id"],
                executor_id=self._executor_id,
                timeout_sec=1.0,
            )
        except (OSError, RuntimeError):
            return False

    def cancel_active(self) -> bool:
        action_requested = super().cancel_active()
        with self._identity_lock:
            identity = dict(self._identity) if self._identity is not None else None
        ipc_requested = self._cancel_identity(identity) if identity is not None else False
        return action_requested or ipc_requested

    def info(self) -> dict:
        info = super().info()
        info.update({
            "mode": self.mode,
            "navigation": "Isaac physics via planned executor IPC",
            "charge": "controller_model_only (not physical charging)",
            "executor_socket": str(self.executor_socket),
            "executor_kind": self._executor_evidence.get("executor_kind", "unconfirmed"),
            "executor_id": self._executor_id,
            "simulator_accessed": self._executor_evidence.get("simulator_accessed"),
            "scope": self.scope,
            "single_order_only": True,
            "supported_order": {"table_id": "table_1", "drink": "COLD_WATER"},
            "world_resume_supported": False,
        })
        return info
