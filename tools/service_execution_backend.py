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
from typing import Protocol


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
