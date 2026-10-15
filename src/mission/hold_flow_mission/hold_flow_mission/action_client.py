"""CLI client for the simulation-only manipulation Action contract."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from hold_flow_interfaces.action import ExecuteManipulationSkill

from .contract import ManipulationGoalSpec


ACTION_NAME = "/execute_manipulation_skill"


class ManipulationActionClient(Node):
    def __init__(self, *, context: object | None = None) -> None:
        super().__init__("manipulation_action_client", context=context)
        self.client = ActionClient(self, ExecuteManipulationSkill, ACTION_NAME)
        self.feedback: list[dict] = []
        self._goal_lock = threading.Lock()
        self._active_goal_handle: object | None = None
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)

    def destroy_node(self) -> None:
        self._executor.remove_node(self)
        self._executor.shutdown()
        super().destroy_node()

    def execute(
        self,
        spec: ManipulationGoalSpec,
        *,
        cancel_after_sec: float | None = None,
    ) -> dict:
        self.feedback = []
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server unavailable: {ACTION_NAME}")
        goal = spec.populate_action_goal(ExecuteManipulationSkill.Goal())
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        self._executor.spin_until_future_complete(send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"accepted": False, "feedback": self.feedback}
        with self._goal_lock:
            self._active_goal_handle = goal_handle

        timer: threading.Timer | None = None
        if cancel_after_sec is not None:
            timer = threading.Timer(cancel_after_sec, goal_handle.cancel_goal_async)
            timer.daemon = True
            timer.start()
        try:
            result_future = goal_handle.get_result_async()
            self._executor.spin_until_future_complete(result_future)
        finally:
            if timer is not None:
                timer.cancel()
            with self._goal_lock:
                if self._active_goal_handle is goal_handle:
                    self._active_goal_handle = None
        wrapped = result_future.result()
        result = wrapped.result
        return {
            "accepted": True,
            "status": int(wrapped.status),
            "status_name": {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(wrapped.status, f"STATUS_{wrapped.status}"),
            "success": result.success,
            "canceled": result.canceled,
            "timed_out": result.timed_out,
            "request_id": result.request_id,
            "mission_id": result.mission_id,
            "order_id": result.order_id,
            "skill_id": result.skill_id,
            "control_strategy": result.control_strategy,
            "phase_ids": list(result.phase_ids),
            "phase_backends": list(result.phase_backends),
            "checkpoint_ids": list(result.checkpoint_ids),
            "failure_stage": result.failure_stage,
            "failure_code": result.failure_code,
            "human_intervention_count": result.human_intervention_count,
            "message": result.message,
            "feedback": self.feedback,
        }

    def cancel_active(self) -> bool:
        """Request cancellation of the currently accepted Action goal."""
        with self._goal_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            return False
        goal_handle.cancel_goal_async()
        return True

    def has_active_goal(self) -> bool:
        with self._goal_lock:
            return self._active_goal_handle is not None

    def _feedback(self, message: object) -> None:
        feedback = message.feedback
        self.feedback.append({
            "current_phase": feedback.current_phase,
            "phase_index": feedback.phase_index,
            "phase_count": feedback.phase_count,
            "progress": round(float(feedback.progress), 6),
            "active_backend": feedback.active_backend,
            "checkpoint_id": feedback.checkpoint_id,
            "status": feedback.status,
        })


def _spec_from_args(args: argparse.Namespace) -> ManipulationGoalSpec:
    command = json.loads(Path(args.command_json).read_text(encoding="utf-8"))
    return ManipulationGoalSpec.from_mission_command(
        command,
        request_id=args.request_id,
        mission_id=args.mission_id,
        timeout_sec=args.timeout_sec,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-json", required=True, type=Path)
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--mission-id", required=True)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--cancel-after-sec", type=float)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    spec = _spec_from_args(args)
    rclpy.init()
    node = ManipulationActionClient()
    try:
        result = node.execute(spec, cancel_after_sec=args.cancel_after_sec)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
