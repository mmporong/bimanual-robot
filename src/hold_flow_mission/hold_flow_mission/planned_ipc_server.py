"""ROS 2 Action bridge to a persistent PLANNED executor over a Unix socket."""
from __future__ import annotations

from pathlib import Path
import json

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hold_flow_interfaces.action import ExecuteManipulationSkill

from .contract import ManipulationGoalSpec
from .planned_ipc import PlannedIpcError, cancel_session, execute_phase


ACTION_NAME = "/execute_manipulation_skill"


class PlannedIpcServer(Node):
    def __init__(self) -> None:
        super().__init__("planned_ipc_manipulation_server")
        self.declare_parameter("socket_path", "/tmp/hold-flow-planned-executor.sock")
        self._socket_path = Path(str(self.get_parameter("socket_path").value))
        self._action_server = ActionServer(
            self,
            ExecuteManipulationSkill,
            ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def destroy_node(self) -> None:
        self._action_server.destroy()
        super().destroy_node()

    @staticmethod
    def _spec(goal: ExecuteManipulationSkill.Goal) -> ManipulationGoalSpec:
        spec = ManipulationGoalSpec.from_action_goal(goal)
        if not spec.dry_run:
            raise ValueError("PLANNED IPC bridge requires dry_run=true")
        if spec.control_strategy != "PLANNED_ALL" or len(spec.phases) != 1:
            raise ValueError("PLANNED IPC bridge accepts one PLANNED_ALL phase")
        return spec

    def goal_callback(self, request: ExecuteManipulationSkill.Goal) -> GoalResponse:
        try:
            self._spec(request)
        except ValueError as exc:
            self.get_logger().warning(f"PLANNED IPC Goal 거부: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: object) -> CancelResponse:
        mission_id = str(goal_handle.request.mission_id)
        try:
            accepted = cancel_session(self._socket_path, mission_id)
        except PlannedIpcError as exc:
            self.get_logger().warning(f"PLANNED IPC 취소 전달 실패 [{exc.code}]: {exc}")
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT if accepted else CancelResponse.REJECT

    def _base_result(self, spec: ManipulationGoalSpec, started_at: object):
        result = ExecuteManipulationSkill.Result()
        result.request_id = spec.request_id
        result.mission_id = spec.mission_id
        result.order_id = spec.order_id
        result.skill_id = spec.skill_id
        result.control_strategy = spec.control_strategy
        result.phase_ids = spec.phase_ids
        result.phase_backends = spec.phase_backends
        result.checkpoint_ids = spec.checkpoint_ids
        result.started_at = started_at
        result.ended_at = self.get_clock().now().to_msg()
        return result

    def execute_callback(self, goal_handle: object):
        spec = self._spec(goal_handle.request)
        phase = spec.phases[0]
        started_at = self.get_clock().now().to_msg()
        feedback = ExecuteManipulationSkill.Feedback()
        feedback.current_phase = phase.phase_id
        feedback.phase_index = 0
        feedback.phase_count = 1
        feedback.progress = 0.0
        feedback.active_backend = "PLANNED_IPC"
        feedback.checkpoint_id = ""
        feedback.status = "WAITING_FOR_EXECUTOR"
        goal_handle.publish_feedback(feedback)
        try:
            response = execute_phase(
                self._socket_path,
                request_id=spec.request_id,
                mission_id=spec.mission_id,
                order_id=spec.order_id,
                phase_id=phase.phase_id,
                table_id=spec.table_id,
                drink=spec.drink,
                timeout_sec=spec.timeout_sec,
            )
        except PlannedIpcError as exc:
            response = {"success": False, "failure_code": exc.code, "message": str(exc)}
            # A lost response is not proof that physics stopped.
            try:
                response['stop_requested'] = cancel_session(self._socket_path, spec.mission_id)
            except PlannedIpcError:
                response['stop_requested'] = False
            response['stop_confirmed'] = False

        result = self._base_result(spec, started_at)
        # Preserve executor provenance and observed poses/contacts in SQLite via Action result.
        result.message = json.dumps(response, ensure_ascii=False)
        if goal_handle.is_cancel_requested:
            try:
                cancel_session(self._socket_path, spec.mission_id)
            except PlannedIpcError:
                pass
            result.canceled = True
            result.failure_stage = phase.phase_id
            result.failure_code = "CANCELED"
            if response.get('executor_kind') == 'isaac_physics' and not response.get('stop_confirmed'):
                result.canceled = False
                result.failure_code = 'CANCEL_STOP_UNCONFIRMED'
                goal_handle.abort()
                return result
            goal_handle.canceled()
            return result
        if response.get("success"):
            feedback.phase_index = 1
            feedback.progress = 1.0
            feedback.status = "SUCCEEDED"
            goal_handle.publish_feedback(feedback)
            result.success = True
            goal_handle.succeed()
            return result
        result.failure_stage = phase.phase_id
        result.failure_code = str(response.get("failure_code") or "PLANNED_EXECUTOR_FAILED")
        if result.failure_code == "CANCELED":
            result.canceled = True
            goal_handle.canceled()
        elif result.failure_code in {"IPC_TIMEOUT", "TIMEOUT"}:
            result.timed_out = True
            goal_handle.abort()
        else:
            goal_handle.abort()
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlannedIpcServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
