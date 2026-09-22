"""Simulation-only ExecuteManipulationSkill Action server."""
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hold_flow_interfaces.action import ExecuteManipulationSkill

from .contract import ManipulationGoalSpec


ACTION_NAME = "/execute_manipulation_skill"


class MockManipulationServer(Node):
    """Runs validated phase transitions without touching robot hardware."""

    def __init__(self) -> None:
        super().__init__("mock_manipulation_server")
        self.declare_parameter("phase_delay_sec", 0.1)
        self.declare_parameter("failure_phase", "")
        self.declare_parameter("failure_code", "MOCK_FAILURE")
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

    def goal_callback(self, goal_request: ExecuteManipulationSkill.Goal) -> GoalResponse:
        try:
            ManipulationGoalSpec.from_action_goal(goal_request)
        except ValueError as exc:
            self.get_logger().warning(f"조작 Action 거부: {exc}")
            return GoalResponse.REJECT
        if not goal_request.dry_run:
            self.get_logger().warning("mock 서버는 dry_run=false 요청을 거부합니다")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle: object) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _base_result(
        self,
        spec: ManipulationGoalSpec,
        started_at: object,
    ) -> ExecuteManipulationSkill.Result:
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

    def execute_callback(self, goal_handle: object) -> ExecuteManipulationSkill.Result:
        spec = ManipulationGoalSpec.from_action_goal(goal_handle.request)
        started_at = self.get_clock().now().to_msg()
        started_monotonic = time.monotonic()
        phase_delay = max(
            0.0, float(self.get_parameter("phase_delay_sec").value)
        )
        failure_phase = str(self.get_parameter("failure_phase").value)
        failure_code = str(self.get_parameter("failure_code").value)
        phase_count = len(spec.phases)

        for index, phase in enumerate(spec.phases):
            if goal_handle.is_cancel_requested:
                result = self._base_result(spec, started_at)
                result.canceled = True
                result.failure_stage = phase.phase_id
                result.failure_code = "CANCELED"
                result.message = "mock execution canceled before the next phase"
                goal_handle.canceled()
                return result
            if time.monotonic() - started_monotonic >= spec.timeout_sec:
                result = self._base_result(spec, started_at)
                result.timed_out = True
                result.failure_stage = phase.phase_id
                result.failure_code = "TIMEOUT"
                result.message = "mock execution exceeded timeout_sec"
                goal_handle.abort()
                return result

            feedback = ExecuteManipulationSkill.Feedback()
            feedback.current_phase = phase.phase_id
            feedback.phase_index = index
            feedback.phase_count = phase_count
            feedback.progress = index / phase_count
            feedback.active_backend = phase.backend
            feedback.checkpoint_id = phase.checkpoint_id
            feedback.status = "RUNNING"
            goal_handle.publish_feedback(feedback)

            phase_deadline = time.monotonic() + phase_delay
            while time.monotonic() < phase_deadline:
                if goal_handle.is_cancel_requested:
                    result = self._base_result(spec, started_at)
                    result.canceled = True
                    result.failure_stage = phase.phase_id
                    result.failure_code = "CANCELED"
                    result.message = "mock execution canceled during phase"
                    goal_handle.canceled()
                    return result
                if time.monotonic() - started_monotonic >= spec.timeout_sec:
                    result = self._base_result(spec, started_at)
                    result.timed_out = True
                    result.failure_stage = phase.phase_id
                    result.failure_code = "TIMEOUT"
                    result.message = "mock execution exceeded timeout_sec"
                    goal_handle.abort()
                    return result
                time.sleep(min(0.01, max(0.0, phase_deadline - time.monotonic())))

            if failure_phase == phase.phase_id:
                result = self._base_result(spec, started_at)
                result.failure_stage = phase.phase_id
                result.failure_code = failure_code
                result.message = "failure injected by mock server parameter"
                goal_handle.abort()
                return result

        feedback = ExecuteManipulationSkill.Feedback()
        feedback.current_phase = spec.phases[-1].phase_id
        feedback.phase_index = phase_count
        feedback.phase_count = phase_count
        feedback.progress = 1.0
        feedback.active_backend = spec.phases[-1].backend
        feedback.checkpoint_id = spec.phases[-1].checkpoint_id
        feedback.status = "SUCCEEDED"
        goal_handle.publish_feedback(feedback)

        result = self._base_result(spec, started_at)
        result.success = True
        result.message = "simulation-only mock execution completed"
        goal_handle.succeed()
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockManipulationServer()
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
