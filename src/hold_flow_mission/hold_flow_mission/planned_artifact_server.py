"""ROS 2 Action server backed by verified Isaac Sim result artifacts."""
from __future__ import annotations

from pathlib import Path
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hold_flow_interfaces.action import ExecuteManipulationSkill

from .contract import ManipulationGoalSpec
from .planned_artifact import (
    ArtifactValidationError,
    PlannedArtifactReport,
    load_planned_artifact,
    validate_request_scope,
)


ACTION_NAME = "/execute_manipulation_skill"


class PlannedArtifactServer(Node):
    """Replays phase verdicts from one hash-verified simulation result."""

    def __init__(self) -> None:
        super().__init__("planned_artifact_manipulation_server")
        self.declare_parameter("result_path", "")
        self.declare_parameter("repo_root", "")
        self.declare_parameter("phase_delay_sec", 0.02)
        self._report: PlannedArtifactReport | None = None
        self._load_error: ArtifactValidationError | None = None
        result_path = str(self.get_parameter("result_path").value)
        repo_root = str(self.get_parameter("repo_root").value)
        self._result_path = Path(result_path) if result_path else None
        self._repo_root = Path(repo_root) if repo_root else None
        if result_path and repo_root:
            try:
                self._reload_report()
            except ArtifactValidationError as exc:
                self.get_logger().error(f"PLANNED 산출물 거부 [{exc.code}]: {exc}")
        else:
            self._load_error = ArtifactValidationError(
                "ARTIFACT_CONFIG_MISSING",
                "result_path and repo_root parameters are required",
            )
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

    def _reload_report(self) -> PlannedArtifactReport:
        if self._result_path is None or self._repo_root is None:
            error = ArtifactValidationError(
                "ARTIFACT_CONFIG_MISSING",
                "result_path and repo_root parameters are required",
            )
            self._load_error = error
            self._report = None
            raise error
        try:
            report = load_planned_artifact(
                self._result_path,
                repo_root=self._repo_root,
            )
        except ArtifactValidationError as exc:
            self._load_error = exc
            self._report = None
            raise
        self._load_error = None
        self._report = report
        return report

    def _contract_spec(self, goal: ExecuteManipulationSkill.Goal) -> ManipulationGoalSpec:
        spec = ManipulationGoalSpec.from_action_goal(goal)
        if not spec.dry_run:
            raise ValueError("artifact replay requires dry_run=true")
        if spec.control_strategy != "PLANNED_ALL":
            raise ValueError("artifact replay accepts PLANNED_ALL only")
        return spec

    def goal_callback(self, goal_request: ExecuteManipulationSkill.Goal) -> GoalResponse:
        try:
            self._contract_spec(goal_request)
        except ValueError as exc:
            self.get_logger().warning(f"PLANNED 산출물 Goal 거부: {exc}")
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
        spec = self._contract_spec(goal_handle.request)
        started_at = self.get_clock().now().to_msg()
        try:
            report = self._reload_report()
            validate_request_scope(
                report,
                table_id=spec.table_id,
                drink=spec.drink,
                phase_ids=spec.phase_ids,
            )
        except ArtifactValidationError as exc:
            result = self._base_result(spec, started_at)
            result.failure_stage = spec.phases[0].phase_id
            result.failure_code = exc.code
            result.message = str(exc)
            goal_handle.abort()
            return result
        phase_count = len(spec.phases)
        delay = max(0.0, float(self.get_parameter("phase_delay_sec").value))
        for index, phase in enumerate(spec.phases):
            if goal_handle.is_cancel_requested:
                result = self._base_result(spec, started_at)
                result.canceled = True
                result.failure_stage = phase.phase_id
                result.failure_code = "CANCELED"
                result.message = "verified artifact replay canceled"
                goal_handle.canceled()
                return result
            feedback = ExecuteManipulationSkill.Feedback()
            feedback.current_phase = phase.phase_id
            feedback.phase_index = index
            feedback.phase_count = phase_count
            feedback.progress = index / phase_count
            feedback.active_backend = "PLANNED_ARTIFACT_REPLAY"
            feedback.checkpoint_id = ""
            feedback.status = "VERIFYING_ARTIFACT"
            goal_handle.publish_feedback(feedback)
            deadline = time.monotonic() + delay
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    result = self._base_result(spec, started_at)
                    result.canceled = True
                    result.failure_stage = phase.phase_id
                    result.failure_code = "CANCELED"
                    result.message = "verified artifact replay canceled"
                    goal_handle.canceled()
                    return result
                time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))

        feedback = ExecuteManipulationSkill.Feedback()
        feedback.current_phase = spec.phases[-1].phase_id
        feedback.phase_index = phase_count
        feedback.phase_count = phase_count
        feedback.progress = 1.0
        feedback.active_backend = "PLANNED_ARTIFACT_REPLAY"
        feedback.checkpoint_id = ""
        feedback.status = "SUCCEEDED"
        goal_handle.publish_feedback(feedback)

        result = self._base_result(spec, started_at)
        result.success = True
        result.message = (
            "hash-verified Isaac Sim artifact replay; simulator was not executed; "
            f"artifact_sha256={report.result_sha256}"
        )
        goal_handle.succeed()
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlannedArtifactServer()
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
