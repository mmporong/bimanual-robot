from dataclasses import dataclass
import unittest

from hold_flow_mission.contract import ManipulationGoalSpec, PhaseSpec


def planned_phase(name="GRASP_CUP"):
    return PhaseSpec(name, "PLANNED")


@dataclass
class GoalMessage:
    request_id: str = "REQ-1"
    mission_id: str = "MIS-1"
    order_id: str = "ORD-1"
    skill_id: str = "grasp_cup"
    control_strategy: str = "PLANNED_ALL"
    table_id: str = "table_1"
    drink: str = "COLD_WATER"
    bottle_id: str = "cold_bottle"
    phase_ids: tuple[str, ...] = ("GRASP_CUP",)
    phase_backends: tuple[str, ...] = ("PLANNED",)
    checkpoint_ids: tuple[str, ...] = ("",)
    timeout_sec: float = 10.0
    dry_run: bool = True


class ContractTest(unittest.TestCase):
    def test_planned_goal_is_valid_and_serializes_arrays(self):
        spec = ManipulationGoalSpec(
            request_id="REQ-1",
            mission_id="MIS-1",
            order_id="ORD-1",
            skill_id="grasp_cup",
            control_strategy="PLANNED_ALL",
            phases=(planned_phase(),),
            timeout_sec=5.0,
            dry_run=True,
        )
        self.assertEqual(spec.phase_ids, ["GRASP_CUP"])
        self.assertEqual(spec.phase_backends, ["PLANNED"])
        self.assertEqual(spec.checkpoint_ids, [""])

    def test_act_requires_checkpoint_and_strategy_backend_match(self):
        with self.assertRaisesRegex(ValueError, "requires checkpoint"):
            PhaseSpec("POUR", "ACT")
        with self.assertRaisesRegex(ValueError, "requires phase backends"):
            ManipulationGoalSpec(
                request_id="REQ-1",
                mission_id="MIS-1",
                order_id="ORD-1",
                skill_id="pour_water",
                control_strategy="ACT_ALL",
                phases=(planned_phase("POUR"),),
                timeout_sec=5.0,
                dry_run=True,
            )

    def test_hybrid_requires_both_backends(self):
        spec = ManipulationGoalSpec(
            request_id="REQ-1",
            mission_id="MIS-1",
            order_id="ORD-1",
            skill_id="water_kitchen_full",
            control_strategy="HYBRID",
            phases=(
                PhaseSpec("GRASP_CUP", "PLANNED"),
                PhaseSpec("POUR", "ACT", "smolvla-pour-v1"),
            ),
            timeout_sec=30.0,
            dry_run=True,
        )
        self.assertEqual(spec.phase_backends, ["PLANNED", "ACT"])

    def test_timeout_must_be_a_positive_finite_number(self):
        for timeout in (True, "5", 0.0, float("inf")):
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    ManipulationGoalSpec(
                        request_id="REQ-1",
                        mission_id="MIS-1",
                        order_id="ORD-1",
                        skill_id="grasp_cup",
                        control_strategy="PLANNED_ALL",
                        phases=(planned_phase(),),
                        timeout_sec=timeout,
                        dry_run=True,
                    )

    def test_action_goal_rejects_mismatched_parallel_arrays(self):
        goal = GoalMessage(phase_backends=("PLANNED", "PLANNED"))
        with self.assertRaisesRegex(ValueError, "equal lengths"):
            ManipulationGoalSpec.from_action_goal(goal)

    def test_mission_command_maps_to_single_planned_phase(self):
        spec = ManipulationGoalSpec.from_mission_command(
            {
                "kind": "manipulate",
                "name": "grasp_cup",
                "phase": "GRASP_CUP",
                "policy_mode": "PLANNED_ALL",
                "order_id": "ORD-1",
                "table_id": "table_1",
                "drink": "COLD_WATER",
                "bottle_id": "cold_bottle",
                "hardware_accessed": False,
            },
            request_id="REQ-1",
            mission_id="MIS-1",
        )
        self.assertEqual(spec.skill_id, "grasp_cup")
        self.assertIs(spec.dry_run, True)
        self.assertEqual(spec.phases, (PhaseSpec("GRASP_CUP", "PLANNED"),))

    def test_adapter_rejects_hardware_marked_command(self):
        with self.assertRaisesRegex(ValueError, "hardware_accessed=false"):
            ManipulationGoalSpec.from_mission_command(
                {
                    "kind": "manipulate",
                    "name": "grasp_cup",
                    "phase": "GRASP_CUP",
                    "policy_mode": "PLANNED_ALL",
                    "order_id": "ORD-1",
                    "hardware_accessed": True,
                },
                request_id="REQ-1",
                mission_id="MIS-1",
            )


if __name__ == "__main__":
    unittest.main()
