"""Pure-Python contract for ExecuteManipulationSkill goals.

The module deliberately has no ROS imports so contract validation can run in
CI and before any ROS node or hardware bridge is started.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Mapping


CONTROL_STRATEGIES = frozenset({"PLANNED_ALL", "ACT_ALL", "HYBRID"})
PHASE_BACKENDS = frozenset({"PLANNED", "ACT"})
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,128}$")


def _required_identifier(value: object, name: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(f"{name} must match {IDENTIFIER_PATTERN.pattern}")
    return value


def _optional_identifier(value: object, name: str) -> str:
    if value == "":
        return ""
    return _required_identifier(value, name)


@dataclass(frozen=True)
class PhaseSpec:
    phase_id: str
    backend: str
    checkpoint_id: str = ""

    def __post_init__(self) -> None:
        _required_identifier(self.phase_id, "phase_id")
        if self.backend not in PHASE_BACKENDS:
            raise ValueError(f"backend must be one of {sorted(PHASE_BACKENDS)}")
        _optional_identifier(self.checkpoint_id, "checkpoint_id")
        if self.backend == "ACT" and not self.checkpoint_id:
            raise ValueError("ACT phase requires checkpoint_id")
        if self.backend == "PLANNED" and self.checkpoint_id:
            raise ValueError("PLANNED phase must not set checkpoint_id")


@dataclass(frozen=True)
class ManipulationGoalSpec:
    request_id: str
    mission_id: str
    order_id: str
    skill_id: str
    control_strategy: str
    phases: tuple[PhaseSpec, ...]
    timeout_sec: float
    dry_run: bool
    table_id: str = ""
    drink: str = ""
    bottle_id: str = ""

    def __post_init__(self) -> None:
        for name in ("request_id", "mission_id", "order_id", "skill_id"):
            _required_identifier(getattr(self, name), name)
        for name in ("table_id", "drink", "bottle_id"):
            _optional_identifier(getattr(self, name), name)
        if self.control_strategy not in CONTROL_STRATEGIES:
            raise ValueError(
                f"control_strategy must be one of {sorted(CONTROL_STRATEGIES)}"
            )
        if not self.phases:
            raise ValueError("at least one phase is required")
        phase_ids = [phase.phase_id for phase in self.phases]
        if len(phase_ids) != len(set(phase_ids)):
            raise ValueError("phase_id values must be unique")
        if (
            isinstance(self.timeout_sec, bool)
            or not isinstance(self.timeout_sec, (int, float))
            or not math.isfinite(self.timeout_sec)
        ):
            raise ValueError("timeout_sec must be finite")
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if not isinstance(self.dry_run, bool):
            raise ValueError("dry_run must be boolean")
        expected = {
            "PLANNED_ALL": {"PLANNED"},
            "ACT_ALL": {"ACT"},
            "HYBRID": {"PLANNED", "ACT"},
        }[self.control_strategy]
        actual = {phase.backend for phase in self.phases}
        if actual != expected:
            raise ValueError(
                f"{self.control_strategy} requires phase backends {sorted(expected)}"
            )

    @property
    def phase_ids(self) -> list[str]:
        return [phase.phase_id for phase in self.phases]

    @property
    def phase_backends(self) -> list[str]:
        return [phase.backend for phase in self.phases]

    @property
    def checkpoint_ids(self) -> list[str]:
        return [phase.checkpoint_id for phase in self.phases]

    @classmethod
    def from_action_goal(cls, goal: object) -> "ManipulationGoalSpec":
        phase_ids = list(goal.phase_ids)
        backends = list(goal.phase_backends)
        checkpoints = list(goal.checkpoint_ids)
        if not (len(phase_ids) == len(backends) == len(checkpoints)):
            raise ValueError(
                "phase_ids, phase_backends, and checkpoint_ids must have equal lengths"
            )
        phases = tuple(
            PhaseSpec(phase_id=phase, backend=backend, checkpoint_id=checkpoint)
            for phase, backend, checkpoint in zip(phase_ids, backends, checkpoints)
        )
        return cls(
            request_id=goal.request_id,
            mission_id=goal.mission_id,
            order_id=goal.order_id,
            skill_id=goal.skill_id,
            control_strategy=goal.control_strategy,
            phases=phases,
            timeout_sec=float(goal.timeout_sec),
            dry_run=goal.dry_run,
            table_id=goal.table_id,
            drink=goal.drink,
            bottle_id=goal.bottle_id,
        )

    @classmethod
    def from_mission_command(
        cls,
        command: Mapping[str, object],
        *,
        request_id: str,
        mission_id: str,
        timeout_sec: float = 30.0,
    ) -> "ManipulationGoalSpec":
        if command.get("kind") != "manipulate":
            raise ValueError("mission command kind must be manipulate")
        if command.get("hardware_accessed") is not False:
            raise ValueError("adapter accepts only commands marked hardware_accessed=false")
        strategy = str(command.get("policy_mode", ""))
        backend = {"PLANNED_ALL": "PLANNED", "ACT_ALL": "ACT"}.get(strategy)
        if backend is None:
            raise ValueError("single-phase mission command supports PLANNED_ALL or ACT_ALL")
        checkpoint = str(command.get("checkpoint_id", ""))
        phase = PhaseSpec(
            phase_id=str(command.get("phase", "")),
            backend=backend,
            checkpoint_id=checkpoint,
        )
        return cls(
            request_id=request_id,
            mission_id=mission_id,
            order_id=str(command.get("order_id", "")),
            skill_id=str(command.get("name", "")),
            control_strategy=strategy,
            phases=(phase,),
            timeout_sec=timeout_sec,
            dry_run=True,
            table_id=str(command.get("table_id", "")),
            drink=str(command.get("drink", "")),
            bottle_id=str(command.get("bottle_id", "")),
        )

    def populate_action_goal(self, goal: object) -> object:
        goal.request_id = self.request_id
        goal.mission_id = self.mission_id
        goal.order_id = self.order_id
        goal.skill_id = self.skill_id
        goal.control_strategy = self.control_strategy
        goal.table_id = self.table_id
        goal.drink = self.drink
        goal.bottle_id = self.bottle_id
        goal.phase_ids = self.phase_ids
        goal.phase_backends = self.phase_backends
        goal.checkpoint_ids = self.checkpoint_ids
        goal.timeout_sec = self.timeout_sec
        goal.dry_run = self.dry_run
        return goal
