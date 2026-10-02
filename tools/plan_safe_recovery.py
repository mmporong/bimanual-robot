#!/usr/bin/env python3
"""현재 SO-101 자세에서 새 충돌 없이 빠져나오는 최소 복귀 목표를 계산한다.

실물 서보에는 접근하지 않는다. ``export_current_joint_state.py``가 만든 JSON을
읽어 URDF 중립 자세 방향을 샘플링하고, 기존 접촉 쌍이 단조롭게 줄면서 관절 한계
여유를 확보하는 첫 자세만 출력한다. 결과는 실행 명령이 아니라 승인 전 미리보기다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from audit_pick_trajectory import audit_trajectory


def build_neutral_plan(neutral_deg: list[float]) -> dict:
    stage = {"joint_deg": neutral_deg}
    return {
        "motion_command_emitted": False,
        "ready_for_collision_review": True,
        "stages": {"pregrasp": stage, "grasp": stage},
    }


def plan_recovery(state: dict, right_deg: list[float], neutral_deg: list[float],
                  max_step_deg: float, minimum_margin_deg: float) -> dict:
    if state.get("motion_command_emitted") is not False:
        raise ValueError("읽기 전용으로 저장된 현재 상태만 허용합니다")
    audit = audit_trajectory(
        build_neutral_plan(neutral_deg),
        state["joint_degrees"],
        right_deg,
        max_step_deg,
        minimum_margin_deg,
    )
    recovery = audit["start_recovery"]
    return {
        "schema_version": "1.0",
        "mode": "recovery_plan_preview_only",
        "motion_command_emitted": False,
        "source_state_safe": state.get("safe_for_trajectory_start"),
        "start_joint_deg": audit["start_joint_deg"],
        "right_joint_deg": audit["right_joint_deg"],
        "neutral_search_target_deg": neutral_deg,
        "max_joint_step_deg": max_step_deg,
        "minimum_required_joint_margin_deg": minimum_margin_deg,
        "recovery": recovery,
        "ready_for_explicit_motion_approval": recovery["ready_for_explicit_motion_approval"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--right-deg", type=float, nargs=5, default=[0.0] * 5,
                        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL"))
    parser.add_argument("--neutral-deg", type=float, nargs=5, default=[0.0] * 5,
                        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL"))
    parser.add_argument("--max-step-deg", type=float, default=2.0)
    parser.add_argument("--minimum-margin-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = json.loads(args.state.read_text(encoding="utf-8"))
    report = plan_recovery(
        state,
        args.right_deg,
        args.neutral_deg,
        args.max_step_deg,
        args.minimum_margin_deg,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if args.strict and not report["ready_for_explicit_motion_approval"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
