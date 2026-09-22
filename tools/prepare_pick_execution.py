#!/usr/bin/env python3
"""충돌 감사된 컵 접근 계획을 실물 실행기의 입력 형식으로 고정한다.

이 도구는 파일만 변환하며 서보에는 접근하지 않는다.
"""

from __future__ import annotations

import argparse
import json
import math
import numbers
from pathlib import Path
from typing import Any


JOINT_COUNT = 5
TARGET_MATCH_TOLERANCE_DEG = 1e-3


def finite_joint_vector(value: Any, label: str) -> list[float]:
    """Return a validated five-axis joint vector in degrees."""
    if not isinstance(value, list) or len(value) != JOINT_COUNT:
        raise ValueError(f"{label}은 유한한 숫자 {JOINT_COUNT}개여야 합니다")

    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, numbers.Real):
            raise ValueError(f"{label}은 유한한 숫자 {JOINT_COUNT}개여야 합니다")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{label}은 유한한 숫자 {JOINT_COUNT}개여야 합니다")
        result.append(number)
    return result


def matched_audited_waypoints(
    plan: dict[str, Any], audit: dict[str, Any]
) -> tuple[list[float], dict[str, list[float]]]:
    """Validate and return the audited start, pregrasp, and grasp waypoints."""
    plan_stages = plan.get("stages")
    audit_stages = audit.get("target_stages_joint_deg")
    if not isinstance(plan_stages, dict) or set(plan_stages) != {"pregrasp", "grasp"}:
        raise ValueError("IK 계획에는 pregrasp와 grasp 두 단계가 정확히 있어야 합니다")
    if not isinstance(audit_stages, dict) or set(audit_stages) != {"pregrasp", "grasp"}:
        raise ValueError("충돌 감사에는 pregrasp와 grasp 두 단계가 정확히 있어야 합니다")

    start = finite_joint_vector(audit.get("start_joint_deg"), "충돌 감사 시작 관절값")
    matched: dict[str, list[float]] = {}
    for name in ("pregrasp", "grasp"):
        stage = plan_stages.get(name)
        if not isinstance(stage, dict):
            raise ValueError(f"IK 계획의 {name} 단계가 올바르지 않습니다")
        planned = finite_joint_vector(stage.get("joint_deg"), f"IK 계획의 {name} 관절값")
        audited = finite_joint_vector(audit_stages.get(name), f"충돌 감사의 {name} 관절값")
        if any(
            abs(planned_value - audited_value) > TARGET_MATCH_TOLERANCE_DEG
            for planned_value, audited_value in zip(planned, audited)
        ):
            raise ValueError(f"IK 계획과 충돌 감사의 {name} 목표가 다릅니다")
        matched[name] = planned
    return start, matched


def path_review(audit: dict[str, Any]) -> dict[str, Any]:
    """Keep collision-free and initial-contact recovery audits distinguishable."""
    collision_free = audit.get("trajectory_ready_for_preview") is True
    failed_sample_count = audit.get("failed_sample_count")
    recovery = audit.get("start_recovery")
    recovery_required = (
        isinstance(recovery, dict)
        and recovery.get("required") is True
    )
    recovery_ready = (
        recovery_required
        and recovery.get("ready_for_explicit_motion_approval") is True
    )
    if collision_free:
        if (
            isinstance(failed_sample_count, bool)
            or failed_sample_count != 0
            or recovery_required
        ):
            raise ValueError("충돌 없음 표시는 실패 샘플 또는 시작 접촉 복구 표시와 모순됩니다")
        classification = "collision_free"
    elif recovery_ready:
        if (
            isinstance(failed_sample_count, bool)
            or not isinstance(failed_sample_count, int)
            or failed_sample_count <= 0
        ):
            raise ValueError("시작 접촉 복구에는 양의 정수 실패 샘플 수가 필요합니다")
        classification = "start_recovery_only"
    else:
        raise ValueError("연속 경로 표시는 충돌 없음 또는 시작 접촉 복구 근거와 일치해야 합니다")
    return {
        "classification": classification,
        "collision_free": collision_free,
        "start_recovery_required": recovery_required,
    }


def write_new_output(output: Path, inputs: list[Path], text: str) -> None:
    """Create a new output file without overwriting an input or existing path."""
    resolved_output = output.resolve()
    if any(resolved_output == input_path.resolve() for input_path in inputs):
        raise ValueError("출력 경로는 입력 계획 또는 감사 경로와 달라야 합니다")
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    except FileExistsError as exc:
        raise ValueError(f"출력 파일이 이미 존재합니다: {output}") from exc


def prepare(plan: dict[str, Any], audit: dict[str, Any], stage: str = "pregrasp") -> dict[str, Any]:
    if stage not in {"pregrasp", "grasp"}:
        raise ValueError("실행 단계는 pregrasp 또는 grasp여야 합니다")
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 IK 계획만 허용합니다")
    if audit.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 충돌 감사만 허용합니다")
    if audit.get("continuous_path_ready_for_preview") is not True:
        raise ValueError("현재 자세부터 이어지는 연속 경로가 충돌 감사를 통과하지 못했습니다")

    audited_start, waypoints = matched_audited_waypoints(plan, audit)
    review = path_review(audit)
    target = waypoints[stage]
    start_joint_deg = audited_start if stage == "pregrasp" else waypoints["pregrasp"]
    return {
        "schema_version": "1.0",
        "mode": "audited_pick_stage_preview_only",
        "motion_command_emitted": False,
        "start_joint_deg": start_joint_deg,
        "recovery": {
            "target_joint_deg": target,
            "source_stage": stage,
        },
        "path_review": review,
        "ready_for_explicit_motion_approval": True,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--stage", choices=["pregrasp", "grasp"], default="pregrasp")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = prepare(
        json.loads(args.plan.read_text(encoding="utf-8")),
        json.loads(args.audit.read_text(encoding="utf-8")),
        args.stage,
    )
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        write_new_output(args.output, [args.plan, args.audit], text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
