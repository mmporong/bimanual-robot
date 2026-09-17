#!/usr/bin/env python3
"""충돌 감사된 컵 접근 계획을 실물 실행기의 입력 형식으로 고정한다.

이 도구는 파일만 변환하며 서보에는 접근하지 않는다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def prepare(plan: dict[str, Any], audit: dict[str, Any], stage: str = "pregrasp") -> dict[str, Any]:
    if stage not in {"pregrasp", "grasp"}:
        raise ValueError("실행 단계는 pregrasp 또는 grasp여야 합니다")
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 IK 계획만 허용합니다")
    if audit.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 충돌 감사만 허용합니다")
    if not audit.get("continuous_path_ready_for_preview", False):
        raise ValueError("현재 자세부터 이어지는 연속 경로가 충돌 감사를 통과하지 못했습니다")

    target = plan.get("stages", {}).get(stage, {}).get("joint_deg")
    audited_target = audit.get("target_stages_joint_deg", {}).get(stage)
    if not isinstance(target, list) or len(target) != 5:
        raise ValueError(f"IK 계획의 {stage} 관절값이 올바르지 않습니다")
    if not isinstance(audited_target, list) or len(audited_target) != 5:
        raise ValueError(f"충돌 감사에 {stage} 관절값이 없습니다")
    if any(abs(float(a) - float(b)) > 1e-3 for a, b in zip(target, audited_target)):
        raise ValueError(f"IK 계획과 충돌 감사의 {stage} 목표가 다릅니다")
    if stage == "grasp":
        pregrasp = plan.get("stages", {}).get("pregrasp", {}).get("joint_deg")
        audited_pregrasp = audit.get("target_stages_joint_deg", {}).get("pregrasp")
        if not isinstance(pregrasp, list) or not isinstance(audited_pregrasp, list):
            raise ValueError("grasp 연속 실행에 필요한 pregrasp 목표가 없습니다")
        if any(abs(float(a) - float(b)) > 1e-3 for a, b in zip(pregrasp, audited_pregrasp)):
            raise ValueError("IK 계획과 충돌 감사의 pregrasp 시작값이 다릅니다")

    start_joint_deg = (
        audit["start_joint_deg"]
        if stage == "pregrasp"
        else pregrasp
    )
    return {
        "schema_version": "1.0",
        "mode": "audited_pick_stage_preview_only",
        "motion_command_emitted": False,
        "start_joint_deg": [float(value) for value in start_joint_deg],
        "recovery": {
            "target_joint_deg": [float(value) for value in target],
            "source_stage": stage,
        },
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
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
