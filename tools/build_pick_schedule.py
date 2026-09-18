#!/usr/bin/env python3
"""감사된 컵 접근 waypoints를 시간 동기 오프라인 미리보기 일정으로 만든다.

모든 관절은 구간마다 같은 scalar progress를 사용한다. 이 도구는 파일만 읽고 쓰며
서보 포트를 열거나 이동 명령을 내보내지 않는다.
"""

from __future__ import annotations

import argparse
import json
import math
import numbers
from pathlib import Path
from typing import Any

from prepare_pick_execution import matched_audited_waypoints, path_review, write_new_output


def _positive_finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{label}은 양의 유한한 숫자여야 합니다")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label}은 양의 유한한 숫자여야 합니다")
    return result


def speed_limits(values: Any) -> list[float]:
    if not isinstance(values, list) or len(values) not in {1, 5}:
        raise ValueError("관절 속도 제한은 양의 유한한 숫자 1개 또는 5개여야 합니다")
    validated = [_positive_finite(value, "관절 속도 제한") for value in values]
    return validated * 5 if len(validated) == 1 else validated


def build_schedule(
    plan: dict[str, Any],
    audit: dict[str, Any],
    max_joint_speed_deg_s: list[float],
    sample_period_s: float = 0.1,
    max_samples: int = 10_000,
) -> dict[str, Any]:
    """Build a synchronized linear schedule without issuing a hardware command."""
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 IK 계획만 허용합니다")
    if audit.get("motion_command_emitted") is not False:
        raise ValueError("명령을 내보내지 않은 충돌 감사만 허용합니다")
    if audit.get("continuous_path_ready_for_preview") is not True:
        raise ValueError("현재 자세부터 이어지는 연속 경로가 준비되지 않았습니다")

    start, stages = matched_audited_waypoints(plan, audit)
    review = path_review(audit)
    limits = speed_limits(max_joint_speed_deg_s)
    period = _positive_finite(sample_period_s, "샘플 주기")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
        raise ValueError("최대 샘플 수는 2 이상의 정수여야 합니다")

    waypoint_items = [
        ("start", start),
        ("pregrasp", stages["pregrasp"]),
        ("grasp", stages["grasp"]),
    ]
    segment_specs: list[tuple[str, list[float], list[float], float, int]] = []
    expected_samples = 1
    for (source_name, source), (target_name, target) in zip(
        waypoint_items, waypoint_items[1:]
    ):
        deltas = [
            abs(target_value - source_value)
            for source_value, target_value in zip(source, target)
        ]
        duration = max(delta / limit for delta, limit in zip(deltas, limits))
        if not math.isfinite(duration):
            raise ValueError("waypoint 차이로 계산한 구간 시간이 유한하지 않습니다")
        if max(deltas) > 0.0 and duration == 0.0:
            raise ValueError("양수 waypoint 차이의 구간 시간을 부동소수점으로 표현할 수 없습니다")
        if duration == 0.0:
            segment_specs.append(
                (f"{source_name}_to_{target_name}", source, target, duration, 0)
            )
            continue
        sample_ratio = duration / period
        if not math.isfinite(sample_ratio):
            raise ValueError("구간 시간과 샘플 주기의 비율이 유한하지 않습니다")
        step_count = max(1, math.ceil(sample_ratio))
        expected_samples += step_count
        if expected_samples > max_samples:
            raise ValueError(
                f"예상 샘플 수 {expected_samples}개가 제한 {max_samples}개를 초과합니다"
            )
        segment_specs.append(
            (f"{source_name}_to_{target_name}", source, target, duration, step_count)
        )

    samples = [
        {
            "sample": 0,
            "time_s": 0.0,
            "phase": "start",
            "scalar_progress": 0.0,
            "joint_deg": start,
        }
    ]
    elapsed = 0.0
    for phase, source, target, duration, step_count in segment_specs:
        if step_count == 0:
            continue
        for step in range(1, step_count + 1):
            progress = step / step_count
            joint_deg = [
                target_value if step == step_count else (
                    source_value + (target_value - source_value) * progress
                )
                for source_value, target_value in zip(source, target)
            ]
            timestamp = elapsed + duration * progress
            if not math.isfinite(timestamp) or timestamp <= samples[-1]["time_s"]:
                raise ValueError("시간 샘플의 양수 간격을 부동소수점으로 표현할 수 없습니다")
            samples.append(
                {
                    "sample": len(samples),
                    "time_s": timestamp,
                    "phase": phase,
                    "scalar_progress": progress,
                    "joint_deg": joint_deg,
                }
            )
        elapsed += duration

    return {
        "schema_version": "1.0",
        "mode": "audited_pick_time_schedule_preview_only",
        "motion_command_emitted": False,
        "real_hardware_execution_approved": False,
        "units": {
            "joint_position": "degree",
            "joint_speed_limit": "degree/second",
            "time": "second",
        },
        "interpolation": "per_segment_common_linear_scalar",
        "max_joint_speed_deg_s": limits,
        "requested_sample_period_s": period,
        "path_review": review,
        "waypoints": [
            {"name": name, "joint_deg": values}
            for name, values in waypoint_items
        ],
        "segment_duration_s": {
            phase: duration
            for phase, _source, _target, duration, _steps in segment_specs
        },
        "total_duration_s": elapsed,
        "sample_count": len(samples),
        "samples": samples,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument(
        "--max-joint-speed-deg-s",
        type=float,
        nargs="+",
        required=True,
        help="전체 축 공통 값 1개 또는 축별 값 5개",
    )
    parser.add_argument("--sample-period-s", type=float, default=0.1)
    parser.add_argument("--max-samples", type=int, default=10_000)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = build_schedule(
        json.loads(args.plan.read_text(encoding="utf-8")),
        json.loads(args.audit.read_text(encoding="utf-8")),
        args.max_joint_speed_deg_s,
        args.sample_period_s,
        args.max_samples,
    )
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        write_new_output(args.output, [args.plan, args.audit], text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
