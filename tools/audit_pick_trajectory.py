#!/usr/bin/env python3
"""컵 파지 IK 계획을 관절 보간해 URDF 충돌·한계 여유를 감사한다.

입력은 ``cup_pick_dry_run.py --plan`` JSON과 현재 왼팔 관절각이다. 시작→pre-grasp→
grasp를 일정한 최대 관절 간격으로 샘플링하고, 좌우 팔·마스트·LiDAR 여유와 왼팔
비인접 링크 자가충돌, 상판·센서 구조물 충돌을 검사한다. 서보 포트를 열지 않는다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
CAD_DIR = REPO_ROOT / "design/cad"
IK_DIR = REPO_ROOT / "src/hold_flow_description/scripts"
for directory in (CAD_DIR, IK_DIR):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from audit_urdf_clearance import (  # noqa: E402
    aabb_gap,
    audit_state,
    collision_polydata,
    intersects,
)
from render_design_handoff import URDF_PATH, UrdfScene  # noqa: E402
from solve_task_poses import ARM_JOINTS, Chain, arm_names  # noqa: E402


MOVING_LEFT_LINKS = [
    "left_shoulder_link",
    "left_upper_arm_link",
    "left_lower_arm_link",
    "left_wrist_link",
    "left_gripper_link",
    "left_moving_jaw_link",
]
STATIC_CRITICAL_LINKS = [
    "tabletop_front_left_link",
    "tabletop_front_right_link",
    "tabletop_rear_left_link",
    "tabletop_rear_right_link",
    "camera_backing_link",
    "camera_mast_link",
    "camera_cradle_link",
    "lidar_mount_plate_link",
    "laser_link",
]


def interpolate_segment(start_deg: np.ndarray, end_deg: np.ndarray, max_step_deg: float) -> list[np.ndarray]:
    if max_step_deg <= 0.0:
        raise ValueError("max_step_deg는 양수여야 합니다")
    steps = max(1, math.ceil(float(np.max(np.abs(end_deg - start_deg))) / max_step_deg))
    return [start_deg + (end_deg - start_deg) * (index / steps) for index in range(1, steps + 1)]


def trajectory_samples(start_deg: np.ndarray, pregrasp_deg: np.ndarray,
                       grasp_deg: np.ndarray, max_step_deg: float) -> list[tuple[str, np.ndarray]]:
    samples: list[tuple[str, np.ndarray]] = [("start", start_deg.copy())]
    samples.extend(
        ("to_pregrasp", value)
        for value in interpolate_segment(start_deg, pregrasp_deg, max_step_deg)
    )
    samples.extend(
        ("to_grasp", value)
        for value in interpolate_segment(pregrasp_deg, grasp_deg, max_step_deg)
    )
    return samples


def collision_hits(polys: dict[str, Any], pairs: list[tuple[str, str]]) -> list[list[str]]:
    hits = []
    for first, second in pairs:
        if first not in polys or second not in polys:
            continue
        if aabb_gap(polys[first], polys[second]) == 0.0 and intersects(polys[first], polys[second]):
            hits.append([first, second])
    return hits


def nonadjacent_self_pairs(scene: UrdfScene) -> list[tuple[str, str]]:
    adjacent = {
        frozenset((joint.parent, joint.child))
        for joint in scene.joints
    }
    pairs = []
    for index, first in enumerate(MOVING_LEFT_LINKS):
        for second in MOVING_LEFT_LINKS[index + 1:]:
            if frozenset((first, second)) not in adjacent:
                pairs.append((first, second))
    return pairs


def joint_margin_deg(chain: Chain, q_rad: np.ndarray) -> float:
    margins = []
    for name, value in zip(arm_names("left"), q_rad):
        lower, upper = chain.limits[name]
        margins.append(min(value - lower, upper - value))
    return math.degrees(min(margins))


def finite_or_none(value: float) -> float | None:
    return round(value, 3) if math.isfinite(value) else None


def _pair_set(pairs: list[list[str]]) -> set[tuple[str, str]]:
    return {tuple(pair) for pair in pairs}


def find_monotonic_recovery(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Find the first safe sample when the path only escapes existing contacts.

    A recovery path is intentionally stricter than an ordinary path.  The initial
    state may already violate a joint limit or touch the table, but every sampled
    step must improve the joint-limit margin, must not introduce a new collision
    pair, and must not re-enter a violation after becoming safe.
    """
    if not samples or not samples[0]["violates"]:
        return {
            "required": False,
            "ready_for_explicit_motion_approval": False,
            "reason": "시작 상태가 정상 경로 게이트를 이미 통과함",
        }

    first = samples[0]
    initial_pairs = (
        _pair_set(first["cross_arm_collision_pairs"])
        | _pair_set(first["self_collision_pairs"])
        | _pair_set(first["structure_collision_pairs"])
    )
    previous_margin = float(first["joint_limit_margin_deg"])
    previous_collision_count = len(initial_pairs)
    safe_index: int | None = None
    reasons: list[str] = []

    for sample in samples[1:]:
        current_pairs = (
            _pair_set(sample["cross_arm_collision_pairs"])
            | _pair_set(sample["self_collision_pairs"])
            | _pair_set(sample["structure_collision_pairs"])
        )
        margin = float(sample["joint_limit_margin_deg"])
        if not current_pairs.issubset(initial_pairs):
            reasons.append(f"sample {sample['sample']}: 새 충돌 쌍 발생")
            break
        if len(current_pairs) > previous_collision_count:
            reasons.append(f"sample {sample['sample']}: 충돌 쌍 수 증가")
            break
        if margin + 1e-6 < previous_margin:
            reasons.append(f"sample {sample['sample']}: 관절 한계 여유 감소")
            break
        previous_collision_count = len(current_pairs)
        previous_margin = margin
        if not sample["violates"]:
            safe_index = int(sample["sample"])
            break

    if safe_index is None:
        return {
            "required": True,
            "ready_for_explicit_motion_approval": False,
            "reason": reasons[0] if reasons else "감사 구간 안에서 안전 상태에 도달하지 못함",
        }

    if any(sample["violates"] for sample in samples[safe_index + 1:]):
        return {
            "required": True,
            "ready_for_explicit_motion_approval": False,
            "reason": "안전 상태 도달 뒤 위반이 다시 발생함",
        }

    target = samples[safe_index]
    start = np.asarray(samples[0]["joint_deg"], dtype=float)
    target_deg = np.asarray(target["joint_deg"], dtype=float)
    return {
        "required": True,
        "ready_for_explicit_motion_approval": True,
        "reason": "기존 접촉만 단계적으로 해소하고 새 충돌 없이 안전 상태에 도달함",
        "safe_sample": safe_index,
        "target_joint_deg": [round(float(value), 3) for value in target_deg],
        "maximum_joint_delta_deg": round(float(np.max(np.abs(target_deg - start))), 3),
        "joint_limit_margin_deg": round(float(target["joint_limit_margin_deg"]), 3),
    }


def audit_trajectory(plan: dict[str, Any], start_deg: list[float], right_deg: list[float],
                     max_step_deg: float, minimum_margin_deg: float) -> dict[str, Any]:
    if plan.get("motion_command_emitted") is not False:
        raise ValueError("motion_command_emitted=false인 드라이런 계획만 허용합니다")
    if not plan.get("ready_for_collision_review", False):
        raise ValueError("IK 계획이 collision review 게이트를 통과하지 못했습니다")
    stages = plan.get("stages", {})
    if set(stages) != {"pregrasp", "grasp"}:
        raise ValueError("계획에는 pregrasp와 grasp 두 단계가 정확히 있어야 합니다")

    start = np.asarray(start_deg, dtype=float)
    pregrasp = np.asarray(stages["pregrasp"]["joint_deg"], dtype=float)
    grasp = np.asarray(stages["grasp"]["joint_deg"], dtype=float)
    right = np.asarray(right_deg, dtype=float)
    if any(values.shape != (len(ARM_JOINTS),) for values in (start, pregrasp, grasp, right)):
        raise ValueError("좌우 관절각은 shoulder_pan부터 wrist_roll까지 5개여야 합니다")

    scene = UrdfScene(URDF_PATH)
    chain = Chain(URDF_PATH)
    samples = trajectory_samples(start, pregrasp, grasp, max_step_deg)
    self_pairs = nonadjacent_self_pairs(scene)
    structure_pairs = [
        (moving, fixed)
        for moving in MOVING_LEFT_LINKS
        for fixed in STATIC_CRITICAL_LINKS
    ]

    findings = []
    sample_reports = []
    worst_margin = float("inf")
    minimum_cross = float("inf")
    minimum_gripper = float("inf")
    minimum_lidar = float("inf")
    minimum_mast = float("inf")
    for index, (phase, q_deg) in enumerate(samples):
        q_rad = np.radians(q_deg)
        margin = joint_margin_deg(chain, q_rad)
        worst_margin = min(worst_margin, margin)
        positions = dict(zip(arm_names("left"), q_rad))
        positions.update(dict(zip(arm_names("right"), np.radians(right))))
        positions["left_gripper"] = 0.5
        positions["right_finger1_joint"] = 0.0325

        clearance = audit_state(scene, positions)
        minimum_cross = min(minimum_cross, float(clearance["minimum_cross_arm_link_clearance_mm"]))
        minimum_gripper = min(
            minimum_gripper, float(clearance["minimum_gripper_to_other_arm_clearance_mm"])
        )
        minimum_lidar = min(
            minimum_lidar, float(clearance["minimum_arm_to_lidar_surface_clearance_mm"])
        )
        minimum_mast = min(minimum_mast, float(clearance["minimum_mast_centerline_radius_mm"]))

        transforms = scene.link_transforms(positions)
        needed = set(MOVING_LEFT_LINKS + STATIC_CRITICAL_LINKS)
        polys = {
            name: collision_polydata(scene, name, transforms)
            for name in needed
            if name in scene.links
        }
        polys = {name: poly for name, poly in polys.items() if poly is not None}
        self_hits = collision_hits(polys, self_pairs)
        structure_hits = collision_hits(polys, structure_pairs)
        sample_report = {
                "sample": index,
                "phase": phase,
                "joint_deg": [round(float(value), 3) for value in q_deg],
                "joint_limit_margin_deg": round(margin, 3),
                "cross_arm_collision_pairs": clearance["triangle_collision_pairs_cross_arm"],
                "self_collision_pairs": self_hits,
                "structure_collision_pairs": structure_hits,
            }
        sample_report["violates"] = bool(
            margin < minimum_margin_deg or not clearance["passes"] or self_hits or structure_hits
        )
        sample_reports.append(sample_report)
        if sample_report["violates"]:
            findings.append({key: value for key, value in sample_report.items() if key != "violates"})

    recovery = find_monotonic_recovery(sample_reports)
    continuous_ready = not findings or recovery["ready_for_explicit_motion_approval"]

    return {
        "mode": "trajectory_collision_audit_only",
        "motion_command_emitted": False,
        "sample_count": len(samples),
        "start_joint_deg": [round(float(value), 4) for value in start],
        "right_joint_deg": [round(float(value), 4) for value in right],
        "target_stages_joint_deg": {
            "pregrasp": [round(float(value), 4) for value in pregrasp],
            "grasp": [round(float(value), 4) for value in grasp],
        },
        "max_joint_step_deg": max_step_deg,
        "minimum_required_joint_margin_deg": minimum_margin_deg,
        "minimum_observed_joint_margin_deg": round(worst_margin, 3),
        "minimum_clearance_mm": {
            "cross_arm": finite_or_none(minimum_cross),
            "gripper_to_other_arm": finite_or_none(minimum_gripper),
            "arm_to_lidar": finite_or_none(minimum_lidar),
            "mast_centerline_radius": finite_or_none(minimum_mast),
        },
        "failed_sample_count": len(findings),
        "failed_samples": findings,
        "trajectory_ready_for_preview": not findings,
        "continuous_path_ready_for_preview": continuous_ready,
        "start_recovery": recovery,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    start = parser.add_mutually_exclusive_group(required=True)
    start.add_argument("--start-deg", type=float, nargs=5,
                       metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL"))
    start.add_argument("--start-state", type=Path,
                       help="export_current_joint_state.py가 만든 읽기 전용 JSON")
    parser.add_argument("--right-deg", type=float, nargs=5, default=[0.0] * 5,
                        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL"))
    parser.add_argument("--max-step-deg", type=float, default=2.0)
    parser.add_argument("--minimum-margin-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    if args.start_state:
        start_state = json.loads(args.start_state.read_text(encoding="utf-8"))
        if start_state.get("motion_command_emitted") is not False:
            raise SystemExit("motion_command_emitted=false인 시작 상태만 허용합니다")
        start_deg = start_state["joint_degrees"]
    else:
        start_deg = args.start_deg
    try:
        report = audit_trajectory(
            plan, start_deg, args.right_deg, args.max_step_deg, args.minimum_margin_deg
        )
    except ValueError as exc:
        report = {
            "mode": "trajectory_audit_blocked",
            "motion_command_emitted": False,
            "trajectory_ready_for_preview": False,
            "blocking_findings": [str(exc)],
        }
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if args.strict and not report.get("continuous_path_ready_for_preview", False):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
