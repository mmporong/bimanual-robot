#!/usr/bin/env python3
"""Generate a CPU-only endpoint plan for horizontal body-side grasps.

The planner reads the committed URDF and never opens a camera, ROS connection,
or servo bus.  It deliberately stops before close/lift because this model has no
validated contact or object-attachment model.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
IK_SCRIPT_DIR = REPO_ROOT / "src/hold_flow_description/scripts"
if str(IK_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(IK_SCRIPT_DIR))

from solve_task_poses import ARM_JOINTS, Chain, URDF_PATH, base_positions  # noqa: E402


SCENE_ASSUMPTIONS = {
    "status": "illustrative_only_not_measured",
    "table_surface_z_m": 0.6931,
    "table_center_xy_m": [0.55, 0.0],
    "table_size_xy_m": [0.60, 1.0],
    "cup_center_xy_m": [0.32, 0.17],
    "cup_radius_m": 0.035,
    "cup_height_m": 0.12,
    "bottle_center_xy_m": [0.39, -0.17],
    "bottle_radius_m": 0.038,
    "bottle_height_m": 0.22,
    "right_parked_joint_deg": [0.0, -68.0, 92.0, -22.0, 0.0],
    "left_stock_gripper_rad": 0.5,
    "right_gripper_open_m": 0.0325,
}

POSITION_LIMIT_MM = 2.0
HORIZONTAL_LIMIT_DEG = 2.0
JOINT_MARGIN_LIMIT_DEG = 3.0
HEADING_LIMIT_DEG = 30.0
ARM_MOUNT_XY_M = {
    "left": np.array([0.020, 0.170]),
    "right": np.array([0.020, -0.170]),
}


@dataclass(frozen=True)
class ObjectSpec:
    name: str
    side: str
    center_xy_m: tuple[float, float]
    height_m: float
    radius_m: float

    def __post_init__(self) -> None:
        values = np.asarray((*self.center_xy_m, self.height_m, self.radius_m), dtype=float)
        if self.side not in {"left", "right"}:
            raise ValueError("side는 left 또는 right여야 합니다")
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("물체 치수와 좌표는 유한한 숫자여야 합니다")
        if self.height_m <= 0.0 or self.radius_m <= 0.0:
            raise ValueError("물체 높이와 반지름은 양수여야 합니다")


def _vector3(value: Any, label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label}은 유한한 3벡터여야 합니다")
    return vector


def object_body_center(spec: ObjectSpec, table_surface_z_m: float) -> np.ndarray:
    surface = float(table_surface_z_m)
    if not math.isfinite(surface):
        raise ValueError("table_surface_z_m은 유한해야 합니다")
    return np.array([*spec.center_xy_m, surface + spec.height_m / 2.0])


def outward_heading(spec: ObjectSpec) -> np.ndarray:
    delta = np.asarray(spec.center_xy_m) - ARM_MOUNT_XY_M[spec.side]
    norm = float(np.linalg.norm(delta))
    if norm <= 1e-9:
        raise ValueError("물체 중심과 팔 장착점이 겹쳐 접근 방향을 정할 수 없습니다")
    return np.array([delta[0] / norm, delta[1] / norm, 0.0])


def grasp_axes(transform: np.ndarray, side: str) -> tuple[np.ndarray, np.ndarray]:
    """Return nominal approach and transverse/closing axes in base coordinates."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("tool0 transform은 유한한 4x4 행렬이어야 합니다")
    if side == "left":
        return transform[:3, 2].copy(), transform[:3, 0].copy()
    if side == "right":
        return -transform[:3, 1].copy(), transform[:3, 0].copy()
    raise ValueError("side는 left 또는 right여야 합니다")


def stage_targets(spec: ObjectSpec, table_surface_z_m: float) -> list[tuple[str, np.ndarray]]:
    center = object_body_center(spec, table_surface_z_m)
    heading = outward_heading(spec)
    return [
        ("reorient", center - heading * (spec.radius_m + 0.100)),
        ("pregrasp", center - heading * (spec.radius_m + 0.050)),
        ("approach_body_center", center),
    ]


def constraint_residual(
    chain: Chain, side: str, q: np.ndarray, target_m: np.ndarray
) -> np.ndarray:
    """Five constraints: xyz position plus horizontal approach/transverse axes."""
    q = np.asarray(q, dtype=float)
    target = _vector3(target_m, "target_m")
    if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
        raise ValueError("q는 유한한 5관절 벡터여야 합니다")
    transform = chain.transforms(base_positions(side, q))[f"{side}_tool0"]
    approach, closing = grasp_axes(transform, side)
    return np.concatenate((target - transform[:3, 3], [-approach[2], -closing[2]]))


def _limits(chain: Chain, side: str) -> tuple[np.ndarray, np.ndarray]:
    names = [f"{side}_{name}" for name in ARM_JOINTS]
    try:
        lower = np.array([chain.limits[name][0] for name in names], dtype=float)
        upper = np.array([chain.limits[name][1] for name in names], dtype=float)
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{side} 팔 관절 한계를 읽을 수 없습니다") from exc
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
        raise ValueError(f"{side} 팔 관절 한계가 유효하지 않습니다")
    return lower, upper


def solve_horizontal_endpoint(
    chain: Chain,
    side: str,
    target_m: np.ndarray,
    seed_q: np.ndarray | None = None,
    *,
    restarts: int = 12,
    random_seed: int = 29,
    iterations: int = 240,
) -> np.ndarray:
    """Bounded numerical DLS without adding a sixth orientation constraint."""
    if isinstance(restarts, bool) or not isinstance(restarts, int) or restarts < 1:
        raise ValueError("restarts는 양의 정수여야 합니다")
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations < 1:
        raise ValueError("iterations는 양의 정수여야 합니다")
    target = _vector3(target_m, "target_m")
    lower, upper = _limits(chain, side)
    margin = math.radians(JOINT_MARGIN_LIMIT_DEG)
    safe_lower, safe_upper = lower + margin, upper - margin
    if np.any(safe_lower >= safe_upper):
        raise ValueError("3도 관절 여유를 적용할 수 없는 관절 한계입니다")

    rng = np.random.default_rng(random_seed)
    starts = []
    if seed_q is not None:
        seed = np.asarray(seed_q, dtype=float)
        if seed.shape != lower.shape or not np.all(np.isfinite(seed)):
            raise ValueError("seed_q는 유한한 5관절 벡터여야 합니다")
        starts.append(np.clip(seed, safe_lower, safe_upper))
    starts.append(np.clip(np.zeros_like(lower), safe_lower, safe_upper))
    while len(starts) < restarts:
        starts.append(rng.uniform(safe_lower, safe_upper))

    best_q: np.ndarray | None = None
    best_cost = float("inf")
    weights = np.array([10.0, 10.0, 10.0, 1.0, 1.0])
    for start in starts[:restarts]:
        q = start.copy()
        damping = 1e-3
        weighted = constraint_residual(chain, side, q, target) * weights
        cost = float(weighted @ weighted)
        for _ in range(iterations):
            raw = constraint_residual(chain, side, q, target)
            position_ok = np.linalg.norm(raw[:3]) * 1000.0 <= POSITION_LIMIT_MM * 0.5
            axes_ok = max(abs(raw[3]), abs(raw[4])) <= math.sin(math.radians(0.5))
            if position_ok and axes_ok:
                break
            weighted = raw * weights
            jacobian = np.empty((5, len(q)))
            step = 1e-5
            for index in range(len(q)):
                probe = q.copy()
                probe[index] = min(probe[index] + step, safe_upper[index])
                actual_step = probe[index] - q[index]
                if actual_step <= 0.0:
                    probe[index] = max(q[index] - step, safe_lower[index])
                    actual_step = probe[index] - q[index]
                jacobian[:, index] = (
                    constraint_residual(chain, side, probe, target) * weights - weighted
                ) / actual_step
            normal = jacobian.T @ jacobian + damping * np.eye(len(q))
            delta = np.linalg.solve(normal, jacobian.T @ weighted)
            candidate = np.clip(q - delta, safe_lower, safe_upper)
            candidate_weighted = constraint_residual(chain, side, candidate, target) * weights
            candidate_cost = float(candidate_weighted @ candidate_weighted)
            if candidate_cost < cost:
                q, cost = candidate, candidate_cost
                damping = max(1e-8, damping * 0.4)
            else:
                damping = min(1e5, damping * 5.0)
        if cost < best_cost:
            best_q, best_cost = q.copy(), cost
    if best_q is None or not np.all(np.isfinite(best_q)):
        raise RuntimeError(f"{side} endpoint IK가 유한한 후보를 만들지 못했습니다")
    return best_q


def measure_stage(
    chain: Chain,
    side: str,
    q: np.ndarray,
    target_m: np.ndarray,
    heading: np.ndarray,
) -> dict[str, Any]:
    q = np.asarray(q, dtype=float)
    target = _vector3(target_m, "target_m")
    heading = _vector3(heading, "heading")
    if q.shape != (len(ARM_JOINTS),) or not np.all(np.isfinite(q)):
        raise ValueError("solver 결과가 유한한 5관절 벡터가 아닙니다")
    transform = np.asarray(
        chain.transforms(base_positions(side, q))[f"{side}_tool0"], dtype=float
    )
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("FK 결과가 유한한 4x4 행렬이 아닙니다")
    approach, closing = grasp_axes(transform, side)
    horizontal = approach.copy()
    horizontal[2] = 0.0
    horizontal_norm = float(np.linalg.norm(horizontal))
    if horizontal_norm <= 1e-9:
        heading_error = 180.0
    else:
        dot = float(np.clip(np.dot(horizontal / horizontal_norm, heading), -1.0, 1.0))
        heading_error = math.degrees(math.acos(dot))
    approach_horizontal_error = math.degrees(math.asin(float(np.clip(abs(approach[2]), 0, 1))))
    closing_horizontal_error = math.degrees(math.asin(float(np.clip(abs(closing[2]), 0, 1))))
    lower, upper = _limits(chain, side)
    joint_margin = math.degrees(float(np.min(np.minimum(q - lower, upper - q))))
    return {
        "position_m": [round(float(value), 6) for value in transform[:3, 3]],
        "approach_axis": [round(float(value), 6) for value in approach],
        "closing_axis": [round(float(value), 6) for value in closing],
        "position_error_mm": round(float(np.linalg.norm(transform[:3, 3] - target) * 1000.0), 4),
        "approach_horizontal_error_deg": round(approach_horizontal_error, 4),
        "closing_horizontal_error_deg": round(closing_horizontal_error, 4),
        "approach_heading_error_deg": round(heading_error, 4),
        "joint_margin_deg": round(joint_margin, 4),
    }


def validate_measurement(measured: dict[str, Any]) -> list[str]:
    required = {
        "position_error_mm": POSITION_LIMIT_MM,
        "approach_horizontal_error_deg": HORIZONTAL_LIMIT_DEG,
        "closing_horizontal_error_deg": HORIZONTAL_LIMIT_DEG,
        "approach_heading_error_deg": HEADING_LIMIT_DEG,
    }
    reasons = []
    for key, limit in required.items():
        value = measured.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            reasons.append(f"{key}가 유한한 숫자가 아님")
        elif value > limit:
            reasons.append(f"{key}={value:.4f} > {limit:.4f}")
    margin = measured.get("joint_margin_deg")
    if isinstance(margin, bool) or not isinstance(margin, (int, float)) or not math.isfinite(margin):
        reasons.append("joint_margin_deg가 유한한 숫자가 아님")
    elif margin < JOINT_MARGIN_LIMIT_DEG:
        reasons.append(f"joint_margin_deg={margin:.4f} < {JOINT_MARGIN_LIMIT_DEG:.4f}")
    for key in ("position_m", "approach_axis", "closing_axis"):
        try:
            _vector3(measured.get(key), key)
        except (TypeError, ValueError):
            reasons.append(f"{key}가 유한한 3벡터가 아님")
    return reasons


def plan_side(chain: Chain, spec: ObjectSpec, table_surface_z_m: float) -> dict[str, Any]:
    heading = outward_heading(spec)
    stages = []
    blocked_reasons = []
    seed_q = None
    for index, (name, target) in enumerate(stage_targets(spec, table_surface_z_m)):
        try:
            q = solve_horizontal_endpoint(
                chain, spec.side, target, seed_q, random_seed=29 + index + (100 if spec.side == "right" else 0)
            )
            measured = measure_stage(chain, spec.side, q, target, heading)
            reasons = validate_measurement(measured)
            accepted = not reasons
            if accepted:
                seed_q = q
        except (ArithmeticError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            q = None
            measured = {"solver_error": str(exc)}
            reasons = [f"solver_failure: {exc}"]
            accepted = False
        stage = {
            "name": name,
            "target_m": [round(float(value), 6) for value in target],
            "measured": measured,
            "accepted": accepted,
            "accepted_scope": "endpoint_ik_only_before_envelope",
        }
        if q is not None:
            stage["joint_deg"] = [round(math.degrees(float(value)), 4) for value in q]
        if reasons:
            stage["rejection_reasons"] = reasons
            blocked_reasons.extend(f"{name}: {reason}" for reason in reasons)
        stages.append(stage)

    blocked_reasons.extend([
        "close: planned_blocked_no_validated_gripper_contact_model",
        "lift: planned_blocked_no_validated_object_attachment_model",
    ])
    return {
        "object": spec.name,
        "tool_frame": f"{spec.side}_tool0",
        "body_center_m": [round(float(value), 6) for value in object_body_center(spec, table_surface_z_m)],
        "outward_heading": [round(float(value), 6) for value in heading],
        "nominal_axes": {
            "approach_local": "+Z" if spec.side == "left" else "-Y",
            "closing_local": "+X",
        },
        "constraint_count": 5,
        "orientation_yaw_free": True,
        "path_checked": False,
        "overall_accepted": False,
        "stages": stages,
        "blocked": True,
        "blocked_reasons": blocked_reasons,
        "planned_blocked": ["close", "lift"],
    }


def build_report(urdf_path: Path = URDF_PATH) -> dict[str, Any]:
    path = Path(urdf_path)
    chain = Chain(path)
    table = float(SCENE_ASSUMPTIONS["table_surface_z_m"])
    specs = [
        ObjectSpec("cup", "left", tuple(SCENE_ASSUMPTIONS["cup_center_xy_m"]),
                   float(SCENE_ASSUMPTIONS["cup_height_m"]), float(SCENE_ASSUMPTIONS["cup_radius_m"])),
        ObjectSpec("bottle", "right", tuple(SCENE_ASSUMPTIONS["bottle_center_xy_m"]),
                   float(SCENE_ASSUMPTIONS["bottle_height_m"]), float(SCENE_ASSUMPTIONS["bottle_radius_m"])),
    ]
    return {
        "schema": "body_side_grasp_v1",
        "mode": "cpu_offline_endpoint_planner",
        "urdf": str(path.resolve()),
        "urdf_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "planner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scene": dict(SCENE_ASSUMPTIONS),
        "solver": "bounded_numerical_dls_position3_approach_z0_closing_z0",
        "modeling_limits": [
            "tool0 axes are nominal URDF axes; actual FinRay contact geometry and TCP are unmeasured",
            "left tool0 +X is treated as a nominal transverse/closing axis, not a validated contact tangent",
            "endpoint feasibility does not prove a collision-free path, stable contact, close, or lift",
        ],
        "acceptance_limits": {
            "position_error_mm": POSITION_LIMIT_MM,
            "axis_horizontal_error_deg": HORIZONTAL_LIMIT_DEG,
            "joint_margin_deg": JOINT_MARGIN_LIMIT_DEG,
            "approach_heading_error_deg": HEADING_LIMIT_DEG,
        },
        "path_checked": False,
        "sides": {spec.side: plan_side(chain, spec, table) for spec in specs},
        "motion_command_emitted": False,
        "object_attached": False,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="새 JSON 출력 경로. 기존 파일은 덮어쓰지 않습니다")
    parser.add_argument("--urdf", type=Path, default=URDF_PATH)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_report(args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
