#!/usr/bin/env python3
"""Search left-arm mount alternatives for upright kitchen-to-center tray motion."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import _limits, base_positions


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = Path("/data/lim/robot-artifacts/pour_20260921/runs/flared03/replacement_hypothesis.urdf")
DEFAULT_PLAN = Path("/data/lim/robot-artifacts/pour_20260921/runs/flared03/plan.json")
DEFAULT_OUTPUT = Path("/data/lim/robot-artifacts/restaurant/mount_search")
KITCHEN_STAGES = (
    ("KITCHEN_CONTACT", np.array([0.39, 0.17, 0.78])),
    ("KITCHEN_LIFT", np.array([0.39, 0.17, 0.84])),
)
POSITION_LIMIT_MM = 2.0
TRAY_CENTER_XY_LIMIT_M = 0.020
TRAY_Z_LIMIT_MM = 2.0
ENDPOINT_TILT_LIMIT_DEG = 2.0
PATH_TILT_LIMIT_DEG = 2.0
JOINT_MARGIN_DEG = 3.0
PATH_SAMPLES = 41
ARM_POINTS = ("shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "tool0")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def materialize_mount_candidate(source: Path, target: Path, rear_offset_m: float, yaw_deg: float) -> dict:
    if rear_offset_m < 0 or not math.isfinite(rear_offset_m) or not math.isfinite(yaw_deg):
        raise ValueError("장착 오프셋과 yaw는 유한하고 rear offset은 음수가 아니어야 합니다")
    tree = ET.parse(source)
    root = tree.getroot()
    origins = {}
    for joint_name in ("left_mount_joint", "left_arm_backing_link_joint"):
        joint = root.find(f"./joint[@name='{joint_name}']")
        if joint is None or joint.find("origin") is None:
            raise ValueError(f"{joint_name} origin이 없습니다")
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", ""), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        if xyz.shape != (3,) or rpy.shape != (3,):
            raise ValueError(f"{joint_name} origin 형식 오류")
        original_xyz = xyz.copy()
        xyz[0] -= rear_offset_m
        rpy[2] += math.radians(yaw_deg)
        origin.set("xyz", " ".join(f"{value:.9g}" for value in xyz))
        origin.set("rpy", " ".join(f"{value:.9g}" for value in rpy))
        origins[joint_name] = {"original_xyz_m": original_xyz.tolist(), "xyz_m": xyz.tolist(),
                               "rpy_rad": rpy.tolist()}
    mount_xyz = np.asarray(origins["left_mount_joint"]["xyz_m"])
    yaw = math.radians(yaw_deg)
    backing_half_x = abs(math.cos(yaw)) * 0.065 + abs(math.sin(yaw)) * 0.035
    backing_half_y = abs(math.sin(yaw)) * 0.065 + abs(math.cos(yaw)) * 0.035
    backing_inside = bool(
        abs(mount_xyz[0]) + backing_half_x <= 0.170
        and abs(mount_xyz[1]) + backing_half_y <= 0.225
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return {
        "original_mount_xyz_m": origins["left_mount_joint"]["original_xyz_m"],
        "candidate_mount_xyz_m": origins["left_mount_joint"]["xyz_m"],
        "candidate_mount_rpy_rad": origins["left_mount_joint"]["rpy_rad"],
        "moved_joints": origins,
        "backing_inside_top_plate": backing_inside,
        "backing_xy_half_extent_m": [backing_half_x, backing_half_y],
        "rear_offset_m": rear_offset_m,
        "yaw_deg": yaw_deg,
    }


def _cup_metrics(chain, q, cup_axis_local):
    transform = chain.transforms(base_positions("left", q))["left_tool0"]
    cup_up = transform[:3, :3] @ cup_axis_local
    tilt = math.degrees(math.acos(float(np.clip(cup_up[2], -1.0, 1.0))))
    return transform[:3, 3].copy(), cup_up, tilt


def _joint_margin_deg(chain, q):
    lower, upper = _limits(chain, "left")
    return math.degrees(float(np.min(np.minimum(q - lower, upper - q))))


def _segment_max_tilt(chain, start, end, cup_axis_local):
    maximum = 0.0
    for fraction in np.linspace(0.0, 1.0, PATH_SAMPLES):
        q = start * (1.0 - fraction) + end * fraction
        maximum = max(maximum, _cup_metrics(chain, q, cup_axis_local)[2])
    return maximum


def _solve_stage(chain, target, previous, cup_axis_local, *, center_region, restarts, iterations,
                 random_seed):
    lower, upper = _limits(chain, "left")
    lower += math.radians(JOINT_MARGIN_DEG)
    upper -= math.radians(JOINT_MARGIN_DEG)
    # 물컵을 잡은 source branch는 wrist roll이 양수다.
    lower[4] = max(lower[4], 0.0)
    rng = np.random.default_rng(random_seed)
    starts = [np.clip(previous, lower, upper)]
    starts.extend(rng.uniform(lower, upper) for _ in range(restarts - 1))

    def residual(q):
        position, cup_up, _ = _cup_metrics(chain, q, cup_axis_local)
        return np.r_[(target - position) * 10.0, cup_up[:2]]

    candidates = []
    for start in starts:
        q = start.copy()
        damping = 1e-3
        for _ in range(iterations):
            raw = residual(q)
            jacobian = np.empty((5, 5))
            for index in range(5):
                probe = q.copy()
                probe[index] = min(q[index] + 1e-5, upper[index])
                if probe[index] == q[index]:
                    probe[index] = max(q[index] - 1e-5, lower[index])
                jacobian[:, index] = (residual(probe) - raw) / (probe[index] - q[index])
            delta = np.linalg.solve(
                jacobian.T @ jacobian + damping * np.eye(5), jacobian.T @ raw,
            )
            candidate = np.clip(q - delta, lower, upper)
            if residual(candidate) @ residual(candidate) < raw @ raw:
                q = candidate
                damping = max(1e-8, damping * 0.4)
            else:
                damping = min(1e5, damping * 5.0)
        position, cup_up, tilt = _cup_metrics(chain, q, cup_axis_local)
        error_mm = float(np.linalg.norm(position - target) * 1000.0)
        xy_error_m = float(np.linalg.norm(position[:2] - target[:2]))
        z_error_mm = abs(float(position[2] - target[2]) * 1000.0)
        margin = _joint_margin_deg(chain, q)
        path_tilt = _segment_max_tilt(chain, previous, q, cup_axis_local)
        position_accepted = (
            xy_error_m <= TRAY_CENTER_XY_LIMIT_M and z_error_mm <= TRAY_Z_LIMIT_MM
            if center_region else error_mm <= POSITION_LIMIT_MM
        )
        accepted = bool(
            position_accepted
            and tilt <= ENDPOINT_TILT_LIMIT_DEG
            and margin >= JOINT_MARGIN_DEG
            and path_tilt <= PATH_TILT_LIMIT_DEG
            and q[4] >= 0.0
        )
        candidates.append({
            "q": q, "position_m": position, "cup_up": cup_up,
            "position_error_mm": error_mm, "endpoint_tilt_deg": tilt,
            "xy_error_m": xy_error_m, "z_error_mm": z_error_mm,
            "position_contract": "tray_center_region" if center_region else "kitchen_exact_point",
            "path_maximum_tilt_deg": path_tilt, "joint_margin_deg": margin,
            "accepted": accepted,
        })
    return min(candidates, key=lambda item: (
        not item["accepted"], item["position_error_mm"], item["path_maximum_tilt_deg"],
        -item["joint_margin_deg"], float(np.linalg.norm(item["q"] - previous)),
    ))


def _point_segment_distance(a, b, c, d):
    # Closest distance between two 3-D line segments.
    u, v, w = b - a, d - c, a - c
    aa, bb, cc = float(u @ u), float(u @ v), float(v @ v)
    dd, ee = float(u @ w), float(v @ w)
    denominator = aa * cc - bb * bb
    small = 1e-12
    s_n, s_d = 0.0, denominator
    t_n, t_d = 0.0, denominator
    if denominator < small:
        s_n, s_d, t_n, t_d = 0.0, 1.0, ee, cc
    else:
        s_n, t_n = bb * ee - cc * dd, aa * ee - bb * dd
        if s_n < 0.0:
            s_n, t_n, t_d = 0.0, ee, cc
        elif s_n > s_d:
            s_n, t_n, t_d = s_d, ee + bb, cc
    if t_n < 0.0:
        t_n = 0.0
        if -dd < 0.0:
            s_n = 0.0
        elif -dd > aa:
            s_n = s_d
        else:
            s_n, s_d = -dd, aa
    elif t_n > t_d:
        t_n = t_d
        if -dd + bb < 0.0:
            s_n = 0.0
        elif -dd + bb > aa:
            s_n = s_d
        else:
            s_n, s_d = -dd + bb, aa
    sc = 0.0 if abs(s_n) < small else s_n / s_d
    tc = 0.0 if abs(t_n) < small else t_n / t_d
    return float(np.linalg.norm(w + sc * u - tc * v))


def _arm_points(chain, side, q):
    transforms = chain.transforms(base_positions(side, q))
    return np.array([transforms[f"{side}_{name}"][:3, 3] for name in ARM_POINTS])


def _centerline_clearance(chain, left_sequences, right_q):
    right = _arm_points(chain, "right", right_q)
    minimum_other = float("inf")
    minimum_self = float("inf")
    for start, end in left_sequences:
        for fraction in np.linspace(0.0, 1.0, PATH_SAMPLES):
            left_q = start * (1.0 - fraction) + end * fraction
            left = _arm_points(chain, "left", left_q)
            for a, b in zip(left[:-1], left[1:]):
                for c, d in zip(right[:-1], right[1:]):
                    minimum_other = min(minimum_other, _point_segment_distance(a, b, c, d))
            for first in range(len(left) - 1):
                for second in range(first + 2, len(left) - 1):
                    minimum_self = min(minimum_self, _point_segment_distance(
                        left[first], left[first + 1], left[second], left[second + 1],
                    ))
    return minimum_other, minimum_self


def evaluate_candidate(model: Path, source: dict, mount: dict, *, source_urdf=DEFAULT_URDF,
                       tray_height_offset_m=0.0, restarts=6, iterations=240):
    chain = BimanualContactChain(model, source["left_config"], source["right_config"])
    held = next(p for p in source["plan"]["poses"] if p["name"] == "POUR_RETURN_CUP")
    parked = next(p for p in source["plan"]["poses"] if p["name"] == "RIGHT_PLACE_HOLD")
    original_chain = BimanualContactChain(source_urdf, source["left_config"], source["right_config"])
    original_q = np.radians(held["left_joint_deg"])
    original_transform = original_chain.transforms(base_positions("left", original_q))["left_tool0"]
    cup_axis_local = original_transform[:3, :3].T @ np.array([0.0, 0.0, 1.0])

    previous = original_q
    stages = []
    sequences = []
    stages_to_solve = (*KITCHEN_STAGES,
                       ("TRAY_ABOVE", np.array([0.0, 0.0, 0.84 + tray_height_offset_m])),
                       ("TRAY_CONTACT", np.array([0.0, 0.0, 0.78 + tray_height_offset_m])))
    for index, (name, target) in enumerate(stages_to_solve):
        result = _solve_stage(
            chain, target, previous, cup_axis_local, center_region=name.startswith("TRAY_"),
            restarts=restarts, iterations=iterations,
            random_seed=2900 + index + round(mount["rear_offset_m"] * 10000) + round(mount["yaw_deg"] * 10),
        )
        q = result.pop("q")
        sequences.append((previous.copy(), q.copy()))
        stages.append({
            "name": name, "target_m": target.tolist(), "joint_deg": np.degrees(q).tolist(),
            **{key: (value.tolist() if isinstance(value, np.ndarray) else value)
               for key, value in result.items()},
        })
        previous = q
    right_q = np.radians(parked["right_joint_deg"])
    other_clearance, self_clearance = _centerline_clearance(chain, sequences, right_q)
    overall = all(stage["accepted"] for stage in stages) and mount["backing_inside_top_plate"]
    return {
        **mount, "tray_height_offset_m": tray_height_offset_m,
        "model": str(model), "model_sha256": sha256(model), "stages": stages,
        "accepted": overall,
        "maximum_path_tilt_deg": max(stage["path_maximum_tilt_deg"] for stage in stages),
        "maximum_position_error_mm": max(stage["position_error_mm"] for stage in stages),
        "minimum_joint_margin_deg": min(stage["joint_margin_deg"] for stage in stages),
        "minimum_left_right_centerline_clearance_m": other_clearance,
        "minimum_left_self_centerline_clearance_m": self_clearance,
        "collision_scope": "centerline_auxiliary_not_mesh_collision_certification",
    }


def search(source_urdf: Path, source_plan: Path, output_dir: Path, *, offsets_mm, yaws_deg,
           tray_heights_mm=(0,), restarts=6, iterations=240):
    output_dir.mkdir(parents=True, exist_ok=True)
    models_dir = output_dir / "models"
    source = json.loads(source_plan.read_text())
    candidates = []
    for offset_mm in offsets_mm:
        for yaw_deg in yaws_deg:
            for height_mm in tray_heights_mm:
                label = (f"rear_{int(offset_mm):03d}mm_yaw_{yaw_deg:+03d}deg_"
                         f"tray_{int(height_mm):+04d}mm")
                model = models_dir / f"{label}.urdf"
                mount = materialize_mount_candidate(source_urdf, model, offset_mm / 1000.0, yaw_deg)
                result = evaluate_candidate(
                    model, source, mount, source_urdf=source_urdf,
                    tray_height_offset_m=height_mm / 1000.0,
                    restarts=restarts, iterations=iterations,
                )
                result["label"] = label
                candidates.append(result)
                print(label, "PASS" if result["accepted"] else "reject",
                      f"tilt={result['maximum_path_tilt_deg']:.3f}",
                      f"error={result['maximum_position_error_mm']:.3f}", flush=True)
    ranked = sorted(candidates, key=lambda item: (
        not item["accepted"], item["maximum_position_error_mm"], item["maximum_path_tilt_deg"],
        -item["minimum_left_right_centerline_clearance_m"], item["rear_offset_m"], abs(item["yaw_deg"]),
    ))
    report = {
        "schema": "tray_mount_search_v1", "source_urdf": str(source_urdf),
        "source_urdf_sha256": sha256(source_urdf), "source_plan": str(source_plan),
        "source_plan_sha256": sha256(source_plan),
        "search_space": {"rear_offsets_mm": list(offsets_mm), "yaw_deg": list(yaws_deg),
                         "tray_height_offsets_mm": list(tray_heights_mm)},
        "acceptance": {
            "kitchen_position_error_mm": POSITION_LIMIT_MM,
            "tray_center_xy_radius_m": TRAY_CENTER_XY_LIMIT_M,
            "tray_z_error_mm": TRAY_Z_LIMIT_MM,
            "endpoint_tilt_deg": ENDPOINT_TILT_LIMIT_DEG,
            "path_tilt_deg": PATH_TILT_LIMIT_DEG,
            "joint_margin_deg": JOINT_MARGIN_DEG,
        },
        "candidates": ranked, "accepted_count": sum(item["accepted"] for item in candidates),
        "hardware_accessed": False, "gpu_used": False,
        "limitations": [
            "centerline clearance is auxiliary and does not certify mesh self-collision",
            "endpoint and joint-linear CPU IK do not certify dynamics, payload torque, or physical mounting",
        ],
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--source-plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--offsets-mm", type=int, nargs="+", default=[0, 20, 40, 60, 80, 100])
    parser.add_argument("--yaws-deg", type=int, nargs="+", default=[-30, -15, 0, 15, 30])
    parser.add_argument("--tray-heights-mm", type=int, nargs="+", default=[0])
    parser.add_argument("--restarts", type=int, default=6)
    parser.add_argument("--iterations", type=int, default=240)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    search(args.urdf, args.source_plan, args.output_dir, offsets_mm=args.offsets_mm,
           yaws_deg=args.yaws_deg, tray_heights_mm=args.tray_heights_mm,
           restarts=args.restarts, iterations=args.iterations)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
