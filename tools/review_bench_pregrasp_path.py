#!/usr/bin/env python3
"""저장된 왼팔 snapshot에서 컵 상부 후보까지의 경로를 오프라인 검토한다.

raw tick으로 양자화한 현재→CLEARANCE_LIFT→두 끝점을 최대 10 tick 간격으로
샘플링한다. URDF 충돌 메시만 읽으며 포트·카메라·프로세스·모터에 접근하지 않는다.
결과는 sampled-only 검토이고 오른팔/책상 extrinsic 미측정 때문에 실행 승인이 아니다.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import vtk


REPO_ROOT = Path(__file__).resolve().parents[1]
CAD_DIR = REPO_ROOT / "design/cad"
for directory in (REPO_ROOT / "tools", CAD_DIR):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from audit_pick_trajectory import MOVING_LEFT_LINKS, collision_hits, nonadjacent_self_pairs  # noqa: E402
from audit_urdf_clearance import collision_polydata  # noqa: E402
from ik_pick_place import apply_joint_reference_limits, digest  # noqa: E402
from plan_body_side_grasp import Chain, URDF_PATH  # noqa: E402
from render_design_handoff import UrdfScene, vtk_matrix  # noqa: E402
from servo.joint_reference import (  # noqa: E402
    JOINT_ORDER,
    joint_deg_to_raw,
    raw_to_joint_deg,
    validate_reference,
)


SCHEMA = "bench_pregrasp_path_review_v1"
MAX_RAW_STEP_TICKS = 10
GRIPPER_ASSUMPTIONS_RAD = (0.0, 0.5, 1.0, 1.74533)
CUP_RADIUS_MM = 40.0
CUP_HEIGHT_MM = 100.0
CUP_AABB_MARGIN_MM = 5.0
CLEARANCE_PROFILES_DEG = {
    "shoulder_only": {"shoulder_lift": -30.0},
    "retract_and_lift": {"shoulder_lift": -30.0, "elbow_flex": 15.0},
    "retract_and_raise_high": {"shoulder_lift": -45.0, "elbow_flex": 15.0},
}


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _finite_vector(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length or any(
            type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise ValueError(f"{label}: 유한한 숫자 {length}개가 필요합니다")
    return [float(item) for item in value]


def _raw_vector(value: Any, length: int, label: str) -> list[int]:
    values = _finite_vector(value, length, label)
    if any(not item.is_integer() for item in values):
        raise ValueError(f"{label}: 정수 tick {length}개가 필요합니다")
    return [int(item) for item in values]


def interpolate_raw_segment(start: list[int], end: list[int], max_step_ticks: int = MAX_RAW_STEP_TICKS) -> list[list[int]]:
    """끝점을 포함하고 각 축의 인접 변화가 max_step_ticks 이하인 raw 표본을 만든다."""
    if max_step_ticks <= 0:
        raise ValueError("max_step_ticks는 양수여야 합니다")
    if len(start) != len(end) or not start:
        raise ValueError("raw 시작/끝 벡터 크기가 같아야 합니다")
    maximum = max(abs(last - first) for first, last in zip(start, end))
    steps = max(1, math.ceil(maximum / max_step_ticks))
    return [
        [int(round(first + (last - first) * index / steps)) for first, last in zip(start, end)]
        for index in range(1, steps + 1)
    ]


def sampled_raw_path(waypoints: list[tuple[str, list[int]]], max_step_ticks: int = MAX_RAW_STEP_TICKS) -> list[dict[str, Any]]:
    if not waypoints:
        raise ValueError("waypoint가 필요합니다")
    samples = [{"sample": 0, "phase": waypoints[0][0], "raw_ticks": waypoints[0][1].copy()}]
    for phase, target in waypoints[1:]:
        for raw in interpolate_raw_segment(samples[-1]["raw_ticks"], target, max_step_ticks):
            samples.append({"sample": len(samples), "phase": phase, "raw_ticks": raw})
    return samples


def _validate_inputs(snapshot: dict[str, Any], candidates: dict[str, Any], calibration_bytes: bytes) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    urdf_sha = _sha(URDF_PATH.read_bytes())
    calibration_sha = _sha(calibration_bytes)
    if snapshot.get("hardware_accessed") is not True or snapshot.get("motion_command_emitted") is not False:
        raise ValueError("현재 입력은 읽기 전용 실물 snapshot이어야 합니다")
    if snapshot.get("torque") != [0] * 6:
        raise ValueError("현재 snapshot의 왼팔 토크 6개가 모두 OFF여야 합니다")
    current_raw = _raw_vector(snapshot.get("raw_ticks"), 6, "snapshot.raw_ticks")
    if snapshot.get("calibration_sha256") != calibration_sha:
        raise ValueError("현재 snapshot과 calibration 해시가 다릅니다")
    reference = snapshot.get("joint_reference")
    validate_reference(reference, calibration_bytes, urdf_sha)
    current_deg = raw_to_joint_deg(current_raw[:5], calibration_bytes, reference)
    if not np.allclose(_finite_vector(snapshot.get("joint_deg"), 5, "snapshot.joint_deg"), current_deg,
                       rtol=0, atol=1e-9):
        raise ValueError("현재 snapshot의 raw와 joint_deg가 다릅니다")

    if candidates.get("schema") != "bench_cup_pregrasp_candidates_v1":
        raise ValueError("후보 schema 오류")
    if candidates.get("hardware_accessed") is not False or candidates.get("motion_command_emitted") is not False:
        raise ValueError("오프라인 후보만 허용합니다")
    if candidates.get("physical_execution_ready") is not False or candidates.get("start_to_endpoint_path_verified") is not False:
        raise ValueError("실물 실행 또는 기존 경로 승인으로 표시된 후보는 허용하지 않습니다")
    if candidates.get("calibration_sha256") != calibration_sha:
        raise ValueError("후보와 calibration 해시가 다릅니다")
    if candidates.get("urdf_sha256") != urdf_sha:
        raise ValueError("후보와 현행 URDF 해시가 다릅니다")
    if candidates.get("snapshot_sha256") != digest(snapshot):
        raise ValueError("후보가 현재 snapshot에서 생성되지 않았습니다")
    candidate_reference = candidates.get("joint_reference")
    validate_reference(candidate_reference, calibration_bytes, urdf_sha)
    if candidate_reference != reference:
        raise ValueError("현재 snapshot과 후보의 joint reference가 다릅니다")
    if candidates.get("endpoints_ready_for_path_review") is not True:
        raise ValueError("끝점 검토 게이트를 통과한 후보가 아닙니다")

    stages = candidates.get("stages")
    expected_phases = ["ALIGN_ABOVE_BEHIND_CUP", "PREGRASP_ABOVE_CUP"]
    if not isinstance(stages, list) or [stage.get("phase") for stage in stages] != expected_phases:
        raise ValueError("후보에는 정해진 두 상부 끝점이 순서대로 있어야 합니다")
    for stage in stages:
        if stage.get("endpoint_violations") != []:
            raise ValueError(f"{stage['phase']}: 끝점 위반이 남아 있습니다")
        raw = _raw_vector(stage.get("arm_raw_ticks"), 5, f"{stage['phase']}.arm_raw_ticks")
        degrees = _finite_vector(stage.get("joint_deg"), 5, f"{stage['phase']}.joint_deg")
        if joint_deg_to_raw(degrees, calibration_bytes, reference) != raw:
            raise ValueError(f"{stage['phase']}: joint_deg와 raw 양자화가 다릅니다")

    measurement = candidates.get("measurement")
    if (not isinstance(measurement, dict)
            or measurement.get("frame") != "left_arm_base_center"
            or measurement.get("same_table_surface") is not True
            or measurement.get("physical_execution_ready") is not False):
        raise ValueError("동일 책상면 컵 실측이 필요합니다")
    for key in ("cup_forward", "cup_lateral"):
        item = measurement.get(key, {})
        if (item.get("unit") != "m" or type(item.get("value")) not in (int, float)
                or not math.isfinite(item["value"])):
            raise ValueError(f"{key}: left_arm_base_center 기준 유한한 m 값이 필요합니다")
    diameter = measurement.get("cup_diameter", {})
    height = measurement.get("cup_height", {})
    if diameter.get("unit") != "m" or not math.isclose(diameter.get("value", -1), 0.08, abs_tol=1e-9):
        raise ValueError("이 검토는 반지름 40 mm 컵에만 적용됩니다")
    if height.get("unit") != "m" or not math.isclose(height.get("value", -1), 0.1, abs_tol=1e-9):
        raise ValueError("이 검토는 높이 100 mm 컵에만 적용됩니다")
    return reference, stages


def _local_collision_meshes(scene: UrdfScene) -> dict[str, vtk.vtkPolyData]:
    """STL/primitive를 한 번만 읽어 링크 좌표계 collision mesh로 보관한다."""
    identity = {name: np.eye(4) for name in scene.links}
    meshes = {name: collision_polydata(scene, name, identity) for name in MOVING_LEFT_LINKS}
    missing = [name for name, mesh in meshes.items() if mesh is None]
    if missing:
        raise ValueError(f"필수 왼팔 collision mesh 누락: {', '.join(missing)}")
    return meshes


def _transform_mesh(mesh: vtk.vtkPolyData, matrix: np.ndarray) -> vtk.vtkPolyData:
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix(matrix))
    transformed = vtk.vtkTransformPolyDataFilter()
    transformed.SetInputData(mesh)
    transformed.SetTransform(transform)
    transformed.Update()
    output = vtk.vtkPolyData()
    output.DeepCopy(transformed.GetOutput())
    return output


def _bounds_overlap(bounds: tuple[float, ...], box: list[float]) -> bool:
    return all(bounds[index] <= box[index + 1] and bounds[index + 1] >= box[index]
               for index in (0, 2, 4))


def _counts(values: list[list[str]]) -> dict[str, int]:
    counter = Counter("|".join(pair) for pairs in values for pair in pairs)
    return dict(sorted(counter.items()))


def _phase_counts(reports: list[dict[str, Any]], key: str) -> dict[str, int]:
    phases = {report["phase"] for report in reports}
    return {phase: sum(bool(report[key]) for report in reports if report["phase"] == phase)
            for phase in sorted(phases)}


def build(snapshot: dict[str, Any], candidates: dict[str, Any], calibration_bytes: bytes,
          clearance_profile: str = "retract_and_lift") -> dict[str, Any]:
    reference, stages = _validate_inputs(snapshot, candidates, calibration_bytes)
    if clearance_profile not in CLEARANCE_PROFILES_DEG:
        raise ValueError(f"지원하지 않는 clearance profile: {clearance_profile}")
    current_raw = snapshot["raw_ticks"][:5]
    current_deg = raw_to_joint_deg(current_raw, calibration_bytes, reference)
    clearance_deg = current_deg.copy()
    for name, delta_deg in CLEARANCE_PROFILES_DEG[clearance_profile].items():
        clearance_deg[JOINT_ORDER.index(name)] += delta_deg
    clearance_raw = joint_deg_to_raw(clearance_deg, calibration_bytes, reference)
    waypoints = [("CURRENT", current_raw), ("CLEARANCE_LIFT", clearance_raw)]
    waypoints.extend((stage["phase"], stage["arm_raw_ticks"]) for stage in stages)
    samples = sampled_raw_path(waypoints)
    limit_chain = Chain(URDF_PATH)
    apply_joint_reference_limits(limit_chain, calibration_bytes, reference)
    minimum_limit_margin_deg = float("inf")
    for sample in samples:
        sample["joint_deg"] = raw_to_joint_deg(sample["raw_ticks"], calibration_bytes, reference)
        margins = []
        for name, value_deg in zip(JOINT_ORDER, sample["joint_deg"]):
            lower, upper = limit_chain.limits[f"left_{name}"]
            value = math.radians(value_deg)
            margin = min(value - lower, upper - value)
            if margin < -1e-9:
                raise ValueError(f"sample {sample['sample']} {name}: URDF/raw 가동 범위 교집합 밖")
            margins.append(math.degrees(margin))
        sample["joint_limit_margin_deg"] = min(margins)
        minimum_limit_margin_deg = min(minimum_limit_margin_deg, sample["joint_limit_margin_deg"])

    scene = UrdfScene(URDF_PATH)
    local_meshes = _local_collision_meshes(scene)
    self_pairs = nonadjacent_self_pairs(scene)
    zero_transforms = scene.link_transforms({})
    table_z_mm = float(zero_transforms["left_base_link"][2, 3])
    measurement = candidates["measurement"]
    base = zero_transforms["left_base_link"][:3, 3]
    cup_x = float(base[0] + measurement["cup_forward"]["value"] * 1000.0)
    cup_y = float(base[1] + measurement["cup_lateral"]["value"] * 1000.0)
    cup_aabb = [
        cup_x - CUP_RADIUS_MM - CUP_AABB_MARGIN_MM,
        cup_x + CUP_RADIUS_MM + CUP_AABB_MARGIN_MM,
        cup_y - CUP_RADIUS_MM - CUP_AABB_MARGIN_MM,
        cup_y + CUP_RADIUS_MM + CUP_AABB_MARGIN_MM,
        table_z_mm - CUP_AABB_MARGIN_MM,
        table_z_mm + CUP_HEIGHT_MM + CUP_AABB_MARGIN_MM,
    ]

    reports_by_assumption: list[dict[str, Any]] = []
    candidate_phases = {stage["phase"] for stage in stages}
    for gripper_rad in GRIPPER_ASSUMPTIONS_RAD:
        reports = []
        for sample in samples:
            positions = dict(zip((f"left_{name}" for name in JOINT_ORDER), np.radians(sample["joint_deg"])))
            positions["left_gripper"] = gripper_rad
            transforms = scene.link_transforms(positions)
            polys = {name: _transform_mesh(mesh, transforms[name]) for name, mesh in local_meshes.items()}
            self_hits = collision_hits(polys, self_pairs)
            cup_links = sorted(name for name, poly in polys.items() if _bounds_overlap(poly.GetBounds(), cup_aabb))
            minimum_z = min(float(poly.GetBounds()[4]) for poly in polys.values())
            reports.append({
                "sample": sample["sample"],
                "phase": sample["phase"],
                "raw_ticks": sample["raw_ticks"],
                "joint_deg": [round(value, 6) for value in sample["joint_deg"]],
                "joint_limit_margin_deg": round(sample["joint_limit_margin_deg"], 3),
                "minimum_height_above_assumed_table_mm": round(minimum_z - table_z_mm, 3),
                "self_collision_pairs": self_hits,
                "cup_aabb_overlap_links": cup_links,
            })

        start_height = reports[0]["minimum_height_above_assumed_table_mm"]
        first_segment = [report for report in reports if report["phase"] in {"CURRENT", "CLEARANCE_LIFT"}]
        first_clear = next((report["sample"] for report in reports
                            if report["minimum_height_above_assumed_table_mm"] >= 0.0), None)
        reentered = first_clear is not None and any(
            report["minimum_height_above_assumed_table_mm"] < 0.0
            for report in reports[first_clear + 1:]
        )
        candidate_reports = [report for report in reports if report["phase"] in candidate_phases]
        reports_by_assumption.append({
            "gripper_rad": gripper_rad,
            "sample_count": len(reports),
            "self_collision_sample_count": sum(bool(report["self_collision_pairs"]) for report in reports),
            "self_collision_pair_occurrences": _counts([report["self_collision_pairs"] for report in reports]),
            "cup_aabb_overlap_sample_count": sum(bool(report["cup_aabb_overlap_links"]) for report in reports),
            "cup_aabb_overlap_link_occurrences": dict(sorted(Counter(
                name for report in reports for name in report["cup_aabb_overlap_links"]).items())),
            "table_below_sample_count": sum(report["minimum_height_above_assumed_table_mm"] < 0.0 for report in reports),
            "minimum_height_above_assumed_table_mm": min(
                report["minimum_height_above_assumed_table_mm"] for report in reports),
            "start_floor_report": {
                "minimum_height_above_assumed_table_mm": start_height,
                "penetration_mm": round(max(0.0, -start_height), 3),
                "reported_separately_from_candidate_connections": True,
            },
            "first_segment_floor_behavior": {
                "minimum_height_above_assumed_table_mm": min(
                    report["minimum_height_above_assumed_table_mm"] for report in first_segment),
                "goes_lower_than_start": min(
                    report["minimum_height_above_assumed_table_mm"] for report in first_segment
                ) < start_height - 1e-6,
                "first_fully_clear_sample": first_clear,
                "reenters_below_plane_after_fully_clear": reentered,
            },
            "candidate_connection_aggregate": {
                "sample_count": len(candidate_reports),
                "self_collision_sample_count": sum(bool(report["self_collision_pairs"])
                                                   for report in candidate_reports),
                "self_collision_sample_count_by_phase": _phase_counts(candidate_reports, "self_collision_pairs"),
                "cup_aabb_overlap_sample_count": sum(bool(report["cup_aabb_overlap_links"])
                                                     for report in candidate_reports),
                "cup_aabb_overlap_sample_count_by_phase": _phase_counts(candidate_reports, "cup_aabb_overlap_links"),
            },
            "failed_samples": [report for report in reports if report["self_collision_pairs"]
                               or report["cup_aabb_overlap_links"]
                               or report["minimum_height_above_assumed_table_mm"] < 0.0],
        })

    maximum_observed_step = max(
        max(abs(a - b) for a, b in zip(previous["raw_ticks"], current["raw_ticks"]))
        for previous, current in zip(samples, samples[1:])
    )
    nominal = reports_by_assumption[0]
    all_sampled_clear = all(not report["failed_samples"] for report in reports_by_assumption)
    return {
        "schema": SCHEMA,
        "mode": "offline_sampled_path_review",
        "hardware_accessed": False,
        "motion_command_emitted": False,
        "process_or_camera_accessed": False,
        "sampled_only": True,
        "rendering_performed": False,
        "executable": False,
        "physical_execution_ready": False,
        "not_executable_reasons": [
            "오른팔 상태와 양팔 간 충돌을 검토하지 않음",
            "책상 extrinsic과 left_base_link 높이 대응이 미측정",
            "그리퍼 raw tick과 URDF 개구각 대응이 미보정",
            "이산 raw 표본 사이의 연속 충돌을 증명하지 않음",
        ],
        "bindings": {
            "snapshot_sha256": digest(snapshot),
            "candidate_snapshot_sha256": candidates["snapshot_sha256"],
            "calibration_sha256": _sha(calibration_bytes),
            "candidate_calibration_sha256": candidates["calibration_sha256"],
            "urdf_sha256": _sha(URDF_PATH.read_bytes()),
            "candidate_urdf_sha256": candidates["urdf_sha256"],
            "joint_reference_sha256": digest(reference),
        },
        "method": {
            "scene_unit": "mm",
            "path_space": "raw_ticks_after_joint_reference_quantization",
            "clearance_profile": clearance_profile,
            "clearance_relative_joint_delta_deg": CLEARANCE_PROFILES_DEG[clearance_profile],
            "maximum_raw_step_ticks": MAX_RAW_STEP_TICKS,
            "maximum_observed_raw_step_ticks": maximum_observed_step,
            "start_sample_included": True,
            "joint_limits": "intersection of URDF limits and joint-reference raw limits",
            "minimum_observed_joint_limit_margin_deg": round(minimum_limit_margin_deg, 3),
            "collision_method": "cached local URDF collision meshes transformed per sample; VTK triangle collision for nonadjacent self pairs",
            "cup_method": "conservative link-AABB overlap against cup bounding AABB",
            "gripper_assumptions_rad": list(GRIPPER_ASSUMPTIONS_RAD),
        },
        "assumed_table_plane": {
            "frame": "URDF scene",
            "z_mm": round(table_z_mm, 3),
            "source": "left_base_link origin height",
            "extrinsic_measured": False,
        },
        "cup_aabb_mm": {
            "bounds_xyz": [round(value, 3) for value in cup_aabb],
            "nominal_radius_mm": CUP_RADIUS_MM,
            "nominal_height_mm": CUP_HEIGHT_MM,
            "margin_each_side_mm": CUP_AABB_MARGIN_MM,
        },
        "waypoints": [
            {"phase": phase, "raw_ticks": raw,
             "joint_deg": [round(value, 6) for value in raw_to_joint_deg(raw, calibration_bytes, reference)]}
            for phase, raw in waypoints
        ],
        "sample_count": len(samples),
        "nominal_start_floor_report": nominal["start_floor_report"],
        "nominal_first_segment_floor_behavior": nominal["first_segment_floor_behavior"],
        "gripper_assumption_reports": reports_by_assumption,
        "sampled_path_clear_under_all_gripper_assumptions": all_sampled_clear,
        "offline_review_complete": True,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--clearance-profile", choices=sorted(CLEARANCE_PROFILES_DEG),
                        default="retract_and_lift")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = build(
        json.loads(args.snapshot.read_text(encoding="utf-8")),
        json.loads(args.candidates.read_text(encoding="utf-8")),
        args.calibration.read_bytes(),
        args.clearance_profile,
    )
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(text)
    print(args.output)


if __name__ == "__main__":
    main()
