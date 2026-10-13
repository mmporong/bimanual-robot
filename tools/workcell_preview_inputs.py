#!/usr/bin/env python3
"""Prepare explicit, offline-only inputs for the URDF workcell preview.

This module reads only paths supplied by the caller.  It does not import ROS,
Isaac Sim, camera, or serial packages and does not issue hardware commands.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any
import xml.etree.ElementTree as ET


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
NOMINAL_PREVIEW_DEG = [0.0, -68.0, 92.0, -22.0, 0.0]
PACKAGE_URI = "package://hold_flow_description/"
MESH_PATTERN = re.compile(r'(<mesh\b[^>]*\bfilename=["\'])' + re.escape(PACKAGE_URI) + r'([^"\']+)(["\'])')


def _number_vector(value: object, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 5:
        raise ValueError(f"{label}: 5개 관절값이 필요합니다")
    if any(type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise ValueError(f"{label}: bool을 제외한 유한 숫자만 허용합니다")
    return [float(item) for item in value]


def _read_json(path: Path) -> tuple[dict[str, Any], dict[str, str]]:
    resolved = path.expanduser().resolve(strict=True)
    if not resolved.is_file():
        raise ValueError(f"일반 파일만 허용합니다: {resolved}")
    raw = resolved.read_bytes()
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON 최상위 값은 객체여야 합니다: {resolved}")
    return value, {"path": str(resolved), "sha256": hashlib.sha256(raw).hexdigest()}


def load_replay(plan_path: Path | None, state_paths: list[Path]) -> dict[str, Any]:
    """Load explicitly selected plan/state files for a visual-only preview."""
    poses: list[dict[str, Any]] = []
    source_files: list[dict[str, str]] = []
    limitations = [
        "관절 자세는 시각화 입력이며 충돌 안전성이나 동역학 타당성을 보장하지 않는다",
        "저장 상태와 계획 자세의 시간 동기화는 이 입력만으로 확인할 수 없다",
    ]

    if plan_path is not None:
        plan, provenance = _read_json(plan_path)
        source_files.append(provenance)
        stages = plan.get("stages")
        if not isinstance(stages, dict):
            raise ValueError("계획 JSON에 stages 객체가 필요합니다")
        for stage_name in ("pregrasp", "grasp"):
            stage = stages.get(stage_name)
            if not isinstance(stage, dict):
                raise ValueError(f"계획 JSON에 {stage_name} 단계가 필요합니다")
            poses.append({
                "name": f"planned_{stage_name}",
                "left_joint_deg": _number_vector(stage.get("joint_deg"), f"계획 {stage_name}"),
                "source_kind": "plan",
                "source_path": provenance["path"],
            })

    for state_path in state_paths:
        state, provenance = _read_json(state_path)
        source_files.append(provenance)
        if state.get("side") != "left":
            raise ValueError(f"왼팔 저장 상태만 허용합니다: {provenance['path']}")
        if state.get("joint_order") != ARM_JOINTS:
            raise ValueError(f"저장 상태의 joint_order가 ARM_JOINTS와 다릅니다: {provenance['path']}")
        poses.append({
            "name": Path(provenance["path"]).stem,
            "left_joint_deg": _number_vector(state.get("joint_degrees"), "저장 자세"),
            "source_kind": "saved_state",
            "source_path": provenance["path"],
        })

    if not poses:
        poses.append({
            "name": "nominal_preview_assumed",
            "left_joint_deg": NOMINAL_PREVIEW_DEG.copy(),
            "source_kind": "ASSUMED",
            "source_path": None,
        })
        limitations.append("입력 파일이 없어 명목상 ASSUMED 자세를 사용한다")

    return {"poses": poses, "source_files": source_files, "limitations": limitations}


def _package_root(source: Path) -> Path:
    for candidate in (source.parent, *source.parents):
        if candidate.name == "hold_flow_description" and (candidate / "package.xml").is_file():
            return candidate.resolve()
    raise ValueError(f"hold_flow_description 패키지 안의 URDF가 아닙니다: {source}")


def _strict_number(value: object, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label}: bool을 제외한 유한 숫자가 필요합니다")
    return float(value)


def _named(root: ET.Element, tag: str, name: str) -> ET.Element:
    matches = root.findall(f"./{tag}[@name='{name}']")
    if len(matches) != 1:
        raise ValueError(f"URDF에 {tag} {name!r}가 정확히 하나 있어야 합니다")
    return matches[0]


def _origin_xyz(joint: ET.Element) -> list[float]:
    origin = joint.find("origin")
    if origin is None:
        raise ValueError(f"joint {joint.get('name')}에 origin이 없습니다")
    values = origin.get("xyz", "").split()
    if len(values) != 3:
        raise ValueError(f"joint {joint.get('name')}의 xyz가 올바르지 않습니다")
    try:
        result = [float(value) for value in values]
    except ValueError as exc:
        raise ValueError(f"joint {joint.get('name')}의 xyz가 숫자가 아닙니다") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"joint {joint.get('name')}의 xyz가 유한하지 않습니다")
    return result


def _set_origin_xyz(joint: ET.Element, xyz: tuple[float, float, float]) -> None:
    origin = joint.find("origin")
    if origin is None:
        raise ValueError(f"joint {joint.get('name')}에 origin이 없습니다")
    origin.set("xyz", " ".join(f"{value:.6f}" for value in xyz))


def _wheel_cylinders(link: ET.Element) -> list[ET.Element]:
    cylinders = link.findall("./visual/geometry/cylinder") + link.findall("./collision/geometry/cylinder")
    if len(cylinders) != 2:
        raise ValueError(f"link {link.get('name')}에 visual/collision cylinder가 필요합니다")
    return cylinders


def _strict_vector3(value: object, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label}: 유한 숫자 3개가 필요합니다")
    return [_strict_number(item, label) for item in value]


def _apply_base_overlay(root: ET.Element, contract: dict[str, Any], provenance: dict[str, str]) -> dict[str, Any]:
    target = contract.get("target_design")
    hardware = contract.get("hardware")
    measured = hardware.get("measured_base_footprint") if isinstance(hardware, dict) else None
    if not isinstance(target, dict) or not isinstance(measured, dict):
        raise ValueError("base contract에 target_design과 hardware.measured_base_footprint가 필요합니다")

    if target.get("drive_type") != "skid_steer_four_wheel_drive":
        raise ValueError("현행 프리뷰에는 skid_steer_four_wheel_drive target_design이 필요합니다")
    if target.get("model_base_frame_definition") != "chassis_center_on_floor":
        raise ValueError("model_base_frame_definition은 chassis_center_on_floor여야 합니다")
    if target.get("runtime_base_frame_definition") != "front_axle_projection_on_floor":
        raise ValueError("runtime_base_frame_definition은 front_axle_projection_on_floor여야 합니다")

    model_from_runtime_m = _strict_vector3(
        target.get("model_from_runtime_translation_m"), "model_from_runtime_translation_m",
    )
    wheelbase_m = _strict_number(target.get("wheelbase_design_m"), "wheelbase_design_m")
    wheel_radius_m = _strict_number(target.get("wheel_radius_initial_m"), "wheel_radius_initial_m")
    separation_m = _strict_number(target.get("wheel_separation_geometric_m"), "wheel_separation_geometric_m")
    wheel_outer_width_m = _strict_number(measured.get("wheel_outer_width_m"), "wheel_outer_width_m")
    if wheel_radius_m <= 0 or wheelbase_m <= 0 or separation_m <= 0 or wheel_outer_width_m <= separation_m:
        raise ValueError("바퀴 반지름·휠베이스·간격·외측 폭의 관계가 올바르지 않습니다")
    if target.get("wheelbase_measured_m") is not None:
        raise ValueError("별도 축간 계측 전에는 wheelbase_measured_m가 null이어야 합니다")
    if target.get("rear_axle_geometry_status") not in {
        "design_assumption_not_measured",
        "user_confirmed_symmetry_derived_not_tape_measured",
    }:
        raise ValueError("rear_axle_geometry_status가 설계 또는 사용자 확인 환산 근거를 보존해야 합니다")

    wheel_half_y_m = separation_m / 2.0
    wheel_length_m = wheel_outer_width_m - separation_m
    base_origin = _origin_xyz(_named(root, "joint", "base_footprint_joint"))
    base_link_from_floor_m = base_origin[2]

    wheel_specs = (
        ("left_wheel_joint", "left_wheel_link", wheel_half_y_m, "front_left"),
        ("right_wheel_joint", "right_wheel_link", -wheel_half_y_m, "front_right"),
        ("rear_left_wheel_joint", "rear_left_wheel_link", wheel_half_y_m, "rear_left"),
        ("rear_right_wheel_joint", "rear_right_wheel_link", -wheel_half_y_m, "rear_right"),
    )
    expected_joint_names = {spec[0] for spec in wheel_specs}
    expected_link_names = {spec[1] for spec in wheel_specs}
    actual_joint_names = {
        joint.get("name") for joint in root.findall("./joint")
        if (joint.get("name") or "").endswith("wheel_joint")
    }
    actual_link_names = {
        link.get("name") for link in root.findall("./link")
        if (link.get("name") or "").endswith("wheel_link")
    }
    if actual_joint_names != expected_joint_names or actual_link_names != expected_link_names:
        raise ValueError("4WD URDF에는 정해진 바퀴 joint/link가 각각 4개 있어야 합니다")
    if any("caster" in (element.get("name") or "").lower()
           for tag in ("joint", "link") for element in root.findall(f"./{tag}")):
        raise ValueError("4WD URDF에는 caster joint/link가 없어야 합니다")

    source_x_by_axle: dict[str, float] = {}
    wheel_changes: dict[str, Any] = {}
    for joint_name, link_name, y_m, position_label in wheel_specs:
        joint = _named(root, "joint", joint_name)
        before_xyz = _origin_xyz(joint)
        axle = "rear" if joint_name.startswith("rear_") else "front"
        prior_x = source_x_by_axle.setdefault(axle, before_xyz[0])
        if not math.isclose(before_xyz[0], prior_x, abs_tol=1e-12):
            raise ValueError(f"{axle} 좌우 바퀴의 source x가 다릅니다")
        parent = joint.find("parent")
        parent_link = parent.get("link") if parent is not None else None
        if parent_link == "base_link":
            wheel_z_m = wheel_radius_m - base_link_from_floor_m
        elif parent_link == "base_footprint":
            wheel_z_m = wheel_radius_m
        else:
            raise ValueError(f"{joint_name}의 parent는 base_link 또는 base_footprint여야 합니다")
        after_xyz = (before_xyz[0], y_m, wheel_z_m)
        _set_origin_xyz(joint, after_xyz)
        link = _named(root, "link", link_name)
        for cylinder in _wheel_cylinders(link):
            cylinder.set("radius", f"{wheel_radius_m:.6f}")
            cylinder.set("length", f"{wheel_length_m:.6f}")
        wheel_changes[joint_name] = {
            "position": position_label,
            "link_name": link_name,
            "parent_link": parent_link,
            "joint_xyz_before_m": before_xyz,
            "joint_xyz_after_m": list(after_xyz),
            "source_design_x_preserved": True,
            "radius_m": wheel_radius_m,
            "length_m": wheel_length_m,
        }

    if not math.isclose(source_x_by_axle["front"], model_from_runtime_m[0], abs_tol=1e-9):
        raise ValueError("front axle source x가 model_from_runtime_translation_m와 다릅니다")
    if not math.isclose(
        source_x_by_axle["front"] - source_x_by_axle["rear"], wheelbase_m, abs_tol=1e-9,
    ):
        raise ValueError("source axle x 간격이 wheelbase_design_m와 다릅니다")

    return {
        "applied": True,
        "contract_path": provenance["path"],
        "contract_sha256": provenance["sha256"],
        "scene_frame_basis": {
            "base_footprint": target["model_base_frame_definition"],
            "runtime_base_frame": target["runtime_base_frame_definition"],
            "model_from_runtime_translation_m": model_from_runtime_m,
        },
        "model_geometry": {
            "drive_type": target["drive_type"],
            "wheel_count": len(wheel_specs),
            "caster_count": 0,
            "wheel_joint_names": [spec[0] for spec in wheel_specs],
            "axle_x_basis": "source_design_preserved",
            "rear_axle_geometry_status": target["rear_axle_geometry_status"],
        },
        "changes": {
            "wheelbase_design_m": wheelbase_m,
            "wheel_separation_m": separation_m,
            "wheel_outer_width_m": wheel_outer_width_m,
            "wheels": wheel_changes,
            "casters": {},
        },
        "untouched_arm_frames": True,
        "untouched_non_base_joint_origins": True,
        "inertias_updated": False,
        "dynamics_validated": False,
        "contact_stability_validated": False,
    }


def materialize_urdf(source: Path, output: Path,
                     base_contract: Path | None = None) -> dict[str, Any]:
    """Write an Isaac-readable URDF copy while preserving the active source."""
    source_resolved = source.expanduser().resolve(strict=True)
    if not source_resolved.is_file():
        raise ValueError(f"URDF 입력이 일반 파일이 아닙니다: {source_resolved}")
    output_resolved = output.expanduser().resolve()
    package_root = _package_root(source_resolved)
    source_raw = source_resolved.read_bytes()
    text = source_raw.decode("utf-8")
    base_overlay: dict[str, Any] = {"applied": False}
    if base_contract is not None:
        contract, contract_provenance = _read_json(base_contract)
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        root = ET.fromstring(text, parser=parser)
        base_overlay = _apply_base_overlay(root, contract, contract_provenance)
        text = ET.tostring(root, encoding="unicode", xml_declaration=False)
    resolved_meshes: list[str] = []

    def replace(match: re.Match[str]) -> str:
        mesh = (package_root / match.group(2)).resolve()
        try:
            mesh.relative_to(package_root)
        except ValueError as exc:
            raise ValueError(f"패키지 밖을 가리키는 메시 경로입니다: {match.group(2)}") from exc
        if not mesh.is_file():
            raise FileNotFoundError(f"URDF 메시 파일이 없습니다: {mesh}")
        resolved_meshes.append(str(mesh))
        return f"{match.group(1)}{mesh.as_posix()}{match.group(3)}"

    materialized = MESH_PATTERN.sub(replace, text)
    if PACKAGE_URI in materialized:
        raise ValueError("지원하지 않는 hold_flow_description package URI가 남아 있습니다")
    output_resolved.parent.mkdir(parents=True, exist_ok=True)
    with output_resolved.open("x", encoding="utf-8") as stream:
        stream.write(materialized)
    return {
        "source_path": str(source_resolved),
        "source_sha256": hashlib.sha256(source_raw).hexdigest(),
        "output_path": str(output_resolved),
        "output_sha256": hashlib.sha256(output_resolved.read_bytes()).hexdigest(),
        "resolved_mesh_files": resolved_meshes,
        "source_preserved": source_resolved.read_bytes() == source_raw,
        "base_overlay": base_overlay,
    }


def sample_pose(poses: list[dict[str, Any]], elapsed_s: float,
                seconds_per_segment: float) -> tuple[str, list[float]]:
    """Loop poses with one common cubic-eased interpolation scalar."""
    if not poses:
        raise ValueError("보간할 자세가 필요합니다")
    if type(elapsed_s) not in (int, float) or not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("elapsed_s는 0 이상의 유한 숫자여야 합니다")
    if (type(seconds_per_segment) not in (int, float)
            or not math.isfinite(seconds_per_segment) or seconds_per_segment <= 0):
        raise ValueError("seconds_per_segment는 양의 유한 숫자여야 합니다")
    vectors = [_number_vector(pose.get("left_joint_deg"), f"자세 {index}")
               for index, pose in enumerate(poses)]
    names = [str(pose.get("name", f"pose_{index}")) for index, pose in enumerate(poses)]
    period = len(poses) * float(seconds_per_segment)
    local = float(elapsed_s) % period
    source_index = min(int(local / float(seconds_per_segment)), len(poses) - 1)
    target_index = (source_index + 1) % len(poses)
    linear = (local - source_index * float(seconds_per_segment)) / float(seconds_per_segment)
    scalar = linear * linear * (3.0 - 2.0 * linear)
    joint_deg = [
        start + (end - start) * scalar
        for start, end in zip(vectors[source_index], vectors[target_index])
    ]
    return f"{names[source_index]}->{names[target_index]}", joint_deg


def main() -> int:
    parser = argparse.ArgumentParser(description="URDF 작업 셀 프리뷰용 오프라인 입력 준비")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--state", type=Path, action="append", default=[])
    parser.add_argument("--urdf-source", type=Path)
    parser.add_argument("--urdf-output", type=Path)
    parser.add_argument("--base-contract", type=Path)
    args = parser.parse_args()
    if (args.urdf_source is None) != (args.urdf_output is None):
        parser.error("--urdf-source와 --urdf-output은 함께 지정해야 합니다")
    if args.base_contract is not None and args.urdf_source is None:
        parser.error("--base-contract는 URDF 입출력과 함께 지정해야 합니다")
    result: dict[str, Any] = {"replay": load_replay(args.plan, args.state)}
    if args.urdf_source is not None:
        result["urdf"] = materialize_urdf(
            args.urdf_source, args.urdf_output, base_contract=args.base_contract,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
