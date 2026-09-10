#!/usr/bin/env python3
"""Isaac Sim 가져오기 전에 필요한 URDF 기하·관절·접촉 계약을 정적 검증한다."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
URDF_PATH = PACKAGE_ROOT / "urdf/hold_flow.urdf"
sys.path.insert(0, str(Path(__file__).parent))
from validate_description import link_transforms, xyz_of  # noqa: E402


def close_xyz(actual: list[float], expected_mm: list[float], tolerance_m: float = 2e-6) -> bool:
    expected = [value / 1000.0 for value in expected_mm]
    return all(math.isclose(a, e, abs_tol=tolerance_m) for a, e in zip(actual, expected))


def main() -> None:
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    root = ET.parse(URDF_PATH).getroot()
    transforms = link_transforms(root)
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    issues: list[str] = []

    tabletop = spec["chassis"]["plates"]["tabletop"]
    expected_xyz = {
        "tabletop_front_left_link": [60.15, 112.65, tabletop["z_bottom"]],
        "tabletop_front_right_link": [60.15, -112.65, tabletop["z_bottom"]],
        "tabletop_rear_left_link": [-110.15, 112.65, tabletop["z_bottom"]],
        "tabletop_rear_right_link": [-110.15, -112.65, tabletop["z_bottom"]],
        "left_base_link": spec["arm_mounts"]["left"]["xyz"],
        "right_base_link": spec["arm_mounts"]["right"]["xyz"],
        "camera_depth_optical_frame": spec["camera"]["optical_center_xyz"],
        "camera_mast_link": [
            *spec["camera"]["mount"]["mast_center_xy"],
            spec["camera"]["mount"]["mast_z_bottom"],
        ],
        "laser_link": spec["navigation"]["lidar_candidate"]["center_xyz"],
        "left_wheel_link": spec["navigation"]["wheel"]["center_xyz_left"],
        "right_wheel_link": spec["navigation"]["wheel"]["center_xyz_right"],
        "front_caster_link": spec["navigation"]["ball_casters"]["centers_xyz"][0],
        "rear_caster_link": spec["navigation"]["ball_casters"]["centers_xyz"][1],
        "battery_link": spec["internal_payloads"]["battery"]["center_xyz"],
        "electronics_link": spec["internal_payloads"]["electronics"]["center_xyz"],
        "battery_mount_plate_link": spec["printed_mounts"]["battery_plate"]["origin_xyz"],
        "lidar_mount_plate_link": spec["printed_mounts"]["lidar_plate"]["origin_xyz"],
    }
    column_names = (
        "front_left_frame_column_link",
        "front_right_frame_column_link",
        "rear_left_frame_column_link",
        "rear_right_frame_column_link",
    )
    column_z = spec["chassis"]["vertical_frame"]["z_bottom"]
    for name, (x, y) in zip(column_names, spec["chassis"]["vertical_frame"]["centers_xy"]):
        expected_xyz[name] = [x, y, column_z]
    for link_name, expected in expected_xyz.items():
        if link_name not in transforms:
            issues.append(f"{link_name}: TF 트리에 없음")
            continue
        actual = xyz_of(transforms[link_name])
        if not close_xyz(actual, expected):
            issues.append(f"{link_name}: xyz={actual}, expected_mm={expected}")

    expected_joint_types = {
        "left_wheel_joint": "continuous",
        "right_wheel_joint": "continuous",
        "left_gripper": "revolute",
        "right_finger1_joint": "prismatic",
        "right_finger2_joint": "prismatic",
    }
    for name, expected_type in expected_joint_types.items():
        joint = joints.get(name)
        if joint is None:
            issues.append(f"{name}: 관절 누락")
        elif joint.attrib["type"] != expected_type:
            issues.append(f"{name}: type={joint.attrib['type']}, expected={expected_type}")

    mimic = [joint.attrib["name"] for joint in joints.values() if joint.find("mimic") is not None]
    if mimic != ["right_finger2_joint"]:
        issues.append(f"mimic 관절 집합 불일치: {mimic}")

    right_proxy_links = ("right_gripper_base_link", "right_finger1_link", "right_finger2_link")
    right_proxy_mass = sum(
        float(links[name].find("inertial/mass").attrib["value"])
        for name in right_proxy_links
    )
    expected_proxy_mass = float(spec["right_parallel_gripper"]["urdf_proxy_mass"])
    if not math.isclose(right_proxy_mass, expected_proxy_mass, abs_tol=1e-9):
        issues.append(
            f"오른쪽 그리퍼 프록시 질량={right_proxy_mass}, expected={expected_proxy_mass}"
        )

    axis_joint_types = {"continuous", "revolute", "prismatic"}
    for joint in joints.values():
        if joint.attrib["type"] not in axis_joint_types:
            continue
        for child in ("axis", "limit", "dynamics"):
            if joint.find(child) is None:
                issues.append(f"{joint.attrib['name']}: {child} 누락")
        if joint.attrib["type"] != "continuous" and joint.find("safety_controller") is None:
            issues.append(f"{joint.attrib['name']}: safety_controller 누락")

    frame_columns = [name for name in links if name.endswith("frame_column_link")]
    if len(frame_columns) != 4:
        issues.append(f"프레임 기둥 수={len(frame_columns)}, expected=4")
    casters = [name for name in links if name in {"front_caster_link", "rear_caster_link"}]
    if len(casters) != 2:
        issues.append(f"볼 캐스터 수={len(casters)}, expected=2")
    tabletop_panels = [name for name in links if name.startswith("tabletop_") and name.endswith("_link")]
    if len(tabletop_panels) != 4:
        issues.append(f"상판 분할 수={len(tabletop_panels)}, expected=4")

    cad_collision_meshes = []
    for link in links.values():
        visual_mesh = link.find("visual/geometry/mesh")
        if visual_mesh is None or "/meshes/cad/" not in visual_mesh.attrib["filename"]:
            continue
        if link.find("collision/geometry/mesh") is not None:
            cad_collision_meshes.append(link.attrib["name"])
    if cad_collision_meshes:
        issues.append(f"커스텀 구조에 triangle collision 사용: {cad_collision_meshes}")

    report = {
        "urdf": str(URDF_PATH.relative_to(REPO_ROOT)),
        "source_spec": str(SPEC_PATH.relative_to(REPO_ROOT)),
        "target": spec["simulation"]["target"],
        "coordinate_checks": len(expected_xyz),
        "articulated_joints": len(
            [joint for joint in joints.values() if joint.attrib["type"] in axis_joint_types]
        ),
        "mimic_joints": mimic,
        "right_gripper_proxy_mass_kg": right_proxy_mass,
        "frame_columns": frame_columns,
        "tabletop_panels": tabletop_panels,
        "ball_casters": casters,
        "custom_structure_uses_primitive_collision": not cad_collision_meshes,
        "issues": issues,
        "static_import_contract_ready": not issues,
        "runtime_note": "실제 USD 생성·접촉·gain 안정성은 Isaac Sim 6.0 장비에서 별도 검증",
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if issues:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
