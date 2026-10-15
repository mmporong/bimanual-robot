#!/usr/bin/env python3
"""Xacro 확장, URDF 구조, 메시 경로와 핵심 기하 파라미터를 검증한다."""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[4] / 'src/description/hold_flow_description'
REPO_ROOT = PACKAGE_ROOT.parents[2]
XACRO = PACKAGE_ROOT / "urdf/hold_flow.urdf.xacro"
COMMITTED_URDF = PACKAGE_ROOT / "urdf/hold_flow.urdf"
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
CALIBRATION_PATH = PACKAGE_ROOT / "config/mechanical_calibration.yaml"


def command_output(command: list[str], env: dict[str, str]) -> str:
    return subprocess.run(command, check=True, text=True, capture_output=True, env=env).stdout


def rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def multiply_transform(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][index] * b[index][column] for index in range(4)) for column in range(4)]
        for row in range(4)
    ]


def joint_transform(joint: ET.Element) -> list[list[float]]:
    origin = joint.find("origin")
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]
    rotation = rpy_matrix(*rpy)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def link_transforms(root: ET.Element) -> dict[str, list[list[float]]]:
    children: dict[str, list[tuple[str, list[list[float]]]]] = {}
    child_links: set[str] = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        children.setdefault(parent, []).append((child, joint_transform(joint)))
        child_links.add(child)
    root_links = {link.attrib["name"] for link in root.findall("link")} - child_links
    assert root_links == {"base_footprint"}, sorted(root_links)
    identity = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    transforms = {"base_footprint": identity}
    stack = ["base_footprint"]
    while stack:
        parent = stack.pop()
        for child, local in children.get(parent, []):
            transforms[child] = multiply_transform(transforms[parent], local)
            stack.append(child)
    return transforms


def xyz_of(transform: list[list[float]]) -> list[float]:
    return [transform[index][3] for index in range(3)]


def assert_xyz(actual: list[float], expected: list[float], tolerance: float = 1e-6) -> None:
    assert all(math.isclose(a, e, abs_tol=tolerance) for a, e in zip(actual, expected)), (
        actual,
        expected,
    )


def inertia_values(link: ET.Element) -> dict[str, float]:
    inertia = link.find("inertial/inertia")
    assert inertia is not None, f"{link.attrib['name']}: inertia 누락"
    return {key: float(inertia.attrib[key]) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}


def assert_drive_limit_contract(
    root: ET.Element,
    wheel_spec: dict,
    drive_config: dict,
    joint_names: list[str],
) -> None:
    """기구 사양, 생성 설정, 네 바퀴 URDF limit의 일치를 검증한다."""
    expected_effort = float(wheel_spec["joint_effort"])
    expected_velocity = float(wheel_spec["joint_velocity"])
    assert math.isclose(
        float(drive_config["joint_effort_nm"]), expected_effort, abs_tol=1e-12
    ), (drive_config["joint_effort_nm"], expected_effort)
    assert math.isclose(
        float(drive_config["joint_velocity_radps"]), expected_velocity, abs_tol=1e-12
    ), (drive_config["joint_velocity_radps"], expected_velocity)
    for joint_name in joint_names:
        joint = root.find(f"joint[@name='{joint_name}']")
        assert joint is not None, f"{joint_name}: 관절 누락"
        limit = joint.find("limit")
        assert limit is not None, f"{joint_name}: limit 누락"
        actual_effort = float(limit.attrib["effort"])
        actual_velocity = float(limit.attrib["velocity"])
        assert math.isclose(actual_effort, expected_effort, abs_tol=1e-12), (
            joint_name, "effort", actual_effort, expected_effort,
        )
        assert math.isclose(actual_velocity, expected_velocity, abs_tol=1e-12), (
            joint_name, "velocity", actual_velocity, expected_velocity,
        )


def main() -> None:
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    calibration = yaml.safe_load(CALIBRATION_PATH.read_text(encoding="utf-8"))
    wheel_spec = spec["navigation"]["wheel"]
    drive_config = calibration["drive"]
    env = os.environ.copy()
    existing = env.get("AMENT_PREFIX_PATH", "")
    ros_python = Path("/opt/ros/jazzy/lib/python3.12/site-packages")
    if ros_python.is_dir():
        existing_python = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = str(ros_python) + (":" + existing_python if existing_python else "")
    with tempfile.TemporaryDirectory(prefix="hold_flow_urdf_") as temp_dir:
        temp_root = Path(temp_dir)
        package_prefix = temp_root / "prefix"
        package_index = package_prefix / "share/ament_index/resource_index/packages"
        package_index.mkdir(parents=True)
        (package_index / "hold_flow_description").write_text("", encoding="utf-8")
        package_share = package_prefix / "share/hold_flow_description"
        package_share.symlink_to(PACKAGE_ROOT, target_is_directory=True)
        env["AMENT_PREFIX_PATH"] = str(package_prefix) + (":" + existing if existing else "")
        urdf_path = temp_root / "hold_flow.urdf"
        expanded_urdf = command_output(["xacro", str(XACRO.relative_to(REPO_ROOT))], env)
        urdf_path.write_text(expanded_urdf, encoding="utf-8")
        check = command_output(["check_urdf", str(urdf_path)], env)
        root = ET.parse(urdf_path).getroot()
    assert COMMITTED_URDF.read_text(encoding="utf-8") == expanded_urdf

    links = root.findall("link")
    joints = root.findall("joint")
    link_names = {link.attrib["name"] for link in links}
    joint_names = {joint.attrib["name"] for joint in joints}
    mesh_files = []
    cad_copy_pairs: list[tuple[Path, Path]] = []
    for mesh in root.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://hold_flow_description/"
        if not uri.startswith(prefix):
            raise AssertionError(f"외부 또는 비표준 메시 URI: {uri}")
        local = PACKAGE_ROOT / uri.removeprefix(prefix)
        if not local.is_file():
            raise AssertionError(f"메시 누락: {local}")
        mesh_files.append(str(local.relative_to(REPO_ROOT)))
        if local.parent.name == "cad":
            source = REPO_ROOT / "design/cad/exports/stl" / local.name
            if not source.is_file():
                raise AssertionError(f"CAD 원본 메시 누락: {source}")
            cad_copy_pairs.append((source, local))

    for source, local in set(cad_copy_pairs):
        source_hash = hashlib.sha256(source.read_bytes()).digest()
        local_hash = hashlib.sha256(local.read_bytes()).digest()
        if source_hash != local_hash:
            raise AssertionError(f"CAD 원본과 ROS 복사본 불일치: {local.name}")

    required_links = {
        "base_footprint",
        "base_link",
        "battery_link",
        "electronics_link",
        "left_base_link",
        "right_base_link",
        "right_gripper_base_link",
        "left_moving_jaw_link",
        "right_finger1_link",
        "right_finger2_link",
        "left_tool0",
        "right_tool0",
        "camera_backing_link",
        "camera_cradle_link",
        "camera_depth_optical_frame",
        "camera_color_optical_frame",
        "laser_link",
        "left_wheel_link",
        "right_wheel_link",
        "rear_left_wheel_link",
        "rear_right_wheel_link",
        "front_left_frame_column_link",
        "front_right_frame_column_link",
        "rear_left_frame_column_link",
        "rear_right_frame_column_link",
    }
    required_joints = {
        "left_wheel_joint",
        "right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
        "left_shoulder_pan",
        "left_shoulder_lift",
        "left_elbow_flex",
        "left_wrist_flex",
        "left_wrist_roll",
        "right_shoulder_pan",
        "right_shoulder_lift",
        "right_elbow_flex",
        "right_wrist_flex",
        "right_wrist_roll",
        "left_gripper",
        "right_finger1_joint",
        "right_finger2_joint",
        "left_tool0_joint",
        "right_tool0_joint",
    }
    assert required_links <= link_names, sorted(required_links - link_names)
    assert required_joints <= joint_names, sorted(required_joints - joint_names)
    assert not {name for name in link_names | joint_names if "caster" in name}, (
        "폐기된 캐스터 링크/관절이 남아 있음"
    )

    for link_name in ("right_finger1_link", "right_finger2_link"):
        link = root.find(f"link[@name='{link_name}']")
        assert link is not None
        material_names = {
            material.attrib.get("name")
            for material in link.findall("visual/material")
        }
        assert "gripper_insert" not in material_names, (
            f"{link_name}에 폐기된 ggao50 인서트 형상이 남아 있음"
        )

    transforms = link_transforms(root)
    expected_joint_names = {
        "front_left": "left_wheel_joint",
        "front_right": "right_wheel_joint",
        "rear_left": "rear_left_wheel_joint",
        "rear_right": "rear_right_wheel_joint",
    }
    assert spec["navigation"]["joint_names_by_corner"] == expected_joint_names
    assert spec["navigation"]["servo_ids_by_corner"] == {
        "front_left": 2, "front_right": 1, "rear_left": 3, "rear_right": 4,
    }
    assert drive_config["servo_ids_by_corner"] == spec["navigation"]["servo_ids_by_corner"]
    assert math.isclose(
        drive_config["geometric_wheel_center_separation_m"] * 1000.0,
        wheel_spec["geometric_center_separation"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        drive_config["wheelbase_design_m"] * 1000.0,
        wheel_spec["wheelbase_design"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        drive_config["visual_width_m"] * 1000.0,
        wheel_spec["visual_width"],
        abs_tol=1e-9,
    )
    assert_drive_limit_contract(
        root, wheel_spec, drive_config, list(expected_joint_names.values())
    )
    wheel_contract = {
        "left_wheel_link": wheel_spec["center_xyz_left"],
        "right_wheel_link": wheel_spec["center_xyz_right"],
        "rear_left_wheel_link": wheel_spec["center_xyz_rear_left"],
        "rear_right_wheel_link": wheel_spec["center_xyz_rear_right"],
    }
    for link_name, expected_mm in wheel_contract.items():
        assert_xyz(xyz_of(transforms[link_name]), [value / 1000.0 for value in expected_mm])
    separation = abs(
        transforms["left_wheel_link"][1][3] - transforms["right_wheel_link"][1][3]
    )
    wheelbase = abs(
        transforms["left_wheel_link"][0][3] - transforms["rear_left_wheel_link"][0][3]
    )
    assert math.isclose(separation * 1000.0, wheel_spec["geometric_center_separation"], abs_tol=1e-6)
    assert math.isclose(wheelbase * 1000.0, wheel_spec["wheelbase_design"], abs_tol=1e-6)
    expected_mass = float(wheel_spec["mass_each"])
    expected_radius = float(wheel_spec["calibrated_effective_radius"]) / 1000.0
    expected_width = float(wheel_spec["visual_width"]) / 1000.0
    expected_inertia = {
        "ixx": expected_mass * (3.0 * expected_radius**2 + expected_width**2) / 12.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyy": expected_mass * expected_radius**2 / 2.0,
        "iyz": 0.0,
        "izz": expected_mass * (3.0 * expected_radius**2 + expected_width**2) / 12.0,
    }
    for link_name in wheel_contract:
        link = root.find(f"link[@name='{link_name}']")
        assert link is not None
        mass = float(link.find("inertial/mass").attrib["value"])
        assert math.isclose(mass, expected_mass, abs_tol=1e-9), (link_name, mass, expected_mass)
        actual_inertia = inertia_values(link)
        assert all(
            math.isclose(actual_inertia[key], value, rel_tol=1e-6, abs_tol=1e-12)
            for key, value in expected_inertia.items()
        ), (link_name, actual_inertia, expected_inertia)
        cylinder = link.find("collision/geometry/cylinder")
        assert cylinder is not None
        assert math.isclose(float(cylinder.attrib["radius"]), expected_radius, abs_tol=1e-9)
        assert math.isclose(float(cylinder.attrib["length"]), expected_width, abs_tol=1e-9)
    for joint_name in expected_joint_names.values():
        joint = root.find(f"joint[@name='{joint_name}']")
        assert joint is not None and joint.attrib["type"] == "continuous"
        assert joint.find("axis").attrib["xyz"] == "0 1 0"
    assert_xyz(xyz_of(transforms["base_link"]), [0.0, 0.0, expected_radius])
    camera_xyz = xyz_of(transforms["camera_depth_optical_frame"])
    lidar_xyz = xyz_of(transforms["laser_link"])
    left_arm_xyz = xyz_of(transforms["left_base_link"])
    right_arm_xyz = xyz_of(transforms["right_base_link"])
    assert_xyz(camera_xyz, [-0.110, 0.0, 1.260], tolerance=2e-6)
    assert_xyz(lidar_xyz, [0.100, 0.0, 0.165])
    assert_xyz(left_arm_xyz, [0.020, 0.170, 0.726])
    assert_xyz(right_arm_xyz, [0.020, -0.170, 0.726])

    report = {
        "xacro": str(XACRO.relative_to(REPO_ROOT)),
        "check_urdf_root": "robot name is: hold_flow" in check,
        "links": len(links),
        "joints": len(joints),
        "mesh_references": len(mesh_files),
        "unique_mesh_files": len(set(mesh_files)),
        "cad_mesh_copies_match_exports": True,
        "committed_urdf_matches_xacro": True,
        "wheel_center_separation_m": separation,
        "wheelbase_design_m": wheelbase,
        "wheel_velocity_joints": [
            "left_wheel_joint", "right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint",
        ],
        "wheel_mass_each_kg": expected_mass,
        "camera_optical_xyz_from_base_footprint_m": [round(value, 6) for value in camera_xyz],
        "lidar_xyz_from_base_footprint_m": [round(value, 6) for value in lidar_xyz],
        "left_arm_base_xyz_from_base_footprint_m": [round(value, 6) for value in left_arm_xyz],
        "right_arm_base_xyz_from_base_footprint_m": [round(value, 6) for value in right_arm_xyz],
        "left_gripper": "stock_so101_rotating_jaw",
        "right_gripper": "ggao50_parallel_proxy_unmodified_flat_jaws",
        "right_gripper_insert": "none",
        "right_parallel_gripper_primary_stroke_m": 0.0333,
        "mimic_joints": [
            joint.attrib["name"] for joint in joints if joint.find("mimic") is not None
        ],
    }
    assert report["check_urdf_root"]
    assert report["mimic_joints"] == ["right_finger2_joint"]
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
