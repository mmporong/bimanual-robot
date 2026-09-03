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


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
XACRO = PACKAGE_ROOT / "urdf/hold_flow.urdf.xacro"
COMMITTED_URDF = PACKAGE_ROOT / "urdf/hold_flow.urdf"


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


def main() -> None:
    env = os.environ.copy()
    existing = env.get("AMENT_PREFIX_PATH", "")
    env["AMENT_PREFIX_PATH"] = str(REPO_ROOT / "install") + (":" + existing if existing else "")
    with tempfile.TemporaryDirectory(prefix="hold_flow_urdf_") as temp_dir:
        urdf_path = Path(temp_dir) / "hold_flow.urdf"
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
        "left_base_link",
        "right_base_link",
        "left_gripper_base_link",
        "right_gripper_base_link",
        "left_finger1_link",
        "left_finger2_link",
        "right_finger1_link",
        "right_finger2_link",
        "left_tool0",
        "right_tool0",
        "camera_backing_link",
        "camera_cradle_link",
        "camera_depth_optical_frame",
        "laser_link",
    }
    required_joints = {
        "left_wheel_joint",
        "right_wheel_joint",
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
        "left_finger1_joint",
        "left_finger2_joint",
        "right_finger1_joint",
        "right_finger2_joint",
        "left_tool0_joint",
        "right_tool0_joint",
    }
    assert required_links <= link_names, sorted(required_links - link_names)
    assert required_joints <= joint_names, sorted(required_joints - joint_names)

    wheel_origins = {}
    for name in ("left_wheel_joint", "right_wheel_joint"):
        joint = root.find(f"joint[@name='{name}']")
        assert joint is not None
        xyz = [float(value) for value in joint.find("origin").attrib["xyz"].split()]
        wheel_origins[name] = xyz
    separation = abs(wheel_origins["left_wheel_joint"][1] - wheel_origins["right_wheel_joint"][1])
    assert math.isclose(separation, 0.270, abs_tol=1e-9)
    transforms = link_transforms(root)
    camera_xyz = xyz_of(transforms["camera_depth_optical_frame"])
    lidar_xyz = xyz_of(transforms["laser_link"])
    left_arm_xyz = xyz_of(transforms["left_base_link"])
    right_arm_xyz = xyz_of(transforms["right_base_link"])
    assert_xyz(camera_xyz, [-0.170, 0.0, 0.800], tolerance=2e-6)
    assert_xyz(lidar_xyz, [0.0, 0.0, 0.165])
    assert_xyz(left_arm_xyz, [-0.065, 0.070, 0.119])
    assert_xyz(right_arm_xyz, [-0.065, -0.070, 0.119])

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
        "camera_optical_xyz_from_base_footprint_m": [round(value, 6) for value in camera_xyz],
        "lidar_xyz_from_base_footprint_m": [round(value, 6) for value in lidar_xyz],
        "left_arm_base_xyz_from_base_footprint_m": [round(value, 6) for value in left_arm_xyz],
        "right_arm_base_xyz_from_base_footprint_m": [round(value, 6) for value in right_arm_xyz],
        "parallel_gripper_primary_stroke_m": 0.037,
        "mimic_joints": [
            joint.attrib["name"] for joint in joints if joint.find("mimic") is not None
        ],
    }
    assert report["check_urdf_root"]
    assert report["mimic_joints"] == ["left_finger2_joint", "right_finger2_joint"]
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
