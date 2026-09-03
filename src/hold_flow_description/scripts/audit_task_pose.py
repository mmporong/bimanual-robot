#!/usr/bin/env python3
"""HOLD THE FLOW URDF의 task-level 준비 상태를 수치로 감사한다.

기존 validate_description.py가 구조와 fixed TF를 확인한다면, 이 스크립트는
운반·붓기 joint state, 명시적 TCP 유무, 현재 jaw visual 중심의 FK proxy와
기계 명세의 목표 좌표를 대조한다. TCP가 정의되기 전 proxy는 합격 근거가
아니며 불일치를 조기에 드러내는 진단값으로만 사용한다.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf/hold_flow.urdf"
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_2.yaml"
CALCULATION_PATH = REPO_ROOT / "design/mechanical/calculate_250mm_design.py"
PACKAGE_URI_PREFIX = "package://hold_flow_description/"
REQUIRED_TASK_FRAMES = {
    "left_tool0",
    "right_tool0",
    "left_bottle_tcp",
    "right_cup_tcp",
}


def vector(value: str | None, default: tuple[float, float, float]) -> list[float]:
    return list(default) if not value else [float(item) for item in value.split()]


def identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][index] * b[index][column] for index in range(4)) for column in range(4)]
        for row in range(4)
    ]


def rpy_rotation(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def origin_transform(element: ET.Element | None) -> list[list[float]]:
    xyz = vector(None if element is None else element.attrib.get("xyz"), (0.0, 0.0, 0.0))
    rpy = vector(None if element is None else element.attrib.get("rpy"), (0.0, 0.0, 0.0))
    rotation = rpy_rotation(rpy)
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_rotation(axis: list[float], angle: float) -> list[list[float]]:
    length = math.sqrt(sum(value * value for value in axis))
    x, y, z = [value / length for value in axis]
    c, s, one_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def translation(axis: list[float], distance: float) -> list[list[float]]:
    output = identity()
    output[0][3] = axis[0] * distance
    output[1][3] = axis[1] * distance
    output[2][3] = axis[2] * distance
    return output


def transform_point(matrix: list[list[float]], point: list[float]) -> list[float]:
    homogeneous = point + [1.0]
    return [sum(matrix[row][column] * homogeneous[column] for column in range(4)) for row in range(3)]


def stl_bounds_m(path: Path, scale: list[float]) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + count * 50:
            for offset in range(84, len(data), 50):
                values = struct.unpack("<12fH", data[offset : offset + 50])
                vertices.extend(
                    [
                        (values[3], values[4], values[5]),
                        (values[6], values[7], values[8]),
                        (values[9], values[10], values[11]),
                    ]
                )
    if not vertices:
        for line in data.decode("utf-8", errors="ignore").splitlines():
            stripped = line.strip()
            if stripped.startswith("vertex "):
                vertices.append(tuple(float(value) for value in stripped.split()[1:4]))
    if not vertices:
        raise ValueError(f"STL 파싱 실패: {path}")
    minimum = [min(vertex[axis] for vertex in vertices) * scale[axis] for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in vertices) * scale[axis] for axis in range(3)]
    return minimum, maximum


def pose_values(spec: dict, state: str) -> dict[str, float]:
    if state == "transport":
        left = spec["workspace"]["transport_joint_degrees"]["left"]
        right = spec["workspace"]["transport_joint_degrees"]["right"]
    else:
        left = spec["workspace"]["pour_joint_degrees"]["left_bottle"]
        right = spec["workspace"]["pour_joint_degrees"]["right_cup"]
    output: dict[str, float] = {}
    suffixes = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]
    for prefix, degrees in (("left", left), ("right", right)):
        output.update(
            {f"{prefix}_{suffix}": math.radians(value) for suffix, value in zip(suffixes, degrees)}
        )
        output[f"{prefix}_finger1_joint"] = 0.0325
    return output


def link_transforms(root: ET.Element, positions: dict[str, float]) -> dict[str, list[list[float]]]:
    links = {element.attrib["name"] for element in root.findall("link")}
    joints = list(root.findall("joint"))
    child_links = {joint.find("child").attrib["link"] for joint in joints}
    root_links = links - child_links
    if root_links != {"base_footprint"}:
        raise AssertionError(f"예상하지 않은 root link: {sorted(root_links)}")
    transforms = {"base_footprint": identity()}
    pending = joints[:]
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").attrib["link"]
            if parent not in transforms:
                continue
            value = positions.get(joint.attrib["name"], 0.0)
            mimic = joint.find("mimic")
            if mimic is not None:
                value = positions.get(mimic.attrib["joint"], 0.0) * float(
                    mimic.attrib.get("multiplier", "1")
                ) + float(mimic.attrib.get("offset", "0"))
            local = origin_transform(joint.find("origin"))
            axis = vector(None if joint.find("axis") is None else joint.find("axis").attrib.get("xyz"), (1, 0, 0))
            if joint.attrib["type"] in {"revolute", "continuous"}:
                local = multiply(local, axis_rotation(axis, value))
            elif joint.attrib["type"] == "prismatic":
                local = multiply(local, translation(axis, value))
            child = joint.find("child").attrib["link"]
            transforms[child] = multiply(transforms[parent], local)
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise AssertionError("URDF joint graph를 완성하지 못했습니다")
    return transforms


def jaw_visual_center(root: ET.Element, transforms: dict[str, list[list[float]]], link_name: str) -> list[float]:
    link = root.find(f"link[@name='{link_name}']")
    visual = link.find("visual")
    mesh = visual.find("geometry/mesh")
    uri = mesh.attrib["filename"]
    if not uri.startswith(PACKAGE_URI_PREFIX):
        raise ValueError(uri)
    path = PACKAGE_ROOT / uri.removeprefix(PACKAGE_URI_PREFIX)
    scale = vector(mesh.attrib.get("scale"), (1, 1, 1))
    minimum, maximum = stl_bounds_m(path, scale)
    center = [(minimum[axis] + maximum[axis]) / 2.0 for axis in range(3)]
    visual_world = multiply(transforms[link_name], origin_transform(visual.find("origin")))
    return transform_point(visual_world, center)


def midpoint(first: list[float], second: list[float]) -> list[float]:
    return [(a + b) / 2.0 for a, b in zip(first, second)]


def distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def chassis_target_to_base_footprint_m(xyz_mm: list[float]) -> list[float]:
    return [(xyz_mm[0] - 90.0) / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0]


def joint_limit_audit(root: ET.Element, positions: dict[str, float]) -> list[dict[str, object]]:
    result = []
    for name, value in positions.items():
        joint = root.find(f"joint[@name='{name}']")
        if joint is None:
            result.append({"joint": name, "status": "missing"})
            continue
        limit = joint.find("limit")
        lower = float(limit.attrib.get("lower", "-inf"))
        upper = float(limit.attrib.get("upper", "inf"))
        result.append(
            {
                "joint": name,
                "value": round(value, 6),
                "lower": lower,
                "upper": upper,
                "within_limit": lower <= value <= upper,
            }
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="task-ready가 아니면 exit 2")
    args = parser.parse_args()
    root = ET.parse(URDF_PATH).getroot()
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    link_names = {link.attrib["name"] for link in root.findall("link")}
    missing_frames = sorted(REQUIRED_TASK_FRAMES - link_names)
    calculation_text = CALCULATION_PATH.read_text(encoding="utf-8")

    states = {}
    for state in ("transport", "pour"):
        positions = pose_values(spec, state)
        transforms = link_transforms(root, positions)
        limit_results = joint_limit_audit(root, positions)
        proxy = {}
        for side in ("left", "right"):
            tool0 = f"{side}_tool0"
            if tool0 in transforms:
                proxy[side] = [transforms[tool0][axis][3] for axis in range(3)]
            else:
                first = jaw_visual_center(root, transforms, f"{side}_finger1_link")
                second = jaw_visual_center(root, transforms, f"{side}_finger2_link")
                proxy[side] = midpoint(first, second)
        if state == "transport":
            targets = {
                "left": chassis_target_to_base_footprint_m([157.236, 133.610, 415.848]),
                "right": chassis_target_to_base_footprint_m([144.204, -134.074, 424.740]),
            }
        else:
            targets = {
                "left": chassis_target_to_base_footprint_m(spec["workspace"]["bottle_gripper_pour_xyz"]),
                "right": chassis_target_to_base_footprint_m(spec["workspace"]["cup_gripper_xyz"]),
            }
        states[state] = {
            "joint_limits_pass": all(item.get("within_limit", False) for item in limit_results),
            "joint_limit_violations": [
                item for item in limit_results if not item.get("within_limit", False)
            ],
            "tool0_from_base_footprint_m": {
                side: [round(value, 4) for value in proxy[side]] for side in proxy
            },
            "documented_target_from_base_footprint_m": {
                side: [round(value, 4) for value in targets[side]] for side in targets
            },
            "tool0_to_target_gap_mm": {
                side: round(distance(proxy[side], targets[side]) * 1000.0, 1) for side in proxy
            },
        }

    blockers = []
    if missing_frames:
        blockers.append("명시적 좌우 tool0와 병/컵 TCP frame이 없다")
    if "Component(\"left_parallel_gripper\"" in calculation_text:
        blockers.append("계산 스크립트의 task 위치가 URDF FK가 아니라 상수로 저장되어 있다")
    if any(
        gap > 50.0
        for state in states.values()
        for gap in state["tool0_to_target_gap_mm"].values()
    ):
        blockers.append("현재 joint state의 tool0 가 문서 목표와 크게 다르다")
    if any(not state["joint_limits_pass"] for state in states.values()):
        blockers.append("문서의 붓기 후보 관절값 일부가 현재 URDF hard limit 밖이다")

    report = {
        "urdf": str(URDF_PATH.relative_to(REPO_ROOT)),
        "links": len(root.findall("link")),
        "joints": len(root.findall("joint")),
        "required_task_frames": sorted(REQUIRED_TASK_FRAMES),
        "missing_task_frames": missing_frames,
        "states": states,
        "tcp_note": "tool0 는 두 죠 사이 파지 기준이다. 손목 장착 오프셋은 실물 확정 전 명목값이다",
        "task_ready": not blockers,
        "blocking_findings": blockers,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.strict and blockers:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
