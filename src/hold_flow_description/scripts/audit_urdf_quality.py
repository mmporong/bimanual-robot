#!/usr/bin/env python3
"""URDF의 질량·관성·관절 한계·메시 스케일을 정적 감사한다.

구조 검증(validate_description.py)과 task pose 검증(audit_task_pose.py) 사이의
빈틈을 메운다. P0에서 허용하는 관성/형상 없는 프레임은 명시적으로 제한한다.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf/hold_flow.urdf"
PACKAGE_URI_PREFIX = "package://hold_flow_description/"
# tool0 는 두 죠 사이 파지 기준 프레임이라 형상이 없는 것이 정상이다.
FRAME_ONLY_LINKS = {
    "base_footprint",
    "camera_depth_optical_frame",
    "left_tool0",
    "right_tool0",
    "left_bottle_tcp",
    "right_bottle_tcp",
    "left_cup_tcp",
    "right_cup_tcp",
}
GEOMETRY_FREE_LINKS = FRAME_ONLY_LINKS | {"base_link"}
AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic"}


def determinant_3x3(matrix: list[list[float]]) -> float:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def principal_moments(matrix: list[list[float]]) -> list[float]:
    """대칭 3×3 행렬의 고유값을 외부 패키지 없이 계산한다."""
    trace = sum(matrix[index][index] for index in range(3))
    mean = trace / 3.0
    centered = [
        [matrix[row][column] - (mean if row == column else 0.0) for column in range(3)]
        for row in range(3)
    ]
    p2 = sum(centered[index][index] ** 2 for index in range(3)) + 2.0 * (
        centered[0][1] ** 2 + centered[0][2] ** 2 + centered[1][2] ** 2
    )
    if p2 <= 1e-30:
        return [mean, mean, mean]
    p = math.sqrt(p2 / 6.0)
    normalized = [[value / p for value in row] for row in centered]
    r = max(-1.0, min(1.0, determinant_3x3(normalized) / 2.0))
    angle = math.acos(r) / 3.0
    values = [
        mean + 2.0 * p * math.cos(angle),
        mean + 2.0 * p * math.cos(angle + 2.0 * math.pi / 3.0),
        mean + 2.0 * p * math.cos(angle + 4.0 * math.pi / 3.0),
    ]
    return sorted(values)


def inertia_matrix(element: ET.Element) -> list[list[float]]:
    values = {name: float(element.attrib.get(name, "0")) for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    return [
        [values["ixx"], values["ixy"], values["ixz"]],
        [values["ixy"], values["iyy"], values["iyz"]],
        [values["ixz"], values["iyz"], values["izz"]],
    ]


def resolve_mesh(uri: str) -> Path | None:
    if not uri.startswith(PACKAGE_URI_PREFIX):
        return None
    return PACKAGE_ROOT / uri.removeprefix(PACKAGE_URI_PREFIX)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true", help="문제가 있으면 exit 2")
    args = parser.parse_args()
    root = ET.parse(URDF_PATH).getroot()
    issues: list[str] = []

    links = root.findall("link")
    inertial_links = 0
    for link in links:
        name = link.attrib["name"]
        inertial = link.find("inertial")
        if inertial is None:
            if name not in FRAME_ONLY_LINKS:
                issues.append(f"{name}: inertial 누락")
            continue
        inertial_links += 1
        mass = float(inertial.find("mass").attrib["value"])
        if not math.isfinite(mass) or mass <= 0.0:
            issues.append(f"{name}: mass가 양수가 아님 ({mass})")
        matrix = inertia_matrix(inertial.find("inertia"))
        moments = principal_moments(matrix)
        if any(not math.isfinite(value) or value <= 0.0 for value in moments):
            issues.append(f"{name}: inertia가 positive definite가 아님 ({moments})")
        if moments[0] + moments[1] + 1e-12 < moments[2]:
            issues.append(f"{name}: principal inertia triangle inequality 위반 ({moments})")

    visual_links = {link.attrib["name"] for link in links if link.find("visual") is not None}
    collision_links = {link.attrib["name"] for link in links if link.find("collision") is not None}
    if visual_links != collision_links:
        issues.append(
            "visual/collision link 집합 불일치: "
            f"visual-only={sorted(visual_links - collision_links)}, "
            f"collision-only={sorted(collision_links - visual_links)}"
        )
    geometry_missing = {link.attrib["name"] for link in links if link.find("visual") is None}
    if geometry_missing != GEOMETRY_FREE_LINKS:
        issues.append(f"예상하지 않은 geometry-free link 집합: {sorted(geometry_missing)}")

    mesh_references = 0
    unique_meshes: set[Path] = set()
    for mesh in root.findall(".//mesh"):
        mesh_references += 1
        uri = mesh.attrib["filename"]
        path = resolve_mesh(uri)
        if path is None:
            issues.append(f"지원하지 않는 mesh URI: {uri}")
        elif not path.is_file():
            issues.append(f"mesh 파일 없음: {uri}")
        else:
            unique_meshes.add(path)
        scale = [float(value) for value in mesh.attrib.get("scale", "1 1 1").split()]
        if len(scale) != 3 or any(not math.isfinite(value) or value <= 0.0 or value > 1.0 for value in scale):
            issues.append(f"비정상 mesh scale: {uri} -> {scale}")

    axis_joints = 0
    bounded_joints = 0
    safety_joints = 0
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        joint_type = joint.attrib["type"]
        if joint_type in AXIS_JOINT_TYPES:
            axis_joints += 1
            axis = [float(value) for value in joint.find("axis").attrib["xyz"].split()]
            norm = math.sqrt(sum(value * value for value in axis))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
                issues.append(f"{name}: axis norm={norm:.6f}")
        limit = joint.find("limit")
        if joint_type in {"revolute", "prismatic"}:
            bounded_joints += 1
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                issues.append(f"{name}: bounded joint limit 누락")
            elif float(limit.attrib["lower"]) > float(limit.attrib["upper"]):
                issues.append(f"{name}: lower > upper")
        if joint_type in AXIS_JOINT_TYPES:
            if limit is None:
                issues.append(f"{name}: effort/velocity limit 누락")
            else:
                for field in ("effort", "velocity"):
                    value = float(limit.attrib.get(field, "nan"))
                    if not math.isfinite(value) or value <= 0.0:
                        issues.append(f"{name}: {field}가 양수가 아님 ({value})")
        safety = joint.find("safety_controller")
        if safety is not None:
            safety_joints += 1
            if limit is None:
                issues.append(f"{name}: hard limit 없이 safety_controller 존재")
            elif joint_type == "revolute":
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
                soft_lower = float(safety.attrib["soft_lower_limit"])
                soft_upper = float(safety.attrib["soft_upper_limit"])
                if not (lower <= soft_lower <= soft_upper <= upper):
                    issues.append(f"{name}: safety limit가 hard limit 밖")

    report = {
        "urdf": str(URDF_PATH.relative_to(REPO_ROOT)),
        "links": len(links),
        "inertial_links": inertial_links,
        "visual_links": len(visual_links),
        "collision_links": len(collision_links),
        "mesh_references": mesh_references,
        "unique_mesh_files": len(unique_meshes),
        "axis_joints": axis_joints,
        "bounded_joints": bounded_joints,
        "safety_controller_joints": safety_joints,
        "issues": issues,
        "quality_ready": not issues,
        "scope_note": "수치 일관성 감사이며 실물 질량특성·TCP·trajectory collision 검증을 대체하지 않는다",
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.strict and issues:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
