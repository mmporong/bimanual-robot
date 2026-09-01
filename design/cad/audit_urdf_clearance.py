#!/usr/bin/env python3
"""현재 URDF collision mesh로 zero/transport 자세의 간섭과 여유를 감사한다."""

from __future__ import annotations

import argparse
import json
import math

import numpy as np
import vtk
import yaml

from render_design_handoff import (
    REPO_ROOT,
    SPEC_PATH,
    URDF_PATH,
    UrdfScene,
    parse_vector,
    pose_positions,
    resolve_package_mesh,
    transform,
    vtk_matrix,
)


def collision_polydata(scene: UrdfScene, link_name: str, transforms: dict[str, np.ndarray]) -> vtk.vtkPolyData | None:
    link = scene.links[link_name]
    collision = link.find("collision")
    if collision is None:
        return None
    origin = collision.find("origin")
    xyz = parse_vector(None if origin is None else origin.attrib.get("xyz"), (0, 0, 0))
    rpy = parse_vector(None if origin is None else origin.attrib.get("rpy"), (0, 0, 0))
    matrix = transforms[link_name] @ transform(xyz, rpy)
    geometry = collision.find("geometry")
    mesh = geometry.find("mesh")
    if mesh is not None:
        source = vtk.vtkSTLReader()
        source.SetFileName(str(resolve_package_mesh(mesh.attrib["filename"])))
        scale = parse_vector(mesh.attrib.get("scale"), (1, 1, 1)) * 1000.0
        scale_matrix = np.eye(4)
        scale_matrix[0, 0], scale_matrix[1, 1], scale_matrix[2, 2] = scale
        matrix = matrix @ scale_matrix
    elif geometry.find("box") is not None:
        size = [float(value) * 1000.0 for value in geometry.find("box").attrib["size"].split()]
        source = vtk.vtkCubeSource()
        source.SetXLength(size[0])
        source.SetYLength(size[1])
        source.SetZLength(size[2])
    elif geometry.find("cylinder") is not None:
        element = geometry.find("cylinder")
        source = vtk.vtkCylinderSource()
        source.SetRadius(float(element.attrib["radius"]) * 1000.0)
        source.SetHeight(float(element.attrib["length"]) * 1000.0)
        source.SetResolution(64)
        matrix = matrix @ transform(rpy=np.array([math.pi / 2.0, 0.0, 0.0]))
    elif geometry.find("sphere") is not None:
        source = vtk.vtkSphereSource()
        source.SetRadius(float(geometry.find("sphere").attrib["radius"]) * 1000.0)
        source.SetThetaResolution(48)
        source.SetPhiResolution(32)
    else:
        return None

    vtk_transform = vtk.vtkTransform()
    vtk_transform.SetMatrix(vtk_matrix(matrix))
    transformed = vtk.vtkTransformPolyDataFilter()
    transformed.SetInputConnection(source.GetOutputPort())
    transformed.SetTransform(vtk_transform)
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(transformed.GetOutputPort())
    triangles.Update()
    return triangles.GetOutput()


def aabb_gap(first: vtk.vtkPolyData, second: vtk.vtkPolyData) -> float:
    a, b = first.GetBounds(), second.GetBounds()
    gaps = [
        max(b[0] - a[1], a[0] - b[1], 0.0),
        max(b[2] - a[3], a[2] - b[3], 0.0),
        max(b[4] - a[5], a[4] - b[5], 0.0),
    ]
    return float(np.linalg.norm(gaps))


def intersects(first: vtk.vtkPolyData, second: vtk.vtkPolyData) -> bool:
    collision = vtk.vtkCollisionDetectionFilter()
    collision.SetInputData(0, first)
    collision.SetInputData(1, second)
    collision.SetTransform(0, vtk.vtkTransform())
    collision.SetTransform(1, vtk.vtkTransform())
    collision.SetCollisionModeToFirstContact()
    collision.SetBoxTolerance(0.0)
    collision.SetCellTolerance(0.0)
    collision.SetNumberOfCellsPerNode(2)
    collision.Update()
    return collision.GetNumberOfContacts() > 0


def directed_surface_distance(points: vtk.vtkPolyData, surface: vtk.vtkPolyData) -> float:
    implicit = vtk.vtkImplicitPolyDataDistance()
    implicit.SetInput(surface)
    source_points = points.GetPoints()
    return min(
        abs(implicit.EvaluateFunction(source_points.GetPoint(index)))
        for index in range(source_points.GetNumberOfPoints())
    )


def surface_distance(first: vtk.vtkPolyData, second: vtk.vtkPolyData) -> float:
    return min(directed_surface_distance(first, second), directed_surface_distance(second, first))


def group_min_distance(polys: dict[str, vtk.vtkPolyData], pairs: list[tuple[str, str]]) -> tuple[float, tuple[str, str]]:
    candidates = []
    for first, second in pairs:
        if aabb_gap(polys[first], polys[second]) <= 80.0:
            candidates.append((surface_distance(polys[first], polys[second]), (first, second)))
    return min(candidates, default=(float("inf"), ("", "")))


def mast_keepout_radius(polys: dict[str, vtk.vtkPolyData], arm_links: list[str]) -> tuple[float, str]:
    # base_footprint 기준 마스트 중심 (-170, 0) mm, 유효 높이 약 127~800 mm.
    best = (float("inf"), "")
    for name in arm_links:
        points = polys[name].GetPoints()
        for index in range(points.GetNumberOfPoints()):
            x, y, z = points.GetPoint(index)
            if 127.0 <= z <= 800.0:
                radius = math.hypot(x + 170.0, y)
                if radius < best[0]:
                    best = (radius, name)
    return best


def audit_state(scene: UrdfScene, positions: dict[str, float]) -> dict[str, object]:
    transforms = scene.link_transforms(positions)
    left = [
        name for name in scene.links
        if name.startswith("left_") and name not in {"left_arm_backing_link", "left_wheel_link"}
    ]
    right = [
        name for name in scene.links
        if name.startswith("right_") and name not in {"right_arm_backing_link", "right_wheel_link"}
    ]
    grippers = ["left_clamp_1", "left_clamp_2", "right_clamp_1", "right_clamp_2"]
    arm_links = left + right
    extra = ["camera_mast_lower_link", "laser_link"]
    polys = {
        name: collision_polydata(scene, name, transforms)
        for name in arm_links + extra
    }
    polys = {name: poly for name, poly in polys.items() if poly is not None}

    cross_pairs = [(first, second) for first in left for second in right]
    triangle_hits = [pair for pair in cross_pairs if aabb_gap(polys[pair[0]], polys[pair[1]]) == 0 and intersects(polys[pair[0]], polys[pair[1]])]
    cross_non_gripper_pairs = [
        pair for pair in cross_pairs if pair[0] not in grippers and pair[1] not in grippers
    ]
    gripper_other_pairs = [
        pair for pair in cross_pairs if (pair[0] in grippers) ^ (pair[1] in grippers)
    ]
    cross_distance, cross_pair = group_min_distance(polys, cross_non_gripper_pairs)
    gripper_distance, gripper_pair = group_min_distance(polys, gripper_other_pairs)
    lidar_distance, lidar_pair = group_min_distance(
        polys,
        [(name, "laser_link") for name in arm_links],
    )
    keepout_radius, keepout_link = mast_keepout_radius(polys, arm_links)
    return {
        "triangle_collision_count_cross_arm": len(triangle_hits),
        "triangle_collision_pairs_cross_arm": [list(pair) for pair in triangle_hits],
        "minimum_cross_arm_link_clearance_mm": round(cross_distance, 2),
        "minimum_cross_arm_link_pair": list(cross_pair),
        "minimum_gripper_to_other_arm_clearance_mm": round(gripper_distance, 2),
        "minimum_gripper_to_other_arm_pair": list(gripper_pair),
        "minimum_arm_to_lidar_surface_clearance_mm": round(lidar_distance, 2),
        "minimum_arm_to_lidar_pair": list(lidar_pair),
        "minimum_mast_centerline_radius_mm": round(keepout_radius, 2),
        "minimum_mast_centerline_link": keepout_link,
        "requirements_mm": {
            "cross_arm_link": 25.0,
            "gripper_to_other_arm": 40.0,
            "mast_centerline_keepout_radius": 55.0,
        },
        "passes": (
            not triangle_hits
            and cross_distance >= 25.0
            and gripper_distance >= 40.0
            and keepout_radius >= 55.0
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    scene = UrdfScene(URDF_PATH)
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    transport = {}
    transport.update(pose_positions("left", spec["workspace"]["transport_joint_degrees"]["left"]))
    transport.update(pose_positions("right", spec["workspace"]["transport_joint_degrees"]["right"]))
    report = {
        "urdf": str(URDF_PATH.relative_to(REPO_ROOT)),
        "method": "VTK triangle collision + bidirectional vertex-to-surface distance",
        "states": {
            "zero": audit_state(scene, {}),
            "transport_candidate": audit_state(scene, transport),
        },
    }
    report["task_clearance_ready"] = report["states"]["transport_candidate"]["passes"]
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.strict and not report["task_clearance_ready"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
