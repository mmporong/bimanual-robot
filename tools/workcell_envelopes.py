"""URDF collision 외곽으로 물체 관통 후보를 거르는 보수적 프리뷰 검사.

메시의 로컬 AABB를 변환한 월드 AABB를 사용하므로 오탐이 가능하다.
통과는 물리 접촉·자가충돌·연속 경로·실물 안전 인증을 뜻하지 않는다.
"""
from __future__ import annotations

from itertools import product
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

from preview_finray_tcp import _stl_vertices

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/hold_flow_description/scripts"))
from solve_task_poses import ARM_JOINTS, Chain, rpy_matrix


def box_corners(lower, upper):
    return np.asarray(list(product(*zip(lower, upper))), dtype=float)


def cylinder_overlap(bounds, center, radius, bottom, top, margin=0.0):
    low, high = bounds
    if high[2] < bottom-margin or low[2] > top+margin:
        return False
    nearest = np.clip(np.asarray(center), low[:2], high[:2])
    return bool(np.linalg.norm(nearest-center) <= radius+margin)


class WorkcellEnvelopes:
    def __init__(self, urdf: Path, scene: dict):
        self.chain = Chain(urdf)
        self.scene = scene
        for key in ("table_center_xy_m", "table_size_xy_m", "cup_center_xy_m", "bottle_center_xy_m"):
            value = np.asarray(scene[key], dtype=float)
            if value.shape != (2,) or not np.isfinite(value).all():
                raise ValueError(f"{key}는 유한한 XY 2벡터여야 합니다")
            if key == "table_size_xy_m" and np.any(value <= 0):
                raise ValueError("책상 크기는 양수여야 합니다")
        for key in ("table_surface_z_m", "cup_radius_m", "cup_height_m", "bottle_radius_m", "bottle_height_m"):
            value = scene[key]
            if type(value) not in (int, float) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{key}는 유한한 양수여야 합니다")
        self.shapes = []
        root = ET.parse(urdf).getroot()
        for link in root.findall("link"):
            name = link.get("name")
            if not name.startswith(("left_", "right_")) or name in {
                    "left_base_link", "right_base_link", "left_wheel_link", "right_wheel_link",
                    "left_arm_backing_link", "right_arm_backing_link"}:
                continue
            for collision in link.findall("collision"):
                origin = collision.find("origin")
                xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
                rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
                geometry = collision.find("geometry")
                shape = list(geometry)[0]
                if shape.tag == "mesh":
                    filename = shape.get("filename")
                    prefix = "package://hold_flow_description/"
                    mesh = ROOT / "src/hold_flow_description" / filename[len(prefix):] if filename.startswith(prefix) else Path(filename)
                    vertices = _stl_vertices(mesh.read_bytes())
                    vertices *= np.fromstring(shape.get("scale", "1 1 1"), sep=" ")
                    low, high = vertices.min(0), vertices.max(0)
                elif shape.tag == "box":
                    high = np.fromstring(shape.get("size"), sep=" ") / 2
                    low = -high
                elif shape.tag == "cylinder":
                    high = np.array([float(shape.get("radius"))]*2+[float(shape.get("length"))/2])
                    low = -high
                elif shape.tag == "sphere":
                    high = np.full(3, float(shape.get("radius")))
                    low = -high
                else:
                    raise ValueError(f"지원하지 않는 collision 형상: {shape.tag}")
                corners = box_corners(low, high) @ rpy_matrix(*rpy).T + xyz
                self.shapes.append((name, corners))

    def check(self, left_deg, right_deg, left_open=1.0, right_open=.0433):
        for name, value in (("left_gripper", left_open), ("right_finger1_joint", right_open),
                            ("right_finger2_joint", right_open)):
            lower, upper = self.chain.limits[name]
            if not np.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"{name} 개방값이 유한하지 않거나 관절 한계 밖입니다")
        positions = {"left_gripper": left_open,
                     "right_finger1_joint": right_open, "right_finger2_joint": right_open}
        limits = []
        for side, values in (("left", left_deg), ("right", right_deg)):
            q = np.asarray(values, dtype=float)
            if q.shape != (5,) or not np.isfinite(q).all():
                raise ValueError("양팔 관절값은 각각 유한한 5개 각도여야 합니다")
            for joint, value in zip(ARM_JOINTS, np.radians(q)):
                name = f"{side}_{joint}"
                positions[name] = value
                lower, upper = self.chain.limits[name]
                if not lower <= value <= upper:
                    limits.append(name)
        transforms = self.chain.transforms(positions)
        bounds = []
        for name, corners in self.shapes:
            tf = transforms[name]
            points = corners @ tf[:3, :3].T + tf[:3, 3]
            bounds.append((name, (points.min(0), points.max(0))))
        hits = set()
        z = self.scene["table_surface_z_m"]
        xy = np.asarray(self.scene["table_center_xy_m"])
        size = np.asarray(self.scene["table_size_xy_m"])
        table_low = np.r_[xy-size/2, z-.03]
        table_high = np.r_[xy+size/2, z]
        for name, bound in bounds:
            for obj in ("cup", "bottle"):
                if cylinder_overlap(bound, np.asarray(self.scene[f"{obj}_center_xy_m"]),
                                    self.scene[f"{obj}_radius_m"], z,
                                    z+self.scene[f"{obj}_height_m"]):
                    hits.add((name, obj))
            if np.all(bound[1] >= table_low) and np.all(bound[0] <= table_high):
                hits.add((name, "table"))
        for left, lb in bounds:
            if not left.startswith("left_"):
                continue
            for right, rb in bounds:
                if right.startswith("right_") and np.all(lb[1] >= rb[0]) and np.all(lb[0] <= rb[1]):
                    hits.add((left, right))
        return {"clear": not hits and not limits,
                "overlap_candidates": [list(pair) for pair in sorted(hits)],
                "joint_limit_violations": limits,
                "method": "conservative_collision_aabb_vs_object_cylinder_table_and_other_arm",
                "physical_contact_validated": False,
                "self_and_robot_structure_collision_validated": False}
