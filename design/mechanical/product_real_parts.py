#!/usr/bin/env python3
"""URDF visual geometry helpers for dimension-faithful product illustrations.

The returned coordinates are millimetres in the product drawing convention
(x forward, y left, z up).  Mesh vertices and URDF origins are never rescaled
for appearance: the only conversion is metres to millimetres after applying
the URDF ``mesh scale`` and forward kinematics.

This module intentionally depends only on NumPy and the Python standard
library.  It reads binary or ASCII STL without trimesh/scipy and preserves all
source triangles.  It is a visual/FK helper, not a collision or structural
validator.
"""

from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path
import struct
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
PACKAGE_ROOT = REPO_ROOT / "src/hold_flow_description"

# A compact upright display pose.  All values are inside the limits recorded
# in hold_flow.urdf.  It is selected only to make scale legible in drawings.
DEFAULT_JOINT_POSITIONS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -math.pi / 2.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.35,
    "finger1_joint": 0.020,
}

_FALLBACK_COLOR = "#34363b"


def _values(text: str | None, count: int, default: float = 0.0) -> np.ndarray:
    if text is None:
        return np.full(count, default, dtype=float)
    result = np.fromstring(text, sep=" ", dtype=float)
    if result.size != count:
        raise ValueError(f"expected {count} values, got {result.size}: {text!r}")
    return result


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    """Return the URDF fixed-axis Rz(yaw) @ Ry(pitch) @ Rx(roll) rotation."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _origin_transform(element: ET.Element) -> np.ndarray:
    origin = element.find("origin")
    result = np.eye(4, dtype=float)
    if origin is not None:
        result[:3, :3] = _rpy_matrix(_values(origin.get("rpy"), 3))
        result[:3, 3] = _values(origin.get("xyz"), 3)
    return result


def _axis_transform(axis: Sequence[float], position: float, joint_type: str) -> np.ndarray:
    result = np.eye(4, dtype=float)
    vector = np.asarray(axis, dtype=float)
    length = float(np.linalg.norm(vector))
    if length == 0.0:
        raise ValueError("joint axis must be non-zero")
    vector /= length
    if joint_type in {"revolute", "continuous"}:
        x, y, z = vector
        skew = np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)), dtype=float)
        result[:3, :3] = (
            np.eye(3)
            + math.sin(position) * skew
            + (1.0 - math.cos(position)) * (skew @ skew)
        )
    elif joint_type == "prismatic":
        result[:3, 3] = vector * position
    return result


def _transform_triangles(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return triangles @ transform[:3, :3].T + transform[:3, 3]


@lru_cache(maxsize=None)
def _read_stl(path_text: str) -> np.ndarray:
    """Load every triangle from a binary or ASCII STL as float64."""
    path = Path(path_text)
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + count * 50
        if expected_size == len(data):
            record = np.dtype(
                [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]
            )
            triangles = np.frombuffer(data, dtype=record, count=count, offset=84)["vertices"]
            result = np.array(triangles, dtype=float, copy=True)
            result.setflags(write=False)
            return result

    vertices: list[list[float]] = []
    for raw_line in data.decode("ascii", errors="strict").splitlines():
        fields = raw_line.strip().split()
        if len(fields) == 4 and fields[0].lower() == "vertex":
            vertices.append([float(value) for value in fields[1:]])
    if not vertices or len(vertices) % 3:
        raise ValueError(f"not a valid binary or ASCII STL: {path}")
    result = np.asarray(vertices, dtype=float).reshape((-1, 3, 3))
    result.setflags(write=False)
    return result


def _box_triangles(size: Sequence[float]) -> np.ndarray:
    half = np.asarray(size, dtype=float) / 2.0
    corners = np.array(
        [
            [-half[0], -half[1], -half[2]], [half[0], -half[1], -half[2]],
            [half[0], half[1], -half[2]], [-half[0], half[1], -half[2]],
            [-half[0], -half[1], half[2]], [half[0], -half[1], half[2]],
            [half[0], half[1], half[2]], [-half[0], half[1], half[2]],
        ],
        dtype=float,
    )
    faces = np.array(
        [
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
        ],
        dtype=int,
    )
    return corners[faces]


@lru_cache(maxsize=1)
def _model() -> tuple[ET.Element, dict[str, str], dict[str, ET.Element], dict[str, list[ET.Element]]]:
    root = ET.parse(DEFAULT_URDF).getroot()
    colors: dict[str, str] = {}
    for material in root.findall("material"):
        color = material.find("color")
        if material.get("name") and color is not None:
            rgba = _values(color.get("rgba"), 4)
            rgb = np.clip(np.rint(rgba[:3] * 255.0), 0, 255).astype(int)
            colors[material.get("name", "")] = "#{:02x}{:02x}{:02x}".format(*rgb)
    links = {link.get("name", ""): link for link in root.findall("link")}
    child_joints: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        if parent is not None:
            child_joints.setdefault(parent.get("link", ""), []).append(joint)
    return root, colors, links, child_joints


def _mesh_path(filename: str) -> Path:
    package_prefix = "package://hold_flow_description/"
    if filename.startswith(package_prefix):
        return PACKAGE_ROOT / filename.removeprefix(package_prefix)
    path = Path(filename)
    return path if path.is_absolute() else DEFAULT_URDF.parent / path


def _joint_position(
    joint: ET.Element, prefix: str, supplied: Mapping[str, float]
) -> float:
    full_name = joint.get("name", "")
    short_name = full_name.removeprefix(prefix)
    if full_name in supplied:
        position = float(supplied[full_name])
    elif short_name in supplied:
        position = float(supplied[short_name])
    else:
        mimic = joint.find("mimic")
        if mimic is not None:
            source_full = mimic.get("joint", "")
            source_short = source_full.removeprefix(prefix)
            source = float(
                supplied.get(
                    source_full,
                    supplied.get(source_short, DEFAULT_JOINT_POSITIONS.get(source_short, 0.0)),
                )
            )
            position = source * float(mimic.get("multiplier", "1")) + float(mimic.get("offset", "0"))
        else:
            position = float(DEFAULT_JOINT_POSITIONS.get(short_name, 0.0))

    limit = joint.find("limit")
    if limit is not None and joint.get("type") != "continuous":
        lower = float(limit.get("lower", "-inf"))
        upper = float(limit.get("upper", "inf"))
        if not lower <= position <= upper:
            raise ValueError(f"{full_name}={position} is outside [{lower}, {upper}]")
    return position


def _visual_triangles(visual: ET.Element) -> np.ndarray:
    geometry = visual.find("geometry")
    if geometry is None:
        raise ValueError("visual has no geometry")
    mesh = geometry.find("mesh")
    box = geometry.find("box")
    if mesh is not None:
        triangles = np.array(_read_stl(str(_mesh_path(mesh.get("filename", "")))), copy=True)
        triangles *= _values(mesh.get("scale"), 3, default=1.0)[None, None, :]
        return triangles
    if box is not None:
        return _box_triangles(_values(box.get("size"), 3))
    raise NotImplementedError("arm visual geometry must be an STL mesh or box")


def arm_visuals(
    prefix: str = "left_",
    mount_xyz_mm: Sequence[float] = (0.0, 75.0, 726.0),
    joint_positions: Mapping[str, float] | None = None,
) -> list[tuple[np.ndarray, str]]:
    """Return all reachable arm visual triangles in millimetres.

    ``mount_xyz_mm`` locates ``<prefix>base_link``.  Joint positions may use
    either complete URDF names (``left_shoulder_lift``) or names with the arm
    prefix removed (``shoulder_lift``).
    """
    if prefix not in {"left_", "right_"}:
        raise ValueError("prefix must be 'left_' or 'right_'")
    mount = _values(" ".join(str(value) for value in mount_xyz_mm), 3)
    supplied = dict(joint_positions or {})
    _, colors, links, child_joints = _model()
    root_link = f"{prefix}base_link"
    if root_link not in links:
        raise KeyError(f"URDF has no {root_link}")

    result: list[tuple[np.ndarray, str]] = []
    pending: list[tuple[str, np.ndarray]] = [(root_link, np.eye(4, dtype=float))]
    while pending:
        link_name, link_transform = pending.pop()
        link = links[link_name]
        for visual in link.findall("visual"):
            material = visual.find("material")
            color = colors.get(material.get("name", "") if material is not None else "", _FALLBACK_COLOR)
            world = link_transform @ _origin_transform(visual)
            triangles_m = _transform_triangles(_visual_triangles(visual), world)
            result.append((triangles_m * 1000.0 + mount[None, None, :], color))

        for joint in child_joints.get(link_name, []):
            child = joint.find("child")
            if child is None:
                continue
            child_name = child.get("link", "")
            if not child_name.startswith(prefix):
                continue
            transform = link_transform @ _origin_transform(joint)
            joint_type = joint.get("type", "fixed")
            if joint_type != "fixed":
                axis_element = joint.find("axis")
                axis = _values(axis_element.get("xyz") if axis_element is not None else None, 3)
                transform = transform @ _axis_transform(
                    axis, _joint_position(joint, prefix, supplied), joint_type
                )
            pending.append((child_name, transform))
    return result


def arm_metadata(
    prefix: str = "left_",
    mount_xyz_mm: Sequence[float] = (0.0, 75.0, 726.0),
    joint_positions: Mapping[str, float] | None = None,
) -> dict[str, object]:
    """Report provenance and exact rendered bounds for an arm visual set."""
    visuals = arm_visuals(prefix, mount_xyz_mm, joint_positions)
    vertices = np.concatenate([triangles.reshape((-1, 3)) for triangles, _ in visuals])
    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)
    supplied = dict(joint_positions or {})
    pose_names = [
        name
        for name in DEFAULT_JOINT_POSITIONS
        if not (prefix == "right_" and name == "gripper")
        and not (prefix == "left_" and name == "finger1_joint")
    ]
    resolved_pose = {
        name: float(
            supplied.get(
                f"{prefix}{name}",
                supplied.get(name, DEFAULT_JOINT_POSITIONS[name]),
            )
        )
        for name in pose_names
    }
    return {
        "prefix": prefix,
        "source_urdf": str(DEFAULT_URDF),
        "mount_xyz_mm": [float(value) for value in mount_xyz_mm],
        "joint_positions_rad_or_m": resolved_pose,
        "bounds_min_mm": minimum.tolist(),
        "bounds_max_mm": maximum.tolist(),
        "size_mm": (maximum - minimum).tolist(),
        "visual_count": len(visuals),
        "triangle_count": int(sum(len(triangles) for triangles, _ in visuals)),
        "triangle_policy": "source STL triangles preserved; URDF boxes triangulated exactly",
        "scale_policy": "URDF mesh scale, transforms, then metres-to-millimetres only",
        "collision_checked": False,
        "right_gripper_geometry_note": (
            "right parallel gripper visuals are URDF dimension proxies, not redistributed upstream CAD"
            if prefix == "right_"
            else None
        ),
    }


if __name__ == "__main__":
    import json

    print(json.dumps({side: arm_metadata(f"{side}_") for side in ("left", "right")}, indent=2))
