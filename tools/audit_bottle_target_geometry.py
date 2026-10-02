#!/usr/bin/env python3
"""저장된 병 파지 목표에서 원본 메쉬 정점 침범을 검사한다. 실물 접근 없음.

정점 침범은 겹침의 양성 근거다. 정점 0개는 삼각형/모서리 무충돌 증명이 아니다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from cup_contact_model import Chain
from workcell_preview_inputs import ARM_JOINTS
from solve_task_poses import rpy_matrix


def binary_stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError("잘린 binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    if count == 0 or len(data) != 84+50*count:
        raise ValueError("검사는 유효한 binary STL만 지원합니다")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise ValueError("메쉬 정점은 유한해야 합니다")
    return np.unique(vertices.astype(float), axis=0)


def cylinder_intrusion(vertices, center, radius, height, margin=.001):
    points, center = np.asarray(vertices, dtype=float), np.asarray(center, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or center.shape != (3,):
        raise ValueError("정점 Nx3과 중심 3벡터 필요")
    if not np.isfinite(points).all() or not np.isfinite([*center, radius, height, margin]).all():
        raise ValueError("유한한 기하 값 필요")
    if not 0 <= margin < min(radius, height/2):
        raise ValueError("침범 여유는 반지름/반높이보다 작아야 합니다")
    radial_depth = radius-np.linalg.norm(points[:, :2]-center[:2], axis=1)
    axial_depth = height/2-np.abs(points[:, 2]-center[2])
    inside = (radial_depth > margin) & (axial_depth > margin)
    return inside


def audit(run_dir):
    run_dir = Path(run_dir)
    manifest_path = run_dir / "plan.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["object_kind"] != "bottle" or manifest["active_side"] != "right":
        raise ValueError("오른손 병 실험 manifest가 필요합니다")
    model = run_dir / manifest["model"].get("proxy_file", "stock_ggao_contact_proxy.urdf")
    if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["model"]["proxy_sha256"]:
        raise ValueError("manifest와 URDF 해시가 다릅니다")
    config = manifest["config"]
    approach = next(p for p in manifest["plan"]["poses"] if p["name"] == "APPROACH")
    positions = dict(zip(("right_"+j for j in ARM_JOINTS), np.radians(approach["joint_deg"])))
    transforms = Chain(model).transforms(positions)
    root = ET.parse(model).getroot()
    link_name = "right_gripper_link"
    rows = []
    plots = []
    for collision in root.find(f"./link[@name='{link_name}']").findall("collision"):
        mesh = collision.find("geometry/mesh")
        if mesh is None:
            continue
        path = Path(mesh.get("filename"))
        vertices = binary_stl_vertices(path)*np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        origin = collision.find("origin")
        transform = np.eye(4)
        if origin is not None:
            transform[:3, :3] = rpy_matrix(*np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        world = transforms[link_name] @ transform
        vertices = vertices @ world[:3, :3].T + world[:3, 3]
        inside = cylinder_intrusion(vertices, config["cup_center_m"], config["cup_radius_m"], config["cup_height_m"])
        rows.append({"mesh": path.name, "mesh_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                     "vertex_count": len(vertices), "intruding_vertex_count": int(inside.sum()),
                     "example_inside_world_m": vertices[inside][0].tolist() if inside.any() else None})
        plots.append((path.name, vertices, inside))
    if not rows:
        raise ValueError("검사할 메쉬가 없습니다")
    report = {"schema": "bottle_target_vertex_audit_v1", "phase": "APPROACH",
              "scope": "right_gripper_link_mesh_vertices_only", "model_sha256": manifest["model"]["proxy_sha256"],
              "plan_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
              "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "center_m": config["cup_center_m"], "radius_m": config["cup_radius_m"],
              "height_m": config["cup_height_m"], "interior_margin_m": .001,
              "target_overlap_observed": any(r["intruding_vertex_count"] for r in rows),
              "collision_free_certified": False, "hardware_accessed": False, "meshes": rows}
    return report, plots


def save_plot(path, report, plots):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig, axes = plt.subplots(1, len(plots), figsize=(11, 5), squeeze=False)
    center = np.asarray(report["center_m"])*1000
    for ax, (name, vertices, inside) in zip(axes[0], plots):
        points = vertices*1000
        ax.scatter(points[:, 0], points[:, 1], s=1, color="silver", label="Original STL vertices")
        ax.scatter(points[inside, 0], points[inside, 1], s=2, color="crimson", label="Inside bottle (>1 mm)")
        ax.add_patch(Circle(center[:2], report["radius_m"]*1000, fill=False, color="royalblue", lw=2))
        ax.set(title=name, xlabel="world X (mm)", ylabel="world Y (mm)", aspect="equal")
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    fig.suptitle("Planned grasp: original geometry overlap, not a physical calibration")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    report, plots = audit(args.run_dir)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    if args.plot:
        save_plot(args.output_dir / "target_overlap.png", report, plots)
    print(json.dumps(report, indent=2))
