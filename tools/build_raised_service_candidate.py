"""Materialize a simulation-only raised tray and recompute the complete pour plan."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from bimanual_pour_plan import build_plan
from raised_tray_transfer import build_raised_transfer


def add_box(root, name, xyz, size, mass):
    link = ET.SubElement(root, "link", name=name)
    inertia = ET.SubElement(link, "inertial")
    ET.SubElement(inertia, "mass", value=str(mass))
    x, y, z = size
    ET.SubElement(inertia, "inertia", ixx=str(mass*(y*y+z*z)/12),
                  iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                  ixy="0", ixz="0", iyz="0")
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind)
        ET.SubElement(ET.SubElement(element, "geometry"), "box", size=" ".join(map(str, size)))
        if kind == "visual":
            ET.SubElement(ET.SubElement(element, "material", name=name+"_material"),
                          "color", rgba="0.15 0.3 0.4 1")
    joint = ET.SubElement(root, "joint", name=name+"_joint", type="fixed")
    ET.SubElement(joint, "parent", link="base_footprint")
    ET.SubElement(joint, "child", link=name)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, xyz)), rpy="0 0 0")


def build(source_dir, report_path, label, output):
    report = json.loads(report_path.read_text())
    for filename, key in (("plan.json", "source_plan_sha256"),
                          ("replacement_hypothesis.urdf", "source_urdf_sha256")):
        if hashlib.sha256((source_dir/filename).read_bytes()).hexdigest() != report[key]:
            raise ValueError("search source hash mismatch: "+filename)
    candidate = next(c for c in report["candidates"] if c["label"] == label)
    if not candidate["accepted"]:
        raise ValueError("candidate must pass CPU search before materialization")
    height = float(candidate["tray_height_offset_m"])
    if not .04 <= height <= .20:
        raise ValueError("unsupported experimental tray height")
    model = Path(candidate["model"])
    if hashlib.sha256(model.read_bytes()).hexdigest() != candidate["model_sha256"]:
        raise ValueError("candidate model hash mismatch")
    output.mkdir(parents=True, exist_ok=False)
    root = ET.parse(model).getroot()
    # Simulation assumptions, not a mass measurement or printable manufacturing design.
    add_box(root, "central_tray_top_link", (0., 0., .72+height-.003), (.11, .11, .006), .05)
    add_box(root, "central_tray_post_link", (0., 0., .72+(height-.006)/2),
            (.025, .025, height-.006), .025)
    ET.indent(root)
    target = output/"replacement_hypothesis.urdf"
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    source = json.loads((source_dir/"plan.json").read_text())
    source["parent_model_provenance"] = source.get("model")
    source["model"] = {"proxy_sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                       "proxy_urdf": str(target), "simulation_only": True,
                       "source_model_sha256": report["source_urdf_sha256"]}
    source["plan"] = build_plan(target, source["left_config"], source["right_config"])
    stages = {s["name"]: s for s in candidate["stages"]}
    source["raised_tray"] = {
        "height_m": height, "width_m": .11, "depth_m": .11,
        "mass_assumption_kg": .075, "manufacturing_validated": False,
        "above_joint_deg": stages["TRAY_ABOVE"]["joint_deg"],
        "contact_joint_deg": stages["TRAY_CONTACT"]["joint_deg"],
        "candidate_label": label, "report_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
    }
    transfer = build_raised_transfer(target, source)
    if not transfer["executable"]:
        raise ValueError(f"raised transfer clearance failed: {transfer['path_audit']}")
    (output/"plan.json").write_text(json.dumps(source, indent=2)+"\n")
    (output/"transfer_plan.json").write_text(json.dumps(transfer, indent=2)+"\n")
    shutil.copy2(source_dir/"scene.usda", output/"scene.usda")
    return transfer


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source_dir, args.report, args.label, args.output_dir), indent=2))
