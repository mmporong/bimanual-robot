"""Materialize a deck candidate; keep the original model and arm mounts unchanged."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions
from raised_tray_transfer import build_plate_transfer
from search_tray_mounts import _solve_stage
from tray_transfer_plan import _named_pose


def build(report_path, candidate_id, output, right_park_joint_deg=None, backoff_m=.075):
    report = json.loads(report_path.read_text())
    source_dir = Path(report["source_dir"])
    if set(report["source_sha256"]) != {"plan.json", "replacement_hypothesis.urdf"}:
        raise ValueError("plate search requires both source hashes")
    for name, digest in report["source_sha256"].items():
        if hashlib.sha256((source_dir/name).read_bytes()).hexdigest() != digest:
            raise ValueError("plate search source changed")
    candidate = next(c for c in report["candidates"] if c["id"] == candidate_id)
    if not candidate["accepted"]:
        raise ValueError("endpoint search did not pass")
    source = json.loads((source_dir/"plan.json").read_text())
    model = source_dir/"replacement_hypothesis.urdf"
    if any(e.get("name", "").startswith("central_tray_") for e in ET.parse(model).getroot()):
        raise ValueError("source model still has a raised support")
    chain = BimanualContactChain(model, source["left_config"], source["right_config"])
    held = _named_pose(source["plan"]["poses"], "POUR_RETURN_CUP")
    start = np.radians(held["left_joint_deg"])
    axis = chain.transforms(base_positions("left", start))["left_tool0"][:3, :3].T @ np.array([0., 0., 1.])
    target = np.asarray(candidate["target_m"])
    lower = np.radians(candidate["joint_deg"])
    above = _solve_stage(chain, target+[0., 0., .075], lower, axis, center_region=False,
                         restarts=6, iterations=240, random_seed=4501)
    if not above["accepted"]:
        raise ValueError("plate pre-lower waypoint unreachable")
    source["plate_transfer"] = {"target_xy_m": target[:2].tolist(),
        "empty_hand_lift_m": .12,
        "empty_hand_backoff_m": backoff_m,
        "contact_joint_deg": candidate["joint_deg"], "above_joint_deg": np.degrees(above["q"]).tolist(),
        "candidate_id": candidate_id, "search_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
        "arm_mounts_changed": False, "raised_support_added": False}
    if right_park_joint_deg is not None:
        source["plate_transfer"]["right_park_joint_deg"] = right_park_joint_deg
    transfer = build_plate_transfer(model, source)
    if not transfer["executable"]:
        raise ValueError("plate carry path rejected: "+str(transfer["path_audit"]))
    output.mkdir(parents=True, exist_ok=False)
    for name in ("replacement_hypothesis.urdf", "scene.usda"):
        shutil.copy2(source_dir/name, output/name)
    (output/"plan.json").write_text(json.dumps(source, indent=2)+"\n")
    (output/"transfer_plan.json").write_text(json.dumps(transfer, indent=2)+"\n")
    (output/"probes.json").write_text(json.dumps({"cases": [
        {"id": f"{i:03d}_"+p["name"], "left_joint_deg": p["left_joint_deg"], "right_joint_deg": p["right_joint_deg"]}
        for i, p in enumerate(transfer["deposit_plan"]["poses"])]}, indent=2)+"\n")
    print(json.dumps({"output": str(output), "path_audit": transfer["path_audit"]}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--right-park-joint-deg", type=float, nargs=5)
    parser.add_argument("--backoff-mm", type=float, default=75.)
    args = parser.parse_args()
    build(args.report, args.candidate_id, args.output_dir, args.right_park_joint_deg, args.backoff_mm/1000.)
