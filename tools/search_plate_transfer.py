"""Search upright endpoints on the existing deck without changing arm mounts."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions
from search_tray_mounts import _solve_stage
from tray_transfer_plan import _named_pose


def search(source_dir, output, x_values_mm=(-120, -60, 0, 60, 120),
           y_values_mm=(-120, -100, -80, -60, -40, 0, 40, 80, 120)):
    source = json.loads((source_dir/"plan.json").read_text())
    if "raised_tray" in source:
        raise ValueError("use a source without the raised support")
    model = source_dir/"replacement_hypothesis.urdf"
    chain = BimanualContactChain(model, source["left_config"], source["right_config"])
    held = _named_pose(source["plan"]["poses"], "POUR_RETURN_CUP")
    parked = _named_pose(source["plan"]["poses"], "RIGHT_PLACE_HOLD")
    start = np.radians(held["left_joint_deg"])
    axis = chain.transforms(base_positions("left", start))["left_tool0"][:3, :3].T @ np.array([0., 0., 1.])
    candidates, cases = [], []
    radius = max(r for _, r in source["experiment"]["cup_radius_profile_m"])
    for x_mm in x_values_mm:
        for y_mm in y_values_mm:
            if abs(x_mm)/1000+radius > .170 or abs(y_mm)/1000+radius > .225:
                raise ValueError("search target outside plate")
            target = np.array([x_mm/1000, y_mm/1000, .78])
            result = _solve_stage(chain, target, start, axis, center_region=False,
                                  restarts=2, iterations=120, random_seed=4000+len(candidates))
            record = {"id": f"x{x_mm:+04d}_y{y_mm:+04d}", "target_m": target.tolist(),
                      "accepted": result["accepted"], "joint_deg": np.degrees(result["q"]).tolist(),
                      "position_error_mm": result["position_error_mm"],
                      "tilt_deg": result["endpoint_tilt_deg"], "path_tilt_deg": result["path_maximum_tilt_deg"]}
            candidates.append(record)
            if result["accepted"]:
                cases.append({"id": record["id"], "left_joint_deg": record["joint_deg"],
                              "right_joint_deg": parked["right_joint_deg"]})
            print(json.dumps(record), flush=True)
    output.mkdir(parents=True, exist_ok=False)
    report = {"source_dir": str(source_dir), "source_sha256": {
        n: hashlib.sha256((source_dir/n).read_bytes()).hexdigest()
        for n in ("plan.json", "replacement_hypothesis.urdf")},
        "plate_surface_z_m": .72, "collision_validated": False, "candidates": candidates}
    (output/"search.json").write_text(json.dumps(report, indent=2)+"\n")
    (output/"probes.json").write_text(json.dumps({"cases": cases}, indent=2)+"\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--x-mm", type=int, nargs="+")
    parser.add_argument("--y-mm", type=int, nargs="+")
    args = parser.parse_args()
    options = {}
    if args.x_mm is not None:
        options["x_values_mm"] = args.x_mm
    if args.y_mm is not None:
        options["y_values_mm"] = args.y_mm
    search(args.source_dir, args.output_dir, **options)
