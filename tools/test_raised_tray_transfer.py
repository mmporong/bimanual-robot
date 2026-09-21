import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from build_raised_service_candidate import add_box, build
from raised_tray_transfer import build_raised_transfer


def test_stand_box_has_mass_inertia_collision_and_fixed_base():
    root = ET.Element("robot")
    add_box(root, "stand", (0, 0, .877), (.11, .11, .006), .05)
    link = root.find("link")
    assert float(link.find("inertial/mass").get("value")) == .05
    inertia = link.find("inertial/inertia")
    assert all(float(inertia.get(axis)) > 0 for axis in ("ixx", "iyy", "izz"))
    assert link.find("collision/geometry/box").get("size") == "0.11 0.11 0.006"
    assert root.find("joint/parent").get("link") == "base_footprint"


def test_builder_rejects_mismatched_search_source_before_creating_output(tmp_path):
    (tmp_path/"plan.json").write_text("{}")
    report = tmp_path/"report.json"
    report.write_text(json.dumps({"source_plan_sha256": "incorrect"}))
    output = tmp_path/"output"
    with pytest.raises(ValueError, match="source hash mismatch"):
        build(tmp_path, report, "unused", output)
    assert not output.exists()


def test_raised_candidate_audits_path_and_rejects_wrong_endpoint():
    directory = Path("/data/lim/robot-artifacts/restaurant/raised_service_input03")
    if not (directory/"plan.json").exists():
        pytest.skip("local simulation candidate not present")
    source = json.loads((directory/"plan.json").read_text())
    model = directory/"replacement_hypothesis.urdf"
    transfer = build_raised_transfer(model, source)
    assert transfer["executable"]
    assert not transfer["physics_validated"]
    assert transfer["tray_surface_z_m"] == pytest.approx(.92)
    assert transfer["rim_backoff_m"] == .075
    assert np.linalg.norm(transfer["tray_center_m"][:2]) < .02
    assert transfer["path_audit"]["maximum_cup_tilt_deg"] < 2.
    assert transfer["guarded_lowering_offset_m"] == .002
    assert transfer["path_audit"]["minimum_support_clearance_m"] >= -.00201
    broken = copy.deepcopy(source)
    broken["raised_tray"]["contact_joint_deg"] = [0.]*5
    with pytest.raises(ValueError, match="endpoint"):
        build_raised_transfer(model, broken)
    broken = copy.deepcopy(source)
    broken["raised_tray"]["height_m"] = float("nan")
    with pytest.raises(ValueError, match="height"):
        build_raised_transfer(model, broken)
