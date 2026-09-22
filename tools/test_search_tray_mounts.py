import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import bimanual_pour_plan as pour
import bottle_contact_model as bottle
import cup_contact_model as cup
from search_tray_mounts import (
    evaluate_candidate, materialize_mount_candidate, search,
)


REPO_URDF = cup.ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"


@pytest.fixture(scope="module")
def source_evidence(tmp_path_factory):
    output = tmp_path_factory.mktemp("tray_mount_source")
    right = bottle.load_config(cup.ROOT / "config/simulation/bottle_replacement_experiment.json")
    model, _ = bottle.prepare_model(output, right)
    left = cup.load_config()
    source = {
        "left_config": left,
        "right_config": right,
        "plan": pour.build_plan(model, left, right),
    }
    plan = output / "plan.json"
    plan.write_text(json.dumps(source, ensure_ascii=False, indent=2) + "\n")
    return model, plan, source


def test_materialize_mount_candidate_preserves_source_and_changes_copy(tmp_path):
    before = REPO_URDF.read_bytes()
    target = tmp_path / "candidate.urdf"
    mount = materialize_mount_candidate(REPO_URDF, target, 0.04, 15.0)
    assert REPO_URDF.read_bytes() == before
    assert mount["candidate_mount_xyz_m"] == [pytest_approx(-0.02), pytest_approx(0.17), pytest_approx(0.6931)]
    root = ET.parse(target).getroot()
    for joint_name in ("left_mount_joint", "left_arm_backing_link_joint"):
        origin = root.find(f"./joint[@name='{joint_name}']/origin")
        assert np.fromstring(origin.get("rpy"), sep=" ")[2] == pytest_approx(np.radians(15.0))
        assert np.fromstring(origin.get("xyz"), sep=" ")[0] == pytest_approx(-0.02)
    assert mount["backing_inside_top_plate"]


def pytest_approx(value):
    return pytest.approx(value, abs=1e-8)


def test_candidate_report_is_fail_closed_and_finite(tmp_path, source_evidence):
    source_urdf, _, source = source_evidence
    model = tmp_path / "candidate.urdf"
    mount = materialize_mount_candidate(source_urdf, model, 0.0, 0.0)
    result = evaluate_candidate(
        model, source, mount, source_urdf=source_urdf,
        tray_height_offset_m=0.04, restarts=2, iterations=80,
    )
    assert len(result["stages"]) == 4
    assert result["collision_scope"].startswith("centerline_auxiliary")
    assert np.isfinite([
        result["maximum_path_tilt_deg"], result["maximum_position_error_mm"],
        result["minimum_joint_margin_deg"], result["minimum_left_right_centerline_clearance_m"],
    ]).all()
    assert all(len(stage["joint_deg"]) == 5 for stage in result["stages"])
    assert result["stages"][2]["target_m"][2] == pytest.approx(0.88)
    assert result["stages"][3]["target_m"][2] == pytest.approx(0.82)


def test_small_search_writes_ranked_report_and_model_copies(tmp_path, source_evidence):
    source_urdf, source_plan, _ = source_evidence
    report = search(source_urdf, source_plan, tmp_path, offsets_mm=[0, 20], yaws_deg=[0],
                    tray_heights_mm=[0], restarts=2, iterations=60)
    stored = json.loads((tmp_path / "report.json").read_text())
    assert stored["schema"] == "tray_mount_search_v1"
    assert len(report["candidates"]) == len(stored["candidates"]) == 2
    assert len(list((tmp_path / "models").glob("*.urdf"))) == 2
    assert not report["hardware_accessed"] and not report["gpu_used"]
    assert report["accepted_count"] == sum(item["accepted"] for item in report["candidates"])
