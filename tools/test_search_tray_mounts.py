import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from search_tray_mounts import (
    DEFAULT_PLAN, DEFAULT_URDF, evaluate_candidate, materialize_mount_candidate, search,
)


def test_materialize_mount_candidate_preserves_source_and_changes_copy(tmp_path):
    before = DEFAULT_URDF.read_bytes()
    target = tmp_path / "candidate.urdf"
    mount = materialize_mount_candidate(DEFAULT_URDF, target, 0.04, 15.0)
    assert DEFAULT_URDF.read_bytes() == before
    assert mount["candidate_mount_xyz_m"] == [pytest_approx(-0.02), pytest_approx(0.17), pytest_approx(0.6931)]
    root = ET.parse(target).getroot()
    for joint_name in ("left_mount_joint", "left_arm_backing_link_joint"):
        origin = root.find(f"./joint[@name='{joint_name}']/origin")
        assert np.fromstring(origin.get("rpy"), sep=" ")[2] == pytest_approx(np.radians(15.0))
        assert np.fromstring(origin.get("xyz"), sep=" ")[0] == pytest_approx(-0.02)
    assert mount["backing_inside_top_plate"]


def pytest_approx(value):
    return pytest.approx(value, abs=1e-8)


def test_candidate_report_is_fail_closed_and_finite(tmp_path):
    model = tmp_path / "candidate.urdf"
    mount = materialize_mount_candidate(DEFAULT_URDF, model, 0.0, 0.0)
    source = json.loads(DEFAULT_PLAN.read_text())
    result = evaluate_candidate(
        model, source, mount, tray_height_offset_m=0.04, restarts=2, iterations=80,
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


def test_small_search_writes_ranked_report_and_model_copies(tmp_path):
    report = search(DEFAULT_URDF, DEFAULT_PLAN, tmp_path, offsets_mm=[0, 20], yaws_deg=[0],
                    tray_heights_mm=[0], restarts=2, iterations=60)
    stored = json.loads((tmp_path / "report.json").read_text())
    assert stored["schema"] == "tray_mount_search_v1"
    assert len(report["candidates"]) == len(stored["candidates"]) == 2
    assert len(list((tmp_path / "models").glob("*.urdf"))) == 2
    assert not report["hardware_accessed"] and not report["gpu_used"]
    assert report["accepted_count"] == sum(item["accepted"] for item in report["candidates"])
