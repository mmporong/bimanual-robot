import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from build_plate_service_candidate import build
from raised_tray_transfer import build_plate_transfer, _solve_unloaded_stage
from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions


def test_plate_contract_rejects_out_of_bounds_and_mixed_supports():
    source = {"plate_transfer": {"target_xy_m": [.15, 0.]},
              "experiment": {"cup_radius_profile_m": [[0., .045]]}}
    with pytest.raises(ValueError, match="outside plate"):
        build_plate_transfer("unused", source)
    source["plate_transfer"]["target_xy_m"] = [0., float("nan")]
    with pytest.raises(ValueError, match="outside plate"):
        build_plate_transfer("unused", source)
    source["raised_tray"] = {}
    with pytest.raises(ValueError, match="mutually exclusive"):
        build_plate_transfer("unused", source)


def test_plate_materializer_rejects_missing_or_changed_provenance_before_writing(tmp_path):
    report_path = tmp_path/"search.json"
    output = tmp_path/"candidate"
    report = {"source_dir": str(tmp_path), "source_sha256": {}}
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="both source hashes"):
        build(report_path, "unused", output)
    (tmp_path/"plan.json").write_text("{}")
    report["source_sha256"] = {"plan.json": "wrong", "replacement_hypothesis.urdf": "wrong"}
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="source changed"):
        build(report_path, "unused", output)
    assert not output.exists()


def test_plate_candidate_has_no_stand_and_uses_upright_loaded_path():
    directory = Path("/data/lim/robot-artifacts/restaurant/plate_input09")
    if not (directory/"plan.json").is_file():
        pytest.skip("local plate candidate unavailable")
    model = directory/"replacement_hypothesis.urdf"
    source = json.loads((directory/"plan.json").read_text())
    before = copy.deepcopy(source)
    assert not any(e.get("name", "").startswith("central_tray_") for e in ET.parse(model).getroot())
    transfer = build_plate_transfer(model, source)
    assert source == before
    assert transfer["executable"] and not transfer["physics_validated"]
    assert transfer["tray_surface_z_m"] == .72
    assert transfer["tray_target_xy_m"] == [.12, -.09]
    assert transfer["path_audit"]["maximum_cup_tilt_deg"] < 2.
    chain = BimanualContactChain(model, source["left_config"], source["right_config"])
    poses = {p["name"]: p for p in transfer["deposit_plan"]["poses"]}
    q = np.radians(poses["TRAY_WITHDRAW"]["left_joint_deg"])
    tcp = chain.transforms(base_positions("left", q))["left_tool0"][:3, 3]
    assert np.linalg.norm(tcp-transfer["tray_center_m"]) == pytest.approx(.05, abs=.002)
    deposit = transfer["deposit_plan"]["poses"]
    assert [p["name"] for p in deposit[:9]] == ["TRAY_START", *(["TRAY_RIGHT_CLEAR_LIFT"]*6), "TRAY_RIGHT_CLEAR", "TRAY_MOVE_ABOVE"]
    origin = chain.transforms(base_positions("right", np.radians(deposit[0]["right_joint_deg"])))["right_tool0"][:3, 3]
    for index in range(1, 7):
        assert deposit[index]["left_joint_deg"] == deposit[0]["left_joint_deg"]
        for fraction in np.linspace(0., 1., 21):
            angles = np.radians(np.asarray(deposit[index-1]["right_joint_deg"])*(1-fraction)
                                +np.asarray(deposit[index]["right_joint_deg"])*fraction)
            tcp = chain.transforms(base_positions("right", angles))["right_tool0"][:3, 3]
            assert np.linalg.norm(tcp[:2]-origin[:2]) < .002
            assert tcp[2] >= origin[2]-.0001
    assert tcp[2]-origin[2] == pytest.approx(.12, abs=.002)
    assert deposit[0]["left_joint_deg"] == deposit[1]["left_joint_deg"]
    assert deposit[0]["right_joint_deg"] != deposit[1]["right_joint_deg"]
    assert transfer["empty_hand_lift_m"] == .12
    assert len([p for p in deposit if p["name"] == "TRAY_CLEAR_ABOVE"]) == 6
    assert len([p for p in deposit if p["name"] == "TRAY_LOWER"]) == 6
    regrasp = transfer["regrasp_plan"]["poses"]
    assert len([p for p in regrasp if p["name"] == "TRAY_PREGRASP"]) == 6
    ascent = [p["left_joint_deg"] for p in deposit if p["name"] == "TRAY_CLEAR_ABOVE"]
    descent = [p["left_joint_deg"] for p in regrasp if p["name"] == "TRAY_PREGRASP"]
    assert descent == [*reversed(ascent[:-1]), poses["TRAY_WITHDRAW"]["left_joint_deg"]]
    for plan, phase in ((deposit, "TRAY_LOWER"), (deposit, "TRAY_CLEAR_ABOVE"),
                        (regrasp, "TRAY_PREGRASP"), (regrasp, "TRAY_RELIFT")):
        indices = [i for i, p in enumerate(plan) if p["name"] == phase]
        assert indices == list(range(indices[0], indices[0]+6))
        first_q = np.radians(plan[indices[0]-1]["left_joint_deg"])
        target_xy = chain.transforms(base_positions("left", first_q))["left_tool0"][:2, 3]
        for i in indices:
            previous_q = np.asarray(plan[i-1]["left_joint_deg"])
            next_q = np.asarray(plan[i]["left_joint_deg"])
            for fraction in np.linspace(0., 1., 21):
                angles = np.radians(previous_q*(1-fraction)+next_q*fraction)
                tcp = chain.transforms(base_positions("left", angles))["left_tool0"][:3, 3]
                assert np.linalg.norm(tcp[:2]-target_xy) < .002
    assert transfer["path_audit"]["scope"].endswith("not_collision_certification")
    assert not transfer["continuous_collision_validated"]
    lift_poses = [p for p in transfer["regrasp_plan"]["poses"] if p["name"] == "TRAY_RELIFT"]
    assert len(lift_poses) == 6
    previous = np.asarray(source["plate_transfer"]["contact_joint_deg"])
    for pose in lift_poses:
        following = np.asarray(pose["left_joint_deg"])
        for fraction in np.linspace(0., 1., 21):
            angles = np.radians(previous*(1-fraction)+following*fraction)
            tcp = chain.transforms(base_positions("left", angles))["left_tool0"][:3, 3]
            assert np.linalg.norm(tcp[:2]-transfer["tray_target_xy_m"]) < .002
        previous = following
    broken = copy.deepcopy(source)
    broken["plate_transfer"]["right_park_joint_deg"] = [999.]*5
    with pytest.raises(ValueError, match="right clearance"):
        build_plate_transfer(model, broken)


def test_unloaded_solver_position_objective_without_robot_assets(monkeypatch):
    import raised_tray_transfer as module

    class CartesianTestChain:
        def transforms(self, q):
            transform = np.eye(4)
            transform[:3, 3] = q[:3]
            return {"left_tool0": transform}

    monkeypatch.setattr(module, "_limits", lambda chain, side: (np.full(5, -2.), np.full(5, 2.)))
    monkeypatch.setattr(module, "base_positions", lambda side, angles: angles)
    seed = np.zeros(5)
    target = np.array([.12, -.09, .78])
    solved = _solve_unloaded_stage(CartesianTestChain(), target, seed)
    assert solved["accepted"]
    assert np.linalg.norm(solved["q"][:3]-target) < .002
    assert np.array_equal(seed, np.zeros(5))
