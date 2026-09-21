import copy
from pathlib import Path

import numpy as np
import pytest

import bimanual_pour_plan as pour
import bottle_contact_model as bottle
import cup_contact_model as cup


def _model(tmp_path):
    right = bottle.load_config(Path("config/simulation/bottle_replacement_experiment.json"))
    model, _ = bottle.prepare_model(tmp_path, right)
    return model, cup.load_config(), right


def test_complete_plan_is_bimanual_and_preserves_configs(tmp_path):
    model, left, right = _model(tmp_path)
    before_left, before_right = copy.deepcopy(left), copy.deepcopy(right)
    plan = pour.build_plan(model, left, right)
    assert left == before_left and right == before_right
    assert plan["schema"] == "bimanual_pour_plan_v1"
    names = [pose["name"] for pose in plan["poses"]]
    assert names.index("LEFT_CLOSE") < names.index("RIGHT_CLOSE") < names.index("POUR_HOLD")
    assert names.index("POUR_HOLD") < names.index("RIGHT_LOWER") < names.index("LEFT_LOWER")
    assert names[-1] == "FINAL_HOLD"
    assert plan["task_geometry"]["maximum_bottle_tilt_deg"] == pour.POUR_TILT_DEG
    assert plan["task_geometry"]["cup_rim_to_bottle_mouth_clearance_m"] == pytest.approx(.04)
    for pose in plan["poses"]:
        assert np.asarray(pose["left_joint_deg"]).shape == (5,)
        assert np.asarray(pose["right_joint_deg"]).shape == (5,)
        assert np.isfinite([*pose["left_joint_deg"], *pose["right_joint_deg"],
                            pose["left_gripper_rad"], pose["right_gripper_m"]]).all()
    assert not plan["hardware_accessed"]
    assert not plan["object_attachment_used"]
    assert not plan["liquid_transfer_validated"]
    for previous, pose in zip(plan["poses"], plan["poses"][1:]):
        if pose["name"].startswith("POUR_TILT") or pose["name"] == "POUR_RETURN":
            peak_speed = 1.5*np.max(np.abs(np.radians(
                np.asarray(pose["right_joint_deg"])-previous["right_joint_deg"])))/pose["duration_s"]
            assert peak_speed <= pour.EXPERIMENT["pour_joint_speed_rad_s"]+1e-10
    lifted = next(p for p in plan["poses"] if p["name"] == "LEFT_LIFT_HOLD")
    work = next(p for p in plan["poses"] if p["name"] == "POUR_MOVE_CUP")
    chain = pour.BimanualContactChain(model, left, right)
    seed = np.radians(lifted["left_joint_deg"])
    axis = chain.transforms(pour.base_positions("left", seed))["left_tool0"][:3,:3].T @ [0.,0.,1.]
    for t in np.linspace(0,1,21):
        q = seed*(1-t)+np.radians(work["left_joint_deg"])*t
        assert (chain.transforms(pour.base_positions("left",q))["left_tool0"][:3,:3] @ axis)[2] > .999


def test_pour_endpoint_tracks_mouth_and_axis(tmp_path):
    model, left, right = _model(tmp_path)
    plan = pour.build_plan(model, left, right)
    final = next(p for p in plan["poses"] if p["name"] == "POUR_TILT")
    assert final["bottle_tilt_deg"] == pour.POUR_TILT_DEG
    assert final["measurement"]["position_error_mm"] <= pour.POUR_POSITION_LIMIT_MM
    assert final["measurement"]["tilt_error_deg"] <= pour.POUR_AXIS_LIMIT_DEG
    desired = np.asarray(final["measurement"]["desired_bottle_axis"])
    assert np.degrees(np.arccos(np.clip(desired @ np.array([0., 0., 1.]), -1., 1.))) == pytest.approx(pour.POUR_TILT_DEG)


def test_sample_plan_interpolates_both_arms_and_rejects_bad_time(tmp_path):
    model, left, right = _model(tmp_path)
    plan = pour.build_plan(model, left, right)
    start = pour.sample_plan(plan, 0.)
    end = pour.sample_plan(plan, 1e6)
    assert start["phase"] == plan["poses"][0]["name"] and not start["done"]
    assert end["phase"] == "FINAL_HOLD" and end["done"]
    with pytest.raises(ValueError):
        pour.sample_plan(plan, -1.)


def test_plan_only_manifest_resolves_source_hashes_without_isaac(tmp_path):
    import hashlib
    import json
    from types import SimpleNamespace
    from simulate_bimanual_pour import run
    output = tmp_path / "plan_only"
    run(SimpleNamespace(output_dir=output, settle_only=False, plan_only=True, headless=True, record=False))
    manifest = json.loads((output / "plan.json").read_text())
    for filename, digest in manifest["tool_sha256"].items():
        assert digest == hashlib.sha256((cup.ROOT / "tools" / filename).read_bytes()).hexdigest()
    assert manifest["ik_source_sha256"] == hashlib.sha256(
        (cup.ROOT / "src/hold_flow_description/scripts/solve_task_poses.py").read_bytes()).hexdigest()
    assert not manifest["hardware_accessed"]
    assert manifest["fluid"]["initial_count"] == 918
