import numpy as np
import pytest
from pour_geometry import contained_mask, initial_liquid, liquid_counts, evaluate_pour


@pytest.mark.parametrize("key,value", [
    ("schema", "unknown"), ("wall_m", 0), ("particle_spacing_m", float("nan")),
    ("fluid_density_kg_m3", True), ("cup_work_center_m", [0, 1]),
    ("bottle_mouth_target_m", [0, 0, float("inf")]),
    ("additional_outward_mount_m", .03), ("pour_tilt_deg", 80),
    ("additional_outward_mount_m", True), ("pour_tilt_deg", "100"),
    ("pour_tilt_deg", 102),
    ("pour_azimuth_deg", float("nan")), ("pour_azimuth_deg", True),
    ("pour_azimuth_deg", "320"), ("pour_azimuth_deg", 360),
    ("pour_azimuth_deg", 320), ("bottle_approach_right_offset_m", 0),
    ("right_pick_ik_seed_joint_deg", [0, 1]),
    ("right_pick_ik_seed_joint_deg", [0, 0, 0, 0, float("nan")]),
    ("cup_radius_profile_m", [[.06,.045],[-.06,.028]]),
    ("cup_radius_profile_m", [[-.06,.002],[.06,.045]]),
    ("mouth_height_schedule", [[0,1.1],[100,.99]]),
    ("bottle_neck_outer_radius_m", .005), ("initial_fill_height_m", .2),
])
def test_experiment_rejects_invalid_geometry(tmp_path, key, value):
    import json
    from cup_contact_model import ROOT
    from pour_geometry import load_experiment
    config = load_experiment(ROOT / "config/simulation/bimanual_pour_experiment.json")
    config[key] = value
    target = tmp_path / "experiment.json"
    target.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_experiment(target)


def test_jaw_gap_ignores_collision_order_and_rejects_changed_axis(tmp_path):
    import xml.etree.ElementTree as ET
    import bottle_contact_model as bottle
    from pour_geometry import minimum_jaw_gap
    model, _ = bottle.prepare_model(tmp_path, bottle.load_config())
    assert minimum_jaw_gap(model) == pytest.approx(.02)
    tree = ET.parse(model)
    for index in (1, 2):
        link = tree.find(f"./link[@name='right_finger{index}_link']")
        collisions = link.findall("collision")
        for item in collisions:
            link.remove(item)
        link.extend(reversed(collisions))
    tree.write(model)
    assert minimum_jaw_gap(model) == pytest.approx(.02)
    tree.find("./joint[@name='right_finger1_joint']/axis").set("xyz", "0 1 0")
    tree.write(model)
    with pytest.raises(ValueError):
        minimum_jaw_gap(model)


@pytest.mark.parametrize("attribute,value", [("lower", "nan"), ("lower", "0.001"), ("upper", "0")])
def test_jaw_gap_rejects_invalid_or_insufficient_limits(tmp_path, attribute, value):
    import xml.etree.ElementTree as ET
    import bottle_contact_model as bottle
    from pour_geometry import minimum_jaw_gap
    model, _ = bottle.prepare_model(tmp_path, bottle.load_config())
    tree = ET.parse(model)
    for index in (1,2):
        tree.find(f"./joint[@name='right_finger{index}_joint']/limit").set(attribute,value)
    tree.write(model)
    with pytest.raises(ValueError):
        minimum_jaw_gap(model)


@pytest.mark.parametrize("change", ["missing", "duplicate", "rotated"])
def test_jaw_gap_requires_exactly_one_unrotated_blade(tmp_path, change):
    import copy
    import xml.etree.ElementTree as ET
    import bottle_contact_model as bottle
    from pour_geometry import minimum_jaw_gap
    model, _ = bottle.prepare_model(tmp_path,bottle.load_config())
    tree = ET.parse(model)
    link = tree.find("./link[@name='right_finger1_link']")
    blade = next(c for c in link.findall("collision")
                 if np.allclose(np.fromstring(c.find("geometry/box").get("size"),sep=" "),[.012,.060,.050]))
    if change == "missing":
        link.remove(blade)
    elif change == "duplicate":
        link.append(copy.deepcopy(blade))
    else:
        blade.find("origin").set("rpy","0 0 .1")
    tree.write(model)
    with pytest.raises(ValueError):
        minimum_jaw_gap(model)


def test_bottle_neck_classification_uses_profile():
    from cup_contact_model import ROOT
    from pour_geometry import load_experiment
    profile = load_experiment(ROOT / "config/simulation/bimanual_pour_experiment.json")
    points = [[.02, 0, 0], [.02, 0, .09], [.005, 0, .09]]
    assert contained_mask(points, [0,0,0], [1,0,0,0], .03, .2,
                          bottle_profile=profile).tolist() == [True, False, True]


def test_mount_audit_records_mesh_hash_and_additional_clearance(tmp_path):
    import bottle_contact_model as bottle
    from cup_contact_model import ROOT
    from pour_geometry import load_experiment, mount_clearance_evidence
    config = bottle.load_config(ROOT / "config/simulation/bottle_replacement_experiment.json")
    config["assembly_hypothesis"]["mount_xyz_m"][2] -= load_experiment(
        ROOT / "config/simulation/bimanual_pour_experiment.json")["additional_outward_mount_m"]
    model, _ = bottle.prepare_model(tmp_path, config)
    evidence = mount_clearance_evidence(model)
    assert evidence["minimum_sampled_gap_m"] == pytest.approx(.00304155, abs=1e-7)
    assert evidence["sample_count"] >= 320
    assert len(evidence["meshes"]) == 2
    assert all(len(row["sha256"]) == 64 for row in evidence["meshes"])
    assert not evidence["physical_calibration"]


def test_fluid_initialization_inside_bottle():
    points = initial_liquid(np.array([.39, -.17, .82]), .03, .2)
    assert len(points) > 500
    assert contained_mask(points, [.39, -.17, .82], [1, 0, 0, 0], .03, .2).all()


def test_flared_cup_contains_wide_rim_but_not_below_narrow_bottom():
    from cup_contact_model import ROOT
    from pour_geometry import load_experiment
    profile = load_experiment(ROOT / "config/simulation/bimanual_pour_experiment.json")["cup_radius_profile_m"]
    points = [[.039,0,.055],[.039,0,0],[.030,0,-.055],[0,0,.061]]
    assert contained_mask(points,[0,0,0],[1,0,0,0],.035,.12,cup_profile=profile).tolist() == [True,False,False,False]
    assert contained_mask([[0,-.055,.039]],[0,0,0],[2**-.5,2**-.5,0,0],.035,.12,cup_profile=profile)[0]


def test_rotated_container_and_outside_classification():
    points = np.array([[0, 0, 0], [.04, 0, 0], [0, 0, .2]])
    config = {"cup_radius_m": .03, "cup_height_m": .2}
    counts = liquid_counts(points, ([0,0,0], [1,0,0,0]), ([0,0,.2], [1,0,0,0]), config, config)
    assert counts == {"cup": 1, "bottle": 1, "outside": 1, "overlap":0, "total": 3}
    assert contained_mask([[.08,0,0]], [0,0,0], [2**-.5,0,2**-.5,0], .03,.2)[0]


def test_empty_or_nonfinite_does_not_pass():
    assert not evaluate_pour([], 0, True)["task_pass"]
    with pytest.raises(ValueError):
        contained_mask([[0,0,0]], [0,0,0], [0,0,0,0], .03,.2)


@pytest.mark.parametrize("side,kind",[("LEFT","cup"),("RIGHT","bottle")])
def test_preclose_contact_and_movement_rejected(side,kind):
    from pour_geometry import preclose_violation
    initial={kind+"_position_m":[.39,0,.8]}
    sample={"phase":side+"_ALIGN_MIDDLE",kind+"_hand_n":[0,0],**initial}
    assert preclose_violation(sample,initial) is None
    assert preclose_violation({**sample,kind+"_hand_n":[.03,0]},initial) == "premature_hand_contact:"+kind
    assert preclose_violation({**sample,kind+"_position_m":[.395,0,.8]},initial) == "container_displaced_before_close:"+kind
    assert preclose_violation({**sample,"phase":side+"_CLOSE",kind+"_hand_n":[1,1]},initial) is None


def test_success_requires_liquid_release_and_support():
    s = {"phase": "FINAL_HOLD", "liquid": {"cup": 60, "bottle": 39, "outside": 1, "overlap":0, "total": 100},
         "cup_support_n": .1, "bottle_support_n": .3, "cup_hand_n": [0,0], "bottle_hand_n": [0,0],
         "cup_rise_m": 0., "bottle_rise_m": 0., "cup_tilt_deg": 0., "bottle_tilt_deg": 0.,
         "left_gripper_actual_rad":1.,"right_gripper_actual_m":[.0433,.0433],
         "cup_position_m":[.39,.17,.78],"bottle_position_m":[.39,-.17,.82],
         "mouth_relative_to_cup_rim_m":[0,0,.04],"cup_grasp_slip_m":0.,"bottle_grasp_slip_m":0.,
         "cup_grasp_rotation_error_deg":0.,"bottle_grasp_rotation_error_deg":0.,
         "bottle_orientation_wxyz":[np.cos(np.radians(50)), -np.sin(np.radians(50)), 0, 0]}
    s.update(cup_holding_expected=False,bottle_holding_expected=False)
    start = {**s, "phase": "RESET", "liquid": {"cup":0,"bottle":100,"outside":0,"overlap":0,"total":100}}
    samples = [start]
    for kind,side in (("cup","LEFT"),("bottle","RIGHT")):
        samples += [{**start,"phase":side+"_LIFT_HOLD",kind+"_rise_m":.05,kind+"_hand_n":[1,1]}]*120
    samples += [{**start,"phase":"POUR_MOVE"}]*120
    samples += [{**start,"phase":"POUR_TILT","bottle_tilt_deg":100.,"cup_hand_n":[1,1],"bottle_hand_n":[1,1]}]
    samples += [{**s, "phase": "POUR_HOLD","bottle_tilt_deg":100.,"cup_hand_n":[1,1],"bottle_hand_n":[1,1]}]*120 + [s]*120
    samples = [{**sample,"time_s":index/120} for index,sample in enumerate(samples)]
    assert evaluate_pour(samples,100,True)["task_pass"]
    premature={**start,"phase":"LEFT_ALIGN_MIDDLE","cup_hand_n":[.03,0],"time_s":0}
    assert not evaluate_pour([premature]+samples,100,True)["task_pass"]
    reversed_direction = [{**x,"bottle_orientation_wxyz":[np.cos(np.radians(50)), np.sin(np.radians(50)), 0, 0]} for x in samples]
    assert not evaluate_pour(reversed_direction,100,True)["task_pass"]
    assert not evaluate_pour(reversed_direction,100,True)["inward_pour_direction"]
    assert not evaluate_pour(samples,100,False)["task_pass"]
    assert not evaluate_pour([{**x,"cup_hand_n":[1,0]} for x in samples],100,True)["task_pass"]
    assert not evaluate_pour([{**x,"liquid":{**x["liquid"],"cup":0}} for x in samples],100,True)["task_pass"]
    assert not evaluate_pour([{**x,"bottle_tilt_deg":0.} for x in samples],100,True)["task_pass"]
    assert not evaluate_pour([{**x,"left_gripper_actual_rad":0.} for x in samples],100,True)["task_pass"]
    early = [{**x,"liquid":s["liquid"]} if x["phase"] == "POUR_MOVE" else x for x in samples]
    assert not evaluate_pour(early,100,True)["task_pass"]
    lost = [{**x,"cup_holding_expected":True,"cup_hand_n":[0,0]} if x["phase"] == "POUR_MOVE" else x for x in samples]
    assert not evaluate_pour(lost,100,True)["task_pass"]
    wrong_place = [{**x,"cup_position_m":[.7,.7,.78]} if x["phase"] == "FINAL_HOLD" else x for x in samples]
    assert not evaluate_pour(wrong_place,100,True)["task_pass"]
    misplaced_transfer = [{**x,"liquid":s["liquid"],"bottle_tilt_deg":20.,
                           "mouth_relative_to_cup_rim_m":[.3,0,.3]} if x["phase"] == "POUR_TILT" else x for x in samples]
    assert not evaluate_pour(misplaced_transfer,100,True)["task_pass"]
