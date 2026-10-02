import copy
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import bottle_contact_model as bottle
from cup_contact_model import Chain, URDF_PATH, effective_config_sha256
from cup_contact_place import supported_window, touchdown_plan


def test_effective_config_fingerprint_is_order_independent():
    assert effective_config_sha256({"a": 1, "b": {"x": 2, "y": 3}}) == effective_config_sha256(
        {"b": {"y": 3, "x": 2}, "a": 1})


def test_effective_config_fingerprint_distinguishes_mass_override():
    original = bottle.load_config()
    changed = copy.deepcopy(original)
    changed["cup_mass_kg"] = .235
    assert effective_config_sha256(original) != effective_config_sha256(changed)


def test_effective_config_fingerprint_rejects_nonfinite():
    with pytest.raises(ValueError):
        effective_config_sha256({"mass": float("nan")})


def test_combined_materialized_model_preserves_left_contact(tmp_path):
    import cup_contact_model as cup
    left, _ = cup.prepare_model(tmp_path, cup.load_config())
    source_bytes = left.read_bytes()
    config = bottle.load_config(cup.ROOT / "config/simulation/bottle_replacement_experiment.json")
    model, provenance = bottle.prepare_model(tmp_path, config, source=left, source_is_materialized=True)
    assert left.read_bytes() == source_bytes
    assert ET.parse(model).find("./link[@name='left_contact_center']") is not None
    assert ET.parse(model).find("./link[@name='right_contact_center']") is not None
    assert provenance["source"]["sha256"] == hashlib.sha256(source_bytes).hexdigest()


def test_stock_collision_matches_visual_and_source_is_preserved(tmp_path):
    before = URDF_PATH.read_bytes()
    model, evidence = bottle.prepare_model(tmp_path, bottle.load_config())
    assert URDF_PATH.read_bytes() == before
    assert evidence["proxy_sha256"] == hashlib.sha256(model.read_bytes()).hexdigest()
    assert not any(evidence[key] for key in ("pads_added", "grooves_added", "tape_added"))
    source, root = ET.fromstring(before), ET.parse(model).getroot()
    for name in ("right_gripper_base_link", *bottle.FINGER_LINKS):
        link = root.find(f"./link[@name='{name}']")
        assert len(link.findall("collision")) == len(link.findall("visual")) == 2
        for visual, collision in zip(link.findall("visual"), link.findall("collision")):
            for tag in ("origin", "geometry/box"):
                assert visual.find(tag).attrib == collision.find(tag).attrib
        assert link.find("inertial/mass").attrib == source.find(f"./link[@name='{name}']/inertial/mass").attrib
    for name in bottle.GRIPPER_JOINTS:
        joint = root.find(f"./joint[@name='{name}']")
        assert joint.get("type") == "prismatic"
        assert joint.find("limit").attrib == source.find(f"./joint[@name='{name}']/limit").attrib
    np.testing.assert_allclose(Chain(model).transforms({})["right_tool0"], Chain(URDF_PATH).transforms({})["right_tool0"])


@pytest.mark.parametrize("key,value", [
    ("bottle_mass_kg", 0), ("bottle_mass_kg", float("nan")),
    ("bottle_center_m", [.39, -.17, .5]), ("bottle_radius_m", .05),
    ("gripper_open_m", .1), ("gripper_close_m", .001),
    ("gripper_unit", "rad"), ("gripper_kp", -1), ("friction", -1),
    ("contact_center_tool_m", [1, 2]),
    ("mesh_collision_mode", "none"),
])
def test_invalid_config_rejected(tmp_path, key, value):
    config = json.loads(bottle.DEFAULT_CONFIG.read_text())
    config[key] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        bottle.load_config(path)


def test_right_plan_uses_meters_and_touchdown_preserves_units(tmp_path):
    config = bottle.load_config()
    model, _ = bottle.prepare_model(tmp_path, config)
    plan = bottle.make_plan(model, config, place=True)
    assert plan["active_side"] == "right"
    assert plan["tcp_frame"] == "right_contact_center"
    assert plan["gripper_unit"] == "m"
    for pose in plan["poses"]:
        assert "gripper_rad" not in pose
        assert config["gripper_close_m"] <= pose["gripper_m"] <= config["gripper_open_m"]
        if "measurement" in pose:
            assert pose["measurement"]["position_error_mm"] < 2
            assert pose["measurement"]["closing_horizontal_error_deg"] < 2
    assert bottle.sample_plan(plan, 1000)["gripper_m"] == config["gripper_open_m"]
    assert bottle.sample_plan(plan, 0)["phase"] == "RESET"
    touchdown = touchdown_plan(plan, np.radians(plan["poses"][-1]["joint_deg"]))
    assert bottle.sample_plan(touchdown, 0)["gripper_m"] == config["gripper_close_m"]
    assert bottle.sample_plan(touchdown, 1000)["done"]


def test_bottle_release_window_checks_meters_not_radians():
    config = bottle.load_config()
    sample = {"phase": "PLACE_HOLD", "cup_position_m": config["cup_center_m"],
              "table_support_force_n": config["cup_mass_kg"]*9.81,
              "cup_tilt_deg": 0., "contact_force_n": [0., 0.], "arm_error_rad": 0.,
              "contact_center_error_m": .1, "gripper_actual_m": config["gripper_open_m"]}
    samples = [copy.deepcopy(sample) for _ in range(120)]
    assert supported_window(samples, config, "PLACE_HOLD", 1., released=True, clear=True)
    samples[-1]["gripper_actual_m"] -= .003
    assert not supported_window(samples, config, "PLACE_HOLD", 1., released=True, clear=True)
    samples[-1]["gripper_actual_m"] = float("nan")
    assert not supported_window(samples, config, "PLACE_HOLD", 1., released=True, clear=True)


def test_non_gripping_contact_is_not_grasp_success():
    config = bottle.load_config()
    assert bottle.non_gripping_contact_failure({"right_gripper_link": 0.}, config) is None
    assert bottle.non_gripping_contact_failure({"right_gripper_link": .02}, config) == "object_contact_with_non_gripping_robot_link"
    assert bottle.non_gripping_contact_failure({"right_gripper_link": float("nan")}, config) == "nonfinite_physics_state"


def test_collision_mode_is_explicit_and_never_disables_collision(tmp_path):
    assert bottle.load_config()["mesh_collision_mode"] == "convexHull"
    config = json.loads(bottle.DEFAULT_CONFIG.read_text())
    config["mesh_collision_mode"] = "convexDecomposition"
    path = tmp_path / "decomposed.json"
    path.write_text(json.dumps(config))
    assert bottle.load_config(path)["mesh_collision_mode"] == "convexDecomposition"


def test_replacement_is_opt_in_and_keeps_colliding_servo_and_inertia(tmp_path):
    config = bottle.load_config(bottle.DEFAULT_CONFIG.with_name("bottle_replacement_experiment.json"))
    original = URDF_PATH.read_bytes()
    model, evidence = bottle.prepare_model(tmp_path, config)
    assert URDF_PATH.read_bytes() == original
    assert evidence["assembly_mode"] == "replacement_hypothesis_proxy"
    assert evidence["status"] == "replacement_hypothesis_uncalibrated"
    assert evidence["proxy_file"] == model.name == "replacement_hypothesis.urdf"
    root = ET.parse(model).getroot()
    mount = root.find("./joint[@name='right_gripper_mount_joint']/origin")
    np.testing.assert_allclose(np.fromstring(mount.get("xyz"), sep=" "), [-.0079, -.000218, -.05527])
    np.testing.assert_allclose(np.fromstring(mount.get("rpy"), sep=" "), [np.pi/2, 0, 0])
    link = root.find("./link[@name='right_gripper_link']")
    assert len(link.findall("collision")) == len(link.findall("visual")) == 1
    assert link.find("collision/geometry/box") is not None
    assert link.find("collision/geometry/mesh") is None
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(.087)
    for origin in (link.find("inertial/origin"), link.find("collision/origin"), link.find("visual/origin")):
        np.testing.assert_allclose(np.fromstring(origin.get("xyz"), sep=" "), [-.0079, -.000218, -.03027])
    np.testing.assert_allclose(np.fromstring(link.find("collision/geometry/box").get("size"), sep=" "), [.03, .04, .045])
    inertia = link.find("inertial/inertia").attrib
    matrix = np.array([[float(inertia['ixx']), float(inertia['ixy']), float(inertia['ixz'])],
                       [float(inertia['ixy']), float(inertia['iyy']), float(inertia['iyz'])],
                       [float(inertia['ixz']), float(inertia['iyz']), float(inertia['izz'])]])
    assert np.min(np.linalg.eigvalsh(matrix)) > 0
    np.testing.assert_allclose(matrix, np.diag([.00002628125, .000018125, .00002120625]), atol=1e-14)
    np.testing.assert_allclose(Chain(model).transforms({})["left_tool0"], Chain(URDF_PATH).transforms({})["left_tool0"])
    plan = bottle.make_plan(model, config, place=True)
    assert plan["active_side"] == "right"
    assert not plan["model_calibrated"]
    assert not bottle.load_config().get("assembly_hypothesis")


@pytest.mark.parametrize("key,value", [("mode", "remove_all"), ("mount_xyz_m", [0, 0]),
                                      ("servo_size_base_m", [0, .04, .045]),
                                      ("servo_center_base_m", [float("nan"), 0, 0])])
def test_replacement_hypothesis_rejects_bad_values(tmp_path, key, value):
    config = json.loads(bottle.DEFAULT_CONFIG.with_name("bottle_replacement_experiment.json").read_text())
    config["assembly_hypothesis"][key] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        bottle.load_config(path)
