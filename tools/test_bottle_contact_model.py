import copy
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import bottle_contact_model as bottle
from cup_contact_model import Chain, URDF_PATH
from cup_contact_place import supported_window, touchdown_plan


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
