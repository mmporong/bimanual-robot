import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import cup_contact_model

from cup_contact_model import (Chain, URDF_PATH, load_config, prepare_model, make_plan,
                               sample_plan, evaluate_lift, ready_to_lift, preclose_failure)


def stock_config():
    return load_config(cup_contact_model.ROOT / "config/simulation/cup_stock_so101_experiment.json")


def test_stock_source_geometry_inertia_and_joint_are_not_replaced(tmp_path):
    original = hashlib.sha256(URDF_PATH.read_bytes()).hexdigest()
    path, provenance = prepare_model(tmp_path, stock_config())
    source, generated = ET.parse(URDF_PATH).getroot(), ET.parse(path).getroot()
    for name in cup_contact_model.FINGER_LINKS:
        before = source.find(f"./link[@name='{name}']")
        after = generated.find(f"./link[@name='{name}']")
        assert before is not None and after is not None
        # Materialization only resolves package mesh paths; compare all other
        # authored geometry, origins, mass and inertia without whitespace.
        for mesh in after.findall(".//mesh"):
            mesh.set("filename", "package://hold_flow_description/meshes/so101/" + Path(mesh.get("filename")).name)
        assert [(e.tag, e.attrib) for e in before.iter()] == [(e.tag, e.attrib) for e in after.iter()]
    assert [(e.tag, e.attrib) for e in source.find("./joint[@name='left_gripper']").iter()] == [
        (e.tag, e.attrib) for e in generated.find("./joint[@name='left_gripper']").iter()]
    assert not any("assumed_" in e.get("name", "") for e in generated.iter())
    assert provenance["assembly_mode"] == "stock_so101_source_geometry"
    assert hashlib.sha256(URDF_PATH.read_bytes()).hexdigest() == original


@pytest.mark.parametrize("geometry", ["tpu", None, True, [], {}])
def test_unknown_geometry_cannot_silently_use_stock_or_pads(geometry):
    config = load_config()
    config["gripper_geometry"] = geometry
    with pytest.raises(ValueError, match="그리퍼 형상"):
        cup_contact_model.validate_config(config)


def test_stock_plan_is_horizontal_midbody_and_uses_same_revolute_contract(tmp_path):
    config = stock_config()
    model, _ = prepare_model(tmp_path, config)
    plan = make_plan(model, config, place=True)
    approach = next(p for p in plan["poses"] if p["name"] == "APPROACH")
    np.testing.assert_allclose(approach["target_m"], config["cup_center_m"])
    assert all("gripper_rad" in p for p in plan["poses"])
    assert max(p["measurement"]["position_error_mm"] for p in plan["poses"] if "measurement" in p) < 2
    assert max(p["measurement"]["closing_horizontal_error_deg"] for p in plan["poses"] if "measurement" in p) < 2


def test_stock_wrist_or_opposite_arm_cup_contact_is_not_a_grasp():
    config = stock_config()
    check = cup_contact_model.stock_non_gripping_contact_failure
    assert check({}, config) is None
    assert check({"left_wrist_link": .019}, config) is None
    for link in ("left_wrist_link", "right_finger1_link", "left_lower_arm_link"):
        assert check({link: .02}, config) == "cup_contact_with_non_gripping_robot_link"
    assert check({"left_wrist_link": 1.}, load_config()) is None


def test_bottle_backend_geometry_metadata_does_not_claim_left_pads():
    import bottle_contact_model
    config = bottle_contact_model.load_config()
    assert config["gripper_geometry"] == "stock_flat_jaw_proxy"
    config["gripper_geometry"] = "stock_so101"
    with pytest.raises(ValueError, match="rad 관절"):
        cup_contact_model.validate_config(config)
    config["gripper_geometry"] = "rigid_pad_proxy"
    with pytest.raises(ValueError, match="rad 관절"):
        cup_contact_model.validate_config(config)


def test_proxy_preserves_source_and_revolute_joint(tmp_path):
    config = load_config()
    original = hashlib.sha256(URDF_PATH.read_bytes()).hexdigest()
    path, provenance = prepare_model(tmp_path, config)
    assert hashlib.sha256(URDF_PATH.read_bytes()).hexdigest() == original
    root = ET.parse(path).getroot()
    joint = root.find("./joint[@name='left_gripper']")
    assert joint is not None
    assert joint.get("type") == "revolute"
    axis = joint.find("axis")
    assert axis is not None
    assert axis.get("xyz") == "0 0 1"
    assert root.find("./link[@name='left_contact_center']") is not None
    assert root.find("./link[@name='left_moving_jaw_link']/collision/geometry/box") is not None
    assert provenance["proxy_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    source_tf, proxy_tf = Chain(URDF_PATH).transforms({}), Chain(path).transforms({})
    np.testing.assert_allclose(source_tf["left_tool0"], proxy_tf["left_tool0"])


def test_missing_required_inertial_reports_clear_error(tmp_path, monkeypatch):
    def incomplete_source(source, target, **kwargs):
        tree = ET.parse(source)
        moving = tree.getroot().find("./link[@name='left_moving_jaw_link']")
        assert moving is not None
        inertial = moving.find("inertial")
        assert inertial is not None
        moving.remove(inertial)
        tree.write(target)
        return {}
    monkeypatch.setattr(cup_contact_model, "materialize_urdf", incomplete_source)
    with pytest.raises(ValueError, match="필수 URDF 요소 누락: inertial"):
        prepare_model(tmp_path, load_config())


@pytest.mark.parametrize("key,value", [
    ("cup_mass_kg", -1), ("physics_dt_s", 0), ("friction", float("nan")),
    ("pad_size_m", [0, .01, .08]), ("cup_center_m", [.3, .17, .5]),
    ("gripper_open_rad", 4), ("minimum_lift_m", .5),
])
def test_config_rejects_bad_values(tmp_path, key, value):
    config = load_config()
    config[key] = value
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_config(path)


@pytest.fixture(scope="module")
def planned(tmp_path_factory):
    config = load_config()
    model, _ = prepare_model(tmp_path_factory.mktemp("contact-model"), config)
    return make_plan(model, config), config


def test_plan_reaches_midbody_and_does_not_flip_wrist(planned):
    plan, config = planned
    poses = plan["poses"]
    approach = next(p for p in poses if p["name"] == "APPROACH")
    np.testing.assert_allclose(approach["target_m"], config["cup_center_m"])
    for a, b in zip(poses[1:], poses[2:]):
        assert abs(a["joint_deg"][4]-b["joint_deg"][4]) < 10
    for pose in poses:
        if "measurement" in pose:
            assert pose["measurement"]["position_error_mm"] < 2
            assert pose["measurement"]["closing_horizontal_error_deg"] < 2
    assert not plan["model_calibrated"]


def test_sampler_finishes_once_with_gripper_closed(planned):
    plan, config = planned
    assert sample_plan(plan, 0)["phase"] == "RESET"
    final = sample_plan(plan, 1000)
    assert final["done"]
    assert final["phase"] == "LIFT_HOLD"
    assert final["gripper_rad"] == config["gripper_close_rad"]
    with pytest.raises(ValueError):
        sample_plan(plan, float("nan"))


def successful_samples(config):
    return [{"phase": "LIFT_HOLD", "cup_position_m": [.39, .17, .84],
             "contact_force_n": [.5, .6], "arm_error_rad": .01, "midbody_height_error_m": .005,
             "contact_center_error_m": .005, "cup_tilt_deg": 2.}
            for _ in range(round(1/config["physics_dt_s"]))]


def test_success_requires_completed_sustained_bilateral_contact_and_height():
    config = load_config()
    samples = successful_samples(config)
    assert evaluate_lift(samples, config, True, "sequence_finished")["rigid_proxy_lift_pass"]
    assert not evaluate_lift(samples, config, False, "closed")["rigid_proxy_lift_pass"]
    assert not evaluate_lift(samples[:1], config, True, "sequence_finished")["rigid_proxy_lift_pass"]
    for key, value in (("contact_force_n", [0, 1]), ("cup_position_m", [.39, .17, .78]),
                       ("arm_error_rad", .5), ("contact_force_n", [float("nan"), 1]),
                       ("midbody_height_error_m", .05), ("contact_center_error_m", .05),
                       ("cup_tilt_deg", 40), ("cup_position_m", [.49, .17, .84])):
        invalid = copy.deepcopy(samples)
        invalid[-1][key] = value
        assert not evaluate_lift(invalid, config, True, "sequence_finished")["rigid_proxy_lift_pass"]


def test_lift_gate_rejects_stale_or_one_sided_contact():
    config = load_config()
    samples = successful_samples(config)
    for sample in samples:
        sample["phase"] = "CONTACT_HOLD"
    assert ready_to_lift(samples, config)
    assert not ready_to_lift(samples[:1], config)
    samples[-1]["contact_force_n"] = [0., 1.]
    assert not ready_to_lift(samples, config)


def test_preclose_touch_or_displacement_is_not_grasp_success():
    config = load_config()
    sample = {"phase": "APPROACH", "cup_displacement_from_start_m": 0., "contact_force_n": [0., 0.]}
    assert preclose_failure(sample, config) is None
    sample["contact_force_n"] = [config["minimum_contact_force_n"], 0.]
    assert preclose_failure(sample, config) == "premature_cup_contact"
    sample["contact_force_n"] = [0., 0.]
    sample["cup_displacement_from_start_m"] = .003
    assert preclose_failure(sample, config) is None
    sample["cup_displacement_from_start_m"] = .004
    assert preclose_failure(sample, config) == "cup_displaced_before_close"
    sample["phase"] = "CLOSE"
    assert preclose_failure(sample, config) is None
