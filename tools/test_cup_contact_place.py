import copy
import json

import numpy as np
import pytest

from cup_contact_model import load_config, prepare_model, make_plan, sample_plan
from cup_contact_place import (validate_placement, supported_window, evaluate_placement,
                               touchdown_plan, placement_gate_failure, lowering_failure)


def settled_samples(config, phase="PLACE_HOLD", seconds=1.0):
    return [{"phase": phase, "cup_position_m": config["cup_center_m"].copy(),
             "table_support_force_n": config["cup_mass_kg"]*config["placement"]["gravity_m_s2"],
             "cup_tilt_deg": 0., "contact_force_n": [0., 0.], "contact_center_error_m": .08,
             "gripper_actual_rad": config["gripper_open_rad"], "arm_error_rad": .001}
            for _ in range(round(seconds/config["physics_dt_s"]))]


def test_place_plan_is_opt_in_and_orders_support_release_retreat(tmp_path):
    config = load_config()
    path, _ = prepare_model(tmp_path, config)
    base = make_plan(path, config)
    plan = make_plan(path, config, place=True)
    assert not base["placement_enabled"]
    assert sample_plan(base, 999)["phase"] == "LIFT_HOLD"
    assert plan["poses"][:len(base["poses"])] == base["poses"]
    poses = plan["poses"][len(base["poses"]):]
    assert [p["name"] for p in poses] == ["LOWER", "TABLE_SETTLE", "OPEN", "RELEASE_HOLD",
                                         "WITHDRAW", "CLEAR_ABOVE", "PLACE_HOLD"]
    assert all(p["gripper_rad"] == config["gripper_close_rad"] for p in poses[:2])
    assert all(p["gripper_rad"] == config["gripper_open_rad"] for p in poses[2:])
    np.testing.assert_allclose(poses[0]["target_m"], np.asarray(config["cup_center_m"])-[0, 0, .002])
    assert poses[0]["measurement"]["position_error_mm"] < 2
    assert poses[-3]["target_m"][0] == pytest.approx(.325)
    assert sample_plan(plan, 999)["done"]
    assert sample_plan(plan, 999)["phase"] == "PLACE_HOLD"
    retry = make_plan(path, config, plan["poses"][2]["joint_deg"], place=True)
    assert retry["placement_enabled"]
    assert sample_plan(retry, 999)["phase"] == "PLACE_HOLD"
    json.dumps(retry, allow_nan=False)
    actual = np.array([0., -.3, .8, .2, 1.5], dtype=np.float32)
    latched = touchdown_plan(plan, actual)
    assert latched["poses"][0]["name"] == "TABLE_SETTLE"
    np.testing.assert_allclose(sample_plan(latched, 0)["joint_deg"], np.degrees(actual.astype(float)))
    assert all("target_m" not in p for p in latched["poses"][:3])
    assert all(p["source"] == "observed_touchdown" for p in latched["poses"][:3])
    assert latched["poses"][3:] == plan["poses"][-3:]
    assert plan["poses"][-7]["name"] == "LOWER"
    json.dumps(latched)


def test_support_is_required_before_open_even_at_correct_height():
    config = load_config()
    samples = settled_samples(config, "TABLE_SETTLE", .5)
    for s in samples:
        s["contact_force_n"] = [1., 1.]
        s["gripper_actual_rad"] = config["gripper_close_rad"]
    assert supported_window(samples, config, "TABLE_SETTLE", .5)
    samples[-1]["table_support_force_n"] = 0
    assert not supported_window(samples, config, "TABLE_SETTLE", .5)


def test_edge_support_is_opt_in_and_does_not_certify_upright_release():
    config = load_config()
    tilted = {**config, "edge_support_radius_m": .028,
              "placement": {**config["placement"], "maximum_tilt_deg": 15.}}
    samples = settled_samples(config, "TABLE_SETTLE", .5)
    for sample in samples:
        sample["cup_tilt_deg"] = 13.
        sample["cup_position_m"][2] = (config["table_surface_z_m"]
            +config["cup_height_m"]/2*np.cos(np.radians(13))+.028*np.sin(np.radians(13)))
    assert supported_window(samples, tilted, "TABLE_SETTLE", .5)
    assert not supported_window(samples, config, "TABLE_SETTLE", .5)
    assert not supported_window(samples, {**tilted, "edge_support_radius_m": float("nan")}, "TABLE_SETTLE", .5)
    samples[-1]["table_support_force_n"] = 0.
    assert not supported_window(samples, tilted, "TABLE_SETTLE", .5)


def test_stable_released_clear_cup_and_successful_lift_required():
    config = load_config()
    samples = settled_samples(config)
    assert evaluate_placement(samples, config, True, True, touchdown_observed=True)
    assert not evaluate_placement(samples, config, True, True, touchdown_observed=False)
    assert not evaluate_placement(samples, config, False, True, True)
    assert not evaluate_placement(samples, config, True, False, True)
    assert not evaluate_placement(samples[:1], config, True, True, True)


@pytest.mark.parametrize("key,value", [
    ("table_support_force_n", 0.), ("table_support_force_n", -.1),
    ("table_support_force_n", float("nan")), ("cup_tilt_deg", 10.),
    ("cup_position_m", [.39, .17, .785]), ("cup_position_m", [.415, .17, .78]),
    ("cup_position_m", [.392, .17, .78]), ("contact_force_n", [.02, 0.]),
    ("contact_center_error_m", .04), ("gripper_actual_rad", .5),
    ("arm_error_rad", .2), ("phase", "RELEASE_HOLD"),
])
def test_bad_terminal_state_cannot_pass(key, value):
    config = load_config()
    samples = settled_samples(config)
    samples[-1][key] = value
    assert not evaluate_placement(samples, config, True, True, touchdown_observed=True)


def test_late_support_without_touchdown_cannot_bypass_lower_limit():
    config = load_config()
    samples = settled_samples(config, "TABLE_SETTLE", .5)
    assert supported_window(samples, config, "TABLE_SETTLE", .5)
    assert placement_gate_failure("TABLE_SETTLE", samples, config, True, False) == "table_contact_not_observed_before_lower_limit"
    assert placement_gate_failure("OPEN", samples, config, True, False) == "table_support_not_observed_before_open"
    assert placement_gate_failure("OPEN", samples, config, True, True) is None
    assert placement_gate_failure("LOWER", [], config, False, False) == "lift_not_verified_before_lowering"
    assert placement_gate_failure("TABLE_SETTLE", samples, config, True, True) is None


def test_lost_grasp_before_table_contact_fails_but_supported_transfer_is_allowed():
    config = load_config()
    sample = {"phase": "LOWER", "table_support_force_n": 0., "contact_force_n": [.5, .5],
              "contact_center_error_m": .003, "midbody_height_error_m": .001, "cup_tilt_deg": 0.}
    assert lowering_failure(sample, config) is None
    sample["contact_force_n"] = [0., .5]
    assert lowering_failure(sample, config) == "grasp_contact_lost_during_lowering"
    sample["table_support_force_n"] = config["cup_mass_kg"]*config["placement"]["gravity_m_s2"]
    assert lowering_failure(sample, config) is None
    sample["contact_center_error_m"] = .06
    assert lowering_failure(sample, config) == "cup_separated_from_hand_during_lowering"
    sample["contact_center_error_m"] = .003
    sample["cup_tilt_deg"] = 30.
    assert lowering_failure(sample, config) == "cup_tilt_exceeded_during_lowering"


def test_release_requires_open_hand_and_no_contact_before_withdraw():
    config = load_config()
    samples = settled_samples(config, "RELEASE_HOLD", .5)
    assert supported_window(samples, config, "RELEASE_HOLD", .5, released=True)
    samples[-1]["contact_force_n"] = [.1, 0.]
    assert not supported_window(samples, config, "RELEASE_HOLD", .5, released=True)


def test_placement_config_validation_and_old_v1_defaults(tmp_path):
    config = load_config()
    limits = config.pop("placement")
    path = tmp_path / "v1.json"
    path.write_text(json.dumps(config))
    assert load_config(path)["placement"] == limits
    for key, value in (("lowering_offset_m", -1), ("maximum_window_motion_m", float("nan")),
                       ("minimum_support_weight_ratio", 2)):
        bad = copy.deepcopy(limits)
        bad[key] = value
        with pytest.raises(ValueError):
            validate_placement(bad)
    for bad in (None, {}, {"lowering_offset_m": .002}):
        with pytest.raises(ValueError):
            validate_placement(bad)
