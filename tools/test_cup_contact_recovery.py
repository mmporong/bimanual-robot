import copy
import json

import numpy as np
import pytest

from cup_contact_model import load_config, prepare_model, make_plan, sample_plan
from cup_contact_recovery import (validate_recovery, retreat_plan, recovery_failure,
                                  observe_stationary_cup)


def observation(config):
    return [{"phase": "REOBSERVE", "cup_position_m": config["cup_center_m"].copy(),
             "cup_tilt_deg": 0., "contact_force_n": [0., 0.]}
            for _ in range(round(config["recovery"]["observation_duration_s"]/config["physics_dt_s"]))]


def test_observation_uses_new_position_without_changing_original_config():
    config = load_config()
    samples = observation(config)
    for s in samples:
        s["cup_position_m"][1] -= .005
    result = observe_stationary_cup(samples, config)
    np.testing.assert_allclose(result, [.39, .165, .78])
    assert config["cup_center_m"][1] == .17


@pytest.mark.parametrize("field,value,reason", [
    ("phase", "RETREAT", "window_incomplete"),
    ("contact_force_n", [.02, 0], "contact_not_released"),
    ("cup_tilt_deg", 10, "not_upright"),
    ("cup_position_m", [.4, .17, .78], "not_stationary"),
    ("cup_position_m", [float("nan"), .17, .78], "nonfinite"),
    ("contact_force_n", [float("nan"), 0], "nonfinite"),
    ("cup_tilt_deg", float("nan"), "nonfinite"),
])
def test_bad_observation_rejected(field, value, reason):
    config = load_config()
    samples = observation(config)
    samples[-1][field] = value
    with pytest.raises(ValueError, match=reason):
        observe_stationary_cup(samples, config)


def test_short_or_fallen_cup_observation_rejected():
    config = load_config()
    with pytest.raises(ValueError, match="incomplete"):
        observe_stationary_cup(observation(config)[:1], config)
    samples = observation(config)
    for s in samples:
        s["cup_position_m"][2] -= .04
    with pytest.raises(ValueError, match="not_on_table"):
        observe_stationary_cup(samples, config)


def test_reverse_replays_actual_positions_with_open_hand_only():
    config = load_config()
    samples = [{"phase": "PREGRASP_ABOVE" if i == 0 else "ALIGN_MIDDLE",
                "left_joint_actual_rad": [i*.001]*5} for i in range(61)]
    plan = retreat_plan(samples, config)
    np.testing.assert_allclose(plan["poses"][0]["joint_deg"], np.degrees(samples[-1]["left_joint_actual_rad"]))
    np.testing.assert_allclose(plan["poses"][-1]["joint_deg"], [0]*5)
    assert all(p["gripper_rad"] == config["gripper_open_rad"] for p in plan["poses"])
    assert sample_plan(plan, 100)["phase"] == "REOBSERVE"
    assert sample_plan(plan, 100)["done"]
    assert sum(p["duration_s"] for p in plan["poses"]) >= .75
    with pytest.raises(ValueError, match="이력 없음"):
        retreat_plan(samples[1:], config)
    samples[-1]["phase"] = "CLOSE"
    with pytest.raises(ValueError, match="이력 없음"):
        retreat_plan(samples, config)


def test_replan_starts_from_observed_joints_and_new_cup_target(tmp_path):
    config = load_config()
    model, _ = prepare_model(tmp_path, config)
    original = make_plan(model, config)
    above = next(p for p in original["poses"] if p["name"] == "PREGRASP_ABOVE")
    config["cup_center_m"][1] -= .005
    retry = make_plan(model, config, np.array(above["joint_deg"], dtype=np.float32))
    json.dumps(retry, allow_nan=False)
    np.testing.assert_allclose(sample_plan(retry, 0)["joint_deg"], above["joint_deg"])
    assert "REORIENT_ABOVE" not in [p["name"] for p in retry["poses"]]
    approach = next(p for p in retry["poses"] if p["name"] == "APPROACH")
    np.testing.assert_allclose(approach["target_m"], [.39, .165, .78])
    assert approach["measurement"]["position_error_mm"] < 2


def test_recovery_limits_and_config_validation():
    config = load_config()
    sample = observation(config)[0]
    assert recovery_failure(sample, np.array(config["cup_center_m"]), config) is None
    for key, value, reason in (("cup_tilt_deg", 20, "tilt"),
                               ("contact_force_n", [2.1, 0], "force"),
                               ("cup_position_m", [.43, .17, .78], "displacement")):
        bad = {**sample, key: value}
        failure = recovery_failure(bad, np.array(config["cup_center_m"]), config)
        assert failure is not None
        assert reason in failure
    for key, value in (("max_retries", -1), ("max_retries", True), ("max_retries", 4),
                       ("reverse_time_scale", .5), ("observation_duration_s", float("nan"))):
        bad = copy.deepcopy(config["recovery"])
        bad[key] = value
        with pytest.raises(ValueError):
            validate_recovery(bad)
    for bad in (None, {}, {"max_retries": 1}):
        with pytest.raises(ValueError, match="필수 항목"):
            validate_recovery(bad)


def test_old_v1_config_gets_recordable_recovery_defaults(tmp_path):
    config = load_config()
    expected = config.pop("recovery")
    path = tmp_path / "old.json"
    path.write_text(json.dumps(config))
    assert load_config(path)["recovery"] == expected
