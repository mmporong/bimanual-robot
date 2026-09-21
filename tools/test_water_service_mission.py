import copy

from water_service_mission import deck_transport_failure, regrasp_lift_verified, central_region_verified


def observation():
    return {"cup_relative_base_m": [0., .05, .78], "cup_hand_n": [0., 0.],
            "gripper_actual_rad": 1., "deck_support_n": .4,
            "contact_center_error_m": .1, "cup_tilt_deg": 0.}


def test_transport_requires_release_support_and_hand_clearance():
    sample = observation()
    center = [0., .05, .78]
    assert deck_transport_failure(sample, center) is None
    for key, value in (("cup_hand_n", [.03, 0.]), ("gripper_actual_rad", .2),
                       ("deck_support_n", 0.), ("contact_center_error_m", .01),
                       ("cup_relative_base_m", [.05, .05, .78])):
        assert deck_transport_failure({**sample, key: value}, center)
    assert deck_transport_failure({**sample, "deck_support_n": float("nan")}, center)


def test_regrasp_requires_sustained_two_finger_lift_not_one_frame():
    sample = {**observation(), "phase": "LIFT_HOLD", "cup_hand_n": [.1, .1],
              "deck_support_n": 0., "cup_relative_base_m": [.39, .17, .84]}
    assert not regrasp_lift_verified([sample])
    samples = [copy.deepcopy(sample) for _ in range(120)]
    assert regrasp_lift_verified(samples)
    samples[60]["cup_hand_n"] = [0., .1]
    assert not regrasp_lift_verified(samples)
    samples[60] = {**sample, "deck_support_n": .4}
    assert not regrasp_lift_verified(samples)


def test_central_deposit_needs_observed_window_not_planned_endpoint():
    sample = {**observation(), "phase": "PLACE_HOLD", "cup_relative_base_m": [0., .01, .78]}
    assert not central_region_verified([])
    assert not central_region_verified([sample])
    assert central_region_verified([sample]*120)
    assert not central_region_verified([sample]*119+[{**sample, "cup_relative_base_m": [0., .03, .78]}])
