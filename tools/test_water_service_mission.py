import copy

from water_service_mission import (
    deck_transport_failure, regrasp_lift_verified, central_region_verified,
    support_config_for_surface,
    raised_support_contact_failure,
    anchor_release_to_touchdown,
)


def test_withdraw_starts_from_observed_touchdown_not_overtravel():
    from bimanual_pour_plan import sample_plan
    import numpy as np
    def pose(name, left):
        return {"name": name, "duration_s": 1., "left_joint_deg": left,
                "right_joint_deg": [0.]*5, "left_gripper_rad": 1., "right_gripper_m": .04}
    plan = {"poses": [pose("TRAY_RELEASE_HOLD", [-1.]*5), pose("TRAY_WITHDRAW", [10.]*5)]}
    anchor_release_to_touchdown(plan, np.radians([1., 2., 3., 4., 5.]))
    command = sample_plan(plan, 1.+1e-9)
    assert np.allclose(command["left_joint_deg"], [1., 2., 3., 4., 5.])


def test_raised_deposit_and_guest_table_use_different_surface_heights():
    left = {"table_surface_z_m": .72, "placement": {"maximum_tilt_deg": 3.}}
    raised = support_config_for_surface(left, [0., 0., .94], .88)
    guest = support_config_for_surface(left, [-2.14, -2.42, .78], .72)
    assert raised["table_surface_z_m"] == .88
    assert guest["table_surface_z_m"] == .72
    assert left["table_surface_z_m"] == .72
    assert raised["placement"]["maximum_tilt_deg"] == 3.


def test_raised_top_side_and_early_contacts_are_not_support():
    for state, phase, force, center in (
        ("DEPOSIT", "MOVE_ABOVE", [0., 0., 1.], [0., 0., .94]),
        ("DEPOSIT", "LOWER", [1., 0., 0.], [0., 0., .94]),
        ("DEPOSIT", "LOWER", [0., 0., 1.], [.06, 0., .94]),
        ("DEPOSIT", "LOWER", [0., 0., 1.], [0., 0., .84]),
    ):
        assert raised_support_contact_failure(state, phase, force, center, 0., .88)
    assert raised_support_contact_failure("DEPOSIT", "LOWER", [0., 0., 1.], [0., 0., .94], 0., .88) is None
    assert raised_support_contact_failure("NAVIGATE", "NAVIGATE", [0., 0., 1.], [0., 0., .94], 0., .88) is None
    assert raised_support_contact_failure("NAVIGATE", "NAVIGATE", [0., 0., float("nan")], [0., 0., .94], 0., .88)


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
