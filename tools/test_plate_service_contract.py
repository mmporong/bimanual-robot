import numpy as np

from water_service_mission import (
    DECK_LINKS,
    build_service_transfer,
    placement_region_verified,
    placement_requirement_fields,
    summed_support_force,
    support_contact_failure,
    transfer_support_links,
    support_contacts_failure,
    cup_tilt_exceeded,
)


def _placement_sample(xy):
    return {"phase": "PLACE_HOLD", "cup_relative_base_m": [*xy, .78]}


def test_plate_placement_is_verified_around_selected_xy_not_origin():
    target = np.array([.08, -.06])
    samples = [_placement_sample(target+[.01, 0.]) for _ in range(120)]
    assert placement_region_verified(samples, target)
    assert not placement_region_verified(samples)


def test_plate_result_does_not_report_arbitrary_xy_as_origin_center_success():
    fields = placement_requirement_fields({"plate_transfer": {}}, True)
    assert fields == {
        "center_requirement_pass": False,
        "placement_requirement_pass": True,
        "placement_requirement_contract": "selected_plate_target_xy_within_20mm",
    }
    assert placement_requirement_fields({}, True)["center_requirement_pass"]


def test_plate_uses_all_four_real_tabletop_support_links():
    assert transfer_support_links({"plate_transfer": {}}) == DECK_LINKS
    assert len(transfer_support_links({"plate_transfer": {}})) == 4
    assert transfer_support_links({"raised_tray": {}}) == ("central_tray_top_link",)


def test_support_force_is_vector_sum_across_all_selected_tabletops():
    forces = np.array([
        [0., 0., .1], [0., 0., .2], [0., 0., .3], [0., 0., .4], [9., 0., 9.],
    ])
    assert np.allclose(summed_support_force(forces, range(4)), [0., 0., 1.])


def test_tabletop_contact_is_allowed_only_at_selected_target_height_and_phase():
    target = [.08, -.06]
    center = [.08, -.06, .78]
    assert support_contact_failure(
        "DEPOSIT", "LOWER", [0., 0., 1.], center, 0., .72, target) is None
    assert support_contact_failure(
        "NAVIGATE", "NAVIGATE", [0., 0., 1.], center, 0., .72, target) is None
    for state, phase, force, observed in (
        ("DEPOSIT", "MOVE_ABOVE", [0., 0., 1.], center),
        ("DEPOSIT", "LOWER", [1., 0., 0.], center),
        ("DEPOSIT", "LOWER", [0., 0., 1.], [0., 0., .78]),
        ("DEPOSIT", "LOWER", [0., 0., 1.], [.08, -.06, .76]),
    ):
        assert support_contact_failure(state, phase, force, observed, 0., .72, target)


def test_plate_transfer_routes_to_plate_builder(monkeypatch):
    import raised_tray_transfer
    expected = {"schema": "plate_transfer_v1"}
    monkeypatch.setattr(raised_tray_transfer, "build_plate_transfer",
                        lambda model, source: expected)
    assert build_service_transfer("model.urdf", {"plate_transfer": {}}) is expected


def test_transfer_variants_are_mutually_exclusive_before_building():
    source = {"raised_tray": {}, "plate_transfer": {}}
    try:
        build_service_transfer("unused.urdf", source)
    except ValueError as exc:
        assert "mutually exclusive" in str(exc)
    else:
        raise AssertionError("exclusive transfer variants were accepted")


def test_opposing_illegal_support_contacts_cannot_cancel():
    forces = [[1., 0., 0.], [-1., 0., 0.]]
    assert np.allclose(summed_support_force(forces, [0, 1]), [0., 0., 0.])
    assert support_contacts_failure("DEPOSIT", "LOWER", forces, [.12, -.09, .78],
                                    0., .72, [.12, -.09]) == "unexpected_support_contact"


def test_plate_physical_tilt_limit_is_two_degrees_not_only_nominal_ik():
    assert not cup_tilt_exceeded({"plate_transfer": {}}, 2.)
    assert cup_tilt_exceeded({"plate_transfer": {}}, 2.1)
    assert cup_tilt_exceeded({"plate_transfer": {}}, float("nan"))
    assert not cup_tilt_exceeded({"raised_tray": {}}, 2.1)
