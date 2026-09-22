import json
from pathlib import Path

import numpy as np
import pytest

from bimanual_pour_plan import BimanualContactChain
from plan_body_side_grasp import base_positions
from tray_transfer_plan import audit_carry_path, build_tray_transfer


MODEL = Path("/data/lim/robot-artifacts/pour_20260921/runs/flared03/replacement_hypothesis.urdf")
SOURCE = Path("/data/lim/robot-artifacts/pour_20260921/runs/flared03/plan.json")


@pytest.fixture(scope="module")
def transfer():
    if not MODEL.exists() or not SOURCE.exists():
        pytest.skip("검증된 flared03 로컬 증거가 없습니다")
    return build_tray_transfer(MODEL, json.loads(SOURCE.read_text()))


def test_tray_endpoint_uses_real_top_surface_and_stays_inside_plate(transfer):
    assert transfer["tray_surface_z_m"] == pytest.approx(0.72)
    assert transfer["tray_center_goal_m"] == pytest.approx([0.0, 0.0, 0.78])
    assert np.linalg.norm(transfer["tray_center_m"][:2]) <= 0.020
    assert abs(transfer["tray_target_xy_m"][0]) + 0.045 <= 0.170
    assert abs(transfer["tray_target_xy_m"][1]) + 0.045 <= 0.225
    assert transfer["endpoint_ik_validated"]
    assert transfer["executable"]
    assert not transfer["continuous_collision_validated"]
    assert transfer["continuous_upright_validated"]
    assert not transfer["physics_validated"]
    assert not transfer["execution_blockers"]
    assert not transfer["hardware_accessed"]


def test_plans_are_bimanual_sampler_compatible_and_regrasp_after_release(transfer):
    deposit = transfer["deposit_plan"]["poses"]
    regrasp = transfer["regrasp_plan"]["poses"]
    assert deposit[0]["name"] == "TRAY_START"
    assert [p["name"] for p in deposit][-7:] == [
        "TRAY_LOWER", "TRAY_TABLE_SETTLE", "TRAY_OPEN", "TRAY_RELEASE_HOLD",
        "TRAY_WITHDRAW", "TRAY_CLEAR_ABOVE", "TRAY_PLACE_HOLD",
    ]
    assert [p["name"] for p in regrasp][-2:] == ["TRAY_RETURN_SERVICE", "TRAY_LIFT_HOLD"]
    for pose in [*deposit, *regrasp]:
        assert np.asarray(pose["left_joint_deg"]).shape == (5,)
        assert np.asarray(pose["right_joint_deg"]).shape == (5,)
        assert np.isfinite([
            *pose["left_joint_deg"], *pose["right_joint_deg"],
            pose["left_gripper_rad"], pose["right_gripper_m"], pose["duration_s"],
        ]).all()
    by_name = {pose["name"]: pose for pose in deposit}
    assert by_name["TRAY_TABLE_SETTLE"]["left_gripper_rad"] < by_name["TRAY_OPEN"]["left_gripper_rad"]
    by_name = {pose["name"]: pose for pose in regrasp}
    assert by_name["TRAY_REAPPROACH"]["left_gripper_rad"] > by_name["TRAY_CLOSE"]["left_gripper_rad"]


def test_endpoint_measurements_pass_but_exact_center_is_rejected(transfer):
    poses = {pose["name"]: pose for pose in transfer["deposit_plan"]["poses"]}
    lower = poses["TRAY_LOWER"]["measurement"]
    above = poses["TRAY_MOVE_ABOVE"]["measurement"]
    assert lower["center_goal_xy_error_m"] <= 0.020
    assert lower["cup_upright_tilt_deg"] <= 15.0
    assert 0.718 <= lower["cup_bottom_z_m"] <= 0.721
    assert above["cup_upright_tilt_deg"] <= 15.0
    assert above["cup_bottom_z_m"] == pytest.approx(0.740, abs=0.0002)
    probe = transfer["exact_center_probe"]
    assert probe["multistart_count"] == 32
    assert not probe["accepted"]
    assert probe["rejection_reasons"]


def test_carry_path_audit_accepts_same_branch_center_region_experiment(transfer):
    audit = transfer["path_audit"]
    assert audit["accepted"]
    assert audit["maximum_cup_tilt_deg"] <= 15.0
    deposit = audit["groups"]["deposit_closed_holding"]
    regrasp = audit["groups"]["regrasp_closed_holding"]
    assert deposit["accepted"]
    assert regrasp["accepted"]


def test_carry_path_audit_still_rejects_opposite_wrist_roll_branch():
    if not MODEL.exists() or not SOURCE.exists():
        pytest.skip("검증된 flared03 로컬 증거가 없습니다")
    source = json.loads(SOURCE.read_text())
    chain = BimanualContactChain(MODEL, source["left_config"], source["right_config"])
    held = next(p for p in source["plan"]["poses"] if p["name"] == "POUR_RETURN_CUP")
    source_q = np.radians(held["left_joint_deg"])
    transform = chain.transforms(base_positions("left", source_q))["left_tool0"]
    cup_axis_local = transform[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    flipped = np.asarray(held["left_joint_deg"], dtype=float).copy()
    flipped[4] -= 180.0
    deposit = {"poses": [
        {"name": "TRAY_START", "left_joint_deg": held["left_joint_deg"]},
        {"name": "TRAY_MOVE_ABOVE", "left_joint_deg": flipped.tolist()},
    ]}
    regrasp = {"poses": [
        {"name": "TRAY_CONTACT_HOLD", "left_joint_deg": flipped.tolist()},
        {"name": "TRAY_RETURN_SERVICE", "left_joint_deg": held["left_joint_deg"]},
        {"name": "TRAY_LIFT_HOLD", "left_joint_deg": held["left_joint_deg"]},
    ]}
    audit = audit_carry_path(
        chain, held["left_joint_deg"], deposit, regrasp, cup_axis_local,
    )
    assert not audit["accepted"]
    assert audit["maximum_cup_tilt_deg"] > 89.0


def test_rejects_target_that_puts_cup_outside_top_plate():
    if not MODEL.exists() or not SOURCE.exists():
        pytest.skip("검증된 flared03 로컬 증거가 없습니다")
    with pytest.raises(ValueError, match="안전영역"):
        build_tray_transfer(MODEL, json.loads(SOURCE.read_text()), (0.16, 0.0))
