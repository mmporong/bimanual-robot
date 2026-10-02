"""Isolated empty-cup withdrawal/regrasp test, not a complete serving trial."""
from __future__ import annotations

import json
import math
import numpy as np

from bimanual_pour_plan import sample_plan
from mobile_service_model import ground_collision
from pour_geometry import quaternion_matrix
from water_service_mission import build_service_transfer, summed_support_force, transfer_support_links
from workcell_preview_inputs import ARM_JOINTS


def arm_tracking_error_rad(actual, goal, arm_indices_by_side):
    return max(float(np.max(np.abs(actual[indices]-goal[indices])))
               for indices in arm_indices_by_side.values())


def run_probe(args, source, world, robot, controller, names, q, wheels, arm_indices,
              body_names, rigid, furniture, contact, ground, cup, cup_filters, output):
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view

    transfer = build_service_transfer(args.source_dir/"replacement_hypothesis.urdf", source)
    deposit = transfer["deposit_plan"]["poses"]
    regrasp = transfer["regrasp_plan"]["poses"]
    nominal = next(p for p in regrasp if p["name"] == "TRAY_REAPPROACH")
    initial = {**nominal, "name": "PROBE_SETTLE", "duration_s": 1.}
    start = next(i for i, p in enumerate(deposit) if p["name"] == "TRAY_WITHDRAW")
    plan = {"poses": [initial, *deposit[start:], *regrasp]}
    (output/"probe_plan.json").write_text(json.dumps(plan, indent=2)+"\n")
    ids = {s: [names.index(s+"_"+j) for j in ARM_JOINTS] for s in ("left", "right")}
    for side in ids:
        q[ids[side]] = np.radians(initial[side+"_joint_deg"])
    q[names.index("left_gripper")] = source["left_config"]["gripper_open_rad"]
    robot.set_world_pose(np.zeros(3), np.array([1., 0., 0., 0.]))
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    cup.set_world_poses(positions=np.array([transfer["tray_center_m"]]), orientations=np.array([[1., 0., 0., 0.]]))
    cup.set_velocities(np.zeros((1, 6)))
    support_ids = [cup_filters.index(rigid[name]) for name in transfer_support_links(source)]
    forbidden = [i for i in range(len(cup_filters)) if i not in {0, 1, *support_ids}]
    baseline = None
    samples = []
    completed = False
    reason = "timeout"
    previous_phase = None
    held_steps = 0
    dt = 1/120
    for tick in range(round(args.duration/dt)):
        t = tick*dt
        command = sample_plan(plan, t)
        phase = command["phase"]
        for side in ids:
            q[ids[side]] = np.radians(command[side+"_joint_deg"])
        q[names.index("left_gripper")] = command["left_gripper_rad"]
        controller.apply_action(ArticulationAction(joint_positions=q[arm_indices], joint_indices=arm_indices))
        controller.apply_action(ArticulationAction(joint_velocities=np.zeros(2), joint_indices=wheels))
        world.step(render=args.record and tick % 60 == 0)
        cp, cq = cup.get_world_poses()
        cup_force = cup.get_contact_force_matrix(dt=dt)[0]
        cf = np.linalg.norm(cup_force, axis=-1)
        support_force = summed_support_force(cup_force, support_ids)
        rf = np.linalg.norm(contact.get_contact_force_matrix(dt=dt), axis=-1)
        gf = np.linalg.norm(ground.get_contact_force_matrix(dt=dt), axis=-1).sum(axis=-1)
        actual = robot.get_joint_positions()
        tilt = math.degrees(math.acos(float(np.clip(quaternion_matrix(cq[0])[2, 2], -1., 1.))))
        # A cup intentionally prevents the gripper from reaching its empty-close target.
        error = arm_tracking_error_rad(actual, q, ids)
        sample = {"time_s": t, "phase": phase, "cup_position_m": cp[0].tolist(),
                  "cup_tilt_deg": tilt, "hand_force_n": cf[:2].tolist(),
                  "support_force_n": float(support_force[2]), "arm_error_rad": error,
                  "gripper_actual_rad": float(actual[names.index("left_gripper")])}
        filters = furniture+list(rigid.values())
        sample["collision_pairs"] = [[body_names[i], filters[j], float(rf[i, j])]
                                     for i, j in np.argwhere(rf > .05)]
        sample["forbidden_cup_contacts"] = {cup_filters[i]: float(cf[i]) for i in forbidden if cf[i] > .05}
        if tick % 12 == 0:
            samples.append(sample)
        if phase != previous_phase:
            print(json.dumps(sample), flush=True)
            previous_phase = phase
        if not np.isfinite([*cp[0], *cq[0], *cf, *gf, *actual, rf.max()]).all():
            reason = "nonfinite_observation"
            break
        if rf.max() > .05 or ground_collision(dict(zip(body_names, gf))):
            reason = "robot_collision"
            break
        if max(cf[forbidden], default=0.) > .05:
            reason = "cup_nonfinger_collision"
            break
        if error > .15 or tilt > 3.:
            reason = "tracking_or_cup_tilt"
            break
        if t >= 1. and baseline is None:
            baseline = cp[0].copy()
        open_hand = command["left_gripper_rad"] > .9
        if t >= 1. and open_hand:
            if max(cf[:2]) >= .02 or np.linalg.norm(cp[0]-baseline) > .003:
                reason = "released_cup_contact_or_motion"
                break
        held = (phase == "TRAY_LIFT_HOLD" and min(cf[:2]) >= .02
                and max(cf[support_ids], default=0.) < .02 and cp[0][2] > .81)
        held_steps = held_steps+1 if held else 0
        if command["done"]:
            completed = held_steps >= 120
            reason = "isolated_regrasp_verified" if completed else "isolated_regrasp_not_held"
            break
    set_camera_view(np.array([-1.1, -1.2, 1.8]), np.array([0., 0., .95]))
    for _ in range(4):
        world.render()
    return {"task_pass": completed, "probe_pass": completed, "failure": reason,
            "mission_executed": False, "fluid_simulated": False,
            "initialization_pose_reset": True, "final_observation": sample,
            "samples": samples, "simulated_s": t}
