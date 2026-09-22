"""Independent static-pose collision probes; never a motion/serving demonstration."""
from __future__ import annotations

import asyncio
import json
import time

import numpy as np

from mobile_service_model import ground_collision
from workcell_preview_inputs import ARM_JOINTS


def load_cases(path):
    cases = json.loads(path.read_text())["cases"]
    if not isinstance(cases, list) or not cases:
        raise ValueError("nonempty cases required")
    ids = set()
    for case in cases:
        if not isinstance(case.get("id"), str) or case["id"] in ids:
            raise ValueError("unique string probe id required")
        ids.add(case["id"])
        for side in ("left", "right"):
            values = np.asarray(case[side+"_joint_deg"], dtype=float)
            if values.shape != (5,) or not np.isfinite(values).all():
                raise ValueError("five finite joint angles required")
    return cases


def run_probes(args, world, robot, controller, names, q, wheels, arm_indices,
               body_names, rigid, furniture, contact, ground, output):
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

    cases = load_cases(args.pose_probes)
    results = []
    dt = 1/120
    filters = furniture+list(rigid.values())
    base = np.array([-.7, 0., 0.])
    left_ids = [names.index("left_"+j) for j in ARM_JOINTS]
    right_ids = [names.index("right_"+j) for j in ARM_JOINTS]
    started = time.monotonic()
    for number, case in enumerate(cases):
        q[left_ids] = np.radians(case["left_joint_deg"])
        q[right_ids] = np.radians(case["right_joint_deg"])
        robot.set_world_pose(base, np.array([1., 0., 0., 0.]))
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        pair_peaks = {}
        unexpected_ground = False
        maximum_error = 0.
        for tick in range(24):
            controller.apply_action(ArticulationAction(joint_positions=q[arm_indices], joint_indices=arm_indices))
            controller.apply_action(ArticulationAction(joint_velocities=np.zeros(2), joint_indices=wheels))
            world.step(render=args.record and tick >= 20)
            forces = np.linalg.norm(contact.get_contact_force_matrix(dt=dt), axis=-1)
            if not np.isfinite(forces).all():
                raise ValueError("nonfinite static probe contact")
            for i, j in np.argwhere(forces > .05):
                key = (body_names[i], filters[j])
                pair_peaks[key] = max(pair_peaks.get(key, 0.), float(forces[i, j]))
            ground_values = np.linalg.norm(ground.get_contact_force_matrix(dt=dt), axis=-1).sum(axis=-1)
            actual = robot.get_joint_positions()[arm_indices]
            if not np.isfinite(ground_values).all() or not np.isfinite(actual).all():
                raise ValueError("nonfinite static probe ground or joint state")
            unexpected_ground |= ground_collision(dict(zip(body_names, ground_values)))
            error = float(np.max(np.abs(actual-q[arm_indices])))
            maximum_error = max(maximum_error, error)
        record = {"id": case["id"], "collision_clear": not pair_peaks and not unexpected_ground,
            "unexpected_ground_contact": bool(unexpected_ground),
            "collision_pairs": [[*key, value] for key, value in pair_peaks.items()],
            "maximum_arm_error_rad": maximum_error,
            "joint_deg": {"left": case["left_joint_deg"], "right": case["right_joint_deg"]},
            "initialization_reset": True, "motion_path_validated": False}
        results.append(record)
        print(json.dumps(record), flush=True)
        if args.record:
            set_camera_view(base+[-1., -.85, 1.4], base+[0., 0., .85])
            # Synchronize the latest physics transforms before requesting an image.
            for _ in range(4):
                world.render()
            async def capture():
                await capture_viewport_to_file(get_active_viewport(), str(output/f"probe_{number:03d}.png")).wait_for_result()
            future = asyncio.ensure_future(capture())
            deadline = time.monotonic()+30.
            while not future.done() and time.monotonic() < deadline:
                world.render()
            if not future.done():
                future.cancel()
                raise TimeoutError("probe capture")
            future.result()
    passed = all(r["collision_clear"] and r["maximum_arm_error_rad"] <= .15 for r in results)
    return {"task_pass": passed, "failure": "static_poses_clear" if passed else "static_pose_collision_or_tracking",
        "probe_results": results, "mission_executed": False, "motion_path_validated": False,
        "pose_resets_are_diagnostic_only": True, "elapsed_wall_s": time.monotonic()-started}
