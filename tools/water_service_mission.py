"""Continuous physical pour, deck transport, regrasp and guest-table service."""
from __future__ import annotations

import asyncio
import copy
import json
import math
from pathlib import Path
import time

import numpy as np

from bimanual_pour_plan import sample_plan
from cup_contact_model import Chain
from cup_contact_place import supported_window, placement_gate_failure
from mobile_service_control import follow_path
from mobile_service_model import ground_collision, ground_window_verified
from pour_geometry import liquid_counts, quaternion_matrix, preclose_violation, inward_pour_direction
from restaurant_layout import load_layout, plan_route
from workcell_preview_inputs import ARM_JOINTS


DECK_LINKS = tuple("tabletop_"+part+"_link" for part in (
    "front_left", "front_right", "rear_left", "rear_right"))
KITCHEN = "/Restaurant/Kitchen/PourWorktop"
GUEST = "/Restaurant/Dining/table_1/Top"


def support_config_for_surface(left, center, surface_z_m, edge_radius_m=None):
    config = {**left, "cup_center_m": list(center), "table_surface_z_m": float(surface_z_m)}
    if edge_radius_m is not None:
        config["edge_support_radius_m"] = edge_radius_m
        config["placement"] = {**left["placement"], "maximum_tilt_deg": 15.}
    return config


def anchor_release_to_touchdown(plan, joint_actual_rad):
    joints_deg = np.degrees(np.asarray(joint_actual_rad, dtype=float))
    if joints_deg.shape != (5,) or not np.isfinite(joints_deg).all():
        raise ValueError("finite five-joint touchdown required")
    for pose in plan["poses"]:
        if any(pose["name"].endswith(s) for s in ("TABLE_SETTLE", "OPEN", "RELEASE_HOLD")):
            pose["left_joint_deg"] = joints_deg.tolist()


def support_contact_failure(state, phase, force_n, cup_center, cup_tilt_deg, surface_z,
                            target_xy_m=(0., 0.)):
    if not np.isfinite([*force_n, *cup_center, cup_tilt_deg, surface_z]).all():
        return "nonfinite_support_observation"
    target_xy = np.asarray(target_xy_m, dtype=float)
    if target_xy.shape != (2,) or not np.isfinite(target_xy).all():
        return "nonfinite_support_observation"
    if np.linalg.norm(force_n) < .02:
        return None
    allowed = (state in {"BACKOUT", "NAVIGATE", "DOCK", "SETTLE_BASE", "REGRASP"}
               or state == "DEPOSIT" and phase in {
                   "LOWER", "TABLE_SETTLE", "OPEN", "RELEASE_HOLD", "WITHDRAW",
                   "CLEAR_ABOVE", "FRONT_CLEAR", "PARK", "PLACE_HOLD"})
    bottom = cup_center[2]-.06*math.cos(math.radians(cup_tilt_deg))
    if (not allowed or np.linalg.norm(np.asarray(cup_center[:2])-target_xy) > .02
            or abs(bottom-surface_z) > .004 or force_n[2] <= 0):
        return "unexpected_support_contact"
    return None


def raised_support_contact_failure(state, phase, force_n, cup_center, cup_tilt_deg, surface_z):
    """Backward-compatible origin-centered raised-support contract."""
    failure = support_contact_failure(
        state, phase, force_n, cup_center, cup_tilt_deg, surface_z)
    return ({"nonfinite_support_observation": "nonfinite_raised_support_observation",
             "unexpected_support_contact": "unexpected_raised_support_contact"}.get(failure, failure))


def support_contacts_failure(state, phase, forces_n, cup_center, tilt_deg, surface_z, target_xy_m):
    for force_n in forces_n:
        failure = support_contact_failure(state, phase, force_n, cup_center, tilt_deg, surface_z, target_xy_m)
        if failure:
            return failure
    return None


def cup_tilt_exceeded(source, tilt_deg):
    limit_deg = 2. if "plate_transfer" in source else 15.
    return not math.isfinite(tilt_deg) or abs(tilt_deg) > limit_deg


def deck_transport_failure(sample, tray_center):
    """A released cup must be supported by the plate, not a hidden attachment."""
    values = [*sample["cup_relative_base_m"], *sample["cup_hand_n"],
              sample["gripper_actual_rad"], sample["deck_support_n"], sample["contact_center_error_m"]]
    if not np.isfinite(values).all():
        return "nonfinite_deck_observation"
    if (np.linalg.norm(np.asarray(sample["cup_relative_base_m"])-tray_center) > .015
            or max(sample["cup_hand_n"]) >= .02 or sample["gripper_actual_rad"] < .9
            or sample["deck_support_n"] < .02 or sample["contact_center_error_m"] < .06):
        return "cup_not_independently_supported_on_deck"
    return None


def regrasp_lift_verified(samples, duration_steps=120):
    window = samples[-duration_steps:]
    return len(window) == duration_steps and all(
        s["phase"] == "LIFT_HOLD" and min(s["cup_hand_n"]) >= .02
        and s["deck_support_n"] < .02
        and np.linalg.norm(np.asarray(s["cup_relative_base_m"])-[.39, .17, .84]) < .02
        and s["cup_tilt_deg"] < 5. for s in window)


def placement_region_verified(samples, target_xy_m=(0., 0.), duration_steps=120):
    target_xy = np.asarray(target_xy_m, dtype=float)
    if target_xy.shape != (2,) or not np.isfinite(target_xy).all():
        return False
    window = samples[-duration_steps:]
    return len(window) == duration_steps and all(
        s["phase"] == "PLACE_HOLD"
        and np.isfinite(s["cup_relative_base_m"]).all()
        and np.linalg.norm(np.asarray(s["cup_relative_base_m"][:2])-target_xy) <= .02
        for s in window)


def central_region_verified(samples, duration_steps=120):
    return placement_region_verified(samples, duration_steps=duration_steps)


def build_service_transfer(model, source):
    variants = [name for name in ("raised_tray", "plate_transfer") if name in source]
    if len(variants) > 1:
        raise ValueError("raised_tray and plate_transfer are mutually exclusive")
    if variants == ["raised_tray"]:
        from raised_tray_transfer import build_raised_transfer
        return build_raised_transfer(model, source)
    if variants == ["plate_transfer"]:
        from raised_tray_transfer import build_plate_transfer
        return build_plate_transfer(model, source)
    from tray_transfer_plan import build_tray_transfer
    return build_tray_transfer(model, source)


def transfer_support_links(source):
    if "raised_tray" in source:
        return ("central_tray_top_link",)
    return DECK_LINKS


def summed_support_force(contact_force_matrix, support_indices):
    forces = np.asarray(contact_force_matrix, dtype=float)
    indices = list(support_indices)
    if forces.ndim != 2 or forces.shape[1] != 3 or not indices:
        raise ValueError("support contacts require a nonempty Nx3 force matrix selection")
    selected = forces[indices]
    if not np.isfinite(selected).all():
        raise ValueError("support contact force must be finite")
    return selected.sum(axis=0)


def placement_requirement_fields(source, verified):
    plate_transfer = "plate_transfer" in source
    return {
        "center_requirement_pass": bool(verified and not plate_transfer),
        "placement_requirement_pass": bool(verified),
        "placement_requirement_contract": ("selected_plate_target_xy_within_20mm"
            if plate_transfer else "origin_center_xy_within_20mm"),
    }


def setup_water_scene(stage, world, context, source, source_dir, rigid, furniture,
                      materials, cup_view, cup_filters):
    import carb
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    from omni.physx.scripts import particleUtils, physicsUtils
    from omni.physx.bindings import _physx as pb
    from isaacsim.core.prims import RigidPrim
    settings = carb.settings.get_settings()
    settings.set(pb.SETTING_UPDATE_TO_USD, True)
    settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, True)
    bottle = stage.DefinePrim("/World/Bottle", "Xform")
    bottle.GetReferences().AddReference(str(source_dir/"scene.usda"), "/World/Bottle")
    UsdPhysics.RigidBodyAPI(bottle).GetRigidBodyEnabledAttr().Set(True)
    # Reuse the coherent settled geometry/particle snapshot, not a new lattice
    # at the pre-settling nominal center. Velocities are initialization-only.
    for path in ("/World/Bottle", "/World/Cup"):
        prim = stage.GetPrimAtPath(path)
        api = UsdPhysics.RigidBodyAPI(prim)
        api.CreateVelocityAttr(Gf.Vec3f(0.))
        api.CreateAngularVelocityAttr(Gf.Vec3f(0.))
    material = UsdShade.Material.Define(stage, "/World/RightWaterServiceMaterial")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(source["right_config"]["friction"])
    api.CreateDynamicFrictionAttr(source["right_config"]["friction"])
    api.CreateRestitutionAttr(0.)
    # Match the support surface used by the source pouring experiment.
    kitchen = stage.GetPrimAtPath(KITCHEN)
    UsdShade.MaterialBindingAPI.Apply(kitchen).Bind(materials["grip"],
        UsdShade.Tokens.strongerThanDescendants, "physics")
    collision = PhysxSchema.PhysxCollisionAPI.Apply(kitchen)
    collision.CreateContactOffsetAttr(.001)
    collision.CreateRestOffsetAttr(0.)
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        right_finger = any(path.startswith(rigid[name]+"/") or path == rigid[name]
                           for name in ("right_finger1_link", "right_finger2_link"))
        if (path.startswith("/World/Bottle") or right_finger) and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
    right_fingers = ("right_finger1_link", "right_finger2_link")
    bottle_filters = [rigid[n] for n in right_fingers]+[KITCHEN, "/World/Cup"]+[
        p for name, p in rigid.items() if name not in right_fingers]+[
        p for p in furniture if p != KITCHEN]+["/Restaurant/Floor"]
    bottle_view = world.scene.add(RigidPrim("/World/Bottle", name="water_bottle_contacts",
        contact_filter_prim_paths_expr=bottle_filters, max_contact_count=4096))
    experiment = source["experiment"]
    spacing = experiment["particle_spacing_m"]
    source_stage = Usd.Stage.Open(str(source_dir/"scene.usda"))
    initial = np.asarray(UsdGeom.Points(source_stage.GetPrimAtPath("/World/Liquid")).GetPointsAttr().Get())
    if initial.ndim != 2 or initial.shape[1] != 3 or not len(initial) or not np.isfinite(initial).all():
        raise ValueError("source liquid snapshot is invalid")
    particle_system = particleUtils.add_physx_particle_system(stage, Sdf.Path("/World/LiquidSystem"),
        simulation_owner=context.prim_path, contact_offset=.0015, rest_offset=.001,
        particle_contact_offset=spacing/1.2, solid_rest_offset=spacing/2, fluid_rest_offset=spacing/2,
        solver_position_iterations=12, enable_ccd=True, max_velocity=5.)
    liquid_material = UsdShade.Material.Define(stage, "/World/LiquidMaterial")
    particleUtils.AddPBDMaterialWater(liquid_material.GetPrim())
    physicsUtils.add_physics_material_to_prim(stage, particle_system.GetPrim(), liquid_material.GetPath())
    fluid = particleUtils.add_physx_particleset_points(stage, "/World/Liquid", initial.tolist(),
        [[0., 0., 0.]]*len(initial), [spacing]*len(initial), particle_system.GetPath(), True, True, 0,
        spacing**3*experiment["fluid_density_kg_m3"], 0.)
    fluid.CreateDisplayColorAttr([Gf.Vec3f(.02, .35, .95)])
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            targets = joint.GetBody0Rel().GetTargets()+joint.GetBody1Rel().GetTargets()
            if any(str(p).startswith(("/World/Cup", "/World/Bottle")) for p in targets):
                raise ValueError("container attachment joint is not allowed")
    return {"cup": cup_view, "bottle": bottle_view, "fluid": fluid, "particle_count": len(initial),
            "cup_filters": cup_filters, "bottle_filters": bottle_filters}


def execute_water_service(args, source, scene, world, robot, controller, names, q, wheels,
                          arm_indices, rigid, body_names, furniture, contact, ground, output):
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from service_camera_views import camera_views
    from pxr import UsdGeom

    camera_config = json.loads((Path(__file__).resolve().parents[1]/"config/simulation/service_cameras.json").read_text())
    camera_frames = []
    detail_count = 0
    cinema = None
    if getattr(args, 'cinematic', False):
        from service_cinematic import CinematicRecorder, cinematic_frame, png_complete
        cinema_config = json.loads((Path(__file__).resolve().parents[1]/'config/simulation/service_cinematic.json').read_text())
        cinema = CinematicRecorder(output, cinema_config)
        args._cinematic_recorder = cinema

    dt = 1/120
    transfer = build_service_transfer(args.source_dir/"replacement_hypothesis.urdf", source)
    (output/"transfer_plan.json").write_text(json.dumps(transfer, indent=2)+"\n")
    all_poses = source["plan"]["poses"]
    end = next(i for i, p in enumerate(all_poses) if p["name"] == "RIGHT_PLACE_HOLD")+1
    pour_plan = {"poses": copy.deepcopy(all_poses[:end])}
    deposit = transfer["deposit_plan"]
    regrasp = transfer["regrasp_plan"]
    guest_poses = copy.deepcopy([p for p in all_poses if p["name"] in {
        "LEFT_LOWER", "LEFT_TABLE_SETTLE", "LEFT_OPEN", "LEFT_RELEASE_HOLD",
        "LEFT_WITHDRAW", "LEFT_CLEAR_ABOVE", "LEFT_PLACE_HOLD"}])
    for pose in guest_poses:
        pose["right_joint_deg"] = regrasp["poses"][-1]["right_joint_deg"]
        pose["right_gripper_m"] = regrasp["poses"][-1]["right_gripper_m"]
    guest_plan = {"poses": [{**regrasp["poses"][-1], "duration_s": .5}, *guest_poses]}
    plans = {"POUR": pour_plan, "DEPOSIT": deposit, "REGRASP": regrasp, "SERVE": guest_plan}
    fk = Chain(args.source_dir/"replacement_hypothesis.urdf")
    left = source["left_config"]
    right = source["right_config"]
    idx = {side: [names.index(side+"_"+j) for j in ARM_JOINTS] for side in ("left", "right")}
    left_grip = names.index("left_gripper")
    right_grip = [names.index(n) for n in ("right_finger1_joint", "right_finger2_joint")]
    cup_filters = scene["cup_filters"]
    deck_indices = [cup_filters.index(rigid[n]) for n in transfer_support_links(source)]
    allowed_cup = {0, 1, cup_filters.index(KITCHEN), cup_filters.index(GUEST), *deck_indices}
    forbidden_cup = [i for i in range(len(cup_filters)) if i not in allowed_cup]
    layout = load_layout()
    layout["waypoints"]["carry_start"] = [-.70, 0., 0.]
    route = plan_route(layout, "carry_start", "table_1")
    route[-1] = layout["waypoints"]["table_1"][:2]
    state, state_start = "POUR", 2.
    state_phase = None
    events, samples, window = [], [], []
    touchdown_events = []
    previous_state = None
    touchdown = {}
    received = None
    placement_verified = False
    baseline = None
    initialization = None
    regrasp_baseline = None
    hold_lost = {"cup": 0., "bottle": 0.}
    holding = {"cup": False, "bottle": False}
    lift_verified = {"cup": False, "bottle": False, "regrasp": False}
    stopped = 0.
    count = scene["particle_count"]
    reason, complete = "timeout", False
    frame_count = 0
    maximum_environment = maximum_self = 0.
    maximum_transit_spill = 0
    last = {}
    started = time.monotonic()
    if args.record:
        (output/"frames").mkdir()
        (output/"detail_frames").mkdir()

    async def capture(path):
        await capture_viewport_to_file(get_active_viewport(), str(path)).wait_for_result()

    def snapshot(path):
        future = asyncio.ensure_future(capture(path))
        deadline = time.monotonic()+30
        while not future.done() and time.monotonic() < deadline:
            world.render()
        if not future.done():
            future.cancel()
            raise TimeoutError("water service capture timed out")
        future.result()
        if cinema is not None:
            # Capture completion schedules an asynchronous PNG writer; await its complete file.
            while not png_complete(path) and time.monotonic() < deadline:
                world.render()
            if not png_complete(path):
                raise TimeoutError('cinematic PNG write did not finish')

    def capture_camera(view, name, path):
        camera_path = "/PresentationCameras/"+name
        camera = UsdGeom.Camera.Define(world.stage, camera_path)
        camera.CreateFocalLengthAttr(view["focal_length_mm"])
        if 'horizontal_aperture_mm' in view:
            camera.CreateHorizontalApertureAttr(view['horizontal_aperture_mm'])
            camera.CreateVerticalApertureAttr(view['horizontal_aperture_mm']*9/16)
        camera.CreateClippingRangeAttr((.01, 100.))
        set_camera_view(np.array(view["eye_m"]), np.array(view["target_m"]), camera_prim_path=camera_path)
        get_active_viewport().camera_path = camera_path
        # Drain the render pipeline after switching cameras without a physics step.
        for _ in range(3):
            world.render()
        snapshot(path)

    def support_config(center, edge_supported=False):
        surface = transfer["tray_surface_z_m"] if state == "DEPOSIT" else left["table_surface_z_m"]
        radius = source["experiment"]["cup_radius_profile_m"][0][1] if edge_supported else None
        return support_config_for_surface(left, center, surface, radius)

    for tick in range(round(args.duration/dt)):
        t = tick*dt
        position, orientation = robot.get_world_pose()
        rotation = quaternion_matrix(orientation)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        phase = state
        command = None
        velocities = [0., 0.]
        nav = None
        if state in plans:
            command = sample_plan(plans[state], max(0., t-state_start))
            phase = command["phase"]
            for side in idx:
                q[idx[side]] = np.radians(command[side+"_joint_deg"])
            q[left_grip], q[right_grip] = command["left_gripper_rad"], command["right_gripper_m"]
            for side in ("left", "right"):
                key = (state, side)
                if key in touchdown and any(phase.endswith(s) for s in ("LOWER", "TABLE_SETTLE", "OPEN", "RELEASE_HOLD")):
                    if phase.startswith(side.upper()) or (side == "left" and state in {"DEPOSIT", "SERVE"}):
                        q[idx[side]] = touchdown[key]
        elif state == "BACKOUT":
            velocities = [-.05/.0329]*2
        elif state in {"NAVIGATE", "DOCK"}:
            path = route if state == "NAVIGATE" else [[-1.75, -2.25]]
            nav = follow_path([*position[:2], yaw], path, math.pi)
            velocities = [nav["left_rad_s"], nav["right_rad_s"]]

        if previous_state != state:
            events.append({"state": state, "time_s": t})
            window = []
            previous_state = state
            state_phase = None
            if state == "REGRASP":
                regrasp_baseline = copy.deepcopy(last)
        local_phase = phase.removeprefix("LEFT_").removeprefix("TRAY_").removeprefix("REGRASP_")
        if state in {"DEPOSIT", "SERVE"} and phase != state_phase:
            target = transfer["tray_center_m"] if state == "DEPOSIT" else [-2.14, -2.42, .78]
            failure = placement_gate_failure(local_phase, window,
                support_config(target, edge_supported=state == "DEPOSIT" and local_phase == "OPEN"
                               and transfer.get("edge_supported_release", False)),
                lift_verified["cup" if state == "DEPOSIT" else "regrasp"],
                (state, "left") in touchdown)
            if failure:
                reason = state.lower()+":"+failure
                break
        state_phase = phase
        controller.apply_action(ArticulationAction(joint_positions=q[arm_indices], joint_indices=arm_indices))
        controller.apply_action(ArticulationAction(joint_velocities=np.asarray(velocities), joint_indices=wheels))
        world.step(render=False)
        position, orientation = robot.get_world_pose()
        rotation = quaternion_matrix(orientation)
        actual = robot.get_joint_positions()
        transforms = fk.transforms(dict(zip(names, actual)))
        forces = np.linalg.norm(contact.get_contact_force_matrix(dt=dt), axis=-1)
        environment_force = float(forces[:, :len(furniture)].max())
        self_force = float(forces[:, len(furniture):].max())
        maximum_environment = max(maximum_environment, environment_force)
        maximum_self = max(maximum_self, self_force)
        ground_forces = np.linalg.norm(ground.get_contact_force_matrix(dt=dt), axis=-1).sum(axis=-1)
        ground_state = dict(zip(body_names, ground_forces.tolist()))
        poses = {kind: tuple(a[0] for a in scene[kind].get_world_poses()) for kind in ("cup", "bottle")}
        cp, cq = poses["cup"]
        bp, bq = poses["bottle"]
        cup_force = scene["cup"].get_contact_force_matrix(dt=dt)[0]
        bottle_force = scene["bottle"].get_contact_force_matrix(dt=dt)[0]
        cf, bf = np.linalg.norm(cup_force, axis=-1), np.linalg.norm(bottle_force, axis=-1)
        counts = liquid_counts(np.asarray(scene["fluid"].GetPointsAttr().Get()), poses["cup"], poses["bottle"], left, right)
        deck_force = summed_support_force(cup_force, deck_indices)
        deck_support = float(deck_force[2])
        guest_support = float(cup_force[cup_filters.index(GUEST), 2])
        tcp = rotation @ transforms["left_contact_center"][:3, 3]+position
        relative_cp = rotation.T @ (cp-position)
        tracking = max(float(np.max(np.abs(actual[idx[side]]-q[idx[side]]))) for side in idx)
        tilt = math.degrees(math.acos(float(np.clip(quaternion_matrix(cq)[2, 2], -1., 1.))))
        last = {"time_s": t, "state": state, "phase": phase, "position_m": position.tolist(),
            "orientation_wxyz": orientation.tolist(), "tilt_deg": math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.)))),
            "cup_position_m": cp.tolist(), "cup_orientation_wxyz": cq.tolist(), "cup_tilt_deg": tilt,
            "cup_relative_base_m": relative_cp.tolist(), "bottle_position_m": bp.tolist(),
            "bottle_orientation_wxyz": bq.tolist(), "cup_hand_n": cf[:2].tolist(), "bottle_hand_n": bf[:2].tolist(),
            "deck_support_n": deck_support, "guest_support_n": guest_support, "bottle_support_n": float(bottle_force[2, 2]),
            "contact_force_n": cf[:2].tolist(), "table_support_force_n": deck_support if state == "DEPOSIT" else guest_support,
            "gripper_actual_rad": float(actual[left_grip]), "contact_center_error_m": float(np.linalg.norm(tcp-cp)),
            "arm_error_rad": tracking, "liquid": counts, "ground_force_n": ground_state,
            "environment_contact_force_n": environment_force, "self_contact_force_n": self_force,
            "wheel_target_rad_s": velocities, "wheel_actual_rad_s": robot.get_joint_velocities()[wheels].tolist()}
        last["bottle_tilt_deg"] = math.degrees(math.acos(float(np.clip(quaternion_matrix(bq)[2, 2], -1., 1.))))
        last["mouth_relative_to_cup_rim_m"] = (
            bp+quaternion_matrix(bq) @ [0., 0., right["cup_height_m"]/2]
            -cp-quaternion_matrix(cq) @ [0., 0., left["cup_height_m"]/2]).tolist()
        last["collision_pairs"] = [
            [body_names[i], (furniture+list(rigid.values()))[j], float(forces[i, j])]
            for i, j in np.argwhere(forces > .05)]
        last["container_forbidden_contacts"] = {
            "cup": {cup_filters[i]: float(cf[i]) for i in forbidden_cup if cf[i] > .05},
            "bottle": {scene["bottle_filters"][i]: float(bf[i]) for i in range(3, len(bf)) if bf[i] > .05}}
        if initialization is None:
            initialization = copy.deepcopy(last)
        if tick == round(2./dt):
            # Match the source runner's settling-before-observation protocol.
            # This is before either arm approaches a container; no pose resets.
            baseline = copy.deepcopy(last)
        window.append({**last, "phase": local_phase,
                       "cup_position_m": relative_cp.tolist() if state == "DEPOSIT" else cp.tolist()})
        if tick % 12 == 0:
            samples.append(last)
        if tick % 600 == 0:
            print(json.dumps({k: last[k] for k in ("time_s", "state", "phase", "cup_relative_base_m", "liquid")}), flush=True)
        if tick % 120 == 0:
            (output/"live.json").write_text(json.dumps(last, indent=2)+"\n")
        if tick % 60 == 0:
            world.render()
            if args.record:
                views = camera_views(camera_config, state, phase, cp)
                capture_camera(views["wide"], "wide", output/"frames"/f"{frame_count:06d}.png")
                record = {"frame": frame_count, "time_s": t, "state": state, "phase": phase, "views": views}
                if "pour_detail" in views:
                    capture_camera(views["pour_detail"], "pour_detail", output/"detail_frames"/f"{detail_count:06d}.png")
                    record["detail_frame"] = detail_count
                    detail_count += 1
                camera_frames.append(record)
                (output/"camera_frames.json").write_text(json.dumps(camera_frames, indent=2)+"\n")
                frame_count += 1
        if cinema is not None:
            cinematic = cinematic_frame(cinema_config, tick, {'base': position, 'cup': cp, 'bottle': bp})
            if cinematic is not None:
                cinematic.update(state=state, phase=phase)
                cinema.record(cinematic, capture_camera)
        if not np.isfinite([*actual, *position, *orientation, *cf, *bf, *ground_forces, environment_force, self_force]).all():
            reason = "nonfinite_observation"
            break
        if environment_force > .05 or self_force > .05 or ground_collision(ground_state):
            reason = "environment_self_or_ground_collision"
            break
        if max(cf[forbidden_cup], default=0.) > .05 or max(bf[3:], default=0.) > .05:
            reason = "container_nonfinger_collision"
            break
        if state == "DEPOSIT" and local_phase in {"WITHDRAW", "CLEAR_ABOVE", "FRONT_CLEAR", "PARK", "PLACE_HOLD"} and max(cf[:2]) >= .02:
            reason = "cup_recontact_after_release"
            break
        if "raised_tray" in source or "plate_transfer" in source:
            support_failure = support_contacts_failure(
                state, local_phase, cup_force[deck_indices], relative_cp, tilt,
                transfer["tray_surface_z_m"], transfer.get("tray_target_xy_m", (0., 0.)))
            if support_failure:
                reason = support_failure
                break
        if tracking > .15 or cup_tilt_exceeded(source, tilt) or last["tilt_deg"] > 1.:
            reason = "arm_tracking_or_container_base_tilt"
            break
        early = preclose_violation(last, baseline) if state == "POUR" and baseline is not None else None
        if state == "REGRASP" and local_phase in {"START", "REGRASP_START", "RETURN_ABOVE", "PREGRASP_ABOVE", "PREGRASP", "REAPPROACH"}:
            early = preclose_violation({**last, "phase": "LEFT_APPROACH"}, regrasp_baseline)
        if early:
            reason = early
            break
        if counts["total"] != count or counts["overlap"]:
            reason = "particle_count_or_container_overlap"
            break
        for kind, side, config, forces_now, pose_now in (
                ("cup", "LEFT", left, cf, cp), ("bottle", "RIGHT", right, bf, bp)):
            if phase == side+"_LIFT" or (kind == "cup" and state == "REGRASP" and local_phase in {"LIFT", "RELIFT"}):
                holding[kind] = True
            if phase == side+"_OPEN" or (kind == "cup" and state == "DEPOSIT" and local_phase == "OPEN"):
                holding[kind] = False
            hold_lost[kind] = hold_lost[kind]+dt if holding[kind] and min(forces_now[:2]) < .02 else 0.
            if hold_lost[kind] >= .1:
                reason = "sustained_grasp_contact_loss:"+kind
                break
            if state == "POUR" and phase == side+"_LIFT_HOLD":
                recent = window[-120:]
                lift_verified[kind] = len(recent) == 120 and all(
                    s["phase"] == phase.removeprefix("LEFT_")
                    and min(s[kind+"_hand_n"]) >= .02
                    and s[kind+"_position_m"][2]-config["cup_center_m"][2] > .04
                    for s in recent)
        if reason.startswith("sustained_grasp_contact_loss"):
            break
        if state == "POUR" and (phase.startswith("POUR_TILT") or phase in {"POUR_HOLD", "POUR_RETURN"}):
            bottle_tilt = math.degrees(math.acos(float(np.clip(quaternion_matrix(bq)[2, 2], -1., 1.))))
            if bottle_tilt >= 15 and not inward_pour_direction(last):
                reason = "pour_direction_not_inward"
                break
        if state == "POUR" and phase == "RIGHT_LOWER" and bottle_force[2, 2] > right["cup_mass_kg"]*9.81*.4:
            touchdown.setdefault((state, "right"), actual[idx["right"]].copy())
        if state in {"DEPOSIT", "SERVE"} and local_phase == "LOWER":
            support = deck_support if state == "DEPOSIT" else guest_support
            if support > left["cup_mass_kg"]*9.81*.4 and (state, "left") not in touchdown:
                touchdown[(state, "left")] = actual[idx["left"]].copy()
                anchor_release_to_touchdown(plans[state], touchdown[(state, "left")])
                touchdown_events.append({"state": state, "time_s": t,
                                         "joint_deg": np.degrees(touchdown[(state, "left")]).tolist()})
        transit = state in {"BACKOUT", "NAVIGATE", "DOCK", "SETTLE_BASE"}
        if transit:
            maximum_transit_spill = max(maximum_transit_spill, received-counts["cup"])
            failure = deck_transport_failure(last, transfer["tray_center_m"])
            if failure:
                reason = failure
                break
            if np.linalg.norm(relative_cp[:2]-transfer.get("tray_target_xy_m", (0., 0.))) > .02:
                placement_verified = False
                reason = "cup_outside_selected_plate_region"
                break
        if received is not None and counts["cup"] < math.ceil(received*.98):
            reason = "water_lost_after_pour"
            break

        next_state = None
        if state == "POUR" and command["done"]:
            if counts["cup"] < math.ceil(count*.60) or counts["outside"] > math.floor(count*.05):
                reason = "pour_delivery_not_verified"
                break
            if float(bottle_force[2, 2]) < .12 or max(bf[:2]) > .02:
                reason = "bottle_return_not_verified"
                break
            received = counts["cup"]
            if not transfer.get("executable", False):
                reason = "tray_carry_path_rejected"
                break
            next_state = "DEPOSIT"
        elif state == "DEPOSIT" and command["done"]:
            if not supported_window(window, support_config(transfer["tray_center_m"]), "PLACE_HOLD", 1., released=True, clear=True):
                reason = "deck_deposit_not_verified"
                break
            placement_verified = placement_region_verified(
                window, transfer.get("tray_target_xy_m", (0., 0.)))
            if not placement_verified:
                reason = "selected_plate_deposit_not_verified"
                break
            next_state = "BACKOUT"
        elif state == "BACKOUT" and position[0] <= -.70:
            next_state = "NAVIGATE"
        elif state == "NAVIGATE" and nav["arrived"]:
            next_state = "DOCK"
        elif state == "DOCK" and nav["arrived"]:
            next_state = "SETTLE_BASE"
        elif state == "SETTLE_BASE":
            rest = np.linalg.norm(robot.get_linear_velocity()) < .005 and np.linalg.norm(robot.get_angular_velocity()) < .01
            stopped = stopped+dt if rest else 0.
            if stopped >= .5:
                next_state = "REGRASP"
        elif state == "REGRASP" and command["done"]:
            if not regrasp_lift_verified(window):
                reason = "regrasp_lift_not_verified"
                break
            lift_verified["regrasp"] = True
            next_state = "SERVE"
        elif state == "SERVE" and command["done"]:
            if supported_window(window, support_config([-2.14, -2.42, .78]), "PLACE_HOLD", 1., released=True, clear=True):
                reason, complete = "water_served", True
            else:
                reason = "guest_release_not_verified"
            break
        if next_state:
            state, state_start = next_state, t+dt
            touchdown = {}
            window = []
    controller.apply_action(ArticulationAction(joint_velocities=np.zeros(2), joint_indices=wheels))
    if args.record or cinema is not None:
        get_active_viewport().camera_path = "/PresentationCameras/cinematic" if cinema is not None else "/PresentationCameras/wide"
        for _ in range(3):
            world.render()
        snapshot(output/"final.png")
    sequence_pass = complete and ground_window_verified(samples[20:])
    requirement = placement_requirement_fields(source, placement_verified)
    if cinema is not None:
        try:
            cinema.close()
        finally:
            args._cinematic_recorder = None
    return {"task_pass": bool(sequence_pass and placement_verified), "sequence_pass": sequence_pass,
        **requirement,
        "failure": "tray_target_placement_not_verified" if sequence_pass and not placement_verified else reason,
        "water_pour_included": True, "deck_transport_included": True, "regrasp_included": True,
        "water_pour_completed": received is not None,
        "deck_transport_completed": any(e["state"] == "REGRASP" for e in events),
        "regrasp_completed": any(e["state"] == "SERVE" for e in events),
        "tray_path_audit": transfer.get("path_audit"),
        "contact_filters": {"cup": cup_filters, "bottle": scene["bottle_filters"]},
        "simulated_s": tick*dt, "elapsed_wall_s": time.monotonic()-started,
        "ground_verified": ground_window_verified(samples[20:]), "final_state": state,
        "events": events, "samples": samples, "final_observation": last,
        "touchdown_events": touchdown_events,
        "initialization_observation": initialization, "pregrasp_baseline": baseline,
        "received_particles": received, "maximum_transit_particle_loss": maximum_transit_spill,
        "maximum_environment_force_n": maximum_environment, "maximum_self_force_n": maximum_self,
        "frame_count": frame_count, "recording_fps": 2, "detail_frame_count": detail_count,
        "cinematic_frame_count": len(cinema.frames) if cinema is not None else 0,
        "cinematic_fps": cinema_config['fps'] if cinema is not None else None,
        "camera_recording": "cinematic_native_frames_edited_speed" if cinema is not None else "fixed_wide_with_synchronous_pour_detail", "tray_center_m": transfer["tray_center_m"],
        "tray_target_xy_m": transfer.get("tray_target_xy_m", [0., 0.]),
        "observation_source": "simulator_ground_truth", "model_calibrated": False}
