#!/usr/bin/env python3
"""Free-base wheel/contact experiment in the restaurant. Never accesses hardware."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import ExitStack, nullcontext
import hashlib
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np

from build_restaurant_scene import build, validate_scene_geometry
from mobile_service_control import follow_path
from restaurant_layout import load_layout
from workcell_preview_inputs import ARM_JOINTS
import bottle_contact_model as bottle_model
from bimanual_pour_plan import sample_plan
from pour_geometry import quaternion_matrix
from restaurant_layout import plan_route
from cup_contact_model import Chain
from cup_contact_place import supported_window, placement_gate_failure
from mobile_service_model import fixed_component_pairs, ground_collision, ground_window_verified


def record_failure(result, exc):
    """Post-processing failures invalidate all success flags as well."""
    result.update(task_pass=False, negative_control_pass=False,
                  failure=type(exc).__name__+": "+str(exc))


def run(args):
    result = {}
    _run(args, result)
    # Evaluate after evidence persistence and all cleanup, not before finally.
    return 0 if result.get('task_pass') or result.get('negative_control_pass') else 1


def _run(args, result):
    roundtrip = getattr(args, 'dock_roundtrip', False)
    if roundtrip and (args.mode != 'water-service' or getattr(args, 'cinematic', False)):
        raise ValueError('--dock-roundtrip requires water-service without cinematic timeline')
    if getattr(args, 'executor_socket', None) and args.mode != 'water-service':
        raise ValueError('--executor-socket requires water-service')
    if getattr(args, 'cinematic', False) and (args.mode != 'water-service' or args.record):
        raise ValueError('--cinematic requires water-service and replaces --record')
    if args.mode == "static-pose-probe" and args.pose_probes is None:
        raise ValueError("static-pose-probe requires --pose-probes")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), output/"executed_runner.py")
    source = json.loads((args.source_dir / "plan.json").read_text())
    water_service = args.mode == "water-service"
    service = args.mode in {"service", "pick-only", "water-service", "released-cup-probe"}
    pick_plan = {"poses": source["plan"]["poses"][:9]}
    placement_poses = [p.copy() for p in source["plan"]["poses"] if p["name"] in {
        "LEFT_LOWER", "LEFT_TABLE_SETTLE", "LEFT_OPEN", "LEFT_RELEASE_HOLD",
        "LEFT_WITHDRAW", "LEFT_CLEAR_ABOVE", "LEFT_PLACE_HOLD"}]
    # Keep the unused right arm parked throughout this transport-only experiment.
    for p in placement_poses:
        p["right_joint_deg"] = pick_plan["poses"][0]["right_joint_deg"]
        p["right_gripper_m"] = pick_plan["poses"][0]["right_gripper_m"]
    place_plan = {"poses": [{**pick_plan["poses"][-1], "duration_s": .5}, *placement_poses]}
    fk = Chain(args.source_dir / "replacement_hypothesis.urdf")
    place_config = {**source["left_config"], "cup_center_m": [-2.14, -2.42, .78]}
    from isaacsim import SimulationApp
    cinematic = getattr(args, 'cinematic', False)
    app = SimulationApp({"headless": args.headless, "width": 1920 if cinematic else (1600 if water_service else 1280), "height": 1080 if cinematic else 900,
                         "renderer": "RayTracedLighting", "anti_aliasing": 0,
                         "multi_gpu": False, "fast_shutdown": True})
    samples = []
    result.update({"task_pass": False, "hardware_accessed": False,
              "object_attachment_used": False, "nav2_executed": False,
              "failure": "initialization", "mode": args.mode})
    result["tool_sha256"] = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ("simulate_restaurant_mobile.py", "mobile_service_control.py", "build_restaurant_scene.py",
            "restaurant_layout.py", "bimanual_pour_plan.py", "pour_geometry.py", "bottle_contact_model.py",
            "cup_contact_model.py", "cup_contact_place.py", "workcell_preview_inputs.py", "plan_body_side_grasp.py", "mobile_service_model.py")}
    result["input_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [args.source_dir/"plan.json", args.source_dir/"replacement_hypothesis.urdf",
            args.source_dir/"scene.usda", Path(__file__).resolve().parents[1]/"config/simulation/restaurant_layout.json",
            Path(__file__).resolve().parents[1]/"src/hold_flow_description/scripts/solve_task_poses.py"]}
    if water_service:
        for name in ("water_service_mission.py", "tray_transfer_plan.py", "raised_tray_transfer.py",
                     "search_tray_mounts.py", "service_camera_views.py"):
            result["tool_sha256"][name] = hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        camera_config = Path(__file__).resolve().parents[1]/"config/simulation/service_cameras.json"
        result["input_sha256"][str(camera_config)] = hashlib.sha256(camera_config.read_bytes()).hexdigest()
        if getattr(args, 'executor_socket', None):
            result['tool_sha256']['isaac_phase_executor.py'] = hashlib.sha256(
                Path(__file__).with_name('isaac_phase_executor.py').read_bytes()).hexdigest()
            result['executor_mode'] = 'isaac_phase_gated_single_mission'
        if roundtrip:
            from restaurant_roundtrip import CONFIG
            result['input_sha256'][str(CONFIG)] = hashlib.sha256(CONFIG.read_bytes()).hexdigest()
            result['tool_sha256']['restaurant_roundtrip.py'] = hashlib.sha256(
                Path(__file__).with_name('restaurant_roundtrip.py').read_bytes()).hexdigest()
            result['mission_scope'] = 'dock_roundtrip'
    if cinematic:
        result['tool_sha256']['service_cinematic.py'] = hashlib.sha256(Path(__file__).with_name('service_cinematic.py').read_bytes()).hexdigest()
        cinematic_config = Path(__file__).resolve().parents[1]/'config/simulation/service_cinematic.json'
        result['input_sha256'][str(cinematic_config)] = hashlib.sha256(cinematic_config.read_bytes()).hexdigest()
    if args.mode == "static-pose-probe":
        result["input_sha256"][str(args.pose_probes)] = hashlib.sha256(args.pose_probes.read_bytes()).hexdigest()
        result["tool_sha256"]["tray_pose_probe.py"] = hashlib.sha256(Path(__file__).with_name("tray_pose_probe.py").read_bytes()).hexdigest()
    if args.mode == "released-cup-probe":
        for name in ("released_cup_probe.py", "water_service_mission.py", "tray_transfer_plan.py",
                     "raised_tray_transfer.py", "search_tray_mounts.py"):
            result["tool_sha256"][name] = hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
    executor_scope = ExitStack()
    phase_executor = None
    try:
        import omni.kit.app
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
            "isaacsim.asset.importer.urdf", True)
        dt = 1 / 120
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=1/30)
        context = world.get_physics_context()
        context.enable_gpu_dynamics(True)
        context.set_broadphase_type("GPU")
        context.set_solver_type("TGS")
        stage = omni.usd.get_context().get_stage()
        layout = load_layout()
        build(stage, layout)
        geometry = validate_scene_geometry(stage, layout)
        ok, importer = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            raise RuntimeError("URDF import configuration failed")
        importer.set_merge_fixed_joints(True)
        importer.set_fix_base(False)
        importer.set_create_physics_scene(False)
        importer.set_make_default_prim(False)
        importer.set_import_inertia_tensor(True)
        importer.set_self_collision(True)
        importer.set_collision_from_visuals(False)
        importer.set_convex_decomp(True)
        importer.set_parse_mimic(False)
        ok, root = omni.kit.commands.execute("URDFParseAndImportFile",
            urdf_path=str(args.source_dir / "replacement_hypothesis.urdf"),
            import_config=importer, get_articulation_root=True)
        if not ok:
            raise RuntimeError("URDF import failed")
        materials = {}
        for label, coefficient in (("wheel", .9), ("caster", .001), ("grip", .8)):
            material = UsdShade.Material.Define(stage, "/World/"+label+"Material")
            api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            api.CreateStaticFrictionAttr(coefficient)
            api.CreateDynamicFrictionAttr(coefficient)
            api.CreateRestitutionAttr(0.)
            if label == "caster":
                PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
            materials[label] = material
        rigid = {}
        furniture = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.startswith("/Restaurant/"):
                if prim.HasAPI(UsdPhysics.CollisionAPI) and path != "/Restaurant/Floor":
                    furniture.append(path)
                continue
            if prim.IsA(UsdPhysics.RevoluteJoint):
                UsdPhysics.DriveAPI.Get(prim, "angular").GetTypeAttr().Set("force")
            if prim.IsA(UsdPhysics.PrismaticJoint):
                UsdPhysics.DriveAPI.Get(prim, "linear").GetTypeAttr().Set("force")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid[prim.GetName()] = path
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                api.CreateSolverPositionIterationCountAttr(32)
                api.CreateSolverVelocityIterationCountAttr(8)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                kind = "caster" if "caster" in path else "wheel" if "wheel" in path else "grip"
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[kind],
                    UsdShade.Tokens.weakerThanDescendants, "physics")
                api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                api.CreateContactOffsetAttr(.001)
                api.CreateRestOffsetAttr(0.)
        a, b = (rigid[n] for n in bottle_model.FINGER_LINKS)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(Sdf.Path(b))
        fixed_pairs = fixed_component_pairs(args.source_dir/"replacement_hypothesis.urdf", rigid)
        for a, b in fixed_pairs:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(rigid[a])).CreateFilteredPairsRel().AddTarget(Sdf.Path(rigid[b]))
        result["internal_collision_filters"] = {"fixed_assembly_pairs": fixed_pairs,
            "rack_proxy_pair": list(bottle_model.FINGER_LINKS), "other_moving_joint_pairs_excluded": False,
            "external_filters_added": False}
        if args.mode == "collision-probe":
            wall = UsdGeom.Cube.Define(stage, "/World/ProbeObstacle")
            wall.CreateSizeAttr(1.)
            wall.AddTranslateOp().Set(Gf.Vec3d(-1.55, 0, .4))
            wall.AddScaleOp().Set(Gf.Vec3f(.05, 1., .8))
            UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
            furniture.append(str(wall.GetPath()))
        robot = world.scene.add(SingleArticulation(prim_path=root, name="mobile_robot"))
        cup_view = None
        if service:
            cup = stage.DefinePrim("/World/Cup", "Xform")
            cup.GetReferences().AddReference(str(args.source_dir / "scene.usda"), "/World/Cup")
            UsdPhysics.RigidBodyAPI(cup).GetRigidBodyEnabledAttr().Set(True)
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith("/World/Cup") and prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials["grip"],
                        UsdShade.Tokens.strongerThanDescendants, "physics")
            cup_filters = [rigid["left_gripper_link"], rigid["left_moving_jaw_link"],
                "/Restaurant/Kitchen/PourWorktop", "/Restaurant/Dining/table_1/Top",
                "/Restaurant/Floor", *[v for k, v in rigid.items() if k not in
                    {"left_gripper_link", "left_moving_jaw_link"}],
                *[p for p in furniture if p not in {"/Restaurant/Kitchen/PourWorktop", "/Restaurant/Dining/table_1/Top"}]]
            cup_view = world.scene.add(RigidPrim("/World/Cup", name="cup_contacts",
                contact_filter_prim_paths_expr=cup_filters, max_contact_count=4096))
        body_names = sorted(rigid)
        contact = world.scene.add(RigidPrim([rigid[n] for n in body_names],
            name="environment_contacts", contact_filter_prim_paths_expr=[furniture+list(rigid.values()) for _ in body_names],
            max_contact_count=16384))
        ground = world.scene.add(RigidPrim([rigid[n] for n in body_names], name="ground_contacts",
            contact_filter_prim_paths_expr=[["/Restaurant/Floor"] for _ in body_names],
            max_contact_count=4096))
        water_scene = None
        if water_service:
            from water_service_mission import setup_water_scene
            water_scene = setup_water_scene(stage, world, context, source, args.source_dir,
                rigid, furniture, materials, cup_view, cup_filters)
        world.reset()
        names = list(robot.dof_names)
        wheels = np.array([names.index(n) for n in ("left_wheel_joint", "right_wheel_joint")])
        arm_indices = np.array([i for i in range(len(names)) if i not in wheels])
        q = np.zeros(len(names), dtype=np.float32)
        initial = source["plan"]["poses"][0]
        for side in ("left", "right"):
            idx = [names.index(side+"_"+j) for j in ARM_JOINTS]
            q[idx] = np.radians(initial[side+"_joint_deg"])
        left_grip = names.index("left_gripper")
        right_grip = [names.index(n) for n in bottle_model.GRIPPER_JOINTS]
        q[left_grip] = initial["left_gripper_rad"]
        q[right_grip] = initial["right_gripper_m"]
        position, orientation = robot.get_world_pose()
        position[:2] = [0., 0.] if service else [-2., 0.]
        if roundtrip:
            position[:2] = layout['waypoints']['dock'][:2]
            yaw_rad = layout['waypoints']['dock'][2]
            orientation = np.array([math.cos(yaw_rad/2), 0., 0., math.sin(yaw_rad/2)])
        robot.set_world_pose(position, orientation)
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        controller = robot.get_articulation_controller()
        kp, kd = np.full(len(q), 600.), np.full(len(q), 35.)
        kp[wheels], kd[wheels] = 0., 2.
        kp[left_grip], kd[left_grip] = 3., .15
        kp[right_grip] = source["right_config"]["gripper_kp"]
        kd[right_grip] = source["right_config"]["gripper_kd"]
        effort = np.full(len(q), 10.)
        effort[wheels] = .981
        effort[left_grip] = .2
        effort[right_grip] = source["right_config"]["gripper_max_effort_n"]
        controller.set_gains(kps=kp, kds=kd)
        controller.set_max_efforts(effort)
        set_camera_view(np.array([-3.5, -2., 1.9]), np.array([-1.6, 0., .7]))
        stage.GetRootLayer().Export(str(output/"scene.usda"))
        if args.mode == "static-pose-probe":
            from tray_pose_probe import run_probes
            result.update(run_probes(args, world, robot, controller, names, q, wheels, arm_indices,
                body_names, rigid, furniture, contact, ground, output))
            return 0 if result["task_pass"] else 1
        if args.mode == "released-cup-probe":
            from released_cup_probe import run_probe
            result.update(run_probe(args, source, world, robot, controller, names, q, wheels,
                arm_indices, body_names, rigid, furniture, contact, ground, cup_view, cup_filters, output))
            samples = result.pop("samples")
            return 0 if result["task_pass"] else 1
        if water_service:
            from water_service_mission import execute_water_service
            scope = nullcontext(None)
            if getattr(args, 'executor_socket', None):
                from isaac_phase_executor import serve_executor
                scope = serve_executor(args.executor_socket, output,
                    idle_timeout_sec=args.executor_idle_timeout, roundtrip=roundtrip)
            phase_executor = executor_scope.enter_context(scope)
            args._phase_executor = phase_executor
            result.update(execute_water_service(args, source, water_scene, world, robot, controller,
                names, q, wheels, arm_indices, rigid, body_names, furniture, contact, ground, output))
            samples = result.pop("samples")
            return 0 if result["task_pass"] else 1
        path = [[-2., 0.], [-1.5, 0.]]
        state = "PICK" if service else "DRIVE"
        state_start = 2.
        events = []
        goal_yaw = 0.
        if service:
            layout["waypoints"]["carry_start"] = [-.70, 0., 0.]
            route = plan_route(layout, "carry_start", "table_1")
            route[-1] = layout["waypoints"]["table_1"][:2]
        place_window = []
        lift_window = []
        unsupported_grip_s = 0.
        placement_samples = []
        previous_place_phase = None
        stopped_s = 0.
        touchdown_q = None
        async def capture_image(path):
            await capture_viewport_to_file(get_active_viewport(), str(path)).wait_for_result()
        def snapshot(path):
            future = asyncio.ensure_future(capture_image(path))
            deadline = time.monotonic()+30
            while not future.done() and time.monotonic() < deadline:
                world.render()
            if not future.done():
                raise TimeoutError("viewport capture timed out")
            future.result()
        if args.record:
            (output/"frames").mkdir()
        frame_count = 0
        reason = "timeout"
        maximum_force = 0.
        maximum_probe_force = 0.
        cup_sample = {}
        started = time.monotonic()
        for step in range(round(args.duration/dt)):
            position, orientation = robot.get_world_pose()
            w, x, y, z = orientation
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            tilt = math.degrees(math.acos(float(np.clip(1-2*(x*x+y*y), -1, 1))))
            command = follow_path([*position[:2], yaw], path, goal_yaw)
            velocities = [command["left_rad_s"], command["right_rad_s"]] if step*dt > 2 else [0., 0.]
            phase = state
            arm = None
            elapsed = max(0., step*dt-state_start)
            if service:
                if state in {"PICK", "PLACE"}:
                    arm = sample_plan(pick_plan if state == "PICK" else place_plan, elapsed)
                    phase = arm["phase"]
                    for side in ("left", "right"):
                        q[[names.index(side+"_"+j) for j in ARM_JOINTS]] = np.radians(arm[side+"_joint_deg"])
                    q[left_grip] = arm["left_gripper_rad"]
                    q[right_grip] = arm["right_gripper_m"]
                    if touchdown_q is not None and phase in {"LEFT_LOWER", "LEFT_TABLE_SETTLE", "LEFT_OPEN", "LEFT_RELEASE_HOLD"}:
                        q[[names.index("left_"+j) for j in ARM_JOINTS]] = touchdown_q
                    if state == "PLACE" and phase != previous_place_phase:
                        failure = placement_gate_failure(phase.removeprefix("LEFT_"), placement_samples,
                            place_config, bool(lift_window), touchdown_q is not None)
                        if failure:
                            reason = failure
                            break
                        previous_place_phase = phase
                    velocities = [0., 0.]
                if state == "SETTLE_BASE":
                    velocities = [0., 0.]
                    at_rest = np.linalg.norm(robot.get_linear_velocity()) < .005 and np.linalg.norm(robot.get_angular_velocity()) < .01
                    stopped_s = stopped_s+dt if at_rest else 0.
                    if stopped_s >= .5:
                        state, state_start = "PLACE", step*dt
                        events.append({"state": state, "time_s": step*dt})
                if state == "BACKOUT":
                    # Reverse straight from the manipulation zone before global planning.
                    velocities = [-.05/.0329, -.05/.0329]
                if state == "PICK" and arm["done"]:
                    if len(lift_window) < 120 or not all(
                        s["cup_position_m"][2] > .82 and min(s["cup_contacts_n"][:2]) > .02
                        for s in lift_window[-120:]):
                        reason = "lift_contact_not_verified"
                        break
                    if args.mode == "pick-only":
                        reason = "picked"
                        break
                    state, state_start = "BACKOUT", step*dt
                    events.append({"state": state, "time_s": step*dt})
                elif state == "BACKOUT" and position[0] <= -.70:
                    state, state_start, path, goal_yaw = "NAVIGATE", step*dt, route, math.pi
                    events.append({"state": state, "time_s": step*dt})
                elif state == "NAVIGATE" and command["arrived"]:
                    state, state_start, path = "DOCK_TABLE", step*dt, [[-1.75, -2.25]]
                    events.append({"state": state, "time_s": step*dt})
                elif state == "DOCK_TABLE" and command["arrived"]:
                    state, state_start = "SETTLE_BASE", step*dt
                    velocities = [0., 0.]
                    events.append({"state": state, "time_s": step*dt})
            controller.apply_action(ArticulationAction(joint_positions=q[arm_indices], joint_indices=arm_indices))
            controller.apply_action(ArticulationAction(joint_velocities=np.array(velocities), joint_indices=wheels))
            world.step(render=False)
            forces = contact.get_contact_force_matrix(dt=dt)
            ground_forces = np.linalg.norm(ground.get_contact_force_matrix(dt=dt), axis=-1).sum(axis=-1)
            ground_state = dict(zip(body_names, ground_forces.tolist()))
            if ground_collision(ground_state):
                result["unexpected_ground_contacts"] = ground_state
                reason = "ground_collision"
                break
            force_magnitudes = np.linalg.norm(forces, axis=-1)
            peak = float(force_magnitudes[:, :len(furniture)].max())
            self_peak = float(force_magnitudes[:, len(furniture):].max())
            if args.mode == "collision-probe":
                maximum_probe_force = max(maximum_probe_force, float(force_magnitudes[:, furniture.index("/World/ProbeObstacle")].max()))
            maximum_force = max(maximum_force, peak)
            if not np.isfinite([*position, *orientation, peak, self_peak]).all():
                reason = "nonfinite_state"
                break
            if peak > .05 or self_peak > .05:
                result["collision_pairs"] = [[body_names[i], (furniture+list(rigid.values()))[j], float(force_magnitudes[i, j])]
                    for i, j in np.argwhere(force_magnitudes > .05)]
                reason = "environment_collision" if peak > .05 else "robot_self_collision"
                break
            cup_sample = {}
            if cup_view is not None:
                cp, cq = cup_view.get_world_poses()
                cp, cq = cp[0], cq[0]
                cup_forces = cup_view.get_contact_force_matrix(dt=dt)[0]
                cf = np.linalg.norm(cup_forces, axis=-1)
                table_support = float(cup_forces[3, 2])
                cup_tilt = math.degrees(math.acos(float(np.clip(quaternion_matrix(cq)[2, 2], -1, 1))))
                cup_sample = {"cup_position_m": cp.tolist(), "cup_tilt_deg": cup_tilt,
                              "cup_contacts_n": cf.tolist(), "phase": phase, "state": state}
                actual = robot.get_joint_positions()
                transforms = fk.transforms(dict(zip(names, actual)))
                tcp = quaternion_matrix(orientation) @ transforms["left_contact_center"][:3, 3]+position
                left_arm_idx = [names.index("left_"+j) for j in ARM_JOINTS]
                cup_sample.update(table_support_force_n=table_support, contact_force_n=cf[:2].tolist(),
                    contact_center_error_m=float(np.linalg.norm(tcp-cp)),
                    gripper_actual_rad=float(actual[left_grip]),
                    arm_error_rad=float(np.max(np.abs(actual[left_arm_idx]-q[left_arm_idx]))))
                if phase == "LEFT_LIFT_HOLD":
                    lift_window.append(cup_sample)
                if state == "PICK" and phase in {"LEFT_RESET", "LEFT_REORIENT_ABOVE", "LEFT_PREGRASP_ABOVE", "LEFT_ALIGN_MIDDLE", "LEFT_APPROACH"}:
                    if max(cf[:2]) > .02 or np.linalg.norm(cp-np.array(source["left_config"]["cup_center_m"])) > .003:
                        reason = "premature_cup_contact"
                        break
                minimum_support = place_config["cup_mass_kg"]*place_config["placement"]["gravity_m_s2"]*place_config["placement"]["minimum_support_weight_ratio"]
                if state == "PLACE" and phase == "LEFT_LOWER" and table_support > minimum_support and touchdown_q is None:
                    touchdown_q = robot.get_joint_positions()[[names.index("left_"+j) for j in ARM_JOINTS]].copy()
                if state == "PLACE" and phase == "LEFT_PLACE_HOLD":
                    place_window.append(cup_sample)
                if state == "PLACE":
                    placement_samples.append({**cup_sample, "phase": phase.removeprefix("LEFT_")})
                if cp[2] < .75 or cup_tilt > 15 or max(cf[4:]) > .05:
                    reason = "cup_fall_tilt_or_nonfinger_contact"
                    break
                if state in {"BACKOUT", "NAVIGATE", "DOCK_TABLE", "SETTLE_BASE"} and (cp[2] < .80 or max(cf[2:]) > .05):
                    reason = "cup_transport_contact_or_drop"
                    break
                if state in {"BACKOUT", "NAVIGATE", "DOCK_TABLE", "SETTLE_BASE"}:
                    unsupported_grip_s = unsupported_grip_s+dt if min(cf[:2]) < .02 else 0.
                    expected = quaternion_matrix(orientation) @ np.array([.39, .17, .84])+position
                    if unsupported_grip_s > .2 or np.linalg.norm(cp-expected) > .02:
                        reason = "cup_grasp_lost"
                        break
                if state == "PLACE" and arm is not None and arm["done"]:
                    supported = supported_window(placement_samples, place_config, "PLACE_HOLD", 1., released=True, clear=True)
                    on_table = -3.20 < cp[0] < -2.10 and -2.55 < cp[1] < -1.95 and abs(cp[2]-.78) < .006
                    reason = "served" if supported and on_table else "placement_not_verified"
                    break
            if step % 12 == 0:
                samples.append({"time_s": step*dt, "position_m": position.tolist(), "yaw_rad": yaw,
                    "tilt_deg": tilt, "environment_contact_force_n": peak, "wheel_target_rad_s": velocities,
                    "self_contact_force_n": self_peak,
                    "wheel_actual_rad_s": robot.get_joint_velocities()[wheels].tolist()})
                samples[-1]["ground_force_n"] = ground_state
                samples[-1].update(cup_sample)
            if step % 60 == 0:
                world.render()
                if args.record:
                    focus = np.array([position[0], position[1], .65])
                    set_camera_view(focus+np.array([-1.5, -1.8, 1.2]), focus)
                    snapshot(output/"frames"/f"{frame_count:06d}.png")
                    frame_count += 1
            if step % 600 == 0:
                print(json.dumps(samples[-1]), flush=True)
            if not np.isfinite([*position, *orientation, peak]).all():
                reason = "nonfinite_state"
                break
            if tilt > 10:
                reason = "base_tilt"
                break
            if peak > .05:
                reason = "environment_collision"
                break
            if self_peak > .05:
                reason = "robot_self_collision"
                break
            if not service and step*dt > 2 and command["arrived"]:
                reason = "arrived"
                break
        controller.apply_action(ArticulationAction(joint_velocities=np.zeros(2), joint_indices=wheels))
        position, orientation = robot.get_world_pose()
        w, x, y, z = orientation
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        stopped_samples = samples[20:]
        ground_verified = ground_window_verified(stopped_samples)
        result.update(task_pass=(reason == "arrived" and args.mode == "drive-smoke") or reason in {"served", "picked"},
            negative_control_pass=args.mode == "collision-probe" and reason == "environment_collision" and maximum_probe_force > .05,
            maximum_probe_force_n=maximum_probe_force,
            failure=reason, maximum_environment_force_n=maximum_force, final_position_m=position.tolist(),
            final_yaw_rad=yaw, elapsed_wall_s=time.monotonic()-started,
            simulated_s=step*dt, furniture_collider_count=len(geometry), dof_names=names,
            events=events, final_state=state, cup_final=cup_sample if service else None,
            ground_verified=ground_verified, frame_count=frame_count, recording_fps=2,
            assumptions={"wheel_radius_m": .0329, "wheel_track_m": .510,
                         "caster_model": "low_friction_fixed_spheres", "model_calibrated": False})
        result["task_pass"] = result["task_pass"] and ground_verified
        if args.record:
            snapshot(output/"final.png")
    except Exception as exc:
        import traceback
        traceback.print_exc()
        record_failure(result, exc)
    finally:
        recorder = getattr(args, '_cinematic_recorder', None)
        if recorder is not None:
            try:
                recorder.close()
            except Exception as exc:
                record_failure(result, exc)
        result["samples"] = samples
        if phase_executor is not None:
            phase_executor.finish(result['task_pass'], result['failure'],
                result.get('final_observation'), persist_result=result)
        else:
            (output/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        executor_scope.close()
        print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)
        # Kit's graceful extension teardown can leave native threads alive.
        # Persist evidence first, then request the appropriate process status.
        import omni.kit.app
        omni.kit.app.get_app().post_uncancellable_quit(
            0 if result["task_pass"] or result.get("negative_control_pass") else 1)
        app.close()
    return 0 if result["task_pass"] or result.get("negative_control_pass") else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--cinematic", action="store_true", help="1080p24 editorial camera capture; no physics changes")
    parser.add_argument("--duration", type=float, default=25.)
    parser.add_argument("--executor-socket", type=Path, help="Gate water-service physics by IPC phase requests")
    parser.add_argument("--executor-idle-timeout", type=float, default=120.)
    parser.add_argument('--dock-roundtrip', action='store_true', help='Drive from dock, serve table_1, then return to dock')
    parser.add_argument("--pose-probes", type=Path)
    parser.add_argument("--mode", choices=("drive-smoke", "collision-probe", "service", "pick-only", "water-service", "static-pose-probe", "released-cup-probe"), default="drive-smoke")
    raise SystemExit(run(parser.parse_args()))
