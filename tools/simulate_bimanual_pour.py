#!/usr/bin/env python3
"""양팔 파지와 PBD 유체 붓기 실험. 미보정 가정 장면이며 실기체 접근 없음."""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import cup_contact_model as cup_model
import bottle_contact_model as bottle_model
from pour_geometry import initial_liquid, liquid_counts, quaternion_matrix, evaluate_pour, minimum_jaw_gap, load_experiment, mount_clearance_evidence, inward_pour_direction, preclose_violation
from workcell_preview_inputs import ARM_JOINTS


def run(args):
    from bimanual_pour_plan import build_plan, sample_plan
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    left = cup_model.load_config()
    right = bottle_model.load_config(cup_model.ROOT / "config/simulation/bottle_replacement_experiment.json")
    experiment_path = cup_model.ROOT / "config/simulation/bimanual_pour_experiment.json"
    experiment = load_experiment(experiment_path)
    if not np.isclose(sum(experiment[k] for k in ("bottle_body_height_m","bottle_shoulder_height_m","bottle_neck_height_m")),right["cup_height_m"]):
        raise ValueError("병 전체 높이와 몸통/어깨/목 높이 합이 다름")
    if not experiment["wall_m"] < experiment["bottle_neck_outer_radius_m"] < right["cup_radius_m"]:
        raise ValueError("병목 반경은 벽 두께보다 크고 몸통 반경보다 작아야 함")
    right["assembly_hypothesis"]["mount_xyz_m"][2] -= experiment["additional_outward_mount_m"]
    right["container_profile"] = {k:experiment[k] for k in ("wall_m","bottle_body_height_m","bottle_neck_height_m","bottle_neck_outer_radius_m")}
    left["container_wall_m"] = experiment["wall_m"]
    left["cup_radius_profile_m"] = experiment["cup_radius_profile_m"]
    left["approach"] = {"reorient_backoff_m":.07,"reorient_height_m":.13,
                        "pregrasp_backoff_m":experiment["cup_pregrasp_backoff_m"],"pregrasp_height_m":.08}
    cup_profile = np.asarray(left["cup_radius_profile_m"])
    if not np.allclose(cup_profile[[0,-1],0], [-left["cup_height_m"]/2,left["cup_height_m"]/2]):
        raise ValueError("컵 높이와 반경 프로파일 높이 불일치")
    if not np.isclose(np.interp(0,cup_profile[:,0],cup_profile[:,1]),left["cup_radius_m"]):
        raise ValueError("파지 중심 반경은 기존 IK 파지 반경과 같아야 함")
    left_model, left_source = cup_model.prepare_model(output, left)
    model, right_source = bottle_model.prepare_model(output, right, source=left_model, source_is_materialized=True)
    if args.settle_only:
        lp = cup_model.make_plan(model,left)["poses"][0]
        rp = bottle_model.make_plan(model,right)["poses"][0]
        plan = {"poses":[{"name":"DIAGNOSTIC_SETTLE","duration_s":3.,
                "left_joint_deg":lp["joint_deg"],"right_joint_deg":rp["joint_deg"],
                "left_gripper_rad":1.,"right_gripper_m":right["gripper_open_m"]}]}
    else:
        plan = build_plan(model, left, right)
    dt = left["physics_dt_s"]
    spacing_m = experiment["particle_spacing_m"]
    particle_mass_kg = spacing_m**3*experiment["fluid_density_kg_m3"]
    initial = initial_liquid(np.asarray(right["cup_center_m"]), right["cup_radius_m"], right["cup_height_m"],
                            spacing=spacing_m,fill_height=experiment["initial_fill_height_m"],wall=experiment["wall_m"])
    files = ["simulate_bimanual_pour.py", "bimanual_pour_plan.py", "pour_geometry.py", "cup_contact_model.py",
             "bottle_contact_model.py", "plan_body_side_grasp.py", "workcell_preview_inputs.py",
             "audit_bottle_target_geometry.py", "cup_contact_place.py"]
    manifest = {"plan": plan, "left_config": left, "right_config": right,
                "experiment":experiment,"experiment_sha256":hashlib.sha256(experiment_path.read_bytes()).hexdigest(),
                "model": right_source, "left_model": left_source,
                "mount_clearance":mount_clearance_evidence(model),
                "tool_sha256": {f: hashlib.sha256(Path(__file__).with_name(f).read_bytes()).hexdigest() for f in files},
                "ik_source_sha256":hashlib.sha256((cup_model.ROOT / "src/hold_flow_description/scripts/solve_task_poses.py").read_bytes()).hexdigest(),
                "fluid": {"solver": "PhysX_PBD_fluid", "spacing_m": spacing_m, "particle_mass_kg": particle_mass_kg,
                          "initial_count": len(initial), "initial_volume_ml": len(initial)*spacing_m**3*1e6,
                          "initialization_only": True, "water_material": "upstream_water_preset_unvalidated"},
                "container": {**right["container_profile"],"bottle_shoulder_height_m":experiment["bottle_shoulder_height_m"],
                              "status":"nominal_compound_vessels_not_measured"},
                "hardware_accessed": False, "model_calibrated": False, "self_collision_enabled": True,
                "diagnostic_settle_only": args.settle_only,
                "object_attachment_used": False, "observation_source": "simulator_ground_truth_not_rgb"}
    (output / "plan.json").write_text(json.dumps(manifest, indent=2)+"\n")
    if args.plan_only:
        return
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "width":1280, "height":900,
                         "renderer":"RayTracedLighting", "anti_aliasing":0,
                         "multi_gpu":False, "fast_shutdown":True})
    samples, events = [], []
    completed, reason = False, "initialization"
    try:
        import carb
        import omni.kit.app
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema
        from omni.physx.scripts import particleUtils, physicsUtils
        from omni.physx.bindings import _physx as pb
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=1/30)
        context = world.get_physics_context()
        context.enable_gpu_dynamics(True)
        context.set_broadphase_type("GPU")
        context.set_solver_type("TGS")
        stage = omni.usd.get_context().get_stage()
        settings = carb.settings.get_settings()
        settings.set(pb.SETTING_UPDATE_TO_USD, True)
        settings.set(pb.SETTING_UPDATE_PARTICLES_TO_USD, True)
        ok, importer = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            raise RuntimeError("URDF import config 실패")
        importer.set_merge_fixed_joints(True)
        importer.set_fix_base(True)
        importer.set_create_physics_scene(False)
        importer.set_make_default_prim(False)
        importer.set_import_inertia_tensor(True)
        importer.set_self_collision(True)
        importer.set_collision_from_visuals(False)
        importer.set_convex_decomp(True)
        importer.set_parse_mimic(False)
        ok, root = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(model), import_config=importer, get_articulation_root=True)
        if not ok:
            raise RuntimeError("URDF import 실패")
        material = UsdShade.Material.Define(stage, "/World/GripMaterial")
        friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        friction.CreateStaticFrictionAttr(.8)
        friction.CreateDynamicFrictionAttr(.8)
        friction.CreateRestitutionAttr(0.)
        right_material = UsdShade.Material.Define(stage, "/World/RightGripMaterial")
        right_friction = UsdPhysics.MaterialAPI.Apply(right_material.GetPrim())
        right_friction.CreateStaticFrictionAttr(right["friction"])
        right_friction.CreateDynamicFrictionAttr(right["friction"])
        right_friction.CreateRestitutionAttr(0.)
        rigid_links = {}
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                UsdPhysics.DriveAPI.Get(prim,"angular").GetTypeAttr().Set("force")
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                UsdPhysics.DriveAPI.Get(prim,"linear").GetTypeAttr().Set("force")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_links[prim.GetName()] = str(prim.GetPath())
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateSolverPositionIterationCountAttr(32)
                body.CreateSolverVelocityIterationCountAttr(4)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                selected = right_material if "right_" in str(prim.GetPath()) else material
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(selected, UsdShade.Tokens.weakerThanDescendants, "physics")
                collider = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collider.CreateContactOffsetAttr(.001)
                collider.CreateRestOffsetAttr(0.)

        filtered_pairs = set()
        # 랙은 같은 평면의 박스 프록시라 맞물림을 모사하지 않는다. 블레이드 간격은 관절 한계로 양수다.
        filtered_pairs.add(tuple(sorted(rigid_links[n] for n in bottle_model.FINGER_LINKS)))
        for a,b in sorted(filtered_pairs):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(Sdf.Path(b))
        manifest["internal_collision_filters"] = {"pairs":sorted(filtered_pairs),
            "reason":"nominal_interlocking_rack_pair_only",
            "right_blade_minimum_gap_m":minimum_jaw_gap(model),
            "rack_gear_contact_simulated":False,"external_object_filters_added":False}
        (output/"plan.json").write_text(json.dumps(manifest,indent=2)+"\n")

        def box(path, center, size, color, angle=0.):
            shape = UsdGeom.Cube.Define(stage,path)
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
            shape.AddRotateZOp().Set(float(angle))
            shape.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            col = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
            col.CreateContactOffsetAttr(.001)
            col.CreateRestOffsetAttr(0.)
            selected = right_material if "/Bottle/" in path else material
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(selected,UsdShade.Tokens.weakerThanDescendants,"physics")
            return shape

        box("/World/Floor",[0,0,-.025],[4,4,.05],[.15,.18,.22])
        box("/World/Table",left["table_center_m"],left["table_size_m"],[.55,.57,.59])
        def container(name, config, color):
            path = "/World/"+name
            body = UsdGeom.Xform.Define(stage,path)
            body.AddTranslateOp().Set(Gf.Vec3d(*config["cup_center_m"]))
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(config["cup_mass_kg"])
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
            api.CreateSolverPositionIterationCountAttr(32)
            api.CreateSolverVelocityIterationCountAttr(4)
            radius, height, wall, count = config["cup_radius_m"],config["cup_height_m"],experiment["wall_m"],32
            base = UsdGeom.Cylinder.Define(stage,path+"/Bottom")
            base.CreateRadiusAttr(cup_profile[0,1] if name == "Cup" else radius)
            base.CreateHeightAttr(wall)
            base.AddTranslateOp().Set(Gf.Vec3d(0,0,-height/2+wall/2))
            base.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(base.GetPrim())
            body_height = experiment["bottle_body_height_m"] if name == "Bottle" else height
            for i in range(count):
                angle = 2*np.pi*i/count
                if name == "Cup":
                    for band, (lower, upper) in enumerate(zip(cup_profile,cup_profile[1:])):
                        points = []
                        for z, outer_radius in (lower, upper):
                            for r in (outer_radius-wall, outer_radius):
                                for edge in (-1,1):
                                    a = angle+edge*np.pi/count*1.02
                                    points.append(Gf.Vec3f(float(r*np.cos(a)),float(r*np.sin(a)),float(z)))
                        segment = UsdGeom.Mesh.Define(stage,path+f"/Wall{i}_Band{band}")
                        segment.CreatePointsAttr(points)
                        segment.CreateFaceVertexCountsAttr([4]*6)
                        segment.CreateFaceVertexIndicesAttr([0,1,3,2,4,6,7,5,0,4,5,1,2,3,7,6,0,2,6,4,1,5,7,3])
                        segment.CreateSubdivisionSchemeAttr("none")
                        segment.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                        UsdPhysics.CollisionAPI.Apply(segment.GetPrim())
                        UsdPhysics.MeshCollisionAPI.Apply(segment.GetPrim()).CreateApproximationAttr("convexHull")
                        collision = PhysxSchema.PhysxCollisionAPI.Apply(segment.GetPrim())
                        collision.CreateContactOffsetAttr(.001)
                        collision.CreateRestOffsetAttr(0.)
                        UsdShade.MaterialBindingAPI.Apply(segment.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,"physics")
                    continue
                center = [(radius-wall/2)*np.cos(angle),(radius-wall/2)*np.sin(angle),(body_height-height)/2]
                box(path+f"/Wall{i}",center,[wall,2*radius*np.tan(np.pi/count)*1.03,body_height],color,np.degrees(angle))
                if name == "Bottle":
                    neck_radius,neck_height = experiment["bottle_neck_outer_radius_m"],experiment["bottle_neck_height_m"]
                    box(path+f"/Neck{i}",[(neck_radius-wall/2)*np.cos(angle),(neck_radius-wall/2)*np.sin(angle),height/2-neck_height/2],
                        [wall,2*neck_radius*np.tan(np.pi/count)*1.03,neck_height],color,np.degrees(angle))
                    points = []
                    for z,radius_at_z in ((-height/2+body_height,radius),(height/2-neck_height,neck_radius)):
                        for radius_at_face in (radius_at_z-wall,radius_at_z):
                            for edge in (-1,1):
                                a = angle+edge*np.pi/count*1.02
                                points.append(Gf.Vec3f(radius_at_face*np.cos(a),radius_at_face*np.sin(a),z))
                    shoulder = UsdGeom.Mesh.Define(stage,path+f"/Shoulder{i}")
                    shoulder.CreatePointsAttr(points)
                    shoulder.CreateFaceVertexCountsAttr([4]*6)
                    shoulder.CreateFaceVertexIndicesAttr([0,1,3,2,4,6,7,5,0,4,5,1,2,3,7,6,0,2,6,4,1,5,7,3])
                    shoulder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                    UsdPhysics.CollisionAPI.Apply(shoulder.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(shoulder.GetPrim()).CreateApproximationAttr("convexHull")
                    shoulder_collision = PhysxSchema.PhysxCollisionAPI.Apply(shoulder.GetPrim())
                    shoulder_collision.CreateContactOffsetAttr(.001)
                    shoulder_collision.CreateRestOffsetAttr(0.)
                    UsdShade.MaterialBindingAPI.Apply(shoulder.GetPrim()).Bind(right_material,UsdShade.Tokens.weakerThanDescendants,"physics")
            return path
        cup_path = container("Cup",left,[.8,.9,.95])
        bottle_path = container("Bottle",right,[.92,.65,.15])
        particle_system = particleUtils.add_physx_particle_system(stage,Sdf.Path("/World/LiquidSystem"),
            simulation_owner=context.prim_path, contact_offset=.0015, rest_offset=.001,
            particle_contact_offset=spacing_m/1.2, solid_rest_offset=spacing_m/2, fluid_rest_offset=spacing_m/2,
            solver_position_iterations=12, enable_ccd=True, max_velocity=5.)
        liquid_material = UsdShade.Material.Define(stage,"/World/LiquidMaterial")
        particleUtils.AddPBDMaterialWater(liquid_material.GetPrim())
        physicsUtils.add_physics_material_to_prim(stage,particle_system.GetPrim(),liquid_material.GetPath())
        fluid = particleUtils.add_physx_particleset_points(stage,"/World/Liquid", initial.tolist(),
            [[0.,0.,0.]]*len(initial),[spacing_m]*len(initial),particle_system.GetPath(),True,True,0,particle_mass_kg,0.)
        fluid.CreateDisplayColorAttr([Gf.Vec3f(.02,.35,.95)])
        robot = world.scene.add(SingleArticulation(prim_path=root,name="robot"))
        objects = {}
        other_links = {}
        for name,path,backend in (("cup",cup_path,cup_model),("bottle",bottle_path,bottle_model)):
            other_links[name] = sorted(set(rigid_links)-set(backend.FINGER_LINKS))
            filters = [*(rigid_links[n] for n in backend.FINGER_LINKS),"/World/Table",
                       *(rigid_links[n] for n in other_links[name]),
                       bottle_path if name == "cup" else cup_path]
            objects[name] = world.scene.add(RigidPrim(path,name=name+"_contacts",contact_filter_prim_paths_expr=filters,max_contact_count=1024))
        robot_body_names = sorted(rigid_links)
        body_contacts = world.scene.add(RigidPrim([rigid_links[n] for n in robot_body_names],name="robot_table_contacts",
            contact_filter_prim_paths_expr=[["/World/Table",*(rigid_links[n] for n in robot_body_names)] for _ in robot_body_names],
            max_contact_count=4096))
        UsdLux.DomeLight.Define(stage,"/World/Fill").CreateIntensityAttr(800)
        UsdLux.DistantLight.Define(stage,"/World/Key").CreateIntensityAttr(1600)
        world.reset()
        masses = {name: float(view.get_masses()[0]) for name, view in objects.items()}
        (output/"runtime.json").write_text(json.dumps({"rigid_masses_kg":masses,
            "gpu_dynamics":context.is_gpu_dynamics_enabled(),"physics_dt_s":dt},indent=2)+"\n")
        names = list(robot.dof_names)
        indices = {side:[names.index(side+"_"+j) for j in ARM_JOINTS] for side in ("left","right")}
        left_grip = names.index("left_gripper")
        right_grip = [names.index(n) for n in bottle_model.GRIPPER_JOINTS]
        q = np.zeros(len(names),dtype=np.float32)
        def command_to_q(command):
            for side in indices:
                q[indices[side]] = np.radians(command[side+"_joint_deg"])
            q[left_grip] = command["left_gripper_rad"]
            q[right_grip] = command["right_gripper_m"]
        command_to_q(sample_plan(plan,0))
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.set_joints_default_state(positions=q.copy(),velocities=np.zeros_like(q))
        control = robot.get_articulation_controller()
        kp,kd = np.full(len(q),600.),np.full(len(q),35.)
        kp[left_grip],kd[left_grip] = 3.,.15
        kp[right_grip],kd[right_grip] = right["gripper_kp"],right["gripper_kd"]
        effort = np.full(len(q),10.)
        effort[left_grip] = left["gripper_max_effort_nm"]
        effort[right_grip] = right["gripper_max_effort_n"]
        control.set_gains(kps=kp,kds=kd)
        control.set_max_efforts(effort)
        set_camera_view(np.array([1.25,.95,1.35]),np.array([.3,0,.87]))
        async def capture(path):
            await capture_viewport_to_file(get_active_viewport(),str(path)).wait_for_result()
        def snapshot(path):
            future = asyncio.ensure_future(capture(path))
            deadline = time.monotonic()+30
            while not future.done() and time.monotonic()<deadline:
                world.render()
            if not future.done():
                future.cancel()
                raise TimeoutError("PNG capture")
            future.result()
        for _ in range(120):
            control.apply_action(ArticulationAction(joint_positions=q))
            world.step(render=False)
        world.render()
        snapshot(output/"initial.png")
        stage.GetRootLayer().Export(str(output/"scene.usda"))
        (output/"frames").mkdir()
        phase, freezes, grasp_references = None, {}, {}
        contact_loss_ticks = {name:0 for name in objects}
        fk = cup_model.Chain(model)
        for tick in range(int(180/dt)):
            command = sample_plan(plan,tick*dt)
            if command["phase"] != phase:
                if command["phase"] in {"LEFT_LIFT", "RIGHT_LIFT"}:
                    kind = "cup" if command["phase"] == "LEFT_LIFT" else "bottle"
                    window = samples[-60:]
                    if len(window) < 60 or any(min(s[kind+"_hand_n"]) < .02 for s in window):
                        reason = "bilateral_contact_missing_before_lift:"+kind
                        break
                phase = command["phase"]
                print("POUR_PHASE",phase,flush=True)
                events.append({"phase":phase,"time_s":tick*dt})
                snapshot(output/(phase+".png"))
            command_to_q(command)
            for side,frozen in freezes.items():
                if phase in {side.upper()+"_LOWER",side.upper()+"_TABLE_SETTLE",side.upper()+"_OPEN",side.upper()+"_RELEASE_HOLD"}:
                    q[indices[side]] = frozen
            control.apply_action(ArticulationAction(joint_positions=q))
            before = world.current_time
            world.step(render=False)
            if not np.isclose(world.current_time-before,dt,atol=1e-6):
                raise RuntimeError("물리 tick 불일치")
            if tick%12 == 0:
                world.render()
            poses = {name:tuple(a[0] for a in view.get_world_poses()) for name,view in objects.items()}
            particles = np.array(fluid.GetPointsAttr().Get())
            counts = liquid_counts(particles,poses["cup"],poses["bottle"],left,right)
            actual = robot.get_joint_positions()
            if not np.isfinite(actual).all():
                reason = "nonfinite_joint_state"
                break
            transforms = fk.transforms(dict(zip(names,actual)))
            tracking = max(float(np.max(np.abs(actual[ids]-q[ids]))) for ids in indices.values())
            robot_contacts = np.linalg.norm(body_contacts.get_contact_force_matrix(dt=dt),axis=2)
            robot_table = robot_contacts[:,0]
            sample = {"time_s":tick*dt,"physics_time_s":world.current_time,"phase":phase,"liquid":counts,"tracking_error_rad":tracking,
                      "left_gripper_actual_rad":float(actual[left_grip]),"right_gripper_actual_m":actual[right_grip].tolist(),
                      "robot_table_contact_n":dict(zip(robot_body_names,robot_table.tolist()))}
            sample["robot_self_contact_max_n"] = float(robot_contacts[:,1:].max(initial=0))
            sample["robot_self_contact_pairs_n"] = [[robot_body_names[i],robot_body_names[j],float(robot_contacts[i,j+1])]
                for i,j in np.argwhere(robot_contacts[:,1:] > .001)]
            for name,side,config in (("cup","left",left),("bottle","right",right)):
                force = objects[name].get_contact_force_matrix(dt=dt)[0]
                if not np.isfinite(force).all():
                    raise ValueError("nonfinite_contact_state")
                sample[name+"_position_m"] = poses[name][0].tolist()
                sample[name+"_orientation_wxyz"] = poses[name][1].tolist()
                sample[name+"_hand_n"] = np.linalg.norm(force[:2],axis=1).tolist()
                sample[name+"_support_n"] = float(force[2,2])
                sample[name+"_non_gripping_max_n"] = float(np.linalg.norm(force[3:-1],axis=1).max(initial=0))
                sample[name+"_non_gripping_links_n"] = dict(zip(other_links[name],np.linalg.norm(force[3:-1],axis=1).tolist()))
                sample[name+"_other_container_contact_n"] = float(np.linalg.norm(force[-1]))
                sample[name+"_rise_m"] = float(poses[name][0][2]-config["cup_center_m"][2])
                sample[name+"_tilt_deg"] = float(np.degrees(np.arccos(np.clip(quaternion_matrix(poses[name][1])[2,2],-1,1))))
                object_transform = np.eye(4)
                object_transform[:3,:3] = quaternion_matrix(poses[name][1])
                object_transform[:3,3] = poses[name][0]
                relative = np.linalg.inv(transforms[side+"_contact_center"]) @ object_transform
                if phase == side.upper()+"_CONTACT_HOLD" and name not in grasp_references:
                    grasp_references[name] = relative
                holding = name in grasp_references and side not in freezes
                sample[name+"_holding_expected"] = holding
                if holding:
                    reference = grasp_references[name]
                    sample[name+"_grasp_slip_m"] = float(np.linalg.norm(relative[:3,3]-reference[:3,3]))
                    sample[name+"_grasp_rotation_error_deg"] = float(np.degrees(np.arccos(np.clip((np.trace(reference[:3,:3].T @ relative[:3,:3])-1)/2,-1,1))))
                sample[side+"_joint_actual_rad"] = actual[indices[side]].tolist()
                if phase == side.upper()+"_LOWER" and force[2,2] > config["cup_mass_kg"]*9.81*.4 and side not in freezes:
                    freezes[side] = actual[indices[side]].copy()
                    events.append({"type":"touchdown","side":side,"time_s":tick*dt})
            samples.append(sample)
            early_failure=preclose_violation(sample,samples[0])
            if early_failure:
                reason=early_failure
                break
            bottle_mouth = poses["bottle"][0]+quaternion_matrix(poses["bottle"][1]) @ [0,0,right["cup_height_m"]/2]
            cup_rim = poses["cup"][0]+quaternion_matrix(poses["cup"][1]) @ [0,0,left["cup_height_m"]/2]
            sample["mouth_relative_to_cup_rim_m"] = (bottle_mouth-cup_rim).tolist()
            if (phase.startswith("POUR_TILT") or phase in {"POUR_HOLD", "POUR_RETURN"}) and sample["bottle_tilt_deg"] >= 15:
                if not inward_pour_direction(sample):
                    reason = "bottle_pour_direction_outside_inward_corridor"
                    break
            if tick%120 == 0:
                (output/"live.json").write_text(json.dumps(sample,indent=2)+"\n")
            if args.record and tick%12 == 0:
                snapshot(output/"frames"/f"frame_{tick//12:05d}.png")
            if tracking > .15:
                reason = "arm_tracking_error"
                break
            if np.min(actual[right_grip]) < .0095 or np.max(actual[right_grip]) > .0438:
                reason = "right_gripper_outside_nominal_joint_limits"
                break
            if not np.isfinite(robot_contacts).all() or max(robot_table,default=0) > .02:
                reason = "robot_table_collision_or_nonfinite"
                break
            if sample["robot_self_contact_max_n"] > .02:
                reason = "robot_self_collision"
                break
            lost = [name for name in objects if sample[name+"_holding_expected"] and (
                sample[name+"_grasp_slip_m"] > .02 or sample[name+"_grasp_rotation_error_deg"] > 15)]
            if lost:
                reason = "grasp_relative_pose_lost:"+",".join(lost)
                break
            for name in objects:
                contact_loss_ticks[name] = contact_loss_ticks[name]+1 if (
                    sample[name+"_holding_expected"] and min(sample[name+"_hand_n"]) < .02) else 0
            if any(count*dt >= .1 for count in contact_loss_ticks.values()):
                reason = "sustained_grasp_contact_loss"
                break
            if max(sample["cup_non_gripping_max_n"],sample["bottle_non_gripping_max_n"]) > .02:
                reason = "non_gripping_robot_contact"
                break
            if sample["cup_tilt_deg"] > 20:
                reason = "cup_tilt"
                break
            if max(sample["cup_other_container_contact_n"],sample["bottle_other_container_contact_n"]) > .1:
                reason = "container_collision"
                break
            if command["done"]:
                completed,reason = True,"sequence_finished"
                break
        else:
            reason = "time_limit"
        snapshot(output/"final.png")
        np.save(output/"final_liquid_positions.npy",particles)
    except Exception as exc:
        completed = False
        reason = f"exception:{type(exc).__name__}:{exc}"
        raise
    finally:
        result = {**evaluate_pour(samples,len(initial),completed), "stop_reason":reason,
                  "completed":completed,"hardware_accessed":False,"object_attachment_used":False,
                  "samples":samples,"events":events}
        (output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print("POUR_RESULT",json.dumps({k:v for k,v in result.items() if k not in {"samples","events"}}),flush=True)
        diagnostic_ok = args.settle_only and completed and samples[-1]["liquid"]["bottle"] >= .95*len(initial)
        exit_code = 0 if result["task_pass"] or diagnostic_ok else 2
        # Kit fast shutdown may terminate before Python reaches the return below.
        omni.kit.app.get_app().post_quit(exit_code)
        app.close()
    if not result["task_pass"]:
        if not args.settle_only or not completed or samples[-1]["liquid"]["bottle"] < .95*len(initial):
            raise SystemExit(2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--plan-only",action="store_true")
    parser.add_argument("--record",action="store_true")
    parser.add_argument("--settle-only",action="store_true",help="정지 장면의 유체 유지 진단만 실행; 태스크 성공으로 세지 않음")
    run(parser.parse_args())
