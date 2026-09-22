#!/usr/bin/env python3
"""Isaac 5.1의 실험용 회전식 강체 패드로 컵 접근·접촉·상승을 검사한다.

실물 접근 없음. 컵 순간이동·부착 없음. 가정 패드의 결과는 FinRay 실물 검증이 아니다.
"""
from __future__ import annotations

import argparse
import asyncio
import copy
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np

import cup_contact_model
from cup_contact_model import (DEFAULT_CONFIG, Chain, evaluate_lift, ready_to_lift, preclose_failure)
from workcell_preview_inputs import ARM_JOINTS
from cup_contact_recovery import retreat_plan, recovery_failure, observe_stationary_cup
from cup_contact_place import evaluate_placement, touchdown_plan, placement_gate_failure, lowering_failure


def run(args, backend=cup_contact_model):
    if args.recover and backend.SIDE != "left":
        raise ValueError("접촉 복구는 아직 왼손 컵 실험만 지원합니다")
    config = backend.load_config(args.config)
    side, unit = backend.SIDE, backend.GRIPPER_UNIT
    object_label = backend.OBJECT_LABEL
    object_path = f"/World/{object_label}"
    if args.cup_mass_kg is not None:
        config["cup_mass_kg"] = args.cup_mass_kg
    if args.spawn_y_offset_mm is not None:
        config["cup_spawn_offset_m"][1] = args.spawn_y_offset_mm / 1000
    if args.spawn_x_offset_mm is not None:
        config["cup_spawn_offset_m"][0] = args.spawn_x_offset_mm / 1000
    cup_contact_model.validate_config(config)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, provenance = backend.prepare_model(output, config)
    plan = backend.make_plan(model, config, place=args.place)
    fk = Chain(model)
    source_config_sha256 = hashlib.sha256(args.config.read_bytes()).hexdigest()
    runtime_overrides = {key: value for key, value in {
        "object_mass_kg": args.cup_mass_kg,
        "spawn_y_offset_mm": args.spawn_y_offset_mm,
        "spawn_x_offset_mm": args.spawn_x_offset_mm,
    }.items() if value is not None}
    manifest = {"config": config, "model": provenance, "plan": plan,
                "tool_sha256": {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in dict.fromkeys(("simulate_cup_contact.py", "cup_contact_model.py", "cup_contact_recovery.py", "cup_contact_place.py",
                                                          "plan_body_side_grasp.py", "workcell_preview_inputs.py", "workcell_preview_motion.py",
                                                          f"simulate_{object_label.lower()}_contact.py", Path(backend.__file__).name))},
                "config_sha256": source_config_sha256,
                "source_config_sha256": source_config_sha256,
                "effective_config_sha256": cup_contact_model.effective_config_sha256(config),
                "runtime_overrides": runtime_overrides,
                "active_side": side, "object_kind": object_label.lower(), "gripper_unit": unit,
                "recovery_enabled": args.recover,
                "placement_enabled": args.place,
                "observation_source": "simulator_ground_truth_not_rgb",
                "hardware_accessed": False, "mode": "rigid_proxy_contact_experiment"}
    (output / "plan.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    if args.plan_only:
        print(f"{object_label.upper()}_CONTACT_PLAN_READY {output / 'plan.json'}")
        return
    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."):
        raise RuntimeError(f"Isaac Sim 5.1 필요: {version}")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 800,
                         "window_width": 1500, "window_height": 980,
                         "renderer": "RayTracedLighting", "anti_aliasing": 0,
                         "multi_gpu": False, "sync_loads": True, "fast_shutdown": True})
    samples = []
    events = []
    attempt = 0
    attempt_start = 0
    evaluation_config = copy.deepcopy(config)
    def evaluate(completed, reason):
        current = samples[attempt_start:]
        outcome = evaluate_lift(current, evaluation_config, completed, reason)
        lift_pass = evaluate_lift(current, evaluation_config, True, reason)["rigid_proxy_lift_pass"]
        touchdown_observed = any(e["type"] == "touchdown" and e["attempt"] == attempt for e in events)
        place_pass = evaluate_placement(current, evaluation_config, completed, lift_pass, touchdown_observed) if args.place else False
        return {**outcome, "object_kind": object_label.lower(), "physical_grasp_verified": False,
                "assembly_mode": provenance.get("assembly_mode", "left_contact_proxy"),
                "model_calibrated": False,
                "lift_stage_pass": lift_pass, "placement_enabled": args.place,
                "touchdown_observed": touchdown_observed,
                "rigid_proxy_place_pass": place_pass,
                "task_pass": place_pass if args.place else outcome["rigid_proxy_lift_pass"]}

    result = evaluate(False, "initializing")

    def save_result():
        temporary = output / "result.json.tmp"
        temporary.write_text(json.dumps({**result, "samples": samples, "events": events,
                                         "retry_count": attempt, "recovery_enabled": args.recover,
                                         "evaluation_center_m": evaluation_config["cup_center_m"],
                                         "observation_source": "simulator_ground_truth_not_rgb"}, indent=2,
                                         allow_nan=False)+"\n")
        temporary.replace(output / "result.json")
    save_result()
    try:
        import carb
        import omni.kit.app
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

        carb.settings.get_settings().set("/app/window/title", f"HOLD THE FLOW | {object_label} contact experiment")
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        dt_s = config["physics_dt_s"]
        world = World(stage_units_in_meters=1.0, physics_dt=dt_s, rendering_dt=1/30)
        world.get_physics_context().set_solver_type("TGS")
        world.get_physics_context().set_gravity(-config["placement"]["gravity_m_s2"])
        stage = omni.usd.get_context().get_stage()
        ok, importer = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            raise RuntimeError("URDF import config 실패")
        # 질량 없는 TCP 좌표 프레임을 독립 동적 바디로 만들지 않는다.
        importer.set_merge_fixed_joints(True)
        importer.set_fix_base(True)
        importer.set_make_default_prim(False)
        importer.set_create_physics_scene(False)
        importer.set_import_inertia_tensor(True)
        importer.set_self_collision(False)
        importer.set_collision_from_visuals(False)
        importer.set_convex_decomp(config["mesh_collision_mode"] == "convexDecomposition")
        importer.set_parse_mimic(False)
        ok, root = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(model),
                                            import_config=importer, get_articulation_root=True)
        if not ok:
            raise RuntimeError("URDF import 실패")
        material = UsdShade.Material.Define(stage, "/World/GripMaterial")
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(config["friction"])
        physics_material.CreateDynamicFrictionAttr(config["friction"])
        physics_material.CreateRestitutionAttr(0)
        rigid_links = {}
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                UsdPhysics.DriveAPI.Get(prim, "angular").GetTypeAttr().Set("force")
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                UsdPhysics.DriveAPI.Get(prim, "linear").GetTypeAttr().Set("force")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_links[prim.GetName()] = str(prim.GetPath())
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateSolverPositionIterationCountAttr(32)
                body.CreateSolverVelocityIterationCountAttr(4)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
                collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collision.CreateContactOffsetAttr(.001)
                collision.CreateRestOffsetAttr(0)

        def box(path, center_m, size_m, color):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center_m))
            shape.AddScaleOp().Set(Gf.Vec3f(*size_m))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            return shape
        box("/World/Floor", [0, 0, -.025], [4, 4, .05], [.15,.18,.22])
        box("/World/Table", config["table_center_m"], config["table_size_m"], [.62,.64,.67])
        cup = UsdGeom.Cylinder.Define(stage, object_path)
        cup.CreateAxisAttr("Z")
        cup.CreateRadiusAttr(config["cup_radius_m"])
        cup.CreateHeightAttr(config["cup_height_m"])
        spawn_position_m = np.array(config["cup_center_m"])+config["cup_spawn_offset_m"]
        cup.AddTranslateOp().Set(Gf.Vec3d(*spawn_position_m))
        cup.CreateDisplayColorAttr([Gf.Vec3f(.95,.64,.12)])
        UsdPhysics.CollisionAPI.Apply(cup.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
        UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr(config["cup_mass_kg"])
        UsdShade.MaterialBindingAPI.Apply(cup.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(cup.GetPrim())
        body.CreateSolverPositionIterationCountAttr(32)
        body.CreateSolverVelocityIterationCountAttr(4)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(cup.GetPrim())
        collision.CreateContactOffsetAttr(.001)
        collision.CreateRestOffsetAttr(0)
        sun = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
        sun.CreateIntensityAttr(1600)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-30,-45,0))
        UsdLux.DomeLight.Define(stage, "/World/Fill").CreateIntensityAttr(500)
        robot = world.scene.add(SingleArticulation(prim_path=root, name="robot"))
        other_links = sorted(set(rigid_links)-set(backend.FINGER_LINKS)) if side == "right" else []
        filters = [*(rigid_links[name] for name in backend.FINGER_LINKS), "/World/Table",
                   *(rigid_links[name] for name in other_links)]
        cup_view = world.scene.add(RigidPrim(object_path, name="object_contacts",
            contact_filter_prim_paths_expr=filters, max_contact_count=128))
        world.reset()
        names = list(robot.dof_names)
        active_indices = [names.index(side+"_"+j) for j in ARM_JOINTS]
        parked_side = "right" if side == "left" else "left"
        parked_indices = [names.index(parked_side+"_"+j) for j in ARM_JOINTS]
        gripper_indices = [names.index(name) for name in backend.GRIPPER_JOINTS]
        q = np.zeros(len(names), dtype=np.float32)
        q[active_indices] = np.radians(plan["poses"][0]["joint_deg"])
        q[parked_indices] = np.radians(plan["parked_joint_deg"])
        q[names.index("left_gripper")] = 1.
        for name in ("right_finger1_joint", "right_finger2_joint"):
            q[names.index(name)] = .0433
        q[gripper_indices] = config[f"gripper_open_{unit}"]
        # 초기 배치만 순간 설정한다. 이후는 물리 드라이브 목표만 보낸다.
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.set_joints_default_state(positions=q.copy(), velocities=np.zeros_like(q))
        control = robot.get_articulation_controller()
        kp, kd = np.full(len(q), 600.), np.full(len(q), 35.)
        kp[gripper_indices], kd[gripper_indices] = config.get("gripper_kp", 3.), config.get("gripper_kd", .15)
        control.set_gains(kps=kp, kds=kd)
        max_efforts = np.full(len(q), 10.)
        max_efforts[gripper_indices] = config["gripper_max_effort_nm" if unit == "rad" else "gripper_max_effort_n"]
        control.set_max_efforts(max_efforts)
        control.apply_action(ArticulationAction(joint_positions=q))

        label = None
        if not args.headless:
            import omni.ui as ui
            panel = ui.Window(f"{object_label.upper()} CONTACT - RIGID PROXY", width=430, height=240)
            with panel.frame:
                with ui.VStack():
                    ui.Label(f"Automatic {object_label.lower()} approach / close / lift")
                    ui.Label("IK, no IL dataset | hardware OFF")
                    ui.Label("GRAVITY + CONTACTS ON | no object attachment")
                    ui.Label(f"Recovery: {args.recover} | simulator coordinates, NOT RGB")
                    ui.Label(f"Place on table and release: {args.place}")
                    ui.Label(config["description"], word_wrap=True)
                    label = ui.Label("Initializing", word_wrap=True)
            panel.dock_in_window("Stage", ui.DockPosition.SAME)
            panel.set_active(True)

        async def capture(path):
            await capture_viewport_to_file(get_active_viewport(), str(path)).wait_for_result()

        def snapshot(path):
            future = asyncio.ensure_future(capture(path))
            deadline_s = time.monotonic()+30
            while not future.done() and time.monotonic() < deadline_s:
                world.render()
            if not future.done():
                future.cancel()
                raise TimeoutError("렌더 캡처 실패")
            future.result()
            while time.monotonic() < deadline_s:
                if path.exists() and path.read_bytes()[-8:-4] == b"IEND":
                    return
                world.render()
            raise TimeoutError("PNG 저장 실패")

        for _ in range(60):
            control.apply_action(ArticulationAction(joint_positions=q))
            world.step(render=False)
        print(f"SETTLED_TARGET {q.tolist()} ACTUAL {robot.get_joint_positions().tolist()}", flush=True)
        set_camera_view(np.array([1.1, 1.02 if side == "left" else -1.02, 1.05]),
                        np.array([.25, .07 if side == "left" else -.07, .84]))
        for _ in range(12):
            world.render()
        snapshot(output / "initial.png")
        observed_start_m = cup_view.get_world_poses()[0][0].copy()
        (output / "frames").mkdir()
        stage.GetRootLayer().Export(str(output / "scene.usda"))
        if label is not None:
            panel.dock_in_window("Stage", ui.DockPosition.SAME)
            panel.set_active(True)
        print(f"{object_label.upper()}_CONTACT_READY {output}", flush=True)
        elapsed_s = 0.
        plan_elapsed_s = 0.
        active_plan = plan
        recovering = False
        attempt_reference_m = observed_start_m.copy()
        index = 0
        phase = None
        reason = "window_closed"
        completed = False
        while app.is_running():
            command = backend.sample_plan(active_plan, plan_elapsed_s)
            if command["phase"] != phase:
                if command["phase"] == "LIFT" and not ready_to_lift(samples[attempt_start:], evaluation_config):
                    reason = "sustained_midbody_bilateral_contact_not_observed"
                    break
                if args.place:
                    gates = evaluate(True, "placement_gate")
                    failure = placement_gate_failure(command["phase"], samples[attempt_start:], evaluation_config,
                                                     gates["lift_stage_pass"], gates["touchdown_observed"])
                    if failure:
                        reason = failure
                        break
                phase = command["phase"]
                result = evaluate(False, f"running:{phase}")
                save_result()
                print(f"{object_label.upper()}_PHASE {phase}", flush=True)
            q[active_indices] = np.radians(command["joint_deg"])
            q[gripper_indices] = command[backend.GRIPPER_KEY]
            control.apply_action(ArticulationAction(joint_positions=q))
            before_s = world.current_time
            world.step(render=False)
            if not np.isclose(world.current_time-before_s, dt_s, atol=1e-6):
                raise RuntimeError("물리 시간과 제어 시간 불일치")
            if index % 4 == 0:
                world.render()
            positions, orientations = cup_view.get_world_poses()
            position, orientation = positions[0], orientations[0]
            cup_tilt_deg = float(np.degrees(np.arccos(np.clip(
                1-2*(orientation[1]**2+orientation[2]**2), -1., 1.))))
            force_vectors = cup_view.get_contact_force_matrix(dt=dt_s)[0]
            force = np.linalg.norm(force_vectors[:2], axis=1)
            table_force_n = float(force_vectors[2, 2])
            other_forces = dict(zip(other_links, np.linalg.norm(force_vectors[3:], axis=1).tolist()))
            actual_q = robot.get_joint_positions()
            error_rad = float(np.max(np.abs(actual_q[active_indices]-q[active_indices])))
            contact_center_m = fk.transforms(dict(zip(names, actual_q)))[backend.CONTACT_FRAME][:3, 3]
            if not np.isfinite([*position, *force, *other_forces.values(), table_force_n, *actual_q, *contact_center_m, cup_tilt_deg]).all():
                reason = "nonfinite_physics_state"
                break
            sample = {"time_s": elapsed_s, "phase": phase, "attempt": attempt, "cup_position_m": position.tolist(),
                      "contact_force_n": force.tolist(), "arm_error_rad": error_rad,
                      "table_contact_force_world_n": force_vectors[2].tolist(),
                      "table_support_force_n": table_force_n,
                      f"gripper_actual_{unit}": float(np.min(actual_q[gripper_indices])),
                      f"gripper_positions_{unit}": actual_q[gripper_indices].tolist(),
                      f"{side}_joint_actual_rad": actual_q[active_indices].tolist(),
                      "object_kind": object_label.lower(), "object_position_m": position.tolist(),
                      "non_gripping_robot_contact_n": other_forces,
                      "contact_center_m": contact_center_m.tolist(),
                      "midbody_height_error_m": float(contact_center_m[2]-position[2]),
                      "contact_center_error_m": float(np.linalg.norm(contact_center_m-position)),
                      "cup_tilt_deg": cup_tilt_deg,
                      "cup_displacement_from_start_m": float(np.linalg.norm(position[:2]-observed_start_m[:2])),
                      "cup_displacement_from_observation_m": float(np.linalg.norm(position[:2]-attempt_reference_m[:2])),
                      "physics_time_s": world.current_time}
            samples.append(sample)
            if label is not None:
                label.text = f"{phase} | retry {attempt}\n{object_label} rise: {(position[2]-config['cup_center_m'][2])*1000:.1f} mm\nContact: {force.round(2)} N"
            if index % 12 == 0 and args.record:
                snapshot(output / "frames" / f"frame_{index//12:04d}.png")
            if error_rad > config["maximum_tracking_error_rad"]:
                reason = "arm_tracking_error"
                break
            if side == "right":
                failure = backend.non_gripping_contact_failure(other_forces, config)
                if failure:
                    reason = failure
                    break
            failure = lowering_failure(sample, config)
            if failure:
                reason = failure
                break
            if phase in {"WITHDRAW", "CLEAR_ABOVE", "PLACE_HOLD"} and max(force) >= config["minimum_contact_force_n"]:
                reason = "post_release_hand_contact"
                break
            if recovering:
                failure = recovery_failure(sample, observed_start_m, config)
                if failure:
                    reason = failure
                    break
                if command["done"]:
                    try:
                        center = observe_stationary_cup(samples, config)
                        evaluation_config["cup_center_m"] = center
                        next_plan = backend.make_plan(model, evaluation_config, np.degrees(actual_q[active_indices]), place=args.place)
                    except ValueError as exc:
                        reason = f"recovery_replan_rejected:{exc}"
                        break
                    attempt += 1
                    attempt_start = len(samples)
                    attempt_reference_m = np.asarray(center)
                    events.append({"type": "replanned", "time_s": elapsed_s, "attempt": attempt,
                                   "observation_source": "simulator_ground_truth_not_rgb",
                                   "observed_center_m": center, "plan": next_plan})
                    active_plan, recovering = next_plan, False
                    plan_elapsed_s = 0.
                    phase = None
                    elapsed_s += dt_s
                    index += 1
                    continue
            else:
                check = {**sample, "cup_displacement_from_start_m": sample["cup_displacement_from_observation_m"]}
                early_failure = preclose_failure(check, config)
                if early_failure:
                    events.append({"type": "preclose_failure", "time_s": elapsed_s, "attempt": attempt,
                                   "failure_code": early_failure, "sample_index": len(samples)-1})
                    if not args.recover or attempt >= config["recovery"]["max_retries"]:
                        reason = early_failure if not args.recover else "recovery_retry_limit"
                        break
                    failure = recovery_failure(sample, observed_start_m, config)
                    if failure:
                        reason = failure
                        break
                    try:
                        active_plan = retreat_plan(samples[attempt_start:], config)
                    except ValueError:
                        reason = "recovery_no_retreat_history"
                        break
                    recovering = True
                    events.append({"type": "retreat_started", "time_s": elapsed_s, "attempt": attempt,
                                   "plan": active_plan})
                    plan_elapsed_s = 0.
                    phase = None
                    elapsed_s += dt_s
                    index += 1
                    continue
            if phase == "LOWER" and table_force_n >= (
                    config["cup_mass_kg"]*config["placement"]["gravity_m_s2"]
                    *config["placement"]["minimum_support_weight_ratio"]):
                active_plan = touchdown_plan(active_plan, actual_q[active_indices])
                events.append({"type": "touchdown", "time_s": elapsed_s, "attempt": attempt,
                               "table_support_force_n": table_force_n,
                               "cup_position_m": position.tolist(), "plan": active_plan})
                plan_elapsed_s = 0.
                phase = None
                elapsed_s += dt_s
                index += 1
                continue
            if command["done"]:
                completed, reason = True, "sequence_finished"
                break
            elapsed_s += dt_s
            plan_elapsed_s += dt_s
            index += 1
        result = evaluate(completed, reason)
        save_result()
        snapshot(output / "final.png")
        print(f"{object_label.upper()}_CONTACT_RESULT {json.dumps(result)}", flush=True)
        if label is not None:
            label.text = f"{reason} | FROZEN RESULT\nTask PASS: {result['task_pass']} | retry {attempt}\nLift: {result['lift_stage_pass']} | place: {result['rigid_proxy_place_pass']}\nNOT physical gripper validation"
        if args.stay_open:
            while app.is_running():
                world.render()
                time.sleep(.01)
    except BaseException as exc:
        traceback.print_exc()
        result = evaluate(False, f"exception:{type(exc).__name__}")
        save_result()
        app._app.post_quit(1)
        raise
    finally:
        if not result["task_pass"]:
            app._app.post_quit(2)
        app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--cup-mass-kg", type=float, help="질량 민감도 시험용 가정값")
    parser.add_argument("--spawn-y-offset-mm", type=float, help="계획 좌표를 유지한 채 초기 물체 위치만 오차 주입")
    parser.add_argument("--spawn-x-offset-mm", type=float, help="초기 물체 전후 위치 오차")
    parser.add_argument("--recover", action="store_true", help="시뮬레이터 정답 좌표로 제한된 접촉 복구 시험")
    parser.add_argument("--place", action="store_true", help="상승 검증 뒤 같은 책상에 내려놓고 손 분리까지 검사")
    args = parser.parse_args()
    if args.headless and args.stay_open:
        parser.error("headless + stay-open은 허용하지 않습니다")
    if args.cup_mass_kg is not None and (not math.isfinite(args.cup_mass_kg) or args.cup_mass_kg <= 0):
        parser.error("cup-mass-kg은 유한한 양수여야 합니다")
    if args.spawn_y_offset_mm is not None and not math.isfinite(args.spawn_y_offset_mm):
        parser.error("spawn-y-offset-mm는 유한한 숫자여야 합니다")
    if args.spawn_x_offset_mm is not None and not math.isfinite(args.spawn_x_offset_mm):
        parser.error("spawn-x-offset-mm는 유한한 숫자여야 합니다")
    run(args)
