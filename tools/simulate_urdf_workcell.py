#!/usr/bin/env python3
"""Isaac Sim 5.1에서 전체 URDF와 가정 작업 환경의 저장 자세를 재생한다.

실물 포트/ROS/카메라를 열지 않는다. 고정 베이스·중력 제외·접촉 제외 자세 재생이며
동역학, 파지, 센서 보정 또는 실물 안전 검증이 아니다. 기존 6.0 importer와 분리한다.
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import time
import traceback

import numpy as np

from workcell_preview_inputs import load_replay, materialize_urdf, sample_pose, _number_vector
from workcell_envelopes import WorkcellEnvelopes
from workcell_preview_motion import build_demo, sample_demo, audit_demo
from workcell_recording import PreviewRecording

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "src/hold_flow_description/urdf/hold_flow.urdf"
DEFAULT_BASE_CONTRACT = REPO_ROOT / "config/navigation/jdamr_migration.json"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# 모두 장면 이해를 위한 가정이며 실측 캘리브레이션으로 사용하지 않는다.
SCENE_ASSUMPTIONS = {
    "status": "illustrative_only_not_measured",
    "table_surface_z_m": 0.6931,
    "table_center_xy_m": [0.55, 0.0],
    "table_size_xy_m": [0.60, 1.0],
    "cup_center_xy_m": [0.32, 0.17],
    "cup_radius_m": 0.035,
    "cup_height_m": 0.12,
    "bottle_center_xy_m": [0.39, -0.17],
    "bottle_radius_m": 0.038,
    "bottle_height_m": 0.22,
    "right_parked_joint_deg": [0.0, -68.0, 92.0, -22.0, 0.0],
    "left_stock_gripper_rad": 0.5,
    "right_gripper_open_m": 0.0325,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--body-plan", type=Path, help="양팔 몸통 중간 수평 파지 후보 보고서")
    parser.add_argument("--state", type=Path, action="append", default=[])
    parser.add_argument("--original-base", action="store_true",
                        help="현행 하단 기록 보정 없이 기존 URDF 바퀴/캐스터 배치를 표시")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="새 출력 디렉터리. 기존 증거를 덮어쓰지 않습니다")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="GUI 재생 시간. 0이면 창을 닫을 때까지 유지")
    parser.add_argument("--segment-seconds", type=float, default=4.0)
    parser.add_argument("--demo", action="store_true", help="리셋·양팔 빈손 방향 시연을 한 번 실행")
    parser.add_argument("--record-demo", action="store_true", help="--demo 시연 viewport를 10fps PNG로 기록")
    args = parser.parse_args()
    if args.body_plan and (args.plan or args.state):
        parser.error("body-plan은 과거 plan/state 갤러리와 함께 사용하지 않습니다")
    if args.record_demo and not args.demo:
        parser.error("record-demo는 demo와 함께 사용합니다")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("seconds는 유한한 0 이상의 값이어야 합니다")
    if not math.isfinite(args.segment_seconds) or args.segment_seconds <= 0:
        parser.error("segment-seconds는 양의 유한한 값이어야 합니다")
    if args.headless and args.seconds == 0 and not args.check_only:
        parser.error("headless 실행은 --seconds로 종료 시간을 지정해야 합니다")
    return args


def run(args: argparse.Namespace) -> None:
    replay = load_replay(args.plan, args.state)
    scene = dict(SCENE_ASSUMPTIONS)
    if args.body_plan:
        body_path = args.body_plan.resolve(strict=True)
        body_bytes = body_path.read_bytes()
        body = json.loads(body_bytes.decode("utf-8"))
        if body.get("schema") != "body_side_grasp_v1" or body.get("motion_command_emitted") is not False:
            raise ValueError("몸통 측면 파지 비동작 보고서가 아닙니다")
        if body.get("urdf_sha256") != hashlib.sha256(args.urdf.read_bytes()).hexdigest():
            raise ValueError("몸통 파지 계획과 현재 URDF 해시가 다릅니다. 계획을 다시 생성하세요")
        if not isinstance(body.get("scene"), dict) or not isinstance(body.get("sides"), dict):
            raise ValueError("몸통 파지 보고서 scene/sides는 객체여야 합니다")
        scene.update(body["scene"])
        replay = {"poses": [], "source_files": [{"path": str(body_path),
                  "sha256": hashlib.sha256(body_bytes).hexdigest()}],
                  "limitations": ["양팔 수평 파지 후보 검토이며 닫기·들어올리기는 미검증"]}
        for side in ("left", "right"):
            if not isinstance(body["sides"].get(side), dict) or not isinstance(body["sides"][side].get("stages"), list):
                raise ValueError("몸통 파지 보고서에 양팔 stages 배열이 필요합니다")
            for candidate in body["sides"][side]["stages"]:
                if not isinstance(candidate, dict) or not isinstance(candidate.get("name"), str) or not candidate["name"]:
                    raise ValueError("몸통 파지 stage 객체에 이름이 필요합니다")
                if "joint_deg" not in candidate:
                    continue
                if type(candidate.get("accepted")) is not bool:
                    raise ValueError("몸통 파지 후보 accepted는 bool이어야 합니다")
                _number_vector(candidate["joint_deg"], "몸통 파지 관절 후보")
                replay["poses"].append({"name": f"{side}_{candidate['name']}",
                    "left_joint_deg": candidate["joint_deg"] if side == "left" else [0,-68,92,-22,0],
                    "right_joint_deg": candidate["joint_deg"] if side == "right" else [0,-68,92,-22,0],
                    "ik_accepted": candidate["accepted"], "source_kind": "body_side_grasp_candidate"})
        if not replay["poses"]:
            raise ValueError("몸통 파지 보고서에 표시할 관절 후보가 없습니다")
    checker = WorkcellEnvelopes(args.urdf, scene)
    _number_vector(scene["right_parked_joint_deg"], "오른팔 대기 자세")
    demo_poses = build_demo(checker.chain)
    home = demo_poses[0]
    scene["right_parked_joint_deg"] = home["right_joint_deg"]
    if args.body_plan:
        for pose in replay["poses"]:
            inactive = "right" if pose["name"].startswith("left_") else "left"
            pose[f"{inactive}_joint_deg"] = home[f"{inactive}_joint_deg"]
    source_hashes = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                     for name in ["simulate_urdf_workcell.py", "workcell_preview_inputs.py", "workcell_envelopes.py",
                                  "workcell_preview_motion.py", "workcell_recording.py"]}
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"새 출력 디렉터리를 지정하세요: {output}")
    if args.check_only:
        print(json.dumps({"replay": replay, "scene": scene,
                          "reset_pose": home,
                          "hardware_accessed": False}, ensure_ascii=False, indent=2))
        return
    version = importlib.metadata.version("isaacsim")
    if not version.startswith("5.1."):
        raise RuntimeError(f"이 실행기는 Isaac Sim 5.1용입니다. 설치 버전: {version}")
    output.mkdir(parents=True)
    prepared_urdf = output / "hold_flow_preview.urdf"
    provenance = materialize_urdf(args.urdf.resolve(), prepared_urdf,
                                 base_contract=None if args.original_base else DEFAULT_BASE_CONTRACT)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 800,
                         "window_width": 1500, "window_height": 980,
                         "renderer": "RayTracedLighting", "anti_aliasing": 0,
                         "multi_gpu": False, "sync_loads": True, "fast_shutdown": True})
    try:
        import carb
        import omni.kit.app
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.core.utils.types import ArticulationAction
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

        carb.settings.get_settings().set("/app/window/title", "HOLD THE FLOW | offline URDF replay")
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            raise RuntimeError("URDF importer config 생성 실패")
        config.set_merge_fixed_joints(False)
        config.set_fix_base(True)
        config.set_make_default_prim(False)
        config.set_create_physics_scene(False)
        config.set_import_inertia_tensor(True)
        config.set_self_collision(False)
        config.set_collision_from_visuals(False)
        config.set_parse_mimic(False)
        config.set_distance_scale(1.0)
        ok, root_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=str(prepared_urdf),
            import_config=config, get_articulation_root=True)
        if not ok or not root_path or not stage.GetPrimAtPath(root_path).IsValid():
            raise RuntimeError(f"URDF import 실패: {root_path}")

        disabled_collisions = 0
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                disabled_collisions += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)

        def cube(path: str, center, scale, color) -> None:
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.0)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.AddScaleOp().Set(Gf.Vec3f(*scale))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            shape.GetPrim().SetCustomDataByKey("geometry_status", "assumed_visual_only")

        def cylinder(path: str, center, radius, height, color, opacity=1.0) -> None:
            shape = UsdGeom.Cylinder.Define(stage, path)
            shape.CreateAxisAttr("Z")
            shape.CreateRadiusAttr(radius)
            shape.CreateHeightAttr(height)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            shape.CreateDisplayOpacityAttr([opacity])
            shape.GetPrim().SetCustomDataByKey("geometry_status", "assumed_visual_only")

        UsdGeom.Xform.Define(stage, "/World/Workcell")
        cube("/World/Workcell/Floor", [0, 0, -0.025], [4, 4, 0.05], [0.22, 0.25, 0.29])
        table_z = scene["table_surface_z_m"]
        table_x, table_y = scene["table_center_xy_m"]
        table_dx, table_dy = scene["table_size_xy_m"]
        cube("/World/Workcell/Table", [table_x, table_y, table_z - 0.015],
             [table_dx, table_dy, .03], [.63, .68, .74])
        for i, x in enumerate([table_x-table_dx/2+.05, table_x+table_dx/2-.05]):
            for j, y in enumerate([table_y-table_dy/2+.07, table_y+table_dy/2-.07]):
                cube(f"/World/Workcell/TableLeg_{i}_{j}", [x, y, (table_z-.03)/2],
                     [.035, .035, table_z-.03], [.25, .28, .32])
        cup_x, cup_y = scene["cup_center_xy_m"]
        cup_r, cup_h = scene["cup_radius_m"], scene["cup_height_m"]
        cylinder("/World/Workcell/Cup", [cup_x,cup_y,table_z+cup_h/2], cup_r,cup_h,[.2,.78,.9],.45)
        cylinder("/World/Workcell/CupRim", [cup_x,cup_y,table_z+cup_h], cup_r+.001,.006,[.55,.95,1])
        bottle_x, bottle_y = scene["bottle_center_xy_m"]
        bottle_r, bottle_h = scene["bottle_radius_m"], scene["bottle_height_m"]
        cylinder("/World/Workcell/Bottle", [bottle_x,bottle_y,table_z+bottle_h/2], bottle_r,bottle_h,[.2,.65,.40],.8)
        cylinder("/World/Workcell/BottleNeck", [bottle_x,bottle_y,table_z+bottle_h+.018], .018,.035,[.2,.65,.40])
        cylinder("/World/Workcell/BottleCap", [bottle_x,bottle_y,table_z+bottle_h+.04], .02,.012,[.1,.2,.13])
        for name, x, y, radius, height in (("Cup",cup_x,cup_y,cup_r,cup_h),
                                           ("Bottle",bottle_x,bottle_y,bottle_r,bottle_h)):
            cylinder(f"/World/Workcell/{name}MidHeight", [x,y,table_z+height/2],
                     radius+.002,.003,[1,.25,.05], .8)
        # 위치 참고 선반. 하중 지지 구조·기존 실물 설치를 뜻하지 않는다.
        cube("/World/Workcell/ShelfProxy", [0,0,.50], [.25,.23,.012],[.12,.38,.50])
        sun = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
        sun.CreateIntensityAttr(1200)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-30,-45,0))
        UsdLux.DomeLight.Define(stage, "/World/FillLight").CreateIntensityAttr(350)

        robot = world.scene.add(SingleArticulation(prim_path=root_path, name="hold_flow"))
        world.reset()
        robot.disable_gravity()
        dof_names = list(robot.dof_names)
        required = [f"{side}_{joint}" for side in ("left", "right") for joint in ARM_JOINTS]
        required += ["left_gripper", "right_finger1_joint", "right_finger2_joint"]
        missing = set(required) - set(dof_names)
        if missing:
            raise RuntimeError(f"import된 DOF 누락: {sorted(missing)}")
        indices = [dof_names.index(f"left_{joint}") for joint in ARM_JOINTS]
        right_indices = [dof_names.index(f"right_{joint}") for joint in ARM_JOINTS]
        q = np.zeros(len(dof_names), dtype=np.float32)
        for name, deg in zip(ARM_JOINTS, scene["right_parked_joint_deg"]):
            q[dof_names.index(f"right_{name}")] = np.deg2rad(deg)
        left_open = 1.0 if args.body_plan else scene["left_stock_gripper_rad"]
        right_open = .0433 if args.body_plan else scene["right_gripper_open_m"]
        q[dof_names.index("left_gripper")] = left_open
        for name in ["right_finger1_joint", "right_finger2_joint"]:
            q[dof_names.index(name)] = right_open

        def apply_pose(degrees, right_degrees=None, accepted=True):
            right_degrees = scene["right_parked_joint_deg"] if right_degrees is None else right_degrees
            check = checker.check(degrees, right_degrees, left_open, right_open)
            if not accepted or not check["clear"]:
                robot.set_joint_positions(q)
                robot.set_joint_velocities(np.zeros_like(q))
                return {"applied": False, "ik_accepted": accepted, **check}
            q[indices] = np.radians(degrees)
            q[right_indices] = np.radians(right_degrees)
            robot.set_joint_positions(q)
            robot.set_joint_velocities(np.zeros_like(q))
            robot.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=q))
            return {"applied": True, "ik_accepted": True, **check}

        initial = apply_pose(home["left_joint_deg"], home["right_joint_deg"])
        if not initial["applied"]:
            raise RuntimeError(f"초기 대기 자세도 프리뷰 외곽 검사 미통과: {initial}")
        home_q = q.copy()
        robot.set_joints_default_state(positions=home_q, velocities=np.zeros_like(q), efforts=np.zeros_like(q))
        world.reset()
        default_reset_error = float(np.max(np.abs(robot.get_joint_positions()-home_q)))
        if not math.isfinite(default_reset_error) or default_reset_error > 1e-4:
            raise RuntimeError(f"Isaac world.reset 기본 관절값 불일치: {default_reset_error}")
        demo_audit = audit_demo(demo_poses, checker, left_open, right_open)
        if getattr(args, "demo", False) and not demo_audit["passes"]:
            raise RuntimeError(f"빈손 시연 경로 외곽 검사 실패: {demo_audit['failures'][:1]}")

        views = {
            "Overview": ([-1.6, 1.9, 1.50], [.22, 0, .55]),
            "Front": ([1.85, .01, 1.0], [.15,0,.60]),
            "Top": ([.25,.001,2.0],[.25,0,.60]),
            "Hands": ([1.15,1.05,1.17],[.21,0,.87]),
        }
        def view(name):
            eye, target = views[name]
            set_camera_view(np.array(eye), np.array(target))

        for _ in range(4):
            app.update()
        view("Overview")
        replay_state = {"playing": False, "selected": None, "elapsed": 0.0,
                        "demo": False, "demo_paused": False, "demo_elapsed": 0.0,
                        "demo_completed": False}
        recording_enabled = bool(getattr(args, "record_demo", False))
        recording: PreviewRecording | None = None

        def finish_recording(reason: str, *, completed: bool = False,
                             final_reset_error_rad: float | None = None) -> None:
            nonlocal recording
            if recording is not None:
                recording.finish(completed=completed, reason=reason,
                                 final_reset_error_rad=final_reset_error_rad)
                recording = None

        def reset_preview(reason="user_reset", finalize_recording=True):
            if finalize_recording:
                finish_recording(reason)
            replay_state.update(playing=False, selected=None, elapsed=0.0,
                                demo=False, demo_paused=False, demo_elapsed=0.0, demo_completed=False)
            robot.post_reset()
            return apply_pose(home["left_joint_deg"], home["right_joint_deg"])

        def start_demo():
            nonlocal recording
            finish_recording("demo_restarted")
            reset_preview(finalize_recording=False)
            if demo_audit["passes"]:
                replay_state["demo"] = True
                if recording_enabled:
                    recording = PreviewRecording(output)

        def next_pose():
            finish_recording("candidate_selected")
            selected = replay_state["selected"]
            replay_state.update(playing=False, demo=False,
                                selected=0 if selected is None else (selected+1) % len(replay["poses"]))
        label = None
        panel = None
        if not args.headless:
            import omni.ui as ui
            panel = ui.Window("HOLD THE FLOW - PREVIEW ONLY", width=440, height=480)
            with panel.frame:
                with ui.VStack(spacing=5):
                    ui.Label("450 x 340 mm | SO101 x 2", height=22)
                    ui.Label("BASE: original legacy" if args.original_base else
                             "BASE: 510 mm track / rear caster proxies", height=20)
                    ui.Label("KINEMATIC REPLAY: base fixed / contacts OFF", height=22)
                    ui.Label("MID-BODY SIDE GRASP" if args.body_plan else "LEGACY POSES / OVERLAP REVIEW", height=20)
                    ui.Label("Red bands = body midpoint / no grasp success claim", height=20)
                    ui.Label("Object/table/other-arm envelope overlap -> HOLD", height=20)
                    ui.Label("FinRay NOT modeled; stock left jaw proxy", height=20)
                    ui.Label("Depth/LiDAR are design placeholders, NOT RGB3 calibration", word_wrap=True, height=38)
                    ui.Label("Table, cup, bottle, shelf: ASSUMED dimensions", height=22)
                    ui.Label("DEMO: empty-hand orientation ONLY, NOT cup pick", height=22)
                    label = ui.Label("Loading saved poses...", word_wrap=True, height=40)
                    def pause():
                        if replay_state["demo"]:
                            replay_state["demo_paused"] = not replay_state["demo_paused"]
                        elif not args.body_plan:
                            replay_state["playing"] = not replay_state["playing"]
                    with ui.HStack(height=28):
                        ui.Button("Reset", clicked_fn=reset_preview)
                        ui.Button("Empty-hand check", clicked_fn=start_demo)
                    with ui.HStack(height=28):
                        ui.Button("Play / Pause", clicked_fn=pause)
                        ui.Button("Next pose", clicked_fn=next_pose)
                    with ui.HStack(height=28):
                        for name in views:
                            ui.Button(name, clicked_fn=lambda name=name: view(name))

        async def capture(path: Path):
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("활성 viewport 없음: 이미지 캡처 불가")
            await capture_viewport_to_file(viewport, str(path)).wait_for_result()

        def capture_sync(path: Path):
            future = asyncio.ensure_future(capture(path))
            deadline = time.monotonic()+30
            while not future.done():
                if time.monotonic() > deadline:
                    future.cancel()
                    raise TimeoutError("스크린샷 저장 제한시간 초과")
                app.update()
            future.result()
            # capture 완료와 PNG 비동기 디스크 쓰기 완료는 다른 시점일 수 있다.
            while time.monotonic() < deadline:
                if path.is_file():
                    data = path.read_bytes()
                    if data.startswith(b"\x89PNG\r\n\x1a\n") and data[-8:-4] == b"IEND":
                        return
                app.update()
                time.sleep(.02)
            raise RuntimeError(f"완전한 PNG가 생성되지 않았습니다: {path}")

        evidence = []
        for _ in range(8):
            world.step(render=True)
        view("Overview")
        for _ in range(4):
            app.update()
        capture_sync(output / "reset.png")
        last_applied_pose_name = "RESET"
        for index, pose in enumerate(replay["poses"]):
            result = apply_pose(pose["left_joint_deg"], pose.get("right_joint_deg"), pose.get("ik_accepted", True))
            for _ in range(12):
                world.step(render=True)
                apply_pose(pose["left_joint_deg"], pose.get("right_joint_deg"), pose.get("ik_accepted", True))
            snapshot = output / f"pose_{index:02d}.png"
            capture_sync(snapshot)
            observed = robot.get_joint_positions()
            displayed_error = float(np.max(np.abs(observed-q)))
            if not math.isfinite(displayed_error) or displayed_error > 1e-4:
                raise RuntimeError(f"표시/유지 자세 불일치: {displayed_error} rad")
            left_error = float(np.max(np.abs(observed[indices]-np.radians(pose["left_joint_deg"]))))
            right_error = float(np.max(np.abs(observed[right_indices]-np.radians(
                pose.get("right_joint_deg", scene["right_parked_joint_deg"])))))
            max_error = max(left_error, right_error)
            if result["applied"] and (not math.isfinite(max_error) or max_error > 1e-4):
                raise RuntimeError(f"시뮬레이터 관절값 불일치: {max_error} rad")
            if result["applied"]:
                last_applied_pose_name = pose["name"]
            evidence.append({"pose": pose["name"], "image": snapshot.name,
                             "candidate_check": result,
                             "image_semantics": "candidate" if result["applied"] else "held_pose_not_failed_candidate",
                             "displayed_pose_name": last_applied_pose_name,
                             "displayed_state_error_rad": displayed_error,
                             "per_arm_pose_assignment_error_rad": {"left": left_error, "right": right_error} if result["applied"] else None,
                             "max_pose_assignment_error_rad": max_error if result["applied"] else None})
            print(f"HOLD_FLOW_POSE_REVIEWED {pose['name']} applied={result['applied']}", flush=True)
        reset_preview(reason="startup_gallery_finished")
        world.step(render=True)
        scene_path = output / "workcell.usda"
        if not stage.GetRootLayer().Export(str(scene_path)):
            raise RuntimeError("USD 장면 저장 실패")
        reopened = Usd.Stage.Open(str(scene_path))
        required_prims = [root_path, "/World/Workcell/Table", "/World/Workcell/Cup", "/World/Workcell/Bottle"]
        if reopened is None or not all(reopened.GetPrimAtPath(path).IsValid() for path in required_prims):
            raise RuntimeError("저장한 USD에서 로봇/작업환경을 다시 읽지 못했습니다")
        manifest = {
            "mode": "isaac_5_1_fixed_base_kinematic_preview",
            "isaac_version": version, "hardware_accessed": False,
            "tool_sha256": source_hashes,
            "arguments": {key: str(value) if isinstance(value, Path) else
                          [str(item) for item in value] if isinstance(value, list) else value
                          for key, value in vars(args).items()},
            "motion_command_emitted": False, "source_urdf": provenance,
            "replay": replay, "scene_assumptions": scene,
            "gripper_display_positions": {"left_gripper_rad": left_open, "right_fingers_m": right_open},
            "reset_pose": home, "world_reset_error_rad": default_reset_error,
            "startup_selection": "RESET_not_first_accepted_candidate",
            "empty_hand_demo": {"poses": demo_poses, "sampled_audit": demo_audit,
                                "physical_motion": False, "grasp": False},
            "object_overlap_check": "conservative_envelopes_not_physics",
            "articulation_root": str(root_path), "dof_names": dof_names,
            "collision_shapes_disabled_for_replay": disabled_collisions,
            "gravity_enabled_on_robot": False, "base_fixed": True,
            "physical_grasp_verified": False, "dynamic_stability_verified": False,
            "collision_safety_verified": False, "joint_limits_validated": True,
            "joint_limits_scope": "displayed_arm_endpoints_only_not_complete_trajectory",
            "replay_semantics": "guarded_endpoint_review_not_grasp_or_audited_trajectory",
            "saved_usd_reopened": True,
            "importer_warning": "massless URDF frames receive importer default inertia; not calibrated dynamics",
            "screenshots": evidence,
        }
        (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2,
                                                        allow_nan=False)+"\n", encoding="utf-8")
        if panel is not None:
            panel.visible = True
            panel.dock_in_window("Stage", ui.DockPosition.SAME)
            panel.set_active(True)
        print(f"HOLD_FLOW_SCENE_READY {output / 'manifest.json'}", flush=True)
        if getattr(args, "demo", False):
            view("Hands")
            start_demo()
        start = last = time.monotonic()
        while app.is_running() and (args.seconds == 0 or time.monotonic()-start < args.seconds):
            now = time.monotonic()
            tick_seconds = min(max(now-last, 0.0), .1)
            demo_step = replay_state["demo"]
            if demo_step:
                pose = sample_demo(demo_poses, replay_state["demo_elapsed"], args.segment_seconds)
                name, degrees = pose["name"], pose["left_joint_deg"]
                right_degrees, accepted = pose["right_joint_deg"], True
            elif replay_state["selected"] is None and not replay_state["playing"]:
                name, degrees = "RESET", home["left_joint_deg"]
                right_degrees, accepted = home["right_joint_deg"], True
            elif replay_state["playing"]:
                replay_state["elapsed"] += min(now-last, .1)
                name, degrees = sample_pose(replay["poses"], replay_state["elapsed"], args.segment_seconds)
                right_degrees, accepted = None, True
            else:
                pose = replay["poses"][replay_state["selected"]]
                name, degrees = pose["name"], pose["left_joint_deg"]
                right_degrees, accepted = pose.get("right_joint_deg"), pose.get("ik_accepted", True)
            last = now
            result = apply_pose(degrees, right_degrees, accepted)
            if not result["applied"]:
                replay_state["playing"] = False
                replay_state["demo_paused"] = True
                finish_recording("apply_pose_rejected")
            if label is not None:
                if not result["applied"]:
                    label.text = name + " | HOLD: IK/overlap rejected; last accepted pose"
                elif demo_step:
                    label.text = name + " | EMPTY-HAND CHECK"
                elif name == "RESET":
                    label.text = "RESET | nominal simulation pose"
                else:
                    label.text = name + " | candidate only"
            world.step(render=True)
            if demo_step:
                observed_error = float(np.max(np.abs(robot.get_joint_positions()-q)))
                if not math.isfinite(observed_error) or observed_error > 1e-4:
                    raise RuntimeError(f"시연 관절 배치 불일치: {observed_error}")
                if recording is not None:
                    capture_sync(recording.frame_path())
                    recording.add_frame({"time_s": replay_state["demo_elapsed"], "phase": name,
                                         "pose_error_rad": observed_error, "applied": result["applied"]})
                if pose["done"] and result["applied"]:
                    reset_preview(finalize_recording=False)
                    replay_state["demo_completed"] = True
                    reset_error = float(np.max(np.abs(robot.get_joint_positions()-home_q)))
                    capture_sync(output / "reset_after_demo.png")
                    finish_recording("demo_completed", completed=True,
                                     final_reset_error_rad=reset_error)
                    print(f"HOLD_FLOW_DEMO_FINISHED reset_error={reset_error}", flush=True)
                elif not replay_state["demo_paused"] and result["applied"]:
                    replay_state["demo_elapsed"] += .1 if recording is not None else tick_seconds
                if recording is not None:
                    time.sleep(max(0, .1-(time.monotonic()-now)))
            time.sleep(.01)
        finish_recording("time_limit" if args.seconds else "window_closed")
        print("HOLD_FLOW_REPLAY_FINISHED", flush=True)
    except BaseException as exc:
        if "finish_recording" in locals():
            finish_recording(f"exception:{type(exc).__name__}")
        traceback.print_exc()
        print("HOLD_FLOW_SCENE_FAILED", flush=True)
        # Kit의 fast shutdown이 예외를 exit 0으로 숨기지 않게 반환 코드를 설정한다.
        app._app.post_quit(1)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    run(parse_args())
