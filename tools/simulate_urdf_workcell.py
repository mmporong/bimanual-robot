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

from workcell_preview_inputs import load_replay, materialize_urdf, sample_pose

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
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("seconds는 유한한 0 이상의 값이어야 합니다")
    if not math.isfinite(args.segment_seconds) or args.segment_seconds <= 0:
        parser.error("segment-seconds는 양의 유한한 값이어야 합니다")
    if args.headless and args.seconds == 0 and not args.check_only:
        parser.error("headless 실행은 --seconds로 종료 시간을 지정해야 합니다")
    return args


def run(args: argparse.Namespace) -> None:
    replay = load_replay(args.plan, args.state)
    source_hashes = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                     for name in ["simulate_urdf_workcell.py", "workcell_preview_inputs.py"]}
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"새 출력 디렉터리를 지정하세요: {output}")
    if args.check_only:
        print(json.dumps({"replay": replay, "scene": SCENE_ASSUMPTIONS,
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
        table_z = SCENE_ASSUMPTIONS["table_surface_z_m"]
        table_x, table_y = SCENE_ASSUMPTIONS["table_center_xy_m"]
        table_dx, table_dy = SCENE_ASSUMPTIONS["table_size_xy_m"]
        cube("/World/Workcell/Table", [table_x, table_y, table_z - 0.015],
             [table_dx, table_dy, .03], [.63, .68, .74])
        for i, x in enumerate([table_x-table_dx/2+.05, table_x+table_dx/2-.05]):
            for j, y in enumerate([table_y-table_dy/2+.07, table_y+table_dy/2-.07]):
                cube(f"/World/Workcell/TableLeg_{i}_{j}", [x, y, (table_z-.03)/2],
                     [.035, .035, table_z-.03], [.25, .28, .32])
        cup_x, cup_y = SCENE_ASSUMPTIONS["cup_center_xy_m"]
        cup_r, cup_h = SCENE_ASSUMPTIONS["cup_radius_m"], SCENE_ASSUMPTIONS["cup_height_m"]
        cylinder("/World/Workcell/Cup", [cup_x,cup_y,table_z+cup_h/2], cup_r,cup_h,[.2,.78,.9],.45)
        cylinder("/World/Workcell/CupRim", [cup_x,cup_y,table_z+cup_h], cup_r+.001,.006,[.55,.95,1])
        bottle_x, bottle_y = SCENE_ASSUMPTIONS["bottle_center_xy_m"]
        bottle_r, bottle_h = SCENE_ASSUMPTIONS["bottle_radius_m"], SCENE_ASSUMPTIONS["bottle_height_m"]
        cylinder("/World/Workcell/Bottle", [bottle_x,bottle_y,table_z+bottle_h/2], bottle_r,bottle_h,[.2,.65,.40],.8)
        cylinder("/World/Workcell/BottleNeck", [bottle_x,bottle_y,table_z+bottle_h+.018], .018,.035,[.2,.65,.40])
        cylinder("/World/Workcell/BottleCap", [bottle_x,bottle_y,table_z+bottle_h+.04], .02,.012,[.1,.2,.13])
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
        q = np.zeros(len(dof_names), dtype=np.float32)
        for name, deg in zip(ARM_JOINTS, SCENE_ASSUMPTIONS["right_parked_joint_deg"]):
            q[dof_names.index(f"right_{name}")] = np.deg2rad(deg)
        q[dof_names.index("left_gripper")] = SCENE_ASSUMPTIONS["left_stock_gripper_rad"]
        for name in ["right_finger1_joint", "right_finger2_joint"]:
            q[dof_names.index(name)] = SCENE_ASSUMPTIONS["right_gripper_open_m"]

        def apply_pose(degrees):
            q[indices] = np.radians(degrees)
            robot.set_joint_positions(q)
            robot.set_joint_velocities(np.zeros_like(q))
            robot.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=q))

        views = {
            "Overview": ([-1.6, 1.9, 1.50], [.22, 0, .55]),
            "Front": ([1.85, .01, 1.0], [.15,0,.60]),
            "Top": ([.25,.001,2.0],[.25,0,.60]),
        }
        def view(name):
            eye, target = views[name]
            set_camera_view(np.array(eye), np.array(target))

        for _ in range(4):
            app.update()
        view("Overview")
        replay_state = {"playing": True, "selected": 0, "elapsed": 0.0}
        label = None
        panel = None
        if not args.headless:
            import omni.ui as ui
            panel = ui.Window("HOLD THE FLOW - PREVIEW ONLY", width=410, height=375)
            with panel.frame:
                with ui.VStack(spacing=5):
                    ui.Label("450 x 340 mm | SO101 x 2", height=22)
                    ui.Label("BASE: original legacy" if args.original_base else
                             "BASE: 510 mm track / rear caster proxies", height=20)
                    ui.Label("KINEMATIC REPLAY: base fixed / contacts OFF", height=22)
                    ui.Label("Pose gallery interpolation, NOT a recorded trajectory", height=20)
                    ui.Label("FinRay NOT modeled; stock left jaw proxy", height=20)
                    ui.Label("Depth/LiDAR are design placeholders, NOT RGB3 calibration", word_wrap=True, height=38)
                    ui.Label("Table, cup, bottle, shelf: ASSUMED dimensions", height=22)
                    label = ui.Label("Loading saved poses...", word_wrap=True, height=40)
                    def pause():
                        replay_state["playing"] = not replay_state["playing"]
                    with ui.HStack(height=28):
                        ui.Button("Play / Pause", clicked_fn=pause)
                        ui.Button("Next pose", clicked_fn=lambda: replay_state.update(
                            playing=False, selected=(replay_state["selected"]+1) % len(replay["poses"])))
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
        for index, pose in enumerate(replay["poses"]):
            apply_pose(pose["left_joint_deg"])
            for _ in range(12):
                world.step(render=True)
                apply_pose(pose["left_joint_deg"])
            snapshot = output / f"pose_{index:02d}.png"
            capture_sync(snapshot)
            observed = robot.get_joint_positions()[indices]
            max_error = float(np.max(np.abs(observed-np.radians(pose["left_joint_deg"]))))
            if not math.isfinite(max_error) or max_error > 1e-4:
                raise RuntimeError(f"시뮬레이터 관절값 불일치: {max_error} rad")
            evidence.append({"pose": pose["name"], "image": snapshot.name,
                             "max_pose_assignment_error_rad": max_error})
            print(f"HOLD_FLOW_POSE_CAPTURED {pose['name']} {max_error:.8f} rad", flush=True)
        apply_pose(replay["poses"][0]["left_joint_deg"])
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
            "replay": replay, "scene_assumptions": SCENE_ASSUMPTIONS,
            "articulation_root": str(root_path), "dof_names": dof_names,
            "collision_shapes_disabled_for_replay": disabled_collisions,
            "gravity_enabled_on_robot": False, "base_fixed": True,
            "physical_grasp_verified": False, "dynamic_stability_verified": False,
            "collision_safety_verified": False, "joint_limits_validated": False,
            "replay_semantics": "pose_gallery_interpolation_not_recorded_or_audited_trajectory",
            "saved_usd_reopened": True,
            "importer_warning": "massless URDF frames receive importer default inertia; not calibrated dynamics",
            "screenshots": evidence,
        }
        (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2,
                                                        allow_nan=False)+"\n", encoding="utf-8")
        print(f"HOLD_FLOW_SCENE_READY {output / 'manifest.json'}", flush=True)
        start = last = time.monotonic()
        while app.is_running() and (args.seconds == 0 or time.monotonic()-start < args.seconds):
            now = time.monotonic()
            if replay_state["playing"]:
                replay_state["elapsed"] += min(now-last, .1)
                name, degrees = sample_pose(replay["poses"], replay_state["elapsed"], args.segment_seconds)
            else:
                pose = replay["poses"][replay_state["selected"]]
                name, degrees = pose["name"], pose["left_joint_deg"]
            last = now
            apply_pose(degrees)
            if label is not None:
                label.text = name
            world.step(render=True)
            time.sleep(.01)
        print("HOLD_FLOW_REPLAY_FINISHED", flush=True)
    except BaseException:
        traceback.print_exc()
        print("HOLD_FLOW_SCENE_FAILED", flush=True)
        # Kit의 fast shutdown이 예외를 exit 0으로 숨기지 않게 반환 코드를 설정한다.
        app._app.post_quit(1)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    run(parse_args())
