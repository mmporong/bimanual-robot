"""Regression checks for the repository-owned JD-AMR runtime tools."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import shlex
import shutil
import socket
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools/jdamr"


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class StandaloneRuntimeTest(unittest.TestCase):
    def test_generated_table_route_stays_outside_recorded_opposing_table_turn_sweep(self):
        assets = ROOT / 'config/navigation/jdamr_runtime'
        geometry = yaml.safe_load((ROOT / 'src/navigation/jdamr_cube_description/config/'
                                    'new_base_geometry.yaml').read_text())
        robot = ET.parse(ROOT / 'src/navigation/jdamr_cube_description/urdf/new_base_real.urdf')
        rear_joint = robot.find("joint[@name='lower_rear_rail_joint']/origin")
        rear_rail = robot.find("link[@name='lower_rear_rail_link']/collision/geometry/box")
        rear_m = -float(rear_joint.attrib['xyz'].split()[0]) + float(
            rear_rail.attrib['size'].split()[0]) / 2
        # The whole wheel-width rectangle contains both front and rear drive wheels.
        radius_m = math.hypot(max(rear_m, geometry['front_to_wheel_axis']['value']),
                              geometry['wheel_outer_width']['value'] / 2)
        face = (0.3537144070752165, -1.3372212134060284)
        normal = (0.9987897624345229, 0.04918343680538581)

        def clearance(point):
            return sum((point[i] - face[i]) * normal[i] for i in (0, 1))

        with tempfile.TemporaryDirectory() as temporary:
            aligned = Path(temporary) / 'map_20260930_manual/aligned'
            shutil.copytree(assets, aligned)
            with mock.patch.dict(os.environ, {'JDAMR_DATA_ROOT': temporary}):
                helper = load('jdamr_route_geometry_test', ROOT / 'src/navigation/scripts/'
                              'map_20260930_manual/jdamr_depart.py')
            helper.pi = mock.Mock(side_effect=AssertionError('no remote operation'))
            helper.sh = mock.Mock(side_effect=AssertionError('no process operation'))
            home = yaml.safe_load((aligned / 'service_destinations.yaml').read_text())['home']
            helper.write_routes(tuple(home[k] for k in ('x_m', 'y_m', 'yaw_rad')))
            route = yaml.safe_load((aligned / 'table_01_route.yaml').read_text())
            pre, observation = route['waypoints']
            mount = yaml.safe_load((ROOT / 'src/navigation/jdamr_cube_navigation/config/'
                                    'camera_mount_current.yaml').read_text())['camera_mount']
            observation_offset_m = mount['table_observation']['base_offset_m']
            face_x_m, face_y_m = helper.DESTINATIONS['table_01']['face_xy']
            self.assertEqual((observation['x'], observation['y'], observation['yaw']),
                             (round(face_x_m - observation_offset_m, 3), face_y_m, 0.0))
            # The aisle turn stays at its original clearance; only the final view closes in.
            self.assertEqual(pre['x'], round(face_x_m - helper.OBSERVE_M, 3))
            self.assertGreater(observation['x'], pre['x'])
            for step in range(21):
                point = tuple(pre[k] + (observation[k] - pre[k]) * step / 20
                              for k in ('x', 'y'))
                self.assertGreater(clearance(point), radius_m)
            self.assertLess(clearance((0.438, -1.817)), radius_m)
            for name in ('water_station_route.yaml', 'table_01_direct_route.yaml',
                         'table_02_direct_route.yaml'):
                generated = yaml.safe_load((aligned / name).read_text())
                self.assertEqual(generated['waypoints'][0]['id'], 'home_exit')
                self.assertEqual(generated['dock_departure_distance_m'], helper.HOME_EXIT_M)
                bundled = yaml.safe_load((assets / name).read_text())
                # The observation endpoint follows the current camera mount;
                # the archival bundle preserves its older endpoint.
                dest = 'water_station' if name.startswith('water_station') else name.split('_direct')[0]
                x, y, normal = helper.along_normal(dest, observation_offset_m)
                endpoint = generated['waypoints'][-1]
                self.assertEqual((endpoint['x'], endpoint['y']), (round(x, 3), round(y, 3)))
                self.assertEqual(generated['waypoints'][:-1], bundled['waypoints'][:-1])
                self.assertEqual(generated['expected_mask_sha256'], bundled['expected_mask_sha256'])
            table2 = yaml.safe_load((aligned / 'table_02_route.yaml').read_text())
            bundled2 = yaml.safe_load((assets / 'table_02_route.yaml').read_text())
            self.assertEqual(table2['waypoints'][:-1], bundled2['waypoints'][:-1])
            x, y, _ = helper.along_normal('table_02', observation_offset_m)
            self.assertEqual((table2['waypoints'][-1]['x'], table2['waypoints'][-1]['y']),
                             (round(x, 3), round(y, 3)))

    def test_existing_owned_user_bus_fills_missing_terminal_environment(self):
        common = load('jdamr_bus_common', TOOLS / '_common.py')
        with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_UNIX) as bus:
            runtime = Path(directory)
            bus.bind(str(runtime / 'bus'))
            with mock.patch.dict(os.environ, {}, clear=True):
                self.assertTrue(common.prepare_user_service_environment(runtime))
                self.assertEqual(os.environ['XDG_RUNTIME_DIR'], str(runtime))
                self.assertEqual(os.environ['DBUS_SESSION_BUS_ADDRESS'], f'unix:path={runtime}/bus')
                os.environ['DBUS_SESSION_BUS_ADDRESS'] = 'existing-session'
                common.prepare_user_service_environment(runtime)
                self.assertEqual(os.environ['DBUS_SESSION_BUS_ADDRESS'], 'existing-session')

    def test_missing_or_non_socket_user_bus_does_not_invent_a_session(self):
        common = load('jdamr_missing_bus_common', TOOLS / '_common.py')
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {}, clear=True):
            runtime = Path(directory)
            self.assertFalse(common.prepare_user_service_environment(runtime))
            (runtime / 'bus').touch()
            self.assertFalse(common.prepare_user_service_environment(runtime))
            self.assertNotIn('DBUS_SESSION_BUS_ADDRESS', os.environ)

    def test_bundled_runtime_excludes_mutable_state_and_keeps_126cm_mount(self):
        assets = ROOT / "config/navigation/jdamr_runtime"
        self.assertFalse((assets / "state.json").exists())
        self.assertFalse((assets / "rviz_initialpose.json").exists())
        mount = yaml.safe_load(
            (ROOT / "src/navigation/jdamr_cube_navigation/config/camera_mount_current.yaml").read_text()
        )
        self.assertEqual(mount["camera_mount"]["transform"]["z_m"], 1.26)

    def test_preserved_helpers_have_pinned_import_provenance(self):
        provenance = json.loads((TOOLS / "provenance.json").read_text())
        self.assertEqual(len(provenance["departure_helper_original_sha256"]), 64)
        for name, expected in provenance["copied_dependencies"].items():
            actual = hashlib.sha256(
                (ROOT / "src/navigation/scripts/map_20260930_manual" / name).read_bytes()
            ).hexdigest()
            derived = provenance.get("derived_dependencies", {}).get(name)
            if derived is not None:
                self.assertEqual(derived["source_sha256"], expected, name)
                self.assertTrue(derived["change_scope"], name)
            self.assertEqual(actual, derived["sha256"] if derived else expected, name)

    def test_help_works_with_empty_data_root_and_creates_no_mutable_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            env = {**os.environ, "JDAMR_DATA_ROOT": temporary}
            result = subprocess.run(
                [str(TOOLS / "jdamr_depart.py"), "--help"],
                cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn("go", result.stdout)
            aligned = Path(temporary) / "map_20260930_manual/aligned"
            self.assertFalse(aligned.exists())
            self.assertFalse((aligned / "state.json").exists())
            self.assertFalse((aligned / "rviz_initialpose.json").exists())

    def test_legacy_help_can_prepare_archived_static_assets_without_runtime_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            env = {**os.environ, "JDAMR_DATA_ROOT": temporary}
            result = subprocess.run(
                [str(TOOLS / "jdamr_depart.py"), "--legacy-map", "--help"],
                cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout)
            aligned = Path(temporary) / "map_20260930_manual/aligned"
            self.assertTrue((aligned / "map.yaml").is_file())
            self.assertFalse((aligned / "state.json").exists())

    def test_bootstrap_preserves_changed_registry_and_refresh_backs_it_up(self):
        common = load("jdamr_common_test", TOOLS / "_common.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime = root / "data/aligned"
            source = root / "source"
            source.mkdir()
            runtime.mkdir(parents=True)
            (source / "service_destinations.yaml").write_text("bundled\n")
            (runtime / "service_destinations.yaml").write_text("operator\n")
            with mock.patch.object(common, "ASSET_SOURCE", source), mock.patch.object(
                common, "LEGACY_RUNTIME_ASSETS", runtime
            ), mock.patch.object(common, "RUNTIME_ROOT", root / "runtime"):
                report = common.bootstrap_assets(legacy=True)
                self.assertEqual(report["preserved"], ["service_destinations.yaml"])
                self.assertEqual(
                    (runtime / "service_destinations.yaml").read_text(), "operator\n"
                )
                report = common.bootstrap_assets(refresh=True, now="stamp", legacy=True)
                self.assertEqual(
                    (root / "runtime/asset_backups/stamp/service_destinations.yaml").read_text(),
                    "operator\n",
                )
                self.assertEqual(
                    (runtime / "service_destinations.yaml").read_text(), "bundled\n"
                )
                self.assertEqual(len(report["backed_up"]), 1)

    def test_deploy_plan_has_full_source_and_never_restarts_services(self):
        sys_path_added = False
        if str(TOOLS) not in os.sys.path:
            os.sys.path.insert(0, str(TOOLS))
            sys_path_added = True
        try:
            runtime = load("jdamr_runtime_test", TOOLS / "runtime.py")
        finally:
            if sys_path_added:
                os.sys.path.remove(str(TOOLS))
        args = type("Args", (), {
            "host": "robot@test", "pi_home": "/srv/robot",
            "pi_ws": "/srv/ws", "replace_assets": False, "all": False,
            "promote": True, "legacy_map": True,
        })()
        rendered = "\n".join(runtime.shell_join(c) for c in runtime.deploy_commands(args))
        self.assertIn(str(ROOT / "src/navigation") + "/", rendered)
        self.assertIn("--workspace /srv/ws", rendered)
        self.assertIn("colcon", rendered)
        self.assertIn(" build ", rendered)
        self.assertIn("camera_mount_current.yaml", rendered)
        self.assertIn("--exclude .git/", rendered)
        self.assertIn("--protect-args", rendered)
        self.assertIn("remote_promote.py", rendered)
        self.assertNotIn("--delete", rendered)
        for package in runtime.RUNTIME_PACKAGES:
            self.assertIn(package, rendered)
        self.assertNotIn("systemctl", rendered)
        self.assertNotIn("ros2 action", rendered)
        self.assertNotIn("cmd_vel", rendered)
        self.assertIn("-DPython3_EXECUTABLE=/usr/bin/python3", rendered)
        self.assertIn("-DPYTHON_EXECUTABLE=/usr/bin/python3", rendered)
        self.assertEqual(rendered.count("colcon build"), 1)

    def test_remote_input_validation_rejects_shell_values_and_accepts_spaces(self):
        if str(TOOLS) not in os.sys.path:
            os.sys.path.insert(0, str(TOOLS))
            remove_path = True
        else:
            remove_path = False
        try:
            runtime = load("jdamr_runtime_validation", TOOLS / "runtime.py")
        finally:
            if remove_path:
                os.sys.path.remove(str(TOOLS))
        for value in (
            "robot", "robot@host;touch /tmp/x", "-F@host",
            "-oProxyCommand@host", "-oProxyCommand=x@host", "robot@host name",
        ):
            with self.assertRaises(ValueError):
                runtime.validate_host(value)
        for value in (
            "relative", "/", "//srv/root", "/srv/./root", "/srv/../root",
            "/srv/a;touch x", "/srv/$HOME",
        ):
            with self.assertRaises(ValueError):
                runtime.validate_remote_path(value, "test")
        self.assertEqual(runtime.validate_host("robot@pi.local"), "robot@pi.local")
        self.assertEqual(
            runtime.validate_remote_path("/srv/robot workspace", "test"),
            "/srv/robot workspace",
        )
        args = type("Args", (), {
            "host": "robot@pi.local", "pi_home": "/srv/robot home",
            "pi_ws": "/srv/robot workspace", "replace_assets": False,
            "all": False, "promote": True, "legacy_map": True,
        })()
        commands = runtime.deploy_commands(args, stamp="stamp")
        rendered = "\n".join(runtime.shell_join(command) for command in commands)
        self.assertIn("'/srv/robot workspace'", rendered)
        self.assertTrue(all(command[-3] == "--" for command in commands if command[0] == "rsync"))
        self.assertTrue(all(command[5] == "--" for command in commands if command[0] == "ssh"))
        self.assertTrue(all(len(command[7:]) == 1 for command in commands if command[0] == "ssh"))
        with self.assertRaises(ValueError):
            runtime.deploy_commands(args, stamp="stamp;touch")

    def test_rsync_inserted_login_user_is_accepted_by_openssh(self):
        with mock.patch.object(os.sys, 'path', [str(TOOLS), *os.sys.path]):
            runtime = load('jdamr_runtime_rsync_transport', TOOLS / 'runtime.py')
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, report, wrapper = root / 'input', root / 'report.json', root / 'ssh-probe'
            source.write_text('no remote writes\n')
            wrapper.write_text(
                f'#!{os.sys.executable}\n'
                'import json,subprocess,sys\nfrom pathlib import Path\n'
                "r=subprocess.run(['ssh','-G','-F','/dev/null',*sys.argv[1:]], "
                'capture_output=True,text=True)\n'
                f'Path({str(report)!r}).write_text(json.dumps({{"code":r.returncode, '
                '"values":[s for s in r.stdout.splitlines() '
                'if s.startswith(("hostname ","user "))]}))\n'
                'sys.exit(99)\n')
            wrapper.chmod(0o755)
            command = runtime.rsync_argv(str(source), 'robot@127.0.0.1',
                                         '/unused-pointcloud-test', '--dry-run')
            transport_index = command.index('-e') + 1
            transport = shlex.split(command[transport_index])
            transport[0] = str(wrapper)
            command[transport_index] = shlex.join(transport)
            # Real rsync constructs the SSH argv. OpenSSH -G parses it without
            # connecting; the wrapper deliberately stops before data transfer.
            subprocess.run(command, capture_output=True, text=True, timeout=10)
            parsed = json.loads(report.read_text())
            self.assertEqual(parsed['code'], 0)
            self.assertIn('user robot', parsed['values'])
            self.assertIn('hostname 127.0.0.1', parsed['values'])

    def _promotion_layout(self, root: Path):
        stage = root / "stage"
        for relative in ("src/jdamr_cube_ros", "install", "tools", "assets"):
            (stage / relative).mkdir(parents=True)
        (stage / "src/jdamr_cube_ros/new.py").write_text("new source")
        (stage / "install/setup.bash").write_text("new install")
        (stage / "tools/tool.py").write_text("new tool")
        (stage / "assets/new.yaml").write_text("new asset")
        (stage / "camera_mount_current.yaml").write_text("camera: new")
        workspace = root / "live workspace"
        source = workspace / "src/jdamr_cube_ros"
        install = workspace / "install"
        tools = root / "live tools"
        assets = root / "live assets"
        mount = root / "deploy/camera_mount_current.yaml"
        for directory in (source, install, tools, assets, mount.parent):
            directory.mkdir(parents=True, exist_ok=True)
        (source / "stale.py").write_text("old source")
        (install / "old.bash").write_text("old install")
        (tools / "old.py").write_text("old tool")
        (assets / "new.yaml").write_text("operator asset")
        mount.write_text("camera: operator")
        return stage, workspace, tools, assets, mount

    def test_remote_promotion_publishes_exact_stage_and_preserves_mutable_assets(self):
        remote = load("jdamr_remote_promote_success", TOOLS / "remote_promote.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage, workspace, tools, assets, mount = self._promotion_layout(root)
            backup = root / "backup"
            receipt = remote.promote(
                stage_root=stage, workspace=workspace, tools_live=tools,
                assets_live=assets, mount_live=mount, backup_root=backup,
            )
            self.assertTrue((workspace / "src/jdamr_cube_ros").is_symlink())
            self.assertEqual(
                (workspace / "src/jdamr_cube_ros").resolve(),
                (stage / "src/jdamr_cube_ros").resolve(),
            )
            self.assertFalse((workspace / "src/jdamr_cube_ros/stale.py").exists())
            self.assertTrue((workspace / "src/jdamr_cube_ros/new.py").is_file())
            self.assertEqual((backup / "slots/source/stale.py").read_text(), "old source")
            self.assertEqual((assets / "new.yaml").read_text(), "operator asset")
            self.assertEqual(mount.read_text(), "camera: operator")
            self.assertIn("new.yaml", receipt["preserved_assets"])

    def test_remote_promotion_rolls_back_slots_on_intermediate_failure(self):
        remote = load("jdamr_remote_promote_failure", TOOLS / "remote_promote.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage, workspace, tools, assets, mount = self._promotion_layout(root)
            with self.assertRaisesRegex(RuntimeError, "injected failure"):
                remote.promote(
                    stage_root=stage, workspace=workspace, tools_live=tools,
                    assets_live=assets, mount_live=mount, backup_root=root / "backup",
                    fail_after=4,
                )
            self.assertFalse((workspace / "src/jdamr_cube_ros").is_symlink())
            self.assertEqual((workspace / "src/jdamr_cube_ros/stale.py").read_text(), "old source")
            self.assertEqual((workspace / "install/old.bash").read_text(), "old install")
            self.assertEqual((tools / "old.py").read_text(), "old tool")

    def test_remote_asset_replacement_rolls_back_existing_and_new_files(self):
        remote = load("jdamr_remote_promote_asset_failure", TOOLS / "remote_promote.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage, workspace, tools, assets, mount = self._promotion_layout(root)
            (stage / "assets/a-created.yaml").write_text("created")
            with self.assertRaisesRegex(RuntimeError, "injected failure"):
                remote.promote(
                    stage_root=stage, workspace=workspace, tools_live=tools,
                    assets_live=assets, mount_live=mount, backup_root=root / "backup",
                    replace_assets=True, fail_after=8,
                )
            self.assertFalse((assets / "a-created.yaml").exists())
            self.assertEqual((assets / "new.yaml").read_text(), "operator asset")
            self.assertEqual(mount.read_text(), "camera: operator")

    def test_remote_slot_backup_rename_failure_does_not_remove_original(self):
        remote = load("jdamr_remote_promote_slot_rename_failure", TOOLS / "remote_promote.py")
        for slot_name in ("source", "install"):
            with self.subTest(slot=slot_name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                stage, workspace, tools, assets, mount = self._promotion_layout(root)
                source = workspace / "src/jdamr_cube_ros"
                install = workspace / "install"
                failing_slot = source if slot_name == "source" else install
                real_replace = remote.os.replace

                def fail_slot_backup(source_path, destination):
                    if Path(source_path) == failing_slot:
                        raise PermissionError(f"mock {slot_name} backup failure")
                    return real_replace(source_path, destination)

                with mock.patch.object(remote.os, "replace", side_effect=fail_slot_backup):
                    with self.assertRaisesRegex(
                        PermissionError, f"mock {slot_name} backup failure"
                    ):
                        remote.promote(
                            stage_root=stage, workspace=workspace, tools_live=tools,
                            assets_live=assets, mount_live=mount,
                            backup_root=root / "backup",
                        )
                self.assertFalse(source.is_symlink())
                self.assertEqual((source / "stale.py").read_text(), "old source")
                self.assertEqual((install / "old.bash").read_text(), "old install")
                self.assertEqual((tools / "old.py").read_text(), "old tool")

    def test_remote_asset_backup_rename_failure_preserves_file_and_rolls_back_slots(self):
        remote = load("jdamr_remote_promote_asset_rename_failure", TOOLS / "remote_promote.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage, workspace, tools, assets, mount = self._promotion_layout(root)
            existing_asset = assets / "new.yaml"
            real_replace = remote.os.replace

            def fail_asset_backup(source, destination):
                if Path(source) == existing_asset:
                    raise OSError("mock asset backup rename failure")
                return real_replace(source, destination)

            with mock.patch.object(remote.os, "replace", side_effect=fail_asset_backup):
                with self.assertRaisesRegex(OSError, "mock asset backup rename failure"):
                    remote.promote(
                        stage_root=stage, workspace=workspace, tools_live=tools,
                        assets_live=assets, mount_live=mount, backup_root=root / "backup",
                        replace_assets=True,
                    )
            self.assertEqual(existing_asset.read_text(), "operator asset")
            self.assertFalse((workspace / "src/jdamr_cube_ros").is_symlink())
            self.assertEqual((workspace / "src/jdamr_cube_ros/stale.py").read_text(), "old source")
            self.assertEqual((workspace / "install/old.bash").read_text(), "old install")
            self.assertEqual((tools / "old.py").read_text(), "old tool")

    def test_remote_asset_copy_and_publish_failures_restore_everything(self):
        remote = load("jdamr_remote_promote_asset_write_failure", TOOLS / "remote_promote.py")
        for failure in ("copy", "publish"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                stage, workspace, tools, assets, mount = self._promotion_layout(root)
                existing_asset = assets / "new.yaml"
                stage_asset = stage / "assets/new.yaml"
                real_copy = remote.shutil.copy2
                real_replace = remote.os.replace

                def maybe_fail_copy(source, destination, *args, **kwargs):
                    if failure == "copy" and Path(source) == stage_asset:
                        raise OSError("mock asset copy failure")
                    return real_copy(source, destination, *args, **kwargs)

                def maybe_fail_publish(source, destination):
                    if (
                        failure == "publish"
                        and Path(destination) == existing_asset
                        and Path(source).name.startswith(".new.yaml.next-")
                    ):
                        raise OSError("mock asset publish failure")
                    return real_replace(source, destination)

                with mock.patch.object(remote.shutil, "copy2", side_effect=maybe_fail_copy), \
                     mock.patch.object(remote.os, "replace", side_effect=maybe_fail_publish):
                    with self.assertRaisesRegex(OSError, f"mock asset {failure} failure"):
                        remote.promote(
                            stage_root=stage, workspace=workspace, tools_live=tools,
                            assets_live=assets, mount_live=mount,
                            backup_root=root / "backup", replace_assets=True,
                        )
                self.assertEqual(existing_asset.read_text(), "operator asset")
                self.assertFalse((workspace / "src/jdamr_cube_ros").is_symlink())
                self.assertEqual(
                    (workspace / "src/jdamr_cube_ros/stale.py").read_text(), "old source"
                )
                self.assertEqual((workspace / "install/old.bash").read_text(), "old install")
                self.assertEqual((tools / "old.py").read_text(), "old tool")

    def test_remote_concurrent_promotion_fails_before_changing_live_slots(self):
        remote = load("jdamr_remote_promote_lock", TOOLS / "remote_promote.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage, workspace, tools, assets, mount = self._promotion_layout(root)
            lock_path = workspace / ".jdamr_deploy.lock"
            with lock_path.open("a+") as held_lock:
                remote.fcntl.flock(
                    held_lock.fileno(), remote.fcntl.LOCK_EX | remote.fcntl.LOCK_NB
                )
                with self.assertRaisesRegex(RuntimeError, "holds the workspace lock"):
                    remote.promote(
                        stage_root=stage, workspace=workspace, tools_live=tools,
                        assets_live=assets, mount_live=mount,
                        backup_root=root / "different-stamp-backup",
                        replace_assets=True,
                    )
            source = workspace / "src/jdamr_cube_ros"
            self.assertFalse(source.is_symlink())
            self.assertEqual((source / "stale.py").read_text(), "old source")
            self.assertEqual((workspace / "install/old.bash").read_text(), "old install")
            self.assertEqual((tools / "old.py").read_text(), "old tool")
            self.assertEqual((assets / "new.yaml").read_text(), "operator asset")
            self.assertEqual(mount.read_text(), "camera: operator")


if __name__ == "__main__":
    unittest.main()
