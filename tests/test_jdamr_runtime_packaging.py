"""JD-AMR 런타임 배포 묶음의 재현성과 비기동 계약을 검증한다."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
JDAMR = ROOT / "src/jdamr"
DEPLOY = JDAMR / "deployment"
DASHBOARD = JDAMR / "dashboard"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_astra_dependency_is_official_pinned_and_patch_is_recorded():
    repos = (DEPLOY / "jdamr-runtime.repos").read_text()
    assert "https://github.com/orbbec/ros2_astra_camera.git" in repos
    assert "f7e71d9ce806e788cb48d8580aac2c778fba4214" in repos

    provenance = json.loads((DEPLOY / "provenance.json").read_text())
    astra = provenance["astra_camera"]
    patch = DEPLOY / astra["patch"]
    assert sha256(patch) == astra["patch_sha256"]
    patch_text = patch.read_text()
    assert patch_text.count("diff --git ") == 10
    assert set(astra["modified_files"]) == {
        line.split(" b/", 1)[0][len("diff --git a/"):]
        for line in patch_text.splitlines()
        if line.startswith("diff --git a/")
    }


def test_dependency_installer_defaults_to_read_only_plan():
    installer = DEPLOY / "install_dependencies.sh"
    with tempfile.TemporaryDirectory() as temporary:
        runtime = Path(temporary) / "runtime-that-must-not-be-created"
        result = subprocess.run(
            ["bash", str(installer)],
            check=True,
            capture_output=True,
            text=True,
            env={"HOME": temporary, "JDAMR_RUNTIME_ROOT": str(runtime)},
        )
        assert "JD-AMR 의존성 설치 계획" in result.stdout
        assert "명시적인 --apply" in result.stdout
        assert not runtime.exists()

    source = installer.read_text()
    plan_exit = source.index('if [[ "$mode" == "plan" ]]')
    assert source.index("sudo apt-get update") > plan_exit
    assert source.index('vcs import "${workspace_root}/astra_ws/src"') > plan_exit
    assert "colcon build" not in source.split("print_plan()", 1)[1]
    assert "systemctl" not in source
    for package in (
        "build-essential", "cmake", "pkg-config", "libeigen3-dev",
        "libgoogle-glog-dev", "libopencv-dev", "libuvc-dev",
    ):
        assert package in source


def test_dependency_installer_rosdep_covers_jdamr_and_astra_sources():
    installer = DEPLOY / "install_dependencies.sh"
    commit = "f7e71d9ce806e788cb48d8580aac2c778fba4214"
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        runtime = root / "runtime"
        astra = runtime / "astra_ws/src/ros2_astra_camera"
        (astra / ".git").mkdir(parents=True)
        fake_bin = root / "bin"
        fake_bin.mkdir()
        log = root / "commands.log"

        def fake_command(name: str, body: str) -> None:
            path = fake_bin / name
            path.write_text("#!/bin/sh\nset -eu\n" + body)
            path.chmod(0o755)

        fake_command(
            "git",
            f'''case "$*" in
  *"rev-parse HEAD"*) printf '%s\\n' "{commit}" ;;
  *"apply --reverse --check"*) exit 0 ;;
  *) exit 0 ;;
esac
''',
        )
        fake_command("vcs", "exit 0\n")
        fake_command("rosdep", 'printf \'%s\\n\' "$*" >> "$MOCK_COMMAND_LOG"\n')
        fake_command("sudo", 'exec "$@"\n')

        env = os.environ.copy()
        env.update({
            "HOME": str(root / "home"),
            "MOCK_COMMAND_LOG": str(log),
            "PATH": f"{fake_bin}:{env['PATH']}",
        })
        subprocess.run(
            [
                "bash", str(installer), "--apply", "--skip-apt",
                "--workspace", str(runtime),
            ],
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )
        commands = log.read_text().splitlines()
        install = next(line for line in commands if line.startswith("install "))
        assert f"--from-paths {ROOT / 'src/jdamr'} {astra}" in install
        assert "--ignore-src -y --rosdistro jazzy" in install


def test_systemd_templates_render_without_machine_specific_user_paths():
    templates = sorted((DEPLOY / "systemd").rglob("*.in"))
    assert len(templates) == 10
    for template in templates:
        text = template.read_text()
        assert "/home/lim" not in text
        assert "User=lim" not in text
        assert "systemctl" not in text

    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary) / "systemd"
        subprocess.run(
            [
                "python3", str(DEPLOY / "render_systemd_templates.py"),
                "--output", str(output),
                "--user", "robot",
                "--home", "/srv/robot",
                "--jdamr-ws", "/srv/robot/jdamr_ws",
                "--astra-ws", "/srv/robot/astra_ws",
                "--deploy-dir", "/srv/robot/jdamr_deploy",
                "--dashboard-dir", "/srv/robot/ammr_dashboard",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        rendered = list(output.rglob("*"))
        files = [path for path in rendered if path.is_file()]
        assert len(files) == len(templates)
        merged = "\n".join(path.read_text() for path in files)
        assert "@RUN_USER@" not in merged
        assert "@JDAMR_WS@" not in merged
        assert "User=robot" in merged
        assert "/srv/robot/jdamr_ws/install/setup.bash" in merged
        assert "--scan-display-frame base_footprint" in merged
        assert "stable_frames:=4" in merged


def test_hardware_rules_preserve_current_uart_and_lidar_names():
    uart = (DEPLOY / "udev/99-jdamr-uart.rules").read_text()
    lidar = (DEPLOY / "udev/99-ydlidar-g4.rules").read_text()
    assert 'KERNEL=="ttyS0"' in uart
    assert 'GROUP="dialout"' in uart
    assert 'ATTRS{idVendor}=="10c4"' in lidar
    assert 'ATTRS{idProduct}=="ea60"' in lidar
    assert 'ATTRS{serial}=="0001"' in lidar
    assert 'SYMLINK+="ydlidar_g4"' in lidar


def test_dashboard_matches_captured_pi_sources_and_is_read_only():
    provenance = json.loads((DEPLOY / "provenance.json").read_text())
    for relative, expected in provenance["dashboard"]["files"].items():
        assert sha256((DEPLOY / relative).resolve()) == expected

    server = (DASHBOARD / "ammr_dashboard.py").read_text()
    compile(server, str(DASHBOARD / "ammr_dashboard.py"), "exec")
    compile((DASHBOARD / "host_metrics.py").read_text(),
            str(DASHBOARD / "host_metrics.py"), "exec")
    assert "create_publisher" not in server
    assert "create_client" not in server
    assert 'default="127.0.0.1"' in server
    assert 'default=8090' in server
    assert "/home/lim" not in server


def test_provenance_covers_current_service_and_device_sources():
    provenance = json.loads((DEPLOY / "provenance.json").read_text())
    services = provenance["systemd_source_sha256"]
    assert len(services) == 8
    assert "/etc/systemd/system/jdamr-base.service" in services
    assert "/etc/systemd/system/jdamr-ammr-dashboard.service" in services
    assert any(path.startswith("/run/systemd/system/") for path in services)
    hardware = provenance["hardware_paths"]
    assert hardware["base_controller"] == "/dev/ttyS0"
    assert hardware["lidar"] == "/dev/ydlidar_g4"
