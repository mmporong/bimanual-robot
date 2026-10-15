#!/usr/bin/env python3
"""Build, inspect and explicitly deploy the repository-owned JD-AMR runtime."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import shlex
import subprocess
import sys
import time

from _common import (
    ASSET_SOURCE,
    PC_WORKSPACE,
    REPO_ROOT,
    RUNTIME_ASSETS,
    bootstrap_assets,
)


SOURCE_ROOT = REPO_ROOT / "src/navigation"
TOOLS_ROOT = SOURCE_ROOT / "scripts/map_20260930_manual"
CAMERA_MOUNT = SOURCE_ROOT / "jdamr_cube_navigation/config/camera_mount_current.yaml"
REMOTE_PROMOTE = REPO_ROOT / "tools/jdamr/remote_promote.py"
RUNTIME_PACKAGES = (
    "jdamr_base_driver",
    "jdamr_cube_bringup",
    "jdamr_cube_cartographer",
    "jdamr_cube_description",
    "jdamr_cube_gazebo",
    "jdamr_cube_navigation",
    "jdamr_cube_node",
    "jdamr_cube_vslam",
    "ydlidar_g4_ros2",
)
SYSTEM_PYTHON = "/usr/bin/python3"
CMAKE_PYTHON_ARGS = (
    "--cmake-args",
    f"-DPython3_EXECUTABLE={SYSTEM_PYTHON}",
    f"-DPYTHON_EXECUTABLE={SYSTEM_PYTHON}",
)
HOST_PATTERN = re.compile(
    r"^[A-Za-z0-9_][A-Za-z0-9._-]*@(?:[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?|"
    r"(?:[0-9]{1,3}\.){3}[0-9]{1,3})$"
)
REMOTE_PATH_PATTERN = re.compile(r"^/[A-Za-z0-9._ /+-]+$")


def shell_join(parts: list[str]) -> str:
    return shlex.join([str(part) for part in parts])


def run(parts: list[str], *, dry_run: bool) -> None:
    print(shell_join(parts))
    if not dry_run:
        subprocess.run(parts, check=True)


def validate_host(host: str) -> str:
    if not HOST_PATTERN.fullmatch(host):
        raise ValueError(f"invalid SSH host (expected user@DNS-or-IPv4): {host!r}")
    return host


def validate_remote_path(value: str, label: str) -> str:
    if not REMOTE_PATH_PATTERN.fullmatch(value):
        raise ValueError(f"invalid {label}: {value!r}")
    path = PurePosixPath(value)
    if (
        not path.is_absolute()
        or value == "/"
        or "//" in value
        or ".." in path.parts
        or str(path) != value
        or value.endswith("/")
    ):
        raise ValueError(f"invalid {label}: {value!r}")
    return value


def ssh_argv(host: str, *remote: str) -> list[str]:
    ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "--", host]
    return [*ssh, shlex.join(remote)]


def ssh_script(host: str, script: str) -> list[str]:
    return ssh_argv(host, "bash", "-lc", script)


def rsync_argv(source: str, host: str, destination: str, *options: str) -> list[str]:
    return [
        "rsync", "-a", "--protect-args", *options,
        "-e", "ssh -o BatchMode=yes -o ConnectTimeout=8 --", "--",
        source, f"{host}:{destination}",
    ]


def package_args(all_packages: bool) -> list[str]:
    return [] if all_packages else ["--packages-select", *RUNTIME_PACKAGES]


def build_command(workspace: Path, *, all_packages: bool) -> list[str]:
    return [
        "colcon", "--log-base", str(workspace / "log"), "build", "--symlink-install",
        "--base-paths", str(SOURCE_ROOT),
        "--build-base", str(workspace / "build"),
        "--install-base", str(workspace / "install"),
        *package_args(all_packages),
        *CMAKE_PYTHON_ARGS,
    ]


def cmd_bootstrap(args: argparse.Namespace) -> None:
    report = bootstrap_assets(refresh=args.refresh)
    print(json.dumps({"asset_root": str(RUNTIME_ASSETS), **report}, indent=2))


def cmd_build(args: argparse.Namespace) -> None:
    if not args.dry_run:
        bootstrap_assets()
        PC_WORKSPACE.mkdir(parents=True, exist_ok=True)
    run(build_command(PC_WORKSPACE, all_packages=args.all), dry_run=args.dry_run)


def deploy_commands(args: argparse.Namespace, *, stamp: str | None = None) -> list[list[str]]:
    host = validate_host(args.host)
    pi_home = validate_remote_path(args.pi_home, "Pi home")
    pi_ws = validate_remote_path(args.pi_ws or f"{pi_home}/jdamr_ws", "Pi workspace")
    remote_tools = f"{pi_home}/jdamr_data/map_20260930_manual/tools"
    remote_assets = f"{pi_home}/jdamr_data/map_20260930_manual/aligned"
    remote_deploy = f"{pi_home}/jdamr_deploy"
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    if not re.fullmatch(r"[0-9]{8}_[0-9]{6}|[A-Za-z0-9._-]+", stamp):
        raise ValueError(f"invalid deployment stamp: {stamp!r}")
    stage = f"{remote_deploy}/bimanual_stage_{stamp}"
    stage_source = f"{stage}/src/jdamr_cube_ros"
    backup = f"{remote_deploy}/bimanual_backup_{stamp}"
    stage_tools = f"{stage}/tools"
    stage_assets = f"{stage}/assets"
    stage_mount = f"{stage}/camera_mount_current.yaml"
    stage_promote = f"{stage}/remote_promote.py"
    create_stage = (
        f"set -e; test ! -e {shlex.quote(stage)}; "
        f"mkdir -p {shlex.quote(stage_source)} {shlex.quote(stage_tools)} "
        f"{shlex.quote(stage_assets)}"
    )
    commands: list[list[str]] = [
        ssh_script(host, create_stage),
        rsync_argv(
            f"{SOURCE_ROOT}/", host, f"{stage_source}/",
            "--exclude", ".git/", "--exclude", "__pycache__/",
            "--exclude", "build/", "--exclude", "install/", "--exclude", "log/",
        ),
        rsync_argv(f"{TOOLS_ROOT}/", host, f"{stage_tools}/", "--exclude", "__pycache__/"),
        rsync_argv(f"{ASSET_SOURCE}/", host, f"{stage_assets}/"),
        rsync_argv(str(CAMERA_MOUNT), host, stage_mount),
        rsync_argv(str(REMOTE_PROMOTE), host, stage_promote),
    ]
    selected = "" if args.all else " --packages-select " + " ".join(RUNTIME_PACKAGES)
    stage_build = (
        "set -e; source /opt/ros/jazzy/setup.bash; "
        f"cd {shlex.quote(stage)}; colcon build --symlink-install "
        f"--base-paths {shlex.quote(stage_source)}{selected} "
        f"--cmake-args -DPython3_EXECUTABLE={SYSTEM_PYTHON} "
        f"-DPYTHON_EXECUTABLE={SYSTEM_PYTHON}"
    )
    commands.append(ssh_script(host, stage_build))
    if not args.promote:
        return commands
    promote = [
        SYSTEM_PYTHON, stage_promote,
        "--stage-root", stage,
        "--workspace", pi_ws,
        "--tools-live", remote_tools,
        "--assets-live", remote_assets,
        "--mount-live", f"{remote_deploy}/camera_mount_current.yaml",
        "--backup-root", backup,
    ]
    if args.replace_assets:
        promote.append("--replace-assets")
    commands.append(ssh_argv(host, *promote))
    return commands


def cmd_deploy(args: argparse.Namespace) -> None:
    if not args.dry_run:
        bootstrap_assets()
    for command in deploy_commands(args):
        run(command, dry_run=args.dry_run)
    if not args.dry_run:
        result = "검증된 Pi stage 슬롯 승격" if args.promote else "격리 stage 복사와 빌드"
        print(f"{result}가 끝났습니다. 서비스 재시작과 이동 명령은 실행하지 않았습니다.")


def cmd_plan(args: argparse.Namespace) -> None:
    scope = "all imported ROS packages" if args.all else "9 physical-runtime ROS packages"
    print(f"# PC fresh build ({scope})")
    print(shell_join(build_command(PC_WORKSPACE, all_packages=args.all)))
    action = "stage, validate, back up and promote" if args.promote else "stage and validate only"
    print(f"# Pi source deployment ({action})")
    for command in deploy_commands(args):
        print(shell_join(command))


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    sub = result.add_subparsers(dest="command", required=True)
    bootstrap = sub.add_parser("bootstrap", help="누락된 지도·등록부·route 자산만 생성")
    bootstrap.add_argument(
        "--refresh", action="store_true",
        help="다른 기존 자산을 백업한 뒤 저장소 번들로 갱신",
    )
    bootstrap.set_defaults(handler=cmd_bootstrap)

    build = sub.add_parser("build-pc", help="저장소 아래 .runtime 워크스페이스에서 PC 빌드")
    build.add_argument("--dry-run", action="store_true")
    build.add_argument(
        "--all", action="store_true",
        help="선택된 실차 런타임 9개 대신 import된 모든 ROS 패키지 빌드",
    )
    build.set_defaults(handler=cmd_build)

    def deployment_arguments(command: argparse.ArgumentParser) -> None:
        command.add_argument(
            "--host", default=os.environ.get("JDAMR_HOST", "lim@192.168.0.159")
        )
        command.add_argument(
            "--all", action="store_true",
            help="선택된 실차 런타임 9개 대신 import된 모든 ROS 패키지 빌드",
        )
        command.add_argument(
            "--promote", action="store_true",
            help="격리 stage 빌드 성공 뒤 현재 Pi 소스에 백업을 남기고 반영",
        )
        command.add_argument("--pi-home", default=os.environ.get("JDAMR_PI_HOME", "/home/lim"))
        command.add_argument("--pi-ws", default=os.environ.get("JDAMR_PI_WS"))
        command.add_argument(
            "--replace-assets", action="store_true",
            help="원격 기존 정적 자산을 타임스탬프 백업 후 교체",
        )

    deploy = sub.add_parser(
        "deploy-pi", help="전체 JD-AMR 소스·도구·자산을 Pi에 복사하고 빌드"
    )
    deployment_arguments(deploy)
    deploy.add_argument("--dry-run", action="store_true")
    deploy.set_defaults(handler=cmd_deploy)

    plan = sub.add_parser("plan", help="PC 빌드와 Pi 배포 명령을 변경 없이 출력")
    deployment_arguments(plan)
    plan.set_defaults(handler=cmd_plan)
    return result


def main() -> None:
    args = parser().parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
