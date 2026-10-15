#!/usr/bin/env python3
"""JD-AMR systemd 템플릿을 사용자별 경로로 렌더링한다.

파일만 생성하며 systemd reload, enable, start는 실행하지 않는다.
"""

from __future__ import annotations

import argparse
import getpass
import os
from pathlib import Path


TOKENS = {
    "RUN_USER": "실행 사용자",
    "HOME": "실행 사용자 홈",
    "JDAMR_WS": "JD-AMR colcon 작업공간",
    "ASTRA_WS": "Astra colcon 작업공간",
    "DEPLOY_DIR": "실차 설정 파일 디렉터리",
    "DASHBOARD_DIR": "대시보드 소스 디렉터리",
}


def parse_args() -> argparse.Namespace:
    run_user = os.environ.get("USER") or getpass.getuser()
    home = os.environ.get("HOME") or str(Path.home())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--user", default=run_user)
    parser.add_argument("--home", default=home)
    parser.add_argument("--jdamr-ws", default=f"{home}/jdamr_ws")
    parser.add_argument("--astra-ws", default=f"{home}/jdamr_runtime/astra_ws")
    parser.add_argument("--deploy-dir", default=f"{home}/jdamr_deploy")
    parser.add_argument("--dashboard-dir", default=f"{home}/ammr_dashboard")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = Path(__file__).resolve().parent / "systemd"
    values = {
        "RUN_USER": args.user,
        "HOME": args.home,
        "JDAMR_WS": args.jdamr_ws,
        "ASTRA_WS": args.astra_ws,
        "DEPLOY_DIR": args.deploy_dir,
        "DASHBOARD_DIR": args.dashboard_dir,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    for template in sorted(source.rglob("*.in")):
        relative = template.relative_to(source)
        destination = args.output / relative.with_suffix("")
        destination.parent.mkdir(parents=True, exist_ok=True)
        content = template.read_text()
        for token, value in values.items():
            content = content.replace("@" + token + "@", value)
        unresolved = [name for name in TOKENS if "@" + name + "@" in content]
        if unresolved:
            raise ValueError(f"미해결 템플릿 토큰: {unresolved}")
        destination.write_text(content)
        print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
