"""resolve_camera_indices로 찾은 인덱스를 명령어에 채워 넣고 바로 실행한다.

각 run_*.py 실행 스크립트가 이 모듈의 run_lerobot()를 불러써서, 매번
"지금 왼쪽/오른쪽/뎁스가 몇 번 인덱스인지"를 새로 찾은 뒤 그 값으로
lerobot-record/lerobot-rollout을 실행한다. 인덱스가 세션마다 흔들려도
이 스크립트를 쓰면 다시 캡쳐해서 확인할 필요가 없다.
"""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from resolve_camera_indices import resolve


def run_lerobot(exe_path: str, extra_args: list[str]) -> None:
    r = resolve()
    print(f"[camera resolve] left={r['left']} right={r['right']} depth={r['depth']}")

    args = [exe_path]
    for a in extra_args:
        args.append(a.format(left=r["left"], right=r["right"], depth=r["depth"]))

    print("[launching]", " ".join(args))
    result = subprocess.run(args)
    sys.exit(result.returncode)
