"""학습 중 체크포인트 정리: eval_loss 최솟값(지금까지 최적)과 가장 최근 체크포인트만 남긴다.

pi0 체크포인트는 개당 10GB 안팎이라 500 step마다 다 남기면 서버 디스크가 넘친다.
최적 고르는 규칙은 체인 업로드 단계와 같다(min, 동률이면 먼저 나온 step).
사용: python3 prune_ckpts.py <모델명>  (체인이 학습 동안 1분마다 호출)
"""
import re
import shutil
import sys
from pathlib import Path

name = sys.argv[1]
LOGDIR = Path("/home/dgu/lerobot-training")
ckdir = LOGDIR / "outputs" / name / "checkpoints"
text = (LOGDIR / f"{name}.log").read_text(errors="ignore").replace("\r", "\n")
pairs = [(int(m.group(1)), float(m.group(2))) for m in re.finditer(r"step (\d+): eval_loss=([\d.]+)", text)]
if not pairs or not ckdir.is_dir():
    sys.exit(0)
best = min(pairs, key=lambda p: p[1])[0]
steps = sorted(int(p.name) for p in ckdir.iterdir() if p.name.isdigit())
if not steps:
    sys.exit(0)
newest = steps[-1]
for s in steps:
    if s != best and s < newest:
        shutil.rmtree(ckdir / f"{s:06d}")
        print(f"pruned {name} {s:06d} (best={best}, newest={newest})", flush=True)
