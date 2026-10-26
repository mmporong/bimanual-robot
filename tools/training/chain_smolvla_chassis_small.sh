#!/bin/bash
# 차체 결합 데이터 SmolVLA 40·20개 4개 학습 (2026-10-26 추가 요청). chain_smolvla_chassis.sh(60·80)와 같은 설정·증강·
# 500 step eval·조기 종료·최적 체크포인트 업로드. 학교 서버의 pi0 체인(chain_pi0_chassis.sh, chain_pi0_chassis_small.sh)이
# 모두 끝나면 시작한다(GPU 1장을 나눠 쓰지 않는다).
# 검증셋은 모든 크기에서 v1의 54~59번 6개: 40개(0~33+54~59)는 eval_split 0.15, 20개(0~13+54~59)는 0.3.
# 최대 step은 60·80과 같은 규칙(프레임 수 비례): 물따르기 8500·4000, 컵배치 9500·4500.
# 데이터 -40/-20 폴더는 서버의 make_subsets.sh로 v1에서 잘라 만들었다.
set -uo pipefail
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot_vla
cd ~/lerobot

LOGDIR=/home/dgu/lerobot-training
DATA=/home/dgu/lerobot-data
CHAINLOG=$LOGDIR/chain_smolvla_chassis_small.log
SUF=position-jitter-gravel-chassis-mounted-handoff-v1

train_and_upload () {  # name dataset_dir eval_split steps
  local name=$1 ds=$2 split=$3 steps=$4
  echo "$(date -Iseconds) $name training start (steps=$steps, eval_split=$split)" >> $CHAINLOG
  python -m lerobot.scripts.lerobot_train \
    --policy.type=smolvla \
    --policy.push_to_hub=false \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --dataset.repo_id=jun-seo/$ds \
    --dataset.root=$DATA/$ds \
    --dataset.eval_split=$split \
    --dataset.image_transforms.enable=true \
    --output_dir=$LOGDIR/outputs/$name \
    --batch_size=8 \
    --steps=$steps \
    --save_freq=500 \
    --eval_steps=500 \
    --save_checkpoint=true \
    >> $LOGDIR/$name.log 2>&1
  local rc=$?  # $(date)가 $?를 덮어쓰지 않게 먼저 받는다
  echo "$(date -Iseconds) $name training finished (exit $rc)" >> $CHAINLOG
  python3 - "$name" <<'PYEOF' >> $CHAINLOG 2>&1
import re, sys
from pathlib import Path
from huggingface_hub import HfApi, create_repo
name = sys.argv[1]
LOGDIR = Path("/home/dgu/lerobot-training")
text = (LOGDIR / f"{name}.log").read_text(errors="ignore").replace("\r", "\n")
pairs = [(int(m.group(1)), float(m.group(2))) for m in re.finditer(r"step (\d+): eval_loss=([\d.]+)", text)]
if not pairs:
    sys.exit(f"{name}: eval_loss 로그 없음 - 업로드 스킵")
print(f"{name}: eval history " + ", ".join(f"{s}:{l}" for s, l in pairs))
best_step, best_loss = min(pairs, key=lambda p: p[1])
ckpt = LOGDIR / "outputs" / name / "checkpoints" / f"{best_step:06d}" / "pretrained_model"
if not (ckpt / "model.safetensors").is_file():  # 저장 중 죽으면 폴더만 남는다
    sys.exit(f"{name}: best step {best_step} 체크포인트 폴더 없음 {ckpt}")
repo = f"jun-seo/{name}_{best_step}S"
create_repo(repo, repo_type="model", private=True, exist_ok=True)
HfApi().upload_folder(folder_path=str(ckpt), repo_id=repo, repo_type="model",
                      commit_message=f"best checkpoint ({name}, step {best_step}, eval_loss {best_loss})")
print(f"{name}: BEST step={best_step} eval_loss={best_loss} -> {repo}")
PYEOF
}

while pgrep -fx "/bin/bash ./chain_pi0_chassis.sh|bash ./chain_pi0_chassis.sh|/bin/bash ./chain_pi0_chassis_small.sh|bash ./chain_pi0_chassis_small.sh" >/dev/null; do
  sleep 300
done
echo "$(date -Iseconds) chain start" >> $CHAINLOG
train_and_upload water_pour_chassis_smolvla_40 water-pour-$SUF-40  0.15 8500
train_and_upload cup_place_chassis_smolvla_40  cup-place-$SUF-40   0.15 9500
train_and_upload water_pour_chassis_smolvla_20 water-pour-$SUF-20  0.3  4000
train_and_upload cup_place_chassis_smolvla_20  cup-place-$SUF-20   0.3  4500
echo "$(date -Iseconds) chain all done" >> $CHAINLOG
