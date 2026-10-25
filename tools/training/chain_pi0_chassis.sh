#!/bin/bash
# 차체 결합 데이터 pi0 4개 학습: SmolVLA 체인(chain_smolvla_chassis.sh)과 같은 데이터·검증셋·증강·batch 8·
# 최대 step·500 step eval·조기 종료(EVAL_PATIENCE=3)·최적 체크포인트 업로드. 4090(24GB) 한계로 다른 점만:
#  - lerobot/pi0_base(사전학습)에서 시작. 4B 전체 학습은 bf16+gradient checkpointing에서도 OOM(2026-10-26 시험)
#    -> VLM 고정, 행동 전문가만 학습(train_expert_only, 578M, batch 8에서 12GB, 1.16 s/step).
#  - pi0_base 카메라 키에 맞춰 rename_map: top->base_0_rgb, left_wrist->left_wrist_0_rgb, right_wrist->right_wrist_0_rgb.
#    추론 때도 같은 --rename_map 필요.
#  - 체크포인트가 개당 10GB 안팎이라 학습 중 최적+최신만 남긴다(prune_ckpts.py).
set -uo pipefail
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot_vla
cd ~/lerobot
LOGDIR=/home/dgu/lerobot-training
DATA=/home/dgu/lerobot-data
CHAINLOG=$LOGDIR/chain_pi0_chassis.log
SUF=position-jitter-gravel-chassis-mounted-handoff-v1
RENAME='{"observation.images.top": "observation.images.base_0_rgb", "observation.images.left_wrist": "observation.images.left_wrist_0_rgb", "observation.images.right_wrist": "observation.images.right_wrist_0_rgb"}'

train_and_upload () {  # name dataset_dir eval_split steps
  local name=$1 ds=$2 split=$3 steps=$4
  echo "$(date -Iseconds) $name training start (steps=$steps, eval_split=$split)" >> $CHAINLOG
  python -m lerobot.scripts.lerobot_train \
    --policy.path=lerobot/pi0_base \
    --policy.push_to_hub=false \
    --policy.dtype=bfloat16 \
    --policy.gradient_checkpointing=true \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=true \
    --rename_map="$RENAME" \
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
    >> $LOGDIR/$name.log 2>&1 &
  local pid=$!
  while kill -0 $pid 2>/dev/null; do
    python3 $LOGDIR/prune_ckpts.py $name >> $CHAINLOG 2>&1
    sleep 60
  done
  wait $pid
  echo "$(date -Iseconds) $name training finished (exit $?)" >> $CHAINLOG
  python3 $LOGDIR/prune_ckpts.py $name >> $CHAINLOG 2>&1
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
if not ckpt.is_dir():
    sys.exit(f"{name}: best step {best_step} 체크포인트 폴더 없음 {ckpt}")
repo = f"jun-seo/{name}_{best_step}S"
create_repo(repo, repo_type="model", private=True, exist_ok=True)
HfApi().upload_folder(folder_path=str(ckpt), repo_id=repo, repo_type="model",
                      commit_message=f"best checkpoint ({name}, step {best_step}, eval_loss {best_loss})")
print(f"{name}: BEST step={best_step} eval_loss={best_loss} -> {repo}")
PYEOF
}

echo "$(date -Iseconds) chain start" >> $CHAINLOG
train_and_upload water_pour_chassis_pi0_60 water-pour-$SUF     0.1  13000
train_and_upload cup_place_chassis_pi0_60  cup-place-$SUF      0.1  14500
train_and_upload water_pour_chassis_pi0_80 water-pour-$SUF-80  0.07 17500
train_and_upload cup_place_chassis_pi0_80  cup-place-$SUF-80   0.07 19500
echo "$(date -Iseconds) chain all done" >> $CHAINLOG
