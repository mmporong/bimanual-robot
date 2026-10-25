#!/bin/bash
# 차체 결합 데이터(chassis-mounted) SmolVLA 4개 학습: 60개(v1) 2 task -> 80개(part2+v1) 2 task.
# 설정은 v2(40개) 학습과 동일하고, 최대 step만 데이터 프레임 수에 비례해 늘렸다
# (v2: 물따르기 74,751프레임/컵배치 31,481프레임에 10,000 step -> 같은 학습률 감쇠 모양 유지).
# 이미지 증강(lerobot 기본: 밝기·대비·채도·색조·선명도·회전±5°/이동±5% 중 최대 3개, 학습셋에만): 차체 결합 후
# 브리지 재연결마다 노출이 달라지고 top 카메라 각도·손목캠 기종이 바뀌는 것에 대비. eval 셋에는 증강 없음.
# 조기 종료: lerobot_train.py의 EVAL_PATIENCE=3 (500 step마다 eval, 3번 연속 미개선 시 중단).
# eval 셋: 60개는 eval_split 0.1 -> 마지막 6개(54~59), 80개는 0.07 -> 마지막 6개(74~79 = v1의 54~59)로 동일.
set -uo pipefail
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot_vla
cd ~/lerobot

LOGDIR=/home/dgu/lerobot-training
DATA=/home/dgu/lerobot-data
CHAINLOG=$LOGDIR/chain_smolvla_chassis.log
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
  echo "$(date -Iseconds) $name training finished (exit $?)" >> $CHAINLOG
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
train_and_upload water_pour_chassis_smolvla_60 water-pour-$SUF     0.1  13000
train_and_upload cup_place_chassis_smolvla_60  cup-place-$SUF      0.1  14500
train_and_upload water_pour_chassis_smolvla_80 water-pour-$SUF-80  0.07 17500
train_and_upload cup_place_chassis_smolvla_80  cup-place-$SUF-80   0.07 19500
echo "$(date -Iseconds) chain all done" >> $CHAINLOG
