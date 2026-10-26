#!/bin/bash
# 차체 결합 데이터 ACT 8개 학습(2 task × 80/60/40/20개, 2026-10-26 요청). SmolVLA·π0·GR00T 체인과 같은 데이터·검증셋·
# 증강·batch 8·최대 step·500 step eval·조기 종료(EVAL_PATIENCE=3)·최적 체크포인트 HF 업로드.
#  - ACT 설정은 lerobot 기본값(이전 ACT 실험과 같음): chunk 100, n_action_steps 100, lr 1e-5, VAE(kl_weight 10),
#    ResNet18 ImageNet 백본. 처음부터 학습한다. eval_loss는 L1+KL이라 다른 모델과 직접 비교하지 않는다.
#  - 검증셋은 모든 크기에서 v1의 54~59번 6개: 80개 0.07, 60개 0.1, 40개 0.15, 20개 0.3.
#  - 체크포인트는 개당 약 0.6GB(가중치 198MB). 학습 중 최적+최신만 남긴다(prune_ckpts.py).
#  - 학교 서버 GPU를 나눠 쓰지 않게 pi0 체인 두 개와 SmolVLA 40·20 체인이 모두 끝난 뒤 시작한다.
set -uo pipefail
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot_vla
cd ~/lerobot
LOGDIR=/home/dgu/lerobot-training
DATA=/home/dgu/lerobot-data
CHAINLOG=$LOGDIR/chain_act_chassis.log
SUF=position-jitter-gravel-chassis-mounted-handoff-v1

train_and_upload () {  # name dataset_dir eval_split steps
  local name=$1 ds=$2 split=$3 steps=$4
  echo "$(date -Iseconds) $name training start (steps=$steps, eval_split=$split)" >> $CHAINLOG
  python -m lerobot.scripts.lerobot_train \
    --policy.type=act \
    --policy.push_to_hub=false \
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
  local rc=$?  # $(date)가 $?를 덮어써서 예전엔 실패도 exit 0으로 남았다
  echo "$(date -Iseconds) $name training finished (exit $rc)" >> $CHAINLOG
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
if not (ckpt / "model.safetensors").is_file():  # 저장 중 죽으면 폴더만 남는다
    sys.exit(f"{name}: best step {best_step} 체크포인트 폴더 없음 {ckpt}")
repo = f"jun-seo/{name}_{best_step}S"
create_repo(repo, repo_type="model", private=True, exist_ok=True)
HfApi().upload_folder(folder_path=str(ckpt), repo_id=repo, repo_type="model",
                      commit_message=f"best checkpoint ({name}, step {best_step}, eval_loss {best_loss})")
print(f"{name}: BEST step={best_step} eval_loss={best_loss} -> {repo}")
PYEOF
}

WAIT='/bin/bash ./chain_pi0_chassis.sh|bash ./chain_pi0_chassis.sh|/bin/bash ./chain_pi0_chassis_small.sh|bash ./chain_pi0_chassis_small.sh|bash ./chain_smolvla_chassis_small.sh'
while pgrep -fx "$WAIT" >/dev/null; do
  sleep 300
done
echo "$(date -Iseconds) chain start" >> $CHAINLOG
train_and_upload water_pour_chassis_act_80 water-pour-$SUF-80  0.07 17500
train_and_upload cup_place_chassis_act_80  cup-place-$SUF-80   0.07 19500
train_and_upload water_pour_chassis_act_60 water-pour-$SUF     0.1  13000
train_and_upload cup_place_chassis_act_60  cup-place-$SUF      0.1  14500
train_and_upload water_pour_chassis_act_40 water-pour-$SUF-40  0.15 8500
train_and_upload cup_place_chassis_act_40  cup-place-$SUF-40   0.15 9500
train_and_upload water_pour_chassis_act_20 water-pour-$SUF-20  0.3  4000
train_and_upload cup_place_chassis_act_20  cup-place-$SUF-20   0.3  4500
echo "$(date -Iseconds) chain all done" >> $CHAINLOG
