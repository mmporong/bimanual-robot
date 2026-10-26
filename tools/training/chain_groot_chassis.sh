#!/bin/bash
# 차체 결합 데이터 GR00T N1.7 8개 학습(2 task × 20/40/60/80개) (NVIDIA Brev A100 80GB): SmolVLA·π0 체인(chain_smolvla_chassis.sh,
# chain_pi0_chassis.sh)과 같은 데이터·검증셋·증강·batch 8·최대 step·500 step eval·조기 종료(EVAL_PATIENCE=3)·
# 최적 체크포인트 HF 업로드. 다른 점:
#  - nvidia/GR00T-N1.7-3B에서 시작. 튜닝 범위는 lerobot 기본값(언어·시각 백본 고정, projector·diffusion head·vlln 학습).
#  - 환경은 brev_groot_setup.sh (lerobot d20a453 + 서버와 같은 조기종료 패치).
#  - 체크포인트가 커서 학습 중 최적+최신만 남긴다(prune_ckpts.py).
# 데이터: $HOME/lerobot-data 에 학교 서버와 같은 v1(60개)·-80(part2+v1) 폴더를 둔다.
set -uo pipefail
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot_groot
cd "$HOME/lerobot"
LOGDIR=$HOME/lerobot-training
DATA=$HOME/lerobot-data
CHAINLOG=$LOGDIR/chain_groot_chassis.log
SUF=position-jitter-gravel-chassis-mounted-handoff-v1

train_and_upload () {  # name dataset_dir eval_split steps
  local name=$1 ds=$2 split=$3 steps=$4
  echo "$(date -Iseconds) $name training start (steps=$steps, eval_split=$split)" >> $CHAINLOG
  python -m lerobot.scripts.lerobot_train \
    --policy.type=groot \
    --policy.base_model_path=nvidia/GR00T-N1.7-3B \
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
    "${EXTRA_ARGS[@]}" \
    >> $LOGDIR/$name.log 2>&1 &
  local pid=$!
  while kill -0 $pid 2>/dev/null; do
    python $LOGDIR/prune_ckpts.py $name >> $CHAINLOG 2>&1
    sleep 60
  done
  wait $pid
  local rc=$?  # $(date)가 $?를 덮어써서 예전엔 실패도 exit 0으로 남았다
  echo "$(date -Iseconds) $name training finished (exit $rc)" >> $CHAINLOG
  python $LOGDIR/prune_ckpts.py $name >> $CHAINLOG 2>&1
  python - "$name" <<'PYEOF' >> $CHAINLOG 2>&1
import re, sys
from pathlib import Path
from huggingface_hub import HfApi, create_repo
name = sys.argv[1]
LOGDIR = Path.home() / "lerobot-training"
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
  # 체크포인트가 24GB라 업로드가 끝난 모델은 지운다(남겨 두면 다음 모델 저장 중 디스크가 넘친다, 2026-10-26 cup_place 80)
  if grep -q "^$name: BEST step=" $CHAINLOG; then
    rm -rf "$LOGDIR/outputs/$name"
  fi
}

# 시험(smoke)에서 필요하다고 확인된 추가 인자는 여기에 넣는다.
EXTRA_ARGS=()

echo "$(date -Iseconds) chain start" >> $CHAINLOG
# 예산(쿠폰 $100)이 중간에 끝나도 중요한 결과부터 남도록 80 -> 60 -> 40 -> 20 순서.
# 검증셋은 모두 v1의 54~59번 6개: 80개는 part2+v1(0.07), 60개는 v1(0.1), 40개는 0~33+54~59(0.15), 20개는 0~13+54~59(0.3).
train_and_upload water_pour_chassis_groot_80 water-pour-$SUF-80  0.07 17500
train_and_upload cup_place_chassis_groot_80  cup-place-$SUF-80   0.07 19500
train_and_upload water_pour_chassis_groot_60 water-pour-$SUF     0.1  13000
train_and_upload cup_place_chassis_groot_60  cup-place-$SUF      0.1  14500
train_and_upload water_pour_chassis_groot_40 water-pour-$SUF-40  0.15 8500
train_and_upload cup_place_chassis_groot_40  cup-place-$SUF-40   0.15 9500
train_and_upload water_pour_chassis_groot_20 water-pour-$SUF-20  0.3  4000
train_and_upload cup_place_chassis_groot_20  cup-place-$SUF-20   0.3  4500
echo "$(date -Iseconds) chain all done" >> $CHAINLOG
