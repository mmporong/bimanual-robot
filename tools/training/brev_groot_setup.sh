#!/bin/bash
# Brev A100 인스턴스에 GR00T N1.7 학습 환경을 만든다. 학교 서버(lerobot_vla)와 같은 lerobot 커밋(d20a453)에
# 같은 조기종료 패치를 적용해, SmolVLA·π0와 같은 학습 스크립트(lerobot_train)를 쓴다.
# 사용: bash brev_groot_setup.sh   (패치 파일은 같은 폴더에 둔다)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
LEROBOT_COMMIT=d20a4538016d9fe0374a34011aebc1c8cddf3e0d

if [ ! -d "$HOME/miniforge3" ]; then
  curl -fsSL -o /tmp/miniforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  bash /tmp/miniforge.sh -b -p "$HOME/miniforge3"
fi
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda env list | grep -q '^lerobot_groot ' || conda create -y -n lerobot_groot python=3.12
conda activate lerobot_groot
# FFmpeg 8은 torchcodec을 깨뜨린다 -> 7로 고정
conda install -y -c conda-forge "ffmpeg>=6,<8"

if [ ! -d "$HOME/lerobot" ]; then
  git clone https://github.com/huggingface/lerobot.git "$HOME/lerobot"
fi
cd "$HOME/lerobot"
git checkout -q "$LEROBOT_COMMIT"
if git apply --check "$HERE/lerobot_train_earlystop_server_d20a453.patch" 2>/dev/null; then
  git apply "$HERE/lerobot_train_earlystop_server_d20a453.patch"
fi
grep -q "EVAL_PATIENCE = 3" src/lerobot/scripts/lerobot_train.py
pip install -e ".[groot]"
# 드라이버가 CUDA 12.8 미만(예: 565 = 12.7)이면 기본 torch(cu128+)가 GPU를 못 쓴다 -> 같은 버전의 cu126 빌드로 교체.
# --no-deps로 바꾸면 nvidia-cudnn 등 짝이 안 맞아 conv에서 CUDNN_STATUS_NOT_INITIALIZED가 난다 -> 의존성째 재설치한다.
if ! python -c "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)"; then
  TV=$(python -c "import torch; print(torch.__version__.split('+')[0])")
  VV=$(python -c "import torchvision; print(torchvision.__version__.split('+')[0])")
  pip install --force-reinstall "torch==$TV" "torchvision==$VV" --index-url https://download.pytorch.org/whl/cu126
fi
# 위 재설치가 numpy를 2.3+로 올리면 lerobot·numba와 충돌한다
pip install "numpy>=2.0,<2.3"
python -c "import torch; x=torch.randn(1,3,32,32,device='cuda'); print('cudnn conv ok', torch.nn.Conv2d(3,4,3).cuda()(x).shape)"
python -c "import torch, lerobot; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0))"
ffmpeg -version | head -1
mkdir -p "$HOME/lerobot-data" "$HOME/lerobot-training"
echo "setup done"
