#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../../.." && pwd)"
workspace_root="${JDAMR_RUNTIME_ROOT:-${HOME}/jdamr_runtime}"
mode="plan"

usage() {
  cat <<'EOF'
사용법: install_dependencies.sh [plan|--apply] [--workspace PATH] [--skip-apt]

기본 동작은 plan이며 시스템이나 작업공간을 변경하지 않는다.
  plan          설치 경로와 실행할 명령만 출력한다.
  --apply       apt, vcs import, rosdep 설치와 Astra 패치 적용을 실행한다.
  --workspace   Astra 작업공간 루트(기본: $JDAMR_RUNTIME_ROOT 또는 ~/jdamr_runtime)
  --skip-apt    apt update와 도구 패키지 설치를 건너뛴다.

이 스크립트는 colcon build, systemd 설치·enable·start, 실차 주행을 실행하지 않는다.
EOF
}

skip_apt=0
while (($#)); do
  case "$1" in
    plan) mode="plan" ;;
    --apply) mode="apply" ;;
    --workspace)
      shift
      [[ $# -gt 0 ]] || { echo "--workspace에 경로가 필요합니다" >&2; exit 2; }
      workspace_root="$1"
      ;;
    --skip-apt) skip_apt=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "알 수 없는 인자: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

astra_src="${workspace_root}/astra_ws/src/ros2_astra_camera"
repos_file="${script_dir}/jdamr-runtime.repos"
patch_file="${script_dir}/patches/ros2_astra_camera-f7e71d9-pi.patch"
astra_commit="f7e71d9ce806e788cb48d8580aac2c778fba4214"

print_plan() {
  cat <<EOF
JD-AMR 의존성 설치 계획
  양팔 저장소: ${repo_root}
  런타임 루트: ${workspace_root}
  Astra 소스:  ${astra_src}
  Astra pin:   ${astra_commit}

명시적인 --apply에서만 다음 작업을 실행합니다.
  1. Git, vcstool, rosdep, colcon과 Astra C++/Python 빌드 의존성 설치
  2. ${repos_file}을 ${workspace_root}/astra_ws/src에 vcs import
  3. Astra pin 확인 후 실차 호환 패치 적용
  4. src/jdamr와 clone한 Astra의 package.xml을 기준으로 rosdep 설치

빌드, 서비스 설치·기동, 주행은 별도 운영 단계입니다.
EOF
}

if [[ "$mode" == "plan" ]]; then
  print_plan
  exit 0
fi

if [[ "$skip_apt" -eq 0 ]]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake git pkg-config \
    libeigen3-dev libgoogle-glog-dev libopencv-dev libuvc-dev \
    python3-colcon-common-extensions python3-numpy python3-opencv \
    python3-rosdep python3-vcstool
fi

command -v vcs >/dev/null || { echo "vcs가 없습니다. --skip-apt 없이 다시 실행하세요." >&2; exit 3; }
command -v rosdep >/dev/null || { echo "rosdep이 없습니다. --skip-apt 없이 다시 실행하세요." >&2; exit 3; }

mkdir -p "${workspace_root}/astra_ws/src"
if [[ -e "$astra_src" ]]; then
  [[ -d "$astra_src/.git" ]] || { echo "기존 경로가 Git 저장소가 아닙니다: $astra_src" >&2; exit 4; }
else
  vcs import "${workspace_root}/astra_ws/src" < "$repos_file"
fi

actual_commit="$(git -C "$astra_src" rev-parse HEAD)"
[[ "$actual_commit" == "$astra_commit" ]] || {
  echo "Astra commit 불일치: expected=$astra_commit actual=$actual_commit" >&2
  exit 5
}

if git -C "$astra_src" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  echo "Astra 실차 패치가 이미 적용되어 있습니다."
else
  [[ -z "$(git -C "$astra_src" status --porcelain)" ]] || {
    echo "Astra 저장소에 다른 변경이 있어 패치를 적용하지 않습니다." >&2
    exit 6
  }
  git -C "$astra_src" apply --check "$patch_file"
  git -C "$astra_src" apply "$patch_file"
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "${repo_root}/src/jdamr" "$astra_src" \
  --ignore-src -y --rosdistro jazzy
echo "의존성 준비가 끝났습니다. 빌드와 서비스 설치는 README 절차를 따르세요."
