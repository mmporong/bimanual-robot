#!/bin/bash
# bimanual-robot 통합 뒤에는 전체 JD-AMR 소스와 도구를 함께 배포해야 한다.
set -eo pipefail

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
runtime_tool="$repo_root/tools/jdamr/runtime.py"
if [ -z "$repo_root" ] || [ ! -x "$runtime_tool" ]; then
  echo "통합 배포 도구를 찾지 못했습니다: tools/jdamr/runtime.py" >&2
  exit 2
fi

echo "이 스크립트의 단일 navigation 패키지 배포 방식은 더 이상 지원하지 않습니다." >&2
echo "먼저 실행 계획을 확인하세요:" >&2
echo "  $runtime_tool deploy-pi --dry-run --host <user@pi>" >&2
echo "격리 빌드만 하려면 --dry-run을 제거하고, 현재 Pi에 반영하려면 --promote도 추가하세요." >&2
exit 2
