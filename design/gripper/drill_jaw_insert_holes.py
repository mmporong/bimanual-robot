#!/usr/bin/env python3
"""ggao50 죠에 인서트 부착용 M3 관통 구멍을 뚫는다.

상류 STEP 을 읽어 구멍만 추가하고 STL 로 내보낸다. 상류 저장소에 라이선스
표기가 없어 원본도 결과물도 이 저장소에 커밋하지 않는다. `--out` 아래에만 둔다.

구멍 위치는 인서트 부착 창의 볼트 자리와 같다.
    Y = -38, -18   (인서트 중심 Y=-28 에서 손가락 방향 ±10)
    Z = 0          (홈이 Z 로 파여 중심선에는 항상 재료가 남는다)

실측 근거
    파지면 평면      x = -49.4
    블레이드 두께    Y=-38 에서 13.5 mm, Y=-18 에서 15.0 mm
    뒷면             Y=-38 에서 x=-62.9, Y=-18 에서 x=-64.4 (테이퍼)

체결은 관통볼트 + 평와셔 + 나일론 락너트다. 뒷면이 테이퍼라 너트 자리를
파지 않고 와셔로 받는다. 필요 볼트 길이는 M3 x 30 mm 다.

사용법:
  "$HOME/.cache/bimanual-cad-venv/bin/python" design/gripper/drill_jaw_insert_holes.py \
      --raw "$HOME/gcode/ggao50/raw" --out "$HOME/gcode/ggao50/print_ready"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cadquery as cq
from cadquery import exporters, importers

BOLT_CLEARANCE_D = 3.4
BOLT_Y = (-38.0, -18.0)
BOLT_Z = 0.0
FACE_X = -49.4          # 파지면 평면
DRILL_START_X = -46.0   # 파지면보다 바깥에서 시작해 확실히 관통시킨다
DRILL_LENGTH = 24.0     # 뒷면(-64.4)까지 충분히 지난다


def drill(shape: cq.Workplane) -> cq.Workplane:
    for y in BOLT_Y:
        cutter = (
            cq.Workplane("YZ")
            .workplane(offset=DRILL_START_X - DRILL_LENGTH)
            .center(y, BOLT_Z)
            .circle(BOLT_CLEARANCE_D / 2.0)
            .extrude(DRILL_LENGTH + 6.0)
        )
        shape = shape.cut(cutter)
    return shape


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", type=Path, required=True, help="상류 STEP 이 있는 위치")
    parser.add_argument("--out", type=Path, required=True, help="구멍 뚫은 STL 을 둘 위치")
    args = parser.parse_args()

    raw = args.raw.expanduser().resolve()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    for name in ("leftgripper", "rightgripper"):
        source = raw / f"{name}.step"
        if not source.exists():
            raise SystemExit(f"상류 STEP 이 없습니다: {source}\n"
                             "prepare_ggao50_print.py 로 먼저 받으세요.")
        shape = drill(importers.importStep(str(source)))
        solid = shape.val()
        if not solid.isValid():
            raise SystemExit(f"{name}: 구멍을 뚫은 뒤 형상이 유효하지 않습니다.")
        target = out / f"{name}_drilled.stl"
        exporters.export(shape, str(target), tolerance=0.05, angularTolerance=0.1)
        box = solid.BoundingBox()
        print(f"{name:14s} -> {target.name}  "
              f"bbox {box.xlen:.1f} x {box.ylen:.1f} x {box.zlen:.1f}  "
              f"부피 {solid.Volume()/1000.0:.2f} cm3")

    print(f"\n볼트 자리 Y = {BOLT_Y[0]:.0f}, {BOLT_Y[1]:.0f} / Z = {BOLT_Z:.0f} / Ø{BOLT_CLEARANCE_D}")
    print("체결: M3 x 30 관통볼트 + 평와셔 + 나일론 락너트")


if __name__ == "__main__":
    main()
