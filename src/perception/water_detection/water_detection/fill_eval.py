# fill_eval.py
# 카메라 없이 저장된 사진으로 충전율 계산을 확인한다. detect_gui와 같은 annotate_frame을
# 써서, 사진마다 충전율을 출력하고 표시 이미지(수면선·목표선)를 --out에 저장한다.
#
# 실행: python -m water_detection.fill_eval 사진1.jpg 사진2.jpg ... --out captures/fill_eval

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from water_detection.detect_gui import _ensure_model, annotate_frame
from water_detection.cup_tracker import CupTracker
from water_detection.fill_level import FlatnessTracker


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=Path("captures/fill_eval"))
    parser.add_argument("--target-fill", type=float, default=0.7)
    parser.add_argument("--tolerance", type=float, default=0.05)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    model = _ensure_model()
    tracker = FlatnessTracker()  # 같은 카메라로 찍은 사진들이라 이어서 쓴다
    for path in args.images:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"{path.name}: 이미지를 읽지 못함")
            continue
        display, cup_infos, flatness = annotate_frame(frame, model, args.target_fill, args.tolerance, tracker,
                                                      CupTracker(lock_frames=1))  # 사진마다 다른 장면
        cv2.imwrite(str(args.out / path.name), display)
        if not cup_infos:
            print(f"{path.name}: 컵 미검출")
        for info in cup_infos:
            r = info["reading"]
            fill = f"{r.ratio * 100:5.1f}%" if r.ratio is not None else f"  -   ({r.reason})"
            print(f"{path.name}: {info['side']:11s} fill {fill} {info['state']:7s} "
                  f"vol={r.volume_ratio and round(r.volume_ratio * 100)}% rim_y={r.rim_y:.0f} surface_y={r.surface_y and round(r.surface_y)} bottom_y={r.bottom_y:.0f} k={flatness and round(flatness, 2)}")


if __name__ == "__main__":
    main()
