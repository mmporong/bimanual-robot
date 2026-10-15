#!/usr/bin/env python3
"""상단 RGB 영상의 픽셀을 base_footprint 작업대 XY에 대응시킨다.

실측한 작업대 점을 ``--base-point X,Y``로 순서대로 넘긴 뒤, 영상에서 같은
점을 같은 순서로 클릭한다. 최소 네 점으로 평면 homography를 만들며 팔을
움직이거나 서보 포트를 열지 않는다.
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import cv2
import numpy as np
import yaml


DEFAULT_TOP = "/dev/v4l/by-path/pci-0000:65:00.3-usb-0:2.1:1.0-video-index0"
DEFAULT_OUTPUT = Path("calibration/cameras/top/workspace_plane.yaml")


def parse_xy(value: str) -> list[float]:
    try:
        x, y = value.split(",")
        return [float(x), float(y)]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("X,Y 형식이어야 합니다. 예: 0.30,0.20") from exc


def capture_frame(camera: str, width: int, height: int) -> np.ndarray:
    capture = cv2.VideoCapture(camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: {camera}")
    try:
        frame = None
        ok = False
        for _ in range(8):
            ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"카메라 프레임을 읽을 수 없습니다: {camera}")
        return frame
    finally:
        capture.release()


def collect_image_points(image: np.ndarray, count: int) -> list[list[float]]:
    points: list[list[float]] = []
    window = "workspace plane: click in --base-point order | backspace undo | enter save | q abort"

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < count:
            points.append([float(x), float(y)])

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    try:
        while True:
            shown = image.copy()
            for index, (x, y) in enumerate(points, start=1):
                center = (round(x), round(y))
                cv2.circle(shown, center, 7, (20, 80, 240), -1)
                cv2.putText(shown, str(index), (center[0] + 9, center[1] - 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 80, 240), 2, cv2.LINE_AA)
            cv2.putText(shown, f"points {len(points)}/{count}", (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (30, 170, 60), 2, cv2.LINE_AA)
            cv2.imshow(window, shown)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                raise RuntimeError("사용자가 작업대 보정을 취소했습니다")
            if key in (8, 127) and points:
                points.pop()
            if key in (10, 13) and len(points) == count:
                return points
    finally:
        cv2.destroyWindow(window)


def build_calibration(
    image_points: list[list[float]],
    base_points: list[list[float]],
    image_size: tuple[int, int],
    surface_z_m: float,
    cup_height_m: float,
    grasp_height_fraction: float,
    pregrasp_clearance_m: float,
) -> dict:
    image_array = np.asarray(image_points, dtype=float)
    base_array = np.asarray(base_points, dtype=float)
    if image_array.shape != base_array.shape or image_array.shape[0] < 4:
        raise ValueError("동일한 개수의 픽셀/실측 대응점이 최소 4개 필요합니다")
    homography, _ = cv2.findHomography(image_array, base_array, method=0)
    if homography is None:
        raise ValueError("homography 계산에 실패했습니다")
    homogeneous = np.column_stack([image_array, np.ones(len(image_array))])
    projected = (homography @ homogeneous.T).T
    projected = projected[:, :2] / projected[:, 2:3]
    errors = np.linalg.norm(projected - base_array, axis=1) * 1000.0
    return {
        "schema_version": "1.0",
        "frame": "base_footprint",
        "mapping": "top_rgb_pixel_to_table_plane_xy_homography",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "image_points_px": [[round(float(v), 3) for v in point] for point in image_array],
        "base_points_m": [[round(float(v), 6) for v in point] for point in base_array],
        "surface_z_m": surface_z_m,
        "cup_height_m": cup_height_m,
        "grasp_height_fraction": grasp_height_fraction,
        "pregrasp_clearance_m": pregrasp_clearance_m,
        "fit_error_mm": {
            "mean": round(float(errors.mean()), 4),
            "max": round(float(errors.max()), 4),
        },
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "measurement_note": "base_points_m은 실측값이며 이미지 클릭 순서와 같아야 함",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="카메라 대신 사용할 정지 이미지")
    parser.add_argument("--camera", default=DEFAULT_TOP)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--base-point",
        action="append",
        type=parse_xy,
        required=True,
        help="base_footprint 기준 실측 X,Y[m]. 클릭할 순서대로 최소 4회 지정",
    )
    parser.add_argument("--surface-z-m", type=float, required=True)
    parser.add_argument("--cup-height-m", type=float, required=True)
    parser.add_argument("--grasp-height-fraction", type=float, default=0.55)
    parser.add_argument("--pregrasp-clearance-m", type=float, default=0.060)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if len(args.base_point) < 4:
        raise SystemExit("--base-point를 최소 4회 지정해야 합니다")
    if not 0.2 <= args.grasp_height_fraction <= 0.8:
        raise SystemExit("--grasp-height-fraction은 0.2~0.8이어야 합니다")
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit(f"이미지를 읽을 수 없습니다: {args.image}")
    else:
        frame = capture_frame(args.camera, args.width, args.height)
    points = collect_image_points(frame, len(args.base_point))
    result = build_calibration(
        points,
        args.base_point,
        (frame.shape[1], frame.shape[0]),
        args.surface_z_m,
        args.cup_height_m,
        args.grasp_height_fraction,
        args.pregrasp_clearance_m,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"작업대 보정: {args.output}")
    print(f"평균/최대 맞춤 오차: {result['fit_error_mm']['mean']}/{result['fit_error_mm']['max']} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
