#!/usr/bin/env python3
"""RGB 카메라 ChArUco 보드 생성·프레임 수집·intrinsic 보정 도구."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import time

import cv2
import numpy as np
import yaml


DEFAULT_TOP = "/dev/v4l/by-path/pci-0000:65:00.3-usb-0:2.1:1.0-video-index0"
DEFAULT_BOARD_DIR = pathlib.Path("calibration/cameras/boards")
DEFAULT_CAPTURE_DIR = pathlib.Path("calibration/cameras/top/samples")
DEFAULT_OUTPUT = pathlib.Path("calibration/cameras/top/intrinsics.yaml")


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def board(squares_x: int, squares_y: int, square_mm: float, marker_mm: float):
    return cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_mm / 1000.0, marker_mm / 1000.0, dictionary()
    )


def render_board(board_obj, width_px: int, height_px: int) -> np.ndarray:
    size = (width_px, height_px)
    if not hasattr(board_obj, "generateImage"):
        raise RuntimeError(
            "OpenCV 4.7 이상이 필요합니다. "
            "$HOME/miniforge3/envs/lerobot/bin/python 으로 실행하세요"
        )
    return board_obj.generateImage(size, marginSize=0, borderBits=1)


def detect_charuco(image: np.ndarray, board_obj):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board_obj)
        return detector.detectBoard(gray)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary())
    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners, marker_ids
    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board_obj
    )
    return charuco_corners, charuco_ids, marker_corners, marker_ids


def command_generate(args: argparse.Namespace) -> int:
    dpi = args.dpi
    a4_width_mm, a4_height_mm = 297.0, 210.0
    page_width = round(a4_width_mm / 25.4 * dpi)
    page_height = round(a4_height_mm / 25.4 * dpi)
    board_width_mm = args.squares_x * args.square_mm
    board_height_mm = args.squares_y * args.square_mm
    if board_width_mm > a4_width_mm or board_height_mm > a4_height_mm:
        raise SystemExit("보드가 A4 가로 용지를 넘습니다")
    board_width = round(board_width_mm / 25.4 * dpi)
    board_height = round(board_height_mm / 25.4 * dpi)
    rendered = render_board(
        board(args.squares_x, args.squares_y, args.square_mm, args.marker_mm),
        board_width,
        board_height,
    )
    page = np.full((page_height, page_width), 255, np.uint8)
    x = (page_width - board_width) // 2
    y = (page_height - board_height) // 2
    page[y : y + board_height, x : x + board_width] = rendered
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), page):
        raise SystemExit(f"파일 저장 실패: {args.output}")
    metadata = {
        "schema_version": "1.0",
        "dictionary": "DICT_4X4_50",
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_length_mm": args.square_mm,
        "marker_length_mm": args.marker_mm,
        "board_size_mm": [board_width_mm, board_height_mm],
        "page": "A4_landscape",
        "dpi": dpi,
        "print_scale": "100_percent_actual_size",
        "verify_after_print": "adjacent chessboard corners must be 30.0 mm apart",
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(f"보드: {args.output}")
    print(f"메타데이터: {metadata_path}")
    print(f"100% 실제 크기로 인쇄 후 한 칸이 {args.square_mm:.1f} mm인지 자로 확인")
    return 0


def command_capture(args: argparse.Namespace) -> int:
    board_obj = board(args.squares_x, args.squares_y, args.square_mm, args.marker_mm)
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    if not capture.isOpened():
        raise SystemExit(f"카메라를 열 수 없습니다: {args.camera}")
    args.output.mkdir(parents=True, exist_ok=True)
    saved = sorted(args.output.glob("frame_*.png"))
    print("보드를 화면 중앙·모서리·거리·기울기를 바꿔 보여주세요.")
    print("SPACE 저장 · Q 종료 · 최소 코너", args.min_corners)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                continue
            corners, ids, marker_corners, marker_ids = detect_charuco(frame, board_obj)
            shown = frame.copy()
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(shown, marker_corners, marker_ids)
            count = 0 if ids is None else len(ids)
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(shown, corners, ids)
            color = (30, 210, 90) if count >= args.min_corners else (40, 120, 240)
            cv2.putText(
                shown,
                f"corners {count} | saved {len(saved)}/{args.target}",
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("top RGB intrinsic calibration", shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32:
                if count < args.min_corners:
                    print(f"저장 거부: 검출 코너 {count} < {args.min_corners}")
                    continue
                timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                path = args.output / f"frame_{timestamp}.png"
                cv2.imwrite(str(path), frame)
                saved.append(path)
                print(f"저장 {len(saved)}/{args.target}: {path.name} ({count} corners)")
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


def command_calibrate(args: argparse.Namespace) -> int:
    board_obj = board(args.squares_x, args.squares_y, args.square_mm, args.marker_mm)
    image_paths = sorted(args.samples.glob("frame_*.png"))
    if len(image_paths) < args.min_images:
        raise SystemExit(f"유효 이미지가 부족합니다: {len(image_paths)} < {args.min_images}")
    all_corners, all_ids = [], []
    image_size = None
    used = []
    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        corners, ids, _, _ = detect_charuco(image, board_obj)
        if ids is None or len(ids) < args.min_corners:
            continue
        all_corners.append(corners)
        all_ids.append(ids)
        used.append(path.name)
    if len(used) < args.min_images or image_size is None:
        raise SystemExit(f"검출 기준을 통과한 이미지가 부족합니다: {len(used)} < {args.min_images}")
    if hasattr(cv2.aruco, "calibrateCameraCharuco"):
        error, camera_matrix, distortion, _, _ = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, board_obj, image_size, None, None
        )
    else:
        object_points, image_points = [], []
        for corners, ids in zip(all_corners, all_ids):
            matched_object, matched_image = board_obj.matchImagePoints(corners, ids)
            object_points.append(matched_object)
            image_points.append(matched_image)
        error, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
            object_points, image_points, image_size, None, None
        )
    result = {
        "schema_version": "1.0",
        "camera_role": "top",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "mean_reprojection_error_px": float(error),
        "used_images": used,
        "board": {
            "dictionary": "DICT_4X4_50",
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_mm,
            "marker_length_mm": args.marker_mm,
        },
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    print(f"intrinsic: {args.output}")
    print(f"사용 이미지: {len(used)}")
    print(f"평균 reprojection error: {error:.4f} px")
    return 0 if error <= args.max_error else 2


def common_board_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=30.0)
    parser.add_argument("--marker-mm", type=float, default=22.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    generate = commands.add_parser("generate", help="A4 가로 인쇄 보드 생성")
    common_board_args(generate)
    generate.add_argument("--dpi", type=int, default=300)
    generate.add_argument(
        "--output", type=pathlib.Path,
        default=DEFAULT_BOARD_DIR / "charuco_7x5_30mm_22mm_a4_300dpi.png",
    )
    generate.set_defaults(run=command_generate)

    capture = commands.add_parser("capture", help="상단 RGB 보정 프레임 수집")
    common_board_args(capture)
    capture.add_argument("--camera", default=DEFAULT_TOP)
    capture.add_argument("--width", type=int, default=640)
    capture.add_argument("--height", type=int, default=480)
    capture.add_argument("--fps", type=int, default=30)
    capture.add_argument("--min-corners", type=int, default=12)
    capture.add_argument("--target", type=int, default=20)
    capture.add_argument("--output", type=pathlib.Path, default=DEFAULT_CAPTURE_DIR)
    capture.set_defaults(run=command_capture)

    calibrate = commands.add_parser("calibrate", help="저장 프레임으로 intrinsic 계산")
    common_board_args(calibrate)
    calibrate.add_argument("--samples", type=pathlib.Path, default=DEFAULT_CAPTURE_DIR)
    calibrate.add_argument("--output", type=pathlib.Path, default=DEFAULT_OUTPUT)
    calibrate.add_argument("--min-images", type=int, default=15)
    calibrate.add_argument("--min-corners", type=int, default=12)
    calibrate.add_argument("--max-error", type=float, default=1.0)
    calibrate.set_defaults(run=command_calibrate)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.run(args)


if __name__ == "__main__":
    raise SystemExit(main())
