#!/usr/bin/env python3
"""현행 450×340 mm 상부·하부 프레임 URDF를 문서용 이미지로 렌더한다."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw

from render_design_handoff import BLUE, INK, LINE, MUTED, WHITE, card, font, render_urdf


REPO_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = REPO_ROOT / "docs/assets/full_size_frame_20260910"


def paste_view(
    canvas: Image.Image,
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    image_path: Path,
    label: str,
) -> None:
    x1, y1, x2, y2 = box
    card(draw, box)
    source = Image.open(image_path).convert("RGB")
    difference = ImageChops.difference(source, Image.new("RGB", source.size, WHITE))
    content_box = difference.getbbox()
    if content_box is not None:
        left = max(0, content_box[0] - 20)
        top = max(0, content_box[1] - 20)
        right = min(source.width, content_box[2] + 20)
        bottom = min(source.height, content_box[3] + 20)
        source = source.crop((left, top, right, bottom))
    source.thumbnail((x2 - x1 - 30, y2 - y1 - 82), Image.Resampling.LANCZOS)
    canvas.paste(source, (x1 + (x2 - x1 - source.width) // 2, y1 + 16))
    draw.text((x1 + 24, y2 - 50), label, font=font(20, "bold"), fill=INK)


def compose(iso_path: Path, front_path: Path, top_path: Path, output: Path) -> None:
    canvas = Image.new("RGB", (2000, 1420), WHITE)
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 52), "HOLD THE FLOW · CURRENT GEOMETRY", font=font(23, "bold"), fill=BLUE)
    draw.text((70, 100), "450×340 mm 전체 모델 · SO-101 좌우 끝 배치", font=font(48, "bold"), fill=INK)
    draw.text(
        (70, 174),
        "커밋된 URDF·STL 실제 축척 · 잘림 없는 등각·정면·상면도",
        font=font(23),
        fill=MUTED,
    )

    paste_view(canvas, draw, (55, 245, 1180, 1235), iso_path, "등각도 · 전체 높이와 하부 장착부")
    paste_view(canvas, draw, (1210, 245, 1945, 730), front_path, "정면도 · 팔 중심 y=±170 mm")
    paste_view(canvas, draw, (1210, 750, 1945, 1235), top_path, "상면도 · 좌우 끝 20 mm 어댑터 여유")

    facts = (
        "프레임 450(W)×340(D)",
        "상판 윗면 z=720 mm",
        "팔 중심 (20, ±170, 726)",
        "Astra 광학 중심 z=970 mm",
    )
    x = 55
    for fact in facts:
        width = 445
        draw.rounded_rectangle((x, 1270, x + width, 1360), radius=16, fill="#F8FAFC", outline=LINE, width=2)
        draw.text((x + 20, 1300), fact, font=font(18, "medium"), fill=INK)
        x += width + 20
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=95)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    positions = {}
    for side, pan_sign in (("left", 1.0), ("right", -1.0)):
        positions.update(
            {
                f"{side}_shoulder_pan": math.radians(12.0 * pan_sign),
                f"{side}_shoulder_lift": math.radians(-68.0),
                f"{side}_elbow_flex": math.radians(92.0),
                f"{side}_wrist_flex": math.radians(-22.0),
            }
        )
    with tempfile.TemporaryDirectory(prefix="hold-flow-full-size-") as directory:
        temporary = Path(directory)
        iso_path = temporary / "iso.png"
        front_path = temporary / "front.png"
        top_path = temporary / "top.png"
        render_urdf(iso_path, positions, view="iso", parallel_scale=650)
        render_urdf(front_path, positions, view="front", parallel_scale=590)
        render_urdf(top_path, positions, view="top", parallel_scale=430)
        compose(iso_path, front_path, top_path, OUTPUT_DIR / "model_overview.png")
    print(OUTPUT_DIR / "model_overview.png")


if __name__ == "__main__":
    main()
