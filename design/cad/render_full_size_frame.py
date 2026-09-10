#!/usr/bin/env python3
"""현행 450×340 mm 상부·하부 프레임 URDF를 문서용 이미지로 렌더한다."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

from render_design_handoff import BLUE, INK, LINE, MUTED, WHITE, card, font, render_urdf


REPO_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = REPO_ROOT / "docs/assets/full_size_frame_20260910"


def compose(iso_path: Path, top_path: Path, output: Path) -> None:
    canvas = Image.new("RGB", (1920, 1240), WHITE)
    draw = ImageDraw.Draw(canvas)
    draw.text((70, 52), "HOLD THE FLOW · CURRENT GEOMETRY", font=font(23, "bold"), fill=BLUE)
    draw.text((70, 100), "하부와 같은 450×340 mm 상부 프레임", font=font(50, "bold"), fill=INK)
    draw.text(
        (70, 174),
        "커밋된 URDF·STL 실제 축척 · 4개 기둥 · 4분할 출력 상판 · SO-101 두 대",
        font=font(23),
        fill=MUTED,
    )

    panels = (
        (55, 245, 1120, 1070, iso_path, "조립 등각도"),
        (1150, 245, 1865, 1070, top_path, "상면 배치"),
    )
    for x1, y1, x2, y2, image_path, label in panels:
        card(draw, (x1, y1, x2, y2))
        source = Image.open(image_path).convert("RGB")
        source.thumbnail((x2 - x1 - 36, y2 - y1 - 100), Image.Resampling.LANCZOS)
        canvas.paste(source, (x1 + (x2 - x1 - source.width) // 2, y1 + 24))
        draw.text((x1 + 28, y2 - 58), label, font=font(22, "bold"), fill=INK)

    facts = (
        "외곽 450(W)×340(D) mm",
        "상판 윗면 720 mm",
        "팔 중심: 전면에서 150 mm",
        "카메라 광학 중심 z=970 mm",
    )
    x = 55
    for fact in facts:
        width = 445
        draw.rounded_rectangle((x, 1110, x + width, 1190), radius=16, fill="#F8FAFC", outline=LINE, width=2)
        draw.text((x + 20, 1135), fact, font=font(18, "medium"), fill=INK)
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
        top_path = temporary / "top.png"
        render_urdf(iso_path, positions, view="iso", parallel_scale=620)
        render_urdf(top_path, positions, view="top")
        compose(iso_path, top_path, OUTPUT_DIR / "model_overview.png")
    print(OUTPUT_DIR / "model_overview.png")


if __name__ == "__main__":
    main()
