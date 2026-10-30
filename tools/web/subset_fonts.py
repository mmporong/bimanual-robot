"""손님 주문 화면용 부분 글꼴 만들기: 주아체(Jua)·고운돋움(Gowun Dodum)에서 화면에 쓰는 글자만 남긴다.

원본 TTF(각 2MB·7MB)는 커밋하지 않는다. Google Fonts 저장소(ofl/jua, ofl/gowundodum)에서 받아 경로를 준다.
    python tools/web/subset_fonts.py --jua Jua-Regular.ttf --gowun GowunDodum-Regular.ttf
화면 문구를 바꾸면 다시 실행한다. 빠진 글자는 시스템 글꼴로 나온다.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from fontTools import subset

STATIC = Path(__file__).resolve().parents[2] / "apps/service_order/static"
SOURCES = ("order.html", "order.js", "api.js")
# 화면에 그대로 안 적혀 있어도 나올 수 있는 글자: 영문·숫자·기호, 테이블 번호 등
EXTRA = "".join(chr(c) for c in range(0x20, 0x7F)) + "×−·…→✓원번테이블개외잔무료"


def page_text() -> str:
    return "".join(sorted(set("".join((STATIC / name).read_text(encoding="utf-8") for name in SOURCES) + EXTRA)))


def build(src: Path, out: Path, text: str) -> None:
    options = subset.Options()
    options.flavor = "woff"
    options.layout_features = ["*"]
    font = subset.load_font(str(src), options)
    subsetter = subset.Subsetter(options)
    subsetter.populate(text=text)
    subsetter.subset(font)
    out.parent.mkdir(parents=True, exist_ok=True)
    subset.save_font(font, str(out), options)
    print(f"{out.name}: {out.stat().st_size // 1024} KB")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jua", type=Path, required=True)
    ap.add_argument("--gowun", type=Path, required=True)
    args = ap.parse_args()
    text = page_text()
    build(args.jua, STATIC / "fonts/jua.woff", text)
    build(args.gowun, STATIC / "fonts/gowun_dodum.woff", text)


if __name__ == "__main__":
    main()
