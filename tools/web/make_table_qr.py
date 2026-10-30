"""테이블 QR 만들기: 테이블마다 손님 주문 화면(/order?table=N)을 여는 QR과 인쇄용 시트(HTML·PDF).

    pip install segno   # 순수 파이썬 QR 라이브러리(의존성 없음)
    # 발표 데모(Netlify, 로봇 미연결)
    python tools/web/make_table_qr.py --out qr_demo
    # 실제 운영(노트북 주문 서버, 손님 폰은 같은 Wi-Fi). 노트북 IP는 네트워크마다 바뀌므로 현장에서 다시 만든다.
    python tools/web/make_table_qr.py --base http://192.168.0.15:8765 --out qr_local

출력: table_N.png(QR만), table_N.svg, qr_sheet.html(A4 한 장에 테이블 카드 4개), --pdf 를 주면 Edge로 PDF까지.
"""
from __future__ import annotations

import argparse
import html
from pathlib import Path
import shutil
import subprocess

import segno

STATIC = Path(__file__).resolve().parents[2] / "apps/service_order/static"
EDGE = Path("C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe")


def card(table: int, url: str, svg: str) -> str:
    return f"""<section class="card">
  <p class="brand">MS RESTAURANT</p>
  <p class="table">{table}번 테이블</p>
  <div class="qr">{svg}</div>
  <p class="hint">카메라로 QR을 찍어 주문하세요</p>
  <p class="sub">음료는 서빙 로봇이 테이블까지 가져다드려요</p>
  <p class="url">{html.escape(url)}</p>
</section>"""


def sheet(cards: list[str], note: str) -> str:
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>테이블 QR</title>
<style>
  @font-face {{ font-family: "Jua"; src: url("fonts/jua.woff") format("woff"); }}
  @page {{ size: A4; margin: 10mm; }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; font-family: "Malgun Gothic", "Apple SD Gothic Neo", sans-serif; color: #1f3553; }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8mm; }}
  .card {{ height: 132mm; padding: 8mm 6mm; border: 1.2mm solid #1677c8; border-radius: 6mm; text-align: center; break-inside: avoid; }}
  .brand {{ margin: 0; font-family: "Jua", sans-serif; font-size: 7mm; color: #1677c8; letter-spacing: .04em; }}
  .table {{ margin: 1mm 0 3mm; font-family: "Jua", sans-serif; font-size: 11mm; }}
  .qr svg {{ width: 62mm; height: 62mm; }}
  .hint {{ margin: 2mm 0 0; font-family: "Jua", sans-serif; font-size: 5.5mm; }}
  .sub {{ margin: 1mm 0 0; font-size: 3.4mm; color: #5a6b80; }}
  .url {{ margin: 2mm 0 0; font-size: 2.6mm; color: #9aa9ba; word-break: break-all; }}
  .note {{ margin: 0 0 4mm; font-size: 3mm; color: #9aa9ba; }}
</style></head>
<body><p class="note">{html.escape(note)}</p><div class="grid">
{chr(10).join(cards)}
</div></body></html>
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default="https://ms-water-serving.netlify.app", help="주문 화면이 있는 주소(끝의 / 없이)")
    ap.add_argument("--tables", default="1,2,3,4")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--pdf", action="store_true", help="Edge(headless)로 qr_sheet.pdf 도 만든다")
    args = ap.parse_args()
    base = args.base.rstrip("/")
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "fonts").mkdir(exist_ok=True)
    shutil.copy2(STATIC / "fonts/jua.woff", args.out / "fonts/jua.woff")
    cards = []
    for table in (int(t) for t in args.tables.split(",")):
        url = f"{base}/order?table={table}"
        # 오류 정정 H(약 30% 가려져도 읽힘): 인쇄물이 접히거나 물에 젖어도 찍히게
        qr = segno.make(url, error="h")
        qr.save(args.out / f"table_{table}.png", scale=16, border=2, dark="#1f3553")
        qr.save(args.out / f"table_{table}.svg", scale=10, border=2, dark="#1f3553", xmldecl=False, svgns=True, omitsize=True)  # viewBox만 남겨 카드 크기에 맞게 늘어나게
        svg = (args.out / f"table_{table}.svg").read_text(encoding="utf-8")
        cards.append(card(table, url, svg))
        print(f"table {table}: {url}")
    note = f"주문 주소: {base}/order?table=N · 데모(Netlify)는 로봇과 연결되지 않습니다." if "netlify" in base else f"주문 주소: {base}/order?table=N · 손님 폰이 노트북과 같은 Wi-Fi에 있어야 합니다."
    page = args.out / "qr_sheet.html"
    page.write_text(sheet(cards, note), encoding="utf-8")
    print(f"sheet: {page}")
    if args.pdf:
        pdf = args.out / "qr_sheet.pdf"
        subprocess.run([str(EDGE), "--headless", "--disable-gpu", "--no-pdf-header-footer",
                        f"--print-to-pdf={pdf.resolve()}", page.resolve().as_uri()], check=True, timeout=120)
        print(f"pdf: {pdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
