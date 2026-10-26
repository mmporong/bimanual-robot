#!/usr/bin/env python3
"""Build a static Netlify bundle of the guest (/order) and operator (/control) pages.

The bundle runs in browser demo mode by default. Pass --api-base to point the pages at a
running service_order_server instead (that server needs --cors-origin set to the site URL).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

STATIC = Path(__file__).resolve().parent / "static"
PAGES = ("order.html", "order.js", "control.html", "control.js", "api.js")


def build(out: Path, api_base: str) -> None:
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    for name in PAGES:
        shutil.copy2(STATIC / name, out / name)
    (out / "config.js").write_text(f"window.ORDER_API_BASE = {json.dumps(api_base)};\n", encoding="utf-8")
    # 루트 주소는 손님 주문 화면으로 보낸다. /order, /control 은 Netlify가 .html 없이 연다.
    (out / "_redirects").write_text("/  /order  302\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="output directory (deleted and recreated)")
    parser.add_argument("--api-base", default="demo", help='"demo" or an https URL of service_order_server')
    args = parser.parse_args(argv)
    build(args.out, args.api_base.rstrip("/"))
    print(f"netlify bundle: {args.out} (api base: {args.api_base})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
