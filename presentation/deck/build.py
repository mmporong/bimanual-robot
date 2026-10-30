"""index.html을 읽어 ../media/ 이미지를 base64로 인라인한 자기완결형 HTML을 만든다.
Artifact로 다시 배포할 때만 이 결과물(dist_artifact.html)을 쓴다.
평소 편집·미리보기는 index.html을 직접 브라우저로 열면 된다.

사용법:
    python build.py
"""
import base64
import mimetypes
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "index.html"
OUT = HERE / "dist_artifact.html"

IMG_TAG = re.compile(r'src="(\.\./media/[^"]+)"')


def to_data_uri(rel_path: str) -> str:
    abs_path = (HERE / rel_path).resolve()
    mime = mimetypes.guess_type(abs_path.name)[0] or "application/octet-stream"
    data = abs_path.read_bytes()
    b64 = base64.b64encode(data).decode("ascii")
    return f"data:{mime};base64,{b64}"


def main():
    html = SRC.read_text(encoding="utf-8")

    def repl(m):
        rel = m.group(1)
        try:
            return f'src="{to_data_uri(rel)}"'
        except FileNotFoundError:
            print("WARNING: file not found, left as-is:", rel)
            return m.group(0)

    html2 = IMG_TAG.sub(repl, html)
    OUT.write_text(html2, encoding="utf-8")
    print(f"done: {len(html2):,} bytes -> {OUT}")


if __name__ == "__main__":
    main()
