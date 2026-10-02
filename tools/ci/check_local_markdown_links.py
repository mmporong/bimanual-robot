#!/usr/bin/env python3
"""저장소 Markdown의 로컬 파일 링크가 실제 경로를 가리키는지 검사한다."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


MARKDOWN_LINK = re.compile(r"!?\[[^\]]*\]\((<[^>]+>|[^)\s]+)(?:\s+['\"].*?['\"])?\)")
HTML_LINK = re.compile(r"(?:href|src)\s*=\s*['\"]([^'\"]+)['\"]", re.IGNORECASE)
IGNORED_SCHEMES = {"http", "https", "mailto", "data", "javascript", "tel"}


def tracked_markdown(repo_root: Path) -> list[Path]:
    output = subprocess.check_output(
        ["git", "ls-files", "-z", "--", "*.md"], cwd=repo_root
    )
    return [repo_root / item.decode() for item in output.split(b"\0") if item]


def local_target(raw_target: str) -> str | None:
    target = raw_target.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1]
    parsed = urlsplit(target)
    if parsed.scheme.lower() in IGNORED_SCHEMES or target.startswith("#"):
        return None
    path = unquote(parsed.path)
    return path or None


def check_file(markdown: Path, repo_root: Path) -> list[str]:
    text = markdown.read_text(encoding="utf-8")
    failures: list[str] = []
    targets = [match.group(1) for match in MARKDOWN_LINK.finditer(text)]
    targets.extend(match.group(1) for match in HTML_LINK.finditer(text))
    for raw_target in targets:
        target = local_target(raw_target)
        if target is None:
            continue
        candidate = (repo_root / target.lstrip("/")) if target.startswith("/") else (markdown.parent / target)
        try:
            resolved = candidate.resolve()
            resolved.relative_to(repo_root.resolve())
        except (OSError, ValueError):
            failures.append(f"{markdown.relative_to(repo_root)}: 저장소 밖 경로: {raw_target}")
            continue
        if not resolved.exists():
            failures.append(f"{markdown.relative_to(repo_root)}: 누락된 경로: {raw_target}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    args = parser.parse_args()
    repo_root = args.repo.resolve()
    failures = [
        failure
        for markdown in tracked_markdown(repo_root)
        for failure in check_file(markdown, repo_root)
    ]
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"Markdown local links OK: {len(tracked_markdown(repo_root))} files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
