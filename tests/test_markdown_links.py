from pathlib import Path

from tools.ci.check_local_markdown_links import check_file, local_target


def test_local_target_filters_remote_and_anchor_links() -> None:
    assert local_target("https://example.com/a") is None
    assert local_target("#section") is None
    assert local_target("docs/example.md#section") == "docs/example.md"
    assert local_target("<assets/file%20name.png>") == "assets/file name.png"


def test_check_file_reports_only_missing_local_paths(tmp_path: Path) -> None:
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "exists.md").write_text("ok", encoding="utf-8")
    page = docs / "page.md"
    page.write_text(
        "[ok](exists.md)\n[remote](https://example.com)\n[missing](missing.md)\n",
        encoding="utf-8",
    )
    failures = check_file(page, tmp_path)
    assert failures == ["docs/page.md: 누락된 경로: missing.md"]
