#!/usr/bin/env python3
"""Atomically publish a validated JD-AMR stage on the Pi.

This helper changes filesystem slots only.  It never starts services or sends ROS
commands.  The validated stage and timestamped backup remain in place after success.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil


MUTABLE_ASSETS = {"state.json", "rviz_initialpose.json"}


class Change:
    def __init__(self, live: Path, backup: Path | None):
        self.live = live
        self.backup = backup
        self.backed_up = False
        self.published = False


def _exists(path: Path) -> bool:
    return os.path.lexists(path)


def _absolute(path: Path, label: str) -> Path:
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{label} must be an absolute normalized path: {path}")
    return path


def _inside(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve(strict=True)
    resolved.relative_to(root.resolve(strict=True))
    if not resolved.is_dir():
        raise ValueError(f"{label} is not a directory: {resolved}")
    return resolved


def _remove_published(path: Path) -> None:
    if not _exists(path):
        return
    if path.is_symlink() or path.is_file():
        path.unlink()
        return
    raise RuntimeError(f"refusing to remove unexpected published directory: {path}")


def _rollback(changes: list[Change]) -> None:
    errors: list[str] = []
    for change in reversed(changes):
        try:
            if change.published:
                _remove_published(change.live)
            if change.backed_up and change.backup is not None and _exists(change.backup):
                change.live.parent.mkdir(parents=True, exist_ok=True)
                os.replace(change.backup, change.live)
        except Exception as exc:  # best effort across every already-published slot
            errors.append(f"{change.live}: {exc}")
    if errors:
        raise RuntimeError("rollback incomplete: " + "; ".join(errors))


def promote(
    *,
    stage_root: Path,
    workspace: Path,
    tools_live: Path,
    assets_live: Path,
    mount_live: Path,
    backup_root: Path,
    replace_assets: bool = False,
    fail_after: int | None = None,
) -> dict[str, object]:
    """Publish stage slots and rollback all changed paths if any step fails."""
    stage_root = _absolute(stage_root, "stage_root").resolve(strict=True)
    workspace = _absolute(workspace, "workspace")
    tools_live = _absolute(tools_live, "tools_live")
    assets_live = _absolute(assets_live, "assets_live")
    mount_live = _absolute(mount_live, "mount_live")
    backup_root = _absolute(backup_root, "backup_root")
    if _exists(backup_root):
        raise FileExistsError(f"backup root already exists: {backup_root}")

    stage_source = _inside(stage_root / "src/jdamr_cube_ros", stage_root, "stage source")
    stage_install = _inside(stage_root / "install", stage_root, "stage install")
    stage_tools = _inside(stage_root / "tools", stage_root, "stage tools")
    stage_assets = _inside(stage_root / "assets", stage_root, "stage assets")
    stage_mount = (stage_root / "camera_mount_current.yaml").resolve(strict=True)
    stage_mount.relative_to(stage_root)
    if not stage_mount.is_file():
        raise ValueError(f"stage camera mount is not a file: {stage_mount}")

    workspace.mkdir(parents=True, exist_ok=True)
    lock_path = workspace / ".jdamr_deploy.lock"
    lock_handle = lock_path.open("a+")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_handle.close()
        raise RuntimeError(
            f"another JD-AMR promotion holds the workspace lock: {lock_path}"
        ) from exc

    changes: list[Change] = []
    operations = 0

    def checkpoint() -> None:
        nonlocal operations
        operations += 1
        if fail_after is not None and operations >= fail_after:
            raise RuntimeError(f"injected failure after operation {operations}")

    def publish_link(target: Path, live: Path, name: str) -> None:
        backup = backup_root / "slots" / name if _exists(live) else None
        change = Change(live=live, backup=backup)
        changes.append(change)
        if backup is not None:
            backup.parent.mkdir(parents=True, exist_ok=True)
            os.replace(live, backup)
            change.backed_up = True
            checkpoint()
        live.parent.mkdir(parents=True, exist_ok=True)
        temporary = live.parent / f".{live.name}.next-{backup_root.name}"
        if _exists(temporary):
            raise FileExistsError(f"temporary publish path exists: {temporary}")
        try:
            os.symlink(target, temporary, target_is_directory=True)
            os.replace(temporary, live)
            change.published = True
        finally:
            if _exists(temporary):
                temporary.unlink()
        checkpoint()

    def publish_file(source: Path, live: Path, relative_backup: Path) -> None:
        backup = backup_root / relative_backup if _exists(live) else None
        change = Change(live=live, backup=backup)
        changes.append(change)
        if backup is not None:
            backup.parent.mkdir(parents=True, exist_ok=True)
            os.replace(live, backup)
            change.backed_up = True
            checkpoint()
        live.parent.mkdir(parents=True, exist_ok=True)
        temporary = live.parent / f".{live.name}.next-{backup_root.name}"
        try:
            shutil.copy2(source, temporary)
            os.replace(temporary, live)
            change.published = True
        finally:
            if _exists(temporary):
                temporary.unlink()
        checkpoint()

    preserved_assets: list[str] = []
    published_assets: list[str] = []
    try:
        publish_link(stage_source, workspace / "src/jdamr_cube_ros", "source")
        publish_link(stage_install, workspace / "install", "install")
        publish_link(stage_tools, tools_live, "tools")

        for source in sorted(stage_assets.iterdir()):
            if not source.is_file() or source.name in MUTABLE_ASSETS:
                continue
            live = assets_live / source.name
            if _exists(live) and not replace_assets:
                preserved_assets.append(source.name)
                continue
            publish_file(source, live, Path("assets") / source.name)
            published_assets.append(source.name)

        if _exists(mount_live) and not replace_assets:
            preserved_assets.append(mount_live.name)
        else:
            publish_file(stage_mount, mount_live, Path("mount") / mount_live.name)

        backup_root.mkdir(parents=True, exist_ok=True)
        receipt = {
            "stage_root": str(stage_root),
            "source": str(workspace / "src/jdamr_cube_ros"),
            "install": str(workspace / "install"),
            "tools": str(tools_live),
            "replace_assets": replace_assets,
            "published_assets": published_assets,
            "preserved_assets": preserved_assets,
        }
        (backup_root / "promotion_receipt.json").write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n"
        )
        return receipt
    except BaseException:
        _rollback(changes)
        raise
    finally:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        lock_handle.close()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--stage-root", type=Path, required=True)
    result.add_argument("--workspace", type=Path, required=True)
    result.add_argument("--tools-live", type=Path, required=True)
    result.add_argument("--assets-live", type=Path, required=True)
    result.add_argument("--mount-live", type=Path, required=True)
    result.add_argument("--backup-root", type=Path, required=True)
    result.add_argument("--replace-assets", action="store_true")
    return result


def main() -> None:
    args = parser().parse_args()
    receipt = promote(**vars(args))
    print(json.dumps(receipt, sort_keys=True))


if __name__ == "__main__":
    main()
