#!/usr/bin/env python3
"""Shared paths and non-destructive runtime asset setup for JD-AMR tools."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_SOURCE = REPO_ROOT / "config/navigation/jdamr_runtime"
DEFAULT_DATA_ROOT = Path.home() / "jdamr_data"
DATA_ROOT = Path(os.environ.get("JDAMR_DATA_ROOT", DEFAULT_DATA_ROOT)).expanduser()
LEGACY_RUNTIME_ASSETS = DATA_ROOT / "map_20260930_manual/aligned"
RUNTIME_ROOT = Path(
    os.environ.get("JDAMR_RUNTIME_ROOT", REPO_ROOT / ".runtime/jdamr")
).expanduser()
PC_WORKSPACE = Path(
    os.environ.get("JDAMR_PC_WS", RUNTIME_ROOT / "pc_ws")
).expanduser()


def map_session_pointer(value: str | Path | None = None) -> Path:
    return Path(value or os.environ.get(
        "JDAMR_MAP_SESSION", DATA_ROOT / "maps/current_manual_mapping.json"
    )).expanduser().resolve()


def runtime_assets(*, map_session: str | Path | None = None,
                   legacy: bool = False) -> Path:
    if legacy:
        return LEGACY_RUNTIME_ASSETS
    pointer = map_session_pointer(map_session)
    if not pointer.is_file():
        raise FileNotFoundError(
            f"current map pointer is missing: {pointer}; select --map-session "
            "or use --legacy-map for archival access")
    document = json.loads(pointer.read_text())
    base = Path(os.path.expandvars(document["directory"])).expanduser()
    if not base.is_absolute():
        base = pointer.parent / base
    return base.resolve() / "runtime"


def prepare_user_service_environment(runtime_dir: Path | None = None) -> bool:
    """Use the current user's existing bus when a terminal lacks session variables."""
    runtime_dir = runtime_dir or Path(
        os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    )
    bus = runtime_dir / 'bus'
    try:
        info = bus.stat()
    except OSError:
        return False
    if info.st_uid != os.getuid() or not stat.S_ISSOCK(info.st_mode):
        return False
    os.environ.setdefault('XDG_RUNTIME_DIR', str(runtime_dir))
    os.environ.setdefault('DBUS_SESSION_BUS_ADDRESS', f'unix:path={bus}')
    return True


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bootstrap_assets(*, refresh: bool = False, now: str | None = None,
                     map_session: str | Path | None = None,
                     legacy: bool = False) -> dict[str, object]:
    """Prepare the selected taught map; archival bundles require explicit opt-in."""
    if not legacy:
        sys.path.insert(0, str(REPO_ROOT / "src/navigation/jdamr_cube_navigation"))
        from jdamr_cube_navigation.map_session_assets import prepare_map_session

        pointer = map_session_pointer(map_session)
        profile = prepare_map_session(pointer)
        return {"asset_root": profile["runtime_directory"],
                "map_session": str(pointer), "mode": "current_taught_map",
                "station_ids": profile["station_ids"],
                "map_hashes": profile["map_hashes"]}
    if not ASSET_SOURCE.is_dir():
        raise FileNotFoundError(f"bundled JD-AMR assets are missing: {ASSET_SOURCE}")

    asset_root = runtime_assets(legacy=True)
    asset_root.mkdir(parents=True, exist_ok=True)
    result: dict[str, list[str]] = {"created": [], "unchanged": [], "preserved": [], "backed_up": []}
    stamp = now or time.strftime("%Y%m%d_%H%M%S")
    backup_root = RUNTIME_ROOT / "asset_backups" / stamp

    for source in sorted(ASSET_SOURCE.iterdir()):
        if not source.is_file():
            continue
        target = asset_root / source.name
        if not target.exists():
            shutil.copy2(source, target)
            result["created"].append(source.name)
            continue
        if sha256(source) == sha256(target):
            result["unchanged"].append(source.name)
            continue
        if not refresh:
            result["preserved"].append(source.name)
            continue
        backup_root.mkdir(parents=True, exist_ok=True)
        backup = backup_root / source.name
        shutil.copy2(target, backup)
        shutil.copy2(source, target)
        result["backed_up"].append(str(backup))
    return {"asset_root": str(asset_root), "mode": "legacy", **result}
