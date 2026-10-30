#!/usr/bin/env python3
"""Run the current taught map through the maintained JD-AMR executor."""

from __future__ import annotations

import importlib.util
import argparse
from pathlib import Path
import sys

from _common import (
    PC_WORKSPACE, REPO_ROOT, bootstrap_assets, map_session_pointer,
    prepare_user_service_environment,
)


HELPER_DIR = REPO_ROOT / "src/navigation/scripts/map_20260930_manual"


def _load_helper():
    prepare_user_service_environment()
    # The helper reads these during import.  setdefault keeps explicit operator
    # overrides authoritative.
    import os
    context = argparse.ArgumentParser(add_help=False)
    context.add_argument('--map-session', type=Path)
    context.add_argument('--legacy-map', action='store_true')
    selected, remaining = context.parse_known_args(sys.argv[1:])
    pointer = map_session_pointer(selected.map_session)
    readonly_context = any(arg in (
        '--help', '-h', 'stop', 'estop', 'session-stop', 'display-stop', 'status'
    ) for arg in remaining)
    if selected.legacy_map:
        bootstrap_assets(legacy=True)
    else:
        if not pointer.is_file() and not any(
                arg in ('--help', '-h', 'stop', 'estop', 'session-stop', 'display-stop', 'status')
                for arg in sys.argv[1:]):
            raise FileNotFoundError(
                f'current taught-map pointer is missing: {pointer}; '
                'select --map-session or use --legacy-map for archival access')
        if pointer.is_file():
            os.environ['JDAMR_MAP_SESSION'] = str(pointer)
    os.environ.setdefault("JDAMR_PC_WS", str(PC_WORKSPACE))
    sys.path.insert(0, str(REPO_ROOT / 'src/navigation/jdamr_cube_navigation'))
    sys.path.insert(0, str(HELPER_DIR))
    spec = importlib.util.spec_from_file_location(
        "jdamr_depart_operational", HELPER_DIR / "jdamr_depart.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the preserved JD-AMR departure helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # API callers and the CLI bind the same current map before any operation.
    # Importing this helper must never leave its archival defaults active.
    if not selected.legacy_map and pointer.is_file():
        if readonly_context:
            module.select_stop_state(pointer)
        else:
            module.configure_map_session(pointer)
    return module


def main() -> None:
    _load_helper().main()


if __name__ == "__main__":
    main()
