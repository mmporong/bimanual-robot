#!/usr/bin/env python3
"""Repository entry point for the preserved JD-AMR departure runbook.

The movement and localization implementation remains in the imported JD-AMR source
tree.  This entry point only prepares missing static assets and hands control to that
single maintained helper.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from _common import (
    PC_WORKSPACE, REPO_ROOT, bootstrap_assets, prepare_user_service_environment,
)


HELPER_DIR = REPO_ROOT / "src/navigation/scripts/map_20260930_manual"


def _load_helper():
    prepare_user_service_environment()
    bootstrap_assets()
    # The helper reads these during import.  setdefault keeps explicit operator
    # overrides authoritative.
    import os
    os.environ.setdefault("JDAMR_PC_WS", str(PC_WORKSPACE))
    sys.path.insert(0, str(HELPER_DIR))
    spec = importlib.util.spec_from_file_location(
        "jdamr_depart_operational", HELPER_DIR / "jdamr_depart.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the preserved JD-AMR departure helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def main() -> None:
    _load_helper().main()


if __name__ == "__main__":
    main()
