import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("camera_dashboard.py")
SPEC = importlib.util.spec_from_file_location("camera_dashboard", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_dashboard_html_is_read_only_and_has_both_streams():
    html = MODULE.INDEX.read_text(encoding="utf-8")
    assert "/stream/top" in html
    assert "/stream/wrist" in html
    assert "읽기 전용" in html
    assert "/cmd" not in html


def test_default_camera_paths_are_stable_by_path():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "/dev/v4l/by-path/pci-0000:65:00.3-usb-0:2.1:1.0-video-index0" in source
    assert "/dev/v4l/by-path/pci-0000:65:00.3-usb-0:4:1.0-video-index0" in source
