import argparse
import math
import pathlib
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from ammr_dashboard import (TelemetryStore, decode_depth, depth_summary,
                            finite_argument, scan_summary)  # noqa: E402


class DashboardTelemetryTest(unittest.TestCase):
    def test_scan_summary_filters_invalid_and_reports_sectors(self):
        ranges = [1.0, float("inf"), 0.5, float("nan"), 2.0]
        result = scan_summary(ranges, -math.pi, math.pi / 2, 0.1, 8.0)
        self.assertEqual(result["valid_points"], 3)
        self.assertEqual(result["min_m"], 0.5)
        self.assertEqual(result["front_m"], 0.5)
        self.assertEqual(len(result["points_xy_m"]), 3)

    def test_scan_summary_rotates_reverse_mounted_lidar(self):
        result = scan_summary([1.0], math.pi, 0.1, 0.1, 8.0,
                              yaw_deg=180.0)
        self.assertEqual(result["front_m"], 1.0)
        self.assertEqual(result["points_xy_m"], [[1.0, 0.0]])

    def test_scan_summary_applies_display_origin_translation(self):
        result = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0, x_m=-0.25)
        self.assertEqual(result["front_m"], 0.75)
        self.assertEqual(result["points_xy_m"], [[0.75, 0.0]])

    def test_scan_summary_zero_transform_preserves_existing_behavior(self):
        original = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0)
        explicit = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0,
                                yaw_deg=0.0, x_m=0.0, y_m=0.0)
        self.assertEqual(original, explicit)

    def test_scan_summary_rejects_non_finite_transform(self):
        with self.assertRaises(ValueError):
            scan_summary([1.0], 0.0, 0.1, 0.1, 8.0, yaw_deg=float("nan"))

    def test_cli_transform_parameter_rejects_non_finite_value(self):
        for value in ("nan", "inf", "-inf", "not-a-number"):
            with self.subTest(value=value), self.assertRaises(
                    argparse.ArgumentTypeError):
                finite_argument(value)

    def test_depth_navigation_status_becomes_stale(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update_depth_navigation(SimpleNamespace(
            data='{"healthy": true, "state": "tracking", "processing_ms": 8.5, '
                 '"age_s": 0.4}'))
        fresh = store.snapshot()["depth_navigation"]
        self.assertTrue(fresh["live"])
        self.assertEqual(fresh["age_s"], 0.0)
        self.assertEqual(fresh["source_age_s"], 0.4)
        now[0] += 2.1
        status = store.snapshot()["depth_navigation"]
        self.assertFalse(status["live"])
        self.assertEqual(status["age_s"], 2.1)
        self.assertEqual(status["source_age_s"], 0.4)
        self.assertEqual(status["state"], "tracking")

    def test_decode_depth_honors_step_padding(self):
        raw = np.array([[1000, 2000, 9999], [0, 500, 9999]], dtype="<u2")
        msg = SimpleNamespace(encoding="16UC1", is_bigendian=False,
                              step=6, width=2, height=2, data=raw.tobytes())
        depth = decode_depth(msg)
        np.testing.assert_allclose(depth, [[1.0, 2.0], [0.0, 0.5]])

    def test_depth_summary_ignores_zero_and_uses_center_window(self):
        depth = np.ones((20, 20), dtype=np.float32) * 2.0
        depth[0, 0] = 0.0
        depth[0:3, 0:3] = 0.75
        result = depth_summary(depth)
        self.assertEqual(result["width"], 20)
        self.assertGreater(result["valid_pct"], 99)
        self.assertEqual(result["center_m"], 2.0)
        self.assertLess(result["nearest_m"], 2.0)


if __name__ == "__main__":
    unittest.main()
