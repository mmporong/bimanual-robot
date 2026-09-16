import math
import pathlib
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from ammr_dashboard import decode_depth, depth_summary, scan_summary  # noqa: E402


class DashboardTelemetryTest(unittest.TestCase):
    def test_scan_summary_filters_invalid_and_reports_sectors(self):
        ranges = [1.0, float("inf"), 0.5, float("nan"), 2.0]
        result = scan_summary(ranges, -math.pi, math.pi / 2, 0.1, 8.0)
        self.assertEqual(result["valid_points"], 3)
        self.assertEqual(result["min_m"], 0.5)
        self.assertEqual(result["front_m"], 0.5)
        self.assertEqual(len(result["points_xy_m"]), 3)

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
