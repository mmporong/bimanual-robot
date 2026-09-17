import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np


MODULE_PATH = Path(__file__).with_name("camera_calibration.py")
SPEC = importlib.util.spec_from_file_location("camera_calibration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


@unittest.skipUnless(
    hasattr(MODULE.board(7, 5, 30.0, 22.0), "generateImage"),
    "ChArUco board generation requires OpenCV >= 4.7; run with the lerobot environment",
)
class CameraCalibrationTest(unittest.TestCase):
    def test_generated_board_has_expected_size_and_detectable_corners(self):
        board = MODULE.board(7, 5, 30.0, 22.0)
        image = MODULE.render_board(board, 1400, 1000)
        self.assertEqual(image.shape, (1000, 1400))
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        corners, ids, _, marker_ids = MODULE.detect_charuco(bgr, board)
        self.assertIsNotNone(marker_ids)
        self.assertIsNotNone(ids)
        self.assertIsNotNone(corners)
        self.assertEqual(len(ids), 24)

    def test_board_survives_perspective_view(self):
        board = MODULE.board(7, 5, 30.0, 22.0)
        image = MODULE.render_board(board, 1400, 1000)
        source = np.float32([[0, 0], [1399, 0], [1399, 999], [0, 999]])
        target = np.float32([[170, 90], [1210, 150], [1300, 850], [100, 910]])
        transform = cv2.getPerspectiveTransform(source, target)
        warped = cv2.warpPerspective(image, transform, (1400, 1000), borderValue=255)
        corners, ids, _, marker_ids = MODULE.detect_charuco(
            cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR), board
        )
        self.assertIsNotNone(marker_ids)
        self.assertIsNotNone(ids)
        self.assertIsNotNone(corners)
        self.assertGreaterEqual(len(ids), 18)


if __name__ == "__main__":
    unittest.main()
