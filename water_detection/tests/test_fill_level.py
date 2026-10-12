"""충전율 계산(타원 중심 기준 컵 높이 대비 수면 높이)과 판정·연속 확정 로직 검증.

비스듬히 본 위가 넓은 컵을 합성한다: 림/바닥/수면은 납작함 K의 타원, 그 사이는 사다리꼴.
정답 충전율은 수면 타원 중심이 림~바닥 타원 중심 사이 어디 있는지로 정해진다.
"""
import unittest

import cv2
import numpy as np

from water_detection.fill_level import (FillConfirmer, FlatnessTracker, classify_fill, estimate_flatness,
                                        measure_fill)

K = 0.5
CX, RIM_Y, BOTTOM_Y = 300, 150, 330  # 타원 중심
RIM_A, BOTTOM_A = 60, 40  # 타원 가로반경 (위가 넓은 컵)
SHAPE = (480, 640)


def body(top_y: float, bottom_y: float, rim_a: float = RIM_A, bottom_a: float = BOTTOM_A) -> np.ndarray:
    """top_y~bottom_y 사이 컵 모양(두 타원 + 사다리꼴 옆면)."""
    def half_width(y):
        return rim_a + (bottom_a - rim_a) * (y - RIM_Y) / (BOTTOM_Y - RIM_Y)
    mask = np.zeros(SHAPE, np.uint8)
    for y, a in ((top_y, half_width(top_y)), (bottom_y, bottom_a)):
        cv2.ellipse(mask, (CX, int(y)), (int(a), int(a * K)), 0, 0, 360, 1, -1)
    pts = np.array([[CX - half_width(top_y), top_y], [CX + half_width(top_y), top_y],
                    [CX + bottom_a, bottom_y], [CX - bottom_a, bottom_y]], np.int32)
    cv2.fillPoly(mask, [pts], 1)
    return mask.astype(bool)


CUP_MASK = body(RIM_Y, BOTTOM_Y)
ys, xs = np.nonzero(CUP_MASK)
CUP_BOX = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))


def water(fill: float, rim_a: float = RIM_A, bottom_a: float = BOTTOM_A) -> np.ndarray:
    return body(BOTTOM_Y - fill * (BOTTOM_Y - RIM_Y), BOTTOM_Y, rim_a, bottom_a)


def frustum_volume_fraction(height_fraction: float, rim_a: float, bottom_a: float) -> float:
    r = bottom_a + (rim_a - bottom_a) * height_fraction
    return (r ** 3 - bottom_a ** 3) / (rim_a ** 3 - bottom_a ** 3)


class FlatnessTest(unittest.TestCase):
    def test_estimates_camera_flatness_from_rim(self):
        self.assertAlmostEqual(estimate_flatness(CUP_BOX, CUP_MASK), K, delta=0.05)

    def test_missing_or_implausible_rim_is_none(self):
        self.assertIsNone(estimate_flatness(CUP_BOX, None))
        flat_block = np.zeros(SHAPE, bool)
        flat_block[100:300, 250:260] = True  # 가는 막대 - 림 타원처럼 안 보임
        self.assertIsNone(estimate_flatness((250, 100, 260, 300), flat_block))

    def test_tracker_uses_median_and_ignores_none(self):
        t = FlatnessTracker(size=5)
        for v in (0.5, 0.52, 3.0, None, 0.48):
            k = t.update(v)
        self.assertAlmostEqual(k, 0.51)


class MeasureFillTest(unittest.TestCase):
    def test_matches_ellipse_center_fill_at_various_levels(self):
        for fill in (0.2, 0.5, 0.8):
            r = measure_fill(CUP_BOX, water(fill), CUP_MASK, K)
            self.assertAlmostEqual(r.ratio, fill, delta=0.03, msg=f"fill={fill}")

    def test_volume_ratio_matches_frustum_formula(self):
        """위가 넓은 컵: 높이 50%면 부피는 약 40% - 컵 모양 가정 없이 윤곽 적분으로 맞아야 함."""
        for fill in (0.2, 0.5, 0.8):
            r = measure_fill(CUP_BOX, water(fill), CUP_MASK, K)
            self.assertAlmostEqual(r.volume_ratio, frustum_volume_fraction(fill, RIM_A, BOTTOM_A), delta=0.03,
                                   msg=f"fill={fill}")

    def test_volume_equals_height_for_straight_cup(self):
        cup = body(RIM_Y, BOTTOM_Y, 50, 50)
        ys, xs = np.nonzero(cup)
        box = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))
        r = measure_fill(box, water(0.5, 50, 50), cup, K)
        self.assertAlmostEqual(r.volume_ratio, 0.5, delta=0.03)

    def test_target_level_line(self):
        r = measure_fill(CUP_BOX, water(0.5), CUP_MASK, K)
        self.assertAlmostEqual(r.level_y(0.5, by_volume=False), (RIM_Y + BOTTOM_Y) / 2, delta=3)
        # 부피 40% 지점 = 높이 약 50% 지점
        target = frustum_volume_fraction(0.5, RIM_A, BOTTOM_A)
        self.assertAlmostEqual(r.level_y(target, by_volume=True), (RIM_Y + BOTTOM_Y) / 2, delta=5)

    def test_box_edge_method_would_overestimate(self):
        """타원 끝끼리 비교하면 높게 나온다는 전제 확인 (중심 방식을 쓰는 이유)."""
        wm = water(0.2)
        top = np.nonzero(wm.any(axis=1))[0][0]
        edge_ratio = (CUP_BOX[3] - top) / (CUP_BOX[3] - CUP_BOX[1])
        self.assertGreater(edge_ratio, 0.2 + 0.08)
        self.assertAlmostEqual(measure_fill(CUP_BOX, wm, CUP_MASK, K).ratio, 0.2, delta=0.03)

    def test_partial_water_mask_gives_same_level(self):
        """모델이 수면 가운데 조각만 잡아도(액체 전체 조각 누락) 수면 높이는 같아야 함."""
        full = water(0.5)
        top = np.nonzero(full.any(axis=1))[0][0]
        partial = np.zeros(SHAPE, bool)
        partial[top:top + 15, CX - 15:CX + 15] = True
        self.assertAlmostEqual(measure_fill(CUP_BOX, partial, CUP_MASK, K).ratio,
                               measure_fill(CUP_BOX, full, CUP_MASK, K).ratio, delta=0.02)

    def test_ellipse_centers_follow_tilted_cup(self):
        """원근 때문에 아래쪽이 왼쪽으로 치우쳐 보이는 컵 - 원마다 그 높이의 컵 가운데에 그려야 함."""
        def tilt(mask):
            out = np.zeros_like(mask)
            for y in range(SHAPE[0]):
                shift = -int(round(20 * max(0, y - RIM_Y) / (BOTTOM_Y - RIM_Y)))
                out[y] = np.roll(mask[y], shift)
            return out
        cup, wm = tilt(CUP_MASK), tilt(water(0.5))
        ys, xs = np.nonzero(cup)
        box = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))
        r = measure_fill(box, wm, cup, K)
        self.assertAlmostEqual(r.rim_center_x, CX, delta=3)
        self.assertAlmostEqual(r.surface_center_x, CX - 10, delta=3)
        self.assertAlmostEqual(r.bottom_center_x, CX - 20, delta=3)

    def test_thin_stream_above_surface_is_ignored(self):
        wm = water(0.5)
        wm[0:240, CX - 2:CX + 2] = True  # 위에서 떨어지는 가는 물줄기
        self.assertAlmostEqual(measure_fill(CUP_BOX, wm, CUP_MASK, K).ratio, 0.5, delta=0.03)

    def test_water_outside_cup_is_ignored(self):
        wm = water(0.5)
        wm[:, 500:600] = True  # 다른 컵의 물
        self.assertAlmostEqual(measure_fill(CUP_BOX, wm, CUP_MASK, K).ratio, 0.5, delta=0.03)

    def test_no_water_is_unknown_not_empty(self):
        r = measure_fill(CUP_BOX, np.zeros(SHAPE, bool), CUP_MASK, K)
        self.assertIsNone(r.ratio)
        self.assertEqual(r.reason, "no_water")

    def test_residual_droplets_are_unknown(self):
        wm = np.zeros(SHAPE, bool)
        wm[320:322, 290:300] = True
        self.assertIsNone(measure_fill(CUP_BOX, wm, CUP_MASK, K).ratio)

    def test_unknown_flatness_is_rim_unclear(self):
        r = measure_fill(CUP_BOX, water(0.5), CUP_MASK, None)
        self.assertIsNone(r.ratio)
        self.assertEqual(r.reason, "rim_unclear")

    def test_cup_cut_off_at_frame_edge_is_unknown(self):
        for box in ((CUP_BOX[0], CUP_BOX[1], CUP_BOX[2], float(SHAPE[0])),  # 아래 끝
                    (CUP_BOX[0], CUP_BOX[1], float(SHAPE[1]), CUP_BOX[3])):  # 오른쪽 끝
            r = measure_fill(box, water(0.5), CUP_MASK, K)
            self.assertIsNone(r.ratio)
            self.assertEqual(r.reason, "cup_at_edge")


class ClassifyTest(unittest.TestCase):
    def test_states(self):
        self.assertEqual(classify_fill(0.5, 0.7, 0.05), "under")
        self.assertEqual(classify_fill(0.72, 0.7, 0.05), "target")
        self.assertEqual(classify_fill(0.8, 0.7, 0.05), "over")
        self.assertEqual(classify_fill(None, 0.7, 0.05), "unknown")


class ConfirmerTest(unittest.TestCase):
    def test_requires_consecutive_frames_and_resets_on_unknown(self):
        c = FillConfirmer(frames=3)
        self.assertIsNone(c.update("target"))
        self.assertIsNone(c.update("target"))
        self.assertIsNone(c.update("unknown"))
        self.assertIsNone(c.update("target"))
        self.assertIsNone(c.update("target"))
        self.assertEqual(c.update("target"), "target")


if __name__ == "__main__":
    unittest.main()
