"""컵 모양 저장(lock)·가려짐·이동 판정과, 가려짐이 충전율 판정에 반영되는지 검증."""
import unittest
from dataclasses import dataclass

import numpy as np

from tests.test_fill_level import BOTTOM_Y, CUP_BOX, CUP_MASK, CX, K, RIM_Y, water
from water_detection.cup_tracker import LOST_FRAMES, CupTracker
from water_detection.fill_level import measure_fill


@dataclass
class Det:
    mask: np.ndarray
    box_xyxy: tuple = CUP_BOX


def locked_tracker() -> CupTracker:
    tracker = CupTracker(lock_frames=3)
    for _ in range(3):
        tracks = tracker.update([Det(CUP_MASK)], K)
    assert tracks[0].locked
    return tracker


def surface_top(fill: float) -> int:
    return int(np.nonzero(water(fill).any(axis=1))[0][0])


class LockTest(unittest.TestCase):
    def test_locks_after_stable_frames(self):
        tracker = CupTracker(lock_frames=3)
        self.assertEqual(tracker.update([Det(CUP_MASK)], K)[0].status, "locking")
        tracker.update([Det(CUP_MASK)], K)
        track = tracker.update([Det(CUP_MASK)], K)[0]
        self.assertTrue(track.locked)
        self.assertEqual(track.status, "ok")
        self.assertEqual(track.locked_flatness, K)

    def test_does_not_lock_without_flatness(self):
        tracker = CupTracker(lock_frames=2)
        for _ in range(3):
            track = tracker.update([Det(CUP_MASK)], None)[0]
        self.assertFalse(track.locked)


class DuplicateTest(unittest.TestCase):
    def test_detection_inside_another_cup_is_ignored(self):
        inner = water(0.5)  # 액체 부분만 "컵"으로 한 번 더 잡힘
        tracker = CupTracker(lock_frames=3)
        for _ in range(3):
            tracks = tracker.update([Det(CUP_MASK), Det(inner)], K)
        self.assertEqual(len(tracks), 1)
        self.assertTrue(tracks[0].locked)
        self.assertEqual(len(tracker.update([Det(inner)], K)), 1)  # 저장된 컵 안쪽만 잡혀도 새 컵 아님


class OcclusionTest(unittest.TestCase):
    def test_occlusion_above_surface_gives_occluded_not_wrong_number(self):
        tracker = locked_tracker()
        hidden = CUP_MASK.copy()
        top = surface_top(0.5)
        hidden[top - 30:top + 10, CX - 40:CX + 40] = False  # 병 입구가 수면 뒤쪽 끝을 가림
        track = tracker.update([Det(hidden)], K)[0]
        self.assertEqual(track.status, "occluded")
        wm = water(0.5)
        wm[top - 30:top + 10, CX - 40:CX + 40] = False  # 물도 그만큼 안 보임 → 수면이 낮게 보임
        r = measure_fill(track.box, wm, track.locked_mask, track.locked_flatness, track.occluded)
        self.assertIsNone(r.ratio)
        self.assertEqual(r.reason, "occluded")

    def test_occlusion_below_surface_still_measures_with_locked_shape(self):
        tracker = locked_tracker()
        hidden = CUP_MASK.copy()
        hidden[BOTTOM_Y - 40:BOTTOM_Y, CX - 60:CX] = False  # 그리퍼가 컵 아래 옆면을 가림
        track = tracker.update([Det(hidden)], K)[0]
        r = measure_fill(track.box, water(0.5), track.locked_mask, track.locked_flatness, track.occluded)
        self.assertAlmostEqual(r.ratio, 0.5, delta=0.03)
        full = measure_fill(CUP_BOX, water(0.5), CUP_MASK, K)
        self.assertAlmostEqual(r.volume_ratio, full.volume_ratio, delta=0.01)  # 저장된 모양이라 부피 그대로

    def test_small_edge_jitter_is_not_occlusion(self):
        tracker = locked_tracker()
        jitter = CUP_MASK.copy()
        jitter[:, CX + 58:] = False  # 가장자리 2px 정도 흔들림
        track = tracker.update([Det(jitter)], K)[0]
        r = measure_fill(track.box, water(0.8), track.locked_mask, track.locked_flatness, track.occluded)
        self.assertIsNotNone(r.ratio)

    def test_lost_cup_is_unknown_then_dropped(self):
        tracker = locked_tracker()
        track = tracker.update([], K)[0]
        self.assertEqual(track.status, "lost")
        r = measure_fill(track.box, water(0.5), track.locked_mask, track.locked_flatness, track.locked_mask)
        self.assertEqual(r.reason, "occluded")
        for _ in range(LOST_FRAMES):
            tracks = tracker.update([], K)
        self.assertEqual(tracks, [])


class AppearanceTest(unittest.TestCase):
    """컵 모델이 컵 안에 들어온 물체까지 컵으로 칠해 마스크가 그대로인 경우 - 모습 비교로 찾아야 함."""

    def setUp(self):
        self.gray = np.full(CUP_MASK.shape, 200, np.uint8)
        self.gray[water(0.3)] = 80  # 저장 시점: 30%까지 찬 컵
        self.tracker = CupTracker(lock_frames=3)
        for _ in range(3):
            self.tracker.update([Det(CUP_MASK)], K, self.gray)

    def test_object_inside_cup_above_surface_is_occluded(self):
        frame = self.gray.copy()
        top = surface_top(0.5)
        frame[top - 40:top - 5, CX - 30:CX + 30] = 60  # 병 입구 - 마스크는 그대로
        frame[water(0.5)] = 80
        track = self.tracker.update([Det(CUP_MASK)], K, frame)[0]
        r = measure_fill(track.box, water(0.5), track.locked_mask, track.locked_flatness, track.occluded)
        self.assertEqual(r.reason, "occluded")

    def test_rising_liquid_is_not_occlusion(self):
        frame = self.gray.copy()
        frame[water(0.6)] = 80  # 30% → 60%로 차오름 - 수면 아래만 바뀜
        track = self.tracker.update([Det(CUP_MASK)], K, frame)[0]
        r = measure_fill(track.box, water(0.6), track.locked_mask, track.locked_flatness, track.occluded)
        self.assertAlmostEqual(r.ratio, 0.6, delta=0.03)

    def test_thin_stream_and_global_lighting_change_are_not_occlusion(self):
        frame = (self.gray.astype(int) + 30).clip(0, 255).astype(np.uint8)  # 전체가 밝아짐
        frame[0:surface_top(0.3), CX - 2:CX + 2] = 80  # 가는 물줄기
        track = self.tracker.update([Det(CUP_MASK)], K, frame)[0]
        r = measure_fill(track.box, water(0.3), track.locked_mask, track.locked_flatness, track.occluded)
        self.assertAlmostEqual(r.ratio, 0.3, delta=0.03)


class MoveTest(unittest.TestCase):
    def test_moved_cup_is_relocked_as_new_track(self):
        tracker = locked_tracker()
        old_id = tracker.tracks[0].id
        moved = np.roll(CUP_MASK, 40, axis=1)  # 컵이 오른쪽으로 40px 밀림
        tracks = tracker.update([Det(moved)], K)
        self.assertEqual(len(tracks), 1)
        self.assertNotEqual(tracks[0].id, old_id)
        self.assertEqual(tracks[0].status, "locking")


if __name__ == "__main__":
    unittest.main()
