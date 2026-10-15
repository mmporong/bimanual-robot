# cup_tracker.py
# 컵 모양을 "가려지기 전에" 재서 고정하고, 이후 프레임에서 가려짐·이동을 판정한다.
#
# 붓는 동안에는 병 입구·물줄기·그리퍼가 받는 컵을 가려서 컵 윤곽(림/바닥 위치, 높이별 너비)이
# 틀어진다. 받는 컵은 붓는 동안 움직이지 않으므로, 컵이 처음 나타나 LOCK_FRAMES 동안 그대로
# 있으면 그 마스크들의 다수결로 컵 모양을, 회색조 영상의 중간값으로 컵 모습을 저장(lock)하고,
# 이후엔 저장된 모양으로 계산한다. 매 프레임 바뀌는 건 수면(물 마스크 맨 윗줄)뿐이다.
#
# 가려짐(occluded 픽셀)은 두 가지로 찾는다:
#   - 저장된 컵 영역인데 지금 컵 마스크에 없는 픽셀 (컵 윤곽 밖으로 튀어나온 그리퍼 등)
#   - 저장된 모습과 밝기가 크게 달라진 덩어리 - 컵 모델은 컵 안에 들어온 물체(병 입구)까지
#     컵으로 칠해서 마스크만으로는 못 찾는다(실측). 가는 물줄기는 열림 연산으로 지우고,
#     조명 전체 변화는 차이의 중간값을 빼서 보정한다.
# fill_level.measure_fill은 이 중 "검출된 수면 위쪽"(빈 컵 부분이라 저장 때와 같아야 함)이
# 가려졌을 때만 "occluded"(판정 불가)를 낸다 - 수면 아래는 액체가 차오르며 원래 바뀐다.
# 그래서 컵에서 액체를 덜어내는 경우(수면이 저장 때보다 내려감)는 가려짐으로 잘못 볼 수 있다.
#
# 그 밖에: 지금 마스크가 저장 영역 밖으로 많이 나가면 컵이 밀린 것 → 저장값 버리고 다시 저장.
# 매칭되는 컵이 안 보이면 판정 불가, LOST_FRAMES 넘으면 저장값 버림.
# 틀린 숫자를 내는 대신 "모름"을 내서, 로봇이 가려진 동안 더 붓지 않고 멈추게 하는 게 목적이다.

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

LOCK_FRAMES = 10  # 이만큼 연속으로 안정적이면 컵 모양 저장 (~1초)
STABLE_IOU = 0.85  # 저장 중 직전 프레임 마스크와 이 이상 겹쳐야 "안 움직임"
MATCH_IOU = 0.3  # 저장된 컵과 지금 검출을 같은 컵으로 볼 최소 겹침 (가려져도 일부는 겹침)
MOVED_OUTSIDE_FRACTION = 0.25  # 지금 마스크 중 저장 영역 밖 비율이 이보다 크면 컵이 밀린 것
LOST_FRAMES = 30  # 저장된 컵이 이만큼 연속으로 안 보이면 저장값 버림 (~3초)
EDGE_ERODE_PX = 4  # 마스크 가장자리 흔들림을 가려짐으로 오인하지 않게 저장 마스크를 깎아서 비교
DIFF_THRESHOLD = 40  # 저장된 모습과 회색조 밝기 차이가 이보다 크면 바뀐 픽셀 (0~255)
STREAM_OPEN_PX = 9  # 이보다 가는 변화(물줄기·잡음)는 가려짐으로 안 봄
CONTAINED_FRACTION = 0.8  # 검출이 다른 컵 안에 이 비율 이상 들어가면 중복으로 보고 무시
DUPLICATE_MAX_AREA_RATIO = 0.6  # 단, 그 컵 면적의 이 비율보다 작을 때만 (가려져 구멍 난 같은 컵은 남김)
MIN_REPORT_FRAMES = 3  # 저장 중인 컵은 이만큼 연속으로 보여야 표시 (잠깐 뜨는 오검출 숨김, 표시 측에서 사용)


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


def _box_of(mask: np.ndarray) -> tuple[float, float, float, float] | None:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)


@dataclass
class CupTrack:
    id: int
    history: list[np.ndarray] = field(default_factory=list)  # 저장 전 최근 마스크들
    gray_history: list[np.ndarray] = field(default_factory=list)  # 저장 전 최근 회색조 프레임들
    locked_mask: np.ndarray | None = None
    locked_box: tuple[float, float, float, float] | None = None
    locked_flatness: float | None = None
    eroded_mask: np.ndarray | None = None
    reference_gray: np.ndarray | None = None  # 저장 시점 컵 모습 (회색조, 프레임 크기)
    missed: int = 0
    # 이번 프레임 결과
    status: str = "locking"  # locking | ok | occluded(윤곽 일부 빠짐) | lost
    occluded: np.ndarray | None = None  # 저장 영역 중 지금 가려졌거나 크게 바뀐 픽셀
    current_box: tuple[float, float, float, float] | None = None

    @property
    def locked(self) -> bool:
        return self.locked_mask is not None

    @property
    def box(self):
        return self.locked_box if self.locked else self.current_box


class CupTracker:
    def __init__(self, lock_frames: int = LOCK_FRAMES) -> None:
        self.lock_frames = lock_frames
        self.tracks: list[CupTrack] = []
        self._next_id = 1

    def overlaps_locked(self, mask: np.ndarray | None) -> bool:
        """이미 모양을 저장한 컵과 겹치는 검출인가."""
        return mask is not None and any(
            t.locked and _iou(t.locked_mask, mask) >= MATCH_IOU for t in self.tracks)

    def update(self, detections: list, flatness: float | None,
               frame_gray: np.ndarray | None = None) -> list[CupTrack]:
        """detections: Detection(box_xyxy, mask) 리스트. flatness: 지금까지 안정화된 타원 납작함.
        frame_gray: 이번 프레임 회색조 (없으면 모습 비교 없이 마스크로만 가려짐 판정).
        반환: 이번 프레임에 보고할 트랙들 (저장 중·가려짐·안 보임 포함)."""
        masks = [d.mask for d in detections if d.mask is not None and d.mask.any()]
        # 다른 컵(이번 프레임의 더 큰 검출 또는 저장된 컵) 안에 대부분 들어가는 검출은 같은 컵의 일부를
        # 한 번 더 잡은 중복이다 - 액체 부분만 "컵"으로 잡히는 경우(실측)
        containers = sorted(masks, key=lambda m: m.sum(), reverse=True) + [
            t.locked_mask for t in self.tracks if t.locked]
        masks = [m for m in masks if not any(
            c is not m and m.sum() < DUPLICATE_MAX_AREA_RATIO * c.sum()
            and np.logical_and(m, c).sum() >= CONTAINED_FRACTION * m.sum()
            for c in containers)]

        # 1) 트랙 ↔ 검출 매칭 (겹침 큰 순서대로 1:1)
        pairs = []
        for ti, track in enumerate(self.tracks):
            ref = track.locked_mask if track.locked else track.history[-1]
            for di, m in enumerate(masks):
                iou = _iou(ref, m)
                if iou >= MATCH_IOU:
                    pairs.append((iou, ti, di))
        matched_t, matched_d = {}, set()
        for iou, ti, di in sorted(pairs, reverse=True):
            if ti not in matched_t and di not in matched_d:
                matched_t[ti] = di
                matched_d.add(di)

        # 2) 트랙 갱신
        kept: list[CupTrack] = []
        for ti, track in enumerate(self.tracks):
            current = masks[matched_t[ti]] if ti in matched_t else None
            if track.locked:
                if self._update_locked(track, current, frame_gray):
                    kept.append(track)
                elif current is not None:
                    matched_d.discard(matched_t[ti])  # 밀린 컵 - 같은 검출로 바로 새 트랙 저장 시작
            elif current is not None:
                self._update_locking(track, current, flatness, frame_gray)
                kept.append(track)

        # 3) 새로 나타난 컵 → 저장 시작
        for di, m in enumerate(masks):
            if di not in matched_d:
                track = CupTrack(id=self._next_id)
                self._next_id += 1
                self._update_locking(track, m, flatness, frame_gray)
                kept.append(track)

        self.tracks = kept
        return kept

    def _update_locking(self, track: CupTrack, current: np.ndarray, flatness: float | None,
                        frame_gray: np.ndarray | None) -> None:
        track.current_box = _box_of(current)
        if track.history and _iou(track.history[-1], current) < STABLE_IOU:
            track.history, track.gray_history = [], []  # 움직이는 중 - 처음부터 다시
        track.history = (track.history + [current])[-self.lock_frames:]
        if frame_gray is not None:
            track.gray_history = (track.gray_history + [frame_gray])[-self.lock_frames:]
        track.status, track.occluded = "locking", None
        if len(track.history) < self.lock_frames or flatness is None:
            return
        locked = np.mean(track.history, axis=0) >= 0.5  # 다수결 - 프레임별 마스크 흔들림 제거
        track.locked_mask, track.locked_box = locked, _box_of(locked)
        track.locked_flatness = flatness
        kernel = np.ones((2 * EDGE_ERODE_PX + 1,) * 2, np.uint8)
        track.eroded_mask = cv2.erode(locked.astype(np.uint8), kernel).astype(bool)
        if track.gray_history:
            track.reference_gray = np.median(track.gray_history, axis=0).astype(np.uint8)
        track.history, track.gray_history = [], []
        track.status = "ok"

    def _update_locked(self, track: CupTrack, current: np.ndarray | None,
                       frame_gray: np.ndarray | None) -> bool:
        if current is None:
            track.missed += 1
            track.status, track.occluded = "lost", None
            return track.missed < LOST_FRAMES
        track.missed = 0
        outside = np.logical_and(current, ~track.locked_mask).sum() / max(1, current.sum())
        if outside > MOVED_OUTSIDE_FRACTION:
            return False  # 컵이 밀림 - 트랙을 버리면 같은 검출로 새 트랙을 다시 저장
        missing = np.logical_and(track.eroded_mask, ~current)
        occluded = missing.copy()
        if frame_gray is not None and track.reference_gray is not None:
            occluded |= self._appearance_change(track, frame_gray)
        track.occluded = occluded
        # status는 컵 윤곽이 빠진 경우만 표시 - 모습 변화는 액체가 차올라도 생기므로, 그게 수면 위쪽인지는
        # measure_fill이 판단한다
        track.status = "occluded" if missing.any() else "ok"
        track.current_box = _box_of(current)
        return True

    @staticmethod
    def _appearance_change(track: CupTrack, frame_gray: np.ndarray) -> np.ndarray:
        """저장된 모습과 크게 달라진 덩어리 (컵 영역 안, 가는 물줄기·조명 전체 변화 제외)."""
        x0, y0, x1, y1 = (int(v) for v in track.locked_box)
        region = track.eroded_mask[y0:y1, x0:x1]
        diff = cv2.absdiff(frame_gray[y0:y1, x0:x1], track.reference_gray[y0:y1, x0:x1]).astype(np.int16)
        if region.any():
            diff -= int(np.median(diff[region]))  # 조명·노출이 전체적으로 바뀐 만큼 빼기
        changed = ((diff > DIFF_THRESHOLD) & region).astype(np.uint8)
        kernel = np.ones((STREAM_OPEN_PX, STREAM_OPEN_PX), np.uint8)
        changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, kernel).astype(bool)
        out = np.zeros_like(track.eroded_mask)
        out[y0:y1, x0:x1] = changed
        return out
