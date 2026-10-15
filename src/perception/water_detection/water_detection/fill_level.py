# fill_level.py
# 컵 마스크와 물 마스크로 컵별 충전율(높이 비율·부피 비율)을 계산한다.
#
# 탑뷰 카메라가 수직이 아니라 비스듬히(약 45도) 내려다보고 있어서, 투명 컵은 옆면
# 너머로 액체 기둥이 보이고 림·수면·바닥은 모두 납작한 타원으로 보인다. 타원의 끝
# (bbox 윗변, 물 마스크 맨 윗줄)끼리 비교하면 위가 넓은 컵에서는 타원 크기 차이만큼
# 충전율이 높게 나온다(저장 사진 실측 7~15%p, 빈 컵도 ~15%). 그래서 세 타원의
# "가로지르는 중심선"끼리 비교한다 - 타원 중심이 컵 중심축 위의 실제 높이에 해당한다.
#
#   납작함 k    = 타원 세로반경 / 가로반경. 컵이 아니라 카메라 내려다보는 각도로 정해지므로
#                 화면의 컵 림에서 매 프레임 재고(estimate_flatness), 호출 측이 여러 프레임
#                 중간값으로 안정화해서 넘긴다 - 컵을 바꿔도 따로 보정할 필요가 없다.
#   rim_y       = 컵 bbox 윗변 + k * 림 가로반경          (림 타원 중심)
#   surface_y   = 물 마스크 맨 윗줄 + k * 수면 가로반경    (수면 타원 중심)
#   bottom_y    = 컵 bbox 아랫변 - k * 바닥 가로반경       (바닥 타원 중심)
#   ratio       = (bottom_y - surface_y) / (bottom_y - rim_y)
#
# 부피 충전율(volume_ratio)은 컵 모양을 가정하지 않고 컵 윤곽으로 직접 계산한다. 컵을 얇은
# 원판(단면)을 쌓은 것으로 보면, 비스듬히 본 각 단면 타원은 자기 중심 줄에서 가장 넓으므로
# "컵 마스크의 각 가로줄 너비 = 그 높이 단면의 지름"이다. 단면 넓이 ∝ 지름²이라
#   volume_ratio = Σ(바닥~수면 줄 너비²) / Σ(바닥~림 줄 너비²)
# 원뿔대·와인잔·볼록한 컵 등 단면이 둥근 컵이면 컵마다 보정 없이 쓸 수 있다. 단면이
# 둥글지 않은 컵(사각 컵)이나 윤곽이 가려진 경우엔 틀린다. 결과는 비율이고 mL는 컵 용량을
# 알아야 한다. 불투명 컵은 옆면 너머 액체가 안 보여서 성립하지 않는다.

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

MIN_WATER_BOX_FRACTION = 0.01  # 컵 박스 면적 대비 물 픽셀이 이보다 적으면 "미검출" (물기·노이즈)
EDGE_MARGIN_PX = 2  # 컵 박스가 화면 끝에서 이 안쪽이면 잘린 것으로 봄
SURFACE_ROW_FRACTION = 0.3  # 가장 넓은 물 행 너비의 이 비율 이상인 첫 행을 수면으로 봄 (위에서 떨어지는 가는 물줄기 제외)
RIM_SEARCH_FRACTION = 0.4  # 컵 박스 위쪽 이 비율 안에서 가장 넓은 줄을 림 타원 중심으로 봄
BOTTOM_ROWS = (0.85, 0.95)  # 컵 박스 높이의 이 구간 줄 너비 중간값을 바닥 지름으로 봄
RIM_TOP_MAX_WIDTH_FRACTION = 0.6  # 림 타원이면 맨 윗줄은 가운데 줄 너비의 이 비율보다 좁아야 함
FLATNESS_RANGE = (0.2, 1.0)  # 이 밖의 k는 림을 잘못 잡은 것으로 보고 버림 (45도 전후 카메라 기준)
OCCLUSION_MIN_PX = 80  # 수면 위쪽 컵 영역에서 이보다 많은 픽셀이 가려지면 판정 불가
OCCLUSION_MARGIN_PX = -3  # 수면 맨 윗줄보다 이만큼 위까지만 검사 (그 아래는 액체가 차오르며 원래 바뀜)


@dataclass(frozen=True)
class FillReading:
    ratio: float | None  # 0=바닥, 1=림. 판정 불가면 None
    rim_y: float  # 림 타원 중심 y
    bottom_y: float  # 바닥 타원 중심 y
    surface_y: float | None  # 수면 타원 중심 y
    reason: str | None = None  # ratio가 None인 이유
    rim_half_width: float = 0.0  # 표시용 - 각 타원 가로반경(px)
    surface_half_width: float = 0.0
    bottom_half_width: float = 0.0
    center_x: float = 0.0  # 컵 박스 가운데 x (원별 가운데를 못 잡았을 때 대신 씀)
    # 원별 가운데 x - 원근 때문에 컵이 비스듬히 보여서(아래쪽이 화면 가운데 쪽으로 기울어짐) 높이마다
    # 컵 윤곽 가운데가 다르다. 박스 가운데 하나로 그리면 수위선·바닥 원이 옆으로 치우친다(실측)
    rim_center_x: float | None = None
    surface_center_x: float | None = None
    bottom_center_x: float | None = None
    volume_ratio: float | None = None  # 0=빈 컵, 1=림까지. 컵 윤곽 단면 적분
    # 부피 누적 곡선 - (바닥→림 순서의 y, 그 y까지의 부피 비율). 목표 부피의 높이선을 그릴 때 씀
    volume_profile: tuple[np.ndarray, np.ndarray] | None = field(default=None, compare=False, repr=False)

    def level_y(self, target: float, by_volume: bool) -> float | None:
        """목표 충전율에 해당하는 이미지 y (높이 기준이면 선형, 부피 기준이면 누적 곡선)."""
        if self.rim_half_width <= 0:
            return None
        if not by_volume:
            return self.bottom_y - target * (self.bottom_y - self.rim_y)
        if self.volume_profile is None:
            return None
        ys, cumulative = self.volume_profile
        return float(ys[min(int(np.searchsorted(cumulative, target)), len(ys) - 1)])


def _smoothed_row_widths(mask: np.ndarray) -> np.ndarray:
    widths = mask.sum(axis=1).astype(float)
    if widths.size >= 5:
        widths = np.convolve(widths, np.ones(5) / 5, mode="same")
    return widths


def _row_centers(crop: np.ndarray, x_offset: int) -> np.ndarray:
    """줄마다 마스크 왼쪽 끝~오른쪽 끝의 가운데 x (프레임 좌표). 빈 줄은 nan."""
    centers = np.full(crop.shape[0], np.nan)
    for r in np.nonzero(crop.any(axis=1))[0]:
        xs = np.nonzero(crop[r])[0]
        centers[r] = x_offset + (xs[0] + xs[-1] + 1) / 2
    return centers


def _center_near(centers: np.ndarray, r0: int, r1: int) -> float | None:
    """r0~r1 줄 가운데 x들의 중간값 (빈 줄 제외)."""
    vals = centers[max(0, r0): max(r0 + 1, r1)]
    vals = vals[~np.isnan(vals)]
    return float(np.median(vals)) if vals.size else None


def _crop(mask: np.ndarray, cup_box) -> tuple[np.ndarray, int]:
    x0, y0, x1, y1 = cup_box
    height, width = mask.shape[:2]
    ix0, iy0 = max(0, int(x0)), max(0, int(y0))
    ix1, iy1 = min(width, int(np.ceil(x1))), min(height, int(np.ceil(y1)))
    return mask[iy0:iy1, ix0:ix1], iy0


def estimate_flatness(cup_box, cup_mask: np.ndarray | None) -> float | None:
    """컵 림 타원의 세로/가로 비. 림 타원 중심 줄에서 컵 윤곽이 가장 넓다(위가 넓은 컵)는
    성질로 중심 줄을 찾는다. 잘못 잡은 것 같으면 None."""
    if cup_mask is None:
        return None
    crop, _ = _crop(cup_mask, cup_box)
    if crop.shape[0] < 10:
        return None
    top_part = crop[: max(3, int(RIM_SEARCH_FRACTION * crop.shape[0]))]
    widths = _smoothed_row_widths(top_part)
    center_row = int(np.argmax(widths))
    if widths[center_row] < 4:
        return None
    if top_part[:2].sum(axis=1).max() > RIM_TOP_MAX_WIDTH_FRACTION * widths[center_row]:
        return None  # 맨 윗줄이 가운데만큼 넓으면 림 타원이 아님 (잘린 마스크·막대 모양 오검출)
    k = center_row / (widths[center_row] / 2)
    return float(k) if FLATNESS_RANGE[0] <= k <= FLATNESS_RANGE[1] else None


def _hidden_above(occluded: np.ndarray | None, cup_box, y_limit: float) -> bool:
    """컵 박스 안, y_limit 위쪽에 가려진 픽셀이 충분히 많은가."""
    if occluded is None:
        return False
    crop, iy0 = _crop(occluded, cup_box)
    return int(crop[: max(0, int(y_limit) - iy0)].sum()) > OCCLUSION_MIN_PX


def measure_fill(cup_box, water_mask: np.ndarray | None, cup_mask: np.ndarray | None,
                 flatness: float | None, occluded: np.ndarray | None = None) -> FillReading:
    """cup_box: (x0, y0, x1, y1) 픽셀. water_mask: 프레임 크기 bool 마스크(모든 물 마스크 합집합).
    cup_mask: 이 컵의 bool 마스크. flatness: 안정화된 타원 납작함 k (없으면 판정 불가).
    occluded: 저장된 컵 영역 중 지금 가려진 픽셀(cup_tracker). 진짜 수면이 숨어 있을 수 있는
    "검출된 수면 맨 윗줄 위쪽"이 가려졌으면 틀린 숫자 대신 "occluded"를 낸다."""
    x0, y0, x1, y1 = cup_box
    center_x = (x0 + x1) / 2
    empty = FillReading(None, float(y0), float(y1), None, center_x=center_x)
    if y1 - y0 < 10 or x1 - x0 < 4:
        return _with_reason(empty, "cup_too_small")
    if water_mask is None:
        return _with_reason(empty, "no_water")
    frame_h, frame_w = water_mask.shape[:2]
    if (y0 <= EDGE_MARGIN_PX or y1 >= frame_h - EDGE_MARGIN_PX
            or x0 <= EDGE_MARGIN_PX or x1 >= frame_w - EDGE_MARGIN_PX):
        return _with_reason(empty, "cup_at_edge")  # 화면 밖에 잘리면 높이 기준·윤곽 너비가 틀어짐
    if cup_mask is None or flatness is None:
        return _with_reason(empty, "rim_unclear")

    # 림/바닥 가로반경 - 컵 마스크 줄 너비에서
    cup_crop, cup_iy0 = _crop(cup_mask, cup_box)
    cup_widths = _smoothed_row_widths(cup_crop)
    h = cup_crop.shape[0]
    rim_half = float(cup_widths[: max(3, int(RIM_SEARCH_FRACTION * h))].max()) / 2
    bottom_half = float(np.median(cup_widths[int(BOTTOM_ROWS[0] * h): max(int(BOTTOM_ROWS[1] * h), int(BOTTOM_ROWS[0] * h) + 1)])) / 2
    rim_y = y0 + flatness * rim_half
    bottom_y = y1 - flatness * bottom_half
    if bottom_y - rim_y < 5:
        return FillReading(None, float(rim_y), float(bottom_y), None, "rim_unclear", center_x=center_x)

    # 부피 누적 곡선 - 림 중심~바닥 중심 사이 줄마다 단면 넓이(∝ 너비²)를 바닥부터 누적
    r_top = int(round(rim_y)) - cup_iy0
    r_bottom = min(int(round(bottom_y)) - cup_iy0, len(cup_widths) - 1)
    areas = cup_widths[r_top: r_bottom + 1][::-1] ** 2  # 바닥 → 림 순서
    profile = None
    if areas.size >= 2 and areas.sum() > 0:
        ys = np.arange(r_bottom, r_top - 1, -1, dtype=float) + cup_iy0
        profile = (ys, np.cumsum(areas) / areas.sum())

    centers = _row_centers(cup_crop, max(0, int(x0)))
    rim_row = int(round(rim_y)) - cup_iy0
    geom = dict(rim_y=float(rim_y), bottom_y=float(bottom_y), rim_half_width=rim_half,
                bottom_half_width=bottom_half, center_x=center_x, volume_profile=profile,
                rim_center_x=_center_near(centers, rim_row - 2, rim_row + 3),
                bottom_center_x=_center_near(centers, int(BOTTOM_ROWS[0] * h), int(BOTTOM_ROWS[1] * h)))

    # 수면 - 물 마스크 맨 윗줄(수면 타원 뒤 끝)만 물 마스크에서 쓴다. 수면 타원은 그 높이의
    # 컵 단면이라 가로반경 = 컵 윤곽 반폭이므로, 중심 줄 r은 r - k*컵반폭(r) = 맨 윗줄을 푼다.
    # 물 마스크 너비는 쓰지 않는다 - 모델이 액체를 "수면 일부"/"액체 전체" 두 조각으로 내고
    # 전체 조각이 신뢰도 기준선에서 깜빡여서 너비가 프레임마다 반토막 났다(실측).
    crop, iy0 = _crop(water_mask, cup_box)
    if crop.size == 0 or crop.sum() < MIN_WATER_BOX_FRACTION * crop.size:
        reason = "occluded" if _hidden_above(occluded, cup_box, bottom_y) else "no_water"
        return FillReading(None, surface_y=None, reason=reason, **geom)
    row_counts = crop.sum(axis=1)
    rows = np.nonzero(row_counts >= SURFACE_ROW_FRACTION * row_counts.max())[0]
    surface_top = iy0 + int(rows[0])
    if _hidden_above(occluded, cup_box, surface_top + OCCLUSION_MARGIN_PX):
        return FillReading(None, surface_y=None, reason="occluded", **geom)
    # 수위선 = 액체가 컵 벽에 닿는 원. 위에서 보면 뒤쪽 끝은 입구로 보이는 액체 맨 윗줄이고, 양옆은
    # 컵 벽(그 높이의 컵 너비), 앞쪽은 컵 벽 너머로 보이는 액체 쪽이다. 입구로 보이는 수면 덩어리만
    # 쓰면 앞쪽이 입구 테두리에 가려져 실제 수위선보다 작고 납작하게 잡힌다(실측) - 그래서 너비는 컵
    # 윤곽에서, 납작함은 림과 같은 값으로 두고 중심 줄 r을 r - k*컵반폭(r) = 맨 윗줄에서 푼다.
    surface_row = r_bottom
    for r in range(max(0, surface_top - cup_iy0), r_bottom + 1):
        if cup_iy0 + r - flatness * cup_widths[r] / 2 >= surface_top:
            surface_row = r
            break
    surface_half = float(cup_widths[surface_row]) / 2
    surface_y = min(float(cup_iy0 + surface_row), bottom_y)

    ratio = (bottom_y - surface_y) / (bottom_y - rim_y)
    volume_ratio = None
    if profile is not None:
        ys, cumulative = profile
        below = ys >= surface_y  # 수면보다 아래(이미지 y가 큰) 줄들
        volume_ratio = float(cumulative[below][-1]) if below.any() else 0.0
        if surface_y <= ys[-1]:
            volume_ratio = 1.0
    return FillReading(float(np.clip(ratio, 0.0, 1.0)), surface_y=float(surface_y),
                       surface_half_width=surface_half, volume_ratio=volume_ratio,
                       surface_center_x=_center_near(centers, surface_row - 2, surface_row + 3), **geom)


def _with_reason(reading: FillReading, reason: str) -> FillReading:
    return FillReading(None, reading.rim_y, reading.bottom_y, None, reason, center_x=reading.center_x)


class FlatnessTracker:
    """프레임마다 잰 타원 납작함의 최근 중간값 - 한 프레임에서 림을 잘못 잡아도 튀지 않게.
    카메라 각도에서 정해지는 값이라 컵이 바뀌어도 이어서 쓴다."""

    def __init__(self, size: int = 30) -> None:
        self.size = size
        self._values: list[float] = []

    def update(self, value: float | None) -> float | None:
        if value is not None:
            self._values = (self._values + [value])[-self.size:]
        return float(np.median(self._values)) if self._values else None


def classify_fill(ratio: float | None, target: float, tolerance: float) -> str:
    """"under" | "target" | "over" | "unknown"."""
    if ratio is None:
        return "unknown"
    if ratio < target - tolerance:
        return "under"
    if ratio > target + tolerance:
        return "over"
    return "target"


class FillConfirmer:
    """같은 판정이 연속 N프레임 나와야 확정한다 - 한 프레임 오검출로 붓기를 멈추거나
    더 붓지 않게. "unknown"이 끼면 카운트를 리셋한다(가림·컵 분실을 도달로 보지 않음)."""

    def __init__(self, frames: int = 5) -> None:
        self.frames = frames
        self._state: str | None = None
        self._count = 0

    def update(self, state: str) -> str | None:
        if state == "unknown":
            self._state, self._count = None, 0
            return None
        if state == self._state:
            self._count += 1
        else:
            self._state, self._count = state, 1
        return state if self._count >= self.frames else None
