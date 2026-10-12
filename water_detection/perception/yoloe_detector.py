# yoloe_detector.py
#
# 로컬 YOLOE 기반 open-vocabulary 탐지기. 텍스트 프롬프트(예: "cup")로 물체를 찾고
# 박스와 세그멘테이션 마스크를 함께 준다 (네트워크 API 불필요, GPU/CPU 로컬 추론).
# 충전율 계산은 컵 마스크의 줄별 너비를 쓰므로 박스만 주는 탐지기로는 안 된다.
#
# 모델: models/yoloe-26s-seg.pt가 있으면 그것을, 없으면 Ultralytics가 같은 이름의 공식 가중치를
# 자동으로 내려받는다.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_LOCAL_MODEL = Path(__file__).resolve().parents[1] / "models" / "yoloe-26s-seg.pt"
MODEL_PATH = str(_LOCAL_MODEL) if _LOCAL_MODEL.exists() else "yoloe-26s-seg.pt"
CONF_THRESHOLD = 0.15
ROUNDNESS_RATIO = 1.3  # minAreaRect 가로세로비가 이보다 작으면(거의 원형) 회전 계산 자체를 건너뜀

_model = None
_device = None
_last_classes: list[str] | None = None


@dataclass
class Detection:
    box_xyxy: tuple[float, float, float, float]  # 픽셀 좌표
    score: float
    label: str
    mask: np.ndarray | None = None  # bool (H,W), 원본 프레임 크기로 리사이즈됨

    @property
    def center_xy(self) -> tuple[float, float]:
        """bbox(축 정렬 사각형) 중심 - 항상 계산 가능하지만, 물체가 좌우/상하로
        비대칭이거나 회전해 있으면 실제 물체 중심과 어긋난다. 정밀한 pick 좌표에는
        target_xy(마스크 있으면 centroid 우선) 사용."""
        x0, y0, x1, y1 = self.box_xyxy
        return ((x0 + x1) / 2, (y0 + y1) / 2)

    @property
    def mask_centroid_xy(self) -> tuple[float, float] | None:
        """마스크 픽셀들의 실제 무게중심 - ~/project/scripts/segment_target.py
        프로토타입이 3D 위치를 계산할 때 쓴 것과 같은 기준(마스크 전체 픽셀 평균).
        bbox 중심과 달리 회전/비대칭 물체에서도 실제 물체 중심을 따라간다.
        마스크가 없으면(클릭 지정 등) None."""
        if self.mask is None:
            return None
        ys, xs = np.nonzero(self.mask)
        if xs.size == 0:
            return None
        return (float(xs.mean()), float(ys.mean()))

    @property
    def target_xy(self) -> tuple[float, float]:
        """실제 pick/place 좌표 계산에 쓸 픽셀 - 마스크 있으면 centroid, 없으면
        bbox 중심으로 폴백. tasks/pick_place.py와 server/app.py는 이걸 쓴다
        (center_xy를 직접 쓰지 않음)."""
        return self.mask_centroid_xy or self.center_xy

    def _largest_contour(self) -> np.ndarray | None:
        if self.mask is None:
            return None
        contours, _ = cv2.findContours(self.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 20:
            return None
        return contour

    @property
    def angle_deg(self) -> float | None:
        """마스크의 최소외접사각형(minAreaRect) 회전각 - 이미지 좌표계 기준 degree.
        top-view 카메라가 작업대를 거의 수직으로 내려다본다는 전제 하에, 물체가
        이미지 평면에서 얼마나 돌아가 있는지를 나타낸다. 마스크가 없거나(클릭 지정
        pick) 너무 작으면 None.

        손잡이 없는 컵처럼 위에서 보면 거의 원형(회전 대칭)인 물체는 "각도"라는
        개념 자체가 의미가 없다 - minAreaRect가 매번 다른(불안정한) 각도를 내놓고,
        그걸 그대로 wrist_roll에 적용하면 불필요한 회전 때문에 오히려 도달 불가/
        오차만 커진다. 가로세로 비율이 ROUNDNESS_RATIO 미만이면(=거의 원형) 아예
        None을 반환해서 회전 보정 자체를 건너뛰게 한다.

        주의: 각도가 유의미한 경우에도, 이걸 실제 wrist_roll 조정치로 변환하는
        부호/배율/오프셋은 카메라 장착 회전과 로봇 base 좌표계 사이의 관계에
        달려있어서 검증되지 않았다 - tasks/pick_place.py에서 실물로 캘리브레이션 필요."""
        contour = self._largest_contour()
        if contour is None:
            return None
        (_cx, _cy), (_w, _h), angle = cv2.minAreaRect(contour)
        if _w > 0 and _h > 0:
            ratio = max(_w, _h) / min(_w, _h)
            if ratio < ROUNDNESS_RATIO:
                return None
        return float(angle)

    @property
    def contour_xy(self) -> list[tuple[float, float]] | None:
        """마스크의 외곽선 - 실제 컨투어를 화면에 그리기 위한 픽셀 점 목록 (bbox가
        아니라 물체 모양 그대로). approxPolyDP로 점 개수를 줄여서 응답을 가볍게 한다."""
        contour = self._largest_contour()
        if contour is None:
            return None
        epsilon = 0.003 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return [(float(p[0][0]), float(p[0][1])) for p in approx]


def _ensure_loaded() -> None:
    global _model, _device
    if _model is not None:
        return
    import torch
    from ultralytics import YOLOE

    _device = "cuda" if torch.cuda.is_available() else "cpu"
    _model = YOLOE(MODEL_PATH)
    _model.to(_device)


def release_gpu() -> None:
    """Any6D처럼 GPU를 많이 쓰는 작업을 돌리기 직전에 호출해서 YOLOE가 물고 있는
    GPU 메모리(수백MB)를 잠깐 비운다. 다음 detect() 호출의 predict(device=...)가
    알아서 다시 GPU로 올리므로 별도 reload는 필요 없다."""
    if _model is None or _device != "cuda":
        return
    import torch
    _model.to("cpu")
    torch.cuda.empty_cache()


def _to_class_id(term: str) -> str:
    # YOLOE.set_classes()는 클래스 이름에 공백을 허용하지 않음(assert " " not in classes)
    return term.strip().lower().replace(" ", "_")


def detect(frame_rgb: np.ndarray, text_query: str) -> list[Detection]:
    """frame_rgb: (H,W,3) uint8. text_query: 쉼표로 여러 후보 클래스 가능(예:
    "bottle,cup"). 신뢰도 높은 순으로 정렬된 Detection 리스트 반환 (없으면 빈 리스트)."""
    global _last_classes
    _ensure_loaded()
    _model.to(_device)  # release_gpu()로 내려가 있었다면 다시 올림 (이미 올라가 있으면 저비용 no-op)

    terms = [t.strip() for t in text_query.split(",") if t.strip()]
    if not terms:
        return []
    class_ids = [_to_class_id(t) for t in terms]
    if class_ids != _last_classes:
        _model.set_classes(class_ids)
        _last_classes = class_ids

    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    results = _model.predict(frame_bgr, conf=CONF_THRESHOLD, verbose=False, device=_device)[0]

    detections: list[Detection] = []
    if results.masks is not None and len(results.boxes) > 0:
        height, width = frame_bgr.shape[:2]
        for i in range(len(results.boxes)):
            score = float(results.boxes.conf[i])
            cls_id = int(results.boxes.cls[i])
            label = results.names.get(cls_id, terms[0] if len(terms) == 1 else "?")
            x0, y0, x1, y1 = (float(v) for v in results.boxes.xyxy[i])
            mask_raw = results.masks.data[i].cpu().numpy()
            mask = cv2.resize(mask_raw, (width, height), interpolation=cv2.INTER_NEAREST) > 0.5
            detections.append(Detection(box_xyxy=(x0, y0, x1, y1), score=score, label=label, mask=mask))

    detections.sort(key=lambda d: d.score, reverse=True)
    return detections


def detect_best(frame_rgb: np.ndarray, text_query: str) -> Detection | None:
    detections = detect(frame_rgb, text_query)
    return detections[0] if detections else None
