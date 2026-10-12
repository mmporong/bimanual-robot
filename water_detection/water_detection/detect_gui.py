# detect_gui.py
# 로봇 팔 없이 탑뷰 카메라만으로 컵별 충전율(컵 높이 대비 수면 높이)을 실시간으로 보는 독립 웹 GUI.
#
# 카메라를 이 프로세스가 직접 연다 (기본: Orbbec Astra, --camera로 일반 웹캠·영상·사진 시퀀스).
# Astra는 한 프로세스만 열 수 있으므로 다른 프로그램이 쓰고 있으면 안 된다.
#
# 처리 흐름 (백그라운드 스레드가 브라우저 접속과 상관없이 계속 돈다):
#   카메라 프레임 → YOLOE "cup"으로 컵 박스 → water_yoloe.pt로 물 마스크
#   → 컵 림에서 타원 납작함 k 측정(최근 중간값) → fill_level.measure_fill로 컵별 충전율 → 목표 대비 under/target/over
#   → CONFIRM_FRAMES 연속 같은 판정이면 [확정] → 표시 이미지 + /status JSON 갱신
# 브라우저는 /stream(이미지)과 /status(숫자)를 받아 보여주고, 슬라이더로 /settings를 바꾼다.
# 충전율 계산 방식과 한계는 fill_level.py 참고. 컵은 화면 중앙 기준 image-left/right로
# 이름만 붙인다(물체 ID 추적 아님).
#
# 깊이 카메라로 실제 수위(mm)를 계산하는 것도 시도해봤는데, Astra의 적외선 구조광
# 방식은 액체 표면처럼 반사/굴절이 심한 곳에서는 유효 depth를 거의 못 읽어(실측:
# 컵 영역 픽셀의 1.2%만 유효값) RGB 이미지상 높이 비율을 쓴다.
#
# 실행:  python -m water_detection.detect_gui   (옵션: --target-fill 0.7 --tolerance 0.05 --port 8095)
# 테스트: --camera 0 (OpenCV 카메라) 또는 --camera '사진폴더/water_%04d.jpg' (사진 반복 재생)
# 브라우저: http://localhost:8095

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel

from perception.yoloe_detector import detect as yoloe_detect
from water_detection.cup_tracker import MIN_REPORT_FRAMES, CupTracker
from water_detection.fill_level import (EDGE_MARGIN_PX, FillConfirmer, FillReading, FlatnessTracker, classify_fill,
                                       estimate_flatness, measure_fill)

app = FastAPI()

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "water_yoloe.pt"
CONF_THRESHOLD = 0.15  # 화면에 물로 칠하는 기준
MEASURE_CONF = 0.05  # 컵 안 수면 측정에 합치는 기준 (조각 깜빡임 대응)
CUP_QUERY = "cup"
WATER_COLOR = (255, 140, 0)  # BGR - 파랑 계열, 컵 색과 겹치지 않게
LEFT_COLOR = (0, 220, 220)  # BGR - 노랑
RIGHT_COLOR = (220, 0, 220)  # BGR - 마젠타
TARGET_COLOR = (0, 200, 0)  # BGR - 초록, 목표 높이선
SURFACE_COLOR = (255, 255, 255)  # BGR - 흰색, 수면 타원
RIM_COLOR = (0, 255, 255)  # BGR - 노랑, 림 타원
BOTTOM_COLOR = (0, 0, 255)  # BGR - 빨강, 바닥 타원
OCCLUDED_COLOR = (0, 0, 200)  # BGR - 어두운 빨강, 저장된 컵 영역 중 지금 가려진 부분
STREAM_FPS = 10
CONFIRM_FRAMES = 5

_model = None
_target_fill = 0.7
_tolerance = 0.05
_by_volume = True  # 목표/판정 기준: True=부피 비율(컵 윤곽 적분), False=높이 비율
_state_lock = threading.Lock()
_latest_jpeg: bytes | None = None
_latest_raw_jpeg: bytes | None = None  # 표시를 그리기 전 원본 - 디버깅·학습 데이터 수집용
_latest_status: dict = {"cups": [], "updated": None}  # /status로 브라우저에 주는 최신 판정


def _ensure_model():
    global _model
    if _model is None:
        import torch
        from ultralytics import YOLOE

        if not MODEL_PATH.exists():
            raise RuntimeError(f"액체 세그멘테이션 모델이 없습니다: {MODEL_PATH} (README의 '모델' 참고)")
        _model = YOLOE(str(MODEL_PATH))
        _model.to("cuda" if torch.cuda.is_available() else "cpu")
    return _model


class AstraSource:
    name = "Astra 탑뷰"

    def __init__(self) -> None:
        from perception.astra_stream import AstraStream

        self._stream = AstraStream()
        self._stream.start()
        self._last_id = 0

    def read_bgr(self) -> np.ndarray:
        self._last_id, rgb = self._stream.read_new(self._last_id)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        self._stream.stop()


class OpenCVSource:
    """카메라 번호/장치 경로/영상/사진 시퀀스. 파일이면 끝나면 처음부터 다시 재생한다."""

    def __init__(self, spec: str) -> None:
        self.name = f"OpenCV {spec}"
        self._is_file = not spec.isdecimal() and not spec.startswith("/dev/")
        self._cap = cv2.VideoCapture(int(spec) if spec.isdecimal() else spec)
        if not self._cap.isOpened():
            raise RuntimeError(f"카메라/파일을 열 수 없습니다: {spec}")

    def read_bgr(self) -> np.ndarray:
        ok, frame = self._cap.read()
        if not ok and self._is_file:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("프레임을 읽지 못했습니다.")
        if self._is_file:
            time.sleep(0.3)  # 사진 시퀀스는 너무 빨리 넘어가지 않게
        return frame

    def close(self) -> None:
        self._cap.release()


def _draw_ellipse(display, cx: float, cy: float, half_width: float, flatness: float, color, thickness: int) -> None:
    axes = (max(1, int(half_width)), max(1, int(half_width * flatness)))
    cv2.ellipse(display, (int(cx), int(cy)), axes, 0, 0, 360, color, thickness)
    cv2.line(display, (int(cx - half_width), int(cy)), (int(cx + half_width), int(cy)), color, 1)


def annotate_frame(frame_bgr: np.ndarray, model, target: float, tolerance: float,
                   flatness_tracker: FlatnessTracker, cup_tracker: CupTracker,
                   by_volume: bool = True) -> tuple[np.ndarray, list[dict], float | None]:
    """한 프레임에서 컵·물을 찾아 컵별 충전율을 계산하고 표시용 이미지를 만든다.
    반환: (표시 이미지, 컵별 정보 리스트, 지금까지 안정화된 타원 납작함)."""
    display = frame_bgr.copy()
    height, width = frame_bgr.shape[:2]
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    # 1) 물 마스크 - 여러 인스턴스를 하나로 합친다. 컵 매칭은 컵 박스와의 교집합으로 하므로
    #    물줄기처럼 여러 컵에 걸친 마스크도 각 컵에 들어간 부분만 쓰인다.
    #    측정에는 신뢰도 낮은 조각(MEASURE_CONF 이상)까지 합친다 - 모델이 수면을 여러 조각으로 내고
    #    일부가 기준선에서 깜빡여서(실측). 컵 박스 밖은 어차피 안 쓰므로 오검출 영향이 작다.
    results = model.predict(frame_bgr, conf=MEASURE_CONF, verbose=False)[0]
    water = np.zeros((height, width), dtype=bool)
    shown_water = np.zeros((height, width), dtype=bool)
    if results.masks is not None:
        for mask_raw, conf in zip(results.masks.data.cpu().numpy(), results.boxes.conf.cpu().numpy()):
            mask = cv2.resize(mask_raw, (width, height), interpolation=cv2.INTER_NEAREST) > 0.5
            water |= mask
            if conf >= CONF_THRESHOLD:
                shown_water |= mask
    if shown_water.any():
        overlay = np.zeros_like(display)
        overlay[shown_water] = WATER_COLOR
        cv2.addWeighted(overlay, 0.4, display, 1.0, 0, dst=display)

    # 2) 타원 납작함 - 아직 모양을 저장하지 않은 컵의 림에서만 잰다(저장된 컵은 가려질 수 있음).
    #    컵이 아니라 카메라 각도 값이라 최근 중간값을 모든 컵에 쓴다.
    # 화면 끝에 걸친 컵은 높이 기준을 잡을 수 없어 추적하지 않고 표시만 한다
    detections, edge_cups = [], []
    for det in yoloe_detect(frame_rgb, CUP_QUERY):
        x0, y0, x1, y1 = det.box_xyxy
        at_edge = x0 <= EDGE_MARGIN_PX or y0 <= EDGE_MARGIN_PX or x1 >= width - EDGE_MARGIN_PX or y1 >= height - EDGE_MARGIN_PX
        (edge_cups if at_edge else detections).append(det)
    for det in detections:
        if not cup_tracker.overlaps_locked(det.mask):
            flatness_tracker.update(estimate_flatness(det.box_xyxy, det.mask))
    flatness = flatness_tracker.update(None)

    # 3) 컵 추적 - 새 컵은 LOCK_FRAMES 동안 그대로면 모양 저장, 저장된 컵은 가려짐/이동 판정
    tracks = cup_tracker.update(detections, flatness, cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY))

    # 4) 컵별 충전율 - 저장된 모양 + 이번 프레임 수면. 좌/우는 표시용 이름일 뿐이다
    cup_infos = []
    for track in tracks:
        box = track.box
        if box is None or (not track.locked and len(track.history) < min(MIN_REPORT_FRAMES, cup_tracker.lock_frames)):
            continue  # 잠깐 뜨는 오검출은 몇 프레임 이어질 때까지 표시 안 함
        x0, y0, x1, y1 = box
        side = "image-left" if (x0 + x1) / 2 < width / 2 else "image-right"
        color = LEFT_COLOR if side == "image-left" else RIGHT_COLOR
        if not track.locked:
            reading = FillReading(None, y0, y1, None, "locking", center_x=(x0 + x1) / 2)
        else:
            # 안 보이면(lost) 컵 전체가 가려진 것으로 보고 판정 불가
            occluded = track.locked_mask if track.status == "lost" else track.occluded
            reading = measure_fill(box, water, track.locked_mask, track.locked_flatness, occluded)
        value = reading.volume_ratio if by_volume else reading.ratio
        state = classify_fill(value, target, tolerance)
        cup_infos.append({"id": track.id, "side": side, "box": box, "reading": reading, "state": state,
                          "status": track.status, "lock_progress": len(track.history) / cup_tracker.lock_frames})

        ix0, iy0, ix1, iy1 = (int(v) for v in box)
        if track.occluded is not None and track.occluded.any():
            shown = track.occluded.copy()
            if reading.surface_y is not None:
                shown[int(reading.surface_y):] = False  # 수면 아래 변화는 액체가 차오른 것이라 표시 안 함
            tint = np.zeros_like(display)
            tint[shown] = OCCLUDED_COLOR
            cv2.addWeighted(tint, 0.6, display, 1.0, 0, dst=display)
        cv2.rectangle(display, (ix0, iy0), (ix1, iy1), color, 1)
        k = track.locked_flatness
        if track.locked and reading.rim_half_width > 0 and k is not None:
            cx = reading.center_x
            _draw_ellipse(display, reading.rim_center_x or cx, reading.rim_y, reading.rim_half_width, k, RIM_COLOR, 2)
            _draw_ellipse(display, reading.bottom_center_x or cx, reading.bottom_y, reading.bottom_half_width, k,
                          BOTTOM_COLOR, 2)
            target_y = reading.level_y(target, by_volume)
            if target_y is not None:
                cv2.line(display, (ix0 - 12, int(target_y)), (ix1 + 12, int(target_y)), TARGET_COLOR, 2)
        if reading.surface_y is not None:
            _draw_ellipse(display, reading.surface_center_x or reading.center_x, reading.surface_y,
                          reading.surface_half_width, k,
                          SURFACE_COLOR, 2)
            vol = "-" if reading.volume_ratio is None else f"{reading.volume_ratio * 100:.0f}%"
            label = f"#{track.id}: vol {vol} / h {reading.ratio * 100:.0f}% {state}"
        elif not track.locked:
            label = f"#{track.id}: locking {len(track.history)}/{cup_tracker.lock_frames}"
        else:
            label = f"#{track.id}: {reading.reason}"
        cv2.putText(display, label, (ix0, max(15, iy0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    for det in edge_cups:
        ix0, iy0, ix1, iy1 = (int(v) for v in det.box_xyxy)
        cv2.rectangle(display, (ix0, iy0), (ix1, iy1), (160, 160, 160), 1)
        cv2.putText(display, "cup_at_edge", (ix0, max(15, iy0 + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        cup_infos.append({"id": None, "side": "image-left" if (ix0 + ix1) / 2 < width / 2 else "image-right",
                          "box": det.box_xyxy, "state": "unknown", "status": "edge", "lock_progress": 0.0,
                          "reading": FillReading(None, iy0, iy1, None, "cup_at_edge", center_x=(ix0 + ix1) / 2)})

    k_text = f"k={flatness:.2f}" if flatness is not None else "k=?"
    header = f"target {'vol' if by_volume else 'h'} {target * 100:.0f}% +/-{tolerance * 100:.0f}%  {k_text}" if cup_infos else f"no cup  {k_text}"
    cv2.putText(display, header, (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return display, cup_infos, flatness


def _worker(source) -> None:
    """카메라 → 충전율 계산 → 최신 이미지/상태 갱신을 계속 반복한다."""
    global _latest_jpeg, _latest_raw_jpeg
    model = _ensure_model()
    confirmers: dict[int, FillConfirmer] = {}  # 컵 트랙 id별 연속 판정
    flatness_tracker = FlatnessTracker()
    cup_tracker = CupTracker()
    fps = None
    while True:
        t0 = time.monotonic()
        try:
            frame_bgr = source.read_bgr()
        except RuntimeError as e:
            with _state_lock:
                _latest_status.update(error=str(e))
            for confirmer in confirmers.values():
                confirmer.update("unknown")
            time.sleep(0.5)
            continue

        target, tolerance, by_volume = _target_fill, _tolerance, _by_volume
        display, cup_infos, flatness = annotate_frame(frame_bgr, model, target, tolerance, flatness_tracker,
                                                      cup_tracker, by_volume)
        cups_status = []
        for info in cup_infos:
            confirmed = None if info["id"] is None else \
                confirmers.setdefault(info["id"], FillConfirmer(CONFIRM_FRAMES)).update(info["state"])
            r = info["reading"]
            cups_status.append({"id": info["id"], "side": info["side"], "status": info["status"],
                                "lock_progress": round(info["lock_progress"], 2),
                                "ratio": r.ratio, "volume_ratio": r.volume_ratio, "reason": r.reason,
                                "state": info["state"], "confirmed": confirmed})
            if confirmed is not None:
                x0, _, _, y1 = (int(v) for v in info["box"])
                cv2.putText(display, f"[{confirmed}]", (x0, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, TARGET_COLOR, 2)
        live_ids = {info["id"] for info in cup_infos}
        for track_id in list(confirmers):
            if track_id not in live_ids:
                del confirmers[track_id]

        ok, jpeg = cv2.imencode(".jpg", display)
        raw_ok, raw_jpeg = cv2.imencode(".jpg", frame_bgr)
        dt = time.monotonic() - t0
        fps = 1.0 / dt if fps is None else 0.9 * fps + 0.1 / dt
        with _state_lock:
            if ok:
                _latest_jpeg = jpeg.tobytes()
            if raw_ok:
                _latest_raw_jpeg = raw_jpeg.tobytes()
            _latest_status.update(cups=sorted(cups_status, key=lambda c: c["id"] or 10**9), updated=time.time(),
                                  target_fill=target, tolerance=tolerance, by_volume=by_volume, error=None,
                                  fps=round(fps, 1), source=source.name,
                                  flatness=None if flatness is None else round(flatness, 3))


class Settings(BaseModel):
    target_fill: float
    tolerance: float
    by_volume: bool = True


@app.get("/settings")
def get_settings() -> dict:
    return {"target_fill": _target_fill, "tolerance": _tolerance, "by_volume": _by_volume}


@app.post("/settings")
def set_settings(req: Settings) -> dict:
    """브라우저 슬라이더로 목표 충전율/허용 오차를 실시간으로 바꾼다 (다음 프레임부터 적용)."""
    global _target_fill, _tolerance, _by_volume
    _by_volume = req.by_volume
    _target_fill = min(1.0, max(0.0, req.target_fill))
    _tolerance = min(0.5, max(0.0, req.tolerance))
    return get_settings()


@app.get("/status")
def status() -> dict:
    with _state_lock:
        return dict(_latest_status)


@app.get("/raw.jpg")
def raw_frame() -> Response:
    """표시를 그리기 전 최신 카메라 원본 한 장."""
    with _state_lock:
        jpeg = _latest_raw_jpeg
    if jpeg is None:
        return Response(status_code=503)
    return Response(content=jpeg, media_type="image/jpeg")


@app.get("/stream")
def stream() -> StreamingResponse:
    def generate():
        while True:
            with _state_lock:
                jpeg = _latest_jpeg
            if jpeg is not None:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            time.sleep(1.0 / STREAM_FPS)

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return INDEX_HTML.replace("__CONFIRM_FRAMES__", str(CONFIRM_FRAMES))


INDEX_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>실시간 충전율</title>
<style>
body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:16px;}
.wrap{display:flex;gap:20px;flex-wrap:wrap;align-items:flex-start;}
img{max-width:min(90vw,800px);border:1px solid #444;}
.panel{min-width:280px;max-width:400px;flex:1;}
.box{background:#1b1b1b;border:1px solid #333;border-radius:8px;padding:12px;margin-bottom:12px;}
h1{font-size:20px;margin:0 0 12px} h2{font-size:15px;margin:0 0 8px;color:#aaa}
input[type=range]{width:100%} .val{font-size:22px;font-weight:bold}
.cup{padding:10px;border-radius:6px;margin-bottom:8px;background:#222}
.fill{font-size:34px;font-weight:bold} .bar{height:10px;background:#333;border-radius:5px;position:relative;margin:6px 0}
.bar i{position:absolute;left:0;top:0;bottom:0;background:#4aa3ff;border-radius:5px}
.bar b{position:absolute;top:-4px;bottom:-4px;width:2px;background:#2ecc40}
.under{color:#f5c542} .target{color:#2ecc40} .over{color:#ff5c5c} .unknown{color:#888}
.note{font-size:12px;color:#888;line-height:1.5}
</style></head><body>
<h1>실시간 충전율 (컵 높이 대비 수면 높이)</h1>
<div id="info" class="note" style="margin:-6px 0 12px">연결 중…</div>
<div class="wrap">
  <img src="/stream">
  <div class="panel">
    <div class="box">
      <h2>판정 기준</h2>
      <label><input type="radio" name="m" value="v" checked> 부피 %</label>
      <label style="margin-left:12px"><input type="radio" name="m" value="h"> 높이 %</label>
      <h2 style="margin-top:10px">목표 충전율</h2>
      <div class="val"><span id="tv">70</span>%</div>
      <input id="t" type="range" min="0" max="100" step="1" value="70">
      <h2 style="margin-top:10px">허용 오차 (±)</h2>
      <div class="val"><span id="lv">5</span>%</div>
      <input id="l" type="range" min="0" max="20" step="1" value="5">
    </div>
    <div class="box"><h2>컵별 상태</h2><div id="cups" class="unknown">대기 중…</div></div>
    <div class="note">노랑 타원=컵 윗면, 흰 타원=수면, 빨강 타원=바닥, 초록선=목표 수면 위치.
    높이 %는 각 타원 가운데 선끼리 비교, 부피 %는 컵 윤곽 너비로 단면을 쌓아 계산합니다(컵마다 보정 없음,
    단면이 둥근 컵 기준). [확정]은 같은 판정이 __CONFIRM_FRAMES__프레임 연속일 때만 표시. mL가 아니라 비율입니다.
    no_water=물 미검출(비었다는 뜻 아님), cup_at_edge=컵이 화면 끝에 잘림,
    rim_unclear=컵 윗면 타원을 못 잡음. 새 컵은 약 1초 동안 그대로 두면 모양을 저장하고, 이후 그리퍼·병에
    가려진 부분(어두운 빨강)이 수면 위쪽이면 숫자 대신 "가려짐"을 냅니다. 컵을 옮기면 다시 저장합니다.</div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
const NAMES = {under:"부족", target:"목표 도달", over:"초과", unknown:"판정 불가"};
const REASONS = {locking:"컵 모양 저장 중", occluded:"가려짐 - 수면을 확인할 수 없음", no_water:"액체 미검출",
  cup_at_edge:"컵이 화면 끝에 잘림", rim_unclear:"컵 윗면을 못 잡음", cup_too_small:"컵이 너무 작음"};
let timer = null;
function send() {
  $("tv").textContent = $("t").value; $("lv").textContent = $("l").value;
  clearTimeout(timer);
  timer = setTimeout(() => fetch("/settings", {method:"POST", headers:{"Content-Type":"application/json"},
    body: JSON.stringify({target_fill: $("t").value / 100, tolerance: $("l").value / 100,
                          by_volume: document.querySelector('input[name=m]:checked').value === "v"})}), 150);
}
$("t").oninput = send; $("l").oninput = send;
document.querySelectorAll('input[name=m]').forEach(r => r.onchange = send);
fetch("/settings").then(r => r.json()).then(s => {
  $("t").value = Math.round(s.target_fill * 100); $("l").value = Math.round(s.tolerance * 100);
  document.querySelector(`input[name=m][value=${s.by_volume ? "v" : "h"}]`).checked = true;
  $("tv").textContent = $("t").value; $("lv").textContent = $("l").value;
});
async function poll() {
  try {
    const s = await (await fetch("/status")).json();
    $("info").textContent = s.error ? `카메라 오류: ${s.error}` : `입력: ${s.source} · 처리 ${s.fps ?? "-"} fps · 타원 납작함 k=${s.flatness ?? "측정 중"}`;
    if (!s.updated) { $("cups").textContent = "카메라 프레임 대기 중…"; }
    else if (!s.cups.length) { $("cups").innerHTML = '<span class="unknown">컵 감지 안 됨</span>'; }
    else {
      $("cups").innerHTML = s.cups.map(c => {
        const main = s.by_volume ? c.volume_ratio : c.ratio, other = s.by_volume ? c.ratio : c.volume_ratio;
        const pct = main == null ? null : Math.round(main * 100);
        const otherText = other == null ? "" : ` <span class="unknown">(${s.by_volume ? "높이" : "부피"} ${Math.round(other * 100)}%)</span>`;
        const conf = c.confirmed ? ` <b class="${c.confirmed}">[확정: ${NAMES[c.confirmed]}]</b>` : "";
        const why = c.reason === "locking" ? `${REASONS.locking} ${Math.round(c.lock_progress * 100)}%` : (REASONS[c.reason] ?? c.reason);
        const fill = pct == null ? `<span class="fill unknown">–</span> <span class="unknown">(${why})</span>`
          : `<span class="fill ${c.state}">${s.by_volume ? "부피" : "높이"} ${pct}%</span>${otherText}`;
        const remain = pct == null ? "" : `<div>목표까지 ${Math.round(s.target_fill * 100) - pct >= 0 ? "+" : ""}${Math.round(s.target_fill * 100) - pct}%p</div>`;
        const hidden = c.status === "occluded" && pct != null ? ` <span class="unknown">(일부 가려짐 - 수면은 보임)</span>` : "";
        return `<div class="cup"><div>${c.id == null ? "화면 끝 물체" : `컵 #${c.id}`} · ${c.side}${hidden}</div>${fill} <span class="${c.state}">${NAMES[c.state]}</span>${conf}
          <div class="bar"><i style="width:${pct ?? 0}%"></i><b style="left:${s.target_fill * 100}%"></b></div>${remain}</div>`;
      }).join("");
    }
  } catch (e) { $("cups").textContent = "서버 연결 끊김"; }
  setTimeout(poll, 300);
}
poll();
</script></body></html>"""


def main() -> None:
    global _target_fill, _tolerance
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8095)
    parser.add_argument("--camera", help="OpenCV 카메라 번호/장치/영상/사진 시퀀스. 생략하면 Astra 탑뷰를 직접 연다.")
    parser.add_argument("--target-fill", type=float, default=0.7, help="목표 충전율 시작값 (0~1, 브라우저에서 변경 가능)")
    parser.add_argument("--tolerance", type=float, default=0.05, help="목표 판정 허용 오차 시작값 (0~1)")
    args = parser.parse_args()
    _target_fill, _tolerance = args.target_fill, args.tolerance

    import uvicorn

    source = OpenCVSource(args.camera) if args.camera is not None else AstraSource()
    print(f"입력: {source.name} - 모델 로딩 후 http://localhost:{args.port} 에서 확인", flush=True)
    threading.Thread(target=_worker, args=(source,), daemon=True).start()
    try:
        uvicorn.run(app, host="127.0.0.1", port=args.port)
    finally:
        source.close()


if __name__ == "__main__":
    main()
