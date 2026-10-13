"""Orbbec Astra S 컬러 스트림을 OBS 가상 카메라로 중계한다.

Astra S는 일반 UVC 웹캠이 아니라 OpenNI2(Astra SDK) 전용 인터페이스로만
컬러 영상을 꺼낼 수 있다. 이 스크립트는 OpenNI2로 컬러 프레임을 읽어
pyvirtualcam(OBS Virtual Camera 백엔드)으로 계속 송출해서, lerobot의
`--robot.cameras` 같은 일반 OpenCV 카메라 설정이 평소처럼 index로
이 영상을 잡을 수 있게 만든다.

사전 조건:
    - OBS Studio가 설치돼 있어야 한다(가상 카메라 드라이버 등록용, OBS 앱 자체는 안 켜도 됨).
    - `pip install primesense pyvirtualcam` 설치돼 있어야 한다.
    - Astra S가 연결돼 있고 장치 관리자에 "ORBBEC Depth Sensor"가 정상(OK)으로 떠 있어야 한다.

사용법:
    python astra_rgb_virtualcam_bridge.py

데이터 수집 전에 이 스크립트를 띄워두고, lerobot 쪽 카메라 설정에서
"OBS Virtual Camera"가 잡히는 index(`output/capture_indexed.py`로 재확인)를
`--robot.cameras`의 `index_or_path`로 쓰면 된다.

(bimanual-robot local patch, 2026-10-13: 자동 재연결)
이 센서가 USB 대역폭 경합 등으로 간헐적으로 "멈춘 프레임을 계속 내보내는"
상태에 빠지는 걸 여러 차례 확인했다 — 에러 없이 조용히 멈추기 때문에,
실제 녹화 세션 하나(5개 에피소드)의 탑뷰 영상이 전부 정지 화면으로
망가진 사고가 있었다. 그래서 연속된 프레임이 일정 횟수 이상 완전히
동일하면(=센서가 멈췄다는 뜻) OpenNI2 연결만 끊었다가 다시 열어서
스스로 복구하도록 했다 — 가상 카메라(pyvirtualcam) 쪽은 계속 열어두므로
lerobot 쪽에서는 index가 끊기지 않는다.
"""

import sys
import time
from pathlib import Path

import numpy as np
import pyvirtualcam
from primesense import openni2

# Astra SDK가 압축 해제된 bin 폴더 — 환경이 바뀌면 이 경로만 고치면 된다.
SDK_BIN = (
    r"C:\Users\USER\Downloads\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
    r"\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64\bin"
)

FPS = 30
# 이만큼 연속으로 완전히 동일한 프레임이 오면 "센서가 멈췄다"고 판단한다.
# 30fps 기준 약 2초치.
STALE_FRAME_LIMIT = FPS * 2


def _connect_color_stream():
    """Astra 장치를 열고 컬러 스트림을 시작해 (device, color_stream, width, height)를 반환한다."""
    try:
        device = openni2.Device.open_any()
    except Exception as exc:  # noqa: BLE001 - 진단용으로 원인 그대로 보여준다
        sys.exit(
            "Astra 장치를 열지 못했습니다. 장치 관리자에서 'ORBBEC Depth Sensor'가 "
            f"정상(OK)인지, 다른 프로그램(SDK 뷰어 등)이 이미 카메라를 쓰고 있지 않은지 확인하세요.\n{exc}"
        )

    color_stream = device.create_color_stream()
    color_stream.start()

    # 워밍업: 오토 노출/화이트밸런스가 지금 조명에 맞게 수렴하도록 몇 프레임 흘려보낸 뒤 고정한다.
    for _ in range(10):
        color_stream.read_frame()

    if color_stream.camera is not None:
        color_stream.camera.auto_exposure = False
        color_stream.camera.auto_white_balance = False
        print(
            "노출/화이트밸런스 고정 완료 "
            f"(gain={color_stream.camera.gain}, exposure={color_stream.camera.exposure}). "
            "이 값은 docs/20261013_뎁스카메라_노출_화이트밸런스_고정.md 에 기록해두고 "
            "평가·추론 때도 반드시 동일하게 적용할 것."
        )
    else:
        print("경고: 이 스트림은 노출/화이트밸런스 수동 제어를 지원하지 않습니다 (자동 상태로 진행).")

    first = color_stream.read_frame()
    width, height = first.width, first.height
    print(f"Astra 컬러 스트림 연결: {width}x{height} @ {FPS}fps")
    return device, color_stream, width, height


def _disconnect(device, color_stream) -> None:
    try:
        color_stream.stop()
    except Exception:  # noqa: BLE001 - 재연결 과정이니 정리 실패는 무시하고 넘어간다
        pass
    try:
        device.close()
    except Exception:  # noqa: BLE001
        pass


def main() -> None:
    if not Path(SDK_BIN).is_dir():
        sys.exit(f"SDK bin 폴더를 찾을 수 없습니다: {SDK_BIN}")

    openni2.initialize(SDK_BIN)

    device, color_stream, width, height = _connect_color_stream()

    with pyvirtualcam.Camera(width=width, height=height, fps=FPS) as cam:
        print(f"가상 카메라 송출 시작: {cam.device} (Ctrl+C로 종료)")
        last_frame_bytes = None
        stale_count = 0
        try:
            while True:
                try:
                    frame = color_stream.read_frame()
                    buf = frame.get_buffer_as_triplet()
                    rgb = np.frombuffer(buf, dtype=np.uint8).reshape(height, width, 3)
                except Exception as exc:  # noqa: BLE001 - 센서 쪽 예외도 재연결로 처리
                    print(f"[watchdog] 프레임 읽기 실패({exc}), 재연결 시도...")
                    _disconnect(device, color_stream)
                    device, color_stream, _, _ = _connect_color_stream()
                    stale_count = 0
                    last_frame_bytes = None
                    continue

                frame_bytes = rgb.tobytes()
                if frame_bytes == last_frame_bytes:
                    stale_count += 1
                else:
                    stale_count = 0
                last_frame_bytes = frame_bytes

                if stale_count >= STALE_FRAME_LIMIT:
                    print(
                        f"[watchdog] 프레임이 {STALE_FRAME_LIMIT}장 연속 동일 — 센서가 멈춘 것으로 "
                        "판단해 재연결합니다."
                    )
                    _disconnect(device, color_stream)
                    device, color_stream, _, _ = _connect_color_stream()
                    stale_count = 0
                    last_frame_bytes = None
                    continue

                cam.send(rgb)
                cam.sleep_until_next_frame()
        except KeyboardInterrupt:
            pass
        finally:
            _disconnect(device, color_stream)
            openni2.unload()


if __name__ == "__main__":
    main()
