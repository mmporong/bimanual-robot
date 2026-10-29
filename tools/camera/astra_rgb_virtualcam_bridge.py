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
    python astra_rgb_virtualcam_bridge.py                 # 자동 수렴 후 그 값으로 고정 (기존 동작)
    python astra_rgb_virtualcam_bridge.py --exposure 83   # 노출 직접 고정 (단위 0.1 ms)

천장 형광등 같은 60Hz 조명 아래에서는 화면에 가로 줄무늬가 흘러간다. 노출을 83의 정수배로
고정하면 사라진다. 값 고르기는 astra_flicker_sweep.py 로 측정한다(2026-10-23: Astra S, 실내
형광등 기준 노출 83에서 줄무늬 24.2 -> 0.15, 다만 그 조명에선 너무 밝아 투명 병이 날아갔다).

데이터 수집 전에 이 스크립트를 띄워두고, lerobot 쪽 카메라 설정에서
"OBS Virtual Camera"가 잡히는 index(`tools/camera/capture_indexed.py`로 재확인)를
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

import argparse
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
# --exposure/--gain 으로 지정하면 자동 수렴값 대신 이 값으로 고정한다(재연결 때도 동일 적용).
MANUAL: dict[str, int | None] = {"exposure": None, "gain": None}
# 이만큼 연속으로 완전히 동일한 프레임이 오면 "센서가 멈췄다"고 판단한다.
# 30fps 기준 약 2초치.
STALE_FRAME_LIMIT = FPS * 2
# --send-pi 용 공장 보정값. 장치마다 다르므로 astra_camera_params.py로 이 장치에서 읽어 저장한다.
DEFAULT_PARAMS = Path(__file__).resolve().parents[2] / "config" / "camera" / "astra_params.json"


def _open_depth_stream(device, color_stream):
    """--send-pi 용: 320x240 1mm 깊이를 컬러 좌표계로 정렬(depth->color registration)해 연다.

    Pi 드라이버 설정(depth 320x240, depth_registration=true)과 같다. 640x480 깊이를 함께 열었을 때는
    3~6분마다 센서가 멈췄다(USB 대역폭 추정, 2026-10-29). QVGA는 깊이 대역폭이 1/4이다.

    컬러 스트림 설정(IL 가상 카메라가 쓰는 화면)은 건드리지 않는다. 깊이 미러링만 컬러와 같게 맞추고,
    Pi로 보낼 때 미러가 켜져 있으면 둘 다 좌우를 되돌린다(Pi 기존 설정은 미러링 없음).
    """
    from primesense import _openni2 as c_api

    depth = device.create_depth_stream()
    depth.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
                                            resolutionX=320, resolutionY=240, fps=FPS))
    if not device.is_image_registration_mode_supported(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR):
        sys.exit("이 장치는 depth->color registration을 지원하지 않습니다. Pi 송신을 켤 수 없습니다.")
    device.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    depth.set_mirroring_enabled(color_stream.get_mirroring_enabled())
    depth.start()
    mode = depth.get_video_mode()
    print(f"Astra 깊이 스트림 연결: {mode.resolutionX}x{mode.resolutionY} 1mm, 컬러에 정렬, "
          f"미러링 컬러={color_stream.get_mirroring_enabled()} 깊이={depth.get_mirroring_enabled()}")
    return depth


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
        # 드라이버는 자기가 기억하는 값과 같은 값을 쓰면 무시한다(읽기는 그 값으로 나오는데
        # 센서는 자동 노출 값 그대로). 그래서 한 칸 다른 값을 먼저 쓰고 목표값을 쓴다.
        for key in ("gain", "exposure"):
            value = MANUAL[key]
            if value is None:
                continue
            setattr(color_stream.camera, key, value + 1)
            for _ in range(5):
                color_stream.read_frame()
            setattr(color_stream.camera, key, value)
            for _ in range(10):
                color_stream.read_frame()
        print(
            "노출/화이트밸런스 고정 완료 "
            f"(gain={color_stream.camera.gain}, exposure={color_stream.camera.exposure}). "
            "이 값은 docs/guides/20261013_뎁스카메라_노출_화이트밸런스_고정.md 에 기록해두고 "
            "평가·추론 때도 반드시 동일하게 적용할 것."
        )
    else:
        print("경고: 이 스트림은 노출/화이트밸런스 수동 제어를 지원하지 않습니다 (자동 상태로 진행).")

    first = color_stream.read_frame()
    width, height = first.width, first.height
    print(f"Astra 컬러 스트림 연결: {width}x{height} @ {FPS}fps")
    return device, color_stream, width, height


def _disconnect(device, color_stream, depth_stream=None) -> None:
    # 스트림을 장치보다 먼저 닫는다. 닫지 않은 스트림이 장치 해제 뒤 GC로 지워지면 프로세스가 죽는다.
    for stream in (depth_stream, color_stream):
        if stream is None:
            continue
        for step in (stream.stop, stream.close):
            try:
                step()
            except Exception:  # noqa: BLE001 - 재연결 과정이니 정리 실패는 무시하고 넘어간다
                pass
    try:
        device.close()
    except Exception:  # noqa: BLE001
        pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exposure", type=int, default=None,
                    help="노출 고정값(단위 0.1 ms). 60Hz 조명 줄무늬는 83의 정수배(83/167/250)에서 사라진다")
    ap.add_argument("--gain", type=int, default=None, help="gain 고정값. 이 장치는 32 이하가 모두 같은 밝기")
    ap.add_argument("--send-pi", default=None, metavar="HOST:PORT",
                    help="RGB·정렬 깊이를 Pi 정밀주차 수신기로 AST1 송신(예: 192.168.0.159:5600)")
    ap.add_argument("--send-fps", type=float, default=10.0, help="Pi 송신 주기")
    ap.add_argument("--params", default=str(DEFAULT_PARAMS),
                    help="astra_camera_params.py가 장치에서 읽어 저장한 공장 보정값 JSON")
    args = ap.parse_args()
    MANUAL["exposure"], MANUAL["gain"] = args.exposure, args.gain
    if not Path(SDK_BIN).is_dir():
        sys.exit(f"SDK bin 폴더를 찾을 수 없습니다: {SDK_BIN}")

    sender = None
    if args.send_pi:
        import json

        from astra_pi_sender import DeviceClock, PiSender

        if not Path(args.params).is_file():
            sys.exit(f"보정값 파일이 없습니다: {args.params}\n브리지를 끈 상태에서 먼저 "
                     "python tools/camera/astra_camera_params.py 를 실행하세요.")
        params = json.loads(Path(args.params).read_text(encoding="utf-8"))
        host, port = args.send_pi.rsplit(":", 1)
        sender = PiSender(host, int(port), params)
        clock = DeviceClock()
        # 카메라 30fps 간격(33ms)이 조금만 흔들려도 정확히 100ms로 자르면 4프레임마다(7.5fps) 보낸다. 반 프레임 당긴다
        send_every_ns = int(1e9 / args.send_fps - 0.5e9 / FPS)
        last_send_ns = 0
        latest_depth = None  # (depth_mm, stamp_ns)
        print(f"Pi 송신: {host}:{port} {args.send_fps:g}fps (RGB 640x480 JPEG q95, 깊이 320x240 16비트 PNG, 컬러에 정렬)")

    openni2.initialize(SDK_BIN)

    device, color_stream, width, height = _connect_color_stream()
    depth_stream = _open_depth_stream(device, color_stream) if sender else None

    with pyvirtualcam.Camera(width=width, height=height, fps=FPS) as cam:
        print(f"가상 카메라 송출 시작: {cam.device} (Ctrl+C로 종료)")
        last_frame_bytes = None
        stale_count = 0
        last_report = time.monotonic()
        try:
            while True:
                try:
                    # 센서가 프레임을 아예 안 보내면 read_frame()이 영원히 막혀 아래 동일프레임
                    # 감시가 동작하지 않는다(가상 카메라는 마지막 화면을 계속 내보냄). 2초 넘게 안 오면 재연결.
                    streams = [color_stream] + ([depth_stream] if depth_stream else [])
                    ready = openni2.wait_for_any_stream(streams, timeout=2.0)
                    if ready is None:
                        raise TimeoutError("2초 동안 새 프레임 없음")
                    if depth_stream is not None and ready is depth_stream:
                        dframe = depth_stream.read_frame()
                        host_ns = time.time_ns()
                        dmm = np.frombuffer(dframe.get_buffer_as_uint16(), dtype=np.uint16).reshape(
                            dframe.height, dframe.width).copy()
                        latest_depth = (dmm, clock.stamp_ns(dframe.timestamp, host_ns))
                        continue
                    frame = color_stream.read_frame()
                    host_ns = time.time_ns()
                    buf = frame.get_buffer_as_triplet()
                    rgb = np.frombuffer(buf, dtype=np.uint8).reshape(height, width, 3)
                except Exception as exc:  # noqa: BLE001 - 센서 쪽 예외도 재연결로 처리
                    print(f"[watchdog] 프레임 읽기 실패({exc}), 재연결 시도...")
                    _disconnect(device, color_stream, depth_stream)
                    device, color_stream, _, _ = _connect_color_stream()
                    depth_stream = _open_depth_stream(device, color_stream) if sender else None
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
                    _disconnect(device, color_stream, depth_stream)
                    device, color_stream, _, _ = _connect_color_stream()
                    depth_stream = _open_depth_stream(device, color_stream) if sender else None
                    stale_count = 0
                    last_frame_bytes = None
                    continue

                cam.send(rgb)
                if sender is None:
                    cam.sleep_until_next_frame()
                    continue

                rgb_ns = clock.stamp_ns(frame.timestamp, host_ns)
                if latest_depth is not None and rgb_ns - last_send_ns >= send_every_ns:
                    last_send_ns = rgb_ns
                    bgr, dmm = rgb[:, :, ::-1], latest_depth[0]
                    if color_stream.get_mirroring_enabled():  # Pi 기존 설정(미러링 없음)에 맞춘다
                        bgr, dmm = bgr[:, ::-1], dmm[:, ::-1]
                    sender.offer(np.ascontiguousarray(bgr), rgb_ns, dmm, latest_depth[1])
                if time.monotonic() - last_report > 10.0:
                    last_report = time.monotonic()
                    s = sender.stats
                    print(f"[ast1] 연결={s['connected']} 보냄={s['sent']} 버림={s['dropped']} "
                          f"{s['bytes'] / 1e6:.1f} MB")
        except KeyboardInterrupt:
            pass
        finally:
            if sender:
                sender.close()
            _disconnect(device, color_stream, depth_stream)
            openni2.unload()


if __name__ == "__main__":
    main()
