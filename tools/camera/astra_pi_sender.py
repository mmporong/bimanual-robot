"""Astra RGB·정렬 깊이를 Pi 정밀주차 수신기로 보내는 AST1 송신기.

astra_rgb_virtualcam_bridge.py --send-pi HOST:PORT 로 켠다. 브리지가 장치를 계속 쥐고
가상 카메라(IL용)를 내보내는 동안, 이 모듈이 같은 프레임을 Pi로 보낸다. Windows에는 ROS가 없다.

연결: Windows가 client, Pi가 listen. 끊기면 1초마다 다시 접속한다.
프레임 1개 = 메시지 1개:
    b"AST1" | uint32 BE 헤더 길이 | UTF-8 JSON 헤더 | rgb JPEG(품질 95) | depth PNG(16비트)
시계 동기화: Pi가 b"PING" + uint64 BE pi_ns 를 보내면, 완성된 프레임 사이에
    b"PONG" + uint64 BE pi_ns + uint64 BE win_ns 로 답한다(win_ns는 PONG을 보내기 직전 Windows 시각).
최신 프레임 우선: 보낼 칸은 1개다. 송신이 밀리면 이전 프레임을 버리고 새 프레임으로 덮어쓴다.

stamp_ns는 Windows 시계 기준 촬영 시각이다. 장치 타임스탬프(us)에 "읽은 시각 - 장치 시각"의
최근 최솟값(전송 지연이 가장 작았던 표본)을 더해 만든다. 수신 시각으로 덮어쓰지 않는다.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np

MAGIC = b"AST1"
FRAME_ID = "camera_color_optical_frame"
# 이보다 긴 공백은 즉시 시각과 함께 로그에 남긴다(10 fps 기준 3프레임). 추론 중 송신 공백 기록용
GAP_LOG_S = 0.3


def now_text() -> str:
    return time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"


class DeviceClock:
    """장치 타임스탬프(us)를 Windows time.time_ns() 기준으로 바꾼다."""

    def __init__(self, window: int = 300):
        self.samples: deque[int] = deque(maxlen=window)

    def stamp_ns(self, device_us: int, host_ns: int) -> int:
        self.samples.append(host_ns - device_us * 1000)
        return device_us * 1000 + min(self.samples)


def camera_info(params: dict, width: int, height: int, base_width: int = 640) -> dict:
    """공장 보정값(컬러 쪽 r_intr_p·r_k, base_width 해상도 기준)으로 해당 해상도의 CameraInfo를 만든다.

    깊이는 컬러 좌표계로 정렬한 뒤 2칸마다 1픽셀을 뽑아 줄이므로(u = 2u') K·P를 같은 배율로 줄인다.
    D는 Pi의 ros2_astra_camera 드라이버와 같게 0으로 낸다. 교시 영상·정렬 기준이 D=0으로 만들어졌다
    (2026-10-29 교시 저장값 대조: K는 장치 r_intr_p와 소수점까지 같고 D는 0). 장치 왜곡값은 헤더 factory_D에 따로 싣는다.
    """
    s = width / base_width
    fx, fy, cx, cy = (float(v) for v in params["r_intr_p"])
    fx, fy, cx, cy = fx * s, fy * s, cx * s, cy * s
    return {"width": width, "height": height, "distortion_model": "plumb_bob",
            "D": [0.0, 0.0, 0.0, 0.0, 0.0],
            "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "P": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]}


class PiSender:
    def __init__(self, host: str, port: int, params: dict, depth_size=(320, 240), png_level: int = 1,
                 jpeg_quality: int = 95):
        self.addr = (host, port)
        self.params = params
        self.depth_size = depth_size
        self.png_level = png_level
        self.jpeg_quality = jpeg_quality
        self.cv = threading.Condition()
        self.slot = None  # (rgb_bgr, rgb_stamp_ns, depth_mm, depth_stamp_ns)
        self.pongs: list[int] = []
        self.seq = 0
        self.stats = {"sent": 0, "dropped": 0, "bytes": 0, "connected": False,
                      "max_send_gap_s": 0.0, "max_offer_gap_s": 0.0}
        self._last_sent = None  # 마지막 프레임 송신 완료 시각(monotonic)
        self._last_offer = None  # 브리지가 마지막으로 프레임을 넘긴 시각(monotonic)
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name="ast1-send").start()

    # 브리지 쪽에서 부른다(빠르게 돌아와야 한다)
    def offer(self, rgb_bgr: np.ndarray, rgb_stamp_ns: int, depth_mm: np.ndarray, depth_stamp_ns: int) -> None:
        now = time.monotonic()
        if self._last_offer is not None:
            gap = now - self._last_offer
            self.stats["max_offer_gap_s"] = max(self.stats["max_offer_gap_s"], gap)
            if gap > GAP_LOG_S:
                print(f"[ast1 {now_text()}] 카메라 쪽 공백 {gap:.2f}s (브리지가 프레임을 못 넘김)")
        self._last_offer = now
        with self.cv:
            if self.slot is not None:
                self.stats["dropped"] += 1
            self.slot = (rgb_bgr, rgb_stamp_ns, depth_mm, depth_stamp_ns)
            self.cv.notify()

    def close(self) -> None:
        self.stop.set()
        with self.cv:
            self.cv.notify()

    def _encode(self, item) -> bytes:
        rgb, rgb_ns, depth, depth_ns = item
        h, w = rgb.shape[:2]
        dw, dh = self.depth_size
        depth_small = np.ascontiguousarray(depth[:: depth.shape[0] // dh, :: depth.shape[1] // dw][:dh, :dw])
        # RGB는 JPEG 품질 95(PNG는 프레임당 약 500 KB라 Wi-Fi에서 5 fps에 그쳤다). 깊이는 무손실 PNG16 유지
        ok1, rgb_png = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        ok2, depth_png = cv2.imencode(".png", depth_small.astype(np.uint16), [cv2.IMWRITE_PNG_COMPRESSION, self.png_level])
        if not (ok1 and ok2):
            raise RuntimeError("PNG 인코딩 실패")
        rgb_b, depth_b = rgb_png.tobytes(), depth_png.tobytes()
        self.seq += 1
        header = {
            "seq": self.seq,
            "rgb": {"stamp_ns": rgb_ns, "width": w, "height": h, "encoding": "bgr8", "compression": "jpeg",
                    "len": len(rgb_b), "frame_id": FRAME_ID, "camera_info": camera_info(self.params, w, h)},
            "depth": {"stamp_ns": depth_ns, "width": dw, "height": dh, "encoding": "16UC1", "unit": "mm",
                      "compression": "png16", "len": len(depth_b), "frame_id": FRAME_ID,
                      "camera_info": camera_info(self.params, dw, dh)},
            "factory_D": [float(v) for v in self.params["r_k"]],  # 장치 컬러 왜곡 [k1, k2, p1, p2, k3], 참고용
            "send_ns": time.time_ns(),
        }
        hb = json.dumps(header, separators=(",", ":")).encode()
        return MAGIC + struct.pack(">I", len(hb)) + hb + rgb_b + depth_b

    def _reader(self, sock: socket.socket) -> None:
        buf = b""
        try:
            while not self.stop.is_set():
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 12:
                    if buf[:4] != b"PING":
                        buf = buf[1:]  # 동기 잃으면 한 바이트씩 버려 다시 맞춘다
                        continue
                    (pi_ns,) = struct.unpack(">Q", buf[4:12])
                    buf = buf[12:]
                    with self.cv:
                        self.pongs.append(pi_ns)
                        self.cv.notify()
        except OSError:
            pass

    def _session(self, sock: socket.socket) -> None:
        threading.Thread(target=self._reader, args=(sock,), daemon=True, name="ast1-recv").start()
        while not self.stop.is_set():
            with self.cv:
                while self.slot is None and not self.pongs and not self.stop.is_set():
                    self.cv.wait(timeout=1.0)
                pongs, self.pongs = self.pongs, []
                item, self.slot = self.slot, None
            for pi_ns in pongs:  # 프레임 사이에만 보낸다
                sock.sendall(b"PONG" + struct.pack(">QQ", pi_ns, time.time_ns()))
            if item is not None:
                msg = self._encode(item)
                sock.sendall(msg)
                self.stats["sent"] += 1
                self.stats["bytes"] += len(msg)
                now = time.monotonic()
                if self._last_sent is not None:
                    gap = now - self._last_sent
                    self.stats["max_send_gap_s"] = max(self.stats["max_send_gap_s"], gap)
                    if gap > GAP_LOG_S:
                        print(f"[ast1 {now_text()}] 송신 공백 {gap:.2f}s (seq {self.seq})")
                self._last_sent = now

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                sock = socket.create_connection(self.addr, timeout=3.0)
            except OSError as exc:
                print(f"[ast1 {now_text()}] {self.addr[0]}:{self.addr[1]} 접속 실패({exc}) - 1초 뒤 재시도")
                time.sleep(1.0)
                continue
            sock.settimeout(None)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.stats["connected"] = True
            self._last_sent = None
            print(f"[ast1 {now_text()}] Pi {self.addr[0]}:{self.addr[1]} 연결됨")
            try:
                self._session(sock)
            except OSError as exc:
                print(f"[ast1 {now_text()}] 연결 끊김({exc}) - 1초 뒤 재접속")
            finally:
                self.stats["connected"] = False
                try:
                    sock.close()
                except OSError:
                    pass
            time.sleep(1.0)
