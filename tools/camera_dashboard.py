#!/usr/bin/env python3
"""RGB 카메라 관제와 사용자 요청에 따른 양팔 토크 OFF."""

from __future__ import annotations

import argparse
import json
import pathlib
import signal
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
from dashboard_stop import StopController


HERE = pathlib.Path(__file__).resolve().parent
INDEX = HERE / "camera_dashboard" / "index.html"


class CameraStream:
    def __init__(self, name: str, path: str, width: int, height: int, fps: int):
        self.name = name
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.lock = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.captured_at = 0.0
        self.actual_width = 0
        self.actual_height = 0
        self.error: str | None = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"camera-{name}", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        capture = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not capture.isOpened():
            with self.lock:
                self.error = f"열 수 없음: {self.path}"
                self.lock.notify_all()
            return
        try:
            failures = 0
            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    failures += 1
                    if failures >= 10:
                        with self.lock:
                            self.error = "프레임 읽기 연속 실패"
                            self.lock.notify_all()
                        time.sleep(0.2)
                    continue
                failures = 0
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 86])
                if not ok:
                    continue
                with self.lock:
                    self.jpeg = encoded.tobytes()
                    self.sequence += 1
                    self.captured_at = time.time()
                    self.actual_height, self.actual_width = frame.shape[:2]
                    self.error = None
                    self.lock.notify_all()
        finally:
            capture.release()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            age = None if not self.captured_at else max(0.0, time.time() - self.captured_at)
            return {
                "name": self.name,
                "path": self.path,
                "online": self.jpeg is not None and age is not None and age < 1.0,
                "sequence": self.sequence,
                "frame_age_ms": None if age is None else round(age * 1000),
                "width": self.actual_width,
                "height": self.actual_height,
                "error": self.error,
            }


def make_handler(streams: dict[str, CameraStream], stop=None):
    stop = stop or StopController()
    token = secrets.token_urlsafe(32)
    index_bytes = INDEX.read_text(encoding="utf-8").replace("__STOP_TOKEN__", token).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        server_version = "HoldFlowCameraDashboard/1.0"

        def log_message(self, fmt: str, *args: object) -> None:
            return

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            path = urlsplit(self.path).path
            if path == "/":
                self._send(200, "text/html; charset=utf-8", index_bytes)
                return
            if path == "/api/status":
                body = json.dumps(
                    {name: stream.snapshot() for name, stream in streams.items()},
                    ensure_ascii=False,
                ).encode("utf-8")
                self._send(200, "application/json; charset=utf-8", body)
                return
            if path == "/api/stop/status":
                self._send(200, "application/json", json.dumps(stop.snapshot()).encode())
                return
            if path.startswith("/stream/"):
                name = path.removeprefix("/stream/")
                stream = streams.get(name)
                if stream is None:
                    self._send(404, "text/plain; charset=utf-8", b"unknown camera")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_sequence = -1
                try:
                    while True:
                        with stream.lock:
                            stream.lock.wait_for(
                                lambda: stream.sequence != last_sequence or stream.stop_event.is_set(),
                                timeout=1.0,
                            )
                            if stream.stop_event.is_set():
                                return
                            jpeg = stream.jpeg
                            last_sequence = stream.sequence
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            self._send(404, "text/plain; charset=utf-8", b"not found")

        def do_POST(self) -> None:  # noqa: N802
            if urlsplit(self.path).path != "/api/stop":
                self._send(404, "text/plain", b"not found")
                return
            expected = f"http://127.0.0.1:{self.server.server_port}"
            if self.headers.get("Origin") != expected or not secrets.compare_digest(
                self.headers.get("X-Stop-Token", ""), token
            ):
                self._send(403, "text/plain", b"same-origin stop token required")
                return
            if not stop.snapshot()["enabled"]:
                self._send(503, "text/plain", b"stop ports not configured")
                return
            result = stop.stop()
            self._send(200 if result["verified_off"] else 503,
                       "application/json", json.dumps(result).encode())

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--top",
        default="/dev/v4l/by-path/pci-0000:67:00.3-usb-0:1.1:1.0-video-index0",
    )
    parser.add_argument(
        "--wrist",
        default="/dev/v4l/by-path/pci-0000:67:00.4-usb-0:1:1.0-video-index0",
    )
    parser.add_argument("--wrist-other", default="/dev/v4l/by-path/pci-0000:65:00.3-usb-0:4.1:1.0-video-index0")
    parser.add_argument("--stop-port", action="append", default=[], help="토크 OFF 대상 팔 포트; 팔마다 지정")
    parser.add_argument("--stop-log", default=str(pathlib.Path.home() / ".local/state/bimanual-robot/dashboard-stop.jsonl"))
    parser.add_argument("--http", type=int, default=8770)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    streams = {
        "top": CameraStream("top", args.top, args.width, args.height, args.fps),
        "wrist": CameraStream("wrist", args.wrist, args.width, args.height, args.fps),
        "wrist_other": CameraStream("wrist_other", args.wrist_other, args.width, args.height, args.fps),
    }
    if len({str(pathlib.Path(p).resolve()) for p in args.stop_port}) != len(args.stop_port):
        raise SystemExit("같은 팔 포트를 중복 지정할 수 없습니다.")
    stop = StopController(args.stop_port, log_path=args.stop_log)
    server = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(streams, stop))
    server.daemon_threads = True
    for stream in streams.values():
        stream.start()

    def shutdown(_signum: int, _frame: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"카메라·토크 OFF 대시보드: http://127.0.0.1:{args.http}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        for stream in streams.values():
            stream.stop()


if __name__ == "__main__":
    main()
