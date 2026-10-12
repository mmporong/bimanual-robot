"""Astra RGB 연속 스트림 - astra_stream 헬퍼 바이너리를 띄우고 최신 컬러 프레임만 보관한다.

Orbbec Astra는 컬러 영상이 일반 웹캠(/dev/video*)으로 잡히지 않아 OpenNI2 C++ API로 만든
작은 헬퍼 프로그램을 서브프로세스로 띄우고 stdout으로 raw 프레임을 받는다. 헬퍼 경로는
환경변수 ASTRA_HELPER로 지정한다. Astra는 OpenNI2 프로세스 하나만 열 수 있으므로, 다른
프로그램이 잡고 있으면 첫 프레임 시간 초과로 실패한다.

프레임 포맷: uint8 stream_id(0=color,1=depth) + uint32 width + uint32 height +
uint32 num_bytes + 픽셀 바이트. color는 RGB888.
"""

from __future__ import annotations

import os
import struct
import subprocess
import threading

import numpy as np

HELPER_PATH = os.environ.get("ASTRA_HELPER", "astra_stream")  # OpenNI2 헬퍼 실행 파일 경로
_HEADER = struct.Struct("<BIII")


def _read_exact(stream, n: int) -> bytes | None:
    chunks = []
    while n > 0:
        chunk = stream.read(n)
        if not chunk:
            return None
        chunks.append(chunk)
        n -= len(chunk)
    return b"".join(chunks)


class AstraStream:
    def __init__(self, helper_path: str = HELPER_PATH) -> None:
        self._helper_path = helper_path
        self._process: subprocess.Popen | None = None
        self._frame: np.ndarray | None = None
        self._frame_id = 0
        self._cond = threading.Condition()

    def start(self, timeout_s: float = 20.0) -> None:
        import shutil
        if not (os.path.isfile(self._helper_path) or shutil.which(self._helper_path)):
            raise RuntimeError(f"Astra 헬퍼를 찾을 수 없다: {self._helper_path} - 환경변수 ASTRA_HELPER로 경로를 지정하거나 "
                               "--camera로 일반 웹캠을 쓴다.")
        self._process = subprocess.Popen([self._helper_path], stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL, bufsize=0)
        threading.Thread(target=self._read_loop, daemon=True).start()
        with self._cond:
            if not self._cond.wait_for(lambda: self._frame is not None, timeout_s):
                self.stop()
                raise RuntimeError(f"Astra 첫 프레임을 {timeout_s:.0f}초 안에 못 받음 - "
                                   "다른 프로그램(server.app 등)이 카메라를 잡고 있는지 확인하세요.")

    def _read_loop(self) -> None:
        stdout = self._process.stdout
        while True:
            header = _read_exact(stdout, _HEADER.size)
            if header is None:
                break
            stream_id, width, height, size = _HEADER.unpack(header)
            payload = _read_exact(stdout, size)
            if payload is None:
                break
            if stream_id != 0:
                continue
            frame = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3).copy()
            with self._cond:
                self._frame = frame
                self._frame_id += 1
                self._cond.notify_all()

    def read_new(self, last_id: int, timeout_s: float = 2.0) -> tuple[int, np.ndarray]:
        """last_id 이후 새로 들어온 RGB 프레임을 기다려 (frame_id, 프레임)을 반환한다."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._frame_id != last_id, timeout_s):
                raise RuntimeError("Astra 새 프레임 시간 초과 - 카메라 연결을 확인하세요.")
            return self._frame_id, self._frame.copy()

    def stop(self) -> None:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
