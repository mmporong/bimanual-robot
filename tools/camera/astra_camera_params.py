"""Astra 장치에 저장된 공장 보정값(orbbec_camera_params)을 읽어 JSON으로 저장한다.

astra_rgb_virtualcam_bridge.py --send-pi 가 이 값으로 Pi에 보낼 CameraInfo를 만든다.
Pi의 astra_camera 드라이버가 CameraInfo를 만들 때 읽는 값과 같은 출처다
(l_* = IR/깊이 카메라, r_* = 컬러 카메라, intr_p = [fx, fy, cx, cy], k = [k1, k2, p1, p2, k3]).
장치를 한 프로그램만 열 수 있으므로 브리지를 끈 상태에서 한 번 실행한다.

사용법:
    python tools/camera/astra_camera_params.py            # config/camera/astra_params.json 에 저장
"""
import ctypes as C
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from astra_sdk import find_sdk_bin  # noqa: E402

SDK_BIN = find_sdk_bin()
os.add_dll_directory(SDK_BIN)
os.chdir(SDK_BIN)
astra = C.CDLL(os.path.join(SDK_BIN, "astra.dll"))
core = C.CDLL(os.path.join(SDK_BIN, "astra_core.dll"))


class Params(C.Structure):
    _fields_ = [("l_intr_p", C.c_float * 4), ("r_intr_p", C.c_float * 4), ("r2l_r", C.c_float * 9),
                ("r2l_t", C.c_float * 3), ("l_k", C.c_float * 5), ("r_k", C.c_float * 5)]


conn = C.c_void_p()
reader = C.c_void_p()
print("init", core.astra_initialize())
print("open", core.astra_streamset_open(b"device/default", C.byref(conn)))
print("reader", core.astra_reader_create(conn, C.byref(reader)))
ds = C.c_void_p()
print("depthstream", astra.astra_reader_get_depthstream(reader, C.byref(ds)))
print("start", core.astra_stream_start(ds))
for _ in range(30):
    core.astra_update()
    time.sleep(0.05)
p = Params()
rc = astra.astra_get_orbbec_camera_params(conn, C.byref(p))
print("params rc", rc)
out = {f: list(getattr(p, f)) for f, _ in Params._fields_}
print(json.dumps(out, indent=1))
dest = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / "config" / "camera" / "astra_params.json"
if rc == 0 and any(out["r_intr_p"]):
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"저장: {dest}")
else:
    print("보정값을 읽지 못해 저장하지 않았습니다.")
core.astra_stream_stop(ds)
core.astra_reader_destroy(C.byref(reader))
core.astra_streamset_close(C.byref(conn))
core.astra_terminate()
