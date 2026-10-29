"""로봇 없이 연결된 카메라(왼쪽 손목·오른쪽 손목·top)를 rerun에 실시간으로 띄운다.

카메라 번호는 resolve_camera_indices로 실행 직전에 찾는다. top은 롤아웃·녹화와 같게
좌우반전해서 보여준다. 뎁스(top)는 브리지(astra_rgb_virtualcam_bridge.py)가 켜져 있어야 나온다.
Ctrl+C로 종료. 롤아웃·녹화·텔레옵을 돌리기 전에는 끈다(같은 카메라를 동시에 열 수 없음).

사용법:
    python tools/camera/rerun_cameras_live.py
"""

import sys
import threading
import time
from pathlib import Path

import cv2
import rerun as rr

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_camera_indices import resolve

FPS = 30
MIRROR = {"top": True}


def stream(name: str, index: int, stop: threading.Event) -> None:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    n, t0 = 0, time.perf_counter()
    while not stop.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        if MIRROR.get(name):
            frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rr.log(f"camera/{name}", rr.Image(rgb).compress(jpeg_quality=85))
        n += 1
        if n % FPS == 0:
            rr.log(f"stats/{name}/fps", rr.Scalars(n / (time.perf_counter() - t0)))
            rr.log(f"stats/{name}/brightness", rr.Scalars(float(frame.mean())))
    cap.release()


def main() -> None:
    idx = resolve()
    cams = {"left_wrist": idx["left"], "right_wrist": idx["right"], "top": idx["depth"]}
    print(f"[camera resolve] {cams}")
    rr.init("camera_live")
    # conda 환경을 활성화하지 않은 PowerShell에선 rerun.exe가 PATH에 없어 경로를 직접 준다.
    # 이미 켜둔 rerun 창(포트 9876)이 있으면 새로 띄우지 않고 그 창에 붙는다.
    rr.spawn(executable_path=str(Path(sys.prefix) / "Scripts" / "rerun.exe"))
    stop = threading.Event()
    threads = [threading.Thread(target=stream, args=(n, i, stop), daemon=True) for n, i in cams.items()]
    for t in threads:
        t.start()
    print("rerun 창에 카메라 3대가 나온다. Ctrl+C로 종료")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2)


if __name__ == "__main__":
    main()
