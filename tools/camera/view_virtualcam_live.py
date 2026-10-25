"""OBS 가상 카메라(Astra 브리지 출력)를 롤아웃과 같은 좌우반전으로 실시간 표시한다.

이 환경의 OpenCV는 창 기능이 없는 버전이라 tkinter로 띄운다. 창을 닫거나 q로 종료.
브리지(astra_rgb_virtualcam_bridge.py)가 먼저 켜져 있어야 한다. 롤아웃 전에는 닫는다.

사용법:
    python tools/camera/view_virtualcam_live.py
    python tools/camera/view_virtualcam_live.py --no-mirror
    python tools/camera/view_virtualcam_live.py --scale 1.5   # 창을 1.5배로 늘려서 보기
    python tools/camera/view_virtualcam_live.py --ref-dataset C:\\Users\\USER\\lerobot-data\\water-pour-position-jitter-gravel-tabletop-handoff-v2
        -> 학습 때 top 화면을 반투명하게 겹쳐 보여준다. 카메라를 움직여 두 화면이 겹치게 맞춘다.
"""

import argparse
import sys
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path

import cv2
from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_camera_indices import enumerate_devices


def load_reference(dataset_root: str, episode: int):
    """학습 데이터 top 영상의 첫 장면(BGR, 640x480). 녹화 영상은 이미 mirror가 적용돼 있다."""
    import av

    files = sorted((Path(dataset_root) / "videos" / "observation.images.top").glob("chunk-*/*.mp4"))
    if not files:
        sys.exit(f"top 영상을 못 찾았다: {dataset_root}")
    with av.open(str(files[min(episode, len(files) - 1)])) as c:
        img = next(c.decode(video=0)).to_ndarray(format="bgr24")
    return cv2.resize(img, (640, 480))


def _edges(img):
    g = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 50, 150).astype("float32")
    return cv2.GaussianBlur(g, (9, 9), 3)


def offset_from_refs(frame, ref_edges, window) -> tuple[float, float]:
    """학습 장면 여러 개 대비 지금 화면이 몇 px 밀렸는지(중앙값). 물체 배치가 에피소드마다 달라 한 장면만 쓰면 흔들린다."""
    cur = _edges(frame)
    shifts = [cv2.phaseCorrelate(r, cur, window)[0] for r in ref_edges]
    xs, ys = zip(*shifts)
    xs, ys = sorted(xs), sorted(ys)
    return xs[len(xs) // 2], ys[len(ys) // 2]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-mirror", action="store_true", help="좌우반전 없이 원본으로 보기")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="창 확대 배율. 기본 1.0 = 녹화·추론에 들어가는 640x480 그대로")
    ap.add_argument("--ref-dataset", default=None,
                    help="학습 데이터셋 로컬 폴더. 주면 그 top 영상의 한 장면을 반투명하게 겹쳐 카메라 위치 맞추기에 쓴다")
    ap.add_argument("--ref-episode", type=int, default=0, help="겹칠 에피소드 영상 파일 순번")
    ap.add_argument("--ref-alpha", type=float, default=0.5, help="겹친 학습 화면의 진하기(0~1)")
    args = ap.parse_args()
    ref = load_reference(args.ref_dataset, args.ref_episode) if args.ref_dataset else None
    ref_edges, window = [], None
    if args.ref_dataset:
        n_files = len(list((Path(args.ref_dataset) / "videos" / "observation.images.top").glob("chunk-*/*.mp4")))
        picks = sorted({round(i * (n_files - 1) / 7) for i in range(8)})
        ref_edges = [_edges(load_reference(args.ref_dataset, i)) for i in picks]
        window = cv2.createHanningWindow((640, 480), cv2.CV_32F)

    idx = next((i for i, name, _ in enumerate_devices() if name == "OBS Virtual Camera"), None)
    if idx is None:
        sys.exit("OBS Virtual Camera 장치를 못 찾았다. OBS가 설치돼 있는지 확인할 것.")
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 카메라 읽기는 별도 스레드에서 최대 속도로 돌린다. 화면 그리기가 느려도 카메라 fps는 그대로 잰다.
    latest: dict = {"frame": None, "cam_times": deque(maxlen=90), "new": False}
    stop = threading.Event()

    def reader() -> None:
        while not stop.is_set():
            ok, frame = cap.read()
            if ok:
                latest["frame"] = frame
                latest["new"] = True
                latest["cam_times"].append(time.perf_counter())

    threading.Thread(target=reader, daemon=True).start()

    # 어긋남 계산은 무거워서 0.5초마다 별도 스레드에서 한다
    align = {"text": ""}

    def aligner() -> None:
        while not stop.is_set():
            frame = latest["frame"]
            if frame is not None and ref_edges:
                f = frame if args.no_mirror else cv2.flip(frame, 1)
                dx, dy = offset_from_refs(cv2.resize(f, (640, 480)), ref_edges, window)
                ok = abs(dx) <= 3 and abs(dy) <= 3
                align["text"] = (f"\n학습 화면 대비 어긋남: 가로 {dx:+5.1f}px, 세로 {dy:+5.1f}px "
                                 f"({'맞음' if ok else '두 숫자가 0에 가까워지도록 카메라를 조금씩 돌릴 것'})")
            time.sleep(0.5)

    threading.Thread(target=aligner, daemon=True).start()

    root = tk.Tk()
    root.title(f"top depth cam live (index {idx}, mirror={not args.no_mirror}) - q to quit")
    label = tk.Label(root)
    label.pack()
    info = tk.Label(root, font=("Consolas", 12))
    info.pack()
    shown: deque = deque(maxlen=60)

    def rate(times: deque) -> float:
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0

    def tick() -> None:
        frame = latest["frame"]
        if frame is not None and latest["new"]:
            latest["new"] = False
            if not args.no_mirror:
                frame = cv2.flip(frame, 1)
            if ref is not None:
                frame = cv2.addWeighted(frame, 1 - args.ref_alpha, ref, args.ref_alpha, 0)
            if args.scale != 1.0:
                frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = ImageTk.PhotoImage(Image.fromarray(rgb))
            label.configure(image=img)
            label.image = img
            shown.append(time.perf_counter())
            info.configure(text=f"카메라 {rate(latest['cam_times']):4.1f} fps (녹화·추론 기준 30) | "
                                f"화면 표시 {rate(shown):4.1f} fps | 밝기 {frame.mean():5.1f}{align['text']}")
        root.after(5, tick)

    def close(_=None) -> None:
        stop.set()
        time.sleep(0.1)
        cap.release()
        root.destroy()

    root.bind("q", close)
    root.protocol("WM_DELETE_WINDOW", close)
    tick()
    root.mainloop()


if __name__ == "__main__":
    main()
