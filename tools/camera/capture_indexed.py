import cv2
import os
import time

OUT_DIR = os.path.join(os.getcwd(), "output")
os.makedirs(OUT_DIR, exist_ok=True)

found = []
for idx in range(6):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        continue
    time.sleep(1.0)
    # discard a few frames to let exposure/auto settings settle
    for _ in range(5):
        cap.read()
    ok, frame = cap.read()
    if ok and frame is not None:
        path = os.path.join(OUT_DIR, f"index{idx}.jpg")
        cv2.imwrite(path, frame)
        found.append((idx, path))
    cap.release()

print("RESULT_START")
for idx, path in found:
    print(f"index{idx}\t{path}")
print("RESULT_END")
