#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""출력 방향 점검. 축정렬 6방향을 비교해 현재 방향이 최선인지 본다.

판정 기준 셋:
  1) 세장비 = 높이 / 바닥 최소변. 크면 넘어진다
  2) 서포트 면적 = 임계각보다 가파른 아래보기 면의 넓이 합
  3) 바닥 면적 = 첫 레이어 접지 면적. 클수록 붙는다
"""
import struct, sys, glob, os
import numpy as np

THRESHOLD = 30.0   # 슬라이서 support_threshold_angle 과 같게

def load(path):
    raw = open(path, 'rb').read()
    n = struct.unpack('<I', raw[80:84])[0]
    a = np.frombuffer(raw[84:84+n*50], dtype=np.uint8).reshape(n, 50)
    return a[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64)

ROT = {
    "그대로":  np.eye(3),
    "X+90":  np.array([[1,0,0],[0,0,-1],[0,1,0]], float),
    "X180":  np.array([[1,0,0],[0,-1,0],[0,0,-1]], float),
    "X-90":  np.array([[1,0,0],[0,0,1],[0,-1,0]], float),
    "Y+90":  np.array([[0,0,1],[0,1,0],[-1,0,0]], float),
    "Y-90":  np.array([[0,0,-1],[0,1,0],[1,0,0]], float),
}

def metrics(tris):
    n = np.cross(tris[:,1]-tris[:,0], tris[:,2]-tris[:,0])
    area = np.linalg.norm(n, axis=1) / 2.0
    ok = area > 1e-12
    n_hat = np.zeros_like(n); n_hat[ok] = n[ok] / (area[ok, None] * 2.0)
    pts = tris.reshape(-1, 3)
    lo, hi = pts.min(0), pts.max(0)
    ext = hi - lo
    # 아래를 보는 면 중 바닥과 이루는 각이 임계보다 작은 것 = 서포트 대상
    down = n_hat[:, 2] < 0
    tilt = np.degrees(np.arcsin(np.clip(-n_hat[:, 2], 0, 1)))   # 90 이면 완전 수평 바닥면
    bottom = down & (np.abs(tris[:,:,2].max(axis=1) - lo[2]) < 0.05)
    need = down & (tilt > (90.0 - THRESHOLD)) & ~bottom
    return dict(
        ext=ext,
        floor=ext[0]*ext[1],
        contact=float(area[bottom].sum()),
        support=float(area[need].sum()),
        slender=ext[2] / max(min(ext[0], ext[1]), 1e-9),
    )

print(f"{'부품':<16} {'방향':<7} {'크기 (mm)':<24} {'세장비':>6} {'접지 mm2':>9} {'서포트 mm2':>10}")
print("-" * 82)
for path in sys.argv[1:]:
    base = load(path)
    name = os.path.basename(path).replace('.stl', '')
    rows = []
    for label, R in ROT.items():
        t = base @ R.T
        m = metrics(t)
        rows.append((label, m))
    cur = rows[0][1]
    # 추천: 세장비 4 이하 중 서포트 면적 최소, 동률이면 접지 큰 쪽
    ok = [r for r in rows if r[1]['slender'] <= 4.0] or rows
    best = min(ok, key=lambda r: (round(r[1]['support']), -r[1]['contact']))
    for label, m in rows:
        mark = ""
        if label == "그대로": mark = " ← 현재"
        if label == best[0] and best[0] != "그대로": mark = " ← 추천"
        e = m['ext']
        print(f"{name if label=='그대로' else '':<16} {label:<7} "
              f"{e[0]:6.1f} x {e[1]:6.1f} x {e[2]:6.1f}   "
              f"{m['slender']:6.2f} {m['contact']:9.0f} {m['support']:10.0f}{mark}")
    print()
