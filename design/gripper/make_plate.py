#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""여러 STL 을 한 판에 배치해 하나의 STL 로 합친다.

부품별 G-code 를 따로 관리하지 않고 한 번 슬라이싱하려고 쓴다. 배치는
바닥면적 기준 greedy shelf packing 이고, 부품 사이 간격과 베드 여백을 지킨다.

경로 뒤에 `:회전` 을 붙이면 축정렬 90도 회전을 먼저 적용한다. 회전 이름은
X+90 / X180 / X-90 / Y+90 / Y-90 이다. 방향 판단은 audit_orient.py 로 한다.

    python3 make_plate.py --out plate.stl --bed 300 300 a.stl b.stl:Y-90 ...
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np


def load(path):
    raw = Path(path).read_bytes()
    count = struct.unpack("<I", raw[80:84])[0]
    block = np.frombuffer(raw[84:84 + count * 50], dtype=np.uint8).reshape(count, 50)
    return block[:, 12:48].copy().view(np.float32).reshape(count, 3, 3).astype(np.float64)


def save(path, tris):
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    body = bytearray(b"\0" * 80 + struct.pack("<I", len(tris)))
    packed = np.concatenate([normals[:, None, :], tris], axis=1).astype(np.float32)
    for row in packed:
        body += row.tobytes() + b"\0\0"
    Path(path).write_bytes(bytes(body))


ROTATIONS = {
    "X+90": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),
    "X180": np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float),
    "X-90": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),
    "Y+90": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),
    "Y-90": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float),
}


def split_spec(spec):
    """`경로:회전` 을 (경로, 회전이름) 으로 나눈다. 회전이 없으면 None."""
    if ":" in spec:
        path, turn = spec.rsplit(":", 1)
        if turn in ROTATIONS:
            return path, turn
    return spec, None


def pack(items, bed_x, bed_y, gap, margin):
    """shelf packing. items 는 (이름, 폭, 깊이) 이고 넓은 것부터 채운다."""
    usable_x = bed_x - 2 * margin
    placed, shelf, shelf_y, shelf_h, cursor = [], [], margin, 0.0, margin
    for name, w, d in items:
        if cursor + w > margin + usable_x and shelf:
            shelf_y += shelf_h + gap
            cursor, shelf_h, shelf = margin, 0.0, []
        if shelf_y + d > bed_y - margin:
            raise SystemExit(f"베드를 넘는다: {name}")
        placed.append((name, cursor, shelf_y))
        shelf.append(name)
        cursor += w + gap
        shelf_h = max(shelf_h, d)
    return placed, shelf_y + shelf_h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--bed", nargs=2, type=float, default=[300.0, 300.0])
    ap.add_argument("--gap", type=float, default=8.0, help="부품 간격 mm. 브림을 고려한다")
    ap.add_argument("--margin", type=float, default=10.0, help="베드 가장자리 여백 mm")
    a = ap.parse_args()

    meshes = {}
    items = []
    labels = {}
    for spec in a.stl:
        path, turn = split_spec(spec)
        tris = load(path)
        if turn:
            tris = tris @ ROTATIONS[turn].T
        points = tris.reshape(-1, 3)
        low = points.min(axis=0)
        tris = tris - low                      # 원점으로 당긴다. Z 도 0 부터
        meshes[spec] = tris
        labels[spec] = path
        size = points.max(axis=0) - low
        items.append((spec, size[0], size[1]))

    items.sort(key=lambda r: -r[1] * r[2])
    placed, used_y = pack(items, a.bed[0], a.bed[1], a.gap, a.margin)

    # 판 전체를 베드 중앙으로 옮긴다
    spans = [(x, y, meshes[n].reshape(-1, 3).max(axis=0)) for n, x, y in placed]
    max_x = max(x + s[0] for x, _, s in spans)
    max_y = max(y + s[1] for _, y, s in spans)
    shift_x = (a.bed[0] - max_x) / 2.0
    shift_y = (a.bed[1] - max_y) / 2.0

    chunks = []
    print(f"베드 {a.bed[0]:.0f} x {a.bed[1]:.0f}, 간격 {a.gap:.0f} mm, 여백 {a.margin:.0f} mm")
    for name, x, y in placed:
        tris = meshes[name] + np.array([x + shift_x, y + shift_y, 0.0])
        chunks.append(tris)
        size = meshes[name].reshape(-1, 3).max(axis=0)
        turn = split_spec(name)[1]
        print(f"  {Path(labels[name]).stem:<18} 위치 ({x + shift_x:6.1f}, {y + shift_y:6.1f})"
              f"  크기 {size[0]:5.1f} x {size[1]:5.1f} x {size[2]:5.1f}"
              f"  {'회전 ' + turn if turn else ''}")

    merged = np.concatenate(chunks, axis=0)
    save(a.out, merged)
    total = merged.reshape(-1, 3)
    span = total.max(axis=0) - total.min(axis=0)
    print(f"\n합본 {a.out}")
    print(f"  삼각형 {len(merged)}, 전체 {span[0]:.1f} x {span[1]:.1f} x {span[2]:.1f} mm")
    if span[0] > a.bed[0] or span[1] > a.bed[1]:
        sys.exit("베드 초과")


if __name__ == "__main__":
    main()
