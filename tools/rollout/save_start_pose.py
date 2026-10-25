"""텔레옵으로 팔로워 자세를 맞춘 뒤 롤아웃 시작 자세로 저장한다.

리더를 움직이면 팔로워가 따라온다(카메라 없이 관절만 연결). 원하는 자세에서 Enter를
누르면 팔로워 12관절 측정값을 config/rollout_poses/<이름>.json으로 저장한다. 여러 번
눌러도 되고 마지막 저장이 남는다(이전 파일은 타임스탬프 백업). q + Enter로 끝낸다.
끝낼 때 팔로워는 마지막 자세에서 토크를 유지한다(disable_torque_on_disconnect=false).

저장한 자세는 rollout_handoff.py에서 쓴다.
    --start.pose=<이름> : 추론 직전에 맞춘다 (물따르기)
    --end.pose=<이름>   : 추론이 다 끝난 뒤 맞춘다 (컵배치)

사용법:
    python tools/rollout/save_start_pose.py --name pour
    python tools/rollout/save_start_pose.py --name place_end
"""
from __future__ import annotations

import argparse
import json
import queue
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig
from lerobot.utils.robot_utils import precise_sleep

REPO_ROOT = Path(__file__).resolve().parents[2]
CALIB_FOLLOWER = REPO_ROOT / "calibration" / "bi_follower"
CALIB_LEADER = REPO_ROOT / "calibration" / "bi_leader"
POSE_DIR = REPO_ROOT / "config" / "rollout_poses"
FPS = 30


def save_pose(name: str, obs: dict) -> Path:
    POSE_DIR.mkdir(parents=True, exist_ok=True)
    path = POSE_DIR / f"{name}.json"
    if path.exists():
        shutil.copy(path, path.with_name(f"{name}_{datetime.now():%Y%m%d_%H%M%S}.json.bak"))
    pose = {k: round(float(v), 2) for k, v in obs.items() if k.endswith(".pos")}
    path.write_text(json.dumps({"saved_at": datetime.now().isoformat(timespec="seconds"), "pose": pose},
                               indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="저장 이름 (예: pour, place)")
    ns = ap.parse_args()

    robot = BiSOFollower(BiSOFollowerConfig(
        id="arms",
        calibration_dir=CALIB_FOLLOWER,
        left_arm_config=SOFollowerConfig(port="COM15", gripper_protection=True, disable_torque_on_disconnect=False),
        right_arm_config=SOFollowerConfig(port="COM16", disable_torque_on_disconnect=False),
    ))
    teleop = BiSOLeader(BiSOLeaderConfig(
        id="arms",
        calibration_dir=CALIB_LEADER,
        left_arm_config=SOLeaderConfig(port="COM5"),
        right_arm_config=SOLeaderConfig(port="COM14"),
    ))
    teleop.connect()
    robot.connect()

    keys: queue.Queue[str] = queue.Queue()
    threading.Thread(target=lambda: [keys.put(input().strip().lower()) for _ in iter(int, 1)], daemon=True).start()
    print(f"\n텔레옵 중. 원하는 자세에서 Enter = '{ns.name}' 저장, q + Enter = 종료\n")

    try:
        while True:
            t0 = time.perf_counter()
            robot.send_action(teleop.get_action())
            try:
                key = keys.get_nowait()
            except queue.Empty:
                key = None
            if key == "q":
                break
            if key is not None:
                obs = robot.get_observation()
                path = save_pose(ns.name, obs)
                vals = ", ".join(f"{k.removesuffix('.pos')}={float(v):.1f}" for k, v in obs.items() if k.endswith(".pos"))
                print(f"[저장] {path}\n  {vals}\n")
            precise_sleep(max(1 / FPS - (time.perf_counter() - t0), 0))
    finally:
        teleop.disconnect()
        robot.disconnect()
        print("종료 - 팔로워는 마지막 자세에서 토크 유지")


if __name__ == "__main__":
    main()
