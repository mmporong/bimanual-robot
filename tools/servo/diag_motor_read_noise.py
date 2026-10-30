"""진단: 토크를 켠 채(자세 고정) 12개 모터의 Present_Position을 고주파로 읽어,
팔이 물리적으로 가만히 있는데도 읽는 값이 떨리는 모터가 있는지 찾는다.

이동 명령(send_action/Goal_Position)은 전혀 보내지 않는다 — 오직 읽기만 한다.
토크는 connect() 시 켜지며, 팔을 현재 자세로 '고정'해서 중력 처짐을 막는다.
따라서 읽는 값의 변동(std)은 물리적 움직임이 아니라 센서/통신 노이즈다.

특정 모터(특히 왼팔 id=4 wrist_flex, id=5 wrist_roll)만 std가 유독 크면,
그 모터의 전기 연결/통신이 불안정하다는 강한 증거다 — 떨림의 유력 원인.

사용법: python tools/servo/diag_motor_read_noise.py   (저장소 루트에서)
끝나면 토크를 끄고 종료한다.
"""
import time
from pathlib import Path
import numpy as np

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(r"C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower")
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}

cfg = BiSOFollowerConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOFollowerConfig(port="COM15", disable_torque_on_disconnect=True),
    right_arm_config=SOFollowerConfig(port="COM16", disable_torque_on_disconnect=True),
)
robot = BiSOFollower(cfg)
robot.connect()  # 토크 ON — 팔을 현재 자세로 고정 (움직임 명령은 안 보냄)
print("connected (torque holds current pose). 5초간 고주파 읽기 시작 — 팔 건드리지 마세요.\n")

keys = [f"{side}_{j}" for side in ("left", "right") for j in JOINT_ORDER]
samples = {k: [] for k in keys}
N = 150
try:
    for _ in range(N):
        obs = robot.get_observation()
        for k in keys:
            pk = f"{k}.pos"
            if pk in obs:
                samples[k].append(obs[pk])
        time.sleep(0.03)  # ~30Hz
finally:
    robot.disconnect()  # 토크 OFF

print("RESULT_START")
print(f"{'motor':22s} {'id':>3s} {'N':>4s} {'mean':>9s} {'std':>7s} {'ptp(max-min)':>13s}  flag")
for side in ("left", "right"):
    for j in JOINT_ORDER:
        k = f"{side}_{j}"
        vals = np.array(samples[k], dtype=float)
        if len(vals) == 0:
            print(f"{k:22s} {IDS[j]:>3d}  NO DATA")
            continue
        std = vals.std()
        ptp = vals.max() - vals.min()
        flag = "<== NOISY" if (std > 0.5 or ptp > 2.0) else ""
        print(f"{k:22s} {IDS[j]:>3d} {len(vals):>4d} {vals.mean():9.2f} {std:7.3f} {ptp:13.2f}  {flag}")
print("RESULT_END")
print("\n팔이 가만히 있었는데 특정 모터만 std/ptp가 크면 그 모터의 연결/통신이 불안정한 것.")
