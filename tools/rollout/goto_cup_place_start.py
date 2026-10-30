"""양팔을 '컵 배치(cup-place) 학습 데이터의 실제 시작 자세'(ep0)로
천천히 정밀 이동시킨 뒤, 그 자세를 토크로 고정한 채 종료한다.

이후 바로 cup-place 체크포인트로 롤아웃을 돌리면, 롤아웃이 이 (올바른)
시작 자세를 캡처해서 시작한다. goto_train_start.py는 물따르기 task용
시작 자세라 이 task에는 안 맞음 - 이 스크립트를 대신 써야 한다.

안전: max_relative_target=20으로 급동작 차단, 4초에 걸쳐 부드럽게 보간 이동.
      Ctrl+C로 즉시 중단(중단 시 토크 끔). 이동 중 팔 주변을 비워둘 것.

사용법: python tools/rollout/goto_cup_place_start.py   (저장소 루트에서)
끝나면 토크를 켠 채로 둔다 → 이어서 cup-place 체크포인트로 롤아웃 실행.
"""
import time
from pathlib import Path
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(r"C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower")

# cup-place-position-jitter-gravel-tabletop-v1 ep0 시작 자세 (학습 데이터, on-manifold)
TARGET = {
    "left_shoulder_pan": -0.18, "left_shoulder_lift": 3.56, "left_elbow_flex": 95.69,
    "left_wrist_flex": -57.14, "left_wrist_roll": -90.68, "left_gripper": 0.22,
    "right_shoulder_pan": 0.00, "right_shoulder_lift": -0.40, "right_elbow_flex": 95.82,
    "right_wrist_flex": -80.26, "right_wrist_roll": 1.10, "right_gripper": 14.34,
}

cfg = BiSOFollowerConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOFollowerConfig(port="COM15", max_relative_target=20.0, disable_torque_on_disconnect=False),
    right_arm_config=SOFollowerConfig(port="COM16", max_relative_target=20.0, disable_torque_on_disconnect=False),
)
robot = BiSOFollower(cfg)
robot.connect()
print("connected. 현재 자세 -> cup-place 학습 시작 자세(ep0)로 4초간 천천히 이동합니다. 팔 주변 비우세요.\n")

cur = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
tgt = {f"{m}.pos": v for m, v in TARGET.items() if f"{m}.pos" in cur}

print("관절별 이동량(도):")
for k in tgt:
    print(f"  {k.removesuffix('.pos'):22s} {cur[k]:7.2f} -> {tgt[k]:7.2f}  (Δ{tgt[k]-cur[k]:+.1f})")
print()

STEPS = 120  # 4초 @ 30Hz
try:
    for i in range(1, STEPS + 1):
        a = i / STEPS
        goal = {k: cur[k] * (1 - a) + tgt[k] * a for k in tgt}
        robot.send_action(goal)
        time.sleep(1 / 30)
    time.sleep(0.5)
    final = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
    print("도착 자세:")
    for k in tgt:
        print(f"  {k.removesuffix('.pos'):22s} {final[k]:7.2f}  (목표 {tgt[k]:7.2f})")
finally:
    robot.disconnect()

print("\n*** 토크 유지됨. 팔이 cup-place 학습 시작 자세로 고정되어 있습니다. ***")
print("*** 이제 바로 cup-place 체크포인트로 롤아웃을 실행하세요. ***")
