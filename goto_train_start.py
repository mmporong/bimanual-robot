"""양팔을 '학습 데이터의 실제 시작 자세'(ep32, 가장 중앙에 가까운 에피소드)로
천천히 정밀 이동시킨 뒤, 그 자세를 토크로 고정한 채 종료한다.

이후 바로 롤아웃을 돌리면, 롤아웃이 이 (올바른) 시작 자세를 캡처해서 시작한다.
눈대중 리셋이 학습 분포에서 벗어나 정책이 떠는 문제를 교정하기 위함.

안전: max_relative_target=20으로 급동작 차단, 4초에 걸쳐 부드럽게 보간 이동.
      Ctrl+C로 즉시 중단(중단 시 토크 끔). 이동 중 팔 주변을 비워둘 것.

사용법: python goto_train_start.py
끝나면 토크를 켠 채로 둔다 → 이어서 롤아웃 실행.
"""
import time
from pathlib import Path
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(r"C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower")

# ep32 시작 자세 (학습 데이터, on-manifold)
TARGET = {
    "left_shoulder_pan": 1.49, "left_shoulder_lift": 3.12, "left_elbow_flex": 94.73,
    "left_wrist_flex": -49.14, "left_wrist_roll": -97.45, "left_gripper": 0.29,
    "right_shoulder_pan": 2.90, "right_shoulder_lift": -4.18, "right_elbow_flex": 95.56,
    "right_wrist_flex": -75.08, "right_wrist_roll": 4.00, "right_gripper": 2.19,
}

cfg = BiSOFollowerConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOFollowerConfig(port="COM15", max_relative_target=20.0, disable_torque_on_disconnect=False),
    right_arm_config=SOFollowerConfig(port="COM16", max_relative_target=20.0, disable_torque_on_disconnect=False),
)
robot = BiSOFollower(cfg)
robot.connect()
print("connected. 현재 자세 -> 학습 시작 자세(ep32)로 4초간 천천히 이동합니다. 팔 주변 비우세요.\n")

cur = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
tgt = {f"{m}.pos": v for m, v in TARGET.items() if f"{m}.pos" in cur}

# 현재 vs 목표 차이 출력
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
    # disable_torque_on_disconnect=False -> 토크 유지한 채 포트만 닫음
    robot.disconnect()

print("\n*** 토크 유지됨. 팔이 학습 시작 자세로 고정되어 있습니다. ***")
print("*** 이제 바로 롤아웃을 실행하세요 (이 자세를 캡처해서 시작합니다). ***")
