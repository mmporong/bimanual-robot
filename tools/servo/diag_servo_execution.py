"""진단: 정책 없이, 직접 만든 '부드러운 움직임'을 서보에 줘서 떠는지 본다.

실제 롤아웃과 같은 경로(get_observation -> send_action, max_relative_target 포함,
30Hz)로 양팔 shoulder_pan만 ±8도, 6초에 걸쳐 아주 천천히 사인파로 움직인다.
다른 관절은 현재 자세로 고정. 매 틱 '목표(goal)'와 '실제(present)'를 기록한다.

판정:
- 이 부드러운 램프인데도 팔이 덜덜 떨린다 -> send_action/서보 실행 경로 문제
  (정책과 무관한 하드웨어/제어 문제로 확정)
- 부드럽게 잘 움직인다 -> 실행 경로는 정상 -> 문제는 실제 롤아웃의 정책 출력 쪽

안전장치: 진폭 8도로 작게, 속도 느리게. max_relative_target=20으로 급동작 차단.
Ctrl+C로 즉시 중단 가능(중단 시 토크 끔).

사용법: python tools/servo/diag_servo_execution.py   (저장소 루트에서)
"""
import math
import time
from pathlib import Path
import numpy as np

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(r"C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower")

cfg = BiSOFollowerConfig(
    id="arms",
    calibration_dir=CALIB_DIR,
    left_arm_config=SOFollowerConfig(port="COM15", max_relative_target=20.0, disable_torque_on_disconnect=True),
    right_arm_config=SOFollowerConfig(port="COM16", max_relative_target=20.0, disable_torque_on_disconnect=True),
)
robot = BiSOFollower(cfg)
robot.connect()
print("connected. 2초 정지 후, 양팔 shoulder_pan을 ±8도 천천히 움직입니다. 팔 주변을 비워두세요.\n")

# 시작 자세 캡처
obs0 = robot.get_observation()
base = {k: v for k, v in obs0.items() if k.endswith(".pos")}

AMP = 8.0       # 진폭(도)
PERIOD = 6.0    # 한 주기(초)
HOLD = 2.0      # 시작 전 정지(초)
MOVE = 12.0     # 움직이는 시간(초) = 2주기
FPS = 30.0
dt = 1.0 / FPS

pan_keys = [k for k in base if k.endswith("shoulder_pan.pos")]
log = []
t0 = time.perf_counter()
try:
    # 정지 구간: 시작 자세 유지
    while time.perf_counter() - t0 < HOLD:
        robot.send_action(dict(base))
        time.sleep(dt)
    # 움직임 구간: shoulder_pan만 사인파
    tm0 = time.perf_counter()
    while True:
        t = time.perf_counter() - tm0
        if t >= MOVE:
            break
        offset = AMP * math.sin(2 * math.pi * t / PERIOD)
        goal = dict(base)
        for k in pan_keys:
            goal[k] = base[k] + offset
        robot.send_action(goal)
        obs = robot.get_observation()
        row = {"t": t, "goal": offset}
        for k in pan_keys:
            row[k] = obs[k] - base[k]  # 시작 대비 실제 변위
        log.append(row)
        time.sleep(dt)
finally:
    # 시작 자세로 부드럽게 복귀 후 토크 끔
    try:
        cur = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
        for i in range(1, 31):
            a = i / 30
            robot.send_action({k: cur[k] * (1 - a) + base[k] * a for k in base})
            time.sleep(dt)
    except Exception:
        pass
    robot.disconnect()

print("RESULT_START")
print("shoulder_pan 추종: goal=명령 변위, L/R=실제 변위(도). 차이가 들쭉날쭉하면 추종 불안정.\n")
print(f"{'t(s)':>6} {'goal':>7}", end="")
for k in pan_keys:
    tag = "L" if "left" in k or "arms_left" in k else "R"
    print(f" {tag+'_pan':>8}", end="")
print(f" {'trackErr':>9}")
errs = []
for r in log[::6]:  # 0.2초 간격으로 출력
    print(f"{r['t']:6.2f} {r['goal']:7.2f}", end="")
    step_errs = []
    for k in pan_keys:
        print(f" {r[k]:8.2f}", end="")
        step_errs.append(abs(r[k] - r['goal']))
    print(f" {max(step_errs):9.2f}")
for r in log:
    for k in pan_keys:
        errs.append(abs(r[k] - r['goal']))
errs = np.array(errs)
print(f"\n추종 오차: mean={errs.mean():.2f}deg  max={errs.max():.2f}deg  std={errs.std():.2f}deg")
print("RESULT_END")
print("\n팔이 눈으로 봤을 때 부드러웠는지 / 덜덜 떨었는지도 같이 알려주세요.")
