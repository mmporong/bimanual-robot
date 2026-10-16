"""양팔을 '학습 데이터의 실제 시작 자세'로 정밀 이동시킨 뒤, 토크로 고정한 채 종료한다.

왜 필요한가
-----------
SmolVLA 같은 모방학습 정책은 학습 데이터 분포 안의 관측에서만 제대로 동작한다.
롤아웃을 '눈대중으로 맞춘 시작 자세'에서 시작하면, 관절 각도가 학습 분포에서
벗어나(특히 눈으로는 구분하기 어려운 shoulder_lift·wrist_flex 같은 관절) 정책이
무엇을 해야 할지 몰라 **제자리에서 떨기만 하는** 실패가 난다. 2026-10-16에 실제로
이 원인으로 종일 헤맸다. 상세:
docs/experiments/20261016_SmolVLA_제자리떨림_시작자세_OOD_원인규명.md

이 도구로 매 롤아웃 전에 시작 자세를 '학습 분포 정중앙의 실제 에피소드 자세'로
정확히 고정하면 그 실패를 원천 차단한다. 롤아웃의 `return_to_initial_position`
(기본 켜짐)이 에피소드 종료 후 이 자세로 자동 복귀하므로, 시작·복귀 모두 학습
분포 안에서 이뤄진다.

TARGET 값의 출처
---------------
`jun-seo/water-pour-position-jitter-v1`의 60개 에피소드 시작 자세 중 전체 평균에
가장 가까운 실제 에피소드(ep32)의 첫 프레임 관절 각도(도). **이 값은 이 데이터셋과
현재 calibration/bi_follower 기준이다.** 데이터셋이나 캘리브레이션이 바뀌면 아래
한 줄로 다시 뽑는다:

    python -c "import numpy as np; from lerobot.datasets.lerobot_dataset import LeRobotDataset; \
d=LeRobotDataset('jun-seo/water-pour-position-jitter-v1'); fi=d.meta.episodes['dataset_from_index']; \
a=np.array([[float(x) for x in d.hf_dataset[int(f)]['observation.state']] for f in fi]); \
b=int(np.abs(a-a.mean(0)).sum(1).argmin()); print(b, a[b])"

안전
----
- max_relative_target=20으로 급동작을 차단하고, 4초에 걸쳐 부드럽게 보간 이동한다.
- Ctrl+C로 즉시 중단(중단 시 토크를 끈다). 이동 중 팔 주변을 비워둘 것.
- 종료 시 토크를 **켠 채로** 둔다(disable_torque_on_disconnect=False) → 이어서
  롤아웃이 이 자세를 그대로 캡처해 시작한다.

사용법
------
    python tools/rollout/goto_train_start.py
    # "토크 유지됨 ... 고정" 메시지 확인 후 손대지 말고 바로 롤아웃 실행
"""
from pathlib import Path
import time

from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig

CALIB_DIR = Path(__file__).resolve().parents[2] / "calibration" / "bi_follower"

# ep32 시작 자세 (학습 데이터, on-manifold). 출처·재생성 방법은 모듈 docstring 참고.
TARGET = {
    "left_shoulder_pan": 1.49, "left_shoulder_lift": 3.12, "left_elbow_flex": 94.73,
    "left_wrist_flex": -49.14, "left_wrist_roll": -97.45, "left_gripper": 0.29,
    "right_shoulder_pan": 2.90, "right_shoulder_lift": -4.18, "right_elbow_flex": 95.56,
    "right_wrist_flex": -75.08, "right_wrist_roll": 4.00, "right_gripper": 2.19,
}

MOVE_SECONDS = 4.0
FPS = 30.0


def main() -> int:
    cfg = BiSOFollowerConfig(
        id="arms",
        calibration_dir=CALIB_DIR,
        left_arm_config=SOFollowerConfig(
            port="COM15", max_relative_target=20.0, disable_torque_on_disconnect=False
        ),
        right_arm_config=SOFollowerConfig(
            port="COM16", max_relative_target=20.0, disable_torque_on_disconnect=False
        ),
    )
    robot = BiSOFollower(cfg)
    robot.connect()
    print("connected. 현재 자세 -> 학습 시작 자세(ep32)로 "
          f"{MOVE_SECONDS:.0f}초간 천천히 이동합니다. 팔 주변 비우세요.\n")

    cur = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
    tgt = {f"{m}.pos": v for m, v in TARGET.items() if f"{m}.pos" in cur}

    print("관절별 이동량(도):")
    for k in tgt:
        print(f"  {k.removesuffix('.pos'):22s} {cur[k]:7.2f} -> {tgt[k]:7.2f}  (Δ{tgt[k] - cur[k]:+.1f})")
    print()

    steps = max(int(MOVE_SECONDS * FPS), 1)
    try:
        for i in range(1, steps + 1):
            a = i / steps
            robot.send_action({k: cur[k] * (1 - a) + tgt[k] * a for k in tgt})
            time.sleep(1 / FPS)
        time.sleep(0.5)
        final = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
        print("도착 자세:")
        for k in tgt:
            print(f"  {k.removesuffix('.pos'):22s} {final[k]:7.2f}  (목표 {tgt[k]:7.2f})")
    finally:
        # disable_torque_on_disconnect=False -> 토크 유지한 채 포트만 닫는다.
        robot.disconnect()

    print("\n*** 토크 유지됨. 팔이 학습 시작 자세로 고정되어 있습니다. ***")
    print("*** 손대지 말고 바로 롤아웃을 실행하세요 (이 자세를 캡처해 시작합니다). ***")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
