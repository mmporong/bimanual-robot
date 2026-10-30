"""실제 lerobot-rollout을 그대로 돌리되, 매 틱마다 계측 로그를 남긴다.

기록:
- 각 카메라 이미지의 평균 밝기 (프레임이 라이브로 변하는지 = frozen 탐지)
- 정책이 send_action에 넘긴 명령(action, 클램프 전) — 몇몇 관절
- 실제로 서보에 쓴 값(result, 클램프 후)
- 현재 위치(present)

판정 포인트:
- 카메라 평균이 매 틱 거의 안 변하면 그 카메라는 frozen(정지 프레임)
- action(정책 명령)이 제자리에서 진동하면 정책이 떨리는 출력을 내는 것
- action은 부드러운데 팔만 떨면 실행 문제(이미 정상 확인됨)

사용법: python tools/rollout/instrumented_rollout.py   (저장소 루트에서)
Ctrl+C로 중단 가능. 끝나면 로그가 scratchpad에 저장된다.
"""
import sys
import time
import json

import numpy as np

LOG_PATH = (r"C:\Users\USER\AppData\Local\Temp\claude"
            r"\C--Users-USER-Desktop-------------bimanual-robot-git"
            r"\491bc734-c842-437e-9b6b-c7141cb2c7dd\scratchpad\instrumented_rollout_log.jsonl")

# 로그 대상 관절 (왼팔 전부 + 오른 gripper)
WATCH = ["left_shoulder_lift", "left_elbow_flex", "left_wrist_flex", "left_wrist_roll",
         "left_gripper", "right_gripper"]

import cv2
FRAME_DIR = (r"C:\Users\USER\AppData\Local\Temp\claude"
             r"\C--Users-USER-Desktop-------------bimanual-robot-git"
             r"\491bc734-c842-437e-9b6b-c7141cb2c7dd\scratchpad\rollout_frames")
import os
os.makedirs(FRAME_DIR, exist_ok=True)

logf = open(LOG_PATH, "w", encoding="utf-8")
_tick = {"n": 0}
_saved = set()

# --- monkeypatch get_observation: 카메라 밝기 + state 기록 ---
from lerobot.robots.so_follower import so_follower

_orig_get_obs = so_follower.SOFollower.get_observation
_orig_send = so_follower.SOFollower.send_action


def _patched_get_obs(self):
    obs = _orig_get_obs(self)
    side = getattr(self, "id", "?")
    cam_means = {}
    for k, v in obs.items():
        if isinstance(v, np.ndarray) and v.ndim == 3:
            cam_means[k] = round(float(v.mean()), 3)
            # 대략 2초마다(60틱) 실제 프레임 저장 — 정책이 본 그대로
            tag = f"{side}:{k}"
            bucket = _tick["n"] // 60
            skey = (tag, bucket)
            if skey not in _saved:
                _saved.add(skey)
                img = v
                if img.ndim == 3 and img.shape[2] == 3:
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    ok, b = cv2.imencode(".jpg", bgr)
                    if ok:
                        fn = f"{FRAME_DIR}\\t{bucket*2:02d}s_{side}_{k}.jpg"
                        with open(fn, "wb") as f:
                            f.write(b.tobytes())
    rec = {"t": round(time.perf_counter(), 4), "ev": "obs", "side": str(side), "cams": cam_means}
    logf.write(json.dumps(rec) + "\n")
    return obs


def _patched_send(self, action):
    present = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
    result = _orig_send(self, action)
    side = getattr(self, "id", "?")
    # action/result 키는 '<motor>.pos', present 키는 '<motor>'
    rec = {"t": round(time.perf_counter(), 4), "ev": "act", "side": str(side)}
    for key, val in action.items():
        m = key.removesuffix(".pos")
        full = f"{side}_{m}" if not str(side).endswith(m) else m
        # 관심 관절만
        for w in WATCH:
            if w.endswith(m) and (w.startswith(str(side).split("_")[0]) or True):
                rec.setdefault("cmd", {})[f"{side}:{m}"] = round(float(val), 2)
                if m in present:
                    rec.setdefault("now", {})[f"{side}:{m}"] = round(float(present[m]), 2)
                if key in result:
                    rec.setdefault("sent", {})[f"{side}:{m}"] = round(float(result[key]), 2)
    logf.write(json.dumps(rec) + "\n")
    logf.flush()
    _tick["n"] += 1
    return result


so_follower.SOFollower.get_observation = _patched_get_obs
so_follower.SOFollower.send_action = _patched_send

# --- 실제 롤아웃 호출 (CLI와 동일 플래그) ---
sys.argv = [
    "lerobot-rollout",
    "--strategy.type=base",
    r"--policy.path=C:\Users\USER\Desktop\so101-mobile-manipulation-main\lerobot\outputs\water_pour_jitter_60_smolvla_v1\checkpoints\009000\pretrained_model",
    "--policy.device=cuda",
    "--robot.type=bi_so_follower",
    "--robot.id=arms",
    r"--robot.calibration_dir=C:\Users\USER\Desktop\프로젝트\최종프로젝트\bimanual-robot-git\calibration\bi_follower",
    "--robot.left_arm_config.port=COM15",
    "--robot.left_arm_config.gripper_protection=true",
    "--robot.left_arm_config.disable_torque_on_disconnect=false",
    "--robot.left_arm_config.max_relative_target=20",
    "--robot.left_arm_config.cameras={ wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }",
    "--robot.right_arm_config.port=COM16",
    "--robot.right_arm_config.disable_torque_on_disconnect=false",
    "--robot.right_arm_config.max_relative_target=20",
    "--robot.right_arm_config.cameras={ wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30} }",
    "--robot.cameras={ top: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30, mirror: true} }",
    "--task=Pick up the cup with the left arm, pick up the water bottle with the right arm, pour water from the bottle into the cup, return the bottle to its place, and load the cup onto the robot chassis.",
    "--duration=25",
]

from lerobot.scripts.lerobot_rollout import rollout

try:
    rollout()
finally:
    logf.close()
    print("\nLOG SAVED:", LOG_PATH)
