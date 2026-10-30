"""물따르기 롤아웃 + 붓기 구간만 코드가 맡는 훅 (IL + 스크립트 조합).

정책(SmolVLA)이 병을 집고 기울이기 시작하는 것까지는 그대로 맡기고, 오른 손목 roll이
붓기 시작 각도를 넘는 순간부터 코드가 제어권을 가져가 아래를 고정 절차로 실행한다.

    WATCH  : 정책이 제어. roll >= trigger_roll 이고 오른 그리퍼가 병을 쥔 상태면 테이크오버
    TILT   : roll을 tilt_target까지 일정 속도로 기울임 (pan은 pan_follow만큼 비례 이동)
    HOLD   : hold_s 동안 유지 (쏟아지는 시간)
    RETURN : roll을 return_roll까지 되돌림
    GUARD  : 정책에 제어권 반환. 이후 roll 명령을 guard_roll 이하로 잘라 재붓기를 막는다

테이크오버 중 나머지 관절은 정책이 마지막으로 보낸 명령값에 고정한다. 그리퍼는 현재
위치가 아니라 명령값으로 고정해야 쥐는 힘이 유지된다(현재 위치로 고정하면 힘이 빠짐).
기울이기/되돌리기 중 명령과 실제 roll 차이가 stall_err를 stall_s 넘게 유지되면 스톨로
보고, 기울이는 중이면 즉시 되돌리기로, 되돌리는 중이면 현재 자세 유지(FAULT)로 멈춘다.
FAULT에서는 정책에 제어권을 돌려주지 않으니 Ctrl+C로 끝낸다.

기본값의 근거(water-pour-...-handoff-v2, 40 에피소드):
  붓기 전 roll 최대 74.9 / 쥘 때 그리퍼 29.7~44.1(빈손 ~1.7) / 정점 155.3~163.8 /
  75->정점 평균 7.0s / 정점->30 이하 복귀 평균 1.8s / 붓는 동안 pan 평균 -11.2 /
  복귀 후 roll 최대 28.2

사용법: 평소 lerobot-rollout 플래그를 그대로 붙인다. --strategy.type=base 전용(RTC 미지원).
카메라 인덱스 자리에 {left} {right} {depth}를 쓰면 실행 직전에 자동으로 찾아 채운다.

    python tools/rollout/rollout_pour_hook.py --strategy.type=base --policy.path=... (이하 동일)
    python tools/rollout/rollout_pour_hook.py --hook.hold_s=5 ...   # 훅 값 조정
    python tools/rollout/rollout_pour_hook.py --hook.disable ...    # 훅 끄고 원래 롤아웃과 동일
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, fields
from pathlib import Path

logger = logging.getLogger("pour_hook")

ROLL = "right_wrist_roll.pos"
PAN = "right_shoulder_pan.pos"
GRIP = "right_gripper.pos"


@dataclass
class HookParams:
    trigger_roll: float = 75.0
    grip_min: float = 20.0
    tilt_target: float = 160.0
    tilt_speed: float = 12.0  # deg/s
    hold_s: float = 3.0
    return_roll: float = 20.0
    return_speed: float = 50.0  # deg/s
    pan_follow: float = -11.0
    guard_roll: float = 45.0
    stall_err: float = 15.0
    stall_s: float = 1.0


class PourHook:
    WATCH, TILT, HOLD, RETURN, GUARD, FAULT = "WATCH", "TILT", "HOLD", "RETURN", "GUARD", "FAULT"

    def __init__(self, params: HookParams):
        self.p = params
        self.state = self.WATCH
        self.last_policy_action: dict[str, float] | None = None
        self.events: list[tuple[float, str, str]] = []
        self._handback = False
        self._base: dict[str, float] = {}
        self._t0 = self._roll0 = self._pan_end = self._roll_cmd = self._ret_from = 0.0
        self._stall_since: float | None = None
        self._t_start: float | None = None

    def _log(self, now: float, msg: str) -> None:
        if self._t_start is None:
            self._t_start = now
        self.events.append((now - self._t_start, self.state, msg))
        logger.info("[pour_hook] %s: %s", self.state, msg)

    def _enter(self, now: float, state: str, msg: str) -> None:
        self.state = state
        self._stall_since = None
        self._log(now, msg)

    def _stalled(self, now: float, roll: float) -> bool:
        if abs(self._roll_cmd - roll) <= self.p.stall_err:
            self._stall_since = None
            return False
        if self._stall_since is None:
            self._stall_since = now
        return now - self._stall_since >= self.p.stall_s

    def _action(self, pan: float) -> dict[str, float]:
        a = dict(self._base)
        a[ROLL] = self._roll_cmd
        a[PAN] = pan
        return a

    def update(self, now: float, obs: dict) -> dict[str, float] | None:
        """이번 틱에 훅이 보낼 액션을 돌려준다. None이면 정책이 제어한다."""
        if self._t_start is None:
            self._t_start = now
        p = self.p
        roll = float(obs[ROLL])

        if self.state in (self.WATCH, self.GUARD):
            if self.state == self.GUARD or roll < p.trigger_roll or float(obs[GRIP]) < p.grip_min:
                return None
            src = self.last_policy_action or {}
            self._base = {k: float(src.get(k, obs[k])) for k in (src or obs) if k.endswith(".pos")}
            self._t0, self._roll0, self._roll_cmd = now, roll, roll
            self._pan_end = self._base[PAN] + p.pan_follow
            self._enter(now, self.TILT, f"테이크오버 roll={roll:.1f} grip={float(obs[GRIP]):.1f}")

        if self.state == self.TILT:
            self._roll_cmd = min(p.tilt_target, self._roll0 + p.tilt_speed * (now - self._t0))
            span = p.tilt_target - self._roll0
            frac = 1.0 if span <= 0 else (self._roll_cmd - self._roll0) / span
            pan = self._base[PAN] + p.pan_follow * frac
            if self._stalled(now, roll):
                self._roll_cmd = roll
                self._t0 = now
                self._enter(now, self.RETURN, f"기울이기 스톨(명령-실제 차이 지속) roll={roll:.1f} -> 되돌리기")
                self._ret_from = roll
                self._pan_end = pan
                return self._action(pan)
            if self._roll_cmd >= p.tilt_target and roll >= p.tilt_target - p.stall_err:
                self._t0 = now
                self._pan_end = pan
                self._enter(now, self.HOLD, f"목표 도달 roll={roll:.1f}, {p.hold_s:.1f}s 유지")
            return self._action(pan)

        if self.state == self.HOLD:
            if now - self._t0 >= p.hold_s:
                self._t0, self._ret_from = now, self._roll_cmd
                self._enter(now, self.RETURN, f"유지 끝 roll={roll:.1f} -> 되돌리기")
            return self._action(self._pan_end)

        if self.state == self.RETURN:
            self._roll_cmd = max(p.return_roll, self._ret_from - p.return_speed * (now - self._t0))
            if self._stalled(now, roll):
                self._roll_cmd = roll
                self._enter(now, self.FAULT, f"되돌리기 스톨 roll={roll:.1f} -> 현재 자세 유지, Ctrl+C로 종료")
                return self._action(self._pan_end)
            if self._roll_cmd <= p.return_roll and roll <= p.return_roll + p.stall_err:
                self._handback = True
                self._enter(now, self.GUARD, f"복귀 완료 roll={roll:.1f} -> 정책에 반환 (roll<={p.guard_roll:.0f} 제한)")
            return self._action(self._pan_end)

        return self._action(self._pan_end)  # FAULT: 마지막 명령 유지

    def filter_policy_action(self, action: dict[str, float]) -> dict[str, float]:
        if self.state == self.GUARD and action.get(ROLL, 0.0) > self.p.guard_roll:
            action = dict(action)
            action[ROLL] = self.p.guard_roll
        return action

    def consume_handback(self) -> bool:
        h, self._handback = self._handback, False
        return h


def install(hook: PourHook) -> None:
    """BaseStrategy가 쓰는 send_next_action을 훅이 끼어든 버전으로 바꾼다."""
    from lerobot.rollout.strategies import base as base_strategy
    from lerobot.utils.constants import OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    def send_next_action(obs_processed, obs_raw, ctx, interpolator):
        hook_action = hook.update(time.perf_counter(), obs_raw)
        if hook_action is not None:
            processed = ctx.processors.robot_action_processor((hook_action, obs_raw))
            ctx.hardware.robot_wrapper.send_action(processed)
            if hook.consume_handback():
                # 테이크오버 전에 쌓인 액션 청크를 버려야 기울이던 동작이 재생되지 않는다
                ctx.policy.inference.reset()
                interpolator.reset()
            return hook_action

        if interpolator.needs_new_action():
            obs_frame = build_dataset_frame(ctx.data.dataset_features, obs_processed, prefix=OBS_STR)
            action_tensor = ctx.policy.inference.get_action(obs_frame)
            if action_tensor is not None:
                interpolator.add(action_tensor.cpu())
        interp = interpolator.get()
        if interp is None:
            return None
        keys = ctx.data.ordered_action_keys
        action = hook.filter_policy_action({k: interp[i].item() for i, k in enumerate(keys)})
        hook.last_policy_action = action
        processed = ctx.processors.robot_action_processor((action, obs_raw))
        ctx.hardware.robot_wrapper.send_action(processed)
        return action

    base_strategy.send_next_action = send_next_action


def parse_args(argv: list[str]) -> tuple[HookParams, bool, list[str]]:
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--hook.disable", dest="disable", action="store_true")
    for f in fields(HookParams):
        ap.add_argument(f"--hook.{f.name}", dest=f.name, type=float, default=f.default)
    ns, rest = ap.parse_known_args(argv)
    params = HookParams(**{f.name: getattr(ns, f.name) for f in fields(HookParams)})
    return params, ns.disable, rest


def fill_camera_indices(args: list[str]) -> list[str]:
    if not any(tok in a for a in args for tok in ("{left}", "{right}", "{depth}")):
        return args
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camera"))
    from resolve_camera_indices import resolve

    r = resolve()
    print(f"[camera resolve] left={r['left']} right={r['right']} depth={r['depth']}")
    return [a.replace("{left}", str(r["left"])).replace("{right}", str(r["right"])).replace("{depth}", str(r["depth"])) for a in args]


def main() -> None:
    params, disabled, rest = parse_args(sys.argv[1:])
    rest = fill_camera_indices(rest)
    if "--strategy.type=base" not in rest:
        sys.exit("--strategy.type=base 로만 동작한다 (훅이 BaseStrategy의 액션 전송을 가로챈다).")

    hook = None
    if disabled:
        print("[pour_hook] 비활성 - 원래 롤아웃과 동일하게 실행")
    else:
        hook = PourHook(params)
        install(hook)
        print(f"[pour_hook] 활성: {params}")

    sys.argv = ["lerobot-rollout", *rest]
    from lerobot.scripts.lerobot_rollout import main as rollout_main

    try:
        rollout_main()
    finally:
        if hook is not None:
            print(f"\n[pour_hook] 최종 상태: {hook.state}")
            for t, state, msg in hook.events:
                print(f"  t={t:6.1f}s [{state}] {msg}")


if __name__ == "__main__":
    main()
