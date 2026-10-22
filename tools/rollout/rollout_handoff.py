"""핸드오프(물따르기 -> 컵배치) 롤아웃 래퍼: 그리퍼 과부하 예방 + 끝 자세 선택 + 붓기 훅.

lerobot-rollout을 그대로 실행하되 세 가지를 덧붙인다. lerobot 원본 코드는 고치지 않는다.

1) 실행 전 그리퍼 풀기 (매 실행 자동)
   이전 롤아웃이 토크를 켠 채 끝나 그리퍼가 물체를 계속 밀고 있으면 과부하 보호가 걸려
   다음 연결이 "Missing motor ID 6"으로 실패한다. 연결 전에 양팔 그리퍼(ID 6)를 읽어,
   목표가 실제 위치보다 grip.margin 틱 넘게 밀고 있으면 목표를 "실제 위치 + margin"으로
   당긴다. 토크는 켠 채라 손가락 위치는 그대로다. 그래도 과부하가 1초 안에 안 풀리면
   그 그리퍼만 토크를 껐다 바로 켠다(수 ms).

2) 끝 자세 (--end.pose)
   final      : 마지막 자세 그대로 토크 유지 (기본). 끝나기 직전에 1)과 같은 방식으로
                그리퍼 미는 힘을 줄여, 다음 실행까지 기다리는 동안 과부하가 쌓이지 않게 한다.
   pour_start : 물따르기 v2 시작 자세(시연 40개 평균)로 end.move_s초에 걸쳐 이동 후 유지.
   initial    : lerobot 기본(이번 실행 시작 자세로 복귀).

3) 붓기 훅 (--pour_hook): tools/rollout/rollout_pour_hook.py의 훅을 함께 켠다.
   훅 값은 --hook.* 로 조정한다.

4) 작업 끝 자동 감지 (--end.auto) - 끝나면 duration을 기다리지 않고 바로 2)로 넘어간다.
   pour  : 오른손이 병을 쥐었다 놓은 뒤, 왼팔(그리퍼 제외 5관절)이 상판 적재 자세
           (컵배치 v2 시작 자세 평균) ±end.tol도 안에 end.settle_s초 머물면 끝.
           v2 시연 40개 모두 병을 놓은 뒤에만 이 자세에 들어왔다(이전 0프레임).
   place : 그리퍼 값은 클수록 열림이다(쥔 컵 10~24, 놓을 때 62~96, 빈손으로 닫으면 ~1).
           컵 놓음(측정값과 정책 명령이 모두 40 초과로 0.3초 유지) -> 놓은 자리에서 왼팔이
           10도 이상 빠짐 -> 그리퍼가 10 미만으로 다시 닫히고(명령도) 왼팔 5관절이 물따르기
           시작 자세 ±end.place_tol도 안에 end.settle_s초 머물면 끝.
           v2 시연 순서: 열기 -> 팔 빼며 닫기(놓은 자리에서 74~86도 이동) -> 약 1초 뒤 시작
           자세 도착 -> 7~10초 정지. 테이블 접촉으로 측정값이 조금 출렁여도 40까지 안 가고,
           컵을 다시 쥐면 컵에 막혀 17~22에 머물러 "닫힘"으로 보지 않는다.
           v2 시연 40개 마지막 자세는 이 자세와 관절별 평균 0.1~2.8도 차이였다.
           ±15도면 wrist_roll이 18도 벗어나 끝난 1개(ep6)를 놓쳐 기본값을 20도로 둔다
           (컵 놓음이 먼저 감지돼야 판정하므로 놓기 전에 자세가 비슷해도 끝나지 않는다).
   duration은 최대 시간(안전 상한)으로만 남는다.

사용법 (평소 lerobot-rollout 플래그를 그대로 붙인다, --strategy.type=base 전용):
    python tools/rollout/rollout_handoff.py --end.pose=final ...물따르기 플래그...
    python tools/rollout/rollout_handoff.py --end.pose=pour_start ...컵배치 플래그...
    python tools/rollout/rollout_handoff.py --pour_hook --end.pose=final ...물따르기 플래그...
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rollout_pour_hook import PourHook, fill_camera_indices, install, parse_args as parse_hook_args  # noqa: E402

logger = logging.getLogger("rollout_handoff")

GRIPPER_ID = 6
OVERLOAD_BIT = 32

# water-pour-position-jitter-gravel-tabletop-handoff-v2 40개 에피소드 시작 state 평균 (관절별 std 0.1~4.6)
POUR_START = {
    "left_shoulder_pan.pos": 0.63, "left_shoulder_lift.pos": -0.47, "left_elbow_flex.pos": 95.61,
    "left_wrist_flex.pos": -53.77, "left_wrist_roll.pos": -90.68, "left_gripper.pos": 1.01,
    "right_shoulder_pan.pos": -2.26, "right_shoulder_lift.pos": -1.35, "right_elbow_flex.pos": 95.58,
    "right_wrist_flex.pos": -80.33, "right_wrist_roll.pos": 0.74, "right_gripper.pos": 1.65,
}
END_POSES = ("final", "pour_start", "initial")

# cup-place-position-jitter-gravel-tabletop-handoff-v2 40개 시작 state 평균 = 물따르기 끝(상판 적재) 자세
CHASSIS_HOLD = {
    "left_shoulder_pan.pos": 66.66, "left_shoulder_lift.pos": 29.00, "left_elbow_flex.pos": 81.53,
    "left_wrist_flex.pos": -95.47, "left_wrist_roll.pos": -94.54,
}
LEFT_5 = tuple(CHASSIS_HOLD)
GRIP_OPEN = 40.0
GRIP_CLOSED = 10.0


class DoneDetector:
    def __init__(self, mode: str, tol: float, settle_s: float, place_tol: float,
                 release_s: float = 0.3, retract_deg: float = 10.0):
        self.mode, self.tol, self.settle_s, self.place_tol = mode, tol, settle_s, place_tol
        self.release_s, self.retract_deg = release_s, retract_deg
        self.bottle_held = self.bottle_released = False
        self.t_in: float | None = None
        self.t_open: float | None = None
        self.cup_released = self.retracted = False
        self.release_pose: dict[str, float] = {}

    def _settled(self, now: float, obs: dict, pose: dict[str, float], tol: float) -> bool:
        if max(abs(float(obs[k]) - pose[k]) for k in LEFT_5) >= tol:
            self.t_in = None
            return False
        if self.t_in is None:
            self.t_in = now
        return now - self.t_in >= self.settle_s

    def update(self, now: float, obs: dict, action: dict | None = None) -> bool:
        if self.mode == "pour":
            rg = float(obs["right_gripper.pos"])
            if rg > 20:
                self.bottle_held = True
            elif self.bottle_held and rg < 10 and not self.bottle_released:
                self.bottle_released = True
                print(f"[auto] 오른손 병 놓고 빈손으로 닫음 감지 (right_gripper={rg:.1f})")
            return self.bottle_released and self._settled(now, obs, CHASSIS_HOLD, self.tol)

        # 그리퍼 값은 클수록 열림. 쥔 컵 10~24 / 놓을 때 62~96 / 빈손으로 닫으면 ~1.
        lg = float(obs["left_gripper.pos"])
        cmd = float(action["left_gripper.pos"]) if action and "left_gripper.pos" in action else None
        if not self.cup_released:
            if not (lg > GRIP_OPEN and cmd is not None and cmd > GRIP_OPEN):
                self.t_open = None
                return False
            if self.t_open is None:
                self.t_open = now
            if now - self.t_open < self.release_s:
                return False
            self.cup_released = True
            self.release_pose = {k: float(obs[k]) for k in LEFT_5}
            print(f"[auto] 컵 놓음 감지 (left_gripper 측정 {lg:.1f}, 명령 {cmd:.1f})")
        if not self.retracted:
            moved = max(abs(float(obs[k]) - self.release_pose[k]) for k in LEFT_5)
            if moved < self.retract_deg:
                return False
            self.retracted = True
            print(f"[auto] 놓은 자리에서 빠짐 감지 ({moved:.1f}도)")
        closed = lg < GRIP_CLOSED and (cmd is None or cmd < GRIP_CLOSED)
        if not closed:
            self.t_in = None
            return False
        return self._settled(now, obs, POUR_START, self.place_tol)


def install_done(det: DoneDetector) -> None:
    from lerobot.rollout.strategies import base as base_strategy

    inner = base_strategy.send_next_action
    last_action: dict[str, float] = {}

    def send_next_action(obs_processed, obs_raw, ctx, interpolator):
        out = inner(obs_processed, obs_raw, ctx, interpolator)
        if out is not None:
            last_action.update(out)
        if not ctx.runtime.shutdown_event.is_set() and det.update(time.perf_counter(), obs_raw, last_action):
            print(f"[auto] {det.mode} 작업 끝 감지 -> 롤아웃 종료")
            ctx.runtime.shutdown_event.set()
        return out

    base_strategy.send_next_action = send_next_action


def relax_gripper(ph, pt, ctrl_table, tag: str, margin: int, clear_overload: bool) -> None:
    """그리퍼 목표가 실제 위치를 margin 넘게 밀고 있으면 목표를 실제 위치 쪽으로 당긴다."""
    a_te, a_goal, a_pres = (ctrl_table[k][0] for k in ("Torque_Enable", "Goal_Position", "Present_Position"))
    te, c, _ = ph.read1ByteTxRx(pt, GRIPPER_ID, a_te)
    if c != 0:
        print(f"[grip] {tag}: 그리퍼 응답 없음(comm={c}) - 건너뜀")
        return
    if te == 0:
        print(f"[grip] {tag}: 토크 꺼짐 - 할 일 없음")
        return
    goal, _, _ = ph.read2ByteTxRx(pt, GRIPPER_ID, a_goal)
    pres, c, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
    if c != 0:
        print(f"[grip] {tag}: 위치 읽기 실패(comm={c}) - 건너뜀")
        return
    diff = goal - pres
    if abs(diff) > margin:
        new_goal = pres + (margin if diff > 0 else -margin)
        ph.write2ByteTxRx(pt, GRIPPER_ID, a_goal, new_goal)
        print(f"[grip] {tag}: 미는 힘 줄임 goal {goal} -> {new_goal} (present {pres}, 차이 {diff})")
    else:
        print(f"[grip] {tag}: 정상 (goal {goal}, present {pres}, 차이 {diff})")

    if clear_overload and err & OVERLOAD_BIT:
        time.sleep(1.0)
        _, _, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
        if err & OVERLOAD_BIT:
            ph.write1ByteTxRx(pt, GRIPPER_ID, a_te, 0)
            ph.write1ByteTxRx(pt, GRIPPER_ID, a_te, 1)
            time.sleep(0.5)
            _, _, err = ph.read2ByteTxRx(pt, GRIPPER_ID, a_pres)
            print(f"[grip] {tag}: 과부하 지속 -> 토크 off/on 즉시 재설정, 결과 err={err}")
        else:
            print(f"[grip] {tag}: 과부하 해제됨")


def pre_relax(ports: dict[str, str], margin: int) -> None:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    for side, port in ports.items():
        bus = FeetechMotorsBus(port, {"gripper": Motor(GRIPPER_ID, "sts3215", MotorNormMode.RANGE_0_100)})
        try:
            bus.connect(handshake=False)
            relax_gripper(bus.packet_handler, bus.port_handler, bus.model_ctrl_table["sts3215"],
                          f"{side}({port}) 실행 전", margin, clear_overload=True)
        except Exception as e:  # noqa: BLE001
            print(f"[grip] {side}({port}) 실행 전 점검 실패: {e} - 그대로 진행")
        finally:
            if bus.port_handler.is_open:
                bus.port_handler.closePort()


def make_pose_mover(target: dict[str, float], move_s: float):
    from lerobot.utils.robot_utils import precise_sleep

    def _move(hw, duration_s: float = 3.0, fps: int = 50) -> None:
        robot = hw.robot_wrapper
        try:
            cur = robot.get_observation()
            start = {k: float(cur[k]) for k in target if k in cur}
            steps = max(int(move_s * fps), 1)
            for i in range(1, steps + 1):
                t = i / steps
                t = t * t * (3 - 2 * t)
                robot.send_action({k: start[k] * (1 - t) + target[k] * t for k in start})
                precise_sleep(1 / fps)
            time.sleep(0.5)
            now = robot.get_observation()
            off = {k: round(float(now[k]) - target[k], 1) for k in start if abs(float(now[k]) - target[k]) > 10}
            if off:
                # 목표에 못 간 관절은 더 밀지 않게 현재 위치를 목표로 고정한다
                robot.send_action({k: float(now[k]) for k in start})
                print(f"[end] 물따르기 시작 자세 도달 실패(10도 초과) {off} - 현재 위치에서 정지")
            else:
                print("[end] 물따르기 시작 자세 도달")
        except Exception as e:  # noqa: BLE001
            logger.warning("물따르기 시작 자세 이동 실패: %s", e)

    return _move


def install_end_pose(pose: str, margin: int, move_s: float) -> None:
    from lerobot.rollout.strategies.core import RolloutStrategy

    orig_teardown = RolloutStrategy._teardown_hardware

    def _teardown(self, hw, return_to_initial_position: bool = True) -> None:
        robot = hw.robot_wrapper.inner
        if pose == "final":
            if robot.is_connected:
                for side in ("left_arm", "right_arm"):
                    bus = getattr(robot, side).bus
                    try:
                        relax_gripper(bus.packet_handler, bus.port_handler, bus.model_ctrl_table["sts3215"],
                                      f"{side} 종료 시", margin, clear_overload=False)
                    except Exception as e:  # noqa: BLE001
                        print(f"[grip] {side} 종료 시 처리 실패: {e}")
            print("[end] 마지막 자세 유지")
            return orig_teardown(self, hw, return_to_initial_position=False)
        if pose == "pour_start":
            print(f"[end] 물따르기 시작 자세로 {move_s:.0f}s 동안 이동")
            return orig_teardown(self, hw, return_to_initial_position=True)
        return orig_teardown(self, hw, return_to_initial_position=return_to_initial_position)

    RolloutStrategy._teardown_hardware = _teardown
    if pose == "pour_start":
        RolloutStrategy._return_to_initial_position = staticmethod(make_pose_mover(POUR_START, move_s))


def arg_value(args: list[str], key: str) -> str | None:
    for a in args:
        if a.startswith(key + "="):
            return a.split("=", 1)[1]
    return None


def main() -> None:
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--end.pose", dest="end_pose", choices=END_POSES, default="final")
    ap.add_argument("--end.move_s", dest="move_s", type=float, default=5.0)
    ap.add_argument("--grip.margin", dest="margin", type=int, default=10)
    ap.add_argument("--grip.skip_pre", dest="skip_pre", action="store_true")
    ap.add_argument("--pour_hook", dest="pour_hook", action="store_true")
    ap.add_argument("--end.auto", dest="auto", choices=("off", "pour", "place"), default="off")
    ap.add_argument("--end.tol", dest="tol", type=float, default=10.0)
    ap.add_argument("--end.settle_s", dest="settle_s", type=float, default=1.0)
    ap.add_argument("--end.place_tol", dest="place_tol", type=float, default=20.0)
    ns, rest = ap.parse_known_args(sys.argv[1:])
    hook_params, _, rest = parse_hook_args(rest)
    rest = fill_camera_indices(rest)
    if "--strategy.type=base" not in rest:
        sys.exit("--strategy.type=base 로만 동작한다.")
    if arg_value(rest, "--return_to_initial_position") is not None:
        sys.exit("--return_to_initial_position 대신 --end.pose 를 쓴다.")

    if not ns.skip_pre:
        ports = {s: p for s, p in (("left", arg_value(rest, "--robot.left_arm_config.port")),
                                    ("right", arg_value(rest, "--robot.right_arm_config.port"))) if p}
        pre_relax(ports, ns.margin)

    install_end_pose(ns.end_pose, ns.margin, ns.move_s)
    hook = None
    if ns.pour_hook:
        hook = PourHook(hook_params)
        install(hook)
        print(f"[pour_hook] 활성: {hook_params}")
    if ns.auto != "off":
        install_done(DoneDetector(ns.auto, ns.tol, ns.settle_s, ns.place_tol))
    print(f"[end] 끝 자세 = {ns.end_pose}, 자동 종료 = {ns.auto}")

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
