"""양팔 시뮬레이션 리셋과 빈손 방향 시연. 실물 홈/파지 궤적이 아니다."""
from __future__ import annotations

import math
import numpy as np

from workcell_envelopes import ARM_JOINTS


def leveled_pose(chain, side, arm_deg):
    """URDF의 공구 X축 수직 성분을 없애는 손목 회전을 계산한다."""
    q = np.asarray(arm_deg, dtype=float).copy()
    values = []
    for roll in (0.0, 90.0):
        q[4] = roll
        positions = dict(zip((f"{side}_{j}" for j in ARM_JOINTS), np.radians(q)))
        values.append(chain.transforms(positions)[f"{side}_tool0"][2, 0])
    if math.hypot(*values) < 1e-8:
        raise ValueError("손목 회전만으로 수평 기준을 결정할 수 없습니다")
    angle = math.degrees(math.atan2(-values[0], values[1])) % 180.0
    q[4] = angle
    return q.tolist()


def build_demo(chain):
    # 시각 검토용 명목 팔 자세. 실제 엔코더 영점/리셋 값으로 사용하지 않는다.
    reset = {side: leveled_pose(chain, side, [0,-55,70,-15,0]) for side in ("left", "right")}
    raised = {side: leveled_pose(chain, side, [0,-40,45,-5,0]) for side in ("left", "right")}
    turned = {side: [*raised[side][:4], raised[side][4]-90.0] for side in ("left", "right")}
    def pose(name, values):
        return {"name": name, "left_joint_deg": values["left"], "right_joint_deg": values["right"]}
    return [pose("RESET", reset), pose("RAISE_CLEAR", raised),
            pose("WRIST_QUARTER_TURN", turned), pose("HORIZONTAL_READY", raised),
            pose("RETURN_RESET", reset)]


def sample_demo(poses, elapsed, segment_seconds=3.0):
    """처음부터 한 번만 재생하고 마지막 리셋에 머무른다. 순환하지 않는다."""
    if not math.isfinite(elapsed) or elapsed < 0 or not math.isfinite(segment_seconds) or segment_seconds <= 0:
        raise ValueError("시연 시간은 유한한 0 이상, 구간 시간은 양수여야 합니다")
    if len(poses) < 2:
        raise ValueError("시연에는 시작/종료 자세가 필요합니다")
    done = elapsed >= segment_seconds*(len(poses)-1)
    if done:
        return {**poses[-1], "done": True}
    index = min(int(elapsed/segment_seconds), len(poses)-2)
    t = (elapsed-index*segment_seconds)/segment_seconds
    weight = t*t*(3-2*t)
    result = {"name": f"{poses[index]['name']} -> {poses[index+1]['name']}", "done": False}
    for key in ("left_joint_deg", "right_joint_deg"):
        a, b = np.asarray(poses[index][key]), np.asarray(poses[index+1][key])
        result[key] = (a+(b-a)*weight).tolist()
    return result


def audit_demo(poses, checker, left_open, right_open):
    """각 관절 간격 1도 이하 샘플의 외곽 검사. 연속 충돌 증명은 아니다."""
    failures = []
    checked = 0
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        delta = max(max(abs(x-y) for x,y in zip(a[key], b[key]))
                    for key in ("left_joint_deg", "right_joint_deg"))
        steps = max(1, math.ceil(delta))
        for n in range(steps+1):
            alpha=n/steps
            values = {key: (np.asarray(a[key])*(1-alpha)+np.asarray(b[key])*alpha).tolist()
                      for key in ("left_joint_deg", "right_joint_deg")}
            result=checker.check(values["left_joint_deg"], values["right_joint_deg"], left_open, right_open)
            checked += 1
            if not result["clear"]:
                failures.append({"segment": i, "sample": n, **result})
    return {"passes": not failures, "sample_count": checked, "failures": failures,
            "maximum_joint_sample_interval_deg": 1.0,
            "continuous_collision_validated": False, "physical_grasp_verified": False}
