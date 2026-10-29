"""상주형 조작 실행기의 이동 자세(stow) 처리: 팔 하드웨어·lerobot 없이 가짜 팔로 확인한다."""
import importlib.util
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROLLOUT_DIR = Path(__file__).resolve().parents[2] / "tools" / "rollout"


@pytest.fixture(scope="module")
def svc():
    sys.path.insert(0, str(ROLLOUT_DIR))
    spec = importlib.util.spec_from_file_location("rollout_service_under_test", ROLLOUT_DIR / "rollout_service.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # 보간 대기 생략. time 모듈 자체를 바꾸면 다른 테스트의 대기까지 사라지므로 이 모듈의 참조만 바꾼다
    module.time = SimpleNamespace(sleep=lambda s: None, perf_counter=time.perf_counter,
                                  strftime=time.strftime)
    return module


class FakeArm:
    def __init__(self, joints, stuck=()):
        self.pos = dict(joints)
        self.stuck = set(stuck)
        self.sent = []

    def send_action(self, action):
        self.sent.append(dict(action))
        for k, v in action.items():
            if k not in self.stuck:
                self.pos[k] = v
        return action


class FakeRobot:
    """robot_wrapper 흉내: get_observation()과 inner.left_arm / inner.right_arm."""

    def __init__(self, left, right, stuck=()):
        self.inner = SimpleNamespace(left_arm=FakeArm(left, stuck), right_arm=FakeArm(right, stuck),
                                     is_connected=True)

    def get_observation(self):
        obs = {f"left_{k}": v for k, v in self.inner.left_arm.pos.items()}
        obs.update({f"right_{k}": v for k, v in self.inner.right_arm.pos.items()})
        return obs


JOINTS = ("shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos", "wrist_roll.pos")
CUP_HOLD = dict(zip(JOINTS, (66.7, 29.0, 81.5, -95.5, -94.5)))
RIGHT_AFTER_POUR = dict(zip(JOINTS, (10.0, 20.0, 60.0, -40.0, 5.0)))
STOW = {**{f"left_{k}": v for k, v in zip(JOINTS, (0.0, 0.4, 96.0, -81.3, -89.5))},
        **{f"right_{k}": v for k, v in zip(JOINTS, (-3.3, 0.0, 95.7, -81.5, 1.5))}}


def make_service(svc, robot, stow=STOW):
    s = svc.Service.__new__(svc.Service)
    s.ns = SimpleNamespace(stow_pose="place_end", start_move_s=0.1, start_max_speed=1000.0, start_tol=10.0,
                           margin=10, move_s=0.1)
    s.contexts = {"pour": SimpleNamespace(hardware=SimpleNamespace(robot_wrapper=robot))}
    s.stow_target = stow
    s.lock = threading.Lock()
    s.arms = "unknown"
    s.state, s.current, s.results, s.timings, s.idle_failures = "ready", None, [], {}, 0
    s.jobs = SimpleNamespace(put=lambda job: None)
    return s


def test_right_only_move_never_commands_the_left_arm(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    right = {k: v for k, v in STOW.items() if k.startswith("right_")}
    ok, off = svc.move_joints(robot, right, 0.1, 1000.0, 10.0, "test")
    assert ok and off == {}
    assert robot.inner.left_arm.sent == []
    assert robot.inner.left_arm.pos == CUP_HOLD
    assert robot.get_observation()["right_elbow_flex.pos"] == pytest.approx(95.7)


def test_already_there_does_not_move(svc):
    robot = FakeRobot({k.removeprefix("left_"): v for k, v in STOW.items() if k.startswith("left_")},
                      {k.removeprefix("right_"): v for k, v in STOW.items() if k.startswith("right_")})
    ok, _ = svc.move_joints(robot, STOW, 0.1, 1000.0, 10.0, "test")
    assert ok
    assert robot.inner.left_arm.sent == [] and robot.inner.right_arm.sent == []


def test_blocked_joint_stops_in_place_and_reports(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR, stuck={"elbow_flex.pos"})
    ok, off = svc.move_joints(robot, STOW, 0.1, 1000.0, 10.0, "test")
    assert not ok
    assert set(off) == {"left_elbow_flex.pos", "right_elbow_flex.pos"}
    # 마지막 명령은 현재 위치 고정(더 밀지 않음)
    assert robot.inner.right_arm.sent[-1]["elbow_flex.pos"] == RIGHT_AFTER_POUR["elbow_flex.pos"]


def test_stow_job_folds_both_arms(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    result = service._run_stow({"submitted": 0.0})
    assert result["ok"] and result["reason"] == "stowed"
    assert service.arms == "stowed"
    assert all(abs(robot.get_observation()[k] - v) < 1e-6 for k, v in STOW.items())


def test_stow_requires_configured_pose(svc):
    service = make_service(svc, FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR), stow=None)
    code, body, job = service.submit("stow")
    assert code == 404 and body["error"] == "stow_not_configured" and job is None


def test_after_pour_only_the_right_arm_folds(svc):
    robot = FakeRobot(CUP_HOLD, RIGHT_AFTER_POUR)
    service = make_service(svc, robot)
    result = {}
    arms = service._stow_right_after_pour(SimpleNamespace(robot_wrapper=robot), result)
    assert arms == "carrying_cup" and result["right_stow"] == "ok"
    assert robot.inner.left_arm.sent == [] and robot.inner.left_arm.pos == CUP_HOLD
