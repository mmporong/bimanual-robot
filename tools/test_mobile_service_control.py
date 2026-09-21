import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


MODULE_PATH = Path(__file__).with_name("mobile_service_control.py")
SPEC = importlib.util.spec_from_file_location("mobile_service_control", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_straight_path_commands_forward_motion():
    command = MODULE.follow_path([0.0, 0.0, 0.0], [[0.0, 0.0], [0.5, 0.0]], 0.0)
    assert command["linear_m_s"] > 0.0
    assert command["left_rad_s"] == pytest.approx(command["right_rad_s"])
    assert command["linear_m_s"] <= 0.08


def test_left_turn_has_positive_yaw_and_faster_right_wheel():
    # Start near the corner so the lookahead point is visibly to the left.
    command = MODULE.follow_path(
        [0.20, 0.0, 0.0],
        [[0.0, 0.0], [0.25, 0.0], [0.25, 0.25]],
        math.pi / 2.0,
    )
    assert command["angular_rad_s"] > 0.0
    assert command["right_rad_s"] > command["left_rad_s"]


def test_goal_position_with_wrong_yaw_does_not_false_arrive():
    command = MODULE.follow_path([0.4, 0.4, 0.0], [[0.4, 0.4]], math.pi / 2.0)
    assert command["arrived"] is False
    assert abs(command["angular_rad_s"]) > 0.0


def test_zero_axle_offset_rotates_in_place_at_goal():
    command = MODULE.follow_path(
        [0.4, 0.4, 0.0], [[0.4, 0.4]], math.pi / 2.0, axle_offset=0.0
    )
    assert command["linear_m_s"] == 0.0
    assert command["angular_rad_s"] > 0.0


def test_arrived_stops_all_motion():
    command = MODULE.follow_path([0.4, 0.4, 0.01], [[0.0, 0.0], [0.4, 0.4]], 0.0)
    assert command["arrived"] is True
    assert command["left_rad_s"] == 0.0
    assert command["right_rad_s"] == 0.0
    assert command["linear_m_s"] == 0.0
    assert command["angular_rad_s"] == 0.0


@pytest.mark.parametrize(
    ("pose", "path", "goal_yaw"),
    [
        ([0.0, 0.0, 0.0], [], 0.0),
        ([0.0, 0.0, math.nan], [[0.0, 0.0]], 0.0),
        ([0.0, 0.0, 0.0], [[0.0, math.inf]], 0.0),
        ([0.0, 0.0, 0.0], [[0.0, 0.0]], math.nan),
    ],
)
def test_invalid_input_is_rejected(pose, path, goal_yaw):
    with pytest.raises(ValueError):
        MODULE.follow_path(pose, path, goal_yaw)


def test_kinematic_simulation_converges_through_ninety_degree_curve():
    axle_offset = 0.105
    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    path = np.array([[0.0, 0.0], [0.40, 0.0], [0.40, 0.40]], dtype=float)
    dt = 0.02

    for _ in range(3000):
        command = MODULE.follow_path(pose, path, math.pi / 2.0, axle_offset=axle_offset)
        if command["arrived"]:
            break
        yaw = float(pose[2])
        linear = command["linear_m_s"]
        angular = command["angular_rad_s"]
        # The axle midpoint moves along the chassis heading.  Since the input
        # pose is behind that axle, rotation adds the lateral offset term.
        pose[0] += (linear * math.cos(yaw) + axle_offset * angular * math.sin(yaw)) * dt
        pose[1] += (linear * math.sin(yaw) - axle_offset * angular * math.cos(yaw)) * dt
        pose[2] = math.atan2(math.sin(yaw + angular * dt), math.cos(yaw + angular * dt))

    assert command["arrived"] is True
    assert pose[:2] == pytest.approx(path[-1], abs=0.015)
    assert pose[2] == pytest.approx(math.pi / 2.0, abs=0.02)
