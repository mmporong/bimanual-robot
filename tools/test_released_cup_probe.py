import numpy as np
import pytest

from released_cup_probe import arm_tracking_error_rad


def test_gripper_stopping_on_cup_is_not_arm_tracking_failure():
    goal = np.zeros(13)
    actual = goal.copy()
    actual[10:] = [.25, .03, .03]
    ids = {"left": list(range(5)), "right": list(range(5, 10))}
    assert arm_tracking_error_rad(actual, goal, ids) == 0.
    actual[2] = .16
    assert arm_tracking_error_rad(actual, goal, ids) == pytest.approx(.16)
