import importlib.util
from pathlib import Path
import sys

import numpy as np


MODULE_PATH = Path(__file__).with_name("audit_pick_trajectory.py")
SPEC = importlib.util.spec_from_file_location("audit_pick_trajectory", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_interpolation_respects_maximum_joint_step():
    start = np.zeros(5)
    end = np.array([0.0, 5.0, -9.0, 0.0, 3.0])
    samples = MODULE.interpolate_segment(start, end, 2.0)
    previous = start
    for sample in samples:
        assert np.max(np.abs(sample - previous)) <= 2.0 + 1e-9
        previous = sample
    np.testing.assert_allclose(samples[-1], end)


def test_trajectory_has_no_duplicate_segment_boundary():
    start = np.zeros(5)
    pregrasp = np.ones(5) * 4.0
    grasp = np.ones(5) * 6.0
    samples = MODULE.trajectory_samples(start, pregrasp, grasp, 2.0)
    values = [tuple(value) for _, value in samples]
    assert values.count(tuple(pregrasp)) == 1
    assert values[-1] == tuple(grasp)


def test_collision_geometry_combines_all_link_collision_elements():
    scene = MODULE.UrdfScene(MODULE.URDF_PATH)
    transforms = scene.link_transforms({"left_gripper": 0.5})
    poly = MODULE.collision_polydata(scene, "left_moving_jaw_link", transforms)
    assert poly is not None
    # mesh 한 개만 읽던 회귀를 막는다. 현행 링크는 mesh+primitive 3개다.
    assert poly.GetNumberOfCells() > 100


def test_unready_ik_plan_is_rejected_before_sampling():
    plan = {"motion_command_emitted": False, "ready_for_collision_review": False, "stages": {}}
    try:
        MODULE.audit_trajectory(plan, [0] * 5, [0] * 5, 2.0, 5.0)
    except ValueError as exc:
        assert "게이트" in str(exc)
    else:
        raise AssertionError("준비되지 않은 IK 계획이 허용됨")
