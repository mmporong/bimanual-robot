from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from workcell_envelopes import WorkcellEnvelopes, cylinder_overlap
from simulate_urdf_workcell import DEFAULT_URDF, SCENE_ASSUMPTIONS


def test_cylinder_containment_is_detected_without_surface_crossing():
    assert cylinder_overlap((np.array([-.1,-.1,0]), np.array([.1,.1,1])),
                            np.zeros(2), .02, .2, .4)


def test_circle_corner_and_vertical_separation():
    assert not cylinder_overlap((np.array([.08,.08,0]), np.array([.1,.1,1])),
                                np.zeros(2), .1, .2, .4)
    assert not cylinder_overlap((np.array([-.1,-.1,1]), np.array([.1,.1,2])),
                                np.zeros(2), .1, .2, .4)


def test_known_oblique_grasp_is_not_accepted_as_clear():
    checker = WorkcellEnvelopes(DEFAULT_URDF, SCENE_ASSUMPTIONS)
    report = checker.check([-.01,-2.02,29.68,27.34,1.36], [0,-68,92,-22,0], .5,.0325)
    assert not report["clear"]
    assert any(pair[1] == "cup" for pair in report["overlap_candidates"])


def test_invalid_joint_shape_and_limits():
    import pytest
    checker = WorkcellEnvelopes(DEFAULT_URDF, SCENE_ASSUMPTIONS)
    with pytest.raises(ValueError):
        checker.check([0]*4,[0]*5)
    with pytest.raises(ValueError):
        checker.check([float("nan")]*5,[0]*5)
    report = checker.check([300,0,0,0,0],[0]*5)
    assert "left_shoulder_pan" in report["joint_limit_violations"]


def test_nonfinite_scene_and_gripper_cannot_bypass_check():
    import pytest
    with pytest.raises(ValueError):
        WorkcellEnvelopes(DEFAULT_URDF, {**SCENE_ASSUMPTIONS, "cup_radius_m": float("nan")})
    checker = WorkcellEnvelopes(DEFAULT_URDF, SCENE_ASSUMPTIONS)
    with pytest.raises(ValueError):
        checker.check([0]*5,[0]*5,left_open=float("nan"))
