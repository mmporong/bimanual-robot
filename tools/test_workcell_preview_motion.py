import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).parent))
from simulate_urdf_workcell import DEFAULT_URDF, SCENE_ASSUMPTIONS
from workcell_envelopes import WorkcellEnvelopes, ARM_JOINTS
from workcell_preview_motion import build_demo, sample_demo, audit_demo


@pytest.fixture(scope="module")
def preview():
    checker=WorkcellEnvelopes(DEFAULT_URDF,SCENE_ASSUMPTIONS)
    return checker,build_demo(checker.chain)


def test_reset_is_bilateral_level_and_not_right_candidate(preview):
    checker,poses=preview
    reset=poses[0]
    assert reset["name"] == "RESET"
    assert reset == {**poses[-1], "name": "RESET"}
    np.testing.assert_allclose(reset["left_joint_deg"],reset["right_joint_deg"],atol=.001)
    for side in ("left","right"):
        tf=checker.chain.transforms(dict(zip((f"{side}_{j}" for j in ARM_JOINTS),
                                            np.radians(reset[f"{side}_joint_deg"]))))[f"{side}_tool0"]
        assert abs(tf[2,0]) < 1e-5


def test_demo_starts_at_reset_and_finishes_there_without_loop(preview):
    _,poses=preview
    for key in ("left_joint_deg","right_joint_deg"):
        np.testing.assert_allclose(sample_demo(poses,0)[key],poses[0][key])
        np.testing.assert_allclose(sample_demo(poses,100)[key],poses[0][key])
    assert sample_demo(poses,100)["done"]


def test_full_demo_has_no_workcell_envelope_overlap(preview):
    checker,poses=preview
    result=audit_demo(poses,checker,1.0,.0433)
    assert result["passes"],result["failures"][:1]
    assert result["sample_count"] > 180


def test_failure_is_not_silently_dropped(preview):
    _,poses=preview
    class Blocked:
        def check(self,*args): return {"clear":False,"reason":"fixture overlap"}
    result=audit_demo(poses,Blocked(),1.,.0433)
    assert not result["passes"] and result["failures"]


@pytest.mark.parametrize("elapsed,seconds",[(-1,3),(float("nan"),3),(1,0)])
def test_invalid_time_rejected(preview,elapsed,seconds):
    with pytest.raises(ValueError): sample_demo(preview[1],elapsed,seconds)
