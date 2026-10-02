import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location("replay_cup_pick", Path(__file__).with_name("replay_cup_pick.py"))
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def inputs():
    chain = MODULE.Chain(MODULE.URDF_PATH)
    q = [0, -25, 31, 49, 0]
    tcp = chain.transforms(MODULE.base_positions("left", np.radians(q)))["left_cup_tcp"][:3, 3]
    state = {"side": "left", "joint_order": MODULE.ARM_JOINTS, "joint_degrees": q}
    plan = {"stages": {"pregrasp": {"joint_deg": q, "target_base_footprint_m": tcp.tolist()}}}
    return chain, state, plan


def test_equal_pose_has_zero_model_error_but_unknown_real_error():
    chain, state, plan = inputs()
    row = MODULE.compare_state(state, plan, "pregrasp", chain)
    assert row["saved_vs_planned_model_tcp_distance_mm"] == 0
    assert row["planned_solver_residual_mm"] == 0
    assert row["real_cup_error_mm"] is None
    assert row["grasp_success"] is None


@pytest.mark.parametrize("value", [[0]*4, [True, 0, 0, 0, 0], [float("nan")]*5, [float("inf")]*5, ["0"]*5])
def test_bad_joint_vector_rejected(value):
    chain, state, plan = inputs()
    state["joint_degrees"] = value
    with pytest.raises(ValueError):
        MODULE.compare_state(state, plan, "pregrasp", chain)


def test_wrong_joint_order_rejected():
    chain, state, plan = inputs()
    state["joint_order"] = list(reversed(state["joint_order"]))
    with pytest.raises(ValueError):
        MODULE.compare_state(state, plan, "pregrasp", chain)


def test_archive_and_report_do_not_overwrite(tmp_path):
    _, state, plan = inputs()
    state_path, plan_path = tmp_path / "state.json", tmp_path / "plan.json"
    state_path.write_text(json.dumps(state))
    plan_path.write_text(json.dumps(plan))
    output = tmp_path / "report"
    result = MODULE.write_report(plan_path, [state_path], [], MODULE.URDF_PATH, "pregrasp", output)
    assert result["hardware_accessed"] is False
    assert len(result["manifest"]) == 3
    assert (output / "state_00.json").read_bytes() == state_path.read_bytes()
    assert (output / "index.html").is_file()
    with pytest.raises(FileExistsError):
        MODULE.write_report(plan_path, [state_path], [], MODULE.URDF_PATH, "pregrasp", output)
