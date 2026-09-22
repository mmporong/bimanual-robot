import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


MODULE_PATH = Path(__file__).with_name("cup_pick_dry_run.py")
SPEC = importlib.util.spec_from_file_location("cup_pick_dry_run", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Scalar:
    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value


class Vector:
    def __init__(self, value):
        self.value = np.asarray([value], dtype=float)

    def __getitem__(self, index):
        return self.value[index]


def box(label_id, confidence, xyxy):
    return SimpleNamespace(cls=Scalar(label_id), conf=Scalar(confidence), xyxy=Vector(xyxy))


def test_select_best_cup_uses_floor_contact_and_confidence():
    result = SimpleNamespace(
        orig_shape=(480, 640),
        names={0: "cup", 1: "bottle"},
        boxes=[
            box(0, 0.61, [100, 50, 180, 210]),
            box(1, 0.99, [200, 30, 300, 300]),
            box(0, 0.87, [300, 60, 420, 500]),
        ],
    )
    detection = MODULE.select_best_cup(result, 0.4)
    assert detection is not None
    assert detection.confidence == 0.87
    assert detection.contact_px == [360.0, 479.0]


def test_pixel_to_base_xy_reconstructs_known_rectangle():
    pixels = np.float32([[10, 20], [110, 20], [110, 220], [10, 220]])
    base = np.float32([[0.30, 0.20], [0.30, -0.20], [0.70, -0.20], [0.70, 0.20]])
    homography, _ = MODULE.cv2.findHomography(pixels, base, method=0)
    xy = MODULE.pixel_to_base_xy([60.0, 120.0], homography)
    np.testing.assert_allclose(xy, [0.50, 0.0], atol=1e-6)


def test_dls_ik_recovers_reachable_left_cup_tcp_target():
    chain = MODULE.Chain(MODULE.URDF_PATH)
    known_q = np.radians(np.array([15.0, -25.0, 45.0, 20.0, 5.0]))
    transform = chain.transforms(MODULE.base_positions("left", known_q))["left_cup_tcp"]
    target = transform[:3, 3]
    q, error = MODULE.solve_with_restarts(
        chain,
        "left",
        "left_cup_tcp",
        target,
        restarts=20,
        seed=11,
    )
    assert error < 0.001
    solved = chain.transforms(MODULE.base_positions("left", q))["left_cup_tcp"][:3, 3]
    np.testing.assert_allclose(solved, target, atol=0.001)


def test_axis_residual_distinguishes_opposite_tool_direction():
    chain = MODULE.Chain(MODULE.URDF_PATH)
    q = np.zeros(len(MODULE.ARM_JOINTS))
    transform = chain.transforms(MODULE.base_positions("left", q))["left_tool0"]
    current_axis = transform[:3, 2]
    same = MODULE.residual(
        chain, "left", q, "left_tool0", transform[:3, 3], "left_tool0", current_axis
    )
    opposite = MODULE.residual(
        chain, "left", q, "left_tool0", transform[:3, 3], "left_tool0", -current_axis
    )
    assert np.linalg.norm(same[3:]) < 1e-9
    assert np.linalg.norm(opposite[3:]) > 3.0


def test_pick_plan_contains_pregrasp_and_grasp_without_motion(monkeypatch):
    monkeypatch.setattr(
        MODULE,
        "solve_with_restarts",
        lambda *_args, **_kwargs: (np.zeros(len(MODULE.ARM_JOINTS)), 0.001),
    )
    workspace = {
        "homography": np.eye(3),
        "surface_z_m": 0.72,
        "cup_height_m": 0.10,
        "grasp_height_fraction": 0.60,
        "pregrasp_clearance_m": 0.06,
    }
    detection = MODULE.Detection("cup", 0.9, [0, 0, 1, 1], [0.35, 0.10])
    result = MODULE.solve_pick_plan(detection, workspace)
    assert set(result["stages"]) == {"pregrasp", "grasp"}
    assert result["motion_command_emitted"] is False
    assert result["solver"] == "numerical_jacobian_damped_least_squares"


def test_measured_target_uses_tilted_axis_and_raises_pregrasp(monkeypatch):
    captured = []

    def fake_solver(_chain, _side, _frame, target, **kwargs):
        captured.append((target.copy(), kwargs["axis_target"].copy()))
        return np.zeros(len(MODULE.ARM_JOINTS)), 0.001

    monkeypatch.setattr(MODULE, "solve_with_restarts", fake_solver)
    detection = MODULE.Detection("cup", 0.9, [0, 0, 1, 1], [0.5, 0.5])
    result = MODULE.solve_pick_plan_at_xy(
        detection,
        np.array([0.320, 0.170]),
        0.6931,
        0.070,
        0.060,
        -55.0,
    )
    pregrasp_target, axis = captured[0]
    grasp_target, _ = captured[1]
    assert pregrasp_target[0] < grasp_target[0]
    assert pregrasp_target[2] > grasp_target[2]
    assert axis[2] < 0.0
    assert result["approach_pitch_deg"] == -55.0
