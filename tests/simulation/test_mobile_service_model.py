import pytest

from simulation.mobile_service_model import fixed_component_pairs, four_wheel_targets, ground_collision, ground_supported, ground_window_verified, wheel_joint_indices


def test_missing_wheel_or_chassis_drag_cannot_pass_ground_check():
    valid = {
        "left_wheel_link": 20.,
        "right_wheel_link": 20.,
        "rear_left_wheel_link": 20.,
        "rear_right_wheel_link": 20.,
    }
    assert ground_supported(valid)
    for wheel in valid:
        assert not ground_supported({**valid, wheel: 0.})
    assert not ground_supported({**valid, "base_link": 2.})
    assert ground_collision({**valid, "left_wrist_link": .1})
    assert ground_collision({**valid, "base_link": float("nan")})


def test_contact_window_requires_all_four_wheels():
    def sample(rear_left=20.):
        return {"ground_force_n": {
                    "left_wheel_link": 20., "right_wheel_link": 20.,
                    "rear_left_wheel_link": rear_left, "rear_right_wheel_link": 20.,
                },
                "position_m": [0., 0., 0.], "tilt_deg": 0.}
    assert ground_window_verified([sample(), sample(), sample(0.), sample(), sample()])
    assert not ground_window_verified([sample(0.) for _ in range(5)])
    assert not ground_window_verified([sample() for _ in range(4)])
    assert not ground_window_verified([{**sample(), "tilt_deg": 2.} for _ in range(5)])


def test_four_wheel_targets_repeat_each_side_for_rear_axle():
    assert four_wheel_targets(1.25, -0.75) == [1.25, -0.75, 1.25, -0.75]


def test_all_wheel_dofs_are_removed_from_arm_group_and_missing_rear_fails():
    names = ["left_shoulder_pan", "left_wheel_joint", "right_wheel_joint",
             "rear_left_wheel_joint", "rear_right_wheel_joint", "right_shoulder_pan"]
    wheels = wheel_joint_indices(names)
    assert wheels == [1, 2, 3, 4]
    assert [name for index, name in enumerate(names) if index not in wheels] == [
        "left_shoulder_pan", "right_shoulder_pan"
    ]
    with pytest.raises(ValueError):
        wheel_joint_indices(names[:-2] + [names[-1]])


def test_fixed_groups_do_not_cross_moving_joint(tmp_path):
    model = tmp_path/"robot.urdf"
    model.write_text('''<robot name="test">
      <link name="root"/><link name="plate"/><link name="mount"/>
      <link name="wheel"/><link name="arm"/><link name="tool"/>
      <joint name="j1" type="fixed"><parent link="root"/><child link="plate"/></joint>
      <joint name="j2" type="fixed"><parent link="plate"/><child link="mount"/></joint>
      <joint name="j3" type="continuous"><parent link="root"/><child link="wheel"/></joint>
      <joint name="j4" type="revolute"><parent link="mount"/><child link="arm"/></joint>
      <joint name="j5" type="fixed"><parent link="arm"/><child link="tool"/></joint>
    </robot>''')
    pairs = set(fixed_component_pairs(model, {"root", "plate", "mount", "wheel", "arm", "tool"}))
    assert pairs == {("mount", "plate"), ("mount", "root"), ("plate", "root"), ("arm", "tool")}
    assert all("wheel" not in pair for pair in pairs)


def test_merged_links_can_be_absent_from_runtime(tmp_path):
    model = tmp_path/"robot.urdf"
    model.write_text('''<robot name="test"><link name="a"/><link name="b"/><link name="c"/>
      <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="bc" type="fixed"><parent link="b"/><child link="c"/></joint>
    </robot>''')
    assert fixed_component_pairs(model, {"a", "c"}) == [("a", "c")]
