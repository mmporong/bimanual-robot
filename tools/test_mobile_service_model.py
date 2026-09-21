from mobile_service_model import fixed_component_pairs, ground_collision, ground_supported, ground_window_verified


def test_one_wheel_or_chassis_drag_cannot_pass_ground_check():
    valid = {"left_wheel_link": 20., "right_wheel_link": 20., "rear_caster_link": 10.}
    assert ground_supported(valid)
    assert not ground_supported({**valid, "left_wheel_link": 0.})
    assert not ground_supported({**valid, "base_link": 2.})
    assert ground_collision({**valid, "left_wrist_link": .1})
    assert ground_collision({**valid, "base_link": float("nan")})


def test_contact_window_requires_both_wheels_and_a_caster():
    def sample(left=20.):
        return {"ground_force_n": {"left_wheel_link": left, "right_wheel_link": 20., "rear_caster_link": 10.},
                "position_m": [0., 0., 0.], "tilt_deg": 0.}
    assert ground_window_verified([sample(), sample(), sample(0.), sample(), sample()])
    assert not ground_window_verified([sample(0.) for _ in range(5)])
    assert not ground_window_verified([sample() for _ in range(4)])
    assert not ground_window_verified([{**sample(), "tilt_deg": 2.} for _ in range(5)])


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
