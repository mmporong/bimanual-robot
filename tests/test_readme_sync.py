from tools.ci.sync_readme_status import END, START, render, synchronize


def sample_spec() -> dict:
    return {
        "revision": "test_revision",
        "status": "test_status",
        "chassis": {
            "frame_outer_footprint": [340.0, 450.0],
            "plates": {"tabletop": {"z_top": 720.0}},
        },
        "right_parallel_gripper": {"model": "right_test"},
        "camera": {"model": "camera_test", "height_above_tabletop": 250.0},
        "navigation": {
            "drive_type": "differential_test",
            "reuse_source": {"motor": "motor_test"},
        },
    }


def sample_gripper() -> dict:
    return {
        "selected_finger": {
            "finger_length_mm": 80.0,
            "tip_lip_mm": 3.0,
            "material": "TPU_95A",
        }
    }


def test_render_uses_single_sources() -> None:
    output = render(sample_spec(), sample_gripper())
    assert "450×340 mm" in output
    assert "80 mm·립 3 mm TPU 95A FinRay 2개" in output
    assert output.count(START) == 1
    assert output.count(END) == 1


def test_synchronize_inserts_and_replaces_generated_block() -> None:
    readme = "# title\n\n## 다음 세션 최우선 작업\nbody\n"
    first = synchronize(readme, render(sample_spec(), sample_gripper()))
    second = synchronize(first, render(sample_spec(), sample_gripper()))
    assert first == second
    assert first.index(START) < first.index("## 다음 세션 최우선 작업")
