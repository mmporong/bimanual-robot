import pytest

from encode_service_views import detail_window


def records():
    return [{"frame": i, "time_s": i/2, **({"detail_frame": i-2} if 2 <= i < 5 else {})}
            for i in range(7)]


def test_detail_retains_same_simulation_time_and_end_boundary():
    assert detail_window(records()) == (1., 2.5, 3)


def test_missing_or_offset_frames_are_rejected():
    data = records()
    del data[3]
    with pytest.raises(ValueError):
        detail_window(data)
    data = records()
    data[3]["time_s"] += .1
    with pytest.raises(ValueError):
        detail_window(data)
