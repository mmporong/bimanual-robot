"""Tests for bounded depth CameraInfo timestamp association."""

from types import SimpleNamespace

import pytest

from jdamr_cube_navigation.camera_info_history import (
    CameraInfoHistory,
    stamp_nanoseconds,
)


MAXIMUM_LAG_NS = 500_000_000


def _info(stamp_ns, name):
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(
        name=name,
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))


def test_selects_latest_prior_info_without_using_future_info():
    history = CameraInfoHistory()
    earlier = _info(10, 'earlier')
    latest = _info(20, 'latest')
    future = _info(40, 'future')
    history.add(latest)
    history.add(future)
    history.add(earlier)

    assert history.select(30, maximum_lag_ns=MAXIMUM_LAG_NS) is latest
    assert history.rewound is False


def test_public_stamp_converter_returns_integer_nanoseconds():
    stamp = SimpleNamespace(sec=3, nanosec=14)

    assert stamp_nanoseconds(stamp) == 3_000_000_014


def test_future_info_is_not_selected_without_prior_info():
    history = CameraInfoHistory()
    history.add(_info(20, 'future'))

    assert history.select(10, maximum_lag_ns=MAXIMUM_LAG_NS) is None


def test_static_info_is_timeless_before_any_stamped_info():
    history = CameraInfoHistory()
    static = _info(0, 'static')
    history.add(static)

    assert history.select(1, maximum_lag_ns=MAXIMUM_LAG_NS) is static
    assert history.select(1, maximum_lag_ns=MAXIMUM_LAG_NS) is static
    assert history.rewound is False


def test_static_fallback_is_disabled_after_any_stamped_info():
    history = CameraInfoHistory()
    history.add(_info(0, 'static'))
    history.add(_info(20, 'stamped'))
    history.add(_info(0, 'new-static'))

    assert history.select(10, maximum_lag_ns=MAXIMUM_LAG_NS) is None


def test_rewind_discards_old_epoch_stamped_history_and_future_entries():
    history = CameraInfoHistory()
    history.add(_info(100, 'old-prior'))
    history.add(_info(200, 'old-latest'))
    assert history.select(
        200, maximum_lag_ns=MAXIMUM_LAG_NS).name == 'old-latest'

    assert history.select(10, maximum_lag_ns=MAXIMUM_LAG_NS) is None
    assert history.rewound is True
    assert history.select(150, maximum_lag_ns=MAXIMUM_LAG_NS) is None
    assert history.rewound is False


def test_rewind_preserves_new_epoch_info_received_at_exact_depth_stamp():
    history = CameraInfoHistory()
    history.add(_info(200, 'old'))
    assert history.select(
        200, maximum_lag_ns=MAXIMUM_LAG_NS).name == 'old'
    current = _info(10, 'new-exact')
    history.add(current)

    assert history.select(10, maximum_lag_ns=MAXIMUM_LAG_NS) is current
    assert history.rewound is True


def test_rewind_does_not_resurrect_old_exact_info_without_new_arrival():
    history = CameraInfoHistory()
    history.add(_info(200, 'old'))
    assert history.select(300, maximum_lag_ns=MAXIMUM_LAG_NS).name == 'old'

    assert history.select(200, maximum_lag_ns=MAXIMUM_LAG_NS) is None
    assert history.rewound is True


def test_equal_stamp_replaces_message_and_counts_as_latest_arrival():
    history = CameraInfoHistory(capacity=2)
    history.add(_info(10, 'first'))
    history.add(_info(20, 'second'))
    replacement = _info(10, 'replacement')
    history.add(replacement)
    history.add(_info(30, 'third'))

    assert history.select(15, maximum_lag_ns=MAXIMUM_LAG_NS) is replacement


def test_history_retains_only_eight_latest_arrivals():
    history = CameraInfoHistory()
    for stamp_ns in range(1, 10):
        history.add(_info(stamp_ns, str(stamp_ns)))

    assert history.select(1, maximum_lag_ns=MAXIMUM_LAG_NS) is None
    assert history.select(
        9, maximum_lag_ns=MAXIMUM_LAG_NS).name == '9'


def test_stamped_info_older_than_maximum_lag_is_rejected():
    history = CameraInfoHistory()
    history.add(_info(10_000_000_000, 'old'))

    assert history.select(
        3_610_000_000_000,
        maximum_lag_ns=MAXIMUM_LAG_NS) is None


def test_stamped_info_from_immediately_prior_frame_is_allowed():
    history = CameraInfoHistory()
    prior = _info(10_000_000_000, 'prior-frame')
    history.add(prior)

    assert history.select(
        10_400_000_000,
        maximum_lag_ns=MAXIMUM_LAG_NS) is prior


def test_static_info_remains_timeless_without_lag_filtering():
    history = CameraInfoHistory()
    static = _info(0, 'static')
    history.add(static)

    assert history.select(
        3_610_000_000_000,
        maximum_lag_ns=0) is static


@pytest.mark.parametrize(
    ('seconds', 'nanoseconds'),
    [
        (-1, 0),
        (0, -1),
        (0, 1_000_000_000),
        (1.0, 0),
        (0, True),
    ],
)
def test_invalid_camera_info_stamp_is_rejected(seconds, nanoseconds):
    info = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))

    with pytest.raises(ValueError, match='stamp'):
        CameraInfoHistory().add(info)


def test_missing_camera_info_stamp_is_rejected():
    with pytest.raises(ValueError, match='stamp is missing'):
        CameraInfoHistory().add(SimpleNamespace())


def test_invalid_stamp_clears_static_cache_before_raising():
    history = CameraInfoHistory()
    history.add(_info(0, 'static'))

    with pytest.raises(ValueError, match='stamp'):
        history.add(SimpleNamespace())

    assert history.select(1, maximum_lag_ns=MAXIMUM_LAG_NS) is None


def test_invalid_stamp_clears_stamped_cache_and_valid_stamp_recovers():
    history = CameraInfoHistory()
    history.add(_info(10, 'old'))

    with pytest.raises(ValueError, match='stamp'):
        history.add(SimpleNamespace())
    history.add(_info(0, 'static-after-stamped'))
    assert history.select(20, maximum_lag_ns=MAXIMUM_LAG_NS) is None

    current = _info(30, 'current')
    history.add(current)
    assert history.select(30, maximum_lag_ns=MAXIMUM_LAG_NS) is current


def test_missing_public_stamp_fields_are_rejected():
    with pytest.raises(ValueError, match='stamp fields are missing'):
        stamp_nanoseconds(SimpleNamespace())


@pytest.mark.parametrize('image_stamp_ns', [-1, 1.0, True])
def test_invalid_depth_stamp_is_rejected(image_stamp_ns):
    with pytest.raises(ValueError, match='depth image stamp'):
        CameraInfoHistory().select(
            image_stamp_ns, maximum_lag_ns=MAXIMUM_LAG_NS)


@pytest.mark.parametrize('maximum_lag_ns', [-1, 1.0, True])
def test_invalid_maximum_lag_is_rejected(maximum_lag_ns):
    with pytest.raises(ValueError, match='maximum camera info lag'):
        CameraInfoHistory().select(0, maximum_lag_ns=maximum_lag_ns)


@pytest.mark.parametrize('capacity', [0, -1, 1.0, True])
def test_invalid_capacity_is_rejected(capacity):
    with pytest.raises(ValueError, match='capacity'):
        CameraInfoHistory(capacity=capacity)
