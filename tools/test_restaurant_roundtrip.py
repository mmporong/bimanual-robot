import math

import pytest

from restaurant_layout import load_layout
from restaurant_roundtrip import load_roundtrip_config, roundtrip_routes, station_rest_verified
from mobile_service_control import follow_path
from isaac_phase_executor import phase_for_tick


def test_routes_preserve_exact_station_endpoints():
    layout = load_layout()
    config = load_roundtrip_config()
    outbound, home = roundtrip_routes(layout, config)
    assert outbound[0] == home[-1] == layout['waypoints']['dock'][:2]
    assert outbound[-1] == layout['waypoints']['kitchen'][:2]
    assert home[0] == config['guest_exit_pose_m_rad'][:2]
    assert len(home) > 2 and len(outbound) > 2


def test_station_requires_pose_and_rest_not_just_path_end():
    config = load_roundtrip_config()
    observation = dict(position_m=[0., 0., .05], base_yaw_rad=0.,
                       base_linear_speed_m_s=0., base_angular_speed_rad_s=0.)
    assert station_rest_verified(observation, [0., 0., 0.], config, kitchen=True)
    for change in [dict(position_m=[.001, 0., .05]), dict(base_yaw_rad=.001),
                   dict(base_linear_speed_m_s=.01), dict(base_angular_speed_rad_s=.02),
                   dict(base_yaw_rad=math.nan)]:
        assert not station_rest_verified({**observation, **change}, [0., 0., 0.], config, kitchen=True)


def test_fine_alignment_does_not_accept_default_navigation_error():
    assert follow_path([-.01, 0., 0.], [[0., 0.]], 0.)['arrived']
    assert not follow_path([-.01, 0., 0.], [[0., 0.]], 0.,
                           position_tolerance_m=.0005, yaw_tolerance_rad=.0005)['arrived']
    with pytest.raises(ValueError):
        follow_path([0., 0., 0.], [[0., 0.]], 0., position_tolerance_m=0.)


def test_alignment_floor_preserves_measured_arrival_gate():
    options = dict(position_tolerance_m=.0005, yaw_tolerance_rad=.0005,
                   minimum_angular_rad_s=.015)
    residual = follow_path([0., -.00038, .0036], [[0., 0.]], 0., **options)
    assert residual['angular_rad_s'] <= -.015 and not residual['arrived']
    arrived = follow_path([0., 0., .0001], [[0., 0.]], 0., **options)
    assert arrived['arrived'] and arrived['angular_rad_s'] == 0.


@pytest.mark.parametrize('state,phase', [
    ('START_SETTLE', 'NAVIGATE_KITCHEN'), ('GO_KITCHEN', 'NAVIGATE_KITCHEN'),
    ('KITCHEN_ALIGN', 'ALIGN_KITCHEN'), ('KITCHEN_SETTLE', 'ALIGN_KITCHEN'),
    ('BACKOUT', 'NAVIGATE_TABLE'), ('NAVIGATE', 'NAVIGATE_TABLE'),
    ('DOCK', 'ALIGN_TABLE'), ('SETTLE_BASE', 'ALIGN_TABLE'),
    ('RETURN_CLEAR', 'NAVIGATE_DOCK'), ('RETURN_HOME', 'NAVIGATE_DOCK'),
    ('HOME_SETTLE', 'NAVIGATE_DOCK')])
def test_physical_navigation_phase_mapping(state, phase):
    assert phase_for_tick(state, state, 0., roundtrip=True) == phase
