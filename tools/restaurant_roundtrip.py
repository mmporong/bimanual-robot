"""Simulation-only station targets and dock roundtrip contracts."""
import json
import math
from pathlib import Path

from restaurant_layout import plan_route

CONFIG = Path(__file__).resolve().parents[1] / 'config/simulation/dock_roundtrip.json'


def load_roundtrip_config(path=CONFIG):
    data = json.loads(path.read_text())
    if data.get('schema') != 'dock_roundtrip_v1':
        raise ValueError('invalid roundtrip config schema')
    for key, value in data.items():
        if key.endswith('_m_rad'):
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                raise ValueError('invalid station target: ' + key)
        elif key.endswith(('_m', '_rad', '_s')):
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid positive roundtrip parameter: ' + key)
    return data


def roundtrip_routes(layout, config):
    # The raster plan ends at the approach station; fine alignment reaches the
    # source manipulation frame without resetting the measured base pose.
    outbound = plan_route(layout, 'dock', 'kitchen')
    outbound[0] = layout['waypoints']['dock'][:2]
    outbound[-1] = layout['waypoints']['kitchen'][:2]
    waypoints = {**layout['waypoints'], 'guest_exit': config['guest_exit_pose_m_rad']}
    home = plan_route({**layout, 'waypoints': waypoints}, 'guest_exit', 'dock')
    home[0] = waypoints['guest_exit'][:2]
    home[-1] = waypoints['dock'][:2]
    return outbound, home


def station_rest_verified(observation, target, config, *, kitchen=False):
    position = observation['position_m']
    yaw = observation['base_yaw_rad']
    position_error_m = math.dist(position[:2], target[:2])
    yaw_error_rad = abs(math.atan2(math.sin(target[2]-yaw), math.cos(target[2]-yaw)))
    prefix = 'kitchen' if kitchen else 'dock'
    return (position_error_m <= config[prefix + '_position_tolerance_m']
            and yaw_error_rad <= config[prefix + '_yaw_tolerance_rad']
            and observation['base_linear_speed_m_s'] < config['rest_linear_speed_m_s']
            and observation['base_angular_speed_rad_s'] < config['rest_angular_speed_rad_s'])
