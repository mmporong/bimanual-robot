"""Exercise departure reserve independently from the running cutoff."""

import io
import json
from pathlib import Path
from unittest.mock import Mock

from jdamr_cube_navigation import corridor_route
from jdamr_cube_navigation.restaurant_service import load_service_contract, ServiceRoute
import pytest
import yaml


CONTRACT = Path(__file__).resolve().parents[1] / 'config/restaurant_service_contract.yaml'


def readiness_node(monkeypatch, *, guard=None, protection=None):
    """Exercise the real readiness methods with a bounded simulated clock."""
    clock = [0.0]
    monkeypatch.setattr(corridor_route.time, 'monotonic', lambda: clock[0])

    def spin(*_args, **_kwargs):
        clock[0] += .1

    monkeypatch.setattr(corridor_route, 'spin_node', spin)
    node = object.__new__(ServiceRoute)
    node.stop_requested = False
    node.navigation_profile = 'new_base_revisit_candidate'
    node._guard_failure = Mock(return_value=guard)
    node._revisit_protection_ready = Mock(return_value=protection)
    node.travel_pose_status = None
    node.get_logger = Mock(return_value=Mock())
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.battery_voltage = 12.0
    node.emit = Mock()
    return node


@pytest.mark.parametrize('reason', [
    'battery low: voltage=10.192V limit=10.500V',
    'scan stale: age=2.000s limit=1.000s',
    'AMCL pose missing',
])
def test_readiness_preserves_guard_failure(monkeypatch, reason):
    node = readiness_node(monkeypatch, guard=reason)
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == reason
    node.get_logger().error.assert_called_once_with(
        'navigation readiness timeout: ' + reason)


def test_readiness_preserves_protection_failure(monkeypatch):
    reason = '/cmd_vel publisher must be collision_monitor only'
    node = readiness_node(monkeypatch, protection=reason)
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == reason


def test_readiness_stop_takes_priority_and_success_clears_old_failure(monkeypatch):
    node = readiness_node(monkeypatch, guard='battery low: voltage=10.192V limit=10.500V')
    node.stop_requested = True
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == 'operator stop'
    node.stop_requested = False
    node._guard_failure.return_value = None
    assert node.wait_until_ready(timeout=.2)
    assert node.last_readiness_failure is None


@pytest.mark.parametrize('voltage,expected', [
    (10.6, 'battery departure reserve: voltage=10.600V limit=10.800V'),
    (None, 'battery voltage missing'),
    (float('nan'), 'battery voltage non-finite: value=nan'),
])
def test_readiness_reports_departure_reserve_after_running_guard_passes(
        monkeypatch, voltage, expected):
    node = readiness_node(monkeypatch)
    node.battery_voltage = voltage
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == expected


def test_readiness_preserves_lost_operator_link(monkeypatch, tmp_path):
    node = readiness_node(monkeypatch)
    node.operator_heartbeat = tmp_path / 'missing-heartbeat'
    assert not node.wait_until_ready(timeout=.2)
    assert node.readiness_failure_reason() == 'operator_link_lost'


@pytest.mark.parametrize('voltage_v,ready', [
    (None, False), (float('nan'), False), (float('inf'), False),
    (10.464, False), (10.6, False), (10.8, True), (12.0, True),
])
def test_departure_reserve(voltage_v, ready):
    """A near-cutoff idle reading must not authorize a new mission."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = node.service_contract['minimum_running_battery_v']
    node.battery_voltage = voltage_v
    node.emit = Mock()
    assert node._departure_battery_ready() is ready
    assert node.minimum_battery_v == 10.5
    if not ready:
        assert node.emit.call_args.kwargs['reason'] == 'battery_departure_reserve'
        json.dumps(node.emit.call_args.kwargs, allow_nan=False)


def test_reserve_is_not_reapplied_between_steps_but_running_cutoff_remains():
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = 10.5
    node.emit = Mock()
    node.battery_voltage = 11.2
    assert node._departure_battery_ready()
    node._mission_started = True
    node.battery_voltage = 10.7
    assert node._departure_battery_ready()
    node.battery_voltage = 10.4
    assert not node._departure_battery_ready()


@pytest.mark.parametrize('method', ['execute', '_execute_reverse_path'])
def test_low_reserve_never_dispatches_forward_or_reverse(method):
    """Both action entrypoints reject low reserve before send_goal_async."""
    node = object.__new__(ServiceRoute)
    node.service_contract = load_service_contract(CONTRACT)
    node.minimum_battery_v = 10.5
    node.battery_voltage = 10.6
    node.emit = Mock()
    node.stop_requested = False
    node._navigation_ready = Mock(return_value=True)
    node.navigate, node.follow_reverse = Mock(), Mock()
    node.waypoints = [{'id': 'home'}]
    node.config = {}
    node.result_stream = io.StringIO()
    assert not getattr(node, method)(*([None] if method.startswith('_') else []))
    node.navigate.send_goal_async.assert_not_called()
    node.follow_reverse.send_goal_async.assert_not_called()


@pytest.mark.parametrize('value', [None, True, float('nan'), -1.0, 10.4])
def test_invalid_departure_contract(tmp_path, value):
    """Reject missing, nonfinite and weaker-than-running reserve settings."""
    document = yaml.safe_load(CONTRACT.read_text())
    document['minimum_start_battery_v'] = value
    path = tmp_path / 'contract.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_service_contract(path)
