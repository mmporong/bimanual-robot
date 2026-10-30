"""Station handoffs recover read failures without authorizing robot motion."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'tools/jdamr/water_service.py'
SPEC = importlib.util.spec_from_file_location('water_service_wait', SOURCE)
water_service = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(water_service)


@pytest.fixture
def controller(tmp_path, monkeypatch):
    node = water_service.WaterServiceController(
        inference_url='http://order.invalid', state_directory=tmp_path,
        inference_timeout_s=1.0)
    clock = [0.0]
    stopped = [False]
    node._stop_event = SimpleNamespace(
        wait=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
        is_set=lambda: stopped[0])
    monkeypatch.setattr(water_service.time, 'monotonic', lambda: clock[0])
    node._state['order_id'] = 'current'
    node._run_helper = Mock()
    return node, clock, stopped


def ready():
    return {
        'orders': [{'order_id': 'current', 'state': 'RUNNING'}],
        'execution_backend': {
            'drive': {'arms_ready': True, 'next': 'table'},
            'arms': 'carrying_cup', 'running_policy': None,
            'waiting_alignment': {'station': 'table', 'order_id': 'current'},
        },
    }


def test_handoff_retries_read_without_sending_motion_or_confirmation(controller):
    node, clock, _ = controller
    node._http_json = Mock(side_effect=[RuntimeError('temporary timeout'), ready()])
    node._wait_drive('table')
    assert clock[0] == .25
    assert all(call.args == ('GET', '/api/state') for call in node._http_json.call_args_list)
    node._run_helper.assert_not_called()


def test_handoff_still_requires_arms_ready_and_no_running_policy(controller):
    node, clock, _ = controller
    busy = ready()
    busy['execution_backend'].update(running_policy='pour')
    busy['execution_backend']['drive']['arms_ready'] = False
    node._http_json = Mock(side_effect=[busy, ready()])
    node._wait_drive('table')
    assert clock[0] == .25


def test_order_failure_is_not_treated_as_a_transient_read_failure(controller):
    node, clock, _ = controller
    failed = ready()
    failed['orders'][0]['state'] = 'FAILED'
    node._http_json = Mock(return_value=failed)
    with pytest.raises(RuntimeError, match='ended: FAILED'):
        node._wait_drive('table')
    assert clock[0] == 0


def test_handoff_remains_bounded_and_preserves_user_stop(controller):
    node, clock, stopped = controller
    node._http_json = Mock(side_effect=RuntimeError('offline'))
    with pytest.raises(RuntimeError, match='last read error: offline'):
        node._wait_drive('table')
    assert clock[0] == 1.0
    stopped[0] = True
    with pytest.raises(water_service._Stopped):
        node._wait_drive('table')


@pytest.mark.parametrize('post_error', [False, True])
def test_alignment_retries_only_reads_and_never_replays_arm_start(controller, post_error):
    node, _, _ = controller
    confirmation = (RuntimeError('POST response lost') if post_error
                    else {'confirmed': True, 'station': 'table'})
    node._http_json = Mock(side_effect=[RuntimeError('GET timeout'), ready(), confirmation])
    if post_error:
        with pytest.raises(RuntimeError, match='POST response lost'):
            node._confirm_alignment('table')
    else:
        node._confirm_alignment('table')
    calls = node._http_json.call_args_list
    assert [call.args[0] for call in calls] == ['GET', 'GET', 'POST']
    node._run_helper.assert_not_called()
