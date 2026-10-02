import threading
import time

import pytest

from isaac_phase_executor import PhaseExecutor, phase_for_tick, serve_executor
from hold_flow_mission.planned_ipc import MANIPULATION_PHASES, ROUNDTRIP_PHASES, PROTOCOL, call_executor


def request(phase='ALIGN_KITCHEN', **values):
    return dict(protocol=PROTOCOL, op='execute_phase', request_id=phase,
                mission_id='mission', order_id='order', table_id='table_1',
                drink='COLD_WATER', phase_id=phase, timeout_sec=2., **values)


def submit(executor, payload):
    result = {}
    thread = threading.Thread(target=lambda: result.update(executor.handle(payload)))
    thread.start()
    with executor.condition:
        assert executor.condition.wait_for(lambda: executor.active is not None, timeout=1)
    return thread, result


def test_phase_mapping_covers_pour_and_transit():
    assert phase_for_tick('POUR', 'LEFT_RESET', 0) == 'ALIGN_KITCHEN'
    assert phase_for_tick('POUR', 'LEFT_LIFT_HOLD', 19) == 'GRASP_CUP'
    assert phase_for_tick('POUR', 'RIGHT_LIFT_HOLD', 40) == 'GRASP_BOTTLE'
    assert phase_for_tick('POUR', 'POUR_RETURN', 85) == 'POUR'
    assert phase_for_tick('POUR', 'RIGHT_LOWER', 100) == 'RETURN_BOTTLE'
    assert phase_for_tick('NAVIGATE', 'NAVIGATE', 170) == 'ALIGN_TABLE'


def test_no_completion_until_simulation_boundary(tmp_path):
    executor = PhaseExecutor(tmp_path)
    first, result = submit(executor, request())
    assert executor.checkpoint('ALIGN_KITCHEN', {}) == ''
    assert first.is_alive() and not result
    boundary = threading.Thread(target=lambda: executor.checkpoint('GRASP_CUP', {'time_s': 2.}))
    boundary.start()
    first.join(1)
    assert result['success'] and result['observation']['time_s'] == 2.
    assert boundary.is_alive()  # Physics cannot proceed without the next request.
    second, second_result = submit(executor, request('GRASP_CUP'))
    boundary.join(1)
    assert not boundary.is_alive()
    executor.finish(False, 'contact_lost')
    second.join(1)
    assert second_result['failure_code'] == 'contact_lost'
    assert second_result['stop_confirmed']


def test_single_world_ownership_and_request_payload_binding(tmp_path):
    executor = PhaseExecutor(tmp_path)
    payload = request()
    thread, result = submit(executor, payload)
    assert executor.handle({**payload, 'mission_id': 'other'})['failure_code'] == 'WORLD_OWNED'
    assert executor.handle({**payload, 'phase_id': 'POUR'})['failure_code'] == 'REQUEST_ID_CONFLICT'
    executor.finish(False, 'test_stop')
    thread.join(1)
    assert executor.handle(payload) == result
    assert executor.handle({**payload, 'request_id': 'new'})['success'] is False


def test_cancel_stops_at_checkpoint_before_next_step(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, request())
    executor.checkpoint('ALIGN_KITCHEN', {})
    response = executor.handle(dict(protocol=PROTOCOL, op='cancel_session', mission_id='mission'))
    assert response['canceled'] and not response['stop_confirmed']
    assert executor.checkpoint('ALIGN_KITCHEN', {}) == 'CANCELED'
    executor.finish(False, 'CANCELED')
    thread.join(1)
    assert result['stop_confirmed'] and not result['success']


def test_timeout_is_terminal_and_not_retryable(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, {**request(), 'timeout_sec': .03})
    executor.checkpoint('ALIGN_KITCHEN', {})
    time.sleep(.04)
    assert executor.checkpoint('ALIGN_KITCHEN', {}) == 'TIMEOUT'
    executor.finish(False, 'TIMEOUT')
    thread.join(1)
    assert result['failure_code'] == 'TIMEOUT'
    assert executor.handle({**request(), 'request_id': 'retry'})['failure_code'] == 'TIMEOUT'


def test_final_success_cannot_win_expired_deadline(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, request())
    with executor.condition:
        executor.deadline = time.monotonic() - 1
        executor.finish(True, 'water_served')
    thread.join(1)
    assert result['failure_code'] == 'TIMEOUT'


def test_timeout_cause_survives_later_cancel(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, request())
    with executor.condition:
        executor.stop_reason = 'TIMEOUT'
    executor.handle(dict(protocol=PROTOCOL, op='cancel_session', mission_id='mission'))
    executor.finish(False, 'CANCELED')
    thread.join(1)
    assert result['failure_code'] == 'TIMEOUT'


def test_final_success_requires_durable_result(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, request())
    (tmp_path / 'result.json').mkdir()  # Force an evidence-write error without mocking physics.
    final = {'task_pass': True, 'failure': 'water_served'}
    executor.finish(True, 'water_served', persist_result=final)
    thread.join(1)
    assert result['failure_code'] == 'EVIDENCE_WRITE_FAILED'
    assert final['task_pass'] is False


def test_failed_grasp_boundary_never_publishes_success(tmp_path):
    executor = PhaseExecutor(tmp_path)
    thread, result = submit(executor, request())
    executor.checkpoint('ALIGN_KITCHEN', {})
    assert executor.checkpoint('GRASP_CUP', {}, boundary_failure='lift_failed') == 'lift_failed'
    executor.finish(False, 'lift_failed')
    thread.join(1)
    assert not result['success']


def test_socket_exclusive_and_status_does_not_touch_hardware(tmp_path):
    path = tmp_path / 'executor.sock'
    with serve_executor(path, tmp_path):
        status = call_executor(path, {'op': 'status'}, timeout_sec=1)
        assert status['executor_kind'] == 'isaac_physics'
        assert status['hardware_accessed'] is False
        with pytest.raises(OSError):
            with serve_executor(path, tmp_path):
                pass
        assert call_executor(path, {'op': 'status'}, timeout_sec=1)['success']
    assert not path.exists()


def test_idle_timeout_and_scope_rejection(tmp_path):
    executor = PhaseExecutor(tmp_path, idle_timeout_sec=.01)
    assert executor.handle({**request(), 'table_id': 'table_2'})['failure_code'] == 'SCOPE_UNSUPPORTED'
    assert executor.checkpoint('ALIGN_KITCHEN', {}) == 'EXECUTOR_IDLE_TIMEOUT'


@pytest.mark.parametrize('roundtrip', [False, True])
def test_all_phases_share_world_and_final_response_follows_result_file(tmp_path, roundtrip):
    executor = PhaseExecutor(tmp_path, roundtrip=roundtrip)
    phases = ROUNDTRIP_PHASES if roundtrip else MANIPULATION_PHASES
    results = []

    def client():
        for phase in phases:
            response = executor.handle(request(phase, executor_id=executor.executor_id))
            results.append(response)
            if phase == phases[-1]:
                assert (tmp_path / 'result.json').is_file()

    thread = threading.Thread(target=client)
    thread.start()
    for index, phase in enumerate(phases):
        assert executor.checkpoint(phase, {'sequence': index}) == ''
    executor.finish(True, 'water_served', persist_result={'task_pass': True})
    thread.join(1)
    assert not thread.is_alive()
    assert len(results) == len(phases)
    assert all(row['success'] for row in results)
    assert results[-1]['session_state'] == 'COMPLETE'
    assert executor.index == len(phases)


def test_restarted_world_rejects_stale_execute_and_cancel(tmp_path):
    old = PhaseExecutor(tmp_path, roundtrip=True)
    new = PhaseExecutor(tmp_path, roundtrip=True)
    assert old.executor_id != new.executor_id
    for payload in (request('NAVIGATE_KITCHEN'),
                    {'protocol': PROTOCOL, 'op': 'cancel_session', 'mission_id': 'mission'}):
        result = new.handle({**payload, 'executor_id': old.executor_id})
        assert result['failure_code'] == 'STALE_EXECUTOR'
        assert not new.stop_reason and new.active is None
    thread, result = submit(new, request('NAVIGATE_KITCHEN', executor_id=new.executor_id))
    new.finish(False, 'test_stop')
    thread.join(1)
    assert result['executor_id'] == new.executor_id
