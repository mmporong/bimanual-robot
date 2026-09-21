import copy
import io
import json
from pathlib import Path

import numpy as np
import pytest

from service_cinematic import cinematic_frame, validate_config, png_complete


def test_async_png_writer_must_finish_before_read(tmp_path):
    path = tmp_path/'frame.png'
    assert not png_complete(path)
    path.write_bytes(b'\x89PNG\r\n\x1a\n')
    assert not png_complete(path)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+b'\x00\x00\x00\x00IEND\xaeB\x60\x82')
    assert png_complete(path)


def config():
    return json.loads((Path(__file__).resolve().parents[1]/'config/simulation/service_cinematic.json').read_text())


def test_schedule_is_physics_tick_aligned_and_pure():
    c = validate_config(config())
    before = copy.deepcopy(c)
    anchors = {'base': np.zeros(3), 'cup': np.array([.39, .17, .78]), 'bottle': np.array([.39, -.17, .82])}
    frames = [f for tick in range(283*120) if (f := cinematic_frame(c, tick, anchors))]
    assert 1500 < len(frames) < 2100
    assert len({f['shot_id'] for f in frames}) == 16
    assert all(a['time_s'] < b['time_s'] for a, b in zip(frames, frames[1:]))
    for a, b in zip(frames, frames[1:]):
        if a['shot_id'] == b['shot_id']:
            assert b['time_s']-a['time_s'] == pytest.approx(a['speed']/24)
            assert np.linalg.norm(np.subtract(a['view']['eye_m'], b['view']['eye_m'])) < .1
    assert c == before


@pytest.mark.parametrize('value', [0, -1, float('nan'), .11])
def test_invalid_speed_rejected(value):
    c = config()
    c['shots'][0]['speed'] = value
    with pytest.raises(ValueError):
        validate_config(c)


def test_no_camera_capture_outside_timeline_and_invalid_anchor_rejected():
    c = config()
    assert cinematic_frame(c, 420*120, {}) is None
    with pytest.raises(ValueError):
        cinematic_frame(c, 0, {'base': [float('nan'), 0, 0]})
    c['shots'][1]['start_s'] = 9
    with pytest.raises(ValueError):
        validate_config(c)


@pytest.mark.parametrize('exit_code', [0, 1])
def test_recorder_awaits_capture_and_preserves_metadata_on_encoder_failure(tmp_path, monkeypatch, exit_code):
    import service_cinematic as module

    class Encoder:
        stdin = io.BytesIO()

        def wait(self, timeout):
            assert timeout == 45
            return exit_code

    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: Encoder())
    recorder = module.CinematicRecorder(tmp_path, config())
    observed = []

    def capture(view, name, path):
        assert not path.exists()
        assert name == 'cinematic'
        observed.append(view)
        path.write_bytes(b'frame')

    frame = cinematic_frame(config(), 0, {'base': [0, 0, 0]})
    recorder.record(frame, capture)
    recorder.record({**frame, 'time_s': 1/12}, capture)
    if exit_code:
        with pytest.raises(RuntimeError, match='encoder failed'):
            recorder.close()
    else:
        recorder.close()
    assert len(observed) == 2
    records = json.loads((tmp_path/'cinematic_frames.json').read_text())
    assert [r['frame'] for r in records] == [0, 1]
    assert not Path(recorder.temporary.name).exists()
    assert (tmp_path/'cinematic_previews'/'01_hero.png').read_bytes() == b'frame'


def test_broken_pipe_during_close_still_waits_and_cleans(tmp_path, monkeypatch):
    import service_cinematic as module
    calls = []

    class Pipe:
        def close(self):
            raise BrokenPipeError('encoder exited')

    class Encoder:
        stdin = Pipe()

        def wait(self, timeout):
            calls.append(timeout)
            return 1

    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: Encoder())
    recorder = module.CinematicRecorder(tmp_path, config())
    with pytest.raises(RuntimeError, match='pipe closed early') as error:
        recorder.close()
    assert isinstance(error.value.__cause__, BrokenPipeError)
    assert calls == [45]
    assert recorder.log.closed and recorder.closed
    assert not Path(recorder.temporary.name).exists()
    assert json.loads((tmp_path/'cinematic_frames.json').read_text()) == []
    recorder.close()
    assert calls == [45]


def test_encoder_timeout_kills_only_owned_process_and_cleans(tmp_path, monkeypatch):
    import service_cinematic as module
    calls = []

    class Encoder:
        stdin = io.BytesIO()

        def wait(self, timeout):
            calls.append(('wait', timeout))
            if timeout == 45:
                raise module.subprocess.TimeoutExpired('ffmpeg', timeout)
            return -9

        def kill(self):
            calls.append(('kill',))

    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: Encoder())
    recorder = module.CinematicRecorder(tmp_path, config())
    with pytest.raises(TimeoutError):
        recorder.close()
    assert calls == [('wait', 45), ('kill',), ('wait', 10)]
    assert recorder.log.closed and recorder.closed
    assert not Path(recorder.temporary.name).exists()
