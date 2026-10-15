"""Editorial cameras and lossless-timestamp capture; never changes physics state."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np


def png_complete(path):
    try:
        with Path(path).open('rb') as stream:
            if stream.read(8) != b'\x89PNG\r\n\x1a\n':
                return False
            stream.seek(-12, 2)
            return stream.read() == b'\x00\x00\x00\x00IEND\xaeB\x60\x82'
    except (OSError, ValueError):
        return False


def validate_config(config):
    if config['fps'] != 24 or (config['width'], config['height']) != (1920, 1080):
        raise ValueError('cinematic contract requires 1080p24')
    previous = 0.
    ids = set()
    for shot in config['shots']:
        if shot['id'] in ids or shot['start_s'] != previous or shot['end_s'] <= previous:
            raise ValueError('shots must be contiguous with unique IDs')
        ids.add(shot['id'])
        previous = shot['end_s']
        if shot['anchor'] not in {'base', 'cup', 'bottle'}:
            raise ValueError('unknown camera anchor')
        numeric = [shot['start_s'], shot['end_s'], shot['speed'], shot['focal_start_mm'],
                   shot['focal_end_mm'], *shot['eye_start_m'], *shot['eye_end_m'], *shot['target_offset_m']]
        if not np.isfinite(numeric).all() or min(shot['speed'], shot['focal_start_mm'], shot['focal_end_mm']) <= 0:
            raise ValueError('invalid camera values')
        if not float(120/config['fps']*shot['speed']).is_integer():
            raise ValueError('capture interval must align to a physics tick')
        for field in ('eye_start_m', 'eye_end_m', 'target_offset_m'):
            if len(shot[field]) != 3:
                raise ValueError('camera vectors require three components')
        if float(shot.get('motion_duration_s', shot['end_s']-shot['start_s'])) <= 0:
            raise ValueError('invalid camera motion duration')
    if not ids or not math.isfinite(config['horizontal_aperture_mm']) or config['horizontal_aperture_mm'] <= 0:
        raise ValueError('empty shots or invalid aperture')
    return config


def cinematic_frame(config, tick, anchors):
    time_s = tick/120
    shot = next((s for s in config['shots'] if s['start_s'] <= time_s < s['end_s']), None)
    if shot is None:
        return None
    interval = round(120/config['fps']*shot['speed'])
    if (tick-round(shot['start_s']*120)) % interval:
        return None
    duration_s = shot.get('motion_duration_s', shot['end_s']-shot['start_s'])
    u = float(np.clip((time_s-shot['start_s'])/duration_s, 0., 1.))
    u = u*u*(3-2*u)
    anchor = np.asarray(anchors[shot['anchor']], dtype=float)
    eye = anchor + np.asarray(shot['eye_start_m'])*(1-u)+np.asarray(shot['eye_end_m'])*u
    target = anchor+shot['target_offset_m']
    if not np.isfinite([*eye, *target]).all() or np.linalg.norm(eye-target) < .05:
        raise ValueError('invalid cinematic camera pose')
    return {'time_s': time_s, 'shot_id': shot['id'], 'title': shot['title'],
            'subtitle': shot['subtitle'], 'speed': shot['speed'],
            'view': {'eye_m': eye.tolist(), 'target_m': target.tolist(),
                     'focal_length_mm': shot['focal_start_mm']*(1-u)+shot['focal_end_mm']*u,
                     'horizontal_aperture_mm': config['horizontal_aperture_mm']}}


class CinematicRecorder:
    """One temporary PNG at a time; stream encoded frames instead of storing a PNG sequence."""
    def __init__(self, output, config):
        self.output, self.config, self.frames = Path(output), validate_config(config), []
        self.closed = False
        self.temporary = tempfile.TemporaryDirectory(prefix='service-cinema-')
        self.frame_path = Path(self.temporary.name)/'frame.png'
        self.log = (self.output/'cinematic_encoder.log').open('wb')
        self.process = subprocess.Popen(['ffmpeg', '-n', '-hide_banner', '-loglevel', 'error',
            '-f', 'image2pipe', '-framerate', str(config['fps']), '-vcodec', 'png', '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '17', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(self.output/'cinematic_raw.mp4')],
            stdin=subprocess.PIPE, stderr=self.log)
        (self.output/'cinematic_previews').mkdir()
        self.previous_shot = None

    def record(self, frame, capture):
        self.frame_path.unlink(missing_ok=True)
        capture(frame['view'], 'cinematic', self.frame_path)
        self.process.stdin.write(self.frame_path.read_bytes())
        frame = {**frame, 'frame': len(self.frames)}
        self.frames.append(frame)
        if frame['shot_id'] != self.previous_shot:
            shutil.copy2(self.frame_path, self.output/'cinematic_previews'/(frame['shot_id']+'.png'))
            self.previous_shot = frame['shot_id']
        if len(self.frames) % 24 == 0:
            self.save_metadata()

    def save_metadata(self):
        (self.output/'cinematic_frames.json').write_text(json.dumps(self.frames, ensure_ascii=False, indent=2)+'\n')

    def close(self):
        if self.closed:
            return
        error = None
        try:
            try:
                self.process.stdin.close()
            except OSError as exc:
                error = exc
            try:
                code = self.process.wait(timeout=45)
            except subprocess.TimeoutExpired as exc:
                self.process.kill()
                self.process.wait(timeout=10)
                raise TimeoutError('cinematic encoder did not finalize') from (error or exc)
        finally:
            self.closed = True
            try:
                self.save_metadata()
            finally:
                try:
                    self.log.close()
                finally:
                    self.temporary.cleanup()
        if error is not None:
            raise RuntimeError('cinematic encoder pipe closed early') from error
        if code:
            raise RuntimeError('cinematic encoder failed; inspect cinematic_encoder.log')
