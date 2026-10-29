"""Check rendered observer arguments using a fake ros2 executable, never ROS."""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


@pytest.mark.parametrize('enabled', ['false', 'true'])
@pytest.mark.parametrize('dropin', [False, True])
def test_systemd_and_four_frame_override_forward_the_selected_observer_mode(
        tmp_path, enabled, dropin):
    deployment = Path(__file__).resolve().parents[2] / 'deployment'
    output = tmp_path / 'rendered'
    subprocess.run([sys.executable, str(deployment / 'render_systemd_templates.py'),
                    '--output', str(output), '--jdamr-ws', str(tmp_path / 'ws'),
                    '--deploy-dir', str(tmp_path / 'deploy')], check=True, capture_output=True)
    base = (output / 'jdamr-box-observer.service').read_text()
    environment = os.environ.copy()
    for line in base.splitlines():
        if line.startswith('Environment='):
            environment.update(item.split('=', 1) for item in shlex.split(line.partition('=')[2]))
    assert environment['JDAMR_TABLE_COMPONENTS'] == 'false'
    assert environment['JDAMR_TABLE_TRACKING'] == 'false'
    assert f'EnvironmentFile=-{tmp_path}/deploy/pointcloud_observer.env' in base
    environment.update(JDAMR_TABLE_COMPONENTS=enabled, JDAMR_TABLE_TRACKING=enabled)
    fakebin = tmp_path / 'bin'
    fakebin.mkdir()
    fake = fakebin / 'ros2'
    fake.write_text(f'#!{sys.executable}\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    fake.chmod(0o755)
    environment['PATH'] = str(fakebin) + os.pathsep + environment['PATH']
    unit = ((output / 'jdamr-box-observer.service.d/50-four-frames.conf').read_text()
            if dropin else base)
    command = next(line.partition('=')[2] for line in unit.splitlines()
                   if line.startswith('ExecStart=') and line != 'ExecStart=')
    executable, option, script = shlex.split(command)
    # Underlay setup is outside this argument-forwarding test. The fake ros2
    # remains the only executable reached by the rendered observer command.
    completed = subprocess.run([executable, option, 'source() { :; }\n' + script],
                               env=environment, check=True, capture_output=True, text=True)
    args = json.loads(completed.stdout)
    assert args[:3] == ['run', 'jdamr_cube_navigation', 'depth_box_parking']
    assert f'table_components:={enabled}' in args
    assert f'table_tracking:={enabled}' in args
    assert ('stable_frames:=4' in args) is dropin
