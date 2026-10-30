"""Tests for the standalone observer camera-mount loader."""

import math
from pathlib import Path

import pytest

from jdamr_cube_navigation.observer_camera_mount import (
    load_observer_camera_mount,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIRECTORY = PACKAGE_ROOT / 'config'


def _mount_document(*, height_m=1.26, roll_rad=-0.04, pitch_rad=0.92):
    return f'''camera_mount:
  status: derived
  parent_frame: base_link
  child_frame: camera_link
  transform:
    x_m: -0.275
    y_m: 0.0
    z_m: {height_m}
    roll_rad: {roll_rad}
    pitch_rad: {pitch_rad}
    yaw_rad: 0.0
'''


def test_empty_path_keeps_existing_observer_parameters():
    assert load_observer_camera_mount('', CONFIG_DIRECTORY) == {}
    assert load_observer_camera_mount('   ', CONFIG_DIRECTORY) == {}


def test_relative_current_mount_maps_si_values_to_observer_parameters():
    result = load_observer_camera_mount(
        'camera_mount_current.yaml', CONFIG_DIRECTORY)

    assert result == pytest.approx({
        'camera_height_m': 1.26,
        'camera_roll_rad': 0.01837857145491081,
        'camera_pitch_rad': 0.8414714126967582,
    })


def test_absolute_and_expanduser_paths_preserve_meter_and_radian_values(
        tmp_path, monkeypatch):
    home = tmp_path / 'home'
    home.mkdir()
    mount_path = home / 'mount.yaml'
    mount_path.write_text(
        _mount_document(
            height_m=0.84,
            roll_rad=math.radians(-3.0),
            pitch_rad=math.radians(41.0)),
        encoding='utf-8')
    monkeypatch.setenv('HOME', str(home))

    result = load_observer_camera_mount('~/mount.yaml', tmp_path)

    assert result['camera_height_m'] == pytest.approx(0.84)
    assert result['camera_roll_rad'] == pytest.approx(math.radians(-3.0))
    assert result['camera_pitch_rad'] == pytest.approx(math.radians(41.0))


@pytest.mark.parametrize(
    ('contents', 'message'),
    [
        ('- not-a-mapping\n', 'must be a mapping'),
        ('camera_mount: [\n', 'YAML is malformed'),
        (_mount_document(height_m='.nan'), 'must be finite'),
        ('camera_mount: {}\n', 'document is incomplete'),
    ],
)
def test_invalid_mount_documents_raise_clear_value_errors(
        tmp_path, contents, message):
    mount_path = tmp_path / 'mount.yaml'
    mount_path.write_text(contents, encoding='utf-8')

    with pytest.raises(ValueError, match=message):
        load_observer_camera_mount(str(mount_path), tmp_path)


def test_missing_mount_file_is_reported_as_value_error(tmp_path):
    with pytest.raises(ValueError, match='cannot read observer camera mount'):
        load_observer_camera_mount('missing.yaml', tmp_path)


def test_non_string_mount_path_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='path must be a string'):
        load_observer_camera_mount(None, tmp_path)
