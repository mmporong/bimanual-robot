"""Load the camera mount values used by the standalone depth observer."""

from pathlib import Path

import yaml

from .box_docking_target import _validated_mount


def load_observer_camera_mount(path: str, config_directory: Path) -> dict:
    """Return observer geometry parameters from a validated mount document."""
    if not isinstance(path, str):
        raise ValueError('observer camera mount path must be a string')
    path = path.strip()
    if not path:
        return {}

    try:
        mount_path = Path(path).expanduser()
        config_directory = Path(config_directory).expanduser()
    except TypeError as error:
        raise ValueError('observer camera mount path is invalid') from error
    if not mount_path.is_absolute():
        mount_path = config_directory / mount_path

    try:
        document = yaml.safe_load(mount_path.read_text(encoding='utf-8'))
    except OSError as error:
        raise ValueError(
            f'cannot read observer camera mount: {mount_path}') from error
    except yaml.YAMLError as error:
        raise ValueError(
            f'observer camera mount YAML is malformed: {mount_path}') from error
    if not isinstance(document, dict):
        raise ValueError('observer camera mount document must be a mapping')

    try:
        _x_m, _y_m, z_m, roll_rad, pitch_rad, _yaw_rad = (
            _validated_mount(document))
    except ValueError as error:
        raise ValueError(f'invalid observer camera mount: {error}') from error
    return {
        'camera_height_m': z_m,
        'camera_roll_rad': roll_rad,
        'camera_pitch_rad': pitch_rad,
    }
