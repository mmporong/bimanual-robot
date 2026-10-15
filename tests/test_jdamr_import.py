"""Check the repository-owned ROS source boundary without opening hardware."""

import json
import hashlib
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/jdamr'
RUNTIME_PACKAGES = {
    'jdamr_base_driver', 'jdamr_cube_bringup', 'jdamr_cube_cartographer',
    'jdamr_cube_description', 'jdamr_cube_gazebo', 'jdamr_cube_navigation',
    'jdamr_cube_node', 'jdamr_cube_vslam', 'ydlidar_g4_ros2',
}


def test_runtime_packages_are_embedded_and_have_unique_ros_names():
    names = {}
    for package in SOURCE.glob('*/package.xml'):
        name = ET.parse(package).getroot().findtext('name')
        assert name not in names, (name, names.get(name), package)
        names[name] = package
    assert RUNTIME_PACKAGES <= names.keys()
    for name in RUNTIME_PACKAGES:
        folder = names[name].parent
        assert (folder / 'setup.py').is_file() or (folder / 'CMakeLists.txt').is_file()


def test_import_inventory_contains_no_agent_state_recordings_or_build_logs():
    manifest = json.loads((ROOT / 'config/navigation/jdamr_import_manifest.json').read_text())
    assert manifest['destination'] == 'src/jdamr'
    assert len(manifest['source_commit']) == 40
    assert not manifest['uncommitted_source_rviz_imported']
    for item in manifest['files']:
        relative = Path(item['source_path'])
        assert not relative.is_absolute() and '..' not in relative.parts
        assert not {'.git', '.omx', '.vscode', 'build', 'install', 'log', 'logs'} & set(relative.parts)
        assert relative.suffix not in {'.bag', '.mcap', '.mp4', '.pt', '.pth', '.onnx'}
        assert (SOURCE / relative).is_file(), relative


def test_live_height_contract_and_embedded_driver_geometry_are_preserved():
    mount = yaml.safe_load((SOURCE / 'jdamr_cube_navigation/config/camera_mount_current.yaml').read_text())
    assert mount['camera_mount']['transform']['z_m'] == 1.26
    urdf = ET.parse(SOURCE / 'jdamr_cube_description/urdf/new_base_real.urdf').getroot()
    joints = {joint.get('name'): joint for joint in urdf.findall('joint')}
    tabletop = next(j for j in joints.values() if j.find('child').get('link') == 'upper_tabletop_surface_frame')
    camera = joints['camera_mount_joint']
    assert float(tabletop.find('origin').get('xyz').split()[2]) == 0.72
    assert float(camera.find('origin').get('xyz').split()[2]) == 0.54
    assert (SOURCE / 'firmware/jdamr_cube_fw/wheel_layout.h').is_file()
    assert (SOURCE / 'ydlidar_g4_ros2/vendor/YDLidar-SDK/src/CYdLidar.cpp').is_file()
    assert (SOURCE / 'ydlidar_g4_ros2/vendor/YDLidar-SDK/LICENSE.txt').is_file()


def test_required_simulation_fixtures_are_versioned_not_only_local_outputs():
    manifest = json.loads((ROOT / 'config/navigation/jdamr_import_manifest.json').read_text())
    fixture = manifest['additional_inputs']['generated_regression_fixtures']
    for name, checksum in fixture['files'].items():
        relative = str(Path(fixture['output']) / name)
        assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == checksum
        subprocess.run(
            ['git', 'ls-files', '--error-unmatch', '--', relative],
            cwd=ROOT, check=True, capture_output=True,
        )
