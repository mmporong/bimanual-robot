"""Real bringup model must match the measured lower-base contract."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'urdf/new_base_real.urdf'
CONTRACT = ROOT / 'config/new_base_geometry.yaml'


def _xyz(joint):
    return tuple(float(value) for value in joint.find('origin').attrib['xyz'].split())


def test_new_base_footprint_lidar_and_wheels():
    geometry = yaml.safe_load(CONTRACT.read_text(encoding='utf-8'))
    root = ET.parse(MODEL).getroot()
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    links = {link.attrib['name']: link for link in root.findall('link')}
    rail_half = float(links['lower_front_rail_link'].find(
        'collision/geometry/box').attrib['size'].split()[0]) / 2.0
    wheel_half = float(links['left_wheel_link'].find(
        'collision/geometry/cylinder').attrib['length']) / 2.0
    front = _xyz(joints['lower_front_rail_joint'])[0] + rail_half
    rear = _xyz(joints['lower_rear_rail_joint'])[0] - rail_half
    left = _xyz(joints['left_wheel_joint'])[1] + wheel_half
    right = _xyz(joints['right_wheel_joint'])[1] - wheel_half
    assert abs(front - geometry['front_to_wheel_axis']['value']) < 1e-9
    assert abs(front - rear - geometry['frame_length']['value']) < 1e-9
    assert abs(left - right - geometry['wheel_outer_width']['value']) < 1e-9
    assert abs(_xyz(joints['left_wheel_joint'])[1] -
               _xyz(joints['right_wheel_joint'])[1] -
               geometry['wheel_center_separation']['value']) < 1e-9
    wheel = links['left_wheel_link'].find('collision/geometry/cylinder')
    assert float(wheel.attrib['radius']) == geometry['wheel_radius_initial']['value']
    assert float(wheel.attrib['length']) == geometry['wheel_visual_width']['value']
    assert _xyz(joints['laser_joint']) == tuple(geometry['laser_translation']['value'])
    assert float(joints['laser_joint'].find('origin').attrib['rpy'].split()[2]) == \
        geometry['laser_yaw']['value']
    imu_rpy = [float(v) for v in joints['imu_joint'].find('origin').attrib['rpy'].split()]
    assert imu_rpy == geometry['imu_rotation_rpy']['value']
    assert joints['imu_joint'].find('parent').attrib['link'] == 'base_link'
    assert 'arm_base_link' not in links
    assert 'caster_link_front' not in links
    assert not any('caster' in name for name in links)
    for side, front_name in (('left', 'left_wheel_joint'), ('right', 'right_wheel_joint')):
        joint = joints[f'rear_{side}_wheel_joint']
        x_m, y_m, z_m = _xyz(joint)
        assert x_m == -geometry['wheelbase']['value']
        assert (y_m, z_m) == _xyz(joints[front_name])[1:]
        assert joint.attrib['type'] == 'continuous'
        rear_wheel = links[f'rear_{side}_wheel_link'].find('collision/geometry/cylinder')
        assert rear_wheel.attrib == wheel.attrib
    assert abs(geometry['wheelbase']['value'] - 2 * (
        geometry['frame_length']['value'] / 2 - geometry['front_to_wheel_axis']['value'])) < 1e-9


def test_runtime_wheelbase_matches_final_bimanual_design_without_claiming_tape_measurement():
    geometry = yaml.safe_load(CONTRACT.read_text())
    repository = ROOT.parents[2]
    navigation = yaml.safe_load((repository / 'design/mechanical/'
                                'hold_flow_mechanical_v0_3.yaml').read_text())['navigation']
    provenance = navigation['geometry_provenance']
    wheelbase_m = (provenance['front_axle_from_chassis_center']['value']
                   - provenance['rear_axle_from_chassis_center']['value'])
    assert geometry['wheelbase']['value'] == wheelbase_m
    assert geometry['wheelbase']['unit'] == 'm'
    assert geometry['wheelbase']['status'] == 'user_confirmed_symmetry_derived_not_tape_measured'


def test_camera_height_chain_matches_current_mount_without_duplicate_optical_tf():
    mount = yaml.safe_load((ROOT.parent / 'jdamr_cube_navigation' / 'config' /
                            'camera_mount_current.yaml').read_text())['camera_mount']
    joints = {joint.attrib['name']: joint
              for joint in ET.parse(MODEL).getroot().findall('joint')}
    tabletop = joints['upper_tabletop_surface_joint']
    camera = joints['camera_mount_joint']
    assert camera.find('parent').attrib['link'] == 'upper_tabletop_surface_frame'
    position = tuple(a + b for a, b in zip(_xyz(tabletop), _xyz(camera)))
    assert position == tuple(mount['transform'][key] for key in ('x_m', 'y_m', 'z_m'))
    assert _xyz(camera)[2] == mount['provenance'][
        'lens_height_above_chassis_tabletop']['value']
    assert tuple(map(float, camera.find('origin').attrib['rpy'].split())) == tuple(
        mount['transform'][key] for key in ('roll_rad', 'pitch_rad', 'yaw_rad'))
    assert 'camera_depth_optical_joint' not in joints
    assert 'camera_color_optical_joint' not in joints
