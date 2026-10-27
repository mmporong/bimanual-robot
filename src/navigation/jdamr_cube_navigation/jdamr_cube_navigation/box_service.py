"""Approach a mapped table region, then park using a fresh observed surface."""

import argparse
from collections import deque
import fcntl
import hashlib
from itertools import islice
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_docking_target import (
    _validated_mount, compute_box_docking_target, MINIMUM_DISTANCE_M,
)
from jdamr_cube_navigation.box_lidar_witness import witness_box_face_with_lidar
from jdamr_cube_navigation.corridor_route import (
    load_route, spin_node, TRANSIT_PLAN_END_TOLERANCE_M)
from jdamr_cube_navigation.docking_stop_profile import apply_docking_stop_profile
from jdamr_cube_navigation.parking import (
    DEFAULT_FRONT_GAP_M, load_parking_contract, MAXIMUM_CONTRACT_VALUES,
    MAXIMUM_FRONT_GAP_M, MINIMUM_FRONT_GAP_M, pose_errors,
)
from jdamr_cube_navigation.rear_box_service import RearBoxParkingMixin
from jdamr_cube_navigation.restaurant_service import (
    LIVE_APPROACH_MAX_M, OPERATOR_LINK_TIMEOUT_S, SCAN_MEMORY_KEYFRAMES,
    ServiceRoute, TRANSIT_VARIANTS)
from jdamr_cube_navigation.reverse_parking import CachedStaticCorridorValidator
from jdamr_cube_navigation.service_destinations import load_registry, route_config, verify_identity
from jdamr_cube_navigation.station_parking_reference import (
    anchor_face_to_taught_lateral,
    LATERAL_PRECISION_STATUS,
    TaughtStationReference,
)
from jdamr_cube_navigation.surface_approach import (
    FACE_APPROACH_TIME_S, face_approach_waypoints,
)
from jdamr_cube_navigation.table_edge_target import compute_table_edge_docking_target
from nav2_msgs.action import Spin
from nav_msgs.msg import Path as RosPath
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
import yaml


SEARCH_STEP_RAD = math.radians(30.0)
# config/depth_box_parking.yaml maximum_depth_m: the observer accepts no wider range.
DEPTH_WINDOW_CEILING_M = 2.0
# The final-approach plan only proves the way to the box is free; the straight
# approach and the 1 cm stop confirmation decide the pose. On 5 cm costmap cells a
# goal 0.135 m from a LiDAR-marked face can sit one cell inside its inflation, and
# the plan then ends 0.05 m short (table_01, 2026-10-01 15:50): one cell + 1 cm.
FINAL_PLAN_END_TOLERANCE_M = 0.06
# That plan check still ends 5 cm from the face; the straight approach then closes to
# the contract's target_front_gap_m.
FINAL_PLAN_GAP_M = 0.05
# The Parking straight stops inside its 1 cm goal checker before the goal: 38 parks
# at a 5 cm target (09-30 to 10-02) ended at 5.51 cm on average (sd 0.10 cm, internal
# estimate), so the Parking straight runs this much further (no record for Graceful).
FINAL_STOP_SHORT_M = 0.005
# The nearest front point is judged within this of the target gap.
FINAL_GAP_TOLERANCE_M = 0.01
TABLE_FINE_GAP_TOLERANCE_M = 0.003
TABLE_FINE_MAX_CORRECTIONS = 3
TABLE_FINE_MIN_PROGRESS_M = 0.001
# Before the straight the base is turned onto the face normal when it is off by more
# than this: face alignment ended 2.9 deg off (inside its 3 deg), the straight added
# 0.4 deg and the 3.28 deg stop failed the contract (2026-10-06 16:00, water station).
FINAL_HEADING_TRIM_RAD = math.radians(1.0)
# Zero-turn final approach (--zero-turn-final): trim residuals from 0.3 deg up to
# 30 deg, accept the trim within 0.3 deg, and the straight within 5 mm.
# The box camera and its observer run only while a box is the next stop (2026-10-06:
# their 0.8 core held the Pi at 80 deg C and throttled, Nav2's control loop fell to
# 2.5-7 Hz and the controller dropped out of the DDS graph; stopped, 80.8 -> 74.0 deg C
# and load 6.3 -> 3.1 within a minute). The Astra needs 11.3 s to its first depth
# image, so it starts with the transit to the first observation point.
CAMERA_UNITS = ('jdamr-box-rgbd.service', 'jdamr-box-observer.service')
CAMERA_READY_TIMEOUT_S = 30.0
FINAL_TRIM_MIN_RAD = math.radians(0.3)
FINAL_TRIM_MAX_RAD = math.radians(30.0)
FINAL_TRIM_TOLERANCE_RAD = math.radians(0.3)
# The zero-turn straight may end this much short: the gap judgement (target +-1 cm)
# decides, and a StopZone stop a few mm early is not a failed approach.
FINAL_STRAIGHT_TOLERANCE_M = 0.01
SEARCH_MAX_STEPS = 12
SEARCH_MAX_CUMULATIVE_RAD = math.tau
# A region bearing smaller than this is treated as centred: the failure is not a
# heading problem and the fixed sweep continues.
SEARCH_DIRECTED_MIN_RAD = math.radians(8.0)
SEARCH_RECOVERABLE_REASONS = frozenset({
    'stable detected front surface is required',
    'LiDAR face support is below five points',
    'LiDAR face tangent spread is too narrow',
    'LiDAR face line residual is too large',
    'LiDAR and depth face normals disagree',
    'LiDAR and depth face distances disagree',
    'observed_box_outside_selected_table_region',
    'stable detected table_edge surface is required',
    'observed_table_edge_outside_selected_table_region',
})
# Observer statuses without a frame: the depth input itself was late.
OBSERVER_LATENCY_REASONS = frozenset({'stale_or_future_depth', 'camera_info_missing'})
# Derived from recorded status timing; see evaluation/
# depth_box_parking_provenance.yaml observer_status_timing_20260929 (test_t30).
OBSERVER_SILENT_S = 1.5
# Box-side rejections that describe input age, not the observed scene.
BOX_INPUT_LATENCY_FAILURES = frozenset({
    'box observation is stale or future-dated',
    'LiDAR scan is stale or future-dated',
    'fresh LiDAR scan is required for box witness',
})
# Face-to-base_link distance at which an in-place turn clears the face: the
# collision monitor rotation polygon radius plus margin (pinned by test_t29).
ESCAPE_CLEARANCE_M = 0.565
# A straight reverse still moves away from a face up to this angle (cos 15 deg
# = 0.97); the reverse path itself is validated on the costmap. 3 deg stopped
# escapes after tilted stops on 2026-09-30 (11 and 23 deg).
ESCAPE_MAX_FACE_ANGLE_RAD = math.radians(15.0)
# Chassis-front band excluded from escape validation: one diagonal costmap
# cell plus capture noise, rounded up to path steps (recomputed by test_t29).
ESCAPE_VALIDATION_EXCLUDE_M = 0.10
# The escape is an intermediate motion judged by the alignment goal checker.
ESCAPE_PATH_CONTRACT = {
    'xy_tolerance_m': MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'],
    'yaw_tolerance_rad': math.radians(MAXIMUM_CONTRACT_VALUES['yaw_tolerance_deg']),
}
# A completed parking confirmation hands off immediately unless a hold is requested.
STOP_DWELL_S = 0.0
SURFACE_MODES = ('front', 'table_edge')
STATION_REFERENCE_MAX_AGE_S = 0.5
STATION_REFERENCE_MIN_PERIOD_S = 0.2


def front_lead_m(front_extent_m, half_width_m, heading_error_rad):
    """
    Return how far the leading front corner is ahead of the axle along the face normal.

    The front edge is straight across the frame, so a base turned by the heading error
    against the face meets it first at a corner: 2.7 deg on the 0.45 m frame puts that
    corner 1.06 cm ahead of the edge centre. The 0.54 m wheels sit 2.7 cm behind the
    front edge and would lead only past a 30 deg turn.
    """
    return (front_extent_m * math.cos(heading_error_rad)
            + half_width_m * abs(math.sin(heading_error_rad)))


class BoxObservationUnavailable(RuntimeError):
    """
    Describe whether changing the camera heading may recover observation.

    latency_limited marks a verdict formed from too few fresh frames after
    an input delay; it may be re-observed in place but never rotates.
    """

    def __init__(self, reason, *, retryable, latency_limited=False,
                 fresh_frames=0, required_fresh_frames=None,
                 fresh_rejection_reasons=(), latency_reasons=()):
        super().__init__(f'box observation unavailable: {reason}')
        self.reason = reason
        self.retryable = retryable
        self.latency_limited = latency_limited
        self.fresh_frames = fresh_frames
        self.required_fresh_frames = required_fresh_frames
        self.fresh_rejection_reasons = frozenset(fresh_rejection_reasons)
        self.latency_reasons = frozenset(latency_reasons)


class BoxSearchBudget:
    """Issue at most one full turn as fixed, auditable search increments."""

    def __init__(self, *, target_directed=False):
        self.steps_used = 0
        self.cumulative_yaw_rad = 0.0
        self.reposition_attempted = False
        self.target_directed = target_directed

    def next_rotation(self, toward_rad=None):
        """
        Reserve the next bounded rotation, or return None when exhausted.

        With a known region bearing the step turns toward it (at most one step);
        otherwise it is the fixed counter-clockwise sweep step.
        """
        if self.steps_used >= SEARCH_MAX_STEPS:
            return None
        if self.target_directed:
            # A registered table is not an unknown object somewhere in the room.
            if (toward_rad is None or not math.isfinite(toward_rad)
                    or abs(toward_rad) < SEARCH_DIRECTED_MIN_RAD
                    or self.steps_used):
                return None
        step = SEARCH_STEP_RAD
        if (toward_rad is not None and math.isfinite(toward_rad)
                and abs(toward_rad) >= SEARCH_DIRECTED_MIN_RAD):
            step = math.copysign(min(abs(toward_rad), SEARCH_STEP_RAD), toward_rad)
        remaining = SEARCH_MAX_CUMULATIVE_RAD - self.cumulative_yaw_rad
        if remaining + 1e-12 < abs(step):
            return None
        self.steps_used += 1
        self.cumulative_yaw_rad += abs(step)
        return step


def last_logged_face(path):
    """Return the last box face a previous run observed (its escape reference)."""
    face = None
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        target = event.get('target') if isinstance(event, dict) else None
        if (event.get('event') == 'box_target_observed' and isinstance(target, dict)
                and 'face_center_map_xy_m' in target and 'outward_normal_map_xy' in target):
            face = {'face_center_map_xy_m': [float(v) for v in target['face_center_map_xy_m']],
                    'outward_normal_map_xy': [float(v) for v in target['outward_normal_map_xy']]}
    if face is None:
        raise ValueError('no observed box face in the resume log')
    return face


class BoxServiceRoute(RearBoxParkingMixin, ServiceRoute):
    """Keep every movement on the existing Nav2 and collision-monitor path."""

    def __init__(self, *args, **kwargs):
        self.last_scan = None
        self.scan_memory = deque(maxlen=SCAN_MEMORY_KEYFRAMES)
        self.box_status_history = deque(maxlen=16)
        self.last_box_face = None
        self.depth_camera_info = None
        self.depth_info_subscription = None
        self.station_reference = None
        self.station_reference_mode = None
        self.station_reference_profile = None
        self.station_reference_rgb_info = None
        self.station_reference_result = None
        self.station_reference_result_received_s = None
        self.station_reference_goal = None
        self.station_reference_goal_frozen = False
        self.station_reference_image_subscription = None
        self.station_reference_info_subscription = None
        self._station_reference_pending = None
        self._station_reference_lock = threading.Lock()
        self._station_reference_wake = threading.Event()
        self._station_reference_stop = threading.Event()
        self._station_reference_thread = None
        # The same observer configuration decides how many frames make a
        # stable face; an unreadable value leaves every latency verdict limited.
        try:
            observer = yaml.safe_load((
                Path(get_package_share_directory('jdamr_cube_navigation'))
                / 'config/depth_box_parking.yaml').read_text(encoding='utf-8'))
            stable_frames = observer['jdamr_depth_box_parking']['ros__parameters'][
                'stable_frames']
        except (KeyError, OSError, TypeError, yaml.YAMLError):
            stable_frames = None
        valid = (isinstance(stable_frames, int) and not isinstance(stable_frames, bool)
                 and stable_frames >= 1)
        self.observer_stable_frames = stable_frames if valid else None
        super().__init__(*args, **kwargs)
        self.precision_parameters = AsyncParameterClient(
            self, 'collision_monitor')

    @staticmethod
    def _station_image_array(message):
        """Decode the two rectified RGB encodings used by the Astra driver."""
        encoding = str(getattr(message, 'encoding', '')).lower()
        if encoding not in ('bgr8', 'rgb8'):
            raise ValueError('station reference requires bgr8 or rgb8 image')
        height, width = int(message.height), int(message.width)
        step = int(message.step)
        if height <= 0 or width <= 0 or step < width * 3:
            raise ValueError('invalid station reference image dimensions')
        raw = np.frombuffer(message.data, dtype=np.uint8)
        if raw.size != height * step:
            raise ValueError('station reference image byte count mismatch')
        image = raw.reshape(height, step)[:, :width * 3].reshape(height, width, 3)
        if encoding == 'rgb8':
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    @staticmethod
    def _stamp_s(message):
        stamp = message.header.stamp
        value = stamp.sec + stamp.nanosec * 1e-9
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('station reference image stamp invalid')
        return value

    @staticmethod
    def _yaw(rotation):
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))

    def _station_info_callback(self, message):
        self.station_reference_rgb_info = message

    def _station_image_callback(self, message):
        """Replace the pending frame; the subscription callback never runs ORB."""
        if getattr(self, 'station_reference', None) is None:
            return
        try:
            image = self._station_image_array(message)
            stamp_s = self._stamp_s(message)
        except (AttributeError, TypeError, ValueError) as error:
            self._evidence_emit_failed('station_reference_image', error)
            return
        info = self.station_reference_rgb_info
        if info is None or info.header.frame_id != message.header.frame_id:
            return
        with self._station_reference_lock:
            self._station_reference_pending = (image, info, stamp_s)
        self._station_reference_wake.set()

    def _station_reference_loop(self):
        """Process at most the newest RGB frame on one managed worker."""
        while not self._station_reference_stop.is_set():
            self._station_reference_wake.wait(timeout=0.1)
            self._station_reference_wake.clear()
            if self._station_reference_stop.is_set():
                break
            with self._station_reference_lock:
                pending = self._station_reference_pending
                self._station_reference_pending = None
            if pending is None:
                continue
            started_s = time.monotonic()
            image, info, stamp_s = pending
            try:
                stamp = rclpy.time.Time(seconds=stamp_s)
                base = self.parking_contract['robot_base_frame']
                # RGB can arrive before its matching TF. This worker may wait
                # while the ROS callback thread continues filling the buffer.
                age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
                if not 0.0 <= age_s < STATION_REFERENCE_MAX_AGE_S:
                    raise ValueError('station reference image is stale or future-dated')
                wait = Duration(seconds=min(
                    STATION_REFERENCE_MIN_PERIOD_S,
                    STATION_REFERENCE_MAX_AGE_S - age_s))
                odom_transform = self.parking_tf.lookup_transform(
                    'odom', base, stamp, timeout=wait).transform
                map_from_odom = self.parking_tf.lookup_transform(
                    'map', 'odom', stamp, timeout=wait).transform
                odom_pose = (
                    odom_transform.translation.x,
                    odom_transform.translation.y,
                    self._yaw(odom_transform.rotation),
                )
                result = self.station_reference.observe(
                    image, info, odom_pose, stamp_s)
                if result.get('valid') is True:
                    goal = result['goal_odom']
                    turn = self._yaw(map_from_odom.rotation)
                    cosine, sine = math.cos(turn), math.sin(turn)
                    result = dict(result)
                    result['goal_map'] = [
                        map_from_odom.translation.x + cosine * goal[0] - sine * goal[1],
                        map_from_odom.translation.y + sine * goal[0] + cosine * goal[1],
                        math.atan2(math.sin(goal[2] + turn), math.cos(goal[2] + turn)),
                    ]
            except Exception as error:
                result = {
                    'valid': False,
                    'reason': f'station_reference_runtime:{type(error).__name__}',
                    'detail': str(error),
                    'stamp_s': stamp_s,
                }
            with self._station_reference_lock:
                self.station_reference_result = result
                self.station_reference_result_received_s = time.monotonic()
            remaining_s = STATION_REFERENCE_MIN_PERIOD_S - (
                time.monotonic() - started_s)
            if remaining_s > 0.0:
                self._station_reference_stop.wait(timeout=remaining_s)

    def configure_station_reference(self, profiles_path, table_id):
        """Select one map-bound target and its available lateral reference."""
        if profiles_path is None:
            return
        document = json.loads(Path(profiles_path).read_text(encoding='utf-8'))
        if not isinstance(document, dict) or document.get('schema_version') != 1:
            raise ValueError('station profiles require schema 1')
        for name in ('map', 'keepout'):
            expected = document.get(name)
            active = self.registry.get(name)
            if not isinstance(expected, dict) or not isinstance(active, dict):
                raise ValueError(f'station profiles {name} identity missing')
            for key in ('yaml_sha256', 'image_sha256'):
                if expected.get(key) != active.get(key):
                    raise ValueError(f'station profiles {name} {key} mismatch')
        stations = document.get('stations')
        profile = stations.get(table_id) if isinstance(stations, dict) else None
        if not isinstance(profile, dict):
            raise ValueError(f'station profile missing: {table_id}')
        if (profile.get('target_priority')
                != 'operator_taught_map_pose_and_heading'):
            raise ValueError('station target must retain operator taught pose priority')
        tolerance = profile.get('lateral_tolerance_m')
        if (profile.get('lateral_precision_status') != LATERAL_PRECISION_STATUS
                or isinstance(tolerance, bool)
                or not isinstance(tolerance, (int, float))
                or not math.isfinite(tolerance)
                or not math.isclose(tolerance, 0.005, abs_tol=1e-12)):
            raise ValueError('station lateral target must remain 0.005 m NOT_VALIDATED')
        table = next((item for item in self.registry.get('tables', ())
                      if item.get('table_id') == table_id), None)
        if table is None:
            raise ValueError(f'registered station missing: {table_id}')
        pose = next((item for item in table.get('service_poses', ())
                     if item.get('priority') == 1), None)
        if pose is None:
            raise ValueError(f'primary station pose missing: {table_id}')
        taught = tuple(pose.get(key) for key in ('x_m', 'y_m', 'yaw_rad'))
        if (any(isinstance(value, bool) or not isinstance(value, (int, float))
                for value in taught)
                or not all(math.isfinite(value) for value in taught)):
            raise ValueError('registered station pose must be finite')
        taught = tuple(float(value) for value in taught)
        rgbd = profile.get('rgbd_reference')
        mode = profile.get('reference_mode', 'rgbd_auxiliary')
        if mode not in ('rgbd', 'rgbd_auxiliary', 'map_pose_only'):
            raise ValueError(f'unsupported station reference mode: {mode}')
        if mode != 'map_pose_only' and not isinstance(rgbd, dict):
            raise ValueError(f'RGB-D station reference missing: {table_id}')
        if mode == 'map_pose_only' and rgbd is not None:
            raise ValueError('map-only station must not carry an RGB-D reference')
        reference = TaughtStationReference(rgbd) if mode != 'map_pose_only' else None
        self._station_reference_stop.set()
        self._station_reference_wake.set()
        worker = getattr(self, '_station_reference_thread', None)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.0)
            if worker.is_alive():
                raise RuntimeError('previous station reference worker did not stop')
        self._station_reference_thread = None
        with self._station_reference_lock:
            self._station_reference_pending = None
            self.station_reference_result = None
            self.station_reference_result_received_s = None
        self.station_reference_goal = None
        self.station_reference_goal_frozen = False
        self.station_reference = reference
        self.station_reference_mode = mode
        self.station_reference_profile = profile
        self.station_reference_taught_map_pose = taught
        self.station_reference_table_id = table_id
        if mode != 'map_pose_only':
            if self.station_reference_info_subscription is None:
                self.station_reference_info_subscription = self.create_subscription(
                    CameraInfo, '/camera/color/camera_info',
                    self._station_info_callback, qos_profile_sensor_data)
            if self.station_reference_image_subscription is None:
                self.station_reference_image_subscription = self.create_subscription(
                    Image, '/camera/color/image_raw',
                    self._station_image_callback, qos_profile_sensor_data)
            self._station_reference_stop.clear()
            self._station_reference_wake.clear()
            self._station_reference_thread = threading.Thread(
                target=self._station_reference_loop,
                name='station-rgbd-reference', daemon=True)
            self._station_reference_thread.start()
        self.emit(
            'station_reference_configured', table_id=table_id,
            target_role='operator_taught_map_pose_and_heading',
            reference_mode=mode,
            lateral_tolerance_m=0.005,
            lateral_precision_status=LATERAL_PRECISION_STATUS,
            millimeter_accuracy_validated=False)

    def _station_reference_enabled(self):
        return (getattr(self, 'station_reference', None) is not None
                or getattr(self, 'station_reference_mode', None) == 'map_pose_only')

    def _map_station_reference(self):
        """Project the taught map pose into odom using the current localization."""
        if not self._navigation_ready(require_fresh_amcl=True):
            return {'valid': False, 'reason': 'map_pose_localization_unavailable'}
        try:
            message = self.parking_tf.lookup_transform('odom', 'map', rclpy.time.Time())
        except TransformException as error:
            return {'valid': False, 'reason': 'map_pose_tf_lookup_failed',
                    'detail': str(error)}
        try:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            max_age = self.amcl_freshness_s
            if (not math.isfinite(stamp_s) or stamp_s <= 0.0
                    or not -max_age <= now_s - stamp_s <= max_age):
                raise ValueError('map to odom transform is stale')
            transform = message.transform
            translation, rotation = transform.translation, transform.rotation
            values = (translation.x, translation.y, rotation.x, rotation.y,
                      rotation.z, rotation.w)
            if (not all(math.isfinite(value) for value in values)
                    or sum(value * value for value in values[2:]) < 1e-12):
                raise ValueError('map to odom transform is invalid')
            yaw = self._yaw(rotation)
            x, y, heading = self.station_reference_taught_map_pose
            cosine, sine = math.cos(yaw), math.sin(yaw)
            goal = [translation.x + cosine * x - sine * y,
                    translation.y + sine * x + cosine * y,
                    math.atan2(math.sin(heading + yaw), math.cos(heading + yaw))]
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            return {'valid': False, 'reason': 'map_pose_transform_invalid',
                    'detail': str(error)}
        return {'valid': True, 'reason': 'registered_map_pose_using_amcl',
                'goal_map': list(self.station_reference_taught_map_pose),
                'goal_odom': goal, 'stamp_s': now_s,
                'reference_mode': 'map_pose_only'}

    def _fresh_station_reference(self):
        """Return a valid recent estimate; keep invalidity visible to callers."""
        if getattr(self, 'station_reference_mode', None) == 'map_pose_only':
            return self._map_station_reference()
        if getattr(self, 'station_reference', None) is None:
            return None
        with self._station_reference_lock:
            result = self.station_reference_result
            received_s = self.station_reference_result_received_s
        if not isinstance(result, dict) or received_s is None:
            return {'valid': False, 'reason': 'station_reference_not_observed'}
        if time.monotonic() - received_s > STATION_REFERENCE_MAX_AGE_S:
            return {'valid': False, 'reason': 'station_reference_stale'}
        now_s = self.get_clock().now().nanoseconds * 1e-9
        stamp_s = result.get('stamp_s')
        if (isinstance(stamp_s, bool) or not isinstance(stamp_s, (int, float))
                or not math.isfinite(stamp_s)
                or not 0.0 <= now_s - stamp_s <= STATION_REFERENCE_MAX_AGE_S):
            return {'valid': False, 'reason': 'station_reference_stale'}
        return result

    def _station_goal_reference(self):
        """Carry an accepted fixed goal in odom across rejected RGB frames.

        The stored object is a destination, never obstacle or sensor evidence.
        Its original stamp and current observation validity remain visible.
        """
        reference = self._fresh_station_reference()
        if (reference and reference.get('valid') is True
                and not getattr(self, 'station_reference_goal_frozen', False)):
            self.station_reference_goal = dict(reference)
        goal = getattr(self, 'station_reference_goal', None)
        return goal, reference

    def _station_anchor_target(self, target, *, reference_frame='map'):
        """Keep the measured normal/gap and select only the taught tangent."""
        if not self._station_reference_enabled():
            return target
        goal, reference = self._station_goal_reference()
        if reference_frame == 'odom':
            taught = goal.get('goal_odom') if goal else None
        else:
            taught = goal.get('goal_map') if goal else None
        if taught is None:
            if reference_frame == 'map':
                taught = self.station_reference_taught_map_pose
            else:
                return target
        anchored = anchor_face_to_taught_lateral(target, taught[:2])
        anchored['rgbd_reference_valid'] = bool(
            reference and reference.get('valid')
            and getattr(self, 'station_reference_mode', None) != 'map_pose_only')
        anchored['rgbd_reference_reason'] = (
            reference.get('reason') if reference else 'station_reference_not_observed')
        if getattr(self, 'station_reference_mode', None) == 'map_pose_only':
            anchored['lateral_reference_mode'] = 'map_pose_only'
        anchored['millimeter_accuracy_validated'] = False
        return anchored

    def _station_lateral_status(self):
        """Report only internal lateral evidence; external millimetres stay unvalidated."""
        if not self._station_reference_enabled():
            return {}
        goal_reference, reference = self._station_goal_reference()
        status = {
            'lateral_reference_valid': bool(
                reference and reference.get('valid') is True),
            'lateral_reference_reason': (
                reference.get('reason') if reference
                else 'station_reference_not_observed'),
            'lateral_tolerance_m': 0.005,
            'lateral_precision_status': LATERAL_PRECISION_STATUS,
            'millimeter_accuracy_validated': False,
            'internal_lateral_error_m': None,
            'internal_lateral_within_tolerance': False,
            'lateral_goal_valid': goal_reference is not None,
            'lateral_goal_basis': (
                ('registered_map_pose_in_odom'
                 if getattr(self, 'station_reference_mode', None) == 'map_pose_only'
                 else 'accepted_rgbd_goal_in_odom') if goal_reference is not None else None),
            'lateral_goal_observation_stamp_s': (
                goal_reference.get('stamp_s') if goal_reference else None),
        }
        if reference and reference.get('detail'):
            status['lateral_reference_detail'] = reference['detail']
        if goal_reference is None or not status['lateral_reference_valid']:
            return status
        try:
            actual = self._odom_pose()
            goal = reference['goal_odom']
            left = (-math.sin(goal[2]), math.cos(goal[2]))
            error_m = ((goal[0] - actual[0]) * left[0]
                       + (goal[1] - actual[1]) * left[1])
        except (KeyError, RuntimeError, TypeError, ValueError):
            status['lateral_reference_valid'] = False
            status['lateral_goal_valid'] = False
            status['lateral_reference_reason'] = 'station_reference_final_pose_unavailable'
            return status
        status['internal_lateral_error_m'] = error_m
        status['internal_lateral_within_tolerance'] = abs(error_m) <= 0.005
        return status

    def _retry_station_lateral_reference(self, status, motion_revision):
        """Retry a missing lateral observation briefly while the base remains stopped."""
        if (not status or status.get('lateral_reference_valid') is True
                or (getattr(self, 'station_reference', None) is None
                    and getattr(self, 'station_reference_mode', None) != 'map_pose_only')):
            return status, False
        deadline = time.monotonic() + 1.0
        task_deadline = getattr(self, 'run_deadline_s', None)
        if task_deadline is not None:
            deadline = min(deadline, task_deadline)
        while time.monotonic() < deadline and not self.stop_requested:
            spin_node(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            if getattr(self, 'parking_motion_revision', None) != motion_revision:
                return status, True
            status = self._station_lateral_status()
            if status.get('lateral_reference_valid') is True:
                break
        return status, False

    def destroy_node(self):
        """Stop the station reference worker before destroying ROS entities."""
        stop = getattr(self, '_station_reference_stop', None)
        wake = getattr(self, '_station_reference_wake', None)
        if stop is not None:
            stop.set()
        if wake is not None:
            wake.set()
        worker = getattr(self, '_station_reference_thread', None)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.0)
            if worker.is_alive():
                raise RuntimeError('station reference worker did not stop')
        return super().destroy_node()

    def _go_home_reverse(self, pose, execute, _map_reapproach=False):
        """Use the registered staging pose, then the configured rear reference."""
        if pose.get('rear_parking_reference', 'map') != 'lidar_box':
            return super()._go_home_reverse(
                pose, execute, _map_reapproach=_map_reapproach)
        config = route_config(self.registry, pose)
        stage = self._rear_reference_stage(config['waypoints'][0])
        config['waypoints'][0] = stage
        self.config, self.waypoints = config, config['waypoints']
        stage_pose = {**pose, 'id': stage['id'], 'x_m': stage['x'], 'y_m': stage['y'],
                      'yaw_rad': stage['yaw'],
                      'parking_direction': 'forward'}
        try:
            # A box may occupy the old map-coordinate endpoint. Plan to the
            # staging pose; the final leg observes and stops before that box.
            with self._localization_bound(True):
                planned = self._plan_with_input_recovery(
                    stage_pose, single=True,
                    end_tolerance_m=TRANSIT_PLAN_END_TOLERANCE_M)
                self.emit('reverse_staging_planned', attempt=planned,
                          rear_parking_reference='lidar_box')
                if not planned['ok']:
                    self.emit('failed', phase='home_staging', reason='home_staging_not_planned')
                    return False
                if not execute:
                    self.emit('home_planned_only', parking_direction='reverse',
                              rear_parking_reference='lidar_box',
                              final_path_validation='LIVE_REAR_LIDAR_AT_STAGING')
                    return True
                if self._reach_staging(stage) is None:
                    return False
            self.pose_id = pose['id']
            self.confirmation = None
            remaining_s = self.run_deadline_s - time.monotonic()
            reason = ('operator_stop' if self.stop_requested else
                      'operator_link_lost' if not self._operator_link_ready() else
                      self._guard_failure(False))
            if reason or remaining_s <= 0.0:
                self.emit('failed', phase='home_rear_box',
                          reason=reason or 'home_return_timeout')
                return False
            success = self.park_rear_box(
                self.home_rear_geometry, self.home_rear_route_path,
                execute=True, timeout_s=remaining_s,
                target_gap_m=pose['target_rear_gap_m'])
            self.pose_id = pose['id']
            self.emit('home_arrived' if success else 'failed',
                      phase='home_rear_box', parking_direction='reverse',
                      rear_parking_reference='lidar_box', confirmation=self.confirmation,
                      reason=None if success else 'rear_box_parking_not_confirmed')
            return success
        finally:
            self.config, self.waypoints = config, config['waypoints']
            self.pose_id = pose['id']

    def _observer_stable_frame_quorum(self):
        """Read the observer's active stable-frame quorum once for this observation."""
        client = getattr(self, 'observer_parameters', None)
        if client is None:
            client = AsyncParameterClient(self, 'jdamr_depth_box_parking')
            self.observer_parameters = client
        try:
            if not client.wait_for_services(timeout_sec=1.0):
                return None
            response = self._wait(client.get_parameters(['stable_frames']), 2.0)
            values = response.values
            if len(values) != 1:
                return None
            value = parameter_value_to_python(values[0])
        except (RuntimeError, AttributeError, TypeError, ValueError):
            return None
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            return None
        return value

    def _camera(self, on):
        """
        Start or stop the box camera and observer when they run on demand.

        Returns whether systemctl accepted the request; always True when the camera
        stays on (camera_on_demand unset, as before 2026-10-06).
        """
        if not getattr(self, 'camera_on_demand', False):
            return True
        if on and getattr(self, '_camera_commanded_on', False):
            return True
        action = 'start' if on else 'stop'
        if on:
            self._camera_requested_s = time.monotonic()
        try:
            result = subprocess.run(['sudo', '-n', 'systemctl', '--no-block',
                                     action, *CAMERA_UNITS],
                                    capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired) as error:
            self.emit('box_camera', action=action, ok=False, reason=str(error))
            return False
        ok = result.returncode == 0
        if ok:
            self._camera_commanded_on = on
        self.emit('box_camera', action=action, ok=ok,
                  reason=None if ok else result.stderr.strip()[-200:])
        return ok

    def _camera_ready(self, timeout_s=CAMERA_READY_TIMEOUT_S):
        """Wait until the observer published after the camera was requested."""
        if not getattr(self, 'camera_on_demand', False):
            return True
        requested_s = getattr(self, '_camera_requested_s', None)
        if requested_s is None:
            if not self._camera(True):
                return False
            requested_s = self._camera_requested_s
        wait_started_s = time.monotonic()
        deadline_s = wait_started_s + timeout_s
        while not self.stop_requested and time.monotonic() < deadline_s:
            received_s = getattr(self, 'box_status_received_s', None)
            if received_s is not None and received_s >= requested_s:
                ready_s = time.monotonic()
                self.emit('box_camera_ready',
                          # Preserve the legacy elapsed time, which includes transit.
                          waited_s=round(ready_s - requested_s, 1),
                          blocking_wait_s=round(ready_s - wait_started_s, 3))
                return True
            spin_node(self, timeout_sec=0.1)
        self.emit('box_camera_ready', ready=False, timeout_s=timeout_s)
        return False

    def _scan_callback(self, message):
        super()._scan_callback(message)
        self.last_scan = message
        self._remember_scan(message)

    def _box_callback(self, message):
        super()._box_callback(message)
        # Recent statuses are failure evidence only, never an observation.
        history = getattr(self, 'box_status_history', None)
        if history is None:
            history = self.box_status_history = deque(maxlen=16)
        history.append((time.monotonic(), self.box_status))

    def _depth_info_callback(self, message):
        self.depth_camera_info = message
        self.depth_camera_info_received_s = time.monotonic()

    def _fresh_depth_frame_after(self, stamp_s, now_s):
        # Astra publishes CameraInfo with the same stamp as each depth image.
        info = getattr(self, 'depth_camera_info', None)
        received = getattr(self, 'depth_camera_info_received_s', None)
        if (info is None or received is None
                or not 0.0 <= time.monotonic() - received <= 0.5):
            return False
        frame_stamp = info.header.stamp.sec + info.header.stamp.nanosec * 1e-9
        return (info.header.frame_id == 'camera_color_optical_frame'
                and math.isfinite(frame_stamp) and frame_stamp >= stamp_s
                and 0.0 <= now_s - frame_stamp <= 0.5)

    def _evidence_emit_failed(self, source, error):
        """Report a diagnostic failure without changing the control flow."""
        try:
            self.emit('evidence_emit_failed', source=source,
                      error_type=type(error).__name__)
        except Exception:
            pass

    def _ensure_depth_info(self):
        """Keep the per-image depth stamp available throughout an approach."""
        try:
            subscription = getattr(self, 'depth_info_subscription', None)
            if subscription is None:
                self.depth_info_subscription = self.create_subscription(
                    CameraInfo, '/camera/depth/camera_info',
                    self._depth_info_callback, qos_profile_sensor_data)
        except Exception as error:
            self._evidence_emit_failed('depth_camera_info', error)

    def _observation_pose_diagnostic(self, robot_pose, camera_mount, region_xy):
        """Record the camera bearing to the table region; never aim or judge."""
        # The search reads only the latest bearing; a failed diagnostic leaves none.
        self.last_region_bearing_error_rad = None
        self.last_region_distance_m = None
        try:
            self._ensure_depth_info()
            info = getattr(self, 'depth_camera_info', None)
            half_fov_rad = None
            if info is not None:
                fx, width = float(info.p[0]), float(info.width)
                if math.isfinite(fx) and fx > 0.0 and width > 0.0:
                    half_fov_rad = math.atan2(width / 2.0, fx)
            fields = {'robot_pose': list(robot_pose), 'region_xy': list(region_xy),
                      'half_fov_rad': half_fov_rad, 'depth_minimum_m': MINIMUM_DISTANCE_M}
            fixed_heading = getattr(self, '_known_table_heading_rad', None)
            self.last_table_heading_error_rad = (
                None if fixed_heading is None else math.atan2(
                    math.sin(fixed_heading - robot_pose[2]),
                    math.cos(fixed_heading - robot_pose[2])))
            try:
                mount_x, mount_y, _z, _roll, _pitch, mount_yaw = _validated_mount(
                    camera_mount)
            except ValueError as error:
                fields['mount_invalid'] = str(error)
            else:
                x, y, yaw = robot_pose
                camera_xy = (x + mount_x * math.cos(yaw) - mount_y * math.sin(yaw),
                             y + mount_x * math.sin(yaw) + mount_y * math.cos(yaw))
                camera_yaw = yaw + mount_yaw
                bearing = math.atan2(region_xy[1] - camera_xy[1],
                                     region_xy[0] - camera_xy[0])
                bearing_error = math.atan2(math.sin(bearing - camera_yaw),
                                           math.cos(bearing - camera_yaw))
                distance_m = math.dist(camera_xy, region_xy)
                self.last_region_bearing_error_rad = bearing_error
                self.last_region_distance_m = distance_m
                fields.update(
                    camera_xy=list(camera_xy), camera_yaw_rad=camera_yaw,
                    bearing_error_rad=bearing_error, camera_region_distance_m=distance_m,
                    in_fov=(None if half_fov_rad is None
                            else abs(bearing_error) <= half_fov_rad),
                    region_inside_depth_minimum=distance_m < MINIMUM_DISTANCE_M)
            self.emit('box_observation_pose_diagnostic', **self._json_evidence(fields))
        except Exception as error:
            self._evidence_emit_failed('box_observation_pose_diagnostic', error)

    def _emit_observation_unavailable_evidence(self, **fields):
        """Record the inputs behind an unavailable observation, once per window."""
        try:
            scan = getattr(self, 'last_scan', None)
            scan_document = None
            if scan is not None:
                ranges, truncated = self._json_scan_ranges(scan.ranges)
                scan_document = {
                    'frame_id': scan.header.frame_id,
                    'stamp_s': scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
                    'angle_min': self._json_scalar(scan.angle_min),
                    'angle_increment': self._json_scalar(scan.angle_increment),
                    'range_min': self._json_scalar(scan.range_min),
                    'range_max': self._json_scalar(scan.range_max),
                    'ranges': ranges, 'ranges_truncated': truncated,
                }
            history = list(getattr(self, 'box_status_history', None) or ())
            self.emit('box_observation_unavailable_evidence', **self._json_evidence({
                **fields, 'status_history': history, 'scan': scan_document}))
        except Exception as error:
            self._evidence_emit_failed('box_observation_unavailable_evidence', error)

    def preflight(self):
        """Also require the transit plan to end at the observation waypoint."""
        self.preflight_end_error_m = None
        if not super().preflight():
            return False
        goal = self.waypoints[-1]
        end_error_m = math.dist(self.preflight_path_end_xy, (goal['x'], goal['y']))
        if not math.isfinite(end_error_m) or end_error_m > TRANSIT_PLAN_END_TOLERANCE_M:
            self.preflight_end_error_m = end_error_m if math.isfinite(end_error_m) else None
            self.get_logger().error(
                'route preflight ends short of the observation waypoint: '
                f'end_error={end_error_m:.3f}m limit={TRANSIT_PLAN_END_TOLERANCE_M:.3f}m')
            return False
        return True

    @staticmethod
    def _json_scan_ranges(ranges, limit=9000):
        """Bound one raw scan and replace non-finite values for JSON evidence."""
        values = list(islice(iter(ranges), limit + 1))
        return [
            (float(value) if not isinstance(value, bool)
             and isinstance(value, (int, float)) and math.isfinite(value)
             else None)
            for value in values[:limit]
        ], len(values) > limit

    @staticmethod
    def _json_scalar(value):
        """Return one finite numeric scalar suitable for strict JSON."""
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            return None
        return float(value)

    @classmethod
    def _json_evidence(cls, value):
        """Normalize nested evidence without changing its measured content."""
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, dict):
            return {str(key): cls._json_evidence(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_evidence(item) for item in value]
        isoformat = getattr(value, 'isoformat', None)
        if callable(isoformat):
            return isoformat()
        return str(value)

    @staticmethod
    def _polygon(value):
        try:
            polygon = json.loads(value) if isinstance(value, str) else value
        except (TypeError, json.JSONDecodeError) as error:
            raise ValueError('collision polygon is not parseable') from error
        if (not isinstance(polygon, (list, tuple)) or len(polygon) < 3
                or any(not isinstance(point, (list, tuple)) or len(point) != 2
                       or any(isinstance(item, bool)
                              or not isinstance(item, (int, float))
                              or not math.isfinite(item) for item in point)
                       for point in polygon)):
            raise ValueError('collision polygon is invalid')
        return [[float(item) for item in point] for point in polygon]

    def _precision_collision_ready(self, geometry):
        """Match the live safety node to the bounded precision profile."""
        package = Path(get_package_share_directory('jdamr_cube_navigation'))
        baseline = yaml.safe_load((
            package / 'config/new_base_nav2_params.yaml').read_text())
        expected_document = apply_docking_stop_profile(baseline, geometry)
        monitor = expected_document['collision_monitor']['ros__parameters']
        names = [
            'polygons', 'observation_sources', 'source_timeout',
            'StopZone.enabled', 'StopZone.action_type', 'StopZone.min_points',
            'StopZone.velocity_polygons',
            'StopZone.translation_forward.points', 'StopZone.stopped.points',
            'StopZone.translation_backward.points',
            'StopZone.translation_forward_straight.points',
            'StopZone.translation_backward_straight.points',
            'StopZone.rotation.points', 'StopZone.rotation_clockwise.points',
            'SlowdownZone.enabled',
            'scan.enabled', 'scan.type', 'scan.topic',
            'FootprintApproach.enabled', 'FootprintApproach.type',
            'FootprintApproach.min_points',
            'FootprintApproach.time_before_collision',
            'FootprintApproach.footprint_topic',
        ]

        def nested(name):
            value = monitor
            for component in name.split('.'):
                value = value[component]
            return value

        expected = [nested(name) for name in names]
        client = getattr(self, 'precision_parameters', None)
        if client is None:
            client = AsyncParameterClient(self, 'collision_monitor')
            self.precision_parameters = client
        if not client.wait_for_services(timeout_sec=2.0):
            return False
        try:
            response = self._wait(client.get_parameters(names), 2.0)
        except RuntimeError:
            return False
        if len(response.values) != len(expected):
            return False
        for name, wanted, value in zip(names, expected, response.values):
            actual = parameter_value_to_python(value)
            if name.endswith('.points'):
                try:
                    if self._polygon(actual) != self._polygon(wanted):
                        return False
                except ValueError:
                    return False
            elif isinstance(wanted, float):
                if (isinstance(actual, bool)
                        or not isinstance(actual, (int, float))
                        or not math.isclose(
                            actual, wanted, rel_tol=0.0, abs_tol=1e-9)):
                    return False
            elif type(actual) is not type(wanted) or actual != wanted:
                return False
        return True

    def _configure_observer(self, camera_mount, region_radius_m):
        """
        Atomically select the surface detector, live mount and bounded depth range.

        The detector keeps the largest plane in view. At table_01 a 1 m wide
        surface 1.33 m away won over the box 0.64 m away for all 12 search
        turns (2026-10-01 15:37); beyond the region there is no box to see.
        """
        _x, _y, height_m, roll_rad, pitch_rad, _yaw = _validated_mount(camera_mount)
        surface_mode = getattr(self, 'surface_mode', 'front')
        if surface_mode not in SURFACE_MODES:
            raise ValueError('surface mode must be front or table_edge')
        distance_m = getattr(self, 'last_region_distance_m', None)
        window_m = DEPTH_WINDOW_CEILING_M if distance_m is None else min(
            DEPTH_WINDOW_CEILING_M, max(MINIMUM_DISTANCE_M + 0.1, distance_m + region_radius_m))
        reason = None
        try:
            client = getattr(self, 'observer_parameters', None)
            if client is None:
                client = AsyncParameterClient(self, 'jdamr_depth_box_parking')
                self.observer_parameters = client
            if not client.wait_for_services(timeout_sec=1.0):
                raise RuntimeError('observer parameter service unavailable')
            observer_parameters = [
                Parameter('maximum_depth_m', value=float(window_m)),
                Parameter('surface_mode', value=surface_mode),
                Parameter('camera_height_m', value=height_m),
                Parameter('camera_roll_rad', value=roll_rad),
                Parameter('camera_pitch_rad', value=pitch_rad),
            ]
            if surface_mode == 'table_edge':
                enabled = bool(getattr(self, 'pointcloud_perception', False))
                region = getattr(self, '_pointcloud_target_region_map', None) if enabled else None
                if enabled and region is None:
                    raise RuntimeError('pointcloud target region has not been selected')
                observer_parameters.extend([
                    Parameter('table_request_enabled', value=enabled),
                    Parameter(
                        'table_target_region_map',
                        value=json.dumps(region) if enabled else ''),
                    Parameter('table_calibration_id', value=hashlib.sha256(
                        json.dumps(camera_mount, sort_keys=True).encode()).hexdigest()
                              if enabled else ''),
                ])
            response = self._wait(
                client.set_parameters_atomically(observer_parameters), 2.0).result
            if response.successful is not True:
                raise RuntimeError(response.reason or 'observer parameters rejected')
        except Exception as error:
            reason = str(error) or type(error).__name__
        self.emit('box_observer_configuration', maximum_depth_m=window_m,
                  surface_mode=surface_mode, camera_height_m=height_m,
                  camera_roll_rad=roll_rad, camera_pitch_rad=pitch_rad,
                  region_distance_m=distance_m, applied=reason is None, reason=reason)
        return reason is None

    def _set_depth_window(self, region_radius_m):
        """Apply the complete observer configuration for the current mount."""
        return self._configure_observer(self._observer_camera_mount, region_radius_m)

    def observe_target(self, camera_mount, front_extent_m, gap_m, region_xy,
                       region_radius_m, timeout_s=12.0, *, reuse_configuration=False):
        """
        Capture a stationary robot and a new stable surface, never a cached one.

        One recoverable input gap restarts the window from a new stationary
        capture. Each observer stamp is judged once. A verdict with fewer than
        stable_frames fresh frames after an input delay is latency-limited.
        """
        if getattr(self, 'candidate_trial', False) is not True:
            raise RuntimeError(
                'box observation consumption requires explicit candidate trial')
        call_started_s = time.monotonic()
        self._ensure_depth_info()
        robot_pose, _ = self.capture_stationary_pose(intermediate=True)
        self._observation_pose_diagnostic(robot_pose, camera_mount, region_xy)
        self._observer_camera_mount = camera_mount
        if getattr(self, 'pointcloud_perception', False):
            selected_region = [float(region_xy[0]), float(region_xy[1]), float(region_radius_m)]
            if selected_region != getattr(self, '_pointcloud_target_region_map', None):
                reuse_configuration = False
            self._pointcloud_target_region_map = selected_region
        if (not reuse_configuration and not self._set_depth_window(region_radius_m)
                and (getattr(self, 'surface_mode', 'front') == 'table_edge'
                     or getattr(self, 'require_observer_configuration', False))):
            raise RuntimeError('observer surface and camera parameters unavailable')
        runtime_stable_frames = self._observer_stable_frame_quorum()
        latency_stable_frames = runtime_stable_frames
        if latency_stable_frames is None:
            latency_stable_frames = getattr(self, 'observer_stable_frames', None)
        revision = self.parking_motion_revision
        cutoff_s = self.get_clock().now().nanoseconds * 1e-9
        # A later cutoff advances by elapsed monotonic time, not another clock read.
        cutoff_monotonic_s = time.monotonic()
        deadline_s = cutoff_monotonic_s + timeout_s
        last_failure = 'no_stable_box'
        last_failure_retryable = False
        rejected_observation_stamps = set()
        input_recovered = False
        latency_reasons = set()
        fresh_since_latency = 0
        fresh_rejection_reasons = set()
        evaluated_stamps = set()
        counted_rejection_s = None

        def emit_evidence(reason, retryable):
            latency_limited = bool(latency_reasons) and (
                latency_stable_frames is None
                or fresh_since_latency < latency_stable_frames)
            self._emit_observation_unavailable_evidence(
                reason=reason, retryable=retryable and not latency_limited,
                latency_limited=latency_limited, latency_reasons=sorted(latency_reasons),
                fresh_since_latency=fresh_since_latency, cutoff_s=cutoff_s,
                robot_pose=list(robot_pose), camera_mount=camera_mount,
                region_xy=list(region_xy))
            return latency_limited

        while not self.stop_requested and time.monotonic() < deadline_s:
            spin_node(self, timeout_sec=0.05)
            if not self._navigation_ready(require_fresh_amcl=False):
                reason = self._guard_failure(False)
                if (not input_recovered and not self.stop_requested
                        and self._input_gap_recoverable(reason)
                        and self._wait_for_input_recovery(reason)):
                    # Nothing observed before the gap may form the verdict.
                    input_recovered = True
                    latency_reasons.add('input_gap')
                    fresh_since_latency = 0
                    fresh_rejection_reasons.clear()
                    robot_pose, _ = self.capture_stationary_pose(intermediate=True)
                    self._observation_pose_diagnostic(robot_pose, camera_mount, region_xy)
                    self._observer_camera_mount = camera_mount
                    if (not self._set_depth_window(region_radius_m)
                            and (getattr(self, 'surface_mode', 'front') == 'table_edge'
                                 or getattr(
                                     self, 'require_observer_configuration', False))):
                        raise RuntimeError(
                            'observer surface and camera parameters unavailable')
                    revision = self.parking_motion_revision
                    now_monotonic_s = time.monotonic()
                    cutoff_s += now_monotonic_s - cutoff_monotonic_s
                    cutoff_monotonic_s = now_monotonic_s
                    deadline_s = now_monotonic_s + timeout_s
                    last_failure = 'no_stable_box'
                    last_failure_retryable = False
                    continue
                emit_evidence(reason, False)
                raise RuntimeError(reason)
            if self.parking_motion_revision != revision:
                raise RuntimeError('robot moved during stationary box observation')
            observation = self.box_status
            received_s = getattr(self, 'box_status_received_s', None)
            if (isinstance(observation, dict) and received_s is not None
                    and received_s >= call_started_s and received_s != counted_rejection_s
                    and isinstance(observation.get('reason'), str)
                    and observation['reason'] in OBSERVER_LATENCY_REASONS):
                counted_rejection_s = received_s
                latency_reasons.add(observation['reason'])
                fresh_since_latency = 0
                fresh_rejection_reasons.clear()
            silent_since_s = (call_started_s if received_s is None
                              else max(call_started_s, received_s))
            if time.monotonic() - silent_since_s > OBSERVER_SILENT_S:
                latency_reasons.add('observer_silent')
                fresh_since_latency = 0
                fresh_rejection_reasons.clear()
            if (not observation or observation.get('frame_id') != 'camera_color_optical_frame'
                    or not isinstance(observation.get('stamp_s'), (int, float))
                    or observation['stamp_s'] <= cutoff_s):
                last_failure_retryable = False
                continue
            # Waiting for the next frame is not latency: judge each stamp once.
            if observation['stamp_s'] in evaluated_stamps:
                continue
            evaluated_stamps.add(observation['stamp_s'])
            try:
                retryable = False
                now_s = self.get_clock().now().nanoseconds * 1e-9
                age_s = now_s - observation['stamp_s']
                if (isinstance(observation['stamp_s'], bool)
                        or not math.isfinite(age_s) or not 0.0 <= age_s <= 0.5):
                    raise ValueError('box observation is stale or future-dated')
                if observation.get('control_ready') is not False:
                    raise ValueError(
                        'observer must remain perception-only; candidate gate is separate')
                surface_mode = getattr(self, 'surface_mode', 'front')
                retryable = (
                    observation.get('detected') is not True
                    or observation.get('stable') is not True
                    or observation.get('surface_kind') != surface_mode)
                robot_pose_document = dict(zip(
                    ('x_m', 'y_m', 'yaw_rad'), robot_pose))
                if surface_mode == 'table_edge':
                    target = compute_table_edge_docking_target(
                        observation, camera_mount, robot_pose_document,
                        requested_gap_m=gap_m, front_extent_m=front_extent_m,
                        now_s=now_s)
                    target.update({
                        'front_extent_m': front_extent_m,
                        'observed_face_span_m': observation.get('width_m'),
                        'observation_robot_pose_map': list(robot_pose),
                        'front_extent_provenance': self._json_evidence(
                            getattr(self, 'front_extent_provenance', None)),
                        'physical_accuracy': 'NOT_EXTERNALLY_MEASURED',
                    })
                    retryable = False
                    if self.parking_motion_revision != revision:
                        raise ValueError('robot moved during table edge observation')
                    retryable = True
                    if math.dist(target['face_center_map_xy_m'], region_xy) > region_radius_m:
                        raise ValueError('observed_table_edge_outside_selected_table_region')
                    target = self._station_anchor_target(target)
                    self.emit(
                        'box_target_observed', observation=observation, target=target,
                        surface_mode=surface_mode, target_basis='tabletop_front_edge',
                        lidar_face_fusion=False, candidate_trial=True,
                        physical_accuracy='NOT_EXTERNALLY_MEASURED')
                    return target
                target = compute_box_docking_target(
                    observation, camera_mount, robot_pose_document,
                    requested_gap_m=gap_m, front_extent_m=front_extent_m,
                    now_s=now_s)
                target.update({
                    'front_extent_m': front_extent_m,
                    'front_extent_provenance': self._json_evidence(
                        getattr(self, 'front_extent_provenance', None)),
                })
                retryable = False
                scan = self.last_scan
                if scan is None:
                    raise ValueError('fresh LiDAR scan is required for box witness')
                scan_stamp_s = (scan.header.stamp.sec
                                + scan.header.stamp.nanosec * 1e-9)
                scan_age_s = now_s - scan_stamp_s
                if (not math.isfinite(scan_stamp_s) or scan_age_s < 0.0
                        or scan_age_s > 0.5):
                    raise ValueError('LiDAR scan is stale or future-dated')
                diagnostics = {}
                try:
                    scan_frame = scan.header.frame_id
                    if scan_frame != 'laser_link':
                        raise ValueError(
                            'LiDAR scan frame must be measured laser_link')
                    retryable = True
                    witness = witness_box_face_with_lidar(
                        scan.ranges, angle_min=scan.angle_min,
                        angle_increment=scan.angle_increment,
                        range_min=scan.range_min, range_max=scan.range_max,
                        geometry=self.box_geometry, depth_target=target,
                        robot_pose=robot_pose_document,
                        diagnostics=diagnostics)
                except ValueError as error:
                    last_failure = str(error)
                    last_failure_retryable = (
                        retryable and last_failure in SEARCH_RECOVERABLE_REASONS)
                    fresh_since_latency += 1
                    observation_stamp = observation['stamp_s']
                    if observation_stamp not in rejected_observation_stamps:
                        rejected_observation_stamps.add(observation_stamp)
                        raw_ranges, ranges_truncated = self._json_scan_ranges(
                            scan.ranges)
                        self.emit(
                            'box_lidar_witness_rejected',
                            observation=self._json_evidence(observation),
                            target=self._json_evidence(target),
                            robot_pose=self._json_evidence(
                                robot_pose_document),
                            camera_mount=self._json_evidence(camera_mount),
                            box_geometry=self._json_evidence(
                                self.box_geometry),
                            scan_frame=scan_frame,
                            scan_stamp_s=scan_stamp_s,
                            scan_angle_min=self._json_scalar(scan.angle_min),
                            scan_angle_increment=self._json_scalar(
                                scan.angle_increment),
                            scan_range_min=self._json_scalar(scan.range_min),
                            scan_range_max=self._json_scalar(scan.range_max),
                            scan_ranges=raw_ranges,
                            scan_ranges_truncated=ranges_truncated,
                            diagnostics=self._json_evidence(diagnostics),
                            reason=last_failure)
                    continue
                face = witness['fused_face_center_map_xy_m']
                outward = witness['outward_normal_map_xy']
                center_offset_m = front_extent_m + gap_m
                target.update({
                    'x_m': face[0] + outward[0] * center_offset_m,
                    'y_m': face[1] + outward[1] * center_offset_m,
                    'yaw_rad': math.atan2(-outward[1], -outward[0]),
                    'estimate_gap_m': gap_m,
                    'face_center_map_xy_m': face,
                    'outward_normal_map_xy': outward,
                    'depth_target_provenance': target['provenance'],
                    'lidar_witness': witness,
                    'candidate_trial': True,
                    'physical_accuracy': 'NOT_EXTERNALLY_MEASURED',
                })
                retryable = False
                if self.parking_motion_revision != revision:
                    raise ValueError('robot moved during box sensor witness')
                retryable = True
                if math.dist(target['face_center_map_xy_m'], region_xy) > region_radius_m:
                    raise ValueError('observed_box_outside_selected_table_region')
                self.emit('box_target_observed', observation=observation,
                          target=target, surface_mode=surface_mode,
                          target_basis='lidar_depth_front_face',
                          lidar_face_fusion=True, candidate_trial=True,
                          physical_accuracy='NOT_EXTERNALLY_MEASURED')
                return target
            except ValueError as error:
                last_failure = str(error)
                last_failure_retryable = (
                    retryable and last_failure in SEARCH_RECOVERABLE_REASONS)
                if last_failure in BOX_INPUT_LATENCY_FAILURES:
                    latency_reasons.add(last_failure)
                    fresh_since_latency = 0
                    fresh_rejection_reasons.clear()
                else:
                    fresh_since_latency += 1
                    rejection_reason = observation.get('reason')
                    if isinstance(rejection_reason, str):
                        fresh_rejection_reasons.add(rejection_reason)
                    if (getattr(self, 'surface_mode', 'front') == 'table_edge'
                            and getattr(self, '_known_table_heading_rad', None) is not None
                            and isinstance(runtime_stable_frames, int)
                            and runtime_stable_frames > 0
                            and fresh_since_latency >= runtime_stable_frames
                            and fresh_rejection_reasons == {'no_table_edge_candidate'}):
                        # A full fresh quorum with no candidate is scene evidence,
                        # not a reason to sit through the remaining timeout.
                        break
        latency_limited = emit_evidence(last_failure, last_failure_retryable)
        raise BoxObservationUnavailable(
            last_failure, retryable=last_failure_retryable and not latency_limited,
            latency_limited=latency_limited, fresh_frames=fresh_since_latency,
            required_fresh_frames=runtime_stable_frames,
            fresh_rejection_reasons=fresh_rejection_reasons,
            latency_reasons=latency_reasons)

    def _reposition_search(self):
        """Try one planned nearby view after a collision or timeout Spin result."""
        if getattr(self, 'surface_mode', 'front') == 'table_edge':
            # A missing mapped table is not evidence that the base is too close.
            # Finish the mapped observation approach; never back away to search.
            return self._advance_to_table_view(require_motion=True)
        if (self.stop_requested or not self._navigation_ready(require_fresh_amcl=False)
                or getattr(self, 'last_search_error_code', None) not in (
                    Spin.Result.COLLISION_AHEAD, Spin.Result.TIMEOUT)):
            return False
        actual, _ = self.capture_stationary_pose()
        # These are candidate waypoints, not open-loop displacement commands.
        # Nav2 must find a valid path through the current footprint/keepout map.
        for offset_m in (0.2, -0.2):
            candidate = {
                'id': 'search_view', 'priority': 1, 'approach_offset_m': 0.5,
                'x_m': actual[0] + offset_m * math.cos(actual[2]),
                'y_m': actual[1] + offset_m * math.sin(actual[2]),
                'yaw_rad': actual[2],
            }
            # Judged by the alignment goal checker, like face alignment.
            planned = self.plan_pose(
                candidate, single=True,
                end_tolerance_m=MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'])
            if not planned['ok']:
                continue
            self.emit('box_search_reposition', candidate=candidate)
            return self.execute(final_parking=False, alignment=True, face=True)
        return False

    def _advance_to_table_view(self, *, require_motion=False):
        """Close a transit waypoint's tolerance before waiting for a table face."""
        mount, region, _radius = self._table_observation_context
        offset_m = mount.get('camera_mount', {}).get('table_observation', {}).get(
            'base_offset_m')
        heading_rad = getattr(self, '_known_table_heading_rad', None)
        if offset_m is None or heading_rad is None:
            return not require_motion
        if (isinstance(offset_m, bool) or not isinstance(offset_m, (int, float))
                or not math.isfinite(offset_m) or not .2 <= offset_m <= ESCAPE_CLEARANCE_M
                or isinstance(heading_rad, bool) or not math.isfinite(heading_rad)):
            raise ValueError('invalid mapped table observation geometry')
        if (self.stop_requested or not self._operator_link_ready()
                or not self._navigation_ready(require_fresh_amcl=False)):
            return False
        actual, _ = self.capture_stationary_pose(intermediate=True)
        observation = getattr(self, 'box_status', None)
        if (not require_motion and isinstance(observation, dict)
                and observation.get('detected') is True
                and observation.get('stable') is True
                and observation.get('surface_kind') == 'table_edge'):
            try:
                target = compute_table_edge_docking_target(
                    observation, mount, dict(zip(('x_m', 'y_m', 'yaw_rad'), actual)),
                    requested_gap_m=self._final_gap_m(),
                    front_extent_m=self.box_geometry['front_to_wheel_axis']['value'],
                    now_s=self.get_clock().now().nanoseconds * 1e-9)
                if math.dist(target['face_center_map_xy_m'], region) <= _radius:
                    # A visible face already permits the sensor-controlled handoff.
                    return True
            except (ValueError, KeyError, TypeError):
                pass
        forward = (math.cos(heading_rad), math.sin(heading_rad))
        clearance_m = sum((region[i] - actual[i]) * forward[i] for i in (0, 1))
        remaining_m = clearance_m - offset_m
        if remaining_m <= .01:
            return not require_motion
        residual_rad = math.atan2(math.sin(heading_rad - actual[2]),
                                  math.cos(heading_rad - actual[2]))
        # Heading recovery uses the existing bounded Spin; it cannot reverse.
        if abs(residual_rad) > FINAL_TRIM_MAX_RAD:
            return not require_motion
        distance_m = remaining_m / math.cos(residual_rad)
        if distance_m > LIVE_APPROACH_MAX_M:
            raise RuntimeError('mapped table observation exceeds bounded approach')
        end = (actual[0] + distance_m * math.cos(actual[2]),
               actual[1] + distance_m * math.sin(actual[2]), actual[2])
        lateral_m = abs((region[0] - end[0]) * -forward[1]
                        + (region[1] - end[1]) * forward[0])
        if lateral_m > _radius:
            return not require_motion
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())
        if not check([actual, end]):
            self.emit('table_view_approach_blocked', reason='static_corridor_blocked')
            return False
        self.emit('table_view_approach_started', motion='forward_same_heading',
                  distance_m=distance_m, mapped_base_offset_m=offset_m,
                  start_pose_map=list(actual), target_pose_map=list(end),
                  final_parking_confirmed=False)
        travelled_m = self._drive_zero_turn(
            distance_m, precision=True, hold_heading=True,
            approach_time_s=FACE_APPROACH_TIME_S)
        reached = (travelled_m is not None and travelled_m >= distance_m - .01
                   and not self.stop_requested and self._operator_link_ready()
                   and getattr(self, '_last_direct_motion_failure', None) is None)
        self.emit('table_view_approach_finished', reached=reached,
                  travelled_m=travelled_m, final_parking_confirmed=False)
        return reached

    def _observe_with_search(self, camera_mount, front_extent_m, gap_m,
                             region_xy, region_radius_m, *, phase,
                             search_enabled, search_budget):
        """Retry a recoverable initial observation after bounded Nav2 rotations."""
        latency_reobserved = False
        if (phase == 'face_alignment'
                and getattr(self, 'surface_mode', 'front') == 'table_edge'
                and not self._advance_to_table_view()):
            raise RuntimeError('mapped table observation approach unavailable')
        while True:
            try:
                return self.observe_target(
                    camera_mount, front_extent_m, gap_m,
                    region_xy, region_radius_m)
            except BoxObservationUnavailable as error:
                latency_limited = getattr(error, 'latency_limited', False) is True
                self.emit(
                    'box_observation_failed', phase=phase,
                    reason=error.reason, retryable=error.retryable,
                    latency_limited=latency_limited,
                    stop_requested=self.stop_requested,
                    search_enabled=search_enabled,
                    search_steps_used=search_budget.steps_used,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))
                # An operator stop or deadline never rotates or observes again.
                if self.stop_requested:
                    raise
                # A late-input verdict is observed once more without moving.
                if latency_limited and not latency_reobserved:
                    latency_reobserved = True
                    self.emit('box_observation_latency_reobserve', phase=phase,
                              reason=error.reason)
                    continue
                # At the close-range final approach, rotation can sweep the
                # chassis into the box.  Only the initial face search rotates.
                if (not search_enabled or phase != 'face_alignment'
                        or not error.retryable):
                    raise
                delta_yaw_rad = search_budget.next_rotation(
                    toward_rad=getattr(
                        self, 'last_table_heading_error_rad'
                        if getattr(search_budget, 'target_directed', False)
                        else 'last_region_bearing_error_rad', None))
                if delta_yaw_rad is None:
                    if (getattr(search_budget, 'target_directed', False)
                            and not search_budget.reposition_attempted):
                        search_budget.reposition_attempted = True
                        if self._reposition_search():
                            continue
                    self.emit(
                        'box_search_exhausted', phase=phase,
                        reason=error.reason,
                        search_steps_used=search_budget.steps_used,
                        search_cumulative_yaw_rad=(
                            search_budget.cumulative_yaw_rad))
                    raise BoxObservationUnavailable(
                        f'search exhausted after {search_budget.steps_used} rotations; '
                        f'last failure: {error.reason}', retryable=False) from error
                self.emit(
                    'box_search_rotation_requested', phase=phase,
                    search_step=search_budget.steps_used,
                    delta_yaw_rad=delta_yaw_rad,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))
                if not self.search_rotation(delta_yaw_rad):
                    if not search_budget.reposition_attempted:
                        search_budget.reposition_attempted = True
                        if self._reposition_search():
                            continue
                    self.emit(
                        'failed', phase='box_search_rotation',
                        observation_phase=phase,
                        reason=(getattr(self, 'last_search_failure_reason', None)
                                or 'rotation_observation_not_confirmed'),
                        search_step=search_budget.steps_used,
                        search_cumulative_yaw_rad=(
                            search_budget.cumulative_yaw_rad))
                    raise RuntimeError('box search rotation failed')
                self.emit(
                    'box_search_rotation_completed', phase=phase,
                    search_step=search_budget.steps_used,
                    search_cumulative_yaw_rad=(
                        search_budget.cumulative_yaw_rad))

    def at_observation(self, route_path):
        """Return True when the stationary robot stands in a route's observation region."""
        route = load_route(route_path)
        limit_m = route.get('max_route_start_distance_m')
        if (isinstance(limit_m, bool) or not isinstance(limit_m, (float, int))
                or not math.isfinite(limit_m) or not 0.0 < limit_m <= 0.3):
            raise ValueError('observation route requires a bounded start pose')
        self.verify_live_maps()
        with self._localization_bound(True):
            if not self.wait_until_ready(timeout=10.0):
                raise RuntimeError(self.readiness_failure_reason())
            actual, _ = self.capture_stationary_pose(intermediate=True)
        end = route['waypoints'][-1]
        return math.dist(actual[:2], (end['x'], end['y'])) <= limit_m

    def visit_observed_box(self, route_path, camera_mount, geometry,
                           table_id, region_xy, region_radius_m, execute=False,
                           candidate_trial=False, resume_at_observation=False,
                           resume_at_final_approach=False, search=False,
                           task_timeout_s=240.0):
        """Separate transit, face alignment and final approach in the result log."""
        if execute and candidate_trial is not True:
            raise RuntimeError(
                'execution requires explicit nominal-camera candidate trial')
        self.candidate_trial = candidate_trial is True
        self.incomplete_gap_stop = False
        self._confirmed_box_stop = None
        self._continuous_table_stop = None
        self.box_geometry = geometry
        surface_mode = getattr(self, 'surface_mode', 'front')
        if surface_mode not in SURFACE_MODES:
            raise ValueError('surface mode must be front or table_edge')
        if resume_at_final_approach and not (
                execute and resume_at_observation and surface_mode == 'table_edge'):
            raise RuntimeError(
                'final approach resume requires executed table_edge observation resume')
        route = load_route(route_path)
        self._known_table_heading_rad = (
            route['waypoints'][-1].get('yaw') if surface_mode == 'table_edge' else None)
        front_extent = geometry['front_to_wheel_axis']
        front_extent_m = front_extent['value']
        self.front_extent_provenance = front_extent
        width_m = geometry['wheel_outer_width']['value']
        if (isinstance(front_extent_m, bool) or not isinstance(front_extent_m, (int, float))
                or not math.isfinite(front_extent_m) or not 0 < front_extent_m < 0.5):
            raise ValueError('invalid chassis front extent')
        if (isinstance(width_m, bool) or not isinstance(width_m, (int, float))
                or not math.isfinite(width_m) or width_m <= 0):
            raise ValueError('invalid chassis width')
        # The front corners are the frame's; older geometry files give only the wheels.
        frame_m = geometry.get('frame_width', {'value': width_m})['value']
        if (isinstance(frame_m, bool) or not isinstance(frame_m, (int, float))
                or not math.isfinite(frame_m) or not 0 < frame_m <= width_m):
            raise ValueError('invalid chassis frame width')
        self.box_front_half_width_m = frame_m / 2
        self._table_observation_context = (camera_mount, region_xy, region_radius_m)
        for key, field in (('map', 'map_yaml'), ('keepout', 'keepout_mask_yaml')):
            verify_identity(self.registry[key], route[field])
        if route.get('frame_id') != 'map':
            raise ValueError('observation route must use map')
        self.table_id = table_id
        self.station_reference_goal = None
        self.station_reference_goal_frozen = False
        self.verify_live_maps()
        if execute and resume_at_observation:
            # An observation resume continues the mission's running battery bound.
            self._mission_started = True
        # A resume starts in the observation stage, where AMCL may still hold the
        # covariance of its last transit update; a fresh start is judged strictly.
        with self._localization_bound(resume_at_observation):
            if not self.wait_until_ready(timeout=10.0):
                raise RuntimeError(self.readiness_failure_reason())
            # Normal starts validate this in preflight; observation resumes skip it.
            if resume_at_observation and not self._parking_parameters_ready():
                raise RuntimeError(getattr(self, 'last_parking_parameters_failure', None)
                                   or 'parking controller parameters unavailable')
            if execute and not self._precision_collision_ready(geometry):
                raise RuntimeError('precision collision-monitor profile unavailable')
            # The starting place is an explicit route contract, not a guessed pose.
            start = route.get('start_pose')
            limit_m = route.get('max_route_start_distance_m')
            if (not isinstance(start, dict) or not isinstance(limit_m, (float, int))
                    or not math.isfinite(limit_m) or not 0.0 < limit_m <= 0.3):
                raise ValueError('observation route requires a bounded start pose')
            actual, _ = self.capture_stationary_pose(intermediate=True)
        reference = route['waypoints'][-1] if resume_at_observation else start
        if math.dist(actual[:2], (reference['x'], reference['y'])) > limit_m:
            if resume_at_observation:
                raise RuntimeError('robot is not at the reached observation region')
            raise RuntimeError('robot is not at the observation route starting place')
        self.config, self.waypoints = route, route['waypoints']
        self.emit('observation_route_selected', waypoints=self.waypoints,
                  surface_mode=surface_mode,
                  front_extent_provenance=self._json_evidence(front_extent),
                  candidate_trial=self.candidate_trial,
                  physical_accuracy='NOT_EXTERNALLY_MEASURED')
        if not resume_at_observation and not self.preflight():
            self.emit('failed', phase='preflight',
                      reason=getattr(self, 'last_preflight_failure', None),
                      end_error_m=getattr(self, 'preflight_end_error_m', None))
            return False
        if not execute:
            self.emit('transit_planned_only', box_target='requires_observation_on_arrival')
            return True
        # Setup time must not consume the motion/search budget.
        self.run_deadline_s = time.monotonic() + task_timeout_s
        # The camera starts with the transit: its 11 s start-up runs while driving.
        if not self._camera(True):
            self.emit('failed', phase='box_camera_start')
            return False
        if resume_at_observation:
            self.emit('resume_at_reached_observation', actual_pose=list(actual),
                      transit_skipped=True, final_parking_confirmed=False)
        elif not self.execute(final_parking=False):
            self.emit('failed', phase='table_region_transit')
            return False
        if not self._camera_ready():
            self.emit('failed', phase='box_camera_ready')
            return False
        search_budget = BoxSearchBudget(target_directed=surface_mode == 'table_edge')
        # Face alignment happens while depth is still in its usable range.
        straight_to_odom = None
        phases = ((('final_approach', FINAL_PLAN_GAP_M),)
                  if resume_at_final_approach else
                  (('face_alignment', 0.45), ('final_approach', FINAL_PLAN_GAP_M)))
        continuous_target = None
        for phase, gap_m in phases:
            # Observation and face alignment are intermediate; the final approach,
            # its confirmation and the final capture keep the strict AMCL bounds.
            with self._localization_bound(phase == 'face_alignment'):
                plan_input_recovered = False
                while True:
                    if continuous_target is None:
                        target = self._observe_with_search(
                            camera_mount, front_extent_m, gap_m, region_xy,
                            region_radius_m, phase=phase, search_enabled=search,
                            search_budget=search_budget)
                    else:
                        target, continuous_target = continuous_target, None
                        face = target['face_center_map_xy_m']
                        normal = target['outward_normal_map_xy']
                        target.update(x_m=face[0] + normal[0] * (front_extent_m + gap_m),
                                      y_m=face[1] + normal[1] * (front_extent_m + gap_m))
                    target.update(id=phase, priority=1, approach_offset_m=0.5)
                    # The escape before any return starts from the last planned face.
                    self.last_box_face = {
                        'face_center_map_xy_m': list(target['face_center_map_xy_m']),
                        'outward_normal_map_xy': list(target['outward_normal_map_xy'])}
                    self.emit('box_phase', phase=phase, requested_front_gap_m=gap_m,
                              surface_mode=surface_mode,
                              target_basis=('tabletop_front_edge'
                                            if surface_mode == 'table_edge'
                                            else 'lidar_depth_front_face'),
                              target_front_gap_m=(self._final_gap_m()
                                                  if phase == 'final_approach' else gap_m))
                    if phase == 'face_alignment' and surface_mode == 'table_edge':
                        observed_pose = target.get('observation_robot_pose_map', actual)
                        residual = math.atan2(
                            math.sin(target['yaw_rad'] - observed_pose[2]),
                            math.cos(target['yaw_rad'] - observed_pose[2]))
                        normal = target['outward_normal_map_xy']
                        face = target['face_center_map_xy_m']
                        remaining_m = (sum((observed_pose[i] - face[i]) * normal[i]
                                           for i in (0, 1))
                                       - front_lead_m(front_extent_m,
                                                      self.box_front_half_width_m, residual)
                                       - self._final_gap_m())
                        if (self._final_gap_m() <= 0.01
                                and abs(residual) <= FINAL_TRIM_MAX_RAD
                                and 0.0 < remaining_m / math.cos(residual) <= LIVE_APPROACH_MAX_M):
                            # No motion occurred: reuse this accepted face for the final
                            # plan, then refine it from stamped depth while approaching.
                            continuous_target = dict(target)
                            self.emit('table_alignment_transit_skipped',
                                      reason='within_continuous_approach_range',
                                      remaining_m=remaining_m, heading_error_rad=residual)
                            planned = {'ok': True, 'alignment_skipped': True}
                            break
                        forward_m = (
                            (target['x_m'] - observed_pose[0]) * math.cos(observed_pose[2])
                            + (target['y_m'] - observed_pose[1]) * math.sin(observed_pose[2]))
                        if forward_m <= 0.0:
                            # A nearby table needs a normal trim and forward approach,
                            # not a forward-only navigation goal behind the base.
                            self.emit('table_alignment_transit_skipped',
                                      reason='alignment_target_behind_observer',
                                      target_forward_m=forward_m)
                            planned = {'ok': True, 'alignment_skipped': True}
                            break
                    if phase == 'face_alignment':
                        # Judged by the alignment goal checker, not the final contract.
                        planned = self.plan_pose(
                            target, single=True,
                            end_tolerance_m=MAXIMUM_CONTRACT_VALUES['xy_tolerance_m'])
                    else:
                        planned = self.plan_pose(
                            target, single=True, end_tolerance_m=FINAL_PLAN_END_TOLERANCE_M)
                    reason = planned.get('guard_failure')
                    if (planned['ok'] or planned.get('reason') != 'navigation_not_ready'
                            or plan_input_recovered or self.stop_requested
                            or not self._input_gap_recoverable(reason)):
                        break
                    # A face observed before an input gap is discarded, never planned.
                    plan_input_recovered = True
                    self.emit('box_target_discarded', phase=phase, reason=reason)
                    if not self._wait_for_input_recovery(reason):
                        break
                if not planned['ok']:
                    self.emit('failed', phase=phase, planning=planned)
                    return False
                if planned.get('alignment_skipped'):
                    continue
                if phase == 'final_approach':
                    self._final_failure_reported = False
                    approached, straight_to_odom = self._straight_final_approach(
                        target, front_extent_m)
                    if not approached:
                        if not self._final_failure_reported:
                            self.emit('failed', phase=phase,
                                      reason=(getattr(self, '_last_direct_motion_failure', None)
                                              or 'nav2_or_stop_confirmation'))
                        return False
                    continue
                # Live map callbacks keep checking the verified identity throughout
                # this visit; do not repeat disk/service configuration audits here.
                if not self.execute(final_parking=phase == 'final_approach',
                                    alignment=phase == 'face_alignment',
                                    face=phase == 'face_alignment'):
                    self.emit('failed', phase=phase, reason='nav2_or_stop_confirmation')
                    return False
        fine_table = surface_mode == 'table_edge' and self._final_gap_m() <= 0.01
        corrections = 0
        reverse_correction_error_m = None
        realignment_reapproached = False
        near_alignment_attempted = False
        previous_gap_m = None
        frozen_target_odom = None
        if not fine_table:
            self.run_deadline_s = None
        while True:
            final_depth_measurement = False
            confirmation_basis = 'pre_approach_surface'
            continuous_stop = getattr(self, '_continuous_table_stop', None)
            if surface_mode == 'table_edge' and continuous_stop is not None:
                target.update(continuous_stop['target'])
                confirmation_basis = 'frozen_table_edge_odom_stop'
                self.emit('table_continuous_stop_confirmed',
                          tracking_basis=continuous_stop['basis'],
                          last_observation_stamp_s=continuous_stop['observation_stamp_s'],
                          final_depth_measurement=False)
            # A carried face proves the motion stopped, not the final physical gap.
            # Prefer fresh depth; retain the carried face only for healthy occlusion.
            if surface_mode == 'table_edge':
                try:
                    target = self.observe_target(
                        camera_mount, front_extent_m, self._final_gap_m(),
                        region_xy, region_radius_m, reuse_configuration=True)
                except (BoxObservationUnavailable, RuntimeError, ValueError) as error:
                    reason = getattr(error, 'reason', None) or str(error) or type(error).__name__
                    if (not fine_table or straight_to_odom is None
                            or not self._fresh_table_edge_occlusion(error)):
                        self.emit('failed', phase='final_depth_measurement', reason=reason,
                                  final_depth_measurement=False)
                        return False
                    confirmation_basis = 'frozen_table_edge_odom_stop'
                    self.emit(
                        'final_table_edge_occluded', reason=reason,
                        fresh_frames=error.fresh_frames,
                        confirmation_basis=confirmation_basis,
                        physical_accuracy='NOT_EXTERNALLY_MEASURED')
                else:
                    final_depth_measurement = True
                    frozen_target_odom = None
                    confirmation_basis = 'fresh_depth_table_edge'
                    observation_revision = getattr(self, 'parking_motion_revision', None)
            parked, _ = self.capture_stationary_pose()
            if final_depth_measurement:
                if observation_revision != getattr(self, 'parking_motion_revision', None):
                    self.emit('box_target_discarded', phase='final_depth_measurement',
                              reason='motion_after_table_observation')
                    continue
                # The stop hold may receive a new map->odom correction while the
                # base stays still. Move the whole observed geometry into that
                # map frame, preserving the measured base-to-table relation.
                target = self._rebase_stopped_table_target(target, parked)
                self.last_box_face = {
                    'face_center_map_xy_m': list(target['face_center_map_xy_m']),
                    'outward_normal_map_xy': list(target['outward_normal_map_xy'])}
            actual = parked
            outward = target['outward_normal_map_xy']
            face = target['face_center_map_xy_m']
            target_yaw = target['yaw_rad']
            parked = actual
            if straight_to_odom is not None and not final_depth_measurement:
                # The straight approach ran in odom from the aligned pose; judge the
                # gap there too, against the face observed before it.
                face_x, face_y, _ = straight_to_odom(face[0], face[1], 0.0)
                tip_x, tip_y, _ = straight_to_odom(face[0] + outward[0], face[1] + outward[1], 0.0)
                face, outward = (face_x, face_y), (tip_x - face_x, tip_y - face_y)
                target_yaw = straight_to_odom(0.0, 0.0, target_yaw)[2]
                frozen_target_odom = {
                    **target,
                    'face_center_map_xy_m': face,
                    'outward_normal_map_xy': outward,
                    'yaw_rad': target_yaw,
                }
                actual = self._odom_pose()
            front = (actual[0] + front_extent_m * math.cos(actual[2]),
                     actual[1] + front_extent_m * math.sin(actual[2]))
            estimated_gap_m = sum((front[i] - face[i]) * outward[i] for i in (0, 1))
            half_width_m = self.box_front_half_width_m
            corners = [
                (front[0] - side * half_width_m * math.sin(actual[2]),
                 front[1] + side * half_width_m * math.cos(actual[2]))
                for side in (-1, 1)
            ]
            corner_gaps = [sum((corner[i] - face[i]) * outward[i] for i in (0, 1))
                           for corner in corners]
            yaw_error = abs(math.atan2(
                math.sin(actual[2] - target_yaw),
                math.cos(actual[2] - target_yaw)))
            target_gap_m = self._final_gap_m()
            nearest_gap_m = min(corner_gaps)
            gap_tolerance_m = (TABLE_FINE_GAP_TOLERANCE_M if fine_table
                               else FINAL_GAP_TOLERANCE_M)
            if surface_mode == 'table_edge':
                # A table edge represents the whole parked face. Confirm both front
                # corners so a near corner cannot hide a visibly skewed far corner.
                gap_confirmation_basis = 'both_front_corners'
                gaps_confirmed = all(
                    gap > 0.0 and abs(gap - target_gap_m) <= gap_tolerance_m
                    for gap in corner_gaps)
                allowed_yaw_error_rad = min(
                    self.parking_contract['yaw_tolerance_rad'], FINAL_HEADING_TRIM_RAD)
            else:
                # Preserve the legacy box/front-face contract, where the leading
                # (nearest) corner is the clearance reference.
                gap_confirmation_basis = 'nearest_front_corner'
                gaps_confirmed = (
                    nearest_gap_m > 0.0
                    and abs(nearest_gap_m - target_gap_m) <= FINAL_GAP_TOLERANCE_M)
                allowed_yaw_error_rad = self.parking_contract['yaw_tolerance_rad']
            gap_confirmed = (
                math.isfinite(estimated_gap_m) and all(math.isfinite(gap) for gap in corner_gaps)
                and gaps_confirmed
                and yaw_error <= allowed_yaw_error_rad)
            lateral_status = self._station_lateral_status()
            if gap_confirmed and lateral_status:
                lateral_status, moved = self._retry_station_lateral_reference(
                    lateral_status, getattr(self, 'parking_motion_revision', None))
                if self.stop_requested:
                    self.emit('failed', phase='final_approach', reason='operator_stop')
                    return False
                if moved:
                    self.emit('box_target_discarded', phase='final_approach',
                              reason='motion_during_station_reference_retry')
                    continue
            lateral_confirmed = (
                not lateral_status
                or (lateral_status.get('lateral_reference_valid') is True
                    and lateral_status['internal_lateral_within_tolerance'] is True))
            parking_confirmed = gap_confirmed and lateral_confirmed
            if parking_confirmed:
                # The dwell holds the confirmed stop where it is (map pose at rest).
                self.selected_pose = {'id': 'final_approach', 'x_m': parked[0],
                                      'y_m': parked[1], 'yaw_rad': parked[2]}
                if confirmation_basis == 'frozen_table_edge_odom_stop':
                    self.last_box_face = {
                        'reference_frame': 'odom',
                        'face_center_xy_m': list(face),
                        'outward_normal_xy': list(outward),
                    }
            self.emit('box_approach_finished' if parking_confirmed else 'box_gap_not_confirmed',
                      estimated_front_gap_m=estimated_gap_m,
                      estimated_front_corner_gaps_m=corner_gaps,
                      estimated_nearest_front_gap_m=nearest_gap_m,
                      estimated_face_yaw_error_rad=yaw_error,
                      gap_confirmation_basis=gap_confirmation_basis,
                      confirmation_basis=confirmation_basis,
                      internal_estimate_only=(
                          confirmation_basis == 'frozen_table_edge_odom_stop'),
                      allowed_face_yaw_error_rad=allowed_yaw_error_rad,
                      desired_front_gap_m=target_gap_m, confirmation=self.confirmation,
                      physical_accuracy='NOT_EXTERNALLY_MEASURED',
                      final_depth_measurement=final_depth_measurement,
                      **lateral_status)
            if parking_confirmed:
                self.run_deadline_s = None
                self._confirmed_box_stop = {
                    'motion_revision': getattr(self, 'parking_motion_revision', None),
                    'selected_pose': dict(self.selected_pose),
                    'gap_confirmation_basis': gap_confirmation_basis,
                    **lateral_status,
                }
                return True
            if gap_confirmed and not lateral_confirmed:
                # At the final clearance a differential base cannot translate
                # sideways. Do not leave as a success or start a blind shuffle.
                self.incomplete_gap_stop = True
                self.emit('failed', phase='final_approach',
                          reason='station_lateral_arrival_unconfirmed', **lateral_status)
                return False
            self.incomplete_gap_stop = (
                fine_table and all(math.isfinite(gap) and gap > 0.0 for gap in corner_gaps))
            if self.incomplete_gap_stop:
                self.last_box_face = (
                    {'reference_frame': 'odom', 'face_center_xy_m': list(face),
                     'outward_normal_xy': list(outward)}
                    if confirmation_basis == 'frozen_table_edge_odom_stop' else
                    {'face_center_map_xy_m': list(face),
                     'outward_normal_map_xy': list(outward)})
            # A small aligned overshoot needs a short retreat, not a failed visit
            # and a return home. Keep the same observed plane and task deadline.
            if (fine_table and self.incomplete_gap_stop
                    and yaw_error <= allowed_yaw_error_rad
                    and nearest_gap_m < target_gap_m - gap_tolerance_m
                    and max(corner_gaps) - nearest_gap_m <= 2.0 * gap_tolerance_m):
                error_m = target_gap_m - sum(corner_gaps) / len(corner_gaps)
                if (reverse_correction_error_m is not None
                        and reverse_correction_error_m - error_m < TABLE_FINE_MIN_PROGRESS_M):
                    self.emit('failed', phase='final_approach', reason='final_gap_no_progress')
                    return False
                if corrections >= TABLE_FINE_MAX_CORRECTIONS:
                    self.emit('failed', phase='final_approach',
                              reason='final_gap_correction_limit')
                    return False
                reverse_correction_error_m = error_m
                corrections += 1
                self.incomplete_gap_stop = False
                if not self._retreat_table_gap(
                        error_m / math.cos(yaw_error), parked, corrections):
                    return False
                continue
            # A skewed stop cannot advance when one corner is already close.
            # Back out using the observed face, then align to a fresh observation
            # within this visit and its original budget, rather than ending it.
            straight_correction = (
                fine_table and all(math.isfinite(gap) and gap > 0 for gap in corner_gaps)
                and yaw_error <= allowed_yaw_error_rad
                and max(corner_gaps) - nearest_gap_m <= 2 * gap_tolerance_m
                and estimated_gap_m > target_gap_m)
            corner_spread_m = max(corner_gaps) - nearest_gap_m
            close_table_misalignment = (
                fine_table
                and all(math.isfinite(gap) and gap > 0.0 for gap in corner_gaps)
                and nearest_gap_m <= FINAL_PLAN_GAP_M
                and (yaw_error > allowed_yaw_error_rad
                     or corner_spread_m > 2.0 * gap_tolerance_m))
            if close_table_misalignment and near_alignment_attempted:
                self.emit('failed', phase='final_approach',
                          reason='final_table_alignment_unconfirmed',
                          near_alignment_attempted=True,
                          estimated_front_corner_gaps_m=corner_gaps)
                return False
            if (close_table_misalignment and final_depth_measurement
                    and nearest_gap_m > target_gap_m
                    and yaw_error < min(FINAL_TRIM_MAX_RAD,
                                        math.atan2(half_width_m, front_extent_m))):
                if (self.stop_requested or not self._operator_link_ready()
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    return False
                near_alignment_attempted = True
                self.emit('table_near_alignment_correction',
                          estimated_front_corner_gaps_m=corner_gaps,
                          estimated_face_yaw_error_rad=yaw_error,
                          target_front_gap_m=target_gap_m)
                self.incomplete_gap_stop = False
                self._final_failure_reported = False
                approached, straight_to_odom = self._straight_final_approach(
                    target, front_extent_m)
                if not approached:
                    if not self._final_failure_reported:
                        reason = ('operator_stop' if self.stop_requested else
                                  getattr(self, '_last_direct_motion_failure', None)
                                  or 'nav2_or_stop_confirmation')
                        self.emit('failed', phase='final_approach', reason=reason)
                    return False
                previous_gap_m = None
                continue
            if close_table_misalignment and not realignment_reapproached:
                if (self.stop_requested or not self._operator_link_ready()
                        or not self._navigation_ready(require_fresh_amcl=False)):
                    return False
                realignment_reapproached = True
                self.emit('table_realignment_reapproach',
                          estimated_front_corner_gaps_m=corner_gaps,
                          target_front_gap_m=target_gap_m)
                self.incomplete_gap_stop = False
                if not self._leave_for_table_reobservation():
                    return False
                try:
                    target = self.observe_target(
                        camera_mount, front_extent_m, target_gap_m, region_xy, region_radius_m,
                        reuse_configuration=True)
                except (BoxObservationUnavailable, RuntimeError, ValueError) as error:
                    self.emit('failed', phase='table_realignment_reobserve', reason=str(error))
                    return False
                approached, straight_to_odom = self._straight_final_approach(
                    target, front_extent_m)
                if not approached:
                    return False
                previous_gap_m = None
                continue
            if close_table_misalignment:
                self.emit(
                    'failed', phase='final_approach',
                    reason='final_table_alignment_unconfirmed',
                    estimated_front_corner_gaps_m=corner_gaps,
                    estimated_face_yaw_error_rad=yaw_error,
                    realignment_reapproach_consumed=True)
                return False
            # Corrections consume fresh stopped observations within the same task
            # budget; a lost face or an overshoot never dispatches another advance.
            if (not fine_table or not all(math.isfinite(gap) for gap in corner_gaps)
                    or (nearest_gap_m <= target_gap_m + gap_tolerance_m
                        and not straight_correction)):
                return False
            if (self.stop_requested or not self._operator_link_ready()
                    or not self._navigation_ready(require_fresh_amcl=False)):
                self.emit('failed', phase='final_approach', reason='navigation_not_ready')
                return False
            if (previous_gap_m is not None
                    and previous_gap_m - nearest_gap_m < TABLE_FINE_MIN_PROGRESS_M):
                self.emit('failed', phase='final_approach', reason='final_gap_no_progress')
                return False
            if corrections >= TABLE_FINE_MAX_CORRECTIONS:
                self.emit('failed', phase='final_approach', reason='final_gap_correction_limit')
                return False
            previous_gap_m = nearest_gap_m
            corrections += 1
            self.emit('final_gap_correction', correction=corrections,
                      estimated_front_corner_gaps_m=corner_gaps,
                      target_front_gap_m=target_gap_m)
            self.incomplete_gap_stop = False
            self._final_failure_reported = False
            if confirmation_basis == 'frozen_table_edge_odom_stop':
                approached, straight_to_odom = self._zero_turn_final_approach(
                    frozen_target_odom, front_extent_m, allow_trim=False,
                    gap_m=(target_gap_m - (max(corner_gaps) - nearest_gap_m) / 2
                           if straight_correction else None),
                    frozen_to_odom=straight_to_odom)
            else:
                approached, straight_to_odom = self._straight_final_approach(
                    target, front_extent_m)
            if not approached:
                if not self._final_failure_reported:
                    self.emit('failed', phase='final_approach',
                              reason=('operator_stop' if self.stop_requested else
                                      getattr(self, '_last_direct_motion_failure', None)
                                      or 'nav2_or_stop_confirmation'))
                return False

    def _rebase_stopped_table_target(self, target, parked):
        """Keep a stopped depth measurement coherent with a later map pose."""
        observed = target.get('observation_robot_pose_map')
        if observed is None:
            return target
        turn = parked[2] - observed[2]
        cosine, sine = math.cos(turn), math.sin(turn)

        def rotate(vector):
            x, y = vector
            return cosine * x - sine * y, sine * x + cosine * y

        def translate(point):
            dx, dy = rotate((point[0] - observed[0], point[1] - observed[1]))
            return parked[0] + dx, parked[1] + dy

        rebased = dict(target)
        rebased['x_m'], rebased['y_m'] = translate((target['x_m'], target['y_m']))
        rebased['face_center_map_xy_m'] = translate(target['face_center_map_xy_m'])
        rebased['outward_normal_map_xy'] = rotate(target['outward_normal_map_xy'])
        if 'edge_basis_map_xy' in target:
            rebased['edge_basis_map_xy'] = rotate(target['edge_basis_map_xy'])
        rebased['yaw_rad'] = math.atan2(
            math.sin(target['yaw_rad'] + turn), math.cos(target['yaw_rad'] + turn))
        rebased['observation_robot_pose_map'] = list(parked)
        self.emit('table_stop_target_rebased', observation_pose_map=list(observed),
                  stopped_pose_map=list(parked), motion_revision_unchanged=True)
        return rebased

    def _fresh_table_edge_occlusion(self, error):
        """Recognize a healthy depth stream whose table edge is hidden at close range."""
        return (isinstance(error, BoxObservationUnavailable)
                and error.reason == 'stable detected table_edge surface is required'
                and error.retryable and not error.latency_limited
                and isinstance(error.required_fresh_frames, int)
                and error.required_fresh_frames >= 1
                and error.fresh_frames >= error.required_fresh_frames
                and error.fresh_rejection_reasons == {'no_table_edge_candidate'})

    def _retreat_table_gap(self, distance_m, start_map, correction):
        """Restore an aligned gap with the existing guarded straight controller."""
        if (not math.isfinite(distance_m) or not 0.0 < distance_m <= self._final_gap_m()
                or self.stop_requested or not self._operator_link_ready()
                or not self._navigation_ready(require_fresh_amcl=False)):
            return False
        end = (start_map[0] - distance_m * math.cos(start_map[2]),
               start_map[1] - distance_m * math.sin(start_map[2]), start_map[2])
        check = CachedStaticCorridorValidator(
            Path(self.registry['map']['yaml_path']),
            Path(self.registry['keepout']['yaml_path']), self._footprint_outline())
        if not check([start_map, end]):
            self.emit('failed', phase='final_approach', reason='table_gap_retreat_static_blocked')
            return False
        self.emit('table_gap_reverse_correction', correction=correction,
                  distance_m=distance_m, target_front_gap_m=self._final_gap_m(),
                  original_deadline_preserved=True)
        travelled = self._drive_zero_turn(-distance_m, precision=True, hold_heading=True)
        return (travelled is not None and not self.stop_requested
                and self._operator_link_ready()
                and getattr(self, '_last_direct_motion_failure', None) is None)

    def _final_gap_m(self):
        """Front gap the final approach closes to: --final-gap-m, else the contract."""
        override = getattr(self, 'final_gap_override_m', None)
        if override is not None:
            return override
        return self.parking_contract.get('target_front_gap_m', DEFAULT_FRONT_GAP_M)

    def _fail_final(self, reason, **fields):
        """Report the final approach failure once, with its own reason."""
        self._final_failure_reported = True
        self.emit('failed', phase='final_approach', reason=reason, **fields)

    def _straight_final_approach(self, target, front_extent_m):
        """Run the configured final approach: Parking along a held path, or zero turn."""
        if (getattr(self, 'surface_mode', 'front') == 'table_edge'
                and self._final_gap_m() <= 0.01):
            return self._table_fine_approach(target, front_extent_m)
        if getattr(self, 'zero_turn_final', False):
            return self._zero_turn_final_approach(target, front_extent_m)
        return self._parking_final_approach(target, front_extent_m)

    def _table_fine_approach(self, target, front_extent_m):
        """Align to the observed normal while closing the gap in one move."""
        self._continuous_table_stop = None
        observed = target.get('observation_robot_pose_map')
        if observed is not None:
            lateral_status = self._station_lateral_status()
            lateral_ready = (not lateral_status
                             or lateral_status.get('internal_lateral_within_tolerance') is True)
            residual = math.atan2(math.sin(target['yaw_rad'] - observed[2]),
                                  math.cos(target['yaw_rad'] - observed[2]))
            span_m = 2 * self.box_front_half_width_m * abs(math.sin(residual))
            distance_m = sum(
                (observed[i] - target['face_center_map_xy_m'][i])
                * target['outward_normal_map_xy'][i] for i in (0, 1))
            nearest_m = distance_m - front_lead_m(
                front_extent_m, self.box_front_half_width_m, residual)
            if (lateral_ready and 0 < nearest_m <= FINAL_PLAN_GAP_M
                    and span_m <= 2 * TABLE_FINE_GAP_TOLERANCE_M
                    and abs(residual) <= min(
                        self.parking_contract['yaw_tolerance_rad'], FINAL_HEADING_TRIM_RAD)):
                # Centre the two corner gaps on the unchanged final target.
                # The leading corner alone would leave the far corner too far out.
                return self._zero_turn_final_approach(
                    target, front_extent_m, gap_m=self._final_gap_m() - span_m / 2,
                    allow_trim=False, continuous_table=False)
        result = self._zero_turn_final_approach(
            target, front_extent_m, allow_trim=False, continuous_table=True)
        if (result[0] or getattr(self, '_last_direct_motion_failure', None) not in {
                'table alignment requires more clearance',
                'table lateral alignment requires more clearance'}):
            return result
        if (self.stop_requested or not self._operator_link_ready()
                or not self._navigation_ready(require_fresh_amcl=False)):
            return result
        self.last_box_face = {
            'face_center_map_xy_m': list(target['face_center_map_xy_m']),
            'outward_normal_map_xy': list(target['outward_normal_map_xy'])}
        self.emit('table_alignment_clearance_reapproach', original_deadline_preserved=True)
        if not self._leave_for_table_reobservation():
            return False, None
        mount, region, radius = self._table_observation_context
        try:
            target.update(self.observe_target(
                mount, front_extent_m, self._final_gap_m(), region, radius,
                reuse_configuration=True))
        except (BoxObservationUnavailable, RuntimeError, ValueError) as error:
            self._fail_final('table_clearance_reobserve_failed', detail=str(error))
            return False, None
        return self._zero_turn_final_approach(
            target, front_extent_m, allow_trim=False, continuous_table=True)

    def _leave_for_table_reobservation(self):
        mount, _region, _radius = self._table_observation_context
        profile = mount.get('camera_mount', {}).get('table_observation', {})
        clearance_m = profile.get('base_offset_m')
        if clearance_m is None:
            return self._leave_parked_pose()
        return self._leave_parked_pose(clearance_m=clearance_m)

    def _table_approach_validator(self, front_extent_m, to_map_pose):
        """Check the nominal steering corridor with the verified static footprint."""
        footprint = self._footprint_outline()
        map_yaml = Path(self.registry['map']['yaml_path'])
        keepout_yaml = Path(self.registry['keepout']['yaml_path'])
        check = CachedStaticCorridorValidator(map_yaml, keepout_yaml, footprint)

        def validate(actual, face, normal, command_heading_rad, span_m=None, *,
                     lateral_goal_odom=None):
            try:
                preview = face_approach_waypoints(
                    actual, face, normal, front_extent_m, self.box_front_half_width_m,
                    self._final_gap_m(), command_heading_rad=command_heading_rad,
                    allow_near_alignment=True, continuous_table=True,
                    **({'lateral_tracking': True,
                        'lateral_goal_odom': lateral_goal_odom,
                        'lateral_tolerance_m': self.station_reference_profile['lateral_tolerance_m'],
                        'max_lateral_heading_rad': FINAL_TRIM_MAX_RAD}
                       if lateral_goal_odom is not None else {}))
            except ValueError as error:
                raise RuntimeError(str(error)) from error
            if len(preview) == 1:
                # The observed-face precision band already closes beyond the
                # costmap plan endpoint. Its relative gap checks remain in force.
                return
            mapped = [to_map_pose(pose) for pose in preview]
            kwargs = {}
            if span_m is not None:
                center = to_map_pose((*face, 0.))
                tip = to_map_pose((face[0] + normal[0], face[1] + normal[1], 0.))
                kwargs['target_surface'] = {
                    'point_xy_m': center[:2],
                    'normal_xy': (tip[0] - center[0], tip[1] - center[1]),
                    'half_span_m': span_m / 2.}
            if not check(mapped, **kwargs):
                raise RuntimeError('table approach static corridor blocked')

        return validate

    def _table_approach_update(self, target, front_extent_m, to_odom):
        """Use moving depth at its odom timestamp, then carry the face to stop."""
        camera_mount, region_xy, region_radius_m = self._table_observation_context
        anchor = to_odom(0.0, 0.0, 0.0)
        cosine, sine = math.cos(anchor[2]), math.sin(anchor[2])

        def to_map(point):
            dx, dy = point[0] - anchor[0], point[1] - anchor[1]
            return cosine * dx + sine * dy, -sine * dx + cosine * dy

        validate = self._table_approach_validator(
            front_extent_m, lambda pose: (*to_map(pose), pose[2] - anchor[2]))

        face = target['face_center_map_xy_m']
        normal = target['outward_normal_map_xy']
        frozen_face = to_odom(*face, 0.0)[:2]
        tip = to_odom(face[0] + normal[0], face[1] + normal[1], 0.0)
        initial = getattr(self, 'box_status', {}) or {}
        state = {'face': frozen_face,
                 'normal': (tip[0] - frozen_face[0], tip[1] - frozen_face[1]),
                 'stamp': initial.get('stamp_s'), 'input_stamp': initial.get('stamp_s'),
                 'basis': 'initial_stopped_face', 'start': self._odom_pose(),
                 'last_valid_s': time.monotonic(), 'occluded_frames': 0,
                 'occlusion_latched': False, 'precision_band_latched': False,
                 'validated_input': None, 'validated_pose': None,
                 'validated_heading': None, 'validated_goal': None,
                 'span_m': target.get('observed_face_span_m')}
        quorum = getattr(self, 'observer_stable_frames', None) or 8

        def station_goal_odom():
            if not self._station_reference_enabled():
                return None, None
            goal, reference = self._station_goal_reference()
            if goal is not None:
                return tuple(goal['goal_odom']), reference
            taught = self.station_reference_taught_map_pose
            return to_odom(*taught), reference

        def validate_control(actual, filtered_heading):
            goal, _reference = station_goal_odom()
            previous_goal = state['validated_goal']
            goal_changed = (
                (goal is None) != (previous_goal is None)
                or (goal is not None and previous_goal is not None
                    and math.dist(goal[:2], previous_goal[:2])
                    >= self.station_reference_profile['lateral_tolerance_m']))
            heading_change = (math.inf if state['validated_heading'] is None else abs(
                math.atan2(math.sin(filtered_heading - state['validated_heading']),
                           math.cos(filtered_heading - state['validated_heading']))))
            if (goal_changed or state['validated_input'] != state['input_stamp']
                    or state['validated_pose'] is None
                    or math.dist(actual[:2], state['validated_pose'][:2]) >= 0.025
                    or heading_change >= FINAL_TRIM_TOLERANCE_RAD):
                validate(actual, state['face'], state['normal'], filtered_heading, state['span_m'],
                         **({'lateral_goal_odom': goal} if goal is not None else {}))
                state.update(validated_input=state['input_stamp'], validated_pose=actual,
                             validated_heading=filtered_heading, validated_goal=goal)

        def update(actual):
            observation = self.box_status
            received = getattr(self, 'box_status_received_s', None)
            if (not isinstance(observation, dict) or received is None
                    or time.monotonic() - received > OBSERVER_SILENT_S):
                raise RuntimeError('table approach observer silent')
            stamp = observation.get('stamp_s')
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (isinstance(stamp, (float, int)) and not isinstance(stamp, bool)
                    and math.isfinite(stamp) and now_s - stamp > 0.5):
                # Consume callbacks already waiting behind TF/odometry before
                # judging the last processed depth message as a sensor outage.
                deadline_s = time.monotonic() + 0.02
                for _ in range(32):
                    if time.monotonic() >= deadline_s:
                        break
                    spin_node(self, timeout_sec=0.0)
                    observation = self.box_status
                    stamp = observation.get('stamp_s') if isinstance(observation, dict) else None
                    now_s = self.get_clock().now().nanoseconds * 1e-9
                    if (isinstance(stamp, (float, int)) and not isinstance(stamp, bool)
                            and math.isfinite(stamp) and 0.0 <= now_s - stamp <= 0.5):
                        break
            valid_stamp = (not isinstance(stamp, bool)
                           and isinstance(stamp, (float, int)) and math.isfinite(stamp))
            carried_live_frame = (valid_stamp and stamp == state['input_stamp']
                                  and now_s - stamp > 0.5
                                  and self._fresh_depth_frame_after(stamp, now_s))
            if (not valid_stamp
                    or not (0.0 <= now_s - stamp <= 0.5 or carried_live_frame)):
                raise RuntimeError('table approach depth stale or future-dated')
            if observation.get('reason') in OBSERVER_LATENCY_REASONS:
                raise RuntimeError('table approach observer latency')
            if stamp != state['input_stamp']:
                if state['input_stamp'] is not None and stamp < state['input_stamp']:
                    raise RuntimeError('table approach depth clock regressed')
                state['input_stamp'] = stamp
                if state['occlusion_latched'] or state['precision_band_latched']:
                    pass
                elif (observation.get('detected') is True
                        and observation.get('stable') is True
                        and observation.get('surface_kind') == 'table_edge'):
                    if (observation.get('frame_id') != 'camera_color_optical_frame'
                            or observation.get('control_ready') is not False):
                        raise RuntimeError('invalid moving table observation frame or mode')
                    try:
                        transform = self.parking_tf.lookup_transform(
                            'odom', self.parking_contract['robot_base_frame'],
                            rclpy.time.Time(seconds=stamp)).transform
                    except Exception as error:
                        raise RuntimeError('table approach stamped odom TF unavailable') from error
                    rotation = transform.rotation
                    yaw = math.atan2(
                        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
                    measured = compute_table_edge_docking_target(
                        observation, camera_mount,
                        {'x_m': transform.translation.x, 'y_m': transform.translation.y,
                         'yaw_rad': yaw}, requested_gap_m=self._final_gap_m(),
                        front_extent_m=front_extent_m, now_s=now_s)
                    goal_odom, station_observation = station_goal_odom()
                    if goal_odom is not None:
                        measured = anchor_face_to_taught_lateral(
                            measured, goal_odom[:2])
                        measured['rgbd_reference_valid'] = bool(
                            station_observation
                            and station_observation.get('valid') is True
                            and getattr(self, 'station_reference_mode', None)
                            != 'map_pose_only')
                        measured['rgbd_reference_reason'] = (
                            station_observation.get('reason')
                            if station_observation else 'station_reference_not_observed')
                        measured['millimeter_accuracy_validated'] = False
                    face = measured['face_center_map_xy_m']
                    normal = measured['outward_normal_map_xy']
                    if math.dist(to_map(face), region_xy) > region_radius_m:
                        raise RuntimeError('moving table face outside selected region')
                    if sum(normal[i] * state['normal'][i] for i in (0, 1)) <= 0.0:
                        raise RuntimeError('moving table normal reversed')
                    state.update(face=face, normal=normal, stamp=stamp,
                                 span_m=observation.get('width_m'),
                                 last_valid_s=time.monotonic(), occluded_frames=0,
                                 basis='moving_depth_at_stamped_odom')
                    self.emit('table_approach_face_updated', observation_stamp_s=stamp,
                              reference_frame='odom', face_center_xy_m=list(face),
                              outward_normal_xy=list(normal))
                elif observation.get('reason') == 'no_table_edge_candidate':
                    state['occluded_frames'] += 1
                    if state['occluded_frames'] >= quorum:
                        state['basis'] = 'healthy_edge_occlusion_frozen_odom'
                        state['occlusion_latched'] = True
                elif (observation.get('reason') == 'collecting_stability'
                      and observation.get('detected') is True
                      and observation.get('stable') is False
                      and observation.get('surface_kind') == 'table_edge'
                      and observation.get('frame_id') == 'camera_color_optical_frame'
                      and observation.get('control_ready') is False):
                    # Carry the accepted face only until moving stability recovers.
                    # A visible candidate must not permanently disable depth updates.
                    state['occluded_frames'] = 0
                    state['basis'] = 'moving_stability_reset_carried_odom'
                elif time.monotonic() - state['last_valid_s'] > OBSERVER_SILENT_S:
                    raise RuntimeError('table approach stable face unavailable')
            heading = math.atan2(-state['normal'][1], -state['normal'][0])
            residual = math.atan2(math.sin(heading - actual[2]),
                                  math.cos(heading - actual[2]))
            if abs(residual) > FINAL_TRIM_MAX_RAD:
                raise RuntimeError('moving table face heading outside approach range')
            center_gap = (sum((actual[i] - state['face'][i]) * state['normal'][i]
                              for i in (0, 1)) - front_extent_m * math.cos(residual))
            gap = center_gap - self.box_front_half_width_m * abs(math.sin(residual))
            remaining = (center_gap - self._final_gap_m()) / math.cos(residual)
            goal_odom, station_observation = station_goal_odom()
            lateral_ready = True
            lateral_error = None
            if goal_odom is not None:
                left = (-math.sin(heading), math.cos(heading))
                lateral_error = sum(
                    (goal_odom[i] - actual[i]) * left[i] for i in (0, 1))
                lateral_ready = (
                    getattr(self, 'station_reference_goal', None) is not None
                    and abs(lateral_error) <= self.station_reference_profile[
                        'lateral_tolerance_m'])
            if (gap <= FINAL_PLAN_GAP_M + 0.0005
                    and abs(residual) <= FINAL_TRIM_TOLERANCE_RAD and lateral_ready):
                state['precision_band_latched'] = True
                state['basis'] = 'aligned_face_frozen_for_precision_band'
                if getattr(self, 'station_reference_goal', None) is not None:
                    self.station_reference_goal_frozen = True
            travelled = math.hypot(actual[0] - state['start'][0],
                                   actual[1] - state['start'][1])
            if travelled + max(0.0, remaining) > LIVE_APPROACH_MAX_M:
                raise RuntimeError('live table approach exceeds bounded travel')
            command_heading = heading
            face_map = to_map(state['face'])
            normal_map = (cosine * state['normal'][0] + sine * state['normal'][1],
                          -sine * state['normal'][0] + cosine * state['normal'][1])
            offset = front_extent_m + self._final_gap_m()
            self._continuous_table_stop = {
                'target': {**target, 'face_center_map_xy_m': face_map,
                           'outward_normal_map_xy': normal_map,
                           'yaw_rad': math.atan2(-normal_map[1], -normal_map[0]),
                           'x_m': face_map[0] + normal_map[0] * offset,
                           'y_m': face_map[1] + normal_map[1] * offset},
                'basis': state['basis'], 'observation_stamp_s': state['stamp'],
                'lateral_reference_valid': bool(
                    station_observation and station_observation.get('valid') is True)}
            return {'remaining_m': remaining, 'heading_rad': command_heading,
                    'face_heading_rad': heading,
                    'lateral_tracking': goal_odom is not None,
                    'lateral_error_m': lateral_error,
                    'lateral_tolerance_m': (self.station_reference_profile['lateral_tolerance_m']
                                            if goal_odom is not None else None),
                    'target_gap_m': self._final_gap_m(),
                    'basis': state['basis'],
                    'surface_gap_m': gap,
                    'allow_near_alignment': (gap > 0. and abs(residual) < math.atan2(
                        self.box_front_half_width_m, front_extent_m)),
                    'validate_control': validate_control}

        return update

    def _zero_turn_final_approach(self, target, front_extent_m, *, gap_m=None,
                                  reobserve=None, realign_attempt=0, allow_trim=True,
                                  frozen_to_odom=None, continuous_table=False):
        """
        Turn out the residual to the re-observed face in place, then close the gap straight.

        Face alignment stops anywhere inside the 3 deg contract (38 parks 09-30 to
        10-02: mean 1.02 deg, max 2.69 deg; front corners up to 1.27 cm apart) and
        the Parking controller steers on the way in. Toward 1 cm (2026-10-05) the
        residual is measured against the face normal and turned out by odom, and the
        table fine approach holds the trimmed odom heading with feedback.
        """
        if allow_trim:
            actual, _ = self.capture_stationary_pose()
        else:
            actual, _ = self.capture_stationary_pose(intermediate=True)
        face = target['face_center_map_xy_m']
        outward = target['outward_normal_map_xy']
        to_odom = None
        if not allow_trim:
            to_odom = frozen_to_odom
            if to_odom is None:
                anchor = RosPath()
                _anchor_in_odom, to_odom = self._frozen_in_odom(anchor)
                face_x, face_y, _ = to_odom(face[0], face[1], 0.0)
                tip_x, tip_y, _ = to_odom(
                    face[0] + outward[0], face[1] + outward[1], 0.0)
                face = (face_x, face_y)
                outward = (tip_x - face_x, tip_y - face_y)
            actual = self._odom_pose()
        face_heading = math.atan2(-outward[1], -outward[0])
        residual = math.atan2(math.sin(face_heading - actual[2]),
                              math.cos(face_heading - actual[2]))
        if abs(residual) > FINAL_TRIM_MAX_RAD:
            self._fail_final('final_trim_out_of_range',
                             residual_deg=round(math.degrees(residual), 2))
            return False, None
        turned = 0.0
        trimmed = False
        if allow_trim and abs(residual) >= FINAL_TRIM_MIN_RAD:
            remaining_rad = residual
            trim_attempts = (3 if getattr(self, 'surface_mode', 'front') == 'table_edge'
                             and self._final_gap_m() <= 0.01 else 1)
            for attempt in range(trim_attempts):
                if attempt and (self.stop_requested or not self._operator_link_ready()
                                or not self._navigation_ready(require_fresh_amcl=False)):
                    self._fail_final('final_trim_interrupted')
                    return False, None
                trimmed = True
                before_rad = remaining_rad
                correction_rad = self._rotate_in_place(remaining_rad)
                if correction_rad is None or not math.isfinite(correction_rad):
                    self._fail_final('final_trim_missed',
                                     residual_deg=round(math.degrees(residual), 2),
                                     turned_deg=None)
                    return False, None
                turned += correction_rad
                remaining_rad = residual - turned
                if abs(remaining_rad) <= FINAL_TRIM_TOLERANCE_RAD:
                    break
                if reobserve is not None and (
                        remaining_rad * before_rad < 0.0
                        or abs(remaining_rad) >= abs(before_rad)):
                    # Recheck the face before counter-steering on the old sample.
                    self.emit('table_trim_reobserve_requested',
                              residual_before_rad=before_rad,
                              residual_after_rad=remaining_rad)
                    break
            else:
                # The trim target is finer than the final arrival contract. A
                # bounded residual may proceed to fresh post-approach measurement.
                allowed_trim_rad = (min(self.parking_contract['yaw_tolerance_rad'],
                                        FINAL_HEADING_TRIM_RAD)
                                    if trim_attempts > 1 else FINAL_TRIM_TOLERANCE_RAD)
                if abs(remaining_rad) > allowed_trim_rad:
                    self._fail_final('final_trim_missed',
                                     residual_deg=round(math.degrees(residual), 2),
                                     turned_deg=round(math.degrees(turned), 2))
                    return False, None
            actual, _ = self.capture_stationary_pose(intermediate=True)
        if reobserve is not None and trimmed:
            try:
                target.update(reobserve())
            except (BoxObservationUnavailable, RuntimeError, ValueError) as error:
                self._fail_final('table_post_trim_observation_lost', detail=str(error))
                return False, None
            actual, _ = self.capture_stationary_pose(intermediate=True)
            face = target['face_center_map_xy_m']
            outward = target['outward_normal_map_xy']
            face_heading = math.atan2(-outward[1], -outward[0])
            observed_residual = math.atan2(math.sin(face_heading - actual[2]),
                                           math.cos(face_heading - actual[2]))
            self.emit('table_post_trim_observed',
                      residual_deg=math.degrees(observed_residual),
                      face_center_map_xy_m=list(face))
            if abs(observed_residual) > FINAL_TRIM_TOLERANCE_RAD:
                if realign_attempt < 2:
                    if (self.stop_requested or not self._operator_link_ready()
                            or not self._navigation_ready(require_fresh_amcl=False)):
                        self._fail_final('final_trim_interrupted')
                        return False, None
                    return self._zero_turn_final_approach(
                        target, front_extent_m, gap_m=gap_m, reobserve=reobserve,
                        realign_attempt=realign_attempt + 1, allow_trim=allow_trim,
                        frozen_to_odom=frozen_to_odom, continuous_table=continuous_table)
                if abs(observed_residual) > min(
                        self.parking_contract['yaw_tolerance_rad'], FINAL_HEADING_TRIM_RAD):
                    self._fail_final('table_alignment_not_converged',
                                     residual_deg=math.degrees(observed_residual))
                    return False, None
        # The last table leg is already frozen in odom; other trims are logged
        # against the current map face after their odom-judged rotation.
        after = math.atan2(math.sin(face_heading - actual[2]),
                           math.cos(face_heading - actual[2]))
        # The trim was judged in odom; a map heading this far off means AMCL moved.
        if (not continuous_table
                and abs(after) > self.parking_contract['yaw_tolerance_rad']):
            self._fail_final('final_trim_missed', residual_deg=round(math.degrees(residual), 2),
                             residual_after_deg=round(math.degrees(after), 2))
            return False, None
        self.emit('final_yaw_trim', residual_deg=round(math.degrees(residual), 3),
                  turned_deg=round(math.degrees(turned), 3),
                  residual_after_deg=round(math.degrees(after), 3))
        distance_m = sum((actual[i] - face[i]) * outward[i] for i in (0, 1))
        lead_m = front_lead_m(front_extent_m, getattr(self, 'box_front_half_width_m', 0.0), after)
        requested_gap_m = self._final_gap_m() if gap_m is None else gap_m
        travel_m = (distance_m - lead_m - requested_gap_m) / math.cos(after)
        if not math.isfinite(travel_m) or not 0.0 < travel_m <= LIVE_APPROACH_MAX_M:
            self._fail_final('final_approach_unavailable', travel_m=self._json_scalar(travel_m))
            return False, None
        if reobserve is not None and (
                self.stop_requested or not self._operator_link_ready()
                or not self._navigation_ready(require_fresh_amcl=False)):
            self._fail_final('final_trim_interrupted')
            return False, None
        if to_odom is None:
            steps = int(math.ceil(travel_m / 0.05)) + 1
            heading = actual[2]
            path = RosPath()
            path.poses = [self._pose(
                0, {'x': actual[0] + travel_m * k / steps * math.cos(heading),
                    'y': actual[1] + travel_m * k / steps * math.sin(heading),
                    'yaw': heading}) for k in range(steps + 1)]
            path.header = path.poses[0].header
            _odom_path, to_odom = self._frozen_in_odom(path)
        self.emit('final_approach_straight', travel_m=travel_m,
                  heading_basis='trimmed' if allow_trim else 'frozen_odom',
                  controller_id='zero_turn', target_front_gap_m=requested_gap_m)
        if continuous_table:
            update = self._table_approach_update(target, front_extent_m, to_odom)
            self.emit('table_continuous_approach_started', travel_m=travel_m,
                      target_front_gap_m=requested_gap_m)
            travelled_m = self._drive_zero_turn(
                travel_m, precision=True, hold_heading=True, target_update=update,
                continuous_table=True)
            stopped = getattr(self, '_continuous_table_stop', None)
            if stopped is not None:
                target.update(stopped['target'])
            reached = travelled_m is not None and stopped is not None
        elif (getattr(self, 'surface_mode', 'front') == 'table_edge'
              and self._final_gap_m() <= 0.01):
            self._continuous_table_stop = None
            travelled_m = self._drive_zero_turn(travel_m, precision=True, hold_heading=True)
        else:
            travelled_m = self._drive_zero_turn(travel_m)
        if not continuous_table:
            reached = (travelled_m is not None
                       and travelled_m >= travel_m - FINAL_STRAIGHT_TOLERANCE_M)
        if not reached:
            self.emit('final_approach_short', travel_m=travel_m,
                      travelled_m=None if travelled_m is None else round(travelled_m, 4))
        return reached, to_odom

    def _parking_final_approach(self, target, front_extent_m):
        """
        Close the last gap straight along the aligned heading, held in odom.

        Face alignment has squared the robot to the face. A planned approach to
        the 1 cm goal arrived 11-23 deg off and stopped on RPP collision checks
        (2026-09-30), so the heading is kept and only the gap closes. Returns
        (reached, map-to-odom function of the approach).
        """
        actual, _ = self.capture_stationary_pose()
        face = target['face_center_map_xy_m']
        outward = target['outward_normal_map_xy']
        face_heading = math.atan2(-outward[1], -outward[0])
        facing = -(math.cos(actual[2]) * outward[0] + math.sin(actual[2]) * outward[1])
        tolerance_cos = math.cos(self.parking_contract['yaw_tolerance_rad'])
        basis = 'robot'
        # Alignment squares the base to the face seen from the observation point;
        # the closer re-observation can differ by a few degrees (3.5 deg at table_02,
        # 2026-10-01). A path along the re-observed normal left Parking 1.0-2.5 deg
        # off at its end (four stops 10-01 to 10-02), and the leading corner then
        # 0.4-1.0 cm nearer than the target (review 2026-10-06), so the base turns
        # onto the normal in place first, judged in odom.
        if math.cos(math.pi / 6) < facing < math.cos(FINAL_HEADING_TRIM_RAD):
            residual = math.atan2(math.sin(face_heading - actual[2]),
                                  math.cos(face_heading - actual[2]))
            if self._rotate_in_place(residual) is None:
                self._fail_final('final_trim_missed',
                                 residual_deg=round(math.degrees(residual), 2))
                return False, None
            actual, _ = self.capture_stationary_pose()
            facing = -(math.cos(actual[2]) * outward[0] + math.sin(actual[2]) * outward[1])
            if facing < tolerance_cos:
                self._fail_final('final_alignment_missed',
                                 face_heading_cos=self._json_scalar(facing))
                return False, None
            basis = 'face'
        distance_m = sum((actual[i] - face[i]) * outward[i] for i in (0, 1))
        heading = actual[2]
        along = facing
        # The held heading sets which front corner leads; it, not the edge centre,
        # closes to the target gap.
        lead_m = front_lead_m(front_extent_m, getattr(self, 'box_front_half_width_m', 0.0),
                              math.acos(max(-1.0, min(1.0, along))))
        closing_m = ((distance_m - lead_m - self._final_gap_m()) / along
                     if facing > math.cos(math.pi / 6) else float('nan'))
        if not math.isfinite(closing_m) or not 0.0 < closing_m <= 0.6:
            self._fail_final('final_approach_unavailable',
                             face_heading_cos=self._json_scalar(facing),
                             travel_m=self._json_scalar(closing_m))
            return False, None
        controller = getattr(self, 'final_approach_controller', 'Parking')
        travel_m = closing_m + (FINAL_STOP_SHORT_M if controller == 'Parking' else 0.0)
        steps = int(math.ceil(travel_m / 0.05)) + 1
        points = [(actual[0] + travel_m * k / steps * math.cos(heading),
                   actual[1] + travel_m * k / steps * math.sin(heading))
                  for k in range(steps + 1)]
        end = {'id': 'final_approach', 'x': points[-1][0], 'y': points[-1][1],
               'yaw': heading}
        saved = (self.config, self.waypoints)
        self.config = {'frame_id': 'map', 'waypoints': [end]}
        self.waypoints = self.config['waypoints']
        try:
            path = RosPath()
            path.poses = [self._pose(0, {'x': x, 'y': y, 'yaw': heading})
                          for x, y in points]
            path.header = path.poses[0].header
            odom_path, to_odom = self._frozen_in_odom(path)
            end_x, end_y, end_yaw = to_odom(end['x'], end['y'], end['yaw'])
            self.emit('final_approach_straight', travel_m=travel_m,
                      face_heading_cos=facing, heading_basis=basis,
                      lead_m=lead_m, controller_id=controller)
            reached = self._execute_reverse_path(
                path, send_path=odom_path,
                controller_id=controller,
                verify_contract={**self.parking_contract, 'reference_frame': 'odom'},
                verify_waypoint={'x': end_x, 'y': end_y, 'yaw': end_yaw})
        finally:
            self.config, self.waypoints = saved
        return bool(reached), to_odom

    def _leave_parked_pose(self, *, clearance_m=ESCAPE_CLEARANCE_M):
        """
        Back straight away from the last box face before any home rotation.

        At the final gap the LiDAR cannot see the face, so an in-place turn
        there is unprotected. The band inside the current chassis front is not
        validated because a straight reverse never re-enters it.
        """
        if (isinstance(clearance_m, bool) or not isinstance(clearance_m, (int, float))
                or not math.isfinite(clearance_m) or not .2 <= clearance_m <= ESCAPE_CLEARANCE_M):
            raise ValueError('invalid table reobservation clearance')
        face = getattr(self, 'last_box_face', None)
        if face is None:
            return True
        if not (self._parking_parameters_ready(reverse=True)
                and self._reverse_smoother_ready()):
            self.emit('failed', phase='box_escape', reason='box_escape_unavailable')
            return False
        reference_frame = face.get('reference_frame', 'map')
        actual_map, _ = self.capture_stationary_pose(intermediate=True)
        if reference_frame == 'odom':
            center = face.get('face_center_xy_m')
            outward = face.get('outward_normal_xy')
            actual = self._odom_pose()
        elif reference_frame == 'map':
            center = face.get('face_center_map_xy_m')
            outward = face.get('outward_normal_map_xy')
            actual = actual_map
        else:
            center = outward = None
            actual = actual_map
        if (not isinstance(center, (tuple, list)) or len(center) != 2
                or not isinstance(outward, (tuple, list)) or len(outward) != 2
                or not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                           and math.isfinite(value) for value in (*center, *outward))):
            self.emit('failed', phase='box_escape', reason='box_escape_unavailable',
                      detail='face_reference_invalid')
            return False
        distance_m = sum((actual[i] - center[i]) * outward[i] for i in (0, 1))
        clear_m = clearance_m - ESCAPE_PATH_CONTRACT['xy_tolerance_m']
        if math.isfinite(distance_m) and distance_m >= clear_m:
            # The rotation room is already there, whatever the heading
            # (2026-09-30 table_02: a reverse ended 33 deg off at 0.554 m).
            self.emit('box_escape_skipped', face_distance_m=distance_m,
                      reason='rotation_clearance_reached')
            return True
        # A straight reverse moves away from the face only from a pose that
        # faces it and is in front of it; refuse any other.
        facing = -(math.cos(actual[2]) * outward[0] + math.sin(actual[2]) * outward[1])
        aligned = facing >= math.cos(ESCAPE_MAX_FACE_ANGLE_RAD)
        if not (aligned and 0.0 < distance_m):
            self.emit('failed', phase='box_escape', reason='box_escape_unavailable',
                      detail=('heading_not_facing_box_face' if not aligned
                              else 'face_distance_outside_escape_range'),
                      face_distance_m=self._json_scalar(distance_m),
                      face_heading_cos=self._json_scalar(facing))
            return False
        length_m = clearance_m - distance_m
        target_reference = (actual[0] - length_m * math.cos(actual[2]),
                            actual[1] - length_m * math.sin(actual[2]), actual[2])
        if reference_frame == 'odom':
            turn = actual_map[2] - actual[2]
            dx, dy = target_reference[0] - actual[0], target_reference[1] - actual[1]
            target = (actual_map[0] + math.cos(turn) * dx - math.sin(turn) * dy,
                      actual_map[1] + math.sin(turn) * dx + math.cos(turn) * dy,
                      actual_map[2])
        else:
            target = target_reference
        saved = (self.config, self.waypoints)
        self.config = {'frame_id': 'map', 'waypoints': [
            {'id': 'box_escape', 'x': target[0], 'y': target[1], 'yaw': target[2]}]}
        self.waypoints = self.config['waypoints']
        failure = 'box_escape_unavailable'
        try:
            path = self._make_reverse_path(
                actual_map, target, path_contract=ESCAPE_PATH_CONTRACT)
            if not self._reverse_path_valid(
                    path, validate_from_m=ESCAPE_VALIDATION_EXCLUDE_M):
                self.emit('failed', phase='box_escape', reason='box_escape_blocked')
                return False
            failure = 'box_escape_failed'
            # Straight back in odom: AMCL jumps near the box cannot bend it.
            odom_path, to_odom = self._frozen_in_odom(path)
            reached = self._execute_reverse_path(
                path, path_contract=ESCAPE_PATH_CONTRACT,
                validate_from_m=ESCAPE_VALIDATION_EXCLUDE_M,
                goal_checker_id='alignment_goal_checker', final=False,
                send_path=odom_path)
            if not reached and self.stop_requested:
                # The preceding result event carries the Nav2 error code.
                self.emit('failed', phase='box_escape', reason='box_escape_failed')
                return False
        except (ValueError, RuntimeError) as error:
            # Record the failed escape step; the caller still receives the error.
            self.emit('failed', phase='box_escape', reason=failure,
                      error_type=type(error).__name__, detail=str(error))
            raise
        finally:
            self.config, self.waypoints = saved
        after, _ = self.capture_stationary_pose(intermediate=True)
        position_error_m, yaw_error_rad = pose_errors(target, after)
        # The reverse ran in odom, so its clearance is judged there too: AMCL
        # read 0.5146 m after an odom-straight escape Nav2 completed (2026-09-30).
        odom_x, odom_y, _ = self._odom_pose()
        if reference_frame == 'odom':
            face_x, face_y = center
            tip_x, tip_y = center[0] + outward[0], center[1] + outward[1]
        else:
            face_x, face_y, _ = to_odom(center[0], center[1], 0.0)
            tip_x, tip_y, _ = to_odom(
                center[0] + outward[0], center[1] + outward[1], 0.0)
        distance_m = ((odom_x - face_x) * (tip_x - face_x)
                      + (odom_y - face_y) * (tip_y - face_y))
        # The escape only has to give the next rotation room: confirm the face
        # clearance. Heading and lateral drift are logged; the next leg is planned
        # from wherever the robot stands (2026-09-30: 6 deg drift stopped the run).
        # A reverse Nav2 ended short also counts once the clearance is there
        # (2026-09-30: 105 near the goal at 0.526 m and 0.554 m).
        if not math.isfinite(distance_m) or distance_m < clear_m:
            self.emit('failed', phase='box_escape',
                      reason='box_escape_not_confirmed' if reached else 'box_escape_failed',
                      face_distance_m=distance_m, position_error_m=position_error_m,
                      yaw_error_rad=yaw_error_rad)
            return False
        self.emit('box_escape_finished', face_distance_m=distance_m,
                  position_error_m=position_error_m, yaw_error_rad=yaw_error_rad,
                  nav2_goal_reached=bool(reached))
        self.last_box_face = None
        return True

    def resume_parked(self, face, dwell_s):
        """
        Continue from a stop an earlier run left at a box: hold, then escape.

        A new process has no final command for wait_parked(), so the dwell is a
        stationary hold confirmed before and after; the escape uses the face that
        run observed, not a new observation (the face is inside the depth minimum).
        """
        (x, y, _yaw), _ = self.capture_stationary_pose(intermediate=True)
        self.emit('resume_parked_started', dwell_s=dwell_s,
                  face_center_map_xy_m=face['face_center_map_xy_m'])
        x_after, y_after = x, y
        if dwell_s > 0.0:
            if not self._hold_still(dwell_s):
                return False
            (x_after, y_after, _yaw_after), _ = self.capture_stationary_pose(intermediate=True)
        moved_m = math.dist((x, y), (x_after, y_after))
        if moved_m > 0.02:
            self.emit('failed', phase='resume_parked_dwell', moved_m=moved_m)
            return False
        self.emit('parked_dwell_complete', dwell_s=dwell_s, moved_m=moved_m, resumed=True)
        if getattr(self, 'config', None) is None:
            self.config, self.waypoints = {'frame_id': 'map', 'waypoints': []}, []
        self.last_box_face = face
        if not self._leave_parked_pose():
            return False
        self.emit('resume_parked_finished')
        return True

    def _wait_optional_box_dwell(self, dwell_s):
        """Reuse this visit's final stop when no additional service hold is requested."""
        if dwell_s > 0.0:
            return self.wait_parked(dwell_s)
        evidence = getattr(self, '_confirmed_box_stop', None)
        revision = getattr(self, 'parking_motion_revision', None)
        if (not isinstance(evidence, dict) or not isinstance(revision, int)
                or evidence['motion_revision'] != revision
                or evidence['selected_pose'] != self.selected_pose
                or self.stop_requested or not self._operator_link_ready()
                or not self._navigation_ready(require_fresh_amcl=False)):
            self.emit('parked_handoff_unavailable', reason='final_stop_not_current')
            return False
        self.emit('parked_dwell_complete', dwell_s=0.0,
                  reused_final_stop=True,
                  gap_confirmation_basis=evidence['gap_confirmation_basis'])
        return True

    def dwell_and_leave(self, dwell_s):
        """Hold at a verified intermediate stop, then back straight away from its face."""
        if not self._wait_optional_box_dwell(dwell_s):
            self.emit('failed', phase='via_dwell')
            return False
        if not self._leave_parked_pose():
            return False
        self.emit('via_stop_finished')
        return True

    def dwell_and_return_home(self, dwell_s, timeout_s):
        """Hold at the verified final target, then return to the dock."""
        if not self._wait_optional_box_dwell(dwell_s):
            self.emit('failed', phase='table_dwell')
            return False
        # The escape uses the face observed on the way in; the dock needs no camera.
        self._camera(False)
        return self.go_home(execute=True, timeout_s=timeout_s)


def parse_args(argv=None):
    """Require map-bound inputs and explicit execution, defaulting to plan only."""
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('registry', 'approach-route', 'camera-mount', 'geometry', 'log'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--table-id', required=True)
    parser.add_argument('--station-profiles', type=Path,
                        help='Map-bound taught station and optional RGB-D reference metadata')
    parser.add_argument('--region-xy', nargs=2, type=float, required=True)
    parser.add_argument('--region-radius-m', type=float, default=0.6)
    parser.add_argument('--parking-contract', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument(
        '--search', action='store_true',
        help='Search for the box with at most twelve 30-degree Nav2 spins')
    parser.add_argument(
        '--resume-at-observation', action='store_true',
        help='Resume from the already reached observation waypoint')
    parser.add_argument(
        '--resume-at-final-approach', action='store_true',
        help='At a reached table_edge observation, skip 45 cm face alignment and '
             'run the observed final approach only')
    parser.add_argument(
        '--candidate-trial', action='store_true',
        help='Explicitly allow the nominal-camera candidate parking trial')
    parser.add_argument(
        '--task-timeout-s', type=float, default=240.0,
        help='Motion and search budget from transit start to the final stop')
    parser.add_argument(
        '--return-home', action='store_true',
        help='After a confirmed approach, hold --dwell-s, escape and dock')
    parser.add_argument(
        '--return-timeout-s', type=float,
        help='Budget for the escape and dock return; required by --return-home')
    parser.add_argument(
        '--via-id', choices=('water_station',),
        help='First stop before the table: approach, hold --dwell-s and escape')
    parser.add_argument('--graceful-final', action='store_true',
                        help='use the Graceful controllers for the box approach and dock '
                             'leg instead of RPP (trajectory collision check, see 105)')
    parser.add_argument('--zero-turn-final', action='store_true',
                        help='final box approach: turn the residual to the face out in '
                             'place, then close the gap at zero turn rate (precision trial)')
    parser.add_argument('--pointcloud-perception', action='store_true',
                        help='send the selected table region to the optional pointcloud observer')
    parser.add_argument('--rpp-transit', action='store_true',
                        help='follow the transit and dock staging legs with RPP on NavFn '
                             'paths (the default; kept for older commands)')
    parser.add_argument('--transit', choices=TRANSIT_VARIANTS,
                        help='planner + controller for the transit and dock staging legs '
                             '(default navfn-rpp, see DEFAULT_TRANSIT)')
    parser.add_argument('--camera-always-on', action='store_true',
                        help='keep the box camera and observer running (before 2026-10-06) '
                             'instead of starting them for the box stops only')
    parser.add_argument('--surface-mode', choices=SURFACE_MODES, default='front',
                        help='parking surface: legacy box front or tabletop front edge')
    parser.add_argument('--home-only', action='store_true',
                        help='return to the dock only (needs --execute and --return-home)')
    parser.add_argument('--rear-box-only', action='store_true',
                        help='park against the rear LiDAR box at the current location')
    parser.add_argument('--via-route', type=Path)
    parser.add_argument('--via-region-xy', nargs=2, type=float)
    parser.add_argument('--via-region-radius-m', type=float, default=0.6)
    parser.add_argument(
        '--resume-parked-from-log', type=Path,
        help='An earlier run stopped at the first box (via stop if given, else the table): '
             'hold --dwell-s, escape from the face logged there, then continue')
    parser.add_argument(
        '--operator-heartbeat', type=Path,
        help='file the operator PC touches while it follows the run; no further goal '
             'starts once it is older than --operator-link-timeout-s')
    parser.add_argument('--operator-link-timeout-s', type=float,
                        default=OPERATOR_LINK_TIMEOUT_S)
    parser.add_argument('--final-gap-m', type=float,
                        help="front gap override; defaults to the parking contract's "
                             'target_front_gap_m for either surface mode')
    parser.add_argument('--dwell-s', type=float, default=STOP_DWELL_S,
                        help='held stop at each box (longer for an external gap measurement)')
    args = parser.parse_args(argv)
    if args.pointcloud_perception and (
            args.surface_mode != 'table_edge' or args.home_only or args.rear_box_only):
        parser.error('--pointcloud-perception requires a table_edge approach task')
    if args.final_gap_m is not None and not (
            math.isfinite(args.final_gap_m)
            and MINIMUM_FRONT_GAP_M <= args.final_gap_m <= MAXIMUM_FRONT_GAP_M):
        parser.error('--final-gap-m must be in [0.01, 0.10] m')
    if not (math.isfinite(args.dwell_s) and 0.0 <= args.dwell_s <= 120.0):
        parser.error('--dwell-s must be in [0, 120] s')
    if (not math.isfinite(args.operator_link_timeout_s)
            or args.operator_link_timeout_s <= 0.0):
        parser.error('--operator-link-timeout-s must be finite and positive')
    if (not all(math.isfinite(v) for v in args.region_xy)
            or not math.isfinite(args.region_radius_m)
            or not 0.0 < args.region_radius_m <= 0.6):
        parser.error('table region must be finite and radius in (0, 0.6] m')
    if args.execute and not args.candidate_trial:
        parser.error('--execute requires --candidate-trial')
    if args.search and not args.execute:
        parser.error('--search requires --execute')
    if args.resume_at_observation and not args.execute:
        parser.error('--resume-at-observation requires --execute')
    if args.resume_at_final_approach and not args.execute:
        parser.error('--resume-at-final-approach requires --execute')
    if args.resume_at_final_approach and not args.resume_at_observation:
        parser.error('--resume-at-final-approach requires --resume-at-observation')
    if args.resume_at_final_approach and args.surface_mode != 'table_edge':
        parser.error('--resume-at-final-approach requires --surface-mode table_edge')
    if not math.isfinite(args.task_timeout_s) or args.task_timeout_s <= 0.0:
        parser.error('--task-timeout-s must be finite and positive')
    if args.return_home and not args.execute:
        parser.error('--return-home requires --execute')
    if args.return_home and args.return_timeout_s is None:
        parser.error('--return-home requires --return-timeout-s')
    if args.home_only and not args.return_home:
        parser.error('--home-only requires --execute and --return-home')
    if args.rear_box_only and (args.home_only or args.return_home or args.via_id
                               or args.resume_parked_from_log):
        parser.error('--rear-box-only cannot be combined with a transit or return task')
    if args.return_timeout_s is not None and (
            not math.isfinite(args.return_timeout_s) or args.return_timeout_s <= 0.0):
        parser.error('--return-timeout-s must be finite and positive')
    via = (args.via_id, args.via_route, args.via_region_xy)
    if any(value is not None for value in via):
        if any(value is None for value in via):
            parser.error('--via-id, --via-route and --via-region-xy go together')
        if not args.return_home:
            parser.error('a via stop requires --return-home')
        if (not all(math.isfinite(v) for v in args.via_region_xy)
                or not math.isfinite(args.via_region_radius_m)
                or not 0.0 < args.via_region_radius_m <= 0.6):
            parser.error('via region must be finite and radius in (0, 0.6] m')
    if args.resume_parked_from_log is not None:
        if not (args.execute and args.return_home):
            parser.error('--resume-parked-from-log requires --execute and --return-home')
        if args.resume_at_observation:
            parser.error('--resume-parked-from-log cannot be combined with '
                         '--resume-at-observation')
    return args


def run_attempt(args, active=None):
    """
    Run one table attempt in an initialized rclpy context; return the exit code.

    A one-shot run installs its own stop handlers. The resident executor
    (serve) passes `active` instead and routes its signals to the node there.
    """
    registry = load_registry(args.registry)
    mount = yaml.safe_load(args.camera_mount.read_text())
    geometry = yaml.safe_load(args.geometry.read_text())
    contract = load_parking_contract(args.parking_contract)
    node = None
    handlers = {}
    with args.log.open('x', encoding='utf-8') as stream:
        try:
            # The dock is accepted with the user's contract, not the box
            # contract; a load failure takes the failure path below.
            home_contract = load_parking_contract(
                Path(get_package_share_directory('jdamr_cube_navigation'))
                / 'config/parking_contract.yaml')
            node = BoxServiceRoute(registry, contract, stream, home_contract=home_contract)
            if active is None:
                for signum in (signal.SIGINT, signal.SIGTERM):
                    handlers[signum] = signal.signal(
                        signum, lambda *_: node.request_operator_stop())
            else:
                active['node'] = node
                if active.get('stop_requested', False):
                    node.request_operator_stop()
                    return 1
            node.home_rear_geometry = geometry
            node.home_rear_route_path = args.approach_route
            if args.graceful_final:
                node.final_approach_controller = 'GracefulParking'
                node.dock_leg_controller = 'GracefulReverse'
            if args.rpp_transit:
                node.use_rpp_transit()
            if args.transit:
                node.use_transit(args.transit)
            node.camera_on_demand = not args.camera_always_on
            node.surface_mode = args.surface_mode
            node.pointcloud_perception = getattr(args, 'pointcloud_perception', False)
            node.require_observer_configuration = True
            node.zero_turn_final = args.zero_turn_final
            node.operator_heartbeat = args.operator_heartbeat
            node.operator_link_timeout_s = args.operator_link_timeout_s
            node.final_gap_override_m = args.final_gap_m
            ok = True
            if getattr(args, 'rear_box_only', False):
                ok = node.park_rear_box(
                    geometry, args.approach_route,
                    execute=args.execute, timeout_s=args.task_timeout_s)
                return 0 if ok else 1
            if args.home_only:
                # Dock return alone, e.g. to re-dock after a crooked stop; it may run
                # down to the running cutoff, since it goes to the charger.
                node.home_return = True
                node.verify_live_maps()
                if not node.wait_until_ready(timeout=10.0):
                    raise RuntimeError(node.readiness_failure_reason())
                ok = node.go_home(execute=True, timeout_s=args.return_timeout_s)
                return 0 if ok else 1
            if args.resume_parked_from_log is not None:
                # The first stop was reached by an earlier run: hold and escape only.
                face = last_logged_face(args.resume_parked_from_log)
                node.verify_live_maps()
                if not node.wait_until_ready(timeout=10.0):
                    raise RuntimeError(node.readiness_failure_reason())
                ok = node.resume_parked(face, args.dwell_s)
                if args.via_id is None:
                    # The parked stop was the table: its escape leads straight home.
                    ok = ok and node.go_home(execute=True, timeout_s=args.return_timeout_s)
                    return 0 if ok else 1
            via_done = False
            if (args.resume_parked_from_log is None and args.via_id is not None
                    and args.resume_at_observation
                    and node.at_observation(args.approach_route)):
                # Stopped at the table's observation point (2026-10-01 15:49): the
                # water stop is already done, so the resume belongs to the table.
                node.emit('via_skipped', via_id=args.via_id,
                          reason='robot is at the table observation region')
                via_done = True
            elif args.resume_parked_from_log is None and args.via_id is not None:
                # The via stop is a full box visit; its escape starts the table route.
                if args.station_profiles is not None:
                    node.configure_station_reference(args.station_profiles, args.via_id)
                ok = node.visit_observed_box(
                    args.via_route, mount, geometry, args.via_id,
                    args.via_region_xy, args.via_region_radius_m, execute=args.execute,
                    candidate_trial=args.candidate_trial,
                    resume_at_observation=args.resume_at_observation,
                    resume_at_final_approach=args.resume_at_final_approach,
                    search=args.search, task_timeout_s=args.task_timeout_s)
                ok = ok and node.dwell_and_leave(args.dwell_s)
                if ok and node.battery_return_due():
                    # Low between stops: skip the next stop and return to the dock.
                    node.go_home(execute=True, timeout_s=args.return_timeout_s)
                    return 1
            # A resume applies to the first stop of this run only.
            if ok and args.station_profiles is not None:
                node.configure_station_reference(args.station_profiles, args.table_id)
            ok = ok and node.visit_observed_box(
                args.approach_route, mount, geometry, args.table_id,
                args.region_xy, args.region_radius_m, execute=args.execute,
                candidate_trial=args.candidate_trial,
                resume_at_observation=(args.resume_at_observation
                                       and (args.via_id is None or via_done)),
                resume_at_final_approach=(args.resume_at_final_approach
                                          and (args.via_id is None or via_done)),
                search=args.search, task_timeout_s=args.task_timeout_s)
            # Exit 0 only when every requested stage, including the return, succeeded.
            if ok and args.return_home:
                ok = node.dwell_and_return_home(args.dwell_s, args.return_timeout_s)
            elif (args.return_home and getattr(node, 'incomplete_gap_stop', False)
                    and not node.stop_requested and node._operator_link_ready()
                    and node._navigation_ready(require_fresh_amcl=False)):
                node.emit('incomplete_parking_return_started', parking_success=False)
                node._camera(False)
                returned = node.go_home(execute=True, timeout_s=args.return_timeout_s)
                node.emit('incomplete_parking_return_finished', home_confirmed=returned,
                          parking_success=False)
            return 0 if ok else 1
        except (ValueError, RuntimeError) as error:
            if node is not None:
                node.emit('failed', reason=str(error))
            print(json.dumps({'failed': str(error)}))
            return 1
        finally:
            try:
                if node is not None and not node.finish_navigation():
                    raise RuntimeError('navigation cancellation unconfirmed')
            finally:
                for signum, handler in handlers.items():
                    signal.signal(signum, handler)
                if active is not None:
                    active['node'] = None
                if node is not None:
                    # Every attempt ends with the camera off, failed ones included.
                    try:
                        if not node._camera(False):
                            node.emit('failed', phase='cleanup', reason='camera_stop_failed')
                            raise RuntimeError('camera stop unconfirmed')
                    finally:
                        node.destroy_node()


def serve(spool):
    """
    Keep one rclpy context (one DDS participant) and run requested attempts in it.

    Every one-shot run was a new participant that rediscovered the graph and
    twice missed the /cmd_vel publisher before moving (2026-10-01). A request
    is <spool>/<id>.request holding {"argv": [...]} (moved in whole); while it
    runs it is <id>.running, and <id>.result holds {"code": n}. Each attempt
    gets a fresh node, so no state carries over. SIGINT stops the running
    attempt; SIGTERM stops it and ends the executor.
    """
    spool = Path(spool)
    spool.mkdir(parents=True, exist_ok=True)
    active = {'node': None, 'stop_requested': False}
    leaving = {'now': False}

    def stop_attempt(*_):
        active['stop_requested'] = True
        if active['node'] is not None:
            active['node'].request_operator_stop()

    def leave(*_):
        leaving['now'] = True
        stop_attempt()

    previous = {signal.SIGINT: signal.signal(signal.SIGINT, stop_attempt),
                signal.SIGTERM: signal.signal(signal.SIGTERM, leave)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        while not leaving['now']:
            requests = sorted(spool.glob('*.request'))
            if not requests:
                time.sleep(0.2)
                continue
            request = requests[0]
            name = request.name[:-len('.request')]
            running = spool / f'{name}.running'
            with (spool / f'{name}.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                if (spool / f'{name}.cancelled').exists():
                    request.unlink(missing_ok=True)
                    continue
                active['stop_requested'] = False
                try:
                    request.rename(running)
                except FileNotFoundError:
                    # The PC cancelled an unclaimed request before this lock.
                    continue
            try:
                code = run_attempt(parse_args(json.loads(running.read_text())['argv']), active)
            except SystemExit as error:   # argparse rejected the request
                code = error.code if isinstance(error.code, int) else 2
            except Exception as error:  # noqa: B902 - report, keep serving
                print(json.dumps({'failed': f'{type(error).__name__}: {error}'}), flush=True)
                code = 1
            with (spool / f'{name}.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                result = spool / f'{name}.result'
                temporary = result.with_suffix('.result.tmp')
                temporary.write_text(json.dumps({'code': code}))
                temporary.replace(result)
                running.unlink()
        return 0
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        rclpy.shutdown()


def main(argv=None):
    """Run one table attempt, or serve attempts with --serve SPOOL_DIR."""
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ['--serve']:
        if len(argv) != 2:
            raise SystemExit('usage: box_service --serve SPOOL_DIR')
        return serve(argv[1])
    args = parse_args(argv)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        return run_attempt(args)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
