#!/usr/bin/env python3
"""AMMR 실기체 센서 관제 대시보드.

ROS 2 토픽을 구독만 하며 publisher·service client·로봇 명령 경로를 만들지 않는다.
HTTP에서는 최신 상태 JSON과 RGB/Depth MJPEG만 제공한다.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import pathlib
import signal
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np


HERE = pathlib.Path(__file__).resolve().parent
STALE_SECONDS = 2.0
RENDER_PERIOD_SECONDS = 0.18


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def rolling_fps(timestamps):
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    return 0.0 if span <= 0 else (len(timestamps) - 1) / span


def scan_summary(ranges, angle_min, angle_increment, range_min, range_max,
                 max_points=240):
    """LaserScan을 방향별 최소거리와 화면용 XY 점으로 줄인다."""
    valid = []
    for index, raw in enumerate(ranges):
        distance = finite(raw)
        if distance is None or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        valid.append((angle, distance))

    def sector(center_deg, half_deg):
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        values = [distance for angle, distance in valid
                  if abs(math.atan2(math.sin(angle - center),
                                    math.cos(angle - center))) <= half]
        return None if not values else round(min(values), 3)

    stride = max(1, math.ceil(len(valid) / max_points))
    points = [[round(distance * math.cos(angle), 3),
               round(distance * math.sin(angle), 3)]
              for angle, distance in valid[::stride]][:max_points]
    distances = [distance for _, distance in valid]
    return {
        "valid_points": len(valid),
        "min_m": None if not distances else round(min(distances), 3),
        "front_m": sector(0, 18),
        "left_m": sector(90, 24),
        "right_m": sector(-90, 24),
        "points_xy_m": points,
    }


def decode_depth(msg):
    """sensor_msgs/Image의 16UC1/mono16/32FC1을 미터 배열로 변환한다."""
    encoding = str(msg.encoding).lower()
    if encoding in ("16uc1", "mono16"):
        dtype, scale = np.dtype("u2"), 0.001
    elif encoding == "32fc1":
        dtype, scale = np.dtype("f4"), 1.0
    else:
        raise ValueError(f"지원하지 않는 depth encoding: {msg.encoding}")
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    row_values = int(msg.step) // dtype.itemsize
    array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_values)
    return array[:, :msg.width].astype(np.float32) * scale


def depth_visual(depth_m):
    """0.2~5.0m 깊이를 가까운 곳=주황, 먼 곳=남색인 JPEG로 만든다."""
    valid = np.isfinite(depth_m) & (depth_m > 0.05)
    clipped = np.clip(depth_m, 0.2, 5.0)
    normalized = ((clipped - 0.2) / 4.8 * 255).astype(np.uint8)
    image = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
    image[~valid] = (21, 25, 35)
    return image


def depth_summary(depth_m):
    valid = np.isfinite(depth_m) & (depth_m > 0.05)
    values = depth_m[valid]
    height, width = depth_m.shape
    radius = max(2, min(height, width) // 40)
    center = depth_m[height // 2 - radius:height // 2 + radius + 1,
                     width // 2 - radius:width // 2 + radius + 1]
    center = center[np.isfinite(center) & (center > 0.05)]
    return {
        "width": width,
        "height": height,
        "valid_pct": round(float(valid.mean() * 100), 1),
        "center_m": None if not center.size else round(float(np.median(center)), 3),
        "nearest_m": None if not values.size else round(float(np.percentile(values, 1)), 3),
    }


def image_to_bgr(msg):
    encoding = str(msg.encoding).lower()
    channels = 1 if encoding in ("mono8", "8uc1") else 3
    row_values = int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, row_values)
    if channels == 1:
        return cv2.cvtColor(raw[:, :msg.width], cv2.COLOR_GRAY2BGR)
    image = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in ("bgr8", "8uc3"):
        return image
    raise ValueError(f"지원하지 않는 color encoding: {msg.encoding}")


class TelemetryStore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.frame_ready = {"depth": threading.Condition(self.lock),
                            "color": threading.Condition(self.lock)}
        self.frames = {"depth": None, "color": None}
        self.sequences = {"depth": 0, "color": 0}
        self.last_render_at = {"depth": 0.0, "color": 0.0}
        self.times = {"depth": deque(maxlen=90), "color": deque(maxlen=90),
                      "scan": deque(maxlen=90)}
        self.state = {
            "started_at": self.clock(),
            "depth": {"topic": None, "error": None},
            "color": {"topic": None, "error": None},
            "lidar": {"topic": "/scan"},
            "motion": {"odom": None, "cmd": None, "requested_cmd": None},
            "battery": None,
            "imu": None,
            "graph": {"nodes": 0, "topics": 0},
        }

    def _stamp(self, key):
        now = self.clock()
        self.times[key].append(now)
        return now

    def update_depth(self, msg, topic):
        now = self._stamp("depth")
        if now - self.last_render_at["depth"] < RENDER_PERIOD_SECONDS:
            return
        self.last_render_at["depth"] = now
        try:
            depth = decode_depth(msg)
            summary = depth_summary(depth)
            image = depth_visual(depth)
            cv2.drawMarker(image, (image.shape[1] // 2, image.shape[0] // 2),
                           (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
            ok, encoded = cv2.imencode(".jpg", image,
                                      [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                raise RuntimeError("JPEG 인코딩 실패")
            with self.lock:
                self.frames["depth"] = encoded.tobytes()
                self.sequences["depth"] += 1
                self.state["depth"] = {
                    **summary, "topic": topic, "encoding": msg.encoding,
                    "fps": round(rolling_fps(self.times["depth"]), 1),
                    "observed_at": now, "error": None,
                }
                self.frame_ready["depth"].notify_all()
        except Exception as exc:
            with self.lock:
                self.state["depth"]["error"] = f"{type(exc).__name__}: {exc}"

    def update_color(self, msg, topic):
        now = self._stamp("color")
        if now - self.last_render_at["color"] < RENDER_PERIOD_SECONDS:
            return
        self.last_render_at["color"] = now
        try:
            image = image_to_bgr(msg)
            ok, encoded = cv2.imencode(".jpg", image,
                                      [cv2.IMWRITE_JPEG_QUALITY, 78])
            if not ok:
                raise RuntimeError("JPEG 인코딩 실패")
            with self.lock:
                self.frames["color"] = encoded.tobytes()
                self.sequences["color"] += 1
                self.state["color"] = {
                    "topic": topic, "encoding": msg.encoding,
                    "width": msg.width, "height": msg.height,
                    "fps": round(rolling_fps(self.times["color"]), 1),
                    "observed_at": now, "error": None,
                }
                self.frame_ready["color"].notify_all()
        except Exception as exc:
            with self.lock:
                self.state["color"]["error"] = f"{type(exc).__name__}: {exc}"

    def update_scan(self, msg):
        now = self._stamp("scan")
        summary = scan_summary(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max)
        with self.lock:
            self.state["lidar"] = {
                **summary, "topic": "/scan", "observed_at": now,
                "fps": round(rolling_fps(self.times["scan"]), 1),
            }

    def update_motion(self, key, linear, angular, **extra):
        with self.lock:
            self.state["motion"][key] = {
                "linear_mps": round(float(linear), 4),
                "angular_rps": round(float(angular), 4),
                "observed_at": self.clock(), **extra,
            }

    def update_battery(self, msg):
        percentage = finite(msg.percentage)
        if percentage is not None and 0 <= percentage <= 1:
            percentage *= 100
        with self.lock:
            self.state["battery"] = {
                "voltage_v": finite(msg.voltage),
                "percentage_pct": percentage,
                "present": bool(msg.present), "observed_at": self.clock(),
            }

    def update_imu(self, msg):
        with self.lock:
            self.state["imu"] = {
                "angular_z_rps": round(float(msg.angular_velocity.z), 4),
                "accel_x_mps2": round(float(msg.linear_acceleration.x), 3),
                "accel_y_mps2": round(float(msg.linear_acceleration.y), 3),
                "observed_at": self.clock(),
            }

    def update_graph(self, node):
        with self.lock:
            self.state["graph"] = {
                "nodes": len(node.get_node_names_and_namespaces()),
                "topics": len(node.get_topic_names_and_types()),
                "observed_at": self.clock(),
            }

    def snapshot(self):
        now = self.clock()
        with self.lock:
            result = copy.deepcopy(self.state)
        result["server"] = {
            "uptime_s": round(now - result.pop("started_at"), 1),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        }
        for group in ("depth", "color", "lidar", "battery", "imu"):
            item = result.get(group)
            if item:
                observed = item.pop("observed_at", None)
                item["age_s"] = None if observed is None else round(now - observed, 2)
                item["live"] = item["age_s"] is not None and item["age_s"] < STALE_SECONDS
        for item in result["motion"].values():
            if item:
                observed = item.pop("observed_at", None)
                item["age_s"] = None if observed is None else round(now - observed, 2)
        result["system"] = system_snapshot()
        return result

    def wait_frame(self, kind, after, timeout=2.0):
        deadline = self.clock() + timeout
        with self.frame_ready[kind]:
            while self.sequences[kind] <= after:
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return self.sequences[kind], self.frames[kind]
                self.frame_ready[kind].wait(remaining)
            return self.sequences[kind], self.frames[kind]


def system_snapshot():
    load = os.getloadavg()
    memory = {}
    try:
        for line in pathlib.Path("/proc/meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            memory[key] = int(value.strip().split()[0])
    except (OSError, ValueError):
        pass
    total, available = memory.get("MemTotal"), memory.get("MemAvailable")
    temperature = None
    try:
        temperature = int(pathlib.Path(
            "/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000
    except (OSError, ValueError):
        pass
    return {
        "load_1m": round(load[0], 2),
        "memory_pct": None if not total or available is None else
            round((1 - available / total) * 100, 1),
        "cpu_temp_c": None if temperature is None else round(temperature, 1),
    }


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        server_version = "AMMRDashboard/1.0"

        def log_message(self, fmt, *args):
            return

        def _headers(self, code, content_type, length=None):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if length is not None:
                self.send_header("Content-Length", str(length))
            self.end_headers()

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                body = (HERE / "dashboard.html").read_bytes()
                self._headers(200, "text/html; charset=utf-8", len(body))
                self.wfile.write(body)
            elif path == "/api/state":
                body = json.dumps(store.snapshot(), ensure_ascii=False,
                                  allow_nan=False).encode()
                self._headers(200, "application/json; charset=utf-8", len(body))
                self.wfile.write(body)
            elif path == "/health":
                state = store.snapshot()
                ok = state["depth"].get("live") and state["lidar"].get("live")
                body = json.dumps({"ok": bool(ok), "depth": state["depth"],
                                   "lidar": state["lidar"]},
                                  ensure_ascii=False).encode()
                self._headers(200 if ok else 503, "application/json", len(body))
                self.wfile.write(body)
            elif path in ("/stream/depth.mjpg", "/stream/color.mjpg"):
                self._stream(path.split("/")[-1].split(".")[0])
            else:
                self._headers(404, "text/plain; charset=utf-8", 9)
                self.wfile.write("not found".encode())

        def _stream(self, kind):
            self._headers(200, "multipart/x-mixed-replace; boundary=frame")
            sequence = -1
            try:
                while True:
                    sequence, jpeg = store.wait_frame(kind, sequence)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def build_node(store, depth_topic, color_topic):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState, Image, Imu, LaserScan

    class DashboardNode(Node):
        def __init__(self):
            super().__init__("ammr_dashboard_monitor")
            self.create_subscription(Image, depth_topic,
                                     lambda msg: store.update_depth(msg, depth_topic),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, color_topic,
                                     lambda msg: store.update_color(msg, color_topic),
                                     qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", store.update_scan,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.odom,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel", self.cmd,
                                     qos_profile_sensor_data)
            self.create_subscription(Twist, "/cmd_vel_smoothed", self.requested,
                                     qos_profile_sensor_data)
            self.create_subscription(BatteryState, "/battery_state",
                                     store.update_battery, qos_profile_sensor_data)
            self.create_subscription(Imu, "/imu/data_raw", store.update_imu,
                                     qos_profile_sensor_data)
            self.create_timer(1.0, lambda: store.update_graph(self))

        def odom(self, msg):
            pose = msg.pose.pose
            q = pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                           1 - 2 * (q.y * q.y + q.z * q.z)))
            store.update_motion("odom", msg.twist.twist.linear.x,
                                msg.twist.twist.angular.z,
                                x_m=round(pose.position.x, 3),
                                y_m=round(pose.position.y, 3),
                                yaw_deg=round(yaw, 1))

        def cmd(self, msg):
            store.update_motion("cmd", msg.linear.x, msg.angular.z)

        def requested(self, msg):
            store.update_motion("requested_cmd", msg.linear.x, msg.angular.z)

    if not rclpy.ok():
        rclpy.init()
    return DashboardNode()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    args = parser.parse_args()

    import rclpy
    store = TelemetryStore()
    node = build_node(store, args.depth_topic, args.color_topic)
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(store))
    server.daemon_threads = True
    server.block_on_close = False
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"AMMR 대시보드: http://{args.bind}:{args.port}", flush=True)

    stopping = threading.Event()

    def stop(_signum=None, _frame=None):
        stopping.set()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
