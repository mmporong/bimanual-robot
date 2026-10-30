"""AST1 Windows camera receiver; publishes sensors only, never motion.

Wire order: AST1 + u32be JSON length + JSON + rgb PNG/JPEG + depth PNG16.
PONG records (4 + 8 + 8 bytes) may occur only between complete AST1 records.
"""

import argparse
from collections import deque
import json
import math
import select
import socket
import struct
import threading
import time

import cv2
import numpy as np


FRAME = 'camera_color_optical_frame'
MAX_AGE_NS = 500_000_000


def exact(connection, size):
    chunks = bytearray()
    deadline = time.monotonic() + 3.0
    while len(chunks) < size:
        if time.monotonic() >= deadline:
            raise TimeoutError('incomplete AST1 record')
        try:
            block = connection.recv(min(size - len(chunks), 65536))
        except socket.timeout:
            continue
        if not block:
            raise EOFError('sender disconnected')
        chunks.extend(block)
    return bytes(chunks)


def integer(value, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('invalid integer field')
    return value


def metadata(value, depth=False):
    if not isinstance(value, dict):
        raise ValueError('image metadata must be an object')
    width, height = (320, 240) if depth else (640, 480)
    if (value.get('width'), value.get('height')) != (width, height):
        raise ValueError('unexpected image resolution')
    if value.get('frame_id') != FRAME:
        raise ValueError('unexpected optical frame')
    if value.get('encoding') != ('16UC1' if depth else 'bgr8'):
        raise ValueError('unexpected image encoding')
    if value.get('compression') not in (('png16',) if depth else ('png', 'jpeg')):
        raise ValueError('unexpected compression')
    if depth and value.get('unit') != 'mm':
        raise ValueError('depth must be millimetres')
    integer(value.get('stamp_ns'), 1, 2**63 - 1)
    length = integer(value.get('len'), 1, 2 * 1024 * 1024)
    info = value['camera_info']
    if not isinstance(info, dict):
        raise ValueError('CameraInfo must be an object')
    if (info.get('width'), info.get('height')) != (width, height):
        raise ValueError('CameraInfo resolution mismatch')
    if info.get('distortion_model') != 'plumb_bob':
        raise ValueError('unexpected distortion model')
    for key, size in (('D', 5), ('K', 9), ('R', 9), ('P', 12)):
        values = info.get(key)
        if (not isinstance(values, list) or len(values) != size
                or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            raise ValueError('invalid CameraInfo ' + key)
    if min(info['K'][0], info['K'][4], info['P'][0], info['P'][5]) <= 0:
        raise ValueError('invalid focal length')
    return length


def jpeg_shape(payload):
    """Read SOF dimensions before allowing the JPEG decoder to allocate."""
    if payload[:2] != b'\xff\xd8':
        raise ValueError('invalid JPEG signature')
    cursor = 2
    while cursor < len(payload):
        if payload[cursor] != 0xff:
            break
        while cursor < len(payload) and payload[cursor] == 0xff:
            cursor += 1
        if cursor >= len(payload):
            break
        marker = payload[cursor]
        cursor += 1
        if marker in (0xd9, 0xda):
            break
        if marker in (0x01, *range(0xd0, 0xd9)):
            continue
        if cursor + 2 > len(payload):
            break
        size = struct.unpack('!H', payload[cursor:cursor + 2])[0]
        if size < 2 or cursor + size > len(payload):
            break
        if marker in (0xc0, 0xc1, 0xc2):
            if size < 8 or payload[cursor + 2] != 8 or payload[cursor + 7] != 3:
                break
            height, width = struct.unpack('!HH', payload[cursor + 3:cursor + 7])
            return height, width
        cursor += size
    raise ValueError('unsupported or incomplete JPEG header')


def decode(payload, meta, depth=False):
    # Validate dimensions before decoding either supported colour codec.
    if not depth and meta['compression'] == 'jpeg':
        if jpeg_shape(payload) != (meta['height'], meta['width']):
            raise ValueError('JPEG header does not match metadata')
    elif (len(payload) < 33 or payload[:8] != b'\x89PNG\r\n\x1a\n'
            or payload[12:16] != b'IHDR'
            or struct.unpack('!II', payload[16:24]) != (meta['width'], meta['height'])
            or payload[24] != (16 if depth else 8)
            or payload[25] != (0 if depth else 2)):
        raise ValueError('PNG header does not match metadata')
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
    expected = (240, 320) if depth else (480, 640, 3)
    if image is None or image.shape != expected or image.dtype != (np.uint16 if depth else np.uint8):
        raise ValueError('decoded image does not match metadata')
    return image


class Receiver:
    def __init__(self, host, port, peer, *, clock_hold_s=120.0, clock_max_rtt_ms=100.0):
        self.host, self.port, self.peer = host, port, peer
        self.clock_hold_s = clock_hold_s
        self.clock_max_rtt_ns = int(clock_max_rtt_ms * 1e6)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.status = {'state': 'listening'}
        self.last_received = 0.0
        self.stage = 'listening'
        self.disconnects = 0
        self.last_disconnect = None
        self.received_frames = 0
        self.accepted_frames = 0
        self.dropped_frames = 0
        self.clock_rejected = 0
        self.last_clock_rtt_ns = None
        self.receive_samples = deque(maxlen=300)
        self.accept_samples = deque(maxlen=300)
        self.last_camera_info = None
        self.published_frames = 0
        self.publish_expired_frames = 0
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(1)
        self.listener.settimeout(.5)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def take(self):
        with self.lock:
            value, self.latest = self.latest, None
            return value

    def current_status(self):
        status = dict(self.status)
        if status.get('state') == 'receiving':
            silence = time.monotonic() - self.last_received
            status['silence_s'] = silence
            if silence > MAX_AGE_NS / 1e9:
                status['state'] = 'stale'
        status.update(stage=self.stage, received_frames=self.received_frames,
                      accepted_frames=self.accepted_frames, dropped_frames=self.dropped_frames,
                      disconnects=self.disconnects, last_disconnect=self.last_disconnect,
                      clock_rejected=self.clock_rejected, last_clock_rtt_ns=self.last_clock_rtt_ns,
                      camera_info=self.last_camera_info, published_frames=self.published_frames,
                      publish_expired_frames=self.publish_expired_frames,
                      clock_hold_s=self.clock_hold_s, clock_max_rtt_ms=self.clock_max_rtt_ns / 1e6)
        now = time.monotonic()
        samples = [x for x in list(self.receive_samples) if now - x[0] < 30.]
        accepted = [x for x in list(self.accept_samples) if now - x[0] < 30.]
        status['accepted_fps'] = 0.0
        if len(samples) > 1:
            elapsed = now - samples[0][0]
            status['receive_fps'] = (len(samples) - 1) / max(elapsed, 1e-9)
            status['receive_MB_s'] = sum(x[1] for x in samples[1:]) / max(elapsed, 1e-9) / 1e6
            status['receive_window_s'] = elapsed
            for index, label in ((2, 'rgb_depth_delta_ms'),
                                 (3, 'rgb_capture_to_send_ms'),
                                 (4, 'depth_capture_to_send_ms')):
                status[label] = dict(zip(('min', 'p50', 'p95', 'max'),
                                         map(float, np.percentile([x[index] for x in samples],
                                                                  [0, 50, 95, 100]))))
        if len(accepted) > 1:
            status['accepted_fps'] = (len(accepted) - 1) / max(now - accepted[0][0], 1e-9)
            for index, label in ((1, 'rgb_capture_to_receive_ms'),
                                 (2, 'depth_capture_to_receive_ms'),
                                 (3, 'rgb_depth_delta_ms'), (4, 'decode_ms')):
                values = [x[index] for x in accepted]
                status[label] = dict(zip(('min', 'p50', 'p95', 'max'),
                                         map(float, np.percentile(values, [0, 50, 95, 100]))))
        return status

    def run(self):
        while not self.stop.is_set():
            try:
                connection, address = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                if self.peer and address[0] != self.peer:
                    continue
                connection.settimeout(.5)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    self.session(connection, address[0])
                except (OSError, EOFError, ValueError, KeyError, TypeError, struct.error, cv2.error) as error:
                    self.disconnects += 1
                    self.last_disconnect = {'reason': str(error), 'stage': self.stage,
                                            'time_ns': time.time_ns()}
                    self.status = {'state': 'disconnected', 'reason': str(error)}
                    print(json.dumps({'event': 'astra_tcp_disconnected',
                                      **self.last_disconnect}), flush=True)
                finally:
                    with self.lock:
                        self.latest = None

    def session(self, connection, peer):
        self.status = {'state': 'connected', 'peer': peer}
        self.stage = 'connected'
        self.last_camera_info = None
        print(json.dumps({'event': 'astra_tcp_connected', 'peer': peer}), flush=True)
        clock = deque(maxlen=256)
        pending = None
        last_ping = 0.0
        last_seq = -1
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_ping >= .5:
                # One outstanding ping; late replies cannot update the clock.
                if pending is None or now - pending[1] > 2.0:
                    pi_ns = time.time_ns()
                    pending = (pi_ns, now)
                    connection.sendall(b'PING' + struct.pack('!Q', pi_ns))
                    last_ping = now
            if not select.select([connection], [], [], .1)[0]:
                continue
            self.stage = 'message_magic'
            magic = exact(connection, 4)
            if magic == b'PONG':
                self.stage = 'pong'
                echo, win_ns = struct.unpack('!QQ', exact(connection, 16))
                received_ns, received_mono = time.time_ns(), time.monotonic()
                if pending is not None and echo == pending[0]:
                    rtt = received_ns - echo
                    elapsed = int((received_mono - pending[1]) * 1e9)
                    pending = None
                    self.last_clock_rtt_ns = rtt
                    if abs(rtt - elapsed) > 20_000_000:
                        clock.clear()
                    elif 0 <= rtt <= self.clock_max_rtt_ns:
                        offset = (echo + received_ns) // 2 - win_ns
                        if clock and abs(offset - min(clock, key=lambda x: x[0])[1]) > self.clock_max_rtt_ns:
                            clock.clear()
                        clock.append((rtt, offset, received_mono))
                    else:
                        self.clock_rejected += 1
                continue
            if magic != b'AST1':
                raise ValueError('unknown message magic or interleaved frame/PONG')
            self.stage = 'header_length'
            size = integer(struct.unpack('!I', exact(connection, 4))[0], 1, 65536)
            self.stage = 'header_json'
            header = json.loads(exact(connection, size).decode('utf-8'))
            if not isinstance(header, dict):
                raise ValueError('header must be an object')
            seq = integer(header['seq'], 0, 2**63 - 1)
            integer(header['send_ns'], 1, 2**63 - 1)
            rgb, depth = header['rgb'], header['depth']
            self.stage = 'rgb_metadata'
            rgb_size = metadata(rgb)
            self.stage = 'depth_metadata'
            depth_size = metadata(depth, True)
            self.stage = 'rgb_payload'
            rgb_data = exact(connection, rgb_size)
            self.stage = 'depth_payload'
            depth_data = exact(connection, depth_size)
            received_ns = time.time_ns()
            received_mono = time.monotonic()
            self.received_frames += 1
            self.receive_samples.append((received_mono, 8 + size + rgb_size + depth_size,
                                         abs(rgb['stamp_ns'] - depth['stamp_ns']) / 1e6,
                                         (header['send_ns'] - rgb['stamp_ns']) / 1e6,
                                         (header['send_ns'] - depth['stamp_ns']) / 1e6))
            if self.last_camera_info is None:
                self.last_camera_info = {'rgb': rgb['camera_info'], 'depth': depth['camera_info']}
                decode(rgb_data, rgb)
                decode(depth_data, depth, True)
                print(json.dumps({'event': 'astra_tcp_first_image_valid',
                                  'seq': seq, 'rgb_bytes': rgb_size, 'depth_bytes': depth_size,
                                  'rgb_compression': rgb['compression']}),
                      flush=True)
            if seq <= last_seq:
                raise ValueError('frame sequence did not increase')
            last_seq = seq
            fresh = [sample for sample in clock
                     if received_mono - sample[2] < self.clock_hold_s]
            if not fresh:
                self.dropped_frames += 1
                self.status = {'state': 'clock_sync', 'samples': len(fresh)}
                continue
            rtt, offset, _ = min(fresh, key=lambda x: x[0])
            stamps = [rgb['stamp_ns'] + offset, depth['stamp_ns'] + offset]
            age = [received_ns - stamp for stamp in stamps]
            if not all(0 <= a <= MAX_AGE_NS for a in age):
                self.dropped_frames += 1
                self.status = {'state': 'dropped', 'reason': 'stale_or_future_frame', 'age_ns': age}
                continue
            self.stage = 'image_decode'
            decode_start = time.monotonic()
            images = decode(rgb_data, rgb), decode(depth_data, depth, True)
            self.accepted_frames += 1
            self.accept_samples.append((received_mono, age[0] / 1e6, age[1] / 1e6,
                                        abs(stamps[0] - stamps[1]) / 1e6,
                                        (time.monotonic() - decode_start) * 1e3))
            with self.lock:
                self.latest = (header, images, stamps)
            self.last_received = time.monotonic()
            self.status = {'state': 'receiving', 'peer': peer, 'seq': seq,
                           'clock_offset_ns': offset, 'clock_rtt_ns': rtt,
                           'clock_uncertainty_ns': rtt // 2,
                           'clock_sample_age_s': received_mono - min(fresh, key=lambda x: x[0])[2],
                           'age_ns': age,
                           'rgb_depth_delta_ns': abs(stamps[0] - stamps[1])}

    def close(self):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=4.0)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5600)
    parser.add_argument('--peer', default='192.168.0.175')
    parser.add_argument('--publish-optical-tf', action='store_true',
                        help='publish existing calibrated lens-origin optical convention')
    parser.add_argument('--clock-hold-s', type=float, default=120.0)
    parser.add_argument('--clock-max-rtt-ms', type=float, default=100.0)
    args = parser.parse_args()
    if (not math.isfinite(args.clock_hold_s) or not 1.0 <= args.clock_hold_s <= 120.0
            or not math.isfinite(args.clock_max_rtt_ms)
            or not 0 < args.clock_max_rtt_ms <= 200.0):
        parser.error('clock hold must be 1..120s and maximum RTT must be 0..200ms')
    rclpy.init()
    node = Node('astra_tcp_receiver')
    if args.publish_optical_tf:
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        # camera_mount_current.yaml defines camera_link at the lens origin.
        # This is the same (z, -x, -y) basis used by table_edge_target; the
        # existing robot description continues to own base_link -> camera_link.
        optical_tf = TransformStamped()
        optical_tf.header.stamp = node.get_clock().now().to_msg()
        optical_tf.header.frame_id = 'camera_link'
        optical_tf.child_frame_id = FRAME
        optical_tf.transform.rotation.x = -0.5
        optical_tf.transform.rotation.y = 0.5
        optical_tf.transform.rotation.z = -0.5
        optical_tf.transform.rotation.w = 0.5
        optical_broadcaster = StaticTransformBroadcaster(node)
        optical_broadcaster.sendTransform(optical_tf)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    pubs = [(node.create_publisher(CameraInfo, f'/camera/{kind}/camera_info', qos),
             node.create_publisher(Image, f'/camera/{kind}/image_raw', qos))
            for kind in ('color', 'depth')]
    status_pub = node.create_publisher(String, '/camera/tcp_status', 1)
    receiver = Receiver(args.host, args.port, args.peer, clock_hold_s=args.clock_hold_s,
                        clock_max_rtt_ms=args.clock_max_rtt_ms)

    def publish():
        value = receiver.take()
        if value is None:
            return
        header, images, stamps = value
        if not all(0 <= time.time_ns() - stamp <= MAX_AGE_NS for stamp in stamps):
            receiver.publish_expired_frames += 1
            return
        # Parking consumes depth. Publish its much smaller sample first so a
        # large raw RGB write cannot delay the paired depth past MAX_AGE_NS.
        for index, kind in ((1, 'depth'), (0, 'rgb')):
            meta, image, stamp = header[kind], images[index], stamps[index]
            info = CameraInfo()
            info.header.frame_id = FRAME
            info.header.stamp.sec, info.header.stamp.nanosec = divmod(stamp, 10**9)
            info.width, info.height = meta['width'], meta['height']
            ci = meta['camera_info']
            info.distortion_model = ci['distortion_model']
            for key in ('D', 'K', 'R', 'P'):
                setattr(info, key.lower(), [float(v) for v in ci[key]])
            message = Image()
            message.header = info.header
            message.width, message.height = info.width, info.height
            message.encoding = meta['encoding']
            message.is_bigendian = 0
            message.step = info.width * (3 if index == 0 else 2)
            message.data = image.astype('<u2' if index else 'u1', copy=False).tobytes()
            pubs[index][0].publish(info)
            pubs[index][1].publish(message)
        receiver.published_frames += 1

    node.create_timer(.01, publish)
    node.create_timer(1.0, lambda: status_pub.publish(String(data=json.dumps(receiver.current_status()))))
    node.get_logger().info(f'AST1 listening on {args.host}:{args.port}; peer={args.peer}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        receiver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
