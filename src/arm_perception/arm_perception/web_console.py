#!/usr/bin/env python3
"""
Read-only ROS 2 gateway for the Robot Arm browser operator console.

The gateway deliberately exposes no actuator command publisher, service client
or action client. It mirrors allow-listed telemetry, serves timestamp-aligned
camera overlays, and manages recordings containing read-only topics only.
"""

from __future__ import annotations

import base64
from collections import deque
from datetime import datetime, timezone
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import signal
import socket
from socketserver import ThreadingMixIn
import struct
import subprocess
import threading
import time
from typing import Callable, Optional
from urllib.parse import unquote, urlparse

from ament_index_python.packages import get_package_share_directory
from arm_interfaces.msg import ArmStatus, ObjectArray
from arm_perception.detection_2d import detection_to_dict
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

DEFAULT_RECORD_TOPICS = (
    '/camera/image_raw/compressed',
    '/camera/camera_info',
    '/joint_states',
    '/tf',
    '/tf_static',
    '/detected_objects',
    '/detections_2d',
    '/arm_status',
    '/robot_description',
)
REPLAY_TOPICS = tuple(
    topic
    for topic in DEFAULT_RECORD_TOPICS
    if topic not in ('/tf', '/tf_static')
)
MAX_REQUEST_BYTES = 64 * 1024
SAFE_ASSET_SEGMENT = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
CONTENT_TYPES = {
    '.css': 'text/css; charset=utf-8',
    '.dae': 'model/vnd.collada+xml',
    '.glb': 'model/gltf-binary',
    '.gltf': 'model/gltf+json',
    '.html': 'text/html; charset=utf-8',
    '.jpeg': 'image/jpeg',
    '.jpg': 'image/jpeg',
    '.js': 'text/javascript; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.png': 'image/png',
    '.stl': 'model/stl',
    '.svg': 'image/svg+xml',
    '.urdf': 'application/xml; charset=utf-8',
    '.webp': 'image/webp',
    '.xacro': 'application/xml; charset=utf-8',
    '.yaml': 'application/yaml; charset=utf-8',
    '.yml': 'application/yaml; charset=utf-8',
}


def _asset_key(value: str) -> Optional[str]:
    """Return a normalized allow-listed relative asset key."""
    parts = PurePosixPath(value).parts
    if not parts or any(
        part in ('.', '..') or SAFE_ASSET_SEGMENT.fullmatch(part) is None
        for part in parts
    ):
        return None
    return '/'.join(parts)


def _content_type(name: str) -> str:
    """Map a trusted suffix to a fixed response header value."""
    suffix = Path(name).suffix.lower()
    return CONTENT_TYPES.get(suffix, 'application/octet-stream')


def _index_static_assets(root: Path) -> dict[str, tuple[bytes, str]]:
    """Read trusted static assets once so requests never form filesystem paths."""
    assets: dict[str, tuple[bytes, str]] = {}
    if not root.is_dir():
        return assets
    for candidate in root.rglob('*'):
        if not candidate.is_file():
            continue
        relative = candidate.relative_to(root).as_posix()
        assets[relative] = (candidate.read_bytes(), _content_type(relative))
    return assets
WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


from arm_perception import console_measurements as cm


def _read_key_values(path: Path) -> dict[str, int]:
    """Read Linux key/value counters without adding a runtime dependency."""
    values = {}
    try:
        lines = path.read_text(encoding='utf-8').splitlines()
    except OSError:
        return values
    for line in lines:
        key, separator, raw_value = line.partition(':')
        if not separator:
            continue
        token = raw_value.strip().split()[0]
        try:
            values[key] = int(token)
        except (ValueError, IndexError):
            continue
    return values


def _memory_used_percent(meminfo_path: Path = Path('/proc/meminfo')):
    values = _read_key_values(meminfo_path)
    total = values.get('MemTotal', 0)
    available = values.get('MemAvailable', 0)
    if total <= 0 or available < 0:
        return None
    return max(0.0, min(100.0, (total - available) * 100.0 / total))


def _cpu_temperature_c(paths=None):
    candidates = paths or (
        Path('/sys/class/thermal/thermal_zone0/temp'),
        Path('/sys/class/hwmon/hwmon0/temp1_input'),
    )
    for path in candidates:
        try:
            raw = float(path.read_text(encoding='utf-8').strip())
        except (OSError, ValueError):
            continue
        value = raw / 1000.0 if raw > 250.0 else raw
        if -20.0 <= value <= 150.0:
            return value
    return None


def _throttle_flags(run=subprocess.run):
    """Return Raspberry Pi throttle flags; unavailable hosts report nulls."""
    try:
        result = run(
            ['vcgencmd', 'get_throttled'],
            capture_output=True,
            check=False,
            text=True,
            timeout=0.75,
        )
    except (OSError, subprocess.SubprocessError):
        return {'raw': None, 'active': None, 'historical': None}
    match = re.search(r'0x([0-9a-fA-F]+)', result.stdout)
    if result.returncode != 0 or match is None:
        return {'raw': None, 'active': None, 'historical': None}
    flags = int(match.group(1), 16)
    return {
        'raw': f'0x{flags:x}',
        'active': bool(flags & 0xF),
        'historical': bool(flags & 0xF0000),
    }


def collect_system_telemetry(recordings_root: Path) -> dict:
    """Collect bounded, read-only host telemetry for the operator console."""
    try:
        load1, load5, load15 = os.getloadavg()
    except OSError:
        load1 = load5 = load15 = 0.0
    try:
        uptime = float(
            Path('/proc/uptime').read_text(encoding='utf-8').split()[0]
        )
    except (OSError, ValueError, IndexError):
        uptime = None
    try:
        disk_free = shutil.disk_usage(recordings_root).free
    except OSError:
        disk_free = None
    return {
        'hostname': socket.gethostname(),
        'platform': platform.machine(),
        'cpuCount': os.cpu_count() or 1,
        'cpuTemperatureC': _cpu_temperature_c(),
        'loadAverage': [float(load1), float(load5), float(load15)],
        'memoryUsedPercent': _memory_used_percent(),
        'diskFreeBytes': disk_free,
        'uptimeSeconds': uptime,
        'throttle': _throttle_flags(),
        'repoCommit': os.environ.get('ROBOT_ARM_REPO_COMMIT', 'not-reported'),
    }


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _age_seconds(
    updated_at: Optional[float], now: Optional[float] = None
) -> Optional[float]:
    if updated_at is None:
        return None
    return max(0.0, (time.monotonic() if now is None else now) - updated_at)


def _status(age: Optional[float], stale_after: float) -> str:
    if age is None:
        return 'offline'
    return 'healthy' if age <= stale_after else 'stale'


def websocket_text_frame(text: str) -> bytes:
    """Encode one unmasked server-to-browser RFC 6455 text frame."""
    payload = text.encode('utf-8')
    length = len(payload)
    if length < 126:
        prefix = struct.pack('!BB', 0x81, length)
    elif length <= 0xFFFF:
        prefix = struct.pack('!BBH', 0x81, 126, length)
    else:
        prefix = struct.pack('!BBQ', 0x81, 127, length)
    return prefix + payload


def _probe_warning(node, message: str) -> None:
    """Sonda uyarisi -- logger'i olmayan kosum takimlarinda da guvenli."""
    logger = getattr(node, 'get_logger', None)
    if logger is None:
        return
    try:
        logger().warning(message, throttle_duration_sec=30.0)
    except Exception:
        pass


class ConsoleState:
    """Thread-safe latest-value store shared by ROS and HTTP threads."""

    def __init__(self, overlay_max_age_s: float = 0.30):
        self._lock = threading.RLock()
        self._frame_ready = threading.Condition(self._lock)
        self._sequence = 0
        self._mode = 'live'
        self._overlay_max_age_ns = int(overlay_max_age_s * 1_000_000_000)
        self._updated = {
            'camera': None,
            'cameraInfo': None,
            'joints': None,
            'detections2d': None,
            'objects3d': None,
            'armStatus': None,
            'robotDescription': None,
            'system': None,
        }
        self._camera_stamp_ns = 0
        self._camera_source = None
        self._camera_arrivals = deque(maxlen=90)
        self._jpeg = None
        self._annotated_cache = None
        self._annotated_cache_key = None
        self._detections_by_stamp = deque(maxlen=40)
        self._joint_state = {
            'stampNs': 0,
            'names': [],
            'positions': [],
            'sourceKind': 'commanded_open_loop',
        }
        self._objects = []
        self._detections = []
        self._arm_status = None
        self._camera_info = None
        self._robot_description = ''
        self._system = None
        self._events = deque(maxlen=250)
        # Olcum saglamligi sondalari (hepsi salt okunur, bkz console_measurements)
        self._calibration = None
        self._flip_actual = None
        self._serial_ports = []
        self._measurements = {'available': False, 'entries': []}
        self._thermal = cm.ThermalHistory()
        self.add_event('info', 'Gateway hazır', 'Salt okunur telemetri etkin')

    def _changed(self) -> None:
        self._sequence += 1

    def add_event(self, level: str, title: str, detail: str) -> None:
        with self._lock:
            self._events.appendleft(
                {
                    'id': f'{time.time_ns()}',
                    'stamp': datetime.now(timezone.utc).isoformat(),
                    'level': level,
                    'title': title,
                    'detail': detail,
                }
            )
            self._changed()

    def set_mode(self, mode: str) -> None:
        if mode not in ('live', 'replay'):
            raise ValueError('mode must be live or replay')
        with self._lock:
            if self._mode != mode:
                self._mode = mode
                self.add_event(
                    'info',
                    'Veri kaynağı değişti',
                    'Kayıt tekrarı' if mode == 'replay' else 'Canlı ROS',
                )

    def update_camera(
        self, message: CompressedImage, source: str = 'compressed'
    ) -> None:
        if message.format and 'jpeg' not in message.format.lower():
            return
        with self._frame_ready:
            self._jpeg = bytes(message.data)
            self._camera_stamp_ns = _stamp_ns(message.header)
            self._camera_source = source
            arrival = time.monotonic()
            self._camera_arrivals.append(arrival)
            self._updated['camera'] = arrival
            self._annotated_cache = None
            self._changed()
            self._frame_ready.notify_all()

    def update_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = {
                'stampNs': _stamp_ns(message.header),
                'frameId': message.header.frame_id,
                'width': int(message.width),
                'height': int(message.height),
                'fx': float(message.k[0]),
                'fy': float(message.k[4]),
                'cx': float(message.k[2]),
                'cy': float(message.k[5]),
                'distortion': [float(v) for v in message.d],
            }
            self._updated['cameraInfo'] = time.monotonic()
            self._changed()

    def update_calibration(self, identity: dict) -> None:
        """Kimlik tespiti sonucu; kalibrasyon degistiginde olay uret."""
        with self._lock:
            previous = self._calibration
            self._calibration = identity
            changed = (
                previous is not None
                and previous.get('sha256') != identity.get('sha256'))
            self._changed()
        if previous is None and not identity.get('matched'):
            self.add_event(
                'warning', 'Kalibrasyon tanınmadı',
                identity.get('reason', ''))
        elif changed:
            self.add_event(
                'warning', 'Yayındaki kalibrasyon değişti',
                f"{previous.get('file')} -> {identity.get('file')}")

    def update_flip_actual(self, value) -> None:
        with self._lock:
            self._flip_actual = value

    def update_serial_ports(self, ports: list) -> None:
        with self._lock:
            self._serial_ports = ports

    def update_measurements(self, registry: dict) -> None:
        with self._lock:
            self._measurements = registry

    def update_joints(self, message: JointState) -> None:
        with self._lock:
            self._joint_state = {
                'stampNs': _stamp_ns(message.header),
                'names': list(message.name),
                'positions': [float(value) for value in message.position],
                'sourceKind': 'commanded_open_loop',
            }
            self._updated['joints'] = time.monotonic()
            self._changed()

    def update_detections(self, message: Detection2DArray) -> None:
        stamp = _stamp_ns(message.header)
        detections = [detection_to_dict(item) for item in message.detections]
        with self._frame_ready:
            self._detections = detections
            self._detections_by_stamp.append((stamp, detections))
            self._updated['detections2d'] = time.monotonic()
            if stamp == self._camera_stamp_ns:
                self._annotated_cache = None
            self._changed()
            self._frame_ready.notify_all()

    def update_objects(self, message: ObjectArray) -> None:
        objects = []
        for item in message.objects:
            pose = item.pose.pose
            objects.append(
                {
                    'id': item.object_id,
                    'className': item.object_type,
                    'confidence': float(item.confidence),
                    'frameId': item.pose.header.frame_id
                    or message.header.frame_id,
                    'stampNs': _stamp_ns(item.pose.header),
                    'position': {
                        'x': float(pose.position.x),
                        'y': float(pose.position.y),
                        'z': float(pose.position.z),
                    },
                    'orientation': {
                        'x': float(pose.orientation.x),
                        'y': float(pose.orientation.y),
                        'z': float(pose.orientation.z),
                        'w': float(pose.orientation.w),
                    },
                    'dimensions': [float(value) for value in item.dimensions],
                }
            )
        with self._lock:
            self._objects = objects
            self._updated['objects3d'] = time.monotonic()
            self._changed()

    def update_arm_status(self, message: ArmStatus) -> None:
        with self._lock:
            self._arm_status = {
                'stampNs': _stamp_ns(message.header),
                'state': int(message.state),
                'phase': message.current_phase,
                'progress': float(message.progress),
                'objectGrasped': bool(message.is_object_grasped),
                'error': message.error_message,
            }
            self._updated['armStatus'] = time.monotonic()
            self._changed()

    def update_robot_description(self, message: String) -> None:
        with self._lock:
            if message.data == self._robot_description:
                return
            self._robot_description = message.data
            self._updated['robotDescription'] = time.monotonic()
            self.add_event(
                'info', 'Robot modeli alındı', 'URDF tarayıcıya hazır'
            )

    def update_system(self, telemetry: dict) -> None:
        self._thermal.add(telemetry.get('cpuTemperatureC'))
        with self._lock:
            self._system = dict(telemetry)
            self._updated['system'] = time.monotonic()
            self._changed()

    def _matching_detections(self, stamp_ns: int) -> tuple[int, list]:
        best_stamp = 0
        best = []
        for candidate_stamp, detections in self._detections_by_stamp:
            delta = stamp_ns - candidate_stamp
            if (
                0 <= delta <= self._overlay_max_age_ns
                and candidate_stamp >= best_stamp
            ):
                best_stamp = candidate_stamp
                best = detections
        return best_stamp, best

    @staticmethod
    def _annotate(jpeg: bytes, detections: list) -> bytes:
        encoded = np.frombuffer(jpeg, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            return jpeg
        height, width = image.shape[:2]
        for item in detections:
            bbox = item['bbox']
            x1 = max(0, int(round(bbox['centerX'] - bbox['width'] / 2.0)))
            y1 = max(0, int(round(bbox['centerY'] - bbox['height'] / 2.0)))
            x2 = min(
                width - 1, int(round(bbox['centerX'] + bbox['width'] / 2.0))
            )
            y2 = min(
                height - 1, int(round(bbox['centerY'] + bbox['height'] / 2.0))
            )
            color = (32, 190, 255)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            label = '{} {:.0f}%'.format(
                item['className'], item['confidence'] * 100.0
            )
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
            )
            label_top = max(0, y1 - text_height - 9)
            cv2.rectangle(
                image,
                (x1, label_top),
                (min(width - 1, x1 + text_width + 10), y1),
                color,
                cv2.FILLED,
            )
            cv2.putText(
                image,
                label,
                (x1 + 5, max(text_height + 1, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (12, 20, 24),
                1,
                cv2.LINE_AA,
            )
        ok, buffer = cv2.imencode(
            '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 82]
        )
        return buffer.tobytes() if ok else jpeg

    def frame_after(
        self, previous_stamp: int, timeout: float = 1.0
    ) -> tuple[int, Optional[bytes]]:
        deadline = time.monotonic() + timeout
        with self._frame_ready:
            while (
                self._jpeg is None or self._camera_stamp_ns == previous_stamp
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return previous_stamp, None
                self._frame_ready.wait(remaining)
            stamp = self._camera_stamp_ns
            jpeg = self._jpeg
            detection_stamp, detections = self._matching_detections(stamp)
            cache_key = (stamp, detection_stamp)
            if (
                self._annotated_cache_key == cache_key
                and self._annotated_cache is not None
            ):
                return stamp, self._annotated_cache
        annotated = self._annotate(jpeg, detections)
        with self._lock:
            if self._camera_stamp_ns == stamp:
                self._annotated_cache_key = cache_key
                self._annotated_cache = annotated
        return stamp, annotated

    def health(self) -> dict:
        with self._lock:
            now = time.monotonic()
            ages = {
                key: _age_seconds(value, now)
                for key, value in self._updated.items()
            }
            fps = 0.0
            if len(self._camera_arrivals) >= 2:
                duration = self._camera_arrivals[-1] - self._camera_arrivals[0]
                if duration > 0.0:
                    fps = (len(self._camera_arrivals) - 1) / duration
            camera_status = _status(ages['camera'], 1.0)
            if camera_status != 'healthy':
                fps = 0.0
            return {
                'mode': self._mode,
                'camera': {
                    'status': camera_status,
                    'ageSeconds': ages['camera'],
                    'fps': fps,
                    'source': self._camera_source,
                },
                'cameraInfo': {
                    'status': _status(ages['cameraInfo'], 2.0),
                    'ageSeconds': ages['cameraInfo'],
                },
                'joints': {
                    'status': _status(ages['joints'], 1.0),
                    'ageSeconds': ages['joints'],
                },
                'perception': {
                    'status': _status(ages['detections2d'], 2.0),
                    'ageSeconds': ages['detections2d'],
                },
                'objects3d': {
                    'status': _status(ages['objects3d'], 2.0),
                    'ageSeconds': ages['objects3d'],
                },
                'armStatus': {
                    'status': _status(ages['armStatus'], 2.0),
                    'ageSeconds': ages['armStatus'],
                },
                'robotDescription': {
                    'status': 'healthy'
                    if self._robot_description
                    else 'offline',
                    'ageSeconds': ages['robotDescription'],
                },
                'system': {
                    'status': _status(ages['system'], 12.0),
                    'ageSeconds': ages['system'],
                },
                'readOnly': True,
            }

    def bootstrap(self) -> dict:
        with self._lock:
            packages = sorted(
                set(
                    re.findall(
                        r'package://([A-Za-z0-9_]+)', self._robot_description
                    )
                )
            )
            return {
                'apiVersion': 'v1',
                'readOnly': True,
                'jointStateSemantics': 'commanded_open_loop',
                'jointStateNotice': 'Komut edilen poz — encoder geri bildirimi yok',
                'robotDescription': self._robot_description,
                'packageMap': {
                    package: f'/api/v1/assets/{package}'
                    for package in packages
                },
                'capabilities': {
                    'cameraOverlay': True,
                    'rawImageFallback': True,
                    'recording': True,
                    'replay': True,
                    'motionCommands': False,
                },
            }

    def live_snapshot(self) -> dict:
        with self._lock:
            return {
                'type': 'snapshot',
                'sequence': self._sequence,
                'mode': self._mode,
                'jointState': self._joint_state,
                'objects': list(self._objects),
                'detections2d': list(self._detections),
                'armStatus': self._arm_status,
                'cameraInfo': self._camera_info,
                'system': self._system,
                'health': self.health(),
                'events': list(self._events),
                'calibration': self._calibration,
                'flipContract': cm.flip_contract(
                    (self._calibration or {}).get('flipMethod'),
                    self._flip_actual),
                'serialPorts': list(self._serial_ports),
                'thermal': self._thermal.snapshot(),
                'measurements': self._measurements,
            }


class RecordingManager:
    """Start/stop allow-listed rosbag recordings and read-only replay."""

    def __init__(
        self,
        root: Path,
        state: ConsoleState,
        min_free_bytes: int = 5_000_000_000,
        popen: Callable = subprocess.Popen,
    ):
        self.root = root.expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.state = state
        self.min_free_bytes = int(min_free_bytes)
        self._popen = popen
        self._lock = threading.RLock()
        self._record_process = None
        self._record_session = None
        self._replay_process = None
        self._replay_session = None

    @staticmethod
    def _slug(value: str) -> str:
        slug = re.sub(r'[^a-zA-Z0-9_-]+', '-', value.strip()).strip('-')
        return slug[:48] or 'oturum'

    @staticmethod
    def _write_json(path: Path, document: dict) -> None:
        temporary = path.with_suffix('.tmp')
        temporary.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        temporary.replace(path)

    def _reap(self) -> None:
        if (
            self._record_process is not None
            and self._record_process.poll() is not None
        ):
            self._finish_recording(
                'completed'
                if self._record_process.returncode == 0
                else 'failed'
            )
        if (
            self._replay_process is not None
            and self._replay_process.poll() is not None
        ):
            replay_id = self._replay_session
            self._replay_process = None
            self._replay_session = None
            self.state.set_mode('live')
            self.state.add_event('info', 'Tekrar tamamlandı', str(replay_id))

    def start_recording(self, name: str, note: str) -> dict:
        with self._lock:
            self._reap()
            if self._record_process is not None:
                raise RuntimeError('Bir kayıt zaten etkin')
            if self._replay_process is not None:
                raise RuntimeError('Tekrar oynatılırken kayıt başlatılamaz')
            free_bytes = shutil.disk_usage(self.root).free
            if free_bytes < self.min_free_bytes:
                raise RuntimeError('Kayıt için yeterli boş disk alanı yok')
            session_id = '{}_{}'.format(
                datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'),
                self._slug(name),
            )
            session_dir = self.root / session_id
            bag_dir = session_dir / 'bag'
            session_dir.mkdir(parents=False, exist_ok=False)
            manifest = {
                'id': session_id,
                'name': name.strip() or 'Oturum',
                'note': note.strip(),
                'status': 'recording',
                'startedAt': datetime.now(timezone.utc).isoformat(),
                'endedAt': None,
                'topics': list(DEFAULT_RECORD_TOPICS),
                'jointStateSemantics': 'commanded_open_loop',
                'cameraCalibration': 'imx219_640x480 flip_method=2',
                'model': os.environ.get('ROBOT_ARM_MODEL_ID', 'not-reported'),
                'calibration': os.environ.get(
                    'ROBOT_ARM_CALIBRATION_ID', 'not-reported'
                ),
                'commit': os.environ.get('ROBOT_ARM_REPO_COMMIT', 'not-reported'),
            }
            self._write_json(session_dir / 'session.json', manifest)
            command = [
                'ros2',
                'bag',
                'record',
                '-o',
                str(bag_dir),
                *DEFAULT_RECORD_TOPICS,
            ]
            try:
                self._record_process = self._popen(
                    command,
                    start_new_session=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError as exc:
                manifest['status'] = 'failed'
                manifest['endedAt'] = datetime.now(timezone.utc).isoformat()
                self._write_json(session_dir / 'session.json', manifest)
                self.state.add_event('error', 'Kayıt başlatılamadı', str(exc))
                raise RuntimeError(
                    'ros2 bag record başlatılamadı: {}'.format(exc)
                ) from exc
            self._record_session = session_id
            self.state.add_event(
                'recording', 'Kayıt başladı', manifest['name']
            )
            return manifest

    def _finish_recording(self, status: str) -> Optional[dict]:
        if self._record_session is None:
            return None
        session_dir = self.root / self._record_session
        manifest_path = session_dir / 'session.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['status'] = status
        manifest['endedAt'] = datetime.now(timezone.utc).isoformat()
        self._write_json(manifest_path, manifest)
        self._record_process = None
        self._record_session = None
        self.state.add_event('info', 'Kayıt durdu', manifest['name'])
        return manifest

    def stop_recording(self) -> dict:
        with self._lock:
            self._reap()
            if self._record_process is None:
                raise RuntimeError('Etkin kayıt yok')
            process = self._record_process
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            return self._finish_recording(
                'completed' if process.returncode == 0 else 'failed'
            )

    def list_sessions(self) -> list[dict]:
        with self._lock:
            self._reap()
            sessions = []
            for manifest_path in sorted(
                self.root.glob('*/session.json'), reverse=True
            ):
                try:
                    document = json.loads(
                        manifest_path.read_text(encoding='utf-8')
                    )
                    bag_dir = manifest_path.parent / 'bag'
                    document['replayAvailable'] = (
                        bag_dir / 'metadata.yaml'
                    ).is_file()
                    sessions.append(document)
                except (OSError, ValueError):
                    continue
            return sessions

    def start_replay(self, session_id: str, rate: float = 1.0) -> dict:
        with self._lock:
            self._reap()
            if (
                self._record_process is not None
                or self._replay_process is not None
            ):
                raise RuntimeError('Başka bir kayıt veya tekrar etkin')
            if not 0.1 <= float(rate) <= 4.0:
                raise ValueError('Replay rate 0.1 ile 4.0 arasında olmalı')
            session_dir = (self.root / session_id).resolve()
            if session_dir.parent != self.root:
                raise ValueError('Geçersiz oturum kimliği')
            bag_dir = session_dir / 'bag'
            if not (bag_dir / 'metadata.yaml').is_file():
                raise FileNotFoundError('Oturum rosbag kaydı içermiyor')
            remaps = []
            for topic in REPLAY_TOPICS:
                target = '/web_replay' + topic
                remaps.extend(['-r', f'{topic}:={target}'])
            command = [
                'ros2',
                'bag',
                'play',
                str(bag_dir),
                '--rate',
                str(float(rate)),
                '--topics',
                *REPLAY_TOPICS,
                '--ros-args',
                *remaps,
            ]
            self._replay_process = self._popen(
                command,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._replay_session = session_id
            self.state.set_mode('replay')
            self.state.add_event('info', 'Tekrar başladı', session_id)
            return {
                'id': session_id,
                'rate': float(rate),
                'status': 'replaying',
            }

    def stop_replay(self) -> dict:
        with self._lock:
            self._reap()
            if self._replay_process is None:
                raise RuntimeError('Etkin tekrar yok')
            process = self._replay_process
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            session_id = self._replay_session
            self._replay_process = None
            self._replay_session = None
            self.state.set_mode('live')
            return {'id': session_id, 'status': 'stopped'}

    def status(self) -> dict:
        with self._lock:
            self._reap()
            return {
                'recording': self._record_session,
                'replay': self._replay_session,
                'freeBytes': shutil.disk_usage(self.root).free,
                'minimumFreeBytes': self.min_free_bytes,
            }

    def shutdown(self) -> None:
        with self._lock:
            if self._record_process is not None:
                self.stop_recording()
            if self._replay_process is not None:
                self.stop_replay()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def make_http_handler(
    state: ConsoleState,
    recordings: RecordingManager,
    web_root: Path,
    asset_resolver: Callable[[str, str], Optional[tuple[bytes, str]]],
):
    """Build the HTTP handler with explicit state dependencies for tests."""
    web_assets = _index_static_assets(web_root)
    index_asset = web_assets.get('index.html')

    class Handler(BaseHTTPRequestHandler):
        server_version = 'RobotArmConsole/1.0'

        def log_message(self, *args):
            pass

        def _json(self, status_code: int, document: dict | list) -> None:
            payload = json.dumps(document, ensure_ascii=False).encode('utf-8')
            self.send_response(status_code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(payload)

        def _body(self) -> dict:
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError as exc:
                raise ValueError('Geçersiz Content-Length') from exc
            if length < 0 or length > MAX_REQUEST_BYTES:
                raise ValueError('İstek gövdesi çok büyük')
            raw = self.rfile.read(length)
            return json.loads(raw.decode('utf-8')) if raw else {}

        def _serve_asset(
            self,
            asset: Optional[tuple[bytes, str]],
            cache: bool = False,
        ) -> None:
            if asset is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            payload, mime_type = asset
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', mime_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header(
                'Cache-Control',
                'public, max-age=86400' if cache else 'no-cache',
            )
            self.end_headers()
            self.wfile.write(payload)

        def _websocket(self) -> None:
            key = self.headers.get('Sec-WebSocket-Key')
            origin = self.headers.get('Origin')
            origin_host = urlparse(origin).netloc if origin else None
            request_host = self.headers.get('Host')
            if (
                not key
                or self.headers.get('Upgrade', '').lower() != 'websocket'
                or (origin_host is not None and origin_host != request_host)
            ):
                self.send_error(HTTPStatus.BAD_REQUEST)
                return
            accept = base64.b64encode(
                hashlib.sha1((key + WEBSOCKET_GUID).encode('ascii')).digest()
            ).decode('ascii')
            self.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
            self.send_header('Upgrade', 'websocket')
            self.send_header('Connection', 'Upgrade')
            self.send_header('Sec-WebSocket-Accept', accept)
            self.end_headers()
            previous = -1
            last_send = 0.0
            try:
                while True:
                    recording_status = recordings.status()
                    snapshot = state.live_snapshot()
                    now = time.monotonic()
                    if (
                        snapshot['sequence'] != previous
                        or now - last_send >= 10.0
                    ):
                        snapshot['recording'] = recording_status
                        payload = json.dumps(
                            snapshot, ensure_ascii=False, separators=(',', ':')
                        )
                        self.connection.sendall(websocket_text_frame(payload))
                        previous = snapshot['sequence']
                        last_send = now
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def _camera_stream(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
            )
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            previous_stamp = -1
            try:
                while True:
                    stamp, jpeg = state.frame_after(
                        previous_stamp, timeout=1.0
                    )
                    if jpeg is None:
                        continue
                    previous_stamp = stamp
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                    )
                    self.wfile.write(
                        f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii')
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def do_GET(self):
            parsed = urlparse(self.path)
            path = unquote(parsed.path)
            if path == '/api/v1/bootstrap':
                self._json(HTTPStatus.OK, state.bootstrap())
                return
            if path == '/api/v1/health':
                recording_status = recordings.status()
                document = state.health()
                document['recording'] = recording_status
                self._json(HTTPStatus.OK, document)
                return
            if path == '/api/v1/recordings':
                self._json(HTTPStatus.OK, recordings.list_sessions())
                return
            if path == '/api/v1/live':
                self._websocket()
                return
            if path == '/api/v1/camera/stream':
                self._camera_stream()
                return
            if path.startswith('/api/v1/assets/'):
                parts = path.split('/', 5)
                if len(parts) != 6:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                resolved = asset_resolver(parts[4], parts[5])
                if resolved is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                else:
                    self._serve_asset(resolved, cache=True)
                return

            relative = (
                'index.html' if path == '/' else _asset_key(path.lstrip('/'))
            )
            if relative is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            asset = web_assets.get(relative)
            if asset is not None:
                self._serve_asset(asset, cache='/assets/' in path)
            else:
                self._serve_asset(index_asset)

        def do_POST(self):
            try:
                body = self._body()
                if self.path == '/api/v1/recordings/start':
                    result = recordings.start_recording(
                        str(body.get('name', 'Oturum')),
                        str(body.get('note', '')),
                    )
                elif self.path == '/api/v1/recordings/stop':
                    result = recordings.stop_recording()
                elif self.path == '/api/v1/replay/start':
                    result = recordings.start_replay(
                        str(body.get('id', '')), float(body.get('rate', 1.0))
                    )
                elif self.path == '/api/v1/replay/stop':
                    result = recordings.stop_replay()
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self._json(HTTPStatus.OK, result)
            except (
                ValueError,
                RuntimeError,
                FileNotFoundError,
                json.JSONDecodeError,
            ) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {'error': str(exc)})

    return Handler


class WebConsoleNode(Node):
    """ROS subscriptions and lifecycle for the read-only web gateway."""

    def __init__(self):
        super().__init__('web_console')
        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('port', 8088)
        self.declare_parameter(
            'recordings_root', '/var/tmp/robot-arm-console/recordings'
        )
        self.declare_parameter('minimum_free_gb', 5.0)
        self.declare_parameter('web_root', '')
        self.declare_parameter(
            'asset_packages', ['robot_arm_description', 'arm_description']
        )
        self.declare_parameter(
            'compressed_image_topic', '/camera/image_raw/compressed'
        )
        self.declare_parameter('raw_image_topic', '/camera/image_raw')
        self.declare_parameter('raw_image_max_fps', 15.0)
        self.declare_parameter('raw_image_jpeg_quality', 82)
        self.declare_parameter('compressed_preference_seconds', 1.0)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('objects_topic', '/detected_objects')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('arm_status_topic', '/arm_status')
        self.declare_parameter('system_telemetry_interval_s', 5.0)
        # --- olcum saglamligi sondalari (salt okunur) ---
        # Yayindaki intrinsics'in HANGI dosyadan geldigini dosya adina
        # guvenmeden, K/D karsilastirarak bulmak icin taranan dizin.
        self.declare_parameter('camera_config_dir', '')
        # flip sozlesmesi denetimi: kalibrasyonun cekildigi flip ile node'un
        # uyguladigi flip ayni mi. Node adi bos birakilirsa denetim yapilmaz.
        self.declare_parameter('camera_node_name', '/csi_camera_node')
        self.declare_parameter('measurement_registry', '')
        # Seri portu KIM tutuyor -- port ACILMAZ, /proc okunur.
        self.declare_parameter(
            'serial_port_globs', ['/dev/ttyUSB*', '/dev/ttyACM*'])
        self.declare_parameter('thermal_window_s', 600.0)

        self.state = ConsoleState()
        thermal_window = float(self.get_parameter('thermal_window_s').value)
        if not np.isfinite(thermal_window) or thermal_window <= 0.0:
            raise ValueError('thermal_window_s must be finite and positive')
        self.state._thermal = cm.ThermalHistory(window_s=thermal_window)
        registry_path = str(
            self.get_parameter('measurement_registry').value).strip()
        if registry_path:
            self.state.update_measurements(
                cm.load_measurement_registry(Path(registry_path)))
        config_dir = str(self.get_parameter('camera_config_dir').value).strip()
        self._camera_config_dir = Path(config_dir) if config_dir else None
        self._serial_port_globs = [
            str(pattern)
            for pattern in self.get_parameter('serial_port_globs').value
        ]
        self._camera_node_name = str(
            self.get_parameter('camera_node_name').value).strip()
        self._flip_client = None
        self._last_calibration_key = None
        recordings_root = Path(
            str(self.get_parameter('recordings_root').value)
        )
        self.recordings = RecordingManager(
            recordings_root,
            self.state,
            int(
                float(self.get_parameter('minimum_free_gb').value)
                * 1_000_000_000
            ),
        )
        self.allowed_packages = set(self.get_parameter('asset_packages').value)
        self._package_assets = {}
        for package in self.allowed_packages:
            try:
                package_root = Path(get_package_share_directory(package))
            except LookupError:
                continue
            self._package_assets[package] = _index_static_assets(package_root)
        self._bridge = CvBridge()
        self._last_compressed_at = None
        self._last_raw_encoded_at = None
        raw_fps = float(self.get_parameter('raw_image_max_fps').value)
        if not np.isfinite(raw_fps) or raw_fps <= 0.0:
            raise ValueError('raw_image_max_fps must be finite and positive')
        self._raw_image_period = 1.0 / raw_fps
        quality = int(self.get_parameter('raw_image_jpeg_quality').value)
        if quality < 1 or quality > 100:
            raise ValueError('raw_image_jpeg_quality must be within [1, 100]')
        self._raw_image_jpeg_quality = quality
        preference = float(
            self.get_parameter('compressed_preference_seconds').value
        )
        if not np.isfinite(preference) or preference < 0.0:
            raise ValueError(
                'compressed_preference_seconds must be finite and non-negative'
            )
        self._compressed_preference_seconds = preference
        telemetry_interval = float(
            self.get_parameter('system_telemetry_interval_s').value
        )
        if not np.isfinite(telemetry_interval) or telemetry_interval <= 0.0:
            raise ValueError(
                'system_telemetry_interval_s must be finite and positive'
            )
        self._recordings_root = recordings_root
        self._system_sample_lock = threading.Lock()
        self._sample_system_telemetry()
        self._system_telemetry_timer = self.create_timer(
            telemetry_interval, self._sample_system_telemetry
        )

        reliable = QoSProfile(depth=10)
        robot_description_qos = QoSProfile(depth=1)
        robot_description_qos.reliability = ReliabilityPolicy.RELIABLE
        robot_description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        robot_description_qos.history = HistoryPolicy.KEEP_LAST

        self._compressed_image_topic = str(
            self.get_parameter('compressed_image_topic').value
        )
        self._compressed_subscription = self.create_subscription(
            CompressedImage,
            self._compressed_image_topic,
            self._live_only(self._update_compressed_camera),
            qos_profile_sensor_data,
        )
        # Do not create the raw Image reader until graph discovery confirms
        # that no compressed publisher exists. A raw 640x480 stream measured
        # ~25 MB/s on the robot LAN; rate-limiting JPEG encode alone does not
        # rate-limit DDS transport.
        self._raw_image_topic = str(
            self.get_parameter('raw_image_topic').value)
        self._raw_image_qos = qos_profile_sensor_data
        self._raw_subscription = None
        self._camera_transport_timer = self.create_timer(
            1.0, self._manage_raw_camera_subscription)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._live_only(self.state.update_camera_info),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._live_only(self.state.update_joints),
            reliable,
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('detections_2d_topic').value),
            self._live_only(self.state.update_detections),
            reliable,
        )
        self.create_subscription(
            ObjectArray,
            str(self.get_parameter('objects_topic').value),
            self._live_only(self.state.update_objects),
            reliable,
        )
        self.create_subscription(
            ArmStatus,
            str(self.get_parameter('arm_status_topic').value),
            self._live_only(self.state.update_arm_status),
            reliable,
        )
        self.create_subscription(
            String,
            '/robot_description',
            self.state.update_robot_description,
            robot_description_qos,
        )

        prefix = '/web_replay'
        self.create_subscription(
            CompressedImage,
            prefix + '/camera/image_raw/compressed',
            self._replay_only(self.state.update_camera),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            prefix + '/camera/camera_info',
            self._replay_only(self.state.update_camera_info),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            prefix + '/joint_states',
            self._replay_only(self.state.update_joints),
            reliable,
        )
        self.create_subscription(
            Detection2DArray,
            prefix + '/detections_2d',
            self._replay_only(self.state.update_detections),
            reliable,
        )
        self.create_subscription(
            ObjectArray,
            prefix + '/detected_objects',
            self._replay_only(self.state.update_objects),
            reliable,
        )
        self.create_subscription(
            ArmStatus,
            prefix + '/arm_status',
            self._replay_only(self.state.update_arm_status),
            reliable,
        )
        self.create_subscription(
            String,
            prefix + '/robot_description',
            self._replay_only(self.state.update_robot_description),
            robot_description_qos,
        )

        configured_web_root = str(self.get_parameter('web_root').value)
        web_root = (
            Path(configured_web_root)
            if configured_web_root
            else Path(get_package_share_directory('arm_perception'))
            / 'web'
            / 'robot-arm-console'
            / 'dist'
        )
        handler = make_http_handler(
            self.state, self.recordings, web_root, self._resolve_asset
        )
        if not (web_root / 'index.html').is_file():
            raise RuntimeError(
                'Web console production build missing: {} (run npm run build)'.format(
                    web_root / 'index.html'
                )
            )
        self.http_server = ThreadingHTTPServer(
            (
                str(self.get_parameter('bind_address').value),
                int(self.get_parameter('port').value),
            ),
            handler,
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            'Robot Arm web console http://{}:{} serving {} '
            '(READ-ONLY, no motion endpoints)'.format(
                self.get_parameter('bind_address').value,
                self.get_parameter('port').value,
                web_root,
            )
        )

    def _update_compressed_camera(self, message: CompressedImage) -> None:
        """Prefer an upstream compressed feed when one is available."""
        self._last_compressed_at = time.monotonic()
        self._drop_raw_camera_subscription()
        self.state.update_camera(message, source='compressed')

    def _sample_system_telemetry(self) -> None:
        """Sample outside the ROS executor; vcgencmd may briefly block."""
        if not self._system_sample_lock.acquire(blocking=False):
            return

        def collect():
            try:
                self.state.update_system(
                    collect_system_telemetry(self._recordings_root)
                )
                # Sondalar ayni is parcaciginda: hepsi dosya sistemi okur ve
                # ROS executor'unu bloklamamalari gerekir. Telemetri yolunun
                # hata davranisi DEGISMEDI; yalnizca sondalar kapsanir, cunku
                # teshis araci arizanin kendisi olamaz.
                probes = getattr(self, '_sample_measurement_probes', None)
                if probes is not None:
                    try:
                        probes()
                    except Exception as error:
                        _probe_warning(
                            self, f'olcum sondasi hatasi: {error}')
            finally:
                self._system_sample_lock.release()

        threading.Thread(target=collect, daemon=True).start()

    def _sample_measurement_probes(self) -> None:
        """Sondalari calistir. Eksik alan varsa sessizce atla.

        `getattr` ile korunur cunku bu metod tam kurulmamis bir node uzerinde de
        (testlerdeki hafif kosum takimlari) cagrilabiliyor ve sonda yuzunden
        telemetri kaybetmek kabul edilemez.
        """
        globs = getattr(self, '_serial_port_globs', None)
        if globs:
            self.state.update_serial_ports(cm.serial_port_owners(globs))
        if getattr(self, '_camera_config_dir', None) is not None:
            self._identify_calibration()
        if getattr(self, '_camera_node_name', ''):
            self._sample_flip_method()

    def _identify_calibration(self) -> None:
        """Yayindaki intrinsics repodaki hangi kalibrasyon dosyasindan geliyor.

        Dosya adina degil SAYILARA bakar. Ayni K/D tekrar tekrar geldiginde
        yeniden dosya taramamak icin son anahtari hatirlar.
        """
        if self._camera_config_dir is None:
            return
        info = self.state.live_snapshot().get('cameraInfo')
        if not info or 'fx' not in info:
            return
        key = (info.get('fx'), info.get('fy'), info.get('cx'), info.get('cy'))
        if key == self._last_calibration_key:
            return
        identity = cm.identify_calibration(
            [info['fx'], 0.0, info['cx'], 0.0, info['fy'], info['cy'],
             0.0, 0.0, 1.0],
            info.get('distortion', []),
            self._camera_config_dir)
        self._last_calibration_key = key
        self.state.update_calibration(identity)

    def _sample_flip_method(self) -> None:
        """Kamera node'unun UYGULADIGI flip'i parametre servisinden oku.

        Salt okuma. Node yoksa ya da servis cevap vermezse sessizce gecilir ve
        sozlesme durumu 'unknown' kalir -- bilinmeyeni 'uyumlu' saymak bu
        denetimin butun anlamini yok ederdi.
        """
        if not self._camera_node_name:
            return
        try:
            from rcl_interfaces.srv import GetParameters
            if self._flip_client is None:
                self._flip_client = self.create_client(
                    GetParameters,
                    f'{self._camera_node_name}/get_parameters')
            if not self._flip_client.service_is_ready():
                self.state.update_flip_actual(None)
                return
            request = GetParameters.Request()
            request.names = ['flip_method']
            future = self._flip_client.call_async(request)
            deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                self.state.update_flip_actual(None)
                return
            values = future.result().values
            self.state.update_flip_actual(
                int(values[0].integer_value) if values else None)
        except Exception:
            self.state.update_flip_actual(None)

    def _manage_raw_camera_subscription(self) -> None:
        """Subscribe to raw only while live and no compressed publisher exists."""
        mode = self.state.health()['mode']
        # Subscription.get_publisher_count() is unavailable in ROS 2 Humble.
        # Node.count_publishers(topic) has the same graph meaning and is shared
        # by the Humble deployment container and the Jazzy development host.
        compressed_publishers = self.count_publishers(
            self._compressed_image_topic
        )
        if mode != 'live' or compressed_publishers > 0:
            self._drop_raw_camera_subscription()
            return
        if self._raw_subscription is None:
            self._raw_subscription = self.create_subscription(
                Image,
                self._raw_image_topic,
                self._live_only(self._update_raw_camera),
                self._raw_image_qos,
            )
            self.get_logger().info(
                'Compressed camera publisher absent; raw JPEG fallback enabled'
            )

    def _drop_raw_camera_subscription(self) -> None:
        """Stop raw DDS transport as soon as compressed transport is usable."""
        subscription = self._raw_subscription
        if subscription is None:
            return
        if self.destroy_subscription(subscription):
            self._raw_subscription = None
            self.get_logger().info(
                'Compressed camera available; raw DDS fallback disabled'
            )
        else:
            self.get_logger().warning(
                'Raw camera subscription could not be destroyed',
                throttle_duration_sec=5.0,
            )

    def _update_raw_camera(self, message: Image) -> None:
        """Encode a rate-limited raw frame when no compressed feed is live."""
        now = time.monotonic()
        if (
            self._last_compressed_at is not None
            and now - self._last_compressed_at
            <= self._compressed_preference_seconds
        ):
            return
        if (
            self._last_raw_encoded_at is not None
            and now - self._last_raw_encoded_at < self._raw_image_period
        ):
            return
        self._last_raw_encoded_at = now
        try:
            image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
            ok, encoded = cv2.imencode(
                '.jpg',
                image,
                [cv2.IMWRITE_JPEG_QUALITY, self._raw_image_jpeg_quality],
            )
        except (CvBridgeError, ValueError) as exc:
            self.get_logger().warning(
                'Raw camera conversion failed: {}'.format(exc),
                throttle_duration_sec=5.0,
            )
            return
        if not ok:
            self.get_logger().warning(
                'Raw camera JPEG encoding failed', throttle_duration_sec=5.0
            )
            return
        compressed = CompressedImage()
        compressed.header = message.header
        compressed.format = 'jpeg'
        compressed.data = encoded.tobytes()
        self.state.update_camera(compressed, source='raw_jpeg_fallback')

    def _replay_only(self, callback: Callable) -> Callable:
        def wrapped(message):
            if self.state.health()['mode'] == 'replay':
                callback(message)

        return wrapped

    def _live_only(self, callback: Callable) -> Callable:
        def wrapped(message):
            if self.state.health()['mode'] == 'live':
                callback(message)

        return wrapped

    def _resolve_asset(
        self, package: str, relative: str
    ) -> Optional[tuple[bytes, str]]:
        package_assets = self._package_assets.get(package)
        relative_key = _asset_key(relative)
        if package_assets is None or relative_key is None:
            return None
        return package_assets.get(relative_key)

    def destroy_node(self):
        self.http_server.shutdown()
        self.recordings.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebConsoleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
