"""Read-only web console state, overlay and recording boundary tests."""

import json
from pathlib import Path
import threading
from types import SimpleNamespace
import urllib.request

from arm_perception.web_console import (
    _memory_used_percent,
    _throttle_flags,
    ConsoleState,
    DEFAULT_RECORD_TOPICS,
    make_http_handler,
    RecordingManager,
    REPLAY_TOPICS,
    ThreadingHTTPServer,
    websocket_text_frame,
    WebConsoleNode,
)
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


def _header(stamp_ns: int) -> Header:
    header = Header()
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000
    header.frame_id = 'camera_optical_frame'
    return header


def _jpeg(stamp_ns: int) -> CompressedImage:
    image = np.zeros((100, 160, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    message = CompressedImage()
    message.header = _header(stamp_ns)
    message.format = 'jpeg'
    message.data = encoded.tobytes()
    return message


def _detections(stamp_ns: int) -> Detection2DArray:
    output = Detection2DArray()
    output.header = _header(stamp_ns)
    detected = Detection2D()
    detected.header = output.header
    detected.id = 'red_box_0'
    detected.bbox.center.position.x = 80.0
    detected.bbox.center.position.y = 50.0
    detected.bbox.size_x = 50.0
    detected.bbox.size_y = 30.0
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = 'red_box'
    result.hypothesis.score = 0.9
    detected.results.append(result)
    output.detections.append(detected)
    return output


def test_state_labels_joint_values_as_commanded_open_loop():
    state = ConsoleState()
    joints = JointState()
    joints.header = _header(5)
    joints.name = ['joint_1', 'joint_2']
    joints.position = [0.1, -0.2]

    state.update_joints(joints)

    snapshot = state.live_snapshot()
    assert snapshot['jointState']['sourceKind'] == 'commanded_open_loop'
    assert snapshot['jointState']['positions'] == [0.1, -0.2]
    assert state.bootstrap()['capabilities']['motionCommands'] is False
    assert state.health()['joints']['status'] == 'healthy'


def test_overlay_requires_compatible_timestamp():
    state = ConsoleState(overlay_max_age_s=0.1)
    state.update_detections(_detections(1_000_000_000))
    state.update_camera(_jpeg(1_050_000_000))
    stamp, annotated = state.frame_after(-1, timeout=0.01)
    assert stamp == 1_050_000_000
    assert annotated is not None
    image = cv2.imdecode(
        np.frombuffer(annotated, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert int(image.sum()) > 0

    stale = ConsoleState(overlay_max_age_s=0.01)
    source = _jpeg(2_000_000_000)
    stale.update_detections(_detections(1_000_000_000))
    stale.update_camera(source)
    _, untouched = stale.frame_after(-1, timeout=0.01)
    image = cv2.imdecode(
        np.frombuffer(untouched, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert int(image.sum()) == 0


def test_camera_health_reports_ingress_source():
    state = ConsoleState()
    state.update_camera(_jpeg(123), source='raw_jpeg_fallback')

    camera = state.health()['camera']
    assert camera['status'] == 'healthy'
    assert camera['source'] == 'raw_jpeg_fallback'
    assert state.bootstrap()['capabilities']['rawImageFallback'] is True


def test_system_telemetry_is_fresh_and_in_snapshot():
    state = ConsoleState()
    telemetry = {
        'hostname': 'pi5',
        'cpuTemperatureC': 52.4,
        'throttle': {'raw': '0x0', 'active': False, 'historical': False},
    }

    state.update_system(telemetry)

    assert state.health()['system']['status'] == 'healthy'
    assert state.live_snapshot()['system'] == telemetry


def test_linux_memory_and_pi_throttle_parsers(tmp_path: Path):
    meminfo = tmp_path / 'meminfo'
    meminfo.write_text(
        'MemTotal:       1000 kB\nMemAvailable:    250 kB\n',
        encoding='utf-8',
    )

    class Result:
        returncode = 0
        stdout = 'throttled=0x50005\n'

    assert _memory_used_percent(meminfo) == 75.0
    flags = _throttle_flags(lambda *_args, **_kwargs: Result())
    assert flags == {'raw': '0x50005', 'active': True, 'historical': True}


def test_system_sampling_does_not_block_ros_executor(monkeypatch, tmp_path):
    entered = threading.Event()
    release = threading.Event()
    updates = []

    def slow_collect(_root):
        entered.set()
        assert release.wait(timeout=1.0)
        return {'hostname': 'pi5'}

    monkeypatch.setattr(
        'arm_perception.web_console.collect_system_telemetry', slow_collect
    )
    harness = SimpleNamespace(
        _system_sample_lock=threading.Lock(),
        _recordings_root=tmp_path,
        state=SimpleNamespace(update_system=updates.append),
    )

    WebConsoleNode._sample_system_telemetry(harness)
    assert entered.wait(timeout=0.2)
    WebConsoleNode._sample_system_telemetry(harness)
    assert updates == []
    release.set()
    for _attempt in range(20):
        if updates:
            break
        threading.Event().wait(0.01)
    assert updates == [{'hostname': 'pi5'}]


class _FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message, **_kwargs):
        pass


def _transport_harness(mode='live', compressed_publishers=0):
    created = []
    destroyed = []
    raw_token = object()
    harness = SimpleNamespace(
        state=SimpleNamespace(health=lambda: {'mode': mode}),
        _compressed_image_topic='/camera/image_raw/compressed',
        compressed_publishers=compressed_publishers,
        _raw_subscription=None,
        _raw_image_topic='/camera/image_raw',
        _raw_image_qos=object(),
        _update_raw_camera=lambda _message: None,
        _live_only=lambda callback: callback,
        get_logger=lambda: _FakeLogger(),
    )
    harness.count_publishers = lambda topic: (
        harness.compressed_publishers
        if topic == harness._compressed_image_topic
        else 0
    )

    def create_subscription(*args):
        created.append(args)
        return raw_token

    def destroy_subscription(subscription):
        destroyed.append(subscription)
        return True

    harness.create_subscription = create_subscription
    harness.destroy_subscription = destroy_subscription
    harness._drop_raw_camera_subscription = lambda: (
        WebConsoleNode._drop_raw_camera_subscription(harness))
    return harness, raw_token, created, destroyed


def test_raw_camera_subscription_is_lazy_and_removed_for_compressed_feed():
    harness, raw_token, created, destroyed = _transport_harness()

    WebConsoleNode._manage_raw_camera_subscription(harness)
    assert harness._raw_subscription is raw_token
    assert len(created) == 1

    harness.compressed_publishers = 1
    WebConsoleNode._manage_raw_camera_subscription(harness)
    assert harness._raw_subscription is None
    assert destroyed == [raw_token]


def test_raw_camera_subscription_stays_off_during_replay():
    harness, _raw_token, created, _destroyed = _transport_harness(mode='replay')

    WebConsoleNode._manage_raw_camera_subscription(harness)

    assert harness._raw_subscription is None
    assert created == []


def test_websocket_server_frames_cover_all_payload_lengths():
    for payload in ('ok', 'x' * 200, 'x' * 70_000):
        frame = websocket_text_frame(payload)
        assert frame[0] == 0x81
        assert frame.endswith(payload.encode())


class FakeProcess:
    pid = 4242
    returncode = None

    def poll(self):
        return self.returncode


def test_recording_is_allowlisted_and_writes_truthful_metadata(tmp_path: Path):
    commands = []

    def fake_popen(command, **kwargs):
        commands.append((command, kwargs))
        return FakeProcess()

    state = ConsoleState()
    manager = RecordingManager(
        tmp_path, state, min_free_bytes=0, popen=fake_popen
    )

    manifest = manager.start_recording('Deneme / güvenli', 'hareket yok')
    command = commands[0][0]
    assert command[:3] == ['ros2', 'bag', 'record']
    assert command[-len(DEFAULT_RECORD_TOPICS):] == list(
        DEFAULT_RECORD_TOPICS
    )
    assert not any(
        'command' in topic or 'trajectory' in topic
        for topic in DEFAULT_RECORD_TOPICS
    )
    assert manifest['jointStateSemantics'] == 'commanded_open_loop'

    session_dir = tmp_path / manifest['id']
    saved = json.loads(
        (session_dir / 'session.json').read_text(encoding='utf-8')
    )
    assert saved['note'] == 'hareket yok'
    assert saved['status'] == 'recording'
    completed = manager._finish_recording('completed')
    assert completed['status'] == 'completed'


def test_recording_spawn_failure_is_persisted(tmp_path: Path):
    """A missing rosbag executable must not leave a recording marked active."""

    def failing_popen(_command, **_kwargs):
        raise OSError('ros2 unavailable')

    state = ConsoleState()
    manager = RecordingManager(
        tmp_path, state, min_free_bytes=0, popen=failing_popen
    )

    try:
        manager.start_recording('failure', '')
    except RuntimeError as exc:
        assert 'ros2 bag record' in str(exc)
    else:
        raise AssertionError('recording start should fail closed')

    manifest_path = next(tmp_path.glob('*/session.json'))
    document = json.loads(manifest_path.read_text(encoding='utf-8'))
    assert document['status'] == 'failed'
    assert manager.status()['recording'] is None


def test_replay_never_republishes_live_tf_topics():
    assert '/tf' in DEFAULT_RECORD_TOPICS
    assert '/tf_static' in DEFAULT_RECORD_TOPICS
    assert '/tf' not in REPLAY_TOPICS
    assert '/tf_static' not in REPLAY_TOPICS


def test_static_server_accepts_symlink_install_layout(tmp_path: Path):
    source = tmp_path / 'source-index.html'
    source.write_text('Robot Arm console', encoding='utf-8')
    web_root = tmp_path / 'install' / 'dist'
    web_root.mkdir(parents=True)
    (web_root / 'index.html').symlink_to(source)
    state = ConsoleState()
    recordings = RecordingManager(
        tmp_path / 'recordings', state, min_free_bytes=0
    )
    handler = make_http_handler(
        state, recordings, web_root, lambda _a, _b: None
    )
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/') as response:
            assert response.status == 200
            assert response.read() == b'Robot Arm console'
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
