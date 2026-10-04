from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from arm_interfaces.msg import ObjectArray, ObjectPose

import arm_perception.sorting_scene as sorting_scene
from arm_perception.sorting_config import load_sorting_config
from arm_perception.sorting_scene import LEGACY_WORLD_MODELS, SortingSceneNode


CONFIG = Path(__file__).parents[1] / 'config' / 'sorting_bins.yaml'


def _legacy_models_available() -> bool:
    """`arm_gazebo` bu workspace'te derleniyor mu?

    Nesne modelleri legacy `arm_gazebo` paketinde ve o paket 2026-08-07'de
    BİLEREK build dışına alındı (`src/LEGACY_6DOF.md`, beş legacy pakette
    COLCON_IGNORE). Robot Arm hattı bu modelleri kullanmıyor; ama sorting sahnesinin
    kendisi hâlâ legacy sim'in parçası ve orada doğru çalışıyor.

    Yani paketin yokluğu bir kusur değil, workspace'in yapılandırması. Bunu
    başarısızlık saymak `colcon test`i kalıcı kırmızı bırakır ve gerçek bir
    regresyon çıktığında kimse fark etmez. Legacy sim derlendiğinde test
    kendiliğinden geri gelir.
    """
    try:
        get_package_share_directory('arm_gazebo')
    except PackageNotFoundError:
        return False
    return True


requires_legacy_models = pytest.mark.skipif(
    not _legacy_models_available(),
    reason="legacy 'arm_gazebo' paketi build disi (COLCON_IGNORE); "
           'sorting nesne modelleri orada yasiyor',
)


def _detection(object_type: str, x: float, y: float) -> ObjectPose:
    detected = ObjectPose()
    detected.object_type = object_type
    detected.pose.pose.position.x = x
    detected.pose.pose.position.y = y
    return detected


def test_reset_scene_removes_phase1_world_models_before_sorting_assets():
    removed = []
    node = SimpleNamespace(
        config=load_sorting_config(CONFIG),
        publish_scene_ready=lambda ready: removed.append(f'ready:{ready}'),
        publish_active_object=lambda model, object_type: removed.append(f'active:{model}:{object_type}'),
        list_models=lambda: {
            *LEGACY_WORLD_MODELS, 'bin_red', 'sorting_blue_cube_09', 'unrelated'},
        remove_model=removed.append,
    )
    SortingSceneNode.reset_scene(node)
    assert removed[0] == 'ready:False'
    assert removed[1] == 'active::'
    assert removed[2:4] == list(LEGACY_WORLD_MODELS)
    assert 'bin_red' in removed
    assert 'sorting_blue_cube_09' in removed
    assert 'unrelated' not in removed


def test_unsorted_detection_gate_ignores_bins_and_unknown_classes():
    config = load_sorting_config(CONFIG)
    red_bin = config.bins['red_box']
    message = ObjectArray()
    message.objects = [
        _detection('red_box', 0.35, 0.0),
        _detection('red_box', red_bin.spawn_world_xyz[0], red_bin.spawn_world_xyz[1]),
        _detection('unknown', 0.34, 0.0),
    ]
    node = SimpleNamespace(config=config, latest_detections=message)
    detections = SortingSceneNode._unsorted_detections(node)
    assert len(detections) == 1
    assert detections[0].object_type == 'red_box'


def test_object_sequence_is_round_robin_by_class_then_index():
    node = SimpleNamespace(args=SimpleNamespace(objects_per_class=2))
    assert list(SortingSceneNode.object_sequence(node)) == [
        ('red_box', 0),
        ('yellow_cylinder', 0),
        ('blue_cube', 0),
        ('red_box', 1),
        ('yellow_cylinder', 1),
        ('blue_cube', 1),
    ]


@requires_legacy_models
def test_spawn_one_object_spawns_only_requested_model():
    spawned = []
    config = load_sorting_config(CONFIG)
    node = SimpleNamespace(
        config=config,
        gazebo_share=Path('/tmp/gazebo'),
        spawn_model=lambda name, model_file, xyz: spawned.append((name, model_file, xyz)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    node._sample_object_xy = lambda rng, occupied: SortingSceneNode._sample_object_xy(
        node, rng, occupied)
    model_name = SortingSceneNode.spawn_one_object(
        node, 'red_box', 1, random.Random(7), [])
    assert model_name == 'sorting_red_box_01'
    assert len(spawned) == 1
    assert spawned[0][0] == 'sorting_red_box_01'
    x, y, z = spawned[0][2]
    x_range, y_range = config.object_zone_world
    assert x_range[0] <= x <= x_range[1]
    assert y_range[0] <= y <= y_range[1]
    assert z == config.table_world_z + 0.025


def test_publish_active_object_uses_model_and_type_payload():
    published = []
    node = SimpleNamespace(
        active_object_publisher=SimpleNamespace(publish=lambda message: published.append(message.data)),
    )
    SortingSceneNode.publish_active_object(node, 'sorting_red_box_00', 'red_box')
    assert published == ['sorting_red_box_00 red_box']


def test_wait_for_object_done_accepts_only_matching_model(monkeypatch):
    warnings = []
    node = SimpleNamespace(
        args=SimpleNamespace(object_timeout=0.01),
        object_done_time=10.1,
        last_completed_object='sorting_yellow_cylinder_00',
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            warning=lambda message: warnings.append(message),
        ),
    )
    monkeypatch.setattr(sorting_scene.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(sorting_scene.rclpy, 'spin_once', lambda *_args, **_kwargs: None)

    with pytest.raises(RuntimeError, match='sorting_red_box_00'):
        SortingSceneNode.wait_for_object_done(node, 10.0, 'sorting_red_box_00')

    assert warnings == [
        'Object gate ignored: expected sorting_red_box_00, '
        'signal=sorting_yellow_cylinder_00'
    ]


def test_wait_for_object_done_passes_matching_model(monkeypatch):
    infos = []
    node = SimpleNamespace(
        args=SimpleNamespace(object_timeout=1.0),
        object_done_time=10.1,
        last_completed_object='sorting_red_box_00',
        get_logger=lambda: SimpleNamespace(
            info=lambda message: infos.append(message),
            warning=lambda _message: None,
        ),
    )
    monkeypatch.setattr(sorting_scene.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(sorting_scene.rclpy, 'spin_once', lambda *_args, **_kwargs: None)

    SortingSceneNode.wait_for_object_done(node, 10.0, 'sorting_red_box_00')

    assert infos == [
        'Object gate PASS: sorting_red_box_00 completed '
        '(signal=sorting_red_box_00)'
    ]
