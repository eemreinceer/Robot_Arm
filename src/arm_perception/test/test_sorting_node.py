from pathlib import Path
from types import SimpleNamespace

from arm_interfaces.msg import ObjectPose

from arm_perception.autonomous_pick_node import AutonomousPickNode
from arm_perception.sorting_config import load_sorting_config


CONFIG = Path(__file__).parents[1] / 'config' / 'sorting_bins.yaml'


def _object(object_type: str, x: float, y: float) -> ObjectPose:
    detected = ObjectPose()
    detected.object_id = f'{object_type}_0'
    detected.object_type = object_type
    detected.confidence = 0.9
    detected.pose.pose.position.x = x
    detected.pose.pose.position.y = y
    return detected


def test_sorting_routes_known_class_and_filters_bin_occupants():
    node = SimpleNamespace(
        sort_all=True,
        bins=load_sorting_config(CONFIG).bins,
        active_sorting_type='',
        _warned_unknown_types=set(),
    )
    red = node.bins['red_box']
    unsorted = _object('red_box', 0.34, 0.00)
    sorted_object = _object('red_box', red.spawn_world_xyz[0], red.spawn_world_xyz[1])
    assert list(AutonomousPickNode._candidate_objects(node, [unsorted])) == [unsorted]
    assert list(AutonomousPickNode._candidate_objects(node, [sorted_object])) == []
    pose = AutonomousPickNode._place_pose(node, 'red_box')
    assert pose.position.x == red.drop_link6_xyz[0]
    assert pose.position.y == red.drop_link6_xyz[1]


def test_post_place_sync_uses_safe_hover_place_pose():
    config = load_sorting_config(CONFIG)
    hover = 0.20
    node = SimpleNamespace(
        sort_all=True,
        bins=config.bins,
        _exact_pose_enabled=lambda: False,
        _post_place_sync_enabled=lambda: True,
        get_parameter=lambda name: SimpleNamespace(value={
            'simulation_place_hover_offset_m': hover,
        }[name]),
    )
    red = config.bins['red_box']
    pose = AutonomousPickNode._place_pose(node, 'red_box')
    assert pose.position.x == red.drop_link6_xyz[0]
    assert pose.position.y == red.drop_link6_xyz[1]
    assert pose.position.z == red.drop_link6_xyz[2] + hover


def test_simulation_fast_sort_syncs_one_unsorted_model_per_completion():
    moved = []
    parameters = {
        'simulation_sync_enabled': True,
        'simulation_objects_per_class': 2,
    }
    node = SimpleNamespace(
        selected_object_id='red_box_0',
        selected_object_type='red_box',
        active_sorting_model='sorting_red_box_01',
        bins=load_sorting_config(CONFIG).bins,
        simulation_synced_models=set(),
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None, warning=lambda _message: None),
        _set_simulation_model_pose=lambda model_name, _bin_spec: moved.append(model_name) or True,
    )
    assert AutonomousPickNode._sync_simulation_object_to_bin(node)
    assert moved == ['sorting_red_box_01']
    assert node.simulation_synced_models == {'sorting_red_box_01'}
    node.active_sorting_model = ''
    assert AutonomousPickNode._sync_simulation_object_to_bin(node)
    assert moved == ['sorting_red_box_01', 'sorting_red_box_00']
    assert node.simulation_synced_models == set(moved)


def test_post_place_sync_enabled_requires_sorting_and_sim_sync():
    parameters = {
        'simulation_sync_enabled': True,
        'simulation_post_place_sync_enabled': True,
    }
    node = SimpleNamespace(
        sort_all=True,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
    )
    assert AutonomousPickNode._post_place_sync_enabled(node)
    node.sort_all = False
    assert not AutonomousPickNode._post_place_sync_enabled(node)
    node.sort_all = True
    parameters['simulation_sync_enabled'] = False
    assert not AutonomousPickNode._post_place_sync_enabled(node)


def test_successful_sorting_result_syncs_sim_pose_before_object_done():
    finishes = []
    node = SimpleNamespace(
        sort_all=True,
        simulation_synced_for_goal=False,
        get_parameter=lambda name: SimpleNamespace(value={
            'simulation_fast_sort_enabled': False,
            'simulation_sync_enabled': True,
            'simulation_post_place_sync_enabled': True,
        }[name]),
        _post_place_sync_enabled=lambda: AutonomousPickNode._post_place_sync_enabled(node),
        _sync_simulation_object_to_bin=lambda: True,
        _finish=lambda message, completed=False: finishes.append((message, completed)),
    )
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            result=SimpleNamespace(success=True, message='Pick/place completed')))
    AutonomousPickNode._on_result(node, future)
    assert node.simulation_synced_for_goal
    assert finishes == [('Pick result: success=True Pick/place completed', True)]


def test_successful_sorting_result_waits_for_required_sim_sync():
    finishes = []
    node = SimpleNamespace(
        sort_all=True,
        simulation_synced_for_goal=False,
        get_parameter=lambda name: SimpleNamespace(value={
            'simulation_fast_sort_enabled': False,
            'simulation_sync_enabled': True,
            'simulation_post_place_sync_enabled': True,
        }[name]),
        _post_place_sync_enabled=lambda: AutonomousPickNode._post_place_sync_enabled(node),
        _sync_simulation_object_to_bin=lambda: False,
        _finish=lambda message, completed=False: finishes.append((message, completed)),
    )
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            result=SimpleNamespace(success=True, message='Pick/place completed')))
    AutonomousPickNode._on_result(node, future)
    assert not node.simulation_synced_for_goal
    assert finishes == [('Pick result: success=True Pick/place completed', False)]


def test_finish_publishes_object_done_once_for_completed_sorting_goal():
    published = []
    node = SimpleNamespace(
        sort_all=True,
        object_done_published_for_goal=False,
        selected_object_id='red_box_0',
        selected_object_type='red_box',
        active_sorting_model='sorting_red_box_00',
        object_done_publisher=SimpleNamespace(publish=lambda message: published.append(message.data)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        busy=True,
        active_goal_handle=object(),
    )
    node._publish_object_done = lambda: AutonomousPickNode._publish_object_done(node)
    AutonomousPickNode._finish(node, 'done', completed=True)
    AutonomousPickNode._finish(node, 'done again', completed=True)
    assert published == ['sorting_red_box_00']
    assert not node.busy
    assert node.active_goal_handle is None


def test_finish_loop_protection_does_not_publish_object_done():
    published = []
    warnings = []
    node = SimpleNamespace(
        sort_all=True,
        object_done_published_for_goal=False,
        selected_object_id='red_box_0',
        selected_object_type='red_box',
        active_sorting_model='sorting_red_box_00',
        _active_model_fail_count=2,
        _max_consecutive_fails=3,
        object_done_publisher=SimpleNamespace(publish=lambda message: published.append(message.data)),
        get_logger=lambda: SimpleNamespace(
            info=lambda _message: None,
            warning=lambda message: warnings.append(message),
        ),
        busy=True,
        active_goal_handle=object(),
    )
    node._publish_object_done = lambda: AutonomousPickNode._publish_object_done(node)
    AutonomousPickNode._finish(node, 'failed', completed=False)
    assert published == []
    assert node._active_model_fail_count == 0
    assert any('not signalling object done' in message for message in warnings)
    assert not node.busy
    assert node.active_goal_handle is None


def test_candidate_objects_filters_to_active_sorting_type():
    node = SimpleNamespace(
        sort_all=True,
        bins=load_sorting_config(CONFIG).bins,
        active_sorting_type='yellow_cylinder',
        _warned_unknown_types=set(),
    )
    red = _object('red_box', 0.46, 0.15)
    yellow = _object('yellow_cylinder', 0.46, 0.15)
    assert list(AutonomousPickNode._candidate_objects(node, [red, yellow])) == [yellow]


def test_return_home_policy_returns_home_after_each_goal():
    assert AutonomousPickNode._return_home_after_goal(SimpleNamespace(sort_all=False))
    assert AutonomousPickNode._return_home_after_goal(SimpleNamespace(sort_all=True))
