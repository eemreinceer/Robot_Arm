import numpy as np

from arm_perception.pose_estimation import (
    camera_link_to_optical_points,
    estimate_pose,
    points_from_bbox,
)


def test_camera_link_points_are_rotated_to_rep103_optical_frame():
    camera_link = np.array([[[1.0, 2.0, 3.0]]])
    optical = camera_link_to_optical_points(camera_link)
    np.testing.assert_allclose(optical, [[[-2.0, -3.0, 1.0]]])


def test_bbox_depth_filter_accepts_normalized_gazebo_points():
    camera_link = np.array(
        [[[0.9, 0.02, -0.10], [0.91, -0.02, -0.11]]],
        dtype=np.float64,
    )
    optical = camera_link_to_optical_points(camera_link)
    cluster = points_from_bbox(optical, width=2, height=1, bbox=(0, 0, 2, 1))
    assert cluster.shape == (2, 3)
    assert np.all(cluster[:, 2] > 0.0)


def test_top_pick_link6_offset_keeps_tcp_above_object():
    from arm_perception.pose_estimation import top_pick_link6_transform

    target_from_object = np.eye(4)
    target_from_object[:3, 3] = [0.40, 0.08, 0.05]
    target_from_link6 = top_pick_link6_transform(
        target_from_object, [0.0, 0.0, 0.1023])
    np.testing.assert_allclose(target_from_link6[:3, 3], [0.40, 0.08, 0.1523])


def test_estimate_pose_uses_cluster_extent_center_not_point_median():
    xs = np.linspace(-0.025, 0.025, 6)
    ys = np.linspace(-0.025, 0.025, 6)
    top = np.array([[x, y, 0.025] for x in xs for y in ys], dtype=np.float64)
    side = np.array([[0.025, -0.025, z] for z in np.linspace(-0.025, 0.025, 6)], dtype=np.float64)
    cluster = np.vstack([top, side])

    estimate = estimate_pose(cluster, minimum_points=20)

    np.testing.assert_allclose(estimate.position, [0.0, 0.0, 0.0], atol=0.006)
