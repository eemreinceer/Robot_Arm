import numpy as np
import pytest

from arm_perception.planar_projection import (
    bbox_contact_pixel,
    bbox_to_object_position,
    image_pixel_to_plane_xy,
    object_dimensions,
)


def test_bbox_contact_pixel_uses_bottom_center():
    assert bbox_contact_pixel([10, 20, 30, 60]) == (20.0, 60.0)


def test_homography_projects_pixel_with_scale():
    homography = [0.001, 0.0, -0.32, 0.0, -0.001, 0.24, 0.0, 0.0, 1.0]
    assert np.allclose(image_pixel_to_plane_xy((420, 140), homography), (0.10, 0.10))


def test_bbox_position_lifts_center_by_known_height():
    homography = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 1.0]
    assert np.allclose(
        bbox_to_object_position([10, 20, 30, 60], homography, 0.12, 0.04),
        (0.02, 0.06, 0.14))


def test_projection_rejects_invalid_calibration_contracts():
    with pytest.raises(ValueError):
        bbox_contact_pixel([10, 20, 10, 30])
    with pytest.raises(ValueError):
        image_pixel_to_plane_xy((1, 2), [1, 2, 3])
    with pytest.raises(ValueError):
        bbox_to_object_position([0, 0, 1, 1], np.eye(3), 0.0, 0.0)
    with pytest.raises(KeyError):
        object_dimensions('missing', {})


def test_object_dimensions_returns_positive_xyz():
    assert object_dimensions('cube', {'cube': [0.02, 0.03, 0.04]}) == (0.02, 0.03, 0.04)
