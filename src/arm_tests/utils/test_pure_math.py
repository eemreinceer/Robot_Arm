#!/usr/bin/env python3
import math
import numpy as np
from gz_gt import calculate_pose_error, get_quat_from_rpy, euler_from_quaternion, get_gt_pose_in_base_link

def get_quat(yaw):
    # pitch, roll = 0
    return (0, 0, math.sin(yaw/2), math.cos(yaw/2))

def test_box_symmetry():
    gt_pos = (0,0,0)
    pred_pos = (0,0,0)
    
    # 0 degrees diff
    err_mm, err_deg = calculate_pose_error(pred_pos, get_quat(0), gt_pos, get_quat(0), 'red_box')
    assert err_deg < 1e-5
    
    # 90 degrees diff (should be 0 error due to symmetry)
    err_mm, err_deg = calculate_pose_error(pred_pos, get_quat(math.pi/2), gt_pos, get_quat(0), 'red_box')
    assert err_deg < 1e-5
    
    # 45 degrees diff (max error, should be 45)
    err_mm, err_deg = calculate_pose_error(pred_pos, get_quat(math.pi/4), gt_pos, get_quat(0), 'red_box')
    assert abs(err_deg - 45.0) < 1e-5
    
    # 100 degrees diff (should be 10 error)
    err_mm, err_deg = calculate_pose_error(pred_pos, get_quat(100 * math.pi/180), gt_pos, get_quat(0), 'red_box')
    assert abs(err_deg - 10.0) < 1e-5

def test_cyl_symmetry():
    gt_pos = (0,0,0)
    pred_pos = (0,0,0)
    
    # Cylinder rotated around Z by 180 degrees -> 0 error
    err_mm, err_deg = calculate_pose_error(pred_pos, get_quat(math.pi), gt_pos, get_quat(0), 'yellow_cylinder')
    assert err_deg < 1e-5

    # Cylinder tilted by 30 degrees around X
    # q = (sin(15), 0, 0, cos(15))
    qx = math.sin(math.radians(15))
    qw = math.cos(math.radians(15))
    err_mm, err_deg = calculate_pose_error(pred_pos, (qx, 0, 0, qw), gt_pos, get_quat(0), 'yellow_cylinder')
    assert abs(err_deg - 30.0) < 1e-5

def test_position_error():
    gt_pos = (0.5, 0.2, 0.1)
    # offset by 10mm in X (0.01 meters)
    pred_pos = (0.51, 0.2, 0.1)
    err_mm, _ = calculate_pose_error(pred_pos, get_quat(0), gt_pos, get_quat(0), 'red_box')
    assert abs(err_mm - 10.0) < 1e-4

    # offset by 5mm in Y, 5mm in Z, 5mm in X
    # sqrt(5^2 + 5^2 + 5^2) = sqrt(75) approx 8.66025 mm
    pred_pos = (0.505, 0.205, 0.105)
    err_mm, _ = calculate_pose_error(pred_pos, get_quat(0), gt_pos, get_quat(0), 'red_box')
    assert abs(err_mm - math.sqrt(75.0)) < 1e-4

def test_rpy_quaternion_round_trip():
    # test a few angles (in radians)
    angles_to_test = [
        (0.0, 0.0, 0.0),
        (0.1, -0.2, 0.3),
        (math.pi/6, math.pi/4, math.pi/3),
        (-math.pi/4, 0.0, math.pi/2)
    ]
    for r, p, y in angles_to_test:
        qx, qy, qz, qw = get_quat_from_rpy(r, p, y)
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert abs(norm - 1.0) < 1e-6
        
        r_out, p_out, y_out = euler_from_quaternion(qx, qy, qz, qw)
        assert abs(r_out - r) < 1e-5
        assert abs(p_out - p) < 1e-5
        assert abs(y_out - y) < 1e-5

def test_gt_pose_in_base_link():
    x, y, z = 0.4, -0.3, 0.8
    roll, pitch, yaw = 0.1, 0.2, 0.3
    base_pos, base_rpy = get_gt_pose_in_base_link(x, y, z, roll, pitch, yaw)
    
    assert abs(base_pos[0] - x) < 1e-6
    assert abs(base_pos[1] - y) < 1e-6
    assert abs(base_pos[2] - (z - 0.6)) < 1e-6
    assert abs(base_rpy[0] - roll) < 1e-6
    assert abs(base_rpy[1] - pitch) < 1e-6
    assert abs(base_rpy[2] - yaw) < 1e-6

def test_box_tilt():
    gt_pos = (0, 0, 0)
    pred_pos = (0, 0, 0)
    
    # Box tilted by 10 degrees around X axis (roll)
    q_pred = get_quat_from_rpy(math.radians(10), 0, 0)
    err_mm, err_deg = calculate_pose_error(pred_pos, q_pred, gt_pos, get_quat(0), 'red_box')
    assert abs(err_deg - 10.0) < 1e-5

    # Box tilted by 15 degrees around Y axis (pitch)
    q_pred = get_quat_from_rpy(0, math.radians(15), 0)
    err_mm, err_deg = calculate_pose_error(pred_pos, q_pred, gt_pos, get_quat(0), 'red_box')
    assert abs(err_deg - 15.0) < 1e-5

if __name__ == '__main__':
    test_box_symmetry()
    test_cyl_symmetry()
    test_position_error()
    test_rpy_quaternion_round_trip()
    test_gt_pose_in_base_link()
    test_box_tilt()
    print("All pure math symmetry and validation tests passed!")
