#!/usr/bin/env python3
import subprocess
import math
import os
import numpy as np

# utils/ -> arm_tests/ -> src/ -> workspace kökü
WORKSPACE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..')
)

def get_quat(yaw):
    return (0, 0, math.sin(yaw/2), math.cos(yaw/2))

def get_quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    return q_x, q_y, q_z, q_w

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def calculate_pose_error(pred_pos, pred_quat, gt_pos, gt_quat, class_name):
    # Position error
    dx = pred_pos[0] - gt_pos[0]
    dy = pred_pos[1] - gt_pos[1]
    dz = pred_pos[2] - gt_pos[2]
    pos_err_m = math.sqrt(dx*dx + dy*dy + dz*dz)
    pos_err_mm = pos_err_m * 1000.0

    # Orientation error with symmetry
    R_pred = quat_to_matrix(pred_quat)
    R_gt = quat_to_matrix(gt_quat)
    
    z_pred = R_pred[:, 2]
    z_gt = R_gt[:, 2]
    
    dot_z = max(-1.0, min(1.0, np.dot(z_pred, z_gt)))
    tilt_err_rad = math.acos(dot_z)
    tilt_err_deg = math.degrees(tilt_err_rad)
    
    if class_name == 'yellow_cylinder':
        return pos_err_mm, tilt_err_deg
    else:
        _, _, yaw_pred = euler_from_quaternion(*pred_quat)
        _, _, yaw_gt = euler_from_quaternion(*gt_quat)
        
        yaw_diff = abs(yaw_pred - yaw_gt)
        yaw_diff_mod = (yaw_diff + math.pi/4) % (math.pi/2) - math.pi/4
        yaw_err_deg = math.degrees(abs(yaw_diff_mod))
        
        ori_err_deg = max(tilt_err_deg, yaw_err_deg)
        return pos_err_mm, ori_err_deg

def get_gt_pose_in_base_link(x, y, z, roll, pitch, yaw):
    return (x, y, z - 0.6), (roll, pitch, yaw)

def spawn_model(model_name, x, y, z, roll, pitch, yaw):
    sdf_path = os.path.join(
        WORKSPACE_ROOT, 'src', 'arm_gazebo', 'models', model_name, 'model.sdf'
    )
    qx, qy, qz, qw = get_quat_from_rpy(roll, pitch, yaw)
    
    req = f"""
    name: "{model_name}"
    sdf_filename: "{sdf_path}"
    pose: {{
      position: {{x: {x}, y: {y}, z: {z}}}
      orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}
    }}
    """
    cmd = [
        "gz", "service", "-s", "/world/pick_and_place_world/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", req.strip()
    ]
    res = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Spawn failed for {model_name}: {res.stderr}")

def remove_model(model_name):
    req = f'name: "{model_name}" type: MODEL'
    cmd = [
        "gz", "service", "-s", "/world/pick_and_place_world/remove",
        "--reqtype", "gz.msgs.Entity",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", req
    ]
    res = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"Remove failed for {model_name}: {res.stderr}")
