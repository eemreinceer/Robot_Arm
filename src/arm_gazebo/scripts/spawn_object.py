#!/usr/bin/env python3
import os
import subprocess
import math
import tempfile
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import SpawnObject
from geometry_msgs.msg import Pose

class SpawnObjectService(Node):
    def __init__(self):
        super().__init__('spawn_object_service')
        self.srv = self.create_service(SpawnObject, 'spawn_object', self.spawn_object_callback)
        self.get_logger().info('Spawn Object Service has been started.')

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

    def spawn_object_callback(self, request, response):
        name = request.object_name
        model_type = request.model_type.lower()
        pose = request.pose
        dims = request.dimensions
        mass = request.mass

        self.get_logger().info(f'Received spawn request for: {name} ({model_type})')

        # Convert quaternion to Euler for SDF pose
        roll, pitch, yaw = self.euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        # Generate SDF content based on shape type
        sdf_content = ""
        if model_type == "box":
            if len(dims) < 3:
                response.success = False
                response.message = "Dimensions array must have at least 3 values [x, y, z] for box"
                return response
            dx, dy, dz = dims[0], dims[1], dims[2]
            ixx = (1.0 / 12.0) * mass * (dy**2 + dz**2)
            iyy = (1.0 / 12.0) * mass * (dx**2 + dz**2)
            izz = (1.0 / 12.0) * mass * (dx**2 + dy**2)
            
            sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{pose.position.x} {pose.position.y} {pose.position.z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{dx} {dy} {dz}</size></box></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{dx} {dy} {dz}</size></box></geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.8 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        elif model_type == "cylinder":
            if len(dims) < 2:
                response.success = False
                response.message = "Dimensions array must have at least 2 values [radius, length] for cylinder"
                return response
            r, l = dims[0], dims[1]
            ixx = (1.0 / 12.0) * mass * (3 * r**2 + l**2)
            iyy = ixx
            izz = 0.5 * mass * r**2
            
            sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{pose.position.x} {pose.position.y} {pose.position.z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>
        <material>
          <ambient>0.1 0.8 0.1 1</ambient>
          <diffuse>0.1 0.8 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        elif model_type == "sphere":
            if len(dims) < 1:
                response.success = False
                response.message = "Dimensions array must have at least 1 value [radius] for sphere"
                return response
            r = dims[0]
            ixx = 0.4 * mass * r**2
            iyy = ixx
            izz = ixx
            
            sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{pose.position.x} {pose.position.y} {pose.position.z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <material>
          <ambient>0.1 0.1 0.8 1</ambient>
          <diffuse>0.1 0.1 0.8 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        else:
            response.success = False
            response.message = f"Unsupported model type: {model_type}"
            return response

        # Write to temporary file
        fd, temp_path = tempfile.mkstemp(suffix=".sdf", text=True)
        try:
            with os.fdopen(fd, 'w') as f:
                f.write(sdf_content)

            # Call Gazebo Entity Factory Service via CLI
            # Standard Gazebo service call
            cmd = [
                "gz", "service",
                "-s", "/world/pick_and_place_world/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000",
                "--req", f'sdf_filename: "{temp_path}" name: "{name}"'
            ]
            
            self.get_logger().info(f"Running command: {' '.join(cmd)}")
            res = subprocess.run(cmd, capture_output=True, text=True)
            
            if res.returncode == 0:
                response.success = True
                response.object_id = name
                response.message = f"Successfully spawned model {name}"
            else:
                response.success = False
                response.message = f"Failed to spawn model. CLI Stderr: {res.stderr}"
                self.get_logger().error(f"Gazebo service error: {res.stderr}")
                
        finally:
            # Clean up temp file
            if os.path.exists(temp_path):
                os.remove(temp_path)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpawnObjectService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
