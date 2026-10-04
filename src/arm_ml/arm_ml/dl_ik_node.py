#!/usr/bin/env python3
"""ROS2 service node for ML IK seed prediction."""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import SolveIk

try:
    import torch
except ImportError:  # pragma: no cover
    torch = None

from arm_ml.kinematics import denormalize_joints, forward_kinematics, normalize_pose
from arm_ml.model import IKNet


class DlIkNode(Node):
    def __init__(self) -> None:
        super().__init__("dl_ik_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("service_name", "dl_ik_solve")
        service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.model = None
        self.device = "cpu"
        self._load_model()
        self.service = self.create_service(SolveIk, service_name, self._handle_solve)
        self.get_logger().info(f"dl_ik_node ready on /{service_name}")

    def _load_model(self) -> None:
        if torch is None:
            self.get_logger().error("PyTorch is not available; /dl_ik_solve will report failure")
            return
        configured = self.get_parameter("model_path").get_parameter_value().string_value
        model_path = Path(configured) if configured else Path("src/arm_ml/models/ik_net_scripted.pt")
        if not model_path.exists():
            self.get_logger().warning(f"ML IK model not found: {model_path}")
            return
        try:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            if model_path.name.endswith("_scripted.pt"):
                self.model = torch.jit.load(str(model_path), map_location=self.device)
            else:
                self.model = IKNet().to(self.device)
                checkpoint = torch.load(str(model_path), map_location=self.device)
                state = checkpoint.get("model_state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
                self.model.load_state_dict(state)
            self.model.eval()
            with torch.no_grad():
                warmup = torch.zeros((1, 7), dtype=torch.float32, device=self.device)
                for _ in range(5):
                    self.model(warmup)
            self.get_logger().info(f"Loaded ML IK model: {model_path}")
        except Exception as exc:  # pragma: no cover
            self.model = None
            self.get_logger().error(f"Failed to load ML IK model {model_path}: {exc}")

    def _handle_solve(self, request: SolveIk.Request, response: SolveIk.Response) -> SolveIk.Response:
        if self.model is None or torch is None:
            response.success = False
            response.position_error = -1.0
            response.message = "ML IK model is not loaded"
            return response

        pose = np.array(
            [
                request.target_pose.position.x,
                request.target_pose.position.y,
                request.target_pose.position.z,
                request.target_pose.orientation.x,
                request.target_pose.orientation.y,
                request.target_pose.orientation.z,
                request.target_pose.orientation.w,
            ],
            dtype=np.float32,
        )
        pose = normalize_pose(pose)
        start = time.perf_counter()
        with torch.no_grad():
            tensor = torch.from_numpy(pose).reshape(1, 7).to(self.device)
            normalized_joints = self.model(tensor).cpu().numpy()[0]
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        joints = denormalize_joints(normalized_joints)
        for i, value in enumerate(joints.tolist()):
            response.joint_angles[i] = float(value)
        predicted_position = forward_kinematics(joints)[:3, 3]
        position_error = float(np.linalg.norm(predicted_position - pose[:3]))
        response.success = True
        response.position_error = position_error
        response.message = (
            f"ML IK seed predicted in {elapsed_ms:.3f} ms "
            f"with {position_error * 1000.0:.3f} mm FK reconstruction error"
        )
        return response


def main() -> None:
    rclpy.init()
    node = DlIkNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
