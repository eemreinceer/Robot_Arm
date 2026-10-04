#!/usr/bin/env python3
"""
ROS2 MCP Server for 6DOF Robotic Arm Project
Exposes ROS2 CLI tools securely over MCP using python-mcp FastMCP.
Runs inside WSL to interact with ROS2 Jazzy.
"""

import json
import subprocess
from mcp.server.fastmcp import FastMCP

# Initialize FastMCP Server
mcp = FastMCP("ROS2-Helper")

def run_ros2_cmd(args, timeout=5.0):
    """Helper to run ROS2 commands inside bash with ROS2 sourced."""
    cmd = ["bash", "-c", f"source /opt/ros/jazzy/setup.bash && {' '.join(args)}"]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if res.returncode == 0:
            return res.stdout.strip()
        else:
            return f"Error (Exit Code {res.returncode}):\n{res.stderr.strip()}"
    except subprocess.TimeoutExpired:
        return f"Error: Command timed out after {timeout} seconds."
    except Exception as e:
        return f"Error executing command: {str(e)}"

@mcp.tool()
def ros2_node_list() -> str:
    """List all active ROS2 nodes in the system."""
    return run_ros2_cmd(["ros2", "node", "list"])

@mcp.tool()
def ros2_topic_list() -> str:
    """List all active ROS2 topics with their types."""
    return run_ros2_cmd(["ros2", "topic", "list", "-t"])

@mcp.tool()
def ros2_topic_info(topic_name: str) -> str:
    """Get detailed info about a specific ROS2 topic.
    
    Args:
        topic_name: The name of the topic (e.g., '/joint_states')
    """
    return run_ros2_cmd(["ros2", "topic", "info", topic_name])

@mcp.tool()
def ros2_topic_echo(topic_name: str, timeout_sec: float = 3.0) -> str:
    """Echo the latest message from a ROS2 topic.
    
    Args:
        topic_name: The name of the topic (e.g., '/joint_states')
        timeout_sec: Timeout in seconds to wait for a message (default: 3.0)
    """
    # Wait for a single message and then exit
    return run_ros2_cmd(["ros2", "topic", "echo", "--once", topic_name], timeout=timeout_sec + 2.0)

@mcp.tool()
def ros2_service_list() -> str:
    """List all active ROS2 services with their types."""
    return run_ros2_cmd(["ros2", "service", "list", "-t"])

@mcp.tool()
def ros2_service_call(service_name: str, service_type: str, json_args: str = "{}") -> str:
    """Call a ROS2 service.
    
    Args:
        service_name: The name of the service (e.g., '/spawn_object')
        service_type: The type of the service (e.g., 'arm_interfaces/srv/SpawnObject')
        json_args: JSON string containing the arguments (e.g., '{"name": "box"}')
    """
    # Escape quotes for bash execution
    safe_args = json_args.replace("'", "'\\''")
    return run_ros2_cmd(["ros2", "service", "call", service_name, service_type, f"'{safe_args}'"], timeout=15.0)

@mcp.tool()
def ros2_action_list() -> str:
    """List all active ROS2 actions with their types."""
    return run_ros2_cmd(["ros2", "action", "list", "-t"])

@mcp.tool()
def ros2_action_send_goal(action_name: str, action_type: str, json_args: str = "{}") -> str:
    """Send a goal to a ROS2 action server.
    
    Args:
        action_name: The name of the action (e.g., '/pick_and_place')
        action_type: The type of the action (e.g., 'arm_interfaces/action/PickAndPlace')
        json_args: JSON string containing the goal arguments
    """
    safe_args = json_args.replace("'", "'\\''")
    return run_ros2_cmd(["ros2", "action", "send_goal", action_name, action_type, f"'{safe_args}'"], timeout=30.0)

@mcp.tool()
def ros2_param_list(node_name: str) -> str:
    """List parameters for a specific ROS2 node.
    
    Args:
        node_name: The name of the node (e.g., '/move_group')
    """
    return run_ros2_cmd(["ros2", "param", "list", node_name])

@mcp.tool()
def ros2_param_get(node_name: str, param_name: str) -> str:
    """Get the value of a parameter on a ROS2 node.
    
    Args:
        node_name: The name of the node
        param_name: The name of the parameter
    """
    return run_ros2_cmd(["ros2", "param", "get", node_name, param_name])

@mcp.tool()
def ros2_param_set(node_name: str, param_name: str, json_value: str) -> str:
    """Set the value of a parameter on a ROS2 node.
    
    Args:
        node_name: The name of the node
        param_name: The name of the parameter
        json_value: JSON representation of the value (e.g., 'true', '1.0', '"hello"')
    """
    return run_ros2_cmd(["ros2", "param", "set", node_name, param_name, json_value])

if __name__ == "__main__":
    mcp.run()
