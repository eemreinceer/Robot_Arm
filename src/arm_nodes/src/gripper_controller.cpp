#include "arm_nodes/gripper_controller.hpp"

#include <future>


namespace arm_nodes
{

GripperController::GripperController(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  action_name_ = node_->declare_parameter<std::string>(
    "gripper_action_name", "/gripper_controller/follow_joint_trajectory");
  client_ = rclcpp_action::create_client<GripperCommand>(node_, action_name_);
}

bool GripperController::open(std::chrono::seconds timeout)
{
  return send_command(-0.5736, 30.0, timeout);
}

bool GripperController::close(std::chrono::seconds timeout)
{
  return send_command(0.100, 50.0, timeout);
}

bool GripperController::grasp(double effort, std::chrono::seconds timeout)
{
  return send_command(0.100, effort, timeout);
}

bool GripperController::send_command(double position, double max_effort, std::chrono::seconds timeout)
{
  if (!client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server is not available: %s", action_name_.c_str());
    return false;
  }

  (void)max_effort;
  GripperCommand::Goal goal;
  goal.trajectory.joint_names = {"gripper_joint1", "gripper_joint2"};
  auto & point = goal.trajectory.points.emplace_back();
  point.positions = {position, -position - 0.1};
  point.time_from_start.sec = 2;
  point.time_from_start.nanosec = 0;

  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out sending gripper command");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper command was rejected");
    return false;
  }

  auto result_future = client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper trajectory result");
    return false;
  }

  return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

}  // namespace arm_nodes
