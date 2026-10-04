#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "arm_nodes/moveit_compat.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

geometry_msgs::msg::Pose top_down_pose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Rz(yaw) * Rx(pi): tool0 Z points vertically down. Yaw is intentionally
  // swept because the physical arm has five positioning joints; arbitrary 6D
  // pose constraints are over-constrained for this mechanism.
  pose.orientation.x = std::cos(yaw * 0.5);
  pose.orientation.y = std::sin(yaw * 0.5);
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

bool move_at_yaw(
  const rclcpp::Logger & logger, MoveGroupInterface & move_group,
  double x, double y, double z, double yaw, const std::string & phase)
{
  constexpr double kPi = 3.14159265358979323846;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(top_down_pose(x, y, z, yaw), "tool0");
  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group.plan(plan))) {
    RCLCPP_INFO(logger, "%s: yaw %.1f deg has no plan", phase.c_str(), yaw * 180.0 / kPi);
    move_group.clearPoseTargets();
    return false;
  }

  RCLCPP_INFO(logger, "%s: executing yaw %.1f deg", phase.c_str(), yaw * 180.0 / kPi);
  const bool success = move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  move_group.clearPoseTargets();
  if (!success) {
    RCLCPP_WARN(logger, "%s: execution failed at yaw %.1f deg", phase.c_str(), yaw * 180.0 / kPi);
  }
  return success;
}

std::optional<double> reach_with_free_yaw(
  const rclcpp::Logger & logger, MoveGroupInterface & move_group,
  double x, double y, double z)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<double> yaw_candidates = {
    0.0, kPi / 4.0, -kPi / 4.0, kPi / 2.0, -kPi / 2.0, kPi, 3.0 * kPi / 4.0,
    -3.0 * kPi / 4.0};

  for (const double yaw : yaw_candidates) {
    if (move_at_yaw(logger, move_group, x, y, z, yaw, "reach")) {
      return yaw;
    }
  }

  RCLCPP_ERROR(logger, "reach: all top-down yaw candidates failed");
  return std::nullopt;
}

bool command_gripper(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr & client,
  double position, const std::string & phase)
{
  if (!client->wait_for_action_server(30s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server is unavailable");
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"joint_6"};
  auto & point = goal.trajectory.points.emplace_back();
  point.positions = {position};
  point.time_from_start.sec = 2;

  auto goal_future = client->async_send_goal(goal);
  if (goal_future.wait_for(30s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "%s gripper goal timed out", phase.c_str());
    return false;
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "%s gripper goal was rejected", phase.c_str());
    return false;
  }
  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(60s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "%s gripper command timed out", phase.c_str());
    return false;
  }
  const bool success = result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  RCLCPP_INFO(node->get_logger(), "%s gripper command: %s", phase.c_str(), success ? "OK" : "FAILED");
  return success;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("robot_arm_top_down_pick_demo");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  const double cube_x = node->declare_parameter("cube_x", 0.18);
  const double cube_y = node->declare_parameter("cube_y", 0.0);
  const double cube_z = node->declare_parameter("cube_z", 0.025);
  const double approach_height = node->declare_parameter("approach_height", 0.08);
  const double lift_height = node->declare_parameter("lift_height", 0.12);
  const double open_position = node->declare_parameter("gripper_open", 0.3);
  const double closed_position = node->declare_parameter("gripper_closed", -0.5);

  MoveGroupInterface move_group(node, "arm");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.10);

  const auto gripper = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/robot_arm_gripper_controller/follow_joint_trajectory");

  bool success = command_gripper(node, gripper, open_position, "open");
  const auto selected_yaw = success ? reach_with_free_yaw(
    node->get_logger(), move_group, cube_x, cube_y, cube_z + approach_height) : std::nullopt;
  success = success && selected_yaw.has_value();
  if (success) {
    success = move_at_yaw(
      node->get_logger(), move_group, cube_x, cube_y, cube_z, *selected_yaw, "descend");
  }
  if (success) {
    success = command_gripper(node, gripper, closed_position, "close");
  }
  if (success) {
    success = move_at_yaw(
      node->get_logger(), move_group, cube_x, cube_y, cube_z + lift_height,
      *selected_yaw, "lift");
  }

  RCLCPP_INFO(node->get_logger(), "Robot Arm top-down pick demo: %s", success ? "PASS" : "FAIL");
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
