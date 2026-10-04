#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace arm_nodes
{

class GripperController
{
public:
  using GripperCommand = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit GripperController(const rclcpp::Node::SharedPtr & node);

  // Generous timeouts: a slow-RTF sim plays gripper motion out in tens of
  // wall-seconds, so 10 s timed out mid-close and failed the grasp phase.
  bool open(std::chrono::seconds timeout = std::chrono::seconds(60));
  bool close(std::chrono::seconds timeout = std::chrono::seconds(60));
  bool grasp(double effort, std::chrono::seconds timeout = std::chrono::seconds(60));

private:
  bool send_command(double position, double max_effort, std::chrono::seconds timeout);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GripperCommand>::SharedPtr client_;
  std::string action_name_;
};

}  // namespace arm_nodes
