#pragma once

#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#if __has_include("moveit/move_group_interface/move_group_interface.hpp")
#define ARM_NODES_MOVEIT_USES_HPP_HEADERS 1
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit/utils/moveit_error_code.hpp"
#elif __has_include("moveit/move_group_interface/move_group_interface.h")
#define ARM_NODES_MOVEIT_USES_HPP_HEADERS 0
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/utils/moveit_error_code.h"
#else
#error "MoveIt MoveGroupInterface headers were not found"
#endif

namespace arm_nodes::moveit_compat
{

inline double compute_cartesian_path(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  bool avoid_collisions = true,
  moveit_msgs::msg::MoveItErrorCodes * error_code = nullptr)
{
#if ARM_NODES_MOVEIT_USES_HPP_HEADERS
  return move_group.computeCartesianPath(
    waypoints, eef_step, trajectory, avoid_collisions, error_code);
#else
  constexpr double kDisabledJumpThreshold = 0.0;
  return move_group.computeCartesianPath(
    waypoints, eef_step, kDisabledJumpThreshold, trajectory, avoid_collisions, error_code);
#endif
}

}  // namespace arm_nodes::moveit_compat

#undef ARM_NODES_MOVEIT_USES_HPP_HEADERS
