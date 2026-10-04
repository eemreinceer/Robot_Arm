#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "arm_interfaces/action/pick_and_place.hpp"
#include "arm_interfaces/msg/object_array.hpp"
#include "arm_interfaces/msg/object_pose.hpp"
#include "arm_interfaces/srv/solve_ik.hpp"
#include "arm_nodes/gripper_controller.hpp"
#include "arm_nodes/moveit_compat.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace arm_nodes
{

enum class Phase
{
  IDLE,
  HOME_START,
  DETECT_OBJECT,
  PLAN_PRE_GRASP,
  MOVE_TO_PRE_GRASP,
  DESCEND_TO_GRASP,
  CLOSE_GRIPPER,
  LIFT_OBJECT,
  MOVING_TO_PRE_PLACE,
  DESCEND_TO_PLACE,
  OPEN_GRIPPER,
  RETREAT,
  HOME_RETURN,
  DONE,
  FAILED,
  CANCELED
};

// A pre-grasp/pre-place approach that shares a single IK branch with the grasp.
// Computing the pre-pose as the JOINT solution lifted off the grasp solution
// (instead of a free pose plan that picks an arbitrary wrist branch) is what
// lets the straight-line descent stay on one configuration -> clean approach.
struct ApproachPlan
{
  bool ok{false};
  std::vector<double> pre_joints;          // joint-space target for the pre-pose
  std::vector<double> grasp_joints;        // joint-space target for the grasp pose
  geometry_msgs::msg::Pose grasp_pose;     // chosen-yaw grasp pose (base_link)
  geometry_msgs::msg::Pose pre_pose;       // chosen-yaw pre-pose  (base_link)
  double yaw_rad{0.0};
  double descent_fraction{-1.0};
  double limit_clearance_rad{-1.0};
  std::string seed_label{"current"};
  // The straight pre->grasp Cartesian descent that was validated here at full
  // fraction. Replayed in the descent phase instead of recomputing the line from
  // the imperfect physical start (fragile near the wrist reconfiguration) or
  // routing an OMPL joint move through the physically-solid but planning-suspended
  // table (which stalls the arm high while the controller reports success).
  moveit_msgs::msg::RobotTrajectory descent_traj;
  bool descent_traj_ok{false};             // true only when the stored descent is full-fraction
};

class PickPlaceNode : public rclcpp::Node
{
public:
  using PickAndPlace = arm_interfaces::action::PickAndPlace;
  using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

  explicit PickPlaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void configure(const rclcpp::Node::SharedPtr & node_handle);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAndPlace::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
  void execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

  bool move_to_pose(
    const geometry_msgs::msg::Pose & target, double speed_scale = 1.0,
    bool strict_orientation = false);
  bool move_cartesian_line(
    const geometry_msgs::msg::Pose & target, double speed_scale = 1.0,
    bool allow_partial = false, bool allow_pose_fallback = true);
  bool move_to_named_target(const std::string & target, double speed_scale = 1.0);
  bool move_to_home_joint_target(double speed_scale = 1.0);
  bool move_to_joint_target(const std::vector<double> & joints, double speed_scale = 1.0);
  // Lower the gripper onto the grasp/drop pose, preferring the approach-validated
  // straight descent trajectory; falls back to the live Cartesian line, then the
  // verified joint config. Shared by the pick and place descent phases.
  bool descend_to_validated(
    const ApproachPlan & approach,
    const geometry_msgs::msg::Pose & grasp_pose, double speed_scale,
    double accepted_tolerance_m = 0.0);
  // Leave a grasp/drop pose by replaying the validated descent trajectory in
  // reverse. This avoids recomputing a fragile Cartesian retreat from the
  // physically settled state and avoids arbitrary OMPL fallback inside bins.
  bool retreat_to_validated(
    const ApproachPlan & approach,
    const geometry_msgs::msg::Pose & retreat_pose, double speed_scale);
  void set_clamped_start_state();
  // Select a grasp orientation whose IK and the lifted pre-grasp IK lie on the
  // same wrist branch, so the pre-grasp -> grasp descent is a clean straight
  // line. Returns ok=false if no yaw candidate yields a consistent pair.
  ApproachPlan plan_consistent_approach(
    const geometry_msgs::msg::Pose & grasp_target,
    double offset_z,
    bool place_branch_search = false) const;
  bool set_dl_joint_target(const geometry_msgs::msg::Pose & target);
  bool execute_plan(moveit::planning_interface::MoveGroupInterface::Plan & plan);
  bool execute_trajectory(moveit_msgs::msg::RobotTrajectory & trajectory, double speed_scale);
  void apply_motion_scaling(double requested_scale);
  bool time_parameterize_trajectory(
    moveit_msgs::msg::RobotTrajectory & trajectory,
    double speed_scale);
  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints_to(
    const geometry_msgs::msg::Pose & target) const;
  bool execute_grasp(bool close);
  geometry_msgs::msg::Pose compute_pre_pose(
    const geometry_msgs::msg::Pose & target,
    double offset_z = 0.10) const;
  bool validate_actual_grasp_pose(const geometry_msgs::msg::Pose & target_link6_pose) const;
  bool validate_actual_link6_pose(
    const geometry_msgs::msg::Pose & target_link6_pose,
    const char * label,
    double tolerance_m = -1.0) const;
  bool validate_home_state() const;

  // --- PlanningScene (collision-aware routing) ---
  void detected_objects_cb(const arm_interfaces::msg::ObjectArray::SharedPtr msg);
  // Look up the latest detected ObjectPose by id; returns false if unknown.
  bool lookup_detected_object(
    const std::string & object_id, arm_interfaces::msg::ObjectPose & out) const;
  // Populate the scene before a pick: the static work_table plus every detected
  // object EXCEPT the one being grasped (so the approach/descent is not blocked
  // by the target while still avoiding the table and neighbouring objects).
  void setup_planning_scene(const std::string & target_id);
  // On grasp: add the target as a collision box at its detected pose and attach
  // it to the gripper so transport plans stay collision-aware of the load.
  void attach_grasped_object(const std::string & object_id);
  // On release: detach the object from the gripper inside MoveIt's planning scene.
  void release_object(const std::string & object_id);
  void clear_managed_collision_objects();
  // Temporarily drop the world obstacles (table + neighbours) so the gripper can
  // descend onto an object resting on the table without CheckStartStateCollision
  // rejecting the grasp; restore them for the collision-aware transit moves.
  void suspend_scene_collision();
  void restore_scene_collision();
  void publish_feedback(
    const std::shared_ptr<GoalHandlePickAndPlace> & goal_handle,
    Phase phase,
    float progress) const;
  static const char * phase_name(Phase phase);

  rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;
  rclcpp::Client<arm_interfaces::srv::SolveIk>::SharedPtr dl_ik_client_;
  rclcpp::Subscription<arm_interfaces::msg::ObjectArray>::SharedPtr detected_objects_sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::unique_ptr<GripperController> gripper_;
  std::atomic_bool configured_{false};
  std::atomic_bool active_goal_{false};
  std::atomic_bool cancel_requested_{false};
  std::string planning_group_;
  std::string ik_solver_;
  std::string dl_ik_service_name_;
  std::chrono::seconds motion_timeout_;
  double cartesian_eef_step_m_{0.01};
  double cartesian_min_fraction_{0.98};
  double cartesian_partial_min_{0.85};   // execute partial line for departure moves
  double max_pose_segment_m_{0.12};
  double motion_velocity_scale_cap_{0.30};
  double motion_acceleration_scale_cap_{0.20};
  double gripper_settle_seconds_{1.0};
  int max_pose_segments_{6};
  // IK-consistent approach tuning.
  double approach_ik_timeout_s_{0.1};
  double approach_branch_max_delta_{1.0};  // rad; reject pre/grasp branch flips
  bool grasp_pose_guard_enabled_{true};
  double grasp_pose_guard_tolerance_m_{0.03};
  double place_release_guard_tolerance_m_{0.08};
  bool home_state_guard_enabled_{true};
  double home_state_guard_tolerance_rad_{0.30};

  // --- PlanningScene state ---
  mutable std::mutex detected_mutex_;
  std::unordered_map<std::string, arm_interfaces::msg::ObjectPose> detected_objects_;
  std::vector<std::string> managed_collision_ids_;   // world objects we added
  std::vector<moveit_msgs::msg::CollisionObject> managed_objects_;  // for restore after suspend
  bool scene_suspended_{false};
  std::string attached_object_id_;                   // currently attached, "" if none
  bool use_planning_scene_{true};
  std::vector<double> table_pose_xyz_;               // world frame
  std::vector<double> table_size_xyz_;
  // Sorting bins as static obstacles, flat groups of 6 in table_frame:
  // [cx, cy, cz, sx, sy, sz] per bin. Added to the scene so transit/pre-place
  // plans avoid neighbouring bins; suspended (like the table) for the descent.
  std::vector<double> bin_obstacles_;
  std::vector<double> bin_inner_size_xy_;
  std::string table_frame_;
  std::vector<double> default_object_size_xyz_;
  std::vector<std::string> grasp_touch_links_;       // gripper links allowed to touch load
  std::string attach_link_;                          // link the object attaches to
  double attach_box_shrink_{0.7};                    // shrink attached box to clear table/fingers
  double detected_obstacle_max_z_{0.15};             // ignore stale detections on the carried object/arm
};

}  // namespace arm_nodes
