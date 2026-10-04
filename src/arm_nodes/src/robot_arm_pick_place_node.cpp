#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "arm_interfaces/action/pick_and_place.hpp"
#include "arm_nodes/moveit_compat.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace arm_nodes
{
using namespace std::chrono_literals;
using PickAndPlace = arm_interfaces::action::PickAndPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickAndPlace>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class RobotArmPickPlaceNode : public rclcpp::Node
{
public:
  RobotArmPickPlaceNode()
  : Node("robot_arm_pick_place_node")
  {
    pre_pick_offset_ = declare_parameter("pre_pick_offset_m", 0.07);
    pre_place_offset_ = declare_parameter("pre_place_offset_m", 0.06);
    retreat_offset_ = declare_parameter("retreat_offset_m", 0.10);
    open_position_ = declare_parameter("gripper_open", 0.20);
    closed_position_ = declare_parameter("gripper_closed", -0.12);
    // 5-DOF: tam-dik + sabit-yaw poz aşırı-kısıtlı (IK erişimi incecik bir
    // halkaya düşüyor — 2026-07-04 /compute_ik taraması). Oryantasyon
    // toleransı ±0.35 rad, constraint sampler'ın hafif eğik ama kavramaya
    // uygun dik-yakın çözümleri bulmasına izin verir; erişim halkası açılır.
    goal_position_tolerance_ = declare_parameter("goal_position_tolerance_m", 0.004);
    goal_orientation_tolerance_ = declare_parameter("goal_orientation_tolerance_rad", 0.35);
    // Park pozu: CAD sıfırı (home=0...0) kolu sarı kutunun üstüne katlıyor ve
    // tepeden kamerada kutuyu örtüyordu. Default q1=-pi/2 katlanmayı boş
    // sektöre (az≈-81°; kutular -122°/180°/+122°, nesne bölgesi 0±33°) taşır.
    park_joint_positions_ = declare_parameter(
      "park_joint_positions", std::vector<double>{-1.5708, 0.0, 0.0, 0.0, 0.0});
    // SİM'de gz-sim mimic desteklemediğinden ikinci parmak (joint_6_mirror)
    // ayrı sürülür: boş değilse gripper komutuna -q ile eklenir. Real (mimic)
    // ve mimic destekleyen ortamlar için boş bırakılır.
    gripper_mirror_joint_ = declare_parameter("gripper_mirror_joint", std::string(""));
  }

  ~RobotArmPickPlaceNode() override
  {
    shutdown();
  }

  void configure(const rclcpp::Node::SharedPtr & node)
  {
    move_group_ = std::make_unique<MoveGroupInterface>(node, "arm");
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setGoalPositionTolerance(goal_position_tolerance_);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group_->setMaxVelocityScalingFactor(0.15);
    move_group_->setMaxAccelerationScalingFactor(0.10);
    gripper_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/robot_arm_gripper_controller/follow_joint_trajectory");
    arm_controller_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/robot_arm_controller/follow_joint_trajectory");
    server_ = rclcpp_action::create_server<PickAndPlace>(
      node, "/pick_and_place",
      [this](const auto &, const auto &) {
        if (shutting_down_.load()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (busy_.exchange(true)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandle> handle) {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        if (shutting_down_.load()) {
          auto result = std::make_shared<PickAndPlace::Result>();
          result->success = false;
          result->message = "Robot Arm pick/place is shutting down";
          handle->abort(result);
          busy_.store(false);
          return;
        }
        if (execution_thread_.joinable()) {
          execution_thread_.join();
        }
        execution_thread_ = std::thread([this, handle]() {execute(handle);});
      });
    RCLCPP_INFO(get_logger(), "Robot Arm pick/place action ready on /pick_and_place");
  }

  void shutdown() noexcept
  {
    if (shutting_down_.exchange(true)) {
      return;
    }

    // Stop accepting work before joining the owned execution thread.  The
    // MoveGroupInterface was constructed with this node's SharedPtr, so it
    // must be reset explicitly to break that ownership cycle before process
    // teardown; otherwise its plugin objects survive until class-loader
    // shutdown and trigger the severe unload warning.
    server_.reset();
    if (busy_.load() && move_group_) {
      try {
        move_group_->stop();
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "MoveGroup stop during shutdown failed: %s", error.what());
      }
    }

    std::thread execution_thread;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      execution_thread = std::move(execution_thread_);
    }
    if (execution_thread.joinable()) {
      execution_thread.join();
    }

    arm_controller_.reset();
    gripper_.reset();
    move_group_.reset();
  }

private:
  static geometry_msgs::msg::Pose top_down_pose(
    const geometry_msgs::msg::Point & point, double yaw)
  {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation.x = std::cos(yaw * 0.5);
    pose.orientation.y = std::sin(yaw * 0.5);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  bool move_at_yaw(
    const geometry_msgs::msg::Point & point, double yaw, const std::string & phase)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(top_down_pose(point, yaw), "tool0");
    MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      move_group_->clearPoseTargets();
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s: executing top-down yaw %.1f deg",
      phase.c_str(), yaw * 180.0 / M_PI);
    const bool ok = move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_->clearPoseTargets();
    return ok;
  }

  std::optional<double> move_with_free_yaw(
    const geometry_msgs::msg::Point & point, const std::string & phase)
  {
    constexpr double pi = 3.14159265358979323846;
    for (const double yaw : {
        0.0, pi / 8.0, -pi / 8.0, pi / 4.0, -pi / 4.0,
        3.0 * pi / 8.0, -3.0 * pi / 8.0, pi / 2.0, -pi / 2.0,
        5.0 * pi / 8.0, -5.0 * pi / 8.0, 3.0 * pi / 4.0,
        -3.0 * pi / 4.0, 7.0 * pi / 8.0, -7.0 * pi / 8.0, pi}) {
      if (move_at_yaw(point, yaw, phase)) {
        return yaw;
      }
    }
    RCLCPP_ERROR(get_logger(), "%s: no reachable top-down yaw", phase.c_str());
    return std::nullopt;
  }

  bool move_position_only(
    const geometry_msgs::msg::Point & point, const std::string & phase)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setPositionTarget(point.x, point.y, point.z, "tool0");
    MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      move_group_->clearPoseTargets();
      RCLCPP_ERROR(get_logger(), "%s: position target is unreachable", phase.c_str());
      return false;
    }
    const bool ok = move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_->clearPoseTargets();
    return ok;
  }

  bool command_gripper(double position)
  {
    if (!gripper_->wait_for_action_server(30s)) {
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"joint_6"};
    auto & point = goal.trajectory.points.emplace_back();
    point.positions = {position};
    if (!gripper_mirror_joint_.empty()) {
      goal.trajectory.joint_names.push_back(gripper_mirror_joint_);
      point.positions.push_back(-position);
    }
    point.time_from_start.sec = 2;
    auto sent = gripper_->async_send_goal(goal);
    if (sent.wait_for(30s) != std::future_status::ready) {
      return false;
    }
    const auto handle = sent.get();
    if (!handle) {
      return false;
    }
    auto result = gripper_->async_get_result(handle);
    return result.wait_for(60s) == std::future_status::ready &&
           result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool command_home()
  {
    if (!arm_controller_->wait_for_action_server(30s)) {
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
    auto & point = goal.trajectory.points.emplace_back();
    point.positions = park_joint_positions_;
    point.time_from_start.sec = 5;
    auto sent = arm_controller_->async_send_goal(goal);
    if (sent.wait_for(30s) != std::future_status::ready) {
      return false;
    }
    const auto handle = sent.get();
    if (!handle) {
      return false;
    }
    auto result = arm_controller_->async_get_result(handle);
    return result.wait_for(90s) == std::future_status::ready &&
           result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void feedback(
    const std::shared_ptr<GoalHandle> & handle, const std::string & phase, float progress)
  {
    auto message = std::make_shared<PickAndPlace::Feedback>();
    message->current_phase = phase;
    message->progress = progress;
    message->current_pose = move_group_->getCurrentPose("tool0").pose;
    handle->publish_feedback(message);
  }

  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto goal = handle->get_goal();
    bool ok = command_gripper(open_position_);

    feedback(handle, "moving_to_pre_pick", 0.10F);
    const auto pick_yaw = ok ? move_with_free_yaw(goal->pick_pose.position, "pick") : std::nullopt;
    ok = ok && pick_yaw.has_value();
    feedback(handle, "approaching_pick", 0.25F);
    feedback(handle, "grasping", 0.40F);
    ok = ok && command_gripper(closed_position_);

    // A 5-DOF arm cannot preserve an exact top-down orientation along an
    // arbitrary vertical Cartesian line. Returning through the validated home
    // joint state provides the physical lift/retreat without over-constraining IK.
    feedback(handle, "lifting", 0.52F);
    ok = ok && command_home();

    // Visual object transport is synchronized by autonomous_pick_node after a
    // successful action, exactly like the old simulation's post-place sync.
    feedback(handle, "moving_to_place", 0.68F);
    feedback(handle, "placing", 0.82F);
    ok = ok && command_gripper(open_position_);
    feedback(handle, "retreating", 0.92F);

    auto result = std::make_shared<PickAndPlace::Result>();
    result->success = ok;
    result->message = ok ? "Robot Arm pick/place completed" : "Robot Arm pick/place failed";
    result->execution_time = std::chrono::duration<float>(
      std::chrono::steady_clock::now() - started).count();
    if (handle->is_canceling()) {
      handle->canceled(result);
    } else if (ok) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
    busy_.store(false);
  }

  std::unique_ptr<MoveGroupInterface> move_group_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_controller_;
  rclcpp_action::Server<PickAndPlace>::SharedPtr server_;
  std::atomic_bool busy_{false};
  std::atomic_bool shutting_down_{false};
  std::mutex execution_mutex_;
  std::thread execution_thread_;
  double pre_pick_offset_;
  double pre_place_offset_;
  double retreat_offset_;
  double open_position_;
  double closed_position_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  std::vector<double> park_joint_positions_;
  std::string gripper_mirror_joint_;
};
}  // namespace arm_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_nodes::RobotArmPickPlaceNode>();
  node->configure(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node->shutdown();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
