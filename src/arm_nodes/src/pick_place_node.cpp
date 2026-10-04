#include "arm_nodes/pick_place_node.hpp"

#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <future>
#include <limits>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace arm_nodes
{
namespace
{
constexpr double kPrePickOffset = 0.10;
constexpr double kPrePlaceOffset = 0.10;
constexpr double kQuaternionEpsilon = 1e-9;

// Yaw-sweep fallback: if the requested wrist orientation is unreachable at a
// given Cartesian point, rotate the grasp about the base vertical axis and
// retry. This preserves the (roughly downward) grasp approach while giving
// MoveIt reachable wrist alternatives — far better than dropping orientation
// entirely, which leaves the gripper pointing in an arbitrary direction.
const std::vector<double> kYawSweepRadians = {
  0.0, 0.262, -0.262, 0.524, -0.524, 0.785, -0.785, 1.047, -1.047, 1.571, -1.571};

// Fixed Link_6 -> grasp_link translation (URDF grasp_joint origin xyz). The
// incoming pose target is a Link_6 pose whose grasp_link sits ON the object;
// the yaw-sweep must rotate about that grasp_link point (not Link_6 in place),
// or the ~0.1 m offset drags the gripper off the object.
constexpr double kLink6ToGraspXyz[3] = {0.0075, -0.0032, 0.1023};

// Rotate vector v by quaternion q (q = [x,y,z,w]).
std::array<double, 3> rotate_vector(const geometry_msgs::msg::Quaternion & q,
  const double v[3])
{
  // t = 2 * (q_xyz x v); v' = v + q_w * t + q_xyz x t
  const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
  const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
  const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
  return {
    v[0] + q.w * tx + (q.y * tz - q.z * ty),
    v[1] + q.w * ty + (q.z * tx - q.x * tz),
    v[2] + q.w * tz + (q.x * ty - q.y * tx)};
}

double pose_distance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = b.position.x - a.position.x;
  const double dy = b.position.y - a.position.y;
  const double dz = b.position.z - a.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::msg::Quaternion normalized_quaternion(
  const geometry_msgs::msg::Quaternion & q)
{
  auto out = q;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < kQuaternionEpsilon) {
    out.x = 0.0;
    out.y = 0.0;
    out.z = 0.0;
    out.w = 1.0;
    return out;
  }
  out.x /= norm;
  out.y /= norm;
  out.z /= norm;
  out.w /= norm;
  return out;
}

geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & target,
  double t)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = start.position.x + (target.position.x - start.position.x) * t;
  pose.position.y = start.position.y + (target.position.y - start.position.y) * t;
  pose.position.z = start.position.z + (target.position.z - start.position.z) * t;

  auto q1 = target.orientation;
  const auto & q0 = start.orientation;
  const double dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;
  if (dot < 0.0) {
    q1.x = -q1.x;
    q1.y = -q1.y;
    q1.z = -q1.z;
    q1.w = -q1.w;
  }
  pose.orientation.x = q0.x + (q1.x - q0.x) * t;
  pose.orientation.y = q0.y + (q1.y - q0.y) * t;
  pose.orientation.z = q0.z + (q1.z - q0.z) * t;
  pose.orientation.w = q0.w + (q1.w - q0.w) * t;
  pose.orientation = normalized_quaternion(pose.orientation);
  return pose;
}

// Rotate the grasp by `yaw` about the base Z axis THROUGH the grasp_link point,
// keeping grasp_link fixed on the object. Returns the corresponding Link_6 pose.
geometry_msgs::msg::Pose yaw_about_grasp_point(const geometry_msgs::msg::Pose & pose, double yaw)
{
  const auto & q = pose.orientation;
  // grasp_link world position = Link_6 position + R(q) * t
  const auto offset0 = rotate_vector(q, kLink6ToGraspXyz);
  const double gx = pose.position.x + offset0[0];
  const double gy = pose.position.y + offset0[1];
  const double gz = pose.position.z + offset0[2];

  // q' = qz(yaw) * q
  const double half = yaw * 0.5;
  const double cz = std::cos(half);
  const double sz = std::sin(half);
  geometry_msgs::msg::Pose out = pose;
  out.orientation.w = cz * q.w - sz * q.z;
  out.orientation.x = cz * q.x - sz * q.y;
  out.orientation.y = cz * q.y + sz * q.x;
  out.orientation.z = cz * q.z + sz * q.w;

  // Recompute Link_6 position so grasp_link stays at (gx, gy, gz).
  const auto offset1 = rotate_vector(out.orientation, kLink6ToGraspXyz);
  out.position.x = gx - offset1[0];
  out.position.y = gy - offset1[1];
  out.position.z = gz - offset1[2];
  return out;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d e = Eigen::Isometry3d::Identity();
  e.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  const Eigen::Quaterniond q(
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  e.linear() = q.normalized().toRotationMatrix();
  return e;
}

// Build an ADD box collision object in `frame` at `pose` with `size` (x,y,z).
moveit_msgs::msg::CollisionObject make_box(
  const std::string & id, const std::string & frame,
  const geometry_msgs::msg::Pose & pose, const std::vector<double> & size)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
    size.size() > 0 ? size[0] : 0.05,
    size.size() > 1 ? size[1] : 0.05,
    size.size() > 2 ? size[2] : 0.05};
  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

double joint_limit_clearance(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * jmg,
  const std::vector<double> & joints)
{
  if (!jmg) {
    return -1.0;
  }
  const auto names = jmg->getVariableNames();
  double clearance = std::numeric_limits<double>::infinity();
  const auto model = state.getRobotModel();
  for (std::size_t i = 0; i < joints.size() && i < names.size(); ++i) {
    const auto & bounds = model->getVariableBounds(names[i]);
    if (!bounds.position_bounded_) {
      continue;
    }
    clearance = std::min(
      clearance,
      std::min(joints[i] - bounds.min_position_, bounds.max_position_ - joints[i]));
  }
  if (!std::isfinite(clearance)) {
    return 0.0;
  }
  return clearance;
}
}  // namespace

PickPlaceNode::PickPlaceNode(const rclcpp::NodeOptions & options)
: Node("pick_place_node", options), motion_timeout_(std::chrono::seconds(30))
{
  planning_group_ = declare_parameter<std::string>("planning_group", "arm");
  ik_solver_ = declare_parameter<std::string>("ik_solver", "moveit");
  dl_ik_service_name_ = declare_parameter<std::string>("dl_ik_service_name", "/dl_ik_solve");
  // Wall-clock budget for a single plan/execute. Real hardware finishes a
  // motion well under this; the generous default tolerates a slow-RTF sim
  // (headless Gazebo + RGB-D point cloud can drop to ~0.1x real time, where a
  // few-second sim trajectory takes tens of wall-seconds to play out).
  const auto timeout_seconds = declare_parameter<int>("motion_timeout_seconds", 300);
  motion_timeout_ = std::chrono::seconds(timeout_seconds);
  cartesian_eef_step_m_ = declare_parameter<double>("cartesian_eef_step_m", 0.01);
  cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.98);
  cartesian_partial_min_ = declare_parameter<double>("cartesian_partial_min", 0.85);
  max_pose_segment_m_ = declare_parameter<double>("max_pose_segment_m", 0.12);
  // Keep MoveIt timing comfortably inside gz_ros2_control's per-cycle command
  // limiter. Higher scales made the controller clamp interpolated commands,
  // leaving the physical robot behind the planned start state.
  motion_velocity_scale_cap_ = declare_parameter<double>("motion_velocity_scale_cap", 0.30);
  motion_acceleration_scale_cap_ = declare_parameter<double>("motion_acceleration_scale_cap", 0.20);
  gripper_settle_seconds_ = declare_parameter<double>("gripper_settle_seconds", 1.0);
  max_pose_segments_ = declare_parameter<int>("max_pose_segments", 6);
  approach_ik_timeout_s_ = declare_parameter<double>("approach_ik_timeout_s", 0.1);
  approach_branch_max_delta_ =
    declare_parameter<double>("approach_branch_max_delta", 1.0);
  grasp_pose_guard_enabled_ = declare_parameter<bool>("grasp_pose_guard_enabled", true);
  grasp_pose_guard_tolerance_m_ =
    declare_parameter<double>("grasp_pose_guard_tolerance_m", 0.03);
  // Place-only tolerance: if a far-bin descent physically stalls slightly above
  // the requested drop pose, accepting a controlled high release is better than
  // failing and dropping the object far from the bin. Pick/grasp remains on the
  // strict grasp_pose_guard_tolerance_m_.
  place_release_guard_tolerance_m_ =
    declare_parameter<double>("place_release_guard_tolerance_m", 0.08);
  home_state_guard_enabled_ = declare_parameter<bool>("home_state_guard_enabled", true);
  // 0.30 (was 0.05): the 0.05 tolerance was over-tight and failed an otherwise
  // good return-home over a few-cm joint residual. 0.30 rad still catches the real
  // "arm stuck in the bin" failure (which shows as 2-3 rad on joint_1/joint_3)
  // while accepting a clean home that settles slightly off zero.
  home_state_guard_tolerance_rad_ =
    declare_parameter<double>("home_state_guard_tolerance_rad", 0.30);

  // PlanningScene (collision-aware routing). Table defaults match
  // arm_gazebo/worlds/sorting.world work_table (world frame).
  use_planning_scene_ = declare_parameter<bool>("use_planning_scene", true);
  table_frame_ = declare_parameter<std::string>("table_frame", "world");
  // Top deliberately ~2 cm below the true 0.6 m surface (center 0.29, height
  // 0.58 -> top 0.58): the box exists to stop plans routing THROUGH the table,
  // but must not block the gripper fingers from reaching an object resting on
  // the surface during the final descent.
  table_pose_xyz_ = declare_parameter<std::vector<double>>(
    "table_pose_xyz", std::vector<double>{0.675, 0.0, 0.29});
  table_size_xyz_ = declare_parameter<std::vector<double>>(
    "table_size_xyz", std::vector<double>{0.55, 1.10, 0.58});
  default_object_size_xyz_ = declare_parameter<std::vector<double>>(
    "default_object_size_xyz", std::vector<double>{0.05, 0.05, 0.05});
  // Sorting bins as static collision obstacles (flat groups of 6 in table_frame:
  // cx, cy, cz, sx, sy, sz). Defaults match arm_perception/config/sorting_bins.yaml
  // (outer footprint 0.184 m, wall height ~0.095 m, sitting on the 0.6 m table).
  // Without these the planning scene only had the table (+0 obstacles), so the arm
  // swung the carried object straight through neighbouring bins on the way to the
  // drop. Suspended together with the table during the descent/retreat so the
  // gripper can still enter and leave the target bin.
  bin_obstacles_ = declare_parameter<std::vector<double>>(
    "bin_obstacles_xyzwhd", std::vector<double>{
      0.492, -0.458, 0.6475, 0.184, 0.184, 0.095,
      0.492, 0.458, 0.6475, 0.184, 0.184, 0.095,
      0.858, 0.000, 0.6475, 0.184, 0.184, 0.095,
    });
  bin_inner_size_xy_ = declare_parameter<std::vector<double>>(
    "bin_inner_size_xy", std::vector<double>{0.16, 0.16});
  attach_link_ = declare_parameter<std::string>("attach_link", "grasp_link");
  attach_box_shrink_ = declare_parameter<double>("attach_box_shrink", 0.7);
  detected_obstacle_max_z_ = declare_parameter<double>("detected_obstacle_max_z", 0.15);
  grasp_touch_links_ = declare_parameter<std::vector<std::string>>(
    "grasp_touch_links",
    std::vector<std::string>{"Gripper_link1", "Gripper_link2", "grasp_link", "Link_6", "tool_link"});
}

void PickPlaceNode::configure(const rclcpp::Node::SharedPtr & node_handle)
{
  bool expected = false;
  if (!configured_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "Ignoring duplicate PickPlaceNode::configure() call");
    return;
  }
  action_server_ = rclcpp_action::create_server<PickAndPlace>(
    node_handle,
    "/pick_and_place",
    std::bind(&PickPlaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PickPlaceNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&PickPlaceNode::handle_accepted, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Pick/place action server ready on /pick_and_place");

  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_handle, planning_group_);
  // Pick/place poses (from perception and the bin config) are expressed in
  // base_link. MoveIt's planning frame is the URDF root 'world' (base_link sits
  // 0.6 m above it via the fixed camera mount added in Faz 4). Without this,
  // unframed pose targets are interpreted in 'world' and the arm reaches 0.6 m
  // off — motion "succeeds" but the gripper never lands on the object.
  move_group_->setPoseReferenceFrame("base_link");
  move_group_->setNumPlanningAttempts(8);
  move_group_->setPlanningTime(3.0);
  // KÖK FIX (2026-06-12): goal-tolerance'ı pick_ik'in çözüm eşiğiyle hizala.
  // MoveGroupInterface default goal-pos toleransı 1e-4 m, ama pick_ik (kinematics
  // .yaml) yalnız 1e-3 m'ye çözer. pick_ik bir çözüm döndürür ama MoveIt'in goal
  // constraint'i (1e-4 m) onu reddeder → OMPL "Unable to sample any valid states
  // for goal tree" → her pick segment 1/6'da çöker, kol hiç hareket etmezdi.
  // 2 mm / ~1.1°: pick_ik'in eriştiği 1e-3 m/rad'ın üstünde rahat marj; kavrama
  // için fazlasıyla hassas. compute_ik bu sıkı kontrolü yapmadığı için
  // "reachable" görünüyordu — gerçek sorun buydu.
  move_group_->setGoalPositionTolerance(0.002);
  move_group_->setGoalOrientationTolerance(0.02);
  gripper_ = std::make_unique<GripperController>(node_handle);
  dl_ik_client_ = node_handle->create_client<arm_interfaces::srv::SolveIk>(dl_ik_service_name_);

  planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  detected_objects_sub_ = node_handle->create_subscription<arm_interfaces::msg::ObjectArray>(
    "/detected_objects", rclcpp::SensorDataQoS(),
    std::bind(&PickPlaceNode::detected_objects_cb, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse PickPlaceNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickAndPlace::Goal> goal)
{
  (void)uuid;
  bool expected = false;
  if (!active_goal_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(), "Rejecting pick/place goal for %s: another goal is active", goal->object_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepted pick/place goal for object '%s'", goal->object_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickPlaceNode::handle_cancel(
  const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  (void)goal_handle;
  cancel_requested_.store(true);
  RCLCPP_WARN(get_logger(), "Cancel requested for pick/place goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickPlaceNode::handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  std::thread{std::bind(&PickPlaceNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PickPlaceNode::execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  cancel_requested_.store(false);
  const auto started = now();
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PickAndPlace::Result>();

  auto fail = [&](const std::string & message) {
      if (move_group_) {
        move_group_->stop();
        move_group_->clearPoseTargets();
      }
      if (gripper_) {
        RCLCPP_WARN(get_logger(), "Opening gripper after pick/place failure");
        (void)gripper_->open(std::chrono::seconds(5));
      }
      // Leave the planning scene clean so the next goal starts from a known
      // state (detaches any held object and removes the boxes we added).
      clear_managed_collision_objects();
      result->success = false;
      result->message = message;
      result->execution_time = static_cast<float>((now() - started).seconds());
      goal_handle->abort(result);
      active_goal_.store(false);
    };

  auto cancel = [&]() {
      publish_feedback(goal_handle, Phase::CANCELED, 0.0f);
      result->success = false;
      result->message = "Pick/place goal canceled";
      result->execution_time = static_cast<float>((now() - started).seconds());
      goal_handle->canceled(result);
      active_goal_.store(false);
    };

  auto step = [&](Phase phase, float progress, const auto & fn) -> bool {
      if (cancel_requested_.load() || goal_handle->is_canceling()) {
        cancel();
        return false;
      }
      publish_feedback(goal_handle, phase, progress);
      if (!fn()) {
        fail(std::string("Failed during phase: ") + phase_name(phase));
        return false;
      }
      return true;
    };

  const auto pre_pick = compute_pre_pose(goal->pick_pose, kPrePickOffset);
  const auto pre_place = compute_pre_pose(goal->place_pose, kPrePlaceOffset);

  publish_feedback(goal_handle, Phase::DETECT_OBJECT, 0.02f);
  publish_feedback(goal_handle, Phase::PLAN_PRE_GRASP, 0.04f);

  if (!step(Phase::HOME_START, 0.06f, [&] {return move_to_named_target("home", 0.6);} )) {return;}
  // Open the gripper before approaching so it is open during the approach (no
  // premature grasp).
  // Best-effort: a transient gripper-action hiccup (seen under heavy/long slow-
  // RTF runs) must NOT abort the whole pick and spin in a tight retry loop.
  if (cancel_requested_.load() || goal_handle->is_canceling()) {cancel(); return;}
  publish_feedback(goal_handle, Phase::OPEN_GRIPPER, 0.08f);
  if (!execute_grasp(false)) {
    RCLCPP_WARN(get_logger(), "Initial gripper open failed; continuing (best-effort)");
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(gripper_settle_seconds_));

  // Step 5: populate the PlanningScene (work table + neighbouring objects) so
  // every plan from here is collision-aware. The target object is excluded and
  // attached at grasp instead, so it never blocks the descent.
  setup_planning_scene(goal->object_id);

  // PICK approach: derive the pre-grasp from the grasp's IK branch so the
  // descent is a clean straight line in one configuration (the user's
  // requirement: hover 10-15 cm above, then approach with that same solution).
  // Fall back to the free pose approach when no consistent branch exists.
  const auto pick_approach = plan_consistent_approach(goal->pick_pose, kPrePickOffset);
  const auto pick_grasp_pose = pick_approach.ok ? pick_approach.grasp_pose : goal->pick_pose;
  const auto pick_pre_pose = pick_approach.ok ? pick_approach.pre_pose : pre_pick;

  // Suspend the table/neighbour obstacles for the WHOLE grasp approach (not just
  // the descent): with them active, the joint move to the IK-consistent pre-grasp
  // config gets CheckStartStateCollision-rejected and falls back to an OMPL pose
  // plan on a DIFFERENT wrist branch -> the straight descent then can't hold
  // orientation. Suspending here lets the pre-grasp reach the grasp's own branch
  // so the descent is a clean vertical line onto the object. Restored after lift.
  suspend_scene_collision();

  // Move to the pre-grasp. Prefer the IK-consistent joint config (same branch as
  // the grasp); fall back to an OMPL pose plan only if that is unavailable.
  if (!step(Phase::MOVE_TO_PRE_GRASP, 0.16f, [&] {
        if (pick_approach.ok && move_to_joint_target(pick_approach.pre_joints, 0.60)) {
          return true;
        }
        return move_to_pose(pick_pre_pose, 0.60);
      })) {return;}
  // Straight vertical descent onto the object. Replay the approach-validated line
  // first (see descend_to_validated): recomputing the Cartesian here from the
  // imperfect physical start is fragile near the wrist reconfiguration, and the
  // OMPL joint fallback routes through the physically-solid (but planning-suspended)
  // table -> the arm stalls ~7 cm high while the controller reports success.
  if (!step(Phase::DESCEND_TO_GRASP, 0.30f, [&] {
        return descend_to_validated(pick_approach, pick_grasp_pose, 0.35);
      })) {return;}

  if (cancel_requested_.load() || goal_handle->is_canceling()) {cancel(); return;}
  publish_feedback(goal_handle, Phase::CLOSE_GRIPPER, 0.42f);
  if (!validate_actual_grasp_pose(pick_grasp_pose)) {
    fail("actual grasp pose mismatch before close gripper");
    return;
  }
  if (!execute_grasp(true)) {
    fail(std::string("Failed during phase: ") + phase_name(Phase::CLOSE_GRIPPER));
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(gripper_settle_seconds_));

  // Step 11: attach the grasped object so the carry/lift/place plans stay
  // collision-aware of the load (it moves with the gripper in the scene).
  attach_grasped_object(goal->object_id);

  if (!step(Phase::LIFT_OBJECT, 0.55f,
    [&] {return retreat_to_validated(pick_approach, pick_pre_pose, 0.30);} )) {return;}

  // Object lifted clear of the table: restore obstacles for the collision-aware
  // transit to the bin.
  restore_scene_collision();

  // PLACE approach: same IK-consistent strategy for the drop.
  const auto place_approach =
    plan_consistent_approach(goal->place_pose, kPrePlaceOffset, /*place_branch_search=*/true);
  const auto place_drop_pose = place_approach.ok ? place_approach.grasp_pose : goal->place_pose;
  const auto place_pre_pose = place_approach.ok ? place_approach.pre_pose : pre_place;

  if (!step(Phase::MOVING_TO_PRE_PLACE, 0.70f, [&] {
        if (place_approach.ok && move_to_joint_target(place_approach.pre_joints, 0.55)) {
          return true;
        }
        return move_to_pose(place_pre_pose, 0.55);
      })) {return;}
  // Suspend obstacles again for the place descent (gripper approaches the bin).
  // Same descent strategy as the pick: replay the approach-validated straight line
  // (descend_to_validated) so the object is lowered to the actual bin height
  // instead of being deposited high (the old OMPL fallback left it at e.g. z=1.058).
  suspend_scene_collision();
  if (!step(Phase::DESCEND_TO_PLACE, 0.82f, [&] {
        return descend_to_validated(
          place_approach, place_drop_pose, 0.30, place_release_guard_tolerance_m_);
      })) {return;}
  if (!step(Phase::OPEN_GRIPPER, 0.90f, [&] {return execute_grasp(false);} )) {return;}
  std::this_thread::sleep_for(std::chrono::duration<double>(gripper_settle_seconds_));

  // Step 16: detach the object from the gripper now that it is released.
  release_object(goal->object_id);

  // Escape the bin by replaying the approach-validated descent in reverse. The
  // previous live Cartesian recompute got low fractions (~0.13 in red bin) and
  // then hid the unsafe branch with a free direct-plan fallback, which reported
  // success while the physical arm remained stuck in/near the bin.
  if (!step(Phase::RETREAT, 0.96f,
    [&] {return retreat_to_validated(place_approach, place_pre_pose, 0.25);} )) {return;}

  // The arm has escaped the bin mouth; restore obstacles before going home so
  // the return path is collision-aware again.
  restore_scene_collision();

  RCLCPP_INFO(get_logger(), "Returning arm to explicit home joint target");
  if (!step(Phase::HOME_RETURN, 0.99f, [&] {return move_to_home_joint_target(0.30);} )) {return;}

  if (!validate_home_state()) {
    fail("home state mismatch after HOME_RETURN");
    return;
  }

  // Clean the boxes we added so the next goal plans from a known scene.
  clear_managed_collision_objects();

  publish_feedback(goal_handle, Phase::DONE, 1.0f);
  result->success = true;
  result->message = "Pick/place completed";
  result->execution_time = static_cast<float>((now() - started).seconds());
  goal_handle->succeed(result);
  active_goal_.store(false);
}

bool PickPlaceNode::move_to_pose(
  const geometry_msgs::msg::Pose & target, double speed_scale, bool strict_orientation)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }

  apply_motion_scaling(speed_scale);
  // KÖK FIX (2026-06-12): serbest geçişi doğrudan hedefe planla, pose-interpolasyonlu
  // ara waypoint'lere BÖLME. cartesian_waypoints_to ara pozlar üretiyordu; ara
  // YÖNELİMLER (home bileği ↔ grasp pozu arası slerp) iki uç nokta erişilebilir olsa
  // bile ERİŞİLEMEZ bilek konfigürasyonlarına düşüyor → segment 1/N OMPL'de "Unable to
  // sample any valid states for goal tree" ile çöküyor ve kol HİÇ hareket etmiyordu.
  // Doğrudan planlamada OMPL joint-space yolu buluyor (kanıt: /plan_kinematic_path
  // pre-pick pozuna başarıyla planlıyor). Düz-çizgi Cartesian iniş ayrı fonksiyonda
  // (move_cartesian_line) computeCartesianPath kullanır; o etkilenmez.
  auto waypoints = std::vector<geometry_msgs::msg::Pose>{target};
  bool using_segmented_waypoints = false;
  RCLCPP_INFO(get_logger(), "Planning directly to target (no pose interpolation)");

  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const auto & waypoint = waypoints[index];
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto plan_current = [&]() -> bool {
        auto plan_future = std::async(std::launch::async, [&] {return move_group_->plan(plan);});
        if (plan_future.wait_for(motion_timeout_) != std::future_status::ready) {
          move_group_->stop();
          RCLCPP_ERROR(get_logger(), "Planning timed out");
          return false;
        }
        return plan_future.get() == moveit::core::MoveItErrorCode::SUCCESS;
      };

    bool planned = false;

    // 1) Optional DL IK joint target (when ik_solver:=dl). MoveIt still checks
    // joint limits and collision validity during planning.
    if (ik_solver_ == "dl") {
      move_group_->clearPoseTargets();
      move_group_->setStartStateToCurrentState();
      if (set_dl_joint_target(waypoint)) {
        planned = plan_current();
      }
      if (!planned) {
        RCLCPP_WARN(get_logger(), "DL IK path failed; falling back to pose target");
      }
    }

    // 2) Full 6-DOF pose target: requested orientation first, then yaw-swept
    //    variants. The first successful planned trajectory is the selected IK
    //    branch for this intermediate target.
    if (!planned) {
      for (double yaw : kYawSweepRadians) {
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(yaw_about_grasp_point(waypoint, yaw), "Link_6");
        if (plan_current()) {
          if (yaw != 0.0) {
            RCLCPP_INFO(
              get_logger(), "Reached pose segment %zu/%zu via yaw-sweep %.0f deg",
              index + 1, waypoints.size(), yaw * 180.0 / M_PI);
          }
          planned = true;
          break;
        }
      }
    }

    // 3) Last resort: position-only target (free wrist) so motion still makes
    //    progress even when no oriented grasp is reachable. SKIPPED when
    //    strict_orientation: for a grasp descent a free wrist puts Link_6 at the
    //    position but swings grasp_link (~10 cm offset) off the object -> the
    //    object is grasped off-centre. Better to fail than grasp wrong.
    if (!planned && !strict_orientation) {
      move_group_->clearPoseTargets();
      move_group_->setStartStateToCurrentState();
      move_group_->setPositionTarget(
        waypoint.position.x, waypoint.position.y, waypoint.position.z, "Link_6");
      RCLCPP_WARN(
        get_logger(), "Oriented planning failed for segment %zu/%zu; using position-only target",
        index + 1, waypoints.size());
      planned = plan_current();
    }

    if (!planned) {
      move_group_->clearPoseTargets();
      if (using_segmented_waypoints) {
        RCLCPP_WARN(
          get_logger(),
          "Joint-space segment %zu/%zu failed; retrying direct final target",
          index + 1, waypoints.size());
        waypoints.clear();
        waypoints.push_back(target);
        using_segmented_waypoints = false;
        index = static_cast<std::size_t>(-1);
        continue;
      }
      RCLCPP_ERROR(get_logger(), "Joint-space planning failed for final target");
      return false;
    }

    if (!execute_plan(plan)) {
      return false;
    }
  }

  move_group_->clearPoseTargets();
  return true;
}

bool PickPlaceNode::move_cartesian_line(
  const geometry_msgs::msg::Pose & target,
  double speed_scale,
  bool allow_partial,
  bool allow_pose_fallback)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }

  apply_motion_scaling(speed_scale);

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::RobotTrajectory best_trajectory;
  double best_fraction = -1.0;
  int best_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  double selected_yaw = 0.0;
  bool planned_cartesian = false;

  for (double yaw : kYawSweepRadians) {
    move_group_->setStartStateToCurrentState();
    const auto candidate = yaw_about_grasp_point(target, yaw);
    const auto waypoints = cartesian_waypoints_to(candidate);
    moveit_msgs::msg::RobotTrajectory candidate_trajectory;
    moveit_msgs::msg::MoveItErrorCodes candidate_error;
    // avoid_collisions = FALSE: these are short, guarded approach/depart line
    // moves to/from a KNOWN grasp/place pose. Collision-checking the gripper as
    // it deliberately approaches the object and the table surface aborted the
    // descent almost immediately (fractions ~0.0-0.5) and forced a free-wrist
    // fallback that grasps ~6 cm off centre. The line only needs to stay
    // IK-feasible; joint limits are still enforced by computeCartesianPath.
    const double fraction = moveit_compat::compute_cartesian_path(
      *move_group_, waypoints, cartesian_eef_step_m_, candidate_trajectory, false,
      &candidate_error);

    if (fraction > best_fraction) {
      best_fraction = fraction;
      best_error_code = candidate_error.val;
      best_trajectory = candidate_trajectory;
    }
    if (fraction >= cartesian_min_fraction_) {
      trajectory = candidate_trajectory;
      selected_yaw = yaw;
      planned_cartesian = true;
      break;
    }
  }

  // Departure moves (lift / retreat) only need the load to clear the grasp/bin:
  // a high-but-incomplete straight line is fine. Execute the best partial path
  // instead of rejecting it and re-planning freely (which aborted before).
  if (!planned_cartesian && allow_partial && best_fraction >= cartesian_partial_min_) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian line only %.3f (< %.3f); executing partial path (departure move)",
      best_fraction, cartesian_min_fraction_);
    trajectory = best_trajectory;
    planned_cartesian = true;
  }

  if (!planned_cartesian) {
    if (!allow_pose_fallback) {
      // Descents pass allow_pose_fallback=false: the caller has a controlled
      // joint-space move to the verified grasp/drop config to fall back on. An
      // OMPL pose plan here would reach the pose via an arbitrary path/branch and
      // can leave the load deposited high/wrong — so we report failure instead.
      RCLCPP_WARN(
        get_logger(),
        "Cartesian line failed (best %.3f < %.3f); deferring to caller's joint fallback",
        best_fraction, cartesian_min_fraction_);
      return false;
    }
    RCLCPP_WARN(
      get_logger(),
      "Cartesian straight-line path failed after yaw-sweep: best fraction %.3f below %.3f "
      "(MoveIt code %d); falling back to oriented direct plan",
      best_fraction, cartesian_min_fraction_, best_error_code);
    // strict_orientation: never use a position-only (free wrist) fallback for a
    // guarded line move — it swings grasp_link off the object / drop point.
    return move_to_pose(target, speed_scale, /*strict_orientation=*/true);
  }

  if (selected_yaw != 0.0) {
    RCLCPP_INFO(
      get_logger(), "Cartesian path reached min fraction via yaw-sweep %.0f deg",
      selected_yaw * 180.0 / M_PI);
  }

  if (!time_parameterize_trajectory(trajectory, speed_scale)) {
    RCLCPP_ERROR(get_logger(), "Cartesian trajectory time-parameterization failed");
    return false;
  }

  return execute_trajectory(trajectory, speed_scale);
}

bool PickPlaceNode::move_to_named_target(const std::string & target, double speed_scale)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }

  apply_motion_scaling(speed_scale);
  set_clamped_start_state();
  if (!move_group_->setNamedTarget(target)) {
    RCLCPP_ERROR(get_logger(), "MoveIt named target is unavailable: %s", target.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_future = std::async(std::launch::async, [&] {return move_group_->plan(plan);});
  if (plan_future.wait_for(motion_timeout_) != std::future_status::ready) {
    move_group_->stop();
    RCLCPP_ERROR(get_logger(), "Planning named target '%s' timed out", target.c_str());
    return false;
  }

  if (plan_future.get() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Planning named target '%s' failed", target.c_str());
    return false;
  }

  return execute_plan(plan);
}

bool PickPlaceNode::move_to_home_joint_target(double speed_scale)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }
  auto state = move_group_->getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(get_logger(), "Unable to read current state for home joint target");
    return false;
  }
  const auto * jmg = state->getJointModelGroup(planning_group_);
  if (!jmg) {
    RCLCPP_ERROR(get_logger(), "Planning group '%s' not found", planning_group_.c_str());
    return false;
  }
  moveit::core::RobotState home_state(*state);
  if (!home_state.setToDefaultValues(jmg, "home")) {
    RCLCPP_ERROR(get_logger(), "MoveIt named target is unavailable: home");
    return false;
  }
  std::vector<double> home_joints;
  home_state.copyJointGroupPositions(jmg, home_joints);
  RCLCPP_INFO(get_logger(), "Returning arm to explicit home joint target");
  return move_to_joint_target(home_joints, speed_scale);
}

bool PickPlaceNode::move_to_joint_target(
  const std::vector<double> & joints, double speed_scale)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }
  if (joints.empty()) {
    RCLCPP_ERROR(get_logger(), "Refusing to plan to an empty joint target");
    return false;
  }

  apply_motion_scaling(speed_scale);
  set_clamped_start_state();
  move_group_->clearPoseTargets();
  if (!move_group_->setJointValueTarget(joints)) {
    RCLCPP_ERROR(get_logger(), "Joint target is out of bounds");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_future = std::async(std::launch::async, [&] {return move_group_->plan(plan);});
  if (plan_future.wait_for(motion_timeout_) != std::future_status::ready) {
    move_group_->stop();
    RCLCPP_ERROR(get_logger(), "Joint-target planning timed out");
    return false;
  }
  if (plan_future.get() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Joint-target planning failed");
    return false;
  }
  return execute_plan(plan);
}

bool PickPlaceNode::descend_to_validated(
  const ApproachPlan & approach,
  const geometry_msgs::msg::Pose & grasp_pose, double speed_scale,
  double accepted_tolerance_m)
{
  const double guard_tolerance =
    accepted_tolerance_m > 0.0 ?
    std::max(accepted_tolerance_m, grasp_pose_guard_tolerance_m_) :
    grasp_pose_guard_tolerance_m_;
  const bool relaxed_release = guard_tolerance > grasp_pose_guard_tolerance_m_ + 1e-9;

  // 1) Replay the straight descent that plan_consistent_approach validated at full
  // fraction: the exact pre->grasp line proven feasible from the IK pre-grasp
  // state. Executing it avoids the live-Cartesian fragility (recomputing from the
  // imperfect physical start drops to ~0.1-0.5 near the wrist reconfiguration) and
  // the table-piercing OMPL joint path.
  if (approach.descent_traj_ok &&
      !approach.descent_traj.joint_trajectory.points.empty())
  {
    moveit_msgs::msg::RobotTrajectory traj = approach.descent_traj;
    if (time_parameterize_trajectory(traj, speed_scale) &&
        execute_trajectory(traj, speed_scale))
    {
      RCLCPP_INFO(get_logger(), "Descent: replayed approach-validated straight line");
      if (validate_actual_link6_pose(grasp_pose, "Descent guard", guard_tolerance)) {
        if (relaxed_release) {
          RCLCPP_WARN(
            get_logger(),
            "Place descent accepted with relaxed release tolerance %.3fm", guard_tolerance);
        }
        return true;
      }
      RCLCPP_WARN(
        get_logger(),
        "Descent: controller reported success but TCP is not at target; trying live correction");
    }
    else {
      RCLCPP_WARN(get_logger(),
        "Descent: validated replay failed; trying live Cartesian line");
    }
  }

  // 2) Live straight Cartesian line (no OMPL pose fallback -> would swing the TCP
  // off-centre).
  if (move_cartesian_line(grasp_pose, speed_scale, /*allow_partial=*/false,
      /*allow_pose_fallback=*/false))
  {
    if (validate_actual_link6_pose(grasp_pose, "Descent correction guard", guard_tolerance)) {
      if (relaxed_release) {
        RCLCPP_WARN(
          get_logger(),
          "Place descent correction accepted with relaxed release tolerance %.3fm",
          guard_tolerance);
      }
      return true;
    }
    RCLCPP_WARN(
      get_logger(),
      "Descent: live Cartesian correction executed but TCP is still off target");
  }

  // 3) Last resort: controlled joint move to the verified grasp/drop config. This
  // can route an arbitrary path through the physically-solid (planning-suspended)
  // table and stall high, so the grasp guard remains the backstop that rejects a
  // short descent.
  if (approach.ok && !approach.grasp_joints.empty()) {
    RCLCPP_WARN(get_logger(),
      "Descent: Cartesian failed; controlled joint move to verified config");
    if (move_to_joint_target(approach.grasp_joints, 0.25) &&
        validate_actual_link6_pose(grasp_pose, "Descent joint correction guard", guard_tolerance))
    {
      if (relaxed_release) {
        RCLCPP_WARN(
          get_logger(),
          "Place descent joint correction accepted with relaxed release tolerance %.3fm",
          guard_tolerance);
      }
      return true;
    }
  }
  return false;
}

bool PickPlaceNode::retreat_to_validated(
  const ApproachPlan & approach,
  const geometry_msgs::msg::Pose & retreat_pose, double speed_scale)
{
  if (approach.descent_traj_ok &&
      !approach.descent_traj.joint_trajectory.points.empty())
  {
    moveit_msgs::msg::RobotTrajectory traj = approach.descent_traj;
    auto & points = traj.joint_trajectory.points;
    std::reverse(points.begin(), points.end());
    for (auto & point : points) {
      point.velocities.clear();
      point.accelerations.clear();
      point.effort.clear();
      point.time_from_start.sec = 0;
      point.time_from_start.nanosec = 0;
    }
    if (time_parameterize_trajectory(traj, speed_scale) &&
        execute_trajectory(traj, speed_scale))
    {
      RCLCPP_INFO(get_logger(), "Retreat: replayed validated descent in reverse");
      return true;
    }
    RCLCPP_WARN(
      get_logger(), "Retreat: reverse validated replay failed; trying full Cartesian line");
  }

  if (move_cartesian_line(
      retreat_pose, speed_scale, /*allow_partial=*/false, /*allow_pose_fallback=*/false))
  {
    return true;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Retreat rejected: unsafe drop/grasp branch; no direct-plan fallback inside bin");
  return false;
}

void PickPlaceNode::set_clamped_start_state()
{
  if (!move_group_) {
    return;
  }
  auto state = move_group_->getCurrentState(2.0);
  if (!state) {
    move_group_->setStartStateToCurrentState();
    return;
  }
  const auto * jmg = state->getJointModelGroup(planning_group_);
  if (!jmg) {
    move_group_->setStartStateToCurrentState();
    return;
  }

  std::vector<double> before;
  std::vector<double> after;
  state->copyJointGroupPositions(jmg, before);
  state->enforceBounds(jmg);
  state->copyJointGroupPositions(jmg, after);
  for (std::size_t i = 0; i < before.size() && i < after.size(); ++i) {
    if (std::abs(before[i] - after[i]) > 1e-9) {
      RCLCPP_WARN(
        get_logger(),
        "Clamped start state for planning: joint %zu %.12f -> %.12f",
        i + 1, before[i], after[i]);
    }
  }
  move_group_->setStartState(*state);
}

ApproachPlan PickPlaceNode::plan_consistent_approach(
  const geometry_msgs::msg::Pose & grasp_target, double offset_z, bool place_branch_search) const
{
  ApproachPlan result;
  if (!move_group_) {
    return result;
  }

  auto state = move_group_->getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(get_logger(), "Unable to read current state for approach planning");
    return result;
  }
  const auto * jmg = state->getJointModelGroup(planning_group_);
  if (!jmg) {
    RCLCPP_ERROR(get_logger(), "Planning group '%s' not found", planning_group_.c_str());
    return result;
  }

  // Pose targets are expressed in base_link; IK (setFromIK) needs them in the
  // model frame (world). Read the actual world<-base_link transform from the
  // live state instead of hard-coding the world_joint offset.
  bool base_found = false;
  const Eigen::Isometry3d t_world_base = state->getFrameTransform("base_link", &base_found);
  if (!base_found) {
    RCLCPP_ERROR(get_logger(), "base_link frame not found in robot state");
    return result;
  }

  struct SeedState
  {
    std::string label;
    moveit::core::RobotState state;
  };
  std::vector<SeedState> seed_states;
  seed_states.push_back({"current", *state});

  if (place_branch_search) {
    moveit::core::RobotState home_state(*state);
    if (home_state.setToDefaultValues(jmg, "home")) {
      seed_states.push_back({"home", home_state});
    }

    std::vector<double> current_joints;
    state->copyJointGroupPositions(jmg, current_joints);
    const auto names = jmg->getVariableNames();
    auto add_joint1_seed = [&](const std::string & label, double joint1) {
        if (current_joints.empty() || names.empty()) {
          return;
        }
        auto joints = current_joints;
        const auto & bounds = state->getRobotModel()->getVariableBounds(names.front());
        if (bounds.position_bounded_) {
          joint1 = std::clamp(joint1, bounds.min_position_, bounds.max_position_);
        }
        joints.front() = joint1;
        moveit::core::RobotState seeded(*state);
        seeded.setJointGroupPositions(jmg, joints);
        seeded.enforceBounds(jmg);
        seed_states.push_back({label, seeded});
      };

    // The red/yellow bins can sit on awkward folded branches. Probe both base
    // turn directions explicitly instead of trusting the current carry state to
    // seed IK into the only usable basin.
    add_joint1_seed("turn_positive", 2.4);
    add_joint1_seed("turn_negative", -2.4);
    add_joint1_seed("turn_center", 0.0);
  }

  double best_descent_frac = -1.0;
  double best_yaw = 0.0;
  double best_clean_clearance = -1.0;
  double best_clean_score = -std::numeric_limits<double>::infinity();
  bool has_clean_candidate = false;
  const auto store_candidate = [&](
      ApproachPlan & out,
      const std::vector<double> & q_pre,
      const std::vector<double> & q_grasp,
      const geometry_msgs::msg::Pose & pre_base,
      const geometry_msgs::msg::Pose & grasp_base,
      const moveit_msgs::msg::RobotTrajectory & descent_traj,
      const std::string & seed_label,
      double yaw,
      double descent_frac,
      double limit_clearance) {
      out.ok = true;
      out.pre_joints = q_pre;
      out.grasp_joints = q_grasp;
      out.grasp_pose = grasp_base;
      out.pre_pose = pre_base;
      out.yaw_rad = yaw;
      out.descent_fraction = descent_frac;
      out.limit_clearance_rad = limit_clearance;
      out.seed_label = seed_label;
      out.descent_traj = descent_traj;
      out.descent_traj_ok = (descent_frac >= cartesian_min_fraction_);
    };
  for (const auto & seed : seed_states) {
    for (double yaw : kYawSweepRadians) {
      const auto grasp_base = yaw_about_grasp_point(grasp_target, yaw);
      geometry_msgs::msg::Pose pre_base = grasp_base;
      pre_base.position.z += offset_z;

      const Eigen::Isometry3d grasp_world = t_world_base * pose_to_eigen(grasp_base);
      const Eigen::Isometry3d pre_world = t_world_base * pose_to_eigen(pre_base);

      // Grasp IK first, then the pre-grasp IK seeded FROM the grasp solution so
      // both land on the same wrist branch.
      moveit::core::RobotState grasp_state(seed.state);
      if (!grasp_state.setFromIK(jmg, grasp_world, "Link_6", approach_ik_timeout_s_)) {
        continue;
      }

      // Self-validate the base_link<->world frame handling: the IK solution's FK
      // must reproduce the requested grasp position. If a frame assumption were
      // wrong the error would be large here, so we reject and fall back rather
      // than execute a bad joint target.
      const Eigen::Isometry3d fk_base =
        t_world_base.inverse() * grasp_state.getGlobalLinkTransform("Link_6");
      const Eigen::Vector3d want(
        grasp_base.position.x, grasp_base.position.y, grasp_base.position.z);
      if ((fk_base.translation() - want).norm() > 0.005) {
        continue;
      }

      moveit::core::RobotState pre_state(grasp_state);
      if (!pre_state.setFromIK(jmg, pre_world, "Link_6", approach_ik_timeout_s_)) {
        continue;
      }

      std::vector<double> q_grasp;
      std::vector<double> q_pre;
      grasp_state.copyJointGroupPositions(jmg, q_grasp);
      pre_state.copyJointGroupPositions(jmg, q_pre);

      // A pure ~10 cm vertical translation on one branch produces a modest joint
      // change; a branch flip shows up as a large per-joint delta. Reject those
      // so the straight-line descent is guaranteed feasible.
      double max_delta = 0.0;
      for (std::size_t i = 0; i < q_grasp.size() && i < q_pre.size(); ++i) {
        max_delta = std::max(max_delta, std::abs(q_grasp[i] - q_pre[i]));
      }
      if (max_delta > approach_branch_max_delta_) {
        continue;
      }

      // Validate the ACTUAL straight vertical descent from this pre-grasp config:
      // plan the Cartesian line pre->grasp starting FROM pre_state. A same-branch
      // pair can still be non-descendable (near-singular -> fraction ~0). Pick the
      // yaw whose vertical descent is genuinely feasible so the gripper comes
      // straight down onto the object instead of arcing in.
      move_group_->setStartState(pre_state);
      std::vector<geometry_msgs::msg::Pose> wp{grasp_base};
      moveit_msgs::msg::RobotTrajectory descent_traj;
      moveit_msgs::msg::MoveItErrorCodes descent_err;
      const double descent_frac = moveit_compat::compute_cartesian_path(
        *move_group_, wp, cartesian_eef_step_m_, descent_traj, false,
        &descent_err);
      move_group_->setStartStateToCurrentState();
      const double limit_clearance = std::min(
        joint_limit_clearance(*state, jmg, q_grasp),
        joint_limit_clearance(*state, jmg, q_pre));

      // Prefer clean full-descent candidates. Pick uses the old conservative
      // limit-clearance ordering. Place also gives a small preference to lower
      // yaw displacement and non-current seeds so an awkward carry-state seed
      // does not monopolize the red-bin drop branch.
      if (descent_frac >= cartesian_min_fraction_) {
        double score = limit_clearance;
        if (place_branch_search) {
          score += 0.12 * std::cos(std::abs(yaw));
          if (seed.label != "current") {
            score += 0.08;
          }
        }
        if (!has_clean_candidate ||
            score > best_clean_score + 1e-6 ||
            (std::abs(score - best_clean_score) <= 1e-6 &&
             limit_clearance > best_clean_clearance + 1e-6) ||
            (std::abs(score - best_clean_score) <= 1e-6 &&
             std::abs(limit_clearance - best_clean_clearance) <= 1e-6 &&
             descent_frac > result.descent_fraction))
        {
          store_candidate(
            result, q_pre, q_grasp, pre_base, grasp_base, descent_traj,
            seed.label, yaw, descent_frac, limit_clearance);
          has_clean_candidate = true;
          best_clean_clearance = limit_clearance;
          best_clean_score = score;
        }
        continue;
      }

      // If no full-descent candidate exists, keep the best partial descent for
      // the caller's explicit fallback path. Do not let partial candidates replace
      // a clean, more escapable one.
      if (!has_clean_candidate && (!result.ok || descent_frac > best_descent_frac)) {
        store_candidate(
          result, q_pre, q_grasp, pre_base, grasp_base, descent_traj,
          seed.label, yaw, descent_frac, limit_clearance);
        best_descent_frac = descent_frac;
        best_yaw = yaw;
      }
    }
  }

  if (has_clean_candidate) {
    RCLCPP_INFO(
      get_logger(),
      "IK-consistent approach: seed %s yaw %.0f deg selected with clean descent "
      "(fraction %.3f, branch delta within %.3f rad, limit clearance %.3f rad)",
      result.seed_label.c_str(),
      result.yaw_rad * 180.0 / M_PI,
      result.descent_fraction,
      approach_branch_max_delta_,
      result.limit_clearance_rad);
  } else if (result.ok) {
    RCLCPP_WARN(
      get_logger(),
      "No yaw gave a clean vertical descent (best %.3f at yaw %.0f deg); using best "
      "candidate with joint-space descent fallback",
      best_descent_frac, best_yaw * 180.0 / M_PI);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "No IK-consistent approach found across yaw-sweep; falling back to free approach");
  }
  return result;
}

void PickPlaceNode::detected_objects_cb(
  const arm_interfaces::msg::ObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(detected_mutex_);
  detected_objects_.clear();
  for (const auto & obj : msg->objects) {
    if (!obj.object_id.empty()) {
      detected_objects_[obj.object_id] = obj;
    }
  }
}

bool PickPlaceNode::lookup_detected_object(
  const std::string & object_id, arm_interfaces::msg::ObjectPose & out) const
{
  std::lock_guard<std::mutex> lock(detected_mutex_);
  const auto it = detected_objects_.find(object_id);
  if (it == detected_objects_.end()) {
    return false;
  }
  out = it->second;
  return true;
}

void PickPlaceNode::setup_planning_scene(const std::string & target_id)
{
  if (!use_planning_scene_ || !planning_scene_) {
    return;
  }

  clear_managed_collision_objects();

  std::vector<moveit_msgs::msg::CollisionObject> objects;

  // Static work table so no plan ever routes through it.
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = table_pose_xyz_.size() > 0 ? table_pose_xyz_[0] : 0.0;
  table_pose.position.y = table_pose_xyz_.size() > 1 ? table_pose_xyz_[1] : 0.0;
  table_pose.position.z = table_pose_xyz_.size() > 2 ? table_pose_xyz_[2] : 0.0;
  table_pose.orientation.w = 1.0;
  objects.push_back(make_box("work_table", table_frame_, table_pose, table_size_xyz_));
  managed_collision_ids_.push_back("work_table");

  // Static sorting bins so transit/pre-place plans route AROUND neighbouring bins
  // instead of dragging the carried object through them. Model them as four walls
  // rather than one solid box: a solid box marks the open bin interior as
  // occupied, so after a valid drop/retreat MoveIt rejects home planning because
  // the gripper starts inside the bin's collision volume.
  for (std::size_t i = 0; i + 5 < bin_obstacles_.size(); i += 6) {
    const double cx = bin_obstacles_[i];
    const double cy = bin_obstacles_[i + 1];
    const double cz = bin_obstacles_[i + 2];
    const double sx = bin_obstacles_[i + 3];
    const double sy = bin_obstacles_[i + 4];
    const double sz = bin_obstacles_[i + 5];
    const double inner_x = bin_inner_size_xy_.size() > 0 ? bin_inner_size_xy_[0] : 0.16;
    const double inner_y = bin_inner_size_xy_.size() > 1 ? bin_inner_size_xy_[1] : inner_x;
    const double wall_x = std::max((sx - inner_x) * 0.5, 0.008);
    const double wall_y = std::max((sy - inner_y) * 0.5, 0.008);
    const std::string bin_id = "sorting_bin_" + std::to_string(i / 6);

    auto add_wall = [&](const std::string & suffix, double x, double y, double wx, double wy) {
        geometry_msgs::msg::Pose wall_pose;
        wall_pose.position.x = x;
        wall_pose.position.y = y;
        wall_pose.position.z = cz;
        wall_pose.orientation.w = 1.0;
        const std::string wall_id = bin_id + "_" + suffix;
        objects.push_back(make_box(wall_id, table_frame_, wall_pose, std::vector<double>{wx, wy, sz}));
        managed_collision_ids_.push_back(wall_id);
      };

    add_wall("wall_left", cx - inner_x * 0.5 - wall_x * 0.5, cy, wall_x, sy);
    add_wall("wall_right", cx + inner_x * 0.5 + wall_x * 0.5, cy, wall_x, sy);
    add_wall("wall_front", cx, cy - inner_y * 0.5 - wall_y * 0.5, inner_x, wall_y);
    add_wall("wall_back", cx, cy + inner_y * 0.5 + wall_y * 0.5, inner_x, wall_y);
  }

  // Every other detected object becomes an obstacle; the target is excluded so
  // it does not block the approach/descent (it is attached at grasp instead).
  std::vector<arm_interfaces::msg::ObjectPose> snapshot;
  {
    std::lock_guard<std::mutex> lock(detected_mutex_);
    snapshot.reserve(detected_objects_.size());
    for (const auto & kv : detected_objects_) {
      snapshot.push_back(kv.second);
    }
  }
  for (const auto & obj : snapshot) {
    if (obj.object_id == target_id) {
      continue;
    }
    if (obj.pose.pose.position.z > detected_obstacle_max_z_) {
      RCLCPP_DEBUG(
        get_logger(),
        "PlanningScene: skipping high/stale detection '%s' at z=%.3f",
        obj.object_id.c_str(), obj.pose.pose.position.z);
      continue;
    }
    const std::string frame =
      obj.pose.header.frame_id.empty() ? "base_link" : obj.pose.header.frame_id;
    const std::vector<double> size =
      obj.dimensions.size() >= 3 ?
      std::vector<double>{obj.dimensions[0], obj.dimensions[1], obj.dimensions[2]} :
      default_object_size_xyz_;
    objects.push_back(make_box(obj.object_id, frame, obj.pose.pose, size));
    managed_collision_ids_.push_back(obj.object_id);
  }

  planning_scene_->applyCollisionObjects(objects);
  managed_objects_ = objects;   // keep for restore after a grasp-window suspend
  scene_suspended_ = false;
  RCLCPP_INFO(
    get_logger(), "PlanningScene: added work_table + %zu obstacle object(s)",
    managed_collision_ids_.size() - 1);
}

void PickPlaceNode::suspend_scene_collision()
{
  if (!use_planning_scene_ || !planning_scene_ || scene_suspended_) {
    return;
  }
  if (!managed_collision_ids_.empty()) {
    planning_scene_->removeCollisionObjects(managed_collision_ids_);
  }
  scene_suspended_ = true;
  RCLCPP_INFO(get_logger(),
    "PlanningScene: obstacles suspended for grasp/place descent");
}

void PickPlaceNode::restore_scene_collision()
{
  if (!use_planning_scene_ || !planning_scene_ || !scene_suspended_) {
    return;
  }
  if (!managed_objects_.empty()) {
    planning_scene_->applyCollisionObjects(managed_objects_);
  }
  scene_suspended_ = false;
  RCLCPP_INFO(get_logger(),
    "PlanningScene: obstacles restored for transit");
}

void PickPlaceNode::attach_grasped_object(const std::string & object_id)
{
  if (!use_planning_scene_ || !planning_scene_ || !move_group_ || object_id.empty()) {
    return;
  }

  // Geometry only from detection; the POSE must come from the gripper. After a
  // successful grasp the object is physically AT grasp_link, not at its last
  // detected pose (which may be stale / from before the approach). Attaching the
  // box at a stale far-away pose rigidly offsets it from grasp_link -> every
  // subsequent plan (the immediate lift) is in collision -> Cartesian fraction
  // 0.0 and abort. So always co-locate the attached body with the gripper.
  std::vector<double> size = default_object_size_xyz_;
  arm_interfaces::msg::ObjectPose detected;
  if (lookup_detected_object(object_id, detected) && detected.dimensions.size() >= 3) {
    size = {detected.dimensions[0], detected.dimensions[1], detected.dimensions[2]};
  }
  // Shrink so the box clears the table surface and the fingers at grasp height
  // (otherwise the start state of the lift can read as in-collision).
  for (auto & s : size) {
    s = std::max(0.01, s * attach_box_shrink_);
  }

  const geometry_msgs::msg::Pose pose = move_group_->getCurrentPose(attach_link_).pose;
  const std::string frame = move_group_->getPoseReferenceFrame();

  auto obj = make_box(object_id, frame, pose, size);
  planning_scene_->applyCollisionObjects({obj});
  move_group_->attachObject(object_id, attach_link_, grasp_touch_links_);
  attached_object_id_ = object_id;
  RCLCPP_INFO(get_logger(), "PlanningScene: attached '%s' to %s at gripper",
    object_id.c_str(), attach_link_.c_str());
}

void PickPlaceNode::release_object(const std::string & object_id)
{
  if (!use_planning_scene_ || !move_group_ || object_id.empty()) {
    return;
  }
  move_group_->detachObject(object_id);
  attached_object_id_.clear();
  RCLCPP_INFO(get_logger(), "PlanningScene: detached '%s'", object_id.c_str());
}

void PickPlaceNode::clear_managed_collision_objects()
{
  if (!planning_scene_) {
    return;
  }
  std::vector<std::string> to_remove = managed_collision_ids_;
  if (!attached_object_id_.empty() && move_group_) {
    move_group_->detachObject(attached_object_id_);
    to_remove.push_back(attached_object_id_);
    attached_object_id_.clear();
  }
  if (!to_remove.empty()) {
    planning_scene_->removeCollisionObjects(to_remove);
  }
  managed_collision_ids_.clear();
  managed_objects_.clear();
  scene_suspended_ = false;
}

bool PickPlaceNode::execute_plan(moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }

  auto execute_future = std::async(std::launch::async, [&] {return move_group_->execute(plan);});
  if (execute_future.wait_for(motion_timeout_) != std::future_status::ready) {
    move_group_->stop();
    move_group_->clearPoseTargets();
    RCLCPP_ERROR(get_logger(), "Execution timed out");
    return false;
  }

  return execute_future.get() == moveit::core::MoveItErrorCode::SUCCESS;
}

bool PickPlaceNode::execute_trajectory(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  double speed_scale)
{
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroupInterface is not configured");
    return false;
  }

  apply_motion_scaling(speed_scale);
  auto execute_future = std::async(
    std::launch::async, [&] {return move_group_->execute(trajectory);});
  if (execute_future.wait_for(motion_timeout_) != std::future_status::ready) {
    move_group_->stop();
    RCLCPP_ERROR(get_logger(), "Cartesian execution timed out");
    return false;
  }

  return execute_future.get() == moveit::core::MoveItErrorCode::SUCCESS;
}

void PickPlaceNode::apply_motion_scaling(double requested_scale)
{
  if (!move_group_) {
    return;
  }
  const double velocity_scale = std::clamp(
    requested_scale, 0.01, std::clamp(motion_velocity_scale_cap_, 0.01, 1.0));
  const double acceleration_scale = std::clamp(
    requested_scale, 0.01, std::clamp(motion_acceleration_scale_cap_, 0.01, 1.0));
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scale);
}

bool PickPlaceNode::time_parameterize_trajectory(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  double speed_scale)
{
  if (!move_group_) {
    return false;
  }
  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Refusing to execute empty trajectory");
    return false;
  }

  auto current_state = move_group_->getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Unable to read current state for trajectory timing");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(move_group_->getRobotModel(), planning_group_);
  robot_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const double velocity_scale = std::clamp(
    speed_scale, 0.01, std::clamp(motion_velocity_scale_cap_, 0.01, 1.0));
  const double acceleration_scale = std::clamp(
    speed_scale, 0.01, std::clamp(motion_acceleration_scale_cap_, 0.01, 1.0));
  if (!totg.computeTimeStamps(robot_trajectory, velocity_scale, acceleration_scale)) {
    return false;
  }
  robot_trajectory.getRobotTrajectoryMsg(trajectory);
  return true;
}

std::vector<geometry_msgs::msg::Pose> PickPlaceNode::cartesian_waypoints_to(
  const geometry_msgs::msg::Pose & target) const
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (!move_group_) {
    waypoints.push_back(target);
    return waypoints;
  }

  geometry_msgs::msg::Pose start = move_group_->getCurrentPose("Link_6").pose;
  const double distance = pose_distance(start, target);
  const double raw_segments = max_pose_segment_m_ > 0.0 ?
    std::ceil(distance / max_pose_segment_m_) : 1.0;
  const int segments = std::max(1, std::min(max_pose_segments_, static_cast<int>(raw_segments)));
  waypoints.reserve(static_cast<std::size_t>(segments));
  for (int index = 1; index <= segments; ++index) {
    waypoints.push_back(interpolate_pose(start, target, static_cast<double>(index) / segments));
  }
  return waypoints;
}

bool PickPlaceNode::set_dl_joint_target(const geometry_msgs::msg::Pose & target)
{
  if (!move_group_ || !dl_ik_client_) {
    return false;
  }
  if (!dl_ik_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(get_logger(), "DL IK service is not available: %s", dl_ik_service_name_.c_str());
    return false;
  }

  auto request = std::make_shared<arm_interfaces::srv::SolveIk::Request>();
  request->target_pose = target;
  request->solver = "dl";
  const auto current_joints = move_group_->getCurrentJointValues();
  const auto seed_count = std::min(current_joints.size(), request->seed_angles.size());
  for (std::size_t i = 0; i < seed_count; ++i) {
    request->seed_angles[i] = current_joints[i];
  }

  auto future = dl_ik_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "DL IK service request timed out");
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(get_logger(), "DL IK service failed: %s", response->message.c_str());
    return false;
  }

  std::vector<double> joint_target(response->joint_angles.begin(), response->joint_angles.end());
  if (!move_group_->setJointValueTarget(joint_target)) {
    RCLCPP_WARN(get_logger(), "MoveIt rejected DL IK joint target");
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "Using DL IK joint target: %s", response->message.c_str());
  return true;
}

bool PickPlaceNode::execute_grasp(bool close)
{
  if (!gripper_) {
    RCLCPP_ERROR(get_logger(), "Gripper controller is not configured");
    return false;
  }
  return close ? gripper_->close() : gripper_->open();
}

geometry_msgs::msg::Pose PickPlaceNode::compute_pre_pose(
  const geometry_msgs::msg::Pose & target,
  double offset_z) const
{
  auto pose = target;
  pose.position.z += offset_z;
  return pose;
}

bool PickPlaceNode::validate_actual_grasp_pose(
  const geometry_msgs::msg::Pose & target_link6_pose) const
{
  return validate_actual_link6_pose(target_link6_pose, "Grasp guard");
}

bool PickPlaceNode::validate_actual_link6_pose(
  const geometry_msgs::msg::Pose & target_link6_pose,
  const char * label,
  double tolerance_m) const
{
  if (!grasp_pose_guard_enabled_) {
    return true;
  }
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "Grasp guard: MoveGroupInterface is not configured");
    return false;
  }
  auto state = move_group_->getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(get_logger(), "Grasp guard: unable to read current robot state");
    return false;
  }

  bool base_found = false;
  const Eigen::Isometry3d t_world_base = state->getFrameTransform("base_link", &base_found);
  if (!base_found) {
    RCLCPP_ERROR(get_logger(), "Grasp guard: base_link frame not found");
    return false;
  }

  const Eigen::Vector3d target_link6(
    target_link6_pose.position.x, target_link6_pose.position.y, target_link6_pose.position.z);
  const auto target_offset = rotate_vector(target_link6_pose.orientation, kLink6ToGraspXyz);
  const Eigen::Vector3d target_grasp(
    target_link6_pose.position.x + target_offset[0],
    target_link6_pose.position.y + target_offset[1],
    target_link6_pose.position.z + target_offset[2]);

  const Eigen::Vector3d actual_link6 =
    (t_world_base.inverse() * state->getGlobalLinkTransform("Link_6")).translation();
  const Eigen::Vector3d actual_grasp =
    (t_world_base.inverse() * state->getGlobalLinkTransform("grasp_link")).translation();

  const double link6_error = (actual_link6 - target_link6).norm();
  const double grasp_error = (actual_grasp - target_grasp).norm();
  const double worst_error = std::max(link6_error, grasp_error);
  const double tolerance =
    tolerance_m > 0.0 ? tolerance_m : grasp_pose_guard_tolerance_m_;
  const char * log_label = label == nullptr ? "Pose guard" : label;
  RCLCPP_INFO(
    get_logger(),
    "%s: Link_6 actual=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
    "err=%.3f; grasp_link actual=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) err=%.3f",
    log_label,
    actual_link6.x(), actual_link6.y(), actual_link6.z(),
    target_link6.x(), target_link6.y(), target_link6.z(), link6_error,
    actual_grasp.x(), actual_grasp.y(), actual_grasp.z(),
    target_grasp.x(), target_grasp.y(), target_grasp.z(), grasp_error);

  if (worst_error > tolerance) {
    RCLCPP_ERROR(
      get_logger(),
      "%s mismatch: error %.3fm > %.3fm",
      log_label, worst_error, tolerance);
    return false;
  }
  return true;
}

bool PickPlaceNode::validate_home_state() const
{
  if (!home_state_guard_enabled_) {
    return true;
  }
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "Home guard: MoveGroupInterface is not configured");
    return false;
  }
  auto state = move_group_->getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(get_logger(), "Home guard: unable to read current robot state");
    return false;
  }
  const auto * jmg = state->getJointModelGroup(planning_group_);
  if (!jmg) {
    RCLCPP_ERROR(get_logger(), "Home guard: planning group '%s' not found", planning_group_.c_str());
    return false;
  }

  moveit::core::RobotState home_state(*state);
  if (!home_state.setToDefaultValues(jmg, "home")) {
    RCLCPP_ERROR(get_logger(), "Home guard: named state 'home' unavailable");
    return false;
  }

  std::vector<double> actual;
  std::vector<double> expected;
  state->copyJointGroupPositions(jmg, actual);
  home_state.copyJointGroupPositions(jmg, expected);
  double max_error = 0.0;
  std::size_t max_index = 0;
  for (std::size_t i = 0; i < actual.size() && i < expected.size(); ++i) {
    const double error = std::abs(actual[i] - expected[i]);
    if (error > max_error) {
      max_error = error;
      max_index = i;
    }
  }
  const auto names = jmg->getVariableNames();
  const std::string joint_name = max_index < names.size() ? names[max_index] : std::to_string(max_index);
  RCLCPP_INFO(
    get_logger(), "Home guard: max joint error %.4frad at %s (tolerance %.4frad)",
    max_error, joint_name.c_str(), home_state_guard_tolerance_rad_);
  if (max_error > home_state_guard_tolerance_rad_) {
    RCLCPP_ERROR(
      get_logger(), "home state mismatch after HOME_RETURN: %.4frad > %.4frad at %s",
      max_error, home_state_guard_tolerance_rad_, joint_name.c_str());
    return false;
  }
  return true;
}

void PickPlaceNode::publish_feedback(
  const std::shared_ptr<GoalHandlePickAndPlace> & goal_handle,
  Phase phase,
  float progress) const
{
  auto feedback = std::make_shared<PickAndPlace::Feedback>();
  feedback->current_phase = phase_name(phase);
  feedback->progress = progress;
  goal_handle->publish_feedback(feedback);
  RCLCPP_INFO(get_logger(), "Pick/place phase: %s (%.0f%%)", phase_name(phase), progress * 100.0f);
}

const char * PickPlaceNode::phase_name(Phase phase)
{
  switch (phase) {
    case Phase::IDLE: return "idle";
    case Phase::HOME_START: return "STATE_1_HOME";
    case Phase::DETECT_OBJECT: return "STATE_2_DETECT_OBJECT";
    case Phase::PLAN_PRE_GRASP: return "STATE_3_PLAN_PRE_GRASP";
    case Phase::MOVE_TO_PRE_GRASP: return "STATE_4_MOVE_TO_PRE_GRASP";
    case Phase::DESCEND_TO_GRASP: return "STATE_5_DESCEND_TO_GRASP";
    case Phase::CLOSE_GRIPPER: return "STATE_6_CLOSE_GRIPPER";
    case Phase::LIFT_OBJECT: return "STATE_7_LIFT_OBJECT";
    case Phase::MOVING_TO_PRE_PLACE: return "STATE_8_MOVE_TO_PRE_PLACE";
    case Phase::DESCEND_TO_PLACE: return "STATE_9_DESCEND_TO_PLACE";
    case Phase::OPEN_GRIPPER: return "STATE_10_OPEN_GRIPPER";
    case Phase::RETREAT: return "STATE_11_RETREAT";
    case Phase::HOME_RETURN: return "STATE_12_HOME";
    case Phase::DONE: return "done";
    case Phase::FAILED: return "failed";
    case Phase::CANCELED: return "canceled";
  }
  return "unknown";
}

}  // namespace arm_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_nodes::PickPlaceNode>();
  node->configure(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
