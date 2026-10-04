// Copyright 2026 Emre Inceer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "arm_interfaces/srv/solve_fk.hpp"
#include "arm_interfaces/srv/solve_ik.hpp"
#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/inverse_kinematics.hpp"
#include "arm_kinematics/jacobian.hpp"

namespace arm_kinematics
{

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("kinematics_node")
  {
    fk_service_ = this->create_service<arm_interfaces::srv::SolveFk>(
      "/fk_solve",
      std::bind(&KinematicsNode::handle_fk, this, std::placeholders::_1, std::placeholders::_2));

    ik_service_ = this->create_service<arm_interfaces::srv::SolveIk>(
      "/ik_solve",
      std::bind(&KinematicsNode::handle_ik, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "Kinematics Node started. Services advertised: /fk_solve, /ik_solve");
  }

private:
  void handle_fk(
    const std::shared_ptr<arm_interfaces::srv::SolveFk::Request> request,
    std::shared_ptr<arm_interfaces::srv::SolveFk::Response> response)
  {
    std::array<double, 6> joint_angles;
    std::copy(request->joint_angles.begin(), request->joint_angles.end(), joint_angles.begin());

    try {
      Eigen::Isometry3d T = computeFK(joint_angles);

      response->tcp_pose.header.stamp = this->now();
      response->tcp_pose.header.frame_id = "base_link";
      response->tcp_pose.pose.position.x = T.translation().x();
      response->tcp_pose.pose.position.y = T.translation().y();
      response->tcp_pose.pose.position.z = T.translation().z();

      Eigen::Quaterniond q(T.linear());
      response->tcp_pose.pose.orientation.x = q.x();
      response->tcp_pose.pose.orientation.y = q.y();
      response->tcp_pose.pose.orientation.z = q.z();
      response->tcp_pose.pose.orientation.w = q.w();

      response->success = true;
      response->message = "FK solved successfully";
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("FK solver exception: ") + e.what();
    }
  }

  void handle_ik(
    const std::shared_ptr<arm_interfaces::srv::SolveIk::Request> request,
    std::shared_ptr<arm_interfaces::srv::SolveIk::Response> response)
  {
    // If the solver parameter is not empty, verify if it is supported (numerical or dls)
    if (!request->solver.empty() && request->solver != "numerical" && request->solver != "dls") {
      response->success = false;
      response->message = "Unsupported solver: " + request->solver +
        ". Supported: 'numerical', 'dls'";
      return;
    }

    Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
    target_pose.translation() = Eigen::Vector3d(
      request->target_pose.position.x,
      request->target_pose.position.y,
      request->target_pose.position.z
    );

    Eigen::Quaterniond q(
      request->target_pose.orientation.w,
      request->target_pose.orientation.x,
      request->target_pose.orientation.y,
      request->target_pose.orientation.z
    );
    target_pose.linear() = q.toRotationMatrix();

    std::array<double, 6> seed_angles;
    std::copy(request->seed_angles.begin(), request->seed_angles.end(), seed_angles.begin());

    auto result = solveIK(target_pose, seed_angles);

    if (result.has_value()) {
      std::copy(result->begin(), result->end(), response->joint_angles.begin());

      // Calculate remaining position error
      Eigen::Isometry3d T_curr = computeFK(*result);
      double error = (target_pose.translation() - T_curr.translation()).norm();

      response->success = true;
      response->position_error = error;
      response->message = "IK solved successfully using DLS";
    } else {
      response->success = false;
      response->position_error = -1.0;
      response->message = "IK solver failed to converge";
      // Fill joint_angles with zeros or seed_angles so it's initialized
      std::copy(seed_angles.begin(), seed_angles.end(), response->joint_angles.begin());
    }
  }

  rclcpp::Service<arm_interfaces::srv::SolveFk>::SharedPtr fk_service_;
  rclcpp::Service<arm_interfaces::srv::SolveIk>::SharedPtr ik_service_;
};

}  // namespace arm_kinematics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_kinematics::KinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
