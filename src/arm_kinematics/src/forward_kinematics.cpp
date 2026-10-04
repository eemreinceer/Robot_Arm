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


#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/dh_parameters.hpp"

namespace arm_kinematics
{

Eigen::Isometry3d computeFK(const std::array<double, 6> & joint_angles)
{
  auto transforms = computeAllFK(joint_angles);
  return transforms.back();
}

std::vector<Eigen::Isometry3d> computeAllFK(const std::array<double, 6> & joint_angles)
{
  auto joints = get_robot_joints();
  std::vector<Eigen::Isometry3d> transforms;
  transforms.reserve(6);

  Eigen::Isometry3d T_current = Eigen::Isometry3d::Identity();

  for (size_t i = 0; i < 6; ++i) {
    const auto & joint = joints[i];
    double q = joint_angles[i];

    Eigen::Isometry3d T_joint = Eigen::Isometry3d::Identity();

    // Origin rotation from RPY
    Eigen::Matrix3d R_origin = (
      Eigen::AngleAxisd(joint.yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(joint.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(joint.roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();

    T_joint.linear() = R_origin;
    T_joint.translation() = joint.xyz;

    // Rotation of joint angle q around the joint axis
    Eigen::Isometry3d T_rot = Eigen::Isometry3d::Identity();
    T_rot.linear() = Eigen::AngleAxisd(q, joint.axis).toRotationMatrix();

    T_current = T_current * T_joint * T_rot;
    transforms.push_back(T_current);
  }

  return transforms;
}

}  // namespace arm_kinematics
