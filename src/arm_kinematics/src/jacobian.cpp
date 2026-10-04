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


#include "arm_kinematics/jacobian.hpp"
#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/dh_parameters.hpp"

namespace arm_kinematics
{

Eigen::MatrixXd computeJacobian(const std::array<double, 6> & joint_angles)
{
  auto joints = get_robot_joints();
  auto transforms = computeAllFK(joint_angles);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Vector3d p_e = transforms.back().translation();

  for (size_t i = 0; i < 6; ++i) {
    const auto & joint = joints[i];
    const auto & T_i = transforms[i];

    // Joint axis in base frame: z_{i-1} = R_0^i * a_i
    Eigen::Vector3d z_i = T_i.linear() * joint.axis;

    // Joint position in base frame: p_i = T_0^i.translation()
    Eigen::Vector3d p_i = T_i.translation();

    // Linear velocity component: J_v = z_i x (p_e - p_i)
    Eigen::Vector3d J_v = z_i.cross(p_e - p_i);

    // Angular velocity component: J_w = z_i
    Eigen::Vector3d J_w = z_i;

    J.block<3, 1>(0, i) = J_v;
    J.block<3, 1>(3, i) = J_w;
  }

  return J;
}

}  // namespace arm_kinematics
