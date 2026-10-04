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


#ifndef ARM_KINEMATICS__FORWARD_KINEMATICS_HPP_
#define ARM_KINEMATICS__FORWARD_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace arm_kinematics
{

/**
 * @brief Computes forward kinematics for the 6DOF arm from base to joint_6 (Link_6)
 * @param joint_angles The 6 joint angles in radians
 * @return Eigen::Isometry3d Homogeneous transform from base_link to Link_6
 */
Eigen::Isometry3d computeFK(const std::array<double, 6> & joint_angles);

/**
 * @brief Computes forward kinematics for all links/joints up to Link_6
 * @param joint_angles The 6 joint angles in radians
 * @return std::vector<Eigen::Isometry3d> List of cumulative transforms from base_link to each link frame
 *         Index 0: base to Link_1
 *         Index 1: base to Link_2
 *         ...
 *         Index 5: base to Link_6
 */
std::vector<Eigen::Isometry3d> computeAllFK(const std::array<double, 6> & joint_angles);

}  // namespace arm_kinematics

#endif  // ARM_KINEMATICS__FORWARD_KINEMATICS_HPP_
