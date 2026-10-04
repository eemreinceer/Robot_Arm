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


#ifndef ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_
#define ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <array>
#include <optional>

namespace arm_kinematics
{

/**
 * @brief Solves the inverse kinematics using Damped Least Squares (DLS)
 * @param target_pose The target pose (Isometry3d) in base_link frame
 * @param seed_angles Initial guess for the joint angles
 * @param max_iter Maximum number of solver iterations
 * @param tolerance Convergence tolerance for the pose error norm
 * @return std::optional<std::array<double, 6>> Joint angles if solved, std::nullopt otherwise
 */
std::optional<std::array<double, 6>> solveIK(
  const Eigen::Isometry3d & target_pose,
  const std::array<double, 6> & seed_angles,
  int max_iter = 200,
  double tolerance = 1e-4);

}  // namespace arm_kinematics

#endif  // ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_
