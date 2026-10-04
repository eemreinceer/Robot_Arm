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


#include "arm_kinematics/inverse_kinematics.hpp"

#include <cmath>
#include <iostream>

#include "arm_kinematics/forward_kinematics.hpp"
#include "arm_kinematics/jacobian.hpp"
#include "arm_kinematics/dh_parameters.hpp"

namespace arm_kinematics
{

std::optional<std::array<double, 6>> solveIK(
  const Eigen::Isometry3d & target_pose,
  const std::array<double, 6> & seed_angles,
  int max_iter,
  double tolerance)
{
  auto joints = get_robot_joints();
  std::array<double, 6> q = seed_angles;

  const double lambda_min = 1e-4;
  const double lambda_max = 0.01;
  const double mu_0 = 0.02;  // Manipulability threshold

  for (int iter = 0; iter < max_iter; ++iter) {
    // 1. Compute current pose
    Eigen::Isometry3d T_curr = computeFK(q);

    // 2. Compute position error
    Eigen::Vector3d pos_err = target_pose.translation() - T_curr.translation();

    // 3. Compute orientation error
    Eigen::Matrix3d R_err = target_pose.linear() * T_curr.linear().transpose();
    Eigen::AngleAxisd angle_axis(R_err);
    Eigen::Vector3d rot_err = Eigen::Vector3d::Zero();
    double angle = angle_axis.angle();

    // Normalize to [-pi, pi]
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    if (std::abs(angle) > 1e-9) {
      rot_err = angle * angle_axis.axis();
    }

    // 6D error vector
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = pos_err;
    error.tail<3>() = rot_err;

    // Check convergence
    if (error.norm() < tolerance) {
      return q;
    }

    // 4. Compute Jacobian
    Eigen::MatrixXd J = computeJacobian(q);

    // 5. Calculate manipulability index for adaptive damping
    // mu = sqrt(det(J * J^T))
    double det_JJT = (J * J.transpose()).determinant();
    double mu = 0.0;
    if (det_JJT > 0.0) {
      mu = std::sqrt(det_JJT);
    }

    double lambda = lambda_min;
    if (mu < mu_0) {
      double ratio = mu / mu_0;
      lambda = lambda_min + (1.0 - ratio * ratio) * (lambda_max - lambda_min);
    }

    // 6. Compute Damped Least Squares update step
    // dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
    Eigen::MatrixXd JJT = J * J.transpose();
    Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * (JJT + lambda * lambda * Identity).inverse() * error;

    // 7. Update and clamp joint angles
    // Using a step size (alpha) to improve convergence stability
    double alpha = 0.8;
    for (size_t i = 0; i < 6; ++i) {
      q[i] += alpha * dq[i];

      // Clamp to joint limits
      if (q[i] < joints[i].min_limit) {q[i] = joints[i].min_limit;}
      if (q[i] > joints[i].max_limit) {q[i] = joints[i].max_limit;}
    }
  }

  // If we reached here, try one last check to see if error is within tolerance
  Eigen::Isometry3d T_curr = computeFK(q);
  Eigen::Vector3d pos_err = target_pose.translation() - T_curr.translation();
  Eigen::Matrix3d R_err = target_pose.linear() * T_curr.linear().transpose();
  Eigen::AngleAxisd angle_axis(R_err);
  double angle = angle_axis.angle();
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  Eigen::Vector3d rot_err = Eigen::Vector3d::Zero();
  if (std::abs(angle) > 1e-9) {
    rot_err = angle * angle_axis.axis();
  }
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = pos_err;
  error.tail<3>() = rot_err;

  if (error.norm() < tolerance) {
    return q;
  }

  return std::nullopt;
}

}  // namespace arm_kinematics
