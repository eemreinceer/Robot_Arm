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


#ifndef ARM_KINEMATICS__DH_PARAMETERS_HPP_
#define ARM_KINEMATICS__DH_PARAMETERS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <array>

namespace arm_kinematics
{

struct JointParameters
{
  Eigen::Vector3d xyz;
  double roll;
  double pitch;
  double yaw;
  Eigen::Vector3d axis;
  double min_limit;
  double max_limit;
};

// URDF-based joint parameters to avoid DH mismatch. Keep limits in sync with arm.urdf.xacro.
inline const std::array<JointParameters, 6> get_robot_joints()
{
  return {{
    // joint_1
    {
      Eigen::Vector3d(0.0, 0.0, 0.084),
      0.0, 0.0, 0.0,
      Eigen::Vector3d(0.0, 0.0, 1.0),
      -3.14, 3.14
    },
    // joint_2
    {
      Eigen::Vector3d(0.11106, 0.11837, 0.16),
      -1.5708, 0.0, 1.2093,
      Eigen::Vector3d(0.0, 0.0, -1.0),
      -2.86, 1.25
    },
    // joint_3
    {
      Eigen::Vector3d(-0.019287, -0.29938, 0.0020019),
      0.0, 0.0, 0.0,
      Eigen::Vector3d(0.0, 0.0, -1.0),
      -3.30, 0.94
    },
    // joint_4
    {
      Eigen::Vector3d(-0.22999, 0.014126, 0.088023),
      1.5708, 1.5199, -1.4356,
      Eigen::Vector3d(0.0, 0.0, -1.0),
      -3.14, 3.14
    },
    // joint_5
    {
      Eigen::Vector3d(-0.011421, 0.0051115, 0.133),
      1.5707, 0.74125, 1.1495,
      Eigen::Vector3d(0.0, 0.0, 1.0),
      -2.44, 2.33
    },
    // joint_6
    {
      Eigen::Vector3d(-0.073341, 0.073736, 0.0125),
      -1.5708, -1.1641, 0.78271,
      Eigen::Vector3d(0.0, 0.0, 1.0),
      -3.14, 3.14
    }
  }};
}

}  // namespace arm_kinematics

#endif  // ARM_KINEMATICS__DH_PARAMETERS_HPP_
