// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0
//
// Does the arm actually fail CLOSED?
//
// write() returns return_type::ERROR when the ESP32 ACK exchange fails. The
// comment in write() asserts that this "deactivates the hardware, which sends
// the stop frame and drops the arm" -- and a design decision rests on it (an
// out-of-envelope request is clamped rather than returned as ERROR, precisely
// to avoid that drop).
//
// But ros2_control routes a write() error through HardwareComponent::error(),
// which is the lifecycle ERROR transition, not deactivate(). We inherit
// LifecycleNodeInterface, whose on_error() default does nothing. So the claim
// needs measuring, not assuming: these tests drive the real lifecycle and
// count stop frames actually emitted in mock-serial mode.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "arm_hardware/stm32_system_interface.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/hardware_component.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{

// Granted access by the second `friend` declaration in the interface header.
class STM32SystemInterfaceLifecycleTestPeer
{
public:
  static hardware_interface::HardwareInfo make_info(const std::string & calibration_file)
  {
    hardware_interface::HardwareInfo info;
    info.name = "robot_arm_real_system";
    info.type = "system";
    info.hardware_parameters["mock_serial"] = "true";
    info.hardware_parameters["calibration_file"] = calibration_file;

    for (int index = 1; index <= 6; ++index) {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint_" + std::to_string(index);
      joint.type = "joint";

      hardware_interface::InterfaceInfo position;
      position.name = "position";
      hardware_interface::InterfaceInfo velocity;
      velocity.name = "velocity";

      joint.command_interfaces.push_back(position);
      joint.state_interfaces.push_back(position);
      joint.state_interfaces.push_back(velocity);
      info.joints.push_back(joint);
    }
    return info;
  }

  // Puts the interface in the state the physical fault happens from: active
  // and armed at a valid operator reference.
  static bool arm_at_zero(STM32SystemInterface & interface)
  {
    std::string reason;
    ActivationSafety::JointArray reference{};
    reference.fill(0.0);
    if (!interface.activation_safety_.set_reference(reference, reason)) {
      ADD_FAILURE() << "set_reference rejected the zero reference: " << reason;
      return false;
    }
    if (!interface.activation_safety_.arm(reason)) {
      ADD_FAILURE() << "arm rejected the zero reference: " << reason;
      return false;
    }
    return true;
  }

  static bool armed(const STM32SystemInterface & interface)
  {
    return interface.activation_safety_.armed();
  }

  static std::size_t stop_frames(const STM32SystemInterface & interface)
  {
    return interface.mock_stop_frame_count_;
  }
};

namespace
{

std::string calibration_path()
{
  // Set from CMake so the test does not depend on the working directory.
  return std::string(ARM_HARDWARE_TEST_CALIBRATION);
}

// Brings a component up to ACTIVE + ARMED and hands back both the wrapper and
// a borrowed pointer to the implementation for white-box assertions.
struct LiveComponent
{
  std::unique_ptr<hardware_interface::HardwareComponent> component;
  STM32SystemInterface * impl{nullptr};
};

LiveComponent make_active_armed_component()
{
  auto owned = std::make_unique<STM32SystemInterface>();
  auto * impl = owned.get();

  hardware_interface::HardwareComponentParams params;
  params.hardware_info =
    STM32SystemInterfaceLifecycleTestPeer::make_info(calibration_path());
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  params.logger = rclcpp::get_logger("test_error_transition");

  auto component = std::make_unique<hardware_interface::HardwareComponent>(std::move(owned));
  component->initialize(params);
  component->configure();
  component->activate();

  EXPECT_TRUE(STM32SystemInterfaceLifecycleTestPeer::arm_at_zero(*impl));
  EXPECT_TRUE(STM32SystemInterfaceLifecycleTestPeer::armed(*impl));
  EXPECT_EQ(STM32SystemInterfaceLifecycleTestPeer::stop_frames(*impl), 0u);

  return LiveComponent{std::move(component), impl};
}

// Control: the ordinary operator-driven path. This one is known to work and
// anchors what "fail closed" is supposed to look like.
TEST(ErrorTransition, DeactivateSendsTheStopFrame)
{
  auto live = make_active_armed_component();

  live.component->deactivate();

  EXPECT_FALSE(STM32SystemInterfaceLifecycleTestPeer::armed(*live.impl));
  EXPECT_EQ(STM32SystemInterfaceLifecycleTestPeer::stop_frames(*live.impl), 1u)
    << "deactivate() must emit exactly one stop frame";
}

// The path an ESP32 ACK failure actually takes. If this reports zero frames,
// the arm is NOT told to stop when the serial link fails -- the opposite of
// what write()'s comment claims.
TEST(ErrorTransition, ErrorTransitionAlsoSendsTheStopFrame)
{
  auto live = make_active_armed_component();

  live.component->error();

  EXPECT_FALSE(STM32SystemInterfaceLifecycleTestPeer::armed(*live.impl))
    << "the error transition must leave the interface disarmed";
  EXPECT_EQ(STM32SystemInterfaceLifecycleTestPeer::stop_frames(*live.impl), 1u)
    << "a write() failure must still command the actuators to stop; otherwise the "
    "last PWM target stays latched while ros2_control reports the hardware as gone";
}

// The stop frame must not be duplicated if the component is torn down after an
// error: a second `S` is harmless on the wire but would mean the disarm
// bookkeeping is not idempotent.
TEST(ErrorTransition, StopFrameIsNotRepeatedAfterErrorThenShutdown)
{
  auto live = make_active_armed_component();

  live.component->error();
  ASSERT_EQ(STM32SystemInterfaceLifecycleTestPeer::stop_frames(*live.impl), 1u);

  live.component->shutdown();

  EXPECT_EQ(STM32SystemInterfaceLifecycleTestPeer::stop_frames(*live.impl), 1u);
}

}  // namespace
}  // namespace arm_hardware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
