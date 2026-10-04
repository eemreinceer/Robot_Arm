// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#include <array>
#include <string>
#include <vector>

#include "arm_hardware/activation_safety.hpp"
#include "arm_hardware/stm32_system_interface.hpp"
#include "gtest/gtest.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"

namespace arm_hardware
{

class STM32SystemInterfaceTestPeer
{
public:
  static void configure_mock(STM32SystemInterface & hardware)
  {
    ActivationSafety::LimitsArray limits{};
    std::string reason;
    for (std::size_t index = 0; index < 6; ++index) {
      hardware.calibration_[index].channel = static_cast<int>(index + 1);
      hardware.calibration_[index].joint = "joint_" + std::to_string(index + 1);
      hardware.calibration_[index].min_rad = -1.57;
      hardware.calibration_[index].max_rad = 1.57;
      hardware.calibration_[index].limit_min_rad = -1.57;
      hardware.calibration_[index].limit_max_rad = 1.57;
      hardware.calibration_[index].min_us = 500;
      hardware.calibration_[index].max_us = 2500;
      hardware.calibration_[index].max_velocity_rad_s = 0.1;
      hardware.last_pulse_us_[index] = 1500;
      limits[index] = {-1.57, 1.57, 0.1};
    }
    ASSERT_TRUE(hardware.activation_safety_.configure(limits, false, 0.3, reason)) << reason;
    hardware.position_commands_.fill(0.0);
    hardware.position_states_.fill(0.0);
    hardware.mock_serial_ = true;
    hardware.active_ = true;
    hardware.last_write_time_ = std::chrono::steady_clock::time_point{};
  }

  static void arm(STM32SystemInterface & hardware, const ActivationSafety::JointArray & reference)
  {
    std::string reason;
    ASSERT_TRUE(hardware.activation_safety_.set_reference(reference, reason)) << reason;
    ASSERT_TRUE(hardware.activation_safety_.arm(reason)) << reason;
    hardware.arm_initialization_pending_ = true;
  }

  static int pulse_for(
    const STM32SystemInterface & hardware, const std::size_t index, const double position)
  {
    return hardware.position_to_microseconds(index, position);
  }

  static void set_safety_limits(
    STM32SystemInterface & hardware, const std::size_t index,
    const double lower, const double upper)
  {
    hardware.calibration_[index].limit_min_rad = lower;
    hardware.calibration_[index].limit_max_rad = upper;
  }

  static std::size_t position_frames(const STM32SystemInterface & hardware)
  {
    return hardware.mock_position_frame_count_;
  }

  static std::size_t stop_frames(const STM32SystemInterface & hardware)
  {
    return hardware.mock_stop_frame_count_;
  }

  static const std::array<int, 6> & pulses(const STM32SystemInterface & hardware)
  {
    return hardware.last_pulse_us_;
  }

  static void set_commands(
    STM32SystemInterface & hardware, const ActivationSafety::JointArray & commands)
  {
    hardware.position_commands_ = commands;
    hardware.last_write_time_ = std::chrono::steady_clock::time_point{};
  }

  static rcl_interfaces::msg::SetParametersResult set_armed(
    STM32SystemInterface & hardware, const bool armed)
  {
    return hardware.on_safety_parameters({rclcpp::Parameter("armed", armed)});
  }
};

ActivationSafety::LimitsArray default_limits(const double max_velocity = 0.1)
{
  ActivationSafety::LimitsArray limits{};
  limits.fill({-1.57, 1.57, max_velocity});
  return limits;
}

TEST(ActivationSafetyTest, RequiresReferenceBeforeArming)
{
  ActivationSafety safety;
  std::string reason;
  ASSERT_TRUE(safety.configure(default_limits(), false, 0.3, reason));
  EXPECT_FALSE(safety.arm(reason));
  EXPECT_NE(reason.find("reference_positions"), std::string::npos);
}

TEST(ActivationSafetyTest, CalibrationGateRejectsOutsideReferenceWindow)
{
  ActivationSafety safety;
  std::string reason;
  ActivationSafety::JointArray reference{};
  ActivationSafety::JointArray requested{};
  ActivationSafety::JointArray delivered{};
  ASSERT_TRUE(safety.configure(default_limits(), false, 0.3, reason));
  ASSERT_TRUE(safety.set_reference(reference, reason));
  ASSERT_TRUE(safety.arm(reason));
  requested[2] = 0.31;
  EXPECT_FALSE(safety.filter(requested, 0.02, delivered, reason));
  EXPECT_NE(reason.find("outside safe range"), std::string::npos);
}

TEST(ActivationSafetyTest, ClampFoldsOutOfEnvelopeRequestToBoundary)
{
  ActivationSafety safety;
  std::string reason;
  ActivationSafety::JointArray reference{};
  ActivationSafety::JointArray requested{};
  ActivationSafety::JointArray clamped{};
  ActivationSafety::JointArray delivered{};
  ASSERT_TRUE(safety.configure(default_limits(), false, 0.3, reason));
  ASSERT_TRUE(safety.set_reference(reference, reason));
  ASSERT_TRUE(safety.arm(reason));

  requested[2] = 0.31;
  EXPECT_TRUE(safety.clamp_into_safe_range(requested, clamped));
  EXPECT_NEAR(clamped[2], 0.3, 1e-12);
  // The clamped request must survive the filter, so an out-of-envelope command
  // holds at the boundary instead of taking PWM away.
  EXPECT_TRUE(safety.filter(clamped, 0.02, delivered, reason)) << reason;

  requested[2] = 0.1;
  EXPECT_FALSE(safety.clamp_into_safe_range(requested, clamped));
  EXPECT_NEAR(clamped[2], 0.1, 1e-12);
}

TEST(ActivationSafetyTest, AppliesPerJointVelocityLimit)
{
  ActivationSafety safety;
  std::string reason;
  ActivationSafety::JointArray reference{};
  ActivationSafety::JointArray requested{};
  ActivationSafety::JointArray delivered{};
  ASSERT_TRUE(safety.configure(default_limits(0.1), false, 0.3, reason));
  ASSERT_TRUE(safety.set_reference(reference, reason));
  ASSERT_TRUE(safety.arm(reason));
  requested.fill(0.2);
  ASSERT_TRUE(safety.filter(requested, 0.02, delivered, reason)) << reason;
  for (const double value : delivered) {
    EXPECT_NEAR(value, 0.002, 1e-12);
  }
  safety.accept(delivered);
  ASSERT_TRUE(safety.filter(requested, 0.02, delivered, reason)) << reason;
  for (const double value : delivered) {
    EXPECT_NEAR(value, 0.004, 1e-12);
  }
}

// Narrowing the safe range must not move neutral. min_rad/max_rad are the
// measured rad<->us scale anchors (500/2500us = 180deg on the bench); the
// safe range is a separate, narrower window. Collapsing the two -- which is
// what an earlier version of this config did -- rescales the mapping, and an
// asymmetric narrowing moves the neutral pulse: with max_rad cut to 0.86 a
// commanded 0.0 rad came out as 1777us, roughly 25 degrees of unasked-for
// motion on the next activation.
TEST(STM32SystemInterfaceTest, NarrowSafeRangeLeavesTheScaleAndNeutralIntact)
{
  STM32SystemInterface hardware;
  STM32SystemInterfaceTestPeer::configure_mock(hardware);

  EXPECT_EQ(STM32SystemInterfaceTestPeer::pulse_for(hardware, 4, 0.0), 1500);

  STM32SystemInterfaceTestPeer::set_safety_limits(hardware, 4, -1.52, 0.86);

  // Neutral and any in-range angle keep the pulse they had before narrowing.
  EXPECT_EQ(STM32SystemInterfaceTestPeer::pulse_for(hardware, 4, 0.0), 1500);
  EXPECT_EQ(
    STM32SystemInterfaceTestPeer::pulse_for(hardware, 4, 0.785),
    STM32SystemInterfaceTestPeer::pulse_for(hardware, 0, 0.785));

  // Beyond the safe range the command is held at the boundary, not rescaled.
  const int at_upper = STM32SystemInterfaceTestPeer::pulse_for(hardware, 4, 0.86);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::pulse_for(hardware, 4, 1.40), at_upper);
  EXPECT_LT(at_upper, 2500);
}

TEST(STM32SystemInterfaceTest, DisarmedWriteProducesNoPositionFrame)
{
  STM32SystemInterface hardware;
  STM32SystemInterfaceTestPeer::configure_mock(hardware);
  EXPECT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::position_frames(hardware), 0U);
}

TEST(STM32SystemInterfaceTest, FirstTargetStartsAtReferenceAndRemainsPulseLimited)
{
  STM32SystemInterface hardware;
  STM32SystemInterfaceTestPeer::configure_mock(hardware);
  ActivationSafety::JointArray reference{};
  STM32SystemInterfaceTestPeer::arm(hardware, reference);

  ActivationSafety::JointArray target{};
  target.fill(0.2);
  STM32SystemInterfaceTestPeer::set_commands(hardware, target);
  ASSERT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::position_frames(hardware), 0U);
  for (const int pulse : STM32SystemInterfaceTestPeer::pulses(hardware)) {
    EXPECT_EQ(pulse, 1500);
  }

  STM32SystemInterfaceTestPeer::set_commands(hardware, target);
  ASSERT_EQ(
    hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::position_frames(hardware), 1U);
  for (const int pulse : STM32SystemInterfaceTestPeer::pulses(hardware)) {
    EXPECT_LE(pulse - 1500, 10);
    EXPECT_GT(pulse, 1500);
  }
}

TEST(STM32SystemInterfaceTest, DisarmSendsExactlyOneStopFrame)
{
  STM32SystemInterface hardware;
  STM32SystemInterfaceTestPeer::configure_mock(hardware);
  ActivationSafety::JointArray reference{};
  STM32SystemInterfaceTestPeer::arm(hardware, reference);

  EXPECT_TRUE(STM32SystemInterfaceTestPeer::set_armed(hardware, false).successful);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::stop_frames(hardware), 1U);
  EXPECT_TRUE(STM32SystemInterfaceTestPeer::set_armed(hardware, false).successful);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::stop_frames(hardware), 1U);

  EXPECT_FALSE(STM32SystemInterfaceTestPeer::set_armed(hardware, true).successful);
  EXPECT_EQ(STM32SystemInterfaceTestPeer::stop_frames(hardware), 1U);
}

}  // namespace arm_hardware
