// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0
//
// Does a radian actually reach the servo it was meant for, pointing the way it
// was meant to point?
//
// Three conversions sit between MoveIt and a pulse width, and until now the
// suite only ever exercised the identity case: mock_calibration.yaml maps
// channel n to joint n, inverts nothing and offsets nothing. Under that fixture
// a mapping bug that fell back to joint order, a dropped inversion, or an
// offset applied in the wrong direction all still pass.
//
// mock_calibration_mapping.yaml breaks that symmetry on purpose -- reversed
// wire order, two inverted joints, asymmetric offsets, non-default pulse ranges
// -- and these tests pin:
//
//   * rad -> us -> rad round-trips inside one pulse of quantisation,
//   * invert mirrors the pulse about the range centre rather than shifting it,
//   * the zero offset moves the neutral pulse and does not rescale the range,
//   * the frame on the wire is ordered by CHANNEL, not by joint.
//
// Distro-portable by construction: no hardware_interface::HardwareComponent,
// so this runs on the Jetson's Humble as well as Jazzy.

#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <string>
#include <vector>

#include "arm_hardware/stm32_system_interface.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{

class STM32SystemInterfaceMappingTestPeer
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

  static int to_us(const STM32SystemInterface & interface, std::size_t joint, double position)
  {
    return interface.position_to_microseconds(joint, position);
  }

  static double to_rad(const STM32SystemInterface & interface, std::size_t joint, int pulse_us)
  {
    return interface.microseconds_to_position(joint, pulse_us);
  }

  static std::string frame(
    const STM32SystemInterface & interface, const std::array<int, 6> & channel_pulse_us)
  {
    return interface.position_command(channel_pulse_us);
  }

  static int channel_of(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.calibration_[joint].channel;
  }

  static bool inverted(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.calibration_[joint].invert;
  }

  static void pulse_range(
    const STM32SystemInterface & interface, std::size_t joint, int & min_us, int & max_us)
  {
    min_us = interface.calibration_[joint].min_us;
    max_us = interface.calibration_[joint].max_us;
  }

  static double zero_offset(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.calibration_[joint].zero_offset_rad;
  }
};

namespace
{

std::string mapping_calibration_path()
{
  return std::string(ARM_HARDWARE_TEST_MAPPING_CALIBRATION);
}

TEST(CalibrationMapping, RadiansRoundTripThroughPulseWidth)
{
  // One microsecond is the quantisation floor: a 2000 us span over ~3.14 rad is
  // ~1.6 mrad per count, so anything inside two counts is the integer pulse,
  // not a conversion error.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceMappingTestPeer::make_info(mapping_calibration_path())),
    hardware_interface::CallbackReturn::SUCCESS);

  for (std::size_t joint = 0; joint < 6; ++joint) {
    int min_us = 0;
    int max_us = 0;
    STM32SystemInterfaceMappingTestPeer::pulse_range(interface, joint, min_us, max_us);
    const double offset = STM32SystemInterfaceMappingTestPeer::zero_offset(interface, joint);

    for (const double target : {-0.20, -0.05, 0.0, 0.05, 0.20}) {
      const int pulse = STM32SystemInterfaceMappingTestPeer::to_us(interface, joint, target);
      EXPECT_GE(pulse, min_us) << "joint " << joint;
      EXPECT_LE(pulse, max_us) << "joint " << joint;

      const double recovered =
        STM32SystemInterfaceMappingTestPeer::to_rad(interface, joint, pulse);
      // to_rad returns the joint-space value, so the offset applied on the way
      // down has already been removed on the way back.
      const double span_per_us =
        (2.0 * 1.57) / static_cast<double>(max_us - min_us);
      EXPECT_NEAR(recovered, target, 2.0 * span_per_us + 1e-9)
        << "joint " << joint << " target " << target << " offset " << offset;
    }
  }
}

TEST(CalibrationMapping, InvertMirrorsThePulseAboutTheRangeCentre)
{
  // joint_2 and joint_4 are inverted in the fixture. Mirroring, not shifting:
  // a command and its negation must land symmetrically about the centre pulse,
  // and the inverted joint must move the OPPOSITE way from the plain one.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceMappingTestPeer::make_info(mapping_calibration_path())),
    hardware_interface::CallbackReturn::SUCCESS);

  constexpr std::size_t kPlain = 0;      // joint_1, invert: false, offset 0
  constexpr std::size_t kInverted = 3;   // joint_4, invert: true,  offset 0
  ASSERT_FALSE(STM32SystemInterfaceMappingTestPeer::inverted(interface, kPlain));
  ASSERT_TRUE(STM32SystemInterfaceMappingTestPeer::inverted(interface, kInverted));

  const int plain_up = STM32SystemInterfaceMappingTestPeer::to_us(interface, kPlain, 0.40);
  const int plain_centre = STM32SystemInterfaceMappingTestPeer::to_us(interface, kPlain, 0.0);
  const int inverted_up = STM32SystemInterfaceMappingTestPeer::to_us(interface, kInverted, 0.40);
  const int inverted_centre =
    STM32SystemInterfaceMappingTestPeer::to_us(interface, kInverted, 0.0);

  EXPECT_EQ(plain_centre, inverted_centre) << "inversion must not move neutral";
  EXPECT_GT(plain_up, plain_centre);
  EXPECT_LT(inverted_up, inverted_centre) << "an inverted joint drives the other way";
  EXPECT_EQ(plain_up - plain_centre, inverted_centre - inverted_up)
    << "mirror, not a shift: the magnitudes have to match";

  // And the inverse conversion has to undo it, or the echoed state would report
  // the arm on the wrong side of neutral.
  EXPECT_NEAR(
    STM32SystemInterfaceMappingTestPeer::to_rad(interface, kInverted, inverted_up), 0.40, 2e-3);
}

TEST(CalibrationMapping, ZeroOffsetMovesNeutralWithoutRescalingTheRange)
{
  // joint_3 carries a -0.30 rad offset. The offset belongs to where the horn
  // sits, not to how many radians a microsecond is worth: the scale must be
  // identical to a joint with no offset and the same pulse range.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceMappingTestPeer::make_info(mapping_calibration_path())),
    hardware_interface::CallbackReturn::SUCCESS);

  constexpr std::size_t kOffset = 2;    // joint_3, offset -0.30, 600..2400 us
  const double offset = STM32SystemInterfaceMappingTestPeer::zero_offset(interface, kOffset);
  ASSERT_NEAR(offset, -0.30, 1e-12);

  const int at_zero = STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, 0.0);
  const int at_offset =
    STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, -offset);

  int min_us = 0;
  int max_us = 0;
  STM32SystemInterfaceMappingTestPeer::pulse_range(interface, kOffset, min_us, max_us);
  const int centre = (min_us + max_us) / 2;
  EXPECT_EQ(at_offset, centre)
    << "commanding -offset must land on the mechanical centre pulse";
  EXPECT_NE(at_zero, centre) << "a non-zero offset must move where joint zero sits";

  // Scale check: equal radian steps must produce equal pulse steps regardless
  // of the offset.
  const int step_low = STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, 0.10) -
    STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, 0.0);
  const int step_high = STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, 0.30) -
    STM32SystemInterfaceMappingTestPeer::to_us(interface, kOffset, 0.20);
  EXPECT_LE(std::abs(step_low - step_high), 1) << "the offset must not rescale the mapping";
}

TEST(CalibrationMapping, FrameIsOrderedByChannelNotByJoint)
{
  // The fixture reverses the wire order, so a frame built in joint order would
  // drive every servo with another joint's pulse -- and on the identity fixture
  // that bug is invisible.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceMappingTestPeer::make_info(mapping_calibration_path())),
    hardware_interface::CallbackReturn::SUCCESS);

  // Distinct pulse per joint, placed the way write() places them.
  std::array<int, 6> channel_pulse_us{};
  for (std::size_t joint = 0; joint < 6; ++joint) {
    const auto channel = STM32SystemInterfaceMappingTestPeer::channel_of(interface, joint);
    ASSERT_GE(channel, 1);
    ASSERT_LE(channel, 6);
    channel_pulse_us[static_cast<std::size_t>(channel - 1)] =
      1000 + 100 * static_cast<int>(joint);
  }

  // joint_1 is on channel 6 and joint_6 on channel 1, so the frame must read
  // back reversed relative to the joint numbering.
  EXPECT_EQ(
    STM32SystemInterfaceMappingTestPeer::frame(interface, channel_pulse_us),
    "P1500,1400,1300,1200,1100,1000\n");
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
