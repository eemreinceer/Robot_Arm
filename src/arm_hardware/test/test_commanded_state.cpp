// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0
//
// Is a command still distinguishable from a measurement?
//
// This arm has no encoders, so read() echoes the delivered command into
// position_states_. Once joint_state_broadcaster publishes that, nothing
// downstream can tell an echo from a measurement -- which is how a stack ends
// up trusting feedback it does not have. The `commanded` GPIO exists to keep
// the two apart BEFORE the encoders arrive.
//
// These tests hold that contract:
//   * a description without the GPIO keeps working unchanged (the mock systems),
//   * the GPIO exports what it declares, and position_is_measured says 0.0
//     while position is an echo,
//   * commanded tracks what write() actually delivered, not what was requested
//     -- the two differ whenever the slew limiter or a calibration clamp bites,
//   * a name this interface cannot provide fails on_init instead of silently
//     disappearing.
//
// Deliberately driven through the interface's own lifecycle calls rather than
// hardware_interface::HardwareComponent: that header is Jazzy-only (see the
// guard around test_error_transition in CMakeLists.txt), and the Jetson runs
// Humble. This test has to run on both.

#include <string>
#include <vector>

#include "arm_hardware/stm32_system_interface.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{

class STM32SystemInterfaceCommandedStateTestPeer
{
public:
  // mock_serial: no ESP32 present, so write() runs its full command path and
  // stops short of the serial exchange.
  static hardware_interface::HardwareInfo make_info(
    const std::string & calibration_file, bool with_gpio,
    const std::string & extra_gpio_interface = "")
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

    if (with_gpio) {
      hardware_interface::ComponentInfo gpio;
      gpio.name = "commanded";
      gpio.type = "gpio";
      for (int index = 1; index <= 6; ++index) {
        hardware_interface::InterfaceInfo commanded;
        commanded.name = "joint_" + std::to_string(index) + "_position";
        gpio.state_interfaces.push_back(commanded);
      }
      hardware_interface::InterfaceInfo flag;
      flag.name = "position_is_measured";
      gpio.state_interfaces.push_back(flag);
      if (!extra_gpio_interface.empty()) {
        hardware_interface::InterfaceInfo extra;
        extra.name = extra_gpio_interface;
        gpio.state_interfaces.push_back(extra);
      }
      info.gpios.push_back(gpio);
    }
    return info;
  }

  static bool arm_at_zero(STM32SystemInterface & interface)
  {
    return arm_at(interface, 0.0);
  }

  static bool arm_at(STM32SystemInterface & interface, double reference_position)
  {
    std::string reason;
    ActivationSafety::JointArray reference{};
    reference.fill(reference_position);
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

  // Mirrors what on_safety_parameters() does on the disarmed -> armed edge
  // (stm32_system_interface.cpp, `arm_initialization_pending_ = true`). Arming
  // through the peer alone skips that latch, and without it write() takes the
  // normal delivery path instead of the arm-initialisation one.
  static void latch_arm_initialization(STM32SystemInterface & interface)
  {
    interface.arm_initialization_pending_ = true;
  }

  static void request(STM32SystemInterface & interface, std::size_t joint, double position)
  {
    interface.position_commands_[joint] = position;
  }

  static double commanded(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.commanded_position_states_[joint];
  }

  static double measured_flag(const STM32SystemInterface & interface)
  {
    return interface.position_is_measured_;
  }

  static double position_state(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.position_states_[joint];
  }

  static std::string fingerprint(const STM32SystemInterface & interface)
  {
    return interface.calibration_fingerprint_;
  }

  using Gate = STM32SystemInterface::CalibrationGate;

  static Gate gate(
    const STM32SystemInterface & interface, bool answered, const std::string & reported)
  {
    return interface.evaluate_calibration_gate(answered, reported);
  }

  static std::string canonical(const STM32SystemInterface & interface)
  {
    return STM32SystemInterface::calibration_canonical(interface.calibration_);
  }

  // Recomputes the fingerprint with one safety limit nudged by a milliradian,
  // without touching the live interface.
  static std::string fingerprint_with_moved_limit(const STM32SystemInterface & interface)
  {
    auto calibration = interface.calibration_;
    calibration[0].limit_max_rad -= 0.001;
    return STM32SystemInterface::calibration_fingerprint(calibration);
  }

  static double delivered_position(const STM32SystemInterface & interface, std::size_t joint)
  {
    return interface.microseconds_to_position(joint, interface.last_pulse_us_[joint]);
  }
};

namespace
{

std::string calibration_path()
{
  return std::string(ARM_HARDWARE_TEST_CALIBRATION);
}

const rclcpp::Time kNow{0, 0, RCL_STEADY_TIME};
const rclcpp::Duration kPeriod = rclcpp::Duration::from_seconds(0.02);

TEST(CommandedState, DescriptionWithoutGpioStillInitialises)
{
  // The mock test systems and any older description carry no GPIO block. If
  // this fails, the split broke back-compatibility.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), false)),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto states = interface.export_state_interfaces();
  EXPECT_EQ(states.size(), 12u) << "six joints x (position, velocity), no GPIO";
}

TEST(CommandedState, GpioExportsWhatItDeclares)
{
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);

  const auto states = interface.export_state_interfaces();
  EXPECT_EQ(states.size(), 19u) << "12 joint states + 6 commanded + 1 flag";

  std::vector<std::string> names;
  names.reserve(states.size());
  for (const auto & state : states) {
    names.push_back(state.get_name());
  }
  for (int index = 1; index <= 6; ++index) {
    const std::string expected = "commanded/joint_" + std::to_string(index) + "_position";
    EXPECT_NE(std::find(names.begin(), names.end(), expected), names.end())
      << "missing " << expected;
  }
  EXPECT_NE(
    std::find(names.begin(), names.end(), "commanded/position_is_measured"), names.end());
}

TEST(CommandedState, FlagStaysFalseWhilePositionIsAnEcho)
{
  // The whole point: with no encoder, the interface must not let anything
  // believe position is measured.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(STM32SystemInterfaceCommandedStateTestPeer::measured_flag(interface), 0.0);

  ASSERT_EQ(interface.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(STM32SystemInterfaceCommandedStateTestPeer::arm_at_zero(interface));
  ASSERT_EQ(interface.read(kNow, kPeriod), hardware_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(STM32SystemInterfaceCommandedStateTestPeer::measured_flag(interface), 0.0)
    << "no encoder is installed, so read() must not claim a measurement";
}

TEST(CommandedState, CommandedFollowsWhatWriteDelivered)
{
  // Requests a step far larger than one slew step, so requested != delivered.
  // If `commanded` echoed the request instead of the delivery, this catches it.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(interface.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(STM32SystemInterfaceCommandedStateTestPeer::arm_at_zero(interface));

  constexpr std::size_t kJoint = 0;
  constexpr double kBigStep = 0.20;   // rad, far beyond one 10 us slew step
  STM32SystemInterfaceCommandedStateTestPeer::request(interface, kJoint, kBigStep);

  // First write clears the arm-initialisation latch; the second one moves.
  ASSERT_EQ(interface.write(kNow, kPeriod), hardware_interface::return_type::OK);
  ASSERT_EQ(interface.write(kNow, kPeriod), hardware_interface::return_type::OK);

  const double commanded =
    STM32SystemInterfaceCommandedStateTestPeer::commanded(interface, kJoint);
  const double delivered =
    STM32SystemInterfaceCommandedStateTestPeer::delivered_position(interface, kJoint);
  EXPECT_NEAR(commanded, delivered, 1e-9)
    << "commanded must carry the delivered command, not the raw request";
  EXPECT_LT(std::abs(commanded), kBigStep)
    << "the slew limiter should still be ramping, so delivered < requested";

  // And read()'s echo agrees with it by construction. Stating it here is what
  // makes the future divergence meaningful: once encoders fill position_states_
  // from a sensor, this equality is exactly what stops holding.
  ASSERT_EQ(interface.read(kNow, kPeriod), hardware_interface::return_type::OK);
  EXPECT_NEAR(
    STM32SystemInterfaceCommandedStateTestPeer::position_state(interface, kJoint),
    commanded, 1e-9);
}

TEST(CommandedState, ArmingSeedsCommandedFromTheOperatorReference)
{
  // Arming latches the operator reference as the starting pulse: the arm is
  // wherever the operator left it; without encoders this pose is
  // the only position knowledge the system has. write()'s arm-initialisation
  // branch seeds position_states_ and last_pulse_us_ from that reference, so
  // commanded has to be seeded from the same place. If it is not, the two
  // channels disagree by the whole reference offset the moment the arm is
  // armed, and a diagnostic consumer reads that gap as a tracking error that
  // never happened.
  constexpr double kReference = 0.10;   // rad, inside the commissioning window

  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(interface.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(STM32SystemInterfaceCommandedStateTestPeer::arm_at(interface, kReference));
  STM32SystemInterfaceCommandedStateTestPeer::latch_arm_initialization(interface);

  // The arm-initialisation write: no PWM target is delivered yet, but the
  // interface's idea of where the arm is has just been set.
  ASSERT_EQ(interface.write(kNow, kPeriod), hardware_interface::return_type::OK);
  ASSERT_EQ(interface.read(kNow, kPeriod), hardware_interface::return_type::OK);

  const double position =
    STM32SystemInterfaceCommandedStateTestPeer::position_state(interface, 0);
  const double commanded =
    STM32SystemInterfaceCommandedStateTestPeer::commanded(interface, 0);
  EXPECT_NEAR(position, kReference, 1e-3) << "arming latches the operator reference";
  EXPECT_NEAR(commanded, position, 1e-3)
    << "commanded must start from the same reference, not from a stale zero";
}

// The fingerprint is only worth anything if the firmware's generator computes
// the SAME string from the SAME YAML. Both implementations are hand-written --
// one in C++ here, one in Python in generate_robot_config.py -- so this pins
// the value they must agree on. Regenerate with:
//
//   python3 -c "import sys; sys.path.insert(0,'firmware/esp32_servo_ctrl/tools');
//   from pathlib import Path; import generate_robot_config as g;
//   print(g.calibration_fingerprint(Path('src/arm_hardware/test/mock_calibration.yaml')))"
//
// If this test fails after a change to either side, the two have drifted and
// the host would start refusing a firmware that is actually correct.
TEST(CalibrationFingerprint, MatchesTheFirmwareGenerator)
{
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    STM32SystemInterfaceCommandedStateTestPeer::fingerprint(interface),
    "79387dc542450ed7")
    << "host and generate_robot_config.py must derive the same fingerprint from "
       "mock_calibration.yaml. Host canonical string was:\n"
    << STM32SystemInterfaceCommandedStateTestPeer::canonical(interface);
}

TEST(CalibrationFingerprint, ChangingASafetyLimitChangesTheFingerprint)
{
  // The whole point is detecting a YAML edited without reflashing. If a limit
  // can move without moving the fingerprint, the check is decorative.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  const std::string original =
    STM32SystemInterfaceCommandedStateTestPeer::fingerprint(interface);

  const std::string moved =
    STM32SystemInterfaceCommandedStateTestPeer::fingerprint_with_moved_limit(interface);
  EXPECT_NE(moved, original);
}

TEST(CalibrationFingerprint, GateSeparatesDisagreementFromInabilityToVerify)
{
  // These are different failures and they must not collapse. A firmware that
  // answers with different numbers is enforcing limits this host is not
  // converting for -- always refuse. A firmware too old to answer is a
  // deployment state, and refusing it by default would brick a working arm.
  STM32SystemInterface interface;
  ASSERT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(calibration_path(), true)),
    hardware_interface::CallbackReturn::SUCCESS);
  const std::string ours = STM32SystemInterfaceCommandedStateTestPeer::fingerprint(interface);

  using Gate = STM32SystemInterfaceCommandedStateTestPeer::Gate;
  EXPECT_EQ(
    STM32SystemInterfaceCommandedStateTestPeer::gate(interface, true, ours), Gate::kAgreed);
  EXPECT_EQ(
    STM32SystemInterfaceCommandedStateTestPeer::gate(interface, true, "0123456789abcdef"),
    Gate::kMismatch);
  EXPECT_EQ(
    STM32SystemInterfaceCommandedStateTestPeer::gate(interface, false, ""), Gate::kUnverified);
  // `K,` with nothing after it must not compare equal to anything.
  EXPECT_EQ(
    STM32SystemInterfaceCommandedStateTestPeer::gate(interface, true, ""), Gate::kUnverified);
}

TEST(CommandedState, UnknownGpioInterfaceFailsInit)
{
  // A typo must not degrade into a silently missing interface.
  STM32SystemInterface interface;
  EXPECT_EQ(
    interface.on_init(
      STM32SystemInterfaceCommandedStateTestPeer::make_info(
        calibration_path(), true, "joint_7_position")),
    hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace arm_hardware

// on_activate() starts the /robot_arm_hardware_safety node, which needs a live
// rclcpp context. Without this the activation fails and the tests below would
// report a broken commanded channel when the real cause is the harness.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
