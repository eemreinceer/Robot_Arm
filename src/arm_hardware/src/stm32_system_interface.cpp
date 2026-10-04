// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#include "arm_hardware/stm32_system_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace arm_hardware
{
namespace
{

const auto kLogger = rclcpp::get_logger("STM32SystemInterface");
rclcpp::Clock kThrottleClock{RCL_STEADY_TIME};

// Successful command-to-ACK exchanges are classified by inclusive upper
// bounds. One-millisecond resolution covers the complete 20 ms control budget;
// the wider tail buckets distinguish a near miss from a severe scheduling
// stall without allocating or storing individual samples in the RT path.
constexpr std::array<std::int64_t, 22> kAckLatencyUpperBoundsUs = {
  1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000,
  11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000,
  25000, 50000};

bool parse_bool(const std::string & value)
{
  return value == "true" || value == "True" || value == "1";
}

speed_t baud_constant(const int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud_rate));
  }
}

}  // namespace

STM32SystemInterface::~STM32SystemInterface()
{
  stop_safety_node();
  bool send_stop = false;
  {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    send_stop = activation_safety_.disarm();
  }
  if (active_ && send_stop) {
    (void)send_stop_frame();
  }
  active_ = false;
  close_serial();
}

hardware_interface::CallbackReturn STM32SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto & parameters = info_.hardware_parameters;
    if (parameters.count("serial_device")) {
      serial_device_ = parameters.at("serial_device");
    }
    if (parameters.count("baud_rate")) {
      baud_rate_ = std::stoi(parameters.at("baud_rate"));
    }
    if (parameters.count("mock_serial")) {
      mock_serial_ = parse_bool(parameters.at("mock_serial"));
    }
    if (parameters.count("command_rate_hz")) {
      command_rate_hz_ = std::stod(parameters.at("command_rate_hz"));
    }
    if (parameters.count("calibration_file")) {
      calibration_file_ = parameters.at("calibration_file");
    }
    if (parameters.count("serial_response_timeout_ms")) {
      serial_response_timeout_ms_ = std::stoi(parameters.at("serial_response_timeout_ms"));
    }
    if (parameters.count("require_calibration_match")) {
      require_calibration_match_ = parse_bool(parameters.at("require_calibration_match"));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kChannelCount) {
    RCLCPP_ERROR(kLogger, "Expected 6 joints, received %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (command_rate_hz_ <= 0.0 || command_rate_hz_ > 50.0) {
    RCLCPP_ERROR(kLogger, "command_rate_hz must be in (0, 50], got %.3f", command_rate_hz_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (serial_response_timeout_ms_ <= 0) {
    RCLCPP_ERROR(
      kLogger, "serial_response_timeout_ms must be positive, got %d", serial_response_timeout_ms_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // write() blocks on the ACK inside the real-time loop, so the window is
  // bounded by the control period by construction: a longer one cannot be
  // waited out without overrunning the cycle it belongs to. Refuse the
  // configuration rather than silently shipping an unmeetable deadline.
  const double command_period_ms = 1000.0 / command_rate_hz_;
  if (static_cast<double>(serial_response_timeout_ms_) > command_period_ms) {
    RCLCPP_ERROR(
      kLogger,
      "serial_response_timeout_ms (%d) exceeds the %.1f ms command period; the ACK cannot be "
      "awaited without overrunning the control cycle. Lower the timeout or command_rate_hz.",
      serial_response_timeout_ms_, command_period_ms);
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(kLogger, "%s must expose one position command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        kLogger, "%s must expose position and velocity state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (calibration_file_.empty() || !load_calibration(calibration_file_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  ActivationSafety::LimitsArray safety_limits{};
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    safety_limits[index] = {
      calibration_[index].limit_min_rad - calibration_[index].zero_offset_rad,
      calibration_[index].limit_max_rad - calibration_[index].zero_offset_rad,
      calibration_[index].max_velocity_rad_s};
  }
  std::string safety_error;
  if (!activation_safety_.configure(
      safety_limits, calibration_complete_, commissioning_range_rad_, safety_error))
  {
    RCLCPP_ERROR(kLogger, "Invalid activation safety configuration: %s", safety_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_commands_.fill(0.0);
  position_states_.fill(0.0);
  velocity_states_.fill(0.0);
  commanded_position_states_.fill(0.0);
  // No encoders on this arm yet, so read() echoes the delivered command. Say so
  // in a channel a consumer can read, instead of leaving /joint_states looking
  // like a measurement it is not. read() raises this the day a sensor fills
  // position_states_.
  position_is_measured_ = 0.0;
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    last_pulse_us_[index] = position_to_microseconds(index, 0.0);
  }

  // Resolve the optional `commanded` GPIO against the description. Unknown
  // names are rejected here rather than ignored: a typo in the URDF would
  // otherwise produce a silently missing interface, which is the failure mode
  // this whole split exists to prevent.
  gpio_bindings_.clear();
  for (const auto & gpio : info_.gpios) {
    for (const auto & state_interface : gpio.state_interfaces) {
      const std::string full_name = gpio.name + "/" + state_interface.name;
      double * target = nullptr;
      if (state_interface.name == "position_is_measured") {
        target = &position_is_measured_;
      } else {
        for (std::size_t index = 0; index < kChannelCount; ++index) {
          if (state_interface.name == info_.joints[index].name + "_position") {
            target = &commanded_position_states_[index];
            break;
          }
        }
      }
      if (target == nullptr) {
        RCLCPP_ERROR(
          kLogger,
          "GPIO state interface '%s' is not one this interface provides. Expected "
          "'<joint>_position' for one of the six joints, or 'position_is_measured'.",
          full_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      gpio_bindings_.push_back({full_name, target});
    }
  }

  RCLCPP_INFO(
    kLogger,
    "Initialized STM32 interface: device=%s baud=%d mock=%s rate=%.1f Hz ack_timeout=%d ms",
    serial_device_.c_str(), baud_rate_, mock_serial_ ? "true" : "false", command_rate_hz_,
    serial_response_timeout_ms_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
STM32SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kChannelCount * 2 + gpio_bindings_.size());
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &position_states_[index]);
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[index]);
  }
  // The commanded/measured split, exported only when the description asks for
  // it. on_init() resolved the names, so an unknown interface has already been
  // rejected there rather than silently exported against a dangling pointer.
  for (const auto & gpio : info_.gpios) {
    for (const auto & state_interface : gpio.state_interfaces) {
      const auto binding = std::find_if(
        gpio_bindings_.begin(), gpio_bindings_.end(),
        [&](const GpioBinding & candidate) {
          return candidate.interface_name == gpio.name + "/" + state_interface.name;
        });
      if (binding != gpio_bindings_.end()) {
        interfaces.emplace_back(gpio.name, state_interface.name, binding->value);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
STM32SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kChannelCount);
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &position_commands_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn STM32SystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!mock_serial_ && !open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!handshake()) {
    RCLCPP_ERROR(kLogger, "STM32 protocol v1 handshake failed");
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Each activation is one acceptance run. Never mix a previous run's tail,
  // timeout or latency distribution into the next run's evidence.
  reset_ack_telemetry();
  {
    std::string reported;
    std::string failure;
    const bool answered = request_calibration_fingerprint(reported, failure);
    switch (evaluate_calibration_gate(answered, reported)) {
      case CalibrationGate::kMismatch: {
        // The firmware is enforcing different joint limits, a different zero or
        // a different channel map than the one this interface is converting
        // with. Every command computed here would be applied against numbers
        // nobody checked. Refuse before any PWM is produced.
        RCLCPP_ERROR(
          kLogger,
          "Calibration mismatch: firmware reports '%s', host loaded '%s' from %s. "
          "Regenerate the firmware config from that YAML and reflash before arming.",
          reported.c_str(), calibration_fingerprint_.c_str(), calibration_file_.c_str());
        close_serial();
        return hardware_interface::CallbackReturn::ERROR;
      }
      case CalibrationGate::kUnverified:
        if (require_calibration_match_) {
          RCLCPP_ERROR(
            kLogger,
            "Calibration fingerprint could not be verified (%s) and "
            "require_calibration_match is set. Flash a firmware that answers 'K?'.",
            failure.c_str());
          close_serial();
          return hardware_interface::CallbackReturn::ERROR;
        }
        // Cannot verify is not the same as disagrees. Say which one this is,
        // and keep the arm usable on firmware that predates the check.
        RCLCPP_WARN(
          kLogger,
          "Calibration fingerprint UNVERIFIED (%s). Host loaded '%s'. The firmware's "
          "limits are being trusted, not checked -- flash a firmware that answers "
          "'K?' and set require_calibration_match:=true.",
          failure.c_str(), calibration_fingerprint_.c_str());
        break;
      case CalibrationGate::kAgreed:
        RCLCPP_INFO(kLogger, "Calibration fingerprint agreed: %s", reported.c_str());
        break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    activation_safety_.reset_reference();
    arm_initialization_pending_ = false;
  }
  active_ = true;
  last_write_time_ = std::chrono::steady_clock::time_point{};
  if (!start_safety_node()) {
    active_ = false;
    close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    kLogger,
    "STM32 hardware interface activated DISARMED; set reference_positions then armed=true on "
    "/robot_arm_hardware_safety");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void STM32SystemInterface::release_to_safe_state(const char * cause)
{
  stop_safety_node();
  bool send_stop = false;
  {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    send_stop = activation_safety_.disarm();
    activation_safety_.reset_reference();
    arm_initialization_pending_ = false;
  }
  if (active_ && send_stop) {
    if (!send_stop_frame()) {
      RCLCPP_ERROR(
        kLogger,
        "Could not deliver the stop frame during %s; the actuators may still hold the last PWM "
        "target until the firmware watchdog expires",
        cause);
    }
  }
  active_ = false;
  close_serial();
  // Print the ACK distribution on every teardown, error path included -- that
  // is precisely the run whose timing you want to inspect afterwards.
  if (ack_exchange_count_ > 0) {
    RCLCPP_INFO(
      kLogger,
      "ACK exchange summary: %" PRIu64 " exchanges, %" PRIu64
      " timeouts, %" PRIu64
      " buffered deadline completions, max %.2f ms, mean %.2f ms, window %d ms",
      ack_exchange_count_,
      ack_timeout_count_,
      ack_buffered_completion_count_,
      static_cast<double>(ack_latency_max_.count()) / 1000.0,
      static_cast<double>(ack_latency_total_.count()) / static_cast<double>(ack_exchange_count_) /
      1000.0,
      serial_response_timeout_ms_);
    const std::string distribution = ack_latency_distribution_summary();
    RCLCPP_INFO(kLogger, "%s", distribution.c_str());
  }
}

hardware_interface::CallbackReturn STM32SystemInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  // A write() that returns ERROR (the ESP32 ACK failure) lands here, not in
  // on_deactivate. Before this override existed the LifecycleNodeInterface
  // default ran, which does nothing: no `S` frame was sent and the interface
  // stayed armed while ros2_control reported the hardware as gone. The last
  // PWM target simply stayed latched until the firmware watchdog expired.
  RCLCPP_ERROR(
    kLogger, "Hardware error transition: disarming and sending the stop frame");
  release_to_safe_state("the error transition");
  // SUCCESS keeps the component recoverable (-> UNCONFIGURED), which matches
  // the behaviour observed in the field. Returning ERROR would finalize it and
  // require a full relaunch to retry.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STM32SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  release_to_safe_state("deactivation");
  RCLCPP_INFO(kLogger, "STM32 hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STM32SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // There is no encoder feedback, but the command sent to the actuator can lag
  // the requested trajectory because write() clamps calibration limits and
  // applies PWM slew limiting.  Publish that delivered command estimate rather
  // than the unconstrained request; otherwise JTC / MoveIt can report success
  // while the actuator is still ramping or has saturated at a limit.
  //
  // THIS IS AN ECHO, NOT A MEASUREMENT, and `position_is_measured_` stays 0.0
  // to say so. When the magnetic encoders are installed, the sensor read
  // replaces the loop below and raises that flag; the `commanded` GPIO keeps
  // carrying the delivered command, so nothing above ros2_control has to change
  // and no consumer silently starts treating an echo as feedback.
  std::lock_guard<std::mutex> lock(safety_mutex_);
  if (activation_safety_.armed()) {
    for (std::size_t index = 0; index < kChannelCount; ++index) {
      position_states_[index] = microseconds_to_position(index, last_pulse_us_[index]);
    }
  }
  velocity_states_.fill(0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STM32SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_) {
    return hardware_interface::return_type::ERROR;
  }

  std::lock_guard<std::mutex> safety_lock(safety_mutex_);
  if (!activation_safety_.armed()) {
    return hardware_interface::return_type::OK;
  }

  if (arm_initialization_pending_) {
    const auto & reference = activation_safety_.reference();
    for (std::size_t index = 0; index < kChannelCount; ++index) {
      position_commands_[index] = reference[index];
      position_states_[index] = reference[index];
      velocity_states_[index] = 0.0;
      last_pulse_us_[index] = position_to_microseconds(index, reference[index]);
      // Seed commanded from the same latch. Without this it stays at the
      // on_init zero while position jumps to the operator reference, so the
      // two channels disagree by the whole reference offset the moment the arm
      // is armed -- a tracking error that never happened, reported by the very
      // channel that exists to make tracking error observable.
      commanded_position_states_[index] =
        microseconds_to_position(index, last_pulse_us_[index]);
    }
    arm_initialization_pending_ = false;
    last_write_time_ = std::chrono::steady_clock::time_point{};
    RCLCPP_INFO(kLogger, "Armed at operator reference; first PWM target remains gated");
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto minimum_period = std::chrono::duration<double>(1.0 / command_rate_hz_);
  if (last_write_time_ != std::chrono::steady_clock::time_point{} &&
    now - last_write_time_ < minimum_period)
  {
    return hardware_interface::return_type::OK;
  }

  ActivationSafety::JointArray requested{};
  ActivationSafety::JointArray rate_limited{};
  std::copy(position_commands_.begin(), position_commands_.end(), requested.begin());
  std::string safety_error;
  double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    period_seconds = 1.0 / command_rate_hz_;
  }
  // An out-of-envelope request must never take PWM away: this arm has no
  // brakes, and returning ERROR here tears the hardware down (lifecycle ERROR
  // transition -> on_error -> stop frame), which drops the arm. Fold the
  // request into the safe range, keep driving, and make the refusal loud
  // instead.
  //
  // NOTE 2026-07-29: this reasoning used to say the teardown "sends the stop
  // frame" as though that were already true on every failure path. It was not:
  // on_error was unimplemented, so ERROR meant no stop frame at all and a
  // still-armed interface. It is true now only because on_error exists.
  ActivationSafety::JointArray safe_request{};
  if (activation_safety_.clamp_into_safe_range(requested, safe_request)) {
    RCLCPP_WARN_THROTTLE(
      kLogger, kThrottleClock, 1000,
      "Position command outside the armed safe range; holding at the boundary. "
      "Check the operator reference and the commissioning envelope.");
  }
  if (!activation_safety_.filter(safe_request, period_seconds, rate_limited, safety_error)) {
    RCLCPP_ERROR(kLogger, "Rejected unsafe position command: %s", safety_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // State is indexed by ROS joint, while the protocol payload is indexed by
  // physical STM32 channel. Keep those domains separate so the YAML channel
  // map remains authoritative even if joint order changes.
  std::array<int, kChannelCount> channel_pulse_us{};
  std::array<int, kChannelCount> delivered_joint_pulse_us{};
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    const int target = position_to_microseconds(index, rate_limited[index]);
    const int delivered = std::clamp(
      target,
      last_pulse_us_[index] - max_delta_us_per_step_,
      last_pulse_us_[index] + max_delta_us_per_step_);
    delivered_joint_pulse_us[index] = delivered;
    const auto channel_index = static_cast<std::size_t>(calibration_[index].channel - 1);
    channel_pulse_us[channel_index] = delivered;
  }

  if (!mock_serial_) {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    const auto ack_timeout = std::chrono::milliseconds(serial_response_timeout_ms_);
    // TX and ACK wait are one control-loop transaction. They must share one
    // absolute deadline; giving each phase a fresh full timeout could block
    // write() for almost two control periods while on_init() claimed the
    // configured budget fit inside one.
    const auto exchange_started = std::chrono::steady_clock::now();
    const auto exchange_deadline = exchange_started + ack_timeout;
    if (!send_line_until(position_command(channel_pulse_us), exchange_deadline)) {
      return hardware_interface::return_type::ERROR;
    }
    std::string response;
    const ReadResult outcome = read_line_until(response, exchange_deadline);
    const auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - exchange_started);

    if (outcome != ReadResult::kLine || response != "OK") {
      // Say WHICH failure this was. The 2026-07-28 field fault logged only
      // `response='O'`, which could not distinguish "MCU refused" from "the
      // ACK line was still in flight when the window closed" -- and those
      // demand opposite fixes.
      if (outcome == ReadResult::kTimeout) {
        ++ack_timeout_count_;
        RCLCPP_ERROR(
          kLogger,
          "ESP32 ACK timed out after %d ms with a partial line '%s' (%zu bytes). The link is "
          "alive but the reply did not complete inside the window; raise "
          "serial_response_timeout_ms if ack_latency_max approaches it.",
          serial_response_timeout_ms_, partial_rx_.c_str(), partial_rx_.size());
      } else if (outcome == ReadResult::kOverflow) {
        RCLCPP_ERROR(kLogger, "ESP32 ACK framing broken: no newline within the frame limit");
      } else if (outcome == ReadResult::kError) {
        RCLCPP_ERROR(kLogger, "ESP32 ACK read failed; serial link lost");
      } else if (response == "E3") {
        // The MCU's own watchdog fired: no position frame reached it for longer
        // than its 1 s window, so it announced E3 and this read picked the
        // announcement up in place of an ACK. Before this branch existed the
        // line was reported as "rejected command, response='E3'", which is the
        // opposite of what happened -- nothing was rejected, the link went
        // stale -- and it said nothing about the arm's physical state.
        RCLCPP_ERROR(
          kLogger,
          "ESP32 watchdog fired: no position frame reached the MCU inside its window. "
          "TORQUE IS STILL LATCHED at the last commanded pulse -- the firmware does not "
          "cut PWM on a dead link, because this arm has no brakes and cutting it would "
          "drop the arm (docs/watchdog_policy_proposal.md, policy C). The error "
          "transition that follows disarms and sends the stop frame, which DOES cut PWM: "
          "support the arm before it lands.");
      } else {
        RCLCPP_ERROR(kLogger, "ESP32 rejected command, response='%s'", response.c_str());
      }
      // Resynchronize before failing closed. The late tail of this reply (the
      // 'K\n' of a split 'OK\n') is still inbound; leaving it queued would make
      // the NEXT exchange read a stale ACK that belongs to the previous
      // command -- a silent off-by-one that no longer looks like an error.
      const std::size_t discarded = drain_serial(std::chrono::milliseconds(5));
      if (discarded > 0) {
        RCLCPP_WARN(
          kLogger, "Discarded %zu late ACK byte(s) to resynchronize the serial stream",
          discarded);
      }
      RCLCPP_ERROR(
        kLogger, "ACK telemetry: %" PRIu64 " exchanges, %" PRIu64
        " timeouts, %" PRIu64 " buffered deadline completions, max latency %.2f ms",
        ack_exchange_count_,
        ack_timeout_count_,
        ack_buffered_completion_count_,
        static_cast<double>(ack_latency_max_.count()) / 1000.0);
      return hardware_interface::return_type::ERROR;
    }

    if (last_read_completed_after_deadline_) {
      ++ack_buffered_completion_count_;
      RCLCPP_WARN_THROTTLE(
        kLogger, kThrottleClock, 30000,
        "ESP32 ACK completed from bytes already buffered when the %d ms wait deadline expired; "
        "no grace wait was added (%" PRIu64 " recoveries)",
        serial_response_timeout_ms_, ack_buffered_completion_count_);
    }
    record_ack_latency(latency);
    // The fault is intermittent (~8000 clean exchanges on 2026-07-29, two
    // failures inside ~500 the day before), so the healthy-path distribution is
    // the evidence needed to pick the timeout. Log it periodically, not just on
    // failure.
    RCLCPP_INFO_THROTTLE(
      kLogger, kThrottleClock, 30000,
      "ACK exchange latency: last %.2f ms, max %.2f ms, mean %.2f ms over %" PRIu64
      " exchanges, %" PRIu64 " buffered deadline completions (window %d ms)",
      static_cast<double>(latency.count()) / 1000.0,
      static_cast<double>(ack_latency_max_.count()) / 1000.0,
      ack_exchange_count_ > 0 ?
      static_cast<double>(ack_latency_total_.count()) / static_cast<double>(ack_exchange_count_) /
      1000.0 : 0.0,
      ack_exchange_count_, ack_buffered_completion_count_, serial_response_timeout_ms_);
  } else {
    ++mock_position_frame_count_;
  }

  activation_safety_.accept(rate_limited);
  last_pulse_us_ = delivered_joint_pulse_us;
  // What we believe we actually delivered, kept apart from whatever
  // position_states_ carries. Today the two agree by construction -- read()
  // echoes this same value -- and that is precisely the point: the agreement is
  // visible instead of implied. When encoders arrive the two diverge, and the
  // difference between them is the tracking error that has never been
  // observable on this arm.
  for (std::size_t index = 0; index < kChannelCount; ++index) {
    commanded_position_states_[index] =
      microseconds_to_position(index, delivered_joint_pulse_us[index]);
  }
  last_write_time_ = now;
  return hardware_interface::return_type::OK;
}

bool STM32SystemInterface::load_calibration(const std::string & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node channels = root["channels"];
    if (!channels || !channels.IsSequence() || channels.size() != kChannelCount) {
      throw std::runtime_error("channels must contain exactly 6 entries");
    }

    std::array<bool, kChannelCount> assigned{};
    std::array<bool, kChannelCount> assigned_channels{};
    for (const auto & node : channels) {
      ChannelCalibration value;
      value.channel = node["channel"].as<int>();
      value.joint = node["joint"].as<std::string>();
      value.min_rad = node["min_rad"].as<double>();
      value.max_rad = node["max_rad"].as<double>();
      value.min_us = node["min_us"].as<int>();
      value.max_us = node["max_us"].as<int>();
      value.zero_offset_rad = node["zero_offset_rad"].as<double>(0.0);
      value.max_velocity_rad_s = node["max_velocity_rad_s"].as<double>(0.1);
      value.invert = node["invert"].as<bool>(false);
      // Guvenlik limiti belirtilmemisse olcek capasina duser.
      value.limit_min_rad = node["limit_min_rad"].as<double>(value.min_rad);
      value.limit_max_rad = node["limit_max_rad"].as<double>(value.max_rad);
      if (value.channel < 1 || value.channel > static_cast<int>(kChannelCount) ||
        value.min_rad >= value.max_rad || value.min_us >= value.max_us ||
        !std::isfinite(value.max_velocity_rad_s) || value.max_velocity_rad_s <= 0.0)
      {
        throw std::runtime_error("invalid range/channel for " + value.joint);
      }
      if (value.limit_min_rad >= value.limit_max_rad ||
        value.limit_min_rad < value.min_rad || value.limit_max_rad > value.max_rad)
      {
        throw std::runtime_error(
                "limit_min_rad/limit_max_rad must be a non-empty subrange of "
                "min_rad/max_rad for " + value.joint);
      }
      const auto joint = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&value](const auto & item) {return item.name == value.joint;});
      if (joint == info_.joints.end()) {
        throw std::runtime_error("calibration contains unknown joint " + value.joint);
      }
      const auto index = static_cast<std::size_t>(std::distance(info_.joints.begin(), joint));
      if (assigned[index]) {
        throw std::runtime_error("duplicate calibration for " + value.joint);
      }
      const auto channel_index = static_cast<std::size_t>(value.channel - 1);
      if (assigned_channels[channel_index]) {
        throw std::runtime_error("duplicate physical channel " + std::to_string(value.channel));
      }
      calibration_[index] = value;
      assigned[index] = true;
      assigned_channels[channel_index] = true;
    }
    if (std::find(assigned.begin(), assigned.end(), false) != assigned.end()) {
      throw std::runtime_error("one or more hardware joints have no calibration");
    }

    const YAML::Node soft_start = root["soft_start"];
    if (soft_start) {
      max_delta_us_per_step_ = soft_start["max_delta_us_per_step"].as<int>(10);
      step_period_ms_ = soft_start["step_period_ms"].as<int>(20);
    }
    if (max_delta_us_per_step_ <= 0 || step_period_ms_ <= 0) {
      throw std::runtime_error("soft_start values must be positive");
    }

    const YAML::Node activation = root["activation_safety"];
    if (activation) {
      calibration_complete_ = activation["calibration_complete"].as<bool>(false);
      commissioning_range_rad_ = activation["commissioning_range_rad"].as<double>(0.3);
    }
    if (!std::isfinite(commissioning_range_rad_) || commissioning_range_rad_ <= 0.0) {
      throw std::runtime_error("activation_safety.commissioning_range_rad must be positive");
    }

    calibration_fingerprint_ = calibration_fingerprint(calibration_);
    if (calibration_fingerprint_.empty()) {
      throw std::runtime_error("could not fingerprint the calibration channel map");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Failed to load calibration '%s': %s", path.c_str(), error.what());
    return false;
  }
  return true;
}

int STM32SystemInterface::position_to_microseconds(
  const std::size_t joint_index, const double position) const
{
  const auto & value = calibration_.at(joint_index);
  // Once GUVENLIK limitine kirp, sonra OLCEK capasiyla esle. Ikisi ayri:
  // kirpma araligini daraltmak us eslemesini kaydirmamali.
  const double calibrated = std::clamp(
    position + value.zero_offset_rad, value.limit_min_rad, value.limit_max_rad);
  double ratio = (calibrated - value.min_rad) / (value.max_rad - value.min_rad);
  if (value.invert) {
    ratio = 1.0 - ratio;
  }
  return static_cast<int>(std::lround(value.min_us + ratio * (value.max_us - value.min_us)));
}

std::string STM32SystemInterface::calibration_canonical(
  const std::array<ChannelCalibration, 6> & calibration)
{
  // Radians are quantised to micro-radians before hashing. Both sides read the
  // same decimal literals out of the same YAML into IEEE-754 doubles, so the
  // quantisation is exact on both -- but it also keeps the fingerprint from
  // depending on the last bit of a float, which is what would turn a harmless
  // reformat into a refused activation in the field.
  auto quantise = [](const double radians) {
      return static_cast<long long>(std::llround(radians * 1.0e6));
    };

  std::array<const ChannelCalibration *, 6> by_channel{};
  for (const auto & entry : calibration) {
    const auto index = static_cast<std::size_t>(entry.channel - 1);
    if (index >= by_channel.size()) {
      return {};
    }
    by_channel[index] = &entry;
  }

  std::ostringstream canonical;
  for (const auto * entry : by_channel) {
    if (entry == nullptr) {
      return {};
    }
    canonical << entry->channel << ':' << entry->joint << ':'
              << quantise(entry->min_rad) << ':' << quantise(entry->max_rad) << ':'
              << quantise(entry->limit_min_rad) << ':' << quantise(entry->limit_max_rad) << ':'
              << quantise(entry->zero_offset_rad) << ':'
              << entry->min_us << ':' << entry->max_us << ':'
              << (entry->invert ? 1 : 0) << ';';
  }

  return canonical.str();
}

std::string STM32SystemInterface::calibration_fingerprint(
  const std::array<ChannelCalibration, 6> & calibration)
{
  const std::string canonical = calibration_canonical(calibration);
  if (canonical.empty()) {
    return {};
  }
  // FNV-1a 64. This guards against an accidental mismatch, not an adversary,
  // so it needs no crypto dependency on either side -- and the ESP32 has to be
  // able to compute the same thing without one.
  // Hex, not decimal: the offset basis is 14695981039346656037 and a dropped
  // digit still compiles, still hashes, and still looks right -- it just stops
  // matching the generator. That is exactly what happened while writing this.
  std::uint64_t hash = 0xCBF29CE484222325ULL;
  for (const char character : canonical) {
    hash ^= static_cast<std::uint8_t>(character);
    hash *= 0x100000001B3ULL;
  }
  std::ostringstream hex;
  hex << std::hex << std::setw(16) << std::setfill('0') << hash;
  return hex.str();
}

double STM32SystemInterface::microseconds_to_position(
  const std::size_t joint_index, const int pulse_us) const
{
  const auto & value = calibration_.at(joint_index);
  const int clamped_pulse = std::clamp(pulse_us, value.min_us, value.max_us);
  double ratio = static_cast<double>(clamped_pulse - value.min_us) /
    static_cast<double>(value.max_us - value.min_us);
  if (value.invert) {
    ratio = 1.0 - ratio;
  }
  const double calibrated = value.min_rad + ratio * (value.max_rad - value.min_rad);
  return std::clamp(
    calibrated - value.zero_offset_rad, value.min_rad - value.zero_offset_rad,
    value.max_rad - value.zero_offset_rad);
}

bool STM32SystemInterface::open_serial()
{
  close_serial();
  serial_fd_ = ::open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(kLogger, "Cannot open %s: %s", serial_device_.c_str(), std::strerror(errno));
    return false;
  }
  // The USB link (2026-08-19, see firmware/esp32_servo_ctrl/README.md) resets
  // the ESP32 on every open -- the CP2102's DTR/RTS pulse the EN pin. On a
  // brakeless arm that means a second process opening this device WHILE this
  // interface is holding it armed drops whatever pose PWM was holding. TIOCEXCL
  // makes the kernel refuse any *other* process's open() on this device for as
  // long as this fd stays open, so that reset can only happen at a moment this
  // interface itself controls (here, or in close_serial()) -- not from a stray
  // debug tool or a second launch reusing the port. Released automatically when
  // serial_fd_ is closed, including on this process's own crash.
  if (::ioctl(serial_fd_, TIOCEXCL) != 0) {
    RCLCPP_ERROR(
      kLogger, "TIOCEXCL failed on %s: %s -- refusing to open a serial link "
      "another process could still reopen while armed",
      serial_device_.c_str(), std::strerror(errno));
    close_serial();
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(kLogger, "tcgetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }
  try {
    const speed_t speed = baud_constant(baud_rate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "%s", error.what());
    close_serial();
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(kLogger, "tcsetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void STM32SystemInterface::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool STM32SystemInterface::send_line(
  const std::string & line, const std::chrono::milliseconds timeout)
{
  return send_line_until(line, std::chrono::steady_clock::now() + timeout);
}

bool STM32SystemInterface::send_line_until(
  const std::string & line, const std::chrono::steady_clock::time_point deadline)
{
  if (serial_fd_ < 0) {
    return false;
  }
  // The fd is O_NONBLOCK, so a full TX buffer returns EAGAIN. The previous
  // version retried with no poll and no deadline, i.e. an unbounded hot loop
  // inside the real-time write() path -- on the Nano that burns a core the
  // control loop shares. Every wait is now bounded and observable.
  std::size_t offset = 0;
  while (offset < line.size()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      RCLCPP_ERROR(
        kLogger, "Serial write timed out after %zu/%zu bytes", offset, line.size());
      return false;
    }
    const ssize_t written = ::write(serial_fd_, line.data() + offset, line.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno != EINTR && errno != EAGAIN) {
      RCLCPP_ERROR(kLogger, "Serial write failed: %s", std::strerror(errno));
      return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    pollfd descriptor{serial_fd_, POLLOUT, 0};
    const int result = ::poll(&descriptor, 1, std::max(1, static_cast<int>(remaining.count())));
    if (result < 0 && errno != EINTR) {
      RCLCPP_ERROR(kLogger, "Serial write poll failed: %s", std::strerror(errno));
      return false;
    }
    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      RCLCPP_ERROR(kLogger, "Serial device hung up during write");
      return false;
    }
  }
  return true;
}

STM32SystemInterface::ReadResult STM32SystemInterface::read_line(
  std::string & line, const std::chrono::milliseconds timeout)
{
  return read_line_until(line, std::chrono::steady_clock::now() + timeout);
}

STM32SystemInterface::ReadResult STM32SystemInterface::read_line_until(
  std::string & line, const std::chrono::steady_clock::time_point deadline)
{
  line.clear();
  partial_rx_.clear();
  last_read_completed_after_deadline_ = false;
  if (serial_fd_ < 0) {
    return ReadResult::kError;
  }
  // Frame limit. Previously an overlong line was silently truncated, which
  // turns a framing fault into a plausible-looking wrong answer.
  constexpr std::size_t kMaxLineLength = 127;
  for (;; ) {
    const auto now = std::chrono::steady_clock::now();
    const bool deadline_expired = now >= deadline;
    int poll_timeout_ms = 0;
    if (!deadline_expired) {
      const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      poll_timeout_ms = std::max(1, static_cast<int>(remaining.count()));
    }

    // The deadline bounds how long this function may WAIT. It must not make
    // bytes already queued in the kernel disappear. On the Nano the thread
    // read 'O', resumed after 20 ms, and returned timeout before checking the
    // immediately available "K\n" tail. Once the deadline has expired poll
    // with a zero timeout: consume only bytes already readable, never add a
    // grace period. A genuinely incomplete reply still times out fail-closed.
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, poll_timeout_ms);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(kLogger, "Serial read poll failed: %s", std::strerror(errno));
      partial_rx_ = line;
      line.clear();
      return ReadResult::kError;
    }
    if (result == 0) {
      if (deadline_expired || std::chrono::steady_clock::now() >= deadline) {
        partial_rx_ = line;
        line.clear();
        return ReadResult::kTimeout;
      }
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLNVAL)) != 0) {
      partial_rx_ = line;
      line.clear();
      return ReadResult::kError;
    }
    if ((descriptor.revents & POLLIN) == 0) {
      // POLLHUP with no data left: the device is gone. Fail fast instead of
      // spinning on a dead fd until the deadline.
      if ((descriptor.revents & POLLHUP) != 0) {
        partial_rx_ = line;
        line.clear();
        return ReadResult::kError;
      }
      continue;
    }
    char byte = 0;
    const ssize_t count = ::read(serial_fd_, &byte, 1);
    if (count == 0) {
      partial_rx_ = line;
      line.clear();
      return ReadResult::kError;
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      RCLCPP_ERROR(kLogger, "Serial read failed: %s", std::strerror(errno));
      partial_rx_ = line;
      line.clear();
      return ReadResult::kError;
    }
    if (byte == '\n') {
      last_read_completed_after_deadline_ =
        std::chrono::steady_clock::now() >= deadline;
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      return ReadResult::kLine;
    }
    if (line.size() >= kMaxLineLength) {
      partial_rx_ = line;
      line.clear();
      return ReadResult::kOverflow;
    }
    line.push_back(byte);
  }
}

std::size_t STM32SystemInterface::drain_serial(const std::chrono::milliseconds grace)
{
  if (serial_fd_ < 0) {
    return 0;
  }
  std::size_t discarded = 0;
  const auto deadline = std::chrono::steady_clock::now() + grace;
  for (;; ) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    pollfd descriptor{serial_fd_, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, std::max(1, static_cast<int>(remaining.count())));
    if (result <= 0 || (descriptor.revents & POLLIN) == 0) {
      break;
    }
    std::array<char, 64> scratch{};
    const ssize_t count = ::read(serial_fd_, scratch.data(), scratch.size());
    if (count <= 0) {
      break;
    }
    discarded += static_cast<std::size_t>(count);
  }
  partial_rx_.clear();
  return discarded;
}

void STM32SystemInterface::reset_ack_telemetry()
{
  ack_exchange_count_ = 0;
  ack_timeout_count_ = 0;
  ack_buffered_completion_count_ = 0;
  ack_latency_max_ = std::chrono::microseconds{0};
  ack_latency_total_ = std::chrono::microseconds{0};
  ack_latency_histogram_.fill(0);
}

void STM32SystemInterface::record_ack_latency(const std::chrono::microseconds latency)
{
  static_assert(
    kAckLatencyHistogramBinCount == kAckLatencyUpperBoundsUs.size() + 1U,
    "latency histogram needs one overflow bucket");
  ++ack_exchange_count_;
  ack_latency_total_ += latency;
  ack_latency_max_ = std::max(ack_latency_max_, latency);
  const auto bucket = std::lower_bound(
    kAckLatencyUpperBoundsUs.begin(), kAckLatencyUpperBoundsUs.end(), latency.count());
  const auto bucket_index = static_cast<std::size_t>(
    std::distance(kAckLatencyUpperBoundsUs.begin(), bucket));
  ++ack_latency_histogram_[bucket_index];
}

std::string STM32SystemInterface::ack_latency_distribution_summary() const
{
  if (ack_exchange_count_ == 0) {
    return "ACK exchange distribution: no successful exchanges";
  }

  const auto percentile_label = [this](const std::uint64_t percentile) {
      const std::uint64_t target_rank =
        (ack_exchange_count_ * percentile + 99U) / 100U;
      std::uint64_t cumulative = 0;
      for (std::size_t index = 0; index < ack_latency_histogram_.size(); ++index) {
        cumulative += ack_latency_histogram_[index];
        if (cumulative < target_rank) {
          continue;
        }
        std::ostringstream label;
        label << std::fixed << std::setprecision(2);
        if (index < kAckLatencyUpperBoundsUs.size()) {
          label << "<="
                << static_cast<double>(kAckLatencyUpperBoundsUs[index]) / 1000.0
                << "ms";
        } else {
          label << ">"
                << static_cast<double>(kAckLatencyUpperBoundsUs.back()) / 1000.0
                << "ms";
        }
        return label.str();
      }
      return std::string{"unavailable"};
    };

  std::ostringstream summary;
  summary << "ACK exchange distribution: p50" << percentile_label(50U)
          << " p95" << percentile_label(95U)
          << " p99" << percentile_label(99U)
          << " bucket_upper_us=[";
  for (std::size_t index = 0; index < ack_latency_histogram_.size(); ++index) {
    if (index > 0) {
      summary << ',';
    }
    if (index < kAckLatencyUpperBoundsUs.size()) {
      summary << kAckLatencyUpperBoundsUs[index];
    } else {
      summary << "+inf";
    }
    summary << ':' << ack_latency_histogram_[index];
  }
  summary << ']';
  return summary.str();
}

bool STM32SystemInterface::handshake()
{
  if (mock_serial_) {
    RCLCPP_INFO(kLogger, "Mock serial handshake: V1,mock");
    return true;
  }
  // The handshake window stays generous (1 s): it runs once, outside the
  // control loop, and the ESP32 may still be booting.
  constexpr auto kHandshakeTimeout = std::chrono::milliseconds(1000);
  if (!send_line("V?\n", kHandshakeTimeout)) {
    return false;
  }
  std::string response;
  if (read_line(response, kHandshakeTimeout) != ReadResult::kLine) {
    RCLCPP_ERROR(
      kLogger, "No version line from the MCU within %" PRId64 " ms (partial '%s')",
      static_cast<std::int64_t>(kHandshakeTimeout.count()), partial_rx_.c_str());
    return false;
  }
  return response.rfind("V1,", 0) == 0;
}

STM32SystemInterface::CalibrationGate STM32SystemInterface::evaluate_calibration_gate(
  const bool answered, const std::string & reported) const
{
  if (!answered) {
    return CalibrationGate::kUnverified;
  }
  // An answer that is present but empty is not agreement -- it is a firmware
  // that replied `K,` with nothing after it. Treat it as unverifiable rather
  // than letting an empty string compare equal to an empty host fingerprint.
  if (reported.empty()) {
    return CalibrationGate::kUnverified;
  }
  return reported == calibration_fingerprint_ ?
         CalibrationGate::kAgreed : CalibrationGate::kMismatch;
}

bool STM32SystemInterface::request_calibration_fingerprint(
  std::string & reported, std::string & failure)
{
  reported.clear();
  failure.clear();
  if (mock_serial_) {
    // Mock stands in for a firmware that agrees, so the check does not turn
    // every mock bring-up into a warning.
    reported = calibration_fingerprint_;
    return true;
  }
  constexpr auto kFingerprintTimeout = std::chrono::milliseconds(1000);
  if (!send_line("K?\n", kFingerprintTimeout)) {
    failure = "could not send the fingerprint request";
    return false;
  }
  std::string response;
  const ReadResult outcome = read_line(response, kFingerprintTimeout);
  if (outcome != ReadResult::kLine) {
    failure = "no reply within " + std::to_string(kFingerprintTimeout.count()) + " ms";
    // Whatever half-line did arrive must not be left for the next exchange to
    // read as its own ACK.
    (void)drain_serial(std::chrono::milliseconds(50));
    return false;
  }
  if (response.rfind("K,", 0) != 0) {
    // An older firmware answers with a parse error rather than a fingerprint.
    // That is "cannot verify", not "disagrees", and the caller separates them.
    failure = "firmware replied '" + response + "'";
    return false;
  }
  reported = response.substr(2);
  return true;
}

std::string STM32SystemInterface::position_command(
  const std::array<int, 6> & pulse_us) const
{
  std::ostringstream stream;
  stream << 'P';
  for (std::size_t index = 0; index < pulse_us.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << pulse_us[index];
  }
  stream << '\n';
  return stream.str();
}

bool STM32SystemInterface::start_safety_node()
{
  try {
    safety_node_ = std::make_shared<rclcpp::Node>("robot_arm_hardware_safety");
    safety_node_->declare_parameter<std::vector<double>>(
      "reference_positions", std::vector<double>{});
    safety_node_->declare_parameter<bool>("armed", false);
    safety_parameter_callback_ = safety_node_->add_on_set_parameters_callback(
      std::bind(&STM32SystemInterface::on_safety_parameters, this, std::placeholders::_1));
    safety_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    safety_executor_->add_node(safety_node_);
    safety_spin_returned_.store(false);
    safety_executor_thread_ = std::thread(
      [this, executor = safety_executor_]() {
        try {
          executor->spin();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(kLogger, "Safety parameter executor stopped: %s", error.what());
        }
        // Must be the last thing the thread does, and must happen on every exit
        // path: stop_safety_node() waits on it before joining.
        safety_spin_returned_.store(true);
      });
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Failed to start activation safety parameter node: %s", error.what());
    stop_safety_node();
    return false;
  }
  return true;
}

void STM32SystemInterface::stop_safety_node()
{
  if (safety_executor_ && safety_executor_thread_.joinable()) {
    // A single cancel() is not enough. rclcpp's cancel() clears the executor's
    // `spinning` flag and trips the interrupt guard condition; spin() SETS that
    // flag on entry. So a cancel issued before the new thread reaches spin() is
    // erased by spin() itself, the thread then waits for work that never comes,
    // and this join() blocks forever.
    //
    // Measured 2026-08-15: this hung roughly one run in ten, and the test that
    // hung most often was the one with the least code between on_activate() and
    // the destructor -- the shortest possible race window. It surfaced as a 60 s
    // ctest timeout and a missing result file, not as a failure.
    //
    // Re-issuing cancel() until the thread reports that spin() returned closes
    // the window from the other side: once spin() is running, one of these
    // lands.
    const auto complained_at = std::chrono::steady_clock::now();
    bool complained = false;
    while (!safety_spin_returned_.load()) {
      safety_executor_->cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (!complained &&
        std::chrono::steady_clock::now() - complained_at > std::chrono::seconds(2))
      {
        complained = true;
        RCLCPP_ERROR(
          kLogger,
          "Safety parameter executor has ignored cancel() for 2 s; still retrying. "
          "This should not happen -- report it rather than treating it as slow shutdown.");
      }
    }
  }
  if (safety_executor_thread_.joinable()) {
    safety_executor_thread_.join();
  }
  safety_parameter_callback_.reset();
  safety_node_.reset();
  safety_executor_.reset();
}

bool STM32SystemInterface::send_stop_frame()
{
  if (mock_serial_) {
    ++mock_stop_frame_count_;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(serial_mutex_);
  // The stop frame is the last thing that reaches the MCU on a fault path, so
  // it gets its own generous window rather than the control-loop ACK budget.
  if (!send_line("S\n", std::chrono::milliseconds(200))) {
    RCLCPP_ERROR(kLogger, "Failed to send disarm stop frame");
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult STM32SystemInterface::on_safety_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  bool send_stop = false;
  {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    ActivationSafety candidate = activation_safety_;
    bool desired_armed = candidate.armed();
    bool reference_supplied = false;
    ActivationSafety::JointArray reference{};

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "armed") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.reason = "armed must be a bool";
          return result;
        }
        desired_armed = parameter.as_bool();
      } else if (parameter.get_name() == "reference_positions") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          result.reason = "reference_positions must be a double array";
          return result;
        }
        const auto values = parameter.as_double_array();
        if (values.size() != kChannelCount) {
          result.reason = "reference_positions must contain exactly 6 values";
          return result;
        }
        std::copy(values.begin(), values.end(), reference.begin());
        reference_supplied = true;
      }
    }

    std::string reason;
    const bool was_armed = candidate.armed();
    if (!desired_armed) {
      if (was_armed && reference_supplied) {
        result.reason = "disarm first, then measure and set a fresh reference_positions value";
        return result;
      }
      (void)candidate.disarm();
      if (reference_supplied && !candidate.set_reference(reference, reason)) {
        result.reason = reason;
        return result;
      }
    } else {
      if (was_armed && reference_supplied) {
        result.reason = "disarm before changing reference_positions";
        return result;
      }
      if (!was_armed) {
        if (reference_supplied && !candidate.set_reference(reference, reason)) {
          result.reason = reason;
          return result;
        }
        if (!candidate.arm(reason)) {
          result.reason = reason;
          return result;
        }
      }
    }

    activation_safety_ = candidate;
    if (!was_armed && candidate.armed()) {
      arm_initialization_pending_ = true;
    }
    send_stop = was_armed && !candidate.armed();
  }

  if (send_stop && !send_stop_frame()) {
    RCLCPP_ERROR(
      kLogger,
      "Hardware remains logically disarmed; physical stop frame failed and firmware "
      "watchdog must expire");
  }
  result.successful = true;
  result.reason = send_stop ? "disarmed" : "activation safety state updated";
  return result;
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(
  arm_hardware::STM32SystemInterface, hardware_interface::SystemInterface)
