// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace arm_hardware
{

class ActivationSafety
{
public:
  static constexpr std::size_t kJointCount = 6;
  using JointArray = std::array<double, kJointCount>;

  struct JointLimits
  {
    double min_position{0.0};
    double max_position{0.0};
    double max_velocity{0.1};
  };

  using LimitsArray = std::array<JointLimits, kJointCount>;

  bool configure(
    const LimitsArray & limits, bool calibration_complete,
    double commissioning_range, std::string & reason);
  bool set_reference(const JointArray & reference, std::string & reason);
  bool arm(std::string & reason);
  bool disarm();
  void reset_reference();

  bool filter(
    const JointArray & requested, double period_seconds,
    JointArray & delivered, std::string & reason) const;

  // Folds a request into the armed safe range instead of rejecting it. Returns
  // true when any joint had to be moved. Killing PWM on an out-of-range request
  // would drop an arm that has no brakes, so the caller clamps and warns.
  bool clamp_into_safe_range(const JointArray & requested, JointArray & clamped) const;

  void accept(const JointArray & delivered);

  bool armed() const {return armed_;}
  bool has_reference() const {return has_reference_;}
  bool calibration_complete() const {return calibration_complete_;}
  const JointArray & reference() const {return reference_;}
  const JointArray & last_command() const {return last_command_;}

private:
  bool validate_reference(const JointArray & reference, std::string & reason) const;

  LimitsArray limits_{};
  JointArray reference_{};
  JointArray last_command_{};
  bool configured_{false};
  bool calibration_complete_{false};
  bool has_reference_{false};
  bool armed_{false};
  double commissioning_range_{0.3};
};

}  // namespace arm_hardware
