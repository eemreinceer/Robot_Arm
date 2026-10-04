// Copyright 2026 Robot Arm Project Contributors
// Licensed under the Apache License, Version 2.0

#include "arm_hardware/activation_safety.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace arm_hardware
{

bool ActivationSafety::configure(
  const LimitsArray & limits, const bool calibration_complete,
  const double commissioning_range, std::string & reason)
{
  if (!std::isfinite(commissioning_range) || commissioning_range <= 0.0) {
    reason = "commissioning_range_rad must be finite and positive";
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const auto & value = limits[index];
    if (!std::isfinite(value.min_position) || !std::isfinite(value.max_position) ||
      !std::isfinite(value.max_velocity) || value.min_position >= value.max_position ||
      value.max_velocity <= 0.0)
    {
      reason = "invalid position/velocity safety limit for joint index " +
        std::to_string(index);
      return false;
    }
  }

  limits_ = limits;
  calibration_complete_ = calibration_complete;
  commissioning_range_ = commissioning_range;
  configured_ = true;
  has_reference_ = false;
  armed_ = false;
  reason.clear();
  return true;
}

bool ActivationSafety::validate_reference(
  const JointArray & reference, std::string & reason) const
{
  if (!configured_) {
    reason = "activation safety is not configured";
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(reference[index])) {
      reason = "reference contains a non-finite value at joint index " +
        std::to_string(index);
      return false;
    }
    if (reference[index] < limits_[index].min_position ||
      reference[index] > limits_[index].max_position)
    {
      reason = "reference is outside calibrated bounds at joint index " +
        std::to_string(index);
      return false;
    }
  }
  return true;
}

bool ActivationSafety::set_reference(const JointArray & reference, std::string & reason)
{
  if (armed_) {
    reason = "reference cannot change while armed";
    return false;
  }
  if (!validate_reference(reference, reason)) {
    return false;
  }
  reference_ = reference;
  last_command_ = reference;
  has_reference_ = true;
  reason.clear();
  return true;
}

bool ActivationSafety::arm(std::string & reason)
{
  if (!configured_) {
    reason = "activation safety is not configured";
    return false;
  }
  if (!has_reference_) {
    reason = "operator reference_positions must be set before arming";
    return false;
  }
  last_command_ = reference_;
  armed_ = true;
  reason.clear();
  return true;
}

bool ActivationSafety::disarm()
{
  const bool transitioned = armed_;
  reset_reference();
  return transitioned;
}

void ActivationSafety::reset_reference()
{
  armed_ = false;
  has_reference_ = false;
  reference_.fill(0.0);
  last_command_.fill(0.0);
}

bool ActivationSafety::filter(
  const JointArray & requested, const double period_seconds,
  JointArray & delivered, std::string & reason) const
{
  if (!armed_) {
    reason = "hardware is disarmed";
    return false;
  }
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    reason = "control period must be finite and positive";
    return false;
  }

  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(requested[index])) {
      reason = "command contains a non-finite value at joint index " +
        std::to_string(index);
      return false;
    }

    double safe_min = limits_[index].min_position;
    double safe_max = limits_[index].max_position;
    if (!calibration_complete_) {
      safe_min = std::max(safe_min, reference_[index] - commissioning_range_);
      safe_max = std::min(safe_max, reference_[index] + commissioning_range_);
    }
    if (requested[index] < safe_min || requested[index] > safe_max) {
      std::ostringstream message;
      message << "command " << requested[index] << " outside safe range [" <<
        safe_min << ", " << safe_max << "] at joint index " << index;
      reason = message.str();
      return false;
    }

    const double max_delta = limits_[index].max_velocity * period_seconds;
    delivered[index] = std::clamp(
      requested[index], last_command_[index] - max_delta,
      last_command_[index] + max_delta);
  }

  reason.clear();
  return true;
}

bool ActivationSafety::clamp_into_safe_range(
  const JointArray & requested, JointArray & clamped) const
{
  bool modified = false;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    double safe_min = limits_[index].min_position;
    double safe_max = limits_[index].max_position;
    if (!calibration_complete_ && has_reference_) {
      safe_min = std::max(safe_min, reference_[index] - commissioning_range_);
      safe_max = std::min(safe_max, reference_[index] + commissioning_range_);
    }

    if (!std::isfinite(requested[index])) {
      clamped[index] = last_command_[index];
      modified = true;
      continue;
    }
    clamped[index] = std::clamp(requested[index], safe_min, safe_max);
    if (clamped[index] != requested[index]) {
      modified = true;
    }
  }
  return modified;
}

void ActivationSafety::accept(const JointArray & delivered)
{
  if (armed_) {
    last_command_ = delivered;
  }
}

}  // namespace arm_hardware
