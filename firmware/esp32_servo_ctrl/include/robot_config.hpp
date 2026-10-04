#pragma once

#include <cstddef>
#include <cstdint>

namespace robot_arm {

constexpr std::size_t kServoCount = 6U;
constexpr std::size_t kMaxServoCount = 8U;
constexpr std::size_t kMotionQueueCapacity = 4U;
constexpr std::size_t kMaxLineLength = 192U;

constexpr uint32_t kServoUpdateIntervalMs = 20U;
constexpr uint32_t kCommunicationTimeoutMs = 2000U;
constexpr uint32_t kMaximumMotionDurationMs = 60000U;
constexpr uint32_t kMinimumMotionDurationMs = kServoUpdateIntervalMs;
constexpr bool kEstopDisablesPwm = false;

struct ServoCalibration {
  uint8_t pin;
  uint8_t ledcChannel;

  // Smooth protocol angles are ROS joint degrees. The zero offset converts
  // them to the calibrated servo frame before the pulse mapping is applied.
  // Safety limits are stored in the ROS joint frame.
  float calibrationMinServoAngleDeg;
  float calibrationMaxServoAngleDeg;
  float jointLimitMinAngleDeg;
  float jointLimitMaxAngleDeg;
  float zeroOffsetDeg;

  uint16_t minPulseUs;
  uint16_t maxPulseUs;

  float maxSpeedDegPerSec;
  float startupSpeedDegPerSec;
  float deadbandDeg;
  bool reversed;
};

}  // namespace robot_arm

#include "robot_config_generated.hpp"
