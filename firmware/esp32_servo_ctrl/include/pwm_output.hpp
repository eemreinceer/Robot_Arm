#pragma once

#include <cmath>
#include <cstdint>

namespace robot_arm {

constexpr uint32_t kPwmPeriodUs = 20000UL;
constexpr uint32_t kPwmMaxDuty = 65535UL;

// Preserve the deployed direct-pulse path byte-for-byte and count-for-count.
inline uint32_t legacyPulseUsToDuty(uint16_t pulseUs) {
  return
    (static_cast<uint32_t>(pulseUs) * (kPwmMaxDuty + 1UL)) / kPwmPeriodUs;
}

// Smooth angles are continuous floats. Convert them straight to LEDC counts
// instead of first discarding the ESP32's sub-microsecond PWM resolution.
inline uint32_t smoothPulseUsToDuty(float pulseUs) {
  if (!std::isfinite(pulseUs) || pulseUs <= 0.0f) {
    return 0U;
  }
  const float duty =
    pulseUs * static_cast<float>(kPwmMaxDuty + 1UL) /
    static_cast<float>(kPwmPeriodUs);
  if (duty >= static_cast<float>(kPwmMaxDuty)) {
    return kPwmMaxDuty;
  }
  return static_cast<uint32_t>(std::lround(duty));
}

}  // namespace robot_arm
