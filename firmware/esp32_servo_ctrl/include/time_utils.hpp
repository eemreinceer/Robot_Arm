#pragma once

#include <cstdint>

namespace robot_arm {

// Unsigned subtraction handles millis() wraparound when timestamps are ordered.
// The half-range guard also rejects a timestamp sampled just before a command
// handler records a newer "last activity" value.
constexpr bool elapsedAtLeast(
  uint32_t nowMs, uint32_t sinceMs, uint32_t intervalMs)
{
  return
    (nowMs - sinceMs) < 0x80000000U &&
    (nowMs - sinceMs) >= intervalMs;
}

}  // namespace robot_arm
