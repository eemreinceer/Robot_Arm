#pragma once

// Servo-rail ADC wiring for Faz 7 / A1. Measurement only — nothing here
// affects PWM, motion or the locked UART v1 contract.
//
// Servo-rail divider wiring:
//
//   servo connector V+ ──[ kRailDividerHighOhm ]──┬── kRailAdcPin
//                                                 │
//                                        [ kRailDividerLowOhm ]
//                                                 │
//   servo connector GND ───────────────────────────┴── ESP32 GND (common)
//
// 20k/10k divides by 3: a 6.0 V rail lands at 2.0 V, inside the ADC's usable
// span at 11 dB attenuation. The divider draws ~200 uA, which is negligible
// next to the servos and cannot itself cause the sag being measured.
//
// PROBE THE SERVO CONNECTOR, NOT THE BUCK OUTPUT. The buck can read 5.5 V
// while the servo sees less because of wiring or connector loss.
//
// Deliberately NO filter capacitor on the node: the transient is the signal.

#include "rail_monitor.hpp"

#include <cstdint>

namespace robot_arm {

// ADC1 only. ADC2 shares hardware with WiFi and stops converting when the
// radio is up. GPIO34 is input-only with no internal pull-ups, which is what
// an analog input wants.
constexpr int kRailAdcPin = 34;

constexpr uint32_t kRailDividerHighOhm = 20000U;
constexpr uint32_t kRailDividerLowOhm = 10000U;

// 12-bit conversion at 11 dB attenuation. The scale is nominal: the ESP32 ADC
// is nonlinear near both ends, so a multimeter anchors it once at a steady
// state. Relative collapse is the quantity of interest.
constexpr uint32_t kRailAdcFullScaleMv = 3100U;
constexpr uint32_t kRailAdcMaxCount = 4095U;

// 20 kHz. Each conversion costs roughly 10 us, so a 50 us period leaves the
// sampler idle most of the time and resolves events an INA226 (>=140 us per
// conversion) cannot see.
constexpr uint32_t kRailSamplePeriodUs = 50U;

constexpr RailCalibration kRailCalibration = {
  kRailDividerHighOhm,
  kRailDividerLowOhm,
  kRailAdcFullScaleMv,
  kRailAdcMaxCount,
};

}  // namespace robot_arm
