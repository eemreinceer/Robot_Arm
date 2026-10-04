#pragma once

// Servo-rail voltage monitor (Faz 7 / A1).
//
// This is a MEASUREMENT path only. It never commands motion, never touches
// PWM and never participates in the smooth-motion safety timeout. The arm has
// no oscilloscope on the bench, so the ESP32 ADC is the fastest instrument
// available: sampled continuously it resolves tens of microseconds, where an
// INA226 conversion takes at least 140 us.
//
// Everything in this header is platform-free so it runs under `pio test -e
// native`. The Arduino adapter in main.cpp owns the ADC and the timestamps.
//
// Absolute accuracy is deliberately not claimed. The ESP32 ADC is nonlinear
// near both rails, so a multimeter anchors the scale once at a steady state.
// What this measures is RELATIVE collapse under load.

#include <cstddef>
#include <cstdint>

namespace robot_arm {

// Number of 100 ms buckets behind the rolling minimum. Ten buckets means
// railMinRecentMv() answers "lowest rail seen in the last ~1 s" in O(1) per
// sample without storing the samples themselves.
constexpr std::size_t kRailBucketCount = 10U;
constexpr uint32_t kRailBucketIntervalUs = 100000U;

// Pre-trigger history kept for a capture. At 20 kHz this is ~102 ms before
// the event and ~102 ms after it, which is enough to show a servo inrush
// transient and still dump over 115200 baud in a couple of seconds.
constexpr std::size_t kRailCaptureSamples = 4096U;

struct RailCalibration {
  // Divider from the servo connector into the ADC pin: the measured node is
  // between highOhm (to rail) and lowOhm (to ground).
  uint32_t dividerHighOhm;
  uint32_t dividerLowOhm;
  // ADC full-scale in millivolts at the configured attenuation, and the raw
  // count corresponding to it.
  uint32_t adcFullScaleMv;
  uint32_t adcMaxCount;
};

enum class RailTriggerState : uint8_t {
  Idle = 0,     // not armed; ring buffer still records history
  Armed,        // waiting for the rail to fall below the threshold
  Capturing,    // threshold crossed, filling the post-trigger half
  Complete,     // capture frozen and ready to dump
};

// Converts a raw ADC count to millivolts at the servo connector.
uint32_t railCountToMillivolts(
  const RailCalibration & calibration, uint32_t rawCount);

class RailMonitor {
public:
  explicit RailMonitor(const RailCalibration & calibration);

  // Feeds one sample. `nowUs` must be monotonic; unsigned subtraction keeps
  // it correct across wraparound.
  void addSample(uint32_t rawCount, uint32_t nowUs);

  void reset(uint32_t nowUs);

  bool hasSample() const;
  uint32_t sampleCount() const;
  uint32_t lastMv() const;
  uint32_t minMv() const;          // lowest since the last reset
  uint32_t maxMv() const;          // highest since the last reset
  uint32_t meanMv() const;         // running mean since the last reset
  uint32_t minRecentMv() const;    // lowest within the rolling ~1 s window

  // Arms a single-shot capture. The rail falling below `thresholdMv` freezes
  // the ring buffer once the post-trigger half is full.
  void armTrigger(uint32_t thresholdMv);
  void disarmTrigger();
  RailTriggerState triggerState() const;
  uint32_t triggerThresholdMv() const;

  // Capture readout. Index 0 is the OLDEST retained sample. `triggerIndex()`
  // marks the sample that crossed the threshold.
  std::size_t captureSize() const;
  std::size_t triggerIndex() const;
  uint32_t captureRawAt(std::size_t index) const;
  uint32_t captureMvAt(std::size_t index) const;
  // Microseconds of the sample relative to the trigger sample. Negative
  // values are pre-trigger history.
  int32_t captureRelativeUsAt(std::size_t index) const;

private:
  void pushRing(uint32_t rawCount, uint32_t nowUs);
  void rotateBuckets(uint32_t nowUs);
  std::size_t ringIndex(std::size_t logicalIndex) const;

  RailCalibration calibration_;

  uint32_t sampleCount_;
  uint32_t lastRaw_;
  uint32_t minRaw_;
  uint32_t maxRaw_;
  uint64_t rawSum_;

  uint32_t bucketMinRaw_[kRailBucketCount];
  std::size_t bucketIndex_;
  uint32_t bucketStartedUs_;

  uint32_t ringRaw_[kRailCaptureSamples];
  uint32_t ringUs_[kRailCaptureSamples];
  std::size_t ringHead_;
  std::size_t ringFill_;

  RailTriggerState triggerState_;
  uint32_t triggerThresholdMv_;
  uint32_t triggerUs_;
  std::size_t postTriggerRemaining_;
  std::size_t frozenTriggerIndex_;
};

}  // namespace robot_arm
