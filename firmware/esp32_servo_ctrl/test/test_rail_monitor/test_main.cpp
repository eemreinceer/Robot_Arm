#include "rail_monitor.hpp"

#include <unity.h>

namespace {

// 20k/10k divider, 12-bit ADC, 3100 mV full scale — the wiring in
// include/rail_config.hpp.
constexpr robot_arm::RailCalibration kCalibration = {20000U, 10000U, 3100U, 4095U};

// Raw count that maps to roughly `mv` at the servo connector, for readable
// test intent.
uint32_t countForMillivolts(uint32_t mv) {
  const uint64_t nodeMv = (static_cast<uint64_t>(mv) * 10000U) / 30000U;
  return static_cast<uint32_t>((nodeMv * 4095U) / 3100U);
}

void testDividerConversion() {
  // Full scale at the node is three times that at the rail.
  TEST_ASSERT_EQUAL_UINT32(9300U, robot_arm::railCountToMillivolts(kCalibration, 4095U));
  TEST_ASSERT_EQUAL_UINT32(0U, robot_arm::railCountToMillivolts(kCalibration, 0U));
  // A count above full scale is clamped, never wrapped.
  TEST_ASSERT_EQUAL_UINT32(
    9300U, robot_arm::railCountToMillivolts(kCalibration, 99999U));
  // 6.0 V nominal round-trips within a millivolt of ADC quantisation.
  TEST_ASSERT_UINT32_WITHIN(
    10U, 6000U,
    robot_arm::railCountToMillivolts(kCalibration, countForMillivolts(6000U)));
}

void testDegenerateCalibrationDoesNotDivideByZero() {
  const robot_arm::RailCalibration zeroCounts = {20000U, 10000U, 3100U, 0U};
  const robot_arm::RailCalibration zeroLow = {20000U, 0U, 3100U, 4095U};
  TEST_ASSERT_EQUAL_UINT32(0U, robot_arm::railCountToMillivolts(zeroCounts, 100U));
  TEST_ASSERT_EQUAL_UINT32(0U, robot_arm::railCountToMillivolts(zeroLow, 100U));
}

void testStatisticsTrackMinimumAndMean() {
  robot_arm::RailMonitor monitor(kCalibration);
  monitor.reset(0U);
  TEST_ASSERT_FALSE(monitor.hasSample());
  TEST_ASSERT_EQUAL_UINT32(0U, monitor.minMv());

  const uint32_t high = countForMillivolts(6000U);
  const uint32_t low = countForMillivolts(4500U);
  monitor.addSample(high, 0U);
  monitor.addSample(low, 50U);
  monitor.addSample(high, 100U);

  TEST_ASSERT_TRUE(monitor.hasSample());
  TEST_ASSERT_EQUAL_UINT32(3U, monitor.sampleCount());
  TEST_ASSERT_UINT32_WITHIN(20U, 6000U, monitor.lastMv());
  TEST_ASSERT_UINT32_WITHIN(20U, 4500U, monitor.minMv());
  TEST_ASSERT_UINT32_WITHIN(20U, 6000U, monitor.maxMv());
  TEST_ASSERT_UINT32_WITHIN(30U, 5500U, monitor.meanMv());
}

void testRollingWindowForgetsOldSags() {
  robot_arm::RailMonitor monitor(kCalibration);
  monitor.reset(0U);

  const uint32_t nominal = countForMillivolts(6000U);
  const uint32_t sag = countForMillivolts(4000U);

  monitor.addSample(sag, 0U);
  TEST_ASSERT_UINT32_WITHIN(20U, 4000U, monitor.minRecentMv());

  // Walk past the whole ten-bucket window with a healthy rail.
  for (uint32_t step = 1U; step <= 12U; ++step) {
    monitor.addSample(nominal, step * robot_arm::kRailBucketIntervalUs);
  }

  // The rolling window has forgotten the sag...
  TEST_ASSERT_UINT32_WITHIN(20U, 6000U, monitor.minRecentMv());
  // ...but the since-reset minimum still remembers it. Losing that would hide
  // exactly the event this instrument exists to catch.
  TEST_ASSERT_UINT32_WITHIN(20U, 4000U, monitor.minMv());
}

void testTriggerCapturesAroundTheEventAndFreezes() {
  robot_arm::RailMonitor monitor(kCalibration);
  monitor.reset(0U);

  const uint32_t nominal = countForMillivolts(6000U);
  const uint32_t sag = countForMillivolts(4000U);
  uint32_t nowUs = 0U;

  monitor.armTrigger(5000U);
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Armed),
    static_cast<int>(monitor.triggerState()));

  // Pre-trigger history: healthy rail, no crossing.
  for (std::size_t index = 0U; index < 500U; ++index) {
    monitor.addSample(nominal, nowUs);
    nowUs += 50U;
  }
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Armed),
    static_cast<int>(monitor.triggerState()));

  // The event.
  const uint32_t triggerUs = nowUs;
  monitor.addSample(sag, nowUs);
  nowUs += 50U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Capturing),
    static_cast<int>(monitor.triggerState()));

  // Post-trigger fill.
  for (std::size_t index = 0U; index < robot_arm::kRailCaptureSamples; ++index) {
    monitor.addSample(nominal, nowUs);
    nowUs += 50U;
  }
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Complete),
    static_cast<int>(monitor.triggerState()));

  // The trigger sample is inside the capture, and it is the sag.
  const std::size_t triggerIndex = monitor.triggerIndex();
  TEST_ASSERT_TRUE(triggerIndex < monitor.captureSize());
  TEST_ASSERT_UINT32_WITHIN(20U, 4000U, monitor.captureMvAt(triggerIndex));
  TEST_ASSERT_EQUAL_INT32(0, monitor.captureRelativeUsAt(triggerIndex));

  // History before the event is retained and reads negative.
  TEST_ASSERT_TRUE(triggerIndex > 0U);
  TEST_ASSERT_TRUE(monitor.captureRelativeUsAt(triggerIndex - 1U) < 0);
  TEST_ASSERT_UINT32_WITHIN(
    20U, 6000U, monitor.captureMvAt(triggerIndex - 1U));

  // A completed capture is evidence: later samples must not overwrite it.
  const uint32_t frozenAtTrigger = monitor.captureRawAt(triggerIndex);
  const std::size_t frozenSize = monitor.captureSize();
  for (std::size_t index = 0U; index < 1000U; ++index) {
    monitor.addSample(sag, nowUs);
    nowUs += 50U;
  }
  TEST_ASSERT_EQUAL_UINT32(frozenAtTrigger, monitor.captureRawAt(triggerIndex));
  TEST_ASSERT_EQUAL_UINT32(frozenSize, monitor.captureSize());
  TEST_ASSERT_EQUAL_INT32(0, monitor.captureRelativeUsAt(triggerIndex));
  // Statistics keep running even while the capture is frozen.
  TEST_ASSERT_UINT32_WITHIN(20U, 4000U, monitor.lastMv());

  (void)triggerUs;
}

void testDisarmedTriggerNeverCaptures() {
  robot_arm::RailMonitor monitor(kCalibration);
  monitor.reset(0U);
  const uint32_t sag = countForMillivolts(3000U);
  for (std::size_t index = 0U; index < 100U; ++index) {
    monitor.addSample(sag, static_cast<uint32_t>(index) * 50U);
  }
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Idle),
    static_cast<int>(monitor.triggerState()));

  monitor.armTrigger(5000U);
  monitor.disarmTrigger();
  monitor.addSample(sag, 10000U);
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Idle),
    static_cast<int>(monitor.triggerState()));
}

void testTimestampWraparoundStaysOrdered() {
  robot_arm::RailMonitor monitor(kCalibration);
  const uint32_t nearWrap = 0xFFFFFF00U;
  monitor.reset(nearWrap);

  const uint32_t nominal = countForMillivolts(6000U);
  const uint32_t sag = countForMillivolts(4000U);

  monitor.armTrigger(5000U);
  uint32_t nowUs = nearWrap;
  for (std::size_t index = 0U; index < 10U; ++index) {
    monitor.addSample(nominal, nowUs);
    nowUs += 50U;  // wraps through zero partway through
  }
  monitor.addSample(sag, nowUs);
  const uint32_t afterTriggerUs = nowUs + 50U;
  monitor.addSample(nominal, afterTriggerUs);

  // Relative time either side of the wrap must stay signed-correct.
  const std::size_t size = monitor.captureSize();
  TEST_ASSERT_TRUE(size >= 12U);
  TEST_ASSERT_EQUAL_INT32(0, monitor.captureRelativeUsAt(size - 2U));
  TEST_ASSERT_EQUAL_INT32(50, monitor.captureRelativeUsAt(size - 1U));
  TEST_ASSERT_EQUAL_INT32(-50, monitor.captureRelativeUsAt(size - 3U));
}

void testResetClearsEverything() {
  robot_arm::RailMonitor monitor(kCalibration);
  monitor.reset(0U);
  monitor.armTrigger(5000U);
  monitor.addSample(countForMillivolts(3000U), 0U);
  TEST_ASSERT_TRUE(monitor.hasSample());

  monitor.reset(1000U);
  TEST_ASSERT_FALSE(monitor.hasSample());
  TEST_ASSERT_EQUAL_UINT32(0U, monitor.sampleCount());
  TEST_ASSERT_EQUAL_UINT32(0U, monitor.minMv());
  TEST_ASSERT_EQUAL_UINT32(0U, monitor.minRecentMv());
  TEST_ASSERT_EQUAL_UINT32(0U, monitor.captureSize());
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RailTriggerState::Idle),
    static_cast<int>(monitor.triggerState()));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(testDividerConversion);
  RUN_TEST(testDegenerateCalibrationDoesNotDivideByZero);
  RUN_TEST(testStatisticsTrackMinimumAndMean);
  RUN_TEST(testRollingWindowForgetsOldSags);
  RUN_TEST(testTriggerCapturesAroundTheEventAndFreezes);
  RUN_TEST(testDisarmedTriggerNeverCaptures);
  RUN_TEST(testTimestampWraparoundStaysOrdered);
  RUN_TEST(testResetClearsEverything);
  return UNITY_END();
}
