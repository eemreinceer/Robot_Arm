#include "motion_controller.hpp"
#include "pwm_output.hpp"
#include "robot_config.hpp"
#include "time_utils.hpp"

#include <unity.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

uint16_t hostReferencePulseUs(
  const robot_arm::ServoCalibration & calibration, float requestedJointAngleDeg)
{
  const float jointAngleDeg =
    robot_arm::clampJointAngle(calibration, requestedJointAngleDeg);
  const float servoAngleDeg = jointAngleDeg + calibration.zeroOffsetDeg;
  float ratio =
    (servoAngleDeg - calibration.calibrationMinServoAngleDeg) /
    (calibration.calibrationMaxServoAngleDeg -
    calibration.calibrationMinServoAngleDeg);
  if (calibration.reversed) {
    ratio = 1.0f - ratio;
  }
  const float pulseUs =
    static_cast<float>(calibration.minPulseUs) +
    ratio * static_cast<float>(
    calibration.maxPulseUs - calibration.minPulseUs);
  return static_cast<uint16_t>(std::lround(pulseUs));
}

void testSmootherStepEndpointsAndMonotonicity() {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 0.0f, robot_arm::smootherStep(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 1.0f, robot_arm::smootherStep(1.0f));
  float previous = 0.0f;
  for (int step = 1; step <= 100; ++step) {
    const float value = robot_arm::smootherStep(static_cast<float>(step) / 100.0f);
    TEST_ASSERT_GREATER_OR_EQUAL(previous, value);
    TEST_ASSERT_TRUE(value >= 0.0f && value <= 1.0f);
    previous = value;
  }
}

void testElapsedTimeRejectsFutureSampleAndHandlesWraparound() {
  TEST_ASSERT_FALSE(robot_arm::elapsedAtLeast(100U, 101U, 1U));
  TEST_ASSERT_FALSE(robot_arm::elapsedAtLeast(1099U, 100U, 1000U));
  TEST_ASSERT_TRUE(robot_arm::elapsedAtLeast(1100U, 100U, 1000U));
  TEST_ASSERT_TRUE(robot_arm::elapsedAtLeast(5U, 0xfffffff5U, 16U));
}

void testAngleClampAndPulseMapping() {
  const robot_arm::ServoCalibration & calibration = robot_arm::kServoConfig[0U];
  TEST_ASSERT_FLOAT_WITHIN(
    1.0e-4f,
    calibration.jointLimitMinAngleDeg,
    robot_arm::clampJointAngle(calibration, -1000.0f));
  TEST_ASSERT_EQUAL_UINT16(1500U, robot_arm::angleToPulseUs(calibration, 0.0f));
  TEST_ASSERT_EQUAL_UINT16(
    robot_arm::angleToPulseUs(calibration, calibration.jointLimitMaxAngleDeg),
    robot_arm::angleToPulseUs(calibration, 1000.0f));
}

void testRosZeroOffsetsMatchCanonicalHostPwm() {
  const uint16_t expectedPulseUs[robot_arm::kServoCount] = {
    1500U, 1914U, 2290U, 1373U, 1436U, 1500U,
  };
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    TEST_ASSERT_UINT16_WITHIN(
      1U,
      expectedPulseUs[index],
      robot_arm::angleToPulseUs(robot_arm::kServoConfig[index], 0.0f));
  }
}

void testFirmwareMappingMatchesHostReferenceWithinOneMicrosecond() {
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    const robot_arm::ServoCalibration & calibration = robot_arm::kServoConfig[index];
    const float span =
      calibration.jointLimitMaxAngleDeg - calibration.jointLimitMinAngleDeg;
    const float samples[] = {
      calibration.jointLimitMinAngleDeg,
      calibration.jointLimitMinAngleDeg + 0.25f * span,
      0.0f,
      calibration.jointLimitMinAngleDeg + 0.75f * span,
      calibration.jointLimitMaxAngleDeg,
    };
    for (float jointAngleDeg : samples) {
      TEST_ASSERT_UINT16_WITHIN(
        1U,
        hostReferencePulseUs(calibration, jointAngleDeg),
        robot_arm::angleToPulseUs(calibration, jointAngleDeg));
    }
  }
}

void testSmoothPwmPreservesSubMicrosecondResolution() {
  const robot_arm::ServoCalibration & calibration = robot_arm::kServoConfig[2U];
  const float firstPulseUs = robot_arm::angleToPulseUsFloat(calibration, 8.0f);
  const float secondPulseUs = robot_arm::angleToPulseUsFloat(calibration, 8.025f);

  TEST_ASSERT_EQUAL_UINT16(
    robot_arm::angleToPulseUs(calibration, 8.0f),
    robot_arm::angleToPulseUs(calibration, 8.025f));
  TEST_ASSERT_NOT_EQUAL(
    robot_arm::smoothPulseUsToDuty(firstPulseUs),
    robot_arm::smoothPulseUsToDuty(secondPulseUs));
  TEST_ASSERT_EQUAL_UINT32(
    (1500U * 65536U) / 20000U,
    robot_arm::legacyPulseUsToDuty(1500U));
}

void testReversedMapping() {
  robot_arm::ServoCalibration calibration = robot_arm::kServoConfig[0U];
  calibration.jointLimitMinAngleDeg =
    calibration.calibrationMinServoAngleDeg - calibration.zeroOffsetDeg;
  calibration.jointLimitMaxAngleDeg =
    calibration.calibrationMaxServoAngleDeg - calibration.zeroOffsetDeg;
  calibration.reversed = true;
  TEST_ASSERT_EQUAL_UINT16(
    calibration.maxPulseUs,
    robot_arm::angleToPulseUs(calibration, calibration.jointLimitMinAngleDeg));
  TEST_ASSERT_EQUAL_UINT16(
    calibration.minPulseUs,
    robot_arm::angleToPulseUs(calibration, calibration.jointLimitMaxAngleDeg));
  TEST_ASSERT_FLOAT_WITHIN(
    0.05f,
    calibration.jointLimitMaxAngleDeg,
    robot_arm::pulseUsToAngle(calibration, calibration.minPulseUs));
}

void armAtZero(robot_arm::MotionController * controller) {
  const float reference[robot_arm::kServoCount] = {};
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(
      controller->arm(reference, robot_arm::kServoCount, 0U)));
}

void testDurationUsesSlowestJointAndQuinticPeakSpeed() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 35.0f;
  target[1U] = 22.0f;
  uint32_t acceptedDurationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      100U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &acceptedDurationMs)));
  // Canonical 0.4 rad/s limit plus quintic 1.875x peak-speed factor.
  TEST_ASSERT_EQUAL_UINT32(2864U, acceptedDurationMs);
}

void testDeadbandDoesNotRestartMotion() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 0.5f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::NoOp),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Replace,
      2U,
      false,
      0U,
      &durationMs)));
  TEST_ASSERT_FALSE(controller.isMoving());
}

void testMotionCompletesExactlyAtTarget() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 10.0f;
  target[1U] = -5.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      2000U,
      robot_arm::MotionMode::Replace,
      42U,
      false,
      10U,
      &durationMs)));
  controller.update(10U + durationMs);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 10.0f, controller.servo(0U).currentAngleDeg);
  TEST_ASSERT_FLOAT_WITHIN(-1.0e-6f, -5.0f, controller.servo(1U).currentAngleDeg);
  TEST_ASSERT_FALSE(controller.isMoving());
  const robot_arm::MotionEvent event = controller.takeCompletionEvent();
  TEST_ASSERT_TRUE(event.available);
  TEST_ASSERT_EQUAL_UINT32(42U, event.motionId);
}

void testAllJointsShareOneNormalizedTime() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  const float target[robot_arm::kServoCount] = {
    10.0f, -20.0f, 10.0f, -10.0f, 20.0f, 5.0f,
  };
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      5000U,
      robot_arm::MotionMode::Replace,
      77U,
      false,
      0U,
      &durationMs)));
  TEST_ASSERT_EQUAL_UINT32(5000U, durationMs);
  controller.update(2500U);
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    TEST_ASSERT_FLOAT_WITHIN(
      1.0e-4f, target[index] * 0.5f, controller.servo(index).currentAngleDeg);
  }
}

void testReplaceStartsFromInterpolatedPosition() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 40.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      4000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
  controller.update(1000U);
  const float beforeReplace = controller.servo(0U).currentAngleDeg;
  target[0U] = -20.0f;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      3000U,
      robot_arm::MotionMode::Replace,
      2U,
      false,
      1000U,
      &durationMs)));
  TEST_ASSERT_FLOAT_WITHIN(
    1.0e-6f, beforeReplace, controller.servo(0U).startAngleDeg);
}

void testQueueCapacityAndExecution() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 5.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
  const uint32_t firstDurationMs = durationMs;

  for (std::size_t index = 0U; index < robot_arm::kMotionQueueCapacity; ++index) {
    target[0U] += 5.0f;
    TEST_ASSERT_EQUAL(
      static_cast<int>(robot_arm::MotionResult::Accepted),
      static_cast<int>(controller.commandSynchronizedMove(
        target,
        robot_arm::kServoCount,
        1000U,
        robot_arm::MotionMode::Queue,
        static_cast<uint32_t>(index + 2U),
        false,
        0U,
        &durationMs)));
  }
  target[0U] += 5.0f;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::QueueFull),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Queue,
      99U,
      false,
      0U,
      &durationMs)));

  controller.update(firstDurationMs);
  TEST_ASSERT_TRUE(controller.isMoving());
  TEST_ASSERT_EQUAL_UINT32(
    static_cast<uint32_t>(robot_arm::kMotionQueueCapacity - 1U),
    static_cast<uint32_t>(controller.queueSize()));
}

void testInvalidReferencesAndTargetsAreRejected() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  float reference[robot_arm::kServoCount] = {};
  reference[0U] = NAN;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::InvalidAngle),
    static_cast<int>(controller.arm(reference, robot_arm::kServoCount, 0U)));

  reference[0U] = 0.0f;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.arm(reference, robot_arm::kServoCount, 0U)));
  float target[robot_arm::kServoCount] = {};
  target[4U] = 60.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::InvalidAngle),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
}

void testCommunicationTimeoutHoldsAndRequiresRearm() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  float target[robot_arm::kServoCount] = {};
  target[0U] = 20.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      4000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
  target[0U] = 30.0f;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      2000U,
      robot_arm::MotionMode::Queue,
      2U,
      false,
      0U,
      &durationMs)));
  controller.update(500U);
  const float held = controller.servo(0U).currentAngleDeg;
  controller.communicationTimeout(500U);
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RobotState::Disarmed),
    static_cast<int>(controller.state()));
  TEST_ASSERT_FALSE(controller.isMoving());
  TEST_ASSERT_EQUAL_UINT32(0U, static_cast<uint32_t>(controller.queueSize()));
  TEST_ASSERT_FLOAT_WITHIN(
    1.0e-6f, held, controller.servo(0U).currentAngleDeg);
}

void testEstopLatchesUntilResetAndRearm() {
  robot_arm::MotionController controller(robot_arm::kServoConfig, robot_arm::kServoCount);
  armAtZero(&controller);
  controller.emergencyStop(0U);
  controller.disarm(0U);
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::RobotState::EmergencyStopped),
    static_cast<int>(controller.state()));
  float target[robot_arm::kServoCount] = {};
  target[0U] = 5.0f;
  uint32_t durationMs = 0U;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::EmergencyStopped),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::Accepted),
    static_cast<int>(controller.resetEmergencyStop()));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionResult::NotReady),
    static_cast<int>(controller.commandSynchronizedMove(
      target,
      robot_arm::kServoCount,
      1000U,
      robot_arm::MotionMode::Replace,
      1U,
      false,
      0U,
      &durationMs)));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(testSmootherStepEndpointsAndMonotonicity);
  RUN_TEST(testElapsedTimeRejectsFutureSampleAndHandlesWraparound);
  RUN_TEST(testAngleClampAndPulseMapping);
  RUN_TEST(testRosZeroOffsetsMatchCanonicalHostPwm);
  RUN_TEST(testFirmwareMappingMatchesHostReferenceWithinOneMicrosecond);
  RUN_TEST(testSmoothPwmPreservesSubMicrosecondResolution);
  RUN_TEST(testReversedMapping);
  RUN_TEST(testDurationUsesSlowestJointAndQuinticPeakSpeed);
  RUN_TEST(testDeadbandDoesNotRestartMotion);
  RUN_TEST(testMotionCompletesExactlyAtTarget);
  RUN_TEST(testAllJointsShareOneNormalizedTime);
  RUN_TEST(testReplaceStartsFromInterpolatedPosition);
  RUN_TEST(testQueueCapacityAndExecution);
  RUN_TEST(testInvalidReferencesAndTargetsAreRejected);
  RUN_TEST(testCommunicationTimeoutHoldsAndRequiresRearm);
  RUN_TEST(testEstopLatchesUntilResetAndRearm);
  return UNITY_END();
}
