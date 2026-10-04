#pragma once

#include "robot_config.hpp"

#include <cstddef>
#include <cstdint>

namespace robot_arm {

enum class MotionMode : uint8_t {
  Replace = 0,
  Queue,
};

enum class RobotState : uint8_t {
  Booting = 0,
  Disarmed,
  Ready,
  Fault,
  EmergencyStopped,
};

enum class MotionResult : uint8_t {
  Accepted = 0,
  NoOp,
  NotReady,
  InvalidServoCount,
  InvalidJoint,
  InvalidAngle,
  InvalidDuration,
  QueueFull,
  EmergencyStopped,
};

struct ServoMotionState {
  float currentAngleDeg;
  float startAngleDeg;
  float targetAngleDeg;
  bool moving;
  bool enabled;
};

struct MotionEvent {
  bool available;
  uint32_t motionId;
};

float smootherStep(float normalizedTime);
// Public smooth-motion angles are ROS joint degrees. Mapping adds the
// generated zero offset internally before producing a servo pulse.
float clampJointAngle(const ServoCalibration & calibration, float requestedAngleDeg);
float angleToPulseUsFloat(
  const ServoCalibration & calibration, float jointAngleDeg);
uint16_t angleToPulseUs(const ServoCalibration & calibration, float jointAngleDeg);
float pulseUsToAngle(const ServoCalibration & calibration, uint16_t pulseUs);

class MotionController {
public:
  MotionController(const ServoCalibration * calibration, std::size_t servoCount);

  std::size_t servoCount() const;
  RobotState state() const;
  bool isMoving() const;
  std::size_t queueSize() const;
  uint32_t activeDurationMs() const;
  uint32_t remainingDurationMs(uint32_t nowMs) const;
  const ServoMotionState & servo(std::size_t index) const;

  MotionResult arm(
    const float * observedAnglesDeg, std::size_t count, uint32_t nowMs);
  void disarm(uint32_t nowMs);
  void communicationTimeout(uint32_t nowMs);
  void emergencyStop(uint32_t nowMs);
  MotionResult resetEmergencyStop();
  MotionResult stop(uint32_t nowMs);
  void clearQueue();

  MotionResult commandSynchronizedMove(
    const float * targetAnglesDeg,
    std::size_t count,
    uint32_t requestedDurationMs,
    MotionMode mode,
    uint32_t motionId,
    bool startupSpeed,
    uint32_t nowMs,
    uint32_t * acceptedDurationMs);

  MotionResult commandJointMove(
    std::size_t jointIndex,
    float targetAngleDeg,
    uint32_t requestedDurationMs,
    MotionMode mode,
    uint32_t motionId,
    bool startupSpeed,
    uint32_t nowMs,
    uint32_t * acceptedDurationMs);

  bool update(uint32_t nowMs);
  MotionEvent takeCompletionEvent();

private:
  struct QueuedMotion {
    float targetsDeg[kMaxServoCount];
    uint32_t requestedDurationMs;
    uint32_t motionId;
    bool startupSpeed;
  };

  bool validateTargets(const float * targetsDeg, std::size_t count) const;
  bool targetWithinDeadband(const float * targetsDeg, const float * referenceDeg) const;
  void plannedTargets(float * targetsDeg) const;
  uint32_t safeDurationMs(
    const float * startDeg,
    const float * targetDeg,
    uint32_t requestedDurationMs,
    bool startupSpeed) const;
  MotionResult startMotion(
    const float * targetsDeg,
    uint32_t requestedDurationMs,
    uint32_t motionId,
    bool startupSpeed,
    uint32_t nowMs,
    uint32_t * acceptedDurationMs);
  bool enqueue(
    const float * targetsDeg,
    uint32_t requestedDurationMs,
    uint32_t motionId,
    bool startupSpeed);
  bool startNextQueued(uint32_t nowMs);
  void snapshotCurrent(uint32_t nowMs);

  const ServoCalibration * calibration_;
  std::size_t servoCount_;
  RobotState state_;
  ServoMotionState servo_[kMaxServoCount];

  uint32_t motionStartMs_;
  uint32_t motionDurationMs_;
  uint32_t activeMotionId_;
  bool activeMotion_;

  QueuedMotion queue_[kMotionQueueCapacity];
  std::size_t queueHead_;
  std::size_t queueTail_;
  std::size_t queueSize_;

  MotionEvent completionEvent_;
};

const char * robotStateName(RobotState state);
const char * motionResultName(MotionResult result);

}  // namespace robot_arm
