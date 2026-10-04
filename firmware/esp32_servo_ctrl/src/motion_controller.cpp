#include "motion_controller.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace robot_arm {
namespace {

constexpr float kSmootherStepPeakSlope = 1.875f;
constexpr float kFloatEpsilon = 1.0e-5f;

bool finiteFloat(float value) {
  return std::isfinite(value);
}

uint32_t ceilToUint32(float value) {
  if (!finiteFloat(value) || value <= 0.0f) {
    return 0U;
  }
  if (value >= static_cast<float>(std::numeric_limits<uint32_t>::max())) {
    return std::numeric_limits<uint32_t>::max();
  }
  return static_cast<uint32_t>(std::ceil(value));
}

float linearMap(
  float value,
  float inputMin,
  float inputMax,
  float outputMin,
  float outputMax)
{
  const float span = inputMax - inputMin;
  if (std::fabs(span) <= kFloatEpsilon) {
    return outputMin;
  }
  const float ratio = (value - inputMin) / span;
  return outputMin + ratio * (outputMax - outputMin);
}

}  // namespace

float smootherStep(float normalizedTime) {
  float t = normalizedTime;
  if (t <= 0.0f) {
    return 0.0f;
  }
  if (t >= 1.0f) {
    return 1.0f;
  }
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float clampJointAngle(
  const ServoCalibration & calibration, float requestedAngleDeg)
{
  if (!finiteFloat(requestedAngleDeg)) {
    return clampJointAngle(calibration, 0.0f);
  }
  if (requestedAngleDeg < calibration.jointLimitMinAngleDeg) {
    return calibration.jointLimitMinAngleDeg;
  }
  if (requestedAngleDeg > calibration.jointLimitMaxAngleDeg) {
    return calibration.jointLimitMaxAngleDeg;
  }
  return requestedAngleDeg;
}

float angleToPulseUsFloat(
  const ServoCalibration & calibration, float angleDeg)
{
  const float jointAngleDeg = clampJointAngle(calibration, angleDeg);
  const float servoAngleDeg = jointAngleDeg + calibration.zeroOffsetDeg;
  float pulseUs = linearMap(
    servoAngleDeg,
    calibration.calibrationMinServoAngleDeg,
    calibration.calibrationMaxServoAngleDeg,
    static_cast<float>(calibration.minPulseUs),
    static_cast<float>(calibration.maxPulseUs));

  if (calibration.reversed) {
    pulseUs =
      static_cast<float>(calibration.minPulseUs + calibration.maxPulseUs) - pulseUs;
  }
  if (pulseUs < static_cast<float>(calibration.minPulseUs)) {
    pulseUs = static_cast<float>(calibration.minPulseUs);
  }
  if (pulseUs > static_cast<float>(calibration.maxPulseUs)) {
    pulseUs = static_cast<float>(calibration.maxPulseUs);
  }
  return pulseUs;
}

uint16_t angleToPulseUs(
  const ServoCalibration & calibration, float angleDeg)
{
  return static_cast<uint16_t>(
    std::lround(angleToPulseUsFloat(calibration, angleDeg)));
}

float pulseUsToAngle(
  const ServoCalibration & calibration, uint16_t pulseUs)
{
  float pulse = static_cast<float>(pulseUs);
  if (pulse < static_cast<float>(calibration.minPulseUs)) {
    pulse = static_cast<float>(calibration.minPulseUs);
  }
  if (pulse > static_cast<float>(calibration.maxPulseUs)) {
    pulse = static_cast<float>(calibration.maxPulseUs);
  }
  if (calibration.reversed) {
    pulse =
      static_cast<float>(calibration.minPulseUs + calibration.maxPulseUs) - pulse;
  }

  const float servoAngleDeg = linearMap(
    pulse,
    static_cast<float>(calibration.minPulseUs),
    static_cast<float>(calibration.maxPulseUs),
    calibration.calibrationMinServoAngleDeg,
    calibration.calibrationMaxServoAngleDeg);
  return clampJointAngle(
    calibration, servoAngleDeg - calibration.zeroOffsetDeg);
}

MotionController::MotionController(
  const ServoCalibration * calibration, std::size_t servoCount)
: calibration_(calibration),
  servoCount_(servoCount <= kMaxServoCount ? servoCount : 0U),
  state_(RobotState::Booting),
  motionStartMs_(0U),
  motionDurationMs_(0U),
  activeMotionId_(0U),
  activeMotion_(false),
  queueHead_(0U),
  queueTail_(0U),
  queueSize_(0U),
  completionEvent_{false, 0U}
{
  std::memset(servo_, 0, sizeof(servo_));
  std::memset(queue_, 0, sizeof(queue_));
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    const float neutralJointAngleDeg =
      clampJointAngle(calibration_[index], 0.0f);
    servo_[index].currentAngleDeg = neutralJointAngleDeg;
    servo_[index].startAngleDeg = neutralJointAngleDeg;
    servo_[index].targetAngleDeg = neutralJointAngleDeg;
    servo_[index].enabled = false;
  }
  state_ = servoCount_ == 0U ? RobotState::Fault : RobotState::Disarmed;
}

std::size_t MotionController::servoCount() const {
  return servoCount_;
}

RobotState MotionController::state() const {
  return state_;
}

bool MotionController::isMoving() const {
  return activeMotion_;
}

std::size_t MotionController::queueSize() const {
  return queueSize_;
}

uint32_t MotionController::activeDurationMs() const {
  return motionDurationMs_;
}

uint32_t MotionController::remainingDurationMs(uint32_t nowMs) const {
  if (!activeMotion_) {
    return 0U;
  }
  const uint32_t elapsedMs = nowMs - motionStartMs_;
  return elapsedMs >= motionDurationMs_ ? 0U : motionDurationMs_ - elapsedMs;
}

const ServoMotionState & MotionController::servo(std::size_t index) const {
  static const ServoMotionState invalidState{0.0f, 0.0f, 0.0f, false, false};
  return index < servoCount_ ? servo_[index] : invalidState;
}

MotionResult MotionController::arm(
  const float * observedAnglesDeg, std::size_t count, uint32_t nowMs)
{
  if (state_ == RobotState::EmergencyStopped) {
    return MotionResult::EmergencyStopped;
  }
  if (state_ != RobotState::Disarmed) {
    return MotionResult::NotReady;
  }
  if (observedAnglesDeg == nullptr || count != servoCount_) {
    return MotionResult::InvalidServoCount;
  }
  if (!validateTargets(observedAnglesDeg, count)) {
    return MotionResult::InvalidAngle;
  }

  activeMotion_ = false;
  clearQueue();
  completionEvent_.available = false;
  motionStartMs_ = nowMs;
  motionDurationMs_ = 0U;
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].currentAngleDeg = observedAnglesDeg[index];
    servo_[index].startAngleDeg = observedAnglesDeg[index];
    servo_[index].targetAngleDeg = observedAnglesDeg[index];
    servo_[index].moving = false;
    servo_[index].enabled = true;
  }
  state_ = RobotState::Ready;
  return MotionResult::Accepted;
}

void MotionController::snapshotCurrent(uint32_t nowMs) {
  if (!activeMotion_) {
    return;
  }
  update(nowMs);
}

void MotionController::disarm(uint32_t nowMs) {
  snapshotCurrent(nowMs);
  activeMotion_ = false;
  clearQueue();
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].startAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].targetAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].moving = false;
  }
  if (state_ != RobotState::Fault &&
    state_ != RobotState::EmergencyStopped)
  {
    state_ = RobotState::Disarmed;
  }
}

void MotionController::communicationTimeout(uint32_t nowMs) {
  disarm(nowMs);
}

void MotionController::emergencyStop(uint32_t nowMs) {
  snapshotCurrent(nowMs);
  activeMotion_ = false;
  clearQueue();
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].startAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].targetAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].moving = false;
  }
  state_ = RobotState::EmergencyStopped;
}

MotionResult MotionController::resetEmergencyStop() {
  if (state_ != RobotState::EmergencyStopped) {
    return MotionResult::NotReady;
  }
  state_ = RobotState::Disarmed;
  return MotionResult::Accepted;
}

MotionResult MotionController::stop(uint32_t nowMs) {
  if (state_ == RobotState::EmergencyStopped) {
    return MotionResult::EmergencyStopped;
  }
  if (state_ != RobotState::Ready) {
    return MotionResult::NotReady;
  }
  snapshotCurrent(nowMs);
  activeMotion_ = false;
  clearQueue();
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].startAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].targetAngleDeg = servo_[index].currentAngleDeg;
    servo_[index].moving = false;
  }
  return MotionResult::Accepted;
}

void MotionController::clearQueue() {
  queueHead_ = 0U;
  queueTail_ = 0U;
  queueSize_ = 0U;
}

bool MotionController::validateTargets(
  const float * targetsDeg, std::size_t count) const
{
  if (targetsDeg == nullptr || count != servoCount_) {
    return false;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    if (!finiteFloat(targetsDeg[index]) ||
      targetsDeg[index] < calibration_[index].jointLimitMinAngleDeg ||
      targetsDeg[index] > calibration_[index].jointLimitMaxAngleDeg)
    {
      return false;
    }
  }
  return true;
}

bool MotionController::targetWithinDeadband(
  const float * targetsDeg, const float * referenceDeg) const
{
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    if (std::fabs(targetsDeg[index] - referenceDeg[index]) >
      calibration_[index].deadbandDeg)
    {
      return false;
    }
  }
  return true;
}

void MotionController::plannedTargets(float * targetsDeg) const {
  if (queueSize_ > 0U) {
    const std::size_t last =
      (queueTail_ + kMotionQueueCapacity - 1U) % kMotionQueueCapacity;
    std::memcpy(
      targetsDeg, queue_[last].targetsDeg, servoCount_ * sizeof(float));
    return;
  }
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    targetsDeg[index] =
      activeMotion_ ? servo_[index].targetAngleDeg : servo_[index].currentAngleDeg;
  }
}

uint32_t MotionController::safeDurationMs(
  const float * startDeg,
  const float * targetDeg,
  uint32_t requestedDurationMs,
  bool startupSpeed) const
{
  uint32_t safeMs = requestedDurationMs;
  if (safeMs < kMinimumMotionDurationMs) {
    safeMs = kMinimumMotionDurationMs;
  }
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    const float speedDegPerSec = startupSpeed ?
      calibration_[index].startupSpeedDegPerSec :
      calibration_[index].maxSpeedDegPerSec;
    if (!finiteFloat(speedDegPerSec) || speedDegPerSec <= kFloatEpsilon) {
      return kMaximumMotionDurationMs + 1U;
    }
    const float distanceDeg = std::fabs(targetDeg[index] - startDeg[index]);
    // Quintic smootherstep reaches 1.875 times its average speed at t=0.5.
    const float requiredMs =
      1000.0f * kSmootherStepPeakSlope * distanceDeg / speedDegPerSec;
    const uint32_t jointMs = ceilToUint32(requiredMs);
    if (jointMs > safeMs) {
      safeMs = jointMs;
    }
  }
  return safeMs;
}

MotionResult MotionController::startMotion(
  const float * targetsDeg,
  uint32_t requestedDurationMs,
  uint32_t motionId,
  bool startupSpeed,
  uint32_t nowMs,
  uint32_t * acceptedDurationMs)
{
  float startsDeg[kMaxServoCount] = {};
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    startsDeg[index] = servo_[index].currentAngleDeg;
  }

  const uint32_t durationMs =
    safeDurationMs(startsDeg, targetsDeg, requestedDurationMs, startupSpeed);
  if (durationMs > kMaximumMotionDurationMs) {
    return MotionResult::InvalidDuration;
  }
  if (acceptedDurationMs != nullptr) {
    *acceptedDurationMs = durationMs;
  }

  motionStartMs_ = nowMs;
  motionDurationMs_ = durationMs;
  activeMotionId_ = motionId;
  activeMotion_ = true;
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].startAngleDeg = startsDeg[index];
    servo_[index].targetAngleDeg = targetsDeg[index];
    servo_[index].moving =
      std::fabs(targetsDeg[index] - startsDeg[index]) > kFloatEpsilon;
  }
  return MotionResult::Accepted;
}

bool MotionController::enqueue(
  const float * targetsDeg,
  uint32_t requestedDurationMs,
  uint32_t motionId,
  bool startupSpeed)
{
  if (queueSize_ >= kMotionQueueCapacity) {
    return false;
  }
  QueuedMotion & slot = queue_[queueTail_];
  std::memcpy(slot.targetsDeg, targetsDeg, servoCount_ * sizeof(float));
  slot.requestedDurationMs = requestedDurationMs;
  slot.motionId = motionId;
  slot.startupSpeed = startupSpeed;
  queueTail_ = (queueTail_ + 1U) % kMotionQueueCapacity;
  ++queueSize_;
  return true;
}

bool MotionController::startNextQueued(uint32_t nowMs) {
  if (queueSize_ == 0U) {
    return false;
  }
  const QueuedMotion motion = queue_[queueHead_];
  queueHead_ = (queueHead_ + 1U) % kMotionQueueCapacity;
  --queueSize_;
  uint32_t ignoredDurationMs = 0U;
  return startMotion(
    motion.targetsDeg,
    motion.requestedDurationMs,
    motion.motionId,
    motion.startupSpeed,
    nowMs,
    &ignoredDurationMs) == MotionResult::Accepted;
}

MotionResult MotionController::commandSynchronizedMove(
  const float * targetAnglesDeg,
  std::size_t count,
  uint32_t requestedDurationMs,
  MotionMode mode,
  uint32_t motionId,
  bool startupSpeed,
  uint32_t nowMs,
  uint32_t * acceptedDurationMs)
{
  if (state_ == RobotState::EmergencyStopped) {
    return MotionResult::EmergencyStopped;
  }
  if (state_ != RobotState::Ready) {
    return MotionResult::NotReady;
  }
  if (count != servoCount_) {
    return MotionResult::InvalidServoCount;
  }
  if (!validateTargets(targetAnglesDeg, count)) {
    return MotionResult::InvalidAngle;
  }
  if (requestedDurationMs > kMaximumMotionDurationMs) {
    return MotionResult::InvalidDuration;
  }

  if (mode == MotionMode::Replace) {
    snapshotCurrent(nowMs);
    float activeReference[kMaxServoCount] = {};
    for (std::size_t index = 0U; index < servoCount_; ++index) {
      activeReference[index] =
        activeMotion_ ? servo_[index].targetAngleDeg : servo_[index].currentAngleDeg;
    }
    if (targetWithinDeadband(targetAnglesDeg, activeReference)) {
      return MotionResult::NoOp;
    }
    activeMotion_ = false;
    clearQueue();
    return startMotion(
      targetAnglesDeg,
      requestedDurationMs,
      motionId,
      startupSpeed,
      nowMs,
      acceptedDurationMs);
  }

  float referenceDeg[kMaxServoCount] = {};
  plannedTargets(referenceDeg);
  if (targetWithinDeadband(targetAnglesDeg, referenceDeg)) {
    return MotionResult::NoOp;
  }
  if (!activeMotion_ && queueSize_ == 0U) {
    return startMotion(
      targetAnglesDeg,
      requestedDurationMs,
      motionId,
      startupSpeed,
      nowMs,
      acceptedDurationMs);
  }
  const uint32_t durationMs =
    safeDurationMs(referenceDeg, targetAnglesDeg, requestedDurationMs, startupSpeed);
  if (durationMs > kMaximumMotionDurationMs) {
    return MotionResult::InvalidDuration;
  }
  if (!enqueue(targetAnglesDeg, requestedDurationMs, motionId, startupSpeed)) {
    return MotionResult::QueueFull;
  }
  if (acceptedDurationMs != nullptr) {
    *acceptedDurationMs = durationMs;
  }
  return MotionResult::Accepted;
}

MotionResult MotionController::commandJointMove(
  std::size_t jointIndex,
  float targetAngleDeg,
  uint32_t requestedDurationMs,
  MotionMode mode,
  uint32_t motionId,
  bool startupSpeed,
  uint32_t nowMs,
  uint32_t * acceptedDurationMs)
{
  if (jointIndex >= servoCount_) {
    return MotionResult::InvalidJoint;
  }
  float targetsDeg[kMaxServoCount] = {};
  if (mode == MotionMode::Queue) {
    plannedTargets(targetsDeg);
  } else {
    snapshotCurrent(nowMs);
    for (std::size_t index = 0U; index < servoCount_; ++index) {
      targetsDeg[index] = servo_[index].currentAngleDeg;
    }
  }
  targetsDeg[jointIndex] = targetAngleDeg;
  return commandSynchronizedMove(
    targetsDeg,
    servoCount_,
    requestedDurationMs,
    mode,
    motionId,
    startupSpeed,
    nowMs,
    acceptedDurationMs);
}

bool MotionController::update(uint32_t nowMs) {
  if (!activeMotion_) {
    return false;
  }

  const uint32_t elapsedMs = nowMs - motionStartMs_;
  if (elapsedMs >= motionDurationMs_) {
    for (std::size_t index = 0U; index < servoCount_; ++index) {
      servo_[index].currentAngleDeg = servo_[index].targetAngleDeg;
      servo_[index].startAngleDeg = servo_[index].targetAngleDeg;
      servo_[index].moving = false;
    }
    activeMotion_ = false;
    completionEvent_ = MotionEvent{true, activeMotionId_};
    startNextQueued(nowMs);
    return true;
  }

  const float normalizedTime =
    static_cast<float>(elapsedMs) / static_cast<float>(motionDurationMs_);
  const float progress = smootherStep(normalizedTime);
  for (std::size_t index = 0U; index < servoCount_; ++index) {
    servo_[index].currentAngleDeg =
      servo_[index].startAngleDeg +
      (servo_[index].targetAngleDeg - servo_[index].startAngleDeg) * progress;
  }
  return true;
}

MotionEvent MotionController::takeCompletionEvent() {
  const MotionEvent result = completionEvent_;
  completionEvent_.available = false;
  return result;
}

const char * robotStateName(RobotState state) {
  switch (state) {
    case RobotState::Booting: return "BOOTING";
    case RobotState::Disarmed: return "DISARMED";
    case RobotState::Ready: return "READY";
    case RobotState::Fault: return "FAULT";
    case RobotState::EmergencyStopped: return "ESTOP";
    default: return "UNKNOWN";
  }
}

const char * motionResultName(MotionResult result) {
  switch (result) {
    case MotionResult::Accepted: return "ACCEPTED";
    case MotionResult::NoOp: return "DEADBAND";
    case MotionResult::NotReady: return "NOT_READY";
    case MotionResult::InvalidServoCount: return "SERVO_COUNT";
    case MotionResult::InvalidJoint: return "JOINT";
    case MotionResult::InvalidAngle: return "ANGLE_LIMIT";
    case MotionResult::InvalidDuration: return "DURATION";
    case MotionResult::QueueFull: return "QUEUE_FULL";
    case MotionResult::EmergencyStopped: return "ESTOP_LATCHED";
    default: return "UNKNOWN";
  }
}

}  // namespace robot_arm
