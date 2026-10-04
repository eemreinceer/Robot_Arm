/* Robot Arm ESP32 servo controller.
 *
 * The deployed UART v1 contract remains byte-for-byte compatible:
 *   V? -> V1,<version>
 *   P<pulse_us x 6> -> OK
 *   S -> OK and PWM off
 *
 * 2026-08-19: the link is UART0 (the onboard CP2102 USB bridge), not UART2.
 * No GPIO wiring to the host is required; connect the USB cable and the host
 * opens /dev/ttyUSB0. See README.md "Hardware and transport".
 *
 * The additive smooth-motion protocol is deliberately separate. It requires
 * an operator-supplied ARM reference because RC servos provide no absolute
 * position feedback. See README.md before using it on assembled hardware.
 */

#include <Arduino.h>

extern "C" {
#include "protocol.h"
}

#include "motion_controller.hpp"
#include "pwm_output.hpp"
#include "rail_config.hpp"
#include "rail_monitor.hpp"
#include "robot_config.hpp"
#include "smooth_protocol.hpp"
#include "time_utils.hpp"

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define FW_VERSION "1.4.0-esp32"

namespace {

constexpr uint32_t kLegacyWatchdogMs = 1000U;
// Policy C (docs/watchdog_policy_proposal.md, decided 2026-08-15): the physical
// behaviour does not change -- PWM stays latched, because this arm has no
// brakes and cutting the pulse drops it. What changes is the silence. A single
// E3 can be lost inside a mangled ACK read, so the announcement repeats for as
// long as the link is dead.
constexpr uint32_t kWatchdogAnnounceMs = 1000U;
// The operator needs to see it without a terminal: the heartbeat doubles as the
// alarm indicator rather than adding a second LED.
constexpr uint32_t kHeartbeatNormalMs = 500U;
constexpr uint32_t kHeartbeatAlarmMs = 100U;
constexpr uint32_t kPwmFrequencyHz = 50U;
constexpr uint32_t kPwmResolutionBits = 16U;
constexpr uint8_t kLedPin = 2U;
constexpr std::size_t kReplyBufferSize = 384U;
constexpr std::size_t kErrorNameSize = 32U;

enum class ControlSource : uint8_t {
  None = 0,
  Legacy,
  Smooth,
};

// 2026-08-19: protocol moved off UART2/GPIO16-17 onto UART0, the same
// hardware UART the onboard CP2102 exposes over the USB port. No GPIO
// wiring to the host is needed anymore -- the USB cable that already powers
// the board is the whole link. `Serial` (defined by the Arduino core) IS
// UART0; do not construct a second HardwareSerial(0), it would fight the
// core's instance over the same peripheral registers.
HardwareSerial & uartLink = Serial;
robot_arm::MotionController motion(robot_arm::kServoConfig, robot_arm::kServoCount);

// Faz 7 / A1 rail monitor. Sampled by a dedicated task on the other core so
// the 20 ms servo scheduler keeps its timing; the spinlock only guards the
// small statistics updates, never the capture dump.
robot_arm::RailMonitor railMonitor(robot_arm::kRailCalibration);
portMUX_TYPE railMux = portMUX_INITIALIZER_UNLOCKED;
bool railSamplerStarted = false;

ControlSource controlSource = ControlSource::None;
bool pwmEnabled = false;
bool legacyWatchdogReported = false;
bool smoothTimeoutReported = false;
// Set by either watchdog, cleared by any command that proves the link is back.
// Drives the LED only; it must never gate PWM.
bool communicationLost = false;
uint32_t lastWatchdogAnnounceMs = 0U;
uint32_t lastLegacyPositionMs = 0U;
uint32_t lastSmoothCommandMs = 0U;
uint32_t lastServoUpdateMs = 0U;
char lastError[kErrorNameSize] = "NONE";

void linkWrite(const char * text) {
  uartLink.write(reinterpret_cast<const uint8_t *>(text), std::strlen(text));
}

void linkPrintf(const char * format, ...) {
  char reply[kReplyBufferSize] = {};
  va_list arguments;
  va_start(arguments, format);
  std::vsnprintf(reply, sizeof(reply), format, arguments);
  va_end(arguments);
  linkWrite(reply);
}

void rememberError(const char * error) {
  std::snprintf(lastError, sizeof(lastError), "%s", error);
}

void stopAllPwm() {
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    ledcWrite(robot_arm::kServoConfig[index].ledcChannel, 0U);
  }
  pwmEnabled = false;
}

void writePulse(std::size_t index, uint16_t pulseUs) {
  if (index >= robot_arm::kServoCount) {
    return;
  }
  ledcWrite(
    robot_arm::kServoConfig[index].ledcChannel,
    robot_arm::legacyPulseUsToDuty(pulseUs));
}

void writeSmoothOutputs() {
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    const float pulseUs = robot_arm::angleToPulseUsFloat(
      robot_arm::kServoConfig[index], motion.servo(index).currentAngleDeg);
    ledcWrite(
      robot_arm::kServoConfig[index].ledcChannel,
      robot_arm::smoothPulseUsToDuty(pulseUs));
  }
  pwmEnabled = true;
}

void applyLegacyPositions(const uint16_t pulseUs[SERVO_CHANNEL_COUNT]) {
  motion.disarm(millis());
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    writePulse(index, pulseUs[index]);
  }
  pwmEnabled = true;
  controlSource = ControlSource::Legacy;
  lastLegacyPositionMs = millis();
  legacyWatchdogReported = false;
  smoothTimeoutReported = false;
  communicationLost = false;
}

void handleLegacyLine(const char * lineText) {
  // Calibration fingerprint request. Answered before the parser runs and
  // WITHOUT touching PWM or the arming state: the host asks this once during
  // activation, to check that the limits compiled into this firmware come from
  // the same YAML it is converting radians with. Answering an unknown request
  // with E1 is what an older build does, and the host treats that as
  // "cannot verify" rather than "disagrees".
  if (lineText[0] == 'K' && lineText[1] == '?' && lineText[2] == '\0') {
    linkPrintf("K,%s\n", robot_arm::kCalibrationFingerprint);
    return;
  }

  protocol_command_t command;
  const protocol_result_t result = protocol_parse_line(lineText, &command);
  if (result != PROTOCOL_OK) {
    linkWrite(result == PROTOCOL_ERROR_CHANNEL_COUNT ? "E2\n" : "E1\n");
    return;
  }

  switch (command.type) {
    case PROTOCOL_COMMAND_POSITION:
      if (motion.state() == robot_arm::RobotState::EmergencyStopped) {
        rememberError("ESTOP_LATCHED");
        linkWrite("ERR,ESTOP_LATCHED\n");
        break;
      }
      applyLegacyPositions(command.pulse_us);
      linkWrite("OK\n");
      break;
    case PROTOCOL_COMMAND_VERSION:
      linkWrite("V1," FW_VERSION "\n");
      break;
    case PROTOCOL_COMMAND_SAFE_STOP:
      if (motion.state() != robot_arm::RobotState::EmergencyStopped) {
        motion.disarm(millis());
      }
      stopAllPwm();
      controlSource = ControlSource::None;
      linkWrite("OK\n");
      break;
    default:
      linkWrite("E1\n");
      break;
  }
}

void markSmoothCommandValid(uint32_t nowMs) {
  lastSmoothCommandMs = nowMs;
  smoothTimeoutReported = false;
  communicationLost = false;
}

void sendMotionReply(
  const char * commandName,
  robot_arm::MotionResult result,
  uint32_t durationMs,
  uint32_t motionId)
{
  if (result == robot_arm::MotionResult::Accepted) {
    linkPrintf(
      "ACK,%s,DURATION_MS=%lu,ID=%lu\n",
      commandName,
      static_cast<unsigned long>(durationMs),
      static_cast<unsigned long>(motionId));
    std::snprintf(lastError, sizeof(lastError), "NONE");
    return;
  }
  if (result == robot_arm::MotionResult::NoOp) {
    linkPrintf(
      "ACK,%s,NOOP=DEADBAND,ID=%lu\n",
      commandName,
      static_cast<unsigned long>(motionId));
    return;
  }
  rememberError(robot_arm::motionResultName(result));
  linkPrintf("ERR,%s\n", robot_arm::motionResultName(result));
}

void sendStatus(uint32_t nowMs) {
  linkPrintf(
    "STATE,%s,MOVING=%u,QUEUE=%u,DURATION_MS=%lu,REMAINING_MS=%lu,"
    "PWM=%u,LINK=ACTIVE,LAST_ERROR=%s,FRAME=ROS_JOINT_DEG,CALIB=%s\n",
    robot_arm::robotStateName(motion.state()),
    motion.isMoving() ? 1U : 0U,
    static_cast<unsigned>(motion.queueSize()),
    static_cast<unsigned long>(motion.activeDurationMs()),
    static_cast<unsigned long>(motion.remainingDurationMs(nowMs)),
    pwmEnabled ? 1U : 0U,
    lastError,
    robot_arm::kCalibrationSha256Short);

  linkPrintf(
    "ANGLES,CURRENT=%.3f|%.3f|%.3f|%.3f|%.3f|%.3f,"
    "TARGET=%.3f|%.3f|%.3f|%.3f|%.3f|%.3f,"
    "MOVING=%u|%u|%u|%u|%u|%u\n",
    motion.servo(0U).currentAngleDeg,
    motion.servo(1U).currentAngleDeg,
    motion.servo(2U).currentAngleDeg,
    motion.servo(3U).currentAngleDeg,
    motion.servo(4U).currentAngleDeg,
    motion.servo(5U).currentAngleDeg,
    motion.servo(0U).targetAngleDeg,
    motion.servo(1U).targetAngleDeg,
    motion.servo(2U).targetAngleDeg,
    motion.servo(3U).targetAngleDeg,
    motion.servo(4U).targetAngleDeg,
    motion.servo(5U).targetAngleDeg,
    motion.servo(0U).moving ? 1U : 0U,
    motion.servo(1U).moving ? 1U : 0U,
    motion.servo(2U).moving ? 1U : 0U,
    motion.servo(3U).moving ? 1U : 0U,
    motion.servo(4U).moving ? 1U : 0U,
    motion.servo(5U).moving ? 1U : 0U);
}

void startDemo(uint32_t nowMs) {
  if (motion.state() != robot_arm::RobotState::Ready) {
    sendMotionReply("DEMO", robot_arm::MotionResult::NotReady, 0U, 0U);
    return;
  }

  constexpr float poseAOffsetDeg[robot_arm::kServoCount] =
    {4.0f, -3.0f, 3.0f, 4.0f, -4.0f, 2.0f};
  constexpr float poseBOffsetDeg[robot_arm::kServoCount] =
    {-4.0f, 3.0f, -3.0f, -4.0f, 4.0f, -2.0f};
  float poseA[robot_arm::kServoCount] = {};
  float poseB[robot_arm::kServoCount] = {};
  float neutral[robot_arm::kServoCount] = {};
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    const float reference = motion.servo(index).currentAngleDeg;
    poseA[index] = robot_arm::clampJointAngle(
      robot_arm::kServoConfig[index], reference + poseAOffsetDeg[index]);
    poseB[index] = robot_arm::clampJointAngle(
      robot_arm::kServoConfig[index], reference + poseBOffsetDeg[index]);
    neutral[index] = 0.0f;
  }

  uint32_t firstDurationMs = 0U;
  const robot_arm::MotionResult first = motion.commandSynchronizedMove(
    poseA,
    robot_arm::kServoCount,
    2000U,
    robot_arm::MotionMode::Replace,
    9001U,
    true,
    nowMs,
    &firstDurationMs);
  if (first != robot_arm::MotionResult::Accepted) {
    sendMotionReply("DEMO", first, firstDurationMs, 9001U);
    return;
  }
  uint32_t ignoredDurationMs = 0U;
  const robot_arm::MotionResult second = motion.commandSynchronizedMove(
    poseB,
    robot_arm::kServoCount,
    2000U,
    robot_arm::MotionMode::Queue,
    9002U,
    true,
    nowMs,
    &ignoredDurationMs);
  const robot_arm::MotionResult third = motion.commandSynchronizedMove(
    neutral,
    robot_arm::kServoCount,
    3000U,
    robot_arm::MotionMode::Queue,
    9003U,
    true,
    nowMs,
    &ignoredDurationMs);
  if (second != robot_arm::MotionResult::Accepted ||
    third != robot_arm::MotionResult::Accepted)
  {
    motion.stop(nowMs);
    sendMotionReply(
      "DEMO",
      second != robot_arm::MotionResult::Accepted ? second : third,
      0U,
      0U);
    return;
  }
  writeSmoothOutputs();
  sendMotionReply("DEMO", first, firstDurationMs, 9001U);
}

void handleSmoothLine(const char * lineText) {
  robot_arm::SmoothCommand command;
  const robot_arm::SmoothParseResult parseResult =
    robot_arm::parseSmoothCommand(lineText, robot_arm::kServoCount, &command);
  if (parseResult != robot_arm::SmoothParseResult::Ok) {
    rememberError(robot_arm::smoothParseResultName(parseResult));
    linkPrintf("ERR,%s\n", robot_arm::smoothParseResultName(parseResult));
    return;
  }

  const uint32_t nowMs = millis();
  switch (command.type) {
    case robot_arm::SmoothCommandType::Ping:
      markSmoothCommandValid(nowMs);
      linkPrintf(
        "ACK,PING,FW=" FW_VERSION ",FRAME=ROS_JOINT_DEG,CALIB=%s\n",
        robot_arm::kCalibrationSha256Short);
      return;

    case robot_arm::SmoothCommandType::Status:
      markSmoothCommandValid(nowMs);
      sendStatus(nowMs);
      return;

    case robot_arm::SmoothCommandType::Arm: {
      if (pwmEnabled && controlSource == ControlSource::Legacy) {
        rememberError("OUTPUT_ACTIVE");
        linkWrite("ERR,OUTPUT_ACTIVE,SEND_S_FIRST\n");
        return;
      }
      const robot_arm::MotionResult result =
        motion.arm(command.anglesDeg, command.angleCount, nowMs);
      if (result == robot_arm::MotionResult::Accepted) {
        controlSource = ControlSource::Smooth;
        markSmoothCommandValid(nowMs);
        linkWrite("ACK,ARM,PWM=OFF\n");
      } else {
        sendMotionReply("ARM", result, 0U, 0U);
      }
      return;
    }

    case robot_arm::SmoothCommandType::Disarm:
      if (motion.state() == robot_arm::RobotState::EmergencyStopped) {
        rememberError("ESTOP_LATCHED");
        linkWrite("ERR,ESTOP_LATCHED\n");
        return;
      }
      if (controlSource == ControlSource::Legacy) {
        rememberError("LEGACY_ACTIVE");
        linkWrite("ERR,LEGACY_ACTIVE,SEND_S_FIRST\n");
        return;
      }
      motion.disarm(nowMs);
      markSmoothCommandValid(nowMs);
      linkWrite("ACK,DISARM,PWM=HOLD\n");
      return;

    case robot_arm::SmoothCommandType::Move: {
      uint32_t acceptedDurationMs = 0U;
      const robot_arm::MotionResult result = motion.commandSynchronizedMove(
        command.anglesDeg,
        command.angleCount,
        command.durationMs,
        command.mode,
        command.motionId,
        false,
        nowMs,
        &acceptedDurationMs);
      if (result == robot_arm::MotionResult::Accepted) {
        markSmoothCommandValid(nowMs);
        writeSmoothOutputs();
      }
      sendMotionReply("MOVE", result, acceptedDurationMs, command.motionId);
      return;
    }

    case robot_arm::SmoothCommandType::Joint: {
      uint32_t acceptedDurationMs = 0U;
      const robot_arm::MotionResult result = motion.commandJointMove(
        command.jointIndex,
        command.anglesDeg[0],
        command.durationMs,
        command.mode,
        command.motionId,
        false,
        nowMs,
        &acceptedDurationMs);
      if (result == robot_arm::MotionResult::Accepted) {
        markSmoothCommandValid(nowMs);
        writeSmoothOutputs();
      }
      sendMotionReply("JOINT", result, acceptedDurationMs, command.motionId);
      return;
    }

    case robot_arm::SmoothCommandType::Neutral: {
      float neutralDeg[robot_arm::kServoCount] = {};
      for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
        neutralDeg[index] = 0.0f;
      }
      uint32_t acceptedDurationMs = 0U;
      const robot_arm::MotionResult result = motion.commandSynchronizedMove(
        neutralDeg,
        robot_arm::kServoCount,
        command.durationMs,
        command.mode,
        command.motionId,
        true,
        nowMs,
        &acceptedDurationMs);
      if (result == robot_arm::MotionResult::Accepted) {
        markSmoothCommandValid(nowMs);
        writeSmoothOutputs();
      }
      sendMotionReply("NEUTRAL", result, acceptedDurationMs, command.motionId);
      return;
    }

    case robot_arm::SmoothCommandType::Stop: {
      if (controlSource == ControlSource::Legacy) {
        rememberError("LEGACY_ACTIVE");
        linkWrite("ERR,LEGACY_ACTIVE,USE_S\n");
        return;
      }
      const robot_arm::MotionResult result = motion.stop(nowMs);
      if (result == robot_arm::MotionResult::Accepted) {
        markSmoothCommandValid(nowMs);
        if (pwmEnabled) {
          writeSmoothOutputs();
        }
        linkWrite("ACK,STOP,PWM=HOLD,QUEUE=CLEARED\n");
      } else {
        sendMotionReply("STOP", result, 0U, 0U);
      }
      return;
    }

    case robot_arm::SmoothCommandType::EmergencyStop:
      motion.emergencyStop(nowMs);
      markSmoothCommandValid(nowMs);
      if (robot_arm::kEstopDisablesPwm) {
        stopAllPwm();
        linkWrite("ACK,ESTOP,PWM=OFF,QUEUE=CLEARED\n");
      } else {
        if (pwmEnabled && controlSource == ControlSource::Smooth) {
          writeSmoothOutputs();
        }
        linkWrite("ACK,ESTOP,PWM=HOLD,QUEUE=CLEARED\n");
      }
      return;

    case robot_arm::SmoothCommandType::ResetEmergencyStop: {
      const robot_arm::MotionResult result = motion.resetEmergencyStop();
      if (result == robot_arm::MotionResult::Accepted) {
        markSmoothCommandValid(nowMs);
        linkWrite("ACK,RESET_ESTOP,STATE=DISARMED\n");
      } else {
        sendMotionReply("RESET_ESTOP", result, 0U, 0U);
      }
      return;
    }

    case robot_arm::SmoothCommandType::ClearQueue:
      motion.clearQueue();
      markSmoothCommandValid(nowMs);
      linkWrite("ACK,CLEAR_QUEUE\n");
      return;

    case robot_arm::SmoothCommandType::Demo:
      markSmoothCommandValid(nowMs);
      startDemo(nowMs);
      return;

    default:
      rememberError("UNKNOWN_COMMAND");
      linkWrite("ERR,UNKNOWN_COMMAND\n");
      return;
  }
}

// Faz 7 / A1 — servo-rail measurement commands.
//
// A separate command family on purpose. These are diagnostics: they never
// command motion, and they deliberately do NOT refresh the smooth-motion
// communication timeout. A host that only polls the rail is not proving the
// control link is alive, and must not be able to hold the arm armed.
// ---------------------------------------------------------------------------

const char * railTriggerStateName(robot_arm::RailTriggerState state) {
  switch (state) {
    case robot_arm::RailTriggerState::Idle: return "IDLE";
    case robot_arm::RailTriggerState::Armed: return "ARMED";
    case robot_arm::RailTriggerState::Capturing: return "CAPTURING";
    case robot_arm::RailTriggerState::Complete: return "COMPLETE";
  }
  return "UNKNOWN";
}

bool isRailLine(const char * lineText) {
  return std::strncmp(lineText, "RAIL", 4U) == 0;
}

void sendRailStatus() {
  uint32_t last = 0U;
  uint32_t minimum = 0U;
  uint32_t maximum = 0U;
  uint32_t mean = 0U;
  uint32_t recent = 0U;
  uint32_t samples = 0U;
  robot_arm::RailTriggerState state = robot_arm::RailTriggerState::Idle;

  taskENTER_CRITICAL(&railMux);
  last = railMonitor.lastMv();
  minimum = railMonitor.minMv();
  maximum = railMonitor.maxMv();
  mean = railMonitor.meanMv();
  recent = railMonitor.minRecentMv();
  samples = railMonitor.sampleCount();
  state = railMonitor.triggerState();
  taskEXIT_CRITICAL(&railMux);

  linkPrintf(
    "RAIL,MV=%u,MIN=%u,MAX=%u,MEAN=%u,MIN1S=%u,N=%u,TRIG=%s,PIN=%d,SAMPLER=%s\n",
    static_cast<unsigned>(last),
    static_cast<unsigned>(minimum),
    static_cast<unsigned>(maximum),
    static_cast<unsigned>(mean),
    static_cast<unsigned>(recent),
    static_cast<unsigned>(samples),
    railTriggerStateName(state),
    robot_arm::kRailAdcPin,
    railSamplerStarted ? "RUNNING" : "FAILED");
}

void sendRailDump() {
  // Only a frozen capture may be dumped. That invariant is what lets the dump
  // run without holding the sampler's lock for the whole transfer: once the
  // trigger completes, addSample() stops writing the ring buffer.
  taskENTER_CRITICAL(&railMux);
  const robot_arm::RailTriggerState state = railMonitor.triggerState();
  taskEXIT_CRITICAL(&railMux);

  if (state != robot_arm::RailTriggerState::Complete) {
    rememberError("RAIL_NO_CAPTURE");
    linkPrintf("ERR,RAIL_NO_CAPTURE,TRIG=%s\n", railTriggerStateName(state));
    return;
  }
  if (motion.isMoving()) {
    // The transfer takes seconds at 115200 baud. Refuse to spend them while
    // the arm is mid-trajectory; the capture is frozen and will keep.
    rememberError("RAIL_BUSY_MOVING");
    linkWrite("ERR,RAIL_BUSY_MOVING\n");
    return;
  }

  const std::size_t count = railMonitor.captureSize();
  linkPrintf(
    "RAILDUMP,BEGIN,N=%u,TRIGGER=%u,THRESHOLD=%u\n",
    static_cast<unsigned>(count),
    static_cast<unsigned>(railMonitor.triggerIndex()),
    static_cast<unsigned>(railMonitor.triggerThresholdMv()));
  for (std::size_t index = 0U; index < count; ++index) {
    linkPrintf(
      "RD,%u,%ld,%u,%u\n",
      static_cast<unsigned>(index),
      static_cast<long>(railMonitor.captureRelativeUsAt(index)),
      static_cast<unsigned>(railMonitor.captureRawAt(index)),
      static_cast<unsigned>(railMonitor.captureMvAt(index)));
  }
  linkWrite("RAILDUMP,END\n");
}

void handleRailLine(const char * lineText) {
  if (std::strcmp(lineText, "RAIL") == 0) {
    sendRailStatus();
    return;
  }
  if (std::strcmp(lineText, "RAILRESET") == 0) {
    taskENTER_CRITICAL(&railMux);
    railMonitor.reset(static_cast<uint32_t>(esp_timer_get_time()));
    taskEXIT_CRITICAL(&railMux);
    linkWrite("ACK,RAILRESET\n");
    return;
  }
  if (std::strcmp(lineText, "RAILDUMP") == 0) {
    sendRailDump();
    return;
  }
  if (std::strcmp(lineText, "RAILTRIG,OFF") == 0) {
    taskENTER_CRITICAL(&railMux);
    railMonitor.disarmTrigger();
    taskEXIT_CRITICAL(&railMux);
    linkWrite("ACK,RAILTRIG,OFF\n");
    return;
  }
  if (std::strncmp(lineText, "RAILTRIG,", 9U) == 0) {
    const char * argument = lineText + 9U;
    char * end = nullptr;
    errno = 0;
    const unsigned long threshold = std::strtoul(argument, &end, 10);
    if (end == argument || *end != '\0' || errno == ERANGE ||
      threshold == 0UL || threshold > 60000UL)
    {
      rememberError("RAIL_INVALID_THRESHOLD");
      linkWrite("ERR,RAIL_INVALID_THRESHOLD\n");
      return;
    }
    taskENTER_CRITICAL(&railMux);
    railMonitor.armTrigger(static_cast<uint32_t>(threshold));
    taskEXIT_CRITICAL(&railMux);
    linkPrintf("ACK,RAILTRIG,MV=%u\n", static_cast<unsigned>(threshold));
    return;
  }

  rememberError("RAIL_UNKNOWN_COMMAND");
  linkWrite("ERR,RAIL_UNKNOWN_COMMAND\n");
}

void railSamplerTask(void *) {
  uint32_t nextDueUs = static_cast<uint32_t>(esp_timer_get_time());
  for (;;) {
    const uint32_t nowUs = static_cast<uint32_t>(esp_timer_get_time());
    // Unsigned comparison against the deadline stays correct across the
    // 32-bit microsecond wrap (~71 minutes).
    if ((nowUs - nextDueUs) < 0x80000000U) {
      const int raw = analogRead(robot_arm::kRailAdcPin);
      taskENTER_CRITICAL(&railMux);
      railMonitor.addSample(
        raw < 0 ? 0U : static_cast<uint32_t>(raw), nowUs);
      taskEXIT_CRITICAL(&railMux);

      nextDueUs += robot_arm::kRailSamplePeriodUs;
      // If the task was starved, resynchronise instead of emitting a burst of
      // back-dated samples. Same policy the servo scheduler already uses.
      if ((nowUs - nextDueUs) < 0x80000000U) {
        nextDueUs = nowUs + robot_arm::kRailSamplePeriodUs;
      }
    }
    taskYIELD();
  }
}

void handleLine(const char * lineText) {
  if (robot_arm::isLegacyProtocolLine(lineText)) {
    handleLegacyLine(lineText);
  } else if (isRailLine(lineText)) {
    handleRailLine(lineText);
  } else {
    handleSmoothLine(lineText);
  }
}

void processRx() {
  static char lineBuffer[robot_arm::kMaxLineLength + 1U] = {};
  static std::size_t length = 0U;
  static bool overflowed = false;

  while (uartLink.available() > 0) {
    const int input = uartLink.read();
    if (input < 0) {
      break;
    }
    const char value = static_cast<char>(input);
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      if (overflowed) {
        rememberError("LINE_TOO_LONG");
        linkWrite("ERR,LINE_TOO_LONG\n");
      } else {
        lineBuffer[length] = '\0';
        handleLine(lineBuffer);
      }
      length = 0U;
      overflowed = false;
      continue;
    }
    if (overflowed) {
      continue;
    }
    if (length < robot_arm::kMaxLineLength) {
      lineBuffer[length++] = value;
    } else {
      overflowed = true;
    }
  }
}

void updateServoMotion(uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - lastServoUpdateMs;
  if (elapsedMs < robot_arm::kServoUpdateIntervalMs) {
    return;
  }

  // Do not run an unbounded catch-up loop after a stall. One fresh update is
  // safer than bursting many old 20 ms samples onto the servos.
  if (elapsedMs > 4U * robot_arm::kServoUpdateIntervalMs) {
    lastServoUpdateMs = nowMs;
  } else {
    lastServoUpdateMs += robot_arm::kServoUpdateIntervalMs;
  }

  if (controlSource != ControlSource::Smooth) {
    return;
  }
  if (motion.update(nowMs) && pwmEnabled) {
    writeSmoothOutputs();
  }

  const robot_arm::MotionEvent event = motion.takeCompletionEvent();
  if (event.available) {
    linkPrintf(
      "EVENT,MOTION_COMPLETE,ID=%lu\n",
      static_cast<unsigned long>(event.motionId));
  }
}

void updateCommunicationSafety(uint32_t nowMs) {
  if (controlSource == ControlSource::Legacy && pwmEnabled &&
    robot_arm::elapsedAtLeast(nowMs, lastLegacyPositionMs, kLegacyWatchdogMs))
  {
    // PWM is deliberately untouched here. See the policy document: on a
    // brakeless arm, cutting the pulse on a dead link drops the arm, which is
    // worse than latched torque. The host, while it is alive, has a better
    // answer -- it reads this line and disarms in a controlled way.
    if (!legacyWatchdogReported ||
      robot_arm::elapsedAtLeast(nowMs, lastWatchdogAnnounceMs, kWatchdogAnnounceMs))
    {
      legacyWatchdogReported = true;
      communicationLost = true;
      lastWatchdogAnnounceMs = nowMs;
      linkWrite("E3\n");
    }
  }

  if (controlSource == ControlSource::Smooth &&
    motion.state() == robot_arm::RobotState::Ready &&
    !smoothTimeoutReported &&
    robot_arm::elapsedAtLeast(
      nowMs, lastSmoothCommandMs, robot_arm::kCommunicationTimeoutMs))
  {
    motion.communicationTimeout(nowMs);
    if (pwmEnabled) {
      writeSmoothOutputs();
    }
    smoothTimeoutReported = true;
    communicationLost = true;
    rememberError("COMM_TIMEOUT");
    linkWrite("EVENT,COMM_TIMEOUT,STATE=DISARMED,PWM=HOLD\n");
  }
}

void updateHeartbeat(uint32_t nowMs) {
  static uint32_t lastLedMs = 0U;
  static bool ledState = false;
  const uint32_t interval =
    communicationLost ? kHeartbeatAlarmMs : kHeartbeatNormalMs;
  if ((nowMs - lastLedMs) >= interval) {
    lastLedMs = nowMs;
    ledState = !ledState;
    digitalWrite(kLedPin, ledState ? HIGH : LOW);
  }
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  uartLink.begin(115200);
  for (std::size_t index = 0U; index < robot_arm::kServoCount; ++index) {
    ledcSetup(
      robot_arm::kServoConfig[index].ledcChannel,
      kPwmFrequencyHz,
      kPwmResolutionBits);
    ledcAttachPin(
      robot_arm::kServoConfig[index].pin,
      robot_arm::kServoConfig[index].ledcChannel);
  }
  stopAllPwm();
  lastServoUpdateMs = millis();

  // Faz 7 / A1 rail monitor. Nothing below emits a servo pulse.
  analogReadResolution(12);
  analogSetPinAttenuation(robot_arm::kRailAdcPin, ADC_11db);
  railMonitor.reset(static_cast<uint32_t>(esp_timer_get_time()));

  // Core 0 is dedicated to sampling; the Arduino loop and the 20 ms servo
  // scheduler stay on core 1 and keep their timing. The idle watchdog on core
  // 0 is disabled because a 20 kHz sampler intentionally never lets the idle
  // task run there. This firmware brings up no WiFi or Bluetooth, so nothing
  // else needs core 0. If either radio is ever enabled, this task must move to
  // a timer-driven design first.
  disableCore0WDT();
  // ESP-IDF sizes the stack in bytes, not words.
  railSamplerStarted = xTaskCreatePinnedToCore(
    railSamplerTask, "rail", 4096, nullptr, 1, nullptr, 0) == pdPASS;
}

void loop() {
  processRx();
  // Command handlers timestamp accepted input with millis(). Sample the loop
  // time afterwards so a millisecond tick during parsing cannot make the
  // activity timestamp appear to be in the future and underflow a timeout.
  const uint32_t nowMs = millis();
  updateServoMotion(nowMs);
  updateCommunicationSafety(nowMs);
  updateHeartbeat(nowMs);
}
