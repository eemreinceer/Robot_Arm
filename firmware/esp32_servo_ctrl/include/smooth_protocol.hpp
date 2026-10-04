#pragma once

#include "motion_controller.hpp"
#include "robot_config.hpp"

#include <cstddef>
#include <cstdint>

namespace robot_arm {

enum class SmoothCommandType : uint8_t {
  Invalid = 0,
  Ping,
  Status,
  Arm,
  Disarm,
  Move,
  Joint,
  Neutral,
  Stop,
  EmergencyStop,
  ResetEmergencyStop,
  ClearQueue,
  Demo,
};

enum class SmoothParseResult : uint8_t {
  Ok = 0,
  Empty,
  LineTooLong,
  UnknownCommand,
  WrongFieldCount,
  InvalidField,
  InvalidNumber,
  InvalidMode,
};

struct SmoothCommand {
  SmoothCommandType type;
  float anglesDeg[kMaxServoCount];
  std::size_t angleCount;
  std::size_t jointIndex;
  uint32_t durationMs;
  MotionMode mode;
  uint32_t motionId;
};

bool isLegacyProtocolLine(const char * line);
SmoothParseResult parseSmoothCommand(
  const char * line, std::size_t servoCount, SmoothCommand * command);
const char * smoothParseResultName(SmoothParseResult result);

}  // namespace robot_arm
