#include "smooth_protocol.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace robot_arm {
namespace {

constexpr std::size_t kMaxFields = kMaxServoCount + 5U;

bool equal(const char * left, const char * right) {
  return std::strcmp(left, right) == 0;
}

bool parseFloat(const char * text, float * value) {
  if (text == nullptr || value == nullptr || *text == '\0') {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const float parsed = std::strtof(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

bool parseUint32(const char * text, uint32_t * value) {
  if (text == nullptr || value == nullptr || *text == '\0' || *text == '-') {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE ||
    parsed > static_cast<unsigned long>(std::numeric_limits<uint32_t>::max()))
  {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseMode(const char * text, MotionMode * mode) {
  if (equal(text, "REPLACE")) {
    *mode = MotionMode::Replace;
    return true;
  }
  if (equal(text, "QUEUE")) {
    *mode = MotionMode::Queue;
    return true;
  }
  return false;
}

SmoothParseResult splitFields(
  char * buffer, char ** fields, std::size_t * fieldCount)
{
  std::size_t count = 0U;
  char * fieldStart = buffer;
  for (char * cursor = buffer;; ++cursor) {
    if (*cursor == ',' || *cursor == '\0') {
      if (cursor == fieldStart) {
        return SmoothParseResult::InvalidField;
      }
      if (count >= kMaxFields) {
        return SmoothParseResult::WrongFieldCount;
      }
      fields[count++] = fieldStart;
      if (*cursor == '\0') {
        *fieldCount = count;
        return SmoothParseResult::Ok;
      }
      *cursor = '\0';
      fieldStart = cursor + 1;
    }
  }
}

SmoothParseResult parseMotionTail(
  char ** fields,
  std::size_t fieldCount,
  std::size_t durationIndex,
  SmoothCommand * command)
{
  if (!parseUint32(fields[durationIndex], &command->durationMs)) {
    return SmoothParseResult::InvalidNumber;
  }
  if (!parseMode(fields[durationIndex + 1U], &command->mode)) {
    return SmoothParseResult::InvalidMode;
  }
  command->motionId = 0U;
  if (fieldCount == durationIndex + 3U &&
    !parseUint32(fields[durationIndex + 2U], &command->motionId))
  {
    return SmoothParseResult::InvalidNumber;
  }
  return SmoothParseResult::Ok;
}

}  // namespace

bool isLegacyProtocolLine(const char * line) {
  if (line == nullptr) {
    return false;
  }
  const char second = line[1];
  const bool legacyPosition =
    line[0] == 'P' &&
    (second == '\0' || second == '+' || second == '-' ||
    (second >= '0' && second <= '9'));
  return
    std::strcmp(line, "V?") == 0 ||
    // `K?` is answered on the legacy path, so it has to be routed there. It was
    // added to handleLegacyLine without being added here, which left the
    // handler unreachable: the line fell through to the smooth parser and came
    // back as ERR,UNKNOWN_COMMAND. Measured on the arm, 2026-08-15.
    std::strcmp(line, "K?") == 0 ||
    std::strcmp(line, "S") == 0 ||
    legacyPosition;
}

SmoothParseResult parseSmoothCommand(
  const char * line, std::size_t servoCount, SmoothCommand * command)
{
  if (line == nullptr || command == nullptr || *line == '\0') {
    return SmoothParseResult::Empty;
  }
  if (servoCount == 0U || servoCount > kMaxServoCount) {
    return SmoothParseResult::InvalidField;
  }

  const std::size_t length = std::strlen(line);
  if (length > kMaxLineLength) {
    return SmoothParseResult::LineTooLong;
  }
  char buffer[kMaxLineLength + 1U] = {};
  std::memcpy(buffer, line, length + 1U);
  char * fields[kMaxFields] = {};
  std::size_t fieldCount = 0U;
  const SmoothParseResult splitResult = splitFields(buffer, fields, &fieldCount);
  if (splitResult != SmoothParseResult::Ok) {
    return splitResult;
  }

  std::memset(command, 0, sizeof(*command));
  command->type = SmoothCommandType::Invalid;
  command->mode = MotionMode::Replace;

  if (equal(fields[0], "PING") || equal(fields[0], "STATUS") ||
    equal(fields[0], "DISARM") || equal(fields[0], "STOP") ||
    equal(fields[0], "ESTOP") || equal(fields[0], "RESET_ESTOP") ||
    equal(fields[0], "CLEAR_QUEUE") || equal(fields[0], "DEMO"))
  {
    if (fieldCount != 1U) {
      return SmoothParseResult::WrongFieldCount;
    }
    if (equal(fields[0], "PING")) command->type = SmoothCommandType::Ping;
    else if (equal(fields[0], "STATUS")) command->type = SmoothCommandType::Status;
    else if (equal(fields[0], "DISARM")) command->type = SmoothCommandType::Disarm;
    else if (equal(fields[0], "STOP")) command->type = SmoothCommandType::Stop;
    else if (equal(fields[0], "ESTOP")) command->type = SmoothCommandType::EmergencyStop;
    else if (equal(fields[0], "RESET_ESTOP")) {
      command->type = SmoothCommandType::ResetEmergencyStop;
    } else if (equal(fields[0], "CLEAR_QUEUE")) {
      command->type = SmoothCommandType::ClearQueue;
    } else {
      command->type = SmoothCommandType::Demo;
    }
    return SmoothParseResult::Ok;
  }

  if (equal(fields[0], "ARM")) {
    if (fieldCount != servoCount + 1U) {
      return SmoothParseResult::WrongFieldCount;
    }
    command->type = SmoothCommandType::Arm;
    command->angleCount = servoCount;
    for (std::size_t index = 0U; index < servoCount; ++index) {
      if (!parseFloat(fields[index + 1U], &command->anglesDeg[index])) {
        return SmoothParseResult::InvalidNumber;
      }
    }
    return SmoothParseResult::Ok;
  }

  if (equal(fields[0], "MOVE")) {
    const std::size_t required = 1U + servoCount + 2U;
    if (fieldCount != required && fieldCount != required + 1U) {
      return SmoothParseResult::WrongFieldCount;
    }
    command->type = SmoothCommandType::Move;
    command->angleCount = servoCount;
    for (std::size_t index = 0U; index < servoCount; ++index) {
      if (!parseFloat(fields[index + 1U], &command->anglesDeg[index])) {
        return SmoothParseResult::InvalidNumber;
      }
    }
    return parseMotionTail(fields, fieldCount, servoCount + 1U, command);
  }

  if (equal(fields[0], "JOINT")) {
    if (fieldCount != 5U && fieldCount != 6U) {
      return SmoothParseResult::WrongFieldCount;
    }
    uint32_t oneBasedJoint = 0U;
    if (!parseUint32(fields[1], &oneBasedJoint) ||
      oneBasedJoint == 0U || oneBasedJoint > servoCount)
    {
      return SmoothParseResult::InvalidField;
    }
    command->type = SmoothCommandType::Joint;
    command->jointIndex = static_cast<std::size_t>(oneBasedJoint - 1U);
    if (!parseFloat(fields[2], &command->anglesDeg[0])) {
      return SmoothParseResult::InvalidNumber;
    }
    return parseMotionTail(fields, fieldCount, 3U, command);
  }

  if (equal(fields[0], "NEUTRAL")) {
    if (fieldCount < 2U || fieldCount > 4U) {
      return SmoothParseResult::WrongFieldCount;
    }
    command->type = SmoothCommandType::Neutral;
    command->angleCount = servoCount;
    if (!parseUint32(fields[1], &command->durationMs)) {
      return SmoothParseResult::InvalidNumber;
    }
    command->mode = MotionMode::Replace;
    command->motionId = 0U;
    if (fieldCount >= 3U && !parseMode(fields[2], &command->mode)) {
      return SmoothParseResult::InvalidMode;
    }
    if (fieldCount == 4U && !parseUint32(fields[3], &command->motionId)) {
      return SmoothParseResult::InvalidNumber;
    }
    return SmoothParseResult::Ok;
  }

  return SmoothParseResult::UnknownCommand;
}

const char * smoothParseResultName(SmoothParseResult result) {
  switch (result) {
    case SmoothParseResult::Ok: return "OK";
    case SmoothParseResult::Empty: return "EMPTY";
    case SmoothParseResult::LineTooLong: return "LINE_TOO_LONG";
    case SmoothParseResult::UnknownCommand: return "UNKNOWN_COMMAND";
    case SmoothParseResult::WrongFieldCount: return "FIELD_COUNT";
    case SmoothParseResult::InvalidField: return "INVALID_FIELD";
    case SmoothParseResult::InvalidNumber: return "INVALID_NUMBER";
    case SmoothParseResult::InvalidMode: return "INVALID_MODE";
    default: return "PARSE";
  }
}

}  // namespace robot_arm
