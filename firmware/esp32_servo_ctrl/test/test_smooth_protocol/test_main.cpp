#include "smooth_protocol.hpp"

#include <unity.h>

#include <cstring>

namespace {

void testMoveAndOptionalId() {
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::Ok),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "MOVE,1,-2,3,-4,5,6,2500,REPLACE,42",
      robot_arm::kServoCount,
      &command)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothCommandType::Move),
    static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_UINT32(2500U, command.durationMs);
  TEST_ASSERT_EQUAL_UINT32(42U, command.motionId);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, -4.0f, command.anglesDeg[3U]);
}

void testLegacyRoutingDoesNotStealPingOrStatus() {
  TEST_ASSERT_TRUE(robot_arm::isLegacyProtocolLine("V?"));
  TEST_ASSERT_TRUE(robot_arm::isLegacyProtocolLine("S"));
  TEST_ASSERT_TRUE(
    robot_arm::isLegacyProtocolLine("P500,500,500,500,500,500"));
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("PING"));
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("STATUS"));
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("PONG"));
}

// Every request handleLegacyLine answers must route to it. This assertion is
// the one that was missing when `K?` was added: the handler was written and
// the routing table was not, so the calibration gate could not be asked on
// real hardware even though the firmware contained the code to answer it.
void testCalibrationRequestRoutesToLegacyHandler() {
  TEST_ASSERT_TRUE(robot_arm::isLegacyProtocolLine("K?"));
  // Neighbours that must NOT be swallowed by the legacy path.
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("K"));
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("K?X"));
  TEST_ASSERT_FALSE(robot_arm::isLegacyProtocolLine("KEEPALIVE"));
}

void testArmRequiresEveryReference() {
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::WrongFieldCount),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "ARM,0,0,0", robot_arm::kServoCount, &command)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::Ok),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "ARM,0,0,0,0,0,0", robot_arm::kServoCount, &command)));
}

void testJointAndNeutral() {
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::Ok),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "JOINT,2,15.5,1200,QUEUE,7", robot_arm::kServoCount, &command)));
  TEST_ASSERT_EQUAL_UINT32(1U, static_cast<uint32_t>(command.jointIndex));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::MotionMode::Queue),
    static_cast<int>(command.mode));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::Ok),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "NEUTRAL,3000,REPLACE,9", robot_arm::kServoCount, &command)));
  TEST_ASSERT_EQUAL_UINT32(3000U, command.durationMs);
}

void testInvalidNumbersAndModesAreRejected() {
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::InvalidNumber),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "MOVE,nan,0,0,0,0,0,1000,REPLACE",
      robot_arm::kServoCount,
      &command)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::InvalidMode),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "JOINT,1,5,1000,MAYBE", robot_arm::kServoCount, &command)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::InvalidField),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "MOVE,0,,0,0,0,0,1000,REPLACE",
      robot_arm::kServoCount,
      &command)));
}

void testExtraFieldsAndUnknownCommandsAreRejected() {
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::WrongFieldCount),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "STOP,EXTRA", robot_arm::kServoCount, &command)));
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::UnknownCommand),
    static_cast<int>(robot_arm::parseSmoothCommand(
      "FLY,1", robot_arm::kServoCount, &command)));
}

void testLineLengthLimit() {
  char line[robot_arm::kMaxLineLength + 3U];
  std::memset(line, 'A', sizeof(line));
  line[sizeof(line) - 1U] = '\0';
  robot_arm::SmoothCommand command;
  TEST_ASSERT_EQUAL(
    static_cast<int>(robot_arm::SmoothParseResult::LineTooLong),
    static_cast<int>(
      robot_arm::parseSmoothCommand(line, robot_arm::kServoCount, &command)));
}

void testControlCommands() {
  const char * commands[] = {
    "PING", "STATUS", "DISARM", "STOP", "ESTOP",
    "RESET_ESTOP", "CLEAR_QUEUE", "DEMO",
  };
  for (const char * line : commands) {
    robot_arm::SmoothCommand command;
    TEST_ASSERT_EQUAL(
      static_cast<int>(robot_arm::SmoothParseResult::Ok),
      static_cast<int>(
        robot_arm::parseSmoothCommand(line, robot_arm::kServoCount, &command)));
  }
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(testMoveAndOptionalId);
  RUN_TEST(testLegacyRoutingDoesNotStealPingOrStatus);
  RUN_TEST(testCalibrationRequestRoutesToLegacyHandler);
  RUN_TEST(testArmRequiresEveryReference);
  RUN_TEST(testJointAndNeutral);
  RUN_TEST(testInvalidNumbersAndModesAreRejected);
  RUN_TEST(testExtraFieldsAndUnknownCommandsAreRejected);
  RUN_TEST(testLineLengthLimit);
  RUN_TEST(testControlCommands);
  return UNITY_END();
}
