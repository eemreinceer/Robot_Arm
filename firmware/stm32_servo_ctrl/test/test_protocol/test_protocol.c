#include "protocol.h"
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_position_command(void) {
  protocol_command_t command;
  TEST_ASSERT_EQUAL(PROTOCOL_OK,
                    protocol_parse_line("P500,1000,1500,2000,2500,1750", &command));
  TEST_ASSERT_EQUAL(PROTOCOL_COMMAND_POSITION, command.type);
  TEST_ASSERT_EQUAL_UINT16(500, command.pulse_us[0]);
  TEST_ASSERT_EQUAL_UINT16(1750, command.pulse_us[5]);
}

static void test_position_values_are_clamped(void) {
  protocol_command_t command;
  TEST_ASSERT_EQUAL(PROTOCOL_OK,
                    protocol_parse_line("P1,499,1500,2501,9999,2000", &command));
  TEST_ASSERT_EQUAL_UINT16(500, command.pulse_us[0]);
  TEST_ASSERT_EQUAL_UINT16(500, command.pulse_us[1]);
  TEST_ASSERT_EQUAL_UINT16(2500, command.pulse_us[3]);
  TEST_ASSERT_EQUAL_UINT16(2500, command.pulse_us[4]);
}

static void test_wrong_channel_count(void) {
  protocol_command_t command;
  TEST_ASSERT_EQUAL(PROTOCOL_ERROR_CHANNEL_COUNT,
                    protocol_parse_line("P1000,1000,1000", &command));
  TEST_ASSERT_EQUAL(PROTOCOL_ERROR_CHANNEL_COUNT,
                    protocol_parse_line("P1,2,3,4,5,6,7", &command));
}

static void test_malformed_position(void) {
  protocol_command_t command;
  TEST_ASSERT_EQUAL(PROTOCOL_ERROR_PARSE,
                    protocol_parse_line("P1,2,nope,4,5,6", &command));
  TEST_ASSERT_EQUAL(PROTOCOL_ERROR_PARSE,
                    protocol_parse_line("P1,2,3,4,5,", &command));
}

static void test_version_and_stop(void) {
  protocol_command_t command;
  TEST_ASSERT_EQUAL(PROTOCOL_OK, protocol_parse_line("V?", &command));
  TEST_ASSERT_EQUAL(PROTOCOL_COMMAND_VERSION, command.type);
  TEST_ASSERT_EQUAL(PROTOCOL_OK, protocol_parse_line("S", &command));
  TEST_ASSERT_EQUAL(PROTOCOL_COMMAND_SAFE_STOP, command.type);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_position_command);
  RUN_TEST(test_position_values_are_clamped);
  RUN_TEST(test_wrong_channel_count);
  RUN_TEST(test_malformed_position);
  RUN_TEST(test_version_and_stop);
  return UNITY_END();
}

