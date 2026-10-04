#include "protocol.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

static uint16_t clamp_pulse(long value) {
  if (value < (long)SERVO_MIN_US) {
    return SERVO_MIN_US;
  }
  if (value > (long)SERVO_MAX_US) {
    return SERVO_MAX_US;
  }
  return (uint16_t)value;
}

protocol_result_t protocol_parse_line(const char *line, protocol_command_t *command) {
  if (line == NULL || command == NULL) {
    return PROTOCOL_ERROR_PARSE;
  }

  memset(command, 0, sizeof(*command));
  if (strcmp(line, "V?") == 0) {
    command->type = PROTOCOL_COMMAND_VERSION;
    return PROTOCOL_OK;
  }
  if (strcmp(line, "S") == 0) {
    command->type = PROTOCOL_COMMAND_SAFE_STOP;
    return PROTOCOL_OK;
  }
  if (line[0] != 'P') {
    return PROTOCOL_ERROR_PARSE;
  }

  const char *cursor = line + 1;
  size_t count = 0U;
  while (*cursor != '\0') {
    if (count >= SERVO_CHANNEL_COUNT) {
      return PROTOCOL_ERROR_CHANNEL_COUNT;
    }

    errno = 0;
    char *end = NULL;
    const long value = strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || value < INT_MIN || value > INT_MAX) {
      return PROTOCOL_ERROR_PARSE;
    }
    command->pulse_us[count++] = clamp_pulse(value);

    if (*end == '\0') {
      cursor = end;
    } else if (*end == ',') {
      cursor = end + 1;
      if (*cursor == '\0') {
        return PROTOCOL_ERROR_PARSE;
      }
    } else {
      return PROTOCOL_ERROR_PARSE;
    }
  }

  if (count != SERVO_CHANNEL_COUNT) {
    return PROTOCOL_ERROR_CHANNEL_COUNT;
  }
  command->type = PROTOCOL_COMMAND_POSITION;
  return PROTOCOL_OK;
}

