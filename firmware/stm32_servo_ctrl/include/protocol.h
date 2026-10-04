#ifndef STM32_SERVO_PROTOCOL_H
#define STM32_SERVO_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERVO_CHANNEL_COUNT 6U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U

typedef enum {
  PROTOCOL_COMMAND_INVALID = 0,
  PROTOCOL_COMMAND_POSITION,
  PROTOCOL_COMMAND_VERSION,
  PROTOCOL_COMMAND_SAFE_STOP,
} protocol_command_type_t;

typedef enum {
  PROTOCOL_OK = 0,
  PROTOCOL_ERROR_PARSE = 1,
  PROTOCOL_ERROR_CHANNEL_COUNT = 2,
  PROTOCOL_ERROR_WATCHDOG = 3,
} protocol_result_t;

typedef struct {
  protocol_command_type_t type;
  uint16_t pulse_us[SERVO_CHANNEL_COUNT];
} protocol_command_t;

protocol_result_t protocol_parse_line(const char *line, protocol_command_t *command);

#endif

