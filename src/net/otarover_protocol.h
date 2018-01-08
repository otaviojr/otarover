/**
 * @file   otarover_protocol.h
 * @author Otavio Ribeiro
 * @date   5 Jan 2018
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
 *
 */
#ifndef __OTAROVER_PROTOCOL_H__
#define __OTAROVER_PROTOCOL_H__

#define OTAROVER_PROTOCOL_MAGIC 0x7777

typedef enum _otarover_protocol_msg_type {
  OTAROVER_PROTOCOL_MSG_TYPE_MOV = 0x01,
  OTAROVER_PROTOCOL_MSG_TYPE_BAT = 0x02
} otarover_protocol_msg_type_t;

typedef enum _otarover_protocol_value_type {
  OTAROVER_PROTOCOL_VALUE_TYPE_INT16 = 0x01,
  OTAROVER_PROTOCOL_VALUE_TYPE_INT32 = 0x02,
  OTAROVER_PROTOCOL_VALUE_TYPE_STRING = 0x03
} otarover_protocol_value_type_t;

typedef enum _otarover_protocol_cmd {
  OTAROVER_PROTOCOL_CMD_DIRECTION = 0x01,
  OTAROVER_PROTOCOL_CMD_SPEED = 0x02
} otarover_protocol_cmd_t;

typedef struct _otarover_protocol {
  long int magic;
  long int message_length;
  otarover_protocol_msg_type_t message_type;
  otarover_protocol_cmd_t cmd;
  otarover_protocol_value_type_t value_type;
  union _value {
    long int int32_val;
    short int int16_val;
    char* str_val;
  } value;
} otarover_protocol_t;

int otarover_protocol_encode_int16(char** buffer, short int value);
int otarover_protocol_encode_int32(char** buffer, long int value);
int otarover_protocol_encode_string(char** buffer, const char* value, size_t length);

int otarover_protocol_decode_int16(const char* buffer, short int* value);
int otarover_protocol_decode_int32(const char* buffer, long int* value);
int otarover_protocol_decode_string(const char* buffer, char** value, size_t max_length);

int otarover_protocol_parse_message(char* buffer, otarover_protocol_t* message, size_t max_length);
int otarover_protocol_encode_message(char** buffer, otarover_protocol_t* message);

void otarover_protocol_destroy_message(otarover_protocol_t* message);

#endif //__OTAROVER_PROTOCOL_H__
