/**
 * @file   otarover_protocol.h
 * @author Otavio Ribeiro
 * @date   5 Jan 2018
 * @brief  Otarover simple UDP protocol
 *
 * Copyright (c) 2018 Otávio Ribeiro <otavio.ribeiro@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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
  int32_t magic;
  int32_t message_length;
  otarover_protocol_msg_type_t message_type;
  otarover_protocol_cmd_t cmd;
  otarover_protocol_value_type_t value_type;
  union _value {
    int32_t int32_val;
    int16_t int16_val;
    char* str_val;
  } value;
} otarover_protocol_t;

int otarover_protocol_encode_int16(char** buffer, int16_t value);
int otarover_protocol_encode_int32(char** buffer, int32_t value);
int otarover_protocol_encode_string(char** buffer, const char* value, size_t length);

int otarover_protocol_decode_int16(const char* buffer, int16_t* value);
int otarover_protocol_decode_int32(const char* buffer, int32_t* value);
int otarover_protocol_decode_string(const char* buffer, char** value, size_t max_length);

int otarover_protocol_parse_message(char* buffer, otarover_protocol_t* message, size_t max_length);
int otarover_protocol_encode_message(char** buffer, otarover_protocol_t* message);

void otarover_protocol_destroy_message(otarover_protocol_t* message);

#endif //__OTAROVER_PROTOCOL_H__
