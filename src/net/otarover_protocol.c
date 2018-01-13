/**
 * @file   otarover_protocol.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "otarover_protocol.h"

int otarover_protocol_encode_int16(char** buffer, int16_t value)
{
  return 0;
}

int otarover_protocol_encode_int32(char** buffer, int32_t value)
{
  return 0;
}

int otarover_protocol_encode_string(char** buffer, const char* value, size_t length)
{
  return 0;
}

int otarover_protocol_decode_int16(const char* buffer, int16_t* value)
{
  *value = 0;
  *value |= buffer[0] << 8;
  *value |= buffer[1];
  return 2;
}

int otarover_protocol_decode_int32(const char* buffer, int32_t* value)
{
  *value = 0;
  *value |= buffer[0] << 24;
  *value |= buffer[1] << 16;
  *value |= buffer[2] << 8;
  *value |= buffer[3];
  return 4;
}

int otarover_protocol_decode_string(const char* buffer, char** value, size_t max_length)
{
  return 0;
}

int otarover_protocol_parse_message(char* buffer, otarover_protocol_t* message, size_t max_length)
{
  int count = 0;

  memset(message,0,sizeof(otarover_protocol_t));

  count += otarover_protocol_decode_int32(&buffer[count], &message->magic);
  count += otarover_protocol_decode_int32(&buffer[count], &message->message_length);

  if(message->magic != OTAROVER_PROTOCOL_MAGIC){
    return 0;
  }

  if(message->message_length > max_length){
    return 0;
  }

  count += otarover_protocol_decode_int16(&buffer[count], (int16_t*)&message->message_type);
  count += otarover_protocol_decode_int16(&buffer[count], (int16_t*)&message->cmd);
  count += otarover_protocol_decode_int16(&buffer[count], (int16_t*)&message->value_type);

  if(message->value_type == OTAROVER_PROTOCOL_VALUE_TYPE_INT16){
    count += otarover_protocol_decode_int16(&buffer[count], &message->value.int16_val);
  } else if(message->value_type == OTAROVER_PROTOCOL_VALUE_TYPE_INT32){
    count += otarover_protocol_decode_int32(&buffer[count], &message->value.int32_val);
  }

  return count;
}

int otarover_protocol_encode_message(char** buffer, otarover_protocol_t* message)
{
  return 0;
}

void otarover_protocol_destroy_message(otarover_protocol_t* message)
{
  if(message->value_type == OTAROVER_PROTOCOL_VALUE_TYPE_STRING && message->value.str_val != NULL){
    free(message->value.str_val);
  }
}
