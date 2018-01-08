#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "otarover_protocol.h"

int otarover_protocol_encode_int16(char** buffer, short int value)
{
  return 0;
}

int otarover_protocol_encode_int32(char** buffer, long int value)
{
  return 0;
}

int otarover_protocol_encode_string(char** buffer, const char* value, size_t length)
{
  return 0;
}

int otarover_protocol_decode_int16(const char* buffer, short int* value)
{
  *value = 0;
  *value |= buffer[0] << 8;
  *value |= buffer[1];
  return 2;
}

int otarover_protocol_decode_int32(const char* buffer, long int* value)
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

  count += otarover_protocol_decode_int16(&buffer[count], (short int*)&message->message_type);
  count += otarover_protocol_decode_int16(&buffer[count], (short int*)&message->cmd);
  count += otarover_protocol_decode_int16(&buffer[count], (short int*)&message->value_type);

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
