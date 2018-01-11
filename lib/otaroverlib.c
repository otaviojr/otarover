/**
 * @file   otaroverlib.c
 * @author Otavio Ribeiro
 * @date   5 Jan 2018
 * @brief  A userspace library to access the kernel module
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
 **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "otaroverlib.h"
#include "platform/linux/otarover_ioctl.h"

#define DEVICE_FILE_NAME "/dev/otarover"

otarover_context_t* otarover_init()
{
  otarover_context_t* context = (otarover_context_t*)malloc(sizeof(otarover_context_t));
  context->dev_fd = open(DEVICE_FILE_NAME,O_RDWR);
  if(context->dev_fd < 0){
    free(context);
    return NULL;
  }
  return context;
}

void otarover_close(otarover_context_t* context)
{
  if(context == NULL) return;
  close(context->dev_fd);
  free(context);
}

int otarover_dc_motor_set_enable(otarover_context_t* context, int enable)
{
  long val;

  val = OTAROVER_IOCTL_DC_MOTOR_DISABLE;
  if(enable > 0) val = OTAROVER_IOCTL_DC_MOTOR_ENABLE;

  return ioctl(context->dev_fd, OTAROVER_IOCTL_SET_M_ENABLE, &val);
}

int otarover_dc_motor_is_enable(otarover_context_t* context, int* enable)
{
  long val;
  int ret;

  if(enable == NULL) return -1;

  ret = ioctl(context->dev_fd, OTAROVER_IOCTL_GET_M_ENABLE, &val);

  *enable = (val == OTAROVER_IOCTL_DC_MOTOR_ENABLE ? OTAROVER_DC_MOTOR_ENABLE : OTAROVER_DC_MOTOR_DISABLE);

  return ret;
}

int otarover_dc_motor_set_speed(otarover_context_t* context, int speed, int motor)
{
  long val;
  int cmd;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_SET_M1_SPEED;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_SET_M2_SPEED;
  } else {
    return -1;
  }

  val = speed;

  if(val > 100) val = 100;

  /* For security reasons only. Should be removed later */
  //if(val > 50) val = 50;

  return ioctl(context->dev_fd, cmd, &val);
}

int otarover_dc_motor_get_speed(otarover_context_t* context, int* speed, int motor)
{
  int cmd;

  if(speed == NULL) return -1;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_GET_M1_SPEED;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_GET_M2_SPEED;
  } else {
    return -1;
  }

  return ioctl(context->dev_fd, cmd, speed);
}

int otarover_dc_motor_set_direction(otarover_context_t* context, int dir, int motor)
{
  long val;
  int cmd;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_SET_M1_DIRECTION;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_SET_M2_DIRECTION;
  } else {
    return -1;
  }

  if(dir == OTAROVER_DIR_FORWARD){
    val = OTAROVER_IOCTL_DIR_FORWARD;
  } else if(dir == OTAROVER_DIR_BACKWARD){
    val = OTAROVER_IOCTL_DIR_BACKWARD;
  } else {
    val = OTAROVER_IOCTL_DIR_STOPPED;
  }

  return ioctl(context->dev_fd, cmd, &val);
}

int otarover_dc_motor_get_direction(otarover_context_t* context, int* dir, int motor)
{
  long val;
  int ret, cmd;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_GET_M1_DIRECTION;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_GET_M2_DIRECTION;
  } else {
    return -1;
  }

  if(dir == NULL) return -1;

  ret = ioctl(context->dev_fd, cmd, &val);

  if(val == OTAROVER_IOCTL_DIR_FORWARD){
    *dir = OTAROVER_DIR_FORWARD;
  } else if(val == OTAROVER_IOCTL_DIR_BACKWARD){
    *dir = OTAROVER_DIR_BACKWARD;
  } else {
    *dir = OTAROVER_DIR_STOPPED;
  }

  return ret;
}

int otarover_dc_motor_set_config(otarover_context_t* context, int config, int motor)
{
  long val;
  int cmd;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_SET_M1_CONFIG;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_SET_M2_CONFIG;
  } else {
    return -1;
  }

  if(config == OTAROVER_CONFIG_NORMAL){
    val = OTAROVER_IOCTL_CONFIG_NORMAL;
  } else {
    val = OTAROVER_IOCTL_CONFIG_REVERSE;
  }

  return ioctl(context->dev_fd, cmd, &val);
}

int otarover_dc_motor_get_config(otarover_context_t* context, int* config, int motor)
{
  long val;
  int ret, cmd;

  if(motor == OTAROVER_DC_MOTOR1){
    cmd = OTAROVER_IOCTL_GET_M1_CONFIG;
  } else if(motor == OTAROVER_DC_MOTOR2){
    cmd = OTAROVER_IOCTL_GET_M2_CONFIG;
  } else {
    return -1;
  }

  if(config == NULL) return -1;

  ret = ioctl(context->dev_fd, cmd, &val);

  if(val == OTAROVER_IOCTL_CONFIG_NORMAL){
    *config = OTAROVER_CONFIG_NORMAL;
  } else {
    *config = OTAROVER_CONFIG_REVERSE;
  }

  return ret;
}
