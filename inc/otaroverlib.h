/**
 * @file   otaroverlib.h
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
 */
#ifndef __OTAROVER_LIB_H__
#define __OTAROVER_LIB_H__

#include "platform/linux/otarover_ioctl.h"

#define  OTAROVER_DC_MOTOR1         1
#define  OTAROVER_DC_MOTOR2         2
#define  OTAROVER_DC_MOTOR3         3
#define  OTAROVER_DC_MOTOR4         4

#define OTAROVER_DC_MOTOR_ENABLE    1
#define OTAROVER_DC_MOTOR_DISABLE   0

#define OTAROVER_DIR_FORWARD        1
#define OTAROVER_DIR_STOPPED        0
#define OTAROVER_DIR_BACKWARD       -1

#define OTAROVER_CONFIG_NORMAL      1
#define OTAROVER_CONFIG_REVERSE     -1

typedef struct _otarover_context{
  int dev_fd;
} otarover_context_t;

extern otarover_context_t* otarover_init();
extern void otarover_close(otarover_context_t* context);
extern int otarover_dc_motor_set_enable(otarover_context_t* context, int enable);
extern int otarover_dc_motor_is_enable(otarover_context_t* context, int* enable);
extern int otarover_dc_motor_set_speed(otarover_context_t* context, int speed, int motor);
extern int otarover_dc_motor_get_speed(otarover_context_t* context, int* speed, int motor);
extern int otarover_dc_motor_set_direction(otarover_context_t* context, int dir, int motor);
extern int otarover_dc_motor_get_direction(otarover_context_t* context, int* dir, int motor);
extern int otarover_dc_motor_set_config(otarover_context_t* context, int speed, int motor);
extern int otarover_dc_motor_get_config(otarover_context_t* context, int* speed, int motor);
extern int otarover_read_sensors(otarover_context_t* context, sensor_info_t* info);

#endif //__OTAROVER_LIB_H__
