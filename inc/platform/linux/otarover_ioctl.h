/**
 * @file   otarover_ioctl.h
 * @author Otavio Ribeiro
 * @date   10 Jan 2018
 * @brief  Kernel IOCTL definitions
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

#ifndef __OTAROVER_IOCTL_H__
#define __OTAROVER_IOCTL_H__

  #define OTAROVER_IOC_MAGIC                'O'

  #define OTAROVER_IOCTL_GET_M_ENABLE       _IOR(OTAROVER_IOC_MAGIC, 1, long*)
  #define OTAROVER_IOCTL_SET_M_ENABLE       _IOW(OTAROVER_IOC_MAGIC, 2, long*)
  #define OTAROVER_IOCTL_GET_M1_SPEED       _IOR(OTAROVER_IOC_MAGIC, 3, long*)
  #define OTAROVER_IOCTL_SET_M1_SPEED       _IOW(OTAROVER_IOC_MAGIC, 4, long*)
  #define OTAROVER_IOCTL_GET_M2_SPEED       _IOR(OTAROVER_IOC_MAGIC, 5, long*)
  #define OTAROVER_IOCTL_SET_M2_SPEED       _IOW(OTAROVER_IOC_MAGIC, 6, long*)
  #define OTAROVER_IOCTL_GET_M1_DIRECTION   _IOR(OTAROVER_IOC_MAGIC, 7, long*)
  #define OTAROVER_IOCTL_SET_M1_DIRECTION   _IOW(OTAROVER_IOC_MAGIC, 8, long*)
  #define OTAROVER_IOCTL_GET_M2_DIRECTION   _IOR(OTAROVER_IOC_MAGIC, 9, long*)
  #define OTAROVER_IOCTL_SET_M2_DIRECTION   _IOW(OTAROVER_IOC_MAGIC, 10, long*)
  #define OTAROVER_IOCTL_GET_M1_CONFIG      _IOR(OTAROVER_IOC_MAGIC, 11, long*)
  #define OTAROVER_IOCTL_SET_M1_CONFIG      _IOW(OTAROVER_IOC_MAGIC, 12, long*)
  #define OTAROVER_IOCTL_GET_M2_CONFIG      _IOR(OTAROVER_IOC_MAGIC, 13, long*)
  #define OTAROVER_IOCTL_SET_M2_CONFIG      _IOW(OTAROVER_IOC_MAGIC, 14, long*)
  #define OTAROVER_IOCTL_READ_SENSORS       _IOR(OTAROVER_IOC_MAGIC, 15, long*)
  #define OTAROVER_IOCTL_MAX_CMD            15

  #define OTAROVER_IOCTL_DC_MOTOR_ENABLE    1
  #define OTAROVER_IOCTL_DC_MOTOR_DISABLE   0

  #define OTAROVER_IOCTL_DIR_FORWARD        1
  #define OTAROVER_IOCTL_DIR_STOPPED        0
  #define OTAROVER_IOCTL_DIR_BACKWARD       -1

  #define OTAROVER_IOCTL_CONFIG_NORMAL      1
  #define OTAROVER_IOCTL_CONFIG_REVERSE     -1

  typedef struct sensor_data {
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
  } sensor_data_t;

#endif //__OTAROVER_IOCTL_H__
