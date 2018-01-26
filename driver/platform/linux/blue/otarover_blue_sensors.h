/**
 * @file   otarover_blue_sensors.h
 * @author Otavio Ribeiro
 * @date   11 Jan 2018
 * @brief  A kernel module for controlling beaglebone blue board
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

#ifndef __OTAROVER_BLUE_SENSORS_H__
#define __OTAROVER_BLUE_SENSORS_H__

  #include "otarover_ioctl.h"

  #define MPU9250_ADDRESS                   0x68
  #define AK8963_ADDRESS                    0x0C

  #define MPU9250_WHOAMI_VALUE              0x71
  #define AK8963_WHOAMI_VALUE               0x48

  #define MPU9250_REG_GYRO_X_SELT_TEST      0x00
  #define MPU9250_REG_GYRO_Y_SELT_TEST      0x01
  #define MPU9250_REG_GYRO_Z_SELT_TEST      0x02
  #define MPU9250_REG_ACCEL_X_SELT_TEST     0x0D
  #define MPU9250_REG_ACCEL_Y_SELT_TEST     0x0E
  #define MPU9250_REG_ACCEL_Z_SELT_TEST     0x0F
  #define MPU9250_REG_X_OFFS_USER_H         0x13
  #define MPU9250_REG_X_OFFS_USER_L         0x14
  #define MPU9250_REG_Y_OFFS_USER_H         0x15
  #define MPU9250_REG_Y_OFFS_USER_L         0x16
  #define MPU9250_REG_Z_OFFS_USER_H         0x17
  #define MPU9250_REG_Z_OFFS_USER_L         0x18
  #define MPU9250_REG_SMPLRT_DIV            0x19
  #define MPU9250_REG_CONFIG                0x1A
  #define MPU9250_REG_GYRO_CONFIG           0x1B
  #define MPU9250_REG_ACCEL_CONFIG1         0x1C
  #define MPU9250_REG_ACCEL_CONFIG2         0x1D
  #define MPU9250_REG_LOW_PWR_ODR           0x1E
  #define MPU9250_REG_WAKE_ON_MOT_THRES     0x1F
  #define MPU9250_REG_FIFO_EN               0x23
  #define MPU9250_REG_I2C_MASTER_CTRL       0x24
  #define MPU9250_REG_I2C_SLAVE0_ADDR       0x25
  #define MPU9250_REG_I2C_SLAVE0_REG        0x26
  #define MPU9250_REG_I2C_SLAVE0_CTRL       0x27
  #define MPU9250_REG_I2C_SLAVE1_ADDR       0x28
  #define MPU9250_REG_I2C_SLAVE1_REG        0x29
  #define MPU9250_REG_I2C_SLAVE1_CTRL       0x2A
  #define MPU9250_REG_I2C_SLAVE2_ADDR       0x2B
  #define MPU9250_REG_I2C_SLAVE2_REG        0x2C
  #define MPU9250_REG_I2C_SLAVE2_CTRL       0x2D
  #define MPU9250_REG_I2C_SLAVE3_ADDR       0x2E
  #define MPU9250_REG_I2C_SLAVE3_REG        0x2F
  #define MPU9250_REG_I2C_SLAVE3_CTRL       0x30
  #define MPU9250_REG_I2C_SLAVE4_ADDR       0x31
  #define MPU9250_REG_I2C_SLAVE4_REG        0x32
  #define MPU9250_REG_I2C_SLAVE4_DO         0x33
  #define MPU9250_REG_I2C_SLAVE4_CTRL       0x34
  #define MPU9250_REG_I2C_SLAVE4_DI         0x35
  #define MPU9250_REG_I2C_MASTER_STATUS     0x36
  #define MPU9250_REG_INT_PIN               0x37
  #define MPU9250_REG_INT_EN                0x38
  #define MPU9250_REG_INT_STATUS            0x3A

  #define MPU9250_REG_ACCEL_XOUT_H          0x3B
  #define MPU9250_REG_ACCEL_XOUT_L          0x3C
  #define MPU9250_REG_ACCEL_YOUT_H          0x3D
  #define MPU9250_REG_ACCEL_YOUT_L          0x3E
  #define MPU9250_REG_ACCEL_ZOUT_H          0x3F
  #define MPU9250_REG_ACCEL_ZOUT_L          0x40

  #define MPU9250_REG_TEMP_OUT_H            0x41
  #define MPU9250_REG_TEMP_OUT_L            0x42

  #define MPU9250_REG_GYRO_XOUT_H           0x43
  #define MPU9250_REG_GYRO_XOUT_L           0x44
  #define MPU9250_REG_GYRO_YOUT_H           0x45
  #define MPU9250_REG_GYRO_YOUT_L           0x46
  #define MPU9250_REG_GYRO_ZOUT_H           0x47
  #define MPU9250_REG_GYRO_ZOUT_L           0x48

  #define MPU9250_REG_EXT_SENS_DATA_00      0x49
  #define MPU9250_REG_EXT_SENS_DATA_01      0x4A
  #define MPU9250_REG_EXT_SENS_DATA_02      0x4B
  #define MPU9250_REG_EXT_SENS_DATA_03      0x4C
  #define MPU9250_REG_EXT_SENS_DATA_04      0x4D
  #define MPU9250_REG_EXT_SENS_DATA_05      0x4E
  #define MPU9250_REG_EXT_SENS_DATA_06      0x4F
  #define MPU9250_REG_EXT_SENS_DATA_07      0x50
  #define MPU9250_REG_EXT_SENS_DATA_08      0x51
  #define MPU9250_REG_EXT_SENS_DATA_09      0x52
  #define MPU9250_REG_EXT_SENS_DATA_10      0x53
  #define MPU9250_REG_EXT_SENS_DATA_11      0x54
  #define MPU9250_REG_EXT_SENS_DATA_12      0x55
  #define MPU9250_REG_EXT_SENS_DATA_13      0x56
  #define MPU9250_REG_EXT_SENS_DATA_14      0x57
  #define MPU9250_REG_EXT_SENS_DATA_15      0x58
  #define MPU9250_REG_EXT_SENS_DATA_16      0x59
  #define MPU9250_REG_EXT_SENS_DATA_17      0x5A
  #define MPU9250_REG_EXT_SENS_DATA_18      0x5B
  #define MPU9250_REG_EXT_SENS_DATA_19      0x5C
  #define MPU9250_REG_EXT_SENS_DATA_20      0x5D
  #define MPU9250_REG_EXT_SENS_DATA_21      0x5E
  #define MPU9250_REG_EXT_SENS_DATA_22      0x5F
  #define MPU9250_REG_EXT_SENS_DATA_23      0x60

  #define MPU9250_REG_I2C_SLAVE0_DO         0x63
  #define MPU9250_REG_I2C_SLAVE1_DO         0x64
  #define MPU9250_REG_I2C_SLAVE2_DO         0x65
  #define MPU9250_REG_I2C_SLAVE3_DO         0x66
  #define MPU9250_REG_I2C_MST_DELAY_CTRL    0x67
  #define MPU9250_REG_SIGNAL_PATH_RESET     0x68
  #define MPU9250_REG_ACCEL_INTEL_CTRL      0x69
  #define MPU9250_REG_USER_CTRL             0x6A
  #define MPU9250_REG_PWR_MGMT_1            0x6B
  #define MPU9250_REG_PWR_MGMT_2            0x6C
  #define MPU9250_REG_FIFO_COUNT_H          0x72
  #define MPU9250_REG_FIFO_COUNT_L          0x73
  #define MPU9250_REG_FIFO_R_W              0x74
  #define MPU9250_REG_WHOAMI                0x75

  #define MPU9250_REG_XA_OFFS_H             0x77
  #define MPU9250_REG_XA_OFFS_L             0x78
  #define MPU9250_REG_YA_OFFS_H             0x7A
  #define MPU9250_REG_YA_OFFS_L             0x7B
  #define MPU9250_REG_ZA_OFFS_H             0x7D
  #define MPU9250_REG_ZA_OFFS_L             0x7E

  #define AK8963_REG_WHOAMI                 0x00
  #define AK8963_REG_INFO                   0x01
  #define AK8963_REG_ST1                    0x02
  #define AK8963_REG_HXL                    0x03
  #define AK8963_REG_HXH                    0x04
  #define AK8963_REG_HYL                    0x05
  #define AK8963_REG_HYH                    0x06
  #define AK8963_REG_HZL                    0x07
  #define AK8963_REG_HZH                    0x08
  #define AK8963_REG_ST2                    0x09
  #define AK8963_REG_CNTL1                  0x0A
  #define AK8963_REG_CNTL2                  0x0B
  #define AK8963_REG_ASTC                   0x0C
  #define AK8963_REG_TS1                    0x0D
  #define AK8963_REG_TS2                    0x0E
  #define AK8963_REG_I2C_DIS                0x0F
  #define AK8963_REG_ASAX                   0x10
  #define AK8963_REG_ASAY                   0x11
  #define AK8963_REG_ASAZ                   0x12

  int otarover_sensors_init( void );
  int otarover_sensors_end( void );
  int otarover_sensors_get_data(sensor_data_t* data );
  int otarover_sensors_get_offset(sensor_offset_t* data );
  int otarover_sensors_set_offset(sensor_offset_t* data );
  int otarover_sensors_set_calibration(sensor_calibration_t* data);
  int otarover_sensors_get_calibration(sensor_calibration_t* data);
#endif //__OTAROVER_BLUE_SENSORS_H__
