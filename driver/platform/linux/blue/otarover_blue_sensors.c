/**
 * @file   otarover_blue_sensors.c
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

#include <linux/kernel.h>
#include <linux/kthread.h>              // Required for threads code
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/delay.h>		            // sleep functions
#include <linux/i2c.h>

#include "otarover_blue_sensors.h"

struct i2c_adapter* i2c_dev;
struct i2c_client* i2c_mpu9250_client;
struct i2c_client* i2c_ak8963_client;

/* Read sensor task */
static struct task_struct *task;
static int otarover_read_sensors(void* arg);

static volatile bool should_read = false;

static uint8_t mag_sens[3] = {0,0,0};
static int16_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};


static struct i2c_board_info board_info_mpu9250[] =  {
  {
    I2C_BOARD_INFO("MPU9250", MPU9250_ADDRESS),
  }
};

static struct i2c_board_info board_info_ak8963[] =  {
  {
    I2C_BOARD_INFO("AK8963", AK8963_ADDRESS),
  }
};

static irq_handler_t otarover_imu_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static short int otarover_sensors_update_temperature( void );
static short int otarover_sensors_update_gyro( void );
static short int otarover_sensors_update_accel( void );
static short int otarover_sensors_update_mag( void );
static short int otarover_sensor_read_mag(int16_t*, int16_t*, int16_t*);

static int otarover_sensors_calibrate_gyro( void );
static int otarover_sensors_calibrate_accel( void );
static int otarover_sensors_calibrate_mag( void );

static sensor_data_t sensor_data;

int otarover_sensors_init()
{
  s8 whoami, value;
  int irq_num, result;

  memset((char*)&sensor_data,0,sizeof(sensor_data_t));

  i2c_dev = i2c_get_adapter(2);
  if (!i2c_dev) {
    printk(KERN_ALERT "OTAROVER: Error openning i2c2 device. Sensors not available.");
    return -ENODEV;
  }

  i2c_mpu9250_client = i2c_new_device(i2c_dev, board_info_mpu9250);
  if (!i2c_mpu9250_client) {
    printk(KERN_ALERT "OTAROVER: Error registering mpu9250 i2c2 client.");
    return -ENODEV;
  }

  i2c_ak8963_client = i2c_new_device(i2c_dev, board_info_ak8963);
  if (!i2c_mpu9250_client) {
    printk(KERN_ALERT "OTAROVER: Error registering ak8963 i2c2 client.");
    return -ENODEV;
  }

  whoami = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_WHOAMI);
  if(whoami != MPU9250_WHOAMI_VALUE){
    printk(KERN_ALERT "OTAROVER: Invalid MPU9250 sensor signature");
    return -ENODEV;
  } else {
    printk(KERN_INFO "OTAROVER: Found a valid MPU9250 sensor on i2c2");
  }

  //exit sleep mode and enable all sensors
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_PWR_MGMT_1, 0x01);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_PWR_MGMT_2, 0x00);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_CONFIG, 0x03);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_SMPLRT_DIV, 0x04);

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_CONFIG);
  value = value & ~0x03;      // Clear Fchoice bits [1:0]
  value = value & ~0x18;      // full scale select 250dps
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_CONFIG, value);

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_CONFIG1);
  value = value | 0x18;      // 16G
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_CONFIG1, value);

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_CONFIG2);
  value = value & ~0x0F;      // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  value = value | 0x03;       // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_CONFIG2, value);

  //enable interrupt - raw data ready
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_INT_PIN, 0x12);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_INT_EN, 0x01);

  mdelay(10);

  otarover_sensors_calibrate_gyro();
  otarover_sensors_calibrate_accel();

  whoami = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_WHOAMI);
  if(whoami != AK8963_WHOAMI_VALUE){
    printk(KERN_ALERT "OTAROVER: Invalid AK8963 sensor signature (0x%x)", whoami);
    return -ENODEV;
  } else {
    printk(KERN_INFO "OTAROVER: Found a valid AK8963 sensor on i2c2");
  }

  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x00);
  mdelay(10);
  otarover_sensors_calibrate_mag();
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x00);
  mdelay(10);
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x01 << 4 | 0x06);
  mdelay(10);

  //IMU_INT - GPIO3_21 ((3*32)+21)
  irq_num = gpio_to_irq(117);
  result = request_irq(irq_num,
                        (irq_handler_t) otarover_imu_irq_handler,
                       IRQF_TRIGGER_RISING,
                       "otarover_imu",
                       NULL);

   /* heartbeat task */
   task = kthread_run(otarover_read_sensors, NULL, "otarover_sensors");
   if(IS_ERR(task))
   {
      printk(KERN_ALERT "OTAROVER: Failed to create heartbeat task");
      return PTR_ERR(task);
   }


  return 0;
}

static int otarover_read_sensors(void* args)
{
  printk(KERN_INFO "OTAROVER: Read Sensor thread has started running \n");
  while(!kthread_should_stop())
  {
    if(should_read == true) {
      set_current_state(TASK_RUNNING);
      otarover_sensors_update_temperature();
      otarover_sensors_update_gyro();
      otarover_sensors_update_accel();
      otarover_sensors_update_mag();
      set_current_state(TASK_INTERRUPTIBLE);
      should_read = false;
    }
    msleep(100);
  }
  printk(KERN_INFO "OTAROVER: Read  Sensor thread has finished \n");
  return 0;
}

static irq_handler_t otarover_imu_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
  should_read = true;
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_INT_STATUS, 0x00);
  return (irq_handler_t) IRQ_HANDLED;
}

int otarover_sensors_end()
{
  int irq_num;

  kthread_stop(task);

  irq_num = gpio_to_irq(117);
  free_irq(irq_num, NULL);

  i2c_unregister_device(i2c_mpu9250_client);
  i2c_unregister_device(i2c_ak8963_client);
  i2c_put_adapter(i2c_dev);
  return 0;
}

sensor_data_t* otarover_sensors_get_data( void )
{
  return &sensor_data;
}

static short int otarover_sensors_update_temperature()
{
  int16_t temperature;
  uint8_t value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_TEMP_OUT_H);
  temperature = value<<8;
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_TEMP_OUT_L);
  temperature |= value;

  sensor_data.temperature = (temperature/334) + 21;

  return 0;
}

static short int otarover_sensors_update_gyro()
{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  uint8_t value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_XOUT_H);
  gyro_x = value<<8;
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_XOUT_L);
  gyro_x |= value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_YOUT_H);
  gyro_y = value<<8;
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_YOUT_L);
  gyro_y |= value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_ZOUT_H);
  gyro_z = value<<8;
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_GYRO_ZOUT_L);
  gyro_z |= value;

  sensor_data.gyro_x = gyro_x;
  sensor_data.gyro_y = gyro_y;
  sensor_data.gyro_z = gyro_z;

  return 0;
}

static short int otarover_sensors_update_accel()
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  uint8_t value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_XOUT_H);
  accel_x = ((int16_t)value<<8);
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_XOUT_L);
  accel_x |= (int16_t)value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_YOUT_H);
  accel_y = ((int16_t)value<<8);
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_YOUT_L);
  accel_y |= (int16_t)value;

  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_ZOUT_H);
  accel_z = ((int16_t)value<<8);
  value = i2c_smbus_read_byte_data(i2c_mpu9250_client, MPU9250_REG_ACCEL_ZOUT_L);
  accel_z |= (int16_t)value;

  sensor_data.accel_x = accel_x;
  sensor_data.accel_y = accel_y;
  sensor_data.accel_z = accel_z;

  return 0;
}

static short int otarover_sensor_read_mag(int16_t* mag_x, int16_t* mag_y, int16_t* mag_z)
{
  uint8_t value[2];
  value[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_ST1) & 0x01;
  if(value[0]){
    value[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HXL);
    value[1] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HXH);
    *mag_x = ((int16_t)value[1] << 8) | value[0];

    value[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HYL);
    value[1] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HYH);
    *mag_y = ((int16_t)value[1] << 8) | value[0];

    value[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HZL);
    value[1] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_HZH);
    *mag_z = ((int16_t)value[1] << 8) | value[0];

    value[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_ST2);
    if(!(value[0] & 0x08)){
      return 0;
    }
  }
  return -1;
}

static short int otarover_sensors_update_mag()
{
  int16_t mag_x, mag_y, mag_z;
  int32_t correction;

  if(otarover_sensor_read_mag(&mag_x, &mag_y, &mag_z) == 0){
    correction = ((mag_sens[0]-128) << 8);
    correction += (1<<16);
    correction *= 4912;
    correction /= 32760;
    sensor_data.mag_x = (((mag_x-mag_bias[0]-mag_scale[0]) * correction) >> 16);
    correction = (mag_sens[1]-128) << 8;
    correction += (1<<16);
    correction *= 4912;
    correction /= 32760;
    sensor_data.mag_y = (((mag_y-mag_bias[1]-mag_scale[1]) * correction) >> 16);
    correction = (mag_sens[2]-128) << 8;
    correction += (1<<16);
    correction *= 4912;
    correction /= 32760;
    sensor_data.mag_z = (((mag_z-mag_bias[2]-mag_scale[2]) * correction) >> 16);;

    return 0;
  }
  return -1;
}

static int otarover_sensors_calibrate_gyro( void )
{
  int x;
  int16_t gyro_x[2] = {0, 0};
  int16_t gyro_y[2] = {0, 0};
  int16_t gyro_z[2] = {0, 0};

  int16_t gyro_x_bias, gyro_y_bias, gyro_z_bias;

  for(x = 0; x < 30; x++){
    otarover_sensors_update_gyro();

    if(sensor_data.gyro_x < gyro_x[0] || gyro_x[0] == 0) gyro_x[0] = sensor_data.gyro_x;
    if(sensor_data.gyro_x > gyro_x[1] || gyro_x[1] == 0) gyro_x[1] = sensor_data.gyro_x;
    if(sensor_data.gyro_y < gyro_y[0] || gyro_y[0] == 0) gyro_y[0] = sensor_data.gyro_y;
    if(sensor_data.gyro_y > gyro_y[1] || gyro_y[1] == 0) gyro_y[1] = sensor_data.gyro_y;
    if(sensor_data.gyro_z < gyro_z[0] || gyro_z[0] == 0) gyro_z[0] = sensor_data.gyro_z;
    if(sensor_data.gyro_z > gyro_z[1] || gyro_z[1] == 0) gyro_z[1] = sensor_data.gyro_z;
  }

  gyro_x_bias = (gyro_x[1] - gyro_x[0]);
  gyro_y_bias = (gyro_y[1] - gyro_y[0]);
  gyro_z_bias = (gyro_z[1] - gyro_z[0]);

  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_X_OFFS_USER_H, (gyro_x_bias >> 8)&0xFF);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_X_OFFS_USER_L, gyro_x_bias&0xFF);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_Y_OFFS_USER_H, (gyro_y_bias >> 8)&0xFF);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_Y_OFFS_USER_L, gyro_y_bias&0xFF);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_Z_OFFS_USER_H, (gyro_z_bias >> 8)&0xFF);
  i2c_smbus_write_byte_data(i2c_mpu9250_client, MPU9250_REG_Z_OFFS_USER_L, gyro_z_bias&0xFF);

  return 0;
}

static int otarover_sensors_calibrate_accel( void )
{
  //TODO
  return 0;
}

static int otarover_sensors_calibrate_mag( void )
{
  uint16_t i = 0, sample_count = 0;
  int16_t mag_x = 0, mag_y = 0, mag_z = 0;
  int16_t mag_max[3] = {0x8008, 0x8008, 0x8008}, mag_min[3] = {0x7FF8, 0x7FF8, 0x7FF8};

  //Fuse ROM access mode
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x00);
  mdelay(10);
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x0F);
  mdelay(10);
  mag_sens[0] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_ASAX);
  mag_sens[1] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_ASAY);
  mag_sens[2] = i2c_smbus_read_byte_data(i2c_ak8963_client, AK8963_REG_ASAZ);
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x00);
  mdelay(10);
  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x01 << 4 | 0x06);
  mdelay(10);

  sample_count = 1500;
  for(i = 0; i < sample_count; i++) {
    if(otarover_sensor_read_mag(&mag_x, &mag_y, &mag_z) == 0){
      if(mag_x > mag_max[0]) mag_max[0] = mag_x;
      if(mag_x < mag_min[0]) mag_min[0] = mag_x;
      if(mag_y > mag_max[1]) mag_max[1] = mag_y;
      if(mag_y < mag_min[1]) mag_min[1] = mag_y;
      if(mag_z > mag_max[2]) mag_max[2] = mag_z;
      if(mag_z < mag_min[2]) mag_min[2] = mag_z;
    }
    mdelay(12); // at 100 Hz ODR, new mag data is available every 12 ms
  }

  // Get hard iron correction
  mag_bias[0] = ((mag_max[0] + mag_min[0])/2); // get average x mag bias in counts
  mag_bias[1] = ((mag_max[1] + mag_min[1])/2); // get average y mag bias in counts
  mag_bias[2] = ((mag_max[2] + mag_min[2])/2); // get average z mag bias in counts

  // Get soft iron correction estimate
  mag_scale[0] = (mag_max[0] - mag_min[0]); // get average x axis max chord length in counts
  mag_scale[1] = (mag_max[1] - mag_min[1]); // get average y axis max chord length in counts
  mag_scale[2] = (mag_max[2] - mag_min[2]); // get average z axis max chord length in counts

  i2c_smbus_write_byte_data(i2c_ak8963_client, AK8963_REG_CNTL1, 0x00);
  mdelay(10);

  return 0;
}
