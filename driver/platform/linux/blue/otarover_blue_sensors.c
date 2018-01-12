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
#include <linux/gpio.h>             // Required for the GPIO functions
#include <linux/i2c.h>

#include "otarover_blue_sensors.h"

struct i2c_adapter* i2c_dev;
struct i2c_client* i2c_client;

static struct i2c_board_info __initdata board_info[] =  {
	{
		I2C_BOARD_INFO("MPU9250", 0x68),
	}
};

int otarover_init_sensors()
{
  s32 whoami;

  i2c_dev = i2c_get_adapter(2);
  if (!i2c_dev) {
    printk(KERN_ALERT "OTAROVER: Error openning i2c2 device. Sensors not available");
    return -ENODEV;
  }
  
  i2c_client = i2c_new_device(i2c_dev, board_info);
  if (!i2c_dev) {
    printk(KERN_ALERT "OTAROVER: Error registering i2c2 client");
    return -ENODEV;
  }

  whoami = i2c_smbus_read_byte_data(i2c_client, 0x75);
  if(whoami != 0x71){
    printk(KERN_ALERT "OTAROVER: Invalid sensor signature");
    return -ENODEV;
  } else {
    printk(KERN_INFO "OTAROVER: Found a valid MPU9250 on i2c2");
  }

  return 0;
}

int otarover_end_sensors()
{
  i2c_unregister_device(i2c_client);
  i2c_put_adapter(i2c_dev);
  return 0;
}
