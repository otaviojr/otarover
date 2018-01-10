/**
 * @file   otarover_blue_drv.c
 * @author Otavio Ribeiro
 * @date   24 Dec 2017
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/kthread.h>              // Required for threads code

#include <linux/delay.h>		            // sleep functions

#include "otarover_blue_drv.h"
#include "otarover_blue_io.h"
#include "otarover_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Otavio Ribeiro");
MODULE_DESCRIPTION("OtaRover beagle bone blue platform driver");
MODULE_VERSION("0.1");

/** GPIO2_2 - (2 * 32) + 2 **/
static unsigned int gpio_heartbeat_led = 66;
module_param(gpio_heartbeat_led, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_heartbeat_led, " GPIO HEART BEAT LED PIN NUMBER (default=66)");

/** GPIO0_20 - (0*32) + 20 */
static unsigned int gpio_stby_pin = 20;
module_param(gpio_stby_pin, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_stby_pin, "STBY MOTOR CONTROLLER PIN (default=20)");

/** GPIO2_0 (2*32) + 0 */
static unsigned int gpio_m1_dir_pin1 = 64;
module_param(gpio_m1_dir_pin1, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_m1_dir_pin1, "M1 DIRECTION PIN1 (default=64)");

/** GPIO0_31 (0*32) + 31 */
static unsigned int gpio_m1_dir_pin2 = 31;
module_param(gpio_m1_dir_pin2, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_m1_dir_pin2, "M1 DIRECTION PIN2 (default=31)");

/** GPIO1_16 (1*32) + 16 */
static unsigned int gpio_m2_dir_pin1 = 48;
module_param(gpio_m2_dir_pin1, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_m2_dir_pin1, "M2 DIRECTION PIN1 (default=48)");

/** GPIO0_10 (0*32) + 10 */
static unsigned int gpio_m2_dir_pin2 = 10;
module_param(gpio_m2_dir_pin2, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_m2_dir_pin2, "M2 DIRECTION PIN2 (default=10)");

/* TODO: allow external configuration for PWM pins */

/* LED status*/
static bool heartbeat_led_on = true;

/* Heartbeat task */
static struct task_struct *task;

/* char device interface */
static int dev_open(struct inode* inodep, struct file* filep);
static int dev_release(struct inode* inodep, struct file* filep);
static long dev_ioctl(struct file* filep, unsigned int cmd, unsigned long arg);

static struct file_operations dev_file_operations = {
  .owner = THIS_MODULE,
  .open = dev_open,
  .release = dev_release,
  .unlocked_ioctl = dev_ioctl
};

/* sysfs interface */
static ssize_t m_show_enable(struct device *dev, struct device_attribute *attr, char* buf);
static ssize_t m_store_enable(struct device *dev, struct device_attribute *attr, const char* buf, size_t count);

static ssize_t m1_show_speed(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m1_store_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t m2_show_speed(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m2_store_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t m1_show_direction(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m1_store_direction(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t m2_show_direction(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m2_store_direction(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t m1_show_config(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m1_store_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t m2_show_config(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t m2_store_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/** SYSFS ATTRIBUTES */
static DEVICE_ATTR(enable,
                   0665,
                   m_show_enable,
                   m_store_enable);

static DEVICE_ATTR(m1_speed,
                   0665,
                   m1_show_speed,
                   m1_store_speed);

static DEVICE_ATTR(m2_speed,
                   0665,
                   m2_show_speed,
                   m2_store_speed);

static DEVICE_ATTR(m1_direction,
                   0665,
                   m1_show_direction,
                   m1_store_direction);

static DEVICE_ATTR(m2_direction,
                   0665,
                   m2_show_direction,
                   m2_store_direction);

static DEVICE_ATTR(m1_config,
                   0665,
                   m1_show_config,
                   m1_store_config);

static DEVICE_ATTR(m2_config,
                   0665,
                   m2_show_config,
                   m2_store_config);

static struct class *device_class;
static struct device *dc_motors_device_object;
static struct device *battery_device_object;

static dev_t dev;
static struct cdev c_dev;
static struct device *char_device_object;

static board_config_t board_config;

/* functions declaration */
static irq_handler_t otarover_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static int heartbeat(void* arg);

/** @brief The LKM initialization function
 *  This function sets up the GPIOs, PWM, IRQ, char device and sysfs interfaces
 *  @return returns 0 if successful
 */
static int __init otarover_init(void)
{
   int result = 0;

   printk(KERN_INFO "OTAROVER: Initializing the platform LKM\n");

   if(!gpio_is_valid(gpio_heartbeat_led))
   {
      printk(KERN_ALERT "OTAROVER: Invalid heartbeat led pin");
      return -ENODEV;
   }

   /* TODO: validate all pins */

   board_config.gpio_heartbeat_led = gpio_heartbeat_led;
   board_config.gpio_stby_pin = gpio_stby_pin;
   board_config.gpio_m1_dir_pin1 = gpio_m1_dir_pin1;
   board_config.gpio_m1_dir_pin2 = gpio_m1_dir_pin2;
   board_config.gpio_m2_dir_pin1 = gpio_m2_dir_pin1;
   board_config.gpio_m2_dir_pin2 = gpio_m2_dir_pin2;

   board_config.state.dc_motors_enable = false;
   board_config.state.m1_config = 1;
   board_config.state.m2_config = 1;
   board_config.state.m1_direction = 0;
   board_config.state.m2_direction = 0;
   board_config.state.m1_speed = 0;
   board_config.state.m2_speed = 0;

   heartbeat_led_on = true;
   gpio_request(gpio_heartbeat_led, "sysfs");
   /* set as output and turn it on */
   gpio_direction_output(gpio_heartbeat_led,heartbeat_led_on);
   /* export and do not allow direction change */
   gpio_export(gpio_heartbeat_led, false);

   /* heartbeat task */
   task = kthread_run(heartbeat, NULL, "otarover_heartbeat");
   if(IS_ERR(task))
   {
      printk(KERN_ALERT "OTAROVER: Failed to create heartbeat task");
      return PTR_ERR(task);
   }

   device_class = class_create(THIS_MODULE, "otarover");
   if(IS_ERR(device_class))
   {
      printk(KERN_ALERT "OTAROVER: Failed to create sysfs class");
      return PTR_ERR(device_class);
   }

   dc_motors_device_object = device_create(device_class, NULL, 0, NULL, "dc_motors");
   if(IS_ERR(dc_motors_device_object))
   {
      printk(KERN_ALERT "OTAROVER: Failed to create sysfs device");
      return PTR_ERR(dc_motors_device_object);
   }

   battery_device_object = device_create(device_class, NULL, 1, NULL, "battery");
   if(IS_ERR(battery_device_object))
   {
      printk(KERN_ALERT "OTAROVER: Failed to create sysfs device");
      return PTR_ERR(battery_device_object);
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_enable);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating enable sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m1_speed);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m1_speed sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m2_speed);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m2_speed sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m1_direction);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m1_direction sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m2_direction);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m2_direction sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m1_config);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m1_config sysfs endpoint");
      return result;
   }

   result = device_create_file(dc_motors_device_object, &dev_attr_m2_config);
   if(result < 0)
   {
      printk(KERN_ALERT "OTAROVER: Failed creating m2_config sysfs endpoint");
      return result;
   }

   /* character device interdace */
   result = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "otarover");
   if(result < 0)
   {
     printk(KERN_ALERT "OTAROVER: Failed registering region");
     return result;
   }
   cdev_init(&c_dev, &dev_file_operations);
   result = cdev_add(&c_dev, dev, MINOR_CNT);
   if(result < 0)
   {
     printk(KERN_ALERT "OTAROVER: Error adding char device to region");
     return result;
   }

   char_device_object = device_create(device_class, NULL, dev, NULL,  "otarover");
   if(IS_ERR(char_device_object))
   {
     printk(KERN_ALERT "OTAROVER: Failed to create char device");
     return PTR_ERR(char_device_object);
   }

   otarover_configure_board(&board_config);

   // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   // irqNumber = gpio_to_irq(gpioButton);
   // printk(KERN_INFO "GPIO_TEST: The button is mapped to IRQ: %d\n", irqNumber);

   // This next call requests an interrupt line
   // result = request_irq(irqNumber,             // The interrupt number requested
   //                      (irq_handler_t) ebbgpio_irq_handler, // The pointer to the handler function below
   //                     IRQF_TRIGGER_RISING,   // Interrupt on rising edge (button press, not release)
   //                     "ebb_gpio_handler",    // Used in /proc/interrupts to identify the owner
   //                     NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

   //printk(KERN_INFO "GPIO_TEST: The interrupt request result is: %d\n", result);

   return result;
}

/** @brief The LKM cleanup function
 *  Cleanup to exit
 */
static void __exit otarover_exit(void){

   /* remover sysfs interface */
   device_remove_file(dc_motors_device_object, &dev_attr_enable);
   device_remove_file(dc_motors_device_object, &dev_attr_m1_speed);
   device_remove_file(dc_motors_device_object, &dev_attr_m2_speed);
   device_remove_file(dc_motors_device_object, &dev_attr_m1_direction);
   device_remove_file(dc_motors_device_object, &dev_attr_m2_direction);
   device_remove_file(dc_motors_device_object, &dev_attr_m1_config);
   device_remove_file(dc_motors_device_object, &dev_attr_m2_config);

   device_destroy(device_class ,1);
   device_destroy(device_class, 0);
   device_destroy(device_class, dev);
   class_destroy(device_class);
   cdev_del(&c_dev);
   unregister_chrdev_region(dev,MINOR_CNT);

   otarover_release_board(&board_config);

   /* stop heartbeat led */
   gpio_set_value(gpio_heartbeat_led, 0);
   gpio_unexport(gpio_heartbeat_led);
   gpio_free(gpio_heartbeat_led);
   kthread_stop(task);

   //free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   printk(KERN_INFO "OTAROVER: Goodbye from the LKM!\n");
}

/** @brief Heartbeat function
 *  This function will blink the beagle bone blue red led
 *  @return returns 0 if successful
 */
static int heartbeat(void* args)
{
   printk(KERN_INFO "OTAROVER: Heartbeat thread has started running \n");
   while(!kthread_should_stop())
   {
      set_current_state(TASK_RUNNING);
      heartbeat_led_on = !heartbeat_led_on;
      gpio_set_value(gpio_heartbeat_led,heartbeat_led_on);
      set_current_state(TASK_INTERRUPTIBLE);
      msleep(1000);
   }
   printk(KERN_INFO "OTAROVER: Heartbeat thread has finished \n");
   return 0;
}

/** @brief The GPIO IRQ Handler function
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t otarover_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
   //ledOn = !ledOn;                          // Invert the LED state on each button press
   //gpio_set_value(gpioLED, ledOn);          // Set the physical LED accordingly
   //printk(KERN_INFO "GPIO_TEST: Interrupt! (button state is %d)\n", gpio_get_value(gpioButton));
   //numberPresses++;                         // Global counter, will be outputted when the module is unloaded
   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

/* sysfs interface show/store functions */
static ssize_t m_show_enable(struct device *dev, struct device_attribute *attr,
                      char* buf)
{
	if(dev->class == device_class){
  		return sprintf(buf,(board_config.state.dc_motors_enable == true ? "true\n" : "false\n"));
	} else {
		return -EPERM;
	}
}

static ssize_t m_store_enable(struct device * dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  //TODO: enable and disable PWM here. Now, just the stby pin is used
  if(dev->class == device_class){
    printk("m_store_enable with value %s of size %d\n",buf,count);
    if(strncmp(buf,"true",(count > 3 ? 3 : count)) == 0){
      board_config.state.dc_motors_enable = true;
    } else if(strncmp(buf,"false",(count > 5 ? 5 : count)) == 0){
      board_config.state.dc_motors_enable = false;
    } else {
      return -EINVAL;
    }
    otarover_set_dc_motors_enable(&board_config);
    return count;
  } else {
    return -EPERM;
  }
}

/* M1 sysfs attributes functions */
static ssize_t m1_show_speed(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
  return sprintf(buf, "%lu\n", board_config.state.m1_speed);
}

static ssize_t m1_store_speed(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
	if(kstrtoul(buf,10,&board_config.state.m1_speed)){
    return -EINVAL;
	}

  if(board_config.state.m1_speed > 100) board_config.state.m1_speed = 100;

  otarover_set_dc_motors_speed(&board_config,1);

  return count;
}

static ssize_t m1_show_direction(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, (board_config.state.m1_direction > 0 ? "forward\n" : (board_config.state.m1_direction < 0 ? "backward\n" : "stopped\n")));
}

static ssize_t m1_store_direction(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  if(strncmp(buf,"forward",(count > 7 ? 7 : count)) == 0){
    board_config.state.m1_direction = 1;
  } else if(strncmp(buf,"backward", (count > 8 ? 8 : count)) == 0){
    board_config.state.m1_direction = -1;
  } else if(strncmp(buf,"stopped", (count > 7 ? 7 : count)) == 0){
    board_config.state.m1_direction = 0;
  } else {
    return -EINVAL;
  }

  otarover_set_dc_motors_dir(&board_config, 1);

  return count;
}

static ssize_t m1_show_config(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
  return sprintf(buf, (board_config.state.m1_config > 0 ? "normal\n" : "reverse\n"));
}

static ssize_t m1_store_config(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  if(strncmp(buf,"normal",(count > 6 ? 6 : count)) == 0){
    board_config.state.m1_config = 1;
  } else if(strncmp(buf,"reverse", (count > 7 ? 7 : count)) == 0){
    board_config.state.m1_config = -1;
  } else {
    return -EINVAL;
  }

  return count;
}

/* M2 attributes functions */
static ssize_t m2_show_speed(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
  return sprintf(buf, "%lu\n", board_config.state.m2_speed);
}

static ssize_t m2_store_speed(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  if(kstrtol(buf,10,&board_config.state.m2_speed)){
    return -EINVAL;
  }

  if(board_config.state.m2_speed > 100) board_config.state.m2_speed = 100;

  otarover_set_dc_motors_speed(&board_config, 2);

  return count;
}

static ssize_t m2_show_direction(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
  return sprintf(buf, (board_config.state.m2_direction > 0 ? "forward\n" : (board_config.state.m2_direction < 0 ? "backward\n" : "stopped\n")));
}

static ssize_t m2_store_direction(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  if(strncmp(buf,"forward",(count > 7 ? 7 : count)) == 0){
    board_config.state.m2_direction = 1;
  } else if(strncmp(buf,"backward", (count > 8 ? 8 : count)) == 0){
    board_config.state.m2_direction = -1;
  } else if(strncmp(buf,"stopped", (count > 7 ? 7 : count)) == 0){
    board_config.state.m2_direction = 0;
  } else {
    return -EINVAL;
  }

  otarover_set_dc_motors_dir(&board_config, 2);

  return count;
}

static ssize_t m2_show_config(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
  return sprintf(buf, (board_config.state.m2_config > 0 ? "normal\n" : "reverse\n"));
}

static ssize_t m2_store_config(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
  if(strncmp(buf,"normal",(count > 6 ? 6 : count)) == 0){
    board_config.state.m2_config = 1;
  } else if(strncmp(buf,"reverse", (count > 7 ? 7 : count)) == 0){
    board_config.state.m2_config = -1;
  } else {
    return -EINVAL;
  }

  return count;
}

/* char device interface implementation */
static int dev_open(struct inode* inodep, struct file* filep)
{
  printk(KERN_INFO "OTAROVER: dev file opened \n");
  return 0;
}

static int dev_release(struct inode* inodep, struct file* filep)
{
  printk(KERN_INFO "OTAROVER: dev file closed \n");
  return 0;
}

static long dev_ioctl(struct file* filep, unsigned int cmd, unsigned long arg)
{
  long ret = 0;

  if (_IOC_TYPE(cmd) != OTAROVER_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > OTAROVER_IOCTL_MAX_CMD) return -ENOTTY;

  if (_IOC_DIR(cmd) & _IOC_READ) {
    ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
  } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
  }

  if (ret) return -EFAULT;

  switch(cmd){
    case OTAROVER_IOCTL_SET_M_ENABLE:
      ret =  get_user(board_config.state.dc_motors_enable, (long*)arg);
      if(ret == 0){
        otarover_set_dc_motors_enable(&board_config);
      }
      break;
    case OTAROVER_IOCTL_GET_M_ENABLE:
      ret = put_user(board_config.state.dc_motors_enable,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M1_SPEED:
      ret =  get_user(board_config.state.m1_speed, (long*)arg);
      if(ret == 0){
        if(board_config.state.m1_speed > 100) board_config.state.m1_speed = 100;
        otarover_set_dc_motors_speed(&board_config,1);
      }
      break;
    case OTAROVER_IOCTL_GET_M1_SPEED:
      ret = put_user(board_config.state.m1_speed,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M2_SPEED:
      ret =  get_user(board_config.state.m2_speed, (long*)arg);
      if(ret == 0){
        if(board_config.state.m2_speed > 100) board_config.state.m2_speed = 100;
        otarover_set_dc_motors_speed(&board_config,2);
      }
      break;
    case OTAROVER_IOCTL_GET_M2_SPEED:
      ret = put_user(board_config.state.m2_speed,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M1_DIRECTION:
      ret =  get_user(board_config.state.m1_direction, (long*)arg);
      if(ret == 0){
        otarover_set_dc_motors_dir(&board_config, 1);
      }
      break;
    case OTAROVER_IOCTL_GET_M1_DIRECTION:
      ret = put_user(board_config.state.m1_direction,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M2_DIRECTION:
      ret =  get_user(board_config.state.m2_direction, (long*)arg);
      if(ret == 0){
        otarover_set_dc_motors_dir(&board_config, 2);
      }
      break;
    case OTAROVER_IOCTL_GET_M2_DIRECTION:
      ret = put_user(board_config.state.m2_direction,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M1_CONFIG:
      ret =  get_user(board_config.state.m1_config, (long*)arg);
      if(ret == 0){
        otarover_set_dc_motors_dir(&board_config, 1);
      }
      break;
    case OTAROVER_IOCTL_GET_M1_CONFIG:
      ret = put_user(board_config.state.m1_config,(long*)arg);
      break;
    case OTAROVER_IOCTL_SET_M2_CONFIG:
      ret =  get_user(board_config.state.m2_config, (long*)arg);
      if(ret == 0){
        otarover_set_dc_motors_dir(&board_config, 2);
      }
      break;
    case OTAROVER_IOCTL_GET_M2_CONFIG:
      ret = put_user(board_config.state.m2_config,(long*)arg);
      break;
    default:
      return -ENOTTY;
  }
  return ret;
}

/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(otarover_init);
module_exit(otarover_exit);
