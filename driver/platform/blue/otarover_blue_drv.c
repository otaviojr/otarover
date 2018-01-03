/**
 * @file   blue_drv.c
 * @author Otavio Ribeiro
 * @date   24 Dec 2017
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/kthread.h>		// Required for threads code

#include <linux/delay.h>		// sleep functions
#include <linux/pwm.h>			// Required for PWM code

#include "otarover_blue_drv.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Otavio Ribeiro");
MODULE_DESCRIPTION("OtaRover beagle bone blue platform driver");
MODULE_VERSION("0.1");

/* 1024 bits - 0x3FF*/
#define MAX_PWM_SPEED		0x3FF

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

/* LED status*/
static bool heartbeat_led_on = true;

/* stby enable motor controller */
static bool motors_enable = false;

/* m1/m2 direction */
static int m1_config = 1;
static int m2_config = 1;
static int m1_direction = 0;
static int m2_direction = 0;
static unsigned long int m1_speed = 0;
static unsigned long int m2_speed = 0;

/* Heartbeat task */
static struct task_struct *task;

/* functions declaration */
static irq_handler_t otarover_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static int heartbeat(void* arg);

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

/** @brief The LKM initialization function
 *  This function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init otarover_init(void)
{
   int result = 0;
   void __iomem *io;

   printk(KERN_INFO "OTAROVER: Initializing the platform LKM\n");

   if(!gpio_is_valid(gpio_heartbeat_led))
   {
      printk(KERN_ALERT "OTAROVER: Invalid heartbeat led pin");
      return -ENODEV;
   }

   /* TODO: validate all pins */

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

   /* motor control pins */

   /* stby motor controller ping*/
   gpio_request(gpio_stby_pin,"sysfs");
   gpio_direction_output(gpio_stby_pin,motors_enable);
   gpio_export(gpio_stby_pin, false);
   //gpio_set_value(20,false);

   /* m1 direction */
   gpio_request(gpio_m1_dir_pin1,"sysfs");
   gpio_direction_output(gpio_m1_dir_pin1,false);
   gpio_export(gpio_m1_dir_pin1, false);
   //gpio_set_value(64,false);

   gpio_request(gpio_m1_dir_pin2,"sysfs");
   gpio_direction_output(gpio_m1_dir_pin2,false);
   gpio_export(gpio_m1_dir_pin2, false);
   //gpio_set_value(31,false);

   /* m2 direction */
   gpio_request(gpio_m2_dir_pin1,"sysfs");
   gpio_direction_output(gpio_m2_dir_pin1,false);
   gpio_export(gpio_m2_dir_pin1, false);
   //gpio_set_value(48,false);

   gpio_request(gpio_m2_dir_pin2,"sysfs");
   gpio_direction_output(gpio_m2_dir_pin2,false);
   gpio_export(gpio_m2_dir_pin2, false);
   //gpio_set_value(10,false);

   /* enable PWM1 - CM_PER */
   io = ioremap(0x44E00000,1024);
   writel(0x2,io + 0xCC);
   iounmap(io);

   /*PWM1 subsystem clock - CLKCONFIG*/
   io = ioremap(0x48302000,4*1024);
   writel(0x111,io + 0x8);
   iounmap(io);

   /* PWM1A/B config */
   io = ioremap(0x48302200,4*1024);
   /* TBCTL - 0b1100100000110000*/
   writew(0xC830,io + 0x00);
   /* TBPRD */
   writew(MAX_PWM_SPEED, io + 0x0A);
   /* TBPHS - 0x00 */
   writew(0x00, io + 0x06);
   /* TBCTN - 0x00 */
   writew(0x00, io + 0x08);
   /* COMPA */
   writew(0x00, io + 0x12);
   /* COMPB */
   writew(0x00, io + 0x14);
   /* CMPCTL - 0b1011010 */
   writew(0x5A, io + 0x0E);
   /* AQCTLA - 0b000000010010 */
   writew(0x12, io + 0x16);
   /* AQCTLB - 0b000100000010 */
   writew(0x102, io + 0x18);
   iounmap(io);

   /* PINMUX settings */
   /* set PWM1A - GPMC_A2 as pwm mode */
   io = ioremap(0x44E10000, 128*1024);
   writel(0x16,io + 0x848);
   /* set PWM1B - GPMC_A3 as pwm mode */
   writel(0x16,io + 0x84c);
   /* enable PWMSS1 - pwmss_ctrl */
   writel(0x02, io + 0x664);
   iounmap(io);

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
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required. Used to release the
 *  GPIOs and display cleanup messages.
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
   class_destroy(device_class);

   gpio_set_value(gpio_stby_pin,0);
   gpio_unexport(gpio_stby_pin);
   gpio_free(gpio_stby_pin);

   gpio_set_value(gpio_m1_dir_pin1,0);
   gpio_unexport(gpio_m1_dir_pin1);
   gpio_free(gpio_m1_dir_pin1);

   gpio_set_value(gpio_m1_dir_pin2,0);
   gpio_unexport(gpio_m1_dir_pin2);
   gpio_free(gpio_m1_dir_pin2);

   gpio_set_value(gpio_m2_dir_pin1,0);
   gpio_unexport(gpio_m2_dir_pin1);
   gpio_free(gpio_m2_dir_pin1);

   gpio_set_value(gpio_m2_dir_pin2,0);
   gpio_unexport(gpio_m2_dir_pin2);
   gpio_free(gpio_m2_dir_pin2);

   /* stop heartbeat led */
   gpio_set_value(gpio_heartbeat_led, 0);
   gpio_unexport(gpio_heartbeat_led);
   gpio_free(gpio_heartbeat_led);
   kthread_stop(task);

   //free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   printk(KERN_INFO "OTAROVER: Goodbye from the LKM!\n");
}

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
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
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

/* Global enable function */
static ssize_t m_show_enable(struct device *dev, struct device_attribute *attr,
                      char* buf)
{
	if(dev->class == device_class){
  		return sprintf(buf,(motors_enable == true ? "true\n" : "false\n"));
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
       			motors_enable = true;
    		} else if(strncmp(buf,"false",(count > 5 ? 5 : count)) == 0){
       			motors_enable = false;
    		} else {
       			return -EINVAL;
    		}
    		gpio_set_value(gpio_stby_pin,motors_enable);
    		return count;
    	} else {
        	return -EPERM;
    	}
}

/* M1 attributes functions */
static ssize_t m1_show_speed(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%lu\n", m1_speed);
}

static ssize_t m1_store_speed(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
        void __iomem* io;
        unsigned short int counter;

	if(kstrtoul(buf,10,&m1_speed)){
           return -EINVAL;
	}

        if(m1_speed > 100) m1_speed = 100;

	counter = MAX_PWM_SPEED*m1_speed/100;

        /* PWM1A/B config */
        io = ioremap(0x48302200,4*1024);
   	/* COMPA */
   	writew(counter, io + 0x12);
	iounmap(io);
        return count;
}

static ssize_t m1_show_direction(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, (m1_direction > 0 ? "forward\n" : (m1_direction < 0 ? "backward\n" : "stopped\n")));
}

static ssize_t m1_store_direction(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
	if(strncmp(buf,"forward",(count > 7 ? 7 : count)) == 0){
        	m1_direction = 1;
        } else if(strncmp(buf,"backward", (count > 8 ? 8 : count)) == 0){
           	m1_direction = -1;
        } else if(strncmp(buf,"stopped", (count > 7 ? 7 : count)) == 0){
           	m1_direction = 0;
        } else {
           	return -EINVAL;
        }

        if(m1_direction > 0){
           	gpio_set_value(gpio_m1_dir_pin1, (m1_config > 0 ? true : false));
           	gpio_set_value(gpio_m1_dir_pin2, (m1_config > 0 ? false : true));
        } else if(m1_direction < 0){
           	gpio_set_value(gpio_m1_dir_pin1, (m1_config > 0 ? false : true));
           	gpio_set_value(gpio_m1_dir_pin2, (m1_config > 0 ? true : false));
        } else {
           	gpio_set_value(gpio_m1_dir_pin1, false);
           	gpio_set_value(gpio_m1_dir_pin2, false);
        }

        return count;
}

static ssize_t m1_show_config(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, (m1_config > 0 ? "normal\n" : "reverse\n"));
}

static ssize_t m1_store_config(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
        if(strncmp(buf,"normal",(count > 6 ? 6 : count)) == 0){
                m1_config = 1;
        } else if(strncmp(buf,"reverse", (count > 7 ? 7 : count)) == 0){
                m1_config = -1;
        }

        return count;
}

/* M2 attributes functions */
static ssize_t m2_show_speed(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%lu\n", m2_speed);
}

static ssize_t m2_store_speed(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
        void __iomem* io;
        unsigned short int counter;

        if(kstrtol(buf,10,&m2_speed)){
           return -EINVAL;
        }

        if(m2_speed > 100) m2_speed = 100;

        counter = MAX_PWM_SPEED*m2_speed/100;

        /* PWM1A/B config */
        io = ioremap(0x48302200,4*1024);
        /* COMPB */
        writew(counter, io + 0x14);
        iounmap(io);
        return count;
}

static ssize_t m2_show_direction(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, (m2_direction > 0 ? "forward\n" : (m2_direction < 0 ? "backward\n" : "stopped\n")));
}

static ssize_t m2_store_direction(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
        if(strncmp(buf,"forward",(count > 7 ? 7 : count)) == 0){
           m2_direction = 1;
        } else if(strncmp(buf,"backward", (count > 8 ? 8 : count)) == 0){
           m2_direction = -1;
        } else if(strncmp(buf,"stopped", (count > 7 ? 7 : count)) == 0){
           m2_direction = 0;
        } else {
           return -EINVAL;
        }

        if(m2_direction > 0){
           gpio_set_value(gpio_m2_dir_pin1, (m2_config > 0 ? true : false));
           gpio_set_value(gpio_m2_dir_pin2, (m2_config > 0 ? false : true));
        } else if(m2_direction < 0){
           gpio_set_value(gpio_m2_dir_pin1, (m2_config > 0 ? false : true));
           gpio_set_value(gpio_m2_dir_pin2, (m2_config > 0 ? true : false));
        } else {
           gpio_set_value(gpio_m2_dir_pin1, false);
           gpio_set_value(gpio_m2_dir_pin2, false);
        }

        return count;
}

static ssize_t m2_show_config(struct device *dev, struct device_attribute *attr,
                      char *buf)
{
        return sprintf(buf, (m2_config > 0 ? "normal\n" : "reverse\n"));
}

static ssize_t m2_store_config(struct device *dev, struct device_attribute *attr,
                      const char *buf, size_t count)
{
        if(strncmp(buf,"normal",(count > 6 ? 6 : count)) == 0){
                m2_config = 1;
        } else if(strncmp(buf,"reverse", (count > 7 ? 7 : count)) == 0){
                m2_config = -1;
        }    

        return count;
}

/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(otarover_init);
module_exit(otarover_exit);
