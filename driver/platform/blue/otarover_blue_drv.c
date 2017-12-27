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

/** GPIO2_2 - (2 * 32) + 2 **/
static unsigned int gpio_heartbeat_led = 66;
module_param(gpio_heartbeat_led, uint, S_IRUGO);
MODULE_PARM_DESC(gpio_heartbeat_led, " GPIO HEART BEAT LED number (default=66)");

static bool heartbeat_led_on = true;

static irq_handler_t  otarover_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static int heartbeat(void* arg);

static struct task_struct *task;

/** @brief The LKM initialization function
 *  This function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init otarover_init(void)
{
   int result = 0;
   void __iomem *io;

   printk(KERN_INFO "OTAROVER: Initializing the platform LKM\n");

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


   /* PINMUX settings */
   /* set PWM1A - GPIO1_18 as pwm mode */
   io = ioremap(0x44E10000, 128*1024);
   writel(0x16,io + 0x848);
   /* set PWM1B - GPIO1_19 as pwm mode */
   writel(0x16,io + 0x84c);
   /* enable PWMSS1 - pwmss_ctrl */
   writel(0x02, io + 0x664);

   /*PWM1 subsystem clock*/
   io = ioremap(0x48302000,4*1024);
   writel(0x111,io + 0x8);

   /* PWM1A/B config */
   io = ioremap(0x48302200,4*1024);
   /* TBCTL - 0b1100100000111100*/
   writew(0xC8C3,io + 0x00);
   /* TBPRD - 0x3FF - 1023 */
   writew(0x3FF, io + 0x0A);
   /* TBPHS - 0x00 */
   writew(0x00, io + 0x06);
   /* TBCTN - 0x00 */
   writew(0x00, io + 0x08);
   /* COMPA - 0x12c - 50% */
   writew(0x1FF, io + 0x12);
   /* COMPB - 0x12c - 50% */
   writew(0x1FF, io + 0x14);
   /* CMPCTL - 0b 11010 */
   writew(0x14, io + 0x0E);
   /* AQCTLA - 0b000010100001 */
   writew(0xA1, io + 0x16);
   /* AQCTLB - 0b101000000001 */
   writew(0xA01, io + 0x18);

   /* enable PWM1 - CM_PER */
   io = ioremap(0x44E00000,1024);
   writel(0x2,io + 0xCC);

   // Is the GPIO a valid GPIO number (e.g., the BBB has 4x32 but not all available)
   //if (!gpio_is_valid(gpioLED)){
   //   printk(KERN_INFO "GPIO_TEST: invalid LED GPIO\n");
   //   return -ENODEV;
   //}
   // Going to set up the LED. It is a GPIO in output mode and will be on by default
   //ledOn = true;
   //gpio_request(gpioLED, "sysfs");          // gpioLED is hardcoded to 49, request it
   //gpio_direction_output(gpioLED, ledOn);   // Set the gpio to be in output mode and on
   // gpio_set_value(gpioLED, ledOn);          // Not required as set by line above (here for reference)
   //gpio_export(gpioLED, false);             // Causes gpio49 to appear in /sys/class/gpio
                     // the bool argument prevents the direction from being changed
   //gpio_request(gpioButton, "sysfs");       // Set up the gpioButton
   //gpio_direction_input(gpioButton);        // Set the button GPIO to be an input
   //gpio_set_debounce(gpioButton, 200);      // Debounce the button with a delay of 200ms
   //gpio_export(gpioButton, false);          // Causes gpio115 to appear in /sys/class/gpio
                     // the bool argument prevents the direction from being changed
   // Perform a quick test to see that the button is working as expected on LKM load
   //printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));
 
   // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   //irqNumber = gpio_to_irq(gpioButton);
   //printk(KERN_INFO "GPIO_TEST: The button is mapped to IRQ: %d\n", irqNumber);
 
   // This next call requests an interrupt line
   //result = request_irq(irqNumber,             // The interrupt number requested
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
   gpio_set_value(gpio_heartbeat_led, 0);
   gpio_unexport(gpio_heartbeat_led);
   gpio_free(gpio_heartbeat_led);

   kthread_stop(task);

   //printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));
   //printk(KERN_INFO "GPIO_TEST: The button was pressed %d times\n", numberPresses);
   //gpio_set_value(gpioLED, 0);              // Turn the LED off, makes it clear the device was unloaded
   //gpio_unexport(gpioLED);                  // Unexport the LED GPIO
   //free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   //gpio_unexport(gpioButton);               // Unexport the Button GPIO
   //gpio_free(gpioLED);                      // Free the LED GPIO
   //gpio_free(gpioButton);                   // Free the Button GPIO
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
 
/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(otarover_init);
module_exit(otarover_exit);
