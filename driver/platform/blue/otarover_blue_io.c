/**
 * @file   otarover_blue_io.c
 * @author Otavio Ribeiro
 * @date   24 Dec 2017
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>             // Required for the GPIO functions

#include "otarover_blue_io.h"

 /** @brief Board Setup
  *  This function sets up the GPIOs, PWM, IRQ, char device and sysfs interfaces
  *  @return returns 0 if successful
  */
int otarover_configure_board(board_config_t* config)
{
  void __iomem *io;

  /* motor control pins */
  /* stby motor controller ping*/
  gpio_request(config->gpio_stby_pin,"sysfs");
  gpio_direction_output(config->gpio_stby_pin,config->state.dc_motors_enable);
  gpio_export(config->gpio_stby_pin, false);

  /* m1 direction */
  gpio_request(config->gpio_m1_dir_pin1,"sysfs");
  gpio_direction_output(config->gpio_m1_dir_pin1,false);
  gpio_export(config->gpio_m1_dir_pin1, false);

  gpio_request(config->gpio_m1_dir_pin2,"sysfs");
  gpio_direction_output(config->gpio_m1_dir_pin2,false);
  gpio_export(config->gpio_m1_dir_pin2, false);

  /* m2 direction */
  gpio_request(config->gpio_m2_dir_pin1,"sysfs");
  gpio_direction_output(config->gpio_m2_dir_pin1,false);
  gpio_export(config->gpio_m2_dir_pin1, false);

  gpio_request(config->gpio_m2_dir_pin2,"sysfs");
  gpio_direction_output(config->gpio_m2_dir_pin2,false);
  gpio_export(config->gpio_m2_dir_pin2, false);

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

  /* TODO: allow external configuration for PWM pins */

  /* PINMUX settings */
  /* set PWM1A - GPMC_A2 as pwm mode */
  io = ioremap(0x44E10000, 128*1024);
  writel(0x16,io + 0x848);
  /* set PWM1B - GPMC_A3 as pwm mode */
  writel(0x16,io + 0x84c);
  /* enable PWMSS1 - pwmss_ctrl */
  writel(0x02, io + 0x664);
  iounmap(io);

  return 0;
}

int otarover_release_board(board_config_t* config)
{
  gpio_set_value(config->gpio_stby_pin,0);
  gpio_unexport(config->gpio_stby_pin);
  gpio_free(config->gpio_stby_pin);

  gpio_set_value(config->gpio_m1_dir_pin1,0);
  gpio_unexport(config->gpio_m1_dir_pin1);
  gpio_free(config->gpio_m1_dir_pin1);

  gpio_set_value(config->gpio_m1_dir_pin2,0);
  gpio_unexport(config->gpio_m1_dir_pin2);
  gpio_free(config->gpio_m1_dir_pin2);

  gpio_set_value(config->gpio_m2_dir_pin1,0);
  gpio_unexport(config->gpio_m2_dir_pin1);
  gpio_free(config->gpio_m2_dir_pin1);

  gpio_set_value(config->gpio_m2_dir_pin2,0);
  gpio_unexport(config->gpio_m2_dir_pin2);
  gpio_free(config->gpio_m2_dir_pin2);

  return 0;
}

int otarover_set_dc_motors_enable(board_config_t* config)
{
  gpio_set_value(config->gpio_stby_pin,config->state.dc_motors_enable);
  return 0;
}

int otarover_set_dc_motors_dir(board_config_t* config, int motor)
{
  if(motor == 1){
    if(config->state.m1_direction > 0){
      gpio_set_value(config->gpio_m1_dir_pin1, (config->state.m1_config > 0 ? true : false));
      gpio_set_value(config->gpio_m1_dir_pin2, (config->state.m1_config > 0 ? false : true));
    } else if(config->state.m1_direction < 0){
      gpio_set_value(config->gpio_m1_dir_pin1, (config->state.m1_config > 0 ? false : true));
      gpio_set_value(config->gpio_m1_dir_pin2, (config->state.m1_config > 0 ? true : false));
    } else {
      gpio_set_value(config->gpio_m1_dir_pin1, false);
      gpio_set_value(config->gpio_m1_dir_pin2, false);
    }
  } else {
    if(config->state.m2_direction > 0){
      gpio_set_value(config->gpio_m2_dir_pin1, (config->state.m2_config > 0 ? true : false));
      gpio_set_value(config->gpio_m2_dir_pin2, (config->state.m2_config > 0 ? false : true));
    } else if(config->state.m2_direction < 0){
      gpio_set_value(config->gpio_m2_dir_pin1, (config->state.m2_config > 0 ? false : true));
      gpio_set_value(config->gpio_m2_dir_pin2, (config->state.m2_config > 0 ? true : false));
    } else {
      gpio_set_value(config->gpio_m2_dir_pin1, false);
      gpio_set_value(config->gpio_m2_dir_pin2, false);
    }
  }
  return 0;
}

int otarover_set_dc_motors_speed(board_config_t* config, int motor)
{
  void __iomem* io;
  unsigned short int counter;

  if(motor == 1) {
    counter = MAX_PWM_SPEED*config->state.m1_speed/100;
  } else {
    counter = MAX_PWM_SPEED*config->state.m2_speed/100;
  }

  /* PWM1A/B config */
  io = ioremap(0x48302200,4*1024);

  if(motor == 1){
    /* COMPA */
    writew(counter, io + 0x12);
  } else {
    /* COMPB */
    writew(counter, io + 0x14);
  }

  iounmap(io);

  return 0;
}
