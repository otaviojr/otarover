/**
 * @file   otarover_blue_drv.h
 * @author Otavio Ribeiro
 * @date   5 Jan 2018
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
 *
 */
#ifndef __OTAROVER_LIB_H__
#define __OTAROVER_LIB_H__

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

#endif //__OTAROVER_LIB_H__
