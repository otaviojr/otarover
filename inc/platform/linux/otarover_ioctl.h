/**
 * @file   otarover_blue_drv.h
 * @author Otavio Ribeiro
 * @date   24 Dec 2017
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
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

  #define OTAROVER_IOCTL_MAX_CMD            14

  #define OTAROVER_IOCTL_DC_MOTOR_ENABLE    1
  #define OTAROVER_IOCTL_DC_MOTOR_DISABLE   0

  #define OTAROVER_IOCTL_DIR_FORWARD        1
  #define OTAROVER_IOCTL_DIR_STOPPED        0
  #define OTAROVER_IOCTL_DIR_BACKWARD       -1

  #define OTAROVER_IOCTL_CONFIG_NORMAL      1
  #define OTAROVER_IOCTL_CONFIG_REVERSE     -1

#endif //__OTAROVER_IOCTL_H__
