/**
 * @file   main.c
 * @author Otavio Ribeiro
 * @date   10 Jan 2018
 * @brief  Otarover control application to query the kernel module
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <math.h>

#include "otaroverlib.h"

#define PI 3.14159265

static struct option long_options[] =
{
  {"dc-motor-enable",     optional_argument,  0, 'a'},
  {"m1-speed",            optional_argument,  0, 'b'},
  {"m1-direction",        optional_argument,  0, 'c'},
  {"m1-config",           optional_argument,  0, 'd'},
  {"m2-speed",            optional_argument,  0, 'e'},
  {"m2-direction",        optional_argument,  0, 'f'},
  {"m2-config",           optional_argument,  0, 'g'},
  {"sensors",             optional_argument,  0, 'i'},
  {"help",                optional_argument,  0, 'h'},
  {0, 0, 0, 0}
};

int main(int argc, char** argv)
{
  int ret, opt, opt_index, val;
  float north;
  sensor_info_t info;
  char* pend;

  otarover_context_t* otarover_context = otarover_init();
  if(otarover_context == NULL){
    printf("Error starting otarover library. Is otarover kernel driver loaded?\n");
    return(EXIT_FAILURE);
  }

  while(1){
    opt = getopt_long (argc, argv, "", long_options, &opt_index);

    if(opt == -1){
      break;
    }

    switch(opt){
      case 'a':
        if(optarg == NULL){
          ret = otarover_dc_motor_is_enable(otarover_context, &val);
          if(ret != 0){
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %s\n", long_options[opt_index].name, (val ? "true" : "false"));
          }
        } else {
          val = 0;
          if(strcmp(optarg,"true") != 0){
            val = strtol(optarg,&pend,10);
          } else {
            val = 1;
          }

          if(val > 0) {
            val = OTAROVER_DC_MOTOR_ENABLE;
          } else {
            val = OTAROVER_DC_MOTOR_DISABLE;
          }

          ret = otarover_dc_motor_set_enable(otarover_context, val);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'b':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_speed(otarover_context, &val, OTAROVER_DC_MOTOR1);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %d\n", long_options[opt_index].name, val);
          }
        } else {
          val = strtol(optarg,&pend, 10);
          ret = otarover_dc_motor_set_speed(otarover_context, val, OTAROVER_DC_MOTOR1);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'c':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_direction(otarover_context, &val, OTAROVER_DC_MOTOR1);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %s\n", long_options[opt_index].name, (val == OTAROVER_DIR_FORWARD ? "forward" : (val == OTAROVER_DIR_BACKWARD ? "backward" : "stopped")));
          }
        } else {
          val = -2;
          if(strcmp(optarg,"forward") == 0){
            val = OTAROVER_DIR_FORWARD;
          } else if(strcmp(optarg,"backward") == 0){
            val = OTAROVER_DIR_BACKWARD;
          } else if(strcmp(optarg,"stopped") == 0){
            val = OTAROVER_DIR_STOPPED;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = otarover_dc_motor_set_direction(otarover_context, val, OTAROVER_DC_MOTOR1);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'd':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_config(otarover_context, &val, OTAROVER_DC_MOTOR1);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %s\n", long_options[opt_index].name, (val == OTAROVER_CONFIG_NORMAL ? "normal" : "reverse"));
          }
        } else {
          val = -2;
          if(strcmp(optarg,"normal") == 0){
            val = OTAROVER_CONFIG_NORMAL;
          } else if(strcmp(optarg,"reverse") == 0){
            val = OTAROVER_CONFIG_REVERSE;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = otarover_dc_motor_set_config(otarover_context, val, OTAROVER_DC_MOTOR1);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'e':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_speed(otarover_context, &val, OTAROVER_DC_MOTOR2);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %d\n", long_options[opt_index].name, val);
          }
        } else {
          val = strtol(optarg,&pend,10);
          ret = otarover_dc_motor_set_speed(otarover_context, val, OTAROVER_DC_MOTOR2);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'f':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_direction(otarover_context, &val, OTAROVER_DC_MOTOR2);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %s\n", long_options[opt_index].name, (val == OTAROVER_DIR_FORWARD ? "forward" : (val == OTAROVER_DIR_BACKWARD ? "backward" : "stopped")));
          }
        } else {
          val = -2;
          if(strcmp(optarg,"forward") == 0){
            val = OTAROVER_DIR_FORWARD;
          } else if(strcmp(optarg,"backward") == 0){
            val = OTAROVER_DIR_BACKWARD;
          } else if(strcmp(optarg,"stopped") == 0){
            val = OTAROVER_DIR_STOPPED;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = otarover_dc_motor_set_direction(otarover_context, val, OTAROVER_DC_MOTOR2);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'g':
        if(optarg == NULL){
          ret = otarover_dc_motor_get_config(otarover_context, &val, OTAROVER_DC_MOTOR2);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("%s: %s\n", long_options[opt_index].name, (val == OTAROVER_CONFIG_NORMAL ? "normal" : "reverse"));
          }
        } else {
          val = -2;
          if(strcmp(optarg,"normal") == 0){
            val = OTAROVER_CONFIG_NORMAL;
          } else if(strcmp(optarg,"reverse") == 0){
            val = OTAROVER_CONFIG_REVERSE;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }

          if(val != -2){
            ret = otarover_dc_motor_set_config(otarover_context, val, OTAROVER_DC_MOTOR2);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'i':
        if(optarg == NULL){
          ret = otarover_read_sensors(otarover_context, &info);
          if(ret != 0) {
            printf ("error getting option %s\n", long_options[opt_index].name);
          } else {
            printf ("temperature: %dC\n", info.temperature);
            printf ("gyro-x: %.1f\n", info.gyro_x);
            printf ("gyro-y: %.1f\n", info.gyro_y);
            printf ("gyro-z: %.1f\n", info.gyro_z);
            printf ("accel-x: %.1f\n", info.accel_x);
            printf ("accel-y: %.1f\n", info.accel_y);
            printf ("accel-z: %.1f\n", info.accel_z);
            printf ("mag-x: %.1f\n", info.mag_x);
            printf ("mag-y: %.1f\n", info.mag_y);
            printf ("mag-z: %.1f\n", info.mag_z);
            north = atan2(info.mag_y, info.mag_x) * (180/PI);
            if(north < 0) north += 360;
            if(north > 360) north -= 360;
            printf ("mag-north: %.1f\n", north);
          }
        } else {
            printf ("option %s is read-only\n", long_options[opt_index].name);
        }
        break;

      case  '?':
        printf("Try 'otaroverctl --help' for more information\n");
        break;

      case 'h':
        break;

      default:
        abort();
    }
  }

  otarover_close(otarover_context);

  return EXIT_SUCCESS;
}
