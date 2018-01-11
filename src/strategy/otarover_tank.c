/**
 * @file   otarover_tank.c
 * @author Otavio Ribeiro
 * @date   11 Jan 2018
 * @brief  Otarover tank strategy
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
#include <getopt.h>
#include <math.h>

#include "otaroverlib.h"
#include "strategy/otarover_strategy.h"

#define PI 3.14159265

static int left_motor = OTAROVER_DC_MOTOR1;
static int right_motor = OTAROVER_DC_MOTOR2;

static int otarover_tank_init()
{
  return 0;
}

static int otarover_tank_set_course(otarover_context_t* context, int current_direction, int current_speed)
{
  int desl, left_speed, right_speed;

  if(context == NULL) return -1;

  printf("OTAROVER: current_direction = %d\n", current_direction);

  if(current_speed > 0){

    desl = current_speed - ceil(abs(cos(current_direction*(PI/180)) * current_speed));

    printf("OTAROVER: desl = %d\n", desl);

    left_speed = current_speed;
    right_speed = current_speed;

    printf("OTAROVER: left_speed before = %d\n", left_speed);
    printf("OTAROVER: right_speed before = %d\n", right_speed);

    if(current_direction < 90){
      right_speed -= desl;
    } else if(current_direction > 270){
      left_speed -= desl;
    } else if(current_direction > 90 && current_direction < 180){
      right_speed -= desl;
    } else {
      left_speed -= desl;
    }

    printf("OTAROVER: left_speed after = %d\n", left_speed);
    printf("OTAROVER: right_speed after = %d\n", right_speed);

    otarover_dc_motor_set_speed(context, left_speed, left_motor);
    otarover_dc_motor_set_speed(context, right_speed, right_motor);

    if(current_direction < 90 || current_direction > 270){
      printf("OTAROVER: direction forward\n");
      otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, left_motor);
      otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, right_motor);
    } else {
      printf("OTAROVER: direction backward\n");
      otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, left_motor);
      otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, right_motor);
    }
  } else {
    if(current_direction == 0){
      otarover_dc_motor_set_speed(context, 0, left_motor);
      otarover_dc_motor_set_speed(context, 0, right_motor);
    } else {
      desl = 100 - ceil(abs(cos(current_direction*(PI/180)) * 100));

      if(current_direction < 90){
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, left_motor);
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, right_motor);
      } else if(current_direction > 270){
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, left_motor);
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, right_motor);
      } else if(current_direction > 90 && current_direction < 180){
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, left_motor);
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, right_motor);
      } else {
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD, left_motor);
        otarover_dc_motor_set_direction(context, OTAROVER_DIR_BACKWARD, right_motor);
      }

      otarover_dc_motor_set_speed(context, desl, left_motor);
      otarover_dc_motor_set_speed(context, desl, right_motor);
    }
  }
  return 0;
}

static int otarover_tank_exit()
{
  return 0;
}

otarover_strategy_t otarover_tank_strategy = {
  .name = "tank",
  .init = otarover_tank_init,
  .set_course = otarover_tank_set_course,
  .exit = otarover_tank_exit
};
