/**
 * @file   otarover_blue_io.h
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

#ifndef __OTAROVER_BLUE_IO_H__
#define __OTAROVER_BLUE_IO_H__

  /* 1024 bits - 0x3FF*/
  #define MAX_PWM_SPEED		0x3FF

  typedef struct _board_state {
    bool dc_motors_enable;
    int m1_config;
    int m2_config;
    int m1_direction;
    int m2_direction;
    unsigned long int m1_speed;
    unsigned long int m2_speed;
  } board_state_t;

  typedef struct _board_config {
    unsigned int gpio_heartbeat_led;
    unsigned int gpio_stby_pin;
    unsigned int gpio_m1_dir_pin1;
    unsigned int gpio_m1_dir_pin2;
    unsigned int gpio_m2_dir_pin1;
    unsigned int gpio_m2_dir_pin2;
    board_state_t state;
  } board_config_t;

  int otarover_configure_board(board_config_t*);
  int otarover_release_board(board_config_t* config);

  int otarover_set_dc_motors_enable(board_config_t* config);
  int otarover_set_dc_motors_dir(board_config_t* config, int motor);
  int otarover_set_dc_motors_speed(board_config_t* config, int motor);

#endif //__OTAROVER_BLUE_IO_H__
