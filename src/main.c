/**
 * @file   main.c
 * @author Otavio Ribeiro
 * @date   10 Jan 2018
 * @brief  Otarover network daemon
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
#include <signal.h>
#include <getopt.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <math.h>

#include "otaroverlib.h"
#include "net/otarover_protocol.h"
#include "strategy/otarover_strategy.h"

static int m1_reverse = 0;
static int m2_reverse = 0;
static int is_daemon = 0;

static struct option long_options[] =
{
  {"m1-reverse",          no_argument,        &m1_reverse,  1},
  {"m2-reverse",          no_argument,        &m2_reverse,  1},
  {"daemon",              no_argument,        &is_daemon,   1},
  {"port",                required_argument,  NULL,         'a'},
  {"strategy",            required_argument,  NULL,         'b'},
  {"help",                no_argument,        0,            'h'},
  {0, 0, 0, 0}
};

#define BUFFER_LEN    256
#define DEFAULT_PORT  7777

static otarover_context_t* context = NULL;
static int current_speed = 0;
static int current_direction  = 0;

otarover_strategy_t* strategy=NULL;

void daemonize()
{
    pid_t pid;
    pid = fork();

    if (pid < 0){
        printf("OTAROVER: Error creating daemon process\n");
        exit(EXIT_FAILURE);
    }
    if (pid > 0)
        exit(EXIT_SUCCESS);

    if (setsid() < 0){
        printf("OTAROVER: Error creating daemon process. SID error\n");
        exit(EXIT_FAILURE);
    }

    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);
}

int main(int argc, char**argv)
{
  struct sockaddr_in si_me, si_other;
  socklen_t slen = sizeof(si_other);
  int s , recv_len, opt, opt_index, ret, socket_port;
  char buf[BUFFER_LEN];
  char* pend;

  otarover_protocol_t message;

  printf("OTAROVER: Starting otarover network daemon v1.0\n");

  socket_port = DEFAULT_PORT;

  while(1){
    opt = getopt_long (argc, argv, "", long_options, &opt_index);

    if(opt == -1){
      break;
    }

    switch(opt){
      case 'a':
        socket_port = strtol(optarg,&pend, 10);
        break;

      case 'b':
        if(optarg == NULL){
          printf ("invalid argument to option %s\n", long_options[opt_index].name);
        }  else {
          if(strcmp(optarg,"tank") == 0){
            strategy = &otarover_tank_strategy;
          }
        }
        break;

      case '?':
        printf("Try 'otaroverctl --help' for more information\n");
        break;

      case 'h':
        break;
    }
  }

  if(strategy == NULL){
    printf("No strategy selected. Use otarover --strategy=<strategy> to select one\n");
    return(EXIT_FAILURE);
  }

  if((*strategy->init)() < 0){
    return(EXIT_FAILURE);
  }

  context = otarover_init();
  if(context == NULL){
    printf("Error starting otarover library. Is otarover kernel driver loaded?\n");
    return(EXIT_FAILURE);
  }

  if(is_daemon) daemonize();

  ret = otarover_dc_motor_set_enable(context, OTAROVER_DC_MOTOR_ENABLE);
  if(ret < 0){
    printf ("otaroverlib error: otarover_dc_motor_set_enable\n");
  }

  if(m1_reverse > 0){
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_REVERSE, OTAROVER_DC_MOTOR1);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }  else  {
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_NORMAL, OTAROVER_DC_MOTOR1);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }

  if(m2_reverse > 0){
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_REVERSE, OTAROVER_DC_MOTOR2);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }  else  {
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_NORMAL, OTAROVER_DC_MOTOR2);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }

  ret = otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD , OTAROVER_DC_MOTOR1);
  if(ret < 0){
    printf ("otaroverlib error: otarover_dc_motor_set_direction\n");
  }
  ret = otarover_dc_motor_set_direction(context, OTAROVER_DIR_FORWARD , OTAROVER_DC_MOTOR2);
  if(ret < 0){
    printf ("otaroverlib error: otarover_dc_motor_set_direction\n");
  }

  ret = otarover_dc_motor_set_speed(context, 0, OTAROVER_DC_MOTOR1);
  if(ret < 0){
    printf ("otaroverlib error: otarover_dc_motor_set_speed\n");
  }
  ret = otarover_dc_motor_set_speed(context, 0, OTAROVER_DC_MOTOR2);
  if(ret < 0){
    printf ("otaroverlib error: otarover_dc_motor_set_speed\n");
  }

  printf("starting socket\n");

  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf("Error creating socket\n");
    return(EXIT_FAILURE);
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(socket_port);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
  {
      printf("Error bindind socket\n");
      return(EXIT_FAILURE);
  }

  while(1){
    printf("Waiting for data...\n");

    if ((recv_len = recvfrom(s, buf, BUFFER_LEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
    {
        printf("Error receiving data from socket\n");
        break;
    }

    printf("Received packet from %s:%d with lent %d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port),  recv_len);

    if(!otarover_protocol_parse_message(buf, &message, recv_len)){
      printf("Ignoring invalid message\n");
      continue;
    }

    if(message.message_type == OTAROVER_PROTOCOL_MSG_TYPE_MOV){
      printf("Processing MOV message\n");
      if(message.cmd == OTAROVER_PROTOCOL_CMD_DIRECTION){
        printf("Changing Direction to %d\n",message.value.int32_val);
        current_direction = message.value.int32_val;
      } else if(message.cmd == OTAROVER_PROTOCOL_CMD_SPEED){
        printf("Changing Speed to %d\n", message.value.int32_val);
        current_speed = message.value.int32_val;
      }
    } else {
      printf("Ignoring non implemented message\n");
    }

    (*strategy->set_course)(context, current_direction, current_speed);

    otarover_protocol_destroy_message(&message);

    //if (sendto(s, buf, recv_len, 0, (struct sockaddr*) &si_other, slen) == -1)
    //{
      //printf("Error sending data from socket\n");
    //}
  }

  (*strategy->exit)();

  close(s);
  otarover_close(context);

  return EXIT_SUCCESS;
}
