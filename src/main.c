#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "otaroverlib.h"
#include "net/otarover_protocol.h"

static int m1_reverse = 0;
static int m2_reverse = 0;
static int is_daemon = 0;

static otarover_context_t* context;

static struct option long_options[] =
{
  {"m1-config",           no_argument,        &m1_reverse,  1},
  {"m2-config",           no_argument,        &m2_reverse,  1},
  {"daemon",              no_argument,        &is_daemon,   1},
  {"port",                required_argument,  NULL,         'a'},
  {"help",                no_argument,        0,            'h'},
  {0, 0, 0, 0}
};

#define BUFFER_LEN    256
#define DEFAULT_PORT          7777

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

  printf("OTAROVER: Starting otarover 1.0\n");

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

      case '?':
        printf("Try 'otaroverctl --help' for more information\n");
        break;

      case 'h':
        break;

      default:
        abort();
    }
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
  if(!m1_reverse){
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_NORMAL, OTAROVER_DC_MOTOR1);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }  else  {
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_REVERSE, OTAROVER_DC_MOTOR1);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }

  if(!m2_reverse){
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_NORMAL, OTAROVER_DC_MOTOR2);
    if(ret < 0){
      printf ("otaroverlib error: otarover_dc_motor_set_config\n");
    }
  }  else  {
    ret = otarover_dc_motor_set_config(context, OTAROVER_CONFIG_REVERSE, OTAROVER_DC_MOTOR2);
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
    printf("Waiting for data...");

    //try to receive some data, this is a blocking call
    if ((recv_len = recvfrom(s, buf, BUFFER_LEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
    {
        printf("Error receiving data from socket\n");
    }

    //print details of the client/peer and the data received
    printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

    if(!otarover_protocol_parse_message(buf, &message, recv_len)){
      printf("Ignoring invalid message\n");
    }

    if(message.message_type == OTAROVER_PROTOCOL_MSG_TYPE_MOV){
      printf("Processing MOV message\n");
      if(message.cmd == OTAROVER_PROTOCOL_CMD_DIRECTION){
        printf("Changing Direction\n");
      } else if(message.cmd == OTAROVER_PROTOCOL_CMD_SPEED){
        printf("Changing Speed\n");        
      }
    } else {
      printf("Ignoring non implemented message\n");
    }

    //now reply the client with the same data
    //if (sendto(s, buf, recv_len, 0, (struct sockaddr*) &si_other, slen) == -1)
    //{
    //  printf("Error sending data from socket\n");
    //}
  }

  close(s);
  otarover_close(context);

  return EXIT_SUCCESS;
}
