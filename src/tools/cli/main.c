#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <getopt.h>

#include "platform/linux/otarover_ioctl.h"

#define DEVICE_FILE_NAME "/dev/otarover"

//TODO: do not use ioct directly, instead, weshould use the otaroverlib to
//      control things, as soon as it is ready

static struct option long_options[] =
{
  {"dc-motor-enable",     optional_argument,  0, 'a'},
  {"m1-speed",            optional_argument,  0, 'b'},
  {"m1-direction",        optional_argument,  0, 'c'},
  {"m1-config",           optional_argument,  0, 'd'},
  {"m2-speed",            optional_argument,  0, 'e'},
  {"m2-direction",        optional_argument,  0, 'f'},
  {"m2-config",           optional_argument,  0, 'g'},
  {"help",                optional_argument,  0, 'h'},
  {0, 0, 0, 0}
};

int main(int argc, char** argv)
{
  long val;
  int dev_file, ret, opt, opt_index;
  char* pend;

  dev_file = open(DEVICE_FILE_NAME,O_RDWR);
  if(dev_file < 0){
    printf("Can't open device file: %s\n",DEVICE_FILE_NAME);
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
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = 0;
          if(strncmp(optarg,"true", (strlen(optarg) > 4 ? 4 : strlen(optarg))) != 0){
            val = strtol(optarg,&pend,10);
          } else {
            val = 1;
          }

          if(val > 0) {
            val = OTAROVER_IOCTL_DC_MOTOR_ENABLE;
          } else {
            val = OTAROVER_IOCTL_DC_MOTOR_DISABLE;
          }

          ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M_ENABLE,&val);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'b':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = strtol(optarg,&pend, 10);
          ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M1_SPEED,&val);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'c':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = -2;
          if(strncmp(optarg,"forward",(strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_FORWARD;
          } else if(strncmp(optarg,"backward", (strlen(optarg) > 8 ? 8 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_BACKWARD;
          } else if(strncmp(optarg,"stopped", (strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_STOPPED;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M1_DIRECTION,&val);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'd':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = -2;
          if(strncmp(optarg,"normal",(strlen(optarg) > 6 ? 6 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_CONFIG_NORMAL;
          } else if(strncmp(optarg,"reverse", (strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_CONFIG_REVERSE;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M1_CONFIG,&val);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'e':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = strtol(optarg,&pend,10);
          ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M2_SPEED,&val);
          if(ret != 0){
            printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
          }
        }
        break;

      case 'f':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = -2;
          if(strncmp(optarg,"forward",(strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_FORWARD;
          } else if(strncmp(optarg,"backward", (strlen(optarg) > 8 ? 8 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_BACKWARD;
          } else if(strncmp(optarg,"stopped", (strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_DIR_STOPPED;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }
          if(val != -2){
            ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M2_DIRECTION,&val);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
        }
        break;

      case 'g':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          val = -2;
          if(strncmp(optarg,"normal",(strlen(optarg) > 6 ? 6 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_CONFIG_NORMAL;
          } else if(strncmp(optarg,"reverse", (strlen(optarg) > 7 ? 7 : strlen(optarg))) == 0){
            val = OTAROVER_IOCTL_CONFIG_REVERSE;
          } else {
            printf ("invalid argument to option %s\n", long_options[opt_index].name);
          }

          if(val != -2){
            ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M2_CONFIG,&val);
            if(ret != 0){
              printf ("error setting option %s with value %s\n", long_options[opt_index].name, optarg);
            }
          }
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

  close(dev_file);

  return EXIT_SUCCESS;
}
