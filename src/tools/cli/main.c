#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
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
  {"help",                optional_argument,  0, 'h'},
  {0, 0, 0, 0}
};

int main(int argc, char** argv)
{
  long val;
  int dev_file, ret, opt, opt_index;

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
          printf ("set option %s with value %s\n", long_options[opt_index].name, optarg);
        }
        break;

      case 'b':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          printf ("set option %s with value %s\n", long_options[opt_index].name, optarg);
        }
        break;

      case 'c':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          printf ("set option %s with value %s\n", long_options[opt_index].name, optarg);
        }
        break;

      case 'd':
        if(optarg == NULL){
          printf ("get option %s\n", long_options[opt_index].name);
        } else {
          printf ("set option %s with value %s\n", long_options[opt_index].name, optarg);
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
  //dev_file = open(DEVICE_FILE_NAME,O_RDWR);
  //if(dev_file < 0){
  //  printf("Can't open device file: %s\n",DEVICE_FILE_NAME);
  //  return(EXIT_FAILURE);
  //}

  //val = OTAROVER_IOCTL_DC_MOTOR_ENABLE;
  //ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M_ENABLE,&val);
  //val = 25;
  //ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M1_SPEED,&val);
  //val = OTAROVER_IOCTL_DIR_FORWARD;
  //ret = ioctl(dev_file, OTAROVER_IOCTL_SET_M1_DIRECTION,&val);

  //close(dev_file);

  return EXIT_SUCCESS;
}
