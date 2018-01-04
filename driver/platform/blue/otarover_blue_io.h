/**
 * @file   otarover_blue_io.h
 * @author Otavio Ribeiro
 * @date   24 Dec 2017
 * @brief  A kernel module for controlling beaglebone blue board
 *
 * TODO: Add license text/version
 *
 */

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
