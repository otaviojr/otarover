#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "otarover_platform.h"

/* GPIO */
#define GPIO1_START_ADDRESS     0x4804C000
#define GPIO1_END_ADDRESS       0x4804E000
#define GPIO1_SIZE              (GPIO1_END_ADDRESS - GPIO1_START_ADDRESS)

#define GPIO2_START_ADDRESS     0x481AC000
#define GPIO2_END_ADDRESS       0x481AD000
#define GPIO2_SIZE              (GPIO2_END_ADDRESS - GPIO2_START_ADDRESS)

#define GPIO_OUTPUT_ENABLE      0x134
#define GPIO_SETDATAOUT         0x194
#define GPIO_CLEARDATAOUT       0x190

#define USR_LED_OUT             (1<<3)


/* CONTROL MODULE */
#define CTRL_MOD_START_ADDRESS	0x44E10000
#define CTRL_MOD_END_ADDRESS	0x44E11FFF
#define CTRL_MOD_SIZE		(CTRL_MOD_END_ADDRESS - CTRL_MOD_START_ADDRESS)
#define CTRL_MOD_PWM0_REGISTER	0x800

void pla_init()
{
    volatile void* gpio_addr;
    volatile unsigned int* gpio_outenabled_addr;
    volatile unsigned int* gpio_setdataout_addr;
    volatile unsigned int* gpio_cleardataout_addr;

    volatile void* ctrl_mod_addr;
    volatile unsigned int* ctrl_mod_pwm0_addr;

    printf("OTAROVER: Starting Beagle Bone Blue Platform\n");
    
    int fd =  open("/dev/mem", O_RDWR | O_SYNC);
    gpio_addr = mmap(0, GPIO2_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO2_START_ADDRESS);
    if( gpio_addr == MAP_FAILED)
    {
        printf("Error accessing shared gpio register memory\n");
        return;
    }
    gpio_outenabled_addr = gpio_addr + GPIO_OUTPUT_ENABLE;
    gpio_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
    gpio_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;

    *gpio_outenabled_addr &= ~USR_LED_OUT;
    *gpio_setdataout_addr = USR_LED_OUT;

    ctrl_mod_addr = mmap(0,CTRL_MOD_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, CTRL_MOD_START_ADDRESS);
    if( ctrl_mod_addr == MAP_FAILED)
    {
        printf("Error accessing shared control module register memory\n");
        return;
    }
    ctrl_mod_pwm0_addr = ctrl_mod_addr + CTRL_MOD_PWM0_REGISTER;
    printf("reg before changing 0x%x\n", *ctrl_mod_pwm0_addr);
    *ctrl_mod_pwm0_addr = 0x6; 
    printf("reg after changing 0x%x\n", *ctrl_mod_pwm0_addr);
}

void pla_exit()
{
}

otarover_platform_t platform = {pla_init,pla_exit};
