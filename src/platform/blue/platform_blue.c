#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "otarover_platform.h"

#define GPIO1_START_ADDRESS     0x4804C000
#define GPIO1_END_ADDRESS       0x4804E000
#define GPIO1_SIZE              (GPIO1_END_ADDRESS - GPIO1_START_ADDRESS)

#define GPIO2_START_ADDRESS     0x481AC000
#define GPIO2_END_ADDRESS       0x481AD000
#define GPIO2_SIZE              (GPIO2_END_ADDRESS - GPIO2_START_ADDRESS)

#define GPIO_OUTPUT_ENABLE      0x134
#define GPIO_SETDATAOUT         0x194
#define GPIO_CLEARDATAOUT       0x190

#define USR_LED_OUT             (1<<2)

void init()
{
    volatile void* gpio_addr;
    volatile unsigned int* gpio_outenabled_addr;
    volatile unsigned int* gpio_setdataout_addr;
    volatile unsigned int* gpio_cleardataout_addr;

    printf("OTAROVER: Starting Beagle Bone Blue Platform\n");

    int fd =  open("/dev/mem", O_RDWR | O_SYNC);
    gpio_addr = mmap(0, GPIO2_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO2_START_ADDRESS);
    if( gpio_addr == MAP_FAILED)
    {
        printf("Error accessing shared gpio register memory\n");
        return 1;
    }
    gpio_outenabled_addr = gpio_addr + GPIO_OUTPUT_ENABLE;
    gpio_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
    gpio_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;

    *gpio_outenabled_addr &= ~USR_LED_OUT;
    *gpio_setdataout_addr = USR_LED_OUT;
}

void exit()
{
}

otarover_platform_t platform = {init,exit};
