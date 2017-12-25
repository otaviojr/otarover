#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "otarover_platform.h"

/* platform user space driver */
extern otarover_platform_t platform;

int main(int argc, char**argv)
{
    printf("OTAROVER: Starting otarover 1.0\n");
    (*platform.init)();
    (*platform.exit)();
    return 0;
}
