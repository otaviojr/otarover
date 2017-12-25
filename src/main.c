#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "otarover_platform.h"

/* platform user space driver */
extern otarover_platform_t platform;

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
    printf("OTAROVER: Starting otarover 1.0\n");

    //TODO: command line to start as daemon
    daemonize();

    (*platform.init)();

    while(1){
        sleep(20);
    }

    (*platform.exit)();

    return EXIT_SUCCESS;
}
