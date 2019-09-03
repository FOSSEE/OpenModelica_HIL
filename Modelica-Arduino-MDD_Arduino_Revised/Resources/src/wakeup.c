#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "../Include/serial.h"



void wakeup() 
{
   /* Set signal to this function. */
   signal(SIGALRM, wakeup);
   /* Set alarm clock for 5 seconds. */
   alarm(2);
}

