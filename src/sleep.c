#include "sleep.h"

/* Delay by (sleep) ticks processor ticks */
void sleep(int ticks) {
    for (int i = 0; i < ticks; i++)
        __asm__("nop");
}

/* Delay by (sleep) us microseconds. */
void usleep(int usec) {
    sleep(usec * AHB_CLOCK_MHZ);
}
