#ifndef SLEEP_H
#define SLEEP_H

#define AHB_CLOCK_MHZ 168

/* Delay by (sleep) ticks processor ticks */
void sleep(int ticks);

/* Delay by (sleep) us microseconds. */
void usleep(int usec);

#endif  /* SLEEP_H */
