#include <time.h>
#include <unistd.h>
#include "time_difference.h"

/* calculate the time difference in nanoseconds (overflow a significant possibility) */
long time_difference_ns( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*(1000*1000*1000) + (end_time->tv_nsec - start_time->tv_nsec);
}

/* calculate the time difference in microseconds (overflow possible) */
long time_difference_us( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*(1000*1000) + (end_time->tv_nsec - start_time->tv_nsec)/(1000);
}

/* calculate the time difference in milliseconds (overflow unlikely) */
long time_difference_ms( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*(1000) + (end_time->tv_nsec - start_time->tv_nsec)/(1000*1000);
}
