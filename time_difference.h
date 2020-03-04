#ifndef TIME_DIFFERENCE
#define TIME_DIFFERENCE

long time_difference_ns( struct timespec *start_time, struct timespec *end_time );
long time_difference_us( struct timespec *start_time, struct timespec *end_time );
long time_difference_ms( struct timespec *start_time, struct timespec *end_time );

#endif  /* TIME_DIFFERENCE */
