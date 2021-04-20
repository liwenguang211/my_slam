#include "time.h"

#include <sys/time.h>
#include <stdlib.h>

long get_current_time_us()
{
    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    return (stamp.tv_sec * 1000000 + stamp.tv_usec);
}
