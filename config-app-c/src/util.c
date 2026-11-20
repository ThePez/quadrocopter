#include "util.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

double now_s(void)
{
    static double origin = -1.0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double t = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    if (origin < 0.0)
        origin = t;
    return t - origin;
}

void str_copy(char *dst, size_t cap, const char *src)
{
    if (cap == 0)
        return;
    strncpy(dst, src, cap - 1);
    dst[cap - 1] = '\0';
}

void format_clock(char *buf, size_t cap)
{
    struct timespec ts;
    struct tm tmv;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tmv);
    snprintf(buf, cap, "%02d:%02d:%02d.%03d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
             (int)(ts.tv_nsec / 1000000));
}
