#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>

/* Seconds on a monotonic clock, measured from the first call. Safe to call
 * from any thread (used to timestamp telemetry in the reader thread). */
double now_s(void);

/* strncpy that always NUL-terminates. */
void str_copy(char *dst, size_t cap, const char *src);

/* Wall-clock "HH:MM:SS.mmm" into buf (cap >= 16). */
void format_clock(char *buf, size_t cap);

#endif
