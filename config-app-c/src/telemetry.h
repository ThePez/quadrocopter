/*
 * Decoded telemetry samples and the background thread that produces them.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>

#include "protocol.h"

/* Column order matches the session CSV files (and the Python recorder). */
enum Field {
    F_PITCH_ANGLE,
    F_ROLL_ANGLE,
    F_YAW_ANGLE,
    F_PITCH_RATE,
    F_ROLL_RATE,
    F_YAW_RATE,
    F_MODE,
    F_PITCH_PID,
    F_ROLL_PID,
    F_YAW_PID,
    F_MOTOR_FL,
    F_MOTOR_BL,
    F_MOTOR_BR,
    F_MOTOR_FR,
    F_BATTERY,
    F_LOOP_DT_MAX_US,
    F_CMD_PITCH,
    F_CMD_ROLL,
    F_CMD_YAW,
    F_CMD_THROTTLE,
    F_COUNT
};

extern const char *const FIELD_NAMES[F_COUNT];

typedef struct {
    double t;         /* seconds, now_s() clock */
    float v[F_COUNT]; /* indexed by enum Field */
} Sample;

/* motor_a..d on the wire are FL, BL, BR, FR. */
void sample_from_wire(Sample *s, const struct sensor_telemetry_t *w, double t);

typedef struct Telemetry Telemetry;

/* Starts a reader thread on `fd`. The caller keeps ownership of the fd and must
 * call telemetry_stop() before closing it. Returns NULL on failure. */
Telemetry *telemetry_start(int fd);
void telemetry_stop(Telemetry *t);

/* Moves up to `max` queued samples into `out` (oldest first). Returns the count. */
int telemetry_poll(Telemetry *t, Sample *out, int max);

/* True exactly once if the serial read failed (e.g. USB unplugged); the reason
 * is copied into `err`. */
bool telemetry_lost(Telemetry *t, char *err, size_t errlen);

#endif
