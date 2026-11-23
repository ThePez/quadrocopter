/*
 *****************************************************************************
 * File: recorder.h
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Interface for session recording and loading.
 *****************************************************************************
 */

/*
 * Session recording (CSV out) and loading (CSV in).
 *
 * File format (one line per sample, plus event marks):
 *
 *   # started,2026-11-10T14:03:22
 *   time_s,pitch_angle,roll_angle,...
 *   0.0132,1.2,...
 *   # event,12.431,Sent Pitch / Roll Rate: Kp=0.2000 ...
 */
#ifndef RECORDER_H
#define RECORDER_H

#include <stdbool.h>
#include <stdio.h>

#include "telemetry.h"

typedef struct {
    FILE *file;
    double start; /* now_s() when recording began */
    int unflushed;
    int rows;
    char path[512];
} Recorder;

bool recorder_active(const Recorder *r);

/* Opens <dir>/session_YYYYMMDD_HHMMSS.csv. Returns 0 or -1 with `err` filled. */
int recorder_start(Recorder *r, const char *dir, char *err, size_t errlen);
void recorder_add(Recorder *r, const Sample *s);
void recorder_event(Recorder *r, const char *text);
/* Closes the file; returns true if one was open. */
bool recorder_stop(Recorder *r);

/* ---- Loading ---- */

typedef struct {
    double t;
    char text[128];
} SessionEvent;

typedef struct {
    char path[512];
    int count;
    double *t;           /* time_s column */
    float *col[F_COUNT]; /* NULL if the file has no such column */
    SessionEvent *events;
    int nevents;
    double duration;
} Session;

int session_load(Session *s, const char *path, char *err, size_t errlen);
void session_free(Session *s);

#endif
