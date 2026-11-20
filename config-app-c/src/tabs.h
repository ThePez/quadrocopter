/*
 * Per-tab state and entry points. Each tab lives in its own tab_*.c file.
 */
#ifndef TABS_H
#define TABS_H

#include <stdbool.h>

#include "attitude3d.h"
#include "plot.h"
#include "recorder.h"
#include "telemetry.h"
#include "ui.h"

struct App;

/* ---- Telemetry ---- */

typedef struct {
    int sub; /* 0 = overview (angles, rates, PID, motors), 1..5 = one plot (those four, then controls) */
    Plot angles, rates, pid, motors, controls;
    Attitude3D att;
} TelemetryTab;

void telemetry_tab_init(TelemetryTab *t);
void telemetry_tab_free(TelemetryTab *t);
/* Limits needed to show the pilot's commands as a percentage of stick travel. */
typedef struct {
    float max_rate;                  /* deg/s at full stick */
    float min_throttle, max_throttle; /* us at the bottom / top of the throttle stick */
} StickScale;

void telemetry_tab_push(TelemetryTab *t, const Sample *s, StickScale scale);
void telemetry_tab_draw(struct App *app, Rectangle area);

/* ---- PID tuning ---- */

typedef struct {
    struct App *app;
    int axis, mode; /* axis: 0 = pitch/roll, 1 = yaw; mode: 0 = rate, 1 = angle */
    const char *title;
    const char *defaults_prefix; /* e.g. "RATE_PITCH_ROLL" */
    NumField kp, ki, kd, alpha;
    bool sent_valid; /* have the values below been sent live? */
    double sent_kp, sent_ki, sent_kd, sent_alpha;
    char current[128];
    bool current_set;
} PidGroup;

typedef struct {
    PidGroup group[3]; /* rate P/R, rate yaw, angle P/R */
} PidTab;

void pid_tab_init(PidTab *t, struct App *app);
void pid_tab_draw(struct App *app, Rectangle area);

/* ---- Drone config ---- */

#define DRONE_FIELDS 8

typedef struct {
    struct App *app;
    NumField f[DRONE_FIELDS];
    bool sent_valid;
    double sent[DRONE_FIELDS];
} DroneTab;

void drone_tab_init(DroneTab *t, struct App *app);
/* Max rate and throttle range as set on the Drone Config tab. */
StickScale drone_tab_stick_scale(const DroneTab *t);
void drone_tab_draw(struct App *app, Rectangle area);

/* ---- Remote config ---- */

typedef struct {
    struct App *app;
    NumField voltage[3];     /* cal multiplier, low, critical */
    NumField joystick[4][3]; /* channel (throttle, pitch, roll, yaw) x (min, centre, max) */
    bool sent_valid;
    double sent_voltage[3];
    double sent_joystick[4][3];
} RemoteTab;

void remote_tab_init(RemoteTab *t, struct App *app);
void remote_tab_draw(struct App *app, Rectangle area);

/* ---- Console ---- */

#define LOG_LINES 2000
#define LOG_LEN   200

typedef struct {
    char (*lines)[LOG_LEN]; /* ring buffer, LOG_LINES entries */
    int head, count;
    int scroll_up; /* lines scrolled away from the bottom (0 = following) */
} ConsoleTab;

void console_tab_init(ConsoleTab *t);
void console_tab_free(ConsoleTab *t);
void console_tab_add(ConsoleTab *t, const char *line);
void console_tab_draw(struct App *app, Rectangle area);

/* ---- Session viewer ---- */

#define SESSION_PLOTS 8

typedef struct {
    bool open;
    Session session;
    PlotView view;
    Plot plot[SESSION_PLOTS];
    int sub;
} SessionTab;

void session_tab_init(SessionTab *t);
void session_tab_free(SessionTab *t);
/* Loads and shows `path`; on failure shows an error dialog and leaves the
 * previous session (if any) alone. */
void session_tab_open(struct App *app, const char *path);
void session_tab_close(SessionTab *t);
void session_tab_draw(struct App *app, Rectangle area);

#endif
