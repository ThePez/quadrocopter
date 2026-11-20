/*
 * Application state shared by main.c and every tab, plus the connection,
 * logging, sending and defaults-saving operations they all use.
 */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "recorder.h"
#include "store.h"
#include "tabs.h"
#include "telemetry.h"
#include "uart.h"

/* Injected by the Makefile; the fallbacks assume you run from config-app-c/. */
#ifndef REPO_DIR
#define REPO_DIR ".."
#endif
#ifndef APP_DIR
#define APP_DIR "."
#endif

#define SESSIONS_DIR       APP_DIR "/sessions"
#define USER_DEFAULTS_PATH REPO_DIR "/config-app/user_defaults.json"

#define BAUD_LIST_TEXT "9600;115200;230400;460800"
extern const int BAUD_RATES[4];

typedef enum { TS_NONE, TS_RECEIVING, TS_TIMEOUT } TelemetryStatus;

/* TAB_SESSION is only offered while a session file is loaded. */
enum MainTab { TAB_TELEMETRY, TAB_PID, TAB_DRONE, TAB_REMOTE, TAB_CONSOLE, TAB_SESSION, TAB_COUNT };
#define TAB_COUNT_VISIBLE(app) ((app)->stab.open ? TAB_COUNT : TAB_SESSION)

typedef struct App {
    /* Connection */
    int fd;
    bool connected;
    Telemetry *tel;
    PortInfo ports[UART_MAX_PORTS];
    int nports;
    int port_active, baud_active;
    bool port_edit, baud_edit; /* dropdown lists currently open */

    /* Latest telemetry */
    TelemetryStatus status;
    double last_rx;
    Sample last;
    bool have_last;
    bool live_plots;

    Recorder rec;

    /* Firmware defaults overlaid with the GUI's saved overrides */
    KVSet pid_defaults, drone_defaults, remote_defaults;

    int main_tab;
    TelemetryTab ttab;
    PidTab ptab;
    DroneTab dtab;
    RemoteTab rtab;
    ConsoleTab ctab;
    SessionTab stab;
} App;

void app_init(App *app);
void app_shutdown(App *app);

/* Per-frame, before drawing: drains telemetry, watches for timeouts and
 * dropped files. */
void app_update(App *app);

/* ---- Connection ---- */
void app_refresh_ports(App *app);
void app_connect(App *app);
void app_disconnect(App *app);

/* ---- Messages ---- */
void app_log(App *app, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void app_error(const char *title, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
/* Shows "Not Connected" and returns false when there's no open port. */
bool app_require_connection(App *app);

/* ---- Sending ---- */

/* Frames `payload` as packet `id` and writes it to the bridge. Shows an error
 * dialog and returns false on failure. */
bool app_send(App *app, uint8_t id, const void *payload, size_t len);

/* Merges `updates` into the given section of user_defaults.json (re-reading
 * the file first so edits by the Python app aren't lost) and into `live`. */
bool app_save_defaults(App *app, const char *section, const KVSet *updates, KVSet *live);

/* ---- Session recording ---- */
void app_toggle_recording(App *app);
void app_stop_recording(App *app);
void app_record_event(App *app, const char *text);

#endif
