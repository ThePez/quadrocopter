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

typedef enum {
    TS_NONE,
    TS_RECEIVING,
    TS_TIMEOUT
} TelemetryStatus;

/* TAB_SESSION is only offered while a session file is loaded. */
enum MainTab {
    TAB_TELEMETRY,
    TAB_PID,
    TAB_DRONE,
    TAB_REMOTE,
    TAB_CONSOLE,
    TAB_SESSION,
    TAB_COUNT
};
#define TAB_COUNT_VISIBLE(app) ((app)->stab.open ? TAB_COUNT : TAB_SESSION)

/* ---- Config acknowledgements ----
 * Each config packet gets an ack from the drone or remote (relayed by the bridge). Requests are
 * tracked per ack type, one in flight at a time, so an ack can only belong to that request.
 * Unanswered requests are re-sent; later ones for the same device wait their turn. */
struct App;

/* Called once when a tracked request finishes: ok = the device confirmed it applied it.
 * Not ok covers a rejection, no ack after every retry, a failed write, and a disconnect.
 * `user` is the copy of the caller's context given to app_send_then(). */
typedef void (*AckDone)(struct App *app, const void *user, bool ok);
#define ACK_USER_MAX 192

typedef struct {
    uint8_t frame[UART_CONFIG_FRAME_SIZE]; /* the encoded frame, re-sent on a retry */
    size_t len;
    AckDone done;
    uint8_t user[ACK_USER_MAX];
    size_t user_len;
    const char *what; /* for messages, e.g. "Drone config" */
    double timeout;   /* seconds to wait for each attempt's ack */
    int max_attempts, attempts;
} AckRequest;

#define ACK_CHANNELS  3 /* PID gains, drone config, remote config */
#define ACK_QUEUE_CAP 8

typedef struct {
    bool in_flight;
    AckRequest cur;
    double sent_at;
    double quiet_until; /* ignore acks until then: a late duplicate must not count for the next
                           request */
    AckRequest queue[ACK_QUEUE_CAP];
    int qhead, qcount;
} AckChannel;

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

    /* Config acks, and the latest result shown in the sidebar */
    AckChannel ack[ACK_CHANNELS];
    char cfg_what[48], cfg_result[80];
    bool cfg_ok;
    double cfg_t; /* when the result was set; 0 = none */

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
 * dialog and returns false on failure. Config packets are then tracked until the ack
 * arrives (or retried / reported as unconfirmed); one for a device that is still waiting
 * on an earlier request is queued behind it. */
bool app_send(App *app, uint8_t id, const void *payload, size_t len);

/* As app_send(), but `done` is called once with the outcome (from app_update() or
 * app_disconnect(), never from inside this call), and `user_len` bytes of `user` are copied
 * for it. Returns false, without calling `done`, if the request could not be started or queued.
 * `done` may itself call app_send_then(), e.g. to follow an update with a flash save. */
bool app_send_then(App *app, uint8_t id, const void *payload, size_t len, AckDone done,
                   const void *user, size_t user_len);

/* Merges `updates` into the given section of user_defaults.json (re-reading
 * the file first so edits by the Python app aren't lost) and into `live`. */
bool app_save_defaults(App *app, const char *section, const KVSet *updates, KVSet *live);

/* ---- Session recording ---- */
void app_toggle_recording(App *app);
void app_stop_recording(App *app);
void app_record_event(App *app, const char *text);

#endif
