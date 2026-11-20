#include "app.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ui.h"
#include "util.h"

const int BAUD_RATES[4] = {9600, 115200, 230400, 460800};

/* ------------------------------------------------------------------------ */
/* Init / shutdown                                                           */
/* ------------------------------------------------------------------------ */

static void load_defaults(App *app)
{
    struct {
        const char *header;
        KVSet *set;
        const char *section;
    } sources[] = {
        {REPO_DIR "/libs/pid/pid_defaults.h", &app->pid_defaults, "pid"},
        {REPO_DIR "/drone/main/drone_defaults.h", &app->drone_defaults, "drone"},
        {REPO_DIR "/remote/main/remote_defaults.h", &app->remote_defaults, "remote"},
    };

    Overrides *ov = malloc(sizeof *ov);
    char err[256] = "";
    bool have_overrides = ov && overrides_load(ov, USER_DEFAULTS_PATH, err, sizeof err) == 0;
    if (!have_overrides)
        app_log(app, "Could not read saved defaults: %s", err[0] ? err : "out of memory");

    for (size_t i = 0; i < sizeof sources / sizeof sources[0]; i++) {
        if (defines_load(sources[i].header, sources[i].set) != 0)
            app_log(app, "Warning: could not read defaults from %s", sources[i].header);
        if (have_overrides) {
            const KVSet *saved = overrides_section(ov, sources[i].section, false);
            if (saved)
                kv_merge(sources[i].set, saved);
        }
    }
    free(ov);
}

void app_init(App *app)
{
    memset(app, 0, sizeof *app);
    app->fd = -1;
    app->baud_active = 1; /* 115200 */
    app->live_plots = true;

    console_tab_init(&app->ctab);
    load_defaults(app);

    telemetry_tab_init(&app->ttab);
    pid_tab_init(&app->ptab, app);
    drone_tab_init(&app->dtab, app);
    remote_tab_init(&app->rtab, app);
    session_tab_init(&app->stab);

    app_refresh_ports(app);
}

void app_shutdown(App *app)
{
    app_disconnect(app);
    session_tab_free(&app->stab);
    telemetry_tab_free(&app->ttab);
    console_tab_free(&app->ctab);
}

/* ------------------------------------------------------------------------ */
/* Messages                                                                  */
/* ------------------------------------------------------------------------ */

void app_log(App *app, const char *fmt, ...)
{
    char msg[LOG_LEN - 24]; /* room for the "[HH:MM:SS.mmm] " prefix */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);

    char clock[16], line[LOG_LEN];
    format_clock(clock, sizeof clock);
    snprintf(line, sizeof line, "[%s] %s", clock, msg);
    console_tab_add(&app->ctab, line);
}

void app_error(const char *title, const char *fmt, ...)
{
    char msg[768];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    ui_modal_open(title, msg, "OK", NULL, NULL);
}

bool app_require_connection(App *app)
{
    if (app->connected)
        return true;
    app_error("Not Connected", "Please connect to serial port first.");
    return false;
}

/* ------------------------------------------------------------------------ */
/* Connection                                                                */
/* ------------------------------------------------------------------------ */

void app_refresh_ports(App *app)
{
    app->nports = uart_list_ports(app->ports, UART_MAX_PORTS);
    if (app->port_active >= app->nports)
        app->port_active = 0;
    if (app->nports == 0)
        app_log(app, "No valid serial ports detected.");
}

void app_connect(App *app)
{
    if (app->nports == 0) {
        app_error("Connection Error", "No valid port selected.");
        return;
    }

    const char *device = app->ports[app->port_active].device;
    int baud = BAUD_RATES[app->baud_active];

    char err[256];
    int fd = uart_open(device, baud, err, sizeof err);
    if (fd < 0) {
        app_log(app, "Connection failed: %s", err);
        app_error("Connection Error", "Failed to connect: %s", err);
        return;
    }

    Telemetry *tel = telemetry_start(fd);
    if (!tel) {
        uart_close(fd);
        app_error("Connection Error", "Failed to start the telemetry thread.");
        return;
    }

    app->fd = fd;
    app->tel = tel;
    app->connected = true;
    app_log(app, "Connected to %s at %d baud", device, baud);
}

void app_disconnect(App *app)
{
    if (!app->connected && !app->tel)
        return;

    app_stop_recording(app);

    if (app->tel) {
        telemetry_stop(app->tel); /* joins the reader thread before the fd closes */
        app->tel = NULL;
    }
    if (app->fd >= 0) {
        uart_close(app->fd);
        app->fd = -1;
        app_log(app, "Disconnected from serial port");
    }

    app->connected = false;
    app->status = TS_NONE;
}

/* ------------------------------------------------------------------------ */
/* Per-frame update                                                          */
/* ------------------------------------------------------------------------ */

static void handle_sample(App *app, const Sample *s)
{
    /* Recording takes every packet, whatever the plots are doing. */
    recorder_add(&app->rec, s);

    if (!app->live_plots)
        return;

    app->last_rx = s->t;
    app->status = TS_RECEIVING;
    app->last = *s;
    app->have_last = true;
    telemetry_tab_push(&app->ttab, s, drone_tab_stick_scale(&app->dtab));
}

static void open_dropped_session(App *app)
{
    if (!IsFileDropped())
        return;
    FilePathList files = LoadDroppedFiles();
    if (files.count > 0 && IsFileExtension(files.paths[0], ".csv"))
        session_tab_open(app, files.paths[0]);
    UnloadDroppedFiles(files);
}

void app_update(App *app)
{
    if (app->tel) {
        Sample batch[128];
        int n;
        while ((n = telemetry_poll(app->tel, batch, 128)) > 0)
            for (int i = 0; i < n; i++)
                handle_sample(app, &batch[i]);

        char err[160];
        if (telemetry_lost(app->tel, err, sizeof err)) {
            app_log(app, "Serial connection lost: %s", err);
            app_disconnect(app);
            app_error("Connection Lost", "The serial connection was lost:\n%s", err);
        }
    }

    if (app->connected && app->status == TS_RECEIVING && now_s() - app->last_rx > 2.0)
        app->status = TS_TIMEOUT;

    open_dropped_session(app);
}

/* ------------------------------------------------------------------------ */
/* Sending / saving                                                          */
/* ------------------------------------------------------------------------ */

bool app_send(App *app, uint8_t id, const void *payload, size_t len)
{
    if (!app_require_connection(app))
        return false;

    uint8_t frame[UART_CONFIG_FRAME_SIZE];
    size_t n = uart_encode_config_packet(id, payload, len, frame, sizeof frame);
    if (n == 0) {
        app_error("Send Error", "Failed to send: payload does not fit in a packet.");
        return false;
    }

    if (uart_write_all(app->fd, frame, n) != 0) {
        const char *why = strerror(errno);
        app_log(app, "Send failed: %s", why);
        app_error("Send Error", "Failed to send: %s", why);
        return false;
    }
    return true;
}

bool app_save_defaults(App *app, const char *section, const KVSet *updates, KVSet *live)
{
    Overrides *ov = malloc(sizeof *ov);
    if (!ov)
        return false;

    char err[256] = "";
    bool ok = false;
    if (overrides_load(ov, USER_DEFAULTS_PATH, err, sizeof err) == 0) {
        KVSet *sec = overrides_section(ov, section, true);
        if (sec) {
            kv_merge(sec, updates);
            ok = overrides_save(ov, USER_DEFAULTS_PATH, err, sizeof err) == 0;
        } else {
            snprintf(err, sizeof err, "too many sections");
        }
    }
    free(ov);

    if (ok)
        kv_merge(live, updates);
    else
        app_log(app, "Could not save defaults: %s", err);
    return ok;
}

/* ------------------------------------------------------------------------ */
/* Session recording                                                         */
/* ------------------------------------------------------------------------ */

void app_toggle_recording(App *app)
{
    if (recorder_active(&app->rec)) {
        app_stop_recording(app);
        return;
    }

    char err[256];
    if (recorder_start(&app->rec, SESSIONS_DIR, err, sizeof err) != 0) {
        app_error("Recording Error", "Could not start: %s", err);
        return;
    }
    app_log(app, "Recording session to %s", app->rec.path);
}

void app_stop_recording(App *app)
{
    int rows = app->rec.rows;
    char path[sizeof app->rec.path];
    str_copy(path, sizeof path, app->rec.path);
    if (recorder_stop(&app->rec))
        app_log(app, "Session saved (%d samples): %s", rows, path);
}

void app_record_event(App *app, const char *text)
{
    recorder_event(&app->rec, text);
}
