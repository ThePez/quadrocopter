/*
 *****************************************************************************
 * File: app.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Application core: connection, config acks and retries, sending, saving defaults.
 *****************************************************************************
 */

/*
 * Application core: everything that is not drawing.
 *
 *  - init / shutdown and loading the default values (firmware headers + saved overrides)
 *  - the serial connection (connect, disconnect, port list)
 *  - app_update(): called once per frame BEFORE drawing. It drains the telemetry thread's
 *    queues, advances the config-ack state machine and notices a lost connection.
 *  - the ack tracker: every config packet is expected to be answered by the device; requests
 *    are queued per device, re-sent if unanswered, and reported through a callback.
 *  - sending, saving defaults to user_defaults.json, and session recording control.
 *
 * Everything here runs on the main (UI) thread. The only other thread is the serial reader in
 * telemetry.c; the two communicate through its mutex-protected queues.
 */

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

/* Fills the three default tables the tabs read their starting values from. For each of the
 * pid / drone / remote groups: scrape the firmware's *_defaults.h (#define NAME number), then
 * overlay whatever the user saved earlier in user_defaults.json. */
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

    Overrides *ov = malloc(sizeof(*ov));
    char err[256] = "";
    bool have_overrides = ov && overrides_load(ov, USER_DEFAULTS_PATH, err, sizeof(err)) == 0;
    if (!have_overrides) {
        app_log(app, "Could not read saved defaults: %s", err[0] ? err : "out of memory");
    }

    for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); i++) {
        if (defines_load(sources[i].header, sources[i].set) != 0) {
            app_log(app, "Warning: could not read defaults from %s", sources[i].header);
        }

        if (have_overrides) {
            const KVSet *saved = overrides_section(ov, sources[i].section, false);
            if (saved) {
                kv_merge(sources[i].set, saved);
            }
        }
    }

    free(ov);
}

/* Zeroes the App and initialises every tab. Call once, after the window exists (some tabs
 * create GPU resources). `app` must be heap-allocated: it is large. */
void app_init(App *app)
{
    memset(app, 0, sizeof(*app));
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

/* printf-style line into the Console tab, prefixed with a wall-clock time. */
void app_log(App *app, const char *fmt, ...)
{
    char msg[LOG_LEN - 24]; /* room for the "[HH:MM:SS.mmm] " prefix */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    char clock[16], line[LOG_LEN];
    format_clock(clock, sizeof(clock));
    snprintf(line, sizeof(line), "[%s] %s", clock, msg);
    console_tab_add(&app->ctab, line);
}

/* printf-style error shown as a modal dialog (does not touch the Console). */
void app_error(const char *title, const char *fmt, ...)
{
    char msg[768];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    ui_modal_open(title, msg, "OK", NULL, NULL);
}

bool app_require_connection(App *app)
{
    if (app->connected) {
        return true;
    }

    app_error("Not Connected", "Please connect to serial port first.");
    return false;
}

/* ------------------------------------------------------------------------ */
/* Connection                                                                */
/* ------------------------------------------------------------------------ */

void app_refresh_ports(App *app)
{
    app->nports = uart_list_ports(app->ports, UART_MAX_PORTS);
    if (app->port_active >= app->nports) {
        app->port_active = 0;
    }

    if (app->nports == 0) {
        app_log(app, "No valid serial ports detected.");
    }
}

/* Opens the selected port and starts the reader thread. On any failure nothing is left open and
 * the user is told why. */
void app_connect(App *app)
{
    if (app->nports == 0) {
        app_error("Connection Error", "No valid port selected.");
        return;
    }

    const char *device = app->ports[app->port_active].device;
    int baud = BAUD_RATES[app->baud_active];

    char err[256];
    int fd = uart_open(device, baud, err, sizeof(err));
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

static void ack_fail_all(App *app);

/* Safe to call when already disconnected. Order matters: the reader thread is joined BEFORE the
 * fd is closed, otherwise it could read from a closed (or reused) descriptor. Also stops any
 * recording and fails every pending config request. */
void app_disconnect(App *app)
{
    if (!app->connected && !app->tel) {
        return;
    }

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
    ack_fail_all(app); /* nothing can be answered any more */
}

/* ------------------------------------------------------------------------ */
/* Per-frame update                                                          */
/* ------------------------------------------------------------------------ */

/* One decoded telemetry sample, on the main thread. */
static void handle_sample(App *app, const Sample *s)
{
    /* Recording takes every packet, whatever the plots are doing. */
    recorder_add(&app->rec, s);

    if (!app->live_plots) {
        return;
    }

    app->last_rx = s->t;
    app->status = TS_RECEIVING;
    app->last = *s;
    app->have_last = true;
    telemetry_tab_push(&app->ttab, s, drone_tab_stick_scale(&app->dtab));
}

static void open_dropped_session(App *app)
{
    if (!IsFileDropped()) {
        return;
    }

    FilePathList files = LoadDroppedFiles();
    if (files.count > 0 && IsFileExtension(files.paths[0], ".csv")) {
        session_tab_open(app, files.paths[0]);
    }

    UnloadDroppedFiles(files);
}

/* ------------------------------------------------------------------------ */
/* Config acks                                                               */
/* ------------------------------------------------------------------------ */

/* ---- Ack tracking -----------------------------------------------------------------------
 * A request goes: queued -> in flight (written to the port, timer running) -> finished.
 * It finishes when the matching ack arrives (ok / rejected), when the retries run out, when the
 * write fails, or when the connection drops. Finishing calls the request's AckDone callback.
 *
 * One request per device may be in flight. The ack carries no request id, only its type, so
 * allowing two at once would make it impossible to tell which one an ack answered.
 */
enum {
    CH_PID,
    CH_DRONE,
    CH_REMOTE
};

static const uint8_t CHANNEL_ACK_ID[ACK_CHANNELS] = {PID_ACK, DRONE_CFG_ACK, REMOTE_CFG_ACK};

typedef struct {
    int channel;
    const char *what;
    double timeout;
    int attempts;
} AckRule;

/* Which channel a packet id belongs to and how patient to be. Returns false for packets that are
 * not tracked (sent fire-and-forget). Saving writes flash on the device, so it gets a longer
 * wait and fewer repeats. */
static bool ack_rule(uint8_t id, AckRule *r)
{
    switch (id) {
    case PID_CONFIG:
        *r = (AckRule){CH_PID, "PID gains", 0.6, 3};
        return true;
    case DRONE_CFG:
        *r = (AckRule){CH_DRONE, "Drone config", 0.6, 3};
        return true;
    case DRONE_CFG_STORE:
        *r = (AckRule){CH_DRONE, "Drone config save", 1.5, 2};
        return true;
    case REMOTE_CFG:
        *r = (AckRule){CH_REMOTE, "Remote config", 0.6, 3};
        return true;
    case REMOTE_CFG_STORE:
        *r = (AckRule){CH_REMOTE, "Remote config save", 1.5, 2};
        return true;
    default:
        return false;
    }
}

static const char *channel_device(int ch)
{
    return ch == CH_REMOTE ? "remote" : "drone";
}

static void set_cfg_status(App *app, bool ok, const char *what, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(app->cfg_result, sizeof(app->cfg_result), fmt, ap);
    va_end(ap);

    str_copy(app->cfg_what, sizeof(app->cfg_what), what);
    app->cfg_ok = ok;
    app->cfg_t = now_s();
    app_log(app, "%s: %s", app->cfg_what, app->cfg_result);
}

static const char *status_text(uint16_t status, char *buf, size_t cap)
{
    if (status == 0x102) { /* ESP_ERR_INVALID_ARG: the drone range-checks config values */
        return "invalid value";
    }

    snprintf(buf, cap, "error 0x%X", status);
    return buf;
}

static bool ack_transmit(App *app, AckChannel *c)
{
    if (uart_write_all(app->fd, c->cur.frame, c->cur.len) != 0) {
        return false;
    }

    c->cur.attempts++;
    c->sent_at = now_s();
    return true;
}

/* Reports a request's outcome to whoever asked for it. */
static void ack_finish(App *app, const AckRequest *r, bool ok)
{
    if (r->done) {
        r->done(app, r->user, ok);
    }
}

static bool ack_start(App *app, AckChannel *c, const AckRequest *r)
{
    c->cur = *r;
    c->cur.attempts = 0;
    if (!ack_transmit(app, c)) {
        c->in_flight = false;
        set_cfg_status(app, false, r->what, "send failed (%s)", strerror(errno));
        return false;
    }

    c->in_flight = true;
    return true;
}

/* Starts the next waiting request once the channel is free and quiet. */
static void ack_pump(App *app, AckChannel *c)
{
    if (c->in_flight || now_s() < c->quiet_until || c->qcount == 0) {
        return;
    }

    AckRequest next = c->queue[c->qhead];
    c->qhead = (c->qhead + 1) % ACK_QUEUE_CAP;
    c->qcount--;
    if (!ack_start(app, c, &next)) {
        ack_finish(app, &next, false);
    }
}

/* An ack arrived from the bridge: complete the request in flight on its channel, then start the
 * next queued one. Acks with nothing in flight (late duplicates) are ignored. */
static void handle_ack(App *app, const AckEvent *ev)
{
    for (int ch = 0; ch < ACK_CHANNELS; ch++) {
        if (CHANNEL_ACK_ID[ch] != ev->id) {
            continue;
        }

        AckChannel *c = &app->ack[ch];
        if (!c->in_flight || now_s() < c->quiet_until) {
            return; /* a late duplicate of an answer already used */
        }

        bool ok = ev->status == 0;
        char buf[24];
        if (ok) {
            set_cfg_status(app, true, c->cur.what, "confirmed");
        } else {
            set_cfg_status(app, false, c->cur.what, "rejected (%s)",
                           status_text(ev->status, buf, sizeof(buf)));
        }

        /* After a retry the first attempt's ack may still be on its way: let it pass
         * before the next request for this device starts. */
        AckRequest finished = c->cur;
        c->quiet_until = finished.attempts > 1 ? now_s() + 0.4 : 0.0;
        c->in_flight = false;
        ack_finish(app, &finished, ok);
        ack_pump(app, c);
        return;
    }
}

/* Per-frame timer work for every channel: re-send a request whose ack is overdue, give up when it
 * is out of tries, and start queued requests. */
static void ack_tick(App *app)
{
    for (int ch = 0; ch < ACK_CHANNELS; ch++) {
        AckChannel *c = &app->ack[ch];

        if (c->in_flight && now_s() - c->sent_at > c->cur.timeout) {
            bool give_up = false;
            if (c->cur.attempts < c->cur.max_attempts) {
                app_log(app, "%s: no ack from the %s yet, sending again (%d of %d)", c->cur.what,
                        channel_device(ch), c->cur.attempts + 1, c->cur.max_attempts);
                if (!ack_transmit(app, c)) {
                    set_cfg_status(app, false, c->cur.what, "send failed (%s)", strerror(errno));
                    give_up = true;
                }
            } else {
                set_cfg_status(app, false, c->cur.what, "no confirmation (%d tries)",
                               c->cur.max_attempts);
                c->quiet_until = now_s() + 0.4;
                give_up = true;
            }

            if (give_up) {
                AckRequest finished = c->cur;
                c->in_flight = false;
                ack_finish(app, &finished, false);
            }
        }

        ack_pump(app, c);
    }
}

/* On disconnect every waiting request is reported as unconfirmed, then the tracker is cleared. */
static void ack_fail_all(App *app)
{
    for (int ch = 0; ch < ACK_CHANNELS; ch++) {
        AckChannel *c = &app->ack[ch];
        if (c->in_flight) {
            AckRequest r = c->cur;
            c->in_flight = false;
            ack_finish(app, &r, false);
        }

        while (c->qcount > 0) {
            AckRequest r = c->queue[c->qhead];
            c->qhead = (c->qhead + 1) % ACK_QUEUE_CAP;
            c->qcount--;
            ack_finish(app, &r, false);
        }
    }

    memset(app->ack, 0, sizeof(app->ack));
}

/* The per-frame "model" step; the drawing code runs after it. Safe to call when disconnected. */
void app_update(App *app)
{
    if (app->tel) {
        Sample batch[128];
        int n;
        while ((n = telemetry_poll(app->tel, batch, 128)) > 0) {
            for (int i = 0; i < n; i++) {
                handle_sample(app, &batch[i]);
            }
        }

        AckEvent acks[16];
        while ((n = telemetry_poll_acks(app->tel, acks, 16)) > 0) {
            for (int i = 0; i < n; i++) {
                handle_ack(app, &acks[i]);
            }
        }

        ack_tick(app);

        char err[160];
        if (telemetry_lost(app->tel, err, sizeof(err))) {
            app_log(app, "Serial connection lost: %s", err);
            app_disconnect(app);
            app_error("Connection Lost", "The serial connection was lost:\n%s", err);
        }
    }

    if (app->connected && app->status == TS_RECEIVING && now_s() - app->last_rx > 2.0) {
        app->status = TS_TIMEOUT;
    }

    open_dropped_session(app);
}

/* ------------------------------------------------------------------------ */
/* Sending / saving                                                          */
/* ------------------------------------------------------------------------ */

/* See app.h for the contract. Internally: untracked packet ids are written straight away;
 * tracked ones become an AckRequest that is started now if the device's channel is idle and
 * otherwise appended to that channel's queue. */
bool app_send_then(App *app, uint8_t id, const void *payload, size_t len, AckDone done,
                   const void *user, size_t user_len)
{
    if (!app_require_connection(app)) {
        return false;
    }

    if (user_len > ACK_USER_MAX) {
        app_log(app, "Internal error: ack context too large (%zu bytes)", user_len);
        return false;
    }

    uint8_t frame[UART_CONFIG_FRAME_SIZE];
    size_t n = uart_encode_config_packet(id, payload, len, frame, sizeof(frame));
    if (n == 0) {
        app_error("Send Error", "Failed to send: payload does not fit in a packet.");
        return false;
    }

    AckRule rule;
    if (ack_rule(id, &rule)) {
        /* Tracked: wait for the device's ack, re-sending if it doesn't come */
        AckChannel *c = &app->ack[rule.channel];
        AckRequest req = {.len = n,
                          .done = done,
                          .user_len = user_len,
                          .what = rule.what,
                          .timeout = rule.timeout,
                          .max_attempts = rule.attempts};
        memcpy(req.frame, frame, n);
        if (user_len) {
            memcpy(req.user, user, user_len);
        }

        if (c->in_flight || now_s() < c->quiet_until || c->qcount > 0) {
            if (c->qcount == ACK_QUEUE_CAP) {
                app_log(app, "%s: too many requests waiting, dropped", rule.what);
                return false;
            }

            c->queue[(c->qhead + c->qcount) % ACK_QUEUE_CAP] = req;
            c->qcount++;
            return true;
        }

        if (!ack_start(app, c, &req)) {
            app_error("Send Error", "Failed to send: %s", strerror(errno));
            return false;
        }

        return true;
    }

    if (uart_write_all(app->fd, frame, n) != 0) {
        const char *why = strerror(errno);
        app_log(app, "Send failed: %s", why);
        app_error("Send Error", "Failed to send: %s", why);
        return false;
    }

    if (done) {
        done(app, user, true); /* nothing to wait for */
    }

    return true;
}

bool app_send(App *app, uint8_t id, const void *payload, size_t len)
{
    return app_send_then(app, id, payload, len, NULL, NULL, 0);
}

/* Read-modify-write of user_defaults.json: the file is re-read first so edits made by anything
 * else are kept. If the file exists but cannot be parsed nothing is written (so it is never
 * clobbered). The live table is only updated once the file write succeeded. */
bool app_save_defaults(App *app, const char *section, const KVSet *updates, KVSet *live)
{
    Overrides *ov = malloc(sizeof(*ov));
    if (!ov) {
        return false;
    }

    char err[256] = "";
    bool ok = false;
    if (overrides_load(ov, USER_DEFAULTS_PATH, err, sizeof(err)) == 0) {
        KVSet *sec = overrides_section(ov, section, true);
        if (sec) {
            kv_merge(sec, updates);
            ok = overrides_save(ov, USER_DEFAULTS_PATH, err, sizeof(err)) == 0;
        } else {
            snprintf(err, sizeof(err), "too many sections");
        }
    }

    free(ov);

    if (ok) {
        kv_merge(live, updates);
    } else {
        app_log(app, "Could not save defaults: %s", err);
    }

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
    if (recorder_start(&app->rec, SESSIONS_DIR, err, sizeof(err)) != 0) {
        app_error("Recording Error", "Could not start: %s", err);
        return;
    }

    app_log(app, "Recording session to %s", app->rec.path);
}

void app_stop_recording(App *app)
{
    int rows = app->rec.rows;
    char path[sizeof(app->rec.path)];
    str_copy(path, sizeof(path), app->rec.path);
    if (recorder_stop(&app->rec)) {
        app_log(app, "Session saved (%d samples): %s", rows, path);
    }
}

void app_record_event(App *app, const char *text)
{
    recorder_event(&app->rec, text);
}
