/*
 *****************************************************************************
 * File: tab_pid.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: PID tuning tab: three gain groups, send, save and offline editing.
 *****************************************************************************
 */

/*
 * PID tuning tab: three gain groups (Rate pitch/roll, Rate yaw, Angle pitch/roll), each with
 * Kp, Ki, Kd and the D-term filter alpha.
 *
 * Per group:
 *   Send to drone         live update (drone RAM); confirmed by the drone's ack
 *   Save to flash & ...   makes sure the on-screen gains are confirmed, tells the drone to store
 *                         its config in flash, then remembers the gains in user_defaults.json
 * Whole tab:
 *   Save all to file      writes every group to user_defaults.json (no drone needed)
 *   Send all to drone     live-sends every group (use after powering the drone)
 *
 * A group's `state` tracks the last Send: IDLE (confirmed), PENDING (waiting for the ack) or
 * FAILED (gave up). `current` is the text of the gains the drone last confirmed; `attempt` is the
 * text of an unconfirmed Send, shown with a warning flag until it is confirmed.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"

/* Control loop rate the D-filter cutoff hint is worked out for (Hz). */
#define LOOP_RATE_HZ 400.0

static const char *const AXIS_NAMES[2] = {"Pitch / Roll", "Yaw"};
static const char *const MODE_NAMES[2] = {"Rate", "Angle"};

/* Cutoff of the drone's first-order D-term filter, or a negative value when
 * the filter is off (alpha = 1). */
static double d_alpha_to_hz(double alpha)
{
    if (alpha >= 1.0) {
        return -1.0;
    }

    return alpha / (1.0 - alpha) * LOOP_RATE_HZ / (2.0 * M_PI);
}

static void d_filter_text(double alpha, char *out, size_t cap)
{
    double hz = d_alpha_to_hz(alpha);
    if (hz < 0) {
        snprintf(out, cap, "unfiltered");
    } else {
        snprintf(out, cap, "~ %.0f Hz cutoff", hz);
    }
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */
/* ------------------------------------------------------------------------ */

static void init_group(PidGroup *g, App *app, int axis, int mode, const char *title,
                       const char *prefix)
{
    memset(g, 0, sizeof(*g));
    g->app = app;
    g->axis = axis;
    g->mode = mode;
    g->title = title;
    g->defaults_prefix = prefix;

    char key[64];
    snprintf(key, sizeof(key), "%s_KP", prefix);
    numfield_init(&g->kp, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof(key), "%s_KI", prefix);
    numfield_init(&g->ki, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof(key), "%s_KD", prefix);
    numfield_init(&g->kd, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof(key), "%s_KD_ALPHA", prefix);
    numfield_init(&g->alpha, kv_get(&app->pid_defaults, key, 1.0), 0.05, 0.01, 1.0, 2);
}

void pid_tab_init(PidTab *t, App *app)
{
    init_group(&t->group[0], app, 0, 0, "Rate Mode PID - Pitch/Roll", "RATE_PITCH_ROLL");
    init_group(&t->group[1], app, 1, 0, "Rate Mode PID - Yaw", "RATE_YAW");
    init_group(&t->group[2], app, 0, 1, "Angle Mode PID - Pitch/Roll", "ANGLE_PITCH_ROLL");
}

/* ------------------------------------------------------------------------ */
/* Actions                                                                   */
/* ------------------------------------------------------------------------ */

/* What a request hands to its completion callback: a copy, since the fields may have changed
 * by the time the ack arrives. */
typedef struct {
    int group;
    double kp, ki, kd, alpha;
    bool then_save; /* follow with the flash save once this update is confirmed */
} PidSnap;

static void snap_text(const PidSnap *s, char *out, size_t cap)
{
    snprintf(out, cap, "Kp %.4g  Ki %.4g  Kd %.4g  a %.2g", s->kp, s->ki, s->kd, s->alpha);
}

static void on_store_done(App *app, const void *user, bool ok)
{
    const PidSnap *s = user;
    PidGroup *g = &app->ptab.group[s->group];

    if (!ok) {
        app_log(app, "%s %s: not saved as default (the drone did not confirm the flash save)",
                AXIS_NAMES[g->axis], g->title);
        return;
    }

    /* The drone has it in flash: now remember the gains as this GUI's working defaults. */
    char key[64];
    KVSet updates = {0};
    snprintf(key, sizeof(key), "%s_KP", g->defaults_prefix);
    kv_set(&updates, key, s->kp);
    snprintf(key, sizeof(key), "%s_KI", g->defaults_prefix);
    kv_set(&updates, key, s->ki);
    snprintf(key, sizeof(key), "%s_KD", g->defaults_prefix);
    kv_set(&updates, key, s->kd);
    snprintf(key, sizeof(key), "%s_KD_ALPHA", g->defaults_prefix);
    kv_set(&updates, key, s->alpha);

    if (app_save_defaults(app, "pid", &updates, &app->pid_defaults)) {
        app_log(app, "Saved %s %s as default: Kp=%.4f, Ki=%.4f, Kd=%.4f", AXIS_NAMES[g->axis],
                g->title, s->kp, s->ki, s->kd);
    } else {
        app_log(app, "Failed to save %s %s defaults", AXIS_NAMES[g->axis], g->title);
    }
}

static bool send_store(App *app, const PidSnap *s)
{
    if (!app_send_then(app, DRONE_CFG_STORE, NULL, 0, on_store_done, s, sizeof(*s))) {
        return false;
    }

    app_log(app, "-> Sent DRONE_CFG_STORE (persist to flash)");
    return true;
}

static void on_pid_done(App *app, const void *user, bool ok)
{
    PidSnap s = *(const PidSnap *)user; /* a copy: send_store() below may start another request */
    PidGroup *g = &app->ptab.group[s.group];

    if (!ok) {
        g->state = PID_SENT_FAILED;
        if (s.then_save) {
            app_log(app, "%s %s: save cancelled, the gains were not applied", AXIS_NAMES[g->axis],
                    g->title);
        }

        return;
    }

    /* Confirmed: only now is this what the drone is running */
    g->state = PID_SENT_IDLE;
    g->sent_valid = true;
    g->sent_kp = s.kp;
    g->sent_ki = s.ki;
    g->sent_kd = s.kd;
    g->sent_alpha = s.alpha;
    snprintf(g->current, sizeof(g->current), "Kp %.4g   Ki %.4g   Kd %.4g   alpha %.2g", s.kp, s.ki,
             s.kd, s.alpha);
    g->current_set = true;

    char event[160];
    snprintf(event, sizeof(event), "%s %s: Kp=%.4f Ki=%.4f Kd=%.4f a=%.2f", AXIS_NAMES[g->axis],
             MODE_NAMES[g->mode], s.kp, s.ki, s.kd, s.alpha);
    app_record_event(app, event);

    if (s.then_save) {
        send_store(app, &s);
    }
}

/* Sends the group's gains live (RAM only). They only count as sent, and only show as the
 * drone's current gains, once the drone confirms. */
static bool send_pid(PidGroup *g, bool then_save)
{
    App *app = g->app;
    if (!app_require_connection(app)) {
        return false;
    }

    if (g->axis == 1 && g->mode != 0) {
        app_error("Invalid Mode", "Yaw only supports Rate mode.");
        return false;
    }

    struct pid_config_telemetry_t p = {
        .kp = (float)g->kp.value,
        .ki = (float)g->ki.value,
        .kd = (float)g->kd.value,
        .d_alpha = (float)g->alpha.value,
        .axis = (uint16_t)g->axis,
        .mode = (uint16_t)g->mode,
    };
    PidSnap s = {.group = (int)(g - app->ptab.group),
                 .kp = g->kp.value,
                 .ki = g->ki.value,
                 .kd = g->kd.value,
                 .alpha = g->alpha.value,
                 .then_save = then_save};
    if (!app_send_then(app, PID_CONFIG, &p, sizeof(p), on_pid_done, &s, sizeof(s))) {
        return false;
    }

    g->state = PID_SENT_PENDING;
    snap_text(&s, g->attempt, sizeof(g->attempt));

    char hint[48];
    d_filter_text(g->alpha.value, hint, sizeof(hint));
    app_log(app,
            "-> Sent %s %s: Kp=%.4f, Ki=%.4f, Kd=%.4f, D filter alpha=%.2f (%s), waiting for "
            "confirmation",
            AXIS_NAMES[g->axis], MODE_NAMES[g->mode], g->kp.value, g->ki.value, g->kd.value,
            g->alpha.value, hint);
    return true;
}

static bool values_match_sent(const PidGroup *g)
{
    return g->sent_valid && g->sent_kp == g->kp.value && g->sent_ki == g->ki.value &&
           g->sent_kd == g->kd.value && g->sent_alpha == g->alpha.value;
}

/* The drone keeps PID gains inside its own config, so "save" means: make sure the live gains are
 * what's on screen (and confirmed), tell the drone to persist its config, and, once that is
 * confirmed too, remember the gains as this GUI's working defaults. */
static void do_save(PidGroup *g)
{
    App *app = g->app;

    if (!values_match_sent(g)) {
        send_pid(g, true); /* the save follows once the drone confirms the gains */
        return;
    }

    PidSnap s = {.group = (int)(g - app->ptab.group),
                 .kp = g->kp.value,
                 .ki = g->ki.value,
                 .kd = g->kd.value,
                 .alpha = g->alpha.value};
    send_store(app, &s);
}

static void on_save_confirmed(void *ctx, int button)
{
    if (button == 0) {
        do_save(ctx);
    }
}

static void request_save(PidGroup *g)
{
    if (!app_require_connection(g->app)) {
        return;
    }

    char msg[512];
    snprintf(msg, sizeof(msg),
             "Persist the drone's currently-live config to flash, and save the %s %s gains as "
             "your local working default:\nKp=%.4f, Ki=%.4f, Kd=%.4f, D filter alpha=%.2f?",
             AXIS_NAMES[g->axis], g->title, g->kp.value, g->ki.value, g->kd.value, g->alpha.value);
    ui_modal_open("Save to Flash & Default", msg, "Yes;No", on_save_confirmed, g);
}

/* ---- Offline editing: keep every group's gains in the defaults file, push them later ---- */

/* Writes all three groups' on-screen gains to the defaults file. No drone needed: they are
 * loaded back into the fields on the next start, ready for "Send all to drone". */
static void save_all_to_file(App *app)
{
    KVSet updates = {0};
    char key[64];
    for (int i = 0; i < 3; i++) {
        const PidGroup *g = &app->ptab.group[i];
        const struct {
            const char *suffix;
            double value;
        } items[4] = {{"KP", g->kp.value},
                      {"KI", g->ki.value},
                      {"KD", g->kd.value},
                      {"KD_ALPHA", g->alpha.value}};
        for (int k = 0; k < 4; k++) {
            snprintf(key, sizeof(key), "%s_%s", g->defaults_prefix, items[k].suffix);
            kv_set(&updates, key, items[k].value);
        }
    }

    if (app_save_defaults(app, "pid", &updates, &app->pid_defaults)) {
        app_log(app, "Saved all PID gains to the defaults file (not sent to the drone)");
    }
}

/* Sends every group's gains live (RAM only). Each is confirmed by its own ack. */
static void send_all(App *app)
{
    if (!app_require_connection(app)) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        send_pid(&app->ptab.group[i], false);
    }
}

/* ------------------------------------------------------------------------ */
/* Drawing                                                                   */
/* ------------------------------------------------------------------------ */

#define PID_CARD_H 372

static const char *const CARD_TITLES[3] = {"Rate  -  Pitch / Roll", "Rate  -  Yaw",
                                           "Angle  -  Pitch / Roll"};

static void draw_group(PidGroup *g, const char *title, Rectangle r)
{
    ui_card(r, title);
    Rectangle b = ui_card_body(r, true);
    float y = b.y;

    /* What the drone is currently running (as of the last Send). */
    Rectangle chip = {b.x, y, b.width, 30};
    ui_rrect(chip, 6, UI_FIELD);
    if (g->state != PID_SENT_IDLE) {
        /* The last Send hasn't been confirmed: show its gains, flagged */
        bool pending = g->state == PID_SENT_PENDING;
        Color flag = pending ? UI_ORANGE : UI_RED;
        const char *label = pending ? "unconfirmed" : "not confirmed";
        ui_dot(chip.x + 13, chip.y + 15, 3.5f, flag);
        ui_text(g->attempt, chip.x + 24, chip.y + 7, UI_SMALL, UI_TEXT);
        ui_text_right(label, chip.x + chip.width - 10, chip.y + 7, UI_SMALL, flag);
    } else if (g->current_set) {
        ui_dot(chip.x + 13, chip.y + 15, 3.5f, UI_GREEN);
        ui_text(g->current, chip.x + 24, chip.y + 7, UI_SMALL, UI_TEXT);
    } else {
        ui_text("Not sent yet - showing defaults", chip.x + 12, chip.y + 7, UI_SMALL, UI_MUTED);
    }

    y += 40;

    ui_field_row((Rectangle){b.x, y, b.width, UI_ROW_H}, "Kp", &g->kp);
    y += UI_ROW_H;
    ui_field_row((Rectangle){b.x, y, b.width, UI_ROW_H}, "Ki", &g->ki);
    y += UI_ROW_H;
    ui_field_row((Rectangle){b.x, y, b.width, UI_ROW_H}, "Kd", &g->kd);
    y += UI_ROW_H;
    ui_field_row((Rectangle){b.x, y, b.width, UI_ROW_H}, "D filter alpha", &g->alpha);
    y += UI_ROW_H - 4;

    char hint[48];
    d_filter_text(g->alpha.value, hint, sizeof(hint));
    ui_text_right(hint, b.x + b.width, y, UI_SMALL, UI_MUTED);
    if (g->axis == 1) {
        ui_text("Rate mode only", b.x, y, UI_SMALL, UI_MUTED);
    }

    float by = b.y + b.height - UI_BTN_H;
    if (ui_button((Rectangle){b.x, by, b.width, UI_BTN_H}, "Save to flash & default", UI_ORANGE)) {
        request_save(g);
    }

    if (ui_button((Rectangle){b.x, by - UI_BTN_H - 8, b.width, UI_BTN_H}, "Send to drone",
                  UI_ACCENT)) {
        send_pid(g, false);
    }
}

void pid_tab_draw(App *app, Rectangle area)
{
    PidTab *t = &app->ptab;

    /* Whole-tab actions: edit with the drone off, save to the file, push when it is on */
    Rectangle bar = {area.x, area.y, area.width, UI_BTN_H};
    ui_text("Edit offline, save to the file, then send all once connected", bar.x, bar.y + 9,
            UI_SMALL, UI_MUTED);
    float bw = 190;
    if (ui_button((Rectangle){bar.x + bar.width - bw, bar.y, bw, UI_BTN_H}, "Send all to drone",
                  UI_ACCENT)) {
        send_all(app);
    }

    if (ui_button((Rectangle){bar.x + bar.width - 2 * bw - 8, bar.y, bw, UI_BTN_H},
                  "Save all to file", UI_ORANGE)) {
        save_all_to_file(app);
    }

    area.y += UI_BTN_H + UI_GAP;
    area.height -= UI_BTN_H + UI_GAP;

    float cards_h = fminf(PID_CARD_H, area.height);
    Rectangle row = {area.x, area.y, area.width, cards_h};
    for (int i = 0; i < 3; i++) {
        draw_group(&t->group[i], CARD_TITLES[i], ui_col(row, 3, i, UI_GAP));
    }

    /* The live response, so gains can be judged without leaving the page. */
    float py = area.y + cards_h + UI_GAP;
    Rectangle below = {area.x, py, area.width, area.y + area.height - py};
    if (below.height >= 170) {
        plot_draw(&app->ttab.angles, ui_col(below, 2, 0, UI_GAP));
        plot_draw(&app->ttab.rates, ui_col(below, 2, 1, UI_GAP));
    }
}
