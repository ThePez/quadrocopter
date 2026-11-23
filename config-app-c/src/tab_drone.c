/*
 *****************************************************************************
 * File: tab_drone.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Drone config tab: limits, throttle range, gyro filter, failsafes.
 *****************************************************************************
 */

/*
 * Drone config tab: flight limits, throttle range, gyro filter and failsafe thresholds.
 *
 * The rows are table driven (FIELDS[]): each has a label, the #define name it takes its default
 * from (and is saved under), a unit scale (the GUI shows seconds where the firmware stores
 * microseconds), and its step / min / max / decimals. The whole table is sent as one
 * `drone_config_telemetry_t` packet.
 *
 * Send applies live; Save also persists to flash on the drone and keeps the values as defaults,
 * but only after the drone has confirmed each step (see on_config_done / on_store_done).
 * Changing the wire struct means rebuilding and flashing the drone as well as this app.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"

enum {
    D_MAX_RATE,
    D_MAX_ANGLE,
    D_FAIL_ANGLE,
    D_MIN_THROTTLE,
    D_MAX_THROTTLE,
    D_COMS_TIMEOUT_S,
    D_LOW_VOLTAGE,
    D_CRITICAL_VOLTAGE,
    D_GYRO_ALPHA,
};

/* Control loop rate the gyro filter cutoff hint is worked out for (Hz). */
#define LOOP_RATE_HZ 400.0

typedef struct {
    const char *label;
    const char *define; /* name in drone_defaults.h / user_defaults.json */
    double scale;       /* GUI value = define value / scale */
    double step, min, max;
    int decimals;
} DroneField;

static const DroneField FIELDS[DRONE_FIELDS] = {
    [D_MAX_RATE] = {"Max Rate (deg/s)", "MAX_RATE", 1.0, 1.0, 0, 1000000, 2},
    [D_MAX_ANGLE] = {"Max Angle (deg)", "MAX_ANGLE", 1.0, 1.0, 0, 1000000, 2},
    [D_FAIL_ANGLE] = {"Fail Angle (deg)", "FAIL_ANGLE", 1.0, 1.0, 0, 1000000, 2},
    [D_MIN_THROTTLE] = {"Min Throttle (us)", "MIN_THROTTLE", 1.0, 1.0, 0, 1000000, 2},
    [D_MAX_THROTTLE] = {"Max Throttle (us)", "MAX_THROTTLE", 1.0, 1.0, 0, 1000000, 2},
    [D_COMS_TIMEOUT_S] = {"Coms Timeout (s)", "FAILSAFE_TIMEOUT_US", 1e6, 1.0, 0, 1000000, 2},
    [D_LOW_VOLTAGE] = {"Low Voltage (mV)", "LOW_VOLTAGE", 1.0, 1.0, 0, 1000000, 2},
    [D_CRITICAL_VOLTAGE] = {"Critical Voltage (mV)", "CRITICAL_VOLTAGE", 1.0, 1.0, 0, 1000000, 2},
    [D_GYRO_ALPHA] = {"Gyro filter alpha", "GYRO_FILTER_ALPHA", 1.0, 0.05, 0.05, 1.0, 2},
};

StickScale drone_tab_stick_scale(const DroneTab *t)
{
    return (StickScale){(float)t->f[D_MAX_RATE].value, (float)t->f[D_MIN_THROTTLE].value,
                        (float)t->f[D_MAX_THROTTLE].value};
}

void drone_tab_init(DroneTab *t, App *app)
{
    memset(t, 0, sizeof(*t));
    t->app = app;
    for (int i = 0; i < DRONE_FIELDS; i++) {
        double v = kv_get(&app->drone_defaults, FIELDS[i].define, 0.0) / FIELDS[i].scale;
        /* a missing/zero alpha would be rejected by the drone, so fall back to unfiltered */
        if (i == D_GYRO_ALPHA && v < FIELDS[i].min) {
            v = 1.0;
        }

        numfield_init(&t->f[i], v, FIELDS[i].step, FIELDS[i].min, FIELDS[i].max,
                      FIELDS[i].decimals);
    }
}

static uint16_t to_u16(double v)
{
    if (v < 0) {
        return 0;
    }

    if (v > 65535) {
        return 65535;
    }

    return (uint16_t)llround(v);
}

static void current_values(const DroneTab *t, double out[DRONE_FIELDS])
{
    for (int i = 0; i < DRONE_FIELDS; i++) {
        out[i] = t->f[i].value;
    }
}

/* What a request hands to its completion callback: a copy, since the fields may have changed
 * by the time the ack arrives. */
typedef struct {
    double v[DRONE_FIELDS];
    bool then_save; /* follow with the flash save once this update is confirmed */
} DroneSnap;

static void on_store_done(App *app, const void *user, bool ok)
{
    const DroneSnap *s = user;
    if (!ok) {
        app_log(app,
                "Drone config: not saved as default (the drone did not confirm the flash save)");
        return;
    }

    /* The drone has it in flash: now remember the values as this GUI's working defaults. */
    KVSet updates = {0};
    for (int i = 0; i < DRONE_FIELDS; i++) {
        kv_set(&updates, FIELDS[i].define, s->v[i] * FIELDS[i].scale);
    }

    if (app_save_defaults(app, "drone", &updates, &app->drone_defaults)) {
        app_log(app, "Saved drone config tab as default");
    } else {
        app_log(app, "Failed to save drone config defaults");
    }
}

static bool send_store(App *app, const DroneSnap *s)
{
    if (!app_send_then(app, DRONE_CFG_STORE, NULL, 0, on_store_done, s, sizeof(*s))) {
        return false;
    }

    app_log(app, "-> Sent DRONE_CFG_STORE (persist to flash)");
    return true;
}

static void on_config_done(App *app, const void *user, bool ok)
{
    DroneSnap s =
        *(const DroneSnap *)user; /* a copy: send_store() below may start another request */
    DroneTab *t = &app->dtab;

    if (!ok) {
        if (s.then_save) {
            app_log(app, "Drone config: save cancelled, the update was not applied");
        }

        return;
    }

    /* Confirmed: only now is this what the drone is running */
    memcpy(t->sent, s.v, sizeof(s.v));
    t->sent_valid = true;

    if (s.then_save) {
        send_store(app, &s);
    }
}

/* Sends the config live (RAM only). It only counts as sent once the drone confirms. */
static bool send_config(DroneTab *t, bool then_save)
{
    App *app = t->app;
    if (!app_require_connection(app)) {
        return false;
    }

    double v[DRONE_FIELDS];
    current_values(t, v);

    struct drone_config_telemetry_t p = {
        .max_rate = (float)v[D_MAX_RATE],
        .max_angle = (float)v[D_MAX_ANGLE],
        .fail_angle = (float)v[D_FAIL_ANGLE],
        .min_throttle = (float)v[D_MIN_THROTTLE],
        .max_throttle = (float)v[D_MAX_THROTTLE],
        .coms_timeout_us = (uint32_t)llround(v[D_COMS_TIMEOUT_S] * 1e6),
        .low_voltage = to_u16(v[D_LOW_VOLTAGE]),
        .critical_voltage = to_u16(v[D_CRITICAL_VOLTAGE]),
        .gyro_alpha = (float)v[D_GYRO_ALPHA],
    };
    DroneSnap snap = {.then_save = then_save};
    memcpy(snap.v, v, sizeof(v));
    if (!app_send_then(app, DRONE_CFG, &p, sizeof(p), on_config_done, &snap, sizeof(snap))) {
        return false;
    }

    app_log(app, "-> Sent drone config update (live, not yet saved), waiting for confirmation");
    return true;
}

/* Saving writes flash on the drone: the update goes first if the values changed, the save follows
 * once the drone confirms it, and the values become this GUI's defaults once the save is confirmed.
 */
static void do_save(DroneTab *t)
{
    App *app = t->app;
    double v[DRONE_FIELDS];
    current_values(t, v);

    if (!t->sent_valid || memcmp(t->sent, v, sizeof(v)) != 0) {
        send_config(t, true);
        return;
    }

    DroneSnap snap = {0};
    memcpy(snap.v, v, sizeof(v));
    send_store(app, &snap);
}

static void on_save_confirmed(void *ctx, int button)
{
    if (button == 0) {
        do_save(ctx);
    }
}

/* Draws one card of label/field rows, taking fields from `first`. */
static void field_card(DroneTab *t, Rectangle r, const char *title, const int *idx, int n)
{
    ui_card(r, title);
    Rectangle b = ui_card_body(r, true);
    for (int i = 0; i < n; i++) {
        ui_field_row((Rectangle){b.x, b.y + (float)i * UI_ROW_H, b.width, UI_ROW_H},
                     FIELDS[idx[i]].label, &t->f[idx[i]]);
    }
}

void drone_tab_draw(App *app, Rectangle area)
{
    DroneTab *t = &app->dtab;

    static const int LIMITS[] = {D_MAX_RATE, D_MAX_ANGLE, D_FAIL_ANGLE};
    static const int THROTTLE[] = {D_MIN_THROTTLE, D_MAX_THROTTLE, D_GYRO_ALPHA};
    static const int SAFETY[] = {D_COMS_TIMEOUT_S, D_LOW_VOLTAGE, D_CRITICAL_VOLTAGE};

    /* One extra line under the rows for the gyro filter's cutoff hint */
    float card_h = UI_CARD_HEAD + 3 * UI_ROW_H + UI_PAD + 16;
    Rectangle row = {area.x, area.y, area.width, card_h};
    field_card(t, ui_col(row, 3, 0, UI_GAP), "Flight limits", LIMITS, 3);

    Rectangle col1 = ui_col(row, 3, 1, UI_GAP);
    field_card(t, col1, "Throttle & gyro filter", THROTTLE, 3);
    double alpha = t->f[D_GYRO_ALPHA].value;
    char hint[48];
    if (alpha >= 1.0) {
        snprintf(hint, sizeof(hint), "gyro unfiltered");
    } else {
        snprintf(hint, sizeof(hint), "gyro ~ %.0f Hz cutoff",
                 alpha / (1.0 - alpha) * LOOP_RATE_HZ / (2.0 * M_PI));
    }

    ui_text_right(hint, col1.x + col1.width - UI_PAD, col1.y + UI_CARD_HEAD + 3 * UI_ROW_H,
                  UI_SMALL, UI_MUTED);

    field_card(t, ui_col(row, 3, 2, UI_GAP), "Failsafe & battery", SAFETY, 3);

    /* Actions */
    Rectangle bar = {area.x, area.y + card_h + UI_GAP, area.width, 84};
    ui_card(bar, NULL);
    ui_text("Send applies these values to the drone's RAM straight away.", bar.x + UI_PAD,
            bar.y + 18, UI_TEXT_SIZE, UI_TEXT);
    ui_text("Save also writes them to flash and keeps them as your local defaults.", bar.x + UI_PAD,
            bar.y + 44, UI_TEXT_SIZE - 1, UI_MUTED);

    float bx = bar.x + bar.width - UI_PAD;
    Rectangle save_r = {bx - 210, bar.y + (bar.height - UI_BTN_H) / 2, 210, UI_BTN_H};
    Rectangle send_r = {save_r.x - 10 - 160, save_r.y, 160, UI_BTN_H};
    if (ui_button(send_r, "Send to drone", UI_ACCENT)) {
        send_config(t, false);
    }

    if (ui_button(save_r, "Save to flash & default", UI_ORANGE) && app_require_connection(app)) {
        ui_modal_open(
            "Save to Flash & Default",
            "Persist the drone's currently-live config (including PID gains) to flash, and "
            "save these values as your local working defaults?",
            "Yes;No", on_save_confirmed, t);
    }

    /* Live response next to the limits being tuned. */
    float py = bar.y + bar.height + UI_GAP;
    Rectangle below = {area.x, py, area.width, area.y + area.height - py};
    if (below.height >= 170) {
        plot_draw(&app->ttab.angles, ui_col(below, 2, 0, UI_GAP));
        plot_draw(&app->ttab.motors, ui_col(below, 2, 1, UI_GAP));
    }
}
