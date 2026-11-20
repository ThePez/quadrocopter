/* Drone config tab: failsafe / throttle / battery constants. */

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
};

typedef struct {
    const char *label;
    const char *define; /* name in drone_defaults.h / user_defaults.json */
    double scale;       /* GUI value = define value / scale */
} DroneField;

static const DroneField FIELDS[DRONE_FIELDS] = {
    [D_MAX_RATE] = {"Max Rate (deg/s)", "MAX_RATE", 1.0},
    [D_MAX_ANGLE] = {"Max Angle (deg)", "MAX_ANGLE", 1.0},
    [D_FAIL_ANGLE] = {"Fail Angle (deg)", "FAIL_ANGLE", 1.0},
    [D_MIN_THROTTLE] = {"Min Throttle (us)", "MIN_THROTTLE", 1.0},
    [D_MAX_THROTTLE] = {"Max Throttle (us)", "MAX_THROTTLE", 1.0},
    [D_COMS_TIMEOUT_S] = {"Coms Timeout (s)", "FAILSAFE_TIMEOUT_US", 1e6},
    [D_LOW_VOLTAGE] = {"Low Voltage (mV)", "LOW_VOLTAGE", 1.0},
    [D_CRITICAL_VOLTAGE] = {"Critical Voltage (mV)", "CRITICAL_VOLTAGE", 1.0},
};

StickScale drone_tab_stick_scale(const DroneTab *t)
{
    return (StickScale){(float)t->f[D_MAX_RATE].value, (float)t->f[D_MIN_THROTTLE].value,
                        (float)t->f[D_MAX_THROTTLE].value};
}

void drone_tab_init(DroneTab *t, App *app)
{
    memset(t, 0, sizeof *t);
    t->app = app;
    for (int i = 0; i < DRONE_FIELDS; i++) {
        double v = kv_get(&app->drone_defaults, FIELDS[i].define, 0.0) / FIELDS[i].scale;
        numfield_init(&t->f[i], v, 1.0, 0, 1000000, 2);
    }
}

static uint16_t to_u16(double v)
{
    if (v < 0)
        return 0;
    if (v > 65535)
        return 65535;
    return (uint16_t)llround(v);
}

static void current_values(const DroneTab *t, double out[DRONE_FIELDS])
{
    for (int i = 0; i < DRONE_FIELDS; i++)
        out[i] = t->f[i].value;
}

static bool send_config(DroneTab *t)
{
    App *app = t->app;
    if (!app_require_connection(app))
        return false;

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
    };
    if (!app_send(app, DRONE_CFG, &p, sizeof p))
        return false;

    app_log(app, "-> Sent drone config update (live, not yet saved)");
    memcpy(t->sent, v, sizeof v);
    t->sent_valid = true;
    return true;
}

static void do_save(DroneTab *t)
{
    App *app = t->app;
    double v[DRONE_FIELDS];
    current_values(t, v);

    if ((!t->sent_valid || memcmp(t->sent, v, sizeof v) != 0) && !send_config(t))
        return;

    if (!app_send(app, DRONE_CFG_STORE, NULL, 0))
        return;
    app_log(app, "-> Sent DRONE_CFG_STORE (persist to flash)");

    KVSet updates = {0};
    for (int i = 0; i < DRONE_FIELDS; i++)
        kv_set(&updates, FIELDS[i].define, v[i] * FIELDS[i].scale);

    if (app_save_defaults(app, "drone", &updates, &app->drone_defaults))
        app_log(app, "Saved drone config tab as default");
    else
        app_log(app, "Failed to save drone config defaults");
}

static void on_save_confirmed(void *ctx, int button)
{
    if (button == 0)
        do_save(ctx);
}

/* Draws one card of label/field rows, taking fields from `first`. */
static void field_card(DroneTab *t, Rectangle r, const char *title, const int *idx, int n)
{
    ui_card(r, title);
    Rectangle b = ui_card_body(r, true);
    for (int i = 0; i < n; i++)
        ui_field_row((Rectangle){b.x, b.y + (float)i * UI_ROW_H, b.width, UI_ROW_H}, FIELDS[idx[i]].label,
                     &t->f[idx[i]]);
}

void drone_tab_draw(App *app, Rectangle area)
{
    DroneTab *t = &app->dtab;

    static const int LIMITS[] = {D_MAX_RATE, D_MAX_ANGLE, D_FAIL_ANGLE};
    static const int THROTTLE[] = {D_MIN_THROTTLE, D_MAX_THROTTLE};
    static const int SAFETY[] = {D_COMS_TIMEOUT_S, D_LOW_VOLTAGE, D_CRITICAL_VOLTAGE};

    float card_h = UI_CARD_HEAD + 3 * UI_ROW_H + UI_PAD;
    Rectangle row = {area.x, area.y, area.width, card_h};
    field_card(t, ui_col(row, 3, 0, UI_GAP), "Flight limits", LIMITS, 3);
    field_card(t, ui_col(row, 3, 1, UI_GAP), "Throttle range", THROTTLE, 2);
    field_card(t, ui_col(row, 3, 2, UI_GAP), "Failsafe & battery", SAFETY, 3);

    /* Actions */
    Rectangle bar = {area.x, area.y + card_h + UI_GAP, area.width, 84};
    ui_card(bar, NULL);
    ui_text("Send applies these values to the drone's RAM straight away.", bar.x + UI_PAD, bar.y + 18,
            UI_TEXT_SIZE, UI_TEXT);
    ui_text("Save also writes them to flash and keeps them as your local defaults.", bar.x + UI_PAD,
            bar.y + 44, UI_TEXT_SIZE - 1, UI_MUTED);

    float bx = bar.x + bar.width - UI_PAD;
    Rectangle save_r = {bx - 210, bar.y + (bar.height - UI_BTN_H) / 2, 210, UI_BTN_H};
    Rectangle send_r = {save_r.x - 10 - 160, save_r.y, 160, UI_BTN_H};
    if (ui_button(send_r, "Send to drone", UI_ACCENT))
        send_config(t);
    if (ui_button(save_r, "Save to flash & default", UI_ORANGE) && app_require_connection(app))
        ui_modal_open("Save to Flash & Default",
                      "Persist the drone's currently-live config (including PID gains) to flash, and "
                      "save these values as your local working defaults?",
                      "Yes;No", on_save_confirmed, t);

    /* Live response next to the limits being tuned. */
    float py = bar.y + bar.height + UI_GAP;
    Rectangle below = {area.x, py, area.width, area.y + area.height - py};
    if (below.height >= 170) {
        plot_draw(&app->ttab.angles, ui_col(below, 2, 0, UI_GAP));
        plot_draw(&app->ttab.motors, ui_col(below, 2, 1, UI_GAP));
    }
}
