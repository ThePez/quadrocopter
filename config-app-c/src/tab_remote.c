/* Remote config tab: battery/voltage calibration and joystick calibration. */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"

enum { V_CAL_MULT, V_LOW, V_CRITICAL };

static const char *const VOLTAGE_LABELS[3] = {"Voltage cal multiplier", "Low voltage (mV)",
                                              "Critical voltage (mV)"};
static const char *const VOLTAGE_DEFINES[3] = {"VOLTAGE_CAL_MULTIPLIER", "LOW_VOLTAGE",
                                               "CRITICAL_VOLTAGE"};

static const char *const CHANNELS[4] = {"throttle", "pitch", "roll", "yaw"};
static const char *const CHANNEL_TITLES[4] = {"Throttle calibration", "Pitch calibration",
                                              "Roll calibration", "Yaw calibration"};
static const char *const CAL_LABELS[3] = {"Min", "Centre", "Max"};
static const char *const CAL_SUFFIX[3] = {"MIN", "CENTRE", "MAX"};

/* "JOYSTICK_THROTTLE_MIN" */
static void joystick_define(char *out, size_t cap, int channel, int which)
{
    char up[16];
    size_t i = 0;
    for (; CHANNELS[channel][i] && i + 1 < sizeof up; i++)
        up[i] = (char)(CHANNELS[channel][i] - 'a' + 'A');
    up[i] = '\0';
    snprintf(out, cap, "JOYSTICK_%s_%s", up, CAL_SUFFIX[which]);
}

void remote_tab_init(RemoteTab *t, App *app)
{
    memset(t, 0, sizeof *t);
    t->app = app;
    const KVSet *d = &app->remote_defaults;

    numfield_init(&t->voltage[V_CAL_MULT], kv_get(d, VOLTAGE_DEFINES[V_CAL_MULT], 1.0), 1.0, 0,
                  100000, 3);
    numfield_init(&t->voltage[V_LOW], kv_get(d, VOLTAGE_DEFINES[V_LOW], 0.0), 1.0, 0, 100000, 3);
    numfield_init(&t->voltage[V_CRITICAL], kv_get(d, VOLTAGE_DEFINES[V_CRITICAL], 0.0), 1.0, 0,
                  100000, 3);

    /* A per-channel value falls back to the shared JOYSTICK_CAL_* default
     * until Save to Flash has stored an override for that channel. */
    const double shared[3] = {kv_get(d, "JOYSTICK_CAL_MIN", 0.0),
                              kv_get(d, "JOYSTICK_CAL_CENTRE", 2048.0),
                              kv_get(d, "JOYSTICK_CAL_MAX", 4095.0)};
    for (int c = 0; c < 4; c++) {
        for (int w = 0; w < 3; w++) {
            char key[48];
            joystick_define(key, sizeof key, c, w);
            numfield_init(&t->joystick[c][w], kv_get(d, key, shared[w]), 1.0, 0, 4095, 0);
        }
    }
}

static void snapshot(const RemoteTab *t, double v[3], double j[4][3])
{
    for (int i = 0; i < 3; i++)
        v[i] = t->voltage[i].value;
    for (int c = 0; c < 4; c++)
        for (int w = 0; w < 3; w++)
            j[c][w] = t->joystick[c][w].value;
}

static uint16_t to_u16(double v)
{
    if (v < 0)
        return 0;
    if (v > 65535)
        return 65535;
    return (uint16_t)llround(v);
}

static struct joystick_cal_telemetry_t cal(const double j[3])
{
    return (struct joystick_cal_telemetry_t){to_u16(j[0]), to_u16(j[1]), to_u16(j[2])};
}

/* What a request hands to its completion callback: a copy, since the fields may have changed
 * by the time the ack arrives. */
typedef struct {
    double v[3];
    double j[4][3];
    bool then_save; /* follow with the flash save once this update is confirmed */
} RemoteSnap;

static void on_store_done(App *app, const void *user, bool ok)
{
    const RemoteSnap *s = user;
    if (!ok) {
        app_log(app, "Remote config: not saved as default (the remote did not confirm the flash save)");
        return;
    }

    /* The remote has it in flash: now remember the values as this GUI's working defaults. */
    KVSet updates = {0};
    for (int i = 0; i < 3; i++)
        kv_set(&updates, VOLTAGE_DEFINES[i], s->v[i]);
    for (int c = 0; c < 4; c++)
        for (int w = 0; w < 3; w++) {
            char key[48];
            joystick_define(key, sizeof key, c, w);
            kv_set(&updates, key, s->j[c][w]);
        }

    if (app_save_defaults(app, "remote", &updates, &app->remote_defaults))
        app_log(app, "Saved remote config tab as default");
    else
        app_log(app, "Failed to save remote config defaults");
}

static bool send_store(App *app, const RemoteSnap *s)
{
    if (!app_send_then(app, REMOTE_CFG_STORE, NULL, 0, on_store_done, s, sizeof *s))
        return false;
    app_log(app, "-> Sent REMOTE_CFG_STORE (persist to flash)");
    return true;
}

static void on_config_done(App *app, const void *user, bool ok)
{
    RemoteSnap s = *(const RemoteSnap *)user; /* a copy: send_store() below may start another request */
    RemoteTab *t = &app->rtab;

    if (!ok) {
        if (s.then_save)
            app_log(app, "Remote config: save cancelled, the update was not applied");
        return;
    }

    /* Confirmed: only now is this what the remote is running */
    memcpy(t->sent_voltage, s.v, sizeof s.v);
    memcpy(t->sent_joystick, s.j, sizeof s.j);
    t->sent_valid = true;

    if (s.then_save)
        send_store(app, &s);
}

/* Sends the config live (RAM only). It only counts as sent once the remote confirms. */
static bool send_config(RemoteTab *t, bool then_save)
{
    App *app = t->app;
    if (!app_require_connection(app))
        return false;

    double v[3], j[4][3];
    snapshot(t, v, j);

    struct remote_config_telemetry_t p = {
        .voltage_cal_multiplier = (float)v[V_CAL_MULT],
        .low_voltage = to_u16(v[V_LOW]),
        .critical_voltage = to_u16(v[V_CRITICAL]),
        .throttle = cal(j[0]),
        .pitch = cal(j[1]),
        .roll = cal(j[2]),
        .yaw = cal(j[3]),
    };
    RemoteSnap snap = {.then_save = then_save};
    memcpy(snap.v, v, sizeof v);
    memcpy(snap.j, j, sizeof j);
    if (!app_send_then(app, REMOTE_CFG, &p, sizeof p, on_config_done, &snap, sizeof snap))
        return false;

    app_log(app, "-> Sent remote config update (live, not yet saved), waiting for confirmation");
    return true;
}

/* Saving writes flash on the remote: the update goes first if the values changed, the save follows
 * once the remote confirms it, and the values become this GUI's defaults once the save is confirmed. */
static void do_save(RemoteTab *t)
{
    App *app = t->app;
    double v[3], j[4][3];
    snapshot(t, v, j);

    bool same = t->sent_valid && memcmp(t->sent_voltage, v, sizeof v) == 0 &&
                memcmp(t->sent_joystick, j, sizeof j) == 0;
    if (!same) {
        send_config(t, true);
        return;
    }

    RemoteSnap snap = {0};
    memcpy(snap.v, v, sizeof v);
    memcpy(snap.j, j, sizeof j);
    send_store(app, &snap);
}

static void on_save_confirmed(void *ctx, int button)
{
    if (button == 0)
        do_save(ctx);
}

void remote_tab_draw(App *app, Rectangle area)
{
    RemoteTab *t = &app->rtab;

    /* Two rows of three cards: battery + throttle + pitch, info + roll + yaw. */
    float card_h = UI_CARD_HEAD + 3 * UI_ROW_H + UI_PAD;

    Rectangle row0 = {area.x, area.y, area.width, card_h};
    Rectangle row1 = {area.x, area.y + card_h + UI_GAP, area.width, card_h};

    /* Battery */
    Rectangle bat = ui_col(row0, 3, 0, UI_GAP);
    ui_card(bat, "Battery / voltage");
    Rectangle bb = ui_card_body(bat, true);
    for (int i = 0; i < 3; i++)
        ui_field_row((Rectangle){bb.x, bb.y + (float)i * UI_ROW_H, bb.width, UI_ROW_H}, VOLTAGE_LABELS[i],
                     &t->voltage[i]);

    /* Joysticks */
    Rectangle slots[4] = {ui_col(row0, 3, 1, UI_GAP), ui_col(row0, 3, 2, UI_GAP), ui_col(row1, 3, 1, UI_GAP),
                          ui_col(row1, 3, 2, UI_GAP)};
    for (int c = 0; c < 4; c++) {
        ui_card(slots[c], CHANNEL_TITLES[c]);
        Rectangle b = ui_card_body(slots[c], true);
        for (int w = 0; w < 3; w++)
            ui_field_row((Rectangle){b.x, b.y + (float)w * UI_ROW_H, b.width, UI_ROW_H}, CAL_LABELS[w],
                         &t->joystick[c][w]);
    }

    /* Info + actions */
    Rectangle info = ui_col(row1, 3, 0, UI_GAP);
    ui_card(info, "Apply changes");
    Rectangle ib = ui_card_body(info, true);
    ui_text("Send updates the remote's RAM.", ib.x, ib.y + 2, UI_TEXT_SIZE - 1, UI_MUTED);
    ui_text("Save also writes to flash and", ib.x, ib.y + 22, UI_TEXT_SIZE - 1, UI_MUTED);
    ui_text("keeps your local defaults.", ib.x, ib.y + 42, UI_TEXT_SIZE - 1, UI_MUTED);

    float half = (ib.width - 8) / 2;
    float by = ib.y + ib.height - UI_BTN_H - 2;
    if (ui_button((Rectangle){ib.x, by, half, UI_BTN_H}, "Send", UI_ACCENT))
        send_config(t, false);
    if (ui_button((Rectangle){ib.x + half + 8, by, half, UI_BTN_H}, "Save", UI_ORANGE) &&
        app_require_connection(app))
        ui_modal_open("Save to Flash & Default",
                      "Persist the remote's currently-live config to flash, and save these values as "
                      "your local working defaults?",
                      "Yes;No", on_save_confirmed, t);
}
