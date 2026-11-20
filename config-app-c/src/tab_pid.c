/* PID tuning tab: three gain groups (Rate P/R, Rate Yaw, Angle P/R). */

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
    if (alpha >= 1.0)
        return -1.0;
    return alpha / (1.0 - alpha) * LOOP_RATE_HZ / (2.0 * M_PI);
}

static void d_filter_text(double alpha, char *out, size_t cap)
{
    double hz = d_alpha_to_hz(alpha);
    if (hz < 0)
        snprintf(out, cap, "unfiltered");
    else
        snprintf(out, cap, "~ %.0f Hz cutoff", hz);
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */
/* ------------------------------------------------------------------------ */

static void init_group(PidGroup *g, App *app, int axis, int mode, const char *title,
                       const char *prefix)
{
    memset(g, 0, sizeof *g);
    g->app = app;
    g->axis = axis;
    g->mode = mode;
    g->title = title;
    g->defaults_prefix = prefix;

    char key[64];
    snprintf(key, sizeof key, "%s_KP", prefix);
    numfield_init(&g->kp, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof key, "%s_KI", prefix);
    numfield_init(&g->ki, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof key, "%s_KD", prefix);
    numfield_init(&g->kd, kv_get(&app->pid_defaults, key, 0.0), 0.001, 0, 100, 4);
    snprintf(key, sizeof key, "%s_KD_ALPHA", prefix);
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

/* Sends the group's gains live (RAM only). */
static bool send_pid(PidGroup *g)
{
    App *app = g->app;
    if (!app_require_connection(app))
        return false;
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
    if (!app_send(app, PID_CONFIG, &p, sizeof p))
        return false;

    g->sent_valid = true;
    g->sent_kp = g->kp.value;
    g->sent_ki = g->ki.value;
    g->sent_kd = g->kd.value;
    g->sent_alpha = g->alpha.value;

    snprintf(g->current, sizeof g->current, "Kp %.4g   Ki %.4g   Kd %.4g   alpha %.2g", g->kp.value,
             g->ki.value, g->kd.value, g->alpha.value);
    g->current_set = true;

    char hint[48];
    d_filter_text(g->alpha.value, hint, sizeof hint);
    app_log(app, "-> Sent %s %s: Kp=%.4f, Ki=%.4f, Kd=%.4f, D filter alpha=%.2f (%s)",
            AXIS_NAMES[g->axis], MODE_NAMES[g->mode], g->kp.value, g->ki.value, g->kd.value,
            g->alpha.value, hint);

    char event[160];
    snprintf(event, sizeof event, "%s %s: Kp=%.4f Ki=%.4f Kd=%.4f a=%.2f", AXIS_NAMES[g->axis],
             MODE_NAMES[g->mode], g->kp.value, g->ki.value, g->kd.value, g->alpha.value);
    app_record_event(app, event);
    return true;
}

static bool values_match_sent(const PidGroup *g)
{
    return g->sent_valid && g->sent_kp == g->kp.value && g->sent_ki == g->ki.value &&
           g->sent_kd == g->kd.value && g->sent_alpha == g->alpha.value;
}

/* The drone keeps PID gains inside its own config, so "save" means: make sure
 * the live gains are what's on screen, tell the drone to persist its config,
 * and remember the gains as this GUI's working defaults. */
static void do_save(PidGroup *g)
{
    App *app = g->app;

    if (!values_match_sent(g) && !send_pid(g))
        return;

    if (!app_send(app, DRONE_CFG_STORE, NULL, 0))
        return;
    app_log(app, "-> Sent DRONE_CFG_STORE (persist to flash)");

    char key[64];
    KVSet updates = {0};
    snprintf(key, sizeof key, "%s_KP", g->defaults_prefix);
    kv_set(&updates, key, g->kp.value);
    snprintf(key, sizeof key, "%s_KI", g->defaults_prefix);
    kv_set(&updates, key, g->ki.value);
    snprintf(key, sizeof key, "%s_KD", g->defaults_prefix);
    kv_set(&updates, key, g->kd.value);
    snprintf(key, sizeof key, "%s_KD_ALPHA", g->defaults_prefix);
    kv_set(&updates, key, g->alpha.value);

    if (app_save_defaults(app, "pid", &updates, &app->pid_defaults))
        app_log(app, "Saved %s %s as default: Kp=%.4f, Ki=%.4f, Kd=%.4f", AXIS_NAMES[g->axis],
                g->title, g->kp.value, g->ki.value, g->kd.value);
    else
        app_log(app, "Failed to save %s %s defaults", AXIS_NAMES[g->axis], g->title);
}

static void on_save_confirmed(void *ctx, int button)
{
    if (button == 0)
        do_save(ctx);
}

static void request_save(PidGroup *g)
{
    if (!app_require_connection(g->app))
        return;

    char msg[512];
    snprintf(msg, sizeof msg,
             "Persist the drone's currently-live config to flash, and save the %s %s gains as "
             "your local working default:\nKp=%.4f, Ki=%.4f, Kd=%.4f, D filter alpha=%.2f?",
             AXIS_NAMES[g->axis], g->title, g->kp.value, g->ki.value, g->kd.value, g->alpha.value);
    ui_modal_open("Save to Flash & Default", msg, "Yes;No", on_save_confirmed, g);
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
    if (g->current_set) {
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
    d_filter_text(g->alpha.value, hint, sizeof hint);
    ui_text_right(hint, b.x + b.width, y, UI_SMALL, UI_MUTED);
    if (g->axis == 1)
        ui_text("Rate mode only", b.x, y, UI_SMALL, UI_MUTED);

    float by = b.y + b.height - UI_BTN_H;
    if (ui_button((Rectangle){b.x, by, b.width, UI_BTN_H}, "Save to flash & default", UI_ORANGE))
        request_save(g);
    if (ui_button((Rectangle){b.x, by - UI_BTN_H - 8, b.width, UI_BTN_H}, "Send to drone", UI_ACCENT))
        send_pid(g);
}

void pid_tab_draw(App *app, Rectangle area)
{
    PidTab *t = &app->ptab;

    float cards_h = fminf(PID_CARD_H, area.height);
    Rectangle row = {area.x, area.y, area.width, cards_h};
    for (int i = 0; i < 3; i++)
        draw_group(&t->group[i], CARD_TITLES[i], ui_col(row, 3, i, UI_GAP));

    /* The live response, so gains can be judged without leaving the page. */
    float py = area.y + cards_h + UI_GAP;
    Rectangle below = {area.x, py, area.width, area.y + area.height - py};
    if (below.height >= 170) {
        plot_draw(&app->ttab.angles, ui_col(below, 2, 0, UI_GAP));
        plot_draw(&app->ttab.rates, ui_col(below, 2, 1, UI_GAP));
    }
}
