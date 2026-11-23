/*
 *****************************************************************************
 * File: tab_telemetry.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Telemetry tab: live plots, 3D view and live values.
 *****************************************************************************
 */

/*
 * Telemetry page: all four live plots at once (or one big one), with the 3D
 * attitude view and the current values in a column on the right.
 *
 * telemetry_tab_push() is the entry point for data: app.c calls it for every sample, and it fans
 * the values out to the five live plots (angles, rates, PID outputs, motors, controls) and the 3D
 * view. The Controls plot converts raw commands to a percentage of stick travel using the limits
 * from the Drone Config tab (StickScale), so changing those limits changes how it is scaled.
 * The PID and Drone tabs also draw some of these plots (plot_draw on app->ttab.*).
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"

#define LIVE_POINTS 1000

static const char *const AXIS_NAMES[3] = {"Pitch", "Roll", "Yaw"};
static const Color AXIS_COLORS[3] = {UI_SERIES_RED, UI_SERIES_GREEN, UI_SERIES_BLUE};

/* "Group|Leaf": the legend prints Front / Back once, then Left / Right. */
static const char *const MOTOR_NAMES[4] = {"Front|Left", "Front|Right", "Back|Left", "Back|Right"};
/* FL, FR, BL, BR. The 3D view uses the same colours per motor. */
static const Color MOTOR_COLORS[4] = {UI_SERIES_RED, UI_SERIES_PURPLE, UI_SERIES_CYAN,
                                      UI_SERIES_ORANGE};

/* The pilot's five control signals on one axis, as a percentage of stick travel:
 * throttle 0..100, pitch/roll/yaw -100..100, and the flight mode as 0 (ACRO) or 100 (STABILISE). */
static const char *const CONTROL_NAMES[5] = {"Throttle", "Pitch", "Roll", "Yaw", "Mode"};
static const Color CONTROL_COLORS[5] = {UI_SERIES_ORANGE, UI_SERIES_RED, UI_SERIES_GREEN,
                                        UI_SERIES_BLUE, UI_SERIES_PURPLE};

void telemetry_tab_init(TelemetryTab *t)
{
    memset(t, 0, sizeof(*t));
    plot_init_live(&t->angles, "Attitude Angles", "\xC2\xB0", 3, AXIS_NAMES, AXIS_COLORS,
                   LIVE_POINTS);
    plot_init_live(&t->rates, "Angular Rates", "\xC2\xB0/s", 3, AXIS_NAMES, AXIS_COLORS,
                   LIVE_POINTS);
    plot_init_live(&t->pid, "PID Outputs", "\xC2\xB5s", 3, AXIS_NAMES, AXIS_COLORS, LIVE_POINTS);
    plot_init_live(&t->motors, "Motors", "pulse \xC2\xB5s", 4, MOTOR_NAMES, MOTOR_COLORS,
                   LIVE_POINTS);
    plot_init_live(&t->controls, "Controls", "% of stick (mode: 100 = STABILISE)", 5, CONTROL_NAMES,
                   CONTROL_COLORS, LIVE_POINTS);
    att_init(&t->att);
}

void telemetry_tab_free(TelemetryTab *t)
{
    plot_free(&t->angles);
    plot_free(&t->rates);
    plot_free(&t->pid);
    plot_free(&t->motors);
    plot_free(&t->controls);
    att_free(&t->att);
}

static float percent(float value, float lo, float hi)
{
    return hi > lo ? (value - lo) / (hi - lo) * 100.0f : 0.0f;
}

void telemetry_tab_push(TelemetryTab *t, const Sample *s, StickScale scale)
{
    const float *v = s->v;
    float angles[3] = {v[F_PITCH_ANGLE], v[F_ROLL_ANGLE], v[F_YAW_ANGLE]};
    float rates[3] = {v[F_PITCH_RATE], v[F_ROLL_RATE], v[F_YAW_RATE]};
    float pid[3] = {v[F_PITCH_PID], v[F_ROLL_PID], v[F_YAW_PID]};
    float motors[4] = {v[F_MOTOR_FL], v[F_MOTOR_FR], v[F_MOTOR_BL], v[F_MOTOR_BR]};
    float controls[5] = {percent(v[F_CMD_THROTTLE], scale.min_throttle, scale.max_throttle),
                         percent(v[F_CMD_PITCH], -scale.max_rate, scale.max_rate) * 2.0f - 100.0f,
                         percent(v[F_CMD_ROLL], -scale.max_rate, scale.max_rate) * 2.0f - 100.0f,
                         percent(v[F_CMD_YAW], -scale.max_rate, scale.max_rate) * 2.0f - 100.0f,
                         v[F_MODE] != 0.0f ? 100.0f : 0.0f};

    plot_push(&t->angles, s->t, angles);
    plot_push(&t->rates, s->t, rates);
    plot_push(&t->pid, s->t, pid);
    plot_push(&t->motors, s->t, motors);
    plot_push(&t->controls, s->t, controls);
    att_set(&t->att, v[F_PITCH_ANGLE], v[F_ROLL_ANGLE], v[F_YAW_ANGLE]);
}

static void on_session_picked(void *ctx, const char *path)
{
    if (path) {
        session_tab_open(ctx, path);
    }
}

/* ------------------------------------------------------------------------ */
/* Live values                                                               */
/* ------------------------------------------------------------------------ */

typedef struct {
    const char *label;
    int field;
    const char *unit;
} ValueRow;

static const ValueRow VALUES[] = {
    {"Pitch", F_PITCH_ANGLE, "\xC2\xB0"},
    {"Roll", F_ROLL_ANGLE, "\xC2\xB0"},
    {"Yaw", F_YAW_ANGLE, "\xC2\xB0"},
    {"Pitch rate", F_PITCH_RATE, " \xC2\xB0/s"},
    {"Roll rate", F_ROLL_RATE, " \xC2\xB0/s"},
    {"Yaw rate", F_YAW_RATE, " \xC2\xB0/s"},
    {"Mode", F_MODE, ""},
    {"Battery", F_BATTERY, " V"},
    {"Loop dt max", F_LOOP_DT_MAX_US, " \xC2\xB5s"},
    /* What the pilot is commanding, as the drone received it */
    {"Cmd pitch", F_CMD_PITCH, " \xC2\xB0/s"},
    {"Cmd roll", F_CMD_ROLL, " \xC2\xB0/s"},
    {"Cmd yaw", F_CMD_YAW, " \xC2\xB0/s"},
    {"Throttle cmd", F_CMD_THROTTLE, " \xC2\xB5s"},
};
#define N_VALUES ((int)(sizeof(VALUES) / sizeof(VALUES[0])))

static Color format_value(const App *app, const ValueRow *row, char *out, size_t cap)
{
    if (!app->have_last) {
        snprintf(out, cap, "--");
        return UI_MUTED;
    }

    float v = app->last.v[row->field];
    switch (row->field) {
    case F_MODE:
        snprintf(out, cap, "%s", v != 0.0f ? "STABILISE" : "ACRO");
        return v != 0.0f ? UI_ACCENT : UI_ORANGE;
    case F_LOOP_DT_MAX_US:
    case F_CMD_THROTTLE:
        snprintf(out, cap, "%.0f%s", (double)v, row->unit);
        break;
    case F_BATTERY: /* mV -> V */
        snprintf(out, cap, "%.2f%s", (double)v / 1000.0, row->unit);
        break;
    default:
        snprintf(out, cap, "%.2f%s", (double)v, row->unit);
        break;
    }
    return UI_TEXT;
}

static void draw_values_card(App *app, Rectangle r)
{
    ui_card(r, "Live values");

    const char *status = "No telemetry";
    Color color = UI_MUTED;
    if (app->status == TS_RECEIVING) {
        status = "Receiving";
        color = UI_GREEN;
    } else if (app->status == TS_TIMEOUT) {
        status = "Timeout";
        color = UI_RED;
    }

    float right = r.x + r.width - UI_PAD;
    ui_text_right(status, right, r.y + 14, UI_SMALL + 1, color);
    ui_dot(right - ui_text_width(status, UI_SMALL + 1) - 10, r.y + 22, 4, color);

    float row_h = (r.height - UI_CARD_HEAD - 10) / (float)N_VALUES;
    for (int i = 0; i < N_VALUES; i++) {
        float y = r.y + UI_CARD_HEAD + (float)i * row_h;
        if (i > 0) {
            DrawLine((int)(r.x + UI_PAD), (int)y, (int)(r.x + r.width - UI_PAD), (int)y, UI_GRID);
        }

        char text[48];
        Color c = format_value(app, &VALUES[i], text, sizeof(text));
        float ty = y + (row_h - UI_TEXT_SIZE) / 2 - 1;
        ui_text(VALUES[i].label, r.x + UI_PAD, ty, UI_TEXT_SIZE, UI_MUTED);
        ui_text_right(text, right, ty, UI_TEXT_SIZE, c);
    }
}

/* ------------------------------------------------------------------------ */
/* Page                                                                      */
/* ------------------------------------------------------------------------ */

static float btn_w(const char *text)
{
    return ui_text_width(text, UI_TEXT_SIZE) + 32;
}

void telemetry_tab_draw(App *app, Rectangle area)
{
    TelemetryTab *t = &app->ttab;
    const float toolbar_h = 40;

    /* ---- toolbar ---- */
    ui_segmented((Rectangle){area.x, area.y, 0, toolbar_h},
                 "Overview;Angles;Rates;PID;Motors;Controls", &t->sub);

    float right = area.x + area.width;
    bool recording = recorder_active(&app->rec);
    const char *rec_text = recording ? "Stop recording" : "Record";
    Rectangle rec_r = {right - btn_w(rec_text) - 8, area.y, btn_w(rec_text) + 8, toolbar_h};
    if (recording) {
        if (ui_button(rec_r, rec_text, UI_RED)) {
            app_toggle_recording(app);
        }
    } else if (ui_btn(rec_r, rec_text)) {
        app_toggle_recording(app);
    }

    right = rec_r.x - 10;

    Rectangle open_r = {right - btn_w("Open session"), area.y, btn_w("Open session"), toolbar_h};
    if (ui_btn(open_r, "Open session")) {
        ui_file_picker_open("Open Session", SESSIONS_DIR, ".csv", on_session_picked, app);
    }

    right = open_r.x - 10;

    Rectangle clear_r = {right - btn_w("Clear"), area.y, btn_w("Clear"), toolbar_h};
    if (ui_btn(clear_r, "Clear")) {
        plot_clear(&t->angles);
        plot_clear(&t->rates);
        plot_clear(&t->pid);
        plot_clear(&t->motors);
        plot_clear(&t->controls);
    }

    right = clear_r.x - 16;

    Rectangle live_r = {right - 112, area.y, 112, toolbar_h};
    ui_checkbox(live_r, "Live plots", &app->live_plots);

    /* ---- body ---- */
    Rectangle body = {area.x, area.y + toolbar_h + UI_GAP, area.width,
                      area.height - toolbar_h - UI_GAP};
    float side_w = body.width * 0.27f;
    if (side_w < 270) {
        side_w = 270;
    }

    if (side_w > 340) {
        side_w = 340;
    }

    Rectangle plots = {body.x, body.y, body.width - side_w - UI_GAP, body.height};
    Rectangle side = {plots.x + plots.width + UI_GAP, body.y, side_w, body.height};

    Plot *all[5] = {&t->angles, &t->rates, &t->pid, &t->motors, &t->controls};
    if (t->sub == 0) {
        /* Overview: angles, rates, PID and motors in a 2x2 grid (Controls has its own tab) */
        for (int i = 0; i < 4; i++) {
            Rectangle cell = ui_col(ui_row(plots, 2, i / 2, UI_GAP), 2, i % 2, UI_GAP);
            plot_draw(all[i], cell);
        }
    } else {
        plot_draw(all[t->sub - 1], plots);
    }

    float values_h = UI_CARD_HEAD + 9 * 29 + 10;
    Rectangle att_r = {side.x, side.y, side.width, side.height - values_h - UI_GAP};
    Rectangle values_r = {side.x, att_r.y + att_r.height + UI_GAP, side.width, values_h};
    att_draw(&t->att, att_r, "3D attitude");
    draw_values_card(app, values_r);
}
