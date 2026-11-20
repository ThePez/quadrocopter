/* Session viewer: zoomable plots of a recorded CSV, one tab per signal group. */

#include <stdio.h>
#include <string.h>

#include "app.h"

typedef struct {
    const char *title;
    const char *ylabel;
    int n;
    const char *names[4];
    Color colors[4];
    int fields[4];
} Group;

#define RED_C    UI_SERIES_RED
#define GREEN_C  UI_SERIES_GREEN
#define BLUE_C   UI_SERIES_BLUE
#define PURPLE_C UI_SERIES_PURPLE
#define ORANGE_C UI_SERIES_ORANGE
#define CYAN_C   UI_SERIES_CYAN

/* Names and colours match the live plots on the Telemetry tab. */
static const Group GROUPS[SESSION_PLOTS] = {
    {"Angles", "Angle (\xC2\xB0)", 3, {"Pitch", "Roll", "Yaw"}, {RED_C, GREEN_C, BLUE_C},
     {F_PITCH_ANGLE, F_ROLL_ANGLE, F_YAW_ANGLE}},
    {"Rates", "Rate (\xC2\xB0/s)", 3, {"Pitch", "Roll", "Yaw"}, {RED_C, GREEN_C, BLUE_C},
     {F_PITCH_RATE, F_ROLL_RATE, F_YAW_RATE}},
    {"PID Outputs", "Output (\xC2\xB5s)", 3, {"Pitch", "Roll", "Yaw"}, {RED_C, GREEN_C, BLUE_C},
     {F_PITCH_PID, F_ROLL_PID, F_YAW_PID}},
    {"Motors", "Pulse width (\xC2\xB5s)", 4, {"Front|Left", "Front|Right", "Back|Left", "Back|Right"},
     {RED_C, PURPLE_C, CYAN_C, ORANGE_C}, {F_MOTOR_FL, F_MOTOR_FR, F_MOTOR_BL, F_MOTOR_BR}},
    {"Battery", "Voltage (mV)", 1, {"Battery"}, {{228, 232, 240, 255}}, {F_BATTERY}},
    {"Stick Commands", "Commanded rate (\xC2\xB0/s)", 3, {"Pitch", "Roll", "Yaw"},
     {RED_C, GREEN_C, BLUE_C}, {F_CMD_PITCH, F_CMD_ROLL, F_CMD_YAW}},
    {"Throttle Command", "Throttle (\xC2\xB5s)", 1, {"Throttle"}, {{228, 232, 240, 255}},
     {F_CMD_THROTTLE}},
    {"Loop Timing", "Worst gap between control cycles (\xC2\xB5s)", 1, {"Loop dt max"},
     {{228, 232, 240, 255}}, {F_LOOP_DT_MAX_US}},
};

void session_tab_init(SessionTab *t)
{
    memset(t, 0, sizeof *t);
}

void session_tab_close(SessionTab *t)
{
    if (!t->open)
        return;
    for (int i = 0; i < SESSION_PLOTS; i++)
        plot_free(&t->plot[i]);
    session_free(&t->session);
    t->open = false;
}

void session_tab_free(SessionTab *t)
{
    session_tab_close(t);
}

void session_tab_open(App *app, const char *path)
{
    SessionTab *t = &app->stab;

    Session loaded;
    char err[256];
    if (session_load(&loaded, path, err, sizeof err) != 0) {
        app_error("Open Session", "Could not open: %s", err);
        return;
    }

    session_tab_close(t);
    t->session = loaded;
    t->open = true;
    t->sub = 0;

    Session *s = &t->session;
    t->view.full0 = s->count ? s->t[0] : 0.0;
    t->view.full1 = s->count ? s->t[s->count - 1] : 1.0;
    if (t->view.full1 - t->view.full0 < 1e-3)
        t->view.full1 = t->view.full0 + 1.0;
    t->view.x0 = t->view.full0;
    t->view.x1 = t->view.full1;

    for (int i = 0; i < SESSION_PLOTS; i++) {
        const Group *g = &GROUPS[i];
        Plot *p = &t->plot[i];
        plot_init_static(p, g->title, g->ylabel, g->n, g->names, g->colors, &t->view);
        for (int k = 0; k < g->n; k++)
            plot_bind(p, k, s->t, s->col[g->fields[k]], s->count);
        p->events = s->events;
        p->nevents = s->nevents;
    }

    app->main_tab = TAB_SESSION;
    app_log(app, "Opened session %s (%d samples)", path, s->count);
}

void session_tab_draw(App *app, Rectangle area)
{
    SessionTab *t = &app->stab;
    const float toolbar_h = 40;

    /* Toolbar: signal groups on the left, view actions on the right. */
    ui_segmented((Rectangle){area.x, area.y, 0, toolbar_h}, "Angles;Rates;PID;Motors;Battery;Loop timing",
                 &t->sub);

    float right = area.x + area.width;
    Rectangle close_r = {right - 100, area.y, 100, toolbar_h};
    if (ui_btn(close_r, "Close")) {
        session_tab_close(t);
        app->main_tab = TAB_TELEMETRY;
        return;
    }
    Rectangle reset_r = {close_r.x - 10 - 120, area.y, 120, toolbar_h};
    if (ui_btn(reset_r, "Reset zoom")) {
        t->view.x0 = t->view.full0;
        t->view.x1 = t->view.full1;
    }

    char summary[700];
    snprintf(summary, sizeof summary, "%s   |   %d samples   |   %.1f s   |   %d events",
             GetFileName(t->session.path), t->session.count, t->session.duration, t->session.nevents);
    ui_text(summary, area.x + 2, area.y + toolbar_h + 12, UI_TEXT_SIZE - 1, UI_MUTED);
    ui_text_right("scroll = zoom   drag = pan   double-click = reset", area.x + area.width,
                  area.y + toolbar_h + 12, UI_SMALL, UI_MUTED);

    float top = area.y + toolbar_h + 40;
    plot_draw(&t->plot[t->sub], (Rectangle){area.x, top, area.width, area.y + area.height - top});
}
