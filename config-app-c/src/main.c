/*
 *****************************************************************************
 * File: main.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Window loop, sidebar and page dispatch.
 *****************************************************************************
 */

/*
 * Drone config / PID tuning GUI (raylib + raygui port of config-app/).
 *
 * main.c owns the window and the sidebar (connection + navigation); each page
 * lives in its tab_*.c file and fills the content area to the right.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "util.h"

#define SIDEBAR_W 204

/* Layout of the window:
 *
 *   +---------+----------------------------------------------+
 *   | sidebar |  page header (title + subtitle)              |
 *   |  card   |  tab content (one tab_*_draw() fills this)   |
 *   |  nav    |                                              |
 *   |  status |                                              |
 *   +---------+----------------------------------------------+
 *
 * Draw order each frame: sidebar, page, then the two port/baud dropdown lists and any modal
 * dialog, so open lists and dialogs end up on top. */

static const char *const NAV_LABELS[] = {"Telemetry", "PID Tuning", "Drone Config", "Remote Config",
                                         "Console"};

typedef struct {
    Rectangle port, baud;
} DropdownRects;

/* Connection card, tab navigation, the config-ack status line and the recording badge. The two
 * dropdowns are only *positioned* here (returned in `dd`); they are drawn later, on top. */
static void draw_sidebar(App *app, float height, DropdownRects *dd)
{
    DrawRectangle(0, 0, SIDEBAR_W, (int)height, UI_SIDEBAR);
    DrawLine(SIDEBAR_W, 0, SIDEBAR_W, (int)height, UI_BORDER);

    ui_text_bold("PID Tuner", 22, 20, 22, UI_TEXT);
    ui_text("Drone configurator", 22, 48, UI_SMALL, UI_MUTED);

    /* ---- Connection card ---- */
    Rectangle card = {12, 78, SIDEBAR_W - 24, 228};
    ui_card(card, NULL);
    float x = card.x + 12, w = card.width - 24, y = card.y + 14;

    ui_dot(x + 5, y + 9, 5, app->connected ? UI_GREEN : UI_MUTED);
    ui_text_bold(app->connected ? "Connected" : "Disconnected", x + 18, y, UI_TEXT_SIZE,
                 app->connected ? UI_GREEN : UI_TEXT);

    y += 34;
    ui_text("PORT", x, y, UI_SMALL - 1, UI_MUTED);
    if (ui_link((Rectangle){x + w - 40, y - 4, 40, 20}, "Scan")) {
        app_refresh_ports(app);
    }

    dd->port = (Rectangle){x, y + 18, w, 32};

    y += 62;
    ui_text("BAUD RATE", x, y, UI_SMALL - 1, UI_MUTED);
    dd->baud = (Rectangle){x, y + 18, w, 32};

    y += 62;
    Rectangle connect = {x, y, w, UI_BTN_H};
    if (app->connected) {
        if (ui_button(connect, "Disconnect", UI_RED)) {
            app_disconnect(app);
        }
    } else if (ui_button(connect, "Connect", UI_ACCENT)) {
        app_connect(app);
    }

    /* ---- Navigation ---- */
    float ny = card.y + card.height + 22;
    int count = TAB_COUNT_VISIBLE(app);
    for (int i = 0; i < count; i++) {
        char label[96];
        if (i == TAB_SESSION) {
            snprintf(label, sizeof(label), "Session");
        } else {
            str_copy(label, sizeof(label), NAV_LABELS[i]);
        }

        if (ui_nav_item((Rectangle){12, ny, SIDEBAR_W - 24, 42}, label, app->main_tab == i)) {
            ui_commit_edit();
            app->main_tab = i;
        }

        ny += 46;
    }

    /* ---- Config requests: unconfirmed while waiting for the ack, then the result ---- */
    const AckChannel *waiting = NULL;
    double newest = -1.0;
    for (int ch = 0; ch < ACK_CHANNELS; ch++) {
        if (app->ack[ch].in_flight && app->ack[ch].sent_at > newest) {
            newest = app->ack[ch].sent_at;
            waiting = &app->ack[ch];
        }
    }

    for (int ch = 0; ch < ACK_CHANNELS && !waiting; ch++) {
        if (app->ack[ch].qcount > 0) {
            waiting = &app->ack[ch];
        }
    }

    const char *what_src = NULL;
    char result[96] = "";
    Color c = UI_MUTED;
    if (waiting) {
        const AckRequest *r = waiting->in_flight ? &waiting->cur : &waiting->queue[waiting->qhead];
        what_src = r->what;
        if (waiting->in_flight && r->attempts > 1) {
            snprintf(result, sizeof(result), "unconfirmed (try %d/%d)", r->attempts,
                     r->max_attempts);
        } else {
            snprintf(result, sizeof(result), "unconfirmed");
        }

        c = UI_ORANGE;
    } else if (app->cfg_t > 0.0 && now_s() - app->cfg_t < 12.0) {
        what_src = app->cfg_what; /* the latest result stays for a few seconds */
        str_copy(result, sizeof(result), app->cfg_result);
        c = app->cfg_ok ? UI_GREEN : UI_RED;
    }

    if (what_src) {
        char what[64];
        str_copy(what, sizeof(what), what_src);
        /* Keep both lines inside the sidebar */
        for (size_t n = strlen(what);
             n > 3 && ui_text_width(what, UI_SMALL + 1) > SIDEBAR_W - 56;) {
            what[--n] = '\0';
        }

        for (size_t n = strlen(result);
             n > 3 && ui_text_width(result, UI_SMALL) > SIDEBAR_W - 56;) {
            result[--n] = '\0';
        }
        ui_dot(26, height - 83, 5, c);
        ui_text(what, 40, height - 91, UI_SMALL + 1, UI_TEXT);
        ui_text(result, 40, height - 72, UI_SMALL, c);
    }

    /* ---- Recording badge ---- */
    if (recorder_active(&app->rec)) {
        char text[64];
        snprintf(text, sizeof(text), "Recording  %d", app->rec.rows);
        ui_dot(26, height - 30, 5, UI_RED);
        ui_text(text, 40, height - 38, UI_SMALL + 1, UI_TEXT);
    }
}

/* Dropdowns are drawn after everything else so their open lists sit on top. */
static void draw_dropdowns(App *app, const DropdownRects *dd)
{
    char items[UART_MAX_PORTS * 48];
    items[0] = '\0';
    if (app->nports == 0) {
        snprintf(items, sizeof(items), "No ports found");
    } else {
        size_t len = 0;
        for (int i = 0; i < app->nports && len + 1 < sizeof(items); i++) {
            const char *dev = app->ports[i].device;
            if (strncmp(dev, "/dev/", 5) == 0) {
                dev += 5;
            }

            int n = snprintf(items + len, sizeof(items) - len, "%s%s", i ? ";" : "", dev);
            if (n < 0) {
                break;
            }

            len += (size_t)n;
        }
    }

    bool modal = ui_modal_active();

    /* Only one list may take input at a time; the port can't change mid-session. */
    if (modal || app->baud_edit || app->connected) {
        GuiLock();
    }

    if (GuiDropdownBox(dd->port, items, &app->port_active, app->port_edit)) {
        app->port_edit = !app->port_edit;
    }

    GuiUnlock();

    if (modal || app->port_edit || app->connected) {
        GuiLock();
    }

    if (GuiDropdownBox(dd->baud, BAUD_LIST_TEXT, &app->baud_active, app->baud_edit)) {
        app->baud_edit = !app->baud_edit;
    }

    GuiUnlock();
}

/* Draws a page title and shrinks `content` to the area below it. */
static void page_header(const char *title, const char *subtitle, Rectangle *content)
{
    ui_text_bold(title, content->x + 2, content->y + 2, 24, UI_TEXT);
    ui_text(subtitle, content->x + 2, content->y + 33, UI_TEXT_SIZE - 1, UI_MUTED);
    content->y += 64;
    content->height -= 64;
}

/* One frame of UI. GuiLock() makes raygui ignore input; it is held while a modal or an open
 * dropdown list owns the mouse, and released before the dropdowns themselves are drawn. */
static void draw_frame(App *app)
{
    float w = (float)GetScreenWidth(), h = (float)GetScreenHeight();

    /* Anything modal or any open dropdown list owns the mouse this frame. */
    if (ui_modal_active() || app->port_edit || app->baud_edit) {
        GuiLock();
    }

    DropdownRects dd;
    draw_sidebar(app, h, &dd);

    if (app->main_tab == TAB_SESSION && !app->stab.open) {
        app->main_tab = TAB_TELEMETRY;
    }

    Rectangle content = {SIDEBAR_W + 20, 18, w - SIDEBAR_W - 40, h - 36};

    switch (app->main_tab) {
    case TAB_TELEMETRY:
        telemetry_tab_draw(app, content);
        break;
    case TAB_PID:
        page_header("PID Tuning", "Tune gains live and watch the response underneath", &content);
        pid_tab_draw(app, content);
        break;
    case TAB_DRONE:
        page_header("Drone Config", "Flight limits, throttle range and failsafe thresholds",
                    &content);
        drone_tab_draw(app, content);
        break;
    case TAB_REMOTE:
        page_header("Remote Config", "Battery sensing and joystick calibration", &content);
        remote_tab_draw(app, content);
        break;
    case TAB_CONSOLE:
        page_header("Console", "Everything sent to and received from the bridge", &content);
        console_tab_draw(app, content);
        break;
    case TAB_SESSION:
        session_tab_draw(app, content);
        break;
    default:
        break;
    }

    GuiUnlock();
    draw_dropdowns(app, &dd);
    ui_modal_draw();
}

int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(1360, 860, "PID Tuner");
    SetWindowMinSize(1100, 720);
    SetExitKey(KEY_NULL); /* Esc cancels field edits and dialogs; it must not quit */
    SetTargetFPS(60);

    ui_init();

    App *app = calloc(1, sizeof(*app));
    if (!app) {
        return 1;
    }

    app_init(app);

    while (!WindowShouldClose()) {
        app_update(app);

        BeginDrawing();
        ClearBackground(UI_BG);
        draw_frame(app);
        ui_end_frame();
        EndDrawing();
    }

    app_shutdown(app);
    free(app);
    ui_shutdown();
    CloseWindow();
    return 0;
}
