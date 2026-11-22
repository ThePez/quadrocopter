/* Console tab: scrolling log of everything sent and received. */

#include <stdlib.h>
#include <string.h>

#include "app.h"

void console_tab_init(ConsoleTab *t)
{
    memset(t, 0, sizeof(*t));
    t->lines = calloc(LOG_LINES, sizeof(*t->lines));
}

void console_tab_free(ConsoleTab *t)
{
    free(t->lines);
    memset(t, 0, sizeof(*t));
}

void console_tab_add(ConsoleTab *t, const char *line)
{
    if (!t->lines) {
        return;
    }

    if (t->count == LOG_LINES) {
        t->head = (t->head + 1) % LOG_LINES;
        t->count--;
    }

    int slot = (t->head + t->count) % LOG_LINES;
    strncpy(t->lines[slot], line, LOG_LEN - 1);
    t->lines[slot][LOG_LEN - 1] = '\0';
    t->count++;

    /* Keep the view steady if the user has scrolled back. */
    if (t->scroll_up > 0 && t->scroll_up < t->count - 1) {
        t->scroll_up++;
    }
}

/* "[12:34:56.789] message": the timestamp is drawn muted, the message coloured. */
static Color message_color(const char *msg)
{
    if (strstr(msg, "ailed") || strstr(msg, "lost") || strstr(msg, "rror") ||
        strstr(msg, "Warning")) {
        return UI_RED;
    }

    if (strncmp(msg, "->", 2) == 0) {
        return UI_ACCENT;
    }

    if (strncmp(msg, "Saved", 5) == 0 || strncmp(msg, "Connected", 9) == 0) {
        return UI_GREEN;
    }

    return UI_TEXT;
}

void console_tab_draw(App *app, Rectangle area)
{
    ConsoleTab *t = &app->ctab;
    const float line_h = 21, text_size = 14;

    ui_card(area, NULL);

    char count[48];
    snprintf(count, sizeof(count), "%d lines", t->count);
    ui_text(count, area.x + UI_PAD, area.y + 19, UI_SMALL + 1, UI_MUTED);
    if (ui_btn((Rectangle){area.x + area.width - UI_PAD - 90, area.y + 10, 90, 34}, "Clear")) {
        t->head = 0;
        t->count = 0;
        t->scroll_up = 0;
    }

    Rectangle log_r = {area.x + UI_PAD - 4, area.y + 54, area.width - 2 * UI_PAD + 8,
                       area.height - 54 - UI_PAD + 4};
    ui_rrect(log_r, 6, UI_FIELD);

    int visible = (int)((log_r.height - 12) / line_h);
    int max_up = t->count > visible ? t->count - visible : 0;

    if (!GuiIsLocked() && CheckCollisionPointRec(GetMousePosition(), log_r)) {
        t->scroll_up += (int)(GetMouseWheelMove() * 3);
    }

    if (t->scroll_up < 0) {
        t->scroll_up = 0;
    }

    if (t->scroll_up > max_up) {
        t->scroll_up = max_up;
    }

    int first = t->count - visible - t->scroll_up;
    if (first < 0) {
        first = 0;
    }

    BeginScissorMode((int)log_r.x, (int)log_r.y, (int)log_r.width, (int)log_r.height);
    if (t->count == 0) {
        ui_text("Nothing yet - connect to the bridge and send something.", log_r.x + 14,
                log_r.y + 12, text_size, UI_MUTED);
    }

    for (int i = 0; i < visible && first + i < t->count; i++) {
        const char *line = t->lines[(t->head + first + i) % LOG_LINES];
        float y = log_r.y + 8 + (float)i * line_h;
        const char *msg = line;
        if (line[0] == '[' && strlen(line) > 15 && line[13] == ']') {
            char stamp[16];
            memcpy(stamp, line, 14);
            stamp[14] = '\0';
            ui_text(stamp, log_r.x + 14, y, text_size, UI_MUTED);
            msg = line + 15;
        }

        ui_text(msg, log_r.x + 14 + 112, y, text_size, message_color(msg));
    }

    EndScissorMode();

    if (t->scroll_up > 0 &&
        ui_button((Rectangle){log_r.x + log_r.width - 150, log_r.y + log_r.height - 44, 138, 32},
                  "Jump to latest", UI_ACCENT)) {
        t->scroll_up = 0;
    }
}
