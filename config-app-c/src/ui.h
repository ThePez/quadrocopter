/*
 *****************************************************************************
 * File: ui.h
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Interface for the shared UI toolkit.
 *****************************************************************************
 */

/*
 * Shared UI toolkit on top of raylib + raygui: dark theme, cards, buttons,
 * segmented control, sidebar items, numeric fields and modal dialogs.
 *
 * raygui supplies the dropdowns, the lock mechanism (GuiLock/GuiIsLocked) and
 * the font/style plumbing. Everything it lacks or draws plainly is built here.
 */
#ifndef UI_H
#define UI_H

#include <stdbool.h>

#include "raylib.h"
#include "raygui.h"

/* ---- Palette ---- */

/* clang-format off */
#define UI_BG      (Color){15, 17, 22, 255}   /* page */
#define UI_SIDEBAR (Color){20, 23, 30, 255}
#define UI_CARD    (Color){27, 31, 40, 255}
#define UI_CARD_HI (Color){38, 43, 56, 255}   /* hover / raised */
#define UI_BORDER  (Color){48, 55, 70, 255}
#define UI_FIELD   (Color){18, 21, 28, 255}   /* inputs, plot wells */
#define UI_GRID    (Color){40, 46, 59, 255}
#define UI_TEXT    (Color){228, 232, 240, 255}
#define UI_MUTED   (Color){132, 141, 158, 255}
#define UI_ACCENT  (Color){70, 152, 255, 255}
#define UI_GREEN   (Color){46, 194, 126, 255}
#define UI_RED     (Color){242, 85, 90, 255}
#define UI_ORANGE  (Color){255, 159, 64, 255}

/* Series colours tuned for the dark background. */
#define UI_SERIES_RED    (Color){255, 99, 99, 255}
#define UI_SERIES_GREEN  (Color){84, 214, 140, 255}
#define UI_SERIES_BLUE   (Color){92, 160, 255, 255}
#define UI_SERIES_PURPLE (Color){187, 128, 255, 255}
#define UI_SERIES_ORANGE (Color){255, 171, 76, 255}
#define UI_SERIES_CYAN   (Color){64, 212, 232, 255}
/* clang-format on */

/* ---- Metrics ---- */

#define UI_TEXT_SIZE    16
#define UI_SMALL        13
#define UI_PAD          16 /* card inner padding */
#define UI_GAP          12 /* space between cards */
#define UI_RADIUS       8
#define UI_FIELD_H      32
#define UI_ROW_H        40 /* label + field row */
#define UI_BTN_H        36
#define UI_CARD_HEAD    42   /* height of a card's title strip */
#define UI_TEXT_SPACING 0.0f /* whole-pixel glyph advances keep small text evenly sharp */

/* ---- Setup ---- */

void ui_init(void); /* after InitWindow(): font + raygui theme */
void ui_shutdown(void);
void ui_end_frame(void); /* call once per frame after drawing */

/* ---- Layout helpers ---- */

static inline Rectangle ui_inset(Rectangle r, float p)
{
    return (Rectangle){r.x + p, r.y + p, r.width - 2 * p, r.height - 2 * p};
}

/* i-th of n equal columns / rows of `area`, separated by `gap`. */
static inline Rectangle ui_col(Rectangle area, int n, int i, float gap)
{
    float w = (area.width - gap * (float)(n - 1)) / (float)n;
    return (Rectangle){area.x + (float)i * (w + gap), area.y, w, area.height};
}

static inline Rectangle ui_row(Rectangle area, int n, int i, float gap)
{
    float h = (area.height - gap * (float)(n - 1)) / (float)n;
    return (Rectangle){area.x, area.y + (float)i * (h + gap), area.width, h};
}

/* ---- Text ---- */

Font ui_font(void);
void ui_text(const char *text, float x, float y, float size, Color c);
void ui_text_bold(const char *text, float x, float y, float size, Color c);
void ui_text_right(const char *text, float right, float y, float size, Color c);
void ui_text_centered(const char *text, Rectangle r, float size, Color c);
float ui_text_width(const char *text, float size);

/* ---- Surfaces ---- */

void ui_rrect(Rectangle r, float radius, Color fill);
void ui_rrect_bordered(Rectangle r, float radius, Color fill, Color border);
/* Rounded card; `title` (optional) is drawn in a strip at the top. Use
 * ui_card_body() for the area underneath it. */
void ui_card(Rectangle r, const char *title);
Rectangle ui_card_body(Rectangle r, bool titled);
void ui_dot(float cx, float cy, float radius, Color c);

/* ---- Controls ---- */

bool ui_btn(Rectangle r, const char *text);                /* neutral */
bool ui_button(Rectangle r, const char *text, Color base); /* filled accent */
bool ui_link(Rectangle r, const char *text);               /* text-only action */
bool ui_nav_item(Rectangle r, const char *label, bool active);
/* Pill-shaped tab strip, items sized to their text. Returns true on change. */
bool ui_segmented(Rectangle r, const char *labels, int *active);
float ui_segmented_width(const char *labels);
bool ui_checkbox(Rectangle r, const char *label, bool *value);

/* Numeric entry: click to type (Enter / click-away commits, Esc cancels); the
 * chevrons or mouse wheel step by `step`. While typing, click or use Left / Right /
 * Home / End to move the caret, and Backspace / Delete to edit at it. */
typedef struct {
    double value;
    double step, min, max;
    int decimals;
    bool editing;
    char buf[32];
    int cursor; /* caret position in buf, 0..strlen(buf) */
} NumField;

void numfield_init(NumField *f, double value, double step, double min, double max, int decimals);
void numfield_draw(NumField *f, Rectangle bounds);
/* Commits whichever field is mid-edit (call when switching tabs). */
void ui_commit_edit(void);
/* Label on the left, field right-aligned in `row`. */
void ui_field_row(Rectangle row, const char *label, NumField *f);

/* ---- Modal dialogs (drawn last; callers lock raygui while one is open) ---- */

typedef void (*ModalCb)(void *ctx, int button);
typedef void (*FilePickCb)(void *ctx, const char *path); /* path == NULL on cancel */

/* `buttons` is semicolon separated ("Yes;No"); the first is the primary
 * action. `cb` may be NULL and is called with the pressed button's index after
 * the dialog has closed, so it may open another dialog. */
void ui_modal_open(const char *title, const char *msg, const char *buttons, ModalCb cb, void *ctx);
void ui_file_picker_open(const char *title, const char *dir, const char *ext, FilePickCb cb,
                         void *ctx);
bool ui_modal_active(void);
void ui_modal_draw(void);

#endif
