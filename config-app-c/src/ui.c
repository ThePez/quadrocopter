#define RAYGUI_IMPLEMENTATION
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

/* ------------------------------------------------------------------------ */
/* Font + raygui theme                                                       */
/* ------------------------------------------------------------------------ */

static Font g_font;
static bool g_font_loaded;

static const char *const FONT_PATHS[] = {
    "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    "/usr/share/fonts/noto/NotoSans-Regular.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
};

static void apply_raygui_theme(void)
{
    static const struct {
        int prop;
        Color c;
    } theme[] = {
        {BORDER_COLOR_NORMAL, UI_BORDER},   {BASE_COLOR_NORMAL, UI_FIELD},
        {TEXT_COLOR_NORMAL, UI_TEXT},       {BORDER_COLOR_FOCUSED, UI_ACCENT},
        {BASE_COLOR_FOCUSED, UI_CARD_HI},   {TEXT_COLOR_FOCUSED, UI_TEXT},
        {BORDER_COLOR_PRESSED, UI_ACCENT},  {BASE_COLOR_PRESSED, UI_CARD_HI},
        {TEXT_COLOR_PRESSED, UI_ACCENT},    {BORDER_COLOR_DISABLED, UI_BORDER},
        {BASE_COLOR_DISABLED, UI_FIELD},    {TEXT_COLOR_DISABLED, UI_MUTED},
        {BACKGROUND_COLOR, UI_BG},          {LINE_COLOR, UI_BORDER},
    };
    for (size_t i = 0; i < sizeof theme / sizeof theme[0]; i++)
        GuiSetStyle(DEFAULT, theme[i].prop, ColorToInt(theme[i].c));
}

void ui_init(void)
{
    g_font = GetFontDefault();

    /* ASCII plus the few symbols the UI prints. */
    int codepoints[95 + 4];
    int n = 0;
    for (int c = 32; c <= 126; c++)
        codepoints[n++] = c;
    codepoints[n++] = 0x00B0; /* degree */
    codepoints[n++] = 0x00B5; /* micro */
    codepoints[n++] = 0x03B1; /* alpha */
    codepoints[n++] = 0x00B7; /* middle dot */

    for (size_t i = 0; i < sizeof FONT_PATHS / sizeof FONT_PATHS[0]; i++) {
        if (!FileExists(FONT_PATHS[i]))
            continue;
        Font f = LoadFontEx(FONT_PATHS[i], 32, codepoints, n);
        if (f.texture.id != 0) {
            GenTextureMipmaps(&f.texture);
            SetTextureFilter(f.texture, TEXTURE_FILTER_TRILINEAR);
            g_font = f;
            g_font_loaded = true;
            break;
        }
    }

    GuiSetFont(g_font);
    GuiSetStyle(DEFAULT, TEXT_SIZE, UI_TEXT_SIZE);
    apply_raygui_theme();
}

void ui_shutdown(void)
{
    if (g_font_loaded)
        UnloadFont(g_font);
}

Font ui_font(void)
{
    return g_font;
}

/* ------------------------------------------------------------------------ */
/* Text                                                                      */
/* ------------------------------------------------------------------------ */

void ui_text(const char *text, float x, float y, float size, Color c)
{
    DrawTextEx(g_font, text, (Vector2){x, y}, size, 0.5f, c);
}

void ui_text_bold(const char *text, float x, float y, float size, Color c)
{
    ui_text(text, x, y, size, c);
    ui_text(text, x + 0.6f, y, size, c); /* faux bold: a second pass half a pixel over */
}

float ui_text_width(const char *text, float size)
{
    return MeasureTextEx(g_font, text, size, 0.5f).x;
}

void ui_text_right(const char *text, float right, float y, float size, Color c)
{
    ui_text(text, right - ui_text_width(text, size), y, size, c);
}

void ui_text_centered(const char *text, Rectangle r, float size, Color c)
{
    ui_text(text, r.x + (r.width - ui_text_width(text, size)) / 2, r.y + (r.height - size) / 2 - 1,
            size, c);
}

/* ------------------------------------------------------------------------ */
/* Surfaces                                                                  */
/* ------------------------------------------------------------------------ */

void ui_rrect(Rectangle r, float radius, Color fill)
{
    float half = fminf(r.width, r.height) / 2;
    float roundness = half > 0 ? fminf(radius / half, 1.0f) : 0;
    DrawRectangleRounded(r, roundness, 10, fill);
}

void ui_rrect_bordered(Rectangle r, float radius, Color fill, Color border)
{
    ui_rrect(r, radius, border);
    ui_rrect(ui_inset(r, 1), radius - 1, fill);
}

void ui_card(Rectangle r, const char *title)
{
    ui_rrect_bordered(r, UI_RADIUS, UI_CARD, UI_BORDER);
    if (title)
        ui_text_bold(title, r.x + UI_PAD, r.y + 13, UI_TEXT_SIZE, UI_TEXT);
}

Rectangle ui_card_body(Rectangle r, bool titled)
{
    float top = titled ? UI_CARD_HEAD : UI_PAD;
    return (Rectangle){r.x + UI_PAD, r.y + top, r.width - 2 * UI_PAD, r.height - top - UI_PAD};
}

void ui_dot(float cx, float cy, float radius, Color c)
{
    DrawCircleV((Vector2){cx, cy}, radius, c);
}

/* ------------------------------------------------------------------------ */
/* Click handling: a click needs the press AND the release inside the widget  */
/* ------------------------------------------------------------------------ */

static Rectangle g_armed;
static bool g_armed_valid;

static bool is_hover(Rectangle r)
{
    return !GuiIsLocked() && CheckCollisionPointRec(GetMousePosition(), r);
}

static bool same_rect(Rectangle a, Rectangle b)
{
    return a.x == b.x && a.y == b.y && a.width == b.width && a.height == b.height;
}

static bool clicked(Rectangle r, bool hover)
{
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        g_armed = r;
        g_armed_valid = true;
    }
    bool hit = hover && IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && g_armed_valid && same_rect(g_armed, r);
    if (hit)
        g_armed_valid = false;
    return hit;
}

static bool held(Rectangle r, bool hover)
{
    return hover && IsMouseButtonDown(MOUSE_BUTTON_LEFT) && g_armed_valid && same_rect(g_armed, r);
}

void ui_end_frame(void)
{
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        g_armed_valid = false;
}

/* ------------------------------------------------------------------------ */
/* Buttons and friends                                                       */
/* ------------------------------------------------------------------------ */

bool ui_btn(Rectangle r, const char *text)
{
    bool hover = is_hover(r);
    bool down = held(r, hover);
    Color fill = down ? (Color){30, 34, 45, 255} : hover ? (Color){48, 55, 72, 255} : UI_CARD_HI;
    ui_rrect_bordered(r, 6, fill, hover ? (Color){72, 82, 104, 255} : UI_BORDER);
    ui_text_centered(text, r, UI_TEXT_SIZE, UI_TEXT);
    return clicked(r, hover);
}

bool ui_button(Rectangle r, const char *text, Color base)
{
    bool hover = is_hover(r);
    bool down = held(r, hover);
    Color fill = down ? ColorBrightness(base, -0.22f) : hover ? ColorBrightness(base, 0.14f) : base;
    ui_rrect(r, 6, fill);
    ui_text_centered(text, r, UI_TEXT_SIZE, WHITE);
    return clicked(r, hover);
}

bool ui_link(Rectangle r, const char *text)
{
    bool hover = is_hover(r);
    ui_text_centered(text, r, UI_SMALL + 1, hover ? ColorBrightness(UI_ACCENT, 0.25f) : UI_ACCENT);
    if (hover) {
        float w = ui_text_width(text, UI_SMALL + 1);
        float y = r.y + (r.height + UI_SMALL) / 2 + 1;
        DrawLine((int)(r.x + (r.width - w) / 2), (int)y, (int)(r.x + (r.width + w) / 2), (int)y, UI_ACCENT);
    }
    return clicked(r, hover);
}

bool ui_nav_item(Rectangle r, const char *label, bool active)
{
    bool hover = is_hover(r);
    if (active)
        ui_rrect(r, 8, UI_CARD_HI);
    else if (hover)
        ui_rrect(r, 8, (Color){30, 34, 45, 255});
    if (active)
        ui_rrect((Rectangle){r.x, r.y + 9, 3, r.height - 18}, 1.5f, UI_ACCENT);
    ui_text(label, r.x + 18, r.y + (r.height - UI_TEXT_SIZE) / 2 - 1, UI_TEXT_SIZE,
            active ? UI_TEXT : UI_MUTED);
    return clicked(r, hover);
}

#define SEG_MAX 8

static int split_labels(const char *labels, char out[][32])
{
    int n = 0;
    const char *p = labels;
    while (*p && n < SEG_MAX) {
        const char *semi = strchr(p, ';');
        size_t len = semi ? (size_t)(semi - p) : strlen(p);
        if (len > 31)
            len = 31;
        memcpy(out[n], p, len);
        out[n][len] = '\0';
        n++;
        if (!semi)
            break;
        p = semi + 1;
    }
    return n;
}

float ui_segmented_width(const char *labels)
{
    char items[SEG_MAX][32];
    int n = split_labels(labels, items);
    float w = 8;
    for (int i = 0; i < n; i++)
        w += ui_text_width(items[i], UI_TEXT_SIZE) + 28;
    return w;
}

bool ui_segmented(Rectangle r, const char *labels, int *active)
{
    char items[SEG_MAX][32];
    int n = split_labels(labels, items);
    bool changed = false;

    Rectangle box = {r.x, r.y, ui_segmented_width(labels), r.height};
    ui_rrect_bordered(box, 8, UI_FIELD, UI_BORDER);

    float x = r.x + 4;
    for (int i = 0; i < n; i++) {
        float w = ui_text_width(items[i], UI_TEXT_SIZE) + 28;
        Rectangle item = {x, r.y + 4, w, r.height - 8};
        bool hover = is_hover(item);
        if (i == *active)
            ui_rrect(item, 6, UI_ACCENT);
        else if (hover)
            ui_rrect(item, 6, UI_CARD_HI);
        ui_text_centered(items[i], item, UI_TEXT_SIZE, i == *active ? WHITE : UI_MUTED);
        if (clicked(item, hover) && i != *active) {
            *active = i;
            changed = true;
        }
        x += w;
    }
    return changed;
}

bool ui_checkbox(Rectangle r, const char *label, bool *value)
{
    bool hover = is_hover(r);
    bool toggled = clicked(r, hover);
    if (toggled)
        *value = !*value;

    Rectangle box = {r.x, r.y + (r.height - 20) / 2, 20, 20};
    if (*value) {
        ui_rrect(box, 5, UI_ACCENT);
        DrawLineEx((Vector2){box.x + 5, box.y + 10.5f}, (Vector2){box.x + 8.5f, box.y + 14}, 2.2f, WHITE);
        DrawLineEx((Vector2){box.x + 8.5f, box.y + 14}, (Vector2){box.x + 15, box.y + 6}, 2.2f, WHITE);
    } else {
        ui_rrect_bordered(box, 5, UI_FIELD, hover ? UI_MUTED : UI_BORDER);
    }
    ui_text(label, box.x + 28, r.y + (r.height - UI_TEXT_SIZE) / 2 - 1, UI_TEXT_SIZE, UI_TEXT);
    return toggled;
}

/* ------------------------------------------------------------------------ */
/* NumField                                                                  */
/* ------------------------------------------------------------------------ */

static NumField *g_editing; /* at most one field is mid-edit */

void numfield_init(NumField *f, double value, double step, double min, double max, int decimals)
{
    memset(f, 0, sizeof *f);
    f->value = value;
    f->step = step;
    f->min = min;
    f->max = max;
    f->decimals = decimals;
}

static void field_clamp(NumField *f)
{
    if (f->value < f->min)
        f->value = f->min;
    if (f->value > f->max)
        f->value = f->max;
}

static void field_commit(NumField *f)
{
    if (!f->editing)
        return;
    char *end;
    double v = strtod(f->buf, &end);
    if (end != f->buf && *end == '\0') {
        f->value = v;
        field_clamp(f);
    }
    f->editing = false;
    if (g_editing == f)
        g_editing = NULL;
}

static void field_begin(NumField *f)
{
    if (g_editing && g_editing != f)
        field_commit(g_editing);
    snprintf(f->buf, sizeof f->buf, "%.*f", f->decimals, f->value);
    f->editing = true;
    g_editing = f;
}

void ui_commit_edit(void)
{
    if (g_editing)
        field_commit(g_editing);
}

static void draw_chevron(Rectangle r, bool up, Color c)
{
    float cx = r.x + r.width / 2, cy = r.y + r.height / 2, s = 3.5f;
    float dy = up ? -s * 0.5f : s * 0.5f;
    Vector2 a = {cx - s, cy - dy}, b = {cx, cy + dy}, d = {cx + s, cy - dy};
    DrawLineEx(a, b, 1.6f, c);
    DrawLineEx(b, d, 1.6f, c);
}

void numfield_draw(NumField *f, Rectangle b)
{
    const float step_w = 24;
    Rectangle text = {b.x, b.y, b.width - step_w, b.height};
    Rectangle up = {b.x + b.width - step_w, b.y + 1, step_w - 1, b.height / 2 - 1};
    Rectangle dn = {up.x, b.y + b.height / 2, step_w - 1, b.height / 2 - 1};

    bool locked = GuiIsLocked();
    Vector2 m = GetMousePosition();
    bool over = !locked && CheckCollisionPointRec(m, b);

    if (!locked && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(m, text)) {
            if (!f->editing)
                field_begin(f);
        } else {
            if (f->editing)
                field_commit(f);
            if (CheckCollisionPointRec(m, up)) {
                f->value += f->step;
                field_clamp(f);
            } else if (CheckCollisionPointRec(m, dn)) {
                f->value -= f->step;
                field_clamp(f);
            }
        }
    }

    if (over && !f->editing) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            f->value += f->step * (wheel > 0 ? 1 : -1);
            field_clamp(f);
        }
    }

    if (f->editing && !locked) {
        int ch;
        while ((ch = GetCharPressed()) != 0) {
            size_t len = strlen(f->buf);
            if (ch < 128 && strchr("0123456789.-+eE", ch) && len + 1 < sizeof f->buf) {
                f->buf[len] = (char)ch;
                f->buf[len + 1] = '\0';
            }
        }
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) {
            size_t len = strlen(f->buf);
            if (len)
                f->buf[len - 1] = '\0';
        }
        if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))
            field_commit(f);
        else if (IsKeyPressed(KEY_ESCAPE)) {
            f->editing = false;
            g_editing = NULL;
        }
    }

    /* Draw */
    Color border = f->editing ? UI_ACCENT : (over ? (Color){72, 82, 104, 255} : UI_BORDER);
    ui_rrect_bordered(b, 6, UI_FIELD, border);

    char shown[40];
    if (f->editing)
        snprintf(shown, sizeof shown, "%s", f->buf);
    else
        snprintf(shown, sizeof shown, "%.*f", f->decimals, f->value);
    float ty = b.y + (b.height - UI_TEXT_SIZE) / 2 - 1;
    ui_text(shown, b.x + 10, ty, UI_TEXT_SIZE, UI_TEXT);
    if (f->editing && ((int)(GetTime() * 2) % 2 == 0)) {
        float cx = b.x + 11 + ui_text_width(shown, UI_TEXT_SIZE);
        DrawLine((int)cx, (int)(ty + 1), (int)cx, (int)(ty + UI_TEXT_SIZE), UI_ACCENT);
    }

    DrawLine((int)(b.x + b.width - step_w), (int)(b.y + 5), (int)(b.x + b.width - step_w),
             (int)(b.y + b.height - 5), UI_BORDER);
    bool hu = !locked && CheckCollisionPointRec(m, up);
    bool hd = !locked && CheckCollisionPointRec(m, dn);
    draw_chevron(up, true, hu ? UI_TEXT : UI_MUTED);
    draw_chevron(dn, false, hd ? UI_TEXT : UI_MUTED);
}

void ui_field_row(Rectangle row, const char *label, NumField *f)
{
    ui_text(label, row.x, row.y + (row.height - UI_TEXT_SIZE) / 2 - 1, UI_TEXT_SIZE, UI_TEXT);
    float fw = fminf(150, row.width * 0.5f);
    numfield_draw(f, (Rectangle){row.x + row.width - fw, row.y + (row.height - UI_FIELD_H) / 2, fw,
                                 UI_FIELD_H});
}

/* ------------------------------------------------------------------------ */
/* Modals                                                                    */
/* ------------------------------------------------------------------------ */

enum { MODAL_NONE, MODAL_MESSAGE, MODAL_FILES };

#define MAX_FILES 256

static struct {
    int kind;
    char title[96];

    /* message */
    char msg[768];
    char buttons[128];
    ModalCb cb;
    void *ctx;

    /* file picker */
    FilePickCb pick_cb;
    char dir[512];
    char (*paths)[512];
    int nfiles;
    int selected;
    int scroll;
    double last_click;
    int last_click_idx;
} M;

bool ui_modal_active(void)
{
    return M.kind != MODAL_NONE;
}

void ui_modal_open(const char *title, const char *msg, const char *buttons, ModalCb cb, void *ctx)
{
    ui_commit_edit();
    M.kind = MODAL_MESSAGE;
    str_copy(M.title, sizeof M.title, title);
    str_copy(M.msg, sizeof M.msg, msg);
    str_copy(M.buttons, sizeof M.buttons, buttons);
    M.cb = cb;
    M.ctx = ctx;
}

static int cmp_desc(const void *a, const void *b)
{
    return strcmp((const char *)b, (const char *)a);
}

void ui_file_picker_open(const char *title, const char *dir, const char *ext, FilePickCb cb, void *ctx)
{
    ui_commit_edit();
    free(M.paths);
    M.paths = malloc(MAX_FILES * sizeof *M.paths);
    M.nfiles = 0;

    FilePathList list = LoadDirectoryFilesEx(dir, ext, false);
    for (unsigned i = 0; i < list.count && M.nfiles < MAX_FILES; i++)
        str_copy(M.paths[M.nfiles++], sizeof M.paths[0], list.paths[i]);
    UnloadDirectoryFiles(list);
    /* Session names embed a timestamp, so reverse order = newest first. */
    qsort(M.paths, (size_t)M.nfiles, sizeof M.paths[0], cmp_desc);

    M.kind = MODAL_FILES;
    str_copy(M.title, sizeof M.title, title);
    str_copy(M.dir, sizeof M.dir, dir);
    M.pick_cb = cb;
    M.ctx = ctx;
    M.selected = M.nfiles ? 0 : -1;
    M.scroll = 0;
    M.last_click_idx = -1;
}

/* Greedy word wrap honouring explicit newlines. */
static int wrap_text(const char *msg, float max_w, float size, char out[][200], int max_lines)
{
    int n = 0;
    char line[200] = "";
    const char *p = msg;

    while (*p && n < max_lines) {
        if (*p == '\n') {
            str_copy(out[n++], 200, line);
            line[0] = '\0';
            p++;
            continue;
        }
        char word[160];
        int w = 0;
        while (*p && *p != ' ' && *p != '\n' && w < 159)
            word[w++] = *p++;
        word[w] = '\0';
        if (*p == ' ')
            p++;

        char trial[400];
        snprintf(trial, sizeof trial, "%s%s%s", line, line[0] ? " " : "", word);
        if (line[0] && ui_text_width(trial, size) > max_w) {
            str_copy(out[n++], 200, line);
            str_copy(line, sizeof line, word);
        } else {
            str_copy(line, sizeof line, trial);
        }
    }
    if (n < max_lines && line[0])
        str_copy(out[n++], 200, line);
    return n;
}

static void draw_panel(Rectangle r, const char *title)
{
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.6f));
    ui_rrect_bordered(r, 12, UI_CARD, UI_BORDER);
    ui_text_bold(title, r.x + 24, r.y + 20, 18, UI_TEXT);
}

static void draw_message_modal(void)
{
    enum { MAX_BTN = 4, MAX_LINES = 14 };
    char lines[MAX_LINES][200];
    const float w = 540;
    int nlines = wrap_text(M.msg, w - 48, UI_TEXT_SIZE, lines, MAX_LINES);

    float h = 20 + 22 + 22 + (float)nlines * 24 + 26 + UI_BTN_H + 24;
    Rectangle r = {(GetScreenWidth() - w) / 2, (GetScreenHeight() - h) / 2, w, h};
    draw_panel(r, M.title);

    for (int i = 0; i < nlines; i++)
        ui_text(lines[i], r.x + 24, r.y + 62 + (float)i * 24, UI_TEXT_SIZE, UI_MUTED);

    char labels[MAX_BTN][32];
    int nb = 0;
    char tmp[128];
    str_copy(tmp, sizeof tmp, M.buttons);
    for (char *save = NULL, *tok = strtok_r(tmp, ";", &save); tok && nb < MAX_BTN;
         tok = strtok_r(NULL, ";", &save))
        str_copy(labels[nb++], sizeof labels[0], tok);
    if (nb == 0)
        str_copy(labels[nb++], sizeof labels[0], "OK");

    /* Buttons right-aligned, primary (first) rightmost. */
    const float bw = 104, gap = 10;
    float bx = r.x + r.width - 24 - bw;
    float by = r.y + r.height - 24 - UI_BTN_H;

    int pressed = -1;
    for (int i = 0; i < nb; i++) {
        Rectangle br = {bx - (float)i * (bw + gap), by, bw, UI_BTN_H};
        bool hit = i == 0 ? ui_button(br, labels[i], UI_ACCENT) : ui_btn(br, labels[i]);
        if (hit)
            pressed = i;
    }
    if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))
        pressed = 0;
    else if (IsKeyPressed(KEY_ESCAPE))
        pressed = nb - 1;

    if (pressed >= 0) {
        ModalCb cb = M.cb;
        void *ctx = M.ctx;
        M.kind = MODAL_NONE; /* close first so cb may open another dialog */
        if (cb)
            cb(ctx, pressed);
    }
}

static void close_picker(const char *chosen)
{
    FilePickCb cb = M.pick_cb;
    void *ctx = M.ctx;
    char path[512] = "";
    if (chosen)
        str_copy(path, sizeof path, chosen);
    M.kind = MODAL_NONE;
    if (cb)
        cb(ctx, chosen ? path : NULL);
}

static void draw_file_modal(void)
{
    const float w = 640, h = 480;
    Rectangle r = {(GetScreenWidth() - w) / 2, (GetScreenHeight() - h) / 2, w, h};
    draw_panel(r, M.title);

    char hint[600];
    snprintf(hint, sizeof hint, "%s  -  or drop a .csv file on the window", M.dir);
    ui_text(hint, r.x + 24, r.y + 50, UI_SMALL, UI_MUTED);

    Rectangle list = {r.x + 24, r.y + 78, w - 48, h - 78 - 24 - UI_BTN_H - 16};
    ui_rrect_bordered(list, 8, UI_FIELD, UI_BORDER);

    const float row_h = 30;
    int visible = (int)((list.height - 8) / row_h);
    Vector2 m = GetMousePosition();

    if (CheckCollisionPointRec(m, list)) {
        M.scroll -= (int)GetMouseWheelMove();
        if (M.scroll > M.nfiles - visible)
            M.scroll = M.nfiles - visible;
        if (M.scroll < 0)
            M.scroll = 0;
    }

    if (M.nfiles == 0)
        ui_text("No session files found.", list.x + 16, list.y + 14, UI_TEXT_SIZE, UI_MUTED);

    int open_idx = -1;
    BeginScissorMode((int)list.x, (int)list.y, (int)list.width, (int)list.height);
    for (int i = 0; i < visible && M.scroll + i < M.nfiles; i++) {
        int idx = M.scroll + i;
        Rectangle row = {list.x + 4, list.y + 4 + (float)i * row_h, list.width - 8, row_h};
        if (idx == M.selected)
            ui_rrect(row, 6, Fade(UI_ACCENT, 0.28f));
        else if (CheckCollisionPointRec(m, row))
            ui_rrect(row, 6, UI_CARD_HI);
        ui_text(GetFileName(M.paths[idx]), row.x + 12, row.y + 6, UI_TEXT_SIZE, UI_TEXT);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(m, row)) {
            double now = GetTime();
            if (M.last_click_idx == idx && now - M.last_click < 0.4)
                open_idx = idx; /* double click */
            M.selected = idx;
            M.last_click_idx = idx;
            M.last_click = now;
        }
    }
    EndScissorMode();

    float by = r.y + r.height - 24 - UI_BTN_H;
    Rectangle open_r = {r.x + r.width - 24 - 104, by, 104, UI_BTN_H};
    Rectangle cancel_r = {open_r.x - 10 - 104, by, 104, UI_BTN_H};
    if (ui_button(open_r, "Open", UI_ACCENT) && M.selected >= 0)
        open_idx = M.selected;
    bool cancel = ui_btn(cancel_r, "Cancel") || IsKeyPressed(KEY_ESCAPE);

    if (open_idx >= 0)
        close_picker(M.paths[open_idx]);
    else if (cancel)
        close_picker(NULL);
}

void ui_modal_draw(void)
{
    switch (M.kind) {
    case MODAL_MESSAGE: draw_message_modal(); break;
    case MODAL_FILES: draw_file_modal(); break;
    default: break;
    }
}
