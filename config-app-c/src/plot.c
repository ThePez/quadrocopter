#include "plot.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ui.h"
#include "util.h"

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */
/* ------------------------------------------------------------------------ */

static void init_common(Plot *p, const char *title, const char *ylabel, int nseries,
                        const char *const *names, const Color *colors)
{
    memset(p, 0, sizeof(*p));
    p->title = title;
    p->ylabel = ylabel;
    p->xlabel = "Time (s)";
    p->nseries = nseries > PLOT_MAX_SERIES ? PLOT_MAX_SERIES : nseries;
    for (int i = 0; i < p->nseries; i++) {
        p->names[i] = names[i];
        p->colors[i] = colors[i];
    }
}

void plot_init_live(Plot *p, const char *title, const char *ylabel, int nseries,
                    const char *const *names, const Color *colors, int capacity)
{
    init_common(p, title, ylabel, nseries, names, colors);
    p->live = true;
    p->cap = capacity;
    p->t = malloc((size_t)capacity * sizeof(*p->t));
    for (int i = 0; i < p->nseries; i++) {
        p->y[i] = malloc((size_t)capacity * sizeof(*p->y[i]));
    }

    p->t0 = now_s();
}

void plot_init_static(Plot *p, const char *title, const char *ylabel, int nseries,
                      const char *const *names, const Color *colors, PlotView *view)
{
    init_common(p, title, ylabel, nseries, names, colors);
    p->view = view;
}

void plot_free(Plot *p)
{
    if (p->live) {
        free(p->t);
        for (int i = 0; i < p->nseries; i++) {
            free(p->y[i]);
        }
    }

    memset(p, 0, sizeof(*p));
}

void plot_push(Plot *p, double t, const float *values)
{
    if (p->paused || !p->live) {
        return;
    }

    if (p->count == p->cap) { /* slide the window by one sample */
        memmove(p->t, p->t + 1, (size_t)(p->cap - 1) * sizeof(*p->t));
        for (int i = 0; i < p->nseries; i++) {
            memmove(p->y[i], p->y[i] + 1, (size_t)(p->cap - 1) * sizeof(*p->y[i]));
        }

        p->count--;
    }

    p->t[p->count] = t - p->t0;
    for (int i = 0; i < p->nseries; i++) {
        p->y[i][p->count] = values[i];
    }

    p->count++;
}

void plot_clear(Plot *p)
{
    p->count = 0;
    p->t0 = now_s();
}

void plot_bind(Plot *p, int series, const double *t, const float *y, int count)
{
    p->t = (double *)t; /* aliased, never written for static plots */
    p->y[series] = (float *)y;
    p->count = count;
}

/* ------------------------------------------------------------------------ */
/* Axis helpers                                                              */
/* ------------------------------------------------------------------------ */

static double nice_step(double raw);

/*
 * Frames the data with limits that are round numbers. Because the limits snap
 * to multiples of a "nice" step, they only move in whole steps - so the axis
 * stays put while a signal wiggles and only jumps when it genuinely leaves
 * the frame, instead of rescaling every sample.
 */
void plot_pick_y_range(double data_lo, double data_hi, double *out_lo, double *out_hi)
{
    double lo = data_lo, hi = data_hi;

    /* A (nearly) flat signal such as battery voltage would otherwise be
     * stretched until its noise fills the plot: enforce a minimum span of 1%
     * of the signal's magnitude. The absolute floor of 1 suits every unit
     * plotted here (deg, deg/s, us, mV) and stops a signal sitting at exactly
     * zero (idle PID outputs) from getting an axis like +/-0.0006. */
    double mag = fmax(fabs(lo), fabs(hi));
    double min_span = fmax(1.0, mag * 0.01);
    if (hi - lo < min_span) {
        double mid = (lo + hi) / 2.0;
        lo = mid - min_span / 2.0;
        hi = mid + min_span / 2.0;
    }

    /* If zero is close to the data, include it: a baseline reads better than
     * a range that starts at an arbitrary offset. */
    double span = hi - lo;
    if (lo > 0 && lo < span * 0.5) {
        lo = 0;
    } else if (hi < 0 && -hi < span * 0.5) {
        hi = 0;
    }

    /* Pad, then round outward to the tick grid. */
    span = hi - lo;
    lo -= span * 0.08;
    hi += span * 0.08;
    double step = nice_step((hi - lo) / 8.0);
    *out_lo = floor(lo / step) * step;
    *out_hi = ceil(hi / step) * step;
}

/* Rounds a raw tick spacing to 1, 2, 5 x 10^n. */
static double nice_step(double raw)
{
    if (raw <= 0 || !isfinite(raw)) {
        return 1.0;
    }

    double e = floor(log10(raw));
    double f = raw / pow(10.0, e);
    double nf = f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10;
    return nf * pow(10.0, e);
}

static void format_tick(char *buf, size_t cap, double v, double step)
{
    int dec = step >= 1.0 ? 0 : (int)ceil(-log10(step));
    if (dec > 6) {
        dec = 6;
    }

    if (fabs(v) < step * 1e-6) {
        v = 0.0; /* avoid "-0" */
    }

    snprintf(buf, cap, "%.*f", dec, v);
}

/* First index with t[i] >= x. */
static int lower_bound(const double *t, int n, double x)
{
    int lo = 0, hi = n;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (t[mid] < x) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }

    return lo;
}

/* ------------------------------------------------------------------------ */
/* Interaction (static plots)                                                */
/* ------------------------------------------------------------------------ */

static void handle_view_input(Plot *p, Rectangle area)
{
    PlotView *v = p->view;
    if (GuiIsLocked()) {
        return;
    }

    Vector2 m = GetMousePosition();
    bool inside = CheckCollisionPointRec(m, area);
    double width = v->x1 - v->x0;
    double full = v->full1 - v->full0;

    if (inside) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            double anchor = v->x0 + (double)(m.x - area.x) / area.width * width;
            double scale = pow(0.85, (double)wheel);
            double nw = width * scale;
            if (nw > full * 1.0) {
                nw = full;
            }

            if (nw < 1e-3) {
                nw = 1e-3;
            }

            double frac = (anchor - v->x0) / width;
            v->x0 = anchor - frac * nw;
            v->x1 = v->x0 + nw;
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            double now = GetTime();
            if (now - p->last_press < 0.3) { /* double click */
                v->x0 = v->full0;
                v->x1 = v->full1;
            } else {
                p->dragging = true;
            }

            p->last_press = now;
        }
    }

    if (p->dragging) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            double dx = -(double)GetMouseDelta().x / area.width * width;
            v->x0 += dx;
            v->x1 += dx;
        } else {
            p->dragging = false;
        }
    }

    /* Keep the window inside the recording. */
    width = v->x1 - v->x0;
    if (v->x0 < v->full0) {
        v->x0 = v->full0;
        v->x1 = v->x0 + width;
    }

    if (v->x1 > v->full1) {
        v->x1 = v->full1;
        v->x0 = v->x1 - width;
        if (v->x0 < v->full0) {
            v->x0 = v->full0;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Drawing                                                                   */
/* ------------------------------------------------------------------------ */

/* Extra height a header needs when its legend wraps onto a second row. */
#define LEGEND_ROW_H 22

/*
 * Series names may be "Group|Leaf" (e.g. "Front|Left"). Consecutive series
 * that share a group print the group once and then just their leaves:
 *     Front  o Left  o Right     Back  o Left  o Right
 * Names without a '|' are plain.
 */
static const char *split_name(const char *name, char *group, size_t cap)
{
    const char *bar = strchr(name, '|');
    if (!bar) {
        group[0] = '\0';
        return name;
    }

    size_t n = (size_t)(bar - name);
    if (n >= cap) {
        n = cap - 1;
    }

    memcpy(group, name, n);
    group[n] = '\0';
    return bar + 1;
}

/* "Front|Left" -> "Front Left" (for the hover read-out). */
static void display_name(const char *name, char *out, size_t cap)
{
    char group[24];
    const char *leaf = split_name(name, group, sizeof(group));
    snprintf(out, cap, "%s%s%s", group, group[0] ? " " : "", leaf);
}

/* Lays the legend out from x; draws it when `draw`. Returns its width
 * (including one trailing gap). */
static float legend_layout(const Plot *p, float x, float y, bool draw)
{
    const float x_start = x;
    char prev[24] = "";

    for (int s = 0; s < p->nseries; s++) {
        char group[24];
        const char *leaf = split_name(p->names[s], group, sizeof(group));

        if (group[0] && strcmp(group, prev) != 0) {
            if (s > 0) {
                x += 8; /* breathing room between groups */
            }

            if (draw) {
                ui_text(group, x, y, UI_SMALL + 1, UI_TEXT);
            }

            x += ui_text_width(group, UI_SMALL + 1) + 12;
        }

        str_copy(prev, sizeof(prev), group);

        if (draw) {
            ui_dot(x + 4, y + 8, 4.5f, p->colors[s]);
            ui_text(leaf, x + 14, y, UI_SMALL + 1, UI_MUTED);
        }

        x += 14 + ui_text_width(leaf, UI_SMALL + 1) + 16;
    }

    return x - x_start;
}

/* Draws one row per group ("Front o Left o Right" / "Back o Left o Right").
 * Returns the number of rows. Used when the whole legend is too wide for one row. */
static int legend_stacked(const Plot *p, float x, float y, bool draw)
{
    int rows = 0;
    float rx = x;
    char prev[24] = "";

    for (int s = 0; s < p->nseries; s++) {
        char group[24];
        const char *leaf = split_name(p->names[s], group, sizeof(group));

        if (s == 0 || strcmp(group, prev) != 0) {
            rows++;
            rx = x;
            if (group[0]) {
                if (draw) {
                    ui_text(group, rx, y + (float)(rows - 1) * LEGEND_ROW_H, UI_SMALL + 1, UI_TEXT);
                }

                rx += ui_text_width(group, UI_SMALL + 1) + 12;
            }
        }

        str_copy(prev, sizeof(prev), group);

        if (draw) {
            float ry = y + (float)(rows - 1) * LEGEND_ROW_H;
            ui_dot(rx + 4, ry + 8, 4.5f, p->colors[s]);
            ui_text(leaf, rx + 14, ry, UI_SMALL + 1, UI_MUTED);
        }

        rx += 14 + ui_text_width(leaf, UI_SMALL + 1) + 16;
    }

    return rows;
}

static bool has_groups(const Plot *p)
{
    for (int s = 0; s < p->nseries; s++) {
        if (strchr(p->names[s], '|')) {
            return true;
        }
    }

    return false;
}

/* True when the legend's names fit on the card at all, but not beside the title. */
static bool legend_wraps(const Plot *p, float card_width)
{
    float avail = card_width - 2 * UI_PAD;
    float title_w = ui_text_width(p->title, UI_TEXT_SIZE);
    float full = legend_layout(p, 0, 0, false);
    return title_w + 16 + full > avail && full <= avail + 16;
}

/* Draws the header and returns how much extra height it used. */
static float draw_header(const Plot *p, Rectangle b)
{
    float x = b.x + UI_PAD;
    float title_w = ui_text_width(p->title, UI_TEXT_SIZE);

    float full_w = legend_layout(p, 0, 0, false);
    if (has_groups(p) && full_w > b.width - 2 * UI_PAD + 16) {
        /* Too wide even on its own row: one row per group. */
        ui_text_bold(p->title, x, b.y + 13, UI_TEXT_SIZE, UI_TEXT);
        if (p->ylabel) {
            ui_text(p->ylabel, x + title_w + 10, b.y + 15, UI_SMALL, UI_MUTED);
        }

        int rows = legend_stacked(p, x, b.y + 14 + LEGEND_ROW_H, true);
        return (float)rows * LEGEND_ROW_H;
    }

    if (legend_wraps(p, b.width)) {
        /* Title (with its unit) on the first row, full legend on a second. */
        ui_text_bold(p->title, x, b.y + 13, UI_TEXT_SIZE, UI_TEXT);
        if (p->ylabel) {
            ui_text(p->ylabel, x + title_w + 10, b.y + 15, UI_SMALL, UI_MUTED);
        }

        legend_layout(p, x, b.y + 14 + LEGEND_ROW_H, true);
        return LEGEND_ROW_H;
    }

    float avail = b.width - 2 * UI_PAD;
    float unit_w = p->ylabel ? ui_text_width(p->ylabel, UI_SMALL) + 10 : 0;
    float legend_full = legend_layout(p, 0, 0, false);
    float legend_dots = 22.0f * (float)p->nseries;

    /* Squeeze gracefully: drop the unit first, then the legend's names. */
    bool show_unit = title_w + unit_w + 16 + legend_full <= avail;
    bool show_names = title_w + 16 + legend_full <= avail;

    ui_text_bold(p->title, x, b.y + 13, UI_TEXT_SIZE, UI_TEXT);
    if (show_unit && p->ylabel) {
        ui_text(p->ylabel, x + title_w + 10, b.y + 15, UI_SMALL, UI_MUTED);
    }

    if (show_names) {
        legend_layout(p, b.x + b.width - UI_PAD - legend_full + 16, b.y + 14, true);
    } else {
        float lx = b.x + b.width - UI_PAD - legend_dots + 14;
        for (int s = 0; s < p->nseries; s++, lx += 22) {
            ui_dot(lx + 4, b.y + 22, 4.5f, p->colors[s]);
        }
    }

    return 0;
}

void plot_draw(Plot *p, Rectangle b)
{
    ui_card(b, NULL);
    float extra = draw_header(p, b); /* a wrapped legend pushes the plot down */

    Rectangle a = {b.x + 14 + 44, b.y + 46 + extra, b.width - 14 - 44 - 16,
                   b.height - 46 - extra - 14 - 22};
    if (a.width < 60 || a.height < 40) {
        return;
    }

    ui_rrect(ui_inset(a, -2), 6, UI_FIELD);

    /* ---- x range ---- */
    double x0, x1;
    if (p->live) {
        if (p->count > 0) {
            x0 = p->t[0];
            x1 = p->t[p->count - 1];
        } else {
            x0 = 0;
            x1 = 1;
        }
    } else {
        handle_view_input(p, a);
        x0 = p->view->x0;
        x1 = p->view->x1;
    }

    if (x1 - x0 < 1e-6) {
        x1 = x0 + 1.0;
    }

    /* ---- visible sample range ---- */
    int i0 = 0, i1 = p->count - 1;
    if (!p->live && p->count > 0) {
        i0 = lower_bound(p->t, p->count, x0) - 1;
        i1 = lower_bound(p->t, p->count, x1);
        if (i0 < 0) {
            i0 = 0;
        }

        if (i1 > p->count - 1) {
            i1 = p->count - 1;
        }
    }

    /* ---- y range ---- */
    double lo = DBL_MAX, hi = -DBL_MAX;
    for (int s = 0; s < p->nseries; s++) {
        if (!p->y[s]) {
            continue;
        }

        for (int i = i0; i <= i1; i++) {
            double v = p->y[s][i];
            if (isnan(v)) {
                continue;
            }

            if (v < lo) {
                lo = v;
            }

            if (v > hi) {
                hi = v;
            }
        }
    }

    double ylo, yhi;
    if (lo > hi) {
        ylo = -1;
        yhi = 1;
    } else {
        plot_pick_y_range(lo, hi, &ylo, &yhi);
    }

    if (yhi - ylo < 1e-12) {
        yhi = ylo + 1.0;
    }

#define PX(xv) (a.x + (float)(((xv) - x0) / (x1 - x0)) * a.width)
#define PY(yv) (a.y + a.height - (float)(((yv) - ylo) / (yhi - ylo)) * a.height)

    /* ---- grid + tick labels ---- */
    char label[32];

    double ystep = nice_step((yhi - ylo) / (a.height / 44.0));
    int guard = 0;
    for (double v = ceil(ylo / ystep) * ystep; v <= yhi && guard < 200; v += ystep, guard++) {
        float py = PY(v);
        DrawLine((int)a.x, (int)py, (int)(a.x + a.width), (int)py, UI_GRID);
        format_tick(label, sizeof(label), v, ystep);
        ui_text_right(label, a.x - 8, py - 7, UI_SMALL, UI_MUTED);
    }

    double xstep = nice_step((x1 - x0) / (a.width / 90.0));
    guard = 0;
    for (double v = ceil(x0 / xstep) * xstep; v <= x1 && guard < 200; v += xstep, guard++) {
        float px = PX(v);
        DrawLine((int)px, (int)a.y, (int)px, (int)(a.y + a.height), UI_GRID);
        format_tick(label, sizeof(label), v, xstep);
        ui_text(label, px - ui_text_width(label, UI_SMALL) / 2, a.y + a.height + 6, UI_SMALL,
                UI_MUTED);
    }

    ui_text_right("time (s)", b.x + b.width - 16, b.y + b.height - 20, UI_SMALL - 1, UI_MUTED);

    /* ---- series ---- */
    BeginScissorMode((int)a.x, (int)a.y, (int)a.width, (int)a.height);

    if (!p->live) {
        for (int e = 0; e < p->nevents; e++) {
            double when = p->events[e].t;
            if (when < x0 || when > x1) {
                continue;
            }

            float px = PX(when);
            for (float yy = a.y; yy < a.y + a.height; yy += 8) {
                DrawLine((int)px, (int)yy, (int)px, (int)(yy + 4), UI_MUTED);
            }

            ui_text(p->events[e].text, px + 4, a.y + 4 + (float)(e % 3) * 14, 12, UI_MUTED);
        }
    }

    int visible = i1 - i0 + 1;
    int stride = visible > (int)(a.width * 2) ? visible / (int)(a.width * 2) : 1;
    for (int s = 0; s < p->nseries; s++) {
        if (!p->y[s] || visible < 2) {
            continue;
        }

        Vector2 prev = {PX(p->t[i0]), PY(p->y[s][i0])};
        bool prev_ok = !isnan(p->y[s][i0]);
        for (int i = i0 + stride; i <= i1 + stride - 1; i += stride) {
            int idx = i > i1 ? i1 : i;
            float yv = p->y[s][idx];
            bool cur_ok = !isnan(yv);
            Vector2 cur = {PX(p->t[idx]), cur_ok ? PY(yv) : 0.0f};
            /* A NaN sample is a gap: nothing is drawn to or from it. */
            if (prev_ok && cur_ok) {
                DrawLineEx(prev, cur, 2.2f, p->colors[s]);
            }

            prev = cur;
            prev_ok = cur_ok;
        }
    }

    EndScissorMode();

    /* ---- hover read-out: nearest sample under the cursor ---- */
    Vector2 m = GetMousePosition();
    if (!GuiIsLocked() && !p->dragging && CheckCollisionPointRec(m, a) && visible >= 2) {
        double tx = x0 + (double)(m.x - a.x) / a.width * (x1 - x0);
        int near = lower_bound(p->t, p->count, tx);
        if (near > i1) {
            near = i1;
        }

        if (near > i0 && fabs(p->t[near - 1] - tx) < fabs(p->t[near] - tx)) {
            near--;
        }

        if (near < i0) {
            near = i0;
        }

        float px = PX(p->t[near]);
        DrawLine((int)px, (int)a.y, (int)px, (int)(a.y + a.height), Fade(UI_TEXT, 0.35f));

        char rows[PLOT_MAX_SERIES + 1][64];
        snprintf(rows[0], sizeof(rows[0]), "t = %.3f s", p->t[near]);
        float bw = ui_text_width(rows[0], UI_SMALL);
        for (int s = 0; s < p->nseries; s++) {
            char shown[48];
            display_name(p->names[s], shown, sizeof(shown));
            if (p->y[s] && isnan(p->y[s][near])) {
                snprintf(rows[s + 1], sizeof(rows[0]), "%s  --", shown);
            } else {
                snprintf(rows[s + 1], sizeof(rows[0]), "%s  %.3f", shown,
                         p->y[s] ? (double)p->y[s][near] : 0.0);
            }

            bw = fmaxf(bw, ui_text_width(rows[s + 1], UI_SMALL));
        }

        float bh = (float)(p->nseries + 1) * 18 + 12;
        float bx = px + 12;
        if (bx + bw + 24 > a.x + a.width) {
            bx = px - 12 - bw - 24;
        }

        float by = a.y + 8;
        ui_rrect_bordered((Rectangle){bx, by, bw + 24, bh}, 6, Fade(UI_CARD_HI, 0.96f), UI_BORDER);
        for (int s = 0; s <= p->nseries; s++) {
            ui_text(rows[s], bx + 12, by + 7 + (float)s * 18, UI_SMALL,
                    s == 0 ? UI_MUTED : p->colors[s - 1]);
        }
    }

#undef PX
#undef PY
}
