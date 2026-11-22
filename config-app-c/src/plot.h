/*
 * Line plot used for both live telemetry and recorded sessions.
 *
 *  - Live plots own a fixed-size sliding buffer and always show all of it.
 *  - Static plots point at someone else's arrays (a loaded Session) and share a
 *    PlotView, so zooming/panning time on one tab moves all of them.
 *    Scroll = zoom, drag = pan, double-click = reset.
 */
#ifndef PLOT_H
#define PLOT_H

#include <stdbool.h>

#include "raylib.h"
#include "recorder.h"

#define PLOT_MAX_SERIES 8

typedef struct {
    double x0, x1;       /* visible time range */
    double full0, full1; /* range to return to on reset */
} PlotView;

typedef struct {
    const char *title;
    const char *ylabel;
    const char *xlabel;

    int nseries;
    const char *names[PLOT_MAX_SERIES];
    Color colors[PLOT_MAX_SERIES];

    /* Data. For live plots these are owned; for static plots they alias. */
    bool live;
    double *t;
    float *y[PLOT_MAX_SERIES]; /* a NULL series is skipped; NaN values leave a gap */
    int count, cap;

    /* Live-plot controls */
    bool paused;
    double t0; /* time zero for x (set by plot_clear) */

    /* Static-plot view + markers */
    PlotView *view;
    const SessionEvent *events;
    int nevents;

    bool dragging;
    double last_press;
} Plot;

void plot_init_live(Plot *p, const char *title, const char *ylabel, int nseries,
                    const char *const *names, const Color *colors, int capacity);
void plot_init_static(Plot *p, const char *title, const char *ylabel, int nseries,
                      const char *const *names, const Color *colors, PlotView *view);
void plot_free(Plot *p);

/* Live: append one point (x = now_s() - t0). No-op while paused. */
void plot_push(Plot *p, double t, const float *values);
void plot_clear(Plot *p);

/* Static: bind a series to external data (shared time axis, `count` points). */
void plot_bind(Plot *p, int series, const double *t, const float *y, int count);

void plot_draw(Plot *p, Rectangle bounds);

/* Chooses the y-axis range to display given the data's min/max. */
void plot_pick_y_range(double data_lo, double data_hi, double *out_lo, double *out_hi);

#endif
