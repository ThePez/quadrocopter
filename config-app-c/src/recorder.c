/*
 *****************************************************************************
 * File: recorder.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Session recording to CSV, and loading a recording back.
 *****************************************************************************
 */

/*
 * Session files. Writing: one CSV row per telemetry sample (time since recording started, then one
 * column per `enum Field`), plus "# event" comment lines that mark moments such as a PID send.
 * Loading: the inverse, into column arrays that the session plots point at directly.
 */

#include "recorder.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "util.h"

#define FLUSH_EVERY 50 /* rows between flushes: a crash loses about a second */

bool recorder_active(const Recorder *r)
{
    return r->file != NULL;
}

int recorder_start(Recorder *r, const char *dir, char *err, size_t errlen)
{
    recorder_stop(r);

    if (mkdir(dir, 0755) != 0 && errno != EEXIST) {
        snprintf(err, errlen, "%s: %s", dir, strerror(errno));
        return -1;
    }

    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char stamp[32];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);
    snprintf(r->path, sizeof(r->path), "%s/session_%s.csv", dir, stamp);

    r->file = fopen(r->path, "w");
    if (!r->file) {
        snprintf(err, errlen, "%s: %s", r->path, strerror(errno));
        return -1;
    }

    strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tmv);
    fprintf(r->file, "# started,%s\n", stamp);
    fprintf(r->file, "time_s");
    for (int i = 0; i < F_COUNT; i++) {
        fprintf(r->file, ",%s", FIELD_NAMES[i]);
    }

    fputc('\n', r->file);

    r->start = now_s();
    r->unflushed = 0;
    r->rows = 0;
    return 0;
}

void recorder_add(Recorder *r, const Sample *s)
{
    if (!r->file) {
        return;
    }

    fprintf(r->file, "%.4f", s->t - r->start);
    for (int i = 0; i < F_COUNT; i++) {
        fprintf(r->file, ",%.7g", (double)s->v[i]);
    }

    fputc('\n', r->file);
    r->rows++;

    if (++r->unflushed >= FLUSH_EVERY) {
        fflush(r->file);
        r->unflushed = 0;
    }
}

void recorder_event(Recorder *r, const char *text)
{
    if (!r->file) {
        return;
    }

    char clean[256];
    str_copy(clean, sizeof(clean), text);
    for (char *p = clean; *p; p++) {
        if (*p == '\n' || *p == '\r') {
            *p = ' ';
        }
    }

    fprintf(r->file, "# event,%.4f,%s\n", now_s() - r->start, clean);
    fflush(r->file);
}

bool recorder_stop(Recorder *r)
{
    if (!r->file) {
        return false;
    }

    fclose(r->file);
    r->file = NULL;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Loading                                                                   */
/* ------------------------------------------------------------------------ */

static bool grow(Session *s, int *cap)
{
    int ncap = *cap ? *cap * 2 : 1024;
    double *t = realloc(s->t, (size_t)ncap * sizeof(*t));
    if (!t) {
        return false;
    }

    s->t = t;
    for (int i = 0; i < F_COUNT; i++) {
        if (!s->col[i]) {
            continue;
        }

        float *c = realloc(s->col[i], (size_t)ncap * sizeof(*c));
        if (!c) {
            return false;
        }

        s->col[i] = c;
    }

    *cap = ncap;
    return true;
}

void session_free(Session *s)
{
    free(s->t);
    for (int i = 0; i < F_COUNT; i++) {
        free(s->col[i]);
    }

    free(s->events);
    memset(s, 0, sizeof(*s));
}

/* Splits `line` in place on commas. Returns the number of cells. */
static int split_csv(char *line, char **cells, int max)
{
    int n = 0;
    char *p = line;
    while (n < max) {
        cells[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }

        *comma = '\0';
        p = comma + 1;
    }

    return n;
}

int session_load(Session *s, const char *path, char *err, size_t errlen)
{
    memset(s, 0, sizeof(*s));
    str_copy(s->path, sizeof(s->path), path);

    FILE *f = fopen(path, "r");
    if (!f) {
        snprintf(err, errlen, "%s: %s", path, strerror(errno));
        return -1;
    }

    enum {
        MAX_CELLS = 64
    };
    char line[4096];
    char *cells[MAX_CELLS];
    int col_map[MAX_CELLS]; /* CSV column -> enum Field, -1 for time_s, -2 unknown */
    int ncols = 0;
    bool have_header = false;
    int cap = 0, ev_cap = 0;

    while (fgets(line, sizeof(line), f)) {
        line[strcspn(line, "\r\n")] = '\0';
        char *p = line;
        while (*p == ' ' || *p == '\t') {
            p++;
        }

        if (*p == '\0') {
            continue;
        }

        if (*p == '#') {
            /* "# event,<time>,<text>" */
            p++;
            while (*p == ' ') {
                p++;
            }

            if (strncmp(p, "event,", 6) == 0) {
                char *comma = strchr(p + 6, ',');
                if (comma) {
                    *comma = '\0';
                    char *end;
                    double when = strtod(p + 6, &end);
                    if (end != p + 6) {
                        if (s->nevents == ev_cap) {
                            ev_cap = ev_cap ? ev_cap * 2 : 16;
                            SessionEvent *ne = realloc(s->events, (size_t)ev_cap * sizeof(*ne));
                            if (!ne) {
                                break;
                            }

                            s->events = ne;
                        }

                        s->events[s->nevents].t = when;
                        str_copy(s->events[s->nevents].text, sizeof(s->events[0].text), comma + 1);
                        s->nevents++;
                    }
                }
            }

            continue;
        }

        int n = split_csv(p, cells, MAX_CELLS);

        if (!have_header) {
            have_header = true;
            ncols = n;
            bool saw_time = false;
            for (int i = 0; i < n; i++) {
                col_map[i] = -2;
                if (strcmp(cells[i], "time_s") == 0) {
                    col_map[i] = -1;
                    saw_time = true;
                    continue;
                }

                for (int fld = 0; fld < F_COUNT; fld++) {
                    if (strcmp(cells[i], FIELD_NAMES[fld]) == 0) {
                        col_map[i] = fld;
                        if (!s->col[fld]) {
                            s->col[fld] = malloc(sizeof(float)); /* mark present; grown below */
                        }
                    }
                }
            }

            if (!saw_time) {
                snprintf(err, errlen, "not a telemetry session file (no time_s column)");
                fclose(f);
                session_free(s);
                return -1;
            }

            continue;
        }

        if (n != ncols) {
            continue; /* row cut short by a crash */
        }

        double vals[MAX_CELLS];
        bool ok = true;
        for (int i = 0; i < n && ok; i++) {
            char *end;
            vals[i] = strtod(cells[i], &end);
            ok = end != cells[i] && *end == '\0';
        }

        if (!ok) {
            continue;
        }

        if (s->count == cap && !grow(s, &cap)) {
            snprintf(err, errlen, "out of memory");
            fclose(f);
            session_free(s);
            return -1;
        }

        for (int i = 0; i < n; i++) {
            if (col_map[i] == -1) {
                s->t[s->count] = vals[i];
            } else if (col_map[i] >= 0) {
                s->col[col_map[i]][s->count] = (float)vals[i];
            }
        }

        s->count++;
    }

    fclose(f);

    if (!have_header) {
        snprintf(err, errlen, "not a telemetry session file");
        session_free(s);
        return -1;
    }

    s->duration = s->count ? s->t[s->count - 1] : 0.0;
    return 0;
}
