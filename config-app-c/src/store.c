/*
 *****************************************************************************
 * File: store.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Firmware default scraping and the saved user_defaults.json overrides.
 *****************************************************************************
 */

/*
 * Small key/value tables and the two places values come from:
 *
 *   1. #define scraping: reads `#define NAME number` lines out of the firmware's *_defaults.h,
 *      so the GUI starts from the same numbers the firmware was compiled with.
 *   2. user_defaults.json: a hand-rolled reader/writer for { "section": { "KEY": number } }.
 *      Only that exact shape is supported; anything else is reported as an error and never
 *      overwritten.
 *
 * KVSet is a fixed-size linear table (KV_MAX entries). That is plenty for a few dozen defaults
 * and avoids any allocation.
 */

#include "store.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

/* ------------------------------------------------------------------------ */
/* KVSet                                                                     */
/* ------------------------------------------------------------------------ */

static KV *kv_find(const KVSet *s, const char *name)
{
    for (int i = 0; i < s->n; i++) {
        if (strcmp(s->items[i].name, name) == 0) {
            return (KV *)&s->items[i];
        }
    }

    return NULL;
}

bool kv_has(const KVSet *s, const char *name)
{
    return kv_find(s, name) != NULL;
}

double kv_get(const KVSet *s, const char *name, double fallback)
{
    const KV *kv = kv_find(s, name);
    return kv ? kv->val : fallback;
}

bool kv_set(KVSet *s, const char *name, double val)
{
    KV *kv = kv_find(s, name);
    if (!kv) {
        if (s->n >= KV_MAX) {
            return false;
        }

        kv = &s->items[s->n++];
        str_copy(kv->name, sizeof(kv->name), name);
    }

    kv->val = val;
    return true;
}

void kv_merge(KVSet *dst, const KVSet *src)
{
    for (int i = 0; i < src->n; i++) {
        kv_set(dst, src->items[i].name, src->items[i].val);
    }
}

/* ------------------------------------------------------------------------ */
/* #define scraping                                                          */
/* ------------------------------------------------------------------------ */

/* Parses a whole token as a C numeric literal (optional f/F/u/U/l/L suffix). */
static bool parse_literal(const char *tok, double *out)
{
    char *end;
    errno = 0;
    double v = strtod(tok, &end);
    if (end == tok) {
        return false;
    }

    while (*end == 'f' || *end == 'F' || *end == 'u' || *end == 'U' || *end == 'l' || *end == 'L') {
        end++;
    }

    if (*end != '\0') {
        return false;
    }

    *out = v;
    return true;
}

int defines_load(const char *path, KVSet *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        const char *p = line;
        while (isspace((unsigned char)*p)) {
            p++;
        }

        if (strncmp(p, "#define", 7) != 0 || !isspace((unsigned char)p[7])) {
            continue;
        }

        p += 7;
        while (isspace((unsigned char)*p)) {
            p++;
        }

        char name[KV_NAME_MAX];
        size_t n = 0;
        while ((isalnum((unsigned char)*p) || *p == '_') && n + 1 < sizeof(name)) {
            name[n++] = *p++;
        }

        name[n] = '\0';
        if (n == 0 || *p == '(') { /* function-like macro */
            continue;
        }

        while (isspace((unsigned char)*p)) {
            p++;
        }

        /* Value token: up to whitespace or the start of a comment. */
        char tok[64];
        size_t t = 0;
        while (*p && !isspace((unsigned char)*p) &&
               !(p[0] == '/' && (p[1] == '/' || p[1] == '*')) && t + 1 < sizeof(tok)) {
            tok[t++] = *p++;
        }

        tok[t] = '\0';
        if (t == 0) {
            continue;
        }

        double v;
        if (parse_literal(tok, &v)) {
            kv_set(out, name, v);
        } else {
            /* Reference to an earlier macro, optionally negated. */
            bool neg = tok[0] == '-';
            const char *ref = neg ? tok + 1 : tok;
            if (kv_has(out, ref)) {
                kv_set(out, name, neg ? -kv_get(out, ref, 0.0) : kv_get(out, ref, 0.0));
            }
        }
    }

    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* user_defaults.json                                                        */
/* ------------------------------------------------------------------------ */

typedef struct {
    const char *p;
    const char *end;
} Cursor;

static void skip_ws(Cursor *c)
{
    while (c->p < c->end && isspace((unsigned char)*c->p)) {
        c->p++;
    }
}

static bool expect(Cursor *c, char ch)
{
    skip_ws(c);
    if (c->p < c->end && *c->p == ch) {
        c->p++;
        return true;
    }

    return false;
}

static bool peek_is(Cursor *c, char ch)
{
    skip_ws(c);
    return c->p < c->end && *c->p == ch;
}

static bool parse_string(Cursor *c, char *out, size_t cap)
{
    if (!expect(c, '"')) {
        return false;
    }

    size_t n = 0;
    while (c->p < c->end && *c->p != '"') {
        char ch = *c->p++;
        if (ch == '\\' && c->p < c->end) {
            ch = *c->p++; /* keys never contain anything fancier than \" or \\ */
        }

        if (n + 1 < cap) {
            out[n++] = ch;
        }
    }

    if (c->p >= c->end) {
        return false;
    }

    c->p++; /* closing quote */
    out[n] = '\0';
    return true;
}

static bool parse_number(Cursor *c, double *out)
{
    skip_ws(c);
    char tmp[64];
    size_t n = 0;
    while (c->p < c->end && (isdigit((unsigned char)*c->p) || strchr("+-.eE", *c->p)) &&
           n + 1 < sizeof(tmp)) {
        tmp[n++] = *c->p++;
    }

    tmp[n] = '\0';
    if (n == 0) {
        return false;
    }

    char *end;
    *out = strtod(tmp, &end);
    return *end == '\0';
}

KVSet *overrides_section(Overrides *o, const char *name, bool create)
{
    for (int i = 0; i < o->n; i++) {
        if (strcmp(o->sec[i].name, name) == 0) {
            return &o->sec[i].kv;
        }
    }

    if (!create || o->n >= OVERRIDE_SECTIONS_MAX) {
        return NULL;
    }

    OverrideSection *s = &o->sec[o->n++];
    memset(s, 0, sizeof(*s));
    str_copy(s->name, sizeof(s->name), name);
    return &s->kv;
}

int overrides_load(Overrides *o, const char *path, char *err, size_t errlen)
{
    memset(o, 0, sizeof(*o));

    FILE *f = fopen(path, "rb");
    if (!f) {
        if (errno == ENOENT) {
            return 0;
        }

        snprintf(err, errlen, "%s: %s", path, strerror(errno));
        return -1;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *text = malloc((size_t)size + 1);
    if (!text) {
        fclose(f);
        snprintf(err, errlen, "out of memory");
        return -1;
    }

    size_t got = fread(text, 1, (size_t)size, f);
    fclose(f);

    Cursor c = {text, text + got};
    bool ok = expect(&c, '{');

    while (ok && !peek_is(&c, '}')) {
        char section[sizeof(o->sec[0].name)];
        ok = parse_string(&c, section, sizeof(section)) && expect(&c, ':') && expect(&c, '{');
        if (!ok) {
            break;
        }

        KVSet *kv = overrides_section(o, section, true);
        ok = kv != NULL;
        while (ok && !peek_is(&c, '}')) {
            char key[KV_NAME_MAX];
            double val;
            ok = parse_string(&c, key, sizeof(key)) && expect(&c, ':') && parse_number(&c, &val);
            if (!ok) {
                break;
            }

            kv_set(kv, key, val);
            if (!peek_is(&c, '}')) {
                ok = expect(&c, ',');
            }
        }

        ok = ok && expect(&c, '}');
        if (ok && !peek_is(&c, '}')) {
            ok = expect(&c, ',');
        }
    }

    ok = ok && expect(&c, '}');

    free(text);
    if (!ok) {
        snprintf(err, errlen, "%s is not a {section:{key:number}} JSON file", path);
        memset(o, 0, sizeof(*o));
        return -1;
    }

    return 0;
}

static int cmp_kv(const void *a, const void *b)
{
    return strcmp(((const KV *)a)->name, ((const KV *)b)->name);
}

static int cmp_sec(const void *a, const void *b)
{
    return strcmp(((const OverrideSection *)a)->name, ((const OverrideSection *)b)->name);
}

int overrides_save(const Overrides *o, const char *path, char *err, size_t errlen)
{
    /* Sorted like json.dump(sort_keys=True) so the Python app and this one
     * produce near-identical files. */
    Overrides *sorted = malloc(sizeof(*sorted));
    if (!sorted) {
        snprintf(err, errlen, "out of memory");
        return -1;
    }

    *sorted = *o;
    qsort(sorted->sec, (size_t)sorted->n, sizeof(sorted->sec[0]), cmp_sec);

    char tmp_path[600];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    FILE *f = fopen(tmp_path, "wb");
    if (!f) {
        snprintf(err, errlen, "%s: %s", tmp_path, strerror(errno));
        free(sorted);
        return -1;
    }

    fprintf(f, "{\n");
    for (int i = 0; i < sorted->n; i++) {
        OverrideSection *s = &sorted->sec[i];
        qsort(s->kv.items, (size_t)s->kv.n, sizeof(s->kv.items[0]), cmp_kv);
        fprintf(f, "  \"%s\": {\n", s->name);
        for (int k = 0; k < s->kv.n; k++) {
            fprintf(f, "    \"%s\": %.10g%s\n", s->kv.items[k].name, s->kv.items[k].val,
                    k + 1 < s->kv.n ? "," : "");
        }

        fprintf(f, "  }%s\n", i + 1 < sorted->n ? "," : "");
    }

    fprintf(f, "}\n");
    free(sorted);

    if (fclose(f) != 0 || rename(tmp_path, path) != 0) {
        snprintf(err, errlen, "%s: %s", path, strerror(errno));
        remove(tmp_path);
        return -1;
    }

    return 0;
}
