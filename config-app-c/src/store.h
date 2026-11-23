/*
 *****************************************************************************
 * File: store.h
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Interface for the key/value tables and the defaults files.
 *****************************************************************************
 */

/*
 * Firmware defaults (#define constants read from the firmware's *_defaults.h)
 * and the GUI's own saved overrides (config-app/user_defaults.json - the same
 * file the Python app uses, so saved values are shared between the two).
 */
#ifndef STORE_H
#define STORE_H

#include <stdbool.h>
#include <stddef.h>

#define KV_NAME_MAX 48
#define KV_MAX      96

typedef struct {
    char name[KV_NAME_MAX];
    double val;
} KV;

typedef struct {
    KV items[KV_MAX];
    int n;
} KVSet;

double kv_get(const KVSet *s, const char *name, double fallback);
bool kv_has(const KVSet *s, const char *name);
bool kv_set(KVSet *s, const char *name, double val);
void kv_merge(KVSet *dst, const KVSet *src);

/* Reads every `#define NAME <number>` from a C header (numeric literals and
 * references to earlier numeric macros; anything else is skipped). Returns 0,
 * or -1 if the file can't be read. */
int defines_load(const char *path, KVSet *out);

/* ---- user_defaults.json: { "section": { "KEY": number, ... }, ... } ---- */

#define OVERRIDE_SECTIONS_MAX 8

typedef struct {
    char name[24];
    KVSet kv;
} OverrideSection;

typedef struct {
    OverrideSection sec[OVERRIDE_SECTIONS_MAX];
    int n;
} Overrides;

/* A missing file is fine (empty result). A file that exists but isn't the
 * expected shape returns -1 so callers don't overwrite it. */
int overrides_load(Overrides *o, const char *path, char *err, size_t errlen);
int overrides_save(const Overrides *o, const char *path, char *err, size_t errlen);
KVSet *overrides_section(Overrides *o, const char *name, bool create);

#endif
