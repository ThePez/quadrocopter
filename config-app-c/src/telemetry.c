/*
 *****************************************************************************
 * File: telemetry.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Serial reader thread and hand-off of decoded samples and acks to the UI.
 *****************************************************************************
 */

/*
 * The serial reader thread and its hand-off to the UI.
 *
 *   reader thread:  read() -> decoder_feed() -> on_frame()/on_ack() -> push into ring buffers
 *   main thread:    telemetry_poll() / telemetry_poll_acks() -> pop from the ring buffers
 *
 * One mutex guards both ring buffers and the "lost" flag. The buffers are fixed-size; if the UI
 * stalls the oldest entries are dropped rather than blocking the reader or growing memory.
 */

#include "telemetry.h"

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "uart.h"
#include "util.h"

#define QUEUE_CAP 2048
#define ACK_CAP   32

const char *const FIELD_NAMES[F_COUNT] = {
    "pitch_angle",    "roll_angle", "yaw_angle", "pitch_rate", "roll_rate",
    "yaw_rate",       "mode",       "pitch_pid", "roll_pid",   "yaw_pid",
    "motor_fl",       "motor_bl",   "motor_br",  "motor_fr",   "battery",
    "loop_dt_max_us", "cmd_pitch",  "cmd_roll",  "cmd_yaw",    "cmd_throttle",
};

void sample_from_wire(Sample *s, const struct sensor_telemetry_t *w, double t)
{
    s->t = t;
    s->v[F_PITCH_ANGLE] = w->pitch_angle;
    s->v[F_ROLL_ANGLE] = w->roll_angle;
    s->v[F_YAW_ANGLE] = w->yaw_angle;
    s->v[F_PITCH_RATE] = w->pitch_rate;
    s->v[F_ROLL_RATE] = w->roll_rate;
    s->v[F_YAW_RATE] = w->yaw_rate;
    s->v[F_MODE] = w->mode;
    s->v[F_PITCH_PID] = w->pid_pitch;
    s->v[F_ROLL_PID] = w->pid_roll;
    s->v[F_YAW_PID] = w->pid_yaw;
    s->v[F_MOTOR_FL] = w->motor_a;
    s->v[F_MOTOR_BL] = w->motor_b;
    s->v[F_MOTOR_BR] = w->motor_c;
    s->v[F_MOTOR_FR] = w->motor_d;
    s->v[F_BATTERY] = w->battery;
    s->v[F_LOOP_DT_MAX_US] = w->loop_dt_max_us;
    s->v[F_CMD_PITCH] = w->cmd_pitch;
    s->v[F_CMD_ROLL] = w->cmd_roll;
    s->v[F_CMD_YAW] = w->cmd_yaw;
    s->v[F_CMD_THROTTLE] = w->cmd_throttle;
}

struct Telemetry {
    int fd;
    pthread_t thread;
    atomic_bool running;
    FrameDecoder decoder;

    pthread_mutex_t mutex; /* guards everything below */
    Sample queue[QUEUE_CAP];
    int head, count;
    AckEvent acks[ACK_CAP];
    int ack_head, ack_count;
    bool lost;
    char err[160];
};

static void on_frame(const struct sensor_telemetry_t *payload, void *ctx)
{
    Telemetry *t = ctx;
    Sample s;
    sample_from_wire(&s, payload, now_s());

    pthread_mutex_lock(&t->mutex);
    if (t->count == QUEUE_CAP) { /* UI stalled: drop the oldest */
        t->head = (t->head + 1) % QUEUE_CAP;
        t->count--;
    }

    t->queue[(t->head + t->count) % QUEUE_CAP] = s;
    t->count++;
    pthread_mutex_unlock(&t->mutex);
}

static void on_ack(const struct wifi_packet_t *pkt, void *ctx)
{
    Telemetry *t = ctx;
    AckEvent e = {.id = pkt->packet_id, .status = pkt->data.result.status};

    pthread_mutex_lock(&t->mutex);
    if (t->ack_count == ACK_CAP) { /* UI stalled: drop the oldest */
        t->ack_head = (t->ack_head + 1) % ACK_CAP;
        t->ack_count--;
    }

    t->acks[(t->ack_head + t->ack_count) % ACK_CAP] = e;
    t->ack_count++;
    pthread_mutex_unlock(&t->mutex);
}

/* Thread body. read() has a 100 ms timeout (set in uart_open) so the loop notices `running`
 * going false promptly; a real read error marks the link lost and ends the thread. */
static void *reader_main(void *arg)
{
    Telemetry *t = arg;
    uint8_t buf[4096];

    while (atomic_load(&t->running)) {
        ssize_t n = read(t->fd, buf, sizeof(buf));
        if (n > 0) {
            decoder_feed(&t->decoder, buf, (size_t)n, on_frame, on_ack, t);
        } else if (n < 0 && errno != EINTR && errno != EAGAIN) {
            pthread_mutex_lock(&t->mutex);
            t->lost = true;
            snprintf(t->err, sizeof(t->err), "%s", strerror(errno));
            pthread_mutex_unlock(&t->mutex);
            break;
        }

        /* n == 0: the 100 ms read timeout - loop and re-check `running`. */
    }

    return NULL;
}

Telemetry *telemetry_start(int fd)
{
    Telemetry *t = calloc(1, sizeof(*t));
    if (!t) {
        return NULL;
    }

    t->fd = fd;
    decoder_init(&t->decoder);
    pthread_mutex_init(&t->mutex, NULL);
    atomic_init(&t->running, true);

    if (pthread_create(&t->thread, NULL, reader_main, t) != 0) {
        pthread_mutex_destroy(&t->mutex);
        free(t);
        return NULL;
    }

    return t;
}

void telemetry_stop(Telemetry *t)
{
    if (!t) {
        return;
    }

    atomic_store(&t->running, false);
    pthread_join(t->thread, NULL);
    pthread_mutex_destroy(&t->mutex);
    free(t);
}

int telemetry_poll(Telemetry *t, Sample *out, int max)
{
    int n = 0;
    pthread_mutex_lock(&t->mutex);
    while (t->count > 0 && n < max) {
        out[n++] = t->queue[t->head];
        t->head = (t->head + 1) % QUEUE_CAP;
        t->count--;
    }

    pthread_mutex_unlock(&t->mutex);
    return n;
}

int telemetry_poll_acks(Telemetry *t, AckEvent *out, int max)
{
    int n = 0;
    pthread_mutex_lock(&t->mutex);
    while (t->ack_count > 0 && n < max) {
        out[n++] = t->acks[t->ack_head];
        t->ack_head = (t->ack_head + 1) % ACK_CAP;
        t->ack_count--;
    }

    pthread_mutex_unlock(&t->mutex);
    return n;
}

bool telemetry_lost(Telemetry *t, char *err, size_t errlen)
{
    bool lost;
    pthread_mutex_lock(&t->mutex);
    lost = t->lost;
    if (lost) {
        snprintf(err, errlen, "%s", t->err);
        t->lost = false;
    }

    pthread_mutex_unlock(&t->mutex);
    return lost;
}
