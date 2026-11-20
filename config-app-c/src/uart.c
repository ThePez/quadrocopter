#include "uart.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "util.h"

/* If these fire, protocol.h no longer matches what the bridge frames. */
_Static_assert(sizeof(struct sensor_telemetry_t) == 52, "sensor_telemetry_t size changed");
_Static_assert(sizeof(struct wifi_packet_t) == 56, "wifi_packet_t size changed");

uint16_t uart_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

static int cmp_ports(const void *a, const void *b)
{
    return strcmp(((const PortInfo *)a)->device, ((const PortInfo *)b)->device);
}

int uart_list_ports(PortInfo *out, int max)
{
    int n = 0;
    DIR *d = opendir("/dev");
    if (!d)
        return 0;

    struct dirent *e;
    while ((e = readdir(d)) != NULL && n < max) {
        if (strncmp(e->d_name, "ttyUSB", 6) != 0 && strncmp(e->d_name, "ttyACM", 6) != 0)
            continue;
        if (strlen(e->d_name) > 32) /* real tty names are short */
            continue;

        char name[40];
        str_copy(name, sizeof name, e->d_name);
        snprintf(out[n].device, sizeof out[n].device, "/dev/%s", name);
        out[n].description[0] = '\0';

        /* The USB device's "product" string sits one level above the tty's
         * interface directory. */
        char path[160];
        snprintf(path, sizeof path, "/sys/class/tty/%s/device/../product", name);
        FILE *f = fopen(path, "r");
        if (f) {
            if (fgets(out[n].description, sizeof out[n].description, f))
                out[n].description[strcspn(out[n].description, "\r\n")] = '\0';
            fclose(f);
        }
        n++;
    }
    closedir(d);

    qsort(out, (size_t)n, sizeof out[0], cmp_ports);
    return n;
}

static int baud_to_speed(int baud, speed_t *speed)
{
    switch (baud) {
    case 9600: *speed = B9600; return 0;
    case 57600: *speed = B57600; return 0;
    case 115200: *speed = B115200; return 0;
    case 230400: *speed = B230400; return 0;
    case 460800: *speed = B460800; return 0;
    case 921600: *speed = B921600; return 0;
    default: return -1;
    }
}

int uart_open(const char *device, int baud, char *err, size_t errlen)
{
    speed_t speed;
    if (baud_to_speed(baud, &speed) != 0) {
        snprintf(err, errlen, "unsupported baud rate %d", baud);
        return -1;
    }

    int fd = open(device, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) {
        snprintf(err, errlen, "%s: %s", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        snprintf(err, errlen, "%s: tcgetattr: %s", device, strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(tcflag_t)CRTSCTS;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cc[VMIN] = 0;  /* return whatever is there... */
    tio.c_cc[VTIME] = 1; /* ...or nothing after 100 ms, so the reader can check for shutdown */

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        snprintf(err, errlen, "%s: tcsetattr: %s", device, strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void uart_close(int fd)
{
    if (fd >= 0)
        close(fd);
}

int uart_write_all(int fd, const uint8_t *data, size_t len)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, data + off, len - off);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return -1;
        }
        off += (size_t)n;
    }
    tcdrain(fd);
    return 0;
}

size_t uart_encode_config_packet(uint8_t packet_id, const void *payload, size_t payload_len,
                                 uint8_t *out, size_t out_cap)
{
    if (payload_len > sizeof(union packet_data) || out_cap < UART_CONFIG_FRAME_SIZE)
        return 0;

    /* Zeroed so struct padding (and the unused tail of the union) is
     * deterministic on the wire. */
    struct wifi_packet_t pkt;
    memset(&pkt, 0, sizeof pkt);
    pkt.packet_id = packet_id;
    if (payload && payload_len)
        memcpy(&pkt.data, payload, payload_len);
    pkt.crc16 = uart_crc16((const uint8_t *)&pkt.data, sizeof pkt.data);

    out[0] = UART_MAGIC0;
    out[1] = UART_MAGIC1;
    memcpy(out + 2, &pkt, sizeof pkt);
    return UART_CONFIG_FRAME_SIZE;
}

void decoder_init(FrameDecoder *d)
{
    d->len = 0;
}

static int decoder_process(FrameDecoder *d, TelemetryFrameCb cb, void *ctx)
{
    const size_t payload = sizeof(struct sensor_telemetry_t);
    int frames = 0;

    for (;;) {
        size_t idx = 0;
        while (idx + 1 < d->len && !(d->buf[idx] == UART_MAGIC0 && d->buf[idx + 1] == UART_MAGIC1))
            idx++;

        if (idx + 1 >= d->len) {
            /* No complete magic pair. Keep the last byte - it may be the
             * first half of a split magic sequence. */
            if (d->len > 1) {
                d->buf[0] = d->buf[d->len - 1];
                d->len = 1;
            }
            break;
        }

        if (idx > 0) {
            memmove(d->buf, d->buf + idx, d->len - idx);
            d->len -= idx;
        }

        if (d->len < UART_TELEMETRY_FRAME_SIZE)
            break;

        uint16_t rx_crc = (uint16_t)(d->buf[2 + payload] | (d->buf[3 + payload] << 8));
        if (uart_crc16(d->buf + 2, payload) != rx_crc) {
            /* Only drop the magic bytes: the real frame boundary may be
             * somewhere inside what we just examined. */
            memmove(d->buf, d->buf + 2, d->len - 2);
            d->len -= 2;
            continue;
        }

        struct sensor_telemetry_t t;
        memcpy(&t, d->buf + 2, sizeof t);
        cb(&t, ctx);
        frames++;

        memmove(d->buf, d->buf + UART_TELEMETRY_FRAME_SIZE, d->len - UART_TELEMETRY_FRAME_SIZE);
        d->len -= UART_TELEMETRY_FRAME_SIZE;
    }
    return frames;
}

int decoder_feed(FrameDecoder *d, const uint8_t *data, size_t n, TelemetryFrameCb cb, void *ctx)
{
    int frames = 0;
    while (n > 0) {
        size_t room = sizeof d->buf - d->len;
        size_t take = n < room ? n : room;
        memcpy(d->buf + d->len, data, take);
        d->len += take;
        data += take;
        n -= take;
        frames += decoder_process(d, cb, ctx);
    }
    return frames;
}
