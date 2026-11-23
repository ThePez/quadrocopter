/*
 *****************************************************************************
 * File: uart.h
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Interface for the serial port and frame protocol.
 *****************************************************************************
 */

/*
 * Serial link to the bridge ESP32.
 *
 * One frame shape in both directions (mirrors bridge/main/bridge.c):
 *   [0xAA 0x55][struct wifi_packet_t]   (the packet's crc16 lives inside it)
 *
 * What the packet is comes from its packet_id:
 *   bridge -> GUI : SENSOR (data.sensor is the telemetry), or a config ack
 *                   PID_ACK / DRONE_CFG_ACK / REMOTE_CFG_ACK (data.result.status, 0 = OK).
 *                   The bridge forwards the packet exactly as it received it.
 *   GUI -> bridge : PID_CONFIG, DRONE_CFG, DRONE_CFG_STORE, REMOTE_CFG, REMOTE_CFG_STORE.
 *
 * Because the struct comes from protocol.h, the wire layout is whatever the
 * compiler says it is - the same as the firmware - so nothing is hand-parsed.
 */
#ifndef UART_H
#define UART_H

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

#define UART_MAGIC0 0xAA
#define UART_MAGIC1 0x55

#define UART_MAX_PORTS 32

/* [magic x2][struct wifi_packet_t] */
#define UART_FRAME_SIZE        (2 + sizeof(struct wifi_packet_t))
#define UART_CONFIG_FRAME_SIZE UART_FRAME_SIZE /* GUI -> bridge: same frame */

typedef struct {
    char device[64];
    char description[64];
} PortInfo;

/* Fills `out` with detected USB serial ports (ttyUSB*, ttyACM*), sorted by name.
 * Returns the number found. */
int uart_list_ports(PortInfo *out, int max);

/* Opens `device` raw 8N1 at `baud` (9600/57600/115200/230400/460800/921600).
 * Returns the fd, or -1 with a message in `err`. Reads time out after 100 ms. */
int uart_open(const char *device, int baud, char *err, size_t errlen);
void uart_close(int fd);

/* Writes every byte and waits for them to drain. Returns 0 or -1 (errno set). */
int uart_write_all(int fd, const uint8_t *data, size_t len);

/* CRC-16/X-25, matching esp_rom_crc16_le(0, ...) on the firmware. */
uint16_t uart_crc16(const uint8_t *data, size_t len);

/* Builds a GUI -> bridge frame carrying `payload` (zero-padded into the packet
 * union). Returns the frame length, or 0 if it doesn't fit. */
size_t uart_encode_config_packet(uint8_t packet_id, const void *payload, size_t payload_len,
                                 uint8_t *out, size_t out_cap);

/* ---- Telemetry frame decoder ---- */

typedef void (*TelemetryFrameCb)(const struct sensor_telemetry_t *payload, void *ctx);
typedef void (*AckFrameCb)(const struct wifi_packet_t *packet, void *ctx);

typedef struct {
    uint8_t buf[2 * UART_FRAME_SIZE + 256];
    size_t len;
} FrameDecoder;

void decoder_init(FrameDecoder *d);

/* Feeds raw bytes and invokes `cb` once per CRC-valid SENSOR frame and `ack_cb` (may be
 * NULL) once per CRC-valid ack frame; any other packet id is consumed and ignored. Resyncs
 * after corruption by discarding only the two magic bytes of a bad candidate frame.
 * Returns the number of valid frames decoded. */
int decoder_feed(FrameDecoder *d, const uint8_t *data, size_t n, TelemetryFrameCb cb,
                 AckFrameCb ack_cb, void *ctx);

#endif
