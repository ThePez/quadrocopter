"""
Framing for the bridge <-> GUI UART link.

Two directions, two different frame shapes:
- Bridge -> GUI (telemetry): [0xAA 0x55][sensor_telemetry_t][crc16], written by
  uart_comm_write_frame() in bridge/main/bridge.c. Decoded here by FrameDecoder.
- GUI -> bridge (config updates): [0xAA 0x55][struct wifi_packet_t], read by
  uart_comm_read_frame() in bridge/main/bridge.c. wifi_packet_t already carries
  its own crc16 and packet_id fields, so there's no separate trailing crc for
  this direction - see encode_config_packet().
"""

import struct
from typing import List

from constants import WIFI_PACKET_SIZE, WIFI_PACKET_UNION_SIZE

MAGIC = bytes([0xAA, 0x55])
CRC_SIZE = 2


def crc16(data: bytes) -> int:
    """CRC-16/X-25 (poly 0x8408 reflected, init/xorout 0xFFFF) - matches esp_rom_crc16_le(0, ...)
    as called from uart_comm_write_frame()/uart_comm_read_frame() in bridge/main/bridge.c
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc ^ 0xFFFF


def frame_size(payload_size: int) -> int:
    return len(MAGIC) + payload_size + CRC_SIZE


def encode_config_packet(packet_id: int, payload: bytes = b"") -> bytes:
    """Builds the wire bytes for a GUI -> bridge config-update packet.

    Mirrors struct wifi_packet_t's layout: `payload` occupies the leading
    bytes of the data union (zero-padded out to WIFI_PACKET_UNION_SIZE),
    followed by a crc16 computed the same way esp_now_send_packet() does
    (over just the union bytes) and the packet_id. Wrapped with the magic
    sync bytes uart_comm_read_frame() expects.

    CONFIG_SAVE/REMOTE_CONFIG_SAVE have no payload - packet_id alone is the
    whole message, so the default empty payload is zero-padded as-is.
    """
    if len(payload) > WIFI_PACKET_UNION_SIZE:
        raise ValueError(
            f"payload of {len(payload)} bytes exceeds union size {WIFI_PACKET_UNION_SIZE}"
        )

    data = payload.ljust(WIFI_PACKET_UNION_SIZE, b"\x00")
    crc = crc16(data)
    packet = data + struct.pack("<H", crc) + bytes([packet_id])
    return MAGIC + packet.ljust(WIFI_PACKET_SIZE, b"\x00")


class FrameDecoder:
    """
    Feed raw serial bytes in, get back complete CRC-valid payloads.

    Resyncs after dropped/corrupt bytes by only discarding the 2 magic bytes
    of a candidate frame that fails its CRC check, rather than a whole frame's
    worth - the real frame boundary may not be where a coincidental magic-byte
    match suggested it was.
    """

    def __init__(self, payload_size: int) -> None:
        self.payload_size: int = payload_size
        self.frame_size: int = frame_size(payload_size)
        self.buffer: bytearray = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
        self.buffer.extend(data)
        payloads: List[bytes] = []

        while True:
            idx = self.buffer.find(MAGIC)
            if idx == -1:
                # Keep the last byte in case it's the start of a split magic sequence
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < self.frame_size:
                break

            payload = bytes(self.buffer[len(MAGIC) : len(MAGIC) + self.payload_size])
            (received_crc,) = struct.unpack(
                "<H",
                bytes(self.buffer[len(MAGIC) + self.payload_size : self.frame_size]),
            )

            if crc16(payload) != received_crc:
                del self.buffer[: len(MAGIC)]
                continue

            del self.buffer[: self.frame_size]
            payloads.append(payload)

        return payloads
