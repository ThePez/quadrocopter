"""
Framing for the bridge <-> GUI UART link: [0xAA 0x55][payload][crc16].
Mirrors uart_comm_write_frame()/uart_comm_read_frame() in bridge/main/bridge.c -
keep both in sync if this changes.
"""

import struct
from typing import List

MAGIC = bytes([0xAA, 0x55])
CRC_SIZE = 2


def crc16(data: bytes) -> int:
    """CRC-16/X-25 (poly 0x8408 reflected, init/xorout 0xFFFF) - matches esp_rom_crc16_le(0, ...)
    as called from uart_comm_write_frame()/uart_comm_read_frame() in bridge/main/bridge.c"""
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


def encode(payload: bytes) -> bytes:
    """Wrap a payload for sending: [magic][payload][crc16]"""
    return MAGIC + payload + struct.pack('<H', crc16(payload))


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

            payload = bytes(self.buffer[len(MAGIC):len(MAGIC) + self.payload_size])
            received_crc, = struct.unpack(
                '<H', bytes(self.buffer[len(MAGIC) + self.payload_size:self.frame_size]))

            if crc16(payload) != received_crc:
                del self.buffer[:len(MAGIC)]
                continue

            del self.buffer[:self.frame_size]
            payloads.append(payload)

        return payloads
