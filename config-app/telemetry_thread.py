import struct
import serial
from datetime import datetime
from typing import Dict, Tuple, Any

from PyQt5.QtCore import QThread, pyqtSignal

from uart_comm import FrameDecoder
from constants import SENSOR_TELEMETRY_FORMAT, SENSOR_TELEMETRY_SIZE


class TelemetryThread(QThread):
    """Thread for handling serial communication and telemetry"""

    response_received = pyqtSignal(str)
    telemetry_received = pyqtSignal(dict)
    connection_lost = pyqtSignal(str)

    def __init__(self, serial_port: serial.Serial) -> None:
        super().__init__()
        self.serial_port: serial.Serial = serial_port
        self.running: bool = True
        self.decoder: FrameDecoder = FrameDecoder(SENSOR_TELEMETRY_SIZE)

    def run(self) -> None:
        """Listen for responses and telemetry from the bridge"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:

                data: bytes = self.serial_port.read(4096)

                # Process complete, CRC-valid sensor_telemetry_t packets
                for packet in self.decoder.feed(data):
                    try:
                        values: Tuple[float, ...] = struct.unpack(
                            SENSOR_TELEMETRY_FORMAT, packet
                        )

                        telemetry: Dict[str, Any] = {
                            "pitch_angle": values[0],
                            "roll_angle": values[1],
                            "yaw_angle": values[2],
                            "pitch_rate": values[3],
                            "roll_rate": values[4],
                            "yaw_rate": values[5],
                            "mode": int(values[6]),
                            "pitch_pid": values[7],
                            "roll_pid": values[8],
                            "yaw_pid": values[9],
                            "motor_fl": values[10],
                            "motor_bl": values[11],
                            "motor_br": values[12],
                            "motor_fr": values[13],
                            "battery": values[14],
                            "timestamp": datetime.now(),
                        }
                        self.telemetry_received.emit(telemetry)
                    except Exception as e:
                        self.response_received.emit(f"Parse error: {e}")

            except serial.SerialException as e:

                self.connection_lost.emit(str(e))
                self.running = False
                break

            except Exception as e:
                self.response_received.emit(f"Error: {e}")

    def stop(self) -> None:
        self.running = False
