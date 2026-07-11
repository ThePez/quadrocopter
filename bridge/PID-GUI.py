"""
PID Tuning GUI for Drone with Real-time Telemetry Plotting
PyQt5 interface for sending PID coefficients and viewing telemetry
"""

import os
import re
import sys
import struct
import serial
import serial.tools.list_ports
from collections import deque
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Deque, Any
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QTextEdit,
    QGroupBox,
    QTabWidget,
    QDoubleSpinBox,
    QMessageBox,
    QCheckBox,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QCloseEvent

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import uart_comm

# Mirrors struct sensor_telemetry_t in libs/espnow-comm/espnow_comm.h:
# 10 floats (angles/rates/mode/pid outputs) then 5 uint16_t (motor duty cycles, battery mV).
# The struct isn't packed, so the compiler pads its sizeof() up to a multiple of 4
# The trailing '2x' skips those 2 pad bytes; bridge.c sends sizeof(struct
# sensor_telemetry_t) bytes per packet, so this must track that exactly.
SENSOR_TELEMETRY_FORMAT = "<10f5H2x"
SENSOR_TELEMETRY_SIZE = struct.calcsize(SENSOR_TELEMETRY_FORMAT)

# Mirrors struct pid_config_telemetry_t in libs/espnow-comm/espnow_comm.h:
# 3 floats (kp, ki, kd) then 2 uint16_t (axis, mode).
PID_CONFIG_FORMAT = "<fffHH"

_DEFINE_RE = re.compile(r"^\s*#define\s+(\w+)\s+([0-9.eE+-]+)\s*$")


def load_pid_defaults() -> Dict[str, float]:
    """Parse libs/pid/pid_defaults.h so the GUI's default gains stay in sync
    with the firmware instead of being hand-copied into two places."""
    header_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "libs",
        "pid",
        "pid_defaults.h",
    )
    defaults: Dict[str, float] = {}
    try:
        with open(header_path, "r") as f:
            for line in f:
                match = _DEFINE_RE.match(line)
                if match:
                    defaults[match.group(1)] = float(match.group(2))
    except OSError as exc:
        print(f"Warning: could not read PID defaults from {header_path}: {exc}")
    return defaults


# Populated at import time from pid_defaults.h
PID_DEFAULTS: Dict[str, float] = load_pid_defaults()

# Maps each (axis, mode) tab to the #define prefix used in pid_defaults.h
PID_DEFAULT_KEYS: Dict[Tuple[int, int], str] = {
    (0, 0): "RATE_PITCH_ROLL",
    (1, 0): "RATE_YAW",
    (0, 1): "ANGLE_PITCH_ROLL",
}


def format_gain(value: float) -> str:
    """Render a gain the way pid_defaults.h already writes them (e.g. 1.0, 0.175)."""
    text = f"{value:.4f}".rstrip("0")
    return text + "0" if text.endswith(".") else text


def save_pid_defaults(axis: int, mode: int, kp: float, ki: float, kd: float) -> bool:
    """Write the given gains back into pid_defaults.h as the new defaults for
    this axis/mode, so the next GUI launch and the next firmware build both
    start from values already validated on the drone."""
    prefix = PID_DEFAULT_KEYS.get((axis, mode))
    if prefix is None:
        return False

    header_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "libs",
        "pid",
        "pid_defaults.h",
    )
    updates = {f"{prefix}_KP": kp, f"{prefix}_KI": ki, f"{prefix}_KD": kd}

    try:
        with open(header_path, "r") as f:
            lines = f.readlines()

        for i, line in enumerate(lines):
            match = _DEFINE_RE.match(line)
            if match and match.group(1) in updates:
                lines[i] = (
                    f"#define {match.group(1)} {format_gain(updates[match.group(1)])}\n"
                )

        with open(header_path, "w") as f:
            f.writelines(lines)
    except OSError as exc:
        print(f"Warning: could not write PID defaults to {header_path}: {exc}")
        return False

    PID_DEFAULTS.update(updates)
    return True


class TelemetryThread(QThread):
    """Thread for handling serial communication and telemetry"""

    response_received = pyqtSignal(str)
    telemetry_received = pyqtSignal(dict)
    connection_lost = pyqtSignal(str)

    def __init__(self, serial_port: serial.Serial) -> None:
        super().__init__()
        self.serial_port: serial.Serial = serial_port
        self.running: bool = True
        self.decoder: uart_comm.FrameDecoder = uart_comm.FrameDecoder(
            SENSOR_TELEMETRY_SIZE
        )

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


class PlotWidget(QWidget):
    """Widget for displaying real-time plots"""

    def __init__(
        self, title: str, labels: List[str], colors: List[str], max_points: int = 500
    ) -> None:
        super().__init__()
        self.max_points: int = max_points
        self.labels: List[str] = labels
        self.colors: List[str] = colors

        # Data storage
        self.time_data: Deque[float] = deque(maxlen=max_points)
        self.data: dict[str, Deque[float]] = {
            label: deque(maxlen=max_points) for label in labels
        }
        self.start_time: datetime = datetime.now()

        # Create layout
        layout: QVBoxLayout = QVBoxLayout(self)

        # Create plot widget
        self.plot_widget: pg.PlotWidget = pg.PlotWidget(title=title)
        self.plot_widget.setBackground("w")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Time (s)")

        # Add legend
        self.plot_widget.addLegend()

        # Create plot curves
        self.curves: dict[str, pg.PlotDataItem] = {}
        for label, color in zip(labels, colors):
            pen = pg.mkPen(color=color, width=2)
            self.curves[label] = self.plot_widget.plot([], [], pen=pen, name=label)

        layout.addWidget(self.plot_widget)

        # Control buttons
        control_layout: QHBoxLayout = QHBoxLayout()

        self.pause_btn: QPushButton = QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.toggle_pause)
        control_layout.addWidget(self.pause_btn)

        clear_btn: QPushButton = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_data)
        control_layout.addWidget(clear_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        self.paused: bool = False

    def toggle_pause(self) -> None:
        self.paused = self.pause_btn.isChecked()
        self.pause_btn.setText("Resume" if self.paused else "Pause")

    def clear_data(self) -> None:
        self.time_data.clear()
        for label in self.labels:
            self.data[label].clear()
        self.start_time = datetime.now()
        self.update_plot()

    def add_data(self, data_dict: Dict[str, float]) -> None:
        if self.paused:
            return

        # Calculate elapsed time
        elapsed: float = (datetime.now() - self.start_time).total_seconds()
        self.time_data.append(elapsed)

        # Add data for each label
        for label in self.labels:
            if label in data_dict:
                self.data[label].append(data_dict[label])
            else:
                self.data[label].append(0)

        self.update_plot()

    def update_plot(self) -> None:
        time_list: tuple[float, ...] = tuple(self.time_data)
        for label in self.labels:
            data_list: tuple[float, ...] = tuple(self.data[label])
            self.curves[label].setData(time_list, data_list)


class Attitude3DWidget(QWidget):
    """3D visualization of the drone's current orientation"""

    ARM_LENGTH = 1.0

    def __init__(self) -> None:
        super().__init__()
        layout: QVBoxLayout = QVBoxLayout(self)

        self.gl_view: gl.GLViewWidget = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=6, elevation=25, azimuth=45)
        layout.addWidget(self.gl_view)

        # Ground reference grid
        grid: gl.GLGridItem = gl.GLGridItem()
        grid.setSize(x=10, y=10)
        grid.setSpacing(x=1, y=1)
        grid.translate(0, 0, -1.5)
        self.gl_view.addItem(grid)

        # Arm endpoints in an X configuration. Scene axes: X=right, Y=forward (nose), Z=up.
        s: float = self.ARM_LENGTH * 0.7071  # sin/cos(45 deg)
        self.fl: np.ndarray = np.array([-s, s, 0])
        self.fr: np.ndarray = np.array([s, s, 0])
        self.bl: np.ndarray = np.array([-s, -s, 0])
        self.br: np.ndarray = np.array([s, -s, 0])
        origin: np.ndarray = np.array([0, 0, 0])

        # Front arms (red, matches the Motors plot's FL/FR coloring) vs rear arms (grey),
        # so the model's heading is visually obvious as it yaws.
        self.front_arms: gl.GLLinePlotItem = gl.GLLinePlotItem(
            pos=np.array([origin, self.fl, origin, self.fr]),
            color=(1, 0, 0, 1),
            width=4,
            mode="lines",
            antialias=True,
        )
        self.rear_arms: gl.GLLinePlotItem = gl.GLLinePlotItem(
            pos=np.array([origin, self.bl, origin, self.br]),
            color=(0.5, 0.5, 0.5, 1),
            width=4,
            mode="lines",
            antialias=True,
        )
        self.gl_view.addItem(self.front_arms)
        self.gl_view.addItem(self.rear_arms)

        # Motor tip markers, colors matching the Motors plot (FL, BL, BR, FR)
        self.motor_markers: gl.GLScatterPlotItem = gl.GLScatterPlotItem(
            pos=np.array([self.fl, self.bl, self.br, self.fr]),
            color=np.array(
                [
                    [1, 0, 0, 1],  # FL - red
                    [1, 0.65, 0, 1],  # BL - orange
                    [0.5, 0, 0.5, 1],  # BR - purple
                    [0, 1, 1, 1],  # FR - cyan
                ]
            ),
            size=15,
            pxMode=True,
        )
        self.gl_view.addItem(self.motor_markers)

        # All the pieces above represent one rigid body, so the same transform
        # gets applied to each of them every update.
        self.rigid_body_items: List[gl.GLGraphicsItem] = [
            self.front_arms,
            self.rear_arms,
            self.motor_markers,
        ]

    def update_attitude(self, pitch: float, roll: float, yaw: float) -> None:
        """Rotate the drone model to the given angles (degrees)"""
        # QMatrix4x4.rotate() post-multiplies (self = self * R), so calling it
        # in this order composes as Ryaw * Rpitch * Rroll applied to a point -
        # i.e. roll happens first, then pitch, then yaw. That's the standard
        # aerospace ZYX Euler convention.
        tr: pg.Transform3D = pg.Transform3D()
        tr.rotate(yaw, 0, 0, 1)
        tr.rotate(pitch, 1, 0, 0)
        tr.rotate(roll, 0, 1, 0)

        for item in self.rigid_body_items:
            item.setTransform(tr)


class PIDTunerGUI(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.serial_port: Optional[serial.Serial] = None
        self.telemetry_thread: Optional[TelemetryThread] = None

        # Track PID configurations
        self.pid_config: Dict[int, dict[int, Optional[dict[str, float]]]] = {
            0: {0: None, 1: None},
            1: {0: None, 1: None},
        }

        self.axis_names: list[str] = ["Pitch / Roll", "Yaw"]
        self.mode_names: list[str] = ["Rate", "Angle"]

        # Telemetry storage
        self.telemetry_enabled: bool = True

        # Telemetry timeout tracking
        self.last_telemetry_time: Optional[datetime] | None = None
        self.telemetry_timeout_ms: int = 2000  # 2 seconds timeout

        # UI elements that will be created in init_ui
        self.port_combo: Optional[QComboBox] = None
        self.baudrate_combo: Optional[QComboBox] = None
        self.connect_btn: Optional[QPushButton] = None
        self.status_label: Optional[QLabel] = None
        self.telemetry_status: Optional[QLabel] = None
        self.record_checkbox: Optional[QCheckBox] = None
        self.log_text: Optional[QTextEdit] = None
        self.main_tabs: Optional[QTabWidget] = None
        self.pitch_roll_tab: Optional[QWidget] = None
        self.yaw_tab: Optional[QWidget] = None
        self.angles_plot: Optional[PlotWidget] = None
        self.rates_plot: Optional[PlotWidget] = None
        self.pid_plot: Optional[PlotWidget] = None
        self.motors_plot: Optional[PlotWidget] = None
        self.attitude_3d: Optional[Attitude3DWidget] = None
        self.value_labels: Optional[dict[str, tuple[QLabel, str]]] = None
        self.timeout_timer: Optional[QTimer] = None

        self.init_ui()

        # Start timeout checker timer
        self.timeout_timer = QTimer()
        self.timeout_timer.timeout.connect(self.check_telemetry_timeout)
        self.timeout_timer.start(500)  # Check every 500ms

    def init_ui(self) -> None:
        """Initialize the user interface"""
        self.setWindowTitle("Drone PID Tuner & Telemetry")
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget: QWidget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout: QVBoxLayout = QVBoxLayout(central_widget)

        # Creat Console tab (This must be created first for logging purposes)
        console_tab: QWidget = self.create_console_tab()

        # Create Connection section
        connection_group: QGroupBox = self.create_connection_section()
        main_layout.addWidget(connection_group)

        # Creat Telemetry tab
        telemetry_tab: QWidget = self.create_telemetry_tab()

        # Create PID tuning tabs
        pid_tabs: QTabWidget = QTabWidget()
        self.pitch_roll_tab = self.create_axis_tab(0)
        self.yaw_tab = self.create_axis_tab(1)
        pid_tabs.addTab(self.pitch_roll_tab, "Pitch + Roll")
        pid_tabs.addTab(self.yaw_tab, "Yaw")

        # Now add the Tabs in the correct order
        self.main_tabs = QTabWidget()
        self.main_tabs.addTab(telemetry_tab, "Telemetry")
        self.main_tabs.addTab(pid_tabs, "PID Tuning")
        self.main_tabs.addTab(console_tab, "Console")

        # Add the main tabs widget to the layout manager
        main_layout.addWidget(self.main_tabs)

        self.log_message("PID Tuner started. Please connect to serial port.")

    def create_telemetry_tab(self) -> QWidget:
        """Create the telemetry visualization tab"""
        labels: list[str] = ["Pitch", "Roll", "Yaw"]
        colours: list[str] = ["r", "g", "b"]

        m_labels: list[str] = ["Front Left", "Front Right", "Back Right", "Back Left"]
        m_colours: list[str] = ["r", "purple", "orange", "cyan"]

        widget: QWidget = QWidget()
        layout: QVBoxLayout = QVBoxLayout(widget)

        # Status bar
        status_layout: QHBoxLayout = QHBoxLayout()
        self.telemetry_status = QLabel("No telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: #888;")
        status_layout.addWidget(self.telemetry_status)

        status_layout.addStretch()

        # Recording control
        self.record_checkbox = QCheckBox("Record Data")
        self.record_checkbox.setChecked(True)
        status_layout.addWidget(self.record_checkbox)

        layout.addLayout(status_layout)

        # Create plot tabs
        plot_tabs: QTabWidget = QTabWidget()

        # Angles plot
        self.angles_plot = PlotWidget("Attitude Angles", labels, colours)
        plot_tabs.addTab(self.angles_plot, "Angles")

        # Rates plot
        self.rates_plot = PlotWidget("Angular Rates", labels, colours)
        plot_tabs.addTab(self.rates_plot, "Rates")

        # PID plot
        self.pid_plot = PlotWidget("PID Outputs", labels, colours)
        plot_tabs.addTab(self.pid_plot, "PID Outputs")

        # Motors plot
        self.motors_plot = PlotWidget("Motor Speeds", m_labels, m_colours)
        plot_tabs.addTab(self.motors_plot, "Motors")

        # 3D attitude view
        self.attitude_3d = Attitude3DWidget()
        plot_tabs.addTab(self.attitude_3d, "3D Attitude")

        layout.addWidget(plot_tabs)

        # Current values display
        values_group: QGroupBox = QGroupBox("Current Values")
        values_layout: QGridLayout = QGridLayout()

        self.value_labels = {}
        legend: List[Tuple[str, str, str]] = [
            ("Pitch Angle:", "pitch_angle", "°"),
            ("Roll Angle:", "roll_angle", "°"),
            ("Yaw Angle:", "yaw_angle", "°"),
            ("Mode:", "mode", ""),
            ("Pitch Rate:", "pitch_rate", "°/s"),
            ("Roll Rate:", "roll_rate", "°/s"),
            ("Yaw Rate:", "yaw_rate", "°/s"),
            ("Battery:", "battery", "V"),
        ]

        for i, (label_text, key, unit) in enumerate(legend):
            row: int = i // 4
            col: int = (i % 4) * 2
            values_layout.addWidget(QLabel(label_text), row, col)
            value_label: QLabel = QLabel("--")
            value_label.setStyleSheet("font-weight: bold; color: #2196F3;")
            self.value_labels[key] = (value_label, unit)
            values_layout.addWidget(value_label, row, col + 1)

        values_group.setLayout(values_layout)
        layout.addWidget(values_group)

        return widget

    def create_connection_section(self) -> QGroupBox:
        """Create the serial port connection section"""
        group: QGroupBox = QGroupBox("Serial Connection")
        layout: QHBoxLayout = QHBoxLayout()

        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMaxVisibleItems(10)
        self.port_combo.view().setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.port_combo.setMinimumWidth(300)
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        refresh_btn: QPushButton = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "115200", "230400", "460800"])
        self.baudrate_combo.setCurrentText("115200")
        layout.addWidget(self.baudrate_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold;"
        )
        layout.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.status_label)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def create_axis_tab(self, axis: int) -> QWidget:
        """Create a tab for configuring an axis"""
        widget: QWidget = QWidget()
        layout: QVBoxLayout = QVBoxLayout(widget)

        rate_group: QGroupBox = self.create_pid_group(axis, 0, "Rate Mode PID")
        layout.addWidget(rate_group)

        if axis != 1:
            angle_group: QGroupBox = self.create_pid_group(axis, 1, "Angle Mode PID")
            layout.addWidget(angle_group)
        else:
            note_label: QLabel = QLabel("Note: Yaw only supports Rate mode")
            note_label.setStyleSheet("color: #666; font-style: italic;")
            layout.addWidget(note_label)

        layout.addStretch()
        return widget

    def create_pid_group(self, axis: int, mode: int, title: str) -> QGroupBox:
        """Create a PID configuration group"""
        group: QGroupBox = QGroupBox(title)
        layout: QGridLayout = QGridLayout()

        prefix = PID_DEFAULT_KEYS.get((axis, mode))
        default_kp = PID_DEFAULTS.get(f"{prefix}_KP", 0.0) if prefix else 0.0
        default_ki = PID_DEFAULTS.get(f"{prefix}_KI", 0.0) if prefix else 0.0
        default_kd = PID_DEFAULTS.get(f"{prefix}_KD", 0.0) if prefix else 0.0

        current_label: QLabel = QLabel("Current:")
        layout.addWidget(current_label, 0, 0)

        current_value: QLabel = QLabel("DEFAULT")
        current_value.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(current_value, 0, 1, 1, 3)
        setattr(self, f"current_label_{axis}_{mode}", current_value)

        layout.addWidget(QLabel("Kp:"), 1, 0)
        kp_spin: QDoubleSpinBox = QDoubleSpinBox()
        kp_spin.setRange(0, 100)
        kp_spin.setDecimals(4)
        kp_spin.setSingleStep(0.001)
        kp_spin.setValue(default_kp)
        layout.addWidget(kp_spin, 1, 1)

        layout.addWidget(QLabel("Ki:"), 1, 2)
        ki_spin: QDoubleSpinBox = QDoubleSpinBox()
        ki_spin.setRange(0, 100)
        ki_spin.setDecimals(4)
        ki_spin.setSingleStep(0.001)
        ki_spin.setValue(default_ki)
        layout.addWidget(ki_spin, 1, 3)

        layout.addWidget(QLabel("Kd:"), 2, 0)
        kd_spin: QDoubleSpinBox = QDoubleSpinBox()
        kd_spin.setRange(0, 100)
        kd_spin.setDecimals(4)
        kd_spin.setSingleStep(0.001)
        kd_spin.setValue(default_kd)
        layout.addWidget(kd_spin, 2, 1)

        send_btn: QPushButton = QPushButton(f"Send to Drone")
        send_btn.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold;"
        )
        send_btn.clicked.connect(
            lambda: self.send_pid(
                axis, mode, kp_spin.value(), ki_spin.value(), kd_spin.value()
            )
        )
        layout.addWidget(send_btn, 2, 2, 1, 2)

        save_btn: QPushButton = QPushButton("Save as Default")
        save_btn.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold;"
        )
        save_btn.clicked.connect(
            lambda: self.save_pid_as_default(
                axis, mode, title, kp_spin.value(), ki_spin.value(), kd_spin.value()
            )
        )
        layout.addWidget(save_btn, 3, 2, 1, 2)

        setattr(self, f"kp_spin_{axis}_{mode}", kp_spin)
        setattr(self, f"ki_spin_{axis}_{mode}", ki_spin)
        setattr(self, f"kd_spin_{axis}_{mode}", kd_spin)

        group.setLayout(layout)
        return group

    def create_console_tab(self) -> QWidget:
        """Create console log tab"""
        widget: QWidget = QWidget()
        layout: QVBoxLayout = QVBoxLayout(widget)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier", 9))
        layout.addWidget(self.log_text)

        clear_btn: QPushButton = QPushButton("Clear Log")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        layout.addWidget(clear_btn)

        return widget

    def refresh_ports(self) -> None:
        """Refresh available serial ports"""
        self.port_combo.clear()
        try:
            ports = list(serial.tools.list_ports.comports())
            valid_ports = []

            for port in ports:
                if not port.device:
                    continue
                if not port.description or port.description.strip().upper() == "N/A":
                    continue
                valid_ports.append(port)
                display_text: str = f"{port.device} ({port.description[:40]})"
                self.port_combo.addItem(display_text, port.device)

            if not valid_ports:
                self.port_combo.addItem("No valid ports found", None)
                self.log_message("No valid serial ports detected.")
        except Exception as e:
            self.port_combo.addItem("Error scanning ports", None)
            self.log_message(f"Error scanning ports: {e}")

    def toggle_connection(self) -> None:
        """Connect or disconnect from serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        """Connect to selected serial port"""
        port: Optional[str] = self.port_combo.currentData()
        baudrate: int = int(self.baudrate_combo.currentText())

        if not port:
            QMessageBox.warning(self, "Connection Error", "No valid port selected.")
            return

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.log_message(f"Connected to {port} at {baudrate} baud")

            # Start the telemetry thread
            self.telemetry_thread = TelemetryThread(self.serial_port)
            self.telemetry_thread.response_received.connect(self.handle_response)
            self.telemetry_thread.telemetry_received.connect(self.handle_telemetry)
            self.telemetry_thread.connection_lost.connect(self.handle_connection_lost)
            self.telemetry_thread.start()

            # Update UI
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet(
                "background-color: #f44336; color: white; font-weight: bold;"
            )

        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect: {e}")
            self.log_message(f"Connection failed: {e}")

    def disconnect(self) -> None:
        """Disconnect from serial port"""
        if self.telemetry_thread:
            self.telemetry_thread.stop()
            self.telemetry_thread.wait()
            self.telemetry_thread = None

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_message("Disconnected from serial port")

        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet(
            "background-color: #4CAF50; color: white; font-weight: bold;"
        )

        self.telemetry_status.setText("⚫ No telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: #888;")
        self.last_telemetry_time = None

    def check_telemetry_timeout(self) -> None:
        """Check if telemetry has timed out"""
        if self.last_telemetry_time is None:
            return

        if not self.serial_port or not self.serial_port.is_open:
            self.last_telemetry_time = None
            return

        elapsed_ms: float = (
            datetime.now() - self.last_telemetry_time
        ).total_seconds() * 1000

        if elapsed_ms > self.telemetry_timeout_ms:
            self.telemetry_status.setText("Telemetry timeout")
            self.telemetry_status.setStyleSheet("font-weight: bold; color: #f44336;")

    def send_pid(self, axis: int, mode: int, kp: float, ki: float, kd: float) -> None:
        """Send PID configuration to drone"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return

        if axis == 1 and mode != 0:
            QMessageBox.warning(self, "Invalid Mode", "Yaw only supports Rate mode.")
            return

        try:
            payload: bytes = struct.pack(PID_CONFIG_FORMAT, kp, ki, kd, axis, mode)
            packet: bytes = uart_comm.encode(payload)

            self.serial_port.write(packet)
            self.serial_port.flush()

            self.pid_config[axis][mode] = {"kp": kp, "ki": ki, "kd": kd}

            current_label: QLabel = getattr(self, f"current_label_{axis}_{mode}")
            current_label.setText(f"Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")
            current_label.setStyleSheet("color: #2196F3; font-weight: bold;")

            axis_name: str = self.axis_names[axis]
            mode_name: str = self.mode_names[mode]
            self.log_message(
                f"→ Sent {axis_name} {mode_name}: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}"
            )

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")

    def save_pid_as_default(
        self, axis: int, mode: int, title: str, kp: float, ki: float, kd: float
    ) -> None:
        """Write the current spin-box values into pid_defaults.h as the new
        boot-time defaults for this loop."""
        axis_name: str = self.axis_names[axis]

        reply = QMessageBox.question(
            self,
            "Save as Default",
            f"Overwrite the {axis_name} {title} defaults in pid_defaults.h with\n"
            f"Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        if save_pid_defaults(axis, mode, kp, ki, kd):
            self.log_message(
                f"Saved {axis_name} {title} as default: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}"
            )
        else:
            QMessageBox.critical(
                self, "Save Error", "Failed to write pid_defaults.h - see console."
            )
            self.log_message(f"Failed to save {axis_name} {title} defaults")

    def handle_telemetry(self, data: Dict[str, Any]) -> None:
        """Handle received telemetry data"""
        if not self.record_checkbox.isChecked():
            return

        # Update timestamp
        self.last_telemetry_time = datetime.now()

        # Update status
        self.telemetry_status.setText("Receiving telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: green;")

        # Update Current value labels
        for key, (label, unit) in self.value_labels.items():
            if key in data:
                text: str
                if key == "mode":
                    text = "STABILISE" if data[key] else "ACRO"
                elif key == "battery":
                    # voltage sent as mV
                    voltage: float = data[key] / 1000
                    text = f"{voltage:.3f}{unit}"
                else:
                    text = f"{data[key]:.3f}{unit}"
                label.setText(text)

        # Update plots
        angles_data: Dict[str, int] = {
            "Pitch": data["pitch_angle"],
            "Roll": data["roll_angle"],
            "Yaw": data["yaw_angle"],
        }
        self.angles_plot.add_data(angles_data)

        rates_data: Dict[str, int] = {
            "Pitch": data["pitch_rate"],
            "Roll": data["roll_rate"],
            "Yaw": data["yaw_rate"],
        }
        self.rates_plot.add_data(rates_data)

        pid_data: Dict[str, int] = {
            "Pitch": data["pitch_pid"],
            "Roll": data["roll_pid"],
            "Yaw": data["yaw_pid"],
        }
        self.pid_plot.add_data(pid_data)

        motors_data: Dict[str, int] = {
            "Front Left": data["motor_fl"],
            "Back Left": data["motor_bl"],
            "Back Right": data["motor_br"],
            "Front Right": data["motor_fr"],
        }
        self.motors_plot.add_data(motors_data)

        self.attitude_3d.update_attitude(
            data["pitch_angle"], data["roll_angle"], data["yaw_angle"]
        )

    def handle_response(self, response: str) -> None:
        """Handle text response from serial port"""
        self.log_message(f"← {response}")

    def handle_connection_lost(self, error: str) -> None:
        """Handle the serial port disappearing (e.g. USB unplugged)"""
        self.log_message(f"Serial connection lost: {error}")
        self.disconnect()
        QMessageBox.warning(
            self,
            "Connection Lost",
            f"The serial connection was lost:\n{error}",
        )

    def log_message(self, message: str) -> None:
        """Add message to log"""
        timestamp: str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.append(f"[{timestamp}] {message}")
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event: QCloseEvent) -> None:
        """Handle window close event"""
        self.disconnect()
        event.accept()


def main() -> None:
    app: QApplication = QApplication(sys.argv)
    app.setStyle("Fusion")

    window: PIDTunerGUI = PIDTunerGUI()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
