"""
Main window: serial connection, PID tuning tabs, and telemetry display.
"""

import math
import struct
import serial
import serial.tools.list_ports
from datetime import datetime
from typing import Dict, List, Optional, Any

from PyQt5.QtWidgets import (
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
    QFileDialog,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QCloseEvent

import uart_comm
from telemetry_thread import TelemetryThread
from constants import (
    PID_CONFIG_FORMAT,
    DRONE_CONFIG_FORMAT,
    REMOTE_CONFIG_FORMAT,
    PACKET_ID,
)
from pid_defaults import PID_DEFAULTS, PID_DEFAULT_KEYS, save_pid_defaults
from drone_config_defaults import (
    DRONE_DEFAULTS,
    REMOTE_DEFAULTS,
    save_drone_defaults,
    save_remote_defaults,
)
from plot_widget import PlotWidget
from attitude_widget import Attitude3DWidget
from session_recorder import SESSIONS_DIR, SessionRecorder
from session_viewer import SessionViewer

# Control loop rate the D-filter cutoff hint is worked out for (Hz)
LOOP_RATE_HZ = 400.0


def d_alpha_to_hz(alpha: float) -> Optional[float]:
    """Cutoff frequency of the drone's first-order D-term filter for a given
    coefficient, or None when the filter is off (alpha = 1)."""
    if alpha >= 1.0:
        return None
    return alpha / (1.0 - alpha) * LOOP_RATE_HZ / (2.0 * math.pi)


def d_filter_text(alpha: float) -> str:
    cutoff = d_alpha_to_hz(alpha)
    return "unfiltered" if cutoff is None else f"≈ {cutoff:.0f} Hz cutoff"



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

        # Snapshots of the values last sent live to the drone/remote
        self.drone_config_sent: Optional[Dict[str, float]] = None
        self.remote_config_sent: Optional[Dict[str, float]] = None

        self.axis_names: list[str] = ["Pitch / Roll", "Yaw"]
        self.mode_names: list[str] = ["Rate", "Angle"]

        # Telemetry storage
        self.telemetry_enabled: bool = True

        # Session recording to disk, and any open session viewer windows
        self.recorder: SessionRecorder = SessionRecorder()
        self.session_viewers: List[SessionViewer] = []
        self.session_btn: Optional[QPushButton] = None

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
        self.pid_tab: Optional[QWidget] = None
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

        # Create PID tuning tab
        self.pid_tab = self.create_pid_tab()

        # Create drone/remote config tabs
        drone_config_tab: QWidget = self.create_drone_config_tab()
        remote_config_tab: QWidget = self.create_remote_config_tab()

        # Now add the Tabs in the correct order
        self.main_tabs = QTabWidget()
        self.main_tabs.addTab(telemetry_tab, "Telemetry")
        self.main_tabs.addTab(self.pid_tab, "PID Tuning")
        self.main_tabs.addTab(drone_config_tab, "Drone Config")
        self.main_tabs.addTab(remote_config_tab, "Remote Config")
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

        # Session recording (saves every packet to a file for later review)
        self.session_btn = QPushButton("Start Recording")
        self.session_btn.setCheckable(True)
        self.session_btn.clicked.connect(self.toggle_session_recording)
        status_layout.addWidget(self.session_btn)

        open_session_btn: QPushButton = QPushButton("Open Session...")
        open_session_btn.clicked.connect(self.open_session)
        status_layout.addWidget(open_session_btn)

        # Live plot control (does not affect session recording)
        self.record_checkbox = QCheckBox("Live Plots")
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
        legend: List[tuple[str, str, str]] = [
            ("Pitch Angle:", "pitch_angle", "°"),
            ("Roll Angle:", "roll_angle", "°"),
            ("Yaw Angle:", "yaw_angle", "°"),
            ("Mode:", "mode", ""),
            ("Pitch Rate:", "pitch_rate", "°/s"),
            ("Roll Rate:", "roll_rate", "°/s"),
            ("Yaw Rate:", "yaw_rate", "°/s"),
            ("Battery:", "battery", "V"),
            ("Loop dt max:", "loop_dt_max_us", " µs"),
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

    def create_pid_tab(self) -> QWidget:
        """Create the single PID tuning tab: a Rate row with Pitch/Roll and
        Yaw side by side, then an Angle row with Pitch/Roll only, since the
        yaw axis has no angle loop (see the axis == 1 guard in send_pid)."""
        widget: QWidget = QWidget()
        layout: QVBoxLayout = QVBoxLayout(widget)

        rate_row: QHBoxLayout = QHBoxLayout()
        rate_row.addWidget(self.create_pid_group(0, 0, "Rate Mode PID - Pitch/Roll"))
        rate_row.addWidget(self.create_pid_group(1, 0, "Rate Mode PID - Yaw"))
        layout.addLayout(rate_row)

        angle_row: QHBoxLayout = QHBoxLayout()
        angle_row.addWidget(self.create_pid_group(0, 1, "Angle Mode PID - Pitch/Roll"))
        layout.addLayout(angle_row)

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
        default_alpha = PID_DEFAULTS.get(f"{prefix}_KD_ALPHA", 1.0) if prefix else 1.0

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

        layout.addWidget(QLabel("D filter α:"), 3, 0)
        alpha_spin: QDoubleSpinBox = QDoubleSpinBox()
        alpha_spin.setRange(0.01, 1.0)
        alpha_spin.setDecimals(2)
        alpha_spin.setSingleStep(0.05)
        alpha_spin.setValue(default_alpha)
        alpha_spin.setToolTip(
            "Low-pass filter on the D term. 1 = unfiltered; lower values smooth\n"
            "more (less noise) but add delay."
        )
        layout.addWidget(alpha_spin, 3, 1)

        alpha_hint: QLabel = QLabel(d_filter_text(default_alpha))
        alpha_hint.setStyleSheet("color: #666; font-style: italic;")
        alpha_spin.valueChanged.connect(
            lambda value: alpha_hint.setText(d_filter_text(value))
        )
        layout.addWidget(alpha_hint, 4, 0, 1, 2)

        send_btn: QPushButton = QPushButton(f"Send to Drone")
        send_btn.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold;"
        )
        send_btn.clicked.connect(
            lambda: self.send_pid(
                axis,
                mode,
                kp_spin.value(),
                ki_spin.value(),
                kd_spin.value(),
                alpha_spin.value(),
            )
        )
        layout.addWidget(send_btn, 2, 2, 1, 2)

        save_btn: QPushButton = QPushButton("Save to Flash && Default")
        save_btn.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold;"
        )
        save_btn.clicked.connect(
            lambda: self.save_pid_as_default(
                axis,
                mode,
                title,
                kp_spin.value(),
                ki_spin.value(),
                kd_spin.value(),
                alpha_spin.value(),
            )
        )
        layout.addWidget(save_btn, 3, 2, 1, 2)

        setattr(self, f"kp_spin_{axis}_{mode}", kp_spin)
        setattr(self, f"ki_spin_{axis}_{mode}", ki_spin)
        setattr(self, f"kd_spin_{axis}_{mode}", kd_spin)
        setattr(self, f"alpha_spin_{axis}_{mode}", alpha_spin)

        group.setLayout(layout)
        return group

    def create_drone_config_tab(self) -> QWidget:
        """Create the tab for the drone's non-PID config (failsafe/throttle/battery)."""
        widget: QWidget = QWidget()
        layout: QGridLayout = QGridLayout(widget)

        defaults = DRONE_DEFAULTS
        fields: List[tuple[str, str, float]] = [
            ("Max Rate (deg/s):", "max_rate", defaults.get("MAX_RATE", 0.0)),
            ("Max Angle (deg):", "max_angle", defaults.get("MAX_ANGLE", 0.0)),
            ("Fail Angle (deg):", "fail_angle", defaults.get("FAIL_ANGLE", 0.0)),
            ("Min Throttle (us):", "min_throttle", defaults.get("MIN_THROTTLE", 0.0)),
            ("Max Throttle (us):", "max_throttle", defaults.get("MAX_THROTTLE", 0.0)),
            (
                "Coms Timeout (s):",
                "coms_timeout_s",
                defaults.get("FAILSAFE_TIMEOUT_US", 0.0) / 1e6,
            ),
            ("Low Voltage (mV):", "low_voltage", defaults.get("LOW_VOLTAGE", 0.0)),
            (
                "Critical Voltage (mV):",
                "critical_voltage",
                defaults.get("CRITICAL_VOLTAGE", 0.0),
            ),
        ]

        self.drone_config_spins: Dict[str, QDoubleSpinBox] = {}
        for i, (label_text, key, default_value) in enumerate(fields):
            layout.addWidget(QLabel(label_text), i, 0)
            spin: QDoubleSpinBox = QDoubleSpinBox()
            spin.setRange(0, 1_000_000)
            spin.setDecimals(2)
            spin.setValue(default_value)
            layout.addWidget(spin, i, 1)
            self.drone_config_spins[key] = spin

        send_btn: QPushButton = QPushButton("Send to Drone")
        send_btn.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold;"
        )
        send_btn.clicked.connect(self.send_drone_config)
        layout.addWidget(send_btn, len(fields), 0)

        save_btn: QPushButton = QPushButton("Save to Flash && Default")
        save_btn.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold;"
        )
        save_btn.clicked.connect(self.save_drone_config)
        layout.addWidget(save_btn, len(fields), 1)

        return widget

    def create_remote_config_tab(self) -> QWidget:
        """Create the tab for the remote's config (joystick calibration/battery)."""
        widget: QWidget = QWidget()
        layout: QVBoxLayout = QVBoxLayout(widget)

        defaults = REMOTE_DEFAULTS

        battery_group: QGroupBox = QGroupBox("Battery / Voltage")
        battery_layout: QGridLayout = QGridLayout()
        battery_fields: List[tuple[str, str, float]] = [
            (
                "Voltage Cal Multiplier:",
                "voltage_cal_multiplier",
                defaults.get("VOLTAGE_CAL_MULTIPLIER", 1.0),
            ),
            ("Low Voltage (mV):", "low_voltage", defaults.get("LOW_VOLTAGE", 0.0)),
            (
                "Critical Voltage (mV):",
                "critical_voltage",
                defaults.get("CRITICAL_VOLTAGE", 0.0),
            ),
        ]
        self.remote_voltage_spins: Dict[str, QDoubleSpinBox] = {}
        for i, (label_text, key, default_value) in enumerate(battery_fields):
            battery_layout.addWidget(QLabel(label_text), i, 0)
            spin: QDoubleSpinBox = QDoubleSpinBox()
            spin.setRange(0, 100_000)
            spin.setDecimals(3)
            spin.setValue(default_value)
            battery_layout.addWidget(spin, i, 1)
            self.remote_voltage_spins[key] = spin

        battery_group.setLayout(battery_layout)
        layout.addWidget(battery_group)

        # Per-channel calibration falls back to the shared JOYSTICK_CAL_*
        # default from remote_defaults.h until Save to Flash has saved a
        # per-channel override for that specific channel.
        shared_cal_defaults: tuple[float, float, float] = (
            defaults.get("JOYSTICK_CAL_MIN", 0.0),
            defaults.get("JOYSTICK_CAL_CENTRE", 2048.0),
            defaults.get("JOYSTICK_CAL_MAX", 4095.0),
        )
        self.joystick_spins: Dict[str, Dict[str, QDoubleSpinBox]] = {}
        for channel in ["throttle", "pitch", "roll", "yaw"]:
            channel_defaults: tuple[float, float, float] = (
                defaults.get(f"JOYSTICK_{channel.upper()}_MIN", shared_cal_defaults[0]),
                defaults.get(
                    f"JOYSTICK_{channel.upper()}_CENTRE", shared_cal_defaults[1]
                ),
                defaults.get(f"JOYSTICK_{channel.upper()}_MAX", shared_cal_defaults[2]),
            )
            group: QGroupBox = QGroupBox(f"{channel.capitalize()} Calibration")
            grid: QGridLayout = QGridLayout()
            self.joystick_spins[channel] = {}
            for col, (label_text, sub_key, default_value) in enumerate(
                zip(
                    ["Min:", "Centre:", "Max:"],
                    ["min", "centre", "max"],
                    channel_defaults,
                )
            ):
                grid.addWidget(QLabel(label_text), 0, col * 2)
                spin = QDoubleSpinBox()
                spin.setRange(0, 4095)
                spin.setDecimals(0)
                spin.setValue(default_value)
                grid.addWidget(spin, 0, col * 2 + 1)
                self.joystick_spins[channel][sub_key] = spin
            group.setLayout(grid)
            layout.addWidget(group)

        btn_layout: QHBoxLayout = QHBoxLayout()
        send_btn: QPushButton = QPushButton("Send to Remote")
        send_btn.setStyleSheet(
            "background-color: #2196F3; color: white; font-weight: bold;"
        )
        send_btn.clicked.connect(self.send_remote_config)
        btn_layout.addWidget(send_btn)

        save_btn: QPushButton = QPushButton("Save to Flash && Default")
        save_btn.setStyleSheet(
            "background-color: #FF9800; color: white; font-weight: bold;"
        )
        save_btn.clicked.connect(self.save_remote_config)
        btn_layout.addWidget(save_btn)

        layout.addLayout(btn_layout)

        layout.addStretch()
        return widget

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

    def toggle_session_recording(self) -> None:
        """Start or stop saving telemetry to a session file"""
        if self.recorder.recording:
            self.stop_session_recording()
            return

        try:
            path: str = self.recorder.start()
        except OSError as e:
            self.session_btn.setChecked(False)
            QMessageBox.critical(self, "Recording Error", f"Could not start: {e}")
            return

        self.session_btn.setChecked(True)
        self.session_btn.setText("Stop Recording")
        self.session_btn.setStyleSheet(
            "background-color: #f44336; color: white; font-weight: bold;"
        )
        self.log_message(f"Recording session to {path}")

    def stop_session_recording(self) -> None:
        """Close the current session file, if one is being recorded"""
        rows: int = self.recorder.rows
        path: Optional[str] = self.recorder.stop()

        self.session_btn.setChecked(False)
        self.session_btn.setText("Start Recording")
        self.session_btn.setStyleSheet("")

        if path:
            self.log_message(f"Session saved ({rows} samples): {path}")

    def open_session(self) -> None:
        """Pick a recorded session file and show it in its own window"""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Session", SESSIONS_DIR, "Session files (*.csv)"
        )
        if not path:
            return

        try:
            viewer: SessionViewer = SessionViewer(path, self)
        except (OSError, ValueError) as e:
            QMessageBox.critical(self, "Open Session", f"Could not open: {e}")
            return

        # Keep a reference so the window isn't garbage collected
        self.session_viewers.append(viewer)
        viewer.show()

    def disconnect(self) -> None:
        """Disconnect from serial port"""
        self.stop_session_recording()

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

    def send_pid(
        self, axis: int, mode: int, kp: float, ki: float, kd: float, d_alpha: float
    ) -> bool:
        """Send PID configuration to drone. Returns whether the send succeeded."""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return False

        if axis == 1 and mode != 0:
            QMessageBox.warning(self, "Invalid Mode", "Yaw only supports Rate mode.")
            return False

        try:
            payload: bytes = struct.pack(
                PID_CONFIG_FORMAT, kp, ki, kd, d_alpha, axis, mode
            )
            packet: bytes = uart_comm.encode_config_packet(
                PACKET_ID.PID_CONFIG, payload
            )

            self.serial_port.write(packet)
            self.serial_port.flush()

            self.pid_config[axis][mode] = {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "d_alpha": d_alpha,
            }

            current_label: QLabel = getattr(self, f"current_label_{axis}_{mode}")
            current_label.setText(
                f"Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}, α={d_alpha:.2f}"
            )
            current_label.setStyleSheet("color: #2196F3; font-weight: bold;")

            axis_name: str = self.axis_names[axis]
            mode_name: str = self.mode_names[mode]
            self.log_message(
                f"→ Sent {axis_name} {mode_name}: Kp={kp:.4f}, Ki={ki:.4f}, "
                f"Kd={kd:.4f}, D filter α={d_alpha:.2f} ({d_filter_text(d_alpha)})"
            )
            self.recorder.add_event(
                f"{axis_name} {mode_name}: Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f} "
                f"a={d_alpha:.2f}"
            )
            return True

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return False

    def save_pid_as_default(
        self,
        axis: int,
        mode: int,
        title: str,
        kp: float,
        ki: float,
        kd: float,
        d_alpha: float,
    ) -> None:
        """
        Tell the drone to persist its currently-live config to flash -
        PID gains live inside the drone's own config.
        """
        axis_name: str = self.axis_names[axis]

        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return

        reply = QMessageBox.question(
            self,
            "Save to Flash & Default",
            f"Persist the drone's currently-live config to flash, and save the\n"
            f"{axis_name} {title} gains as your local working default:\n"
            f"Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}, D filter α={d_alpha:.2f}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        live = {"kp": kp, "ki": ki, "kd": kd, "d_alpha": d_alpha}
        if self.pid_config[axis][mode] != live:
            if not self.send_pid(axis, mode, kp, ki, kd, d_alpha):
                return

        try:
            packet: bytes = uart_comm.encode_config_packet(PACKET_ID.DRONE_CFG_STORE)
            self.serial_port.write(packet)
            self.serial_port.flush()
            self.log_message("→ Sent DRONE_CFG_STORE (persist to flash)")

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return

        if save_pid_defaults(axis, mode, kp, ki, kd, d_alpha):
            self.log_message(
                f"Saved {axis_name} {title} as default: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}"
            )
        else:
            self.log_message(f"Failed to save {axis_name} {title} defaults")

    def _drone_config_values(self) -> Dict[str, float]:
        """Current drone config tab spin-box values, keyed the same way as
        self.drone_config_spins."""
        return {key: spin.value() for key, spin in self.drone_config_spins.items()}

    def send_drone_config(self) -> bool:
        """Apply the drone config tab's values live (RAM only, not yet
        saved). Returns whether the send succeeded."""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return False

        values = self._drone_config_values()
        try:
            payload: bytes = struct.pack(
                DRONE_CONFIG_FORMAT,
                values["max_rate"],
                values["max_angle"],
                values["fail_angle"],
                values["min_throttle"],
                values["max_throttle"],
                int(values["coms_timeout_s"] * 1e6),
                int(values["low_voltage"]),
                int(values["critical_voltage"]),
            )
            packet: bytes = uart_comm.encode_config_packet(PACKET_ID.DRONE_CFG, payload)

            self.serial_port.write(packet)
            self.serial_port.flush()
            self.log_message("→ Sent drone config update (live, not yet saved)")
            self.drone_config_sent = values
            return True

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return False

    def save_drone_config(self) -> None:
        """
        Tell the drone to persist its currently-live config to flash, and 
        save the drone config tab's values as the GUI's own working defaults 
        (user_defaults.json).
        """
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return

        reply = QMessageBox.question(
            self,
            "Save to Flash & Default",
            "Persist the drone's currently-live config (including PID gains) to flash,\n"
            "and save these values as your local working defaults?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        values = self._drone_config_values()
        if self.drone_config_sent != values:
            if not self.send_drone_config():
                return

        try:
            packet: bytes = uart_comm.encode_config_packet(PACKET_ID.DRONE_CFG_STORE)
            self.serial_port.write(packet)
            self.serial_port.flush()
            self.log_message("→ Sent DRONE_CFG_STORE (persist to flash)")

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return

        updates = {
            "MAX_RATE": values["max_rate"],
            "MAX_ANGLE": values["max_angle"],
            "FAIL_ANGLE": values["fail_angle"],
            "MIN_THROTTLE": values["min_throttle"],
            "MAX_THROTTLE": values["max_throttle"],
            "FAILSAFE_TIMEOUT_US": values["coms_timeout_s"] * 1e6,
            "LOW_VOLTAGE": values["low_voltage"],
            "CRITICAL_VOLTAGE": values["critical_voltage"],
        }

        if save_drone_defaults(updates):
            self.log_message("Saved drone config tab as default")
        else:
            self.log_message("Failed to save drone config defaults")

    def _remote_config_values(self) -> Dict[str, float]:
        """Current remote config tab spin-box values (battery/voltage and
        per-channel joystick calibration), flattened into one dict."""
        v = self.remote_voltage_spins
        j = self.joystick_spins
        values: Dict[str, float] = {
            "voltage_cal_multiplier": v["voltage_cal_multiplier"].value(),
            "low_voltage": v["low_voltage"].value(),
            "critical_voltage": v["critical_voltage"].value(),
        }
        for channel in ["throttle", "pitch", "roll", "yaw"]:
            values[f"{channel}_min"] = j[channel]["min"].value()
            values[f"{channel}_centre"] = j[channel]["centre"].value()
            values[f"{channel}_max"] = j[channel]["max"].value()
        return values

    def send_remote_config(self) -> bool:
        """Apply the remote config tab's values live (RAM only, not yet
        saved). Returns whether the send succeeded."""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return False

        values = self._remote_config_values()
        try:
            payload: bytes = struct.pack(
                REMOTE_CONFIG_FORMAT,
                values["voltage_cal_multiplier"],
                int(values["low_voltage"]),
                int(values["critical_voltage"]),
                int(values["throttle_min"]),
                int(values["throttle_centre"]),
                int(values["throttle_max"]),
                int(values["pitch_min"]),
                int(values["pitch_centre"]),
                int(values["pitch_max"]),
                int(values["roll_min"]),
                int(values["roll_centre"]),
                int(values["roll_max"]),
                int(values["yaw_min"]),
                int(values["yaw_centre"]),
                int(values["yaw_max"]),
            )
            packet: bytes = uart_comm.encode_config_packet(
                PACKET_ID.REMOTE_CFG, payload
            )

            self.serial_port.write(packet)
            self.serial_port.flush()
            self.log_message("→ Sent remote config update (live, not yet saved)")
            self.remote_config_sent = values
            return True

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return False

    def save_remote_config(self) -> None:
        """Tell the remote to persist its currently-live config to flash, and
        save the remote tab's values as the GUI's own working defaults."""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(
                self, "Not Connected", "Please connect to serial port first."
            )
            return

        reply = QMessageBox.question(
            self,
            "Save to Flash & Default",
            "Persist the remote's currently-live config to flash,\n"
            "and save these values as your local working defaults?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        values = self._remote_config_values()
        if self.remote_config_sent != values:
            if not self.send_remote_config():
                return

        try:
            packet: bytes = uart_comm.encode_config_packet(PACKET_ID.REMOTE_CFG_STORE)
            self.serial_port.write(packet)
            self.serial_port.flush()
            self.log_message("→ Sent REMOTE_CFG_STORE (persist to flash)")

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
            return

        updates: Dict[str, float] = {
            "VOLTAGE_CAL_MULTIPLIER": values["voltage_cal_multiplier"],
            "LOW_VOLTAGE": values["low_voltage"],
            "CRITICAL_VOLTAGE": values["critical_voltage"],
        }
        for channel in ["throttle", "pitch", "roll", "yaw"]:
            updates[f"JOYSTICK_{channel.upper()}_MIN"] = values[f"{channel}_min"]
            updates[f"JOYSTICK_{channel.upper()}_CENTRE"] = values[f"{channel}_centre"]
            updates[f"JOYSTICK_{channel.upper()}_MAX"] = values[f"{channel}_max"]

        if save_remote_defaults(updates):
            self.log_message("Saved remote config tab as default")
        else:
            self.log_message("Failed to save remote config defaults")

    def handle_telemetry(self, data: Dict[str, Any]) -> None:
        """Handle received telemetry data"""
        # Session recording takes every packet, whatever the plots are doing
        self.recorder.add(data)

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
                elif key == "loop_dt_max_us":
                    text = f"{data[key]:.0f}{unit}"
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
