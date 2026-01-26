#!/usr/bin/env python3
"""
PID Tuning GUI for Drone with Real-time Telemetry Plotting
PyQt5 interface for sending PID coefficients and viewing telemetry
"""

import sys
import struct
import serial
import serial.tools.list_ports
from collections import deque
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QLabel, QLineEdit,
                             QPushButton, QComboBox, QTextEdit, QGroupBox,
                             QTabWidget, QDoubleSpinBox, QMessageBox, QCheckBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette

import pyqtgraph as pg


class TelemetryThread(QThread):
    """Thread for handling serial communication and telemetry"""
    response_received = pyqtSignal(str)
    telemetry_received = pyqtSignal(dict)

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
        self.buffer = bytearray()

    def run(self):
        """Listen for responses and telemetry from the bridge"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                # Read available bytes
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.buffer.extend(data)

                    # Process complete 32-byte packets
                    while len(self.buffer) >= 32:
                        packet = self.buffer[:32]
                        self.buffer = self.buffer[32:]

                        # Parse packet
                        try:
                            values = struct.unpack('<16H', packet)

                            # Check if it's telemetry (cmd_id = 3 at index 15)
                            if values[15] == 3:
                                # Convert to signed where needed
                                telemetry = {
                                    'pitch_angle': struct.unpack('<h', struct.pack('<H', values[0]))[0],
                                    'roll_angle': struct.unpack('<h', struct.pack('<H', values[1]))[0],
                                    'yaw_angle': struct.unpack('<h', struct.pack('<H', values[2]))[0],
                                    'pitch_rate': struct.unpack('<h', struct.pack('<H', values[3]))[0],
                                    'roll_rate': struct.unpack('<h', struct.pack('<H', values[4]))[0],
                                    'yaw_rate': struct.unpack('<h', struct.pack('<H', values[5]))[0],
                                    'mode': values[6],
                                    'pitch_pid': struct.unpack('<h', struct.pack('<H', values[7]))[0],
                                    'roll_pid': struct.unpack('<h', struct.pack('<H', values[8]))[0],
                                    'yaw_pid': struct.unpack('<h', struct.pack('<H', values[9]))[0],
                                    'motor_fl': values[10],
                                    'motor_bl': values[11],
                                    'motor_br': values[12],
                                    'motor_fr': values[13],
                                    'battery': values[14],
                                    'timestamp': datetime.now()
                                }
                                self.telemetry_received.emit(telemetry)
                        except Exception as e:
                            self.response_received.emit(f"Parse error: {e}")

            except Exception as e:
                self.response_received.emit(f"Error: {e}")
            self.msleep(10)

    def stop(self):
        self.running = False


class PlotWidget(QWidget):
    """Widget for displaying real-time plots"""

    def __init__(self, title, labels, colors, max_points=500):
        super().__init__()
        self.max_points = max_points
        self.labels = labels
        self.colors = colors

        # Data storage
        self.time_data = deque(maxlen=max_points)
        self.data = {label: deque(maxlen=max_points) for label in labels}
        self.start_time = datetime.now()

        # Create layout
        layout = QVBoxLayout(self)

        # Create plot widget
        self.plot_widget = pg.PlotWidget(title=title)
        self.plot_widget.setBackground('w')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (s)')

        # Create plot curves
        self.curves = {}
        for label, color in zip(labels, colors):
            pen = pg.mkPen(color=color, width=2)
            self.curves[label] = self.plot_widget.plot([], [], pen=pen, name=label)

        # Add legend
        self.plot_widget.addLegend()

        layout.addWidget(self.plot_widget)

        # Control buttons
        control_layout = QHBoxLayout()

        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.toggle_pause)
        control_layout.addWidget(self.pause_btn)

        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_data)
        control_layout.addWidget(clear_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        self.paused = False

    def toggle_pause(self):
        self.paused = self.pause_btn.isChecked()
        self.pause_btn.setText("Resume" if self.paused else "Pause")

    def clear_data(self):
        self.time_data.clear()
        for label in self.labels:
            self.data[label].clear()
        self.start_time = datetime.now()
        self.update_plot()

    def add_data(self, data_dict):
        if self.paused:
            return

        # Calculate elapsed time
        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.time_data.append(elapsed)

        # Add data for each label
        for label in self.labels:
            if label in data_dict:
                self.data[label].append(data_dict[label])
            else:
                self.data[label].append(0)

        self.update_plot()

    def update_plot(self):
        time_list = list(self.time_data)
        for label in self.labels:
            data_list = list(self.data[label])
            self.curves[label].setData(time_list, data_list)


class PIDTunerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.telemetry_thread = None

        # Track PID configurations
        self.pid_config = {
            0: {0: None, 1: None},
            1: {0: None, 1: None},
        }

        self.axis_names = ['Pitch / Roll', 'Yaw']
        self.mode_names = ['Rate', 'Angle']

        # Telemetry storage
        self.telemetry_enabled = True

        # Telemetry timeout tracking
        self.last_telemetry_time = None
        self.telemetry_timeout_ms = 2000  # 2 seconds timeout

        self.init_ui()

        # Start timeout checker timer
        self.timeout_timer = QTimer()
        self.timeout_timer.timeout.connect(self.check_telemetry_timeout)
        self.timeout_timer.start(500)  # Check every 500ms

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Drone PID Tuner & Telemetry')
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Console tab
        console_tab = self.create_console_tab()

        # Connection section
        connection_group = self.create_connection_section()
        main_layout.addWidget(connection_group)

        # Main tabs
        self.main_tabs = QTabWidget()

        # Telemetry tab
        telemetry_tab = self.create_telemetry_tab()
        self.main_tabs.addTab(telemetry_tab, "📊 Telemetry")

        # PID tuning tabs
        pid_tabs = QTabWidget()
        self.pitch_roll_tab = self.create_axis_tab(0)
        self.yaw_tab = self.create_axis_tab(1)
        pid_tabs.addTab(self.pitch_roll_tab, "Pitch + Roll")
        pid_tabs.addTab(self.yaw_tab, "Yaw")
        self.main_tabs.addTab(pid_tabs, "⚙️ PID Tuning")

        # Add console tab to layout now
        self.main_tabs.addTab(console_tab, "📝 Console")

        main_layout.addWidget(self.main_tabs)

        self.log_message("PID Tuner started. Please connect to serial port.")

    def create_telemetry_tab(self):
        """Create the telemetry visualization tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Status bar
        status_layout = QHBoxLayout()
        self.telemetry_status = QLabel("⚫ No telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: #888;")
        status_layout.addWidget(self.telemetry_status)

        status_layout.addStretch()

        # Recording control
        self.record_checkbox = QCheckBox("Record Data")
        self.record_checkbox.setChecked(True)
        status_layout.addWidget(self.record_checkbox)

        layout.addLayout(status_layout)

        # Create plot tabs
        plot_tabs = QTabWidget()

        # Angles plot
        self.angles_plot = PlotWidget(
            "Attitude Angles",
            ["Pitch", "Roll", "Yaw"],
            ['r', 'g', 'b']
        )
        plot_tabs.addTab(self.angles_plot, "Angles")

        # Rates plot
        self.rates_plot = PlotWidget(
            "Angular Rates",
            ["Pitch Rate", "Roll Rate", "Yaw Rate"],
            ['r', 'g', 'b']
        )
        plot_tabs.addTab(self.rates_plot, "Rates")

        # PID outputs plot
        self.pid_plot = PlotWidget(
            "PID Outputs",
            ["Pitch PID", "Roll PID", "Yaw PID"],
            ['r', 'g', 'b']
        )
        plot_tabs.addTab(self.pid_plot, "PID Outputs")

        # Motors plot
        self.motors_plot = PlotWidget(
            "Motor Speeds",
            ["FL", "BL", "BR", "FR"],
            ['r', 'orange', 'purple', 'cyan']
        )
        plot_tabs.addTab(self.motors_plot, "Motors")

        layout.addWidget(plot_tabs)

        # Current values display
        values_group = QGroupBox("Current Values")
        values_layout = QGridLayout()

        self.value_labels = {}
        labels = [
            ("Pitch Angle:", "pitch_angle", "°"),
            ("Roll Angle:", "roll_angle", "°"),
            ("Yaw Angle:", "yaw_angle", "°"),
            ("Pitch Rate:", "pitch_rate", "°/s"),
            ("Roll Rate:", "roll_rate", "°/s"),
            ("Yaw Rate:", "yaw_rate", "°/s"),
            ("Mode:", "mode", ""),
            ("Battery:", "battery", "V"),
        ]

        for i, (label_text, key, unit) in enumerate(labels):
            row, col = i // 4, (i % 4) * 2
            values_layout.addWidget(QLabel(label_text), row, col)
            value_label = QLabel("--")
            value_label.setStyleSheet("font-weight: bold; color: #2196F3;")
            self.value_labels[key] = (value_label, unit)
            values_layout.addWidget(value_label, row, col + 1)

        values_group.setLayout(values_layout)
        layout.addWidget(values_group)

        return widget

    def create_connection_section(self):
        """Create the serial port connection section"""
        group = QGroupBox("Serial Connection")
        layout = QHBoxLayout()

        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMaxVisibleItems(10)
        self.port_combo.view().setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.port_combo.setMinimumWidth(300)
        self.refresh_ports()
        layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)

        layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '115200', '230400', '460800'])
        self.baudrate_combo.setCurrentText('115200')
        layout.addWidget(self.baudrate_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        layout.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.status_label)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def create_axis_tab(self, axis):
        """Create a tab for configuring an axis"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        rate_group = self.create_pid_group(axis, 0, "Rate Mode PID")
        layout.addWidget(rate_group)

        if axis != 1:
            angle_group = self.create_pid_group(axis, 1, "Angle Mode PID")
            layout.addWidget(angle_group)
        else:
            note_label = QLabel("Note: Yaw only supports Rate mode")
            note_label.setStyleSheet("color: #666; font-style: italic;")
            layout.addWidget(note_label)

        layout.addStretch()
        return widget

    def create_pid_group(self, axis, mode, title):
        """Create a PID configuration group"""
        group = QGroupBox(title)
        layout = QGridLayout()

        current_label = QLabel("Current:")
        layout.addWidget(current_label, 0, 0)

        current_value = QLabel("DEFAULT")
        current_value.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(current_value, 0, 1, 1, 3)
        setattr(self, f'current_label_{axis}_{mode}', current_value)

        layout.addWidget(QLabel("Kp:"), 1, 0)
        kp_spin = QDoubleSpinBox()
        kp_spin.setRange(0, 100)
        kp_spin.setDecimals(4)
        kp_spin.setSingleStep(0.001)
        kp_spin.setValue(0.0)
        layout.addWidget(kp_spin, 1, 1)

        layout.addWidget(QLabel("Ki:"), 1, 2)
        ki_spin = QDoubleSpinBox()
        ki_spin.setRange(0, 100)
        ki_spin.setDecimals(4)
        ki_spin.setSingleStep(0.001)
        ki_spin.setValue(0.0)
        layout.addWidget(ki_spin, 1, 3)

        layout.addWidget(QLabel("Kd:"), 2, 0)
        kd_spin = QDoubleSpinBox()
        kd_spin.setRange(0, 100)
        kd_spin.setDecimals(4)
        kd_spin.setSingleStep(0.001)
        kd_spin.setValue(0.0)
        layout.addWidget(kd_spin, 2, 1)

        send_btn = QPushButton(f"Send to Drone")
        send_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        send_btn.clicked.connect(lambda: self.send_pid(axis, mode, kp_spin.value(),
                                                       ki_spin.value(), kd_spin.value()))
        layout.addWidget(send_btn, 2, 2, 1, 2)

        setattr(self, f'kp_spin_{axis}_{mode}', kp_spin)
        setattr(self, f'ki_spin_{axis}_{mode}', ki_spin)
        setattr(self, f'kd_spin_{axis}_{mode}', kd_spin)

        group.setLayout(layout)
        return group

    def create_console_tab(self):
        """Create console log tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier", 9))
        layout.addWidget(self.log_text)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        layout.addWidget(clear_btn)

        return widget

    def refresh_ports(self):
        """Refresh available serial ports"""
        self.port_combo.clear()
        try:
            ports = serial.tools.list_ports.comports()
            valid_ports = []

            for port in ports:
                if not port.device:
                    continue
                if not port.description or port.description.strip().upper() == "N/A":
                    continue
                valid_ports.append(port)
                display_text = f"{port.device} ({port.description[:40]})"
                self.port_combo.addItem(display_text, port.device)

            if not valid_ports:
                self.port_combo.addItem("No valid ports found", None)
                self.log_message("No valid serial ports detected.")
        except Exception as e:
            self.port_combo.addItem("Error scanning ports", None)
            self.log_message(f"Error scanning ports: {e}")

    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        """Connect to selected serial port"""
        port = self.port_combo.currentData()
        baudrate = int(self.baudrate_combo.currentText())

        if not port:
            QMessageBox.warning(self, "Connection Error", "No valid port selected.")
            return

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.log_message(f"Connected to {port} at {baudrate} baud")

            # Start telemetry thread
            self.telemetry_thread = TelemetryThread(self.serial_port)
            self.telemetry_thread.response_received.connect(self.handle_response)
            self.telemetry_thread.telemetry_received.connect(self.handle_telemetry)
            self.telemetry_thread.start()

            # Update UI
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")

        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect: {e}")
            self.log_message(f"Connection failed: {e}")

    def disconnect(self):
        """Disconnect from serial port"""
        if self.telemetry_thread:
            self.telemetry_thread.stop()
            self.telemetry_thread.wait()

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_message("Disconnected from serial port")

        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")

        self.telemetry_status.setText("⚫ No telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: #888;")
        self.last_telemetry_time = None

    def check_telemetry_timeout(self):
        """Check if telemetry has timed out"""
        if self.last_telemetry_time is None:
            return

        if not self.serial_port or not self.serial_port.is_open:
            self.last_telemetry_time = None
            return

        elapsed_ms = (datetime.now() - self.last_telemetry_time).total_seconds() * 1000

        if elapsed_ms > self.telemetry_timeout_ms:
            self.telemetry_status.setText("🔴 Telemetry timeout")
            self.telemetry_status.setStyleSheet("font-weight: bold; color: #f44336;")

    def send_pid(self, axis, mode, kp, ki, kd):
        """Send PID configuration to drone"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Not Connected", "Please connect to serial port first.")
            return

        if axis == 1 and mode != 0:
            QMessageBox.warning(self, "Invalid Mode", "Yaw only supports Rate mode.")
            return

        try:
            packet = struct.pack(
                '<HHHfff',
                2, axis, mode, kp, ki, kd
            )

            self.serial_port.write(packet)
            self.serial_port.flush()

            self.pid_config[axis][mode] = {'kp': kp, 'ki': ki, 'kd': kd}

            current_label = getattr(self, f'current_label_{axis}_{mode}')
            current_label.setText(f"Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")
            current_label.setStyleSheet("color: #2196F3; font-weight: bold;")

            axis_name = self.axis_names[axis]
            mode_name = self.mode_names[mode]
            self.log_message(f"→ Sent {axis_name} {mode_name}: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")

        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")

    def handle_telemetry(self, data):
        """Handle received telemetry data"""
        if not self.record_checkbox.isChecked():
            return

        # Update timestamp
        self.last_telemetry_time = datetime.now()

        # Update status
        self.telemetry_status.setText("🟢 Receiving telemetry")
        self.telemetry_status.setStyleSheet("font-weight: bold; color: green;")

        # Update value labels
        for key, (label, unit) in self.value_labels.items():
            if key in data:
                if key == 'mode':
                    text = "STABILISE" if data[key] else "ACRO"
                elif key == 'battery':
                    # voltage sent as mV
                    voltage = (data[key] / 1000)
                    text = f"{voltage:.2f}{unit}"
                else:
                    text = f"{data[key]}{unit}"
                label.setText(text)

        # Update plots
        angles_data = {
            "Pitch": data['pitch_angle'],
            "Roll": data['roll_angle'],
            "Yaw": data['yaw_angle']
        }
        self.angles_plot.add_data(angles_data)

        rates_data = {
            "Pitch Rate": data['pitch_rate'],
            "Roll Rate": data['roll_rate'],
            "Yaw Rate": data['yaw_rate']
        }
        self.rates_plot.add_data(rates_data)

        pid_data = {
            "Pitch PID": data['pitch_pid'],
            "Roll PID": data['roll_pid'],
            "Yaw PID": data['yaw_pid']
        }
        self.pid_plot.add_data(pid_data)

        motors_data = {
            "FL": data['motor_fl'],
            "BL": data['motor_bl'],
            "BR": data['motor_br'],
            "FR": data['motor_fr']
        }
        self.motors_plot.add_data(motors_data)

    def handle_response(self, response):
        """Handle text response from serial port"""
        self.log_message(f"← {response}")

    def log_message(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.append(f"[{timestamp}] {message}")
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        """Handle window close event"""
        self.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = PIDTunerGUI()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()