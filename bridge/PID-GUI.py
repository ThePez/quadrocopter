#!/usr/bin/env python3
"""
PID Tuning GUI for Drone
PyQt5 interface for sending PID coefficients to drone via ESP32 bridge
"""

import sys
import struct
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QLineEdit, 
                             QPushButton, QComboBox, QTextEdit, QGroupBox,
                             QTabWidget, QDoubleSpinBox, QMessageBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette


class SerialThread(QThread):
    """Thread for handling serial communication"""
    response_received = pyqtSignal(str)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
    
    def run(self):
        """Listen for responses from the bridge"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode('utf-8').strip()
                    if response:
                        self.response_received.emit(response)
            except Exception as e:
                self.response_received.emit(f"Error: {e}")
            self.msleep(50)
    
    def stop(self):
        self.running = False


class PIDTunerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.serial_thread = None
        
        # Track PID configurations
        self.pid_config = {
            0: {0: None, 1: None},  # Pitch: Rate, Angle
            1: {0: None, 1: None},  # Roll: Rate, Angle
            2: {0: None, 1: None},  # Yaw: Rate (mode 1 not used)
        }
        
        self.axis_names = ['Pitch', 'Roll', 'Yaw']
        self.mode_names = ['Rate', 'Angle']
        
        self.init_ui()
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Drone PID Tuner')
        self.setGeometry(100, 100, 900, 700)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Connection section
        connection_group = self.create_connection_section()
        main_layout.addWidget(connection_group)
        
        # Tabs for each axis
        self.tabs = QTabWidget()
        
        # Create tabs for Pitch, Roll, Yaw
        self.pitch_tab = self.create_axis_tab(0)
        self.roll_tab = self.create_axis_tab(1)
        self.yaw_tab = self.create_axis_tab(2)
        
        self.tabs.addTab(self.pitch_tab, "Pitch")
        self.tabs.addTab(self.roll_tab, "Roll")
        self.tabs.addTab(self.yaw_tab, "Yaw")
        
        main_layout.addWidget(self.tabs)
        
        # Log/Console output
        log_group = self.create_log_section()
        main_layout.addWidget(log_group)
        
        self.log_message("PID Tuner started. Please connect to serial port.")
    
    def create_connection_section(self):
        """Create the serial port connection section"""
        group = QGroupBox("Serial Connection")
        layout = QHBoxLayout()
        
        # Port selection
        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMaxVisibleItems(10)  # Limit visible items to 10
        self.port_combo.view().setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.port_combo.setMinimumWidth(300)  # Ensure enough width for descriptions
        self.refresh_ports()
        layout.addWidget(self.port_combo)
        
        # Refresh button
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        # Baudrate
        layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '115200', '230400', '460800'])
        self.baudrate_combo.setCurrentText('115200')
        layout.addWidget(self.baudrate_combo)
        
        # Connect/Disconnect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        layout.addWidget(self.connect_btn)
        
        # Connection status
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_axis_tab(self, axis):
        """Create a tab for configuring an axis (Pitch, Roll, or Yaw)"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Rate mode section
        rate_group = self.create_pid_group(axis, 0, "Rate Mode PID")
        layout.addWidget(rate_group)
        
        # Angle mode section (not for Yaw)
        if axis != 2:
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
        
        # Current values display
        current_label = QLabel("Current:")
        layout.addWidget(current_label, 0, 0)
        
        current_value = QLabel("DEFAULT")
        current_value.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(current_value, 0, 1, 1, 3)
        
        # Store reference for updating
        setattr(self, f'current_label_{axis}_{mode}', current_value)
        
        # Kp input
        layout.addWidget(QLabel("Kp:"), 1, 0)
        kp_spin = QDoubleSpinBox()
        kp_spin.setRange(0, 100)
        kp_spin.setDecimals(3)
        kp_spin.setSingleStep(0.01)
        kp_spin.setValue(0.0)
        layout.addWidget(kp_spin, 1, 1)
        
        # Ki input
        layout.addWidget(QLabel("Ki:"), 1, 2)
        ki_spin = QDoubleSpinBox()
        ki_spin.setRange(0, 100)
        ki_spin.setDecimals(3)
        ki_spin.setSingleStep(0.001)
        ki_spin.setValue(0.0)
        layout.addWidget(ki_spin, 1, 3)
        
        # Kd input
        layout.addWidget(QLabel("Kd:"), 2, 0)
        kd_spin = QDoubleSpinBox()
        kd_spin.setRange(0, 100)
        kd_spin.setDecimals(3)
        kd_spin.setSingleStep(0.001)
        kd_spin.setValue(0.0)
        layout.addWidget(kd_spin, 2, 1)
        
        # Send button
        send_btn = QPushButton(f"Send to Drone")
        send_btn.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        send_btn.clicked.connect(lambda: self.send_pid(axis, mode, kp_spin.value(), 
                                                        ki_spin.value(), kd_spin.value()))
        layout.addWidget(send_btn, 2, 2, 1, 2)
        
        # Store references to spin boxes
        setattr(self, f'kp_spin_{axis}_{mode}', kp_spin)
        setattr(self, f'ki_spin_{axis}_{mode}', ki_spin)
        setattr(self, f'kd_spin_{axis}_{mode}', kd_spin)
        
        group.setLayout(layout)
        return group
        
    def create_log_section(self):
        """Create log/console section"""
        group = QGroupBox("Console Log")
        layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier", 9))
        
        layout.addWidget(self.log_text)
        
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        layout.addWidget(clear_btn)
        
        group.setLayout(layout)
        return group
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        self.port_combo.clear()

        ports = serial.tools.list_ports.comports()
        valid_ports = []

        for port in ports:
            # Filter out junk / N/A ports
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

    
    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_port and self.serial_port.is_open:
            # Disconnect
            self.disconnect()
        else:
            # Connect
            self.connect()
    
    def connect(self):
        """Connect to selected serial port"""
        port = self.port_combo.currentData()
        baudrate = int(self.baudrate_combo.currentText())
        
        if not port:
            QMessageBox.warning(self, "Connection Error", "No valid port selected.")
            return
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.log_message(f"Connected to {port} at {baudrate} baud")
            
            # Start serial thread
            self.serial_thread = SerialThread(self.serial_port)
            self.serial_thread.response_received.connect(self.handle_response)
            self.serial_thread.start()
            
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
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_message("Disconnected from serial port")
        
        # Update UI
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
    
    def send_pid(self, axis, mode, kp, ki, kd):
        """Send PID configuration to drone"""
        if not self.serial_port or not self.serial_port.is_open:
            QMessageBox.warning(self, "Not Connected", "Please connect to serial port first.")
            return
        
        # Yaw only supports Rate mode
        if axis == 2 and mode != 0:
            QMessageBox.warning(self, "Invalid Mode", "Yaw only supports Rate mode.")
            return
        
        try:
            # Create packet
            packet = struct.pack(
                '<HHHfffHHHHHHHHH',
                2,  # command_id = 2 for PID update
                axis,
                mode,
                kp, ki, kd,
                0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            
            self.serial_port.write(packet)
            self.serial_port.flush()
            
            # Store configuration
            self.pid_config[axis][mode] = {'kp': kp, 'ki': ki, 'kd': kd}
            
            # Update current value label
            current_label = getattr(self, f'current_label_{axis}_{mode}')
            current_label.setText(f"Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
            current_label.setStyleSheet("color: #2196F3; font-weight: bold;")
                        
            # Log
            axis_name = self.axis_names[axis]
            mode_name = self.mode_names[mode]
            self.log_message(f"→ Sent {axis_name} {mode_name}: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
            
        except Exception as e:
            QMessageBox.critical(self, "Send Error", f"Failed to send: {e}")
            self.log_message(f"Send failed: {e}")
    
    def handle_response(self, response):
        """Handle response from serial port"""
        self.log_message(f"← {response}")
        
    def log_message(self, message):
        """Add message to log"""
        self.log_text.append(message)
        # Auto-scroll to bottom
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = PIDTunerGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()