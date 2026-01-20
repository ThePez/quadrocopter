"""
PID Tuning Interface for Drone
Sends PID coefficients via serial to ESP32-S3 bridge
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys

class PIDTuner:
    def __init__(self, port=None, baudrate=115200):
        """Initialize serial connection to ESP32-S3 bridge"""
        
        # If no port specified, let user choose
        if port is None:
            port = self.select_port()
            if port is None:
                print("No port selected. Exiting.")
                sys.exit(1)
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for connection to establish
            print(f"Connected to {port} at {baudrate} baud\n")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)
        
        # Track PID configurations
        # Format: self.pid_config[axis][mode] = {'kp': val, 'ki': val, 'kd': val}
        self.pid_config = {
            0: {0: None, 1: None},  # Pitch: Rate, Angle
            1: {0: None, 1: None},  # Roll: Rate, Angle
            2: {0: None, 1: None},  # Yaw: Rate (mode 1 not used)
        }
    
    @staticmethod
    def select_port():
        """List available serial ports and let user select one"""
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            print("No serial ports found!")
            return None
        
        print("\n" + "="*70)
        print("                    AVAILABLE SERIAL PORTS")
        print("="*70)
        
        # Display ports with details
        for i, port in enumerate(ports, 1):
            print(f"\n[{i}] {port.device}")
            print(f"    Description: {port.description}")
            if port.manufacturer:
                print(f"    Manufacturer: {port.manufacturer}")
            if port.serial_number:
                print(f"    Serial Number: {port.serial_number}")
            if port.hwid:
                print(f"    Hardware ID: {port.hwid}")
        
        print("\n" + "="*70)
        
        # Get user selection
        while True:
            try:
                choice = input(f"\nSelect port (1-{len(ports)}) or 'q' to quit: ").strip().lower()
                
                if choice == 'q':
                    return None
                
                choice_num = int(choice)
                if 1 <= choice_num <= len(ports):
                    selected_port = ports[choice_num - 1].device
                    print(f"Selected: {selected_port}")
                    return selected_port
                else:
                    print(f"Invalid choice. Please enter 1-{len(ports)}")
                    
            except ValueError:
                print("Invalid input. Please enter a number or 'q'")
            except KeyboardInterrupt:
                print("\nCancelled.")
                return None
    
    def send_pid_packet(self, axis, mode, kp, ki, kd):
        """
        Send PID configuration packet
        
        Args:
            axis: 0=Pitch, 1=Roll, 2=Yaw
            mode: 0=Rate, 1=Angle
            kp, ki, kd: PID coefficients (float)
        """
        # Packet structure (32 bytes to match your existing packets):
        # uint16_t command_id = 2 (PID update)
        # uint16_t axis (0=pitch, 1=roll, 2=yaw)
        # uint16_t mode (0=rate, 1=angle)
        # float kp, ki, kd
        # uint16_t reserved[9]
        
        packet = struct.pack(
            '<HHHfffHHHHHHHHH',  # Little-endian format
            2,                    # command_id = 2 for PID update
            axis,                 # axis
            mode,                 # mode (rate/angle)
            kp, ki, kd,          # PID coefficients
            0, 0, 0, 0, 0, 0, 0, 0, 0  # reserved
        )
        
        self.ser.write(packet)
        self.ser.flush()
        
        # Store the configuration
        self.pid_config[axis][mode] = {'kp': kp, 'ki': ki, 'kd': kd}
        
        # Wait for acknowledgment
        response = self.ser.readline().decode('utf-8').strip()
        if response:
            print(f"Response: {response}")
    
    def display_current_config(self):
        """Display current PID configuration for all axes/modes"""
        axis_names = ['Pitch', 'Roll', 'Yaw']
        mode_names = ['Rate', 'Angle']
        
        print("\n" + "="*70)
        print("                    CURRENT PID CONFIGURATION")
        print("="*70)
        
        for axis in range(3):
            print(f"\n{axis_names[axis]}:")
            print("-" * 70)
            
            for mode in range(2):
                # Skip Yaw Angle mode
                if axis == 2 and mode == 1:
                    continue
                
                config = self.pid_config[axis][mode]
                
                if config is None:
                    status = "DEFAULT (not modified)"
                    print(f"  {mode_names[mode]:6s}: {status}")
                else:
                    print(f"  {mode_names[mode]:6s}: Kp={config['kp']:.3f}, Ki={config['ki']:.3f}, Kd={config['kd']:.3f}")
        
        print("="*70 + "\n")
    
    def interactive_mode(self):
        """Interactive terminal interface for PID tuning"""
        axis_names = ['Pitch', 'Roll', 'Yaw']
        mode_names = ['Rate', 'Angle']
        
        print("\n" + "="*50)
        print("    DRONE PID TUNING INTERFACE")
        print("="*50)
        
        # Show initial config
        self.display_current_config()
        
        while True:
            print("\n--- Select Control Loop ---")
            print("Axis: 0=Pitch, 1=Roll, 2=Yaw")
            print("Mode: 0=Rate, 1=Angle (Yaw is always Rate)")
            print("Commands: 'q' to quit, 's' to show current config\n")
            
            try:
                # Get axis
                axis_input = input("Enter axis (0-2) or command: ").strip().lower()
                if axis_input == 'q':
                    break
                if axis_input == 's':
                    self.display_current_config()
                    continue
                    
                axis = int(axis_input)
                if axis not in [0, 1, 2]:
                    print("Invalid axis. Must be 0, 1, or 2.")
                    continue
                
                # Get mode (force Rate for Yaw)
                if axis == 2:
                    mode = 0  # Yaw is always Rate mode
                    print("Yaw axis - Rate mode automatically selected")
                else:
                    mode_input = input("Enter mode (0=Rate, 1=Angle): ").strip()
                    mode = int(mode_input)
                    if mode not in [0, 1]:
                        print("Invalid mode. Must be 0 or 1.")
                        continue
                
                # Show current values for this axis/mode
                current = self.pid_config[axis][mode]
                if current:
                    print(f"\nCurrent {axis_names[axis]} {mode_names[mode]} PID:")
                    print(f"  Kp={current['kp']:.3f}, Ki={current['ki']:.3f}, Kd={current['kd']:.3f}")
                else:
                    print(f"\nCurrent {axis_names[axis]} {mode_names[mode]} PID: DEFAULT (not modified)")
                
                # Get PID values
                print(f"\n--- Enter New {axis_names[axis]} {mode_names[mode]} PID ---")
                kp = float(input("Enter Kp: "))
                ki = float(input("Enter Ki: "))
                kd = float(input("Enter Kd: "))
                
                # Confirm
                print(f"\nSending to drone:")
                print(f"  Axis: {axis_names[axis]}")
                print(f"  Mode: {mode_names[mode]}")
                print(f"  Kp={kp}, Ki={ki}, Kd={kd}")
                
                confirm = input("Send? (y/n): ").strip().lower()
                if confirm == 'y':
                    self.send_pid_packet(axis, mode, kp, ki, kd)
                    print("PID values sent!")
                    
                    # Show updated config
                    self.display_current_config()
                else:
                    print("Cancelled.")
                    
            except ValueError as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                print("\nExiting...")
                break
        
        self.close()
    
    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    # Auto-detect and select port (or specify manually if needed)
    # tuner = PIDTuner(port='/dev/ttyACM0', baudrate=115200)  # Manual override
    tuner = PIDTuner(baudrate=115200)  # Auto-detect
    tuner.interactive_mode()