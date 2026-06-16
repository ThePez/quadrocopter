# Quadrocopter

Custom quadrocopter firmware written in C using the ESP-IDF framework. The system is split across three ESP32 microcontrollers — a drone flight controller, a handheld remote, and a PC bridge — coordinating over ESP-NOW. A PyQt5 desktop app provides real-time telemetry display and live PID tuning.

---

## System Architecture

```
┌──────────────┐       ESP-NOW        ┌──────────────┐       UART        ┌──────────────┐
│   Remote     │ ──────────────────►  │    Drone     │ ◄───────────────► │    Bridge    │ ◄──► PC (PID-GUI.py)
│   (ESP32)    │ ◄──────────────────  │   (ESP32)    │                   │  (ESP32-S3)  │
└──────────────┘                      └──────────────┘                   └──────────────┘
```

| Node   | MCU       | Role                                                          |
|--------|-----------|---------------------------------------------------------------|
| Drone  | ESP32     | Flight controller — PID loops, motor mixing, sensor fusion    |
| Remote | ESP32     | Joystick transmitter — reads ADC axes, sends control packets  |
| Bridge | ESP32-S3  | Ground station interface — relays telemetry/tuning over UART  |

---

## Hardware

### Drone (ESP32)
| Component         | Interface | Pins / Notes                           |
|-------------------|-----------|----------------------------------------|
| 4× ESC            | MCPWM     | GPIOs 17, 16, 27, 26 — 50 Hz RC PWM   |
| BNO085 IMU        | I2C       | 9-DOF attitude sensor                  |
| MCP3208 ADC       | SPI       | 8-ch 12-bit ADC for battery monitoring |
| Battery monitor   | GPIO 34   | Voltage divider (46.4 kΩ + 9.84 kΩ)   |

### Remote (ESP32)
| Component       | Interface | Notes                               |
|-----------------|-----------|-------------------------------------|
| 2× Joystick     | SPI/ADC   | MCP3208 channels 0–3                |
| Mode button     | GPIO 25   | Toggles ACRO ↔ STABILISE            |
| Emergency stop  | GPIO 32   | Cuts motors immediately             |
| Status LED      | GPIO 2    | Off = ACRO, On = STABILISE          |

### Bridge (ESP32-S3)
| Component | Interface | Notes                         |
|-----------|-----------|-------------------------------|
| UART      | 115200 baud | USB-serial to host PC       |

---

## Flight Control

### Modes

**STABILISE** — Cascaded PID: outer angle loop feeding into inner rate loop. Good for beginners; the drone self-levels.

**ACRO** — Rate-only PID. Direct stick-to-rate mapping for full 3D flight.

### PID Loops

| Controller     | Kp    | Ki | Kd | Rate  |
|----------------|-------|----|----|-------|
| Pitch rate     | 0.175 | 0  | 0  | 400 Hz |
| Roll rate      | 0.175 | 0  | 0  | 400 Hz |
| Yaw rate       | 0.1   | 0  | 0  | 400 Hz |
| Pitch angle    | 1.0   | 0  | 0  | 400 Hz |
| Roll angle     | 1.0   | 0  | 0  | 400 Hz |

PID coefficients can be updated live from the desktop app without reflashing.

### Motor Mixing (X-Configuration)

```
Front-Left  (A): throttle − pitch + roll − yaw
Back-Left   (B): throttle + pitch + roll + yaw
Back-Right  (C): throttle + pitch − roll − yaw
Front-Right (D): throttle − pitch − roll + yaw
```

Each motor output is clamped to [1050, 2000] µs.

### Failsafes

| Condition                         | Response                                                |
|-----------------------------------|---------------------------------------------------------|
| Pitch or roll > ±30°              | Motors killed; must throttle down and level to re-arm  |
| No remote signal for > 1 s        | Motors killed immediately                               |
| Battery ≤ 14 V (warn) / 13.6 V   | Warning / critical alert via telemetry                 |

---

## Wireless Communication (ESP-NOW)

- **Channel**: 9, long-range mode enabled
- **Encryption**: PMK + LMK configured per peer
- **Remote → Drone**: 20 Hz control packets (throttle, pitch, roll, yaw, flight mode)
- **Drone → Bridge**: 10 Hz telemetry packets (angles, rates, PID outputs, motor PWM, battery mV)
- **Bridge → Drone**: PID tuning packets forwarded from PC

MAC addresses are configured in `components/espnow-comm/espnow_comm.c`.

---

## Repository Structure

```
quadrocopter/
├── drone/              # Flight controller firmware (ESP32)
│   └── main/drone.c
├── remote/             # Remote control firmware (ESP32)
│   └── main/remote.c
├── bridge/             # Bridge firmware (ESP32-S3)
│   ├── main/bridge.c
│   └── PID-GUI.py      # Desktop telemetry & tuning app
└── components/
    ├── motors/         # MCPWM ESC driver
    ├── imu/            # BNO085 driver (quaternion → Euler, gyro filter)
    ├── mcp3208/        # SPI ADC driver
    ├── espnow-comm/    # ESP-NOW peer management & queues
    ├── common_functions/ # mapf(), constrainf(), SPI/I2C bus init
    └── esp32_BNO08x/   # Third-party BNO085 SH2-HAL library
```

---

## Building & Flashing

Requires the [ESP-IDF toolchain](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

```bash
# Drone
cd drone && idf.py build flash monitor

# Remote
cd remote && idf.py build flash monitor

# Bridge
cd bridge && idf.py build flash monitor
```

Each target has its own `sdkconfig` and `CMakeLists.txt`. The bridge target must be built with the ESP32-S3 IDF target set:

```bash
cd bridge && idf.py set-target esp32s3 && idf.py build flash
```

---

## Desktop App (PID-GUI.py)

A PyQt5 application for real-time monitoring and PID tuning.

**Dependencies:**
```bash
pip install PyQt5 pyqtgraph pyserial
```

**Run:**
```bash
cd bridge && python3 PID-GUI.py
```

**Features:**
- Live plots of angles, rates, PID outputs, and motor speeds (500-point rolling window)
- Battery voltage display
- Live Kp/Ki/Kd adjustment for rate and angle loops on each axis — changes are sent to the drone immediately over the bridge

---

## Operation

### Arming
Press **both** the mode button and emergency button simultaneously on the remote to arm/disarm.

### Flight
- **Left stick**: throttle (up/down) and yaw (left/right)
- **Right stick**: pitch (forward/back) and roll (left/right)
- **Mode button** (while armed): toggle ACRO ↔ STABILISE
- **Emergency button**: immediate motor shutdown (disarms)

### PID Tuning
1. Connect the bridge to your PC via USB
2. Run `python3 PID-GUI.py` and select the correct serial port
3. Adjust gains while the drone is flying; changes take effect within one telemetry cycle

---

## Implementation Notes

- **Dual-core**: Core 0 handles incoming remote packets; Core 1 runs the 400 Hz PID loop at elevated FreeRTOS priority
- **IMU filtering**: Gyroscope data is low-pass filtered (α = 0.4) with a 10 °/s deadband to reduce noise
- **ADC calibration**: Battery voltage uses ESP-IDF ADC calibration for mV-level accuracy
- **Memory**: All dynamic allocations checked at startup; device restarts on malloc failure

---

## License

Apache 2.0
