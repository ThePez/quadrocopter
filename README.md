# Quadrocopter

Custom quadrocopter firmware written in C using the ESP-IDF framework. The system is split across three ESP32 microcontrollers — a drone flight controller, a handheld remote, and a PC bridge — coordinating over ESP-NOW. A PyQt5 desktop app provides real-time telemetry display and live PID tuning.

---

## System Architecture

```
┌──────────────┐       ESP-NOW        ┌──────────────┐       ESP-NOW     ┌──────────────┐        UART       ┌──────────────┐
│   Remote     │ ──────────────────►  │    Drone     │ ────────────────► │    Bridge    │ ────────────────► |      PC      |
│   (ESP32)    │                      │   (ESP32)    │ ◄──────────────── │  (ESP32-S3)  │ ◄──────────────── | (config-app) |
└──────────────┘                      └──────────────┘                   └──────────────┘                   └──────────────┘
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
| No remote signal for > 2 s        | Motors killed immediately                               |
| Battery ≤ 14 V (warn) / 13.6 V   | Warning / critical alert via telemetry                 |

---

## Wireless Communication (ESP-NOW)

- **Channel**: 9, long-range mode enabled
- **Encryption**: PMK + LMK configured per peer
- **Remote → Drone**: 20 Hz control packets (throttle, pitch, roll, yaw, flight mode)
- **Drone → Bridge**: 10 Hz telemetry packets (angles, rates, PID outputs, motor PWM, battery mV)
- **Bridge → Drone**: PID tuning packets forwarded from PC

MAC addresses are configured in `libs/espnow-comm/espnow_comm.c`.

---

## Bridge ↔ PC Link (UART)

The bridge and desktop app talk over a framed, CRC-checked link at 115200 baud: `[0xAA 0x55][payload][crc16]`. Framing (`uart_comm_write_frame()` / `uart_comm_read_frame()`) lives in `bridge/main/bridge.c`; the Python mirror is `config-app/uart_comm.py` — keep both in sync if the format changes. On a CRC failure the decoder discards just the 2 magic bytes and resyncs, rather than dropping a whole frame.

- **Bridge → PC**: `sensor_telemetry_t` packets forwarded from the drone's ESP-NOW telemetry
- **PC → Bridge → Drone**: `pid_config_telemetry_t` packets carrying live PID gain updates

---

## Repository Structure

```
quadrocopter/
├── drone/              # Flight controller firmware (ESP32)
│   └── main/            # drone.c, pid_task.c, input_task.c
├── remote/             # Remote control firmware (ESP32)
│   └── main/remote.c
├── bridge/             # Bridge firmware (ESP32-S3)
│   └── main/bridge.c
├── config-app/          # Desktop telemetry & PID tuning app (PyQt5)
│   ├── main.py             # Entry point
│   ├── pid_tuner_gui.py    # Main window - telemetry, PID/drone/remote config tabs
│   ├── uart_comm.py        # UART framing/CRC, mirrors bridge/main/bridge.c
│   ├── parser.py           # Parses structs/unions/enums out of a C header
│   ├── layout.py           # Simulates C struct layout to build struct-module formats
│   └── constants.py        # Wire-format constants, generated from libs/espnow-comm/espnow_comm.h
├── libs/                # ESP-IDF components shared across targets
│   ├── pid/              # PID controller
│   ├── motors/           # MCPWM ESC driver
│   ├── kalman/           # Single-axis Kalman filter for attitude (currently disabled)
│   ├── espnow-comm/      # ESP-NOW peer management, queues & wire structs
│   ├── device_config/     # NVS-persisted drone/remote config load & save
│   └── common_functions/ # mapf(), constrainf(), SPI/I2C bus init
├── components/          # Sensor driver components
│   ├── imu/               # BNO085 driver (quaternion → Euler)
│   ├── mcp3208/           # SPI ADC driver
│   └── esp32_BNO08x/      # Third-party BNO085 SH2-HAL library
└── 3d-models/           # STL files for printable frame parts
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

## ESC Programming Mode

A separate drone build that bypasses flight control entirely and drives all 4 ESCs directly from the remote's throttle channel — no PID, no mixing, no arming gate. This is needed because normal flight firmware always brings the ESCs up at minimum throttle on boot as a safety floor, which prevents entering most ESCs' factory programming/calibration menu (that requires the ESC to see *max* throttle as its first signal on power-up).

Selected at compile time via `ESC_PROGRAMMING_MODE`, which switches `main.c` to call `init_esc_programming()` instead of `init_drone()`. It uses its own build directory and sdkconfig (`drone/sdkconfig.esc-program`, seeded from the real one) so it can never interfere with the normal drone build.

**Build & flash:**
```bash
just build-esc-program
just flash-esc-program [port]
just monitor-esc-program [port]
```

**Procedure:** set the transmitter throttle to max *before* powering on the drone — the ESCs need to see max throttle as their very first signal. Once powered, follow your ESC's normal programming-menu sequence via the throttle stick as you normally would.

---

## Desktop App (config-app)

A PyQt5 application for real-time monitoring, PID tuning, and drone/remote configuration.

**Dependencies:**
```bash
pip install PyQt5 pyqtgraph PyOpenGL pyserial numpy
```

**Run:**
```bash
cd config-app && python3 main.py
```

**Features:**
- Live plots of angles, rates, PID outputs, and motor speeds (500-point rolling window)
- 3D attitude view — a rotating rigid-body model (front arms in red, rear in grey) driven live by pitch/roll/yaw telemetry, with a numeric angle readout
- Battery voltage display
- A single PID Tuning tab: Rate mode gains for Pitch/Roll and Yaw side by side, Angle mode gains for Pitch/Roll only (yaw has no angle loop) — changes are sent to the drone immediately over the bridge
- Drone Config and Remote Config tabs for live failsafe/throttle/battery/joystick-calibration updates, with buttons to persist them to flash
- Wire-format struct/union layouts (`constants.py`) are parsed straight out of `libs/espnow-comm/espnow_comm.h` (`parser.py`/`layout.py` simulate the C compiler's struct-packing rules) rather than hand-maintained, so they can't drift out of sync with the firmware

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
2. Run `python3 main.py` (from `config-app/`) and select the correct serial port
3. Adjust gains while the drone is flying; changes take effect within one telemetry cycle

---

## Implementation Notes

- **Dual-core**: Core 0 handles incoming remote packets; Core 1 runs the 400 Hz PID loop at elevated FreeRTOS priority
- **Thread safety**: Shared drone state (`droneData`) is guarded by a mutex, since `pid_task` and `input_task` write to it from separate cores
- **IMU filtering**: A single-axis Kalman filter (`libs/kalman`) exists to fuse gyro rate with the BNO085's fused angle, but is currently disabled (`#ifdef ENABLE`) — attitude passes through the sensor's fused rotation vector unfiltered
- **ADC calibration**: Battery voltage uses ESP-IDF ADC calibration for mV-level accuracy
- **Memory**: All dynamic allocations checked at startup; device restarts on malloc failure

---

## License

Apache 2.0
