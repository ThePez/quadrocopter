# Quadrocopter

Custom quadrocopter firmware written in C using the ESP-IDF framework. The system is split across three ESP32 microcontrollers — a drone flight controller, a handheld remote, and a PC bridge — coordinating over ESP-NOW. A C desktop app (raylib) provides real-time telemetry display, live PID tuning, drone/remote configuration and session recording.

---

## System Architecture

```
┌──────────────┐   control packets   ┌──────────────┐
│    Remote    │ ──────────────────► │    Drone     │
│   (ESP32)    │      (ESP-NOW)      │   (ESP32)    │
└──────┬───────┘                     └──────┬───────┘
       ▲                                    ▲
       │ ESP-NOW (two-way)                  │ ESP-NOW (two-way)
       ▼                                    ▼
       └───────────────►┌──────────────┐◄───┘
                        │    Bridge    │
                        │  (ESP32-S3)  │
                        └──────┬───────┘
                               ▲
                               │ UART / USB serial (two-way)
                               ▼
                        ┌──────────────┐
                        │      PC      │
                        │(config-app-c)│
                        └──────────────┘
```

| Node   | MCU       | Role                                                          |
|--------|-----------|---------------------------------------------------------------|
| Drone  | ESP32     | Flight controller — PID loops, motor mixing, sensor fusion    |
| Remote | ESP32     | Joystick transmitter — reads ADC axes, sends control packets to the drone |
| Bridge | ESP32-S3  | Ground station interface — talks to both the drone and the remote over ESP-NOW and relays telemetry/config to the PC over UART |

The remote sends control packets to the drone, which only listens to it. The bridge talks both ways with the drone and with the remote: it receives the drone's telemetry and sends config packets to either device, and each device answers a config packet with an ack.

---

## Hardware

### Drone (ESP32)
| Component         | Interface | Pins / Notes                           |
|-------------------|-----------|----------------------------------------|
| 4× ESC            | MCPWM     | GPIOs 17, 16, 27, 26 — 250 Hz RC PWM  |
| BNO085 IMU        | SPI       | 9-DOF attitude sensor                  |
| Battery monitor   | GPIO 35   | ESP32 ADC1 ch 7, divider (46.4 kΩ + 9.84 kΩ) |

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

Compile-time defaults (`libs/pid/pid_defaults.h`). The gains actually flown are whatever was last sent from the desktop app, or saved to the drone's flash, so they will usually differ from this table.

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
Front-Left  (A): throttle − pitch + roll + yaw
Back-Left   (B): throttle + pitch + roll − yaw
Back-Right  (C): throttle + pitch − roll + yaw
Front-Right (D): throttle − pitch − roll − yaw
```

A small static trim per axis (`PITCH_TRIM_US`, `ROLL_TRIM_US`, `YAW_TRIM_US` in `drone/main/pid_task.c`) is added to the PID outputs before mixing, to cancel a constant mechanical bias such as an off-centre battery. The trims are scaled with throttle, so they fade out near idle.

Each motor output is clamped to [1050, 2000] µs.

### Failsafes

| Condition                         | Response                                                |
|-----------------------------------|---------------------------------------------------------|
| Pitch or roll > ±30°              | Motors killed; must throttle down and level to re-arm  |
| No remote signal for > 2 s        | Motors killed immediately                               |
| Battery ≤ 14 V (warn)             | Low-battery alert sent to the remote (if it is connected) |
| Battery ≤ 13.6 V (critical)       | No take-off. In flight, throttle is limited to 50% of its range so the drone descends instead of cutting out |

---

## Wireless Communication (ESP-NOW)

- **Channel**: 9
- **Encryption**: PMK + LMK configured per peer
- **Remote → Drone**: 20 Hz control packets (throttle, pitch, roll, yaw, flight mode)
- **Drone → Bridge**: 50 Hz telemetry packets (angles, rates, PID outputs, motor PWM, battery mV, loop timing, pilot commands)
- **Bridge → Drone / Remote**: config packets (PID gains, drone config, remote config, "save to flash") forwarded from the PC
- **Drone / Remote → Bridge**: an ack for every config packet, forwarded to the PC

MAC addresses are configured in `libs/espnow-comm/espnow_comm.c`.

---

## Bridge ↔ PC Link (UART)

The bridge and desktop app talk over a framed, CRC-checked link at 115200 baud. Every frame, in both directions, is `[0xAA 0x55][struct wifi_packet_t]`; the packet carries its own CRC-16 and a `packet_id` saying what it is. The bridge forwards packets exactly as it received them. Framing on the firmware side (`uart_comm_write_frame()` / `uart_comm_read_frame()`) lives in `bridge/main/bridge.c`; the PC side is `config-app-c/src/uart.c`. Both use the packet structs in `libs/espnow-comm/protocol.h`, so a change to a struct means rebuilding and flashing every side that uses it. On a CRC failure the decoder discards just the 2 magic bytes and resyncs, rather than dropping a whole frame.

- **Bridge → PC**: `SENSOR` packets (telemetry) and the config acks `PID_ACK`, `DRONE_CFG_ACK`, `REMOTE_CFG_ACK`
- **PC → Bridge**: `PID_CONFIG`, `DRONE_CFG`, `DRONE_CFG_STORE`, `REMOTE_CFG`, `REMOTE_CFG_STORE`

The desktop app waits for the ack of every config packet, re-sends if none arrives, and shows the setting as unconfirmed until it does.

---

## Repository Structure

```
quadrocopter/
├── drone/              # Flight controller firmware (ESP32)
│   └── main/            # drone.c, pid_task.c, input_task.c
├── remote/             # Remote control firmware (ESP32)
│   └── main/            # remote.c, controller_task.c, rx_task.c
├── bridge/             # Bridge firmware (ESP32-S3)
│   └── main/bridge.c
├── config-app-c/        # Desktop telemetry & PID tuning app (C, raylib/raygui)
│   ├── src/              # app, ui, plot, serial/protocol and one tab_*.c per page
│   ├── README.md         # Build and usage
│   └── DEVELOPING.md     # How the code works and how to change it
├── libs/                # ESP-IDF components shared across targets
│   ├── pid/              # PID controller and default gains
│   ├── motors/           # MCPWM ESC driver
│   ├── espnow-comm/      # ESP-NOW peer management, queues & wire structs (protocol.h)
│   ├── device_config/     # NVS-persisted drone/remote config load & save
│   └── common_functions/ # mapf(), constrainf(), SPI/I2C bus init
├── components/          # Sensor driver components
│   ├── imu/               # BNO085 driver (quaternion → Euler)
│   ├── mcp3208/           # SPI ADC driver
│   ├── esp32_BNO08x/      # Third-party BNO085 SH2-HAL library
│   └── un-used/           # Retired drivers, kept for reference
├── scripts/             # check_device.sh: verifies the connected board before flashing
├── circuits/            # KiCad symbols and notes for the electronics
├── 3d-models/           # STL files for printable frame parts
└── Justfile             # Build / flash / monitor / run recipes
```

---

## Building & Flashing

Requires the [ESP-IDF toolchain](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) and [`just`](https://github.com/casey/just). Activate your ESP-IDF environment in the terminal first; the recipes stop with a message if `idf.py` is not on the path.

```bash
just build drone      # also: remote, bridge (sets the right chip target on a fresh build dir)
just flash drone      # optionally: just flash drone /dev/ttyUSB0
just monitor drone
just all drone        # build + flash
just clean drone
```

`just flash` checks that the connected board is the right chip and that its MAC address matches the one listed for that role in `libs/espnow-comm/espnow_comm.c`, so the wrong firmware cannot be flashed to the wrong board. Add `force` to override a MAC mismatch (`just flash drone /dev/ttyUSB0 force`).

Each target has its own `sdkconfig` and `CMakeLists.txt`; the bridge is built for the ESP32-S3.

**Changing the stored config:** the drone and remote keep their settings in flash as a versioned blob (`libs/device_config/device_config.h`). If that layout changes, bump its version; on the next boot the stored config resets to the compiled defaults, so re-send and save your values from the app.

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

## Desktop App (config-app-c)

A C application (raylib + raygui, Linux) for real-time monitoring, PID tuning, and drone/remote configuration.

**Build and run** (needs raylib and `raygui.h` installed; see [config-app-c/README.md](config-app-c/README.md)):
```bash
just gui-c          # build and run
just gui-c-build    # build only
```

**Features:**
- Live plots of angles, rates, PID outputs, motor speeds and the pilot's control inputs (1000-point rolling window), with a live-values card
- 3D attitude view — a rotating rigid-body model driven live by pitch/roll/yaw telemetry
- Battery voltage and worst-case loop timing display
- Session recording — **Record** on the Telemetry tab saves every packet to `config-app-c/sessions/session_<date>_<time>.csv`; **Open session** (or dropping a `.csv` on the window) reopens one as zoomable plots, with each PID gain change marked on the time axis
- PID Tuning tab: Rate mode gains for Pitch/Roll and Yaw, Angle mode gains for Pitch/Roll (yaw has no angle loop), each with a D-term low-pass coefficient (shown with the equivalent cutoff in Hz). Gains can be sent live, saved to the drone's flash, or edited offline and saved to a file (**Save all to file**) and pushed later (**Send all to drone**)
- Drone Config and Remote Config tabs for live flight limits, throttle range, gyro filter, failsafe, battery and joystick-calibration updates, with buttons to persist them to flash
- Every config send is confirmed by the device's ack and retried if it is missing
- Defaults for every field are read straight from the firmware's `*_defaults.h` headers, and the values you save are kept in `user_defaults.json`, so the app starts from the same numbers the firmware was built with
- Wire formats come from the same `libs/espnow-comm/protocol.h` the firmware compiles, so the app and firmware cannot disagree about a struct layout

To work on the app's code, start with [config-app-c/DEVELOPING.md](config-app-c/DEVELOPING.md).

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
1. Connect the bridge to your PC via USB (you may need to be in the `dialout` group)
2. Run `just gui-c`, choose the serial port and click **Connect**
3. Adjust gains on the PID Tuning tab and click **Send to drone**; the new gains take effect as soon as the drone acknowledges them
4. When you are happy, **Save to flash & default** keeps them on the drone across power cycles

---

## Implementation Notes

- **Dual-core**: Core 0 handles incoming remote packets; Core 1 runs the PID loop at elevated FreeRTOS priority, one cycle per IMU sample (400 Hz)
- **Thread safety**: Shared drone state (`droneData`) is guarded by a mutex, since `pid_task` and `input_task` write to it from separate cores
- **IMU filtering**: Angles are the BNO085's fused rotation vector, used as-is; gyro rates get a first-order low-pass whose coefficient is a live drone config setting (`gyro_alpha`, default `GYRO_FILTER_ALPHA` in `drone_defaults.h`, set from the Drone Config tab). `imuQueue` is a one-deep mailbox, so the loop always flies on the newest sample
- **ADC calibration**: Battery voltage uses ESP-IDF ADC calibration for mV-level accuracy
- **Memory**: All dynamic allocations checked at startup; device restarts on malloc failure

---

## License

Apache 2.0
