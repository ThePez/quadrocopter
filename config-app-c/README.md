# config-app-c

Desktop GUI for tuning and monitoring the quadcopter over the bridge's USB serial link, built with
raylib + raygui. It replaces the old PyQt `config-app/`, which is retired; the only thing still
shared with it is the saved-values file `config-app/user_defaults.json` (and the session CSV format).

**Working on the code? Read [DEVELOPING.md](DEVELOPING.md)**: how the immediate-mode UI works, the
frame loop, the ack/retry system, and step-by-step recipes for common changes.

## Build and run

Needs raylib and `raygui.h` installed (the Makefile uses `pkg-config` if it
knows raylib, otherwise the usual Linux link line).

```bash
make
```

```bash
make run
```

Linux only (serial I/O uses `termios`). You may need to be in the `dialout`
group to open `/dev/ttyUSB*` / `/dev/ttyACM*`.

## Editor support (clangd)

clangd reads `config-app-c/compile_commands.json` (git-ignored), generated from
the Makefile's real compile lines so it has the right include paths and defines:

```bash
make compile_commands
```

Re-run it after adding a source file or changing the Makefile, then restart the
language server in your editor. The repo-root `.clangd` only maps the firmware
folders, so it doesn't affect this one.

## Layout

| File | Role |
|---|---|
| `libs/espnow-comm/protocol.h` | Packet structs + enum shared with the firmware (replaces `parser.py`/`layout.py`/`constants.py`) |
| `src/uart.c` | Serial port, CRC16, frame decoder, config-packet encoder |
| `src/telemetry.c` | Reader thread + sample queue |
| `src/store.c` | `#define` scraper for firmware defaults, `user_defaults.json` read/write |
| `src/recorder.c` | Session CSV record / load |
| `src/ui.c` | Font, coloured buttons, numeric field, modal dialogs, file picker |
| `src/plot.c` | Live and zoomable line plots |
| `src/attitude3d.c` | 3D drone view |
| `src/tab_*.c` | One file per tab (telemetry, PID, drone, remote, console, session viewer) |
| `src/util.c` | Monotonic clock, safe string copy, clock string |
| `src/app.c` | Connection, logging, sending, saving defaults |
| `src/main.c` | Window, connection bar, tab bar |

## Pages

- **Sidebar:** connection (port, baud, connect) and navigation. A *Session* entry appears while a recording is loaded.
- **Telemetry:** all four live plots at once (or one large one), with the 3D view and live values in a right-hand column.
- **PID Tuning:** three gain cards with the live Angle and Rate plots underneath, so you can judge a change without switching pages.
- **Drone / Remote Config:** grouped cards with an action bar; the Drone page also shows live angle and motor plots.

## Serial frames and config acknowledgements

Every frame, in both directions, is `[0xAA 0x55][struct wifi_packet_t]`. The packet carries its own
CRC16 (over its data), and its `packet_id` says what it is. The bridge forwards packets exactly as
it received them.

- **Bridge to GUI:** `SENSOR` (telemetry in `data.sensor`) and the config acks `PID_ACK`,
  `DRONE_CFG_ACK` and `REMOTE_CFG_ACK` (`data.result.status` is 0 for success, otherwise the
  firmware's error code). Any other packet id is ignored.
- **GUI to bridge:** the config packets (`PID_CONFIG`, `DRONE_CFG`, `DRONE_CFG_STORE`, `REMOTE_CFG`,
  `REMOTE_CFG_STORE`).

Each config packet is expected to be answered by the device it was sent to:

- One request per device is in flight at a time; later ones for the same device wait behind it.
- No ack within 0.6 s (1.5 s for a save) re-sends the packet: 3 tries for an update, 2 for a save.
- The result shows at the bottom of the sidebar for a few seconds and is written to the Console.

## Differences from the Python app

- The session viewer is a mode inside the main window (Telemetry tab, "Open
  Session..."), not a separate window. You can also drop a `.csv` on the window.
- Text fields: click to type (click again, or use Left / Right / Home / End, to move the caret),
  Enter or click away to commit, Esc to cancel. Arrow buttons and the mouse wheel step the
  value. There is no clipboard.
- Plot hover shows the nearest sample's values. Static (session) plots: scroll
  to zoom, drag to pan, double-click to reset.
- Sessions are written to `config-app-c/sessions/`.
