# Developing the config app

A guide for changing `config-app-c/` yourself. Read it top to bottom once; after that use the
[recipes](#recipes) and the [file map](#file-map) as a reference.

For build/run instructions, see [README.md](README.md).

---

## 1. The big picture

The app is a **desktop GUI that talks to the drone through the bridge ESP32 over USB serial**.

```
 drone ESP32  <--ESP-NOW-->  bridge ESP32-S3  <--USB serial-->  this app
 remote ESP32 <--ESP-NOW-->        (forwards packets unchanged)
```

- **Downstream (bridge to app):** telemetry samples (about 50 Hz) and *acks* (the drone or remote
  saying "I applied your config").
- **Upstream (app to bridge):** config packets (PID gains, drone config, remote config, "save to
  flash").

Every frame in both directions is `[0xAA 0x55][struct wifi_packet_t]`. The struct comes from
`libs/espnow-comm/protocol.h`, the same header the firmware compiles, so the app never hand-parses
bytes: it just casts to the shared struct. If the firmware changes a struct, rebuild the app.

The window is drawn with **raylib** (graphics, input) and a little **raygui** (only the two
dropdowns). Almost every widget is custom code in `ui.c`.

---

## 2. The most important idea: immediate-mode UI

If you have used Qt or web frameworks, forget widget objects. This app has **no widget tree and no
event handlers**. Instead, 60 times a second `main()` does:

```c
while (!WindowShouldClose()) {
    app_update(app);        // 1. process data: serial bytes, acks, timeouts
    BeginDrawing();
    ClearBackground(...);
    draw_frame(app);        // 2. draw everything AND handle clicks, in the same calls
    ui_end_frame();
    EndDrawing();
}
```

A button is just a function call that **draws itself and returns `true` on the frame it was
clicked**:

```c
if (ui_button(rect, "Send to drone", UI_ACCENT)) {
    send_pid(g, false);          // runs once, on the frame of the click
}
```

Consequences you need to keep in mind:

1. **Layout is recomputed every frame.** Positions are plain `Rectangle` maths (`ui_col`,
   `ui_row`, `ui_card_body`). There is no layout engine.
2. **State lives in structs you own**, not in widgets. A text field's value, a tab's selection and
   "is a request pending" are all fields in `App` / `PidTab` / ... (see `app.h`, `tabs.h`).
3. **Draw order is z-order.** Whatever is drawn last is on top. That is why `draw_dropdowns()` and
   `ui_modal_draw()` run after the page: their open lists / dialogs must cover it.
4. **Input blocking is done with `GuiLock()`.** While a modal or an open dropdown list is showing,
   `draw_frame()` locks raygui; every `ui_*` control checks `GuiIsLocked()` (via `is_hover`) so the
   page underneath ignores the mouse.
5. **Never block.** Anything slow (waiting for an ack) must be a state machine advanced a little
   each frame, not a loop that waits.

### How a click is detected (`ui.c`, `clicked()`)

A click needs the press *and* the release inside the same widget. On press, the widget's rectangle
is remembered in `g_armed`; on release over the same rectangle it counts. `ui_end_frame()` clears
the arm when the button is up. That is why dragging off a button cancels it.

> **Gotcha:** widgets are identified by their rectangle. If two widgets share an identical
> rectangle in one frame, the click can arm both.

---

## 3. Frame flow, in detail

```
main()
 ├─ app_update(app)                      app.c
 │    ├─ telemetry_poll()   drain samples queued by the reader thread
 │    │    └─ handle_sample()  -> recorder_add(), telemetry_tab_push() (plots + 3D view)
 │    ├─ telemetry_poll_acks() -> handle_ack()   match an ack to the request in flight
 │    ├─ ack_tick()          timeouts and retries
 │    ├─ telemetry_lost()    serial unplugged -> disconnect + error dialog
 │    └─ open_dropped_session()  a .csv dropped on the window
 └─ draw_frame(app)                      main.c
      ├─ draw_sidebar()      connection card, nav, ack status, recording badge
      ├─ switch (main_tab)   one tab_*_draw() fills the content area
      ├─ draw_dropdowns()    port / baud lists (on top of the page)
      └─ ui_modal_draw()     message box or file picker (on top of everything)
```

### Threads

There are exactly two:

- **Main thread:** everything above.
- **Reader thread** (`telemetry.c`, `reader_main`): blocks reading the serial port, decodes frames
  and pushes `Sample`s / `AckEvent`s into two ring buffers guarded by one mutex.

The main thread drains those buffers each frame (`telemetry_poll`, `telemetry_poll_acks`). The
mutex is the only shared state; nothing else in the app is thread-safe, so **never call raylib or
touch `App` from the reader thread**. If the UI stalls the buffers drop the oldest entries rather
than grow.

---

## 4. Data flow: telemetry

1. `uart.c` `decoder_feed()` scans for `AA 55`, checks the CRC-16, and calls back per valid frame.
   A bad CRC discards only the 2 magic bytes, so it resyncs on the next real frame.
2. `telemetry.c` converts the wire struct into a `Sample` (`sample_from_wire`): a timestamp plus a
   `float v[F_COUNT]` array indexed by `enum Field`.
3. On the main thread `handle_sample()` (app.c):
   - records it if recording (every sample, even when the live plots are off),
   - if "Live plots" is on, stores it as `app->last` and calls `telemetry_tab_push()`, which feeds
     five live `Plot`s and the 3D view.

`enum Field` order **is the CSV column order** (`FIELD_NAMES`). Reordering it changes the file
format and breaks old recordings.

---

## 5. Data flow: sending config and the ack system

This is the most intricate part of the app (`app.c`, "Config acks"). The goal: when you press
*Send*, show **unconfirmed** until the drone says it applied the values, retry if it doesn't, and
only then treat the values as "what the drone is running".

### Pieces

| Piece | Where | Role |
|---|---|---|
| `app_send_then(app, id, payload, len, done, user, user_len)` | app.c | The one function tabs call to send. |
| `AckChannel` (x3: PID, drone, remote) | app.h | One request in flight per device + a queue of 8. |
| `AckRequest` | app.h | Encoded frame, callback, copy of caller context, timeout, attempts. |
| `ack_rule()` | app.c | Which packet ids are tracked, on which channel, timeout and retries. |
| `AckDone` callback | tab code | Runs **once** with `ok` true/false, from `app_update()`. |

### Lifecycle of a request

```
tab calls app_send_then(...)
   │ not tracked id? -> write now, call done(ok=true) immediately
   │ channel busy?   -> queue it (max 8)
   ▼
ack_start(): write frame to serial, remember sent_at          (in_flight = true)
   ▼
each frame ack_tick():  no ack within timeout?  re-send (up to max_attempts)
                        out of attempts?        done(ok=false), "no confirmation"
bridge relays an ack -> handle_ack(): status 0 -> done(ok=true), else "rejected"
   ▼
done() runs -> ack_pump() starts the next queued request
```

Rules worth knowing:

- Updates (`PID_CONFIG`, `DRONE_CFG`, `REMOTE_CFG`): wait **0.6 s** per try, **3 tries**.
- Saves (`*_STORE`, which write flash): wait **1.5 s**, **2 tries**.
- After a retry, a `quiet_until` window (0.4 s) swallows a late duplicate ack so it cannot be
  mistaken for the *next* request's answer.
- A disconnect calls `ack_fail_all()`: every waiting request gets `done(ok=false)`.
- `done` is **never** called from inside `app_send_then()` (except for untracked packets), so it is
  safe for a callback to send a follow-up request.
- The caller's context is **copied** (`user`, at most `ACK_USER_MAX` bytes). The UI fields may have
  changed by the time the ack arrives, so the callback works on the snapshot of what was sent.

### Example: "Save to flash" on a PID group (`tab_pid.c`)

1. `do_save()` checks `values_match_sent()`. If the on-screen gains were not confirmed yet, it sends
   `PID_CONFIG` with `then_save = true`.
2. `on_pid_done()` (confirmed) records the gains as "current" and, because `then_save`, calls
   `send_store()`.
3. `send_store()` sends `DRONE_CFG_STORE`; `on_store_done()` (confirmed) writes the gains into
   `user_defaults.json` via `app_save_defaults()`.

So a value is only ever remembered as a default once the drone has confirmed it is in flash.

---

## 6. Defaults and persistence (`store.c`)

Where a field's initial value comes from, in order:

1. **Firmware headers.** At startup `defines_load()` scrapes every `#define NAME <number>` from
   `libs/pid/pid_defaults.h`, `drone/main/drone_defaults.h` and `remote/main/remote_defaults.h`
   into a `KVSet` (a small name -> double table). Numeric literals and references to earlier
   numeric macros work; anything else is skipped.
2. **Saved overrides.** `config-app-c/user_defaults.json` (`{ "section": { "KEY": number } }`, git-ignored),
   merged on top with `kv_merge`. It is created the first time you save something.

This means: **adding a `#define` to a firmware defaults header makes it available to the GUI with
no parsing code**. Tabs read it with `kv_get(&app->drone_defaults, "NAME", fallback)`.

`app_save_defaults()` always re-reads the file before writing, so edits made elsewhere are not
lost, then merges the new values into the live `KVSet` too.

The *offline workflow* on the PID tab ("Save all to file" / "Send all to drone") relies on this:
the file is the source of truth while the drone is off.

---

## 7. The UI toolkit (`ui.h` / `ui.c`)

Read `ui.h` first; every control is declared there with a one-line summary.

- **Palette and metrics** are macros: `UI_BG`, `UI_CARD`, `UI_ACCENT`, ..., `UI_PAD`,
  `UI_ROW_H`, and so on. Use them instead of literal colours/sizes. The palette block is wrapped in
  `/* clang-format off */` so it stays aligned.
- **Text:** `ui_text`, `ui_text_bold` (draws twice, 0.6 px apart), `ui_text_right`,
  `ui_text_centered`, `ui_text_width`. One font, rasterised at `UI_TEXT_SIZE` (16 px) with
  `UI_TEXT_SPACING` 0 so 16 px text lands on whole pixels. Other sizes are scaled and slightly
  softer. Do not add fractional letter spacing: identical glyphs will render differently.
- **Surfaces:** `ui_card(r, title)` + `ui_card_body(r, titled)` give you the padded interior.
- **Layout helpers:** `ui_col(area, n, i, gap)` and `ui_row(...)` return the i-th of n equal parts.
- **Controls:** `ui_btn` (neutral), `ui_button` (filled colour), `ui_link`, `ui_nav_item`,
  `ui_segmented`, `ui_checkbox`, `numfield_draw` / `ui_field_row`.
- **Modals:** `ui_modal_open(title, msg, "Yes;No", callback, ctx)` and `ui_file_picker_open(...)`.
  Only one can be open (it is a single static `M`). The first button is the primary one and Enter
  triggers it; Esc triggers the last. The modal is closed *before* the callback runs, so a callback
  may open another.

### NumField (the number entry box)

State is in the `NumField` struct (`value`, `step`, `min`, `max`, `decimals`, plus `editing`,
`buf`, `cursor` while typing). Behaviour, all inside `numfield_draw()`:

- Click the text area: `field_begin()` loads `value` into `buf` and starts editing. Clicking again
  moves the caret (`caret_from_x()` finds the nearest gap between characters).
- Typed characters are filtered to `0-9 . - + e E` and inserted at the caret. Backspace/Delete/
  Left/Right/Home/End edit and move it.
- Enter or clicking away calls `field_commit()` (parse with `strtod`, clamp to min/max; an invalid
  string keeps the old value). Esc cancels.
- The chevrons and mouse wheel add/subtract `step`.
- A global `g_editing` guarantees only one field is mid-edit. `ui_commit_edit()` is called when
  switching tabs or opening a modal so a half-typed value is not lost.

---

## 8. Plots (`plot.c`)

One `Plot` struct serves two jobs:

- **Live plot** (`plot_init_live`): owns a fixed buffer (1000 points). `plot_push()` appends and
  slides the window once full. The x axis is `now - t0`, always showing everything in the buffer.
- **Static plot** (`plot_init_static` + `plot_bind`): *aliases* arrays from a loaded `Session`
  and shares one `PlotView` (visible time range). Scroll zooms, drag pans, double-click resets, and
  because all plots of a session share the view they move together.

`plot_draw()` does it all each frame: computes the x range, the visible index range (binary search,
`lower_bound`), the y range (`plot_pick_y_range` snaps to round numbers so the axis does not
rescale every sample), grid and ticks (`nice_step`), the series (decimated when there are more
points than twice the pixel width) and the hover read-out. A `NaN` value is a gap in the line.

Drawing is clipped with `BeginScissorMode` so lines never leave the plot area.

---

## 9. Tabs: the pattern to copy

Each tab is a `tab_*.c` file with the same shape. `tab_drone.c` is the cleanest example:

| Step | Function (drone tab) | What it does |
|---|---|---|
| State | `DroneTab` in `tabs.h` | `NumField f[]`, `sent[]` (last confirmed values), `sent_valid`. |
| Field table | `FIELDS[]` | label, `#define` name, scale, step, min, max, decimals. |
| Init | `drone_tab_init()` | create each `NumField` from `app->drone_defaults`. |
| Send | `send_config()` | build the wire struct from the fields, `app_send_then(..., on_config_done, &snap)`. |
| Confirm | `on_config_done()` | on `ok`, record `sent[]`; if `then_save`, chain `send_store()`. |
| Save | `do_save()`, `on_store_done()` | persist in flash, then save defaults to the JSON. |
| Draw | `drone_tab_draw()` | cards + buttons + live plots underneath. |

Tabs are registered in three places: `enum MainTab` (app.h), `NAV_LABELS` and the `switch` in
`draw_frame()` (main.c), plus a state struct and `*_init` in `app_init()`.

---

## 10. Recording and sessions (`recorder.c`, `tab_session.c`)

CSV format:

```
# started,2026-11-22T11:19:34
time_s,pitch_angle,roll_angle,...            <- one column per enum Field
0.0132,1.2,...
# event,12.431,Sent Pitch / Roll Rate: Kp=0.2000 ...   <- marks (PID sends) between rows
```

- Writing: `app_toggle_recording()` -> `recorder_start()` -> `recorder_add()` per sample, flushed every
  50 rows. `app_record_event()` writes `# event` lines (PID sends are recorded this way so you can
  see *when* a gain changed on the plots).
- Loading: `session_load()` reads columns into arrays; `session_tab_open()` builds eight static
  `Plot`s that point into those arrays. Files are saved in `config-app-c/sessions/`.

---

## 11. Conventions

- **Style** (enforced by `/home/jack/Documents/Projects/.clang-format`): 4 spaces, Linux braces,
  100 columns, braces on every `if/for/while`, and **a blank line after every closing `}` of an
  `if/for/while` block** (clang-format cannot do that last one, so do it by hand).
- **`sizeof(x)` always with parentheses.**
- Prefer `static` for anything not declared in a header. Public names are prefixed by module
  (`app_`, `ui_`, `plot_`, `numfield_`).
- Strings: use `str_copy()` (always NUL-terminates) and `snprintf` with `sizeof(buf)`.
- Warnings are on (`-Wall -Wextra`); keep the build warning-free.
- Do not allocate per frame. Big arrays live in `App` (which is `calloc`ed once in `main`).

---

## 12. Recipes

### Add a field to the Drone Config tab

1. Add the matching field to `struct drone_config_telemetry_t` in `libs/espnow-comm/protocol.h`
   and handle it on the drone (`drone/main/drone.c`). Mind the union size: the static asserts in
   `uart.c` will fail if `sensor_telemetry_t` / `wifi_packet_t` change size.
2. Add a `#define` with the default to `drone/main/drone_defaults.h`.
3. `tab_drone.c`: add an enum entry (e.g. `D_FOO`), a row in `FIELDS[]`, bump `DRONE_FIELDS` in
   `tabs.h`, add it to one of the card index arrays in `drone_tab_draw()` and set
   `.foo = (float)v[D_FOO]` in `send_config()`.
4. Rebuild and flash the drone. If you changed the stored config layout bump
   `NVS_DRONE_CFG_VERSION` (`libs/device_config/device_config.h`); note that this resets the
   drone's stored config to compiled defaults, so re-send and save afterwards.

### Add a new telemetry value

1. Add it to `struct sensor_telemetry_t` (protocol.h) and keep the size assert in `uart.c` in sync.
2. Add an `F_...` entry **at the end** of `enum Field` (telemetry.h), a name in `FIELD_NAMES`, and
   map it in `sample_from_wire()` (telemetry.c). Old sessions simply lack the column.
3. To show it: add a `ValueRow` in `tab_telemetry.c` (live values card) and/or a series in a plot.

### Add a plot series

Live: add a name and colour to the arrays in `telemetry_tab_init()`, raise `nseries`, and extend
the array in `telemetry_tab_push()`. Maximum is `PLOT_MAX_SERIES` (8).

### Add a new tab

Add `TAB_X` to `enum MainTab` (before `TAB_SESSION`), a label in `NAV_LABELS` (main.c), a `case` in
`draw_frame()`, `x_tab_init()` / `x_tab_draw()` in a new `tab_x.c` (the Makefile picks up new
`src/*.c` automatically), and the state struct in `tabs.h` + a member in `App`. Run
`make compile_commands` so clangd sees the new file.

### Add a new tracked config packet

Add its id to `ack_rule()` (channel, label, timeout, tries). If it needs a new ack type, also add a
channel (`ACK_CHANNELS`, `CHANNEL_ACK_ID`, `CH_*`) and teach `is_ack_id()` in `uart.c` about it.

### Add a button / control

Use an existing one from `ui.h`; compute its `Rectangle` from the parent card body and call it
inside the tab's `*_draw()` function with `if (ui_button(...)) { ... }`.

---

## 13. Debugging tips

- The **Console** tab shows `app_log()` output (connection events, every send, every ack result,
  retries). Add your own `app_log(app, "...")` lines freely; it is the easiest print debugging.
- Build with `-g` (the default); run under `gdb build/pid_tuner`.
- A control that "does nothing" is usually: the modal/dropdown lock is on (`GuiIsLocked()`), the
  rectangle is wrong, or something drawn later is covering it.
- Text that flickers or jumps is almost always a fractional position; round it.
- If sends stay "unconfirmed" the app side is fine and the cause is on the radio link or the
  firmware (the bridge must relay the ack; the target must reply with the right `*_ACK` id).
- Un-flashed firmware is the classic reason a new setting seems ignored. The app and the
  firmware share `protocol.h`, so rebuild **and** flash whichever side changed.

---

## 14. File map

| File | Responsibility |
|---|---|
| `main.c` | Window loop, sidebar (connection, nav, ack status), page dispatch, dropdowns. |
| `app.h` / `app.c` | `App` state, connect/disconnect, per-frame update, **ack tracking**, sending, saving defaults, recording control, log/error helpers. |
| `ui.h` / `ui.c` | Theme, text, cards, buttons, segmented control, checkbox, `NumField`, modals, file picker. |
| `uart.h` / `uart.c` | Serial port open/list/write, CRC-16, config-packet encoder, frame decoder. |
| `telemetry.h` / `telemetry.c` | `Sample`/`Field`, reader thread, sample and ack ring buffers. |
| `store.h` / `store.c` | `KVSet`, `#define` scraper, `user_defaults.json` read/write. |
| `recorder.h` / `recorder.c` | Session CSV writer and loader. |
| `plot.h` / `plot.c` | Live and static line plots. |
| `attitude3d.h` / `attitude3d.c` | Offscreen-rendered 3D drone model. |
| `tabs.h` | State structs and entry points for every tab. |
| `tab_telemetry.c` | Live plots, live values card, record / open session controls. |
| `tab_pid.c` | Three PID groups, send / save, offline save-all / send-all. |
| `tab_drone.c` | Flight limits, throttle range, gyro filter, failsafe thresholds. |
| `tab_remote.c` | Voltage and joystick calibration. |
| `tab_console.c` | Scrolling log (ring buffer). |
| `tab_session.c` | Viewer for a loaded recording. |
| `util.h` / `util.c` | `now_s()` clock, `str_copy`, `format_clock`. |
