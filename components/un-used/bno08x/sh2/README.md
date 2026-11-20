# SH2 library (manufacturer code)

These files are the sensor manufacturer's SH-2 host library (Hillcrest
Laboratories / CEVA), licensed under Apache 2.0 - see the header of each file.
Upstream: https://github.com/ceva-dsp/sh2

They were copied as-is from `components/esp32_BNO08x/SH2`, the copy bundled
with the third-party C++ driver. Treat them as read-only: all board-specific
code belongs in `../bno08x.c`, which provides the `sh2_Hal_t` functions this
library calls.

| File | Purpose |
|------|---------|
| `sh2_hal.h` | The interface `bno08x.c` has to implement (open/close/read/write/getTimeUs) |
| `sh2.h`, `sh2.c` | Main API: `sh2_open()`, `sh2_service()`, `sh2_setSensorConfig()`, ... |
| `sh2_SensorValue.h/.c` | Decodes a raw sensor event into a typed `sh2_SensorValue_t` |
| `shtp.h/.c` | The SHTP packet transport underneath SH-2 |
| `sh2_util.h/.c`, `sh2_err.h` | Byte helpers and error codes |
