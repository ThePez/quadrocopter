"""
Wire-format constants shared across the GUI's modules.
Each one mirrors a struct in libs/espnow-comm/espnow_comm.h - keep both in
sync if the firmware side changes.
"""

import struct

# Mirrors struct sensor_telemetry_t in libs/espnow-comm/espnow_comm.h:
# 10 floats (angles/rates/mode/pid outputs) then 5 uint16_t (motor duty cycles, battery mV).
# The struct isn't packed, so the compiler pads its sizeof() up to a multiple of 4.
# The trailing '2x' skips those 2 pad bytes; bridge.c sends sizeof(struct
# sensor_telemetry_t) bytes per packet, so this must track that exactly.
SENSOR_TELEMETRY_FORMAT = "<10f5H2x"
SENSOR_TELEMETRY_SIZE = struct.calcsize(SENSOR_TELEMETRY_FORMAT)

# Mirrors struct pid_config_telemetry_t in libs/espnow-comm/espnow_comm.h:
# 3 floats (kp, ki, kd) then 2 uint16_t (axis, mode).
PID_CONFIG_FORMAT = "<fffHH"
