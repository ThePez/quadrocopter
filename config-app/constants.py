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

# Mirrors enum wifi_packet_id in libs/espnow-comm/espnow_comm.h. CONFIG_SAVE
# and REMOTE_CONFIG_SAVE carry no payload - the packet_id alone tells the
# drone/remote to persist its currently-live config to flash.
(
    SENSOR,
    REMOTE,
    PID_CONFIG,
    POWER,
    DRONE_CONFIG,
    CONFIG_SAVE,
    REMOTE_CONFIG,
    REMOTE_CONFIG_SAVE,
) = range(8)

# Mirrors struct drone_config_telemetry_t: 5 floats (max_rate, max_angle,
# fail_angle, min_throttle, max_throttle), 1 uint32_t (coms_timeout_us), then
# 2 uint16_t (low_voltage, critical_voltage). Already 4-byte aligned - no pad.
DRONE_CONFIG_FORMAT = "<5fI2H"
DRONE_CONFIG_SIZE = struct.calcsize(DRONE_CONFIG_FORMAT)

# Mirrors struct joystick_cal_telemetry_t: 3 uint16_t (min, centre, max).
JOYSTICK_CAL_FORMAT = "<3H"

# Mirrors struct remote_config_telemetry_t: 1 float (voltage_cal_multiplier),
# 2 uint16_t (low_voltage, critical_voltage), then 4 joystick_cal_telemetry_t
# (throttle, pitch, roll, yaw), each 3 uint16_t.
REMOTE_CONFIG_FORMAT = "<f2H" + "3H" * 4
REMOTE_CONFIG_SIZE = struct.calcsize(REMOTE_CONFIG_FORMAT)

# Mirrors sizeof(union packet_data) on the firmware side - driven by
# sensor_telemetry_t, still the largest member (see SENSOR_TELEMETRY_SIZE
# above). Every GUI -> bridge config packet's payload is placed at the start
# of this many bytes (zero-padded) before the wire-level crc16/packet_id,
# mirroring struct wifi_packet_t's layout exactly. Update this if a union
# member ever grows past sensor_telemetry_t's size.
WIFI_PACKET_UNION_SIZE = SENSOR_TELEMETRY_SIZE

# Mirrors sizeof(struct wifi_packet_t): the union above, then a uint16_t
# crc16, a uint8_t packet_id, rounded up to the struct's 4-byte alignment
# (from the union's floats) - whatever trailing pad that rounding implies.
_WIFI_PACKET_RAW_SIZE = WIFI_PACKET_UNION_SIZE + 2 + 1  # data + crc16 + packet_id
WIFI_PACKET_SIZE = (_WIFI_PACKET_RAW_SIZE + 3) & ~3
