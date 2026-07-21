"""
Wire-format constants shared across the GUI's modules.
Formats/sizes are generated from libs/espnow-comm/espnow_comm.h via
parser.py/layout.py, so they can't drift out of sync with the firmware side.
"""

from pathlib import Path
from types import SimpleNamespace

from parser import parse_structs, parse_unions, parse_enum, parse_defines, read_file
from layout import build_formats

_ESPNOW_HEADER = Path(__file__).resolve().parent.parent / "libs" / "espnow-comm" / "espnow_comm.h"

_header_text = read_file(str(_ESPNOW_HEADER))
_structs, _is_packed = parse_structs(str(_ESPNOW_HEADER))
_unions = parse_unions(str(_ESPNOW_HEADER))
_defines = parse_defines(_header_text)

_struct_layouts, _union_layouts = build_formats(_structs, _is_packed, _unions, _defines)


def _format(struct_name: str) -> str:
    return "<" + _struct_layouts[struct_name][0]


def _size(struct_name: str) -> int:
    return _struct_layouts[struct_name][1]


# Mirrors struct sensor_telemetry_t in libs/espnow-comm/espnow_comm.h.
SENSOR_TELEMETRY_FORMAT = _format("sensor_telemetry_t")
SENSOR_TELEMETRY_SIZE = _size("sensor_telemetry_t")

# Mirrors struct pid_config_telemetry_t in libs/espnow-comm/espnow_comm.h.
PID_CONFIG_FORMAT = _format("pid_config_telemetry_t")

# Mirrors enum esp_now_packet_id in libs/espnow-comm/espnow_comm.h - values
# are parsed from the header itself so the two files can't drift out of sync.
# Access as e.g. PACKET_ID.DRONE_CFG.
PACKET_ID = SimpleNamespace(**parse_enum(_header_text, "esp_now_packet_id"))

# Mirrors struct drone_config_telemetry_t in libs/espnow-comm/espnow_comm.h.
DRONE_CONFIG_FORMAT = _format("drone_config_telemetry_t")
DRONE_CONFIG_SIZE = _size("drone_config_telemetry_t")

# Mirrors struct joystick_cal_telemetry_t in libs/espnow-comm/espnow_comm.h.
JOYSTICK_CAL_FORMAT = _format("joystick_cal_telemetry_t")

# Mirrors struct remote_config_telemetry_t in libs/espnow-comm/espnow_comm.h.
REMOTE_CONFIG_FORMAT = _format("remote_config_telemetry_t")
REMOTE_CONFIG_SIZE = _size("remote_config_telemetry_t")

# Mirrors sizeof(union packet_data) on the firmware side.
WIFI_PACKET_UNION_SIZE, _ = _union_layouts["packet_data"]

# Mirrors sizeof(struct wifi_packet_t): packet_id, crc16, then the union
# above, padded out to the struct's own alignment.
WIFI_PACKET_SIZE = _size("wifi_packet_t")
