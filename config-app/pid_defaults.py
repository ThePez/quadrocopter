"""
Reads libs/pid/pid_defaults.h for the firmware's built-in PID gains, then
layers the GUI's own saved-as-default gains (from user_overrides.py) on top.
"""

import os
from typing import Dict, Tuple

from parser import parse_defines, read_file
from user_overrides import get_section, save_section

_HEADER_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "libs",
    "pid",
    "pid_defaults.h",
)

# Maps each (axis, mode) tab to the #define prefix used in pid_defaults.h
PID_DEFAULT_KEYS: Dict[Tuple[int, int], str] = {
    (0, 0): "RATE_PITCH_ROLL",
    (1, 0): "RATE_YAW",
    (0, 1): "ANGLE_PITCH_ROLL",
}


def load_pid_defaults() -> Dict[str, int | float]:
    """
    Parse libs/pid/pid_defaults.h for the firmware defaults, then apply
    any gains the GUI has since saved as its own working defaults.
    """
    defaults: Dict[str, int | float] = {}
    try:
        defaults = parse_defines(read_file(_HEADER_PATH))
    except OSError as exc:
        print(f"Warning: could not read PID defaults from {_HEADER_PATH}: {exc}")

    defaults.update(get_section("pid"))
    return defaults


# Populated at import time from pid_defaults.h + saved overrides
PID_DEFAULTS: Dict[str, int | float] = load_pid_defaults()


def save_pid_defaults(
    axis: int, mode: int, kp: float, ki: float, kd: float, d_alpha: float
) -> bool:
    """
    Save the given gains as the GUI's own working defaults for this
    axis/mode, so the next GUI launch starts from them
    """
    prefix = PID_DEFAULT_KEYS.get((axis, mode))
    if prefix is None:
        return False

    updates = {
        f"{prefix}_KP": kp,
        f"{prefix}_KI": ki,
        f"{prefix}_KD": kd,
        f"{prefix}_KD_ALPHA": d_alpha,
    }

    if not save_section("pid", updates):
        return False

    PID_DEFAULTS.update(updates)
    return True
