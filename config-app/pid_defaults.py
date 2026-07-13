"""
Reads and writes libs/pid/pid_defaults.h, so the GUI's default gains stay in
sync with the firmware instead of being hand-copied into two places.
"""

import os
import re
from typing import Dict, Tuple

_DEFINE_RE = re.compile(r"^\s*#define\s+(\w+)\s+([0-9.eE+-]+)\s*$")

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


def load_pid_defaults() -> Dict[str, float]:
    """Parse libs/pid/pid_defaults.h so the GUI's default gains stay in sync
    with the firmware instead of being hand-copied into two places."""
    defaults: Dict[str, float] = {}
    try:
        with open(_HEADER_PATH, "r") as header:
            for line in header:
                match = _DEFINE_RE.match(line)
                if match:
                    defaults[match.group(1)] = float(match.group(2))
    except OSError as exc:
        print(f"Warning: could not read PID defaults from {_HEADER_PATH}: {exc}")
    return defaults


# Populated at import time from pid_defaults.h
PID_DEFAULTS: Dict[str, float] = load_pid_defaults()


def format_gain(value: float) -> str:
    """Render a gain the way pid_defaults.h already writes them (e.g. 1.0, 0.175)."""
    text = f"{value:.4f}".rstrip("0")
    return text + "0" if text.endswith(".") else text


def save_pid_defaults(axis: int, mode: int, kp: float, ki: float, kd: float) -> bool:
    """Write the given gains back into pid_defaults.h as the new defaults for
    this axis/mode, so the next GUI launch and the next firmware build both
    start from values already validated on the drone."""
    prefix = PID_DEFAULT_KEYS.get((axis, mode))
    if prefix is None:
        return False

    updates = {f"{prefix}_KP": kp, f"{prefix}_KI": ki, f"{prefix}_KD": kd}

    try:
        with open(_HEADER_PATH, "r") as header:
            lines: list[str] = header.readlines()

        for i, line in enumerate(lines):
            match = _DEFINE_RE.match(line)
            if match and match.group(1) in updates:
                lines[i] = (
                    f"#define {match.group(1)} {format_gain(updates[match.group(1)])}\n"
                )

        with open(_HEADER_PATH, "w") as header:
            header.writelines(lines)

    except OSError as exc:
        print(f"Warning: could not write PID defaults to {_HEADER_PATH}: {exc}")
        return False

    PID_DEFAULTS.update(updates)
    return True
