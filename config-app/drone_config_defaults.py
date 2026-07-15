"""
Reads drone/main/drone_defaults.h and remote/main/remote_defaults.h so the
GUI's default values for the drone/remote config tabs stay in sync with
firmware instead of being hand-copied.
"""

import os
import re
from typing import Dict

_DEFINE_RE = re.compile(r"^\s*#define\s+(\w+)\s+([0-9.eE+-]+)")

_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_DRONE_DEFAULTS_PATH = os.path.join(_APP_DIR, "..", "drone", "main", "drone_defaults.h")
_REMOTE_DEFAULTS_PATH = os.path.join(
    _APP_DIR, "..", "remote", "main", "remote_defaults.h"
)


def _parse_defines(path: str) -> Dict[str, float]:
    """Parse simple `#define NAME VALUE` lines, ignoring any trailing comment."""
    defines: Dict[str, float] = {}
    try:
        with open(path, "r") as header:
            for line in header:
                match = _DEFINE_RE.match(line)
                if match:
                    defines[match.group(1)] = float(match.group(2))
    except OSError as exc:
        print(f"Warning: could not read defaults from {path}: {exc}")
    return defines


# Populated at import time from drone_defaults.h / remote_defaults.h
DRONE_DEFAULTS: Dict[str, float] = _parse_defines(_DRONE_DEFAULTS_PATH)
REMOTE_DEFAULTS: Dict[str, float] = _parse_defines(_REMOTE_DEFAULTS_PATH)
