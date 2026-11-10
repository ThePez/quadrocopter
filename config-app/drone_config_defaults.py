"""
Reads drone/main/drone_defaults.h and remote/main/remote_defaults.h for the
firmware's built-in config values, then layers the GUI's own saved-as-default
values (from user_overrides.py) on top.
"""

import os
from typing import Dict

from parser import parse_defines, read_file
from user_overrides import get_section, save_section

_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_DRONE_DEFAULTS_PATH = os.path.join(_APP_DIR, "..", "drone", "main", "drone_defaults.h")
_REMOTE_DEFAULTS_PATH = os.path.join(
    _APP_DIR, "..", "remote", "main", "remote_defaults.h"
)


def _load_defaults(path: str, section: str) -> Dict[str, int | float]:
    """
    Parse a *_defaults.h header for the firmware defaults, then apply any
    values the GUI has since saved as its own working defaults.
    """
    defaults: Dict[str, int | float] = {}
    try:
        defaults = parse_defines(read_file(path))
    except OSError as exc:
        print(f"Warning: could not read defaults from {path}: {exc}")

    defaults.update(get_section(section))
    return defaults


def save_drone_defaults(updates: Dict[str, float]) -> bool:
    """
    Save the given values as the GUI's own working defaults for the
    drone config tab, without touching drone_defaults.h.
    """
    if not save_section("drone", updates):
        return False

    DRONE_DEFAULTS.update(updates)
    return True


def save_remote_defaults(updates: Dict[str, float]) -> bool:
    """
    Save the given values as the GUI's own working defaults for the
    remote config tab, without touching remote_defaults.h.
    """
    if not save_section("remote", updates):
        return False
    
    REMOTE_DEFAULTS.update(updates)
    return True


# Populated at import time from drone_defaults.h/remote_defaults.h + saved overrides
DRONE_DEFAULTS: Dict[str, int | float] = _load_defaults(_DRONE_DEFAULTS_PATH, "drone")
REMOTE_DEFAULTS: Dict[str, int | float] = _load_defaults(_REMOTE_DEFAULTS_PATH, "remote")
