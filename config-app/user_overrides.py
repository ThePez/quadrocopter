"""
Reads and writes config-app/user_defaults.json, the GUI's own store for
"save as default" values (PID gains, drone config, remote config).

This is deliberately separate from the firmware's *_defaults.h headers:
those stay the true, version-controlled defaults the firmware ships with,
while this file holds the current user's locally-saved working values and
is not meant to be committed.
"""

import json
import os
from typing import Dict

_OVERRIDES_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "user_defaults.json"
)


def _load_all() -> Dict[str, Dict[str, float]]:
    try:
        with open(_OVERRIDES_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Warning: could not read {_OVERRIDES_PATH}: {exc}")
        return {}


def get_section(section: str) -> Dict[str, float]:
    """Returns the saved overrides for one section (e.g. "pid", "drone",
    "remote"), or an empty dict if none have been saved yet."""
    return _load_all().get(section, {})


def save_section(section: str, updates: Dict[str, float]) -> bool:
    """Merges `updates` into the given section and writes the whole
    overrides file back out."""
    all_overrides = _load_all()
    all_overrides.setdefault(section, {}).update(updates)

    try:
        with open(_OVERRIDES_PATH, "w") as f:
            json.dump(all_overrides, f, indent=2, sort_keys=True)
            f.write("\n")
    except OSError as exc:
        print(f"Warning: could not write {_OVERRIDES_PATH}: {exc}")
        return False

    return True
