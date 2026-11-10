"""
Records telemetry sessions to CSV files and reads them back.

A session file is plain CSV: a header row, then one row per telemetry packet
with `time_s` (seconds since recording started) followed by the telemetry
fields. Lines starting with `#` carry extra information and are skipped by
ordinary CSV readers that are told to treat `#` as a comment:

    # started,2026-11-10T14:03:22
    # event,12.431,Sent Pitch / Roll Rate: Kp=0.2000, Ki=0.0000, Kd=0.0000

This module has no Qt dependency, so sessions can also be loaded from a
script or notebook with load_session().
"""

import csv
import os
from datetime import datetime
from typing import IO, Any, Dict, List, Optional, Tuple

SESSIONS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sessions")

# Telemetry keys written to each row, in column order (see telemetry_thread.py)
FIELDS: List[str] = [
    "pitch_angle",
    "roll_angle",
    "yaw_angle",
    "pitch_rate",
    "roll_rate",
    "yaw_rate",
    "mode",
    "pitch_pid",
    "roll_pid",
    "yaw_pid",
    "motor_fl",
    "motor_bl",
    "motor_br",
    "motor_fr",
    "battery",
    "loop_dt_max_us",
]

# Rows between flushes to disk, so a crash loses at most about a second
_FLUSH_EVERY = 50


class SessionRecorder:
    """Writes telemetry packets to a new CSV file while recording is active."""

    def __init__(self) -> None:
        self._file: Optional[IO[str]] = None
        self._writer: Any = None
        self._start: Optional[datetime] = None
        self._unflushed: int = 0
        self.path: Optional[str] = None
        self.rows: int = 0

    @property
    def recording(self) -> bool:
        return self._file is not None

    def start(self) -> str:
        """Opens a new timestamped session file and returns its path."""
        self.stop()
        os.makedirs(SESSIONS_DIR, exist_ok=True)

        now = datetime.now()
        self.path = os.path.join(
            SESSIONS_DIR, f"session_{now.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self._file = open(self.path, "w", newline="")
        self._file.write(f"# started,{now.isoformat(timespec='seconds')}\n")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["time_s"] + FIELDS)
        self._start = now
        self._unflushed = 0
        self.rows = 0
        return self.path

    def _elapsed(self, when: Optional[datetime] = None) -> float:
        return ((when or datetime.now()) - self._start).total_seconds()

    def add(self, telemetry: Dict[str, Any]) -> None:
        """Appends one telemetry packet. No-op when not recording."""
        if not self.recording:
            return

        elapsed = self._elapsed(telemetry.get("timestamp"))
        self._writer.writerow(
            [f"{elapsed:.4f}"] + [telemetry.get(key, "") for key in FIELDS]
        )
        self.rows += 1

        self._unflushed += 1
        if self._unflushed >= _FLUSH_EVERY:
            self._file.flush()
            self._unflushed = 0

    def add_event(self, text: str) -> None:
        """Notes something that happened mid-session (e.g. a gain change)."""
        if not self.recording:
            return

        clean = text.replace("\n", " ")
        self._file.write(f"# event,{self._elapsed():.4f},{clean}\n")
        self._file.flush()

    def stop(self) -> Optional[str]:
        """Closes the current session file and returns its path, if any."""
        if not self.recording:
            return None

        self._file.close()
        self._file = None
        self._writer = None
        return self.path


def load_session(
    path: str,
) -> Tuple[Dict[str, List[float]], List[Tuple[float, str]]]:
    """
    Reads a session file back.

    Returns (columns, events): columns maps each CSV column name (including
    "time_s") to its list of values, and events is a list of (time_s, text).
    Rows that are incomplete or unreadable (e.g. a final row cut short by a
    crash) are skipped.
    """
    columns: Dict[str, List[float]] = {}
    events: List[Tuple[float, str]] = []
    header: Optional[List[str]] = None

    with open(path, "r", newline="") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("#"):
                parts = line[1:].strip().split(",", 2)
                if len(parts) == 3 and parts[0] == "event":
                    try:
                        events.append((float(parts[1]), parts[2]))
                    except ValueError:
                        pass
                continue

            cells = next(csv.reader([line]))
            if header is None:
                header = cells
                columns = {name: [] for name in header}
                continue

            if len(cells) != len(header):
                continue

            try:
                values = [float(cell) for cell in cells]
            except ValueError:
                continue

            for name, value in zip(header, values):
                columns[name].append(value)

    if header is None or "time_s" not in columns:
        raise ValueError("Not a telemetry session file")

    return columns, events
