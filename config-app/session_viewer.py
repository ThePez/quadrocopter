"""Window for looking back at a recorded telemetry session."""

import os
from typing import Dict, List, Optional, Tuple

from PyQt5.QtWidgets import QLabel, QMainWindow, QTabWidget, QVBoxLayout, QWidget

import pyqtgraph as pg

from session_recorder import load_session

# Tab title -> (y-axis label, [(curve name, session column, colour)])
# Names and colours match the live plots in pid_tuner_gui.py.
PLOT_GROUPS: Dict[str, Tuple[str, List[Tuple[str, str, str]]]] = {
    "Angles": (
        "Angle (°)",
        [
            ("Pitch", "pitch_angle", "r"),
            ("Roll", "roll_angle", "g"),
            ("Yaw", "yaw_angle", "b"),
        ],
    ),
    "Rates": (
        "Rate (°/s)",
        [
            ("Pitch", "pitch_rate", "r"),
            ("Roll", "roll_rate", "g"),
            ("Yaw", "yaw_rate", "b"),
        ],
    ),
    "PID Outputs": (
        "Output (µs)",
        [
            ("Pitch", "pitch_pid", "r"),
            ("Roll", "roll_pid", "g"),
            ("Yaw", "yaw_pid", "b"),
        ],
    ),
    "Motors": (
        "Pulse width (µs)",
        [
            ("Front Left", "motor_fl", "r"),
            ("Front Right", "motor_fr", "purple"),
            ("Back Right", "motor_br", "orange"),
            ("Back Left", "motor_bl", "cyan"),
        ],
    ),
    "Battery": ("Voltage (mV)", [("Battery", "battery", "k")]),
    "Loop Timing": (
        "Worst gap between control cycles (µs)",
        [("Loop dt max", "loop_dt_max_us", "k")],
    ),
}


class SessionViewer(QMainWindow):
    """Shows one recorded session as zoomable plots, one tab per group."""

    def __init__(self, path: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        columns, events = load_session(path)
        time_s: List[float] = columns["time_s"]

        self.setWindowTitle(f"Session - {os.path.basename(path)}")
        self.resize(1200, 700)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        duration: float = time_s[-1] if time_s else 0.0
        summary = QLabel(
            f"{os.path.basename(path)}  |  {len(time_s)} samples  |  "
            f"{duration:.1f} s  |  {len(events)} events  |  "
            "Scroll to zoom, drag to pan, right-click for more"
        )
        summary.setStyleSheet("color: #555;")
        layout.addWidget(summary)

        tabs = QTabWidget()
        layout.addWidget(tabs)

        first_plot: Optional[pg.PlotWidget] = None
        for title, (y_label, curves) in PLOT_GROUPS.items():
            plot = pg.PlotWidget(title=title)
            plot.setBackground("w")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setLabel("left", y_label)
            plot.setLabel("bottom", "Time (s)")
            plot.addLegend()

            for name, column, colour in curves:
                if column in columns:
                    plot.plot(
                        time_s,
                        columns[column],
                        pen=pg.mkPen(color=colour, width=2),
                        name=name,
                    )

            # Mark gain changes and other events with labelled vertical lines
            for when, text in events:
                plot.addItem(
                    pg.InfiniteLine(
                        pos=when,
                        angle=90,
                        pen=pg.mkPen(color="#888", style=pg.QtCore.Qt.DashLine),
                        label=text,
                        labelOpts={
                            "position": 0.95,
                            "color": "#555",
                            "rotateAxis": (1, 0),
                        },
                    )
                )

            # Zooming in time on one tab zooms them all
            if first_plot is None:
                first_plot = plot
            else:
                plot.setXLink(first_plot)

            tabs.addTab(plot, title)
