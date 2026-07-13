"""Widget for displaying real-time plots"""

from collections import deque
from datetime import datetime
from typing import Dict, List, Deque

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton

import pyqtgraph as pg


class PlotWidget(QWidget):
    """Widget for displaying real-time plots"""

    def __init__(
        self, title: str, labels: List[str], colors: List[str], max_points: int = 500
    ) -> None:
        super().__init__()
        self.max_points: int = max_points
        self.labels: List[str] = labels
        self.colors: List[str] = colors

        # Data storage
        self.time_data: Deque[float] = deque(maxlen=max_points)
        self.data: dict[str, Deque[float]] = {
            label: deque(maxlen=max_points) for label in labels
        }
        self.start_time: datetime = datetime.now()

        # Create layout
        layout: QVBoxLayout = QVBoxLayout(self)

        # Create plot widget
        self.plot_widget: pg.PlotWidget = pg.PlotWidget(title=title)
        self.plot_widget.setBackground("w")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.setLabel("bottom", "Time (s)")

        # Add legend
        self.plot_widget.addLegend()

        # Create plot curves
        self.curves: dict[str, pg.PlotDataItem] = {}
        for label, color in zip(labels, colors):
            pen = pg.mkPen(color=color, width=2)
            self.curves[label] = self.plot_widget.plot([], [], pen=pen, name=label)

        layout.addWidget(self.plot_widget)

        # Control buttons
        control_layout: QHBoxLayout = QHBoxLayout()

        self.pause_btn: QPushButton = QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.clicked.connect(self.toggle_pause)
        control_layout.addWidget(self.pause_btn)

        clear_btn: QPushButton = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_data)
        control_layout.addWidget(clear_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        self.paused: bool = False

    def toggle_pause(self) -> None:
        self.paused = self.pause_btn.isChecked()
        self.pause_btn.setText("Resume" if self.paused else "Pause")

    def clear_data(self) -> None:
        self.time_data.clear()
        for label in self.labels:
            self.data[label].clear()
        self.start_time = datetime.now()
        self.update_plot()

    def add_data(self, data_dict: Dict[str, float]) -> None:
        if self.paused:
            return

        # Calculate elapsed time
        elapsed: float = (datetime.now() - self.start_time).total_seconds()
        self.time_data.append(elapsed)

        # Add data for each label
        for label in self.labels:
            if label in data_dict:
                self.data[label].append(data_dict[label])
            else:
                self.data[label].append(0)

        self.update_plot()

    def update_plot(self) -> None:
        time_list: tuple[float, ...] = tuple(self.time_data)
        for label in self.labels:
            data_list: tuple[float, ...] = tuple(self.data[label])
            self.curves[label].setData(time_list, data_list)
