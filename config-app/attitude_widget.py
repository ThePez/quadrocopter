"""3D visualization of the drone's current orientation"""

from typing import List

from PyQt5.QtWidgets import QWidget, QVBoxLayout

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class Attitude3DWidget(QWidget):
    """3D visualization of the drone's current orientation"""

    ARM_LENGTH = 1.0

    def __init__(self) -> None:
        super().__init__()
        layout: QVBoxLayout = QVBoxLayout(self)

        self.gl_view: gl.GLViewWidget = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=6, elevation=25, azimuth=45)
        layout.addWidget(self.gl_view)

        # Ground reference grid
        grid: gl.GLGridItem = gl.GLGridItem()
        grid.setSize(x=10, y=10)
        grid.setSpacing(x=1, y=1)
        grid.translate(0, 0, -1.5)
        self.gl_view.addItem(grid)

        # Arm endpoints in an X configuration. Scene axes: X=right, Y=forward (nose), Z=up.
        s: float = self.ARM_LENGTH * 0.7071  # sin/cos(45 deg)
        self.fl: np.ndarray = np.array([-s, s, 0])
        self.fr: np.ndarray = np.array([s, s, 0])
        self.bl: np.ndarray = np.array([-s, -s, 0])
        self.br: np.ndarray = np.array([s, -s, 0])
        origin: np.ndarray = np.array([0, 0, 0])

        # Front arms (red, matches the Motors plot's FL/FR coloring) vs rear arms (grey),
        # so the model's heading is visually obvious as it yaws.
        self.front_arms: gl.GLLinePlotItem = gl.GLLinePlotItem(
            pos=np.array([origin, self.fl, origin, self.fr]),
            color=(1, 0, 0, 1),
            width=4,
            mode="lines",
            antialias=True,
        )
        self.rear_arms: gl.GLLinePlotItem = gl.GLLinePlotItem(
            pos=np.array([origin, self.bl, origin, self.br]),
            color=(0.5, 0.5, 0.5, 1),
            width=4,
            mode="lines",
            antialias=True,
        )
        self.gl_view.addItem(self.front_arms)
        self.gl_view.addItem(self.rear_arms)

        # Motor tip markers, colors matching the Motors plot (FL, BL, BR, FR)
        self.motor_markers: gl.GLScatterPlotItem = gl.GLScatterPlotItem(
            pos=np.array([self.fl, self.bl, self.br, self.fr]),
            color=np.array(
                [
                    [1, 0, 0, 1],  # FL - red
                    [1, 0.65, 0, 1],  # BL - orange
                    [0.5, 0, 0.5, 1],  # BR - purple
                    [0, 1, 1, 1],  # FR - cyan
                ]
            ),
            size=15,
            pxMode=True,
        )
        self.gl_view.addItem(self.motor_markers)

        # All the pieces above represent one rigid body, so the same transform
        # gets applied to each of them every update.
        self.rigid_body_items: List[gl.GLGraphicsItem] = [
            self.front_arms,
            self.rear_arms,
            self.motor_markers,
        ]

    def update_attitude(self, pitch: float, roll: float, yaw: float) -> None:
        """Rotate the drone model to the given angles (degrees)"""
        # QMatrix4x4.rotate() post-multiplies (self = self * R), so calling it
        # in this order composes as Ryaw * Rpitch * Rroll applied to a point -
        # i.e. roll happens first, then pitch, then yaw. That's the standard
        # aerospace ZYX Euler convention.
        tr: pg.Transform3D = pg.Transform3D()
        tr.rotate(yaw, 0, 0, 1)
        tr.rotate(pitch, 1, 0, 0)
        tr.rotate(roll, 0, 1, 0)

        for item in self.rigid_body_items:
            item.setTransform(tr)
