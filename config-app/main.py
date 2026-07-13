"""
PID Tuning GUI for Drone with Real-time Telemetry Plotting
PyQt5 interface for sending PID coefficients and viewing telemetry
"""

import sys

from PyQt5.QtWidgets import QApplication

from pid_tuner_gui import PIDTunerGUI


def main() -> None:
    app: QApplication = QApplication(sys.argv)
    app.setStyle("Fusion")

    window: PIDTunerGUI = PIDTunerGUI()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
