#!/usr/bin/env python3
"""Fun motor gauge demo designed for kids at Maker Faire 2025.

This application presents the motor angle on a colourful dial and star
indicator.  It reuses the data connection from :mod:`MotorGaugeDemo`
so it works with real hardware or in demo mode.  A banner displays the
slogan "Absolute inductive sensors – magnet free encoder solution".
"""

from __future__ import annotations

import math
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
import serial.tools.list_ports

# Reuse the scope wrapper from the regular gauge demo
from MotorGaugeDemo import _ScopeWrapper, StarWidget


class KidMotorGauge(QtWidgets.QMainWindow):
    """Simplified gauge window with bright colours and large text."""

    DT_MS = 20

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("MakerFaire 2025 Motor Fun Demo")

        self._scope = _ScopeWrapper()
        self.connected = False

        self.t0 = time.perf_counter()
        self.prev_ang: float | None = None
        self.turns = 0.0

        self._build_ui()
        self._apply_style()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update)

    # ---------------------------------------------------------------- UI -----
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)

        title = QtWidgets.QLabel("MakerFaire 2025")
        f = title.font()
        f.setPointSize(22)
        f.setBold(True)
        title.setFont(f)
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(title)

        tagline = QtWidgets.QLabel(
            "Absolute inductive sensors – magnet free encoder solution"
        )
        tagline.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(tagline)

        conn_box = QtWidgets.QGroupBox("Connection")
        gl = QtWidgets.QGridLayout(conn_box)
        gl.addWidget(QtWidgets.QLabel("ELF file:"), 0, 0)
        self.elf_edit = QtWidgets.QLineEdit()
        gl.addWidget(self.elf_edit, 0, 1)
        self.browse_btn = QtWidgets.QPushButton("Browse…")
        self.browse_btn.clicked.connect(self._browse)
        gl.addWidget(self.browse_btn, 0, 2)

        gl.addWidget(QtWidgets.QLabel("COM port:"), 1, 0)
        self.port_combo = QtWidgets.QComboBox()
        self._refresh_ports()
        gl.addWidget(self.port_combo, 1, 1)
        self.ref_btn = QtWidgets.QPushButton("↻")
        self.ref_btn.clicked.connect(self._refresh_ports)
        gl.addWidget(self.ref_btn, 1, 2)

        self.conn_btn = QtWidgets.QPushButton("Connect")
        self.conn_btn.clicked.connect(self._toggle_conn)
        gl.addWidget(self.conn_btn, 2, 0, 1, 3)
        vbox.addWidget(conn_box)

        self.star = StarWidget()
        self.star.setFixedSize(200, 200)
        vbox.addWidget(self.star, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        self.dial = QtWidgets.QDial()
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(True)
        self.dial.setRange(0, 359)
        self.dial.setEnabled(False)
        self.dial.setFixedSize(200, 200)
        vbox.addWidget(self.dial, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        self.lbl_angle = QtWidgets.QLabel("Angle: —")
        self.lbl_angle.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        f = self.lbl_angle.font()
        f.setPointSize(16)
        f.setBold(True)
        self.lbl_angle.setFont(f)
        vbox.addWidget(self.lbl_angle)

        self.lbl_speed = QtWidgets.QLabel("Speed: —")
        self.lbl_speed.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(self.lbl_speed)

        self.lbl_turns = QtWidgets.QLabel("Turns: —")
        self.lbl_turns.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(self.lbl_turns)

        vbox.addStretch(1)
        self.setCentralWidget(central)

    def _apply_style(self) -> None:
        base = "#fffaf0"
        accent_red = "#E2231A"
        accent_blue = "#0098DB"
        self.setStyleSheet(
            f"""
            QWidget {{ background-color: {base}; font-family: 'Comic Sans MS'; }}
            QGroupBox {{ border: 2px solid {accent_blue}; margin-top: 6px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; color: {accent_blue}; }}
            QPushButton {{ background-color: {accent_red}; color: white; border-radius: 6px; padding: 4px; }}
            QPushButton:hover {{ background-color: {accent_blue}; }}
            QDial {{ background-color: white; border: 2px solid {accent_blue}; }}
            """
        )

    # ------------------------------------------------------------- utilities --
    @staticmethod
    def _ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()] or ["-"]

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        self.port_combo.addItems(self._ports())

    def _browse(self) -> None:
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select ELF", "", "ELF files (*.elf);;All files (*)"
        )
        if fn:
            self.elf_edit.setText(fn)

    # -------------------------------------------------------------- connect ---
    def _toggle_conn(self) -> None:
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_combo.currentText()
        elf = self.elf_edit.text()
        if not port or port == "-" or not elf:
            QtWidgets.QMessageBox.warning(self, "Missing", "Choose COM port and ELF file")
            return
        try:
            self._scope.connect(port, elf)
        except Exception as e:  # pragma: no cover - hardware errors
            QtWidgets.QMessageBox.critical(self, "Connect", str(e))
            return
        self.connected = True
        self.conn_btn.setText("Disconnect")
        self.t0 = time.perf_counter()
        self.prev_ang = None
        self.turns = 0.0
        self._timer.start(self.DT_MS)

    def _disconnect(self) -> None:
        self._timer.stop()
        self._scope.disconnect()
        self.connected = False
        self.conn_btn.setText("Connect")

    # --------------------------------------------------------------- update ---
    def _update(self) -> None:
        ang = self._scope.read()
        now = time.perf_counter() - self.t0

        if self.prev_ang is None:
            rpm = 0.0
        else:
            delta = ang - self.prev_ang
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            self.turns += delta / (2 * math.pi)
            rpm = delta / (self.DT_MS / 1000.0) * 60 / (2 * math.pi)
        self.prev_ang = ang

        deg = math.degrees(ang) % 360
        val = int(deg)
        self.dial.setValue(val)
        self.star.setAngle(ang)
        self.lbl_angle.setText(f"Angle: {deg:.1f}°")
        self.lbl_speed.setText(f"Speed: {rpm:.1f} RPM")
        self.lbl_turns.setText(f"Turns: {self.turns:.2f}")


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = KidMotorGauge()
    win.resize(400, 500)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
