#!/usr/bin/env python3
"""Motor position gauge demo using PyQt6.

This tiny application shows the motor angle on a dial gauge.  The data
source is shared with :mod:`InductiveSensorDemo`, so it works either with
a real target via ``pyX2Cscope`` or in demo mode if that package is not
available.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

from PyQt6 import QtCore, QtGui, QtWidgets

try:
    from pyx2cscope.x2cscope import X2CScope  # type: ignore
except Exception:  # pragma: no cover - missing dependency
    X2CScope = None

import serial.tools.list_ports


@dataclass
class _DemoSource:
    """Fallback data source when pyX2Cscope isn't available."""

    freq: float = 1.0  # Hz

    def __post_init__(self) -> None:
        self.t_last = time.perf_counter()
        self.angle = 0.0

    def read(self) -> float:
        now = time.perf_counter()
        dt = now - self.t_last
        self.t_last = now
        self.angle += 2 * math.pi * self.freq * dt
        return ((self.angle + math.pi) % (2 * math.pi)) - math.pi


class _ScopeWrapper:
    """Tiny wrapper around pyX2Cscope with demo fallback."""

    def __init__(self) -> None:
        self.demo = X2CScope is None
        self.scope = None
        self.ang = None
        self.sin_raw = None
        self.cos_raw = None
        self.sin_off = None
        self.cos_off = None
        self.sin_amp = None
        self.cos_amp = None
        self.demo_src = _DemoSource()

    def connect(self, port: str, elf: str) -> None:
        if X2CScope is None:
            self.demo = True
            return
        self.scope = X2CScope(port=port)
        self.scope.import_variables(elf)
        self.ang = self.scope.get_variable("resolver_position")
        self.sin_raw = self.scope.get_variable("sin_raw")
        self.cos_raw = self.scope.get_variable("cos_raw")
        self.sin_off = self.scope.get_variable("sin_offset")
        self.cos_off = self.scope.get_variable("cos_offset")
        self.sin_amp = self.scope.get_variable("sin_amplitude")
        self.cos_amp = self.scope.get_variable("cos_amplitude")
        self.demo = False

    def disconnect(self) -> None:
        if self.scope is not None:
            self.scope.disconnect()
        self.scope = None
        self.demo = X2CScope is None

    def read(self) -> float:
        if self.demo or self.scope is None:
            return self.demo_src.read()
        return float(self.ang.get_value())  # type: ignore[call-arg]

    def calibrate(self, samples: int = 200, delay: float = 0.005) -> None:
        """Measure raw waveforms and update offset/amplitude registers.

        The resolver generates sine and cosine signals that may be shifted
        or scaled.  ``calibrate`` samples the raw values for ``samples``
        iterations, finds their min/max, then computes offset and amplitude
        correction factors which are written back to the target via X2CScope.
        """
        if self.demo or self.scope is None:
            return
        min_s = float("inf")
        max_s = float("-inf")
        min_c = float("inf")
        max_c = float("-inf")
        for _ in range(samples):
            s = float(self.sin_raw.get_value())  # type: ignore[call-arg]
            c = float(self.cos_raw.get_value())  # type: ignore[call-arg]
            min_s = min(min_s, s)
            max_s = max(max_s, s)
            min_c = min(min_c, c)
            max_c = max(max_c, c)
            time.sleep(delay)
        off_s = (max_s + min_s) / 2.0
        off_c = (max_c + min_c) / 2.0
        amp_s = 2.0 / (max_s - min_s) if max_s != min_s else 1.0
        amp_c = 2.0 / (max_c - min_c) if max_c != min_c else 1.0
        self.sin_off.set_value(off_s)
        self.cos_off.set_value(off_c)
        self.sin_amp.set_value(amp_s)
        self.cos_amp.set_value(amp_c)


class MotorGaugeDemo(QtWidgets.QMainWindow):
    DT_MS = 20

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Motor Position Gauge")

        self._scope = _ScopeWrapper()
        self.connected = False

        self.t0 = time.perf_counter()
        self.prev_ang: float | None = None
        self.turns = 0.0

        self._build_ui()
        self._apply_style()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update)

    # ------------------------------------------------------------------ UI ---
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)

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
        self.cal_btn = QtWidgets.QPushButton("Calibrate")
        self.cal_btn.clicked.connect(self._run_calibration)
        gl.addWidget(self.cal_btn, 3, 0, 1, 3)
        self.wave_btn = QtWidgets.QPushButton("Show Waveforms")
        self.wave_btn.clicked.connect(self._show_waveforms)
        gl.addWidget(self.wave_btn, 4, 0, 1, 3)
        vbox.addWidget(conn_box)

        self.dial = QtWidgets.QDial()
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(True)
        self.dial.setRange(0, 359)
        self.dial.setEnabled(False)
        self.dial.setMinimumSize(150, 150)
        self.dial.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        # Let the layout control its geometry so it scales with the window
        vbox.addWidget(self.dial)

        self.lbl_angle = QtWidgets.QLabel("Angle: —")
        font = self.lbl_angle.font()
        font.setPointSize(14)
        font.setBold(True)
        self.lbl_angle.setFont(font)
        self.lbl_angle.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
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
        # Maker Faire inspired colour scheme
        base = "#ffffff"
        accent_red = "#E2231A"  # Maker Faire red
        accent_blue = "#0098DB"  # Maker Faire blue
        text = "#000000"
        self.setStyleSheet(
            f"""
            QWidget {{ background-color: {base}; color: {text}; font-family: Arial; }}
            QGroupBox {{ border: 1px solid {accent_blue}; margin-top: 6px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; color: {accent_blue}; }}
            QPushButton {{ background-color: {accent_red}; color: white; border-radius: 4px; padding: 4px; }}
            QPushButton:hover {{ background-color: {accent_blue}; }}
            QDial {{ background-color: {base}; border: 2px solid {accent_blue}; }}
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
        self._scope.calibrate()
        self._timer.start(self.DT_MS)

    def _disconnect(self) -> None:
        self._timer.stop()
        self._scope.disconnect()
        self.connected = False
        self.conn_btn.setText("Connect")

    # --------------------------------------------------------------- update ---
    def _update(self) -> None:
        ang = self._scope.read()

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
        self.dial.setValue(int(deg))
        self.lbl_angle.setText(f"Angle: {deg:.1f}°")
        self.lbl_speed.setText(f"Speed: {rpm:.1f} RPM")
        self.lbl_turns.setText(f"Turns: {self.turns:.2f}")

    # ---------------------------------------------------------- extra actions ---
    def _run_calibration(self) -> None:
        if self.connected:
            self._scope.calibrate()

    def _show_waveforms(self) -> None:
        import subprocess, sys, os
        script = os.path.join(os.path.dirname(__file__), "InductiveSensorDemo.py")
        subprocess.Popen([sys.executable, script])


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = MotorGaugeDemo()
    win.resize(300, 400)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
