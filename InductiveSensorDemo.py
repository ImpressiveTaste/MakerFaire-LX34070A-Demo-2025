#!/usr/bin/env python3
"""Microchip 2025 MakerFaire InductiveSensor Demo

Displays resolver sine and cosine signals along with the calculated
angle using PyQt6 and pyqtgraph.  If pyX2Cscope is unavailable, runs in
Demo Mode with synthesised data.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg

try:
    from pyx2cscope.x2cscope import X2CScope  # type: ignore
except Exception:  # pragma: no cover - missing dependency
    X2CScope = None

import serial.tools.list_ports


@dataclass
class _DemoSource:
    """Fallback data source when pyX2Cscope isn't available."""

    freq: float = 1.0  # Hz

    def __post_init__(self):
        self.t_last = time.perf_counter()
        self.angle = 0.0

    def read(self) -> tuple[float, float, float]:
        now = time.perf_counter()
        dt = now - self.t_last
        self.t_last = now
        self.angle += 2 * math.pi * self.freq * dt
        s = math.sin(self.angle)
        c = math.cos(self.angle)
        ang = ((self.angle + math.pi) % (2 * math.pi)) - math.pi
        return s, c, ang


class _ScopeWrapper:
    """Tiny wrapper around pyX2Cscope with demo fallback."""

    def __init__(self):
        self.demo = X2CScope is None
        self.scope = None
        self.sin = None
        self.cos = None
        self.ang = None
        self.demo_src = _DemoSource()

    def connect(self, port: str, elf: str) -> None:
        if X2CScope is None:
            self.demo = True
            return
        self.scope = X2CScope(port=port)
        self.scope.import_variables(elf)
        self.sin = self.scope.get_variable("sin_calibrated")
        self.cos = self.scope.get_variable("cos_calibrated")
        self.ang = self.scope.get_variable("resolver_position")
        self.demo = False

    def disconnect(self) -> None:
        if self.scope is not None:
            self.scope.disconnect()
        self.scope = None
        self.demo = X2CScope is None

    def read(self) -> tuple[float, float, float]:
        if self.demo or self.scope is None:
            return self.demo_src.read()
        return (
            float(self.sin.get_value()),  # type: ignore[call-arg]
            float(self.cos.get_value()),  # type: ignore[call-arg]
            float(self.ang.get_value()),  # type: ignore[call-arg]
        )


class InductiveSensorDemo(QtWidgets.QMainWindow):
    DT_MS = 20

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Microchip 2025 MakerFaire InductiveSensor Demo")
        pg.setConfigOptions(antialias=True)

        self._scope = _ScopeWrapper()
        self.connected = False

        self.t0 = time.perf_counter()
        self.prev_ang: float | None = None
        self.turns = 0.0

        self._build_ui()
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
        vbox.addWidget(conn_box)

        self.plot = pg.PlotWidget(background="w")
        self.plot.addLegend()
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Value")
        vbox.addWidget(self.plot)

        pen_sin = pg.mkPen("b", width=1.5)
        pen_cos = pg.mkPen("g", width=1.5)
        pen_ang = pg.mkPen((255, 105, 180), width=1.5)
        self.curve_sin = self.plot.plot(pen=pen_sin, name="Sine")
        self.curve_cos = self.plot.plot(pen=pen_cos, name="Cosine")
        self.curve_ang = self.plot.plot(pen=pen_ang, name="Angle/π")

        hbox = QtWidgets.QHBoxLayout()
        self.lbl_angle = QtWidgets.QLabel("Angle: —")
        self.lbl_speed = QtWidgets.QLabel("Speed: —")
        self.lbl_turns = QtWidgets.QLabel("Turns: —")
        for w in (self.lbl_angle, self.lbl_speed, self.lbl_turns):
            hbox.addWidget(w)
        hbox.addStretch(1)
        vbox.addLayout(hbox)

        self.setCentralWidget(central)

        self.data_t: list[float] = []
        self.data_sin: list[float] = []
        self.data_cos: list[float] = []
        self.data_ang: list[float] = []

    # ------------------------------------------------------------- utilities ---
    @staticmethod
    def _ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()] or ["-"]

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        self.port_combo.addItems(self._ports())

    def _browse(self) -> None:
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select ELF", "", "ELF files (*.elf);;All files (*)")
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
        self.data_t.clear()
        self.data_sin.clear()
        self.data_cos.clear()
        self.data_ang.clear()
        self._timer.start(self.DT_MS)

    def _disconnect(self) -> None:
        self._timer.stop()
        self._scope.disconnect()
        self.connected = False
        self.conn_btn.setText("Connect")

    # --------------------------------------------------------------- update ---
    def _update(self) -> None:
        s, c, ang = self._scope.read()
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

        self.data_t.append(now)
        self.data_sin.append(s)
        self.data_cos.append(c)
        self.data_ang.append(ang / math.pi)
        if len(self.data_t) > 1000:
            self.data_t.pop(0)
            self.data_sin.pop(0)
            self.data_cos.pop(0)
            self.data_ang.pop(0)

        self.curve_sin.setData(self.data_t, self.data_sin)
        self.curve_cos.setData(self.data_t, self.data_cos)
        self.curve_ang.setData(self.data_t, self.data_ang)
        self.plot.setXRange(max(0, now - 5), now)

        self.lbl_angle.setText(f"Angle: {math.degrees(ang):.1f}°")
        self.lbl_speed.setText(f"Speed: {rpm:.1f} RPM")
        self.lbl_turns.setText(f"Turns: {self.turns:.2f}")


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = InductiveSensorDemo()
    win.resize(800, 600)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
