#!/usr/bin/env python3
"""Resolver encoder debug GUI.

This application connects to the dsPIC firmware from ``main.c`` and
provides tools to calibrate the resolver inputs, adjust the encoder
resolution and inspect key variables.  When ``pyX2Cscope`` is not
installed, it runs in a demo mode with synthesised data.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import collections
from PyQt6 import QtCore, QtGui, QtWidgets


try:
    from pyx2cscope.x2cscope import X2CScope  # type: ignore
except Exception:  # pragma: no cover - missing dependency
    X2CScope = None

import serial.tools.list_ports


@dataclass
class _DemoSource:
    """Fallback generator when no hardware is present."""

    freq: float = 1.0  # Hz
    counts_per_rev: int = 500
    sin_offset: float = 0.0
    cos_offset: float = 0.0
    sin_amplitude: float = 1.0
    cos_amplitude: float = 1.0

    def __post_init__(self) -> None:
        self.t_last = time.perf_counter()
        self.angle = 0.0

    def _advance(self) -> float:
        now = time.perf_counter()
        dt = now - self.t_last
        self.t_last = now
        self.angle += 2 * math.pi * self.freq * dt
        ang = ((self.angle + math.pi) % (2 * math.pi)) - math.pi
        return ang

    def read_waveforms(self) -> tuple[float, float, float]:
        ang = self._advance()
        return math.sin(ang), math.cos(ang), ang

    def read_status(self) -> tuple[float, int, int, int]:
        ang = self._advance()
        total = self.counts_per_rev * 4
        idx = int(((ang + math.pi) / (2 * math.pi)) * total) % total
        quad = idx % 4
        count = idx // 4
        a = 1 if quad in (1, 2) else 0
        b = 1 if quad in (2, 3) else 0
        z = 1 if count == 0 else 0
        return ang, a, b, z

    def calibrate(self, duration: float = 1.0, delay: float = 0.005) -> None:
        # Demo calibration simply resets parameters
        self.sin_offset = 0.0
        self.cos_offset = 0.0
        self.sin_amplitude = 1.0
        self.cos_amplitude = 1.0


class _ScopeWrapper:
    """Small helper around ``pyX2Cscope`` with demo fallback."""

    def __init__(self) -> None:
        self.demo = X2CScope is None
        self.scope = None
        self.demo_src = _DemoSource()
        # Variables (set on connect)
        self.ang = None
        self.sin_raw = None
        self.cos_raw = None
        self.sin_off = None
        self.cos_off = None
        self.sin_amp = None
        self.cos_amp = None
        self.sin_cal = None
        self.cos_cal = None
        self.enc_a = None
        self.enc_b = None
        self.enc_z = None
        self.cpr = None

    # ------------------------------------------------------------------ HW ---
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
        self.sin_cal = self.scope.get_variable("sin_calibrated")
        self.cos_cal = self.scope.get_variable("cos_calibrated")
        self.enc_a = self.scope.get_variable("encoder_A")
        self.enc_b = self.scope.get_variable("encoder_B")
        self.enc_z = self.scope.get_variable("encoder_Z")
        self.cpr = self.scope.get_variable("counts_per_rev")
        self.demo = False

    def disconnect(self) -> None:
        if self.scope is not None:
            self.scope.disconnect()
        self.scope = None
        self.demo = X2CScope is None

    # --------------------------------------------------------------- helpers ---
    def read_waveforms(self) -> tuple[float, float, float]:
        if self.demo or self.scope is None:
            return self.demo_src.read_waveforms()
        return (
            float(self.sin_cal.get_value()),  # type: ignore[call-arg]
            float(self.cos_cal.get_value()),  # type: ignore[call-arg]
            float(self.ang.get_value()),      # type: ignore[call-arg]
        )

    def read_status(self) -> tuple[float, int, int, int]:
        if self.demo or self.scope is None:
            return self.demo_src.read_status()
        return (
            float(self.ang.get_value()),      # type: ignore[call-arg]
            int(self.enc_a.get_value()),      # type: ignore[call-arg]
            int(self.enc_b.get_value()),      # type: ignore[call-arg]
            int(self.enc_z.get_value()),      # type: ignore[call-arg]
        )

    def calibrate(self, duration: float = 1.0, delay: float = 0.005) -> None:
        if self.demo or self.scope is None:
            self.demo_src.calibrate(duration, delay)
            return
        samples = max(1, int(duration / delay))
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

    def get_counts(self) -> int:
        if self.demo or self.scope is None:
            return self.demo_src.counts_per_rev
        return int(self.cpr.get_value())

    def set_counts(self, val: int) -> None:
        if self.demo or self.scope is None:
            self.demo_src.counts_per_rev = val
        else:
            self.cpr.set_value(val)

    def get_cal_values(self) -> tuple[float, float, float, float]:
        if self.demo or self.scope is None:
            return (
                self.demo_src.sin_offset,
                self.demo_src.cos_offset,
                self.demo_src.sin_amplitude,
                self.demo_src.cos_amplitude,
            )
        return (
            float(self.sin_off.get_value()),  # type: ignore[call-arg]
            float(self.cos_off.get_value()),  # type: ignore[call-arg]
            float(self.sin_amp.get_value()),  # type: ignore[call-arg]
            float(self.cos_amp.get_value()),  # type: ignore[call-arg]
        )

class WaveformWindow(QtWidgets.QMainWindow):
    """Lightweight window showing sine and cosine waveforms."""
    DT_MS = 50
    MAX_SAMPLES = 500

    def __init__(self, scope: _ScopeWrapper) -> None:
        super().__init__()
        self.setWindowTitle('Resolver Waveforms')

        self.scope = scope
        self.t0 = time.perf_counter()

        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)
        self.canvas = _WaveformCanvas(self)
        vbox.addWidget(self.canvas)
        self.setCentralWidget(central)

        self.data = collections.deque(maxlen=self.MAX_SAMPLES)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update)

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # type: ignore
        self.t0 = time.perf_counter()
        self.data.clear()

        self.timer.start(self.DT_MS)
        super().showEvent(event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # type: ignore
        self.timer.stop()
        super().closeEvent(event)

    def _update(self) -> None:
        s, c, ang = self.scope.read_waveforms()
        now = time.perf_counter() - self.t0
        self.data.append((now, s, c, ang / math.pi))
        self.canvas.update()


class _WaveformCanvas(QtWidgets.QWidget):
    """Very basic plotting widget using QPainter."""

    def __init__(self, window: WaveformWindow) -> None:
        super().__init__(window)
        self.win = window

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: D401
        painter = QtGui.QPainter(self)
        try:
            rect = self.rect()
            painter.fillRect(rect, QtCore.Qt.GlobalColor.white)
            if not self.win.data:
                return
            times = [d[0] for d in self.win.data]
            sin_v = [d[1] for d in self.win.data]
            cos_v = [d[2] for d in self.win.data]
            ang_v = [d[3] for d in self.win.data]
            t0 = times[0]
            t1 = times[-1] if times[-1] != t0 else t0 + 1e-6
            sx = rect.width() / (t1 - t0)
            def pts(vals):
                return [QtCore.QPointF((t - t0) * sx, rect.bottom() - (v + 1) * rect.height() / 2) for t, v in zip(times, vals)]
            for values, color in zip((sin_v, cos_v, ang_v), (QtCore.Qt.GlobalColor.blue, QtCore.Qt.GlobalColor.darkGreen, QtCore.Qt.GlobalColor.magenta)):
                p = pts(values)
                if p:
                    path = QtGui.QPainterPath(p[0])
                    for q in p[1:]:
                        path.lineTo(q)
                    painter.setPen(QtGui.QPen(color, 1))
                    painter.drawPath(path)
        finally:
            painter.end()


class ResolverEncoderDemo(QtWidgets.QMainWindow):
    """Main window for resolver encoder debugging."""

    DT_MS = 20

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Resolver Encoder Debugger")

        self._scope = _ScopeWrapper()
        self.connected = False
        self.wave_win: WaveformWindow | None = None

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

        ctrl_box = QtWidgets.QGroupBox("Controls")
        cl = QtWidgets.QGridLayout(ctrl_box)
        self.cal_btn = QtWidgets.QPushButton("Calibrate")
        self.cal_btn.clicked.connect(self._run_calibration)
        cl.addWidget(self.cal_btn, 0, 0, 1, 2)

        cl.addWidget(QtWidgets.QLabel("Cal. time (s):"), 1, 0)
        self.cal_time_spin = QtWidgets.QDoubleSpinBox()
        self.cal_time_spin.setRange(0.1, 10.0)
        self.cal_time_spin.setSingleStep(0.1)
        self.cal_time_spin.setValue(1.0)
        cl.addWidget(self.cal_time_spin, 1, 1)

        cl.addWidget(QtWidgets.QLabel("Counts/rev:"), 2, 0)
        self.counts_spin = QtWidgets.QSpinBox()
        self.counts_spin.setRange(1, 20000)
        self.counts_spin.setValue(500)
        self.counts_spin.editingFinished.connect(self._set_counts)
        cl.addWidget(self.counts_spin, 2, 1)

        self.wave_btn = QtWidgets.QPushButton("Waveforms")
        self.wave_btn.clicked.connect(self._show_waveforms)
        cl.addWidget(self.wave_btn, 3, 0, 1, 2)

        self.values_btn = QtWidgets.QPushButton("Show Values")
        self.values_btn.clicked.connect(self._show_values)
        cl.addWidget(self.values_btn, 4, 0, 1, 2)
        vbox.addWidget(ctrl_box)

        self.dial = QtWidgets.QDial()
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(True)
        self.dial.setRange(0, 359)
        self.dial.setEnabled(False)
        vbox.addWidget(self.dial, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter)

        self.lbl_angle = QtWidgets.QLabel("Angle: —")
        font = self.lbl_angle.font()
        font.setPointSize(14)
        font.setBold(True)
        self.lbl_angle.setFont(font)
        self.lbl_angle.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(self.lbl_angle)

        self.lbl_abz = QtWidgets.QLabel("A: —  B: —  Z: —")
        self.lbl_abz.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        vbox.addWidget(self.lbl_abz)

        self.setCentralWidget(central)

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
        self.counts_spin.setValue(self._scope.get_counts())
        self._scope.calibrate(duration=self.cal_time_spin.value())
        self._timer.start(self.DT_MS)

    def _disconnect(self) -> None:
        self._timer.stop()
        self._scope.disconnect()
        self.connected = False
        self.conn_btn.setText("Connect")
        if self.wave_win is not None:
            self.wave_win.close()

    # --------------------------------------------------------------- update ---
    def _update(self) -> None:
        ang, a, b, z = self._scope.read_status()
        deg = math.degrees(ang) % 360
        self.dial.setValue(int(deg))
        self.lbl_angle.setText(f"Angle: {deg:.1f}°  |  Counts/rev: {self._scope.get_counts()}")
        self.lbl_abz.setText(f"A: {a}  B: {b}  Z: {z}")

    # ----------------------------------------------------------- user actions --
    def _run_calibration(self) -> None:
        if self.connected:
            self._scope.calibrate(duration=self.cal_time_spin.value())

    def _set_counts(self) -> None:
        if self.connected:
            self._scope.set_counts(self.counts_spin.value())

    def _show_values(self) -> None:
        off_s, off_c, amp_s, amp_c = self._scope.get_cal_values()
        msg = (
            f"sin_offset = {off_s:.4f}\n"
            f"cos_offset = {off_c:.4f}\n"
            f"sin_amplitude = {amp_s:.6f}\n"
            f"cos_amplitude = {amp_c:.6f}\n"
            f"counts_per_rev = {self._scope.get_counts()}"
        )
        QtWidgets.QMessageBox.information(self, "Calibration Values", msg)

    def _show_waveforms(self) -> None:
        if self.wave_win is None:
            self.wave_win = WaveformWindow(self._scope)
        self.wave_win.show()
        self.wave_win.raise_()


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = ResolverEncoderDemo()
    win.resize(400, 500)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
