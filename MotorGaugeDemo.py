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
import pyqtgraph as pg
import collections

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
        self.sin_cal = None
        self.cos_cal = None
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
        self.sin_cal = self.scope.get_variable("sin_calibrated")
        self.cos_cal = self.scope.get_variable("cos_calibrated")
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

    def read_waveforms(self) -> tuple[float, float, float]:
        if self.demo or self.scope is None:
            ang = self.demo_src.read()
            return math.sin(ang), math.cos(ang), ang
        return (
            float(self.sin_cal.get_value()),  # type: ignore[call-arg]
            float(self.cos_cal.get_value()),  # type: ignore[call-arg]
            float(self.ang.get_value()),      # type: ignore[call-arg]
        )

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


class CompassWidget(QtWidgets.QWidget):
    """Simple compass-like widget with a rotating needle."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.angle = 0.0

    def setAngle(self, ang: float) -> None:
        self.angle = ang
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: D401 - Qt override
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        radius = min(rect.width(), rect.height()) / 2 - 10
        center = rect.center()
        painter.drawEllipse(center, radius, radius)
        painter.save()
        painter.translate(center)
        painter.rotate(-math.degrees(self.angle))
        pen = QtGui.QPen(QtCore.Qt.GlobalColor.red, 3)
        painter.setPen(pen)
        painter.drawLine(0, 0, 0, -radius)
        painter.restore()


class WaveformWindow(QtWidgets.QMainWindow):
    """Window showing sine/cosine waveforms using pyqtgraph."""

    DT_MS = 20

    def __init__(self, scope: _ScopeWrapper) -> None:
        super().__init__()
        self.setWindowTitle("Resolver Waveforms")
        pg.setConfigOptions(antialias=True)
        self.scope = scope
        self.t0 = time.perf_counter()

        central = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(central)
        self.plot = pg.PlotWidget(background="w")
        self.plot.addLegend()
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Value")
        vbox.addWidget(self.plot)
        self.setCentralWidget(central)

        pen_s = pg.mkPen("b", width=1.5)
        pen_c = pg.mkPen("g", width=1.5)
        pen_a = pg.mkPen("m", width=1.5)
        self.curve_s = self.plot.plot(pen=pen_s, name="Sine")
        self.curve_c = self.plot.plot(pen=pen_c, name="Cosine")
        self.curve_a = self.plot.plot(pen=pen_a, name="Angle/π")

        self.data_t: list[float] = []
        self.data_s: list[float] = []
        self.data_c: list[float] = []
        self.data_a: list[float] = []

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update)

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: D401
        self.t0 = time.perf_counter()
        self.data_t.clear()
        self.data_s.clear()
        self.data_c.clear()
        self.data_a.clear()
        self.timer.start(self.DT_MS)
        super().showEvent(event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: D401
        self.timer.stop()
        super().closeEvent(event)

    def _update(self) -> None:
        s, c, ang = self.scope.read_waveforms()
        now = time.perf_counter() - self.t0
        self.data_t.append(now)
        self.data_s.append(s)
        self.data_c.append(c)
        self.data_a.append(ang / math.pi)
        if len(self.data_t) > 1000:
            self.data_t.pop(0)
            self.data_s.pop(0)
            self.data_c.pop(0)
            self.data_a.pop(0)
        self.curve_s.setData(self.data_t, self.data_s)
        self.curve_c.setData(self.data_t, self.data_c)
        self.curve_a.setData(self.data_t, self.data_a)
        self.plot.setXRange(max(0, now - 5), now)

class MotorGaugeDemo(QtWidgets.QMainWindow):
    DT_MS = 20

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Motor Position Gauge")

        self._scope = _ScopeWrapper()
        self.connected = False

        self.wave_win: WaveformWindow | None = None
        self.modes = ["Dial", "Slider", "Bar", "LCD", "Compass"]
        self.rpm_vals = collections.deque(maxlen=20)

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
        gl.addWidget(self.cal_btn, 3, 0, 1, 2)
        self.help_btn = QtWidgets.QPushButton("?")
        self.help_btn.clicked.connect(self._show_help)
        gl.addWidget(self.help_btn, 3, 2)

        self.wave_btn = QtWidgets.QPushButton("Show Waveforms")
        self.wave_btn.clicked.connect(self._show_waveforms)
        gl.addWidget(self.wave_btn, 4, 0, 1, 3)

        self.mode_btn = QtWidgets.QPushButton("Change View")
        self.mode_btn.clicked.connect(self._next_mode)
        gl.addWidget(self.mode_btn, 5, 0, 1, 3)
        vbox.addWidget(conn_box)

        self.stack = QtWidgets.QStackedWidget()
        self.stack.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        vbox.addWidget(self.stack)

        self.dial = QtWidgets.QDial()
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(True)
        self.dial.setRange(0, 359)
        self.dial.setEnabled(False)
        self.stack.addWidget(self.dial)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setRange(0, 359)
        self.slider.setEnabled(False)
        self.stack.addWidget(self.slider)

        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, 359)
        self.stack.addWidget(self.bar)

        self.lcd = QtWidgets.QLCDNumber()
        self.lcd.setDigitCount(6)
        self.stack.addWidget(self.lcd)

        self.compass = CompassWidget()
        self.stack.addWidget(self.compass)

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
        self.rpm_vals.clear()
        self._scope.calibrate()
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
            rpm_inst = delta / (self.DT_MS / 1000.0) * 60 / (2 * math.pi)
            self.rpm_vals.append(rpm_inst)
            rpm = sum(self.rpm_vals) / len(self.rpm_vals)
        self.prev_ang = ang

        deg = math.degrees(ang) % 360
        val = int(deg)
        self.dial.setValue(val)
        self.slider.setValue(val)
        self.bar.setValue(val)
        self.lcd.display(f"{deg:6.1f}")
        self.compass.setAngle(ang)
        self.lbl_angle.setText(f"Angle: {deg:.1f}°")
        self.lbl_speed.setText(f"Speed: {rpm:.1f} RPM")
        self.lbl_turns.setText(f"Turns: {self.turns:.2f}")

    # ---------------------------------------------------------- extra actions ---
    def _run_calibration(self) -> None:
        if self.connected:
            self._scope.calibrate()

    def _show_help(self) -> None:
        msg = (
            "Start the motor and rotate it slowly through at least one full "
            "turn while calibration samples the raw sine and cosine signals. "
            "Extreme noise or clipping may cause inaccurate results."
        )
        QtWidgets.QMessageBox.information(self, "Calibration Help", msg)

    def _show_waveforms(self) -> None:
        if self.wave_win is None:
            self.wave_win = WaveformWindow(self._scope)
        self.wave_win.show()
        self.wave_win.raise_()

    def _next_mode(self) -> None:
        idx = (self.stack.currentIndex() + 1) % self.stack.count()
        self.stack.setCurrentIndex(idx)
        self.mode_btn.setText(f"Mode: {self.modes[idx]}")


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = MotorGaugeDemo()
    win.resize(400, 500)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
