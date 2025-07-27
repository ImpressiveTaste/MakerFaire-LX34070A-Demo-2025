#!/usr/bin/env python3
"""Microchip 2025 MakerFaire InductiveSensor Demo (Tkinter version)

Displays resolver sine and cosine signals along with the calculated
angle using Tkinter and matplotlib. If pyX2Cscope is unavailable, runs
in Demo Mode with synthesised data.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import tkinter as tk
from tkinter import filedialog, ttk, messagebox

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # type: ignore
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - optional deps missing
    plt = None  # type: ignore

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

    def __init__(self) -> None:
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


class InductiveSensorDemoTk:
    DT_MS = 20

    def __init__(self) -> None:
        if plt is None:
            raise RuntimeError("matplotlib is required for the Tkinter demo")

        self.root = tk.Tk()
        self.root.title("Microchip 2025 MakerFaire InductiveSensor Demo")

        self._scope = _ScopeWrapper()
        self.connected = False

        self.t0 = time.perf_counter()
        self.prev_ang: Optional[float] = None
        self.turns = 0.0

        self._build_ui()
        self._after_id: Optional[str] = None

    # ------------------------------------------------------------------ UI ---
    def _build_ui(self) -> None:
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=10, pady=10)

        conn = ttk.LabelFrame(main, text="Connection", padding=8)
        conn.pack(fill="x")

        ttk.Label(conn, text="ELF file:").grid(row=0, column=0, sticky="e")
        self.elf_var = tk.StringVar()
        ttk.Entry(conn, textvariable=self.elf_var, width=40).grid(row=0, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Browse…", command=self._browse).grid(row=0, column=2)

        ttk.Label(conn, text="COM port:").grid(row=1, column=0, sticky="e", pady=(4,0))
        self.port_var = tk.StringVar()
        self.port_menu = ttk.OptionMenu(conn, self.port_var, "-", *self._ports())
        self.port_menu.grid(row=1, column=1, sticky="we", padx=4, pady=(4,0))
        ttk.Button(conn, text="↻", width=3, command=self._refresh_ports).grid(row=1, column=2, pady=(4,0))

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_conn)
        self.conn_btn.grid(row=2, column=0, columnspan=3, pady=(6, 0))

        fig, ax = plt.subplots()
        fig.set_facecolor("white")
        self.canvas = FigureCanvasTkAgg(fig, master=main)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=8)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.grid(True)
        self._ax = ax
        (self.line_sin,) = ax.plot([], [], label="Sine", color="b")
        (self.line_cos,) = ax.plot([], [], label="Cosine", color="g")
        (self.line_ang,) = ax.plot([], [], label="Angle/π", color="hotpink")
        ax.legend()

        info = ttk.Frame(main)
        info.pack(fill="x")
        self.lbl_angle = ttk.Label(info, text="Angle: —")
        self.lbl_speed = ttk.Label(info, text="Speed: —")
        self.lbl_turns = ttk.Label(info, text="Turns: —")
        for w in (self.lbl_angle, self.lbl_speed, self.lbl_turns):
            w.pack(side="left", padx=6)
        info.pack_propagate(False)

        self.data_t: list[float] = []
        self.data_sin: list[float] = []
        self.data_cos: list[float] = []
        self.data_ang: list[float] = []

    # ------------------------------------------------------------- utilities --
    @staticmethod
    def _ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()] or ["-"]

    def _refresh_ports(self) -> None:
        menu = self.port_menu["menu"]
        menu.delete(0, "end")
        for p in self._ports():
            menu.add_command(label=p, command=lambda v=p: self.port_var.set(v))
        self.port_var.set("-")

    def _browse(self) -> None:
        fn = filedialog.askopenfilename(title="Select ELF", filetypes=[("ELF", "*.elf"), ("All", "*.*")])
        if fn:
            self.elf_var.set(fn)

    # -------------------------------------------------------------- connect ---
    def _toggle_conn(self) -> None:
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get()
        elf = self.elf_var.get()
        if not port or port == "-" or not elf:
            messagebox.showwarning("Missing", "Choose COM port and ELF file")
            return
        try:
            self._scope.connect(port, elf)
        except Exception as e:  # pragma: no cover - hardware errors
            messagebox.showerror("Connect", str(e))
            return
        self.connected = True
        self.conn_btn.config(text="Disconnect")
        self.t0 = time.perf_counter()
        self.prev_ang = None
        self.turns = 0.0
        self.data_t.clear()
        self.data_sin.clear()
        self.data_cos.clear()
        self.data_ang.clear()
        self._schedule_update()

    def _disconnect(self) -> None:
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        self._scope.disconnect()
        self.connected = False
        self.conn_btn.config(text="Connect")

    # --------------------------------------------------------------- update ---
    def _schedule_update(self) -> None:
        self._after_id = self.root.after(self.DT_MS, self._update)

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

        self.line_sin.set_data(self.data_t, self.data_sin)
        self.line_cos.set_data(self.data_t, self.data_cos)
        self.line_ang.set_data(self.data_t, self.data_ang)
        self._ax.relim()
        self._ax.autoscale_view()
        self.canvas.draw()

        self.lbl_angle.config(text=f"Angle: {math.degrees(ang):.1f}°")
        self.lbl_speed.config(text=f"Speed: {rpm:.1f} RPM")
        self.lbl_turns.config(text=f"Turns: {self.turns:.2f}")

        if self.connected:
            self._schedule_update()


def main() -> None:
    demo = InductiveSensorDemoTk()
    demo.root.geometry("800x600")
    demo.root.mainloop()


if __name__ == "__main__":
    main()

