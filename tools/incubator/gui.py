"""
gui.py — PySide6 window for the incubator bring-up tool.

Visual approach: the app's own theme (``gui/styles.py``) plus the supplemental
palette/QSS in :mod:`.theme`, since the app QSS alone leaves stock widgets on the
platform's light palette. See ``theme.py`` for why all three pieces are needed.

Threading: controller callbacks arrive on background threads.
:class:`_HeaterBridge` re-emits them as Qt signals and PySide6's automatic queued
connections marshal delivery onto the GUI thread, so nothing here needs a lock.

Glyph policy: labels use plain words and ASCII rather than decorative symbols.
Characters like U+27F3 are missing from Segoe UI and render as tofu boxes, which
is what "icons don't load" looked like. Only degree/plus-minus signs are used,
which every Windows UI font has.
"""

from __future__ import annotations

import time
from collections import deque

from PySide6.QtCore import QObject, Qt, QPointF, Signal, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPlainTextEdit, QProgressBar, QPushButton, QScrollArea, QSizePolicy,
    QSpinBox, QSplitter,
    QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, scaled_font_size
from gui.styles import COLORS

from . import device_config
from .controller import IncubatorController
from .safety import CAUTION_SETPOINT_C, MAX_SETPOINT_C
from .stability import format_duration
from .theme import mono_font, ui_font_family
from .zones import ALL_ZONES, ZoneSpec

ZONE_COLORS = {"bed": "blue", "hotend": "peach"}


def _c(key: str, fallback: str = "#cdd6f4") -> QColor:
    return QColor(COLORS.get(key, fallback))


def _hex(key: str, fallback: str = "#cdd6f4") -> str:
    return COLORS.get(key, fallback)


def _plot_font() -> QFont:
    """
    Small font for in-plot labels.

    Uses an explicitly resolved family rather than ``QFont("")``: an empty family
    lets Qt substitute whatever it likes, and the substitute lacks glyphs such as
    the em dash, which then renders as a stray bar inside the legend.
    """
    f = QFont(ui_font_family())
    f.setPointSize(max(7, int(sf(8))))
    return f


# ═══════════════════════════════════════════════════════════════════
# Thread bridge
# ═══════════════════════════════════════════════════════════════════

class _HeaterBridge(QObject):
    """Re-emits controller callbacks as Qt signals (thread-safe by construction)."""

    channels = Signal(object)
    zone_state = Signal(str, object, object)
    raw_line = Signal(str, str)
    pid_updated = Signal(object)
    autotune_progress = Signal(str, object, float)
    autotune_done = Signal(object, object, str, str)
    fault = Signal(object)
    divergence = Signal(object)
    connection_changed = Signal(bool)
    probe_done = Signal(object)
    status = Signal(str)
    ramp = Signal(str, object)

    def attach(self, ctrl: IncubatorController) -> None:
        ctrl.on_channels(self.channels.emit)
        ctrl.on_zone_state(self.zone_state.emit)
        ctrl.on_raw_line(self.raw_line.emit)
        ctrl.on_pid(self.pid_updated.emit)
        ctrl.on_autotune_progress(self.autotune_progress.emit)
        ctrl.on_autotune_done(
            lambda z, pid, err, hint: self.autotune_done.emit(z, pid, err, hint)
        )
        ctrl.on_fault(self.fault.emit)
        ctrl.on_divergence(self.divergence.emit)
        ctrl.on_connection_changed(self.connection_changed.emit)
        ctrl.on_probe(self.probe_done.emit)
        ctrl.on_status(self.status.emit)
        ctrl.on_ramp(self.ramp.emit)


# ═══════════════════════════════════════════════════════════════════
# Small presentation widgets
# ═══════════════════════════════════════════════════════════════════

class _Pill(QLabel):
    """A coloured state badge. Clearer at a glance than a sentence."""

    def __init__(self, text: str = "", tone: str = "overlay0",
                 parent: QWidget | None = None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self.set_state(text, tone)

    def set_state(self, text: str, tone: str = "overlay0") -> None:
        self.setText(text)
        self.setStyleSheet(
            f"background:{_hex(tone)};color:{_hex('crust', '#11111b')};"
            f"border-radius:{s(9)}px;padding:{s(2)}px {s(10)}px;"
            f"font-weight:700;font-size:{sf(8.5)}pt;"
        )


class _Banner(QFrame):
    """Persistent, severity-coloured message strip. Hidden when empty."""

    TONES = {"info": "blue", "warn": "yellow", "error": "red"}

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(s(10), s(7), s(10), s(7))
        self._label = QLabel("")
        self._label.setWordWrap(True)
        lay.addWidget(self._label, 1)
        self._dismiss = QPushButton("Dismiss")
        self._dismiss.clicked.connect(self.clear)
        lay.addWidget(self._dismiss)
        self.setVisible(False)

    def show_message(self, text: str, severity: str = "info",
                     dismissable: bool = True) -> None:
        tone = _hex(self.TONES.get(severity, "blue"))
        self.setStyleSheet(
            f"QFrame{{background:{tone};border-radius:{s(5)}px;}}"
            f"QLabel{{color:{_hex('crust', '#11111b')};font-weight:600;}}"
            f"QPushButton{{background:rgba(0,0,0,60);color:{_hex('crust')};"
            f"border:none;border-radius:{s(3)}px;padding:{s(3)}px {s(8)}px;}}"
        )
        self._label.setText(text)
        self._dismiss.setVisible(dismissable)
        self.setVisible(True)

    def clear(self) -> None:
        self._label.setText("")
        self.setVisible(False)


class _DutyBar(QWidget):
    """Heater duty as a labelled bar — far easier to read than a number."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))
        cap = QLabel("Heater")
        cap.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(cap)
        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setValue(0)
        self._bar.setFormat("%p%")
        lay.addWidget(self._bar, 1)

    def set_duty(self, pct: float | None) -> None:
        if pct is None:
            self._bar.setValue(0)
            self._bar.setFormat("—")
            return
        v = max(0, min(100, int(round(pct))))
        self._bar.setValue(v)
        self._bar.setFormat("%p%")
        # Saturation is the interesting signal, so colour for it.
        tone = "blue" if v < 80 else ("yellow" if v < 95 else "red")
        self._bar.setStyleSheet(
            f"QProgressBar::chunk{{background-color:{_hex(tone)};"
            f"border-radius:{s(3)}px;}}"
        )


# ═══════════════════════════════════════════════════════════════════
# Trend plot
# ═══════════════════════════════════════════════════════════════════

class _TempTrendPlot(QWidget):
    """
    Rolling temperature trend with a real °C axis.

    Follows ``gui/pages/print_results.py::ErrorTimeSeriesWidget`` for the labelled
    axis and gridlines, and the DPI-scaled style of the timing-workflow strip
    charts. Per zone: actual (solid), target (dashed step), duty (faint, on its
    own 0-100% scale along the bottom).
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(230))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._window_s = 900.0
        self._series: dict[str, deque] = {}
        self._labels: dict[str, str] = {}
        self._t_latest = 0.0
        self._show_duty = True

    def set_window_s(self, window_s: float) -> None:
        self._window_s = max(30.0, float(window_s))
        self._trim()
        self.update()

    def set_show_duty(self, show: bool) -> None:
        self._show_duty = bool(show)
        self.update()

    def clear(self) -> None:
        self._series.clear()
        self._t_latest = 0.0
        self.update()

    def add_sample(self, zone_id: str, label: str, temp_c: float,
                   target_c: float | None, duty_pct: float | None) -> None:
        dq = self._series.setdefault(zone_id, deque())
        self._labels[zone_id] = label
        t = time.monotonic()
        self._t_latest = max(self._t_latest, t)
        dq.append((t, float(temp_c), target_c, duty_pct))
        self._trim()
        self.update()

    def _trim(self) -> None:
        cutoff = self._t_latest - self._window_s
        for dq in self._series.values():
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, _c("crust", "#11111b"))

        pad_l, pad_r = s(50), s(14)
        pad_t, pad_b = s(26), s(30)
        pw = max(1, w - pad_l - pad_r)
        ph = max(1, h - pad_t - pad_b)

        if not any(len(dq) >= 2 for dq in self._series.values()):
            p.setPen(_c("overlay0", "#6c7086"))
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Temperature trend — connect and set a target to populate")
            p.end()
            return

        vals: list[float] = []
        for dq in self._series.values():
            for _t, v, tg, _d in dq:
                vals.append(v)
                if tg:
                    vals.append(tg)
        lo, hi = min(vals), max(vals)
        if hi - lo < 4.0:
            mid = (hi + lo) / 2.0
            lo, hi = mid - 2.0, mid + 2.0
        margin = (hi - lo) * 0.12
        lo, hi = lo - margin, hi + margin
        span = max(hi - lo, 1e-6)

        # X range: use the DATA extent until it outgrows the selected window.
        # Always spanning the full window would squash a fresh run into a spike
        # at the right-hand edge, which is exactly what it did before.
        t_hi = self._t_latest
        t_first = min(
            (dq[0][0] for dq in self._series.values() if dq), default=t_hi
        )
        have_s = max(t_hi - t_first, 1e-3)
        shown_s = min(self._window_s, max(have_s * 1.05, 20.0))
        t_lo = t_hi - shown_s
        t_span = max(shown_s, 1e-3)

        def _x(t: float) -> float:
            return pad_l + (t - t_lo) / t_span * pw

        def _y(v: float) -> float:
            return pad_t + ph - (v - lo) / span * ph

        p.setFont(_plot_font())
        for tv in self._nice_ticks(lo, hi):
            yy = _y(tv)
            p.setPen(QPen(_c("surface0", "#313244"), 1, Qt.DotLine))
            p.drawLine(int(pad_l), int(yy), int(pad_l + pw), int(yy))
            p.setPen(_c("subtext0", "#a6adc8"))
            p.drawText(int(s(6)), int(yy + s(4)), f"{tv:g}°")

        p.setPen(QPen(_c("surface1", "#45475a"), 1))
        p.drawLine(int(pad_l), int(pad_t), int(pad_l), int(pad_t + ph))
        p.drawLine(int(pad_l), int(pad_t + ph), int(pad_l + pw), int(pad_t + ph))

        if self._show_duty:
            duty_h = ph * 0.26
            base = pad_t + ph
            for zid, dq in self._series.items():
                col = _c(ZONE_COLORS.get(zid, "text"))
                col.setAlpha(55)
                p.setPen(QPen(col, s(1)))
                prev = None
                for t, _v, _tg, d in dq:
                    if d is None:
                        prev = None
                        continue
                    pt = QPointF(_x(t), base - (d / 100.0) * duty_h)
                    if prev is not None:
                        p.drawLine(prev, pt)
                    prev = pt

        for zid, dq in self._series.items():
            col = _c(ZONE_COLORS.get(zid, "text"))
            col.setAlpha(140)
            p.setPen(QPen(col, s(1), Qt.DashLine))
            prev = None
            for t, _v, tg, _d in dq:
                if not tg:
                    prev = None
                    continue
                pt = QPointF(_x(t), _y(tg))
                if prev is not None:
                    p.drawLine(QPointF(prev.x(), prev.y()), QPointF(pt.x(), prev.y()))
                    p.drawLine(QPointF(pt.x(), prev.y()), pt)
                prev = pt

        for zid, dq in self._series.items():
            p.setPen(QPen(_c(ZONE_COLORS.get(zid, "text")), s(2)))
            prev = None
            for t, v, _tg, _d in dq:
                pt = QPointF(_x(t), _y(v))
                if prev is not None:
                    p.drawLine(prev, pt)
                prev = pt

        p.setPen(_c("overlay0", "#6c7086"))
        p.drawText(int(pad_l), int(pad_t + ph + s(17)),
                   f"-{format_duration(shown_s)}")
        p.drawText(int(pad_l + pw - s(26)), int(pad_t + ph + s(17)), "now")

        lx, ly = pad_l + s(4), s(16)
        for zid in sorted(self._series):
            p.setPen(QPen(_c(ZONE_COLORS.get(zid, "text")), s(2)))
            p.drawLine(int(lx), int(ly - s(4)), int(lx + s(15)), int(ly - s(4)))
            p.setPen(_c("subtext0", "#a6adc8"))
            label = self._labels.get(zid, zid)
            p.drawText(int(lx + s(20)), int(ly), label)
            lx += s(20) + p.fontMetrics().horizontalAdvance(label) + s(20)
        p.end()

    @staticmethod
    def _nice_ticks(lo: float, hi: float, target: int = 5) -> list[float]:
        span = hi - lo
        if span <= 0:
            return [lo]
        raw = span / max(1, target)
        step = 50.0
        for cand in (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50):
            if raw <= cand:
                step = float(cand)
                break
        first = step * (int(lo / step) + (1 if lo > 0 else 0))
        out, v = [], first
        while v <= hi and len(out) < 20:
            out.append(round(v, 3))
            v += step
        return out


# ═══════════════════════════════════════════════════════════════════
# Zone control card
# ═══════════════════════════════════════════════════════════════════

class _ZoneCard(QGroupBox):
    """Readout + setpoint controls for one zone."""

    def __init__(self, spec: ZoneSpec, ctrl: IncubatorController,
                 parent: QWidget | None = None):
        super().__init__(spec.title, parent)
        self._spec = spec
        self._ctrl = ctrl

        lay = QVBoxLayout(self)
        lay.setContentsMargins(s(12), s(14), s(12), s(12))
        lay.setSpacing(s(7))

        # ── header: state pill + wiring reminder ──
        top = QHBoxLayout()
        self._pill = _Pill("OFFLINE", "overlay0")
        top.addWidget(self._pill)
        top.addStretch(1)
        wiring = QLabel(f"{spec.heater_connector} / {spec.sensor_connector}")
        wiring.setStyleSheet(
            f"color:{_hex('overlay0', '#6c7086')};font-size:{sf(8.5)}pt;"
        )
        top.addWidget(wiring)
        lay.addLayout(top)

        # ── big temperature ──
        self._temp = QLabel("—")
        f = QFont()
        f.setPointSize(scaled_font_size(28))
        f.setBold(True)
        self._temp.setFont(f)
        self._temp.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._temp)

        self._sub = QLabel(spec.blurb)
        self._sub.setAlignment(Qt.AlignCenter)
        self._sub.setWordWrap(True)
        self._sub.setStyleSheet(
            f"color:{_hex('overlay0', '#6c7086')};font-size:{sf(8.5)}pt;"
        )
        lay.addWidget(self._sub)

        self._target_lbl = QLabel("Target: off")
        self._target_lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._target_lbl)

        self._duty = _DutyBar()
        lay.addWidget(self._duty)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color:{_hex('surface1', '#45475a')};")
        lay.addWidget(line)

        # ── setpoint ──
        row = QHBoxLayout()
        row.addWidget(QLabel("Setpoint"))
        self._sp = QDoubleSpinBox()
        self._sp.setRange(0.0, MAX_SETPOINT_C)
        self._sp.setDecimals(1)
        self._sp.setSingleStep(0.5)
        self._sp.setValue(37.0)
        self._sp.setSuffix(" °C")
        row.addWidget(self._sp, 1)
        self._set_btn = QPushButton("Set")
        self._set_btn.setObjectName("primaryButton")
        self._set_btn.setToolTip(
            "Command the setpoint directly. On stock firmware a big jump can "
            "false-trip Marlin's heat-up watchdog on a slow water block — use "
            "Ramp for that."
        )
        self._set_btn.clicked.connect(self._on_set)
        row.addWidget(self._set_btn)
        self._ramp_btn = QPushButton("Ramp")
        self._ramp_btn.setObjectName("primaryButton")
        self._ramp_btn.setToolTip(
            "Walk the setpoint up in small steps so Marlin's heat-up watchdog "
            "never arms. Use this on stock firmware, where commanding 37 C "
            "outright makes the board declare thermal runaway and halt."
        )
        self._ramp_btn.clicked.connect(self._on_ramp_clicked)
        row.addWidget(self._ramp_btn)
        lay.addLayout(row)

        self._ramp_lbl = QLabel("")
        self._ramp_lbl.setWordWrap(True)
        self._ramp_lbl.setVisible(False)
        lay.addWidget(self._ramp_lbl)

        prow = QHBoxLayout()
        prow.setSpacing(s(4))
        for val in (25.0, 30.0, 37.0):
            b = QPushButton(f"{val:g}°")
            b.setToolTip(f"Set {val:g} °C")
            b.clicked.connect(lambda _=False, v=val: self._apply_preset(v))
            prow.addWidget(b)
        self._off_btn = QPushButton("Heater OFF")
        self._off_btn.setObjectName("warnButton")
        self._off_btn.clicked.connect(
            lambda: self._ctrl.heater_off(self._spec.zone_id)
        )
        prow.addWidget(self._off_btn, 1)
        lay.addLayout(prow)

        # ── fine setpoint ──
        frow = QHBoxLayout()
        self._fine = QCheckBox("Fine setpoint, every")
        self._fine.setToolTip(
            "Marlin stores targets as whole degrees, so 37.5 °C cannot be sent "
            "directly. This alternates the integer setpoint either side of your "
            "value; the block's large thermal mass averages it into a smooth "
            "fractional hold."
        )
        frow.addWidget(self._fine)
        self._fine_period = QSpinBox()
        self._fine_period.setRange(10, 600)
        self._fine_period.setValue(60)
        self._fine_period.setSuffix(" s")
        frow.addWidget(self._fine_period)
        frow.addStretch(1)
        lay.addLayout(frow)

        self._plan = QLabel("")
        self._plan.setWordWrap(True)
        self._plan.setStyleSheet(
            f"color:{_hex('overlay0', '#6c7086')};font-size:{sf(8.5)}pt;"
        )
        lay.addWidget(self._plan)

        self._blocked = QLabel("")
        self._blocked.setWordWrap(True)
        self._blocked.setVisible(False)
        lay.addWidget(self._blocked)

        # Offered right next to the block message, because that is where the
        # operator is looking when they have just plugged a sensor in and want
        # the tool to notice.
        self._rescan_btn = QPushButton("↻  Re-check this sensor")
        self._rescan_btn.setToolTip(
            "Re-read the board's sensors (one M105) and re-evaluate this zone.\n"
            "Use after plugging a thermistor in: Marlin reports it immediately, "
            "but this tool caches its verdict from connect time.\n"
            "Touches no heater and changes no setpoint."
        )
        self._rescan_btn.setVisible(False)
        self._rescan_btn.clicked.connect(self._ctrl.rescan_sensors)
        lay.addWidget(self._rescan_btn)

        self._sp.valueChanged.connect(self._refresh_plan)
        lay.addStretch(1)

    # ── actions ─────────────────────────────────────────────────────

    def _apply_preset(self, value: float) -> None:
        self._sp.setValue(value)
        self._on_set()

    def _on_set(self) -> None:
        if not self._ctrl.connected:
            QMessageBox.information(self, "Not connected",
                                    "Connect to the board first.")
            return
        plan = self._ctrl.preview_setpoint(self._spec.zone_id, self._sp.value())
        chk = plan["check"]

        if chk.clamped or chk.needs_confirm:
            msg = [f"Set {self._spec.title} to {chk.allowed_c:g} °C?"]
            if chk.reason:
                msg.append(f"\n\nThis value was {chk.reason}.")
            if chk.allowed_c > CAUTION_SETPOINT_C:
                msg.append(
                    "\n\nThe vessel contains water. Well above body temperature "
                    "there is an evaporation and scald risk."
                )
            if QMessageBox.question(
                self, "Confirm setpoint", "".join(msg),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            ) != QMessageBox.Yes:
                return

        if self._fine.isChecked():
            self._ctrl.set_fine_target(
                self._spec.zone_id, chk.allowed_c,
                period_s=float(self._fine_period.value()),
            )
        else:
            self._ctrl.set_target(self._spec.zone_id, chk.allowed_c)
        self._refresh_plan()

    def _on_ramp_clicked(self) -> None:
        if not self._ctrl.connected:
            QMessageBox.information(self, "Not connected",
                                    "Connect to the board first.")
            return
        zid = self._spec.zone_id
        if self._ctrl.ramp_active(zid):
            self._ctrl.stop_ramp(zid)
            return
        plan = self._ctrl.preview_setpoint(zid, self._sp.value())
        chk = plan["check"]
        if chk.needs_confirm or chk.clamped:
            msg = [f"Ramp {self._spec.title} up to {chk.allowed_c:g} °C?"]
            if chk.reason:
                msg.append(f"\n\nThis value was {chk.reason}.")
            if QMessageBox.question(
                self, "Confirm ramp", "".join(msg),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            ) != QMessageBox.Yes:
                return
        self._ctrl.start_ramp(zid, chk.allowed_c)

    def show_ramp(self, st) -> None:
        """Render ramp progress on the card."""
        if st is None or not (st.active or st.stalled):
            self._ramp_lbl.setVisible(False)
            self._ramp_btn.setText("Ramp")
            return
        self._ramp_btn.setText("Stop ramp")
        if st.stalled:
            tone = "yellow"
            text = (
                f"Ramp STALLED at step {st.current_step_c:.0f} °C — the heater "
                f"cannot get further. Check steady-state duty."
            )
        else:
            tone = "blue"
            text = (
                f"Ramping to {st.final_target_c:.1f} °C — commanding "
                f"{st.current_step_c:.0f} °C now  ({st.progress_pct:.0f}% of the "
                f"way, {format_duration(st.elapsed_s)} elapsed)"
            )
        self._ramp_lbl.setText(text)
        self._ramp_lbl.setStyleSheet(
            f"background:{_hex('surface0', '#313244')};color:{_hex(tone)};"
            f"border-left:{s(3)}px solid {_hex(tone)};border-radius:{s(3)}px;"
            f"padding:{s(5)}px;font-size:{sf(8.5)}pt;"
        )
        self._ramp_lbl.setVisible(True)

    def _refresh_plan(self) -> None:
        if not self._ctrl.connected:
            self._plan.setText("")
            return
        try:
            plan = self._ctrl.preview_setpoint(self._spec.zone_id, self._sp.value())
        except Exception:
            return
        bits = [f"sends {self._spec.set_cmd} S{plan['commanded_c']}"]
        if plan["calibrated"]:
            bits.append(f"expect {plan['predicted_real_c']:.2f} °C real")
        qe = plan["quantisation_error_c"]
        if abs(qe) >= 0.05 and not self._fine.isChecked():
            bits.append(f"{qe:+.2f} °C from whole-degree rounding")
        self._plan.setText("  ·  ".join(bits))

    # ── updates ─────────────────────────────────────────────────────

    def update_state(self, report, runtime) -> None:
        if report.temp_c is None:
            return
        self._temp.setText(f"{report.temp_c:.2f} °C")

        if not runtime.sensor_ok:
            self._pill.set_state("SENSOR FAULT", "red")
            self._temp.setStyleSheet(f"color:{_hex('red', '#f38ba8')};")
        elif not report.target_c:
            self._pill.set_state("OFF", "overlay0")
            self._temp.setStyleSheet(f"color:{_hex('text', '#cdd6f4')};")
        elif report.in_band:
            self._pill.set_state("HOLDING" if report.settled else "AT TARGET",
                                 "green")
            self._temp.setStyleSheet(f"color:{_hex('green', '#a6e3a1')};")
        else:
            rising = (report.error_c or 0) < 0
            self._pill.set_state("HEATING" if rising else "COOLING", "peach")
            self._temp.setStyleSheet(f"color:{_hex('peach', '#fab387')};")

        ch = self._ctrl.hub.marlin_channel(self._spec.temp_key)
        if ch is not None:
            if ch.calibrated:
                self._sub.setText(
                    f"raw {ch.raw_c:.2f} °C, corrected {ch.offset_c:+.2f} °C"
                )
            else:
                self._sub.setText(f"raw sensor reading, uncalibrated")

        if report.target_c:
            extra = ""
            if runtime.dither_enabled:
                extra = f" (dithering, board at {runtime.commanded_c} °C)"
            elif runtime.commanded_c:
                extra = f" (board target {runtime.commanded_c} °C)"
            self._target_lbl.setText(
                f"Target {report.target_c:.2f} °C{extra}   "
                f"error {report.error_c:+.2f} °C"
            )
        else:
            self._target_lbl.setText("Target: off")

        self._duty.set_duty(report.duty_pct)

    def update_capability(self, cap) -> None:
        """Enable/disable controls based on what the firmware and sensor allow."""
        usable = cap.sensor_present and not cap.sensor_fault
        for wdg in (self._sp, self._set_btn, self._ramp_btn, self._fine,
                    self._fine_period):
            wdg.setEnabled(usable)
        self._off_btn.setEnabled(cap.sensor_present)

        if not cap.sensor_present:
            self._pill.set_state("NO SENSOR", "red")
            self._show_blocked(
                f"The firmware reports no sensor on {self._spec.sensor_connector} "
                f"(no '{self._spec.temp_key}:' field in M105). This zone cannot be "
                f"controlled until TEMP_SENSOR is configured — see FIRMWARE_NOTES.md. "
                f"Note a configured-but-unplugged sensor still shows up here as an "
                f"open circuit, so a missing field points at the firmware build, "
                f"not the wiring.",
                "red", rescan=True,
            )
            return

        if cap.sensor_fault:
            self._pill.set_state("SENSOR FAULT", "red")
            self._show_blocked(
                f"Sensor fault: {cap.sensor_fault} Heating is blocked — driving a "
                f"heater with a broken sensor is how thermal runaway happens. "
                f"Plugged one in just now? Re-check it below.",
                "red", rescan=True,
            )
            return

        if not cap.pid_available:
            self._pill.set_state("BANG-BANG", "yellow")
            self._show_blocked(
                f"This zone has no PID in firmware, so it switches fully on/off "
                f"and will swing about ±1-2 °C. Enable "
                f"{self._spec.pid_config_symbol} for a stable hold.",
                "yellow",
            )
            return

        self._pill.set_state("READY", "green")
        self._blocked.setVisible(False)
        self._rescan_btn.setVisible(False)
        self._refresh_plan()

    def _show_blocked(self, text: str, tone: str, *, rescan: bool = False) -> None:
        self._blocked.setText(text)
        self._blocked.setStyleSheet(
            f"background:{_hex('surface0', '#313244')};"
            f"color:{_hex(tone)};border-left:{s(3)}px solid {_hex(tone)};"
            f"border-radius:{s(3)}px;padding:{s(6)}px;font-size:{sf(8.5)}pt;"
        )
        self._blocked.setVisible(True)
        self._rescan_btn.setVisible(rescan)


# ═══════════════════════════════════════════════════════════════════
# Main window
# ═══════════════════════════════════════════════════════════════════

class IncubatorWindow(QMainWindow):

    def __init__(self, ctrl: IncubatorController | None = None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.ctrl = ctrl or IncubatorController()
        self.bridge = _HeaterBridge()
        self._last_at: tuple[str, object] | None = None
        #: True while the banner is showing a message the probe put there. Lets a
        #: later probe/rescan take its own message down once resolved, without
        #: clearing a banner some other event owns.
        self._probe_flagged = False

        self.setWindowTitle("MEBP Incubator Heater — standalone bring-up tool")
        self.resize(s(1240), s(880))

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(s(12), s(12), s(12), s(10))
        outer.setSpacing(s(9))

        outer.addWidget(self._build_connection_bar())
        self._banner = _Banner()
        outer.addWidget(self._banner)

        split = QSplitter(Qt.Vertical)
        split.addWidget(self._build_upper())
        split.addWidget(self._build_lower())
        split.setStretchFactor(0, 6)
        split.setStretchFactor(1, 3)
        # Favour the live readouts on first open; the tabs below are reference
        # material. The left column scrolls, so a short window stays usable.
        split.setSizes([s(590), s(280)])
        outer.addWidget(split, 1)

        outer.addWidget(self._build_bottom_bar())

        self._wire()

        # Seed from whatever the controller already knows. The probe fires during
        # connect(), so a window built against an already-connected controller
        # would otherwise show an empty Firmware tab and no capability labels.
        self._set_connected_ui(self.ctrl.connected)
        if self.ctrl.report is not None:
            self._on_probe(self.ctrl.report)
        if self.ctrl.connected:
            self._on_pid({
                z.zone_id: self.ctrl.zone_runtime(z.zone_id).pid for z in ALL_ZONES
            })
            self._on_channels(self.ctrl.hub.all_channels())
            # The status line would otherwise still read "Not connected", since
            # connection_changed fired before this window existed.
            self._status.setText(
                f"Connected on {self.ctrl.active_port or 'simulator'}."
            )

        self._tick = QTimer(self)
        self._tick.setInterval(1000)
        self._tick.timeout.connect(self._on_tick)
        self._tick.start()

    # ── construction ────────────────────────────────────────────────

    def _build_connection_bar(self) -> QWidget:
        box = QGroupBox("Board connection")
        lay = QHBoxLayout(box)
        lay.setContentsMargins(s(12), s(14), s(12), s(12))
        lay.setSpacing(s(8))

        lay.addWidget(QLabel("Port"))
        self._port = QComboBox()
        self._port.setEditable(True)
        self._port.setMinimumWidth(s(190))
        lay.addWidget(self._port)

        rescan = QPushButton("Rescan")
        rescan.setToolTip("Re-enumerate serial ports")
        rescan.clicked.connect(self._refresh_ports)
        lay.addWidget(rescan)

        detect = QPushButton("Detect board")
        detect.setToolTip(
            "Ask each serial port for its firmware identity (M115) and select the "
            "one that answers as Marlin. Read-only and safe."
        )
        detect.clicked.connect(self._on_detect)
        lay.addWidget(detect)

        lay.addWidget(QLabel("Baud"))
        self._baud = QComboBox()
        self._baud.setEditable(True)
        for b in device_config.COMMON_BAUDS:
            self._baud.addItem(str(b))
        self._baud.setCurrentText(str(device_config.DEFAULT_BAUD))
        self._baud.setToolTip(
            "38400 matches the ZP board in this repo. Note a board using the MCU's "
            "native USB (like the SKR Mini E3 V3) ignores baud entirely."
        )
        lay.addWidget(self._baud)

        self._sim = QCheckBox("Simulate")
        self._sim.setToolTip(
            "Run against a built-in simulated board with a real thermal model — "
            "no hardware required."
        )
        lay.addWidget(self._sim)
        self._sim_scale = QSpinBox()
        self._sim_scale.setRange(1, 5000)
        self._sim_scale.setValue(300)
        self._sim_scale.setPrefix("x")
        self._sim_scale.setToolTip(
            "Simulated-time acceleration. The real rig's thermal time constant is "
            "of order an hour, which is impractical to watch in real time."
        )
        lay.addWidget(self._sim_scale)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setObjectName("primaryButton")
        self._connect_btn.setMinimumWidth(s(110))
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        lay.addWidget(self._connect_btn)

        self._conn_pill = _Pill("DISCONNECTED", "overlay0")
        lay.addWidget(self._conn_pill)

        self._fw = QLabel("")
        self._fw.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(self._fw, 1)

        self._refresh_ports()
        return box

    def _build_upper(self) -> QWidget:
        wrap = QWidget()
        lay = QHBoxLayout(wrap)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(10))

        cards = QWidget()
        cl = QVBoxLayout(cards)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(s(10))
        self._cards: dict[str, _ZoneCard] = {}
        for spec in ALL_ZONES:
            card = _ZoneCard(spec, self.ctrl)
            self._cards[spec.zone_id] = card
            cl.addWidget(card)
        cl.addStretch(1)

        # Scroll rather than squeeze: a zone showing a fault message needs more
        # height than one that is fine, and compressing the cards made controls
        # overlap each other.
        scroll = QScrollArea()
        scroll.setWidget(cards)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setFixedWidth(s(372))
        lay.addWidget(scroll)

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(s(8))

        trend = QGroupBox("Temperature trend")
        tl = QVBoxLayout(trend)
        tl.setContentsMargins(s(10), s(14), s(10), s(10))
        ctlrow = QHBoxLayout()
        ctlrow.addWidget(QLabel("Window"))
        self._window = QComboBox()
        for label, secs in (("5 min", 300), ("15 min", 900), ("1 hour", 3600),
                            ("6 hours", 21600)):
            self._window.addItem(label, secs)
        self._window.setCurrentIndex(1)
        self._window.currentIndexChanged.connect(
            lambda: self._plot.set_window_s(float(self._window.currentData()))
        )
        ctlrow.addWidget(self._window)
        self._duty_chk = QCheckBox("Show heater duty")
        self._duty_chk.setChecked(True)
        self._duty_chk.toggled.connect(lambda v: self._plot.set_show_duty(v))
        ctlrow.addWidget(self._duty_chk)
        ctlrow.addStretch(1)
        self._log_btn = QPushButton("Start logging")
        self._log_btn.setCheckable(True)
        self._log_btn.setToolTip("Record every sample to logs/incubator/ as JSONL")
        self._log_btn.toggled.connect(self._on_log_toggled)
        ctlrow.addWidget(self._log_btn)
        tl.addLayout(ctlrow)
        self._plot = _TempTrendPlot()
        tl.addWidget(self._plot, 1)
        rl.addWidget(trend, 1)

        sensors = QGroupBox("Sensor channels")
        sl = QVBoxLayout(sensors)
        sl.setContentsMargins(s(10), s(14), s(10), s(10))

        srow = QHBoxLayout()
        srow.addStretch(1)
        self._rescan_all_btn = QPushButton("↻  Rescan sensors")
        self._rescan_all_btn.setToolTip(
            "Re-read every sensor (one M105) and re-evaluate which zones may be "
            "heated.\nPress after plugging a thermistor in — the firmware picks it "
            "up straight away, but this tool caches its verdict from connect "
            "time.\nNo heater is touched and no setpoint changes."
        )
        self._rescan_all_btn.clicked.connect(self.ctrl.rescan_sensors)
        srow.addWidget(self._rescan_all_btn)
        sl.addLayout(srow)

        self._table = QTableWidget(0, 6)
        self._table.setHorizontalHeaderLabels(
            ["Channel", "Value", "Raw", "Target", "Duty", "State"]
        )
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.NoSelection)
        self._table.setAlternatingRowColors(True)
        self._table.setFixedHeight(s(124))
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.Stretch)
        for _col in range(1, 6):
            hdr.setSectionResizeMode(_col, QHeaderView.ResizeToContents)
        sl.addWidget(self._table)
        rl.addWidget(sensors)

        lay.addWidget(right, 1)
        return wrap

    def _build_lower(self) -> QWidget:
        tabs = QTabWidget()
        tabs.addTab(self._build_health_tab(), "Stability && health")
        tabs.addTab(self._build_pid_tab(), "PID")
        tabs.addTab(self._build_autotune_tab(), "Autotune")
        tabs.addTab(self._build_calibration_tab(), "Sensor calibration")
        tabs.addTab(self._build_firmware_tab(), "Firmware")
        tabs.addTab(self._build_console_tab(), "Console")
        return tabs

    def _build_bottom_bar(self) -> QWidget:
        bar = QWidget()
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(s(2), 0, s(2), 0)
        lay.setSpacing(s(8))

        self._status = QLabel("Not connected.")
        self._status.setWordWrap(True)
        lay.addWidget(self._status, 1)

        self._all_off = QPushButton("ALL HEATERS OFF")
        self._all_off.setObjectName("warnButton")
        self._all_off.setToolTip("Set every zone's target to 0 (M140 S0 / M104 S0)")
        self._all_off.setMinimumWidth(s(150))
        self._all_off.clicked.connect(self.ctrl.all_heaters_off)
        lay.addWidget(self._all_off)

        self._estop = QPushButton("EMERGENCY STOP")
        self._estop.setObjectName("dangerButton")
        self._estop.setMinimumWidth(s(150))
        self._estop.setToolTip(
            "M112. HALTS the board — it ignores everything until power-cycled. "
            "For a normal stop use ALL HEATERS OFF."
        )
        self._estop.clicked.connect(self._on_estop)
        lay.addWidget(self._estop)
        return bar

    def _build_health_tab(self) -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setSpacing(s(10))
        self._health: dict[str, dict[str, QLabel]] = {}
        rows = [
            ("temp", "Temperature"), ("error", "Error"),
            ("rate", "Rate of change"), ("eta", "Time to setpoint"),
            ("settle", "Settle time"), ("overshoot", "Overshoot"),
            ("ripple", "Steady ripple"), ("duty", "Steady-state duty"),
            ("tau", "Thermal time constant"),
        ]
        for spec in ALL_ZONES:
            box = QGroupBox(spec.title)
            g = QGridLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.setColumnStretch(1, 1)
            fields: dict[str, QLabel] = {}
            for i, (key, label) in enumerate(rows):
                lbl = QLabel(label)
                lbl.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
                g.addWidget(lbl, i, 0)
                val = QLabel("—")
                val.setStyleSheet("font-weight:600;")
                fields[key] = val
                g.addWidget(val, i, 1)
            note = QLabel("")
            note.setWordWrap(True)
            note.setStyleSheet(
                f"color:{_hex('subtext0', '#a6adc8')};padding-top:{s(6)}px;"
            )
            g.addWidget(note, len(rows), 0, 1, 2)
            fields["note"] = note
            self._health[spec.zone_id] = fields
            lay.addWidget(box)
        return w

    def _build_pid_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "These are the constants the BOARD is using. Marlin has no \"get PID\" "
            "command, so they are read from the M503 settings dump. If a zone's "
            "line is absent, PID is not compiled in for that heater and it runs "
            "bang-bang."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        row = QHBoxLayout()
        row.setSpacing(s(10))
        self._pid_fields: dict[str, dict] = {}
        for spec in ALL_ZONES:
            box = QGroupBox(f"{spec.title}   ({spec.pid_cmd})")
            g = QGridLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.addWidget(QLabel("On board"), 0, 1)
            g.addWidget(QLabel("New value"), 0, 2)
            entry: dict = {}
            for i, key in enumerate(("kp", "ki", "kd"), start=1):
                lbl = QLabel(key.capitalize())
                lbl.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
                g.addWidget(lbl, i, 0)
                cur = QLabel("—")
                cur.setStyleSheet("font-weight:600;")
                g.addWidget(cur, i, 1)
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 10000.0)
                spin.setDecimals(2)
                g.addWidget(spin, i, 2)
                entry[key] = cur
                entry[f"{key}_spin"] = spin
            apply_btn = QPushButton("Apply to board (RAM)")
            apply_btn.clicked.connect(
                lambda _=False, z=spec.zone_id: self._on_apply_pid(z)
            )
            g.addWidget(apply_btn, 4, 0, 1, 3)
            status = QLabel("")
            status.setWordWrap(True)
            g.addWidget(status, 5, 0, 1, 3)
            entry["status"] = status
            entry["apply"] = apply_btn
            self._pid_fields[spec.zone_id] = entry
            row.addWidget(box)
        lay.addLayout(row)

        btns = QHBoxLayout()
        for text, slot, tip in (
            ("Refresh from board (M503)", self.ctrl.query_pid,
             "Re-read the board's settings"),
            ("Save to EEPROM (M500)", self._on_save_eeprom,
             "Persist current values across power cycles"),
            ("Reload EEPROM (M501)", self._on_load_eeprom,
             "Discard unsaved changes and reload stored values"),
            ("Factory reset (M502)", self._on_factory_reset,
             "Load firmware defaults into RAM (not saved until M500)"),
        ):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            btns.addWidget(b)
        btns.addStretch(1)
        lay.addLayout(btns)
        lay.addStretch(1)
        return w

    def _build_autotune_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "PID autotune (M303) drives the heater through heat/cool cycles and "
            "computes constants. On a large water mass with a low-power heater a "
            "cycle can exceed Marlin's internal time limit and fail with "
            "\"timeout\" — a firmware limit, not a fault here. Results apply to RAM "
            "only; save to EEPROM to keep them."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        row = QHBoxLayout()
        row.addWidget(QLabel("Zone"))
        self._at_zone = QComboBox()
        for spec in ALL_ZONES:
            self._at_zone.addItem(spec.title, spec.zone_id)
        row.addWidget(self._at_zone)
        row.addWidget(QLabel("Target"))
        self._at_target = QDoubleSpinBox()
        self._at_target.setRange(0.0, MAX_SETPOINT_C)
        self._at_target.setValue(37.0)
        self._at_target.setSuffix(" °C")
        row.addWidget(self._at_target)
        row.addWidget(QLabel("Cycles"))
        self._at_cycles = QSpinBox()
        self._at_cycles.setRange(1, 20)
        self._at_cycles.setValue(3)
        row.addWidget(self._at_cycles)
        self._at_apply = QCheckBox("Apply result (U1)")
        self._at_apply.setChecked(True)
        row.addWidget(self._at_apply)
        self._at_start = QPushButton("Start autotune")
        self._at_start.setObjectName("primaryButton")
        self._at_start.clicked.connect(self._on_autotune_start)
        row.addWidget(self._at_start)
        self._at_cancel = QPushButton("Cancel (M108)")
        self._at_cancel.clicked.connect(self.ctrl.cancel_autotune)
        row.addWidget(self._at_cancel)
        row.addStretch(1)
        lay.addLayout(row)

        self._at_status = QLabel("Idle.")
        self._at_status.setWordWrap(True)
        lay.addWidget(self._at_status)

        self._at_result = QPlainTextEdit()
        self._at_result.setReadOnly(True)
        lay.addWidget(self._at_result, 1)

        self._at_adopt = QPushButton("Copy result into the PID tab")
        self._at_adopt.setEnabled(False)
        self._at_adopt.clicked.connect(self._on_adopt_result)
        lay.addWidget(self._at_adopt)
        return w

    def _build_calibration_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        info = QLabel(
            "SENSOR calibration corrects what the board reports against a trusted "
            "external thermometer. It is applied on the host, at the display and "
            "setpoint-translation layer only — the firmware's thermistor tables are "
            "never touched. This is a different thing from PID autotune, which "
            "tunes the control loop.\n\n"
            "Put a reference thermometer next to the sensor, wait for both to "
            "settle, then enter the reference reading."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
        lay.addWidget(info)

        self._cal_fields: dict[str, dict] = {}
        for spec in ALL_ZONES:
            box = QGroupBox(spec.title)
            g = QHBoxLayout(box)
            g.setContentsMargins(s(12), s(14), s(12), s(12))
            g.addWidget(QLabel("Board reads"))
            board = QLabel("—")
            board.setStyleSheet("font-weight:600;")
            g.addWidget(board)
            g.addSpacing(s(12))
            g.addWidget(QLabel("Reference reads"))
            ref = QDoubleSpinBox()
            ref.setRange(0.0, 120.0)
            ref.setDecimals(2)
            ref.setValue(37.0)
            ref.setSuffix(" °C")
            g.addWidget(ref)
            apply_btn = QPushButton("Apply offset")
            apply_btn.clicked.connect(
                lambda _=False, z=spec.zone_id: self._on_calibrate(z)
            )
            g.addWidget(apply_btn)
            clear = QPushButton("Clear")
            clear.clicked.connect(
                lambda _=False, z=spec.zone_id: self.ctrl.clear_calibration(z)
            )
            g.addWidget(clear)
            state = QLabel("uncalibrated")
            state.setStyleSheet(f"color:{_hex('overlay0', '#6c7086')};")
            g.addWidget(state, 1)
            self._cal_fields[spec.zone_id] = {
                "board": board, "ref": ref, "state": state,
            }
            lay.addWidget(box)
        lay.addStretch(1)
        return w

    def _build_firmware_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        self._fw_text = QPlainTextEdit()
        self._fw_text.setReadOnly(True)
        mono = mono_font(scaled_font_size(9))
        self._fw_text.setFont(mono)
        self._fw_text.setPlainText(
            "Connect to probe the firmware.\n\n"
            "The probe asks the board what it actually supports (M115, M105, M503) "
            "instead of assuming, and names the exact Configuration.h option for "
            "anything missing."
        )
        lay.addWidget(self._fw_text)
        return w

    def _build_console_tab(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(4000)
        mono = mono_font(scaled_font_size(9))
        self._console.setFont(mono)
        lay.addWidget(self._console, 1)

        row = QHBoxLayout()
        self._show_temps = QCheckBox("Show temperature lines")
        self._show_temps.setToolTip(
            "Autoreport pushes one line per second; hide them to see other traffic."
        )
        row.addWidget(self._show_temps)
        clear = QPushButton("Clear")
        clear.clicked.connect(self._console.clear)
        row.addWidget(clear)
        row.addStretch(1)
        row.addWidget(QLabel("Send G-code"))
        self._raw_in = QLineEdit()
        self._raw_in.setPlaceholderText("e.g. M105")
        self._raw_in.returnPressed.connect(self._on_send_raw)
        self._raw_in.setMinimumWidth(s(240))
        row.addWidget(self._raw_in)
        send = QPushButton("Send")
        send.clicked.connect(self._on_send_raw)
        row.addWidget(send)
        lay.addLayout(row)
        return w

    # ── wiring ──────────────────────────────────────────────────────

    def _wire(self) -> None:
        b = self.bridge
        b.attach(self.ctrl)
        b.channels.connect(self._on_channels)
        b.zone_state.connect(self._on_zone_state)
        b.raw_line.connect(self._on_raw_line)
        b.pid_updated.connect(self._on_pid)
        b.autotune_progress.connect(self._on_at_progress)
        b.autotune_done.connect(self._on_at_done)
        b.fault.connect(self._on_fault)
        b.divergence.connect(self._on_divergence)
        b.connection_changed.connect(self._on_connection)
        b.probe_done.connect(self._on_probe)
        b.status.connect(self._on_status)
        b.ramp.connect(self._on_ramp)

    # ── connection ──────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        current = self._port.currentText()
        self._port.clear()
        for dev, score, desc in device_config.ranked_ports():
            hint = "  (likely the board)" if score >= 100 else ""
            self._port.addItem(f"{dev}{hint}", dev)
        stale = device_config.stale_hints()
        if current:
            self._port.setCurrentText(current)
        elif stale and self._port.count() == 0:
            self._status.setText(
                f"No serial ports found. The saved port {', '.join(stale)} is not "
                f"present — check the USB cable."
            )

    def _selected_port(self) -> str:
        data = self._port.currentData()
        if data:
            return str(data)
        # The user may have typed a port name directly.
        return self._port.currentText().split()[0] if self._port.currentText() else ""

    def _on_detect(self) -> None:
        self._status.setText("Scanning for a Marlin board…")
        self._status.repaint()
        found = self.ctrl.detect_board()
        self._refresh_ports()
        if found is None:
            self._banner.show_message(
                "No Marlin board found on any serial port. Check the USB cable, and "
                "make sure the main MEBP app is not connected to the board — only "
                "one program can hold a COM port.",
                "error",
            )
            return
        idx = self._port.findData(found.port)
        if idx >= 0:
            self._port.setCurrentIndex(idx)
        else:
            self._port.setCurrentText(found.port)
        self._baud.setCurrentText(str(found.baud))
        self._banner.show_message(
            f"Found {found.firmware} on {found.port}. Press Connect.", "info"
        )

    def _on_connect_clicked(self) -> None:
        if self.ctrl.connected:
            self.ctrl.disconnect()
            return
        self._connect_btn.setEnabled(False)
        self._conn_pill.set_state("CONNECTING", "yellow")
        self._status.setText("Connecting and probing firmware…")
        self._status.repaint()
        try:
            baud = int(self._baud.currentText())
        except ValueError:
            baud = device_config.DEFAULT_BAUD
        try:
            ok = self.ctrl.connect(
                self._selected_port(), baud,
                simulate=self._sim.isChecked(),
                sim_time_scale=float(self._sim_scale.value()),
            )
            if not ok:
                self._conn_pill.set_state("DISCONNECTED", "overlay0")
                self._banner.show_message(
                    "Could not connect. Use \"Detect board\" to scan for it, check "
                    "the USB cable, and make sure the main MEBP app is not holding "
                    "the port.",
                    "error",
                )
        finally:
            self._connect_btn.setEnabled(True)

    def _on_connection(self, connected: bool) -> None:
        self._set_connected_ui(connected)
        if not connected:
            self._banner.show_message(
                "Link lost. IMPORTANT: the board runs its own control loop and will "
                "keep holding its last setpoint. Power it down if that is not what "
                "you want.",
                "error",
            )
        else:
            self._banner.clear()
            self._plot.clear()
            port = self.ctrl.active_port or "simulator"
            self._status.setText(f"Connected on {port}.")

    def _set_connected_ui(self, connected: bool) -> None:
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        self._conn_pill.set_state(
            ("SIMULATED" if self.ctrl.simulated else "CONNECTED") if connected
            else "DISCONNECTED",
            ("mauve" if self.ctrl.simulated else "green") if connected else "overlay0",
        )
        for wdg in (self._port, self._baud, self._sim, self._sim_scale):
            wdg.setEnabled(not connected)

    # ── probe ───────────────────────────────────────────────────────

    def _on_probe(self, report) -> None:
        fw = report.firmware_name or "unknown firmware"
        port = self.ctrl.active_port
        self._fw.setText(f"{fw}" + (f"  on {port}" if port else ""))

        for spec in ALL_ZONES:
            self._cards[spec.zone_id].update_capability(report.zone(spec.zone_id))

        lines = [
            f"Firmware       : {report.firmware_name or '(not reported)'}",
            f"Port / baud    : {self.ctrl.active_port or '(simulated)'}"
            + (f" @ {self.ctrl.active_baud}" if self.ctrl.active_baud else ""),
            f"Sensor fields  : {', '.join(report.sensor_fields) or '(none)'}",
            f"Setpoint step  : {report.setpoint_resolution_c:g} C"
            + ("   (whole degrees only)" if report.integer_setpoints else ""),
            "",
            "Capabilities:",
        ]
        if report.capabilities:
            for k, v in sorted(report.capabilities.items()):
                lines.append(f"    {'yes' if v else 'NO '}  {k}")
        else:
            lines.append("    (not reported — built without "
                         "EXTENDED_CAPABILITIES_REPORT)")
        lines.append("")
        for spec in ALL_ZONES:
            cap = report.zone(spec.zone_id)
            lines.append(f"{spec.title}")
            lines.append(
                f"    sensor      : "
                f"{'present' if cap.sensor_present else 'MISSING'}"
                f"  ({spec.sensor_connector}, M105 '{spec.temp_key}:')"
                + (f"  reading {cap.reading_c:.2f} C"
                   if cap.reading_c is not None else "")
            )
            lines.append(f"    control     : {cap.control_mode}")
            if cap.pid:
                lines.append(
                    f"    PID         : Kp={cap.pid.kp:.2f}  Ki={cap.pid.ki:.2f}  "
                    f"Kd={cap.pid.kd:.2f}"
                )
            if cap.sensor_fault:
                lines.append(f"    SENSOR FAULT: {cap.sensor_fault}")
            if cap.note:
                lines.append(f"    note        : {cap.note}")
            lines.append("")

        warnings = report.warnings()
        if warnings:
            lines.append("=" * 66)
            lines.append("THINGS THAT WILL LIMIT THIS RIG")
            lines.append("=" * 66)
            for i, wtext in enumerate(warnings, 1):
                lines.append(f"{i}. {wtext}")
                lines.append("")
        if report.errors:
            lines.append("Probe errors:")
            lines.extend(f"  * {e}" for e in report.errors)
        self._fw_text.setPlainText("\n".join(lines))

        # Lead with the most serious thing found.
        faults = [
            spec for spec in ALL_ZONES
            if report.zone(spec.zone_id).sensor_fault
            or not report.zone(spec.zone_id).sensor_present
        ]
        if faults:
            names = ", ".join(z.title for z in faults)
            self._banner.show_message(
                f"{names}: sensor problem — heating is blocked for that zone. "
                f"See the Firmware tab for the detail.",
                "error",
            )
            self._probe_flagged = True
        elif warnings:
            self._banner.show_message(
                (f"{len(warnings)} firmware limitation detected — see the "
                 f"Firmware tab." if len(warnings) == 1 else
                 f"{len(warnings)} firmware limitations detected — see the "
                 f"Firmware tab."),
                "warn",
            )
            self._probe_flagged = True
        elif self._probe_flagged:
            # A re-check cleared what we were complaining about; leaving the old
            # banner up would contradict the now-unblocked zone card.
            self._banner.clear()
            self._probe_flagged = False

    # ── data ────────────────────────────────────────────────────────

    def _on_channels(self, channels) -> None:
        self._table.setRowCount(len(channels))
        for r, ch in enumerate(channels):
            state, tone = "live", "green"
            if ch.stale:
                state, tone = "stale", "yellow"
            elif ch.raw_c < 0:
                state, tone = "sensor fault", "red"
            elif ch.calibrated:
                state, tone = "calibrated", "blue"
            vals = [
                ch.label,
                f"{ch.value_c:.2f} °C",
                f"{ch.raw_c:.2f} °C",
                "—" if ch.target_c is None else f"{ch.target_c:.1f} °C",
                "—" if ch.power_pct is None else f"{ch.power_pct:.0f} %",
                state,
            ]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                if col == 5:
                    item.setForeground(_c(tone))
                if col in (1, 2, 3, 4):
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._table.setItem(r, col, item)

        for spec in ALL_ZONES:
            cal = self._cal_fields.get(spec.zone_id)
            chan = self.ctrl.hub.marlin_channel(spec.temp_key)
            if cal and chan is not None:
                cal["board"].setText(f"{chan.raw_c:.2f} °C")
                cal["state"].setText(
                    self.ctrl.calibration.get(chan.uid).describe()
                )

    def _on_zone_state(self, zone_id: str, report, runtime) -> None:
        card = self._cards.get(zone_id)
        if card is not None:
            card.update_state(report, runtime)

        f = self._health.get(zone_id)
        if f is not None:
            f["temp"].setText(
                "—" if report.temp_c is None else f"{report.temp_c:.2f} °C")
            f["error"].setText(
                "—" if report.error_c is None else f"{report.error_c:+.2f} °C")
            f["rate"].setText(
                "—" if report.rate_c_per_min is None
                else f"{report.rate_c_per_min:+.3f} °C/min")
            f["eta"].setText(
                "reached" if (report.eta_s is not None and report.eta_s <= 0.5)
                else format_duration(report.eta_s))
            f["settle"].setText(format_duration(report.settle_time_s))
            f["overshoot"].setText(
                "—" if report.overshoot_c is None
                else f"{report.overshoot_c:.2f} °C")
            f["ripple"].setText(
                "—" if report.ripple_half_c is None
                else f"± {report.ripple_half_c:.3f} °C")
            f["duty"].setText(
                "—" if report.steady_duty_pct is None
                else f"{report.steady_duty_pct:.1f} %")
            f["tau"].setText(format_duration(report.time_constant_s))
            note = report.headroom_note
            if self.ctrl.simulated:
                # Rates and durations are measured in wall-clock, so accelerated
                # simulated time makes them look implausibly fast. Say so rather
                # than letting someone read 60 °C/min as a real figure.
                note = (note + "  ").strip() + "  (simulated time accelerated)"
            f["note"].setText(note)

        if report.temp_c is not None and runtime.sensor_ok:
            spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
            self._plot.add_sample(
                zone_id, spec.title, report.temp_c, report.target_c, report.duty_pct
            )

    def _on_tick(self) -> None:
        if not self.ctrl.connected:
            return
        self._plot.update()
        for zid, card in self._cards.items():
            st = self.ctrl.ramp_state(zid)
            if st is not None and (st.active or st.stalled):
                card.show_ramp(st)

    def _on_raw_line(self, direction: str, text: str) -> None:
        if not self._show_temps.isChecked():
            low = text.strip().lower()
            if (low.startswith("ok t:") or low.startswith("t:")
                    or low.startswith("ok b:") or low.startswith("b:")):
                return
        self._console.appendPlainText(
            ("TX  " if direction == "tx" else "RX  ") + text
        )

    # ── PID ─────────────────────────────────────────────────────────

    def _on_pid(self, snapshot) -> None:
        for zone_id, pid in (snapshot or {}).items():
            e = self._pid_fields.get(zone_id)
            if e is None:
                continue
            spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
            if pid is None:
                for k in ("kp", "ki", "kd"):
                    e[k].setText("unavailable")
                    e[k].setStyleSheet(
                        f"color:{_hex('yellow', '#f9e2af')};font-weight:600;")
                    e[f"{k}_spin"].setEnabled(False)
                e["apply"].setEnabled(False)
                e["status"].setText(
                    f"This heater has no PID in firmware — enable "
                    f"{spec.pid_config_symbol} in Configuration.h. It is currently "
                    f"bang-bang controlled."
                )
                e["status"].setStyleSheet(f"color:{_hex('yellow', '#f9e2af')};")
                continue
            for k, v in (("kp", pid.kp), ("ki", pid.ki), ("kd", pid.kd)):
                e[k].setText(f"{v:.2f}")
                e[k].setStyleSheet(
                    f"color:{_hex('text', '#cdd6f4')};font-weight:600;")
                e[f"{k}_spin"].setEnabled(True)
                if e[f"{k}_spin"].value() == 0.0:
                    e[f"{k}_spin"].setValue(v)
            e["apply"].setEnabled(True)
            e["status"].setText("")

    def _on_apply_pid(self, zone_id: str) -> None:
        e = self._pid_fields[zone_id]
        self.ctrl.set_pid(
            zone_id, e["kp_spin"].value(), e["ki_spin"].value(),
            e["kd_spin"].value(),
        )

    def _confirm(self, title: str, text: str) -> bool:
        return QMessageBox.question(
            self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) == QMessageBox.Yes

    def _on_save_eeprom(self) -> None:
        if self._confirm(
            "Save to EEPROM",
            "Write the board's current settings to EEPROM (M500)?\n\n"
            "This overwrites the stored values permanently.",
        ):
            self.ctrl.save_eeprom()

    def _on_load_eeprom(self) -> None:
        if self._confirm(
            "Reload EEPROM",
            "Reload settings from EEPROM (M501)?\n\n"
            "Any unsaved changes you have applied will be discarded.",
        ):
            self.ctrl.load_eeprom()

    def _on_factory_reset(self) -> None:
        if self._confirm(
            "Factory reset",
            "Load the firmware's compiled-in default settings (M502)?\n\n"
            "This discards your tuning. It affects RAM only until you also save "
            "to EEPROM.",
        ):
            self.ctrl.factory_reset()

    # ── autotune ────────────────────────────────────────────────────

    def _on_autotune_start(self) -> None:
        zone_id = self._at_zone.currentData()
        target = self._at_target.value()
        cycles = self._at_cycles.value()
        spec = next(z for z in ALL_ZONES if z.zone_id == zone_id)
        if not self._confirm(
            "Start PID autotune",
            f"Run PID autotune on {spec.title} at {target:g} °C for {cycles} "
            f"cycles?\n\nThe heater will be driven through repeated heat/cool "
            f"cycles. On a large water mass this can take a long time and may hit "
            f"Marlin's per-cycle time limit.",
        ):
            return
        self._at_result.clear()
        self._at_adopt.setEnabled(False)
        self.ctrl.start_autotune(
            zone_id, target, cycles, apply_result=self._at_apply.isChecked()
        )

    def _on_at_progress(self, zone_id, progress, elapsed) -> None:
        bits = []
        if progress.t_min is not None and progress.t_max is not None:
            bits.append(f"swing {progress.t_min:.2f}-{progress.t_max:.2f} °C")
        if progress.ku is not None:
            bits.append(f"Ku={progress.ku:.2f} Tu={progress.tu:.2f}")
        if progress.kp is not None:
            bits.append(
                f"Kp={progress.kp:.2f} Ki={progress.ki:.2f} Kd={progress.kd:.2f}")
        self._at_status.setText(
            f"Autotune running on {zone_id} — {format_duration(elapsed)} elapsed."
        )
        if bits:
            self._at_result.appendPlainText("   ".join(bits))

    def _on_at_done(self, zone_id, pid, err, hint) -> None:
        if pid is None:
            self._at_status.setText(f"Autotune failed: {err}")
            self._at_result.appendPlainText(f"\nFAILED: {err}\n{hint}")
            self._at_adopt.setEnabled(False)
            if hint:
                self._banner.show_message(f"Autotune failed: {err}. {hint}", "warn")
            return
        self._at_status.setText(
            f"Autotune finished: Kp={pid.kp:.2f} Ki={pid.ki:.2f} Kd={pid.kd:.2f}. "
            f"Save to EEPROM to keep it."
        )
        self._at_result.appendPlainText(
            f"\nRESULT   Kp={pid.kp:.2f}   Ki={pid.ki:.2f}   Kd={pid.kd:.2f}"
        )
        self._last_at = (zone_id, pid)
        self._at_adopt.setEnabled(True)

    def _on_adopt_result(self) -> None:
        if not self._last_at:
            return
        zone_id, pid = self._last_at
        e = self._pid_fields.get(zone_id)
        if e is None:
            return
        e["kp_spin"].setValue(pid.kp)
        e["ki_spin"].setValue(pid.ki)
        e["kd_spin"].setValue(pid.kd)
        self._status.setText(
            f"Copied the autotune result into the PID tab. Press \"Apply to board\" "
            f"to send it."
        )

    # ── misc ────────────────────────────────────────────────────────

    def _on_calibrate(self, zone_id: str) -> None:
        self.ctrl.calibrate_single_point(
            zone_id, self._cal_fields[zone_id]["ref"].value()
        )

    def _on_fault(self, latched) -> None:
        f = latched.fault
        self._banner.show_message(
            f"HEATER FAULT — {latched.summary}. {f.message}  {f.hint}  "
            f"The board has halted and must be POWER-CYCLED.",
            "error", dismissable=False,
        )
        QMessageBox.critical(
            self, "Heater fault",
            f"{f.message}\n\n{f.hint}\n\nThe board will ignore further commands "
            f"until it is power-cycled.",
        )

    def _on_divergence(self, ev) -> None:
        self._banner.show_message(f"SENSOR DIVERGENCE — {ev.message}", "warn")

    def _on_estop(self) -> None:
        if self._confirm(
            "Emergency stop",
            "Send M112?\n\nThis HALTS the board immediately. It will ignore all "
            "commands until it is power-cycled.\n\nFor a normal stop use "
            "\"ALL HEATERS OFF\" instead.",
        ):
            self.ctrl.emergency_stop()

    def _on_send_raw(self) -> None:
        cmd = self._raw_in.text().strip()
        if not cmd:
            return
        self.ctrl.send_raw(cmd)
        self._raw_in.clear()

    def _on_log_toggled(self, on: bool) -> None:
        if on:
            path = self.ctrl.start_logging("hold")
            self._log_btn.setText("Stop logging" if path else "Logging disabled")
        else:
            self.ctrl.stop_logging()
            self._log_btn.setText("Start logging")

    def _on_ramp(self, zone_id: str, st) -> None:
        card = self._cards.get(zone_id)
        if card is not None:
            card.show_ramp(st)

    def _on_status(self, text: str) -> None:
        self._status.setText(text)

    # ── shutdown ────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        """
        Offer to switch the heaters off on close.

        Worth being explicit: this is a courtesy, not a safety mechanism. The
        firmware holds its setpoint independently of us, so closing the window
        does NOT inherently stop heating.
        """
        if self.ctrl.connected:
            leave_on = QMessageBox.question(
                self, "Leave heaters running?",
                "Closing this window does NOT stop the board — Marlin keeps its own "
                "control loop running and will hold the current setpoint.\n\n"
                "Turn both heaters OFF before closing?\n\n"
                "Yes = turn heaters off (recommended)\n"
                "No  = leave them running for a long soak",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes,
            ) != QMessageBox.Yes
            self.ctrl.disconnect(heaters_off=not leave_on)
        self._tick.stop()
        super().closeEvent(event)
