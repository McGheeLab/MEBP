"""
incubator_widgets.py — presentation widgets for the Incubator page (v7.18).

Ported from the standalone tool's window (``tools/incubator/gui.py``) with the
app's own theme/scaling. Everything here is display-only: the widgets drive
the framework-agnostic ``SupportClasses.incubator.controller`` facade and
render what its callbacks publish.

Threading: controller callbacks arrive on background threads.
:class:`HeaterBridge` re-emits them as Qt signals and PySide6's automatic
queued connections marshal delivery onto the GUI thread, so nothing here
needs a lock.

Glyph policy: labels use plain words and ASCII rather than decorative symbols.
Characters like U+27F3 are missing from Segoe UI and render as tofu boxes.
Only degree/plus-minus signs are used, which every Windows UI font has.
"""

from __future__ import annotations

import time
from collections import deque

from PySide6.QtCore import QObject, QPointF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFrame, QGroupBox, QHBoxLayout, QLabel,
    QMessageBox, QProgressBar, QPushButton, QSizePolicy, QSpinBox,
    QVBoxLayout, QWidget,
)

from gui.scaling import s, sf, scaled_font_size
from gui.styles import COLORS

from SupportClasses.incubator.safety import CAUTION_SETPOINT_C
from SupportClasses.incubator.stability import format_duration

ZONE_COLORS = {"bed": "blue", "hotend": "peach"}


def _c(key: str, fallback: str = "#cdd6f4") -> QColor:
    return QColor(COLORS.get(key, fallback))


def _hex(key: str, fallback: str = "#cdd6f4") -> str:
    return COLORS.get(key, fallback)


def _first_available(prefs: tuple[str, ...], fallback: str) -> str:
    """First installed font family from ``prefs`` (else ``fallback``).

    An empty ``QFont("")`` lets Qt substitute whatever it likes, and the
    substitute can lack glyphs such as the em dash — which then renders as a
    stray bar inside the plot legend. Resolve explicitly instead.
    """
    try:
        families = set(QFontDatabase.families())
    except Exception:
        return fallback
    for name in prefs:
        if name in families:
            return name
    return fallback


def ui_font_family() -> str:
    return _first_available(
        ("Segoe UI", "Noto Sans", "DejaVu Sans", "Arial"), "Segoe UI")


def mono_font(point_size: int | None = None) -> QFont:
    f = QFont(_first_available(
        ("Cascadia Mono", "Consolas", "Courier New"), "Consolas"))
    if point_size:
        f.setPointSize(point_size)
    return f


def _plot_font() -> QFont:
    f = QFont(ui_font_family())
    f.setPointSize(max(7, int(sf(8))))
    return f


# ═══════════════════════════════════════════════════════════════════
# Thread bridge
# ═══════════════════════════════════════════════════════════════════

class HeaterBridge(QObject):
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
    staircase_row = Signal(object)          # one sampled row, ~1 Hz
    staircase_done = Signal(str, object)    # zone_id, StaircaseOutcome

    def attach(self, ctrl) -> None:
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
        # Round 6: these fire from the staircase's OWN thread, so they must
        # cross into Qt the same way everything else here does.
        if hasattr(ctrl, "on_staircase_row"):
            ctrl.on_staircase_row(self.staircase_row.emit)
            ctrl.on_staircase_done(self.staircase_done.emit)


# ═══════════════════════════════════════════════════════════════════
# Small presentation widgets
# ═══════════════════════════════════════════════════════════════════

class StatePill(QLabel):
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


class Banner(QFrame):
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


class DutyBar(QWidget):
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

class TempTrendPlot(QWidget):
    """
    Rolling temperature trend with a real °C axis.

    Follows ``gui/pages/print_results.py::ErrorTimeSeriesWidget`` for the
    labelled axis and gridlines. Per zone: actual (solid), target (dashed
    step), duty (faint, on its own 0-100% scale along the bottom).
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(230))
        # A floor on BOTH axes: the plot now lives between splitter handles
        # (the page's upper area), and the axis labels/gridlines stop being
        # readable well before a dragged pane reaches zero.
        self.setMinimumWidth(s(300))
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
        # Always spanning the full window would squash a fresh run into a
        # spike at the right-hand edge.
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
                    p.drawLine(QPointF(prev.x(), prev.y()),
                               QPointF(pt.x(), prev.y()))
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

class ZoneCard(QGroupBox):
    """Readout + setpoint controls for one zone."""

    def __init__(self, spec, ctrl, parent: QWidget | None = None, *,
                 title: str | None = None, ramp_step_getter=None):
        super().__init__(title or spec.title, parent)
        self._spec = spec
        self._ctrl = ctrl
        #: () -> float | None — the Hardware Setup tab's ramp-step preference.
        self._ramp_step_getter = ramp_step_getter

        lay = QVBoxLayout(self)
        lay.setContentsMargins(s(12), s(14), s(12), s(12))
        lay.setSpacing(s(7))

        # ── header: state pill + wiring reminder ──
        top = QHBoxLayout()
        self._pill = StatePill("OFFLINE", "overlay0")
        top.addWidget(self._pill)
        top.addStretch(1)
        wiring = QLabel(f"{spec.heater_connector} / {spec.sensor_connector}")
        wiring.setToolTip(
            f"Which physical board ports this zone drives:\n"
            f"heater output {spec.heater_connector} · "
            f"thermistor {spec.sensor_connector}.\n"
            f"If your heater is plugged into the OTHER output, use the "
            f"other zone's card — the commands differ "
            f"({spec.set_cmd} vs the sibling zone's)."
        )
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

        self._duty = DutyBar()
        lay.addWidget(self._duty)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color:{_hex('surface1', '#45475a')};")
        lay.addWidget(line)

        # ── setpoint ──
        row = QHBoxLayout()
        row.addWidget(QLabel("Setpoint"))
        self._sp = QDoubleSpinBox()
        self._sp.setRange(0.0, float(ctrl.MAX_SETPOINT_C))
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

        # v7.18: a firmware-REFUSED command latches on the zone runtime and is
        # shown HERE, persistently — a transient status-bar line was how
        # "M104 refused for 48 minutes" stayed invisible on the bench.
        self._refused_lbl = QLabel("")
        self._refused_lbl.setWordWrap(True)
        self._refused_lbl.setVisible(False)
        self._refused_lbl.setStyleSheet(
            f"background:{_hex('surface0', '#313244')};"
            f"color:{_hex('red', '#f38ba8')};"
            f"border-left:{s(3)}px solid {_hex('red', '#f38ba8')};"
            f"border-radius:{s(3)}px;padding:{s(5)}px;font-size:{sf(8.5)}pt;"
        )
        lay.addWidget(self._refused_lbl)

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
        self._rescan_btn = QPushButton("Re-check this sensor")
        self._rescan_btn.setToolTip(
            "Re-read the board's sensors (one M105) and re-evaluate this zone.\n"
            "Use after plugging a thermistor in: Marlin reports it immediately, "
            "but the verdict is cached from connect time.\n"
            "Touches no heater and changes no setpoint."
        )
        self._rescan_btn.setVisible(False)
        self._rescan_btn.clicked.connect(self._ctrl.rescan_sensors)
        lay.addWidget(self._rescan_btn)

        self._sp.valueChanged.connect(self._refresh_plan)
        lay.addStretch(1)

    # ── knobs from the Hardware Setup tab ──────────────────────────

    def set_fine_period(self, seconds: int) -> None:
        try:
            self._fine_period.setValue(int(seconds))
        except Exception:
            pass

    def set_preset_value(self, celsius: float) -> None:
        try:
            self._sp.setValue(float(celsius))
        except Exception:
            pass

    def refresh_ceiling(self) -> None:
        """Re-read the controller's (possibly store-lowered) ceiling."""
        self._sp.setRange(0.0, float(self._ctrl.MAX_SETPOINT_C))

    # ── actions ─────────────────────────────────────────────────────

    def _apply_preset(self, value: float) -> None:
        self._sp.setValue(value)
        self._on_set()

    def _ask_watchdog(self, plan: dict, chk) -> str:
        """Offer the Ramp instead of a setpoint that will arm the watchdog.

        Returns "ramp", "set" or "cancel". Separated from :meth:`_on_set` so
        it is one overridable seam rather than a bare modal: offscreen a
        modal blocks forever, so a hidden one turns every future test that
        presses Set into a hang.
        """
        gap = plan.get("watchdog_arm_gap_c", 6.0)
        cur = plan.get("current_c")
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Heat-up watchdog")
        box.setText(
            f"Setting {self.title()} straight to {chk.allowed_c:g} °C"
            + (f" from {cur:.1f} °C" if cur is not None else "")
            + " will arm Marlin's heat-up watchdog."
        )
        box.setInformativeText(
            "The firmware then demands a fast rise, and halts the board with "
            "\"Heating Failed\" if this heater cannot deliver it. On the "
            "shared link that stops Z and the pumps too."
            + chr(10) + chr(10) +
            f"The Ramp walks the setpoint up in steps smaller than "
            f"{gap:.0f} °C, so the watchdog never arms — the heater still "
            f"runs at full power throughout."
        )
        use_ramp = box.addButton("Use Ramp", QMessageBox.AcceptRole)
        set_anyway = box.addButton("Set anyway", QMessageBox.DestructiveRole)
        box.addButton(QMessageBox.Cancel)
        box.setDefaultButton(use_ramp)
        box.exec()
        clicked = box.clickedButton()
        if clicked is use_ramp:
            return "ramp"
        if clicked is set_anyway:
            return "set"
        return "cancel"

    def _on_set(self) -> None:
        if not self._ctrl.connected:
            QMessageBox.information(self, "Not connected",
                                    "Connect to the board first.")
            return
        plan = self._ctrl.preview_setpoint(self._spec.zone_id, self._sp.value())
        chk = plan["check"]

        # v7.18 round 6c: a direct setpoint far above the current temperature
        # ARMS Marlin's heat-up watchdog, and on a slow thermal load that ends
        # in "Heating Failed" -> kill(): the board halts and, on the shared
        # link, takes Z and the pumps with it. Bench 2026-08-17: M140 S37 from
        # ~21 C killed the board at 60 s on BOTH boards. The Ramp exists for
        # exactly this, so offer it rather than either silently converting the
        # press (that would change what the button means) or letting a known
        # board-killer through unremarked.
        if plan.get("watchdog_risk"):
            choice = self._ask_watchdog(plan, chk)
            if choice == "ramp":
                self._on_ramp_clicked()
                return
            if choice != "set":
                return

        if chk.clamped or chk.needs_confirm:
            msg = [f"Set {self.title()} to {chk.allowed_c:g} °C?"]
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

    def _ramp_step(self) -> float | None:
        if self._ramp_step_getter is None:
            return None
        try:
            return self._ramp_step_getter()
        except Exception:
            return None

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
            msg = [f"Ramp {self.title()} up to {chk.allowed_c:g} °C?"]
            if chk.reason:
                msg.append(f"\n\nThis value was {chk.reason}.")
            if QMessageBox.question(
                self, "Confirm ramp", "".join(msg),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            ) != QMessageBox.Yes:
                return
        self._ctrl.start_ramp(zid, chk.allowed_c, step_c=self._ramp_step())

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
            plan = self._ctrl.preview_setpoint(self._spec.zone_id,
                                               self._sp.value())
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

        refused = str(getattr(runtime, "refused", "") or "")
        if refused:
            self._refused_lbl.setText(
                f"Board REFUSED {refused}. This zone is NOT heating and the "
                f"automatic hold was stopped. Check that this zone's heater "
                f"is really wired to {self._spec.heater_connector} before "
                f"trying again."
            )
            self._refused_lbl.setVisible(True)
        else:
            self._refused_lbl.setVisible(False)

        if not runtime.sensor_ok:
            self._pill.set_state("SENSOR FAULT", "red")
            self._temp.setStyleSheet(f"color:{_hex('red', '#f38ba8')};")
        elif refused:
            self._pill.set_state("REFUSED", "red")
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
                self._sub.setText("raw sensor reading, uncalibrated")

        if report.target_c:
            extra = ""
            if runtime.dither_enabled:
                extra = f" (dithering, board at {runtime.commanded_c} °C)"
            elif runtime.commanded_c:
                extra = f" (board target {runtime.commanded_c} °C)"
            # v7.18: a board that keeps forgetting its setpoint is a fault in
            # its own right (it is rebooting under the hold), so the count is
            # shown rather than silently patched over by the keeper.
            reasserts = int(getattr(runtime, "reasserts", 0) or 0)
            if reasserts:
                extra += (f"  ·  board forgot the setpoint x{reasserts}, "
                          f"restored")
            self._target_lbl.setText(
                f"Target {report.target_c:.2f} °C{extra}   "
                f"error {report.error_c:+.2f} °C"
            )
            self._target_lbl.setToolTip(
                "A Marlin reset clears every heater target. Re-opening the "
                "board's serial port resets it, so this happens on every ZP "
                "reconnect. The host puts the setpoint back; a rising count "
                "means the board is rebooting repeatedly."
                if reasserts else "")
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
                f"controlled until TEMP_SENSOR is configured — see the Firmware "
                f"tab. Note a configured-but-unplugged sensor still shows up here "
                f"as an open circuit, so a missing field points at the firmware "
                f"build, not the wiring.",
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
