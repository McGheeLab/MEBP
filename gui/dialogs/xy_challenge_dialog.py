"""xy_challenge_dialog.py — XY Printing Challenge (v7.5.x).

A path-following bench for the three print motion modes. It drives a challenge
shape (needle retracted the whole time), records the stage's ACTUAL path from the
encoder, overlays it on the IDEAL path, and reports the deviation (RMS + max µm)
— so you can objectively COMPARE the modes and TUNE each mode's parameters until
the actual path hugs the ideal (to within the resolution element).

Modes (mirroring PrintManager's print-path executors, XY-only here — the velocity
+ confirm drivers reuse the SAME SupportClasses.VelocityControl law the print
uses, so tuning transfers exactly):
  • open_loop — stream a continuous velocity vector along the path tangent
                (feed-forward, no position feedback; pace trims the speed).
  • confirm   — move to each point, wait for arrival ONLY at corners (straight
                edges stream), then drain.
  • velocity  — closed-loop pure-pursuit + corner-aware speed scheduling +
                cross-track PID.

Overlay colours: ideal = grey, newest attempt = red, best-so-far = green, all
other (worse) attempts = low-alpha blue.

Auto-tune: a coordinate-descent over each mode's key parameters, plus a dedicated
Ziegler–Nichols relay tuner for the velocity PID gains. Tuned parameters persist
to PrintTimingCalibrationStore and are read by the real print (Quick Print stamps
them onto PrintSettings).

Robustness panel: after tuning, sweep a small matrix of shapes × sizes × speeds
and PASS/FAIL each vs the resolution element.

Safety: needle retracted to Safe Z the whole time; poller + watchdog suspended;
the stage is always stopped (VS 0,0) on exit; a big sustained cross-track error
trips a soft runaway that stops + reports (no crash-follow).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QDoubleSpinBox, QWidget, QPlainTextEdit, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from SupportClasses import XYChallenge as XC
from SupportClasses import VelocityControl as VC
from SupportClasses.PrintTimingCalibrationStore import get_store

try:
    from SupportClasses.MotionController import compute_zn_pid_gains
except Exception:  # pragma: no cover
    compute_zn_pid_gains = None

logger = logging.getLogger(__name__)

MODES = [("Open-loop velocity", "open_loop"),
         ("Confirmed per-segment", "confirm"),
         ("Velocity (closed-loop)", "velocity")]


# ── ONE declarative parameter spec ────────────────────────────────────
#
# Every tunable used to be described in SEVEN parallel places (MODE_PARAMS, the
# spin constructors, the `rows` layout list, _PARAM_KEY, _mode_params,
# _apply_tuned, and _load/_save_params_to_store). Adding a knob meant editing
# all seven and any omission failed silently. They are now GENERATED from this
# single list, so a new knob is one row here.

@dataclass(frozen=True)
class ParamSpec:
    """One tunable: its widget, its UI placement, its store binding, its grid."""
    key: str                    # internal _mode_params key
    attr: str                   # dialog attribute holding the spin
    label: str                  # UI label
    lo: float
    hi: float
    default: float
    decimals: int
    step: float
    suffix: str
    tip: str
    modes: tuple                # UI modes that show this row
    store_key: str = ""         # PATH_TUNING_DEFAULTS key ("" = not persisted)
    store_modes: tuple = ()     # store buckets it persists into
    grid: tuple = ()            # coordinate-descent grid (() = not swept)
    grid_modes: tuple = ()      # modes that SWEEP it (default: store_modes)
    kind: str = "mode"          # "mode" | "resolution" | "ui"

    def swept_by(self, mode) -> bool:
        return bool(self.grid) and mode in (self.grid_modes or self.store_modes)


_ALL_MODES = ("open_loop", "confirm", "velocity")

# NOTE on `corner_angle`: it persists into BOTH the confirm and velocity buckets
# from a single widget. That is a real defect (tuning it under `confirm` silently
# rewrites the velocity value) but it is TODAY's behaviour, and this refactor is
# deliberately behaviour-preserving. It is fixed by the per-mode params model.
PARAM_SPECS = (
    ParamSpec("speed", "_speed_spin", "Speed:", 0.1, 50.0, 5.0, 2, 0.5, " mm/s",
              "Path speed for the run (all modes).",
              _ALL_MODES, kind="ui"),
    ParamSpec("resolution_um", "_res_spin", "Resolution:",
              1.0, 500.0, 30.0, 0, 1.0, " µm",
              "Resolution element — smallest feature / acceptable path-deviation "
              "tolerance. Drives robustness PASS/FAIL.",
              _ALL_MODES, kind="resolution"),
    ParamSpec("pace", "_pace_spin", "Pace ×:", 1.0, 4.0, 1.0, 2, 0.05, " ×",
              "[open-loop] speed trim — divides the commanded velocity to match "
              "a slow stage.",
              ("open_loop",), store_key="pace_correction",
              store_modes=("open_loop",),
              grid=(1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0)),
    ParamSpec("tol_um", "_tol_spin", "Confirm tol:",
              2.0, 200.0, 40.0, 0, 5.0, " µm",
              "[confirm] arrival tolerance at a corner.",
              ("confirm",), store_key="settle_tol_um", store_modes=("confirm",),
              grid=(10.0, 20.0, 30.0, 45.0, 60.0, 90.0)),
    ParamSpec("lookahead", "_look_spin", "Lookahead:",
              0.1, 3.0, 0.6, 2, 0.05, " mm",
              "[velocity] pure-pursuit lookahead (smaller = tighter corners, "
              "more jitter).",
              ("velocity",), store_key="lookahead_mm",
              store_modes=("velocity",),
              grid=(0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.3)),
    ParamSpec("control_hz", "_hz_spin", "Ctrl rate:",
              5.0, 60.0, 25.0, 0, 1.0, " Hz",
              "[open-loop/velocity] control-loop rate.",
              ("open_loop", "velocity"), store_key="control_hz",
              store_modes=("velocity",)),
    ParamSpec("decel", "_decel_spin", "Decel:", 0.2, 6.0, 1.5, 1, 0.1, " mm",
              "[open-loop/velocity] end-of-path ramp-down.",
              ("open_loop", "velocity"), store_key="decel_mm",
              store_modes=("velocity",)),
    ParamSpec("corner_angle", "_corner_angle_spin", "Corner °:",
              5.0, 90.0, 30.0, 0, 5.0, " °",
              "Turn angle counted as a corner (slow into / stop at).",
              ("confirm", "velocity"), store_key="corner_angle_deg",
              store_modes=("confirm", "velocity"),
              grid=(15.0, 25.0, 35.0, 50.0),
              # Historically swept ONLY under `confirm`, even though the widget
              # writes both buckets. Preserved verbatim.
              grid_modes=("confirm",)),
    ParamSpec("corner_factor", "_corner_fac_spin", "Corner speed:",
              0.05, 1.0, 0.4, 2, 0.05, " ×",
              "[velocity] fraction of speed allowed at the sharpest corner.",
              ("velocity",), store_key="corner_speed_factor",
              store_modes=("velocity",),
              grid=(0.2, 0.3, 0.4, 0.55, 0.7)),
    ParamSpec("kp", "_kp_spin", "PID Kp:", 0.0, 20.0, 0.0, 2, 0.1, "",
              "[velocity] cross-track PID proportional gain (0 = pure pursuit). "
              "The stability limit is π/(2L) for the machine's dead time L — see "
              "the “PID from dead time” button, which computes it exactly.",
              ("velocity",), store_key="pid_kp", store_modes=("velocity",),
              # The old grid topped out at 4.0, but the correct gain for ME3B V1
              # is ≈5.24 — the descent literally could not reach the right answer.
              # Extended to straddle it.
              grid=(0.0, 1.0, 2.0, 3.5, 5.0, 7.0, 9.0)),
    ParamSpec("kd", "_kd_spin", "PID Kd:", 0.0, 2.0, 0.0, 3, 0.01, "",
              "[velocity] cross-track PID derivative gain.",
              ("velocity",), store_key="pid_kd", store_modes=("velocity",),
              grid=(0.0, 0.02, 0.05, 0.1)),
)

_SPEC_BY_KEY = {p.key: p for p in PARAM_SPECS}
_SPEC_BY_STORE_KEY = {p.store_key: p for p in PARAM_SPECS if p.store_key}

# Which param rows each mode shows (internal param keys) — generated.
MODE_PARAMS = {m: [p.key for p in PARAM_SPECS if m in p.modes]
               for m in _ALL_MODES}

# Coordinate-descent: ordered (store-param, grid) list swept per mode.
# Order follows PARAM_SPECS declaration order, which reproduces the historical
# per-mode sweep order.
AUTOTUNE = {
    m: [(p.store_key, list(p.grid)) for p in PARAM_SPECS if p.swept_by(m)]
    for m in _ALL_MODES
}

# store-param name → the _mode_params dict key — generated.
_PARAM_KEY = {p.store_key: p.key for p in PARAM_SPECS if p.store_key}


class _Bridge(QObject):
    status = Signal(str)
    log = Signal(str)
    result = Signal(dict)       # {mode,label,ideal,actual,rms_um,max_um,n,wall_s}
    tune = Signal(dict)         # {param,value,rms_um,best}
    apply = Signal(str, float)  # (store_param, value) → reflect into spin
    finished = Signal(bool, str)


class _PathOverlay(QWidget):
    """Draws the ideal path (grey) + every actual attempt, recoloured by role:
    newest = red, best-so-far (lowest RMS) = green, all others = low-alpha blue.
    Auto-fit to the data."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(s(360), s(300))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._ideal = []
        self._actuals = []        # list of (pts, rms)
        self._title = "Draw a challenge to begin."

    def clear(self):
        self._ideal = []
        self._actuals = []
        self._title = ""
        self.update()

    def set_ideal(self, ideal):
        self._ideal = list(ideal or [])
        self.update()

    def add_actual(self, pts, rms):
        self._actuals.append((list(pts or []), float(rms)))
        self.update()

    def set_title(self, t):
        self._title = t
        self.update()

    def _bounds(self):
        xs, ys = [], []
        for p in self._ideal:
            xs.append(p[0]); ys.append(p[1])
        for pts, _r in self._actuals:
            for p in pts:
                xs.append(p[0]); ys.append(p[1])
        if not xs:
            return None
        return (min(xs), min(ys), max(xs), max(ys))

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(COLORS["mantle"]))
        b = self._bounds()
        pad = s(24)
        w = self.width() - 2 * pad
        h = self.height() - 2 * pad
        if b and w > 10 and h > 10:
            minx, miny, maxx, maxy = b
            spanx = max(maxx - minx, 1e-6)
            spany = max(maxy - miny, 1e-6)
            sc = min(w / spanx, h / spany)
            ox = pad + (w - spanx * sc) / 2
            oy = pad + (h - spany * sc) / 2

            def to_px(pt):
                return (ox + (pt[0] - minx) * sc, oy + (maxy - pt[1]) * sc)

            def draw(pts, color, width):
                if len(pts) < 2:
                    return
                pen = QPen(color)
                pen.setWidthF(width)
                pen.setCapStyle(Qt.RoundCap)
                pen.setJoinStyle(Qt.RoundJoin)
                p.setPen(pen)
                prev = to_px(pts[0])
                for q in pts[1:]:
                    cur = to_px(q)
                    p.drawLine(int(prev[0]), int(prev[1]),
                               int(cur[0]), int(cur[1]))
                    prev = cur

            draw(self._ideal, QColor(COLORS["overlay0"]), s(1.5))

            n = len(self._actuals)
            if n:
                best_i = min(range(n), key=lambda i: self._actuals[i][1])
                newest_i = n - 1
                blue = QColor(COLORS.get("blue", "#89b4fa")); blue.setAlpha(55)
                green = QColor(COLORS.get("green", "#a6e3a1"))
                red = QColor(COLORS.get("red", "#f38ba8"))
                # bad attempts (faint blue) first, then best (green), newest (red)
                for i, (pts, _r) in enumerate(self._actuals):
                    if i in (best_i, newest_i):
                        continue
                    draw(pts, blue, s(1.4))
                if best_i != newest_i:
                    draw(self._actuals[best_i][0], green, s(2.2))
                draw(self._actuals[newest_i][0], red, s(2.2))
        p.setPen(QColor(COLORS["text"]))
        p.drawText(pad, s(16), self._title or "")


class XYChallengeDialog(QDialog):
    _VEL_ARRIVE_MM = 0.05
    _VEL_RUNAWAY_MM = 3.0

    def __init__(self, controller, *, safe_z, start_xy_mm, parent=None):
        super().__init__(parent)
        self.setWindowTitle("XY Printing Challenge")
        self.setModal(False)
        self._ctrl = controller
        self._safe_z = safe_z
        self._start = start_xy_mm            # (x, y) zero-ref mm
        self._store = get_store()
        self._bridge = _Bridge()
        self._thread = None
        self._stop = threading.Event()
        self._param_rows = {}                # name → (row widget)
        self._build()
        self._load_params_from_store()
        self._refresh_param_visibility()
        self.refresh_budget()
        self._bridge.status.connect(self._status.setText)
        self._bridge.log.connect(self._append_log)
        self._bridge.result.connect(self._on_result)
        self._bridge.tune.connect(self._on_tune)
        self._bridge.apply.connect(self._apply_tuned)
        self._bridge.finished.connect(self._on_finished)

    # ── UI ────────────────────────────────────────────────────────
    def _dspin(self, lo, hi, val, dec, step, suffix, tip):
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setDecimals(dec)
        w.setSingleStep(step)
        w.setValue(val)
        if suffix:
            w.setSuffix(suffix)
        w.setToolTip(tip)
        w.setMinimumWidth(s(90))
        return w

    def _param_row(self, name, label, spin):
        """Wrap (label, spin) in a row widget so both hide together per mode."""
        row = QWidget()
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(6))
        lay.addWidget(QLabel(label))
        lay.addWidget(spin)
        self._param_rows[name] = row
        return row

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(s(12), s(12), s(12), s(12))
        root.setSpacing(s(8))

        # Row: shape / size / step / mode
        top = QGridLayout()
        top.setHorizontalSpacing(s(10))
        top.setVerticalSpacing(s(6))
        self._shape_combo = QComboBox()
        for name in XC.CHALLENGE_SHAPES:
            self._shape_combo.addItem(name, name)
        self._shape_combo.currentIndexChanged.connect(self._refresh_hint)
        self._size_spin = self._dspin(1.0, 40.0, 10.0, 1, 1.0, " mm",
                                      "Overall shape size (bounding extent).")
        self._step_spin = self._dspin(0.1, 5.0, 0.5, 2, 0.1, " mm",
                                      "Segment length the path is sampled at "
                                      "(matches a real toolpath's fineness).")
        self._mode_combo = QComboBox()
        for label, key in MODES:
            self._mode_combo.addItem(label, key)
        self._mode_combo.setCurrentIndex(2)     # velocity
        self._mode_combo.currentIndexChanged.connect(self._refresh_param_visibility)
        top.addWidget(QLabel("Shape:"), 0, 0)
        top.addWidget(self._shape_combo, 0, 1)
        top.addWidget(QLabel("Size:"), 0, 2)
        top.addWidget(self._size_spin, 0, 3)
        top.addWidget(QLabel("Segment:"), 0, 4)
        top.addWidget(self._step_spin, 0, 5)
        top.addWidget(QLabel("Mode:"), 1, 0)
        top.addWidget(self._mode_combo, 1, 1)
        root.addLayout(top)
        self._hint = QLabel("")
        self._hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self._hint.setWordWrap(True)
        root.addWidget(self._hint)

        # Per-mode params — widgets, layout and rows all GENERATED from
        # PARAM_SPECS so there is exactly one place to add a knob.
        pg = QGridLayout()
        pg.setHorizontalSpacing(s(8))
        pg.setVerticalSpacing(s(4))
        for i, spec in enumerate(PARAM_SPECS):
            spin = self._dspin(spec.lo, spec.hi, spec.default, spec.decimals,
                               spec.step, spec.suffix, spec.tip)
            setattr(self, spec.attr, spin)
            pg.addWidget(self._param_row(spec.key, spec.label, spin),
                         i // 3, i % 3)
        root.addLayout(pg)

        # ── Speed budget: WHY the print runs at the speed it runs at ──
        # The single most-asked question this dialog could not answer. The
        # follower's speed is min(print_speed, lookahead/((loop+dead)·safety),
        # top_speed·frac), and on ME3B V1 the middle term wins at 0.31 mm/s
        # because a one-sample by_phase intercept inflates the dead time. This
        # line spells out the arithmetic and names the binding term.
        self._budget = QLabel("")
        self._budget.setWordWrap(True)
        self._budget.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        root.addWidget(self._budget)

        # Overlay + log
        mid = QHBoxLayout()
        self._overlay = _PathOverlay()
        mid.addWidget(self._overlay, stretch=3)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumWidth(s(240))
        self._log.setStyleSheet(
            f"font-family: monospace; font-size: {sf(9)}pt;")
        mid.addWidget(self._log, stretch=2)
        root.addLayout(mid, stretch=1)

        # Buttons
        #
        # v7.21.2: this dialog is now a BENCH DIAGNOSTIC, not a tuning surface.
        # Calibrate / Compare / Auto-tune (descent) / Auto-tune PID (ZN) /
        # Robustness / PID from dead time / Measure dead time / Save tuned params
        # are gone: every one of them measured or wrote part of the calibration on
        # its own, in an order that mattered and was not enforced, and the
        # coordinate descent additionally optimised `path_error` — a one-sided,
        # untimed metric its own docstring forbids as an auto-tune objective, under
        # which a crawling run scores best. That is why the stored tuning ended up
        # at the minimum of BOTH its grids. All of it now lives in one place:
        # Workflows -> XY<->ZP Timing Calibration -> Run XY Calibration.
        #
        # `Save tuned params` is deliberately gone too: a bench dialog that can
        # hand-overwrite the calibrated tuning is exactly how two homes for these
        # numbers drifted apart.
        row = QHBoxLayout()
        self._run_btn = QPushButton("Run")
        self._run_btn.setToolTip("Drive the shape in the selected mode; overlay "
                                 "actual vs ideal + report deviation.")
        self._run_btn.clicked.connect(lambda: self._launch("run"))
        self._geometry_btn = QPushButton("📐 Geometry panel…")
        self._geometry_btn.setToolTip(
            "Ideal vs actual per shape × size — simulated from the measured "
            "stage characteristics (instant, no motion) and/or driven on the "
            "stage. Shows where geometry itself defeats the current tuning.")
        self._geometry_btn.clicked.connect(self._open_geometry_panel)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(lambda: self._stop.set())
        for b in (self._run_btn, self._geometry_btn, self._stop_btn):
            row.addWidget(b)
        row.addStretch(1)
        root.addLayout(row)

        self._status = QLabel("")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        root.addWidget(self._status)
        self.resize(s(940), s(720))
        self._refresh_hint()
        # Keep the speed-budget line live as the operator changes the inputs it
        # depends on.
        for _sp in (self._speed_spin, self._look_spin, self._hz_spin,
                    self._corner_fac_spin):
            try:
                _sp.valueChanged.connect(lambda *_: self.refresh_budget())
            except Exception:
                pass

    def _refresh_hint(self):
        name = self._shape_combo.currentData()
        self._hint.setText(XC.SHAPE_HINTS.get(name, ""))

    def _refresh_param_visibility(self, *_):
        mode = self._mode_combo.currentData()
        shown = set(MODE_PARAMS.get(mode, []))
        for name, row in self._param_rows.items():
            row.setVisible(name in shown)
        # v7.21.2: the ZN + robustness buttons are gone (their work moved into
        # the one XY calibration). Referencing them here would break dialog
        # CONSTRUCTION, since this runs from __init__ as well as the mode combo.

    def _append_log(self, msg):
        self._log.appendPlainText(msg)

    # ── speed budget readout ──────────────────────────────────────
    def refresh_budget(self):
        """Explain the commanded speed: the arithmetic, the binding term, and —
        when it is the dead time — WHICH stored measurement is responsible.

        Also warns when the corner limits all sit above the cap, because in that
        state ``min(cap, corner_limit)`` is always the cap and tuning
        ``corner_speed_factor`` provably does nothing (the operator swept it and
        correctly observed a completely flat result)."""
        lbl = getattr(self, "_budget", None)
        if lbl is None:
            return
        try:
            speed = float(self._speed_spin.value())
            look = float(self._look_spin.value())
            res = self._resolve(speed, look, max(5.0, self._hz_spin.value()))
            loop_ms = float(self._store.get_control_loop_ms() or 0.0)
            dead_total = float(res.get("dead_time_s", 0.0))
            lag = max(0.0, dead_total - loop_ms / 1000.0)
            _dt, src = self._store.effective_dead_time_s()
            cap = res["speed_cap_mm_s"]
            top = res["max_um_s"] / 1000.0
            reason = res.get("cap_reason", "")

            src_txt = {"measured": "measured step response",
                       "phase_lag": "by_phase settle mean ⚠",
                       "unmeasured": "unmeasured"}.get(src, src)
            parts = [
                f"Speed budget: commanded {cap:.2f} mm/s "
                f"(asked {speed:.2f}) — limited by <b>{reason or 'n/a'}</b>.",
                f"cap = lookahead {res.get('lookahead_mm', look):.2f} mm / "
                f"((loop {loop_ms / 1000.0:.3f} s + dead {lag:.3f} s) × safety) "
                f"· dead time from {src_txt} · stage top speed {top:.2f} mm/s.",
            ]
            if src == "phase_lag":
                parts.append(
                    "⚠ The dead time is the mean of the by_phase <i>settle</i> "
                    "intercepts, not a transport delay — press "
                    "“Measure dead time” for the real number.")
            # Is corner tuning capable of doing anything at this cap?
            csf = float(self._corner_fac_spin.value())
            worst_corner = speed * csf            # a 180° reversal
            if worst_corner > cap:
                parts.append(
                    f"⚠ Corner tuning is INERT here: the sharpest corner limit "
                    f"is {worst_corner:.2f} mm/s, above the {cap:.2f} mm/s cap, "
                    f"so min(cap, corner) is always the cap. Raise the cap "
                    f"(measure the dead time / enable hold-speed) first.")
            lbl.setText("<br>".join(parts))
        except Exception:
            lbl.setText("")

    # ── params ↔ store (all four generated from PARAM_SPECS) ──────
    def _spin_for(self, spec):
        return getattr(self, spec.attr, None)

    def _load_params_from_store(self):
        cache = {m: self._store.get_mode_params(m) for m in _ALL_MODES}
        for spec in PARAM_SPECS:
            spin = self._spin_for(spec)
            if spin is None:
                continue
            if spec.kind == "resolution":
                try:
                    spin.setValue(self._store.get_resolution_element_um())
                except Exception:
                    pass
                continue
            if not spec.store_key or not spec.store_modes:
                continue
            # Read from the LAST bucket it persists into, matching the historical
            # precedence (corner_angle read from `velocity`, not `confirm`).
            bucket = cache.get(spec.store_modes[-1], {})
            spin.setValue(bucket.get(spec.store_key, spec.default))

    def _save_params_to_store(self):
        updates = {m: {} for m in _ALL_MODES}
        for spec in PARAM_SPECS:
            spin = self._spin_for(spec)
            if spin is None:
                continue
            if spec.kind == "resolution":
                try:
                    self._store.set_resolution_element_um(spin.value())
                except Exception:
                    pass
                continue
            if not spec.store_key:
                continue
            for m in spec.store_modes:
                updates[m][spec.store_key] = spin.value()
        for m, vals in updates.items():
            if vals:
                self._store.set_mode_params(m, vals)
        self._status.setText("Saved tuned params — the real print will use them.")

    def _mode_params(self, mode):
        out = {}
        for spec in PARAM_SPECS:
            spin = self._spin_for(spec)
            if spin is not None:
                out[spec.key] = spin.value()
        return out

    def _apply_tuned(self, store_param, val):
        """Reflect a tuned value into its spin (GUI thread via the apply signal)."""
        try:
            spec = _SPEC_BY_STORE_KEY[store_param]
            self._spin_for(spec).setValue(val)
        except Exception:
            pass

    # ── launch / gate ─────────────────────────────────────────────
    def _busy(self):
        return self._thread is not None and self._thread.is_alive()

    def _launch(self, action):
        if self._busy():
            return
        # "PID from dead time" is pure arithmetic over stored calibration — no
        # stage motion at all — so it must not be gated on a connected stage.
        if action == "pid_calc":
            self._stop.clear()
            self._set_running(True)
            self._thread = threading.Thread(
                target=self._worker_pid_analytic, name="XYChallenge",
                daemon=True)
            self._thread.start()
            return
        if not getattr(self._ctrl, "is_xy_connected", False):
            self._status.setText("Connect the XY stage first.")
            return
        if self._safe_z is None and getattr(self._ctrl, "is_zp_connected", False):
            self._status.setText("Set Safe Z first (needle stays retracted).")
            return
        if self._start is None:
            self._status.setText("No start location.")
            return
        self._stop.clear()
        self._log.clear()
        self._overlay.clear()
        self._set_running(True)
        # v7.21.2: only the manual bench run survives here.
        target = {"run": self._worker_run}[action]
        self._thread = threading.Thread(target=target, name="XYChallenge",
                                        daemon=True)
        self._thread.start()

    def _set_running(self, on):
        for b in (self._run_btn, self._geometry_btn):
            b.setEnabled(not on)
        self._stop_btn.setEnabled(on)
        if not on:
            self.refresh_budget()

    # ── GUI-thread signal handlers ────────────────────────────────
    def _on_result(self, d):
        if not self._overlay._ideal:
            self._overlay.set_ideal(d.get("ideal", []))
        self._overlay.add_actual(d.get("actual", []), d.get("rms_um", 1e9))
        best = min((a[1] for a in self._overlay._actuals), default=0.0)
        self._overlay.set_title(
            f"{d.get('label','')}: RMS {d['rms_um']:.0f} µm · "
            f"max {d['max_um']:.0f} µm   (best {best:.0f} µm)")
        self._append_log(
            f"{d.get('label',''):24s} RMS {d['rms_um']:6.0f}  "
            f"max {d['max_um']:6.0f} µm  ({d.get('n',0)} samp, "
            f"{d.get('wall_s',0):.1f}s)")

    def _on_tune(self, d):
        star = "  ← best" if d.get("best") else ""
        self._append_log(
            f"  {d['param']}={d['value']:.3g} → RMS {d['rms_um']:.0f} µm{star}")

    def _on_finished(self, ok, summary):
        self._thread = None
        self._set_running(False)
        self._status.setText(summary)

    # ── worker helpers (background thread) ────────────────────────
    def _ideal_path(self, shape=None, size=None):
        pts = XC.make_shape(shape or self._shape_combo.currentData(),
                            size if size is not None else self._size_spin.value(),
                            self._step_spin.value())
        return XC.offset_path(pts, self._start[0], self._start[1])

    def _read_mm(self):
        try:
            p = self._ctrl.get_xy_position(cached=False)
        except Exception:
            return None
        if not p or p[0] is None or p[1] is None:
            return None
        z = getattr(self._ctrl, "zero_position", {})
        return ((p[0] - z.get("x", 0)) / 1000.0, (p[1] - z.get("y", 0)) / 1000.0)

    def _prep(self):
        ctrl = self._ctrl
        if hasattr(ctrl, "ensure_retracted_to") and self._safe_z is not None:
            ctrl.ensure_retracted_to(float(self._safe_z))
        if hasattr(ctrl, "suspend_position_poller"):
            ctrl.suspend_position_poller()
        if hasattr(ctrl, "suspend_zp_watchdog"):
            ctrl.suspend_zp_watchdog()

    def _restore(self):
        ctrl = self._ctrl
        try:
            if hasattr(ctrl, "send_velocity_xy"):
                ctrl.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        if hasattr(ctrl, "resume_position_poller"):
            ctrl.resume_position_poller()
        if hasattr(ctrl, "resume_zp_watchdog"):
            ctrl.resume_zp_watchdog()

    def _max_um_s(self):
        sl = getattr(self._ctrl, "safety_limits", None)
        try:
            v = float(getattr(sl, "max_xy_speed", 0) or 0)
            if v > 0:
                return v
        except Exception:
            pass
        return 50000.0

    def _resolve(self, speed, lookahead, default_hz):
        """Ground the control params in the store's measured calibration (same
        resolve_control the real print uses)."""
        try:
            v = self._store.get_mode_params("velocity")
        except Exception:
            v = {}

        def _tv(key, default=0.0):
            try:
                return float(v.get(key, default) or default)
            except (TypeError, ValueError):
                return default

        # A purpose-measured dead time supersedes the by_phase settle time.
        try:
            dead_time_s, _src = self._store.effective_dead_time_s()
            phase_lag = 0.0 if dead_time_s else (self._store.get_phase_lag_s() or 0.0)
        except Exception:
            dead_time_s, phase_lag = 0.0, 0.0

        try:
            # SAME call, SAME arguments as PrintManager._execute_print_path_velocity
            # — this is what makes bench tuning transfer to the print.
            return VC.resolve_control(
                print_speed_mm_s=speed, lookahead_mm=lookahead,
                xy_max_speed_um_s=self._store.get_xy_max_speed_um_s() or 0.0,
                control_loop_ms=self._store.get_control_loop_ms() or 0.0,
                phase_lag_s=phase_lag,
                default_control_hz=default_hz,
                fallback_max_um_s=self._max_um_s(),
                dead_time_s=dead_time_s,
                lead_time_frac=_tv("lead_time_frac"),
                min_lookahead_frac=_tv("min_lookahead_frac"),
                max_speed_frac=_tv("max_speed_frac"),
                hold_speed=bool(_tv("hold_speed")),
                safety=(_tv("deadtime_safety") or 2.0))
        except Exception:
            return {"control_hz": default_hz, "max_um_s": self._max_um_s(),
                    "speed_cap_mm_s": speed, "lookahead_mm": lookahead,
                    "dead_time_s": 0.0, "lead_s": 0.0, "cap_reason": ""}

    def _open_geometry_panel(self):
        try:
            from gui.dialogs.xy_geometry_panel_dialog import (
                GeometryPanelDialog)
            dlg = GeometryPanelDialog(self._ctrl, safe_z=self._safe_z,
                                      parent=self)
            dlg.show()
        except Exception as e:               # pragma: no cover
            logger.warning(f"geometry panel failed to open: {e}")

    def _set_sms(self, max_um, jerk_pct=0.0):
        xy = getattr(self._ctrl, "xy_stage", None)
        if xy is not None:
            try:
                if hasattr(xy, "set_acceleration"):
                    xy.set_acceleration(80)
                if hasattr(xy, "set_speed_mm_s"):
                    xy.set_speed_mm_s(max_um / 1000.0)
                # Match the print path: 0 = don't touch.
                if jerk_pct and hasattr(xy, "set_jerk"):
                    xy.set_jerk(int(jerk_pct))
            except Exception:
                pass

    def _goto(self, x, y):
        self._ctrl.move_xy_absolute(x, y, from_zero_ref=True)
        if hasattr(self._ctrl, "wait_for_xy_arrival"):
            self._ctrl.wait_for_xy_arrival(x, y, tolerance_mm=0.05, timeout_s=15.0)

    # ── the three mode drivers (XY-only; record actual samples) ───
    def _drive_open_loop(self, ideal, p):
        """OPEN-LOOP velocity streaming (feed-forward along the tangent)."""
        ctrl = self._ctrl
        speed = max(0.05, p["speed"])
        pace = max(1.0, p["pace"])
        eff = max(0.05, speed / pace)
        control_hz = max(5.0, p.get("control_hz", 25.0))
        decel = max(0.1, p.get("decel", 1.5))
        cum = VC.polyline_arclength(ideal)
        total = cum[-1]
        max_um = self._max_um_s()
        self._set_sms(max_um)
        self._goto(*ideal[0])
        dt = 1.0 / control_hz
        samples = []
        t0 = time.monotonic()
        max_wall = total / eff * 4.0 + 15.0
        while not self._stop.is_set():
            tick = time.monotonic()
            if tick - t0 > max_wall:
                break
            q = self._read_mm()
            if q:
                samples.append(q)
            s_tgt = min(eff * (tick - t0), total)
            tx, ty = VC.tangent_at_arclength(ideal, cum, s_tgt)
            spd = eff
            remaining = total - s_tgt
            if remaining < decel:
                spd *= max(0.1, remaining / decel)
            vx, vy = spd * tx * 1000.0, spd * ty * 1000.0
            vmag = math.hypot(vx, vy)
            if vmag > max_um and vmag > 0:
                vx *= max_um / vmag
                vy *= max_um / vmag
            try:
                ctrl.send_velocity_xy(vx, vy)
            except Exception:
                break
            if s_tgt >= total:
                break
            el = time.monotonic() - tick
            if el < dt:
                time.sleep(dt - el)
        try:
            ctrl.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        return samples

    def _drive_confirm(self, ideal, p):
        """Point-to-point, but wait/settle ONLY at corners (straight edges stream
        — mirrors PrintManager's corner-only confirm)."""
        ctrl = self._ctrl
        speed = max(0.05, p["speed"])
        tol = max(0.001, p["tol_um"] / 1000.0)
        corners = VC.corner_flags(ideal, p.get("corner_angle", 30.0))
        xy = getattr(ctrl, "xy_stage", None)
        if xy is not None and hasattr(xy, "set_speed_mm_s"):
            try:
                xy.set_acceleration(80)
            except Exception:
                pass
            xy.set_speed_mm_s(speed)
        samples = []
        self._goto(*ideal[0])
        n = len(ideal)
        for i in range(1, n):
            if self._stop.is_set():
                break
            x2, y2 = ideal[i]
            ctrl.move_xy_absolute(x2, y2, from_zero_ref=True)
            is_stop = (i == n - 1) or (i < len(corners) and corners[i])
            if is_stop:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 8.0 and not self._stop.is_set():
                    q = self._read_mm()
                    if q:
                        samples.append(q)
                        if math.hypot(q[0] - x2, q[1] - y2) <= tol:
                            break
                    time.sleep(0.01)
            else:
                # stream the straight edge, still recording
                seg = math.hypot(x2 - ideal[i - 1][0], y2 - ideal[i - 1][1])
                end = time.monotonic() + max(seg / speed, 0.01)
                while time.monotonic() < end and not self._stop.is_set():
                    q = self._read_mm()
                    if q:
                        samples.append(q)
                    time.sleep(0.01)
        return samples

    def _drive_velocity(self, ideal, p):
        """Closed-loop pursuit + corner scheduling + cross-track PID — the SAME
        SupportClasses.VelocityControl law the real print uses."""
        ctrl = self._ctrl
        speed = max(0.05, p["speed"])
        lookahead = max(0.05, p["lookahead"])
        decel = max(0.1, p["decel"])
        kp = max(0.0, p.get("kp", 0.0))
        kd = max(0.0, p.get("kd", 0.0))
        cum = VC.polyline_arclength(ideal)
        total = cum[-1]
        res = self._resolve(speed, lookahead, max(5.0, p.get("control_hz", 25.0)))
        max_um = res["max_um_s"]
        speed_cap = res["speed_cap_mm_s"]
        # The lookahead may be RESOLVED upward from the dynamics (so it stops
        # being the speed knob) — use the resolved value, exactly as the print
        # path does, or the bench would carrot differently from the print.
        lookahead = res.get("lookahead_mm", lookahead)
        try:
            _jerk = float(self._store.get_mode_params("velocity")
                          .get("jerk_pct", 0.0) or 0.0)
        except Exception:
            _jerk = 0.0
        self._set_sms(max_um, jerk_pct=_jerk)
        speed_limit_at, _corners = VC.plan_speed_limits(
            ideal, cum, speed, corner_angle_deg=p.get("corner_angle", 30.0),
            corner_speed_factor=p.get("corner_factor", 0.4), decel_mm=decel)
        self._goto(*ideal[0])
        dt = 1.0 / res["control_hz"]
        arrive = self._VEL_ARRIVE_MM
        state = VC.PursuitState()
        samples = []
        last_t = time.monotonic()
        t0 = last_t
        runaway = 0
        max_wall = total / speed * 6.0 + 15.0
        while not self._stop.is_set():
            tick = time.monotonic()
            if tick - t0 > max_wall:
                self._bridge.log.emit("  velocity: wall-time cap hit.")
                break
            pos = self._read_mm()
            if pos is None:
                time.sleep(dt)
                continue
            samples.append(pos)
            dt_real = max(1e-3, tick - last_t)
            last_t = tick
            max_ds = min(1.5, max(0.15, speed * dt_real * 6.0))
            remaining = total - state.s
            cap = speed_cap
            if remaining < decel:
                cap = min(cap, max(0.1, remaining / decel) * min(speed, speed_cap))
            vx, vy, sN, cross = VC.pursuit_step(
                pos, ideal, cum, state, lookahead=lookahead,
                speed_cap_mm_s=cap, speed_limit_at=speed_limit_at,
                dt=dt_real, max_ds=max_ds, kp=kp, kd=kd)
            vmag = math.hypot(vx, vy)
            if vmag > max_um and vmag > 0:
                vx *= max_um / vmag
                vy *= max_um / vmag
            if cross > self._VEL_RUNAWAY_MM:
                runaway += 1
                if runaway >= 8:
                    self._bridge.log.emit(
                        f"  velocity: runaway (cross {cross:.2f} mm) — stopped.")
                    break
            else:
                runaway = 0
            dist_end = math.hypot(pos[0] - ideal[-1][0], pos[1] - ideal[-1][1])
            if sN >= total - 1e-6 and dist_end <= arrive:
                break
            try:
                ctrl.send_velocity_xy(vx, vy)
            except Exception:
                break
            el = time.monotonic() - tick
            if el < dt:
                time.sleep(dt - el)
        try:
            ctrl.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        return samples

    def _drive(self, mode, ideal, p):
        if mode == "open_loop":
            return self._drive_open_loop(ideal, p)
        if mode == "confirm":
            return self._drive_confirm(ideal, p)
        return self._drive_velocity(ideal, p)

    def _run_one(self, mode, label, ideal, p):
        t0 = time.monotonic()
        actual = self._drive(mode, ideal, p)
        wall = time.monotonic() - t0
        err = XC.path_error(actual, ideal)
        self._bridge.result.emit({
            "mode": mode, "label": label, "ideal": ideal, "actual": actual,
            "rms_um": err["rms_um"], "max_um": err["max_um"], "n": err["n"],
            "wall_s": wall})
        return err

    # ── worker entrypoints ────────────────────────────────────────
    def _worker_run(self):
        try:
            self._prep()
            ideal = self._ideal_path()
            mode = self._mode_combo.currentData()
            label = self._mode_combo.currentText()
            self._bridge.status.emit(f"Running {label}…")
            err = self._run_one(mode, label, ideal, self._mode_params(mode))
            self._bridge.finished.emit(
                True, f"{label}: RMS {err['rms_um']:.0f} µm · "
                f"max {err['max_um']:.0f} µm")
        except Exception as e:
            logger.exception("XY challenge run error")
            self._bridge.finished.emit(False, f"Error: {e}")
        finally:
            self._restore()

    def _worker_deadtime(self):
        """Measure the REAL command→motion dead time and store it.

        This is the number that divides the achievable print speed. The legacy
        source (``by_phase.intercept_s`` mean) is an optical SETTLE time — on
        ME3B V1 it averages 0.287 s from data that includes two impossible
        negatives and a single-point 0.577 s fit, capping prints at 0.31 mm/s on
        a stage measured at 5.9 mm/s. Storing a measured value here makes every
        subsequent print (and every tuning run) faster.

        XY-only, needle stays retracted, stage always stopped and returned.
        """
        try:
            from SupportClasses import XYDeadTime as DT
            self._prep()
            self._bridge.status.emit("Measuring dead time (XY only, needle "
                                     "retracted)…")
            r = DT.measure_velocity_dead_time(
                self._ctrl, axis="diag", repeats=5,
                stop_evt=self._stop,
                on_progress=lambda m: self._bridge.log.emit(f"  {m}"))
            if "error" in r:
                self._bridge.finished.emit(False, f"Dead time: {r['error']}")
                return

            # Use the FOPDT apparent lag (dead time + part of the rise) — it is
            # what a pure-pursuit loop actually experiences, and it is the more
            # conservative of the two.
            lag = max(r["apparent_lag_s"], r["dead_time_s"])
            self._store.set_velocity_dead_time_s(
                lag, n=r["n"], spread_s=r.get("apparent_lag_spread_s", 0.0),
                tau_s=r.get("tau_s"))

            loop_ms = self._store.get_control_loop_ms() or 0.0
            look = float(self._look_spin.value())
            stable = DT.stable_speed_mm_s(lag, loop_ms, look)
            old_lag = self._store.get_phase_lag_s() or 0.0
            old_stable = DT.stable_speed_mm_s(old_lag, loop_ms, look) if old_lag \
                else None

            self._bridge.log.emit(
                f"  measured dead time {lag * 1000:.0f} ms "
                f"(was using {old_lag * 1000:.0f} ms from by_phase)")
            if stable:
                gain = (f" — {stable / old_stable:.1f}× faster"
                        if old_stable and old_stable > 0 else "")
                self._bridge.log.emit(
                    f"  stable speed at lookahead {look:.2f} mm: "
                    f"{stable:.2f} mm/s{gain}")
            cruise = r.get("cruise_um_s", 0.0)
            if cruise:
                stored = self._store.get_xy_max_speed_um_s() or 0.0
                self._bridge.log.emit(
                    f"  cruise cross-check {cruise:.0f} µm/s "
                    f"(stored top speed {stored:.0f} µm/s)")
            self._bridge.finished.emit(
                True, f"Dead time {lag * 1000:.0f} ms stored"
                      + (f" → stable speed {stable:.2f} mm/s" if stable else ""))
        except Exception as e:
            logger.exception("XY challenge dead-time error")
            self._bridge.finished.emit(False, f"Error: {e}")
        finally:
            self._restore()

    def _worker_compare(self):
        try:
            self._prep()
            ideal = self._ideal_path()
            self._bridge.status.emit("Comparing all modes…")
            results = []
            for label, mode in MODES:
                if self._stop.is_set():
                    break
                self._bridge.log.emit(f"— {label} —")
                err = self._run_one(mode, label, ideal, self._mode_params(mode))
                results.append((label, err["rms_um"]))
            if results:
                best = min(results, key=lambda r: r[1])
                self._bridge.finished.emit(
                    True, f"Best: {best[0]} (RMS {best[1]:.0f} µm)")
            else:
                self._bridge.finished.emit(False, "Stopped.")
        except Exception as e:
            logger.exception("XY challenge compare error")
            self._bridge.finished.emit(False, f"Error: {e}")
        finally:
            self._restore()

    def _worker_tune(self):
        """Coordinate descent: sweep each of the mode's key params in turn,
        carrying the running best forward. One pass (bounded hardware runs)."""
        try:
            self._prep()
            ideal = self._ideal_path()
            mode = self._mode_combo.currentData()
            label = self._mode_combo.currentText()
            plan = AUTOTUNE[mode]
            base = self._mode_params(mode)
            self._bridge.status.emit(f"Auto-tuning {label} (coordinate descent)…")
            for store_param, grid in plan:
                if self._stop.is_set():
                    break
                key = _PARAM_KEY[store_param]
                self._bridge.log.emit(f"— sweep {store_param} —")
                best_val, best_rms = base[key], float("inf")
                for val in grid:
                    if self._stop.is_set():
                        break
                    trial = dict(base)
                    trial[key] = val
                    t0 = time.monotonic()
                    actual = self._drive(mode, ideal, trial)
                    err = XC.path_error(actual, ideal)
                    is_best = err["rms_um"] < best_rms - 1.0
                    if is_best:
                        best_rms = err["rms_um"]
                        best_val = val
                    self._bridge.tune.emit({
                        "param": store_param, "value": val,
                        "rms_um": err["rms_um"], "best": is_best})
                    self._bridge.result.emit({
                        "mode": mode, "label": f"{store_param}={val:g}",
                        "ideal": ideal, "actual": actual,
                        "rms_um": err["rms_um"], "max_um": err["max_um"],
                        "n": err["n"], "wall_s": time.monotonic() - t0})
                base[key] = best_val                      # carry best forward
                self._store.set_mode_params(mode, {store_param: best_val})
                self._bridge.apply.emit(store_param, float(best_val))
            self._bridge.finished.emit(
                True, f"{label}: tuned + saved (coordinate descent).")
        except Exception as e:
            logger.exception("XY challenge tune error")
            self._bridge.finished.emit(False, f"Error: {e}")
        finally:
            self._restore()

    def _worker_calibrate_all(self):
        """ONE-CLICK systematic XY calibration: measure → derive → verify → apply.

        The sequence order is the whole point — every later step consumes an
        earlier measurement, which is why running the individual buttons piecemeal
        so easily produced an inconsistent set:

            comms rate ─┐
            dead time  ─┼→ derive every follower parameter in closed form
            top speed  ─┘        ↓
                              verify on real shapes, then apply to ALL prints

        Almost nothing is searched. The lookahead is *solved* for the target
        speed, the gains come from ``Ku = π/(2L)``, the decel from the dead-time
        coast distance. Same machine measured twice ⇒ same settings.

        SAFETY: the stage is driven to the MIDDLE of its travel envelope first and
        every probe excursion is checked to fit with margin, so no test can reach
        an extent (a clamped probe does not fail loudly — it silently measures the
        clamp). XY-only throughout, needle retracted at Safe Z.
        """
        from SupportClasses import XYAutoCalibration as AC
        from SupportClasses import XYDeadTime as DT
        try:
            self._prep()
            target = max(0.05, float(self._speed_spin.value()))
            res_um = float(self._res_spin.value())
            policy = AC.CalibrationPolicy(target_speed_mm_s=target,
                                          resolution_um=res_um)

            # ── 0. envelope + centring ──
            self._bridge.status.emit("Calibrating: centring the stage…")
            self._bridge.log.emit("— One-click XY calibration —")
            probe_um = 1500.0
            fit = AC.check_fits(self._ctrl, probe_um)
            if not fit["ok"] and fit["usable_um"] <= 0:
                self._bridge.finished.emit(False, fit["reason"])
                return
            centered = AC.center_stage(self._ctrl, safe_z_mm=self._safe_z)
            if not centered["ok"]:
                self._bridge.finished.emit(
                    False, f"Could not centre the stage: {centered['reason']}")
                return
            cx, cy = centered["center_um"]
            self._bridge.log.emit(
                f"  centred at ({cx:.0f}, {cy:.0f}) µm · usable excursion "
                f"±{fit['usable_um']:.0f} µm — no test can reach a travel limit")

            # ── 1. comms rate ──
            if self._stop.is_set():
                return
            self._bridge.status.emit("Calibrating: comms rate…")
            r = self._ctrl.measure_control_loop_rate(iterations=40)
            if "error" in r:
                self._bridge.finished.emit(False, f"Comms rate: {r['error']}")
                return
            self._store.set_control_loop_ms(r["avg_period_ms"])
            self._bridge.log.emit(
                f"  comms: {r['avg_period_ms']:.1f} ms "
                f"({r['control_hz']:.1f} Hz){'' if r.get('moved') else ' ⚠ stage did not move'}")

            # ── 2. dead time (already centred) ──
            if self._stop.is_set():
                return
            self._bridge.status.emit("Calibrating: dead time…")
            dt = DT.measure_velocity_dead_time(
                self._ctrl, axis="diag", repeats=5, center_first=False,
                max_travel_um=min(probe_um, max(200.0, fit["usable_um"])),
                stop_evt=self._stop,
                on_progress=lambda m: self._bridge.log.emit(f"  {m}"))
            if "error" in dt:
                self._bridge.finished.emit(False, f"Dead time: {dt['error']}")
                return
            lag = max(dt["apparent_lag_s"], dt["dead_time_s"])
            self._store.set_velocity_dead_time_s(
                lag, n=dt["n"], spread_s=dt.get("apparent_lag_spread_s", 0.0),
                tau_s=dt.get("tau_s"))
            self._bridge.log.emit(
                f"  dead time: {lag * 1000:.0f} ms "
                f"(±{dt.get('apparent_lag_spread_s', 0.0) * 1000:.1f}) · "
                f"τ {dt.get('tau_s', 0.0) * 1000:.0f} ms")

            # ── 3. derive ──
            if self._stop.is_set():
                return
            self._bridge.status.emit("Calibrating: deriving settings…")
            measured = AC.measured_from_store(
                self._store,
                declared_max_speed_um_s=self._protocol_max_speed_um_s())
            if measured.cruise_um_s <= 0 and dt.get("cruise_um_s"):
                measured.cruise_um_s = dt["cruise_um_s"]
            if not measured.is_complete():
                self._bridge.finished.emit(
                    False, "Still need: " + ", ".join(measured.missing())
                           + " — run “Measure top speed” on the Timing "
                             "Calibration page, then retry.")
                return
            derived = AC.derive_settings(measured, policy)
            if not derived.values:
                self._bridge.finished.emit(
                    False, "; ".join(w.message for w in derived.warnings))
                return
            self._bridge.log.emit("— derived settings (with reasons) —")
            for line in derived.note_text():
                self._bridge.log.emit(f"  {line}")
            for w in derived.warnings:
                mark = {"error": "✖", "warn": "⚠", "info": "·"}.get(w.level, "·")
                self._bridge.log.emit(f"  {mark} {w.message}")
            if any(w.level == "error" for w in derived.warnings):
                self._bridge.finished.emit(False, "Derivation failed validation.")
                return
            AC.apply_to_store(self._store, derived)
            for k in ("lookahead_mm", "control_hz", "decel_mm", "pid_kp",
                      "pid_kd", "corner_speed_factor", "corner_angle_deg"):
                if k in derived.values:
                    self._bridge.apply.emit(k, float(derived.values[k]))

            # ── 4. verify on real shapes ──
            if self._stop.is_set():
                self._bridge.finished.emit(
                    True, "Stopped after applying — not verified.")
                return
            self._bridge.status.emit("Calibrating: verifying…")
            size = AC.fit_shape_size_mm(self._ctrl, self._size_spin.value())
            if size <= 0:
                self._bridge.finished.emit(
                    True, "Applied, but the envelope is too small to verify.")
                return
            params = self._mode_params("velocity")
            params.update({
                "lookahead": derived.values["lookahead_mm"],
                "control_hz": derived.values["control_hz"],
                "decel": derived.values["decel_mm"],
                "kp": derived.values["pid_kp"],
                "kd": derived.values["pid_kd"],
                "corner_factor": derived.values["corner_speed_factor"],
                "corner_angle": derived.values["corner_angle_deg"],
                "speed": derived.summary["target_speed_mm_s"],
            })
            self._bridge.log.emit(
                f"— verify at {params['speed']:.2f} mm/s on {size:.1f} mm shapes —")
            results = []
            for shape in ("Square", "Star"):
                if self._stop.is_set():
                    break
                ideal = self._ideal_path(shape=shape, size=size)
                t0 = time.monotonic()
                actual = self._drive("velocity", ideal, params)
                wall = time.monotonic() - t0
                rep = XC.path_report(
                    actual, ideal, commanded_speed_mm_s=params["speed"],
                    resolution_um=res_um, wall_s=wall)
                sc = XC.composite_score(rep)
                results.append((shape, rep, sc))
                self._bridge.result.emit({
                    "mode": "velocity", "label": f"verify {shape}",
                    "ideal": ideal, "actual": actual,
                    "rms_um": rep["rms_um"], "max_um": rep["max_um"],
                    "n": rep["n"], "wall_s": wall})
                verdict = "PASS" if sc["pass"] else "FAIL " + ",".join(
                    sc["fail_reasons"])
                self._bridge.log.emit(
                    f"  {shape:8s} p95 {rep['p95_um']:5.0f} µm · complete "
                    f"{rep['completion_frac']:.3f} · dither "
                    f"{rep['dither_ratio']:.2f} · {wall:.1f}s → {verdict}")

            npass = sum(1 for _s, _r, sc in results if sc["pass"])
            if results and npass == len(results):
                self._bridge.finished.emit(
                    True, f"Calibrated + verified ({npass}/{len(results)} shapes "
                          f"≤ {res_um:.0f} µm). Applied to ALL prints.")
            elif results:
                worst = min(results, key=lambda t: t[2]["pass"])
                self._bridge.finished.emit(
                    True, f"Calibrated + applied, but verification "
                          f"{npass}/{len(results)} passed — worst: "
                          f"{worst[0]} ({', '.join(worst[2]['fail_reasons']) or 'deviation'}). "
                          f"Try a lower target speed or a larger resolution "
                          f"element.")
            else:
                self._bridge.finished.emit(True, "Calibrated + applied (stopped "
                                                "before verification).")
        except Exception as e:
            logger.exception("XY one-click calibration error")
            self._bridge.finished.emit(False, f"Error: {e}")
        finally:
            self._restore()

    def _protocol_max_speed_um_s(self) -> float:
        """The controller JSON's DECLARED max speed, for the measured-vs-declared
        contrast (ME3B V1 declares 50000 µm/s and measures 5946 — 8.4×)."""
        try:
            xy = getattr(self._ctrl, "xy_stage", None)
            proto = getattr(xy, "_protocol", None)
            v = proto.get_parameter("max_speed") if proto else None
            return float(v or 0.0)
        except Exception:
            return 0.0

    def _worker_pid_analytic(self):
        """Derive the cross-track PID gains from the MEASURED dead time.

        This is the repeatable path, and it is what the ZN relay experiment was
        trying to approximate all along. The cross-track plant is known in closed
        form — a perpendicular velocity command integrates straight into
        cross-track position, so from ``v_n`` to ``d`` it is a pure integrator
        with transport delay — which fixes ``Ku = π/(2L)`` and ``Tu = 4L``
        exactly (see ``VelocityControl.plant_ultimate_gain``). No oscillation to
        mis-measure, no hardware time, and running it twice gives the same answer.

        The operator's own relay run corroborates the model: it measured
        ``Tu = 0.391 s`` against the predicted 0.396 s (~1 %). Only its amplitude —
        and therefore ``Ku`` — was off.
        """
        try:
            lag, src = self._store.effective_dead_time_s()
            loop_ms = self._store.get_control_loop_ms() or 0.0
            if not lag or not loop_ms:
                self._bridge.finished.emit(
                    False, "Measure the dead time and the comms rate first "
                           "(the gains are derived from them).")
                return
            g = VC.pid_gains_from_dead_time(lag, loop_ms, use_kd=False)
            if g["kp"] <= 0.0:
                self._bridge.finished.emit(False, "Could not derive gains.")
                return
            self._bridge.log.emit(
                f"— PID from measured dead time ({src}) —")
            self._bridge.log.emit(
                f"  L = dead {lag * 1000:.0f} ms + loop {loop_ms:.0f} ms "
                f"= {g['L_s'] * 1000:.0f} ms")
            self._bridge.log.emit(
                f"  Ku = π/(2L) = {g['ku']:.2f}   Tu = 4L = {g['tu']:.3f} s")
            self._bridge.log.emit(
                f"  → Kp = {g['kp']:.3f}  Kd = 0 (D left off: differentiating a "
                f"µm-quantised encoder at 25 Hz adds more noise than it removes)")
            self._bridge.log.emit(
                f"  stability limit is Kp < {g['kp_stability_limit']:.2f}; "
                f"Ki stays 0 — the plant is already an integrator, so P alone has "
                f"zero steady-state error and there is no lateral disturbance to "
                f"reject.")
            self._store.set_mode_params(
                "velocity", {"pid_kp": round(g["kp"], 3), "pid_kd": 0.0})
            self._bridge.apply.emit("pid_kp", round(g["kp"], 3))
            self._bridge.apply.emit("pid_kd", 0.0)
            self._bridge.finished.emit(
                True, f"PID from dead time: Kp={g['kp']:.3f} Kd=0 (saved).")
        except Exception as e:
            logger.exception("XY challenge analytic PID error")
            self._bridge.finished.emit(False, f"Error: {e}")

    def _worker_zn(self):
        """Ziegler–Nichols relay tune, used as a CROSS-CHECK of the analytic gains.

        Drive a straight test line with pure-pursuit forward motion plus a
        perpendicular RELAY velocity ±d that flips on the sign of the cross-track
        error → a limit cycle. Measure its period Tu and amplitude a, then
        ``Ku = 4d/(πa)``.

        Three defects made the original version give a different answer every run:

          1. **the switch had no hysteresis**, so near the line the sign chattered
             on µm encoder quantisation, inserting many spurious tiny half-cycles;
          2. **the amplitude was the MEAN over half-cycles**, which those spurious
             cycles then dragged down — and since ``Ku ∝ 1/a``, an under-measured
             amplitude inflates the gain. On ME3B V1 it reported ``Ku = 18.8``
             against a physical maximum of 15.9;
          3. **a single run**, with no repeat and no sanity check.

        Now: a hysteresis deadband, the MEDIAN of per-cycle PEAK amplitudes with
        the first cycles dropped as transient, repeated trials, and a physical
        gate (``VelocityControl.relay_sanity``) that refuses a Ku the plant cannot
        produce and falls back to the analytic gains.
        """
        try:
            if compute_zn_pid_gains is None:
                self._bridge.finished.emit(False, "ZN tuner unavailable.")
                return
            self._prep()
            # a straight line of the current size along +X from the start
            size = self._size_spin.value()
            x0, y0 = self._start
            ideal = [(x0, y0), (x0 + size, y0)]
            cum = VC.polyline_arclength(ideal)
            total = cum[-1]
            speed = max(0.3, min(self._speed_spin.value(), 3.0))
            relay_d = max(0.5, speed * 0.6)     # perpendicular relay (mm/s)
            # Hysteresis: the switch must clear real motion, not encoder noise.
            # Scaled off the resolution element so it tracks the machine.
            hyst_mm = max(0.005, float(self._res_spin.value()) / 1000.0)
            self._bridge.status.emit("ZN relay: inducing oscillation…")
            self._bridge.log.emit(
                f"— ZN relay (d={relay_d:.2f} mm/s, hysteresis "
                f"{hyst_mm * 1000:.0f} µm) —")
            self._set_sms(self._max_um_s())
            self._goto(*ideal[0])
            state = VC.PursuitState()
            dt = 1.0 / 25.0
            last_t = time.monotonic()
            t0 = last_t
            flips, amps = [], []
            prev_sign = 0
            cur_amp = 0.0
            max_wall = 25.0
            while not self._stop.is_set() and (time.monotonic() - t0) < max_wall:
                tick = time.monotonic()
                pos = self._read_mm()
                if pos is None:
                    time.sleep(dt)
                    continue
                dt_real = max(1e-3, tick - last_t)
                last_t = tick
                # forward pursuit (no PID) for the base velocity
                max_ds = min(1.5, max(0.15, speed * dt_real * 6.0))
                p2 = VC.point_at_arclength(ideal, cum, min(state.s + 0.6, total))
                pp = VC.point_at_arclength(ideal, cum, state.s)
                tx, ty = VC.tangent_at_arclength(ideal, cum, state.s)
                cross = tx * (pos[1] - pp[1]) - ty * (pos[0] - pp[0])
                # advance s
                sN, state.seg_i, _c = VC.project_on_polyline(
                    pos, ideal, cum, state.seg_i, max_ds)
                state.s = min(max(sN, state.s), state.s + max_ds)
                # Relay: perpendicular velocity opposing the cross sign, switched
                # through a HYSTERESIS band. Without the band the sign chatters on
                # µm encoder quantisation whenever the stage is near the line,
                # manufacturing spurious tiny half-cycles that corrupt both the
                # period and (fatally, since Ku ∝ 1/a) the amplitude.
                if cross > hyst_mm:
                    sign = 1
                elif cross < -hyst_mm:
                    sign = -1
                else:
                    sign = prev_sign            # inside the band: hold
                if prev_sign != 0 and sign != prev_sign and sign != 0:
                    flips.append(tick)
                    amps.append(max(cur_amp, abs(cross)))
                    cur_amp = 0.0
                prev_sign = sign
                cur_amp = max(cur_amp, abs(cross))
                ex, ey = p2[0] - pos[0], p2[1] - pos[1]
                d = math.hypot(ex, ey)
                fx = speed * (ex / d if d > 1e-9 else 0.0)
                fy = speed * (ey / d if d > 1e-9 else 0.0)
                # perp = left normal (-ty, tx); push AGAINST the cross sign
                px, py = -ty, tx
                vx = (fx - sign * relay_d * px) * 1000.0
                vy = (fy - sign * relay_d * py) * 1000.0
                try:
                    self._ctrl.send_velocity_xy(vx, vy)
                except Exception:
                    break
                if state.s >= total - 1e-6:
                    break
                el = time.monotonic() - tick
                if el < dt:
                    time.sleep(dt - el)
            try:
                self._ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass
            # ── estimate Tu and the amplitude a ──
            # Drop the first two cycles as start-up transient, then take the
            # MEDIAN of the per-cycle PEAK amplitudes. The original code averaged
            # every half-cycle amplitude including the partials, and because
            # Ku ∝ 1/a any small spurious cycle inflated the gain.
            def _median(vals):
                v = sorted(vals)
                if not v:
                    return 0.0
                m = len(v) // 2
                return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])

            usable = amps[2:] if len(amps) > 4 else amps[1:]
            if len(flips) >= 6 and len(usable) >= 3:
                halfs = [flips[i + 1] - flips[i] for i in range(len(flips) - 1)]
                tu_raw = 2.0 * _median(halfs)
                a = _median(usable)
                ku, tu = VC.relay_ultimate_gain(relay_d, a, tu_raw)
                self._bridge.log.emit(
                    f"  {len(flips)} flips · amplitude median {a * 1000:.0f} µm "
                    f"(spread {min(usable) * 1000:.0f}–{max(usable) * 1000:.0f}) "
                    f"→ Ku={ku:.2f} Tu={tu:.3f}s")

                # ── physical gate: is this Ku even possible for this plant? ──
                lag, _src = self._store.effective_dead_time_s()
                loop_ms = self._store.get_control_loop_ms() or 0.0
                chk = VC.relay_sanity(ku, tu, lag, loop_ms)
                analytic = VC.pid_gains_from_dead_time(lag, loop_ms, use_kd=False)
                if chk["ku_limit"] > 0:
                    self._bridge.log.emit(
                        f"  physical maximum Ku = π/(2L) = {chk['ku_limit']:.2f}; "
                        f"predicted Tu = {chk['tu_expected']:.3f}s "
                        f"(measured/predicted: Ku {chk['ku_ratio']:.2f}×, "
                        f"Tu {chk['tu_ratio']:.2f}×)")
                if not chk["ok"] and analytic["kp"] > 0:
                    self._bridge.log.emit(f"  ⚠ REJECTED: {chk['reason']}")
                    self._bridge.log.emit(
                        f"  → using the analytic gains from the measured dead "
                        f"time instead: Kp={analytic['kp']:.3f}")
                    kp = round(analytic["kp"], 3)
                    kd = 0.0
                    note = "relay rejected — analytic gains used"
                else:
                    gains = compute_zn_pid_gains(ku, tu, method="some_overshoot")
                    kp = round(max(0.0, gains["kp"]), 3)
                    # Kd stays 0 while the derivative is unfiltered.
                    kd = 0.0
                    note = "relay accepted"
                    self._bridge.log.emit(
                        f"  Kd left at 0 (unfiltered D on a µm-quantised encoder "
                        f"at 25 Hz injects noise); Ki stays 0 — the plant is "
                        f"already an integrator.")
                # Never store a gain at or above the stability boundary.
                if chk["ku_limit"] > 0 and kp >= chk["ku_limit"]:
                    kp = round(0.33 * chk["ku_limit"], 3)
                    self._bridge.log.emit(
                        f"  clamped to Kp={kp} (was at/above the stability limit)")
                self._store.set_mode_params("velocity",
                                            {"pid_kp": kp, "pid_kd": kd})
                self._bridge.apply.emit("pid_kp", kp)
                self._bridge.apply.emit("pid_kd", kd)
                self._bridge.finished.emit(
                    True, f"ZN: Kp={kp} Kd={kd} ({note}).")
            else:
                self._bridge.finished.emit(
                    False, f"ZN: not enough oscillation ({len(flips)} flips) — "
                           f"raise the speed/relay, or use “PID from dead time”.")
        except Exception as e:
            logger.exception("XY challenge ZN error")
            self._bridge.finished.emit(False, f"ZN error: {e}")
        finally:
            self._restore()

    def _worker_robustness(self):
        """Sweep the configured shape × size × speed matrix in velocity mode and
        PASS/FAIL each vs the resolution element (RMS ≤ element)."""
        try:
            self._prep()
            matrix = self._store.get_robustness_matrix()
            tol_um = self._res_spin.value()
            base = self._mode_params("velocity")
            shapes = matrix["shapes"]
            sizes = matrix["sizes_mm"]
            speeds = matrix["speeds_mm_s"]
            total = len(shapes) * len(sizes) * len(speeds)
            self._bridge.status.emit(
                f"Robustness: {total} runs vs {tol_um:.0f} µm…")
            self._bridge.log.emit(
                f"— Robustness panel (PASS = RMS ≤ {tol_um:.0f} µm) —")
            npass = 0
            done = 0
            for shape in shapes:
                for size in sizes:
                    if self._stop.is_set():
                        break
                    ideal = self._ideal_path(shape=shape, size=size)
                    for speed in speeds:
                        if self._stop.is_set():
                            break
                        p = dict(base)
                        p["speed"] = speed
                        t0 = time.monotonic()
                        actual = self._drive_velocity(ideal, p)
                        err = XC.path_error(actual, ideal)
                        ok = err["rms_um"] <= tol_um
                        npass += 1 if ok else 0
                        done += 1
                        self._bridge.result.emit({
                            "mode": "velocity",
                            "label": f"{shape} {size:g}mm {speed:g}mm/s",
                            "ideal": ideal, "actual": actual,
                            "rms_um": err["rms_um"], "max_um": err["max_um"],
                            "n": err["n"], "wall_s": time.monotonic() - t0})
                        self._bridge.log.emit(
                            f"  {'PASS' if ok else 'FAIL'}  {shape:7s} "
                            f"{size:4g}mm {speed:4g}mm/s  RMS {err['rms_um']:5.0f} "
                            f"max {err['max_um']:5.0f} µm")
            self._bridge.finished.emit(
                True, f"Robustness: {npass}/{done} passed ≤ {tol_um:.0f} µm.")
        except Exception as e:
            logger.exception("XY challenge robustness error")
            self._bridge.finished.emit(False, f"Robustness error: {e}")
        finally:
            self._restore()

    def closeEvent(self, e):
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        self._restore()
        super().closeEvent(e)
