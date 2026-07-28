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

# Which param rows each mode shows (internal param keys).
MODE_PARAMS = {
    "open_loop": ["speed", "resolution_um", "pace", "control_hz", "decel"],
    "confirm":   ["speed", "resolution_um", "tol_um", "corner_angle"],
    "velocity":  ["speed", "resolution_um", "lookahead", "control_hz", "decel",
                  "corner_angle", "corner_factor", "kp", "kd"],
}

# Coordinate-descent: ordered (store-param, grid) list swept per mode. The
# internal _mode_params key is resolved via _PARAM_KEY.
AUTOTUNE = {
    "open_loop": [("pace_correction", [1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0])],
    "confirm":   [("settle_tol_um", [10.0, 20.0, 30.0, 45.0, 60.0, 90.0]),
                  ("corner_angle_deg", [15.0, 25.0, 35.0, 50.0])],
    "velocity":  [("lookahead_mm", [0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.3]),
                  ("corner_speed_factor", [0.2, 0.3, 0.4, 0.55, 0.7]),
                  ("pid_kp", [0.0, 0.5, 1.0, 2.0, 4.0]),
                  ("pid_kd", [0.0, 0.02, 0.05, 0.1])],
}

# store-param name → the _mode_params dict key
_PARAM_KEY = {
    "pace_correction": "pace", "settle_tol_um": "tol_um",
    "corner_angle_deg": "corner_angle", "lookahead_mm": "lookahead",
    "corner_speed_factor": "corner_factor", "pid_kp": "kp", "pid_kd": "kd",
    "control_hz": "control_hz", "decel_mm": "decel",
}


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

        # Per-mode params (each in a row widget, shown/hidden by mode)
        self._speed_spin = self._dspin(0.1, 50.0, 5.0, 2, 0.5, " mm/s",
                                       "Path speed for the run (all modes).")
        self._res_spin = self._dspin(1.0, 500.0, 30.0, 0, 1.0, " µm",
                                     "Resolution element — smallest feature / "
                                     "acceptable path-deviation tolerance. Drives "
                                     "robustness PASS/FAIL.")
        self._pace_spin = self._dspin(1.0, 4.0, 1.0, 2, 0.05, " ×",
                                      "[open-loop] speed trim — divides the "
                                      "commanded velocity to match a slow stage.")
        self._tol_spin = self._dspin(2.0, 200.0, 40.0, 0, 5.0, " µm",
                                     "[confirm] arrival tolerance at a corner.")
        self._look_spin = self._dspin(0.1, 3.0, 0.6, 2, 0.05, " mm",
                                      "[velocity] pure-pursuit lookahead "
                                      "(smaller = tighter corners, more jitter).")
        self._hz_spin = self._dspin(5.0, 60.0, 25.0, 0, 1.0, " Hz",
                                    "[open-loop/velocity] control-loop rate.")
        self._decel_spin = self._dspin(0.2, 6.0, 1.5, 1, 0.1, " mm",
                                       "[open-loop/velocity] end-of-path ramp-down.")
        self._corner_angle_spin = self._dspin(5.0, 90.0, 30.0, 0, 5.0, " °",
                                     "Turn angle counted as a corner (slow into / "
                                     "stop at).")
        self._corner_fac_spin = self._dspin(0.05, 1.0, 0.4, 2, 0.05, " ×",
                                     "[velocity] fraction of speed allowed at the "
                                     "sharpest corner.")
        self._kp_spin = self._dspin(0.0, 20.0, 0.0, 2, 0.1, "",
                                    "[velocity] cross-track PID proportional gain "
                                    "(0 = pure pursuit).")
        self._kd_spin = self._dspin(0.0, 2.0, 0.0, 3, 0.01, "",
                                    "[velocity] cross-track PID derivative gain.")

        pg = QGridLayout()
        pg.setHorizontalSpacing(s(8))
        pg.setVerticalSpacing(s(4))
        rows = [
            ("speed", "Speed:", self._speed_spin),
            ("resolution_um", "Resolution:", self._res_spin),
            ("pace", "Pace ×:", self._pace_spin),
            ("tol_um", "Confirm tol:", self._tol_spin),
            ("lookahead", "Lookahead:", self._look_spin),
            ("control_hz", "Ctrl rate:", self._hz_spin),
            ("decel", "Decel:", self._decel_spin),
            ("corner_angle", "Corner °:", self._corner_angle_spin),
            ("corner_factor", "Corner speed:", self._corner_fac_spin),
            ("kp", "PID Kp:", self._kp_spin),
            ("kd", "PID Kd:", self._kd_spin),
        ]
        for i, (name, label, spin) in enumerate(rows):
            pg.addWidget(self._param_row(name, label, spin), i // 3, i % 3)
        root.addLayout(pg)

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
        row = QHBoxLayout()
        self._run_btn = QPushButton("Run")
        self._run_btn.setToolTip("Drive the shape in the selected mode; overlay "
                                 "actual vs ideal + report deviation.")
        self._run_btn.clicked.connect(lambda: self._launch("run"))
        self._cmp_btn = QPushButton("Compare all modes")
        self._cmp_btn.clicked.connect(lambda: self._launch("compare"))
        self._tune_btn = QPushButton("Auto-tune (descent)")
        self._tune_btn.setToolTip("Coordinate-descent over the selected mode's "
                                  "key parameters; keep the lowest-deviation set.")
        self._tune_btn.clicked.connect(lambda: self._launch("tune"))
        self._zn_btn = QPushButton("Auto-tune PID (ZN)")
        self._zn_btn.setToolTip("[velocity] Ziegler–Nichols relay test → cross-"
                                "track PID gains.")
        self._zn_btn.clicked.connect(lambda: self._launch("zn"))
        self._robust_btn = QPushButton("Robustness panel")
        self._robust_btn.setToolTip("[velocity] Sweep shapes × sizes × speeds; "
                                    "PASS/FAIL each vs the resolution element.")
        self._robust_btn.clicked.connect(lambda: self._launch("robust"))
        self._save_btn = QPushButton("Save tuned params")
        self._save_btn.setToolTip("Persist the current parameters so the real "
                                  "print uses them.")
        self._save_btn.clicked.connect(self._save_params_to_store)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(lambda: self._stop.set())
        for b in (self._run_btn, self._cmp_btn, self._tune_btn, self._zn_btn,
                  self._robust_btn, self._save_btn, self._stop_btn):
            row.addWidget(b)
        row.addStretch(1)
        root.addLayout(row)

        self._status = QLabel("")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        root.addWidget(self._status)
        self.resize(s(940), s(680))
        self._refresh_hint()

    def _refresh_hint(self):
        name = self._shape_combo.currentData()
        self._hint.setText(XC.SHAPE_HINTS.get(name, ""))

    def _refresh_param_visibility(self, *_):
        mode = self._mode_combo.currentData()
        shown = set(MODE_PARAMS.get(mode, []))
        for name, row in self._param_rows.items():
            row.setVisible(name in shown)
        # ZN + robustness only meaningful for the closed-loop velocity mode
        is_vel = (mode == "velocity")
        self._zn_btn.setEnabled(is_vel and not self._busy())
        self._robust_btn.setEnabled(is_vel and not self._busy())

    def _append_log(self, msg):
        self._log.appendPlainText(msg)

    # ── params ↔ store ────────────────────────────────────────────
    def _load_params_from_store(self):
        ol = self._store.get_mode_params("open_loop")
        cf = self._store.get_mode_params("confirm")
        ve = self._store.get_mode_params("velocity")
        self._pace_spin.setValue(ol.get("pace_correction", 1.0))
        self._tol_spin.setValue(cf.get("settle_tol_um", 40.0))
        self._look_spin.setValue(ve.get("lookahead_mm", 0.6))
        self._hz_spin.setValue(ve.get("control_hz", 25.0))
        self._decel_spin.setValue(ve.get("decel_mm", 1.5))
        self._corner_angle_spin.setValue(ve.get("corner_angle_deg", 30.0))
        self._corner_fac_spin.setValue(ve.get("corner_speed_factor", 0.4))
        self._kp_spin.setValue(ve.get("pid_kp", 0.0))
        self._kd_spin.setValue(ve.get("pid_kd", 0.0))
        try:
            self._res_spin.setValue(self._store.get_resolution_element_um())
        except Exception:
            pass

    def _save_params_to_store(self):
        self._store.set_mode_params(
            "open_loop", {"pace_correction": self._pace_spin.value()})
        self._store.set_mode_params("confirm", {
            "settle_tol_um": self._tol_spin.value(),
            "corner_angle_deg": self._corner_angle_spin.value()})
        self._store.set_mode_params("velocity", {
            "lookahead_mm": self._look_spin.value(),
            "control_hz": self._hz_spin.value(),
            "decel_mm": self._decel_spin.value(),
            "corner_angle_deg": self._corner_angle_spin.value(),
            "corner_speed_factor": self._corner_fac_spin.value(),
            "pid_kp": self._kp_spin.value(),
            "pid_kd": self._kd_spin.value()})
        try:
            self._store.set_resolution_element_um(self._res_spin.value())
        except Exception:
            pass
        self._status.setText("Saved tuned params — the real print will use them.")

    def _mode_params(self, mode):
        return {
            "speed": self._speed_spin.value(),
            "resolution_um": self._res_spin.value(),
            "pace": self._pace_spin.value(),
            "tol_um": self._tol_spin.value(),
            "lookahead": self._look_spin.value(),
            "control_hz": self._hz_spin.value(),
            "decel": self._decel_spin.value(),
            "corner_angle": self._corner_angle_spin.value(),
            "corner_factor": self._corner_fac_spin.value(),
            "kp": self._kp_spin.value(),
            "kd": self._kd_spin.value(),
        }

    def _apply_tuned(self, store_param, val):
        """Reflect a tuned value into its spin (GUI thread via the apply signal)."""
        try:
            {
                "pace_correction": self._pace_spin,
                "settle_tol_um": self._tol_spin,
                "corner_angle_deg": self._corner_angle_spin,
                "lookahead_mm": self._look_spin,
                "corner_speed_factor": self._corner_fac_spin,
                "pid_kp": self._kp_spin,
                "pid_kd": self._kd_spin,
                "control_hz": self._hz_spin,
                "decel_mm": self._decel_spin,
            }[store_param].setValue(val)
        except Exception:
            pass

    # ── launch / gate ─────────────────────────────────────────────
    def _busy(self):
        return self._thread is not None and self._thread.is_alive()

    def _launch(self, action):
        if self._busy():
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
        target = {"run": self._worker_run, "compare": self._worker_compare,
                  "tune": self._worker_tune, "zn": self._worker_zn,
                  "robust": self._worker_robustness}[action]
        self._thread = threading.Thread(target=target, name="XYChallenge",
                                        daemon=True)
        self._thread.start()

    def _set_running(self, on):
        for b in (self._run_btn, self._cmp_btn, self._tune_btn, self._save_btn):
            b.setEnabled(not on)
        is_vel = (self._mode_combo.currentData() == "velocity")
        self._zn_btn.setEnabled(is_vel and not on)
        self._robust_btn.setEnabled(is_vel and not on)
        self._stop_btn.setEnabled(on)

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
            return VC.resolve_control(
                print_speed_mm_s=speed, lookahead_mm=lookahead,
                xy_max_speed_um_s=self._store.get_xy_max_speed_um_s() or 0.0,
                control_loop_ms=self._store.get_control_loop_ms() or 0.0,
                phase_lag_s=self._store.get_phase_lag_s() or 0.0,
                default_control_hz=default_hz,
                fallback_max_um_s=self._max_um_s())
        except Exception:
            return {"control_hz": default_hz, "max_um_s": self._max_um_s(),
                    "speed_cap_mm_s": speed}

    def _set_sms(self, max_um):
        xy = getattr(self._ctrl, "xy_stage", None)
        if xy is not None:
            try:
                if hasattr(xy, "set_acceleration"):
                    xy.set_acceleration(80)
                if hasattr(xy, "set_speed_mm_s"):
                    xy.set_speed_mm_s(max_um / 1000.0)
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
        self._set_sms(max_um)
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

    def _worker_zn(self):
        """Ziegler–Nichols relay tune of the velocity cross-track PID gains.

        Drive a straight test line at low speed with pure-pursuit forward motion
        plus a perpendicular RELAY velocity ±d that flips on the sign of the
        cross-track error → a limit cycle. Measure its period Tu and amplitude a,
        then Ku = 4d/(πa) (VelocityControl.relay_ultimate_gain) → ZN gains.
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
            self._bridge.status.emit("ZN relay: inducing oscillation…")
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
                # relay: perpendicular velocity opposing the cross sign
                sign = 1 if cross > 0 else (-1 if cross < 0 else prev_sign)
                if prev_sign != 0 and sign != prev_sign:
                    flips.append(tick)
                    amps.append(cur_amp)
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
            # estimate Tu (2× mean half-period) + amplitude a
            if len(flips) >= 4 and len(amps) >= 3:
                halfs = [flips[i + 1] - flips[i] for i in range(len(flips) - 1)]
                tu = 2.0 * (sum(halfs) / len(halfs))
                a = sum(amps[1:]) / len(amps[1:])   # skip the first partial
                ku, tu = VC.relay_ultimate_gain(relay_d, a, tu)
                gains = compute_zn_pid_gains(ku, tu, method="some_overshoot")
                kp = round(max(0.0, gains["kp"]), 3)
                kd = round(max(0.0, gains["kd"]), 4)
                self._store.set_mode_params("velocity",
                                            {"pid_kp": kp, "pid_kd": kd})
                self._bridge.apply.emit("pid_kp", kp)
                self._bridge.apply.emit("pid_kd", kd)
                self._bridge.log.emit(
                    f"  Ku={ku:.2f} Tu={tu:.2f}s → Kp={kp} Kd={kd}")
                self._bridge.finished.emit(
                    True, f"ZN: Kp={kp} Kd={kd} (saved).")
            else:
                self._bridge.finished.emit(
                    False, "ZN: not enough oscillation — raise speed / relay.")
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
