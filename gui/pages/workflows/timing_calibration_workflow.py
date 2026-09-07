"""timing_calibration_workflow.py — segment settle-delay sweep.

Goal: measure how far the XY (Prior) stage physically LAGS the issued move
commands as a function of how many segments are streamed back-to-back — i.e.
the controller's phase lag building up over a print's command stream.

Method (deliberately simple + robust — NO optical flow):
  • Issue a straight LINE of N collinear segment moves back-to-back, paced like
    a real print (one move per segment, sleep ≈ segment time between them).
  • Watch the camera frames with a plain FRAME-DIFFERENCE test ("do two frames
    look the same?"). When the frames stop changing, the stage has physically
    stopped. Log the DELAY from the last command to stillness.
  • Sweep N = 1, 2, 3, … : with one segment the delay is just the single-move
    settle; as N grows the open-loop command stream outruns the stage and a
    backlog accumulates, so the delay GROWS. The slope (delay per added
    segment) is the per-segment phase lag — what must be accounted for to keep
    the pump synced to the needle. (A single jog-speed scale cannot fix this;
    the slope tells you the per-segment timing to allocate.)

Outputs: a live frame-change strip (watch the frames go still), a delay-vs-N
result plot with the fitted phase-lag line, and a stored per-speed model
(intercept = base settle, slope = lag per segment) refined across runs. The
needle stays RETRACTED at the Safe Z the whole time (pure XY-timing
measurement; it never descends).
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QSpinBox, QDoubleSpinBox, QFrame, QSizePolicy, QPlainTextEdit, QSplitter,
    QComboBox, QProgressBar,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.section_stack import (
    PromotedSectionsPanel, wire_section_promotion)
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.camera_feed_view import CameraFeedView
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)

from SupportClasses.PrintTimingCalibrationStore import get_store
from SupportClasses import XYCalibrationRun as _CR
from SupportClasses import XYTopSpeed as TS

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover
    cv2 = None
    np = None

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:  # pragma: no cover
    CameraRole = None

logger = logging.getLogger(__name__)


def _mmss(seconds: float) -> str:
    """m:ss for the run clock. Negative/absent reads as 0:00."""
    t = max(0, int(round(float(seconds or 0.0))))
    return "%d:%02d" % (t // 60, t % 60)

_LOG_DIR = Path("logs/timing")


class _FrameMotion:
    """Frame-difference motion detector — answers 'are the frames changing?'
    via the mean absolute pixel difference of consecutive (downscaled, blurred)
    grayscale frames. No optical flow, no direction — just change vs. still."""

    def __init__(self, camera_manager, cam_idx: int, downscale_w: int = 160):
        self._mgr = camera_manager
        self._cam = cam_idx
        self._w = downscale_w
        self._prev = None

    def available(self) -> bool:
        return (cv2 is not None and np is not None
                and self._mgr is not None and self._cam is not None)

    def reset(self) -> None:
        self._prev = None

    def _gray(self, frame):
        h, w = frame.shape[:2]
        if w > self._w:
            scale = self._w / float(w)
            frame = cv2.resize(frame, (self._w, max(1, int(h * scale))))
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        try:
            return cv2.GaussianBlur(g, (5, 5), 0)   # kill per-pixel sensor noise
        except Exception:
            return g

    def metric(self):
        """Mean abs frame-to-frame difference, or None if no/!ready frame."""
        if not self.available():
            return None
        try:
            frame = self._mgr.get_current_frame(self._cam)
        except Exception:
            return None
        if frame is None or getattr(frame, "size", 0) == 0:
            return None
        g = self._gray(frame)
        if self._prev is None or g.shape != self._prev.shape:
            self._prev = g
            return None
        # Skip DUPLICATE frames (polled faster than the camera delivers): an
        # identical frame diffs to exactly 0 and would masquerade as 'still'
        # mid-motion. Only compare genuinely-new frames (keep _prev), so every
        # returned value is a real inter-frame difference.
        if np.array_equal(g, self._prev):
            return None
        d = float(np.mean(cv2.absdiff(g, self._prev)))
        self._prev = g
        return d

    def frame_count(self):
        try:
            cams = getattr(self._mgr, "_cameras", None)
            if cams and 0 <= self._cam < len(cams):
                cam = cams[self._cam]
                if hasattr(cam, "frame_count_value"):
                    return int(cam.frame_count_value())
        except Exception:
            pass
        return None


class _EncoderMotion:
    """Stillness detector driven by the STAGE'S OWN reported position — the
    CAMERA-FREE detector. Drop-in for ``_FrameMotion`` (same available/reset/
    metric interface), so every measurement (top speed, settle sweep) works with
    no microscope.

    ``metric()`` = |Δposition| in µm since the previous call. At rest the Prior's
    reported position is constant (≈ 0 µm/poll); while moving it changes by
    hundreds of µm/poll — a far cleaner still/moving signal than frame
    differencing, and immune to focus/texture/lighting. Each call is one direct
    ``get_xy_position(cached=False)`` query; the position poller is suspended for
    the duration of a run, so these queries are uncontended.
    """

    # Physical still/moving thresholds for the position-delta metric (µm per
    # poll). The metric is ABSOLUTE distance, so these are fixed — unlike the
    # camera's arbitrary units, no scene calibration is needed (and the camera's
    # scene calibration is exactly what made the settle sweep fail: on a short
    # segment its 'peak' caught one poll spanning the whole move and set the
    # threshold too high, so the sweep's smaller per-poll motions never cleared
    # it and every measurement timed out). At any real jog/print speed a poll
    # moves tens–hundreds of µm, while at-rest jitter is ≤ a couple of counts, so
    # a 3 µm still-threshold separates them with huge margin.
    STILL_UM = 3.0
    MOTION_UM = 60.0     # nominal "clearly moving" peak (display/log only)

    def __init__(self, controller):
        self._ctrl = controller
        self._prev = None

    def fixed_threshold(self):
        """(floor, peak, still_thresh) in the metric's µm units — used INSTEAD of
        the scene-relative ``_calibrate_threshold`` for this physical detector."""
        return (0.0, self.MOTION_UM, self.STILL_UM)

    def available(self) -> bool:
        c = self._ctrl
        return (c is not None and getattr(c, "is_xy_connected", False)
                and hasattr(c, "get_xy_position"))

    def reset(self) -> None:
        self._prev = None

    def _read(self):
        try:
            p = self._ctrl.get_xy_position(cached=False)
        except Exception:
            return None
        if not p or p[0] is None or p[1] is None:
            return None
        return (float(p[0]), float(p[1]))

    def metric(self):
        p = self._read()
        if p is None:
            return None
        if self._prev is None:      # first sample after reset — no delta yet
            self._prev = p
            return None
        d = math.hypot(p[0] - self._prev[0], p[1] - self._prev[1])
        self._prev = p
        return d

    def frame_count(self):
        return None


@dataclass
class _TimingConfig:
    # v7.21.2: `max_segments` / `seg_len_mm` described the settle-delay sweep,
    # which is gone. They keep defaults rather than being deleted so the legacy
    # top-speed worker and its tests still construct a config unchanged.
    speed_mm_s: float
    repeats: int
    still_window_s: float
    top_max_dist_mm: float = 10.0
    max_segments: int = 8
    seg_len_mm: float = 1.0


# ── Live frame-change strip ───────────────────────────────────────────

class _MotionStrip(QWidget):
    """Live strip-chart of the frame-change value over time, with markers for
    each 'last command issued' (peach) and 'frames went still' (green). Lets
    the operator watch the frames stop changing."""

    def __init__(self, window_s: float = 14.0, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(120))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._window_s = window_s
        self._pts: deque = deque()       # (t, change)
        self._marks: deque = deque()     # (t, kind)
        self._thresh = None
        self._t_latest = 0.0

    def clear(self) -> None:
        self._pts.clear()
        self._marks.clear()
        self._t_latest = 0.0
        self.update()

    def set_threshold(self, thr: float) -> None:
        self._thresh = thr
        self.update()

    def add(self, t: float, change: float) -> None:
        self._t_latest = max(self._t_latest, t)
        self._pts.append((t, change))
        self._trim(self._pts)
        self.update()

    def mark(self, t: float, kind: str) -> None:
        self._marks.append((t, kind))
        self._trim(self._marks)
        self.update()

    def _trim(self, dq) -> None:
        tmin = self._t_latest - self._window_s
        while dq and dq[0][0] < tmin:
            dq.popleft()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(COLORS.get("mantle", "#181825")))
        pad = s(6)
        x0, y0 = pad, pad
        pw, ph = max(1, w - 2 * pad), max(1, h - 2 * pad)

        if not self._pts:
            p.setPen(QColor(COLORS.get("overlay0", "#6c7086")))
            p.drawText(self.rect(), Qt.AlignCenter,
                       "frame change vs time — run to populate")
            p.end()
            return

        t_hi = self._t_latest
        t_lo = t_hi - self._window_s
        span = max(t_hi - t_lo, 1e-3)
        ymax = max(max(c for _, c in self._pts), 1e-3) * 1.15

        def _x(t):
            return x0 + (t - t_lo) / span * pw

        def _y(c):
            return y0 + ph - (c / ymax) * ph

        # threshold line
        if self._thresh is not None:
            p.setPen(QPen(QColor(COLORS.get("surface2", "#585b70")), 1, Qt.DashLine))
            yy = _y(self._thresh)
            p.drawLine(int(x0), int(yy), int(x0 + pw), int(yy))

        # markers
        for (t, kind) in self._marks:
            if t < t_lo:
                continue
            col = COLORS.get("green", "#a6e3a1") if kind == "still" \
                else COLORS.get("peach", "#fab387")
            p.setPen(QPen(QColor(col), 1, Qt.DotLine))
            xx = _x(t)
            p.drawLine(int(xx), int(y0), int(xx), int(y0 + ph))

        # change trace
        p.setPen(QPen(QColor(COLORS.get("blue", "#89b4fa")), s(2)))
        prev = None
        for (t, c) in self._pts:
            pt = QPointF(_x(t), _y(c))
            if prev is not None:
                p.drawLine(prev, pt)
            prev = pt

        p.setFont(QFont("", max(7, int(sf(8)))))
        p.setPen(QColor(COLORS.get("subtext0", "#a6adc8")))
        p.drawText(int(x0 + s(4)), int(y0 + s(12)), "frame change (still ⇢ flat)")
        p.end()


# ── Delay-vs-N result plot ────────────────────────────────────────────

class _DelayPlot(QWidget):
    """Result: settle delay (ms) vs number of streamed segments, with the fitted
    phase-lag line. The slope is the per-segment lag."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setMinimumHeight(s(120))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pts: list = []             # (x, y)
        self._slope = 0.0
        self._intercept = 0.0
        self._headline = ""
        self._xlabel = "segments →"
        self._ylabel = "delay (ms)"

    def clear(self) -> None:
        self._pts = []
        self._slope = 0.0
        self._intercept = 0.0
        self._headline = ""
        self.update()

    def set_result(self, points: list, slope: float, intercept: float,
                   headline: str = "", xlabel: str = "segments →",
                   ylabel: str = "delay (ms)") -> None:
        self._pts = sorted(points)
        self._slope = slope
        self._intercept = intercept
        self._headline = headline or (
            f"phase lag {slope * 1000:.1f} ms/segment "
            f"(base {intercept * 1000:.0f} ms)")
        self._xlabel = xlabel
        self._ylabel = ylabel
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(COLORS.get("mantle", "#181825")))
        pad = s(22)
        x0, y0 = pad, s(6)
        pw, ph = max(1, w - pad - s(8)), max(1, h - pad - s(6))

        if not self._pts:
            p.setPen(QColor(COLORS.get("overlay0", "#6c7086")))
            p.drawText(self.rect(), Qt.AlignCenter,
                       "delay vs segment count — result after the run")
            p.end()
            return

        n_max = max(n for n, _ in self._pts)
        d_max = max(max(d for _, d in self._pts),
                    self._intercept + self._slope * n_max, 1e-4) * 1.15

        def _x(n):
            return x0 + (n / max(n_max, 1)) * pw

        def _y(d):
            return y0 + ph - (d / d_max) * ph

        # axes
        p.setPen(QPen(QColor(COLORS.get("surface1", "#45475a")), 1))
        p.drawLine(int(x0), int(y0), int(x0), int(y0 + ph))
        p.drawLine(int(x0), int(y0 + ph), int(x0 + pw), int(y0 + ph))

        # fit line
        p.setPen(QPen(QColor(COLORS.get("peach", "#fab387")), s(2)))
        p.drawLine(QPointF(_x(0), _y(self._intercept)),
                   QPointF(_x(n_max), _y(self._intercept + self._slope * n_max)))

        # points
        p.setPen(QPen(QColor(COLORS.get("blue", "#89b4fa")), 1))
        p.setBrush(QColor(COLORS.get("blue", "#89b4fa")))
        for (n, d) in self._pts:
            p.drawEllipse(QPointF(_x(n), _y(d)), s(3), s(3))

        p.setFont(QFont("", max(7, int(sf(8)))))
        p.setPen(QColor(COLORS.get("text", "#cdd6f4")))
        p.drawText(int(x0 + s(2)), int(y0 + s(11)), self._headline)
        p.setPen(QColor(COLORS.get("overlay0", "#6c7086")))
        p.drawText(int(x0 + pw - s(80)), int(y0 + ph + s(14)), self._xlabel)
        p.save()
        p.translate(int(s(10)), int(y0 + ph))
        p.rotate(-90)
        p.drawText(0, 0, self._ylabel)
        p.restore()
        p.end()


class _TimingBridge(QObject):
    """Worker-thread → GUI-thread relay (queued)."""

    log_line = Signal(str)
    stats = Signal(dict)
    finished = Signal(bool, str)
    motion = Signal(float, float)        # t_rel, change
    marker = Signal(float, str)          # t_rel, kind ('cmd' | 'still')
    result = Signal(dict)                # points, slope, intercept, labels
    # v7.21.2: one StepProgress per phase boundary of the single calibration.
    step = Signal(object)


class TimingCalibrationWorkflowPage(QWidget):
    """Segment settle-delay sweep — measures the controller's per-segment phase
    lag via frame-difference 'when did the stage stop' timing. Also measures the
    stage's true top speed (distance sweep at full SMS) to fix the mm/s↔SMS
    conversion. The start location is selectable (plate centre or the current
    jogged position)."""

    back_requested = Signal()

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        self._plate = None
        self._well_positions = None
        self._safe_z: float | None = None
        self._z_references: dict = {}

        self._context_widget: StandardJogContextPanel | None = None
        self._cam_started_by_us = False
        self._cam_idx_started: int | None = None
        self._cam_view_started_by_us = False
        self._live_t0: float | None = None
        self._start_xy_mm: tuple | None = None    # selected start (zero-ref mm)

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._bridge = _TimingBridge()
        self._bridge.log_line.connect(self._append_log)
        self._bridge.stats.connect(self._update_stats)
        self._bridge.finished.connect(self._on_finished)
        self._bridge.motion.connect(self._on_motion)
        self._bridge.marker.connect(self._on_marker)
        self._bridge.result.connect(self._on_result)
        self._bridge.step.connect(self._on_step)

        # Comprehensive settings popout (scrollable, saveable timing presets).
        self._settings_dialog = WorkflowSettingsDialog(
            "timing_calibration", "XY↔ZP Timing Calibration",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))
        outer.addLayout(self._build_header())
        outer.addWidget(self._build_main_area(), stretch=1)
        outer.addWidget(self._build_run_row())
        # v7.21: a section moved out of ⚙ Settings lands in the drawer, which is
        # hidden (zero footprint) until something is in it. AFTER the run row on
        # purpose, so Start / Abort never move.
        self._promoted_panel = PromotedSectionsPanel()
        outer.addWidget(self._promoted_panel)
        self._layout_store = wire_section_promotion(
            self, self._settings_dialog, self._promoted_panel.stack,
            settings=self._settings, workflow_id="timing_calibration")
        self._update_button_state()
        self._settings_dialog.load_last()
        self._update_settings_summary()

    # ── UI ────────────────────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)
        title = QLabel("XY↔ZP Timing Calibration — segment settle delay")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        row.addWidget(title)
        self._settings_summary = QLabel("")
        self._settings_summary.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._settings_summary)
        row.addStretch(1)
        settings_btn = QPushButton("⚙ Settings")
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.setToolTip(
            "Open the full, saveable timing-calibration configuration "
            "(sweep, top-speed, start location).")
        settings_btn.clicked.connect(self._open_settings)
        row.addWidget(settings_btn)
        return row

    # ── Settings popout ───────────────────────────────────────────

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        self._update_button_state()
        self._update_settings_summary()

    def _update_settings_summary(self):
        """v7.21.2: summarise the CALIBRATION TARGET.

        This used to read ``_spin_maxn``/``_spin_seg`` inside a bare ``except:
        pass`` — with the settle sweep gone those attributes no longer exist, and
        the swallowed AttributeError would have left the summary permanently blank
        with nothing to explain why.
        """
        if not hasattr(self, "_settings_summary"):
            return
        try:
            n = _CR.GRID_LEVELS.get(self._grid_level(), 7)
            self._settings_summary.setText(
                f"≤{self._spin_speed.value():g} mm/s · "
                f"{self._spin_res_um.value():g} µm element · "
                f"{self._spin_feature_mm.value():g} mm star · grid {n}×{n}")
        except Exception as e:
            logger.debug("settings summary refresh failed: %s", e)

    @staticmethod
    def _dspin(lo, hi, val, suffix="", decimals=2, step=None, tip=""):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setDecimals(decimals)
        if suffix:
            sb.setSuffix(suffix)
        sb.setValue(val)
        if step is not None:
            sb.setSingleStep(step)
        if tip:
            sb.setToolTip(tip)
        return sb

    @staticmethod
    def _ispin(lo, hi, val, tip=""):
        sb = QSpinBox()
        sb.setRange(lo, hi)
        sb.setValue(val)
        if tip:
            sb.setToolTip(tip)
        return sb

    def _build_settings_dialog(self, dlg: WorkflowSettingsDialog):
        # ── What the calibration is aiming at ──
        # v7.21.2: these replace the settle-sweep knobs. `_spin_speed` keeps its
        # attribute name (several tests and the settings profile reference it) but
        # has a real new job: it is the target speed the derivation and the grid
        # aim for — a CEILING, since the achievable speed is an output of the run.
        self._spin_speed = self._dspin(0.1, 50.0, 5.0, " mm/s", 2, 0.5)
        self._spin_speed.setToolTip(
            "The fastest you want to print. The calibration reports what this "
            "machine can actually hold on the test feature, which is usually "
            "lower — the dead time and the feature's corners set the real limit.")
        self._spin_speed.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        self._spin_res_um = self._dspin(
            1.0, 500.0, 30.0, " µm", 0, 5.0,
            "Resolution element — how far the printed path may stray before a "
            "tuning counts as failing. Near this machine's physical floor there "
            "is nothing to gain by asking for less.")
        self._spin_res_um.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        self._spin_feature_mm = self._dspin(
            0.5, 20.0, 2.0, " mm", 1, 0.5,
            "Size of the test star. Small features are the hard case: their "
            "corners demand a short lookahead, which is what limits the speed.")
        self._spin_feature_mm.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        sec = dlg.add_section("Calibration target")
        sec.add("speed", "Target print speed", self._spin_speed, 5.0)
        sec.add("res_um", "Resolution element", self._spin_res_um, 30.0)
        sec.add("feature_mm", "Test feature size", self._spin_feature_mm, 2.0)

        # ── Probes ──
        self._spin_reps = self._ispin(
            1, 10, 2, "Timed moves per distance in the top-speed sweep.")
        self._spin_still = self._dspin(
            0.1, 2.0, 0.35, " s", 2, 0.05,
            "Motion must stay below threshold this long to count as 'stopped'.")
        self._spin_topdist = self._dspin(
            1.0, 50.0, 10.0, " mm", 1, 1.0,
            "The top-speed sweep moves distances up to this at full speed; the "
            "time-vs-distance slope = 1/top-speed. Bigger = more accurate, and "
            "it is shrunk automatically to stay clear of the travel limits.")
        self._detector_combo = QComboBox()
        self._detector_combo.addItem("Stage position (no camera)", "encoder")
        self._detector_combo.addItem("Microscope camera", "camera")
        self._detector_combo.setToolTip(
            "How stage motion / stillness is measured.\n"
            "Stage position = poll the Prior's own reported position (camera-"
            "free; rest ≈ 0 µm, motion ≈ hundreds of µm per poll).\n"
            "Microscope camera = frame-difference (needs a well-textured, "
            "in-focus, well-lit view).")
        self._detector_combo.setMinimumWidth(s(170))
        sec = dlg.add_section("Probes")
        sec.add("topdist", "Top-speed max distance", self._spin_topdist, 10.0)
        sec.add("reps", "Repeats per distance", self._spin_reps, 2)
        sec.add("still", "Still window", self._spin_still, 0.35)
        sec.add_widget(self._detector_combo)

        # ── Diagnostics ──
        # The bench moved off the run row but must stay REACHABLE, or
        # `_open_challenge` becomes dead code and the only manual path-following
        # tool in the app is unreachable.
        sec = dlg.add_section("Diagnostics")
        self._challenge_btn = QPushButton("XY Printing Challenge…")
        self._challenge_btn.setToolTip(
            "Bench: drive challenge shapes manually and compare actual vs ideal "
            "path. Diagnostic only — the calibration does the tuning.")
        self._challenge_btn.clicked.connect(self._open_challenge)
        sec.add_widget(self._challenge_btn)

        # ── Start location ──
        sec = dlg.add_section("Start location")
        start_row = QWidget()
        srl = QHBoxLayout(start_row)
        srl.setContentsMargins(0, 0, 0, 0)
        srl.setSpacing(s(8))
        srl.addWidget(QLabel("Start:"))
        self._start_label = QLabel("plate centre")
        self._start_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        srl.addWidget(self._start_label)
        srl.addStretch(1)
        btn_here = QPushButton("Use current position")
        btn_here.setToolTip(
            "Jog the stage (left context panel) to where you want the calibration "
            "to run — somewhere the microscope sees texture with room along +X — "
            "then capture it here.")
        btn_here.clicked.connect(self._capture_start)
        srl.addWidget(btn_here)
        btn_centre = QPushButton("Plate centre")
        btn_centre.clicked.connect(self._reset_start)
        srl.addWidget(btn_centre)
        sec.add_widget(start_row)
        sec.add_note(
            "Needle stays retracted; the microscope (optical) is REQUIRED — it "
            "detects, by frame difference, when the stage has stopped. The sweep "
            "moves along +X from the start, so leave envelope room there.")

        # ── Locations & Hardware (read-only) ──
        dlg.add_info_section()
        dlg.set_info_refresher(lambda: build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z))
        dlg.finalize()

    # ── start location ────────────────────────────────────────────

    def _capture_start(self) -> None:
        ctrl = self._controller
        try:
            pos = ctrl.get_xy_position(cached=True)
            zero = ctrl.zero_position
            if pos and pos[0] is not None and pos[1] is not None:
                self._start_xy_mm = ((pos[0] - zero["x"]) / 1000.0,
                                     (pos[1] - zero["y"]) / 1000.0)
                self._update_start_label()
                self._status.setText("Start set to current stage position.")
            else:
                self._status.setText("Couldn't read the stage position.")
        except Exception as e:
            self._status.setText(f"Couldn't read position: {e}")

    def _reset_start(self) -> None:
        self._start_xy_mm = None
        self._update_start_label()
        self._status.setText("Start reset to plate centre.")

    def _update_start_label(self) -> None:
        if self._start_xy_mm is None:
            self._start_label.setText("plate centre")
        else:
            self._start_label.setText(
                f"({self._start_xy_mm[0]:.2f}, {self._start_xy_mm[1]:.2f}) mm")

    def _resolve_start_mm(self):
        return (self._start_xy_mm if self._start_xy_mm is not None
                else self._print_center_mm())

    def _detector_mode(self) -> str:
        """'encoder' (stage-position, camera-free — default) or 'camera'."""
        w = getattr(self, "_detector_combo", None)
        try:
            return w.currentData() or "encoder"
        except Exception:
            return "encoder"

    def _make_tracker(self):
        """The stillness detector for this run — the stage's own position
        (camera-free) or the microscope, per the Detector selector."""
        if self._detector_mode() == "camera":
            return _FrameMotion(self._camera_manager, self._optical_cam_idx)
        return _EncoderMotion(self._controller)

    def _build_main_area(self) -> QWidget:
        split = QSplitter(Qt.Horizontal)
        left = QSplitter(Qt.Vertical)
        try:
            self._camera_view = CameraFeedView(
                self._camera_manager,
                cam_idx=self._resolve_microscope_cam_idx(),
                label="Microscope — frame-difference still detection",
                auto_orient=True,   # v7.5.x: calibrated orientation
                enable_settings=False)
        except Exception as e:
            logger.debug("Timing calibration camera view unavailable: %s", e)
            self._camera_view = None
        if self._camera_view is not None:
            left.addWidget(self._camera_view)
        self._strip = _MotionStrip()
        left.addWidget(self._strip)
        left.setSizes([s(240), s(150)])

        right = QSplitter(Qt.Vertical)
        self._delay_plot = _DelayPlot()
        right.addWidget(self._delay_plot)
        right.addWidget(self._build_monitor())
        right.setSizes([s(180), s(240)])

        split.addWidget(left)
        split.addWidget(right)
        split.setSizes([s(440), s(500)])
        return split

    def _build_monitor(self) -> QWidget:
        card = Card("Monitor — sweep progress · event log", flush=True,
                    compact=True)
        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(s(8), s(6), s(8), s(8))
        v.setSpacing(s(8))

        self._banner = QLabel("Idle — configure and press Start.")
        self._banner.setAlignment(Qt.AlignCenter)
        self._banner.setStyleSheet(
            f"color: {COLORS['subtext0']}; background-color: {COLORS['surface0']};"
            f" border-radius: 6px; padding: {s(6)}px; font-size: {sf(11)}pt;"
            f" font-weight: 600;")
        v.addWidget(self._banner)

        stat = QGridLayout()
        stat.setHorizontalSpacing(s(16))
        stat.setVerticalSpacing(s(4))
        self._stat_labels: dict[str, QLabel] = {}
        cells = [("seg", "Segments (N)"), ("delay", "Settle delay (ms)"),
                 ("change", "Frame change"), ("thresh", "Still threshold")]
        for i, (key, title) in enumerate(cells):
            r, c = divmod(i, 4)
            box = QVBoxLayout()
            t = QLabel(title)
            t.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: {sf(8)}pt;")
            val = QLabel("—")
            val.setStyleSheet(
                f"color: {COLORS['text']}; font-size: {sf(12)}pt;"
                f" font-weight: 600;")
            box.addWidget(t)
            box.addWidget(val)
            stat.addLayout(box, r, c)
            self._stat_labels[key] = val
        v.addLayout(stat)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(4000)
        self._log.setStyleSheet(
            f"QPlainTextEdit {{ background-color: {COLORS['mantle']};"
            f" color: {COLORS['subtext0']}; border: 1px solid "
            f"{COLORS['surface1']}; border-radius: 6px; font-family: "
            f"'Cascadia Code','Consolas',monospace; font-size: {sf(9)}pt; }}")
        v.addWidget(self._log, stretch=1)
        card.add_widget(body)
        return card

    def _build_run_row(self) -> QFrame:
        """ONE action.

        v7.21.2: this row used to carry four probe buttons — Start sweep, Measure
        top speed, Check comms rate — plus the Challenge bench, whose ORDER was
        load-bearing and undocumented (every later one consumes an earlier
        measurement). The whole sequence is now a single button; the detector, the
        calibration targets and the bench moved into the ⚙ Settings popout, which
        is where a rarely-changed choice belongs.
        """
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))

        row.addWidget(QLabel("Grid:"))
        self._grid_combo = QComboBox()
        for key in _CR.GRID_LEVEL_ORDER:
            self._grid_combo.addItem(_CR.GRID_LEVEL_LABELS[key], key)
        self._grid_combo.setCurrentIndex(_CR.GRID_LEVEL_ORDER.index("medium"))
        self._grid_combo.setToolTip(
            "How finely the tuning grid is searched (lookahead x corner speed).\n"
            "Candidates are scored in SIMULATION against this machine's measured "
            "dynamics, so even the finest grid costs seconds, not minutes — the "
            "coarseness trades how precisely the fastest usable lookahead is "
            "located, not how long the stage moves.")
        self._grid_combo.setMinimumWidth(s(180))
        row.addWidget(self._grid_combo)

        self._run_btn = QPushButton("Run XY Calibration")
        self._run_btn.setToolTip(
            "Measure this stage and tune the print follower in one pass:\n"
            "  1. comms rate (closed-loop cadence while moving)\n"
            "  2. dead time (command to motion transport delay)\n"
            "  3. top speed (distance sweep at full speed)\n"
            "  4. apply the measured top speed everywhere\n"
            "  5. derive the follower settings in closed form\n"
            "  6. tune on a simulated grid, then verify on the stage\n"
            "The needle stays retracted throughout and the stage is centred "
            "first, so no probe can reach a travel limit. Under three minutes.")
        self._run_btn.clicked.connect(self._on_run_calibration)
        row.addWidget(self._run_btn)

        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        row.addWidget(self._stop_btn)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(False)
        self._progress.setFixedWidth(s(140))
        row.addWidget(self._progress)

        self._step_label = QLabel("")
        self._step_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._step_label.setMinimumWidth(s(160))
        row.addWidget(self._step_label)

        self._clock_label = QLabel("")
        self._clock_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._clock_label.setMinimumWidth(s(150))
        row.addWidget(self._clock_label)

        row.addStretch(1)
        self._status = QLabel("")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)
        return frame

    # ── MainWindow hooks ──────────────────────────────────────────

    def get_page_title(self) -> str:
        return "XY↔ZP Timing Calibration"

    def get_sub_page_title(self) -> str:
        return "XY↔ZP Timing Calibration"

    def get_context_widget(self) -> QWidget:
        if self._context_widget is None:
            self._context_widget = StandardJogContextPanel(
                controller=self._controller, settings=self._settings,
                show_connect=False, bypass_safety=False)
            if self._hw_config is not None:
                self._context_widget.set_hardware_config(self._hw_config)
            if any(v is not None for v in
                   (self._plate, self._well_positions, self._safe_z)):
                self._context_widget.set_calibration_data(
                    self._plate, self._well_positions, self._safe_z)
        return self._context_widget

    def on_status_update(self) -> None:
        if self._context_widget is not None and hasattr(
                self._context_widget, "on_status_update"):
            self._context_widget.on_status_update()
        # The Start/Measure buttons gate on live XY+ZP connection; re-evaluate
        # each tick so they enable as soon as the hardware connects (they were
        # only refreshed on construction / calibration-data pushes before, so a
        # connect made after opening this page left the button stuck greyed).
        self._update_button_state()

    def set_settings(self, settings) -> None:
        self._settings = settings
        if self._context_widget is not None:
            self._context_widget.set_settings(settings)

    def set_hardware_config(self, hw_config) -> None:
        self._hw_config = hw_config
        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, well_positions, safe_z)
        self._update_button_state()

    def set_z_references(self, refs) -> None:
        if isinstance(refs, dict):
            self._z_references = dict(refs)
        if self._context_widget is not None and hasattr(
                self._context_widget, "set_z_references"):
            try:
                self._context_widget.set_z_references(refs)
            except Exception:
                pass

    # ── live camera feed ──────────────────────────────────────────

    def showEvent(self, event):
        super().showEvent(event)
        self._start_live_feed()
        # Re-evaluate button gating on entry — the connection state may have
        # changed since the page was built.
        self._update_button_state()

    def hideEvent(self, event):
        super().hideEvent(event)
        if not self._running():
            self._stop_live_feed()
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass

    def _resolve_microscope_cam_idx(self) -> int:
        if self._hw_config is not None and CameraRole is not None:
            try:
                idx = self._hw_config.camera_for_role(CameraRole.MICROSCOPE)
                if idx is not None:
                    return int(idx)
            except Exception:
                pass
        return 0

    def _start_live_feed(self) -> None:
        if self._camera_manager is None or self._camera_view is None:
            return
        idx = self._resolve_microscope_cam_idx()
        try:
            if self._camera_view.cam_idx != idx:
                self._camera_view.set_camera(idx)
            if not self._camera_manager.is_running(idx):
                self._camera_manager.start(idx)
                self._cam_view_started_by_us = True
        except Exception as e:
            logger.debug("Timing live feed start failed: %s", e)

    def _stop_live_feed(self) -> None:
        if (self._camera_manager is None or self._camera_view is None
                or not self._cam_view_started_by_us):
            return
        try:
            self._camera_manager.stop(self._camera_view.cam_idx)
        except Exception:
            pass
        finally:
            self._cam_view_started_by_us = False

    def _start_microscope_camera(self) -> int | None:
        mgr = self._camera_manager
        if mgr is None:
            return None
        idx = self._resolve_microscope_cam_idx()
        try:
            if not mgr.is_running(idx):
                mgr.start(idx)
                self._cam_started_by_us = True
                self._cam_idx_started = idx
            return idx
        except Exception as e:
            logger.debug("Timing calibration camera start failed: %s", e)
            return None

    def _stop_microscope_camera(self) -> None:
        if (self._camera_manager is None or not self._cam_started_by_us
                or self._cam_idx_started is None):
            return
        try:
            self._camera_manager.stop(self._cam_idx_started)
        except Exception:
            pass
        finally:
            self._cam_started_by_us = False
            self._cam_idx_started = None

    # ── GUI-thread slots ──────────────────────────────────────────

    def _append_log(self, line: str) -> None:
        self._log.appendPlainText(line)

    def _update_stats(self, d: dict) -> None:
        for key, lbl in self._stat_labels.items():
            if key in d:
                lbl.setText(str(d[key]))

    def _rel(self, t: float) -> float:
        if self._live_t0 is None:
            self._live_t0 = t
        return t - self._live_t0

    def _on_motion(self, t_rel: float, change: float) -> None:
        self._strip.add(t_rel, change)
        if "change" in self._stat_labels:
            self._stat_labels["change"].setText(f"{change:.2f}")

    def _on_marker(self, t_rel: float, kind: str) -> None:
        self._strip.mark(t_rel, kind)

    def _on_result(self, d: dict) -> None:
        pts = d.get("points") or []
        self._delay_plot.set_result(
            pts, d.get("slope", 0.0), d.get("intercept", 0.0),
            headline=d.get("headline", ""),
            xlabel=d.get("xlabel", "segments →"),
            ylabel=d.get("ylabel", "delay (ms)"))
        # v7.5.x: a measured XY top speed WRITES the single common XY-max source
        # (so every page inherits it). Runs on the GUI thread (bridge signal).
        top = d.get("xy_top_speed_um_s")
        if top:
            self._apply_measured_xy_top_speed(float(top))

    def _apply_measured_xy_top_speed(self, um_s: float) -> None:
        """Persist a measured XY top speed as THE single common XY max:
        live ``safety_limits.max_xy_speed`` + settings, then broadcast so every
        jog/speed surface re-reads it. The worker already applied it to the live
        XYStage + the timing store; this makes it the canonical safety value too."""
        if um_s <= 0:
            return
        ctrl = self._controller
        sl = getattr(ctrl, "safety_limits", None) if ctrl else None
        if sl is not None:
            try:
                sl.max_xy_speed = float(um_s)
            except Exception as e:
                logger.debug(f"set live max_xy_speed failed: {e}")
        if self._settings is not None:
            try:
                self._settings.set("safety_limits.max_xy_speed", float(um_s))
                self._settings.save()
            except Exception as e:
                logger.debug(f"persist max_xy_speed failed: {e}")
        # Broadcast on the GUI thread → app fans refresh_speed_limits() out.
        if ctrl is not None and hasattr(ctrl, "notify_speed_limits_changed"):
            try:
                ctrl.notify_speed_limits_changed()
            except Exception as e:
                logger.debug(f"notify_speed_limits_changed failed: {e}")

    def _on_finished(self, ok: bool, summary: str) -> None:
        self._thread = None
        self._stop_microscope_camera()
        color = COLORS["green"] if ok else COLORS["red"]
        self._banner.setText(("✅ " if ok else "⚠ ") + summary)
        self._banner.setStyleSheet(
            f"color: {COLORS['base']}; background-color: {color};"
            f" border-radius: 6px; padding: {s(6)}px;"
            f" font-size: {sf(11)}pt; font-weight: 700;")
        self._status.setText("Done.")
        self._update_button_state()

    # ── run / stop ────────────────────────────────────────────────

    def _running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _update_button_state(self, *_):
        running = self._running()
        connected = (getattr(self._controller, "is_xy_connected", False)
                     and getattr(self._controller, "is_zp_connected", False))
        if hasattr(self, "_run_btn"):
            self._run_btn.setEnabled(connected and not running)
        if hasattr(self, "_start_btn"):
            self._start_btn.setEnabled(connected and not running)
        self._stop_btn.setEnabled(running)

    def _gather_config(self) -> _TimingConfig:
        """Probe knobs for the legacy top-speed worker + the preflight.

        ``max_segments`` / ``seg_len_mm`` keep their dataclass defaults now that the
        settle-delay sweep is gone — they described that sweep alone.
        """
        return _TimingConfig(
            speed_mm_s=float(self._spin_speed.value()),
            repeats=int(self._spin_reps.value()),
            still_window_s=float(self._spin_still.value()),
            top_max_dist_mm=float(self._spin_topdist.value()),
        )

    def _preflight(self):
        """Shared pre-run checks for both measurements. Returns cfg or None
        (with a status message set)."""
        ctrl = self._controller
        if not getattr(ctrl, "is_xy_connected", False) or \
                not getattr(ctrl, "is_zp_connected", False):
            self._status.setText("Connect the XY and ZP stages first.")
            return None
        if self._safe_z is None:
            self._status.setText(
                "No Safe Z calibrated — set it on Calibration → Needle Offset "
                "first (the needle stays retracted there for the whole run).")
            return None
        if self._resolve_start_mm() is None:
            self._status.setText(
                "No start location — calibrate a plate (for plate centre), or "
                "jog the stage and press 'Use current position'.")
            return None
        # The camera is only needed for the 'Microscope camera' detector; the
        # default 'Stage position' detector measures motion from the stage's own
        # reported position, so it needs no camera at all.
        if self._detector_mode() == "camera":
            self._optical_cam_idx = self._start_microscope_camera()
            if self._optical_cam_idx is None or self._camera_manager is None:
                self._status.setText(
                    "The 'Microscope camera' detector needs the microscope — "
                    "start it on Hardware Setup → Cameras, or switch Detector to "
                    "'Stage position (no camera)'.")
                return None
        else:
            self._optical_cam_idx = None
        return self._gather_config()

    def _launch(self, target, cfg, status_msg):
        self._log.clear()
        self._strip.clear()
        self._delay_plot.clear()
        self._live_t0 = None
        self._start_live_feed()
        self._banner.setText("Running…")
        self._banner.setStyleSheet(
            f"color: {COLORS['base']}; background-color: {COLORS['blue']};"
            f" border-radius: 6px; padding: {s(6)}px; font-size: {sf(11)}pt;"
            f" font-weight: 700;")
        self._status.setText(status_msg)
        self._stop.clear()
        self._thread = threading.Thread(
            target=target, args=(cfg,), name="TimingCalibration", daemon=True)
        self._thread.start()
        self._update_button_state()

    # ── v7.21.2: the ONE calibration ──────────────────────────────

    def _grid_level(self) -> str:
        combo = getattr(self, "_grid_combo", None)
        if combo is None:
            return "medium"
        return str(combo.currentData() or "medium")

    def _calibration_request(self):
        """Build the request from the popout's targets + the run row's grid."""
        def _val(attr, default):
            w = getattr(self, attr, None)
            try:
                return float(w.value()) if w is not None else default
            except Exception:
                return default
        return _CR.CalibrationRequest(
            target_speed_mm_s=_val("_spin_speed", 5.0),
            resolution_um=_val("_spin_res_um", 30.0),
            grid_level=self._grid_level(),
            feature_mm=_val("_spin_feature_mm", 2.0),
            safe_z_mm=self._safe_z,
            top_speed_max_dist_mm=_val("_spin_topdist", 10.0),
            top_speed_repeats=int(_val("_spin_reps", 2)),
        )

    def _on_run_calibration(self):
        if self._running():
            return
        cfg = self._preflight()
        if cfg is None:
            return
        self._launch(self._run_calibration, self._calibration_request(),
                     "XY calibration running…")

    def _run_calibration(self, request) -> None:
        """Worker: the whole calibration, start to finish.

        All the sequencing, safety and scoring lives in the GUI-free
        ``XYCalibrationRun`` module — this method only relays progress and the
        final summary, so the same run is testable without Qt.
        """
        def _on_step(step):
            self._bridge.step.emit(step)
            if step.message:
                self._log_t(f"{step.title}: {step.message}")
        try:
            res = _CR.run_calibration(
                self._controller, settings=self._settings, store=get_store(),
                request=request, stop_evt=self._stop, on_progress=_on_step)
            ts = res.top_speed_fit or {}
            if res.top_speed_um_s and ts:
                try:
                    TS.write_jsonl({**ts, "top_speed_um_s": res.top_speed_um_s,
                                    "rows": [], "distances_mm": []}, _LOG_DIR)
                except Exception:
                    pass
            if res.grid is not None and res.grid.best is not None:
                self._bridge.stats.emit({
                    "top": f"{res.top_speed_um_s / 1000.0:.2f} mm/s",
                    "cap": f"{res.grid.resolved_cap_mm_s:.2f} mm/s",
                    "la": f"{res.grid.best.lookahead_mm:.3f} mm",
                    "p95": f"{res.grid.best.p95_um:.0f} µm"})
            self._bridge.finished.emit(bool(res.ok), res.summary or res.error)
        except Exception as e:                                # pragma: no cover
            logger.exception("XY calibration failed")
            self._bridge.finished.emit(False, f"Error: {e}")

    def _on_step(self, step) -> None:
        """GUI thread: render one phase boundary."""
        try:
            idx = _CR.STEP_ORDER.index(step.step) + 1
        except ValueError:
            idx = 0
        n = len(_CR.STEP_ORDER)
        done = idx - (0 if step.state in ("done", "skipped") else 1)
        if hasattr(self, "_progress"):
            self._progress.setValue(int(100.0 * max(0, done) / n))
        if hasattr(self, "_step_label"):
            self._step_label.setText(f"{idx}/{n} · {step.title}")
        if hasattr(self, "_clock_label"):
            self._clock_label.setText(
                f"{_mmss(step.elapsed_s)} elapsed · ~{_mmss(step.remaining_s)} left")

    def _on_measure_speed(self):
        if self._running():
            return
        cfg = self._preflight()
        if cfg is not None:
            self._launch(self._run_top_speed, cfg, "Measuring XY top speed…")

    def _on_check_comms(self):
        """Measure the closed-loop control cadence under motion (comms rate)."""
        if self._running():
            return
        ctrl = self._controller
        if not getattr(ctrl, "is_xy_connected", False):
            self._status.setText("Connect the XY stage first.")
            return
        if self._safe_z is None and getattr(ctrl, "is_zp_connected", False):
            self._status.setText(
                "No Safe Z calibrated — set it on Calibration → Needle Offset "
                "first (the needle stays retracted for the measurement).")
            return
        if not hasattr(ctrl, "measure_control_loop_rate"):
            self._status.setText("This controller can't measure the comms rate.")
            return
        # No camera / start-location needed — this is a pure comms + motion probe.
        self._launch(self._run_comms_check, self._gather_config(),
                     "Measuring closed-loop comms rate (stage moving)…")

    def _run_comms_check(self, _cfg) -> None:
        """Worker: retract the needle, run the interleaved send-velocity +
        read-position probe under motion, store + report the loop period."""
        ctrl = self._controller
        try:
            # Needle retracted for the whole probe (it never descends).
            if self._safe_z is not None and hasattr(ctrl, "ensure_retracted_to"):
                ctrl.ensure_retracted_to(float(self._safe_z))
            self._log_t("Probing control-loop cadence under motion…")
            res = ctrl.measure_control_loop_rate(iterations=40)
            if not isinstance(res, dict) or res.get("error"):
                msg = (res or {}).get("error", "no result") \
                    if isinstance(res, dict) else "no result"
                self._bridge.finished.emit(False, f"Comms check failed: {msg}")
                return
            hz = res.get("control_hz", 0.0)
            per = res.get("avg_period_ms", 0.0)
            self._log_t(
                f"loop {per:.1f} ms/cycle → {hz:.1f} Hz  "
                f"(read {res.get('avg_read_ms', 0):.1f} + "
                f"cmd {res.get('avg_cmd_ms', 0):.1f} ms)")
            self._log_t(
                f"min/max {res.get('min_period_ms', 0):.0f}/"
                f"{res.get('max_period_ms', 0):.0f} ms · "
                f"moved={res.get('moved')} · "
                f"excursion {res.get('max_excursion_um', 0):.0f} µm")
            # Persist + derive the max stable velocity-follow speed.
            store = get_store()
            store.set_control_loop_ms(per)
            v = store.stable_velocity_speed_mm_s()
            look = store.get_mode_params("velocity").get("lookahead_mm", 0.6)
            if v is not None:
                self._log_t(
                    f"→ max stable velocity-follow speed ≈ {v:.2f} mm/s "
                    f"(lookahead {look:.2f} mm, ×2 safety)")
                if not res.get("moved"):
                    self._log_t(
                        "⚠ stage did not visibly move — result is comms-only; "
                        "check SMS / that the stage is free to move.")
                self._bridge.finished.emit(
                    True, f"Comms {hz:.0f} Hz ({per:.0f} ms) · "
                    f"max stable ≈ {v:.2f} mm/s")
            else:
                self._bridge.finished.emit(
                    True, f"Comms {hz:.0f} Hz ({per:.0f} ms/cycle)")
        except Exception as e:
            logger.exception("Comms-rate check error")
            self._bridge.finished.emit(False, f"Comms check error: {e}")

    def _open_challenge(self):
        """Open the XY Printing Challenge bench (path-following compare + tune)."""
        try:
            from gui.dialogs.xy_challenge_dialog import XYChallengeDialog
        except Exception as e:
            self._status.setText(f"Challenge unavailable: {e}")
            return
        dlg = XYChallengeDialog(
            self._controller, safe_z=self._safe_z,
            start_xy_mm=self._resolve_start_mm(), parent=self)
        self._challenge_dlg = dlg          # keep a reference so it isn't GC'd
        dlg.show()

    def _on_stop(self):
        self._stop.set()
        self._status.setText("Stopping… (finishing the current move)")

    # ── helpers ───────────────────────────────────────────────────

    def _log_t(self, msg: str) -> None:
        self._bridge.log_line.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _print_center_mm(self):
        ctrl = self._controller
        if self._well_positions:
            try:
                name = next(iter(self._well_positions))
                wx, wy = self._well_positions[name]
                zero = ctrl.zero_position
                return ((wx - zero["x"]) / 1000.0, (wy - zero["y"]) / 1000.0)
            except Exception:
                pass
        if self._plate is not None:
            try:
                cx, cy = ctrl.default_plate_center_um()
                zero = ctrl.zero_position
                return ((cx - zero["x"]) / 1000.0, (cy - zero["y"]) / 1000.0)
            except Exception:
                pass
        return None

    def _emit_motion(self, t: float, change: float) -> None:
        self._bridge.motion.emit(self._rel(t), change)

    def _sleep_sampling(self, dur: float, tracker: "_FrameMotion") -> None:
        """Sleep ``dur`` while sampling the frame-change for the live strip."""
        end = time.monotonic() + dur
        while time.monotonic() < end and not self._stop.is_set():
            m = tracker.metric()
            if m is not None:
                self._emit_motion(time.monotonic(), m)
            time.sleep(0.025)

    def _dwell(self, tracker, dur: float) -> list:
        """Sample the frame-change for ``dur`` (emitting to the live strip);
        return the list of change values seen."""
        out = []
        end = time.monotonic() + dur
        while time.monotonic() < end and not self._stop.is_set():
            m = tracker.metric()
            if m is not None:
                out.append(m)
                self._emit_motion(time.monotonic(), m)
            time.sleep(0.025)
        return out

    def _wait_quiet(self, tracker, thresh, window, timeout) -> bool:
        """Block until the frame-change stays below ``thresh`` for ``window``
        (the stage is settled). Returns False on timeout."""
        t0 = time.monotonic()
        below_since = None
        while not self._stop.is_set() and (time.monotonic() - t0) < timeout:
            now = time.monotonic()
            m = tracker.metric()
            if m is not None:
                self._emit_motion(now, m)
                if m < thresh:
                    if below_since is None:
                        below_since = now
                    elif now - below_since >= window:
                        return True
                else:
                    below_since = None
            time.sleep(0.025)
        return False

    def _watch_until_still(self, tracker, thresh, window, timeout):
        """Watch frames until MOTION is seen (change exceeds ``thresh``) and
        THEN the change stays below ``thresh`` for ``window``. Returns the time
        the frames first went still (so the reported delay excludes the
        confirmation window), or None on timeout. The motion-first requirement
        prevents a false 0 ms reading when the motion isn't being detected —
        that surfaces as a timeout instead."""
        t0 = time.monotonic()
        below_since = None
        seen_motion = False
        while not self._stop.is_set() and (time.monotonic() - t0) < timeout:
            now = time.monotonic()
            m = tracker.metric()
            if m is not None:
                self._emit_motion(now, m)
                if m >= thresh:
                    seen_motion = True
                    below_since = None
                elif seen_motion:
                    if below_since is None:
                        below_since = now
                    elif now - below_since >= window:
                        return below_since
            time.sleep(0.02)
        return None

    def _calibrate_threshold(self, ctrl, tracker, cx, cy, seg, seg_time):
        """Derive the still/moving threshold from the ACTUAL scene by measuring
        the rest noise floor and the frame-change PEAK during a real test move,
        then placing the threshold between them. Returns (floor, peak, thresh)
        or None if there were no frames. This adapts to the scene's contrast so
        the threshold can't sit above the motion (the 0 ms-delay bug)."""
        ctrl.move_xy_absolute(cx, cy, from_zero_ref=True)
        if hasattr(ctrl, "wait_for_xy_arrival"):
            ctrl.wait_for_xy_arrival(cx, cy, tolerance_mm=0.1, timeout_s=15.0)
        tracker.reset()
        # Let the stage physically settle (open-loop drain can exceed 1 s), then
        # measure the at-rest noise floor.
        self._dwell(tracker, 1.8)
        floor_samples = self._dwell(tracker, 0.8)
        if len(floor_samples) < 3:
            return None
        floor = sorted(floor_samples)[len(floor_samples) // 2]   # median
        # Calibration move: out one segment, capture the peak change, then back.
        self._log_t("Calibrating: test move to measure motion vs. rest…")
        ctrl.move_xy_absolute(cx + seg[0], cy + seg[1], from_zero_ref=True)
        move_samples = self._dwell(tracker, max(2.0, seg_time * 4))
        ctrl.move_xy_absolute(cx, cy, from_zero_ref=True)
        self._dwell(tracker, max(2.0, seg_time * 4))
        peak = max(move_samples) if move_samples else floor
        thresh = floor + max((peak - floor) * 0.3, 0.0)
        return (floor, peak, thresh)

    def _encoder_threshold(self, ctrl, tracker, cx, cy):
        """Still/moving threshold for the STAGE-POSITION detector: measure only
        the at-rest jitter (no move — the move-peak is exactly what made the
        camera threshold fragile on short segments) and place the threshold
        safely above it. The metric is absolute µm, so real motion (tens–hundreds
        of µm/poll) always clears this by a wide margin. Returns (floor, peak,
        still_thresh) in µm/poll."""
        ctrl.move_xy_absolute(cx, cy, from_zero_ref=True)
        if hasattr(ctrl, "wait_for_xy_arrival"):
            ctrl.wait_for_xy_arrival(cx, cy, tolerance_mm=0.1, timeout_s=15.0)
        tracker.reset()
        self._dwell(tracker, 1.5)                    # let open-loop drain finish
        rest = self._dwell(tracker, 1.0)             # sample at-rest jitter
        if rest:
            srt = sorted(rest)
            floor = srt[min(len(srt) - 1, int(len(srt) * 0.9))]   # 90th pct
        else:
            floor = 0.0
        still_um = getattr(tracker, "STILL_UM", 3.0)
        motion_um = getattr(tracker, "MOTION_UM", 60.0)
        thresh = max(still_um, floor * 3.0, floor + 3.0)
        return (floor, motion_um, thresh)

    # ── worker thread ─────────────────────────────────────────────

    def _run_top_speed(self, cfg: _TimingConfig) -> None:
        """Measure the stage's TRUE top speed: at FULL speed (SMS,100), sweep
        single-move distances and time each to stillness; the time-vs-distance
        slope = 1/top-speed (the constant accel/decel + still-detection overhead
        falls into the intercept, so it cancels). Store + apply the result so
        the mm/s↔SMS-% conversion is correct (1 mm/s really means 1 mm/s)."""
        ctrl = self._controller
        center = self._resolve_start_mm()
        if center is None:
            self._bridge.finished.emit(False, "No start location — aborted.")
            return
        tracker = self._make_tracker()
        if not tracker.available():
            self._bridge.finished.emit(
                False, "Motion detector unavailable — connect the XY stage "
                "(stage-position detector) or start the microscope (camera "
                "detector).")
            return

        cx, cy = center
        npts = 5
        maxd = max(cfg.top_max_dist_mm, 1.0)
        dists = [round(maxd * k / npts, 3) for k in range(1, npts + 1)]
        had_poller = hasattr(ctrl, "suspend_position_poller")
        had_wd = hasattr(ctrl, "suspend_zp_watchdog")
        ok = True
        summary = ""
        rows: list[dict] = []
        self._log_t(
            f"Top-speed measurement at FULL speed (SMS,100): distances "
            f"{', '.join(f'{d:g}' for d in dists)} mm. Needle retracted.")
        try:
            if hasattr(ctrl, "ensure_retracted_to") and self._safe_z is not None:
                ctrl.ensure_retracted_to(float(self._safe_z))
            if self._stop.is_set():
                self._bridge.finished.emit(False, "Stopped before start.")
                return
            xy = getattr(ctrl, "xy_stage", None)
            if xy is not None:
                if hasattr(xy, "set_acceleration"):
                    try:
                        xy.set_acceleration(80)
                    except Exception:
                        pass
                if hasattr(xy, "set_velocity"):
                    try:
                        xy.set_velocity(100)        # SMS,100 = full speed
                    except Exception:
                        pass
            if had_poller:
                ctrl.suspend_position_poller()
            if had_wd:
                ctrl.suspend_zp_watchdog()

            _fixed = getattr(tracker, "fixed_threshold", None)
            if _fixed is not None:
                floor, peak, still_thresh = self._encoder_threshold(
                    ctrl, tracker, cx, cy)
                self._log_t(
                    f"Stage-position detector: rest jitter {floor:.1f} → still "
                    f"threshold {still_thresh:.1f} µm/poll.")
            else:
                cal = self._calibrate_threshold(
                    ctrl, tracker, cx, cy, (dists[0], 0.0), dists[0] / 5.0)
                if cal is None:
                    self._bridge.finished.emit(
                        False, "No camera frames — is the microscope running?")
                    return
                floor, peak, still_thresh = cal
                self._log_t(
                    f"Calibration: rest {floor:.2f}, motion peak {peak:.2f} "
                    f"→ still threshold {still_thresh:.2f}.")
                if (peak - floor) < max(floor * 0.5, 0.5):
                    self._bridge.finished.emit(
                        False, "Camera not detecting stage motion — check "
                        "focus / texture / lighting / FPS.")
                    return
            self._strip.set_threshold(still_thresh)
            self._bridge.stats.emit({"thresh": f"{still_thresh:.2f}"})

            for d in dists:
                if self._stop.is_set():
                    break
                times = []
                for _rep in range(cfg.repeats):
                    if self._stop.is_set():
                        break
                    t = self._measure_n(ctrl, tracker, (cx, cy), (d, 0.0), 1,
                                        0.0, still_thresh, cfg.still_window_s)
                    if t is not None:
                        times.append(t)
                if not times:
                    self._log_t(f"  d={d:g}mm: no stop detected (timeout).")
                    continue
                avg = sum(times) / len(times)
                rows.append({"dist_mm": d, "time_s": avg, "reps": len(times)})
                self._bridge.stats.emit(
                    {"seg": f"{d:g}mm", "delay": f"{avg * 1000:.0f}"})
                self._log_t(f"  d={d:g}mm: {avg * 1000:.0f} ms (n={len(times)})")

            pts = [(r["dist_mm"], r["time_s"]) for r in rows]
            slope, intercept = _fit_line(pts)          # slope = s per mm
            self._log_t("──── RESULT ────")
            if len(pts) >= 2 and slope > 1e-6:
                cruise_mm_s = 1.0 / slope
                cruise_um_s = cruise_mm_s * 1000.0
                if xy is not None and hasattr(xy, "set_max_speed_um_s"):
                    xy.set_max_speed_um_s(cruise_um_s)
                try:
                    get_store().set_xy_max_speed_um_s(cruise_um_s)
                except Exception as e:
                    self._log_t(f"Store save failed: {e}")
                self._bridge.result.emit({
                    "points": pts, "slope": slope, "intercept": intercept,
                    "headline": (f"top speed {cruise_mm_s:.2f} mm/s "
                                 f"(overhead {intercept * 1000:.0f} ms)"),
                    "xlabel": "distance (mm) →", "ylabel": "time (ms)",
                    # v7.5.x: hand the measured top speed to the GUI thread so
                    # _on_result writes it into the SINGLE common XY-max source
                    # (safety_limits.max_xy_speed + settings) and fans out.
                    "xy_top_speed_um_s": cruise_um_s})
                self._log_t(
                    f"XY TOP SPEED = {cruise_mm_s:.2f} mm/s ({cruise_um_s:.0f} "
                    f"µm/s) at SMS,100. STORED + APPLIED — the mm/s↔SMS "
                    f"conversion is now correct (move overhead ≈ "
                    f"{intercept * 1000:.0f} ms). Re-run the settle sweep to "
                    f"confirm the per-segment lag is now small.")
                summary = (f"top speed {cruise_mm_s:.2f} mm/s — stored & applied")
            else:
                summary = ("couldn't fit a slope — need ≥2 distances that move "
                           "(check the camera sees motion / longer distances)")
                self._log_t(summary)
            self._write_topspeed_jsonl(cfg, dists, rows, slope, intercept)
        except Exception as e:
            logger.exception("Top-speed measurement error: %s", e)
            self._log_t(f"⚠ Worker error: {e}")
            ok = False
            summary = f"error: {e}"
        finally:
            # leave a moderate jog speed, not full speed
            try:
                if xy is not None and hasattr(xy, "set_velocity"):
                    xy.set_velocity(20)
            except Exception:
                pass
            if had_poller:
                try:
                    ctrl.resume_position_poller()
                except Exception:
                    pass
            if had_wd:
                try:
                    ctrl.resume_zp_watchdog()
                except Exception:
                    pass
            try:
                if (getattr(ctrl, "is_zp_connected", False)
                        and self._safe_z is not None
                        and hasattr(ctrl, "ensure_retracted_to")):
                    ctrl.ensure_retracted_to(float(self._safe_z))
            except Exception as e:
                logger.warning("Top-speed final retract failed: %s", e)

        if self._stop.is_set() and not summary:
            summary = "Stopped by operator."
            ok = False
        self._bridge.finished.emit(ok, summary or "done")

    def _write_topspeed_jsonl(self, cfg, dists, rows, slope, intercept):
        try:
            _LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = _LOG_DIR / f"topspeed_{stamp}.jsonl"
            cruise = (1.0 / slope) if slope > 1e-6 else None
            with open(path, "w", encoding="utf-8") as f:
                f.write(json.dumps({
                    "event": "config", "kind": "top_speed_sweep",
                    "distances_mm": dists, "repeats": cfg.repeats,
                    "still_window_s": cfg.still_window_s}) + "\n")
                for r in rows:
                    f.write(json.dumps({"event": "point", **r}) + "\n")
                f.write(json.dumps({
                    "event": "fit", "slope_s_per_mm": round(slope, 6),
                    "intercept_s": round(intercept, 5),
                    "top_speed_mm_s": (None if cruise is None
                                       else round(cruise, 4))}) + "\n")
            self._log_t(f"Top-speed sweep written to {path}")
        except Exception as e:
            self._log_t(f"JSONL write failed: {e}")

    def _measure_n(self, ctrl, tracker, start, seg, n, seg_time,
                   still_thresh, still_window):
        """One measurement of N back-to-back segments: go to start, settle,
        stream N moves (paced like a print), then time how long after the LAST
        command the frames stop changing. Returns the delay (s) or None."""
        sx, sy = start
        # return to the line start and wait until the stage is settled (quiet)
        ctrl.move_xy_absolute(sx, sy, from_zero_ref=True)
        if hasattr(ctrl, "wait_for_xy_arrival"):
            ctrl.wait_for_xy_arrival(sx, sy, tolerance_mm=0.1, timeout_s=15.0)
        tracker.reset()
        if not self._wait_quiet(tracker, still_thresh, still_window,
                                timeout=15.0):
            return None
        # stream N collinear segments, paced like a print
        t_last = None
        for k in range(1, n + 1):
            if self._stop.is_set():
                return None
            tgt = (sx + seg[0] * k, sy + seg[1] * k)
            ctrl.move_xy_absolute(tgt[0], tgt[1], from_zero_ref=True)
            t_last = time.monotonic()
            if k < n:
                self._sleep_sampling(seg_time, tracker)
        if t_last is None:
            return None
        self._bridge.marker.emit(self._rel(t_last), "cmd")
        t_still = self._watch_until_still(tracker, still_thresh, still_window,
                                          timeout=20.0)
        if t_still is None:
            return None
        self._bridge.marker.emit(self._rel(t_still), "still")
        return max(0.0, t_still - t_last)


def _fit_line(points: list) -> tuple:
    """Least-squares fit of delay vs N → (slope_s_per_seg, intercept_s)."""
    if not points:
        return (0.0, 0.0)
    if len(points) == 1:
        return (0.0, points[0][1])
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    slope = (sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
             if den > 1e-9 else 0.0)
    return (slope, my - slope * mx)
