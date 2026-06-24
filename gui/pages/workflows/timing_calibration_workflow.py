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
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.camera_feed_view import CameraFeedView
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)

from SupportClasses.PrintTimingCalibrationStore import get_store

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


@dataclass
class _TimingConfig:
    max_segments: int
    seg_len_mm: float
    speed_mm_s: float
    repeats: int
    still_window_s: float
    top_max_dist_mm: float = 10.0


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
        if not hasattr(self, "_settings_summary"):
            return
        try:
            self._settings_summary.setText(
                f"N≤{self._spin_maxn.value()} · seg {self._spin_seg.value():g} mm "
                f"· {self._spin_speed.value():g} mm/s")
        except Exception:
            pass

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
        # ── Settle-delay sweep ──
        self._spin_maxn = self._ispin(
            1, 40, 8,
            "Sweeps N = 1, 2, … up to this. The delay's growth across N is the "
            "per-segment phase lag.")
        self._spin_maxn.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        self._spin_seg = self._dspin(0.1, 10.0, 1.0, " mm", 2, 0.1)
        self._spin_seg.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        self._spin_speed = self._dspin(0.1, 50.0, 5.0, " mm/s", 2, 0.5)
        self._spin_speed.valueChanged.connect(
            lambda *_: self._update_settings_summary())
        self._spin_reps = self._ispin(
            1, 10, 2, "Average this many runs per N (less noise).")
        self._spin_still = self._dspin(
            0.1, 2.0, 0.35, " s", 2, 0.05,
            "Frames must stay unchanged this long to count as 'stopped'.")
        sec = dlg.add_section("Settle-delay sweep")
        sec.add("maxn", "Max segments (N)", self._spin_maxn, 8)
        sec.add("seg", "Segment length", self._spin_seg, 1.0)
        sec.add("speed", "Print speed", self._spin_speed, 5.0)
        sec.add("reps", "Repeats / N", self._spin_reps, 2)
        sec.add("still", "Still window", self._spin_still, 0.35)

        # ── Top-speed measurement ──
        self._spin_topdist = self._dspin(
            1.0, 50.0, 10.0, " mm", 1, 1.0,
            "Measure-top-speed sweeps move distances up to this at full speed; "
            "the time-vs-distance slope = 1/top-speed. Bigger = more accurate "
            "(needs envelope room + a textured view along +X).")
        sec = dlg.add_section("Top-speed measurement")
        sec.add("topdist", "Top-speed max distance", self._spin_topdist, 10.0)

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

    def _build_main_area(self) -> QWidget:
        split = QSplitter(Qt.Horizontal)
        left = QSplitter(Qt.Vertical)
        try:
            self._camera_view = CameraFeedView(
                self._camera_manager,
                cam_idx=self._resolve_microscope_cam_idx(),
                label="Microscope — frame-difference still detection",
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
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))
        self._start_btn = QPushButton("Start sweep")
        self._start_btn.setToolTip(
            "Segment settle-delay sweep — measures the per-segment phase lag.")
        self._start_btn.clicked.connect(self._on_start)
        row.addWidget(self._start_btn)
        self._speed_btn = QPushButton("Measure top speed")
        self._speed_btn.setToolTip(
            "Sweep move distance at FULL speed; the time-vs-distance slope = "
            "1/top-speed. Stores it so the mm/s↔SMS conversion is correct "
            "(makes commanded speed real). Do this FIRST.")
        self._speed_btn.clicked.connect(self._on_measure_speed)
        row.addWidget(self._speed_btn)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        row.addWidget(self._stop_btn)
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
        self._start_btn.setEnabled(connected and not running)
        if hasattr(self, "_speed_btn"):
            self._speed_btn.setEnabled(connected and not running)
        self._stop_btn.setEnabled(running)

    def _gather_config(self) -> _TimingConfig:
        return _TimingConfig(
            max_segments=int(self._spin_maxn.value()),
            seg_len_mm=float(self._spin_seg.value()),
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
        self._optical_cam_idx = self._start_microscope_camera()
        if self._optical_cam_idx is None or self._camera_manager is None:
            self._status.setText(
                "This measurement needs the microscope camera — start it on "
                "Hardware Setup → Cameras first.")
            return None
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

    def _on_start(self):
        if self._running():
            return
        cfg = self._preflight()
        if cfg is not None:
            self._launch(self._run, cfg, "Segment settle-delay sweep running…")

    def _on_measure_speed(self):
        if self._running():
            return
        cfg = self._preflight()
        if cfg is not None:
            self._launch(self._run_top_speed, cfg, "Measuring XY top speed…")

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

    # ── worker thread ─────────────────────────────────────────────

    def _run(self, cfg: _TimingConfig) -> None:
        ctrl = self._controller
        center = self._resolve_start_mm()
        if center is None:
            self._bridge.finished.emit(False, "No start location — aborted.")
            return
        tracker = _FrameMotion(self._camera_manager, self._optical_cam_idx)
        if not tracker.available():
            self._bridge.finished.emit(False, "Microscope camera unavailable.")
            return

        cx, cy = center
        seg = (cfg.seg_len_mm, 0.0)               # straight line along +X
        seg_time = cfg.seg_len_mm / max(cfg.speed_mm_s, 0.01)
        had_poller = hasattr(ctrl, "suspend_position_poller")
        had_wd = hasattr(ctrl, "suspend_zp_watchdog")
        ok = True
        summary = ""
        rows: list[dict] = []
        self._log_t(
            f"Settle-delay sweep: N=1..{cfg.max_segments} × {cfg.repeats} "
            f"reps · seg {cfg.seg_len_mm:.2f}mm @ {cfg.speed_mm_s:.2f}mm/s "
            f"(seg time {seg_time * 1000:.0f}ms). Needle retracted.")
        try:
            if hasattr(ctrl, "ensure_retracted_to") and self._safe_z is not None:
                ctrl.ensure_retracted_to(float(self._safe_z))
            if self._stop.is_set():
                self._bridge.finished.emit(False, "Stopped before start.")
                return
            xy = getattr(ctrl, "xy_stage", None)
            if xy is not None:
                # Set acceleration too — short segments are accel-dominated, and
                # an unset/low Prior accel makes moves far slower than
                # seg_len/speed implies (a confounder for this very measurement).
                if hasattr(xy, "set_acceleration"):
                    try:
                        xy.set_acceleration(80)
                    except Exception:
                        pass
                if hasattr(xy, "set_speed_mm_s"):
                    try:
                        xy.set_speed_mm_s(cfg.speed_mm_s)
                    except Exception:
                        pass
            if had_poller:
                ctrl.suspend_position_poller()
            if had_wd:
                ctrl.suspend_zp_watchdog()

            # Derive the still/moving threshold from the actual scene via a
            # test move (see _calibrate_threshold) — robust to scene contrast.
            cal = self._calibrate_threshold(ctrl, tracker, cx, cy, seg, seg_time)
            if cal is None:
                self._bridge.finished.emit(
                    False, "No camera frames — is the microscope running?")
                return
            floor, peak, still_thresh = cal
            self._strip.set_threshold(still_thresh)
            self._bridge.stats.emit({"thresh": f"{still_thresh:.2f}"})
            self._log_t(
                f"Calibration: rest noise {floor:.2f}, motion peak {peak:.2f} "
                f"→ still threshold {still_thresh:.2f}.")
            # If a real move doesn't move the frames clearly above the rest
            # noise, the camera can't see the motion — abort with guidance
            # rather than report a bogus 0 ms.
            if (peak - floor) < max(floor * 0.5, 0.5):
                self._bridge.finished.emit(
                    False, f"Camera not detecting stage motion (rest {floor:.2f} "
                    f"≈ move {peak:.2f}) — check the microscope is focused on a "
                    f"TEXTURED region, well-lit, and running at a usable FPS.")
                return

            for n in range(1, cfg.max_segments + 1):
                if self._stop.is_set():
                    break
                delays = []
                for rep in range(cfg.repeats):
                    if self._stop.is_set():
                        break
                    d = self._measure_n(ctrl, tracker, (cx, cy), seg, n,
                                        seg_time, still_thresh,
                                        cfg.still_window_s)
                    if d is not None:
                        delays.append(d)
                if not delays:
                    self._log_t(f"  N={n}: no stop detected (timeout).")
                    continue
                avg = sum(delays) / len(delays)
                rows.append({"N": n, "delay_s": avg, "reps": len(delays),
                             "delays_s": [round(x, 4) for x in delays]})
                self._bridge.stats.emit({"seg": n, "delay": f"{avg * 1000:.0f}"})
                self._log_t(f"  N={n}: settle delay {avg * 1000:.0f} ms "
                            f"(n={len(delays)})")

            # Fit delay vs N → slope (per-segment phase lag) + intercept.
            pts = [(r["N"], r["delay_s"]) for r in rows]
            slope, intercept = _fit_line(pts)

            self._bridge.result.emit({
                "points": pts, "slope": slope, "intercept": intercept,
                "headline": (f"phase lag {slope * 1000:.1f} ms/segment "
                             f"(base {intercept * 1000:.0f} ms)"),
                "xlabel": "segments →", "ylabel": "delay (ms)"})
            self._log_t("──── RESULT ────")
            if len(pts) >= 2:
                self._log_t(
                    f"phase lag = {slope * 1000:.1f} ms PER SEGMENT "
                    f"(base settle {intercept * 1000:.0f} ms). Over an M-segment "
                    f"print the stage ends ≈{intercept * 1000:.0f} + "
                    f"{slope * 1000:.1f}·M ms behind the commands.")
                summary = (f"phase lag {slope * 1000:.1f} ms/segment · "
                           f"base {intercept * 1000:.0f} ms · {len(pts)} points")
            else:
                summary = f"{len(pts)} point(s) — need ≥2 N for a slope"
                self._log_t(summary)

            try:
                entry = get_store().update_phase(
                    cfg.speed_mm_s, cfg.seg_len_mm, intercept, slope,
                    len(pts))
                self._log_t(f"Saved (run #{entry.get('runs')}).")
            except Exception as e:
                self._log_t(f"Store update failed: {e}")
            self._write_jsonl(cfg, seg_time, rows, slope, intercept)
        except Exception as e:
            logger.exception("Timing calibration worker error: %s", e)
            self._log_t(f"⚠ Worker error: {e}")
            ok = False
            summary = f"error: {e}"
        finally:
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
                logger.warning("Timing final retract failed: %s", e)

        if self._stop.is_set() and not summary:
            summary = "Stopped by operator."
            ok = False
        self._bridge.finished.emit(ok, summary or "done")

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
        tracker = _FrameMotion(self._camera_manager, self._optical_cam_idx)
        if not tracker.available():
            self._bridge.finished.emit(False, "Microscope camera unavailable.")
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

            cal = self._calibrate_threshold(
                ctrl, tracker, cx, cy, (dists[0], 0.0), dists[0] / 5.0)
            if cal is None:
                self._bridge.finished.emit(
                    False, "No camera frames — is the microscope running?")
                return
            floor, peak, still_thresh = cal
            self._strip.set_threshold(still_thresh)
            self._bridge.stats.emit({"thresh": f"{still_thresh:.2f}"})
            self._log_t(f"Calibration: rest {floor:.2f}, motion peak {peak:.2f} "
                        f"→ still threshold {still_thresh:.2f}.")
            if (peak - floor) < max(floor * 0.5, 0.5):
                self._bridge.finished.emit(
                    False, "Camera not detecting stage motion — check focus / "
                    "texture / lighting / FPS.")
                return

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
                    "xlabel": "distance (mm) →", "ylabel": "time (ms)"})
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

    def _write_jsonl(self, cfg, seg_time, rows, slope, intercept):
        try:
            _LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = _LOG_DIR / f"settle_{stamp}_{cfg.speed_mm_s:.1f}mmps.jsonl"
            with open(path, "w", encoding="utf-8") as f:
                f.write(json.dumps({
                    "event": "config", "kind": "settle_delay_sweep",
                    "max_segments": cfg.max_segments,
                    "seg_len_mm": cfg.seg_len_mm, "speed_mm_s": cfg.speed_mm_s,
                    "seg_time_s": round(seg_time, 5), "repeats": cfg.repeats,
                    "still_window_s": cfg.still_window_s}) + "\n")
                for r in rows:
                    f.write(json.dumps({"event": "point", **r}) + "\n")
                f.write(json.dumps({
                    "event": "fit", "slope_s_per_seg": round(slope, 6),
                    "intercept_s": round(intercept, 5)}) + "\n")
            self._log_t(f"Sweep written to {path}")
        except Exception as e:
            self._log_t(f"JSONL write failed: {e}")


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
