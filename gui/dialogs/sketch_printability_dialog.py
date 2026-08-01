"""sketch_printability_dialog.py — will this sketch actually print? (v7.5.x)

Takes a compiled sketch trajectory, splits it into its printing sub-paths, and
simulates each with the machine's SAVED stage characteristics + the CURRENT
velocity tuning (``XYPathSimulator`` — the same follower loop the real print
runs, driving the measured stage model instead of the stage). The canvas shows
the ideal geometry in grey, the predicted path coloured by verdict, and marks
the regions whose predicted deviation exceeds the resolution element — so the
operator sees *where* a print would fail (a corner that will round, a feature
finer than the tuning can track, a spike the follower would orbit) before any
ink or plate is committed.

The speed spin re-simulates instantly, so "would this work at 2 mm/s?" is a
click, not a print.

Prediction headroom: on ME3B V1 the model tracked real geometry-panel runs to
the correct verdict in 16/18 cells but its p95 magnitudes ran ~2× optimistic
(the model has no encoder noise or comms jitter), so verdicts here are computed
against the element divided by ``PREDICTION_HEADROOM`` — a prediction must
clear a 2× tighter bar before it is called a pass.
"""

from __future__ import annotations

import logging
import math
import threading

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QDialog, QDoubleSpinBox, QHBoxLayout, QLabel, QPlainTextEdit,
    QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from SupportClasses import XYPathSimulator as PS
from SupportClasses.PrintTimingCalibrationStore import get_store
from SupportClasses.XYStageModel import StageCharacteristics

logger = logging.getLogger(__name__)

#: Validated on ME3B V1 (geometry panel, 2026-07-28): the deterministic model's
#: p95 runs ~2× below the measured value, with verdicts matching in 16/18
#: cells. Predictions must therefore clear a 2× tighter bar to be called PASS.
PREDICTION_HEADROOM = 2.0

_VERDICT_COLOR = {"pass": "green", "marginal": "peach", "fail": "red"}


class _Bridge(QObject):
    done = Signal(dict)


class _PathsView(QWidget):
    """Ideal sub-paths (grey) + predicted paths (verdict colour) + fail-region
    markers, auto-fitted."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._paths = []        # [(ideal, predicted, verdict)]
        self._regions = []      # fail-region dicts (world mm)
        self.setMinimumSize(s(420), s(380))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_result(self, check: dict):
        self._paths = [(e["ideal"], [(sm[0], sm[1])
                                     for sm in e["result"].samples],
                        e["result"].verdict)
                       for e in check.get("paths", [])]
        self._regions = check.get("fail_regions", [])
        self.update()

    def clear(self):
        self._paths = []
        self._regions = []
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(COLORS["base"]))
        xs, ys = [], []
        for ideal, pred, _v in self._paths:
            for seq in (ideal, pred):
                for pt in seq:
                    xs.append(pt[0])
                    ys.append(pt[1])
        if not xs:
            p.setPen(QPen(QColor(COLORS["overlay0"])))
            p.drawText(self.rect(), Qt.AlignCenter,
                       "No printing sub-paths")
            p.end()
            return
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        x1 = x1 if x1 > x0 else x0 + 1e-6
        y1 = y1 if y1 > y0 else y0 + 1e-6
        m = s(16)
        w, h = self.width() - 2 * m, self.height() - 2 * m
        sc = min(w / (x1 - x0), h / (y1 - y0))
        ox = m + (w - (x1 - x0) * sc) / 2.0
        oy = m + (h - (y1 - y0) * sc) / 2.0

        def px(pt):
            return (ox + (pt[0] - x0) * sc, oy + (y1 - pt[1]) * sc)

        def draw(seq, pen):
            p.setPen(pen)
            last = None
            for pt in seq:
                cur = px(pt)
                if last is not None:
                    p.drawLine(int(last[0]), int(last[1]),
                               int(cur[0]), int(cur[1]))
                last = cur

        for ideal, pred, verdict in self._paths:
            draw(ideal, QPen(QColor(COLORS["surface2"]), 2))
        for ideal, pred, verdict in self._paths:
            draw(pred, QPen(QColor(
                COLORS[_VERDICT_COLOR.get(verdict, "red")]), 1))
        for reg in self._regions:
            c = QColor(COLORS["red" if reg.get("severity") == "fail"
                              else "peach"])
            c.setAlpha(170)
            p.setPen(QPen(c, 2))
            cx, cy = px((reg["x_mm"], reg["y_mm"]))
            r = max(4, int(s(4) + math.log1p(reg.get("peak_um", 0.0))))
            p.drawEllipse(int(cx - r), int(cy - r), 2 * r, 2 * r)
        p.end()


class SketchPrintabilityDialog(QDialog):
    """Simulate the compiled sketch on the saved stage model and show where it
    would fail."""

    def __init__(self, trajectory, *, print_speed_mm_s=3.0, store=None,
                 parent=None):
        super().__init__(parent)
        self.setWindowTitle("Printability check — simulated stage motion")
        self.setModal(False)
        self._traj = trajectory
        self._store = store or get_store()
        self._bridge = _Bridge()
        self._bridge.done.connect(self._on_done)
        self._thread = None
        self._last_check = None
        self._build(print_speed_mm_s)
        self.resize(s(860), s(640))
        self._recheck()

    # ── UI ────────────────────────────────────────────────────────
    def _build(self, speed):
        root = QVBoxLayout(self)
        char = StageCharacteristics.from_store(self._store)
        self._char = char
        top = QHBoxLayout()
        self._banner = QLabel("")
        self._banner.setWordWrap(True)
        top.addWidget(self._banner, 1)
        top.addWidget(QLabel("Print speed:"))
        self._speed = QDoubleSpinBox()
        self._speed.setRange(0.1, 20.0)
        self._speed.setDecimals(1)
        self._speed.setSuffix(" mm/s")
        self._speed.setValue(float(speed))
        self._speed.valueChanged.connect(lambda _v: self._recheck())
        top.addWidget(self._speed)
        self._again = QPushButton("↻ Re-check")
        self._again.clicked.connect(self._recheck)
        top.addWidget(self._again)
        root.addLayout(top)

        body = QHBoxLayout()
        self._view = _PathsView()
        body.addWidget(self._view, 2)
        self._detail = QPlainTextEdit()
        self._detail.setReadOnly(True)
        self._detail.setMinimumWidth(s(280))
        self._detail.setStyleSheet(f"font-size: {sf(8.5)}pt;")
        body.addWidget(self._detail, 1)
        root.addLayout(body, 1)

        foot = QLabel(
            f"Grey = sketch geometry · coloured = predicted stage path with "
            f"the saved characteristics + current tuning · rings = regions "
            f"predicted beyond the resolution element (verdicts use a "
            f"{PREDICTION_HEADROOM:g}× headroom — the model runs optimistic).")
        foot.setWordWrap(True)
        foot.setStyleSheet(f"color: {COLORS['subtext0']}; "
                           f"font-size: {sf(8)}pt;")
        root.addWidget(foot)

    # ── simulation ────────────────────────────────────────────────
    def _recheck(self):
        if self._thread is not None:
            return
        if not self._char.is_complete():
            self._banner.setText(
                "⚠ The stage's characteristics have not been measured — run "
                "the one-click XY calibration (Timing Calibration page) "
                "first. Missing: " + ", ".join(self._char.missing()))
            self._banner.setStyleSheet(f"color: {COLORS['yellow']};")
            self._again.setEnabled(False)
            return
        self._banner.setText("simulating…")
        self._banner.setStyleSheet(f"color: {COLORS['subtext0']};")
        self._again.setEnabled(False)
        speed = float(self._speed.value())
        tuning = PS.tuning_from_store(self._store)
        try:
            res_um = float(self._store.get_resolution_element_um() or 30.0)
        except Exception:
            res_um = 30.0
        char = self._char
        traj = self._traj

        def _work():
            try:
                check = PS.check_trajectory(
                    traj, char=char, print_speed_mm_s=speed, tuning=tuning,
                    resolution_um=res_um / PREDICTION_HEADROOM)
                check["resolution_um"] = res_um
            except Exception as e:              # pragma: no cover
                logger.exception("printability check failed")
                check = {"error": str(e)}
            self._bridge.done.emit(check)

        self._thread = threading.Thread(target=_work, daemon=True)
        self._thread.start()

    def _on_done(self, check: dict):
        self._thread = None
        self._again.setEnabled(True)
        if "error" in check:
            self._banner.setText(f"check failed: {check['error']}")
            self._banner.setStyleSheet(f"color: {COLORS['red']};")
            return
        self._last_check = check
        self._view.set_result(check)
        verdict = check["verdict"]
        n_fail = sum(1 for r in check["fail_regions"]
                     if r["severity"] == "fail")
        n_warn = sum(1 for r in check["fail_regions"]
                     if r["severity"] == "warn")
        icon = {"pass": "✅", "marginal": "⚠", "fail": "✖"}.get(verdict, "✖")
        msg = (f"{icon} Predicted: {verdict.upper()} — worst p95 "
               f"{check['worst_p95_um']:.0f} µm, worst max "
               f"{check['worst_max_um']:.0f} µm over {len(check['paths'])} "
               f"printing path(s)")
        if not check["all_completed"]:
            msg += " · ⚠ a path is predicted to STALL (follower cannot track it)"
        if n_fail or n_warn:
            msg += f" · {n_fail} fail / {n_warn} warn region(s)"
        self._banner.setText(msg)
        self._banner.setStyleSheet(
            f"color: {COLORS[_VERDICT_COLOR.get(verdict, 'red')]}; "
            f"font-weight: bold;")

        lines = []
        for e in check["paths"]:
            r = e["result"]
            lines.append(
                f"path {e['index'] + 1}: {e['length_mm']:.1f} mm — "
                f"{r.verdict}  p95 {r.report['p95_um']:.0f} µm"
                + ("" if r.completed
                   else f"  ⚠ stalls at {r.report['completion_frac']*100:.0f}%"))
            for reg in r.fail_regions:
                lines.append(
                    f"    {reg['severity']} @ ({reg['x_mm']:.2f}, "
                    f"{reg['y_mm']:.2f}) mm — predicted {reg['peak_um']:.0f} µm")
        self._detail.setPlainText("\n".join(lines) or "No printing sub-paths.")

    def closeEvent(self, ev):
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        super().closeEvent(ev)
