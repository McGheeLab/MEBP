"""xy_geometry_panel_dialog.py — Geometry panel: ideal vs actual, per shape × size.

A matrix of challenge shapes at several sizes, each cell showing the IDEAL path
(grey) against what the tuned stage does — either SIMULATED from the machine's
saved characteristics (``XYStageModel`` — no hardware, instant) or DRIVEN on the
real stage. Both go through the SAME ``XYPathSimulator.follow_path`` loop with
different I/O bindings, so a simulated cell and a hardware cell differ only by
the stage itself, never by loop logic.

What it is for: seeing the effect of *geometry* on the tuned follower — where
corners round, which feature sizes the current tuning cannot resolve (a 2 mm
star against a 0.9 mm lookahead), and how a candidate tuning would change that
before committing it to the stage.

Results persist to ``logs/challenge/geometry_panel_<ts>.json`` (the same schema
the calibration bench scripts write) and old panels can be re-loaded.

Safety (hardware runs): needle retracted to Safe Z before any XY motion; the
stage is centred in its envelope first (every excursion is relative, so a
clamped run silently measures the clamp); poller + watchdog suspended;
``follow_path`` always sends VS 0,0 on every exit; cells that do not fit the
usable radius are skipped, never clamped.
"""

from __future__ import annotations

import datetime
import json
import logging
import math
import os
import threading
import time

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDoubleSpinBox, QFileDialog, QGridLayout, QHBoxLayout,
    QLabel, QLineEdit, QMessageBox, QPushButton, QScrollArea, QSizePolicy,
    QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from SupportClasses import XYAutoCalibration as AC
from SupportClasses import XYChallenge as XC
from SupportClasses import XYPathSimulator as PS
from SupportClasses.PrintTimingCalibrationStore import get_store
from SupportClasses.XYStageModel import StageCharacteristics

logger = logging.getLogger(__name__)

_VERDICT_COLOR = {"pass": "green", "marginal": "peach", "fail": "red"}


def _log_dir() -> str:
    # MEBP_CHALLENGE_LOG_DIR redirects panel records (tests use it so suite
    # runs never write into the repo's real logs/challenge).
    d = os.environ.get("MEBP_CHALLENGE_LOG_DIR") or os.path.join(
        os.getcwd(), "logs", "challenge")
    os.makedirs(d, exist_ok=True)
    return d


class _Bridge(QObject):
    cell = Signal(dict)
    log = Signal(str)
    done = Signal()


class _CellView(QWidget):
    """One matrix cell: ideal (grey) + hardware trace (solid, verdict colour)
    + simulated trace (dashed) — whichever exist."""

    def __init__(self, shape: str, size_mm: float, parent=None):
        super().__init__(parent)
        self.shape = shape
        self.size_mm = size_mm
        self.ideal = None       # [(x, y), …] mm (relative)
        self.hw = None          # {"samples": [...], "verdict": str, ...}
        self.sim = None
        self.setMinimumSize(s(150), s(150))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_run(self, source: str, ideal, samples, verdict: str):
        self.ideal = ideal
        rec = {"samples": samples, "verdict": verdict}
        if source == "hardware":
            self.hw = rec
        else:
            self.sim = rec
        self.update()

    def clear_runs(self):
        self.hw = None
        self.sim = None
        self.update()

    # ── painting ──────────────────────────────────────────────────
    def _bounds(self):
        xs, ys = [], []
        for seq in (self.ideal or [],
                    (self.hw or {}).get("samples") or [],
                    (self.sim or {}).get("samples") or []):
            for p in seq:
                xs.append(p[0])
                ys.append(p[1])
        if not xs:
            return None
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        return x0, x1 if x1 > x0 else x0 + 1e-6, y0, \
            y1 if y1 > y0 else y0 + 1e-6

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(COLORS["base"]))
        b = self._bounds()
        if not b:
            p.setPen(QPen(QColor(COLORS["overlay0"])))
            p.drawText(self.rect(), Qt.AlignCenter, "—")
            p.end()
            return
        x0, x1, y0, y1 = b
        m = s(10)
        w, h = self.width() - 2 * m, self.height() - 2 * m
        sc = min(w / (x1 - x0), h / (y1 - y0))
        ox = m + (w - (x1 - x0) * sc) / 2.0
        oy = m + (h - (y1 - y0) * sc) / 2.0

        def px(pt):
            # +y up on screen
            return (ox + (pt[0] - x0) * sc,
                    oy + (y1 - pt[1]) * sc)

        def draw(seq, pen):
            p.setPen(pen)
            last = None
            for pt in seq:
                cur = px(pt)
                if last is not None:
                    p.drawLine(int(last[0]), int(last[1]),
                               int(cur[0]), int(cur[1]))
                last = cur

        draw(self.ideal or [], QPen(QColor(COLORS["surface2"]), 2))
        if self.sim:
            pen = QPen(QColor(COLORS[_VERDICT_COLOR.get(
                self.sim["verdict"], "red")]), 1)
            pen.setStyle(Qt.DashLine)
            draw(self.sim["samples"], pen)
        if self.hw:
            draw(self.hw["samples"],
                 QPen(QColor(COLORS[_VERDICT_COLOR.get(
                     self.hw["verdict"], "red")]), 2))
        p.end()


class GeometryPanelDialog(QDialog):
    """Shape × size matrix, simulated from the saved stage characteristics
    and/or driven on the real stage."""

    SHAPES = PS.DEFAULT_PANEL_SHAPES

    def __init__(self, controller=None, *, safe_z=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("XY Geometry Panel — ideal vs actual")
        self.setModal(False)
        self._ctrl = controller
        self._safe_z = safe_z
        self._store = get_store()
        self._bridge = _Bridge()
        self._bridge.cell.connect(self._on_cell)
        self._bridge.log.connect(self._on_log)
        self._bridge.done.connect(self._on_done)
        self._stop = threading.Event()
        self._thread = None
        self._cells = {}          # (shape, size) → _CellView
        self._labels = {}         # (shape, size) → QLabel
        self._records = []        # saved-schema cell dicts of the last run
        self._build()
        self.resize(s(1050), s(760))

    # ── UI ────────────────────────────────────────────────────────
    def _build(self):
        root = QVBoxLayout(self)

        char = StageCharacteristics.from_store(self._store, name="")
        top = QHBoxLayout()
        chartxt = (f"Stage model: dead {char.dead_time_s*1000:.0f} ms · "
                   f"τ {char.tau_s*1000:.0f} ms · "
                   f"top {char.top_speed_um_s/1000:.2f} mm/s · "
                   f"loop {char.control_loop_ms:.1f} ms"
                   if char.is_complete()
                   else "Stage model: NOT measured — run the one-click "
                        "XY calibration first (simulation unavailable)")
        self._char_lbl = QLabel(chartxt)
        top.addWidget(self._char_lbl)
        top.addStretch(1)
        top.addWidget(QLabel("Speed:"))
        self._speed = QDoubleSpinBox()
        self._speed.setRange(0.1, 20.0)
        self._speed.setValue(3.0)
        self._speed.setDecimals(1)
        self._speed.setSuffix(" mm/s")
        top.addWidget(self._speed)
        top.addWidget(QLabel("Sizes (mm):"))
        self._sizes = QLineEdit("2, 5, 10")
        self._sizes.setMaximumWidth(s(110))
        top.addWidget(self._sizes)
        root.addLayout(top)

        run = QHBoxLayout()
        self._sim_btn = QPushButton("▶ Simulate panel")
        self._sim_btn.clicked.connect(lambda: self._start("simulated"))
        self._sim_btn.setEnabled(char.is_complete())
        run.addWidget(self._sim_btn)
        self._hw_btn = QPushButton("▶ Run on stage")
        self._hw_btn.clicked.connect(lambda: self._start("hardware"))
        run.addWidget(self._hw_btn)
        self._stop_btn = QPushButton("■ Stop")
        self._stop_btn.clicked.connect(self._stop.set)
        self._stop_btn.setEnabled(False)
        run.addWidget(self._stop_btn)
        run.addStretch(1)
        self._load_btn = QPushButton("📂 Load panel…")
        self._load_btn.clicked.connect(self._load_panel)
        run.addWidget(self._load_btn)
        root.addLayout(run)

        self._status = QLabel("")
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(self._status)

        grid_host = QWidget()
        self._grid = QGridLayout(grid_host)
        self._grid.setSpacing(s(6))
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_host)
        root.addWidget(scroll, 1)
        self._rebuild_grid()
        self._refresh_hw_gate()

    def _parse_sizes(self):
        out = []
        for tok in (self._sizes.text() or "").replace(";", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                v = float(tok)
            except ValueError:
                continue
            if 0.2 <= v <= 60.0:
                out.append(v)
        return out or [2.0, 5.0, 10.0]

    def _rebuild_grid(self):
        while self._grid.count():
            it = self._grid.takeAt(0)
            w = it.widget()
            if w is not None:
                w.deleteLater()
        self._cells.clear()
        self._labels.clear()
        sizes = self._parse_sizes()
        for col, size in enumerate(sizes):
            hdr = QLabel(f"{size:g} mm")
            hdr.setAlignment(Qt.AlignCenter)
            self._grid.addWidget(hdr, 0, col + 1)
        for row, shape in enumerate(self.SHAPES):
            name = QLabel(shape)
            name.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._grid.addWidget(name, row * 2 + 1, 0)
            for col, size in enumerate(sizes):
                cell = _CellView(shape, size)
                lbl = QLabel("")
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(f"font-size: {sf(8)}pt; "
                                  f"color: {COLORS['subtext0']};")
                self._grid.addWidget(cell, row * 2 + 1, col + 1)
                self._grid.addWidget(lbl, row * 2 + 2, col + 1)
                self._cells[(shape, size)] = cell
                self._labels[(shape, size)] = lbl

    def _refresh_hw_gate(self):
        ctrl = self._ctrl
        ok = bool(ctrl is not None
                  and getattr(ctrl, "is_xy_connected", False))
        why = ""
        if not ok:
            why = "XY stage not connected"
        elif getattr(ctrl, "is_zp_connected", False) and self._safe_z is None:
            ok = False
            why = "Safe Z not set (needle would not be retracted)"
        self._hw_btn.setEnabled(ok and self._thread is None)
        self._hw_btn.setToolTip(why)

    # ── runs ──────────────────────────────────────────────────────
    def _start(self, source: str):
        if self._thread is not None:
            return
        self._rebuild_grid()
        self._records = []
        self._stop.clear()
        speed = float(self._speed.value())
        sizes = self._parse_sizes()
        char = StageCharacteristics.from_store(self._store)
        tuning = PS.tuning_from_store(self._store)
        if source == "simulated" and not char.is_complete():
            QMessageBox.information(
                self, "No stage model",
                "The stage's characteristics have not been measured yet — "
                "run the one-click XY calibration (Timing Calibration page) "
                "first.")
            return
        self._sim_btn.setEnabled(False)
        self._hw_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._status.setText(f"{source} panel running…")
        args = (source, speed, sizes, char, tuning)
        self._thread = threading.Thread(target=self._worker, args=args,
                                        daemon=True)
        self._thread.start()

    def _worker(self, source, speed, sizes, char, tuning):
        try:
            if source == "simulated":
                self._worker_sim(speed, sizes, char, tuning)
            else:
                self._worker_hw(speed, sizes, char, tuning)
        except Exception as e:               # pragma: no cover
            logger.exception("geometry panel worker failed")
            self._bridge.log.emit(f"panel failed: {e}")
        finally:
            self._bridge.done.emit()

    def _cell_record(self, source, shape, size, speed, ideal, samples,
                     rep, verdict, reason):
        return {
            "source": source, "shape": shape, "size_mm": size,
            "speed_mm_s": speed, "status": "ok",
            "ideal": [list(pt) for pt in ideal],
            "samples": [list(sm) for sm in samples],
            "actual": {k: rep.get(k) for k in
                       ("p95_um", "rms_um", "max_um", "dither_ratio",
                        "completion_frac", "corner_p95_um", "wall_s")},
            "actual_verdict": verdict, "actual_reason": reason,
        }

    def _worker_sim(self, speed, sizes, char, tuning):
        for shape in self.SHAPES:
            for size in sizes:
                if self._stop.is_set():
                    return
                ideal = XC.make_shape(shape, size, 0.5)
                r = PS.simulate_follow(ideal, char=char,
                                       print_speed_mm_s=speed, tuning=tuning)
                rec = self._cell_record("simulated", shape, size, speed,
                                        ideal, r.samples, r.report,
                                        r.verdict, r.stopped_reason)
                self._records.append(rec)
                self._bridge.cell.emit(rec)

    def _worker_hw(self, speed, sizes, char, tuning):
        ctrl = self._ctrl
        resolved = PS.resolve_for(char, print_speed_mm_s=speed, tuning=tuning)

        def _read_um():
            for _ in range(5):
                try:
                    p = ctrl.get_xy_position(cached=False)
                except Exception:
                    p = None
                if p and p[0] is not None:
                    return (float(p[0]), float(p[1]))
                time.sleep(0.05)
            return None

        def _goto_um(x_um, y_um, tol=80.0, tries=3):
            # verify by polling the actual position — wait_for_xy_arrival is
            # corrupted by the known Prior stale-ack bug
            for _ in range(tries):
                try:
                    ctrl.move_xy_absolute_um(x_um, y_um)
                except Exception:
                    return False
                t0 = time.monotonic()
                while time.monotonic() - t0 < 12.0:
                    if self._stop.is_set():
                        return False
                    p = _read_um()
                    if p and math.hypot(p[0] - x_um, p[1] - y_um) <= tol:
                        return True
                    time.sleep(0.2)
            return False

        if hasattr(ctrl, "suspend_position_poller"):
            ctrl.suspend_position_poller()
        if hasattr(ctrl, "suspend_zp_watchdog"):
            ctrl.suspend_zp_watchdog()
        try:
            ok, msg = AC.center_stage(ctrl, safe_z_mm=self._safe_z)
            if not ok:
                self._bridge.log.emit(f"centre failed: {msg} — stopped.")
                return
            zero = getattr(ctrl, "zero_position", {})
            zx, zy = zero.get("x", 0.0), zero.get("y", 0.0)
            centre = AC.envelope_center_um(ctrl)
            radius_um = AC.usable_test_radius_um(ctrl)
            ccx = (centre[0] - zx) / 1000.0
            ccy = (centre[1] - zy) / 1000.0
            xy = getattr(ctrl, "xy_stage", None)
            if xy is not None:
                if hasattr(xy, "set_acceleration"):
                    xy.set_acceleration(80)
                if hasattr(xy, "set_speed_mm_s"):
                    xy.set_speed_mm_s(resolved["max_um_s"] / 1000.0)
                time.sleep(0.2)
                _read_um()      # drain any stale ack from the SMS/SAS writes

            for shape in self.SHAPES:
                for size in sizes:
                    if self._stop.is_set():
                        return
                    if size * 1000.0 * 0.75 > radius_um:
                        self._bridge.log.emit(
                            f"{shape} {size:g}: does not fit — skipped")
                        continue
                    ideal = XC.offset_path(XC.make_shape(shape, size, 0.5),
                                           ccx, ccy)
                    total = PS.VC.polyline_arclength(ideal)[-1]
                    sx = ideal[0][0] * 1000.0 + zx
                    sy = ideal[0][1] * 1000.0 + zy
                    if not _goto_um(sx, sy):
                        self._bridge.log.emit(
                            f"{shape} {size:g}: start unreachable — skipped")
                        continue
                    samples, info = PS.follow_path(
                        ideal, PS.controller_io(ctrl),
                        print_speed_mm_s=speed, tuning=tuning,
                        resolved=resolved, stop=self._stop,
                        max_wall_s=min(90.0, total / speed * 6.0 + 15.0))
                    rep = XC.path_report(
                        samples, ideal, commanded_speed_mm_s=speed,
                        resolution_um=self._resolution_um(),
                        status="ok" if info["completed"]
                        else info["stopped_reason"],
                        wall_s=info["wall_s"])
                    verdict = XC.verdict_for(rep, self._resolution_um())
                    rec = self._cell_record(
                        "hardware", shape, size, speed,
                        [(pt[0] - ccx, pt[1] - ccy) for pt in ideal],
                        [(sm[0] - ccx, sm[1] - ccy) for sm in samples],
                        rep, verdict, info["stopped_reason"])
                    self._records.append(rec)
                    self._bridge.cell.emit(rec)
        finally:
            try:
                ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass
            if hasattr(ctrl, "resume_position_poller"):
                ctrl.resume_position_poller()
            if hasattr(ctrl, "resume_zp_watchdog"):
                ctrl.resume_zp_watchdog()

    def _resolution_um(self):
        try:
            return float(self._store.get_resolution_element_um() or 30.0)
        except Exception:
            return 30.0

    # ── bridge slots ──────────────────────────────────────────────
    def _on_cell(self, rec: dict):
        key = (rec["shape"], float(rec["size_mm"]))
        cell = self._cells.get(key)
        lbl = self._labels.get(key)
        if cell is None:
            return
        cell.set_run(rec.get("source", "hardware"), rec["ideal"],
                     rec["samples"], rec["actual_verdict"])
        a = rec["actual"]
        if lbl is not None:
            comp = a.get("completion_frac")
            extra = ("" if (comp is None or comp >= 0.99)
                     else f" · stopped at {comp*100:.0f}%")
            lbl.setText(f"p95 {a['p95_um']:.0f} µm · rms {a['rms_um']:.0f}"
                        f" · {rec['actual_verdict']}{extra}")
            lbl.setStyleSheet(
                f"font-size: {sf(8)}pt; color: "
                f"{COLORS[_VERDICT_COLOR.get(rec['actual_verdict'], 'red')]};")

    def _on_log(self, msg: str):
        self._status.setText(msg)

    def _on_done(self):
        if self._thread is not None:
            self._thread = None
        self._sim_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._refresh_hw_gate()
        n = len(self._records)
        if n:
            path = self._save_panel()
            self._status.setText(
                f"done — {n} cell(s); saved "
                f"{os.path.basename(path) if path else '(save failed)'}")
        elif not self._status.text():
            self._status.setText("done — no cells run")

    # ── persistence ───────────────────────────────────────────────
    def _save_panel(self):
        try:
            char = StageCharacteristics.from_store(self._store)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(_log_dir(), f"geometry_panel_{stamp}.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump({
                    "started": datetime.datetime.now().isoformat(
                        timespec="seconds"),
                    "speed_mm_s": float(self._speed.value()),
                    "tuning": PS.tuning_from_store(self._store),
                    "characteristics": char.to_dict(),
                    "resolution_um": self._resolution_um(),
                    "cells": self._records}, f)
            return path
        except Exception as e:               # pragma: no cover
            logger.warning(f"geometry panel save failed: {e}")
            return None

    def _load_panel(self):
        path, _f = QFileDialog.getOpenFileName(
            self, "Load geometry panel", _log_dir(), "Panel (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.load_panel_data(data)
            self._status.setText(f"loaded {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.warning(self, "Load failed", str(e))

    def load_panel_data(self, data: dict):
        """Display a saved panel (also used headlessly by tests)."""
        cells = data.get("cells") or []
        sizes = sorted({float(c["size_mm"]) for c in cells
                        if c.get("status", "ok") == "ok"})
        if sizes:
            self._sizes.setText(", ".join(f"{v:g}" for v in sizes))
        self._rebuild_grid()
        self._records = [c for c in cells if c.get("status", "ok") == "ok"]
        for c in self._records:
            # legacy hardware-script records carry no "source" field
            c.setdefault("source", "hardware")
            # panels recorded with world-frame coords: normalise about the
            # ideal's centroid so cells draw shape-relative
            ideal = c.get("ideal") or []
            if ideal:
                cx = sum(pt[0] for pt in ideal) / len(ideal)
                cy = sum(pt[1] for pt in ideal) / len(ideal)
                if abs(cx) > 1.0 or abs(cy) > 1.0:
                    c["ideal"] = [[pt[0] - cx, pt[1] - cy] for pt in ideal]
                    c["samples"] = [[sm[0] - cx, sm[1] - cy] + list(sm[2:])
                                    for sm in (c.get("samples") or [])]
            self._on_cell(c)

    # ── lifecycle ─────────────────────────────────────────────────
    def closeEvent(self, ev):
        if self._thread is not None and self._thread.is_alive():
            self._stop.set()
            self._thread.join(timeout=3.0)
        try:
            if self._ctrl is not None and hasattr(self._ctrl,
                                                  "send_velocity_xy"):
                self._ctrl.send_velocity_xy(0.0, 0.0)
        except Exception:
            pass
        super().closeEvent(ev)
