"""spheroid_sink_calibration.py — guided sink-timing + disengage calibration.

v7.5.x: A modeless dialog launched from the Spheroid Pick & Place settings popout
("Calibrate sink timing…"). Two panels:

  • Sink-timing staircase (the automatable part). Over ONE test spheroid the
    operator has jogged the needle above, the dialog aspirates a CUMULATIVE
    staircase of increasing volumes; after each pull the operator watches the
    live microscope and clicks "Visible at tip" when the spheroid has sunk back
    to the needle opening. Each step records (lift_mm = ΔV/bore_area, sink_s).
    The collected samples build the timing curve t_sink(lift), saved to the
    SpheroidSinkCalibrationStore. Runtime then inverts it to size each pickup
    aspirate so the spheroid finishes sinking as the needle arrives.

  • Disengage test (operator-judged; no auto vision detection). Runs a carrier
    aspirate then a trial disengage aspirate so the operator can see whether a
    stuck spheroid releases; the trial values can be copied into the settings.

Safety (mirrors PickPlaceExecutor / timing_calibration_workflow): refuses to run
without the ZP board, a Safe Z, a calibrated plate bottom, and the needle inner
Ø. All pump/Z actuation runs on a worker thread (never the GUI thread — that
freezes the camera). Every worker + Abort + hide retracts to Safe Z (the spheroid
is intentionally left at the tip BETWEEN accepted staircase steps for a quick
re-aspirate). Cross-position safety is inherited from safe_travel_to /
ensure_retracted_to.
"""

from __future__ import annotations

import logging
import threading
import time

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QSpinBox, QSplitter, QPlainTextEdit, QSizePolicy,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card

from SupportClasses.PickAndPlaceManager import _settled_pump_move
from SupportClasses.SpheroidSinkCalibrationStore import get_store, SinkCurve

try:
    from gui.widgets.camera_feed_view import CameraFeedView
except Exception:  # pragma: no cover
    CameraFeedView = None

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:  # pragma: no cover
    CameraRole = None

logger = logging.getLogger(__name__)


class _CalibBridge(QObject):
    """Worker-thread → GUI-thread signal bridge (queued across threads)."""
    status = Signal(str)
    armed = Signal(float, float)     # lift_mm, aspirated_uL (arm the sink timer)
    finished = Signal(bool, str)     # ok, message


class SinkDisengageCalibrationDialog(QDialog):
    """Guided sink-timing staircase + disengage test for spheroid pickup."""

    def __init__(self, page, *, parent=None):
        super().__init__(parent)
        self._page = page
        self.setModal(False)
        self.setWindowTitle("Calibrate — spheroid sink timing & disengage")
        self.resize(s(920), s(560))

        # Actuation / measurement state
        self._bridge = _CalibBridge()
        self._bridge.status.connect(self._on_status)
        self._bridge.armed.connect(self._on_armed)
        self._bridge.finished.connect(self._on_finished)
        self._thread: threading.Thread | None = None
        self._abort = threading.Event()
        self._busy = False               # a worker is running
        self._timing_active = False      # a step is armed, waiting for the click
        self._armed_t0: float | None = None
        self._armed_lift = 0.0
        self._samples: list[tuple[float, float]] = []
        self._step_index = 0
        self._cumulative_uL = 0.0
        # Resolved-at-start context
        self._bore = "P1"
        self._bore_area = 0.0
        self._pick_z = 0.0
        self._capacity_uL = 1e9
        self._start_vol = 0.5
        self._step_vol = 0.5
        self._n_steps = 5
        self._pull_rate = 1.0
        # Camera
        self._camera_view = None
        self._cam_started_by_us = False

        self._build_ui()
        self._update_buttons()

    # ── UI ────────────────────────────────────────────────────────

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

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        header = QHBoxLayout()
        title = QLabel("Spheroid sink timing & disengage")
        title.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {sf(13)}pt; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        self._status = QLabel("Jog the needle over a test spheroid, then Start.")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        header.addWidget(self._status, stretch=1)
        outer.addLayout(header)

        split = QSplitter(Qt.Horizontal, self)
        split.setChildrenCollapsible(False)

        # LEFT — live microscope
        if CameraFeedView is not None and self._page._camera_manager is not None:
            try:
                self._camera_view = CameraFeedView(
                    self._page._camera_manager,
                    cam_idx=self._resolve_microscope_cam_idx(),
                    label="Microscope — watch the spheroid sink to the tip",
                    enable_settings=False)
            except Exception as e:
                logger.debug("Sink calibration camera view unavailable: %s", e)
                self._camera_view = None
        if self._camera_view is not None:
            split.addWidget(self._camera_view)

        # RIGHT — the two calibration cards
        right = QSplitter(Qt.Vertical, self)
        right.setChildrenCollapsible(False)
        right.addWidget(self._build_sink_card())
        right.addWidget(self._build_disengage_card())
        right.setSizes([s(340), s(220)])
        split.addWidget(right)
        split.setSizes([s(440), s(480)])
        outer.addWidget(split, stretch=1)

        footer = QHBoxLayout()
        footer.addStretch(1)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.hide)
        footer.addWidget(close_btn)
        outer.addLayout(footer)

    def _build_sink_card(self) -> QWidget:
        card = Card("Sink-timing staircase", flush=True)
        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(s(10), s(8), s(10), s(10))
        v.setSpacing(s(8))

        note = QLabel(
            "Aspirates increasing volumes on one spheroid. After each pull, click "
            "“Visible at tip” when it has sunk back to the needle opening.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        v.addWidget(note)

        grid = QHBoxLayout()
        self._start_vol_spin = self._dspin(
            0.01, 100.0, 0.5, " µL", 3, 0.1, "First (smallest) pull volume.")
        self._step_vol_spin = self._dspin(
            0.0, 100.0, 0.5, " µL", 3, 0.1, "Extra volume added each step.")
        self._steps_spin = self._ispin(1, 30, 5, "Number of increasing pulls.")
        self._pull_rate_spin = self._dspin(
            0.01, 50.0, 1.0, " µL/s", 2, 0.5, "Aspirate flow for each pull.")
        for lbl, w in (("Start", self._start_vol_spin), ("Step", self._step_vol_spin),
                       ("Steps", self._steps_spin), ("Rate", self._pull_rate_spin)):
            col = QVBoxLayout()
            t = QLabel(lbl)
            t.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: {sf(8)}pt;")
            col.addWidget(t)
            col.addWidget(w)
            grid.addLayout(col)
        v.addLayout(grid)

        for w in (self._start_vol_spin, self._step_vol_spin,
                  self._pull_rate_spin):
            w.valueChanged.connect(self._refresh_lift_preview)
        self._steps_spin.valueChanged.connect(self._refresh_lift_preview)

        self._lift_preview = QLabel("")
        self._lift_preview.setWordWrap(True)
        self._lift_preview.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        v.addWidget(self._lift_preview)

        btns = QHBoxLayout()
        self._start_btn = QPushButton("Start")
        self._start_btn.clicked.connect(self._on_start_staircase)
        self._visible_btn = QPushButton("Visible at tip")
        self._visible_btn.clicked.connect(self._on_visible)
        self._abort_btn = QPushButton("Abort")
        self._abort_btn.clicked.connect(self._on_abort)
        for b in (self._start_btn, self._visible_btn, self._abort_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        v.addLayout(btns)

        self._samples_view = QPlainTextEdit()
        self._samples_view.setReadOnly(True)
        self._samples_view.setMaximumBlockCount(200)
        self._samples_view.setStyleSheet(
            f"QPlainTextEdit {{ background-color: {COLORS['mantle']};"
            f" color: {COLORS['subtext0']}; border: 1px solid "
            f"{COLORS['surface1']}; border-radius: 6px; font-family: "
            f"'Cascadia Code','Consolas',monospace; font-size: {sf(9)}pt; }}")
        v.addWidget(self._samples_view, stretch=1)

        save_row = QHBoxLayout()
        self._save_btn = QPushButton("Save curve")
        self._save_btn.clicked.connect(self._on_save_curve)
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._on_clear_samples)
        save_row.addWidget(self._save_btn)
        save_row.addWidget(self._clear_btn)
        save_row.addStretch(1)
        v.addLayout(save_row)

        card.add_widget(body)
        self._refresh_lift_preview()
        return card

    def _build_disengage_card(self) -> QWidget:
        card = Card("Disengage test", flush=True)
        body = QWidget()
        v = QVBoxLayout(body)
        v.setContentsMargins(s(10), s(8), s(10), s(10))
        v.setSpacing(s(8))

        note = QLabel(
            "Runs a carrier aspirate then a trial disengage aspirate so you can "
            "see if a stuck spheroid releases. Operator-judged — there is no "
            "automatic detection.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        v.addWidget(note)

        row = QHBoxLayout()
        self._trial_vol_spin = self._dspin(
            0.0, 200.0, 0.5, " µL", 3, 0.1, "Trial disengage aspirate volume.")
        self._trial_rate_spin = self._dspin(
            0.01, 50.0, 2.0, " µL/s", 2, 0.5, "Trial disengage aspirate flow.")
        for lbl, w in (("Volume", self._trial_vol_spin),
                       ("Rate", self._trial_rate_spin)):
            col = QVBoxLayout()
            t = QLabel(lbl)
            t.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: {sf(8)}pt;")
            col.addWidget(t)
            col.addWidget(w)
            row.addLayout(col)
        row.addStretch(1)
        v.addLayout(row)

        btns = QHBoxLayout()
        self._disengage_btn = QPushButton("Test disengage")
        self._disengage_btn.clicked.connect(self._on_test_disengage)
        self._copy_btn = QPushButton("Copy trial → settings")
        self._copy_btn.clicked.connect(self._on_copy_trial)
        btns.addWidget(self._disengage_btn)
        btns.addWidget(self._copy_btn)
        btns.addStretch(1)
        v.addLayout(btns)

        card.add_widget(body)
        # Seed the trial values from the page's current disengage settings.
        try:
            self._trial_vol_spin.setValue(float(self._page._disengage_vol.value()))
            self._trial_rate_spin.setValue(float(self._page._disengage_rate.value()))
        except Exception:
            pass
        return card

    # ── Camera lifecycle ──────────────────────────────────────────

    def _resolve_microscope_cam_idx(self) -> int:
        hw = self._page._hw_config
        if hw is not None and CameraRole is not None:
            try:
                idx = hw.camera_for_role(CameraRole.MICROSCOPE)
                if idx is not None:
                    return int(idx)
            except Exception:
                pass
        return 0

    def _start_live_feed(self):
        mgr = self._page._camera_manager
        if mgr is None or self._camera_view is None:
            return
        idx = self._resolve_microscope_cam_idx()
        try:
            if self._camera_view.cam_idx != idx:
                self._camera_view.set_camera(idx)
            if not mgr.is_running(idx):
                mgr.start(idx)
                self._cam_started_by_us = True
        except Exception as e:
            logger.debug("Sink calibration live feed start failed: %s", e)

    def _stop_live_feed(self):
        mgr = self._page._camera_manager
        if mgr is None or self._camera_view is None or not self._cam_started_by_us:
            return
        try:
            mgr.stop(self._camera_view.cam_idx)
        except Exception:
            pass
        finally:
            self._cam_started_by_us = False

    def showEvent(self, event):
        super().showEvent(event)
        self._start_live_feed()
        self._refresh_lift_preview()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._abort.set()
        self._stop_live_feed()
        # An accepted staircase step leaves the needle at the pick Z; retract on
        # a short daemon thread so hide never blocks on a pump/Z move.
        threading.Thread(target=self._safe_retract, name="SinkCalibRetract",
                         daemon=True).start()

    # ── Preflight / helpers ───────────────────────────────────────

    def _running(self) -> bool:
        return self._busy or self._timing_active

    def _syringe_capacity_uL(self) -> float:
        hw = self._page._hw_config
        try:
            pcfg = hw.pumps.get(self._bore)
            vol = getattr(getattr(pcfg, "syringe", None), "volume_uL", None)
            return float(vol) if vol else 1e9
        except Exception:
            return 1e9

    def _preflight(self):
        """Returns (pick_z, bore_area) or None (with a status message set)."""
        page = self._page
        ctrl = page._controller
        if not getattr(ctrl, "is_zp_connected", False):
            self._set_status("ZP (Z + pump) board not connected — reconnect it "
                             "first (needed to retract the needle and drive the pump).")
            return None
        if page._safe_z is None:
            self._set_status("No Safe Z calibrated — set it on the Calibration "
                             "page first.")
            return None
        pick_z = page._plate_offset_to_zref(float(page._pick_z.value()))
        if pick_z is None:
            self._set_status("Plate bottom Z is not calibrated — can't resolve "
                             "the pick height.")
            return None
        area = page._bore_area_mm2()
        if area <= 0.0:
            self._set_status("Needle inner Ø not set (Hardware Setup → Needle) — "
                             "can't compute the lift height.")
            return None
        return pick_z, area

    def _set_status(self, msg: str):
        try:
            self._status.setText(msg)
        except Exception:
            pass

    def _safe_retract(self):
        try:
            ctrl = self._page._controller
            safe = self._page._safe_z
            if getattr(ctrl, "is_zp_connected", False) and safe is not None:
                ctrl.ensure_retracted_to(float(safe))
        except Exception:
            pass

    def _refresh_lift_preview(self):
        area = self._page._bore_area_mm2()
        if area <= 0.0:
            self._lift_preview.setText("Needle inner Ø not set — set it to see lift.")
            return
        start = float(self._start_vol_spin.value())
        step = float(self._step_vol_spin.value())
        n = int(self._steps_spin.value())
        last = start + (n - 1) * step
        self._lift_preview.setText(
            f"Bore area {area:.5f} mm² · lifts "
            f"{start / area:.3f} … {last / area:.3f} mm "
            f"({n} pulls, cumulative {sum(start + i * step for i in range(n)):.2f} µL)")

    def _update_buttons(self):
        busy = self._busy
        timing = self._timing_active
        self._start_btn.setEnabled(not busy and not timing)
        self._visible_btn.setEnabled(timing and not busy)
        self._abort_btn.setEnabled(busy or timing)
        self._save_btn.setEnabled(bool(self._samples) and not busy and not timing)
        self._clear_btn.setEnabled(not busy and not timing)
        self._disengage_btn.setEnabled(not busy and not timing)
        self._copy_btn.setEnabled(not busy and not timing)

    # ── Sink staircase ────────────────────────────────────────────

    def _on_start_staircase(self):
        if self._running():
            return
        pf = self._preflight()
        if pf is None:
            return
        self._pick_z, self._bore_area = pf
        self._bore = self._page._bore.currentText() or "P1"
        self._capacity_uL = self._syringe_capacity_uL()
        self._start_vol = float(self._start_vol_spin.value())
        self._step_vol = float(self._step_vol_spin.value())
        self._n_steps = int(self._steps_spin.value())
        self._pull_rate = float(self._pull_rate_spin.value())
        self._samples = []
        self._step_index = 0
        self._cumulative_uL = 0.0
        self._samples_view.clear()
        self._abort.clear()
        self._start_live_feed()
        self._launch(lambda: self._stair_worker(0),
                     "Retracting, moving to the spheroid, aspirating step 1…")

    def _launch(self, target, status_msg):
        self._busy = True
        self._timing_active = False
        self._set_status(status_msg)
        self._update_buttons()
        self._thread = threading.Thread(target=self._guard(target),
                                        name="SinkCalibWorker", daemon=True)
        self._thread.start()

    def _guard(self, fn):
        def run():
            try:
                fn()
            except Exception as e:  # pragma: no cover - defensive
                logger.exception("Sink calibration worker crashed: %s", e)
                self._bridge.finished.emit(False, str(e))
                self._safe_retract()
        return run

    def _stair_worker(self, step_index: int):
        ctrl = self._page._controller
        safe = float(self._page._safe_z)
        if self._abort.is_set():
            return
        if step_index == 0:
            ctrl.ensure_retracted_to(safe)
            xy = ctrl.get_xy_position(cached=True)
            ctrl.safe_travel_to(target_x_um=xy[0], target_y_um=xy[1],
                                safe_z_mm=safe, target_z_mm=self._pick_z)
        if self._abort.is_set():
            return
        dv = self._start_vol + step_index * self._step_vol
        if self._cumulative_uL + dv > self._capacity_uL:
            self._bridge.finished.emit(
                False, f"Would exceed syringe capacity "
                       f"({self._capacity_uL:.1f} µL) — stopping and dumping.")
            self._dump_and_retract()
            return
        _settled_pump_move(ctrl, self._bore, -dv, rate_uL_s=self._pull_rate)
        self._cumulative_uL += dv
        if self._abort.is_set():
            return
        self._bridge.armed.emit(dv / self._bore_area, dv)

    def _stair_finish_worker(self):
        self._dump_and_retract()
        self._bridge.finished.emit(
            True, "Staircase complete — review the samples and Save curve.")

    def _dump_and_retract(self):
        """Dispense the accumulated fluid back in place, then retract."""
        try:
            ctrl = self._page._controller
            if self._cumulative_uL > 1e-9:
                _settled_pump_move(ctrl, self._bore, +self._cumulative_uL,
                                   rate_uL_s=self._pull_rate)
                self._cumulative_uL = 0.0
        except Exception:
            pass
        self._safe_retract()

    def _on_status(self, msg: str):
        self._set_status(msg)

    def _on_armed(self, lift_mm: float, vol_uL: float):
        self._busy = False
        self._armed_t0 = time.monotonic()
        self._armed_lift = float(lift_mm)
        self._timing_active = True
        self._set_status(
            f"Step {self._step_index + 1}/{self._n_steps}: aspirated {vol_uL:.3f} µL "
            f"(lift {lift_mm:.3f} mm). Click “Visible at tip” when it returns.")
        self._update_buttons()

    def _on_visible(self):
        if not self._timing_active or self._armed_t0 is None:
            return
        elapsed = time.monotonic() - self._armed_t0
        if elapsed <= 1e-3:
            self._set_status("Clicked too fast — re-run this step.")
            return
        self._samples.append((self._armed_lift, elapsed))
        self._samples_view.appendPlainText(
            f"lift {self._armed_lift:.3f} mm  →  {elapsed:.2f} s")
        self._timing_active = False
        self._armed_t0 = None
        self._step_index += 1
        self._update_buttons()
        if self._step_index >= self._n_steps:
            self._launch(self._stair_finish_worker,
                         "Dumping accumulated fluid and retracting…")
        else:
            self._launch(lambda: self._stair_worker(self._step_index),
                         f"Aspirating step {self._step_index + 1}…")

    def _on_finished(self, ok: bool, msg: str):
        self._busy = False
        self._timing_active = False
        self._set_status(msg)
        if ok and self._samples:
            try:
                rate = SinkCurve([[l, t] for l, t in self._samples]).effective_rate_mm_s()
                self._samples_view.appendPlainText(
                    f"— {len(self._samples)} sample(s), ~{rate:.3f} mm/s —")
            except Exception:
                pass
        self._update_buttons()

    def _on_save_curve(self):
        if not self._samples:
            self._set_status("No samples to save — run the staircase first.")
            return
        needle = getattr(self._page._hw_config, "needle", None)
        try:
            get_store().set_curve(
                [[l, t] for l, t in self._samples],
                bore_area_mm2=self._page._bore_area_mm2(),
                needle_gauge=getattr(needle, "gauge", None),
                needle_id_um=getattr(needle, "id_um", None),
                spheroid_diameter_um=float(self._page._diameter.value()),
            )
            self._set_status(f"Saved sink curve ({len(self._samples)} points).")
            try:
                self._page._refresh_sink_status()
            except Exception:
                pass
        except Exception as e:
            logger.exception("Save sink curve failed: %s", e)
            self._set_status(f"Save failed: {e}")

    def _on_clear_samples(self):
        if self._running():
            return
        self._samples = []
        self._step_index = 0
        self._samples_view.clear()
        self._set_status("Samples cleared.")
        self._update_buttons()

    # ── Disengage test ────────────────────────────────────────────

    def _on_test_disengage(self):
        if self._running():
            return
        pf = self._preflight()
        if pf is None:
            return
        self._pick_z, self._bore_area = pf
        self._bore = self._page._bore.currentText() or "P1"
        try:
            carrier = float(self._page._current_config().compute_volume_uL())
            carrier_rate = float(self._page._pick_flow.value())
        except Exception:
            carrier, carrier_rate = 0.0, 1.0
        diseng = float(self._trial_vol_spin.value())
        diseng_rate = float(self._trial_rate_spin.value())
        self._abort.clear()
        self._start_live_feed()
        self._launch(
            lambda: self._disengage_worker(carrier, carrier_rate, diseng, diseng_rate),
            "Aspirating carrier + trial disengage — observe release…")

    def _disengage_worker(self, carrier, carrier_rate, diseng, diseng_rate):
        ctrl = self._page._controller
        safe = float(self._page._safe_z)
        try:
            ctrl.ensure_retracted_to(safe)
            xy = ctrl.get_xy_position(cached=True)
            ctrl.safe_travel_to(target_x_um=xy[0], target_y_um=xy[1],
                                safe_z_mm=safe, target_z_mm=self._pick_z)
            if self._abort.is_set():
                return
            if carrier > 1e-9:
                _settled_pump_move(ctrl, self._bore, -carrier, rate_uL_s=carrier_rate)
            if self._abort.is_set():
                return
            if diseng > 1e-9:
                _settled_pump_move(ctrl, self._bore, -diseng, rate_uL_s=diseng_rate)
            self._bridge.finished.emit(
                True, "Disengage test done — observe whether the spheroid released.")
        finally:
            self._safe_retract()

    def _on_copy_trial(self):
        try:
            self._page._disengage_vol.setValue(float(self._trial_vol_spin.value()))
            self._page._disengage_rate.setValue(float(self._trial_rate_spin.value()))
            self._page._disengage_enabled.setChecked(True)
            self._page._on_settings_changed()
            self._set_status("Copied trial disengage values into the settings.")
        except Exception as e:
            self._set_status(f"Couldn't copy to settings: {e}")

    def _on_abort(self):
        self._abort.set()
        self._set_status("Aborting — retracting to Safe Z…")
        threading.Thread(target=self._safe_retract, name="SinkCalibAbortRetract",
                         daemon=True).start()
