"""print_calibrator_workflow.py — Print Calibrator workflow page (v7.20).

Helps the operator TUNE print settings by measuring them on the real machine
instead of typing a guess. The first (and, for now, only) calibration is the
**prime time**.

WHY
---
``build_well_plate_job`` dispenses a prime of ``flow * pump_prime_time_s`` just
before every print path, so material is already forming a bead when the path
starts. That number was a hand-typed global (default 0.25 s) that nothing
measured, yet it depends on syringe compliance, needle bore/length and ink
viscosity. Too small and the first millimetres of every print are missing; too
large and there is a blob at the start.

THE MEASUREMENT
---------------
Print one straight line A→B with the prime forced to 0. Ink does not emerge at A
— it emerges at some point C along the line, and the delay is exactly the prime
time that would have put ink at A::

    tau = prime_used_in_run + (distance A->C along the line) / velocity

The operator clicks C on the live microscope view; the arithmetic (and every
refusal) lives in the pure :mod:`SupportClasses.PrimeTimeCalibration`.

DESIGN — THIS PAGE HOSTS A LIVE QUICK PRINT
-------------------------------------------
The calibration print is an ORDINARY print: same well selection, pump, ink,
needle prep, ink pickup, syringe-budget preflight, print-floor confirm,
preposition, ``PrintManager`` launch, abort and post-print cleanup. So rather
than re-implement any of that, this page mounts a live
:class:`QuickPrintWorkflowPage` (``embedded=True, owns_camera=False``) as its
"Print" step — the repo's existing embedded-instance pattern
(:class:`FullPrintWorkflowPage` wraps ``PrintingModePage``; the Spheroid page
hosts a live Fluorescence Mosaic page). There is exactly one print-launch
implementation and it cannot diverge.

What this page adds is only the calibration on top: it authors the line object
and pushes it via ``set_external_object``, forces the prime via
``set_prime_override_s``, and then measures.

TWO THINGS THAT ARE NOT OBVIOUS
-------------------------------
* **A and B are read back out of the finished run's execution log**, not
  recomputed from this page's own length/angle/offset arithmetic.
  ``path_start.points`` IS the path handed to the executor, so taking the
  endpoints from it makes a desync between "what printed" and "what is being
  measured" structurally impossible. The computed values are only a fallback for
  a pre-v7.7 log that recorded no points.
* **The stage does not stay where the print ended.** On a clean completion Quick
  Print launches the post-print cleanup immediately, which travels to waste →
  wash → oil. So this page never assumes the live view still shows the printed
  line: it waits for ``cleanup_finished`` / ``cleanup_running()`` before offering
  to drive anywhere, and the operator drives back to the line with an explicit
  button.
"""

from __future__ import annotations

import logging
import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QCheckBox, QRadioButton, QButtonGroup, QStackedWidget,
    QSplitter, QFrame, QMessageBox, QGridLayout,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card, SectionHeader
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.safe_travel_worker import SafeTravelWorker
from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage

from SupportClasses import PrimeTimeCalibration as ptc

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:                                        # pragma: no cover
    CameraRole = None

logger = logging.getLogger(__name__)

#: Keep the line (and any offset) this far inside the well wall, on top of the
#: needle radius, so a calibration line can never scrape the wall.
WALL_MARGIN_MM = 0.25

#: Default calibration-line geometry.
DEFAULT_LENGTH_MM = 4.0
DEFAULT_STEP_MM = 1.0
#: Perpendicular spacing between the two calibration lines.
DEFAULT_GAP_MM = 1.0

#: Shortest line worth printing. Below this there is not enough bead to see where
#: the ink starts — at a few mm/s it is a fraction of a typical prime time — so an
#: offset that leaves only a sliver is refused rather than clamped down to it.
MIN_USABLE_LINE_MM = 0.5

#: Live-view overlay hues for the calibration line. A fourth and fifth hue, kept
#: clear of the A / B / C dots (green / blue / mauve): the dashed PLANNED line has
#: to read as "not printed yet" and so is deliberately subdued, while the solid
#: line is what the last run actually commanded. Catppuccin Mocha overlay1 / teal;
#: ``COLORS`` carries neither, and two sites already reach for ``teal`` with this
#: same literal fallback.
_PLANNED_HEX = COLORS.get("overlay1", "#7f849c")
_PRINTED_HEX = COLORS.get("teal", "#94e2d5")


class PrintCalibratorWorkflowPage(QWidget):
    """Print Calibrator — measure print settings on the real machine.

    Signals:
        back_requested: the operator clicked "← Back to Workflows".
        sub_page_changed: the Print/Measure step changed, so the MainWindow can
            refresh the top-bar title.
    """

    back_requested = Signal()
    sub_page_changed = Signal(int)

    #: The steps, in the pill strip's order.
    _STEPS = (("print", "1 · Print"), ("measure", "2 · Measure"))

    #: The calibrations this page offers. Only prime time is implemented; the
    #: tuple is the extension seam — a second entry needs an object-builder and
    #: a Measure panel, not another page.
    _CALIBRATIONS = (("prime_time", "Prime time"),)

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None
        self._common = None

        # Calibration data (pushed in by the WorkflowsModePage fan-out). Held
        # here as well as in the embedded page because this page needs the plate
        # (well radius) and the safe Z (its own "go to A/B" travel) — it receives
        # them from the SAME fan-out, so this is not a second path to one fact.
        self._plate = None
        self._well_positions = None
        self._safe_z: float | None = None

        # Live camera state (this page OWNS the microscope for the whole shell —
        # see QuickPrintWorkflowPage's `owns_camera` docstring).
        self._camera_started_by_us = False

        # ── Measurement state ──
        #: One entry per printed line, in print order. Each carries the COMMANDED
        #: endpoints in ABSOLUTE stage µm, that line's OWN velocity (the samples
        #: restart per segment, so a shared one would be wrong), the clicked ink
        #: start, and its result.
        self._lines: list[dict] = []
        self._run_prime_used_s = 0.0
        self._run_flow_uL_s = 0.0
        self._run_hop: dict = {}
        #: Did the wash/clean prep run before the measured print? Line 1 is
        #: DEFINED as the cold start after it, so this is part of the meaning.
        self._prep_ran = False
        self._run_provenance = ""
        #: The cold-start vs restart comparison, once BOTH lines are marked.
        self._compare: ptc.PrimeComparison | None = None
        #: What a live-view click does next: None / "mark:<i>". A click can ONLY
        #: record where the ink started — it can never move the line, whose
        #: position is computed from the Length / Angle / Offset spins.
        self._arm = None

        self._travel = SafeTravelWorker(self)
        self._travel.finished.connect(self._on_travel_finished)

        # ── The embedded, live Quick Print ──
        self._qp = QuickPrintWorkflowPage(
            controller, settings, camera_manager=camera_manager,
            embedded=True, owns_camera=False,
            settings_id="print_calibrator", settings_title="Print Calibrator")
        self._qp.print_state_changed.connect(self._on_print_state)
        self._qp.cleanup_finished.connect(self._on_cleanup_finished)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(8))
        outer.addLayout(self._build_header())
        outer.addWidget(self._build_line_card())
        outer.addWidget(self._build_step_strip())
        outer.addWidget(self._build_steps(), stretch=1)

        self._push_line_object()
        self._apply_run_mode()
        self._refresh_result()

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)

        title = QLabel("Print Calibrator")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(14)}pt; font-weight: 600;")
        row.addWidget(title)
        row.addStretch(1)

        row.addWidget(QLabel("Calibration:"))
        self._cal_combo = QComboBox()
        for key, label in self._CALIBRATIONS:
            self._cal_combo.addItem(label, key)
        self._cal_combo.setToolTip(
            "Which print setting to measure. More calibrations will appear "
            "here; each one reuses this same print surface.")
        row.addWidget(self._cal_combo)
        return row

    @staticmethod
    def _dspin(lo, hi, val, suffix="", decimals=2, step=None, tip="") -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setDecimals(decimals)
        w.setValue(val)
        if suffix:
            w.setSuffix(suffix)
        if step is not None:
            w.setSingleStep(step)
        if tip:
            w.setToolTip(tip)
        return w

    def _build_line_card(self) -> QWidget:
        card = Card("Calibration line")

        grid = QWidget()
        g = QGridLayout(grid)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(s(10))
        g.setVerticalSpacing(s(4))

        self._length_spin = self._dspin(
            0.2, 40.0, DEFAULT_LENGTH_MM, " mm", 2, 0.5,
            "Length of the single straight line to print. It must be long "
            "enough that ink starts WITHIN it — if nothing appears, lengthen "
            "the line or lower the print speed.")
        self._angle_spin = self._dspin(
            -180.0, 180.0, 0.0, " °", 0, 15.0,
            "Direction of the line within the well, measured from +X.")
        self._off_x_spin = self._dspin(
            -20.0, 20.0, 0.0, " mm", 2, 0.5,
            "Offset of the line's MIDPOINT from the well centre. Move it "
            "between runs so a new line does not print on top of the last one.")
        self._off_y_spin = self._dspin(-20.0, 20.0, 0.0, " mm", 2, 0.5,
                                       self._off_x_spin.toolTip())
        for w in (self._length_spin, self._angle_spin,
                  self._off_x_spin, self._off_y_spin):
            w.valueChanged.connect(self._on_line_changed)

        g.addWidget(QLabel("Length"), 0, 0)
        g.addWidget(self._length_spin, 0, 1)
        g.addWidget(QLabel("Angle"), 0, 2)
        g.addWidget(self._angle_spin, 0, 3)
        g.addWidget(QLabel("Offset X"), 0, 4)
        g.addWidget(self._off_x_spin, 0, 5)
        g.addWidget(QLabel("Y"), 0, 6)
        g.addWidget(self._off_y_spin, 0, 7)
        g.setColumnStretch(8, 1)
        card.add_widget(grid)

        run_row = QWidget()
        rl = QHBoxLayout(run_row)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(s(8))
        rl.addWidget(QLabel("Run:"))
        self._mode_baseline = QRadioButton("Baseline — prime forced to 0")
        self._mode_baseline.setToolTip(
            "The measurement run. With no prime at all, the distance from the "
            "line start to where ink appears IS the prime time needed.")
        self._mode_verify = QRadioButton("Verify — use the configured prime")
        self._mode_verify.setToolTip(
            "Check an applied prime time. Ink should now start AT the line "
            "start; any residual is added to the prime, so re-running converges.")
        self._mode_baseline.setChecked(True)
        self._mode_group = QButtonGroup(run_row)
        self._mode_group.setExclusive(True)
        for b in (self._mode_baseline, self._mode_verify):
            self._mode_group.addButton(b)
            rl.addWidget(b)
        self._mode_group.buttonClicked.connect(lambda *_: self._apply_run_mode())

        self._second_check = QCheckBox("Second line (pump restart)")
        self._second_check.setChecked(True)
        self._second_check.setToolTip(
            "Two lines measure two different primes. Line 1 is the COLD START "
            "after the wash and clean steps, with a relaxed column. Between the "
            "lines the pump PAUSES while the needle lifts, repositions and "
            "lowers, so line 2 is the RESTART from a column that is still "
            "mostly pressurised — normally a shorter prime. One setting has to "
            "serve both, and the panel prices that.")
        self._second_check.toggled.connect(self._on_line_changed)
        rl.addWidget(self._second_check)
        self._gap_spin = self._dspin(
            0.1, 10.0, DEFAULT_GAP_MM, " mm", 2, 0.25,
            "Perpendicular spacing between the two calibration lines — far "
            "enough apart that the second bead is unmistakably its own.")
        self._gap_spin.valueChanged.connect(self._on_line_changed)
        rl.addWidget(self._gap_spin)

        self._step_check = QCheckBox("Step the offset after each run")
        self._step_check.setChecked(True)
        self._step_check.setToolTip(
            "After a completed run, move the line sideways by this much so the "
            "next calibration line prints on clean glass.")
        rl.addWidget(self._step_check)
        self._step_spin = self._dspin(0.1, 10.0, DEFAULT_STEP_MM, " mm", 2, 0.5)
        rl.addWidget(self._step_spin)
        rl.addStretch(1)
        card.add_widget(run_row)

        self._line_note = QLabel("")
        self._line_note.setWordWrap(True)
        self._line_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        card.add_widget(self._line_note)
        return card

    def _build_step_strip(self) -> QWidget:
        row = QWidget()
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(4))
        self._step_group = QButtonGroup(row)
        self._step_group.setExclusive(True)
        self._step_buttons: dict[str, QPushButton] = {}
        for key, text in self._STEPS:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setChecked(key == "print")
            b.setCursor(Qt.PointingHandCursor)
            b._step_key = key
            b.setStyleSheet(f"font-size: {sf(9)}pt; padding: {s(3)}px {s(12)}px;")
            self._step_group.addButton(b)
            self._step_buttons[key] = b
            lay.addWidget(b)
        self._step_group.buttonClicked.connect(
            lambda b: self.show_step(getattr(b, "_step_key", "print")))
        lay.addStretch(1)
        return row

    def _build_steps(self) -> QWidget:
        self._stack = QStackedWidget()
        self._stack.addWidget(self._qp)             # index 0 — "print"
        self._stack.addWidget(self._build_measure_step())   # index 1 — "measure"
        return self._stack

    def _build_measure_step(self) -> QWidget:
        split = QSplitter(Qt.Horizontal)

        self._feed = CameraFeedView(
            camera_manager=self._camera_manager, cam_idx=0,
            show_crosshair=True, label="No microscope camera",
            auto_orient=True)
        self._feed.clicked.connect(self._on_feed_clicked)
        split.addWidget(self._feed)

        panel = QWidget()
        pl = QVBoxLayout(panel)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(s(8))

        pl.addWidget(SectionHeader("Where did the ink start?"))
        self._instr = QLabel("")
        self._instr.setWordWrap(True)
        pl.addWidget(self._instr)

        self._prov_lbl = QLabel("")
        self._prov_lbl.setWordWrap(True)
        self._prov_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        pl.addWidget(self._prov_lbl)

        btns = QWidget()
        bl = QHBoxLayout(btns)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(s(6))
        self._goto_a_btn = QPushButton("⤵ Go to line 1 start")
        self._goto_a_btn.setToolTip(
            "Retract the needle and travel over the first line's start. The "
            "needle is never lowered by this button.")
        self._goto_a_btn.clicked.connect(lambda: self._goto_point(0, "a"))
        self._goto_b_btn = QPushButton("⤵ Go to line 2 start")
        self._goto_b_btn.setToolTip(
            "Travel over the second line's start (retracted). Only available "
            "when a second line was printed.")
        self._goto_b_btn.clicked.connect(lambda: self._goto_point(1, "a"))
        bl.addWidget(self._goto_a_btn)
        bl.addWidget(self._goto_b_btn)
        bl.addStretch(1)
        pl.addWidget(btns)

        btns2 = QWidget()
        bl2 = QHBoxLayout(btns2)
        bl2.setContentsMargins(0, 0, 0, 0)
        bl2.setSpacing(s(6))
        self._mark_btns: list[QPushButton] = []
        for i, (text, tip) in enumerate((
                ("◎ Mark line 1 (cold start)",
                 "Then click the live view where the FIRST line's bead begins — "
                 "the prime needed to START flow after the wash and clean "
                 "steps."),
                ("◎ Mark line 2 (restart)",
                 "Then click the live view where the SECOND line's bead begins "
                 "— the prime needed to RESTART after the pump paused for the "
                 "reposition."))):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setToolTip(tip)
            b.clicked.connect(lambda _c=False, k=i: self._arm_click(f"mark:{k}"))
            self._mark_btns.append(b)
            bl2.addWidget(b)
        self._clear_btn = QPushButton("✕ Clear marks")
        self._clear_btn.clicked.connect(self._clear_marks)
        bl2.addWidget(self._clear_btn)
        bl2.addStretch(1)
        pl.addWidget(btns2)

        # The line's LOCATION is never clicked. It is placed by the Length /
        # Angle / Offset spins above and merely DRAWN here, so the geometry the
        # executor prints and the geometry on screen are one computed thing —
        # see _refresh_markers.
        self._overlay_lbl = QLabel(
            "The live view overlays the calibration line where it sits on the "
            "plate: dashed = where the next run will print (move it with "
            "Offset X/Y), solid = the stroke the last run actually commanded. "
            "Jog around and the overlay stays on the glass.")
        self._overlay_lbl.setWordWrap(True)
        self._overlay_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        pl.addWidget(self._overlay_lbl)

        self._result_lbl = QLabel("")
        self._result_lbl.setWordWrap(True)
        self._result_lbl.setStyleSheet(f"font-size: {sf(10)}pt;")
        pl.addWidget(self._result_lbl)

        self._warn_lbl = QLabel("")
        self._warn_lbl.setWordWrap(True)
        self._warn_lbl.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        pl.addWidget(self._warn_lbl)

        self._apply_btn = QPushButton("Apply as prime time")
        self._apply_btn.setEnabled(False)
        self._apply_btn.setToolTip(
            "Write the measured prime time to the shared pump prime time, so "
            "every print uses it.")
        self._apply_btn.clicked.connect(self._on_apply)
        pl.addWidget(self._apply_btn)
        pl.addStretch(1)

        wrap = QFrame()
        wl = QVBoxLayout(wrap)
        wl.setContentsMargins(s(8), s(4), s(4), s(4))
        wl.addWidget(panel)
        split.addWidget(wrap)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        return split

    # ── Step navigation ───────────────────────────────────────────

    def show_step(self, key: str) -> None:
        order = [k for k, _ in self._STEPS]
        if key not in order:
            return
        idx = order.index(key)
        self._stack.setCurrentIndex(idx)
        b = self._step_buttons.get(key)
        if b is not None and not b.isChecked():
            b.setChecked(True)
        self.sub_page_changed.emit(idx)

    def current_step(self) -> str:
        order = [k for k, _ in self._STEPS]
        i = self._stack.currentIndex()
        return order[i] if 0 <= i < len(order) else "print"

    # ── Camera lifecycle (this page owns it for the whole shell) ───

    def _resolve_microscope_cam_idx(self) -> int:
        if self._hw_config is not None and CameraRole is not None:
            try:
                idx = self._hw_config.camera_for_role(CameraRole.MICROSCOPE)
                if idx is not None:
                    return int(idx)
            except Exception:
                pass
        return 0

    def _start_camera(self) -> None:
        if self._camera_manager is None:
            return
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            if self._feed.cam_idx != cam_idx:
                self._feed.set_camera(cam_idx)
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as e:
            logger.debug("Print Calibrator camera start failed: %s", e)

    def _stop_camera(self) -> None:
        if self._camera_manager is None or not self._camera_started_by_us:
            return
        try:
            self._camera_manager.stop(self._feed.cam_idx)
        except Exception as e:
            logger.debug("Print Calibrator camera stop failed: %s", e)
        finally:
            self._camera_started_by_us = False

    def showEvent(self, event):
        self._start_camera()
        self._push_line_object()
        super().showEvent(event)

    def hideEvent(self, event):
        self._stop_camera()
        super().hideEvent(event)

    # ── The calibration line ──────────────────────────────────────

    def _needle_od_mm(self) -> float:
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        try:
            from SupportClasses.PhysicalModels import needle_orifice_od_mm
            v = float(needle_orifice_od_mm(needle) or 0.0)
            return max(0.0, v)
        except Exception:
            return 0.0

    def _safe_radius_mm(self) -> float:
        """Radius the line must stay inside: the well radius less the needle
        radius and a wall margin. 0.0 when the well size is unknown, in which
        case no clamp is applied (there is nothing to clamp against)."""
        try:
            r_um = float(self._qp.well_radius_um() or 0.0)
        except Exception:
            r_um = 0.0
        if r_um <= 0:
            return 0.0
        return max(0.0, r_um / 1000.0 - self._needle_od_mm() / 2.0 - WALL_MARGIN_MM)

    def _line_geometry_mm(self):
        """The line as ``(a, b, clamped_length, note)`` in WELL-RELATIVE mm.

        The length is CLAMPED so both endpoints stay inside the safe radius, and
        the clamp is always stated: shortening changes the measurement range (a
        shorter line can fail to contain the ink start), so a silent clamp would
        be a silently different measurement.
        """
        length = float(self._length_spin.value())
        ang = math.radians(float(self._angle_spin.value()))
        ox, oy = float(self._off_x_spin.value()), float(self._off_y_spin.value())
        ux, uy = math.cos(ang), math.sin(ang)
        note = ""

        r_safe = self._safe_radius_mm()
        if r_safe > 0:
            off = math.hypot(ox, oy)
            if off >= r_safe:
                return None, None, 0.0, (
                    f"⚠ The offset ({off:.2f} mm) puts the line outside the "
                    f"usable well radius ({r_safe:.2f} mm) — reduce it.")
            # The endpoints are midpoint ± (L/2)·u. Keeping both inside a circle
            # of radius r about the well centre bounds the half-length by the
            # positive root of |m ± h·u| = r.
            proj = ox * ux + oy * uy
            perp2 = max(0.0, off * off - proj * proj)
            if perp2 >= r_safe * r_safe:
                return None, None, 0.0, (
                    f"⚠ The line's direction leaves the usable well radius "
                    f"({r_safe:.2f} mm) — reduce the offset.")
            # h_max > 0 is GUARANTEED here, so there is deliberately no
            # "h_max <= 0" branch: off² = proj² + perp² and the first guard
            # already established off < r_safe, hence proj² + perp² < r² and
            # sqrt(r² − perp²) > |proj| strictly. A third guard for it would be
            # unreachable code that no test could ever exercise (a mutation run
            # proved exactly that) — one enforcement point, not three.
            h_max = math.sqrt(r_safe * r_safe - perp2) - abs(proj)
            # What IS reachable: an offset just inside r_safe leaves room for
            # only a sliver. Refuse that here, where the operator can see why,
            # rather than printing a 0.02 mm "line" that the measurement would
            # later reject as degenerate with nothing pointing at the offset.
            if 2.0 * h_max < MIN_USABLE_LINE_MM:
                return None, None, 0.0, (
                    f"⚠ This offset leaves room for only "
                    f"{2.0 * h_max:.2f} mm of line inside the usable well "
                    f"radius ({r_safe:.2f} mm) — at least "
                    f"{MIN_USABLE_LINE_MM:.2f} mm is needed to see where the "
                    f"ink starts. Move the line closer to the well centre.")
            if length / 2.0 > h_max:
                clamped = 2.0 * h_max
                well = self._qp.selected_well() or "the well"
                note = (f"⚠ Line clamped to {clamped:.2f} mm (from "
                        f"{length:.2f} mm) to stay inside {well}. A shorter "
                        "line may not contain the ink start.")
                length = clamped

        h = length / 2.0
        a = (ox - h * ux, oy - h * uy)
        b = (ox + h * ux, oy + h * uy)
        return a, b, length, note

    def _second_line_enabled(self) -> bool:
        return bool(self._second_check.isChecked())

    def _line_pair_mm(self):
        """Both lines as ``[(a, b), …]`` in well-relative mm, plus the note.

        The second line is offset PERPENDICULAR to the first by the gap, and runs
        in the SAME direction — the measurement compares two identical strokes
        that differ only in what preceded them (an approach vs a mid-print hop).
        """
        a, b, length, note = self._line_geometry_mm()
        if a is None or b is None or length <= 0:
            return [], length, note
        pairs = [(a, b)]
        if self._second_line_enabled():
            gap = float(self._gap_spin.value())
            ang = math.radians(float(self._angle_spin.value()))
            nx, ny = -math.sin(ang), math.cos(ang)
            a2 = (a[0] - gap * nx, a[1] - gap * ny)
            b2 = (b[0] - gap * nx, b[1] - gap * ny)
            r_safe = self._safe_radius_mm()
            if r_safe > 0 and max(math.hypot(*a2), math.hypot(*b2)) > r_safe:
                note = (note + "  " if note else "") + (
                    f"⚠ The second line would leave the usable well radius "
                    f"({r_safe:.2f} mm) — reduce the gap or the offset. Only "
                    "one line will print, so the hop cannot be measured.")
            else:
                pairs.append((a2, b2))
        return pairs, length, note

    def _line_objects(self):
        """One object dict per line. Separate objects on purpose: Quick Print
        turns each into its own ``PRINT_PATH``, so ``build_well_plate_job``
        inserts the lift → hop → lower → re-prime between them — which IS the
        thing the second line measures."""
        pairs, length, note = self._line_pair_mm()
        self._line_note.setText(note)
        self._line_note.setStyleSheet(
            f"color: {COLORS['peach'] if note else COLORS['subtext0']}; "
            f"font-size: {sf(9)}pt;")
        if not pairs:
            return [], length
        # ~1 point per 0.1 mm, bounded, so the executor gets a real polyline
        # (a 2-point path has nothing to pace against mid-line).
        n = int(max(20, min(400, round(length / 0.1))))
        return [{
            "name": f"CalibrationLine{i + 1}",
            "object_type": "line",
            "params": {"x1": a[0], "y1": a[1], "x2": b[0], "y2": b[1],
                       "num_points": n},
            "source": "parametric",
        } for i, (a, b) in enumerate(pairs)], length

    def _push_line_object(self) -> None:
        objs, length = self._line_objects()
        if not objs:
            # Refuse rather than push a line that does not fit: with no object
            # the embedded page's own readiness blocks Print and says why.
            self._qp.set_external_object(None)
            return
        label = (f"⟂ Calibration line — {length:.2f} mm" if len(objs) == 1
                 else f"⟂ Calibration lines ×{len(objs)} — {length:.2f} mm "
                      f"+ hop")
        self._qp.set_external_objects(objs, label=label)

    def _on_line_changed(self, *_):
        self._push_line_object()
        self._refresh_markers()

    def _apply_run_mode(self) -> None:
        """Baseline forces the prime to 0; Verify hands the popout back."""
        baseline = self._mode_baseline.isChecked()
        self._qp.set_prime_override_s(0.0 if baseline else None)

    # ── A print finished ──────────────────────────────────────────

    def _on_print_state(self, st) -> None:
        """Terminal states arrive here. NOTE this can be re-entrant (see
        ``QuickPrintWorkflowPage.print_state_changed``), so nothing here does
        real work beyond reading state and repainting."""
        name = getattr(st, "name", str(st))
        # Two unthrottled live views plus the print's 10 Hz sampler would fight
        # for the GUI thread; the embedded page throttles its own feed, so
        # throttle ours in step with it.
        try:
            self._feed.set_throttled(name == "RUNNING")
        except Exception:
            pass
        if name != "COMPLETED":
            if name in ("ABORTED", "ERROR"):
                self._instr.setText(
                    f"The run {name.lower()} — nothing to measure. "
                    "Fix the cause and print again.")
                self._refresh_buttons()
            return
        self._capture_run()
        self._step_offset_for_next_run()
        self.show_step("measure")

    def _capture_run(self) -> None:
        """Read everything the measurement needs out of the finished run."""
        pump = self._qp.pump()
        # Captured from the embedded page rather than the log: the exec log does
        # not record whether the prep sequence ran, and it is the operator's own
        # checkbox that decides.
        try:
            self._prep_ran = bool(self._qp._prep_check.isChecked())
        except Exception:
            self._prep_ran = False
        log = None
        path = self._qp.last_log_path()
        if path is not None:
            try:
                from SupportClasses.PrintLogReader import read_log
                log = read_log(path)
            except Exception as e:
                logger.warning("Print Calibrator could not read %s: %s", path, e)

        # One record per PRINTED segment, each with its OWN velocity — the
        # samples restart per segment, so a single whole-log velocity would span
        # the reset AND the hop and be badly wrong.
        runs = ptc.segments_from_log(log)
        src = "the run's log"
        self._lines = []
        for seg in runs:
            a_mm, b_mm = seg.a_mm, seg.b_mm
            if a_mm is None or b_mm is None:
                fb = self._computed_endpoints_zref_mm(seg.index)
                if fb is None:
                    continue
                a_mm, b_mm = fb
                src = "this page's geometry (the log recorded no points)"
            self._lines.append({
                "index": seg.index,
                "a_um": self._zref_mm_to_abs_um(a_mm),
                "b_um": self._zref_mm_to_abs_um(b_mm),
                "velocity": seg.velocity_mm_s,
                "velocity_source": seg.velocity_source,
                "c_um": None,
                "result": None,
            })
        if not self._lines:
            # No log at all (or nothing usable): fall back to this page's own
            # geometry and the commanded speed, and say so.
            speed, flow, _prime_uL = self._qp.resolved_print_kinematics()
            src = "this page's geometry (no usable log)"
            for i in range(2 if self._second_line_enabled() else 1):
                fb = self._computed_endpoints_zref_mm(i)
                if fb is None:
                    continue
                self._lines.append({
                    "index": i,
                    "a_um": self._zref_mm_to_abs_um(fb[0]),
                    "b_um": self._zref_mm_to_abs_um(fb[1]),
                    "velocity": speed,
                    "velocity_source": ptc.VEL_COMMANDED,
                    "c_um": None, "result": None,
                })

        if log is not None:
            self._run_prime_used_s = ptc.prime_used_s_from_log(log, pump)
            self._run_flow_uL_s = ptc.flow_uL_s_from_log(log, pump)
            self._run_hop = ptc.hop_settings_from_log(log)
        else:
            _speed, flow, prime_uL = self._qp.resolved_print_kinematics()
            self._run_flow_uL_s = flow
            self._run_prime_used_s = (prime_uL / flow) if flow > 0 else 0.0
            self._run_hop = {}

        self._compare = None
        self._run_provenance = self._describe_run(pump, src)
        n = len(self._lines)
        self._instr.setText(
            "Click where each bead BEGINS on the live view (arm the matching "
            "“Mark” button first). Line 1 is the COLD START after wash & clean; "
            "line 2 is the RESTART after the pump paused for the reposition. "
            "Line 1 normally needs the longer prime, and one setting has to "
            "serve both. Use the “Go to” buttons to drive back over each line; "
            "you can jog freely, because the click is read against the live "
            "stage position."
            if n > 1 else
            "Click where the bead BEGINS on the live view (arm “Mark line 1” "
            "first). Tick “Second line” to also measure the pump restart after "
            "a pause.")
        self._refresh_result()

    def _describe_run(self, pump: str, endpoint_src: str) -> str:
        ink = "—"
        try:
            ink = self._qp._selected_ink() or "(none)"
        except Exception:
            pass
        bore = "—"
        try:
            needle = getattr(self._hw_config, "needle", None)
            if needle is not None:
                idu = (getattr(needle, "orifice_id_um", None)
                       or getattr(needle, "id_um", 0) or 0)
                bore = f"{float(idu):.0f} µm"
        except Exception:
            pass
        speeds = " / ".join(
            f"{ln['velocity']:.3f} ({ptc._velocity_phrase(ln['velocity_source'])})"
            for ln in self._lines) or "—"
        out = (f"Run: {speeds} mm/s · flow {self._run_flow_uL_s:.4f} µL/s · "
               f"prime used {self._run_prime_used_s:.3f} s · {pump} · "
               f"needle {bore} · ink {ink}")
        # Line 1 is defined as "flow start after wash & clean", so whether prep
        # actually ran is part of what the number MEANS — not a detail.
        out += "\n" + ("Needle prep (waste → oil → wash → buffer) RAN before "
                       "this print, so line 1 is a true cold start."
                       if self._prep_ran else
                       "⚠ Needle prep did NOT run before this print, so line 1 "
                       "is not a cold start after wash & clean — it measures "
                       "whatever state the needle was already in.")
        if len(self._lines) > 1 and self._run_hop:
            out += "\n" + ptc.describe_hop_settings(self._run_hop)
        return out + f"\nEndpoints from {endpoint_src}."

    def _step_offset_for_next_run(self) -> None:
        """Nudge the line sideways so the next run prints on CLEAN glass.

        The step clears the whole PAIR, not one line. With two lines the pair
        occupies the band ``[-gap, 0]`` perpendicular to the stroke, so a bare
        1 mm step at a 1 mm gap would put the next run's line 2 exactly on this
        run's line 1 — and a calibration line printed over an existing bead has
        no readable start, which is the one thing this workflow needs to see.
        """
        if not self._step_check.isChecked():
            return
        step = float(self._step_spin.value())
        if self._second_line_enabled():
            step += float(self._gap_spin.value())
        ang = math.radians(float(self._angle_spin.value()))
        # Perpendicular to the line, so a stepped line stays parallel. Positive,
        # i.e. away from the second line (which sits at -gap).
        nx, ny = -math.sin(ang), math.cos(ang)
        for spin, delta in ((self._off_x_spin, step * nx),
                            (self._off_y_spin, step * ny)):
            spin.blockSignals(True)
            spin.setValue(spin.value() + delta)
            spin.blockSignals(False)
        self._push_line_object()

    def _on_cleanup_finished(self, _err: str) -> None:
        self._refresh_buttons()

    # ── Frames ────────────────────────────────────────────────────

    def _zref_mm_to_abs_um(self, p) -> tuple[float, float] | None:
        try:
            zero = self._controller.zero_position
            return (float(p[0]) * 1000.0 + float(zero["x"]),
                    float(p[1]) * 1000.0 + float(zero["y"]))
        except Exception:
            return None

    def _abs_um_to_zref_mm(self, p) -> tuple[float, float] | None:
        try:
            zero = self._controller.zero_position
            return ((float(p[0]) - float(zero["x"])) / 1000.0,
                    (float(p[1]) - float(zero["y"])) / 1000.0)
        except Exception:
            return None

    def _computed_endpoints_zref_mm(self, index: int = 0):
        """Fallback A/B for line ``index`` in zero-ref mm, from this page's own
        arithmetic. Only used when the log recorded no points (pre-v7.7)."""
        pairs, _length, _note = self._line_pair_mm()
        centre = self._qp.well_center_zero_ref_mm()
        if centre is None or index >= len(pairs):
            return None
        a, b = pairs[index]
        return ((centre[0] + a[0], centre[1] + a[1]),
                (centre[0] + b[0], centre[1] + b[1]))

    def _live_stage_um(self) -> tuple[float, float] | None:
        try:
            pos = self._controller.get_xy_position(cached=True)
            if pos is None or pos[0] is None or pos[1] is None:
                return None
            return (float(pos[0]), float(pos[1]))
        except Exception:
            return None

    # ── Live-view clicks ──────────────────────────────────────────

    def _arm_click(self, what: str) -> None:
        self._arm = what if self._arm != what else None
        self._sync_arm_buttons()

    def _sync_arm_buttons(self) -> None:
        for i, b in enumerate(self._mark_btns):
            b.setChecked(self._arm == f"mark:{i}")

    def _on_feed_clicked(self, px_x: float, px_y: float) -> None:
        if self._arm is None:
            return
        pt = self._click_to_abs_um(px_x, px_y)
        if pt is None:
            self._warn_lbl.setText(
                "Could not convert that click — the camera needs a calibrated "
                "µm/px and the XY stage must be connected.")
            return
        what, self._arm = self._arm, None
        self._sync_arm_buttons()
        # A click can only ever record WHERE THE INK STARTED. It deliberately
        # cannot move the line: the line's position is computed from the Length /
        # Angle / Offset spins, and a click that rewrote them would make the
        # printed geometry and the on-screen geometry two different things.
        if what.startswith("mark:"):
            try:
                idx = int(what.split(":", 1)[1])
            except ValueError:
                return
            if 0 <= idx < len(self._lines):
                self._lines[idx]["c_um"] = pt
                self._lines[idx]["result"] = None
            self._refresh_result()

    def _click_to_abs_um(self, px_x: float, px_y: float):
        """RAW frame px → ABSOLUTE stage µm.

        ``get_xy_position`` already returns µm (``get_xy_position_mm`` is the one
        that divides by 1000) — the ×1000 here was a documented 1000× bug.
        The live ``image_size`` is passed because ``pixel_to_stage_offset``
        resolves µm/px against the frame width.
        """
        if self._camera_manager is None:
            return None
        try:
            img_w, img_h = self._feed.image_size
        except Exception:
            return None
        if not img_w or not img_h:
            return None
        stage = self._live_stage_um()
        if stage is None:
            return None
        try:
            dx_um, dy_um = self._camera_manager.pixel_to_stage_offset(
                self._feed.cam_idx, px_x, px_y, img_w, img_h)
        except Exception as e:
            logger.debug("pixel_to_stage_offset failed: %s", e)
            return None
        return (stage[0] + float(dx_um), stage[1] + float(dy_um))

    def _clear_marks(self) -> None:
        for ln in self._lines:
            ln["c_um"] = None
            ln["result"] = None
        self._compare = None
        self._refresh_result()

    # ── Result ────────────────────────────────────────────────────

    def _evaluate_line(self, ln: dict):
        """Evaluate one line's clicked ink start. ``None`` until it is marked."""
        if ln["a_um"] is None or ln["b_um"] is None or ln["c_um"] is None:
            return None
        a = self._abs_um_to_zref_mm(ln["a_um"])
        b = self._abs_um_to_zref_mm(ln["b_um"])
        c = self._abs_um_to_zref_mm(ln["c_um"])
        if a is None or b is None or c is None:
            return None
        return ptc.evaluate(
            a, b, c,
            velocity_mm_s=ln["velocity"],
            velocity_source=ln["velocity_source"],
            prime_used_s=self._run_prime_used_s,
            flow_uL_s=self._run_flow_uL_s)

    def _applicable_result(self):
        """The result the Apply button would use.

        With both lines measured this is the LARGER of the two, because the one
        ``pump_prime_time_s`` knob primes EVERY segment and so has to cover the
        worst of them."""
        oks = [ln["result"] for ln in self._lines
               if ln["result"] is not None and ln["result"].ok]
        if not oks:
            return None
        return max(oks, key=lambda r: r.prime_time_s)

    def _refresh_result(self) -> None:
        self._prov_lbl.setText(self._run_provenance)
        if not self._lines:
            self._result_lbl.setText("Print a calibration line first (step 1).")
            self._result_lbl.setStyleSheet(f"font-size: {sf(10)}pt;")
            self._warn_lbl.setText("")
            self._compare = None
            self._refresh_buttons()
            self._refresh_markers()
            return

        for ln in self._lines:
            ln["result"] = self._evaluate_line(ln)

        first = self._lines[0]["result"] if self._lines else None
        second = self._lines[1]["result"] if len(self._lines) > 1 else None
        self._compare = ptc.compare_primes(first, second)

        rows, warns, refused = [], [], False
        for i, ln in enumerate(self._lines):
            name = (f"Line {i + 1} — "
                    + ("restart after the pump pause" if i
                       else "flow start after wash & clean"))
            res = ln["result"]
            if res is None:
                rows.append(f"{name}: not marked yet.")
                continue
            rows.append(f"{name}: {res.message}")
            if res.refusal:
                refused = True
            elif res.warning:
                warns.append(f"{name}: {res.warning}")
        if self._compare is not None:
            rows.append("")
            rows.append(self._compare.message)
        self._result_lbl.setText("\n".join(rows))
        self._result_lbl.setStyleSheet(
            f"color: {COLORS['red']}; font-size: {sf(10)}pt;" if refused
            else f"font-size: {sf(10)}pt;")
        self._warn_lbl.setText("  ".join(warns))
        self._refresh_buttons()
        self._refresh_markers()

    def _refresh_buttons(self) -> None:
        busy = self._qp.is_printing() or self._qp.cleanup_running()
        tip = ("The needle is busy — wait for the print / cleanup to finish."
               if busy else "")
        for i, b in enumerate((self._goto_a_btn, self._goto_b_btn)):
            have = i < len(self._lines) and self._lines[i]["a_um"] is not None
            b.setEnabled(have and not busy)
            if tip:
                b.setToolTip(tip)
        for i, b in enumerate(self._mark_btns):
            b.setEnabled(i < len(self._lines))
        self._clear_btn.setEnabled(
            any(ln["c_um"] is not None for ln in self._lines))
        best = self._applicable_result()
        self._apply_btn.setEnabled(best is not None)
        if best is not None and self._compare is not None:
            self._apply_btn.setText(
                f"Apply {self._compare.required_s:.3f} s as prime time"
                + (" (the RESTART binds)" if self._compare.restart_binds
                   else ""))
        elif best is not None:
            self._apply_btn.setText(
                f"Apply {best.prime_time_s:.3f} s as prime time")
        else:
            self._apply_btn.setText("Apply as prime time")

    def _planned_pairs_abs_um(self):
        """The line pair as ABSOLUTE stage µm, from the spins — where the NEXT
        run will print. Pure arithmetic (no label is touched), because this runs
        on the ~300 ms status tick; ``_line_objects`` writes the note label and
        must not be called from here."""
        centre = self._qp.well_center_zero_ref_mm()
        if centre is None:
            return []
        pairs, _length, _note = self._line_pair_mm()
        out = []
        for a, b in pairs:
            pa = self._zref_mm_to_abs_um((centre[0] + a[0], centre[1] + a[1]))
            pb = self._zref_mm_to_abs_um((centre[0] + b[0], centre[1] + b[1]))
            if pa is not None and pb is not None:
                out.append((pa, pb))
        return out

    def _refresh_markers(self) -> None:
        """Draw the calibration line, and A / B / C, on the live view.

        The line's position is NEVER set from the view — it is computed from the
        Length / Angle / Offset spins and merely drawn here, at the absolute XY
        it occupies on the plate. Both overlays are pushed camera-centre-relative
        against the LIVE stage position and re-pushed on every status tick, so
        jogging slides them across the frame and they stay on the glass: the
        drawn stroke landing on the real bead is itself the check that the
        camera↔stage frame is right.

        Two overlays, because they answer different questions:
          * DASHED — the planned pair, i.e. where the next run will print. This
            is what replaces clicking to place the line: adjust Offset X/Y and
            watch it move over the actual glass.
          * SOLID — the pair the last run actually commanded (read back out of
            its exec log), with A/B dots at the ends and the clicked ink start C.

        ``set_stage_paths`` / ``set_bore_markers`` (camera-centre-relative µm)
        are used deliberately rather than ``set_reference_markers``: they route
        through ``stage_offset_to_pixel``, the exact inverse of the click path,
        so everything lands correctly on a rotated or mirrored camera, and one
        shared frame means the path, its end dots and a click on it cannot
        disagree. Both are change-gated, so pushing per tick is free.
        """
        feed = getattr(self, "_feed", None)
        if feed is None or not hasattr(feed, "set_bore_markers"):
            return
        stage = self._live_stage_um()
        marks, paths = [], []
        if stage is not None:
            def rel(pt):
                return (pt[0] - stage[0], pt[1] - stage[1])

            try:
                for a, b in self._planned_pairs_abs_um():
                    paths.append(((rel(a), rel(b)), _PLANNED_HEX, True))
            except Exception as e:
                logger.debug("planned path build failed: %s", e)

            for i, ln in enumerate(self._lines):
                n = i + 1
                if ln["a_um"] is not None and ln["b_um"] is not None:
                    paths.append(((rel(ln["a_um"]), rel(ln["b_um"])),
                                  _PRINTED_HEX, False))
                for label, pt, colour in (
                        (f"A{n}", ln["a_um"], COLORS["green"]),
                        (f"B{n}", ln["b_um"], COLORS["blue"]),
                        (f"C{n}", ln["c_um"], COLORS["mauve"])):
                    if pt is None:
                        continue
                    marks.append((label, pt[0] - stage[0],
                                  pt[1] - stage[1], colour))
        try:
            feed.set_bore_markers(marks)
        except Exception as e:
            logger.debug("marker push failed: %s", e)
        fn = getattr(feed, "set_stage_paths", None)
        if callable(fn):
            try:
                fn(paths)
            except Exception as e:
                logger.debug("path push failed: %s", e)

    # ── Motion (retract-only travel) ──────────────────────────────

    def _goto_point(self, line_index: int, which: str = "a") -> None:
        if line_index >= len(self._lines):
            return
        ln = self._lines[line_index]
        pt = ln["a_um"] if which == "a" else ln["b_um"]
        if pt is None:
            return
        if self._qp.is_printing() or self._qp.cleanup_running():
            self._warn_lbl.setText(
                "The needle is busy — wait for the print / cleanup to finish.")
            return
        # v7.19 proxy for "another sequence is driving the stage".
        try:
            if bool(self._controller.is_position_poller_suspended()):
                self._warn_lbl.setText(
                    "Another sequence is driving the stage — try again in a "
                    "moment.")
                return
        except Exception:
            pass
        if self._safe_z is None:
            self._warn_lbl.setText(
                "No Safe Z is calibrated, so the needle cannot be retracted "
                "for travel — set it on the Calibration page.")
            return
        # target_z_mm=None → raise → wait → XY → wait, and NEVER lower.
        if self._travel.start(self._controller, pt[0], pt[1],
                              safe_z_mm=float(self._safe_z), target_z_mm=None):
            self._warn_lbl.setText("")
            self._goto_a_btn.setEnabled(False)
            self._goto_b_btn.setEnabled(False)

    def _on_travel_finished(self, ok: bool) -> None:
        if not ok:
            self._warn_lbl.setText(
                "The travel did not confirm arrival — the view may not be over "
                "the line.")
        self._refresh_buttons()
        self._refresh_markers()

    # ── Apply ─────────────────────────────────────────────────────

    def _on_apply(self) -> None:
        res = self._applicable_result()
        if res is None:
            return
        # With both lines measured the required prime is the LARGER: one knob
        # primes every segment, so it has to cover the worst of them. Normally
        # that is the COLD START (line 1) — a restart after a mere pause needs
        # less — and the cost of the choice is spelled out either way.
        cmp_ = self._compare
        tau = float(cmp_.required_s if cmp_ is not None else res.prime_time_s)
        current = self._qp.prime_default_s()
        detail = cmp_.message if cmp_ is not None else res.message
        extra = ""
        if cmp_ is not None and cmp_.restart_binds:
            extra = ("\n\n⚠ The RESTART after a pump pause is what sets this "
                     "value — more than the cold start needs, which is the "
                     "wrong way round. Look at the quick-move pressure relief "
                     "and how long the pump is paused (the hop height and "
                     "speeds) before accepting it.")
        resp = QMessageBox.question(
            self, "Apply prime time",
            f"Set the shared pump prime time to {tau:.3f} s?\n\n"
            f"Currently {current:.3f} s.\n\n{detail}{extra}\n\n"
            f"{self._run_provenance}\n\n"
            "A prime time is specific to this needle, ink, syringe and flow "
            "rate — re-measure after changing any of them.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if resp != QMessageBox.StandardButton.Yes:
            return

        applied = False
        if self._common is not None:
            try:
                self._common.set("pump_prime_time_s", tau)
                applied = True
            except Exception as e:
                logger.warning("Applying prime time via common settings "
                               "failed: %s", e)
        if not applied:
            # No shared model was fanned out (a page built outside MainWindow).
            # Write the hardware config directly and SAY it will not persist,
            # rather than silently doing nothing.
            cfg = self._hw_config
            if cfg is not None:
                try:
                    cfg.pump_prime_time_s = tau
                    applied = True
                except Exception as e:
                    logger.warning("Applying prime time failed: %s", e)
            self._warn_lbl.setText(
                "Applied to the running session only — the shared settings "
                "model is not available, so this was not saved."
                if applied else "Could not apply the prime time.")
        else:
            self._warn_lbl.setText("")
        if not applied:
            return
        # Re-push the shared model so the embedded page's linked pre-flow field
        # re-syncs immediately, even if the app-level fan-out is not wired.
        if self._common is not None:
            try:
                self._qp.set_common_print_settings(self._common)
            except Exception:
                pass
        self._result_lbl.setText(
            f"Applied — pump prime time is now {tau:.3f} s. Run a Verify line "
            "to confirm ink starts at A.")
        self._mode_verify.setChecked(True)
        self._apply_run_mode()
        self._refresh_buttons()

    # ── Workflow-page contract ────────────────────────────────────

    def get_page_title(self) -> str:
        return "Print Calibrator"

    def get_sub_page_title(self) -> str:
        return ("Print Calibrator — Measure"
                if self.current_step() == "measure"
                else "Print Calibrator — Print")

    def get_context_widget(self):
        """Delegate to the embedded page's jog panel.

        One panel, not two: the operator needs to jog while hunting for the ink
        start, and the embedded page already builds (and ticks) exactly this
        widget."""
        try:
            return self._qp.get_context_widget()
        except Exception:
            return None

    def on_status_update(self) -> None:
        try:
            self._qp.on_status_update()
        except Exception:
            pass
        self._refresh_markers()
        self._refresh_buttons()

    def set_hardware_config(self, hw_config) -> None:
        self._hw_config = hw_config
        self._qp.set_hardware_config(hw_config)
        self._start_camera()
        self._push_line_object()

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        try:
            self._qp.set_calibration_data(plate, well_positions, safe_z)
        except Exception as e:
            logger.debug("calibrator set_calibration_data failed: %s", e)
        self._push_line_object()
        self._refresh_buttons()

    def set_z_references(self, refs) -> None:
        try:
            self._qp.set_z_references(refs)
        except Exception:
            pass

    def set_visible_z_references(self, keys) -> None:
        fn = getattr(self._qp, "set_visible_z_references", None)
        if callable(fn):
            try:
                fn(keys)
            except Exception:
                pass

    def set_common_print_settings(self, common) -> None:
        self._common = common
        try:
            self._qp.set_common_print_settings(common)
        except Exception:
            pass

    def set_settings(self, settings) -> None:
        self._settings = settings
        try:
            self._qp.set_settings(settings)
        except Exception:
            pass

    def set_well_list(self, wells) -> None:
        fn = getattr(self._qp, "set_well_list", None)
        if callable(fn):
            try:
                fn(wells)
            except Exception:
                pass

    def set_well_positions(self, positions) -> None:
        fn = getattr(self._qp, "set_well_positions", None)
        if callable(fn):
            try:
                fn(positions)
            except Exception:
                pass
