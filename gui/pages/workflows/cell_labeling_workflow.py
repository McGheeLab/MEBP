"""cell_labeling_workflow.py — Cell Labeling / staining workflow page.

v7.5.x: A stain-in-place workflow, built on the same scaffold as the Cell
Targeting & Removal page (shared LiveTargetPicker + WorkspaceTargetView +
XZSideView + StandardJogContextPanel + the PickPlaceExecutor bridge). It is a
deliberate sibling of Cell Targeting & Removal, with two differences the
operator asked for:

  * the deposit AND the aspirate are both SLOW (a stain is delivered and
    withdrawn gently), and
  * the headline knob is the stain incubation time — how long the stain is
    allowed to develop on the cells.

There is NO placement target: the operator only selects the *regions to stain*
(the pick list — the place machinery is hidden via the picker's ``pick_only``
mode) and the *stain reagent* to use. For each region the needle:

    1. Loads the stain from its reagent well.
    2. Travels to the region and lowers to the label Z (just off the bottom).
    3. SLOWLY deposits a small column of stain (needle inner area × depth).
    4. Waits the user-defined incubation time.
    5. SLOWLY aspirates a multiple (default 2×) of the deposited volume back up.
    6. Travels to the waste well and dispenses it (there is no place target).

An optional needle prep brackets the loop at the start (waste → oil → wash →
buffer) and an optional clean at the end (waste → wash → buffer). The reagent
well + service wells are inherited from Hardware Setup → Ink (Reagent
Locations); all Z heights are expressed as a height above the calibrated plate
bottom and resolved polarity-safely. The waste well is ALWAYS required (the
post-incubation dump is core to the op, not part of the optional prep/clean).
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QSplitter, QMessageBox, QCheckBox, QSpinBox,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.live_target_picker import LiveTargetPicker
from gui.widgets.safe_travel_worker import SafeTravelWorker
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.workspace_target_view import WorkspaceTargetView
from gui.widgets.xz_side_view import XZSideView
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)
from gui.pages.workflows._fluorescence_overlay import (
    load_plate_fluor_overlay, plate_key_of,
)
from gui.pages.workflows import _reagent_prep
from gui.pages.workflows._reagent_prep import (
    SERVICE_ROLES, service_well_names, resolve_service_positions,
    needle_volume_uL, resolve_pickup_well,
)

from SupportClasses.PickAndPlaceManager import (
    OperationQueue, OperationType, PickPlaceExecutor, PickPlaceOperation,
    CellLabelingConfig,
)

logger = logging.getLogger(__name__)


class _ExecutorBridge(QObject):
    """Bridges PickPlaceExecutor callbacks (daemon thread) → Qt signals."""

    op_started = Signal(object)        # PickPlaceOperation
    op_completed = Signal(object)
    op_failed = Signal(object, str)
    progress = Signal(int, int, str)   # completed, total, message
    sub_step = Signal(object, str)     # operation, step text
    finished = Signal(bool)            # True if all completed, False otherwise


class CellLabelingWorkflowPage(QWidget):
    """Cell Labeling / staining workflow page.

    Signals:
        back_requested: User clicked the Back button.
    """

    back_requested = Signal()

    def __init__(
        self,
        controller,
        settings,
        camera_manager,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        # Calibration data state (pushed in by MainWindow).
        self._plate = None
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None
        self._z_references: dict[str, float | None] = {
            "replace_z": None, "max_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }

        # Left context panel — lazy, identical lifecycle to JogControlPage
        self._context_widget: StandardJogContextPanel | None = None

        self._executor: Optional[PickPlaceExecutor] = None
        self._exec_thread: Optional[threading.Thread] = None
        self._bridge = _ExecutorBridge()
        self._bridge.op_started.connect(self._on_op_started)
        self._bridge.op_completed.connect(self._on_op_completed)
        self._bridge.op_failed.connect(self._on_op_failed)
        self._bridge.progress.connect(self._on_progress)
        self._bridge.sub_step.connect(self._on_sub_step)
        self._bridge.finished.connect(self._on_finished)

        # v7.5.x FREEZE FIX: click-to-travel safe_travel_to runs on a worker
        # thread so a needle-down retract can't freeze the GUI (see the Jog page
        # / gui/widgets/safe_travel_worker.py). Separate from the executor thread.
        self._travel_worker = SafeTravelWorker(self)
        self._travel_worker.finished.connect(self._on_travel_finished)

        # Comprehensive settings popout (scrollable, saveable). Built eagerly so
        # config widgets exist for _current_config() / _on_start() + the tests.
        self._settings_dialog = WorkflowSettingsDialog(
            "cell_labeling", "Cell Labeling",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())

        # Main horizontal split:
        #   LEFT  — LiveTargetPicker (pick_only = regions to stain)
        #   RIGHT — vertical split: WorkspaceTargetView (top) + XZSideView (bottom)
        main_split = QSplitter(Qt.Horizontal, self)
        main_split.setChildrenCollapsible(False)

        self._picker = LiveTargetPicker(controller, camera_manager,
                                        pick_only=True)
        self._picker.picks_changed.connect(self._on_targets_changed)
        main_split.addWidget(self._picker)

        right = QSplitter(Qt.Vertical, self)
        right.setChildrenCollapsible(False)

        self._workspace_view = WorkspaceTargetView()
        try:
            self._workspace_view.set_safety_limits(controller.safety_limits)
        except Exception:
            pass
        self._workspace_view.position_clicked.connect(
            self._on_workspace_position_clicked)
        self._workspace_view.fast_travel_requested.connect(
            self._on_workspace_fast_travel_requested)
        ws_card = Card("XY Workspace", flush=True)
        ws_card.add_widget(self._workspace_view)
        right.addWidget(ws_card)

        self._xz_view = XZSideView()
        try:
            self._xz_view.set_safety_limits(controller.safety_limits)
        except Exception:
            pass
        self._xz_view.go_to_z_requested.connect(self._on_go_to_z_requested)
        xz_card = Card("Side View (XZ)", flush=True)
        xz_card.add_widget(self._xz_view)
        right.addWidget(xz_card)

        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 2)
        main_split.addWidget(right)
        main_split.setStretchFactor(0, 1)
        main_split.setStretchFactor(1, 1)
        outer.addWidget(main_split, stretch=1)

        outer.addWidget(self._build_run_row())

        # Periodic position refresh so the workspace + XZ tracks the stage.
        from PySide6.QtCore import QTimer
        self._pos_timer = QTimer(self)
        self._pos_timer.setInterval(200)
        self._pos_timer.timeout.connect(self._refresh_position_indicators)
        self._pos_timer.start()

        self._refresh_volume_label()
        self._update_button_state()
        self._settings_dialog.load_last()
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._update_settings_summary()

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)

        title = QLabel("Cell Labeling")
        title.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(14)}pt;"
            f"font-weight: 600;"
        )
        row.addWidget(title)

        hint = QLabel("Pick = regions to stain")
        hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(hint)

        self._settings_summary = QLabel("")
        self._settings_summary.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._settings_summary)

        row.addStretch(1)

        self._fluor_check = QCheckBox("🔬 Fluorescence")
        self._fluor_check.setToolTip(
            "Overlay the captured fluorescence mosaic(s) for this plate "
            "(from the Fluorescence Mosaic workflow).")
        self._fluor_check.toggled.connect(self._on_fluor_toggled)
        row.addWidget(self._fluor_check)

        settings_btn = QPushButton("⚙ Settings")
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.setToolTip(
            "Open the full, saveable settings for this workflow (stain reagent, "
            "incubation time, deposit/aspirate, prep/clean, locations).")
        settings_btn.clicked.connect(self._open_settings)
        row.addWidget(settings_btn)
        return row

    def _on_fluor_toggled(self, checked: bool):
        """Show/hide the persisted fluorescence overlay on the XY workspace."""
        if not checked:
            try:
                self._workspace_view.set_fluor_visible(False)
            except Exception:
                pass
            return
        ok = load_plate_fluor_overlay(
            self._workspace_view, plate_key_of(self._hw_config), visible=True)
        if not ok:
            self._fluor_check.setChecked(False)
            self._status.setText(
                "No fluorescence mosaic captured for this plate yet "
                "(run the Fluorescence Mosaic workflow).")

    # ── Settings popout ───────────────────────────────────────────

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        self._refresh_volume_label()
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._update_settings_summary()
        self._update_button_state()

    def _update_settings_summary(self):
        if not hasattr(self, "_settings_summary"):
            return
        try:
            deposit, aspirate = self._deposit_aspirate_uL()
            prep = "prep on" if self._prep_check.isChecked() else "prep off"
            self._settings_summary.setText(
                f"stain {self._fmt_dwell(self._dwell.value())} · deposit "
                f"{deposit:.3g}/aspirate {aspirate:.3g} µL · {prep}")
        except Exception:
            pass

    @staticmethod
    def _fmt_dwell(seconds: float) -> str:
        seconds = float(seconds)
        if seconds < 60:
            return f"{seconds:.0f}s"
        if seconds < 3600:
            return f"{seconds / 60:.1f}m"
        return f"{seconds / 3600:.1f}h"

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
        # ── Stain incubation (the headline knob) ──
        self._dwell = self._dspin(
            0.0, 14400.0, 300.0, " s", 0, 15.0,
            "How long the stain is allowed to develop on the cells before it is "
            "aspirated back off. The headline setting of this workflow.")
        self._dwell.valueChanged.connect(self._update_settings_summary)
        sec = dlg.add_section("Stain incubation")
        sec.add("dwell", "Stain time (incubation)", self._dwell, 300.0,
                "How long the stain develops before removal.")
        self._dwell_hint = QLabel("")
        self._dwell_hint.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._dwell_hint)

        # ── Region height ──
        self._label_z = self._dspin(
            0.0, 20.0, 0.10, " mm", 2, 0.05,
            "Needle height above the plate bottom at the stain region.")
        sec = dlg.add_section("Region height")
        sec.add("label_z", "Label Z (↑ bottom)", self._label_z, 0.10)

        # ── Pump & stain reagent ──
        self._bore = QComboBox()
        self._bore.setMinimumWidth(s(110))
        self._bore.currentIndexChanged.connect(self._on_bore_changed)
        self._reagent_combo = QComboBox()
        self._reagent_combo.setMinimumWidth(s(180))
        self._reagent_combo.setToolTip(
            "Stain the needle is loaded with (e.g. a fluorescent dye). Listed "
            "reagents are library inks with a reagent location (Hardware "
            "Setup → Ink).")
        self._reagent_combo.currentIndexChanged.connect(
            lambda *_: (self._refresh_reagent_status(),
                        self._update_settings_summary()))
        self._reagent_z = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom when aspirating the stain.")
        sec = dlg.add_section("Pump & stain reagent")
        sec.add("bore", "Pump / bore", self._bore, "P1")
        # v7.9: states the bore-1-only restriction where the choice is made,
        # rather than letting a silent 6.75× dose error look like a preference.
        self._multi_bore_note = QLabel("")
        self._multi_bore_note.setWordWrap(True)
        self._multi_bore_note.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        self._multi_bore_note.setVisible(False)
        sec.add_widget(self._multi_bore_note)
        sec.add("reagent", "Stain reagent", self._reagent_combo, "")
        sec.add("reagent_z", "Stain dip Z (↑ bottom)", self._reagent_z, 0.50)
        self._reagent_status = QLabel("")
        self._reagent_status.setWordWrap(True)
        self._reagent_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._reagent_status)

        # ── Stain deposit / removal mechanics ──
        self._deposit_depth = self._dspin(
            0.001, 5.0, 0.10, " mm", 3, 0.05,
            "Stain column deposited = needle inner area × this depth.")
        self._deposit_depth.valueChanged.connect(self._refresh_volume_label)
        self._aspirate_mult = self._dspin(
            1.0, 20.0, 2.0, "", 2, 0.5,
            "Stain-removal volume = this multiple of the deposited volume.")
        self._aspirate_mult.valueChanged.connect(self._refresh_volume_label)
        self._deposit_speed = self._dspin(
            0.01, 50.0, 0.5, " µL/s", 2, 0.1,
            "SLOW flow used to deposit the stain (and the waste dispense).")
        self._aspirate_speed = self._dspin(
            0.01, 50.0, 0.5, " µL/s", 2, 0.1,
            "SLOW flow used to load and to aspirate the stain back off.")
        sec = dlg.add_section("Stain deposit / removal")
        sec.add("deposit_depth", "Deposit depth", self._deposit_depth, 0.10)
        sec.add("aspirate_mult", "Aspirate (×)", self._aspirate_mult, 2.0)
        sec.add("deposit_speed", "Deposit flow (slow)", self._deposit_speed, 0.5)
        sec.add("aspirate_speed", "Aspirate flow (slow)", self._aspirate_speed, 0.5)
        self._volume_label = QLabel("V = —")
        self._volume_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._volume_label)

        # ── Needle prep / clean ──
        self._prep_check = QCheckBox("Prep needle (waste → oil → wash → buffer)")
        self._prep_check.setChecked(True)
        self._prep_check.toggled.connect(self._on_prep_toggled)
        self._clean_check = QCheckBox("Clean after (waste → wash → buffer)")
        self._clean_check.setChecked(True)
        self._clean_check.toggled.connect(self._on_prep_toggled)
        self._wash_after_pickup_check = QCheckBox(
            "Wash needle after stain pickup (rinse exterior before deposit)")
        self._wash_after_pickup_check.setChecked(True)
        self._wash_after_pickup_check.setToolTip(
            "After aspirating the stain, dip + jiggle the needle at the wash "
            "well to rinse the stain film off its exterior before travelling to "
            "the region — so only the metered deposited volume reaches the "
            "cells. The aspirated stain stays in the bore.")
        self._wash_after_pickup_check.toggled.connect(self._on_prep_toggled)
        self._service_z = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom at the service + waste "
            "wells (also used for the post-incubation waste dump).")
        self._prep_rate = self._dspin(
            0.01, 50.0, 1.0, " µL/s", 2, 0.1, "Aspirate/dispense flow during prep/clean.")
        self._oil_needles = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5,
            "Needles of oil dispensed to waste / aspirated from oil.")
        self._buffer_needles = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5, "Needles of buffer drawn after the wash.")
        self._wash_cycles = self._ispin(
            0, 50, 3, "Dip-jiggle cycles at the wash well.")
        self._wash_z_amp = self._dspin(
            0.0, 10.0, 0.5, " mm", 2, 0.1, "How far up/down each wash jiggle moves.")
        self._wash_xy_amp = self._dspin(
            0.0, 5000.0, 200.0, " µm", 0, 10.0, "Random XY radius during the wash.")
        self._wash_dwell = self._dspin(
            0.0, 30.0, 0.3, " s", 2, 0.1, "Settle time between wash jiggles.")
        self._post_dispense = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5,
            "Needles of residual dispensed to waste during the post-clean.")
        sec = dlg.add_section("Needle prep / clean")
        sec.add_check("prep", self._prep_check, True)
        sec.add_check("clean", self._clean_check, True)
        sec.add_check("wash_after_pickup", self._wash_after_pickup_check, True)
        sec.add_note(
            "Prep values are shared defaults from Common Print Settings — tick "
            "Override to set a workflow-specific value.")
        sec.add_common("service_z", "Service / waste dip Z (↑ bottom)", self._service_z, 0.50)
        sec.add_common("prep_rate", "Prep / clean flow", self._prep_rate, 1.0)
        sec.add_common("oil_needles", "Oil (needles)", self._oil_needles, 1.0)
        sec.add_common("buffer_needles", "Buffer (needles)", self._buffer_needles, 1.0)
        sec.add_common("wash_cycles", "Wash cycles", self._wash_cycles, 3)
        sec.add_common("wash_z_amp", "Wash Z jiggle", self._wash_z_amp, 0.5)
        sec.add_common("wash_xy_amp", "Wash XY jiggle", self._wash_xy_amp, 200.0)
        sec.add_common("wash_dwell", "Wash settle", self._wash_dwell, 0.3)
        sec.add("post_dispense", "Clean dispense (needles)", self._post_dispense, 1.0)
        self._prep_status = QLabel("")
        self._prep_status.setWordWrap(True)
        self._prep_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._prep_status)

        # ── Motion & timeouts ──
        self._intra_retract = self._dspin(
            0.0, 20.0, 1.0, " mm", 2, 0.1, "Short Z retract for moves within one well.")
        self._z_timeout = self._dspin(
            1.0, 120.0, 15.0, " s", 0, 1.0, "Z move arrival timeout.")
        self._xy_timeout = self._dspin(
            1.0, 240.0, 30.0, " s", 0, 1.0, "XY move arrival timeout.")
        sec = dlg.add_section("Motion & timeouts (advanced)")
        sec.add("intra_retract", "Intra-well retract", self._intra_retract, 1.0)
        sec.add("z_timeout", "Z timeout", self._z_timeout, 15.0)
        sec.add("xy_timeout", "XY timeout", self._xy_timeout, 30.0)

        # ── Common — Pump (global) ──
        self._g_settle = self._dspin(0.0, 30.0, 0.0, " s", 2, 0.05)
        self._g_prime = self._dspin(0.0, 30.0, 0.25, " s", 2, 0.05)
        sec = dlg.add_section("Common — Pump (global, shared by all workflows)")
        sec.add_note(
            "Global pump values (edited here or on the Common Print Settings "
            "page — one value used everywhere).")
        sec.add_common("g_settle", "Dwell after syringe moves", self._g_settle,
                       0.0, common_key="pump_settle_time_s", overridable=False)
        sec.add_common("g_prime", "Prime time", self._g_prime, 0.25,
                       common_key="pump_prime_time_s", overridable=False)

        # ── Locations & Hardware (read-only) ──
        dlg.add_info_section()
        dlg.set_info_refresher(self._build_locations_panel)
        dlg.finalize()

    def _build_locations_panel(self):
        extras = []
        try:
            reagent = self._selected_reagent()
            well = self._reagent_source_well()
            if reagent:
                extras.append(("Stain reagent",
                               f"{reagent} ← {well or '(no well)'}"))
        except Exception:
            pass
        return build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z, extras=extras)

    def _on_prep_toggled(self, *_):
        prep_clean = self._prep_check.isChecked() or self._clean_check.isChecked()
        wash_ap = self._wash_after_pickup_check.isChecked()
        any_service = prep_clean or wash_ap
        # NOTE: _service_z is deliberately NOT disabled — the post-incubation
        # waste dump always needs the service dip Z, even with prep/clean off.
        # Wash mechanics are shared by prep/clean AND the after-pickup wash.
        for w in (self._wash_cycles, self._wash_z_amp, self._wash_xy_amp,
                  self._wash_dwell):
            w.setEnabled(any_service)
        # Oil / buffer / prep flow / post-dispense are prep/clean-only.
        for w in (self._prep_rate, self._oil_needles, self._buffer_needles,
                  self._post_dispense):
            w.setEnabled(prep_clean)
        self._refresh_prep_status()
        self._update_settings_summary()

    def _on_bore_changed(self, *_):
        self._refresh_reagent_combo()
        self._refresh_reagent_status()

    # ── Stain reagent resolution (mirror of Quick Print's ink) ──

    def _bore_id(self) -> str:
        return self._bore.currentText() or "P1"

    def _reagent_inks(self) -> list[str]:
        """Library inks that have a reagent location and are printable
        (NOT a service reagent — by ink_type or by name)."""
        hw = self._hw_config
        if hw is None:
            return []
        lib = getattr(hw, "ink_library", {}) or {}
        locs = getattr(hw, "ink_locations", {}) or {}
        out: list[str] = []
        for name, spec in lib.items():
            itype = (getattr(spec, "ink_type", "") or "").strip().lower()
            if itype in SERVICE_ROLES or (name or "").strip().lower() in SERVICE_ROLES:
                continue
            if locs.get(name):  # must have at least one reagent location
                out.append(name)
        return out

    def _bore_assigned_ink(self) -> str | None:
        """The first ink assigned to the currently-selected bore, if any."""
        hw = self._hw_config
        if hw is None:
            return None
        pcfg = (getattr(hw, "pumps", {}) or {}).get(self._bore_id())
        names = getattr(pcfg, "ink_names", []) if pcfg is not None else []
        return names[0] if names else None

    def _refresh_reagent_combo(self) -> None:
        if not hasattr(self, "_reagent_combo"):
            return
        prev = self._reagent_combo.currentData()
        reagents = self._reagent_inks()
        self._reagent_combo.blockSignals(True)
        self._reagent_combo.clear()
        self._reagent_combo.addItem("(none — needle already loaded)", "")
        for name in reagents:
            self._reagent_combo.addItem(name, name)
        assigned = self._bore_assigned_ink()
        target = assigned if assigned in reagents else (
            prev if prev in reagents else None)
        if target:
            idx = self._reagent_combo.findData(target)
            if idx >= 0:
                self._reagent_combo.setCurrentIndex(idx)
        self._reagent_combo.blockSignals(False)

    def _selected_reagent(self) -> str | None:
        if not hasattr(self, "_reagent_combo"):
            return None
        return self._reagent_combo.currentData() or None

    def _reagent_source_well(self) -> str | None:
        ink = self._selected_reagent()
        if not ink or self._hw_config is None:
            return None
        wells = (getattr(self._hw_config, "ink_locations", {}) or {}).get(ink) or []
        # Prefer a real sub-well over a flattened rosette parent (e.g. "A2").
        return resolve_pickup_well(wells, self._plate)

    def _reagent_source_pos(self) -> tuple[float, float] | None:
        wn = self._reagent_source_well()
        wells = self._well_positions or {}
        if wn and wn in wells:
            return wells[wn]
        return None

    def _refresh_reagent_status(self):
        if not hasattr(self, "_reagent_status"):
            return
        ink = self._selected_reagent()
        if not ink:
            self._reagent_status.setText(
                "⚠ No stain selected — pick the stain reagent.")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        well = self._reagent_source_well()
        pos = self._reagent_source_pos()
        if well is None:
            self._reagent_status.setText(
                f"⚠ Stain “{ink}” has no reagent location (Hardware Setup → Ink).")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        if pos is None:
            self._reagent_status.setText(
                f"⚠ Stain well {well} not in the calibrated plate — run Plate "
                "Location.")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        deposit, _aspirate = self._deposit_aspirate_uL()
        self._reagent_status.setText(
            f"✓ Stain “{ink}” ← {well} · load ~{deposit:.4f} µL on the "
            f"{self._bore_id()} bore")
        self._reagent_status.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {sf(9)}pt;")

    # ── Prep / clean service-well resolution (inherit from HW setup) ──

    def _refresh_prep_status(self):
        if not hasattr(self, "_prep_status"):
            return
        positions, missing = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        # The waste well is ALWAYS required (the post-incubation dump uses it),
        # even when prep/clean are off.
        if "waste" not in positions:
            self._prep_status.setText(
                "⚠ Assign + calibrate a waste well (Hardware Setup → Ink → "
                "Reagent Locations) — the stain is dumped there after each "
                "region.")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        if self._plate_offset_to_zref(float(self._service_z.value())) is None:
            self._prep_status.setText(
                "⚠ Plate bottom Z not calibrated — needed for the service / "
                "waste dip Z.")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        if not (self._prep_check.isChecked() or self._clean_check.isChecked()):
            names = service_well_names(self._hw_config, self._plate)
            wash_ap = self._wash_after_pickup_check.isChecked()
            if wash_ap and "wash" not in positions:
                self._prep_status.setText(
                    "⚠ Wash-after-pickup needs a wash well assigned + "
                    "calibrated (Hardware Setup → Ink → Reagent Locations).")
                self._prep_status.setStyleSheet(
                    f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
                return
            wash_note = (f" · wash after pickup={names.get('wash', '?')}"
                         if wash_ap else "")
            self._prep_status.setText(
                f"Prep + clean disabled — staining only "
                f"(waste={names.get('waste', '?')}{wash_note}).")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            return
        needle_uL = needle_volume_uL(self._hw_config)
        names = service_well_names(self._hw_config, self._plate)
        if needle_uL <= 0:
            self._prep_status.setText(
                "⚠ Needle inner Ø / length not set (Hardware Setup → Needle).")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        if missing:
            self._prep_status.setText(
                f"⚠ Assign + calibrate wells for: {', '.join(missing)} "
                f"(Hardware Setup → Ink → Reagent Locations).")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        mapping = "  ".join(f"{r}={names.get(r, '?')}" for r in SERVICE_ROLES)
        self._prep_status.setText(
            f"✓ 1 needle = {needle_uL:.3f} µL  ·  {mapping}")
        self._prep_status.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {sf(9)}pt;")

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))

        self._start_btn = QPushButton("Start staining")
        self._start_btn.clicked.connect(self._on_start)
        row.addWidget(self._start_btn)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._on_abort)
        row.addWidget(self._abort_btn)

        row.addStretch(1)

        self._status = QLabel("Idle.")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)

        return frame

    # ── Required by MainWindow ────────────────────────────────────

    def get_page_title(self) -> str:
        return "Cell Labeling"

    def get_sub_page_title(self) -> str:
        return "Cell Labeling"

    def get_context_widget(self) -> QWidget:
        """Lazy `StandardJogContextPanel` — same as the Jog page."""
        if self._context_widget is None:
            self._context_widget = StandardJogContextPanel(
                controller=self._controller,
                settings=self._settings,
                show_connect=False,
                bypass_safety=False,
            )
            if self._hw_config is not None:
                self._context_widget.set_hardware_config(self._hw_config)
            if any(v is not None for v in
                   (self._plate, self._well_positions, self._safe_z)):
                self._context_widget.set_calibration_data(
                    self._plate, self._well_positions, self._safe_z)
            if (hasattr(self._context_widget, "set_z_references")
                    and any(v is not None
                            for v in self._z_references.values())):
                try:
                    self._context_widget.set_z_references(self._z_references)
                except Exception:
                    pass
        return self._context_widget

    def on_status_update(self) -> None:
        """Forwarded from MainWindow's ~300ms tick."""
        if self._context_widget is not None and hasattr(
                self._context_widget, "on_status_update"):
            self._context_widget.on_status_update()

    def set_settings(self, settings) -> None:
        self._settings = settings
        if self._context_widget is not None:
            self._context_widget.set_settings(settings)

    def hideEvent(self, event):
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        super().hideEvent(event)

    # ── Common Print Settings hook ────────────────────────────────

    def set_common_print_settings(self, common):
        if getattr(self, "_settings_dialog", None) is not None:
            self._settings_dialog.set_common(common)

    # ── hw_config hook ────────────────────────────────────────────

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        # Refresh bore options from pump ids
        self._bore.blockSignals(True)
        previous = self._bore.currentText()
        self._bore.clear()
        # v7.9: on a multi-bore assembly, offer ONLY the pump feeding bore 1.
        # This workflow positions bore 1 and sizes its stain column from bore 1's
        # orifice area, so any other pump would over/under-dose by the bore-area
        # ratio (6.75× on the reference backpack) and deposit 100-500 µm off the
        # region. None ⇒ single bore or unknown ⇒ no restriction.
        datum = _reagent_prep.datum_bore_pump(hw_config)
        if hw_config is not None and hasattr(hw_config, "pumps"):
            for pid, pcfg in hw_config.pumps.items():
                enabled = getattr(pcfg, "enabled", True)
                configured = getattr(pcfg, "is_configured", True)
                if enabled and configured and (datum is None or pid == datum):
                    self._bore.addItem(pid)
        if self._bore.count() == 0:
            self._bore.addItem(datum or "P1")
        idx = self._bore.findText(previous)
        if idx >= 0:
            self._bore.setCurrentIndex(idx)
        self._bore.blockSignals(False)
        note = _reagent_prep.multi_bore_restriction_note(hw_config)
        if hasattr(self, "_multi_bore_note"):
            self._multi_bore_note.setText(note)
            self._multi_bore_note.setVisible(bool(note))

        # Push to the shared live picker + workspace/XZ needle size
        self._picker.set_hardware_config(hw_config)
        try:
            needle = getattr(hw_config, "needle", None) if hw_config else None
            # v7.6: the ORIFICE OD is what approaches the plate (the pulled tip
            # on a capillary), and the needle is barrel + tip long.
            od_um = float(getattr(needle, "orifice_od_um", None)
                          or getattr(needle, "od_um", 0.0) or 0.0)
            length_mm = float(getattr(needle, "total_length_mm", None)
                              or getattr(needle, "length_mm", 0.0) or 0.0)
            if od_um > 0:
                self._workspace_view.set_needle(od_um)
                self._xz_view.set_needle(od_um, length_mm or None)
        except Exception as e:
            logger.debug("workspace/xz set_needle failed: %s", e)

        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)
        self._refresh_reagent_combo()
        # The bore + reagent combos just (re)populated — apply any saved
        # selections that were pending because the combos were empty at load.
        try:
            self._settings_dialog.resolve_pending()
        except Exception:
            pass
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._refresh_volume_label()
        self._update_button_state()
        self._update_settings_summary()

    # ── Calibration data routing (mirror of JogControlPage) ──────

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if plate is not None:
            self._workspace_view.set_plate(plate)
            self._workspace_view.set_well_positions({}, "approximate")
        self._workspace_view.set_well_positions(
            self._wells_in_zero_ref(), "calibrated")
        try:
            self._xz_view.set_safe_z(safe_z)
        except Exception:
            pass
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, well_positions, safe_z)
        self._refresh_target_overlays()
        if getattr(self, "_fluor_check", None) is not None and self._fluor_check.isChecked():
            load_plate_fluor_overlay(
                self._workspace_view, plate_key_of(self._hw_config), visible=True)
        # Reagent + service-well resolution depends on the calibrated positions.
        self._refresh_reagent_status()
        self._refresh_prep_status()

    def set_z_references(self, refs: dict) -> None:
        if not isinstance(refs, dict):
            return
        for k in self._z_references.keys():
            if k in refs:
                self._z_references[k] = refs[k]
        try:
            self._xz_view.set_z_references(self._z_references)
        except Exception:
            pass
        if self._context_widget is not None and hasattr(
                self._context_widget, "set_z_references"):
            try:
                self._context_widget.set_z_references(self._z_references)
            except Exception:
                pass
        # Plate-bottom datum affects the resolvable reagent/service dip Z.
        self._refresh_reagent_status()
        self._refresh_prep_status()

    def _wells_in_zero_ref(self) -> dict[str, tuple[float, float]]:
        if not self._well_positions:
            return {}
        try:
            zero = self._controller.zero_position
        except Exception:
            return {}
        return {
            name: (wx - zero["x"], wy - zero["y"])
            for name, (wx, wy) in self._well_positions.items()
        }

    # ── Target overlays on the XY workspace ──────────────────────

    def _on_targets_changed(self, _targets):
        self._update_button_state()
        self._refresh_target_overlays()

    def _refresh_target_overlays(self):
        """Convert pick (region) targets from stage-frame µm to zero-ref µm and
        push them into the workspace view's overlay layer."""
        try:
            zero = self._controller.zero_position
        except Exception:
            zero = {"x": 0.0, "y": 0.0}

        picks_zr = [
            (t.x_um - zero["x"], t.y_um - zero["y"], t.target_id)
            for t in self._picker.picks()
        ]
        self._workspace_view.set_pick_targets(picks_zr)
        self._workspace_view.set_place_targets([])

    # ── Stage position indicator refresh ─────────────────────────

    def _refresh_position_indicators(self):
        try:
            xy = self._controller.get_xy_position(cached=True)
            zero = self._controller.zero_position
        except Exception:
            return
        if hasattr(self._workspace_view, "set_zero_offset"):
            self._workspace_view.set_zero_offset(zero["x"], zero["y"])
        if hasattr(self._xz_view, "set_zero_offset_x"):
            self._xz_view.set_zero_offset_x(zero["x"])
        if hasattr(self._xz_view, "set_zero_offset_z"):
            self._xz_view.set_zero_offset_z(zero.get("Z", 0.0))
        if xy is not None and xy[0] is not None and xy[1] is not None:
            zx_um = float(xy[0]) - zero["x"]
            zy_um = float(xy[1]) - zero["y"]
            self._workspace_view.set_position(zx_um, zy_um)

        try:
            zp = self._controller.get_zp_position(cached=True)
        except Exception:
            zp = None
        if zp is not None and zp[0] is not None:
            try:
                z_raw = self._controller.zp_logical_value(zp, "Z")
                if z_raw is not None and xy is not None and xy[0] is not None:
                    self._xz_view.set_position(
                        float(xy[0]) - zero["x"],
                        z_raw - zero["Z"],
                    )
            except Exception:
                pass

    # ── Workspace + XZ click handlers (mirror of JogControlPage) ──

    def _travel_blocked_by_run(self) -> bool:
        """True (+ shows a hint) if a workflow run is active — don't launch a
        manual click-to-travel on top of the executor thread (both drive the
        stage and toggle the non-refcounted poller suspend). Abort the run
        first. The Jog page owns no executor and needs no such guard."""
        t = getattr(self, "_exec_thread", None)
        if t is not None and t.is_alive():
            self._status.setText("Busy running — abort first to move manually.")
            return True
        return False

    def _on_workspace_position_clicked(
        self, x_um_zr: float, y_um_zr: float
    ) -> None:
        """Click-to-travel from the XY workspace (zero-ref µm)."""
        if not getattr(self._controller, "is_xy_connected", False):
            return
        if self._travel_blocked_by_run():
            return

        zero = self._controller.zero_position
        stage_x = x_um_zr + zero["x"]
        stage_y = y_um_zr + zero["y"]

        if self._safe_z is None:
            resp = QMessageBox.question(
                self, "No Safe Z",
                "No safe Z is configured. Travel XY without retracting Z?",
                QMessageBox.StandardButton.Yes
                | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if resp != QMessageBox.StandardButton.Yes:
                return
            self._controller.move_xy_absolute(
                x_um_zr / 1000.0, y_um_zr / 1000.0, from_zero_ref=True)
            return

        # Cross-position click-to-travel ALWAYS retracts via safe_travel_to
        # (polarity-safe; a near-no-op when the needle is already retracted).
        # v7.5.x FREEZE FIX: dispatch the blocking move to a worker thread so a
        # needle-down retract can't freeze the GUI; busy-guard ignores re-clicks.
        self._travel_worker.start(
            self._controller, stage_x, stage_y,
            safe_z_mm=self._safe_z, target_z_mm=None)

    def _on_workspace_fast_travel_requested(
        self, x_um_zr: float, y_um_zr: float
    ) -> None:
        """Right-click → Fast travel here: retract Z, travel XY, restore Z."""
        if not getattr(self._controller, "is_xy_connected", False):
            return
        if self._travel_blocked_by_run():
            return

        if self._safe_z is None:
            QMessageBox.warning(
                self, "No Safe Z",
                "Fast travel requires a safe Z height. Run the "
                "Calibration page first to set one.")
            return

        zero = self._controller.zero_position
        stage_x = float(x_um_zr) + zero["x"]
        stage_y = float(y_um_zr) + zero["y"]

        current_z_zr: float | None = None
        if getattr(self._controller, "is_zp_connected", False):
            zp = self._controller.get_zp_position(cached=True)
            if zp is not None and zp[0] is not None:
                try:
                    z_raw = self._controller.zp_logical_value(zp, "Z")
                    if z_raw is not None:
                        current_z_zr = z_raw - zero["Z"]
                except Exception:
                    current_z_zr = None

        # v7.5.x FREEZE FIX: worker thread (see _on_workspace_position_clicked).
        self._travel_worker.start(
            self._controller, stage_x, stage_y,
            safe_z_mm=self._safe_z, target_z_mm=current_z_zr)

    def _on_travel_finished(self, ok: bool) -> None:
        """Worker-thread click-to-travel finished (queued to the GUI thread)."""
        if not ok:
            logger.warning(
                "Click-to-travel did not confirm (Z retract / XY arrival timed "
                "out, or the ZP board is not connected).")

    def _on_go_to_z_requested(self, z_mm: float) -> None:
        """Z-reference badge in the XZ view → move_z_absolute (zero-ref mm)."""
        if not getattr(self._controller, "is_zp_connected", False):
            return
        try:
            self._controller.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("Go-to-Z failed: %s", exc)

    # ── config helpers ────────────────────────────────────────────

    def _needle_area_mm2(self) -> float:
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        if needle is None:
            return 0.0
        try:
            return float(getattr(needle, "cross_section_area_mm2", 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _deposit_aspirate_uL(self) -> tuple[float, float]:
        """(deposit, aspirate) volume in µL from the needle bore area × depth."""
        deposit = self._needle_area_mm2() * float(self._deposit_depth.value())
        aspirate = deposit * float(self._aspirate_mult.value())
        return deposit, aspirate

    def _current_config(self) -> CellLabelingConfig:
        deposit, _aspirate = self._deposit_aspirate_uL()
        return CellLabelingConfig(
            stain_bore=self._bore_id(),
            deposit_depth_mm=float(self._deposit_depth.value()),
            deposit_volume_uL=deposit,   # resolved from the needle (display + backstop)
            aspirate_multiplier=float(self._aspirate_mult.value()),
            stain_dwell_time_s=float(self._dwell.value()),
            deposit_speed_uL_s=float(self._deposit_speed.value()),
            aspirate_speed_uL_s=float(self._aspirate_speed.value()),
            label_z_offset_mm=float(self._label_z.value()),
        )

    def _plate_offset_to_zref(self, offset_mm: float) -> float | None:
        """Height above the calibrated plate bottom (mm) → zero-ref Z (mm),
        polarity-correct. Returns None if the plate bottom isn't calibrated."""
        ctrl = self._controller
        try:
            if hasattr(ctrl, "print_height_to_zref"):
                z = ctrl.print_height_to_zref(offset_mm)
                if z is not None:
                    return z
        except Exception:
            pass
        pb = self._z_references.get("plate_bottom_z")
        if pb is None:
            return None
        zdir = -1.0
        try:
            if hasattr(ctrl, "print_z_dir"):
                zdir = float(ctrl.print_z_dir())
            elif hasattr(ctrl, "z_up_sign"):
                zdir = float(ctrl.z_up_sign())
        except Exception:
            zdir = -1.0
        return pb + zdir * float(offset_mm)

    def _refresh_volume_label(self, *_):
        deposit, aspirate = self._deposit_aspirate_uL()
        if deposit <= 0:
            self._volume_label.setText("V = — (set needle inner Ø)")
        else:
            self._volume_label.setText(
                f"deposit {deposit:.4f} µL · aspirate {aspirate:.4f} µL")
        if hasattr(self, "_dwell_hint"):
            self._dwell_hint.setText(
                f"= {self._fmt_dwell(self._dwell.value())} of incubation per region.")
        if hasattr(self, "_reagent_status"):
            self._refresh_reagent_status()

    def _update_button_state(self, *_):
        has_regions = len(self._picker.picks()) > 0
        has_bore = self._bore.count() > 0
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        self._start_btn.setEnabled(has_regions and has_bore and not running)
        self._abort_btn.setEnabled(running)

    # ── Start / Abort ─────────────────────────────────────────────

    def _on_start(self):
        if self._exec_thread is not None and self._exec_thread.is_alive():
            return

        regions = self._picker.picks()
        if not regions:
            self._status.setText("Select at least one region to stain.")
            return

        cfg = self._current_config()

        if self._safe_z is None:
            self._status.setText(
                "No safe Z configured — set it on the Calibration page "
                "before staining.")
            return

        # The ZP (Z + pump) board is REQUIRED (retract + pumps). Refuse otherwise.
        if not getattr(self._controller, "is_zp_connected", False):
            self._status.setText(
                "ZP (Z + pump) board not connected — staining needs it to "
                "retract the needle and run the pumps. Reconnect it first.")
            return

        # v7.9 backstop: the combo is already restricted, but a settings profile
        # saved before the assembly changed can still carry another bore's pump.
        # Driving it would deposit the wrong volume in the wrong place.
        datum = _reagent_prep.datum_bore_pump(self._hw_config)
        if datum and str(cfg.stain_bore).strip().upper() != datum:
            self._status.setText(
                f"This workflow drives bore 1 ({datum}) on a multi-bore "
                f"assembly, but the saved pump is {cfg.stain_bore}. Its stain "
                f"volume and position are both resolved from bore 1, so another "
                f"bore would dose the wrong amount in the wrong place. Select "
                f"{datum}, or use Cell Targeting & Removal for a per-bore program.")
            return

        label_z = self._plate_offset_to_zref(cfg.label_z_offset_mm)
        if label_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — calibrate it on the "
                "Calibration page so the label height can be resolved.")
            return

        # The stain reagent is required: resolve its well + dip Z.
        deposit_uL, _aspirate_uL = self._deposit_aspirate_uL()
        if deposit_uL <= 0:
            self._status.setText(
                "Set the needle inner diameter (Hardware Setup → Needle) so the "
                "deposit volume (needle area × depth) can be computed.")
            return
        reagent = self._selected_reagent()
        if reagent is None:
            self._status.setText(
                "Select the stain reagent to load.")
            return
        reagent_pos = self._reagent_source_pos()
        if reagent_pos is None:
            self._status.setText(
                f"Stain “{reagent}” has no calibrated reagent well — assign it "
                "(Hardware Setup → Ink) and run Plate Location.")
            return
        reagent_dip_z = self._plate_offset_to_zref(float(self._reagent_z.value()))
        if reagent_dip_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — can't resolve the stain "
                "dip Z.")
            return

        # The waste well + its dip Z are ALWAYS required — the stain is dumped
        # there after each region's incubation (independent of prep/clean).
        service_positions, missing_service = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        waste_pos = service_positions.get("waste")
        if waste_pos is None:
            self._status.setText(
                "Assign + calibrate a waste well (Hardware Setup → Ink → "
                "Reagent Locations) — the stain is dumped there after each "
                "region.")
            return
        service_z = self._plate_offset_to_zref(float(self._service_z.value()))
        if service_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — can't resolve the service "
                "/ waste dip Z.")
            return

        # The after-pickup wash needs the wash well (independent of prep/clean).
        wash_after_pickup = self._wash_after_pickup_check.isChecked()
        if wash_after_pickup and "wash" not in service_positions:
            self._status.setText(
                "Wash-after-pickup needs a wash well assigned + calibrated "
                "(Hardware Setup → Ink → Reagent Locations), or turn it off.")
            return

        # Prep / clean inputs + gates (shared service wells).
        prep_enabled = self._prep_check.isChecked()
        clean_enabled = self._clean_check.isChecked()
        needle_uL = 0.0
        if prep_enabled or clean_enabled:
            needle_uL = needle_volume_uL(self._hw_config)
            if needle_uL <= 0:
                self._status.setText(
                    "Prep / clean needs the needle inner diameter + length "
                    "(Hardware Setup → Needle), or turn them off.")
                return
            if missing_service:
                self._status.setText(
                    "Prep / clean needs these reagent wells assigned in Hardware "
                    f"Setup → Ink (Reagent Locations) and calibrated: "
                    f"{', '.join(missing_service)} — or turn them off.")
                return

        queue = OperationQueue()
        for region in regions:
            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=OperationType.CELL_LABELING,
                source_target=region,
                dest_target=None,         # no placement — stain in place
                config=cfg,
            )
            queue.add(op)

        executor = PickPlaceExecutor(self._controller, self._hw_config)
        executor.safe_z_mm = float(self._safe_z)
        executor.pick_z_mm = label_z        # label height (reused field)
        executor.reagent_well_pos = reagent_pos
        executor.reagent_dip_z_mm = reagent_dip_z
        # Waste dump (always set — core to the op, not part of prep/clean).
        executor.waste_well_pos = waste_pos
        executor.service_z_mm = service_z
        # Advanced motion / timeout knobs (always applied).
        executor.intra_well_retract_mm = float(self._intra_retract.value())
        executor.z_timeout_s = float(self._z_timeout.value())
        executor.xy_timeout_s = float(self._xy_timeout.value())
        # Wash mechanics + wash well — shared by prep/clean AND the after-pickup
        # wash (service_z is already set above for the waste dump).
        if prep_enabled or clean_enabled or wash_after_pickup:
            executor.wash_cycles = int(self._wash_cycles.value())
            executor.wash_z_amplitude_mm = float(self._wash_z_amp.value())
            executor.wash_xy_amplitude_um = float(self._wash_xy_amp.value())
            executor.wash_dwell_s = float(self._wash_dwell.value())
            executor.wash_well_pos = service_positions.get("wash")
        if prep_enabled or clean_enabled:
            executor.prep_bore = cfg.stain_bore
            executor.needle_volume_uL = needle_uL
            executor.prep_rate_uL_s = float(self._prep_rate.value())
            executor.oil_needles = float(self._oil_needles.value())
            executor.buffer_needles = float(self._buffer_needles.value())
            executor.post_dispense_needles = float(self._post_dispense.value())
            executor.oil_well_pos = service_positions.get("oil")
            executor.buffer_well_pos = service_positions.get("buffer")
        executor.do_prep = prep_enabled
        executor.do_post_clean = clean_enabled
        executor.wash_after_pickup = wash_after_pickup

        bridge = self._bridge
        executor.on_op_started = lambda op: bridge.op_started.emit(op)
        executor.on_op_completed = lambda op: bridge.op_completed.emit(op)
        executor.on_op_failed = lambda op, msg="": bridge.op_failed.emit(op, msg)
        executor.on_sub_step = lambda op, step: bridge.sub_step.emit(op, step)
        self._executor = executor

        def progress_cb(done, total, msg):
            bridge.progress.emit(done, total, msg)

        def worker():
            ok = False
            try:
                ok = executor.execute_queue(queue, on_progress=progress_cb)
            except Exception as e:
                logger.exception("PickPlaceExecutor crashed: %s", e)
                ok = False
            bridge.finished.emit(ok)

        n = len(regions)
        self._status.setText(
            f"Staining {n} region{'s' if n > 1 else ''}…")
        self._exec_thread = threading.Thread(
            target=worker, name="CellLabelingExecutor", daemon=True)
        self._exec_thread.start()
        self._update_button_state()

    def _on_abort(self):
        if self._executor is None:
            return
        try:
            self._executor._abort_flag.set()
        except Exception as e:
            logger.warning("abort flag set failed: %s", e)
        self._status.setText("Abort requested…")

    # ── Bridge slot handlers (main thread) ────────────────────────

    def _on_op_started(self, op):
        self._status.setText(f"{op.op_id}: starting…")

    def _on_op_completed(self, op):
        self._status.setText(f"{op.op_id}: complete.")

    def _on_op_failed(self, op, msg: str):
        self._status.setText(f"{op.op_id}: FAILED — {msg}")

    def _on_progress(self, done: int, total: int, msg: str):
        self._status.setText(f"[{done}/{total}] {msg}")

    def _on_sub_step(self, op, step: str):
        self._status.setText(f"{op.op_id}: {step}")

    def _on_finished(self, ok: bool):
        self._exec_thread = None
        self._executor = None
        self._status.setText("Done." if ok else "Stopped (aborted or failed).")
        self._update_button_state()
