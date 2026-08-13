"""spheroid_pickup_workflow.py — Spheroid Pick & Place workflow page.

v7.4.x: First functional workflow in the new Workflows mode. Composes:

    - Minimal spheroid config row (diameter, bore, safety factor)
    - LiveTargetPicker tool        (one shared camera view, pick + place lists)
    - WorkspaceTargetView          (top-down XY workspace, target overlays, click-to-travel)
    - XZSideView                   (side view, click Z-ref badges to move)
    - StandardJogContextPanel      (left context panel — identical to the Jog page's)
    - Start / Abort row + status   (wraps PickPlaceExecutor)

The XY workspace and XZ side view are the same widgets the Jog page
uses; click-to-travel uses the same `safe_travel_to` / `move_xy_absolute`
routing logic. Targets selected in the camera view are also rendered as
overlays on the XY workspace by converting stage-frame µm → zero-ref µm
through `controller.zero_position`.

On Start, the page builds one `PickPlaceOperation` per pick target (all
sharing the same place target + SpheroidPickupConfig), drops them in an
`OperationQueue`, and runs `PickPlaceExecutor.execute_queue()` on a
daemon thread. Executor callbacks are bridged back to the GUI via a
small QObject signal bridge so all UI updates land on the main thread.
"""

from __future__ import annotations

import dataclasses
import logging
import threading
from typing import Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QSplitter, QMessageBox, QCheckBox, QSpinBox,
    QTabWidget,
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
from gui.pages.workflows._reagent_prep import resolve_pickup_well

from SupportClasses.PickAndPlaceManager import (
    OperationQueue, OperationType, PickPlaceExecutor, PickPlaceOperation,
    PickPlaceTarget, SpheroidPickupConfig,
)

logger = logging.getLogger(__name__)


class _ExecutorBridge(QObject):
    """Bridges PickPlaceExecutor callbacks (daemon thread) → Qt signals.

    Executor invokes its callbacks from the worker thread. We hop to
    the GUI thread by emitting Qt signals (QueuedConnection by default
    across threads), so handlers run on the main loop.
    """

    op_started = Signal(object)        # PickPlaceOperation
    op_completed = Signal(object)
    op_failed = Signal(object, str)
    progress = Signal(int, int, str)   # completed, total, message
    sub_step = Signal(object, str)     # operation, step text
    finished = Signal(bool)            # True if all completed, False if aborted/error


class SpheroidPickupWorkflowPage(QWidget):
    """Spheroid Pick & Place workflow page.

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

        # Calibration data state (pushed in by MainWindow). Held here so
        # the lazily-created context panel and XZ view get it whenever
        # they're constructed.
        self._plate = None
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None
        self._z_references: dict[str, float | None] = {
            "replace_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }

        # Left context panel — lazy, identical lifecycle to JogControlPage
        self._context_widget: StandardJogContextPanel | None = None

        self._executor: Optional[PickPlaceExecutor] = None
        self._exec_thread: Optional[threading.Thread] = None
        # Survey-tab state (built in _build_survey_tab).
        self._scan_page = None
        self._survey = None
        self._mosaic_overlay = None
        self._crop_worker = None
        self._pending_crop: Optional[dict] = None
        self._sink_calib_dialog = None  # lazy SinkDisengageCalibrationDialog
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
        # the config widgets exist for _current_config() / _on_start() and the
        # existing tests; shown on demand via the header's ⚙ Settings button.
        self._settings_dialog = WorkflowSettingsDialog(
            "spheroid_pickup", "Spheroid Pick & Place",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())

        # Two tabs: the pick & place surface (unchanged), and the spheroid
        # survey (scan → detect → curate → transfer).
        self._tabs = QTabWidget(self)
        outer.addWidget(self._tabs, stretch=1)

        pick_tab = QWidget(self)
        pick_layout = QVBoxLayout(pick_tab)
        pick_layout.setContentsMargins(0, 0, 0, 0)
        pick_layout.setSpacing(s(10))

        # Main horizontal split:
        #   LEFT  — LiveTargetPicker (shared camera + pick/place lists)
        #   RIGHT — vertical split: WorkspaceTargetView (top) + XZSideView (bottom)
        main_split = QSplitter(Qt.Horizontal, self)
        main_split.setChildrenCollapsible(False)

        self._picker = LiveTargetPicker(controller, camera_manager)
        self._picker.picks_changed.connect(self._on_targets_changed)
        self._picker.places_changed.connect(self._on_targets_changed)
        self._picker.target_changed.connect(self._on_target_changed)
        self._picker.goto_requested.connect(self._on_target_goto)
        self._picker.pick_added.connect(self._on_pick_added)
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
        pick_layout.addWidget(main_split, stretch=1)
        pick_layout.addWidget(self._build_run_row())
        self._tabs.addTab(pick_tab, "Pick && Place")

        self._tabs.addTab(self._build_survey_tab(), "Spheroid survey")

        # Periodic position refresh so the workspace + XZ tracks the stage.
        from PySide6.QtCore import QTimer
        self._pos_timer = QTimer(self)
        self._pos_timer.setInterval(200)
        self._pos_timer.timeout.connect(self._refresh_position_indicators)
        self._pos_timer.start()

        self._update_button_state()
        # Restore the operator's last-used settings (combos resolve later, once
        # set_hardware_config populates the bore).
        self._settings_dialog.load_last()
        self._refresh_volume_label()
        self._on_prep_toggled()  # sets prep/clean widget enabled state + status

    # ── Survey tab ────────────────────────────────────────────────

    def _build_survey_tab(self) -> QWidget:
        """Scan → detect → curate → transfer.

        The left half is a real INSTANCE of the Fluorescence Mosaic page
        (``embedded=True`` drops only its back-button header), so there is
        exactly one single-well mosaic scan implementation in the app and this
        tab tracks any change made to that page automatically. It is the same
        "re-home, don't rewrite" pattern ``FullPrintWorkflowPage`` uses to host
        ``PrintingModePage``.
        """
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        from gui.widgets.spheroid_mosaic_items import SpheroidOverlay
        from gui.widgets.spheroid_survey_panel import SpheroidSurveyPanel

        wrap = QWidget(self)
        layout = QVBoxLayout(wrap)
        layout.setContentsMargins(0, 0, 0, 0)

        split = QSplitter(Qt.Horizontal, wrap)
        split.setChildrenCollapsible(False)

        self._scan_page = FluorescenceMosaicWorkflowPage(
            self._controller, self._settings, self._camera_manager,
            embedded=True)
        self._scan_page.mosaic_ready.connect(self._on_mosaic_ready)
        split.addWidget(self._scan_page)

        self._survey = SpheroidSurveyPanel()
        self._survey.set_context_provider(self._mosaic_context_for_channel)
        self._survey.set_fit_badge_provider(self._fit_badge)
        self._survey.goto_requested.connect(self._on_spheroid_goto)
        self._survey.transfer_requested.connect(self._on_transfer_to_picks)
        self._survey.crop_requested.connect(self._on_save_training_crop)
        self._survey.selection_changed.connect(self._on_survey_selection)
        self._survey.detections_changed.connect(self._refresh_mosaic_circles)
        split.addWidget(self._survey)

        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        layout.addWidget(split)

        # Editable circles live on the embedded page's own mosaic view, so the
        # operator surveys and edits in one place. Interactive-items mode frees
        # the left button for the circles and moves panning to the middle button.
        view = self._scan_page.mosaic_view()
        view.set_interactive_items(True)
        self._mosaic_overlay = SpheroidOverlay(view, parent=self)
        view.scene_clicked.connect(self._mosaic_overlay.on_scene_press)
        view.scene_dragged.connect(self._mosaic_overlay.on_scene_drag)
        view.scene_released.connect(self._mosaic_overlay.on_scene_release)
        self._mosaic_overlay.radius_changed.connect(
            lambda tid, r: self._survey.apply_radius_px(tid, r, commit=False))
        self._mosaic_overlay.radius_committed.connect(
            lambda tid, r: self._survey.apply_radius_px(tid, r, commit=True))
        self._mosaic_overlay.center_committed.connect(
            self._survey.apply_center_px)
        self._mosaic_overlay.circle_clicked.connect(
            self._survey.select_detection)
        self._mosaic_overlay.rim_fitted.connect(self._on_mosaic_rim_fitted)
        self._mosaic_overlay.empty_clicked.connect(self._on_mosaic_empty_click)
        return wrap

    def _mosaic_context_for_channel(self, channel=None):
        page = getattr(self, "_scan_page", None)
        if page is None:
            return None
        return page.mosaic_context(channel)

    def _on_mosaic_ready(self, _well: str):
        """A scan finished, or a well change loaded a saved mosaic."""
        ctx = self._mosaic_context_for_channel(None)
        self._survey.set_mosaic_context(ctx)
        self._refresh_mosaic_circles()

    def _refresh_mosaic_circles(self):
        """Redraw the editable circles from the survey panel's detections."""
        overlay = getattr(self, "_mosaic_overlay", None)
        if overlay is None:
            return
        entries = []
        for det in self._survey.detections():
            badge, _msg = self._fit_badge(det.diameter_um)
            entries.append({
                "det_id": det.det_id,
                "cx": det.center_px[0], "cy": det.center_px[1],
                "r": det.radius_px,
                # Peach = fails the needle-fit rule; the same visual language the
                # list and the run warnings use.
                "color": COLORS["peach"] if badge else COLORS["green"],
                "dashed": det.source == "user",
                "label": f"{det.det_id} Ø{det.diameter_um:.0f}",
            })
        overlay.set_circles(entries)
        sel = self._survey.selected_id()
        if sel:
            overlay.set_highlight(sel)

    def _on_survey_selection(self, det_id: str):
        overlay = getattr(self, "_mosaic_overlay", None)
        if overlay is None:
            return
        overlay.set_highlight(det_id)
        if det_id:
            overlay.center_on(det_id)

    def _on_mosaic_rim_fitted(self, cx_px: float, cy_px: float, r_px: float):
        """3+ rim points on the mosaic → size the selected spheroid, or add one."""
        sel = self._survey.selected_id()
        if sel:
            self._survey.apply_radius_px(sel, r_px, commit=True)
            self._survey.apply_center_px(sel, cx_px, cy_px)
            return
        self._survey.add_manual(cx_px, cy_px, r_px)

    def _on_mosaic_empty_click(self, cx_px: float, cy_px: float):
        """A click on bare mosaic adds a hand-placed spheroid at the default Ø.

        Detection can legitimately return nothing (a nuclear stain is puncta, not
        a disc), so this path has to exist for the workflow to stay usable.
        """
        ctx = self._mosaic_context_for_channel(None)
        if ctx is None or not ctx.get("mosaic_scale"):
            return
        from SupportClasses.SpheroidDetector import radius_px_for_diameter_um
        r_px = radius_px_for_diameter_um(
            float(self._diameter.value()), ctx["mosaic_scale"])
        det_id = self._survey.add_manual(cx_px, cy_px, r_px)
        if det_id:
            self._survey.select_detection(det_id)
            self._survey.set_status(
                f"Added {det_id} by hand at Ø{self._diameter.value():.0f} µm — "
                f"drag its handle or use rim points to size it.")

    # ── Survey → picks ────────────────────────────────────────────

    def _on_transfer_to_picks(self, entries: list):
        """Copy the curated spheroids into the pick list.

        Each is stamped ``PROV_MOSAIC``: its position came from a mosaic, so it
        renders dashed until a live click on that spheroid confirms it. The
        DIAMETER needs no such caveat — a length is translation-invariant, so the
        registration shift cannot corrupt it.
        """
        from gui.widgets.live_target_picker import PROV_MOSAIC
        added = 0
        for e in entries or []:
            self._picker.add_pick(
                float(e["x_um"]), float(e["y_um"]),
                size_um=float(e.get("diameter_um") or 0.0),
                provenance=PROV_MOSAIC)
            added += 1
        if not added:
            return
        self._survey.set_status(
            f"Transferred {added} spheroid(s) to the pick list. Their positions "
            f"came from the mosaic — go to each one and click it on the live "
            f"view to confirm before running.")
        self._status.setText(
            f"{added} spheroid(s) added as picks (mosaic positions — confirm on "
            f"the live view). Add a place target for each.")
        # Surface the pick list so the operator sees what landed there.
        self._tabs.setCurrentIndex(0)

    def _on_target_changed(self, _target_id: str):
        self._refresh_target_overlays()
        self._refresh_fit_summary()

    def _on_target_goto(self, target_id: str):
        """Row [Go to] on a pick/place list."""
        t = self._picker.target_by_id(target_id)
        if t is None:
            return
        self._travel_to_absolute(float(t.x_um), float(t.y_um))

    def _on_pick_added(self, target_id: str, provenance: str):
        """Bank a training crop for a spheroid the operator just selected.

        Gated tightly on purpose, so an ordinary un-measured click behaves exactly
        as it always has (no capture, no message):

        * only a LIVE-derived pick — a transferred mosaic position is not
          necessarily under the camera, and a crop of the wrong place labelled
          with this diameter is worse than no crop at all;
        * only one that carries a MEASURED diameter, which is the label;
        * only when the operator has crops enabled.

        The capture itself still self-refuses if the stage turns out not to be on
        the spheroid (see ``SpheroidTrainingStore.refuse_crop_reason``).
        """
        from gui.widgets.live_target_picker import PROV_MOSAIC
        if provenance == PROV_MOSAIC:
            return
        if not self._crop_enabled.isChecked():
            return
        t = self._picker.target_by_id(target_id)
        if t is None:
            return
        diameter = float(getattr(t, "size_um", 0.0) or 0.0)
        if diameter <= 0:
            return
        self._on_save_training_crop(
            float(t.x_um), float(t.y_um), diameter, target_id)

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)

        title = QLabel("Spheroid Pick & Place")
        title.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(14)}pt;"
            f"font-weight: 600;"
        )
        row.addWidget(title)

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
            "Open the full, saveable settings for this workflow "
            "(offsets, flow rates, pauses, prep, post-clean, locations).")
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
        self._refresh_sink_status()
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        """Called after a profile load / reset / import re-applies values."""
        self._refresh_volume_label()
        self._refresh_prep_status()
        self._refresh_sink_status()
        self._update_settings_summary()
        self._update_button_state()

    def _update_settings_summary(self):
        if not hasattr(self, "_settings_summary"):
            return
        try:
            prep = "prep on" if self._prep_check.isChecked() else "prep off"
            extras = []
            if self._sink_timing_enabled.isChecked():
                extras.append("sink-timed")
            if self._disengage_enabled.isChecked():
                extras.append("disengage")
            if self._per_target_volume_enabled():
                extras.append("per-Ø volume")
            extra = (" · " + "/".join(extras)) if extras else ""
            diam = ("Ø measured" if self._per_target_volume_enabled()
                    else f"Ø{self._diameter.value():.0f}µm")
            self._settings_summary.setText(
                f"{diam} · "
                f"pick {self._pick_flow.value():.2g}/place "
                f"{self._place_flow.value():.2g} µL/s · {prep}{extra}")
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
        # ── Spheroid & bore ──
        self._diameter = self._dspin(1.0, 5000.0, 200.0, " µm", 1, 10.0)
        self._bore = QComboBox()
        self._bore.setMinimumWidth(s(110))
        # v7.8: range widened from 1.0–5.0 at the operator's request. Values
        # below 1.0 aspirate LESS than the spheroid's own volume — allowed, but
        # flagged, since the carrier column then cannot fully contain it.
        self._safety = self._dspin(0.01, 10.0, 1.5, "", 2, 0.1)
        self._per_target_volume = QCheckBox(
            "Size each aspirate from that spheroid's measured Ø")
        self._per_target_volume.setChecked(True)
        self._per_target_volume.setToolTip(
            "On: a pick with a measured diameter aspirates the volume computed "
            "from ITS diameter (× the safety factor). Off: every pick uses the "
            "default diameter below.\n"
            "Volume scales as diameter cubed, so a 200 → 350 µm measurement is "
            "a 5.4× volume change.")
        sec = dlg.add_section("Spheroid & bore")
        sec.add("diameter", "Spheroid Ø (default / unmeasured)",
                self._diameter, 200.0,
                "Fallback diameter for a pick with no measured size — and the "
                "diameter used for every pick when per-spheroid sizing is off.")
        sec.add("bore", "Pump / bore", self._bore, "P1",
                "Which pump drives the aspirate + dispense.")
        # v7.9: states the bore-1-only restriction where the choice is made.
        self._multi_bore_note = QLabel("")
        self._multi_bore_note.setWordWrap(True)
        self._multi_bore_note.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        self._multi_bore_note.setVisible(False)
        sec.add_widget(self._multi_bore_note)
        sec.add("safety_factor", "Safety factor (×)", self._safety, 1.5,
                "Volume multiplier on the computed spheroid volume. Applied to "
                "the per-spheroid volume too.")
        sec.add_check("per_target_volume", self._per_target_volume, True)
        self._volume_label = QLabel("V = —")
        self._volume_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._volume_label)

        # ── Needle fit (advisory) ──
        # The operator's rule: needle ID at least 1.5× the spheroid Ø. This
        # feeds NeedleSpec.spheroid_pickup_detail's `clearance` kwarg, whose
        # ratio IS orifice_id_um / spheroid_um — so 1.5 here is exactly that
        # rule. Advisory only: a deformable spheroid can squeeze through a
        # slightly smaller orifice, so it warns and proceeds.
        self._clearance = self._dspin(
            1.0, 5.0, 1.5, "×", 2, 0.1,
            "Warn when the needle orifice ID is less than this multiple of a "
            "spheroid's diameter. Never blocks the run.")
        sec = dlg.add_section("Needle fit (advisory)")
        sec.add("needle_clearance", "Needle ID ≥ … × spheroid Ø",
                self._clearance, 1.5)
        self._fit_summary = QLabel("")
        self._fit_summary.setWordWrap(True)
        self._fit_summary.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._fit_summary)

        # ── Training crops ──
        self._crop_enabled = QCheckBox(
            "Save a training crop when a spheroid is picked")
        self._crop_enabled.setChecked(True)
        self._crop_enabled.setToolTip(
            "Bank a cropped microscope image of each selected spheroid as "
            "future detector training data. Images only — nothing is trained.")
        self._crop_pad = self._dspin(
            0.0, 2.0, 0.25, "×", 2, 0.05,
            "Context kept around the circle, as a fraction of its radius. A "
            "zero-context crop is poor training data; too much pulls in a "
            "neighbouring spheroid.")
        self._crop_settle = self._ispin(
            0, 5000, 300,
            "Wait this long after the stage settles before capturing.")
        self._crop_fresh = self._ispin(
            1, 20, 3,
            "Discard this many frames after the settle, so the capture is not "
            "a stale buffered frame.")
        sec = dlg.add_section("Training crops")
        sec.add_check("crop_enabled", self._crop_enabled, True)
        sec.add("crop_pad_frac", "Crop padding (× radius)", self._crop_pad, 0.25)
        sec.add("crop_settle_ms", "Settle before capture", self._crop_settle, 300)
        sec.add("crop_fresh_frames", "Fresh frames to wait", self._crop_fresh, 3)
        self._crop_status = QLabel("")
        self._crop_status.setWordWrap(True)
        self._crop_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._crop_status)

        # ── Heights ──
        self._pick_z = self._dspin(
            0.0, 20.0, 0.10, " mm", 2, 0.05,
            "Needle height above the calibrated plate bottom when aspirating.")
        self._place_z = self._dspin(
            0.0, 20.0, 0.50, " mm", 2, 0.05,
            "Needle height above the calibrated plate bottom when dispensing.")
        sec = dlg.add_section("Heights (above plate bottom)")
        sec.add("pick_z", "Pick Z (↑ bottom)", self._pick_z, 0.10)
        sec.add("place_z", "Place Z (↑ bottom)", self._place_z, 0.50)

        # ── Flow rates ──
        self._pick_flow = self._dspin(
            0.01, 50.0, 1.0, " µL/s", 2, 0.1,
            "Aspiration flow rate at the pick location.")
        self._place_flow = self._dspin(
            0.01, 50.0, 1.0, " µL/s", 2, 0.1,
            "Dispense flow rate at the place location.")
        sec = dlg.add_section("Flow rates")
        sec.add("pick_flow", "Pick (aspirate) flow", self._pick_flow, 1.0)
        sec.add("place_flow", "Place (dispense) flow", self._place_flow, 1.0)

        # ── Pauses ──
        self._pick_dwell = self._dspin(
            0.0, 3600.0, 0.0, " s", 1, 0.5,
            "Hold after aspirating, so the spheroid settles into the bore.")
        self._place_dwell = self._dspin(
            0.0, 3600.0, 0.0, " s", 1, 0.5,
            "Hold after dispensing, so the spheroid releases from the bore.")
        sec = dlg.add_section("Pauses / dwell")
        sec.add("pick_dwell", "Pause after pick", self._pick_dwell, 0.0)
        sec.add("place_dwell", "Pause after place", self._place_dwell, 0.0)

        # ── Sink timing & disengage (optional) ──
        self._disengage_enabled = QCheckBox(
            "Extra disengage aspirate (pop the spheroid off the glass)")
        self._disengage_enabled.setToolTip(
            "A spheroid stuck to the plate sometimes needs extra suction. When "
            "on, an extra aspirate is applied as the fast leading portion of the "
            "pickup so it releases before rising up the bore.")
        self._disengage_vol = self._dspin(
            0.0, 200.0, 0.0, " µL", 3, 0.1,
            "Extra aspirate volume used to disengage a stuck spheroid.")
        self._disengage_rate = self._dspin(
            0.01, 50.0, 2.0, " µL/s", 2, 0.5,
            "Flow rate of the disengage aspirate (usually faster than the pick).")
        self._sink_timing_enabled = QCheckBox(
            "Size aspirate from sink timing (per move)")
        self._sink_timing_enabled.setToolTip(
            "Use the calibrated sink curve to choose the aspirate volume per "
            "move so the spheroid finishes sinking to the tip right as the "
            "needle arrives — it can't sink out in the well and isn't lifted "
            "more than needed. Requires a calibrated curve + the needle inner Ø.")
        self._travel_margin = self._dspin(
            0.0, 60.0, 1.0, " s", 1, 0.5,
            "Arrive with the spheroid slightly under-sunk (waited out at the "
            "destination) — a safety buffer against travel-time estimate error.")
        self._release_enabled = QCheckBox(
            "Minimal-excess release dispense (needle keeps the rest)")
        self._release_enabled.setToolTip(
            "After the spheroid sinks to the tip, dispense only this small "
            "volume so minimal excess is deposited. The needle retains the rest "
            "— it accumulates across picks, so leave Post-clean on to clear it. "
            "Off = dispense the full aspirated volume (volume-balanced).")
        self._release_vol = self._dspin(
            0.0, 200.0, 0.0, " µL", 3, 0.1,
            "Small dispense volume at placement (minimal excess).")
        sec = dlg.add_section("Sink timing & disengage (optional)")
        sec.add_check("disengage_enabled", self._disengage_enabled, False)
        sec.add("disengage_vol", "Disengage volume", self._disengage_vol, 0.0)
        sec.add("disengage_rate", "Disengage flow", self._disengage_rate, 2.0)
        sec.add_check("sink_timing", self._sink_timing_enabled, False)
        sec.add("travel_margin", "Travel margin", self._travel_margin, 1.0)
        sec.add_check("release_enabled", self._release_enabled, False)
        sec.add("release_vol", "Release volume", self._release_vol, 0.0)
        self._sink_status = QLabel("Sink curve: not calibrated")
        self._sink_status.setWordWrap(True)
        self._sink_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._sink_status)
        self._calibrate_btn = QPushButton("Calibrate sink timing…")
        self._calibrate_btn.setToolTip(
            "Guided staircase: aspirate increasing volumes on one test spheroid "
            "and click each time it reappears at the tip → builds the sink "
            "timing curve. Also a disengage-test panel.")
        self._calibrate_btn.clicked.connect(self._open_sink_calibration)
        sec.add_widget(self._calibrate_btn)

        # ── Needle prep ──
        self._prep_check = QCheckBox("Prep needle (waste → oil → wash → buffer)")
        self._prep_check.setChecked(True)
        self._prep_check.setToolTip(
            "Before picking: dispense 1 needle of oil to waste, aspirate 1 needle "
            "of fresh oil, wash, then aspirate buffer. Service-well locations are "
            "inherited from Hardware Setup → Ink (Reagent Locations).")
        self._prep_check.toggled.connect(self._on_prep_toggled)
        self._service_z = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom at the service wells.")
        self._prep_rate = self._dspin(
            0.01, 50.0, 1.0, " µL/s", 2, 0.1, "Aspirate/dispense flow during prep.")
        self._oil_needles = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5,
            "Needles of oil dispensed to waste AND aspirated from the oil well.")
        self._buffer_needles = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5,
            "Needles of buffer aspirated after the wash.")
        self._wash_cycles = self._ispin(
            0, 50, 3, "Dip-jiggle (Z + random XY) cycles at the wash well.")
        self._wash_z_amp = self._dspin(
            0.0, 10.0, 0.5, " mm", 2, 0.1, "How far up/down each wash jiggle moves.")
        self._wash_xy_amp = self._dspin(
            0.0, 5000.0, 200.0, " µm", 0, 10.0,
            "Random XY radius about the well centre during the wash.")
        self._wash_dwell = self._dspin(
            0.0, 30.0, 0.3, " s", 2, 0.1, "Settle time between wash jiggles.")
        sec = dlg.add_section("Needle prep")
        sec.add_check("prep", self._prep_check, True)
        sec.add_note(
            "Prep values are shared defaults from Common Print Settings — tick "
            "Override to set a workflow-specific value.")
        sec.add_common("service_z", "Service dip Z (↑ bottom)", self._service_z, 0.50)
        sec.add_common("prep_rate", "Prep flow", self._prep_rate, 1.0)
        sec.add_common("oil_needles", "Oil (needles)", self._oil_needles, 1.0)
        sec.add_common("buffer_needles", "Buffer (needles)", self._buffer_needles, 1.0)
        sec.add_common("wash_cycles", "Wash cycles", self._wash_cycles, 3)
        sec.add_common("wash_z_amp", "Wash Z jiggle", self._wash_z_amp, 0.5)
        sec.add_common("wash_xy_amp", "Wash XY jiggle", self._wash_xy_amp, 200.0)
        sec.add_common("wash_dwell", "Wash settle", self._wash_dwell, 0.3)
        self._prep_status = QLabel("")
        self._prep_status.setWordWrap(True)
        self._prep_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        sec.add_widget(self._prep_status)

        # ── Post-clean ──
        self._post_clean_check = QCheckBox(
            "Clean after the run (dispense → wash → buffer)")
        self._post_clean_check.setToolTip(
            "After the last place: dispense residual to waste, wash, then reload "
            "buffer so the needle ends conditioned.")
        self._post_clean_check.toggled.connect(self._on_prep_toggled)
        self._post_dispense = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5, "Needles of residual dispensed to waste.")
        sec = dlg.add_section("Post-clean")
        sec.add_check("post_clean", self._post_clean_check, False)
        sec.add("post_dispense", "Dispense (needles)", self._post_dispense, 1.0)

        # ── Motion & timeouts ──
        self._intra_retract = self._dspin(
            0.0, 20.0, 1.0, " mm", 2, 0.1,
            "Short Z retract used for moves that stay within one well.")
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
        # The detection band + the two live-view mode flags live on widgets that
        # belong to the picker and the survey panel (built after this dialog), so
        # they round-trip through the sanctioned extra-state hook rather than
        # being duplicated here. DELIBERATELY excluded: the detected spheroid list
        # itself — a restored coordinate from another plate would be a crash, and
        # nothing here can prove the mosaic it came from is still the one loaded.
        dlg.set_extra_state(self._collect_extra_state,
                            self._apply_extra_state)
        dlg.finalize()

        # Signals (connected after all widgets exist so handlers find their labels)
        self._diameter.valueChanged.connect(self._refresh_volume_label)
        self._safety.valueChanged.connect(self._refresh_volume_label)
        self._diameter.valueChanged.connect(self._update_settings_summary)
        self._pick_flow.valueChanged.connect(self._update_settings_summary)
        self._place_flow.valueChanged.connect(self._update_settings_summary)
        self._sink_timing_enabled.toggled.connect(self._update_settings_summary)
        self._disengage_enabled.toggled.connect(self._update_settings_summary)
        self._per_target_volume.toggled.connect(self._update_settings_summary)
        self._clearance.valueChanged.connect(self._refresh_fit_summary)
        self._refresh_volume_label()
        self._refresh_sink_status()
        self._refresh_fit_summary()

    def _build_locations_panel(self):
        return build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z)

    # ── Non-widget settings state ─────────────────────────────────

    def _collect_extra_state(self) -> dict:
        """Detection parameters + the live-view mode flags, for the profile."""
        state: dict = {}
        survey = self._survey
        if survey is not None:
            state["det_min_diameter"] = float(survey._min_d.value())
            state["det_max_diameter"] = float(survey._max_d.value())
            state["det_restrict_to_well"] = bool(
                survey._restrict_well.isChecked())
        picker = getattr(self, "_picker", None)
        if picker is not None:
            state["measure_mode"] = bool(picker.measure_mode())
        return state

    def _apply_extra_state(self, state) -> None:
        if not isinstance(state, dict):
            return
        survey = self._survey
        if survey is not None:
            if "det_min_diameter" in state:
                survey._min_d.setValue(float(state["det_min_diameter"]))
            if "det_max_diameter" in state:
                survey._max_d.setValue(float(state["det_max_diameter"]))
            if "det_restrict_to_well" in state:
                survey._restrict_well.setChecked(
                    bool(state["det_restrict_to_well"]))
        picker = getattr(self, "_picker", None)
        if picker is not None and "measure_mode" in state:
            picker.set_measure_mode(bool(state["measure_mode"]))

    # Reagent roles the prep inherits from Hardware Setup → Ink (Reagent
    # Locations) by the assigned ink's ink_type.
    _SERVICE_ROLES = ("waste", "oil", "wash", "buffer")

    def _on_prep_toggled(self, *_):
        # Wash / buffer / service knobs are shared by BOTH prep and post-clean
        # (the clean step also washes + reloads buffer), so enable them when
        # either is on. Oil is prep-only; the clean dispense is clean-only.
        prep = self._prep_check.isChecked()
        clean = self._post_clean_check.isChecked()
        shared = prep or clean
        for w in (self._service_z, self._prep_rate, self._buffer_needles,
                  self._wash_cycles, self._wash_z_amp, self._wash_xy_amp,
                  self._wash_dwell):
            w.setEnabled(shared)
        self._oil_needles.setEnabled(prep)
        self._post_dispense.setEnabled(clean)
        self._refresh_prep_status()
        self._update_settings_summary()

    # ── Service-well + needle-volume resolution (inherit from HW setup) ──

    def _needle_volume_uL(self) -> float:
        """One needle's internal bore volume (µL) from the configured needle."""
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        if needle is None:
            return 0.0
        try:
            return float(getattr(needle, "internal_volume_uL", 0.0) or 0.0)
        except Exception:
            return 0.0

    def _bore_area_mm2(self) -> float:
        """Needle NEAR-TIP cross-section (mm²) — used for volume↔lift-height in
        the sink-timing model. 0.0 if no needle is configured.

        v7.6: on a pulled capillary this is the tip area, because the first
        millimetre of lift happens entirely inside the tip — that is what makes
        the calibration staircase read correctly.
        """
        profile = self._bore_profile()
        return profile.near_tip_area_mm2 if profile else 0.0

    def _bore_profile(self):
        """Full two-stage bore geometry for volume↔lift, or None with no needle."""
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        if needle is None:
            return None
        try:
            from SupportClasses.PhysicalModels import needle_bore_profile
            profile = needle_bore_profile(needle)
            return profile if profile.is_usable() else None
        except Exception:
            return None

    def _needle_clearance(self) -> float:
        """The operator's "needle ID ≥ N × spheroid Ø" factor (default 1.5)."""
        spin = getattr(self, "_clearance", None)
        try:
            val = float(spin.value())
        except (AttributeError, TypeError, ValueError):
            return 1.5
        return val if val > 0 else 1.5

    def _spheroid_fit_detail(self, cfg) -> dict | None:
        """Advisory check that the spheroid clears the needle orifice.

        Returns the ``{status, ratio, message, severity}`` dict, or None when
        the needle can't answer (no geometry, or an older NeedleSpec).

        The ``clearance`` kwarg carries the operator's rule: the ratio computed
        inside is ``orifice_id_um / spheroid_um``, so a clearance of 1.5 IS
        "needle ID at least 1.5× the spheroid diameter".
        """
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        fn = getattr(needle, "spheroid_pickup_detail", None)
        if not callable(fn):
            return None
        d_um = float(getattr(cfg, "spheroid_diameter_um", 0.0) or 0.0)
        try:
            vol = cfg.compute_volume_uL()
        except Exception:
            return None
        try:
            return fn(d_um, volume_uL=vol, clearance=self._needle_clearance())
        except TypeError:
            # An older NeedleSpec, or a duck-typed stub without the kwarg.
            try:
                return fn(d_um, volume_uL=vol)
            except Exception:
                return None
        except Exception:
            return None

    def _fit_badge(self, diameter_um: float) -> tuple[str, str]:
        """``(badge, message)`` for one diameter — ``("", "")`` when it is fine.

        Used to annotate a single list row, so the operator sees which spheroid
        is the problem rather than a run-level warning naming none of them.
        """
        if not diameter_um or diameter_um <= 0:
            return ("", "")
        cfg = dataclasses.replace(self._current_config(),
                                  spheroid_diameter_um=float(diameter_um))
        detail = self._spheroid_fit_detail(cfg)
        if not detail or detail.get("severity") != "warning":
            return ("", "")
        status = str(detail.get("status", ""))
        ratio = detail.get("ratio")
        badge = {"too_large": "⚠ too large",
                 "tight": "⚠ tight",
                 "past_tip": "⚠ past tip"}.get(status, "⚠")
        if isinstance(ratio, (int, float)) and ratio:
            badge = f"{badge} {float(ratio):.2f}×"
        return (badge, str(detail.get("message", "")))

    def _warn_spheroid_fits(self, cfgs) -> None:
        """Log + surface non-blocking warnings for EVERY queued spheroid.

        With per-spheroid sizing the run no longer has one diameter, so a single
        check against one config would silently ignore the rest. Never prevents
        the run (operator decision, per the advisory-only contract every needle
        feasibility check in this app follows).
        """
        worst = ""
        n_warn = 0
        for cfg in cfgs:
            detail = self._spheroid_fit_detail(cfg)
            if not detail or detail.get("severity") != "warning":
                continue
            n_warn += 1
            msg = str(detail.get("message", ""))
            logger.warning("Spheroid pickup fit: %s", msg)
            if not worst:
                worst = msg
        if not worst or not hasattr(self, "_status"):
            return
        total = len(list(cfgs))
        suffix = (f"  ({n_warn} of {total} picks)" if total > 1 else "")
        self._status.setText(f"⚠ {worst}{suffix}")

    def _warn_spheroid_fit(self, cfg) -> None:
        """Single-config shim over :meth:`_warn_spheroid_fits`."""
        self._warn_spheroid_fits([cfg])

    def _refresh_fit_summary(self) -> None:
        """Aggregate needle-fit line in the settings popout.

        States the actual limit in µm — "orifice 300 µm → max 200 µm spheroid" —
        because a bare ratio does not tell the operator which spheroids to drop.
        """
        if not hasattr(self, "_fit_summary"):
            return
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        orifice = 0.0
        try:
            from SupportClasses.PhysicalModels import needle_orifice_id_um
            orifice = float(needle_orifice_id_um(needle)) if needle else 0.0
        except Exception:
            orifice = 0.0
        if orifice <= 0:
            self._fit_summary.setText("Needle bore unknown — no fit check.")
            return
        clearance = self._needle_clearance()
        max_um = orifice / clearance
        picks = self._picker.picks() if hasattr(self, "_picker") else []
        over = [t for t in picks
                if float(getattr(t, "size_um", 0.0) or 0.0) > max_um]
        text = (f"Orifice {orifice:.0f} µm at {clearance:.2f}× → spheroids up "
                f"to {max_um:.0f} µm.")
        if over:
            text += (f"  ⚠ {len(over)} of {len(picks)} pick(s) exceed it: "
                     + ", ".join(t.target_id for t in over[:6])
                     + ("…" if len(over) > 6 else ""))
        self._fit_summary.setText(text)
        self._fit_summary.setStyleSheet(
            f"color: {COLORS['peach'] if over else COLORS['subtext0']}; "
            f"font-size: {sf(9)}pt;")

    def _refresh_sink_status(self):
        """Update the sink-curve status label in the settings popout."""
        if not hasattr(self, "_sink_status"):
            return
        try:
            from SupportClasses.SpheroidSinkCalibrationStore import get_store
            store = get_store()
            curve = store.get_curve()
            if curve is None:
                self._sink_status.setText("Sink curve: not calibrated")
                return
            meta = store.get_meta() or {}
            when = str(meta.get("updated", "")).replace("T", " ")
            self._sink_status.setText(
                f"Sink curve: {curve.n_points} pt, "
                f"~{curve.effective_rate_mm_s():.3f} mm/s"
                + (f" · {when}" if when else ""))
        except Exception:
            self._sink_status.setText("Sink curve: not calibrated")

    def _open_sink_calibration(self):
        """Open the guided sink-timing / disengage calibration dialog (lazy)."""
        if getattr(self, "_sink_calib_dialog", None) is None:
            try:
                from gui.pages.workflows.spheroid_sink_calibration import (
                    SinkDisengageCalibrationDialog)
            except Exception as exc:
                logger.exception("Sink calibration dialog import failed: %s", exc)
                QMessageBox.warning(
                    self, "Calibration unavailable",
                    f"Could not open the sink calibration dialog:\n{exc}")
                return
            self._sink_calib_dialog = SinkDisengageCalibrationDialog(
                self, parent=self)
        self._sink_calib_dialog.show()
        self._sink_calib_dialog.raise_()
        self._sink_calib_dialog.activateWindow()

    def _service_well_names(self) -> dict[str, str]:
        """role → well name, read from Hardware Setup reagent locations. A well
        is matched to a role by the assigned ink's ``ink_type`` (or, as a
        fallback, an ink literally named waste/oil/wash/buffer)."""
        out: dict[str, str] = {}
        hw = self._hw_config
        if hw is None:
            return out
        ink_locations = getattr(hw, "ink_locations", {}) or {}
        ink_library = getattr(hw, "ink_library", {}) or {}
        for ink_name, wells in ink_locations.items():
            if not wells:
                continue
            spec = ink_library.get(ink_name)
            itype = ((getattr(spec, "ink_type", "") or "").strip().lower()
                     if spec is not None else "")
            name_l = (ink_name or "").strip().lower()
            role = (itype if itype in self._SERVICE_ROLES
                    else name_l if name_l in self._SERVICE_ROLES else None)
            if role and role not in out:
                # Prefer a real sub-well over a flattened rosette parent.
                out[role] = resolve_pickup_well(wells, self._plate)
        return out

    def _resolve_service_positions(self):
        """Return (positions, missing): role → absolute stage µm for service
        wells resolvable in the calibrated well map, and the roles that aren't."""
        names = self._service_well_names()
        wells = self._well_positions or {}
        positions: dict[str, tuple[float, float]] = {}
        for role in self._SERVICE_ROLES:
            wn = names.get(role)
            if wn and wn in wells:
                positions[role] = wells[wn]
        missing = [r for r in self._SERVICE_ROLES if r not in positions]
        return positions, missing

    def _refresh_prep_status(self):
        if not hasattr(self, "_prep_status"):
            return
        if not self._prep_check.isChecked():
            self._prep_status.setText("Prep disabled — pick & place only.")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            return
        needle_uL = self._needle_volume_uL()
        positions, missing = self._resolve_service_positions()
        names = self._service_well_names()
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
        # The service dip Z (like pick/place) needs the plate bottom calibrated;
        # match the start gate so the status never reads ✓ when Start would block.
        if self._plate_offset_to_zref(float(self._service_z.value())) is None:
            self._prep_status.setText(
                "⚠ Plate bottom Z not calibrated — needed for the service dip Z.")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        mapping = "  ".join(f"{r}={names.get(r, '?')}" for r in self._SERVICE_ROLES)
        self._prep_status.setText(
            f"✓ 1 needle = {needle_uL:.3f} µL  ·  {mapping}")
        self._prep_status.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {sf(9)}pt;")

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))

        self._start_btn = QPushButton("Start spheroid pickup")
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
        return "Spheroid Pick & Place"

    def get_sub_page_title(self) -> str:
        return "Spheroid Pick & Place"

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
        # Tuck the settings popout away (and persist last values) when the
        # operator leaves the workflow.
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        try:
            if (self._sink_calib_dialog is not None
                    and self._sink_calib_dialog.isVisible()):
                self._sink_calib_dialog.hide()
        except Exception:
            pass
        # The embedded scan page owns its own modeless dialog (and a live camera)
        # that must stop when this page goes away.
        #
        # v7.9: do NOT call self._scan_page.hide() here. It is a CHILD widget, so
        # Qt already delivers it a hide event when this page hides — which is
        # what stops the camera. An EXPLICIT hide() additionally sets the
        # widget's own hidden flag, which STICKS: Qt then never re-shows it with
        # the parent, so the survey tab came back permanently BLANK on the second
        # visit. Verified: with the explicit call the child reports
        # isHidden()==True after a re-show; without it the child still receives
        # exactly one hide event and is auto re-shown.
        super().hideEvent(event)

    # ── Common Print Settings hook ────────────────────────────────

    def set_common_print_settings(self, common):
        """v7.5.x: shared common settings — the dialog's inheriting prep fields
        re-sync to these defaults; the global pump fields mirror/edit them."""
        if getattr(self, "_settings_dialog", None) is not None:
            self._settings_dialog.set_common(common)
        if self._scan_page is not None and hasattr(
                self._scan_page, "set_common_print_settings"):
            try:
                self._scan_page.set_common_print_settings(common)
            except Exception as exc:
                logger.debug("scan page common settings failed: %s", exc)

    # ── hw_config hook ────────────────────────────────────────────

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        # Refresh bore options from pump ids
        self._bore.blockSignals(True)
        previous = self._bore.currentText()
        self._bore.clear()
        # v7.9: on a multi-bore assembly, offer ONLY the pump feeding bore 1.
        # This workflow positions bore 1 and resolves its pickup volume, bore
        # area AND sink-timing bore profile from bore 1, so another pump would
        # mis-size every aspirate by the bore-area ratio and land 100-500 µm off
        # the spheroid. None ⇒ single bore or unknown ⇒ no restriction.
        datum = _reagent_prep.datum_bore_pump(hw_config)
        if hw_config is not None and hasattr(hw_config, "pumps"):
            for pid, pcfg in hw_config.pumps.items():
                # Only show enabled + configured pumps when those flags exist
                enabled = getattr(pcfg, "enabled", True)
                configured = getattr(pcfg, "is_configured", True)
                if enabled and configured and (datum is None or pid == datum):
                    self._bore.addItem(pid)
        if self._bore.count() == 0:
            self._bore.addItem(datum or "P1")
        # Restore previous selection if still present
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
            # NeedleSpec exposes od_um / length_mm (the old "outer_diameter_mm"
            # name never existed, so the needle outline silently never drew).
            # v7.6: prefer the ORIFICE OD (the pulled tip approaches the plate)
            # and the barrel+tip length.
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
        # The bore combo just (re)populated — apply any saved bore selection
        # that was pending because the combo was empty at load time.
        try:
            self._settings_dialog.resolve_pending()
        except Exception:
            pass
        self._forward_to_scan_page("set_hardware_config", hw_config)
        self._refresh_prep_status()
        self._update_button_state()
        self._update_settings_summary()
        self._refresh_fit_summary()
        if self._survey is not None:
            self._survey.refresh_badges()

    def _forward_to_scan_page(self, method: str, *args) -> None:
        """Relay a host push to the embedded scan page, best-effort."""
        page = self._scan_page
        fn = getattr(page, method, None) if page is not None else None
        if not callable(fn):
            return
        try:
            fn(*args)
        except Exception as exc:
            logger.debug("scan page %s failed: %s", method, exc)

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
        # Re-overlay targets so they sit on top of the refreshed plate.
        self._refresh_target_overlays()
        # Re-apply the fluorescence overlay if the operator has it on.
        if getattr(self, "_fluor_check", None) is not None and self._fluor_check.isChecked():
            load_plate_fluor_overlay(
                self._workspace_view, plate_key_of(self._hw_config), visible=True)
        # Service-well resolution depends on the calibrated well positions.
        self._refresh_prep_status()
        # The survey tab's scan + well geometry come from the same calibration.
        self._forward_to_scan_page(
            "set_calibration_data", plate, well_positions, safe_z)

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
        self._forward_to_scan_page("set_z_references", self._z_references)


    def set_visible_z_references(self, keys) -> None:
        """v7.9.1: which Z references get a quick-move badge (see the Jog page).

        Presentation only — the reference VALUES are untouched.
        """
        try:
            self._xz_view.set_visible_z_references(keys)
        except Exception:
            pass

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
        """Convert pick + place targets from stage-frame µm to zero-ref µm
        and push them into the workspace view's overlay layer."""
        try:
            zero = self._controller.zero_position
        except Exception:
            zero = {"x": 0.0, "y": 0.0}

        # 4-tuples: the workspace draws a measured spheroid at its true relative
        # size (the 4th element is optional, so 3-tuple callers still work).
        picks_zr = [
            (t.x_um - zero["x"], t.y_um - zero["y"], t.target_id,
             float(getattr(t, "size_um", 0.0) or 0.0))
            for t in self._picker.picks()
        ]
        places_zr = [
            (t.x_um - zero["x"], t.y_um - zero["y"], t.target_id,
             float(getattr(t, "size_um", 0.0) or 0.0))
            for t in self._picker.places()
        ]
        self._workspace_view.set_pick_targets(picks_zr)
        self._workspace_view.set_place_targets(places_zr)
        self._refresh_fit_summary()

    # ── Stage position indicator refresh ─────────────────────────

    def _refresh_position_indicators(self):
        try:
            xy = self._controller.get_xy_position(cached=True)
            zero = self._controller.zero_position
        except Exception:
            return
        # v7.5.x: absolute envelope → push zero so the view draws it zero-ref.
        if hasattr(self._workspace_view, "set_zero_offset"):
            self._workspace_view.set_zero_offset(zero["x"], zero["y"])
        if hasattr(self._xz_view, "set_zero_offset_x"):
            self._xz_view.set_zero_offset_x(zero["x"])
        if hasattr(self._xz_view, "set_zero_offset_z"):
            self._xz_view.set_zero_offset_z(zero.get("Z", 0.0))
        if xy is not None and xy[0] is not None and xy[1] is not None:
            # get_xy_position is absolute stage µm; zero is also µm — subtract
            # to get the zero-ref µm the views draw in (no × 1000; that
            # inflated the indicator ~1000× and placed it off-canvas).
            zx_um = float(xy[0]) - zero["x"]
            zy_um = float(xy[1]) - zero["y"]
            self._workspace_view.set_position(zx_um, zy_um)

        # Push current Z (zero-ref mm) into XZ view.
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

    def _stage_busy(self) -> bool:
        """True (+ shows a hint) when something else is already driving the stage.

        Consulted by EVERY entry point that can move the stage — Start, the two
        workspace clicks, a row's Go to, and a survey Go to. Two drivers on one
        serial channel is bad enough, but the mosaic scan worker also toggles
        ``suspend_position_poller``, which is NOT refcounted: whichever finishes
        first re-enables the poller underneath the other.
        """
        t = getattr(self, "_exec_thread", None)
        if t is not None and t.is_alive():
            self._status.setText("Busy running — abort first to move manually.")
            return True
        page = getattr(self, "_scan_page", None)
        if page is not None and page.is_scanning():
            self._status.setText(
                "A mosaic scan is running — wait for it or abort it first.")
            return True
        worker = getattr(self, "_crop_worker", None)
        if worker is not None and worker.isRunning():
            return True
        return False

    # Kept as an alias: the old name reads better at the two workspace-click
    # sites and is what the existing tests reference.
    def _travel_blocked_by_run(self) -> bool:
        return self._stage_busy()

    def _travel_to_absolute(self, x_um_abs: float, y_um_abs: float) -> bool:
        """Retract to safe Z, then travel to an ABSOLUTE stage µm point.

        Deliberately separate from ``_on_workspace_position_clicked``, which
        receives ZERO-REF µm and adds ``controller.zero_position`` — sharing one
        handler between the two frames would add the zero twice and put the move
        millimetres away. Note the parameter names.

        Goes through ``SafeTravelWorker`` with ``target_z_mm=None``, so the needle
        retracts and WAITS before any XY motion and never descends on arrival.
        """
        if not getattr(self._controller, "is_xy_connected", False):
            self._status.setText("XY stage not connected.")
            return False
        if self._stage_busy():
            return False
        if not getattr(self._controller, "is_zp_connected", False):
            # Without the ZP board safe_travel_to silently skips its retract, so
            # the stage would drive XY with the needle possibly down.
            self._status.setText(
                "ZP (Z + pump) board not connected — it is what retracts the "
                "needle before travel. Reconnect it first.")
            return False
        if self._safe_z is None:
            self._status.setText(
                "No safe Z configured — set it on the Calibration page before "
                "travelling.")
            return False
        self._travel_worker.start(
            self._controller, x_um_abs, y_um_abs,
            safe_z_mm=self._safe_z, target_z_mm=None)
        return True

    def _on_spheroid_goto(self, x_um_abs: float, y_um_abs: float):
        """Survey [Go to] — travel to a detected spheroid (absolute stage µm)."""
        if self._travel_to_absolute(x_um_abs, y_um_abs):
            self._survey.set_status(
                "Travelling… then click the spheroid on the live view to "
                "confirm its position.")

    # ── Training crops ────────────────────────────────────────────

    def _on_save_training_crop(self, x_um: float, y_um: float,
                               diameter_um: float, det_id: str):
        """Bank a cropped image of one spheroid as future training data.

        The capture runs on a worker thread (settling + waiting for fresh frames
        would otherwise stall the event loop and freeze every camera feed), and
        the crop is taken from the RAW frame — never the display pixmap, which
        carries our own crosshair and target rings.
        """
        from gui.widgets.spheroid_crop_worker import SpheroidCropWorker
        worker = getattr(self, "_crop_worker", None)
        if worker is not None and worker.isRunning():
            return
        cam = self._microscope_widget()
        if cam is None:
            self._crop_message("No microscope camera to capture from.")
            return
        eff = self._live_um_per_px()
        if not eff:
            self._crop_message(
                "The camera has no µm/px calibration, so a crop could not be "
                "scaled. Calibrate the objective first.")
            return
        self._pending_crop = {
            "x_um": float(x_um), "y_um": float(y_um),
            "diameter_um": float(diameter_um), "det_id": str(det_id),
        }
        self._crop_worker = SpheroidCropWorker(
            cam, self._read_stage_xy_um, eff,
            settle_ms=int(self._crop_settle.value()),
            fresh_frames=int(self._crop_fresh.value()))
        self._crop_worker.captured.connect(self._on_crop_captured)
        self._crop_worker.failed.connect(
            lambda msg: self._crop_message(f"Crop failed: {msg}"))
        self._crop_message("Capturing a fresh frame…")
        self._crop_worker.start()

    def _crop_message(self, text: str) -> None:
        """Report a crop outcome on both surfaces that could be in view."""
        if self._survey is not None:
            self._survey.set_status(text)
        if hasattr(self, "_crop_status"):
            self._crop_status.setText(text)

    def _on_crop_captured(self, frame, stage_um, um_per_px: float):
        from SupportClasses import SpheroidTrainingStore as sts
        pending = getattr(self, "_pending_crop", None) or {}
        target = (pending.get("x_um", 0.0), pending.get("y_um", 0.0))
        diameter = float(pending.get("diameter_um") or 0.0)
        try:
            h, w = frame.shape[:2]
        except Exception:
            self._crop_message("Crop failed: unreadable frame.")
            return
        radius_px = (diameter / 2.0) / um_per_px if um_per_px else 0.0
        refusal = sts.refuse_crop_reason(
            target, stage_um, um_per_px, (w, h), radius_px,
            pad_frac=float(self._crop_pad.value()))
        if refusal:
            self._crop_message(f"Not saved — {refusal}")
            return
        centre = sts.target_center_px(target, stage_um, um_per_px, (w, h))
        rect = sts.crop_rect_for_circle(centre, radius_px, (w, h),
                                        pad_frac=float(self._crop_pad.value()))
        crop = sts.crop_from_frame(frame, rect)
        if crop is None:
            self._crop_message("Not saved — the crop window was empty.")
            return
        det = self._survey.detection(pending.get("det_id", ""))
        source = sts.SOURCE_AUTO
        if det is not None:
            source = (sts.SOURCE_MANUAL if det.source == "user"
                      else (sts.SOURCE_REDRAWN if det.user_edited
                            else sts.SOURCE_AUTO))
        ctx = self._mosaic_context_for_channel(None) or {}
        rel = sts.get_store().add_crop(
            crop, diameter_um=diameter, radius_px=radius_px,
            um_per_px=um_per_px,
            center_px_in_crop=rect.center_px_in_crop,
            crop_origin_px=(rect.x0, rect.y0), frame_wh=(w, h),
            stage_um=stage_um, well=str(ctx.get("well") or ""),
            plate_key=str(ctx.get("plate_key") or ""),
            objective=str(ctx.get("objective") or ""),
            channel=str(ctx.get("channel") or ""),
            detection_source=source,
            user_edited=bool(det.user_edited) if det is not None else False,
            clipped=rect.clipped, pad_frac=float(self._crop_pad.value()),
            target_id=str(pending.get("det_id") or ""))
        if not rel:
            self._crop_message("Crop failed to write — see the log.")
            return
        store = sts.get_store()
        self._crop_message(
            f"Saved training crop {rel} (Ø{diameter:.0f} µm) — "
            f"{store.count()} sample(s), {store.total_bytes() / 1e6:.1f} MB.")

    def _microscope_widget(self):
        """The live microscope CameraWidget, or None."""
        mgr = self._camera_manager
        if mgr is None:
            return None
        try:
            cams = mgr.cameras
        except Exception:
            return None
        idx = 0
        if self._hw_config is not None:
            try:
                from SupportClasses.HardwareConfig import CameraRole
                resolved = self._hw_config.camera_for_role(CameraRole.MICROSCOPE)
                if resolved is not None:
                    idx = int(resolved)
            except Exception:
                idx = 0
        return cams[idx] if 0 <= idx < len(cams) else None

    def _live_um_per_px(self) -> float:
        """Effective µm/px for the CURRENT live frame width, or 0.0."""
        mgr = self._camera_manager
        cam = self._microscope_widget()
        if mgr is None or cam is None:
            return 0.0
        width = 0
        try:
            frame = cam.get_current_frame()
            if frame is not None:
                width = int(frame.shape[1])
        except Exception:
            width = 0
        try:
            eff = getattr(mgr, "effective_um_per_px", None)
            idx = getattr(cam, "cam_idx", 0)
            if callable(eff) and width:
                return float(eff(self._microscope_index(), width))
            return float(mgr.get_um_per_px(self._microscope_index()))
        except Exception:
            return 0.0

    def _microscope_index(self) -> int:
        if self._hw_config is None:
            return 0
        try:
            from SupportClasses.HardwareConfig import CameraRole
            idx = self._hw_config.camera_for_role(CameraRole.MICROSCOPE)
            return int(idx) if idx is not None else 0
        except Exception:
            return 0

    def _read_stage_xy_um(self):
        """Absolute stage XY in µm (for the crop worker's snapshot)."""
        try:
            pos = self._controller.get_xy_position(cached=False)
        except Exception:
            return None
        if pos is None or pos[0] is None or pos[1] is None:
            return None
        return (float(pos[0]), float(pos[1]))

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

        zp = self._controller.get_zp_position(cached=True)
        current_z = None
        if zp is not None and zp[0] is not None:
            try:
                z_raw = self._controller.zp_logical_value(zp, "Z")
                if z_raw is not None:
                    current_z = z_raw - zero["Z"]
            except Exception:
                current_z = None

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
            # v7.5.x bugfix: move_xy_absolute(from_zero_ref=True) expects mm;
            # the workspace emits zero-ref µm → convert (was 1000× overshoot).
            self._controller.move_xy_absolute(
                x_um_zr / 1000.0, y_um_zr / 1000.0, from_zero_ref=True)
            return

        # v7.5.x CRITICAL FIX: cross-position click-to-travel ALWAYS retracts
        # via safe_travel_to. The old "current_z >= safe_z → skip retract" gate
        # was polarity-wrong on ME3B V1 (ZDIR=-1) and skipped the retract while
        # the needle was DOWN. safe_travel_to is a near-no-op when the needle is
        # already retracted, so always using it is safe.
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

    def _on_go_to_z_requested(self, z_mm: float) -> None:
        """Z-reference badge in the XZ view → move_z_absolute (zero-ref mm)."""
        if not getattr(self._controller, "is_zp_connected", False):
            return
        try:
            self._controller.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("Go-to-Z failed: %s", exc)

    def _on_travel_finished(self, ok: bool) -> None:
        """Worker-thread click-to-travel finished (queued to the GUI thread)."""
        if not ok:
            logger.warning(
                "Click-to-travel did not confirm (Z retract / XY arrival timed "
                "out, or the ZP board is not connected).")

    # ── config helpers ────────────────────────────────────────────

    def _current_config(self) -> SpheroidPickupConfig:
        return SpheroidPickupConfig(
            spheroid_diameter_um=float(self._diameter.value()),
            safety_factor=float(self._safety.value()),
            pickup_bore=self._bore.currentText() or "P1",
            pickup_speed_uL_s=float(self._pick_flow.value()),
            release_speed_uL_s=float(self._place_flow.value()),
            pick_z_offset_mm=float(self._pick_z.value()),
            place_z_offset_mm=float(self._place_z.value()),
            pick_dwell_s=float(self._pick_dwell.value()),
            place_dwell_s=float(self._place_dwell.value()),
            disengage_enabled=bool(self._disengage_enabled.isChecked()),
            disengage_volume_uL=float(self._disengage_vol.value()),
            disengage_rate_uL_s=float(self._disengage_rate.value()),
            sink_timing_enabled=bool(self._sink_timing_enabled.isChecked()),
            travel_margin_s=float(self._travel_margin.value()),
            release_enabled=bool(self._release_enabled.isChecked()),
            release_volume_uL=float(self._release_vol.value()),
        )

    _MAX_TARGET_DIAMETER_UM = 5000.0

    def _per_target_volume_enabled(self) -> bool:
        chk = getattr(self, "_per_target_volume", None)
        try:
            return bool(chk.isChecked())
        except AttributeError:
            return False

    def _effective_diameter_um(self, target) -> float:
        """A target's measured diameter, or 0.0 when it has none.

        ``PickPlaceTarget.size_um`` has serialized since v7.3.3 but nothing wrote
        it before v7.8, so a hand-edited or restored value is clamped to a sane
        band — it now changes an aspirate VOLUME, which scales as diameter cubed.
        """
        if not self._per_target_volume_enabled():
            return 0.0
        try:
            d = float(getattr(target, "size_um", 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0
        if d <= 0:
            return 0.0
        return min(d, self._MAX_TARGET_DIAMETER_UM)

    def _config_for_pick(self, base_cfg, pick):
        """``base_cfg`` for an unmeasured pick, else a copy sized from its Ø.

        Returning the SAME OBJECT when nothing was measured is what keeps the
        built queue byte-identical to the pre-v7.8 single-config queue.
        """
        d = self._effective_diameter_um(pick)
        if not d or abs(d - float(base_cfg.spheroid_diameter_um)) < 1e-9:
            return base_cfg
        return dataclasses.replace(base_cfg, spheroid_diameter_um=d)

    def _warn_release_residual(self, cfgs) -> None:
        """Warn when a fixed release volume meets varying aspirate volumes.

        With ``release_enabled`` the dispense is a fixed ``release_volume_uL``
        while a per-spheroid aspirate now varies with diameter cubed, so the
        retained residual differs per pick and ACCUMULATES across the queue —
        potentially past the needle's own internal volume. Advisory; the operator
        may well want it (that is what the release volume is for).
        """
        if not hasattr(self, "_status"):
            return
        cfgs = list(cfgs)
        if not cfgs or not getattr(cfgs[0], "release_enabled", False):
            return
        release = float(getattr(cfgs[0], "release_volume_uL", 0.0) or 0.0)
        if release <= 0:
            return
        retained = sum(max(0.0, c.compute_volume_uL() - release) for c in cfgs)
        if retained <= 0:
            return
        capacity = self._needle_volume_uL()
        msg = (f"Release volume is fixed at {release:.4f} µL, so ~{retained:.4f} "
               f"µL is retained across {len(cfgs)} pick(s)")
        if capacity and capacity > 0:
            msg += f" against a {capacity:.3f} µL needle"
            if retained > capacity:
                logger.warning("Spheroid run: %s — post-clean will be needed.", msg)
                self._status.setText(f"⚠ {msg} — run the post-clean.")
                return
        logger.info("Spheroid run: %s.", msg)

    def _plate_offset_to_zref(self, offset_mm: float) -> float | None:
        """Height above the calibrated plate bottom (mm) → zero-ref Z (mm),
        polarity-correct. Returns None if the plate bottom isn't calibrated."""
        ctrl = self._controller
        # Canonical: the controller's plate-bottom datum + print_z_dir (the same
        # conversion the print path uses).
        try:
            if hasattr(ctrl, "print_height_to_zref"):
                z = ctrl.print_height_to_zref(offset_mm)
                if z is not None:
                    return z
        except Exception:
            pass
        # Fallback: the page's own plate-bottom reference + polarity.
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

    def _refresh_volume_label(self):
        cfg = self._current_config()
        v = cfg.compute_volume_uL()
        text = f"V = {v:.4f} µL  (×{cfg.safety_factor:.2f})"
        if self._per_target_volume_enabled():
            text += " — for an unmeasured pick"
        if cfg.safety_factor < 1.0:
            # Allowed (the operator asked for the wider range) but worth saying:
            # the carrier column is then smaller than the spheroid itself.
            text += "  ⚠ below 1× the spheroid's own volume"
        self._volume_label.setText(text)
        self._refresh_fit_summary()

    def _update_button_state(self, *_):
        balanced = self._picker.is_balanced()
        has_bore = self._bore.count() > 0
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        # A mosaic scan is also driving the stage (and the non-refcounted poller
        # suspend), so Start must wait for it — see _stage_busy.
        page = getattr(self, "_scan_page", None)
        scanning = page is not None and page.is_scanning()
        self._start_btn.setEnabled(
            balanced and has_bore and not running and not scanning)
        self._abort_btn.setEnabled(running)

    # ── Start / Abort ─────────────────────────────────────────────

    def _on_start(self):
        # ONE busy check for every stage driver — the executor thread (which used
        # to be checked here alone, and silently) AND a mosaic scan, which also
        # drives the stage and toggles the non-refcounted poller suspend.
        # _update_button_state greys Start out too, but a race or a programmatic
        # call must not get through either, and now it says why.
        if self._stage_busy():
            return

        if not self._picker.is_balanced():
            self._status.setText(
                "Each pick needs a paired place — pick and place counts must match.")
            return

        pairs = self._picker.pairs()
        cfg = self._current_config()

        # Resolve the pick/place heights (above plate bottom) to zero-ref Z and
        # gate on the calibration they depend on, so the executor never falls
        # back to a bogus operating Z (the old default 0.0 left the needle
        # jiggling at the wrong height and timing out).
        if self._safe_z is None:
            self._status.setText(
                "No safe Z configured — set it on the Calibration page "
                "before running pick & place.")
            return

        # The ZP (Z + pump) board is REQUIRED: it retracts the needle before
        # each cross-position move and runs the aspirate/dispense. If it is
        # disconnected, safe_travel_to would skip its retract and the stage
        # would drive XY with an unretracted needle — refuse to start.
        if not getattr(self._controller, "is_zp_connected", False):
            self._status.setText(
                "ZP (Z + pump) board not connected — pick & place needs it to "
                "retract the needle and run the pumps. Reconnect it first.")
            return

        # v7.9 backstop: the combo is already restricted, but a settings profile
        # saved before the assembly changed can still carry another bore's pump.
        datum = _reagent_prep.datum_bore_pump(self._hw_config)
        if datum and str(cfg.pickup_bore).strip().upper() != datum:
            self._status.setText(
                f"This workflow drives bore 1 ({datum}) on a multi-bore "
                f"assembly, but the saved pump is {cfg.pickup_bore}. The pickup "
                f"volume, bore area and sink timing are all resolved from bore 1, "
                f"so another bore would mis-size every aspirate. Select {datum} "
                f"in Settings.")
            return

        pick_z = self._plate_offset_to_zref(cfg.pick_z_offset_mm)
        place_z = self._plate_offset_to_zref(cfg.place_z_offset_mm)
        if pick_z is None or place_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — calibrate it on the "
                "Calibration page so the pick/place heights can be resolved.")
            return

        # (The per-spheroid fit advisory runs once the per-pair configs are built,
        # further down — with per-spheroid sizing there is no single diameter to
        # check here.)

        # Sink-timing model: resolve the bore geometry + calibrated curve and
        # gate on them so the executor doesn't silently fall back to the fixed
        # carrier.
        bore_area = self._bore_area_mm2()
        sink_curve = None
        if cfg.sink_timing_enabled:
            if bore_area <= 0.0:
                self._status.setText(
                    "Sink timing needs the needle inner Ø (Hardware Setup → "
                    "Needle) to size the aspirate — set it or turn sink timing off.")
                return
            try:
                from SupportClasses.SpheroidSinkCalibrationStore import get_store
                sink_curve = get_store().get_curve()
            except Exception:
                sink_curve = None
            if sink_curve is None:
                self._status.setText(
                    "Sink timing needs a calibrated curve — run “Calibrate sink "
                    "timing…” in Settings, or turn sink timing off.")
                return

        # Resolve the prep / post-clean (needle conditioning) inputs up front and
        # gate on the service-well locations they inherit from Hardware Setup → Ink.
        prep_enabled = self._prep_check.isChecked()
        clean_enabled = self._post_clean_check.isChecked()
        service_positions: dict[str, tuple[float, float]] = {}
        service_z = None
        needle_uL = 0.0
        if prep_enabled or clean_enabled:
            needed = set()
            if prep_enabled:
                needed |= {"waste", "oil", "wash", "buffer"}
            if clean_enabled:
                needed |= {"waste", "wash", "buffer"}
            needle_uL = self._needle_volume_uL()
            if needle_uL <= 0:
                self._status.setText(
                    "Prep / clean needs the needle inner diameter + length "
                    "(Hardware Setup → Needle), or turn them off.")
                return
            service_positions, _missing = self._resolve_service_positions()
            missing = [r for r in needed if r not in service_positions]
            if missing:
                self._status.setText(
                    "Prep / clean needs these reagent wells assigned in Hardware "
                    "Setup → Ink (Reagent Locations) and calibrated: "
                    f"{', '.join(sorted(missing))} — or turn them off.")
                return
            service_z = self._plate_offset_to_zref(float(self._service_z.value()))
            if service_z is None:
                self._status.setText(
                    "Plate bottom Z is not calibrated — can't resolve the "
                    "service dip Z for prep / clean.")
                return

        # One config PER PAIR, so a measured spheroid is aspirated with the
        # volume computed from ITS OWN diameter (× the safety factor, which
        # replace() preserves). PickPlaceOperation.config is already per-op and
        # _planned_aspirate_uL reads only cfg.compute_volume_uL(), so this needs
        # no executor change.
        #
        # An unmeasured pick keeps the SAME config OBJECT as before, so with
        # per-spheroid sizing off — or every size_um still 0 — the queue is
        # byte-identical to the pre-v7.8 single-config queue.
        queue = OperationQueue()
        per_op_cfgs: list = []
        for pick, place in pairs:
            cfg_i = self._config_for_pick(cfg, pick)
            per_op_cfgs.append(cfg_i)
            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=OperationType.SPHEROID_PICKUP,
                source_target=pick,
                dest_target=place,
                config=cfg_i,
            )
            queue.add(op)

        # Does each spheroid physically fit the orifice? ADVISORY only — a
        # deformable spheroid can squeeze through a slightly smaller tip, so we
        # warn per pick and proceed rather than blocking the run.
        self._warn_spheroid_fits(per_op_cfgs)
        self._warn_release_residual(per_op_cfgs)

        executor = PickPlaceExecutor(self._controller, self._hw_config)
        executor.safe_z_mm = float(self._safe_z)
        executor.pick_z_mm = pick_z
        executor.place_z_mm = place_z
        # Sink-timing model inputs (bore area + calibrated curve). No-op unless
        # cfg.sink_timing_enabled (gated above); harmless to set otherwise.
        executor.bore_area_mm2 = bore_area
        # Two-stage geometry so volume↔lift stays correct once a spheroid
        # leaves a pulled tip and enters the wide barrel.
        executor.bore_profile = self._bore_profile()
        executor.sink_curve = sink_curve
        # Advanced motion / timeout knobs (always applied).
        executor.intra_well_retract_mm = float(self._intra_retract.value())
        executor.z_timeout_s = float(self._z_timeout.value())
        executor.xy_timeout_s = float(self._xy_timeout.value())
        if prep_enabled or clean_enabled:
            executor.needle_volume_uL = needle_uL
            executor.service_z_mm = service_z
            executor.prep_bore = cfg.pickup_bore
            executor.prep_rate_uL_s = float(self._prep_rate.value())
            # Wash + buffer reload are used by BOTH prep and post-clean, so apply
            # the operator's settings whenever either runs (a clean-only run must
            # not silently fall back to the executor defaults).
            executor.buffer_needles = float(self._buffer_needles.value())
            executor.wash_cycles = int(self._wash_cycles.value())
            executor.wash_z_amplitude_mm = float(self._wash_z_amp.value())
            executor.wash_xy_amplitude_um = float(self._wash_xy_amp.value())
            executor.wash_dwell_s = float(self._wash_dwell.value())
            for role in ("waste", "oil", "wash", "buffer"):
                if role in service_positions:
                    setattr(executor, f"{role}_well_pos", service_positions[role])
        if prep_enabled:
            executor.do_prep = True
            executor.oil_needles = float(self._oil_needles.value())  # oil = prep-only
        if clean_enabled:
            executor.do_post_clean = True
            executor.post_dispense_needles = float(self._post_dispense.value())
        # Bridge callbacks → Qt signals so the GUI updates on the main thread.
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

        n = len(pairs)
        self._status.setText(
            f"Running {n} spheroid pickup{'s' if n > 1 else ''}…")
        self._exec_thread = threading.Thread(
            target=worker, name="SpheroidPickupExecutor", daemon=True)
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
