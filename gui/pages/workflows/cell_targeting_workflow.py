"""cell_targeting_workflow.py — Cell Targeting & Removal workflow page.

v7.5.x: A trypsinize-in-place-then-extract workflow, built on the same scaffold
as the Spheroid Pick & Place page (shared LiveTargetPicker + WorkspaceTargetView
+ XZSideView + StandardJogContextPanel + the PickPlaceExecutor bridge).

Operator flow (one op per picked removal→placement pair):

    1. Standard needle prep (waste → oil → wash → buffer) — once, like Quick
       Print / Spheroid Pick & Place.
    2. Load the needle with a cell-release reagent (e.g. trypsin) assigned to a
       reagent well.
    3. Travel to the cell-removal location and lower to a removal Z (a small
       height — default 0.1 mm — off the plate bottom).
    4. SLOWLY push in a small column of reagent (needle inner area × push depth).
    5. Wait a user-defined incubation time.
    6. QUICKLY pull up a multiple (default 2×) of the pushed volume.
    7. Travel to the user-defined placing location and dispense the cells.
    8. Needle clean-up + reset (waste → wash → buffer) — once, after the loop.

Steps 1 and 8 bracket the whole loop (run once each by the executor); steps 2–7
run per picked pair. The reagent well + service wells are inherited from
Hardware Setup → Ink (Reagent Locations); all Z heights are expressed as a
height above the calibrated plate bottom and resolved polarity-safely.

── v7.9: TWO TABS ────────────────────────────────────────────────────

The page is **Plan** then **Run** (operator, 2026-08-04: *"we want two tabs, one
for planing and one for runing the experiment"*), superseding the v7.9
Setup / Well-Survey split — planning an experiment and running it want different
things on screen, so the setup controls stopped competing with the live views.

* **Plan** — the well beside the protocol. The well is a live INSTANCE of the
  Fluorescence Mosaic page (``embedded=True``) with the **live camera frame
  floating over the mosaic at the current stage position**: the mosaic is the map,
  the feed is "you are here". The view FOLLOWS the camera (map-app model —
  dragging breaks away, the ⊙ toggle resumes), and a click in Point mode sends
  the stage there. The protocol column is
  :class:`CellTargetingSetupPanel`: the per-bore program table, target types, the
  dosing bore's reagent well, and the structural run parameters.
* **Run** — auto-selected on Start. The XY / XZ instruments, a per-cell progress
  record, and the four TUNING knobs (dose · incubation · pull × and flow ·
  removal height), which live here and only here.

Two consequences worth stating, because both are load-bearing:

* A widget has exactly one parent, so a control on Run is NOT on Plan. Plan shows
  the tuning as read-only text. Two editing surfaces for one setting is the
  ``UNIFIED_MOSAIC_CALIBRATION`` failure.
* Mid-run edits reach the cells still to come (operator decision), so a batch is
  no longer one uniform experiment. That is why :class:`LiveTuning` is an
  immutable snapshot published under a lock, applied only between operations, and
  recorded per cell on ``PickPlaceOperation.applied_tuning``.

The embedded scan page is constructed ``owns_camera=False``: Qt delivers
``showEvent`` to a CHILD BEFORE ITS PARENT, so ownership of the microscope camera
has to be declared rather than won by whoever reaches it first.

Embedding a second stage driver is what makes :meth:`_stage_busy` mandatory: the
scan worker toggles ``suspend_position_poller``, which is a plain bool and NOT
refcounted, so whichever of the two finishes first would re-enable the poller
underneath the other — the v7.5.x false ZP-disconnect. Every motion entry point on
this page therefore routes through it.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import replace
from typing import Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QListWidget,
    QPushButton, QDoubleSpinBox, QComboBox, QFrame, QSizePolicy, QSplitter,
    QMessageBox, QCheckBox, QSpinBox, QTabWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card, FormRow
from gui.widgets.section_stack import (
    PromotedSectionsPanel, wire_section_promotion)
from gui.widgets.live_target_picker import LiveTargetPicker, PROV_MOSAIC
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
from gui.pages.workflows._reagent_prep import (
    SERVICE_ROLES, service_well_names, resolve_service_positions,
    needle_volume_uL, resolve_pickup_well,
)
from gui.pages.workflows.cell_targeting_setup_panel import (
    CellTargetingSetupPanel,
)

from SupportClasses.PhysicalModels import (
    needle_bore_at, needle_orifice_area_mm2, needle_bore_offset_um,
)
from SupportClasses.PickAndPlaceManager import (
    BoreRole, LiveTuning, OperationQueue, OperationType, PickPlaceExecutor,
    PickPlaceOperation, CellRemovalConfig,
)

logger = logging.getLogger(__name__)


#: µL/nL formatting, imported rather than re-implemented so the Setup tab, the
#: readiness checklist and the confirm dialog cannot print the same volume three
#: different ways. At nanolitre scale a plain ``:.4f`` renders a real 0.07 nL dose
#: as "0.0001 µL", which reads as zero.
from SupportClasses.CellRemovalReadiness import _fmt_uL     # noqa: E402
from gui.pages.workflows.cell_targeting_setup_panel import (   # noqa: E402
    NL_PER_UL as _NL_PER_UL, _MIN_DOSE_NL,
)


def _safe_float(v):
    """``float(v)`` for a REAL number, else None.

    Strict on purpose: ``MagicMock`` implements ``__float__`` and returns 1.0, so
    a duck-typed coercion would let a stubbed page fabricate plausible-looking
    volumes and clearances in the readiness model. None means "unknown", which
    the evaluator is required never to treat as blocking.
    """
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


class _ExecutorBridge(QObject):
    """Bridges PickPlaceExecutor callbacks (daemon thread) → Qt signals."""

    op_started = Signal(object)        # PickPlaceOperation
    op_completed = Signal(object)
    op_failed = Signal(object, str)
    progress = Signal(int, int, str)   # completed, total, message
    sub_step = Signal(object, str)     # operation, step text
    finished = Signal(bool)            # True if all completed, False otherwise


class CellTargetingWorkflowPage(QWidget):
    """Cell Targeting & Removal workflow page.

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
            "replace_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }

        # Left context panel — lazy, identical lifecycle to JogControlPage
        self._context_widget: StandardJogContextPanel | None = None

        # v7.9: the embedded Fluorescence Mosaic page (the viewer tab) and the
        # last point clicked on its mosaic (ABSOLUTE stage µm).
        self._scan_page = None
        self._setup_panel: CellTargetingSetupPanel | None = None
        self._mosaic_point_um: tuple[float, float] | None = None
        # ⚠ THIS page owns the microscope camera, not the embedded scan page.
        # Before this the camera was started ONLY by `_scan_page.showEvent`, so
        # `LiveTargetPicker` received frames purely because it happened to share
        # a tab with that page — and any layout that separated them would have
        # left the picker showing a dead feed. Owning it here also neuters the
        # embedded page's own `hideEvent` stop (its `_camera_started_by_us`
        # stays False because it finds the camera already running), which closes
        # a latent hazard: leaving a surface mid-scan used to stop the camera out
        # from under a running scan worker.
        self._camera_started_by_us: bool = False

        # ── live tuning ───────────────────────────────────────────────
        # An immutable snapshot published from the GUI thread and read by the
        # executor thread, under a lock. Operator decision: mid-run edits DO
        # reach the cells still to come, so this has to cross threads safely and
        # be recorded per cell (see `PickPlaceOperation.applied_tuning`).
        self._tuning_lock = threading.Lock()
        self._live_tuning: Optional[LiveTuning] = None
        #: Cells already run at least once, so "test one cell" advances.
        self._tried_target_ids: set[str] = set()

        self._executor: Optional[PickPlaceExecutor] = None
        self._exec_thread: Optional[threading.Thread] = None
        # True after a run stopped with a bore still loaded — enables the manual
        # "Clean needle now" recovery (see _on_clean_now for why it is manual).
        self._needs_clean: bool = False
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

        # v7.9: every config widget is created EAGERLY and up front, before either
        # surface lays one out, so `_current_config()` / `_on_start()` and the
        # `__new__`-partial page tests can read them without showing anything.
        self._create_config_widgets()

        # Comprehensive settings popout (scrollable, saveable). It keeps the
        # advanced + Common-linked fields; the primary ones are laid out on the
        # Setup tab and registered here via `register_external` (v7.7), so a
        # setting has exactly ONE editing surface but still rides along with the
        # saved profile.
        self._settings_dialog = WorkflowSettingsDialog(
            "cell_targeting", "Cell Targeting & Removal",
            parent=self, on_change=self._on_settings_changed,
            migrate=self._migrate_legacy_settings)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())

        # PLAN then RUN. Planning an experiment and running it want different
        # things on screen, so the setup controls stop competing with the live
        # views — and the run surface can auto-select itself on Start.
        self._tabs = QTabWidget(self)
        self._tabs.addTab(self._build_plan_tab(), "Plan")
        self._tabs.addTab(self._build_run_tab(), "Run")
        outer.addWidget(self._tabs, stretch=1)

        # The run row stays OUTSIDE the tabs so Start / Abort and the status line
        # are reachable from either one (the v7.7 Quick Print zone pattern).
        outer.addWidget(self._build_run_row())

        # Promoted fields are registered after the Setup tab has laid them out,
        # and the bore table rides along through the extra-state hook — both
        # BEFORE load_last(), or a restored profile would have nowhere to land.
        self._register_promoted_fields(self._settings_dialog)
        self._settings_dialog.set_extra_state(
            self._collect_extra_state, self._apply_extra_state)

        # Periodic position refresh so the workspace + XZ tracks the stage.
        from PySide6.QtCore import QTimer
        self._pos_timer = QTimer(self)
        self._pos_timer.setInterval(200)
        self._pos_timer.timeout.connect(self._refresh_position_indicators)
        self._pos_timer.start()

        self._refresh_volume_label()
        self._refresh_readiness()
        # v7.21: a section moved out of ⚙ Settings lands in the drawer, which is
        # hidden (zero footprint) until something is in it. AFTER the run row on
        # purpose, so Start / Abort never move.
        self._promoted_panel = PromotedSectionsPanel()
        outer.addWidget(self._promoted_panel)
        self._layout_store = wire_section_promotion(
            self, self._settings_dialog, self._promoted_panel.stack,
            settings=self._settings, workflow_id="cell_targeting")

        self._settings_dialog.load_last()
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._refresh_trypsin_status()
        self._update_settings_summary()

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)

        title = QLabel("Cell Targeting & Removal")
        title.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(14)}pt;"
            f"font-weight: 600;"
        )
        row.addWidget(title)

        hint = QLabel("Pick = removal location(s) · Place = placement(s)")
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
            "Open the full, saveable settings for this workflow (profiles, prep "
            "sub-parameters, motion timeouts, global pump values, locations).")
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

    # ── Plan tab ──────────────────────────────────────────────────

    def _build_plan_tab(self) -> QWidget:
        """Choose the cells, then say what to do to them.

        Left: the well — the fluorescence mosaic with the live camera floating
        over it at the current stage position — plus the removal / placement
        lists. Right: the protocol. The two things planning needs, side by side.
        """
        split = QSplitter(Qt.Horizontal, self)
        split.setChildrenCollapsible(False)
        split.addWidget(self._build_survey_column())
        split.addWidget(self._build_protocol_panel())
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        return split

    def _build_protocol_panel(self) -> QWidget:
        """The primary establishment surface.

        The panel owns the per-bore table / target types / trypsin reagent; the
        assembly-wide fields below are created by this page (so `_current_config`
        can read them without the tab being shown) and merely LAID OUT here.
        """
        panel = CellTargetingSetupPanel(self)
        panel.programs_changed.connect(self._on_programs_changed)
        panel.goto_targets_requested.connect(self._goto_targets_tab)
        self._setup_panel = panel
        # The readiness card lives on the panel; the HOST owns the evaluation
        # (only it can read the controller, the picker and the mosaic).
        self._ready_list = panel.readiness_list

        grp = panel.add_group("Step 3 · Heights (above the calibrated plate bottom)")
        # ⚠ The REMOVAL height is not here — it is one of the four tuning knobs and
        # lives on the Run tab. A widget has exactly one parent, so a control
        # cannot appear on both surfaces without becoming two editing surfaces for
        # one number: the `UNIFIED_MOSAIC_CALIBRATION` failure. Plan echoes it as
        # read-only text instead (see `_refresh_tuning_echo`).
        grp.add("Place Z (↑ bottom)", self._place_z)
        # ⚠ The reagent dip Z belongs HERE, with the other heights, because it
        # governs EVERY reagent well dip — including a dosing bore's own well
        # (`PickAndPlaceManager._execute_cell_removal` step 0 passes
        # `reagent_dip_z_mm` for the trypsin bore). Filing it under a group
        # titled "single-bore sequence" is what led to it being greyed out and
        # labelled "Not used" in the one configuration that needs it most.
        grp.add("Reagent dip Z (↑ bottom)", self._reagent_z)

        grp = panel.add_group("Cell-release reagent (single-bore sequence)")
        grp.add("Pump / bore (fallback)", self._bore)
        grp.add("Cell-release reagent", self._reagent_combo)
        grp.add_widget(self._reagent_status)

        # ⚠ "Column", not "Push depth": the per-bore table has its own dose
        # depth, and two live controls with the same label 15 cm apart is a
        # wrong-field-entry waiting to happen.
        grp = panel.add_group("Aspirating bore — reagent column & extraction")
        grp.add("Column push flow (slow)", self._push_speed)
        # Laid out but HIDDEN: it is derived from the dose, and showing it would
        # be a second place to read the same quantity. Still parented here so it
        # stays saved / loaded / imported exactly as before.
        self._push_depth_row = grp.add("≙ column depth", self._push_depth)
        self._column_note = grp.add_note("")
        grp.add_widget(self._volume_label)
        # The four tuning knobs live on Run; this is the read-only echo, so Plan
        # still shows the whole recipe without owning any of it twice.
        self._tuning_echo = grp.add_note("")
        self._hide_form_row(self._push_depth)

        # Collapsed by default: three checkboxes that are already ON, whose
        # sub-parameters live in ⚙ Settings. It is the part of this page an
        # operator sets once and then stops looking at, so it should not cost
        # vertical space on every visit.
        grp = panel.add_group("Needle prep / clean", collapsible=True,
                              collapsed=True)
        grp.add_widget(self._prep_check)
        grp.add_widget(self._clean_check)
        grp.add_widget(self._wash_after_pickup_check)
        grp.card().setToolTip(
            "The prep sub-parameters (service dip Z, flows, needle multiples, "
            "wash mechanics) are shared with Common Print Settings and live in "
            "⚙ Settings.")
        grp.add_widget(self._prep_status)

        panel.finalize()
        return panel

    @staticmethod
    def _hide_form_row(widget) -> None:
        """Hide the ``FormRow`` a widget was laid out in, keeping its parenting.

        The widget stays a descendant of the tab (so it is still saved/loaded and
        the promoted-field contract holds) but claims no space and shows no label.
        """
        from gui.widgets.components import FormRow
        w = widget
        for _ in range(4):
            w = w.parent() if w is not None else None
            if w is None:
                return
            if isinstance(w, FormRow):
                w.setVisible(False)
                return

    def _on_programs_changed(self) -> None:
        """A per-bore role / pump / target type / push parameter changed."""
        # Which bore ASPIRATES sets the orifice the reagent column is metered
        # through, so the push/pull readout has to follow a role change too —
        # it also re-runs the reagent status line. And because the DOSE is what
        # the operator chose, a change of aspirating bore keeps the dose and
        # re-derives the depth, not the other way round.
        self._refresh_release_group_state()
        self._sync_dose_to_needle()
        self._refresh_volume_label()
        self._refresh_trypsin_status()
        self._update_settings_summary()
        self._refresh_readiness()

    def _refresh_release_group_state(self) -> None:
        """Grey the single-bore reagent controls when a dosing bore owns the dose.

        The tab used to imply a dosing bore REPLACED this push while the executor
        did BOTH, and the Start gate demanded a cell-release reagent
        unconditionally — which is precisely what forced the operator into the
        double-dose configuration. Now the dosing bore does replace it, so these
        controls must visibly stop applying, WITH a reason (the discipline the
        per-bore rows already follow rather than greying out in silence).
        """
        dosing = False
        try:
            dosing = self._program_for_role(BoreRole.PUSH_REAGENT) is not None
        except Exception:
            dosing = False
        tip = (
            "Not used: a dosing bore delivers the reagent, so the aspirating "
            "bore loads nothing and only pulls the cell up."
            if dosing else
            "The reagent the aspirating bore loads, pushes onto the cell, and "
            "then pulls back up with it.")
        if self._reagent_combo is not None:
            self._reagent_combo.setEnabled(not dosing)
            self._reagent_combo.setToolTip(tip)
        # `_bore` stays enabled: it is still the aspirate-pump fallback.
        # `_reagent_z` is NOT greyed and lives with the other heights: it is the
        # dip height for EVERY reagent well, and with a dosing bore armed it is
        # that bore's own well the needle dips into (executor step 0). Greying it
        # here — as this method used to — made the one height a two-bore run
        # depends on look inapplicable.
        if getattr(self, "_column_note", None) is not None:
            try:
                idx = self._aspirate_bore_index()
                self._column_note.setText(
                    f"Applies to bore {idx + 1}, the aspirating bore."
                    + (" With a dosing bore armed, its column no longer doses "
                       "anything — it only sizes the extraction pull."
                       if dosing else ""))
            except Exception:
                pass

    # ── Plan tab: the well ────────────────────────────────────────

    def _build_survey_column(self) -> QWidget:
        """The well: scan it, see where the stage is, choose the cells.

        The left half is a real INSTANCE of the Fluorescence Mosaic page
        (``embedded=True`` drops only its back-button header), so a change to the
        single-well scan workflow shows up here automatically and the two cannot
        diverge — the pattern ``SpheroidPickupWorkflowPage`` and
        ``FullPrintWorkflowPage`` both use.
        """
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)

        wrap = QWidget(self)
        layout = QVBoxLayout(wrap)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(s(6))

        vsplit = QSplitter(Qt.Vertical, wrap)
        vsplit.setChildrenCollapsible(False)

        scan_wrap = QWidget(vsplit)
        scan_layout = QVBoxLayout(scan_wrap)
        scan_layout.setContentsMargins(0, 0, 0, 0)
        scan_layout.setSpacing(s(4))
        # ⚠ `owns_camera=False`: THIS page starts and stops the microscope camera.
        # Qt delivers showEvent to a child BEFORE its parent, so with the scan
        # page on the visible Plan tab it would otherwise always claim the camera
        # first and then stop it from its own hideEvent — cutting the feed the
        # picker and (step 6) the composited well view depend on.
        self._scan_page = FluorescenceMosaicWorkflowPage(
            self._controller, self._settings, self._camera_manager,
            embedded=True, owns_camera=False)
        # A scan finishing OR a well change loads a different mosaic, so the
        # point picked off the previous one must not stay armed — it would send
        # the stage to a coordinate from another well.
        self._scan_page.mosaic_ready.connect(self._on_mosaic_ready)
        scan_layout.addWidget(self._scan_page, stretch=1)
        scan_layout.addWidget(self._build_mosaic_point_row())
        vsplit.addWidget(scan_wrap)

        self._picker = LiveTargetPicker(self._controller, self._camera_manager)
        self._picker.picks_changed.connect(self._on_targets_changed)
        self._picker.places_changed.connect(self._on_targets_changed)
        vsplit.addWidget(self._picker)

        vsplit.setStretchFactor(0, 3)
        vsplit.setStretchFactor(1, 2)
        layout.addWidget(vsplit)

        # Interactive-items mode frees the left button for scene clicks and moves
        # panning to the middle button — the same trade the spheroid survey makes.
        # It is ALSO the Hand/Point toggle: interactive = Point (left clicks),
        # non-interactive = Hand (left pans). One mechanism, two names.
        try:
            view = self._scan_page.mosaic_view()
            view.set_interactive_items(True)
            view.scene_clicked.connect(self._on_mosaic_scene_clicked)
            view.follow_broken.connect(self._on_follow_broken)
            view.set_follow(True)
        except Exception as exc:
            logger.debug("mosaic view wiring failed: %s", exc)
        scan_layout.insertWidget(0, self._build_well_view_toolbar())
        return wrap

    # ── the live frame floating over the mosaic ───────────────────

    def _build_well_view_toolbar(self) -> QWidget:
        """Follow / Hand / Point, above the well view."""
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(6))

        self._follow_btn = QPushButton("⊙ Follow")
        self._follow_btn.setCheckable(True)
        self._follow_btn.setChecked(True)
        self._follow_btn.setCursor(Qt.PointingHandCursor)
        self._follow_btn.setToolTip(
            "Hold the live camera frame at the centre of the view and scroll the "
            "mosaic under it. Zooming keeps it centred; dragging the mosaic "
            "breaks away, and this button brings you back.")
        self._follow_btn.toggled.connect(self._on_follow_toggled)
        row.addWidget(self._follow_btn)

        self._hand_btn = QPushButton("✋ Hand")
        self._point_btn = QPushButton("✛ Point")
        for b, tip in (
            (self._hand_btn, "Drag to pan the mosaic. Clicks do nothing."),
            (self._point_btn, "Click a cell to send the stage there, then click "
                              "it on the live feed to confirm it as a target."),
        ):
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setToolTip(tip)
            row.addWidget(b)
        self._point_btn.setChecked(True)
        self._hand_btn.clicked.connect(lambda: self._set_well_view_mode("hand"))
        self._point_btn.clicked.connect(lambda: self._set_well_view_mode("point"))

        row.addStretch(1)
        self._well_view_note = QLabel("")
        self._well_view_note.setWordWrap(True)
        self._well_view_note.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        row.addWidget(self._well_view_note, stretch=1)
        return frame

    def _set_well_view_mode(self, mode: str) -> None:
        """Hand = the view pans on a left-drag; Point = a left-click travels."""
        point = (mode == "point")
        self._hand_btn.setChecked(not point)
        self._point_btn.setChecked(point)
        try:
            self._scan_page.mosaic_view().set_interactive_items(point)
        except Exception:
            logger.debug("could not set the well-view mode", exc_info=True)

    def _on_follow_toggled(self, on: bool) -> None:
        try:
            self._scan_page.mosaic_view().set_follow(bool(on))
        except Exception:
            logger.debug("could not set follow", exc_info=True)
        self._refresh_follow_button()

    def _on_follow_broken(self) -> None:
        """The operator dragged away — reflect it without fighting them."""
        if getattr(self, "_follow_btn", None) is None:
            return
        self._follow_btn.blockSignals(True)
        self._follow_btn.setChecked(False)
        self._follow_btn.blockSignals(False)
        self._refresh_follow_button()

    def _refresh_follow_button(self) -> None:
        """Accent the toggle when the camera is off-screen, so the way back is
        where the operator would look for it."""
        btn = getattr(self, "_follow_btn", None)
        if btn is None:
            return
        off = False
        if not btn.isChecked():
            try:
                view = self._scan_page.mosaic_view()
                pt = self._camera_scene_point()
                off = bool(pt is not None
                           and not view.viewport().rect().contains(
                               view.mapFromScene(pt)))
            except Exception:
                off = False
        btn.setText("⊙ Follow ●" if off else "⊙ Follow")
        btn.setToolTip(
            "The live camera is outside the view — click to bring it back."
            if off else
            "Hold the live camera frame at the centre of the view and scroll the "
            "mosaic under it. Zooming keeps it centred; dragging the mosaic "
            "breaks away, and this button brings you back.")

    def _camera_scene_point(self):
        """The stage's current position in mosaic-pixel (scene) coords, or None.

        Routed through ``forward_project_um`` — the exact inverse of the
        ``back_project_px`` the travel path uses — so the frame is drawn where
        the stage really is. Two separate copies of this transform would let the
        overlay and the motion disagree, which is the whole hazard here.
        """
        ctx = self._mosaic_context()
        if not ctx:
            return None
        scale = float(ctx.get("mosaic_scale") or 0.0)
        if scale <= 0:
            return None
        xy = self._stage_xy_um()
        if xy is None:
            return None
        from SupportClasses.SpheroidDetector import forward_project_um
        try:
            px, py = forward_project_um(
                xy, ctx["extent_um"], scale, ctx.get("shift_um") or (0.0, 0.0))
        except Exception:
            return None
        from PySide6.QtCore import QPointF
        return QPointF(float(px), float(py))

    def _stage_xy_um(self):
        """Absolute stage XY in µm from the poller cache, or None."""
        try:
            xy = self._controller.get_xy_position(cached=True)
            return (float(xy[0]), float(xy[1]))
        except Exception:
            return None

    def _refresh_live_frame_overlay(self) -> None:
        """Draw the live camera frame over the mosaic at the stage position.

        Both are ``QGraphicsPixmapItem``s in a scene whose coordinates ARE mosaic
        pixels, so Qt's own view transform does the scaling — which sidesteps the
        documented ``JogWorkspaceView`` trap of re-scaling an ~18 MB mosaic pixmap
        on every paint. That would bite hard here: this view repaints on every
        stage tick and every camera frame.
        """
        view = None
        try:
            view = self._scan_page.mosaic_view()
        except Exception:
            return
        if view is None:
            return
        pt = self._camera_scene_point()
        view.follow_point(pt)
        self._refresh_follow_button()

        # ⚠ `_ZoomImageView.set_image` calls `scene.clear()`, which DELETES the
        # underlying C++ object — it does not merely remove it. Touching a stale
        # wrapper raises, so validity is checked, not assumed.
        item = self._live_frame_item_or_none()
        frame_px = self._live_frame_pixmap()
        if pt is None or frame_px is None:
            if item is not None and item.scene() is not None:
                item.scene().removeItem(item)
            return

        ctx = self._mosaic_context() or {}
        scale = float(ctx.get("mosaic_scale") or 0.0)
        cam_um_per_px = self._live_um_per_px(frame_px.width())
        if scale <= 0 or cam_um_per_px <= 0:
            return
        # mosaic px per camera px
        k = cam_um_per_px * scale

        from PySide6.QtGui import QTransform
        from PySide6.QtWidgets import QGraphicsPixmapItem
        if item is None:
            item = QGraphicsPixmapItem()
            item.setZValue(10.0)
            item.setOpacity(0.92)
            self._live_frame_item = item
        # `_ZoomImageView.set_image` calls `scene.clear()`, which destroys any
        # host item — so re-add rather than assume it survived.
        scene = view.scene_obj()
        if item.scene() is not scene:
            scene.addItem(item)
        item.setPixmap(frame_px)

        rot, flip_x, flip_y = self._live_frame_orientation()
        sx = -1.0 if flip_x else 1.0
        sy = -1.0 if flip_y else 1.0
        t = QTransform()
        t.translate(pt.x(), pt.y())
        t.rotate(rot)
        t.scale(sx * k, sy * k)
        t.translate(-frame_px.width() / 2.0, -frame_px.height() / 2.0)
        item.setTransform(t)

    def _live_frame_item_or_none(self):
        """The overlay item, or None if Qt has destroyed it.

        ``QGraphicsScene.clear()`` deletes its items' C++ objects, so the Python
        wrapper this page holds can outlive them; any attribute access on it then
        raises. ``shiboken6.isValid`` is the supported way to ask.
        """
        item = getattr(self, "_live_frame_item", None)
        if item is None:
            return None
        try:
            from shiboken6 import isValid
            if not isValid(item):
                self._live_frame_item = None
                return None
        except ImportError:                        # pragma: no cover
            try:
                item.scene()
            except RuntimeError:
                self._live_frame_item = None
                return None
        return item

    def _live_frame_pixmap(self):
        """The current camera frame as a QPixmap, or None."""
        mgr = self._camera_manager
        if mgr is None:
            return None
        try:
            frame = mgr.get_current_frame(self._camera_slot())
        except Exception:
            return None
        if frame is None:
            return None
        try:
            # The shared BGR→QPixmap helper the jog workspace's own mosaic
            # overlay uses, rather than a second conversion here.
            from gui.widgets.jog_workspace_view import pixmap_from_bgr
            return pixmap_from_bgr(frame)
        except Exception:
            logger.debug("could not convert the live frame", exc_info=True)
            return None

    def _live_um_per_px(self, frame_w: int) -> float:
        """The live frame's µm/px, rescaled to its ACTUAL width."""
        mgr = self._camera_manager
        if mgr is None:
            return 0.0
        cam = self._camera_slot()
        try:
            return float(mgr.effective_um_per_px(cam, int(frame_w)) or 0.0)
        except Exception:
            pass
        try:
            return float(mgr.get_um_per_px(cam) or 0.0)
        except Exception:
            return 0.0

    def _live_frame_orientation(self) -> tuple[float, bool, bool]:
        """``(rotation_deg, flip_x, flip_y)`` — the ONE measured orientation.

        ⚠ ``CameraManager.full_orientation`` returns ``(flip_x, flip_y,
        rotation_deg)`` — flips FIRST. Unpacking it rotation-first silently
        swaps a mirror for an angle, which would draw the frame at the right
        place with the wrong handedness. Named locals, not positional reuse.
        """
        mgr = self._camera_manager
        if mgr is None:
            return (0.0, False, False)
        try:
            flip_x, flip_y, rot = mgr.full_orientation(self._camera_slot())
            return (float(rot or 0.0), bool(flip_x), bool(flip_y))
        except Exception:
            return (0.0, False, False)

    # ── Run tab ───────────────────────────────────────────────────

    def _build_run_tab(self) -> QWidget:
        """Watch it work, try one cell, and tune as you go.

        ``_workspace_view`` and ``_xz_view`` live here because that is what they
        are for — the XZ view especially, while a 0.100 mm removal height is the
        live setting. The four tuning knobs live here and ONLY here, so there is
        one editing surface for each; Plan echoes them as read-only text.
        """
        wrap = QWidget(self)
        outer = QHBoxLayout(wrap)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(s(8))

        left = QWidget(wrap)
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(s(6))

        views = QSplitter(Qt.Horizontal, left)
        views.setChildrenCollapsible(False)

        self._workspace_view = WorkspaceTargetView()
        try:
            self._workspace_view.set_safety_limits(self._controller.safety_limits)
        except Exception:
            pass
        self._workspace_view.position_clicked.connect(
            self._on_workspace_position_clicked)
        self._workspace_view.fast_travel_requested.connect(
            self._on_workspace_fast_travel_requested)
        ws_card = Card("XY Workspace", flush=True)
        ws_card.add_widget(self._workspace_view)
        views.addWidget(ws_card)

        self._xz_view = XZSideView()
        try:
            self._xz_view.set_safety_limits(self._controller.safety_limits)
        except Exception:
            pass
        self._xz_view.go_to_z_requested.connect(self._on_go_to_z_requested)
        xz_card = Card("Side View (XZ)", flush=True)
        xz_card.add_widget(self._xz_view)
        views.addWidget(xz_card)

        layout.addWidget(views, stretch=3)
        layout.addWidget(self._build_progress_card(), stretch=2)
        outer.addWidget(left, stretch=3)
        outer.addWidget(self._build_tuning_card(), stretch=1)
        return wrap

    def _build_progress_card(self) -> QWidget:
        """Where the run is, and what each finished cell actually got."""
        card = Card("Progress")
        self._run_progress = QLabel("Not running.")
        self._run_progress.setWordWrap(True)
        self._run_progress.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {sf(10)}pt;")
        card.add_widget(self._run_progress)

        self._run_log = QListWidget()
        self._run_log.setAlternatingRowColors(True)
        self._run_log.setMinimumHeight(s(90))
        self._run_log.setToolTip(
            "One line per cell, with the tuning it actually received. With "
            "mid-run edits allowed a batch is not one uniform experiment, so "
            "this is the record of which cell got what.")
        card.add_widget(self._run_log)
        return card

    def _build_tuning_card(self) -> QWidget:
        """The four knobs, live during a run.

        Nothing structural is here — reagent, bores, wells and prep stay on Plan
        — so no edit made while the machine is moving can change WHAT it does,
        only how much and for how long.
        """
        holder = QWidget(self)
        col = QVBoxLayout(holder)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(s(8))

        card = Card("Tuning")
        for label, widget in (
            ("Reagent dose", self._push_volume),
            ("Incubation", self._dwell),
            ("Pull (×)", self._pull_mult),
            ("Pull flow (fast)", self._pull_speed),
            ("Removal Z (↑ bottom)", self._removal_z),
        ):
            card.add_widget(FormRow(label, widget,
                                    help_text=widget.toolTip() or None))
        # v7.13 — sample-surface removal Z (per-target, from the fluorescence
        # scan's autofocus survey).
        card.add_widget(self._surface_z_chk)
        card.add_widget(FormRow("↑ above surface", self._surface_offset,
                                help_text=self._surface_offset.toolTip()))
        self._tuning_derived = QLabel("")
        self._tuning_derived.setWordWrap(True)
        self._tuning_derived.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        card.add_widget(self._tuning_derived)
        self._tuning_note = QLabel(
            "Changes apply to the cells still to come, and are recorded against "
            "each one.")
        self._tuning_note.setWordWrap(True)
        self._tuning_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        card.add_widget(self._tuning_note)
        col.addWidget(card)

        trial = Card("Try it")
        self._test_one_btn = QPushButton("Test one cell")
        self._test_one_btn.setCursor(Qt.PointingHandCursor)
        self._test_one_btn.setToolTip(
            "Run the next un-tried cell on its own, so the numbers can be judged "
            "on one cell before the rest are committed. The needle is prepped on "
            "the first try and reused after that.")
        self._test_one_btn.clicked.connect(self._on_test_one_cell)
        trial.add_widget(self._test_one_btn)

        self._run_rest_btn = QPushButton("Run remaining")
        self._run_rest_btn.setCursor(Qt.PointingHandCursor)
        self._run_rest_btn.setToolTip(
            "Run every cell that has not been tried yet, with the tuning above.")
        self._run_rest_btn.clicked.connect(self._on_run_remaining)
        trial.add_widget(self._run_rest_btn)

        self._trial_status = QLabel("")
        self._trial_status.setWordWrap(True)
        self._trial_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        trial.add_widget(self._trial_status)
        col.addWidget(trial)

        col.addStretch(1)
        return holder

    def _build_mosaic_point_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(8))
        self._mosaic_point_lbl = QLabel("Click the mosaic to pick a point.")
        self._mosaic_point_lbl.setWordWrap(True)
        self._mosaic_point_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._mosaic_point_lbl, stretch=1)

        self._mosaic_goto_btn = QPushButton("⤵ Go to point")
        self._mosaic_goto_btn.setEnabled(False)
        self._mosaic_goto_btn.setToolTip(
            "Retract the needle, then travel to the clicked mosaic point "
            "(absolute stage µm).")
        self._mosaic_goto_btn.clicked.connect(self._on_mosaic_goto)
        row.addWidget(self._mosaic_goto_btn)

        self._mosaic_pick_btn = QPushButton("＋ Add as removal target")
        self._mosaic_pick_btn.setEnabled(False)
        self._mosaic_pick_btn.setToolTip(
            "Add the clicked mosaic point to the removal list. A mosaic position "
            "is a SEARCH HINT — go there and click the object on the live view "
            "to confirm it before running.")
        self._mosaic_pick_btn.clicked.connect(self._on_mosaic_add_pick)
        row.addWidget(self._mosaic_pick_btn)
        return frame

    # ── Settings popout ───────────────────────────────────────────

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        self._refresh_volume_label()
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._refresh_trypsin_status()
        self._update_settings_summary()
        self._refresh_readiness()

    def _update_settings_summary(self):
        if not hasattr(self, "_settings_summary"):
            return
        try:
            push, pull = self._push_pull_uL()
            prep = "prep on" if self._prep_check.isChecked() else "prep off"
            tryp = self._program_for_role(BoreRole.PUSH_REAGENT)
            extra = (f" · dosing bore {tryp.bore_index + 1}"
                     if tryp is not None else "")
            self._settings_summary.setText(
                f"pull {pull:.3g} µL (column {push:.3g} µL) · incubation "
                f"{self._dwell.value():.0f} s · {prep}{extra}")
        except Exception:
            # A blanket `pass` let this header summary silently go STALE — still
            # showing the previous run's numbers, which is worse than showing
            # nothing. A visible "—" is recoverable; a frozen number is not.
            logger.debug("settings summary refresh failed", exc_info=True)
            self._settings_summary.setText("—")

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

    def _create_config_widgets(self) -> None:
        """Create EVERY config widget, before either surface lays one out.

        Split out of ``_build_settings_dialog`` in v7.9 so the primary fields can
        be laid out on the Setup tab (a widget has one parent) while still being
        registered with the dialog for persistence. Nothing here is parented yet.
        """
        # ── Removal & placement heights ──
        # ⚠ 0.00 mm is the needle ON THE GLASS, and at 2 decimals 0.005 could not
        # even be expressed — so the minimum is a real (small) clearance and the
        # resolution goes to microns. A saved 0.0 loads as 0.005, which is the safe
        # direction; the readiness card reports the resolved height either way.
        self._removal_z = self._dspin(
            0.005, 20.0, 0.10, " mm", 3, 0.05,
            "Needle height above the plate bottom at the cell-removal location. "
            "0 would put the tip on the glass, so the minimum is 0.005 mm.")
        self._place_z = self._dspin(
            0.0, 20.0, 0.50, " mm", 2, 0.05,
            "Needle height above the plate bottom when dispensing extracted cells.")
        # v7.13 — removal Z from the MEASURED sample surface (the fluorescence
        # mosaic's per-tile autofocus survey). Cells often sit ABOVE the well
        # bottom (e.g. on hydrogel); a plate-bottom offset then aims below
        # them. Gated at Start on the survey + the verified focus↔needle
        # datum; low-confidence targets fall back to the plate-bottom offset.
        self._surface_z_chk = QCheckBox(
            "Removal Z from measured sample surface")
        self._surface_z_chk.setChecked(False)
        self._surface_z_chk.setToolTip(
            "Per-target removal height evaluated on the sample surface "
            "measured by the fluorescence scan's autofocus (Survey tab), "
            "converted through the verified focus↔needle datum. Targets "
            "outside the surveyed region fall back to the plate-bottom "
            "offset above. Needs: a focus survey for the scanned well + the "
            "focus↔needle datum.")
        self._surface_z_chk.toggled.connect(
            lambda *_: self._update_settings_summary())
        self._surface_offset = self._dspin(
            0.0, 500.0, 10.0, " µm", 0, 5.0,
            "Extra clearance ABOVE the measured sample surface for the "
            "aspirating bore.")

        # ── Pump & reagent ──
        self._bore = QComboBox()
        self._bore.setMinimumWidth(s(110))
        self._bore.setToolTip(
            "Pump feeding the aspirating bore. Used when the per-bore table "
            "above names no pump for the aspirating bore.")
        self._bore.currentIndexChanged.connect(self._on_bore_changed)
        self._reagent_combo = QComboBox()
        self._reagent_combo.setMinimumWidth(s(180))
        self._reagent_combo.setToolTip(
            "Reagent the needle is loaded with (e.g. trypsin). Listed reagents "
            "are library inks with a reagent location (Hardware Setup → Ink).")
        self._reagent_combo.currentIndexChanged.connect(
            lambda *_: (self._refresh_reagent_status(),
                        self._update_settings_summary()))
        self._reagent_z = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom at ANY reagent well — the "
            "aspirating bore's cell-release reagent and a dosing bore's own "
            "reagent well both dip to this height.")
        self._reagent_status = QLabel("")
        self._reagent_status.setWordWrap(True)
        self._reagent_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")

        # ── Push / pull mechanics ──
        # ⚠ VOLUME-PRIMARY, in nL: the dose is the quantity, the column depth is
        # how it gets metered. `_push_depth` survives as a HIDDEN, derived mirror
        # because it is the field the executor actually uses
        # (`CellRemovalConfig.compute_release_volume_uL` prefers
        # `release_depth_mm × orifice area` whenever a needle is resolvable, and
        # only falls back to the stamped `release_volume_uL`). Keeping it means
        # `_push_pull_uL()` and `_current_config()` are untouched, which is what
        # makes this change verifiable by exact equality against the old value.
        self._push_volume = self._dspin(
            _MIN_DOSE_NL, 2_000_000.0, 3.142, " nL", 3, 0.5,
            "Reagent column the aspirating bore pushes onto the cell, and the "
            "basis for the extraction pull. Capped at what that bore holds.")
        self._push_volume.valueChanged.connect(
            lambda *_: self._sync_dose("volume"))
        # Range widened from 5.0 mm: it is now derived from a volume, and a fine
        # bore needs a long column for the same dose (a 30 µm bore holding ~9 nL
        # is 12.7 mm of column). A 5.0 mm ceiling would have clamped it silently.
        # 6 decimals so the volume → depth → volume round trip is exact enough
        # that the operator's typed dose is the dose that runs.
        self._push_depth = self._dspin(
            0.0, 500.0, 0.10, " mm", 6, 0.0,
            "Column depth this dose corresponds to through the aspirating bore "
            "— derived from the volume, not typed.")
        self._push_depth.setReadOnly(True)
        self._push_depth.setButtonSymbols(QAbstractSpinBox.NoButtons)
        #: Re-entry guard for the volume ⇄ depth mirror.
        self._dose_syncing = False
        #: ``(dose nL, orifice area mm²)`` the depth was last derived from, so a
        #: sync that changes neither cannot re-round the column. None = never.
        self._dose_last_key: tuple[float, float] | None = None
        #: True when a loaded profile predates the volume field, so its
        #: ``push_depth`` is authoritative — resolved ONCE, at the first needle
        #: sync, because at load time there is no bore area to convert with.
        self._dose_legacy_pending = False
        self._pull_mult = self._dspin(
            1.0, 20.0, 2.0, "", 2, 0.5,
            "Extraction pull volume = this multiple of the pushed volume.")
        self._pull_mult.valueChanged.connect(self._refresh_volume_label)
        # decimals=0 silently ROUNDED a typed 90.5 s; one decimal keeps what the
        # operator entered. The 3600 s ceiling is stated so a typed 5400 being
        # clamped is not a surprise.
        self._dwell = self._dspin(
            0.0, 3600.0, 60.0, " s", 1, 5.0,
            "Incubation time the reagent dwells at the cell before extraction "
            "(max 3600 s). With a dosing bore armed, the full dose→aspirate "
            "interval is this PLUS that bore's lead time.")
        self._push_speed = self._dspin(
            0.01, 50.0, 0.5, " µL/s", 2, 0.1, "Slow flow used to push reagent in.")
        self._pull_speed = self._dspin(
            0.01, 50.0, 5.0, " µL/s", 2, 0.5, "Fast flow used to pull cells up.")
        self._volume_label = QLabel("V = —")
        self._volume_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")

        # The four tuning knobs republish their snapshot on every edit, so a
        # change reaches the cells still to come without the operator pressing
        # anything — which is what "tune as we go" means.
        for w in (self._push_volume, self._dwell, self._pull_mult,
                  self._pull_speed, self._removal_z):
            w.valueChanged.connect(self._publish_tuning)

        # ── Needle prep / clean ──
        self._prep_check = QCheckBox("Prep needle (waste → oil → wash → buffer)")
        self._prep_check.setChecked(True)
        self._prep_check.toggled.connect(self._on_prep_toggled)
        self._clean_check = QCheckBox("Clean after (waste → wash → buffer)")
        self._clean_check.setChecked(True)
        self._clean_check.toggled.connect(self._on_prep_toggled)
        self._wash_after_pickup_check = QCheckBox(
            "Wash needle after reagent pickup (rinse exterior before deposit)")
        self._wash_after_pickup_check.setChecked(True)
        self._wash_after_pickup_check.setToolTip(
            "After aspirating the cell-release reagent, dip + jiggle the needle "
            "at the wash well to rinse the reagent film off its exterior before "
            "travelling to the cell — so only the metered pushed volume reaches "
            "the cells. The aspirated reagent stays in the bore.")
        self._wash_after_pickup_check.toggled.connect(self._on_prep_toggled)
        self._service_z = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom at the service wells.")
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
        self._prep_status = QLabel("")
        self._prep_status.setWordWrap(True)
        self._prep_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")

        # ── Motion & timeouts ──
        self._intra_retract = self._dspin(
            0.0, 20.0, 1.0, " mm", 2, 0.1, "Short Z retract for moves within one well.")
        self._z_timeout = self._dspin(
            1.0, 120.0, 15.0, " s", 0, 1.0, "Z move arrival timeout.")
        self._xy_timeout = self._dspin(
            1.0, 240.0, 30.0, " s", 0, 1.0, "XY move arrival timeout.")

        # ── Common — Pump (global) ──
        self._g_settle = self._dspin(0.0, 30.0, 0.0, " s", 2, 0.05)
        self._g_prime = self._dspin(0.0, 30.0, 0.25, " s", 2, 0.05)

    #: Fields laid out on the Setup tab. Registered with the settings dialog via
    #: ``register_external`` so they are saved / loaded / imported / exported
    #: exactly as before — the keys are unchanged, so an existing last-used file
    #: keeps loading.
    _PROMOTED_DEFAULTS = (
        ("removal_z", "_removal_z", 0.10),
        ("place_z", "_place_z", 0.50),
        ("bore", "_bore", "P1"),
        ("reagent", "_reagent_combo", ""),
        ("reagent_z", "_reagent_z", 0.50),
        # The dose the operator types (nL) and the depth the executor meters
        # with (mm). Both persist: `push_depth` keeps being written so an OLDER
        # build still loads this profile and runs the same column, and
        # `push_volume_nL`'s absence is how `_migrate_legacy_settings` recognises
        # a pre-volume profile.
        ("push_volume_nL", "_push_volume", 3.142),
        ("push_depth", "_push_depth", 0.10),
        ("pull_mult", "_pull_mult", 2.0),
        ("dwell", "_dwell", 60.0),
        ("push_speed", "_push_speed", 0.5),
        ("pull_speed", "_pull_speed", 5.0),
        ("prep", "_prep_check", True),
        ("clean", "_clean_check", True),
        ("wash_after_pickup", "_wash_after_pickup_check", True),
        # v7.13 — sample-surface removal Z (old profiles lack the keys →
        # defaults, mode off).
        ("surface_z", "_surface_z_chk", False),
        ("surface_offset_um", "_surface_offset", 10.0),
    )

    def _register_promoted_fields(self, dlg: WorkflowSettingsDialog) -> None:
        for key, attr, default in self._PROMOTED_DEFAULTS:
            dlg.register_external(key, getattr(self, attr), default)

    def _build_settings_dialog(self, dlg: WorkflowSettingsDialog):
        """The ⚙ popout: profiles + the advanced / Common-linked fields.

        The primary fields are NOT here — they are on the Setup tab (see
        ``_PROMOTED_DEFAULTS``). Everything that remains either needs the
        Common Print Settings override machinery (which is a dialog row) or is an
        advanced knob the operator sets once.
        """
        # ── Needle prep / clean sub-parameters (Common-linked) ──
        sec = dlg.add_section("Needle prep / clean — shared sub-parameters")
        sec.add_note(
            "The prep / clean / wash toggles are on the Plan tab. These values "
            "are shared defaults from Common Print Settings — tick Override to "
            "set a workflow-specific value.")
        sec.add_common("service_z", "Service dip Z (↑ bottom)", self._service_z, 0.50)
        sec.add_common("prep_rate", "Prep / clean flow", self._prep_rate, 1.0)
        sec.add_common("oil_needles", "Oil (needles)", self._oil_needles, 1.0)
        sec.add_common("buffer_needles", "Buffer (needles)", self._buffer_needles, 1.0)
        sec.add_common("wash_cycles", "Wash cycles", self._wash_cycles, 3)
        sec.add_common("wash_z_amp", "Wash Z jiggle", self._wash_z_amp, 0.5)
        sec.add_common("wash_xy_amp", "Wash XY jiggle", self._wash_xy_amp, 200.0)
        sec.add_common("wash_dwell", "Wash settle", self._wash_dwell, 0.3)
        sec.add("post_dispense", "Clean dispense (needles)", self._post_dispense, 1.0)

        # ── Motion & timeouts ──
        sec = dlg.add_section("Motion & timeouts (advanced)")
        sec.add("intra_retract", "Intra-well retract", self._intra_retract, 1.0)
        sec.add("z_timeout", "Z timeout", self._z_timeout, 15.0)
        sec.add("xy_timeout", "XY timeout", self._xy_timeout, 30.0)

        # ── Common — Pump (global) ──
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
                extras.append(("Cell-release reagent",
                               f"{reagent} ← {well or '(no well)'}"))
            tryp = self._trypsin_reagent()
            if tryp:
                extras.append(("Trypsin bore reagent",
                               f"{tryp} ← "
                               f"{self._trypsin_source_well() or '(no well)'}"))
        except Exception:
            pass
        return build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z, extras=extras)

    # ── Non-widget settings state (v7.9) ──────────────────────────

    def _collect_extra_state(self) -> dict:
        """The per-bore program table, for the profile.

        The table is rebuilt whenever the needle changes, so it cannot be a
        registered field (``widget_value`` handles only the four fixed widget
        types) — it would silently vanish on every restart. This is the hook
        added in v7.7 for exactly that, following the Quick Print multi-ink
        mapping precedent.
        """
        panel = self._setup_panel
        state = dict(panel.to_state()) if panel is not None else {}
        # v7.9 (post-audit): two view preferences that silently reverted on every
        # restart. Small, but the fluorescence overlay is the operator's own
        # reference for where the cells are, and re-enabling it every session is
        # exactly the kind of friction that reads as the software forgetting.
        try:
            if getattr(self, "_fluor_check", None) is not None:
                state["fluor_overlay"] = bool(self._fluor_check.isChecked())
            if getattr(self, "_tabs", None) is not None:
                state["active_tab"] = int(self._tabs.currentIndex())
        except Exception:
            logger.debug("could not collect the view state", exc_info=True)
        return state

    def _apply_extra_state(self, state) -> None:
        panel = self._setup_panel
        if panel is not None:
            panel.apply_state(state)
        if not isinstance(state, dict):
            return
        try:
            if "fluor_overlay" in state and \
                    getattr(self, "_fluor_check", None) is not None:
                # Setting it fires _on_fluor_toggled, which applies the overlay —
                # that is wanted here, unlike a silent state restore.
                self._fluor_check.setChecked(bool(state["fluor_overlay"]))
            # ⚠ Only PLAN is restored. The key predates the Plan/Run split, where
            # index 1 meant the survey viewer and now means the run monitor, so a
            # saved 1 would open a session on a monitoring surface with nothing
            # running. A session starts by planning; Start is what opens Run.
            idx = state.get("active_tab")
            if idx == self._TAB_PLAN and getattr(self, "_tabs", None) is not None:
                self._tabs.setCurrentIndex(self._TAB_PLAN)
        except Exception:
            logger.debug("could not apply the view state", exc_info=True)

    def _on_prep_toggled(self, *_):
        prep_clean = self._prep_check.isChecked() or self._clean_check.isChecked()
        wash_ap = self._wash_after_pickup_check.isChecked()
        any_service = prep_clean or wash_ap
        # Service dip Z + wash mechanics are shared by prep/clean AND the
        # after-pickup wash.
        for w in (self._service_z, self._wash_cycles, self._wash_z_amp,
                  self._wash_xy_amp, self._wash_dwell):
            w.setEnabled(any_service)
        # Oil / buffer / prep flow / post-dispense are prep/clean-only (the
        # after-pickup wash does no pump moves).
        for w in (self._prep_rate, self._oil_needles, self._buffer_needles,
                  self._post_dispense):
            w.setEnabled(prep_clean)
        self._refresh_prep_status()
        self._update_settings_summary()

    def _on_bore_changed(self, *_):
        self._refresh_reagent_combo()
        self._refresh_reagent_status()

    # ── Cell-release reagent resolution (mirror of Quick Print's ink) ──

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
        # The trypsin bore draws from the same reagent library.
        if self._setup_panel is not None:
            self._setup_panel.set_reagent_choices(reagents)

    def _selected_reagent(self) -> str | None:
        if not hasattr(self, "_reagent_combo"):
            return None
        return self._reagent_combo.currentData() or None

    def _well_for_ink(self, ink: str | None) -> str | None:
        if not ink or self._hw_config is None:
            return None
        wells = (getattr(self._hw_config, "ink_locations", {}) or {}).get(ink) or []
        # Prefer a real sub-well over a flattened rosette parent (e.g. "A2").
        return resolve_pickup_well(wells, self._plate)

    def _pos_for_well(self, well: str | None) -> tuple[float, float] | None:
        wells = self._well_positions or {}
        if well and well in wells:
            return wells[well]
        return None

    def _reagent_source_well(self) -> str | None:
        return self._well_for_ink(self._selected_reagent())

    def _reagent_source_pos(self) -> tuple[float, float] | None:
        return self._pos_for_well(self._reagent_source_well())

    def _refresh_reagent_status(self):
        if not hasattr(self, "_reagent_status"):
            return
        # With a dosing bore armed this whole ladder is moot: the aspirating bore
        # loads NOTHING. The old text promised "load ~X µL on the P2 bore", a load
        # that will not happen — and demanding a reagent here is what produced the
        # double dose in the first place.
        try:
            if self._program_for_role(BoreRole.PUSH_REAGENT) is not None:
                self._reagent_status.setText(
                    "Not needed: the dosing bore delivers the reagent, so the "
                    "aspirating bore loads nothing — it only pulls the cell up.")
                self._reagent_status.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
                return
        except Exception:
            pass
        ink = self._selected_reagent()
        if not ink:
            self._reagent_status.setText(
                "⚠ No reagent selected — pick the cell-release reagent.")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        well = self._reagent_source_well()
        pos = self._reagent_source_pos()
        if well is None:
            self._reagent_status.setText(
                f"⚠ Reagent “{ink}” has no reagent location (Hardware Setup → Ink).")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        if pos is None:
            self._reagent_status.setText(
                f"⚠ Reagent well {well} not in the calibrated plate — run Plate "
                "Location.")
            self._reagent_status.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
            return
        push, _pull = self._push_pull_uL()
        self._reagent_status.setText(
            f"✓ Reagent “{ink}” ← {well} · load ~{push:.4f} µL on the "
            f"{self._aspirate_pump_id()} bore")
        self._reagent_status.setStyleSheet(
            f"color: {COLORS['green']}; font-size: {sf(9)}pt;")

    # ── Trypsin bore (v7.9) ───────────────────────────────────────

    def _program_for_role(self, role: BoreRole):
        panel = self._setup_panel
        return panel.program_for_role(role) if panel is not None else None

    def _aspirate_pump_id(self) -> str:
        """Pump feeding the aspirating bore — the table wins, the combo backs it."""
        prog = self._program_for_role(BoreRole.ASPIRATE_TARGET)
        if prog is not None and prog.pump_id:
            return prog.pump_id
        return self._bore_id()

    def _trypsin_reagent(self) -> str:
        panel = self._setup_panel
        return panel.trypsin_reagent_name() if panel is not None else ""

    def _trypsin_source_well(self) -> str | None:
        return self._well_for_ink(self._trypsin_reagent() or None)

    def _trypsin_source_pos(self) -> tuple[float, float] | None:
        return self._pos_for_well(self._trypsin_source_well())

    def _refresh_trypsin_status(self):
        panel = self._setup_panel
        if panel is None:
            return
        prog = self._program_for_role(BoreRole.PUSH_REAGENT)
        if prog is None:
            panel.set_trypsin_status(
                "No bore is dosing — the single-bore sequence runs: the "
                "aspirating bore loads the cell-release reagent, pushes it, "
                "incubates, pulls the cell up and dispenses it.")
            return
        if not prog.pump_id:
            panel.set_trypsin_status(
                f"⚠ Bore {prog.bore_index + 1} pushes reagent but names no pump.",
                "peach")
            return
        ink = self._trypsin_reagent()
        if not ink:
            panel.set_trypsin_status(
                f"⚠ Bore {prog.bore_index + 1} ({prog.pump_id}) needs a reagent "
                "to load — choose one.", "peach")
            return
        well = self._trypsin_source_well()
        if well is None:
            panel.set_trypsin_status(
                f"⚠ “{ink}” has no reagent location (Hardware Setup → Ink).",
                "peach")
            return
        if self._trypsin_source_pos() is None:
            panel.set_trypsin_status(
                f"⚠ Reagent well {well} is not in the calibrated plate — run "
                "Plate Location.", "peach")
            return
        vol = self._trypsin_push_uL()
        # ⚠ A green ✓ next to a configuration Start REFUSES is the exact defect
        # this replaces: `_trypsin_push_uL` swallows every failure into 0.0, and
        # the status painted green regardless. Kept as a defensive guard even
        # though the dose field's own minimum now makes a zero unreachable from
        # the table — a legacy profile can still arrive with one.
        if vol <= 0:
            panel.set_trypsin_status(
                f"⚠ Bore {prog.bore_index + 1}'s dose volume resolves to 0 µL — "
                f"set an explicit volume, or a dose depth with that bore's "
                f"geometry on Hardware Setup → Needle.", "peach")
            return
        # A pump can displace a volume through a bore of unknown bore — so the
        # dose is deliverable — but nothing can then check it against what the
        # bore HOLDS or against that bore's own flow ceiling. Say so instead of
        # ticking it green, which is what the retired depth-derived zero was
        # really telling the operator.
        if self._bore_orifice_area_mm2(prog.bore_index) <= 0:
            panel.set_trypsin_status(
                f"⚠ Bore {prog.bore_index + 1}'s geometry is not entered, so a "
                f"{_fmt_uL(vol)} dose cannot be checked against what the bore "
                f"holds or against its flow ceiling — set its inner Ø on "
                f"Hardware Setup → Needle.", "peach")
            return
        # Show the rate that will ACTUALLY run: the per-bore ceiling clamps it
        # silently, and the operator's lead-time arithmetic depends on it.
        requested = float(prog.rate_uL_s or 0.0)
        effective = panel.effective_dose_rate(prog.pump_id, requested)
        if effective is None:
            effective = requested
        rate_txt = f"{effective:.2f} µL/s"
        if effective < requested - 1e-9:
            rate_txt += f" (auto-limited from {requested:.2f})"
        secs = (vol / effective) if effective > 0 else 0.0
        panel.set_trypsin_status(
            f"✓ Bore {prog.bore_index + 1} ({prog.pump_id}) ← “{ink}” at {well} "
            f"· dose {_fmt_uL(vol)} @ {rate_txt} (≈{secs:.1f} s) · lead "
            f"{prog.lead_time_s:.1f} s + incubation "
            f"{float(self._dwell.value()):.0f} s", "green")

    def _bore_orifice_area_mm2(self, bore_index: int) -> float:
        """One bore's orifice area (mm²), or 0.0 when it cannot be resolved."""
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        if needle is None:
            return 0.0
        try:
            return max(0.0, float(needle_orifice_area_mm2(
                needle_bore_at(needle, int(bore_index)))))
        except (TypeError, ValueError, AttributeError):
            return 0.0

    def _trypsin_push_uL(self) -> float:
        """The resolved trypsin push volume — the config's own arithmetic.

        Asking ``CellRemovalConfig`` rather than recomputing it here is what keeps
        the readout and the executed move from ever disagreeing (the v7.7 lesson
        where a warning and the commanded flow read two different bore areas).
        """
        cfg = self._current_config()
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        try:
            return float(cfg.compute_trypsin_volume_uL(needle))
        except Exception:
            return 0.0

    # ── Prep / clean service-well resolution (inherit from HW setup) ──

    def _refresh_prep_status(self):
        if not hasattr(self, "_prep_status"):
            return
        if not (self._prep_check.isChecked() or self._clean_check.isChecked()):
            # Prep + clean off. The after-pickup wash still needs the wash well.
            if self._wash_after_pickup_check.isChecked():
                positions, _missing = resolve_service_positions(
                    self._hw_config, self._well_positions, self._plate)
                names = service_well_names(self._hw_config, self._plate)
                if "wash" not in positions:
                    self._prep_status.setText(
                        "⚠ Wash-after-pickup needs a wash well assigned + "
                        "calibrated (Hardware Setup → Ink → Reagent Locations).")
                    self._prep_status.setStyleSheet(
                        f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
                    return
                if self._plate_offset_to_zref(
                        float(self._service_z.value())) is None:
                    self._prep_status.setText(
                        "⚠ Plate bottom Z not calibrated — needed for the wash "
                        "dip Z.")
                    self._prep_status.setStyleSheet(
                        f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
                    return
                self._prep_status.setText(
                    f"Prep + clean off — wash after reagent pickup only "
                    f"(wash={names.get('wash', '?')}).")
                self._prep_status.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
                return
            self._prep_status.setText("Prep + clean disabled — removal only.")
            self._prep_status.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            return
        needle_uL = needle_volume_uL(self._hw_config)
        _positions, missing = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
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
        if self._plate_offset_to_zref(float(self._service_z.value())) is None:
            self._prep_status.setText(
                "⚠ Plate bottom Z not calibrated — needed for the service dip Z.")
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

        self._start_btn = QPushButton("Start cell removal")
        self._start_btn.clicked.connect(self._on_start)
        row.addWidget(self._start_btn)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._on_abort)
        row.addWidget(self._abort_btn)

        # Deliberately MANUAL and deliberately not on the abort path: an abort
        # already refuses new pump moves (that is what keeps Abort responsive
        # during a long drain), and adding motion to the abort path — often
        # entered because of a pump/board fault — is the wrong trade. This button
        # is the recovery, taken when the operator decides the machine is safe.
        self._clean_btn = QPushButton("Clean needle now")
        self._clean_btn.setEnabled(False)
        self._clean_btn.setToolTip(
            "Run the post-clean cycle (waste → wash → reload buffer) on every "
            "bore this run used. Enabled after a run stops with a bore still "
            "loaded — an un-cleaned bore carries residual reagent into the next "
            "run.")
        self._clean_btn.clicked.connect(self._on_clean_now)
        row.addWidget(self._clean_btn)

        row.addStretch(1)

        self._status = QLabel("Idle.")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)

        return frame

    # ── Required by MainWindow ────────────────────────────────────

    def get_page_title(self) -> str:
        return "Cell Targeting & Removal"

    def get_sub_page_title(self) -> str:
        return "Cell Targeting & Removal"

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
        # The embedded scan page keeps its own settings reference (its scan knobs
        # read it). The spheroid host omits this forward; the omission is a bug,
        # not a convention, so it is not copied here.
        self._forward_to_scan_page("set_settings", settings)

    # ── Live camera (host-owned) ──────────────────────────────────

    def _camera_slot(self) -> int:
        """The microscope slot, resolved ONCE by the picker.

        Deliberately delegated rather than re-derived: a third copy of
        ``camera_for_role(MICROSCOPE)`` in this file could start one slot while
        the picker projected clicks through another.
        """
        picker = getattr(self, "_picker", None)
        try:
            if picker is not None:
                return int(picker.cam_idx)
        except Exception:
            pass
        return 0

    def _start_camera(self) -> None:
        mgr = self._camera_manager
        if mgr is None:
            return
        cam_idx = self._camera_slot()
        try:
            if not mgr.is_running(cam_idx):
                mgr.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as exc:
            logger.debug("Cell targeting camera start failed: %s", exc)

    def _stop_camera(self) -> None:
        mgr = self._camera_manager
        if mgr is None or not self._camera_started_by_us:
            return
        # Never pull the feed out from under a running scan — the scan worker
        # samples frames, and `_stage_busy()` already treats a scan as busy.
        try:
            if self._scan_page is not None and self._scan_page.is_scanning():
                return
        except Exception:
            pass
        try:
            mgr.stop(self._camera_slot())
        except Exception as exc:
            logger.debug("Cell targeting camera stop failed: %s", exc)
        finally:
            self._camera_started_by_us = False

    def showEvent(self, event):
        self._start_camera()
        # The measured bore offsets are written onto the live needle by the
        # Calibration page on each hardware-config push, which can land after
        # this page built its table — re-read them on the way in.
        if self._setup_panel is not None:
            try:
                self._setup_panel.refresh_offsets()
            except Exception as exc:
                logger.debug("bore offset refresh failed: %s", exc)
            # Register the panel's FormRows with the Help toggle. MUST be here,
            # not in __init__: at construction the panel is not yet in the window,
            # so the parent walk finds no MainWindow and every help_text the panel
            # so carefully wires up stays permanently invisible.
            try:
                self._setup_panel.register_help_rows()
                self._connect_help_mode()
            except Exception as exc:
                logger.debug("help-row registration failed: %s", exc)
        self._refresh_readiness()
        super().showEvent(event)

    def _connect_help_mode(self) -> None:
        """Subscribe the bore-table legend to the global Help toggle, once."""
        if getattr(self, "_help_connected", False):
            return
        w = self.parent()
        seen = 0
        while w is not None and seen < 12:
            sig = getattr(w, "help_mode_changed", None)
            if sig is not None and hasattr(sig, "connect"):
                sig.connect(self._setup_panel.set_help_mode)
                self._help_connected = True
                return
            w = w.parent()
            seen += 1

    def hideEvent(self, event):
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        # PERSIST ON THE WAY OUT. ``WorkflowSettingsDialog`` auto-saves last-used
        # from its OWN hideEvent/closeEvent, which was sufficient while the popout
        # was the only editing surface — touching a setting implied opening it.
        # v7.9 promoted 13 fields onto the Setup tab and put the bore program
        # table there, so the normal workflow never opens the popout and nothing
        # was written at all: a session's whole Setup tab (and its bore roles)
        # came back at defaults. Same moment the dialog would have saved, so the
        # semantics are unchanged; ``save_last`` is idempotent and swallows its
        # own errors, so the double call after the hide above is harmless.
        try:
            self._settings_dialog.save_last()
        except Exception as exc:
            logger.debug("save_last on hide failed: %s", exc)
        # ⚠ The embedded scan page is deliberately NOT hidden here. It is a CHILD
        # WIDGET, and an explicit ``hide()`` STICKS: Qt will not re-show an
        # explicitly-hidden child when its parent is shown again, so the whole
        # Well Survey / Viewer tab came back BLANK on every return to this page —
        # which would make decision D4's live embedded instance a one-visit
        # feature. Qt already delivers a hide event to a *visible* child when its
        # parent hides (verified offscreen), and that event is what stops the
        # scan page's camera and tucks its own modeless dialog away; the child is
        # then re-shown automatically. Nothing extra is needed.
        # (``SpheroidPickupWorkflowPage`` carries the same explicit hide and has
        # the same symptom — fixing it there is its own change.)
        self._stop_camera()
        super().hideEvent(event)

    # ── Common Print Settings hook ────────────────────────────────

    def set_common_print_settings(self, common):
        # The embedded fluorescence page defines no set_common_print_settings, so
        # there is deliberately no hasattr-guarded forward here (the spheroid host
        # has one and it has always been dead code).
        if getattr(self, "_settings_dialog", None) is not None:
            self._settings_dialog.set_common(common)

    # ── hw_config hook ────────────────────────────────────────────

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        # Refresh bore options from pump ids
        pump_ids: list[str] = []
        if hw_config is not None and hasattr(hw_config, "pumps"):
            for pid, pcfg in hw_config.pumps.items():
                enabled = getattr(pcfg, "enabled", True)
                configured = getattr(pcfg, "is_configured", True)
                if enabled and configured:
                    pump_ids.append(pid)
        self._bore.blockSignals(True)
        previous = self._bore.currentText()
        self._bore.clear()
        for pid in pump_ids:
            self._bore.addItem(pid)
        if self._bore.count() == 0:
            self._bore.addItem("P1")
        idx = self._bore.findText(previous)
        if idx >= 0:
            self._bore.setCurrentIndex(idx)
        self._bore.blockSignals(False)

        # Push to the shared live picker + workspace/XZ needle size
        self._picker.set_hardware_config(hw_config)
        needle = getattr(hw_config, "needle", None) if hw_config else None
        try:
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

        # v7.9: the per-bore table IS the assembly, so it rebuilds here.
        if self._setup_panel is not None:
            self._setup_panel.set_pump_ids(pump_ids)
            self._setup_panel.set_needle(needle)
            # Per-pump flow ceilings, so a clamped dose flow is SHOWN rather than
            # only logged. The panel never reaches for the controller itself.
            self._setup_panel.set_flow_ceilings(self._pump_flow_ceilings())

        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)
        self._refresh_reagent_combo()
        # The bore + reagent combos just (re)populated — apply any saved
        # selections that were pending because the combos were empty at load.
        try:
            self._settings_dialog.resolve_pending()
        except Exception:
            pass
        self._forward_to_scan_page("set_hardware_config", hw_config)
        # The bore geometry is only knowable HERE, so this is where a pre-volume
        # profile's saved column depth becomes a dose (once), and where the dose
        # cap and the derived depth are re-resolved against the mounted assembly.
        self._sync_dose_to_needle()
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._refresh_trypsin_status()
        self._refresh_volume_label()
        self._refresh_readiness()
        self._update_settings_summary()

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
        self._refresh_target_overlays()
        if getattr(self, "_fluor_check", None) is not None and self._fluor_check.isChecked():
            load_plate_fluor_overlay(
                self._workspace_view, plate_key_of(self._hw_config), visible=True)
        # Reagent + service-well resolution depends on the calibrated positions.
        self._refresh_reagent_status()
        self._refresh_prep_status()
        self._refresh_trypsin_status()
        # The viewer tab's scan + well geometry come from the same calibration.
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
        # Plate-bottom datum affects the resolvable reagent/service dip Z.
        self._refresh_reagent_status()
        self._refresh_prep_status()
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
        self._refresh_readiness()
        self._refresh_target_overlays()

    def _refresh_target_overlays(self):
        """Convert pick (removal) + place targets from stage-frame µm to zero-ref
        µm and push them into the workspace view's overlay layer."""
        try:
            zero = self._controller.zero_position
        except Exception:
            zero = {"x": 0.0, "y": 0.0}

        picks_zr = [
            (t.x_um - zero["x"], t.y_um - zero["y"], t.target_id)
            for t in self._picker.picks()
        ]
        places_zr = [
            (t.x_um - zero["x"], t.y_um - zero["y"], t.target_id)
            for t in self._picker.places()
        ]
        self._workspace_view.set_pick_targets(picks_zr)
        self._workspace_view.set_place_targets(places_zr)

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

        # The live frame floating over the mosaic rides this existing tick — no
        # second timer, and the poller cache is already what everything else here
        # reads (it is back-filled during a print, so it stays live).
        if self._tabs.currentIndex() == self._TAB_PLAN:
            try:
                self._refresh_live_frame_overlay()
            except Exception:
                logger.debug("live frame overlay failed", exc_info=True)

    # ── Workspace + XZ click handlers (mirror of JogControlPage) ──

    def _stage_busy(self, quiet: bool = False) -> bool:
        """True (+ shows a hint) when something else is already driving the stage.

        Consulted by EVERY entry point that can move the stage — Start, both
        workspace clicks, and the mosaic Go-to. Two drivers on one serial channel
        is bad enough, but the embedded mosaic scan worker also toggles
        ``suspend_position_poller``, which is NOT refcounted: whichever finishes
        first re-enables the poller underneath the other.

        ``quiet=True`` for BUTTON-STATE queries: those run on a repaint, and
        overwriting the status line from one would stamp "Busy running" over the
        message that actually explains what is happening.
        """
        t = getattr(self, "_exec_thread", None)
        if t is not None and t.is_alive():
            if not quiet:
                self._status.setText(
                    "Busy running — abort first to move manually.")
            return True
        page = getattr(self, "_scan_page", None)
        if page is not None:
            try:
                scanning = bool(page.is_scanning())
            except Exception:
                scanning = False
            if scanning:
                if not quiet:
                    self._status.setText(
                        "A mosaic scan is running — wait for it or abort it "
                        "first.")
                return True
        return False

    # Kept as an alias: the old name reads better at the two workspace-click
    # sites and is what the existing tests reference.
    def _travel_blocked_by_run(self) -> bool:
        return self._stage_busy()

    def _travel_to_absolute(self, x_um_abs: float, y_um_abs: float) -> bool:
        """Retract to safe Z, then travel to an ABSOLUTE stage µm point.

        Deliberately separate from :meth:`_on_workspace_position_clicked`, which
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
        # Re-follow: sending the camera somewhere is the reason the operator
        # panned away from it, so the view should be back on it when it arrives.
        # Armed here rather than on arrival because the follow re-centres from the
        # position tick — it picks the camera up as it moves, not just at the end.
        try:
            if getattr(self, "_follow_btn", None) is not None:
                self._follow_btn.setChecked(True)
        except Exception:
            pass
        self._travel_worker.start(
            self._controller, x_um_abs, y_um_abs,
            safe_z_mm=self._safe_z, target_z_mm=None)
        return True

    def _on_workspace_position_clicked(
        self, x_um_zr: float, y_um_zr: float
    ) -> None:
        """Click-to-travel from the XY workspace (zero-ref µm)."""
        if not getattr(self._controller, "is_xy_connected", False):
            return
        if self._stage_busy():
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
        if self._stage_busy():
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
        # The 4th motion entry point on this page, and the one that was still
        # unguarded: the embedded mosaic scan drives Z through safe_travel_to, so
        # commanding an absolute Z underneath it puts two drivers on the serial
        # channel (and on the non-refcounted poller suspend).
        if self._stage_busy():
            return
        try:
            self._controller.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("Go-to-Z failed: %s", exc)

    # ── Mosaic point (viewer tab) ─────────────────────────────────

    def _mosaic_context(self):
        page = self._scan_page
        if page is None:
            return None
        try:
            return page.mosaic_context(None)
        except Exception as exc:
            logger.debug("mosaic_context failed: %s", exc)
            return None

    def _mosaic_can_command_motion(self) -> bool:
        """True only when this mosaic's pixel→stage mapping is trustworthy.

        No context ⇒ False, matching ``SpheroidSurveyPanel.can_command_motion``:
        an unrecorded registration shift is bounded only by ~20 % of the FOV
        width, which at 10× is larger than a cell, and cell removal descends to
        ~0.1 mm off the glass.
        """
        ctx = self._mosaic_context()
        return bool(ctx and ctx.get("has_shift") and ctx.get("mosaic_scale"))

    def _mosaic_shift_bound_text(self, ctx) -> str:
        """The mapping error bound in the operator's own units, when derivable."""
        try:
            eff = float((ctx or {}).get("um_per_px") or 0.0)
            image = (ctx or {}).get("image")
            if eff > 0 and image is not None:
                return f" (up to about ±{0.2 * float(image.shape[1]) * eff:.0f} µm)"
        except Exception:
            pass
        return ""

    def _on_mosaic_ready(self, well: str) -> None:
        """A different mosaic is loaded — drop the picked point."""
        self._mosaic_point_um = None
        ctx = self._mosaic_context()
        if ctx is None:
            self._mosaic_point_lbl.setText(
                f"No mosaic stored for {well or 'this well'} — scan it first.")
        else:
            self._mosaic_point_lbl.setText(
                f"Mosaic loaded for {ctx.get('well') or well} — click it to pick "
                f"a point.")
        self._mosaic_point_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self._refresh_mosaic_point_buttons()
        self._refresh_well_view_note()
        self._refresh_readiness()

    def _on_mosaic_scene_clicked(self, point) -> None:
        """A click on the mosaic in POINT mode: record it, and travel there.

        Travelling on the click is what the operator asked for ("click anywhere on
        the mosaic and the stage moves to that point"). It stays gated by exactly
        the same two guards the Go-to button uses — ``_stage_busy()`` and a mosaic
        whose registration is recorded — so making it one gesture instead of two
        removes a click, not a safeguard. Hand mode does not reach here at all.
        """
        ctx = self._mosaic_context()
        if ctx is None:
            self._mosaic_point_um = None
            self._mosaic_point_lbl.setText(
                "No mosaic stored for the selected well — scan it first.")
            self._refresh_mosaic_point_buttons()
            return
        scale = float(ctx.get("mosaic_scale") or 0.0)
        if scale <= 0:
            self._mosaic_point_um = None
            self._mosaic_point_lbl.setText(
                "This mosaic has no usable px/µm scale — re-scan the well.")
            self._refresh_mosaic_point_buttons()
            return
        from SupportClasses.SpheroidDetector import back_project_px
        try:
            x_um, y_um = back_project_px(
                (float(point.x()), float(point.y())),
                ctx["extent_um"], scale, ctx.get("shift_um") or (0.0, 0.0))
        except Exception as exc:
            logger.debug("back-projection failed: %s", exc)
            return
        self._mosaic_point_um = (x_um, y_um)
        text = f"Mosaic point: {x_um:.0f}, {y_um:.0f} µm (stage)"
        if not ctx.get("has_shift"):
            text += ("  ⚠ this saved mosaic predates registration-shift "
                     "recording, so its pixel→stage mapping may be off"
                     + self._mosaic_shift_bound_text(ctx)
                     + " — re-scan the well before travelling here.")
            self._mosaic_point_lbl.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        else:
            self._mosaic_point_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self._mosaic_point_lbl.setText(text)
        self._refresh_mosaic_point_buttons()
        # Then go there. Same guards as the button; re-following on arrival is
        # handled by `_travel_to_absolute`, because sending the camera somewhere
        # is the reason the operator panned away from it.
        if self._mosaic_can_command_motion():
            self._on_mosaic_goto()

    def _refresh_well_view_note(self) -> None:
        """Say when the mosaic's own registration makes the overlay unreliable.

        With the live feed composited on top, a mosaic with no recorded shift is
        now VISIBLY misaligned — which is better than being told about it. The
        note names the bound; ``_mosaic_can_command_motion`` still blocks Start.
        """
        lbl = getattr(self, "_well_view_note", None)
        if lbl is None:
            return
        ctx = self._mosaic_context()
        if not ctx:
            lbl.setText("")
            return
        if not ctx.get("has_shift"):
            lbl.setText("⚠ this mosaic has no recorded registration"
                        + self._mosaic_shift_bound_text(ctx)
                        + " — the live frame will not line up with it. Re-scan.")
        elif ctx.get("scale_warning"):
            lbl.setText("⚠ " + str(ctx["scale_warning"]))
        else:
            lbl.setText("")

    def _refresh_mosaic_point_buttons(self) -> None:
        ok = (self._mosaic_point_um is not None
              and self._mosaic_can_command_motion())
        self._mosaic_goto_btn.setEnabled(ok)
        self._mosaic_pick_btn.setEnabled(ok)

    def _on_mosaic_goto(self) -> None:
        """Travel to the clicked mosaic point (ABSOLUTE stage µm)."""
        pt = self._mosaic_point_um
        if pt is None or not self._mosaic_can_command_motion():
            return
        if self._travel_to_absolute(float(pt[0]), float(pt[1])):
            self._status.setText(
                "Travelling… then click the object on the live view to confirm "
                "its position.")

    def _on_mosaic_add_pick(self) -> None:
        """Add the clicked mosaic point to the removal list as a search hint."""
        pt = self._mosaic_point_um
        if pt is None or not self._mosaic_can_command_motion():
            return
        self._picker.add_pick(float(pt[0]), float(pt[1]),
                              provenance=PROV_MOSAIC)
        self._status.setText(
            "Added a removal target from the mosaic. Its position is a search "
            "hint — go there and click the object on the live view to confirm "
            "it, then add a placement for it.")

    # ── readiness (v7.9) ──────────────────────────────────────────

    def _readiness_context(self):
        """Resolve everything the pure evaluator needs.

        ALL the getattr-safe / mock-tolerant reads live here — that is the
        division of labour ``PrintReadiness`` established: the evaluator is pure
        and fully unit-testable, and this method is the only place that has to
        cope with a half-built page or a partially-stubbed controller.
        """
        from SupportClasses.CellRemovalReadiness import (
            BoreView, CellRemovalContext)

        ctx = CellRemovalContext()
        ctrl = self._controller
        ctx.xy_connected = bool(getattr(ctrl, "is_xy_connected", False))
        ctx.zp_connected = bool(getattr(ctrl, "is_zp_connected", False))
        try:
            ctx.stage_busy_reason = self._stage_busy() or ""
        except Exception:
            ctx.stage_busy_reason = ""

        needle = getattr(self._hw_config, "needle", None) \
            if self._hw_config is not None else None
        ctx.needle_configured = needle is not None
        panel = self._setup_panel
        programs = []
        try:
            programs = list(panel.programs()) if panel is not None else []
        except Exception:
            logger.debug("readiness: programs() failed", exc_info=True)
        ctx.bore_count = len(programs)

        declared = {}
        try:
            if needle is not None:
                for k, b in enumerate(needle.bores_resolved()):
                    pid = getattr(b, "pump_id", None)
                    declared[k] = str(pid).strip().upper() if pid else None
        except Exception:
            declared = {}

        ceilings = {}
        try:
            limits = getattr(ctrl, "safety_limits", None)
            getter = getattr(limits, "get_max_flow_rate", None)
            if callable(getter):
                for pid in ("P1", "P2", "P3"):
                    v = getter(pid)
                    # A bare mock's __float__ is 1.0 and would fabricate a limit.
                    if isinstance(v, (int, float)) and not isinstance(v, bool):
                        ceilings[pid] = float(v)
        except Exception:
            ceilings = {}

        known_types = set()
        try:
            known_types = {t.id for t in (panel.target_types() or [])} \
                if panel is not None else set()
        except Exception:
            known_types = set()

        for p in programs:
            k = int(getattr(p, "bore_index", 0) or 0)
            pid = getattr(p, "pump_id", None)
            pid = str(pid).strip().upper() if pid else None
            measured = None
            off = (0.0, 0.0)
            if needle is not None:
                try:
                    off = needle_bore_offset_um(needle, k)
                    measured = (k == 0) or bool(abs(off[0]) > 1e-6
                                                or abs(off[1]) > 1e-6)
                except Exception:
                    measured = None
            tt_id = str(getattr(p, "target_type_id", "") or "")
            ctx.bores.append(BoreView(
                index=k,
                role=str(getattr(getattr(p, "role", None), "value", "idle")),
                pump_id=pid,
                declared_pump=declared.get(k),
                offset_measured=measured,
                offset_um=off,
                internal_volume_uL=self._bore_internal_volume_uL(needle, k),
                dose_volume_uL=(self._trypsin_push_uL()
                                if str(getattr(getattr(p, "role", None), "value",
                                               "")) == "push_reagent" else None),
                dose_rate_uL_s=_safe_float(getattr(p, "rate_uL_s", None)),
                flow_ceiling_uL_s=ceilings.get(pid),
                target_type_id=tt_id,
                target_type_missing=bool(tt_id and known_types
                                         and tt_id not in known_types),
            ))

        try:
            ctx.n_picks = len(self._picker.picks())
            ctx.n_places = len(self._picker.places())
        except Exception:
            pass
        try:
            targets = list(self._picker.picks()) + list(self._picker.places())
            unshifted = [t for t in targets
                         if self._picker.provenance(t.target_id) == PROV_MOSAIC]
            if unshifted and not self._mosaic_can_command_motion():
                ctx.n_unshifted_mosaic = len(unshifted)
                ctx.mosaic_error_bound_um = self._mosaic_shift_bound_um()
        except Exception:
            logger.debug("readiness: mosaic provenance failed", exc_info=True)

        cfg = None
        try:
            cfg = self._current_config()
        except Exception:
            logger.debug("readiness: _current_config failed", exc_info=True)
        if cfg is not None:
            ctx.dosing_enabled = bool(cfg.trypsin_enabled)
            ctx.column_volume_uL = _safe_float(
                cfg.compute_release_volume_uL(needle))
            ctx.pull_volume_uL = _safe_float(
                cfg.compute_extract_volume_uL(needle))
            ctx.extract_multiplier = _safe_float(cfg.extract_multiplier)
            ctx.lead_time_s = _safe_float(cfg.trypsin_lead_time_s)
            ctx.incubation_s = _safe_float(cfg.dwell_time_s)
            ctx.dose_volume_uL = (self._trypsin_push_uL()
                                  if cfg.trypsin_enabled else None)
            try:
                r = self._prep_pump_refusal(cfg)
                if r:
                    ctx.unconfigured_prep_pumps = [
                        e.get("pump_id") or "(unassigned)"
                        for e in cfg.active_bores()]
            except Exception:
                pass

        if panel is not None:
            try:
                name = panel.trypsin_reagent_name()
                ctx.dosing_reagent = name or None
                if name:
                    ctx.dosing_well_calibrated = \
                        self._reagent_well_is_calibrated(name)
            except Exception:
                pass
        try:
            name = self._selected_reagent()
            ctx.release_reagent = name or None
            if name:
                ctx.release_well_calibrated = \
                    self._reagent_well_is_calibrated(name)
        except Exception:
            pass

        ctx.prep_enabled = bool(self._prep_check.isChecked()) \
            if hasattr(self, "_prep_check") else False
        ctx.clean_enabled = bool(self._clean_check.isChecked()) \
            if hasattr(self, "_clean_check") else False
        if ctx.prep_enabled or ctx.clean_enabled:
            try:
                _pos, missing = resolve_service_positions(
                    self._hw_config, self._well_positions, self._plate)
                ctx.missing_service_wells = list(missing or [])
            except Exception:
                pass

        ctx.safe_z_mm = _safe_float(self._safe_z)
        ctx.plate_bottom_calibrated = \
            self._plate_offset_to_zref(0.0) is not None
        if cfg is not None:
            ctx.removal_clearance_mm = _safe_float(cfg.removal_z_offset_mm)
            ctx.place_clearance_mm = _safe_float(cfg.place_z_offset_mm)
        return ctx

    def _pump_flow_ceilings(self) -> dict:
        """pump id → its own flow ceiling (µL/s), for the panel's clamp warning.

        Reads permissively and STRICTLY: a bare ``MagicMock``'s ``__float__`` is
        1.0, which would fabricate a 1 µL/s limit for every pump and produce a
        false "exceeds the ceiling" warning on every partial test page. An absent
        or unreadable ceiling is simply omitted — unknown data must not warn.
        """
        out: dict[str, float] = {}
        try:
            limits = getattr(self._controller, "safety_limits", None)
            getter = getattr(limits, "get_max_flow_rate", None)
            if not callable(getter):
                return out
            for pid in ("P1", "P2", "P3"):
                v = getter(pid)
                if isinstance(v, bool) or not isinstance(v, (int, float)):
                    continue
                if math.isfinite(float(v)) and float(v) > 0:
                    out[pid] = float(v)
        except Exception:
            logger.debug("could not resolve pump flow ceilings", exc_info=True)
        return out

    def _mosaic_shift_bound_um(self):
        """The mosaic pixel→stage error bound in µm, or None when underivable.

        Numeric twin of :meth:`_mosaic_shift_bound_text` (same derivation: 20 % of
        the FOV width, the documented bound on an unrecorded registration shift)
        so the readiness model gets a number rather than a formatted string.
        """
        try:
            ctx = self._mosaic_context()
            eff = float((ctx or {}).get("um_per_px") or 0.0)
            image = (ctx or {}).get("image")
            if eff > 0 and image is not None:
                return 0.2 * float(image.shape[1]) * eff
        except Exception:
            pass
        return None

    def _reagent_well_is_calibrated(self, ink: str | None):
        """True/False/None — whether this reagent's well has a taught position."""
        if not ink:
            return None
        well = self._well_for_ink(ink)
        if not well:
            return False
        return self._pos_for_well(well) is not None

    def _bore_internal_volume_uL(self, needle, bore_index: int):
        if needle is None:
            return None
        try:
            from SupportClasses.PhysicalModels import (
                needle_bore_internal_volume_uL)
            return _safe_float(needle_bore_internal_volume_uL(needle, bore_index))
        except Exception:
            return None

    #: Tab indices. Named because several handlers switch surfaces and a bare
    #: literal is how a reorder silently sends the operator to the wrong one.
    _TAB_PLAN = 0
    _TAB_RUN = 1

    def _goto_targets_tab(self):
        """Bring the well into view — where the cells are chosen."""
        try:
            self._tabs.setCurrentIndex(self._TAB_PLAN)
            # The picker is the surface that actually confirms a target, and on a
            # narrow window it can be scrolled out of the Plan tab's left column.
            self._picker.setFocus(Qt.FocusReason.OtherFocusReason)
        except Exception:
            logger.debug("could not switch to the Plan tab", exc_info=True)

    def _refresh_readiness(self):
        """Evaluate once, then paint the card, the status line and the button.

        ONE evaluation feeds all three surfaces, which is what makes it
        impossible for the Setup tab to show a green tick beside something Start
        refuses — the defect this replaces.
        """
        from SupportClasses.CellRemovalReadiness import evaluate
        try:
            self._readiness = evaluate(self._readiness_context())
        except Exception:
            logger.debug("readiness evaluation failed", exc_info=True)
            self._readiness = None
            return
        if getattr(self, "_ready_list", None) is not None:
            try:
                self._ready_list.set_readiness(self._readiness)
            except Exception:
                logger.debug("readiness render failed", exc_info=True)
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        if not running and hasattr(self, "_status"):
            self._status.setText(self._readiness.headline())
        self._update_button_state()

    # ── config helpers ────────────────────────────────────────────

    def _aspirate_bore_index(self) -> int:
        """Which bore of the assembly pulls the cell up (0 = the datum bore)."""
        prog = self._program_for_role(BoreRole.ASPIRATE_TARGET)
        try:
            return int(prog.bore_index) if prog is not None else 0
        except (TypeError, ValueError):
            return 0

    def _needle_area_mm2(self) -> float:
        """Orifice area (mm²) of the bore that ASPIRATES.

        It has to be the SAME bore ``CellRemovalConfig.compute_release_volume_uL``
        resolves, because that is the passage the executor meters the reagent
        column through. Reading the flat ``cross_section_area_mm2`` — which by
        v7.6's fail-safe rule resolves BORE 0 — made the readout, the
        ``release_volume_uL`` backstop and the Start gate disagree with the
        executed move by the bore-area ratio as soon as a bore other than the
        datum aspirated: measured **4.00×** on a 200 µm / 100 µm backpack. That is
        the v7.7 failure where a warning and the commanded flow read two
        different bore areas, and the reason ``_trypsin_push_uL`` asks the config.

        Falls back to the flat value for any needle-like stub that cannot resolve
        a bore, so a single-bore assembly is bit-identical (verified).
        """
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        if needle is None:
            return 0.0
        try:
            area = float(needle_orifice_area_mm2(
                needle_bore_at(needle, self._aspirate_bore_index())))
            if area > 0:
                return area
        except (TypeError, ValueError, AttributeError):
            pass
        try:
            return float(getattr(needle, "cross_section_area_mm2", 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _push_pull_uL(self) -> tuple[float, float]:
        """(push, pull) volume in µL from the needle bore area × push depth.

        UNCHANGED by the move to volume entry — deliberately. The nL field is a
        transducer onto ``_push_depth``, so this arithmetic, ``_current_config``
        and the executor all still read the one number they always did, and the
        change is verifiable by exact equality against the previous value.
        """
        push = self._needle_area_mm2() * float(self._push_depth.value())
        pull = push * float(self._pull_mult.value())
        return push, pull

    # ── dose ⇄ derived column depth ───────────────────────────────

    def _sync_dose(self, source: str) -> None:
        """Mirror the typed dose onto the hidden column depth, or vice versa.

        ``source="volume"`` — the normal direction: the operator's dose is held
        and the depth is re-derived. This is what makes a bore change keep the
        dose rather than keep the geometry.

        ``source="depth"`` — used once, to migrate a pre-volume profile whose
        ``push_depth`` is the authoritative record of what it ran at.
        """
        if self._dose_syncing:
            return
        area = self._needle_area_mm2()
        if area <= 0:
            # ⚠ Leave BOTH fields alone. An earlier cut zeroed the depth here to
            # display a "bore Ø unknown" placeholder — which DESTROYED the saved
            # column depth every time a settings load or a combo repopulation ran
            # before the needle arrived (a real ordering that made the operator's
            # own profile run a 3125× smaller column). The depth is still exactly
            # what the executor will meter; only the resulting VOLUME is unknown,
            # and `_volume_label` already says so.
            self._push_depth.setSpecialValueText("— (bore Ø unknown)")
            return
        self._push_depth.setSpecialValueText("")
        self._dose_syncing = True
        try:
            if source == "depth":
                nL = area * float(self._push_depth.value()) * _NL_PER_UL
                self._push_volume.blockSignals(True)
                self._push_volume.setValue(
                    max(self._push_volume.minimum(),
                        min(nL, self._push_volume.maximum())))
                self._push_volume.blockSignals(False)
                self._dose_last_key = (float(self._push_volume.value()), area)
            else:
                nL = float(self._push_volume.value())
                # A sync where NEITHER the dose nor the bore changed must not
                # re-round the depth: the nL field holds 3 decimals, so a
                # volume→depth→volume round trip shifts the column by ~0.01 %
                # every pass, and after a legacy migration the depth is EXACT —
                # re-deriving it from its own rounded display would be silent
                # drift on a value the operator never touched. The AREA is part
                # of the key because a bore change is exactly when the depth
                # must be re-derived despite an unchanged dose.
                if (self._dose_last_key is not None
                        and abs(nL - self._dose_last_key[0]) < 1e-9
                        and abs(area - self._dose_last_key[1]) < 1e-15
                        and self._push_depth.value() > 0):
                    return
                mm = (nL / _NL_PER_UL) / area
                self._push_depth.blockSignals(True)
                self._push_depth.setValue(min(mm, self._push_depth.maximum()))
                self._push_depth.blockSignals(False)
                self._dose_last_key = (nL, area)
        finally:
            self._dose_syncing = False
        self._refresh_volume_label()

    def _sync_dose_to_needle(self) -> None:
        """Re-derive the dose ⇄ depth pair after a needle / bore-role change.

        Order matters: cap first (so a dose larger than the new bore holds is
        brought inside range), then convert. A legacy profile migrates here and
        exactly once, after which the VOLUME is authoritative.
        """
        area = self._needle_area_mm2()
        holdup = self._bore_internal_volume_uL(
            getattr(self._hw_config, "needle", None) if self._hw_config else None,
            self._aspirate_bore_index())
        try:
            cap_uL = float(holdup)
        except (TypeError, ValueError):
            cap_uL = 0.0
        if cap_uL > 0:
            q = 10.0 ** self._push_volume.decimals()
            cap_nL = math.floor(cap_uL * _NL_PER_UL * q) / q
            self._push_volume.setMaximum(max(cap_nL, _MIN_DOSE_NL))
        if self._dose_legacy_pending and area > 0:
            self._dose_legacy_pending = False
            logger.info("Cell targeting: migrating a saved column depth of "
                        "%.4f mm to a dose through the aspirating bore.",
                        float(self._push_depth.value()))
            self._sync_dose("depth")
            return
        self._sync_dose("volume")

    def _migrate_legacy_settings(self, values: dict) -> dict:
        """Mark a pre-volume profile so its ``push_depth`` wins, once.

        The conversion CANNOT happen here: this runs from the dialog's
        ``__init__``/load, before ``set_hardware_config``, so there is no bore
        area yet to turn a depth into a volume. Flagging it and resolving at the
        first needle sync is what keeps an existing profile running the dose it
        was saved with instead of snapping to the new field's default.
        """
        try:
            if "push_volume_nL" not in values and "push_depth" in values:
                self._dose_legacy_pending = True
        except (TypeError, AttributeError):
            pass
        return values

    def _current_config(self) -> CellRemovalConfig:
        """Build the executor config from the promoted fields + the bore table.

        The per-bore table supplies the ROLE-derived wiring: which bore aspirates
        (its index, and its pump for the reagent load) and whether a SEPARATE bore
        pushes reagent (decision D5), with that row's own push volume / flow /
        lead time. With no table (no needle configured, or a `__new__`-partial
        page) every trypsin field stays at its default off/zero, which is the
        byte-identical pre-v7.9 single-bore sequence.
        """
        push, _pull = self._push_pull_uL()
        asp = self._program_for_role(BoreRole.ASPIRATE_TARGET)
        kwargs = dict(
            reagent_bore=self._aspirate_pump_id(),
            release_depth_mm=float(self._push_depth.value()),
            release_volume_uL=push,   # resolved from the needle (display + backstop)
            extract_multiplier=float(self._pull_mult.value()),
            dwell_time_s=float(self._dwell.value()),
            push_speed_uL_s=float(self._push_speed.value()),
            pull_speed_uL_s=float(self._pull_speed.value()),
            removal_z_offset_mm=float(self._removal_z.value()),
            place_z_offset_mm=float(self._place_z.value()),
            aspirate_bore_index=(asp.bore_index if asp is not None else 0),
        )
        tryp = self._program_for_role(BoreRole.PUSH_REAGENT)
        if tryp is not None and tryp.pump_id:
            kwargs.update(
                trypsin_enabled=True,
                trypsin_bore=tryp.pump_id,
                trypsin_bore_index=tryp.bore_index,
                trypsin_depth_mm=float(tryp.depth_mm),
                trypsin_volume_uL=float(tryp.volume_uL),
                trypsin_push_rate_uL_s=float(tryp.rate_uL_s),
                trypsin_lead_time_s=float(tryp.lead_time_s),
            )
        return CellRemovalConfig(**kwargs)

    # ── live tuning (published GUI-thread → read executor-thread) ─────

    def _publish_tuning(self, *_) -> None:
        """Snapshot the four knobs for the executor thread.

        Built HERE, on the GUI thread, because reading a ``QDoubleSpinBox`` from
        the executor thread is not safe. The removal height is resolved to
        zero-ref Z here too, so the executor never has to resolve a plate frame.

        A snapshot that fails its bounds is NOT published: the previous one keeps
        running and the reason is shown. Reuses the readiness evaluation rather
        than a second copy of the same rules.
        """
        self._refresh_tuning_echo()
        push, _pull = self._push_pull_uL()
        removal_z = self._plate_offset_to_zref(float(self._removal_z.value()))
        tuning = LiveTuning(
            incubation_s=float(self._dwell.value()),
            dose_volume_uL=(push if push > 0 else None),
            release_depth_mm=float(self._push_depth.value()) or None,
            extract_multiplier=float(self._pull_mult.value()),
            pull_speed_uL_s=float(self._pull_speed.value()),
            removal_z_zref_mm=removal_z,
        )
        reason = self._tuning_refusal(tuning)
        if reason:
            self._set_trial_status("⚠ " + reason + " — the cells still to come "
                                   "keep the previous values.", "peach")
            return
        with self._tuning_lock:
            self._live_tuning = tuning
        if self._exec_thread is not None and self._exec_thread.is_alive():
            self._set_trial_status(
                f"Queued for the cells still to come: {tuning.summary()}")

    def _tuning_refusal(self, tuning) -> str:
        """Why this tuning must not reach the machine, or ""."""
        if tuning.removal_z_zref_mm is None:
            return ("the plate bottom is not calibrated, so a removal height "
                    "cannot be resolved")
        dose = tuning.dose_volume_uL
        if dose is not None:
            holdup = self._bore_internal_volume_uL(
                getattr(self._hw_config, "needle", None)
                if self._hw_config else None, self._aspirate_bore_index())
            if isinstance(holdup, (int, float)) and holdup > 0 and dose > holdup:
                return (f"{_fmt_uL(dose)} is more than the aspirating bore holds "
                        f"({_fmt_uL(holdup)})")
        return ""

    def _current_tuning(self):
        """The published snapshot — called FROM THE EXECUTOR THREAD.

        Touches no Qt object: it only reads an immutable dataclass out from under
        a lock. That is the whole reason the snapshot exists.
        """
        with self._tuning_lock:
            return self._live_tuning

    def _refresh_tuning_echo(self) -> None:
        """Plan's read-only view of the knobs that live on Run."""
        push, pull = self._push_pull_uL()
        text = (f"Tuning (set on Run): dose {_fmt_uL(push)} · "
                f"incubation {float(self._dwell.value()):.0f} s · "
                f"pull {float(self._pull_mult.value()):.2f}× "
                f"@ {float(self._pull_speed.value()):.2f} µL/s · "
                f"removal Z {float(self._removal_z.value()):.3f} mm")
        if getattr(self, "_tuning_echo", None) is not None:
            self._tuning_echo.setText(text)
        if getattr(self, "_tuning_derived", None) is not None:
            self._tuning_derived.setText(
                f"≙ {float(self._push_depth.value()):.3f} mm of column through "
                f"bore {self._aspirate_bore_index() + 1} · "
                f"extraction {_fmt_uL(pull)}")

    def _set_trial_status(self, text: str, color_key: str = "subtext0") -> None:
        if getattr(self, "_trial_status", None) is None:
            return
        self._trial_status.setText(text)
        self._trial_status.setStyleSheet(
            f"color: {COLORS[color_key]}; font-size: {sf(9)}pt;")

    def _surface_z_resolver(self):
        """v7.13 — build a per-target removal-Z evaluator from the measured
        sample surface, or refuse with an operator-actionable reason.

        Returns ``(resolve, "")`` where ``resolve(x_um, y_um) -> zref_mm |
        None`` (None = low-confidence at that point → caller falls back per
        target), or ``(None, why)`` when the mode cannot run at all. The
        conversion chain is: surface focus-µm → needle zref through the
        VERIFIED focus↔needle datum → height above the taught plate bottom
        (polarity-safe) → + the operator's clearance. A surface converting to
        BELOW the plate bottom means the datum or the taught bottom is wrong
        — the whole mode refuses rather than aim a needle there.
        """
        page = getattr(self, "_scan_page", None)
        well = getattr(page, "_scan_well", None) if page is not None else None
        plate_key = None
        try:
            plate_key = page._plate_key() if page is not None else None
        except Exception:
            plate_key = None
        if not well or not plate_key:
            return None, "no scanned well selected on the Survey tab"
        try:
            from SupportClasses import FluorescenceMosaicStore as fms
            survey = fms.get_store().get_focus_survey(plate_key, well)
        except Exception:
            survey = None
        if not survey or not survey.get("samples"):
            return None, (f"no focus survey stored for {well} — run a "
                          f"fluorescence scan with autofocus enabled")
        summary = survey.get("summary") or {}
        center = summary.get("well_center_um")
        radius = summary.get("well_radius_um")
        if not center or not radius:
            try:
                center = page._well_center_um(well)
                radius = page._well_diameter_mm(well) / 2.0 * 1000.0
            except Exception:
                center = radius = None
        if not center or not radius:
            return None, "well geometry unknown"
        try:
            from SupportClasses.SampleSurface import SampleSurfaceModel, CONF_HIGH
            model = SampleSurfaceModel(
                survey["samples"], well_center_um=center,
                well_radius_um=float(radius),
                model=str(survey.get("model") or "plane"))
        except Exception as exc:
            return None, f"surface model unusable ({exc})"
        try:
            from SupportClasses.PlateFocusDatumStore import (
                get_store as datum_store)
            ds = datum_store()
        except Exception:
            return None, "focus↔needle datum store unavailable"
        cam = ""
        try:
            cam = page._camera_key() or ""
        except Exception:
            cam = ""
        ctrl = self._controller
        if ds.needle_z_zref_mm(cam, "", plate_key, 0.0) is None:
            return None, ("no verified focus↔needle datum for this camera / "
                          "plate — capture it via the plate touch-off with "
                          "the focus confirmation")
        if not hasattr(ctrl, "zref_to_print_height") \
                or not hasattr(ctrl, "print_height_to_zref"):
            return None, "controller lacks the plate-bottom height frame"
        # Whole-mode sanity: the surface at the well centre must sit at or
        # above the plate bottom.
        f_c, _conf = model.evaluate(float(center[0]), float(center[1]))
        z_c = ds.needle_z_zref_mm(cam, "", plate_key, float(f_c))
        h_c = ctrl.zref_to_print_height(float(z_c)) if z_c is not None else None
        if h_c is None:
            return None, "plate bottom Z is not calibrated"
        if h_c < -1e-6:
            return None, ("the measured surface converts to BELOW the plate "
                          "bottom — the focus↔needle datum or the taught "
                          "plate bottom is wrong; re-teach before using the "
                          "surface for needle heights")
        offset_mm = float(self._surface_offset.value()) / 1000.0

        def resolve(x_um: float, y_um: float):
            try:
                f_um, conf = model.evaluate(float(x_um), float(y_um))
                if conf != CONF_HIGH:
                    return None
                z_zref = ds.needle_z_zref_mm(cam, "", plate_key, float(f_um))
                if z_zref is None:
                    return None
                h = ctrl.zref_to_print_height(float(z_zref))
                if h is None or h < 0:
                    return None
                return ctrl.print_height_to_zref(h + offset_mm)
            except Exception:
                return None

        return resolve, ""

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
        push, pull = self._push_pull_uL()
        if push <= 0:
            self._volume_label.setText("V = — (set needle inner Ø)")
        else:
            # nL, matching the field the operator typed into: a real column
            # through a 200 µm bore is ~0.003 µL, which printed as "0.0031" and
            # through a 30 µm bore as "0.0001" — indistinguishable from zero.
            self._volume_label.setText(
                f"push {_fmt_uL(push)} · pull {_fmt_uL(pull)} "
                f"(≙ {float(self._push_depth.value()):.3f} mm of column)")
        if hasattr(self, "_reagent_status"):
            self._refresh_reagent_status()

    def _update_button_state(self, *_):
        # The picker + run row are built after the Setup tab, and a settings
        # restore can fire a change notification in between — so tolerate a
        # half-built page rather than crashing construction.
        picker = getattr(self, "_picker", None)
        if picker is None or not hasattr(self, "_start_btn"):
            return
        balanced = picker.is_balanced()
        has_bore = self._bore.count() > 0
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        # A mosaic scan is also driving the stage (and the non-refcounted poller
        # suspend), so Start must wait for it — see _stage_busy.
        page = getattr(self, "_scan_page", None)
        scanning = False
        if page is not None:
            try:
                scanning = bool(page.is_scanning())
            except Exception:
                scanning = False
        readiness = getattr(self, "_readiness", None)
        ready = readiness.can_start() if readiness is not None else True
        self._start_btn.setEnabled(
            balanced and has_bore and ready and not running and not scanning)
        # ALWAYS set a tooltip. A disabled button with no tooltip and a status
        # line reading "Idle." was a dead end: the explanation existed only
        # inside a handler the operator could not reach by clicking.
        if running:
            tip = "A run is in progress."
        elif scanning:
            tip = "A mosaic scan is driving the stage — wait for it to finish."
        elif not balanced:
            tip = ("Each removal needs a paired placement. Pick the cells on the "
                   "well view on the Plan tab, then a placement for each one.")
        elif readiness is not None:
            tip = readiness.headline()
        else:
            tip = "Start the cell-removal run."
        self._start_btn.setToolTip(tip)
        self._abort_btn.setEnabled(running)
        if hasattr(self, "_clean_btn"):
            self._clean_btn.setEnabled(
                bool(getattr(self, "_needs_clean", False))
                and not running and not scanning)

    # ── Start gates ───────────────────────────────────────────────

    def _mosaic_shift_refusal(self) -> str | None:
        """Refuse the run when a target's position came from an unshifted mosaic.

        ``FluorescenceMosaicStore.has_shift`` distinguishes "recorded as zero"
        from "never recorded"; a pre-v7.8 mosaic has no recorded registration
        shift and its pixel→stage error is bounded only by ~20 % of the FOV width
        (hundreds of µm at 10× — larger than a cell). Cell removal descends to
        ~0.1 mm off the glass, so such a position must never command motion.

        In v7.8 this gate was GUI-visual only; here it blocks Start. A target the
        operator clicked on the LIVE view carries no mosaic provenance and is
        therefore never affected.
        """
        picker = self._picker
        try:
            targets = list(picker.picks()) + list(picker.places())
            from_mosaic = [t for t in targets
                           if picker.provenance(t.target_id) == PROV_MOSAIC]
        except Exception:
            return None
        if not from_mosaic:
            return None
        if self._mosaic_can_command_motion():
            return None
        ctx = self._mosaic_context()
        ids = ", ".join(t.target_id for t in from_mosaic[:4])
        if len(from_mosaic) > 4:
            ids += ", …"
        return (
            f"{len(from_mosaic)} target(s) ({ids}) came from a mosaic whose "
            f"registration shift was never recorded, so their pixel→stage "
            f"mapping may be off{self._mosaic_shift_bound_text(ctx)} — larger "
            f"than a cell. Re-scan the well, or go to each one and click it on "
            f"the live view to confirm its position, before running.")

    def _bore_offset_refusal(self, cfg) -> str | None:
        """Refuse a two-bore run whose mount offsets were never measured.

        If the dosing and aspirating bores are different but report the SAME
        offset, their measured separation is zero: the shift between the dose and
        the aspirate moves nothing, so every cell is dosed and then aspirated
        100-500 µm away from. The whole run destroys its cells and collects none,
        silently — the single most consequential thing that could be advisory-only,
        which is why it now blocks.

        Deliberately paired with an executor-side backstop placed BEFORE the dose;
        this GUI gate is what lets the operator see the reason and the fix.
        """
        try:
            if not getattr(cfg, "trypsin_enabled", False):
                return None
            tryp_idx = int(getattr(cfg, "trypsin_bore_index", 0) or 0)
            asp_idx = int(getattr(cfg, "aspirate_bore_index", 0) or 0)
            if tryp_idx == asp_idx:
                return None          # one bore does both — no shift needed
            needle = getattr(self._hw_config, "needle", None) \
                if getattr(self, "_hw_config", None) is not None else None
            if needle is None:
                return None          # unknown data never blocks
            tox, toy = needle_bore_offset_um(needle, tryp_idx)
            aox, aoy = needle_bore_offset_um(needle, asp_idx)
        except Exception:
            logger.debug("bore-offset refusal check failed", exc_info=True)
            return None
        if abs(tox - aox) > 1e-6 or abs(toy - aoy) > 1e-6:
            return None
        return (
            f"Bore {tryp_idx + 1} (dosing) and bore {asp_idx + 1} (aspirating) "
            f"report the same mount offset, so their measured separation is "
            f"zero — the offsets have not been measured for this assembly. "
            f"Running would dose every target and then aspirate 100-500 µm away "
            f"from it, destroying the cells without collecting any. Measure the "
            f"bore offsets on Calibration → Needle Location first (they must be "
            f"re-measured after every needle change or re-seat).")

    def _run_narrative(self, cfg, prep_enabled, clean_enabled) -> list[str]:
        """The per-cell sequence in the operator's terms, with resolved numbers.

        Built from the SAME config the executor receives, so the dialog cannot
        describe a run different from the one that happens.
        """
        needle = getattr(self._hw_config, "needle", None) \
            if self._hw_config is not None else None
        asp = int(getattr(cfg, "aspirate_bore_index", 0) or 0) + 1
        lines: list[str] = []
        if cfg.trypsin_enabled:
            tb = int(getattr(cfg, "trypsin_bore_index", 0) or 0) + 1
            dose = self._trypsin_push_uL()
            rate = self._setup_panel.effective_dose_rate(
                cfg.trypsin_bore, cfg.trypsin_push_rate_uL_s) \
                if self._setup_panel is not None else cfg.trypsin_push_rate_uL_s
            rate = rate or cfg.trypsin_push_rate_uL_s
            secs = (dose / rate) if rate else 0.0
            reagent = (self._setup_panel.trypsin_reagent_name()
                       if self._setup_panel is not None else "")
            lines.append(
                f"Load {_fmt_uL(dose)} of {reagent or 'the dosing reagent'} into "
                f"bore {tb} ({cfg.trypsin_bore})")
            lines.append(
                f"Lower bore {tb} to {cfg.removal_z_offset_mm:.3f} mm above the "
                f"plate bottom and dose {_fmt_uL(dose)} at {rate:.2f} µL/s "
                f"(≈{secs:.1f} s)")
            try:
                tox, toy = needle_bore_offset_um(needle, tb - 1)
                aox, aoy = needle_bore_offset_um(needle, asp - 1)
                lines.append(
                    f"Shift {aox - tox:+.0f}, {aoy - toy:+.0f} µm so bore {asp} "
                    f"({cfg.reagent_bore}) is on the same cell, then wait "
                    f"{float(cfg.trypsin_lead_time_s):.1f} s")
            except Exception:
                lines.append(f"Shift so bore {asp} is on the same cell")
            lines.append(
                f"Incubate {float(cfg.dwell_time_s):.0f} s "
                f"(total dose→aspirate "
                f"{float(cfg.trypsin_lead_time_s) + float(cfg.dwell_time_s):.1f} s)")
        else:
            col = cfg.compute_release_volume_uL(needle)
            lines.append(
                f"Load {_fmt_uL(col)} of "
                f"{self._selected_reagent() or 'the cell-release reagent'} into "
                f"bore {asp} ({cfg.reagent_bore})")
            lines.append(
                f"Lower to {cfg.removal_z_offset_mm:.3f} mm above the plate "
                f"bottom and push {_fmt_uL(col)} at "
                f"{float(cfg.push_speed_uL_s):.2f} µL/s")
            lines.append(f"Incubate {float(cfg.dwell_time_s):.0f} s")
        pull = cfg.compute_extract_volume_uL(needle)
        lines.append(
            f"Pull {_fmt_uL(pull)} at {float(cfg.pull_speed_uL_s):.2f} µL/s "
            f"({float(cfg.extract_multiplier):.2f}× the column)")
        lines.append(
            f"Travel to the placement, lower to "
            f"{cfg.place_z_offset_mm:.3f} mm above the plate bottom and dispense")
        return lines

    def _confirm_run(self, cfg, n_targets, removal_z, place_z,
                     prep_enabled, clean_enabled) -> bool:
        """Show the pre-run confirmation. True = the operator confirmed.

        A failure to BUILD the dialog must not block a run the gates already
        approved, so any exception here degrades to "proceed" — the same
        unknown-data-never-blocks rule the readiness model follows.
        """
        try:
            from gui.dialogs.cell_removal_confirm_dialog import (
                CellRemovalConfirmDialog)
            bracket = []
            if prep_enabled:
                bracket.append("needle prep (waste → oil → wash → buffer)")
            if clean_enabled:
                bracket.append("post-clean (waste → wash → reload buffer)")
            note = ("Once per run: " + " and ".join(bracket) + "."
                    if bracket else
                    "Needle prep and post-clean are OFF — the needle is used "
                    "as-is and stays loaded afterwards.")
            dlg = CellRemovalConfirmDialog(
                self,
                n_targets=int(n_targets),
                steps=self._run_narrative(cfg, prep_enabled, clean_enabled),
                readiness=getattr(self, "_readiness", None),
                clearance_mm=float(cfg.removal_z_offset_mm),
                bracket_note=note)
        except Exception:
            logger.exception("could not build the pre-run confirmation")
            return True
        return bool(dlg.exec()) and dlg.acknowledged()

    def _prep_pump_refusal(self, cfg) -> str | None:
        """Refuse when a bore this run drives names an unconfigured pump.

        ``move_pumps_uL`` raises ``ValueError`` for a pump with no syringe — and
        it would do so during PREP, with the needle already dipped in a service
        well. Catching it here means the operator sees the fix instead of a
        traceback mid-run.
        """
        try:
            if not (self._prep_check.isChecked() or self._clean_check.isChecked()):
                return None
            pumps = getattr(self._hw_config, "pumps", None) or {}
            driven = cfg.active_bores()
        except Exception:
            logger.debug("prep-pump refusal check failed", exc_info=True)
            return None
        missing = []
        for entry in driven:
            pid = entry.get("pump_id")
            pump = pumps.get(pid) if hasattr(pumps, "get") else None
            if pump is None:
                missing.append((pid, int(entry.get("bore_index", 0)) + 1))
                continue
            # `is_configured` is the canonical marker (a syringe assigned AND
            # enabled). Read it permissively: an unknown/partial stand-in must
            # not block the run — unknown data never blocks.
            ok = getattr(pump, "is_configured", True)
            if isinstance(ok, bool) and not ok:
                missing.append((pid, int(entry.get("bore_index", 0)) + 1))
        if not missing:
            return None
        which = ", ".join(f"bore {b} → {p or '(unassigned)'}" for p, b in missing)
        return (
            f"Needle prep would drive a pump that is not set up: {which}. "
            f"Enable that pump and give it a syringe on Hardware Setup → Pump, "
            f"or set that bore's role to Idle. (Prep would otherwise fail with "
            f"the needle already dipped in a service well.)")

    def _on_start(self, *, limit: int | None = None,
                  prep: bool | None = None, clean: bool | None = None):
        """Run the picked cells.

        ``limit`` / ``prep`` / ``clean`` exist so "Test one cell" is the SAME
        gated path as a full run rather than a parallel one — every refusal, the
        confirmation and the executor wiring are shared. Defaults reproduce the
        button's behaviour exactly.
        """
        # ONE busy check for every stage driver — the executor thread AND the
        # embedded mosaic scan, which also drives the stage and toggles the
        # non-refcounted poller suspend. _update_button_state greys Start out too,
        # but a race or a programmatic call must not get through either.
        if self._stage_busy():
            return

        if not self._picker.is_balanced():
            self._status.setText(
                "Each removal needs a paired placement — pick and place counts "
                "must match.")
            return

        refusal = self._mosaic_shift_refusal()
        if refusal:
            self._status.setText(refusal)
            return

        pairs = (self._untried_pairs() if limit is not None
                 else self._picker.pairs())
        if limit is not None:
            pairs = pairs[:max(1, int(limit))]
        if not pairs:
            self._status.setText("No cells left to run.")
            return
        cfg = self._current_config()

        # Promoted from advisory to BLOCKING. Same reasoning that promoted the
        # mosaic-shift check above: a position silently wrong by more than a cell
        # must not command a descent to 0.1 mm off the glass.
        refusal = self._bore_offset_refusal(cfg)
        if refusal:
            self._status.setText(refusal)
            return

        refusal = self._prep_pump_refusal(cfg)
        if refusal:
            self._status.setText(refusal)
            return

        if self._safe_z is None:
            self._status.setText(
                "No safe Z configured — set it on the Calibration page "
                "before running cell removal.")
            return

        # The ZP (Z + pump) board is REQUIRED (retract + pumps). Refuse otherwise.
        if not getattr(self._controller, "is_zp_connected", False):
            self._status.setText(
                "ZP (Z + pump) board not connected — cell removal needs it to "
                "retract the needle and run the pumps. Reconnect it first.")
            return

        removal_z = self._plate_offset_to_zref(cfg.removal_z_offset_mm)
        place_z = self._plate_offset_to_zref(cfg.place_z_offset_mm)
        if removal_z is None or place_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — calibrate it on the "
                "Calibration page so the removal/place heights can be resolved.")
            return

        # v7.13 — sample-surface removal Z: resolve the surface evaluator up
        # front so a missing prerequisite REFUSES before anything moves
        # (a silent fallback for the whole run would look like the feature
        # working while every removal ran at the plate-bottom offset).
        surface_resolver = None
        if getattr(self, "_surface_z_chk", None) is not None \
                and self._surface_z_chk.isChecked():
            surface_resolver, why = self._surface_z_resolver()
            if surface_resolver is None:
                self._status.setText(
                    f"Sample-surface removal Z unavailable: {why} — untick "
                    f"'Removal Z from measured sample surface' or fix the "
                    f"prerequisite.")
                return

        # v7.9 (post-audit): a DOSING bore REPLACES the aspirating bore's push, so
        # the aspirate-side reagent is only required when there is no dosing bore.
        # Demanding it unconditionally is exactly what forced the operator into the
        # double-dose configuration: they had to name a second reagent for a load
        # that would then also be pushed onto the cell.
        dosing_armed = bool(cfg.trypsin_enabled)

        push_uL, _pull_uL = self._push_pull_uL()
        if push_uL <= 0:
            # Still unconditional, but the CONSEQUENCE differs: with a dosing bore
            # the column no longer doses anything — it only sizes the pull.
            self._status.setText(
                "The aspirating bore's column resolves to 0 µL, so the "
                "extraction volume would be 0 and nothing would be collected — "
                "set that bore's geometry on Hardware Setup → Needle."
                if dosing_armed else
                "Set the needle inner diameter (Hardware Setup → Needle) so the "
                "push volume (needle area × depth) can be computed.")
            return
        reagent = self._selected_reagent()
        if reagent is None and not dosing_armed:
            self._status.setText(
                "Select the cell-release reagent (e.g. trypsin) to load.")
            return
        reagent_pos = self._reagent_source_pos()
        if reagent_pos is None and not dosing_armed:
            self._status.setText(
                f"Reagent “{reagent}” has no calibrated reagent well — assign it "
                "(Hardware Setup → Ink) and run Plate Location.")
            return
        reagent_dip_z = self._plate_offset_to_zref(float(self._reagent_z.value()))
        if reagent_dip_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — can't resolve the reagent "
                "dip Z.")
            return

        # A dedicated pushing bore must have something to load, or the executor
        # would raise mid-run with the needle already in the well.
        trypsin_pos = None
        if cfg.trypsin_enabled:
            if not self._trypsin_reagent():
                self._status.setText(
                    f"Bore {cfg.trypsin_bore_index + 1} is set to push reagent — "
                    "choose the reagent it loads on the Plan tab, or set that "
                    "bore to Idle.")
                return
            trypsin_pos = self._trypsin_source_pos()
            if trypsin_pos is None:
                self._status.setText(
                    f"The pushing bore's reagent “{self._trypsin_reagent()}” has "
                    "no calibrated reagent well — assign it (Hardware Setup → "
                    "Ink) and run Plate Location.")
                return
            if self._trypsin_push_uL() <= 0:
                self._status.setText(
                    f"Bore {cfg.trypsin_bore_index + 1}'s push volume resolves "
                    "to 0 µL — set an explicit volume or a push depth with that "
                    "bore's geometry on Hardware Setup → Needle.")
                return

        # Prep / clean / wash-after-pickup inputs + gates (shared service wells).
        # A test try can suppress either bracket (reuse the loaded needle, keep it
        # loaded for the next try) — but only DOWNWARD: `and` means a try can
        # never turn on a bracket the operator switched off.
        prep_enabled = self._prep_check.isChecked() and (
            True if prep is None else bool(prep))
        clean_enabled = self._clean_check.isChecked() and (
            True if clean is None else bool(clean))
        wash_after_pickup = self._wash_after_pickup_check.isChecked()
        service_positions: dict[str, tuple[float, float]] = {}
        service_z = None
        needle_uL = 0.0
        if prep_enabled or clean_enabled or wash_after_pickup:
            service_positions, missing = resolve_service_positions(
                self._hw_config, self._well_positions, self._plate)
            service_z = self._plate_offset_to_zref(float(self._service_z.value()))
            if service_z is None:
                self._status.setText(
                    "Plate bottom Z is not calibrated — can't resolve the "
                    "service / wash dip Z.")
                return
        if prep_enabled or clean_enabled:
            needle_uL = needle_volume_uL(self._hw_config)
            if needle_uL <= 0:
                self._status.setText(
                    "Prep / clean needs the needle inner diameter + length "
                    "(Hardware Setup → Needle), or turn them off.")
                return
            if missing:
                self._status.setText(
                    "Prep / clean needs these reagent wells assigned in Hardware "
                    f"Setup → Ink (Reagent Locations) and calibrated: "
                    f"{', '.join(missing)} — or turn them off.")
                return
        elif wash_after_pickup and "wash" not in service_positions:
            self._status.setText(
                "Wash-after-pickup needs a wash well assigned + calibrated "
                "(Hardware Setup → Ink → Reagent Locations), or turn it off.")
            return

        # ── LAST GATE BEFORE MOTION ───────────────────────────────────────
        # Every advisory used to be appended to the status label on the same line
        # as "Running N cell removals…", immediately before the thread started —
        # i.e. after the decision. This run lowers a needle to ~0.1 mm above glass
        # and doses live cells; it gets a summary and an explicit acknowledgement.
        if not self._confirm_run(cfg, len(pairs), removal_z, place_z,
                                 prep_enabled, clean_enabled):
            self._status.setText("Cancelled — nothing has moved.")
            return

        # v7.13 — stamp per-target sample-surface removal heights. A target
        # whose surface evaluation is low-confidence (outside the surveyed
        # region) keeps the run-level plate-bottom offset, per target, logged.
        n_surface = n_fallback = 0
        if surface_resolver is not None:
            stamped = []
            for pick, place in pairs:
                z_over = surface_resolver(float(pick.x_um), float(pick.y_um))
                if z_over is not None:
                    pick = replace(pick, pick_z_zref_mm=float(z_over))
                    n_surface += 1
                else:
                    n_fallback += 1
                stamped.append((pick, place))
            pairs = stamped
            logger.info(
                "Cell removal: sample-surface Z on %d target(s), plate-bottom "
                "fallback on %d", n_surface, n_fallback)

        queue = OperationQueue()
        for pick, place in pairs:
            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=OperationType.CELL_TARGET_REMOVAL,
                source_target=pick,
                dest_target=place,
                # ⚠ Each operation gets its OWN config object, not one shared
                # instance. `_apply_live_tuning` replaces `op.config` per
                # operation, and a shared object would make one operator edit
                # retroactively rewrite the record of every cell already done.
                config=replace(cfg),
            )
            queue.add(op)

        executor = PickPlaceExecutor(self._controller, self._hw_config)
        executor.safe_z_mm = float(self._safe_z)
        executor.pick_z_mm = removal_z      # removal height (reused field)
        executor.place_z_mm = place_z       # placement height
        executor.reagent_well_pos = reagent_pos
        executor.reagent_dip_z_mm = reagent_dip_z
        # The dedicated pushing bore's own reagent well. Goes through
        # set_well_positions (never a direct dict mutation — _well_positions is a
        # CLASS-level default, so mutating it in place would leak across runs).
        if trypsin_pos is not None:
            executor.set_well_positions({cfg.trypsin_well_key: trypsin_pos})
        # Advanced motion / timeout knobs (always applied).
        executor.intra_well_retract_mm = float(self._intra_retract.value())
        executor.z_timeout_s = float(self._z_timeout.value())
        executor.xy_timeout_s = float(self._xy_timeout.value())
        # Service dip Z + wash mechanics + the wash well — shared by prep/clean
        # AND the after-pickup wash.
        if prep_enabled or clean_enabled or wash_after_pickup:
            executor.service_z_mm = service_z
            executor.wash_cycles = int(self._wash_cycles.value())
            executor.wash_z_amplitude_mm = float(self._wash_z_amp.value())
            executor.wash_xy_amplitude_um = float(self._wash_xy_amp.value())
            executor.wash_dwell_s = float(self._wash_dwell.value())
            executor.wash_well_pos = service_positions.get("wash")
        if prep_enabled or clean_enabled:
            executor.prep_bore = cfg.reagent_bore
            # v7.9 (D8) — condition EVERY bore this run drives, simultaneously.
            # ⚠ This one line is what makes the whole simultaneous-prep feature
            # live: without it `prep_bores` had no production writer, so a
            # dedicated dosing bore was never conditioned or cleaned and arrived
            # at its reagent well full of AIR — dosing nothing, releasing no
            # cells, and reporting success. A single-bore run collapses back to
            # the legacy `prep_bore` path inside `_prep_bore_plan`, byte-identical.
            executor.prep_bores = cfg.active_bores()
            executor.needle_volume_uL = needle_uL
            executor.prep_rate_uL_s = float(self._prep_rate.value())
            executor.oil_needles = float(self._oil_needles.value())
            executor.buffer_needles = float(self._buffer_needles.value())
            executor.post_dispense_needles = float(self._post_dispense.value())
            executor.waste_well_pos = service_positions["waste"]
            executor.oil_well_pos = service_positions["oil"]
            executor.buffer_well_pos = service_positions["buffer"]
        executor.do_prep = prep_enabled
        executor.do_post_clean = clean_enabled
        executor.wash_after_pickup = wash_after_pickup

        # Operator decision: mid-run edits reach the cells still to come. The
        # provider is called on the EXECUTOR thread and only reads the immutable
        # snapshot `_publish_tuning` left under a lock — never a Qt widget.
        self._publish_tuning()
        executor.tuning_provider = self._current_tuning

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
        # Surface the per-bore advisories at the moment of the run — an unmeasured
        # bore offset does not fail loudly, it just lands 100-500 µm out.
        notes = []
        if self._setup_panel is not None:
            notes = self._setup_panel.validation_notes()
        note_text = ("  ⚠ " + "  ⚠ ".join(notes)) if notes else ""
        self._status.setText(
            f"Running {n} cell removal{'s' if n > 1 else ''}…{note_text}")
        # Auto-select Run: the operator has just committed, so show them the work.
        # Deliberately no auto-return afterwards — the run's outcome is on Run,
        # and yanking the surface away at the end would hide it.
        self._show_run_tab()
        self._exec_thread = threading.Thread(
            target=worker, name="CellRemovalExecutor", daemon=True)
        self._exec_thread.start()
        self._update_button_state()
        self._refresh_trial_buttons()

    def _show_run_tab(self) -> None:
        try:
            self._tabs.setCurrentIndex(self._TAB_RUN)
        except Exception:
            logger.debug("could not switch to the Run tab", exc_info=True)

    def _on_clean_now(self):
        """Run the post-clean cycle on a FRESH executor, on the operator's say-so.

        A fresh executor (rather than reusing the aborted one) is deliberate: the
        aborted executor's abort flag is set, and ``move_pump_uL`` refuses new
        moves while it is — which is exactly the guarantee that keeps Abort
        responsive and must not be un-set to squeeze a clean out of it.
        """
        if self._exec_thread is not None:
            return
        busy = self._stage_busy()
        if busy:
            self._status.setText(busy)
            return
        cfg = self._current_config()
        refusal = self._prep_pump_refusal(cfg)
        if refusal:
            self._status.setText(refusal)
            return
        service_positions, _unresolved = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        missing = [r for r in ("waste", "wash", "buffer")
                   if not service_positions.get(r)]
        if missing:
            self._status.setText(
                f"Cleaning needs these reagent wells assigned in Hardware Setup "
                f"→ Ink (Reagent Locations) and calibrated: "
                f"{', '.join(missing)}.")
            return
        service_z = self._plate_offset_to_zref(float(self._service_z.value()))
        if service_z is None:
            self._status.setText(
                "Plate bottom Z is not calibrated — calibrate it on the "
                "Calibration page so the service dip height can be resolved.")
            return
        needle_uL = needle_volume_uL(self._hw_config)
        if needle_uL <= 0:
            self._status.setText(
                "Set the needle geometry (Hardware Setup → Needle) so one "
                "bore's worth of fluid can be computed for the clean.")
            return

        executor = PickPlaceExecutor(self._controller, self._hw_config)
        executor.safe_z_mm = self._safe_z
        executor.prep_bore = cfg.reagent_bore
        executor.prep_bores = cfg.active_bores()
        executor.needle_volume_uL = needle_uL
        executor.prep_rate_uL_s = float(self._prep_rate.value())
        executor.buffer_needles = float(self._buffer_needles.value())
        executor.post_dispense_needles = float(self._post_dispense.value())
        executor.service_z_mm = service_z
        executor.wash_cycles = int(self._wash_cycles.value())
        executor.wash_z_amplitude_mm = float(self._wash_z_amp.value())
        executor.wash_xy_amplitude_um = float(self._wash_xy_amp.value())
        executor.wash_dwell_s = float(self._wash_dwell.value())
        executor.waste_well_pos = service_positions["waste"]
        executor.wash_well_pos = service_positions["wash"]
        executor.buffer_well_pos = service_positions["buffer"]
        executor.z_timeout_s = float(self._z_timeout.value())
        executor.xy_timeout_s = float(self._xy_timeout.value())
        self._executor = executor
        bridge = self._bridge
        executor.on_sub_step = lambda op, step: bridge.sub_step.emit(op, step)

        def worker():
            ok = False
            try:
                executor.run_post_clean()
                ok = True
            except Exception as e:
                logger.exception("Clean-needle cycle failed: %s", e)
            finally:
                # The needle must end retracted whatever happened, exactly as a
                # full run guarantees.
                try:
                    executor._retract_to_safe_z()
                except Exception:
                    logger.debug("post-clean retract failed", exc_info=True)
            bridge.finished.emit(ok)

        self._needs_clean = False
        self._status.setText("Cleaning the needle…")
        self._exec_thread = threading.Thread(
            target=worker, name="CellRemovalClean", daemon=True)
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
        self._note_run_progress(op, "▸")

    def _on_op_completed(self, op):
        self._status.setText(f"{op.op_id}: complete.")
        # Mark the cell tried BEFORE logging, so "test one cell" advances even if
        # the operator never presses it again.
        try:
            tid = getattr(op.source_target, "target_id", "")
            if tid:
                self._tried_target_ids.add(str(tid))
        except Exception:
            pass
        self._note_run_progress(op, "✓")
        self._refresh_trial_buttons()

    def _on_op_failed(self, op, msg: str):
        self._status.setText(f"{op.op_id}: FAILED — {msg}")
        self._note_run_progress(op, "✗")

    def _note_run_progress(self, op, mark: str) -> None:
        """One line per cell, naming the tuning it actually received.

        This is what makes the mid-run-edit decision honest: a batch is no longer
        one uniform experiment, so the record has to say which cell got what
        rather than leaving it to be inferred from when the edit was made.
        """
        log = getattr(self, "_run_log", None)
        if log is None:
            return
        try:
            tid = getattr(op.source_target, "target_id", "") or op.op_id
            tuning = getattr(op, "applied_tuning", None)
            detail = tuning.summary() if tuning is not None else ""
            text = f"{mark} {tid}" + (f"   {detail}" if detail else "")
            # Replace the in-progress line for this cell rather than stacking.
            for i in range(log.count()):
                item = log.item(i)
                if item.data(Qt.ItemDataRole.UserRole) == op.op_id:
                    item.setText(text)
                    break
            else:
                from PySide6.QtWidgets import QListWidgetItem
                item = QListWidgetItem(text)
                item.setData(Qt.ItemDataRole.UserRole, op.op_id)
                log.addItem(item)
            log.scrollToBottom()
        except Exception:
            logger.debug("could not record the run progress", exc_info=True)

    def _on_progress(self, done: int, total: int, msg: str):
        self._status.setText(f"[{done}/{total}] {msg}")
        if getattr(self, "_run_progress", None) is not None:
            self._run_progress.setText(f"Cell {min(done + 1, total)} of {total}"
                                       + (f" — {msg}" if msg else ""))

    def _on_sub_step(self, op, step: str):
        self._status.setText(f"{op.op_id}: {step}")
        if getattr(self, "_run_progress", None) is not None:
            self._run_progress.setText(step)

    # ── test one cell / run remaining ─────────────────────────────

    def _untried_pairs(self) -> list:
        """The pick/place pairs whose cell has not been run yet."""
        try:
            pairs = self._picker.pairs()
        except Exception:
            return []
        out = []
        for pick, place in pairs:
            tid = str(getattr(pick, "target_id", "") or "")
            if tid and tid in self._tried_target_ids:
                continue
            out.append((pick, place))
        return out

    def _on_test_one_cell(self):
        """Run the NEXT UN-TRIED cell on its own.

        Prep runs only when the needle is not already loaded, so a second try
        reuses what the first one conditioned. No post-clean between tries — that
        is what the explicit "Clean needle now" button is for.
        """
        remaining = self._untried_pairs()
        if not remaining:
            self._set_trial_status(
                "Every picked cell has been tried. Clear the list or pick more "
                "cells to try again.", "peach")
            return
        self._on_start(limit=1, prep=not self._needle_is_loaded(),
                       clean=False)

    def _on_run_remaining(self):
        """Run every cell not yet tried, with the tuning as it stands."""
        if not self._untried_pairs():
            self._set_trial_status("Nothing left to run.", "peach")
            return
        self._on_start(prep=not self._needle_is_loaded())

    def _needle_is_loaded(self) -> bool:
        """True when a previous try already conditioned the needle.

        ``_needs_clean`` is set exactly when a run stopped with a bore still
        loaded, which is the same condition as "already prepped" from the point of
        view of a follow-up try.
        """
        return bool(self._tried_target_ids) or bool(
            getattr(self, "_needs_clean", False))

    def _refresh_trial_buttons(self) -> None:
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        left = len(self._untried_pairs())
        for btn in (getattr(self, "_test_one_btn", None),
                    getattr(self, "_run_rest_btn", None)):
            if btn is not None:
                btn.setEnabled(bool(left) and not running
                               and not self._stage_busy(quiet=True))
        if getattr(self, "_test_one_btn", None) is not None:
            nxt = self._untried_pairs()
            label = "Test one cell"
            if nxt:
                tid = getattr(nxt[0][0], "target_id", "")
                if tid:
                    label = f"Test one cell ({tid})"
            self._test_one_btn.setText(label)
        if getattr(self, "_run_rest_btn", None) is not None:
            self._run_rest_btn.setText(
                f"Run remaining ({left})" if left else "Run remaining")

    def _on_finished(self, ok: bool):
        # Read the dose state BEFORE dropping the executor reference.
        warning = None
        try:
            ex = self._executor
            if ex is not None and hasattr(ex, "pending_dose_warning"):
                warning = ex.pending_dose_warning()
        except Exception:
            logger.debug("pending-dose check failed", exc_info=True)
        self._exec_thread = None
        self._executor = None
        if warning:
            # A stop that left reagent on a live cell is not "Stopped." — the
            # cells keep digesting and the needle is still loaded. Say both, and
            # offer the clean.
            self._status.setText(warning)
            self._needs_clean = True
        else:
            self._status.setText(
                "Done." if ok else "Stopped (aborted or failed).")
        self._update_button_state()
