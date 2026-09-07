"""quick_print_workflow.py — Quick Print workflow page.

v7.5.x: No-frills single-object print. The user picks one object (a built-in
simple shape or any saved print from the ``config/prints`` library), clicks one
well, and presses Print. The page builds a single-well :class:`PrintJob` with the
same ``build_well_plate_job()`` helper the standard Print Setup uses, then runs it
through a fresh :class:`PrintManager` in discrete mode — so motion is identical to
the normal print flow (no new coordinate/motion math).

Coordinate contract (verified against the discrete executor):
    - ``build_well_plate_job(well_positions, path_points, ...)`` takes
      ``well_positions`` as ``[(name, x_mm, y_mm)]`` in **zero-ref mm** and
      ``path_points`` in **mm relative to well center**.
    - ``MOVE_XY {x,y}`` → ``move_xy_absolute(x, y, from_zero_ref=True)``.
    - The page receives calibrated well positions as **absolute stage µm** via
      ``set_calibration_data``; ``controller.zero_position`` is in µm. The well
      center is taken from the calibrated position when available (converted to
      zero-ref mm), else the geometric ``plate.get_well_position`` (already
      A1-relative mm = zero-ref mm).

Object geometry is produced through the canonical backend pipeline
(``GeometryEngine.PrintObject.from_dict`` + ``generate_object_trajectory``), the
same one Print Builder uses, so every saved object type renders correctly. Only
the XY columns of the trajectory are used as ``path_points`` (extrusion is driven
uniformly by the flow knob).
"""

from __future__ import annotations

import bisect
import contextlib
import copy
import logging
import math
import re
import threading
import time
from typing import Optional

import numpy as np

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QMessageBox, QSplitter, QCheckBox, QSpinBox,
    QButtonGroup, QStackedWidget, QScrollArea, QListWidget, QListWidgetItem,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card, FormRow, StatusBadge
from gui.pages.workflows.quick_print_report import QuickPrintReportPanel
from gui.widgets.jog_well_plate import WellPlateNavigator
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.print_trajectory_monitor import PrintTrajectoryMonitorView
from gui.widgets.section_stack import SectionStack, wire_section_promotion
# v7.6: the measured-calibration store is read by the two-parameter surface
# (resolution default, stage-max cap, the machine-characterised gate), so it is
# imported once here rather than locally at each site.
from SupportClasses.PrintTimingCalibrationStore import get_store
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)
from gui.pages.workflows._reagent_prep import (
    SERVICE_ROLES, service_well_names, resolve_service_positions,
    needle_volume_uL, resolve_pickup_well,
)

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:  # pragma: no cover - defensive import
    CameraRole = None

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, build_well_plate_job,
)
from SupportClasses.PrintFileManager import PrintFileManager
from SupportClasses.WorkflowLayoutStore import WorkflowLayoutStore
from SupportClasses.PhysicalModels import (
    needle_orifice_area_mm2, needle_orifice_od_mm,
)
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor, AbortException
from SupportClasses.PrintQueue import (
    ORDER_INK, ORDER_PLATE, QueuedPrint, count_ink_swaps, order_queue,
    plate_index_from_names, queue_from_state, queue_to_state,
)
from SupportClasses import PrintQueue as _pq

logger = logging.getLogger(__name__)


#: v7.7: ``PrintManager._report_progress`` already prefixes "[i/total] " to its
#: messages, so the page must not add a second counter. Matches (and strips) an
#: existing prefix.
_PROGRESS_PREFIX_RE = re.compile(r"^\[\d+/\d+\]\s*")

# Smallest ink reserve worth calling a reserve (µL). A pulled glass capillary's
# tip holds only a few nanolitres, which is below one pump step — the operator
# has to supply the margin as ink padding instead, so we say so rather than
# silently shipping a reserve that cannot do its job.
_MIN_MEANINGFUL_RESERVE_UL = 0.05


# Built-in simple shapes → object dicts fed through the same geometry pipeline
# as saved objects. Size (mm) maps to circle/disc radius; the dot ignores it.
_SIMPLE_SHAPES = {
    "dot": "⋅ Dot",
    "circle": "◯ Circle",
    "meander": "◉ Meander (filled disc)",
}


def _overlay_channels(sim_result, *, vol_per_mm: float,
                      t_base: float = 0.0) -> tuple:
    """v7.6: per-sample overlay channels for one simulated segment.

    Returns ``(speed_mm_s, flow_uL_s, error_um, time_s)`` — four lists, each
    one value per sample of ``sim_result.samples``, so they align 1:1 with the
    predicted polyline the monitor draws.

      • speed = |Δposition| / Δt between consecutive samples (the first sample
        copies the second, so the lists match the point count),
      • flow  = speed × volume-per-mm (what the pump must deliver there),
      • error = |signed cross-track| from the run's ``cross_profile``, looked up
        by arc length,
      • time  = the sample's own timestamp, offset by ``t_base`` so multiple
        segments form one continuous print clock.

    Pure/module-level so the prediction worker can call it off the GUI thread.
    """
    samples = list(getattr(sim_result, "samples", None) or [])
    n = len(samples)
    if n == 0:
        return [], [], [], []
    times = [float(sm[2] if len(sm) > 2 and sm[2] is not None else 0.0)
             for sm in samples]
    speeds = [0.0] * n
    for i in range(1, n):
        dt = max(1e-6, times[i] - times[i - 1])
        speeds[i] = math.hypot(samples[i][0] - samples[i - 1][0],
                               samples[i][1] - samples[i - 1][1]) / dt
    if n > 1:
        speeds[0] = speeds[1]
    flows = [v * max(0.0, vol_per_mm) for v in speeds]

    # error: nearest cross-profile entry by arc length (profile is s-ordered)
    prof = list(getattr(sim_result, "cross_profile", None) or [])
    errors = [0.0] * n
    if prof:
        s_vals = [p[0] for p in prof]
        s_acc = 0.0
        for i in range(n):
            if i:
                s_acc += math.hypot(samples[i][0] - samples[i - 1][0],
                                    samples[i][1] - samples[i - 1][1])
            j = bisect.bisect_left(s_vals, s_acc)
            j = min(max(j, 0), len(prof) - 1)
            errors[i] = abs(float(prof[j][1]))
    return speeds, flows, errors, [t_base + t for t in times]


class _QueueResolveError(Exception):
    """v7.19: a queued print cannot be resolved into runnable units.

    Carries an operator-facing reason, which ``_resolve_queue`` collects into the
    "NOT running" list of the confirm dialog — an entry that cannot run is named,
    never silently dropped.
    """


class _PrintBridge(QObject):
    """Bridges PrintManager callbacks (daemon thread) → Qt signals so GUI
    updates land on the main thread (queued across threads)."""

    progress = Signal(int, int, str)   # current, total, message
    state = Signal(object)             # PrintState
    # v7.5.x: the pre-position move runs on a worker thread so the live
    # microscope feed keeps updating during positioning; this fires (on the
    # GUI thread) when the move finishes, carrying whether Z/XY confirmed.
    prepositioned = Signal(bool)
    # v7.5.x: the post-print cleanup (waste → wash → reset oil) runs on a worker
    # thread; this fires when it finishes (empty string = ok, else the reason).
    cleanup_done = Signal(str)
    # v7.5.x: a multi-ink (abstract-ink) print runs its whole sequential
    # ink-swap sequence on one worker thread; this fires when it finishes
    # (empty string = ok, else the reason).
    multi_done = Signal(str)
    # v7.19: the plate-wide print QUEUE finished ("" = ok, else the reason).
    # Deliberately a SEPARATE signal from `multi_done`: that handler writes
    # multi-ink-specific status text and is referenced from four coupling points,
    # so sharing it would make the status line lie about which engine ran.
    queue_done = Signal(str)
    # v7.19: the queue advanced to a unit — (index, total, label). Carries the
    # GUI-thread work (status text, live-sampling toggle) OFF the worker, because
    # `_set_print_live` drives a QTimer that may only be touched here.
    queue_unit = Signal(int, int, str)
    # v7.19: the queue parked at a well boundary (True) or resumed (False), so the
    # operator can go run another workflow mid-batch.
    queue_held = Signal(bool)
    # v7.5.x: background XYPathSimulator run for the planned path finished —
    # (generation, predicted segments in zero-ref µm, caption text, overlays).
    # v7.6: overlays = {mode: {values, unit, vmin, vmax}} for the optional
    # speed / flow / error / time colouring of the predicted path.
    predicted = Signal(int, object, str, object)
    # v7.7: one telemetry record from the velocity/feed-plan follower (the
    # `vel_sample` dict). Emitted from the print thread at the executor's
    # existing ~5 Hz decimation; the slot only stores it, so a slow paint can
    # never back-pressure the control loop.
    vel_sample = Signal(object)


class QuickPrintWorkflowPage(QWidget):
    """Quick Print workflow page.

    Signals:
        back_requested: User clicked the Back button.
        print_state_changed: A print reached a terminal state (COMPLETED /
            ABORTED / ERROR). Carries the :class:`PrintState`. Emitted for
            hosts that EMBED this page (see the ``embedded`` ctor kwarg) and
            need to act on a finished run — e.g. the Print Calibrator, which
            then reads :attr:`last_log_path`.
    """

    back_requested = Signal()
    #: v7.20: EVERY PrintState transition of this page's own PrintManager, for an
    #: embedding host. On a terminal state it is emitted AFTER ``_last_log_path``
    #: is captured, so a slot can read it immediately.
    #: ⚠ RE-ENTRANT: ``PrintManager._set_state`` runs on the CALLING thread and
    #: ``pm.start()`` is called from the GUI thread, so a slot can execute inside
    #: ``pm.start()``. Read state there; defer any real work with
    #: ``QTimer.singleShot(0, …)``.
    print_state_changed = Signal(object)
    #: v7.20: the post-print cleanup worker finished — "" = ok, else the reason.
    #: A host must wait for this (or poll :meth:`cleanup_running`) before moving
    #: the stage or trusting where it is: the cleanup travels to waste/wash/oil.
    cleanup_finished = Signal(str)

    # v7.5.x print-setup routine step 4: pump pre-flow lead-in (seconds) — the
    # pump runs at the print flow rate for this long after the confirmed descent
    # to print Z and before the print trajectory begins.
    _PREFLOW_S = 0.25

    # Fallback for the calibrated XY max speed (mm/s) when neither the timing
    # calibration nor the safety limits report one — matches SafetyLimits'
    # default max_xy_speed (10_000 µm/s). Print speed = % × this.
    _XY_MAX_FALLBACK_MM_S = 10.0
    # Fallback for the calibrated Z max speed (mm/s) when no per-axis / safety
    # Z max feedrate is reported. Line-move Z speed = % × this.
    _Z_MAX_FALLBACK_MM_S = 5.0
    # Fallback flow @100% (µL/s) when no needle is configured so its bore
    # cross-section is unknown (can't auto-calculate). Plain prints still flow.
    _FLOW_FALLBACK_UL_S = 0.25

    #: v7.20: combo userData for a host-supplied object. ONE slot — a host
    #: pushes one object at a time — and a STABLE token, which is what lets
    #: ``_refresh_objects``' existing ``findData(prev)`` restore the selection
    #: through a rebuild with no second selection path.
    _EXTERNAL_DATA = "external:0"

    # v7.21 — ids for the Setup column's built-in cards. Stable strings, never
    # ordinals: the stored order is keyed by these, so renumbering when a card is
    # added would silently reshuffle every operator's saved arrangement. Kept
    # distinct from a settings section's slug id (which comes from its title) by
    # the `builtin_` prefix, so a section titled "Queue" cannot collide with the
    # queue card.
    SEC_OBJECT = "builtin_object"
    SEC_QUEUE = "builtin_queue"
    SEC_PARAMS = "builtin_params"
    SEC_READINESS = "builtin_readiness"
    SEC_STATUS = "builtin_status"

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None, *,
                 embedded: bool = False, owns_camera: bool = True,
                 settings_id: str = "quick_print",
                 settings_title: str = "Quick Print"):
        """v7.20 embedding kwargs, for a host that mounts this page inside its
        own shell (the Print Calibrator). All defaults are today's behaviour.

        ``embedded=True`` drops the "← Back to Workflows" button and the page
        title from the header (the host supplies its own) and hides the v7.19
        plate-wide QUEUE surface. The queue is hidden rather than merely
        disabled because it stores the object combo's userData as a *reference*
        (``QueuedPrint.object_data``) and applies a per-well snapshot on every
        plate click — either of which would overwrite, or persist a dangling
        reference to, a host-supplied object. The ⚙ Settings button and the
        summary line are KEPT: they are the only route to pump / flow / ink /
        prep, and the only statement of what will actually run.

        ``owns_camera=False`` cedes the microscope lifecycle to the host, while
        the feed is still BOUND to the right slot here. Load-bearing, and a
        separate flag from ``embedded`` because it is a different fact: Qt
        delivers ``showEvent`` to a CHILD BEFORE ITS PARENT and hides an
        inactive tab's page, so an embedded instance would otherwise win the
        start race, claim the stop, and then kill the host's live view the
        moment the operator looks at another tab. Ownership is declared, not
        won. (Same shape as ``FluorescenceMosaicWorkflowPage``.)

        ``settings_id`` / ``settings_title`` name the
        :class:`WorkflowSettingsStore` directory (``config/workflows/<id>/``).
        Also load-bearing: the settings popout auto-saves ``__last__.json`` on
        hide/close and this page hides its popout on every ``hideEvent``, so two
        live instances sharing ``"quick_print"`` would silently overwrite each
        other's last-used values.
        """
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None
        self._embedded = bool(embedded)
        self._owns_camera = bool(owns_camera)

        # Live camera + trajectory monitor state.
        self._camera_view: CameraFeedView | None = None
        self._camera_started_by_us = False

        # v7.20: caller-supplied objects (the Print Calibrator's calibration
        # lines). A LIST: each entry becomes its own print segment, so
        # build_well_plate_job inserts the lift → hop → lower → re-prime between
        # them — which is what the calibrator's second line measures.
        self._external_objs: list[dict] = []
        self._external_label = "External object"
        self._external_lock = False
        # v7.20: forces the pre-flow prime for a calibration run. None = use the
        # settings-popout value (today's behaviour).
        self._prime_override_s: float | None = None

        # Calibration data (pushed in by MainWindow fanout).
        self._plate = None
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None
        self._z_references: dict[str, float | None] = {
            "replace_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }
        self._printz_seeded = False

        self._selected_well: str | None = None
        self._print_mgr = PrintFileManager()  # for listing/loading saved prints

        # Left context panel — lazy, identical lifecycle to the Jog page.
        self._context_widget: StandardJogContextPanel | None = None

        # Execution state.
        self._pm: Optional[PrintManager] = None
        self._bridge = _PrintBridge()
        self._bridge.progress.connect(self._on_progress)
        self._bridge.state.connect(self._on_state)
        self._bridge.prepositioned.connect(self._on_prepositioned)
        self._bridge.cleanup_done.connect(self._on_cleanup_done)
        self._bridge.multi_done.connect(self._on_multi_done)
        self._bridge.queue_done.connect(self._on_queue_done)
        self._bridge.queue_unit.connect(self._on_queue_unit)
        self._bridge.queue_held.connect(self._on_queue_held)
        self._bridge.predicted.connect(self._on_predicted)
        self._bridge.vel_sample.connect(self._on_vel_sample)
        # v7.6: fast live-position sampling while a print runs (see
        # _set_print_live); idle updates still ride the shared 300 ms app tick.
        self._live_pos_timer = QTimer(self)
        self._live_pos_timer.setInterval(self._LIVE_POS_MS)
        self._live_pos_timer.timeout.connect(self._push_live_position)

        # v7.5.x: multi-ink (abstract-ink) print state. When the loaded object
        # is a sketch that uses ≥2 abstract inks, Quick Print maps each abstract
        # ink → a configured ink and runs sequential ink swaps.
        self._loaded_sketch = None                 # SketchTrajectory.Sketch|None
        self._ink_map: dict[int, str] = {}         # abstract ink id → ink name
        self._ink_map_combos: dict[int, object] = {}
        self._ink_map_last: dict[str, str] = {}    # ink name → mapped, remembered
        self._multi_thread = None
        self._multi_abort_requested = False

        # ── v7.19: the plate-wide print queue ─────────────────────────
        # One QueuedPrint per well, each a COMPLETE set of inputs, so two wells
        # can print different objects with different inks at different heights.
        # Empty ⇒ every existing single-print path behaves exactly as before.
        self._queue: list = []
        self._queue_thread = None
        self._queue_abort_requested = False
        self._queue_results: list[dict] = []
        self._queue_ran = 0
        self._queue_total = 0
        self._queue_swaps = 0
        self._queue_is_held = False
        self._hold_requested = False
        self._hold_event = threading.Event()
        # Guards the write-back sink while a snapshot is being APPLIED to the
        # widgets — without it, applying an entry would write itself back through
        # every intermediate half-applied state.
        self._applying_snapshot = False
        # Display-only polyline cache keyed by CONFIG identity (object + size +
        # resolution), NOT by well: a real queue is usually "the same print in 40
        # wells", so this normally holds one entry. Regenerating geometry per well
        # per repaint would make a 384-well plate unusable.
        self._glyph_cache: dict[tuple, tuple] = {}

        # v7.5.x: pre-position runs off the GUI thread. Holds the print context
        # captured at launch so the confirm-and-run continuation
        # (:meth:`_on_prepositioned`) has everything it needs.
        self._preposition_thread = None
        self._pending_print: dict | None = None
        # v7.5.x: the pre-print preamble (needle prep + ink pickup) runs on the
        # same worker via a PickPlaceExecutor; held so Abort can interrupt it and
        # so the worker outcome (None / "aborted" / "<error>") can gate the
        # continuation in :meth:`_on_prepositioned`.
        self._active_executor: Optional[PickPlaceExecutor] = None
        self._preflight_error: str | None = None
        # Sticky abort request (GUI-thread state, race-free): set by _on_abort
        # while the preflight worker runs, consumed by _on_prepositioned. The
        # executor's _abort_flag is only polled at _check_abort() points, so an
        # Abort during a blocking pump/safe_travel move would otherwise be
        # dropped and the print would start anyway.
        self._preflight_abort_requested: bool = False
        # v7.5.x: post-print cleanup (waste → wash → reset oil). Context is
        # stashed when the print starts and consumed when it COMPLETES; the
        # cleanup runs on its own worker thread.
        self._post_print_ctx: dict | None = None
        self._cleanup_thread = None
        # v7.7: the last progress message (so a terminal state can report WHY
        # it ended instead of a generic line) and the last execution log (so it
        # can be opened rather than merely named).
        self._last_progress_msg: str = ""
        self._last_log_path = None
        # v7.7: the plan's own time estimate / stop count, stashed by
        # _append_limit_warnings so the readiness checklist and the derived-facts
        # line report them without re-planning; and the last evaluated readiness,
        # which is the single source of truth for the Print button.
        self._last_est_s = None
        self._last_stops = None
        self._readiness = None
        self._predicted_p95_um = None
        # v7.7: live-print accumulators (set by _set_print_live, fed by the
        # executor's telemetry, rendered on the 10 Hz timer). None = not printing.
        self._live: dict | None = None
        self._last_path_len_mm = None

        # Comprehensive settings popout (scrollable, saveable). Built eagerly so
        # the config widgets exist for _build_settings() / _on_print() + tests.
        # v7.21: kept because the layout store is keyed by it — the arrangement
        # belongs to the surface the operator sees, and an embedded instance runs
        # under its own id (see _hide_queue_surface).
        self._settings_id = str(settings_id or "quick_print")
        self._layout_store = None
        self._settings_dialog = WorkflowSettingsDialog(
            self._settings_id,
            str(settings_title or "Quick Print"),
            parent=self, on_change=self._on_settings_changed,
            # v7.6: upgrade saved profiles from the old speed_pct knob.
            migrate=self._migrate_legacy_settings,
            # v7.7: every field notifies (debounced), so editing e.g. the
            # pre-flow lead-in or the cleanup margin refreshes the numbers the
            # readiness panel shows. Nine fields previously edited silently.
            notify_on_field_change=True)
        # v7.7: the abstract-ink → configured-ink mapping is rebuilt per object
        # (so it can't be a fixed registered field) but it IS a real operator
        # choice; ride it along with the profile instead of losing it on restart.
        # v7.19: the plate-wide queue rides along too, so a saved profile carries
        # the whole plate layout.
        self._settings_dialog.set_extra_state(
            self._collect_extra_state, self._restore_extra_state)
        self._build_settings_dialog(self._settings_dialog)

        # v7.19: ONE debounced sink that notices "the operator changed the
        # print". The alternative — hooking _on_object_changed, _on_settings_changed
        # and the four inline per-widget lambdas — is four seams, and a fifth
        # would be forgotten the next time a field is added.
        self._writeback_timer = QTimer(self)
        self._writeback_timer.setSingleShot(True)
        self._writeback_timer.setInterval(120)
        self._writeback_timer.timeout.connect(self._write_back)
        for _w in self._snapshot_widgets():
            for _signame in ("valueChanged", "currentIndexChanged"):
                _sig = getattr(_w, _signame, None)
                if _sig is not None:
                    _sig.connect(lambda *_: self._queue_writeback_soon())
                    break

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())
        # v7.7: Setup → Run → Report. The object row and the readiness surface
        # now live inside the Setup zone; the run row stays outside the stack so
        # Print / Pause / Abort and the status line are reachable from any zone.
        outer.addWidget(self._build_zone_strip())
        outer.addWidget(self._build_zones(), stretch=1)

        outer.addWidget(self._build_run_row())

        # v7.21: hand the popout somewhere to move a section TO, then re-apply
        # the stored arrangement. After the layout exists, because promotion
        # reparents cards INTO the stack the setup zone just built.
        self._wire_section_promotion()

        self._refresh_objects()
        self._on_object_changed()
        self._refresh_ink_combo()
        self._refresh_setup_status()
        self._update_button_state()
        self._settings_dialog.load_last()
        self._refresh_setup_status()
        self._update_settings_summary()
        if self._embedded:
            self._hide_queue_surface()

    # ── v7.21: customisable Setup column ──────────────────────────

    def _wire_section_promotion(self) -> None:
        """Register the Setup column as the settings popout's promotion host and
        restore the operator's stored arrangement.

        Best-effort throughout: a page built via ``__new__`` for a partial-page
        test has no stack, and an embedded instance deliberately gets none of
        this (see below), so every step is guarded.
        """
        stack = getattr(self, "_section_stack", None)
        dlg = getattr(self, "_settings_dialog", None)
        if stack is None or dlg is None:
            return
        # ⚠ NOT when embedded. An embedded instance shares this page's class but
        # not its purpose: the host owns the surrounding layout, its own settings
        # id keeps a separate profile, and letting the operator move a section
        # onto an embedded surface would put a card inside someone else's page
        # with no way to see it had happened.
        if self._embedded:
            return
        # The SHARED helper, not a copy: it owns the restore ORDER (promotions
        # first so the cards exist, then the stored order), and that ordering is
        # exactly the kind of thing that drifts between seven hand-written copies.
        self._layout_store = wire_section_promotion(
            self, dlg, stack,
            settings=self._settings, workflow_id=self._settings_id)

    def _hide_queue_surface(self) -> None:
        """v7.20: hide the v7.19 plate-wide QUEUE when embedded.

        Hidden, not disabled, and not removed — every widget stays alive so the
        queue's own methods keep working untouched. The queue is wrong for an
        embedded host in three concrete ways: ``QueuedPrint.object_data`` stores
        the object combo's userData as a REFERENCE (so a host-supplied object
        would persist as a dangling one), the per-well snapshot write-back
        re-applies stored widget values on every plate click (overwriting the
        host's object), and "Apply print to wells" / "Run all queued" would stamp
        a one-off calibration print across the whole plate. Getattr-guarded so
        this survives the queue moving or being renamed.
        """
        for attr in ("_queue_card", "_apply_btn", "_run_all_btn", "_hold_btn"):
            w = getattr(self, attr, None)
            if w is not None:
                try:
                    w.setVisible(False)
                except Exception:
                    pass
        nav = getattr(self, "_navigator", None)
        if nav is not None and hasattr(nav, "set_multi_select_enabled"):
            try:
                nav.set_multi_select_enabled(False)
            except Exception:
                pass

    def showEvent(self, event):
        # Re-scan config/prints each time the page is shown so prints created
        # this session (Sketch, Image Import, manual import) appear without a
        # restart. _refresh_objects preserves the current selection.
        self._refresh_objects()
        self._start_camera()
        self._refresh_planned_path()
        super().showEvent(event)

    def hideEvent(self, event):
        # Stop the live feed when the page is hidden (only if we started it),
        # so we don't keep the microscope camera running in the background.
        self._stop_camera()
        try:
            if self._settings_dialog.isVisible():
                self._settings_dialog.hide()
        except Exception:
            pass
        super().hideEvent(event)

    # ── Live camera (microscope) ──────────────────────────────────

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
        if self._camera_manager is None or self._camera_view is None:
            return
        cam_idx = self._resolve_microscope_cam_idx()
        try:
            # BIND unconditionally: the view must show the right slot even when
            # the host owns start/stop, so this must NOT be skipped by the
            # ownership gate below (v7.20).
            if self._camera_view.cam_idx != cam_idx:
                self._camera_view.set_camera(cam_idx)
            if not self._owns_camera:
                return          # the host starts/stops it — see the ctor doc
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as e:
            logger.debug("Quick Print camera start failed: %s", e)

    def _stop_camera(self) -> None:
        # v7.20: THE load-bearing half of `owns_camera`. A host that stacks this
        # page in a tab gets a hideEvent every time the operator looks at another
        # tab; stopping the microscope there would blank the host's own live view
        # in the middle of a measurement.
        if not self._owns_camera:
            return
        if (self._camera_manager is None or self._camera_view is None
                or not self._camera_started_by_us):
            return
        try:
            self._camera_manager.stop(self._camera_view.cam_idx)
        except Exception as e:
            logger.debug("Quick Print camera stop failed: %s", e)
        finally:
            self._camera_started_by_us = False

    # ── UI construction ───────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        # v7.20: an embedding host owns the Back button and the page title, so
        # showing ours too would give the operator two of each.
        if not self._embedded:
            back = QPushButton("← Back to Workflows")
            back.setCursor(Qt.PointingHandCursor)
            back.clicked.connect(self.back_requested.emit)
            row.addWidget(back)

            title = QLabel("Quick Print")
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

        settings_btn = QPushButton("⚙ Settings")
        settings_btn.setCursor(Qt.PointingHandCursor)
        settings_btn.setToolTip(
            "Open the full, saveable settings for this print (pump, flow, speed, "
            "height, ink, prep, post-print cleanup, locations).")
        settings_btn.clicked.connect(self._open_settings)
        row.addWidget(settings_btn)
        return row

    # ── Settings popout ───────────────────────────────────────────

    def _open_settings(self):
        self._settings_dialog.show()
        self._settings_dialog.raise_()
        self._settings_dialog.activateWindow()

    def _on_settings_changed(self):
        # v7.7: apply the measured stage-max cap here — after any profile load /
        # legacy migration has written its value, so the migration is not
        # pre-clamped (the method was dead code before).
        self._update_top_speed_cap()
        self._refresh_setup_status()
        self._refresh_planned_path()
        self._update_settings_summary()
        self._update_button_state()

    def _update_settings_summary(self):
        if not hasattr(self, "_settings_summary"):
            return
        try:
            ink = self._selected_ink() or "(loaded)"
            prep = "prep on" if self._prep_check.isChecked() else "prep off"
            speed, flow, _prime = self._resolved_print_kinematics()
            self._settings_summary.setText(
                f"{self._pump()} · {speed:.2f} mm/s · {flow:.3g} µL/s "
                f"(×{self._extrusion_modifier():g}) · "
                f"res {self._resolution_um():.0f} µm · ink {ink} · {prep}")
        except Exception:
            # v7.7: was a silent `pass`, which left the PREVIOUS summary on
            # screen with no hint it had gone stale.
            logger.exception("Quick Print settings summary failed")
            self._settings_summary.setText("settings summary unavailable — see log")

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

    def _build_object_row(self) -> QFrame:
        """Inline object + size selection (drives the live trajectory preview)."""
        frame = QFrame(self)
        frame.setObjectName("cfgRow")
        frame.setStyleSheet(
            f"QFrame#cfgRow {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: 6px;"
            f"}}"
        )
        row = QHBoxLayout(frame)
        row.setContentsMargins(s(10), s(8), s(10), s(8))
        row.setSpacing(s(10))

        row.addWidget(QLabel("Object:"))
        self._object_combo = QComboBox()
        self._object_combo.setMinimumWidth(s(180))
        self._object_combo.currentIndexChanged.connect(self._on_object_changed)
        row.addWidget(self._object_combo)

        refresh = QPushButton("⟳")
        refresh.setToolTip("Refresh saved prints")
        refresh.setFixedWidth(s(28))
        refresh.clicked.connect(self._refresh_objects)
        row.addWidget(refresh)

        self._size_label = QLabel("Size:")
        row.addWidget(self._size_label)
        self._size_spin = QDoubleSpinBox()
        self._size_spin.setRange(0.1, 20.0)
        self._size_spin.setDecimals(2)
        self._size_spin.setSingleStep(0.5)
        self._size_spin.setValue(1.0)
        self._size_spin.setSuffix(" mm")
        self._size_spin.valueChanged.connect(
            lambda *_: (self._refresh_planned_path(), self._refresh_setup_status()))
        row.addWidget(self._size_spin)

        row.addStretch(1)

        hint = QLabel("Use ⚙ Settings for pump / flow / ink / prep …")
        hint.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: {sf(9)}pt;")
        row.addWidget(hint)
        return frame

    def _build_queue_card(self) -> QWidget:
        """The queue: a read-only readout plus the actions on it.

        The LIST is deliberately not selectable. The plate is the map, and the
        map is the single answer to "which wells" — a second selection model in a
        list beside it would be two homes for one fact. Rows are keyed by well
        NAME, never by row index.
        """
        card = Card("Queue", compact=True)
        self._queue_list = QListWidget()
        self._queue_list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self._queue_list.setFocusPolicy(Qt.NoFocus)
        self._queue_list.setMaximumHeight(s(150))
        self._queue_list.setStyleSheet(f"font-size: {sf(9)}pt;")
        self._queue_list.itemDoubleClicked.connect(
            lambda it: self._on_well_clicked(str(it.data(Qt.UserRole) or "")))
        card.add_widget(self._queue_list)

        btns = QHBoxLayout()
        btns.setSpacing(s(6))
        self._queue_remove_btn = QPushButton("Remove selected")
        self._queue_remove_btn.setToolTip(
            "Remove the queued prints in the wells selected on the plate. With "
            "nothing selected, removes the well being edited.")
        self._queue_remove_btn.clicked.connect(self._on_queue_remove)
        btns.addWidget(self._queue_remove_btn)
        self._queue_select_btn = QPushButton("Select all queued")
        self._queue_select_btn.setToolTip(
            "Select every well that holds a print, so new parameters can be "
            "stamped over the whole layout in one click.")
        self._queue_select_btn.clicked.connect(
            lambda: self._navigator.set_selected_wells(
                [qp.well for qp in self._queue]))
        btns.addWidget(self._queue_select_btn)
        self._queue_clear_btn = QPushButton("Clear all")
        self._queue_clear_btn.setToolTip("Remove every queued print.")
        self._queue_clear_btn.clicked.connect(self._on_queue_clear)
        btns.addWidget(self._queue_clear_btn)
        btns.addStretch(1)
        card.add_layout(btns)

        opts = QHBoxLayout()
        opts.setSpacing(s(10))
        self._group_by_ink_check = QCheckBox("Group by ink")
        self._group_by_ink_check.setToolTip(
            "Run every well using one ink before switching, instead of in plate "
            "order. Each ink swap is a full waste → wash → buffer cycle, so this "
            "can save a lot of time — but it changes how long each well sits "
            "before the next handling step.")
        self._group_by_ink_check.toggled.connect(
            lambda *_: self._refresh_queue_ui())
        opts.addWidget(self._group_by_ink_check)
        self._clean_between_check = QCheckBox("Clean between every print")
        self._clean_between_check.setToolTip(
            "Wash the needle between every queued print, not just when the ink "
            "changes. Slower, but it resets the syringe each time — the remedy "
            "when a long same-ink batch will not fit the plunger envelope.")
        self._clean_between_check.toggled.connect(
            lambda *_: self._refresh_queue_ui())
        opts.addWidget(self._clean_between_check)
        opts.addStretch(1)
        card.add_layout(opts)

        note = QLabel(
            "Each queued print keeps its own object, size, pump, ink, speed, "
            "resolution, height and extrusion ×. Prep / cleanup / wash and the "
            "between-lines settings are shared by the whole run.")
        note.setWordWrap(True)
        note.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: {sf(8)}pt;")
        card.add_widget(note)
        self._queue_card = card
        return card

    def _build_status_strip(self) -> QFrame:
        """The 'confirm all is setup' readiness surface, kept on the page so the
        operator sees it without opening the settings popout."""
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(s(2), 0, s(2), 0)
        row.setSpacing(s(8))
        self._setup_status = QLabel("")
        self._setup_status.setWordWrap(True)
        self._setup_status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        self._setup_status.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._setup_status, stretch=1)
        return frame

    def _on_prep_toggled(self, on: bool):
        # Service Z + Wash cycles are shared with the post-print cleanup, so they
        # stay enabled regardless of the prep checkbox.
        self._refresh_setup_status()
        self._update_settings_summary()

    def _build_settings_dialog(self, dlg: WorkflowSettingsDialog):
        # ── Print (pump / flow / speed / height) ──
        self._pump_combo = QComboBox()
        self._pump_combo.setMinimumWidth(s(90))
        self._pump_combo.addItem("P1")
        self._pump_combo.currentIndexChanged.connect(self._on_pump_changed)
        # v7.5.x: the flow @100% is AUTO-calculated from the needle bore and the
        # print length (see _auto_flow_100_uL_s); the operator tunes line
        # thickness with this extrusion modifier instead of a raw flow value.
        self._extrusion_mod_spin = self._dspin(
            0.1, 10.0, 1.0, " ×", 2, 0.1,
            "Extrusion modifier on the auto-calculated flow. 1.0 = deposit a "
            "bead equal to the needle bore cross-section along the path; >1 = a "
            "thicker line. Scales the pump flow AND the ink pickup volume.")
        self._extrusion_mod_spin.valueChanged.connect(
            lambda *_: (self._refresh_setup_status(), self._update_settings_summary()))
        # ── v7.6: THE TWO PARAMETERS everything else derives from ──
        #
        # Accuracy through a corner is bought with TIME (the feed plan stops on
        # sharp corners and slows through curvature), so the operator states the
        # trade directly: how fine must it be, and how fast may the stage go.
        # Every other motion number — per-section lookahead, per-section speed,
        # pump flow, corner stops, the time estimate — is derived from these.
        self._top_speed_spin = self._dspin(
            0.1, 20.0, 2.5, " mm/s", 2, 0.5,
            "Top XY speed for this print. The run uses min(this, the measured "
            "stage maximum, the needle's flow-limited maximum) — you are told "
            "which one binds. Straights run at this speed; corners and tight "
            "curves slow only as much as the resolution demands. Pump flow "
            "follows automatically, so the bead stays bore-area × modifier per "
            "mm at any speed.")
        self._top_speed_spin.valueChanged.connect(
            lambda *_: (self._refresh_setup_status(),
                        self._update_settings_summary(),
                        self._refresh_planned_path()))
        try:
            _default_res = float(get_store().get_resolution_element_um() or 30.0)
        except Exception:
            _default_res = 30.0
        self._resolution_spin = self._dspin(
            5.0, 500.0, _default_res, " µm", 0, 5.0,
            "Resolution element: the feature size this print must hold. Drives "
            "the corner-stop planning and the per-section lookahead — a finer "
            "element means more corner stops and slower curves, so it COSTS "
            "TIME (the estimate below shows how much). Asking for finer than "
            "the machine's own stopping accuracy is flagged.")
        self._resolution_spin.valueChanged.connect(
            lambda *_: (self._refresh_setup_status(),
                        self._update_settings_summary(),
                        self._refresh_planned_path()))
        self._printz_spin = self._dspin(
            0.0, 40.0, 0.2, " mm", 2, 0.1,
            "Print height measured up from the calibrated plate bottom. 0 = at "
            "the plate bottom; larger = higher. Clamped so it never goes below.")
        self._printz_spin.valueChanged.connect(
            lambda *_: self._refresh_setup_status())
        # v7.5.x: print-path MOTION MODE (A/B on hardware). The XY trajectory
        # during ink laydown can be driven three ways — pick per-run:
        #   • open_loop — the ORIGINAL streamed path (each segment fire-and-forget,
        #     paced by a timed sleep). Correct when the stage tracks the commanded
        #     speed; on a stage that runs slower than commanded it lags/smears.
        #   • velocity  — CLOSED-LOOP: polls real position + re-commands a velocity
        #     vector toward a carrot ahead of real progress (immune to a wrong
        #     speed calibration; needs a continuous-velocity stage, e.g. Prior VS).
        #   • confirm   — stop-and-go: wait for arrival + drain the pump each
        #     segment. Correct but slow.
        self._motion_mode_combo = QComboBox()
        self._motion_mode_combo.setMinimumWidth(s(150))
        # v7.7: closed-loop velocity FIRST and default. Open-loop reads no
        # position at all, so it has no prediction, no live deviation and no
        # deviation in the report — every information surface added in v7.7 is
        # inert in it.
        self._motion_mode_combo.addItem("Velocity (closed-loop) — recommended",
                                        "velocity")
        self._motion_mode_combo.addItem("Open-loop velocity (streamed vectors)", "open_loop")
        self._motion_mode_combo.addItem("Confirmed per-segment (point-to-point)", "confirm")
        self._motion_mode_combo.setToolTip(
            "How the XY stage traces the print path.\n"
            "Open-loop velocity = stream a continuous velocity vector along the "
            "path (feed-forward, no position reads; smooth/continuous motion, "
            "but no drift correction).\n"
            "Velocity = closed-loop position feedback (same continuous motion, "
            "plus it polls position and corrects drift; robust to a wrong speed "
            "calibration).\n"
            "Confirmed = move to each point and wait for arrival (accurate but "
            "stop-and-go).")
        # v7.5.x: the trajectory monitor overlays a simulated prediction in
        # velocity mode — refresh it when the mode changes.
        self._motion_mode_combo.currentIndexChanged.connect(
            lambda _i: self._refresh_planned_path())
        sec = dlg.add_section("Print")
        sec.add("pump", "Pump / bore", self._pump_combo, "P1")
        sec.add("extrusion_mod", "Extrusion modifier (×)",
                self._extrusion_mod_spin, 1.0)
        # v7.7: the two DRIVING parameters are promoted onto the Setup zone —
        # they are the whole operator interface to the accuracy/time trade, and
        # burying them in a popout made that trade invisible. They are still
        # persisted with the profile via register_external (a widget has one
        # parent, so it cannot also be laid out in this section).
        dlg.register_external("top_speed", self._top_speed_spin, 2.5)
        dlg.register_external("resolution", self._resolution_spin,
                              self._resolution_spin.value())
        sec.add_note("Top XY speed and Resolution are on the Setup zone of the "
                     "page — they drive everything else and are saved with this "
                     "profile.")
        sec.add("printz", "Height above bottom", self._printz_spin, 0.2)
        sec.add("motion_mode", "Motion mode", self._motion_mode_combo, "velocity")

        # ── Ink ──
        self._ink_combo = QComboBox()
        self._ink_combo.setMinimumWidth(s(180))
        self._ink_combo.addItem("(none — needle already loaded)", "")
        self._ink_combo.setToolTip(
            "Which ink this print uses (library inks with a reagent location). "
            "The needle picks it up from that well before printing; default is "
            "the pump's assigned ink, or “(none)” to use whatever is loaded.")
        self._ink_combo.currentIndexChanged.connect(
            lambda *_: (self._refresh_setup_status(), self._update_settings_summary()))
        self._ink_z_spin = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom when aspirating ink.")
        self._ink_padding_spin = self._dspin(
            0.0, 100.0, 0.0, " µL", 2, 0.1,
            "Extra ink aspirated beyond the computed print volume so the needle "
            "never runs dry (you don't dispense the very last of the ink). "
            "Added to the pickup; flushed to waste in the post-print reset.")
        self._ink_padding_spin.valueChanged.connect(
            lambda *_: self._refresh_setup_status())
        # ── Tip prime (aspirate extra, dispense back) ──
        # Aspirate an EXTRA prime volume beyond the print pickup, then dispense
        # that same amount back into the ink well. This advances ink to the very
        # tip and purges the air gap so the ink is ready to deposit; net retained
        # volume is unchanged. When on, the pickup runs WITHOUT compliance
        # compensation (the compliance is primed by this dispense-back).
        self._ink_prime_check = QCheckBox(
            "Prime tip (aspirate extra, then dispense back into the well)")
        self._ink_prime_check.setChecked(False)
        self._ink_prime_check.setToolTip(
            "Before printing: aspirate an extra volume, then dispense the same "
            "amount back into the ink well. Advances ink to the tip and purges "
            "the air gap so the ink is ready to deposit. While active, the ink "
            "pickup skips backlash/compliance compensation (it is already primed "
            "by the dispense-back).")
        self._ink_prime_check.toggled.connect(
            lambda *_: (self._refresh_setup_status(),
                        self._update_settings_summary()))
        self._ink_prime_spin = self._dspin(
            0.0, 100.0, 2.0, " µL", 2, 0.1,
            "Extra volume aspirated then dispensed back into the ink well to "
            "prime the tip. Net retained volume is unchanged.")
        # ── Granular anti-clog circular pickup ──
        self._orbit_check = QCheckBox("Circular pickup for granular inks")
        self._orbit_check.setChecked(True)
        self._orbit_check.setToolTip(
            "Orbit the needle in a small circle while aspirating an ink whose "
            "subtype is 'granular material', so the granules behave more "
            "fluid-like and don't clog the bore.")
        self._orbit_all_check = QCheckBox("Force circular pickup for all inks")
        self._orbit_all_check.setChecked(False)
        self._orbit_all_check.setToolTip(
            "Apply the circular pickup orbit to every ink, not just granular ones.")
        self._orbit_dia_spin = self._dspin(
            0.1, 10.0, 1.0, " mm", 2, 0.1,
            "Diameter of the circle the needle orbits while picking up.")
        self._orbit_speed_spin = self._dspin(
            0.1, 50.0, 2.0, " mm/s", 1, 0.5,
            "Tangential speed of the needle along the pickup orbit circle.")
        sec = dlg.add_section("Ink pickup")
        sec.add("ink", "Ink", self._ink_combo, "")
        sec.add("ink_z", "Ink dip Z (↑ bottom)", self._ink_z_spin, 0.50)
        sec.add("ink_padding", "Ink padding (µL)", self._ink_padding_spin, 0.0)
        sec.add_check("ink_prime", self._ink_prime_check, False)
        sec.add("ink_prime_uL", "Prime volume (µL)", self._ink_prime_spin, 2.0)
        sec.add_check("orbit_granular", self._orbit_check, True)
        sec.add_check("orbit_all", self._orbit_all_check, False)
        sec.add("orbit_dia", "Orbit diameter (mm)", self._orbit_dia_spin, 1.0)
        sec.add("orbit_speed", "Orbit speed (mm/s)", self._orbit_speed_spin, 2.0)

        # ── Ink mapping (multi-ink sketch) ──
        # When the loaded print is a sketch that uses ≥2 abstract inks, map each
        # abstract ink → a configured ink. The run does sequential ink swaps
        # (print → waste → wash → pick up next ink → print). The rows are
        # rebuilt per object (session state, not persisted).
        sec = dlg.add_section("Ink mapping (multi-ink sketch)")
        sec.add_note(
            "For a sketch that uses more than one abstract ink, map each "
            "abstract ink to a configured ink here. The print runs with "
            "sequential ink swaps between them.")
        self._ink_map_container = QWidget()
        self._ink_map_layout = QVBoxLayout(self._ink_map_container)
        self._ink_map_layout.setContentsMargins(0, 0, 0, 0)
        self._ink_map_layout.setSpacing(s(4))
        sec.add_widget(self._ink_map_container)

        # ── Needle prep ──
        self._prep_check = QCheckBox("Prep needle (waste → oil → wash → buffer)")
        self._prep_check.setChecked(True)
        self._prep_check.setToolTip(
            "Before picking up ink: dispense 1 needle of oil to waste, aspirate "
            "fresh oil, wash, then aspirate buffer. Service wells from Hardware "
            "Setup → Ink.")
        self._prep_check.toggled.connect(self._on_prep_toggled)
        self._service_z_spin = self._dspin(
            0.0, 30.0, 0.50, " mm", 2, 0.1,
            "Needle dip height above the plate bottom at the service wells.")
        self._buffer_needles_spin = self._dspin(
            0.0, 20.0, 1.0, "", 1, 0.5,
            "Needles of buffer aspirated after the wash during prep.")
        self._wash_cycles_spin = self._ispin(
            0, 50, 3, "Dip-jiggle cycles at the wash well.")
        sec = dlg.add_section("Needle prep")
        sec.add_check("prep", self._prep_check, True)
        sec.add_note(
            "Prep values are shared defaults from Common Print Settings — tick "
            "Override to set a workflow-specific value.")
        sec.add_common("service_z", "Service dip Z (↑ bottom)", self._service_z_spin, 0.50)
        sec.add_common("buffer_needles", "Buffer (needles)", self._buffer_needles_spin, 1.0)
        sec.add_common("wash_cycles", "Wash cycles", self._wash_cycles_spin, 3)

        # ── Between print lines (lift between strokes / sub-paths) ──
        self._line_retract_spin = self._dspin(
            0.0, 40.0, 1.0, " mm", 2, 0.1,
            "Height the needle lifts above the print Z between separate strokes "
            "(sub-paths / objects) before travelling to the next one. A "
            "continuous fill prints as one stroke and is not lifted mid-fill.")
        self._line_z_speed_spin = self._dspin(
            1.0, 100.0, 100.0, " % max", 0, 5.0,
            "Quick-move Z speed for the inter-line lift + lower, as a % of the "
            "Z stage's calibrated max feedrate.")
        self._line_xy_speed_spin = self._dspin(
            1.0, 100.0, 100.0, " % max", 0, 5.0,
            "Quick-move XY speed for the inter-line travel, as a % of the XY "
            "stage's calibrated max speed.")
        sec = dlg.add_section("Between print lines")
        sec.add("line_retract", "Retract after each line",
                self._line_retract_spin, 1.0)
        sec.add("line_z_speed", "Line-move Z speed (% max)",
                self._line_z_speed_spin, 100.0)
        sec.add("line_xy_speed", "Line-move XY speed (% max)",
                self._line_xy_speed_spin, 100.0)

        # ── Post-print cleanup ──
        self._postclean_check = QCheckBox(
            "Reset syringe to initial condition after print")
        self._postclean_check.setChecked(True)
        self._postclean_check.setToolTip(
            "After the print: dispense the unprinted ink + buffer (computed live "
            "from the plunger position) plus a small oil flush margin to waste, "
            "wash, then top the oil back up so the plunger returns to its pre-run "
            "(initial) position — the syringe ends as it started.")
        self._postclean_check.toggled.connect(
            lambda *_: (self._refresh_setup_status(), self._update_settings_summary()))
        self._postclean_margin_spin = self._dspin(
            0.0, 50.0, 1.0, "", 1, 0.5,
            "Small extra oil (× needle) flushed past the tip to expel the last "
            "of the ink/buffer, then re-aspirated so the plunger returns to its "
            "initial position. 0 = waste exactly the unprinted volume.")
        sec = dlg.add_section("Post-print cleanup")
        sec.add_check("postclean", self._postclean_check, True)
        sec.add("postclean_margin", "Oil flush margin (× needle)",
                self._postclean_margin_spin, 1.0)

        # ── Advanced ──
        self._travel_speed = self._dspin(
            0.5, 100.0, 10.0, " mm/s", 1, 1.0,
            "XY speed for non-printing travel moves (approach / between objects).")
        self._preflow = self._dspin(
            0.0, 5.0, self._prime_default_s(), " s", 2, 0.05,
            "Pump pre-flow lead-in: the pump runs at the print flow for this long "
            "after the descent to print Z and before the path starts. Seeded "
            "from Hardware Setup → Pump (Prime time).")
        sec = dlg.add_section("Advanced")
        sec.add("travel_speed", "Travel speed", self._travel_speed, 10.0)
        # v7.20: LINKED to the global prime time, not merely seeded from it.
        # It was `sec.add(...)`, seeded once at build time — when `_hw_config` is
        # still None, so it took the 0.25 s fallback — and then restored from the
        # saved profile on top. So a change to `pump_prime_time_s` (Hardware
        # Setup → Pump, the Common Print Settings page, or the Print Calibrator
        # applying a MEASURED prime time) reached every workflow that links it
        # and silently did NOT reach Quick Print: the calibration would appear to
        # succeed and change nothing. `overridable=True` keeps the per-run knob
        # the comment below describes — unchecked INHERITS the global live
        # (the widget is force-set, so `_preflow_s()`'s read is already the
        # effective value), checked keeps a local value — and the dialog's
        # `_migrate_common_links` promotes an existing customised profile value
        # to an explicit override, so saved profiles keep their prime.
        sec.add_common("preflow", "Pre-flow lead-in", self._preflow,
                       self._prime_default_s(),
                       common_key="pump_prime_time_s", overridable=True)

        # ── Common — Pump (global) ──
        # (Prime time is linked above via the per-run Pre-flow knob — so only
        # settle here. Pressure relief / compliance is now per-pump µL on the
        # Common Print Settings page, not a global proxied here.)
        self._g_settle = self._dspin(0.0, 30.0, 0.0, " s", 2, 0.05)
        sec = dlg.add_section("Common — Pump (global, shared by all workflows)")
        sec.add_note(
            "Global pump values (edited here or on the Common Print Settings "
            "page — one value used everywhere).")
        sec.add_common("g_settle", "Dwell after syringe moves", self._g_settle,
                       0.0, common_key="pump_settle_time_s", overridable=False)
        # v7.21.7: hold the needle in the liquid after a reagent aspirate. A
        # SEPARATE knob from the settle dwell above, because they cover different
        # halves of the same move: the settle dwell brackets a pump move that
        # already blocks until the PLUNGER has drained from Marlin's planner,
        # while this one covers the FLUID still being drawn in afterwards through
        # a compliant column. Lift Z inside that window and the tail of the
        # aspirate is air.
        self._g_hold_liquid = self._dspin(0.0, 120.0, 2.0, " s", 2, 0.5)
        self._g_hold_liquid.setToolTip(
            "Extra time the needle stays IN THE LIQUID after a reagent aspirate "
            "(ink / oil / buffer) finishes, before Z retracts and the stage "
            "travels on. Raise it if a pickup ends with air drawn into the "
            "needle; 0 = no extra hold.")
        sec.add_common("g_hold_liquid", "Hold in liquid after aspirating",
                       self._g_hold_liquid, 2.0,
                       common_key="pump_post_aspirate_dwell_s",
                       overridable=False)

        # ── Locations & Hardware (read-only) ──
        dlg.add_info_section()
        dlg.set_info_refresher(self._build_locations_panel)
        dlg.finalize()

    def _build_locations_panel(self):
        extras = []
        try:
            ink = self._selected_ink()
            well = self._ink_source_well()
            if ink:
                extras.append(("Print ink", f"{ink} ← {well or '(no well)'}"))
        except Exception:
            pass
        return build_locations_widget(
            self._controller, self._hw_config, self._well_positions,
            z_references=self._z_references, safe_z=self._safe_z, extras=extras)

    def _prime_default_s(self) -> float:
        """Default pre-flow / prime time (s) — from Hardware Setup → Pump
        (``pump_prime_time_s``), so the hardware page's prime time ports into
        this workflow. Falls back to the legacy constant."""
        hw = getattr(self, "_hw_config", None)
        try:
            v = float(getattr(hw, "pump_prime_time_s", self._PREFLOW_S))
            return v if v >= 0 else self._PREFLOW_S
        except (TypeError, ValueError):
            return self._PREFLOW_S

    def _preflow_s(self) -> float:
        # v7.20: a host override wins over the popout. THE single choke point —
        # `_resolved_print_kinematics` is the only consumer and it is what
        # `_build_settings` stamps into `prime_amounts_uL`, so forcing it here
        # forces the prime everywhere it matters (the job's EXTRUDE prime, the
        # pickup volume, the syringe budget and the readiness numbers).
        ov = getattr(self, "_prime_override_s", None)
        if ov is not None:
            try:
                return max(0.0, float(ov))
            except (TypeError, ValueError):
                pass
        w = getattr(self, "_preflow", None)
        if w is None:
            return self._prime_default_s()
        try:
            return float(w.value())
        except Exception:
            return self._prime_default_s()

    _OVERLAY_PILLS = (("none", "None"), ("speed", "XY speed"),
                      ("flow", "Flow"), ("error", "Error"), ("time", "Time"))

    def _build_overlay_pills(self) -> QWidget:
        """v7.6: exclusive pills selecting what the SIMULATED path is coloured
        by. View-only (not a saved setting) — the data comes from the same
        prediction the monitor already shows."""
        row = QWidget()
        lay = QHBoxLayout(row)
        lay.setContentsMargins(s(6), 0, s(6), s(2))
        lay.setSpacing(s(4))
        lbl = QLabel("Overlay:")
        lbl.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt;")
        lay.addWidget(lbl)
        self._overlay_group = QButtonGroup(row)
        self._overlay_group.setExclusive(True)
        for key, text in self._OVERLAY_PILLS:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setChecked(key == "none")
            b._overlay_key = key
            b.setStyleSheet(f"font-size: {sf(8)}pt; padding: {s(2)}px {s(7)}px;")
            self._overlay_group.addButton(b)
            lay.addWidget(b)
        self._overlay_group.buttonClicked.connect(
            lambda _b: self._apply_overlay_mode())
        lay.addStretch(1)
        row.setToolTip(
            "Colour the predicted stage path by XY speed, pump flow rate, "
            "predicted tracking error, or elapsed time — so you can see WHERE "
            "the plan slows down and why.")
        return row

    # ── v7.7: three zones — Setup → Run → Report ──────────────────
    #
    # The page used to be one flat surface where the four inline controls, a
    # dense warning string and the live monitor all competed, while the
    # parameters that drive the print sat in a popout and a finished print left
    # nothing behind. The zones follow what the operator actually does: decide
    # what to print and confirm the machine can do it, watch it happen, then
    # read what happened. Widgets are PROMOTED and REGROUPED, not rewritten —
    # every attribute name is unchanged so the partial-page tests still hold.
    _ZONES = (("setup", "1 · Setup"), ("run", "2 · Run"),
              ("report", "3 · Report"))

    def _build_zone_strip(self) -> QWidget:
        row = QWidget()
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(4))
        self._zone_group = QButtonGroup(row)
        self._zone_group.setExclusive(True)
        self._zone_buttons: dict[str, QPushButton] = {}
        for key, text in self._ZONES:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setChecked(key == "setup")
            b.setCursor(Qt.PointingHandCursor)
            b._zone_key = key
            b.setStyleSheet(f"font-size: {sf(9)}pt; padding: {s(3)}px {s(12)}px;")
            self._zone_group.addButton(b)
            self._zone_buttons[key] = b
            lay.addWidget(b)
        self._zone_group.buttonClicked.connect(
            lambda b: self.show_zone(getattr(b, "_zone_key", "setup")))
        lay.addStretch(1)
        return row

    def show_zone(self, key: str) -> None:
        """Switch zones. Safe to call before the UI exists (tests build partials)."""
        stack = getattr(self, "_zone_stack", None)
        if stack is None:
            return
        order = [k for k, _ in self._ZONES]
        if key not in order:
            return
        stack.setCurrentIndex(order.index(key))
        btn = getattr(self, "_zone_buttons", {}).get(key)
        if btn is not None and not btn.isChecked():
            btn.setChecked(True)
        # The camera and the plan preview only matter in Run; the report only
        # needs building when it is looked at.
        if key == "run":
            self._refresh_planned_path()

    def current_zone(self) -> str:
        stack = getattr(self, "_zone_stack", None)
        if stack is None:
            return "setup"
        order = [k for k, _ in self._ZONES]
        idx = stack.currentIndex()
        return order[idx] if 0 <= idx < len(order) else "setup"

    def _build_zones(self) -> QWidget:
        self._zone_stack = QStackedWidget()
        self._zone_stack.addWidget(self._build_setup_zone())
        self._zone_stack.addWidget(self._build_run_zone())
        self._report_panel = QuickPrintReportPanel()
        self._zone_stack.addWidget(self._report_panel)
        return self._zone_stack

    def _build_setup_zone(self) -> QWidget:
        """What to print, where, and whether the machine can honour it."""
        # v7.21: the left column is a SectionStack, so the operator can order
        # these cards — and interleave sections moved out of the ⚙ popout — into
        # whatever sequence matches how they actually work. The default order is
        # the pre-v7.21 one, and with nothing promoted and nothing reordered the
        # column renders exactly as it did.
        #
        # The built-in cards are ordinary stack entries: the alternative (a fixed
        # block plus a reorderable tail) would mean two placement rules for one
        # column, and "move Readiness above Queue" — the operator's own example
        # of a customisable workflow — would be impossible.
        # Persistence + the ↩ handler are attached later by
        # wire_section_promotion, the one place that knows the restore order.
        self._section_stack = SectionStack()

        # The two driving parameters, in front of the operator at last.
        param_card = Card("Print parameters — everything derives from these")
        param_card.add_widget(FormRow(
            "Top XY speed", self._top_speed_spin,
            help_text=self._top_speed_spin.toolTip()))
        param_card.add_widget(FormRow(
            "Resolution", self._resolution_spin,
            help_text=self._resolution_spin.toolTip()))
        self._derived_lbl = QLabel("")
        self._derived_lbl.setWordWrap(True)
        self._derived_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        param_card.add_widget(self._derived_lbl)

        # Readiness checklist (Stage 1b fills it; the legacy one-line strip is
        # kept underneath as the compact summary and for existing tests).
        self._ready_card = Card("Readiness")
        self._ready_host = QWidget()
        self._ready_layout = QVBoxLayout(self._ready_host)
        self._ready_layout.setContentsMargins(0, 0, 0, 0)
        self._ready_layout.setSpacing(s(3))
        self._ready_card.add_widget(self._ready_host)

        # v7.19: the queue sits with the config it describes, and reads in the
        # operator's own DEFAULT sequence — what to print → what's queued → the
        # parameters that get stamped → readiness. It goes in the left column
        # because that is the one surface here (a QScrollArea) that absorbs an
        # arbitrarily long list without fighting the plate for height.
        for sid, label, w in (
                (self.SEC_OBJECT, "Object", self._build_object_row()),
                (self.SEC_QUEUE, "Queue", self._build_queue_card()),
                (self.SEC_PARAMS, "Print parameters", param_card),
                (self.SEC_READINESS, "Readiness", self._ready_card),
                (self.SEC_STATUS, "Setup status", self._build_status_strip())):
            self._section_stack.add(sid, w, label=label)
        self._builtin_section_ids = [
            self.SEC_OBJECT, self.SEC_QUEUE, self.SEC_PARAMS,
            self.SEC_READINESS, self.SEC_STATUS]

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(s(8))
        ll.addWidget(self._section_stack)
        ll.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setWidget(left)

        # ── v7.19 right pane: the ACTIVE print's preview ABOVE the plate that
        # holds the queue. A SECOND PrintTrajectoryMonitorView, not a new widget
        # and not WellPreviewWidget: this one already consumes exactly what
        # _refresh_planned_path computes (zero-ref µm segments + a well-boundary
        # circle + the needle size), so no second copy of the well-centre →
        # zero-ref conversion has to exist. That conversion lives in
        # _well_center_zero_ref_mm, which handles calibrated-vs-geometric and
        # plate_axis_sign(); a copy would get the sign wrong exactly once, on
        # hardware. WellPreviewWidget would also add a second coordinate
        # convention (well-relative mm, Y UP) to one page.
        self._setup_preview = PrintTrajectoryMonitorView()
        # Per-INSTANCE minimum: the class default is shared with the Run zone's
        # monitor, so lowering it there would shrink that one too. This view
        # auto-fits, so it degrades gracefully to any size.
        self._setup_preview.setMinimumSize(s(180), s(140))
        if self._controller is not None and hasattr(
                self._controller, "plate_flip_180"):
            self._setup_preview.set_plate_flip_180(
                self._controller.plate_flip_180())
        preview_card = Card("Print preview — the ACTIVE print", flush=True,
                            compact=True)
        preview_card.add_widget(self._setup_preview)
        preview_card.add_layout(self._build_apply_row())

        # Where it prints — selection belongs with the rest of setup.
        self._navigator = WellPlateNavigator()
        # v7.7: the widget's default tooltip says "click to fast-travel", which
        # is true on the Jog page but not here — a click only SELECTS the well
        # the object will print in; nothing moves until Print.
        self._navigator.setToolTip(
            "Click a well to choose where the object prints, or to edit the "
            "print already queued there. Drag across wells (or Ctrl-click) to "
            "select several, then use “Apply print to wells”. This does not "
            "move the stage.")
        self._navigator.set_multi_select_enabled(True)
        self._navigator.well_clicked.connect(self._on_well_clicked)
        self._navigator.selection_changed.connect(
            self._on_well_selection_changed)
        nav_card = Card("Plate queue — click to edit · drag to select",
                        flush=True, compact=True)
        nav_card.add_widget(self._navigator)
        nav_card.add_widget(self._build_plate_legend())

        right = QSplitter(Qt.Vertical)
        right.setChildrenCollapsible(False)
        right.addWidget(preview_card)
        right.addWidget(nav_card)
        # 2:3 in favour of the plate: the plate must stay CLICKABLE (a 384-well
        # plate is already at WellPlateNavigator's radius floor), while the
        # preview auto-fits whatever it is given.
        right.setStretchFactor(0, 2)
        right.setStretchFactor(1, 3)
        right.setSizes([s(300), s(430)])
        self._setup_right_split = right

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)
        split.addWidget(scroll)
        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([s(520), s(420)])
        return split

    def _build_apply_row(self) -> QHBoxLayout:
        """The queue count + "Apply print to wells", right-aligned under the
        preview.

        A footer ROW, not a floating overlay child: the monitor view paints its
        prediction caption along the bottom — the very text an operator needs
        before stamping a print into forty wells — and an overlay child would
        occlude it, would need a subclass or an event filter to hook
        ``resizeEvent``, and contributes nothing to ``sizeHint`` so it would clip
        on a narrow pane instead of the layout yielding.
        """
        row = QHBoxLayout()
        row.setContentsMargins(s(10), s(4), s(10), s(6))
        row.setSpacing(s(8))
        self._queue_count_lbl = QLabel("")
        self._queue_count_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._queue_count_lbl)
        row.addStretch(1)
        self._apply_btn = QPushButton("Apply print to wells")
        self._apply_btn.setCursor(Qt.PointingHandCursor)
        self._apply_btn.setEnabled(False)
        self._apply_btn.clicked.connect(self._on_apply_print_to_wells)
        row.addWidget(self._apply_btn)
        return row

    def _build_plate_legend(self) -> QWidget:
        """Four colours is three too many to learn by experiment."""
        lbl = QLabel(
            "<span style='color:#94e2d5'>●</span> queued &nbsp;"
            "<span style='color:#cba6f7'>◯</span> selected &nbsp;"
            "<span style='color:#89b4fa'>◉</span> editing &nbsp;"
            "<span style='color:#f38ba8'>●</span> can't run")
        lbl.setStyleSheet(f"font-size: {sf(8)}pt; padding: 0 {s(10)}px "
                          f"{s(4)}px {s(10)}px;")
        return lbl

    def _build_run_zone(self) -> QWidget:
        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(s(8))
        lay.addWidget(self._build_main_area(), stretch=1)
        lay.addWidget(self._build_run_info())
        return holder

    def _build_run_info(self) -> QWidget:
        """Live numbers during the print (populated in Stage 3b from the
        executor's ~5 Hz telemetry)."""
        card = Card("Live", compact=True)
        self._run_info_lbl = QLabel("Not printing.")
        self._run_info_lbl.setWordWrap(True)
        self._run_info_lbl.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
            f"font-family: monospace;")
        card.add_widget(self._run_info_lbl)
        self._run_info_card = card
        return card

    def _build_main_area(self) -> QSplitter:
        """Horizontal splitter: [trajectory monitor | live camera].

        v7.7: the well selector moved to the Setup zone (choosing a well is a
        setup action), so this is the WATCH surface only.
        """
        # ── Top row: trajectory monitor + live camera ──────────────
        self._traj_view = PrintTrajectoryMonitorView()
        if self._controller is not None and hasattr(
                self._controller, "plate_flip_180"):
            self._traj_view.set_plate_flip_180(
                self._controller.plate_flip_180())
        traj_card = Card("Print plan — planned path · live trace · needle",
                         flush=True, compact=True)
        traj_card.add_widget(self._traj_view)
        traj_card.add_widget(self._build_overlay_pills())

        if self._camera_manager is not None:
            self._camera_view = CameraFeedView(
                camera_manager=self._camera_manager,
                cam_idx=self._resolve_microscope_cam_idx(),
                show_crosshair=True,
                auto_orient=True,   # v7.5.x: calibrated orientation everywhere
                label="Microscope feed — starts on this page",
            )
            cam_widget: QWidget = self._camera_view
        else:
            placeholder = QLabel("No camera manager available.")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet(f"color: {COLORS['overlay0']};")
            cam_widget = placeholder
        cam_card = Card("Live camera", flush=True, compact=True)
        cam_card.add_widget(cam_widget)

        top_split = QSplitter(Qt.Horizontal)
        top_split.setChildrenCollapsible(False)
        top_split.addWidget(traj_card)
        top_split.addWidget(cam_card)
        top_split.setStretchFactor(0, 3)
        top_split.setStretchFactor(1, 2)
        top_split.setSizes([s(560), s(440)])
        return top_split

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))

        # v7.19: NOT renamed — `_print_btn` is in the attribute contract every
        # partial-page suite reads. Only its tooltip gains the "active print"
        # wording (see _update_button_state), beside the new Run-all.
        self._print_btn = QPushButton("Print")
        self._print_btn.clicked.connect(self._on_print)
        row.addWidget(self._print_btn)

        # v7.19: run the whole queue. Lives HERE, outside the zone stack, so it
        # is reachable from the Run zone — which is where the operator is when
        # they want to know whether to keep going.
        self._run_all_btn = QPushButton("Run all queued")
        self._run_all_btn.setEnabled(False)
        self._run_all_btn.clicked.connect(self._on_run_all)
        row.addWidget(self._run_all_btn)

        # v7.19: park the batch at a well boundary. The needle retracts to safe Z
        # and the run holds nothing, so the operator can go image the plate.
        self._hold_btn = QPushButton("Hold after well")
        self._hold_btn.setEnabled(False)
        self._hold_btn.clicked.connect(self._on_hold_clicked)
        row.addWidget(self._hold_btn)

        # v7.7: PrintManager.pause()/resume() existed but were unreachable.
        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setEnabled(False)
        self._pause_btn.setToolTip(
            "Pause after the current command completes. The needle stays where "
            "it is — use Abort if you need motion to stop now.")
        self._pause_btn.clicked.connect(self._on_pause)
        row.addWidget(self._pause_btn)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._on_abort)
        row.addWidget(self._abort_btn)

        # v7.7: the execution log was named in the status text but not openable.
        self._log_btn = QPushButton("Open log")
        self._log_btn.setEnabled(False)
        self._log_btn.setToolTip("Open the JSONL execution log for the last run.")
        self._log_btn.clicked.connect(self._on_open_log)
        row.addWidget(self._log_btn)

        row.addStretch(1)

        self._status = QLabel("Idle. Pick an object and a well.")
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(10)}pt;")
        self._status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        row.addWidget(self._status, stretch=1)

        return frame

    # ── Required by MainWindow ────────────────────────────────────

    def get_page_title(self) -> str:
        return "Quick Print"

    def get_sub_page_title(self) -> str:
        return "Quick Print"

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
        if self._context_widget is not None and hasattr(
                self._context_widget, "on_status_update"):
            self._context_widget.on_status_update()
        self._push_live_position()
        self._update_button_state()

    # ── v7.20 embedding API (public) ──────────────────────────────
    # For a host that mounts this page inside its own shell (the Print
    # Calibrator). Read accessors are thin wrappers over the existing privates —
    # no logic is moved out, so there is still exactly one implementation of
    # each fact. Deliberately NOT proxied: plate / well_positions / safe_z /
    # z_references / hw_config / CommonPrintSettings. The host receives all of
    # those from the SAME WorkflowsModePage fan-out, so proxying them would
    # create a second path to one fact.

    def set_external_object(self, obj_dict: dict | None, *,
                            label: str = "External object",
                            lock: bool = True) -> None:
        """Use a single host-supplied object dict as the selected object.

        Thin wrapper over :meth:`set_external_objects` — see it for the details.
        """
        self.set_external_objects(
            [obj_dict] if obj_dict else None, label=label, lock=lock)

    def set_external_objects(self, obj_dicts, *,
                             label: str = "External object",
                             lock: bool = True) -> None:
        """Use a host-supplied LIST of object dicts as the selected object.

        Each entry is the canonical GeometryEngine shape
        ``{"object_type": …, "params": {…}, "source": "parametric"}`` and runs
        through the same ``PrintObject.from_dict`` +
        ``generate_object_trajectory`` pipeline as a saved print — no new
        geometry path exists.

        A LIST rather than one dict because the objects become SEPARATE print
        segments, exactly as a multi-object saved print does, so
        ``build_well_plate_job`` inserts a lift → hop → lower → re-prime between
        them instead of extruding across the gap. (The Print Calibrator's second
        line exists precisely to measure that hop.)

        ``None``/empty clears it and re-enables the combo. ``lock=True`` disables
        the object combo (a calibration run must print the host's object) —
        disabled-and-visible rather than hidden, so the operator can still SEE
        what will print. ``label`` reaches the job name, the confirm dialog and
        the exec log via ``_object_label``, so put the geometry in it.
        """
        items = [copy.deepcopy(d) for d in (obj_dicts or []) if d]
        self._external_objs = items
        self._external_label = str(label or "External object")
        self._external_lock = bool(lock) and bool(items)
        self._object_combo.setToolTip(
            "The object is set by the hosting page and cannot be changed here."
            if self._external_lock else "")
        # _refresh_objects re-adds + re-selects the entry, re-asserts the lock
        # and tails into _on_object_changed (geometry + readiness + buttons).
        self._refresh_objects()

    def external_object(self) -> dict | None:
        """The FIRST host-supplied object, or None. Back-compat accessor."""
        objs = self.external_objects()
        return objs[0] if objs else None

    def external_objects(self) -> list:
        return [copy.deepcopy(d) for d in self._external_objs]

    def set_prime_override_s(self, value: float | None) -> None:
        """Force the pre-flow prime (s) for the next run, or ``None`` to use the
        settings popout again.

        While set, :meth:`_preflow_s` returns this regardless of the popout and
        the popout's own spin is DISABLED — one home for the fact, so the
        operator cannot silently defeat a calibration run by editing it.
        """
        self._prime_override_s = (
            None if value is None else max(0.0, float(value)))
        w = getattr(self, "_preflow", None)
        if w is not None:
            try:
                w.setEnabled(self._prime_override_s is None)
            except Exception:
                pass
        self._refresh_setup_status()
        self._update_settings_summary()
        self._refresh_planned_path()

    def prime_override_s(self) -> float | None:
        return self._prime_override_s

    def set_preflow_s(self, value: float) -> None:
        """Write the per-run pre-flow spin (what :meth:`_preflow_s` returns when
        no override is active). The spin is linked to the global
        ``pump_prime_time_s``, so this is only for the case where the host wants
        the LOCAL value set too."""
        w = getattr(self, "_preflow", None)
        if w is not None:
            try:
                w.setValue(max(0.0, float(value)))
            except Exception:
                pass

    def prime_default_s(self) -> float:
        """``HardwareConfig.pump_prime_time_s`` as this page resolves it — the
        "prime currently configured" figure an Apply dialog must show."""
        return self._prime_default_s()

    def resolved_print_kinematics(self) -> tuple[float, float, float]:
        """``(print_speed_mm_s, flow_uL_s, prime_uL)`` as the run will use them."""
        return self._resolved_print_kinematics()

    def selected_well(self) -> str | None:
        return self._selected_well

    def well_center_zero_ref_mm(self, well: str | None = None):
        """Calibrated-then-geometric well centre in ZERO-REF mm (the frame
        ``build_well_plate_job`` takes), or ``None``."""
        w = well or self._selected_well
        return self._well_center_zero_ref_mm(w) if w else None

    def well_radius_um(self, well: str | None = None) -> float:
        """FULL well radius (µm), 0.0 when unknown. The caller owns any inset
        margin — there is no shared "usable radius" convention in this repo."""
        w = well or self._selected_well
        return self._well_radius_um(w) if w else 0.0

    def pump(self) -> str:
        return self._pump()

    def last_log_path(self):
        """The last finished run's JSONL exec log, or ``None``. Valid as soon as
        :attr:`print_state_changed` fires with a terminal state."""
        return self._last_log_path

    def is_printing(self) -> bool:
        return self._is_running()

    def cleanup_running(self) -> bool:
        """True while the post-print cleanup worker is driving the stage.

        A host must not move the stage — nor trust where it is — until this is
        False: on a clean completion the cleanup travels to waste → wash → oil,
        so the live view is no longer showing the print."""
        t = getattr(self, "_cleanup_thread", None)
        return t is not None and t.is_alive()

    def _push_live_position(self) -> None:
        """Feed the live needle XY (zero-ref µm) into the trajectory monitor —
        the same conversion the Jog page uses (absolute stage µm − zero).

        v7.7: also repaints the live-numbers panel, so the executor's telemetry
        is rendered at a fixed 10 Hz regardless of how fast it arrives.
        """
        try:
            self._render_run_info()
        except Exception:
            pass
        if not hasattr(self, "_traj_view"):
            return
        try:
            xy = self._controller.get_xy_position(cached=True)
            zero = self._controller.zero_position
        except Exception:
            return
        if not xy or xy[0] is None or xy[1] is None:
            self._traj_view.set_position(None, None)
            return
        try:
            zx = float(xy[0]) - float(zero["x"])
            zy = float(xy[1]) - float(zero["y"])
        except Exception:
            return
        self._traj_view.set_position(zx, zy)

    def set_settings(self, settings) -> None:
        self._settings = settings
        if self._context_widget is not None:
            self._context_widget.set_settings(settings)

    def set_common_print_settings(self, common):
        """v7.5.x: shared common settings — re-sync the popout's inheriting prep
        fields + global pump fields."""
        if getattr(self, "_settings_dialog", None) is not None:
            self._settings_dialog.set_common(common)

    def set_hardware_config(self, hw_config) -> None:
        self._hw_config = hw_config

        # Refresh pump options from enabled+configured pumps.
        self._pump_combo.blockSignals(True)
        previous = self._pump_combo.currentText()
        self._pump_combo.clear()
        if hw_config is not None and hasattr(hw_config, "pumps"):
            for pid, pcfg in hw_config.pumps.items():
                enabled = getattr(pcfg, "enabled", True)
                configured = getattr(pcfg, "is_configured", True)
                if enabled and configured:
                    self._pump_combo.addItem(pid)
        if self._pump_combo.count() == 0:
            self._pump_combo.addItem("P1")
        idx = self._pump_combo.findText(previous)
        if idx >= 0:
            self._pump_combo.setCurrentIndex(idx)
        self._pump_combo.blockSignals(False)

        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)
        # Rebind the live camera to the (possibly changed) microscope slot.
        if self._camera_view is not None:
            cam_idx = self._resolve_microscope_cam_idx()
            try:
                if self._camera_view.cam_idx != cam_idx:
                    self._camera_view.set_camera(cam_idx)
                    if self.isVisible():
                        self._start_camera()
            except Exception as e:
                logger.debug("Quick Print camera rebind failed: %s", e)
        # Repopulate the ink-override combo from the new library/locations and
        # refresh the setup-confirm status (pump combo was blockSignals'd above).
        self._refresh_ink_combo()
        # The pump + ink combos just (re)populated — apply any saved selections
        # that were pending because the combos were empty at load time.
        try:
            self._settings_dialog.resolve_pending()
        except Exception:
            pass
        self._refresh_setup_status()
        self._refresh_planned_path()
        self._update_button_state()
        self._update_settings_summary()

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if plate is not None:
            self._navigator.set_plate(plate)
        if well_positions:
            self._navigator.set_well_positions(well_positions)
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, well_positions, safe_z)
        # v7.19: `set_plate` clears everything keyed by well name (the wells may
        # have been renamed), and the calibration change can also make a queued
        # well newly runnable or not — so re-push the queue's glyphs and flags.
        self._glyph_cache.clear()
        self._refresh_queue_ui()
        # Service-well + ink-well resolution depends on the calibrated positions.
        self._refresh_setup_status()
        self._refresh_planned_path()
        self._update_button_state()

    def set_z_references(self, refs: dict) -> None:
        if not isinstance(refs, dict):
            return
        for k in self._z_references.keys():
            if k in refs:
                self._z_references[k] = refs[k]
        # v7.5.x: the Print-Z spin is now a *height above the plate bottom*
        # (relative), so it is no longer seeded from the absolute plate_bottom_z
        # reference — its small positive default is already plate-relative.
        if self._context_widget is not None and hasattr(
                self._context_widget, "set_z_references"):
            try:
                self._context_widget.set_z_references(self._z_references)
            except Exception:
                pass
        # Plate-bottom datum affects the resolvable service/ink dip Z.
        self._refresh_setup_status()

    # ── Object selection ──────────────────────────────────────────

    def _refresh_objects(self) -> None:
        """Rebuild the object combo: built-in simple shapes + saved prints."""
        prev = self._object_combo.currentData()
        self._object_combo.blockSignals(True)
        self._object_combo.clear()
        # userData is a "kind:ref" string — QComboBox.findData matches strings
        # reliably (it does not for tuples). file names may contain ':', so
        # parse with split(":", 1).
        # v7.20: a host-supplied object goes FIRST and carries a stable token, so
        # the `findData(prev)` restore below survives every rebuild — and this
        # method is called from showEvent, i.e. on every tab-in.
        if self._external_objs:
            self._object_combo.addItem(
                self._external_label or "External object", self._EXTERNAL_DATA)
        for key, label in _SIMPLE_SHAPES.items():
            self._object_combo.addItem(label, f"simple:{key}")
        try:
            files = self._print_mgr.list_files()
        except Exception as e:
            logger.warning("Failed to list saved prints: %s", e)
            files = []
        for info in files:
            name = info.get("name", "")
            count = info.get("object_count", 0)
            if not name:
                continue
            self._object_combo.addItem(f"📄 {name}  ({count})", f"file:{name}")
        # Restore previous selection if still present.
        if prev is not None:
            idx = self._object_combo.findData(prev)
            if idx >= 0:
                self._object_combo.setCurrentIndex(idx)
        # v7.20: a host-locked object must also be re-SELECTED, not merely
        # present: the very first rebuild after set_external_object has no `prev`
        # to restore, and a rebuild must never silently re-enable the combo.
        if self._external_objs and self._external_lock:
            idx = self._object_combo.findData(self._EXTERNAL_DATA)
            if idx >= 0:
                self._object_combo.setCurrentIndex(idx)
        self._object_combo.blockSignals(False)
        self._object_combo.setEnabled(not self._external_lock)
        self._on_object_changed()

    @staticmethod
    def _parse_obj_data(data):
        """Parse a combo userData string 'kind:ref' → (kind, ref) or None."""
        if not data or ":" not in data:
            return None
        kind, ref = data.split(":", 1)
        return (kind, ref)

    def _on_object_changed(self, *_):
        data = self._parse_obj_data(self._object_combo.currentData())
        is_simple = bool(data) and data[0] == "simple"
        ref = data[1] if is_simple else None
        sized = is_simple and ref in ("circle", "meander")
        self._size_label.setVisible(sized)
        self._size_spin.setVisible(sized)
        # v7.5.x: detect an abstract-ink sketch + rebuild the runtime ink map.
        self._loaded_sketch = self._load_selected_sketch()
        self._rebuild_ink_mapping_ui()
        self._refresh_planned_path()
        self._refresh_setup_status()
        self._update_button_state()

    def _on_well_clicked(self, name: str) -> None:
        if not name:
            return
        # v7.19: flush a pending edit into the OUTGOING well BEFORE the incoming
        # one overwrites the editors. A 120 ms debounce that has not fired yet
        # would otherwise land in the wrong entry, or be lost outright.
        self._flush_write_back()

        self._selected_well = name
        # The navigator's `_current_well` is a RENDER of `_selected_well`: pushed
        # one way, never read back. Reading it back is how a second home for the
        # fact appears.
        self._navigator.set_current_well(name)

        entry = self._queue_get(name)
        if entry is not None:
            # _apply_snapshot alone performs no fan-out, so do it once here.
            self._applying_snapshot = True
            widgets = self._snapshot_widgets()
            for w in widgets:
                w.blockSignals(True)
            try:
                warnings = self._apply_snapshot(entry)
            finally:
                for w in widgets:
                    w.blockSignals(False)
                self._applying_snapshot = False
            self._on_object_changed()      # the ONE fan-out
            self._update_settings_summary()
            self._status.setText(
                f"Well {name} — editing its queued print."
                + (f"  ⚠ {warnings[0]}" if warnings else ""))
        else:
            self._refresh_planned_path()
            self._status.setText(f"Well {name} selected (nothing queued).")
        self._refresh_apply_button()
        self._update_button_state()

    def _on_pump_changed(self, *_):
        """Pump changed → re-default the ink combo to that pump's ink."""
        self._refresh_ink_combo()
        self._refresh_setup_status()

    # ── Geometry → path points ────────────────────────────────────

    def _needle_and_syringe(self):
        """Build (needle, syringe_map) from hw_config, with a default-needle
        fallback so simple shapes work even before hardware is configured."""
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        syringe_map: dict[str, object] = {}
        pumps = getattr(self._hw_config, "pumps", {}) if self._hw_config else {}
        for pid, pcfg in (pumps or {}).items():
            syr = getattr(pcfg, "syringe", None)
            if syr is not None:
                syringe_map[pid] = syr
        if needle is None:
            try:
                from SupportClasses.PhysicalModels import NeedleSpec
                needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
            except Exception as e:
                logger.warning("Default NeedleSpec construction failed: %s", e)
                needle = None
        return needle, syringe_map

    def _simple_shape_dict(self, ref: str) -> dict:
        size = float(self._size_spin.value())
        if ref == "dot":
            return {
                "name": "QuickDot", "object_type": "point",
                "params": {"dispense_volume_uL": 0.5, "dwell_time_s": 1.0},
                "source": "parametric",
            }
        if ref == "circle":
            return {
                "name": "QuickCircle", "object_type": "circle",
                "params": {"radius": size, "num_points": 64, "filled": False},
                "source": "parametric",
            }
        # meander — filled circular meander raster, stays inside a round well
        return {
            "name": "QuickMeander", "object_type": "circle",
            "params": {"radius": size, "num_points": 64, "filled": True},
            "source": "parametric",
        }

    def _obj_dict_to_trajectory(self, obj_dict, needle, syringe_map):
        """Build one object's Nx7 trajectory ``[x,y,z,p1,p2,p3,t]`` (mm), in the
        well-relative frame, or ``None`` if it can't be built.

        Uses the persisted trajectory for csv-sourced objects, otherwise the
        canonical ``generate_object_trajectory`` pipeline. The full Nx7 array
        (not just XY) is returned so callers can inspect the Z (travel lifts)
        and pump (extrude-on/off) columns — see :meth:`_obj_dict_to_subpaths`.
        """
        from SupportClasses.GeometryEngine import (
            PrintObject, generate_object_trajectory,
        )

        od = copy.deepcopy(obj_dict)
        od.setdefault("name", "object")
        otype = od.get("object_type", "")
        traj = None

        if od.get("source") == "csv" or otype == "csv_import":
            csv_data = od.get("trajectory")
            if not csv_data:
                params = od.get("params", {}) or {}
                csv_data = params.get("_csv_data")
                csv_src = params.get("source_file") or params.get("csv_path")
                if csv_data is None and csv_src:
                    try:
                        from SupportClasses.TrajectoryPlanner import (
                            import_csv_trajectory,
                        )
                        csv_data = import_csv_trajectory(csv_src)
                    except Exception as e:
                        logger.warning("CSV load failed for path: %s", e)
            if csv_data is not None:
                traj = np.asarray(csv_data, dtype=np.float64)
        else:
            obj = PrintObject.from_dict(od)
            if obj.has_trajectory:
                traj = obj.trajectory
            elif needle is not None:
                generate_object_trajectory(
                    obj, needle, syringe_map, pump_id=self._pump())
                traj = obj.trajectory

        if traj is None or len(traj) == 0:
            return None
        arr = np.asarray(traj, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            return None
        return arr

    # ══ v7.21.5: the sketch's own extrusion, along the path ══════════
    #
    # A Print-Builder sketch bakes ``extrusion_profile`` — one extrusion
    # MODIFIER per trajectory segment (0.0 = travel or a no-extrude shape) —
    # plus the reference bead width it was measured against. Quick Print used
    # to discard it and re-derive ONE flow for the whole path from its own
    # knobs, so a sketch whose shapes declare different line widths printed
    # them all at the same width. Now the profile rides through to the
    # executor, and Quick Print's own Extrusion × becomes a TRIM on top of it.

    @staticmethod
    def _obj_dict_extrusion_modifiers(obj_dict, n_segments: int):
        """The baked per-segment extrusion modifiers for one object, or None.

        REFUSES a profile whose length does not match the object's own
        trajectory (``n_segments``): an off-by-one profile would apply a
        shape's flow to the wrong part of the path — worse than falling back
        to a single flow, which is merely the previous behaviour.
        """
        params = (obj_dict or {}).get("params", {}) or {}
        raw = params.get("extrusion_profile")
        if not raw or n_segments <= 0:
            return None
        try:
            vals = [max(0.0, float(v)) for v in raw]
        except (TypeError, ValueError):
            logger.warning("extrusion_profile is not numeric — ignoring it")
            return None
        if len(vals) != n_segments:
            logger.warning(
                "extrusion_profile has %d entries for %d segments — ignoring "
                "it and using a single flow", len(vals), n_segments)
            return None
        return vals

    def _obj_dict_to_path_points(self, obj_dict, needle, syringe_map):
        """Convert one object dict → flat list[(x_mm, y_mm)] (well-relative).

        Back-compat helper (XY only). Execution uses :meth:`_obj_dict_to_subpaths`
        so internal travel moves break the path into separate print segments.
        """
        arr = self._obj_dict_to_trajectory(obj_dict, needle, syringe_map)
        if arr is None:
            return []
        return [(float(arr[i, 0]), float(arr[i, 1])) for i in range(arr.shape[0])]

    @staticmethod
    def _travel_mask(arr: np.ndarray):
        """Per-segment travel flag for an Nx7 trajectory, or ``None`` when the
        path carries no travel information (so the whole object is one segment).

        A segment i→i+1 is **travel** (a non-extruding "quick move to a new
        location" — the needle should lift and the pump must stop) when:
        - the path has pump info: the pump columns do NOT advance over it
          (the compiler's printing=False moves) **AND the needle actually
          moves** over the segment. This is the PREFERRED signal — robust,
          layer-agnostic, and **polarity-independent**. The motion requirement
          is essential: where the sketch compiler WELDS two connected paths
          (shapes sharing a node) into one continuous bead, it emits a
          zero-distance coincident waypoint at the shared node whose pump
          column is flat. That is NOT a travel — it's the join — and splitting
          there would lift the needle and restart the print mid-stroke (the
          reported "connected lines print discontinuously" bug); OR
        - the path has NO usable pump column but has Z lifts: the segment
          touches the top Z band (the travel/lift height). This is only a
          FALLBACK because it assumes travel is the numerically HIGHER Z, which
          is NOT universal across saved trajectories — some older sketch/CSV
          files lift to a LOWER Z (print plane numerically above the lift). If
          the z-band were OR'd in alongside the pump signal, those reversed-
          polarity files would have their entire print plane flagged as travel
          (every sub-path dropped → blank preview + an empty, no-op print), so
          the z-band is used ONLY when no pump info is available.

        Returns a bool array of length N-1, or ``None`` if the path is flat in
        BOTH Z and pump (e.g. a plain circle/meander) so legacy single-segment
        behaviour is preserved exactly.
        """
        n = arr.shape[0]
        if n < 2:
            return None
        has_pump = arr.shape[1] >= 6
        if has_pump:
            dp = np.abs(np.diff(arr[:, 3:6], axis=0)).sum(axis=1)
            pump_info = bool(dp.sum() > 1e-9)
        else:
            dp = np.zeros(n - 1)
            pump_info = False
        z = arr[:, 2]
        z_lo, z_hi = float(z.min()), float(z.max())
        z_rng = z_hi - z_lo
        z_info = z_rng > 1e-3
        if not pump_info and not z_info:
            return None     # flat path → one print segment (legacy)

        mask = np.zeros(n - 1, dtype=bool)
        if pump_info:
            # Preferred: travel = the pump does not advance AND the needle
            # actually repositions. Polarity-safe. Excluding zero-distance
            # pump-flat segments keeps welded/coincident nodes (shared nodes of
            # connected shapes) continuous instead of splitting the print there.
            d3 = np.diff(arr[:, :3], axis=0)
            seg_dist = np.sqrt((d3 ** 2).sum(axis=1))
            mask |= (dp <= 1e-9) & (seg_dist > 1e-6)
        elif z_info:
            # Fallback only (no usable pump column): travel = the top Z band.
            z_thresh = z_hi - max(1e-4, 0.02 * z_rng)
            top = z >= z_thresh
            mask |= (top[:-1] | top[1:])
        return mask

    def _obj_dict_to_subpaths(self, obj_dict, needle, syringe_map):
        """Convert one object → list of print sub-paths ``[[(x,y)…], …]``.

        Splits the object's trajectory at its internal travel moves so
        ``build_well_plate_job`` inserts a lift→hop→lower→prime between print
        runs (the pump stops + the needle retracts across the "quick move to a
        new location"), instead of extruding straight through at print Z. A
        path with no travel info yields a single sub-path == the legacy flat
        path.
        """
        arr = self._obj_dict_to_trajectory(obj_dict, needle, syringe_map)
        n_seg = (arr.shape[0] - 1) if arr is not None and len(arr) else 0
        return self._subpaths_from_array(
            arr, self._obj_dict_extrusion_modifiers(obj_dict, n_seg))

    def _subpaths_from_array(self, arr, modifiers=None):
        """Split an Nx7 trajectory into print sub-paths ``[[(x,y)…], …]`` at its
        internal travel moves (shared by the object path and the per-ink-group
        multi-ink path). A path with no travel info yields one sub-path.

        v7.21.5: when ``modifiers`` (one extrusion modifier per trajectory
        SEGMENT) is given, it is sliced in LOCKSTEP with the same split and
        recorded on :attr:`_last_subpath_modifiers` — one list per returned
        sub-path, each with ``len(sub) - 1`` entries. Kept as a side channel
        rather than a changed return type because ``_path_segments_for_selection``
        has a dozen callers that want plain XY.
        """
        self._last_subpath_modifiers = []
        if arr is None:
            return []
        pts = [(float(arr[i, 0]), float(arr[i, 1])) for i in range(arr.shape[0])]
        mods = list(modifiers) if modifiers is not None else None
        if mods is not None and len(mods) != max(0, len(pts) - 1):
            mods = None                     # refuse a mismatched profile
        mask = self._travel_mask(arr)
        if mask is None:
            if len(pts) < 2:
                return []
            self._last_subpath_modifiers = [list(mods)] if mods is not None else [None]
            return [pts]

        subpaths: list[list[tuple[float, float]]] = []
        sub_mods: list = []
        cur: list[tuple[float, float]] = [pts[0]]
        cur_mods: list[float] = []
        for i in range(len(pts) - 1):
            if bool(mask[i]):
                # Segment i→i+1 is travel: close the current print run and
                # restart at the travel destination (drop the travel hop).
                if len(cur) >= 2:
                    subpaths.append(cur)
                    sub_mods.append(cur_mods if mods is not None else None)
                cur = [pts[i + 1]]
                cur_mods = []
            else:
                cur.append(pts[i + 1])
                if mods is not None:
                    cur_mods.append(mods[i])
        if len(cur) >= 2:
            subpaths.append(cur)
            sub_mods.append(cur_mods if mods is not None else None)
        self._last_subpath_modifiers = sub_mods
        return subpaths

    def _path_segments_for_selection(self) -> list[list[tuple[float, float]]]:
        """One print sub-path per contiguous extrusion run, well-relative.

        A saved multi-object print (e.g. several spirals at different offsets)
        yields one segment per object, AND each object's own internal travel
        moves (pen-up "quick moves to a new location" — e.g. a multi-shape
        sketch) split it further, so ``build_well_plate_job`` inserts a
        lift→hop→lower→prime between every print run. This stops the pump and
        retracts Z across the travel instead of extruding across the seam and
        dragging the needle through already-printed material.
        """
        data = self._parse_obj_data(self._object_combo.currentData())
        if not data:
            return []
        kind, ref = data
        needle, syringe_map = self._needle_and_syringe()

        if kind == "external":
            # v7.20: host-supplied object dicts. Deep-copied because
            # `PrintObject.from_dict` pops "trajectory" off the dict it is given,
            # and they then run the SAME generate_object_trajectory pipeline as a
            # saved print — no new geometry path exists. Each becomes its own
            # segment below, exactly as a multi-object saved print does.
            obj_dicts = []
            for i, od in enumerate(self._external_objs):
                od = copy.deepcopy(od)
                if not od:
                    continue
                od.setdefault("name", f"External{i + 1}")
                obj_dicts.append(od)
            if not obj_dicts:
                return []
        elif kind == "simple":
            obj_dicts = [self._simple_shape_dict(ref)]
        else:  # saved print file (ref = display name)
            pf = self._print_mgr.load(ref)
            if pf is None:
                return []
            obj_dicts = []
            for name, od in pf.objects.items():
                if isinstance(od, dict):
                    od = dict(od)
                    od.setdefault("name", name)
                    obj_dicts.append(od)

        segments: list[list[tuple[float, float]]] = []
        profiles: list = []
        for od in obj_dicts:
            subs = self._obj_dict_to_subpaths(od, needle, syringe_map)
            mods = list(getattr(self, "_last_subpath_modifiers", []) or [])
            for k, sub in enumerate(subs):
                if sub and len(sub) >= 2:
                    segments.append(sub)
                    profiles.append(mods[k] if k < len(mods) else None)
        # v7.21.5: kept beside the segments (same order, same length) so the job
        # builder can hand each PRINT_PATH its own deposition without changing
        # this method's return type — a dozen callers want plain XY.
        self._last_segment_modifiers = profiles
        return segments

    def _segments_and_modifiers_for_selection(self):
        """``(segments, modifier_profiles)`` for the selected object.

        ``modifier_profiles[k]`` is either None (no baked profile — use one
        flow) or a list of ``len(segments[k]) - 1`` extrusion modifiers.
        """
        segs = self._path_segments_for_selection()
        mods = list(getattr(self, "_last_segment_modifiers", []) or [])
        while len(mods) < len(segs):
            mods.append(None)
        return segs, mods[:len(segs)]

    def _vol_per_mm_profiles(self, modifier_profiles):
        """Convert per-segment MODIFIERS → µL/mm, against the needle fitted NOW.

        Recomputing from the modifier (rather than replaying the sketch's own
        µL/mm) prints the DECLARED WIDTHS with the current needle instead of
        reproducing a volume that was only right for the needle present at bake
        time. Quick Print's own Extrusion × rides on top as a trim, so the
        operator keeps one knob over a sketch's numbers. Returns a list the same
        length as the input, with None where there was no profile.
        """
        from SupportClasses.ExtrusionProfile import modifier_to_vol_per_mm
        area = self._needle_cross_section_mm2()
        trim = self._extrusion_modifier()
        out = []
        for mods in modifier_profiles or []:
            if not mods or area <= 0:
                out.append(None)
            else:
                out.append(modifier_to_vol_per_mm(mods, area, trim))
        return out

    def _extrusion_profile_summary(self):
        """``(has_profile, min_mod, max_mod)`` over the PRINTING segments of the
        selected object — what the setup status reports and what bounds the
        flow ceiling. Zeros are skipped: a travel/no-extrude segment is an
        absence of deposition, not a thin bead."""
        _segs, mods = self._segments_and_modifiers_for_selection()
        vals = [v for prof in mods if prof for v in prof if v > 0.0]
        if not vals:
            return False, 0.0, 0.0
        return True, min(vals), max(vals)

    def _path_points_for_selection(self) -> list[tuple[float, float]]:
        """Flattened path (all objects concatenated) — kept for geometry
        previews and the printable-path guard; execution uses the segmented
        form via :meth:`_path_segments_for_selection`."""
        points: list[tuple[float, float]] = []
        for seg in self._path_segments_for_selection():
            points.extend(seg)
        return points

    # ══ v7.19: the plate-wide print queue ═══════════════════════════
    #
    # Widgets whose values a snapshot captures, IN WRITE ORDER. Object first: it
    # re-derives the size row's visibility, the sketch detection and the ink-map
    # rows. Pump before ink: changing the pump re-defaults the ink combo.
    _SNAPSHOT_WIDGETS = (
        "_object_combo", "_size_spin", "_pump_combo", "_ink_combo",
        "_ink_z_spin", "_ink_padding_spin", "_top_speed_spin",
        "_resolution_spin", "_printz_spin", "_extrusion_mod_spin",
        "_motion_mode_combo",
    )
    #: Stamping into at least this many wells at once asks first.
    _APPLY_CONFIRM_N = 24

    def _snapshot_widgets(self) -> list:
        return [w for w in (getattr(self, n, None)
                            for n in self._SNAPSHOT_WIDGETS) if w is not None]

    def _snapshot_from_widgets(self, well: str = "") -> QueuedPrint:
        """Capture what is on screen as a QueuedPrint.

        Reads the widgets, so GUI-thread only. Heights are captured in the SPIN's
        own frame (above plate bottom), never resolved to zero-ref — see
        ``SupportClasses.PrintQueue``'s rule 1.
        """
        def _val(name, default):
            w = getattr(self, name, None)
            try:
                return w.value()
            except Exception:
                return default

        ink_map_by_name: dict[str, str] = {}
        sk = self._loaded_sketch
        if sk is not None:
            for iid, mapped in (self._ink_map or {}).items():
                try:
                    ink = sk.ink_by_id(int(iid))
                except Exception:
                    ink = None
                name = getattr(ink, "name", None)
                if name and mapped:
                    ink_map_by_name[str(name)] = str(mapped)
        return QueuedPrint(
            well=well or (self._selected_well or ""),
            object_data=str(self._object_combo.currentData() or ""),
            object_label=self._object_label(),
            size_mm=float(_val("_size_spin", 1.0)),
            pump=self._pump(),
            ink_name=self._selected_ink() or "",
            ink_dip_z_mm=float(_val("_ink_z_spin", 0.5)),
            ink_padding_uL=float(_val("_ink_padding_spin", 0.0)),
            top_speed_mm_s=float(_val("_top_speed_spin", 2.5)),
            resolution_um=float(_val("_resolution_spin", 30.0)),
            print_z_mm=float(_val("_printz_spin", 0.2)),
            extrusion_mod=float(_val("_extrusion_mod_spin", 1.0)),
            motion_mode=self._motion_mode(),
            ink_map_by_name=ink_map_by_name,
        )

    @staticmethod
    def _set_combo_data(combo, data) -> bool:
        """Select the item whose userData == *data*.

        Returns False and leaves the combo ALONE when the value is absent (e.g. a
        saved print deleted off disk). It must never be coerced to index 0 —
        that is how merely OPENING a queue entry would silently EDIT it.
        """
        if combo is None:
            return False
        idx = combo.findData(data)
        if idx < 0:
            return False
        combo.setCurrentIndex(idx)
        return True

    @staticmethod
    def _set_combo_text(combo, text) -> bool:
        if combo is None or not text:
            return False
        idx = combo.findText(str(text))
        if idx < 0:
            return False
        combo.setCurrentIndex(idx)
        return True

    def _apply_snapshot(self, qp: QueuedPrint) -> list[str]:
        """Drive the config widgets to *qp*. Returns disclosure warnings.

        Signals are expected to be blocked by the caller; this performs NO
        fan-out of its own, so the caller decides when the one refresh happens.

        ⚠ Two things that do NOT follow from setting the widgets, because the
        blocked signals mean ``_on_object_changed`` never runs:
          * ``_loaded_sketch`` — set explicitly, or ``_is_multi_ink()`` /
            ``_ink_groups()`` would read the PREVIOUS object's sketch.
          * the ink mapping — seeded into ``_ink_map_last`` (name-keyed) and NOT
            into ``_ink_map``, because ``_rebuild_ink_mapping_ui`` hard-resets
            ``_ink_map`` and re-derives it from ``_ink_map_last``; assigning
            ``_ink_map`` here would be wiped by the very refresh that follows.

        ⚠ ``setValue`` CLAMPS silently, and ``_update_top_speed_cap`` puts a
        dynamic maximum on the speed spin, so a print queued at 6 mm/s on a
        machine since measured at 4.2 would quietly run slower. Every numeric set
        is compared back and a difference is DISCLOSED.
        """
        warnings: list[str] = []
        if qp is None:
            return warnings

        def _set_num(name, wanted, label, unit=""):
            w = getattr(self, name, None)
            if w is None:
                return
            try:
                w.setValue(float(wanted))
                got = float(w.value())
            except Exception:
                return
            if abs(got - float(wanted)) > 1e-6:
                warnings.append(
                    f"{qp.well}: {label} {float(wanted):g}{unit} limited to "
                    f"{got:g}{unit}")

        if not self._set_combo_data(self._object_combo, qp.object_data):
            warnings.append(
                f"{qp.well}: “{qp.object_label or qp.object_data}” is no longer "
                "in the print library")
        _set_num("_size_spin", qp.size_mm, "size", " mm")
        self._set_combo_text(self._pump_combo, qp.pump)
        self._set_combo_data(self._ink_combo, qp.ink_name)
        _set_num("_ink_z_spin", qp.ink_dip_z_mm, "ink dip Z", " mm")
        _set_num("_ink_padding_spin", qp.ink_padding_uL, "ink padding", " µL")
        _set_num("_top_speed_spin", qp.top_speed_mm_s, "top speed", " mm/s")
        _set_num("_resolution_spin", qp.resolution_um, "resolution", " µm")
        _set_num("_printz_spin", qp.print_z_mm, "print height", " mm")
        _set_num("_extrusion_mod_spin", qp.extrusion_mod, "extrusion ×")
        self._set_combo_data(self._motion_mode_combo, qp.motion_mode)
        # The two derived caches the blocked signals did not refresh.
        try:
            self._loaded_sketch = self._load_selected_sketch()
        except Exception:
            self._loaded_sketch = None
        if qp.ink_map_by_name:
            self._ink_map_last.update(
                {str(k): str(v) for k, v in qp.ink_map_by_name.items() if v})
        return warnings

    @contextlib.contextmanager
    def _snapshot_applied(self, qp: QueuedPrint):
        """Transiently drive the config widgets to *qp*, then restore.

        GUI-THREAD ONLY, and **nothing inside may re-enter the Qt event loop** —
        no modal dialog, no ``processEvents``. The app's ~300 ms status tick would
        otherwise run ``_refresh_setup_status`` / ``_refresh_planned_path``
        against the TRANSIENT widget values and repaint from the wrong snapshot.
        (This is why the print-floor check inside the resolve pass is numeric and
        its one dialog is shown after everything has been restored.)

        Restoration IS an apply of the snapshot captured on the way in, so the
        save-list and the restore-list are the same list by construction and
        cannot drift apart as fields are added.
        """
        active = self._snapshot_from_widgets()
        prev_sketch = self._loaded_sketch
        prev_ink_map = dict(self._ink_map)
        widgets = self._snapshot_widgets()
        prev_applying = self._applying_snapshot
        self._applying_snapshot = True
        for w in widgets:
            w.blockSignals(True)
        try:
            yield self._apply_snapshot(qp)
        finally:
            try:
                self._apply_snapshot(active)
            except Exception:
                logger.exception("restoring the active print config failed")
            finally:
                self._loaded_sketch = prev_sketch
                self._ink_map = prev_ink_map
                for w in widgets:
                    w.blockSignals(False)
                self._applying_snapshot = prev_applying

    # ── queue mutation ────────────────────────────────────────────

    def _queue_get(self, well: str):
        return _pq.get(self._queue, well)

    def _write_back(self) -> None:
        """Mirror the editors into the ACTIVE well's queue entry.

        WRITE-ONLY: never calls a widget setter and never calls
        ``_apply_snapshot``. That single rule is what makes the
        edit → write-back → refresh → edit cycle impossible.

        No-ops while a snapshot is being applied, with no active well, or when the
        active well has no entry — editing an un-queued active well is COMPOSING a
        new print, which becomes an entry only when Apply says so.
        """
        if self._applying_snapshot:
            return
        well = self._selected_well
        if not well or self._queue_get(well) is None:
            return
        try:
            self._queue = _pq.upsert(self._queue,
                                     self._snapshot_from_widgets(well))
        except Exception:
            logger.exception("queue write-back failed")
            return
        self._refresh_queue_ui()

    def _queue_writeback_soon(self) -> None:
        t = getattr(self, "_writeback_timer", None)
        if t is not None and not self._applying_snapshot:
            t.start()

    def _flush_write_back(self) -> None:
        """Force a pending debounced write-back to land NOW.

        Called before the active well changes: a 120 ms debounce that has not
        fired yet would otherwise land in the WRONG entry, or be lost outright.
        """
        t = getattr(self, "_writeback_timer", None)
        if t is not None:
            t.stop()
        self._write_back()

    def _on_apply_print_to_wells(self) -> None:
        wells = self._navigator.selected_wells()
        if not wells:
            self._status.setText("Select one or more wells on the plate first.")
            return
        if not self._object_combo.currentData():
            self._status.setText("Choose an object first.")
            return
        overwriting = [w for w in wells if self._queue_get(w) is not None]
        if len(wells) >= self._APPLY_CONFIRM_N or overwriting:
            lines = [f"Apply this print to {len(wells)} well"
                     f"{'' if len(wells) == 1 else 's'}?"]
            if overwriting:
                lines.append(
                    f"{len(overwriting)} of them already hold a queued print, "
                    "which will be REPLACED "
                    f"({', '.join(overwriting[:6])}"
                    f"{' …' if len(overwriting) > 6 else ''}).")
            if QMessageBox.question(
                    self, "Apply print to wells", "\n\n".join(lines),
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
            ) != QMessageBox.StandardButton.Yes:
                self._status.setText("Cancelled.")
                return
        snap = self._snapshot_from_widgets()
        for w in wells:
            # A COPY per well. One shared object would make editing A17 edit all
            # forty, which is exactly what "each print is an independent
            # activity" rules out.
            entry = snap.copy()
            entry.well = w
            self._queue = _pq.upsert(self._queue, entry)
        if self._selected_well is None:
            # Otherwise the preview above the plate would stay blank.
            self._selected_well = wells[0]
            self._navigator.set_current_well(wells[0])
            self._refresh_planned_path()
        self._refresh_queue_ui()
        self._refresh_setup_status()
        self._update_button_state()
        self._status.setText(
            f"Applied “{self._object_label()}” to {len(wells)} well"
            f"{'' if len(wells) == 1 else 's'}.")

    def _on_queue_remove(self) -> None:
        wells = [w for w in self._navigator.selected_wells()
                 if self._queue_get(w) is not None]
        if not wells and self._selected_well:
            # Falling back to the active well spares an operator who clicked one
            # well and wants it gone from having to go re-select it.
            if self._queue_get(self._selected_well) is not None:
                wells = [self._selected_well]
        if not wells:
            self._status.setText("No queued wells selected.")
            return
        for w in wells:
            self._queue = _pq.remove(self._queue, w)
        self._refresh_queue_ui()
        self._refresh_setup_status()
        self._update_button_state()
        self._status.setText(f"Removed {len(wells)} queued print"
                             f"{'' if len(wells) == 1 else 's'}.")

    def _on_queue_clear(self) -> None:
        if not self._queue:
            return
        if QMessageBox.question(
                self, "Clear the queue",
                f"Remove all {len(self._queue)} queued prints?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
        ) != QMessageBox.StandardButton.Yes:
            return
        self._queue = []
        self._refresh_queue_ui()
        self._refresh_setup_status()
        self._update_button_state()
        self._status.setText("Queue cleared.")

    def _on_well_selection_changed(self, wells) -> None:
        self._refresh_apply_button()

    # ── queue → UI ────────────────────────────────────────────────

    def _queue_group_by_ink(self) -> bool:
        w = getattr(self, "_group_by_ink_check", None)
        try:
            return bool(w.isChecked())
        except Exception:
            return False

    def _queue_clean_between(self) -> bool:
        w = getattr(self, "_clean_between_check", None)
        try:
            return bool(w.isChecked())
        except Exception:
            return False

    def _plate_index(self):
        names = getattr(self._plate, "well_names", None) if self._plate else None
        return plate_index_from_names(names or [])

    def _ordered_queue(self) -> list:
        return order_queue(
            self._queue,
            ORDER_INK if self._queue_group_by_ink() else ORDER_PLATE,
            self._plate_index())

    def _queue_entry_problem(self, qp: QueuedPrint) -> str:
        """A short, operator-facing reason this entry cannot run, or "".

        Cheap checks only — the authoritative per-unit gating happens in
        :meth:`_resolve_queue`. This drives the plate flag and the list row, so it
        must not build geometry.
        """
        if not qp.object_data:
            return "no object"
        if self._object_combo.findData(qp.object_data) < 0:
            return "print no longer in the library"
        kind_ref = self._parse_obj_data(qp.object_data)
        if kind_ref and kind_ref[0] == "file":
            try:
                if self._print_mgr.load(kind_ref[1]) is None:
                    return "print file missing"
            except Exception:
                return "print file unreadable"
        wells = self._well_positions or {}
        if self._plate is not None and qp.well not in (
                getattr(self._plate, "well_names", None) or []):
            return "not a well on this plate"
        if wells and qp.well not in wells:
            return "well not calibrated"
        return ""

    def _refresh_queue_ui(self) -> None:
        """One entry point for everything the queue drives: the plate glyphs, the
        list readout, the counts and the buttons."""
        if not hasattr(self, "_queue_list"):
            return
        problems = {qp.well: self._queue_entry_problem(qp)
                    for qp in self._queue}
        self._refresh_queue_overlay(problems)
        self._queue_list.clear()
        for qp in self._ordered_queue():
            why = problems.get(qp.well, "")
            text = qp.label() + (f"   ⚠ {why}" if why else "")
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, qp.well)
            if why:
                item.setForeground(QColor(COLORS["red"]))
            self._queue_list.addItem(item)
        n = len(self._queue)
        card = getattr(self, "_queue_card", None)
        if card is not None and hasattr(card, "set_title"):
            card.set_title(f"Queue — {n} print{'' if n == 1 else 's'}")
        self._refresh_apply_button()
        for name in ("_queue_remove_btn", "_queue_clear_btn",
                     "_queue_select_btn"):
            b = getattr(self, name, None)
            if b is not None:
                b.setEnabled(n > 0)
        self._update_button_state()

    def _refresh_apply_button(self) -> None:
        btn = getattr(self, "_apply_btn", None)
        if btn is None:
            return
        wells = self._navigator.selected_wells() if hasattr(
            self, "_navigator") else []
        n = len(wells)
        has_obj = bool(self._object_combo.currentData())
        # NOT gated on readiness: laying out a plate with the stage disconnected
        # or busy is the whole point of a queue.
        btn.setEnabled(self._plate is not None and has_obj and n >= 1)
        if self._plate is None:
            btn.setToolTip("No plate loaded — run Calibration first.")
        elif not has_obj:
            btn.setToolTip("Choose an object first.")
        elif n == 0:
            btn.setToolTip(
                "Select one or more wells on the plate below — click, or drag "
                "across wells — then apply this print to them.")
        else:
            shown = ", ".join(wells[:6]) + (" …" if n > 6 else "")
            btn.setToolTip(
                f"Stamp this print and its parameters into the {n} selected "
                f"well{'' if n == 1 else 's'} ({shown}). Any print already "
                "queued in those wells is REPLACED.")
        lbl = getattr(self, "_queue_count_lbl", None)
        if lbl is not None:
            lbl.setText(f"{len(self._queue)} queued · {n} selected")

    def _glyph_paths_for(self, qp: QueuedPrint):
        """Well-relative polylines to draw inside *qp*'s well, decimated.

        Cached by CONFIG identity (object + size + resolution), not by well: a
        real queue is usually "the same print in forty wells", so this normally
        holds one entry. Without it, a 384-well plate would rebuild hundreds of
        trajectories on every debounced edit.
        """
        key = (qp.object_data, round(float(qp.size_mm), 4),
               round(float(qp.resolution_um), 3))
        hit = self._glyph_cache.get(key)
        if hit is not None:
            return hit
        paths: list = []
        try:
            with self._snapshot_applied(qp):
                for seg in self._path_segments_for_selection():
                    if not seg or len(seg) < 2:
                        continue
                    # DISPLAY-ONLY decimation. A meander at 30 µm resolution is
                    # thousands of points rendered into a ~12 px circle; the
                    # EXECUTOR always uses the full geometry.
                    step = max(1, len(seg) // 64)
                    thin = list(seg[::step])
                    if thin[-1] != seg[-1]:
                        thin.append(seg[-1])       # keep the endpoint
                    paths.append(thin)
        except Exception as exc:
            logger.debug("glyph geometry failed for %s: %s", qp.well, exc)
            paths = []
        if len(self._glyph_cache) > 32:
            self._glyph_cache.clear()
        self._glyph_cache[key] = paths
        return paths

    def _refresh_queue_overlay(self, problems: dict | None = None) -> None:
        """Push the queue into the plate as per-well toolpath glyphs."""
        nav = getattr(self, "_navigator", None)
        if nav is None or not hasattr(nav, "set_well_paths"):
            return
        if not self._queue:
            nav.set_well_paths(None)
            return
        if problems is None:
            problems = {qp.well: self._queue_entry_problem(qp)
                        for qp in self._queue}
        overlays: dict[str, tuple] = {}
        for qp in self._queue:
            ok = not problems.get(qp.well)
            overlays[qp.well] = (self._glyph_paths_for(qp) if ok else [],
                                 None, ok)
        nav.set_well_paths(overlays)

    # ── Planned-path preview (zero-ref µm) ─────────────────────────

    def _well_radius_um(self, well: str) -> float:
        """Well radius (µm) for the boundary circle, or 0 if unknown."""
        plate = self._plate
        if plate is None:
            return 0.0
        try:
            d = plate.get_well_info(well).diameter
            if d and d > 0:
                return float(d) * 1000.0 / 2.0
        except Exception:
            pass
        try:
            d = float(getattr(plate, "well_diameter", 0.0) or 0.0)
            if d > 0:
                return d * 1000.0 / 2.0
        except Exception:
            pass
        return 0.0

    def _plan_views(self) -> list:
        """Every view that renders the PLAN.

        ``_traj_view`` first — it is the name the attribute contract pins. Returns
        [] on a partially-built page, so callers need no hasattr guard.

        NOTE the deliberate omission: the LIVE surfaces (``set_position`` /
        ``set_recording`` / ``reset_live``) are NOT routed through here. During a
        queue run the Setup preview shows the ACTIVE print, which may not be the
        well currently printing, so a needle dot there would be false rather than
        merely noisy — and the Run zone is where "what is happening now" belongs.
        """
        out = []
        for name in ("_traj_view", "_setup_preview"):
            v = getattr(self, name, None)
            if v is not None:
                out.append(v)
        return out

    def _refresh_planned_path(self) -> None:
        """Recompute the planned toolpath for the selected well + object and
        push it (in zero-ref µm) into every plan view. Resets the live overlay so
        a previous run's trace doesn't linger on a new plan."""
        views = self._plan_views()
        if not views:
            return
        well = self._selected_well
        if not well or not self._object_combo.currentData():
            for v in views:
                v.set_planned_path(None)
                v.set_well_boundary(None, 0.0)
            return
        center = self._well_center_zero_ref_mm(well)
        if center is None:
            for v in views:
                v.set_planned_path(None)
                v.set_well_boundary(None, 0.0)
            return
        cx_um, cy_um = center[0] * 1000.0, center[1] * 1000.0
        try:
            segments = self._path_segments_for_selection()
        except Exception as e:
            logger.debug("Planned path build failed: %s", e)
            segments = []
        seg_um = [
            [(cx_um + px * 1000.0, cy_um + py * 1000.0) for (px, py) in seg]
            for seg in segments if seg
        ]
        radius_um = self._well_radius_um(well)
        for v in views:
            v.set_planned_path(seg_um or None)
            v.set_well_boundary(
                (cx_um, cy_um) if radius_um > 0 else None, radius_um)

        # Size the needle marker from the configured needle's ORIFICE OD — what
        # actually approaches the plate (best-effort).
        try:
            needle, _ = self._needle_and_syringe()
            od_um = needle_orifice_od_mm(needle) * 1000.0
            if od_um:
                for v in views:
                    v.set_needle(float(od_um))
        except Exception:
            pass

        # Live trace: the Run zone's view ONLY (see _plan_views).
        self._traj_view.reset_live()
        self._kick_prediction(segments, cx_um, cy_um)

    # v7.5.x: simulate the plan on the SAVED stage characteristics (velocity
    # mode) and overlay what the stage is predicted to actually draw.
    # v7.6: the simulation now runs the FEED PLAN (same sections the print
    # executes) and also produces the per-point overlay channels.
    def _kick_prediction(self, segments, cx_um, cy_um) -> None:
        self._pred_gen = getattr(self, "_pred_gen", 0) + 1
        gen = self._pred_gen
        # v7.19: clear on EVERY plan view — otherwise the Setup preview keeps the
        # previous selection's prediction on screen.
        for v in self._plan_views():
            v.set_predicted_path(None)
        if self._motion_mode() != "velocity" or not segments:
            return
        try:
            from SupportClasses import XYFeedPlan as FP
            from SupportClasses.XYStageModel import StageCharacteristics
            from SupportClasses.PrintTimingCalibrationStore import get_store
            store = get_store()
            char = StageCharacteristics.from_store(store)
            if not char.is_complete():
                return
            # Read every GUI value here — the worker must not touch widgets.
            speed = float(self._resolved_print_kinematics()[0])
            res_um = float(self._resolution_um())
            vol_per_mm = (self._needle_cross_section_mm2()
                          * self._extrusion_modifier())
        except Exception:
            return

        def _work():
            try:
                pred, worst_p95, stops, est = [], 0.0, 0, 0.0
                stalled = False
                chans = {"speed": [], "flow": [], "error": [], "time": []}
                t_base = 0.0
                for seg in segments:
                    if len(seg) < 2:
                        continue
                    pts = [(float(x), float(y)) for x, y in seg]
                    plan = FP.build_plan(pts, char,
                                         target_speed_mm_s=max(0.05, speed),
                                         element_um=res_um)
                    r = FP.simulate_plan(plan, char)
                    pred.append([(cx_um + sm[0] * 1000.0,
                                  cy_um + sm[1] * 1000.0) for sm in r.samples])
                    sp, fl, er, tm = _overlay_channels(
                        r, vol_per_mm=vol_per_mm, t_base=t_base)
                    chans["speed"].append(sp)
                    chans["flow"].append(fl)
                    chans["error"].append(er)
                    chans["time"].append(tm)
                    t_base = tm[-1] if tm else t_base
                    worst_p95 = max(worst_p95,
                                    float(r.report.get("p95_um") or 0.0))
                    stops += plan.n_stops
                    est += plan.est_time_s
                    stalled = stalled or not r.completed
                note = (f"plan: {stops} stop{'' if stops == 1 else 's'} · "
                        f"est {est:.0f} s · sim p95 {worst_p95:.0f} µm")
                if stalled:
                    note = "sim: ⚠ predicted to STALL — " + note
                units = {"speed": "mm/s", "flow": "µL/s", "error": "µm",
                         "time": "s"}
                overlays = {}
                for key, per_seg in chans.items():
                    flat = [v for seq in per_seg for v in seq]
                    if flat:
                        overlays[key] = {"values": per_seg,
                                         "unit": units[key],
                                         "vmin": min(flat), "vmax": max(flat)}
                # v7.7: carry the prediction as a NUMBER (reserved key, popped
                # by _on_predicted) so the live readout can show it beside the
                # measured deviation during the print. Kept inside the existing
                # payload so the 4-arg `predicted` signal is unchanged.
                overlays["_pred_p95_um"] = worst_p95
                self._bridge.predicted.emit(gen, pred, note, overlays)
            except Exception as e:                     # pragma: no cover
                logger.debug("prediction sim failed: %s", e)

        threading.Thread(target=_work, daemon=True).start()

    def _on_predicted(self, gen: int, segments, note: str,
                      overlays=None) -> None:
        if gen != getattr(self, "_pred_gen", 0):
            return                       # selection changed while simulating
        p95 = None
        if isinstance(overlays, dict):
            p95 = overlays.pop("_pred_p95_um", None)
        self._pred_overlays = overlays or {}
        self._predicted_p95_um = p95
        views = self._plan_views()
        if views:
            for v in views:
                v.set_predicted_path(segments, note, predicted_p95_um=p95)
            self._apply_overlay_mode()

    def _overlay_pill_mode(self) -> str:
        group = getattr(self, "_overlay_group", None)
        if group is None:
            return "none"
        btn = group.checkedButton()
        return getattr(btn, "_overlay_key", "none") if btn else "none"

    def _apply_overlay_mode(self) -> None:
        """Push the selected overlay channel (or clear it) into every plan view.

        The PILLS stay in the Run zone only: one owner of the chosen channel, two
        renderers of it.
        """
        views = self._plan_views()
        if not views:
            return
        mode = self._overlay_pill_mode()
        data = (getattr(self, "_pred_overlays", None) or {}).get(mode)
        for v in views:
            if mode == "none" or not data:
                v.set_overlay(None)
            else:
                v.set_overlay(mode, data["values"], data["unit"],
                              data.get("vmin"), data.get("vmax"))

    # ── Ink selection / override + pickup volume ──────────────────

    def _printable_inks(self) -> list[str]:
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

    def _pump_assigned_ink(self) -> str | None:
        """The first ink assigned to the currently-selected pump, if any."""
        hw = self._hw_config
        if hw is None:
            return None
        pcfg = (getattr(hw, "pumps", {}) or {}).get(self._pump())
        names = getattr(pcfg, "ink_names", []) if pcfg is not None else []
        return names[0] if names else None

    def _refresh_ink_combo(self) -> None:
        """Repopulate the ink combo: '(none)' + printable inks with a reagent
        location. Default to the selected pump's assigned ink when it qualifies,
        else keep the prior selection if still present."""
        if not hasattr(self, "_ink_combo"):
            return
        prev = self._ink_combo.currentData()
        printable = self._printable_inks()
        self._ink_combo.blockSignals(True)
        self._ink_combo.clear()
        self._ink_combo.addItem("(none — needle already loaded)", "")
        for name in printable:
            self._ink_combo.addItem(name, name)
        assigned = self._pump_assigned_ink()
        target = assigned if assigned in printable else (
            prev if prev in printable else None)
        if target:
            idx = self._ink_combo.findData(target)
            if idx >= 0:
                self._ink_combo.setCurrentIndex(idx)
        self._ink_combo.blockSignals(False)

    def _selected_ink(self) -> str | None:
        """The ink selected for pickup, or None for '(none)'/no inks."""
        if not hasattr(self, "_ink_combo"):
            return None
        return self._ink_combo.currentData() or None

    def _ink_source_well(self) -> str | None:
        """Reagent well name holding the selected ink, or None."""
        ink = self._selected_ink()
        if not ink or self._hw_config is None:
            return None
        wells = (getattr(self._hw_config, "ink_locations", {}) or {}).get(ink) or []
        # Prefer a real sub-well over a flattened rosette parent (e.g. "A2").
        return resolve_pickup_well(wells, self._plate)

    def _ink_source_pos(self) -> tuple[float, float] | None:
        """(x_um, y_um) absolute stage µm of the selected ink's source well,
        or None if it isn't in the calibrated well map."""
        wn = self._ink_source_well()
        wells = self._well_positions or {}
        if wn and wn in wells:
            return wells[wn]
        return None

    def _xy_max_mm_s(self) -> float:
        """Calibrated max XY speed (mm/s) = the '100%' anchor for the print
        speed. v7.5.x: read the SINGLE common source via
        ``StageController.get_max_xy_speed_um_s`` so the print inherits the same
        XY max as every other page. Falls back to the legacy inline read for a
        controller-like without the resolver (older callers / test stubs), then
        a conservative constant."""
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "get_max_xy_speed_um_s"):
            try:
                v = float(ctrl.get_max_xy_speed_um_s())
                if v > 0:
                    return v / 1000.0
            except Exception:
                pass
        else:
            # Legacy fallback: measured top speed, then safety-limit max.
            try:
                from SupportClasses.PrintTimingCalibrationStore import get_store
                ms = get_store().get_xy_max_speed_um_s()
                if ms and float(ms) > 0:
                    return float(ms) / 1000.0
            except Exception:
                pass
            try:
                v = getattr(getattr(ctrl, "safety_limits", None),
                            "max_xy_speed", None)
                if v is not None and float(v) > 0:
                    return float(v) / 1000.0
            except (TypeError, ValueError):
                pass
        return self._XY_MAX_FALLBACK_MM_S

    def _z_max_mm_s(self) -> float:
        """Calibrated max Z speed (mm/s) = the '100%' anchor for the line-move
        Z speed. v7.5.x: read the SINGLE common source via
        ``StageController.get_max_z_feedrate_mm_min`` (÷60). Falls back to the
        legacy inline read for a controller-like without the resolver, then a
        conservative constant."""
        ctrl = self._controller
        if ctrl is not None and hasattr(ctrl, "get_max_z_feedrate_mm_min"):
            try:
                v = float(ctrl.get_max_z_feedrate_mm_min())
                if v > 0:
                    return v / 60.0
            except Exception:
                pass
        else:
            z_feed_mm_min = None
            try:
                pa = getattr(ctrl, "_pending_per_axis_max_feedrate", None) or {}
                if pa.get("Z"):
                    z_feed_mm_min = float(pa["Z"])
            except (TypeError, ValueError, AttributeError):
                pass
            if not z_feed_mm_min:
                try:
                    v = getattr(getattr(ctrl, "safety_limits", None),
                                "max_z_feedrate", None)
                    if v is not None and float(v) > 0:
                        z_feed_mm_min = float(v)
                except (TypeError, ValueError):
                    pass
            if z_feed_mm_min and z_feed_mm_min > 0:
                return z_feed_mm_min / 60.0
        return self._Z_MAX_FALLBACK_MM_S

    def _top_speed_mm_s(self) -> float:
        """v7.6 parameter 1 — the REQUESTED top XY speed (mm/s), before the
        stage/flow limits are applied. See :meth:`_resolved_print_kinematics`."""
        w = getattr(self, "_top_speed_spin", None)
        try:
            return max(0.05, float(w.value())) if w is not None else 2.5
        except Exception:
            return 2.5

    def _resolution_um(self) -> float:
        """v7.6 parameter 2 — the resolution element (µm) the print must hold.
        Falls back to the machine's stored element, then 30 µm."""
        w = getattr(self, "_resolution_spin", None)
        try:
            if w is not None:
                return max(0.5, float(w.value()))
        except Exception:
            pass
        try:
            return float(get_store().get_resolution_element_um() or 30.0)
        except Exception:
            return 30.0

    def _needle_orifice_area_mm2(self) -> float:
        """Orifice cross-section (mm²) of the configured needle, or 0.0 when no
        needle is configured. = π·(orifice Ø / 2)².

        v7.6: the orifice is the pulled tip on a capillary, else the inner bore —
        the bead is set by what leaves the tip, not by the bulk barrel.
        """
        hw = getattr(self, "_hw_config", None)
        needle = getattr(hw, "needle", None) if hw else None
        if needle is None:
            return 0.0
        try:
            return float(needle_orifice_area_mm2(needle) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def _needle_cross_section_mm2(self) -> float:
        """Legacy name for :meth:`_needle_orifice_area_mm2`.

        v7.7: a DELEGATING method, not a class-level alias. As an alias the two
        names were the same function object, so overriding one of them (which
        every partial-page test does) silently left the other pointing at the
        real implementation — and the flow-ceiling warning, which read the other
        name, quietly stopped firing. Delegation makes an override of either name
        behave the way a reader expects.
        """
        return self._needle_orifice_area_mm2()

    def _extrusion_modifier(self) -> float:
        """Line-thickness multiplier on the auto-calculated extrusion (≥ 0)."""
        w = getattr(self, "_extrusion_mod_spin", None)
        try:
            return max(0.0, float(w.value())) if w is not None else 1.0
        except Exception:
            return 1.0

    def _ink_padding_uL(self) -> float:
        """Extra ink (µL) added to the pickup so the needle never runs dry."""
        w = getattr(self, "_ink_padding_spin", None)
        try:
            return max(0.0, float(w.value())) if w is not None else 0.0
        except Exception:
            return 0.0

    def _max_pump_flow_uL_s(self) -> float:
        """Per-pump needle-derived max flow ceiling (µL/s) from the controller's
        safety limits, or 0.0 when unknown / no limit configured.

        This is the Hagen–Poiseuille ceiling that ``SafetyLimits`` derives from
        the needle bore (see ``SafetyLimits.update_from_hardware_config``); the
        pump move itself is hard-clamped to it. We read it here so the print
        SPEED can be bounded too — keeping the deposited bead correct instead of
        letting the flow be silently clamped (under-extrusion). Guards against a
        bare ``MagicMock`` (whose ``__float__`` is 1.0) by requiring a real
        numeric return, so test stubs don't impose a bogus 1 µL/s limit."""
        sl = getattr(self._controller, "safety_limits", None)
        getter = getattr(sl, "get_max_flow_rate", None)
        if not callable(getter):
            return 0.0
        try:
            raw = getter(self._pump())
        except Exception:
            return 0.0
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return 0.0
        return float(raw) if raw > 0 else 0.0

    def _flow_limited_xy_max_mm_s(self) -> float:
        """XY-max anchor (mm/s) bounded so the auto pump flow at 100% never
        exceeds the needle's max safe flow.

        flow@100% = area × xy_max × modifier; requiring flow@100% ≤ max_flow
        gives xy_max ≤ max_flow / (area × modifier). Returns the unbounded XY
        max when no flow limit / no bore is known, so behaviour is unchanged
        until a needle-derived ceiling exists."""
        xy = self._xy_max_mm_s()
        maxflow = self._max_pump_flow_uL_s()
        area = self._needle_cross_section_mm2()
        # v7.21.5: with a per-segment profile the flow is NOT uniform, so the
        # ceiling has to be sized by the THICKEST segment — the peak is what
        # would over-pressure the needle (a pulled glass tip shatters), and a
        # mean would let it through. ``_flow_modifier_peak`` is the trim alone
        # when there is no profile, so an ordinary print is unchanged.
        mod = self._flow_modifier_peak()
        if maxflow > 0 and area > 0 and mod > 0:
            xy_flow = maxflow / (area * mod)
            if xy_flow < xy:
                return xy_flow
        return xy

    def _flow_modifier_peak(self) -> float:
        """The largest effective extrusion modifier the selected print uses.

        ``Extrusion ×`` (the trim) alone when the object carries no baked
        profile — so behaviour is unchanged for every non-sketch print — and
        ``trim × peak(profile)`` when it does. Read from a cache refreshed in
        :meth:`_refresh_setup_status` (and on every object change), because the
        flow ceiling is consulted from the status refresh and re-deriving the
        geometry there would load the print file again on each pass.
        """
        trim = self._extrusion_modifier()
        peak = getattr(self, "_ext_profile_peak", 0.0) or 0.0
        return trim * peak if peak > 0 else trim

    def _auto_flow_100_uL_s(self) -> float:
        """Auto-calculated pump flow at 100% print speed (µL/s).

        The deposited bead is modelled as a cylinder of the needle's inner-bore
        cross-section run along the path, so the volume per mm of travel =
        ``cross_section_area`` (mm²) and the flow needed to keep up with the
        stage is ``area × speed``. At 100% print speed (= the flow-limited XY
        max), with the operator's extrusion modifier, ``flow@100% = area ×
        xy_max × modifier`` (mm² × mm/s = mm³/s = µL/s). The XY max is bounded by
        :meth:`_flow_limited_xy_max_mm_s` so flow@100% never exceeds the needle's
        safe flow. Falls back to a small constant when no needle is configured
        (its bore is unknown), so a plain print without prep still extrudes."""
        area = self._needle_cross_section_mm2()
        if area <= 0:
            return self._FLOW_FALLBACK_UL_S
        return area * self._flow_limited_xy_max_mm_s() * self._extrusion_modifier()

    def _resolved_print_kinematics(self) -> tuple[float, float, float]:
        """Resolve the operator's TOP SPEED against the hardware limits and
        return ``(print_speed_mm_s, flow_uL_s, prime_uL)``.

        v7.6: the speed parameter is an absolute mm/s (it was a % of the
        measured maximum). The resolved speed is

            ``min(top_speed, measured stage max, needle flow-limited max)``

        — and :meth:`_refresh_setup_status` names whichever term binds, instead
        of clamping silently as before. The flow then FOLLOWS the speed
        (``bore area × speed × modifier``), so the deposited volume-per-mm is
        ``area × modifier`` at any speed: changing the speed changes how long
        the print takes, never how thick the bead is.
        """
        requested = self._top_speed_mm_s()
        # _flow_limited_xy_max_mm_s is already min(stage max, flow ceiling)
        speed = min(requested, self._flow_limited_xy_max_mm_s())
        speed = max(0.05, speed)
        area = self._needle_cross_section_mm2()
        if area > 0:
            flow = area * speed * self._extrusion_modifier()
        else:
            # No needle configured (bore unknown) → keep the legacy fallback
            # flow, scaled by how fast we ended up going.
            xy = max(0.05, self._flow_limited_xy_max_mm_s())
            flow = self._FLOW_FALLBACK_UL_S * min(1.0, speed / xy)
        prime = flow * self._preflow_s()
        return speed, flow, prime

    def _migrate_legacy_settings(self, values: dict) -> dict:
        """v7.6: carry a saved ``speed_pct`` (% of the flow-limited XY max)
        forward into the absolute ``top_speed`` (mm/s) parameter, so existing
        profiles keep printing at the speed they were saved with instead of
        snapping to the new default."""
        try:
            if "top_speed" not in values and "speed_pct" in values:
                pct = max(0.01, min(1.0, float(values["speed_pct"]) / 100.0))
                anchor = self._flow_limited_xy_max_mm_s()
                values["top_speed"] = round(
                    max(0.1, min(20.0, pct * anchor)), 2)
        except (TypeError, ValueError):
            pass
        return values

    def _append_limit_warnings(self, msgs: list, requested: float,
                               res_um: float) -> bool:
        """v7.6: append the hardware-limit warnings + the time estimate.

        Returns True if anything warrants the attention colour. Every message
        names the limiting hardware term and what it costs, because the whole
        point of the two-parameter surface is that the operator can see which
        of their two asks the machine cannot honour.
        """
        warn = False
        # Same accessor the resolved kinematics use (three sibling call sites),
        # so the warning threshold and the flow actually commanded can never
        # disagree about which area the bead is based on.
        area = self._needle_cross_section_mm2()
        mod = self._extrusion_modifier()

        # (0) a pulled tip's ink reserve can be too small to hold the plug back
        reserve_msg = self._reserve_warning()
        if reserve_msg:
            msgs.append(reserve_msg)
            warn = True

        # (a) the needle's flow ceiling — the pump physically cannot keep up
        maxflow = self._max_pump_flow_uL_s()
        if maxflow > 0 and area > 0 and mod > 0:
            attain = maxflow / (area * mod)
            if requested > attain * 1.001:
                msgs.append(
                    f"⚠ Top speed {requested:.2f} mm/s exceeds the needle's "
                    f"flow ceiling ({maxflow:.3g} µL/s ≙ {attain:.2f} mm/s at "
                    f"×{mod:g}) — auto-limited to {attain:.2f} mm/s.")
                warn = True

        # (b) the measured stage maximum. Only warn on a PLAUSIBLE measurement:
        # an unconfigured/zero max would otherwise produce the nonsense
        # "exceeds the measured stage max (0.00 mm/s)".
        xy_max = self._xy_max_mm_s()
        if xy_max >= 0.1 and requested > xy_max * 1.001:
            msgs.append(
                f"⚠ Top speed {requested:.2f} mm/s exceeds the measured stage "
                f"max ({xy_max:.2f} mm/s) — auto-limited to {xy_max:.2f} mm/s.")
            warn = True

        # (c)/(d)/(e) plan-derived: the resolution floor, the calibration gate,
        # and what this resolution costs in time.
        try:
            from SupportClasses import XYFeedPlan as _fp
            from SupportClasses.XYStageModel import StageCharacteristics
            char = StageCharacteristics.from_store(get_store())
        except Exception as e:
            # Logged, not silent: a swallowed error here hides the warnings the
            # operator is relying on (it once hid a NameError for a whole run).
            logger.warning("limit warnings unavailable: %s", e)
            return warn

        if not char.is_complete():
            if self._motion_mode() == "velocity":
                msgs.append(
                    "⚠ Stage motion not characterised — run the one-click XY "
                    "calibration (Timing Calibration) to enable feature-aware "
                    "feed planning; printing with the legacy follower.")
                warn = True
            return warn

        floor = _fp.min_attainable_resolution_um(char)
        if floor > 0 and res_um < floor:
            msgs.append(
                f"⚠ Resolution {res_um:.0f} µm is finer than this machine can "
                f"hold (~{floor:.0f} µm floor: stop tolerance + coast + "
                f"encoder) — corners will exceed it.")
            warn = True

        # (e) the TIME cost of this resolution — the trade the operator is making
        try:
            segments = self._path_segments_for_selection()
            n_pts = sum(len(seg) for seg in segments)
            if segments and n_pts <= 20000:
                # use the REQUESTED speed passed in (resolved against the
                # hardware limits), not a second read of the spin
                speed = min(requested, self._flow_limited_xy_max_mm_s())
                est, stops = 0.0, 0
                for seg in segments:
                    if len(seg) < 2:
                        continue
                    plan = _fp.build_plan(
                        [(float(x), float(y)) for x, y in seg], char,
                        target_speed_mm_s=max(0.05, speed),
                        element_um=res_um)
                    est += plan.est_time_s
                    stops += plan.n_stops
                if est > 0:
                    msgs.append(f"resolution {res_um:.0f} µm → est {est:.0f} s "
                                f"({stops} corner stop"
                                f"{'' if stops == 1 else 's'})")
                    # v7.7: stash so the readiness checklist and the derived-facts
                    # line can report the estimate without re-planning.
                    self._last_est_s, self._last_stops = est, stops
        except Exception:
            pass
        return warn

    def refresh_speed_limits(self) -> None:
        """v7.21.2: re-read everything the XY calibration just changed.

        Called by ``MainWindow._refresh_all_speed_limits`` (via
        ``StageController.notify_speed_limits_changed``) the moment a calibration
        commits, so Quick Print picks the new tuning up WITHOUT a restart: the
        top-speed cap re-anchors to the measured maximum and the "stage motion not
        characterised" warning clears.

        The tuning itself needs no refresh — ``_build_settings`` calls
        ``stamp_print_settings(settings, get_store())`` on every print, so the next
        print reads the freshly persisted values straight out of the per-machine
        store. This method only re-syncs what is DISPLAYED.
        """
        try:
            self._update_top_speed_cap()
        except Exception as e:
            logger.debug("quick print top-speed cap refresh failed: %s", e)
        try:
            self._refresh_setup_status()
        except Exception as e:
            logger.debug("quick print status refresh failed: %s", e)

    def _update_top_speed_cap(self) -> None:
        """Cap the top-speed spin at the MEASURED stage maximum (only when a
        real measurement exists — never at the fallback constant, which would
        wrongly limit an uncalibrated machine)."""
        spin = getattr(self, "_top_speed_spin", None)
        if spin is None:
            return
        measured = 0.0
        try:
            getter = getattr(self._controller, "get_max_xy_speed_um_s", None)
            if callable(getter):
                measured = float(getter() or 0.0) / 1000.0
        except Exception:
            measured = 0.0
        if measured <= 0:
            try:
                measured = float(
                    get_store().get_xy_max_speed_um_s() or 0.0) / 1000.0
            except Exception:
                measured = 0.0
        if measured > 0:
            try:
                spin.setMaximum(max(0.1, min(20.0, measured)))
            except Exception:
                pass

    def _compute_pickup_volume_uL(self) -> float:
        """Volume to aspirate at the ink well = what the print path dispenses
        (path length / print speed × flow, i.e. bore area × modifier × length)
        + the pre-flow prime + the operator's ink padding. The extrusion
        modifier is already folded into the resolved flow, so a thicker line
        automatically picks up more. Returns 0 with no flow."""
        speed, flow, prime = self._resolved_print_kinematics()
        if flow <= 0:
            return 0.0
        try:
            segments = self._path_segments_for_selection()
        except Exception:
            segments = []
        path_len_mm = 0.0
        for seg in segments:
            for i in range(1, len(seg)):
                dx = seg[i][0] - seg[i - 1][0]
                dy = seg[i][1] - seg[i - 1][1]
                path_len_mm += (dx * dx + dy * dy) ** 0.5
        print_time_s = (path_len_mm / speed) if speed > 0 else 0.0
        dispensed = flow * print_time_s
        # Keep a full needle-bore of ink BEHIND the deposit (the bore "dead
        # volume") so the print never dispenses the buffer/oil sitting behind
        # the ink plug. This matters most after a needle prep, which resets the
        # needle to oil + buffer with NO residual ink — so the run relies solely
        # on this pickup, and a pickup equal to just the deposit would leave the
        # buffer right at the tip and print clear oil/buffer once the thin plug
        # is gone. The operator's ink padding is additional reserve on top.
        reserve = self._needle_dead_volume_uL()
        return dispensed + prime + reserve + self._ink_padding_uL()

    def _needle_dead_volume_uL(self) -> float:
        """The ink reserve (µL) kept behind the deposit so the print never
        reaches the buffer/oil. 0 if no needle.

        v7.6: this is ``NeedleSpec.ink_reserve_volume_uL`` — the TIP volume on a
        pulled capillary (the ink that actually sits in the working section;
        reserving the whole 1 mm barrel would exceed a 25 µL syringe) and the
        full bore volume on a straight needle, i.e. exactly the pre-v7.6 number.
        ⚠ On a fine tip the reserve is very small; ``_reserve_warning`` surfaces
        that so the operator can raise the ink padding instead.
        """
        needle = getattr(self._hw_config, "needle", None) if self._hw_config else None
        reserve = getattr(needle, "ink_reserve_volume_uL", None)
        if isinstance(reserve, (int, float)):
            return float(reserve)
        try:
            return float(needle_volume_uL(self._hw_config) or 0.0)
        except Exception:
            return 0.0

    def _reserve_warning(self) -> str:
        """Non-blocking warning when the resolved ink reserve is too small to be
        meaningful — i.e. a pulled tip holds far less than the deposit needs
        behind it. Empty string when the reserve is fine."""
        # getattr-safe on `self` too: this runs from _append_limit_warnings, and
        # the page's own convention is that a partially-built page (tests build
        # `__new__` partials) must degrade to "no warning", not raise.
        hw = getattr(self, "_hw_config", None)
        needle = getattr(hw, "needle", None) if hw else None
        if not getattr(needle, "has_tip", False):
            return ""
        reserve = self._needle_dead_volume_uL()
        padding = self._ink_padding_uL()
        if reserve >= _MIN_MEANINGFUL_RESERVE_UL or padding >= _MIN_MEANINGFUL_RESERVE_UL:
            return ""
        return (f"⚠ Ink reserve is only {reserve * 1000:.2f} nL (the pulled tip's "
                f"volume) — below one pump step. Raise the ink padding so the "
                f"buffer plug can't reach the tip mid-print.")

    def _print_dispense_volume_uL(self) -> float:
        """Volume the print path itself DISPENSES (µL) = flow × print time,
        excluding the prime and the pickup safety factor. Used by the
        syringe-budget pre-flight to model the print's net dispense."""
        speed, flow, _prime = self._resolved_print_kinematics()
        if flow <= 0 or speed <= 0:
            return 0.0
        try:
            segments = self._path_segments_for_selection()
        except Exception:
            segments = []
        path_len_mm = 0.0
        for seg in segments:
            for i in range(1, len(seg)):
                dx = seg[i][0] - seg[i - 1][0]
                dy = seg[i][1] - seg[i - 1][1]
                path_len_mm += (dx * dx + dy * dy) ** 0.5
        return flow * (path_len_mm / speed)

    # ── v7.7: readiness checklist ─────────────────────────────────

    def _readiness_context(self, est_s=None, stops=None):
        """Gather the already-resolved numbers for ``PrintReadiness.evaluate``.

        Everything here is individually try-wrapped: this runs on the status tick
        against a possibly half-configured machine, and a readiness panel that
        raises is worse than one that says "unknown".
        """
        from SupportClasses.PrintReadiness import ReadinessContext
        ctrl = self._controller
        ctx = ReadinessContext()

        def _try(fn, default=None):
            try:
                return fn()
            except Exception:
                return default

        ctx.xy_connected = bool(getattr(ctrl, "is_xy_connected", False))
        ctx.zp_connected = bool(getattr(ctrl, "is_zp_connected", False))
        ctx.object_selected = bool(_try(
            lambda: self._object_combo.currentData(), None))
        ctx.object_label = _try(self._object_label, "") or ""
        ctx.well = getattr(self, "_selected_well", None)
        ctx.plate_available = getattr(self, "_plate", None) is not None
        ctx.well_center_calibrated = _try(
            lambda: (None if not ctx.well else
                     ctx.well in (self._well_positions or {})), None)

        ctx.safe_z = getattr(self, "_safe_z", None)
        ctx.plate_bottom_z = _try(lambda: ctrl.get_plate_bottom_z(), None)
        ctx.print_z_zref = _try(self._resolve_print_z, None)
        if ctx.safe_z is None:
            ctx.travel_z_synthesised = True
            ctx.travel_z_zref = _try(
                lambda: ctrl.default_travel_z(ctx.print_z_zref, margin_mm=10.0),
                None)
        else:
            ctx.travel_z_zref = ctx.safe_z

        # ── machine characterisation + its provenance ──
        try:
            from SupportClasses.XYStageModel import StageCharacteristics
            char = StageCharacteristics.from_store(get_store())
            ctx.char_complete = bool(char.is_complete())
            ctx.char_missing = tuple(char.missing() or ())
            ctx.char_measured_at = getattr(char, "measured_at", None)
        except Exception:
            char = None
        try:
            dt, src = get_store().effective_dead_time_s()
            ctx.dead_time_s, ctx.dead_time_source = dt, str(src)
        except Exception:
            pass
        ctx.motion_mode = _try(self._motion_mode, "") or ""
        ctx.resolved_motion_mode = ctx.motion_mode

        # ── calibration staleness ──
        try:
            from SupportClasses.CalibrationStatusStore import get_store as _cs
            st = _cs()
            ctx.xy_travel_since_cal_mm = float(
                st.xy_travel_since_cal_um() or 0.0) / 1000.0
            ctx.hours_since_xy_cal = st.hours_since("xy")
            thr = st.thresholds()
            ctx.xy_recal_travel_mm = float(thr[0]) if thr else None
            ctx.recal_interval_hours = float(thr[1]) if thr else None
        except Exception:
            pass

        # ── the two parameters, resolved ──
        ctx.requested_speed_mm_s = _try(self._top_speed_mm_s, None)
        kin = _try(self._resolved_print_kinematics, None)
        if kin:
            ctx.resolved_speed_mm_s, ctx.flow_uL_s, _p = kin
        ctx.stage_max_mm_s = _try(self._xy_max_mm_s, None)
        ctx.flow_ceiling_speed_mm_s = _try(self._flow_limited_xy_max_mm_s, None)
        ctx.resolution_um = _try(self._resolution_um, None)
        if char is not None:
            try:
                from SupportClasses import XYFeedPlan as _fp
                floor = _fp.min_attainable_resolution_um(char)
                ctx.resolution_floor_um = floor if floor > 0 else None
            except Exception:
                pass
        ctx.est_time_s, ctx.n_corner_stops = est_s, stops

        # ── geometry ──
        segs = _try(self._path_segments_for_selection, []) or []
        # v7.21.5: refresh the extrusion-profile cache from the SAME geometry
        # pass (the modifiers are recorded beside the segments), so the flow
        # ceiling and the status text agree and neither re-reads the print file.
        try:
            has_prof, lo_mod, hi_mod = self._extrusion_profile_summary()
            self._ext_profile_peak = hi_mod if has_prof else 0.0
            self._ext_profile_span = (lo_mod, hi_mod) if has_prof else None
        except Exception:
            self._ext_profile_peak = 0.0
            self._ext_profile_span = None
        ctx.n_strokes = len(segs) or None
        length = 0.0
        rmax = 0.0
        for seg in segs:
            for i, (px, py) in enumerate(seg):
                rmax = max(rmax, (px * px + py * py) ** 0.5)
                if i:
                    length += ((px - seg[i - 1][0]) ** 2
                               + (py - seg[i - 1][1]) ** 2) ** 0.5
        ctx.path_length_mm = length or None
        # Stashed for the live panel, which needs the WHOLE path's length to
        # state planned volume (a vel_sample's tot_mm is only its section).
        self._last_path_len_mm = length or None
        ctx.object_radius_mm = rmax or None
        wr = _try(lambda: self._well_radius_um(ctx.well), None) if ctx.well else None
        ctx.well_radius_mm = (wr / 1000.0) if wr else None

        # ── needle / bead ──
        hw = getattr(self, "_hw_config", None)
        needle = getattr(hw, "needle", None) if hw else None
        ctx.needle_configured = needle is not None and bool(
            _try(self._needle_cross_section_mm2, 0.0))
        ctx.needle_gauge = str(getattr(needle, "gauge", "") or "")
        ctx.bore_id_um = _try(lambda: float(needle.id_um), None) if needle else None
        area = _try(self._needle_cross_section_mm2, 0.0) or 0.0
        mod = _try(self._extrusion_modifier, 1.0) or 1.0
        span = getattr(self, "_ext_profile_span", None)
        if span:
            # The bead is no longer ONE width: report the effective range and
            # that the × is now a trim, and size the bead readout from the
            # WIDEST segment (the one that has to fit the flow ceiling).
            ctx.extrusion_span = (span[0] * mod, span[1] * mod)
            ctx.extrusion_trim = mod
            mod = span[1] * mod
        if area > 0:
            # An area-equivalent circular bead: Ø = 2·sqrt(A·mod/π).
            ctx.bead_width_um = 2.0 * ((area * mod / 3.141592653589793) ** 0.5) * 1000.0

        # ── fluidics (the full run budget is still verified at Print) ──
        pump = _try(self._pump, "P1") or "P1"
        ctx.pump = pump
        ctx.pump_plunger_calibrated = _try(
            lambda: bool(ctrl.is_pump_plunger_calibrated(pump)), None)
        ctx.syringe_capacity_uL = _try(lambda: ctrl.pump_capacity_uL(pump), None)
        ctx.syringe_fill_uL = _try(lambda: ctrl.pump_fill_uL(pump), None)
        ctx.pickup_uL = _try(self._compute_pickup_volume_uL, None)
        ctx.pickup_dispense_uL = _try(self._print_dispense_volume_uL, None)
        ctx.pickup_dead_volume_uL = _try(self._needle_dead_volume_uL, None)
        ctx.pickup_padding_uL = _try(self._ink_padding_uL, None)
        if kin:
            ctx.pickup_prime_uL = kin[2]

        # ── ink: the bioprinting advisories that never ran ──
        ink_name = _try(self._selected_ink, None)
        ctx.ink_name = ink_name
        if ink_name:
            ctx.ink_well = _try(self._ink_source_well, None)
            ctx.ink_well_calibrated = _try(
                lambda: (ctx.ink_well in (self._well_positions or {}))
                if ctx.ink_well else None, None)
        ink = None
        if ink_name and hw is not None:
            ink = _try(lambda: (getattr(hw, "ink_library", {}) or {}).get(ink_name),
                       None)
        if ink is not None and needle is not None:
            ctx.clog = _try(lambda: ink.flow_compatibility_detail(needle), None)
            ctx.ink_particle_um = _try(
                lambda: float(ink.max_particle_diameter_um or 0.0), None)
            self._fill_flow_physics(ctx, needle, ink)

        # ── prep / cleanup prerequisites ──
        ctx.prep_enabled = bool(_try(lambda: self._prep_check.isChecked(), False))
        ctx.cleanup_enabled = bool(
            _try(lambda: self._postclean_check.isChecked(), False))
        if ctx.prep_enabled or ctx.cleanup_enabled:
            missing = _try(
                lambda: resolve_service_positions(
                    self._hw_config, self._well_positions, self._plate)[1],
                ()) or ()
            if ctx.prep_enabled:
                ctx.prep_missing = tuple(missing)
            if ctx.cleanup_enabled:
                ctx.cleanup_missing = tuple(
                    m for m in missing if m in ("waste", "wash", "oil"))
        return ctx

    def _fill_flow_physics(self, ctx, needle, ink) -> None:
        """Wall shear vs the cell-viability limit, and the flow ceiling
        recomputed with THIS ink's viscosity instead of the reference fluid the
        enforced ceiling assumes."""
        try:
            from SupportClasses import FlowPhysics as FP
        except Exception:
            return
        flow = ctx.flow_uL_s or 0.0
        try:
            visc = float(getattr(ink, "viscosity_Pa_s", 0.0) or 0.0)
            d_m = float(needle.id_m)
            if visc > 0 and d_m > 0 and flow > 0:
                ctx.wall_shear_pa = FP.wall_shear_stress(
                    visc, flow * 1e-9, d_m)          # µL/s → m³/s
                ctx.shear_limit_pa = FP.DEFAULT_SHEAR_STRESS_LIMIT_PA
        except Exception:
            pass
        try:
            ctx.flow_ceiling_ink_uL_s = FP.max_safe_flow_rate_uL_s(needle, ink)
            ctx.flow_ceiling_ref_uL_s = self._max_pump_flow_uL_s() or None
        except Exception:
            pass

    def _render_readiness(self, ctx) -> "object":
        """Evaluate + paint the checklist. Returns the Readiness so the button
        state and the compact summary can use the SAME evaluation."""
        from SupportClasses.PrintReadiness import evaluate, BLOCK, WARN, OK, INFO
        readiness = evaluate(ctx)
        host = getattr(self, "_ready_layout", None)
        if host is None:
            return readiness
        while host.count():
            it = host.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
        variant = {BLOCK: "err", WARN: "warn", OK: "ok", INFO: "info"}
        for group, checks in readiness.by_group():
            # Only groups that need attention are expanded by default; an all-OK
            # group collapses to one line so the panel stays scannable.
            attention = [c for c in checks if c.state in (BLOCK, WARN)]
            hdr = QLabel(group)
            hdr.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(8)}pt; "
                f"font-weight: 600; letter-spacing: 1px;")
            host.addWidget(hdr)
            shown = checks if attention else checks[:0]
            if not attention:
                row = QWidget()
                rl = QHBoxLayout(row)
                rl.setContentsMargins(0, 0, 0, 0)
                rl.setSpacing(s(6))
                rl.addWidget(StatusBadge("ok", variant="ok"))
                lbl = QLabel(", ".join(c.label for c in checks))
                lbl.setWordWrap(True)
                lbl.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
                rl.addWidget(lbl, stretch=1)
                host.addWidget(row)
            for c in shown:
                row = QWidget()
                rl = QHBoxLayout(row)
                rl.setContentsMargins(0, 0, 0, 0)
                rl.setSpacing(s(6))
                rl.addWidget(StatusBadge(
                    {BLOCK: "blocked", WARN: "check", OK: "ok",
                     INFO: "fyi"}[c.state],
                    variant=variant.get(c.state, "info")))
                text = f"<b>{c.label}</b> — {c.detail}" if c.detail else \
                    f"<b>{c.label}</b>"
                if c.fix:
                    text += (f" <span style='color:{COLORS['overlay0']}'>"
                             f"({c.fix})</span>")
                lbl = QLabel(text)
                lbl.setWordWrap(True)
                lbl.setTextFormat(Qt.TextFormat.RichText)
                lbl.setStyleSheet(
                    f"color: {COLORS['text']}; font-size: {sf(9)}pt;")
                rl.addWidget(lbl, stretch=1)
                host.addWidget(row)
        return readiness

    def _render_derived(self, ctx) -> None:
        """The facts the page computed and used to discard."""
        lbl = getattr(self, "_derived_lbl", None)
        if lbl is None:
            return
        bits = []
        if ctx.resolved_speed_mm_s:
            bits.append(f"{ctx.resolved_speed_mm_s:.2f} mm/s")
        if ctx.flow_uL_s:
            bits.append(f"{ctx.flow_uL_s:.3f} µL/s")
        if ctx.bead_width_um:
            bits.append(f"bead ~{ctx.bead_width_um:.0f} µm")
        if ctx.path_length_mm:
            n = ctx.n_strokes or 1
            bits.append(f"{ctx.path_length_mm:.1f} mm in {n} stroke"
                        f"{'' if n == 1 else 's'}")
        if ctx.est_time_s:
            bits.append(f"est {ctx.est_time_s:.0f} s")
        if ctx.n_corner_stops:
            bits.append(f"{ctx.n_corner_stops} corner stop"
                        f"{'' if ctx.n_corner_stops == 1 else 's'}")
        if ctx.print_z_zref is not None:
            bits.append(f"print Z {ctx.print_z_zref:.3f} mm")
        lbl.setText(" · ".join(bits) or "Pick an object and a well.")

    def _check_syringe_budget(self, pump, prep_enabled, ink_on, pickup_uL,
                              settings, cleanup_enabled, cleanup_ctx):
        """Pre-flight the FULL run's pump moves against the plunger envelope.

        Assembles the ordered signed volumes (prep dispense/aspirate → ink
        pickup → print prime + path dispense), folds in the post-print reset
        trough (``baseline − oil margin``), and simulates via
        ``StageController.simulate_pump_budget``. Returns the budget dict ONLY
        when it is ACTIONABLE (calibrated, live fill readable, out of bounds);
        returns None to proceed (in-bounds, uncalibrated, or fill unreadable)."""
        ctrl = self._controller
        try:
            start_fill = ctrl.pump_fill_uL(pump)
        except Exception:
            start_fill = None
        if start_fill is None:
            return None
        try:
            needle_uL = needle_volume_uL(self._hw_config)
        except Exception:
            needle_uL = 0.0
        moves = []
        if prep_enabled and needle_uL > 0:
            # prep: dispense 1 needle of oil → aspirate 1 needle of oil →
            # aspirate buffer. Oil count = the executor default (1 needle);
            # Quick Print exposes only the buffer count.
            try:
                buf = float(self._buffer_needles_spin.value())
            except Exception:
                buf = 1.0
            moves += [+needle_uL, -needle_uL, -buf * needle_uL]
        if ink_on and pickup_uL > 0:
            moves += [-float(pickup_uL)]              # aspirate ink
        try:
            prime_uL = float(settings.prime_amounts_uL.get(pump, 0.0))
        except Exception:
            prime_uL = 0.0
        if prime_uL > 0:
            moves += [+prime_uL]
        printed_uL = self._print_dispense_volume_uL()
        if printed_uL > 0:
            moves += [+printed_uL]
        # The post-print reset dips the plunger to (baseline − oil margin) before
        # topping oil back up. That trough does NOT shift with a starting-oil
        # remedy, so fold it in as a fixed low-water mark, not a shiftable move.
        extra_min = None
        if cleanup_enabled and cleanup_ctx and cleanup_ctx.get("reset_to_initial"):
            extra_min = start_fill - float(cleanup_ctx.get("oil_margin_uL", 0.0))
        try:
            budget = ctrl.simulate_pump_budget(
                pump, moves, start_fill_uL=start_fill,
                extra_min_fill_uL=extra_min)
        except Exception as e:
            logger.warning("Quick Print syringe budget skipped: %s", e)
            return None
        if budget.get("ok", True) or budget.get("reason") != "out_of_bounds":
            return None
        return budget

    def _offer_oil_remedy(self, budget):
        """Ask whether to waste / aspirate oil to a feasible starting fill.
        Returns a ``starting_oil`` dict to apply before the run, or None if the
        operator declines or the remedy well isn't assigned + calibrated."""
        remedy = budget.get("remedy")
        rem_uL = float(budget.get("remedy_uL", 0.0))
        rec_fill = float(budget.get("recommended_start_fill_uL", 0.0))
        start_fill = float(budget.get("start_fill_uL", 0.0))
        over = (remedy == "waste_oil")
        role = "waste" if over else "oil"
        positions, _missing = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        service_z = self._plate_offset_to_zref(
            float(self._service_z_spin.value()))
        if role not in positions or service_z is None or self._safe_z is None:
            QMessageBox.warning(
                self, "Syringe won't fit at the current fill",
                ("This run would over-fill the syringe by " if over
                 else "This run would run the syringe dry by ")
                + f"{rem_uL:.2f} µL. To auto-correct it I'd "
                + ("waste" if over else "aspirate")
                + f" {rem_uL:.2f} µL of oil first, but that needs a Safe Z and "
                f"the {role} well assigned + calibrated (and the plate bottom Z "
                "set). Set that up, or reduce the print / pickup / prep volumes.",
                QMessageBox.StandardButton.Ok)
            self._status.setText(
                f"Syringe over budget — needs the {role} well to auto-correct.")
            return None
        verb = (f"waste {rem_uL:.2f} µL of oil to the waste well" if over
                else f"aspirate {rem_uL:.2f} µL of oil from the oil well")
        resp = QMessageBox.question(
            self, "Adjust the starting oil?",
            "The syringe can't hold this whole run at its current fill "
            f"({start_fill:.2f} µL): it would "
            + ("over-fill by " if over else "run dry by ")
            + f"{rem_uL:.2f} µL.\n\nI can {verb} first so it starts at "
            f"{rec_fill:.2f} µL, which fits. Proceed?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if resp != QMessageBox.StandardButton.Yes:
            self._status.setText("Cancelled — syringe over budget.")
            return None
        return {
            "volume_uL": rem_uL,
            "dispense_to_waste": over,
            "role": role,
            "well_pos": positions[role],
            "service_z": service_z,
        }

    def _plate_offset_to_zref(self, offset_mm: float) -> float | None:
        """Height above the calibrated plate bottom (mm) → zero-ref Z (mm),
        polarity-correct. Returns None if the plate bottom isn't calibrated.
        (Mirror of the spheroid page helper.)"""
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

    def _preamble_active(self) -> bool:
        """True when Print should run a pick-and-place preamble (needle prep
        and/or ink pickup) before the print itself."""
        return self._prep_check.isChecked() or (self._selected_ink() is not None)

    def _refresh_setup_status(self) -> None:
        """Update the 'confirm all is setup' status line: ink ← source well +
        pickup volume, prep service-well mapping, and any ⚠ gaps. Warnings turn
        the line peach; an all-clear (or plain print) turns it green/subtext."""
        if not hasattr(self, "_setup_status"):
            return
        # v7.5.x: multi-ink (abstract-ink) sketch → show the ink mapping + any
        # gaps; the run does sequential swaps (no single ink/pump applies).
        if self._is_multi_ink():
            sk = self._loaded_sketch
            parts = []
            for iid in self._used_abstract_inks():
                ink = sk.ink_by_id(iid) if sk else None
                label = ink.name if ink else f"Ink {iid}"
                mapped = self._ink_map.get(iid)
                parts.append(f"{label}→{mapped}" if mapped else f"{label}→?")
            warns = self._validate_ink_map()
            text = "Multi-ink (sequential swaps): " + ", ".join(parts)
            if warns:
                text += "   ⚠ " + "; ".join(warns)
            self._setup_status.setText(text)
            self._setup_status.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af') if warns else COLORS['green']}; "
                f"font-size: {sf(9)}pt;")
            return
        msgs: list[str] = []
        warn = False
        # v7.6: the two driving parameters, what they resolved to, and WHY —
        # every limit that bites is named instead of silently clamping.
        try:
            requested = self._top_speed_mm_s()
            speed, flow, _prime = self._resolved_print_kinematics()
            res_um = self._resolution_um()
            msgs.append(f"Print: {speed:.2f} mm/s · {flow:.3f} µL/s · "
                        f"res {res_um:.0f} µm")
            if self._append_limit_warnings(msgs, requested, res_um):
                warn = True
        except Exception:
            pass
        ink = self._selected_ink()
        if ink:
            well = self._ink_source_well()
            pos = self._ink_source_pos()
            if well is None:
                msgs.append(f"⚠ Ink “{ink}” has no reagent location "
                            "(Hardware Setup → Ink).")
                warn = True
            elif pos is None:
                msgs.append(f"⚠ Ink well {well} not in the calibrated plate — "
                            "run Plate Location.")
                warn = True
            else:
                vol = self._compute_pickup_volume_uL()
                msgs.append(f"Ink “{ink}” ← {well} · pick up ~{vol:.3f} µL")
        else:
            msgs.append("No ink pickup — printing with whatever is loaded.")

        if self._prep_check.isChecked():
            nv = needle_volume_uL(self._hw_config)
            _positions, missing = resolve_service_positions(
                self._hw_config, self._well_positions, self._plate)
            names = service_well_names(self._hw_config, self._plate)
            if nv <= 0:
                msgs.append("⚠ Prep on: needle inner Ø/length not set "
                            "(Hardware Setup → Needle).")
                warn = True
            elif missing:
                msgs.append("⚠ Prep on: assign + calibrate wells for "
                            f"{', '.join(missing)}.")
                warn = True
            else:
                mapping = " ".join(f"{r}={names.get(r, '?')}"
                                   for r in SERVICE_ROLES)
                msgs.append(f"✓ Prep: 1 needle = {nv:.3f} µL · {mapping}")

        postclean_on = (getattr(self, "_postclean_check", None) is not None
                        and self._postclean_check.isChecked())
        if postclean_on:
            nv = needle_volume_uL(self._hw_config)
            positions, _m = resolve_service_positions(
                self._hw_config, self._well_positions, self._plate)
            cl_need = [r for r in ("waste", "wash", "oil") if r not in positions]
            if nv <= 0:
                msgs.append("⚠ Clean-after on: needle inner Ø/length not set.")
                warn = True
            elif cl_need:
                msgs.append("⚠ Clean-after on: assign + calibrate "
                            f"{', '.join(cl_need)}.")
                warn = True
            else:
                msgs.append(
                    "✓ Reset to initial: waste unprinted ink+buffer → wash → "
                    "top up oil")

        # Calibration prerequisites for any well descent (preamble or cleanup).
        active = self._preamble_active() or postclean_on
        if active:
            if self._safe_z is None:
                msgs.append("⚠ Safe Z not set (Calibration).")
                warn = True
            if self._plate_offset_to_zref(0.0) is None:
                msgs.append("⚠ Plate bottom Z not calibrated (Calibration).")
                warn = True

        color = COLORS['peach'] if warn else (
            COLORS['green'] if active else COLORS['subtext0'])
        self._setup_status.setText("   ".join(msgs))
        self._setup_status.setStyleSheet(
            f"color: {color}; font-size: {sf(9)}pt;")
        # v7.7: the structured checklist replaces this run-on line as the primary
        # surface (the line stays as the compact summary, and existing tests
        # assert against it). Best-effort: a readiness failure must not stop the
        # page refreshing.
        try:
            self._refresh_readiness(est_s=self._last_est_s,
                                    stops=self._last_stops)
        except Exception:
            logger.exception("readiness refresh failed")

    def _refresh_readiness(self, est_s=None, stops=None) -> None:
        """Evaluate readiness once and use it for the checklist, the derived
        facts and the Print button — one evaluation, three consumers."""
        ctx = self._readiness_context(est_s=est_s, stops=stops)
        self._readiness = self._render_readiness(ctx)
        self._render_derived(ctx)
        self._update_button_state()

    # ── Well center resolution (zero-ref mm) ──────────────────────

    def _pump(self) -> str:
        return self._pump_combo.currentText() or "P1"

    def _well_center_zero_ref_mm(self, well: str):
        """Calibrated position (preferred) → zero-ref mm, else geometric."""
        if self._well_positions and well in self._well_positions:
            try:
                zero = self._controller.zero_position
                wx_um, wy_um = self._well_positions[well]
                return ((wx_um - zero["x"]) / 1000.0,
                        (wy_um - zero["y"]) / 1000.0)
            except Exception:
                pass
        if self._plate is not None:
            try:
                # Geometric fallback (no calibration): map the plate-local
                # offset onto the stage axes with the per-machine sign so it
                # lands on the physically-correct well (ME3B V1). A malformed
                # sign degrades to aligned (1, 1), never crashes.
                wx, wy = self._plate.get_well_position(well)
                try:
                    sign = self._controller.plate_axis_sign()
                    sx, sy = float(sign[0]), float(sign[1])
                except Exception:
                    sx, sy = 1.0, 1.0
                return (sx * wx, sy * wy)
            except Exception:
                pass
        return None

    def _resolve_print_z(self) -> float:
        """The spin value is a *height above the plate bottom*. Convert it to a
        zero-ref Z via the controller's calibrated plate-bottom datum. Falls
        back to the raw value if the plate bottom is not calibrated."""
        height = float(self._printz_spin.value())
        try:
            z = self._controller.print_height_to_zref(height)
            if z is not None:
                return float(z)
        except Exception:
            pass
        return height

    def _confirm_print_floor(self) -> bool:
        """Warn if the resolved print Z would punch through the plate bottom.

        Returns True to proceed (no violation, or the user accepted the
        warning), False to cancel. The controller hard-clamps regardless.
        """
        try:
            if not self._controller.print_floor_violation(self._resolve_print_z()):
                return True
        except Exception:
            return True
        resp = QMessageBox.warning(
            self, "Below plate bottom",
            "The chosen height is below the calibrated plate bottom, so the "
            "needle would be clamped to the plate bottom (it will not print "
            "deeper). Continue anyway?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return resp == QMessageBox.StandardButton.Yes

    def _motion_mode(self) -> str:
        """Selected print-path motion mode: 'open_loop' (default), 'velocity',
        or 'confirm'. Getattr-safe for a partially-built page / tests."""
        w = getattr(self, "_motion_mode_combo", None)
        try:
            return w.currentData() or "velocity"
        except Exception:
            return "velocity"

    def _build_settings(self, pump: str | None = None) -> PrintSettings:
        # Print speed % scales the XY traverse AND the pump flow together.
        # ``pump`` overrides which pump the flow/prime are stamped for (used by
        # the multi-ink path per group); the flow VALUE is bore-based and the
        # same for every pump, so this only routes the stamping.
        print_speed_mm_s, flow, prime_uL = self._resolved_print_kinematics()
        pump = pump or self._pump()
        # v7.5.x CRITICAL SAFETY: never fall back to a raw literal travel Z.
        # On ME3B V1 (ZDIR=-1) a raw 5.0 is a DESCENT toward the plate, not a
        # retract. When no Safe Z is calibrated, derive a polarity-safe travel
        # height 10 mm ABOVE the print Z so the per-well TRAVEL_UP / HOME_XY
        # retract is genuine (the discrete MOVE_XY/HOME_XY also self-retract).
        travel_z = self._safe_z
        if travel_z is None:
            try:
                travel_z = self._controller.default_travel_z(
                    self._resolve_print_z(), margin_mm=10.0)
            except Exception:
                travel_z = self._resolve_print_z()
        # Travel speed is a settings-popout knob; tolerate a partially-built page
        # (the same defensive contract as _print_speed_pct / _xy_max_mm_s).
        travel_w = getattr(self, "_travel_speed", None)
        try:
            travel_speed_mm_s = float(travel_w.value()) if travel_w else 10.0
        except Exception:
            travel_speed_mm_s = 10.0
        settings = PrintSettings(
            num_layers=1,
            travel_z_height=travel_z,
            print_z_height=self._resolve_print_z(),
            pump_rate_uL_s=flow,
            print_speed_mm_s=print_speed_mm_s,
            travel_speed_mm_s=travel_speed_mm_s,
        )
        # v7.5.x: print-path motion mode (operator-selectable, A/B on hardware).
        # open_loop = original streamed path; velocity = closed-loop follower;
        # confirm = stop-and-go per segment. Default open_loop = the behaviour
        # that worked on the other setup.
        _mode = self._motion_mode()
        settings.velocity_follow = (_mode == "velocity")
        settings.confirm_each_segment = (_mode == "confirm")
        # open_loop now = OPEN-LOOP velocity streaming (feed-forward velocity
        # vectors along the path), NOT point-to-point moves (that's confirm).
        settings.velocity_open_loop = (_mode == "open_loop")
        # v7.5.x: stamp the per-mode path-following params tuned by the XY
        # Printing Challenge (PrintTimingCalibrationStore) so the real print uses
        # them. Best-effort; defaults preserve legacy behaviour.
        # v7.5.x: ONE shared stamper (XYAutoCalibration.stamp_print_settings) is
        # used by BOTH print paths, so a bench tuning session reaches every print
        # rather than only this one. Previously Full Print stamped none of it.
        try:
            from SupportClasses.PrintTimingCalibrationStore import get_store
            from SupportClasses import XYAutoCalibration as _AC
            _AC.stamp_print_settings(settings, get_store())
        except Exception:
            pass
        # v7.6: the feature-aware FEED PLAN — the second of the two driving
        # parameters reaches the executor here. Only meaningful in closed-loop
        # velocity mode; the executor itself falls back to the legacy follower
        # when the machine is not characterised.
        try:
            settings.feed_plan_enabled = (self._motion_mode() == "velocity")
            settings.feed_plan_element_um = float(self._resolution_um())
        except Exception:
            pass
        # v7.5.x: stamp the reference-vector up-direction (print_z_height above
        # is already polarity-correct via controller.print_height_to_zref).
        try:
            settings.z_up_sign = float(self._controller.print_z_dir())
        except (TypeError, ValueError, AttributeError):
            pass  # keep the PrintSettings default (legacy additive)
        # v7.5.x: stamp the per-machine plate orientation (only affects a
        # GEOMETRIC well-centre fallback; the calibrated well centre below is
        # already in the correct frame and is NOT re-signed).
        try:
            settings.plate_axis_sign = self._controller.plate_axis_sign()
        except (TypeError, ValueError, AttributeError):
            pass
        try:
            settings.pump_rates_uL_s[pump] = flow
        except Exception:
            pass
        # v7.5.x print-setup routine step 4: "start pump flow and wait 0.25 s"
        # before the trajectory begins — a pre-flow lead-in so material is
        # already forming a bead when the path starts. Implemented via the
        # standard prime: extrude (flow × 0.25 s) µL at the print flow rate, so
        # the pump runs for ~0.25 s in the MOVE_Z → PRINT_PATH gap.
        # build_well_plate_job emits this as the EXTRUDE prime right before the
        # PRINT_PATH (after the confirmed descent to print Z).
        try:
            settings.prime_amounts_uL[pump] = prime_uL
        except Exception:
            pass
        # v7.5.x Feature 3: per-line retract height + fast inter-line move speeds.
        # build_well_plate_job lifts to (print Z + intra_well_hop_z_mm) between
        # sub-paths and uses the line speeds for that hop's lift / XY / lower.
        # The line-move speeds are entered as a % of each stage's calibrated max
        # and resolved to mm/s here (PrintSettings stays in mm/s).
        try:
            settings.intra_well_hop_z_mm = float(self._line_retract_spin.value())
        except Exception:
            pass
        try:
            pct_z = max(0.0, float(self._line_z_speed_spin.value())) / 100.0
            settings.line_move_z_speed_mm_s = pct_z * self._z_max_mm_s()
        except Exception:
            pass
        try:
            pct_xy = max(0.0, float(self._line_xy_speed_spin.value())) / 100.0
            settings.line_move_xy_speed_mm_s = pct_xy * self._xy_max_mm_s()
        except Exception:
            pass
        # NOTE: quick-move pressure relief (suck-back before each inter-object
        # hop) is handled at EXECUTION by PrintManager._print_pump_suckback
        # ("quick_move"), which fires on every hop MOVE_XY this run's path-split
        # creates — no per-run setting needed here.
        return settings

    # ── Run / abort ───────────────────────────────────────────────

    def _is_running(self) -> bool:
        return self._pm is not None and getattr(
            self._pm, "state", None) == PrintState.RUNNING

    def _preposition_for_print(self, start_zref_mm, travel_z) -> bool:
        """Retract the needle and travel (XY only, needle stays up) to the
        print-start point so the operator can confirm the position on the
        microscope before any extrusion.

        Blocks until the move completes — the same pattern the Calibration page
        uses for ``safe_travel_to`` from a GUI handler. ``start_zref_mm`` is the
        print-start in zero-ref mm; ``travel_z`` is the safe travel Z (zero-ref
        mm). Delegates to :meth:`StageController.safe_travel_to` with
        ``target_z_mm=None`` so it raise→wait→XY→wait and NEVER lowers the
        needle. Returns True on confirmed arrival, False on timeout/error.
        """
        ctrl = self._controller
        try:
            zero = ctrl.zero_position
            x_um = start_zref_mm[0] * 1000.0 + zero["x"]
            y_um = start_zref_mm[1] * 1000.0 + zero["y"]
        except Exception as e:
            logger.warning("Quick Print pre-position target failed: %s", e)
            return False
        try:
            return bool(ctrl.safe_travel_to(
                x_um, y_um, safe_z_mm=float(travel_z), target_z_mm=None))
        except Exception as e:
            logger.exception("Quick Print pre-position move failed: %s", e)
            return False

    # ── Multi-ink (abstract-ink) sketch: detect / map / group / run ────

    def _ink_color(self, ink_name: str) -> str | None:
        hw = self._hw_config
        lib = getattr(hw, "ink_library", {}) if hw else {}
        spec = (lib or {}).get(ink_name)
        return getattr(spec, "color", None) if spec is not None else None

    def _load_selected_sketch(self):
        """The vector Sketch embedded in the selected saved print, or None
        (built-in shape / CSV / image import / no embedded sketch)."""
        data = self._parse_obj_data(self._object_combo.currentData())
        if not data or data[0] != "file":
            return None
        try:
            pf = self._print_mgr.load(data[1])
        except Exception:
            return None
        if pf is None:
            return None
        for od in (getattr(pf, "objects", None) or {}).values():
            params = (od or {}).get("params") or {}
            sk = params.get("sketch")
            if isinstance(sk, dict) and "shapes" in sk:
                try:
                    from SupportClasses.SketchTrajectory import Sketch
                    return Sketch.from_dict(sk)
                except Exception:
                    return None
        return None

    def _used_abstract_inks(self) -> list[int]:
        """Distinct abstract ink ids used by PRINTING shapes, in first-use
        order. Empty for a non-sketch object."""
        sk = self._loaded_sketch
        if sk is None:
            return []
        seen: set = set()
        out: list[int] = []
        for sh in sk.shapes:
            if (getattr(sh, "kind", None) == "travel"
                    or getattr(sh, "no_print", False)):
                continue
            iid = int(getattr(sh, "ink_id", 1))
            if iid not in seen:
                seen.add(iid)
                out.append(iid)
        return out

    def _is_multi_ink(self) -> bool:
        """True when the loaded sketch prints with ≥2 abstract inks (→ the
        sequential ink-swap path). Single-ink / non-sketch = the legacy flow."""
        return len(self._used_abstract_inks()) >= 2

    def _rebuild_ink_mapping_ui(self) -> None:
        """Rebuild the per-abstract-ink → configured-ink mapping rows for the
        loaded sketch. v7.7: the chosen mapping is remembered by ink NAME in
        ``_ink_map_last``, which now rides along with the settings profile (see
        ``WorkflowSettingsDialog.set_extra_state``), so it survives a restart."""
        if not hasattr(self, "_ink_map_layout"):
            return
        while self._ink_map_layout.count():
            it = self._ink_map_layout.takeAt(0)
            w = it.widget()
            if w is not None:
                w.setParent(None)
        self._ink_map_combos = {}
        self._ink_map = {}
        sk = self._loaded_sketch
        used = self._used_abstract_inks()
        if sk is None or len(used) < 2:
            lbl = QLabel("(loaded print uses a single ink — no mapping needed)")
            lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
            self._ink_map_layout.addWidget(lbl)
            return
        printable = self._printable_inks()
        for iid in used:
            ink = sk.ink_by_id(iid)
            name = (ink.name if ink else f"Ink {iid}")
            color = (ink.color if ink else "#89b4fa")
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(s(6))
            sw = QLabel()
            sw.setFixedSize(s(14), s(14))
            sw.setStyleSheet(f"background: {color}; border-radius: {s(3)}px;")
            h.addWidget(sw)
            nl = QLabel(name)
            nl.setMinimumWidth(s(70))
            h.addWidget(nl)
            combo = QComboBox()
            combo.addItem("(choose ink)", "")
            for pn in printable:
                combo.addItem(pn, pn)
            default = self._ink_map_last.get(name)
            if default not in printable:
                default = next((pn for pn in printable
                                if self._ink_color(pn) == color), None) \
                    or (printable[0] if printable else None)
            if default:
                idx = combo.findData(default)
                combo.setCurrentIndex(idx if idx >= 0 else 0)
                self._ink_map[iid] = default
                self._ink_map_last[name] = default
            combo.currentIndexChanged.connect(
                lambda _i, c=combo, i=iid, nm=name:
                self._on_ink_map_changed(i, nm, c.currentData()))
            h.addWidget(combo, 1)
            self._ink_map_layout.addWidget(row)
            self._ink_map_combos[iid] = combo

    def _collect_extra_state(self) -> dict:
        """v7.7/v7.19: page state that is not a registered widget, saved with the
        profile — the remembered ink mapping and the whole plate-wide queue."""
        out: dict = {"ink_map_last": dict(self._ink_map_last)}
        try:
            out["print_queue"] = queue_to_state(self._queue)
            out["queue_group_by_ink"] = self._queue_group_by_ink()
            out["queue_clean_between"] = self._queue_clean_between()
        except Exception as exc:
            logger.debug("queue state collect failed: %s", exc)
        return out

    def _restore_extra_state(self, extra: dict) -> None:
        """v7.7: restore the remembered abstract-ink → configured-ink mapping
        from the loaded profile, then rebuild the rows so they show it.
        v7.19: and the plate-wide print queue."""
        if not isinstance(extra, dict):
            return
        remembered = extra.get("ink_map_last")
        if isinstance(remembered, dict):
            self._ink_map_last = {str(k): str(v)
                                  for k, v in remembered.items() if v}
            if hasattr(self, "_ink_map_layout"):
                self._rebuild_ink_mapping_ui()
        try:
            # Entries for wells that are not on the CURRENT plate are KEPT, not
            # dropped: the operator may be loading a 96-well profile with a
            # 24-well plate mounted, and silently destroying their layout would be
            # unrecoverable. They are flagged on the plate and refused BY NAME at
            # run time instead.
            self._queue = queue_from_state(extra.get("print_queue"))
        except Exception as exc:
            logger.debug("queue state apply failed: %s", exc)
            self._queue = []
        for key, name in (("queue_group_by_ink", "_group_by_ink_check"),
                          ("queue_clean_between", "_clean_between_check")):
            w = getattr(self, name, None)
            if w is not None and key in extra:
                w.blockSignals(True)
                w.setChecked(bool(extra.get(key)))
                w.blockSignals(False)
        self._glyph_cache.clear()
        self._refresh_queue_ui()

    def _on_ink_map_changed(self, ink_id: int, name: str, value) -> None:
        if value:
            self._ink_map[int(ink_id)] = value
            self._ink_map_last[name] = value
        else:
            self._ink_map.pop(int(ink_id), None)
        self._refresh_setup_status()
        self._update_button_state()

    def _validate_ink_map(self) -> list[str]:
        """Warnings for the current mapping (empty = OK to run)."""
        hw = self._hw_config
        printable = set(self._printable_inks())
        wells = self._well_positions or {}
        pumps = (getattr(hw, "pumps", {}) or {}) if hw else {}
        locs = (getattr(hw, "ink_locations", {}) or {}) if hw else {}
        warns: list[str] = []
        sk = self._loaded_sketch
        for iid in self._used_abstract_inks():
            ink = sk.ink_by_id(iid) if sk else None
            label = (ink.name if ink else f"Ink {iid}")
            mapped = self._ink_map.get(iid)
            if not mapped:
                warns.append(f"“{label}” not mapped")
                continue
            if mapped not in printable:
                warns.append(f"“{mapped}” has no reagent location")
                continue
            pump = hw.get_pump_for_ink(mapped) if hw else None
            if not pump:
                warns.append(f"“{mapped}” has no pump")
                continue
            pcfg = pumps.get(pump)
            if pcfg is None or getattr(pcfg, "syringe", None) is None:
                warns.append(f"{pump} has no syringe")
                continue
            wl = locs.get(mapped) or []
            wn = resolve_pickup_well(wl, self._plate)
            if not wn or wn not in wells:
                warns.append(f"“{mapped}” well not calibrated")
        return warns

    def _ink_groups(self):
        """Partition the loaded sketch into ordered ink-contiguous groups →
        list of ``(ink_id, sub_Sketch)`` (a maximal run of consecutive shapes
        sharing one abstract ink). Travel markers ride with the following
        group; all-``no_print`` groups are dropped."""
        sk = self._loaded_sketch
        if sk is None:
            return []
        groups: list = []                         # [(ink_id, [shapes])]
        cur_id = None
        pending: list = []
        for sh in sk.shapes:
            if getattr(sh, "kind", None) == "travel":
                pending.append(sh)
                continue
            iid = int(getattr(sh, "ink_id", 1))
            if iid != cur_id:
                groups.append((iid, list(pending)))
                cur_id = iid
            else:
                groups[-1][1].extend(pending)
            pending = []
            groups[-1][1].append(sh)
        out = []
        for iid, shapes in groups:
            if not any(getattr(x, "kind", None) != "travel"
                       and not getattr(x, "no_print", False) for x in shapes):
                continue
            sub = sk.copy()
            sub.shapes = shapes
            out.append((iid, sub))
        return out

    def _group_segments(self, sub, pump: str):
        """Compile a sub-sketch → well-relative print sub-paths (recompiled, so
        welds / retrace / overlap continuity within the group are preserved)."""
        from SupportClasses.SketchTrajectory import compile_to_trajectory
        needle, syringe_map = self._needle_and_syringe()
        try:
            res = compile_to_trajectory(sub, needle, syringe_map.get(pump))
        except Exception:
            return []
        # v7.21.5: the sub-sketch is compiled RIGHT HERE, so its per-segment
        # extrusion is in hand — a multi-ink run gets the same fidelity as a
        # single-ink one instead of falling back to one flow per group.
        return self._subpaths_from_array(
            res.trajectory, res.extrusion_profile or None)

    def _pickup_uL_for_length(self, path_len_mm: float) -> float:
        speed, flow, prime = self._resolved_print_kinematics()
        if flow <= 0:
            return 0.0
        dispensed = flow * ((path_len_mm / speed) if speed > 0 else 0.0)
        return (dispensed + prime + self._needle_dead_volume_uL()
                + self._ink_padding_uL())

    def _ink_pickup_kwargs(self, ink_name) -> dict:
        """Resolve the tip-prime + granular-orbit kwargs for ``aspirate_ink``
        given the ink being picked up. Prime is a global µL setting; the orbit
        auto-applies to inks whose subtype is 'granular material' (or every ink
        when the force-all override is ticked). Guarded for the ``__new__``
        partial pages used in tests."""
        prime_uL = 0.0
        if getattr(self, "_ink_prime_check", None) is not None and \
                self._ink_prime_check.isChecked():
            prime_uL = float(self._ink_prime_spin.value())
        lib = (getattr(self._hw_config, "ink_library", {}) or {}
               if self._hw_config else {})
        spec = lib.get(ink_name)
        is_gran = ((getattr(spec, "ink_subtype", "") or "").strip().lower()
                   == "granular material")
        orbit = False
        if getattr(self, "_orbit_check", None) is not None:
            orbit = (self._orbit_all_check.isChecked()
                     or (self._orbit_check.isChecked() and is_gran))
        dia = (float(self._orbit_dia_spin.value())
               if getattr(self, "_orbit_dia_spin", None) is not None else 1.0)
        spd = (float(self._orbit_speed_spin.value())
               if getattr(self, "_orbit_speed_spin", None) is not None else 2.0)
        return {"prime_uL": prime_uL, "orbit": orbit,
                "orbit_diameter_mm": dia, "orbit_speed_mm_s": spd}

    def _hold_in_liquid_s(self) -> float:
        """v7.21.7: the global hold-in-liquid dwell (s) in force, for the
        confirm dialogs and the settings summary. Read from the SAME place the
        executor resolves it (the controller, which reads HardwareConfig), so
        the dialog cannot quote a value the run will not use."""
        ctrl = getattr(self, "_controller", None)
        getter = getattr(ctrl, "pump_post_aspirate_dwell_s", None)
        if not callable(getter):
            return 0.0
        try:
            return max(0.0, float(getter()))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _segments_length_mm(segments) -> float:
        total = 0.0
        for seg in segments:
            for i in range(1, len(seg)):
                dx = seg[i][0] - seg[i - 1][0]
                dy = seg[i][1] - seg[i - 1][1]
                total += (dx * dx + dy * dy) ** 0.5
        return total

    def _start_multi_ink_run(self):
        """Gate + resolve + launch a multi-ink sequential-swap run (one worker
        owns the whole sequence; per group: swap → pick up mapped ink → print).
        Non-sketch / single-ink runs never reach here (handled in _on_print)."""
        well = self._selected_well
        center = self._well_center_zero_ref_mm(well)
        if center is None:
            self._status.setText(f"Could not resolve position for well {well}.")
            return
        if self._safe_z is None:
            self._status.setText(
                "Multi-ink prints need a Safe Z — set it on the Calibration "
                "page.")
            return
        warns = self._validate_ink_map()
        if warns:
            self._status.setText("Fix ink mapping: " + "; ".join(warns))
            return
        # Service wells are required for the between-ink swaps (waste/wash/
        # buffer) and prep (oil).
        service_positions, missing = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        need = [r for r in ("waste", "oil", "wash", "buffer")
                if r not in service_positions]
        if need:
            self._status.setText(
                "Multi-ink swaps need these reagent wells assigned + "
                f"calibrated: {', '.join(need)}.")
            return
        needle_uL = needle_volume_uL(self._hw_config)
        if needle_uL <= 0:
            self._status.setText(
                "Multi-ink needs the needle inner Ø + length "
                "(Hardware Setup → Needle).")
            return
        service_z = self._plate_offset_to_zref(float(self._service_z_spin.value()))
        ink_dip_z = self._plate_offset_to_zref(float(self._ink_z_spin.value()))
        if service_z is None or ink_dip_z is None:
            self._status.setText(
                "Plate bottom Z not calibrated — can't resolve the service / "
                "ink dip Z.")
            return
        if not self._confirm_print_floor():
            return

        hw = self._hw_config
        wells = self._well_positions or {}
        locs = (getattr(hw, "ink_locations", {}) or {})
        base_settings = self._build_settings()
        travel_z = base_settings.travel_z_height

        groups = []
        for iid, sub in self._ink_groups():
            ink_name = self._ink_map.get(iid)
            pump = hw.get_pump_for_ink(ink_name)
            wl = locs.get(ink_name) or []
            wn = resolve_pickup_well(wl, self._plate)
            ink_pos = wells.get(wn) if wn else None
            segments = self._group_segments(sub, pump)
            group_mods = list(getattr(self, "_last_subpath_modifiers", []) or [])
            if not segments or ink_pos is None:
                continue
            length = self._segments_length_mm(segments)
            groups.append({
                "ink_id": iid, "ink_name": ink_name, "pump": pump,
                "ink_pos": ink_pos, "ink_dip_z": ink_dip_z,
                "segments": segments, "well": well, "center": center,
                "extrusion_profiles": self._vol_per_mm_profiles(group_mods),
                "settings": self._build_settings(pump=pump),
                "pickup_uL": self._pickup_uL_for_length(length),
                # Resolve prime/orbit kwargs on the GUI thread (reads widgets).
                "pickup_kwargs": self._ink_pickup_kwargs(ink_name),
            })
        if len(groups) < 2:
            self._status.setText(
                "Multi-ink run produced fewer than 2 printable ink groups.")
            return

        prep_ctx = {
            "needle_uL": needle_uL, "service_z": service_z,
            "service_positions": service_positions,
            "wash_cycles": int(self._wash_cycles_spin.value()),
            "buffer_needles": float(self._buffer_needles_spin.value()),
        }
        do_prep = self._prep_check.isChecked()
        cleanup = self._postclean_check.isChecked()

        lines = ["This run will print with sequential ink swaps:"]
        if do_prep:
            lines.append("  • Prep the needle (waste → oil → wash → buffer)")
        for g in groups:
            lines.append(f"  • Pick up ~{g['pickup_uL']:.3f} µL of "
                         f"“{g['ink_name']}” ({g['pump']}) → print")
        lines.append("    (waste → wash → buffer between inks)")
        if self._ink_prime_check.isChecked():
            lines.append(
                f"  • Prime the tip (+{self._ink_prime_spin.value():.2f} µL "
                "aspirated & dispensed back; no compliance comp)")
        if self._orbit_all_check.isChecked() or self._orbit_check.isChecked():
            lines.append(
                f"  • Circular pickup ({self._orbit_dia_spin.value():.2f} mm) "
                + ("for all inks" if self._orbit_all_check.isChecked()
                   else "for granular inks"))
        hold_s = self._hold_in_liquid_s()
        if hold_s > 0:
            lines.append(
                f"  • Hold the needle in the liquid {hold_s:.1f}s after every "
                "aspirate before lifting")
        if cleanup:
            lines.append("  • Clean the needle at the end")
        lines += ["", "Hardware set up correctly and ready to start?"]
        resp = QMessageBox.question(
            self, "Confirm multi-ink print", "\n".join(lines),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        if resp != QMessageBox.StandardButton.Yes:
            self._status.setText("Cancelled.")
            return

        self._print_btn.setEnabled(False)
        self._abort_btn.setEnabled(True)
        self._multi_abort_requested = False
        self._status.setText(
            f"Multi-ink print in {well}: {len(groups)} inks…")
        bridge = self._bridge
        safe_z = float(self._safe_z)

        def _worker():
            err = None
            executor = None
            try:
                executor = PickPlaceExecutor(self._controller, self._hw_config)
                executor.safe_z_mm = safe_z
                executor.needle_volume_uL = prep_ctx["needle_uL"]
                executor.service_z_mm = prep_ctx["service_z"]
                executor.wash_cycles = prep_ctx["wash_cycles"]
                executor.buffer_needles = prep_ctx["buffer_needles"]
                # Between-ink swap (run_post_clean) expels the previous ink to
                # waste before wash+buffer — clear several needles so no prior
                # ink carries into the next.
                executor.post_dispense_needles = 6.0
                sp = prep_ctx["service_positions"]
                executor.waste_well_pos = sp["waste"]
                executor.oil_well_pos = sp["oil"]
                executor.wash_well_pos = sp["wash"]
                executor.buffer_well_pos = sp["buffer"]
                executor.on_sub_step = (
                    lambda op, step: bridge.progress.emit(0, 0, str(step)))
                self._active_executor = executor
                if do_prep:
                    executor.prep_bore = groups[0]["pump"]
                    executor.run_prep()
                for gi, g in enumerate(groups):
                    if self._sequence_abort_requested():
                        raise AbortException()
                    if not getattr(self._controller, "is_zp_connected", False):
                        raise AbortException()
                    executor.prep_bore = g["pump"]
                    if gi > 0:
                        # Between-ink swap: waste → wash → buffer.
                        bridge.progress.emit(
                            0, 0, f"Ink swap → {g['ink_name']}…")
                        executor.run_post_clean()
                    bridge.progress.emit(
                        0, 0,
                        f"Picking up {g['pickup_uL']:.3f} µL of {g['ink_name']}…")
                    executor.aspirate_ink(
                        g["ink_pos"], g["pickup_uL"], bore=g["pump"],
                        z_mm=g["ink_dip_z"], **g.get("pickup_kwargs", {}))
                    self._run_group_job_blocking(g)
                if cleanup:
                    executor.run_print_cleanup()
            except AbortException:
                err = "aborted"
            except Exception as e:
                logger.exception("Quick Print multi-ink run failed: %s", e)
                err = str(e)
            finally:
                if err is None and executor is not None:
                    try:
                        if executor._abort_flag.is_set():
                            err = "aborted"
                    except Exception:
                        pass
                if executor is not None:
                    try:
                        executor._retract_to_safe_z()
                    except Exception:
                        pass
                self._active_executor = None
                self._pm = None
            bridge.multi_done.emit(err or "")

        self._multi_thread = threading.Thread(
            target=_worker, name="QuickPrintMultiInkRun", daemon=True)
        self._multi_thread.start()
        self._update_button_state()

    def _sequence_abort_requested(self) -> bool:
        """Sticky abort for EITHER long-running sequence engine.

        v7.19: ``_run_group_job_blocking`` is shared by the multi-ink run and the
        print queue, and it reads this to close the documented ``pm.start()``
        race. One predicate, so that guard cannot be correct for one engine and
        dead for the other.
        """
        return bool(self._multi_abort_requested or self._queue_abort_requested)

    def _run_group_job_blocking(self, g: dict):
        """Build + start one unit's discrete print job and BLOCK the worker thread
        until it reaches a terminal state (off the GUI thread).

        Returns the execution-log path (or None). Raises on abort / error so the
        worker's finally retracts to safe Z.
        """
        job = build_well_plate_job(
            well_positions=[(g["well"], g["center"][0], g["center"][1])],
            path_points=[p for seg in g["segments"] for p in seg],
            settings=g["settings"], pump=g["pump"], flow_rate=0.01,
            job_name=g.get("job_name")
            or f"Quick Print — {g['ink_name']} @ {g['well']}",
            path_segments=g["segments"],
            path_extrusion_profiles=g.get("extrusion_profiles"),
            return_home=False)
        done = threading.Event()
        result = {"state": None}
        pm = PrintManager(self._controller)
        bridge = self._bridge
        pm.on_progress = lambda c, t, m: bridge.progress.emit(int(c), int(t),
                                                              str(m))

        def _st(st):
            # Do NOT route through bridge.state (that drives the single-ink
            # terminal/cleanup logic) — just release the worker.
            if st in (PrintState.COMPLETED, PrintState.ABORTED,
                      PrintState.ERROR):
                result["state"] = st
                done.set()
        pm.on_state_changed = _st
        self._pm = pm
        pm.load_job(job)
        # Close the start-race: PrintManager.abort() is gated on RUNNING/PAUSED
        # and start() clears the abort flag, so an Abort requested in the window
        # between `self._pm = pm` and this group reaching RUNNING would be a
        # no-op for this group (it would run to completion, then the run halts
        # only at the next-group check). Honor a sticky abort request before we
        # start this group's print at all.
        if self._sequence_abort_requested():
            self._pm = None
            raise AbortException()
        pm.start()
        done.wait()
        # v7.19 BUGFIX: this used to discard the log path, so after a multi-ink run
        # `_last_log_path` still pointed at the PREVIOUS single print and "Open
        # log" opened the wrong file. The logger is created inside start().
        log_path = getattr(getattr(pm, "exec_logger", None), "path", None)
        self._pm = None
        if result["state"] != PrintState.COMPLETED:
            if result["state"] == PrintState.ABORTED:
                raise AbortException()
            raise RuntimeError(f"group print {result['state']}")
        return log_path

    def _on_multi_done(self, err: str) -> None:
        self._multi_thread = None
        self._multi_abort_requested = False
        self._pm = None
        self._set_print_live(False)      # v7.6: stop the fast live sampling
        if err == "aborted":
            self._status.setText("Multi-ink print aborted — needle at safe Z.")
        elif err:
            self._status.setText(f"Multi-ink print failed: {err}")
        else:
            self._status.setText("Multi-ink print complete — needle at safe Z.")
        self._update_button_state()

    # ══ v7.19: resolve + run the plate-wide queue ═══════════════════

    def _expand_queued_print(self, qp: QueuedPrint, qp_index: int) -> list[dict]:
        """Resolve ONE queued print into 1+ execution units.

        MUST be called inside ``self._snapshot_applied(qp)`` — everything below
        reads the widgets. Raises :class:`_QueueResolveError` with an
        operator-facing reason when the snapshot cannot run at all.

        A single-ink / non-sketch print contributes exactly one unit. A multi-ink
        sketch contributes one unit per ``_ink_groups()`` entry **in sketch
        order**, and that ordered run is ATOMIC — see ``_resolve_queue``.
        """
        well = qp.well
        center = self._well_center_zero_ref_mm(well)
        if center is None:
            raise _QueueResolveError("could not resolve the well position")
        # A batch dips into ink and service wells and prints at a computed height,
        # so it must not run against `_well_center_zero_ref_mm`'s GEOMETRIC
        # fallback for an uncalibrated well.
        if self._well_positions and well not in self._well_positions:
            raise _QueueResolveError("well not calibrated — run Plate Location")
        print_z = self._resolve_print_z()
        if print_z is None:
            raise _QueueResolveError("plate bottom not calibrated")

        units: list[dict] = []
        if self._is_multi_ink():
            groups = self._ink_groups()
            sk = self._loaded_sketch
            for sub_i, (iid, sub) in enumerate(groups):
                ink_name = self._ink_map.get(iid)
                if not ink_name:
                    raise _QueueResolveError(
                        f"abstract ink {iid} is not mapped to a configured ink")
                pump = (self._hw_config.get_pump_for_ink(ink_name)
                        if self._hw_config else None)
                if not pump:
                    raise _QueueResolveError(f"“{ink_name}” has no pump")
                segments = self._group_segments(sub, pump)
                if not segments:
                    continue
                ink_pos = self._ink_pos_for(ink_name)
                if ink_pos is None:
                    raise _QueueResolveError(
                        f"“{ink_name}” has no calibrated reagent well")
                label = getattr(sk.ink_by_id(iid), "name", f"ink {iid}") \
                    if sk else f"ink {iid}"
                units.append(self._make_unit(
                    qp, qp_index, sub_i, len(groups), well, center, segments,
                    pump, ink_name, ink_pos, print_z,
                    pickup_uL=self._pickup_uL_for_length(
                        self._segments_length_mm(segments)),
                    label_extra=label))
            if not units:
                raise _QueueResolveError("no printable path")
            for u in units:
                u["sub_total"] = len(units)
            return units

        segments = self._path_segments_for_selection()
        if not segments:
            raise _QueueResolveError("selected object produced no printable path")
        ink_name = self._selected_ink() or ""
        if not ink_name:
            # After the single up-front prep (and after every swap) the needle has
            # been washed out, so "print with whatever is loaded" is a fluidic
            # continuity claim this engine has already invalidated. Refuse it,
            # unless nothing is being serviced at all.
            if self._prep_check.isChecked() or self._postclean_check.isChecked():
                raise _QueueResolveError(
                    "no ink selected — Run all needs an explicit ink per print "
                    "when prep or cleanup is on")
            ink_pos = None
            pump = self._pump()
        else:
            pump = self._pump()
            ink_pos = self._ink_pos_for(ink_name)
            if ink_pos is None:
                raise _QueueResolveError(
                    f"“{ink_name}” has no calibrated reagent well")
        units.append(self._make_unit(
            qp, qp_index, 0, 1, well, center, segments, pump, ink_name, ink_pos,
            print_z, pickup_uL=self._compute_pickup_volume_uL()))
        return units

    def _ink_pos_for(self, ink_name: str):
        """The calibrated absolute-µm position of *ink_name*'s reagent well."""
        if not ink_name:
            return None
        locs = (getattr(self._hw_config, "ink_locations", {}) or {}) \
            if self._hw_config else {}
        wn = resolve_pickup_well(locs.get(ink_name) or [], self._plate)
        if not wn:
            return None
        return (self._well_positions or {}).get(wn)

    def _make_unit(self, qp, qp_index, sub_i, sub_total, well, center, segments,
                   pump, ink_name, ink_pos, print_z, *, pickup_uL,
                   label_extra: str = "") -> dict:
        """One execution unit — a SUPERSET of the multi-ink group dict, so
        ``_run_group_job_blocking`` consumes it unchanged."""
        ideal = [(center[0] + px, center[1] + py)
                 for seg in segments for (px, py) in seg]
        label = qp.object_label or "print"
        if label_extra:
            label = f"{label} · {label_extra}"
        return {
            "well": well, "center": center, "segments": segments,
            "settings": self._build_settings(pump=pump), "pump": pump,
            "ink_name": ink_name,
            "ink_pos": ink_pos,
            "ink_dip_z": self._plate_offset_to_zref(qp.ink_dip_z_mm),
            "pickup_uL": float(pickup_uL or 0.0),
            "pickup_kwargs": self._ink_pickup_kwargs(ink_name),
            "qp_index": qp_index, "sub_index": sub_i, "sub_total": sub_total,
            "obj_label": qp.object_label,
            "job_name": f"Quick Print — {label} @ {well}",
            "print_z_zref": print_z,
            "ideal_pts": ideal if len(ideal) >= 2 else None,
        }

    def _resolve_queue(self, queue) -> tuple[list, list, list]:
        """Resolve every enabled queued print into execution units.

        GUI-THREAD ONLY. Returns ``(units, skipped, warnings)`` where *skipped* is
        ``[(well, reason)]`` — an entry that cannot run is never silently dropped.
        Shows NO dialogs (a modal here would re-enter the event loop while a
        transient snapshot is applied), and restores the on-screen config even
        when a middle entry raises.
        """
        units: list[dict] = []
        skipped: list[tuple[str, str]] = []
        warnings: list[str] = []
        floor_bad: list[tuple[str, float]] = []
        try:
            for i, qp in enumerate(queue):
                if not qp.enabled:
                    continue
                problem = self._queue_entry_problem(qp)
                if problem:
                    skipped.append((qp.well, problem))
                    continue
                try:
                    with self._snapshot_applied(qp) as clamp_warnings:
                        warnings.extend(clamp_warnings or [])
                        got = self._expand_queued_print(qp, i)
                        # Per-UNIT floor check, evaluated NUMERICALLY: each
                        # snapshot has its own print height, and a modal inside
                        # the transient-apply block would let the status tick
                        # repaint from the wrong snapshot.
                        for u in got:
                            z = u.get("print_z_zref")
                            try:
                                if z is not None and \
                                        self._controller.print_floor_violation(z):
                                    floor_bad.append((u["well"], qp.print_z_mm))
                            except Exception:
                                pass
                        units.extend(got)
                except _QueueResolveError as exc:
                    skipped.append((qp.well, str(exc)))
                except Exception as exc:
                    logger.exception("resolving %s failed", qp.well)
                    skipped.append((qp.well, f"could not resolve ({exc})"))
        finally:
            # Restore the derived UI exactly once — with signals blocked, none of
            # the usual refreshes ran.
            self._on_object_changed()
            self._on_settings_changed()
        if floor_bad:
            seen = []
            for well, z in floor_bad:
                if well not in [w for w, _ in seen]:
                    seen.append((well, z))
            warnings.append(
                "below the calibrated plate bottom (will be clamped, so they "
                "will not print deeper): "
                + ", ".join(f"{w} at {z:g} mm" for w, z in seen))
        return units, skipped, warnings

    def _queue_pump_moves(self, units, *, prep_enabled: bool) -> dict:
        """Ordered signed plunger moves per pump across the WHOLE resolved queue.

        ``+`` = dispense, ``−`` = aspirate (the ``_check_syringe_budget``
        convention). Models prep once on the first unit's bore, then per unit
        ``−pickup`` / ``+printed``. Deliberately does NOT model the wash / oil
        volumes inside ``run_post_clean`` / ``run_print_cleanup`` — that
        under-claim is DISCLOSED in the confirm dialog rather than hidden.
        """
        moves: dict[str, list[float]] = {}
        needle_uL = needle_volume_uL(self._hw_config) or 0.0
        if prep_enabled and units:
            bore = units[0]["pump"]
            buf = 1.0
            try:
                buf = float(self._buffer_needles_spin.value())
            except Exception:
                pass
            moves.setdefault(bore, []).extend(
                [needle_uL, -needle_uL, -buf * needle_uL])
        for u in units:
            lst = moves.setdefault(u["pump"], [])
            if u.get("pickup_uL"):
                lst.append(-float(u["pickup_uL"]))
            try:
                dispensed = self._print_dispense_volume_uL(u["segments"])
            except Exception:
                dispensed = 0.0
            if dispensed:
                lst.append(float(dispensed))
        return moves

    def _check_queue_syringe_budget(self, units, *, prep_enabled: bool) -> dict:
        """Per pump: ``{"verdict": fits|no_fit|unchecked, …}``.

        The single-print pre-flight models ONE print, and the multi-ink path skips
        it with no mention anywhere in the UI. A queue is MORE exposed, not less:
        same-ink adjacency skips the wash, so the plunger drifts monotonically
        across the batch. So this is computed and one of three verdicts is always
        stated.
        """
        out: dict[str, dict] = {}
        for pump, moves in self._queue_pump_moves(
                units, prep_enabled=prep_enabled).items():
            if not moves:
                continue
            try:
                res = self._controller.simulate_pump_budget(pump, moves)
            except Exception as exc:
                out[pump] = {"verdict": "unchecked", "reason": str(exc)}
                continue
            if not isinstance(res, dict):
                out[pump] = {"verdict": "unchecked", "reason": "no result"}
            elif res.get("ok"):
                out[pump] = {"verdict": "fits", **res}
            elif res.get("reason") in ("uncalibrated", "fill_unreadable"):
                out[pump] = {"verdict": "unchecked", **res}
            else:
                out[pump] = {"verdict": "no_fit", **res}
        return out

    def _on_run_all(self) -> None:
        """Gate → resolve every queued print on the GUI thread → ONE confirm →
        ONE worker owning prep-once / per-unit pickup+print / swap-on-ink-change /
        cleanup-once. Mirrors ``_start_multi_ink_run``'s contract."""
        if self._is_running():
            return
        for t in (self._queue_thread, self._multi_thread,
                  self._preposition_thread, self._cleanup_thread):
            if t is not None and t.is_alive():
                return
        if not getattr(self._controller, "is_xy_connected", False) or \
                not getattr(self._controller, "is_zp_connected", False):
            self._status.setText("Connect the XY and ZP stages first.")
            return
        if self._plate is None:
            self._status.setText("No plate available — run Calibration first.")
            return
        if not [qp for qp in self._queue if qp.enabled]:
            self._status.setText("The queue is empty.")
            return
        # A batch ALWAYS services and dips, so a Safe Z is a hard requirement —
        # not the "continue anyway?" prompt a plain single print offers.
        if self._safe_z is None:
            self._status.setText(
                "Run all needs a Safe Z — set it on the Calibration page.")
            return

        do_prep = self._prep_check.isChecked()
        cleanup = self._postclean_check.isChecked()
        needle_uL = needle_volume_uL(self._hw_config)
        service_positions, missing = resolve_service_positions(
            self._hw_config, self._well_positions, self._plate)
        if do_prep or cleanup:
            if needle_uL <= 0:
                self._status.setText(
                    "Run all needs the needle inner Ø + length (Hardware Setup "
                    "→ Needle), or turn prep and cleanup off.")
                return
            if missing:
                self._status.setText(
                    "Run all needs these reagent wells assigned + calibrated: "
                    f"{', '.join(missing)} — or turn prep and cleanup off.")
                return
        service_z = self._plate_offset_to_zref(float(self._service_z_spin.value()))
        if (do_prep or cleanup) and service_z is None:
            self._status.setText(
                "Plate bottom Z not calibrated — can't resolve the service "
                "dip Z.")
            return

        ordered = self._ordered_queue()
        units, skipped, warnings = self._resolve_queue(ordered)
        if not units:
            lines = ["Nothing in the queue can run."]
            lines += [f"  • {w} — {why}" for w, why in skipped]
            QMessageBox.warning(self, "Nothing to run", "\n".join(lines),
                                QMessageBox.StandardButton.Ok)
            self._status.setText("Nothing in the queue can run — see dialog.")
            return

        force_clean = self._queue_clean_between()
        swaps = count_ink_swaps(units, force_clean_between=force_clean)
        alt_mode = ORDER_PLATE if self._queue_group_by_ink() else ORDER_INK
        try:
            alt_units, _s, _w = None, None, None
            alt_swaps = count_ink_swaps(
                [{"ink_name": u["ink_name"]} for u in units]
                if alt_mode == ORDER_PLATE else
                # Cheap counterfactual: re-order the RESOLVED units by ink while
                # keeping each print's own units together.
                sorted(units, key=lambda u: (u["ink_name"], u["qp_index"],
                                             u["sub_index"])),
                force_clean_between=force_clean)
        except Exception:
            alt_swaps = None

        budget = self._check_queue_syringe_budget(units, prep_enabled=do_prep)
        blocked = [p for p, v in budget.items() if v["verdict"] == "no_fit"]

        lines = [f"Run all {len({u['qp_index'] for u in units})} queued print"
                 f"{'' if len(units) == 1 else 's'} "
                 f"({len(units)} print unit"
                 f"{'' if len(units) == 1 else 's'}, "
                 f"{len({u['well'] for u in units})} wells)."]
        lines.append("  • Prep the needle ONCE now (waste → oil → wash → buffer)"
                     if do_prep else
                     "  • No prep — the needle is used as-is.")
        lines.append("")
        lines.append("Order: " + ("grouped by ink" if self._queue_group_by_ink()
                                  else "plate order"))
        for n, u in enumerate(units, 1):
            atom = (f" [{u['sub_index'] + 1}/{u['sub_total']} of one sketch]"
                    if u["sub_total"] > 1 else "")
            lines.append(
                f"  {n}. {u['well']} — {u['obj_label'] or 'print'} — "
                f"{u['ink_name'] or '(loaded)'} ({u['pump']}) — "
                f"pick up {u['pickup_uL']:.3f} µL{atom}")
        swap_line = (f"  • {swaps} ink swap{'' if swaps == 1 else 's'} "
                     "(waste → wash → buffer each)")
        if alt_swaps is not None and alt_swaps != swaps:
            swap_line += (f" — {'plate order' if self._queue_group_by_ink() else 'Group by ink'}"
                          f" would need {alt_swaps}")
        lines.append("")
        lines.append(swap_line)
        lines.append("  • Clean the needle ONCE at the end" if cleanup else
                     "  • No cleanup — the needle will still hold ink/buffer "
                     "when this finishes.")
        for pump, v in budget.items():
            if v["verdict"] == "fits":
                lines.append(
                    f"  • Syringe budget {pump}: OK — spans "
                    f"{v.get('span_uL', 0.0):.2f} µL of "
                    f"{v.get('capacity_uL', 0.0):.2f} µL (print + pickup moves "
                    "only; the wash/prep volumes are not modelled)")
            elif v["verdict"] == "no_fit":
                lines.append(
                    f"  • Syringe budget {pump}: WON'T FIT — needs "
                    f"{v.get('span_uL', 0.0):.2f} µL but holds "
                    f"{v.get('capacity_uL', 0.0):.2f} µL")
            else:
                lines.append(
                    f"  • Syringe budget {pump}: NOT CHECKED "
                    f"({v.get('reason', 'unavailable')})")
        if skipped:
            lines.append("")
            lines.append("⚠ NOT running:")
            lines += [f"    {w} — {why}" for w, why in skipped]
        if warnings:
            lines.append("")
            lines += [f"⚠ {w}" for w in warnings]
        lines.append("")
        lines.append("All prints use the current between-lines, travel and "
                     "pre-flow settings.")
        lines.append(
            "Once started this runs unattended. Abort stops after the current "
            "step and retracts to safe Z; the needle will still hold ink.")

        if blocked:
            QMessageBox.warning(
                self, "Won't fit the syringe",
                "\n".join(lines)
                + "\n\nReduce the queue, the print sizes or the ink padding, or "
                  "tick “Clean between every print” so the syringe is reset each "
                  "time.",
                QMessageBox.StandardButton.Ok)
            self._status.setText(
                f"Run all blocked — {', '.join(blocked)} won't fit the syringe.")
            return
        lines.append("")
        lines.append("Hardware set up correctly and ready to start?")
        if QMessageBox.question(
                self, "Confirm run all", "\n".join(lines),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
        ) != QMessageBox.StandardButton.Yes:
            self._status.setText("Cancelled.")
            return

        self._queue_abort_requested = False
        self._hold_requested = False
        self._queue_is_held = False
        self._hold_event.clear()
        self._queue_results = []
        self._queue_ran = 0
        self._queue_total = len(units)
        self._queue_swaps = swaps
        self._print_btn.setEnabled(False)
        self._run_all_btn.setEnabled(False)
        self._abort_btn.setEnabled(True)
        self._hold_btn.setEnabled(True)
        self._status.setText(f"Run all: {len(units)} prints…")

        bridge = self._bridge
        safe_z = float(self._safe_z)
        wash_cycles = int(self._wash_cycles_spin.value())
        buffer_needles = float(self._buffer_needles_spin.value())

        def _worker():
            err = None
            executor = None
            prev_ink = None
            ran = 0
            try:
                executor = PickPlaceExecutor(self._controller, self._hw_config)
                executor.safe_z_mm = safe_z
                executor.needle_volume_uL = needle_uL
                executor.service_z_mm = service_z
                executor.wash_cycles = wash_cycles
                executor.buffer_needles = buffer_needles
                # Clear several needles on a swap so no prior ink carries over.
                executor.post_dispense_needles = 6.0
                for role in ("waste", "oil", "wash", "buffer"):
                    if role in service_positions:
                        setattr(executor, f"{role}_well_pos",
                                service_positions[role])
                executor.on_sub_step = (
                    lambda op, step: bridge.progress.emit(0, 0, str(step)))
                self._active_executor = executor
                if do_prep:
                    executor.prep_bore = units[0]["pump"]
                    executor.run_prep()
                for u in units:
                    if self._sequence_abort_requested():
                        raise AbortException()
                    if not getattr(self._controller, "is_zp_connected", False):
                        raise AbortException()
                    # ── hold at a WELL BOUNDARY ────────────────────
                    if self._hold_requested:
                        self._hold_requested = False
                        try:
                            executor._retract_to_safe_z()
                        except Exception:
                            pass
                        self._hold_event.clear()
                        bridge.queue_held.emit(True)
                        self._hold_event.wait()
                        bridge.queue_held.emit(False)
                        if self._sequence_abort_requested():
                            raise AbortException()
                    executor.prep_bore = u["pump"]
                    need_swap = (prev_ink is not None
                                 and (force_clean
                                      or u["ink_name"] != prev_ink))
                    if need_swap:
                        bridge.progress.emit(
                            0, 0, f"Ink swap → {u['ink_name']}…")
                        executor.run_post_clean()
                    if u["ink_pos"] is not None and u["pickup_uL"] > 0:
                        bridge.progress.emit(
                            0, 0, f"Picking up {u['pickup_uL']:.3f} µL of "
                                  f"{u['ink_name']}…")
                        executor.aspirate_ink(
                            u["ink_pos"], u["pickup_uL"], bore=u["pump"],
                            z_mm=u["ink_dip_z"], **u.get("pickup_kwargs", {}))
                    bridge.queue_unit.emit(ran + 1, len(units), u["job_name"])
                    log_path = self._run_group_job_blocking(u)
                    self._queue_results.append({
                        "qp_index": u["qp_index"], "well": u["well"],
                        "obj_label": u["obj_label"], "ink_name": u["ink_name"],
                        "log_path": log_path, "ideal_pts": u["ideal_pts"],
                    })
                    prev_ink = u["ink_name"]
                    ran += 1
                    self._queue_ran = ran
                if cleanup:
                    executor.run_print_cleanup()
            except AbortException:
                err = "aborted"
            except Exception as e:
                logger.exception("Quick Print run-all failed: %s", e)
                err = str(e)
            finally:
                if err is None and executor is not None:
                    try:
                        if executor._abort_flag.is_set():
                            err = "aborted"
                    except Exception:
                        pass
                if executor is not None:
                    try:
                        executor._retract_to_safe_z()
                    except Exception:
                        pass
                self._active_executor = None
                self._pm = None
            self._queue_ran = ran
            bridge.queue_done.emit(err or "")

        self._queue_thread = threading.Thread(
            target=_worker, name="QuickPrintRunAll", daemon=True)
        self._queue_thread.start()
        self._update_button_state()

    def _on_queue_unit(self, index: int, total: int, label: str) -> None:
        """GUI thread: the queue advanced. `_set_print_live` drives a QTimer, so it
        may only be toggled here — never from the worker."""
        self._status.setText(f"Print {index}/{total}: {label}")
        self._set_print_live(True)

    def _on_queue_held(self, held: bool) -> None:
        self._queue_is_held = bool(held)
        if held:
            self._set_print_live(False)
            left = max(0, self._queue_total - self._queue_ran)
            self._status.setText(
                f"Held after {self._queue_ran} of {self._queue_total} prints — "
                "needle at safe Z. The stage is free, so you can run another "
                f"workflow now. Press “Resume ({left} left)” to continue.")
        else:
            self._status.setText("Resuming…")
        self._update_button_state()

    def _on_hold_clicked(self) -> None:
        if self._queue_thread is None or not self._queue_thread.is_alive():
            return
        if self._queue_is_held:
            # ⚠ A PROXY, not a lease: this catches another workflow's scan or
            # print (both suspend the poller for their duration), but not a manual
            # jog, and it cannot name which workflow is driving.
            try:
                busy = bool(self._controller.is_position_poller_suspended())
            except Exception:
                busy = False
            if busy:
                self._status.setText(
                    "Something else is driving the stage right now (a scan or a "
                    "print). Wait for it to finish, then Resume.")
                return
            if not getattr(self._controller, "is_xy_connected", False) or \
                    not getattr(self._controller, "is_zp_connected", False):
                self._status.setText(
                    "Reconnect the XY and ZP stages before resuming.")
                return
            self._hold_event.set()
        else:
            self._hold_requested = True
            self._status.setText(
                "Will hold after the print currently running.")
            self._update_button_state()

    def _on_queue_done(self, err: str) -> None:
        self._queue_thread = None
        self._queue_abort_requested = False
        self._hold_requested = False
        self._queue_is_held = False
        self._hold_event.clear()
        self._pm = None
        self._set_print_live(False)
        ran, total = self._queue_ran, self._queue_total
        swaps = self._queue_swaps
        if err == "aborted":
            self._status.setText(
                f"Run all aborted after {ran} of {total} prints — needle at safe "
                "Z. ⚠ Cleanup did NOT run, so the needle still holds "
                "ink/buffer, and the dispensed volume of the interrupted print "
                "is indeterminate.")
        elif err:
            self._status.setText(
                f"Run all failed after {ran} of {total} prints: {err}")
        else:
            self._status.setText(
                f"Ran {ran} of {total} prints ({swaps} ink "
                f"swap{'' if swaps == 1 else 's'}) — needle at safe Z.")
        last = next((r for r in reversed(self._queue_results)
                     if r.get("log_path")), None)
        if last is not None:
            self._last_log_path = last["log_path"]
            self._refresh_log_button()
            self._load_report(
                last["log_path"], ideal_pts=last.get("ideal_pts"),
                context={"object": last.get("obj_label") or "",
                         "well": last.get("well") or "",
                         "queue": f"print {ran} of {total}"})
        self._update_button_state()

    def _on_print(self):
        if self._is_running():
            return
        if (self._multi_thread is not None and self._multi_thread.is_alive()):
            return  # a multi-ink run is already in flight
        if (self._preposition_thread is not None
                and self._preposition_thread.is_alive()):
            return  # a positioning / preflight move is already in flight
        # v7.19: a queued run owns the machine for the whole batch. (And the
        # _cleanup_thread guard closes a pre-existing hole: a post-print cleanup
        # could be running while Print was re-clicked, with only the 300 ms status
        # tick standing between them.)
        if self._queue_thread is not None and self._queue_thread.is_alive():
            return
        if self._cleanup_thread is not None and self._cleanup_thread.is_alive():
            return
        if not getattr(self._controller, "is_xy_connected", False) or \
                not getattr(self._controller, "is_zp_connected", False):
            self._status.setText("Connect the XY and ZP stages first.")
            return
        well = self._selected_well
        if not well:
            self._status.setText("Click a well first.")
            return
        if self._plate is None:
            self._status.setText("No plate available — run Calibration first.")
            return
        if not self._object_combo.currentData():
            self._status.setText("Choose an object.")
            return

        # v7.5.x: an abstract-ink sketch using ≥2 inks runs the sequential
        # ink-swap path; everything else takes the legacy single-ink flow below.
        if self._is_multi_ink():
            self._start_multi_ink_run()
            return

        center = self._well_center_zero_ref_mm(well)
        if center is None:
            self._status.setText(f"Could not resolve position for well {well}.")
            return

        try:
            # v7.21.5: the modifiers come back beside the segments, so the print
            # deposits what the sketch computed per shape instead of one flow.
            path_segments, path_mods =                 self._segments_and_modifiers_for_selection()
        except Exception as e:
            logger.exception("Quick Print geometry failed: %s", e)
            self._status.setText(f"Geometry error: {e}")
            return
        path_points = [p for seg in path_segments for p in seg]
        if not path_points:
            self._status.setText("Selected object produced no printable path.")
            return

        # v7.5.x: early-warn if the configured print height would punch through
        # the plate bottom (the controller still hard-clamps during motion).
        if not self._confirm_print_floor():
            return

        # ── Resolve the pick-and-place preamble (needle prep + ink pickup) ──
        # Only active when prep is on OR an ink is selected for pickup; a plain
        # print (no prep, ink "(none)") behaves exactly as before.
        prep_enabled = self._prep_check.isChecked()
        ink = self._selected_ink()
        preamble = prep_enabled or (ink is not None)

        ink_pos: tuple[float, float] | None = None
        ink_dip_z: float | None = None
        pickup_uL = 0.0
        service_positions: dict[str, tuple[float, float]] = {}
        service_z: float | None = None
        needle_uL = 0.0
        if preamble:
            # Any well descent in the preamble needs a Safe Z + plate bottom.
            if self._safe_z is None:
                self._status.setText(
                    "Needle prep / ink pickup needs a Safe Z — set it on the "
                    "Calibration page (or turn Prep off and pick “(none)”).")
                return
            if ink is not None:
                ink_pos = self._ink_source_pos()
                if ink_pos is None:
                    self._status.setText(
                        f"Ink “{ink}” has no calibrated reagent well — assign "
                        "it (Hardware Setup → Ink) and run Plate Location, or "
                        "pick “(none)”.")
                    return
                ink_dip_z = self._plate_offset_to_zref(
                    float(self._ink_z_spin.value()))
                if ink_dip_z is None:
                    self._status.setText(
                        "Plate bottom Z not calibrated — can't resolve the ink "
                        "dip Z.")
                    return
                pickup_uL = self._compute_pickup_volume_uL()
            if prep_enabled:
                needle_uL = needle_volume_uL(self._hw_config)
                if needle_uL <= 0:
                    self._status.setText(
                        "Prep needs the needle inner Ø + length "
                        "(Hardware Setup → Needle), or turn Prep off.")
                    return
                service_positions, missing = resolve_service_positions(
                    self._hw_config, self._well_positions, self._plate)
                if missing:
                    self._status.setText(
                        "Prep needs these reagent wells assigned + calibrated: "
                        f"{', '.join(missing)} — or turn Prep off.")
                    return
                service_z = self._plate_offset_to_zref(
                    float(self._service_z_spin.value()))
                if service_z is None:
                    self._status.setText(
                        "Plate bottom Z not calibrated — can't resolve the "
                        "prep service dip Z.")
                    return
                # Capture wash cycles + buffer count on the GUI thread (the spins
                # live in the modeless popout the operator can touch while the
                # preamble runs).
                wash_cycles = int(self._wash_cycles_spin.value())
                buffer_needles = float(self._buffer_needles_spin.value())
        elif self._safe_z is None:
            # Plain print, no preamble: keep the legacy "no Safe Z" prompt.
            resp = QMessageBox.question(
                self, "No Safe Z",
                "No safe Z is configured. The needle will not retract to a safe "
                "height before travel. Continue anyway?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if resp != QMessageBox.StandardButton.Yes:
                return

        # ── Resolve the post-print cleanup (waste → wash → reset oil) ──
        cleanup_enabled = self._postclean_check.isChecked()
        cleanup_ctx = None
        if cleanup_enabled:
            if self._safe_z is None:
                self._status.setText(
                    "Post-print clean needs a Safe Z — set it on the "
                    "Calibration page (or untick “Reset syringe to initial "
                    "condition after print”).")
                return
            cl_needle = needle_volume_uL(self._hw_config)
            if cl_needle <= 0:
                self._status.setText(
                    "Post-print clean needs the needle inner Ø + length "
                    "(Hardware Setup → Needle), or turn it off.")
                return
            cl_positions, _cl_missing = resolve_service_positions(
                self._hw_config, self._well_positions, self._plate)
            cl_need = [r for r in ("waste", "wash", "oil")
                       if r not in cl_positions]
            if cl_need:
                self._status.setText(
                    "Post-print clean needs these reagent wells assigned + "
                    f"calibrated: {', '.join(cl_need)} — or turn it off.")
                return
            cl_service_z = self._plate_offset_to_zref(
                float(self._service_z_spin.value()))
            if cl_service_z is None:
                self._status.setText(
                    "Plate bottom Z not calibrated — can't resolve the cleanup "
                    "service dip Z.")
                return
            # Pre-run plunger position → the oil step resets back to it.
            try:
                oil_baseline = self._controller.get_pump_position_uL(self._pump())
            except Exception:
                oil_baseline = None
            # v7.5.x: "reset to initial condition" — waste the unprinted
            # ink + buffer (computed live) + a small oil flush margin, then top
            # the oil back up to the pre-run plunger position.
            margin_needles = float(self._postclean_margin_spin.value())
            cleanup_ctx = {
                "bore": self._pump(),
                "needle_uL": cl_needle,
                "service_positions": cl_positions,
                "service_z": cl_service_z,
                "reset_to_initial": True,
                "oil_margin_uL": margin_needles * cl_needle,
                # Fixed fallbacks used only if the live plunger position can't
                # be read at cleanup time (a bad M114).
                "waste_needles": 6.0,
                "oil_needles": max(margin_needles, 1.0),
                "oil_baseline_uL": oil_baseline,
                "wash_cycles": int(self._wash_cycles_spin.value()),
            }

        obj_label = self._object_combo.currentText()

        # v7.5.x: build settings up-front so the pre-position move uses the same
        # travel / safe Z the print job will use.
        settings = self._build_settings()
        pump = self._pump()
        travel_z = settings.travel_z_height

        # The print starts at the first path point of the first object (well
        # center + first point, zero-ref mm) — exactly where the needle will
        # descend. Travel there with the needle RETRACTED, then ask the operator
        # to confirm the position on the live microscope BEFORE any extrusion.
        start_zref_mm = (center[0] + path_points[0][0],
                         center[1] + path_points[0][1])

        # ── v7.5.x Feature 1: syringe-budget pre-flight (Quick Print only) ──
        # Simulate EVERY pump move this run makes — prep dispense/aspirate → ink
        # pickup → print prime + path dispense → the post-print reset trough —
        # against the calibrated plunger envelope [empty, full]. If it can't fit
        # at ANY starting fill, tell the operator it won't work and block. If it
        # CAN fit with a different start, offer to waste (over-fill) or aspirate
        # (runs dry) that much oil first, then proceed. Skipped (proceeds) when
        # the pump isn't plunger-calibrated or the live fill is unreadable.
        starting_oil = None
        budget = self._check_syringe_budget(
            pump, prep_enabled, ink is not None, pickup_uL, settings,
            cleanup_enabled, cleanup_ctx)
        if budget is not None:
            if not budget.get("feasible_by_shift"):
                cap = budget.get("capacity_uL") or 0.0
                QMessageBox.warning(
                    self, "Won't fit the syringe",
                    "This print won't work: its pump moves span "
                    f"{budget.get('span_uL', 0.0):.2f} µL but the {pump} "
                    f"syringe only holds {cap:.2f} µL. Reduce the print size, "
                    "the ink pickup, or the prep buffer/oil volumes.",
                    QMessageBox.StandardButton.Ok)
                self._status.setText("Print won't fit the syringe — see dialog.")
                return
            # Feasible with a different starting fill → offer the oil remedy.
            starting_oil = self._offer_oil_remedy(budget)
            if starting_oil is None:
                return  # operator cancelled, or the remedy well isn't set up

        # Resolve tip-prime + granular-orbit kwargs on the GUI thread (reads
        # widgets); reused in the confirm dialog note and the off-thread worker.
        pickup_kwargs = self._ink_pickup_kwargs(ink)

        # ── "Confirm all is setup" dialog — when there's a preamble or a
        # post-print cleanup (both are significant automated routines). ──
        if preamble or cleanup_enabled:
            lines = ["This run will:"]
            if prep_enabled:
                lines.append("  • Prep the needle: waste → oil → wash → buffer")
            if ink is not None:
                lines.append(
                    f"  • Pick up ~{pickup_uL:.3f} µL of “{ink}” from "
                    f"{self._ink_source_well()}")
                if pickup_kwargs["prime_uL"] > 0:
                    lines.append(
                        f"      + prime the tip (+{pickup_kwargs['prime_uL']:.2f} "
                        "µL aspirated & dispensed back; no compliance comp)")
                if pickup_kwargs["orbit"]:
                    lines.append(
                        f"      + circular pickup "
                        f"({pickup_kwargs['orbit_diameter_mm']:.2f} mm)")
            hold_s = self._hold_in_liquid_s()
            if hold_s > 0:
                lines.append(
                    f"      + hold in the liquid {hold_s:.1f}s before lifting "
                    "(so the aspirate does not finish in air)")
            lines.append(f"  • Print “{obj_label}” in well {well}")
            if cleanup_enabled:
                lines.append(
                    "  • Reset the syringe to its initial condition: waste the "
                    "unprinted ink+buffer → wash → top up oil")
            lines += ["", "Hardware set up correctly and ready to start?"]
            resp = QMessageBox.question(
                self, "Confirm print setup", "\n".join(lines),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if resp != QMessageBox.StandardButton.Yes:
                self._status.setText("Cancelled.")
                return

        # Stash the context the post-positioning continuation needs.
        self._pending_print = {
            "well": well, "center": center, "path_points": path_points,
            "path_segments": path_segments, "settings": settings,
            "pump": pump, "obj_label": obj_label, "cleanup": cleanup_ctx,
            # v7.21.5: resolved HERE, while the selection is known — the
            # continuation runs after a worker thread and must not re-derive
            # geometry (the operator may have touched a control meanwhile).
            "extrusion_profiles": self._vol_per_mm_profiles(path_mods),
        }

        self._print_btn.setEnabled(False)
        # Abort can interrupt the preamble (prep / ink) or the starting-oil
        # remedy — any automated pre-print routine on the worker thread.
        self._abort_btn.setEnabled(preamble or (starting_oil is not None))
        self._preflight_error = None
        self._preflight_abort_requested = False
        self._status.setText(
            (f"Preparing needle / picking up ink for {well}…"
             if (preamble or starting_oil is not None)
             else f"Positioning needle over well {well} (retracted to safe Z)…"))

        bridge = self._bridge

        # Worker: (optional) prep → pick up ink → preposition over the print
        # start (needle retracted). Runs off the GUI thread so the live feed
        # keeps painting; on every exit the executor's raise-only retract leaves
        # the needle at the safe Z. The outcome (None / "aborted" / "<error>")
        # gates the continuation in _on_prepositioned.
        def _worker():
            err = None
            ok = False
            executor = None
            # The executor runs the optional prep + ink pickup AND, when the
            # syringe-budget pre-flight asked for it, the starting-oil remedy.
            run_executor = preamble or (starting_oil is not None)
            try:
                if run_executor:
                    executor = PickPlaceExecutor(
                        self._controller, self._hw_config)
                    executor.safe_z_mm = float(self._safe_z)
                    executor.prep_bore = pump
                    executor.on_sub_step = (
                        lambda op, step: bridge.progress.emit(0, 0, str(step)))
                    self._active_executor = executor
                    # v7.5.x Feature 1: bring the syringe to a feasible starting
                    # fill BEFORE prep — waste (over-fill) or aspirate (runs dry)
                    # the computed oil at the resolved service well.
                    if starting_oil is not None:
                        executor.service_z_mm = starting_oil["service_z"]
                        setattr(executor,
                                f"{starting_oil['role']}_well_pos",
                                starting_oil["well_pos"])
                        executor.prepare_starting_oil(
                            starting_oil["volume_uL"],
                            dispense_to_waste=starting_oil["dispense_to_waste"])
                    if prep_enabled:
                        executor.needle_volume_uL = needle_uL
                        executor.service_z_mm = service_z
                        executor.wash_cycles = wash_cycles
                        executor.buffer_needles = buffer_needles
                        executor.waste_well_pos = service_positions["waste"]
                        executor.oil_well_pos = service_positions["oil"]
                        executor.wash_well_pos = service_positions["wash"]
                        executor.buffer_well_pos = service_positions["buffer"]
                        executor.run_prep()
                    if ink is not None and ink_pos is not None:
                        bridge.progress.emit(
                            0, 0, f"Picking up {pickup_uL:.3f} µL of {ink}…")
                        executor.aspirate_ink(
                            ink_pos, pickup_uL, bore=pump, z_mm=ink_dip_z,
                            **pickup_kwargs)
                ok = self._preposition_for_print(start_zref_mm, travel_z)
            except AbortException:
                err = "aborted"
            except Exception as e:
                logger.exception("Quick Print preflight failed: %s", e)
                err = str(e)
            finally:
                # Honor an Abort that was requested during a blocking move that
                # the executor's _check_abort() points didn't poll (pump moves /
                # safe_travel_to): treat a set abort flag as "aborted" so the
                # print does NOT start. (Belt-and-suspenders with the sticky
                # _preflight_abort_requested consumed on the GUI thread.)
                if err is None and executor is not None:
                    try:
                        if executor._abort_flag.is_set():
                            err = "aborted"
                    except Exception:
                        pass
                if executor is not None:
                    try:
                        executor._retract_to_safe_z()
                    except Exception:
                        pass
                self._active_executor = None
            self._preflight_error = err
            bridge.prepositioned.emit(bool(ok))

        self._preposition_thread = threading.Thread(
            target=_worker, name="QuickPrintPreflight", daemon=True)
        self._preposition_thread.start()

    def _on_prepositioned(self, positioned: bool) -> None:
        """GUI-thread continuation after the preflight worker finishes.

        v7.5.x: the run is HANDS-FREE after the up-front "confirm setup" gate —
        there is no per-print position confirmation. Once the needle is prepped,
        loaded with ink, and positioned (retracted) over the print start, this
        builds and starts the print directly. It still refuses to print if the
        preamble aborted/errored, if the ZP board dropped during positioning, or
        if the positioning move did not confirm arrival.
        """
        ctx = self._pending_print
        self._pending_print = None
        err = self._preflight_error
        self._preflight_error = None
        # Consume the sticky abort request (set by _on_abort while the preflight
        # worker ran). Race-free: both writes/reads are on the GUI thread.
        aborted_by_user = self._preflight_abort_requested
        self._preflight_abort_requested = False
        self._update_button_state()
        if not ctx:
            return
        well = ctx["well"]
        # The preamble (prep / ink pickup) aborted or errored → do NOT proceed
        # to the print; the worker's finally already retracted to safe Z.
        if err == "aborted" or aborted_by_user:
            self._status.setText("Aborted — needle retracted to safe Z.")
            return
        if err:
            self._status.setText(f"Setup / pickup failed: {err}")
            return

        # v7.5.x: re-verify the ZP stage is still connected — the pre-position
        # move can surface a board drop (now within ~1 s, not 15 s). Starting a
        # print against a dead board would dry-run with the needle parked down.
        if not getattr(self._controller, "is_zp_connected", False):
            self._status.setText(
                "ZP stage disconnected during positioning — reconnect it "
                "before printing.")
            return

        # Hands-free safety: with no operator confirm, refuse to print if the
        # positioning move did not confirm arrival (timeout / unsettled stage)
        # — the needle could be off-target. The needle is at the safe Z.
        if not positioned:
            self._status.setText(
                f"Positioning over {well} did not confirm — print aborted. "
                "Re-run once the stage settles.")
            return

        obj_label = ctx["obj_label"]
        center = ctx["center"]
        path_points = ctx["path_points"]
        path_segments = ctx["path_segments"]
        settings = ctx["settings"]
        pump = ctx["pump"]

        # Step 3: build + run the job. return_home=False → at the end the needle
        # retracts out of the well to the travel Z (TRAVEL_UP) and STOPS; it does
        # NOT drive the stage back to XY 0,0.
        job = build_well_plate_job(
            well_positions=[(well, center[0], center[1])],
            path_points=path_points,
            settings=settings,
            pump=pump,
            flow_rate=0.01,
            job_name=f"Quick Print — {obj_label} @ {well}",
            # v7.5.x: one PRINT_PATH per object with lift→travel→lower between
            # them, so a multi-object print doesn't extrude across the seams.
            path_segments=path_segments,
            # v7.21.5: each sub-path's own deposition (µL/mm per segment) from
            # the sketch's baked extrusion profile, so declared line widths and
            # no-extrude sections print as designed. None → one flow (legacy).
            path_extrusion_profiles=ctx.get("extrusion_profiles"),
            return_home=False,
        )

        # Arm the post-print cleanup (consumed in _on_state on COMPLETED).
        self._post_print_ctx = ctx.get("cleanup")

        pm = PrintManager(self._controller)
        bridge = self._bridge
        pm.on_progress = lambda c, t, m: bridge.progress.emit(int(c), int(t), str(m))
        pm.on_state_changed = lambda st: bridge.state.emit(st)
        # v7.7: live telemetry from the follower (already decimated to ~5 Hz by
        # the executor) → queued Qt signal → stored, rendered on the 10 Hz timer.
        pm.on_vel_sample = lambda rec: bridge.vel_sample.emit(rec)
        self._pm = pm
        try:
            pm.load_job(job)
            pm.start()
        except Exception as e:
            logger.exception("Quick Print start failed: %s", e)
            self._status.setText(f"Start failed: {e}")
            self._pm = None
            self._post_print_ctx = None  # no print → no cleanup; don't leak it
            self._update_button_state()
            return

        # Begin recording the live executed path over the planned preview.
        self._set_print_live(True)
        # v7.7: the operator's attention belongs on the Run zone now.
        self.show_zone("run")

        log_path = getattr(getattr(pm, "exec_logger", None), "path", None)
        if log_path is not None:
            self._status.setText(
                f"Printing “{obj_label}” at {well}…  (log: {log_path.name})")
        else:
            self._status.setText(f"Printing “{obj_label}” at {well}…")
        self._update_button_state()

    #: v7.6: live-position sampling period while a print runs (ms). The app's
    #: shared 300 ms tick is fine for idle, but too coarse to draw a moving
    #: needle — and it is starved by GUI-thread camera work.
    _LIVE_POS_MS = 100

    def _set_print_live(self, on: bool) -> None:
        """v7.6: start/stop the fast live-position sampling for a running print.

        The trajectory monitor used to FREEZE during every print: the print path
        suspends the position poller, and the monitor read the (now stale) poller
        cache through the 300 ms app tick. Two changes fix it —
        ``get_xy_position(cached=False)`` now back-fills that cache from the
        print loop's own 25–31 Hz reads (``PositionPoller.note_xy``), and this
        timer samples it at 10 Hz so the needle actually moves on screen.
        """
        if hasattr(self, "_traj_view"):
            if on:
                self._traj_view.reset_live()
            self._traj_view.set_recording(bool(on))
        # v7.7: reset the live-numbers accumulators at the start of each print.
        if on:
            self._live = {"t0": time.monotonic(), "sample": None,
                          "dev_max_um": 0.0, "devs": []}
        elif getattr(self, "_run_info_lbl", None) is not None:
            self._render_run_info(final=True)
        timer = getattr(self, "_live_pos_timer", None)
        if timer is None:
            return
        if on:
            timer.start(self._LIVE_POS_MS)
        else:
            timer.stop()
        # Ease GUI-thread camera work while the print needs the event loop.
        try:
            view = getattr(self, "_camera_view", None)
            thr = getattr(view, "set_throttled", None)
            if callable(thr):
                thr(bool(on))
        except Exception:
            pass

    def _on_vel_sample(self, rec) -> None:
        """v7.7: one telemetry record from the follower (GUI thread, queued).

        Only STORES it — the render happens on the existing 10 Hz timer, so a
        slow paint can never back-pressure the ~25 Hz control loop that this
        arrives from (already decimated to ~5 Hz by the executor).
        """
        live = getattr(self, "_live", None)
        if live is None or not isinstance(rec, dict):
            return
        live["sample"] = rec
        try:
            dev = abs(float(rec.get("cross_um") or 0.0))
        except (TypeError, ValueError):
            return
        live["dev_max_um"] = max(live.get("dev_max_um", 0.0), dev)
        devs = live.setdefault("devs", [])
        devs.append(dev)
        if len(devs) > 4000:                 # bounded; a long print can't grow
            del devs[:2000]

    def _render_run_info(self, final: bool = False) -> None:
        """Paint the live numbers. Called from the 10 Hz timer during a print."""
        lbl = getattr(self, "_run_info_lbl", None)
        if lbl is None:
            return
        live = getattr(self, "_live", None)
        if live is None:
            lbl.setText("Not printing.")
            return
        rec = live.get("sample") or {}
        elapsed = time.monotonic() - float(live.get("t0") or 0.0)
        element = 0.0
        try:
            element = float(self._resolution_um())
        except Exception:
            pass

        # Progress + ETA, from the monitor's arc-length projection over the WHOLE
        # planned path. Deliberately NOT from the record's s_mm/tot_mm: in a
        # feed-plan run those are SECTION-local, so they would read ~100 % once
        # per section.
        frac = None
        try:
            frac = self._traj_view.progress_fraction()
        except Exception:
            frac = None

        bits = []
        if frac is not None:
            eta = ""
            if frac > 0.1 and elapsed > 1.0:
                remain = elapsed * (1.0 - frac) / frac
                eta = f" · {remain:.0f} s left"
            bits.append(f"{frac * 100.0:5.1f}%  {elapsed:5.1f} s elapsed{eta}")
        else:
            bits.append(f"{elapsed:5.1f} s elapsed")

        # Deviation against the operator's OWN resolution element — both numbers
        # were in hand and were never compared.
        devs = live.get("devs") or []
        if devs:
            ordered = sorted(devs)
            p95 = ordered[min(len(ordered) - 1, int(0.95 * (len(ordered) - 1)))]
            now = devs[-1]
            verdict = "ok"
            if element > 0:
                verdict = ("ok" if p95 <= element else
                           "OVER the element" if p95 <= 2 * element else
                           "WELL OVER the element")
            bits.append(f"deviation {now:5.1f} µm now · p95 {p95:5.1f} · "
                        f"max {live.get('dev_max_um', 0.0):5.1f}"
                        + (f"  (element {element:.0f} µm — {verdict})"
                           if element > 0 else ""))

        # Volume: what the pump has actually been commanded, vs the plan. The
        # plan total uses the FULL path length (the record's tot_mm is the
        # current section's length, not the print's).
        dep = rec.get("deposited_uL")
        if dep is not None:
            planned = None
            try:
                _sp, flow, _pr = self._resolved_print_kinematics()
                total_mm = getattr(self, "_last_path_len_mm", None)
                if total_mm and _sp > 0:
                    planned = flow / _sp * float(total_mm)
            except Exception:
                planned = None
            line = f"dispensed {float(dep):7.4f} µL"
            if planned:
                line += f" of {planned:.4f} planned"
            bits.append(line)

        # Which section, and WHY it is slow there. In a feed plan the section
        # count is stops + 1, but only when the plan actually reported stops —
        # otherwise say nothing rather than "of 1".
        sec = rec.get("sec")
        if sec is not None:
            n = None
            stops = getattr(self, "_last_stops", None)
            if isinstance(stops, int) and stops > 0:
                n = stops + 1
            line = f"section {int(sec) + 1}" + (f" of {n}" if n else "")
            vm, vcmd = rec.get("v_meas_mm_s"), None
            try:
                vcmd = (float(rec.get("vx") or 0.0) ** 2
                        + float(rec.get("vy") or 0.0) ** 2) ** 0.5 / 1000.0
            except Exception:
                vcmd = None
            if vm is not None and vcmd:
                line += f" · {float(vm):.2f} mm/s measured vs {vcmd:.2f} commanded"
            bits.append(line)

        if final:
            bits.append("— print finished; see the Report zone.")
        lbl.setText("\n".join(bits))

    def _kick_abort_all_motion(self) -> None:
        """v7.6: kill XY + Z + pump motion NOW, on a daemon thread so the GUI
        never blocks (``abort_all_motion`` is bounded but touches serial).

        Idempotent — ``PrintManager.abort()``'s own worker also calls it; a
        second Prior ``I`` / ``VS 0,0`` / ``M410`` is harmless.
        """
        ctrl = self._controller
        fn = getattr(ctrl, "abort_all_motion", None)
        if not callable(fn):
            return
        def _work():
            try:
                fn("quick_print_abort")
            except Exception as e:              # pragma: no cover
                logger.warning("abort_all_motion failed: %s", e)
        threading.Thread(target=_work, name="qp-abort-motion",
                         daemon=True).start()

    def _on_abort(self):
        # v7.6: whichever phase is live, stop physical motion immediately —
        # the phase-specific flags below only stop the SOFTWARE from issuing
        # more commands.
        self._kick_abort_all_motion()
        # v7.19: a queued run — same shape as the multi-ink branch below, PLUS
        # waking a HELD worker. Without the _hold_event.set() an abort pressed
        # while the batch is parked would never reach the loop at all.
        if self._queue_thread is not None and self._queue_thread.is_alive():
            self._queue_abort_requested = True
            self._hold_event.set()
            ex = self._active_executor
            if ex is not None:
                try:
                    ex._abort_flag.set()
                except Exception:
                    pass
            if self._pm is not None:
                try:
                    self._pm.abort()
                except Exception:
                    pass
            self._status.setText(
                "Abort requested — finishing the current step…")
            return
        # v7.5.x: a multi-ink run — set its sticky flag AND abort whichever
        # sub-step is live (the executor between/around prints, or the
        # PrintManager during one group's print).
        if self._multi_thread is not None and self._multi_thread.is_alive():
            self._multi_abort_requested = True
            ex = self._active_executor
            if ex is not None:
                try:
                    ex._abort_flag.set()
                except Exception:
                    pass
            if self._pm is not None:
                try:
                    self._pm.abort()
                except Exception:
                    pass
            self._status.setText("Abort requested…")
            return
        # Abort routes to whichever stage is active: the pick-and-place preamble
        # (prep / ink pickup) during the preflight worker, else the PrintManager.
        ex = self._active_executor
        if ex is not None:
            # Sticky request consumed in _on_prepositioned — guarantees the
            # print does NOT start even if the abort flag was set during a
            # blocking move (pump / safe_travel) that no _check_abort() polls.
            self._preflight_abort_requested = True
            try:
                ex._abort_flag.set()
            except Exception as e:
                logger.warning("Quick Print preflight abort failed: %s", e)
            self._status.setText("Abort requested…")
            return
        if self._pm is None:
            return
        try:
            self._pm.abort()
        except Exception as e:
            logger.warning("Quick Print abort failed: %s", e)
        self._status.setText("Abort requested…")

    # ── Bridge slot handlers (main thread) ────────────────────────

    def _on_progress(self, done: int, total: int, msg: str):
        # total <= 0 → a preamble sub-step (prep / pickup) with no count.
        # v7.7: PrintManager._report_progress ALREADY prefixes "[i/total] " to
        # its own messages, so prefixing again produced "[14/57] [14/57] …".
        # Only add the counter when the message doesn't already carry one.
        text = msg if (total <= 0 or _PROGRESS_PREFIX_RE.match(msg)) \
            else f"[{done}/{total}] {msg}"
        # Remembered so a terminal state can report WHY it ended (the specific
        # error / ZP-disconnect text arrives here and used to be overwritten by
        # the generic "Error — see log.").
        self._last_progress_msg = msg
        self._status.setText(text)

    def _terminal_status_text(self, st, cleanup_armed: bool) -> str:
        """The operator-facing text for a finished print.

        v7.7: an abort or error leaves real fluidic state behind — the needle
        still holds ink/buffer because the cleanup routine is skipped, and a
        pump move cut mid-stroke makes the dispensed volume indeterminate (a
        fact ``PrintManager._abort_worker`` only wrote to the log file). Say so,
        because the next run's volumes depend on it.
        """
        if st == PrintState.COMPLETED:
            return "Done."
        last = (self._last_progress_msg or "").strip()
        # Strip any "[i/total] " counter so the reason reads cleanly.
        last = _PROGRESS_PREFIX_RE.sub("", last)
        if st == PrintState.ERROR:
            head = f"Error: {last}" if last else "Error — see log."
        else:
            head = "Aborted — needle retracted to safe Z."
        notes = ["the dispensed volume is indeterminate (a pump move may have "
                 "been cut mid-stroke) — re-check the syringe fill before the "
                 "next run"]
        if cleanup_armed:
            notes.insert(0, "cleanup did NOT run, so the needle still holds "
                            "ink/buffer")
        return head + "  ⚠ " + "; ".join(notes) + "."

    def _on_state(self, st):
        if st in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            log_path = getattr(
                getattr(self._pm, "exec_logger", None), "path", None)
            self._last_log_path = log_path
            self._pm = None
            # Stop accumulating; keep the executed trace visible on the plan.
            self._set_print_live(False)
            cleanup_ctx = self._post_print_ctx
            self._post_print_ctx = None
            if st == PrintState.COMPLETED and cleanup_ctx is not None:
                # Clean completion → waste / wash / reset-oil the needle.
                self._status.setText("Print done — cleaning needle…")
                self._start_cleanup_worker(cleanup_ctx)
            else:
                # Aborted / errored prints skip the cleanup (the print's own
                # finally already left the needle at safe Z).
                self._status.setText(self._terminal_status_text(
                    st, cleanup_ctx is not None))
            self._refresh_log_button()
            # v7.7: a finished print goes somewhere instead of nowhere.
            self._load_report(log_path)
        self._update_button_state()
        # v7.20: tell an embedding host LAST, so by the time its slot runs `_pm`
        # is cleared, `_last_log_path` is set, live sampling is stopped and any
        # cleanup worker has already been started (which the host must wait for
        # via `cleanup_running()` before trusting where the stage is).
        try:
            self.print_state_changed.emit(st)
        except Exception as e:
            logger.debug("print_state_changed emit failed: %s", e)

    def _start_cleanup_worker(self, cleanup_ctx: dict) -> None:
        """Run the post-print cleanup (waste → wash → reset oil) on a worker
        thread. Best-effort: always ends at safe Z; never blocks the GUI."""
        if (self._cleanup_thread is not None
                and self._cleanup_thread.is_alive()):
            return
        if not getattr(self._controller, "is_zp_connected", False):
            self._status.setText(
                "Print done — skipped cleanup (ZP board disconnected).")
            return
        bridge = self._bridge
        sz = float(self._safe_z) if self._safe_z is not None else None

        def _worker():
            err = None
            executor = None
            try:
                executor = PickPlaceExecutor(self._controller, self._hw_config)
                if sz is not None:
                    executor.safe_z_mm = sz
                executor.prep_bore = cleanup_ctx["bore"]
                executor.needle_volume_uL = cleanup_ctx["needle_uL"]
                executor.service_z_mm = cleanup_ctx["service_z"]
                executor.wash_cycles = int(cleanup_ctx["wash_cycles"])
                # v7.5.x "reset to initial condition": waste = live leftover +
                # margin; the needle multiples are fixed fallbacks only.
                executor.cleanup_reset_to_initial = bool(
                    cleanup_ctx.get("reset_to_initial", False))
                executor.cleanup_oil_margin_uL = float(
                    cleanup_ctx.get("oil_margin_uL", 0.0))
                executor.cleanup_waste_needles = cleanup_ctx["waste_needles"]
                executor.cleanup_oil_needles = cleanup_ctx["oil_needles"]
                executor.cleanup_oil_baseline_uL = cleanup_ctx["oil_baseline_uL"]
                sp = cleanup_ctx["service_positions"]
                executor.waste_well_pos = sp.get("waste")
                executor.wash_well_pos = sp.get("wash")
                executor.oil_well_pos = sp.get("oil")
                executor.on_sub_step = (
                    lambda op, step: bridge.progress.emit(0, 0, str(step)))
                self._active_executor = executor
                executor.run_print_cleanup()
            except AbortException:
                err = "aborted"
            except Exception as e:
                logger.exception("Quick Print cleanup failed: %s", e)
                err = str(e)
            finally:
                if executor is not None:
                    try:
                        executor._retract_to_safe_z()
                    except Exception:
                        pass
                self._active_executor = None
            bridge.cleanup_done.emit(err or "")

        self._cleanup_thread = threading.Thread(
            target=_worker, name="QuickPrintCleanup", daemon=True)
        self._cleanup_thread.start()
        self._update_button_state()

    def _on_cleanup_done(self, err: str):
        if err == "aborted":
            self._status.setText("Cleanup aborted — needle at safe Z.")
        elif err:
            self._status.setText(f"Print done. Cleanup failed: {err}")
        else:
            self._status.setText(
                "Print done — needle cleaned (waste → wash → reset oil).")
        self._update_button_state()
        # v7.20: the stage has stopped being driven by the cleanup — an embedding
        # host may now reposition and trust where the needle is.
        try:
            self.cleanup_finished.emit(err or "")
        except Exception as e:
            logger.debug("cleanup_finished emit failed: %s", e)

    def _on_pause(self):
        """Toggle pause on the running print (v7.7 — previously unreachable)."""
        pm = self._pm
        if pm is None:
            return
        try:
            if getattr(pm, "state", None) == PrintState.PAUSED:
                pm.resume()
                self._status.setText("Resumed.")
            else:
                pm.pause()
                self._status.setText(
                    "Pausing after the current command… (the needle stays put; "
                    "use Abort to stop motion now)")
        except Exception as e:
            logger.exception("Quick Print pause/resume failed: %s", e)
            self._status.setText(f"Pause failed: {e}")
        self._update_button_state()

    def _load_report(self, log_path, *, ideal_pts=None, context=None) -> None:
        """v7.7: build the Report zone from the run that just finished and show
        it. The prediction is passed alongside so the report can put predicted
        and measured side by side — the comparison that makes the simulator's
        optimism visible instead of a private discovery.

        v7.19 BUGFIX: *ideal_pts* / *context* may be given explicitly. Defaults
        preserve the single-print behaviour, but ``_ideal_path_mm`` reads the LIVE
        widgets and ``_selected_well``, so after a queue run the report would
        otherwise have scored the last unit's executed trace against whatever
        object happened to be on screen, in whatever well was selected — a
        silently wrong accuracy number in a report whose whole purpose is
        credible accuracy.

        Best-effort: a report that cannot be built must never disturb the
        machine state a finished print left behind.
        """
        panel = getattr(self, "_report_panel", None)
        if panel is None or log_path is None:
            return
        try:
            predicted = {}
            p95 = getattr(self, "_predicted_p95_um", None)
            if p95 is not None:
                predicted["p95_um"] = p95
            ideal = ideal_pts if ideal_pts is not None else self._ideal_path_mm()
            ctx = context if context is not None else {
                "object": self._object_label(),
                "well": self._selected_well or ""}
            ok = panel.load(log_path, ideal_pts=ideal, predicted=predicted,
                            context=ctx)
            if ok:
                self.show_zone("report")
        except Exception as exc:
            logger.exception("could not build the print report: %s", exc)

    def _ideal_path_mm(self) -> list | None:
        """The printed toolpath in zero-ref mm — the same absolute frame the
        executor logged, so the report can score against it."""
        try:
            segs = self._path_segments_for_selection()
            cx, cy = self._well_center_zero_ref_mm(self._selected_well)
        except Exception:
            return None
        pts = []
        for seg in segs or []:
            for (px, py) in seg:
                pts.append((cx + float(px), cy + float(py)))
        return pts if len(pts) >= 2 else None

    def _object_label(self) -> str:
        combo = getattr(self, "_object_combo", None)
        try:
            return combo.currentText() if combo is not None else ""
        except Exception:
            return ""

    def _refresh_log_button(self):
        if hasattr(self, "_log_btn"):
            self._log_btn.setEnabled(self._last_log_path is not None)

    def _on_open_log(self):
        """Open the last run's execution log in the OS default handler."""
        p = self._last_log_path
        if p is None:
            return
        from PySide6.QtCore import QUrl
        from PySide6.QtGui import QDesktopServices
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(p))):
            # No handler for .jsonl → reveal the containing folder instead.
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(p.parent)))

    def _update_button_state(self, *_):
        running = self._is_running()
        # v7.5.x: a pre-position move on the worker thread (or its pending
        # continuation) counts as busy so the periodic status tick can't
        # re-enable Print mid-positioning and let a second move launch.
        positioning = (self._pending_print is not None
                       or (self._preposition_thread is not None
                           and self._preposition_thread.is_alive())
                       or (self._cleanup_thread is not None
                           and self._cleanup_thread.is_alive())
                       or (self._multi_thread is not None
                           and self._multi_thread.is_alive())
                       or (self._queue_thread is not None
                           and self._queue_thread.is_alive()))
        connected = (getattr(self._controller, "is_xy_connected", False)
                     and getattr(self._controller, "is_zp_connected", False))
        ready = (connected and self._selected_well is not None
                 and self._plate is not None
                 and bool(self._object_combo.currentData()))
        # v7.7: the readiness model is the single source of truth when it has run.
        # By construction its BLOCK states are exactly the four conditions above
        # plus the one physically-impossible case (ink particles larger than the
        # needle bore), so this cannot start refusing prints that used to be
        # allowed — and a refusal can now SAY which precondition is missing.
        readiness = getattr(self, "_readiness", None)
        if readiness is not None:
            ready = readiness.can_print()
            blocking = readiness.blocking()
            self._print_btn.setToolTip(
                "Not ready — " + "; ".join(
                    f"{c.label}: {c.detail}" for c in blocking)
                if blocking else "Start the print.")
        self._print_btn.setEnabled(ready and not running and not positioning)
        if self._queue:
            self._print_btn.setToolTip(
                (self._print_btn.toolTip() or "")
                + f"\n(Runs only the ACTIVE print, in "
                  f"{self._selected_well or 'the selected well'}. Use “Run all "
                  f"queued” for the whole plate.)")
        # v7.19: Run all is gated on the readiness model MINUS the two checks that
        # describe the ACTIVE on-screen print — the queue supplies its own object
        # and well per entry, so blocking on those would refuse a perfectly valid
        # queue because the object combo was left empty. Per-entry refusals come
        # from _resolve_queue, by name.
        if hasattr(self, "_run_all_btn"):
            n_queued = len([qp for qp in self._queue if qp.enabled])
            blockers = []
            if readiness is not None:
                blockers = [c for c in readiness.blocking()
                            if c.id not in ("object", "well")]
            queue_ready = (connected and self._plate is not None
                           and n_queued > 0 and not blockers)
            self._run_all_btn.setEnabled(
                queue_ready and not running and not positioning)
            self._run_all_btn.setText(
                f"Run all queued ({n_queued})" if n_queued else "Run all queued")
            if n_queued == 0:
                self._run_all_btn.setToolTip(
                    "Nothing queued. Select wells on the plate and use “Apply "
                    "print to wells”.")
            elif blockers:
                self._run_all_btn.setToolTip(
                    "Not ready — " + "; ".join(
                        f"{c.label}: {c.detail}" for c in blockers))
            else:
                self._run_all_btn.setToolTip(
                    f"Run all {n_queued} queued prints: one needle prep, then "
                    "each well's own print, then one cleanup.")
        if hasattr(self, "_hold_btn"):
            queue_live = (self._queue_thread is not None
                          and self._queue_thread.is_alive())
            self._hold_btn.setEnabled(queue_live)
            if self._queue_is_held:
                left = max(0, self._queue_total - self._queue_ran)
                self._hold_btn.setText(f"Resume ({left} left)")
                self._hold_btn.setToolTip(
                    "Continue the queue. Refused while another workflow is "
                    "driving the stage.")
            else:
                self._hold_btn.setText("Hold after well")
                self._hold_btn.setToolTip(
                    "Finish the print now running, retract to safe Z and wait. "
                    "The stage is then free, so you can image the plate or run "
                    "another workflow before resuming."
                    if queue_live else
                    "Available while a queued run is in progress.")
        # v7.7: Abort is live during a print AND during ANY positioning phase.
        # It used to be gated on `self._active_executor is not None`, which left
        # it DEAD through a plain-print preposition — i.e. while the stage was
        # physically travelling. `_on_abort` always calls abort_all_motion()
        # first, which stops motion regardless of which phase we're in.
        self._abort_btn.setEnabled(running or positioning)
        paused = (self._pm is not None
                  and getattr(self._pm, "state", None) == PrintState.PAUSED)
        if hasattr(self, "_pause_btn"):
            self._pause_btn.setEnabled(running or paused)
            self._pause_btn.setText("Resume" if paused else "Pause")
        self._refresh_log_button()
