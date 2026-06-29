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

import copy
import logging
import threading
from typing import Optional

import numpy as np

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QMessageBox, QSplitter, QCheckBox, QSpinBox,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.jog_well_plate import WellPlateNavigator
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.print_trajectory_monitor import PrintTrajectoryMonitorView
from gui.dialogs.workflow_settings_dialog import (
    WorkflowSettingsDialog, build_locations_widget,
)
from gui.pages.workflows._reagent_prep import (
    SERVICE_ROLES, service_well_names, resolve_service_positions,
    needle_volume_uL,
)

try:
    from SupportClasses.HardwareConfig import CameraRole
except Exception:  # pragma: no cover - defensive import
    CameraRole = None

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, build_well_plate_job,
)
from SupportClasses.PrintFileManager import PrintFileManager
from SupportClasses.PickAndPlaceManager import PickPlaceExecutor, AbortException

logger = logging.getLogger(__name__)


# Built-in simple shapes → object dicts fed through the same geometry pipeline
# as saved objects. Size (mm) maps to circle/disc radius; the dot ignores it.
_SIMPLE_SHAPES = {
    "dot": "⋅ Dot",
    "circle": "◯ Circle",
    "meander": "◉ Meander (filled disc)",
}


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


class QuickPrintWorkflowPage(QWidget):
    """Quick Print workflow page.

    Signals:
        back_requested: User clicked the Back button.
    """

    back_requested = Signal()

    # v7.5.x print-setup routine step 4: pump pre-flow lead-in (seconds) — the
    # pump runs at the print flow rate for this long after the confirmed descent
    # to print Z and before the print trajectory begins.
    _PREFLOW_S = 0.25

    # Fallback for the calibrated XY max speed (mm/s) when neither the timing
    # calibration nor the safety limits report one — matches SafetyLimits'
    # default max_xy_speed (10_000 µm/s). Print speed = % × this.
    _XY_MAX_FALLBACK_MM_S = 10.0

    def __init__(self, controller, settings, camera_manager=None,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        # Live camera + trajectory monitor state.
        self._camera_view: CameraFeedView | None = None
        self._camera_started_by_us = False

        # Calibration data (pushed in by MainWindow fanout).
        self._plate = None
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None
        self._z_references: dict[str, float | None] = {
            "replace_z": None, "max_z": None,
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

        # Comprehensive settings popout (scrollable, saveable). Built eagerly so
        # the config widgets exist for _build_settings() / _on_print() + tests.
        self._settings_dialog = WorkflowSettingsDialog(
            "quick_print", "Quick Print",
            parent=self, on_change=self._on_settings_changed)
        self._build_settings_dialog(self._settings_dialog)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())
        outer.addWidget(self._build_object_row())
        outer.addWidget(self._build_status_strip())

        outer.addWidget(self._build_main_area(), stretch=1)

        outer.addWidget(self._build_run_row())

        self._refresh_objects()
        self._on_object_changed()
        self._refresh_ink_combo()
        self._refresh_setup_status()
        self._update_button_state()
        self._settings_dialog.load_last()
        self._refresh_setup_status()
        self._update_settings_summary()

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
            if self._camera_view.cam_idx != cam_idx:
                self._camera_view.set_camera(cam_idx)
            if not self._camera_manager.is_running(cam_idx):
                self._camera_manager.start(cam_idx)
                self._camera_started_by_us = True
        except Exception as e:
            logger.debug("Quick Print camera start failed: %s", e)

    def _stop_camera(self) -> None:
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
            self._settings_summary.setText(
                f"{self._pump()} · {self._flow_spin.value():g} µL/s @ "
                f"{self._speed_pct_spin.value():.0f}% · ink {ink} · {prep}")
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
        self._flow_spin = self._dspin(
            0.0, 50.0, 0.25, " µL/s", 3, 0.05,
            "Pump flow at 100% print speed. Print-speed % scales BOTH the XY "
            "traverse and this flow together, so the bead width is constant.")
        self._flow_spin.valueChanged.connect(
            lambda *_: (self._refresh_setup_status(), self._update_settings_summary()))
        self._speed_pct_spin = self._dspin(
            1.0, 100.0, 25.0, " % max", 0, 5.0,
            "Print speed as a % of the calibrated maximum XY speed; scales the "
            "XY traverse (= % × XY max) and the pump flow together.")
        self._speed_pct_spin.valueChanged.connect(
            lambda *_: (self._refresh_setup_status(), self._update_settings_summary()))
        self._printz_spin = self._dspin(
            0.0, 40.0, 0.2, " mm", 2, 0.1,
            "Print height measured up from the calibrated plate bottom. 0 = at "
            "the plate bottom; larger = higher. Clamped so it never goes below.")
        self._printz_spin.valueChanged.connect(
            lambda *_: self._refresh_setup_status())
        sec = dlg.add_section("Print")
        sec.add("pump", "Pump / bore", self._pump_combo, "P1")
        sec.add("flow", "Flow @100%", self._flow_spin, 0.25)
        sec.add("speed_pct", "Print speed", self._speed_pct_spin, 25.0)
        sec.add("printz", "Height above bottom", self._printz_spin, 0.2)

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
        self._pickup_safety_spin = self._dspin(
            1.0, 10.0, 1.5, "", 2, 0.1,
            "Safety multiplier on the computed ink pickup volume.")
        self._pickup_safety_spin.valueChanged.connect(
            lambda *_: self._refresh_setup_status())
        sec = dlg.add_section("Ink pickup")
        sec.add("ink", "Ink", self._ink_combo, "")
        sec.add("ink_z", "Ink dip Z (↑ bottom)", self._ink_z_spin, 0.50)
        sec.add("pickup_safety", "Pickup safety (×)", self._pickup_safety_spin, 1.5)

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
        sec.add("service_z", "Service dip Z (↑ bottom)", self._service_z_spin, 0.50)
        sec.add("buffer_needles", "Buffer (needles)", self._buffer_needles_spin, 1.0)
        sec.add("wash_cycles", "Wash cycles", self._wash_cycles_spin, 3)

        # ── Between print lines (lift between strokes / sub-paths) ──
        self._line_retract_spin = self._dspin(
            0.0, 40.0, 1.0, " mm", 2, 0.1,
            "Height the needle lifts above the print Z between separate strokes "
            "(sub-paths / objects) before travelling to the next one. A "
            "continuous fill prints as one stroke and is not lifted mid-fill.")
        self._line_z_speed_spin = self._dspin(
            0.0, 100.0, 0.0, " mm/s", 1, 1.0,
            "Quick-move Z speed for the inter-line lift + lower. 0 = use the "
            "controller default.")
        self._line_xy_speed_spin = self._dspin(
            0.0, 200.0, 0.0, " mm/s", 1, 1.0,
            "Quick-move XY speed for the inter-line travel. 0 = use the Advanced "
            "travel speed.")
        sec = dlg.add_section("Between print lines")
        sec.add("line_retract", "Retract after each line",
                self._line_retract_spin, 1.0)
        sec.add("line_z_speed", "Line-move Z speed", self._line_z_speed_spin, 0.0)
        sec.add("line_xy_speed", "Line-move XY speed", self._line_xy_speed_spin, 0.0)

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
        sec.add("preflow", "Pre-flow lead-in", self._preflow, self._prime_default_s())

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
        w = getattr(self, "_preflow", None)
        if w is None:
            return self._prime_default_s()
        try:
            return float(w.value())
        except Exception:
            return self._prime_default_s()

    def _build_main_area(self) -> QSplitter:
        """Vertical splitter: a top row of [trajectory monitor | live camera]
        over the well-plate selector. All boundaries are user-resizable."""
        # ── Top row: trajectory monitor + live camera ──────────────
        self._traj_view = PrintTrajectoryMonitorView()
        if self._controller is not None and hasattr(
                self._controller, "plate_flip_180"):
            self._traj_view.set_plate_flip_180(
                self._controller.plate_flip_180())
        traj_card = Card("Print plan — planned path · live trace · needle",
                         flush=True, compact=True)
        traj_card.add_widget(self._traj_view)

        if self._camera_manager is not None:
            self._camera_view = CameraFeedView(
                camera_manager=self._camera_manager,
                cam_idx=self._resolve_microscope_cam_idx(),
                show_crosshair=True,
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

        # ── Bottom: well selector ──────────────────────────────────
        self._navigator = WellPlateNavigator()
        self._navigator.well_clicked.connect(self._on_well_clicked)
        nav_card = Card("Well — click to place the object", flush=True,
                        compact=True)
        nav_card.add_widget(self._navigator)

        main_split = QSplitter(Qt.Vertical)
        main_split.setChildrenCollapsible(False)
        main_split.addWidget(top_split)
        main_split.addWidget(nav_card)
        main_split.setStretchFactor(0, 3)
        main_split.setStretchFactor(1, 2)
        main_split.setSizes([s(420), s(300)])
        return main_split

    def _build_run_row(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(10))

        self._print_btn = QPushButton("Print")
        self._print_btn.clicked.connect(self._on_print)
        row.addWidget(self._print_btn)

        self._abort_btn = QPushButton("Abort")
        self._abort_btn.setEnabled(False)
        self._abort_btn.clicked.connect(self._on_abort)
        row.addWidget(self._abort_btn)

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

    def _push_live_position(self) -> None:
        """Feed the live needle XY (zero-ref µm) into the trajectory monitor —
        the same conversion the Jog page uses (absolute stage µm − zero)."""
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
        self._object_combo.blockSignals(False)
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
        self._refresh_planned_path()
        self._refresh_setup_status()
        self._update_button_state()

    def _on_well_clicked(self, name: str) -> None:
        self._selected_well = name
        self._navigator.set_current_well(name)
        self._status.setText(f"Well {name} selected.")
        self._refresh_planned_path()
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

    def _obj_dict_to_path_points(self, obj_dict, needle, syringe_map):
        """Convert one object dict → list[(x_mm, y_mm)] relative to well center.

        Uses the persisted trajectory for csv-sourced objects, otherwise the
        canonical ``generate_object_trajectory`` pipeline. Only XY is used.
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

        if traj is None or len(traj) == 0 or np.asarray(traj).ndim != 2:
            return []
        arr = np.asarray(traj, dtype=np.float64)
        if arr.shape[1] < 2:
            return []
        return [(float(arr[i, 0]), float(arr[i, 1])) for i in range(len(arr))]

    def _path_segments_for_selection(self) -> list[list[tuple[float, float]]]:
        """One sub-path per object, relative to well center.

        A saved multi-object print (e.g. several spirals at different offsets)
        yields one segment per object, so ``build_well_plate_job`` inserts a
        lift→travel→lower between them instead of extruding across the seam and
        dragging the needle through already-printed material.
        """
        data = self._parse_obj_data(self._object_combo.currentData())
        if not data:
            return []
        kind, ref = data
        needle, syringe_map = self._needle_and_syringe()

        if kind == "simple":
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
        for od in obj_dicts:
            pts = self._obj_dict_to_path_points(od, needle, syringe_map)
            if pts:
                segments.append(pts)
        return segments

    def _path_points_for_selection(self) -> list[tuple[float, float]]:
        """Flattened path (all objects concatenated) — kept for geometry
        previews and the printable-path guard; execution uses the segmented
        form via :meth:`_path_segments_for_selection`."""
        points: list[tuple[float, float]] = []
        for seg in self._path_segments_for_selection():
            points.extend(seg)
        return points

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

    def _refresh_planned_path(self) -> None:
        """Recompute the planned toolpath for the selected well + object and
        push it (in zero-ref µm) into the trajectory monitor. Resets the live
        overlay so a previous run's trace doesn't linger on a new plan."""
        if not hasattr(self, "_traj_view"):
            return
        well = self._selected_well
        if not well or not self._object_combo.currentData():
            self._traj_view.set_planned_path(None)
            self._traj_view.set_well_boundary(None, 0.0)
            return
        center = self._well_center_zero_ref_mm(well)
        if center is None:
            self._traj_view.set_planned_path(None)
            self._traj_view.set_well_boundary(None, 0.0)
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
        self._traj_view.set_planned_path(seg_um or None)

        radius_um = self._well_radius_um(well)
        self._traj_view.set_well_boundary(
            (cx_um, cy_um) if radius_um > 0 else None, radius_um)

        # Size the needle marker from the configured needle OD (best-effort).
        try:
            needle, _ = self._needle_and_syringe()
            od_um = getattr(needle, "od_um", None)
            if not od_um:
                od_mm = getattr(needle, "od_mm", 0.0) or 0.0
                od_um = float(od_mm) * 1000.0
            if od_um:
                self._traj_view.set_needle(float(od_um))
        except Exception:
            pass

        self._traj_view.reset_live()

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
        return wells[0] if wells else None

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
        speed. Prefers the measured top speed from the timing calibration, else
        the safety-limit max (both µm/s); conservative fallback otherwise."""
        try:
            from SupportClasses.PrintTimingCalibrationStore import get_store
            ms = get_store().get_xy_max_speed_um_s()
            if ms and float(ms) > 0:
                return float(ms) / 1000.0
        except Exception:
            pass
        try:
            sl = getattr(self._controller, "safety_limits", None)
            v = getattr(sl, "max_xy_speed", None)
            if v is not None and float(v) > 0:
                return float(v) / 1000.0
        except (TypeError, ValueError):
            pass
        return self._XY_MAX_FALLBACK_MM_S

    def _print_speed_pct(self) -> float:
        """Print speed fraction in [0.01, 1.0] from the % spin."""
        try:
            return max(0.01, min(1.0, float(self._speed_pct_spin.value()) / 100.0))
        except Exception:
            return 0.25

    def _resolved_print_kinematics(self) -> tuple[float, float, float]:
        """Apply the print-speed % to both axes and return
        ``(print_speed_mm_s, flow_uL_s, prime_uL)``.

        The Flow knob is the flow at 100% speed; both the XY traverse (% × XY
        max) and the pump flow (% × Flow@100%) scale by the same %, so the
        deposited volume-per-mm (bead width) is independent of the % — the
        single lever scales all print-path waypoint pacing for XY and the pump.
        """
        pct = self._print_speed_pct()
        flow_100 = float(self._flow_spin.value())
        speed = pct * self._xy_max_mm_s()
        flow = pct * flow_100
        prime = flow * self._preflow_s()
        return speed, flow, prime

    def _compute_pickup_volume_uL(self) -> float:
        """Volume to aspirate at the ink well = what the print path dispenses
        (path length / print speed × flow) + the pre-flow prime, × safety;
        floored to the prime so a single-point (dot) path still draws ink.
        Uses the resolved (speed-%-scaled) kinematics. Returns 0 with no flow."""
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
        safety = float(self._pickup_safety_spin.value())
        return max((dispensed + prime) * safety, prime)

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
            self._hw_config, self._well_positions)
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
        msgs: list[str] = []
        warn = False
        # Resolved print kinematics (speed % applied to both XY and the pump).
        try:
            speed, flow, _prime = self._resolved_print_kinematics()
            msgs.append(f"Print: {speed:.1f} mm/s · {flow:.3f} µL/s")
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
                self._hw_config, self._well_positions)
            names = service_well_names(self._hw_config)
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
                self._hw_config, self._well_positions)
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

    def _build_settings(self) -> PrintSettings:
        # Print speed % scales the XY traverse AND the pump flow together.
        print_speed_mm_s, flow, prime_uL = self._resolved_print_kinematics()
        pump = self._pump()
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
        try:
            settings.intra_well_hop_z_mm = float(self._line_retract_spin.value())
        except Exception:
            pass
        try:
            settings.line_move_z_speed_mm_s = float(self._line_z_speed_spin.value())
        except Exception:
            pass
        try:
            settings.line_move_xy_speed_mm_s = float(
                self._line_xy_speed_spin.value())
        except Exception:
            pass
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

    def _on_print(self):
        if self._is_running():
            return
        if (self._preposition_thread is not None
                and self._preposition_thread.is_alive()):
            return  # a positioning / preflight move is already in flight
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

        center = self._well_center_zero_ref_mm(well)
        if center is None:
            self._status.setText(f"Could not resolve position for well {well}.")
            return

        try:
            path_segments = self._path_segments_for_selection()
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
                    self._hw_config, self._well_positions)
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
                    "Calibration page (or turn “Clean needle after print” off).")
                return
            cl_needle = needle_volume_uL(self._hw_config)
            if cl_needle <= 0:
                self._status.setText(
                    "Post-print clean needs the needle inner Ø + length "
                    "(Hardware Setup → Needle), or turn it off.")
                return
            cl_positions, _cl_missing = resolve_service_positions(
                self._hw_config, self._well_positions)
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
                            ink_pos, pickup_uL, bore=pump, z_mm=ink_dip_z)
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
            return_home=False,
        )

        # Arm the post-print cleanup (consumed in _on_state on COMPLETED).
        self._post_print_ctx = ctx.get("cleanup")

        pm = PrintManager(self._controller)
        bridge = self._bridge
        pm.on_progress = lambda c, t, m: bridge.progress.emit(int(c), int(t), str(m))
        pm.on_state_changed = lambda st: bridge.state.emit(st)
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
        if hasattr(self, "_traj_view"):
            self._traj_view.reset_live()
            self._traj_view.set_recording(True)

        log_path = getattr(getattr(pm, "exec_logger", None), "path", None)
        if log_path is not None:
            self._status.setText(
                f"Printing “{obj_label}” at {well}…  (log: {log_path.name})")
        else:
            self._status.setText(f"Printing “{obj_label}” at {well}…")
        self._update_button_state()

    def _on_abort(self):
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
        if total <= 0:
            self._status.setText(msg)
        else:
            self._status.setText(f"[{done}/{total}] {msg}")

    def _on_state(self, st):
        terminal = {
            PrintState.COMPLETED: "Done.",
            PrintState.ABORTED: "Aborted.",
            PrintState.ERROR: "Error — see log.",
        }
        if st in terminal:
            text = terminal[st]
            log_path = getattr(
                getattr(self._pm, "exec_logger", None), "path", None)
            if log_path is not None:
                text += f"  Execution log: logs/prints/{log_path.name}"
            self._pm = None
            # Stop accumulating; keep the executed trace visible on the plan.
            if hasattr(self, "_traj_view"):
                self._traj_view.set_recording(False)
            cleanup_ctx = self._post_print_ctx
            self._post_print_ctx = None
            if st == PrintState.COMPLETED and cleanup_ctx is not None:
                # Clean completion → waste / wash / reset-oil the needle.
                self._status.setText("Print done — cleaning needle…")
                self._start_cleanup_worker(cleanup_ctx)
            else:
                # Aborted / errored prints skip the cleanup (the print's own
                # finally already left the needle at safe Z).
                self._status.setText(text)
        self._update_button_state()

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

    def _update_button_state(self, *_):
        running = self._is_running()
        # v7.5.x: a pre-position move on the worker thread (or its pending
        # continuation) counts as busy so the periodic status tick can't
        # re-enable Print mid-positioning and let a second move launch.
        positioning = (self._pending_print is not None
                       or (self._preposition_thread is not None
                           and self._preposition_thread.is_alive())
                       or (self._cleanup_thread is not None
                           and self._cleanup_thread.is_alive()))
        connected = (getattr(self._controller, "is_xy_connected", False)
                     and getattr(self._controller, "is_zp_connected", False))
        ready = (connected and self._selected_well is not None
                 and self._plate is not None
                 and bool(self._object_combo.currentData()))
        self._print_btn.setEnabled(ready and not running and not positioning)
        # Abort is live during a print AND during the preflight preamble (when
        # there's an executor to interrupt); a plain preposition has nothing to
        # abort, so gate the positioning case on an active executor.
        self._abort_btn.setEnabled(
            running or (positioning and self._active_executor is not None))
