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

import logging
import threading
from typing import Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QSplitter, QMessageBox,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.live_target_picker import LiveTargetPicker
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.workspace_target_view import WorkspaceTargetView
from gui.widgets.xz_side_view import XZSideView

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

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())
        outer.addWidget(self._build_config_row())

        # Main horizontal split:
        #   LEFT  — LiveTargetPicker (shared camera + pick/place lists)
        #   RIGHT — vertical split: WorkspaceTargetView (top) + XZSideView (bottom)
        main_split = QSplitter(Qt.Horizontal, self)
        main_split.setChildrenCollapsible(False)

        self._picker = LiveTargetPicker(controller, camera_manager)
        self._picker.picks_changed.connect(self._on_targets_changed)
        self._picker.places_changed.connect(self._on_targets_changed)
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

        self._update_button_state()

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
        row.addStretch(1)
        return row

    def _build_config_row(self) -> QFrame:
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
        row.setSpacing(s(12))

        row.addWidget(QLabel("Diameter:"))
        self._diameter = QDoubleSpinBox()
        self._diameter.setRange(1.0, 5000.0)
        self._diameter.setSuffix(" µm")
        self._diameter.setDecimals(1)
        self._diameter.setValue(200.0)
        self._diameter.setSingleStep(10.0)
        row.addWidget(self._diameter)

        row.addWidget(QLabel("Bore:"))
        self._bore = QComboBox()
        self._bore.setMinimumWidth(s(110))
        row.addWidget(self._bore)

        row.addWidget(QLabel("Safety factor:"))
        self._safety = QDoubleSpinBox()
        self._safety.setRange(1.0, 5.0)
        self._safety.setDecimals(2)
        self._safety.setSingleStep(0.1)
        self._safety.setValue(1.5)
        row.addWidget(self._safety)

        row.addStretch(1)

        self._volume_label = QLabel("V = —")
        self._volume_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        row.addWidget(self._volume_label)

        self._diameter.valueChanged.connect(self._refresh_volume_label)
        self._safety.valueChanged.connect(self._refresh_volume_label)
        self._refresh_volume_label()

        return frame

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

    # ── hw_config hook ────────────────────────────────────────────

    def set_hardware_config(self, hw_config):
        self._hw_config = hw_config
        # Refresh bore options from pump ids
        self._bore.blockSignals(True)
        previous = self._bore.currentText()
        self._bore.clear()
        if hw_config is not None and hasattr(hw_config, "pumps"):
            for pid, pcfg in hw_config.pumps.items():
                # Only show enabled + configured pumps when those flags exist
                enabled = getattr(pcfg, "enabled", True)
                configured = getattr(pcfg, "is_configured", True)
                if enabled and configured:
                    self._bore.addItem(pid)
        if self._bore.count() == 0:
            self._bore.addItem("P1")
        # Restore previous selection if still present
        idx = self._bore.findText(previous)
        if idx >= 0:
            self._bore.setCurrentIndex(idx)
        self._bore.blockSignals(False)

        # Push to the shared live picker + workspace/XZ needle size
        self._picker.set_hardware_config(hw_config)
        try:
            needle = getattr(hw_config, "needle", None) if hw_config else None
            od_um = float(getattr(needle, "outer_diameter_mm", 0.0) or 0.0) * 1000.0
            length_mm = float(getattr(needle, "length_mm", 0.0) or 0.0)
            if od_um > 0:
                self._workspace_view.set_needle(od_um)
                self._xz_view.set_needle(od_um, length_mm or None)
        except Exception as e:
            logger.debug("workspace/xz set_needle failed: %s", e)

        if self._context_widget is not None:
            self._context_widget.set_hardware_config(hw_config)
        self._update_button_state()

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
        # v7.5.x: absolute envelope → push zero so the view draws it zero-ref.
        if hasattr(self._workspace_view, "set_zero_offset"):
            self._workspace_view.set_zero_offset(zero["x"], zero["y"])
        if hasattr(self._xz_view, "set_zero_offset_x"):
            self._xz_view.set_zero_offset_x(zero["x"])
        if hasattr(self._xz_view, "set_zero_offset_z"):
            self._xz_view.set_zero_offset_z(zero.get("Z", 0.0))
        if xy is not None and xy[0] is not None and xy[1] is not None:
            zx_um = float(xy[0]) * 1000.0 - zero["x"]
            zy_um = float(xy[1]) * 1000.0 - zero["y"]
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
                        float(xy[0]) * 1000.0 - zero["x"],
                        z_raw - zero["Z"],
                    )
            except Exception:
                pass

    # ── Workspace + XZ click handlers (mirror of JogControlPage) ──

    def _on_workspace_position_clicked(
        self, x_um_zr: float, y_um_zr: float
    ) -> None:
        """Click-to-travel from the XY workspace (zero-ref µm)."""
        if not getattr(self._controller, "is_xy_connected", False):
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
        self._controller.safe_travel_to(
            stage_x, stage_y, safe_z_mm=self._safe_z, target_z_mm=None)

    def _on_workspace_fast_travel_requested(
        self, x_um_zr: float, y_um_zr: float
    ) -> None:
        """Right-click → Fast travel here: retract Z, travel XY, restore Z."""
        if not getattr(self._controller, "is_xy_connected", False):
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

        self._controller.safe_travel_to(
            stage_x, stage_y,
            safe_z_mm=self._safe_z, target_z_mm=current_z_zr)

    def _on_go_to_z_requested(self, z_mm: float) -> None:
        """Z-reference badge in the XZ view → move_z_absolute (zero-ref mm)."""
        if not getattr(self._controller, "is_zp_connected", False):
            return
        try:
            self._controller.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("Go-to-Z failed: %s", exc)

    # ── config helpers ────────────────────────────────────────────

    def _current_config(self) -> SpheroidPickupConfig:
        return SpheroidPickupConfig(
            spheroid_diameter_um=float(self._diameter.value()),
            safety_factor=float(self._safety.value()),
            pickup_bore=self._bore.currentText() or "P1",
        )

    def _refresh_volume_label(self):
        cfg = self._current_config()
        v = cfg.compute_volume_uL()
        self._volume_label.setText(f"V = {v:.4f} µL  (×{cfg.safety_factor:.2f})")

    def _update_button_state(self, *_):
        balanced = self._picker.is_balanced()
        has_bore = self._bore.count() > 0
        running = self._exec_thread is not None and self._exec_thread.is_alive()
        self._start_btn.setEnabled(balanced and has_bore and not running)
        self._abort_btn.setEnabled(running)

    # ── Start / Abort ─────────────────────────────────────────────

    def _on_start(self):
        if self._exec_thread is not None and self._exec_thread.is_alive():
            return

        if not self._picker.is_balanced():
            self._status.setText(
                "Each pick needs a paired place — pick and place counts must match.")
            return

        pairs = self._picker.pairs()
        cfg = self._current_config()
        queue = OperationQueue()
        for pick, place in pairs:
            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=OperationType.SPHEROID_PICKUP,
                source_target=pick,
                dest_target=place,
                config=cfg,
            )
            queue.add(op)

        executor = PickPlaceExecutor(self._controller, self._hw_config)
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
