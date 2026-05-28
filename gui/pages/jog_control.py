"""
Jog Control Page — v7.4.3 visualization-first cockpit.

Main content (no controls, just live state):

    ┌────────────────────────────────────────┐┌──────────────────┐
    │                                        ││                  │
    │   XY top-down workspace                ││  XZ side view    │
    │   (dominant — takes most of the page)  ││  (narrow)        │
    │                                        ││                  │
    │                                        ││                  │
    └────────────────────────────────────────┘└──────────────────┘
    ┌────────────────────────────────────────────────────────────┐
    │  ┌──────────┐  ┌──────────┐  ┌──────────┐                  │
    │  │   P1     │  │   P2     │  │   P3     │                  │
    │  │  syringe │  │  syringe │  │  syringe │                  │
    │  │  ──────  │  │  ──────  │  │  ──────  │                  │
    │  │  42.3 µL │  │  18.5 µL │  │   —      │                  │
    │  │  Ink A   │  │  Buffer  │  │ unconfig │                  │
    │  └──────────┘  └──────────┘  └──────────┘                  │
    └────────────────────────────────────────────────────────────┘

The left context panel is the project-wide reusable
:class:`StandardJogContextPanel` (the same jog UI used on the
Calibration page, plus an Absolute Go To section and a Hardware Info
quick-reference at the bottom).

Click-to-travel on the workspace canvas is the only "control" exposed
in the main area — clicking the visualization itself targets a
destination. Defaults to Snap-to-wells; the canvas has a Free/Snap
toggle bar at the top.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (
    QHBoxLayout, QMessageBox, QVBoxLayout, QWidget,
)

from SupportClasses.StageController import StageController
from gui.scaling import s
from gui.widgets.components import Card
from gui.widgets.jog_workspace_view import JogWorkspaceView
from gui.widgets.pump_rack import PumpRack
from gui.widgets.standard_jog_context import StandardJogContextPanel
from gui.widgets.xz_side_view import XZSideView

logger = logging.getLogger(__name__)


# Keyboard-shortcut default step sizes. Independent of the on-screen
# jog buttons (which live in the StandardJogContextPanel's
# JogButtonArray) — these are sensible "tap the arrow keys for medium
# nudges" defaults.
_KEY_STEP_XY_UM = 100.0
_KEY_STEP_Z_MM = 0.1


class JogControlPage(QWidget):
    """v7.4.3 Jog page — visualization in the main area, controls on the left."""

    _page_title_text = "Jog Control"

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._context_widget: StandardJogContextPanel | None = None
        self._hardware_config = None
        self._settings = None

        # Calibration state
        self._plate = None
        # well_name → (x_um_stage_frame, y_um_stage_frame)
        self._well_positions: dict[str, tuple[float, float]] | None = None
        self._safe_z: float | None = None

        # XY position scale (legacy hook for MainWindow)
        from gui.unit_helpers import DEFAULT_XY_POSITION_SCALE
        self._xy_position_scale: float = DEFAULT_XY_POSITION_SCALE

        self._setup_ui()
        self._setup_shortcuts()

    # ── Required by MainWindow ─────────────────────────────────────

    def get_page_title(self) -> str:
        return self._page_title_text

    def get_context_widget(self) -> QWidget:
        if self._context_widget is None:
            self._context_widget = StandardJogContextPanel(
                controller=self.controller,
                settings=self._settings,
                show_connect=False,
                bypass_safety=False,
            )
            if self._hardware_config is not None:
                self._context_widget.set_hardware_config(self._hardware_config)
            if any(v is not None for v in
                   (self._plate, self._well_positions, self._safe_z)):
                self._context_widget.set_calibration_data(
                    self._plate, self._well_positions, self._safe_z)
            # Push any Z-references captured before this widget existed
            if (hasattr(self._context_widget, 'set_z_references')
                    and any(v is not None
                            for v in self._z_references.values())):
                try:
                    self._context_widget.set_z_references(
                        self._z_references)
                except Exception:
                    pass
        return self._context_widget

    def set_xy_position_scale(self, value: float) -> None:
        """Legacy hook from MainWindow — kept for compat."""
        self._xy_position_scale = max(0.001, value)

    def set_settings(self, settings) -> None:
        """Optional hook — wired by MainWindow if available."""
        self._settings = settings
        if self._context_widget is not None:
            self._context_widget.set_settings(settings)

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT — visualization only
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self) -> None:
        # Vertical split: visualisations on top (XY left + XZ right),
        # pump rack across the full width below.
        root = QVBoxLayout(self)
        root.setContentsMargins(s(12), s(8), s(12), s(8))
        root.setSpacing(s(8))

        # ── Top row: XY (max space) + XZ (narrow) ──────────────
        top = QHBoxLayout()
        top.setSpacing(s(8))

        self._workspace_view = JogWorkspaceView()
        self._workspace_view.set_safety_limits(self.controller.safety_limits)
        self._workspace_view.position_clicked.connect(
            self._on_workspace_position_clicked)
        self._workspace_view.well_clicked.connect(
            self._on_workspace_well_clicked)
        self._workspace_view.fast_travel_requested.connect(
            self._on_workspace_fast_travel_requested)

        # Pending Z-ref values from the calibration page (None until
        # set_z_references is called). Pushed into the XZ view after
        # it's constructed below.
        self._z_references: dict[str, float | None] = {
            "replace_z": None, "max_z": None,
            "fast_move_z": None, "plate_top_z": None,
            "plate_bottom_z": None,
        }

        # Flush card so the workspace canvas fills the entire interior
        # width — no padding bars on the left/right edges.
        ws_card = Card("XY Workspace", flush=True)
        ws_card.add_widget(self._workspace_view)
        top.addWidget(ws_card, stretch=10)

        self._xz_view = XZSideView()
        self._xz_view.set_safety_limits(self.controller.safety_limits)
        self._xz_view.go_to_z_requested.connect(
            self._on_go_to_z_requested)

        xz_card = Card("Side View (XZ)", flush=True)
        xz_card.add_widget(self._xz_view)
        top.addWidget(xz_card, stretch=3)

        # ── Bottom row: pump rack (full width, 3 columns) ──────
        self._pump_panel = PumpRack()
        pump_card = Card("Pumps")
        pump_card.add_widget(self._pump_panel)

        # The top band keeps the lion's share of vertical space; the
        # pump rack stays compact thanks to its own min/max height.
        root.addLayout(top, stretch=1)
        root.addWidget(pump_card)

    # ════════════════════════════════════════════════════════════════
    #  STATUS UPDATE — main-area visualizations + forward to panel
    # ════════════════════════════════════════════════════════════════

    def on_status_update(self) -> None:
        """Called by MainWindow timer (~300 ms)."""
        ctrl = self.controller
        zero = ctrl.zero_position

        # XY position (stage-frame → zero-ref for the visualizations)
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            zx_um = xy[0] - zero["x"]
            zy_um = xy[1] - zero["y"]
            self._workspace_view.set_position(zx_um, zy_um)
        else:
            zx_um = None
            zy_um = None
            self._workspace_view.set_position(None, None)

        # Z position
        zp = ctrl.get_zp_position(cached=True)
        z_raw = ctrl.zp_logical_value(zp, "Z") if zp[0] is not None else None
        if z_raw is not None:
            self._xz_view.set_position(zx_um, z_raw - zero["Z"])
        else:
            self._xz_view.set_position(zx_um, None)

        # XZ view: well under needle (drawn under the needle when over a well)
        self._update_xz_well_under_needle(zx_um, zy_um)

        # Pump rack
        self._refresh_pump_panel()

        # Forward tick to the context panel (it owns its own readouts)
        if self._context_widget is not None:
            self._context_widget.on_status_update()

    def _update_xz_well_under_needle(
        self, zx_um: float | None, zy_um: float | None,
    ) -> None:
        if (self._plate is None or self._well_positions is None
                or zx_um is None):
            self._xz_view.set_well_under_needle(None, None)
            return
        well_r_um = (self._plate.well_diameter / 2.0) * 1000.0
        zero = self.controller.zero_position
        for _, (wx_stage, wy_stage) in self._well_positions.items():
            wx_zr = wx_stage - zero["x"]
            wy_zr = wy_stage - zero["y"]
            if abs(wx_zr - zx_um) <= well_r_um and abs(wy_zr - zy_um) <= well_r_um:
                self._xz_view.set_well_under_needle(
                    self._plate.well_diameter, self._plate.well_depth_mm)
                return
        self._xz_view.set_well_under_needle(None, None)

    def _refresh_pump_panel(self) -> None:
        if self._hardware_config is None:
            return
        try:
            from SupportClasses.PhysicalModels import (
                PrintingMode, PumpLoadout,
            )
        except ImportError:
            return

        ctrl = self.controller
        zp = ctrl.get_zp_position(cached=True)

        loadouts: dict[str, PumpLoadout] = {}
        for pid in ("P1", "P2", "P3"):
            pump_cfg = self._hardware_config.pumps.get(pid)
            if pump_cfg is None or not pump_cfg.is_configured:
                continue
            p_raw = (ctrl.zp_logical_value(zp, pid)
                     if zp[0] is not None else None)
            position_mm = 0.0
            if p_raw is not None:
                position_mm = p_raw - ctrl.zero_position[pid]
            loadouts[pid] = PumpLoadout(
                pump_id=pid,
                syringe=pump_cfg.syringe,
                fluid_column=pump_cfg.fluid_column,
                printing_mode=PrintingMode.INCREMENTAL,
                current_position_mm=float(position_mm),
            )
        if loadouts:
            self._pump_panel.update_from_workspace(loadouts)

    # ════════════════════════════════════════════════════════════════
    #  HARDWARE CONFIG + CALIBRATION
    # ════════════════════════════════════════════════════════════════

    def set_hardware_config(self, config) -> None:
        self._hardware_config = config
        # Push needle metadata into the visualizations
        od = None
        length_mm = None
        if config is not None and getattr(config, 'needle', None) is not None:
            try:
                od = float(config.needle.outer_diameter_um)
            except (AttributeError, TypeError, ValueError):
                pass
            try:
                length_mm = float(config.needle.length_mm)
            except (AttributeError, TypeError, ValueError):
                pass
        self._workspace_view.set_needle(od)
        self._xz_view.set_needle(od, length_mm)
        # Refresh safety limits in case HW config changed them
        self._workspace_view.set_safety_limits(self.controller.safety_limits)
        self._xz_view.set_safety_limits(self.controller.safety_limits)
        # Forward into the standard context panel
        if self._context_widget is not None:
            self._context_widget.set_hardware_config(config)

    def set_z_references(self, refs: dict) -> None:
        """Forward captured Z heights from the Calibration page.

        Updates the XZ side view (which renders clickable badges for
        each Z) and the StandardJogContextPanel (which lists them in
        the Hardware Info card).
        """
        if not refs:
            return
        for key in self._z_references:
            if key in refs:
                value = refs[key]
                self._z_references[key] = (
                    float(value) if value is not None else None)
        # Also keep the legacy safe_z value in sync
        if "fast_move_z" in refs and refs["fast_move_z"] is not None:
            self._safe_z = float(refs["fast_move_z"])
        if self._xz_view is not None:
            self._xz_view.set_z_references(self._z_references)
        if self._context_widget is not None:
            try:
                self._context_widget.set_z_references(self._z_references)
            except Exception:
                pass

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        """Receive calibration state from the calibration page.

        Args:
            plate: WellPlate instance (or None).
            well_positions: dict mapping well_name → (x_um, y_um) in
                            stage-frame µm (or None).
            safe_z: Safe travel Z height in mm (zero-ref), or None.
        """
        self._plate = plate
        self._well_positions = well_positions
        self._safe_z = safe_z
        if plate is not None:
            self._workspace_view.set_plate(plate)
            # v7.4.4: when a new plate arrives, the leftover approximate
            # wells from ``load_startup_plate`` (which uses settings,
            # not the hardware config) may be for a different format.
            # Wipe them so the workspace shows only the plate that
            # actually belongs to the current hardware config.
            self._workspace_view.set_well_positions({}, "approximate")
        self._workspace_view.set_well_positions(
            self._wells_in_zero_ref(), "calibrated")
        self._xz_view.set_safe_z(safe_z)
        # Forward into the standard context panel
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, well_positions, safe_z)

    def load_startup_plate(self, settings) -> None:
        """Populate the workspace with geometry-predicted (approximate) wells.

        With no real calibration yet, the plate is centred on the
        controller's zero reference — which means it appears at
        (0, 0) in zero-ref µm, i.e. dead-centre of the safety envelope
        on the XY workspace canvas. Once the user calibrates wells the
        Calibration page pushes their actual positions and this is
        replaced wholesale.
        """
        try:
            from SupportClasses.WellPlate import WellPlate
        except ImportError:
            return
        plate_format = (
            settings.get("calibration.plate_format")
            or settings.get("workspace.plate_format")
            or 96
        )
        try:
            plate = WellPlate.from_format(int(plate_format))
        except (ValueError, TypeError):
            plate = WellPlate.from_format(96)

        # Stage-frame coords whose zero-ref representation is (0, 0).
        zero = self.controller.zero_position
        center_x = float(zero.get("x", 0.0))
        center_y = float(zero.get("y", 0.0))
        approx_positions = plate.get_all_positions_from_plate_center(
            center_x, center_y)

        self._plate = plate
        self._well_positions = approx_positions
        self._workspace_view.set_plate(plate)
        self._workspace_view.set_well_positions(
            self._wells_in_zero_ref(), "approximate")
        if self._context_widget is not None:
            self._context_widget.set_calibration_data(
                plate, approx_positions, self._safe_z)

    def _wells_in_zero_ref(self) -> dict[str, tuple[float, float]]:
        if not self._well_positions:
            return {}
        zero = self.controller.zero_position
        return {
            name: (wx - zero["x"], wy - zero["y"])
            for name, (wx, wy) in self._well_positions.items()
        }

    # ════════════════════════════════════════════════════════════════
    #  WORKSPACE CLICK HANDLING
    # ════════════════════════════════════════════════════════════════

    def _on_workspace_position_clicked(
        self, x_um_zr: float, y_um_zr: float,
    ) -> None:
        """Click-to-travel from the XY workspace.

        Coordinates are zero-ref µm. Routes through safe_travel_to when
        current Z is below safe_Z, direct move_xy_absolute otherwise.
        """
        if not self.controller.is_xy_connected:
            return

        zero = self.controller.zero_position
        stage_x = x_um_zr + zero["x"]
        stage_y = y_um_zr + zero["y"]

        zp = self.controller.get_zp_position(cached=True)
        current_z = None
        if zp[0] is not None:
            z_raw = self.controller.zp_logical_value(zp, "Z")
            if z_raw is not None:
                current_z = z_raw - zero["Z"]

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
            self.controller.move_xy_absolute(
                x_um_zr, y_um_zr, from_zero_ref=True)
            return

        if current_z is not None and current_z >= self._safe_z - 0.05:
            self.controller.move_xy_absolute(
                x_um_zr, y_um_zr, from_zero_ref=True)
            return

        self.controller.safe_travel_to(
            stage_x, stage_y, safe_z_mm=self._safe_z, target_z_mm=None)

    def _on_workspace_well_clicked(self, well_name: str) -> None:
        logger.debug("Workspace: well clicked %s", well_name)

    def _on_go_to_z_requested(self, z_mm: float) -> None:
        """Click on a Z-reference badge in the XZ view → move Z there.

        Z is in mm, zero-referenced (the same frame the calibration
        page captures). Uses the standard ``move_z_absolute`` so soft
        limits are still respected.
        """
        if not self.controller.is_zp_connected:
            logger.info("Go-to-Z: ZP stage not connected")
            return
        logger.info("Go-to-Z: %.2f mm (zero-ref)", z_mm)
        try:
            self.controller.move_z_absolute(z_mm, from_zero_ref=True)
        except Exception as exc:
            logger.warning("Go-to-Z failed: %s", exc)

    def _on_workspace_fast_travel_requested(
        self, x_um_zr: float, y_um_zr: float,
    ) -> None:
        """Right-click → Fast travel here: full safe-travel workflow.

        1. Raise Z to safe-travel height.
        2. Wait for Z to confirm at safe height.
        3. Fast XY move to the target.
        4. Lower Z back to the original height.

        Steps 1–3 + the final Z descent are orchestrated by
        :meth:`StageController.safe_travel_to`, which already does the
        ``flush_moves`` + ``wait_for_z_arrival`` double-check before
        starting XY. Passing ``target_z_mm`` = current Z (zero-ref)
        makes step 4 restore the original Z.
        """
        if not self.controller.is_xy_connected:
            logger.info("Fast travel: XY stage not connected")
            return

        if self._safe_z is None:
            QMessageBox.warning(
                self, "No Safe Z",
                "Fast travel requires a safe Z height. Run the "
                "Calibration page first to set one.")
            return

        zero = self.controller.zero_position
        stage_x = float(x_um_zr) + zero["x"]
        stage_y = float(y_um_zr) + zero["y"]

        # Read current Z (zero-ref mm) so we can return to it after XY.
        current_z_zr: float | None = None
        if self.controller.is_zp_connected:
            zp = self.controller.get_zp_position(cached=True)
            if zp[0] is not None:
                z_raw = self.controller.zp_logical_value(zp, "Z")
                if z_raw is not None:
                    current_z_zr = z_raw - zero["Z"]

        logger.info(
            "Fast travel → (%.1f, %.1f) µm, safe_z=%.2f mm, "
            "return_z=%s",
            x_um_zr, y_um_zr, self._safe_z,
            f"{current_z_zr:.2f} mm" if current_z_zr is not None else "none",
        )

        self.controller.safe_travel_to(
            stage_x, stage_y,
            safe_z_mm=self._safe_z,
            target_z_mm=current_z_zr,
        )

    # ════════════════════════════════════════════════════════════════
    #  KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════

    def _setup_shortcuts(self) -> None:
        self._key_actions = {
            Qt.Key.Key_Left:     lambda: self._key_jog_xy(-1, 0),
            Qt.Key.Key_Right:    lambda: self._key_jog_xy(+1, 0),
            Qt.Key.Key_Up:       lambda: self._key_jog_xy(0, -1),
            Qt.Key.Key_Down:     lambda: self._key_jog_xy(0, +1),
            Qt.Key.Key_PageUp:   lambda: self._key_jog_z(+1),
            Qt.Key.Key_PageDown: lambda: self._key_jog_z(-1),
            Qt.Key.Key_Escape:   self._emergency_stop,
        }

    def _key_jog_xy(self, dx_sign: int, dy_sign: int) -> None:
        if not self.controller.is_xy_connected:
            return
        self.controller.move_xy_relative_um(
            dx_sign * _KEY_STEP_XY_UM, dy_sign * _KEY_STEP_XY_UM)

    def _key_jog_z(self, dz_sign: int) -> None:
        if not self.controller.is_zp_connected:
            return
        self.controller.move_z_relative(dz_sign * _KEY_STEP_Z_MM)

    def _emergency_stop(self) -> None:
        if self.controller.zp_stage:
            try:
                self.controller.zp_stage.emergency_stop()
                logger.warning("EMERGENCY STOP sent to ZP stage")
            except Exception as exc:
                logger.error("ZP e-stop failed: %s", exc)
        if self.controller.xy_stage:
            try:
                self.controller.xy_stage.stop_stage()
                logger.warning("XY STOP sent")
            except Exception as exc:
                logger.error("XY stop failed: %s", exc)

    def keyPressEvent(self, event: QKeyEvent):
        action = self._key_actions.get(event.key())
        if action and not event.isAutoRepeat():
            action()
        else:
            super().keyPressEvent(event)
