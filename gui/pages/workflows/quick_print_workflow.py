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
from typing import Optional

import numpy as np

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QDoubleSpinBox,
    QComboBox, QFrame, QSizePolicy, QMessageBox,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.components import Card
from gui.widgets.jog_well_plate import WellPlateNavigator
from gui.widgets.standard_jog_context import StandardJogContextPanel

from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, build_well_plate_job,
)
from SupportClasses.PrintFileManager import PrintFileManager

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


class QuickPrintWorkflowPage(QWidget):
    """Quick Print workflow page.

    Signals:
        back_requested: User clicked the Back button.
    """

    back_requested = Signal()

    def __init__(self, controller, settings, parent: QWidget | None = None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._hw_config = None

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

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))

        outer.addLayout(self._build_header())
        outer.addWidget(self._build_config_row())

        self._navigator = WellPlateNavigator()
        self._navigator.well_clicked.connect(self._on_well_clicked)
        nav_card = Card("Well — click to place the object", flush=True)
        nav_card.add_widget(self._navigator)
        outer.addWidget(nav_card, stretch=1)

        outer.addWidget(self._build_run_row())

        self._refresh_objects()
        self._on_object_changed()
        self._update_button_state()

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
        row.addWidget(self._size_spin)

        row.addWidget(QLabel("Pump:"))
        self._pump_combo = QComboBox()
        self._pump_combo.setMinimumWidth(s(70))
        self._pump_combo.addItem("P1")
        row.addWidget(self._pump_combo)

        row.addWidget(QLabel("Flow:"))
        self._flow_spin = QDoubleSpinBox()
        self._flow_spin.setRange(0.0, 50.0)
        self._flow_spin.setDecimals(3)
        self._flow_spin.setSingleStep(0.05)
        self._flow_spin.setValue(0.25)
        self._flow_spin.setSuffix(" µL/s")
        row.addWidget(self._flow_spin)

        printz_label = QLabel("Height above bottom:")
        printz_label.setToolTip(
            "Print height measured up from the calibrated plate bottom.\n"
            "0 = at the plate bottom; larger = higher above it. The needle is "
            "clamped so it can never go below the plate bottom.")
        row.addWidget(printz_label)
        self._printz_spin = QDoubleSpinBox()
        self._printz_spin.setRange(0.0, 40.0)
        self._printz_spin.setDecimals(2)
        self._printz_spin.setSingleStep(0.1)
        self._printz_spin.setValue(0.2)
        self._printz_spin.setSuffix(" mm")
        self._printz_spin.setToolTip(printz_label.toolTip())
        row.addWidget(self._printz_spin)

        row.addStretch(1)
        return frame

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
        self._update_button_state()

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
        self._update_button_state()

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
        self._update_button_state()

    def _on_well_clicked(self, name: str) -> None:
        self._selected_well = name
        self._navigator.set_current_well(name)
        self._status.setText(f"Well {name} selected.")
        self._update_button_state()

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
                if csv_data is None and "source_file" in params:
                    try:
                        from SupportClasses.TrajectoryPlanner import (
                            import_csv_trajectory,
                        )
                        csv_data = import_csv_trajectory(params["source_file"])
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

    def _path_points_for_selection(self) -> list[tuple[float, float]]:
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

        points: list[tuple[float, float]] = []
        for od in obj_dicts:
            points.extend(self._obj_dict_to_path_points(od, needle, syringe_map))
        return points

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
                return self._plate.get_well_position(well)
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
        flow = float(self._flow_spin.value())
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
        settings = PrintSettings(
            num_layers=1,
            travel_z_height=travel_z,
            print_z_height=self._resolve_print_z(),
            pump_rate_uL_s=flow,
            print_speed_mm_s=5.0,
            travel_speed_mm_s=10.0,
        )
        # v7.5.x: stamp the reference-vector up-direction (print_z_height above
        # is already polarity-correct via controller.print_height_to_zref).
        try:
            settings.z_up_sign = float(self._controller.print_z_dir())
        except (TypeError, ValueError, AttributeError):
            pass  # keep the PrintSettings default (legacy additive)
        try:
            settings.pump_rates_uL_s[pump] = flow
        except Exception:
            pass
        return settings

    # ── Run / abort ───────────────────────────────────────────────

    def _is_running(self) -> bool:
        return self._pm is not None and getattr(
            self._pm, "state", None) == PrintState.RUNNING

    def _on_print(self):
        if self._is_running():
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

        center = self._well_center_zero_ref_mm(well)
        if center is None:
            self._status.setText(f"Could not resolve position for well {well}.")
            return

        if self._safe_z is None:
            resp = QMessageBox.question(
                self, "No Safe Z",
                "No safe Z is configured. The needle will not retract to a safe "
                "height before travel. Continue anyway?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if resp != QMessageBox.StandardButton.Yes:
                return

        try:
            path_points = self._path_points_for_selection()
        except Exception as e:
            logger.exception("Quick Print geometry failed: %s", e)
            self._status.setText(f"Geometry error: {e}")
            return
        if not path_points:
            self._status.setText("Selected object produced no printable path.")
            return

        # v7.5.x: early-warn if the configured print height would punch through
        # the plate bottom (the controller still hard-clamps during motion).
        if not self._confirm_print_floor():
            return

        obj_label = self._object_combo.currentText()
        resp = QMessageBox.question(
            self, "Quick Print",
            f"Move the needle to well {well} and print “{obj_label}”?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if resp != QMessageBox.StandardButton.Yes:
            return

        settings = self._build_settings()
        pump = self._pump()
        job = build_well_plate_job(
            well_positions=[(well, center[0], center[1])],
            path_points=path_points,
            settings=settings,
            pump=pump,
            flow_rate=0.01,
            job_name=f"Quick Print — {obj_label} @ {well}",
        )

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
            self._update_button_state()
            return

        log_path = getattr(getattr(pm, "exec_logger", None), "path", None)
        if log_path is not None:
            self._status.setText(
                f"Printing “{obj_label}” at {well}…  (log: {log_path.name})")
        else:
            self._status.setText(f"Printing “{obj_label}” at {well}…")
        self._update_button_state()

    def _on_abort(self):
        if self._pm is None:
            return
        try:
            self._pm.abort()
        except Exception as e:
            logger.warning("Quick Print abort failed: %s", e)
        self._status.setText("Abort requested…")

    # ── Bridge slot handlers (main thread) ────────────────────────

    def _on_progress(self, done: int, total: int, msg: str):
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
            self._status.setText(text)
            self._pm = None
        self._update_button_state()

    def _update_button_state(self, *_):
        running = self._is_running()
        connected = (getattr(self._controller, "is_xy_connected", False)
                     and getattr(self._controller, "is_zp_connected", False))
        ready = (connected and self._selected_well is not None
                 and self._plate is not None
                 and bool(self._object_combo.currentData()))
        self._print_btn.setEnabled(ready and not running)
        self._abort_btn.setEnabled(running)
