"""
Print Setup Page — v7.1 Tabbed Workspace with Execution Controls.

PyDracula layout:
    Main content  = 3-tab workflow (Workspace / Print Objects / Well Setup)
    Context panel = Print Settings, Execution Controls, Print Queue

v7.1 restructure: The original File/WellPlate/Pattern tabs are replaced by
a 3-tab workspace-oriented workflow:
    Tab 1 — Workspace:     Syringe/needle/ink/rosette configuration
    Tab 2 — Print Objects:  Parametric object design, CSV import, collections
    Tab 3 — Well Setup:    Well assignment, roles, plane calibration

The execution engine, print queue, progress tracking, and context panel
are preserved from v7.0 but updated to work with the new WorkspaceConfig-
driven pipeline.

Interface contract:
    get_page_title()     → str
    get_context_widget() → QWidget  (execution controls + queue)
    on_status_update()   → called by MainWindow timer
    resume_print(data)   → called from MainWindow resume dialog
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QProgressBar, QFrame,
    QCheckBox, QListWidget, QListWidgetItem, QAbstractItemView,
    QSizePolicy, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QObject

from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, PrintQueue,
    build_well_plate_job, save_print_job, export_gcode,
)
from SupportClasses.PhysicalModels import WorkspaceConfig
from gui.styles import COLORS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Thread → GUI Signal Bridge
# ═══════════════════════════════════════════════════════════════════

class PrintSignalBridge(QObject):
    """Thread-safe bridge for PrintManager → GUI signals."""
    progress_signal = Signal(int, int, str)        # step, total, message
    state_signal = Signal(object)                  # PrintState enum
    queue_progress_signal = Signal(int, int, str)  # job_idx, total, name
    queue_completed_signal = Signal()


# ═══════════════════════════════════════════════════════════════════
# Print Setup Page — v7.1 3-Tab Wrapper
# ═══════════════════════════════════════════════════════════════════

class PrintSetupPage(QWidget):
    """
    Print setup: 3-tab workspace workflow + execution controls.

    Tab 1 — Workspace:     Physical hardware config (needle, syringes, inks)
    Tab 2 — Print Objects:  Design objects, import CSV, build collections
    Tab 3 — Well Setup:    Assign prints to wells, calibrate plane, set roles

    Context panel provides execution controls, print queue, and settings.
    """

    # Emitted when workspace config changes (for app.py to forward to monitor)
    workspace_updated = Signal(object)  # WorkspaceConfig

    def __init__(self, controller: StageController, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self.print_manager = PrintManager(controller)
        self.print_queue = PrintQueue(controller)

        # Signal bridge for thread → GUI communication
        self._bridge = PrintSignalBridge()
        self._bridge.progress_signal.connect(self._on_progress)
        self._bridge.state_signal.connect(self._on_state_changed)
        self._bridge.queue_progress_signal.connect(self._on_queue_progress)
        self._bridge.queue_completed_signal.connect(self._on_queue_completed)

        self.print_manager.on_progress = (
            lambda s, t, m: self._bridge.progress_signal.emit(s, t, m))
        self.print_manager.on_state_changed = (
            lambda st: self._bridge.state_signal.emit(st))
        self.print_queue.on_progress = (
            lambda s, t, m: self._bridge.progress_signal.emit(s, t, m))
        self.print_queue.on_state_changed = (
            lambda st: self._bridge.state_signal.emit(st))
        self.print_queue.on_queue_progress = (
            lambda i, t, n: self._bridge.queue_progress_signal.emit(i, t, n))
        self.print_queue.on_queue_completed = (
            lambda: self._bridge.queue_completed_signal.emit())

        # Workspace config (shared between tabs)
        self._workspace = WorkspaceConfig()

        self._context_widget = None
        self._microsteps_per_micron = 10.0  # Default, updated by MainWindow
        self._setup_ui()

    # ════════════════════════════════════════════════════════════════
    #  PAGE INTERFACE
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Print Setup"

    def set_microsteps_per_micron(self, value: float):
        """Update the microsteps-per-micron conversion factor."""
        self._microsteps_per_micron = value

    def get_page_subtitle(self) -> str:
        return "Configure workspace, design objects, assign wells"

    def on_status_update(self):
        """Called by MainWindow timer (~300 ms)."""
        # Forward to active tab if it has an update method
        idx = self.tabs.currentIndex()
        current = self.tabs.currentWidget()
        if hasattr(current, 'on_status_update'):
            current.on_status_update()

    def get_context_widget(self) -> QWidget:
        """Build execution controls + queue context panel."""
        if self._context_widget:
            return self._context_widget
        return self._build_context_panel()

    def resume_print(self, resume_data: dict):
        """Resume a previously interrupted print."""
        job = resume_data["job"]
        step = resume_data["current_step"]
        self.print_manager.start(job, resume_from_step=step)

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # ── 3-Tab Workflow ────────────────────────────────────────
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: none;
                background: {COLORS['base']};
            }}
            QTabBar::tab {{
                background: {COLORS['surface0']};
                color: {COLORS['subtext0']};
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                font-weight: bold;
            }}
            QTabBar::tab:selected {{
                background: {COLORS['surface1']};
                color: {COLORS['text']};
            }}
            QTabBar::tab:hover:!selected {{
                background: {COLORS['surface0']};
                color: {COLORS['text']};
            }}
        """)

        # Import tab widgets (lazy to avoid circular imports at module level)
        from gui.pages.print_workspace import WorkspaceTab
        from gui.pages.print_objects import PrintObjectsTab
        from gui.pages.print_well_setup import WellSetupTab

        # Create tabs
        self.tab_workspace = WorkspaceTab(
            controller=self.controller,
            settings=self.settings,
        )
        self.tab_objects = PrintObjectsTab(
            controller=self.controller,
            settings=self.settings,
        )
        self.tab_wells = WellSetupTab(
            controller=self.controller,
            settings=self.settings,
            workspace=self._workspace,
        )

        self.tabs.addTab(self.tab_workspace, "1. Workspace")
        self.tabs.addTab(self.tab_objects, "2. Print Objects")
        self.tabs.addTab(self.tab_wells, "3. Well Setup")

        # ── Wire cross-tab signals ────────────────────────────────

        # Workspace → Objects + Wells + Monitor
        self.tab_workspace.workspace_changed.connect(self._on_workspace_changed)

        # Objects → Wells (available print collections)
        if hasattr(self.tab_objects, 'collections_changed'):
            self.tab_objects.collections_changed.connect(
                self._on_collections_changed)

        # Wells → execution readiness
        if hasattr(self.tab_wells, 'setup_changed'):
            self.tab_wells.setup_changed.connect(self._on_setup_changed)

        outer.addWidget(self.tabs)

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL — Execution Controls + Queue
    # ════════════════════════════════════════════════════════════════

    def _build_context_panel(self) -> QWidget:
        ctx = QWidget()
        ctx.setObjectName("contextPanel")
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Print Settings ────────────────────────────────────────
        settings_label = QLabel("Print Settings")
        settings_label.setObjectName("contextSectionLabel")
        settings_label.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 13px;")
        layout.addWidget(settings_label)

        # Feedrate
        fr_row = QHBoxLayout()
        fr_row.addWidget(QLabel("XY Feed:"))
        self.xy_feed_spin = QDoubleSpinBox()
        self.xy_feed_spin.setRange(0.1, 50.0)
        self.xy_feed_spin.setValue(5.0)
        self.xy_feed_spin.setSuffix(" mm/s")
        self.xy_feed_spin.setDecimals(1)
        fr_row.addWidget(self.xy_feed_spin)
        layout.addLayout(fr_row)

        zf_row = QHBoxLayout()
        zf_row.addWidget(QLabel("Z Feed:"))
        self.z_feed_spin = QDoubleSpinBox()
        self.z_feed_spin.setRange(0.01, 10.0)
        self.z_feed_spin.setValue(1.0)
        self.z_feed_spin.setSuffix(" mm/s")
        self.z_feed_spin.setDecimals(2)
        zf_row.addWidget(self.z_feed_spin)
        layout.addLayout(zf_row)

        pf_row = QHBoxLayout()
        pf_row.addWidget(QLabel("Pump Feed:"))
        self.pump_feed_spin = QDoubleSpinBox()
        self.pump_feed_spin.setRange(0.001, 5.0)
        self.pump_feed_spin.setValue(0.1)
        self.pump_feed_spin.setSuffix(" mm/s")
        self.pump_feed_spin.setDecimals(3)
        pf_row.addWidget(self.pump_feed_spin)
        layout.addLayout(pf_row)

        # Layers
        ly_row = QHBoxLayout()
        ly_row.addWidget(QLabel("Layers:"))
        self.layers_spin = QSpinBox()
        self.layers_spin.setRange(1, 100)
        self.layers_spin.setValue(1)
        ly_row.addWidget(self.layers_spin)
        layout.addLayout(ly_row)

        lh_row = QHBoxLayout()
        lh_row.addWidget(QLabel("Layer H:"))
        self.layer_height_spin = QDoubleSpinBox()
        self.layer_height_spin.setRange(0.01, 5.0)
        self.layer_height_spin.setValue(0.2)
        self.layer_height_spin.setSuffix(" mm")
        self.layer_height_spin.setDecimals(2)
        lh_row.addWidget(self.layer_height_spin)
        layout.addLayout(lh_row)

        # Active pump
        pump_row = QHBoxLayout()
        pump_row.addWidget(QLabel("Pump:"))
        self.pump_combo = QComboBox()
        self.pump_combo.addItems(["P1", "P2", "P3"])
        pump_row.addWidget(self.pump_combo)
        layout.addLayout(pump_row)

        # Flow rate
        flow_row = QHBoxLayout()
        flow_row.addWidget(QLabel("Flow:"))
        self.flow_spin = QDoubleSpinBox()
        self.flow_spin.setRange(0.0, 100.0)
        self.flow_spin.setValue(1.0)
        self.flow_spin.setSuffix(" µL/s")
        self.flow_spin.setDecimals(2)
        flow_row.addWidget(self.flow_spin)
        layout.addLayout(flow_row)

        # Retract / Prime
        ret_row = QHBoxLayout()
        ret_row.addWidget(QLabel("Retract:"))
        self.retract_spin = QDoubleSpinBox()
        self.retract_spin.setRange(0.0, 50.0)
        self.retract_spin.setValue(2.0)
        self.retract_spin.setSuffix(" µL")
        ret_row.addWidget(self.retract_spin)
        layout.addLayout(ret_row)

        prime_row = QHBoxLayout()
        prime_row.addWidget(QLabel("Prime:"))
        self.prime_spin = QDoubleSpinBox()
        self.prime_spin.setRange(0.0, 50.0)
        self.prime_spin.setValue(2.0)
        self.prime_spin.setSuffix(" µL")
        prime_row.addWidget(self.prime_spin)
        layout.addLayout(prime_row)

        # ── Separator ─────────────────────────────────────────────
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.HLine)
        sep1.setStyleSheet(f"background: {COLORS['surface1']};")
        layout.addWidget(sep1)

        # ── Execution Controls ────────────────────────────────────
        exec_label = QLabel("Execution")
        exec_label.setObjectName("contextSectionLabel")
        exec_label.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 13px;")
        layout.addWidget(exec_label)

        self.btn_start = QPushButton("▶ Start Print")
        self.btn_start.setObjectName("accentBtn")
        self.btn_start.setMaximumHeight(32)
        self.btn_start.clicked.connect(self._start_print)
        layout.addWidget(self.btn_start)

        btn_row = QHBoxLayout()
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setMaximumHeight(28)
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self._pause_print)
        btn_row.addWidget(self.btn_pause)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setMaximumHeight(28)
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self._abort_print)
        btn_row.addWidget(self.btn_abort)
        layout.addLayout(btn_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumHeight(16)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        self.status_label = QLabel("Idle")
        self.status_label.setObjectName("dimLabel")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        # ── Export ────────────────────────────────────────────────
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet(f"background: {COLORS['surface1']};")
        layout.addWidget(sep2)

        export_row = QHBoxLayout()
        btn_save_json = QPushButton("💾 Save Job")
        btn_save_json.setMaximumHeight(24)
        btn_save_json.clicked.connect(self._save_job)
        export_row.addWidget(btn_save_json)

        btn_export_gc = QPushButton("📄 Export G-code")
        btn_export_gc.setMaximumHeight(24)
        btn_export_gc.clicked.connect(self._export_gcode)
        export_row.addWidget(btn_export_gc)
        layout.addLayout(export_row)

        # ── Print Queue ───────────────────────────────────────────
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.Shape.HLine)
        sep3.setStyleSheet(f"background: {COLORS['surface1']};")
        layout.addWidget(sep3)

        q_lbl = QLabel("Print Queue")
        q_lbl.setObjectName("contextSectionLabel")
        q_lbl.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 13px;")
        layout.addWidget(q_lbl)

        self.queue_list = QListWidget()
        self.queue_list.setMaximumHeight(80)
        self.queue_list.setDragDropMode(
            QAbstractItemView.DragDropMode.InternalMove)
        layout.addWidget(self.queue_list)

        q_btn_row = QHBoxLayout()
        q_btn_row.setSpacing(3)
        btn_add = QPushButton("+")
        btn_add.setToolTip("Add current job to queue")
        btn_add.setMaximumHeight(24)
        btn_add.clicked.connect(self._add_to_queue)
        q_btn_row.addWidget(btn_add)
        btn_rm = QPushButton("−")
        btn_rm.setToolTip("Remove selected")
        btn_rm.setMaximumHeight(24)
        btn_rm.clicked.connect(self._remove_from_queue)
        q_btn_row.addWidget(btn_rm)
        btn_clr = QPushButton("Clear")
        btn_clr.setMaximumHeight(24)
        btn_clr.clicked.connect(self._clear_queue)
        q_btn_row.addWidget(btn_clr)
        layout.addLayout(q_btn_row)

        btn_start_q = QPushButton("▶ Start Queue")
        btn_start_q.setObjectName("accentBtn")
        btn_start_q.setMaximumHeight(28)
        btn_start_q.clicked.connect(self._start_queue)
        layout.addWidget(btn_start_q)

        self.queue_progress_label = QLabel("")
        self.queue_progress_label.setObjectName("dimLabel")
        layout.addWidget(self.queue_progress_label)

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  CROSS-TAB SIGNAL HANDLERS
    # ════════════════════════════════════════════════════════════════

    def _on_workspace_changed(self, workspace: WorkspaceConfig):
        """Workspace tab config changed → propagate to other tabs + monitor."""
        self._workspace = workspace

        # Forward to Print Objects tab
        if hasattr(self.tab_objects, 'set_workspace'):
            self.tab_objects.set_workspace(workspace)

        # Forward to Well Setup tab
        if hasattr(self.tab_wells, 'set_workspace'):
            self.tab_wells.set_workspace(workspace)

        # Forward to Print Monitor (via app.py signal)
        self.workspace_updated.emit(workspace)

        logger.debug("Workspace config propagated to all tabs")

    def _on_collections_changed(self, collection_names: list[str]):
        """Print Objects tab changed collections → update Well Setup tab."""
        if hasattr(self.tab_wells, 'set_available_prints'):
            self.tab_wells.set_available_prints(collection_names)

    def _on_setup_changed(self):
        """Well Setup tab changed → may affect execution readiness."""
        logger.debug("Well setup changed — checking execution readiness")

    # ════════════════════════════════════════════════════════════════
    #  SETTINGS HELPERS
    # ════════════════════════════════════════════════════════════════

    def _get_settings(self) -> PrintSettings:
        """Read current settings from context panel into PrintSettings."""
        return PrintSettings(
            feedrate_xy=self.xy_feed_spin.value(),
            feedrate_z=self.z_feed_spin.value(),
            feedrate_pump=self.pump_feed_spin.value(),
            num_layers=self.layers_spin.value(),
            layer_height=self.layer_height_spin.value(),
            active_pump=self.pump_combo.currentText(),
            flow_rate=self.flow_spin.value(),
            retract_volume=self.retract_spin.value(),
            prime_volume=self.prime_spin.value(),
        )

    # ════════════════════════════════════════════════════════════════
    #  EXECUTION CONTROLS
    # ════════════════════════════════════════════════════════════════

    def _build_current_job(self):
        """
        Build a PrintJob from current workspace + well setup state.

        Uses TrajectoryPlanner if available, falls back to legacy
        well plate job builder otherwise.
        """
        try:
            from SupportClasses.TrajectoryPlanner import TrajectoryPlanner
            from SupportClasses.GeometryEngine import PrintCollection

            # Get collections from Tab 2
            collections = {}
            if hasattr(self.tab_objects, 'get_collections'):
                collections = self.tab_objects.get_collections()

            # Get well assignments from Tab 3
            well_setup = None
            if hasattr(self.tab_wells, 'model'):
                well_setup = self.tab_wells.model

            if well_setup and collections:
                # v7.1 trajectory-based job
                planner = TrajectoryPlanner(self._workspace)
                settings = self._get_settings()

                # Build job using trajectory planner + well assignments
                from SupportClasses.PrintManager import PrintJob, CommandType
                job = PrintJob(
                    name=f"Workspace Print",
                    workspace=self._workspace,
                    well_setup=well_setup.to_dict() if hasattr(well_setup, 'to_dict') else {},
                    settings=settings,
                )

                # Plan trajectories for each assigned well
                for name, assignment in well_setup.assignments.items():
                    if assignment.role.value == "print" and assignment.print_collections:
                        for coll_name in assignment.print_collections:
                            if coll_name in collections:
                                coll = collections[coll_name]
                                well_pos = well_setup.plate.get_well_center_mm(name)
                                z_off = assignment.get_effective_z()
                                wps = planner.plan_well_print(
                                    coll, well_pos, z_off)
                                if wps:
                                    job.trajectory_waypoints.extend(wps)

                if job.trajectory_waypoints:
                    job.total_steps = len(job.trajectory_waypoints)
                    return job

        except (ImportError, AttributeError, Exception) as e:
            logger.debug(f"Trajectory-based job build failed: {e}")

        # Fallback: legacy well plate job (backward compat)
        return self._build_legacy_job()

    def _build_legacy_job(self):
        """Build a legacy job from well plate settings (v7.0 compat)."""
        # Check if Tab 3 has a well plate configured
        if hasattr(self.tab_wells, '_model') and self.tab_wells._model:
            model = self.tab_wells._model
            plate = model.plate
            settings = self._get_settings()

            # Get print wells
            print_wells = [
                name for name, a in model.assignments.items()
                if a.role.value == "print"
            ]

            if print_wells and plate:
                return build_well_plate_job(
                    plate=plate,
                    selected_wells=print_wells,
                    settings=settings,
                )

        logger.warning("No printable job could be built from current setup")
        return None

    def _start_print(self):
        """Start print execution."""
        job = self._build_current_job()
        if job is None:
            self.status_label.setText("No job to print — configure wells first")
            return

        self.print_manager.start(job)
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_abort.setEnabled(True)

    def _pause_print(self):
        state = self.print_manager.state
        if state == PrintState.RUNNING:
            self.print_manager.pause()
            self.btn_pause.setText("▶ Resume")
        elif state == PrintState.PAUSED:
            self.print_manager.resume()
            self.btn_pause.setText("⏸ Pause")

    def _abort_print(self):
        reply = QMessageBox.question(
            self, "Abort Print",
            "Are you sure you want to abort the current print?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.print_manager.abort()

    # ════════════════════════════════════════════════════════════════
    #  PROGRESS / STATE CALLBACKS
    # ════════════════════════════════════════════════════════════════

    def _on_progress(self, step: int, total: int, message: str):
        if total > 0:
            self.progress_bar.setMaximum(total)
            self.progress_bar.setValue(step)
        self.status_label.setText(message)

    def _on_state_changed(self, state):
        if state == PrintState.IDLE:
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_abort.setEnabled(False)
            self.btn_pause.setText("⏸ Pause")
            self.status_label.setText("Idle")
        elif state == PrintState.COMPLETED:
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_abort.setEnabled(False)
            self.status_label.setText("Print completed!")
            self.progress_bar.setValue(self.progress_bar.maximum())
        elif state == PrintState.ERROR:
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_abort.setEnabled(False)
            self.status_label.setText("Print error — check log")
        elif state == PrintState.ABORTED:
            self.btn_start.setEnabled(True)
            self.btn_pause.setEnabled(False)
            self.btn_abort.setEnabled(False)
            self.status_label.setText("Print aborted")

    # ════════════════════════════════════════════════════════════════
    #  EXPORT
    # ════════════════════════════════════════════════════════════════

    def _save_job(self):
        job = self._build_current_job()
        if job is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Print Job", "", "JSON Files (*.json)")
        if path:
            save_print_job(job, path)
            self.status_label.setText(f"Job saved: {path}")

    def _export_gcode(self):
        job = self._build_current_job()
        if job is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export G-code", "", "G-code Files (*.gcode)")
        if path:
            export_gcode(job, path)
            self.status_label.setText(f"G-code exported: {path}")

    # ════════════════════════════════════════════════════════════════
    #  PRINT QUEUE
    # ════════════════════════════════════════════════════════════════

    def _add_to_queue(self):
        job = self._build_current_job()
        if job is None:
            return
        self.print_queue.add(job)
        item = QListWidgetItem(f"{job.name} ({job.total_steps} steps)")
        self.queue_list.addItem(item)

    def _remove_from_queue(self):
        row = self.queue_list.currentRow()
        if row >= 0:
            self.queue_list.takeItem(row)
            self.print_queue.remove(row)

    def _clear_queue(self):
        self.queue_list.clear()
        self.print_queue.clear()

    def _start_queue(self):
        if self.print_queue.size == 0:
            return
        self.print_queue.start()
        self.btn_start.setEnabled(False)

    def _on_queue_progress(self, job_idx: int, total: int, name: str):
        self.queue_progress_label.setText(
            f"Queue: {job_idx + 1}/{total} — {name}")

    def _on_queue_completed(self):
        self.queue_progress_label.setText("Queue completed!")
        self.btn_start.setEnabled(True)
