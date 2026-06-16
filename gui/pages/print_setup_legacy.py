"""
Print Setup Page — v7.2.9 Tabbed Workspace with Finalize Tab.

v7.2.9 Changes:
    - Tab 4 (Finalize) added: print settings, plan of action, generate & send
    - Context panel simplified to display style options only
    - Plan of Action removed from Well Setup tab (now in Finalize tab)

PyDracula layout:
    Main content  = 4-tab workflow (Workspace / Print Objects / Well Setup / Finalize)
    Context panel = Display style options only

Interface contract:
    get_page_title()     → str
    get_context_widget() → QWidget
    on_status_update()   → called by MainWindow timer
    set_hardware_config  → v7.2: receives HardwareConfig from app.py
    resume_print(data)   → called from MainWindow resume dialog
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QProgressBar, QFrame,
    QCheckBox, QGroupBox, QSizePolicy, QMessageBox, QSlider,
    QScrollArea,
)
from PySide6.QtCore import Qt, Signal, QObject

from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, PrintQueue,
    build_well_plate_job, save_print_job, export_gcode,
)
from SupportClasses.PhysicalModels import WorkspaceConfig
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s as _sc, sf as _sf, sp as _sp, scaled_font_size

try:
    from SupportClasses.HardwareConfig import HardwareConfig
except ImportError:
    HardwareConfig = None

from SupportClasses.PrintPlanOfAction import (
    PrintExecutionConfig,
    InkSwapStrategy,
    InkGatherConfig,
    ZTravelConfig,
    XYTravelConfig,
    FinalCleanupConfig,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Thread → GUI Signal Bridge  (kept for S3 transition)
# ═══════════════════════════════════════════════════════════════════

class PrintSignalBridge(QObject):
    """Thread-safe bridge for PrintManager → GUI signals."""
    progress_signal = Signal(int, int, str)        # step, total, message
    state_signal = Signal(object)                  # PrintState enum
    queue_progress_signal = Signal(int, int, str)  # job_idx, total, name
    queue_completed_signal = Signal()


# ═══════════════════════════════════════════════════════════════════
# Print Setup Page — v7.2.3
# ═══════════════════════════════════════════════════════════════════

class PrintSetupPage(QWidget):
    """
    Print setup: 4-tab workflow + display options context panel.

    v7.2.9:
        Tab 1 — Workspace:    READ-ONLY hardware summary + WorkspaceConfig bridge
        Tab 2 — Print Objects: Design objects, import CSV, build collections
        Tab 3 — Well Setup:   Assign prints to wells, calibrate plane, set roles
        Tab 4 — Finalize:     Print settings, plan of action, generate & send

    Context panel provides display style options for the print setup page.
    """

    # Emitted when workspace config changes (for app.py to forward to monitor)
    workspace_updated = Signal(object)  # WorkspaceConfig

    # v7.2.3: Request navigation to a specific page (e.g. Hardware Setup)
    navigate_to_page = Signal(int)  # page index

    # v7.2.3: Signal that a print job is ready to send to monitor
    job_ready = Signal(object)  # PrintJob

    def __init__(self, controller: StageController, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # v7.2.3: PrintManager/PrintQueue still owned here until S3
        self.print_manager = PrintManager(controller)
        self.print_queue = PrintQueue(controller)

        # Signal bridge for thread → GUI
        self._bridge = PrintSignalBridge()
        self._bridge.progress_signal.connect(self._on_progress)
        self._bridge.state_signal.connect(self._on_state_changed)

        self.print_manager.on_progress = (
            lambda s, t, m: self._bridge.progress_signal.emit(s, t, m))
        self.print_manager.on_state_changed = (
            lambda st: self._bridge.state_signal.emit(st))

        # Workspace config (shared between tabs)
        self._workspace = WorkspaceConfig()

        self._context_widget = None
        from gui.unit_helpers import DEFAULT_XY_POSITION_SCALE
        self._xy_position_scale = DEFAULT_XY_POSITION_SCALE
        self._hardware_config = None
        self._setup_ui()

    # ════════════════════════════════════════════════════════════════
    #  PAGE INTERFACE
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Print Setup"

    def get_page_subtitle(self) -> str:
        return "Configure workspace, design objects, assign wells"

    def set_xy_position_scale(self, value: float):
        self._xy_position_scale = value

    def set_hardware_config(self, config):
        """
        v7.2.3: Set hardware config — forward to all tabs.

        # v7.2.4: Forward to Well Setup tab for ink/rosette refresh
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'set_hardware_config'):
            self.tab_wells.set_hardware_config(config)
        # v7.2.4: Forward to Print Objects tab for ink refresh
        if hasattr(self, 'tab_objects') and hasattr(self.tab_objects, 'set_hardware_config'):
            self.tab_objects.set_hardware_config(config)

        The workspace tab receives the config and builds a WorkspaceConfig
        from it via the bridge method, then emits workspace_changed.
        """

        # v7.2.6: Forward to ALL sub-tabs with error handling
        self._hw_config = config
        self._hardware_config = config
        # Recalculate derived parameters with new hardware
        if hasattr(self, '_calc_labels'):
            self._update_derived()
        for tab_name in ['tab_workspace', 'tab_objects', 'tab_wells']:
            tab = getattr(self, tab_name, None)
            if tab and hasattr(tab, 'set_hardware_config'):
                try:
                    tab.set_hardware_config(config)
                except Exception as e:
                    logger.error(f"HW config forward to {tab_name} failed: {e}")

    def on_status_update(self):
        """Called by MainWindow timer (~300 ms)."""
        idx = self.tabs.currentIndex()
        current = self.tabs.currentWidget()
        if hasattr(current, 'on_status_update'):
            current.on_status_update()

    def get_context_widget(self) -> QWidget:
        """Build print settings context panel (v7.2.3: no execution controls)."""
        if self._context_widget:
            return self._context_widget
        return self._build_context_panel()

    def resume_print(self, resume_data: dict):
        """Resume a previously interrupted print (kept for S3 transition)."""
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
                padding: {_sc(8)}px {_sc(16)}px;
                margin-right: {_sc(2)}px;
                border-top-left-radius: {_sc(6)}px;
                border-top-right-radius: {_sc(6)}px;
                font: bold {_sf(10)}pt "Segoe UI", "Apple Color Emoji", sans-serif;
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

        # Import tab widgets (lazy to avoid circular imports)
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

        # Tab 4: Finalize — print settings, plan of action, generate & send
        self.tab_finalize = self._build_finalize_tab()
        self.tabs.addTab(self.tab_finalize, "4. Finalize")

        self._generated_job = None  # v7.2.6: pre-generated job

        # ── Wire cross-tab signals ────────────────────────────────

        # Workspace → Objects + Wells + Monitor
        self.tab_workspace.workspace_changed.connect(self._on_workspace_changed)

        # v7.2.3: Workspace "Edit Hardware Setup" → navigate to Page 0
        if hasattr(self.tab_workspace, 'navigate_to_page'):
            self.tab_workspace.navigate_to_page.connect(
                self.navigate_to_page.emit)

        # Objects → Wells (available print collections)
        if hasattr(self.tab_objects, 'collections_changed'):
            self.tab_objects.collections_changed.connect(
                self._on_collections_changed)

        # v7.2.5: Also connect prints_changed (primary signal)
        if hasattr(self.tab_objects, 'prints_changed'):
            self.tab_objects.prints_changed.connect(
                self._on_collections_changed)

        # Wells → execution readiness
        if hasattr(self.tab_wells, 'setup_changed'):
            self.tab_wells.setup_changed.connect(self._on_setup_changed)

        outer.addWidget(self.tabs)

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL — Display Style Options (v7.2.9)
    # ════════════════════════════════════════════════════════════════

    def _build_context_panel(self) -> QWidget:
        """Build display style options for the print setup page."""
        ctx = QWidget()
        ctx.setObjectName("contextPanel")
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        grp_style = f"""
            QGroupBox {{
                font-weight: bold; font-size: {_sf(11)}pt;
                color: {COLORS.get('text', '#cdd6f4')};
                background: {COLORS.get('base', '#1e1e2e')};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: {_sp(4)}; margin-top: {_sp(10)}; padding-top: {_sp(24)};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 0px; right: 0px; top: 0px;
                padding: {_sp(6)} {_sp(10)};
                background: {COLORS.get('surface1', '#45475a')};
                border-top-left-radius: {_sp(4)};
                border-top-right-radius: {_sp(4)};
            }}
        """

        title = QLabel("Display Options")
        title.setStyleSheet(
            f"font-weight: bold; font-size: {_sf(12)}pt; "
            f"color: {COLORS.get('peach', '#fab387')}; "
            f"padding: {_sp(4)} 0px;")
        layout.addWidget(title)

        # ── Well Plate Display ────────────────────────────────────
        plate_grp = QGroupBox("Well Plate")
        plate_grp.setStyleSheet(grp_style)
        plate_lay = QVBoxLayout(plate_grp)
        plate_lay.setSpacing(4)

        self._show_well_labels_cb = QCheckBox("Show well labels")
        self._show_well_labels_cb.setChecked(True)
        plate_lay.addWidget(self._show_well_labels_cb)

        self._show_trajectory_cb = QCheckBox("Show trajectory paths")
        self._show_trajectory_cb.setChecked(True)
        plate_lay.addWidget(self._show_trajectory_cb)

        color_row = QHBoxLayout()
        color_row.addWidget(QLabel("Color by:"))
        self._color_mode_combo = QComboBox()
        self._color_mode_combo.addItems(["Role", "Ink", "Status"])
        color_row.addWidget(self._color_mode_combo)
        plate_lay.addLayout(color_row)

        layout.addWidget(plate_grp)

        # ── Object Preview ────────────────────────────────────────
        obj_grp = QGroupBox("Object Preview")
        obj_grp.setStyleSheet(grp_style)
        obj_lay = QVBoxLayout(obj_grp)
        obj_lay.setSpacing(4)

        self._show_grid_cb = QCheckBox("Show grid")
        self._show_grid_cb.setChecked(True)
        obj_lay.addWidget(self._show_grid_cb)

        self._show_axes_cb = QCheckBox("Show axes")
        self._show_axes_cb.setChecked(True)
        obj_lay.addWidget(self._show_axes_cb)

        self._show_dimensions_cb = QCheckBox("Show dimensions")
        self._show_dimensions_cb.setChecked(False)
        obj_lay.addWidget(self._show_dimensions_cb)

        layout.addWidget(obj_grp)

        layout.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  FINALIZE TAB — Print Settings + Plan of Action (v7.2.9)
    # ════════════════════════════════════════════════════════════════

    # ── Gauge max flow lookup (µL/s) ─────────────────────────────
    GAUGE_MAX_FLOW = {
        16: 50.0, 18: 30.0, 20: 15.0, 22: 8.0, 23: 5.0,
        25: 3.0, 27: 1.5, 28: 1.0, 30: 0.5, 32: 0.2,
    }

    def _build_finalize_tab(self) -> QWidget:
        """Tab 4: Finalize — print settings, plan of action, generate & send."""
        import math

        wrapper = QWidget()
        # v7.6.0: expand to fill the full height of the left-context
        # Tools column (the form carries its own inner scroll area, so
        # the column should not leave empty space beneath it).
        wrapper.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        wrapper_layout = QVBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"background: {COLORS.get('base', '#1e1e2e')};")

        content = QWidget()
        outer = QVBoxLayout(content)
        outer.setContentsMargins(16, 12, 16, 12)
        outer.setSpacing(8)

        grp_style = f"""
            QGroupBox {{
                font-weight: bold; font-size: {_sf(11)}pt;
                color: {COLORS.get('text', '#cdd6f4')};
                background: {COLORS.get('base', '#1e1e2e')};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: {_sp(4)}; margin-top: {_sp(10)}; padding-top: {_sp(24)};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 0px; right: 0px; top: 0px;
                padding: {_sp(6)} {_sp(10)};
                background: {COLORS.get('surface1', '#45475a')};
                border-top-left-radius: {_sp(4)};
                border-top-right-radius: {_sp(4)};
            }}
        """
        dim_style = f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: {_sf(9)}pt;"

        # ── Two-column layout ─────────────────────────────────────
        columns = QHBoxLayout()
        columns.setSpacing(16)
        left_col = QVBoxLayout()
        left_col.setSpacing(6)
        right_col = QVBoxLayout()
        right_col.setSpacing(6)

        # ══════════════════════════════════════════════════════════
        #  LEFT COLUMN: Print Parameters
        # ══════════════════════════════════════════════════════════

        left_title = QLabel("Print Parameters")
        left_title.setStyleSheet(
            f"font-weight: bold; font-size: {_sf(12)}pt; "
            f"color: {COLORS.get('blue', '#89b4fa')}; "
            f"padding: {_sp(2)} 0px;")
        left_col.addWidget(left_title)

        # ── Extrusion ─────────────────────────────────────────────
        ext_grp = QGroupBox("Extrusion")
        ext_grp.setStyleSheet(grp_style)
        ext_lay = QVBoxLayout(ext_grp)
        ext_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        ext_lay.setSpacing(_sc(7))

        vf_row = QHBoxLayout()
        vf_row.addWidget(QLabel("Fill:"))
        self.volume_fraction_spin = QSpinBox()
        self.volume_fraction_spin.setRange(1, 100)
        self.volume_fraction_spin.setValue(50)
        self.volume_fraction_spin.setSuffix(" %")
        self.volume_fraction_spin.setToolTip(
            "Volume fraction of needle bore cylinder to fill (0-100%)")
        self.volume_fraction_spin.valueChanged.connect(self._update_derived)
        vf_row.addWidget(self.volume_fraction_spin)
        ext_lay.addLayout(vf_row)

        sp_row = QHBoxLayout()
        sp_row.addWidget(QLabel("Speed:"))
        self.speed_scale_spin = QSpinBox()
        self.speed_scale_spin.setRange(1, 100)
        self.speed_scale_spin.setValue(50)
        self.speed_scale_spin.setSuffix(" %")
        self.speed_scale_spin.setToolTip(
            "Percentage of maximum print speed (higher = faster but less accurate)")
        self.speed_scale_spin.valueChanged.connect(self._update_derived)
        sp_row.addWidget(self.speed_scale_spin)
        ext_lay.addLayout(sp_row)

        left_col.addWidget(ext_grp)

        # ── Calculated (derived read-only) ────────────────────────
        calc_grp = QGroupBox("Calculated")
        calc_grp.setStyleSheet(grp_style)
        calc_lay = QVBoxLayout(calc_grp)
        calc_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        calc_lay.setSpacing(_sc(4))

        self._calc_labels = {}
        for key, label_text in [
            ("needle_bore", "Bore:"),
            ("max_flow", "Max Flow:"),
            ("max_speed", "Max Speed:"),
            ("print_speed", "Print Speed:"),
            ("flow_rate", "Flow Rate:"),
            ("pump_speed", "Pump Speed:"),
        ]:
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setFixedWidth(_sc(80))
            row.addWidget(lbl)
            val = QLabel("—")
            val.setStyleSheet(dim_style)
            row.addWidget(val)
            calc_lay.addLayout(row)
            self._calc_labels[key] = val

        left_col.addWidget(calc_grp)

        # ── Layers ────────────────────────────────────────────────
        layer_grp = QGroupBox("Layers")
        layer_grp.setStyleSheet(grp_style)
        layer_lay = QVBoxLayout(layer_grp)
        layer_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        layer_lay.setSpacing(_sc(7))

        ly_row = QHBoxLayout()
        ly_row.addWidget(QLabel("Count:"))
        self.layers_spin = QSpinBox()
        self.layers_spin.setRange(1, 100)
        self.layers_spin.setValue(1)
        ly_row.addWidget(self.layers_spin)
        layer_lay.addLayout(ly_row)

        lh_row = QHBoxLayout()
        lh_row.addWidget(QLabel("Height:"))
        self.layer_height_spin = QDoubleSpinBox()
        self.layer_height_spin.setRange(0.01, 5.0)
        self.layer_height_spin.setValue(0.2)
        self.layer_height_spin.setSuffix(" mm")
        self.layer_height_spin.setDecimals(2)
        self.layer_height_spin.valueChanged.connect(self._update_derived)
        lh_row.addWidget(self.layer_height_spin)
        layer_lay.addLayout(lh_row)

        # v7.5.x: print height is measured up from the calibrated plate
        # bottom (not an absolute Z), so it is frame-agnostic and can never be
        # set below the plate. 0 = at the plate bottom; the needle is clamped
        # so it never goes deeper than the plate bottom.
        ph_row = QHBoxLayout()
        _ph_label = QLabel("Print Z (above bottom):")
        _ph_label.setToolTip(
            "Print height measured up from the calibrated plate bottom.\n"
            "0 = at the plate bottom; larger = higher. The needle is clamped "
            "so it can never punch through the plate bottom.")
        ph_row.addWidget(_ph_label)
        self.print_height_spin = QDoubleSpinBox()
        self.print_height_spin.setRange(0.0, 40.0)
        self.print_height_spin.setValue(0.2)
        self.print_height_spin.setSuffix(" mm")
        self.print_height_spin.setDecimals(2)
        self.print_height_spin.setSingleStep(0.1)
        self.print_height_spin.setToolTip(_ph_label.toolTip())
        ph_row.addWidget(self.print_height_spin)
        layer_lay.addLayout(ph_row)

        left_col.addWidget(layer_grp)

        # ── Advanced ──────────────────────────────────────────────
        adv_grp = QGroupBox("Advanced")
        adv_grp.setStyleSheet(grp_style)
        adv_grp.setCheckable(True)
        adv_grp.setChecked(False)
        adv_lay = QVBoxLayout(adv_grp)
        adv_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        adv_lay.setSpacing(_sc(6))

        zf_row = QHBoxLayout()
        zf_row.addWidget(QLabel("Z Feed:"))
        self.z_feed_spin = QDoubleSpinBox()
        self.z_feed_spin.setRange(0.01, 10.0)
        self.z_feed_spin.setValue(1.0)
        self.z_feed_spin.setSuffix(" mm/s")
        self.z_feed_spin.setDecimals(2)
        zf_row.addWidget(self.z_feed_spin)
        adv_lay.addLayout(zf_row)

        tz_row = QHBoxLayout()
        tz_row.addWidget(QLabel("Travel Z:"))
        self.travel_z_spin = QDoubleSpinBox()
        self.travel_z_spin.setRange(0.1, 30.0)
        self.travel_z_spin.setValue(5.0)
        self.travel_z_spin.setSuffix(" mm")
        self.travel_z_spin.setDecimals(1)
        tz_row.addWidget(self.travel_z_spin)
        adv_lay.addLayout(tz_row)

        sd_row = QHBoxLayout()
        sd_row.addWidget(QLabel("Settle:"))
        self.settle_spin = QDoubleSpinBox()
        self.settle_spin.setRange(0.0, 10.0)
        self.settle_spin.setValue(0.0)
        self.settle_spin.setSuffix(" s")
        self.settle_spin.setDecimals(1)
        sd_row.addWidget(self.settle_spin)
        adv_lay.addLayout(sd_row)

        self._retract_spins: dict[str, QDoubleSpinBox] = {}
        self._prime_spins: dict[str, QDoubleSpinBox] = {}

        for pid in ["P1", "P2", "P3"]:
            row = QHBoxLayout()
            lbl = QLabel(f"{pid}:")
            lbl.setFixedWidth(_sc(24))
            lbl.setStyleSheet(
                f"font-weight: bold; color: {COLORS.get('blue', '#89b4fa')};")
            row.addWidget(lbl)

            row.addWidget(QLabel("Ret:"))
            ret_spin = QDoubleSpinBox()
            ret_spin.setRange(0.0, 20.0)
            ret_spin.setValue(0.5)
            ret_spin.setSuffix(" uL")
            ret_spin.setDecimals(2)
            ret_spin.setMaximumWidth(_sc(90))
            row.addWidget(ret_spin)
            self._retract_spins[pid] = ret_spin

            row.addWidget(QLabel("Pri:"))
            prime_spin = QDoubleSpinBox()
            prime_spin.setRange(0.0, 20.0)
            prime_spin.setValue(0.5)
            prime_spin.setSuffix(" uL")
            prime_spin.setDecimals(2)
            prime_spin.setMaximumWidth(_sc(90))
            row.addWidget(prime_spin)
            self._prime_spins[pid] = prime_spin

            adv_lay.addLayout(row)

        left_col.addWidget(adv_grp)
        # v7.6.0: no stretch here — Plan of Action stacks directly
        # beneath Print Parameters in the single-column layout.

        # ══════════════════════════════════════════════════════════
        #  RIGHT COLUMN: Plan of Action
        # ══════════════════════════════════════════════════════════

        right_title = QLabel("Plan of Action")
        right_title.setStyleSheet(
            f"font-weight: bold; font-size: {_sf(12)}pt; "
            f"color: {COLORS.get('peach', '#fab387')}; "
            f"padding: {_sp(2)} 0px;")
        right_col.addWidget(right_title)

        # ── Ink Swap Strategy ─────────────────────────────────────
        swap_grp = QGroupBox("Ink Swap Sequence")
        swap_grp.setStyleSheet(grp_style)
        swap_grp.setCheckable(True)
        swap_grp.setChecked(True)
        swap_grp.setToolTip(
            "When a pump switches between inks, these steps run.\n"
            "Sequence: waste > wash > buffer > wash > ink load > wash")
        swap_lay = QVBoxLayout(swap_grp)
        swap_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        swap_lay.setSpacing(_sc(7))

        self._swap_checks = {}
        swap_steps = [
            ("waste", "Waste (expel remaining ink)"),
            ("wash_pre", "Wash (pre-buffer rinse)"),
            ("buffer", "Buffer flush"),
            ("wash_post", "Wash (post-buffer rinse)"),
            ("ink_load", "Load new ink"),
            ("wash_final", "Wash (final, before print)"),
        ]
        for key, label in swap_steps:
            cb = QCheckBox(label)
            cb.setChecked(True)
            swap_lay.addWidget(cb)
            self._swap_checks[key] = cb

        # v7.6.0: per-step volumes in a 2-column grid below a small
        # caption. Right-aligned labels + expanding spinboxes keep the
        # pairs evenly spaced (not crowded) in the narrow column.
        from PySide6.QtWidgets import QGridLayout
        vol_caption = QLabel("Volumes (µL)")
        vol_caption.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {_sf(9)}pt; padding-top: {_sp(4)}px;")
        swap_lay.addWidget(vol_caption)

        vol_form = QGridLayout()
        vol_form.setHorizontalSpacing(_sc(10))
        vol_form.setVerticalSpacing(_sc(6))
        self._swap_waste_vol = QDoubleSpinBox()
        self._swap_wash_vol = QDoubleSpinBox()
        self._swap_buffer_vol = QDoubleSpinBox()
        self._swap_ink_load_vol = QDoubleSpinBox()
        for i, (spin, tip, default) in enumerate([
            (self._swap_waste_vol, "Waste", 50.0),
            (self._swap_wash_vol, "Wash", 100.0),
            (self._swap_buffer_vol, "Buffer", 100.0),
            (self._swap_ink_load_vol, "Ink", 50.0),
        ]):
            spin.setRange(0, 5000)
            spin.setSuffix(" µL")
            spin.setDecimals(0)
            spin.setValue(default)
            spin.setToolTip(f"{tip} volume")
            spin.setMinimumWidth(0)
            spin.setMaximumWidth(_sc(96))
            spin.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            r, c = i // 2, (i % 2) * 2
            lbl = QLabel(f"{tip}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')};")
            vol_form.addWidget(lbl, r, c)
            vol_form.addWidget(spin, r, c + 1)
        vol_form.setColumnStretch(1, 1)
        vol_form.setColumnStretch(3, 1)
        swap_lay.addLayout(vol_form)

        right_col.addWidget(swap_grp)

        # ── Ink Gathering ─────────────────────────────────────────
        gather_grp = QGroupBox("Ink Gathering")
        gather_grp.setStyleSheet(grp_style)
        gather_lay = QVBoxLayout(gather_grp)
        gather_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        gather_lay.setSpacing(_sc(7))

        extra_row = QHBoxLayout()
        extra_row.addWidget(QLabel("Extra:"))
        self._ink_extra_pct_spin = QDoubleSpinBox()
        self._ink_extra_pct_spin.setRange(0, 100)
        self._ink_extra_pct_spin.setValue(10.0)
        self._ink_extra_pct_spin.setSuffix(" %")
        self._ink_extra_pct_spin.setDecimals(0)
        self._ink_extra_pct_spin.setToolTip(
            "Extra ink percentage to pick up beyond what is needed.\n"
            "E.g. 10% = pick up 10% more than calculated need.")
        extra_row.addWidget(self._ink_extra_pct_spin)
        gather_lay.addLayout(extra_row)

        max_row = QHBoxLayout()
        max_row.addWidget(QLabel("Max:"))
        self._ink_max_pickup_spin = QDoubleSpinBox()
        self._ink_max_pickup_spin.setRange(0, 5000)
        self._ink_max_pickup_spin.setValue(0)
        self._ink_max_pickup_spin.setSuffix(" uL")
        self._ink_max_pickup_spin.setDecimals(0)
        self._ink_max_pickup_spin.setToolTip(
            "Maximum ink pickup per run (0 = use syringe capacity)")
        max_row.addWidget(self._ink_max_pickup_spin)
        gather_lay.addLayout(max_row)

        right_col.addWidget(gather_grp)

        # ── Z Travel ──────────────────────────────────────────────
        z_grp = QGroupBox("Z Travel")
        z_grp.setStyleSheet(grp_style)
        z_lay = QVBoxLayout(z_grp)
        z_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        z_lay.setSpacing(_sc(7))

        self._z_wait_confirm_cb = QCheckBox("Wait for Z position confirm")
        self._z_wait_confirm_cb.setChecked(True)
        self._z_wait_confirm_cb.setToolTip(
            "Wait for Z axis to reach target and confirm\n"
            "position before proceeding. Slower but safer.")
        z_lay.addWidget(self._z_wait_confirm_cb)

        self._z_fast_exit_cb = QCheckBox("Fast Z exit from well")
        self._z_fast_exit_cb.setChecked(True)
        self._z_fast_exit_cb.setToolTip(
            "Use fast Z speed when raising needle out of a well")
        z_lay.addWidget(self._z_fast_exit_cb)

        self._z_fast_enter_cb = QCheckBox("Fast Z entry into well")
        self._z_fast_enter_cb.setChecked(False)
        self._z_fast_enter_cb.setToolTip(
            "Use fast Z speed when lowering needle into a well.\n"
            "When disabled, needle enters slowly for safety.")
        z_lay.addWidget(self._z_fast_enter_cb)

        self._z_plunge_cb = QCheckBox("Plunge buffer before print Z")
        self._z_plunge_cb.setChecked(True)
        self._z_plunge_cb.setToolTip(
            "Travel fast to (print_z + buffer), then slowly\n"
            "cover the last buffer distance to print height.")
        z_lay.addWidget(self._z_plunge_cb)

        buf_row = QHBoxLayout()
        buf_row.addWidget(QLabel("Buffer:"))
        self._z_plunge_buffer_spin = QDoubleSpinBox()
        self._z_plunge_buffer_spin.setRange(0, 5000)
        self._z_plunge_buffer_spin.setValue(500)
        self._z_plunge_buffer_spin.setSuffix(" um")
        self._z_plunge_buffer_spin.setDecimals(0)
        self._z_plunge_buffer_spin.setToolTip(
            "Distance in microns above print Z to switch\n"
            "from fast travel to slow plunge.")
        buf_row.addWidget(self._z_plunge_buffer_spin)
        z_lay.addLayout(buf_row)

        right_col.addWidget(z_grp)

        # ── XY Travel ─────────────────────────────────────────────
        xy_grp = QGroupBox("XY Travel")
        xy_grp.setStyleSheet(grp_style)
        xy_lay = QVBoxLayout(xy_grp)
        xy_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        xy_lay.setSpacing(_sc(7))

        self._xy_fast_cb = QCheckBox("Fast XY travel between wells")
        self._xy_fast_cb.setChecked(True)
        self._xy_fast_cb.setToolTip(
            "Move XY at max speed between wells.\n"
            "Disable for slower, more controlled travel.")
        xy_lay.addWidget(self._xy_fast_cb)

        xy_spd_row = QHBoxLayout()
        xy_spd_row.addWidget(QLabel("Speed:"))
        self._xy_travel_speed_spin = QDoubleSpinBox()
        self._xy_travel_speed_spin.setRange(0.1, 50.0)
        self._xy_travel_speed_spin.setValue(10.0)
        self._xy_travel_speed_spin.setSuffix(" mm/s")
        self._xy_travel_speed_spin.setDecimals(1)
        xy_spd_row.addWidget(self._xy_travel_speed_spin)
        xy_lay.addLayout(xy_spd_row)

        right_col.addWidget(xy_grp)

        # ── Final Cleanup ─────────────────────────────────────────
        clean_grp = QGroupBox("Final Cleanup")
        clean_grp.setStyleSheet(grp_style)
        clean_grp.setCheckable(True)
        clean_grp.setChecked(True)
        clean_lay = QVBoxLayout(clean_grp)
        clean_lay.setContentsMargins(_sc(10), _sc(6), _sc(10), _sc(10))
        clean_lay.setSpacing(_sc(7))
        self._cleanup_grp = clean_grp

        self._cleanup_waste_cb = QCheckBox("Waste (eject remaining)")
        self._cleanup_waste_cb.setChecked(True)
        clean_lay.addWidget(self._cleanup_waste_cb)

        self._cleanup_wash_cb = QCheckBox("Wash needle")
        self._cleanup_wash_cb.setChecked(True)
        clean_lay.addWidget(self._cleanup_wash_cb)

        self._cleanup_dry_cb = QCheckBox("Dry run (clear residual)")
        self._cleanup_dry_cb.setChecked(False)
        clean_lay.addWidget(self._cleanup_dry_cb)

        right_col.addWidget(clean_grp)
        right_col.addStretch()

        # v7.6.0: single-column layout — Print Parameters stacked
        # above Plan of Action — so the whole form fits the narrow
        # left-context Tools column without horizontal overflow.
        # (``columns`` QHBoxLayout is left unused.)
        outer.addLayout(left_col)
        outer.addLayout(right_col)

        # ══════════════════════════════════════════════════════════
        #  Bottom: Validate, Generate & Send
        # ══════════════════════════════════════════════════════════
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color: {COLORS.get('surface1', '#45475a')};")
        outer.addWidget(sep)

        # v7.6.0: stack the two primary actions vertically (full
        # width) so they fit the narrow left-context column.
        btn_row = QVBoxLayout()
        btn_row.setSpacing(6)

        self.btn_generate_print = QPushButton("Validate & Generate Print")
        self.btn_generate_print.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS.get('blue', '#89b4fa')};
                color: {COLORS.get('base', '#1e1e2e')};
                font-weight: bold; padding: {_sp(10)} {_sp(20)};
                border-radius: {_sp(4)}; font-size: {_sf(12)}pt;
            }}
            QPushButton:hover {{
                background: {COLORS.get('sapphire', '#74c7ec')};
            }}
        """)
        self.btn_generate_print.clicked.connect(self._generate_print)
        btn_row.addWidget(self.btn_generate_print)

        self.btn_send_to_monitor = QPushButton("Send to Monitor")
        self.btn_send_to_monitor.setObjectName("accentBtn")
        self.btn_send_to_monitor.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS.get('green', '#a6e3a1')};
                color: {COLORS.get('base', '#1e1e2e')};
                font-weight: bold; padding: {_sp(10)} {_sp(20)};
                border-radius: {_sp(4)}; font-size: {_sf(12)}pt;
            }}
            QPushButton:hover {{
                background: {COLORS.get('teal', '#94e2d5')};
            }}
        """)
        self.btn_send_to_monitor.clicked.connect(self._send_to_monitor)
        btn_row.addWidget(self.btn_send_to_monitor)

        outer.addLayout(btn_row)

        # Status + export row
        status_row = QHBoxLayout()
        status_row.setSpacing(8)

        self._gen_status_label = QLabel("")
        self._gen_status_label.setStyleSheet(dim_style)
        self._gen_status_label.setWordWrap(True)
        status_row.addWidget(self._gen_status_label, stretch=1)

        self.status_label = QLabel("Configure wells > Finalize > Generate")
        self.status_label.setObjectName("dimLabel")
        self.status_label.setStyleSheet(dim_style)
        status_row.addWidget(self.status_label, stretch=1)

        outer.addLayout(status_row)

        export_row = QHBoxLayout()
        btn_export = QPushButton("Export G-code")
        btn_export.setMaximumHeight(_sc(28))
        btn_export.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn_export.clicked.connect(self._export_gcode)
        export_row.addWidget(btn_export)
        btn_save = QPushButton("Save JSON")
        btn_save.setMaximumHeight(_sc(28))
        btn_save.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn_save.clicked.connect(self._save_job)
        export_row.addWidget(btn_save)
        outer.addLayout(export_row)

        scroll.setWidget(content)
        wrapper_layout.addWidget(scroll)

        # Initial derived calculation
        self._update_derived()

        return wrapper

    def _update_derived(self):
        """Recalculate derived print parameters from volume fraction + speed scale."""
        import math

        hw = getattr(self, '_hardware_config', None)
        needle = getattr(hw, 'needle', None) if hw else None

        if needle is None:
            for v in self._calc_labels.values():
                v.setText("— (no needle)")
            return

        bore_mm = needle.id_mm
        bore_area_mm2 = math.pi * (bore_mm / 2) ** 2  # mm²

        vf = self.volume_fraction_spin.value() / 100.0  # 0-1
        speed_pct = self.speed_scale_spin.value() / 100.0  # 0-1

        # Volume per mm of travel at this fill fraction (µL/mm, since 1 mm³ = 1 µL)
        flow_per_mm = vf * bore_area_mm2  # µL/mm

        # Max flow rate for this gauge
        gauge = needle.gauge
        max_flow = self.GAUGE_MAX_FLOW.get(gauge, 5.0)  # µL/s

        # Max print speed = max_flow / flow_per_mm
        if flow_per_mm > 0:
            max_speed = max_flow / flow_per_mm  # mm/s
        else:
            max_speed = 0.0

        # Actual print speed
        print_speed = max_speed * speed_pct  # mm/s
        actual_flow = print_speed * flow_per_mm  # µL/s

        # Pump plunger speed (if syringe available)
        pump_speed_str = "—"
        if hw:
            for pid, pcfg in hw.pumps.items():
                if pcfg.is_configured and pcfg.syringe:
                    ps_mm_s = actual_flow * pcfg.syringe.mm_per_uL
                    pump_speed_str = f"{ps_mm_s:.4f} mm/s"
                    break

        self._calc_labels["needle_bore"].setText(f"{bore_mm*1000:.0f} µm ({gauge}G)")
        self._calc_labels["max_flow"].setText(f"{max_flow:.1f} µL/s")
        self._calc_labels["max_speed"].setText(f"{max_speed:.2f} mm/s")
        self._calc_labels["print_speed"].setText(f"{print_speed:.2f} mm/s")
        self._calc_labels["flow_rate"].setText(f"{actual_flow:.3f} µL/s")
        self._calc_labels["pump_speed"].setText(pump_speed_str)

    # ════════════════════════════════════════════════════════════════
    #  SETTINGS EXTRACTION
    # ════════════════════════════════════════════════════════════════

    def _compute_auto_settings(self, s, hw_config=None):
        """Auto-derive motion parameters from hardware config + calibration."""
        import math

        # Calibration data (travel Z, top Z)
        try:
            _app_settings = getattr(self, '_app_settings', None)
            if _app_settings is None:
                _p = self.parent() if hasattr(self, 'parent') else None
                while _p is not None:
                    if hasattr(_p, 'settings') and hasattr(_p.settings, 'get_section'):
                        _app_settings = _p.settings
                        break
                    _p = _p.parent() if hasattr(_p, 'parent') else None
            if _app_settings:
                cal = _app_settings.get_section('calibration') or {}
                # Fast Move Z (legacy "safe_z") → fast XY travel height.
                safe_z = cal.get('safe_z')
                if safe_z is not None and safe_z > 0:
                    s.travel_z_height = float(safe_z)
                # Plate Top Z (legacy "top_z") → top of well opening.
                top_z = cal.get('top_z')
                if top_z is not None and top_z >= 0:
                    s.top_z_height = float(top_z)
                # v7.4.4: pass through the new Z reference heights so
                # downstream consumers (safety, trajectory clamping,
                # GCode comments) can pick them up via hasattr-guarded
                # reads.
                for key in ("replace_z", "max_z", "plate_bottom_z"):
                    val = cal.get(key)
                    if val is not None:
                        setattr(s, key, float(val))
        except Exception:
            pass

        # Z speed tiers
        s.fast_z_feedrate_mm_min = 120.0
        s.entry_z_feedrate_mm_min = s.z_feedrate

        # Service XY speed
        s.service_xy_speed_mm_s = 50.0

        # Needle-derived rates
        needle = getattr(hw_config, 'needle', None) if hw_config else None
        gauge = getattr(needle, 'gauge', None) if needle else None

        max_flow = self.GAUGE_MAX_FLOW.get(gauge, 5.0) if gauge else 5.0
        s.service_pump_rate_uL_s = max_flow

        # Pump feedrate for service moves
        _uL_per_mm = 3.378
        if hw_config:
            for pid, pcfg in hw_config.pumps.items():
                if pcfg.is_configured and pcfg.syringe:
                    _uL_per_mm = pcfg.syringe.uL_per_mm
                    break
        s.pump_feedrate = max(max_flow * 60.0 / _uL_per_mm, 0.5)

        # Auto pump rate from extrusion cylinder model
        s.auto_pump_rate_uL_s = s.pump_rate_uL_s

    def _get_derived_speed_and_flow(self):
        """Calculate print speed and flow rate from volume fraction + speed scale.

        Returns (print_speed_mm_s, flow_rate_uL_s, max_speed_mm_s).
        """
        import math

        hw = getattr(self, '_hardware_config', None)
        needle = getattr(hw, 'needle', None) if hw else None
        if needle is None:
            return (1.0, 0.1, 1.0)

        bore_mm = needle.id_mm
        bore_area = math.pi * (bore_mm / 2) ** 2
        vf = self.volume_fraction_spin.value() / 100.0
        speed_pct = self.speed_scale_spin.value() / 100.0
        flow_per_mm = vf * bore_area

        max_flow = self.GAUGE_MAX_FLOW.get(needle.gauge, 5.0)
        max_speed = max_flow / flow_per_mm if flow_per_mm > 0 else 1.0
        print_speed = max_speed * speed_pct
        flow_rate = print_speed * flow_per_mm

        return (print_speed, flow_rate, max_speed)

    # ════════════════════════════════════════════════════════════════
    #  v7.5.x: plate-bottom-relative print Z
    # ════════════════════════════════════════════════════════════════

    def _find_app_settings(self):
        """Locate the app-level Settings object (walks the parent chain)."""
        _app_settings = getattr(self, '_app_settings', None)
        if _app_settings is not None:
            return _app_settings
        _p = self.parent() if hasattr(self, 'parent') else None
        while _p is not None:
            if hasattr(_p, 'settings') and hasattr(_p.settings, 'get_section'):
                return _p.settings
            _p = _p.parent() if hasattr(_p, 'parent') else None
        return None

    def _calibration_plate_bottom_z(self):
        """Calibrated plate-bottom Z (zero-ref mm), or None if uncalibrated."""
        try:
            app_settings = self._find_app_settings()
            if app_settings is None:
                return None
            cal = app_settings.get_section('calibration') or {}
            pb = cal.get('plate_bottom_z')
            return None if pb is None else float(pb)
        except Exception:
            return None

    def _print_height_above_bottom(self):
        """Desired print height above the plate bottom (mm).

        A selected object that carries ``z_above_plate_bottom_mm`` metadata
        (e.g. a Sketch print) overrides the spin so its authored height is
        honoured; otherwise the Print-Z spin value is used.
        """
        obj_h = self._object_print_height_above_bottom()
        if obj_h is not None:
            return obj_h
        spin = getattr(self, 'print_height_spin', None)
        return float(spin.value()) if spin is not None else 0.2

    def _object_print_height_above_bottom(self):
        """Scan selected print objects for a ``z_above_plate_bottom_mm`` param.

        Returns the shallowest (smallest) such height if any object carries it,
        else None. Shallowest is the safest choice when multiple objects mix.
        """
        tab = getattr(self, 'tab_objects', None)
        if tab is None:
            return None
        candidates = []

        def _scan(objs):
            for obj in objs or []:
                params = obj.get('params', {}) if isinstance(obj, dict) else {}
                if isinstance(params, dict) and 'z_above_plate_bottom_mm' in params:
                    try:
                        candidates.append(float(params['z_above_plate_bottom_mm']))
                    except (TypeError, ValueError):
                        pass

        _scan(getattr(tab, '_objects', None))
        cf = getattr(tab, '_current_file', None)
        if cf is not None and getattr(cf, 'objects', None):
            _scan(list(cf.objects.values()))
        return min(candidates) if candidates else None

    def _print_z_dir(self) -> float:
        """v7.5.x: reference-vector print-Z up-direction as a clean float.

        Falls back to the module ``ZDIR`` when the controller can't provide one
        (older controller / test stub), so callers can always pass a numeric
        ``zdir``.
        """
        from SupportClasses.StageController import ZDIR
        try:
            return float(self.controller.print_z_dir())
        except (TypeError, ValueError, AttributeError):
            return ZDIR

    def _apply_plate_relative_print_z(self, s) -> None:
        """Set ``s.print_z_height`` from the plate-bottom datum + the desired
        height above it. Leaves the default untouched if uncalibrated."""
        pb = self._calibration_plate_bottom_z()
        if pb is None:
            return
        try:
            from SupportClasses.StageController import plate_relative_to_zref
            s.print_z_height = plate_relative_to_zref(
                pb, self._print_height_above_bottom(),
                zdir=self._print_z_dir())
        except Exception as e:
            logger.debug(f"_apply_plate_relative_print_z failed: {e}")

    def _confirm_print_floor(self, job) -> bool:
        """Warn if the job's print Z (or its deepest layer) is below the plate
        bottom. Returns True to proceed, False to cancel."""
        pb = self._calibration_plate_bottom_z()
        if pb is None or job is None:
            return True
        try:
            from SupportClasses.StageController import zref_to_plate_relative
            st = job.settings
            n = max(1, int(getattr(st, 'num_layers', 1)))
            lh = float(getattr(st, 'layer_height', 0.0))
            pz = float(getattr(st, 'print_z_height', 0.0))
            up = self._print_z_dir()
            # Check both layer endpoints so the test is correct for either Z
            # polarity. Layers step by `up*lh` (see build_well_plate_job), so
            # the deepest point is the base layer; convert both with `up`.
            heights = [zref_to_plate_relative(pb, pz, zdir=up),
                       zref_to_plate_relative(pb, pz + up * (n - 1) * lh, zdir=up)]
            if min(heights) >= -1e-6:
                return True
        except Exception:
            return True
        from PySide6.QtWidgets import QMessageBox
        resp = QMessageBox.warning(
            self, "Below plate bottom",
            "The configured print height is below the calibrated plate bottom, "
            "so the needle will be clamped to the plate bottom (it will not "
            "print deeper, and multi-layer build-up may be capped). Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return resp == QMessageBox.StandardButton.Yes

    def _get_settings(self) -> PrintSettings:
        """Build PrintSettings from simplified context panel controls."""
        s = PrintSettings()

        # Derive print speed and flow from volume fraction + speed scale
        print_speed, flow_rate, _ = self._get_derived_speed_and_flow()

        s.print_speed_mm_s = print_speed
        s.travel_speed_mm_s = print_speed * 2.0
        s.xy_feedrate = print_speed
        s.print_feedrate = print_speed * 60.0  # mm/min compat

        s.z_feedrate = self.z_feed_spin.value() * 60.0  # mm/s → mm/min
        s.pump_rate_uL_s = flow_rate
        s.flow_rate = flow_rate
        s.auto_pump_rate_uL_s = flow_rate

        # Convert flow to pump feedrate
        _uL_per_mm = 3.378
        hw = getattr(self, '_hardware_config', None)
        if hw:
            for pid, pcfg in hw.pumps.items():
                if pcfg.is_configured and pcfg.syringe:
                    _uL_per_mm = pcfg.syringe.uL_per_mm
                    break
        s.pump_feedrate = max(flow_rate * 60.0 / _uL_per_mm, 0.5)

        s.num_layers = self.layers_spin.value()
        s.layer_height = self.layer_height_spin.value()
        # v7.5.x: stamp the reference-vector up-direction so layer build-up and
        # per-well dispense Z step the correct way on either Z polarity.
        s.z_up_sign = self._print_z_dir()
        s.travel_z_height = self.travel_z_spin.value()
        s.settle_delay = self.settle_spin.value()

        # Find first enabled pump as active
        s.active_pump = "P1"
        if hw:
            for pid, pcfg in hw.pumps.items():
                if pcfg.is_configured:
                    s.active_pump = pid
                    break

        # Per-pump retract/prime
        s.retract_amounts = {
            pid: spin.value() for pid, spin in self._retract_spins.items()
        }
        s.prime_amounts = {
            pid: spin.value() for pid, spin in self._prime_spins.items()
        }

        # v7.5.x: print Z is a height above the calibrated plate bottom (not an
        # absolute Z), resolved here so both job-build paths (_build_current_job
        # and the generate path) pick it up.
        self._apply_plate_relative_print_z(s)

        return s

    # ════════════════════════════════════════════════════════════════
    #  SEND TO MONITOR (v7.2.3: replaces direct execution)
    # ════════════════════════════════════════════════════════════════

    def _send_to_monitor(self):
        """v7.2.6: Prefer pre-generated job from _generate_print().

        Build a print job and emit job_ready for Print Monitor.
        Validates well setup before sending. Shows issues dialog on failure.
        Includes PrintPlanOfAction in the job when valid.
        """
        # Run validation
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'validate'):
            try:
                is_valid, issues = self.tab_wells.validate()
            except Exception as exc:
                is_valid, issues = False, [f"Validation error: {exc}"]

            if not is_valid:
                try:
                    from PySide6.QtWidgets import QMessageBox
                    msg = QMessageBox(self)
                    msg.setWindowTitle("Setup Validation Failed")
                    msg.setIcon(QMessageBox.Icon.Warning)
                    msg.setText(
                        f"Cannot send to monitor: {len(issues)} issue(s) found.")
                    msg.setDetailedText("\n".join(f"\u2022 {i}" for i in issues))
                    msg.exec()
                except Exception:
                    pass
                if hasattr(self, 'status_label'):
                    self.status_label.setText(
                        f"\u26a0 {len(issues)} validation issue(s)")
                    self.status_label.setStyleSheet(
                        f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                logger.warning(f"Send-to-monitor blocked: {issues}")
                return

        # Use pre-generated job if available, otherwise build fresh
        job = getattr(self, '_generated_job', None) or self._build_current_job()
        if job is None:
            if hasattr(self, 'status_label'):
                self.status_label.setText("\u26a0 No job \u2014 configure wells first")
                self.status_label.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 10px;")
            return

        # v7.5.x: early-warn if the configured print Z (or its deepest layer)
        # would punch through the plate bottom. The controller hard-clamps
        # during motion regardless; this just lets the user reconsider first.
        if not self._confirm_print_floor(job):
            return

        # Attach plan of action to job if available
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'get_plan'):
            plan = self.tab_wells.get_plan()
            if plan is not None:
                job.plan_of_action = plan

        # Show confirmation dialog
        summary_parts = [f"{job.total_steps} commands"]
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, '_model'):
            model = self.tab_wells._model
            print_wells = [
                n for n, a in model.assignments.items()
                if getattr(getattr(a, 'role', None), 'value', None) == 'print'
            ]
            summary_parts.insert(0, f"{len(print_wells)} print wells")
        plan = getattr(job, 'plan_of_action', None)
        if plan:
            summary_parts.append(f"{getattr(plan, 'total_runs', '?')} run(s)")
        # v7.2.9: Prefer hybrid estimate over plan's naive estimate
        _hybrid_est = getattr(job, 'estimated_duration_s', 0.0)
        if _hybrid_est > 0:
            summary_parts.append(f"~{_hybrid_est / 60:.1f} min")
        elif plan:
            est = getattr(plan, 'estimated_total_seconds', 0)
            if est > 0:
                summary_parts.append(f"~{est / 60:.1f} min")

        try:
            from PySide6.QtWidgets import QMessageBox
            confirm = QMessageBox.question(
                self,
                "Send to Monitor",
                f"Send job to Print Monitor?\n\n" + "\n".join(summary_parts),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if confirm != QMessageBox.StandardButton.Yes:
                return
        except Exception:
            pass  # If dialog fails, proceed anyway

        self.job_ready.emit(job)
        self._generated_job = None  # Clear after sending
        if hasattr(self, 'status_label'):
            self.status_label.setText(f"\u2713 Job sent: {job.name}")
            self.status_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        logger.info(f"Print job sent to Monitor: {job.name}")


    def _build_current_job(self):
        """Build a PrintJob from current well setup + print objects.

        v7.2.6: Fixed job builder with correct build_well_plate_job API.
        Extracts geometry from Tab 2 objects, well positions from Tab 3 model.
        """
        # 1. Get well model from Tab 3
        if not (hasattr(self, 'tab_wells') and hasattr(self.tab_wells, '_model')
                and self.tab_wells._model):
            logger.warning("No well model available for job building")
            return None

        model = self.tab_wells._model
        plate = getattr(model, 'plate', None)
        if plate is None:
            logger.warning("No plate geometry available")
            return None

        # 2. Get print wells
        print_wells = []
        for name, assignment in model.assignments.items():
            role = getattr(assignment, 'role', None)
            if role is not None and getattr(role, 'value', None) == 'print':
                print_wells.append(name)

        if not print_wells:
            logger.warning("No print wells assigned")
            return None

        # 3. Build well_positions: list of (name, x_mm, y_mm)
        well_positions = []
        for name in print_wells:
            try:
                x, y = plate.get_well_position(name)
            except Exception:
                x, y = 0.0, 0.0
            well_positions.append((name, x, y))

        # 4. Get path points from Tab 2 objects
        path_points = self._get_path_from_objects()
        if not path_points:
            # Fallback: default meander pattern sized to well
            try:
                from SupportClasses.WellPlate import generate_meander_path
                well_diam = getattr(plate, 'well_diameter', 6.0)
                if well_diam <= 0:
                    well_diam = 6.0
                fill_frac = 0.7
                path_points = generate_meander_path(
                    well_diam * fill_frac,
                    well_diam * fill_frac,
                    0.5,
                )
            except Exception:
                path_points = [(0.0, 0.0)]
            logger.info(f"Using fallback meander pattern ({len(path_points)} pts)")

        # 5. Get settings from context panel
        settings = self._get_settings()

        # v7.2.7: push speed to workspace
        # Sync GUI speed to workspace so TrajectoryPlanner uses it
        try:
            _ws = getattr(self, "_workspace", None)
            if _ws is None:
                _ws = getattr(getattr(self, "tab_objects", None), "_workspace", None)
            if _ws and hasattr(_ws, "print_settings"):
                if isinstance(_ws.print_settings, dict):
                    _ws.print_settings["print_speed_mm_s"] = settings.print_speed_mm_s
                    _ws.print_settings["travel_speed_mm_s"] = getattr(settings, "travel_speed_mm_s", 10.0)
                    logger.info(f"Workspace speed synced: {settings.print_speed_mm_s:.1f} mm/s")
        except Exception as _e:
            logger.debug(f"Workspace speed sync: {_e}")

        # 6. Get pump/flow info
        pump = getattr(settings, 'active_pump', 'P1') or 'P1'
        flow_rate = getattr(settings, 'flow_rate', 0.01) or 0.01

        # 7. Build multi-material params from HardwareConfig
        mm_params = {}
        hw = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)
        if hw and hasattr(hw, 'pumps'):
            active_pumps = [
                pid for pid, pcfg in hw.pumps.items()
                if getattr(pcfg, 'enabled', False)
            ]
            if len(active_pumps) > 1:
                mm_params['pump_sequence'] = active_pumps

        # 8. Build job with correct v7.0 API
        try:
            from SupportClasses.PrintManager import build_well_plate_job
            job = build_well_plate_job(
                well_positions=well_positions,
                path_points=path_points,
                settings=settings,
                pump=pump,
                flow_rate=flow_rate,
                job_name=f"Well Plate {getattr(plate, 'format', '?')}-well",
                **mm_params,
            )
            logger.info(
                f"Built job: {job.name}, {job.total_steps} commands, "
                f"{len(well_positions)} wells, {len(path_points)} path pts")
            return job
        except Exception as exc:
            logger.error(f"build_well_plate_job failed: {exc}", exc_info=True)
            return None


    def _get_path_from_objects(self):
        """Extract path points from Tab 2 print objects.

        v7.2.6: Bridge between PrintObjects tab geometry and job builder.
        Returns list of (x, y) tuples or empty list.
        """
        if not hasattr(self, 'tab_objects'):
            return []

        tab = self.tab_objects

        # Primary: read from _objects list (list of dicts)
        objects_list = getattr(tab, '_objects', None)
        if objects_list and isinstance(objects_list, list) and len(objects_list) > 0:
            all_points = []
            for obj in objects_list:
                if not isinstance(obj, dict):
                    continue
                pts = self._single_object_to_points(obj)
                pos = obj.get('position', (0, 0, 0))
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
                else:
                    all_points.extend(pts)
            if all_points:
                logger.info(f"Extracted {len(all_points)} path points from {len(objects_list)} objects")
                return all_points

        # Secondary: try PrintFileData objects dict
        current_file = getattr(tab, '_current_file', None)
        if current_file and hasattr(current_file, 'objects') and current_file.objects:
            all_points = []
            for obj_name, obj_data in current_file.objects.items():
                if isinstance(obj_data, dict):
                    pts = self._single_object_to_points(obj_data)
                    pos = obj_data.get('position', [0, 0, 0])
                    if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                        all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
                    else:
                        all_points.extend(pts)
            if all_points:
                logger.info(f"Extracted {len(all_points)} path points from file data")
                return all_points

        return []

    def _single_object_to_points(self, obj_data) -> list:
        """Convert a single print object dict to path point list.

        v7.2.6: Supports line, meander, spiral, grid, and raw points.
        v7.2.9: Added csv_import support — extracts XY from Nx7 trajectory.
        """
        if isinstance(obj_data, dict):
            obj_type = obj_data.get('object_type', '')
            params = obj_data.get('params', {})
        else:
            obj_type = getattr(obj_data, 'object_type', '')
            params = getattr(obj_data, 'params', {})

        if not isinstance(params, dict):
            params = {}

        # CSV import: extract XY from stored trajectory data
        if obj_type == 'csv_import':
            csv_data = obj_data.get('_csv_data') if isinstance(obj_data, dict) else None
            if csv_data is None and 'source_file' in params:
                try:
                    from SupportClasses.TrajectoryPlanner import import_csv_trajectory
                    csv_data = import_csv_trajectory(params['source_file'])
                except Exception as e:
                    logger.warning(f"Failed to load CSV for path extraction: {e}")
                    return [(0.0, 0.0)]
            if csv_data is not None:
                try:
                    import numpy as np
                    arr = np.asarray(csv_data, dtype=np.float64)
                    if arr.ndim == 2 and arr.shape[1] >= 2 and len(arr) > 0:
                        return [(float(arr[i, 0]), float(arr[i, 1]))
                                for i in range(len(arr))]
                except Exception as e:
                    logger.warning(f"Failed to extract XY from CSV data: {e}")
            return [(0.0, 0.0)]

        try:
            from SupportClasses.WellPlate import (
                generate_line_path, generate_meander_path,
                generate_spiral_path, generate_grid_path,
            )
        except ImportError:
            logger.warning("WellPlate path generators not available")
            return []

        try:
            if obj_type == 'line':
                return generate_line_path(
                    params.get('length', 5.0),
                    params.get('angle', 0.0),
                )
            elif obj_type == 'meander':
                return generate_meander_path(
                    params.get('width', 5.0),
                    params.get('height', 5.0),
                    params.get('spacing', 0.5),
                )
            elif obj_type == 'spiral':
                return generate_spiral_path(
                    params.get('radius', 3.0),
                    params.get('spacing', 0.5),
                )
            elif obj_type == 'grid':
                return generate_grid_path(
                    params.get('width', 5.0),
                    params.get('height', 5.0),
                    params.get('spacing_x', params.get('spacing', 1.0)),
                    params.get('spacing_y', params.get('spacing', 1.0)),
                )
            elif obj_type == 'point' or obj_type == 'dot':
                return [(0.0, 0.0)]
            elif 'points' in params:
                raw = params['points']
                return [(p[0], p[1]) for p in raw if len(p) >= 2]
            else:
                logger.debug(f"Unknown object type '{obj_type}', using center point")
                return [(0.0, 0.0)]
        except Exception as exc:
            logger.warning(f"Failed to generate path for {obj_type}: {exc}")
            return [(0.0, 0.0)]

    def _build_execution_config(self) -> PrintExecutionConfig:
        """v7.2.9: Read Plan of Action UI widgets into a PrintExecutionConfig."""
        cfg = PrintExecutionConfig()

        # Ink swap strategy
        cfg.ink_swap = InkSwapStrategy(
            waste=self._swap_checks.get("waste", None) is not None
                  and self._swap_checks["waste"].isChecked(),
            wash_pre=self._swap_checks.get("wash_pre", None) is not None
                     and self._swap_checks["wash_pre"].isChecked(),
            buffer=self._swap_checks.get("buffer", None) is not None
                   and self._swap_checks["buffer"].isChecked(),
            wash_post=self._swap_checks.get("wash_post", None) is not None
                      and self._swap_checks["wash_post"].isChecked(),
            ink_load=self._swap_checks.get("ink_load", None) is not None
                     and self._swap_checks["ink_load"].isChecked(),
            wash_final=self._swap_checks.get("wash_final", None) is not None
                       and self._swap_checks["wash_final"].isChecked(),
            waste_volume_uL=self._swap_waste_vol.value(),
            wash_volume_uL=self._swap_wash_vol.value(),
            buffer_volume_uL=self._swap_buffer_vol.value(),
            ink_load_volume_uL=self._swap_ink_load_vol.value(),
        )

        # Service sequence toggles (derived from ink swap enables)
        cfg.use_waste = cfg.ink_swap.waste
        cfg.use_wash = cfg.ink_swap.wash_pre or cfg.ink_swap.wash_post
        cfg.use_buffer = cfg.ink_swap.buffer

        # Ink gathering
        extra_pct = self._ink_extra_pct_spin.value()
        max_pickup = self._ink_max_pickup_spin.value()
        cfg.default_gather_config = InkGatherConfig(
            ink_name="default",
            max_pickup_uL=max_pickup,
            extra_percent=extra_pct,
        )

        # Z travel
        cfg.z_travel = ZTravelConfig(
            safe_z_mm=self.travel_z_spin.value(),
            wait_for_z_confirm=self._z_wait_confirm_cb.isChecked(),
            fast_z_exit_well=self._z_fast_exit_cb.isChecked(),
            fast_z_enter_well=self._z_fast_enter_cb.isChecked(),
            use_plunge_buffer=self._z_plunge_cb.isChecked(),
            plunge_buffer_um=self._z_plunge_buffer_spin.value(),
        )

        # XY travel
        cfg.xy_travel = XYTravelConfig(
            fast_xy_travel=self._xy_fast_cb.isChecked(),
            fast_xy_speed_mm_s=self._xy_travel_speed_spin.value(),
        )

        # Final cleanup
        cleanup_grp = getattr(self, '_cleanup_grp', None)
        cfg.final_cleanup = FinalCleanupConfig(
            enabled=cleanup_grp.isChecked() if cleanup_grp else True,
            do_waste=self._cleanup_waste_cb.isChecked(),
            do_wash=self._cleanup_wash_cb.isChecked(),
            do_dry_run=self._cleanup_dry_cb.isChecked(),
        )

        return cfg

    def _generate_print(self):
        """v7.2.9: Use plan_to_trajectory with PrintExecutionConfig.

        Generates a time-parameterized trajectory instead of flat commands.
        The trajectory encodes all axis positions with proper timing from
        feedrate settings, enabling smooth coordinated motion.
        Reads Plan of Action config from UI before generating.
        """
        gen_label = getattr(self, '_gen_status_label', None)

        # Step 1: Validate
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'validate'):
            try:
                is_valid, issues = self.tab_wells.validate()
            except Exception as exc:
                is_valid, issues = False, [f"Validation error: {exc}"]
        else:
            is_valid, issues = False, ["Well setup tab missing validate()"]

        if not is_valid:
            if gen_label:
                gen_label.setText(
                    f"\u26a0 {len(issues)} issue(s):\n"
                    + "\n".join(f"  \u2022 {i}" for i in issues[:5]))
                gen_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            if hasattr(self, 'btn_send_to_monitor'):
                self.btn_send_to_monitor.setEnabled(False)
            return

        # Step 2: Build execution config from Plan of Action UI + generate plan
        exec_cfg = self._build_execution_config()
        plan = None
        if hasattr(self, 'tab_wells'):
            if hasattr(self.tab_wells, '_generate_plan'):
                try:
                    self.tab_wells._generate_plan(execution_config=exec_cfg)
                except Exception as e:
                    if gen_label:
                        gen_label.setText(f"\u26a0 Plan failed: {e}")
                        gen_label.setStyleSheet(
                            f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                    if hasattr(self, 'btn_send_to_monitor'):
                        self.btn_send_to_monitor.setEnabled(False)
                    return
            if hasattr(self.tab_wells, 'get_plan'):
                plan = self.tab_wells.get_plan()

        # Step 3: Get path points and settings
        path_points = self._get_path_from_objects()
        if not path_points:
            try:
                from SupportClasses.WellPlate import generate_meander_path
                model = self.tab_wells._model
                plate = getattr(model, 'plate', None)
                d = getattr(plate, 'well_diameter', 6.0) if plate else 6.0
                path_points = generate_meander_path(d * 0.7, d * 0.7, 0.5)
            except Exception:
                path_points = [(0.0, 0.0)]

        settings = self._get_settings()
        # v7.2.6-auto-C2: auto-derive speeds + pump rates from HW + calibration
        _hw_for_auto = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)
        if hasattr(self, '_compute_auto_settings'):
            self._compute_auto_settings(settings, _hw_for_auto)
        model = getattr(self.tab_wells, '_model', None)
        plate = getattr(model, 'plate', None) if model else None
        hw = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)

        # Step 4: Generate trajectory (preferred) or fall back to commands
        job = None
        trajectory_result = None

        if plan is not None and plate is not None:
            try:
                from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
                # v7.2.7: rescale trajectory speed
                # Object trajectories were time-parameterized at object creation
                # speed (default 5mm/s). Rescale timestamps to match GUI speed.
                _gui_speed = getattr(settings, "print_speed_mm_s", 5.0)
                if _gui_speed > 0 and _gui_speed != 5.0:
                    try:
                        _objects_tab = getattr(self, "tab_objects", None)
                        if _objects_tab:
                            _objs = getattr(_objects_tab, "_objects", [])
                            for _odata in _objs:
                                _obj = _odata if not isinstance(_odata, dict) else None
                                if _obj is None and isinstance(_odata, dict):
                                    _obj = _odata.get("_print_object")
                                if _obj and hasattr(_obj, "trajectory") and _obj.trajectory is not None:
                                    import numpy as _np
                                    _traj = _obj.trajectory
                                    if len(_traj) > 1:
                                        # Compute what speed the trajectory was generated at
                                        _dx = _np.diff(_traj[:, 0])
                                        _dy = _np.diff(_traj[:, 1])
                                        _dt = _np.diff(_traj[:, 6])
                                        _dt_safe = _np.maximum(_dt, 1e-9)
                                        _dists = _np.sqrt(_dx**2 + _dy**2)
                                        _speeds = _dists / _dt_safe
                                        _mask = _dists > 0.001
                                        if _np.any(_mask):
                                            _orig_speed = float(_np.median(_speeds[_mask]))
                                            if _orig_speed > 0.1:
                                                _scale = _orig_speed / _gui_speed
                                                _traj[:, 6] *= _scale
                                                logger.info(f"Rescaled trajectory: "
                                                           f"{_orig_speed:.1f} -> {_gui_speed:.1f} mm/s "
                                                           f"(scale={_scale:.3f})")
                    except Exception as _e:
                        logger.warning(f"Trajectory speed rescale failed: {_e}")

                # v7.2.7: sync all speeds to workspace

                try:

                    _ws = getattr(self, "_workspace", None)

                    if _ws is None:

                        _ws = getattr(getattr(self, "tab_objects", None), "_workspace", None)

                    if _ws and hasattr(_ws, "print_settings") and isinstance(_ws.print_settings, dict):

                        _gui_spd = getattr(settings, "print_speed_mm_s", 5.0)

                        _gui_travel = getattr(settings, "travel_speed_mm_s", _gui_spd * 2)

                        _gui_z = getattr(settings, "z_feedrate", 60.0)

                        _ws.print_settings["print_speed_mm_s"] = _gui_spd

                        _ws.print_settings["travel_speed_mm_s"] = _gui_travel

                        _ws.print_settings["z_feed_rate_mm_s"] = _gui_z if _gui_z < 20 else _gui_z / 60.0

                        _ws.print_settings["travel_z_mm"] = getattr(settings, "travel_z_height", 5.0)

                        logger.info(f"Workspace synced: print={_gui_spd:.1f}, "

                                   f"travel={_gui_travel:.1f}, z={_ws.print_settings['z_feed_rate_mm_s']:.1f} mm/s")

                except Exception as _e:

                    logger.debug(f"Workspace speed sync: {_e}")


                trajectory_result = plan_to_trajectory(
                    plan=plan, well_model=model, plate=plate,
                    path_points=path_points, settings=settings, hw_config=hw)

                if trajectory_result.valid:
                    # Build a lightweight PrintJob that carries the waypoints
                    from SupportClasses.PrintManager import PrintJob
                    job = PrintJob(
                        name=f"Trajectory: {trajectory_result.well_count} wells",
                        description=trajectory_result.summary(),
                        settings=settings,
                        commands=[],  # empty — execution uses waypoints
                    )
                    job.trajectory_waypoints = trajectory_result.waypoints
                    job.trajectory_result = trajectory_result
                    if plan:
                        job.plan_of_action = plan
                    # v7.2.9: Attach context for hybrid execution
                    job.plate = plate
                    job.path_points = path_points
                    job.well_setup = model
                    job.hw_config = hw

                    # v7.2.9: Compute hybrid time estimate (blocking-aware)
                    # This replaces the naive trajectory_result.total_duration_s
                    # which doesn't account for per-waypoint blocking overhead.
                    try:
                        from SupportClasses.PrintManager import HybridPlanExecutor
                        _est_executor = HybridPlanExecutor(
                            controller=None,  # estimate_time doesn't need HW
                            plan=plan, well_model=model, plate=plate,
                            path_points=path_points, settings=settings,
                            hw_config=hw)
                        hybrid_est = _est_executor.estimate_time()
                        job.estimated_duration_s = hybrid_est
                        logger.info(f"Hybrid time estimate: {hybrid_est:.1f}s "
                                    f"(vs trajectory {trajectory_result.total_duration_s:.1f}s)")
                    except Exception as e:
                        logger.warning(f"Hybrid estimate failed: {e}")
                        job.estimated_duration_s = 0.0

                    logger.info(
                        f"Trajectory generated: {len(trajectory_result.waypoints)} "
                        f"waypoints, {trajectory_result.total_duration_s:.1f}s")
                else:
                    logger.warning(f"Trajectory invalid: {trajectory_result.issues}")
                    if gen_label:
                        gen_label.setText(
                            "\u26a0 " + "; ".join(trajectory_result.issues[:3]))
                        gen_label.setStyleSheet(
                            f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                    if hasattr(self, 'btn_send_to_monitor'):
                        self.btn_send_to_monitor.setEnabled(False)
                    return
            except ImportError:
                logger.warning("PrintTrajectoryPlanner not available, using commands")
            except Exception as exc:
                logger.error(f"Trajectory generation failed: {exc}", exc_info=True)

        # Fallback to command-based job
        if job is None:
            job = self._build_current_job()

        if job is None:
            if gen_label:
                gen_label.setText("\u26a0 Could not build print job")
                gen_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            if hasattr(self, 'btn_send_to_monitor'):
                self.btn_send_to_monitor.setEnabled(False)
            return

        self._generated_job = job

        # Step 5: Show success
        parts = [f"\u2713 Ready: {getattr(job, 'name', '?')}"]
        if trajectory_result:
            # v7.2.9: Show hybrid estimate if available (blocking-aware)
            _est_s = getattr(job, 'estimated_duration_s', 0.0)
            if _est_s <= 0:
                _est_s = trajectory_result.total_duration_s
            parts.append(f"{_est_s / 60:.1f} min")
            parts.append(f"{len(trajectory_result.waypoints)} waypoints")
        elif hasattr(job, 'total_steps'):
            parts.append(f"{job.total_steps} commands")
        if gen_label:
            gen_label.setText(" | ".join(parts))
            gen_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        if hasattr(self, 'btn_send_to_monitor'):
            self.btn_send_to_monitor.setEnabled(True)
        logger.info(f"Generate Print complete: {parts}")


    def _export_gcode(self):
        """Export the current job as G-code."""
        job = self._build_current_job()
        if job is None:
            QMessageBox.warning(self, "No Job",
                                "Configure wells before exporting.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export G-code", f"{job.name}.gcode",
            "G-code files (*.gcode)")
        if path:
            try:
                export_gcode(job, path)
                self.status_label.setText(f"✓ Exported to {path}")
            except Exception as e:
                QMessageBox.critical(self, "Export Error", str(e))

    def _save_job(self):
        """Save the current job as JSON."""
        job = self._build_current_job()
        if job is None:
            QMessageBox.warning(self, "No Job",
                                "Configure wells before saving.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Job", f"{job.name}.json",
            "JSON files (*.json)")
        if path:
            try:
                save_print_job(job, path)
                self.status_label.setText(f"✓ Saved to {path}")
            except Exception as e:
                QMessageBox.critical(self, "Save Error", str(e))

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
        """v7.2.5: Print Objects tab changed collections → update Well Setup tab."""
        if hasattr(self.tab_wells, 'set_available_prints'):
            self.tab_wells.set_available_prints(collection_names)
            logger.info(f"Forwarded {len(collection_names)} prints to well setup")

    def _on_setup_changed(self):
        """Well Setup tab changed → may affect execution readiness."""
        pass

    # ════════════════════════════════════════════════════════════════
    #  PROGRESS / STATE CALLBACKS (kept for S3 transition)
    # ════════════════════════════════════════════════════════════════

    def _on_progress(self, step: int, total: int, message: str):
        """Handle progress updates (minimal — will move to Monitor in S3)."""
        if hasattr(self, 'status_label'):
            self.status_label.setText(message)

    def _on_state_changed(self, state):
        """Handle state changes (minimal — will move to Monitor in S3)."""
        if state == PrintState.COMPLETED:
            if hasattr(self, 'status_label'):
                self.status_label.setText("✓ Print completed!")
        elif state == PrintState.ERROR:
            if hasattr(self, 'status_label'):
                self.status_label.setText("⚠ Print error")
