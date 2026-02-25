"""
Print Setup Page - Load, preview, configure, and execute print jobs.

Session 4 additions:
- Task 1: Multi-material pump selector (per-well or per-layer)
- Task 4: Live print visualization (current position, completed/remaining coloring, layers)
- Task 6: Print queue UI (job list, add/remove/reorder, start queue)

Layout:
┌─────────────────────────────────────────────────────────────────────┐
│  [File / Well Plate / Pattern tabs]                                 │
│  ┌──────────────────────┬──────────────────────────────────────────┐│
│  │                      │  Settings Panel                         ││
│  │   2D Path Preview    │  ──────────────────                     ││
│  │   Canvas             │  Print Settings + Multi-material        ││
│  │                      │  Execution Controls                     ││
│  │                      │  Queue Panel                            ││
│  │                      │  Progress                               ││
│  └──────────────────────┴──────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────┘
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QProgressBar, QScrollArea, QFrame,
    QCheckBox, QListWidget, QListWidgetItem, QAbstractItemView,
    QSplitter, QSizePolicy,
)
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF, Signal, QObject
from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPainterPath

from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintJob, PrintSettings, PrintState, PrintQueue,
    load_print_file, build_well_plate_job, save_print_job,
)
from SupportClasses.WellPlate import (
    WellPlate, PLATE_DEFINITIONS,
    generate_meander_path, generate_spiral_path, generate_line_path,
    generate_grid_path,
)

import logging
logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Thread → GUI Signal Bridge
# ═══════════════════════════════════════════════════════════════════

class PrintSignalBridge(QObject):
    """Thread-safe bridge for PrintManager → GUI signals."""
    progress_signal = Signal(int, int, str)     # step, total, message
    state_signal = Signal(object)               # PrintState enum
    queue_progress_signal = Signal(int, int, str)  # job_idx, total, name
    queue_completed_signal = Signal()


# ═══════════════════════════════════════════════════════════════════
# 2D Path Preview Canvas (Task 4: Enhanced)
# ═══════════════════════════════════════════════════════════════════

class PathPreviewCanvas(QWidget):
    """
    2D canvas that renders the print toolpath.
    Shows travel moves (dashed gray) and print moves (solid colored).
    Optionally shows well plate outlines.
    
    Session 4 enhancements:
    - Current position marker during printing
    - Completed vs remaining path coloring
    - Layer filtering
    """

    # Pump color mapping for multi-material
    PUMP_COLORS = {
        "P1": QColor("#a6e3a1"),  # green
        "P2": QColor("#89b4fa"),  # blue
        "P3": QColor("#f9e2af"),  # yellow
    }
    COMPLETED_COLOR = QColor("#a6e3a1")    # bright green
    REMAINING_COLOR = QColor(166, 227, 161, 80)  # dim green
    TRAVEL_DONE_COLOR = QColor(100, 100, 100, 100)
    TRAVEL_TODO_COLOR = QColor(100, 100, 100, 40)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._segments: list[dict] = []
        self._wells: list[dict] = []
        self._margin = 40
        self._show_grid = True

        # Task 4: Live visualization state
        self._current_pos: tuple | None = None   # (x, y) in zero-ref coords
        self._completed_step: int = 0             # command index completed so far
        self._total_steps: int = 0
        self._active_layer: int = -1              # -1 = show all layers
        self._layer_boundaries: list[tuple[int, str]] = []

    def set_path_segments(self, segments: list[dict]):
        """Set path segments for rendering."""
        self._segments = segments
        self.update()

    def set_wells(self, wells: list[dict]):
        """Set well positions for rendering."""
        self._wells = wells
        self.update()

    def set_current_position(self, x: float, y: float):
        """Task 4: Set current position marker during active print."""
        self._current_pos = (x, y)
        self.update()

    def clear_current_position(self):
        """Remove the current position marker."""
        self._current_pos = None
        self.update()

    def set_progress(self, completed_step: int, total_steps: int):
        """Task 4: Set completed progress for dual-color rendering."""
        self._completed_step = completed_step
        self._total_steps = total_steps
        self.update()

    def set_layer_boundaries(self, boundaries: list[tuple[int, str]]):
        """Task 4: Set layer boundaries for layer filtering."""
        self._layer_boundaries = boundaries

    def set_active_layer(self, layer: int):
        """Task 4: Set which layer to show (-1 = all)."""
        self._active_layer = layer
        self.update()

    def clear(self):
        self._segments = []
        self._wells = []
        self._current_pos = None
        self._completed_step = 0
        self._total_steps = 0
        self.update()

    def _get_bounds(self):
        """Calculate bounding box of all geometry."""
        all_x, all_y = [], []

        for seg in self._segments:
            for pt in seg.get("points", []):
                all_x.append(pt[0])
                all_y.append(pt[1])

        for well in self._wells:
            r = well.get("diameter", 10) / 2
            all_x.extend([well["x"] - r, well["x"] + r])
            all_y.extend([well["y"] - r, well["y"] + r])

        if not all_x or not all_y:
            return QRectF(-10, -10, 20, 20)

        margin = 2
        return QRectF(
            min(all_x) - margin, min(all_y) - margin,
            max(all_x) - min(all_x) + 2 * margin,
            max(all_y) - min(all_y) + 2 * margin,
        )

    def _transform(self, x, y, bounds, w, h):
        """Transform data coordinates to canvas coordinates."""
        if bounds.width() == 0 or bounds.height() == 0:
            return self._margin + w / 2, self._margin + h / 2

        scale_x = (w - 2 * self._margin) / bounds.width()
        scale_y = (h - 2 * self._margin) / bounds.height()
        scale = min(scale_x, scale_y)

        cx = (x - bounds.x()) * scale + self._margin
        cy = (y - bounds.y()) * scale + self._margin

        # Offset to center
        used_w = bounds.width() * scale
        used_h = bounds.height() * scale
        cx += (w - 2 * self._margin - used_w) / 2
        cy += (h - 2 * self._margin - used_h) / 2

        return cx, cy

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        bounds = self._get_bounds()

        # Background
        painter.fillRect(0, 0, w, h, QColor("#181825"))

        # Grid
        if self._show_grid:
            self._draw_grid(painter, bounds, w, h)

        # Origin marker
        ox, oy = self._transform(0, 0, bounds, w, h)
        painter.setPen(QPen(QColor("#f38ba8"), 2))
        painter.drawLine(int(ox - 8), int(oy), int(ox + 8), int(oy))
        painter.drawLine(int(ox), int(oy - 8), int(ox), int(oy + 8))

        # Wells
        for well in self._wells:
            wx, wy = self._transform(well["x"], well["y"], bounds, w, h)
            r = well.get("diameter", 6) / 2
            # Scale radius
            scale = min(
                (w - 2 * self._margin) / max(bounds.width(), 0.001),
                (h - 2 * self._margin) / max(bounds.height(), 0.001),
            )
            pr = r * scale

            painter.setPen(QPen(QColor("#585b70"), 1))
            painter.setBrush(QBrush(QColor(88, 91, 112, 30)))
            painter.drawEllipse(QPointF(wx, wy), pr, pr)

            # Well label
            painter.setPen(QColor("#6c7086"))
            font = QFont("Consolas", 7)
            painter.setFont(font)
            painter.drawText(int(wx - 10), int(wy - pr - 3), well.get("name", ""))

        # Path segments with progress coloring
        cmd_index = 0
        for seg in self._segments:
            points = seg.get("points", [])
            if len(points) < 2:
                cmd_index += 1
                continue

            is_print = seg.get("type") == "print"
            pump = seg.get("pump", "P1")

            if is_print:
                if cmd_index < self._completed_step and self._total_steps > 0:
                    color = self.COMPLETED_COLOR
                elif self._total_steps > 0:
                    color = self.REMAINING_COLOR
                else:
                    color = self.PUMP_COLORS.get(pump, self.COMPLETED_COLOR)
                pen = QPen(color, 2)
            else:
                if cmd_index < self._completed_step and self._total_steps > 0:
                    color = self.TRAVEL_DONE_COLOR
                else:
                    color = self.TRAVEL_TODO_COLOR
                pen = QPen(color, 1, Qt.PenStyle.DashLine)

            painter.setPen(pen)
            for i in range(len(points) - 1):
                x1, y1 = self._transform(points[i][0], points[i][1], bounds, w, h)
                x2, y2 = self._transform(points[i + 1][0], points[i + 1][1], bounds, w, h)
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

            cmd_index += 1

        # Task 4: Current position marker
        if self._current_pos:
            cx, cy = self._transform(self._current_pos[0], self._current_pos[1], bounds, w, h)

            # Pulsing circle effect (using step count for animation)
            pulse = 6 + (self._completed_step % 4)
            painter.setPen(QPen(QColor("#f38ba8"), 2))
            painter.setBrush(QBrush(QColor(243, 139, 168, 120)))
            painter.drawEllipse(QPointF(cx, cy), pulse, pulse)

            # Crosshair
            painter.setPen(QPen(QColor("#f38ba8"), 1))
            painter.drawLine(int(cx - 12), int(cy), int(cx + 12), int(cy))
            painter.drawLine(int(cx), int(cy - 12), int(cx), int(cy + 12))

        painter.end()

    def _draw_grid(self, painter, bounds, w, h):
        """Draw a light grid background."""
        painter.setPen(QPen(QColor("#313244"), 1, Qt.PenStyle.DotLine))
        grid_step = max(1, int(bounds.width() / 10))
        if grid_step < 1:
            return

        for gx in range(int(bounds.x()), int(bounds.x() + bounds.width()), grid_step):
            x, _ = self._transform(gx, 0, bounds, w, h)
            painter.drawLine(int(x), 0, int(x), h)

        grid_step_y = max(1, int(bounds.height() / 10))
        for gy in range(int(bounds.y()), int(bounds.y() + bounds.height()), grid_step_y):
            _, y = self._transform(0, gy, bounds, w, h)
            painter.drawLine(0, int(y), w, int(y))


# ═══════════════════════════════════════════════════════════════════
# Print Setup Page
# ═══════════════════════════════════════════════════════════════════

class PrintSetupPage(QWidget):
    """
    Complete print setup page with file loading, well plate generation,
    pattern generation, settings, execution, and live preview.
    """

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.print_manager = PrintManager(controller)
        self.print_queue = PrintQueue(controller)

        # Signal bridge for thread → GUI communication
        self._bridge = PrintSignalBridge()
        self._bridge.progress_signal.connect(self._on_progress)
        self._bridge.state_signal.connect(self._on_state_changed)
        self._bridge.queue_progress_signal.connect(self._on_queue_progress)
        self._bridge.queue_completed_signal.connect(self._on_queue_completed)

        self.print_manager.on_progress = lambda s, t, m: self._bridge.progress_signal.emit(s, t, m)
        self.print_manager.on_state_changed = lambda st: self._bridge.state_signal.emit(st)

        # Wire queue callbacks
        self.print_queue.on_progress = lambda s, t, m: self._bridge.progress_signal.emit(s, t, m)
        self.print_queue.on_state_changed = lambda st: self._bridge.state_signal.emit(st)
        self.print_queue.on_queue_progress = lambda i, t, n: self._bridge.queue_progress_signal.emit(i, t, n)
        self.print_queue.on_queue_completed = lambda: self._bridge.queue_completed_signal.emit()

        self._current_well_plate = None

        self._setup_ui()

    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(8)

        # ── Left: Source Tabs + Canvas ─────────────────────────────
        left_panel = QVBoxLayout()

        # Source selection tabs
        self.source_tabs = QTabWidget()
        self.source_tabs.setMaximumHeight(260)

        self._build_file_tab()
        self._build_wellplate_tab()
        self._build_pattern_tab()

        left_panel.addWidget(self.source_tabs)

        # Task 4: Layer selector
        layer_row = QHBoxLayout()
        layer_row.addWidget(QLabel("Layer:"))
        self.layer_combo = QComboBox()
        self.layer_combo.addItem("All Layers", -1)
        self.layer_combo.currentIndexChanged.connect(self._on_layer_changed)
        layer_row.addWidget(self.layer_combo, stretch=1)
        left_panel.addLayout(layer_row)

        # Preview canvas
        canvas_group = QGroupBox("Path Preview")
        canvas_layout = QVBoxLayout(canvas_group)
        self.canvas = PathPreviewCanvas()
        canvas_layout.addWidget(self.canvas)
        left_panel.addWidget(canvas_group, stretch=1)

        main_layout.addLayout(left_panel, stretch=3)

        # ── Right: Settings + Controls ─────────────────────────────
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.Shape.NoFrame)
        right_scroll.setMinimumWidth(280)

        right_content = QWidget()
        right_panel = QVBoxLayout(right_content)

        self._build_settings_panel(right_panel)
        self._build_execution_panel(right_panel)
        self._build_queue_panel(right_panel)

        right_panel.addStretch()
        right_scroll.setWidget(right_content)
        main_layout.addWidget(right_scroll, stretch=1)

    # ── Source Tab: File Loading ────────────────────────────────────

    def _build_file_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        browse_row = QHBoxLayout()
        self.file_label = QLabel("No file loaded")
        self.file_label.setStyleSheet("color: #a6adc8;")
        browse_row.addWidget(self.file_label, stretch=1)

        btn_browse = QPushButton("Browse...")
        btn_browse.clicked.connect(self._browse_file)
        browse_row.addWidget(btn_browse)

        layout.addLayout(browse_row)

        self.job_info_label = QLabel("")
        self.job_info_label.setWordWrap(True)
        self.job_info_label.setStyleSheet("color: #6c7086;")
        layout.addWidget(self.job_info_label)

        layout.addStretch()
        self.source_tabs.addTab(tab, "📁 File")

    # ── Source Tab: Well Plate ─────────────────────────────────────

    def _build_wellplate_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        format_row = QHBoxLayout()
        format_row.addWidget(QLabel("Plate:"))
        self.plate_combo = QComboBox()
        for fmt in PLATE_DEFINITIONS:
            desc = PLATE_DEFINITIONS[fmt]["description"]
            self.plate_combo.addItem(f"{fmt}-well ({desc})", fmt)
        self.plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        format_row.addWidget(self.plate_combo)
        layout.addLayout(format_row)

        well_row = QHBoxLayout()
        self.well_list = QListWidget()
        self.well_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)
        self.well_list.setMaximumHeight(120)
        well_row.addWidget(self.well_list)

        btn_col = QVBoxLayout()
        btn_all = QPushButton("All")
        btn_all.clicked.connect(self._select_all_wells)
        btn_col.addWidget(btn_all)
        btn_none = QPushButton("None")
        btn_none.clicked.connect(self._select_no_wells)
        btn_col.addWidget(btn_none)
        btn_col.addStretch()
        well_row.addLayout(btn_col)
        layout.addLayout(well_row)

        btn_generate = QPushButton("Generate Well Plate Job")
        btn_generate.setObjectName("connectBtn")
        btn_generate.clicked.connect(self._generate_well_plate_job)
        layout.addWidget(btn_generate)

        self.source_tabs.addTab(tab, "🧫 Well Plate")
        self._on_plate_changed()

    # ── Source Tab: Pattern ────────────────────────────────────────

    def _build_pattern_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        grid = QGridLayout()
        row = 0

        grid.addWidget(QLabel("Pattern:"), row, 0)
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(["Line", "Meander", "Spiral", "Grid"])
        self.pattern_combo.currentIndexChanged.connect(self._update_pattern_preview)
        grid.addWidget(self.pattern_combo, row, 1)
        row += 1

        for label, attr, default, suffix in [
            ("Width:", "pat_width", 5.0, "mm"),
            ("Height:", "pat_height", 5.0, "mm"),
            ("Spacing:", "pat_spacing", 0.5, "mm"),
            ("Angle:", "pat_angle", 0.0, "°"),
        ]:
            grid.addWidget(QLabel(label), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(0.1, 100)
            spin.setValue(default)
            spin.setSuffix(f" {suffix}")
            spin.valueChanged.connect(self._update_pattern_preview)
            grid.addWidget(spin, row, 1)
            setattr(self, f"{attr}", spin)
            row += 1

        layout.addLayout(grid)

        btn_preview = QPushButton("Preview Pattern")
        btn_preview.clicked.connect(self._update_pattern_preview)
        layout.addWidget(btn_preview)

        layout.addStretch()
        self.source_tabs.addTab(tab, "🔷 Pattern")

    # ── Settings Panel ─────────────────────────────────────────────

    def _build_settings_panel(self, parent_layout):
        group = QGroupBox("Print Settings")
        grid = QGridLayout(group)
        row = 0

        def add_spin(label, attr, min_v, max_v, default, decimals=1, suffix=""):
            nonlocal row
            grid.addWidget(QLabel(label), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setValue(default)
            spin.setDecimals(decimals)
            if suffix:
                spin.setSuffix(f" {suffix}")
            grid.addWidget(spin, row, 1)
            setattr(self, f"setting_{attr}", spin)
            row += 1

        add_spin("Travel Z Height:", "travel_z", 0, 50, 5.0, 2, "mm")
        add_spin("Print Z Height:", "print_z", -5, 50, 0.1, 3, "mm")
        add_spin("Layer Height:", "layer_h", 0.01, 5, 0.1, 3, "mm")

        grid.addWidget(QLabel("Layers:"), row, 0)
        self.setting_layers = QSpinBox()
        self.setting_layers.setRange(1, 999)
        self.setting_layers.setValue(1)
        grid.addWidget(self.setting_layers, row, 1)
        row += 1

        add_spin("Print Speed:", "print_speed", 1, 10000, 200, 0, "mm/min")
        add_spin("Z Feedrate:", "z_feed", 1, 1000, 60, 0, "mm/min")
        add_spin("Pump Feedrate:", "pump_feed", 1, 500, 30, 0, "mm/min")
        add_spin("Flow Rate:", "flow_rate", 0.001, 10, 0.01, 3, "mm/mm")

        # Task 1: Pump selector with multi-material option
        grid.addWidget(QLabel("Pump:"), row, 0)
        self.pump_combo = QComboBox()
        self.pump_combo.addItems(["P1", "P2", "P3"])
        grid.addWidget(self.pump_combo, row, 1)
        row += 1

        # Multi-material checkbox
        self.chk_multi_material = QCheckBox("Multi-material")
        self.chk_multi_material.toggled.connect(self._on_multi_material_toggled)
        grid.addWidget(self.chk_multi_material, row, 0, 1, 2)
        row += 1

        # Multi-material mode (hidden by default)
        self.multi_material_widget = QWidget()
        mm_layout = QVBoxLayout(self.multi_material_widget)
        mm_layout.setContentsMargins(0, 0, 0, 0)

        mm_layout.addWidget(QLabel("Assignment Mode:"))
        self.mm_mode_combo = QComboBox()
        self.mm_mode_combo.addItems(["Per-layer (P1→P2→P3)", "Alternate wells (P1, P2)"])
        mm_layout.addWidget(self.mm_mode_combo)

        mm_layout.addWidget(QLabel("Pumps to use:"))
        self.chk_mm_p1 = QCheckBox("P1")
        self.chk_mm_p1.setChecked(True)
        self.chk_mm_p2 = QCheckBox("P2")
        self.chk_mm_p2.setChecked(True)
        self.chk_mm_p3 = QCheckBox("P3")
        pump_row = QHBoxLayout()
        pump_row.addWidget(self.chk_mm_p1)
        pump_row.addWidget(self.chk_mm_p2)
        pump_row.addWidget(self.chk_mm_p3)
        mm_layout.addLayout(pump_row)

        self.multi_material_widget.setVisible(False)
        grid.addWidget(self.multi_material_widget, row, 0, 1, 2)
        row += 1

        add_spin("Retraction:", "retract", 0, 10, 0.0, 2, "mm")
        add_spin("Prime:", "prime", 0, 10, 0.0, 2, "mm")
        add_spin("Settle Delay:", "settle", 0, 10, 0.0, 1, "s")

        parent_layout.addWidget(group)

    def _on_multi_material_toggled(self, checked):
        self.multi_material_widget.setVisible(checked)
        self.pump_combo.setEnabled(not checked)

    def _get_settings(self) -> PrintSettings:
        """Read current settings from UI into a PrintSettings object."""
        retract = self.setting_retract.value()
        prime = self.setting_prime.value()
        return PrintSettings(
            xy_feedrate=1000.0,
            z_feedrate=self.setting_z_feed.value(),
            print_feedrate=self.setting_print_speed.value(),
            pump_feedrate=self.setting_pump_feed.value(),
            travel_z_height=self.setting_travel_z.value(),
            print_z_height=self.setting_print_z.value(),
            layer_height=self.setting_layer_h.value(),
            num_layers=self.setting_layers.value(),
            retract_amount=retract,
            prime_amount=prime,
            dwell_after_move=self.setting_settle.value(),
            # Per-pump amounts (use global as default for all)
            retract_amounts={"P1": retract, "P2": retract, "P3": retract},
            prime_amounts={"P1": prime, "P2": prime, "P3": prime},
        )

    def _get_multi_material_params(self) -> dict:
        """Get multi-material parameters from UI."""
        if not self.chk_multi_material.isChecked():
            return {}

        pumps = []
        if self.chk_mm_p1.isChecked():
            pumps.append("P1")
        if self.chk_mm_p2.isChecked():
            pumps.append("P2")
        if self.chk_mm_p3.isChecked():
            pumps.append("P3")

        mode = self.mm_mode_combo.currentIndex()
        if mode == 0:  # Per-layer
            pump_per_layer = {}
            for i in range(self.setting_layers.value()):
                pump_per_layer[str(i + 1)] = pumps[i % len(pumps)] if pumps else "P1"
            return {"pump_per_layer": pump_per_layer}
        else:  # Alternate wells
            return {"pump_sequence": pumps if pumps else ["P1"]}

    # ── Execution Panel ────────────────────────────────────────────

    def _build_execution_panel(self, parent_layout):
        group = QGroupBox("Execution")
        layout = QVBoxLayout(group)

        # Buttons
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ Start Print")
        self.btn_start.setObjectName("connectBtn")
        self.btn_start.clicked.connect(self._on_start)
        btn_row.addWidget(self.btn_start)

        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.clicked.connect(self._on_pause)
        btn_row.addWidget(self.btn_pause)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setObjectName("disconnectBtn")
        self.btn_abort.clicked.connect(self._on_abort)
        btn_row.addWidget(self.btn_abort)
        layout.addLayout(btn_row)

        # Progress
        self.progress_bar = QProgressBar()
        self.progress_bar.setFormat("%p%")
        layout.addWidget(self.progress_bar)

        self.progress_label = QLabel("Status: Idle")
        self.progress_label.setWordWrap(True)
        layout.addWidget(self.progress_label)

        parent_layout.addWidget(group)

    # ── Queue Panel (Task 6) ───────────────────────────────────────

    def _build_queue_panel(self, parent_layout):
        group = QGroupBox("Print Queue")
        layout = QVBoxLayout(group)

        self.queue_list = QListWidget()
        self.queue_list.setMaximumHeight(100)
        self.queue_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        layout.addWidget(self.queue_list)

        btn_row = QHBoxLayout()
        btn_add = QPushButton("+ Add to Queue")
        btn_add.clicked.connect(self._add_to_queue)
        btn_row.addWidget(btn_add)

        btn_remove = QPushButton("- Remove")
        btn_remove.clicked.connect(self._remove_from_queue)
        btn_row.addWidget(btn_remove)

        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self._clear_queue)
        btn_row.addWidget(btn_clear)
        layout.addLayout(btn_row)

        btn_start_queue = QPushButton("▶ Start Queue")
        btn_start_queue.setObjectName("connectBtn")
        btn_start_queue.clicked.connect(self._start_queue)
        layout.addWidget(btn_start_queue)

        self.queue_progress_label = QLabel("")
        self.queue_progress_label.setStyleSheet("color: #6c7086;")
        layout.addWidget(self.queue_progress_label)

        parent_layout.addWidget(group)

    # ── File Loading ───────────────────────────────────────────────

    def _browse_file(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Open Print File", "",
            "Print Files (*.json *.gcode *.gco);;JSON (*.json);;G-Code (*.gcode *.gco);;All (*)"
        )
        if filepath:
            try:
                self.print_manager.load_file(filepath)
                job = self.print_manager.job
                self.file_label.setText(filepath.split("/")[-1].split("\\")[-1])
                self.job_info_label.setText(
                    f"Job: {job.name}\n"
                    f"{job.description}\n"
                    f"Commands: {job.total_steps}"
                )
                self._refresh_preview()
                self._update_layer_combo()
            except Exception as e:
                self.file_label.setText(f"Error: {e}")
                logger.error(f"Failed to load file: {e}")

    # ── Well Plate ─────────────────────────────────────────────────

    def _on_plate_changed(self):
        fmt = self.plate_combo.currentData()
        if fmt is None:
            return
        self._current_well_plate = WellPlate.from_format(fmt)
        self.well_list.clear()
        for well in self._current_well_plate.get_all_wells():
            item = QListWidgetItem(well.name)
            self.well_list.addItem(item)
        self._update_well_preview()

    def _select_all_wells(self):
        for i in range(self.well_list.count()):
            self.well_list.item(i).setSelected(True)

    def _select_no_wells(self):
        self.well_list.clearSelection()

    def _get_selected_wells(self) -> list:
        return [item.text() for item in self.well_list.selectedItems()]

    def _update_well_preview(self):
        if not self._current_well_plate:
            return
        wells = []
        for well in self._current_well_plate.get_all_wells():
            wells.append({
                "name": well.name, "x": well.x, "y": well.y,
                "diameter": well.diameter,
            })
        self.canvas.set_wells(wells)

    def _generate_well_plate_job(self):
        if not self._current_well_plate:
            return

        selected = self._get_selected_wells()
        if not selected:
            self.progress_label.setText("No wells selected!")
            return

        pattern_points = self._generate_current_pattern()
        if not pattern_points:
            self.progress_label.setText("No pattern generated!")
            return

        well_positions = []
        for name in selected:
            try:
                x, y = self._current_well_plate.get_well_position(name)
                well_positions.append((name, x, y))
            except KeyError:
                pass

        settings = self._get_settings()
        pump = self.pump_combo.currentText()
        flow_rate = self.setting_flow_rate.value()

        # Task 1: Get multi-material params
        mm_params = self._get_multi_material_params()

        job = build_well_plate_job(
            well_positions=well_positions,
            path_points=pattern_points,
            settings=settings,
            pump=pump,
            flow_rate=flow_rate,
            job_name=f"Well Plate {self._current_well_plate.format}-well",
            **mm_params,
        )

        self.print_manager.load_job(job)
        self.job_info_label.setText(
            f"Job: {job.name}\n{job.description}\nCommands: {job.total_steps}"
        )
        self._refresh_preview()
        self._update_layer_combo()
        self.progress_label.setText(f"Job generated: {len(selected)} wells, {settings.num_layers} layers")

    # ── Pattern Generation ─────────────────────────────────────────

    def _generate_current_pattern(self) -> list[tuple[float, float]]:
        pattern = self.pattern_combo.currentText()
        w = self.pat_width.value()
        h = self.pat_height.value()
        spacing = self.pat_spacing.value()
        angle = self.pat_angle.value()

        if pattern == "Line":
            return generate_line_path(w, angle)
        elif pattern == "Meander":
            return generate_meander_path(w, h, spacing)
        elif pattern == "Spiral":
            return generate_spiral_path(w, spacing)
        elif pattern == "Grid":
            return generate_grid_path(w, h, spacing, spacing)
        return []

    def _update_pattern_preview(self):
        points = self._generate_current_pattern()
        if not points:
            return
        segments = [{"type": "print", "points": points}]
        self.canvas.set_path_segments(segments)
        self._update_well_preview()

    # ── Preview Refresh ────────────────────────────────────────────

    def _refresh_preview(self):
        if not self.print_manager.job:
            self.canvas.clear()
            return
        segments = self.print_manager.job.get_path_segments()
        self.canvas.set_path_segments(segments)
        self._update_well_preview()

    # ── Layer Selector (Task 4) ────────────────────────────────────

    def _update_layer_combo(self):
        """Update layer selector from current job."""
        self.layer_combo.clear()
        self.layer_combo.addItem("All Layers", -1)
        if self.print_manager.job:
            boundaries = self.print_manager.job.get_layer_boundaries()
            self.canvas.set_layer_boundaries(boundaries)
            for i, (_, label) in enumerate(boundaries):
                self.layer_combo.addItem(f"Layer {i + 1}", i)

    def _on_layer_changed(self):
        layer = self.layer_combo.currentData()
        if layer is not None:
            self.canvas.set_active_layer(layer)

    # ── Execution Control ──────────────────────────────────────────

    def _on_start(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job loaded!")
            return

        if not self.controller.is_xy_connected or not self.controller.is_zp_connected:
            self.progress_label.setText("Connect both stages first!")
            return

        self.print_manager.job.settings = self._get_settings()
        self.print_manager.start()

    def _on_pause(self):
        if self.print_manager.state == PrintState.PAUSED:
            self.print_manager.resume()
            self.btn_pause.setText("⏸ Pause")
        else:
            self.print_manager.pause()
            self.btn_pause.setText("▶ Resume")

    def _on_abort(self):
        self.print_manager.abort()
        if self.print_queue.is_running:
            self.print_queue.abort()

    def _on_progress(self, step, total, message):
        if total > 0:
            percent = int(step / total * 100)
            self.progress_bar.setValue(percent)
            self.progress_bar.setFormat(f"{step}/{total} ({percent}%)")

            # Task 4: Update canvas progress
            self.canvas.set_progress(step, total)

        self.progress_label.setText(message)

    def _on_state_changed(self, state):
        state_labels = {
            PrintState.IDLE: ("Status: Idle", "#cdd6f4"),
            PrintState.RUNNING: ("Status: Printing...", "#a6e3a1"),
            PrintState.PAUSED: ("Status: Paused", "#f9e2af"),
            PrintState.COMPLETED: ("Status: Complete!", "#a6e3a1"),
            PrintState.ABORTED: ("Status: Aborted", "#f38ba8"),
            PrintState.ERROR: ("Status: Error!", "#f38ba8"),
        }
        text, color = state_labels.get(state, ("Unknown", "#cdd6f4"))
        self.progress_label.setText(text)
        self.progress_label.setStyleSheet(f"color: {color};")

        if state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            self.canvas.clear_current_position()
            self.canvas.set_progress(0, 0)
            self.btn_pause.setText("⏸ Pause")

    # ── Queue Controls (Task 6) ────────────────────────────────────

    def _add_to_queue(self):
        if self.print_manager.job:
            job = self.print_manager.job
            self.print_queue.add_job(job)
            self.queue_list.addItem(f"{job.name} ({job.total_steps} cmds)")
            self.queue_progress_label.setText(f"Queue: {self.print_queue.total_jobs} jobs")

    def _remove_from_queue(self):
        row = self.queue_list.currentRow()
        if row >= 0:
            self.print_queue.remove_job(row)
            self.queue_list.takeItem(row)
            self.queue_progress_label.setText(f"Queue: {self.print_queue.total_jobs} jobs")

    def _clear_queue(self):
        self.print_queue.clear()
        self.queue_list.clear()
        self.queue_progress_label.setText("Queue cleared")

    def _start_queue(self):
        if self.print_queue.total_jobs == 0:
            self.queue_progress_label.setText("Queue is empty!")
            return
        if not self.controller.is_xy_connected or not self.controller.is_zp_connected:
            self.queue_progress_label.setText("Connect both stages first!")
            return
        self.print_queue.start_all()

    def _on_queue_progress(self, job_idx, total_jobs, job_name):
        self.queue_progress_label.setText(f"Job {job_idx}/{total_jobs}: {job_name}")

    def _on_queue_completed(self):
        self.queue_progress_label.setText("Queue completed!")

    # ── Live Update (Task 4) ───────────────────────────────────────

    def update_data(self):
        """
        Called by the main window's update timer.
        During active printing, updates the canvas with current position.
        """
        if self.print_manager.state == PrintState.RUNNING:
            pos = self.controller.get_xy_position(cached=True)
            if pos[0] is not None:
                # Convert from machine coords to zero-ref coords
                zero_x = pos[0] - self.controller.zero_position["x"]
                zero_y = pos[1] - self.controller.zero_position["y"]
                self.canvas.set_current_position(zero_x, zero_y)
