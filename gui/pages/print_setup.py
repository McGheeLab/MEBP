"""
Print Setup Page — Job loading, preview, configure, execute, queue.

PyDracula layout:
    Main content  = Source tabs (File / Well Plate / Pattern) + 2D Path Canvas
    Context panel = Print Settings, Execution Controls, Print Queue

All original functionality preserved:
- File loading (JSON / G-code)
- Well plate generation with multi-well selection
- Pattern generation (Line, Meander, Spiral, Grid)
- Full print settings including multi-material
- Live 2D preview with progress coloring
- Start / Pause / Abort / Resume
- G-code export + JSON save
- Print queue with drag-reorder
- Layer filtering
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QProgressBar, QFrame,
    QCheckBox, QListWidget, QListWidgetItem, QAbstractItemView,
    QSizePolicy,
)
from PySide6.QtCore import Qt, QRectF, QPointF, Signal, QObject
from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QFont

from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintSettings, PrintState, PrintQueue,
    build_well_plate_job, save_print_job, export_gcode,
)
from SupportClasses.WellPlate import (
    WellPlate, PLATE_DEFINITIONS,
    generate_meander_path, generate_spiral_path, generate_line_path,
    generate_grid_path,
)
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
# 2D Path Preview Canvas
# ═══════════════════════════════════════════════════════════════════

class PathPreviewCanvas(QWidget):
    """
    2D canvas rendering print toolpath with live progress.

    Features:
    - Travel moves (dashed gray) + print moves (solid per-pump color)
    - Well plate outlines
    - Current position marker during printing
    - Completed vs remaining dual-color rendering
    - Layer filtering
    """

    PUMP_COLORS = {
        "P1": QColor("#a6e3a1"),  # green
        "P2": QColor("#89b4fa"),  # blue
        "P3": QColor("#f9e2af"),  # yellow
    }
    COMPLETED_COLOR = QColor("#a6e3a1")
    REMAINING_COLOR = QColor(166, 227, 161, 80)
    TRAVEL_DONE_COLOR = QColor(100, 100, 100, 100)
    TRAVEL_TODO_COLOR = QColor(100, 100, 100, 40)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)

        self._segments: list[dict] = []
        self._wells: list[dict] = []
        self._margin = 40
        self._show_grid = True

        # Live visualization state
        self._current_pos: tuple | None = None
        self._completed_step: int = 0
        self._total_steps: int = 0
        self._active_layer: int = -1
        self._layer_boundaries: list[tuple[int, str]] = []

    # ── public setters ─────────────────────────────────────────────

    def set_path_segments(self, segments: list[dict]):
        self._segments = segments
        self.update()

    def set_wells(self, wells: list[dict]):
        self._wells = wells
        self.update()

    def set_current_position(self, x: float, y: float):
        self._current_pos = (x, y)
        self.update()

    def clear_current_position(self):
        self._current_pos = None
        self.update()

    def set_progress(self, completed_step: int, total_steps: int):
        self._completed_step = completed_step
        self._total_steps = total_steps
        self.update()

    def set_layer_boundaries(self, boundaries: list[tuple[int, str]]):
        self._layer_boundaries = boundaries

    def set_active_layer(self, layer: int):
        self._active_layer = layer
        self.update()

    def clear(self):
        self._segments = []
        self._wells = []
        self._current_pos = None
        self._completed_step = 0
        self._total_steps = 0
        self.update()

    # ── geometry helpers ───────────────────────────────────────────

    def _get_bounds(self):
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
        m = 2
        return QRectF(min(all_x) - m, min(all_y) - m,
                       max(all_x) - min(all_x) + 2 * m,
                       max(all_y) - min(all_y) + 2 * m)

    def _transform(self, x, y, bounds, w, h):
        if bounds.width() == 0 or bounds.height() == 0:
            return self._margin + w / 2, self._margin + h / 2
        sx = (w - 2 * self._margin) / bounds.width()
        sy = (h - 2 * self._margin) / bounds.height()
        scale = min(sx, sy)
        cx = (x - bounds.x()) * scale + self._margin
        cy = (y - bounds.y()) * scale + self._margin
        uw = bounds.width() * scale
        uh = bounds.height() * scale
        cx += (w - 2 * self._margin - uw) / 2
        cy += (h - 2 * self._margin - uh) / 2
        return cx, cy

    # ── painting ───────────────────────────────────────────────────

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        bounds = self._get_bounds()

        # Background
        painter.fillRect(0, 0, w, h, QColor(COLORS["crust"]))

        # Grid
        if self._show_grid:
            self._draw_grid(painter, bounds, w, h)

        # Origin marker
        ox, oy = self._transform(0, 0, bounds, w, h)
        painter.setPen(QPen(QColor(COLORS["red"]), 2))
        painter.drawLine(int(ox - 8), int(oy), int(ox + 8), int(oy))
        painter.drawLine(int(ox), int(oy - 8), int(ox), int(oy + 8))

        # Wells
        for well in self._wells:
            wx, wy = self._transform(well["x"], well["y"], bounds, w, h)
            r = well.get("diameter", 6) / 2
            scale = min(
                (w - 2 * self._margin) / max(bounds.width(), 0.001),
                (h - 2 * self._margin) / max(bounds.height(), 0.001),
            )
            pr = r * scale
            painter.setPen(QPen(QColor(COLORS["surface2"]), 1))
            painter.setBrush(QBrush(QColor(88, 91, 112, 30)))
            painter.drawEllipse(QPointF(wx, wy), pr, pr)
            painter.setPen(QColor(COLORS["overlay0"]))
            painter.setFont(QFont("Consolas", 7))
            painter.drawText(int(wx - 10), int(wy - pr - 3),
                             well.get("name", ""))

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
                x1, y1 = self._transform(points[i][0], points[i][1],
                                          bounds, w, h)
                x2, y2 = self._transform(points[i + 1][0], points[i + 1][1],
                                          bounds, w, h)
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            cmd_index += 1

        # Current position marker
        if self._current_pos:
            cx, cy = self._transform(self._current_pos[0],
                                      self._current_pos[1], bounds, w, h)
            pulse = 6 + (self._completed_step % 4)
            painter.setPen(QPen(QColor(COLORS["red"]), 2))
            painter.setBrush(QBrush(QColor(243, 139, 168, 120)))
            painter.drawEllipse(QPointF(cx, cy), pulse, pulse)
            painter.setPen(QPen(QColor(COLORS["red"]), 1))
            painter.drawLine(int(cx - 12), int(cy), int(cx + 12), int(cy))
            painter.drawLine(int(cx), int(cy - 12), int(cx), int(cy + 12))

        painter.end()

    def _draw_grid(self, painter, bounds, w, h):
        painter.setPen(QPen(QColor(COLORS["surface0"]), 1,
                            Qt.PenStyle.DotLine))
        gsx = max(1, int(bounds.width() / 10))
        for gx in range(int(bounds.x()),
                        int(bounds.x() + bounds.width()), gsx):
            x, _ = self._transform(gx, 0, bounds, w, h)
            painter.drawLine(int(x), 0, int(x), h)
        gsy = max(1, int(bounds.height() / 10))
        for gy in range(int(bounds.y()),
                        int(bounds.y() + bounds.height()), gsy):
            _, y = self._transform(0, gy, bounds, w, h)
            painter.drawLine(0, int(y), w, int(y))


# ═══════════════════════════════════════════════════════════════════
# Print Setup Page
# ═══════════════════════════════════════════════════════════════════

class PrintSetupPage(QWidget):
    """
    Print setup: file loading, well plate generation, pattern generation,
    settings, execution, and live preview.

    Interface contract:
        get_page_title()     → str
        get_context_widget() → QWidget  (settings + execution + queue)
        on_status_update()   → called by MainWindow timer
        resume_print(data)   → called from MainWindow resume dialog
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

        self._current_well_plate = None
        self._context_widget = None

        self._setup_ui()

    # ════════════════════════════════════════════════════════════════
    #  PAGE INTERFACE
    # ════════════════════════════════════════════════════════════════

    def get_page_title(self) -> str:
        return "Print Setup"

    def on_status_update(self):
        """Called by MainWindow timer (~300 ms). Updates live canvas."""
        if self.print_manager.state == PrintState.RUNNING:
            pos = self.controller.get_xy_position(cached=True)
            if pos[0] is not None:
                zero_x = pos[0] - self.controller.zero_position["x"]
                zero_y = pos[1] - self.controller.zero_position["y"]
                self.canvas.set_current_position(zero_x, zero_y)

    # legacy alias used by original codebase
    def update_data(self):
        self.on_status_update()

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL  — Print Settings + Execution + Queue
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        lay = QVBoxLayout(ctx)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)

        # ── Print Settings section ───────────────────────────────
        lbl = QLabel("Print Settings")
        lbl.setObjectName("contextSectionLabel")
        lay.addWidget(lbl)

        grid = QGridLayout()
        grid.setSpacing(3)
        row = 0

        def _add_dspin(label_text, attr, min_v, max_v, default,
                       decimals=1, suffix=""):
            nonlocal row
            gl = QLabel(label_text)
            gl.setObjectName("contextLabel")
            grid.addWidget(gl, row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setValue(default)
            spin.setDecimals(decimals)
            if suffix:
                spin.setSuffix(f" {suffix}")
            spin.setMaximumHeight(24)
            grid.addWidget(spin, row, 1)
            setattr(self, f"setting_{attr}", spin)
            row += 1

        _add_dspin("Travel Z:", "travel_z", 0, 50, 5.0, 2, "mm")
        _add_dspin("Print Z:", "print_z", -5, 50, 0.1, 3, "mm")
        _add_dspin("Layer H:", "layer_h", 0.01, 5, 0.1, 3, "mm")

        gl = QLabel("Layers:")
        gl.setObjectName("contextLabel")
        grid.addWidget(gl, row, 0)
        self.setting_layers = QSpinBox()
        self.setting_layers.setRange(1, 999)
        self.setting_layers.setValue(1)
        self.setting_layers.setMaximumHeight(24)
        grid.addWidget(self.setting_layers, row, 1)
        row += 1

        _add_dspin("Print Spd:", "print_speed", 1, 10000, 200, 0, "mm/m")
        _add_dspin("Z Feed:", "z_feed", 1, 1000, 60, 0, "mm/m")
        _add_dspin("Pump Feed:", "pump_feed", 1, 500, 30, 0, "mm/m")
        _add_dspin("Flow Rate:", "flow_rate", 0.001, 10, 0.01, 3, "mm/mm")

        # Pump selector
        gl = QLabel("Pump:")
        gl.setObjectName("contextLabel")
        grid.addWidget(gl, row, 0)
        self.pump_combo = QComboBox()
        self.pump_combo.addItems(["P1", "P2", "P3"])
        self.pump_combo.setMaximumHeight(24)
        grid.addWidget(self.pump_combo, row, 1)
        row += 1

        lay.addLayout(grid)

        # Multi-material toggle
        self.chk_multi_material = QCheckBox("Multi-material")
        self.chk_multi_material.toggled.connect(self._on_multi_material_toggled)
        lay.addWidget(self.chk_multi_material)

        self.multi_material_widget = QWidget()
        mm_lay = QVBoxLayout(self.multi_material_widget)
        mm_lay.setContentsMargins(0, 0, 0, 0)
        mm_lay.setSpacing(2)
        self.mm_mode_combo = QComboBox()
        self.mm_mode_combo.addItems(
            ["Per-layer (P1→P2→P3)", "Alternate wells"])
        self.mm_mode_combo.setMaximumHeight(24)
        mm_lay.addWidget(self.mm_mode_combo)
        pump_row = QHBoxLayout()
        self.chk_mm_p1 = QCheckBox("P1"); self.chk_mm_p1.setChecked(True)
        self.chk_mm_p2 = QCheckBox("P2"); self.chk_mm_p2.setChecked(True)
        self.chk_mm_p3 = QCheckBox("P3")
        pump_row.addWidget(self.chk_mm_p1)
        pump_row.addWidget(self.chk_mm_p2)
        pump_row.addWidget(self.chk_mm_p3)
        mm_lay.addLayout(pump_row)
        self.multi_material_widget.setVisible(False)
        lay.addWidget(self.multi_material_widget)

        # Retraction / prime / settle
        grid2 = QGridLayout()
        grid2.setSpacing(3)
        r2 = 0

        def _add_dspin2(label_text, attr, min_v, max_v, default,
                        decimals=1, suffix=""):
            nonlocal r2
            gl2 = QLabel(label_text)
            gl2.setObjectName("contextLabel")
            grid2.addWidget(gl2, r2, 0)
            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setValue(default)
            spin.setDecimals(decimals)
            if suffix:
                spin.setSuffix(f" {suffix}")
            spin.setMaximumHeight(24)
            grid2.addWidget(spin, r2, 1)
            setattr(self, f"setting_{attr}", spin)
            r2 += 1

        _add_dspin2("Retract:", "retract", 0, 10, 0.0, 2, "mm")
        _add_dspin2("Prime:", "prime", 0, 10, 0.0, 2, "mm")
        _add_dspin2("Settle:", "settle", 0, 10, 0.0, 1, "s")
        lay.addLayout(grid2)

        # ── Execution Controls ───────────────────────────────────
        exec_lbl = QLabel("Execution")
        exec_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(exec_lbl)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(3)

        self.btn_start = QPushButton("▶ Start")
        self.btn_start.setObjectName("successBtn")
        self.btn_start.setToolTip("Start Print")
        self.btn_start.setMaximumHeight(28)
        self.btn_start.clicked.connect(self._on_start)
        btn_row.addWidget(self.btn_start)

        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setToolTip("Pause / Resume")
        self.btn_pause.setMaximumHeight(28)
        self.btn_pause.clicked.connect(self._on_pause)
        btn_row.addWidget(self.btn_pause)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setObjectName("dangerBtn")
        self.btn_abort.setToolTip("Abort Print")
        self.btn_abort.setMaximumHeight(28)
        self.btn_abort.clicked.connect(self._on_abort)
        btn_row.addWidget(self.btn_abort)
        lay.addLayout(btn_row)

        # Export / Save buttons
        btn_row2 = QHBoxLayout()
        btn_row2.setSpacing(3)
        btn_gcode = QPushButton("Export G-code")
        btn_gcode.setObjectName("flatBtn")
        btn_gcode.setToolTip("Export current job to G-code")
        btn_gcode.setMaximumHeight(24)
        btn_gcode.clicked.connect(self._export_gcode)
        btn_row2.addWidget(btn_gcode)
        btn_save = QPushButton("Save JSON")
        btn_save.setObjectName("flatBtn")
        btn_save.setToolTip("Save job as JSON")
        btn_save.setMaximumHeight(24)
        btn_save.clicked.connect(self._save_job_json)
        btn_row2.addWidget(btn_save)
        lay.addLayout(btn_row2)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setFormat("%p%")
        self.progress_bar.setMaximumHeight(16)
        lay.addWidget(self.progress_bar)

        self.progress_label = QLabel("Idle")
        self.progress_label.setObjectName("contextLabel")
        self.progress_label.setWordWrap(True)
        lay.addWidget(self.progress_label)

        # ── Print Queue ──────────────────────────────────────────
        q_lbl = QLabel("Print Queue")
        q_lbl.setObjectName("contextSectionLabel")
        lay.addWidget(q_lbl)

        self.queue_list = QListWidget()
        self.queue_list.setMaximumHeight(80)
        self.queue_list.setDragDropMode(
            QAbstractItemView.DragDropMode.InternalMove)
        lay.addWidget(self.queue_list)

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
        lay.addLayout(q_btn_row)

        btn_start_q = QPushButton("▶ Start Queue")
        btn_start_q.setObjectName("accentBtn")
        btn_start_q.setMaximumHeight(28)
        btn_start_q.clicked.connect(self._start_queue)
        lay.addWidget(btn_start_q)

        self.queue_progress_label = QLabel("")
        self.queue_progress_label.setObjectName("dimLabel")
        lay.addWidget(self.queue_progress_label)

        lay.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTENT — Source Tabs + Canvas
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(6)
        layout.setContentsMargins(12, 8, 12, 8)

        # Canvas (create before tabs — tabs may trigger preview)
        self.canvas = PathPreviewCanvas()

        # Source tabs
        self.source_tabs = QTabWidget()
        self.source_tabs.setMaximumHeight(230)
        self._build_file_tab()
        self._build_wellplate_tab()
        self._build_pattern_tab()
        layout.addWidget(self.source_tabs)

        # Layer selector row
        layer_row = QHBoxLayout()
        layer_row.addWidget(QLabel("Layer:"))
        self.layer_combo = QComboBox()
        self.layer_combo.addItem("All Layers", -1)
        self.layer_combo.currentIndexChanged.connect(self._on_layer_changed)
        layer_row.addWidget(self.layer_combo, stretch=1)
        layer_row.addStretch(2)
        layout.addLayout(layer_row)

        # Canvas inside styled card
        canvas_frame = QFrame()
        canvas_frame.setObjectName("cardFrame")
        cf_lay = QVBoxLayout(canvas_frame)
        cf_lay.setContentsMargins(4, 4, 4, 4)
        cf_lay.addWidget(self.canvas)
        layout.addWidget(canvas_frame, stretch=1)

        outer.addWidget(container)

    # ── File Tab ──────────────────────────────────────────────────

    def _build_file_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(4)

        browse_row = QHBoxLayout()
        self.file_label = QLabel("No file loaded")
        self.file_label.setStyleSheet(f"color: {COLORS['subtext0']};")
        browse_row.addWidget(self.file_label, stretch=1)
        btn = QPushButton("Browse...")
        btn.clicked.connect(self._browse_file)
        browse_row.addWidget(btn)
        layout.addLayout(browse_row)

        self.job_info_label = QLabel("")
        self.job_info_label.setWordWrap(True)
        self.job_info_label.setStyleSheet(f"color: {COLORS['overlay0']};")
        layout.addWidget(self.job_info_label)
        layout.addStretch()
        self.source_tabs.addTab(tab, "📁 File")

    # ── Well Plate Tab ────────────────────────────────────────────

    def _build_wellplate_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(4)

        fmt_row = QHBoxLayout()
        fmt_row.addWidget(QLabel("Plate:"))
        self.plate_combo = QComboBox()
        for fmt in PLATE_DEFINITIONS:
            desc = PLATE_DEFINITIONS[fmt]["description"]
            self.plate_combo.addItem(f"{fmt}-well ({desc})", fmt)
        self.plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        fmt_row.addWidget(self.plate_combo)
        layout.addLayout(fmt_row)

        well_row = QHBoxLayout()
        self.well_list = QListWidget()
        self.well_list.setSelectionMode(
            QAbstractItemView.SelectionMode.MultiSelection)
        self.well_list.setMaximumHeight(100)
        well_row.addWidget(self.well_list)

        btn_col = QVBoxLayout()
        btn_all = QPushButton("All")
        btn_all.setMaximumHeight(24)
        btn_all.clicked.connect(self._select_all_wells)
        btn_col.addWidget(btn_all)
        btn_none = QPushButton("None")
        btn_none.setMaximumHeight(24)
        btn_none.clicked.connect(self._select_no_wells)
        btn_col.addWidget(btn_none)
        btn_col.addStretch()
        well_row.addLayout(btn_col)
        layout.addLayout(well_row)

        btn_gen = QPushButton("Generate Well Plate Job")
        btn_gen.setObjectName("accentBtn")
        btn_gen.clicked.connect(self._generate_well_plate_job)
        layout.addWidget(btn_gen)
        self.source_tabs.addTab(tab, "🧫 Well Plate")
        self._on_plate_changed()

    # ── Pattern Tab ───────────────────────────────────────────────

    def _build_pattern_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(4)

        grid = QGridLayout()
        grid.setSpacing(3)
        row = 0

        grid.addWidget(QLabel("Pattern:"), row, 0)
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(["Line", "Meander", "Spiral", "Grid"])
        self.pattern_combo.currentIndexChanged.connect(
            self._update_pattern_preview)
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
            setattr(self, attr, spin)
            row += 1

        layout.addLayout(grid)

        btn_preview = QPushButton("Preview Pattern")
        btn_preview.clicked.connect(self._update_pattern_preview)
        layout.addWidget(btn_preview)
        layout.addStretch()
        self.source_tabs.addTab(tab, "🔷 Pattern")

    # ════════════════════════════════════════════════════════════════
    #  SETTINGS HELPERS
    # ════════════════════════════════════════════════════════════════

    def _on_multi_material_toggled(self, checked):
        self.multi_material_widget.setVisible(checked)
        self.pump_combo.setEnabled(not checked)

    def _get_settings(self) -> PrintSettings:
        """Read current settings from context panel into PrintSettings."""
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
            retract_amounts={"P1": retract, "P2": retract, "P3": retract},
            prime_amounts={"P1": prime, "P2": prime, "P3": prime},
        )

    def _get_multi_material_params(self) -> dict:
        if not self.chk_multi_material.isChecked():
            return {}
        pumps = []
        if self.chk_mm_p1.isChecked(): pumps.append("P1")
        if self.chk_mm_p2.isChecked(): pumps.append("P2")
        if self.chk_mm_p3.isChecked(): pumps.append("P3")
        mode = self.mm_mode_combo.currentIndex()
        if mode == 0:  # Per-layer
            ppl = {}
            for i in range(self.setting_layers.value()):
                ppl[str(i + 1)] = pumps[i % len(pumps)] if pumps else "P1"
            return {"pump_per_layer": ppl}
        else:  # Alternate wells
            return {"pump_sequence": pumps if pumps else ["P1"]}

    # ════════════════════════════════════════════════════════════════
    #  FILE LOADING
    # ════════════════════════════════════════════════════════════════

    def _browse_file(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Open Print File", "",
            "Print Files (*.json *.gcode *.gco);;"
            "JSON (*.json);;G-Code (*.gcode *.gco);;All (*)")
        if filepath:
            try:
                self.print_manager.load_file(filepath)
                job = self.print_manager.job
                fname = filepath.split("/")[-1].split("\\")[-1]
                self.file_label.setText(fname)
                self.job_info_label.setText(
                    f"Job: {job.name}\n"
                    f"{job.description}\n"
                    f"Commands: {job.total_steps}")
                self._refresh_preview()
                self._update_layer_combo()
            except Exception as e:
                self.file_label.setText(f"Error: {e}")
                logger.error(f"Failed to load file: {e}")

    # ════════════════════════════════════════════════════════════════
    #  WELL PLATE
    # ════════════════════════════════════════════════════════════════

    def _on_plate_changed(self):
        fmt = self.plate_combo.currentData()
        if fmt is None:
            return
        self._current_well_plate = WellPlate.from_format(fmt)
        self.well_list.clear()
        for well in self._current_well_plate.get_all_wells():
            self.well_list.addItem(QListWidgetItem(well.name))
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
            f"Job: {job.name}\n{job.description}\n"
            f"Commands: {job.total_steps}")
        self._refresh_preview()
        self._update_layer_combo()
        self.progress_label.setText(
            f"Generated: {len(selected)} wells, "
            f"{settings.num_layers} layers")

    # ════════════════════════════════════════════════════════════════
    #  PATTERN GENERATION
    # ════════════════════════════════════════════════════════════════

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
        self.canvas.set_path_segments([{"type": "print", "points": points}])
        self._update_well_preview()

    def _refresh_preview(self):
        if not self.print_manager.job:
            self.canvas.clear()
            return
        self.canvas.set_path_segments(
            self.print_manager.job.get_path_segments())
        self._update_well_preview()

    def _update_layer_combo(self):
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

    # ════════════════════════════════════════════════════════════════
    #  EXECUTION CONTROL
    # ════════════════════════════════════════════════════════════════

    def _on_start(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job loaded!")
            return
        if (not self.controller.is_xy_connected
                or not self.controller.is_zp_connected):
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
            self.canvas.set_progress(step, total)
        self.progress_label.setText(message)

    def _on_state_changed(self, state):
        labels = {
            PrintState.IDLE: ("Idle", COLORS["text"]),
            PrintState.RUNNING: ("Printing...", COLORS["green"]),
            PrintState.PAUSED: ("Paused", COLORS["yellow"]),
            PrintState.COMPLETED: ("Complete!", COLORS["green"]),
            PrintState.ABORTED: ("Aborted", COLORS["red"]),
            PrintState.ERROR: ("Error!", COLORS["red"]),
        }
        text, color = labels.get(state, ("Unknown", COLORS["text"]))
        self.progress_label.setText(text)
        self.progress_label.setStyleSheet(f"color: {color};")
        if state in (PrintState.COMPLETED, PrintState.ABORTED,
                     PrintState.ERROR):
            self.canvas.clear_current_position()
            self.canvas.set_progress(0, 0)
            self.btn_pause.setText("⏸ Pause")

    # ════════════════════════════════════════════════════════════════
    #  EXPORT / SAVE
    # ════════════════════════════════════════════════════════════════

    def _export_gcode(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job to export!")
            return
        default = f"{self.print_manager.job.name.replace(' ', '_')}.gcode"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export G-code", default,
            "G-code Files (*.gcode *.gco);;All (*)")
        if path:
            export_gcode(self.print_manager.job, path)
            self.progress_label.setText(f"Exported: {path}")
            logger.info(f"G-code exported: {path}")

    def _save_job_json(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job to save!")
            return
        default = f"{self.print_manager.job.name.replace(' ', '_')}.json"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Print Job", default,
            "JSON Files (*.json);;All (*)")
        if path:
            self.print_manager.job.settings = self._get_settings()
            save_print_job(self.print_manager.job, path)
            self.progress_label.setText(f"Saved: {path}")

    # ════════════════════════════════════════════════════════════════
    #  PRINT RESUME
    # ════════════════════════════════════════════════════════════════

    def resume_print(self, resume_data: dict):
        """Resume a print from saved progress data."""
        job = resume_data["job"]
        self.print_manager.load_job(job)
        self._refresh_preview()
        if (not self.controller.is_xy_connected
                or not self.controller.is_zp_connected):
            self.progress_label.setText(
                "Connect stages first, then resume")
            return
        self.print_manager.resume_from_saved(resume_data)
        step = resume_data["current_step"]
        self.progress_label.setText(
            f"Resuming '{job.name}' from step {step}/{job.total_steps}")

    # ════════════════════════════════════════════════════════════════
    #  QUEUE CONTROLS
    # ════════════════════════════════════════════════════════════════

    def _add_to_queue(self):
        if self.print_manager.job:
            job = self.print_manager.job
            self.print_queue.add_job(job)
            self.queue_list.addItem(
                f"{job.name} ({job.total_steps} cmds)")
            self.queue_progress_label.setText(
                f"Queue: {self.print_queue.total_jobs} jobs")

    def _remove_from_queue(self):
        row = self.queue_list.currentRow()
        if row >= 0:
            self.print_queue.remove_job(row)
            self.queue_list.takeItem(row)
            self.queue_progress_label.setText(
                f"Queue: {self.print_queue.total_jobs} jobs")

    def _clear_queue(self):
        self.print_queue.clear()
        self.queue_list.clear()
        self.queue_progress_label.setText("Queue cleared")

    def _start_queue(self):
        if self.print_queue.total_jobs == 0:
            self.queue_progress_label.setText("Queue is empty!")
            return
        if (not self.controller.is_xy_connected
                or not self.controller.is_zp_connected):
            self.queue_progress_label.setText("Connect stages first!")
            return
        self.print_queue.start_all()

    def _on_queue_progress(self, job_idx, total_jobs, job_name):
        self.queue_progress_label.setText(
            f"Job {job_idx}/{total_jobs}: {job_name}")

    def _on_queue_completed(self):
        self.queue_progress_label.setText("Queue completed!")
