"""
Print Setup Page - Load, preview, configure, and execute print jobs.

Layout:
┌─────────────────────────────────────────────────┐
│  [File / Well Plate / Pattern tabs]             │
│  ┌─────────────────────┬───────────────────────┐ │
│  │                     │  Settings Panel       │ │
│  │   2D Path Preview   │  ─────────────────    │ │
│  │   Canvas            │  Print Settings       │ │
│  │                     │  Execution Controls   │ │
│  │                     │  Progress             │ │
│  └─────────────────────┴───────────────────────┘ │
└─────────────────────────────────────────────────┘
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
    PrintManager, PrintJob, PrintSettings, PrintState,
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
# 2D Path Preview Canvas
# ═══════════════════════════════════════════════════════════════════

class PathPreviewCanvas(QWidget):
    """
    2D canvas that renders the print toolpath.
    Shows travel moves (dashed gray) and print moves (solid colored).
    Optionally shows well plate outlines.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self._segments: list[dict] = []    # {"type": "travel"|"print", "points": [...]}
        self._wells: list[dict] = []       # {"name": str, "x": float, "y": float, "diameter": float}
        self._margin = 40
        self._show_grid = True

    def set_path_segments(self, segments: list[dict]):
        """Set path segments for rendering."""
        self._segments = segments
        self.update()

    def set_wells(self, wells: list[dict]):
        """Set well positions for rendering."""
        self._wells = wells
        self.update()

    def clear(self):
        self._segments = []
        self._wells = []
        self.update()

    def _get_bounds(self):
        """Calculate bounding box of all geometry."""
        all_x, all_y = [], []

        for seg in self._segments:
            for pt in seg.get("points", []):
                all_x.append(pt[0])
                all_y.append(pt[1])

        for well in self._wells:
            r = well.get("diameter", 0) / 2
            all_x.extend([well["x"] - r, well["x"] + r])
            all_y.extend([well["y"] - r, well["y"] + r])

        if not all_x or not all_y:
            return -10, -10, 10, 10

        padding = 5
        return (min(all_x) - padding, min(all_y) - padding,
                max(all_x) + padding, max(all_y) + padding)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor("#11111b"))

        w = self.width()
        h = self.height()
        margin = self._margin

        # Calculate transform
        min_x, min_y, max_x, max_y = self._get_bounds()
        data_w = max(max_x - min_x, 1)
        data_h = max(max_y - min_y, 1)

        draw_w = w - 2 * margin
        draw_h = h - 2 * margin

        scale = min(draw_w / data_w, draw_h / data_h)
        offset_x = margin + (draw_w - data_w * scale) / 2
        offset_y = margin + (draw_h - data_h * scale) / 2

        def to_screen(x, y):
            sx = offset_x + (x - min_x) * scale
            sy = offset_y + (data_h - (y - min_y)) * scale  # Flip Y
            return QPointF(sx, sy)

        # Grid
        if self._show_grid:
            self._draw_grid(painter, min_x, min_y, max_x, max_y, to_screen, scale)

        # Well outlines
        for well in self._wells:
            cx, cy = to_screen(well["x"], well["y"]).x(), to_screen(well["x"], well["y"]).y()
            r = well.get("diameter", 0) / 2 * scale

            painter.setPen(QPen(QColor("#585b70"), 1.5))
            painter.setBrush(QBrush(QColor(88, 91, 112, 30)))
            painter.drawEllipse(QPointF(cx, cy), r, r)

            # Well label
            painter.setPen(QColor("#6c7086"))
            painter.setFont(QFont("Consolas", max(7, int(r / 3))))
            text_rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
            painter.drawText(text_rect, Qt.AlignmentFlag.AlignCenter, well.get("name", ""))

        # Path segments
        for seg in self._segments:
            points = seg.get("points", [])
            if len(points) < 2:
                continue

            if seg["type"] == "print":
                pen = QPen(QColor("#a6e3a1"), 2.0)
            else:
                pen = QPen(QColor("#6c7086"), 1.0, Qt.PenStyle.DashLine)

            painter.setPen(pen)

            path = QPainterPath()
            start = to_screen(points[0][0], points[0][1])
            path.moveTo(start)

            for pt in points[1:]:
                p = to_screen(pt[0], pt[1])
                path.lineTo(p)

            painter.drawPath(path)

        # Start point marker
        all_pts = []
        for seg in self._segments:
            all_pts.extend(seg.get("points", []))
        if all_pts:
            start_pt = to_screen(all_pts[0][0], all_pts[0][1])
            painter.setPen(QPen(QColor("#89b4fa"), 2))
            painter.setBrush(QBrush(QColor("#89b4fa")))
            painter.drawEllipse(start_pt, 5, 5)

            # End point
            end_pt = to_screen(all_pts[-1][0], all_pts[-1][1])
            painter.setPen(QPen(QColor("#f38ba8"), 2))
            painter.setBrush(QBrush(QColor("#f38ba8")))
            painter.drawEllipse(end_pt, 5, 5)

        # Origin marker
        origin = to_screen(0, 0)
        painter.setPen(QPen(QColor("#f9e2af"), 1.5))
        size = 8
        painter.drawLine(
            QPointF(origin.x() - size, origin.y()),
            QPointF(origin.x() + size, origin.y()),
        )
        painter.drawLine(
            QPointF(origin.x(), origin.y() - size),
            QPointF(origin.x(), origin.y() + size),
        )

        # Legend
        self._draw_legend(painter, w, h)

        painter.end()

    def _draw_grid(self, painter, min_x, min_y, max_x, max_y, to_screen, scale):
        """Draw background grid."""
        painter.setPen(QPen(QColor("#1e1e2e"), 0.5))

        # Choose grid spacing based on scale
        data_range = max(max_x - min_x, max_y - min_y)
        if data_range > 200:
            spacing = 50
        elif data_range > 50:
            spacing = 10
        elif data_range > 10:
            spacing = 5
        else:
            spacing = 1

        import math
        x_start = math.floor(min_x / spacing) * spacing
        y_start = math.floor(min_y / spacing) * spacing

        x = x_start
        while x <= max_x:
            p1 = to_screen(x, min_y)
            p2 = to_screen(x, max_y)
            painter.drawLine(p1, p2)
            x += spacing

        y = y_start
        while y <= max_y:
            p1 = to_screen(min_x, y)
            p2 = to_screen(max_x, y)
            painter.drawLine(p1, p2)
            y += spacing

    def _draw_legend(self, painter, w, h):
        """Draw a small legend in the corner."""
        painter.setPen(QColor("#6c7086"))
        painter.setFont(QFont("Segoe UI", 8))

        x, y = 8, h - 60

        # Print path
        painter.setPen(QPen(QColor("#a6e3a1"), 2))
        painter.drawLine(QPointF(x, y), QPointF(x + 20, y))
        painter.setPen(QColor("#a6adc8"))
        painter.drawText(QPointF(x + 25, y + 4), "Print")

        y += 16

        # Travel
        painter.setPen(QPen(QColor("#6c7086"), 1, Qt.PenStyle.DashLine))
        painter.drawLine(QPointF(x, y), QPointF(x + 20, y))
        painter.setPen(QColor("#a6adc8"))
        painter.drawText(QPointF(x + 25, y + 4), "Travel")

        y += 16

        # Origin
        painter.setPen(QPen(QColor("#f9e2af"), 1.5))
        painter.drawText(QPointF(x, y + 4), "✚ Origin")


# ═══════════════════════════════════════════════════════════════════
# Signal bridge for thread-safe state updates
# ═══════════════════════════════════════════════════════════════════

class PrintSignalBridge(QObject):
    progress_signal = Signal(int, int, str)   # step, total, message
    state_signal = Signal(object)              # PrintState


# ═══════════════════════════════════════════════════════════════════
# Print Setup Page
# ═══════════════════════════════════════════════════════════════════

class PrintSetupPage(QWidget):
    """Print setup, preview, and execution page."""

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.print_manager = PrintManager(controller)

        # Signal bridge for thread → GUI communication
        self._bridge = PrintSignalBridge()
        self._bridge.progress_signal.connect(self._on_progress)
        self._bridge.state_signal.connect(self._on_state_changed)

        self.print_manager.on_progress = lambda s, t, m: self._bridge.progress_signal.emit(s, t, m)
        self.print_manager.on_state_changed = lambda st: self._bridge.state_signal.emit(st)

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

        # Preview canvas
        canvas_group = QGroupBox("Path Preview")
        canvas_layout = QVBoxLayout(canvas_group)
        self.canvas = PathPreviewCanvas()
        canvas_layout.addWidget(self.canvas)
        left_panel.addWidget(canvas_group, stretch=1)

        main_layout.addLayout(left_panel, stretch=3)

        # ── Right: Settings + Controls ─────────────────────────────
        right_panel = QVBoxLayout()

        self._build_settings_panel(right_panel)
        self._build_execution_panel(right_panel)

        right_panel.addStretch()
        main_layout.addLayout(right_panel, stretch=1)

    # ── Source Tab: File Loading ────────────────────────────────────

    def _build_file_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # File browse row
        browse_row = QHBoxLayout()
        self.file_label = QLabel("No file loaded")
        self.file_label.setStyleSheet("color: #a6adc8;")
        browse_row.addWidget(self.file_label, stretch=1)

        btn_browse = QPushButton("Browse...")
        btn_browse.clicked.connect(self._browse_file)
        browse_row.addWidget(btn_browse)

        layout.addLayout(browse_row)

        # Job info
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

        # Plate format
        format_row = QHBoxLayout()
        format_row.addWidget(QLabel("Plate:"))
        self.plate_combo = QComboBox()
        for fmt in PLATE_DEFINITIONS:
            desc = PLATE_DEFINITIONS[fmt]["description"]
            self.plate_combo.addItem(f"{fmt}-well ({desc})", fmt)
        self.plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        format_row.addWidget(self.plate_combo)
        layout.addLayout(format_row)

        # Well selection
        well_row = QHBoxLayout()

        self.well_list = QListWidget()
        self.well_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)
        self.well_list.setMaximumHeight(120)
        well_row.addWidget(self.well_list)

        # Quick select buttons
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

        # Generate button
        btn_generate = QPushButton("Generate Well Plate Job")
        btn_generate.setObjectName("connectBtn")
        btn_generate.clicked.connect(self._generate_well_plate_job)
        layout.addWidget(btn_generate)

        self.source_tabs.addTab(tab, "🧫 Well Plate")

        # Initialize with first plate
        self._on_plate_changed()

    # ── Source Tab: Pattern ─────────────────────────────────────────

    def _build_pattern_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Pattern type
        pat_row = QHBoxLayout()
        pat_row.addWidget(QLabel("Pattern:"))
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(["Line", "Meander", "Spiral", "Grid"])
        self.pattern_combo.currentIndexChanged.connect(self._update_pattern_preview)
        pat_row.addWidget(self.pattern_combo)
        layout.addLayout(pat_row)

        # Pattern parameters (2-column grid)
        params = QGridLayout()

        params.addWidget(QLabel("Width:"), 0, 0)
        self.pat_width = QDoubleSpinBox()
        self.pat_width.setRange(0.1, 100)
        self.pat_width.setValue(10.0)
        self.pat_width.setSuffix(" mm")
        self.pat_width.valueChanged.connect(self._update_pattern_preview)
        params.addWidget(self.pat_width, 0, 1)

        params.addWidget(QLabel("Height:"), 0, 2)
        self.pat_height = QDoubleSpinBox()
        self.pat_height.setRange(0.1, 100)
        self.pat_height.setValue(10.0)
        self.pat_height.setSuffix(" mm")
        self.pat_height.valueChanged.connect(self._update_pattern_preview)
        params.addWidget(self.pat_height, 0, 3)

        params.addWidget(QLabel("Spacing:"), 1, 0)
        self.pat_spacing = QDoubleSpinBox()
        self.pat_spacing.setRange(0.05, 50)
        self.pat_spacing.setValue(1.0)
        self.pat_spacing.setSuffix(" mm")
        self.pat_spacing.valueChanged.connect(self._update_pattern_preview)
        params.addWidget(self.pat_spacing, 1, 1)

        params.addWidget(QLabel("Angle:"), 1, 2)
        self.pat_angle = QDoubleSpinBox()
        self.pat_angle.setRange(0, 360)
        self.pat_angle.setValue(0)
        self.pat_angle.setSuffix(" °")
        self.pat_angle.valueChanged.connect(self._update_pattern_preview)
        params.addWidget(self.pat_angle, 1, 3)

        layout.addLayout(params)

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

        # Pump selector
        grid.addWidget(QLabel("Pump:"), row, 0)
        self.pump_combo = QComboBox()
        self.pump_combo.addItems(["P1", "P2", "P3"])
        grid.addWidget(self.pump_combo, row, 1)
        row += 1

        add_spin("Retraction:", "retract", 0, 10, 0.0, 2, "mm")
        add_spin("Prime:", "prime", 0, 10, 0.0, 2, "mm")
        add_spin("Settle Delay:", "settle", 0, 10, 0.0, 1, "s")

        parent_layout.addWidget(group)

    def _get_settings(self) -> PrintSettings:
        """Read current settings from UI into a PrintSettings object."""
        return PrintSettings(
            travel_z_height=self.setting_travel_z.value(),
            print_z_height=self.setting_print_z.value(),
            layer_height=self.setting_layer_h.value(),
            num_layers=self.setting_layers.value(),
            print_feedrate=self.setting_print_speed.value(),
            z_feedrate=self.setting_z_feed.value(),
            pump_feedrate=self.setting_pump_feed.value(),
            retract_amount=self.setting_retract.value(),
            prime_amount=self.setting_prime.value(),
            dwell_after_move=self.setting_settle.value(),
        )

    # ── Execution Panel ────────────────────────────────────────────

    def _build_execution_panel(self, parent_layout):
        group = QGroupBox("Print Execution")
        layout = QVBoxLayout(group)

        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setObjectName("headerLabel")
        layout.addWidget(self.status_label)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #45475a;
                border-radius: 4px;
                background-color: #313244;
                text-align: center;
                color: #cdd6f4;
                min-height: 24px;
            }
            QProgressBar::chunk {
                background-color: #89b4fa;
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.progress_bar)

        # Progress message
        self.progress_label = QLabel("")
        self.progress_label.setStyleSheet("color: #a6adc8; font-size: 9pt;")
        self.progress_label.setWordWrap(True)
        layout.addWidget(self.progress_label)

        # Control buttons
        btn_row = QGridLayout()

        self.btn_start = QPushButton("▶ Start Print")
        self.btn_start.setObjectName("connectBtn")
        self.btn_start.setMinimumHeight(40)
        self.btn_start.clicked.connect(self._on_start)
        btn_row.addWidget(self.btn_start, 0, 0)

        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setMinimumHeight(40)
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self._on_pause)
        btn_row.addWidget(self.btn_pause, 0, 1)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setObjectName("disconnectBtn")
        self.btn_abort.setMinimumHeight(40)
        self.btn_abort.setEnabled(False)
        self.btn_abort.clicked.connect(self._on_abort)
        btn_row.addWidget(self.btn_abort, 1, 0, 1, 2)

        layout.addLayout(btn_row)

        # Save job button
        btn_save = QPushButton("💾 Save Job as JSON...")
        btn_save.clicked.connect(self._save_job)
        layout.addWidget(btn_save)

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
        """Get selected well names."""
        return [item.text() for item in self.well_list.selectedItems()]

    def _update_well_preview(self):
        """Update the canvas to show well plate layout."""
        if not self._current_well_plate:
            return

        wells = []
        for well in self._current_well_plate.get_all_wells():
            wells.append({
                "name": well.name,
                "x": well.x,
                "y": well.y,
                "diameter": well.diameter,
            })
        self.canvas.set_wells(wells)

    def _generate_well_plate_job(self):
        """Generate a print job from well plate selection + current pattern."""
        if not self._current_well_plate:
            return

        selected = self._get_selected_wells()
        if not selected:
            self.progress_label.setText("No wells selected!")
            return

        # Get pattern
        pattern_points = self._generate_current_pattern()
        if not pattern_points:
            self.progress_label.setText("No pattern generated!")
            return

        # Get well positions
        well_positions = []
        for name in selected:
            try:
                x, y = self._current_well_plate.get_well_position(name)
                well_positions.append((name, x, y))
            except KeyError:
                pass

        # Build the job
        settings = self._get_settings()
        pump = self.pump_combo.currentText()
        flow_rate = self.setting_flow_rate.value()

        job = build_well_plate_job(
            well_positions=well_positions,
            path_points=pattern_points,
            settings=settings,
            pump=pump,
            flow_rate=flow_rate,
            job_name=f"Well Plate {self._current_well_plate.format}-well",
        )

        self.print_manager.load_job(job)
        self.job_info_label.setText(
            f"Job: {job.name}\n{job.description}\nCommands: {job.total_steps}"
        )
        self._refresh_preview()
        self.progress_label.setText(f"Job generated: {len(selected)} wells, {settings.num_layers} layers")

    # ── Pattern Generation ─────────────────────────────────────────

    def _generate_current_pattern(self) -> list[tuple[float, float]]:
        """Generate pattern points based on current UI settings."""
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
        """Preview the pattern on the canvas."""
        points = self._generate_current_pattern()
        if not points:
            return

        # Show as a single print segment
        segments = [{"type": "print", "points": points}]
        self.canvas.set_path_segments(segments)

        # Also show well plate if applicable
        self._update_well_preview()

    # ── Preview Refresh ────────────────────────────────────────────

    def _refresh_preview(self):
        """Refresh the canvas from the current print job."""
        if not self.print_manager.job:
            self.canvas.clear()
            return

        segments = self.print_manager.job.get_path_segments()
        self.canvas.set_path_segments(segments)

        # If well plate tab, show wells too
        self._update_well_preview()

    # ── Execution Control ──────────────────────────────────────────

    def _on_start(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job loaded!")
            return

        if not self.controller.is_xy_connected or not self.controller.is_zp_connected:
            self.progress_label.setText("Connect both stages first!")
            return

        # Apply settings from UI to job
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

    def _on_progress(self, step, total, message):
        """Handle progress update (runs in GUI thread via signal)."""
        if total > 0:
            percent = int(step / total * 100)
            self.progress_bar.setValue(percent)
            self.progress_bar.setFormat(f"{step}/{total} ({percent}%)")
        self.progress_label.setText(message)

    def _on_state_changed(self, state):
        """Handle state change (runs in GUI thread via signal)."""
        state_labels = {
            PrintState.IDLE: ("Status: Idle", "#cdd6f4"),
            PrintState.RUNNING: ("Status: Printing...", "#a6e3a1"),
            PrintState.PAUSED: ("Status: Paused", "#f9e2af"),
            PrintState.COMPLETED: ("Status: Complete!", "#a6e3a1"),
            PrintState.ABORTED: ("Status: Aborted", "#f38ba8"),
            PrintState.ERROR: ("Status: Error!", "#f38ba8"),
        }

        text, color = state_labels.get(state, ("Status: Unknown", "#cdd6f4"))
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

        is_running = state == PrintState.RUNNING
        is_paused = state == PrintState.PAUSED
        is_active = is_running or is_paused

        self.btn_start.setEnabled(not is_active)
        self.btn_pause.setEnabled(is_active)
        self.btn_abort.setEnabled(is_active)

        if state == PrintState.PAUSED:
            self.btn_pause.setText("▶ Resume")
        else:
            self.btn_pause.setText("⏸ Pause")

        if state == PrintState.COMPLETED:
            self.progress_bar.setValue(100)

    # ── Save Job ───────────────────────────────────────────────────

    def _save_job(self):
        if not self.print_manager.job:
            self.progress_label.setText("No job to save!")
            return

        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Print Job", "",
            "JSON (*.json);;All (*)"
        )
        if filepath:
            try:
                save_print_job(self.print_manager.job, filepath)
                self.progress_label.setText(f"Saved to {filepath}")
            except Exception as e:
                self.progress_label.setText(f"Save error: {e}")

    # ── Periodic Update ────────────────────────────────────────────

    def update_data(self):
        """Called by main window timer."""
        pass  # Progress is handled via signals
