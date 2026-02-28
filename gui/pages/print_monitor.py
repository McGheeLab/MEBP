"""
print_monitor.py — Print Monitor Page for MEBP v7.1.

Dedicated page (6th nav button) for monitoring active prints.

Layout:
┌───────────────────────┬─────────────────────────────────────────┐
│  Full Plate Overview  │  Current Well Detail (L-shaped proj.)   │
│  (miniature, per-well │  ┌────────────────────┬───────────────┐ │
│   progress coloring)  │  │  XY view (large)   │  ZY (tall)    │ │
│                       │  │  needle ✛ path --- │  needle ✛     │ │
│  🟩🟩🟨⬜⬜⬜...      │  ├────────────────────┴───────────────┤ │
│                       │  │  XZ view (wide)                    │ │
│  🟩=done 🟨=active    │  │  path ─── completed  ╌╌╌╌ upcoming│ │
│  ⬜=pending 🔵=ink     │  └───────────────────────────────────┘ │
├───────────────────────┼─────────────────────────────────────────┤
│  Syringe Status       │  Print Progress                        │
│  [P1] [P2] [P3]      │  Job, Well, Layer, Step, Time, ETA     │
│  fill bars + labels   │  [Pause] [Abort]  progress bar         │
│  Needle info          │  Tracking error + controller status    │
└───────────────────────┴─────────────────────────────────────────┘

Session H — Tasks P6.1, P6.2, P6.3, P6.7, P6.8, P6.9, P6.10.
Session I — Tasks P7.4-P7.7, P8.34-P8.36:
  - get_page_title() / get_context_widget() page interface (P8.34)
  - Recording browser in context panel (P7.6)
  - Replay visualization overlay (P7.7)
  - PrintRecorder integration hooks (P7.4, P7.5)
"""

from __future__ import annotations

import logging
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QFrame, QSplitter, QSizePolicy,
    QProgressBar, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QListWidget, QListWidgetItem, QScrollArea,
)
from PySide6.QtCore import Qt, Signal, QTimer, QRectF
from PySide6.QtGui import QColor, QPen, QBrush, QPainter, QFont

from gui.styles import COLORS
from gui.widgets.syringe_display import SyringeStatusPanel
from gui.widgets.trajectory_view import TrajectoryView

from SupportClasses.PhysicalModels import (
    WorkspaceConfig, PumpLoadout, NeedleSpec, WellRole, ROLE_COLORS,
)
from SupportClasses.WellPlate import WellPlate, WellInfo, ROW_LABELS
from SupportClasses.PrintManager import PrintState

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Well Progress Colors
# ═══════════════════════════════════════════════════════════════════

WELL_DONE_COLOR = "#a6e3a1"     # Green — completed
WELL_ACTIVE_COLOR = "#f9e2af"   # Yellow — currently printing
WELL_PENDING_COLOR = "#45475a"  # Dark gray — waiting
WELL_ERROR_COLOR = "#f38ba8"    # Red — error
WELL_SKIP_COLOR = "#313244"     # Darkest — not assigned

# Mini plate scaling
MINI_SCALE = 3.5
MINI_WELL_RADIUS = 5


# ═══════════════════════════════════════════════════════════════════
# Mini Plate Overview
# ═══════════════════════════════════════════════════════════════════

class MiniPlateOverview(QGraphicsView):
    """
    Miniature plate view showing per-well progress coloring.

    Wells are colored:
    - Done (green) — print completed for this well
    - Active (yellow) — currently being printed
    - Pending (gray) — waiting to be printed
    - Error (red) — error occurred during this well
    - Service (role color) — ink/wash/waste/buffer wells

    Clicking a well emits well_clicked with the well name.
    """

    well_clicked = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self._plate: WellPlate | None = None
        self._well_items: dict[str, QGraphicsEllipseItem] = {}
        self._well_states: dict[str, str] = {}    # "done"/"active"/"pending"/"error"/"skip"
        self._well_roles: dict[str, WellRole] = {}

        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet(f"background-color: {COLORS['crust']}; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumSize(180, 120)
        self.setMaximumWidth(280)

    def set_plate(self, plate: WellPlate) -> None:
        """Load plate geometry."""
        self._plate = plate
        self._well_states.clear()
        self._well_roles.clear()
        self._rebuild()

    def set_well_roles(self, roles: dict[str, WellRole]) -> None:
        """Set well roles for color coding service wells."""
        self._well_roles = roles
        self._update_colors()

    def set_well_state(self, well_name: str, state: str) -> None:
        """Update a single well's progress state."""
        self._well_states[well_name] = state
        self._update_well_color(well_name)

    def set_all_states(self, states: dict[str, str]) -> None:
        """Bulk update well states."""
        self._well_states = states
        self._update_colors()

    def _rebuild(self) -> None:
        self._scene.clear()
        self._well_items.clear()
        if not self._plate:
            return

        for well in self._plate.get_all_wells():
            cx = well.x * MINI_SCALE
            cy = well.y * MINI_SCALE
            r = MINI_WELL_RADIUS
            item = self._scene.addEllipse(
                cx - r, cy - r, 2 * r, 2 * r,
                QPen(QColor("#585b70"), 0.5),
                QBrush(QColor(WELL_SKIP_COLOR)),
            )
            item.setToolTip(well.name)
            self._well_items[well.name] = item

        # Fit view
        rect = self._scene.itemsBoundingRect().adjusted(-10, -10, 10, 10)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    def _update_colors(self) -> None:
        """Update all well colors from states and roles."""
        for name in self._well_items:
            self._update_well_color(name)

    def _update_well_color(self, name: str) -> None:
        item = self._well_items.get(name)
        if not item:
            return

        state = self._well_states.get(name, "")
        role = self._well_roles.get(name, WellRole.EMPTY)

        if state == "done":
            color = WELL_DONE_COLOR
        elif state == "active":
            color = WELL_ACTIVE_COLOR
        elif state == "pending":
            color = WELL_PENDING_COLOR
        elif state == "error":
            color = WELL_ERROR_COLOR
        elif role != WellRole.EMPTY and role != WellRole.PRINT:
            # Service wells use role color
            color = ROLE_COLORS.get(role, WELL_SKIP_COLOR)
        elif role == WellRole.PRINT:
            color = WELL_PENDING_COLOR
        else:
            color = WELL_SKIP_COLOR

        item.setBrush(QBrush(QColor(color)))

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        rect = self._scene.itemsBoundingRect().adjusted(-10, -10, 10, 10)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    def mousePressEvent(self, event) -> None:
        scene_pos = self.mapToScene(event.pos())
        for name, item in self._well_items.items():
            if item.contains(item.mapFromScene(scene_pos)):
                self.well_clicked.emit(name)
                break
        super().mousePressEvent(event)


# ═══════════════════════════════════════════════════════════════════
# Print Monitor Page
# ═══════════════════════════════════════════════════════════════════

class PrintMonitorPage(QWidget):
    """
    Full Print Monitor page — added to navigation as 6th page.

    Provides real-time visualization of:
    - Plate-level progress (which wells done/active/pending)
    - Current well trajectory detail (L-shaped XY/ZY/XZ projections)
    - Syringe fluid column states
    - Print progress (well, layer, step, time, ETA)
    - Tracking error statistics
    - Pause/Abort controls
    """

    # Emitted when user clicks Pause/Resume
    pause_requested = Signal()
    resume_requested = Signal()
    abort_requested = Signal()

    def __init__(
        self,
        controller=None,
        settings=None,
        workspace: WorkspaceConfig | None = None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._workspace = workspace or WorkspaceConfig()
        self._print_state = PrintState.IDLE
        self._print_start_time: float | None = None

        self._build_ui()
        self._connect_signals()

        # v7.1 P7.4: PrintRecorder reference (set by app.py wiring)
        self._recorder = None

    # ── Page Interface (P8.34) ───────────────────────────────────

    def get_page_title(self) -> str:
        return "Print Monitor"

    def get_context_widget(self) -> QWidget:
        """P7.6: Context panel with recording browser + replay controls."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # ── Recording Browser ────────────────────────────────────
        title = QLabel("Print Recordings")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {COLORS['text']};")
        layout.addWidget(title)

        # Refresh button
        btn_refresh = QPushButton("🔄 Refresh")
        btn_refresh.clicked.connect(self._refresh_recording_list)
        layout.addWidget(btn_refresh)

        # Recording list
        self._recording_list = QListWidget()
        self._recording_list.setStyleSheet(
            f"QListWidget {{ background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; font-size: 10pt; }}"
            f"QListWidget::item:selected {{ background-color: {COLORS['surface2']}; }}")
        self._recording_list.setMaximumHeight(200)
        self._recording_list.itemClicked.connect(self._on_recording_selected)
        layout.addWidget(self._recording_list)

        # Selected recording info
        self._rec_info_label = QLabel("Select a recording to view")
        self._rec_info_label.setWordWrap(True)
        self._rec_info_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10pt;")
        layout.addWidget(self._rec_info_label)

        # Replay controls
        replay_row = QHBoxLayout()
        self._btn_load_replay = QPushButton("📂 Load Replay")
        self._btn_load_replay.setToolTip("Load recording and overlay on trajectory view")
        self._btn_load_replay.clicked.connect(self._load_replay)
        self._btn_load_replay.setEnabled(False)
        replay_row.addWidget(self._btn_load_replay)

        self._btn_clear_replay = QPushButton("✕ Clear")
        self._btn_clear_replay.setToolTip("Clear replay overlay")
        self._btn_clear_replay.clicked.connect(self._clear_replay)
        self._btn_clear_replay.setEnabled(False)
        replay_row.addWidget(self._btn_clear_replay)
        layout.addLayout(replay_row)

        # Replay info
        self._replay_info = QLabel("")
        self._replay_info.setWordWrap(True)
        self._replay_info.setStyleSheet(
            f"color: {COLORS['green']}; font-size: 10pt;")
        layout.addWidget(self._replay_info)

        # ── Delete button ────────────────────────────────────────
        self._btn_delete = QPushButton("🗑 Delete Recording")
        self._btn_delete.setStyleSheet(
            f"QPushButton {{ color: {COLORS['red']}; }}")
        self._btn_delete.clicked.connect(self._delete_recording)
        self._btn_delete.setEnabled(False)
        layout.addWidget(self._btn_delete)

        layout.addStretch()

        # Initial refresh
        QTimer.singleShot(500, self._refresh_recording_list)

        return widget

    # ── Recorder Integration (P7.4, P7.5) ────────────────────────

    def set_recorder(self, recorder) -> None:
        """P7.4: Set PrintRecorder reference for live recording access."""
        self._recorder = recorder

    @property
    def recorder(self):
        return self._recorder

    # ── Recording Browser (P7.6) ─────────────────────────────────

    def _refresh_recording_list(self) -> None:
        """Refresh the list of past recordings."""
        self._recording_list.clear()
        self._selected_recording_path = None
        self._btn_load_replay.setEnabled(False)
        self._btn_delete.setEnabled(False)

        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            recordings = recorder.list_recordings()

            for rec in recordings:
                # rec is a RecordingInfo namedtuple
                label = f"{rec.job_name}  ({rec.date_str})"
                item = QListWidgetItem(label)
                item.setData(Qt.ItemDataRole.UserRole, rec.meta_path)
                item.setToolTip(
                    f"Samples: {rec.num_samples}\n"
                    f"Duration: {rec.duration_s:.1f}s\n"
                    f"Path: {rec.meta_path}")
                self._recording_list.addItem(item)

            if not recordings:
                self._rec_info_label.setText("No recordings found")
            else:
                self._rec_info_label.setText(
                    f"{len(recordings)} recording(s) found")

        except Exception as e:
            logger.warning(f"Failed to list recordings: {e}")
            self._rec_info_label.setText(f"Error: {e}")

    def _on_recording_selected(self, item: QListWidgetItem) -> None:
        """Handle recording selection."""
        meta_path = item.data(Qt.ItemDataRole.UserRole)
        self._selected_recording_path = meta_path
        self._btn_load_replay.setEnabled(True)
        self._btn_delete.setEnabled(True)
        self._rec_info_label.setText(item.toolTip())

    # ── Replay Visualization (P7.7) ──────────────────────────────

    def _load_replay(self) -> None:
        """P7.7: Load a recording and overlay on trajectory view."""
        if not hasattr(self, '_selected_recording_path') or not self._selected_recording_path:
            return

        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            data = recorder.load_recording(self._selected_recording_path)

            if data is None:
                self._replay_info.setText("Failed to load recording")
                self._replay_info.setStyleSheet(
                    f"color: {COLORS['red']}; font-size: 10pt;")
                return

            meta = data.get("meta", {})
            samples = data.get("samples", [])

            if not samples:
                self._replay_info.setText("Recording has no samples")
                return

            # Extract actual path points for overlay
            replay_points = []
            for s in samples:
                x = s.get("actual_x", s.get("planned_x", 0))
                y = s.get("actual_y", s.get("planned_y", 0))
                z = s.get("actual_z", s.get("planned_z", 0))
                replay_points.append((x, y, z))

            # Overlay on trajectory view
            if hasattr(self.trajectory_view, 'set_replay_path'):
                self.trajectory_view.set_replay_path(replay_points)

            # Show stats
            summary = meta.get("summary", {})
            avg_err = summary.get("mean_tracking_error_xy_mm", 0)
            max_err = summary.get("max_tracking_error_xy_mm", 0)
            duration = summary.get("total_duration_s", 0)
            num = len(samples)

            self._replay_info.setText(
                f"Loaded: {meta.get('job_name', '?')}\n"
                f"Samples: {num}  Duration: {duration:.1f}s\n"
                f"Avg error: {avg_err*1000:.0f} µm  Max: {max_err*1000:.0f} µm")
            self._replay_info.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 10pt;")
            self._btn_clear_replay.setEnabled(True)

        except Exception as e:
            logger.warning(f"Failed to load replay: {e}")
            self._replay_info.setText(f"Error: {e}")
            self._replay_info.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 10pt;")

    def _clear_replay(self) -> None:
        """Clear replay overlay."""
        if hasattr(self.trajectory_view, 'clear_replay_path'):
            self.trajectory_view.clear_replay_path()
        self._replay_info.setText("")
        self._btn_clear_replay.setEnabled(False)

    def _delete_recording(self) -> None:
        """Delete selected recording."""
        if not hasattr(self, '_selected_recording_path') or not self._selected_recording_path:
            return

        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            recorder.delete_recording(self._selected_recording_path)
            self._refresh_recording_list()
            self._rec_info_label.setText("Recording deleted")
        except Exception as e:
            logger.warning(f"Failed to delete recording: {e}")
            self._rec_info_label.setText(f"Delete error: {e}")

    # ── Properties ────────────────────────────────────────────────

    def set_workspace(self, workspace: WorkspaceConfig) -> None:
        """Update workspace config."""
        self._workspace = workspace
        self.syringe_panel.update_from_workspace(workspace.pumps)
        self._update_needle_info()

    # ── UI Construction ───────────────────────────────────────────

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # ── Top Area: Plate overview + trajectory detail ──────────
        top_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: plate overview + legend
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        plate_group = QGroupBox("Plate Overview")
        plate_group.setStyleSheet(self._group_style())
        plate_inner = QVBoxLayout(plate_group)

        self.mini_plate = MiniPlateOverview()
        plate_inner.addWidget(self.mini_plate)

        # Legend
        legend_layout = QHBoxLayout()
        for label_text, color in [
            ("Done", WELL_DONE_COLOR),
            ("Active", WELL_ACTIVE_COLOR),
            ("Pending", WELL_PENDING_COLOR),
            ("Error", WELL_ERROR_COLOR),
        ]:
            lbl = QLabel(f"● {label_text}")
            lbl.setStyleSheet(f"color: {color}; font-size: 10px;")
            legend_layout.addWidget(lbl)
        legend_layout.addStretch()
        plate_inner.addLayout(legend_layout)

        left_layout.addWidget(plate_group)
        left_layout.addStretch()
        top_splitter.addWidget(left_panel)

        # Right: trajectory detail (L-shaped projections)
        detail_group = QGroupBox("Current Well Detail")
        detail_group.setStyleSheet(self._group_style())
        detail_layout = QVBoxLayout(detail_group)

        self.trajectory_view = TrajectoryView()
        detail_layout.addWidget(self.trajectory_view)

        # Path legend
        path_legend = QHBoxLayout()
        path_legend.addWidget(self._color_label("── Completed", COMPLETED_PATH_COLOR))
        path_legend.addWidget(self._color_label("╌╌ Upcoming", UPCOMING_PATH_COLOR))
        path_legend.addWidget(self._color_label("✛ Needle", NEEDLE_COLOR))
        path_legend.addStretch()
        detail_layout.addLayout(path_legend)

        top_splitter.addWidget(detail_group)
        top_splitter.setStretchFactor(0, 1)
        top_splitter.setStretchFactor(1, 3)

        layout.addWidget(top_splitter, stretch=3)

        # ── Bottom Area: Syringe status + Progress ────────────────
        bottom_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: Syringe status + needle info
        syringe_group = QGroupBox("Syringe Status")
        syringe_group.setStyleSheet(self._group_style())
        syringe_layout = QVBoxLayout(syringe_group)

        self.syringe_panel = SyringeStatusPanel()
        syringe_layout.addWidget(self.syringe_panel)

        # Needle info
        self.needle_label = QLabel("Needle: —")
        self.needle_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        syringe_layout.addWidget(self.needle_label)

        bottom_splitter.addWidget(syringe_group)

        # Right: Print progress + controls
        progress_group = QGroupBox("Print Progress")
        progress_group.setStyleSheet(self._group_style())
        progress_layout = QVBoxLayout(progress_group)

        # Progress info grid
        info_grid = QGridLayout()
        info_grid.setSpacing(4)

        self._progress_labels: dict[str, QLabel] = {}
        progress_fields = [
            ("Job:", "job_name"),
            ("Well:", "well_info"),
            ("Layer:", "layer_info"),
            ("Step:", "step_info"),
            ("Time:", "time_info"),
        ]
        for row, (label_text, key) in enumerate(progress_fields):
            name_lbl = QLabel(label_text)
            name_lbl.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: 11px;")
            info_grid.addWidget(name_lbl, row, 0)

            val_lbl = QLabel("—")
            val_lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-size: 11px; font-weight: bold;")
            info_grid.addWidget(val_lbl, row, 1)
            self._progress_labels[key] = val_lbl

        progress_layout.addLayout(info_grid)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setStyleSheet(
            f"QProgressBar {{ background-color: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"text-align: center; color: {COLORS['text']}; height: 20px; }}"
            f"QProgressBar::chunk {{ background-color: {COLORS['green']}; "
            f"border-radius: 3px; }}")
        progress_layout.addWidget(self.progress_bar)

        # Tracking error + controller status
        error_row = QHBoxLayout()
        self.tracking_error_label = QLabel("Tracking error: — µm")
        self.tracking_error_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        error_row.addWidget(self.tracking_error_label)

        self.controller_label = QLabel("Controller: —")
        self.controller_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        error_row.addWidget(self.controller_label)
        error_row.addStretch()
        progress_layout.addLayout(error_row)

        # Control buttons
        btn_row = QHBoxLayout()
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setMinimumHeight(32)
        self.btn_pause.setStyleSheet(
            f"QPushButton {{ background-color: {COLORS['yellow']}; "
            f"color: {COLORS['crust']}; font-weight: bold; border-radius: 4px; "
            f"padding: 4px 12px; }}"
            f"QPushButton:hover {{ background-color: #e8d49e; }}")
        btn_row.addWidget(self.btn_pause)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setMinimumHeight(32)
        self.btn_abort.setStyleSheet(
            f"QPushButton {{ background-color: {COLORS['red']}; "
            f"color: {COLORS['crust']}; font-weight: bold; border-radius: 4px; "
            f"padding: 4px 12px; }}"
            f"QPushButton:hover {{ background-color: #e07a96; }}")
        btn_row.addWidget(self.btn_abort)

        # State label
        self.state_label = QLabel("IDLE")
        self.state_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 14px; font-weight: bold;")
        self.state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        btn_row.addWidget(self.state_label)

        btn_row.addStretch()
        progress_layout.addLayout(btn_row)

        bottom_splitter.addWidget(progress_group)
        bottom_splitter.setStretchFactor(0, 1)
        bottom_splitter.setStretchFactor(1, 2)

        layout.addWidget(bottom_splitter, stretch=1)

    # ── Signal Connections ────────────────────────────────────────

    def _connect_signals(self) -> None:
        self.btn_pause.clicked.connect(self._on_pause_clicked)
        self.btn_abort.clicked.connect(self._on_abort_clicked)

    def _on_pause_clicked(self) -> None:
        if self._print_state == PrintState.RUNNING:
            self.pause_requested.emit()
        elif self._print_state == PrintState.PAUSED:
            self.resume_requested.emit()

    def _on_abort_clicked(self) -> None:
        if self._print_state in (PrintState.RUNNING, PrintState.PAUSED):
            self.abort_requested.emit()

    # ── External Update API ───────────────────────────────────────

    def on_print_state_changed(self, state: PrintState) -> None:
        """Called when print state changes."""
        self._print_state = state
        state_colors = {
            PrintState.IDLE: COLORS["overlay0"],
            PrintState.RUNNING: COLORS["green"],
            PrintState.PAUSED: COLORS["yellow"],
            PrintState.COMPLETED: COLORS["blue"],
            PrintState.ABORTED: COLORS["red"],
            PrintState.ERROR: COLORS["red"],
        }
        color = state_colors.get(state, COLORS["overlay0"])
        self.state_label.setText(state.name)
        self.state_label.setStyleSheet(
            f"color: {color}; font-size: 14px; font-weight: bold;")

        # Update pause button text
        if state == PrintState.PAUSED:
            self.btn_pause.setText("▶ Resume")
        else:
            self.btn_pause.setText("⏸ Pause")

        # Enable/disable buttons
        active = state in (PrintState.RUNNING, PrintState.PAUSED)
        self.btn_pause.setEnabled(active)
        self.btn_abort.setEnabled(active)

        # Track start time
        if state == PrintState.RUNNING and self._print_start_time is None:
            self._print_start_time = time.time()
        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            self._print_start_time = None

    def on_progress_update(
        self,
        current_step: int,
        total_steps: int,
        message: str = "",
        job_name: str = "",
        well_name: str = "",
        well_index: int = 0,
        total_wells: int = 0,
        layer: int = 0,
        total_layers: int = 0,
    ) -> None:
        """Called on each print step progress."""
        # Progress bar
        pct = int(100 * current_step / max(total_steps, 1))
        self.progress_bar.setValue(pct)
        self.progress_bar.setFormat(f"{pct}%")

        # Labels
        self._progress_labels["job_name"].setText(job_name or "—")
        self._progress_labels["well_info"].setText(
            f"{well_name} ({well_index}/{total_wells})" if well_name else "—")
        self._progress_labels["layer_info"].setText(
            f"{layer}/{total_layers}" if total_layers > 0 else "—")
        self._progress_labels["step_info"].setText(
            f"{current_step:,} / {total_steps:,}")

        # Time elapsed + ETA
        if self._print_start_time:
            elapsed = time.time() - self._print_start_time
            elapsed_str = self._format_time(elapsed)
            if pct > 0:
                eta_total = elapsed / (pct / 100.0)
                eta_remaining = eta_total - elapsed
                eta_str = f"~{self._format_time(eta_remaining)}"
            else:
                eta_str = "—"
            self._progress_labels["time_info"].setText(
                f"{elapsed_str} / {eta_str}")
        else:
            self._progress_labels["time_info"].setText("—")

        # Update plate overview
        if well_name:
            self.mini_plate.set_well_state(well_name, "active")

    def on_well_completed(self, well_name: str) -> None:
        """Called when a single well finishes printing."""
        self.mini_plate.set_well_state(well_name, "done")

    def on_well_error(self, well_name: str) -> None:
        """Called when a well encounters an error."""
        self.mini_plate.set_well_state(well_name, "error")

    def update_needle_position(
        self,
        x_mm: float | None,
        y_mm: float | None,
        z_mm: float | None = None,
    ) -> None:
        """Update needle position on trajectory view."""
        self.trajectory_view.set_needle_position(x_mm, y_mm, z_mm)
        if x_mm is not None and y_mm is not None and z_mm is not None:
            self.trajectory_view.add_completed_point(x_mm, y_mm, z_mm)

    def update_upcoming_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> None:
        """Update upcoming waypoints on trajectory view."""
        self.trajectory_view.set_upcoming_waypoints(waypoints)

    def update_tracking_error(
        self,
        error_mm: float,
        controller_type: str = "",
    ) -> None:
        """Update tracking error display."""
        error_um = error_mm * 1000
        self.tracking_error_label.setText(f"Tracking error: {error_um:.0f} µm (avg)")
        self.trajectory_view.set_tracking_error(error_mm)

        # Color code: green < 50µm, yellow < 200µm, red >= 200µm
        if error_um < 50:
            color = COLORS["green"]
        elif error_um < 200:
            color = COLORS["yellow"]
        else:
            color = COLORS["red"]
        self.tracking_error_label.setStyleSheet(
            f"color: {color}; font-size: 11px;")

        if controller_type:
            self.controller_label.setText(f"Controller: {controller_type}")

    def update_syringe_state(
        self,
        pumps: dict[str, PumpLoadout],
        active_pump: str | None = None,
    ) -> None:
        """Update syringe display from pump loadouts."""
        self.syringe_panel.update_from_workspace(pumps)
        # Show flow animation on active pump
        for pid in ["P1", "P2", "P3"]:
            self.syringe_panel.set_flowing(
                pid, pid == active_pump and self._print_state == PrintState.RUNNING)

    def setup_plate(
        self,
        plate: WellPlate,
        well_roles: dict[str, WellRole] | None = None,
        print_wells: list[str] | None = None,
        well_diameter_mm: float = 0.0,
    ) -> None:
        """
        Initialize plate overview and trajectory view for a new print job.

        Called before print starts, with the well setup data.
        """
        self.mini_plate.set_plate(plate)

        if well_roles:
            self.mini_plate.set_well_roles(well_roles)

        # Set print wells to "pending"
        if print_wells:
            states = {name: "pending" for name in print_wells}
            self.mini_plate.set_all_states(states)

        # Set well boundary on trajectory views
        if well_diameter_mm > 0:
            self.trajectory_view.set_well_diameter(well_diameter_mm)

        # Clear previous path data
        self.trajectory_view.clear_path()

    def reset(self) -> None:
        """Reset monitor to idle state."""
        self.on_print_state_changed(PrintState.IDLE)
        self.progress_bar.setValue(0)
        for lbl in self._progress_labels.values():
            lbl.setText("—")
        self.tracking_error_label.setText("Tracking error: — µm")
        self.controller_label.setText("Controller: —")
        self.trajectory_view.clear_path()
        self._print_start_time = None

    # ── Helpers ───────────────────────────────────────────────────

    def _update_needle_info(self) -> None:
        """Update needle info label from workspace."""
        needle = self._workspace.needle
        if needle:
            self.needle_label.setText(
                f"Needle: {needle.gauge}G × {needle.length_inches}\"  "
                f"ID: {needle.id_um:.0f} µm")
        else:
            self.needle_label.setText("Needle: —")

    @staticmethod
    def _format_time(seconds: float) -> str:
        """Format seconds as M:SS or H:MM:SS."""
        seconds = max(0, seconds)
        if seconds < 3600:
            m = int(seconds) // 60
            s = int(seconds) % 60
            return f"{m}:{s:02d}"
        else:
            h = int(seconds) // 3600
            m = (int(seconds) % 3600) // 60
            s = int(seconds) % 60
            return f"{h}:{m:02d}:{s:02d}"

    @staticmethod
    def _color_label(text: str, color: str) -> QLabel:
        """Create a colored label for legends."""
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color: {color}; font-size: 10px;")
        return lbl

    @staticmethod
    def _group_style() -> str:
        return (
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}"
        )


# ═══════════════════════════════════════════════════════════════════
# Color constants re-exported for imports from trajectory_view
# ═══════════════════════════════════════════════════════════════════

COMPLETED_PATH_COLOR = "#a6e3a1"
UPCOMING_PATH_COLOR = "#f9e2af"
NEEDLE_COLOR = "#f5c2e7"
