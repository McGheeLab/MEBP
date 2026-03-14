"""
pp_target_selection.py — Pick & Place Target Selection page.

v7.3.3: Interactive camera/stitched-image viewer where users:
  1. View live camera feed or stitched composite
  2. Click to mark pick-and-place targets (→ stage XY coordinates)
  3. Manually capture frames or auto-scan wells to build the composite
  4. Manage a target list with selection/deselection

Layout:
    ┌───────────────────────────────┬────────────────────────┐
    │   Stitched Image View         │  Target List           │
    │   (click to mark, scroll zoom)│  [T001] [T002] ...     │
    ├───────────────────────────────┴────────────────────────┤
    │ Scan: [Manual Capture] [Auto-Scan ▾] | View: ○Live ●St│
    └───────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import logging
import time
import threading
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QFrame,
    QPushButton, QLabel, QListWidget, QListWidgetItem,
    QGroupBox, QComboBox, QDoubleSpinBox, QCheckBox,
    QRadioButton, QButtonGroup, QProgressBar, QSizePolicy,
    QScrollArea, QMessageBox, QGridLayout, QStackedWidget,
)
from PySide6.QtCore import Qt, Signal, QTimer, QPointF
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont,
    QMouseEvent, QWheelEvent, QTransform,
)

import numpy as np

from gui.styles import COLORS
from gui.scaling import s, scaled_font_size
from gui.widgets.camera_feed_view import CameraFeedView
from gui.widgets.target_overlay_camera_view import TargetOverlayCameraView
from SupportClasses.ImageStitcher import StitchedImage, generate_scan_pattern
from SupportClasses.PickAndPlaceManager import (
    PickPlaceTarget, PickPlaceOperation, OperationType, OperationQueue,
)

logger = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════
#  STITCHED IMAGE VIEW — Interactive widget for viewing + marking
# ════════════════════════════════════════════════════════════════════

class StitchedImageView(QWidget):
    """Interactive view for the stitched composite image.

    Features:
      - Display stitched image or live camera frame
      - Click to add targets (emits signal with stage coords)
      - Right-click to remove nearest target
      - Scroll to zoom, middle-drag to pan
      - Draws target markers, camera FOV, crosshair
    """

    target_added = Signal(float, float)      # (x_um, y_um)
    target_removed = Signal(str)             # target_id
    target_hovered = Signal(str)             # target_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setMinimumSize(s(400), s(300))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Image data
        self._composite_image: Optional[QImage] = None
        self._live_frame: Optional[QImage] = None
        self._show_live: bool = False  # False = stitched, True = live

        # Stitcher reference (for coordinate conversion)
        self._stitcher: Optional[StitchedImage] = None

        # Targets
        self._targets: list[PickPlaceTarget] = []

        # Camera FOV in stage coords
        self._camera_fov_x_um: float = 0.0
        self._camera_fov_y_um: float = 0.0
        self._camera_pos_x_um: float = 0.0
        self._camera_pos_y_um: float = 0.0
        self._um_per_px: float = 1.67  # fallback; set from CameraManager

        # View transform
        self._zoom: float = 1.0
        self._pan_x: float = 0.0
        self._pan_y: float = 0.0
        self._dragging: bool = False
        self._drag_start = QPointF()
        self._drag_pan_start = (0.0, 0.0)

        # Selection
        self._hovered_target: Optional[str] = None
        self._selected_target: Optional[str] = None

    # ── Public API ───────────────────────────────────────────────

    def set_stitcher(self, stitcher: StitchedImage):
        """Set the stitcher reference for coordinate conversion."""
        self._stitcher = stitcher

    def set_composite(self, composite: np.ndarray):
        """Update the displayed composite image."""
        if composite is None:
            self._composite_image = None
        else:
            h, w = composite.shape[:2]
            if composite.ndim == 3 and composite.shape[2] == 3:
                # BGR → RGB
                rgb = composite[:, :, ::-1].copy()
                self._composite_image = QImage(
                    rgb.data, w, h, 3 * w, QImage.Format_RGB888)
                # Keep reference to prevent garbage collection
                self._composite_image._np_data = rgb
            else:
                self._composite_image = None
        self.update()

    def set_live_frame(self, frame: np.ndarray):
        """Update the live camera frame."""
        if frame is None:
            self._live_frame = None
        else:
            h, w = frame.shape[:2]
            if frame.ndim == 3 and frame.shape[2] == 3:
                rgb = frame[:, :, ::-1].copy()
                self._live_frame = QImage(
                    rgb.data, w, h, 3 * w, QImage.Format_RGB888)
                self._live_frame._np_data = rgb
        if self._show_live:
            self.update()

    def set_targets(self, targets: list[PickPlaceTarget]):
        """Update the target list for drawing."""
        self._targets = list(targets)
        self.update()

    def set_show_live(self, show_live: bool):
        """Switch between live camera and stitched view."""
        self._show_live = show_live
        self.update()

    def set_um_per_px(self, value: float):
        """Set the µm/px scale for live camera coordinate conversion."""
        self._um_per_px = value

    def set_camera_fov(self, fov_w_um: float, fov_h_um: float):
        """Set the camera field of view size in µm."""
        self._camera_fov_x_um = fov_w_um
        self._camera_fov_y_um = fov_h_um

    def set_camera_position(self, x_um: float, y_um: float):
        """Set the current camera/stage position."""
        self._camera_pos_x_um = x_um
        self._camera_pos_y_um = y_um
        self.update()

    def set_selected_target(self, target_id: Optional[str]):
        """Highlight a specific target."""
        self._selected_target = target_id
        self.update()

    def fit_to_content(self):
        """Reset zoom/pan to fit the image in the widget."""
        img = self._live_frame if self._show_live else self._composite_image
        if img is None:
            return
        w_ratio = self.width() / max(1, img.width())
        h_ratio = self.height() / max(1, img.height())
        self._zoom = min(w_ratio, h_ratio) * 0.95
        self._pan_x = 0.0
        self._pan_y = 0.0
        self.update()

    # ── Coordinate transforms ────────────────────────────────────

    def _widget_to_image(self, wx: float, wy: float) -> tuple[float, float]:
        """Convert widget pixel coords to image pixel coords."""
        cx = self.width() / 2 + self._pan_x
        cy = self.height() / 2 + self._pan_y
        img = self._live_frame if self._show_live else self._composite_image
        if img is None:
            return (0, 0)
        ix = (wx - cx) / self._zoom + img.width() / 2
        iy = (wy - cy) / self._zoom + img.height() / 2
        return (ix, iy)

    def _image_to_widget(self, ix: float, iy: float) -> tuple[float, float]:
        """Convert image pixel coords to widget pixel coords."""
        cx = self.width() / 2 + self._pan_x
        cy = self.height() / 2 + self._pan_y
        img = self._live_frame if self._show_live else self._composite_image
        if img is None:
            return (0, 0)
        wx = (ix - img.width() / 2) * self._zoom + cx
        wy = (iy - img.height() / 2) * self._zoom + cy
        return (wx, wy)

    def _widget_to_stage(self, wx: float, wy: float) -> Optional[tuple[float, float]]:
        """Convert widget pixel coords to stage coords (µm)."""
        if self._show_live:
            # In live mode, coords are relative to current camera position
            img = self._live_frame
            if img is None:
                return None
            ix, iy = self._widget_to_image(wx, wy)
            # Use stitcher's um_per_px if available, else the direct value
            um_per_px = self._um_per_px
            if self._stitcher is not None:
                um_per_px = self._stitcher.um_per_px
            dx_um = (ix - img.width() / 2) * um_per_px
            dy_um = (iy - img.height() / 2) * um_per_px
            return (self._camera_pos_x_um + dx_um,
                    self._camera_pos_y_um + dy_um)
        else:
            # In stitched mode, use stitcher's coordinate system
            if self._stitcher is None:
                return None
            ix, iy = self._widget_to_image(wx, wy)
            return self._stitcher.pixel_to_stage(int(ix), int(iy))

    def _stage_to_widget(self, x_um: float, y_um: float) -> tuple[float, float]:
        """Convert stage coords to widget pixel coords."""
        if self._show_live:
            img = self._live_frame
            if img is None:
                return (0, 0)
            um_per_px = self._um_per_px
            if self._stitcher is not None:
                um_per_px = self._stitcher.um_per_px
            ix = (x_um - self._camera_pos_x_um) / um_per_px + img.width() / 2
            iy = (y_um - self._camera_pos_y_um) / um_per_px + img.height() / 2
        else:
            if self._stitcher is None:
                return (0, 0)
            px, py = self._stitcher.stage_to_pixel(x_um, y_um)
            ix, iy = float(px), float(py)
        return self._image_to_widget(ix, iy)

    # ── Paint ────────────────────────────────────────────────────

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(COLORS["crust"]))

        # Draw image
        img = self._live_frame if self._show_live else self._composite_image
        if img is not None:
            cx = self.width() / 2 + self._pan_x
            cy = self.height() / 2 + self._pan_y
            scaled_w = img.width() * self._zoom
            scaled_h = img.height() * self._zoom
            dest_x = cx - scaled_w / 2
            dest_y = cy - scaled_h / 2

            painter.drawImage(
                int(dest_x), int(dest_y),
                img.scaled(int(scaled_w), int(scaled_h),
                           Qt.KeepAspectRatio, Qt.SmoothTransformation))

        # Draw camera FOV rectangle (in stitched view)
        if not self._show_live and self._camera_fov_x_um > 0:
            self._draw_camera_fov(painter)

        # Draw targets
        self._draw_targets(painter)

        # Draw crosshair at center
        self._draw_crosshair(painter)

        # "No image" label
        if img is None:
            painter.setPen(QColor(COLORS["overlay0"]))
            painter.setFont(QFont("Segoe UI", scaled_font_size(12)))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "No image — capture frames to build composite")

        painter.end()

    def _draw_camera_fov(self, painter: QPainter):
        """Draw the camera field-of-view rectangle on the stitched view."""
        if self._stitcher is None:
            return
        half_w = self._camera_fov_x_um / 2
        half_h = self._camera_fov_y_um / 2
        tl = self._stage_to_widget(
            self._camera_pos_x_um - half_w,
            self._camera_pos_y_um - half_h)
        br = self._stage_to_widget(
            self._camera_pos_x_um + half_w,
            self._camera_pos_y_um + half_h)

        pen = QPen(QColor(COLORS["blue"]), 2, Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(int(tl[0]), int(tl[1]),
                         int(br[0] - tl[0]), int(br[1] - tl[1]))

    def _draw_targets(self, painter: QPainter):
        """Draw target markers on the image."""
        for target in self._targets:
            wx, wy = self._stage_to_widget(target.x_um, target.y_um)

            # Marker radius in widget pixels
            if target.size_um > 0 and self._stitcher:
                r_px = (target.size_um / 2 / self._stitcher.um_per_px) * self._zoom
                r_px = max(6, r_px)
            else:
                r_px = 8

            # Color based on state
            if target.target_id == self._selected_target:
                color = QColor(COLORS["mauve"])
                pen_width = 3
            elif target.target_id == self._hovered_target:
                color = QColor(COLORS["peach"])
                pen_width = 2
            elif target.selected:
                color = QColor(COLORS["green"])
                pen_width = 2
            else:
                color = QColor(COLORS["overlay0"])
                pen_width = 1

            painter.setPen(QPen(color, pen_width))
            painter.setBrush(QBrush(QColor(color.red(), color.green(),
                                           color.blue(), 40)))
            painter.drawEllipse(QPointF(wx, wy), r_px, r_px)

            # ID label
            painter.setPen(QPen(color, 1))
            painter.setFont(QFont("Consolas", scaled_font_size(8)))
            painter.drawText(int(wx + r_px + 3), int(wy - 2), target.target_id)

    def _draw_crosshair(self, painter: QPainter):
        """Draw a small crosshair at the current stage position."""
        if self._stitcher is None:
            return
        wx, wy = self._stage_to_widget(self._camera_pos_x_um,
                                        self._camera_pos_y_um)
        pen = QPen(QColor(COLORS["red"]), 1, Qt.SolidLine)
        painter.setPen(pen)
        size = 12
        painter.drawLine(int(wx - size), int(wy), int(wx + size), int(wy))
        painter.drawLine(int(wx), int(wy - size), int(wx), int(wy + size))

    # ── Mouse events ─────────────────────────────────────────────

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            # Add target
            stage = self._widget_to_stage(event.position().x(),
                                          event.position().y())
            if stage:
                self.target_added.emit(stage[0], stage[1])
        elif event.button() == Qt.RightButton:
            # Remove nearest target
            nearest = self._find_nearest_target(
                event.position().x(), event.position().y())
            if nearest:
                self.target_removed.emit(nearest.target_id)
        elif event.button() == Qt.MiddleButton:
            # Start pan
            self._dragging = True
            self._drag_start = event.position()
            self._drag_pan_start = (self._pan_x, self._pan_y)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._dragging:
            dx = event.position().x() - self._drag_start.x()
            dy = event.position().y() - self._drag_start.y()
            self._pan_x = self._drag_pan_start[0] + dx
            self._pan_y = self._drag_pan_start[1] + dy
            self.update()
        else:
            # Hover detection
            nearest = self._find_nearest_target(
                event.position().x(), event.position().y(), threshold=20)
            new_hover = nearest.target_id if nearest else None
            if new_hover != self._hovered_target:
                self._hovered_target = new_hover
                if new_hover:
                    self.target_hovered.emit(new_hover)
                self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MiddleButton:
            self._dragging = False

    def wheelEvent(self, event: QWheelEvent):
        # Zoom around mouse position
        delta = event.angleDelta().y()
        factor = 1.1 if delta > 0 else 1.0 / 1.1
        self._zoom = max(0.01, min(100, self._zoom * factor))
        self.update()

    def _find_nearest_target(self, wx: float, wy: float,
                             threshold: float = 30) -> Optional[PickPlaceTarget]:
        """Find the target nearest to widget coords within threshold."""
        best = None
        best_dist = threshold
        for t in self._targets:
            tx, ty = self._stage_to_widget(t.x_um, t.y_um)
            dist = ((wx - tx) ** 2 + (wy - ty) ** 2) ** 0.5
            if dist < best_dist:
                best = t
                best_dist = dist
        return best


# ════════════════════════════════════════════════════════════════════
#  TARGET SELECTION PAGE
# ════════════════════════════════════════════════════════════════════

class PPTargetSelectionPage(QWidget):
    """Pick & Place Target Selection page.

    Provides the stitched image viewer, target list, and scan controls.
    """

    targets_changed = Signal(list)   # Emits updated target list
    queue_changed = Signal(object)    # Emits OperationQueue when targets change

    def __init__(self, controller=None, settings=None,
                 camera_manager=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self._camera_manager = camera_manager  # v7.3.3: shared CameraManager

        self._stitcher: Optional[StitchedImage] = None
        self._targets: list[PickPlaceTarget] = []
        self._target_counter: int = 0
        self._hardware_config = None
        self._um_per_px: float = 1.67  # Default, updated from camera config

        # Operation queue — auto-built as targets are added
        self._queue = OperationQueue()
        self._current_op_type: Optional[OperationType] = OperationType.SPHEROID_PICKUP
        self._current_config = None  # Set by PPOperationSetupPage
        self._dest_well: str = ""

        # Scanning state
        self._scanning: bool = False
        self._scan_thread: Optional[threading.Thread] = None

        # v7.3.3: Camera state
        self._active_cam_idx: int = 0  # Which camera is selected for this page

        self._build_ui()

        # Connect to CameraManager signals so combos stay in sync
        if camera_manager is not None:
            camera_manager.cameras_detected.connect(self._on_cameras_detected)
            camera_manager.camera_started.connect(self._on_camera_started)

    def _build_ui(self):
        self.setStyleSheet(f"background-color: {COLORS['base']};")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Main splitter: image view | target list
        splitter = QSplitter(Qt.Horizontal)

        # Left: Stacked widget — index 0 = StitchedImageView, index 1 = CameraFeedView
        self._view_stack = QStackedWidget()

        # Page 0: Stitched composite view (zoom/pan/click-to-add)
        self._image_view = StitchedImageView()
        self._image_view.target_added.connect(self._on_target_added)
        self._image_view.target_removed.connect(self._on_target_removed)
        self._image_view.target_hovered.connect(self._on_target_hovered)
        self._view_stack.addWidget(self._image_view)  # index 0

        # Page 1: Live camera feed with target overlays
        self._live_feed_view = TargetOverlayCameraView(
            camera_manager=self._camera_manager,
            cam_idx=self._active_cam_idx,
            show_crosshair=True,
            label="Live Camera — select source in context panel",
        )
        self._live_feed_view.clicked.connect(self._on_live_feed_clicked)
        self._view_stack.addWidget(self._live_feed_view)  # index 1

        self._view_stack.setCurrentIndex(0)  # Start on stitched view
        splitter.addWidget(self._view_stack)

        # Right: Target list panel
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        # Target list header
        header = QLabel("Targets")
        header.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: 12pt; font-weight: bold;")
        right_layout.addWidget(header)

        # Target list
        self._target_list = QListWidget()
        self._target_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 4px;
                color: {COLORS['text']};
            }}
            QListWidget::item {{
                padding: 4px;
                border-bottom: 1px solid {COLORS['surface1']};
            }}
            QListWidget::item:selected {{
                background-color: {COLORS['surface1']};
            }}
        """)
        self._target_list.currentItemChanged.connect(self._on_target_selected)
        right_layout.addWidget(self._target_list, 1)

        # Target list buttons
        btn_row = QHBoxLayout()
        self._btn_remove = QPushButton("Remove")
        self._btn_remove.clicked.connect(self._remove_selected_target)
        btn_row.addWidget(self._btn_remove)

        self._btn_clear = QPushButton("Clear All")
        self._btn_clear.clicked.connect(self._clear_all_targets)
        btn_row.addWidget(self._btn_clear)
        right_layout.addLayout(btn_row)

        # Target count label
        self._lbl_count = QLabel("0 targets")
        self._lbl_count.setStyleSheet(f"color: {COLORS['subtext0']};")
        right_layout.addWidget(self._lbl_count)

        right_panel.setMinimumWidth(s(200))
        right_panel.setMaximumWidth(s(300))
        splitter.addWidget(right_panel)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

        # Bottom: Scan controls bar
        scan_bar = QFrame()
        scan_bar.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS['surface0']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px;
            }}
        """)
        scan_layout = QHBoxLayout(scan_bar)
        scan_layout.setContentsMargins(12, 6, 12, 6)

        # View mode toggle
        self._rb_stitched = QRadioButton("Stitched")
        self._rb_live = QRadioButton("Live Camera")
        self._rb_stitched.setChecked(True)
        view_group = QButtonGroup(self)
        view_group.addButton(self._rb_stitched)
        view_group.addButton(self._rb_live)
        self._rb_stitched.toggled.connect(self._on_view_mode_changed)
        scan_layout.addWidget(QLabel("View:"))
        scan_layout.addWidget(self._rb_stitched)
        scan_layout.addWidget(self._rb_live)

        scan_layout.addWidget(self._make_separator())

        # Manual capture
        self._btn_capture = QPushButton("Manual Capture")
        self._btn_capture.setToolTip("Capture current frame at current position")
        self._btn_capture.clicked.connect(self._manual_capture)
        scan_layout.addWidget(self._btn_capture)

        # Well selector for auto-scan
        scan_layout.addWidget(QLabel("Well:"))
        self._well_combo = QComboBox()
        self._well_combo.setMinimumWidth(s(80))
        self._well_combo.setPlaceholderText("Select well")
        scan_layout.addWidget(self._well_combo)

        # Auto-scan button
        self._btn_autoscan = QPushButton("Auto-Scan")
        self._btn_autoscan.setToolTip("Automatically scan the selected well")
        self._btn_autoscan.clicked.connect(self._start_auto_scan)
        scan_layout.addWidget(self._btn_autoscan)

        # Progress bar (hidden until scanning)
        self._scan_progress = QProgressBar()
        self._scan_progress.setMaximumHeight(s(16))
        self._scan_progress.setVisible(False)
        scan_layout.addWidget(self._scan_progress)

        # Fit view button
        self._btn_fit = QPushButton("Fit")
        self._btn_fit.setToolTip("Fit view to content")
        self._btn_fit.clicked.connect(self._image_view.fit_to_content)
        scan_layout.addWidget(self._btn_fit)

        scan_layout.addStretch()

        layout.addWidget(scan_bar)

    def _make_separator(self) -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color: {COLORS['surface1']};")
        return sep

    # ── Page interface ───────────────────────────────────────────

    def get_page_title(self) -> str:
        return "Target Selection"

    def get_context_widget(self) -> QWidget:
        """Build context panel with camera selector, jog controls,
        scan settings, and target options."""
        if hasattr(self, '_context_widget') and self._context_widget:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Camera (shared via CameraManager) ─────────────────
        cam_label = QLabel("Camera")
        cam_label.setObjectName("contextSectionLabel")
        layout.addWidget(cam_label)

        self._ctx_camera_combo = QComboBox()
        self._ctx_camera_combo.setPlaceholderText("No cameras available")
        self._ctx_camera_combo.setToolTip("Select camera for live feed and capture")
        self._ctx_camera_combo.currentIndexChanged.connect(self._on_cam_selected)
        layout.addWidget(self._ctx_camera_combo)

        # Source selector (populated from CameraManager's detected sources)
        src_row = QHBoxLayout()
        src_row.addWidget(QLabel("Source:"))
        self._ctx_source_combo = QComboBox()
        self._ctx_source_combo.setPlaceholderText("Detect in Hardware Setup")
        self._ctx_source_combo.setToolTip("Camera source (detect in Hardware Setup)")
        src_row.addWidget(self._ctx_source_combo, 1)
        layout.addLayout(src_row)

        cam_btns = QHBoxLayout()
        self._ctx_btn_start_cam = QPushButton("Start")
        self._ctx_btn_start_cam.setToolTip("Start the selected camera feed")
        self._ctx_btn_start_cam.clicked.connect(self._start_camera)
        cam_btns.addWidget(self._ctx_btn_start_cam)
        self._ctx_btn_stop_cam = QPushButton("Stop")
        self._ctx_btn_stop_cam.setToolTip("Stop camera feed")
        self._ctx_btn_stop_cam.clicked.connect(self._stop_camera)
        self._ctx_btn_stop_cam.setEnabled(False)
        cam_btns.addWidget(self._ctx_btn_stop_cam)
        layout.addLayout(cam_btns)

        # ── XY Jog ────────────────────────────────────────────────
        jog_label = QLabel("XY Jog")
        jog_label.setObjectName("contextSectionLabel")
        layout.addWidget(jog_label)

        # Step size selector
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step:"))
        self._ctx_xy_step = QComboBox()
        for s in [1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0]:
            if s >= 1.0:
                self._ctx_xy_step.addItem(f"{s:g} µm", s)
            else:
                self._ctx_xy_step.addItem(f"{s:.2f} µm", s)
        self._ctx_xy_step.setCurrentIndex(3)  # 50 µm default
        step_row.addWidget(self._ctx_xy_step, 1)
        layout.addLayout(step_row)

        # Jog arrow buttons (3x3 grid, center empty)
        _btn_style = (
            f"QPushButton {{ background-color: {COLORS['surface0']}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"color: {COLORS['text']}; font-weight: bold; min-height: 32px; }}"
            f"QPushButton:hover {{ background-color: {COLORS['surface1']}; }}"
        )
        jog_grid = QGridLayout()
        jog_grid.setSpacing(2)

        btn_yp = QPushButton("Y+")
        btn_yp.setStyleSheet(_btn_style)
        btn_yp.clicked.connect(lambda: self._ctx_jog_xy(0, 1))
        jog_grid.addWidget(btn_yp, 0, 1)

        btn_xm = QPushButton("X-")
        btn_xm.setStyleSheet(_btn_style)
        btn_xm.clicked.connect(lambda: self._ctx_jog_xy(-1, 0))
        jog_grid.addWidget(btn_xm, 1, 0)

        btn_xp = QPushButton("X+")
        btn_xp.setStyleSheet(_btn_style)
        btn_xp.clicked.connect(lambda: self._ctx_jog_xy(1, 0))
        jog_grid.addWidget(btn_xp, 1, 2)

        btn_ym = QPushButton("Y-")
        btn_ym.setStyleSheet(_btn_style)
        btn_ym.clicked.connect(lambda: self._ctx_jog_xy(0, -1))
        jog_grid.addWidget(btn_ym, 2, 1)

        layout.addLayout(jog_grid)

        # Z jog
        z_label = QLabel("Z Jog")
        z_label.setObjectName("contextSectionLabel")
        layout.addWidget(z_label)

        z_step_row = QHBoxLayout()
        z_step_row.addWidget(QLabel("Step:"))
        self._ctx_z_step = QComboBox()
        for s in [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]:
            self._ctx_z_step.addItem(f"{s} mm", s)
        self._ctx_z_step.setCurrentIndex(2)  # 0.1 mm default
        z_step_row.addWidget(self._ctx_z_step, 1)
        layout.addLayout(z_step_row)

        z_btns = QHBoxLayout()
        btn_zp = QPushButton("Z+")
        btn_zp.setStyleSheet(_btn_style)
        btn_zp.clicked.connect(lambda: self._ctx_jog_z(1))
        z_btns.addWidget(btn_zp)
        btn_zm = QPushButton("Z-")
        btn_zm.setStyleSheet(_btn_style)
        btn_zm.clicked.connect(lambda: self._ctx_jog_z(-1))
        z_btns.addWidget(btn_zm)
        layout.addLayout(z_btns)

        # ── Scan Settings ─────────────────────────────────────────
        scan_label = QLabel("Scan Settings")
        scan_label.setObjectName("contextSectionLabel")
        layout.addWidget(scan_label)

        row = QHBoxLayout()
        row.addWidget(QLabel("Overlap:"))
        self._spin_overlap = QDoubleSpinBox()
        self._spin_overlap.setRange(0.0, 0.8)
        self._spin_overlap.setValue(0.2)
        self._spin_overlap.setSingleStep(0.05)
        self._spin_overlap.setSuffix(" (frac)")
        row.addWidget(self._spin_overlap)
        layout.addLayout(row)

        # ── Target Settings ───────────────────────────────────────
        tgt_label = QLabel("Target Settings")
        tgt_label.setObjectName("contextSectionLabel")
        layout.addWidget(tgt_label)

        row = QHBoxLayout()
        row.addWidget(QLabel("Size (µm):"))
        self._spin_target_size = QDoubleSpinBox()
        self._spin_target_size.setRange(1, 5000)
        self._spin_target_size.setValue(200)
        self._spin_target_size.setSingleStep(10)
        row.addWidget(self._spin_target_size)
        layout.addLayout(row)

        layout.addStretch()

        self._context_widget = ctx
        return ctx

    # ── Context widget callbacks ──────────────────────────────────

    def _ctx_jog_xy(self, dx: int, dy: int):
        """Jog XY stage from context panel."""
        if not self.controller or not self.controller.is_xy_connected:
            return
        step_um = self._ctx_xy_step.currentData() or 50.0
        self.controller.move_xy_relative_um(dx * step_um, dy * step_um)

    def _ctx_jog_z(self, direction: int):
        """Jog Z axis from context panel."""
        if not self.controller or not self.controller.is_zp_connected:
            return
        step_mm = self._ctx_z_step.currentData() or 0.1
        self.controller.move_z_relative(direction * step_mm)

    # ── View mode switching ──────────────────────────────────────

    def showEvent(self, event):
        """Auto-switch to live view if a camera is running."""
        super().showEvent(event)
        self._auto_switch_to_live_if_running()

    def _on_camera_started(self, cam_idx: int):
        """A camera was started (from any page) — auto-switch to live view."""
        self._auto_switch_to_live_if_running()
        self._sync_cam_button_state()

    def _auto_switch_to_live_if_running(self):
        """If any camera is running, switch to live view and select it."""
        mgr = self._camera_manager
        if mgr is None:
            return
        # Check if the currently selected camera is running
        if mgr.is_running(self._active_cam_idx):
            self._switch_to_live()
            return
        # Otherwise check all cameras and select the first running one
        for i in range(mgr.max_cameras):
            if mgr.is_running(i):
                self._active_cam_idx = i
                if hasattr(self, '_live_feed_view'):
                    self._live_feed_view.set_camera(i)
                # Update combo to match
                combo = getattr(self, '_ctx_camera_combo', None)
                if combo:
                    combo.blockSignals(True)
                    idx = combo.findData(i)
                    if idx >= 0:
                        combo.setCurrentIndex(idx)
                    combo.blockSignals(False)
                self._switch_to_live()
                return

    def _switch_to_live(self):
        """Switch the view stack to live camera feed."""
        if hasattr(self, '_view_stack'):
            self._view_stack.setCurrentIndex(1)
        if hasattr(self, '_rb_live'):
            self._rb_live.blockSignals(True)
            self._rb_live.setChecked(True)
            self._rb_live.blockSignals(False)

    def _on_view_mode_changed(self, stitched_checked: bool):
        """Switch between stitched composite view and live camera feed."""
        if stitched_checked:
            self._view_stack.setCurrentIndex(0)  # StitchedImageView
        else:
            self._view_stack.setCurrentIndex(1)  # CameraFeedView

    def _on_live_feed_clicked(self, px_x: float, px_y: float):
        """Handle click on live CameraFeedView — add target at stage coords.

        Converts image pixel coords to stage XY using the camera's µm/px
        and the current stage position (center of the image).
        """
        mgr = self._camera_manager
        if mgr is None:
            return

        # Get image dimensions from the feed view
        img_w, img_h = self._live_feed_view.image_size
        if img_w == 0 or img_h == 0:
            return

        # Get current stage position (image center = stage position)
        stage_x, stage_y = 0.0, 0.0
        if self.controller:
            try:
                xy = self.controller.get_xy_position(cached=True)
                if xy[0] is not None:
                    stage_x, stage_y = xy[0], xy[1]
            except Exception:
                pass

        # Convert pixel offset from center → stage offset
        dx_um, dy_um = mgr.pixel_to_stage_offset(
            self._active_cam_idx, px_x, px_y, img_w, img_h)

        # Target position = stage center + pixel offset
        target_x = stage_x + dx_um
        target_y = stage_y + dy_um

        self._on_target_added(target_x, target_y)

    # ── Operation config (from setup page) ───────────────────────

    def set_operation_config(self, op_type, config):
        """Receive operation type + config from the setup page.

        This determines what kind of PickPlaceOperation is created when
        the user clicks to add a target.
        """
        self._current_op_type = op_type
        self._current_config = config
        # Extract destination well from config if present
        if hasattr(config, 'dest_well'):
            self._dest_well = config.dest_well

    def get_queue(self) -> OperationQueue:
        """Return the current operation queue."""
        return self._queue

    def set_camera_manager(self, manager):
        """v7.3.3: Set the shared CameraManager (can be called after init)."""
        self._camera_manager = manager
        manager.cameras_detected.connect(self._on_cameras_detected)
        manager.camera_started.connect(self._on_camera_started)
        # Update the live feed view's manager reference
        if hasattr(self, '_live_feed_view'):
            self._live_feed_view._manager = manager
            self._live_feed_view._connect_camera()
        self._refresh_camera_combo()

    def _on_cameras_detected(self, num_sources: int):
        """v7.3.3: Called when CameraManager detects cameras (from any page)."""
        self._refresh_camera_combo()
        self._refresh_source_combo()

    def _refresh_camera_combo(self):
        """Rebuild camera combo from CameraManager."""
        mgr = self._camera_manager
        combo = getattr(self, '_ctx_camera_combo', None)
        if combo is None:
            return
        combo.blockSignals(True)
        combo.clear()
        if mgr and mgr.is_available:
            for i in range(mgr.max_cameras):
                combo.addItem(f"Camera {i + 1}", i)
        if combo.count() == 0:
            combo.setPlaceholderText("No cameras available")
        combo.blockSignals(False)
        self._sync_cam_button_state()

    def _refresh_source_combo(self):
        """Rebuild source combo from CameraManager detected sources."""
        mgr = self._camera_manager
        combo = getattr(self, '_ctx_source_combo', None)
        if combo is None or mgr is None:
            return
        combo.blockSignals(True)
        prev_data = combo.currentData()
        combo.clear()
        combo.addItem("— None —", None)
        for text, data in mgr.available_sources:
            combo.addItem(text, data)
        # Restore or match current camera's source
        current_src = mgr.get_source(self._active_cam_idx)
        if current_src:
            idx = combo.findData(current_src)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        elif prev_data:
            idx = combo.findData(prev_data)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _on_cam_selected(self, index: int):
        """User selected a different camera."""
        data = self._ctx_camera_combo.currentData()
        if data is not None:
            self._active_cam_idx = data
            # Switch the CameraFeedView to the new camera
            if hasattr(self, '_live_feed_view'):
                self._live_feed_view.set_camera(data)
        self._sync_cam_button_state()
        self._refresh_source_combo()

    def _sync_cam_button_state(self):
        """Update Start/Stop buttons based on selected camera state."""
        mgr = self._camera_manager
        if mgr is None or not mgr.is_available:
            self._ctx_btn_start_cam.setEnabled(False)
            self._ctx_btn_stop_cam.setEnabled(False)
            return
        running = mgr.is_running(self._active_cam_idx)
        self._ctx_btn_start_cam.setEnabled(not running)
        self._ctx_btn_stop_cam.setEnabled(running)
        self._ctx_btn_start_cam.setText("Stop" if running else "Start")

    def _start_camera(self):
        """Start the selected camera via CameraManager."""
        mgr = self._camera_manager
        if mgr is None:
            return
        # Assign source from combo before starting
        src_combo = getattr(self, '_ctx_source_combo', None)
        if src_combo:
            src_data = src_combo.currentData()
            if src_data:
                mgr.set_source(self._active_cam_idx, src_data)
        mgr.start(self._active_cam_idx)
        self._sync_cam_button_state()
        logger.info(f"Started camera {self._active_cam_idx + 1} from target selection")

    def _stop_camera(self):
        """Stop the selected camera via CameraManager."""
        mgr = self._camera_manager
        if mgr is None:
            return
        mgr.stop(self._active_cam_idx)
        self._sync_cam_button_state()
        logger.info(f"Stopped camera {self._active_cam_idx + 1} from target selection")

    def on_status_update(self):
        """Called by mode page on timer — update stage position and overlays.

        Live camera frames are handled automatically by CameraFeedView's
        signal subscription — no polling needed. We only push position
        and target data here.
        """
        if self.controller:
            try:
                xy = self.controller.get_xy_position(cached=True)
                if xy[0] is not None:
                    self._image_view.set_camera_position(xy[0], xy[1])
                    # Feed position to overlay view for target rendering
                    self._live_feed_view.set_stage_position(xy[0], xy[1])
            except Exception:
                pass

        # Keep overlay view in sync with targets and um_per_px
        mgr = self._camera_manager
        if mgr:
            self._live_feed_view.set_um_per_px(
                mgr.get_um_per_px(self._active_cam_idx))
        self._live_feed_view.set_targets(self._targets)

    def set_hardware_config(self, config):
        """Receive hardware config — extract camera settings."""
        self._hardware_config = config
        if config and hasattr(config, 'camera_config'):
            cam = config.camera_config
            if hasattr(cam, 'micron_per_pixel') and cam.micron_per_pixel > 0:
                self._um_per_px = cam.micron_per_pixel

    # ── Stitcher management ──────────────────────────────────────

    def get_or_create_stitcher(self) -> StitchedImage:
        """Get or create the shared stitcher instance."""
        if self._stitcher is None:
            self._stitcher = StitchedImage(self._um_per_px)
            self._image_view.set_stitcher(self._stitcher)
        return self._stitcher

    def get_stitcher(self) -> Optional[StitchedImage]:
        return self._stitcher

    # ── Target management ────────────────────────────────────────

    def get_targets(self) -> list[PickPlaceTarget]:
        return list(self._targets)

    def _on_target_added(self, x_um: float, y_um: float):
        """Handle click-to-add target from image view.

        Also creates a PickPlaceOperation using the current operation
        config (from the setup page) and adds it to the queue.
        """
        self._target_counter += 1
        tid = f"T{self._target_counter:03d}"
        default_size = self._spin_target_size.value() if hasattr(self, '_spin_target_size') else 200.0

        # Determine which well this point is in (simplified — uses current well)
        well_name = self._well_combo.currentText() or "unknown"

        target = PickPlaceTarget(
            target_id=tid,
            x_um=x_um,
            y_um=y_um,
            well_name=well_name,
            size_um=default_size,
        )

        # Convert to pixel coords in stitcher if available
        if self._stitcher:
            px, py = self._stitcher.stage_to_pixel(x_um, y_um)
            target.pixel_x = px
            target.pixel_y = py

        self._targets.append(target)

        # Auto-create operation for this target
        if self._current_op_type and self._current_config:
            dest = None
            if self._dest_well:
                dest = PickPlaceTarget(
                    target_id=f"DEST_{tid}",
                    x_um=0, y_um=0,
                    well_name=self._dest_well,
                )
            op = PickPlaceOperation(
                op_id=PickPlaceOperation.make_id(),
                op_type=self._current_op_type,
                source_target=target,
                dest_target=dest,
                config=self._current_config,
            )
            self._queue.add(op)
            self.queue_changed.emit(self._queue)

        self._update_target_list()
        self._image_view.set_targets(self._targets)
        self.targets_changed.emit(self._targets)
        logger.info(f"Target added: {tid} at ({x_um:.1f}, {y_um:.1f}) um")

    def _on_target_removed(self, target_id: str):
        """Handle right-click remove from image view."""
        self._targets = [t for t in self._targets if t.target_id != target_id]
        # Also remove corresponding operation from queue
        for op in self._queue.operations:
            if op.source_target.target_id == target_id:
                self._queue.remove(op.op_id)
                break
        self._update_target_list()
        self._image_view.set_targets(self._targets)
        self.targets_changed.emit(self._targets)
        self.queue_changed.emit(self._queue)
        logger.info(f"Target removed: {target_id}")

    def _on_target_hovered(self, target_id: str):
        """Highlight target in list when hovered in image."""
        for i in range(self._target_list.count()):
            item = self._target_list.item(i)
            if item and item.data(Qt.UserRole) == target_id:
                self._target_list.setCurrentItem(item)
                break

    def _on_target_selected(self, current, previous):
        """Highlight target in image when selected in list."""
        if current:
            tid = current.data(Qt.UserRole)
            self._image_view.set_selected_target(tid)
        else:
            self._image_view.set_selected_target(None)

    def _remove_selected_target(self):
        """Remove the currently selected target."""
        item = self._target_list.currentItem()
        if item:
            tid = item.data(Qt.UserRole)
            self._on_target_removed(tid)

    def _clear_all_targets(self):
        """Clear all targets and the operation queue."""
        self._targets.clear()
        self._target_counter = 0
        self._queue.clear()
        self._update_target_list()
        self._image_view.set_targets(self._targets)
        self.targets_changed.emit(self._targets)
        self.queue_changed.emit(self._queue)

    def _update_target_list(self):
        """Rebuild the target list widget."""
        self._target_list.clear()
        for t in self._targets:
            label = f"{t.target_id}  ({t.x_um:.1f}, {t.y_um:.1f})  ({t.size_um:.0f}µm)  {t.well_name}"
            if t.label:
                label += f"  [{t.label}]"
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, t.target_id)
            if not t.selected:
                item.setForeground(QColor(COLORS["overlay0"]))
            self._target_list.addItem(item)
        self._lbl_count.setText(f"{len(self._targets)} targets")

    # ── Scan operations ──────────────────────────────────────────

    def _manual_capture(self):
        """Capture the current camera frame at the current stage position."""
        if not self.controller:
            logger.warning("No controller — cannot capture")
            return

        stitcher = self.get_or_create_stitcher()

        # Get current frame from camera (via calibration page or camera widget)
        frame = self._get_current_frame()
        if frame is None:
            logger.warning("No camera frame available")
            return

        # Get current stage position
        try:
            xy = self.controller.get_xy_position(cached=True)
            if xy[0] is None:
                logger.warning("No XY position available")
                return
            x_um, y_um = xy[0], xy[1]
        except Exception as e:
            logger.error(f"Failed to get position: {e}")
            return

        stitcher.add_tile(frame, x_um, y_um, time.monotonic())

        # Update view
        composite = stitcher.get_composite()
        if composite is not None:
            self._image_view.set_composite(composite)

        logger.info(f"Captured tile at ({x_um:.1f}, {y_um:.1f}) µm, "
                     f"total tiles: {stitcher.tile_count}")

    def _start_auto_scan(self):
        """Start an automatic scan of the selected well."""
        well = self._well_combo.currentText()
        if not well:
            logger.warning("No well selected for auto-scan")
            return

        if self._scanning:
            logger.warning("Scan already in progress")
            return

        # TODO: Resolve well center position and diameter from hardware config
        # For now, this is a placeholder that will be wired up when
        # well positions are available from calibration
        logger.info(f"Auto-scan requested for well {well}")
        # self._scan_thread = threading.Thread(target=self._auto_scan_worker, ...)

    def _get_current_frame(self) -> Optional[np.ndarray]:
        """Get the current camera frame from the active camera via CameraManager.

        Returns BGR numpy array or None.
        """
        mgr = self._camera_manager
        if mgr is None:
            return None
        return mgr.get_current_frame(self._active_cam_idx)

    # ── Well list management ─────────────────────────────────────

    def set_well_list(self, wells: list[str]):
        """Update the well selector combo box."""
        self._well_combo.clear()
        self._well_combo.addItems(wells)

    def set_well_positions(self, positions: dict[str, tuple[float, float]]):
        """Store well positions for coordinate resolution."""
        self._well_positions = positions
