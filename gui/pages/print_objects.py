"""
print_objects.py — Tab 2: Print Objects for MEBP v7.1.

Design parametric print objects and build print collections:
- Object designer form (type, parameters, ink assignment)
- Parametric preview generation via GeometryEngine
- Object library (add/edit/delete/duplicate)
- Print collection builder (ordered list with positions)
- L-shaped projection preview (XY large, ZY right, XZ bottom)
- CSV trajectory import
- Print simulation playback
- Color coding by ink assignment

Session D — Tasks P5.10–P5.18.
"""

from __future__ import annotations

import logging
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QListWidget, QListWidgetItem,
    QSplitter, QScrollArea, QSizePolicy, QLineEdit, QFormLayout,
    QColorDialog, QSlider, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QTimer, QMimeData
from PySide6.QtGui import QColor, QDrag

from gui.styles import COLORS

logger = logging.getLogger(__name__)

# Try importing backend modules (graceful if not yet available)
try:
    from SupportClasses.GeometryEngine import (
        PrintObject, PrintCollection, ObjectType,
        OBJECT_TYPE_INFO, get_default_params, get_available_object_types,
        generate_object_trajectory,
    )
    HAS_GEOMETRY = True
except ImportError:
    HAS_GEOMETRY = False
    logger.warning("GeometryEngine not available — preview disabled")

try:
    from SupportClasses.PhysicalModels import WorkspaceConfig, InkSpec
    HAS_MODELS = True
except ImportError:
    HAS_MODELS = False

try:
    from gui.widgets.projection_canvas import (
        ProjectionCanvas, ObjectPath, create_well_preview,
        InteractiveProjectionCanvas, PlacedObject,
        OBJECT_MIME_TYPE, create_interactive_well_preview,
    )
    HAS_PROJECTION_CANVAS = True
except ImportError:
    HAS_PROJECTION_CANVAS = False
    logger.warning("ProjectionCanvas not available — using placeholder")

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# ═══════════════════════════════════════════════════════════════════
# Object type parameters (from GeometryEngine catalog or fallback)
# ═══════════════════════════════════════════════════════════════════

def _build_type_catalog() -> tuple[dict, dict]:
    """Build (type_names, default_params) from GeometryEngine or fallback."""
    if HAS_GEOMETRY:
        info = get_available_object_types()
        names = {k: v.get("label", k) for k, v in info.items()}
        params = {k: dict(v.get("params", {})) for k, v in info.items()}
        # Always include CSV import option
        names["csv_import"] = "CSV Trajectory Import"
        params["csv_import"] = {}
        return names, params

    # Fallback when GeometryEngine is not importable
    names = {
        "point": "Point (single drop)",
        "line": "Line",
        "circle": "Circle",
        "spiral": "Spiral (2D)",
        "cylinder_solid": "Cylinder (solid fill)",
        "cylinder_shell": "Cylinder (shell only)",
        "csv_import": "CSV Trajectory Import",
    }
    params = {
        "point": {"cx": 0.0, "cy": 0.0, "dwell_time_s": 1.0,
                  "dispense_volume_uL": 0.1},
        "line": {"x1": -1.0, "y1": 0.0, "x2": 1.0, "y2": 0.0,
                 "num_points": 50},
        "circle": {"radius": 1.0, "num_points": 64},
        "spiral": {"max_radius": 2.0, "num_points_per_turn": 64},
        "cylinder_solid": {"radius": 1.0, "height": 2.0,
                           "layer_height": 0.2},
        "cylinder_shell": {"radius": 1.0, "height": 2.0,
                           "layer_height": 0.2, "num_points": 64},
        "csv_import": {},
    }
    return names, params

OBJECT_TYPE_NAMES, DEFAULT_PARAMS = _build_type_catalog()

# Default ink colors for preview
DEFAULT_COLORS = [
    "#a6e3a1",  # green
    "#89b4fa",  # blue
    "#f5c2e7",  # pink
    "#f9e2af",  # yellow
    "#cba6f7",  # mauve
    "#94e2d5",  # teal
]


# ═══════════════════════════════════════════════════════════════════
# Drag-enabled Library List
# ═══════════════════════════════════════════════════════════════════

class DragLibraryList(QListWidget):
    """
    QListWidget that starts a drag with OBJECT_MIME_TYPE payload
    containing the library object name.  Drop targets (the
    InteractiveProjectionPane) accept this MIME type.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setDefaultDropAction(Qt.DropAction.CopyAction)

    def startDrag(self, supportedActions):
        item = self.currentItem()
        if not item:
            return
        # Extract library key (name before " (type)")
        name = item.text().split(" (")[0]
        mime = QMimeData()
        if HAS_PROJECTION_CANVAS:
            mime.setData(OBJECT_MIME_TYPE, name.encode("utf-8"))
        else:
            mime.setText(name)
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.DropAction.CopyAction)


# ═══════════════════════════════════════════════════════════════════
# Print Objects Tab
# ═══════════════════════════════════════════════════════════════════

class PrintObjectsTab(QWidget):
    """
    Tab 2: Design parametric print objects and build collections.

    Layout:
    ┌───────────────────────┬─────────────────────────────┐
    │  Object Designer      │  Projection Preview         │
    │  + Object Library     │  (XY large, ZY right,       │
    │  + Collection Builder │   XZ bottom)                │
    └───────────────────────┴─────────────────────────────┘
    """

    # Emitted when print collections change (names list for Well Setup tab)
    collections_changed = Signal(list)  # list[str]

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # Workspace (set by Tab 1)
        self._workspace = WorkspaceConfig() if HAS_MODELS else None

        # Object library: name → PrintObject template
        self._object_library: dict[str, dict] = {}

        # Named collections: name → PrintCollection
        self._collections: dict[str, object] = {}
        self._active_collection_name: str = "Default"

        # Simulation state
        self._sim_timer = QTimer(self)
        self._sim_timer.setInterval(50)  # 20 FPS
        self._sim_timer.timeout.connect(self._sim_step)
        self._sim_index = 0
        self._sim_playing = False

        self._build_ui()
        self._update_well_diameter()

    # ── Public API ────────────────────────────────────────────────

    def set_workspace(self, workspace) -> None:
        """Update workspace config (called when Tab 1 changes)."""
        self._workspace = workspace
        self._refresh_ink_combos()
        self._update_well_diameter()

    def get_collections(self) -> dict:
        """Return all named PrintCollections for job building."""
        return dict(self._collections)

    def on_status_update(self):
        """Called by parent tab timer."""
        pass

    def _update_well_diameter(self) -> None:
        """Read plate format from workspace and set well boundary on preview."""
        if not (HAS_PROJECTION_CANVAS
                and isinstance(self._preview, ProjectionCanvas)):
            return
        diameter = 10.0  # sensible default for 6-well plate
        if self._workspace and hasattr(self._workspace, 'plate_format'):
            try:
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                fmt = self._workspace.plate_format
                if fmt in PLATE_DEFINITIONS:
                    diameter = PLATE_DEFINITIONS[fmt].get(
                        "well_diameter_mm", 10.0)
            except ImportError:
                pass
        self._preview.set_well_diameter(diameter)
        self._preview.refresh()

    # ── UI Construction ───────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # ── Left panel: Designer + Library + Collection ───────────
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(4, 2, 4, 2)
        left_layout.setSpacing(4)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll_content = QWidget()
        self._left_content = QVBoxLayout(scroll_content)
        self._left_content.setContentsMargins(0, 0, 0, 0)
        self._left_content.setSpacing(6)

        self._build_designer_section()
        self._build_library_section()
        self._build_collection_section()
        self._build_arrangement_section()
        self._build_csv_section()

        self._left_content.addStretch()
        scroll.setWidget(scroll_content)
        left_layout.addWidget(scroll)
        splitter.addWidget(left)

        # ── Right panel: Projection Preview ───────────────────────
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(2, 2, 2, 2)
        right_layout.setSpacing(4)

        preview_label = QLabel("Preview")
        preview_label.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 12px;")
        right_layout.addWidget(preview_label)

        if HAS_PROJECTION_CANVAS:
            self._preview = create_interactive_well_preview()
            self._preview.set_library_resolver(self._resolve_library_object)
            self._preview.object_placed.connect(self._on_object_placed)
            self._preview.object_moved.connect(self._on_object_moved)
            self._preview.object_removed.connect(self._on_object_removed)
        else:
            self._preview = QLabel("Preview unavailable\n(projection_canvas.py missing)")
            self._preview.setAlignment(Qt.AlignCenter)
            self._preview.setStyleSheet(
                f"color: {COLORS['subtext0']}; "
                f"background: {COLORS['mantle']}; "
                f"border-radius: 6px; min-height: 200px;")
        right_layout.addWidget(self._preview, stretch=1)

        # Tip label
        tip = QLabel("Drag objects from Library → drop into XY preview to arrange")
        tip.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10px; font-style: italic;")
        tip.setWordWrap(True)
        right_layout.addWidget(tip)

        # Simulation controls
        sim_row = QHBoxLayout()
        self.btn_sim = QPushButton("▶ Simulate")
        self.btn_sim.setMaximumHeight(28)
        self.btn_sim.clicked.connect(self._toggle_simulation)
        sim_row.addWidget(self.btn_sim)

        sim_row.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 20)
        self.speed_slider.setValue(5)
        self.speed_slider.setMaximumWidth(100)
        sim_row.addWidget(self.speed_slider)

        self.btn_clear_preview = QPushButton("Clear")
        self.btn_clear_preview.setMaximumHeight(28)
        self.btn_clear_preview.clicked.connect(self._clear_preview)
        sim_row.addWidget(self.btn_clear_preview)
        sim_row.addStretch()
        right_layout.addLayout(sim_row)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        outer.addWidget(splitter)

    # ── Object Designer Section ───────────────────────────────────

    def _build_designer_section(self):
        group = QGroupBox("Object Designer")
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QFormLayout(group)
        layout.setSpacing(4)

        # Object name
        self.name_edit = QLineEdit("Object_1")
        layout.addRow("Name:", self.name_edit)

        # Type selector
        self.type_combo = QComboBox()
        for key, label in OBJECT_TYPE_NAMES.items():
            self.type_combo.addItem(label, key)
        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        layout.addRow("Type:", self.type_combo)

        # Dynamic parameter area
        self._param_container = QWidget()
        self._param_layout = QFormLayout(self._param_container)
        self._param_layout.setSpacing(3)
        self._param_layout.setContentsMargins(0, 0, 0, 0)
        layout.addRow(self._param_container)

        # Ink assignment
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("P1 (default)", "P1")
        self.ink_combo.addItem("P2", "P2")
        self.ink_combo.addItem("P3", "P3")
        layout.addRow("Ink/Pump:", self.ink_combo)

        # Color
        color_row = QHBoxLayout()
        self._color = DEFAULT_COLORS[0]
        self.color_btn = QPushButton("  ")
        self.color_btn.setFixedSize(24, 24)
        self.color_btn.setStyleSheet(
            f"background: {self._color}; border-radius: 4px;")
        self.color_btn.clicked.connect(self._pick_color)
        color_row.addWidget(self.color_btn)
        color_row.addWidget(QLabel("Preview color"))
        color_row.addStretch()
        layout.addRow("Color:", color_row)

        # Position offsets
        pos_row = QHBoxLayout()
        self.pos_x = QDoubleSpinBox()
        self.pos_x.setRange(-50, 50)
        self.pos_x.setDecimals(2)
        self.pos_x.setSuffix(" mm")
        pos_row.addWidget(QLabel("X:"))
        pos_row.addWidget(self.pos_x)

        self.pos_y = QDoubleSpinBox()
        self.pos_y.setRange(-50, 50)
        self.pos_y.setDecimals(2)
        self.pos_y.setSuffix(" mm")
        pos_row.addWidget(QLabel("Y:"))
        pos_row.addWidget(self.pos_y)

        self.pos_z = QDoubleSpinBox()
        self.pos_z.setRange(-20, 20)
        self.pos_z.setDecimals(2)
        self.pos_z.setSuffix(" mm")
        pos_row.addWidget(QLabel("Z:"))
        pos_row.addWidget(self.pos_z)
        layout.addRow("Position:", pos_row)

        # Layers (for 3D objects)
        layer_row = QHBoxLayout()
        self.obj_layers = QSpinBox()
        self.obj_layers.setRange(1, 100)
        self.obj_layers.setValue(1)
        layer_row.addWidget(self.obj_layers)

        self.obj_layer_h = QDoubleSpinBox()
        self.obj_layer_h.setRange(0.01, 5.0)
        self.obj_layer_h.setValue(0.2)
        self.obj_layer_h.setSuffix(" mm")
        layer_row.addWidget(QLabel("H:"))
        layer_row.addWidget(self.obj_layer_h)
        layout.addRow("Layers:", layer_row)

        # Generate button
        btn_row = QHBoxLayout()
        self.btn_generate = QPushButton("🔄 Generate Preview")
        self.btn_generate.setObjectName("accentBtn")
        self.btn_generate.clicked.connect(self._generate_preview)
        btn_row.addWidget(self.btn_generate)

        self.btn_add_lib = QPushButton("📚 Add to Library")
        self.btn_add_lib.clicked.connect(self._add_to_library)
        btn_row.addWidget(self.btn_add_lib)
        layout.addRow(btn_row)

        # Info label
        self.obj_info = QLabel("")
        self.obj_info.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 10px;")
        self.obj_info.setWordWrap(True)
        layout.addRow(self.obj_info)

        self._left_content.addWidget(group)

        # Initialize parameters for first type
        self._param_spins = {}
        self._on_type_changed()

    # ── Object Library Section ────────────────────────────────────

    def _build_library_section(self):
        group = QGroupBox("Object Library")
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)

        self.library_list = DragLibraryList()
        self.library_list.setMaximumHeight(100)
        self.library_list.currentRowChanged.connect(self._on_library_select)
        layout.addWidget(self.library_list)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(3)

        btn_load = QPushButton("Load")
        btn_load.setMaximumHeight(24)
        btn_load.setToolTip("Load selected into designer")
        btn_load.clicked.connect(self._load_from_library)
        btn_row.addWidget(btn_load)

        btn_dup = QPushButton("Dup")
        btn_dup.setMaximumHeight(24)
        btn_dup.setToolTip("Duplicate selected")
        btn_dup.clicked.connect(self._duplicate_library_item)
        btn_row.addWidget(btn_dup)

        btn_del = QPushButton("Del")
        btn_del.setMaximumHeight(24)
        btn_del.clicked.connect(self._delete_library_item)
        btn_row.addWidget(btn_del)

        btn_row.addStretch()
        layout.addLayout(btn_row)

        self._left_content.addWidget(group)

    # ── Print Collection Section ──────────────────────────────────

    def _build_collection_section(self):
        group = QGroupBox("Print Collection")
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)

        # Collection name + selector
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self.coll_name_edit = QLineEdit("Default")
        name_row.addWidget(self.coll_name_edit)

        btn_new_coll = QPushButton("New")
        btn_new_coll.setMaximumHeight(24)
        btn_new_coll.clicked.connect(self._new_collection)
        name_row.addWidget(btn_new_coll)
        layout.addLayout(name_row)

        # Collection selector
        coll_sel_row = QHBoxLayout()
        coll_sel_row.addWidget(QLabel("Active:"))
        self.coll_selector = QComboBox()
        self.coll_selector.addItem("Default")
        self.coll_selector.currentTextChanged.connect(self._on_collection_selected)
        coll_sel_row.addWidget(self.coll_selector, 1)
        layout.addLayout(coll_sel_row)

        # Objects in collection
        self.coll_list = QListWidget()
        self.coll_list.setMaximumHeight(100)
        self.coll_list.setDragDropMode(
            QAbstractItemView.DragDropMode.InternalMove)
        layout.addWidget(self.coll_list)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(3)

        btn_add = QPushButton("+Add")
        btn_add.setMaximumHeight(24)
        btn_add.setToolTip("Add selected library object to collection")
        btn_add.clicked.connect(self._add_to_collection)
        btn_row.addWidget(btn_add)

        btn_up = QPushButton("↑")
        btn_up.setMaximumHeight(24)
        btn_up.clicked.connect(self._move_up)
        btn_row.addWidget(btn_up)

        btn_down = QPushButton("↓")
        btn_down.setMaximumHeight(24)
        btn_down.clicked.connect(self._move_down)
        btn_row.addWidget(btn_down)

        btn_rm = QPushButton("Del")
        btn_rm.setMaximumHeight(24)
        btn_rm.clicked.connect(self._remove_from_collection)
        btn_row.addWidget(btn_rm)

        btn_row.addStretch()
        layout.addLayout(btn_row)

        # Collection stats
        self.coll_info = QLabel("")
        self.coll_info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px;")
        self.coll_info.setWordWrap(True)
        layout.addWidget(self.coll_info)

        self._left_content.addWidget(group)

    # ── Well Arrangement Section ──────────────────────────────────

    def _build_arrangement_section(self):
        group = QGroupBox("Well Arrangement")
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)

        info = QLabel(
            "Drag objects from the Library into the XY preview to "
            "arrange them within the well. Reposition by dragging. "
            "Right-click to remove.")
        info.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        # Placed objects summary
        self.arrangement_list = QListWidget()
        self.arrangement_list.setMaximumHeight(80)
        layout.addWidget(self.arrangement_list)

        btn_row = QHBoxLayout()

        btn_create = QPushButton("Create Print from Arrangement")
        btn_create.setObjectName("accentBtn")
        btn_create.setToolTip(
            "Bundle all placed objects into a named print collection")
        btn_create.clicked.connect(self._create_from_arrangement)
        btn_row.addWidget(btn_create)

        btn_clear_arr = QPushButton("Clear All")
        btn_clear_arr.setMaximumWidth(70)
        btn_clear_arr.clicked.connect(self._clear_arrangement)
        btn_row.addWidget(btn_clear_arr)

        layout.addLayout(btn_row)

        self.arrangement_info = QLabel("")
        self.arrangement_info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px;")
        self.arrangement_info.setWordWrap(True)
        layout.addWidget(self.arrangement_info)

        self._left_content.addWidget(group)

    # ── Drag-drop callbacks ───────────────────────────────────────

    def _resolve_library_object(
        self, library_key: str, x_mm: float, y_mm: float,
    ):
        """
        Resolve a library key into a PlacedObject with trajectory points.

        Called by InteractiveProjectionPane when an object is dropped.
        Returns a PlacedObject or None.
        """
        entry = self._object_library.get(library_key)
        if entry is None:
            return None

        color = entry.get("color", DEFAULT_COLORS[0])

        # Try to generate trajectory points
        points = []
        if HAS_GEOMETRY:
            try:
                obj = self._build_print_object(
                    name=library_key,
                    obj_type=entry["object_type"],
                    params=entry.get("params", {}),
                    position=(0, 0, 0),  # centered, offset applied by item
                    color=color,
                    pump_id=entry.get("ink_pump", "P1"),
                )
                if obj and obj.has_trajectory and HAS_NUMPY:
                    traj = obj.trajectory
                    points = [
                        (float(traj[i, 0]),
                         float(traj[i, 1]),
                         float(traj[i, 2]))
                        for i in range(len(traj))
                    ]
            except Exception as e:
                logger.warning(
                    f"Failed to generate trajectory for drop: {e}")

        # Fall back: check for stored _print_object
        if not points:
            po = entry.get("_print_object")
            if po and hasattr(po, "trajectory") and po.trajectory is not None:
                traj = po.trajectory
                points = [
                    (float(traj[i, 0]),
                     float(traj[i, 1]),
                     float(traj[i, 2]))
                    for i in range(len(traj))
                ]

        # Still no points? Create a small marker
        if not points:
            points = [(0.0, 0.0, 0.0)]

        placed = PlacedObject(
            name=library_key,
            color=color,
            points=points,
            x_offset=x_mm,
            y_offset=y_mm,
            z_offset=0.0,
            library_key=library_key,
        )
        return placed

    def _on_object_placed(self, name: str, x_mm: float, y_mm: float):
        """Handle object dropped into preview."""
        self._refresh_arrangement_list()
        self.arrangement_info.setText(
            f"Placed '{name}' at ({x_mm:.1f}, {y_mm:.1f})")

    def _on_object_moved(self, name: str, x_mm: float, y_mm: float):
        """Handle object repositioned by drag."""
        self._refresh_arrangement_list()
        self.arrangement_info.setText(
            f"Moved '{name}' → ({x_mm:.1f}, {y_mm:.1f})")

    def _on_object_removed(self, name: str):
        """Handle object removed via right-click."""
        self._refresh_arrangement_list()
        self.arrangement_info.setText(f"Removed '{name}'")

    def _refresh_arrangement_list(self):
        """Update the arrangement summary list."""
        self.arrangement_list.clear()
        if not (HAS_PROJECTION_CANVAS
                and isinstance(self._preview, InteractiveProjectionCanvas)):
            return
        placed = self._preview.get_placed_objects()
        for p in placed:
            self.arrangement_list.addItem(
                f"{p.name} @ ({p.x_offset:.1f}, {p.y_offset:.1f})")

    def _clear_arrangement(self):
        """Remove all placed objects from the preview."""
        if (HAS_PROJECTION_CANVAS
                and isinstance(self._preview, InteractiveProjectionCanvas)):
            self._preview.clear_placed_objects()
        self._refresh_arrangement_list()
        self.arrangement_info.setText("Cleared arrangement")

    def _create_from_arrangement(self):
        """
        Bundle all placed objects into a named print collection.

        Each placed object keeps its user-set XY offset as the position
        within the collection, so when assigned to a well the whole
        arrangement is reproduced.
        """
        if not (HAS_PROJECTION_CANVAS
                and isinstance(self._preview, InteractiveProjectionCanvas)):
            self.arrangement_info.setText("Preview not available")
            return

        placed = self._preview.get_placed_objects()
        if not placed:
            self.arrangement_info.setText("No objects in arrangement")
            return

        # Use active collection name (or generate one)
        coll_name = self.coll_name_edit.text().strip()
        if not coll_name:
            coll_name = f"Arrangement_{len(self._collections) + 1}"
            self.coll_name_edit.setText(coll_name)

        # Build collection
        if HAS_GEOMETRY:
            coll = PrintCollection(name=coll_name)
            for p in placed:
                entry = self._object_library.get(p.library_key, {})
                obj = PrintObject(
                    name=p.name,
                    object_type=entry.get("object_type", "point"),
                    params=entry.get("params", {}),
                    position=(p.x_offset, p.y_offset, p.z_offset),
                    color=p.color,
                    ink_assignments={
                        entry.get("ink_pump", "P1"): "ink"},
                    num_layers=entry.get("num_layers", 1),
                )
                # Copy trajectory with offset applied
                if p.points:
                    import numpy as np
                    pts = p.offset_points
                    traj = np.array([
                        [pt[0], pt[1], pt[2], 0.0, 0.0, 0.0, 0.0]
                        for pt in pts
                    ])
                    # Add time column (evenly spaced)
                    if len(traj) > 1:
                        traj[:, 6] = np.linspace(
                            0, len(traj) * 0.01, len(traj))
                    obj.trajectory = traj
                coll.add_object(obj)
            self._collections[coll_name] = coll
        else:
            # Fallback dict-based collection
            objs = []
            for p in placed:
                entry = dict(self._object_library.get(
                    p.library_key, {}))
                entry["position"] = (
                    p.x_offset, p.y_offset, p.z_offset)
                entry["name"] = p.name
                objs.append(entry)
            self._collections[coll_name] = {
                "name": coll_name, "objects": objs}

        # Update UI
        if self.coll_selector.findText(coll_name) < 0:
            self.coll_selector.addItem(coll_name)
        self.coll_selector.setCurrentText(coll_name)
        self._refresh_collection_list()
        self._emit_collections()

        n = len(placed)
        self.arrangement_info.setText(
            f"✅ Created '{coll_name}' with {n} object(s)")

    # ── CSV Import Section ────────────────────────────────────────

    def _build_csv_section(self):
        group = QGroupBox("CSV Trajectory Import")
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)

        info = QLabel(
            "Import CSV with columns: x, y, z, p1, p2, p3, t\n"
            "Units: mm for positions, seconds for time.")
        info.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 10px;")
        info.setWordWrap(True)
        layout.addWidget(info)

        btn_import = QPushButton("📂 Import CSV Trajectory")
        btn_import.clicked.connect(self._import_csv)
        layout.addWidget(btn_import)

        self.csv_info = QLabel("")
        self.csv_info.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px;")
        self.csv_info.setWordWrap(True)
        layout.addWidget(self.csv_info)

        self._left_content.addWidget(group)

    # ════════════════════════════════════════════════════════════════
    #  PARAMETER MANAGEMENT
    # ════════════════════════════════════════════════════════════════

    def _on_type_changed(self, _index=0):
        """Rebuild parameter form for selected object type."""
        obj_type = self.type_combo.currentData()
        if obj_type is None:
            return

        # Clear existing params
        while self._param_layout.count():
            item = self._param_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._param_spins.clear()

        # CSV type has no parameters (imported from file)
        if obj_type == "csv_import":
            lbl = QLabel("Use 'Import CSV' below to load trajectory data")
            lbl.setStyleSheet(f"color: {COLORS['overlay0']}; font-style: italic;")
            self._param_layout.addRow(lbl)
            return

        params = DEFAULT_PARAMS.get(obj_type, {})
        for key, default in params.items():
            label = key.replace("_", " ").title()
            if isinstance(default, float):
                spin = QDoubleSpinBox()
                spin.setRange(0.01, 100.0)
                spin.setDecimals(2)
                spin.setValue(default)
                if "mm" in key:
                    spin.setSuffix(" mm")
                elif "deg" in key:
                    spin.setSuffix("°")
                    spin.setRange(-360, 360)
                elif "percent" in key or "overlap" in key:
                    spin.setSuffix("%")
                    spin.setRange(0, 100)
            elif isinstance(default, int):
                spin = QSpinBox()
                spin.setRange(1, 1000)
                spin.setValue(default)
            elif isinstance(default, str):
                spin = QComboBox()
                # Fill patterns
                if "pattern" in key:
                    spin.addItems(["spiral", "meander", "concentric"])
                else:
                    spin = QLineEdit(default)
            else:
                continue

            self._param_spins[key] = spin
            self._param_layout.addRow(f"{label}:", spin)

        # Show/hide layers based on 3D type
        is_3d = "cylinder" in obj_type
        self.obj_layers.setEnabled(is_3d)
        self.obj_layer_h.setEnabled(is_3d)
        if is_3d and self.obj_layers.value() == 1:
            self.obj_layers.setValue(10)

    def _get_params(self) -> dict:
        """Read current parameter values from form."""
        params = {}
        for key, widget in self._param_spins.items():
            if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
                params[key] = widget.value()
            elif isinstance(widget, QComboBox):
                params[key] = widget.currentText()
            elif isinstance(widget, QLineEdit):
                params[key] = widget.text()
        return params

    # ════════════════════════════════════════════════════════════════
    #  OBJECT GENERATION + PREVIEW
    # ════════════════════════════════════════════════════════════════

    def _extract_needle_syringe(self) -> tuple:
        """Extract (needle, syringe_map, settings) from workspace."""
        needle = None
        syringe_map = {}
        speed = 5.0
        layer_h = 0.2

        if self._workspace:
            needle = getattr(self._workspace, 'needle', None)
            for pid, pump in getattr(self._workspace, 'pumps', {}).items():
                if hasattr(pump, 'syringe') and pump.syringe is not None:
                    syringe_map[pid] = pump.syringe
            ps = getattr(self._workspace, 'print_settings', {})
            speed = ps.get('print_speed_mm_s', 5.0)
            layer_h = ps.get('layer_height_mm', 0.2)

        # Fallback: create minimal needle if workspace has none
        if needle is None:
            try:
                from SupportClasses.PhysicalModels import NeedleSpec
                needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
            except ImportError:
                pass

        return needle, syringe_map, speed, layer_h

    def _build_print_object(
        self,
        name: str,
        obj_type: str,
        params: dict,
        position: tuple = (0, 0, 0),
        color: str = "#a6e3a1",
        pump_id: str = "P1",
    ) -> PrintObject | None:
        """
        Create a PrintObject and generate its trajectory.

        Returns the PrintObject with .trajectory populated, or None on failure.
        """
        if not HAS_GEOMETRY:
            return None

        obj = PrintObject(
            name=name,
            object_type=obj_type,
            params=dict(params),
            position=position,
            ink_assignments={pump_id: "ink"},
            color=color,
        )

        needle, syringe_map, speed, layer_h = self._extract_needle_syringe()
        if needle is None:
            logger.warning("No needle configured — cannot generate trajectory")
            return obj  # Return object without trajectory

        try:
            generate_object_trajectory(
                obj, needle, syringe_map,
                print_speed_mm_s=speed,
                layer_height_mm=layer_h,
                pump_id=pump_id,
            )
        except Exception as e:
            logger.error(f"Trajectory generation failed for '{name}': {e}",
                         exc_info=True)

        return obj

    def _generate_preview(self):
        """Generate a print object and show in projection view."""
        obj_type = self.type_combo.currentData()
        name = self.name_edit.text().strip() or "Unnamed"
        params = self._get_params()
        position = (self.pos_x.value(), self.pos_y.value(), self.pos_z.value())
        color = self._color
        pump = self.ink_combo.currentData() or "P1"

        if not HAS_GEOMETRY:
            self.obj_info.setText("GeometryEngine not available")
            return

        try:
            obj = self._build_print_object(
                name, obj_type, params, position, color, pump)

            if obj and obj.has_trajectory:
                self._show_trajectory_preview(obj)
                self.obj_info.setText(
                    f"✅ {obj.num_waypoints} waypoints | "
                    f"{obj.total_length_mm:.1f} mm path | "
                    f"{obj.total_time_s:.1f} s | "
                    f"{obj.num_layers} layer(s)")
            else:
                self.obj_info.setText("⚠ No trajectory generated")

        except Exception as e:
            self.obj_info.setText(f"Error: {e}")
            logger.error(f"Object generation failed: {e}", exc_info=True)

    def _show_trajectory_preview(self, obj):
        """Render object trajectory in the projection canvas."""
        if not HAS_PROJECTION_CANVAS or not isinstance(self._preview, ProjectionCanvas):
            return

        self._preview.clear_path()

        if not HAS_NUMPY or not obj.has_trajectory:
            return

        traj = obj.trajectory
        # Build an ObjectPath with the object's color
        pts = [(float(traj[i, 0]), float(traj[i, 1]), float(traj[i, 2]))
               for i in range(len(traj))]
        color = getattr(obj, 'color', "#a6e3a1")
        obj_path = ObjectPath(name=obj.name, color=color, points=pts)
        self._preview.set_object_paths([obj_path])

        # Ensure well boundary is current
        self._update_well_diameter()

    # ════════════════════════════════════════════════════════════════
    #  OBJECT LIBRARY
    # ════════════════════════════════════════════════════════════════

    def _add_to_library(self):
        """Add current designer config to library."""
        name = self.name_edit.text().strip()
        if not name:
            return

        # Store as dict for recreation
        entry = {
            "name": name,
            "object_type": self.type_combo.currentData(),
            "params": self._get_params(),
            "position": (self.pos_x.value(), self.pos_y.value(), self.pos_z.value()),
            "color": self._color,
            "ink_pump": self.ink_combo.currentData() or "P1",
            "num_layers": self.obj_layers.value(),
            "layer_height": self.obj_layer_h.value(),
        }

        self._object_library[name] = entry
        self._refresh_library_list()
        self.obj_info.setText(f"Added '{name}' to library")

    def _load_from_library(self):
        """Load selected library item into designer."""
        item = self.library_list.currentItem()
        if not item:
            return
        name = item.text().split(" (")[0]
        entry = self._object_library.get(name)
        if not entry:
            return

        self.name_edit.setText(entry["name"])
        idx = self.type_combo.findData(entry["object_type"])
        if idx >= 0:
            self.type_combo.setCurrentIndex(idx)
        # Params will be set after type change rebuilds form
        QTimer.singleShot(50, lambda: self._apply_params(entry))

    def _apply_params(self, entry: dict):
        """Apply stored parameters to form."""
        for key, val in entry.get("params", {}).items():
            spin = self._param_spins.get(key)
            if spin and isinstance(spin, (QDoubleSpinBox, QSpinBox)):
                spin.setValue(val)
            elif spin and isinstance(spin, QComboBox):
                idx = spin.findText(str(val))
                if idx >= 0:
                    spin.setCurrentIndex(idx)

        pos = entry.get("position", (0, 0, 0))
        self.pos_x.setValue(pos[0])
        self.pos_y.setValue(pos[1])
        self.pos_z.setValue(pos[2])
        self._color = entry.get("color", DEFAULT_COLORS[0])
        self.color_btn.setStyleSheet(
            f"background: {self._color}; border-radius: 4px;")
        self.obj_layers.setValue(entry.get("num_layers", 1))
        self.obj_layer_h.setValue(entry.get("layer_height", 0.2))

    def _duplicate_library_item(self):
        item = self.library_list.currentItem()
        if not item:
            return
        name = item.text().split(" (")[0]
        entry = self._object_library.get(name)
        if entry:
            new_name = f"{name}_copy"
            new_entry = dict(entry)
            new_entry["name"] = new_name
            self._object_library[new_name] = new_entry
            self._refresh_library_list()

    def _delete_library_item(self):
        item = self.library_list.currentItem()
        if not item:
            return
        name = item.text().split(" (")[0]
        self._object_library.pop(name, None)
        self._refresh_library_list()

    def _on_library_select(self, row: int):
        """Show preview of selected library item (helps before dragging)."""
        if row < 0:
            return
        item = self.library_list.item(row)
        if not item:
            return
        name = item.text().split(" (")[0]
        entry = self._object_library.get(name)
        if not entry or not HAS_GEOMETRY:
            return
        try:
            obj = self._build_print_object(
                name=name,
                obj_type=entry["object_type"],
                params=entry.get("params", {}),
                position=entry.get("position", (0, 0, 0)),
                color=entry.get("color", DEFAULT_COLORS[0]),
                pump_id=entry.get("ink_pump", "P1"),
            )
            if obj and obj.has_trajectory:
                self._show_trajectory_preview(obj)
        except Exception:
            pass  # Non-critical

    def _refresh_library_list(self):
        self.library_list.clear()
        for name, entry in self._object_library.items():
            type_name = OBJECT_TYPE_NAMES.get(
                entry["object_type"], entry["object_type"])
            self.library_list.addItem(f"{name} ({type_name})")

    # ════════════════════════════════════════════════════════════════
    #  PRINT COLLECTION
    # ════════════════════════════════════════════════════════════════

    def _new_collection(self):
        """Create a new named collection."""
        name = self.coll_name_edit.text().strip()
        if not name:
            return
        if name not in self._collections:
            if HAS_GEOMETRY:
                self._collections[name] = PrintCollection(name=name)
            else:
                self._collections[name] = {"name": name, "objects": []}
            self.coll_selector.addItem(name)
            self.coll_selector.setCurrentText(name)
            self._emit_collections()

    def _on_collection_selected(self, name: str):
        self._active_collection_name = name
        self._refresh_collection_list()

    def _add_to_collection(self):
        """Add selected library item to active collection."""
        item = self.library_list.currentItem()
        if not item:
            self.coll_info.setText("Select a library object first")
            return

        name = item.text().split(" (")[0]
        entry = self._object_library.get(name)
        if not entry:
            return

        coll_name = self._active_collection_name
        if coll_name not in self._collections:
            self._new_collection()

        coll = self._collections.get(coll_name)
        if coll is None:
            return

        if HAS_GEOMETRY and isinstance(coll, PrintCollection):
            obj = PrintObject(
                name=entry["name"],
                object_type=entry["object_type"],
                params=entry.get("params", {}),
                position=entry.get("position", (0, 0, 0)),
                color=entry.get("color", DEFAULT_COLORS[0]),
                ink_assignments={entry.get("ink_pump", "P1"): "ink"},
                num_layers=entry.get("num_layers", 1),
            )
            coll.add_object(obj)
        elif isinstance(coll, dict):
            coll["objects"].append(dict(entry))

        self._refresh_collection_list()
        self._emit_collections()

    def _remove_from_collection(self):
        row = self.coll_list.currentRow()
        if row < 0:
            return
        coll = self._collections.get(self._active_collection_name)
        if coll is None:
            return

        if HAS_GEOMETRY and isinstance(coll, PrintCollection):
            coll.remove_object(row)
        elif isinstance(coll, dict):
            objects = coll.get("objects", [])
            if 0 <= row < len(objects):
                objects.pop(row)

        self._refresh_collection_list()
        self._emit_collections()

    def _move_up(self):
        row = self.coll_list.currentRow()
        if row <= 0:
            return
        coll = self._collections.get(self._active_collection_name)
        if HAS_GEOMETRY and isinstance(coll, PrintCollection):
            coll.move_object(row, row - 1)
        self._refresh_collection_list()
        self.coll_list.setCurrentRow(row - 1)

    def _move_down(self):
        row = self.coll_list.currentRow()
        coll = self._collections.get(self._active_collection_name)
        max_idx = self.coll_list.count() - 1
        if row < 0 or row >= max_idx:
            return
        if HAS_GEOMETRY and isinstance(coll, PrintCollection):
            coll.move_object(row, row + 1)
        self._refresh_collection_list()
        self.coll_list.setCurrentRow(row + 1)

    def _refresh_collection_list(self):
        self.coll_list.clear()
        coll = self._collections.get(self._active_collection_name)
        if coll is None:
            return

        objects = []
        if HAS_GEOMETRY and isinstance(coll, PrintCollection):
            objects = coll.objects
        elif isinstance(coll, dict):
            objects = coll.get("objects", [])

        for i, obj in enumerate(objects):
            if HAS_GEOMETRY and isinstance(obj, PrintObject):
                pos = obj.position
                label = f"{i+1}. {obj.name} @ ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
            elif isinstance(obj, dict):
                pos = obj.get("position", (0, 0, 0))
                label = f"{i+1}. {obj['name']} @ ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
            else:
                label = f"{i+1}. {obj}"
            self.coll_list.addItem(label)

        num = len(objects)
        self.coll_info.setText(
            f"{num} object(s) in '{self._active_collection_name}'")

    def _emit_collections(self):
        """Emit collection names for Well Setup tab."""
        names = list(self._collections.keys())
        self.collections_changed.emit(names)

    # ════════════════════════════════════════════════════════════════
    #  CSV IMPORT
    # ════════════════════════════════════════════════════════════════

    def _import_csv(self):
        """Import a CSV trajectory file as a custom object."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Import CSV Trajectory", "",
            "CSV Files (*.csv);;TSV Files (*.tsv);;All Files (*)")
        if not path:
            return

        try:
            if HAS_NUMPY:
                import pandas as pd
            else:
                self.csv_info.setText("numpy/pandas required for CSV import")
                return

            df = pd.read_csv(path)

            # Validate columns
            required = {"x", "y", "z", "t"}
            optional = {"p1", "p2", "p3"}
            found = set(c.lower().strip() for c in df.columns)

            if not required.issubset(found):
                missing = required - found
                self.csv_info.setText(f"Missing columns: {missing}")
                return

            # Build trajectory array [x, y, z, p1, p2, p3, t]
            col_map = {c.lower().strip(): c for c in df.columns}
            traj = np.zeros((len(df), 7), dtype=np.float64)
            traj[:, 0] = df[col_map["x"]].values
            traj[:, 1] = df[col_map["y"]].values
            traj[:, 2] = df[col_map["z"]].values
            traj[:, 6] = df[col_map["t"]].values
            for i, p in enumerate(["p1", "p2", "p3"]):
                if p in col_map:
                    traj[:, 3 + i] = df[col_map[p]].values

            # Create a PrintObject with this trajectory
            name = Path(path).stem
            if HAS_GEOMETRY:
                obj = PrintObject(
                    name=name,
                    object_type="csv_import",
                    params={"source_file": str(path)},
                    color=DEFAULT_COLORS[len(self._object_library) % len(DEFAULT_COLORS)],
                )
                obj.trajectory = traj
                obj.total_length_mm = float(np.sum(np.sqrt(
                    np.diff(traj[:, 0])**2 + np.diff(traj[:, 1])**2
                )))
                obj.total_time_s = float(traj[-1, 6] - traj[0, 6])
                obj.num_layers = 1

                # Add to library
                self._object_library[name] = {
                    "name": name,
                    "object_type": "csv_import",
                    "params": {"source_file": str(path)},
                    "position": (0, 0, 0),
                    "color": obj.color,
                    "ink_pump": "P1",
                    "num_layers": 1,
                    "layer_height": 0.2,
                    "_print_object": obj,  # Store generated object
                }
                self._refresh_library_list()

                # Show preview
                self._show_trajectory_preview(obj)
                self.csv_info.setText(
                    f"✅ Imported: {len(traj)} waypoints, "
                    f"{obj.total_length_mm:.1f} mm, "
                    f"{obj.total_time_s:.1f} s")
            else:
                self.csv_info.setText(
                    f"Loaded {len(traj)} rows (GeometryEngine unavailable)")

        except Exception as e:
            self.csv_info.setText(f"Import error: {e}")
            logger.error(f"CSV import failed: {e}", exc_info=True)

    # ════════════════════════════════════════════════════════════════
    #  SIMULATION PLAYBACK
    # ════════════════════════════════════════════════════════════════

    def _toggle_simulation(self):
        if self._sim_playing:
            self._stop_simulation()
        else:
            self._start_simulation()

    def _start_simulation(self):
        """Start animated playback of active collection."""
        if not HAS_PROJECTION_CANVAS or not isinstance(self._preview, ProjectionCanvas):
            return

        # Collect all trajectories from active collection
        self._sim_trajectories = []
        coll = self._collections.get(self._active_collection_name)
        if coll and HAS_GEOMETRY and isinstance(coll, PrintCollection):
            for obj in coll.objects:
                if obj.has_trajectory:
                    self._sim_trajectories.append(obj.trajectory)
        elif coll and isinstance(coll, dict):
            for entry in coll.get("objects", []):
                po = entry.get("_print_object")
                if po and hasattr(po, "trajectory") and po.trajectory is not None:
                    self._sim_trajectories.append(po.trajectory)

        if not self._sim_trajectories:
            self.obj_info.setText("No trajectory data to simulate")
            return

        # Merge all trajectories
        if HAS_NUMPY:
            self._sim_merged = np.vstack(self._sim_trajectories)
        else:
            return

        self._sim_index = 0
        self._sim_playing = True
        self._preview.clear_path()
        self.btn_sim.setText("⏹ Stop")
        self._sim_timer.start()

    def _stop_simulation(self):
        self._sim_playing = False
        self._sim_timer.stop()
        self.btn_sim.setText("▶ Simulate")

    def _sim_step(self):
        """Advance simulation by speed_slider steps."""
        if not self._sim_playing or not HAS_NUMPY:
            return

        steps_per_frame = self.speed_slider.value()
        data = self._sim_merged

        for _ in range(steps_per_frame):
            if self._sim_index >= len(data):
                self._stop_simulation()
                return

            row = data[self._sim_index]
            x, y, z = float(row[0]), float(row[1]), float(row[2])
            self._preview.add_completed_point(x, y, z)
            self._preview.set_needle_position(x, y, z)

            # Show upcoming waypoints
            future = min(self._sim_index + 20, len(data))
            upcoming = [
                (float(data[j, 0]), float(data[j, 1]), float(data[j, 2]))
                for j in range(self._sim_index + 1, future)
            ]
            self._preview.set_upcoming_waypoints(upcoming)

            self._sim_index += 1

        self._preview.refresh()

    # ════════════════════════════════════════════════════════════════
    #  HELPERS
    # ════════════════════════════════════════════════════════════════

    def _pick_color(self):
        color = QColorDialog.getColor(
            QColor(self._color), self, "Object Color")
        if color.isValid():
            self._color = color.name()
            self.color_btn.setStyleSheet(
                f"background: {self._color}; border-radius: 4px;")

    def _clear_preview(self):
        if HAS_PROJECTION_CANVAS and isinstance(self._preview, ProjectionCanvas):
            self._preview.clear_path()
            if isinstance(self._preview, InteractiveProjectionCanvas):
                self._preview.clear_placed_objects()
                self._refresh_arrangement_list()
            self._preview.refresh()

    def _refresh_ink_combos(self):
        """Update ink combo from workspace ink library."""
        current = self.ink_combo.currentData()
        self.ink_combo.clear()

        if self._workspace and hasattr(self._workspace, 'pumps'):
            for pid, pump in self._workspace.pumps.items():
                ink_name = ""
                if hasattr(pump, 'fluid_column') and pump.fluid_column:
                    ink_name = getattr(pump.fluid_column, 'ink_name', '') or ''
                label = f"{pid}: {ink_name}" if ink_name else pid
                self.ink_combo.addItem(label, pid)
        else:
            self.ink_combo.addItem("P1 (default)", "P1")
            self.ink_combo.addItem("P2", "P2")
            self.ink_combo.addItem("P3", "P3")

        # Restore selection
        idx = self.ink_combo.findData(current)
        if idx >= 0:
            self.ink_combo.setCurrentIndex(idx)