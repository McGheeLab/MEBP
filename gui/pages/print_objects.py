"""
print_objects.py — Tab 2: Print Objects Designer for MEBP v7.2.3.

File-centric workflow redesign (Section 3.4):
- Print File Bar: New / Load / Save / Duplicate / Delete with auto-save
- Object type icon buttons (not dropdown)
- Dynamic parameter panel per type (QStackedWidget)
- Live preview with 100ms debounce
- Single objects list (replaces library + collection)
- Edit-in-place mode (select → edit → Update Object)
- Auto-layout: Ring, Grid, Hex, Line, Concentric patterns
- CSV Import as first-class action
- Print summary widget
- prints_changed signal for Tab 3 integration
- Auto-save on every change (500ms debounce)

Layout:
    ┌─────────────────────────────────────────────────────────────────┐
    │ [+ New Print]  Name: [Scaffold_v1]  │ Saved: [▾] [Load][Dup]  │
    ├──────────────────────┬──────────────────────────────────────────┤
    │ Object Designer      │ Well Preview (L-shaped projection)       │
    │ [●][╱][◯][◎][▦][📄]│                                           │
    │ Parameters (dynamic) │                                          │
    │ [Add Object]         ├──────────────────────────────────────────┤
    │                      │ Objects in This Print:                    │
    │ Auto-Layout:         │ 1. ● Base Scaffold P1 @ (0,0,0)         │
    │ Pattern: [Ring ▾]    │ [▲][▼][Edit][Dup][✕]                    │
    │ [Apply] [Clear]      │ Summary: 3 obj | P1,P2 | ~45s | 2.3µL  │
    └──────────────────────┴──────────────────────────────────────────┘
"""

from __future__ import annotations

import logging
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QSpinBox,
    QFileDialog, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QListWidget, QListWidgetItem,
    QSplitter, QScrollArea, QSizePolicy, QLineEdit, QFormLayout,
    QColorDialog, QSlider, QMessageBox, QStackedWidget,
    QInputDialog, QToolBar, QCheckBox,
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QFont, QIcon

from gui.styles import COLORS
from gui.scaling import s as _sc, scaled_font_size

# v7.2.4: XY-only well preview with zoom/pan/bounds (S4.2)
try:
    from gui.widgets.well_preview import WellPreviewWidget, ObjectPath as WPObjectPath
    HAS_WELL_PREVIEW = True
except ImportError:
    HAS_WELL_PREVIEW = False


logger = logging.getLogger(__name__)

# ── Optional imports (graceful fallback) ──────────────────────────
try:
    from SupportClasses.GeometryEngine import (
        PrintObject, PrintCollection,
        OBJECT_TYPE_INFO, get_default_params, get_available_object_types,
        generate_object_trajectory,
    )
    HAS_GEOMETRY = True
except ImportError:
    HAS_GEOMETRY = False
    logger.warning("GeometryEngine not available — preview disabled")

try:
    from SupportClasses.PhysicalModels import WorkspaceConfig, InkSpec, NeedleSpec
    HAS_MODELS = True
except ImportError:
    HAS_MODELS = False

try:
    from SupportClasses.PrintFileManager import (
        PrintFileManager, PrintFileData, PrintFileMetadata,
        validate_print_file, migrate_print_file,
    )
    HAS_FILE_MANAGER = True
except ImportError:
    HAS_FILE_MANAGER = False
    logger.warning("PrintFileManager not available — using in-memory only")

try:
    from SupportClasses.auto_layout import (
        generate_layout, validate_layout, LAYOUT_INFO,
    )
    HAS_AUTO_LAYOUT = True
except ImportError:
    HAS_AUTO_LAYOUT = False
    logger.warning("auto_layout not available")

try:
    from gui.widgets.projection_canvas import (
        ProjectionCanvas, ObjectPath, InteractiveProjectionCanvas,
        PlacedObject, create_interactive_well_preview,
        create_horizontal_well_preview,
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
# Constants
# ═══════════════════════════════════════════════════════════════════

# Object type definitions: key → (label, icon_text)
# Consolidated: no shell/solid split — the filled checkbox handles that.
_ICON_MAP = {
    "point": "●", "line": "╱", "circle": "◯", "square": "□",
    "triangle": "△", "spiral": "◎", "ellipse": "⬭",
    "sphere": "◉", "cube": "◼", "cylinder": "⬤", "ellipsoid": "⬬",
    "csv_import": "📄",
}

# Category for each consolidated type
_TYPE_CATEGORY = {
    "point": "1D",
    "line": "2D", "circle": "2D", "square": "2D",
    "triangle": "2D", "spiral": "2D", "ellipse": "2D",
    "sphere": "3D", "cube": "3D", "cylinder": "3D", "ellipsoid": "3D",
    "csv_import": "Import",
}

OBJECT_TYPES: dict[str, tuple[str, str]] = {
    "point":     ("Point",     "●"),
    "line":      ("Line",      "╱"),
    "circle":    ("Circle",    "◯"),
    "square":    ("Square",    "□"),
    "triangle":  ("Triangle",  "△"),
    "spiral":    ("Spiral",    "◎"),
    "ellipse":   ("Ellipse",   "⬭"),
    "sphere":    ("Sphere",    "◉"),
    "cube":      ("Cube",      "◼"),
    "cylinder":  ("Cylinder",  "⬤"),
    "ellipsoid": ("Ellipsoid", "⬬"),
    "csv_import": ("CSV Import", "📄"),
}

# Map consolidated 3D GUI type → engine type based on filled flag
_3D_TYPE_MAP = {
    ("sphere", True): "sphere_solid",    ("sphere", False): "sphere_shell",
    ("cube", True): "cube_solid",        ("cube", False): "cube_shell",
    ("cylinder", True): "cylinder_solid", ("cylinder", False): "cylinder_shell",
    ("ellipsoid", True): "ellipsoid_solid", ("ellipsoid", False): "ellipsoid_shell",
}

# Parameter names that must be integers (count/index values)
INT_PARAMS = {
    "num_points", "num_points_per_turn", "points_per_side",
    "count", "rows", "cols", "n", "num_layers",
}

DEFAULT_COLORS = [
    "#a6e3a1", "#89b4fa", "#f5c2e7", "#f9e2af", "#cba6f7", "#94e2d5",
]


def _normalize_file_list(raw_list) -> list[str]:
    """Normalize PrintFileManager.list_files() output to list of name strings.

    list_files() may return list[str] or list[dict] depending on version.
    This helper handles both gracefully.
    """
    names = []
    for item in (raw_list or []):
        if isinstance(item, str):
            names.append(item)
        elif isinstance(item, dict):
            # Try common key names for the print file name
            name = item.get("name") or item.get("print_name") or item.get("filename", "")
            if name:
                names.append(str(name))
        else:
            names.append(str(item))
    return names

# Auto-incrementing name counters
_type_counters: dict[str, int] = {}


def _auto_name(obj_type: str) -> str:
    """Generate auto-incrementing name like 'Dot_1', 'Line_2'."""
    label = OBJECT_TYPES.get(obj_type, ("Obj", "?"))[0]
    _type_counters[obj_type] = _type_counters.get(obj_type, 0) + 1
    return f"{label}_{_type_counters[obj_type]}"


# Default params for consolidated 3D types (superset of shell+solid params)
_CONSOLIDATED_3D_PARAMS = {
    "sphere":    {"radius": 1.0, "layer_height": 0.2, "num_points": 64},
    "cube":      {"side": 2.0, "height": 2.0, "layer_height": 0.2, "points_per_side": 20},
    "cylinder":  {"radius": 1.0, "height": 2.0, "layer_height": 0.2, "num_points": 64},
    "ellipsoid": {"a": 2.0, "b": 1.5, "c": 1.0, "layer_height": 0.2, "num_points": 64},
}


def _get_type_params(obj_type: str) -> dict:
    """Get default parameters for an object type from GeometryEngine."""
    # Consolidated 3D types — use local defaults
    if obj_type in _CONSOLIDATED_3D_PARAMS:
        return dict(_CONSOLIDATED_3D_PARAMS[obj_type])
    if HAS_GEOMETRY:
        return get_default_params(obj_type)
    # Fallback defaults
    fallbacks = {
        "point": {"cx": 0.0, "cy": 0.0, "dwell_time_s": 1.0, "dispense_volume_uL": 0.1},
        "line": {"x1": -1.0, "y1": 0.0, "x2": 1.0, "y2": 0.0, "num_points": 50},
        "circle": {"radius": 1.0, "num_points": 64},
        "square": {"side": 2.0, "points_per_side": 20},
        "triangle": {"side": 2.0, "points_per_side": 20},
        "spiral": {"max_radius": 2.0, "num_points_per_turn": 64},
        "ellipse": {"a": 2.0, "b": 1.0, "num_points": 64},
        "csv_import": {},
    }
    return dict(fallbacks.get(obj_type, {}))


# ═══════════════════════════════════════════════════════════════════
# PrintObjectsTab — Complete Rewrite for v7.2.3
# ═══════════════════════════════════════════════════════════════════

class PrintObjectsTab(QWidget):
    """
    Tab 2: File-centric print object designer.

    Manages persistent print files in config/prints/ with auto-save,
    streamlined object creation via icon buttons and dynamic parameters,
    auto-layout patterns, and integration with Tab 3 via prints_changed.
    """

    # Replaces collections_changed — emits list of print file names
    prints_changed = Signal(list)
    # Emitted on file load/save/new
    file_changed = Signal(str)
    # Legacy compatibility — forwards to prints_changed
    collections_changed = Signal(list)

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # Workspace (set by Tab 1)
        self._workspace = WorkspaceConfig() if HAS_MODELS else None
        self._hw_config = None

        # Print file manager
        self._file_manager = None
        if HAS_FILE_MANAGER:
            prints_dir = Path("config/prints")
            prints_dir.mkdir(parents=True, exist_ok=True)
            self._file_manager = PrintFileManager(prints_dir)

        # Current print file data (in-memory)
        self._current_file: PrintFileData | None = None
        self._active_file_name: str | None = None

        # Objects list: ordered list of dicts
        # Each: {name, object_type, params, position, color, ink, auto_layout, in_well}
        self._objects: list[dict] = []

        # Edit mode tracking
        self._editing_index: int | None = None  # None = add mode, int = edit mode

        # v7.2.6: Staging list + OOB flash timers
        self._staging_print_names: list[str] = []
        self._flash_timers: dict[int, QTimer] = {}

        # Auto-save timer
        self._auto_save_timer = QTimer(self)
        self._auto_save_timer.setSingleShot(True)
        self._auto_save_timer.setInterval(500)
        self._auto_save_timer.timeout.connect(self._do_auto_save)

        # Preview update timer (100ms debounce)
        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(100)
        self._preview_timer.timeout.connect(self._update_live_preview)

        # Designer preview blink (not-yet-added indicator)
        self._designer_blink_timer = QTimer(self)
        self._designer_blink_timer.setInterval(600)
        self._designer_blink_timer.timeout.connect(self._toggle_designer_blink)
        self._designer_blink_dim = False
        self._designer_preview_obj = None  # cached for blink redraws

        # Simulation
        self._sim_timer = QTimer(self)
        self._sim_timer.setInterval(50)
        self._sim_timer.timeout.connect(self._sim_step)
        self._sim_index = 0
        self._sim_playing = False

        self._build_ui()

        # v7.2.4: Out-of-bounds detection (S4.9-S4.10)
        self._oob_indices: set[int] = set()
        self._oob_flash_state: bool = False
        self._oob_flash_timer = QTimer(self)
        self._oob_flash_timer.setInterval(500)
        self._oob_flash_timer.timeout.connect(self._toggle_oob_flash)

        self._restore_last_print()

    # ══════════════════════════════════════════════════════════════
    #  PUBLIC API
    # ══════════════════════════════════════════════════════════════

    def set_workspace(self, workspace) -> None:
        """Update workspace config (called when Tab 1 changes)."""
        self._workspace = workspace
        self._refresh_ink_pump_combos()
        self._update_well_diameter()

    def set_hardware_config(self, config) -> None:
        """v7.2: Receive hardware config for syringe/needle info."""
        self._hw_config = config
        self._refresh_ink_pump_combos()

    def get_collections(self) -> dict:
        """Legacy: return collections for job building."""
        return self._build_legacy_collections()

    def get_print_file_names(self) -> list[str]:
        """Return list of available print file names."""
        if self._file_manager:
            return _normalize_file_list(self._file_manager.list_files())
        return []

    def on_status_update(self):
        """Called by parent tab timer."""
        pass

    # ══════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ══════════════════════════════════════════════════════════════

    def _make_collapsible_group(self, title: str, layout_target,
                                expanded: bool = True) -> QGroupBox:
        """Create a collapsible QGroupBox with a visible header bar."""
        grp = QGroupBox(title)
        grp.setCheckable(True)
        grp.setChecked(expanded)
        grp.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; font-size: 12px;
                color: {COLORS.get('text', '#cdd6f4')};
                background: {COLORS.get('base', '#1e1e2e')};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: 4px; margin-top: 10px; padding-top: 24px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 0px; right: 0px; top: 0px;
                padding: 6px 10px;
                background: {COLORS.get('surface1', '#45475a')};
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QGroupBox::indicator {{
                width: 12px; height: 12px;
            }}
        """)
        inner = QVBoxLayout(grp)
        inner.setContentsMargins(6, 4, 6, 6)
        inner.setSpacing(4)
        content = QWidget()
        content_lay = QVBoxLayout(content)
        content_lay.setContentsMargins(0, 0, 0, 0)
        content_lay.setSpacing(4)
        inner.addWidget(content)

        def _toggle(checked, w=content):
            w.setVisible(checked)
        grp.toggled.connect(_toggle)
        content.setVisible(expanded)

        layout_target.addWidget(grp)
        return grp, content_lay

    def _build_ui(self):
        _bg = COLORS.get('base', '#1e1e2e')

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.setStyleSheet(f"background: {_bg};")

        # ── Print File Bar (always visible, pinned top) ──────────
        self._build_file_bar(outer)

        # v7.5.3: body is the projection trio with the file bar kept
        # on top. Designer, Ink & Material, Auto-Layout, and CSV
        # Import are built as detached panels that the wizard's
        # context pane reparents into its Tools section.
        body_inner = QWidget()
        body_inner.setStyleSheet(f"background: {_bg};")
        preview_lay = QVBoxLayout(body_inner)
        preview_lay.setContentsMargins(4, 4, 4, 4)
        preview_lay.setSpacing(4)
        self._build_preview_section(preview_lay)
        outer.addWidget(body_inner, 1)

        # ── Detached panels for the wizard sidebar / context ─────
        # All built as standalone widgets that the wizard reparents.
        # ``objects_panel`` (v7.5.1): wizard right sidebar (Objects
        #     in This Print + Summary).
        # ``auto_layout_panel`` (v7.5.2): wizard left context Tools.
        # ``designer_panel`` (v7.5.3): wizard left context Tools —
        #     Object Designer (type list + params + Add button).
        # ``ink_material_panel`` (v7.5.3): wizard left context Tools
        #     — Ink + Filled + Position cluster.
        # v7.5.6: CSV Import is no longer a standalone panel — it's a
        # custom-object type at the top of the designer's type list.
        self._csv_chosen_path: str | None = None
        self.objects_panel = self._build_objects_panel()
        self.designer_panel = self._build_designer_panel()
        self.ink_material_panel = self._build_ink_material_panel()
        self.auto_layout_panel = self._build_auto_layout_panel()

    # ── File Bar ──────────────────────────────────────────────────

    def _build_file_bar(self, parent_layout):
        """Print File Bar: New / Name / Load dropdown / Save / Dup / Delete / Status."""
        bar = QFrame()
        bar.setStyleSheet(
            f"background: {COLORS['surface0']}; border-radius: 4px; padding: 4px;")
        bar_layout = QHBoxLayout(bar)
        bar_layout.setContentsMargins(8, 4, 8, 4)
        bar_layout.setSpacing(8)

        # + New Print
        btn_new = QPushButton("+ New Print")
        btn_new.setStyleSheet(
            f"background: {COLORS['green']}; color: {COLORS['base']}; "
            f"font-weight: bold; padding: 4px 12px; border-radius: 4px;")
        btn_new.clicked.connect(self._on_new_print)
        bar_layout.addWidget(btn_new)

        # Print name
        bar_layout.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Enter print name...")
        self._name_edit.setMaximumWidth(_sc(200))
        self._name_edit.editingFinished.connect(self._on_name_changed)
        bar_layout.addWidget(self._name_edit)

        bar_layout.addWidget(self._separator_v())

        # Load dropdown
        bar_layout.addWidget(QLabel("Saved:"))
        self._file_combo = QComboBox()
        self._file_combo.setMinimumWidth(_sc(150))
        self._file_combo.currentTextChanged.connect(self._on_file_selected)
        bar_layout.addWidget(self._file_combo)

        btn_load = QPushButton("Load")
        btn_load.clicked.connect(self._on_load_print)
        bar_layout.addWidget(btn_load)

        btn_dup = QPushButton("Dup")
        btn_dup.setToolTip("Duplicate current print")
        btn_dup.clicked.connect(self._on_duplicate_print)
        bar_layout.addWidget(btn_dup)

        btn_del = QPushButton("Delete")
        btn_del.setStyleSheet(f"color: {COLORS['red']};")
        btn_del.clicked.connect(self._on_delete_print)
        bar_layout.addWidget(btn_del)

        btn_export = QPushButton("Export")
        btn_export.setToolTip("Export print file as JSON")
        btn_export.clicked.connect(self._on_export_print)
        bar_layout.addWidget(btn_export)

        bar_layout.addStretch()

        # Auto-save status
        self._save_status = QLabel("● No print loaded")
        self._save_status.setStyleSheet(f"color: {COLORS['overlay0']};")
        bar_layout.addWidget(self._save_status)

        parent_layout.addWidget(bar)

    @staticmethod
    def _separator_v():
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet(f"color: {COLORS['surface1']};")
        return sep

    # ── Object Designer ───────────────────────────────────────────

    def _build_designer_section(self, parent_layout):
        layout = parent_layout

        # S4B.1: Object type categorised list (replaces icon button row)
        type_label = QLabel("Object Type:")
        type_label.setStyleSheet(f"font-weight: bold; color: {COLORS['subtext0']};")
        layout.addWidget(type_label)

        self._type_list = QListWidget()
        # Size to always show exactly 6 items regardless of DPI/resolution
        self._type_list.addItem("")  # temp item to measure row height
        row_h = self._type_list.sizeHintForRow(0)
        self._type_list.clear()
        if row_h < 1:
            row_h = 20  # fallback
        margins = self._type_list.contentsMargins()
        frame = self._type_list.frameWidth() * 2
        self._type_list.setFixedHeight(row_h * 6 + margins.top() + margins.bottom() + frame)
        self._type_list.setStyleSheet(f"""
            QListWidget {{
                background: {COLORS['surface0']}; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']}; border-radius: 4px;
                outline: none;
            }}
            QListWidget::item {{
                padding: 2px 6px; border-radius: 3px;
            }}
            QListWidget::item:selected {{
                background: {COLORS['surface2']}; color: {COLORS['blue']};
            }}
            QListWidget::item:hover {{
                background: {COLORS['surface1']};
            }}
        """)

        # Build categorised entries
        self._type_buttons: dict[str, QPushButton] = {}  # kept for compat (unused)
        self._type_list_map: dict[int, str] = {}  # row → obj_type key

        categories = self._categorize_object_types()
        row_idx = 0
        for cat_name, types_in_cat in categories:
            # Section header
            header = QListWidgetItem(f"── {cat_name} ──")
            header.setFlags(Qt.ItemFlag.NoItemFlags)  # not selectable
            header.setForeground(QColor(COLORS['overlay0']))
            font = header.font()
            font.setBold(True)
            font.setPointSize(font.pointSize() - 1)
            header.setFont(font)
            self._type_list.addItem(header)
            row_idx += 1

            for obj_type, (label, icon_text) in types_in_cat:
                item = QListWidgetItem(f"  {icon_text}  {label}")
                item.setData(Qt.ItemDataRole.UserRole, obj_type)
                self._type_list.addItem(item)
                self._type_list_map[row_idx] = obj_type
                row_idx += 1

        # Select first selectable item
        first_type = list(OBJECT_TYPES.keys())[0]
        self._current_type = first_type
        for i in range(self._type_list.count()):
            item = self._type_list.item(i)
            if item.data(Qt.ItemDataRole.UserRole) == first_type:
                self._type_list.setCurrentItem(item)
                break

        self._type_list.currentItemChanged.connect(self._on_type_list_changed)
        layout.addWidget(self._type_list)

        # S4B.2: Dynamic parameters panel (QStackedWidget)
        self._param_stack = QStackedWidget()
        self._param_stack.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        self._param_stack.currentChanged.connect(self._resize_param_stack)
        self._param_widgets: dict[str, dict] = {}  # type → {param_name: widget}
        self._param_stack_indices: dict[str, int] = {}

        for obj_type in OBJECT_TYPES:
            page, widgets = self._build_param_page(obj_type)
            idx = self._param_stack.addWidget(page)
            self._param_widgets[obj_type] = widgets
            self._param_stack_indices[obj_type] = idx

        layout.addWidget(self._param_stack)

        # v7.5.1: ink / filled / position widgets moved to
        # _build_ink_material_section() so they live in a separate
        # fixed-height card and don't reflow the page on toggle.

        # Add/Update Object button
        btn_row = QHBoxLayout()
        self._add_btn = QPushButton("+ Add Object")
        self._add_btn.setStyleSheet(
            f"background: {COLORS['green']}; color: {COLORS['base']}; "
            f"font-weight: bold; padding: 6px 16px; border-radius: 4px;")
        self._add_btn.clicked.connect(self._on_add_or_update)
        btn_row.addWidget(self._add_btn)

        self._cancel_edit_btn = QPushButton("Cancel Edit")
        self._cancel_edit_btn.setVisible(False)
        self._cancel_edit_btn.clicked.connect(self._cancel_edit_mode)
        btn_row.addWidget(self._cancel_edit_btn)

        btn_row.addStretch()

        # Info label
        self._obj_info = QLabel("")
        self._obj_info.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 10px;")
        self._obj_info.setWordWrap(True)
        btn_row.addWidget(self._obj_info)

        layout.addLayout(btn_row)

        # Edit mode indicator
        self._edit_indicator = QLabel("")
        self._edit_indicator.setStyleSheet(
            f"color: {COLORS['yellow']}; font-weight: bold; font-size: 11px;")
        self._edit_indicator.setVisible(False)
        layout.addWidget(self._edit_indicator)

    # ── Ink & Material (v7.5.1) ───────────────────────────────────

    def _build_ink_material_section(self, parent_layout):
        """Ink selector + filled/pattern + outer-ring volume + position.

        Lives in a separate card from the Object Designer so toggling
        Filled never reflows the page.
        """
        form = QFormLayout()
        form.setSpacing(4)

        # ── Ink selector + read-only color swatch ─────────────────
        ink_row = QHBoxLayout()
        self._ink_combo = QComboBox()
        self._ink_combo.addItem("(no inks defined)", "")
        self._ink_combo.currentIndexChanged.connect(self._on_ink_combo_changed)
        ink_row.addWidget(self._ink_combo)

        self._ink_color_swatch = QLabel()
        self._ink_color_swatch.setFixedSize(_sc(24), _sc(24))
        self._current_color = DEFAULT_COLORS[0]
        self._ink_color_swatch.setStyleSheet(
            f"background: {self._current_color}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
        self._ink_color_swatch.setToolTip("Ink color (set in Hardware Setup)")
        ink_row.addWidget(self._ink_color_swatch)
        ink_row.addStretch()
        form.addRow("Ink:", ink_row)

        # ── Filled checkbox + fill pattern combo ─────────────────
        fill_row = QHBoxLayout()
        self._filled_check = QCheckBox("Filled / Solid")
        self._filled_check.setToolTip(
            "2D: fill interior with pattern (and outline pass).\n"
            "3D: solid fill instead of shell.")
        self._filled_check.stateChanged.connect(self._on_filled_toggled)
        fill_row.addWidget(self._filled_check)

        self._fill_pattern_label = QLabel("Pattern:")
        self._fill_pattern_label.setVisible(False)
        fill_row.addWidget(self._fill_pattern_label)
        self._fill_pattern_combo = QComboBox()
        self._fill_pattern_combo.addItem("Meander", "meander")
        self._fill_pattern_combo.addItem("Spiral", "spiral")
        self._fill_pattern_combo.setVisible(False)
        self._fill_pattern_combo.setMaximumWidth(_sc(100))
        self._fill_pattern_combo.currentIndexChanged.connect(
            self._schedule_preview)
        fill_row.addWidget(self._fill_pattern_combo)
        fill_row.addStretch()
        form.addRow("Fill:", fill_row)

        # ── v7.5.1: outer-ring volume (extra ink on the outline) ──
        outer_row = QHBoxLayout()
        self._outer_ring_uL = QDoubleSpinBox()
        self._outer_ring_uL.setRange(0.0, 100.0)
        self._outer_ring_uL.setDecimals(2)
        self._outer_ring_uL.setSingleStep(0.05)
        self._outer_ring_uL.setSuffix(" µL")
        self._outer_ring_uL.setMaximumWidth(_sc(120))
        self._outer_ring_uL.setToolTip(
            "Volume of ink deposited along the outline pass that\n"
            "wraps a filled 2D shape. 0 = use natural extrusion rate.")
        self._outer_ring_uL.valueChanged.connect(self._schedule_preview)
        outer_row.addWidget(self._outer_ring_uL)
        outer_row.addStretch()
        self._outer_ring_label = QLabel("Outer-ring uL:")
        form.addRow(self._outer_ring_label, outer_row)
        # Same visibility rules as fill_pattern
        self._outer_ring_uL.setVisible(False)
        self._outer_ring_label.setVisible(False)

        # ── Position ──────────────────────────────────────────────
        # Compact X/Y/Z row: no " mm" suffix (the label says mm) and
        # narrow spinboxes so all three fit the Tools column width.
        pos_row = QHBoxLayout()
        pos_row.setSpacing(_sc(2))
        self._pos_x = QDoubleSpinBox()
        self._pos_y = QDoubleSpinBox()
        self._pos_z = QDoubleSpinBox()
        for spin, label in [
            (self._pos_x, "X"), (self._pos_y, "Y"), (self._pos_z, "Z"),
        ]:
            spin.setRange(-50.0, 50.0)
            spin.setSingleStep(0.1)
            spin.setDecimals(2)
            spin.setMinimumWidth(0)
            spin.setMaximumWidth(_sc(62))
            spin.setSizePolicy(QSizePolicy.Policy.Expanding,
                               QSizePolicy.Policy.Fixed)
            spin.valueChanged.connect(self._schedule_preview)
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
            pos_row.addWidget(lbl)
            pos_row.addWidget(spin, 1)
        form.addRow("Position (mm):", pos_row)

        parent_layout.addLayout(form)

    # ── Detached panels: Designer / Ink / CSV (v7.5.3) ───────────

    def _build_designer_panel(self) -> QWidget:
        """Object Designer (type list + param stack + Add button) as
        a standalone widget for the wizard's left context Tools."""
        host = QWidget()
        host.setObjectName("printDesignerPanel")
        host.setStyleSheet(
            f"#printDesignerPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(host)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        _, designer_lay = self._make_collapsible_group(
            "Object Designer", lay, expanded=True)
        self._build_designer_section(designer_lay)
        lay.addStretch()
        return host

    def _build_ink_material_panel(self) -> QWidget:
        """Ink & Material (ink combo + filled + outer ring + position)
        as a standalone widget for the wizard's left context Tools."""
        host = QWidget()
        host.setObjectName("printInkMaterialPanel")
        host.setStyleSheet(
            f"#printInkMaterialPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(host)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        _, ink_lay = self._make_collapsible_group(
            "Ink && Material", lay, expanded=True)
        self._build_ink_material_section(ink_lay)
        lay.addStretch()
        return host

    def _build_csv_import_panel(self) -> QWidget:
        """CSV Import as a standalone widget for the wizard's left
        context Tools."""
        host = QWidget()
        host.setObjectName("printCsvImportPanel")
        host.setStyleSheet(
            f"#printCsvImportPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(host)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        _, csv_lay = self._make_collapsible_group(
            "CSV Import", lay, expanded=False)
        self._build_csv_import_section(csv_lay)
        lay.addStretch()
        return host

    # ── Detached Auto-Layout panel (v7.5.2) ──────────────────────

    def _build_auto_layout_panel(self) -> QWidget:
        """Build the Auto-Layout group as a standalone widget that the
        wizard's left context pane reparents into its Step 1 tools
        page. The widget isn't part of *this* tab's body layout."""
        host = QWidget()
        host.setObjectName("printAutoLayoutPanel")
        host.setStyleSheet(
            f"#printAutoLayoutPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(host)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        _, auto_lay = self._make_collapsible_group(
            "Auto-Layout", lay, expanded=True)
        self._build_auto_layout_section(auto_lay)
        lay.addStretch()
        return host

    # ── Detached Objects + Summary panel (v7.5.1) ────────────────

    def _build_objects_panel(self) -> QWidget:
        """Build the right-sidebar panel: Print List + Objects in This
        Print + Summary, stacked top-to-bottom as a standalone widget
        the wizard reparents into its right sidebar.

        Print List (v7.5.3): catalog view of the session's print
        objects + collections — fed by the wizard's PrintObjectsModel
        via :meth:`set_print_list_model`."""
        from PySide6.QtWidgets import QListWidget, QListWidgetItem
        host = QWidget()
        host.setObjectName("printObjectsListPanel")
        host.setStyleSheet(
            f"#printObjectsListPanel {{ background: {COLORS['base']}; }}"
        )
        lay = QVBoxLayout(host)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # ── Print List ────────────────────────────────────────────
        _, list_lay = self._make_collapsible_group(
            "Print List", lay, expanded=True)
        self._print_list_count_label = QLabel("0 objects")
        self._print_list_count_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px;")
        list_lay.addWidget(self._print_list_count_label)
        self._print_list_list = QListWidget()
        self._print_list_list.setMaximumHeight(_sc(140))
        self._print_list_list.setStyleSheet(
            f"QListWidget {{"
            f"  background: {COLORS['mantle']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: 4px;"
            f"}}"
            f"QListWidget::item {{ padding: 2px 6px; }}"
            f"QListWidget::item:selected {{"
            f"  background: {COLORS['surface1']};"
            f"  color: {COLORS['text']};"
            f"}}"
        )
        list_lay.addWidget(self._print_list_list)
        self._print_list_coll_label = QLabel("Collections")
        self._print_list_coll_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px; "
            f"font-weight: 700; padding-top: 4px;")
        list_lay.addWidget(self._print_list_coll_label)
        self._print_list_coll_list = QListWidget()
        self._print_list_coll_list.setMaximumHeight(_sc(80))
        self._print_list_coll_list.setStyleSheet(
            self._print_list_list.styleSheet())
        list_lay.addWidget(self._print_list_coll_list)

        # ── Objects in This Print ─────────────────────────────────
        _, objects_lay = self._make_collapsible_group(
            "Objects in This Print", lay, expanded=True)
        self._build_objects_list_section(objects_lay)

        # ── Summary & Staging ─────────────────────────────────────
        _, summary_lay = self._make_collapsible_group(
            "Summary && Staging", lay, expanded=True)
        self._build_summary_section(summary_lay)

        lay.addStretch()
        return host

    def set_print_list_model(self, model) -> None:
        """v7.5.3: bind the right-pane Print List section to a
        PrintObjectsModel so it reflects the session's objects +
        collections in real time."""
        if not hasattr(self, "_print_list_list"):
            return
        self._print_list_model = model
        model.changed.connect(self._refresh_print_list_section)
        self._refresh_print_list_section()

    def _refresh_print_list_section(self) -> None:
        from PySide6.QtWidgets import QListWidgetItem
        model = getattr(self, "_print_list_model", None)
        if model is None or not hasattr(self, "_print_list_list"):
            return
        self._print_list_list.clear()
        for obj in model.objects():
            name = obj.get("name", "?")
            otype = obj.get("object_type", "")
            ink = obj.get("ink")
            if not ink and isinstance(obj.get("ink_assignments"), dict):
                ink = next(iter(obj["ink_assignments"].values()), "")
            label = name
            if otype:
                label = f"{label}   ·   {otype}"
            if ink:
                label = f"{label}   ·   {ink}"
            self._print_list_list.addItem(QListWidgetItem(label))
        n = self._print_list_list.count()
        self._print_list_count_label.setText(
            "No objects defined yet." if n == 0
            else f"{n} object{'' if n == 1 else 's'}"
        )
        self._print_list_coll_list.clear()
        for coll in model.collections():
            cname = coll.get("name", "?")
            count = len(coll.get("objects", []))
            self._print_list_coll_list.addItem(
                f"{cname}   ·   {count} object{'' if count == 1 else 's'}"
            )

    def _build_param_page(self, obj_type: str) -> tuple[QWidget, dict]:
        """Build a parameter form for one object type. Returns (widget, {name: spinbox})."""
        page = QWidget()
        form = QFormLayout(page)
        form.setSpacing(4)
        widgets = {}

        if obj_type == "csv_import":
            # v7.5.6: CSV import is a custom-object type. The file
            # picker lives right here in the designer; clicking
            # "+ Add Object" imports the chosen file as a custom
            # object.
            choose_btn = QPushButton("Choose CSV File…")
            choose_btn.setStyleSheet(
                f"background: {COLORS['surface1']}; "
                f"padding: 6px 12px; border-radius: 4px;")
            choose_btn.clicked.connect(self._choose_csv_file)
            form.addRow(choose_btn)

            self._csv_chosen_label = QLabel("No file chosen")
            self._csv_chosen_label.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: 10px; "
                f"font-style: italic;")
            self._csv_chosen_label.setWordWrap(True)
            form.addRow(self._csv_chosen_label)

            hint = QLabel("Imports an XYZ(P/T) trajectory as a custom object. "
                          "Then click + Add Object.")
            hint.setStyleSheet(
                f"color: {COLORS['overlay0']}; font-size: 9px;")
            hint.setWordWrap(True)
            form.addRow(hint)
            return page, widgets

        defaults = _get_type_params(obj_type)
        for param_name, default_val in defaults.items():
            # Use QSpinBox for integer params, QDoubleSpinBox for floats
            is_int_param = param_name in INT_PARAMS or isinstance(default_val, int)

            if is_int_param:
                spin = QSpinBox()
                spin.setRange(1, 10000)
                spin.setSingleStep(1)
                spin.setValue(int(default_val))
            else:
                spin = QDoubleSpinBox()
                spin.setDecimals(3)
                spin.setSingleStep(0.1)

                # Set sensible ranges based on param name
                if "angle" in param_name:
                    spin.setRange(-360, 360)
                    spin.setSuffix("°")
                elif "time" in param_name:
                    spin.setRange(0, 300)
                    spin.setSuffix(" s")
                elif "volume" in param_name:
                    spin.setRange(0, 1000)
                    spin.setSuffix(" µL")
                else:
                    spin.setRange(-100, 100)
                    spin.setSuffix(" mm")

                spin.setValue(float(default_val))

            spin.valueChanged.connect(self._schedule_preview)
            # v7.6.0: allow spinboxes to shrink so the param form fits
            # the (sometimes-narrow) left-context column width.
            spin.setMinimumWidth(0)
            spin.setMaximumWidth(_sc(130))
            spin.setSizePolicy(QSizePolicy.Policy.Expanding,
                               QSizePolicy.Policy.Fixed)

            # Pretty label
            label = param_name.replace("_", " ").title()
            form.addRow(f"{label}:", spin)
            widgets[param_name] = spin

        return page, widgets

    # ── Auto-Layout Section ───────────────────────────────────────

    def _build_auto_layout_section(self, parent_layout):
        layout = parent_layout

        # Pattern selector
        pat_row = QHBoxLayout()
        pat_row.addWidget(QLabel("Pattern:"))
        self._layout_pattern = QComboBox()

        patterns = ["ring", "grid", "hex", "line", "concentric"]
        if HAS_AUTO_LAYOUT:
            for p in patterns:
                info = LAYOUT_INFO.get(p, {})
                self._layout_pattern.addItem(info.get("label", p.title()), p)
        else:
            for p in patterns:
                self._layout_pattern.addItem(p.title(), p)

        self._layout_pattern.currentIndexChanged.connect(self._on_layout_pattern_changed)
        pat_row.addWidget(self._layout_pattern)
        pat_row.addStretch()
        layout.addLayout(pat_row)

        # Dynamic layout parameters
        self._layout_param_stack = QStackedWidget()
        self._layout_param_widgets: dict[str, dict] = {}

        for pattern in patterns:
            page, widgets = self._build_layout_param_page(pattern)
            self._layout_param_stack.addWidget(page)
            self._layout_param_widgets[pattern] = widgets

        layout.addWidget(self._layout_param_stack)

        # Object source label
        self._layout_source_label = QLabel("Uses current designer object type + params")
        self._layout_source_label.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10px; font-style: italic;")
        layout.addWidget(self._layout_source_label)

        # Apply / Clear buttons
        btn_row = QHBoxLayout()
        self._btn_apply_layout = QPushButton("Apply Layout")
        self._btn_apply_layout.setStyleSheet(
            f"background: {COLORS['blue']}; color: {COLORS['base']}; "
            f"padding: 4px 12px; border-radius: 4px;")
        self._btn_apply_layout.clicked.connect(self._apply_auto_layout)
        btn_row.addWidget(self._btn_apply_layout)

        self._btn_clear_layout = QPushButton("Clear Layout Objects")
        self._btn_clear_layout.clicked.connect(self._clear_auto_layout)
        btn_row.addWidget(self._btn_clear_layout)

        self._layout_count_label = QLabel("")
        self._layout_count_label.setStyleSheet(f"color: {COLORS['overlay0']};")
        btn_row.addWidget(self._layout_count_label)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _build_layout_param_page(self, pattern: str) -> tuple[QWidget, dict]:
        """Build parameter controls for one auto-layout pattern."""
        page = QWidget()
        form = QFormLayout(page)
        form.setSpacing(4)
        widgets = {}

        if HAS_AUTO_LAYOUT:
            info = LAYOUT_INFO.get(pattern, {})
            param_defs = info.get("params", {})
        else:
            # Fallback definitions
            param_defs = {
                "ring": {"count": 6, "radius": 2.0, "start_angle_deg": 0.0},
                "grid": {"rows": 3, "cols": 3, "spacing": 1.0},
                "hex": {"rows": 3, "cols": 3, "spacing": 1.0},
                "line": {"count": 5, "spacing": 1.0},
                "concentric": {"ring_counts": "6,12", "inner_radius": 1.0, "outer_radius": 3.0},
            }.get(pattern, {})

        for pname, default in param_defs.items():
            if isinstance(default, str):
                # Text field (e.g., ring_counts as comma-separated)
                edit = QLineEdit(default)
                edit.setMaximumWidth(_sc(120))
                form.addRow(f"{pname.replace('_', ' ').title()}:", edit)
                widgets[pname] = edit
            else:
                spin = QDoubleSpinBox()
                spin.setDecimals(2)
                if "count" in pname or "rows" in pname or "cols" in pname:
                    spin.setRange(1, 100)
                    spin.setDecimals(0)
                    spin.setSingleStep(1)
                elif "angle" in pname:
                    spin.setRange(-360, 360)
                    spin.setSuffix("°")
                else:
                    spin.setRange(0.1, 50.0)
                    spin.setSuffix(" mm")
                spin.setValue(float(default))
                form.addRow(f"{pname.replace('_', ' ').title()}:", spin)
                widgets[pname] = spin

        return page, widgets

    # ── CSV Import Section ────────────────────────────────────────

    def _build_csv_import_section(self, parent_layout):
        layout = parent_layout

        btn = QPushButton("Import CSV Trajectory")
        btn.setStyleSheet(
            f"background: {COLORS['surface1']}; padding: 6px 12px; border-radius: 4px;")
        btn.clicked.connect(self._import_csv)
        layout.addWidget(btn)

        self._csv_info = QLabel("Import XYZ(T/P) trajectory from CSV file")
        self._csv_info.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 10px; font-style: italic;")
        layout.addWidget(self._csv_info)

    # ── Preview Section ───────────────────────────────────────────

    def _build_preview_section(self, parent_layout):
        if HAS_PROJECTION_CANVAS:
            # v7.5.1: horizontal trio (XY|XZ|YZ) with pan+zoom locked.
            self._preview = create_horizontal_well_preview()
            self._preview.set_library_resolver(self._resolve_library_object)
            if hasattr(self._preview, 'object_placed'):
                self._preview.object_placed.connect(self._on_object_repositioned)
            if hasattr(self._preview, 'object_moved'):
                self._preview.object_moved.connect(self._on_object_repositioned)
        else:
            self._preview = QLabel(
                "Preview unavailable\n(projection_canvas.py not found)")
            self._preview.setAlignment(Qt.AlignCenter)
            self._preview.setStyleSheet(
                f"color: {COLORS['subtext0']}; background: {COLORS['mantle']}; "
                f"border-radius: 6px; min-height: 200px;")

        parent_layout.addWidget(self._preview, stretch=1)

    # ── Objects List Section (S4B.5) ──────────────────────────────

    def _build_objects_list_section(self, parent_layout):
        layout = parent_layout

        self._objects_list = QListWidget()
        self._objects_list.setMaximumHeight(_sc(200))
        self._objects_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._objects_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self._objects_list.currentRowChanged.connect(self._on_object_selected)
        self._objects_list.model().rowsMoved.connect(self._on_objects_reordered)
        layout.addWidget(self._objects_list)

        # Action buttons — compact row of 5 (Edit/Dup/▲/▼/✕). Each
        # button shares the row width equally so they never overflow
        # the narrow "This Print" pane.
        btn_row = QHBoxLayout()
        btn_row.setSpacing(_sc(2))
        for label, slot, tip in [
            ("Edit", self._edit_selected, "Load into designer for editing"),
            ("Dup", self._duplicate_selected, "Duplicate with offset"),
            ("▲", self._move_up, "Move up in order"),
            ("▼", self._move_down, "Move down in order"),
            ("✕", self._remove_selected, "Remove from print"),
        ]:
            btn = QPushButton(label)
            btn.setToolTip(tip)
            btn.setFixedHeight(_sc(24))
            btn.setSizePolicy(QSizePolicy.Policy.Expanding,
                              QSizePolicy.Policy.Fixed)
            btn.setMinimumWidth(0)
            btn.clicked.connect(slot)
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        # "Remove from Well" on its own full-width row so it never
        # pushes the compact row out of the pane.
        self._btn_remove_from_well = QPushButton("Remove from Well")
        self._btn_remove_from_well.setToolTip(
            "Remove selected object from the well preview")
        self._btn_remove_from_well.setFixedHeight(_sc(24))
        self._btn_remove_from_well.setEnabled(False)
        self._btn_remove_from_well.clicked.connect(self._remove_from_well)
        layout.addWidget(self._btn_remove_from_well)

    # ── Summary Section (S4B.11) ──────────────────────────────────

    def _build_summary_section(self, parent_layout):
        """v7.2.6: Summary + staging list for selective print sending."""
        self._summary_label = QLabel("No objects")
        self._summary_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; background: {COLORS['surface0']}; "
            f"padding: 4px 8px; border-radius: 4px; font-size: 11px;")
        parent_layout.addWidget(self._summary_label)

        # Staging list
        stg_lbl = QLabel("Prints to Send:")
        stg_lbl.setStyleSheet(f"color: {COLORS['blue']}; font-weight: bold; font-size: 10px;")
        parent_layout.addWidget(stg_lbl)

        self._prints_staging_list = QListWidget()
        self._prints_staging_list.setMaximumHeight(_sc(80))
        self._prints_staging_list.setStyleSheet(f"""
            QListWidget {{ background: {COLORS['surface0']}; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']}; border-radius: 4px; font-size: 10px; }}
            QListWidget::item:selected {{ background: {COLORS['surface2']}; }}""")
        parent_layout.addWidget(self._prints_staging_list)

        btns = QHBoxLayout()
        b1 = QPushButton("+ Add Current"); b1.setFixedHeight(_sc(24)); b1.clicked.connect(self._staging_add_current); btns.addWidget(b1)
        b2 = QPushButton("- Remove"); b2.setFixedHeight(_sc(24)); b2.clicked.connect(self._staging_remove_selected); btns.addWidget(b2)
        self._btn_send_to_prints = QPushButton("\U0001f4cb Send"); self._btn_send_to_prints.setFixedHeight(_sc(24))
        self._btn_send_to_prints.setStyleSheet(f"QPushButton {{ background: {COLORS.get('mauve','#cba6f7')}; color: {COLORS.get('base','#1e1e2e')}; font-weight: bold; border-radius: 4px; font-size: 10px; }}")
        self._btn_send_to_prints.clicked.connect(self._send_staged_prints); btns.addWidget(self._btn_send_to_prints)
        btns.addStretch()
        parent_layout.addLayout(btns)


    def _on_new_print(self):
        """Create a new empty print file."""
        name, ok = QInputDialog.getText(
            self, "New Print", "Print name:",
            text=self._next_print_name())
        if not ok or not name.strip():
            return
        name = name.strip()

        self._objects.clear()
        self._editing_index = None
        _type_counters.clear()

        if self._file_manager:
            self._file_manager.new_file(name)
            self._active_file_name = name

        self._name_edit.setText(name)
        self._refresh_objects_list()
        self._refresh_preview_all()
        self._update_summary()
        self._refresh_file_combo()
        self._save_status.setText("✅ New print created")
        self._save_status.setStyleSheet(f"color: {COLORS['green']};")
        self._emit_prints_changed()
        self._staging_add_current()  # v7.2.6
        self.file_changed.emit(name)

    def _on_load_print(self):
        """Load selected print from combo."""
        name = self._file_combo.currentText()
        if not name or not self._file_manager:
            return
        self._load_print_file(name)

    def _on_file_selected(self, name: str):
        """Handle file combo selection change (preview only, not auto-load)."""
        pass  # Load is explicit via button

    def _on_name_changed(self):
        """Handle print name edit."""
        new_name = self._name_edit.text().strip()
        if new_name and new_name != self._active_file_name:
            if self._file_manager and self._active_file_name:
                # Rename = save-as + delete old
                try:
                    self._file_manager.save_as(new_name)
                    old = self._active_file_name
                    self._active_file_name = new_name
                    self._file_manager.delete(old)
                    self._refresh_file_combo()
                    self._emit_prints_changed()
                except Exception as e:
                    logger.warning(f"Rename failed: {e}")
            self._active_file_name = new_name

    def _on_duplicate_print(self):
        """Duplicate current print file."""
        if not self._active_file_name or not self._file_manager:
            return
        new_name, ok = QInputDialog.getText(
            self, "Duplicate Print", "New name:",
            text=f"{self._active_file_name}_copy")
        if ok and new_name.strip():
            try:
                self._file_manager.duplicate(new_name.strip())
                self._refresh_file_combo()
                self._emit_prints_changed()
                self._save_status.setText(f"✅ Duplicated as '{new_name.strip()}'")
            except Exception as e:
                QMessageBox.warning(self, "Duplicate Failed", str(e))

    def _on_delete_print(self):
        """Delete current print file."""
        if not self._active_file_name or not self._file_manager:
            return
        reply = QMessageBox.question(
            self, "Delete Print",
            f"Delete '{self._active_file_name}'? This cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            self._file_manager.delete(self._active_file_name)
            self._active_file_name = None
            self._objects.clear()
            self._name_edit.clear()
            self._refresh_objects_list()
            self._refresh_preview_all()
            self._update_summary()
            self._refresh_file_combo()
            self._emit_prints_changed()
            self._save_status.setText("● Deleted")

    def _on_export_print(self):
        """Export current print as standalone JSON."""
        if not self._objects:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Print", f"{self._active_file_name or 'print'}.json",
            "JSON Files (*.json)")
        if path:
            import json
            data = self._serialize_current_state()
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._save_status.setText(f"✅ Exported to {Path(path).name}")

    def _load_print_file(self, name: str):
        """Load a print file by name."""
        if not self._file_manager:
            return
        try:
            self._file_manager.load(name)
            self._active_file_name = name
            self._current_file = self._file_manager.current

            # Rebuild objects list from file data
            self._objects.clear()
            if self._current_file and hasattr(self._current_file, 'objects'):
                for obj_name, obj_data in self._current_file.objects.items():
                    entry = {
                        "name": obj_name,
                        "object_type": obj_data.get("object_type", "point"),
                        "params": obj_data.get("params", {}),
                        "position": tuple(obj_data.get("position", [0, 0, 0])),
                        "color": obj_data.get("color", DEFAULT_COLORS[0]),
                        "ink": obj_data.get("ink", obj_data.get("ink_pump", "")),
                        "auto_layout": obj_data.get("auto_layout", False),
                        "in_well": obj_data.get("in_well", True),
                    }
                    self._objects.append(entry)

            self._name_edit.setText(name)
            self._editing_index = None
            self._cancel_edit_mode()
            self._refresh_objects_list()
            self._refresh_preview_all()
            self._update_summary()
            self._save_status.setText(f"✅ Loaded '{name}'")
            self._save_status.setStyleSheet(f"color: {COLORS['green']};")
            self.file_changed.emit(name)

        except Exception as e:
            QMessageBox.warning(self, "Load Failed", str(e))
            logger.error(f"Failed to load print '{name}': {e}", exc_info=True)

    def _refresh_file_combo(self):
        """Update file combo with available prints."""
        self._file_combo.blockSignals(True)
        self._file_combo.clear()
        if self._file_manager:
            for name in _normalize_file_list(self._file_manager.list_files()):
                self._file_combo.addItem(name)
            if self._active_file_name:
                idx = self._file_combo.findText(self._active_file_name)
                if idx >= 0:
                    self._file_combo.setCurrentIndex(idx)
        self._file_combo.blockSignals(False)

    def _next_print_name(self) -> str:
        """Generate next available print name."""
        if self._file_manager and hasattr(self._file_manager, 'get_next_name'):
            try:
                result = self._file_manager.get_next_name()
                return str(result) if result else "Print_001"
            except Exception:
                pass
        # Fallback: count existing files
        existing = self.get_print_file_names()
        n = len(existing) + 1
        return f"Print_{n:03d}"

    def _restore_last_print(self):
        """Restore last active print from settings."""
        self._refresh_file_combo()
        if self.settings:
            last = self.settings.get("print_objects.last_active_print", None)
            if last and self._file_manager:
                try:
                    self._load_print_file(last)
                    return
                except Exception:
                    pass

    # ══════════════════════════════════════════════════════════════
    #  OBJECT DESIGNER OPERATIONS (S4B.3, S4B.4, S4B.6)
    # ══════════════════════════════════════════════════════════════

    def _on_type_list_changed(self, current, previous):
        """Handle type list selection changed."""
        if current is None:
            return
        obj_type = current.data(Qt.ItemDataRole.UserRole)
        if obj_type is None:
            # Header item clicked — re-select previous or skip
            if previous is not None:
                self._type_list.blockSignals(True)
                self._type_list.setCurrentItem(previous)
                self._type_list.blockSignals(False)
            return
        self._current_type = obj_type
        idx = self._param_stack_indices.get(obj_type, 0)
        self._param_stack.setCurrentIndex(idx)
        self._sync_filled_checkbox(obj_type)
        self._schedule_preview()

    def _resize_param_stack(self, index: int):
        """Resize the stacked widget to fit only the current page."""
        for i in range(self._param_stack.count()):
            w = self._param_stack.widget(i)
            if w is not None:
                if i == index:
                    w.setSizePolicy(QSizePolicy.Policy.Preferred,
                                    QSizePolicy.Policy.Preferred)
                else:
                    w.setSizePolicy(QSizePolicy.Policy.Preferred,
                                    QSizePolicy.Policy.Ignored)
        self._param_stack.adjustSize()

    # Types where the filled checkbox row is hidden
    _NO_FILL_TYPES = {"point", "line", "spiral", "csv_import"}

    # Object types that get the v7.5.1 outer-ring outline pass when
    # filled — currently only 2D fillable shapes.
    _OUTER_RING_TYPES = {"circle", "square", "triangle", "ellipse"}

    def _sync_filled_checkbox(self, obj_type: str):
        """Show/hide the filled checkbox + fill-pattern combo +
        outer-ring spinbox based on object type."""
        if not hasattr(self, '_filled_check'):
            return
        hide = obj_type in self._NO_FILL_TYPES
        self._filled_check.setVisible(not hide)
        is_filled = self._filled_check.isChecked()
        # Show fill-pattern combo only when filled is checked and visible
        if hasattr(self, '_fill_pattern_combo'):
            show_pattern = not hide and is_filled
            self._fill_pattern_combo.setVisible(show_pattern)
            self._fill_pattern_label.setVisible(show_pattern)
        # v7.5.1: outer-ring volume shows only for 2D filled shapes.
        if hasattr(self, '_outer_ring_uL'):
            show_outer = (
                not hide
                and is_filled
                and obj_type in self._OUTER_RING_TYPES
            )
            self._outer_ring_uL.setVisible(show_outer)
            self._outer_ring_label.setVisible(show_outer)

    @staticmethod
    def _categorize_object_types() -> list[tuple[str, list]]:
        """Group OBJECT_TYPES into (category_name, [(key, (label, icon))]) sections."""
        cats_1d = []
        cats_2d = []
        cats_3d = []
        cats_other = []

        for obj_type, (label, icon_text) in OBJECT_TYPES.items():
            entry = (obj_type, (label, icon_text))
            cat = _TYPE_CATEGORY.get(obj_type, "")
            if cat == "1D":
                cats_1d.append(entry)
            elif cat == "2D":
                cats_2d.append(entry)
            elif cat == "3D":
                cats_3d.append(entry)
            else:
                cats_other.append(entry)

        result = []
        # v7.5.6: Import (CSV custom objects) first, per user request.
        if cats_other:
            result.append(("Import", cats_other))
        if cats_1d:
            result.append(("1D Objects", cats_1d))
        if cats_2d:
            result.append(("2D Objects", cats_2d))
        if cats_3d:
            result.append(("3D Objects", cats_3d))
        return result

    def _get_current_params(self) -> dict:
        """Collect parameters from current type's widgets.

        QSpinBox values are returned as int, QDoubleSpinBox as float.
        Additional safety: any param in INT_PARAMS is cast to int.
        """
        widgets = self._param_widgets.get(self._current_type, {})
        params = {}
        for pname, widget in widgets.items():
            if isinstance(widget, QSpinBox):
                params[pname] = widget.value()  # already int
            elif isinstance(widget, QDoubleSpinBox):
                val = widget.value()
                # Cast to int if this is a count/index parameter
                if pname in INT_PARAMS:
                    params[pname] = int(val)
                else:
                    params[pname] = val
            elif isinstance(widget, QLineEdit):
                params[pname] = widget.text()
        # Add filled flag and fill pattern from common widgets
        if hasattr(self, '_filled_check'):
            params["filled"] = self._filled_check.isChecked()
        if hasattr(self, '_fill_pattern_combo') and self._filled_check.isChecked():
            params["fill_pattern"] = self._fill_pattern_combo.currentData() or "meander"
        # v7.5.1: outer-ring outline volume for 2D filled shapes.
        if (hasattr(self, '_outer_ring_uL')
                and self._outer_ring_uL.isVisible()):
            params["outer_ring_volume_uL"] = float(self._outer_ring_uL.value())
        return params

    def _set_params_from_dict(self, obj_type: str, params: dict):
        """Set parameter widgets from a dict."""
        widgets = self._param_widgets.get(obj_type, {})
        for pname, val in params.items():
            w = widgets.get(pname)
            if w is None:
                continue
            if isinstance(w, QSpinBox):
                w.blockSignals(True)
                w.setValue(int(val))
                w.blockSignals(False)
            elif isinstance(w, QDoubleSpinBox):
                w.blockSignals(True)
                w.setValue(float(val))
                w.blockSignals(False)
            elif isinstance(w, QLineEdit):
                w.setText(str(val))
        # Restore filled checkbox + fill pattern state
        if hasattr(self, '_filled_check'):
            self._filled_check.blockSignals(True)
            self._filled_check.setChecked(params.get("filled", False))
            self._filled_check.blockSignals(False)
        if hasattr(self, '_fill_pattern_combo'):
            pat = params.get("fill_pattern", "meander")
            idx = self._fill_pattern_combo.findData(pat)
            if idx >= 0:
                self._fill_pattern_combo.blockSignals(True)
                self._fill_pattern_combo.setCurrentIndex(idx)
                self._fill_pattern_combo.blockSignals(False)
        # v7.5.1: outer-ring outline volume
        if hasattr(self, '_outer_ring_uL'):
            self._outer_ring_uL.blockSignals(True)
            self._outer_ring_uL.setValue(
                float(params.get("outer_ring_volume_uL", 0.0)))
            self._outer_ring_uL.blockSignals(False)
        self._sync_filled_checkbox(obj_type)

    def _on_filled_toggled(self, _state):
        """Handle filled checkbox toggled — show/hide fill pattern combo."""
        if hasattr(self, '_fill_pattern_combo'):
            show = self._filled_check.isChecked()
            self._fill_pattern_combo.setVisible(show)
            self._fill_pattern_label.setVisible(show)
        self._schedule_preview()

    def _schedule_preview(self, *_args):
        """Schedule a debounced preview update."""
        self._preview_timer.start()

    def _update_live_preview(self):
        """Generate trajectory for current designer state and show in preview (S4B.3)."""
        if self._current_type == "csv_import":
            self._designer_preview_obj = None
            self._designer_blink_timer.stop()
            return

        params = self._get_current_params()
        position = (self._pos_x.value(), self._pos_y.value(), self._pos_z.value())
        pump_id = self._ink_combo.currentData() or "P1"

        obj = self._build_print_object(
            name="__preview__",
            obj_type=self._current_type,
            params=params,
            position=position,
            color=self._current_color,
            pump_id=pump_id,
        )

        if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
            self._designer_preview_obj = obj
            self._designer_blink_dim = False
            self._show_single_preview(obj, ghost=True)
            if not self._designer_blink_timer.isActive():
                self._designer_blink_timer.start()
            info_parts = []
            if hasattr(obj, 'num_waypoints') and obj.num_waypoints:
                info_parts.append(f"{obj.num_waypoints} pts")
            if hasattr(obj, 'total_length_mm') and obj.total_length_mm:
                info_parts.append(f"{obj.total_length_mm:.1f} mm")
            if hasattr(obj, 'total_time_s') and obj.total_time_s:
                info_parts.append(f"{obj.total_time_s:.1f} s")
            self._obj_info.setText(" | ".join(info_parts) if info_parts else "Preview ready")
        else:
            self._obj_info.setText("⚠ No trajectory generated")

    def _on_add_or_update(self):
        """Add new object or update existing (S4B.4, S4B.6)."""
        if self._current_type == "csv_import":
            self._import_csv()
            return

        params = self._get_current_params()
        position = (self._pos_x.value(), self._pos_y.value(), self._pos_z.value())
        ink_name = self._ink_combo.currentData() or ""

        entry = {
            "object_type": self._current_type,
            "params": dict(params),
            "position": position,
            "color": self._current_color,
            "ink": ink_name,
            "num_layers": 1,
            "layer_height": 0.2,
            "auto_layout": False,
        }

        if self._editing_index is not None:
            # Update existing — preserve in_well state
            entry["name"] = self._objects[self._editing_index]["name"]
            entry["in_well"] = self._objects[self._editing_index].get("in_well", True)
            self._objects[self._editing_index] = entry
            self._cancel_edit_mode()
        else:
            # Add new — adding from designer means it goes into the well
            entry["name"] = _auto_name(self._current_type)
            entry["in_well"] = True
            self._objects.append(entry)

        # Stop designer blink — object is now committed
        self._designer_blink_timer.stop()
        self._designer_preview_obj = None

        self._refresh_objects_list()
        self._refresh_preview_all()
        self._update_summary()
        self._trigger_auto_save()
        self._emit_prints_changed()

    def _cancel_edit_mode(self):
        """Exit edit mode, return to add mode."""
        self._editing_index = None
        self._add_btn.setText("+ Add Object")
        self._add_btn.setStyleSheet(
            f"background: {COLORS['green']}; color: {COLORS['base']}; "
            f"font-weight: bold; padding: 6px 16px; border-radius: 4px;")
        self._cancel_edit_btn.setVisible(False)
        self._edit_indicator.setVisible(False)

    def _enter_edit_mode(self, index: int):
        """Enter edit mode for an object (S4B.6)."""
        if index < 0 or index >= len(self._objects):
            return
        self._editing_index = index
        entry = self._objects[index]

        # Map old shell/solid types to consolidated GUI type
        obj_type = entry["object_type"]
        gui_type = obj_type
        infer_filled = None
        if obj_type.endswith("_solid"):
            gui_type = obj_type.replace("_solid", "")
            infer_filled = True
        elif obj_type.endswith("_shell"):
            gui_type = obj_type.replace("_shell", "")
            infer_filled = False

        self._current_type = gui_type
        for i in range(self._type_list.count()):
            item = self._type_list.item(i)
            if item.data(Qt.ItemDataRole.UserRole) == gui_type:
                self._type_list.blockSignals(True)
                self._type_list.setCurrentItem(item)
                self._type_list.blockSignals(False)
                break
        self._param_stack.setCurrentIndex(self._param_stack_indices.get(gui_type, 0))

        # Set params
        self._set_params_from_dict(gui_type, entry.get("params", {}))

        # Set common fields
        pos = entry.get("position", (0, 0, 0))
        self._pos_x.setValue(pos[0])
        self._pos_y.setValue(pos[1])
        self._pos_z.setValue(pos[2] if len(pos) > 2 else 0.0)
        # Color from ink (read-only)
        self._current_color = entry.get("color", DEFAULT_COLORS[0])
        self._ink_color_swatch.setStyleSheet(
            f"background: {self._current_color}; border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px;")

        # Ink
        ink_name = entry.get("ink", entry.get("ink_pump", ""))
        idx = self._ink_combo.findData(ink_name)
        if idx >= 0:
            self._ink_combo.setCurrentIndex(idx)

        # Filled checkbox — restore from params or infer from old type
        if hasattr(self, '_filled_check'):
            filled = entry.get("params", {}).get("filled", False)
            if infer_filled is not None:
                filled = infer_filled
            self._filled_check.blockSignals(True)
            self._filled_check.setChecked(filled)
            self._filled_check.blockSignals(False)
            self._sync_filled_checkbox(gui_type)

        # UI indicators
        self._add_btn.setText("✓ Update Object")
        self._add_btn.setStyleSheet(
            f"background: {COLORS['yellow']}; color: {COLORS['base']}; "
            f"font-weight: bold; padding: 6px 16px; border-radius: 4px;")
        self._cancel_edit_btn.setVisible(True)
        self._edit_indicator.setText(f"Editing: {entry['name']}")
        self._edit_indicator.setVisible(True)

        self._schedule_preview()

    # ══════════════════════════════════════════════════════════════
    #  OBJECTS LIST OPERATIONS (S4B.5)
    # ══════════════════════════════════════════════════════════════

    def _refresh_objects_list(self):
        """Rebuild the objects list widget from self._objects."""
        self._objects_list.clear()
        for i, obj in enumerate(self._objects):
            type_info = OBJECT_TYPES.get(obj["object_type"], ("?", "?"))
            icon_text = type_info[1]
            pos = obj.get("position", (0, 0, 0))
            ink = obj.get("ink", obj.get("ink_pump", ""))
            auto = " [auto]" if obj.get("auto_layout") else ""
            well_tag = " [IN WELL]" if obj.get("in_well") else ""
            text = (f"{i+1}. {icon_text} {obj['name']} "
                    f"[{ink}] @ ({pos[0]:.1f}, {pos[1]:.1f}){auto}{well_tag}")
            item = QListWidgetItem(text)
            color = QColor(obj.get("color", DEFAULT_COLORS[0]))
            item.setForeground(color)
            self._objects_list.addItem(item)

        # v7.2.4: Apply OOB flash colors (S4.10)
        if hasattr(self, '_oob_indices'):
            self._refresh_objects_list_colors()

    def _on_object_selected(self, row: int):
        """Update button states when selection changes (no auto-preview)."""
        has_sel = 0 <= row < len(self._objects)
        if hasattr(self, '_btn_remove_from_well'):
            in_well = self._objects[row].get("in_well", False) if has_sel else False
            self._btn_remove_from_well.setEnabled(has_sel and in_well)

    def _remove_from_well(self):
        """Remove selected object from the well preview."""
        row = self._objects_list.currentRow()
        if 0 <= row < len(self._objects):
            self._objects[row]["in_well"] = False
            self._refresh_objects_list()
            self._objects_list.setCurrentRow(row)
            self._refresh_preview_all()
            self._trigger_auto_save()

    def _on_objects_reordered(self, *_args):
        """Handle drag reorder in objects list."""
        # Rebuild _objects from current list order
        new_order = []
        for i in range(self._objects_list.count()):
            text = self._objects_list.item(i).text()
            # Extract original index from "N. icon Name..."
            try:
                idx_str = text.split(".")[0].strip()
                orig_idx = int(idx_str) - 1
                if 0 <= orig_idx < len(self._objects):
                    new_order.append(self._objects[orig_idx])
            except (ValueError, IndexError):
                pass

        if len(new_order) == len(self._objects):
            self._objects = new_order
            self._refresh_objects_list()
            self._trigger_auto_save()

    def _edit_selected(self):
        """Load selected object into designer for editing."""
        row = self._objects_list.currentRow()
        if 0 <= row < len(self._objects):
            self._enter_edit_mode(row)

    def _duplicate_selected(self):
        """Duplicate selected object with small position offset."""
        row = self._objects_list.currentRow()
        if row < 0 or row >= len(self._objects):
            return
        import copy
        entry = copy.deepcopy(self._objects[row])
        entry["name"] = _auto_name(entry["object_type"])
        pos = list(entry.get("position", (0, 0, 0)))
        pos[0] += 0.5  # Offset so it's visible
        entry["position"] = tuple(pos)
        entry["auto_layout"] = False
        self._objects.insert(row + 1, entry)
        self._refresh_objects_list()
        self._refresh_preview_all()
        self._update_summary()
        self._trigger_auto_save()

    def _move_up(self):
        row = self._objects_list.currentRow()
        if row > 0:
            self._objects[row], self._objects[row - 1] = (
                self._objects[row - 1], self._objects[row])
            self._refresh_objects_list()
            self._objects_list.setCurrentRow(row - 1)
            self._trigger_auto_save()

    def _move_down(self):
        row = self._objects_list.currentRow()
        if 0 <= row < len(self._objects) - 1:
            self._objects[row], self._objects[row + 1] = (
                self._objects[row + 1], self._objects[row])
            self._refresh_objects_list()
            self._objects_list.setCurrentRow(row + 1)
            self._trigger_auto_save()

    def _remove_selected(self):
        row = self._objects_list.currentRow()
        if 0 <= row < len(self._objects):
            self._objects.pop(row)
            self._refresh_objects_list()
            self._refresh_preview_all()
            self._update_summary()
            self._trigger_auto_save()
            self._emit_prints_changed()

    # ══════════════════════════════════════════════════════════════
    #  AUTO-LAYOUT (S4B.7, S4B.8)
    # ══════════════════════════════════════════════════════════════

    def _on_layout_pattern_changed(self, index: int):
        self._layout_param_stack.setCurrentIndex(index)

    def _get_layout_params(self) -> dict:
        """Collect auto-layout parameters from current pattern."""
        pattern = self._layout_pattern.currentData()
        widgets = self._layout_param_widgets.get(pattern, {})
        params = {}
        for pname, w in widgets.items():
            if isinstance(w, QDoubleSpinBox):
                params[pname] = w.value()
            elif isinstance(w, QLineEdit):
                params[pname] = w.text()
        return params

    def _apply_auto_layout(self):
        """Generate N objects using auto-layout pattern (S4B.7)."""
        pattern = self._layout_pattern.currentData()
        params = self._get_layout_params()

        # Generate positions
        if HAS_AUTO_LAYOUT:
            try:
                positions = generate_layout(pattern, **params)
            except Exception as e:
                QMessageBox.warning(self, "Layout Error", str(e))
                return
        else:
            # Fallback: simple ring
            import math
            count = int(params.get("count", 6))
            radius = float(params.get("radius", 2.0))
            positions = [
                (radius * math.cos(2 * math.pi * i / count),
                 radius * math.sin(2 * math.pi * i / count))
                for i in range(count)
            ]

        if not positions:
            self._layout_count_label.setText("No positions generated")
            return

        # Use current designer params as template
        obj_type = self._current_type
        obj_params = self._get_current_params()
        ink_name = self._ink_combo.currentData() or ""

        for x, y in positions:
            entry = {
                "name": _auto_name(obj_type),
                "object_type": obj_type,
                "params": dict(obj_params),
                "position": (x, y, self._pos_z.value()),
                "color": self._current_color,
                "ink": ink_name,
                "num_layers": 1,
                "layer_height": 0.2,
                "auto_layout": True,
                # v7.5.1: without this, _refresh_preview_all() filters
                # auto-layout entries out (it requires in_well=True)
                # and the user sees no objects appear after Apply.
                "in_well": True,
            }
            self._objects.append(entry)

        self._layout_count_label.setText(f"{len(positions)} objects added")
        self._refresh_objects_list()
        self._refresh_preview_all()
        self._update_summary()
        self._trigger_auto_save()
        self._emit_prints_changed()

    def _clear_auto_layout(self):
        """Remove all objects tagged as auto_layout (S4B.7)."""
        before = len(self._objects)
        self._objects = [o for o in self._objects if not o.get("auto_layout")]
        removed = before - len(self._objects)
        self._layout_count_label.setText(f"Removed {removed} auto-layout objects")
        self._refresh_objects_list()
        self._refresh_preview_all()
        self._update_summary()
        self._trigger_auto_save()
        self._emit_prints_changed()

    # ══════════════════════════════════════════════════════════════
    #  CSV IMPORT (S4B.10)
    # ══════════════════════════════════════════════════════════════

    def _choose_csv_file(self):
        """v7.5.6: pick a CSV file in the Object Designer. The file is
        imported as a custom object when the user clicks + Add Object."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose CSV Trajectory", "",
            "CSV Files (*.csv *.txt);;All Files (*)")
        if not path:
            return
        self._csv_chosen_path = path
        if hasattr(self, "_csv_chosen_label"):
            self._csv_chosen_label.setText(f"✓ {Path(path).name}")

    def _import_csv(self):
        """Import the chosen CSV trajectory as a custom object. If no
        file was chosen via the designer, prompt for one."""
        path = getattr(self, "_csv_chosen_path", None)
        if not path:
            path, _ = QFileDialog.getOpenFileName(
                self, "Import CSV Trajectory", "",
                "CSV Files (*.csv *.txt);;All Files (*)")
        if not path:
            return

        try:
            if HAS_NUMPY:
                data = np.genfromtxt(path, delimiter=",", skip_header=1)
            else:
                import csv
                with open(path) as f:
                    reader = csv.reader(f)
                    next(reader, None)  # skip header
                    data = [list(map(float, row)) for row in reader if row]

            name, ok = QInputDialog.getText(
                self, "Name CSV Import", "Object name:",
                text=f"CSV_{Path(path).stem}")
            if not ok or not name.strip():
                return

            entry = {
                "name": name.strip(),
                "object_type": "csv_import",
                "params": {"source_file": str(path)},
                "position": (0.0, 0.0, 0.0),
                "color": DEFAULT_COLORS[len(self._objects) % len(DEFAULT_COLORS)],
                "ink": self._ink_combo.currentData() or "",
                "auto_layout": False,
                "in_well": True,
                "_csv_data": data if HAS_NUMPY else None,
            }
            self._objects.append(entry)
            self._refresh_objects_list()
            self._refresh_preview_all()
            self._update_summary()
            self._trigger_auto_save()
            self._emit_prints_changed()
            # Reset the chosen-file state for the next import.
            self._csv_chosen_path = None
            if hasattr(self, "_csv_chosen_label"):
                self._csv_chosen_label.setText("No file chosen")

        except Exception as e:
            QMessageBox.warning(self, "CSV Import Failed", str(e))
            if hasattr(self, "_csv_chosen_label"):
                self._csv_chosen_label.setText(f"⚠ Import failed: {e}")

    # ══════════════════════════════════════════════════════════════
    #  PREVIEW AND TRAJECTORY
    # ══════════════════════════════════════════════════════════════

    def _extract_needle_syringe(self):
        """Get needle and syringe info from workspace or hardware config."""
        needle = None
        syringe_map = {}
        speed = 5.0
        # v7.2.7: fallback speed from parent
        # Try to get speed from parent PrintSetup page's GUI
        try:
            _parent = self.parent()
            while _parent is not None:
                if hasattr(_parent, "xy_feed_spin"):
                    speed = _parent.xy_feed_spin.value()
                    break
                _parent = _parent.parent() if hasattr(_parent, "parent") else None
        except Exception:
            pass
        layer_h = 0.2

        if self._workspace:
            needle = getattr(self._workspace, 'needle', None)
            for pid, pump in getattr(self._workspace, 'pumps', {}).items():
                if hasattr(pump, 'syringe') and pump.syringe is not None:
                    syringe_map[pid] = pump.syringe
            ps = getattr(self._workspace, 'print_settings', {})
            speed = ps.get('print_speed_mm_s', 5.0) if isinstance(ps, dict) else 5.0
            layer_h = ps.get('layer_height_mm', 0.2) if isinstance(ps, dict) else 0.2

        # Fall back to HardwareConfig
        if self._hw_config is not None:
            if needle is None and hasattr(self._hw_config, 'needle'):
                needle = self._hw_config.needle
            if not syringe_map and hasattr(self._hw_config, 'pumps'):
                for pid, pump_cfg in self._hw_config.pumps.items():
                    if hasattr(pump_cfg, 'syringe') and pump_cfg.syringe is not None:
                        syringe_map[pid] = pump_cfg.syringe

        # Last resort fallback
        if needle is None:
            try:
                needle = NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152)
            except Exception:
                pass

        return needle, syringe_map, speed, layer_h

    def _build_print_object(self, name, obj_type, params, position=(0, 0, 0),
                            color="#a6e3a1", pump_id="P1"):
        """Create a PrintObject and generate its trajectory."""
        if not HAS_GEOMETRY:
            return None

        # Resolve consolidated 3D GUI type → engine type via filled flag
        filled = params.get("filled", False)
        effective_type = _3D_TYPE_MAP.get((obj_type, filled), obj_type)

        is_csv = (obj_type == "csv_import")
        obj = PrintObject(
            name=name,
            object_type=effective_type,
            params=dict(params),
            position=position,
            ink_assignments={pump_id: "ink"},
            color=color,
            source=("csv" if is_csv else "parametric"),
        )

        # CSV import: use pre-loaded trajectory directly (skip GeometryEngine)
        if is_csv:
            csv_data = params.get("_csv_data")
            csv_src = params.get("source_file") or params.get("csv_path")
            if csv_data is None and csv_src:
                try:
                    from SupportClasses.TrajectoryPlanner import import_csv_trajectory
                    csv_data = import_csv_trajectory(csv_src)
                except Exception as e:
                    logger.error(f"Failed to load CSV trajectory for '{name}': {e}")
            if csv_data is not None and HAS_NUMPY:
                arr = np.asarray(csv_data, dtype=np.float64)
                if arr.ndim == 2 and arr.shape[1] >= 7:
                    obj.trajectory = arr[:, :7]
                    obj.total_time_s = float(arr[-1, 6] - arr[0, 6]) if len(arr) > 0 else 0.0
                    dx = np.diff(arr[:, 0])
                    dy = np.diff(arr[:, 1])
                    dz = np.diff(arr[:, 2])
                    obj.total_length_mm = float(np.sum(np.sqrt(dx**2 + dy**2 + dz**2)))
                    obj.num_layers = 1
            return obj

        needle, syringe_map, speed, layer_h = self._extract_needle_syringe()
        if needle is None:
            return obj

        try:
            fill_pattern = params.get("fill_pattern", "meander")
            generate_object_trajectory(
                obj, needle, syringe_map,
                print_speed_mm_s=speed,
                layer_height_mm=layer_h,
                fill_pattern=fill_pattern,
                pump_id=pump_id,
            )
        except Exception as e:
            logger.error(f"Trajectory generation failed for '{name}': {e}")

        return obj

    @staticmethod
    def _darken_color(hex_color: str, factor: float = 0.4) -> str:
        """Return a darker version of a hex color (factor 0-1, lower=darker)."""
        c = QColor(hex_color)
        return QColor.fromHslF(
            c.hslHueF(), c.hslSaturationF(),
            max(0.0, c.lightnessF() * factor)).name()

    def _toggle_designer_blink(self):
        """Alternate the designer preview between normal and dim."""
        obj = self._designer_preview_obj
        if obj is None or not hasattr(obj, 'trajectory') or obj.trajectory is None:
            self._designer_blink_timer.stop()
            return
        self._designer_blink_dim = not self._designer_blink_dim
        self._show_single_preview(obj, ghost=True, dim=self._designer_blink_dim)

    def _show_single_preview(self, obj, ghost=False, dim=False):
        """Show a single object trajectory in the preview."""
        if not HAS_PROJECTION_CANVAS or not isinstance(self._preview, ProjectionCanvas):
            return
        if not HAS_NUMPY or not hasattr(obj, 'trajectory') or obj.trajectory is None:
            return

        traj = obj.trajectory
        pts = [(float(traj[i, 0]), float(traj[i, 1]), float(traj[i, 2]))
               for i in range(len(traj))]
        color = getattr(obj, 'color', "#a6e3a1")
        if dim:
            color = self._darken_color(color)
        obj_path = ObjectPath(name=obj.name, color=color, points=pts)
        self._preview.set_object_paths([obj_path])
        self._update_well_diameter()
        self._preview.refresh()

    def _refresh_preview_all(self, highlight_index=None):
        """Regenerate preview showing all objects as draggable items."""
        # Run OOB check first so we can color paths red
        self._refresh_oob_state()

        self._update_well_diameter()

        # Use the draggable PlacedObject system so users can drag objects
        if HAS_PROJECTION_CANVAS and hasattr(self._preview, 'clear_placed_objects'):
            self._preview.clear_placed_objects()
            if hasattr(self._preview, 'clear_object_paths'):
                self._preview.clear_object_paths()

            for i, entry in enumerate(self._objects):
                # Only show objects explicitly added to well
                if not entry.get("in_well", False):
                    continue
                # Build trajectory at ORIGIN so PlacedObject offsets control position
                # For csv_import, pass cached _csv_data through params
                build_params = entry.get("params", {})
                if entry["object_type"] == "csv_import" and "_csv_data" in entry:
                    build_params = dict(build_params)
                    build_params["_csv_data"] = entry["_csv_data"]
                obj = self._build_print_object(
                    name=entry["name"],
                    obj_type=entry["object_type"],
                    params=build_params,
                    position=(0, 0, 0),
                    color=entry.get("color", DEFAULT_COLORS[0]),
                    pump_id=self._resolve_ink_to_pump(entry.get("ink", entry.get("ink_pump", ""))),
                )
                if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
                    traj = obj.trajectory
                    pts = [(float(traj[j, 0]), float(traj[j, 1]), float(traj[j, 2]))
                           for j in range(len(traj))]

                    # OOB = red, highlighted = white, else original
                    if i in self._oob_indices:
                        color = "#f38ba8"
                    elif highlight_index is not None and i == highlight_index:
                        color = "#ffffff"
                    else:
                        color = entry.get("color", DEFAULT_COLORS[0])

                    pos = entry.get("position", (0, 0, 0))
                    # For point objects, compute sphere radius from volume
                    sphere_r = 0.0
                    if entry.get("object_type") == "point":
                        vol_uL = entry.get("params", {}).get("dispense_volume_uL", 0.1)
                        vol_mm3 = vol_uL  # 1 µL = 1 mm³
                        sphere_r = (3 * vol_mm3 / (4 * 3.14159265)) ** (1/3)

                    placed = PlacedObject(
                        name=entry["name"],
                        color=color,
                        points=pts,
                        x_offset=float(pos[0]),
                        y_offset=float(pos[1]),
                        z_offset=float(pos[2]) if len(pos) > 2 else 0.0,
                        library_key=entry["name"],
                        sphere_radius_mm=sphere_r,
                    )
                    self._preview._add_placed_object(placed)

            if hasattr(self._preview, 'refresh'):
                self._preview.refresh()
            return

        # Fallback: static paths for non-interactive preview
        all_paths = []
        for i, entry in enumerate(self._objects):
            if not entry.get("in_well", False):
                continue
            build_params = entry.get("params", {})
            if entry["object_type"] == "csv_import" and "_csv_data" in entry:
                build_params = dict(build_params)
                build_params["_csv_data"] = entry["_csv_data"]
            obj = self._build_print_object(
                name=entry["name"],
                obj_type=entry["object_type"],
                params=build_params,
                position=entry.get("position", (0, 0, 0)),
                color=entry.get("color", DEFAULT_COLORS[0]),
                pump_id=self._resolve_ink_to_pump(entry.get("ink", entry.get("ink_pump", ""))),
            )
            if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
                traj = obj.trajectory
                pts = [(float(traj[j, 0]), float(traj[j, 1]), float(traj[j, 2]))
                       for j in range(len(traj))]
                color = entry.get("color", DEFAULT_COLORS[0])
                all_paths.append(ObjectPath(
                    name=entry["name"], color=color, points=pts))

        if all_paths:
            self._preview.set_object_paths(all_paths)
        else:
            if hasattr(self._preview, 'clear_object_paths'):
                self._preview.clear_object_paths()

        if hasattr(self._preview, 'refresh'):
            self._preview.refresh()


    def _resolve_ink_to_pump(self, ink_name: str | None) -> str:
        """Look up which pump can handle this ink. Falls back to P1."""
        if ink_name and hasattr(self, '_hw_config') and self._hw_config:
            for pid, pcfg in self._hw_config.pumps.items():
                if pcfg.enabled and pcfg.can_handle_ink(ink_name):
                    return pid
        return "P1"

    def _on_ink_combo_changed(self, index: int):
        """Update color swatch when ink selection changes."""
        ink_name = self._ink_combo.currentData()
        if ink_name and hasattr(self, '_hw_config') and self._hw_config:
            ink = self._hw_config.ink_library.get(ink_name)
            if ink:
                self._current_color = getattr(ink, 'color', DEFAULT_COLORS[0])
                self._ink_color_swatch.setStyleSheet(
                    f"background: {self._current_color}; "
                    f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")
                self._schedule_preview()
                return
        # Fallback
        self._current_color = DEFAULT_COLORS[0]
        self._ink_color_swatch.setStyleSheet(
            f"background: {self._current_color}; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px;")

    def _refresh_ink_options_from_config(self):
        """Update ink combo from HardwareConfig ink library (all defined inks)."""
        if not hasattr(self, '_hw_config') or not self._hw_config:
            return
        if not hasattr(self, '_ink_combo'):
            return
        current = self._ink_combo.currentData()
        self._ink_combo.blockSignals(True)
        self._ink_combo.clear()
        # Show all inks from the library — pump resolution happens at print time
        for ink_name, ink_spec in self._hw_config.ink_library.items():
            self._ink_combo.addItem(ink_name, ink_name)
        if current:
            idx = self._ink_combo.findData(current)
            if idx >= 0:
                self._ink_combo.setCurrentIndex(idx)
        self._ink_combo.blockSignals(False)
        # Sync color swatch to current selection
        self._on_ink_combo_changed(self._ink_combo.currentIndex())
        self._update_well_diameter()
        logger.debug(f"PrintObjects: refreshed ink options from config")

    def _update_well_diameter(self):
        """Set well boundary + depth on preview from workspace/HW config."""
        diam = self._get_well_diameter_mm()
        if hasattr(self._preview, 'set_well_diameter'):
            self._preview.set_well_diameter(diam)
        # v7.6.0: well depth drives the XZ/YZ well-height outline.
        depth = self._get_well_depth_mm()
        if hasattr(self._preview, 'set_well_depth'):
            self._preview.set_well_depth(depth)

    # ── Out-of-Bounds Detection (v7.2.4 S4.9-S4.11) ─────────────

    def _check_bounds(self, entry: dict) -> bool:
        """Check if a print object is within the well boundary.

        Returns True if in-bounds, False if out-of-bounds.
        """
        well_diam = self._get_well_diameter_mm()
        well_radius = well_diam / 2.0
        import math

        # Get trajectory points
        obj = self._build_print_object(
            name=entry.get("name", "check"),
            obj_type=entry.get("object_type", "point"),
            params=entry.get("params", {}),
            position=entry.get("position", (0, 0, 0)),
            color=entry.get("color", "#ffffff"),
            pump_id=self._resolve_ink_to_pump(entry.get("ink", entry.get("ink_pump", ""))),
        )
        if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
            traj = obj.trajectory
            for i in range(len(traj)):
                px, py = float(traj[i, 0]), float(traj[i, 1])
                if math.sqrt(px * px + py * py) > well_radius:
                    return False
        return True

    def _refresh_oob_state(self):
        """Recheck all objects for OOB and start/stop flash timer."""
        self._stop_all_flashing()  # v7.2.6
        old_oob = set(self._oob_indices)
        new_oob = set()
        for i, entry in enumerate(self._objects):
            if not self._check_bounds(entry):
                new_oob.add(i)
        self._oob_indices = new_oob

        if new_oob:
            if not self._oob_flash_timer.isActive():
                self._oob_flash_timer.start()
        else:
            self._oob_flash_timer.stop()
            self._oob_flash_state = False

        # Update list colors if changed
        if old_oob != new_oob:
            self._refresh_objects_list_colors()

    def _on_oob_detected(self, indices: list):
        """Handle OOB signal from WellPreviewWidget."""
        self._oob_indices = set(indices)
        if indices:
            if not self._oob_flash_timer.isActive():
                self._oob_flash_timer.start()
        self._refresh_objects_list_colors()

    def _toggle_oob_flash(self):
        """Toggle flash state for OOB items (called by QTimer every 500ms)."""
        self._oob_flash_state = not self._oob_flash_state
        self._refresh_objects_list_colors()

    def _refresh_objects_list_colors(self):
        """Update list item colors — flash red for OOB items."""
        for i in range(self._objects_list.count()):
            item = self._objects_list.item(i)
            if item is None:
                continue
            if i in self._oob_indices:
                if self._oob_flash_state:
                    item.setBackground(QColor("#f38ba8"))  # Red flash
                    item.setForeground(QColor("#1e1e2e"))  # Dark text
                else:
                    item.setBackground(QColor("#45475a"))  # Surface2
                    if i < len(self._objects):
                        color = QColor(self._objects[i].get("color", "#cdd6f4"))
                        item.setForeground(color)
                # Prepend warning icon to text if not already there
                text = item.text()
                if not text.startswith("⚠"):
                    item.setText(f"⚠ {text}")
            else:
                item.setBackground(QColor("transparent"))
                if i < len(self._objects):
                    color = QColor(self._objects[i].get("color", "#cdd6f4"))
                    item.setForeground(color)
                # Remove warning icon if present
                text = item.text()
                if text.startswith("⚠ "):
                    item.setText(text[2:])

    def _get_well_diameter_mm(self) -> float:
        """Get well diameter from hardware config or workspace."""
        if hasattr(self, '_hw_config') and self._hw_config:
            try:
                fmt = self._hw_config.plate_format
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                plate_def = PLATE_DEFINITIONS.get(fmt, {})
                return plate_def.get("well_diameter", 6.0)
            except (ImportError, AttributeError):
                pass
        if hasattr(self, '_workspace') and self._workspace:
            try:
                fmt = self._workspace.plate_format
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                plate_def = PLATE_DEFINITIONS.get(fmt, {})
                return plate_def.get("well_diameter", 6.0)
            except (ImportError, AttributeError):
                pass
        return 6.0  # Default 96-well plate

    def _get_well_depth_mm(self) -> float:
        """v7.6.0: well depth from HW config or workspace plate format."""
        for src_attr in ("_hw_config", "_workspace"):
            src = getattr(self, src_attr, None)
            if not src:
                continue
            try:
                fmt = src.plate_format
                from SupportClasses.WellPlate import PLATE_DEFINITIONS
                plate_def = PLATE_DEFINITIONS.get(fmt, {})
                depth = plate_def.get("well_depth_mm", 0.0)
                if depth:
                    return float(depth)
            except (ImportError, AttributeError):
                pass
        return 10.67  # Default 96-well depth

    def _resolve_library_object(self, library_key, x_mm, y_mm):
        """Resolve a library key to a PlacedObject (for drag-drop compat)."""
        for entry in self._objects:
            if entry["name"] == library_key:
                obj = self._build_print_object(
                    name=library_key,
                    obj_type=entry["object_type"],
                    params=entry.get("params", {}),
                    color=entry.get("color", DEFAULT_COLORS[0]),
                    pump_id=self._resolve_ink_to_pump(entry.get("ink", entry.get("ink_pump", ""))),
                )
                pts = []
                if obj and hasattr(obj, 'trajectory') and obj.trajectory is not None:
                    traj = obj.trajectory
                    pts = [(float(traj[i, 0]), float(traj[i, 1]), float(traj[i, 2]))
                           for i in range(len(traj))]
                if not pts:
                    pts = [(0.0, 0.0, 0.0)]
                if HAS_PROJECTION_CANVAS:
                    return PlacedObject(
                        name=library_key, color=entry.get("color", DEFAULT_COLORS[0]),
                        points=pts, x_offset=x_mm, y_offset=y_mm, z_offset=0.0,
                        library_key=library_key,
                    )
        return None

    def _on_object_repositioned(self, name: str, x_mm: float, y_mm: float):
        """Handle object repositioned in preview (S4B.9)."""
        for entry in self._objects:
            if entry["name"] == name:
                pos = list(entry.get("position", (0, 0, 0)))
                pos[0] = x_mm
                pos[1] = y_mm
                entry["position"] = tuple(pos)
                self._refresh_objects_list()
                self._trigger_auto_save()
                break

    # ══════════════════════════════════════════════════════════════
    #  SIMULATION
    # ══════════════════════════════════════════════════════════════

    def _toggle_simulation(self):
        if self._sim_playing:
            self._sim_timer.stop()
            self._sim_playing = False
        else:
            self._sim_index = 0
            self._sim_playing = True
            self._sim_timer.start()

    def _sim_step(self):
        self._sim_index += 1
        # Simulation step — advance visual marker in preview
        # (simplified; full implementation depends on ProjectionCanvas API)

    # ══════════════════════════════════════════════════════════════
    #  AUTO-SAVE (S4A.7, 3.4.9)
    # ══════════════════════════════════════════════════════════════

    def _trigger_auto_save(self):
        """Schedule auto-save after 500ms debounce."""
        self._save_status.setText("⏳ Saving...")
        self._save_status.setStyleSheet(f"color: {COLORS['yellow']};")
        self._auto_save_timer.start()

    def _do_auto_save(self):
        """Persist current state to print file."""
        if not self._file_manager or not self._active_file_name:
            self._save_status.setText("● Not saved (no active file)")
            self._save_status.setStyleSheet(f"color: {COLORS['overlay0']};")
            return

        try:
            data = self._serialize_current_state()
            # Update file manager's current file
            pf = (self._file_manager.current
                  if self._file_manager
                  else self._current_file)
            if pf and hasattr(pf, 'objects'):
                pf.objects = data.get("objects", {})
                pf.collections = data.get("collections", {})
                pf.layout_presets = data.get("layout_presets", {})
                self._file_manager.save()
            else:
                # Save raw
                import json
                path = Path("config/prints") / f"{self._active_file_name}.json"
                with open(path, "w") as f:
                    json.dump(data, f, indent=2)

            self._save_status.setText("✅ Saved")
            self._save_status.setStyleSheet(f"color: {COLORS['green']};")

            # Remember last active
            if self.settings:
                self.settings.set("print_objects.last_active_print",
                                  self._active_file_name)

        except Exception as e:
            self._save_status.setText(f"⚠ Save failed: {e}")
            self._save_status.setStyleSheet(f"color: {COLORS['red']};")
            logger.error(f"Auto-save failed: {e}", exc_info=True)

    def _serialize_current_state(self) -> dict:
        """Serialize current print state to JSON-compatible dict."""
        from datetime import datetime, timezone
        objects = {}
        for obj in self._objects:
            obj_data = {
                "object_type": obj["object_type"],
                "params": obj.get("params", {}),
                "position": list(obj.get("position", (0, 0, 0))),
                "color": obj.get("color", DEFAULT_COLORS[0]),
                "ink": obj.get("ink", ""),
                "auto_layout": obj.get("auto_layout", False),
                "in_well": obj.get("in_well", True),
            }
            objects[obj["name"]] = obj_data

        return {
            "schema_version": "7.2.3",
            "metadata": {
                "name": self._active_file_name or "Untitled",
                "description": "",
                "modified": datetime.now(timezone.utc).isoformat(),
            },
            "objects": objects,
            "collections": {},
            "layout_presets": {},
        }

    # ══════════════════════════════════════════════════════════════
    #  SUMMARY (S4B.11)
    # ══════════════════════════════════════════════════════════════

    def _update_summary(self):
        """Update summary label with current print statistics."""
        n = len(self._objects)
        if n == 0:
            self._summary_label.setText("No objects")
            return

        inks = set(o.get("ink", o.get("ink_pump", "")) for o in self._objects)
        inks.discard("")
        auto_count = sum(1 for o in self._objects if o.get("auto_layout"))

        parts = [f"{n} objects"]
        if inks:
            parts.append(f"Inks: {', '.join(sorted(inks))}")
        if auto_count:
            parts.append(f"{auto_count} auto-layout")

        self._summary_label.setText(" | ".join(parts))

    # ══════════════════════════════════════════════════════════════
    #  SIGNALS & INTEGRATION (S4B.12, S4A.9)
    # ══════════════════════════════════════════════════════════════



    def _staging_add_current(self):
        """v7.2.6: Add currently loaded print to staging list."""
        name = getattr(self, '_active_file_name', None)
        if not name: return
        if name not in self._staging_print_names:
            self._staging_print_names.append(name)
            self._prints_staging_list.addItem(name)

    def _staging_remove_selected(self):
        """v7.2.6: Remove selected from staging."""
        for item in self._prints_staging_list.selectedItems():
            name = item.text()
            if name in self._staging_print_names: self._staging_print_names.remove(name)
            self._prints_staging_list.takeItem(self._prints_staging_list.row(item))

    def _send_staged_prints(self):
        """v7.2.6: Send only staged prints to well setup."""
        if not self._staging_print_names:
            return
        self.prints_changed.emit(list(self._staging_print_names))
        n = len(self._staging_print_names)
        if hasattr(self, '_btn_send_to_prints'):
            self._btn_send_to_prints.setText(f"\u2713 {n} sent!")
            QTimer.singleShot(1500, lambda: self._btn_send_to_prints.setText("\U0001f4cb Send"))

    def _stop_all_flashing(self):
        """v7.2.6: Stop OOB flash timers."""
        for t in self._flash_timers.values():
            if hasattr(t, 'stop'): t.stop()
        self._flash_timers.clear()

    def _start_flash_item(self, index: int):
        """v7.2.6: Flash objects list item red for out-of-bounds."""
        if index in self._flash_timers: return
        if not hasattr(self, '_objects_list'): return
        item = self._objects_list.item(index)
        if not item: return
        _state = [True]
        def _toggle():
            it = self._objects_list.item(index) if index < self._objects_list.count() else None
            if it is None:
                if index in self._flash_timers: self._flash_timers[index].stop(); del self._flash_timers[index]
                return
            it.setForeground(QColor(COLORS['red']) if _state[0] else QColor(COLORS['text']))
            _state[0] = not _state[0]
        timer = QTimer(self); timer.timeout.connect(_toggle); timer.start(500)
        self._flash_timers[index] = timer
        item.setForeground(QColor(COLORS['red']))

    def _send_to_available_prints(self):
        """v7.2.5: Explicitly send current prints list to well setup."""
        self._emit_prints_changed()
        # Provide visual feedback
        if hasattr(self, '_btn_send_to_prints'):
            original_text = self._btn_send_to_prints.text()
            self._btn_send_to_prints.setText("✓ Prints list sent!")
            self._btn_send_to_prints.setStyleSheet(f"""
                QPushButton {{
                    background: {COLORS.get('green', '#a6e3a1')};
                    color: {COLORS.get('base', '#1e1e2e')};
                    font-weight: bold; padding: 6px 12px;
                    border-radius: 4px; font-size: 11px;
                }}
            """)
            QTimer.singleShot(1500, lambda: self._restore_send_button(original_text))

    def _restore_send_button(self, text: str):
        """Restore send button to default style after feedback."""
        if hasattr(self, '_btn_send_to_prints'):
            self._btn_send_to_prints.setText(text)
            self._btn_send_to_prints.setStyleSheet(f"""
                QPushButton {{
                    background: {COLORS.get('mauve', '#cba6f7')};
                    color: {COLORS.get('base', '#1e1e2e')};
                    font-weight: bold; padding: 6px 12px;
                    border-radius: 4px; font-size: 11px;
                }}
                QPushButton:hover {{
                    background: {COLORS.get('pink', '#f5c2e7')};
                }}
            """)

    def _emit_prints_changed(self):
        """Emit prints_changed with list of print file names."""
        names = self.get_print_file_names()
        self.prints_changed.emit(names)
        # Legacy compatibility
        self.collections_changed.emit(names)

    def _build_legacy_collections(self) -> dict:
        """Build legacy PrintCollection dict for backward compatibility."""
        if not HAS_GEOMETRY or not self._objects:
            return {}

        coll = PrintCollection(name=self._active_file_name or "Default")
        for entry in self._objects:
            obj = PrintObject(
                name=entry["name"],
                object_type=entry["object_type"],
                params=entry.get("params", {}),
                position=entry.get("position", (0, 0, 0)),
                ink_assignments={self._resolve_ink_to_pump(entry.get("ink", entry.get("ink_pump", ""))): entry.get("ink", "ink")},
                color=entry.get("color", DEFAULT_COLORS[0]),
                source=("csv" if entry["object_type"] == "csv_import" else "parametric"),
            )
            coll.add_object(obj)

        return {coll.name: coll}

    # ══════════════════════════════════════════════════════════════
    #  INK / PUMP COMBO MANAGEMENT
    # ══════════════════════════════════════════════════════════════

    def _refresh_ink_pump_combos(self):
        """Update ink combo from hardware config."""
        self._refresh_ink_options_from_config()

    # ══════════════════════════════════════════════════════════════
    #  COLOR PICKER
    # ══════════════════════════════════════════════════════════════

    # ══════════════════════════════════════════════════════════════
    #  CONTEXT WIDGET (for sidebar panel)
    # ══════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        """Build context panel: file browser + presets + color legend."""
        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(4, 4, 4, 4)

        # File browser
        file_group = QGroupBox("Print Files")
        file_layout = QVBoxLayout(file_group)
        self._ctx_file_list = QListWidget()
        self._ctx_file_list.setMaximumHeight(_sc(150))
        self._ctx_file_list.itemDoubleClicked.connect(
            lambda item: self._load_print_file(item.text()))
        file_layout.addWidget(self._ctx_file_list)
        self._refresh_ctx_file_list()
        layout.addWidget(file_group)

        # Color legend
        legend_group = QGroupBox("Ink → Pump Legend")
        legend_layout = QVBoxLayout(legend_group)
        self._legend_label = QLabel("No inks configured")
        self._legend_label.setStyleSheet(f"color: {COLORS['subtext0']};")
        self._legend_label.setWordWrap(True)
        legend_layout.addWidget(self._legend_label)
        layout.addWidget(legend_group)
        self._refresh_legend()

        layout.addStretch()
        return ctx

    def _refresh_ctx_file_list(self):
        """Update context file browser."""
        if not hasattr(self, '_ctx_file_list'):
            return
        self._ctx_file_list.clear()
        if self._file_manager:
            for name in _normalize_file_list(self._file_manager.list_files()):
                self._ctx_file_list.addItem(name)

    def _refresh_legend(self):
        """Update ink-pump color legend."""
        if not hasattr(self, '_legend_label'):
            return
        if not self._objects:
            self._legend_label.setText("No objects")
            return
        legend_parts = []
        seen = set()
        for obj in self._objects:
            ink = obj.get("ink", obj.get("ink_pump", ""))
            color = obj.get("color", "#a6e3a1")
            if ink and ink not in seen:
                seen.add(ink)
                legend_parts.append(f'<span style="color:{color}">■</span> {ink}')
        self._legend_label.setText("<br>".join(legend_parts) if legend_parts else "No inks")