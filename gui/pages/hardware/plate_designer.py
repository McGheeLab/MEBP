"""
plate_designer.py — v7.4.5 inline well-plate designer widget.

Composes the canvas, picker header, toolbar column, properties panel,
and DOF status bar into the surface that lives in the Hardware Setup →
Plate sub-page.

Architecture:

    PlateDesignerWidget
    ├── header (combo + Save / Save As / Delete + Fit + Snap toggle)
    ├── splitter
    │   ├── toolbar column   (Select / Single Well / Grid / + Constraint / Lock / Delete / Fit)
    │   ├── PlateDesignerCanvas (mm-coordinate QGraphicsView)
    │   └── properties panel (re-built per selection)
    └── DOF status bar  ("DOF: 0  •  Hover (12.0, 8.0)")

The widget owns the `PlateDesign` document (one at a time) and routes
the canvas's selection/solve signals to the properties panel.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, Signal, QSize, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QToolButton, QLabel,
    QComboBox, QPushButton, QCheckBox, QSpinBox, QDoubleSpinBox,
    QLineEdit, QMessageBox, QInputDialog, QFrame, QSizePolicy,
    QScrollArea, QGroupBox, QFormLayout, QButtonGroup,
)

from gui.scaling import s, sp
from gui.widgets.icons import icon
from gui.widgets.plate_designer_canvas import PlateDesignerCanvas, Tool
from SupportClasses.PlateDesign import (
    PlateDesign, Well, Point, Group, Line, Constraint, EntityId,
)
from functools import partial
from SupportClasses.PlateSketchSolver import SolveReport, DOFStatus
from SupportClasses.WellPlate import (
    PLATE_DEFINITIONS, USER_PLATES_DIR,
)

logger = logging.getLogger(__name__)


# Standard formats listed first in the picker; "● " prefix marks bundled,
# "○ " prefix marks user-saved.
_BUNDLED_FORMATS: tuple[int, ...] = (6, 12, 24, 48, 96, 384)


def _letter_suffix(i: int) -> str:
    """0→a, 1→b, … 25→z, 26→aa, … for rosette sub-well naming (v7.4.8)."""
    s = ""
    i += 1
    while i > 0:
        i, rem = divmod(i - 1, 26)
        s = chr(ord("a") + rem) + s
    return s


def _letter_name_subwells(rosette: "PlateDesign") -> None:
    """Rename a rosette's sub-wells a, b, c… in id order, so flattened
    names read A1.a, A1.b, … (v7.4.8). Center wells sort first by id."""
    for idx, w in enumerate(rosette.get_wells()):
        w.name = _letter_suffix(idx)
        w.naming_scheme = "MANUAL"


class PlateDesignerWidget(QWidget):
    """The full Plate sub-page designer surface."""

    # Emitted whenever the current plate identifier changes (built-in
    # int or saved-design name). The hardware-setup page bridges this
    # into HardwareConfig.plate_name / plate_format.
    plate_changed = Signal(object)        # int | str
    # v7.4.8: rosette-mode signals consumed by the Hardware Setup page.
    save_requested = Signal()             # rosette page → save the plate
    design_edited = Signal()              # any edit (mark plate dirty)

    def __init__(self, parent: QWidget | None = None, mode: str = "plate"):
        super().__init__(parent)
        # v7.4.8: "plate" = full plate designer (Plate sub-page);
        # "rosette" = rosette designer (Rosette sub-page) — starts from the
        # shared plate layout, double-click a well to drill into its
        # rosette. Plate-management chrome (picker/save/new) is hidden in
        # rosette mode; saving is delegated to the plate page.
        self._mode = mode
        self._current_key: int | str = 96
        self._design: Optional[PlateDesign] = None
        self._dirty = False               # unsaved edits flag
        # v7.4.7 rosette nesting: (parent_design, well_id) while editing a
        # rosette in-place; None when editing the top-level plate.
        self._edit_context: Optional[tuple] = None

        self._build_ui()
        # Start with a default standard plate so the user sees something.
        self.load_plate(96)

    # ─────────────────────────────────────────────────────────────
    # v7.4.8 — design sharing (Plate page ↔ Rosette page)
    # ─────────────────────────────────────────────────────────────

    def current_design(self):
        """The active top-level plate design (parent when drilled in)."""
        if self._edit_context is not None:
            return self._edit_context[0]
        return self._design

    def adopt_design(self, design, key: int | str = None) -> None:
        """Adopt an externally-owned PlateDesign (shared object) and show it.

        Used by the Rosette sub-page to mirror the Plate page's current
        plate. Resets any in-progress rosette drill-in.
        """
        if design is None:
            return
        self._edit_context = None
        self._design = design
        if key is not None:
            self._current_key = key
        self._canvas.set_design(design)
        self._update_breadcrumb()
        self._refresh_picker_selection()
        self._rebuild_properties_panel()
        self._update_dirty_label()

    def refresh(self) -> None:
        """Re-render the canvas from the current design (e.g. after the
        rosette page added rosettes to the shared plate)."""
        if self._design is not None:
            self._canvas.set_design(self._design)
            self._rebuild_properties_panel()

    # ─────────────────────────────────────────────────────────────
    # Public API used by hardware_setup.py
    # ─────────────────────────────────────────────────────────────

    def current_plate_key(self) -> int | str:
        """The active plate identifier (int for standards, str for custom)."""
        return self._current_key

    def current_plate_name(self) -> str:
        """The custom plate name, or '' for built-in standards."""
        return self._current_key if isinstance(self._current_key, str) else ""

    def current_plate_format(self) -> int:
        """The legacy plate_format int (for the standard combo) — 24 for custom."""
        return self._current_key if isinstance(self._current_key, int) else 24

    def load_plate(self, key: int | str) -> None:
        """Load a plate by identifier and mount it in the canvas."""
        if isinstance(key, str) and key.isdigit():
            key = int(key)
        if isinstance(key, int):
            try:
                design = PlateDesign.from_standard_format(key)
            except ValueError:
                logger.warning(
                    f"Unknown standard plate format {key} — falling back to 96")
                design = PlateDesign.from_standard_format(96)
                key = 96
        else:
            try:
                design = PlateDesign.load(key)
            except FileNotFoundError:
                logger.warning(
                    f"Custom plate '{key}' not found — falling back to 96-well")
                design = PlateDesign.from_standard_format(96)
                key = 96

        self.load_design(design, key=key, dirty=False)

    def load_design(
        self, design: "PlateDesign", key: int | str = "", dirty: bool = True,
    ) -> None:
        """Mount an in-memory design (no file lookup). v7.4.7.

        Used by `load_plate` and by the "New" button (blank design with
        an empty `key` sentinel until the user runs Save As).
        """
        self._edit_context = None        # leaving any rosette edit context
        self._current_key = key
        self._design = design
        self._dirty = dirty
        self._canvas.set_design(design)
        self._update_breadcrumb()
        self._refresh_picker_selection()
        self._set_buttons_for_current_kind()
        self._rebuild_properties_panel()
        self._update_dirty_label()

    # ─────────────────────────────────────────────────────────────
    # UI construction
    # ─────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(4), s(4), s(4), s(4))
        outer.setSpacing(s(6))

        # ── Header row ────────────────────────────────────────────
        header = QHBoxLayout()
        header.setSpacing(s(6))

        rosette_mode = (self._mode == "rosette")

        if rosette_mode:
            # Rosette page: no plate-management chrome. The plate layout is
            # adopted from the Plate sub-page; here you double-click a well
            # to design its rosette.
            hint = QLabel("Rosette designer — double-click a well to add "
                          "sub-wells inside it")
            hint.setStyleSheet("color: #94e2d5; font-weight: 600;")
            header.addWidget(hint)
        else:
            header.addWidget(QLabel("Plate:"))
        self._picker = QComboBox()
        self._picker.setMinimumWidth(s(220))
        self._picker.currentIndexChanged.connect(self._on_picker_changed)
        self._picker.setVisible(not rosette_mode)
        header.addWidget(self._picker)

        self._dirty_lbl = QLabel("")
        self._dirty_lbl.setStyleSheet("color: #fab387; font-weight: 600;")
        header.addWidget(self._dirty_lbl)

        # v7.4.7: start a fresh blank plate.
        self._btn_new = QPushButton("New")
        self._btn_new.setToolTip("Start a fresh blank plate (ANSI footprint, no wells)")
        self._btn_new.clicked.connect(self._on_new)
        header.addWidget(self._btn_new)

        self._btn_save = QPushButton("Save")
        # In rosette mode, Save persists the shared plate via the Plate page.
        self._btn_save.clicked.connect(
            self.save_requested.emit if rosette_mode else self._on_save)
        header.addWidget(self._btn_save)

        self._btn_save_as = QPushButton("Save As…")
        self._btn_save_as.clicked.connect(self._on_save_as)
        header.addWidget(self._btn_save_as)

        self._btn_delete = QPushButton("Delete")
        self._btn_delete.setObjectName("dangerBtn")
        self._btn_delete.clicked.connect(self._on_delete)
        header.addWidget(self._btn_delete)

        # Hide plate-file management on the rosette page.
        self._btn_new.setVisible(not rosette_mode)
        self._btn_save_as.setVisible(not rosette_mode)
        self._btn_delete.setVisible(not rosette_mode)
        if rosette_mode:
            self._btn_save.setText("Save plate")
            self._btn_save.setToolTip("Save the plate (including rosettes)")

        header.addStretch(1)

        self._btn_refresh = QPushButton("⟳")
        self._btn_refresh.setToolTip("Refresh plate list")
        self._btn_refresh.setFixedWidth(s(28))
        self._btn_refresh.clicked.connect(self._refresh_picker)
        self._btn_refresh.setVisible(not rosette_mode)
        header.addWidget(self._btn_refresh)

        self._btn_fit = QPushButton("Fit")
        self._btn_fit.setToolTip("Fit plate to view (F)")
        self._btn_fit.setFixedWidth(s(40))
        self._btn_fit.clicked.connect(self._on_fit_view)
        header.addWidget(self._btn_fit)

        self._chk_snap = QCheckBox("Snap")
        self._chk_snap.setToolTip("Snap drag to 1 mm grid")
        self._chk_snap.toggled.connect(self._on_snap_toggled)
        header.addWidget(self._chk_snap)

        # v7.4.7: wheel-zoom toggle.
        self._chk_wheel = QCheckBox("Wheel zoom")
        self._chk_wheel.setToolTip(
            "Allow the scroll wheel to zoom the canvas in/out")
        self._chk_wheel.setChecked(True)
        self._chk_wheel.toggled.connect(self._on_wheel_zoom_toggled)
        header.addWidget(self._chk_wheel)

        # v7.4.7: edge-dimension reference (center vs well edge).
        header.addWidget(QLabel("Dim ref:"))
        self._dim_ref_combo = QComboBox()
        self._dim_ref_combo.addItems(["Center", "Edge"])
        self._dim_ref_combo.setToolTip(
            "Reference for new edge dimensions: the well center or its near edge")
        self._dim_ref_combo.currentTextChanged.connect(
            self._on_dim_ref_changed)
        header.addWidget(self._dim_ref_combo)

        outer.addLayout(header)

        # ── Breadcrumb bar (shown only while editing a rosette) ────
        self._breadcrumb = QFrame()
        self._breadcrumb.setStyleSheet(
            "background: #313244; border-radius: 4px;")
        bc_lay = QHBoxLayout(self._breadcrumb)
        bc_lay.setContentsMargins(s(8), s(4), s(8), s(4))
        self._btn_back_to_plate = QPushButton("◀ Back to plate")
        self._btn_back_to_plate.setObjectName("accentBtn")
        self._btn_back_to_plate.clicked.connect(self._on_back_to_plate)
        bc_lay.addWidget(self._btn_back_to_plate)
        self._breadcrumb_lbl = QLabel("")
        self._breadcrumb_lbl.setStyleSheet("color: #cdd6f4; font-weight: 600;")
        bc_lay.addWidget(self._breadcrumb_lbl)
        bc_lay.addStretch(1)
        bc_lay.addWidget(QLabel("Preset:"))
        self._rosette_preset_combo = QComboBox()
        self._rosette_preset_combo.addItems([
            "(keep)", "Blank", "Ring of 6", "Ring of 6 + center",
            "Ring of 8 + center", "Ring of 12 + center",
        ])
        self._rosette_preset_combo.currentTextChanged.connect(
            self._on_rosette_preset_changed)
        bc_lay.addWidget(self._rosette_preset_combo)
        # v7.4.8: save the current rosette as a reusable standard insert.
        self._btn_save_insert = QPushButton("Save as standard insert…")
        self._btn_save_insert.clicked.connect(self._on_save_standard_insert)
        bc_lay.addWidget(self._btn_save_insert)
        self._breadcrumb.setVisible(False)
        outer.addWidget(self._breadcrumb)

        # ── Splitter: toolbar | canvas | properties ───────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        self._toolbar = self._build_toolbar()
        splitter.addWidget(self._toolbar)

        self._canvas = PlateDesignerCanvas(self)
        self._canvas.selection_changed.connect(self._on_selection_changed)
        self._canvas.design_changed.connect(self._on_design_changed)
        self._canvas.solve_report.connect(self._on_solve_report)
        self._canvas.hover_pos_changed.connect(self._on_hover_pos)
        self._canvas.tool_changed.connect(self._on_tool_changed)
        # v7.4.8: double-click a well → zoom in + design its rosette.
        # Only on the Rosette sub-page; the Plate sub-page is layout-only.
        if self._mode == "rosette":
            self._canvas.well_drill_requested.connect(self._on_edit_rosette)
        splitter.addWidget(self._canvas)

        self._props_scroll = QScrollArea()
        self._props_scroll.setWidgetResizable(True)
        self._props_scroll.setFrameShape(QFrame.NoFrame)
        self._props_scroll.setMinimumWidth(s(260))
        self._props_scroll.setMaximumWidth(s(360))
        self._props_widget = QWidget()
        self._props_layout = QVBoxLayout(self._props_widget)
        self._props_layout.setContentsMargins(s(6), s(6), s(6), s(6))
        self._props_layout.setSpacing(s(8))
        self._props_scroll.setWidget(self._props_widget)
        splitter.addWidget(self._props_scroll)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        splitter.setSizes([s(64), s(700), s(280)])
        outer.addWidget(splitter, 1)

        # ── DOF status bar ────────────────────────────────────────
        status_row = QHBoxLayout()
        status_row.setSpacing(s(10))
        self._status_dof = QLabel("DOF: —")
        self._status_dof.setFont(QFont("Segoe UI", 10, QFont.DemiBold))
        status_row.addWidget(self._status_dof)
        status_row.addStretch(1)
        self._status_hover = QLabel("Hover: —")
        status_row.addWidget(self._status_hover)
        outer.addLayout(status_row)

        self._refresh_picker()

        # Keyboard shortcuts.
        #
        # The single-letter tool bindings (S/W/G/C/L/Shift+L/D/K) deliberately
        # do NOT live here any more. As window-scoped QShortcuts they fired for
        # the entire Hardware Setup window, so typing "s" or "g" into the plate
        # name — or any other text field on the page — switched tools and ate
        # the keystroke. They are now handled by PlateDesignerCanvas.keyPressEvent,
        # which cannot reach a sibling editor. The canvas syncs the toolbar back
        # through its tool_changed signal, so the buttons still track the keys.
        #
        # What remains is chorded and unambiguous, but still scoped to this
        # widget subtree rather than the window.
        for seq, slot in (
            ("Ctrl+Z", self._on_undo),
            ("Ctrl+Shift+Z", self._on_redo),
            ("Ctrl+Y", self._on_redo),
        ):
            sc = QShortcut(QKeySequence(seq), self, activated=slot)
            sc.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)

    # Toolbar button geometry — all buttons share the same shape so the
    # column reads as a coherent palette.
    TOOL_BTN_SIZE = 44      # pixels (pre-DPI scale); square buttons
    TOOL_ICON_SIZE = 22

    def _build_toolbar(self) -> QWidget:
        """Vertical tool palette with icon-only square buttons."""
        frame = QFrame()
        frame.setObjectName("plateDesignerToolbar")
        frame.setFrameShape(QFrame.NoFrame)
        frame.setFixedWidth(s(self.TOOL_BTN_SIZE + 12))
        col = QVBoxLayout(frame)
        col.setContentsMargins(s(6), s(8), s(6), s(8))
        col.setSpacing(s(4))

        # Tool-group buttons (exclusive: only one tool active at a time).
        self._tool_group = QButtonGroup(frame)
        self._tool_group.setExclusive(True)

        self._btn_select = self._make_tool_btn(
            "cursor", "Select  (S)\nPick & drag wells", Tool.SELECT)
        self._tool_group.addButton(self._btn_select)
        col.addWidget(self._btn_select)

        self._btn_well = self._make_tool_btn(
            "circle-plus", "Single Well  (W)\nClick to place one well",
            Tool.DRAW_SINGLE_WELL)
        self._tool_group.addButton(self._btn_well)
        col.addWidget(self._btn_well)

        self._btn_grid = self._make_tool_btn(
            "grid", "Grid Pattern  (G)\nDrop a rectangular grid",
            Tool.DRAW_GRID)
        self._tool_group.addButton(self._btn_grid)
        col.addWidget(self._btn_grid)

        self._btn_circle = self._make_tool_btn(
            "compass", "Circle Pattern  (C)\nDrop N wells on a ring",
            Tool.DRAW_CIRCLE_PATTERN)
        self._tool_group.addButton(self._btn_circle)
        col.addWidget(self._btn_circle)

        self._btn_line = self._make_tool_btn(
            "line", "Line  (L)\nTwo-click reference line",
            Tool.DRAW_LINE)
        self._tool_group.addButton(self._btn_line)
        col.addWidget(self._btn_line)

        self._btn_construction = self._make_tool_btn(
            "line",
            "Construction Line  (Shift+L)\nDashed reference, excluded from compile",
            Tool.DRAW_CONSTRUCTION_LINE)
        # Visually distinguish construction from solid line — semi-mauve tint.
        self._btn_construction.setStyleSheet(
            self._tool_button_qss(checkable=True)
            + "QToolButton { color:#94e2d5; }")
        self._tool_group.addButton(self._btn_construction)
        col.addWidget(self._btn_construction)

        # v7.4.7: edge-distance dimension tool.
        self._btn_dimension = self._make_tool_btn(
            "ruler",
            "Dimension  (D)\nClick a well to add editable distance "
            "dimensions to the left + top plate edges",
            Tool.DIMENSION)
        self._tool_group.addButton(self._btn_dimension)
        col.addWidget(self._btn_dimension)

        col.addSpacing(s(10))
        col.addWidget(self._make_separator())
        col.addSpacing(s(6))

        # Action buttons (non-exclusive — momentary).
        self._btn_lock = self._make_action_btn(
            "lock", "Lock  (K)\nPin selected wells at their position",
            self._on_lock_clicked)
        col.addWidget(self._btn_lock)

        self._btn_del = self._make_action_btn(
            "trash", "Delete selected  (Del)", self._on_delete_clicked,
            danger=True)
        col.addWidget(self._btn_del)

        col.addSpacing(s(10))
        col.addWidget(self._make_separator())
        col.addSpacing(s(6))

        self._btn_undo = self._make_action_btn(
            "undo", "Undo  (Ctrl+Z)", self._on_undo)
        col.addWidget(self._btn_undo)
        self._btn_redo = self._make_action_btn(
            "redo", "Redo  (Ctrl+Shift+Z)", self._on_redo)
        col.addWidget(self._btn_redo)

        col.addStretch(1)

        # Pre-select the SELECT tool.
        self._btn_select.setChecked(True)
        # Selection-dependent buttons start disabled.
        self._btn_lock.setEnabled(False)
        self._btn_del.setEnabled(False)

        return frame

    def _make_tool_btn(
        self, icon_name: str, tooltip: str, tool: Tool,
    ) -> QToolButton:
        """A square checkable tool button with a centered icon."""
        b = QToolButton()
        b.setIcon(icon(icon_name))
        b.setIconSize(QSize(s(self.TOOL_ICON_SIZE), s(self.TOOL_ICON_SIZE)))
        b.setToolTip(tooltip)
        b.setCheckable(True)
        b.setAutoExclusive(False)        # exclusivity managed by QButtonGroup
        b.setFixedSize(s(self.TOOL_BTN_SIZE), s(self.TOOL_BTN_SIZE))
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.setStyleSheet(self._tool_button_qss(checkable=True))
        b.clicked.connect(lambda checked, t=tool: self._canvas.set_tool(t))
        return b

    def _make_action_btn(
        self, icon_name: str, tooltip: str, handler,
        danger: bool = False,
    ) -> QToolButton:
        """A square momentary action button with a centered icon."""
        b = QToolButton()
        b.setIcon(icon(icon_name))
        b.setIconSize(QSize(s(self.TOOL_ICON_SIZE), s(self.TOOL_ICON_SIZE)))
        b.setToolTip(tooltip)
        b.setFixedSize(s(self.TOOL_BTN_SIZE), s(self.TOOL_BTN_SIZE))
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.setStyleSheet(self._tool_button_qss(checkable=False, danger=danger))
        b.clicked.connect(handler)
        return b

    @staticmethod
    def _tool_button_qss(checkable: bool, danger: bool = False) -> str:
        """Uniform QToolButton stylesheet for the designer's tool column."""
        bg = "#313244"
        bg_hover = "#45475a"
        bg_active = "#585b70"
        border = "#45475a"
        accent = "#cba6f7"          # mauve — Catppuccin
        danger_bg = "#f38ba8"
        danger_hover = "#eba0ac"
        if danger:
            return (
                f"QToolButton {{ background:{bg}; color:#cdd6f4; "
                f"border:1px solid {border}; border-radius:{sp(6)}; }}"
                f"QToolButton:hover {{ background:{danger_hover}; "
                f"color:#1e1e2e; border-color:{danger_bg}; }}"
                f"QToolButton:pressed {{ background:{danger_bg}; "
                f"color:#1e1e2e; }}"
                f"QToolButton:disabled {{ background:#1e1e2e; "
                f"color:#585b70; border-color:#313244; }}"
            )
        checked = (
            f"QToolButton:checked {{ background:{accent}; color:#1e1e2e; "
            f"border-color:{accent}; }}"
        ) if checkable else ""
        return (
            f"QToolButton {{ background:{bg}; color:#cdd6f4; "
            f"border:1px solid {border}; border-radius:{sp(6)}; }}"
            f"QToolButton:hover {{ background:{bg_hover}; "
            f"border-color:{accent}; }}"
            f"QToolButton:pressed {{ background:{bg_active}; }}"
            f"QToolButton:disabled {{ background:#1e1e2e; "
            f"color:#585b70; border-color:#313244; }}"
            f"{checked}"
        )

    @staticmethod
    def _make_separator() -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color: #45475a; background: #45475a;")
        line.setFixedHeight(s(1))
        return line

    # ─────────────────────────────────────────────────────────────
    # Picker
    # ─────────────────────────────────────────────────────────────

    def _refresh_picker(self) -> None:
        """Rebuild the picker combo from bundled standards + user files."""
        blocker = self._picker.blockSignals(True)
        self._picker.clear()
        for fmt in _BUNDLED_FORMATS:
            self._picker.addItem(f"● {fmt}-well", fmt)
        USER_PLATES_DIR.mkdir(parents=True, exist_ok=True)
        for path in sorted(USER_PLATES_DIR.glob("*.json")):
            name = path.stem
            self._picker.addItem(f"○ {name}", name)
        self._picker.blockSignals(blocker)
        self._refresh_picker_selection()

    def _refresh_picker_selection(self) -> None:
        idx = self._picker.findData(self._current_key)
        if idx >= 0:
            self._picker.blockSignals(True)
            self._picker.setCurrentIndex(idx)
            self._picker.blockSignals(False)

    def _on_picker_changed(self, _idx: int) -> None:
        key = self._picker.currentData()
        if key is None or key == self._current_key:
            return
        if self._dirty:
            answer = QMessageBox.question(
                self, "Unsaved changes",
                "Discard unsaved changes to the current plate?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No)
            if answer != QMessageBox.Yes:
                self._refresh_picker_selection()  # revert
                return
        self.load_plate(key)
        self.plate_changed.emit(self._current_key)

    def _set_buttons_for_current_kind(self) -> None:
        # Delete only valid for a saved custom plate (non-empty str key).
        is_user = isinstance(self._current_key, str) and bool(self._current_key)
        self._btn_delete.setEnabled(is_user)

    def _on_new(self) -> None:
        """Start a fresh blank plate (v7.4.7)."""
        if self._dirty:
            answer = QMessageBox.question(
                self, "Unsaved changes",
                "Discard unsaved changes and start a new blank plate?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        # Empty-string key = unsaved sentinel; Save As assigns a name.
        self.load_design(PlateDesign.blank(), key="", dirty=True)
        self.plate_changed.emit(self._current_key)

    # ─────────────────────────────────────────────────────────────
    # Rosette nested editing (v7.4.7)
    # ─────────────────────────────────────────────────────────────

    def _update_breadcrumb(self) -> None:
        """Show/hide the rosette breadcrumb bar based on edit context."""
        editing = self._edit_context is not None
        self._breadcrumb.setVisible(editing)
        if editing:
            parent_design, well_id = self._edit_context
            well = parent_design.entities.get(well_id)
            wname = getattr(well, "name", "?")
            self._breadcrumb_lbl.setText(
                f"Editing rosette for well {wname}")
            # Reset preset combo to "(keep)" without firing the handler.
            self._rosette_preset_combo.blockSignals(True)
            self._rosette_preset_combo.setCurrentText("(keep)")
            self._rosette_preset_combo.blockSignals(False)
        # The Dimension tool is meaningless on a circular bore — hide it.
        if hasattr(self, "_btn_dimension"):
            self._btn_dimension.setVisible(not editing)

    def _on_edit_rosette(self, well_id) -> None:
        """Enter in-place rosette editing for the given well."""
        if self._design is None:
            return
        well = self._design.entities.get(well_id)
        if not isinstance(well, Well):
            return
        # Create a blank rosette design if the well has none yet.
        if well.rosette_design is None:
            bore_r = max(well.diameter / 2.0, 0.5)
            well.rosette_design = PlateDesign.blank_rosette(
                bore_radius_mm=bore_r, name=f"{well.name}-rosette")
            self._dirty = True
        # Stash context: (parent design, well id). Mount the nested design.
        parent_design = self._design
        self._edit_context = (parent_design, well_id)
        self._design = well.rosette_design
        self._canvas.set_design(well.rosette_design)
        self._update_breadcrumb()
        self._rebuild_properties_panel()
        self._update_dirty_label()

    def _on_back_to_plate(self) -> None:
        """Exit rosette editing, return to the parent plate."""
        if self._edit_context is None:
            return
        parent_design, well_id = self._edit_context
        # Discard an empty rosette so an accidental drill-in (double-click
        # with no sub-wells placed) leaves the well pristine.
        well = parent_design.entities.get(well_id)
        if (isinstance(well, Well) and well.rosette_design is not None
                and not well.rosette_design.get_wells()):
            well.rosette_design = None
        self._edit_context = None
        self._design = parent_design
        self._canvas.set_design(parent_design)
        self._update_breadcrumb()
        # Re-select the well we were editing.
        self._canvas.set_selection([well_id])
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()

    def _on_rosette_preset_changed(self, text: str) -> None:
        """Apply a rosette preset to the nested design being edited."""
        if self._edit_context is None or text == "(keep)":
            return
        if self._design is None:
            return
        bore_r = self._design.outline.radius or 3.2
        # Rebuild a fresh nested design from the preset.
        ros = PlateDesign.blank_rosette(
            bore_radius_mm=bore_r, name=self._design.name)
        sub_d = max(bore_r * 0.35, 0.4)
        ring_r = bore_r * 0.6
        presets = {
            "Ring of 6": (6, False),
            "Ring of 6 + center": (6, True),
            "Ring of 8 + center": (8, True),
            "Ring of 12 + center": (12, True),
        }
        if text == "Blank":
            pass  # circular outline only
        elif text in presets:
            count, center = presets[text]
            if center:
                ros.add_well(x=0.0, y=0.0, diameter=sub_d, name="x",
                             naming_scheme="MANUAL")
            ros.add_circle_pattern(
                count=count, center_x=0.0, center_y=0.0,
                radius=ring_r, diameter=sub_d, group_name="ring")
        else:
            return
        # Re-letter sub-wells a, b, c… so flattened names read A1.a, A1.b.
        _letter_name_subwells(ros)
        # Swap the nested design in place (keep edit context).
        parent_design, well_id = self._edit_context
        well = parent_design.entities.get(well_id)
        if isinstance(well, Well):
            well.rosette_design = ros
        self._design = ros
        self._canvas.set_design(ros)
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()

    def _on_save_standard_insert(self) -> None:
        """Save the rosette currently being edited as a standard insert."""
        if self._edit_context is None or self._design is None:
            return
        if not self._design.get_wells():
            QMessageBox.information(
                self, "Empty rosette",
                "Add at least one sub-well before saving an insert.")
            return
        from SupportClasses.PlateDesign import save_standard_insert
        name, ok = QInputDialog.getText(
            self, "Save Standard Insert", "Insert name:")
        if not ok or not name.strip():
            return
        try:
            # Save a copy so later edits don't mutate the library file.
            import copy
            save_standard_insert(copy.deepcopy(self._design), name.strip())
        except Exception as e:
            QMessageBox.warning(self, "Save failed", str(e))
            return
        QMessageBox.information(
            self, "Saved",
            f"Standard insert '{name.strip()}' saved. You can drop it into "
            f"any well from the well's properties panel.")

    def _on_remove_rosette(self, well_id) -> None:
        """Detach the rosette design from a well."""
        if self._design is None:
            return
        well = self._design.entities.get(well_id)
        if isinstance(well, Well) and well.rosette_design is not None:
            self._canvas.push_undo_snapshot()
            well.rosette_design = None
            self._dirty = True
            self._update_dirty_label()
            self._rebuild_properties_panel()

    # ─────────────────────────────────────────────────────────────
    # Save / Save As / Delete
    # ─────────────────────────────────────────────────────────────

    def _name_rejection(self, name: str) -> str:
        """Why *name* cannot be used as a plate name, or "" if it is fine."""
        if not name:
            return "Name cannot be empty."
        if name in (str(f) for f in _BUNDLED_FORMATS) or name.isdigit():
            return f"'{name}' clashes with a standard format. Pick another name."
        return ""

    def _on_save(self) -> None:
        if self._design is None:
            return
        # Saving a bundled standard → behaves as Save As.
        if isinstance(self._current_key, int):
            self._on_save_as()
            return

        # The Plate card's Name field is a RENAME request, not decoration.
        # Until v7.12 this line read `self._design.name = self._current_key`,
        # which silently threw the typed name away on every Save.
        typed = (self._design.name or "").strip()
        if typed and typed != self._current_key:
            if not self._rename_current(typed):
                # Refused or cancelled — keep the file's name so the card and
                # the document agree again rather than showing a phantom edit.
                self._design.name = self._current_key
                self._rebuild_properties_panel()
                return
            return

        self._design.name = self._current_key
        path = self._design.save()
        self._dirty = False
        self._update_dirty_label()
        logger.info(f"Saved plate to {path}")

    def _rename_current(self, new_name: str) -> bool:
        """Save the current design under *new_name* and retire the old file.

        Returns True when the rename committed.

        The plate name is also the per-plate store key (mosaics, plate templates,
        well training, and the taught calibration archive), and none of those
        follow a rename, so this asks first and says what is at stake.
        """
        why = self._name_rejection(new_name)
        if why:
            QMessageBox.warning(self, "Rename Plate", why)
            return False

        old_key = self._current_key
        new_path = USER_PLATES_DIR / f"{new_name}.json"
        if new_path.exists():
            answer = QMessageBox.question(
                self, "Rename Plate",
                f"'{new_name}' already exists. Overwrite it?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return False

        answer = QMessageBox.question(
            self, "Rename Plate",
            f"Rename '{old_key}' to '{new_name}'?\n\n"
            f"Taught calibration, mosaics and well training are stored under the "
            f"plate name. '{new_name}' starts a fresh set — the data taught for "
            f"'{old_key}' stays with that name.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return False

        self._design.name = new_name
        self._design.save()
        old_path = USER_PLATES_DIR / f"{old_key}.json"
        if old_path.exists() and old_path != new_path:
            try:
                old_path.unlink()
            except OSError as exc:
                # The new file is already written, so the rename stands; the
                # stale copy just lingers in the picker. Say so rather than
                # failing a save the operator can see succeeded.
                logger.warning("Could not remove old plate file %s: %s",
                               old_path, exc)

        self._current_key = new_name
        self._dirty = False
        self._refresh_picker()
        self._refresh_picker_selection()
        self._set_buttons_for_current_kind()
        self._update_dirty_label()
        self.plate_changed.emit(self._current_key)
        logger.info("Renamed plate '%s' → '%s'", old_key, new_name)
        return True

    def _on_save_as(self) -> None:
        if self._design is None:
            return
        suggested = (self._current_key
                     if isinstance(self._current_key, str)
                     else f"plate-{self._current_key}")
        name, ok = QInputDialog.getText(
            self, "Save Plate As", "Plate name:", text=suggested)
        if not ok:
            return
        name = name.strip()
        why = self._name_rejection(name)
        if why:
            QMessageBox.warning(self, "Save As", why)
            return
        # Overwrite confirmation if file exists.
        path = USER_PLATES_DIR / f"{name}.json"
        if path.exists():
            answer = QMessageBox.question(
                self, "Save As",
                f"'{name}' already exists. Overwrite?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self._design.name = name
        self._design.save()
        self._current_key = name
        self._dirty = False
        self._refresh_picker()
        self._refresh_picker_selection()
        self._set_buttons_for_current_kind()
        self._update_dirty_label()
        self.plate_changed.emit(self._current_key)

    def _on_delete(self) -> None:
        if not isinstance(self._current_key, str):
            return
        answer = QMessageBox.question(
            self, "Delete Plate",
            f"Delete custom plate '{self._current_key}'? "
            f"This cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        path = USER_PLATES_DIR / f"{self._current_key}.json"
        if path.exists():
            try:
                path.unlink()
            except OSError as e:
                QMessageBox.warning(self, "Delete failed", str(e))
                return
        # Fall back to a default standard.
        self.load_plate(96)
        self._refresh_picker()
        self.plate_changed.emit(self._current_key)

    # ─────────────────────────────────────────────────────────────
    # Canvas → properties panel reactions
    # ─────────────────────────────────────────────────────────────

    def _on_selection_changed(self, ids: list) -> None:
        # Lock/Delete require at least one selected well.
        has_sel = bool(ids)
        self._btn_lock.setEnabled(has_sel)
        self._btn_del.setEnabled(has_sel)
        self._rebuild_properties_panel()

    def _on_design_changed(self) -> None:
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()
        # History buttons reflect the canvas's stack state.
        self._btn_undo.setEnabled(self._canvas.can_undo())
        self._btn_redo.setEnabled(self._canvas.can_redo())
        # v7.4.8: let the Hardware Setup page mark the plate dirty when the
        # rosette page edits the shared design.
        self.design_edited.emit()

    def _on_solve_report(self, report) -> None:
        if not isinstance(report, SolveReport):
            return
        status_text = self._format_status(report)
        self._status_dof.setText(status_text)

    def _on_hover_pos(self, x_mm: float, y_mm: float) -> None:
        self._status_hover.setText(f"Hover: ({x_mm:6.2f}, {y_mm:6.2f}) mm")

    def _on_tool_changed(self, tool: Tool) -> None:
        if tool == Tool.SELECT:
            self._btn_select.setChecked(True)
        elif tool == Tool.DRAW_SINGLE_WELL:
            self._btn_well.setChecked(True)
        elif tool == Tool.DRAW_GRID:
            self._btn_grid.setChecked(True)
        elif tool == Tool.DRAW_CIRCLE_PATTERN:
            self._btn_circle.setChecked(True)
        elif tool == Tool.DRAW_LINE:
            self._btn_line.setChecked(True)
        elif tool == Tool.DRAW_CONSTRUCTION_LINE:
            self._btn_construction.setChecked(True)
        elif tool == Tool.DIMENSION:
            self._btn_dimension.setChecked(True)
        # v7.4.8: surface Circle Pattern options when that tool is active.
        self._rebuild_properties_panel()

    @staticmethod
    def _format_status(report: SolveReport) -> str:
        if report.status == DOFStatus.WELL_DETERMINED:
            return "DOF: 0 (fully constrained) ✓"
        if report.status == DOFStatus.EMPTY:
            return "DOF: — (no free wells)"
        if report.status == DOFStatus.UNDER_DETERMINED:
            return (f"DOF: {report.dof} (under-constrained) ⚠ "
                    f"— drag freely")
        if report.status == DOFStatus.INCONSISTENT:
            conflicts = (f" conflicts: {report.conflicts}"
                         if report.conflicts else "")
            return (f"DOF: over-constrained ✗ "
                    f"residual {report.residual_norm:.3g}{conflicts}")
        return f"DOF: {report.status.value}"

    def _update_dirty_label(self) -> None:
        # v7.4.7: while editing a rosette, top-level plate actions are
        # suppressed — commit via "Back to plate" first.
        in_rosette = self._edit_context is not None
        # Whether the *top-level plate* has wells (not the nested design).
        top_design = (self._edit_context[0] if in_rosette else self._design)
        has_wells = bool(top_design and top_design.get_wells())
        is_blank = self._current_key == ""
        name = "Untitled" if is_blank else ""
        suffix = " • unsaved" if self._dirty else ""
        self._dirty_lbl.setText(f"{name}{suffix}".strip(" "))

        # v7.4.8: the Rosette sub-page's "Save plate" delegates to the
        # Plate page (which forks standards to a custom name via Save As).
        # Enable it whenever the underlying plate has wells — regardless of
        # the standard/custom key — so rosettes can actually be saved.
        if self._mode == "rosette":
            self._btn_save.setEnabled(has_wells)
            self._btn_save.setToolTip(
                "Save the plate (including rosettes)"
                if has_wells else "Add a well on the Plate page first")
            return

        # Save semantics (plate mode):
        #   - Saved custom plate (non-empty str key): enabled when dirty
        #   - Standard (int) or blank (""): use "Save As…" to write a file
        self._btn_save.setEnabled(
            (not in_rosette) and self._dirty
            and isinstance(self._current_key, str)
            and bool(self._current_key) and has_wells)
        self._btn_save_as.setEnabled((not in_rosette) and has_wells)
        self._btn_save_as.setToolTip(
            "Save this plate under a new name"
            if has_wells else "Add at least one well before saving")
        self._btn_delete.setEnabled(
            (not in_rosette)
            and isinstance(self._current_key, str)
            and bool(self._current_key))
        self._btn_new.setEnabled(not in_rosette)
        self._picker.setEnabled(not in_rosette)

    # ─────────────────────────────────────────────────────────────
    # Toolbar / shortcut actions
    # ─────────────────────────────────────────────────────────────

    def _on_lock_clicked(self) -> None:
        self._canvas.lock_selection()

    def _on_delete_clicked(self) -> None:
        self._canvas.delete_selection()

    def _on_fit_view(self) -> None:
        self._canvas.fit_view()

    def _on_snap_toggled(self, checked: bool) -> None:
        self._canvas.set_snap_to_grid(checked)

    def _on_wheel_zoom_toggled(self, checked: bool) -> None:
        self._canvas.set_wheel_zoom_enabled(checked)

    def _on_dim_ref_changed(self, text: str) -> None:
        self._canvas.set_dim_ref_mode(text.lower())

    def _on_undo(self) -> None:
        self._canvas.undo()
        self._dirty = True
        self._update_dirty_label()
        self._btn_undo.setEnabled(self._canvas.can_undo())
        self._btn_redo.setEnabled(self._canvas.can_redo())

    def _on_redo(self) -> None:
        self._canvas.redo()
        self._dirty = True
        self._update_dirty_label()
        self._btn_undo.setEnabled(self._canvas.can_undo())
        self._btn_redo.setEnabled(self._canvas.can_redo())

    # ─────────────────────────────────────────────────────────────
    # Properties panel
    # ─────────────────────────────────────────────────────────────

    def _rebuild_properties_panel(self) -> None:
        # Tear down.
        while self._props_layout.count():
            item = self._props_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        if self._design is None:
            return

        sel = self._canvas.selected_ids()
        if not sel:
            # v7.4.8: when the Circle Pattern tool is active, show its
            # options up top (count / center well / diameter). Radius is
            # set by dragging on the canvas.
            if self._canvas.get_tool() == Tool.DRAW_CIRCLE_PATTERN:
                self._props_layout.addWidget(self._build_circle_options_card())
            # v7.4.8: placed circle patterns keep their options visible in
            # the standard view (no need to click a well). Edit + Apply, or
            # drag the radius handle on the canvas.
            for grp in self._design.entities.values():
                if (isinstance(grp, Group)
                        and grp.pattern_kind == "circle"
                        and grp.members):
                    self._props_layout.addWidget(self._build_group_card(grp))
            self._props_layout.addWidget(self._build_plate_card())
            self._props_layout.addWidget(self._build_summary_card())
            self._props_layout.addStretch(1)
            return

        if len(sel) == 1:
            ent = self._design.entities.get(sel[0])
            if isinstance(ent, Well):
                self._props_layout.addWidget(self._build_well_card(ent))
                # If the well belongs to a group, surface group params too
                # so the user can edit the underlying pattern (rows/cols
                # for grids, count/radius for rings) without hunting.
                if ent.group is not None:
                    grp = self._design.entities.get(ent.group)
                    if isinstance(grp, Group):
                        self._props_layout.addWidget(
                            self._build_group_card(grp))
                self._props_layout.addWidget(self._build_constraints_card(ent))
                self._props_layout.addStretch(1)
                return
            if isinstance(ent, Line):
                self._props_layout.addWidget(self._build_line_card(ent))
                self._props_layout.addStretch(1)
                return

        # Multi-select.
        ents = [self._design.entities.get(eid) for eid in sel]
        wells = [w for w in ents if isinstance(w, Well)]
        lines = [l for l in ents if isinstance(l, Line)]

        # Mixed selection: one well + one line → point-on-line is the
        # primary action.
        if len(wells) == 1 and len(lines) == 1:
            self._props_layout.addWidget(
                self._build_well_line_card(wells[0], lines[0]))
            self._props_layout.addStretch(1)
            return

        if wells:
            self._props_layout.addWidget(self._build_multi_card(wells))
            self._props_layout.addStretch(1)

    def _build_circle_options_card(self) -> QGroupBox:
        """v7.4.8: Circle Pattern tool options (count / center well / Ø).

        Radius is set by dragging on the canvas (snaps to centre, drag
        outward) and stays editable on the placed ring via its group card.
        """
        params = self._canvas.get_circle_params()
        gb = QGroupBox("Circle Pattern options")
        form = QFormLayout(gb)
        form.setSpacing(s(6))

        form.addRow(QLabel("Click the centre, then drag out the radius."))

        count = QSpinBox()
        count.setRange(1, 256)
        count.setValue(int(params.get("count", 6)))
        count.valueChanged.connect(
            lambda v: self._canvas.set_circle_param("count", v))
        form.addRow("Wells:", count)

        center_chk = QCheckBox("Add a center well")
        center_chk.setChecked(bool(params.get("center_well", False)))
        center_chk.toggled.connect(
            lambda on: self._canvas.set_circle_param("center_well", on))
        form.addRow("", center_chk)

        diam = QDoubleSpinBox()
        diam.setRange(0.05, 200.0)
        diam.setDecimals(3)
        diam.setSingleStep(0.1)
        diam.setSuffix(" mm")
        diam.setValue(float(params.get("diameter", 6.0)))
        diam.valueChanged.connect(
            lambda v: self._canvas.set_circle_param("diameter", v))
        form.addRow("Well Ø:", diam)
        return gb

    def _build_plate_card(self) -> QGroupBox:
        gb = QGroupBox("Plate")
        form = QFormLayout(gb)
        form.setSpacing(s(6))

        name_edit = QLineEdit(self._design.name if self._design else "")
        name_edit.editingFinished.connect(
            lambda: self._set_design_name(name_edit.text()))
        form.addRow("Name:", name_edit)

        w_spin = QDoubleSpinBox()
        w_spin.setRange(10.0, 500.0)
        w_spin.setSingleStep(0.1)
        w_spin.setSuffix(" mm")
        if self._design:
            w_spin.setValue(self._design.outline.width)
        w_spin.valueChanged.connect(self._set_outline_width)
        form.addRow("Width:", w_spin)

        h_spin = QDoubleSpinBox()
        h_spin.setRange(10.0, 500.0)
        h_spin.setSingleStep(0.1)
        h_spin.setSuffix(" mm")
        if self._design:
            h_spin.setValue(self._design.outline.height)
        h_spin.valueChanged.connect(self._set_outline_height)
        form.addRow("Height:", h_spin)
        return gb

    def _build_summary_card(self) -> QGroupBox:
        gb = QGroupBox("Summary")
        lay = QVBoxLayout(gb)
        if self._design is None:
            lay.addWidget(QLabel("(no design)"))
            return gb
        wells = self._design.get_wells()
        diam_min = min((w.diameter for w in wells), default=0.0)
        diam_max = max((w.diameter for w in wells), default=0.0)
        groups = {e.name for e in self._design.entities.values()
                  if isinstance(e, Group)}
        lay.addWidget(QLabel(f"Wells: {len(wells)}"))
        if wells:
            lay.addWidget(QLabel(
                f"Ø range: {diam_min:.2f} – {diam_max:.2f} mm"))
        lay.addWidget(QLabel(f"Groups: {len(groups)}"))
        lay.addWidget(QLabel(f"Constraints: {len(self._design.constraints)}"))
        return gb

    def _build_well_card(self, well: Well) -> QGroupBox:
        gb = QGroupBox(f"Well: {well.name}")
        form = QFormLayout(gb)
        form.setSpacing(s(6))

        name_edit = QLineEdit(well.name)
        name_edit.editingFinished.connect(
            lambda: self._rename_well(well.id, name_edit.text()))
        form.addRow("Name:", name_edit)

        center = self._design.entities.get(well.center) if self._design else None
        x_spin = QDoubleSpinBox()
        x_spin.setRange(-1000.0, 1000.0)
        x_spin.setDecimals(3)
        x_spin.setSuffix(" mm")
        if isinstance(center, Point):
            x_spin.setValue(center.x)
        x_spin.editingFinished.connect(
            lambda: self._set_well_x(well.id, x_spin.value()))
        form.addRow("X:", x_spin)

        y_spin = QDoubleSpinBox()
        y_spin.setRange(-1000.0, 1000.0)
        y_spin.setDecimals(3)
        y_spin.setSuffix(" mm")
        if isinstance(center, Point):
            y_spin.setValue(center.y)
        y_spin.editingFinished.connect(
            lambda: self._set_well_y(well.id, y_spin.value()))
        form.addRow("Y:", y_spin)

        d_spin = QDoubleSpinBox()
        d_spin.setRange(0.1, 200.0)
        d_spin.setDecimals(3)
        d_spin.setSingleStep(0.1)
        d_spin.setSuffix(" mm")
        d_spin.setValue(well.diameter)
        d_spin.editingFinished.connect(
            lambda: self._set_well_diameter(well.id, d_spin.value()))
        form.addRow("Diameter:", d_spin)

        depth_spin = QDoubleSpinBox()
        depth_spin.setRange(0.1, 50.0)
        depth_spin.setDecimals(2)
        depth_spin.setSingleStep(0.1)
        depth_spin.setSuffix(" mm")
        depth_spin.setValue(well.well_depth_mm)
        depth_spin.editingFinished.connect(
            lambda: self._set_well_depth(well.id, depth_spin.value()))
        form.addRow("Depth:", depth_spin)

        # v7.4.8: insert/tube geometry — shown when editing a rosette
        # (sub-wells = inserts/tubes). rim = how far the tube top sits
        # above the plate (travel clearance); ink Z = dispense height
        # relative to plate top (blank = use global print Z).
        if self._edit_context is not None:
            # v7.5.x: well-type preset picker — stamps this sub-well's
            # geometry (diameter / depth / rim height / ink Z) from a named
            # vessel type (e.g. "0.1 mL PCR tube"). Rim height is the one
            # that drives needle travel clearance.
            from SupportClasses.WellTypeStore import get_store as _wt_get_store
            _wt_store = _wt_get_store()
            wt_combo = QComboBox()
            wt_combo.addItem("(custom)", None)
            for wt in _wt_store.all():
                wt_combo.addItem(wt.label, wt.id)
            _wt_idx = (wt_combo.findData(well.well_type_id)
                       if well.well_type_id else 0)
            wt_combo.setCurrentIndex(_wt_idx if _wt_idx >= 0 else 0)
            wt_combo.setToolTip(
                "Stamp this sub-well's geometry from a standard vessel type. "
                "Rim height drives the plate-wide needle travel clearance.")
            # Connect AFTER setCurrentIndex so seeding the selection does not
            # trigger a re-stamp.
            wt_combo.currentIndexChanged.connect(
                lambda _=0, c=wt_combo, wid=well.id:
                    self._apply_well_type(wid, c.currentData()))
            form.addRow("Well type:", wt_combo)

            wt_btn_row = QHBoxLayout()
            save_wt_btn = QPushButton("Save as well type…")
            save_wt_btn.clicked.connect(
                lambda _=False, wid=well.id:
                    self._save_current_as_well_type(wid))
            wt_btn_row.addWidget(save_wt_btn)
            del_wt_btn = QPushButton("Delete")
            _cur_wt = _wt_store.get(well.well_type_id)
            del_wt_btn.setEnabled(_cur_wt is not None and not _cur_wt.builtin)
            del_wt_btn.clicked.connect(
                lambda _=False, tid=well.well_type_id:
                    self._delete_selected_well_type(tid))
            wt_btn_row.addWidget(del_wt_btn)
            wt_btn_holder = QWidget()
            wt_btn_holder.setLayout(wt_btn_row)
            form.addRow("", wt_btn_holder)

            rim_spin = QDoubleSpinBox()
            rim_spin.setRange(0.0, 100.0)
            rim_spin.setDecimals(2)
            rim_spin.setSingleStep(0.5)
            rim_spin.setSuffix(" mm")
            rim_spin.setValue(well.rim_height_mm)
            rim_spin.setToolTip(
                "Height the insert/tube top sits ABOVE the plate surface. "
                "Needle travel moves clear the tallest insert plate-wide.")
            rim_spin.editingFinished.connect(
                lambda: self._set_well_rim(well.id, rim_spin.value()))
            form.addRow("Rim height:", rim_spin)

            ink_chk = QCheckBox("Prescribe ink Z")
            ink_chk.setChecked(well.ink_z_mm is not None)
            ink_spin = QDoubleSpinBox()
            ink_spin.setRange(-100.0, 100.0)
            ink_spin.setDecimals(2)
            ink_spin.setSingleStep(0.5)
            ink_spin.setSuffix(" mm")
            ink_spin.setEnabled(well.ink_z_mm is not None)
            ink_spin.setValue(well.ink_z_mm if well.ink_z_mm is not None else 0.0)
            ink_spin.setToolTip(
                "Ink dispense Z relative to the plate top "
                "(negative = below the plate surface, into the tube)")
            ink_chk.toggled.connect(ink_spin.setEnabled)
            ink_chk.toggled.connect(
                lambda on: self._set_well_ink_z(
                    well.id, ink_spin.value() if on else None))
            ink_spin.editingFinished.connect(
                lambda: self._set_well_ink_z(well.id, ink_spin.value()))
            form.addRow("", ink_chk)
            form.addRow("Ink Z:", ink_spin)

        # Lock toggle.
        lock_btn = QPushButton(
            "Unlock" if self._is_locked(well) else "Lock at current position")
        lock_btn.clicked.connect(self._on_lock_clicked)
        form.addRow("", lock_btn)

        # v7.4.7/v7.4.8: rosette editing lives on the Rosette sub-page
        # (mode == "rosette") and only at the top level (no nested
        # rosettes inside rosettes).
        if self._mode == "rosette" and self._edit_context is None:
            has_ros = well.rosette_design is not None
            ros_btn = QPushButton(
                "Edit rosette…" if has_ros else "Add rosette…")
            ros_btn.setToolTip(
                "Design a multi-well rosette inside this well")
            ros_btn.clicked.connect(
                lambda _=False, wid=well.id: self._on_edit_rosette(wid))
            form.addRow("", ros_btn)
            if has_ros:
                # v7.4.8: rotation aligns a dropped standard insert.
                rot_spin = QDoubleSpinBox()
                rot_spin.setRange(-360.0, 360.0)
                rot_spin.setDecimals(1)
                rot_spin.setSingleStep(5.0)
                rot_spin.setSuffix(" °")
                rot_spin.setValue(well.rosette_rotation_deg)
                rot_spin.setToolTip(
                    "Rotate the rosette/insert layout to match the "
                    "physical part")
                rot_spin.editingFinished.connect(
                    lambda: self._set_rosette_rotation(
                        well.id, rot_spin.value()))
                form.addRow("Rosette rotation:", rot_spin)

                from_std_btn = QPushButton("Replace from standard insert…")
                from_std_btn.clicked.connect(
                    lambda _=False, wid=well.id:
                        self._on_drop_standard_insert(wid))
                form.addRow("", from_std_btn)

                clear_btn = QPushButton("Remove rosette")
                clear_btn.setObjectName("dangerBtn")
                clear_btn.clicked.connect(
                    lambda _=False, wid=well.id: self._on_remove_rosette(wid))
                form.addRow("", clear_btn)
            else:
                from_std_btn = QPushButton("Add from standard insert…")
                from_std_btn.clicked.connect(
                    lambda _=False, wid=well.id:
                        self._on_drop_standard_insert(wid))
                form.addRow("", from_std_btn)
        return gb

    def _build_group_card(self, group: Group) -> QGroupBox:
        """Edit a group's pattern parameters (grid rows/cols or ring count)."""
        kind = group.pattern_kind
        gb = QGroupBox(f"Group ({kind}): {group.name}")
        form = QFormLayout(gb)
        form.setSpacing(s(6))

        # Per-kind editors backed by a snapshot dict that we hand to
        # rebuild_group on Apply.
        editors: dict[str, QDoubleSpinBox | QSpinBox] = {}
        bool_editors: dict[str, QCheckBox] = {}

        def add_int(label: str, key: str, lo: int, hi: int) -> None:
            sb = QSpinBox()
            sb.setRange(lo, hi)
            sb.setValue(int(group.params.get(key, 1)))
            editors[key] = sb
            form.addRow(label, sb)

        def add_float(label: str, key: str, lo: float, hi: float,
                      step: float = 0.1, decimals: int = 3,
                      suffix: str = " mm") -> None:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(decimals)
            sb.setSingleStep(step)
            sb.setSuffix(suffix)
            sb.setValue(float(group.params.get(key, 0.0)))
            editors[key] = sb
            form.addRow(label, sb)

        if kind == "grid":
            add_int("Rows", "rows", 1, 64)
            add_int("Cols", "cols", 1, 64)
            add_float("Spacing X", "spacing_x", 0.01, 200.0)
            add_float("Spacing Y", "spacing_y", 0.01, 200.0)
            add_float("Origin X", "origin_x", -500.0, 500.0)
            add_float("Origin Y", "origin_y", -500.0, 500.0)
            add_float("Diameter", "diameter", 0.05, 200.0)
        elif kind == "circle":
            add_int("Count", "count", 1, 256)
            add_float("Center X", "center_x", -500.0, 500.0)
            add_float("Center Y", "center_y", -500.0, 500.0)
            add_float("Radius", "radius", 0.1, 200.0)
            add_float("Diameter", "diameter", 0.05, 200.0)
            add_float("Start angle", "start_angle_deg", -360.0, 360.0,
                      step=1.0, decimals=2, suffix=" °")
            cw = QCheckBox("Center well")
            cw.setChecked(bool(group.params.get("center_well", False)))
            bool_editors["center_well"] = cw
            form.addRow("", cw)
        else:
            form.addRow(QLabel(
                f"No editor available for pattern kind '{kind}'"))
            return gb

        apply_btn = QPushButton("Apply changes")
        apply_btn.setObjectName("accentBtn")
        apply_btn.setToolTip(
            "Rebuild this group's wells with the new parameters. "
            "Constraints touching the old wells will be removed.")
        apply_btn.clicked.connect(
            partial(self._apply_group_edit, group.id, editors, bool_editors))
        form.addRow("", apply_btn)
        return gb

    def _apply_group_edit(
        self, group_id: EntityId,
        editors: dict,
        bool_editors: dict = None,
    ) -> None:
        """Pull current editor values, call rebuild_group_now, refresh panel."""
        new_params: dict = {}
        for key, widget in editors.items():
            new_params[key] = widget.value()
        for key, chk in (bool_editors or {}).items():
            new_params[key] = chk.isChecked()
        self._canvas.rebuild_group_now(group_id, new_params)
        # rebuild_group_now emits selection_changed([]); panel rebuilds.

    def _build_constraints_card(self, well: Well) -> QGroupBox:
        gb = QGroupBox("Constraints touching this well")
        lay = QVBoxLayout(gb)
        if self._design is None:
            return gb
        # List constraints whose refs include this well or its center.
        rows = []
        for c in self._design.constraints:
            if (well.id in c.refs or well.center in c.refs):
                rows.append(c)
        if not rows:
            lay.addWidget(QLabel("(none)"))
            return gb
        for c in rows:
            row = QHBoxLayout()
            label = QLabel(self._format_constraint(c))
            row.addWidget(label, 1)
            # v7.4.8: edge-distance constraints are editable here (a normal
            # panel spinbox — NOT an in-scene widget, which crashed).
            if c.kind in ("dist_left_edge", "dist_top_edge",
                          "distance_pp") and c.value is not None:
                val_spin = QDoubleSpinBox()
                val_spin.setRange(-500.0, 500.0)
                val_spin.setDecimals(2)
                val_spin.setSingleStep(0.5)
                val_spin.setSuffix(" mm")
                val_spin.setFixedWidth(s(86))
                val_spin.setValue(float(c.value))
                val_spin.editingFinished.connect(
                    lambda cid=c.id, sp=val_spin:
                        self._canvas.set_constraint_value(cid, sp.value()))
                row.addWidget(val_spin)
            del_btn = QPushButton("✕")
            del_btn.setFixedWidth(s(28))
            del_btn.setToolTip("Remove this constraint")
            del_btn.clicked.connect(
                lambda _, cid=c.id: self._remove_constraint(cid))
            row.addWidget(del_btn)
            wrap = QWidget()
            wrap.setLayout(row)
            lay.addWidget(wrap)
        return gb

    def _build_multi_card(self, wells: list[Well]) -> QGroupBox:
        gb = QGroupBox(f"Batch ({len(wells)} selected)")
        lay = QVBoxLayout(gb)

        lay.addWidget(QLabel(
            "Add a constraint between the two selected wells:"))

        if len(wells) == 2:
            for label, kind in (
                ("Distance (current)", "distance_pp"),
                ("Horizontal align", "horizontal"),
                ("Vertical align", "vertical"),
                ("Coincident", "coincident_pp"),
                ("Concentric", "concentric"),
                ("Equal Ø", "equal_radius"),
                ("Tangent (touching circles)", "tangent_cc"),
            ):
                btn = QPushButton(f"+ {label}")
                btn.clicked.connect(
                    lambda _, k=kind: self._add_constraint_to_selection(k))
                lay.addWidget(btn)

            # Custom distance.
            dist_row = QHBoxLayout()
            dist_row.addWidget(QLabel("Custom distance:"))
            dist_spin = QDoubleSpinBox()
            dist_spin.setRange(0.0, 500.0)
            dist_spin.setDecimals(3)
            dist_spin.setSuffix(" mm")
            dist_spin.setValue(9.0)
            dist_row.addWidget(dist_spin, 1)
            apply_btn = QPushButton("Apply")
            apply_btn.clicked.connect(
                lambda: self._add_constraint_to_selection(
                    "distance_pp", value=dist_spin.value()))
            dist_row.addWidget(apply_btn)
            wrap = QWidget()
            wrap.setLayout(dist_row)
            lay.addWidget(wrap)
        else:
            lay.addWidget(QLabel(
                "Select exactly two wells to add a constraint."))

        lay.addSpacing(s(6))
        lay.addWidget(QLabel("Batch operations:"))
        lock_btn = QPushButton("Lock selected (toggle)")
        lock_btn.clicked.connect(self._on_lock_clicked)
        lay.addWidget(lock_btn)

        diam_row = QHBoxLayout()
        diam_row.addWidget(QLabel("Set Ø:"))
        diam_spin = QDoubleSpinBox()
        diam_spin.setRange(0.1, 200.0)
        diam_spin.setDecimals(3)
        diam_spin.setSingleStep(0.1)
        diam_spin.setSuffix(" mm")
        diam_spin.setValue(wells[0].diameter)
        diam_row.addWidget(diam_spin, 1)
        diam_apply = QPushButton("Apply")
        diam_apply.clicked.connect(
            lambda: self._set_diameter_batch(diam_spin.value()))
        diam_row.addWidget(diam_apply)
        wrap = QWidget()
        wrap.setLayout(diam_row)
        lay.addWidget(wrap)

        del_btn = QPushButton("Delete selected")
        del_btn.setObjectName("dangerBtn")
        del_btn.clicked.connect(self._on_delete_clicked)
        lay.addWidget(del_btn)
        return gb

    def _build_line_card(self, line: Line) -> QGroupBox:
        """Properties card for a single Line."""
        gb = QGroupBox(
            f"{'Construction line' if line.construction else 'Line'} #{line.id}")
        form = QFormLayout(gb)
        form.setSpacing(s(6))

        p1 = self._design.entities.get(line.p1)
        p2 = self._design.entities.get(line.p2)
        if isinstance(p1, Point) and isinstance(p2, Point):
            import math
            length = math.hypot(p2.x - p1.x, p2.y - p1.y)
            form.addRow("Length:",
                        QLabel(f"{length:.3f} mm"))
            form.addRow("Start:",
                        QLabel(f"({p1.x:.2f}, {p1.y:.2f}) mm"))
            form.addRow("End:",
                        QLabel(f"({p2.x:.2f}, {p2.y:.2f}) mm"))

        construction_chk = QCheckBox("Construction (excluded from compile)")
        construction_chk.setChecked(line.construction)
        construction_chk.toggled.connect(
            lambda checked, lid=line.id:
                self._set_line_construction(lid, checked))
        form.addRow("", construction_chk)

        del_btn = QPushButton("Delete line")
        del_btn.setObjectName("dangerBtn")
        del_btn.clicked.connect(self._on_delete_clicked)
        form.addRow("", del_btn)
        return gb

    def _set_line_construction(self, line_id: EntityId,
                               construction: bool) -> None:
        if self._design is None:
            return
        line = self._design.entities.get(line_id)
        if not isinstance(line, Line):
            return
        self._canvas.push_undo_snapshot()
        line.construction = construction
        # Toggle endpoint `fixed` to match the new convention so behavior
        # tracks the construction/solid intent.
        for pid in (line.p1, line.p2):
            p = self._design.entities.get(pid)
            if isinstance(p, Point):
                p.fixed = construction
        self._canvas._rebuild_scene()
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()

    def _build_well_line_card(self, well: Well, line: Line) -> QGroupBox:
        """Selection = 1 well + 1 line → point-on-line constraint."""
        gb = QGroupBox(f"Well + Line: {well.name} ↔ line #{line.id}")
        lay = QVBoxLayout(gb)
        lay.addWidget(QLabel(
            "Constrain the well's center to lie on the (infinite) line "
            "through both line endpoints."))
        btn = QPushButton("+ Point on line")
        btn.setObjectName("accentBtn")
        btn.clicked.connect(self._on_apply_point_on_line)
        lay.addWidget(btn)
        return gb

    def _on_apply_point_on_line(self) -> None:
        """Apply point_on_line constraint between selected well + line."""
        sel = self._canvas.selected_ids()
        if len(sel) != 2 or self._design is None:
            return
        ents = [self._design.entities.get(eid) for eid in sel]
        well = next((e for e in ents if isinstance(e, Well)), None)
        line = next((e for e in ents if isinstance(e, Line)), None)
        if well is None or line is None:
            return
        self._canvas.add_constraint_explicit(
            kind="point_on_line",
            refs=[well.center, line.id],
        )

    @staticmethod
    def _format_constraint(c: Constraint) -> str:
        if c.kind == "ground":
            return f"Ground"
        if c.kind == "fix":
            snap = (f" @ ({c.snapshot[0]:.2f}, {c.snapshot[1]:.2f})"
                    if c.snapshot else "")
            return f"Lock{snap}"
        if c.kind == "distance_pp" and c.value is not None:
            return f"Distance = {c.value:.3f} mm"
        if c.kind == "horizontal":
            return "Horizontal align"
        if c.kind == "vertical":
            return "Vertical align"
        if c.kind == "coincident_pp":
            return "Coincident"
        if c.kind == "concentric":
            return "Concentric"
        if c.kind == "equal_radius":
            return "Equal Ø"
        if c.kind == "dist_left_edge" and c.value is not None:
            ref = "edge" if c.mode == "edge" else "center"
            return f"← {c.value:.2f} mm from left edge ({ref})"
        if c.kind == "dist_top_edge" and c.value is not None:
            ref = "edge" if c.mode == "edge" else "center"
            return f"↑ {c.value:.2f} mm from top edge ({ref})"
        if c.kind == "point_on_line":
            return "Point on line"
        if c.kind == "parallel":
            return "Parallel"
        if c.kind == "perpendicular":
            return "Perpendicular"
        if c.kind == "equal_length":
            return "Equal length"
        if c.kind == "tangent_cc":
            return "Tangent"
        if c.kind == "symmetric_pp":
            return "Symmetric"
        return c.kind

    # ── Mutators ──────────────────────────────────────────────────

    def _is_locked(self, well: Well) -> bool:
        if self._design is None:
            return False
        for c in self._design.constraints:
            if c.kind in ("ground", "fix") and well.center in c.refs:
                return True
        return False

    def _set_design_name(self, new_name: str) -> None:
        if self._design is None:
            return
        new_name = new_name.strip()
        if not new_name or new_name == self._design.name:
            return
        self._design.name = new_name
        self._dirty = True
        self._update_dirty_label()

    def _set_outline_width(self, value: float) -> None:
        if self._design is None:
            return
        self._design.outline.width = value
        self._canvas._rebuild_scene()
        self._dirty = True
        self._update_dirty_label()

    def _set_outline_height(self, value: float) -> None:
        if self._design is None:
            return
        self._design.outline.height = value
        self._canvas._rebuild_scene()
        self._dirty = True
        self._update_dirty_label()

    def _rename_well(self, well_id: EntityId, new_name: str) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            new_name = new_name.strip()
            if not new_name or new_name == ent.name:
                return
            ent.name = new_name
            ent.naming_scheme = "MANUAL"
            self._dirty = True
            self._update_dirty_label()
            self._rebuild_properties_panel()

    def _set_well_x(self, well_id: EntityId, value: float) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            center = self._design.entities.get(ent.center)
            if isinstance(center, Point):
                center.x = value
                if self._canvas.solver:
                    report = self._canvas.solver.solve()
                    self._canvas._apply_solver_result()
                    self._canvas.solve_report.emit(report)
                self._dirty = True
                self._update_dirty_label()

    def _set_well_y(self, well_id: EntityId, value: float) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            center = self._design.entities.get(ent.center)
            if isinstance(center, Point):
                center.y = value
                if self._canvas.solver:
                    report = self._canvas.solver.solve()
                    self._canvas._apply_solver_result()
                    self._canvas.solve_report.emit(report)
                self._dirty = True
                self._update_dirty_label()

    def _set_well_diameter(self, well_id: EntityId, value: float) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            ent.diameter = value
            self._canvas._rebuild_scene()
            self._dirty = True
            self._update_dirty_label()

    def _set_well_depth(self, well_id: EntityId, value: float) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            ent.well_depth_mm = value
            self._dirty = True
            self._update_dirty_label()

    def _set_well_rim(self, well_id: EntityId, value: float) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            ent.rim_height_mm = value
            self._dirty = True
            self._update_dirty_label()

    def _set_well_ink_z(self, well_id: EntityId, value) -> None:
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            ent.ink_z_mm = value      # float or None
            self._dirty = True
            self._update_dirty_label()

    # ── Well-type presets (v7.5.x) ────────────────────────────────

    def _apply_well_type(self, well_id: EntityId, type_id) -> None:
        """Stamp a well-type preset's geometry onto a sub-well.

        ``type_id`` None = "(custom)" → keep the current geometry and just
        clear the preset link. Otherwise copy diameter / depth / rim height
        (and ink Z, only if the type prescribes one) from the type.
        """
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if not isinstance(ent, Well):
            return
        if type_id is None:
            ent.well_type_id = None
        else:
            from SupportClasses.WellTypeStore import get_store
            wt = get_store().get(type_id)
            if wt is None:
                return
            ent.diameter = wt.diameter_mm
            ent.well_depth_mm = wt.well_depth_mm
            ent.rim_height_mm = wt.rim_height_mm
            if wt.ink_z_mm is not None:
                ent.ink_z_mm = wt.ink_z_mm
            ent.well_type_id = type_id
        self._dirty = True
        self._update_dirty_label()
        self._canvas._rebuild_scene()
        # Defer the panel rebuild — the combo that emitted this is the
        # sender and would otherwise be destroyed mid-signal.
        QTimer.singleShot(0, self._rebuild_properties_panel)

    def _save_current_as_well_type(self, well_id: EntityId) -> None:
        """Save the sub-well's current geometry as a user well type."""
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if not isinstance(ent, Well):
            return
        name, ok = QInputDialog.getText(
            self, "Save Well Type", "Well type name:")
        if not ok or not name.strip():
            return
        from SupportClasses.WellTypeStore import WellType, get_store, safe_id
        tid = safe_id(name.strip().lower().replace(" ", "-"))
        wt = WellType(
            id=tid,
            display_name=name.strip(),
            diameter_mm=ent.diameter,
            well_depth_mm=ent.well_depth_mm,
            rim_height_mm=ent.rim_height_mm,
            ink_z_mm=ent.ink_z_mm,
        )
        if not get_store().save_user(wt):
            QMessageBox.warning(
                self, "Save failed", "Could not save the well type.")
            return
        ent.well_type_id = tid
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()
        QMessageBox.information(
            self, "Saved", f"Well type '{name.strip()}' saved.")

    def _delete_selected_well_type(self, type_id) -> None:
        """Delete a USER well type (built-ins cannot be removed)."""
        if not type_id:
            return
        from SupportClasses.WellTypeStore import get_store
        store = get_store()
        wt = store.get(type_id)
        if wt is None or wt.builtin:
            QMessageBox.information(
                self, "Cannot delete",
                "Built-in well types cannot be deleted.")
            return
        if QMessageBox.question(
                self, "Delete well type",
                f"Delete well type '{wt.label}'?") != QMessageBox.Yes:
            return
        store.delete_user(type_id)
        self._rebuild_properties_panel()

    def _set_rosette_rotation(self, well_id: EntityId, value: float) -> None:
        """Rotate a well's rosette layout (top-level design)."""
        if self._design is None:
            return
        ent = self._design.entities.get(well_id)
        if isinstance(ent, Well):
            ent.rosette_rotation_deg = value
            self._dirty = True
            self._update_dirty_label()
            self._canvas._rebuild_scene()   # redraw the rosette badge

    def _on_drop_standard_insert(self, well_id: EntityId) -> None:
        """Pick a saved standard insert and copy it into the well."""
        from SupportClasses.PlateDesign import (
            list_standard_inserts, load_standard_insert,
        )
        names = list_standard_inserts()
        if not names:
            QMessageBox.information(
                self, "No standard inserts",
                "No standard inserts saved yet. Edit a rosette and use "
                "'Save as standard insert…' to create one.")
            return
        name, ok = QInputDialog.getItem(
            self, "Drop standard insert",
            "Choose an insert to place in this well:", names, 0, False)
        if not ok or not name:
            return
        ent = self._design.entities.get(well_id) if self._design else None
        if not isinstance(ent, Well):
            return
        try:
            insert = load_standard_insert(name)
        except Exception as e:
            QMessageBox.warning(self, "Load failed", str(e))
            return
        self._canvas.push_undo_snapshot()
        ent.rosette_design = insert
        self._dirty = True
        self._update_dirty_label()
        self._canvas._rebuild_scene()
        self._rebuild_properties_panel()

    def _set_diameter_batch(self, value: float) -> None:
        if self._design is None:
            return
        for eid in self._canvas.selected_ids():
            ent = self._design.entities.get(eid)
            if isinstance(ent, Well):
                ent.diameter = value
        self._canvas._rebuild_scene()
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()

    def _add_constraint_to_selection(
        self, kind: str, value: Optional[float] = None,
    ) -> None:
        if self._canvas.add_constraint_now(kind, value=value):
            self._dirty = True
            self._update_dirty_label()
            self._rebuild_properties_panel()

    def _remove_constraint(self, constraint_id: int) -> None:
        if self._design is None:
            return
        self._design.remove_constraint(constraint_id)
        if self._canvas.solver:
            report = self._canvas.solver.solve()
            self._canvas._apply_solver_result()
            self._canvas.solve_report.emit(report)
        self._dirty = True
        self._update_dirty_label()
        self._rebuild_properties_panel()
