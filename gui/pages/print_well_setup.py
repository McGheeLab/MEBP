"""
print_well_setup.py — Tab 3: Well Setup & Assignment for MEBP v7.1.

Complete well plate setup page:
- Interactive plate view with multi-select (WellPlateView widget)
- ZY side projection + XZ bottom projection (well bottoms + needle)
- Selection actions: role assignment, print assignment, rosette attach
- Rosette sub-well role editor
- Well bottom plane detection (teach + fit)
- Service sequence configuration
- Assignment summary table (scrollable, synced with plate)
- Auto-assign patterns (block, checkerboard, border)
- Save/Load well setup JSON

Layout matches the coding plan Tab 3 wireframe:
┌───────────────────────────────────┬────────────────────┐
│  Interactive Plate View (XY)      │ ZY Side Projection │
├───────────────────────────────────┴────────────────────┤
│  XZ Bottom Projection                                  │
├────────────────────────────────────────────────────────┤
│  Selection Actions + Rosette Editor                    │
├────────────────────────────────────────────────────────┤
│  Well Bottom Detection                                 │
├────────────────────────────────────────────────────────┤
│  Assignment Summary Table                              │
└────────────────────────────────────────────────────────┘

Session G — Tasks P5.19, P5.23, P5.26, P5.27, P5.32, P5.33, P5.34, P5.35.

v7.1 minor gap fix: MiniProjectionView now imported from
gui.widgets.projection_canvas (unified L-shaped projection widget)
instead of being defined inline.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QSpinBox,
    QFileDialog, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QScrollArea, QSizePolicy,
    QMenu, QSplitter,
)
from PySide6.QtCore import Qt, Signal, QPointF, QTimer
from PySide6.QtGui import QColor, QCursor

from gui.styles import COLORS
from gui.widgets.well_plate_view import WellPlateView, WellRoleLegend
from gui.widgets.projection_canvas import MiniProjectionView

from SupportClasses.PhysicalModels import (
    WellRole, ROLE_COLORS, InkSpec, RosetteInsert, WorkspaceConfig,
)
from SupportClasses.WellPlate import WellPlate, ROW_LABELS
from SupportClasses.WellSetup import (
    WellAssignment, WellSetupModel, ServiceSequence,
    WashBehavior, WasteBehavior, BufferBehavior,
    InkPickupBehavior, SortedCellBehavior, PlaneResult,
    auto_assign_block, auto_assign_checkerboard, auto_assign_border,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Well Setup Tab (Tab 3)
# ═══════════════════════════════════════════════════════════════════

class WellSetupTab(QWidget):
    """
    Tab 3: Well Setup & Assignment.

    Manages:
    - Plate visualization with interactive multi-select
    - Role assignment for selected wells
    - Print collection assignment
    - Rosette insert attachment + sub-well editing
    - Well bottom plane calibration
    - Service sequence configuration
    - Summary table of all assignments
    - Save/Load well setup to JSON
    """

    # Emitted when assignments change (for cross-tab sync)
    setup_changed = Signal()

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

        # Model
        self._model = WellSetupModel(self._workspace.plate_format)

        # Available print collections (populated from Tab 2)
        self._available_prints: list[str] = []

        self._build_ui()
        self._connect_signals()
        self._refresh_plate()

    # ── Properties ────────────────────────────────────────────────

    @property
    def model(self) -> WellSetupModel:
        return self._model

    def set_workspace(self, workspace: WorkspaceConfig) -> None:
        """Update workspace config (called when Tab 1 changes)."""
        self._workspace = workspace
        if workspace.plate_format != self._model.plate_format:
            self._model.set_plate_format(workspace.plate_format)
            self._refresh_plate()

    def set_available_prints(self, names: list[str]) -> None:
        """Update available print collections (called when Tab 2 changes)."""
        self._available_prints = list(names)
        self._refresh_print_combo()

    # ── UI Construction ───────────────────────────────────────────

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # ── Top area: Plate view + ZY projection ──────────────────
        top_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Plate view
        plate_container = QWidget()
        plate_layout = QVBoxLayout(plate_container)
        plate_layout.setContentsMargins(0, 0, 0, 0)
        plate_layout.setSpacing(2)

        self.plate_view = WellPlateView()
        plate_layout.addWidget(self.plate_view, stretch=1)

        # Legend + selection count
        info_row = QHBoxLayout()
        self.legend = WellRoleLegend()
        info_row.addWidget(self.legend)
        self.selection_label = QLabel("[0 wells selected]")
        self.selection_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        info_row.addWidget(self.selection_label)
        plate_layout.addLayout(info_row)

        top_splitter.addWidget(plate_container)

        # ZY side projection (uses unified projection_canvas widget)
        self.zy_view = MiniProjectionView("ZY")
        top_splitter.addWidget(self.zy_view)
        top_splitter.setStretchFactor(0, 5)
        top_splitter.setStretchFactor(1, 1)

        main_layout.addWidget(top_splitter, stretch=3)

        # ── XZ bottom projection ──────────────────────────────────
        self.xz_view = MiniProjectionView("XZ")
        main_layout.addWidget(self.xz_view)

        # ── Scroll area for panels below ──────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(4)

        # ── Selection Actions Group ───────────────────────────────
        self._build_selection_actions(scroll_layout)

        # ── Rosette Sub-well Editor ───────────────────────────────
        self._build_rosette_editor(scroll_layout)

        # ── Well Bottom Detection ─────────────────────────────────
        self._build_plane_detection(scroll_layout)

        # ── Service Sequence ──────────────────────────────────────
        self._build_service_sequence(scroll_layout)

        # ── Assignment Summary Table ──────────────────────────────
        self._build_summary_table(scroll_layout)

        # ── Save/Load + Auto-assign ───────────────────────────────
        self._build_bottom_buttons(scroll_layout)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll, stretch=2)

    # ── Selection Actions ─────────────────────────────────────────

    def _build_selection_actions(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("Selection Actions")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}")
        layout = QGridLayout(group)
        layout.setSpacing(4)

        # Row 0: Role assignment
        layout.addWidget(QLabel("Set Role:"), 0, 0)
        self.role_combo = QComboBox()
        for role in WellRole:
            self.role_combo.addItem(role.value.capitalize(), role)
        layout.addWidget(self.role_combo, 0, 1)

        self.role_index_spin = QSpinBox()
        self.role_index_spin.setRange(0, 20)
        self.role_index_spin.setPrefix("# ")
        self.role_index_spin.setToolTip("Role index (e.g., Ink 1, Ink 2)")
        layout.addWidget(self.role_index_spin, 0, 2)

        btn_set_role = QPushButton("Apply Role")
        btn_set_role.setObjectName("accentBtn")
        btn_set_role.clicked.connect(self._apply_role)
        layout.addWidget(btn_set_role, 0, 3)

        # Row 1: Ink assignment (for INK wells)
        layout.addWidget(QLabel("Ink:"), 1, 0)
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("(none)", None)
        layout.addWidget(self.ink_combo, 1, 1, 1, 2)

        btn_set_ink = QPushButton("Set Ink")
        btn_set_ink.clicked.connect(self._apply_ink)
        layout.addWidget(btn_set_ink, 1, 3)

        # Row 2: Print assignment
        layout.addWidget(QLabel("Print:"), 2, 0)
        self.print_combo = QComboBox()
        self.print_combo.addItem("(no prints available)", None)
        layout.addWidget(self.print_combo, 2, 1, 1, 2)

        btn_row = QHBoxLayout()
        btn_append = QPushButton("+Append")
        btn_append.clicked.connect(lambda: self._assign_print(replace=False))
        btn_row.addWidget(btn_append)
        btn_replace = QPushButton("Replace")
        btn_replace.clicked.connect(lambda: self._assign_print(replace=True))
        btn_row.addWidget(btn_replace)
        btn_clear_prints = QPushButton("Clear")
        btn_clear_prints.clicked.connect(self._clear_prints)
        btn_row.addWidget(btn_clear_prints)
        btn_container = QWidget()
        btn_container.setLayout(btn_row)
        layout.addWidget(btn_container, 2, 3)

        # Row 3: Rosette attachment
        layout.addWidget(QLabel("Rosette:"), 3, 0)
        self.rosette_combo = QComboBox()
        self.rosette_combo.addItem("None", None)
        layout.addWidget(self.rosette_combo, 3, 1, 1, 2)

        btn_attach = QPushButton("Attach")
        btn_attach.clicked.connect(self._attach_rosette)
        layout.addWidget(btn_attach, 3, 3)

        # Row 4: Quick actions
        btn_row2 = QHBoxLayout()
        btn_clear = QPushButton("Clear Selection")
        btn_clear.clicked.connect(self._clear_selected_assignments)
        btn_row2.addWidget(btn_clear)

        btn_select_role = QPushButton("Select Same Role")
        btn_select_role.clicked.connect(self._select_same_role)
        btn_row2.addWidget(btn_select_role)

        btn_container2 = QWidget()
        btn_container2.setLayout(btn_row2)
        layout.addWidget(btn_container2, 4, 0, 1, 4)

        parent_layout.addWidget(group)

    # ── Rosette Sub-well Editor ───────────────────────────────────

    def _build_rosette_editor(self, parent_layout: QVBoxLayout) -> None:
        self.rosette_group = QGroupBox("Rosette Sub-well Roles")
        self.rosette_group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}")
        self.rosette_group.setVisible(False)  # Hidden until rosette attached

        layout = QVBoxLayout(self.rosette_group)
        self.subwell_table = QTableWidget()
        self.subwell_table.setColumnCount(3)
        self.subwell_table.setHorizontalHeaderLabels(
            ["Sub-well", "Role", "Ink"])
        self.subwell_table.horizontalHeader().setStretchLastSection(True)
        self.subwell_table.verticalHeader().setVisible(False)
        self.subwell_table.setMaximumHeight(200)
        layout.addWidget(self.subwell_table)

        parent_layout.addWidget(self.rosette_group)

    # ── Well Bottom Plane Detection ───────────────────────────────

    def _build_plane_detection(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("Well Bottom Detection")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}")
        layout = QVBoxLayout(group)

        self.plane_info_label = QLabel(
            "Jog needle to glass surface in 3+ wells → Calculate plane")
        self.plane_info_label.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        self.plane_info_label.setWordWrap(True)
        layout.addWidget(self.plane_info_label)

        # Teach point display
        self.teach_points_label = QLabel("Teach points: (none)")
        self.teach_points_label.setStyleSheet(f"color: {COLORS['text']};")
        layout.addWidget(self.teach_points_label)

        # Plane result display
        self.plane_result_label = QLabel("")
        self.plane_result_label.setStyleSheet(f"color: {COLORS['green']};")
        layout.addWidget(self.plane_result_label)

        # Buttons
        btn_row = QHBoxLayout()
        btn_teach = QPushButton("Teach Current Well")
        btn_teach.setToolTip("Record Z position at selected well")
        btn_teach.clicked.connect(self._teach_well)
        btn_row.addWidget(btn_teach)

        btn_fit = QPushButton("Calculate Plane")
        btn_fit.setObjectName("accentBtn")
        btn_fit.clicked.connect(self._fit_plane)
        btn_row.addWidget(btn_fit)

        btn_clear_pts = QPushButton("Clear Points")
        btn_clear_pts.clicked.connect(self._clear_teach_points)
        btn_row.addWidget(btn_clear_pts)

        layout.addLayout(btn_row)
        parent_layout.addWidget(group)

    # ── Service Sequence ──────────────────────────────────────────

    def _build_service_sequence(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("Service Sequence")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}")
        layout = QHBoxLayout(group)

        layout.addWidget(QLabel("Preset:"))
        self.service_preset_combo = QComboBox()
        seq = ServiceSequence()
        for name in seq.PRESETS:
            self.service_preset_combo.addItem(name)
        self.service_preset_combo.currentTextChanged.connect(
            self._apply_service_preset)
        layout.addWidget(self.service_preset_combo)

        self.service_steps_label = QLabel("waste → wash → buffer → ink")
        self.service_steps_label.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        layout.addWidget(self.service_steps_label)
        layout.addStretch()

        parent_layout.addWidget(group)

    # ── Assignment Summary Table ──────────────────────────────────

    def _build_summary_table(self, parent_layout: QVBoxLayout) -> None:
        group = QGroupBox("Well Assignment Summary")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS['text']}; font-weight: bold; "
            f"border: 1px solid {COLORS['surface1']}; border-radius: 4px; "
            f"margin-top: 6px; padding-top: 14px; }}")
        layout = QVBoxLayout(group)

        self.summary_table = QTableWidget()
        self.summary_table.setColumnCount(5)
        self.summary_table.setHorizontalHeaderLabels(
            ["Well", "Role", "Insert", "Prints", "Z Offset"])
        self.summary_table.horizontalHeader().setStretchLastSection(True)
        self.summary_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents)
        self.summary_table.verticalHeader().setVisible(False)
        self.summary_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.summary_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self.summary_table.setMaximumHeight(200)
        self.summary_table.setAlternatingRowColors(True)
        self.summary_table.setStyleSheet(
            f"QTableWidget {{ background-color: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; gridline-color: {COLORS['surface1']}; }}"
            f"QTableWidget::item:alternate {{ background-color: {COLORS['base']}; }}")

        # Click summary row → select well on plate
        self.summary_table.cellClicked.connect(self._on_summary_row_clicked)
        layout.addWidget(self.summary_table)
        parent_layout.addWidget(group)

    # ── Bottom Buttons ────────────────────────────────────────────

    def _build_bottom_buttons(self, parent_layout: QVBoxLayout) -> None:
        row = QHBoxLayout()

        btn_save = QPushButton("Save Layout")
        btn_save.clicked.connect(self._save_layout)
        row.addWidget(btn_save)

        btn_load = QPushButton("Load Layout")
        btn_load.clicked.connect(self._load_layout)
        row.addWidget(btn_load)

        # Auto-assign dropdown button
        btn_auto = QPushButton("Auto-assign Pattern...")
        menu = QMenu(self)
        menu.addAction("Block → Print",
                       lambda: self._auto_assign("block_print"))
        menu.addAction("Checkerboard (Print/Empty)",
                       lambda: self._auto_assign("checker"))
        menu.addAction("Border = Service, Inner = Print",
                       lambda: self._auto_assign("border"))
        menu.addAction("Row A = Service, Rest = Print",
                       lambda: self._auto_assign("row_service"))
        btn_auto.setMenu(menu)
        row.addWidget(btn_auto)

        btn_validate = QPushButton("Validate Setup")
        btn_validate.setObjectName("accentBtn")
        btn_validate.clicked.connect(self._validate_setup)
        row.addWidget(btn_validate)

        row.addStretch()
        parent_layout.addLayout(row)

    # ── Signal Connections ────────────────────────────────────────

    def _connect_signals(self) -> None:
        self.plate_view.selection_changed.connect(
            self._on_selection_changed)
        self.plate_view.well_double_clicked.connect(
            self._on_well_double_clicked)
        self.plate_view.context_menu_requested.connect(
            self._on_context_menu)

    # ── Refresh Helpers ───────────────────────────────────────────

    def _refresh_plate(self) -> None:
        """Rebuild plate view from model."""
        self.plate_view.set_plate(self._model.plate)
        self._refresh_well_colors()
        self._refresh_projections()
        self._refresh_summary()
        self._refresh_ink_combo()
        self._refresh_rosette_combo()

    def _refresh_well_colors(self) -> None:
        """Update all well colors/labels from model assignments."""
        appearances: dict[str, tuple[WellRole, str]] = {}
        for name, wa in self._model.assignments.items():
            appearances[name] = (wa.role, wa.get_display_label())
        self.plate_view.update_all_wells(appearances)

    def _refresh_projections(self) -> None:
        """Update ZY and XZ projection views."""
        z_offsets = {}
        for name, wa in self._model.assignments.items():
            z_offsets[name] = wa.get_effective_z()
        self.zy_view.set_plate_data(self._model.plate, z_offsets)
        self.xz_view.set_plate_data(self._model.plate, z_offsets)

    def _refresh_summary(self) -> None:
        """Rebuild the assignment summary table."""
        data = self._model.get_assignment_summary()
        self.summary_table.setRowCount(len(data))
        for i, row in enumerate(data):
            self.summary_table.setItem(
                i, 0, QTableWidgetItem(row["well"]))
            self.summary_table.setItem(
                i, 1, QTableWidgetItem(row["role"]))
            self.summary_table.setItem(
                i, 2, QTableWidgetItem(row["insert"]))
            self.summary_table.setItem(
                i, 3, QTableWidgetItem(row["prints"]))
            self.summary_table.setItem(
                i, 4, QTableWidgetItem(row["z_offset"]))

            # Color the role cell
            color = QColor(row["color"])
            role_item = self.summary_table.item(i, 1)
            if role_item:
                role_item.setForeground(color)

    def _refresh_ink_combo(self) -> None:
        """Populate ink combo from workspace ink library."""
        self.ink_combo.clear()
        self.ink_combo.addItem("(none)", None)
        for name in self._workspace.ink_library:
            self.ink_combo.addItem(name, name)

    def _refresh_rosette_combo(self) -> None:
        """Populate rosette combo from workspace rosette library."""
        self.rosette_combo.clear()
        self.rosette_combo.addItem("None", None)
        for name in self._workspace.rosette_library:
            self.rosette_combo.addItem(name, name)

    def _refresh_print_combo(self) -> None:
        """Populate print collection combo."""
        self.print_combo.clear()
        if self._available_prints:
            for name in self._available_prints:
                self.print_combo.addItem(name, name)
        else:
            self.print_combo.addItem("(no prints available)", None)

    def _refresh_teach_display(self) -> None:
        """Update teach point and plane result labels."""
        detector = self._model.detector
        if detector.num_points == 0:
            self.teach_points_label.setText("Teach points: (none)")
        else:
            pts = []
            for p in detector.points:
                pts.append(f"[{p.well_name}: z={p.z_mm:+.3f}]")
            self.teach_points_label.setText(
                f"Teach points: {' '.join(pts)}")

        result = detector.result
        if result:
            self.plane_result_label.setText(
                f"Plane: {result.describe()}")
            self.plane_result_label.setStyleSheet(
                f"color: {COLORS['green']};")
        else:
            self.plane_result_label.setText("")

    def _refresh_rosette_editor(
        self, well_name: str | None = None,
    ) -> None:
        """Show/hide and populate rosette sub-well editor."""
        if well_name is None:
            self.rosette_group.setVisible(False)
            return

        wa = self._model.get_assignment(well_name)
        if not wa.rosette_name:
            self.rosette_group.setVisible(False)
            return

        rosette = self._workspace.get_rosette(wa.rosette_name)
        if not rosette:
            self.rosette_group.setVisible(False)
            return

        self.rosette_group.setVisible(True)
        n = rosette.num_subwells
        self.subwell_table.setRowCount(n)

        for i in range(n):
            # Sub-well label
            label = (wa.subwell_labels[i]
                     if i < len(wa.subwell_labels) else f"SW{i}")
            self.subwell_table.setItem(i, 0, QTableWidgetItem(label))

            # Role combo
            role_combo = QComboBox()
            for role in WellRole:
                role_combo.addItem(role.value.capitalize(), role.value)
            if i < len(wa.subwell_roles):
                idx = role_combo.findData(wa.subwell_roles[i])
                if idx >= 0:
                    role_combo.setCurrentIndex(idx)
            role_combo.currentIndexChanged.connect(
                partial(self._on_subwell_role_changed, well_name, i))
            self.subwell_table.setCellWidget(i, 1, role_combo)

            # Ink combo
            ink_combo = QComboBox()
            ink_combo.addItem("(none)", None)
            for ink_name in self._workspace.ink_library:
                ink_combo.addItem(ink_name, ink_name)
            if i < len(wa.subwell_inks) and wa.subwell_inks[i]:
                idx = ink_combo.findData(wa.subwell_inks[i])
                if idx >= 0:
                    ink_combo.setCurrentIndex(idx)
            ink_combo.currentIndexChanged.connect(
                partial(self._on_subwell_ink_changed, well_name, i))
            self.subwell_table.setCellWidget(i, 2, ink_combo)

    # ── Selection Handlers ────────────────────────────────────────

    def _on_selection_changed(self, well_names: list[str]) -> None:
        """Handle plate view selection change."""
        n = len(well_names)
        self.selection_label.setText(
            f"[{n} well{'s' if n != 1 else ''} selected]")

        # Show rosette editor for single selection
        if n == 1:
            self._refresh_rosette_editor(well_names[0])
        else:
            self._refresh_rosette_editor(None)

    def _on_well_double_clicked(self, well_name: str) -> None:
        """Double-click: could open detail editor (future)."""
        logger.info(f"Double-clicked well {well_name}")

    def _on_context_menu(
        self, well_names: list[str], pos: QPointF,
    ) -> None:
        """Right-click context menu on selected wells."""
        menu = QMenu(self)

        # Role submenu
        role_menu = menu.addMenu("Set Role")
        for role in WellRole:
            action = role_menu.addAction(role.value.capitalize())
            action.triggered.connect(
                partial(self._context_set_role, well_names, role))

        # Print submenu
        if self._available_prints:
            print_menu = menu.addMenu("Assign Print")
            for name in self._available_prints:
                action = print_menu.addAction(name)
                action.triggered.connect(
                    partial(self._context_assign_print, well_names, name))

        menu.addSeparator()
        menu.addAction("Clear Assignments",
                       lambda: self._clear_wells(well_names))
        menu.addAction("Reset to Empty",
                       lambda: self._clear_wells(well_names))
        menu.addSeparator()
        menu.addAction("Select All with Same Role",
                       lambda: self._select_same_role_from(well_names))

        menu.exec(QCursor.pos())

    def _on_summary_row_clicked(self, row: int, col: int) -> None:
        """Click summary table row → select that well on plate."""
        item = self.summary_table.item(row, 0)
        if item:
            self.plate_view.set_selection([item.text()])

    # ── Action Handlers ───────────────────────────────────────────

    def _apply_role(self) -> None:
        """Apply selected role to all selected wells."""
        selected = self.plate_view.get_selected_wells()
        if not selected:
            return
        role = self.role_combo.currentData()
        idx = self.role_index_spin.value()
        self._model.set_role(selected, role, idx)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _apply_ink(self) -> None:
        """Apply ink to selected INK wells."""
        selected = self.plate_view.get_selected_wells()
        if not selected:
            return
        ink_name = self.ink_combo.currentData()
        self._model.set_ink(selected, ink_name)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _assign_print(self, replace: bool = False) -> None:
        """Assign print collection to selected wells."""
        selected = self.plate_view.get_selected_wells()
        if not selected:
            return
        print_name = self.print_combo.currentData()
        if not print_name:
            return
        self._model.assign_print(selected, print_name, replace=replace)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _clear_prints(self) -> None:
        """Clear print assignments from selected wells."""
        selected = self.plate_view.get_selected_wells()
        for name in selected:
            wa = self._model.get_assignment(name)
            wa.clear_prints()
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _attach_rosette(self) -> None:
        """Attach rosette to selected wells."""
        selected = self.plate_view.get_selected_wells()
        if not selected:
            return
        rosette_name = self.rosette_combo.currentData()
        self._model.attach_rosette(
            selected, rosette_name, self._workspace)
        self._refresh_well_colors()
        self._refresh_summary()
        if len(selected) == 1:
            self._refresh_rosette_editor(selected[0])
        self.setup_changed.emit()

    def _clear_selected_assignments(self) -> None:
        """Clear assignments for selected wells."""
        selected = self.plate_view.get_selected_wells()
        self._clear_wells(selected)

    def _clear_wells(self, well_names: list[str]) -> None:
        """Reset wells to EMPTY."""
        self._model.clear_assignments(well_names)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _select_same_role(self) -> None:
        """Select all wells with same role as first selected well."""
        selected = self.plate_view.get_selected_wells()
        if not selected:
            return
        first_wa = self._model.get_assignment(selected[0])
        same = self._model.get_wells_by_role(first_wa.role)
        self.plate_view.set_selection(same)

    def _select_same_role_from(self, well_names: list[str]) -> None:
        """Select all wells matching role of given wells."""
        if not well_names:
            return
        wa = self._model.get_assignment(well_names[0])
        same = self._model.get_wells_by_role(wa.role)
        self.plate_view.set_selection(same)

    def _context_set_role(
        self, well_names: list[str], role: WellRole,
    ) -> None:
        """Context menu: set role."""
        self._model.set_role(well_names, role)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    def _context_assign_print(
        self, well_names: list[str], name: str,
    ) -> None:
        """Context menu: assign print."""
        self._model.assign_print(well_names, name)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    # ── Sub-well Editor Handlers ──────────────────────────────────

    def _on_subwell_role_changed(
        self, well_name: str, subwell_idx: int, combo_idx: int,
    ) -> None:
        """Handle sub-well role combo change."""
        wa = self._model.get_assignment(well_name)
        combo = self.subwell_table.cellWidget(subwell_idx, 1)
        if combo and isinstance(combo, QComboBox):
            role_str = combo.currentData()
            if subwell_idx < len(wa.subwell_roles):
                wa.subwell_roles[subwell_idx] = role_str
        self.setup_changed.emit()

    def _on_subwell_ink_changed(
        self, well_name: str, subwell_idx: int, combo_idx: int,
    ) -> None:
        """Handle sub-well ink combo change."""
        wa = self._model.get_assignment(well_name)
        combo = self.subwell_table.cellWidget(subwell_idx, 2)
        if combo and isinstance(combo, QComboBox):
            ink_name = combo.currentData()
            if subwell_idx < len(wa.subwell_inks):
                wa.subwell_inks[subwell_idx] = ink_name
        self.setup_changed.emit()

    # ── Well Bottom Detection ─────────────────────────────────────

    def _teach_well(self) -> None:
        """Record current Z position at selected well."""
        selected = self.plate_view.get_selected_wells()
        if len(selected) != 1:
            self.plane_info_label.setText(
                "Select exactly 1 well to teach")
            return

        well_name = selected[0]

        # Get current Z from controller
        z_mm = 0.0
        if self._controller:
            try:
                pos = self._controller.get_zp_position()
                if pos:
                    z_mm = pos.get("Z", 0.0)
                    # Convert steps to mm if needed
                    if abs(z_mm) > 100:
                        z_mm = z_mm / 1000.0  # Rough conversion
            except Exception as e:
                logger.warning(f"Could not read Z position: {e}")
                self.plane_info_label.setText(f"Error reading Z: {e}")
                return

        self._model.teach_well_z(well_name, z_mm)
        self._refresh_teach_display()

    def _fit_plane(self) -> None:
        """Calculate well bottom plane from teach points."""
        result = self._model.fit_plane()
        if result:
            self.plane_info_label.setText(
                f"Plane fitted with R²={result.r_squared:.4f}")
            self._refresh_projections()
        else:
            self.plane_info_label.setText(
                "Need ≥3 teach points to fit plane")
        self._refresh_teach_display()
        self._refresh_summary()

    def _clear_teach_points(self) -> None:
        """Clear all teach points."""
        self._model.detector.clear()
        self._refresh_teach_display()
        self.plane_result_label.setText("")
        self.plane_info_label.setText(
            "Jog needle to glass surface in 3+ wells → Calculate plane")

    # ── Service Sequence ──────────────────────────────────────────

    def _apply_service_preset(self, preset_name: str) -> None:
        """Apply a service sequence preset."""
        self._model.service_sequence = ServiceSequence.from_preset(
            preset_name)
        steps = self._model.service_sequence.steps
        self.service_steps_label.setText(
            " → ".join(steps) if steps else "(none)")
        self.setup_changed.emit()

    # ── Auto-assign Patterns ──────────────────────────────────────

    def _auto_assign(self, pattern: str) -> None:
        """Apply an auto-assign pattern."""
        plate = self._model.plate

        if pattern == "block_print":
            # All wells = PRINT
            result = {
                w.name: WellRole.PRINT for w in plate.get_all_wells()
            }
        elif pattern == "checker":
            result = auto_assign_checkerboard(
                plate, WellRole.PRINT, WellRole.EMPTY)
        elif pattern == "border":
            result = auto_assign_border(
                plate, WellRole.WASH, WellRole.PRINT)
        elif pattern == "row_service":
            # Row A = service wells, rest = print
            result = {}
            for w in plate.get_all_wells():
                if w.row == 0:
                    # Distribute service roles across row A
                    services = [WellRole.WASH, WellRole.WASTE,
                                WellRole.BUFFER, WellRole.INK]
                    result[w.name] = services[w.col % len(services)]
                else:
                    result[w.name] = WellRole.PRINT
        else:
            return

        self._model.apply_auto_pattern(result)
        self._refresh_well_colors()
        self._refresh_summary()
        self.setup_changed.emit()

    # ── Validate ──────────────────────────────────────────────────

    def _validate_setup(self) -> None:
        """Run validation and display results."""
        issues = self._model.validate(self._workspace)
        if not issues:
            self.plane_info_label.setText(
                "✅ Setup is valid and ready to print")
            self.plane_info_label.setStyleSheet(
                f"color: {COLORS['green']};")
        else:
            text = "⚠ Issues found:\n" + "\n".join(
                f"  • {i}" for i in issues)
            self.plane_info_label.setText(text)
            self.plane_info_label.setStyleSheet(
                f"color: {COLORS['yellow']};")

    # ── Save/Load ─────────────────────────────────────────────────

    def _save_layout(self) -> None:
        """Save well setup to JSON."""
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Well Setup", "", "JSON (*.json)")
        if filepath:
            self._model.save_json(filepath)
            self.plane_info_label.setText(f"Saved: {filepath}")

    def _load_layout(self) -> None:
        """Load well setup from JSON."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Well Setup", "", "JSON (*.json)")
        if filepath:
            try:
                self._model = WellSetupModel.load_json(filepath)
                self._refresh_plate()
                self.plane_info_label.setText(f"Loaded: {filepath}")
                self.setup_changed.emit()
            except Exception as e:
                self.plane_info_label.setText(f"Load error: {e}")
                logger.error(f"Failed to load well setup: {e}")

    # ── External Update API ───────────────────────────────────────

    def update_needle_position(
        self,
        x_mm: float | None,
        y_mm: float | None,
        z_mm: float | None = None,
    ) -> None:
        """
        Update needle position overlay on all views.

        Called by the main app's position poller.
        """
        self.plate_view.set_needle_position(x_mm, y_mm)
        self.zy_view.set_needle_position(x_mm, y_mm, z_mm)
        self.xz_view.set_needle_position(x_mm, y_mm, z_mm)
