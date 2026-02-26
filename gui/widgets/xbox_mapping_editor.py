"""
Xbox Mapping Editor - Dialog for editing controller button/axis/dpad mappings.

Provides a table-based editor for current_button_mapping.json with
dropdown selectors for mapping each input to a command.
"""

import json
import logging
from pathlib import Path

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QComboBox, QPushButton, QLabel, QGroupBox, QHeaderView, QTabWidget,
    QWidget, QMessageBox, QFileDialog,
)
from PySide6.QtCore import Qt

logger = logging.getLogger(__name__)

# All available commands that can be mapped
AVAILABLE_COMMANDS = [
    "None",
    "move_stage_at_velocity",
    "move_z_at_velocity",
    "move_p1_at_velocity",
    "move_p2_at_velocity",
    "move_p3_at_velocity",
    "zero_needle_pos",
    "increment_xyspeed_up",
    "increment_xyspeed_down",
    "increment_zspeed_up",
    "increment_zspeed_down",
    "increment_pspeed_up",
    "increment_pspeed_down",
]

# Human-readable labels for Xbox controller inputs
BUTTON_LABELS = {
    "0": "A",
    "1": "B",
    "2": "X",
    "3": "Y",
    "4": "LB (Left Bumper)",
    "5": "RB (Right Bumper)",
    "6": "Back / Select",
    "7": "Start / Menu",
    "8": "L3 (Left Stick Press)",
    "9": "R3 (Right Stick Press)",
    "10": "Guide / Xbox",
    "11": "Share",
}

AXIS_LABELS = {
    "0-1": "Left Stick (X/Y)",
    "2-3": "Right Stick (X/Y)",
    "4": "Left Trigger",
    "5": "Right Trigger",
}

DPAD_LABELS = {
    "up": "D-Pad Up",
    "down": "D-Pad Down",
    "left": "D-Pad Left",
    "right": "D-Pad Right",
}


class XboxMappingEditor(QDialog):
    """
    Dialog for editing Xbox controller button/axis/dpad mappings.
    
    Loads from and saves to a JSON mapping file.
    """

    def __init__(self, mapping_file: str = "current_button_mapping.json", parent=None):
        super().__init__(parent)
        self.mapping_file = mapping_file
        self.mapping = self._load_mapping()

        self.setWindowTitle("Xbox Controller Mapping Editor")
        self.setMinimumSize(500, 500)
        self.resize(550, 600)

        self._setup_ui()
        self._populate_tables()

    def _load_mapping(self) -> dict:
        """Load current mapping from file."""
        try:
            with open(self.mapping_file, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Failed to load mapping: {e}")
            return {"buttons": {}, "axes": {}, "dpad": {}}

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # Info label
        info = QLabel(
            "Map Xbox controller inputs to stage commands.\n"
            "Changes are saved to the mapping file and hot-reloaded by the controller."
        )
        info.setStyleSheet("color: #a6adc8; font-size: 9pt;")
        info.setWordWrap(True)
        layout.addWidget(info)

        # Tabs for buttons / axes / dpad
        tabs = QTabWidget()

        # ── Buttons tab ────────────────────────────────────────────
        btn_tab = QWidget()
        btn_layout = QVBoxLayout(btn_tab)
        self.btn_table = QTableWidget()
        self.btn_table.setColumnCount(3)
        self.btn_table.setHorizontalHeaderLabels(["#", "Input", "Command"])
        self.btn_table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch
        )
        self.btn_table.setColumnWidth(0, 30)
        self.btn_table.setColumnWidth(1, 160)
        self.btn_table.verticalHeader().setVisible(False)
        btn_layout.addWidget(self.btn_table)
        tabs.addTab(btn_tab, "🎮 Buttons")

        # ── Axes tab ──────────────────────────────────────────────
        axis_tab = QWidget()
        axis_layout = QVBoxLayout(axis_tab)
        self.axis_table = QTableWidget()
        self.axis_table.setColumnCount(3)
        self.axis_table.setHorizontalHeaderLabels(["Group", "Input", "Command"])
        self.axis_table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch
        )
        self.axis_table.setColumnWidth(0, 50)
        self.axis_table.setColumnWidth(1, 160)
        self.axis_table.verticalHeader().setVisible(False)
        axis_layout.addWidget(self.axis_table)
        tabs.addTab(axis_tab, "🕹️ Axes")

        # ── D-Pad tab ─────────────────────────────────────────────
        dpad_tab = QWidget()
        dpad_layout = QVBoxLayout(dpad_tab)
        self.dpad_table = QTableWidget()
        self.dpad_table.setColumnCount(3)
        self.dpad_table.setHorizontalHeaderLabels(["Direction", "Label", "Command"])
        self.dpad_table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch
        )
        self.dpad_table.setColumnWidth(0, 60)
        self.dpad_table.setColumnWidth(1, 120)
        self.dpad_table.verticalHeader().setVisible(False)
        dpad_layout.addWidget(self.dpad_table)
        tabs.addTab(dpad_tab, "✛ D-Pad")

        layout.addWidget(tabs)

        # ── Bottom buttons ─────────────────────────────────────────
        btn_row = QHBoxLayout()

        btn_reset = QPushButton("Reset to Defaults")
        btn_reset.clicked.connect(self._reset_defaults)
        btn_row.addWidget(btn_reset)

        btn_row.addStretch()

        btn_export = QPushButton("Export...")
        btn_export.clicked.connect(self._export_mapping)
        btn_row.addWidget(btn_export)

        btn_import = QPushButton("Import...")
        btn_import.clicked.connect(self._import_mapping)
        btn_row.addWidget(btn_import)

        btn_save = QPushButton("Save")
        btn_save.setObjectName("successBtn")
        btn_save.clicked.connect(self._save_mapping)
        btn_row.addWidget(btn_save)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(btn_cancel)

        layout.addLayout(btn_row)

    def _make_command_combo(self, current_value: str) -> QComboBox:
        """Create a command dropdown with the current value selected."""
        combo = QComboBox()
        combo.addItems(AVAILABLE_COMMANDS)
        idx = combo.findText(current_value)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        elif current_value:
            # Custom command not in default list; add it
            combo.addItem(current_value)
            combo.setCurrentIndex(combo.count() - 1)
        return combo

    def _populate_tables(self):
        """Fill all tables from the current mapping."""
        buttons = self.mapping.get("buttons", {})
        axes = self.mapping.get("axes", {})
        dpad = self.mapping.get("dpad", {})

        # ── Buttons ────────────────────────────────────────────────
        # Ensure all standard buttons are shown
        all_buttons = sorted(
            set(list(buttons.keys()) + list(BUTTON_LABELS.keys())),
            key=lambda x: int(x),
        )
        self.btn_table.setRowCount(len(all_buttons))
        self._btn_combos = {}

        for row, btn_id in enumerate(all_buttons):
            # ID column
            id_item = QTableWidgetItem(btn_id)
            id_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.btn_table.setItem(row, 0, id_item)

            # Label column
            label = BUTTON_LABELS.get(btn_id, f"Button {btn_id}")
            label_item = QTableWidgetItem(label)
            label_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.btn_table.setItem(row, 1, label_item)

            # Command combo
            current = buttons.get(btn_id, "None")
            combo = self._make_command_combo(current)
            self.btn_table.setCellWidget(row, 2, combo)
            self._btn_combos[btn_id] = combo

        # ── Axes ──────────────────────────────────────────────────
        all_axes = list(AXIS_LABELS.keys())
        self.axis_table.setRowCount(len(all_axes))
        self._axis_combos = {}

        for row, axis_id in enumerate(all_axes):
            id_item = QTableWidgetItem(axis_id)
            id_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.axis_table.setItem(row, 0, id_item)

            label = AXIS_LABELS[axis_id]
            label_item = QTableWidgetItem(label)
            label_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.axis_table.setItem(row, 1, label_item)

            current = axes.get(axis_id, "None")
            combo = self._make_command_combo(current)
            self.axis_table.setCellWidget(row, 2, combo)
            self._axis_combos[axis_id] = combo

        # ── D-Pad ─────────────────────────────────────────────────
        all_dpad = list(DPAD_LABELS.keys())
        self.dpad_table.setRowCount(len(all_dpad))
        self._dpad_combos = {}

        for row, direction in enumerate(all_dpad):
            dir_item = QTableWidgetItem(direction)
            dir_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.dpad_table.setItem(row, 0, dir_item)

            label = DPAD_LABELS[direction]
            label_item = QTableWidgetItem(label)
            label_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.dpad_table.setItem(row, 1, label_item)

            current = dpad.get(direction, "None")
            combo = self._make_command_combo(current)
            self.dpad_table.setCellWidget(row, 2, combo)
            self._dpad_combos[direction] = combo

    def _collect_mapping(self) -> dict:
        """Read current table state into a mapping dict."""
        buttons = {}
        for btn_id, combo in self._btn_combos.items():
            buttons[btn_id] = combo.currentText()

        axes = {}
        for axis_id, combo in self._axis_combos.items():
            axes[axis_id] = combo.currentText()

        dpad = {}
        for direction, combo in self._dpad_combos.items():
            dpad[direction] = combo.currentText()

        return {"buttons": buttons, "axes": axes, "dpad": dpad}

    def _save_mapping(self):
        """Save the current mapping to file."""
        mapping = self._collect_mapping()
        try:
            with open(self.mapping_file, "w") as f:
                json.dump(mapping, f, indent=4)
            logger.info(f"Mapping saved to {self.mapping_file}")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save: {e}")

    def _reset_defaults(self):
        """Reset to the default mapping."""
        self.mapping = {
            "buttons": {
                "0": "zero_needle_pos",
                "1": "None", "2": "None", "3": "None",
                "4": "increment_zspeed_down",
                "5": "increment_zspeed_up",
                "6": "increment_pspeed_down",
                "7": "increment_pspeed_up",
                "8": "increment_xyspeed_down",
                "9": "increment_xyspeed_up",
                "10": "None", "11": "None",
            },
            "axes": {
                "0-1": "move_stage_at_velocity",
                "2-3": "move_z_at_velocity",
                "4": "move_p3_at_velocity",
                "5": "move_p3_at_velocity",
            },
            "dpad": {
                "up": "increment_zspeed_up",
                "down": "increment_zspeed_down",
                "left": "increment_pspeed_down",
                "right": "increment_pspeed_up",
            },
        }
        self._populate_tables()

    def _export_mapping(self):
        """Export current mapping to a file."""
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Mapping", "", "JSON (*.json);;All (*)"
        )
        if filepath:
            mapping = self._collect_mapping()
            with open(filepath, "w") as f:
                json.dump(mapping, f, indent=4)

    def _import_mapping(self):
        """Import mapping from a file."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Import Mapping", "", "JSON (*.json);;All (*)"
        )
        if filepath:
            try:
                with open(filepath, "r") as f:
                    self.mapping = json.load(f)
                self._populate_tables()
            except Exception as e:
                QMessageBox.critical(self, "Import Error", f"Failed to import: {e}")
