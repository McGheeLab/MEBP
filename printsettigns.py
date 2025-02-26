import sys, json, os, random
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QTabWidget,
    QDoubleSpinBox, QSpinBox, QLineEdit, QGroupBox, QCheckBox, QPushButton,
    QFileDialog, QMessageBox, QComboBox, QScrollArea, QListWidget, QListWidgetItem
)
from PySide6.QtGui import QPainter, QColor,QRegularExpressionValidator, QValidator, QDrag
from PySide6.QtCore import QRegularExpression, Qt


# --- Custom Validator for Ink Well Location ---
class InkWellLocationValidator(QValidator):
    def __init__(self, max_rows, max_cols, parent=None):
        super().__init__(parent)
        self.max_rows = max_rows
        self.max_cols = max_cols

    def setLimits(self, max_rows, max_cols):
        self.max_rows = max_rows
        self.max_cols = max_cols

    def validate(self, input_str, pos):
        if not input_str:
            return (QValidator.Intermediate, input_str, pos)
        if not input_str[0].isalpha():
            return (QValidator.Invalid, input_str, pos)
        letter = input_str[0].upper()
        max_letter = chr(ord('A') + self.max_rows - 1)
        if letter < 'A' or letter > max_letter:
            return (QValidator.Invalid, input_str, pos)
        digits = input_str[1:]
        if not digits:
            return (QValidator.Intermediate, input_str, pos)
        if not digits.isdigit():
            return (QValidator.Invalid, input_str, pos)
        num = int(digits)
        if num < 1 or num > self.max_cols:
            return (QValidator.Invalid, input_str, pos)
        return (QValidator.Acceptable, input_str, pos)

# --- CSV List Widget (supports drag) ---
class CSVListWidget(QListWidget):
    def startDrag(self, supportedActions):
        item = self.currentItem()
        if item:
            data = item.data(Qt.UserRole)
            mimeData = self.model().mimeData(self.selectedIndexes())
            if data:
                file_path, color = data
                mimeData.setText(f"{file_path};{color}")
            drag = QDrag(self)
            drag.setMimeData(mimeData)
            drag.exec(supportedActions)

# --- Plate Layout Widget ---
class PlateLayoutWidget(QWidget):
    def __init__(self, main_widget, parent=None):
        super().__init__(parent)
        self.main_widget = main_widget  # reference to the main PrintSettingsWidget
        self.associations = {}  # mapping: well location (e.g. "A1") -> {"file":..., "color":...}
        self.setAcceptDrops(True)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        width = self.width()
        height = self.height()
        margin = 20
        available_width = width - 2*margin
        available_height = height - 2*margin

        # Get rows and columns from well properties
        rows = self.main_widget.well_rows_input.value()
        cols = self.main_widget.well_cols_input.value()
        if cols <= 0 or rows <= 0:
            return

        cell_width = available_width / cols
        cell_height = available_height / rows
        radius = min(cell_width, cell_height) * 0.4

        for r in range(rows):
            for c in range(cols):
                center_x = margin + (c + 0.5) * cell_width
                center_y = margin + (r + 0.5) * cell_height
                well_loc = f"{chr(ord('A')+r)}{c+1}"
                fill_color = None
                # Priority 1: CSV association
                if well_loc in self.associations:
                    fill_color = self.associations[well_loc]["color"]
                else:
                    # Priority 2: Ink well assignment
                    for ink_widget in self.main_widget.ink_well_widgets:
                        if ink_widget["location"].text().strip().upper() == well_loc.upper():
                            # Use the ink's color from its color field.
                            fill_color = ink_widget["color"].text().strip()
                            break
                if not fill_color:
                    fill_color = "#FFFFFF"  # default white
                painter.setPen(QColor("black"))
                painter.setBrush(QColor(fill_color))
                painter.drawEllipse(int(center_x - radius), int(center_y - radius), int(2*radius), int(2*radius))
                painter.drawText(int(center_x - 10), int(center_y + 5), well_loc)

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event):
        event.acceptProposedAction()

    def dropEvent(self, event):
        text = event.mimeData().text()
        parts = text.split(";")
        if len(parts) < 2:
            event.ignore()
            return
        file_path, file_color = parts[0], parts[1]
        pos = event.position() if hasattr(event, "position") else event.posF()
        rows = self.main_widget.well_rows_input.value()
        cols = self.main_widget.well_cols_input.value()
        margin = 20
        available_width = self.width() - 2*margin
        available_height = self.height() - 2*margin
        cell_width = available_width / cols
        cell_height = available_height / rows
        col = int((pos.x() - margin) / cell_width)
        row = int((pos.y() - margin) / cell_height)
        if 0 <= row < rows and 0 <= col < cols:
            well_loc = f"{chr(ord('A')+row)}{col+1}"
            self.associations[well_loc] = {"file": file_path, "color": file_color}
            self.update()
            event.acceptProposedAction()
        else:
            event.ignore()

    def update_layout(self):
        self.update()

# --- Plate Layout Tab (container for CSV list and plate layout view) ---
class PlateLayoutTab(QWidget):
    def __init__(self, main_widget, parent=None):
        super().__init__(parent)
        self.main_widget = main_widget
        layout = QHBoxLayout(self)
        # Left side: CSV file list and buttons.
        left_layout = QVBoxLayout()
        self.csv_list = CSVListWidget()
        self.csv_list.setDragEnabled(True)
        left_layout.addWidget(self.csv_list)
        btn_layout = QHBoxLayout()
        add_csv_btn = QPushButton("Add CSV Files")
        add_csv_btn.clicked.connect(self.add_csv_files)
        btn_layout.addWidget(add_csv_btn)
        remove_csv_btn = QPushButton("Remove Selected")
        remove_csv_btn.clicked.connect(self.remove_selected_csv)
        btn_layout.addWidget(remove_csv_btn)
        left_layout.addLayout(btn_layout)
        layout.addLayout(left_layout, 1)
        # Right side: PlateLayoutWidget.
        self.plate_layout_widget = PlateLayoutWidget(main_widget)
        layout.addWidget(self.plate_layout_widget, 3)

    def add_csv_files(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select CSV Files", "", "CSV Files (*.csv)")
        if files:
            for file in files:
                color = '#%02X%02X%02X' % (random.randint(0,255), random.randint(0,255), random.randint(0,255))
                item = QListWidgetItem(os.path.basename(file))
                item.setBackground(QColor(color))
                item.setData(Qt.UserRole, (file, color))
                self.csv_list.addItem(item)

    def remove_selected_csv(self):
        for item in self.csv_list.selectedItems():
            self.csv_list.takeItem(self.csv_list.row(item))

# --- Main PrintSettingsWidget ---
class PrintSettingsWidget(QWidget):
    def __init__(self, print_manager, parent=None):
        super().__init__(parent)
        self.print_manager = print_manager
        self.setWindowTitle("Print Settings")
        self.ink_well_widgets = []  # will be created dynamically
        self.standard_syringe_types = {}  # will be loaded from "Syringes.json"
        self.initUI()
        self.load_standard_syringe_types_from_file()
        self.update_all_standard_type_dropdowns()
        self.update_tab_validity()

    def initUI(self):
        main_layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget(self)

        # Tab 0: Well Properties
        self.well_tab = QWidget()
        well_layout = QVBoxLayout(self.well_tab)
        well_btn_layout = QHBoxLayout()
        load_well_btn = QPushButton("Load Well Properties")
        load_well_btn.clicked.connect(self.load_well_properties)
        well_btn_layout.addWidget(load_well_btn)
        save_well_btn = QPushButton("Save Well Properties")
        save_well_btn.clicked.connect(self.save_well_properties)
        well_btn_layout.addWidget(save_well_btn)
        well_layout.addLayout(well_btn_layout)
        self.well_form = QFormLayout()
        self.fastz_input = QDoubleSpinBox()
        self.fastz_input.setValue(self.print_manager.well_properties.get("fastz", 0))
        self.fastz_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Fast Z:", self.fastz_input)
        self.floorz_input = QDoubleSpinBox()
        self.floorz_input.setValue(self.print_manager.well_properties.get("floorz", 0))
        self.floorz_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Floor Z:", self.floorz_input)
        self.topz_input = QDoubleSpinBox()
        self.topz_input.setValue(self.print_manager.well_properties.get("topz", 0))
        self.topz_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Top Z:", self.topz_input)
        self.ink_topz_input = QDoubleSpinBox()
        self.ink_topz_input.setValue(self.print_manager.well_properties.get("ink_topz", 0))
        self.ink_topz_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Ink Top Z:", self.ink_topz_input)
        self.ink_floorz_input = QDoubleSpinBox()
        self.ink_floorz_input.setValue(self.print_manager.well_properties.get("ink_floorz", 0))
        self.ink_floorz_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Ink Floor Z:", self.ink_floorz_input)
        self.well_A1_x_input = QDoubleSpinBox()
        self.well_A1_x_input.setValue(self.print_manager.well_properties.get("Well_A1_x", 0))
        self.well_A1_x_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Well A1 X:", self.well_A1_x_input)
        self.well_A1_y_input = QDoubleSpinBox()
        self.well_A1_y_input.setValue(self.print_manager.well_properties.get("Well_A1_y", 0))
        self.well_A1_y_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Well A1 Y:", self.well_A1_y_input)
        self.well_dx_input = QDoubleSpinBox()
        self.well_dx_input.setValue(self.print_manager.well_properties.get("well_dx", 0))
        self.well_dx_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Well DX:", self.well_dx_input)
        self.well_dy_input = QDoubleSpinBox()
        self.well_dy_input.setValue(self.print_manager.well_properties.get("well_dy", 0))
        self.well_dy_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Well DY:", self.well_dy_input)
        self.well_rows_input = QSpinBox()
        self.well_rows_input.setValue(self.print_manager.well_properties.get("well_rows", 0))
        self.well_rows_input.valueChanged.connect(lambda: (self.update_ink_well_validators(), self.update_tab_validity()))
        self.well_form.addRow("Well Rows:", self.well_rows_input)
        self.well_cols_input = QSpinBox()
        self.well_cols_input.setValue(self.print_manager.well_properties.get("well_cols", 0))
        self.well_cols_input.valueChanged.connect(lambda: (self.update_ink_well_validators(), self.update_tab_validity()))
        self.well_form.addRow("Well Cols:", self.well_cols_input)
        self.well_diameter_input = QDoubleSpinBox()
        self.well_diameter_input.setValue(self.print_manager.well_properties.get("well_diameter", 0))
        self.well_diameter_input.valueChanged.connect(self.update_tab_validity)
        self.well_form.addRow("Well Diameter:", self.well_diameter_input)
        well_layout.addLayout(self.well_form)
        self.tab_widget.addTab(self.well_tab, "Well Properties")

        # Tab 1: Ink Well Properties
        self.ink_tab = QWidget()
        ink_layout = QVBoxLayout(self.ink_tab)
        ink_btn_layout = QHBoxLayout()
        add_ink_btn = QPushButton("Add Ink Well")
        add_ink_btn.clicked.connect(self.add_ink_well_widget)
        ink_btn_layout.addWidget(add_ink_btn)
        ink_layout.addLayout(ink_btn_layout)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        container = QWidget()
        self.ink_well_layout = QVBoxLayout(container)
        container.setLayout(self.ink_well_layout)
        scroll_area.setWidget(container)
        ink_layout.addWidget(scroll_area)
        self.tab_widget.addTab(self.ink_tab, "Ink Well Properties")

        # Tab 2: Syringe Properties
        self.syringe_tab = QWidget()
        syringe_layout = QVBoxLayout(self.syringe_tab)
        syringe_btn_layout = QHBoxLayout()
        load_syringe_btn = QPushButton("Load Syringe Layout")
        load_syringe_btn.clicked.connect(self.load_syringe_layout)
        syringe_btn_layout.addWidget(load_syringe_btn)
        save_syringe_btn = QPushButton("Save Syringe Layout")
        save_syringe_btn.clicked.connect(self.save_syringe_layout)
        syringe_btn_layout.addWidget(save_syringe_btn)
        syringe_layout.addLayout(syringe_btn_layout)
        self.syringe_form = QFormLayout()
        self.syringe_inputs = {}
        for pump in ["p1", "p2", "p3"]:
            group_box = QGroupBox(f"Syringe {pump.upper()}")
            group_layout = QFormLayout(group_box)
            std_type_combo = QComboBox()
            std_type_combo.addItem("None")
            for key in sorted(self.standard_syringe_types.keys()):
                std_type_combo.addItem(key)
            std_type_combo.currentIndexChanged.connect(lambda idx, p=pump, combo=std_type_combo: self.standard_type_changed(p, combo))
            group_layout.addRow("Standard Type:", std_type_combo)
            diameter_spin = QDoubleSpinBox()
            diameter_spin.setValue(getattr(self.print_manager.syringes[pump], "diameter") or 0)
            diameter_spin.valueChanged.connect(self.update_tab_validity)
            group_layout.addRow("Diameter:", diameter_spin)
            length_spin = QDoubleSpinBox()
            length_spin.setValue(getattr(self.print_manager.syringes[pump], "length") or 0)
            length_spin.valueChanged.connect(self.update_tab_validity)
            group_layout.addRow("Length:", length_spin)
            cell_type_combo = QComboBox()
            self.update_syringe_cell_type_options_for(pump, cell_type_combo)
            cell_type_combo.currentIndexChanged.connect(self.update_tab_validity)
            group_layout.addRow("Cell Type:", cell_type_combo)
            self.syringe_inputs[pump] = {
                "std_type": std_type_combo,
                "diameter": diameter_spin,
                "length": length_spin,
                "cell_type": cell_type_combo
            }
            self.syringe_form.addRow(group_box)
        syringe_layout.addLayout(self.syringe_form)
        self.tab_widget.addTab(self.syringe_tab, "Syringe Properties")

        # Tab 3: Plate Layout
        self.plate_layout_tab = PlateLayoutTab(self)
        self.tab_widget.addTab(self.plate_layout_tab, "Plate Layout")

        main_layout.addWidget(self.tab_widget)
        all_settings_layout = QHBoxLayout()
        load_all_btn = QPushButton("Load All Settings")
        load_all_btn.clicked.connect(self.load_all_settings)
        all_settings_layout.addWidget(load_all_btn)
        save_all_btn = QPushButton("Save All Settings")
        save_all_btn.clicked.connect(self.save_all_settings)
        all_settings_layout.addWidget(save_all_btn)
        main_layout.addLayout(all_settings_layout)
        apply_button = QPushButton("Apply Settings")
        apply_button.clicked.connect(self.apply_settings)
        main_layout.addWidget(apply_button)

    # ----- Ink Well Methods (including added Color field) -----
    def add_ink_well_widget(self):
        group_box = QGroupBox(f"Ink Well {len(self.ink_well_widgets) + 1}")
        layout = QFormLayout(group_box)
        location_edit = QLineEdit()
        validator = InkWellLocationValidator(self.well_rows_input.value(), self.well_cols_input.value())
        location_edit.setValidator(validator)
        location_edit.textChanged.connect(self.update_tab_validity)
        layout.addRow("Location:", location_edit)
        cell_type_edit = QLineEdit("DefaultCell")
        cell_type_edit.textChanged.connect(lambda: (self.update_all_syringe_cell_type_options(), self.update_tab_validity()))
        layout.addRow("Cell Type:", cell_type_edit)
        chilled_check = QCheckBox()
        chilled_check.setChecked(True)
        layout.addRow("Chilled:", chilled_check)
        volume_spin = QDoubleSpinBox()
        volume_spin.setValue(0)
        volume_spin.valueChanged.connect(self.update_tab_validity)
        layout.addRow("Volume:", volume_spin)
        color_edit = QLineEdit("#CCCCCC")
        color_edit.textChanged.connect(self.update_tab_validity)
        layout.addRow("Color:", color_edit)
        del_btn = QPushButton("Delete")
        del_btn.clicked.connect(lambda: self.remove_ink_well_widget(group_box))
        layout.addRow(del_btn)
        ink_widget = {
            "group": group_box,
            "location": location_edit,
            "cell_type": cell_type_edit,
            "chilled": chilled_check,
            "volume": volume_spin,
            "color": color_edit
        }
        self.ink_well_widgets.append(ink_widget)
        self.ink_well_layout.addWidget(group_box)
        self.update_all_syringe_cell_type_options()
        self.update_tab_validity()

    def remove_ink_well_widget(self, widget):
        for ink_widget in self.ink_well_widgets:
            if ink_widget["group"] == widget:
                self.ink_well_widgets.remove(ink_widget)
                break
        widget.setParent(None)
        widget.deleteLater()
        self.update_all_syringe_cell_type_options()
        self.update_tab_validity()

    # ----- Standard Syringe Types (from file "Syringes.json") -----
    def load_standard_syringe_types_from_file(self):
        try:
            if os.path.exists("Syringes.json"):
                with open("Syringes.json", "r") as f:
                    self.standard_syringe_types = json.load(f)
            else:
                self.standard_syringe_types = {}
        except Exception as e:
            QMessageBox.warning(self, "Standard Types Load Error", f"Error loading Syringes.json: {e}")
            self.standard_syringe_types = {}

    def update_all_standard_type_dropdowns(self):
        for pump, inputs in self.syringe_inputs.items():
            std_combo = inputs["std_type"]
            current = std_combo.currentText()
            std_combo.clear()
            std_combo.addItem("None")
            for key in sorted(self.standard_syringe_types.keys()):
                std_combo.addItem(key)
            index = std_combo.findText(current)
            if index < 0:
                std_combo.setCurrentIndex(0)
            else:
                std_combo.setCurrentIndex(index)

    # ----- Update Ink Well Validators -----
    def update_ink_well_validators(self):
        max_rows = self.well_rows_input.value()
        max_cols = self.well_cols_input.value()
        for widget in self.ink_well_widgets:
            validator = widget["location"].validator()
            if isinstance(validator, InkWellLocationValidator):
                validator.setLimits(max_rows, max_cols)
                widget["location"].setValidator(validator)

    # ----- Syringe Cell Type Options -----
    def update_syringe_cell_type_options_for(self, pump, combo_box):
        valid_types = { widget["cell_type"].text().strip() for widget in self.ink_well_widgets if widget["cell_type"].text().strip() }
        if not valid_types:
            valid_types.add("None")
        items = sorted(valid_types)
        current = combo_box.currentText().strip()
        combo_box.clear()
        for item in items:
            combo_box.addItem(item)
        index = combo_box.findText(current)
        if index < 0:
            index = combo_box.findText("None")
            if index < 0:
                index = 0
        combo_box.setCurrentIndex(index)

    def update_all_syringe_cell_type_options(self):
        for pump, inputs in self.syringe_inputs.items():
            self.update_syringe_cell_type_options_for(pump, inputs["cell_type"])

    # ----- Standard Type Change -----
    def standard_type_changed(self, pump, combo_box):
        std_type = combo_box.currentText()
        if std_type in self.standard_syringe_types:
            values = self.standard_syringe_types[std_type]
            self.syringe_inputs[pump]["diameter"].setValue(values.get("diameter", 0))
            self.syringe_inputs[pump]["length"].setValue(values.get("length", 0))
        self.update_tab_validity()

    # ----- Tab Validity and Appearance -----
    def update_tab_validity(self):
        well_valid, well_msg = self.validate_well_tab()
        ink_valid, ink_msg = self.validate_ink_tab()
        syringe_valid, syringe_msg = self.validate_syringe_tab()
        tab_bar = self.tab_widget.tabBar()
        tab_bar.setTabTextColor(0, QColor("red") if not well_valid else QColor("black"))
        tab_bar.setTabTextColor(1, QColor("red") if not ink_valid else QColor("black"))
        tab_bar.setTabTextColor(2, QColor("red") if not syringe_valid else QColor("black"))
        tab_bar.setTabTextColor(3, QColor("black"))  # Plate Layout tab need not be validated
        overall_valid = well_valid and ink_valid and syringe_valid
        return overall_valid, well_msg or ink_msg or syringe_msg

    def validate_well_tab(self):
        if self.well_rows_input.value() <= 0 or self.well_cols_input.value() <= 0:
            return False, "Well Rows and Well Cols must be greater than 0."
        return True, ""

    def validate_ink_tab(self):
        if len(self.ink_well_widgets) == 0:
            return True, ""
        for i, widget in enumerate(self.ink_well_widgets, start=1):
            text = widget["location"].text()
            validator = widget["location"].validator()
            state, _, _ = validator.validate(text, 0)
            if state != QValidator.Acceptable:
                return False, f"Ink well {i} location is invalid."
            if not widget["cell_type"].text().strip():
                return False, f"Ink well {i} cell type must not be empty."
            if not widget["color"].text().strip():
                return False, f"Ink well {i} must have a color code."
        return True, ""

    def validate_syringe_tab(self):
        if len(self.ink_well_widgets) == 0:
            valid_cell_types = {"None"}
        else:
            valid_cell_types = { widget["cell_type"].text().strip() for widget in self.ink_well_widgets if widget["cell_type"].text().strip() }
        non_none_exists = False
        for pump, inputs in self.syringe_inputs.items():
            cell_type = inputs["cell_type"].currentText().strip()
            if cell_type not in valid_cell_types:
                return False, f"Syringe {pump.upper()} cell type '{cell_type}' does not match any ink well cell type."
            if cell_type != "None":
                non_none_exists = True
                if inputs["diameter"].value() <= 0 or inputs["length"].value() <= 0:
                    return False, f"Syringe {pump.upper()} must have positive diameter and length if a cell type is selected."
        if not non_none_exists:
            return False, "At least one syringe must have a cell type other than 'None'."
        return True, ""

    # ----- Syringe Layout Load/Save -----
    def load_syringe_layout(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Syringe Layout", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                for pump in ["p1", "p2", "p3"]:
                    if pump in data:
                        pump_data = data[pump]
                        self.syringe_inputs[pump]["diameter"].setValue(pump_data.get("diameter", 0))
                        self.syringe_inputs[pump]["length"].setValue(pump_data.get("length", 0))
                        self.syringe_inputs[pump]["cell_type"].setCurrentText(pump_data.get("cell_type", "None"))
                        self.syringe_inputs[pump]["std_type"].setCurrentText(pump_data.get("standard_type", "None"))
                QMessageBox.information(self, "Load", "Syringe layout loaded successfully.")
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading JSON: {e}")

    def save_syringe_layout(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Syringe Layout", "", "JSON Files (*.json)")
        if filename:
            try:
                data = {}
                for pump in ["p1", "p2", "p3"]:
                    data[pump] = {
                        "diameter": self.syringe_inputs[pump]["diameter"].value(),
                        "length": self.syringe_inputs[pump]["length"].value(),
                        "cell_type": self.syringe_inputs[pump]["cell_type"].currentText(),
                        "standard_type": self.syringe_inputs[pump]["std_type"].currentText()
                    }
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=4)
                QMessageBox.information(self, "Save", "Syringe layout saved successfully.")
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving JSON: {e}")

    # ----- Well Properties Load/Save -----
    def load_well_properties(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Well Properties", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                self.fastz_input.setValue(float(data.get("fastz", 0)))
                self.floorz_input.setValue(float(data.get("floorz", 0)))
                self.topz_input.setValue(float(data.get("topz", 0)))
                self.ink_topz_input.setValue(float(data.get("ink_topz", 0)))
                self.ink_floorz_input.setValue(float(data.get("ink_floorz", 0)))
                self.well_A1_x_input.setValue(float(data.get("Well_A1_x", 0)))
                self.well_A1_y_input.setValue(float(data.get("Well_A1_y", 0)))
                self.well_dx_input.setValue(float(data.get("well_dx", 0)))
                self.well_dy_input.setValue(float(data.get("well_dy", 0)))
                self.well_rows_input.setValue(int(data.get("well_rows", 0)))
                self.well_cols_input.setValue(int(data.get("well_cols", 0)))
                self.well_diameter_input.setValue(float(data.get("well_diameter", 0)))
                QMessageBox.information(self, "Load", "Well properties loaded successfully.")
                self.update_ink_well_validators()
                self.update_tab_validity()
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading JSON: {e}")

    def save_well_properties(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Well Properties", "", "JSON Files (*.json)")
        if filename:
            try:
                data = {
                    "fastz": self.fastz_input.value(),
                    "floorz": self.floorz_input.value(),
                    "topz": self.topz_input.value(),
                    "ink_topz": self.ink_topz_input.value(),
                    "ink_floorz": self.ink_floorz_input.value(),
                    "Well_A1_x": self.well_A1_x_input.value(),
                    "Well_A1_y": self.well_A1_y_input.value(),
                    "well_dx": self.well_dx_input.value(),
                    "well_dy": self.well_dy_input.value(),
                    "well_rows": self.well_rows_input.value(),
                    "well_cols": self.well_cols_input.value(),
                    "well_diameter": self.well_diameter_input.value()
                }
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=4)
                QMessageBox.information(self, "Save", "Well properties saved successfully.")
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving JSON: {e}")

    # ----- All Settings Load/Save -----
    def load_all_settings(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load All Settings", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                wp = data.get("well_properties", {})
                self.fastz_input.setValue(float(wp.get("fastz", 0)))
                self.floorz_input.setValue(float(wp.get("floorz", 0)))
                self.topz_input.setValue(float(wp.get("topz", 0)))
                self.ink_topz_input.setValue(float(wp.get("ink_topz", 0)))
                self.ink_floorz_input.setValue(float(wp.get("ink_floorz", 0)))
                self.well_A1_x_input.setValue(float(wp.get("Well_A1_x", 0)))
                self.well_A1_y_input.setValue(float(wp.get("Well_A1_y", 0)))
                self.well_dx_input.setValue(float(wp.get("well_dx", 0)))
                self.well_dy_input.setValue(float(wp.get("well_dy", 0)))
                self.well_rows_input.setValue(int(wp.get("well_rows", 0)))
                self.well_cols_input.setValue(int(wp.get("well_cols", 0)))
                self.well_diameter_input.setValue(float(wp.get("well_diameter", 0)))
                syr = data.get("syringes", {})
                for pump in ["p1", "p2", "p3"]:
                    if pump in syr:
                        pump_data = syr[pump]
                        self.syringe_inputs[pump]["diameter"].setValue(pump_data.get("diameter", 0))
                        self.syringe_inputs[pump]["length"].setValue(pump_data.get("length", 0))
                        self.syringe_inputs[pump]["cell_type"].setCurrentText(pump_data.get("cell_type", "None"))
                        self.syringe_inputs[pump]["std_type"].setCurrentText(pump_data.get("standard_type", "None"))
                for widget in list(self.ink_well_widgets):
                    self.remove_ink_well_widget(widget["group"])
                ink_wells = data.get("ink_wells", {})
                for key in sorted(ink_wells.keys()):
                    ink_data = ink_wells[key]
                    self.add_ink_well_widget()
                    widget = self.ink_well_widgets[-1]
                    widget["location"].setText(ink_data.get("location", ""))
                    widget["cell_type"].setText(ink_data.get("cell_type", "DefaultCell"))
                    widget["chilled"].setChecked(ink_data.get("chilled", True))
                    widget["volume"].setValue(ink_data.get("volume", 0))
                    widget["color"].setText(ink_data.get("color", "#CCCCCC"))
                QMessageBox.information(self, "Load All", "All settings loaded successfully.")
                self.update_tab_validity()
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading all settings: {e}")

    def save_all_settings(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save All Settings", "", "JSON Files (*.json)")
        if filename:
            try:
                data = {}
                data["well_properties"] = {
                    "fastz": self.fastz_input.value(),
                    "floorz": self.floorz_input.value(),
                    "topz": self.topz_input.value(),
                    "ink_topz": self.ink_topz_input.value(),
                    "ink_floorz": self.ink_floorz_input.value(),
                    "Well_A1_x": self.well_A1_x_input.value(),
                    "Well_A1_y": self.well_A1_y_input.value(),
                    "well_dx": self.well_dx_input.value(),
                    "well_dy": self.well_dy_input.value(),
                    "well_rows": self.well_rows_input.value(),
                    "well_cols": self.well_cols_input.value(),
                    "well_diameter": self.well_diameter_input.value()
                }
                syr_data = {}
                for pump in ["p1", "p2", "p3"]:
                    syr_data[pump] = {
                        "diameter": self.syringe_inputs[pump]["diameter"].value(),
                        "length": self.syringe_inputs[pump]["length"].value(),
                        "cell_type": self.syringe_inputs[pump]["cell_type"].currentText(),
                        "standard_type": self.syringe_inputs[pump]["std_type"].currentText()
                    }
                data["syringes"] = syr_data
                ink_data = {}
                for i, widget in enumerate(self.ink_well_widgets, start=1):
                    ink_data[f"ink_{i}"] = {
                        "location": widget["location"].text(),
                        "cell_type": widget["cell_type"].text(),
                        "chilled": widget["chilled"].isChecked(),
                        "volume": widget["volume"].value(),
                        "color": widget["color"].text()
                    }
                data["ink_wells"] = ink_data
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=4)
                QMessageBox.information(self, "Save All", "All settings saved successfully.")
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving all settings: {e}")

    # ----- Apply Settings (with Comprehensive Error Checks) -----
    def apply_settings(self):
        errors = []
        valid, msg = self.validate_well_tab()
        if not valid:
            errors.append("Well Properties: " + msg)
        valid, msg = self.validate_ink_tab()
        if not valid:
            errors.append("Ink Well Properties: " + msg)
        valid, msg = self.validate_syringe_tab()
        if not valid:
            errors.append("Syringe Properties: " + msg)
        if errors:
            QMessageBox.warning(self, "Apply Error", "Cannot apply settings due to the following errors:\n" + "\n".join(errors))
            return
        wp = self.print_manager.well_properties
        wp["fastz"] = self.fastz_input.value()
        wp["floorz"] = self.floorz_input.value()
        wp["topz"] = self.topz_input.value()
        wp["ink_topz"] = self.ink_topz_input.value()
        wp["ink_floorz"] = self.ink_floorz_input.value()
        wp["Well_A1_x"] = self.well_A1_x_input.value()
        wp["Well_A1_y"] = self.well_A1_y_input.value()
        wp["well_dx"] = self.well_dx_input.value()
        wp["well_dy"] = self.well_dy_input.value()
        wp["well_rows"] = self.well_rows_input.value()
        wp["well_cols"] = self.well_cols_input.value()
        wp["well_diameter"] = self.well_diameter_input.value()
        for pump, inputs in self.syringe_inputs.items():
            diameter = inputs["diameter"].value()
            length = inputs["length"].value()
            cell_type = inputs["cell_type"].currentText()
            self.print_manager.syringes[pump].set_syringe_properties(diameter, length)
            self.print_manager.syringes[pump].assign_cell_type(cell_type)
        new_ink_wells = {}
        for i, widget in enumerate(self.ink_well_widgets, start=1):
            loc_text = widget["location"].text().strip()
            if not loc_text:
                QMessageBox.warning(self, "Validation Error", f"Ink well {i}: Location cannot be empty.")
                return
            ink_well = InkWell()
            ink_well.set_location(loc_text)
            ink_well.set_cell_type(widget["cell_type"].text().strip())
            ink_well.set_chilled(widget["chilled"].isChecked())
            ink_well.set_volume(widget["volume"].value())
            new_ink_wells[f"ink_{i}"] = ink_well
        self.print_manager.ink_wells = new_ink_wells
        self.update_all_syringe_cell_type_options()
        QMessageBox.information(self, "Apply Settings", "Settings applied successfully.")

# ----- Dummy Classes for Testing -----
class DummyPrintManager:
    def __init__(self):
        self.well_properties = {
            "fastz": 10.0, "floorz": 5.0, "topz": 20.0,
            "ink_topz": 15.0, "ink_floorz": 3.0,
            "Well_A1_x": 100.0, "Well_A1_y": 200.0,
            "well_dx": 10.0, "well_dy": 10.0,
            "well_rows": 8, "well_cols": 12,
            "well_diameter": 5.0
        }
        self.syringes = {"p1": Syringe("p1"), "p2": Syringe("p2"), "p3": Syringe("p3")}
        self.ink_wells = {}

class Syringe:
    def __init__(self, axis):
        self.axis = axis
        self.diameter = 0
        self.length = 0
        self.area = 0
        self.max_volume = 0
        self.current_volume = 0
        self.cell_type = None
        self.ink_volume = 0

    def set_syringe_properties(self, diameter, length):
        self.diameter = diameter
        self.length = length
        self.area = 3.1416 * (diameter / 2) ** 2
        self.max_volume = self.area * length

    def assign_cell_type(self, cell_type):
        self.cell_type = cell_type

class InkWell:
    def __init__(self):
        self.well_location = ""
        self.cell_type = ""
        self.chilled = True
        self.volume = 0

    def set_location(self, location):
        self.well_location = location

    def set_cell_type(self, cell_type):
        self.cell_type = cell_type

    def set_chilled(self, chilled):
        self.chilled = chilled

    def set_volume(self, volume):
        self.volume = volume

# ----- Main Execution -----
if __name__ == "__main__":
    app = QApplication(sys.argv)
    dummy_pm = DummyPrintManager()
    widget = PrintSettingsWidget(dummy_pm)
    widget.resize(900, 900)
    widget.show()
    sys.exit(app.exec())
