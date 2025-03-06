import sys, json, os, random, csv, math
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QTabWidget, QSplitter, QStyle, QStyleOptionViewItem,
    QDoubleSpinBox, QSpinBox, QLineEdit, QGroupBox, QCheckBox, QPushButton, QLabel, QStyledItemDelegate, 
    QFileDialog, QMessageBox, QComboBox, QScrollArea, QListWidget, QListWidgetItem, QColorDialog, QMenu, QAbstractItemView
)
from PySide6.QtGui import QPainter, QColor, QRegularExpressionValidator, QValidator, QDrag, QPen
from PySide6.QtCore import QRegularExpression, QRectF, QPointF, QSizeF, Qt, Signal, QSignalBlocker, QEvent, QMimeData

from SupportClasses.Printer import InkWell, Syringe, PrintFile

############################################################################################################
# improvements to make
# [x] [COMPLETE] the views of the waypoint files dont seem to reflect reality still the spiral.csv looks bigger than it is
# [x] the isoview should resize the view to fit the print
# [x] when the window changes, the drawn diameter of the well should update the scaling of the waypoint file 
# [ ] if we select multiple printfiles in the printsetup tab we want to see all the bounding boxes in the view colored to the printfile color
# [ ] multiple printfiles should be draggable to the plate layout tab simultaneouly
# [ ] the platelayout representation of the print should be taken directly from the printmanager so that if printfiles are deleted 
#    somewhere else it is reflected in the view 



def get_contrast_text_color(bg_color: str) -> str:
    """
    Returns either black (#000000) or white (#FFFFFF) depending on whether the background color
    is light or dark based on its perceived luminance.
    """
    color = QColor(bg_color)
    # Calculate luminance using standard coefficients.
    r, g, b = color.red(), color.green(), color.blue()
    luminance = 0.299 * r + 0.587 * g + 0.114 * b
    # If luminance is high, use black text; if low, use white text.
    return "#000000" if luminance > 128 else "#FFFFFF"

############################################################################################################
#####################################  Main tab window  ####################################################
############################################################################################################
class PrintSettingsWidget(QWidget):
    def __init__(self, print_manager, parent=None):
        super().__init__(parent)
        self.print_manager = print_manager
        self.setWindowTitle("Print Settings")
        self.initUI()
        self.update_tab_validity()

    def initUI(self):
        from PySide6.QtWidgets import QTabWidget, QHBoxLayout, QVBoxLayout, QPushButton  # local import for clarity
        main_layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget(self)

        # Create separate tabs.
        self.well_tab = WellPropertiesTab(self, self.update_tab_validity)
        self.tab_widget.addTab(self.well_tab, "Well Properties")

        self.ink_tab = InkPropertiesTab(self, self.update_tab_validity)
        self.tab_widget.addTab(self.ink_tab, "Ink Well Properties")

        self.syringe_tab = SyringePropertiesTab(self, self.update_tab_validity)
        self.tab_widget.addTab(self.syringe_tab, "Syringe Properties")
        
        self.print_setup_tab = PrintSetupTab(self)
        self.tab_widget.addTab(self.print_setup_tab, "Print Setup")
        
        self.plate_layout_tab = PlateLayoutTab(self)
        self.tab_widget.addTab(self.plate_layout_tab, "Plate Layout")

        main_layout.addWidget(self.tab_widget)

        # Bottom controls (Load All, Save All, Apply Settings, etc.)
        controls_layout = QHBoxLayout()
        load_all_btn = QPushButton("Load All Settings")
        load_all_btn.clicked.connect(self.load_all_settings)
        controls_layout.addWidget(load_all_btn)
        save_all_btn = QPushButton("Save All Settings")
        save_all_btn.clicked.connect(self.save_all_settings)
        controls_layout.addWidget(save_all_btn)
        main_layout.addLayout(controls_layout)
        apply_button = QPushButton("Apply Settings")
        apply_button.clicked.connect(self.apply_settings)
        main_layout.addWidget(apply_button)

    def update_tab_validity(self):
        well_valid, well_msg = self.well_tab.validate()
        ink_valid, ink_msg = self.ink_tab.validate()
        syringe_valid, syringe_msg = self.syringe_tab.validate()
        self.syringe_tab.update_all_cell_type_options()

        # Validate PrintSetupTab.
        self.print_setup_tab.update_from_well_properties()
        setup_valid, setup_msg = self.print_setup_tab.validate()
        
        # Update tab text colors for feedback.
        tab_bar = self.tab_widget.tabBar()
        tab_bar.setTabTextColor(0, QColor("red") if not well_valid else QColor("black"))
        tab_bar.setTabTextColor(1, QColor("red") if not ink_valid else QColor("black"))
        tab_bar.setTabTextColor(2, QColor("red") if not syringe_valid else QColor("black"))
        tab_bar.setTabTextColor(3, QColor("red") if not setup_valid else QColor("black"))
    
        overall_valid = well_valid and ink_valid and syringe_valid and setup_valid
        return overall_valid, well_msg or ink_msg or syringe_msg or setup_msg

    def load_all_settings(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load All Settings", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, "r") as f:
                    data = json.load(f)
                # --- Load well plate properties ---
                wp = data.get("well_properties", {})
                self.well_tab.fastz_input.setValue(float(wp.get("fastz", 0)))
                self.well_tab.floorz_input.setValue(float(wp.get("floorz", 0)))
                self.well_tab.topz_input.setValue(float(wp.get("topz", 0)))
                self.well_tab.ink_topz_input.setValue(float(wp.get("ink_topz", 0)))
                self.well_tab.ink_floorz_input.setValue(float(wp.get("ink_floorz", 0)))
                self.well_tab.well_A1_x_input.setValue(float(wp.get("Well_A1_x", 0)))
                self.well_tab.well_A1_y_input.setValue(float(wp.get("Well_A1_y", 0)))
                self.well_tab.well_dx_input.setValue(float(wp.get("well_dx", 0)))
                self.well_tab.well_dy_input.setValue(float(wp.get("well_dy", 0)))
                self.well_tab.well_rows_input.setValue(int(wp.get("well_rows", 0)))
                self.well_tab.well_cols_input.setValue(int(wp.get("well_cols", 0)))
                self.well_tab.well_diameter_input.setValue(float(wp.get("well_diameter", 0)))
                plate_mode = wp.get("plate_mode", "Well Plate")
                index = self.well_tab.plate_type_combo.findText(plate_mode)
                if index >= 0:
                    self.well_tab.plate_type_combo.setCurrentIndex(index)
                
                # --- Load ink well settings ---
                inkwells = data.get("ink_wells", {})
                # Clear existing ink well widgets
                for widget in self.ink_tab.ink_well_widgets:
                    widget["group"].setParent(None)
                    widget["group"].deleteLater()
                self.ink_tab.ink_well_widgets.clear()
                # For each saved ink well (assumed keys like "ink_1", "ink_2", etc.)
                for key in sorted(inkwells.keys()):
                    inkwell_data = inkwells[key]
                    self.ink_tab.add_ink_well_widget()
                    widget = self.ink_tab.ink_well_widgets[-1]
                    widget["location"].setText(inkwell_data.get("location", ""))
                    widget["cell_type"].setText(inkwell_data.get("cell_type", ""))
                    widget["chilled"].setChecked(inkwell_data.get("chilled", True))
                    widget["volume"].setValue(float(inkwell_data.get("volume", 0)))
                    color = inkwell_data.get("color", "#868eff")
                    widget["color"].setText(color)
                    widget["color"].setStyleSheet(f"background-color: {color}")
                    # Update the InkWell object accordingly
                    widget["inkwell"].set_location(inkwell_data.get("location", ""))
                    widget["inkwell"].set_cell_type(inkwell_data.get("cell_type", ""))
                    widget["inkwell"].set_chilled(inkwell_data.get("chilled", True))
                    widget["inkwell"].set_volume(float(inkwell_data.get("volume", 0)))
                    widget["inkwell"].set_color(color)
                
                # --- Load syringe settings ---
                syringes = data.get("syringes", {})
                for pump in ["p1", "p2", "p3"]:
                    if pump in syringes:
                        pump_data = syringes[pump]
                        self.syringe_tab.syringe_inputs[pump]["diameter"].setValue(float(pump_data.get("diameter", 0)))
                        self.syringe_tab.syringe_inputs[pump]["length"].setValue(float(pump_data.get("length", 0)))
                        self.syringe_tab.syringe_inputs[pump]["cell_type"].setCurrentText(pump_data.get("cell_type", "None"))
                        self.syringe_tab.syringe_inputs[pump]["std_type"].setCurrentText(pump_data.get("standard_type", "None"))
                
                
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading all settings: {e}")
            self.update_tab_validity()
    
    def save_all_settings(self):
        # Only collect well plate, ink well, and syringe information.
        data = {
            "well_properties": self.well_tab.get_properties(),
            "ink_wells": self.ink_tab.get_ink_wells_json(),
            "syringes": self.syringe_tab.get_syringe_data()
        }
        filename, _ = QFileDialog.getSaveFileName(self, "Save All Settings", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=4)
                
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving all settings: {e}")

    def apply_settings(self):
        valid, msg = self.update_tab_validity()
        if not valid:
            QMessageBox.warning(self, "Apply Error", f"Cannot apply settings: {msg}")
            return

        # Update well properties.
        wp = self.well_tab.get_properties()
        self.print_manager.well_properties.update(wp)

        # Update syringe properties.
        syr_data = self.syringe_tab.get_syringe_data()
        for pump, vals in syr_data.items():
            self.print_manager.syringes[pump].set_syringe_properties(vals["diameter"], vals["length"])
            self.print_manager.syringes[pump].assign_cell_type(vals["cell_type"])

        # Update ink wells.
        ink_data = self.ink_tab.get_ink_wells()
        self.print_manager.ink_wells.update(ink_data)

        # get the print setup data which is a dict of well location and printfile object
        pf = self.plate_layout_tab.get_assigned_printfiles()
        # go through pf and queue the printfiles in the printmanager.queue_a_waypoint(self, well_id, offset, csv_file, **kwargs):
        # the printfile object has the csv file path, color, and offset
        # there could be multiple printfiles for a single pf.tiem
        self.print_manager.well_queue.clear()
        for well_id, printfiles in pf.items():
            for printfile in printfiles:
                self.print_manager.queue_a_printfile(well_id, printfile)
        
            
############################################################################################################
###################################  All Widgets within the tabs  ##########################################
############################################################################################################
# --- CSV List Widget (supports drag) ---
from PySide6.QtWidgets import QListWidget, QAbstractItemView
from PySide6.QtCore import Qt, QMimeData
from PySide6.QtGui import QDrag
import json

class PrintListWidget(QListWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        # Allow multi-selection.
        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        # Attribute to store selection when shift is held.
        self._multi_drag_selection = None

    def mousePressEvent(self, event):
        # If Shift is held, store the current selection.
        if event.modifiers() & Qt.ShiftModifier:
            self._multi_drag_selection = self.selectedItems()
        else:
            self._multi_drag_selection = None
        # Call the base method to handle normal selection behavior.
        super().mousePressEvent(event)

    def startDrag(self, supportedActions):
        # Use the stored selection if available; otherwise, use the current selection.
        selected_items = self._multi_drag_selection if self._multi_drag_selection else self.selectedItems()
        if not selected_items:
            return

        files_data = []
        for item in selected_items:
            # Assuming each item's UserRole data is a tuple: (uid, file_path, file_color, file_name)
            data = item.data(Qt.UserRole)
            if data:
                uid, file_path, file_color, file_name = data
                files_data.append({
                    "uid": uid,
                    "file_path": file_path,
                    "file_color": file_color,
                    "file_name": file_name,
                })
        mimeData = QMimeData()
        # Package the data as JSON under our custom MIME type.
        mimeData.setData("application/json", json.dumps(files_data).encode("utf-8"))
        drag = QDrag(self)
        drag.setMimeData(mimeData)
        drag.exec(supportedActions)
        # Clear the stored selection.
        self._multi_drag_selection = None


# --- PlateLayoutWidget ---
class PlateLayoutWidget(QWidget):
    # Signal emitted whenever assignments change.
    associationsChanged = Signal()

    def __init__(self, main_widget, parent=None):
        """
        main_widget should contain:
          - well_tab with well_rows_input, well_cols_input, and plate_mode (from the new Plate Type combobox)
          - print_setup_tab (used by PlateLayoutTab to get print files)
        """
        super().__init__(parent)
        self.main_widget = main_widget
        # associations: mapping well location (e.g. "A1" for petridish or "A2" ... "D2" for small wells) -> { printfile_name: {"color": ...} }
        self.associations = {}
        # Set of well identifiers for multi-selection (if needed)
        self.selected_wells = set()
        # For drag handling.
        self.clicked_well = None
        self.drag_start_pos = None
        self.setAcceptDrops(True)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 20
        width = self.width()
        height = self.height()

        # Determine plate mode from well properties.
        props = self.main_widget.well_tab.get_properties()
        plate_mode = props.get("plate_mode", "Well Plate")

        if plate_mode == "Well Plate":
            # Use grid layout as before.
            rows = self.main_widget.well_tab.well_rows_input.value()
            cols = self.main_widget.well_tab.well_cols_input.value()
            if rows <= 0 or cols <= 0:
                return
            available_width = width - 2 * margin
            available_height = height - 2 * margin
            cell_width = available_width / cols
            cell_height = available_height / rows
            radius = min(cell_width, cell_height) * 0.4
            for r in range(rows):
                for c in range(cols):
                    center_x = margin + (c + 0.5) * cell_width
                    center_y = margin + (r + 0.5) * cell_height
                    well_loc = f"{chr(ord('A') + r)}{c + 1}"
                    if well_loc in self.associations and self.associations[well_loc]:
                        assignments = list(self.associations[well_loc].values())
                        if len(assignments) == 1:
                            fill_color = assignments[0]["color"]
                            painter.setPen(Qt.black)
                            painter.setBrush(QColor(fill_color))
                            painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                                                int(2 * radius), int(2 * radius))
                        else:
                            start_angle = 0
                            span_angle = 360 / len(assignments)
                            for assign in assignments:
                                painter.setPen(Qt.NoPen)
                                painter.setBrush(QColor(assign["color"]))
                                painter.drawPie(int(center_x - radius), int(center_y - radius),
                                                int(2 * radius), int(2 * radius),
                                                int(start_angle * 16), int(span_angle * 16))
                                start_angle += span_angle
                            painter.setPen(Qt.black)
                            painter.setBrush(Qt.NoBrush)
                            painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                                                int(2 * radius), int(2 * radius))
                    else:
                        painter.setPen(Qt.black)
                        painter.setBrush(QColor("#FFFFFF"))
                        painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                                            int(2 * radius), int(2 * radius))
                    painter.drawText(int(center_x - 10), int(center_y + 5), well_loc)
                    if well_loc in self.selected_wells:
                        pen = QPen(Qt.blue, 3, Qt.DashLine)
                        painter.setPen(pen)
                        painter.setBrush(Qt.NoBrush)
                        painter.drawEllipse(int(center_x - radius), int(center_y - radius),
                                            int(2 * radius), int(2 * radius))
        elif plate_mode == "Petridish":
            # Petridish layout: left 75% for the large dish, right 25% divided into 4 wells.
            total_width = width
            total_height = height
            left_area_width = total_width * 0.75 - 2 * margin
            right_area_width = total_width * 0.25 - 2 * margin
            available_height = total_height - 2 * margin
            left_rect = QRectF(margin, margin, total_width * 0.75 - 2 * margin, available_height)
            right_rect = QRectF(total_width * 0.75 + margin, margin, right_area_width, available_height)
            # Draw the petridish on the left.
            petridish_diameter = min(left_rect.width() * 0.75, left_rect.height())
            petridish_radius = petridish_diameter / 2
            petridish_center = QPointF(left_rect.center().x(), left_rect.center().y())
            if "A1" in self.associations and self.associations["A1"]:
                assignments = list(self.associations["A1"].values())
                if len(assignments) == 1:
                    fill_color = assignments[0]["color"]
                    painter.setPen(Qt.black)
                    painter.setBrush(QColor(fill_color))
                    painter.drawEllipse(petridish_center, petridish_radius, petridish_radius)
                else:
                    start_angle = 0
                    span_angle = 360 / len(assignments)
                    for assign in assignments:
                        painter.setPen(Qt.NoPen)
                        painter.setBrush(QColor(assign["color"]))
                        painter.drawPie(int(petridish_center.x()-petridish_radius), int(petridish_center.y()-petridish_radius),
                                        int(2 * petridish_radius), int(2 * petridish_radius),
                                        int(start_angle * 16), int(span_angle * 16))
                        start_angle += span_angle
                    painter.setPen(Qt.black)
                    painter.setBrush(Qt.NoBrush)
                    painter.drawEllipse(petridish_center, petridish_radius, petridish_radius)
            else:
                painter.setPen(Qt.black)
                painter.setBrush(QColor("#FFFFFF"))
                painter.drawEllipse(petridish_center, petridish_radius, petridish_radius)
            painter.drawText(int(petridish_center.x()-10), int(petridish_center.y()+5), "A1")
            # Draw the 4 wells on the right.
            num_wells = 4
            well_diameter = min(right_rect.width(), right_rect.height()/num_wells)
            for i in range(num_wells):
                center_x = right_rect.center().x()
                center_y = right_rect.top() + (i + 0.5) * (right_rect.height()/num_wells)
                well_label = f"{chr(ord('A') + i)}2"  # Wells in column 2: A2, B2, C2, D2
                if well_label in self.associations and self.associations[well_label]:
                    assignments = list(self.associations[well_label].values())
                    if len(assignments) == 1:
                        fill_color = assignments[0]["color"]
                        painter.setPen(Qt.black)
                        painter.setBrush(QColor(fill_color))
                        painter.drawEllipse(QPointF(center_x, center_y), well_diameter/2, well_diameter/2)
                    else:
                        start_angle = 0
                        span_angle = 360 / len(assignments)
                        for assign in assignments:
                            painter.setPen(Qt.NoPen)
                            painter.setBrush(QColor(assign["color"]))
                            painter.drawPie(int(center_x - well_diameter/2), int(center_y - well_diameter/2),
                                            int(well_diameter), int(well_diameter),
                                            int(start_angle * 16), int(span_angle * 16))
                            start_angle += span_angle
                        painter.setPen(Qt.black)
                        painter.setBrush(Qt.NoBrush)
                        painter.drawEllipse(QPointF(center_x, center_y), well_diameter/2, well_diameter/2)
                else:
                    painter.setPen(Qt.black)
                    painter.setBrush(QColor("#FFFFFF"))
                    painter.drawEllipse(QPointF(center_x, center_y), well_diameter/2, well_diameter/2)
                painter.drawText(int(center_x-10), int(center_y+5), well_label)
                if well_label in self.selected_wells:
                    pen = QPen(Qt.blue, 3, Qt.DashLine)
                    painter.setPen(pen)
                    painter.setBrush(Qt.NoBrush)
                    painter.drawEllipse(QPointF(center_x, center_y), well_diameter/2, well_diameter/2)

    def mousePressEvent(self, event):
        self.drag_start_pos = event.pos()
        pos = event.pos()
        margin = 20
        props = self.main_widget.well_tab.get_properties()
        plate_mode = props.get("plate_mode", "Well Plate")
        if plate_mode == "Well Plate":
            rows = self.main_widget.well_tab.well_rows_input.value()
            cols = self.main_widget.well_tab.well_cols_input.value()
            available_width = self.width() - 2 * margin
            available_height = self.height() - 2 * margin
            cell_width = available_width / cols
            cell_height = available_height / rows
            col = int((pos.x() - margin) / cell_width)
            row = int((pos.y() - margin) / cell_height)
            if 0 <= row < rows and 0 <= col < cols:
                well_loc = f"{chr(ord('A') + row)}{col + 1}"
                self.clicked_well = well_loc
                if well_loc not in self.associations or not self.associations[well_loc]:
                    if well_loc in self.selected_wells:
                        self.selected_wells.remove(well_loc)
                    else:
                        self.selected_wells.add(well_loc)
                    self.update()
        elif plate_mode == "Petridish":
            total_width = self.width()
            total_height = self.height()
            left_area = QRectF(margin, margin, total_width * 0.75 - 2 * margin, total_height - 2 * margin)
            right_area = QRectF(total_width * 0.75 + margin, margin, total_width * 0.25 - 2 * margin, total_height - 2 * margin)
            if left_area.contains(pos):
                self.clicked_well = "A1"
            elif right_area.contains(pos):
                relative_y = pos.y() - right_area.top()
                index = int(relative_y / (right_area.height()/4))
                index = max(0, min(3, index))
                self.clicked_well = f"{chr(ord('A') + index)}2"
            self.update()

    def mouseMoveEvent(self, event):
        if self.clicked_well and self.clicked_well in self.associations:
            if (event.pos() - self.drag_start_pos).manhattanLength() > QApplication.startDragDistance():
                from PySide6.QtCore import QMimeData
                drag = QDrag(self)
                mimeData = QMimeData()
                mimeData.setText(f"remove;{self.clicked_well}")
                drag.setMimeData(mimeData)
                drag.exec(Qt.MoveAction)
                self.clicked_well = None

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        pos = event.position() if hasattr(event, "position") else event.pos()
        margin = 20
        props = self.main_widget.well_tab.get_properties()
        plate_mode = props.get("plate_mode", "Well Plate")
        
        # Determine target well location based on plate mode.
        if plate_mode == "Well Plate":
            rows = self.main_widget.well_tab.well_rows_input.value()
            cols = self.main_widget.well_tab.well_cols_input.value()
            available_width = self.width() - 2 * margin
            available_height = self.height() - 2 * margin
            cell_width = available_width / cols
            cell_height = available_height / rows
            col = int((pos.x() - margin) / cell_width)
            row = int((pos.y() - margin) / cell_height)
            if not (0 <= row < rows and 0 <= col < cols):
                event.ignore()
                return
            well_loc = f"{chr(ord('A') + row)}{col + 1}"
        elif plate_mode == "Petridish":
            total_width = self.width()
            total_height = self.height()
            left_area = QRectF(margin, margin, total_width * 0.75 - 2 * margin, total_height - 2 * margin)
            right_area = QRectF(total_width * 0.75 + margin, margin, total_width * 0.25 - 2 * margin, total_height - 2 * margin)
            if left_area.contains(pos):
                well_loc = "A1"
            elif right_area.contains(pos):
                relative_y = pos.y() - right_area.top()
                index = int(relative_y / (right_area.height() / 4))
                index = max(0, min(3, index))
                well_loc = f"{chr(ord('A') + index)}2"
            else:
                event.ignore()
                return
        else:
            event.ignore()
            return

        mime = event.mimeData()
        
        # Handle multi-file drops if our custom JSON data is available.
        if mime.hasFormat("application/json"):
            try:
                json_bytes = mime.data("application/json")
                json_str = bytes(json_bytes).decode('utf-8')
                files_data = json.loads(json_str)
            except Exception:
                event.ignore()
                return
            
            if not isinstance(files_data, list):
                event.ignore()
                return
            
            # Prevent file drops if an inkwell is already assigned.
            if (well_loc in self.associations and 
                any(key.startswith("InkWell") for key in self.associations[well_loc])):
                event.ignore()
                return
            
            # Determine target wells: use selected wells if available, else the calculated well.
            targets = self.selected_wells if self.selected_wells else {well_loc}
            for file_data in files_data:
                uid = file_data.get("uid")
                file_path = file_data.get("file_path")
                file_color = file_data.get("file_color")
                file_name = file_data.get("file_name")
                if not (uid and file_path and file_color and file_name):
                    continue
                for target in targets:
                    if target not in self.associations:
                        self.associations[target] = {}
                    if uid not in self.associations[target]:
                        self.associations[target][uid] = {"color": file_color, "name": file_name}
            self.selected_wells.clear()
            self.associationsChanged.emit()
            self.update()
            event.acceptProposedAction()
            self.main_widget.apply_settings()
            return

        # Fallback: handle removals, inkwell drops, or single file drops (unchanged)
        mime_text = mime.text()
        parts = mime_text.split(";")
        if parts[0] == "remove":
            source_well = parts[1] if len(parts) > 1 else well_loc
            if source_well in self.associations:
                keys = list(self.associations[source_well].keys())
                removed = False
                for key in keys:
                    if not key.startswith("InkWell"):
                        del self.associations[source_well][key]
                        removed = True
                        break
                if removed and not self.associations[source_well]:
                    del self.associations[source_well]
                self.associationsChanged.emit()
                self.update()
            event.acceptProposedAction()
            self.main_widget.apply_settings()
            return

        if parts[0] == "inkwell":
            if len(parts) < 3:
                event.ignore()
                return
            try:
                index = int(parts[1])
            except ValueError:
                event.ignore()
                return
            inkwell_widgets = self.main_widget.ink_tab.ink_well_widgets
            if index >= len(inkwell_widgets):
                event.ignore()
                return
            inkwell = inkwell_widgets[index]
            if (well_loc in self.associations and 
                any(key.startswith("InkWell") for key in self.associations[well_loc])):
                event.ignore()
                return
            if well_loc not in self.associations:
                self.associations[well_loc] = {}
            self.associations[well_loc][f"InkWell_{index}"] = {
                "color": inkwell["color"].text().strip(),
                "name": inkwell["cell_type"].text().strip()
            }
            inkwell["location"].setText(well_loc)
            self.associationsChanged.emit()
            self.update()
            event.acceptProposedAction()
            self.main_widget.apply_settings()
            return

        if parts[0] == "file":
            # Single file drop (text-based)
            if len(parts) < 5:
                event.ignore()
                return
            uid = parts[1]
            file_path = parts[2]
            file_color = parts[3]
            file_name = parts[4]
            if (well_loc in self.associations and 
                any(key.startswith("InkWell") for key in self.associations[well_loc])):
                event.ignore()
                return
            targets = self.selected_wells if self.selected_wells else {well_loc}
            for target in targets:
                if target not in self.associations:
                    self.associations[target] = {}
                if uid not in self.associations[target]:
                    self.associations[target][uid] = {"color": file_color, "name": file_name}
            self.selected_wells.clear()
            self.associationsChanged.emit()
            self.update()
            event.acceptProposedAction()
            self.main_widget.apply_settings()
            return

        event.ignore()
        self.main_widget.apply_settings()

    def update_layout(self):
        self.update()

############################################################################################################
###################################  All Tabs  #############################################################
############################################################################################################
# --- Well Properties Tab as a Separate Class ---
class WellPropertiesTab(QWidget):
    def __init__(self, main_widget, on_change_callback, parent=None):
        super().__init__(parent)
        self.print_manager = main_widget.print_manager
        self.on_change_callback = on_change_callback
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        btn_layout = QHBoxLayout()
        load_btn = QPushButton("Load Well Properties")
        load_btn.clicked.connect(self.load_properties)
        btn_layout.addWidget(load_btn)
        save_btn = QPushButton("Save Well Properties")
        save_btn.clicked.connect(self.save_properties)
        btn_layout.addWidget(save_btn)
        layout.addLayout(btn_layout)

        self.form = QFormLayout()
        self.fastz_input = QDoubleSpinBox()
        self.fastz_input.setValue(self.print_manager.well_properties.get("fastz", 0))
        self.fastz_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Fast Z:", self.fastz_input)

        self.floorz_input = QDoubleSpinBox()
        self.floorz_input.setValue(self.print_manager.well_properties.get("floorz", 0))
        self.floorz_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Floor Z:", self.floorz_input)

        self.topz_input = QDoubleSpinBox()
        self.topz_input.setValue(self.print_manager.well_properties.get("topz", 0))
        self.topz_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Top Z:", self.topz_input)

        self.ink_topz_input = QDoubleSpinBox()
        self.ink_topz_input.setValue(self.print_manager.well_properties.get("ink_topz", 0))
        self.ink_topz_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Ink Top Z:", self.ink_topz_input)

        self.ink_floorz_input = QDoubleSpinBox()
        self.ink_floorz_input.setValue(self.print_manager.well_properties.get("ink_floorz", 0))
        self.ink_floorz_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Ink Floor Z:", self.ink_floorz_input)

        self.well_A1_x_input = QDoubleSpinBox()
        self.well_A1_x_input.setValue(self.print_manager.well_properties.get("Well_A1_x", 0))
        self.well_A1_x_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Well A1 X:", self.well_A1_x_input)

        self.well_A1_y_input = QDoubleSpinBox()
        self.well_A1_y_input.setValue(self.print_manager.well_properties.get("Well_A1_y", 0))
        self.well_A1_y_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Well A1 Y:", self.well_A1_y_input)

        self.well_dx_input = QDoubleSpinBox()
        self.well_dx_input.setValue(self.print_manager.well_properties.get("well_dx", 0))
        self.well_dx_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Well DX:", self.well_dx_input)

        self.well_dy_input = QDoubleSpinBox()
        self.well_dy_input.setValue(self.print_manager.well_properties.get("well_dy", 0))
        self.well_dy_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Well DY:", self.well_dy_input)

        self.well_rows_input = QSpinBox()
        self.well_rows_input.setValue(self.print_manager.well_properties.get("well_rows", 0))
        self.well_rows_input.valueChanged.connect(lambda: self.on_change_callback())
        self.form.addRow("Well Rows:", self.well_rows_input)

        self.well_cols_input = QSpinBox()
        self.well_cols_input.setValue(self.print_manager.well_properties.get("well_cols", 0))
        self.well_cols_input.valueChanged.connect(lambda: self.on_change_callback())
        self.form.addRow("Well Cols:", self.well_cols_input)

        self.well_diameter_input = QDoubleSpinBox()
        self.well_diameter_input.setValue(self.print_manager.well_properties.get("well_diameter", 0))
        self.well_diameter_input.valueChanged.connect(self.on_change_callback)
        self.form.addRow("Well Diameter:", self.well_diameter_input)

        # New: Plate Type selection
        self.plate_type_combo = QComboBox()
        self.plate_type_combo.addItems(["Well Plate", "Petridish"])
        self.plate_type_combo.currentIndexChanged.connect(self.on_change_callback)
        self.form.addRow("Plate Type:", self.plate_type_combo)

        layout.addLayout(self.form)

    def get_properties(self):
        return {
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
            "well_diameter": self.well_diameter_input.value(),
            "plate_mode": self.plate_type_combo.currentText()
        }

    def load_properties(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Well Properties", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, "r") as f:
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
                plate_mode = data.get("plate_mode", "Well Plate")
                index = self.plate_type_combo.findText(plate_mode)
                if index >= 0:
                    self.plate_type_combo.setCurrentIndex(index)
                self.on_change_callback()
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading JSON: {e}")

    def save_properties(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Well Properties", "", "JSON Files (*.json)")
        if filename:
            try:
                data = self.get_properties()
                with open(filename, "w") as f:
                    json.dump(data, f, indent=4)
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving JSON: {e}")

    def validate(self):
        if self.well_rows_input.value() <= 0 or self.well_cols_input.value() <= 0:
            return False, "Well Rows and Well Cols must be greater than 0."
        return True, ""

class InkPropertiesTab(QWidget):
    def __init__(self, main_widget, on_change_callback, parent=None):
        """
        well_properties_tab: a reference to the WellPropertiesTab
           (so we can read well_rows and well_cols to update validators)
        on_change_callback: a callback to notify when values change.
        """
        super().__init__(parent)
        self.main_widget = main_widget
        self.well_properties_tab = main_widget.well_tab
        self.on_change_callback = on_change_callback
        self.ink_well_widgets = []  # each entry will be a dict containing UI elements and an InkWell instance
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        btn_layout = QHBoxLayout()
        add_btn = QPushButton("Add Ink Well")
        add_btn.clicked.connect(self.add_ink_well_widget)
        btn_layout.addWidget(add_btn)
        layout.addLayout(btn_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        container = QWidget()
        self.ink_well_layout = QVBoxLayout(container)
        container.setLayout(self.ink_well_layout)
        scroll_area.setWidget(container)
        layout.addWidget(scroll_area)

    def add_ink_well_widget(self):
        group_box = QGroupBox(f"Ink Well {len(self.ink_well_widgets) + 1}")
        layout = QFormLayout(group_box)
        
        # Create the location field with a validator.
        location_edit = QLineEdit()
        validator = InkWellLocationValidator(
            self.well_properties_tab.well_rows_input.value(),
            self.well_properties_tab.well_cols_input.value()
        )
        location_edit.setValidator(validator)
        location_edit.textChanged.connect(self.on_change_callback)
        # Use editingFinished so update fires when editing is complete.
        location_edit.editingFinished.connect(lambda idx=len(self.ink_well_widgets), le=location_edit:
                                                self.update_plate_layout_for_inkwell(idx, le.text()))
        layout.addRow("Location:", location_edit)
        
        # Create the cell type (name) field.
        cell_type_edit = QLineEdit("DefaultCell")
        cell_type_edit.textChanged.connect(self.on_change_callback)
        # Update the plate layout when the cell type changes.
        cell_type_edit.editingFinished.connect(lambda idx=len(self.ink_well_widgets), ct=cell_type_edit, le=location_edit:
                                                self.update_plate_layout_for_inkwell(idx, le.text()))
        layout.addRow("Cell Type:", cell_type_edit)
        
        chilled_checkbox = QPushButton("Chilled")
        chilled_checkbox.setCheckable(True)
        chilled_checkbox.setChecked(True)
        layout.addRow("Chilled:", chilled_checkbox)
        
        volume_spin = QDoubleSpinBox()
        volume_spin.setValue(0)
        volume_spin.valueChanged.connect(self.on_change_callback)
        layout.addRow("Volume:", volume_spin)
        
        color_btn = QPushButton()
        default_color = "#868eff"
        color_btn.setText(default_color)
        color_btn.setStyleSheet(f"background-color: {default_color}")
        # Color changes are handled in pick_color below.
        color_btn.clicked.connect(lambda: self.pick_color(color_btn))
        layout.addRow("Color:", color_btn)
        
        del_btn = QPushButton("Delete")
        del_btn.clicked.connect(lambda: self.remove_ink_well_widget(group_box))
        layout.addRow(del_btn)
        
        # Enable drag for moving the inkwell.
        group_box.setMouseTracking(True)
        group_box.mousePressEvent = lambda event, idx=len(self.ink_well_widgets): self.start_drag_inkwell(event, idx)
        
        # Create an InkWell instance to hold the data.
        inkwell_obj = InkWell()
        inkwell_obj.set_location("")
        inkwell_obj.set_cell_type(cell_type_edit.text().strip())
        inkwell_obj.set_chilled(chilled_checkbox.isChecked())
        inkwell_obj.set_volume(volume_spin.value())
        inkwell_obj.set_color(color_btn.text().strip())
        
        # Store the UI elements and the InkWell instance.
        ink_widget = {
            "group": group_box,
            "location": location_edit,
            "cell_type": cell_type_edit,
            "chilled": chilled_checkbox,
            "volume": volume_spin,
            "color": color_btn,
            "index": len(self.ink_well_widgets),
            "inkwell": inkwell_obj
        }
        self.ink_well_widgets.append(ink_widget)
        self.ink_well_layout.addWidget(group_box)
        self.on_change_callback()

    def update_plate_layout_for_inkwell(self, index, new_location):
        try:
            ink_widget = self.ink_well_widgets[index]
        except IndexError:
            return

        # Update the InkWell instance with the new location and current cell type.
        ink_widget["inkwell"].set_location(new_location)
        ink_widget["inkwell"].set_cell_type(ink_widget["cell_type"].text().strip())

        plate_widget = self.main_widget.plate_layout_tab.plate_layout_widget

        # Validate the new location using the QLineEdit's validator.
        validator = ink_widget["location"].validator()
        state, _, _ = validator.validate(new_location, 0)
        if state != QValidator.Acceptable:
            # Remove any existing assignment if invalid.
            for well in list(plate_widget.associations.keys()):
                if f"InkWell_{index}" in plate_widget.associations[well]:
                    del plate_widget.associations[well][f"InkWell_{index}"]
                    if not plate_widget.associations[well]:
                        del plate_widget.associations[well]
            plate_widget.associationsChanged.emit()
            plate_widget.update()
            return

        current_color = ink_widget["color"].text().strip()
        cell_type = ink_widget["cell_type"].text().strip()

        # Remove previous assignment if location has changed.
        old_well = None
        for well, assignments in plate_widget.associations.items():
            if f"InkWell_{index}" in assignments:
                old_well = well
                break
        if old_well and old_well != new_location:
            del plate_widget.associations[old_well][f"InkWell_{index}"]
            if not plate_widget.associations[old_well]:
                del plate_widget.associations[old_well]

        # Add or update the assignment at the new location.
        if new_location not in plate_widget.associations:
            plate_widget.associations[new_location] = {}
        plate_widget.associations[new_location][f"InkWell_{index}"] = {
            "color": current_color,
            "name": cell_type
        }
        plate_widget.associationsChanged.emit()
        plate_widget.update()

    def pick_color(self, button):
        color = QColorDialog.getColor(QColor(button.text()), self, "Pick Ink Well Color")
        if color.isValid():
            new_color = color.name()
            button.setText(new_color)
            button.setStyleSheet(f"background-color: {new_color}")
            self.on_change_callback()
            # Update the corresponding InkWell instance.
            for idx, widget in enumerate(self.ink_well_widgets):
                if widget["color"] == button:
                    loc = widget["location"].text().strip()
                    widget["inkwell"].set_color(new_color)
                    if loc:
                        self.update_plate_layout_for_inkwell(idx, loc)
                    break

    def remove_ink_well_widget(self, widget):
        for ink_widget in self.ink_well_widgets:
            if ink_widget["group"] == widget:
                self.ink_well_widgets.remove(ink_widget)
                break
        widget.setParent(None)
        widget.deleteLater()
        self.on_change_callback()

    def update_validators(self):
        max_rows = self.well_properties_tab.well_rows_input.value()
        max_cols = self.well_properties_tab.well_cols_input.value()
        for widget in self.ink_well_widgets:
            validator = widget["location"].validator()
            if isinstance(validator, InkWellLocationValidator):
                validator.setLimits(max_rows, max_cols)
                widget["location"].setValidator(validator)

    def get_ink_wells(self):
        """
        Update each InkWell instance from its UI fields and return a dictionary of InkWell objects.
        """
        data = {}
        for i, widget in enumerate(self.ink_well_widgets, start=1):
            inkwell_obj = widget["inkwell"]
            inkwell_obj.set_location(widget["location"].text().strip())
            inkwell_obj.set_cell_type(widget["cell_type"].text().strip())
            inkwell_obj.set_chilled(widget["chilled"].isChecked())
            inkwell_obj.set_volume(widget["volume"].value())
            inkwell_obj.set_color(widget["color"].text().strip())
            data[f"ink_{i}"] = inkwell_obj
        return data
    
    def get_ink_wells_json(self):
        """
        Update each InkWell instance from its UI fields and return a dictionary with
        only JSON serializable values.
        """
        data = {}
        for i, widget in enumerate(self.ink_well_widgets, start=1):
            # Update the InkWell instance from UI fields (if needed)
            widget["inkwell"].set_location(widget["location"].text().strip())
            widget["inkwell"].set_cell_type(widget["cell_type"].text().strip())
            widget["inkwell"].set_chilled(widget["chilled"].isChecked())
            widget["inkwell"].set_volume(widget["volume"].value())
            widget["inkwell"].set_color(widget["color"].text().strip())
            
            # Create a serializable dictionary instead of returning the InkWell object.
            data[f"ink_{i}"] = {
                "location": widget["location"].text().strip(),
                "cell_type": widget["cell_type"].text().strip(),
                "chilled": widget["chilled"].isChecked(),
                "volume": widget["volume"].value(),
                "color": widget["color"].text().strip()
            }
        return data

    def start_drag_inkwell(self, event, index):
        from PySide6.QtCore import QMimeData  # if not already imported
        drag = QDrag(self)
        mimeData = QMimeData()
        # Format: "inkwell;{index};{color}"
        color = self.ink_well_widgets[index]["color"].text().strip()
        mimeData.setText(f"inkwell;{index};{color}")
        drag.setMimeData(mimeData)
        drag.exec(Qt.MoveAction)
   
    def validate(self):
        if not self.ink_well_widgets:
            return False, "At least one ink well must be defined."
            
        seen = {}
        for i, widget in enumerate(self.ink_well_widgets, start=1):
            loc = widget["location"].text().strip().upper()
            if not loc:
                return False, f"Ink well {i} must have a location."
            if loc in seen:
                return False, f"Ink well {i} duplicates assignment from ink well {seen[loc]}."
            seen[loc] = i
            state, _, _ = widget["location"].validator().validate(widget["location"].text(), 0)
            if state != QValidator.Acceptable:
                return False, f"Ink well {i} location is invalid."
            if not widget["cell_type"].text().strip():
                return False, f"Ink well {i} must have a name (cell type) defined."
            if widget["volume"].value() <= 0:
                return False, f"Ink well {i} must have a volume greater than zero."
            if not widget["color"].text().strip():
                return False, f"Ink well {i} must have a color code."
        return True, ""

# --- Syringe Properties Tab as a Separate Class ---
class SyringePropertiesTab(QWidget):
    def __init__(self, main_widget, on_change_callback, parent=None):
        super().__init__(parent)
        self.main_widget = main_widget
        self.print_manager = main_widget.print_manager
        self.standard_syringe_types = {}  # will be loaded from "Syringes.json"
        self.load_standard_syringe_types_from_file(file_path="Syringes.json")
        self.on_change_callback = on_change_callback
        self.ink_tab = main_widget.ink_tab  # Save direct reference to InkPropertiesTab
        self.syringe_inputs = {}
        self.current_inks = {"None"}
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        btn_layout = QHBoxLayout()
        load_btn = QPushButton("Load Syringe Layout")
        load_btn.clicked.connect(self.load_layout)
        btn_layout.addWidget(load_btn)
        save_btn = QPushButton("Save Syringe Layout")
        save_btn.clicked.connect(self.save_layout)
        btn_layout.addWidget(save_btn)
        layout.addLayout(btn_layout)

        self.syringe_form = QFormLayout()
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
            diameter_spin.valueChanged.connect(self.on_change_callback)
            group_layout.addRow("Diameter:", diameter_spin)
            length_spin = QDoubleSpinBox()
            length_spin.setValue(getattr(self.print_manager.syringes[pump], "length") or 0)
            length_spin.valueChanged.connect(self.on_change_callback)
            group_layout.addRow("Length:", length_spin)
            cell_type_combo = QComboBox()
            self.update_syringe_cell_type_options_for(pump, cell_type_combo)
            cell_type_combo.currentIndexChanged.connect(lambda idx, p=pump, combo=cell_type_combo: self.update_cell_type_color(p, combo))
            cell_type_combo.currentIndexChanged.connect(self.on_change_callback)
            group_layout.addRow("Cell Type:", cell_type_combo)
            self.syringe_inputs[pump] = {
                "std_type": std_type_combo,
                "diameter": diameter_spin,
                "length": length_spin,
                "cell_type": cell_type_combo
            }
            self.syringe_form.addRow(group_box)
        layout.addLayout(self.syringe_form)

    def update_syringe_cell_type_options_for(self, pump, combo_box):
        valid_types = {"None"}
        cell_type_to_color = {"None": ""}
        for ink_widget in self.main_widget.ink_tab.ink_well_widgets:
            ct = ink_widget["cell_type"].text().strip()
            if ct:
                valid_types.add(ct)
                cell_type_to_color[ct] = ink_widget["color"].text().strip()
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
        selected = combo_box.currentText().strip()
        color = cell_type_to_color.get(selected, "")
        if color:
            combo_box.setStyleSheet(f"background-color: {color}")
        else:
            combo_box.setStyleSheet("")
        
    def update_cell_type_color(self, pump, combo_box):
        selected = combo_box.currentText().strip()
        for ink_widget in self.ink_tab.ink_well_widgets:
            if ink_widget["cell_type"].text().strip() == selected:
                combo_box.setStyleSheet(f"background-color: {ink_widget['color'].text().strip()}")
                return
        combo_box.setStyleSheet("")

    def update_all_cell_type_options(self):
        self.current_inks = {"None"}
        for ink_widget in self.main_widget.ink_tab.ink_well_widgets:
            cell_type = ink_widget["cell_type"].text().strip()
            if cell_type:
                self.current_inks.add(cell_type)
        for pump, inputs in self.syringe_inputs.items():
            current = inputs["cell_type"].currentText().strip()
            with QSignalBlocker(inputs["cell_type"]):
                inputs["cell_type"].clear()
                for ct in sorted(self.current_inks):
                    inputs["cell_type"].addItem(ct)
                index = inputs["cell_type"].findText(current)
                if index < 0:
                    index = inputs["cell_type"].findText("None")
                    if index < 0:
                        index = 0
                inputs["cell_type"].setCurrentIndex(index)

    def standard_type_changed(self, pump, combo_box):
        std_type = combo_box.currentText()
        if std_type in self.standard_syringe_types:
            values = self.standard_syringe_types[std_type]
            self.syringe_inputs[pump]["diameter"].setValue(values.get("diameter", 0))
            self.syringe_inputs[pump]["length"].setValue(values.get("length", 0))
        self.on_change_callback()

    def get_syringe_data(self):
        data = {}
        for pump, inputs in self.syringe_inputs.items():
            data[pump] = {
                "diameter": inputs["diameter"].value(),
                "length": inputs["length"].value(),
                "cell_type": inputs["cell_type"].currentText(),
                "standard_type": inputs["std_type"].currentText()
            }
        return data

    def load_standard_syringe_types_from_file(self, file_path="Syringes.json"):
        try:
            if os.path.exists(file_path):
                with open(file_path, "r") as f:
                    self.standard_syringe_types = json.load(f)
            else:
                self.standard_syringe_types = {}
        except Exception as e:
            QMessageBox.warning(self, "Standard Types Load Error", f"Error loading Syringes.json: {e}")
            self.standard_syringe_types = {}

    def load_layout(self):
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
                
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Error loading JSON: {e}")

    def save_layout(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Syringe Layout", "", "JSON Files (*.json)")
        if filename:
            try:
                data = self.get_syringe_data()
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=4)
                
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Error saving JSON: {e}")

    def validate(self):
        non_none_exists = False
        valid_cell_types = self.current_inks
        print(valid_cell_types)
        for pump, inputs in self.syringe_inputs.items():
            cell_type = inputs["cell_type"].currentText().strip()
            if cell_type not in valid_cell_types:
                return False, f"Syringe {pump.upper()} cell type '{cell_type}' is invalid."
            if cell_type != "None":
                non_none_exists = True
                if inputs["diameter"].value() <= 0 or inputs["length"].value() <= 0:
                    return False, f"Syringe {pump.upper()} must have positive diameter and length if a cell type is selected."
        if not non_none_exists:
            return False, "At least one syringe must have a cell type other than 'None'."
        return True, ""

# --- Print Setup Tab ---
class PrintSetupTab(QWidget):
    # Signal to notify that the list of print files has changed.
    printFilesChanged = Signal()

    def __init__(self, main_widget, parent=None):
        super().__init__(parent)
        
        self.main_widget = main_widget
        self.well_diameter_physical = 1.0  
        self.well_height_physical = 1.0 
        
        self.print_files = []   # list of PrintFile objects
        self.current_print = None

        main_layout = QVBoxLayout(self)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        self.list_widget = QListWidget()
        self.list_widget.setItemDelegate(ColorItemDelegate())
        self.list_widget.setStyleSheet("""
            QListWidget::item {
                padding: 2px;
            }
            QListWidget::item:hover {
                border: 2px solid grey;
            }
            QListWidget::item:selected {
                border: 2px solid black;
            }
        """)
        self.list_widget.currentRowChanged.connect(self.selection_changed)
        left_layout.addWidget(self.list_widget)
        btn_layout = QHBoxLayout()
        new_print_btn = QPushButton("New Print")
        new_print_btn.clicked.connect(self.new_print)
        btn_layout.addWidget(new_print_btn)
        dup_btn = QPushButton("Duplicate")
        dup_btn.clicked.connect(self.duplicate_print_file)
        btn_layout.addWidget(dup_btn)
        rem_btn = QPushButton("Remove")
        rem_btn.clicked.connect(self.remove_print_file)
        btn_layout.addWidget(rem_btn)
        left_layout.addLayout(btn_layout)
        splitter.addWidget(left_panel)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        details_panel = QWidget()
        details_layout = QHBoxLayout(details_panel)
        details_layout.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit()
        self.name_edit.editingFinished.connect(self.update_current_print)
        details_layout.addWidget(self.name_edit)
        details_layout.addWidget(QLabel("Color:"))
        self.color_btn = QPushButton("Pick Color")
        self.color_btn.clicked.connect(self.pick_color)
        details_layout.addWidget(self.color_btn)
        self.current_color = "#FF0000"
        details_layout.addWidget(QLabel("CSV File:"))
        self.csv_edit = QLineEdit("waypoint.csv")
        details_layout.addWidget(self.csv_edit)
        load_csv_btn = QPushButton("Load CSV")
        load_csv_btn.clicked.connect(self.load_csv)
        details_layout.addWidget(load_csv_btn)
        right_layout.addWidget(details_panel)

        preview_panel = QWidget()
        preview_layout = QHBoxLayout(preview_panel)
        self.well_preview = WellPreviewWidget()
        self.well_preview.well_diameter_physical = self.well_diameter_physical
        self.well_preview.well_height_physical = self.well_height_physical
        preview_layout.addWidget(self.well_preview)
        self.iso_preview = IsometricPreviewWidget()
        preview_layout.addWidget(self.iso_preview)
        right_layout.addWidget(preview_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([150, 800])

    def update_from_well_properties(self):
        props = self.main_widget.well_tab.get_properties()
        self.well_diameter_physical = props["well_diameter"]
        self.well_height_physical = props["topz"] - props["floorz"]
        self.well_preview.well_diameter_physical = self.well_diameter_physical
        self.well_preview.well_height_physical = self.well_height_physical
    
    def update_print_file_bbox(self, pf):
        if pf.waypoints:
            xs = [pt['x'] for pt in pf.waypoints]
            ys = [pt['y'] for pt in pf.waypoints]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            range_x = max_x - min_x
            range_y = max_y - min_y

            margin = 10
            area_width = (self.well_preview.width() // 2) - 2 * margin
            area_height = self.well_preview.height() - 2 * margin
            radius = min(area_width, area_height) / 2
            drawn_diameter = 2 * radius

            # Compute bbox dimensions based on the physical-to-drawn scale.
            new_width = (range_x / self.well_diameter_physical) * drawn_diameter
            new_height = (range_y / self.well_diameter_physical) * drawn_diameter
            if new_width < 10: new_width = 10
            if new_height < 10: new_height = 10
            pf.bbox_size = QSizeF(new_width, new_height)

            # Compute the center of the waypoint data and scale it.
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            scale = drawn_diameter / self.well_diameter_physical
            pf.bbox_offset = QPointF(center_x * scale, center_y * scale)

    
    def validate(self):
        errors = []
        if self.well_diameter_physical <= 0:
            errors.append("Well diameter must be greater than zero.")
        if self.well_height_physical <= 0:
            errors.append("Well height (topz - floorz) must be greater than zero.")
        if errors:
            return False, "; ".join(errors)
        return True, ""
    
    def new_print(self):
        files, _ = QFileDialog.getOpenFileNames(self, "New Print - Import CSV Files", "", "CSV Files (*.csv)")
        if files:
            for file in files:
                base = os.path.basename(file)
                name = os.path.splitext(base)[0]
                r = random.randint(0,255)
                g = random.randint(0,255)
                b = random.randint(0,255)
                color = f"#{r:02X}{g:02X}{b:02X}"
                pf = PrintFile(name, color, file)
                pf.load_csv()
                if pf.waypoints:
                    self.update_print_file_bbox(pf)
                else:
                    pf.bbox_size = QSizeF(50, 50)
                    pf.bbox_offset = QPointF(0, 0)
                pf.floor_offset = None
                self.print_files.append(pf)
                item = QListWidgetItem(pf.name)
                item.setBackground(QColor(pf.color))
                item.setForeground(QColor(get_contrast_text_color(pf.color)))
                self.list_widget.addItem(item)
            self.list_widget.setCurrentRow(0)
            self.printFilesChanged.emit()


    def duplicate_print_file(self):
        if self.current_print:
            pf = PrintFile(self.current_print.name + " Copy", self.current_print.color, self.current_print.csv_file)
            pf.bbox_offset = QPointF(self.current_print.bbox_offset)
            pf.floor_offset = self.current_print.floor_offset
            pf.waypoints = list(self.current_print.waypoints)
            pf.bbox_size = QSizeF(self.current_print.bbox_size.width(), self.current_print.bbox_size.height())
            self.print_files.append(pf)
            item = QListWidgetItem(pf.name)
            item.setBackground(QColor(pf.color))
            item.setForeground(QColor(get_contrast_text_color(pf.color)))
            self.list_widget.addItem(item)
            self.list_widget.setCurrentRow(len(self.print_files)-1)
            self.printFilesChanged.emit()

    def remove_print_file(self):
        row = self.list_widget.currentRow()
        if 0 <= row < len(self.print_files):
            del self.print_files[row]
            self.list_widget.takeItem(row)
            if self.print_files:
                self.list_widget.setCurrentRow(0)
            else:
                self.current_print = None
                self.well_preview.print_file = None
                self.iso_preview.print_file = None
                self.well_preview.update()
                self.iso_preview.update()
            self.printFilesChanged.emit()

    def selection_changed(self, index):
        if not (0 <= index < len(self.print_files)):
            self.current_print = None
            return
        self.current_print = self.print_files[index]
        self.name_edit.setText(self.current_print.name)
        self.csv_edit.setText(self.current_print.csv_file)
        self.current_color = self.current_print.color
        self.color_btn.setStyleSheet(f"background-color: {self.current_color}")
        self.current_print.load_csv()
        if self.current_print.waypoints:
            self.update_print_file_bbox(self.current_print)
        margin = 10
        side_bottom = self.well_preview.height() - margin
        if self.current_print.floor_offset is None:
            self.current_print.floor_offset = side_bottom
        self.well_preview.print_file = self.current_print
        self.iso_preview.print_file = self.current_print
        self.well_preview.update()
        self.iso_preview.update()
        self.list_widget.item(index).setBackground(QColor(self.current_print.color))
        self.list_widget.item(index).setForeground(QColor(get_contrast_text_color(self.current_print.color)))

    def update_current_print(self):
        if self.current_print:
            new_name = self.name_edit.text().strip()
            self.current_print.name = new_name
            cur_row = self.list_widget.currentRow()
            if cur_row >= 0:
                self.list_widget.item(cur_row).setText(new_name)
            plate_layout = self.main_widget.plate_layout_tab.plate_layout_widget
            for well, assignments in plate_layout.associations.items():
                if self.current_print.uid in assignments:
                    assignments[self.current_print.uid]["name"] = new_name
            plate_layout.associationsChanged.emit()
            plate_layout.update()
            self.printFilesChanged.emit()

    def pick_color(self):
        color = QColorDialog.getColor(QColor(self.current_color), self, "Pick Color")
        if color.isValid():
            self.current_color = color.name()
            self.color_btn.setStyleSheet(f"background-color: {self.current_color}")
            if self.current_print:
                self.current_print.color = self.current_color
                cur_row = self.list_widget.currentRow()
                if cur_row >= 0:
                    self.list_widget.item(cur_row).setBackground(QColor(self.current_color))
                plate_layout = self.main_widget.plate_layout_tab.plate_layout_widget
                for well, assignments in plate_layout.associations.items():
                    if self.current_print.uid in assignments:
                        assignments[self.current_print.uid]["color"] = self.current_print.color
                plate_layout.associationsChanged.emit()
                plate_layout.update()
            self.printFilesChanged.emit()

    def load_csv(self):
        file, _ = QFileDialog.getOpenFileName(self, "Select CSV File", "", "CSV Files (*.csv)")
        if file:
            self.csv_edit.setText(file)
            if self.current_print:
                self.current_print.csv_file = file
                self.current_print.load_csv()
                if self.current_print.waypoints:
                    self.update_print_file_bbox(self.current_print)
                self.well_preview.update()
                self.iso_preview.update()
                self.printFilesChanged.emit()

    def reset_view(self):
        self.iso_preview.yaw = 0
        self.iso_preview.pitch = 1.0
        self.iso_preview.update()

    def top_view(self):
        self.iso_preview.yaw = 0
        self.iso_preview.pitch = 0
        self.iso_preview.update()

    def side_view(self):
        self.iso_preview.yaw = 90
        self.iso_preview.pitch = 5.0
        self.iso_preview.update()

    def get_print_files(self):
        return self.print_files

# --- Plate Layout Tab (container for CSV list and plate layout view) ---
class PlateLayoutTab(QWidget):
    def __init__(self, main_widget, parent=None):
        """
        main_widget should provide:
          - well_tab with well_rows_input and well_cols_input or petri dish parameters.
          - print_setup_tab with a get_print_files() method and a printFilesChanged signal.
        """
        super().__init__(parent)
        self.main_widget = main_widget
        layout = QHBoxLayout(self)

        self.printfile_list = QListWidget()
        self.printfile_list.setDragEnabled(False)
        self.printfile_list.viewport().installEventFilter(self)
        self.printfile_list.setSelectionMode(QAbstractItemView.ExtendedSelection)

        layout.addWidget(self.printfile_list, 1)

        self.plate_layout_widget = PlateLayoutWidget(main_widget)
        layout.addWidget(self.plate_layout_widget, 3)

        self.running_list = QListWidget()
        layout.addWidget(self.running_list, 1)

        self.plate_layout_widget.associationsChanged.connect(self.refresh_running_list)
        self.main_widget.print_setup_tab.printFilesChanged.connect(self.refresh_print_file_list)

        self.refresh_print_file_list()
        self.refresh_running_list()

    def eventFilter(self, source, event):
        from PySide6.QtCore import QEvent
        if source is self.printfile_list.viewport():
            if event.type() == QEvent.MouseButtonPress:
                self._drag_start_pos = event.pos()
            elif event.type() == QEvent.MouseMove:
                if event.buttons() & Qt.LeftButton:
                    if (event.pos() - self._drag_start_pos).manhattanLength() >= QApplication.startDragDistance():
                        item = self.printfile_list.itemAt(self._drag_start_pos)
                        if item is not None:
                            data = item.data(Qt.UserRole)
                            if data:
                                uid, file_path, file_color, file_name = data
                                from PySide6.QtCore import QMimeData
                                drag = QDrag(self.printfile_list)
                                mimeData = QMimeData()
                                mimeData.setText(f"file;{uid};{file_path};{file_color};{file_name}")
                                drag.setMimeData(mimeData)
                                drag.exec(Qt.CopyAction)
                        return True
        return super().eventFilter(source, event)

    def refresh_print_file_list(self):
        self.printfile_list.clear()
        for pf in self.main_widget.print_setup_tab.get_print_files():
            item = QListWidgetItem(pf.name)
            item.setBackground(QColor(pf.color))
            item.setForeground(QColor(get_contrast_text_color(pf.color)))
            item.setData(Qt.UserRole, (pf.uid, pf.csv_file, pf.color, pf.name))
            self.printfile_list.addItem(item)

    def refresh_running_list(self):
        self.running_list.clear()
        print_files = self.main_widget.print_setup_tab.get_print_files()
        pf_dict = {pf.uid: pf for pf in print_files}
        for well, assignments in self.plate_layout_widget.associations.items():
            for uid, info in assignments.items():
                if uid in pf_dict:
                    name = pf_dict[uid].name
                    color = pf_dict[uid].color
                else:
                    name = info.get("name", uid)
                    color = info.get("color", "#FFFFFF")
                item_text = f"{well}: {name}"
                item = QListWidgetItem(item_text)
                item.setBackground(QColor(color))
                self.running_list.addItem(item)
        
    def get_assigned_printfiles(self):
        assigned = {}
        print_files = self.main_widget.print_setup_tab.get_print_files()
        pf_dict = {pf.uid: pf for pf in print_files}
        for well, assignments in self.plate_layout_widget.associations.items():
            for uid, info in assignments.items():
                pf_obj = pf_dict.get(uid, None)
                if pf_obj is not None:
                    if well not in assigned:
                        assigned[well] = []
                    assigned[well].append(pf_obj)
        return assigned


############################################################################################################
############################### Print Setup Helpers ########################################################
############################################################################################################
class ColorItemDelegate(QStyledItemDelegate):
    def paint(self, painter, option, index):
        bg = index.data(Qt.BackgroundRole)
        if not bg:
            bg = QColor("white")
        rect = option.rect
        painter.fillRect(rect, bg)
        if option.state & QStyle.State_Selected:
            pen = QPen(Qt.black, 2)
            painter.setPen(pen)
            painter.drawRect(rect.adjusted(1, 1, -1, -1))
        elif option.state & QStyle.State_MouseOver:
            pen = QPen(Qt.gray, 2)
            painter.setPen(pen)
            painter.drawRect(rect.adjusted(1, 1, -1, -1))
        painter.setPen(Qt.black)
        text = index.data(Qt.DisplayRole)
        painter.drawText(rect, Qt.AlignCenter, text)


class WellPreviewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.print_file = None  
        self.dragging = None    
        self.drag_offset = QPointF(0, 0)
        self.well_diameter_physical = 5.0  
        self.well_height_physical = 15.0   

    def resizeEvent(self, event):
        # Recalculate the waypoint's bounding box when the widget is resized.
        if self.print_file and self.print_file.waypoints:
            xs = [pt['x'] for pt in self.print_file.waypoints]
            ys = [pt['y'] for pt in self.print_file.waypoints]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            range_x = max_x - min_x
            range_y = max_y - min_y

            margin = 10
            # The well is drawn in the left half of the widget.
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            radius = min(area_width, area_height) / 2
            drawn_diameter = 2 * radius

            # Compute new bounding box dimensions based on the current scale.
            new_width = (range_x / self.well_diameter_physical) * drawn_diameter
            new_height = (range_y / self.well_diameter_physical) * drawn_diameter
            new_width = max(new_width, 10)
            new_height = max(new_height, 10)
            self.print_file.bbox_size = QSizeF(new_width, new_height)

            # Compute the center of the waypoint data and update its offset.
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            scale = drawn_diameter / self.well_diameter_physical
            self.print_file.bbox_offset = QPointF(center_x * scale, center_y * scale)
        
        # Call the base class implementation.
        super().resizeEvent(event)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 10
        area_width = (self.width() // 2) - 2 * margin
        area_height = self.height() - 2 * margin
        well_center = QPointF(margin + area_width/2, margin + area_height/2)
        radius = min(area_width, area_height) / 2
        drawn_diameter = 2 * radius

        # Draw the well circle.
        painter.setPen(QPen(Qt.black, 2))
        painter.drawEllipse(well_center, radius, radius)

        if self.print_file:
            # Use the updated bbox_offset and bbox_size computed in resizeEvent.
            bbox_center = well_center + self.print_file.bbox_offset
            bbox_size = self.print_file.bbox_size
            bbox_rect = QRectF(bbox_center.x() - bbox_size.width()/2,
                               bbox_center.y() - bbox_size.height()/2,
                               bbox_size.width(), bbox_size.height())
            half_diag = math.sqrt((bbox_size.width()/2)**2 + (bbox_size.height()/2)**2)
            dist = math.hypot(bbox_center.x()-well_center.x(), bbox_center.y()-well_center.y())
            pen_color = Qt.green if dist + half_diag <= radius else Qt.red
            painter.setPen(QPen(pen_color, 2))
            painter.drawRect(bbox_rect)

            # Draw waypoint path if available.
            if self.print_file.waypoints:
                painter.setPen(QPen(Qt.blue, 2))
                xs = [pt['x'] for pt in self.print_file.waypoints]
                ys = [pt['y'] for pt in self.print_file.waypoints]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                range_x = max_x - min_x if (max_x - min_x) != 0 else 1
                range_y = max_y - min_y if (max_y - min_y) != 0 else 1
                mapped_points = []
                for pt in self.print_file.waypoints:
                    norm_x = (pt['x'] - min_x) / range_x
                    norm_y = (pt['y'] - min_y) / range_y
                    x = bbox_rect.left() + norm_x * bbox_rect.width()
                    y = bbox_rect.top() + norm_y * bbox_rect.height()
                    mapped_points.append(QPointF(x, y))
                for i in range(len(mapped_points)-1):
                    painter.drawLine(mapped_points[i], mapped_points[i+1])

        # Draw the side view.
        side_rect = QRectF(self.width()//2 + margin, margin,
                           (self.width()//2) - 2 * margin, self.height() - 2 * margin)
        painter.setPen(QPen(Qt.black, 2))
        painter.drawRect(side_rect)
        if self.print_file:
            if self.print_file.floor_offset is None:
                self.print_file.floor_offset = side_rect.bottom()
            floor_y = self.print_file.floor_offset
            if self.print_file.waypoints:
                z_vals = [pt['z'] for pt in self.print_file.waypoints]
                z_range = max(z_vals) - min(z_vals)
            else:
                z_range = self.well_height_physical
            scale = side_rect.height() / self.well_height_physical
            top_y = floor_y - (z_range * scale)
            painter.setPen(QPen(Qt.black, 2))
            painter.drawLine(side_rect.left(), floor_y, side_rect.right(), floor_y)
            painter.drawLine(side_rect.left(), top_y, side_rect.right(), top_y)

    def mousePressEvent(self, event):
        pos = QPointF(event.position())
        margin = 10
        if not self.print_file:
            return
        if pos.x() < self.width()//2:
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            well_center = QPointF(margin + area_width/2, margin + area_height/2)
            bbox_center = well_center + self.print_file.bbox_offset
            bbox_size = self.print_file.bbox_size
            bbox_rect = QRectF(bbox_center.x()-bbox_size.width()/2,
                               bbox_center.y()-bbox_size.height()/2,
                               bbox_size.width(), bbox_size.height())
            if bbox_rect.contains(pos):
                self.dragging = "bbox"
                self.drag_offset = pos - bbox_rect.topLeft()
        else:
            if self.print_file.floor_offset is not None and abs(pos.y() - self.print_file.floor_offset) < 5:
                self.dragging = "floor"
                self.drag_offset = pos.y() - self.print_file.floor_offset

    def mouseMoveEvent(self, event):
        pos = QPointF(event.position())
        margin = 10
        if self.dragging == "bbox" and self.print_file:
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            well_center = QPointF(margin + area_width/2, margin + area_height/2)
            new_top_left = pos - self.drag_offset
            new_bbox_center = new_top_left + QPointF(self.print_file.bbox_size.width()/2,
                                                     self.print_file.bbox_size.height()/2)
            half_diag = math.sqrt((self.print_file.bbox_size.width()/2)**2 + (self.print_file.bbox_size.height()/2)**2)
            max_allowed = (min(area_width, area_height)/2) - half_diag
            delta = new_bbox_center - well_center
            dist = math.hypot(delta.x(), delta.y())
            if dist > max_allowed:
                factor = max_allowed / dist if dist != 0 else 1
                delta = delta * factor
                new_bbox_center = well_center + delta
            self.print_file.bbox_offset = new_bbox_center - well_center
            self.update()
        elif self.dragging == "floor" and self.print_file:
            side_rect = QRectF(self.width()//2 + margin, margin,
                               (self.width()//2) - 2 * margin, self.height() - 2 * margin)
            new_floor = pos.y() - self.drag_offset
            if new_floor < side_rect.top():
                new_floor = side_rect.top()
            if new_floor > side_rect.bottom():
                new_floor = side_rect.bottom()
            self.print_file.floor_offset = new_floor
            self.update()

    def mouseReleaseEvent(self, event):
        self.dragging = None

class WellPreviewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.print_file = None  
        self.dragging = None    
        self.drag_offset = QPointF(0, 0)
        self.well_diameter_physical = 5.0  
        self.well_height_physical = 15.0   

    def update_physical_offset(self):
        """Calculate the physical (x, y, z) offset from the well center,
        accounting for the current drawn scale in both the well view and side view.
        The result is stored in self.print_file.physical_offset as a tuple (x, y, z)."""
        if not self.print_file:
            return
        margin = 10
        # --- Compute for the well (left) view ---
        area_width = (self.width() // 2) - 2 * margin
        area_height = self.height() - 2 * margin
        well_center = QPointF(margin + area_width / 2, margin + area_height / 2)
        radius = min(area_width, area_height) / 2
        drawn_diameter = 2 * radius
        # This scale converts drawn pixels into physical units.
        scale_well = drawn_diameter / self.well_diameter_physical

        # The bbox_offset is stored in drawn pixels relative to well_center.
        physical_x = self.print_file.bbox_offset.x() / scale_well
        physical_y = self.print_file.bbox_offset.y() / scale_well

        # --- Compute for the side view (z) ---
        side_rect = QRectF(self.width() // 2 + margin, margin,
                           (self.width() // 2) - 2 * margin, self.height() - 2 * margin)
        scale_side = side_rect.height() / self.well_height_physical
        # Use the current floor_offset; if not set, default to the bottom of the side view.
        floor_offset = self.print_file.floor_offset if self.print_file.floor_offset is not None else side_rect.bottom()
        # Compute the physical z position (distance from the bottom) by converting drawn pixels.
        physical_z = (side_rect.bottom() - floor_offset) / scale_side
        # The well center in z (vertical) is at half the physical well height.
        physical_z_offset = physical_z - (self.well_height_physical / 2)

        # Store the full (x, y, z) offset in the print file.
        self.print_file.offset = (physical_x, physical_y, physical_z_offset)

    def resizeEvent(self, event):
        # When the widget resizes, recalc the drawn waypoint based on the new scale.
        if self.print_file and self.print_file.waypoints:
            xs = [pt['x'] for pt in self.print_file.waypoints]
            ys = [pt['y'] for pt in self.print_file.waypoints]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            range_x = max_x - min_x
            range_y = max_y - min_y

            margin = 10
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            radius = min(area_width, area_height) / 2
            drawn_diameter = 2 * radius

            # Recalculate bbox size based on the current drawn well diameter.
            new_width = (range_x / self.well_diameter_physical) * drawn_diameter
            new_height = (range_y / self.well_diameter_physical) * drawn_diameter
            new_width = max(new_width, 10)
            new_height = max(new_height, 10)
            self.print_file.bbox_size = QSizeF(new_width, new_height)

            # Recalculate bbox offset based on the center of the waypoint data.
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            scale = drawn_diameter / self.well_diameter_physical
            self.print_file.bbox_offset = QPointF(center_x * scale, center_y * scale)
        
        # Update the physical offset after recalculating drawn values.
        super().resizeEvent(event)
        self.update_physical_offset()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 10
        # --- Draw the well (left half) ---
        area_width = (self.width() // 2) - 2 * margin
        area_height = self.height() - 2 * margin
        well_center = QPointF(margin + area_width / 2, margin + area_height / 2)
        radius = min(area_width, area_height) / 2
        drawn_diameter = 2 * radius

        painter.setPen(QPen(Qt.black, 2))
        painter.drawEllipse(well_center, radius, radius)

        if self.print_file:
            bbox_center = well_center + self.print_file.bbox_offset
            bbox_size = self.print_file.bbox_size
            bbox_rect = QRectF(bbox_center.x() - bbox_size.width() / 2,
                               bbox_center.y() - bbox_size.height() / 2,
                               bbox_size.width(), bbox_size.height())
            half_diag = math.sqrt((bbox_size.width() / 2) ** 2 + (bbox_size.height() / 2) ** 2)
            dist = math.hypot(bbox_center.x() - well_center.x(), bbox_center.y() - well_center.y())
            pen_color = Qt.green if dist + half_diag <= radius else Qt.red
            painter.setPen(QPen(pen_color, 2))
            painter.drawRect(bbox_rect)

            if self.print_file.waypoints:
                painter.setPen(QPen(Qt.blue, 2))
                xs = [pt['x'] for pt in self.print_file.waypoints]
                ys = [pt['y'] for pt in self.print_file.waypoints]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                range_x = max_x - min_x if (max_x - min_x) != 0 else 1
                range_y = max_y - min_y if (max_y - min_y) != 0 else 1
                mapped_points = []
                for pt in self.print_file.waypoints:
                    norm_x = (pt['x'] - min_x) / range_x
                    norm_y = (pt['y'] - min_y) / range_y
                    x = bbox_rect.left() + norm_x * bbox_rect.width()
                    y = bbox_rect.top() + norm_y * bbox_rect.height()
                    mapped_points.append(QPointF(x, y))
                for i in range(len(mapped_points) - 1):
                    painter.drawLine(mapped_points[i], mapped_points[i + 1])

        # --- Draw the side view ---
        side_rect = QRectF(self.width() // 2 + margin, margin,
                           (self.width() // 2) - 2 * margin, self.height() - 2 * margin)
        painter.setPen(QPen(Qt.black, 2))
        painter.drawRect(side_rect)
        if self.print_file:
            if self.print_file.floor_offset is None:
                self.print_file.floor_offset = side_rect.bottom()
            floor_y = self.print_file.floor_offset
            if self.print_file.waypoints:
                z_vals = [pt['z'] for pt in self.print_file.waypoints]
                z_range = max(z_vals) - min(z_vals)
            else:
                z_range = self.well_height_physical
            scale_side = side_rect.height() / self.well_height_physical
            top_y = floor_y - (z_range * scale_side)
            painter.setPen(QPen(Qt.black, 2))
            painter.drawLine(side_rect.left(), floor_y, side_rect.right(), floor_y)
            painter.drawLine(side_rect.left(), top_y, side_rect.right(), top_y)

    def mousePressEvent(self, event):
        pos = QPointF(event.position())
        margin = 10
        if not self.print_file:
            return
        # Check if the click is in the left (well) view.
        if pos.x() < self.width() // 2:
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            well_center = QPointF(margin + area_width / 2, margin + area_height / 2)
            bbox_center = well_center + self.print_file.bbox_offset
            bbox_size = self.print_file.bbox_size
            bbox_rect = QRectF(bbox_center.x() - bbox_size.width() / 2,
                               bbox_center.y() - bbox_size.height() / 2,
                               bbox_size.width(), bbox_size.height())
            if bbox_rect.contains(pos):
                self.dragging = "bbox"
                self.drag_offset = pos - bbox_rect.topLeft()
        else:
            # Check if the click is near the floor line in the side view.
            if self.print_file.floor_offset is not None and abs(pos.y() - self.print_file.floor_offset) < 5:
                self.dragging = "floor"
                self.drag_offset = pos.y() - self.print_file.floor_offset

    def mouseMoveEvent(self, event):
        pos = QPointF(event.position())
        margin = 10
        if self.dragging == "bbox" and self.print_file:
            area_width = (self.width() // 2) - 2 * margin
            area_height = self.height() - 2 * margin
            well_center = QPointF(margin + area_width / 2, margin + area_height / 2)
            new_top_left = pos - self.drag_offset
            new_bbox_center = new_top_left + QPointF(self.print_file.bbox_size.width() / 2,
                                                      self.print_file.bbox_size.height() / 2)
            half_diag = math.sqrt((self.print_file.bbox_size.width() / 2) ** 2 +
                                  (self.print_file.bbox_size.height() / 2) ** 2)
            max_allowed = (min(area_width, area_height) / 2) - half_diag
            delta = new_bbox_center - well_center
            dist = math.hypot(delta.x(), delta.y())
            if dist > max_allowed:
                factor = max_allowed / dist if dist != 0 else 1
                delta = delta * factor
                new_bbox_center = well_center + delta
            self.print_file.bbox_offset = new_bbox_center - well_center
            # Update the physical x,y offset.
            self.update_physical_offset()
            self.update()
        elif self.dragging == "floor" and self.print_file:
            side_rect = QRectF(self.width() // 2 + margin, margin,
                               (self.width() // 2) - 2 * margin, self.height() - 2 * margin)
            new_floor = pos.y() - self.drag_offset
            if new_floor < side_rect.top():
                new_floor = side_rect.top()
            if new_floor > side_rect.bottom():
                new_floor = side_rect.bottom()
            self.print_file.floor_offset = new_floor
            # Update the physical z offset.
            self.update_physical_offset()
            self.update()

    def mouseReleaseEvent(self, event):
        self.dragging = None



class IsometricPreviewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.print_file = None
        self.waypoints = []         # List of (x, y, z, ...)
        self.pump_states_all = []   # Each state is a list of 3 effective flow values
        self.pump_colors = []       # Expected base colors [(1,0,0), (0,1,0), (0,0,1)]
        self.max_flow_rates = [100.0, 100.0, 100.0]
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.zoom = 1.0
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.last_mouse_pos = None
        self._right_press_pos = None
        self._right_dragging = False
        self.focus_layer_index = 0

        self.layer_indices = []  

        self.setContextMenuPolicy(Qt.NoContextMenu)
        self.setMouseTracking(True)

    def set_toolpath(self, waypoints, pump_states_all, pump_colors, max_flow_rates):
        self.waypoints = waypoints
        self.pump_states_all = pump_states_all
        self.pump_colors = pump_colors
        self.max_flow_rates = max_flow_rates
        self.layer_indices = sorted(set(round(wp[2], 5) for wp in self.waypoints))
        self.focus_layer_index = 0
        self.fit_to_view()
        self.update()

    def set_focus_layer(self, index):
        self.focus_layer_index = index
        self.update()

    def raw_transform_point(self, x, y, z):
        rx = self.rotation_x
        ry = self.rotation_y
        x1 = x
        y1 = y * math.cos(rx) - z * math.sin(rx)
        z1 = y * math.sin(rx) + z * math.cos(rx)
        x2 = x1 * math.cos(ry) + z1 * math.sin(ry)
        y2 = y1
        return x2, y2

    def transform_point(self, x, y, z):
        tx, ty = self.raw_transform_point(x, y, z)
        proj_x = tx * self.zoom + self.offset_x
        proj_y = ty * self.zoom + self.offset_y
        return proj_x, proj_y

    def fit_to_view(self):
        if not self.waypoints or self.width() <= 0 or self.height() <= 0:
            return
        projected = [self.raw_transform_point(wp['x'], wp['y'], wp['z']) for wp in self.waypoints]
        xs = [pt[0] for pt in projected]
        ys = [pt[1] for pt in projected]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        bbox_width = max_x - min_x
        bbox_height = max_y - min_y
        if bbox_width == 0 or bbox_height == 0:
            return
        margin = 0.9
        scale_x = (self.width() * margin) / bbox_width
        scale_y = (self.height() * margin) / bbox_height
        self.zoom = min(scale_x, scale_y)
        self.offset_x = (self.width() - bbox_width * self.zoom) / 2 - min_x * self.zoom
        self.offset_y = (self.height() - bbox_height * self.zoom) / 2 - min_y * self.zoom

    def get_segment_color(self, state):
        if not self.max_flow_rates or len(self.max_flow_rates) < 3:
            return QColor(255, 255, 255)
        try:
            r = int(state[0] / self.max_flow_rates[0] * 255) if self.max_flow_rates[0] != 0 else 0
            g = int(state[1] / self.max_flow_rates[1] * 255) if self.max_flow_rates[1] != 0 else 0
            b = int(state[2] / self.max_flow_rates[2] * 255) if self.max_flow_rates[2] != 0 else 0
        except Exception:
            r = g = b = 0
        r = min(max(r, 0), 255)
        g = min(max(g, 0), 255)
        b = min(max(b, 0), 255)
        return QColor(r, g, b)

    def paintEvent(self, event):
        alpha_factor = 0.2
        if self.print_file is None:
            return
        self.waypoints = self.print_file.waypoints
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        focus_z = None
        if self.layer_indices:
            focus_z = self.layer_indices[self.focus_layer_index]
        tol = 1e-3
        if self.waypoints:
            # waypoints is a list of dicts with keys 'x', 'y', 'z'
            transformed_points = [self.transform_point(wp['x'], wp['y'], wp['z']) for wp in self.waypoints]
            for i in range(len(transformed_points)-1):
                p1 = transformed_points[i]
                p2 = transformed_points[i+1]
                base_color = self.get_segment_color(self.pump_states_all[i]) if i < len(self.pump_states_all) else QColor(0, 0, 0)
                z_val = self.waypoints[i]['z']
                alpha = 255 if (focus_z is not None and abs(z_val - focus_z) < tol) else int(255 * alpha_factor)
                color = QColor(base_color)
                color.setAlpha(alpha)
                pen = QPen(color)
                pen.setWidth(2)
                painter.setPen(pen)
                painter.drawLine(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))
            for i, pt in enumerate(transformed_points):
                base_color = self.get_segment_color(self.pump_states_all[i]) if i < len(self.pump_states_all) else QColor(0, 0, 0)
                z_val = self.waypoints[i]['z']
                if focus_z is not None and abs(z_val - focus_z) < tol:
                    alpha = 255
                    radius = 4
                else:
                    alpha = 0
                    radius = 2
                color = QColor(base_color)
                color.setAlpha(alpha)
                color_outline = QColor(0, 0, 0, alpha)
                pen = QPen(color_outline)
                painter.setPen(pen)
                painter.setBrush(color)
                painter.drawEllipse(int(pt[0]-radius), int(pt[1]-radius), radius*2, radius*2)
            self.fit_to_view()
                
        else:
            painter.drawText(self.rect(), Qt.AlignCenter, "No waypoints to display")
        self.drawRosette(painter)

    def drawRosette(self, painter):
        painter.save()
        margin = 10
        rosette_size = 80
        center = QPointF(margin + rosette_size/2, self.height()-margin-rosette_size/2)
        radius = rosette_size/2 - 5
        pen = QPen(QColor(0, 0, 0))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius, radius)
        def rosette_vector(dx, dy, dz, length):
            rx = self.rotation_x
            ry = self.rotation_y
            x1 = dx
            y1 = dy * math.cos(rx) - dz * math.sin(rx)
            z1 = dy * math.sin(rx) + dz * math.cos(rx)
            x2 = x1 * math.cos(ry) + z1 * math.sin(ry)
            y2 = y1
            mag = math.hypot(x2, y2)
            if mag == 0:
                return 0, 0
            return (x2/mag*length, y2/mag*length)
        arrow_length = radius - 5
        vx, vy = rosette_vector(1, 0, 0, arrow_length)
        pen.setColor(QColor(255, 0, 0))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x()+vx, center.y()-vy))
        pen.setColor(QColor(0, 255, 0))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x(), center.y()-arrow_length))
        vx, vy = rosette_vector(0, 0, 1, arrow_length)
        pen.setColor(QColor(0, 0, 255))
        painter.setPen(pen)
        painter.drawLine(center, QPointF(center.x()+vx, center.y()-vy))
        painter.restore()

    def mousePressEvent(self, event):
        self.last_mouse_pos = event.position()
        if event.button() == Qt.RightButton:
            self._right_press_pos = event.position()
            self._right_dragging = False

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is None:
            self.last_mouse_pos = event.position()
            return
        delta = event.position() - self.last_mouse_pos
        buttons = event.buttons()
        if buttons & Qt.LeftButton:
            self.offset_x += delta.x()
            self.offset_y += delta.y()
        if buttons & Qt.RightButton:
            if self._right_press_pos is not None:
                if (event.position() - self._right_press_pos).manhattanLength() > 5:
                    self._right_dragging = True
            self.rotation_y += delta.x() * 0.01
            self.rotation_x += delta.y() * 0.01
        self.last_mouse_pos = event.position()
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            if not self._right_dragging:
                self.showContextMenu(event.globalPos())
            self._right_press_pos = None
            self._right_dragging = False
        self.last_mouse_pos = event.position()

    def wheelEvent(self, event):
        zoom_factor = 1.0 + event.angleDelta().y() / 1200.0
        self.zoom *= zoom_factor
        self.update()

    def showContextMenu(self, global_pos):
        menu = QMenu(self)
        front_action = menu.addAction("Front")
        top_action = menu.addAction("Top")
        left_action = menu.addAction("Left")
        right_action = menu.addAction("Right")
        isometric_action = menu.addAction("Isometric")
        bottom_action = menu.addAction("Bottom")
        action = menu.exec_(global_pos)
        if action == front_action:
            self.rotation_x = 0.0; self.rotation_y = 0.0
        elif action == top_action:
            self.rotation_x = -math.pi/2; self.rotation_y = 0.0
        elif action == left_action:
            self.rotation_x = 0.0; self.rotation_y = math.pi/2
        elif action == right_action:
            self.rotation_x = 0.0; self.rotation_y = -math.pi/2
        elif action == isometric_action:
            self.rotation_x = math.radians(-35); self.rotation_y = math.radians(45)
        elif action == bottom_action:
            self.rotation_x = math.pi/2; self.rotation_y = 0.0
        self.fit_to_view()
        self.update()


############################################################################################################
#################################  Ink well Helpers  #######################################################
############################################################################################################
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

