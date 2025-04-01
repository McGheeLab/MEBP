# c_controller_layout.py
# Author:   Suraj Bharaj
# Created:  2025-03-06

from qt_core import *
import json
import threading
from queue import Queue
import time
import pygame
import os


# --- Aspect Ratio Preserving QLabel ---
class AspectRatioPixmapLabel(QLabel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pixmap = None
        # Allow the label to expand or shrink freely
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(1, 1)

    def setPixmap(self, pixmap):
        self._pixmap = pixmap
        self.update_scaled_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_scaled_pixmap()

    def update_scaled_pixmap(self):
        if self._pixmap:
            # Scale the pixmap to the current label size while keeping the aspect ratio
            scaled = self._pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            super().setPixmap(scaled)

# --- Main Widget ---
class ControllerLayoutWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.allowed_functions = self.load_allowed_functions()
        self.setup_ui()
        self.controller_interface = None
        self.command_queue = Queue()

    def setup_ui(self):
        main_layout = QVBoxLayout(self)

        # --- Top Layout ---
        top_layout = QHBoxLayout()

        # LEFT COLUMN: Controller Image + Log List
        left_column = QVBoxLayout()

        # Controller Image (Aspect-ratio-safe)
        self.controller_image = AspectRatioPixmapLabel()
        self.controller_image.setStyleSheet("QLabel { background-color: black; }")  # Set background color to black
        pixmap = QPixmap("xbox_controller_new.png")  # Ensure the image exists at this path
        if pixmap.isNull():
            self.controller_image.setText("Image not found: xbox_controller_new.png")
        else:
            self.controller_image.setPixmap(pixmap)
        self.controller_image.setAlignment(Qt.AlignCenter)
        left_column.addWidget(self.controller_image, 2)

        # Log List for Messages
        self.log_list = QListWidget()
        left_column.addWidget(self.log_list, 1)

        # Add the left column to the top layout
        top_layout.addLayout(left_column, 1)

        # RIGHT COLUMN: Control Panel Layout
        control_layout = QVBoxLayout()

        # Connect Button
        self.connect_button = QPushButton("Connect Xbox Controller")
        control_layout.addWidget(self.connect_button)

        # Scroll Area for Mappings
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # --- Mapping Group: Button Mapping ---
        self.button_mapping_group = QGroupBox("Button Mapping")
        button_layout = QFormLayout()
        self.button_comboboxes = {}
        btn_options = self.allowed_functions.get("button", ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"])
        for i in range(10):
            combo = QComboBox()
            combo.addItems(btn_options)
            button_layout.addRow(f"Button {i}", combo)
            self.button_comboboxes[i] = combo
        self.button_mapping_group.setLayout(button_layout)
        scroll_layout.addWidget(self.button_mapping_group)

        # --- Mapping Group: Axis Mapping ---
        self.axis_mapping_group = QGroupBox("Axis Mapping")
        axis_layout = QFormLayout()
        self.axis_comboboxes = {}
        axis_groups = [("0-1", "axis"), ("2-3", "axis"), ("4", "trigger"), ("5", "trigger")]
        for group_name, group_type in axis_groups:
            combo = QComboBox()
            combo.addItems(self.allowed_functions.get(group_type, ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"]))
            axis_layout.addRow(f"Axis {group_name}", combo)
            self.axis_comboboxes[group_name] = combo
        self.axis_mapping_group.setLayout(axis_layout)
        scroll_layout.addWidget(self.axis_mapping_group)

        # --- Mapping Group: DPad Mapping ---
        self.dpad_mapping_group = QGroupBox("DPad Mapping")
        dpad_layout = QFormLayout()
        self.dpad_comboboxes = {}
        for direction in ["up", "down", "left", "right"]:
            combo = QComboBox()
            combo.addItems(self.allowed_functions.get("dpad", ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"]))
            dpad_layout.addRow(f"DPad {direction.capitalize()}", combo)
            self.dpad_comboboxes[direction] = combo
        self.dpad_mapping_group.setLayout(dpad_layout)
        scroll_layout.addWidget(self.dpad_mapping_group)

        scroll_area.setWidget(scroll_content)
        control_layout.addWidget(scroll_area)

        # Save and Load Mapping buttons
        self.save_mapping_button = QPushButton("Save Mapping")
        self.load_mapping_button = QPushButton("Load Mapping")
        control_layout.addWidget(self.save_mapping_button)
        control_layout.addWidget(self.load_mapping_button)

        # Wrap the control layout in a container widget and set a maximum width
        right_container = QWidget()
        right_container.setLayout(control_layout)
        right_container.setMaximumWidth(450)  # Adjust this value as needed

        # Add the right container to the top layout
        top_layout.addWidget(right_container, 2)

        # Add the top layout to the main layout
        main_layout.addLayout(top_layout)

        # Timer to poll the command queue every 100ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_queue)
        self.timer.start(100)

        # Connect signals to functions
        self.connect_button.clicked.connect(self.start_controller)
        self.save_mapping_button.clicked.connect(self.save_mapping)
        self.load_mapping_button.clicked.connect(self.load_mapping)

    def load_allowed_functions(self):
        try:
            with open("available_functions.json", "r") as f:
                return json.load(f)
        except Exception as e:
            print("Error loading available functions:", e)
            return {
                "button": ["None", "Button Function 1", "Button Function 2", "Button Function 3"],
                "dpad": ["None", "DPad Function 1", "DPad Function 2"],
                "axis": ["None", "Axis Function 1", "Axis Function 2"],
                "trigger": ["None", "Trigger Function 1", "Trigger Function 2"]
            }

    def get_current_mapping(self):
        mapping = {
            "buttons": {str(k): cb.currentText() for k, cb in self.button_comboboxes.items() if cb.currentText() != "None"},
            "axes": {k: cb.currentText() for k, cb in self.axis_comboboxes.items() if cb.currentText() != "None"},
            "dpad": {k: cb.currentText() for k, cb in self.dpad_comboboxes.items() if cb.currentText() != "None"}
        }
        return mapping

    def load_mapping_file(self, filename):
        try:
            with open(filename, "r") as f:
                mapping = json.load(f)
            for key, combo in self.button_comboboxes.items():
                func = mapping.get("buttons", {}).get(str(key), "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            for key, combo in self.axis_comboboxes.items():
                func = mapping.get("axes", {}).get(key, "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            for key, combo in self.dpad_comboboxes.items():
                func = mapping.get("dpad", {}).get(key, "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            self.log_list.addItem(f"Mapping loaded from {filename}.")
            if self.controller_interface and self.controller_interface.is_alive():
                self.controller_interface.update_mapping(mapping)
        except Exception as e:
            self.log_list.addItem(f"Error loading mapping from {filename}: {e}")

    def start_controller(self):
        default_mapping_file = "current_button_mapping.json"
        if os.path.exists(default_mapping_file):
            self.load_mapping_file(default_mapping_file)
        else:
            self.log_list.addItem("Default mapping file not found; using UI defaults.")

        if self.controller_interface is None or not self.controller_interface.is_alive():
            mapping = self.get_current_mapping()
            self.controller_interface = XboxControllerInterface(
                self.command_queue,
                deadzone=0.2,
                mapping_file=default_mapping_file,
                avg_interval=0.5
            )
            self.controller_interface.update_mapping(mapping)
            self.controller_interface.start()
            self.log_list.addItem("Controller thread started.")
        else:
            self.log_list.addItem("Controller already running.")

    def save_mapping(self):
        mapping = self.get_current_mapping()
        try:
            with open("current_button_mapping.json", "w") as f:
                json.dump(mapping, f, indent=4)
            self.log_list.addItem("Mapping saved to current_button_mapping.json.")
            if self.controller_interface and self.controller_interface.is_alive():
                self.controller_interface.update_mapping(mapping)
        except Exception as e:
            self.log_list.addItem(f"Error saving mapping: {e}")

    def load_mapping(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Mapping", "", "JSON Files (*.json)")
        if filename:
            self.load_mapping_file(filename)

    def poll_queue(self):
        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.log_list.addItem(str(message))

    def closeEvent(self, event):
        if self.controller_interface and self.controller_interface.is_alive():
            self.controller_interface.stop()
            self.controller_interface.join()
        event.accept()
