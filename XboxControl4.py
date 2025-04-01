import sys
import os
import json
import time
import threading
from queue import Queue

import qt_core
import pygame


class XboxControllerInterface(threading.Thread):
    """
    Polls the Xbox controller using pygame.
    
    Reads:
      - Buttons (digital): sends an immediate command when pressed.
      - Axes: samples every 0.05 sec and averages over avg_interval seconds.
         • Axes 0 and 1 are linked (sent as a tuple).
         • Axes 2 and 3 are linked (sent as a tuple).
         • Axes 4 and 5 (triggers) are processed independently (remapped from [-1,1] to [0,1]).
      - DPad (hat): sends a command when the state changes.
    
    Mapping is loaded from a JSON file with structure:
      {
         "buttons": { "0": "Button Function", ... },
         "axes":    { "0-1": "Axis Function", "2-3": "Axis Function", "4": "Trigger Function", "5": "Trigger Function" },
         "dpad":    { "up": "DPad Function", "down": "DPad Function", "left": "DPad Function", "right": "DPad Function" }
      }
    """
    def __init__(self, command_queue, deadzone=0.2, mapping_file="button_mapping.json", avg_interval=0.5):
        super().__init__()
        self.command_queue = command_queue
        self.deadzone = deadzone
        self.mapping_file = mapping_file
        self.avg_interval = avg_interval  # averaging period in seconds
        self.running = False

        # Load mapping from file (or default to empty sections)
        self.load_mapping()

        # Initialize pygame joystick support.
        pygame.init()
        pygame.joystick.init()
        self.joystick = None

    def load_mapping(self):
        try:
            with open(self.mapping_file, "r") as f:
                mapping = json.load(f)
            # Ensure the expected sections exist.
            if "buttons" not in mapping:
                mapping["buttons"] = {}
            if "axes" not in mapping:
                mapping["axes"] = {}
            if "dpad" not in mapping:
                mapping["dpad"] = {}
            self.mapping = mapping
        except Exception as e:
            self.command_queue.put(f"[DEBUG] Mapping file error: {e}")
            self.mapping = {"buttons": {}, "axes": {}, "dpad": {}}

    def save_mapping(self):
        with open(self.mapping_file, "w") as f:
            json.dump(self.mapping, f)

    def update_mapping(self, new_mapping):
        """Update the mapping (from the UI) and save it to the default file."""
        self.mapping = new_mapping
        self.save_mapping()

    def connect(self):
        """Connect to the first available joystick."""
        count = pygame.joystick.get_count()
        self.command_queue.put(f"[DEBUG] Found {count} joystick(s).")
        if count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.command_queue.put(f"[DEBUG] Controller connected: {self.joystick.get_name()}")
            self.command_queue.put(
                f"[DEBUG] Joystick has {self.joystick.get_numbuttons()} buttons, "
                f"{self.joystick.get_numaxes()} axes, {self.joystick.get_numhats()} hat(s)."
            )
        else:
            self.command_queue.put("No controller connected.")

    def keep_alive(self):
        """Dummy keep-alive message."""
        self.command_queue.put("[DEBUG] Sending keep alive to controller.")

    def run(self):
        self.running = True
        self.connect()

        if not self.joystick:
            self.command_queue.put("[DEBUG] No joystick available. Exiting thread.")
            return

        # Initialize per-axis accumulators.
        num_axes = self.joystick.get_numaxes()
        axis_accum = {axis: 0.0 for axis in range(num_axes)}
        axis_count = {axis: 0 for axis in range(num_axes)}
        last_axis_time = time.time()

        # For DPad (hat) state.
        last_hat = (0, 0)

        last_keep_alive = time.time()

        while self.running:
            pygame.event.pump()

            # --- Process Button Presses ---
            for i in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(i):
                    mapped_func = self.mapping.get("buttons", {}).get(str(i), "None")
                    if mapped_func != "None":
                        self.command_queue.put(f"Button {i} pressed -> {mapped_func}")
                        time.sleep(0.2)  # debouncing

            # --- Accumulate Axis Readings ---
            for axis in range(num_axes):
                val = self.joystick.get_axis(axis)
                axis_accum[axis] += val
                axis_count[axis] += 1

            current_time = time.time()
            if current_time - last_axis_time >= self.avg_interval:
                # Process axes in groups:
                groups = [
                    {"name": "0-1", "axes": [0, 1], "type": "axis"},
                    {"name": "2-3", "axes": [2, 3], "type": "axis"},
                    {"name": "4", "axes": [4], "type": "trigger"},
                    {"name": "5", "axes": [5], "type": "trigger"}
                ]
                for group in groups:
                    averages = []
                    for axis in group["axes"]:
                        if axis_count[axis]:
                            avg_val = axis_accum[axis] / axis_count[axis]
                        else:
                            avg_val = 0.0
                        if group["type"] == "trigger":
                            avg_val = (avg_val + 1) / 2  # remap trigger value from [-1,1] to [0,1]
                        averages.append(avg_val)
                    # Decide whether to send a command based on deadzone.
                    send = False
                    if len(averages) == 1:
                        if averages[0] > self.deadzone:
                            send = True
                    else:
                        if any(abs(v) > self.deadzone for v in averages):
                            send = True
                    mapped_func = self.mapping.get("axes", {}).get(group["name"], "None")
                    if mapped_func != "None" and send:
                        if len(averages) == 1:
                            self.command_queue.put(f"Axis {group['name']} avg={averages[0]:.2f} -> {mapped_func}")
                        else:
                            tup = tuple(round(v, 2) for v in averages)
                            self.command_queue.put(f"Axis {group['name']} avg={tup} -> {mapped_func}")
                    # Reset accumulators for this group.
                    for axis in group["axes"]:
                        axis_accum[axis] = 0.0
                        axis_count[axis] = 0
                last_axis_time = current_time

            # --- Process DPad (Hat) Input ---
            if self.joystick.get_numhats() > 0:
                current_hat = self.joystick.get_hat(0)  # (x, y)
                if current_hat != last_hat:
                    dpad_map = self.mapping.get("dpad", {})
                    if current_hat[1] == 1:
                        func = dpad_map.get("up", "None")
                        if func != "None":
                            self.command_queue.put(f"DPad Up -> {func}")
                    elif current_hat[1] == -1:
                        func = dpad_map.get("down", "None")
                        if func != "None":
                            self.command_queue.put(f"DPad Down -> {func}")
                    if current_hat[0] == 1:
                        func = dpad_map.get("right", "None")
                        if func != "None":
                            self.command_queue.put(f"DPad Right -> {func}")
                    elif current_hat[0] == -1:
                        func = dpad_map.get("left", "None")
                        if func != "None":
                            self.command_queue.put(f"DPad Left -> {func}")
                    last_hat = current_hat

            # --- Keep-Alive Every 5 Seconds ---
            if current_time - last_keep_alive > 5.0:
                self.keep_alive()
                last_keep_alive = current_time

            time.sleep(0.05)

        pygame.quit()

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """
    The main PyQt5 window includes:
      - A button to connect the Xbox controller (which auto-loads the default mapping).
      - Mapping groups for:
          • Buttons (digital; one combo per button)
          • Axes (four groups: "0-1" and "2-3" for linked axes, and "4" and "5" for triggers)
          • DPad (hat)
      - Save and Load Mapping buttons (the latter opens a file dialog).
      - An image of an Xbox controller layout.
      - A log list showing messages from the controller thread.
    
    The combo boxes are populated with allowed functions from a JSON file
    (available_functions.json) based on control type:
       • "button"
       • "dpad"
       • "axis"  (for linked axes groups)
       • "trigger" (for independent triggers)
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Xbox Controller Interface")
        self.resize(900, 700)

        # Load allowed functions from JSON.
        self.allowed_functions = self.load_allowed_functions()

        main_layout = QtWidgets.QVBoxLayout(self)

        # --- Top Layout: Controller Image and Control Panel ---
        top_layout = QtWidgets.QHBoxLayout()

        # Controller image.
        self.controller_image = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("xbox_controller.png")
        if pixmap.isNull():
            self.controller_image.setText("Image not found: xbox_controller.png")
            self.controller_image.setAlignment(QtCore.Qt.AlignCenter)
        else:
            self.controller_image.setPixmap(pixmap.scaled(300, 300, QtCore.Qt.KeepAspectRatio))
        top_layout.addWidget(self.controller_image)

        # Control panel.
        control_layout = QtWidgets.QVBoxLayout()

        self.connect_button = QtWidgets.QPushButton("Connect Xbox Controller")
        control_layout.addWidget(self.connect_button)

        # --- Mapping Group: Buttons ---
        self.button_mapping_group = QtWidgets.QGroupBox("Button Mapping")
        button_layout = QtWidgets.QFormLayout()
        self.button_comboboxes = {}
        for i in range(10):
            combo = QtWidgets.QComboBox()
            # Populate using allowed functions for buttons.
            combo.addItems(self.allowed_functions.get("button", ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"]))
            button_layout.addRow(f"Button {i}", combo)
            self.button_comboboxes[i] = combo
        self.button_mapping_group.setLayout(button_layout)
        control_layout.addWidget(self.button_mapping_group)

        # --- Mapping Group: Axes ---
        # We create one combo box per axis group:
        #   "0-1" and "2-3" (linked axes, use allowed "axis" functions)
        #   "4" and "5" (triggers, use allowed "trigger" functions)
        self.axis_mapping_group = QtWidgets.QGroupBox("Axis Mapping")
        axis_layout = QtWidgets.QFormLayout()
        self.axis_comboboxes = {}
        axis_groups = [("0-1", "axis"), ("2-3", "axis"), ("4", "trigger"), ("5", "trigger")]
        for group_name, group_type in axis_groups:
            combo = QtWidgets.QComboBox()
            combo.addItems(self.allowed_functions.get(group_type, ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"]))
            axis_layout.addRow(f"Axis {group_name}", combo)
            self.axis_comboboxes[group_name] = combo
        self.axis_mapping_group.setLayout(axis_layout)
        control_layout.addWidget(self.axis_mapping_group)

        # --- Mapping Group: DPad ---
        self.dpad_mapping_group = QtWidgets.QGroupBox("DPad Mapping")
        dpad_layout = QtWidgets.QFormLayout()
        self.dpad_comboboxes = {}
        for direction in ["up", "down", "left", "right"]:
            combo = QtWidgets.QComboBox()
            combo.addItems(self.allowed_functions.get("dpad", ["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"]))
            dpad_layout.addRow(f"DPad {direction.capitalize()}", combo)
            self.dpad_comboboxes[direction] = combo
        self.dpad_mapping_group.setLayout(dpad_layout)
        control_layout.addWidget(self.dpad_mapping_group)

        # Save and Load mapping buttons.
        self.save_mapping_button = QtWidgets.QPushButton("Save Mapping")
        self.load_mapping_button = QtWidgets.QPushButton("Load Mapping")
        control_layout.addWidget(self.save_mapping_button)
        control_layout.addWidget(self.load_mapping_button)

        top_layout.addLayout(control_layout)
        main_layout.addLayout(top_layout)

        # --- Log List ---
        self.log_list = QtWidgets.QListWidget()
        main_layout.addWidget(self.log_list)

        self.command_queue = Queue()
        self.controller_thread = None

        # Connect signals.
        self.connect_button.clicked.connect(self.start_controller)
        self.save_mapping_button.clicked.connect(self.save_mapping)
        self.load_mapping_button.clicked.connect(self.load_mapping)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.poll_queue)
        self.timer.start(100)

    def load_allowed_functions(self):
        """Load the allowed functions from available_functions.json."""
        try:
            with open("available_functions.json", "r") as f:
                return json.load(f)
        except Exception as e:
            print("Error loading available functions:", e)
            # Fallback defaults.
            return {
                "button": ["None", "Button Function 1", "Button Function 2", "Button Function 3"],
                "dpad": ["None", "DPad Function 1", "DPad Function 2"],
                "axis": ["None", "Axis Function 1", "Axis Function 2"],
                "trigger": ["None", "Trigger Function 1", "Trigger Function 2"]
            }

    def get_current_mapping(self):
        """
        Build a mapping dictionary from the UI combo boxes.
        Structure:
          {
            "buttons": { "0": "func", ... },
            "axes":    { "0-1": "func", "2-3": "func", "4": "func", "5": "func" },
            "dpad":    { "up": "func", "down": "func", ... }
          }
        Only items not set to "None" are stored.
        """
        mapping = {}

        buttons = {}
        for btn, combo in self.button_comboboxes.items():
            func = combo.currentText()
            if func != "None":
                buttons[str(btn)] = func
        mapping["buttons"] = buttons

        axes = {}
        for group, combo in self.axis_comboboxes.items():
            func = combo.currentText()
            if func != "None":
                axes[group] = func
        mapping["axes"] = axes

        dpad = {}
        for direction, combo in self.dpad_comboboxes.items():
            func = combo.currentText()
            if func != "None":
                dpad[direction] = func
        mapping["dpad"] = dpad

        return mapping

    def load_mapping_file(self, filename):
        """Load a mapping from a JSON file and update the UI."""
        try:
            with open(filename, "r") as f:
                mapping = json.load(f)
            # Update button mapping.
            for key, combo in self.button_comboboxes.items():
                func = mapping.get("buttons", {}).get(str(key), "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            # Update axis mapping.
            for key, combo in self.axis_comboboxes.items():
                func = mapping.get("axes", {}).get(key, "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            # Update DPad mapping.
            for key, combo in self.dpad_comboboxes.items():
                func = mapping.get("dpad", {}).get(key, "None")
                index = combo.findText(func)
                combo.setCurrentIndex(index if index != -1 else 0)
            self.log_list.addItem(f"Mapping loaded from {filename}.")
            if self.controller_thread and self.controller_thread.is_alive():
                self.controller_thread.update_mapping(mapping)
        except Exception as e:
            self.log_list.addItem(f"Error loading mapping from {filename}: {e}")

    def start_controller(self):
        """On connection, auto-load the default mapping and start the controller thread."""
        default_mapping_file = "button_mapping.json"
        if os.path.exists(default_mapping_file):
            self.load_mapping_file(default_mapping_file)
        else:
            self.log_list.addItem("Default mapping file not found; using UI defaults.")

        if self.controller_thread is None or not self.controller_thread.is_alive():
            mapping = self.get_current_mapping()
            self.controller_thread = XboxControllerInterface(self.command_queue, deadzone=0.2,
                                                             mapping_file=default_mapping_file, avg_interval=0.5)
            self.controller_thread.update_mapping(mapping)
            self.controller_thread.start()
            self.log_list.addItem("Controller thread started.")
        else:
            self.log_list.addItem("Controller already running.")

    def save_mapping(self):
        """Save the current UI mapping to the default mapping file and update the thread."""
        mapping = self.get_current_mapping()
        try:
            with open("button_mapping.json", "w") as f:
                json.dump(mapping, f)
            self.log_list.addItem("Mapping saved to button_mapping.json.")
            if self.controller_thread and self.controller_thread.is_alive():
                self.controller_thread.update_mapping(mapping)
        except Exception as e:
            self.log_list.addItem(f"Error saving mapping: {e}")

    def load_mapping(self):
        """Open a file dialog to select a mapping JSON file and load it."""
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Mapping", "", "JSON Files (*.json)")
        if filename:
            self.load_mapping_file(filename)

    def poll_queue(self):
        """Poll messages from the controller thread and add them to the log list."""
        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.log_list.addItem(message)

    def closeEvent(self, event):
        """Ensure the controller thread stops when closing the window."""
        if self.controller_thread and self.controller_thread.is_alive():
            self.controller_thread.stop()
            self.controller_thread.join()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())
