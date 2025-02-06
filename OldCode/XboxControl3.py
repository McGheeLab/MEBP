import sys
import json
import time
import threading
from queue import Queue

from PyQt5 import QtWidgets, QtCore, QtGui
import pygame


class XboxControllerInterface(threading.Thread):
    """
    This class polls the Xbox controller using pygame.
    Button events are mapped to functions (from a JSON mapping).
    Joystick/trigger (axis) values are reported only if they exceed the deadzone.
    Trigger axes (4 and 5) are remapped from [-1,1] to [0,1].
    A keep-alive signal is sent every 5 seconds.
    """
    def __init__(self, command_queue, deadzone=0.2, mapping_file="button_mapping.json"):
        super().__init__()
        self.command_queue = command_queue
        self.deadzone = deadzone
        self.mapping_file = mapping_file
        self.running = False
        self.button_mapping = {}
        self.load_mapping()

        # Initialize pygame for controller support.
        pygame.init()
        pygame.joystick.init()
        self.joystick = None

    def load_mapping(self):
        try:
            with open(self.mapping_file, "r") as f:
                self.button_mapping = json.load(f)
        except Exception as e:
            # No mapping file exists or error reading file? Start with an empty mapping.
            print("Mapping file not found or error reading file:", e)
            self.button_mapping = {}

    def save_mapping(self):
        with open(self.mapping_file, "w") as f:
            json.dump(self.button_mapping, f)

    def update_mapping(self, new_mapping):
        """Update the mapping (from the UI) and save it."""
        self.button_mapping = new_mapping
        self.save_mapping()

    def connect(self):
        """Connect to the first available joystick."""
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.command_queue.put(
                f"Controller connected: {self.joystick.get_name()}"
            )
        else:
            self.command_queue.put("No controller connected.")

    def keep_alive(self):
        """
        Dummy function to simulate a keep-alive to prevent the controller from turning off.
        In a real implementation you might send a vibration command or similar.
        """
        self.command_queue.put("Sending keep alive to controller.")

    def run(self):
        self.running = True
        # Try to connect to a controller.
        self.connect()
        last_keep_alive = time.time()
        while self.running:
            # Pump pygame events to update joystick state.
            pygame.event.pump()

            if self.joystick:
                # Check each button.
                for i in range(self.joystick.get_numbuttons()):
                    if self.joystick.get_button(i):
                        # Look up the mapping for this button (if any).
                        mapped_func = self.button_mapping.get(str(i), "None")
                        self.command_queue.put(
                            f"Button {i} pressed: executing {mapped_func}"
                        )
                        # Small pause to help debouncing.
                        time.sleep(0.2)
                # Check each axis (joysticks/triggers)
                for axis in range(self.joystick.get_numaxes()):
                    val = self.joystick.get_axis(axis)
                    # For trigger axes (assumed to be axes 4 and 5), remap from [-1,1] to [0,1]
                    if axis in (4, 5):
                        trigger_val = (val + 1) / 2
                        if trigger_val > self.deadzone:
                            self.command_queue.put(
                                f"Axis {axis} (trigger) moved: {trigger_val:.2f}"
                            )
                    else:
                        if abs(val) > self.deadzone:
                            self.command_queue.put(
                                f"Axis {axis} moved: {val:.2f}"
                            )
            # Every 5 seconds, send a keep-alive.
            if time.time() - last_keep_alive > 60.0:
                self.keep_alive()
                last_keep_alive = time.time()
            time.sleep(0.05)
        pygame.quit()

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """
    This PyQt5 window provides:
      - A button to connect to the Xbox controller,
      - A set of drop-downs to map controller buttons to functions,
      - Buttons to save and load the mapping,
      - An image of the Xbox controller layout,
      - A list widget that shows log messages coming from the controller thread.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Xbox Controller Interface")
        self.resize(800, 600)

        # Main vertical layout.
        main_layout = QtWidgets.QVBoxLayout(self)

        # --- Top layout with the controller image and control buttons ---
        top_layout = QtWidgets.QHBoxLayout()

        # Controller image.
        self.controller_image = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("xbox_controller.jpg")
        if pixmap.isNull():
            self.controller_image.setText("Image not found: xbox_controller.jpg")
            self.controller_image.setAlignment(QtCore.Qt.AlignCenter)
        else:
            # Scale the image to a reasonable size while keeping aspect ratio.
            self.controller_image.setPixmap(pixmap.scaled(300, 300, QtCore.Qt.KeepAspectRatio))
        top_layout.addWidget(self.controller_image)

        # Vertical layout for controls.
        control_layout = QtWidgets.QVBoxLayout()

        # Button to connect to the Xbox controller.
        self.connect_button = QtWidgets.QPushButton("Connect Xbox Controller")
        control_layout.addWidget(self.connect_button)

        # Group for mapping buttons to functions.
        self.mapping_group = QtWidgets.QGroupBox("Button Mapping")
        mapping_layout = QtWidgets.QFormLayout()
        self.button_comboboxes = {}
        # Assume 10 buttons – adjust as needed.
        for i in range(10):
            combo = QtWidgets.QComboBox()
            combo.addItems(["None", "Function 1", "Function 2", "Function 3", "Function 4", "Function 5"])
            mapping_layout.addRow(f"Button {i}", combo)
            self.button_comboboxes[i] = combo
        self.mapping_group.setLayout(mapping_layout)
        control_layout.addWidget(self.mapping_group)

        # Buttons to save and load the mapping.
        self.save_mapping_button = QtWidgets.QPushButton("Save Mapping")
        self.load_mapping_button = QtWidgets.QPushButton("Load Mapping")
        control_layout.addWidget(self.save_mapping_button)
        control_layout.addWidget(self.load_mapping_button)

        # Add the controls to the top layout.
        top_layout.addLayout(control_layout)
        main_layout.addLayout(top_layout)

        # --- Log List ---
        self.log_list = QtWidgets.QListWidget()
        main_layout.addWidget(self.log_list)

        # Create a thread-safe queue for messages from the controller thread.
        self.command_queue = Queue()

        # The controller thread (initially not started).
        self.controller_thread = None

        # Connect signals.
        self.connect_button.clicked.connect(self.start_controller)
        self.save_mapping_button.clicked.connect(self.save_mapping)
        self.load_mapping_button.clicked.connect(self.load_mapping)

        # Timer to poll the command queue every 100 ms.
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.poll_queue)
        self.timer.start(100)

    def get_current_mapping(self):
        """
        Read the mapping from the combo boxes.
        Returns a dictionary mapping button indices (as strings) to function names.
        """
        mapping = {}
        for btn, combo in self.button_comboboxes.items():
            selected = combo.currentText()
            if selected != "None":
                mapping[str(btn)] = selected
        return mapping

    def start_controller(self):
        """
        Start the Xbox controller thread (if not already running) and load the current mapping.
        """
        if self.controller_thread is None or not self.controller_thread.is_alive():
            mapping = self.get_current_mapping()
            self.controller_thread = XboxControllerInterface(self.command_queue, deadzone=0.2)
            self.controller_thread.update_mapping(mapping)
            self.controller_thread.start()
            self.log_list.addItem("Controller thread started.")
        else:
            self.log_list.addItem("Controller already running.")

    def save_mapping(self):
        """
        Save the mapping from the UI. If the controller thread is running,
        update its mapping immediately.
        """
        mapping = self.get_current_mapping()
        if self.controller_thread and self.controller_thread.is_alive():
            self.controller_thread.update_mapping(mapping)
        # Also save to file.
        try:
            with open("button_mapping.json", "w") as f:
                json.dump(mapping, f)
            self.log_list.addItem("Mapping saved.")
        except Exception as e:
            self.log_list.addItem(f"Error saving mapping: {e}")

    def load_mapping(self):
        """
        Load the mapping from file, update the UI's combo boxes, and update the controller thread (if running).
        """
        try:
            with open("button_mapping.json", "r") as f:
                mapping = json.load(f)
            # Update the combo boxes accordingly.
            for btn, combo in self.button_comboboxes.items():
                if str(btn) in mapping:
                    func_name = mapping[str(btn)]
                    index = combo.findText(func_name)
                    if index != -1:
                        combo.setCurrentIndex(index)
                else:
                    combo.setCurrentIndex(0)  # "None"
            self.log_list.addItem("Mapping loaded from file.")
            # If thread is running, update its mapping.
            if self.controller_thread and self.controller_thread.is_alive():
                self.controller_thread.update_mapping(mapping)
        except Exception as e:
            self.log_list.addItem(f"Error loading mapping: {e}")

    def poll_queue(self):
        """Poll the command queue and append any messages to the log list."""
        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.log_list.addItem(message)

    def closeEvent(self, event):
        """Ensure that the controller thread stops on exit."""
        if self.controller_thread and self.controller_thread.is_alive():
            self.controller_thread.stop()
            self.controller_thread.join()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())
