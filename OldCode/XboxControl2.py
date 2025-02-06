import sys
import json
import time
import threading
from queue import Queue

from PyQt5 import QtWidgets, QtCore
import pygame


class XboxControllerInterface(threading.Thread):
    """
    This class polls the Xbox controller using pygame.
    Button events are mapped to functions (as strings) per a JSON mapping.
    Joystick/trigger (axis) values are only reported if they exceed the deadzone.
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
            # No mapping file exists? Start with an empty mapping.
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
                # Check each axis (joysticks, triggers) and report only if outside deadzone.
                for axis in range(self.joystick.get_numaxes()):
                    val = self.joystick.get_axis(axis)
                    if abs(val) > self.deadzone:
                        self.command_queue.put(
                            f"Axis {axis} moved: {val:.2f}"
                        )
            # Every 5 seconds, send a keep-alive.
            if time.time() - last_keep_alive > 5.0:
                self.keep_alive()
                last_keep_alive = time.time()
            time.sleep(0.05)
        pygame.quit()

    def stop(self):
        self.running = False


class MainWindow(QtWidgets.QWidget):
    """
    This PyQt5 window provides:
      - A button to start the controller (which also connects it),
      - A set of dropdowns (combo boxes) to map controller buttons to functions,
      - A button to save the mapping, and
      - A list widget that shows log messages coming from the controller thread.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Xbox Controller Interface")
        self.resize(600, 400)

        # Set up the main layout.
        main_layout = QtWidgets.QVBoxLayout(self)

        # Button to connect to the Xbox controller.
        self.connect_button = QtWidgets.QPushButton("Connect Xbox Controller")
        main_layout.addWidget(self.connect_button)

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
        main_layout.addWidget(self.mapping_group)

        # Button to save the mapping.
        self.save_mapping_button = QtWidgets.QPushButton("Save Mapping")
        main_layout.addWidget(self.save_mapping_button)

        # A list widget to show the log of commands from the controller.
        self.log_list = QtWidgets.QListWidget()
        main_layout.addWidget(self.log_list)

        # Create a thread-safe queue for messages from the controller thread.
        self.command_queue = Queue()

        # The controller thread (initially not started).
        self.controller_thread = None

        # Connect button signals.
        self.connect_button.clicked.connect(self.start_controller)
        self.save_mapping_button.clicked.connect(self.save_mapping)

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
        self.log_list.addItem("Mapping saved.")

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
