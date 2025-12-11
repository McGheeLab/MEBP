# stage_monitor_widget.py

from qt_core import *

class StageMonitorWidget(QWidget):
    def __init__(self, app_controller, parent=None):
        super().__init__(parent)
        self.app_controller = app_controller
        self.processor = app_controller.processor
        self.init_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # --- Control Toggles (default off) ---
        control_layout = QHBoxLayout()
        self.zp_toggle = QCheckBox("ZP Stage")
        self.zp_toggle.setChecked(False)  # Off at startup
        self.xy_toggle = QCheckBox("XY Stage")
        self.xy_toggle.setChecked(False)
        self.xbox_toggle = QCheckBox("Xbox Controller")
        self.xbox_toggle.setChecked(False)
        # When toggled, send control commands via the processor.
        self.zp_toggle.toggled.connect(lambda state: self.control_module("ZP", state))
        self.xy_toggle.toggled.connect(lambda state: self.control_module("XY", state))
        self.xbox_toggle.toggled.connect(lambda state: self.control_module("XBOX", state))
        control_layout.addWidget(self.zp_toggle)
        control_layout.addWidget(self.xy_toggle)
        control_layout.addWidget(self.xbox_toggle)
        main_layout.addLayout(control_layout)

        # --- ZP Stage Group ---
        self.zp_group = QGroupBox("ZP Stage")
        self.zp_layout = QGridLayout(self.zp_group)
        headers = ["Axis", "Position", "Velocity", "Active", "Deactivated", "Move 1mm"]
        for col, header in enumerate(headers):
            header_label = QLabel(f"<b>{header}</b>")
            self.zp_layout.addWidget(header_label, 0, col)
        self.zp_axis_widgets = {}
        for row, axis in enumerate(["Z", "P1", "P2", "P3"], start=1):
            self.zp_layout.addWidget(QLabel(axis), row, 0)
            pos_label = QLabel("0.0")
            self.zp_layout.addWidget(pos_label, row, 1)
            vel_label = QLabel("0.0")
            self.zp_layout.addWidget(vel_label, row, 2)
            active_label = QLabel("False")
            self.zp_layout.addWidget(active_label, row, 3)
            deact_checkbox = QCheckBox()
            deact_checkbox.stateChanged.connect(lambda state, s="ZP", a=axis: self.handle_deactivation(s, a, state))
            self.zp_layout.addWidget(deact_checkbox, row, 4)
            move_button = QPushButton("Move 1mm")
            move_button.clicked.connect(lambda checked, s="ZP", a=axis: self.move_axis(s, a))
            self.zp_layout.addWidget(move_button, row, 5)
            self.zp_axis_widgets[axis] = {
                "position": pos_label,
                "velocity": vel_label,
                "active": active_label,
                "deactivated": deact_checkbox,
            }
        main_layout.addWidget(self.zp_group)

        # --- XY Stage Group ---
        self.xy_group = QGroupBox("XY Stage")
        self.xy_layout = QGridLayout(self.xy_group)
        for col, header in enumerate(headers):
            header_label = QLabel(f"<b>{header}</b>")
            self.xy_layout.addWidget(header_label, 0, col)
        self.xy_axis_widgets = {}
        for row, axis in enumerate(["x", "y", "f"], start=1):
            self.xy_layout.addWidget(QLabel(axis), row, 0)
            pos_label = QLabel("0.0")
            self.xy_layout.addWidget(pos_label, row, 1)
            vel_label = QLabel("0.0")
            self.xy_layout.addWidget(vel_label, row, 2)
            active_label = QLabel("False")
            self.xy_layout.addWidget(active_label, row, 3)
            deact_checkbox = QCheckBox()
            deact_checkbox.stateChanged.connect(lambda state, s="XY", a=axis: self.handle_deactivation(s, a, state))
            self.xy_layout.addWidget(deact_checkbox, row, 4)
            move_button = QPushButton("Move 1mm")
            move_button.clicked.connect(lambda checked, s="XY", a=axis: self.move_axis(s, a))
            self.xy_layout.addWidget(move_button, row, 5)
            self.xy_axis_widgets[axis] = {
                "position": pos_label,
                "velocity": vel_label,
                "active": active_label,
                "deactivated": deact_checkbox,
            }
        main_layout.addWidget(self.xy_group)
        self.setLayout(main_layout)

    def control_module(self, module, state):
        """Send a control command to start or stop a module based on the toggle state."""
        action = "start" if state else "stop"
        if module in ["ZP", "XY"]:
            self.processor.add_command("control_stage", stage=module, action=action)
            print(f"Command sent: {action.capitalize()} {module} stage")
        elif module == "XBOX":
            self.processor.add_command("control_xbox", action=action)
            print(f"Command sent: {action.capitalize()} Xbox controller")

    def update_ui(self):
        stage_data = self.app_controller.get_stage_info()
        zp_data = stage_data.get("ZP", {})
        for axis, widgets in self.zp_axis_widgets.items():
            axis_data = zp_data.get(axis, {})
            pos = axis_data.get("position", 0.0)
            vel = axis_data.get("velocity", 0.0)
            active = axis_data.get("active", False)
            deactivated = axis_data.get("user_deactivated", False)
            widgets["position"].setText(f"{pos:.3f}")
            widgets["velocity"].setText(f"{vel:.3f}")
            widgets["active"].setText(str(active))
            widgets["deactivated"].blockSignals(True)
            widgets["deactivated"].setChecked(deactivated)
            widgets["deactivated"].blockSignals(False)

        xy_data = stage_data.get("XY", {})
        for axis, widgets in self.xy_axis_widgets.items():
            axis_data = xy_data.get(axis, {})
            pos = axis_data.get("position", 0.0)
            vel = axis_data.get("velocity", 0.0)
            active = axis_data.get("active", False)
            deactivated = axis_data.get("user_deactivated", False)
            widgets["position"].setText(f"{pos:.3f}")
            widgets["velocity"].setText(f"{vel:.3f}")
            widgets["active"].setText(str(active))
            widgets["deactivated"].blockSignals(True)
            widgets["deactivated"].setChecked(deactivated)
            widgets["deactivated"].blockSignals(False)

    def move_axis(self, stage_type, axis):
        self.processor.add_command("move_axis_1mm", stage=stage_type, axis=axis, distance=1)
        print(f"Command sent: Move 1mm on {stage_type} stage, axis {axis}")

    def handle_deactivation(self, stage_type, axis, state):
        deactivated = (state == Qt.Checked)
        self.processor.add_command("set_axis_deactivation", stage=stage_type, axis=axis, deactivated=deactivated)
        print(f"Axis {axis} on {stage_type} stage deactivation set to {deactivated}")
