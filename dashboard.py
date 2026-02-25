"""
Dashboard Page - Status overview of all devices.

Shows connection status, current positions, speed multipliers,
and zero reference positions.

Session 4: Added position log export button (Task 2).
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QFileDialog, QFrame,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController

import logging
logger = logging.getLogger(__name__)


class DashboardPage(QWidget):
    """Overview dashboard showing device status and positions."""

    def __init__(self, controller: StageController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)

        mono = QFont("Consolas", 12)

        # ── XY Position ────────────────────────────────────────
        xy_group = QGroupBox("XY Stage Position")
        xy_grid = QGridLayout(xy_group)

        xy_grid.addWidget(QLabel("X:"), 0, 0)
        self.lbl_x = QLabel("—")
        self.lbl_x.setFont(mono)
        xy_grid.addWidget(self.lbl_x, 0, 1)

        xy_grid.addWidget(QLabel("Y:"), 0, 2)
        self.lbl_y = QLabel("—")
        self.lbl_y.setFont(mono)
        xy_grid.addWidget(self.lbl_y, 0, 3)

        xy_grid.addWidget(QLabel("Status:"), 1, 0)
        self.lbl_xy_status = QLabel("Disconnected")
        self.lbl_xy_status.setStyleSheet("color: #f38ba8;")
        xy_grid.addWidget(self.lbl_xy_status, 1, 1, 1, 3)

        layout.addWidget(xy_group)

        # ── ZP Position ────────────────────────────────────────
        zp_group = QGroupBox("ZP Stage Position")
        zp_grid = QGridLayout(zp_group)

        for col, (name, attr) in enumerate([
            ("Z", "lbl_z"), ("P1", "lbl_p1"), ("P2", "lbl_p2"), ("P3", "lbl_p3")
        ]):
            zp_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(mono)
            zp_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)

        zp_grid.addWidget(QLabel("Status:"), 1, 0)
        self.lbl_zp_status = QLabel("Disconnected")
        self.lbl_zp_status.setStyleSheet("color: #f38ba8;")
        zp_grid.addWidget(self.lbl_zp_status, 1, 1, 1, 7)

        layout.addWidget(zp_group)

        # ── Speed Multipliers ──────────────────────────────────
        speed_group = QGroupBox("Speed Multipliers")
        speed_grid = QGridLayout(speed_group)

        for col, (name, attr) in enumerate([
            ("XY", "lbl_speed_xy"), ("Z", "lbl_speed_z"), ("Pump", "lbl_speed_p")
        ]):
            speed_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(mono)
            speed_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)

        layout.addWidget(speed_group)

        # ── Zero Reference ─────────────────────────────────────
        zero_group = QGroupBox("Zero Reference Position")
        zero_grid = QGridLayout(zero_group)

        self.zero_labels = {}
        for i, key in enumerate(["x", "y", "Z", "P1", "P2", "P3"]):
            row, col = divmod(i, 3)
            zero_grid.addWidget(QLabel(f"{key}:"), row, col * 2)
            lbl = QLabel("0.0")
            lbl.setFont(mono)
            zero_grid.addWidget(lbl, row, col * 2 + 1)
            self.zero_labels[key] = lbl

        layout.addWidget(zero_group)

        # ── Safety Limits Status (Task 3) ──────────────────────
        safety_group = QGroupBox("Safety Limits")
        safety_layout = QHBoxLayout(safety_group)

        self.lbl_safety_status = QLabel("Enabled")
        self.lbl_safety_status.setStyleSheet("color: #a6e3a1; font-weight: bold;")
        safety_layout.addWidget(self.lbl_safety_status)

        self.lbl_safety_info = QLabel("")
        self.lbl_safety_info.setStyleSheet("color: #6c7086;")
        self.lbl_safety_info.setWordWrap(True)
        safety_layout.addWidget(self.lbl_safety_info, stretch=1)

        layout.addWidget(safety_group)

        # ── Position Log (Task 2) ──────────────────────────────
        log_group = QGroupBox("Position Log")
        log_layout = QHBoxLayout(log_group)

        self.lbl_log_count = QLabel("Entries: 0")
        self.lbl_log_count.setFont(mono)
        log_layout.addWidget(self.lbl_log_count)

        btn_export_csv = QPushButton("Export CSV")
        btn_export_csv.clicked.connect(self._export_log_csv)
        log_layout.addWidget(btn_export_csv)

        btn_export_json = QPushButton("Export JSON")
        btn_export_json.clicked.connect(self._export_log_json)
        log_layout.addWidget(btn_export_json)

        btn_clear_log = QPushButton("Clear Log")
        btn_clear_log.setObjectName("disconnectBtn")
        btn_clear_log.clicked.connect(self._clear_log)
        log_layout.addWidget(btn_clear_log)

        layout.addWidget(log_group)

        layout.addStretch()

    # ── Update ─────────────────────────────────────────────────

    def update_data(self):
        """Called by the main window's periodic timer."""
        ctrl = self.controller

        # XY position
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{zx:.0f}")
            self.lbl_y.setText(f"{zy:.0f}")
            self.lbl_xy_status.setText("Connected")
            self.lbl_xy_status.setStyleSheet("color: #a6e3a1;")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")
            conn = "Connected" if ctrl.is_xy_connected else "Disconnected"
            color = "#a6e3a1" if ctrl.is_xy_connected else "#f38ba8"
            self.lbl_xy_status.setText(conn)
            self.lbl_xy_status.setStyleSheet(f"color: {color};")

        # ZP position
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position['P1']:.2f}")
            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position['P2']:.2f}")
            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position['P3']:.2f}")
            self.lbl_zp_status.setText("Connected")
            self.lbl_zp_status.setStyleSheet("color: #a6e3a1;")
        else:
            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:
                lbl.setText("—")
            conn = "Connected" if ctrl.is_zp_connected else "Disconnected"
            color = "#a6e3a1" if ctrl.is_zp_connected else "#f38ba8"
            self.lbl_zp_status.setText(conn)
            self.lbl_zp_status.setStyleSheet(f"color: {color};")

        # Speeds
        speeds = ctrl.get_speed_info()
        self.lbl_speed_xy.setText(f"{speeds['xy']:.0f}")
        self.lbl_speed_z.setText(f"{speeds['z']:.1f}")
        self.lbl_speed_p.setText(f"{speeds['p']:.1f}")

        # Zero reference
        for key, lbl in self.zero_labels.items():
            lbl.setText(f"{ctrl.zero_position.get(key, 0):.1f}")

        # Safety limits
        sl = ctrl.safety_limits
        if sl.enabled:
            self.lbl_safety_status.setText("🛡️ Enabled")
            self.lbl_safety_status.setStyleSheet("color: #a6e3a1; font-weight: bold;")
            self.lbl_safety_info.setText(
                f"XY: [{sl.xy_min_x:.0f}..{sl.xy_max_x:.0f}] × [{sl.xy_min_y:.0f}..{sl.xy_max_y:.0f}]  "
                f"Z: [{sl.z_min:.1f}..{sl.z_max:.1f}]  P: [{sl.p1_min:.1f}..{sl.p1_max:.1f}]"
            )
        else:
            self.lbl_safety_status.setText("⚠ Disabled")
            self.lbl_safety_status.setStyleSheet("color: #f38ba8; font-weight: bold;")
            self.lbl_safety_info.setText("Software endstops are OFF — be careful!")

        # Position log count
        self.lbl_log_count.setText(f"Entries: {ctrl.position_logger.count}")

    # ── Position Log Export (Task 2) ───────────────────────────

    def _export_log_csv(self):
        pl = self.controller.position_logger
        if pl.count == 0:
            return

        default_name = pl.generate_filename()
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Position Log", default_name,
            "CSV Files (*.csv);;All (*)"
        )
        if filepath:
            pl.save_csv(filepath)
            logger.info(f"Position log exported to {filepath}")

    def _export_log_json(self):
        pl = self.controller.position_logger
        if pl.count == 0:
            return

        default_name = pl.generate_filename("position_log").replace(".csv", ".json")
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Position Log", default_name,
            "JSON Files (*.json);;All (*)"
        )
        if filepath:
            pl.save_json(filepath)
            logger.info(f"Position log exported to {filepath}")

    def _clear_log(self):
        self.controller.position_logger.clear()
