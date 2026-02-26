"""
Dashboard Page — Status overview with compact card-based layout.

Main content: position readouts, speed, safety status, print history
Context panel: connection management (XY, ZP, Xbox), position log export
"""

from __future__ import annotations

import logging
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QFileDialog, QFrame, QScrollArea,
    QSizePolicy,
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from gui.styles import COLORS

logger = logging.getLogger(__name__)


class DashboardPage(QWidget):
    """Overview dashboard showing device status and positions."""

    _page_title_text = "Dashboard"

    def __init__(self, controller: StageController, print_history=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.print_history = print_history
        self._context_widget = None
        self._setup_ui()

    def get_page_title(self) -> str:
        return "Dashboard"

    def get_context_widget(self) -> QWidget:
        """Build and return the context panel for the dashboard."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        # ── Connections Section ───────────────────────────────────
        section = QLabel("Connections")
        section.setObjectName("contextSectionLabel")
        layout.addWidget(section)

        # XY Stage
        layout.addWidget(self._make_conn_section(
            "XY Stage", "xy",
            connect_slot=self._connect_xy,
            disconnect_slot=self._disconnect_xy,
        ))

        # ZP Stage
        layout.addWidget(self._make_conn_section(
            "ZP Stage", "zp",
            connect_slot=self._connect_zp,
            disconnect_slot=self._disconnect_zp,
        ))

        # Xbox Controller
        layout.addWidget(self._make_conn_section(
            "Xbox Controller", "xbox",
            connect_slot=self._connect_xbox,
            disconnect_slot=self._disconnect_xbox,
        ))

        # Xbox mapping button
        btn_mapping = QPushButton("🎮 Edit Xbox Mapping")
        btn_mapping.setObjectName("flatBtn")
        btn_mapping.clicked.connect(self._open_xbox_editor)
        layout.addWidget(btn_mapping)

        # ── Position Log Section ──────────────────────────────────
        sep = QLabel("Position Log")
        sep.setObjectName("contextSectionLabel")
        layout.addWidget(sep)

        self.ctx_lbl_log_count = QLabel("Entries: 0")
        self.ctx_lbl_log_count.setObjectName("contextLabel")
        layout.addWidget(self.ctx_lbl_log_count)

        btn_csv = QPushButton("Export CSV")
        btn_csv.clicked.connect(self._export_log_csv)
        layout.addWidget(btn_csv)

        btn_json = QPushButton("Export JSON")
        btn_json.clicked.connect(self._export_log_json)
        layout.addWidget(btn_json)

        btn_clear = QPushButton("Clear Log")
        btn_clear.setObjectName("dangerBtn")
        btn_clear.clicked.connect(self._clear_log)
        layout.addWidget(btn_clear)

        # ── Print History Export ──────────────────────────────────
        sep2 = QLabel("Print History")
        sep2.setObjectName("contextSectionLabel")
        layout.addWidget(sep2)

        btn_hist_csv = QPushButton("Export History CSV")
        btn_hist_csv.clicked.connect(self._export_history_csv)
        layout.addWidget(btn_hist_csv)

        btn_hist_clear = QPushButton("Clear History")
        btn_hist_clear.setObjectName("dangerBtn")
        btn_hist_clear.clicked.connect(self._clear_history)
        layout.addWidget(btn_hist_clear)

        layout.addStretch()

        self._context_widget = ctx
        return ctx

    # ── Context panel connection helpers ──────────────────────────

    def _make_conn_section(self, label: str, prefix: str,
                           connect_slot, disconnect_slot) -> QFrame:
        """Create a compact connection section for the context panel."""
        frame = QFrame()
        frame.setObjectName("cardFrame")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)

        # Title + status
        top_row = QHBoxLayout()
        title = QLabel(label)
        title.setObjectName("contextLabel")
        title.setStyleSheet("font-weight: bold; color: #cdd6f4;")
        top_row.addWidget(title)
        top_row.addStretch()

        status = QLabel("●  Disconnected")
        status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
        setattr(self, f"_ctx_status_{prefix}", status)
        top_row.addWidget(status)
        layout.addLayout(top_row)

        # Buttons
        btn_row = QHBoxLayout()
        btn_conn = QPushButton("Connect")
        btn_conn.setObjectName("successBtn")
        btn_conn.setMaximumHeight(26)
        btn_conn.clicked.connect(connect_slot)
        setattr(self, f"_ctx_btn_conn_{prefix}", btn_conn)

        btn_disc = QPushButton("Disconnect")
        btn_disc.setObjectName("dangerBtn")
        btn_disc.setMaximumHeight(26)
        btn_disc.setEnabled(False)
        btn_disc.clicked.connect(disconnect_slot)
        setattr(self, f"_ctx_btn_disc_{prefix}", btn_disc)

        btn_row.addWidget(btn_conn)
        btn_row.addWidget(btn_disc)
        layout.addLayout(btn_row)

        return frame

    def _update_conn_status(self, prefix: str, connected: bool):
        """Update the context panel connection status for a device."""
        status = getattr(self, f"_ctx_status_{prefix}", None)
        btn_conn = getattr(self, f"_ctx_btn_conn_{prefix}", None)
        btn_disc = getattr(self, f"_ctx_btn_disc_{prefix}", None)
        if status:
            if connected:
                status.setText("●  Connected")
                status.setStyleSheet(f"color: {COLORS['green']}; font-size: 9pt;")
            else:
                status.setText("●  Disconnected")
                status.setStyleSheet(f"color: {COLORS['red']}; font-size: 9pt;")
        if btn_conn:
            btn_conn.setEnabled(not connected)
        if btn_disc:
            btn_disc.setEnabled(connected)

    def _connect_xy(self):
        main_win = self.window()
        if hasattr(main_win, 'connect_xy'):
            main_win.connect_xy()

    def _disconnect_xy(self):
        main_win = self.window()
        if hasattr(main_win, 'disconnect_xy'):
            main_win.disconnect_xy()

    def _connect_zp(self):
        main_win = self.window()
        if hasattr(main_win, 'connect_zp'):
            main_win.connect_zp()

    def _disconnect_zp(self):
        main_win = self.window()
        if hasattr(main_win, 'disconnect_zp'):
            main_win.disconnect_zp()

    def _connect_xbox(self):
        main_win = self.window()
        if hasattr(main_win, 'connect_xbox'):
            main_win.connect_xbox()

    def _disconnect_xbox(self):
        main_win = self.window()
        if hasattr(main_win, 'disconnect_xbox'):
            main_win.disconnect_xbox()

    def _open_xbox_editor(self):
        main_win = self.window()
        if hasattr(main_win, 'open_xbox_editor'):
            main_win.open_xbox_editor()

    # ── Main Content UI ──────────────────────────────────────────

    def _setup_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(16, 12, 16, 12)
        scroll.setWidget(container)

        mono = QFont("Consolas", 12)
        small_mono = QFont("Consolas", 10)

        # ── Positions row (XY + ZP side by side) ─────────────────
        pos_row = QHBoxLayout()
        pos_row.setSpacing(8)

        # XY card
        xy_card = QFrame()
        xy_card.setObjectName("cardFrame")
        xy_layout = QVBoxLayout(xy_card)
        xy_layout.setSpacing(4)

        xy_title = QLabel("XY Stage Position")
        xy_title.setObjectName("sectionLabel")
        xy_layout.addWidget(xy_title)

        xy_grid = QGridLayout()
        xy_grid.setSpacing(4)
        xy_grid.addWidget(QLabel("X:"), 0, 0)
        self.lbl_x = QLabel("—")
        self.lbl_x.setFont(mono)
        self.lbl_x.setObjectName("valueLabel")
        xy_grid.addWidget(self.lbl_x, 0, 1)

        xy_grid.addWidget(QLabel("Y:"), 0, 2)
        self.lbl_y = QLabel("—")
        self.lbl_y.setFont(mono)
        self.lbl_y.setObjectName("valueLabel")
        xy_grid.addWidget(self.lbl_y, 0, 3)

        self.lbl_xy_status = QLabel("Disconnected")
        self.lbl_xy_status.setStyleSheet(f"color: {COLORS['red']};")
        xy_grid.addWidget(self.lbl_xy_status, 1, 0, 1, 4)

        xy_layout.addLayout(xy_grid)
        pos_row.addWidget(xy_card)

        # ZP card
        zp_card = QFrame()
        zp_card.setObjectName("cardFrame")
        zp_layout = QVBoxLayout(zp_card)
        zp_layout.setSpacing(4)

        zp_title = QLabel("ZP Stage Position")
        zp_title.setObjectName("sectionLabel")
        zp_layout.addWidget(zp_title)

        zp_grid = QGridLayout()
        zp_grid.setSpacing(4)
        for col, (name, attr) in enumerate([
            ("Z", "lbl_z"), ("P1", "lbl_p1"), ("P2", "lbl_p2"), ("P3", "lbl_p3")
        ]):
            zp_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(small_mono)
            lbl.setObjectName("valueLabel")
            zp_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)

        self.lbl_zp_status = QLabel("Disconnected")
        self.lbl_zp_status.setStyleSheet(f"color: {COLORS['red']};")
        zp_grid.addWidget(self.lbl_zp_status, 1, 0, 1, 8)

        zp_layout.addLayout(zp_grid)
        pos_row.addWidget(zp_card)

        layout.addLayout(pos_row)

        # ── Speed + Safety + Zero row ────────────────────────────
        info_row = QHBoxLayout()
        info_row.setSpacing(8)

        # Speed card
        speed_card = QFrame()
        speed_card.setObjectName("cardFrame")
        speed_layout = QVBoxLayout(speed_card)
        speed_layout.setSpacing(4)
        speed_layout.addWidget(self._section("Speed Multipliers"))

        speed_grid = QGridLayout()
        speed_grid.setSpacing(4)
        for col, (name, attr) in enumerate([
            ("XY", "lbl_speed_xy"), ("Z", "lbl_speed_z"), ("Pump", "lbl_speed_p")
        ]):
            speed_grid.addWidget(QLabel(f"{name}:"), 0, col * 2)
            lbl = QLabel("—")
            lbl.setFont(small_mono)
            lbl.setObjectName("valueLabel")
            speed_grid.addWidget(lbl, 0, col * 2 + 1)
            setattr(self, attr, lbl)
        speed_layout.addLayout(speed_grid)
        info_row.addWidget(speed_card)

        # Safety card
        safety_card = QFrame()
        safety_card.setObjectName("cardFrame")
        safety_layout = QVBoxLayout(safety_card)
        safety_layout.setSpacing(4)
        safety_layout.addWidget(self._section("Safety Limits"))

        self.lbl_safety_status = QLabel("🛡️ Enabled")
        self.lbl_safety_status.setStyleSheet(f"color: {COLORS['green']}; font-weight: bold;")
        safety_layout.addWidget(self.lbl_safety_status)

        self.lbl_safety_info = QLabel("")
        self.lbl_safety_info.setObjectName("dimLabel")
        self.lbl_safety_info.setWordWrap(True)
        safety_layout.addWidget(self.lbl_safety_info)
        info_row.addWidget(safety_card)

        layout.addLayout(info_row)

        # ── Zero Reference card ──────────────────────────────────
        zero_card = QFrame()
        zero_card.setObjectName("cardFrame")
        zero_layout = QVBoxLayout(zero_card)
        zero_layout.setSpacing(4)
        zero_layout.addWidget(self._section("Zero Reference Position"))

        zero_grid = QGridLayout()
        zero_grid.setSpacing(4)
        self.zero_labels = {}
        for i, key in enumerate(["x", "y", "Z", "P1", "P2", "P3"]):
            row, col = divmod(i, 6)
            zero_grid.addWidget(QLabel(f"{key}:"), row, col * 2)
            lbl = QLabel("0.0")
            lbl.setFont(small_mono)
            zero_grid.addWidget(lbl, row, col * 2 + 1)
            self.zero_labels[key] = lbl
        zero_layout.addLayout(zero_grid)
        layout.addWidget(zero_card)

        # ── Print History Stats card ─────────────────────────────
        hist_card = QFrame()
        hist_card.setObjectName("cardFrame")
        hist_layout = QVBoxLayout(hist_card)
        hist_layout.setSpacing(4)
        hist_layout.addWidget(self._section("Print History"))

        hist_grid = QGridLayout()
        hist_grid.setSpacing(4)
        stats_labels = [
            ("Total:", "lbl_hist_total"),
            ("Completed:", "lbl_hist_completed"),
            ("Aborted:", "lbl_hist_aborted"),
            ("Errors:", "lbl_hist_errors"),
            ("Print Time:", "lbl_hist_time"),
            ("Success Rate:", "lbl_hist_rate"),
        ]
        for i, (name, attr) in enumerate(stats_labels):
            row, col = divmod(i, 3)
            hist_grid.addWidget(QLabel(name), row, col * 2)
            lbl = QLabel("—")
            lbl.setFont(small_mono)
            hist_grid.addWidget(lbl, row, col * 2 + 1)
            setattr(self, attr, lbl)
        hist_layout.addLayout(hist_grid)
        layout.addWidget(hist_card)

        layout.addStretch()

    def _section(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName("sectionLabel")
        return lbl

    # ── Status Update ────────────────────────────────────────────

    def on_status_update(self):
        """Called by MainWindow's periodic timer."""
        self.update_data()

        # Update context panel connection statuses
        self._update_conn_status("xy", self.controller.is_xy_connected)
        self._update_conn_status("zp", self.controller.is_zp_connected)
        self._update_conn_status("xbox", getattr(self.controller, 'is_xbox_connected', False))

        # Update log count in context panel
        if hasattr(self, 'ctx_lbl_log_count'):
            self.ctx_lbl_log_count.setText(f"Entries: {self.controller.position_logger.count}")

    def update_data(self):
        """Update all data readouts."""
        ctrl = self.controller

        # XY position
        xy = ctrl.get_xy_position(cached=True)
        if xy[0] is not None:
            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{zx:.0f}")
            self.lbl_y.setText(f"{zy:.0f}")
            self.lbl_xy_status.setText("Connected")
            self.lbl_xy_status.setStyleSheet(f"color: {COLORS['green']};")
        else:
            self.lbl_x.setText("—")
            self.lbl_y.setText("—")
            conn = "Connected" if ctrl.is_xy_connected else "Disconnected"
            color = COLORS['green'] if ctrl.is_xy_connected else COLORS['red']
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
            self.lbl_zp_status.setStyleSheet(f"color: {COLORS['green']};")
        else:
            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:
                lbl.setText("—")
            conn = "Connected" if ctrl.is_zp_connected else "Disconnected"
            color = COLORS['green'] if ctrl.is_zp_connected else COLORS['red']
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
            self.lbl_safety_status.setStyleSheet(f"color: {COLORS['green']}; font-weight: bold;")
            self.lbl_safety_info.setText(
                f"XY: [{sl.xy_min_x:.0f}..{sl.xy_max_x:.0f}] × [{sl.xy_min_y:.0f}..{sl.xy_max_y:.0f}]  "
                f"Z: [{sl.z_min:.1f}..{sl.z_max:.1f}]  P: [{sl.p1_min:.1f}..{sl.p1_max:.1f}]"
            )
        else:
            self.lbl_safety_status.setText("⚠ Disabled")
            self.lbl_safety_status.setStyleSheet(f"color: {COLORS['red']}; font-weight: bold;")
            self.lbl_safety_info.setText("Software endstops are OFF — be careful!")

        # Print history stats
        if self.print_history:
            stats = self.print_history.get_stats()
            self.lbl_hist_total.setText(str(stats["total_prints"]))
            self.lbl_hist_completed.setText(str(stats["completed"]))
            self.lbl_hist_aborted.setText(str(stats["aborted"]))
            self.lbl_hist_errors.setText(str(stats["errors"]))
            hours = stats["total_print_time_hours"]
            self.lbl_hist_time.setText(f"{hours:.1f} hrs" if hours >= 1 else f"{hours * 60:.1f} min")
            self.lbl_hist_rate.setText(f"{stats['success_rate']:.0f}%")

    # ── Export / Clear Actions ───────────────────────────────────

    def _export_log_csv(self):
        pl = self.controller.position_logger
        if pl.count == 0:
            return
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Position Log", pl.generate_filename(),
            "CSV Files (*.csv);;All (*)")
        if filepath:
            pl.save_csv(filepath)

    def _export_log_json(self):
        pl = self.controller.position_logger
        if pl.count == 0:
            return
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Position Log",
            pl.generate_filename("position_log").replace(".csv", ".json"),
            "JSON Files (*.json);;All (*)")
        if filepath:
            pl.save_json(filepath)

    def _clear_log(self):
        self.controller.position_logger.clear()

    def _export_history_csv(self):
        if not self.print_history or self.print_history.count == 0:
            return
        default_name = f"print_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Export Print History", default_name,
            "CSV Files (*.csv);;All (*)")
        if filepath:
            self.print_history.export_csv(filepath)

    def _clear_history(self):
        if self.print_history:
            self.print_history.clear()
