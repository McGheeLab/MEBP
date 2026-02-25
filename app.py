"""
Main Application Window

Clean tab-based layout:
  - Connection panel at top (always visible)
  - Tab pages: Dashboard, Jog Control, Calibration, Print Setup, Settings
  - Console log at bottom (collapsible)
  - Status bar for quick info

Session 4 additions:
  - Settings tab (Task 5)
  - Safety limits restoration from settings (Task 3)
  - Position log count in status bar (Task 2)
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QPushButton, QLabel, QStatusBar, QSplitter, QGroupBox, QCheckBox,
    QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.Settings import Settings
from gui.styles import DARK_THEME
from gui.pages.dashboard import DashboardPage
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.settings_page import SettingsPage
from gui.widgets.console_log import ConsoleLogWidget
from gui.widgets.xbox_mapping_editor import XboxMappingEditor


class _DisconnectBridge(QObject):
    """Thread-safe bridge for disconnect notifications from watchdog."""
    disconnected = Signal(str)  # stage_name


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self, controller: StageController, settings: Settings = None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings or Settings()

        # Session 4: Restore safety limits from settings
        saved_limits = self.settings.get_section("safety_limits")
        if saved_limits:
            self.controller.safety_limits = SafetyLimits.from_dict(saved_limits)

        # Disconnect bridge for thread-safe notifications
        self._disconnect_bridge = _DisconnectBridge()
        self._disconnect_bridge.disconnected.connect(self._on_hardware_disconnect)
        self.controller.on_disconnect = lambda name: self._disconnect_bridge.disconnected.emit(name)

        self.setWindowTitle("Stage Controller")
        self.setMinimumSize(1000, 700)
        self.setStyleSheet(DARK_THEME)

        self._setup_ui()
        self._setup_status_bar()
        self._setup_timers()
        self._apply_settings()

    def _setup_ui(self):
        """Build the main UI layout."""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(8, 8, 8, 4)

        # Connection panel at top
        conn_frame = QGroupBox("Connections")
        conn_layout = QHBoxLayout(conn_frame)
        conn_layout.setSpacing(12)

        # XY stage connection
        conn_layout.addWidget(self._make_connection_row(
            "XY Stage", "xy",
            self._connect_xy, self._disconnect_xy,
        ))

        # ZP stage connection
        conn_layout.addWidget(self._make_connection_row(
            "ZP Stage", "zp",
            self._connect_zp, self._disconnect_zp,
        ))

        # Xbox connection
        conn_layout.addWidget(self._make_connection_row(
            "Xbox", "xbox",
            self._connect_xbox, self._disconnect_xbox,
        ))

        # Xbox mapping button
        btn_mapping = QPushButton("🎮 Edit Mapping")
        btn_mapping.clicked.connect(self._open_xbox_editor)
        conn_layout.addWidget(btn_mapping)

        main_layout.addWidget(conn_frame)

        # Splitter: Tabs + Console
        self._splitter = QSplitter(Qt.Orientation.Vertical)

        # Tab widget
        self.tabs = QTabWidget()
        self.dashboard_page = DashboardPage(self.controller)
        self.jog_page = JogControlPage(self.controller)
        self.calibration_page = CalibrationPage(self.controller)

        self.tabs.addTab(self.dashboard_page, "📊 Dashboard")
        self.tabs.addTab(self.jog_page, "🕹️ Jog Control")
        self.tabs.addTab(self.calibration_page, "📐 Calibration")
        self.print_setup_page = PrintSetupPage(self.controller)
        self.tabs.addTab(self.print_setup_page, "🖨️ Print Setup")

        # Session 4: Settings tab (Task 5)
        self.settings_page = SettingsPage(self.controller, self.settings)
        self.tabs.addTab(self.settings_page, "⚙ Settings")

        self._splitter.addWidget(self.tabs)

        # Console log
        self.console = ConsoleLogWidget()
        self._splitter.addWidget(self.console)

        self._splitter.setStretchFactor(0, 4)
        self._splitter.setStretchFactor(1, 1)

        main_layout.addWidget(self._splitter)

    def _make_connection_row(self, label_text, prefix, connect_fn, disconnect_fn):
        """Create a connection row with label, status, and connect/disconnect buttons."""
        frame = QWidget()
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.setSpacing(4)

        label = QLabel(label_text)
        label.setObjectName("headerLabel")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

        status = QLabel("Disconnected")
        status.setObjectName("statusDisconnected")
        status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        setattr(self, f"status_{prefix}", status)
        layout.addWidget(status)

        btn_row = QHBoxLayout()
        btn_connect = QPushButton("Connect")
        btn_connect.setObjectName("connectBtn")
        btn_connect.clicked.connect(connect_fn)
        setattr(self, f"btn_connect_{prefix}", btn_connect)

        btn_disconnect = QPushButton("Disconnect")
        btn_disconnect.setObjectName("disconnectBtn")
        btn_disconnect.clicked.connect(disconnect_fn)
        btn_disconnect.setEnabled(False)
        setattr(self, f"btn_disconnect_{prefix}", btn_disconnect)

        btn_row.addWidget(btn_connect)
        btn_row.addWidget(btn_disconnect)
        layout.addLayout(btn_row)

        return frame

    def _setup_status_bar(self):
        """Create the status bar with position readouts."""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        mono = QFont("Consolas", 9)

        self.sb_xy = QLabel("XY: -- , --")
        self.sb_xy.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_xy)

        self.sb_zp = QLabel("Z: -- | P1: -- P2: -- P3: --")
        self.sb_zp.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_zp)

        self.sb_speed = QLabel("Speed XY:-- Z:-- P:--")
        self.sb_speed.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_speed)

        # Session 4: Safety limits indicator
        self.sb_safety = QLabel("🛡️ ON")
        self.sb_safety.setFont(mono)
        self.sb_safety.setToolTip("Safety limits status")
        self.status_bar.addPermanentWidget(self.sb_safety)

        # Session 4: Position log count
        self.sb_log_count = QLabel("📝 0")
        self.sb_log_count.setFont(mono)
        self.sb_log_count.setToolTip("Position log entries")
        self.status_bar.addPermanentWidget(self.sb_log_count)

    def _setup_timers(self):
        """Setup periodic UI update timers."""
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(500)

    def _update_ui(self):
        """Periodic UI refresh."""
        # Position display
        xy = self.controller.get_xy_position(cached=True)
        if xy[0] is not None:
            zero_x = xy[0] - self.controller.zero_position["x"]
            zero_y = xy[1] - self.controller.zero_position["y"]
            self.sb_xy.setText(f"XY: {zero_x:.0f}, {zero_y:.0f}")
        else:
            self.sb_xy.setText("XY: --, --")

        zp = self.controller.get_zp_position(cached=True)
        if zp[0] is not None:
            zero_z = zp[0] - self.controller.zero_position["Z"]
            zero_p1 = zp[1] - self.controller.zero_position["P1"]
            zero_p2 = zp[2] - self.controller.zero_position["P2"]
            zero_p3 = zp[3] - self.controller.zero_position["P3"]
            self.sb_zp.setText(
                f"Z: {zero_z:.2f} | P1: {zero_p1:.2f} P2: {zero_p2:.2f} P3: {zero_p3:.2f}"
            )
        else:
            self.sb_zp.setText("Z: -- | P1: -- P2: -- P3: --")

        # Speed display
        speeds = self.controller.get_speed_info()
        self.sb_speed.setText(
            f"Speed XY:{speeds['xy']:.0f} Z:{speeds['z']:.1f} P:{speeds['p']:.1f}"
        )

        # Safety limits indicator
        sl = self.controller.safety_limits
        self.sb_safety.setText(f"🛡️ {'ON' if sl.enabled else 'OFF'}")
        self.sb_safety.setStyleSheet(
            f"color: {'#a6e3a1' if sl.enabled else '#f38ba8'};"
        )

        # Position log count
        self.sb_log_count.setText(f"📝 {self.controller.position_logger.count}")

        # Update active pages
        active_page = self.tabs.currentWidget()
        if hasattr(active_page, 'update_data'):
            active_page.update_data()

    # ── Connection Actions ─────────────────────────────────────────

    def _connect_xy(self):
        try:
            self.controller.connect_stages()
            self.status_xy.setText("Connected" if self.controller.simulate_xy else "Connected (HW)")
            self.status_xy.setObjectName("statusConnected")
            self.status_xy.setStyleSheet("color: #a6e3a1;")
            self.btn_connect_xy.setEnabled(False)
            self.btn_disconnect_xy.setEnabled(True)
            # ZP also connected via connect_stages
            self._update_zp_status()
            self.console.log("Stages connected", "success")
        except Exception as e:
            self.console.log(f"Connection failed: {e}", "error")

    def _disconnect_xy(self):
        self.controller.disconnect_xy()
        self.status_xy.setText("Disconnected")
        self.status_xy.setStyleSheet("color: #f38ba8;")
        self.btn_connect_xy.setEnabled(True)
        self.btn_disconnect_xy.setEnabled(False)

    def _connect_zp(self):
        try:
            self.controller.connect_stages()
            self._update_zp_status()
            self._update_xy_status()
            self.console.log("Stages connected", "success")
        except Exception as e:
            self.console.log(f"Connection failed: {e}", "error")

    def _disconnect_zp(self):
        self.controller.disconnect_zp()
        self.status_zp.setText("Disconnected")
        self.status_zp.setStyleSheet("color: #f38ba8;")
        self.btn_connect_zp.setEnabled(True)
        self.btn_disconnect_zp.setEnabled(False)

    def _update_xy_status(self):
        if self.controller.is_xy_connected:
            self.status_xy.setText("Connected" if self.controller.simulate_xy else "Connected (HW)")
            self.status_xy.setStyleSheet("color: #a6e3a1;")
            self.btn_connect_xy.setEnabled(False)
            self.btn_disconnect_xy.setEnabled(True)

    def _update_zp_status(self):
        if self.controller.is_zp_connected:
            self.status_zp.setText("Connected" if self.controller.simulate_zp else "Connected (HW)")
            self.status_zp.setStyleSheet("color: #a6e3a1;")
            self.btn_connect_zp.setEnabled(False)
            self.btn_disconnect_zp.setEnabled(True)

    def _connect_xbox(self):
        try:
            mapping = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
            self.controller.connect_xbox(mapping)
            self.status_xbox.setText("Connected")
            self.status_xbox.setStyleSheet("color: #a6e3a1;")
            self.btn_connect_xbox.setEnabled(False)
            self.btn_disconnect_xbox.setEnabled(True)
            self.console.log("Xbox controller connected", "success")
        except Exception as e:
            self.console.log(f"Xbox connection failed: {e}", "error")

    def _disconnect_xbox(self):
        self.controller.disconnect_xbox()
        self.status_xbox.setText("Disconnected")
        self.status_xbox.setStyleSheet("color: #f38ba8;")
        self.btn_connect_xbox.setEnabled(True)
        self.btn_disconnect_xbox.setEnabled(False)

    def _open_xbox_editor(self):
        mapping_file = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
        editor = XboxMappingEditor(mapping_file, parent=self)
        editor.exec()

    def _on_hardware_disconnect(self, stage_name):
        """Handle unexpected hardware disconnect."""
        self.console.log(f"⚠ {stage_name} stage disconnected!", "error")
        if stage_name == "XY":
            self.status_xy.setText("Disconnected")
            self.status_xy.setStyleSheet("color: #f38ba8;")
            self.btn_connect_xy.setEnabled(True)
            self.btn_disconnect_xy.setEnabled(False)
        elif stage_name == "ZP":
            self.status_zp.setText("Disconnected")
            self.status_zp.setStyleSheet("color: #f38ba8;")
            self.btn_connect_zp.setEnabled(True)
            self.btn_disconnect_zp.setEnabled(False)

    # ── Window Events ──────────────────────────────────────────────

    def closeEvent(self, event):
        self.update_timer.stop()
        self.save_settings()
        self.controller.shutdown()
        event.accept()

    # ── Settings Persistence ───────────────────────────────────────

    def _apply_settings(self):
        """Apply saved settings to the window."""
        s = self.settings

        x = s.get("window.x", 100)
        y = s.get("window.y", 100)
        w = s.get("window.width", 1200)
        h = s.get("window.height", 800)
        self.setGeometry(x, y, w, h)

        tab_idx = s.get("window.active_tab", 0)
        if 0 <= tab_idx < self.tabs.count():
            self.tabs.setCurrentIndex(tab_idx)

        splitter_sizes = s.get("window.splitter_sizes", None)
        if splitter_sizes and hasattr(self, '_splitter'):
            self._splitter.setSizes(splitter_sizes)

        saved_speeds = s.get_section("speeds")
        if saved_speeds:
            self._pending_speeds = saved_speeds

    def save_settings(self):
        """Save current window state to settings."""
        s = self.settings

        geo = self.geometry()
        s.set("window.x", geo.x())
        s.set("window.y", geo.y())
        s.set("window.width", geo.width())
        s.set("window.height", geo.height())
        s.set("window.active_tab", self.tabs.currentIndex())

        if hasattr(self, '_splitter'):
            s.set("window.splitter_sizes", self._splitter.sizes())

        s.set("simulation.simulate_xy", self.controller.simulate_xy)
        s.set("simulation.simulate_zp", self.controller.simulate_zp)

        speeds = self.controller.get_speed_info()
        s.set("speeds.xy", speeds["xy"])
        s.set("speeds.z", speeds["z"])
        s.set("speeds.p", speeds["p"])

        s.set_section("zero_position", self.controller.zero_position)

        # Session 4: Save safety limits
        s.set_section("safety_limits", self.controller.safety_limits.to_dict())

        s.save()
