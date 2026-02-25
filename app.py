"""
Main Application Window

Clean tab-based layout:
  - Connection panel at top (always visible)
  - Tab pages: Dashboard, Jog Control, Calibration, Print Setup
  - Console log at bottom (collapsible)
  - Status bar for quick info
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QPushButton, QLabel, QStatusBar, QSplitter, QGroupBox, QCheckBox,
    QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings
from gui.styles import DARK_THEME
from gui.pages.dashboard import DashboardPage
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_setup import PrintSetupPage
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
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # ── Top: Connection Panel ──────────────────────────────────
        conn_group = QGroupBox("Connections")
        conn_layout = QHBoxLayout(conn_group)

        # XY Stage
        xy_frame = self._make_connection_row(
            "XY Stage (Prior)", "xy",
            self._on_connect_xy, self._on_disconnect_xy
        )
        conn_layout.addWidget(xy_frame)

        # Separator
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.VLine)
        conn_layout.addWidget(sep1)

        # ZP Stage
        zp_frame = self._make_connection_row(
            "ZP Stage (Marlin)", "zp",
            self._on_connect_zp, self._on_disconnect_zp
        )
        conn_layout.addWidget(zp_frame)

        # Separator
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.VLine)
        conn_layout.addWidget(sep2)

        # Xbox
        xbox_frame = self._make_connection_row(
            "Xbox Controller", "xbox",
            self._on_connect_xbox, self._on_disconnect_xbox
        )
        conn_layout.addWidget(xbox_frame)

        # Xbox mapping editor button
        btn_mapping = QPushButton("⚙")
        btn_mapping.setToolTip("Edit Xbox button mapping")
        btn_mapping.setFixedSize(36, 36)
        btn_mapping.clicked.connect(self._open_mapping_editor)
        conn_layout.addWidget(btn_mapping)

        # Separator
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.Shape.VLine)
        conn_layout.addWidget(sep3)

        # Emergency Stop
        self.btn_estop = QPushButton("EMERGENCY\nSTOP")
        self.btn_estop.setObjectName("emergencyStop")
        self.btn_estop.setFixedWidth(140)
        self.btn_estop.clicked.connect(self._on_emergency_stop)
        conn_layout.addWidget(self.btn_estop)

        main_layout.addWidget(conn_group)

        # ── Middle: Splitter with tabs and console ─────────────────
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

        # Label
        label = QLabel(label_text)
        label.setObjectName("headerLabel")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

        # Status
        status = QLabel("Disconnected")
        status.setObjectName("statusDisconnected")
        status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        setattr(self, f"status_{prefix}", status)
        layout.addWidget(status)

        # Buttons
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

    def _setup_timers(self):
        """Setup periodic UI update timers."""
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(500)  # 500ms update cycle

    # ── Connection Handlers ────────────────────────────────────────

    def _set_connected(self, prefix, connected):
        """Update UI state for a connection."""
        status = getattr(self, f"status_{prefix}")
        btn_c = getattr(self, f"btn_connect_{prefix}")
        btn_d = getattr(self, f"btn_disconnect_{prefix}")

        if connected:
            status.setText("Connected")
            status.setObjectName("statusConnected")
            btn_c.setEnabled(False)
            btn_d.setEnabled(True)
        else:
            status.setText("Disconnected")
            status.setObjectName("statusDisconnected")
            btn_c.setEnabled(True)
            btn_d.setEnabled(False)

        # Force style refresh
        status.style().unpolish(status)
        status.style().polish(status)

    def _on_connect_xy(self):
        try:
            if self.controller.xy_stage is None:
                from SupportClasses.XYStage import XYStageManager
                from SupportClasses.StageController import XYJogHandler
                self.controller.xy_stage = XYStageManager(simulate=self.controller.simulate_xy)
                self.controller.xy_jog = XYJogHandler(self.controller.processor, self.controller.xy_stage)
                # Apply saved speed
                saved_xy = self.settings.get("speeds.xy")
                if saved_xy and self.controller.xy_jog:
                    self.controller.xy_jog.xy_speed = saved_xy
                self.controller.xy_jog.start()
                # Register with position poller
                self.controller._pos_poller.set_stages(
                    self.controller.xy_stage, self.controller.zp_stage
                )
            self._set_connected("xy", True)
            self.console.log("XY stage connected")
        except Exception as e:
            self.console.log(f"XY connection failed: {e}", "error")

    def _on_disconnect_xy(self):
        if self.controller.xy_jog:
            self.controller.xy_jog.stop()
            self.controller.xy_jog = None
        if self.controller.xy_stage:
            self.controller.xy_stage.stop()
            self.controller.xy_stage = None
        self.controller._pos_poller.set_stages(None, self.controller.zp_stage)
        self._set_connected("xy", False)
        self.console.log("XY stage disconnected")

    def _on_connect_zp(self):
        try:
            if self.controller.zp_stage is None:
                from SupportClasses.ZPStage import ZPStageManager
                from SupportClasses.StageController import ZPJogHandler
                self.controller.zp_stage = ZPStageManager(simulate=self.controller.simulate_zp)
                self.controller.zp_jog = ZPJogHandler(self.controller.processor, self.controller.zp_stage)
                # Apply saved speeds
                saved_z = self.settings.get("speeds.z")
                saved_p = self.settings.get("speeds.p")
                if self.controller.zp_jog:
                    if saved_z:
                        self.controller.zp_jog.speeds["z"] = saved_z
                    if saved_p:
                        self.controller.zp_jog.speeds["p"] = saved_p
                self.controller.zp_jog.start()
                # Register with position poller
                self.controller._pos_poller.set_stages(
                    self.controller.xy_stage, self.controller.zp_stage
                )
            self._set_connected("zp", True)
            self.console.log("ZP stage connected")
        except Exception as e:
            self.console.log(f"ZP connection failed: {e}", "error")

    def _on_disconnect_zp(self):
        if self.controller.zp_jog:
            self.controller.zp_jog.stop()
            self.controller.zp_jog = None
        if self.controller.zp_stage:
            self.controller.zp_stage.stop()
            self.controller.zp_stage = None
        self.controller._pos_poller.set_stages(self.controller.xy_stage, None)
        self._set_connected("zp", False)
        self.console.log("ZP stage disconnected")

    def _on_connect_xbox(self):
        try:
            self.controller.connect_xbox()
            self._set_connected("xbox", True)
            self.console.log("Xbox controller connected")
        except Exception as e:
            self.console.log(f"Xbox connection failed: {e}", "error")

    def _on_disconnect_xbox(self):
        self.controller.disconnect_xbox()
        self._set_connected("xbox", False)
        self.console.log("Xbox controller disconnected")

    def _open_mapping_editor(self):
        """Open the Xbox button mapping editor dialog."""
        mapping_file = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
        dialog = XboxMappingEditor(mapping_file, parent=self)
        dialog.setStyleSheet(self.styleSheet())
        if dialog.exec():
            self.console.log("Xbox mapping updated (hot-reloaded in ~5s)", "success")

    def _on_emergency_stop(self):
        """Emergency stop all axes."""
        self.console.log("EMERGENCY STOP", "error")
        if self.controller.zp_stage:
            self.controller.zp_stage.emergency_stop()
        if self.controller.xy_stage:
            self.controller.xy_stage.move_stage_at_velocity(0, 0)

    def _on_hardware_disconnect(self, stage_name: str):
        """Handle unexpected hardware disconnect (called from watchdog via signal)."""
        self.console.log(f"{stage_name} stage disconnected unexpectedly!", "error")
        # Update connection status in the UI
        if stage_name == "XY":
            self._set_connected("xy", False)
        elif stage_name == "ZP":
            self._set_connected("zp", False)

    # ── Periodic UI Update ─────────────────────────────────────────

    def _update_ui(self):
        """Update status bar and page data."""
        # XY position
        if self.controller.is_xy_connected:
            try:
                x, y, f = self.controller.get_xy_position()
                if x is not None:
                    self.sb_xy.setText(f"XY: {x:.1f} , {y:.1f}")
            except Exception:
                pass
        else:
            self.sb_xy.setText("XY: --")

        # ZP position
        if self.controller.is_zp_connected:
            try:
                z, p1, p2, p3 = self.controller.get_zp_position()
                if z is not None:
                    self.sb_zp.setText(f"Z:{z:.3f} | P1:{p1:.3f} P2:{p2:.3f} P3:{p3:.3f}")
            except Exception:
                pass
        else:
            self.sb_zp.setText("Z: -- | P1: -- P2: -- P3: --")

        # Speed info
        speeds = self.controller.get_speed_info()
        self.sb_speed.setText(
            f"Speed XY:{speeds['xy']:.0f} Z:{speeds['z']:.2f} P:{speeds['p']:.2f}"
        )

        # Update connection status indicators
        self._set_connected("xy", self.controller.is_xy_connected)
        self._set_connected("zp", self.controller.is_zp_connected)
        self._set_connected("xbox", self.controller.is_xbox_connected)

        # Update active tab
        current = self.tabs.currentWidget()
        if hasattr(current, 'update_data'):
            current.update_data()

    def closeEvent(self, event):
        """Clean shutdown on window close."""
        self.update_timer.stop()
        self.save_settings()
        self.controller.shutdown()
        event.accept()

    # ── Settings Persistence ───────────────────────────────────────

    def _apply_settings(self):
        """Apply saved settings to the window."""
        s = self.settings

        # Window geometry
        x = s.get("window.x", 100)
        y = s.get("window.y", 100)
        w = s.get("window.width", 1200)
        h = s.get("window.height", 800)
        self.setGeometry(x, y, w, h)

        # Active tab
        tab_idx = s.get("window.active_tab", 0)
        if 0 <= tab_idx < self.tabs.count():
            self.tabs.setCurrentIndex(tab_idx)

        # Splitter sizes
        splitter_sizes = s.get("window.splitter_sizes", None)
        if splitter_sizes and hasattr(self, '_splitter'):
            self._splitter.setSizes(splitter_sizes)

        # Apply saved speeds to jog handlers (if stages are connected later)
        saved_speeds = s.get_section("speeds")
        if saved_speeds:
            self._pending_speeds = saved_speeds

    def save_settings(self):
        """Save current window state to settings."""
        s = self.settings

        # Window geometry
        geo = self.geometry()
        s.set("window.x", geo.x())
        s.set("window.y", geo.y())
        s.set("window.width", geo.width())
        s.set("window.height", geo.height())
        s.set("window.active_tab", self.tabs.currentIndex())

        if hasattr(self, '_splitter'):
            s.set("window.splitter_sizes", self._splitter.sizes())

        # Simulation mode
        s.set("simulation.simulate_xy", self.controller.simulate_xy)
        s.set("simulation.simulate_zp", self.controller.simulate_zp)

        # Speeds
        speeds = self.controller.get_speed_info()
        s.set("speeds.xy", speeds["xy"])
        s.set("speeds.z", speeds["z"])
        s.set("speeds.p", speeds["p"])

        # Zero position
        s.set_section("zero_position", self.controller.zero_position)

        s.save()
