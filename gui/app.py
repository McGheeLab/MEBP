"""
Main Application Window — PyDracula-style layout.

Layout:
    ┌────┬──────────┬──────────────────────────────┐
    │ L  │ Context  │  Top Bar (title + conn dots)  │
    │ E  │ Panel    ├──────────────────────────────┤
    │ F  │ (extra   │                              │
    │ T  │  left    │   Content Pages (stacked)    │
    │    │  box)    │                              │
    │ M  │          ├──────────────────────────────┤
    │ E  │          │   Console Log (collapsible)  │
    │ N  │          ├──────────────────────────────┤
    │ U  │          │   Bottom Bar (status)        │
    └────┴──────────┴──────────────────────────────┘

Workflow tabs (left menu icons):
    📊  Dashboard       — device status, positions, print history
    🕹️  Jog Control     — manual movement, dpad, speed control
    📐  Calibration     — needle zero, plate teach, validate
    🖨️  Print Setup     — job loading, preview, execution, queue
    ⚙️  Settings        — connections, safety, polling, logging

Each page provides:
    - get_context_widget() → QWidget for the extra-left panel
    - get_page_title() → str for the top bar
    - get_page_subtitle() → str
"""

from __future__ import annotations

import logging
from functools import partial

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFrame,
    QPushButton, QLabel, QSizePolicy, QStackedWidget, QSplitter,
    QScrollArea, QMessageBox, QApplication,
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject, QSize
from PySide6.QtGui import QFont, QKeyEvent, QIcon, QPixmap, QPainter, QColor

from SupportClasses.StageController import StageController
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.Settings import Settings
from SupportClasses.PrintHistory import PrintHistory
from SupportClasses.PrintManager import load_print_progress, clear_print_progress
from gui.styles import DARK_THEME, COLORS
from gui.ui_functions import UIFunctions, AppSettings
from gui.pages.dashboard import DashboardPage
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.settings_page import SettingsPage
from gui.widgets.console_log import ConsoleLogWidget
from gui.widgets.xbox_mapping_editor import XboxMappingEditor

logger = logging.getLogger(__name__)


# ── Emoji → colored pixmap icon helper ───────────────────────────
def _make_text_icon(text: str, size: int = 24, color: str = "#a6adc8") -> QIcon:
    """Create a QIcon from a text character (emoji or symbol)."""
    pix = QPixmap(size, size)
    pix.fill(QColor(0, 0, 0, 0))
    painter = QPainter(pix)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setPen(QColor(color))
    font = QFont("Segoe UI Emoji", int(size * 0.55))
    painter.setFont(font)
    painter.drawText(pix.rect(), Qt.AlignCenter, text)
    painter.end()
    return QIcon(pix)


class _DisconnectBridge(QObject):
    """Thread-safe bridge for disconnect notifications from watchdog."""
    disconnected = Signal(str)  # stage_name


class MainWindow(QMainWindow):
    """Main application window with PyDracula-style navigation."""

    # Page indices
    PAGE_DASHBOARD = 0
    PAGE_JOG = 1
    PAGE_CALIBRATION = 2
    PAGE_PRINT = 3
    PAGE_SETTINGS = 4

    def __init__(self, controller: StageController, settings: Settings = None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings or Settings()

        # Restore safety limits from settings
        saved_limits = self.settings.get_section("safety_limits")
        if saved_limits:
            self.controller.safety_limits = SafetyLimits.from_dict(saved_limits)

        # Print history
        self.print_history = PrintHistory()
        self.print_history.load()

        # Disconnect bridge
        self._disconnect_bridge = _DisconnectBridge()
        self._disconnect_bridge.disconnected.connect(self._on_hardware_disconnect)
        self.controller.on_disconnect = lambda name: self._disconnect_bridge.disconnected.emit(name)

        self.setWindowTitle("MEBP Bioprinter")
        self.setMinimumSize(1100, 700)
        self.setStyleSheet(DARK_THEME)

        self._current_page_index = 0
        self._menu_buttons: list[QPushButton] = []
        self._page_widgets: list[QWidget] = []
        self._context_widgets: list[QWidget | None] = []

        self._build_shell()
        self._create_pages()
        self._build_bottom_bar()
        self._setup_timers()
        self._setup_global_shortcuts()
        self._apply_settings()

        # Start on dashboard
        self._switch_page(self.PAGE_DASHBOARD)

        # Check for print resume data
        QTimer.singleShot(1000, self._check_print_resume)

    # ════════════════════════════════════════════════════════════════
    #  SHELL CONSTRUCTION
    # ════════════════════════════════════════════════════════════════

    def _build_shell(self):
        """Build the PyDracula-style shell layout."""
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        app_layout = QHBoxLayout(central)
        app_layout.setSpacing(0)
        app_layout.setContentsMargins(0, 0, 0, 0)

        # ── Left Menu (icon sidebar) ─────────────────────────────
        self.ui_leftMenuBg = QFrame()
        self.ui_leftMenuBg.setObjectName("leftMenuBg")
        self.ui_leftMenuBg.setMinimumWidth(60)
        self.ui_leftMenuBg.setMaximumWidth(60)
        self.ui_leftMenuBg.setFrameShape(QFrame.NoFrame)

        left_layout = QVBoxLayout(self.ui_leftMenuBg)
        left_layout.setSpacing(0)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # Logo / Title area
        logo_frame = QFrame()
        logo_frame.setObjectName("topLogo")
        logo_frame.setMinimumHeight(50)
        logo_frame.setMaximumHeight(50)
        logo_layout = QHBoxLayout(logo_frame)
        logo_layout.setContentsMargins(8, 0, 8, 0)
        logo_label = QLabel("🧬")
        logo_label.setFont(QFont("Segoe UI Emoji", 18))
        logo_label.setAlignment(Qt.AlignCenter)
        logo_layout.addWidget(logo_label)
        self._logo_text = QLabel("MEBP")
        self._logo_text.setObjectName("titleLeftApp")
        self._logo_text.setVisible(False)  # Hidden when collapsed
        logo_layout.addWidget(self._logo_text)
        logo_layout.addStretch()
        left_layout.addWidget(logo_frame)

        # Toggle button
        self._toggle_btn = QPushButton("≡")
        self._toggle_btn.setObjectName("toggleButton")
        self._toggle_btn.setMinimumHeight(36)
        self._toggle_btn.setCursor(Qt.PointingHandCursor)
        self._toggle_btn.setToolTip("Expand menu")
        self._toggle_btn.clicked.connect(lambda: UIFunctions.toggleMenu(self))
        left_layout.addWidget(self._toggle_btn)

        # Separator
        sep = QFrame()
        sep.setObjectName("leftMenuFrame")
        sep.setFrameShape(QFrame.HLine)
        sep.setMaximumHeight(2)
        left_layout.addWidget(sep)

        # Top menu (workflow buttons)
        self.ui_topMenu = QFrame()
        self.ui_topMenu.setObjectName("topMenu")
        self.ui_topMenu.setFrameShape(QFrame.NoFrame)
        top_menu_layout = QVBoxLayout(self.ui_topMenu)
        top_menu_layout.setSpacing(0)
        top_menu_layout.setContentsMargins(0, 4, 0, 4)

        # Create workflow buttons
        menu_items = [
            ("btn_dashboard", "📊", "Dashboard"),
            ("btn_jog",       "🕹️", "Jog Control"),
            ("btn_calibrate", "📐", "Calibration"),
            ("btn_print",     "🖨️", "Print Setup"),
        ]
        for obj_name, icon_text, tooltip in menu_items:
            btn = self._make_menu_button(obj_name, icon_text, tooltip)
            top_menu_layout.addWidget(btn)
            self._menu_buttons.append(btn)

        left_layout.addWidget(self.ui_topMenu, 0, Qt.AlignTop)

        # Spacer
        left_layout.addStretch()

        # Bottom menu (settings, etc.)
        bottom_menu = QFrame()
        bottom_menu.setObjectName("bottomMenu")
        bottom_menu.setFrameShape(QFrame.NoFrame)
        bottom_layout = QVBoxLayout(bottom_menu)
        bottom_layout.setSpacing(0)
        bottom_layout.setContentsMargins(0, 0, 0, 4)

        btn_settings = self._make_menu_button("btn_settings", "⚙️", "Settings")
        bottom_layout.addWidget(btn_settings)
        self._menu_buttons.append(btn_settings)

        left_layout.addWidget(bottom_menu, 0, Qt.AlignBottom)

        app_layout.addWidget(self.ui_leftMenuBg)

        # ── Extra Left Box (context panel) ───────────────────────
        self.ui_extraLeftBox = QFrame()
        self.ui_extraLeftBox.setObjectName("extraLeftBox")
        self.ui_extraLeftBox.setMinimumWidth(0)
        self.ui_extraLeftBox.setMaximumWidth(0)
        self.ui_extraLeftBox.setFrameShape(QFrame.NoFrame)

        extra_layout = QVBoxLayout(self.ui_extraLeftBox)
        extra_layout.setSpacing(0)
        extra_layout.setContentsMargins(0, 0, 0, 0)

        # Top bar of context panel
        extra_top = QFrame()
        extra_top.setObjectName("extraTopBg")
        extra_top.setMinimumHeight(40)
        extra_top.setMaximumHeight(40)
        extra_top_layout = QHBoxLayout(extra_top)
        extra_top_layout.setContentsMargins(10, 0, 6, 0)

        self._context_title = QLabel("Settings")
        self._context_title.setObjectName("extraLabel")
        extra_top_layout.addWidget(self._context_title)
        extra_top_layout.addStretch()

        btn_close_context = QPushButton("✕")
        btn_close_context.setObjectName("extraCloseColumnBtn")
        btn_close_context.setFixedSize(28, 28)
        btn_close_context.setCursor(Qt.PointingHandCursor)
        btn_close_context.clicked.connect(lambda: UIFunctions.toggleLeftBox(self))
        extra_top_layout.addWidget(btn_close_context)

        extra_layout.addWidget(extra_top)

        # Context content area (stacked)
        self._context_stack = QStackedWidget()
        self._context_stack.setObjectName("extraContent")
        extra_layout.addWidget(self._context_stack)

        app_layout.addWidget(self.ui_extraLeftBox)

        # ── Content Area ─────────────────────────────────────────
        content_frame = QFrame()
        content_frame.setObjectName("contentBox")
        content_layout = QVBoxLayout(content_frame)
        content_layout.setSpacing(0)
        content_layout.setContentsMargins(0, 0, 0, 0)

        # Top bar
        top_bar = QFrame()
        top_bar.setObjectName("contentTopBg")
        top_bar.setMinimumHeight(40)
        top_bar.setMaximumHeight(40)
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(12, 0, 12, 0)

        # Page title area
        title_frame = QWidget()
        title_layout = QVBoxLayout(title_frame)
        title_layout.setSpacing(0)
        title_layout.setContentsMargins(0, 4, 0, 4)

        self._page_title = QLabel("Dashboard")
        self._page_title.setObjectName("pageTitle")
        self._page_title.setFont(QFont("Segoe UI", 12, QFont.Bold))
        title_layout.addWidget(self._page_title)

        top_bar_layout.addWidget(title_frame)
        top_bar_layout.addStretch()

        # Connection dots (compact status indicators)
        conn_frame = QFrame()
        conn_frame.setObjectName("connStatusFrame")
        conn_layout = QHBoxLayout(conn_frame)
        conn_layout.setSpacing(12)
        conn_layout.setContentsMargins(0, 0, 0, 0)

        self._conn_xy = self._make_conn_dot("XY")
        self._conn_zp = self._make_conn_dot("ZP")
        self._conn_xbox = self._make_conn_dot("Xbox")
        conn_layout.addWidget(self._conn_xy)
        conn_layout.addWidget(self._conn_zp)
        conn_layout.addWidget(self._conn_xbox)

        # Context panel toggle button
        self._btn_context_toggle = QPushButton("☰")
        self._btn_context_toggle.setObjectName("flatBtn")
        self._btn_context_toggle.setFixedSize(32, 32)
        self._btn_context_toggle.setCursor(Qt.PointingHandCursor)
        self._btn_context_toggle.setToolTip("Toggle settings panel")
        self._btn_context_toggle.clicked.connect(lambda: UIFunctions.toggleLeftBox(self))
        conn_layout.addWidget(self._btn_context_toggle)

        top_bar_layout.addWidget(conn_frame)

        content_layout.addWidget(top_bar)

        # Content splitter (pages + console)
        self._splitter = QSplitter(Qt.Vertical)
        self._splitter.setObjectName("contentBottom")

        # Stacked pages
        self._page_stack = QStackedWidget()
        self._splitter.addWidget(self._page_stack)

        # Console log
        self.console = ConsoleLogWidget()
        self._splitter.addWidget(self.console)

        self._splitter.setStretchFactor(0, 5)
        self._splitter.setStretchFactor(1, 1)

        content_layout.addWidget(self._splitter)

        app_layout.addWidget(content_frame)

    def _make_menu_button(self, obj_name: str, icon_text: str, tooltip: str) -> QPushButton:
        """Create a left-menu navigation button."""
        btn = QPushButton(f"  {icon_text}")
        btn.setObjectName(obj_name)
        btn.setMinimumHeight(45)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(tooltip)
        btn.setFont(QFont("Segoe UI Emoji", 12))
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn.clicked.connect(self._on_menu_click)
        return btn

    def _make_conn_dot(self, label: str) -> QWidget:
        """Create a compact connection status indicator (dot + label)."""
        frame = QWidget()
        layout = QHBoxLayout(frame)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)

        dot = QLabel("●")
        dot.setObjectName("connDotOff")
        dot.setFixedWidth(14)
        dot.setAlignment(Qt.AlignCenter)
        layout.addWidget(dot)

        lbl = QLabel(label)
        lbl.setObjectName("connLabelOff")
        layout.addWidget(lbl)

        # Store dot reference for status updates
        setattr(self, f"_dot_{label.lower()}", dot)
        setattr(self, f"_lbl_{label.lower()}", lbl)

        return frame

    def _build_bottom_bar(self):
        """Build the bottom status bar."""
        self.status_bar = self.statusBar()
        self.status_bar.setObjectName("bottomBar")

        mono = QFont("Consolas", 9)

        self.sb_xy = QLabel("XY: — , —")
        self.sb_xy.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_xy)

        self.sb_zp = QLabel("Z: — | P1: — P2: — P3: —")
        self.sb_zp.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_zp)

        self.sb_speed = QLabel("Speed XY:— Z:— P:—")
        self.sb_speed.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_speed)

        self.sb_safety = QLabel("🛡️ ON")
        self.sb_safety.setFont(mono)
        self.sb_safety.setToolTip("Safety limits status")
        self.status_bar.addPermanentWidget(self.sb_safety)

        self.sb_log_count = QLabel("📝 0")
        self.sb_log_count.setFont(mono)
        self.sb_log_count.setToolTip("Position log entries")
        self.status_bar.addPermanentWidget(self.sb_log_count)

    # ════════════════════════════════════════════════════════════════
    #  PAGE CREATION
    # ════════════════════════════════════════════════════════════════

    def _create_pages(self):
        """Instantiate all page widgets and their context panels."""
        # Create pages
        self.dashboard_page = DashboardPage(self.controller, print_history=self.print_history)
        self.jog_page = JogControlPage(self.controller)
        self.calibration_page = CalibrationPage(self.controller, settings=self.settings)
        self.print_setup_page = PrintSetupPage(self.controller)
        self.settings_page = SettingsPage(self.controller, self.settings)

        pages = [
            self.dashboard_page,
            self.jog_page,
            self.calibration_page,
            self.print_setup_page,
            self.settings_page,
        ]

        for page in pages:
            self._page_stack.addWidget(page)
            self._page_widgets.append(page)

            # Get context widget if the page provides one
            ctx = None
            if hasattr(page, 'get_context_widget'):
                ctx = page.get_context_widget()
            if ctx is None:
                # Provide a default empty context widget
                ctx = self._make_default_context(page)

            # Wrap in scroll area
            scroll = QScrollArea()
            scroll.setObjectName("contextScrollArea")
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            scroll.setWidget(ctx)
            self._context_stack.addWidget(scroll)
            self._context_widgets.append(ctx)

    def _make_default_context(self, page) -> QWidget:
        """Create a default context panel for pages that don't have one yet."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = getattr(page, '_page_title_text', page.__class__.__name__)
        lbl = QLabel(f"No settings panel\nfor {title}")
        lbl.setObjectName("dimLabel")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setWordWrap(True)
        layout.addWidget(lbl)
        layout.addStretch()
        return widget

    # ════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ════════════════════════════════════════════════════════════════

    def _on_menu_click(self):
        """Handle left menu button clicks."""
        btn = self.sender()
        if not btn:
            return

        name = btn.objectName()
        page_map = {
            "btn_dashboard": self.PAGE_DASHBOARD,
            "btn_jog": self.PAGE_JOG,
            "btn_calibrate": self.PAGE_CALIBRATION,
            "btn_print": self.PAGE_PRINT,
            "btn_settings": self.PAGE_SETTINGS,
        }

        idx = page_map.get(name)
        if idx is not None:
            self._switch_page(idx)

    def _switch_page(self, index: int):
        """Switch to the specified page and update context panel."""
        self._current_page_index = index

        # Update page stack
        self._page_stack.setCurrentIndex(index)

        # Update context stack
        self._context_stack.setCurrentIndex(index)

        # Update page title
        page = self._page_widgets[index]
        title = "MEBP Bioprinter"
        if hasattr(page, 'get_page_title'):
            title = page.get_page_title()
        else:
            titles = ["Dashboard", "Jog Control", "Calibration", "Print Setup", "Settings"]
            title = titles[index] if index < len(titles) else title
        self._page_title.setText(title)

        # Update context panel title
        context_titles = ["Dashboard", "Jog Settings", "Calibration", "Print Settings", "Settings"]
        self._context_title.setText(context_titles[index] if index < len(context_titles) else "Settings")

        # Update menu button styling
        btn_names = ["btn_dashboard", "btn_jog", "btn_calibrate", "btn_print", "btn_settings"]
        for i, btn in enumerate(self._menu_buttons):
            if i == index:
                btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))
            else:
                btn.setStyleSheet(UIFunctions.deselectMenu(btn.styleSheet()))

        # Auto-open context panel if page has context content
        if hasattr(page, 'get_context_widget') and page.get_context_widget() is not None:
            if self.ui_extraLeftBox.width() == 0:
                UIFunctions.setLeftBoxWidth(self, AppSettings.LEFT_BOX_WIDTH)
        else:
            if self.ui_extraLeftBox.width() > 0:
                UIFunctions.setLeftBoxWidth(self, 0)

    # ════════════════════════════════════════════════════════════════
    #  TIMERS & STATUS UPDATES
    # ════════════════════════════════════════════════════════════════

    def _setup_timers(self):
        """Set up periodic update timers."""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_status)
        interval = self.settings.get("polling.position_interval_ms", 300)
        self.update_timer.start(interval)

    def _update_status(self):
        """Update connection dots and status bar readouts."""
        # Connection indicators
        xy_ok = self.controller.is_xy_connected
        zp_ok = self.controller.is_zp_connected
        xbox_ok = getattr(self.controller, 'is_xbox_connected', False)

        self._update_conn_dot("xy", xy_ok)
        self._update_conn_dot("zp", zp_ok)
        self._update_conn_dot("xbox", xbox_ok)

        # Position readouts
        try:
            xy = self.controller.get_xy_position(cached=True)
            zp = self.controller.get_zp_position(cached=True)
            speeds = self.controller.get_speed_info()

            if xy[0] is not None:
                zx = self.controller.zero_position["x"]
                zy = self.controller.zero_position["y"]
                self.sb_xy.setText(f"XY: {xy[0] - zx:,.0f} , {xy[1] - zy:,.0f}")
            else:
                self.sb_xy.setText("XY: — , —")

            if zp[0] is not None:
                zz = self.controller.zero_position.get("Z", 0)
                zp1 = self.controller.zero_position.get("P1", 0)
                zp2 = self.controller.zero_position.get("P2", 0)
                zp3 = self.controller.zero_position.get("P3", 0)
                self.sb_zp.setText(
                    f"Z: {zp[0] - zz:.2f} | "
                    f"P1: {zp[1] - zp1:.2f} "
                    f"P2: {zp[2] - zp2:.2f} "
                    f"P3: {zp[3] - zp3:.2f}"
                )
            else:
                self.sb_zp.setText("Z: — | P1: — P2: — P3: —")

            self.sb_speed.setText(
                f"Speed XY:{speeds['xy']:,.0f} Z:{speeds['z']:.1f} P:{speeds['p']:.1f}"
            )
        except Exception:
            pass

        # Safety status
        sl = self.controller.safety_limits
        if sl.enabled:
            self.sb_safety.setText("🛡️ ON")
            self.sb_safety.setStyleSheet(f"color: {COLORS['green']};")
        else:
            self.sb_safety.setText("🛡️ OFF")
            self.sb_safety.setStyleSheet(f"color: {COLORS['yellow']};")

        # Position log count
        if hasattr(self.controller, 'position_logger'):
            count = self.controller.position_logger.count
            self.sb_log_count.setText(f"📝 {count}")

        # Propagate updates to active page
        page = self._page_widgets[self._current_page_index]
        if hasattr(page, 'on_status_update'):
            page.on_status_update()

    def _update_conn_dot(self, name: str, connected: bool):
        """Update a connection dot indicator with state-based styling."""
        dot = getattr(self, f"_dot_{name}", None)
        lbl = getattr(self, f"_lbl_{name}", None)
        if dot:
            if connected:
                dot.setObjectName("connDotOn")
                dot.setText("●")
            else:
                dot.setObjectName("connDotOff")
                dot.setText("●")
            dot.style().unpolish(dot)
            dot.style().polish(dot)
        if lbl:
            if connected:
                lbl.setObjectName("connLabelOn")
            else:
                lbl.setObjectName("connLabelOff")
            lbl.style().unpolish(lbl)
            lbl.style().polish(lbl)

    # ════════════════════════════════════════════════════════════════
    #  CONNECTION MANAGEMENT (delegated to Settings page context)
    # ════════════════════════════════════════════════════════════════

    def connect_xy(self):
        """Connect XY stage — called from context panels."""
        try:
            self.controller.connect_stages(xy=True, zp=False)
            self.console.log("XY stage connected", "success")
        except Exception as e:
            self.console.log(f"XY connection failed: {e}", "error")

    def disconnect_xy(self):
        self.controller.disconnect_xy()
        self.console.log("XY stage disconnected", "info")

    def connect_zp(self):
        try:
            self.controller.connect_stages(xy=False, zp=True)
            self.console.log("ZP stage connected", "success")
        except Exception as e:
            self.console.log(f"ZP connection failed: {e}", "error")

    def disconnect_zp(self):
        self.controller.disconnect_zp()
        self.console.log("ZP stage disconnected", "info")

    def connect_xbox(self):
        try:
            mapping_file = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
            self.controller.connect_xbox(mapping_file)
            # Verify connection after a brief delay — the worker process exits
            # immediately if no controller is found, so we check is_alive().
            QTimer.singleShot(800, self._verify_xbox_connection)
            self.console.log("Xbox controller starting…", "info")
        except Exception as e:
            self.console.log(f"Xbox connection failed: {e}", "error")

    def _verify_xbox_connection(self):
        """Check if the Xbox process is still alive after startup."""
        if self.controller.is_xbox_connected:
            self.console.log("Xbox controller connected", "success")
        else:
            self.console.log(
                "Xbox controller not found — is a controller plugged in?", "warning"
            )
            # Clean up the dead process
            self.controller.disconnect_xbox()

    def disconnect_xbox(self):
        self.controller.disconnect_xbox()
        self.console.log("Xbox controller disconnected", "info")

    def open_xbox_editor(self):
        mapping_file = self.settings.get("xbox.mapping_file", "current_button_mapping.json")
        editor = XboxMappingEditor(mapping_file, parent=self)
        editor.exec()

    # ════════════════════════════════════════════════════════════════
    #  HARDWARE DISCONNECT HANDLER
    # ════════════════════════════════════════════════════════════════

    def _on_hardware_disconnect(self, stage_name: str):
        """Handle unexpected hardware disconnect."""
        self.console.log(f"⚠ {stage_name} stage disconnected unexpectedly!", "error")

    # ════════════════════════════════════════════════════════════════
    #  GLOBAL KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════

    def _setup_global_shortcuts(self):
        """Set up global keyboard jog shortcuts."""
        self._global_xy_step = 500
        self._global_z_step = 0.1

    def keyPressEvent(self, event: QKeyEvent):
        """Global keyboard shortcuts for jogging in any tab."""
        from PySide6.QtWidgets import QLineEdit, QTextEdit, QSpinBox, QDoubleSpinBox
        focus = self.focusWidget()
        if isinstance(focus, (QLineEdit, QTextEdit, QSpinBox, QDoubleSpinBox)):
            super().keyPressEvent(event)
            return

        key = event.key()
        handled = False

        if key in (Qt.Key.Key_Left, Qt.Key.Key_Right, Qt.Key.Key_Up, Qt.Key.Key_Down):
            if self.controller.is_xy_connected and not event.isAutoRepeat():
                dx, dy = 0, 0
                if key == Qt.Key.Key_Left:
                    dx = -1
                elif key == Qt.Key.Key_Right:
                    dx = 1
                elif key == Qt.Key.Key_Up:
                    dy = -1
                elif key == Qt.Key.Key_Down:
                    dy = 1
                self._global_jog_xy(dx, dy)
                handled = True

        elif key == Qt.Key.Key_PageUp and not event.isAutoRepeat():
            if self.controller.is_zp_connected:
                self.controller.move_z_relative(-self._global_z_step)
                handled = True
        elif key == Qt.Key.Key_PageDown and not event.isAutoRepeat():
            if self.controller.is_zp_connected:
                self.controller.move_z_relative(self._global_z_step)
                handled = True

        elif key == Qt.Key.Key_Home and not event.isAutoRepeat():
            self.controller.move_xy_absolute(0, 0, from_zero_ref=True)
            self.controller.move_z_absolute(0, from_zero_ref=True)
            handled = True

        elif key == Qt.Key.Key_Escape:
            if self.controller.zp_stage:
                try:
                    self.controller.zp_stage.emergency_stop()
                    self.console.log("EMERGENCY STOP sent!", "error")
                except Exception:
                    pass
            handled = True

        if not handled:
            super().keyPressEvent(event)

    def _global_jog_xy(self, dx: int, dy: int):
        pos = self.controller.get_xy_position(cached=True)
        if pos[0] is None:
            return
        zx = self.controller.zero_position["x"]
        zy = self.controller.zero_position["y"]
        target_x = (pos[0] - zx) + dx * self._global_xy_step
        target_y = (pos[1] - zy) + dy * self._global_xy_step
        self.controller.move_xy_absolute(target_x, target_y, from_zero_ref=True)

    # ════════════════════════════════════════════════════════════════
    #  PRINT RESUME CHECK
    # ════════════════════════════════════════════════════════════════

    def _check_print_resume(self):
        resume_data = load_print_progress()
        if resume_data is None:
            return

        job = resume_data["job"]
        step = resume_data["current_step"]
        saved_at = resume_data.get("saved_at", "unknown time")

        reply = QMessageBox.question(
            self,
            "Resume Print?",
            f"Found saved print progress:\n\n"
            f"  Job: {job.name}\n"
            f"  Progress: {step}/{job.total_steps} commands\n"
            f"  Saved at: {saved_at}\n\n"
            f"Would you like to resume this print?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )

        if reply == QMessageBox.StandardButton.Yes:
            self.console.log(f"Resuming print: {job.name} from step {step}", "success")
            self._switch_page(self.PAGE_PRINT)
            if hasattr(self.print_setup_page, 'resume_print'):
                self.print_setup_page.resume_print(resume_data)
        else:
            clear_print_progress()
            self.console.log("Discarded saved print progress", "info")

    # ════════════════════════════════════════════════════════════════
    #  SETTINGS PERSISTENCE & WINDOW EVENTS
    # ════════════════════════════════════════════════════════════════

    def _apply_settings(self):
        s = self.settings
        x = s.get("window.x", 100)
        y = s.get("window.y", 100)
        w = s.get("window.width", 1200)
        h = s.get("window.height", 800)
        self.setGeometry(x, y, w, h)

        tab_idx = s.get("window.active_tab", 0)
        if 0 <= tab_idx < len(self._page_widgets):
            self._switch_page(tab_idx)

        splitter_sizes = s.get("window.splitter_sizes", None)
        if splitter_sizes and hasattr(self, '_splitter'):
            self._splitter.setSizes(splitter_sizes)

        saved_speeds = s.get_section("speeds")
        if saved_speeds:
            try:
                if "xy" in saved_speeds and hasattr(self.controller, 'xy_jog'):
                    self.controller.xy_jog.xy_speed = float(saved_speeds["xy"])
                if "z" in saved_speeds and hasattr(self.controller, 'zp_jog'):
                    self.controller.zp_jog.z_speed = float(saved_speeds["z"])
                if "p" in saved_speeds and hasattr(self.controller, 'zp_jog'):
                    self.controller.zp_jog.p_speed = float(saved_speeds["p"])
            except (AttributeError, TypeError, ValueError):
                pass  # Jog controllers may not be initialized yet

    def save_settings(self):
        s = self.settings
        geo = self.geometry()
        s.set("window.x", geo.x())
        s.set("window.y", geo.y())
        s.set("window.width", geo.width())
        s.set("window.height", geo.height())
        s.set("window.active_tab", self._current_page_index)

        if hasattr(self, '_splitter'):
            s.set("window.splitter_sizes", self._splitter.sizes())

        s.set("simulation.simulate_xy", self.controller.simulate_xy)
        s.set("simulation.simulate_zp", self.controller.simulate_zp)

        speeds = self.controller.get_speed_info()
        s.set("speeds.xy", speeds["xy"])
        s.set("speeds.z", speeds["z"])
        s.set("speeds.p", speeds["p"])

        s.set_section("zero_position", self.controller.zero_position)
        s.set_section("safety_limits", self.controller.safety_limits.to_dict())
        s.save()

    def closeEvent(self, event):
        self.update_timer.stop()
        self.save_settings()
        self.controller.shutdown()
        event.accept()

    def resizeEvent(self, event):
        """Adapt context panel width for smaller screens."""
        super().resizeEvent(event)
        w = event.size().width()
        # On narrow windows, shrink context panel
        if w < 1000:
            AppSettings.LEFT_BOX_WIDTH = 200
        elif w < 1300:
            AppSettings.LEFT_BOX_WIDTH = 230
        else:
            AppSettings.LEFT_BOX_WIDTH = 260
        # If context panel is currently open, resize it live
        if self.ui_extraLeftBox.width() > 0:
            self.ui_extraLeftBox.setMinimumWidth(AppSettings.LEFT_BOX_WIDTH)
            self.ui_extraLeftBox.setMaximumWidth(AppSettings.LEFT_BOX_WIDTH)
