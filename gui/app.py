"""
Main Application Window — PyDracula-inspired layout for MEBP bioprinter.

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
    📈  Print Monitor   — live trajectory, plate progress, recordings (v7.1)
    ⚙️  Settings        — connections, safety, polling, logging

Each page provides:
    - get_context_widget() → QWidget for the extra-left panel
    - get_page_title() → str for the top bar
    - get_page_subtitle() → str

v7.1 Session I additions:
    - Print Monitor page (6th nav button) — P8.34
    - WorkspaceConfig shared state passing — P8.35
    - PrintRecorder wiring (auto-start/stop) — P7.4, P7.5
    - Recording browser + replay overlay — P7.6, P7.7

v7.1.1 Improvements:
    - Menu icons visible when sidebar collapsed (centered icon, left-aligned icon+label when expanded)
    - XY positions displayed in microns (µm) throughout, configurable via microsteps_per_micron
    - Centralized color scheme via styles.py COLORS dict
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
from SupportClasses.PrintRecorder import PrintRecorder
from gui.styles import DARK_THEME, COLORS
from gui.ui_functions import UIFunctions, AppSettings
from gui.unit_helpers import (
    steps_to_um, um_to_steps, format_um,
    get_microsteps_per_micron_from_protocol,
    DEFAULT_MICROSTEPS_PER_MICRON,
)
from gui.pages.dashboard import DashboardPage
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.settings_page import SettingsPage
from gui.pages.print_monitor import PrintMonitorPage
from gui.widgets.console_log import ConsoleLogWidget
from gui.widgets.xbox_mapping_editor import XboxMappingEditor

logger = logging.getLogger(__name__)


# ── Emoji → colored pixmap icon helper ───────────────────────────
def _make_text_icon(text: str, size: int = 24, color: str = "#a6adc8") -> QIcon:
    """Create a QIcon from a text character (emoji or symbol)."""
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setPen(QColor(color))
    painter.setFont(QFont("Segoe UI Emoji", int(size * 0.6)))
    painter.drawText(pixmap.rect(), Qt.AlignCenter, text)
    painter.end()
    return QIcon(pixmap)


class MainWindow(QMainWindow):
    """MEBP main application window with PyDracula-style sidebar navigation."""

    def __init__(self, controller: StageController, settings: Settings,
                 print_history: PrintHistory | None = None,
                 recorder: PrintRecorder | None = None):
        super().__init__()
        self.controller = controller
        self.settings = settings
        self.print_history = print_history
        self.recorder = recorder

        # Menu button references
        self._menu_buttons: list[QPushButton] = []
        self._page_widgets: list[QWidget] = []
        self._current_page_index = 0

        # Microsteps-per-micron conversion factor
        # Priority: 1) controller protocol  2) settings.json  3) default (10.0)
        self._microsteps_per_micron: float = self._resolve_microsteps_per_micron()
        self._protocol_checked = False  # Set True after first XY connect

        self.setWindowTitle("MEBP Bioprinter — v7.1")
        self.setMinimumSize(1100, 700)
        self.resize(1400, 850)

        # Apply dark theme
        self.setStyleSheet(DARK_THEME)

        # Build the UI shell
        self._build_ui()
        self._build_bottom_bar()
        self._create_pages()
        self._setup_timers()

        # Select the dashboard by default
        self._navigate_to(0)

        logger.info("MainWindow initialized")

    @property
    def microsteps_per_micron(self) -> float:
        """Get the microsteps-per-micron conversion factor."""
        return self._microsteps_per_micron

    @microsteps_per_micron.setter
    def microsteps_per_micron(self, value: float):
        """Set the conversion factor and propagate to pages."""
        self._microsteps_per_micron = max(0.001, value)
        self.settings.set("stage.microsteps_per_micron", self._microsteps_per_micron)
        # Notify pages that need to update their displays
        for page in self._page_widgets:
            if hasattr(page, 'set_microsteps_per_micron'):
                page.set_microsteps_per_micron(self._microsteps_per_micron)
        logger.info(f"microsteps_per_micron set to {self._microsteps_per_micron}")

    def _resolve_microsteps_per_micron(self) -> float:
        """
        Resolve microsteps_per_micron from best available source.

        Priority:
            1. Controller protocol JSON (if XY stage already connected)
            2. settings.json  →  stage.microsteps_per_micron
            3. DEFAULT_MICROSTEPS_PER_MICRON (10.0)
        """
        # Try connected protocol first
        proto_val = self._try_load_from_protocol()
        if proto_val is not None:
            logger.info(f"microsteps_per_micron from protocol: {proto_val}")
            self.settings.set("stage.microsteps_per_micron", proto_val)
            return proto_val

        # Try settings
        saved = self.settings.get("stage.microsteps_per_micron")
        if saved is not None:
            try:
                val = float(saved)
                if val > 0:
                    return val
            except (TypeError, ValueError):
                pass

        return DEFAULT_MICROSTEPS_PER_MICRON

    def _try_load_from_protocol(self) -> float | None:
        """Attempt to read microsteps_per_micron from XY stage protocol."""
        xy = getattr(self.controller, 'xy_stage', None)
        if xy is None:
            return None
        protocol = getattr(xy, '_protocol', None)
        return get_microsteps_per_micron_from_protocol(protocol)

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ════════════════════════════════════════════════════════════════

    def _build_ui(self):
        """Build the main UI shell: left menu, context panel, content area."""
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

        # Create workflow buttons (icon, label stored separately for toggle)
        menu_items = [
            ("btn_dashboard", "📊", "Dashboard"),
            ("btn_jog",       "🕹️", "Jog Control"),
            ("btn_calibrate", "📐", "Calibration"),
            ("btn_print",     "🖨️", "Print Setup"),
            ("btn_monitor",   "📈", "Print Monitor"),
        ]
        for obj_name, icon_text, label_text in menu_items:
            btn = self._make_menu_button(obj_name, icon_text, label_text)
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

    def _make_menu_button(self, obj_name: str, icon_text: str, label_text: str) -> QPushButton:
        """
        Create a left-menu navigation button.

        Stores icon and label text separately so the toggle can switch between:
        - Collapsed (60px): centered icon only
        - Expanded (200px): left-aligned icon + label
        """
        # Start in collapsed state: icon only, centered
        btn = QPushButton(icon_text)
        btn.setObjectName(obj_name)
        btn.setMinimumHeight(45)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(label_text)
        btn.setFont(QFont("Segoe UI Emoji", 14))
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn.clicked.connect(self._on_menu_click)

        # Store icon/label for toggle switching
        btn._icon_text = icon_text
        btn._label_text = label_text

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

        self.sb_xy = QLabel("XY: — , — µm")
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
        pages = [
            DashboardPage(self.controller, self.print_history),
            JogControlPage(self.controller),
            CalibrationPage(self.controller, settings=self.settings),
            PrintSetupPage(self.controller),
            SettingsPage(self.controller, self.settings),
            PrintMonitorPage(self.controller, self.settings),
        ]

        # Wire recorder to print manager and monitor if available
        if self.recorder:
            monitor = pages[5]
            if hasattr(monitor, 'set_recorder'):
                monitor.set_recorder(self.recorder)
            setup = pages[3]
            if hasattr(setup, 'print_manager') and setup.print_manager:
                setup.print_manager.recorder = self.recorder

        for page in pages:
            self._page_widgets.append(page)
            self._page_stack.addWidget(page)

            # Create context panel (wrapped in scroll area)
            ctx = None
            if hasattr(page, 'get_context_widget'):
                ctx = page.get_context_widget()
            if ctx is not None:
                scroll = QScrollArea()
                scroll.setObjectName("contextScrollArea")
                scroll.setWidgetResizable(True)
                scroll.setWidget(ctx)
                self._context_stack.addWidget(scroll)
            else:
                # Placeholder
                placeholder = QWidget()
                self._context_stack.addWidget(placeholder)

            # Propagate microsteps_per_micron to pages that support it
            if hasattr(page, 'set_microsteps_per_micron'):
                page.set_microsteps_per_micron(self._microsteps_per_micron)

    # ════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ════════════════════════════════════════════════════════════════

    def _on_menu_click(self):
        """Handle menu button click: navigate to the corresponding page."""
        btn = self.sender()
        if not btn:
            return

        # Map button names to page indices
        btn_map = {
            "btn_dashboard": 0,
            "btn_jog": 1,
            "btn_calibrate": 2,
            "btn_print": 3,
            "btn_monitor": 5,
            "btn_settings": 4,
        }
        index = btn_map.get(btn.objectName(), 0)
        self._navigate_to(index)

    def _navigate_to(self, index: int):
        """Switch to a page by index."""
        if index < 0 or index >= len(self._page_widgets):
            return

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
            titles = ["Dashboard", "Jog Control", "Calibration", "Print Setup",
                       "Settings", "Print Monitor"]
            title = titles[index] if index < len(titles) else title
        self._page_title.setText(title)

        # Update context panel title
        context_titles = ["Dashboard", "Jog Settings", "Calibration",
                          "Print Settings", "Settings", "Recordings"]
        self._context_title.setText(
            context_titles[index] if index < len(context_titles) else "Settings"
        )

        # Update menu button styling
        btn_names = ["btn_dashboard", "btn_jog", "btn_calibrate",
                     "btn_print", "btn_monitor", "btn_settings"]
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

        # Re-check protocol on first XY connect to pick up microsteps_per_micron
        if xy_ok and not self._protocol_checked:
            self._protocol_checked = True
            proto_val = self._try_load_from_protocol()
            if proto_val is not None and proto_val != self._microsteps_per_micron:
                self.microsteps_per_micron = proto_val
                logger.info(
                    f"Updated µsteps/µm from protocol on connect: {proto_val}")

        # Position readouts (XY in µm, ZP in mm)
        try:
            xy = self.controller.get_xy_position(cached=True)
            zp = self.controller.get_zp_position(cached=True)
            speeds = self.controller.get_speed_info()

            if xy[0] is not None:
                zx = xy[0] - self.controller.zero_position["x"]
                zy = xy[1] - self.controller.zero_position["y"]
                # Convert steps → microns for display
                ux = steps_to_um(zx, self._microsteps_per_micron)
                uy = steps_to_um(zy, self._microsteps_per_micron)
                self.sb_xy.setText(f"XY: {ux:,.1f} , {uy:,.1f} µm")
            else:
                self.sb_xy.setText("XY: — , — µm")

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
        if dot is None:
            return
        if connected:
            dot.setObjectName("connDotOn")
            if lbl:
                lbl.setObjectName("connLabelOn")
        else:
            dot.setObjectName("connDotOff")
            if lbl:
                lbl.setObjectName("connLabelOff")
        # Force style refresh
        dot.setStyleSheet(dot.styleSheet())
        if lbl:
            lbl.setStyleSheet(lbl.styleSheet())

    # ════════════════════════════════════════════════════════════════
    #  KEYBOARD SHORTCUTS
    # ════════════════════════════════════════════════════════════════

    def keyPressEvent(self, event: QKeyEvent):
        """Global keyboard shortcuts."""
        if event.key() == Qt.Key.Key_Escape:
            # Emergency stop
            if self.controller.zp_stage:
                try:
                    self.controller.zp_stage.emergency_stop()
                    logger.warning("EMERGENCY STOP via Escape key")
                except Exception as e:
                    logger.error(f"E-stop failed: {e}")
        else:
            # Propagate to active page
            page = self._page_widgets[self._current_page_index]
            if hasattr(page, 'keyPressEvent'):
                page.keyPressEvent(event)
            else:
                super().keyPressEvent(event)

    # ════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        """Clean shutdown."""
        self.update_timer.stop()
        if self.recorder and self.recorder.is_recording:
            self.recorder.stop_recording()
        self.controller.shutdown()
        event.accept()