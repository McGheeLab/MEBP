"""
app.py — MEBP Main Window with PyDracula-style sidebar navigation.

v7.2.3 changes:
    - Hardware Setup page added as page 0 (🔧)
    - Pages 1-5 gated until hardware config is valid
    - Settings page (⚙️) always accessible
    - HardwareConfig propagated to all pages
    - Bottom bar pump readouts in µL when syringe is configured
    - Hardware config persisted in settings.json
    - v7.2.3: Job pipeline: PrintSetup → app → PrintMonitor
    - v7.2.3: Execution controls wired from Monitor to PrintManager
    - v7.2.3: PrintManager callbacks forwarded to Monitor for live updates
"""

from __future__ import annotations

import logging
import os

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFrame,
    QPushButton, QLabel, QStackedWidget, QScrollArea, QSizePolicy,
    QSplitter,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QIcon, QPixmap, QPainter, QColor, QKeyEvent

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings
from SupportClasses.PrintHistory import PrintHistory
from SupportClasses.PrintManager import load_print_progress, clear_print_progress
from SupportClasses.PrintRecorder import PrintRecorder
from SupportClasses.HardwareConfig import HardwareConfig
from gui.styles import DARK_THEME, COLORS
from gui.ui_functions import UIFunctions, AppSettings
from gui.unit_helpers import (
    steps_to_um, um_to_steps, format_um,
    get_microsteps_per_micron_from_protocol,
    DEFAULT_MICROSTEPS_PER_MICRON,
)
from gui.pages.hardware_setup import HardwareSetupPage
from gui.pages.dashboard import DashboardPage
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.settings_page import SettingsPage
from gui.pages.print_monitor import PrintMonitorPage
from gui.pages.helper_functions import HelperFunctionsPage
from gui.pages.print_results import PrintResultsPage  # v7.2.6
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

        self._propagating_config = False
        self.controller = controller
        self.settings = settings
        self.print_history = print_history
        self.recorder = recorder

        # Menu button references
        self._menu_buttons: list[QPushButton] = []
        self._page_widgets: list[QWidget] = []
        self._current_page_index = 0

        # v7.2: Hardware configuration
        self._hardware_config: HardwareConfig | None = None

        # Microsteps-per-micron conversion factor
        self._microsteps_per_micron: float = self._resolve_microsteps_per_micron()
        self._protocol_checked = False

        self.setWindowTitle("MEBP Bioprinter — v7.2.3")
        self.setMinimumSize(1100, 700)
        self.resize(1400, 850)

        # Apply dark theme
        self.setStyleSheet(DARK_THEME)

        # Build the UI shell
        self._build_ui()
        self._build_bottom_bar()
        self._create_pages()
        self._setup_timers()

        # Start on Hardware Setup page
        self._navigate_to(0)

        logger.info("MainWindow initialized (v7.2.3)")

    # ════════════════════════════════════════════════════════════════
    #  MICROSTEPS-PER-MICRON PROPERTY
    # ════════════════════════════════════════════════════════════════

    @property
    def microsteps_per_micron(self) -> float:
        return self._microsteps_per_micron

    @microsteps_per_micron.setter
    def microsteps_per_micron(self, value: float):
        self._microsteps_per_micron = max(0.001, value)
        self.settings.set("stage.microsteps_per_micron", self._microsteps_per_micron)
        for page in self._page_widgets:
            # v7.2.5 NOTE: Jog page now sends microns directly to the
            # Prior controller via move_xy_relative_um(). The microsteps_per_micron
            # factor is kept for backward compat with other pages that may need it.
            # v7.2.5 NOTE: Jog page now sends microns directly to the
            # Prior controller via move_xy_relative_um(). The microsteps_per_micron
            # factor is kept for backward compat with other pages that may need it.
            # v7.2.5 NOTE: Jog page now sends microns directly to the
            # Prior controller via move_xy_relative_um(). The microsteps_per_micron
            # factor is kept for backward compat with other pages that may need it.
            if hasattr(page, 'set_microsteps_per_micron'):
                page.set_microsteps_per_micron(self._microsteps_per_micron)
        logger.info(f"microsteps_per_micron set to {self._microsteps_per_micron}")

    def _resolve_microsteps_per_micron(self) -> float:
        proto_val = self._try_load_from_protocol()
        if proto_val is not None:
            self.settings.set("stage.microsteps_per_micron", proto_val)
            return proto_val
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

        # Logo
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
        self._logo_text.setVisible(False)
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

        # Page buttons — Hardware Setup is first
        menu_items = [
            ("btn_hardware",  "🔧", "Hardware Setup"),
            ("btn_dashboard", "📊", "Dashboard"),
            ("btn_jog",       "🕹️", "Jog Control"),
            ("btn_calibrate", "📐", "Calibration"),
            ("btn_print",     "🖨️", "Print Setup"),
            ("btn_monitor",   "📈", "Print Monitor"),            ("btn_results",  "📋", "Print Results"),  # v7.2.6

            ("btn_helpers",  "🧰", "Helper Functions"),
        ]
        for obj_name, icon_text, label_text in menu_items:
            btn = self._make_menu_button(obj_name, icon_text, label_text)
            top_menu_layout.addWidget(btn)
            self._menu_buttons.append(btn)

        left_layout.addWidget(self.ui_topMenu, 0, Qt.AlignTop)
        left_layout.addStretch()

        # Bottom menu (settings)
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

        title_frame = QWidget()
        title_layout = QVBoxLayout(title_frame)
        title_layout.setSpacing(0)
        title_layout.setContentsMargins(0, 4, 0, 4)

        self._page_title = QLabel("Hardware Setup")
        self._page_title.setObjectName("pageTitle")
        self._page_title.setFont(QFont("Segoe UI", 12, QFont.Bold))
        title_layout.addWidget(self._page_title)

        top_bar_layout.addWidget(title_frame)
        top_bar_layout.addStretch()

        # Connection dots
        conn_frame = QFrame()
        conn_frame.setObjectName("connStatusFrame")
        conn_layout = QHBoxLayout(conn_frame)
        conn_layout.setSpacing(12)
        conn_layout.setContentsMargins(0, 0, 0, 0)
        conn_layout.addWidget(self._make_conn_dot("XY"))
        conn_layout.addWidget(self._make_conn_dot("ZP"))
        conn_layout.addWidget(self._make_conn_dot("Xbox"))
        top_bar_layout.addWidget(conn_frame)

        # Context panel toggle
        btn_context = QPushButton("☰")
        btn_context.setObjectName("extraBtn")
        btn_context.setFixedSize(32, 32)
        btn_context.setCursor(Qt.PointingHandCursor)
        btn_context.setToolTip("Toggle context panel")
        btn_context.clicked.connect(lambda: UIFunctions.toggleLeftBox(self))
        top_bar_layout.addWidget(btn_context)

        content_layout.addWidget(top_bar)

        # Content splitter (pages + console)
        self._splitter = QSplitter(Qt.Vertical)
        self._splitter.setObjectName("contentBottom")

        # Page stack
        self._page_stack = QStackedWidget()
        self._page_stack.setObjectName("pagesContainer")
        self._page_stack.setMinimumHeight(200)  # v7.2.6
        self._splitter.addWidget(self._page_stack)

        # Console log
        self.console = ConsoleLogWidget()
        self.console.setMinimumHeight(40)  # v7.2.6
        self._splitter.addWidget(self.console)

        self._splitter.setStretchFactor(0, 5)
        self._splitter.setStretchFactor(1, 1)

        # v7.2.6: Prevent splitter collapse crash
        self._splitter.setChildrenCollapsible(False)

        content_layout.addWidget(self._splitter)

        app_layout.addWidget(content_frame)

    def _make_menu_button(self, obj_name: str, icon_text: str,
                          label: str) -> QPushButton:
        """Create a sidebar navigation button."""
        btn = QPushButton(icon_text)
        btn.setObjectName(obj_name)
        btn.setMinimumHeight(44)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip(label)
        btn.clicked.connect(self._on_menu_click)
        # Store icon/label for UIFunctions.updateMenuButtonStates
        btn._icon_text = icon_text
        btn._label_text = label
        return btn

    def _make_conn_dot(self, label: str) -> QWidget:
        """Create a connection status indicator dot + label."""
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

        setattr(self, f"_dot_{label.lower()}", dot)
        setattr(self, f"_lbl_{label.lower()}", lbl)
        return frame

    def _build_bottom_bar(self):
        """Build the bottom status bar with µL pump readouts."""
        self.status_bar = self.statusBar()
        self.status_bar.setObjectName("bottomBar")

        mono = QFont("Consolas", 9)

        self.sb_xy = QLabel("XY: — , — µm")
        self.sb_xy.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_xy)

        self.sb_z = QLabel("Z: —")
        self.sb_z.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_z)

        self.sb_p1 = QLabel("P1: —")
        self.sb_p1.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_p1)

        self.sb_p2 = QLabel("P2: —")
        self.sb_p2.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_p2)

        self.sb_p3 = QLabel("P3: —")
        self.sb_p3.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_p3)

        self.sb_speed = QLabel("Speed XY:— Z:— P:—")
        self.sb_speed.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_speed)

        self.sb_safety = QLabel("🛡️ ON")
        self.sb_safety.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_safety)

        self.sb_log_count = QLabel("📝 0")
        self.sb_log_count.setFont(mono)
        self.status_bar.addPermanentWidget(self.sb_log_count)

    # ════════════════════════════════════════════════════════════════
    #  PAGE CREATION
    # ════════════════════════════════════════════════════════════════

    def _create_pages(self):
        """
        Instantiate all page widgets.

        Page 0: Hardware Setup (always enabled)
        Pages 1-5: Gated until hardware config is valid
        Page 6: Settings (always enabled)

        v7.2.3: Also wires the job pipeline (Setup → Monitor)
        and execution control signals (Monitor → PrintManager).
        """
        # Restore hardware config from settings
        self._hardware_config = self._restore_hardware_config()

        pages = [
            HardwareSetupPage(),                                          # 0
            DashboardPage(self.controller, self.print_history, settings=self.settings),  # 1
            JogControlPage(self.controller),                              # 2
            CalibrationPage(self.controller, settings=self.settings),     # 3
            PrintSetupPage(self.controller),                              # 4
            PrintMonitorPage(self.controller, self.settings),             # 5
            PrintResultsPage(self.controller, self.settings),             # 6  v7.2.6
            HelperFunctionsPage(),                                        # 7  v7.2.7

            SettingsPage(self.controller, self.settings),                 # 8  ← was 7  v7.2.6
        ]

        # Wire Hardware Setup signals
        hw_page = pages[0]
        hw_page.config_changed.connect(self._on_hardware_config_changed)
        hw_page.config_validated.connect(self._on_hardware_validated)

        # Restore saved config to Hardware Setup page
        if self._hardware_config:
            hw_page.set_config(self._hardware_config)

        # Wire recorder
        if self.recorder:
            monitor = pages[5]
            if hasattr(monitor, 'set_recorder'):
                monitor.set_recorder(self.recorder)

            # v7.2.6: wire recorder to results
            results_page = pages[6]
            if hasattr(results_page, 'set_recorder'):
                results_page.set_recorder(self.recorder)
            setup = pages[4]
            if hasattr(setup, 'print_manager') and setup.print_manager:
                setup.print_manager.recorder = self.recorder

        # Register all pages with the stacked widgets
        for page in pages:
            self._page_widgets.append(page)
            self._page_stack.addWidget(page)

            # Create context panel
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
                placeholder = QWidget()
                self._context_stack.addWidget(placeholder)

            # Propagate microsteps_per_micron
            if hasattr(page, 'set_microsteps_per_micron'):
                page.set_microsteps_per_micron(self._microsteps_per_micron)

        # Propagate existing hardware config to pages
        if self._hardware_config:
            self._propagate_hardware_config(self._hardware_config)

        # Initial page gating
        is_valid = (self._hardware_config is not None
                    and self._hardware_config.is_valid)
        self._update_page_gating(is_valid)

        # ── v7.2.3: Wire job pipeline and execution controls ─────
        self._wire_job_pipeline()
        self._wire_print_manager_to_monitor()


        # v7.2.7: Wire helper functions signal
        for pg in self._page_widgets:
            if hasattr(pg, 'print_file_created'):
                pg.print_file_created.connect(self._on_helper_print_created)
                break

    # ════════════════════════════════════════════════════════════════
    #  v7.2.3: JOB PIPELINE & EXECUTION CONTROL WIRING
    # ════════════════════════════════════════════════════════════════

    def _wire_job_pipeline(self):
        """
        Wire the print job flow from PrintSetupPage through to
        PrintMonitorPage, and connect execution control signals.

        Signal flow:
            PrintSetup.job_ready(PrintJob)
                → app._send_job_to_monitor(job)
                → Monitor.receive_job(job)
                → auto-switch to page 5

            PrintSetup.navigate_to_page(int)
                → app._switch_page(index)

            Monitor.start_requested(PrintJob)
                → app._on_monitor_start(job)
                → PrintManager.start(job)

            Monitor.pause_requested()  → PrintManager.pause()
            Monitor.resume_requested() → PrintManager.resume()
            Monitor.abort_requested()  → PrintManager.abort()
        """
        setup_page = self._page_widgets[4]   # PrintSetupPage
        monitor_page = self._page_widgets[5]  # PrintMonitorPage

        # Job pipeline: PrintSetup → app → PrintMonitor
        if hasattr(setup_page, 'job_ready'):
            setup_page.job_ready.connect(self._send_job_to_monitor)

        # Navigation: "Edit Hardware Setup" button → switch to page 0
        if hasattr(setup_page, 'navigate_to_page'):
            setup_page.navigate_to_page.connect(self._switch_page)

        # Execution control signals from Monitor
        if hasattr(monitor_page, 'start_requested'):
            monitor_page.start_requested.connect(self._on_monitor_start)
        if hasattr(monitor_page, 'pause_requested'):
            monitor_page.pause_requested.connect(self._on_monitor_pause)
        if hasattr(monitor_page, 'resume_requested'):
            monitor_page.resume_requested.connect(self._on_monitor_resume)
        if hasattr(monitor_page, 'abort_requested'):
            monitor_page.abort_requested.connect(self._on_monitor_abort)

    def _wire_print_manager_to_monitor(self):
        """v7.2.6: Thread-safe signal bridge with QueuedConnection.

        PrintManager callbacks fire from a daemon thread.
        Direct widget calls from threads cause SIGSEGV on macOS/PySide6.
        We bounce through a QObject signal bridge so all GUI updates
        execute on the main thread.
        """
        setup_page = self._page_widgets[4]
        monitor_page = self._page_widgets[5]

        if not hasattr(setup_page, "print_manager"):
            logger.warning("_wire_print_manager_to_monitor: no print_manager")
            return

        pm = setup_page.print_manager

        # Create signal bridge owned by app.py
        try:
            from gui.pages.print_setup import PrintSignalBridge
        except ImportError:
            from PySide6.QtCore import QObject, Signal as _Sig
            class PrintSignalBridge(QObject):
                progress_signal = _Sig(int, int, str)
                state_signal = _Sig(object)

        self._print_signal_bridge = PrintSignalBridge(self)
        bridge = self._print_signal_bridge

        # Connect bridge → monitor with QueuedConnection (thread-safe)
        from PySide6.QtCore import Qt
        if hasattr(monitor_page, "on_print_progress"):
            bridge.progress_signal.connect(
                monitor_page.on_print_progress,
                type=Qt.ConnectionType.QueuedConnection,
            )
        if hasattr(monitor_page, "on_print_state_changed"):
            bridge.state_signal.connect(
                monitor_page.on_print_state_changed,
                type=Qt.ConnectionType.QueuedConnection,
            )

        # Wire PrintManager callbacks → emit bridge signals
        # These lambdas may fire from the print thread — .emit() is thread-safe
        original_progress = pm.on_progress
        original_state = pm.on_state_changed

        def safe_progress(step, total, msg):
            try:
                if original_progress:
                    original_progress(step, total, msg)
            except Exception:
                pass
            try:
                bridge.progress_signal.emit(step, total, msg)
            except Exception:
                pass

        def safe_state(state):
            try:
                if original_state:
                    original_state(state)
            except Exception:
                pass
            try:
                bridge.state_signal.emit(state)
            except Exception:
                pass
            # v7.2.6: notify results page on completion
            try:
                from SupportClasses.PrintManager import PrintState as _PS
                if state == _PS.COMPLETED:
                    self._on_print_completed_v726()
            except Exception:
                pass

        pm.on_progress = safe_progress
        pm.on_state_changed = safe_state
        logger.info("Print manager -> monitor wired via thread-safe signal bridge")


    def _send_job_to_monitor(self, job):
        """
        v7.2.6: Forward job to monitor WITH visualization setup.

        Initializes plate overview, trajectory view, and syringe displays
        before the job is received by the monitor page.
        """
        monitor_page = self._page_widgets[5]
        setup_page = self._page_widgets[4]

        # ── Initialize monitor visualization ──────────────────────
        try:
            self._setup_monitor_visualization(setup_page, monitor_page, job)
        except Exception as exc:
            logger.error(f"Monitor viz setup error: {exc}", exc_info=True)

        if hasattr(monitor_page, 'receive_job'):
            monitor_page.receive_job(job)

        # Auto-switch to Print Monitor page
        self._switch_page(5)
    def _setup_monitor_visualization(self, setup_page, monitor_page, job):
        """v7.2.6: Feed plate/trajectory/syringe data to monitor widgets."""
        try:
            # ── 1. Plate Overview ─────────────────────────────────
            plate = None
            well_roles = {}
            print_wells = []

            tab_wells = getattr(setup_page, 'tab_wells', None)
            model = getattr(tab_wells, '_model', None) if tab_wells else None

            if model is not None:
                plate = getattr(model, 'plate', None)
                assignments = getattr(model, 'assignments', {})

                for name, assignment in assignments.items():
                    role = getattr(assignment, 'role', None)
                    if role is not None:
                        well_roles[name] = role
                        if getattr(role, 'value', '') == 'print':
                            print_wells.append(name)

            if plate is not None and hasattr(monitor_page, 'setup_plate'):
                well_diam = getattr(plate, 'well_diameter', 0.0)
                monitor_page.setup_plate(
                    plate=plate,
                    well_roles=well_roles if well_roles else None,
                    print_wells=print_wells if print_wells else None,
                    well_diameter_mm=well_diam,
                )
                logger.info(
                    f"Monitor plate: {len(print_wells)} print wells, "
                    f"diam={well_diam:.1f}mm")

            # ── 2. Trajectory View — path segments from job ───────
            if (hasattr(monitor_page, 'trajectory_view')
                    and hasattr(job, 'get_path_segments')):
                try:
                    segments = job.get_path_segments()
                    tv = monitor_page.trajectory_view
                    if segments:
                        if hasattr(tv, 'set_path_segments'):
                            tv.set_path_segments(segments)
                        elif hasattr(tv, 'load_path_segments'):
                            tv.load_path_segments(segments)
                        logger.info(f"Monitor trajectory: {len(segments)} segments")
                except Exception as exc:
                    logger.debug(f"Trajectory load skipped: {exc}")

            # ── 3. Syringe Display from HardwareConfig ────────────
            hw = getattr(self, '_hardware_config', None)
            if hw is not None and hasattr(monitor_page, 'syringe_panel'):
                try:
                    sp = monitor_page.syringe_panel
                    # Try different syringe panel APIs
                    workspace = getattr(setup_page, '_workspace', None)
                    if workspace:
                        pump_loadouts = getattr(workspace, 'pumps', None)
                        if pump_loadouts and hasattr(monitor_page, 'update_syringe_state'):
                            active = getattr(
                                getattr(job, 'settings', None), 'active_pump', 'P1')
                            monitor_page.update_syringe_state(pump_loadouts, active)
                except Exception as exc:
                    logger.debug(f"Syringe setup skipped: {exc}")

            # ── 4. Needle info label ──────────────────────────────
            if hw and hasattr(monitor_page, 'needle_label'):
                try:
                    needle = getattr(hw, 'needle', None)
                    if needle:
                        g = getattr(needle, 'gauge', '?')
                        l = getattr(needle, 'length_inches', '?')
                        d = getattr(needle, 'id_um', 0)
                        monitor_page.needle_label.setText(
                            f"Needle: {g}G x {l}\"  ID: {d:.0f} um")
                except Exception:
                    pass

            # ── 5. Push workspace to monitor ──────────────────────
            workspace = getattr(setup_page, '_workspace', None)
            if workspace and hasattr(monitor_page, '_workspace'):
                monitor_page._workspace = workspace
                if hasattr(monitor_page, '_update_needle_info'):
                    try:
                        monitor_page._update_needle_info()
                    except Exception:
                        pass

        except Exception as exc:
            logger.error(f"Monitor viz setup failed: {exc}", exc_info=True)


    def _on_monitor_start(self, job):
        """v7.3: Trajectory execution with configurable control mode.

        Execution modes (set in settings.json → execution.mode):
          "discrete"  — legacy command-based PrintManager (v7.0 behavior)
          "position"  — trajectory waypoints with position commands (DEFAULT)
          "kalman"    — velocity control via Kalman filter
          "pid"       — velocity control via PID controller

        Default is "position" — simplest and most reliable.
        """
        setup_page = self._page_widgets[4]
        if not hasattr(setup_page, "print_manager"):
            logger.error("No print_manager on setup page")
            return

        pm = setup_page.print_manager
        waypoints = getattr(job, 'trajectory_waypoints', None)

        # Read execution mode from settings (default: hybrid)
        exec_mode = "hybrid"
        if hasattr(self, '_settings') and self._settings:
            exec_mode = self._settings.get("execution.mode", "hybrid")
        logger.info(f"Execution mode: {exec_mode}")

        # ── Hybrid mode: plan-step-driven execution ─────────────
        plan = getattr(job, 'plan_of_action', None)
        if exec_mode == "hybrid" and plan is not None:
            try:
                from SupportClasses.PrintManager import (
                    HybridPlanExecutor, PrintState)
                import threading
                pm._set_state(PrintState.IDLE)
                pm._pause_event.set()

                executor = HybridPlanExecutor(
                    controller=pm.controller,
                    plan=plan,
                    well_model=getattr(job, 'well_setup', None),
                    plate=getattr(job, 'plate', None),
                    path_points=getattr(job, 'path_points', []),
                    settings=job.settings,
                    hw_config=getattr(job, 'hw_config', None),
                    recorder=pm.recorder,
                )

                # Estimate total print time
                try:
                    est = executor.estimate_time()
                    job.estimated_duration_s = est
                    logger.info(f"Hybrid estimated duration: {est:.1f}s")
                except Exception as e:
                    logger.warning(f"Time estimate failed: {e}")

                # Start recording
                if hasattr(pm, '_start_recorder'):
                    pm._start_recorder()

                def _hybrid_thread():
                    pm._set_state(PrintState.RUNNING)
                    try:
                        def on_prog(idx, total, msg):
                            if pm.on_progress:
                                pm.on_progress(idx, total, msg)

                        success = executor.execute(
                            pause_event=pm._pause_event,
                            on_progress=on_prog,
                        )
                        if success:
                            pm._set_state(PrintState.COMPLETED)
                        else:
                            pm._set_state(PrintState.ABORTED)
                    except Exception as exc:
                        logger.error(f"Hybrid exec error: {exc}",
                                     exc_info=True)
                        pm._set_state(PrintState.ERROR)
                    finally:
                        if hasattr(pm, '_stop_recorder'):
                            try:
                                pm._stop_recorder(pm.state.name.lower())
                            except Exception:
                                pass

                pm._thread = threading.Thread(
                    target=_hybrid_thread, daemon=True)
                pm._thread.start()
                logger.info(f"Hybrid execution started: {job.name}")
                return

            except Exception as exc:
                logger.error(f"Hybrid start failed: {exc}", exc_info=True)
                logger.info("Falling back to trajectory mode")
                exec_mode = "position"  # ensure trajectory fallback works

        # ── Discrete mode: use legacy PrintManager ────────────────
        if exec_mode == "discrete" or not waypoints or len(waypoints) == 0:
            try:
                pm.load_job(job)
                pm.start()
                logger.info(f"Print started (discrete): {job.name}")
            except Exception as exc:
                logger.error(f"PrintManager start failed: {exc}", exc_info=True)
            return

        # ── Trajectory modes: position / kalman / pid ─────────────
        logger.info(
            f"Starting trajectory ({exec_mode}): {job.name} "
            f"({len(waypoints)} waypoints)")
        try:
            from SupportClasses.PrintManager import PrintState
            import threading

            pm.job = job
            pm._abort_flag.clear()
            pm._pause_event.set()

            # Choose executor based on mode
            if exec_mode in ("kalman", "pid"):
                try:
                    from SupportClasses.VelocityExecutor import VelocityExecutor
                    tex = VelocityExecutor(
                        pm.controller, strategy=exec_mode,
                        recorder=pm.recorder)
                    logger.info(f"Using VelocityExecutor ({exec_mode})")
                except ImportError:
                    logger.warning("VelocityExecutor not available, using position mode")
                    exec_mode = "position"

            if exec_mode == "position":
                # Use the v7.1 TrajectoryExecutor — simplest, most reliable
                from SupportClasses.PrintManager import TrajectoryExecutor
                tex = TrajectoryExecutor(pm.controller, recorder=pm.recorder)
                logger.info("Using TrajectoryExecutor (position mode)")

            pm._trajectory_executor = tex

            # Start recording
            if hasattr(pm, '_start_recorder'):
                pm._start_recorder()

            def _traj_thread():
                pm._set_state(PrintState.RUNNING)
                try:
                    def on_prog(idx, total, msg):
                        if pm.on_progress:
                            pm.on_progress(idx, total, msg)

                    success = tex.execute(
                        waypoints=waypoints,
                        pause_event=pm._pause_event,
                        on_progress=on_prog,
                    )
                    if success:
                        pm._set_state(PrintState.COMPLETED)
                        if pm.on_progress:
                            pm.on_progress(len(waypoints), len(waypoints),
                                           "Complete!")
                    else:
                        pm._set_state(PrintState.ABORTED)
                except Exception as exc:
                    logger.error(f"Trajectory error: {exc}", exc_info=True)
                    pm._set_state(PrintState.ERROR)
                finally:
                    if hasattr(pm, '_stop_recorder'):
                        try:
                            pm._stop_recorder(pm.state.name.lower())
                        except Exception:
                            pass

            pm._thread = threading.Thread(target=_traj_thread, daemon=True)
            pm._thread.start()

        except Exception as exc:
            logger.error(f"Trajectory start failed: {exc}", exc_info=True)


    def _on_monitor_pause(self):
        """Monitor requested pause — forward to PrintManager."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.pause()

    def _on_monitor_resume(self):
        """Monitor requested resume — forward to PrintManager."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.resume()

    def _on_monitor_abort(self):
        """Monitor requested abort — forward to PrintManager."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.abort()

    def _on_print_completed_v726(self):
        """v7.2.6: On print completion, notify results page."""
        if len(self._page_widgets) > 6:
            results_page = self._page_widgets[6]
            if hasattr(results_page, 'load_latest_recording'):
                try:
                    results_page.load_latest_recording()
                    logger.info(
                        "v7.2.6: Loaded latest recording into results"
                    )
                except Exception as e:
                    logger.warning(
                        "v7.2.6: Failed to load recording: "
                        + str(e)
                    )


    # ════════════════════════════════════════════════════════════════
    #  v7.2.7: HELPER FUNCTIONS INTEGRATION
    # ════════════════════════════════════════════════════════════════

    def _on_helper_print_created(self, filename: str):
        """Helper Functions page created a print file — notify Print Objects tab."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, 'tab_objects'):
            tab = setup_page.tab_objects
            if hasattr(tab, '_load_file_by_name'):
                tab._load_file_by_name(filename)
            elif hasattr(tab, '_emit_prints_changed'):
                tab._emit_prints_changed()
        # Auto-switch to Print Setup page
        self._navigate_to(4)


    # ════════════════════════════════════════════════════════════════
    #  HARDWARE CONFIG MANAGEMENT
    # ════════════════════════════════════════════════════════════════

    def _on_hardware_config_changed(self, config: HardwareConfig):
        """Called when hardware setup changes. Propagates to all pages."""
        if self._propagating_config:
            return  # Guard against re-entrant calls
        self._propagating_config = True
        try:
            self._hardware_config = config
            self._propagate_hardware_config(config)
            self._save_hardware_config(config)
            logger.info(f"Hardware config updated: {config}")
        finally:
            self._propagating_config = False

    def _on_hardware_validated(self, is_valid: bool):
        """Called when hardware setup validity changes. Gates other pages."""
        self._update_page_gating(is_valid)
        if is_valid:
            logger.info("Hardware setup valid — all pages unlocked")
        else:
            logger.info("Hardware setup incomplete — pages locked")

    def _propagate_hardware_config(self, config: HardwareConfig):
        """Push hardware config to all pages and the controller.

        v7.2.4: Added per-page logging to audit propagation completeness.
        """
        if hasattr(self.controller, 'set_hardware_config'):
            self.controller.set_hardware_config(config)
            logger.debug("HW config → StageController")

        for i, page in enumerate(self._page_widgets):
            # Skip the hardware setup page — it's the SOURCE, not the target
            if isinstance(page, HardwareSetupPage):
                continue
            page_name = getattr(page, '_page_title_text', page.__class__.__name__)
            if hasattr(page, 'set_hardware_config'):
                try:
                    page.set_hardware_config(config)
                    logger.debug(f"HW config → Page {i}: {page_name}")
                except Exception as e:
                    logger.error(f"HW config propagation FAILED for Page {i} "
                                 f"({page_name}): {e}")
            else:
                logger.debug(f"HW config → Page {i}: {page_name} (no set_hardware_config)")

    def _update_page_gating(self, hardware_valid: bool):
        """Enable/disable navigation buttons for pages requiring hardware setup."""
        # Page indices: 0=Hardware, 1=Dashboard, 2=Jog, 3=Calibrate,
        #               4=Print, 5=Monitor, 6=Results, 7=Helpers, 8=Settings  # v7.2.6
        for i, btn in enumerate(self._menu_buttons):
            if i == 0:
                # Hardware Setup — always enabled
                btn.setEnabled(True)
                btn.setToolTip("Hardware Setup")
            elif i == 8 or btn.objectName() == "btn_settings":  # v7.2.7: settings at 8
                # Settings — always enabled
                btn.setEnabled(True)
                btn.setToolTip("Settings")
            else:
                btn.setEnabled(hardware_valid)
                if not hardware_valid:
                    btn.setToolTip("Complete Hardware Setup first")
                else:
                    btn.setToolTip("")

    def _restore_hardware_config(self) -> HardwareConfig | None:
        """Try to restore hardware config from settings."""
        try:
            hw_data = self.settings.get("hardware_config")
            if hw_data and isinstance(hw_data, dict):
                config = HardwareConfig.from_dict(hw_data)
                logger.info(f"Restored hardware config: {config}")
                return config
        except Exception as e:
            logger.warning(f"Failed to restore hardware config: {e}")
        return None

    def _save_hardware_config(self, config: HardwareConfig):
        """Persist hardware config to settings.json."""
        try:
            self.settings.set("hardware_config", config.to_dict())
            self.settings.save()
        except Exception as e:
            logger.warning(f"Failed to save hardware config: {e}")

    # ════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ════════════════════════════════════════════════════════════════

    def _on_menu_click(self):
        """Handle sidebar menu button click."""
        btn = self.sender()
        if not btn:
            return

        btn_map = {
            "btn_hardware":  0,
            "btn_dashboard": 1,
            "btn_jog":       2,
            "btn_calibrate": 3,
            "btn_print":     4,
            "btn_monitor":   5,
            "btn_results":   6,   # v7.2.6
            "btn_helpers":   7,   # v7.2.7
            "btn_settings":  8,   # v7.2.7
        }
        index = btn_map.get(btn.objectName(), 0)
        self._navigate_to(index)

    def _navigate_to(self, index: int):
        """Switch to the page at the given index."""
        if index < 0 or index >= len(self._page_widgets):
            return

        self._current_page_index = index
        self._page_stack.setCurrentIndex(index)
        self._context_stack.setCurrentIndex(index)

        page = self._page_widgets[index]
        title = "MEBP Bioprinter"
        if hasattr(page, 'get_page_title'):
            title = page.get_page_title()
        else:
            titles = ["Hardware Setup", "Dashboard", "Jog Control",
                      "Calibration", "Print Setup", "Print Monitor",
                      "Print Results", "Helper Functions",
                      "Settings"]
            title = titles[index] if index < len(titles) else title
        self._page_title.setText(title)

        context_titles = ["Hardware", "Dashboard", "Jog Settings",
                          "Calibration", "Print Settings", "Recordings",
                          "Results", "Helpers",
                          "Settings"]
        self._context_title.setText(
            context_titles[index] if index < len(context_titles) else "Settings"
        )

        # Highlight the active menu button
        for i, btn in enumerate(self._menu_buttons):
            if i == index:
                btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))
            else:
                btn.setStyleSheet(UIFunctions.deselectMenu(btn.styleSheet()))

        # Auto-show/hide context panel based on page
        if (hasattr(page, 'get_context_widget')
                and page.get_context_widget() is not None):
            if self.ui_extraLeftBox.width() == 0:
                UIFunctions.setLeftBoxWidth(self, AppSettings.LEFT_BOX_WIDTH)
        else:
            if self.ui_extraLeftBox.width() > 0:
                UIFunctions.setLeftBoxWidth(self, 0)

    def _switch_page(self, index: int):
        """
        Public alias for _navigate_to, used by page signals
        (e.g., PrintSetupPage.navigate_to_page).
        """
        self._navigate_to(index)

    # ════════════════════════════════════════════════════════════════
    #  TIMERS & STATUS UPDATES
    # ════════════════════════════════════════════════════════════════

    def _setup_timers(self):
        """Start the position/status polling timer."""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_status)
        interval = self.settings.get("polling.position_interval_ms", 300)
        self.update_timer.start(interval)

    def _update_status(self):
        """Update connection dots, position readouts, and page callbacks."""
        # Connection indicators
        xy_ok = self.controller.is_xy_connected
        zp_ok = self.controller.is_zp_connected
        self._update_conn_dot("xy", "on" if xy_ok else "off")
        self._update_conn_dot("zp", "on" if zp_ok else "off")

        # Xbox: tri-state — green/yellow/red
        _xbox_st = getattr(self.controller, "xbox_status", "disconnected")
        if callable(_xbox_st):
            _xbox_st = _xbox_st()
        if _xbox_st in ("connected", "alive"):
            self._update_conn_dot("xbox", "on")
        elif _xbox_st == "reconnecting":
            self._update_conn_dot("xbox", "warn")
        else:
            self._update_conn_dot("xbox", "off")

        # Protocol check on first XY connect
        if xy_ok and not self._protocol_checked:
            self._protocol_checked = True
            proto_val = self._try_load_from_protocol()
            if proto_val is not None and proto_val != self._microsteps_per_micron:
                self.microsteps_per_micron = proto_val

        # Position readouts
        try:
            xy = self.controller.get_xy_position(cached=True)
            zp = self.controller.get_zp_position(cached=True)
            speeds = self.controller.get_speed_info()

            # XY in µm
            if xy[0] is not None:
                zx = xy[0] - self.controller.zero_position["x"]
                zy = xy[1] - self.controller.zero_position["y"]
                ux = steps_to_um(zx, self._microsteps_per_micron)
                uy = steps_to_um(zy, self._microsteps_per_micron)
                self.sb_xy.setText(f"XY: {ux:,.1f} , {uy:,.1f} µm")
            else:
                self.sb_xy.setText("XY: — , — µm")

            # Z in mm, Pumps in µL when syringe configured else mm
            if zp[0] is not None:
                zz = self.controller.zero_position.get("Z", 0)
                self.sb_z.setText(f"Z: {zp[0] - zz:.2f}")

                for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
                    pos_mm = zp[idx] if idx < len(zp) else None
                    lbl = getattr(self, f"sb_p{idx}", None)
                    if lbl is None:
                        continue

                    zero_ref = self.controller.zero_position.get(pid, 0)
                    if pos_mm is not None and self._hardware_config:
                        pump_cfg = self._hardware_config.pumps.get(pid)
                        if pump_cfg and pump_cfg.is_configured:
                            try:
                                pos_uL = pump_cfg.mm_to_uL(pos_mm - zero_ref)
                                lbl.setText(f"{pid}: {pos_uL:.2f} µL")
                                continue
                            except ValueError:
                                pass
                    if pos_mm is not None:
                        lbl.setText(f"{pid}: {pos_mm - zero_ref:.2f} mm")
                    else:
                        lbl.setText(f"{pid}: —")
            else:
                self.sb_z.setText("Z: —")
                for idx in [1, 2, 3]:
                    lbl = getattr(self, f"sb_p{idx}", None)
                    if lbl:
                        lbl.setText(f"P{idx}: —")

            self.sb_speed.setText(
                f"Speed XY:{speeds['xy']:,.0f} Z:{speeds['z']:.1f} "
                f"P:{speeds['p']:.1f}"
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
            self.sb_log_count.setText(
                f"📝 {self.controller.position_logger.count}")

        # Propagate to active page
        page = self._page_widgets[self._current_page_index]
        if hasattr(page, 'on_status_update'):
            page.on_status_update()

    def _update_conn_dot(self, name: str, state: str):
        """Update a connection status dot.

        Args:
            name:  Device key ("xy", "zp", "xbox").
            state: "on" (green), "warn" (yellow), or "off" (red).
        """
        dot = getattr(self, f"_dot_{name}", None)
        lbl = getattr(self, f"_lbl_{name}", None)
        if dot is None:
            return
        _dot_names = {"on": "connDotOn", "warn": "connDotWarn", "off": "connDotOff"}
        _lbl_names = {"on": "connLabelOn", "warn": "connLabelWarn", "off": "connLabelOff"}
        dot.setObjectName(_dot_names.get(state, "connDotOff"))
        if lbl:
            lbl.setObjectName(_lbl_names.get(state, "connLabelOff"))
        dot.style().unpolish(dot)
        dot.style().polish(dot)
        dot.update()
        if lbl:
            lbl.style().unpolish(lbl)
            lbl.style().polish(lbl)
            lbl.update()


    def keyPressEvent(self, event: QKeyEvent):
        """Global keyboard handling — Escape triggers E-stop."""
        if event.key() == Qt.Key.Key_Escape:
            if self.controller.zp_stage:
                try:
                    self.controller.zp_stage.emergency_stop()
                    logger.warning("EMERGENCY STOP via Escape key")
                except Exception as e:
                    logger.error(f"E-stop failed: {e}")
        else:
            page = self._page_widgets[self._current_page_index]
            if hasattr(page, 'keyPressEvent'):
                page.keyPressEvent(event)
            else:
                super().keyPressEvent(event)

    # ════════════════════════════════════════════════════════════════
    #  SETTINGS SAVE / RESTORE
    # ════════════════════════════════════════════════════════════════

    def save_settings(self):
        """Save all settings including window geometry and hardware config."""
        self.settings.set("window.x", self.x())
        self.settings.set("window.y", self.y())
        self.settings.set("window.width", self.width())
        self.settings.set("window.height", self.height())
        self.settings.set("window.active_tab", self._current_page_index)
        if self._hardware_config:
            self._save_hardware_config(self._hardware_config)
        self.settings.save()

    # ════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        """Clean shutdown — stop timers, save settings, stop recording."""
        self.update_timer.stop()
        self.save_settings()
        if self.recorder and self.recorder.is_recording:
            self.recorder.stop_recording()

        # Shut down any background threads owned by pages
        for page in self._page_widgets:
            if hasattr(page, '_shutdown_detection_worker'):
                page._shutdown_detection_worker()

        self.controller.shutdown()
        event.accept()
