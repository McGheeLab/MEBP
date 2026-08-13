"""
app.py — MEBP Main Window with PyDracula-style sidebar navigation.

v7.3.3 changes:
    - Mode-based navigation: Printing + Pick & Place are mode pages
      with right-side sub-page icon columns
    - Printing mode wraps: Print Setup, Monitor, Results, Helpers
    - Pick & Place mode: Target Selection, Operation Queue, Execution
    - v7.4.3 page indices: 0=Hardware, 1=Calibration, 2=Jog,
      3=Printing(mode), 4=PickPlace(mode), 5=Settings
      (Calibration moved above Jog — both share StandardJogContextPanel)

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
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QIcon, QPixmap, QPainter, QColor, QKeyEvent

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings
from SupportClasses.PrintHistory import PrintHistory
from SupportClasses.PrintManager import load_print_progress, clear_print_progress
from SupportClasses.PrintRecorder import PrintRecorder
from SupportClasses.HardwareConfig import HardwareConfig
from SupportClasses.CommonPrintSettings import CommonPrintSettings
from SupportClasses.ContextPanelLayoutStore import ContextPanelLayoutStore
from gui.styles import DARK_THEME, COLORS, build_theme, apply_scaled_styles
from gui.ui_functions import UIFunctions, AppSettings
from gui.scaling import s, scale_factor, scaled_font_size
from gui.unit_helpers import (
    stage_to_um, um_to_stage, format_um,
    get_position_scale_from_protocol,
    DEFAULT_XY_POSITION_SCALE,
)
from gui.pages.hardware_setup import HardwareSetupPage
# v7.4.2: DashboardPage removed; its readouts merged into Jog + Hardware Setup
from gui.pages.jog_control import JogControlPage
from gui.pages.calibration import CalibrationPage
from gui.pages.print_builder import PrintBuilderPage       # v7.5.x
from gui.pages.workflows_mode import WorkflowsModePage    # v7.4.3
from gui.pages.settings_page import SettingsPage
from gui.widgets.console_log import ConsoleLogWidget
from gui.widgets.xbox_mapping_editor import XboxMappingEditor
from gui.widgets.camera_manager import CameraManager       # v7.3.3
from gui.widgets.components import LoadingBanner            # v7.4.0-a
from gui.widgets.page_transition import fade_swap           # v7.4.0-a
from gui.widgets.context_panel_host import ContextPanelHost  # v7.5.x
from gui.widgets.context_sections import (                  # v7.5.x
    SectionContext, known_types as _context_section_types,
)

logger = logging.getLogger(__name__)


# ── Emoji → colored pixmap icon helper ───────────────────────────

def _make_text_icon(text: str, size: int = 0, color: str = "#a6adc8") -> QIcon:
    """Create a QIcon from a text character (emoji or symbol)."""
    if size == 0:
        size = s(24)
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

    # v7.4.0-b: Emitted when hardware config changes in a way that affects
    # downstream derived data on other pages (plate format, pumps, inks,
    # needle). Pages can subscribe and either auto-refresh or surface an
    # InvalidationBanner.
    hw_config_invalidated = Signal(dict)

    # v7.4.0-c: Emitted when the global Help toggle flips. FormRow widgets
    # registered via register_form_row() listen and reveal/hide their
    # inline help text.
    help_mode_changed = Signal(bool)

    # v7.5.x ZP reconnect hotfix: emitted when StageController detects a stage
    # disconnect (watchdog OR poller-driven liveness). Wired so the backend
    # can push an immediate status refresh instead of waiting for the next
    # ~300ms poll tick. Emitted from a worker thread → queued to the GUI
    # thread, so the slot is the only place that touches widgets.
    stage_disconnected = Signal(str)

    # v7.5.x: emitted when StageController reports a successful (re)connect.
    # The ZP edge drives the last-known-position restore prompt. on_connect
    # may fire on a worker thread (onboarding) → bridged to the GUI thread.
    stage_connected = Signal(str)

    def __init__(self, controller: StageController, settings: Settings,
                 print_history: PrintHistory | None = None,
                 recorder: PrintRecorder | None = None):
        super().__init__()

        self._propagating_config = False
        self.controller = controller
        self.settings = settings
        self.print_history = print_history
        self.recorder = recorder

        # v7.4.0-c: Help-mode state + FormRow registry
        self._help_mode: bool = False
        self._registered_form_rows: list = []

        # Menu button references
        self._menu_buttons: list[QPushButton] = []
        self._page_widgets: list[QWidget] = []
        self._current_page_index = 0

        # v7.2: Hardware configuration
        self._hardware_config: HardwareConfig | None = None

        # Tutorial walkthrough recorder — constructed lazily on first arm, so
        # a normal session never installs the event filter.
        self._action_recorder = None

        # XY position scale factor (stage readout units per µm, 1.0 for ProScan)
        self._xy_position_scale: float = self._resolve_xy_position_scale()
        self._protocol_checked = False

        self.setWindowTitle("MEBP Bioprinter — v7.4.2")
        self.setMinimumSize(s(1100), s(700))
        self.resize(s(1400), s(850))

        # Apply DPI-scaled dark theme
        k = scale_factor()
        apply_scaled_styles(k)
        self.setStyleSheet(build_theme(k))

        # Build the UI shell
        self._build_ui()
        self._build_bottom_bar()
        self._create_pages()
        self._setup_timers()

        # v7.5.x ZP reconnect hotfix: let the backend push disconnects to the
        # GUI. on_disconnect fires on the watchdog/poller thread, so the
        # callback only emits a signal; _on_stage_disconnected (GUI thread)
        # does the actual refresh.
        self.stage_disconnected.connect(self._on_stage_disconnected)
        self.controller.on_disconnect = self._emit_stage_disconnected

        # v7.5.x: ZP last-known-position restore. On the first ZP connect
        # this session, offer to re-stamp the firmware counter with the
        # position saved at the previous clean shutdown.
        self._zp_restore_prompted = False
        # v7.5.x: once-per-session guard for the last-known calibration restore
        # prompt (needle zero + plate + Z) — see _maybe_prompt_calibration_restore.
        self._calibration_restore_prompted = False
        # v7.5.x: once-per-session guard for the calibration-usability pop-up
        # (XY/Z/P last-calibrated + recalibration warnings). See
        # _maybe_show_calibration_status.
        self._usability_prompted = False
        self.stage_connected.connect(self._on_stage_connected)
        self.controller.on_connect = self._emit_stage_connected

        # Start on Hardware Setup page
        self._navigate_to(0)

        # v7.4.0-c: Schedule onboarding wizard if first-run (defer until
        # after the window is shown so it appears on top).
        QTimer.singleShot(0, self._maybe_show_onboarding)

        # v7.17: construct the LabLink upload service iff the operator turned
        # it on — publish_* calls are no-ops while peek_service() is None, so
        # this single call is what arms them for the whole session.
        self._init_lablink()

        logger.info("MainWindow initialized (v7.4.2)")

    def _init_lablink(self) -> None:
        """Minimal v7.17 wiring: start the upload service when configured.

        No UI page yet — the service's own per-source config gating
        (`LabLinkConfigStore.source_enabled`) decides what actually uploads,
        and everything here degrades to a silent no-op: feature off, config
        unreadable, vendored client absent. Deliberately after the pages are
        built so `_lablink_busy` can see the print manager."""
        try:
            from SupportClasses import LabLinkConfigStore
            cfg = LabLinkConfigStore.get_store()
            if not (cfg.enabled() and cfg.is_configured()):
                return
            from SupportClasses.LabLinkService import get_service
            get_service(busy_check=self._lablink_busy).start()
            logger.info("LabLink: upload service started")
        except Exception:
            logger.exception("LabLink: service startup failed (feature off)")

    def _lablink_busy(self) -> bool:
        """True while a print runs — uploads are held back, not dropped.

        Hashing hundreds of MB and writing a socket alongside a print can
        starve the serial reader enough to trip a disconnect watchdog (the
        failure class three update plans document), so the LabLink worker
        polls this before each job. PAUSED counts as busy: the machine is still
        mid-print and must not be disturbed. Called from the LabLink worker
        thread — reads only, defensively."""
        try:
            page = getattr(self, "_full_print_page", None)
            pm = getattr(getattr(page, "setup_page", None),
                         "print_manager", None)
            if pm is None:
                return False
            from SupportClasses.PrintManager import PrintState
            return pm.state in (PrintState.RUNNING, PrintState.PAUSED)
        except Exception:
            return False                # never let the probe wedge an upload

    # ════════════════════════════════════════════════════════════════
    #  v7.4.0-c: ONBOARDING WIZARD TRIGGER
    # ════════════════════════════════════════════════════════════════

    def _maybe_show_onboarding(self):
        """Show OnboardingWizard if first-run detected.

        Trigger: no needle gauge saved AND no last hardware config file
        path saved. Skips silently otherwise.
        """
        try:
            from gui.onboarding.wizard import (
                OnboardingWizard, should_show_onboarding,
            )
        except Exception as e:
            logger.warning(f"Onboarding import failed: {e}")
            return

        if not should_show_onboarding(self.settings):
            return

        logger.info("First-run detected — launching OnboardingWizard")
        wizard = OnboardingWizard(self.controller, self.settings, parent=self)
        wizard.completed.connect(self._on_onboarding_completed)
        wizard.exec()

    def _on_onboarding_completed(self, config):
        """Apply the config produced by the onboarding wizard."""
        try:
            hw_page = self._page_widgets[0]
            hw_page.set_config(config)
            # If user opted into the deep-link, switch to HW Setup → Pump
            target = getattr(self.sender(), 'get_deep_link_target', lambda: None)()
            if target is not None:
                self._navigate_to(target)
                # Land on the Pump tab, the first of the material-config tabs.
                # v7.12: resolved BY NAME. The hardcoded index 3 here was stale —
                # it dated from before the Rosette/Ink tabs were inserted, so
                # onboarding actually dropped the operator on Rosette while the
                # comment claimed Pump.
                if hasattr(hw_page, 'sub_page_index'):
                    # v7.17.x: a rig with NO device profile has no identity
                    # yet — naming it on Device is what gives it its own
                    # config/hardware/<name>/ calibration folder, so that has
                    # to come before any material config. Otherwise land on
                    # Pump as before.
                    landing = "Pump"
                    try:
                        from gui.pages.hardware.device_profile import list_profiles
                        if not list_profiles():
                            landing = "Device"
                    except Exception:
                        pass
                    idx = hw_page.sub_page_index(landing)
                    if idx < 0:
                        idx = hw_page.sub_page_index("Pump")
                    if idx >= 0:
                        hw_page.switch_to(idx)
            logger.info("Onboarding config applied")
        except Exception as e:
            logger.error(f"Failed to apply onboarding config: {e}")

    # ════════════════════════════════════════════════════════════════
    #  XY POSITION SCALE PROPERTY
    # ════════════════════════════════════════════════════════════════

    @property
    def xy_position_scale(self) -> float:
        return self._xy_position_scale

    @xy_position_scale.setter
    def xy_position_scale(self, value: float):
        self._xy_position_scale = max(0.001, value)
        self.settings.set("stage.xy_position_scale", self._xy_position_scale)
        for page in self._page_widgets:
            if hasattr(page, 'set_xy_position_scale'):
                page.set_xy_position_scale(self._xy_position_scale)
        logger.info(f"xy_position_scale set to {self._xy_position_scale}")

    def _resolve_xy_position_scale(self) -> float:
        proto_val = self._try_load_from_protocol()
        if proto_val is not None:
            self.settings.set("stage.xy_position_scale", proto_val)
            return proto_val
        saved = self.settings.get("stage.xy_position_scale")
        if saved is not None:
            try:
                val = float(saved)
                if val > 0:
                    return val
            except (TypeError, ValueError):
                pass
        return DEFAULT_XY_POSITION_SCALE

    def _try_load_from_protocol(self) -> float | None:
        xy = getattr(self.controller, 'xy_stage', None)
        if xy is None:
            return None
        protocol = getattr(xy, '_protocol', None)
        return get_position_scale_from_protocol(protocol)

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
        self.ui_leftMenuBg.setMinimumWidth(s(60))
        self.ui_leftMenuBg.setMaximumWidth(s(60))
        self.ui_leftMenuBg.setFrameShape(QFrame.NoFrame)

        left_layout = QVBoxLayout(self.ui_leftMenuBg)
        left_layout.setSpacing(0)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # Logo
        logo_frame = QFrame()
        logo_frame.setObjectName("topLogo")
        logo_frame.setMinimumHeight(s(50))
        logo_frame.setMaximumHeight(s(50))
        logo_layout = QHBoxLayout(logo_frame)
        logo_layout.setContentsMargins(8, 0, 8, 0)
        logo_label = QLabel("🧬")
        logo_label.setFont(QFont("Segoe UI Emoji", scaled_font_size(18)))
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
        self._toggle_btn.setMinimumHeight(s(36))
        self._toggle_btn.setCursor(Qt.PointingHandCursor)
        self._toggle_btn.setToolTip("Expand menu")
        self._toggle_btn.clicked.connect(lambda: UIFunctions.toggleMenu(self))
        left_layout.addWidget(self._toggle_btn)

        # Separator
        sep = QFrame()
        sep.setObjectName("leftMenuFrame")
        sep.setFrameShape(QFrame.HLine)
        sep.setMaximumHeight(s(2))
        left_layout.addWidget(sep)

        # Top menu (workflow buttons)
        self.ui_topMenu = QFrame()
        self.ui_topMenu.setObjectName("topMenu")
        self.ui_topMenu.setFrameShape(QFrame.NoFrame)
        top_menu_layout = QVBoxLayout(self.ui_topMenu)
        top_menu_layout.setSpacing(0)
        top_menu_layout.setContentsMargins(0, 4, 0, 4)

        # Page buttons — Hardware Setup is first
        # v7.3.3: Printing + Pick & Place are mode pages (sub-pages inside)
        # v7.4.2: Dashboard removed; its readouts moved into Jog and
        # Hardware Setup → Device.
        # v7.4.3: Calibration moved above Jog Control. The calibration
        # step naturally precedes everyday driving, and both pages now
        # share the same StandardJogContextPanel on the left, so the
        # earlier-in-list ordering reads as the natural workflow.
        menu_items = [
            ("btn_hardware",     "🔧", "Hardware Setup"),
            ("btn_calibrate",    "📐", "Calibration"),
            ("btn_jog",          "🕹️", "Jog Control"),
            ("btn_printbuilder", "✏️", "Print Builder"),     # mode page (v7.5.x)
            ("btn_workflows",    "🧫", "Workflows"),            # mode page (v7.4.3)
            # v7.5.x: the former "Printing" mode is now the "Full Print" tile
            # inside Workflows (gui/pages/workflows/full_print_workflow.py).
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
        self.ui_extraLeftBox.setFrameShape(QFrame.NoFrame)

        extra_layout = QVBoxLayout(self.ui_extraLeftBox)
        extra_layout.setSpacing(0)
        extra_layout.setContentsMargins(0, 0, 0, 0)

        extra_top = QFrame()
        extra_top.setObjectName("extraTopBg")
        extra_top.setMinimumHeight(s(40))
        extra_top.setMaximumHeight(s(40))
        extra_top_layout = QHBoxLayout(extra_top)
        extra_top_layout.setContentsMargins(s(10), 0, s(6), 0)

        self._context_title = QLabel("Settings")
        self._context_title.setObjectName("extraLabel")
        extra_top_layout.addWidget(self._context_title)
        extra_top_layout.addStretch()

        from gui.widgets.icons import icon_button as _icon_button
        btn_close_context = _icon_button("", "x", tooltip="Hide context panel")
        btn_close_context.setObjectName("extraCloseColumnBtn")
        btn_close_context.setFixedSize(s(28), s(28))
        btn_close_context.clicked.connect(self._toggle_left_context)
        extra_top_layout.addWidget(btn_close_context)

        extra_layout.addWidget(extra_top)

        # v7.5.x: the left context is a SINGLE global ContextPanelHost (pill
        # picker: Jog | Custom) mounted here, replacing the old per-page
        # QStackedWidget. The host itself is created in _create_pages (once the
        # controller / camera manager / layout store exist) and dropped into
        # this holder. One managed widget + explicit visibility (see
        # _refresh_left_context) eliminates the old show/hide-vs-content desync
        # that made the jog panel "sometimes disappear" in workflows.
        self._context_content = QWidget()
        self._context_content.setObjectName("extraContent")
        _context_content_lay = QVBoxLayout(self._context_content)
        _context_content_lay.setContentsMargins(0, 0, 0, 0)
        _context_content_lay.setSpacing(0)
        self._context_content_layout = _context_content_lay
        self._context_host = None  # created in _create_pages
        extra_layout.addWidget(self._context_content)

        # ── Right context panel (v7.4.2) ─────────────────────────
        # A mirror of the left context panel anchored on the right
        # edge. Pages opt in by implementing ``get_right_context_widget``.
        # If a page returns None, the right panel hides.
        self.ui_extraRightBox = QFrame()
        self.ui_extraRightBox.setObjectName("extraRightBox")
        self.ui_extraRightBox.setFrameShape(QFrame.NoFrame)
        self.ui_extraRightBox.setStyleSheet(
            f"#extraRightBox {{ background-color: {COLORS['mantle']}; "
            f"border-left: 1px solid {COLORS['surface1']}; }}"
        )
        right_layout = QVBoxLayout(self.ui_extraRightBox)
        right_layout.setSpacing(0)
        right_layout.setContentsMargins(0, 0, 0, 0)

        right_top = QFrame()
        right_top.setMinimumHeight(s(40))
        right_top.setMaximumHeight(s(40))
        right_top_layout = QHBoxLayout(right_top)
        right_top_layout.setContentsMargins(s(10), 0, s(6), 0)
        self._right_context_title = QLabel("")
        self._right_context_title.setObjectName("extraLabel")
        right_top_layout.addWidget(self._right_context_title)
        right_top_layout.addStretch()
        right_layout.addWidget(right_top)

        self._right_context_stack = QStackedWidget()
        right_layout.addWidget(self._right_context_stack)
        self.ui_extraRightBox.hide()  # hidden until a page opts in

        # ── Content Area ─────────────────────────────────────────
        content_frame = QFrame()
        content_frame.setObjectName("contentBox")
        content_layout = QVBoxLayout(content_frame)
        content_layout.setSpacing(0)
        content_layout.setContentsMargins(0, 0, 0, 0)

        # Top bar
        top_bar = QFrame()
        top_bar.setObjectName("contentTopBg")
        top_bar.setMinimumHeight(s(40))
        top_bar.setMaximumHeight(s(40))
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(s(12), 0, s(12), 0)

        # Context panel toggle — left side, next to where panel opens
        btn_context = QPushButton("☰")
        btn_context.setObjectName("extraBtn")
        btn_context.setFixedSize(s(32), s(32))
        btn_context.setCursor(Qt.PointingHandCursor)
        btn_context.setToolTip("Toggle context panel")
        btn_context.clicked.connect(self._toggle_left_context)
        top_bar_layout.addWidget(btn_context)

        title_frame = QWidget()
        title_layout = QVBoxLayout(title_frame)
        title_layout.setSpacing(0)
        title_layout.setContentsMargins(0, 4, 0, 4)

        self._page_title = QLabel("Hardware Setup")
        self._page_title.setObjectName("pageTitle")
        self._page_title.setFont(QFont("Segoe UI", scaled_font_size(12), QFont.Bold))
        title_layout.addWidget(self._page_title)

        top_bar_layout.addWidget(title_frame)
        top_bar_layout.addStretch()

        # v7.4.0-c: Help toggle — reveals inline FormRow help text
        from gui.widgets.help_toggle import HelpToggle
        self._help_toggle = HelpToggle()
        self._help_toggle.toggled.connect(self._on_help_toggled)
        top_bar_layout.addWidget(self._help_toggle)

        # Tutorial action recorder — captures what you DO, as material for
        # a walkthrough video. Inert until armed; see docs/videos/DIRECTING.md.
        self._rec_btn = QPushButton("● REC")
        self._rec_btn.setObjectName("recToggle")
        self._rec_btn.setCheckable(True)
        self._rec_btn.setCursor(Qt.PointingHandCursor)
        self._rec_btn.setToolTip(
            "Record a tutorial walkthrough.\n"
            "While recording, press F9 to mark a step that matters.")
        self._rec_btn.setStyleSheet(self._rec_button_qss(False))
        self._rec_btn.clicked.connect(self._on_toggle_recording)
        top_bar_layout.addWidget(self._rec_btn)

        # Connection dots
        conn_frame = QFrame()
        conn_frame.setObjectName("connStatusFrame")
        conn_layout = QHBoxLayout(conn_frame)
        conn_layout.setSpacing(s(12))
        conn_layout.setContentsMargins(0, 0, 0, 0)
        conn_layout.addWidget(self._make_conn_dot("XY"))
        conn_layout.addWidget(self._make_conn_dot("ZP"))
        conn_layout.addWidget(self._make_conn_dot("Xbox"))
        top_bar_layout.addWidget(conn_frame)

        content_layout.addWidget(top_bar)

        # v7.4.0-a: Global loading banner — show_loading(msg) / hide_loading()
        self._loading_banner = LoadingBanner()
        content_layout.addWidget(self._loading_banner)

        # v7.4.0-c: Global invalidation banner shown when hardware config
        # changes invalidate derived data on other pages. Click Refresh →
        # re-propagate config so pages re-read their inputs.
        from gui.widgets.invalidation_banner import InvalidationBanner
        self._invalidation_banner = InvalidationBanner(
            message="Hardware changed — refresh pages to re-apply.",
            on_refresh=self._on_invalidation_refresh,
        )
        self._invalidation_banner.hide()
        content_layout.addWidget(self._invalidation_banner)

        # Content splitter (pages + console)
        self._splitter = QSplitter(Qt.Vertical)
        self._splitter.setObjectName("contentBottom")

        # Page stack
        self._page_stack = QStackedWidget()
        self._page_stack.setObjectName("pagesContainer")
        self._page_stack.setMinimumHeight(s(200))  # v7.2.6
        self._splitter.addWidget(self._page_stack)

        # Console log
        # v7.4.2: bump minimum height so the terminal pane is comfortably
        # tall by default (the previous 40px floor let it collapse to a
        # one-line strip). Splitter stretch ratio drops from 5:1 to 3:1
        # so it gets ~25% of the vertical space initially.
        # v7.4.2: start with the terminal collapsed — most users only need
        # it when something goes wrong, and the default print/jog/cal
        # workflows benefit from the extra vertical real estate.
        self.console = ConsoleLogWidget(start_collapsed=True)
        self._console_min_expanded = s(180)
        # Min height stays at 0 while collapsed so the splitter can shrink
        # the pane to its toolbar; the collapse handler restores the
        # comfortable floor on expand.
        self.console.setMinimumHeight(0)
        self._splitter.addWidget(self.console)

        self._splitter.setStretchFactor(0, 3)
        self._splitter.setStretchFactor(1, 1)

        # v7.2.6: Prevent splitter collapse crash
        self._splitter.setChildrenCollapsible(False)

        # v7.4.2: when the user collapses the terminal, drop the splitter
        # min-height floor so the pane can actually shrink to the toolbar;
        # restore the comfortable default when re-expanded.
        self.console.collapse_toggled.connect(self._on_console_collapsed)

        content_layout.addWidget(self._splitter)

        # v7.3.2: Horizontal splitter for resizable context panel + content
        # v7.4.2: third pane on the right for pages that need it (Calibration).
        self._context_splitter = QSplitter(Qt.Horizontal)
        self._context_splitter.setObjectName("contextSplitter")
        self._context_splitter.addWidget(self.ui_extraLeftBox)
        self._context_splitter.addWidget(content_frame)
        self._context_splitter.addWidget(self.ui_extraRightBox)
        self._context_splitter.setStretchFactor(0, 0)  # left context: fixed
        self._context_splitter.setStretchFactor(1, 1)   # content: stretches
        self._context_splitter.setStretchFactor(2, 0)  # right context: fixed
        # v7.5.x: the left context panel is a *bounded sidebar*, not a free
        # splitter pane. The knobs that make dragging behave:
        #
        #   • slot 0 non-collapsible + a modest minimum width → it can't be
        #     dragged shut. The minimum is small (s(260)) because the panel
        #     content is now **responsive** (see StandardJogContextPanel /
        #     JogButtonArray): its buttons + text scale to fit and wrap, so the
        #     panel is genuinely usable when narrow instead of clipping buttons.
        #     It still OPENS at the comfortable LEFT_BOX_WIDTH (see
        #     _apply_saved_context_width) — the small min just lets the user drag
        #     it narrow.
        #   • slot 1 (content) is also non-collapsible → it yields width by
        #     shrinking (its own children scroll/clip) instead of snapping shut,
        #     which is what produced the "snaps to mid / full open" jump. With
        #     both panes non-collapsible, Qt simply *stops* the drag handle when
        #     the content pane reaches its own minimum — no dynamic-maximum race,
        #     no snap.
        #   • a light *dynamic maximum width* on slot 0 (see _update_context_
        #     panel_bounds, refreshed on every resize) keeps a small content
        #     reserve so the panel can't swallow the whole window.
        #
        # The right pane (slot 2) is shown/hidden explicitly per page and stays
        # collapsible.
        self._context_splitter.setChildrenCollapsible(True)
        self._context_splitter.setCollapsible(0, False)
        self._context_splitter.setCollapsible(1, False)
        # Small minimum — the panel content is fully proportional (every item
        # takes a %% of the width), so it fits itself to any width ≥ this. It
        # still OPENS at the comfortable LEFT_BOX_WIDTH.
        self.ui_extraLeftBox.setMinimumWidth(s(100))
        self._context_splitter.setHandleWidth(s(4))
        # Reserve at least this much for the content pane when sizing the left
        # panel. Smaller than the panel's own minimum: with content now
        # non-collapsible, Qt stops the drag at content's real minimum, so this
        # only needs to keep the panel from consuming the entire window.
        self._content_reserve_px = s(240)
        # Remember the dragged/restored panel width across hide/show. Track it
        # live as the user drags so a page-switch (which hides the panel)
        # preserves whatever they last set.
        self._context_splitter.splitterMoved.connect(
            self._on_context_splitter_moved)
        # Start with context panel hidden
        self.ui_extraLeftBox.hide()
        # Remember last width (scaled — LEFT_BOX_WIDTH is a base px value, so
        # the first open must use the runtime-scaled width, not the raw 540).
        self._context_panel_width = s(AppSettings.LEFT_BOX_WIDTH)

        app_layout.addWidget(self._context_splitter)

    def _make_menu_button(self, obj_name: str, icon_text: str,
                          label: str) -> QPushButton:
        """Create a sidebar navigation button."""
        btn = QPushButton(icon_text)
        btn.setObjectName(obj_name)
        btn.setMinimumHeight(s(44))
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
        dot.setFixedWidth(s(14))
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

        mono = QFont("Consolas", scaled_font_size(9))

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

        v7.5.x page indices (Printing mode re-homed into Workflows as the
        "Full Print" tile, so its sidebar slot is removed):
            0: Hardware Setup (always enabled)
            1: Calibration
            2: Jog Control
            3: Print Builder (mode — sub-pages: Sketch, Image Import,
               Hardware, Print Settings)
            4: Workflows (mode — incl. the Full Print tile = Setup/Monitor/Results)
            5: Settings (always enabled)

        v7.2.3: Also wires the job pipeline (Setup → Monitor)
        and execution control signals (Monitor → PrintManager).
        """
        # Restore hardware config from settings
        self._hardware_config = self._restore_hardware_config()

        # v7.5.x: shared "Common Print Settings" model. Globals (settle / relief
        # / prime) proxy the live HardwareConfig; the promoted prep defaults are
        # restored from settings.json. A listener persists + propagates changes.
        self._common_print_settings = CommonPrintSettings()
        self._common_print_settings.set_hardware_config(self._hardware_config)
        try:
            self._common_print_settings.load_promoted(
                self.settings.get("common_print_settings"))
        except Exception as e:
            logger.debug("restore common_print_settings failed: %s", e)
        self._common_print_settings.add_listener(self._on_common_setting_changed)

        # v7.3.3: Shared camera manager for all pages.
        # v7.5.x: slot count follows MAX_LIVE_CAMERAS (4 — incl. the
        # Monitor overview camera) via CameraManager's default.
        self._camera_manager = CameraManager()

        # v7.14 — the capture buttons live on ~18 live-feed widgets that are
        # constructed with only (camera_manager, cam_idx). Registering the
        # settings + stage controller here lets a capture stamp its stage
        # position and read its destination without threading both through
        # every one of those constructors.
        try:
            from SupportClasses.CaptureContext import register as _cap_register
            _cap_register(settings=self.settings, controller=self.controller,
                          camera_manager=self._camera_manager)
        except Exception as exc:
            logger.debug(f"capture context not registered: {exc}")

        # v7.3.3: Mode pages wrap sub-pages internally
        self._print_builder = PrintBuilderPage(self.controller, self.settings)  # v7.5.x
        self._workflows_mode = WorkflowsModePage(
            self.controller, self.settings,
            camera_manager=self._camera_manager)
        # v7.5.x: the former "Printing" mode (Setup/Monitor/Results) now lives
        # inside Workflows as the "Full Print" tile. Grab a direct handle so the
        # job pipeline / monitor / results wiring below can reach its
        # setup_page / monitor_page / results_page (same surface PrintingModePage
        # exposed). The wrapper hosts a PrintingModePage internally.
        self._full_print_page = self._workflows_mode.full_print_page

        # v7.4.3: Calibration sits between Hardware Setup and Jog Control,
        # matching the sidebar menu order above. The pages list index is
        # the menu-button index, so the order here must mirror menu_items.
        pages = [
            HardwareSetupPage(),                                          # 0
            CalibrationPage(self.controller, settings=self.settings,
                           camera_manager=self._camera_manager),          # 1
            JogControlPage(self.controller,
                          camera_manager=self._camera_manager),           # 2
            self._print_builder,                                          # 3  v7.5.x mode
            self._workflows_mode,                                         # 4  v7.4.3 mode
            SettingsPage(self.controller, self.settings),                 # 5
        ]

        # Wire Hardware Setup signals
        hw_page = pages[0]
        hw_page.set_camera_manager(self._camera_manager)  # v7.3.3
        hw_page.set_controller(self.controller)  # v7.3.3: for pixel calibration
        hw_page.set_settings(self.settings)       # v7.4.0-b: for Stage sub-page
        hw_page.config_changed.connect(self._on_hardware_config_changed)
        hw_page.config_validated.connect(self._on_hardware_validated)

        # v7.4.2: Push saved axis_map + steps_per_mm + per_axis_max_feedrate
        # into the controller so they're ready when the ZP stage connects
        # (or pushed live now if it's already connected).
        try:
            self.controller.apply_device_settings(
                axis_map=self.settings.get("device_profile.axis_map") or None,
                steps_per_mm=self.settings.get("device_profile.steps_per_mm") or None,
                per_axis_max_feedrate=self.settings.get(
                    "device_profile.per_axis_max_feedrate") or None,
                persist_steps=False,    # Don't re-send M92 on startup
                persist_feedrate=False,  # _setup_printer handles initial M203
            )
        except Exception as e:
            logger.warning(f"v7.4.2 apply_device_settings failed: {e}")

        # Restore saved config to Hardware Setup page
        if self._hardware_config:
            hw_page.set_config(self._hardware_config)

        # Wire recorder to the Full Print sub-pages (Setup/Monitor/Results,
        # re-homed into the Workflows "Full Print" tile).
        if self.recorder and self._full_print_page is not None:
            monitor = self._full_print_page.monitor_page
            if hasattr(monitor, 'set_recorder'):
                monitor.set_recorder(self.recorder)

            results_page = self._full_print_page.results_page
            if hasattr(results_page, 'set_recorder'):
                results_page.set_recorder(self.recorder)

            setup = self._full_print_page.setup_page
            if hasattr(setup, 'print_manager') and setup.print_manager:
                setup.print_manager.recorder = self.recorder

        # Register all pages with the stacked widgets
        from gui.pages.mode_page import ModePage
        for page in pages:
            self._page_widgets.append(page)
            self._page_stack.addWidget(page)

            # v7.5.x: the LEFT context is no longer a per-page stack — a single
            # global ContextPanelHost (created below) hosts each page's context
            # widget in its "Jog" view slot on demand. See _refresh_left_context.

            # v7.4.2: parallel right-context panel. Pages opt in by
            # implementing get_right_context_widget; everyone else
            # gets a placeholder so stack indices line up with page
            # indices.
            right_ctx = None
            if hasattr(page, 'get_right_context_widget'):
                right_ctx = page.get_right_context_widget()
            if right_ctx is not None:
                rscroll = QScrollArea()
                rscroll.setObjectName("rightContextScrollArea")
                rscroll.setWidgetResizable(True)
                rscroll.setWidget(right_ctx)
                self._right_context_stack.addWidget(rscroll)
            else:
                self._right_context_stack.addWidget(QWidget())

            # Propagate XY position scale
            if hasattr(page, 'set_xy_position_scale'):
                page.set_xy_position_scale(self._xy_position_scale)

        # v7.3.3: Wire mode page sub-page changes → context panel updates
        # v7.4.x: also wire WorkflowsModePage (not a ModePage subclass) so
        # the left context panel swaps to the active workflow's
        # StandardJogContextPanel when the user picks a workflow.
        for i, page in enumerate(self._page_widgets):
            if isinstance(page, ModePage) or isinstance(page, WorkflowsModePage):
                page.sub_page_changed.connect(
                    lambda idx, page_idx=i: self._on_mode_sub_page_changed(page_idx))

        # v7.5.x: build the single global left-context host now that the
        # controller, camera manager and settings all exist. The Custom view's
        # layout is a shared, persisted machine-level thing (ContextPanelLayoutStore).
        self._context_layout_store = ContextPanelLayoutStore(
            known_types=_context_section_types())
        self._context_section_ctx = SectionContext(
            controller=self.controller,
            hardware_config=self._hardware_config,
            camera_manager=self._camera_manager,
            settings=self.settings,
            layout_store=self._context_layout_store,
        )
        self._context_host = ContextPanelHost(self._context_section_ctx)
        self._context_host.view_changed.connect(self._on_context_view_changed)
        self._context_content_layout.addWidget(self._context_host)
        # Restore persisted collapse + active-view state.
        self._left_context_user_collapsed = bool(
            self.settings.get("context_panel.collapsed", False))
        try:
            self._context_host.set_active_view(
                self.settings.get("context_panel.active_view", "jog"))
        except Exception as e:
            logger.debug("restore context_panel.active_view failed: %s", e)

        # Propagate existing hardware config to pages
        if self._hardware_config:
            self._propagate_hardware_config(self._hardware_config)

        # v7.5.x: fan the Common Print Settings model out to the workflow pages
        # (Common Print Settings page + each workflow's inheriting fields).
        self._fanout_common_print_settings()

        # Initial page gating
        is_valid = (self._hardware_config is not None
                    and self._hardware_config.is_valid)
        self._update_page_gating(is_valid)

        # ── v7.2.3: Wire job pipeline and execution controls ─────
        self._wire_job_pipeline()
        self._wire_print_manager_to_monitor()

        # v7.2.7 / v7.5.x: Wire the Print Builder's authoring sub-pages
        # (Image Import + Sketch) — both bake a csv_import object that
        # should land in Print Setup's custom-prints area.
        for _author_page in (self._print_builder.image_import_page,
                             self._print_builder.sketch_page,
                             self._print_builder.library_page):
            if hasattr(_author_page, 'print_file_created'):
                _author_page.print_file_created.connect(
                    self._on_print_created)

        # v7.5.x: the Print Library (Prints tab) can rename/delete/duplicate
        # print files on disk — refresh the consumers' saved-print lists.
        _lib = self._print_builder.library_page
        if hasattr(_lib, 'print_files_changed'):
            _lib.print_files_changed.connect(self._on_print_files_changed)
        # Saving edits to an existing print (Sketch "Save changes") also
        # changes files on disk without creating a new one.
        if hasattr(self._print_builder.sketch_page, 'print_file_saved'):
            self._print_builder.sketch_page.print_file_saved.connect(
                lambda _n: self._on_print_files_changed())

        # v7.4.3: page order is now (HW=0, Calibration=1, Jog=2,
        # Printing=3, P&P=4, Settings=5).
        cal_page = pages[1]   # CalibrationPage
        jog_page = pages[2]   # JogControlPage

        # v7.3.2: Load approximate well plate first (geometry-predicted baseline)
        if hasattr(jog_page, 'load_startup_plate'):
            jog_page.load_startup_plate(self.settings)

        # v7.3.1: Wire calibration data → jog page (well positions, safe_z)
        # v7.3.4: Also push immediately — _load_calibration fires before this signal is wired
        # v7.4.4: Also push the full Z-reference set so the XZ side
        # view can render clickable "go to Z" badges for each captured
        # height (Replace / Max / Safe / Plate ↑ / Plate ↓).
        def _push_cal_to_jog():
            try:
                jog_page.set_calibration_data(*cal_page.get_calibration_data())
            except Exception as e:
                logger.debug(f"set_calibration_data on jog page failed: {e}")
            if (hasattr(jog_page, 'set_z_references')
                    and hasattr(cal_page, 'get_z_references')):
                try:
                    jog_page.set_z_references(cal_page.get_z_references())
                except Exception as e:
                    logger.debug(f"set_z_references on jog page failed: {e}")
            # v7.9.1: and WHICH of them get a quick-move badge. Pushed as its
            # own call rather than folded into the reference dict — that dict
            # also feeds the print-floor datum, so a hidden badge must not read
            # as a missing height.
            _vis = None
            if hasattr(cal_page, 'visible_z_reference_keys'):
                try:
                    _vis = cal_page.visible_z_reference_keys()
                except Exception as e:
                    logger.debug(f"visible_z_reference_keys failed: {e}")
            if _vis is not None and hasattr(jog_page,
                                            'set_visible_z_references'):
                try:
                    jog_page.set_visible_z_references(_vis)
                except Exception as e:
                    logger.debug(f"set_visible_z_references on jog failed: {e}")
            # v7.4.x: also push to the Workflows mode (active workflow
            # pages need the same plate / safe_z / Z-references so their
            # embedded XY workspace + XZ side view + StandardJogContextPanel
            # render identically to the Jog page).
            wf = getattr(self, "_workflows_mode", None)
            if wf is not None:
                try:
                    if hasattr(wf, "set_calibration_data"):
                        wf.set_calibration_data(*cal_page.get_calibration_data())
                    if (hasattr(wf, "set_z_references")
                            and hasattr(cal_page, "get_z_references")):
                        wf.set_z_references(cal_page.get_z_references())
                    if _vis is not None and hasattr(
                            wf, "set_visible_z_references"):
                        wf.set_visible_z_references(_vis)
                    if hasattr(wf, "set_settings"):
                        wf.set_settings(self.settings)
                except Exception as e:
                    logger.debug(f"push cal data to workflows mode failed: {e}")
            # v7.5.x: the Full Print setup page (re-homed into Workflows) gets
            # the calibrated well positions through the Workflows-mode fanout
            # above (wf.set_calibration_data → FullPrintWorkflowPage →
            # setup_page), so its print path drives to the TAUGHT wells instead
            # of a geometric grid anchored at the stage origin. No separate push
            # is needed now that "Printing" no longer has its own sidebar slot.
            # v7.5.x: push the Z-reference set to the Print Builder so its
            # Sketch sub-page can express print Z as a height above the
            # plate bottom.
            pb_builder = getattr(self, "_print_builder", None)
            if pb_builder is not None and hasattr(pb_builder, "set_z_references"):
                try:
                    if hasattr(cal_page, "get_z_references"):
                        pb_builder.set_z_references(cal_page.get_z_references())
                except Exception as e:
                    logger.debug(f"set_z_references on print builder failed: {e}")
            # v7.4.8: push the plate-wide insert-clearance floor to the
            # controller so every safe_travel_to clears the tallest tube.
            self._update_insert_clearance(cal_page)
            # v7.5.x: push the plate-bottom Z datum to the controller so every
            # print clamps against "don't punch through the plate bottom".
            self._update_print_floor_datum(cal_page)
        if hasattr(cal_page, 'calibration_data_changed') and hasattr(jog_page, 'set_calibration_data'):
            cal_page.calibration_data_changed.connect(_push_cal_to_jog)
            _push_cal_to_jog()

        # v7.5.x: when the user saves new XY safety limits, re-centre the
        # default (uncalibrated) plate on the new envelope. The Calibration
        # page is the single source of truth: recenter_default_plate() is
        # gated (no-op once calibrated) and, when it does re-seed, emits
        # calibration_data_changed → _push_cal_to_jog, so the Jog page /
        # Workflows mode update through the normal push path. We deliberately
        # do NOT poke the Jog page directly — that would be an ungated write
        # that could clobber calibrated positions on its workspace.
        hw_page = pages[0]  # HardwareSetupPage
        if hasattr(hw_page, 'safety_limits_changed') and hasattr(cal_page, 'recenter_default_plate'):
            def _recenter_plate_in_bounds():
                try:
                    cal_page.recenter_default_plate()
                except Exception as e:
                    logger.debug(f"recenter_default_plate failed: {e}")
            hw_page.safety_limits_changed.connect(_recenter_plate_in_bounds)

        # v7.5.x: ONE common per-axis MAX speed — when it changes anywhere
        # (Stage Panel edit, per-axis Z popup, or the timing tool's measured
        # top speed) fan a refresh out to every jog/speed surface so they
        # re-read the single source. Page edits use the Qt signal; the timing
        # worker's GUI-thread continuation calls controller.on_speed_limits_changed
        # → the same signal (so the fan-out always runs on the GUI thread).
        if hasattr(hw_page, 'safety_limits_changed'):
            hw_page.safety_limits_changed.connect(self._refresh_all_speed_limits)
            try:
                self.controller.on_speed_limits_changed = \
                    hw_page.safety_limits_changed.emit
            except Exception as e:
                logger.debug(f"wire on_speed_limits_changed failed: {e}")

        # v7.3.3: CameraManager is shared — no need to manually wire cameras

        # v7.3.3: Wire calibration → hardware page µm/px updates
        if hasattr(cal_page, 'um_per_px_calibrated'):
            cal_page.um_per_px_calibrated.connect(hw_page.set_calibrated_um_per_px)

        # v7.5.x: once the event loop is running (after show()), offer to
        # restore the last-known-good calibration (needle zero + plate + Z) if
        # the live calibration came up empty. Deferred so the prompt never
        # opens mid-construction. See _maybe_prompt_calibration_restore.
        QTimer.singleShot(450, self._maybe_prompt_calibration_restore)

        # v7.5.x: after the restore prompt, show the calibration-usability
        # pop-up (when XY/Z/P were last calibrated + recalibration warnings) —
        # but only if something needs attention. Deferred a little longer so it
        # never stacks on top of the restore prompt. See
        # _maybe_show_calibration_status.
        QTimer.singleShot(900, self._maybe_show_calibration_status)

    # ════════════════════════════════════════════════════════════════
    #  v7.2.3: JOB PIPELINE & EXECUTION CONTROL WIRING
    # ════════════════════════════════════════════════════════════════

    def _wire_job_pipeline(self):
        """
        Wire the print job flow from PrintSetupPage through to
        PrintMonitorPage, and connect execution control signals.

        v7.3.3: Access sub-pages through PrintingModePage.

        Signal flow:
            PrintSetup.job_ready(PrintJob)
                → app._send_job_to_monitor(job)
                → Monitor.receive_job(job)
                → auto-switch to Printing mode, Monitor sub-page

            PrintSetup.navigate_to_page(int)
                → app._switch_page(index)

            Monitor.start_requested(PrintJob)
                → app._on_monitor_start(job)
                → PrintManager.start(job)

            Monitor.pause_requested()  → PrintManager.pause()
            Monitor.resume_requested() → PrintManager.resume()
            Monitor.abort_requested()  → PrintManager.abort()
        """
        if self._full_print_page is None:
            logger.warning("_wire_job_pipeline: no full_print_page")
            return
        setup_page = self._full_print_page.setup_page
        monitor_page = self._full_print_page.monitor_page

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

        v7.5.x: Access sub-pages through the Full Print workflow page.
        """
        if self._full_print_page is None:
            logger.warning("_wire_print_manager_to_monitor: no full_print_page")
            return
        setup_page = self._full_print_page.setup_page
        monitor_page = self._full_print_page.monitor_page

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

        v7.5.x: Access via the Full Print workflow page + auto-navigate to
        Workflows → Full Print → Monitor sub-page.
        """
        if self._full_print_page is None:
            logger.warning("_send_job_to_monitor: no full_print_page")
            return
        monitor_page = self._full_print_page.monitor_page
        setup_page = self._full_print_page.setup_page

        # ── Initialize monitor visualization ──────────────────────
        try:
            self._setup_monitor_visualization(setup_page, monitor_page, job)
        except Exception as exc:
            logger.error(f"Monitor viz setup error: {exc}", exc_info=True)

        if hasattr(monitor_page, 'receive_job'):
            monitor_page.receive_job(job)

        # v7.5.x: navigate sidebar → Workflows, open the Full Print tile, then
        # switch its inner mode page to the Monitor sub-page. Order matters: the
        # Workflows page must be the visible stack page before we select the
        # tile + sub-page so the title/context refresh lands correctly. (This
        # also fixes the old stale `_switch_page(3)` which navigated to Print
        # Builder instead of Printing.)
        self._navigate_to(self._workflows_index())
        self._workflows_mode.open_workflow("full_print")
        self._full_print_page.switch_to_monitor()

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
                # v7.12: `well_diameter` is 0.0 on a parametric plate ("varies
                # — see WellInfo per well"), and setup_plate skips the XY-detail
                # and YZ well geometry entirely when it gets 0.
                well_diam = getattr(plate, 'representative_well_diameter',
                                    None)
                if well_diam is None:
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
        if self._full_print_page is None:
            logger.error("No full_print_page for monitor start")
            return
        setup_page = self._full_print_page.setup_page
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

                # v7.5.x: machine-readable execution log for this run
                from SupportClasses.PrintExecutionLogger import (
                    PrintExecutionLogger)
                try:
                    pm.exec_logger = PrintExecutionLogger(
                        job_name=job.name, mode="hybrid")
                    pm.exec_logger.start(
                        pm.controller,
                        PrintExecutionLogger.manifest_for_job(
                            job, pm.controller, "hybrid"))
                except Exception as _lex:
                    logger.warning(f"Execution log unavailable: {_lex}")
                    pm.exec_logger = None

                executor = HybridPlanExecutor(
                    controller=pm.controller,
                    plan=plan,
                    well_model=getattr(job, 'well_setup', None),
                    plate=getattr(job, 'plate', None),
                    path_points=getattr(job, 'path_points', []),
                    settings=job.settings,
                    hw_config=getattr(job, 'hw_config', None),
                    recorder=pm.recorder,
                    exec_logger=pm.exec_logger,
                )

                # v7.5.x CRITICAL: register the hybrid executor on the
                # PrintManager so Monitor → Abort (→ pm.abort()) reaches it.
                # pm.abort() does `if self._trajectory_executor:
                # self._trajectory_executor.abort()`; without this assignment
                # that flag never gets set, so abort marked the run ABORTED but
                # the executor thread kept printing the REST of the plan (every
                # remaining well / step) — "abort only stopped the current task,
                # not the whole print". HybridPlanExecutor polls its own
                # _abort_flag between steps and per well, so this cancels the
                # entire run at the next boundary.
                pm._trajectory_executor = executor

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
                        if pm.exec_logger:
                            pm.exec_logger.log_error(str(exc), exc)
                        pm._set_state(PrintState.ERROR)
                    finally:
                        # v7.5.x CRITICAL SAFETY: always leave the needle at the
                        # safe / travel Z (completion, error, or abort) — raise-
                        # only + idempotent, so a plan that already ended at safe
                        # Z is a no-op.
                        if hasattr(pm, '_retract_to_safe_z'):
                            pm._retract_to_safe_z(
                                "hybrid_end",
                                travel_z=getattr(job.settings,
                                                 'travel_z_height', None))
                        if hasattr(pm, '_stop_recorder'):
                            try:
                                pm._stop_recorder(pm.state.name.lower())
                            except Exception:
                                pass
                        if hasattr(pm, '_end_exec_log'):
                            try:
                                pm._end_exec_log()
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

            # v7.5.x: machine-readable execution log for this run
            from SupportClasses.PrintExecutionLogger import (
                PrintExecutionLogger)
            try:
                pm.exec_logger = PrintExecutionLogger(
                    job_name=job.name, mode=f"trajectory-{exec_mode}")
                pm.exec_logger.start(
                    pm.controller,
                    PrintExecutionLogger.manifest_for_job(
                        job, pm.controller, f"trajectory-{exec_mode}"))
            except Exception as _lex:
                logger.warning(f"Execution log unavailable: {_lex}")
                pm.exec_logger = None

            if exec_mode == "position":
                # Use the v7.1 TrajectoryExecutor — simplest, most reliable
                from SupportClasses.PrintManager import TrajectoryExecutor
                tex = TrajectoryExecutor(pm.controller, recorder=pm.recorder,
                                         exec_logger=pm.exec_logger)
                logger.info("Using TrajectoryExecutor (position mode)")
            elif tex is not None and pm.exec_logger is not None \
                    and hasattr(tex, 'exec_logger'):
                tex.exec_logger = pm.exec_logger

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
                    if pm.exec_logger:
                        pm.exec_logger.log_error(str(exc), exc)
                    pm._set_state(PrintState.ERROR)
                finally:
                    # v7.5.x CRITICAL SAFETY: always leave the needle at the
                    # safe / travel Z (completion, error, or abort) — raise-only
                    # + idempotent, so a trajectory that already ended retracted
                    # is a no-op. Covers the planner's known ZDIR=+1 Z-geometry
                    # gap by guaranteeing a polarity-safe final retract.
                    if hasattr(pm, '_retract_to_safe_z'):
                        pm._retract_to_safe_z(
                            "trajectory_end",
                            travel_z=getattr(job.settings,
                                             'travel_z_height', None))
                    if hasattr(pm, '_stop_recorder'):
                        try:
                            pm._stop_recorder(pm.state.name.lower())
                        except Exception:
                            pass
                    if hasattr(pm, '_end_exec_log'):
                        try:
                            pm._end_exec_log()
                        except Exception:
                            pass

            pm._thread = threading.Thread(target=_traj_thread, daemon=True)
            pm._thread.start()

        except Exception as exc:
            logger.error(f"Trajectory start failed: {exc}", exc_info=True)


    def _on_monitor_pause(self):
        """Monitor requested pause — forward to PrintManager."""
        if self._full_print_page is None:
            return
        setup_page = self._full_print_page.setup_page
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.pause()

    def _on_monitor_resume(self):
        """Monitor requested resume — forward to PrintManager."""
        if self._full_print_page is None:
            return
        setup_page = self._full_print_page.setup_page
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.resume()

    def _on_monitor_abort(self):
        """Monitor requested abort — forward to PrintManager."""
        if self._full_print_page is None:
            return
        setup_page = self._full_print_page.setup_page
        if hasattr(setup_page, 'print_manager'):
            setup_page.print_manager.abort()

    def _on_print_completed_v726(self):
        """v7.2.6: On print completion, notify results page.

        v7.5.x: Access results page through the Full Print workflow page.
        """
        if self._full_print_page is None:
            return
        results_page = self._full_print_page.results_page
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

    def _on_print_created(self, filename: str):
        """A Print Builder authoring page (Sketch / Image Import) baked a
        csv_import print file — load it into Print Setup's object list and
        jump to the Setup sub-page.

        v7.5.x: sources are Print Builder sub-pages; the print stack now lives
        in the Workflows "Full Print" tile.
        """
        if self._full_print_page is None:
            return
        setup_page = self._full_print_page.setup_page
        # The wizard PrintSetupPage composes the legacy page internally as
        # ``_legacy``; the objects tab (PrintObjectsTab) lives on either.
        tab = getattr(setup_page, 'tab_objects', None)
        if tab is None:
            legacy = getattr(setup_page, '_legacy', None)
            tab = getattr(legacy, 'tab_objects', None)
        if tab is not None:
            if hasattr(tab, '_load_print_file'):
                tab._load_print_file(filename)
            elif hasattr(tab, '_emit_prints_changed'):
                tab._emit_prints_changed()
        # Navigate to Workflows → Full Print → Setup sub-page.
        self._navigate_to(self._workflows_index())
        self._workflows_mode.open_workflow("full_print")
        self._full_print_page.switch_to_setup()

    def _on_print_files_changed(self):
        """The Print Library renamed/deleted/duplicated print files on disk —
        best-effort refresh of the pages that list saved prints so their combos
        don't go stale (both also re-scan on their own showEvent)."""
        # Quick Print objects combo.
        try:
            qp = self._workflows_mode.quick_print_page
            if qp is not None and hasattr(qp, "_refresh_objects"):
                qp._refresh_objects()
        except Exception as e:
            logger.debug(f"quick print refresh after file change failed: {e}")
        # Full Print objects tab list.
        try:
            if self._full_print_page is not None:
                setup_page = self._full_print_page.setup_page
                tab = getattr(setup_page, "tab_objects", None)
                if tab is None:
                    legacy = getattr(setup_page, "_legacy", None)
                    tab = getattr(legacy, "tab_objects", None)
                for m in ("_refresh_print_list_section", "_refresh_file_combo",
                          "_emit_prints_changed"):
                    if tab is not None and hasattr(tab, m):
                        getattr(tab, m)()
                        break
        except Exception as e:
            logger.debug(f"full print refresh after file change failed: {e}")


    # ════════════════════════════════════════════════════════════════
    #  HARDWARE CONFIG MANAGEMENT
    # ════════════════════════════════════════════════════════════════

    def _update_insert_clearance(self, cal_page) -> None:
        """Fast Move Z is authoritative for travel height — no auto floor.

        v7.5.x (operator decision, 2026-07-27: "when I set the Fast Move Z
        that is THE Z, and I'm already accounting for the top of the rosette
        inserts"): the operator sets the Fast Move / safe travel Z with the
        tallest insert already cleared, so the app no longer computes an
        ADDITIONAL tube-clearance floor and stacks it on top of that Z (which
        would double-count and silently raise the retract above the assigned
        Fast Move Z).

        This disarms the floor (``set_min_travel_z(None)``) — clearing any
        value a previous config push left armed — so every ``safe_travel_to``
        / ``ensure_retracted_to`` retracts to exactly the assigned safe Z and
        never silently raises above it.

        The floor MACHINERY (``StageController.set_min_travel_z`` +
        ``apply_insert_floor``) is left intact but disarmed — nothing in the
        app arms it now. Re-enable here if a future workflow ever needs an
        automatic clearance independent of the operator's Fast Move Z.
        """
        ctrl = getattr(self, "controller", None)
        if ctrl is None or not hasattr(ctrl, "set_min_travel_z"):
            return
        try:
            ctrl.set_min_travel_z(None)
        except Exception as e:
            logger.debug(f"_update_insert_clearance failed: {e}")

    def _update_print_floor_datum(self, cal_page) -> None:
        """v7.5.x: push the calibrated plate-bottom + plate-top Z to the
        controller.

        The plate bottom is the print-floor clamp (the needle can never punch
        through it while printing). The plate top, paired with the bottom, forms
        the reference vector that derives the print-Z up-direction
        (``StageController.print_z_dir()``) so print offsets are polarity-correct.
        Both are cleared (None) when not calibrated.
        """
        ctrl = getattr(self, "controller", None)
        if ctrl is None or not hasattr(ctrl, "set_plate_bottom_z"):
            return
        try:
            pb = pt = None
            if hasattr(cal_page, "get_z_references"):
                refs = cal_page.get_z_references()
                pb = refs.get("plate_bottom_z")
                pt = refs.get("plate_top_z")
            ctrl.set_plate_bottom_z(pb)
            if hasattr(ctrl, "set_plate_top_z"):
                ctrl.set_plate_top_z(pt)
        except Exception as e:
            logger.debug(f"_update_print_floor_datum failed: {e}")

    def _on_hardware_config_changed(self, config: HardwareConfig):
        """Called when hardware setup changes. Propagates to all pages.

        v7.4.0-b: Also emits hw_config_invalidated so downstream pages
        (Print Setup, Calibration) can surface a refresh banner.
        """
        if self._propagating_config:
            return  # Guard against re-entrant calls
        self._propagating_config = True
        try:
            prev = self._hardware_config
            self._hardware_config = config
            self._propagate_hardware_config(config)
            self._save_hardware_config(config)
            logger.info(f"Hardware config updated: {config}")
            # v7.4.0-b: Notify pages that derived data may be stale.
            self._emit_invalidation(prev, config)
        finally:
            self._propagating_config = False

    def _fanout_common_print_settings(self):
        """v7.5.x: push the shared CommonPrintSettings model to the Workflows
        mode so the Common Print Settings page + every workflow's inheriting
        fields re-sync to the current common values."""
        wf = getattr(self, "_workflows_mode", None)
        cps = getattr(self, "_common_print_settings", None)
        if wf is not None and cps is not None and hasattr(
                wf, "set_common_print_settings"):
            try:
                wf.set_common_print_settings(cps)
            except Exception as e:
                logger.debug("fanout common print settings failed: %s", e)

    def _on_common_setting_changed(self, common, key: str):
        """v7.5.x: a Common Print Settings value changed (from the Common page
        OR a workflow popout's global field). Persist + propagate.

        Global keys live on HardwareConfig (single source of truth) → persist +
        propagate the config. Promoted prep defaults persist to settings.json.
        Either way, re-fan-out so inheriting workflow fields re-sync.
        """
        try:
            if common.is_global(key):
                cfg = self._hardware_config
                if cfg is not None:
                    self._save_hardware_config(cfg)
                    self._propagate_hardware_config(cfg)
            else:
                self.settings.set(
                    "common_print_settings", common.promoted_dict())
                self.settings.save()
        except Exception as e:
            logger.warning("persist common setting '%s' failed: %s", key, e)
        self._fanout_common_print_settings()

    def _emit_invalidation(self, prev, current):
        """v7.4.0-b: Emit hw_config_invalidated with what changed.

        Payload is a dict of changed keys; subscribers can decide how
        much UI to invalidate based on which keys changed.

        v7.4.0-c: Also surfaces the global InvalidationBanner so users
        see something has changed even if no specific page subscribed.
        """
        changed = {}
        if prev is None or getattr(prev, 'plate_format', None) != \
                getattr(current, 'plate_format', None):
            changed["plate_format"] = getattr(current, 'plate_format', None)
        # v7.12: the plate IDENTITY matters as much as the well count. Switching
        # between two custom plates, or between two products of the same base
        # format, leaves `plate_format` untouched — so before this, changing
        # plate never invalidated anything. Every per-plate store (mosaics,
        # templates, well training, taught calibration) is keyed off exactly
        # these two fields.
        # v7.12: `plate_doc_id` joins these. It is what every per-plate store
        # keys on for a parametric design, and two plates may legitimately
        # share a display name — so watching `plate_name` alone would let a
        # switch between them go unnoticed and leave the calibration page on
        # the previous plate's taught wells.
        for attr in ("plate_name", "plate_type_id", "plate_doc_id"):
            if prev is None or getattr(prev, attr, None) != \
                    getattr(current, attr, None):
                changed[attr] = getattr(current, attr, None)
        if prev is None or getattr(prev, 'pumps', None) != \
                getattr(current, 'pumps', None):
            changed["pumps"] = True
        if changed:
            self.hw_config_invalidated.emit(changed)
            self._show_invalidation_banner(changed)

    def _show_invalidation_banner(self, changed: dict):
        """v7.4.0-c: Surface the global invalidation banner."""
        if not hasattr(self, '_invalidation_banner'):
            return
        # Don't show the banner while we're on the Hardware Setup page —
        # the user is actively editing config, no need to nag them.
        if self._current_page_index == 0:
            return
        parts = []
        if "plate_format" in changed:
            parts.append("plate format")
        if ("plate_name" in changed or "plate_type_id" in changed
                or "plate_doc_id" in changed):
            parts.append("plate")
        if "pumps" in changed:
            parts.append("pumps")
        if parts:
            self._invalidation_banner.set_message(
                f"Hardware changed ({', '.join(parts)}) — refresh pages to re-apply.")
        self._invalidation_banner.show()

    def _on_invalidation_refresh(self):
        """v7.4.0-c: User clicked Refresh on invalidation banner.

        Re-propagates the current hardware config to every page so they
        re-read their inputs.
        """
        if self._hardware_config is not None:
            self._propagate_hardware_config(self._hardware_config)
        logger.info("Hardware config re-propagated via invalidation refresh")

    def _on_console_collapsed(self, collapsed: bool):
        """v7.4.2: relax/restore the console pane minimum height so the
        Vertical splitter can actually shrink it to its toolbar when the
        user collapses the terminal."""
        if collapsed:
            self.console.setMinimumHeight(0)
            sizes = self._splitter.sizes()
            if len(sizes) == 2:
                toolbar_h = self.console.sizeHint().height()
                delta = max(0, sizes[1] - toolbar_h)
                self._splitter.setSizes([sizes[0] + delta, toolbar_h])
        else:
            self.console.setMinimumHeight(self._console_min_expanded)
            sizes = self._splitter.sizes()
            if len(sizes) == 2:
                target = max(self._console_min_expanded, sizes[1])
                delta = target - sizes[1]
                self._splitter.setSizes([max(0, sizes[0] - delta), target])

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

        # v7.5.x: keep the Common Print Settings model pointed at the live
        # config so its global params (settle / relief / prime) proxy it.
        cps = getattr(self, "_common_print_settings", None)
        if cps is not None:
            cps.set_hardware_config(config)

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

        # v7.5.x: the left-context host is not a page — push config to it so the
        # Custom view's config-driven sections (positions ranges, syringe,
        # hardware info) refresh too.
        host = getattr(self, "_context_host", None)
        if host is not None:
            try:
                host.set_hardware_config(config)
            except Exception as e:
                logger.debug("HW config → context host failed: %s", e)

    def _update_page_gating(self, hardware_valid: bool):
        """Enable/disable navigation buttons for pages requiring hardware setup.

        Index-agnostic: position 0 (Hardware Setup) and the Settings button
        (matched by objectName) are always enabled; everything else is gated
        on ``hardware_valid``.
        """
        for i, btn in enumerate(self._menu_buttons):
            if i == 0:
                # Hardware Setup — always enabled
                btn.setEnabled(True)
                btn.setToolTip("Hardware Setup")
            elif btn.objectName() == "btn_settings":
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

        # v7.4.3: Calibration moved above Jog Control. Index order
        # must match `menu_items` and the `pages` list in `_create_pages`.
        btn_map = {
            "btn_hardware":     0,
            "btn_calibrate":    1,
            "btn_jog":          2,
            "btn_printbuilder": 3,   # mode page (v7.5.x)
            "btn_workflows":    4,   # mode page (v7.4.3); hosts Full Print
            "btn_settings":     5,
        }
        index = btn_map.get(btn.objectName(), 0)
        self._navigate_to(index)

    def _workflows_index(self) -> int:
        """Page-stack index of the Workflows mode page. Derived from the live
        page list so it survives reindexing (e.g. the v7.5.x Printing-mode
        removal), with a static fallback."""
        try:
            return self._page_widgets.index(self._workflows_mode)
        except (ValueError, AttributeError):
            return 4

    def _navigate_to(self, index: int):
        """Switch to the page at the given index.

        v7.3.3: Mode pages (Printing, Pick & Place) delegate title/context
        to their active sub-page.
        """
        if index < 0 or index >= len(self._page_widgets):
            return

        self._current_page_index = index
        # v7.4.0-a: Smooth fade for page swaps (skipped for camera-bearing pages)
        fade_swap(self._page_stack, index)

        page = self._page_widgets[index]

        # Title: mode pages delegate to active sub-page
        title = "MEBP Bioprinter"
        if hasattr(page, 'get_page_title'):
            title = page.get_page_title()
        else:
            # v7.5.x: Printing mode re-homed into Workflows (Full Print tile).
            titles = ["Hardware Setup", "Calibration", "Jog Control",
                      "Print Builder", "Workflows", "Settings"]
            title = titles[index] if index < len(titles) else title
        self._page_title.setText(title)

        # v7.5.x: left context — one centralized refresh that mounts the page's
        # context widget in the host's Jog slot AND sets visibility explicitly.
        self._refresh_left_context()

        # Highlight the active menu button
        for i, btn in enumerate(self._menu_buttons):
            if i == index:
                btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))
            else:
                btn.setStyleSheet(UIFunctions.deselectMenu(btn.styleSheet()))

        # v7.4.2: same dance for the right context panel.
        right_ctx = None
        if hasattr(page, 'get_right_context_widget'):
            right_ctx = page.get_right_context_widget()
        if right_ctx is not None:
            # Stack index follows page index (placeholder QWidgets were
            # added for pages without a right context).
            self._right_context_stack.setCurrentIndex(index)
            self._right_context_title.setText(
                getattr(page, "get_right_context_title", lambda: "")()
                or "")
            self.ui_extraRightBox.show()
            # The splitter's stretch factor for the right pane is 0,
            # which means it keeps whatever its current width is —
            # since it was hidden at startup, that width is 0 even
            # after .show(). Explicitly allocate ~420 px on first
            # reveal; subsequent shows preserve whatever the user
            # last dragged it to.
            self._allocate_right_context_width()
            # A visible right pane eats into the room available to the
            # left panel — recompute its max drag width.
            self._update_context_panel_bounds()
        else:
            self.ui_extraRightBox.hide()
            self._update_context_panel_bounds()

    def resizeEvent(self, event):
        """Keep the context panel's drag bounds in sync with the window."""
        super().resizeEvent(event)
        self._update_context_panel_bounds()

    def showEvent(self, event):
        """Size the context panel once the window has a real geometry.

        ``_navigate_to(0)`` runs in __init__ (before show), when the
        splitter width is still 0 — sizing then would pin the panel at its
        minimum. Defer the first real sizing to here.
        """
        super().showEvent(event)
        if not getattr(self, "_context_sized_once", False):
            self._context_sized_once = True
            from PySide6.QtCore import QTimer
            QTimer.singleShot(0, self._apply_saved_context_width)

    def _apply_saved_context_width(self) -> None:
        """Open the left context panel at its remembered width.

        Refreshes the drag bounds first, then restores the saved width.
        Qt clamps the panel to its dynamic maximumWidth, so a saved width
        wider than the window currently allows simply yields the rest to
        content — no manual squeeze math, no jitter.
        """
        sp = getattr(self, "_context_splitter", None)
        if sp is None:
            return
        total = sp.width()
        if total <= 0 or not self.ui_extraLeftBox.isVisible():
            return
        self._update_context_panel_bounds()
        sizes = sp.sizes()
        right_w = sizes[2] if len(sizes) == 3 else 0
        min_left = (self.ui_extraLeftBox.minimumWidth()
                    or s(AppSettings.LEFT_BOX_WIDTH))
        saved = getattr(self, "_context_panel_width",
                        s(AppSettings.LEFT_BOX_WIDTH))
        target = max(min_left, saved)
        content_w = max(0, total - target - right_w)
        sp.setSizes([target, content_w, right_w])

    def _on_context_splitter_moved(self, pos: int, index: int) -> None:
        """Remember the panel width as the user drags (for hide/show restore).

        This only records the width — it does NOT call setSizes, so it can't
        fight the live drag. The actual drag bounds are enforced by the
        panel's dynamic maximumWidth (see _update_context_panel_bounds).
        """
        if self.ui_extraLeftBox.isVisible():
            w = self.ui_extraLeftBox.width()
            if w > 0:
                self._context_panel_width = w

    def _update_context_panel_bounds(self) -> None:
        """Recompute the left context panel's maximum drag width.

        The panel may grow until the content pane would be squeezed below
        ``_content_reserve_px`` (accounting for a visible right pane), but never
        below its own (small) minimum. The panel content is responsive, so a
        narrow panel fits its buttons by scaling/wrapping rather than clipping;
        this cap only keeps the panel from swallowing the whole window when
        dragged wide. With both panes non-collapsible, Qt also stops the drag at
        the content pane's own minimum.
        """
        sp = getattr(self, "_context_splitter", None)
        if sp is None:
            return
        total = sp.width()
        if total <= 0:
            return
        right_w = 0
        if self.ui_extraRightBox.isVisible():
            sizes = sp.sizes()
            if len(sizes) == 3:
                right_w = sizes[2]
        min_left = self.ui_extraLeftBox.minimumWidth() or s(AppSettings.LEFT_BOX_WIDTH)
        reserve = getattr(self, "_content_reserve_px", s(240))
        max_left = max(min_left, total - right_w - reserve)
        self.ui_extraLeftBox.setMaximumWidth(max_left)

    def _allocate_right_context_width(self) -> None:
        """v7.4.2: give the right context pane a real pixel width.

        Splitter's stretch factor on slot 2 is 0, so without an
        explicit ``setSizes`` the pane never gets any pixels even
        after ``.show()``. Default to ~420 px the first time it
        appears; if it's already > 50 px we trust the user's
        previous drag and leave it alone.
        """
        try:
            splitter = self._context_splitter
            sizes = splitter.sizes()
            if len(sizes) != 3:
                return
            target = s(420)
            if sizes[2] >= s(80):
                return  # already visible at a reasonable width
            total = sum(sizes) or splitter.width()
            left_w = sizes[0]
            right_w = min(target, max(s(280), total // 4))
            content_w = max(s(300), total - left_w - right_w)
            splitter.setSizes([left_w, content_w, right_w])
        except Exception as e:
            logger.debug(f"_allocate_right_context_width failed: {e}")

    def _on_mode_sub_page_changed(self, mode_page_index: int):
        """v7.3.3: When a mode/workflow page switches sub-pages, refresh the
        top-bar title AND the left context (content + visibility) in one place.

        v7.5.x: the left context is now managed solely by _refresh_left_context
        (which reads the active page's get_context_widget and sets visibility
        explicitly) — so a workflow tile switch can no longer leave the box
        shown-but-empty or content-mounted-but-hidden (the old jog-vanishing bug)."""
        if self._current_page_index == mode_page_index:
            page = self._page_widgets[mode_page_index]
            if hasattr(page, 'get_page_title'):
                self._page_title.setText(page.get_page_title())
            self._refresh_left_context()

    def _context_title_for(self, page, index: int) -> str:
        """The small label above the left context box."""
        if page is not None and hasattr(page, 'get_sub_page_title'):
            try:
                t = page.get_sub_page_title()
                if t:
                    return t
            except Exception:
                pass
        titles = ["Hardware", "Calibration", "Jog Settings",
                  "Print Builder", "Workflows", "Settings"]
        return titles[index] if 0 <= index < len(titles) else "Context"

    def _refresh_left_context(self) -> None:
        """v7.5.x: THE single source of truth for the left context box.

        Mounts the current page's context widget in the host's native (Jog) slot
        and sets the box's visibility EXPLICITLY (no animation toggle). Called
        from both _navigate_to and _on_mode_sub_page_changed so visibility and
        content can never desync — the root cause of the "jog panel sometimes
        disappears in a workflow" report.
        """
        host = getattr(self, "_context_host", None)
        if host is None:
            return
        idx = self._current_page_index
        page = (self._page_widgets[idx]
                if 0 <= idx < len(self._page_widgets) else None)

        native = None
        if page is not None and hasattr(page, "get_context_widget"):
            try:
                native = page.get_context_widget()
            except Exception as e:  # a delegate error must never vanish the box
                logger.debug("get_context_widget failed for page %s: %s",
                             type(page).__name__, e)
                native = None

        host.set_native_widget(native)
        host.set_native_available(native is not None)
        # Label the native pill for the current page ("Jog" for jog-capable
        # pages; the page's own controls elsewhere).
        cls = type(page).__name__ if page is not None else ""
        host.set_native_label(
            {"HardwareSetupPage": "Controls",
             "SettingsPage": "Safety"}.get(cls, "Jog"))
        self._context_title.setText(self._context_title_for(page, idx))

        # Scope = "everywhere the left box already appears": a page qualifies iff
        # it provides a native context widget (jog panel / control panel / quick
        # safety). The shared Custom view rides alongside it. Pages that return
        # None (Print Builder, Full Print inner, Fluorescence, Common Print
        # Settings, the workflow picker) keep NO left box — unchanged behaviour.
        in_scope = native is not None
        should_show = in_scope and not getattr(
            self, "_left_context_user_collapsed", False)

        box = self.ui_extraLeftBox
        if should_show:
            if not box.isVisible():
                box.show()
            if hasattr(self, "_apply_saved_context_width"):
                self._apply_saved_context_width()
        else:
            if box.isVisible():
                box.hide()
        self._update_context_panel_bounds()

    def _toggle_left_context(self) -> None:
        """Manual show/hide via the context close-x or the top-bar toggle. Flips
        the user's collapse preference, then re-evaluates (so re-showing mounts
        the right content). Persisted in save_settings."""
        self._left_context_user_collapsed = not getattr(
            self, "_left_context_user_collapsed", False)
        self._refresh_left_context()

    def _on_context_view_changed(self, view: str) -> None:
        """Persist the active pill (Jog / Custom) immediately."""
        try:
            self.settings.set("context_panel.active_view", view)
        except Exception as e:
            logger.debug("persist context_panel.active_view failed: %s", e)

    def _switch_page(self, index: int):
        """
        Public alias for _navigate_to, used by page signals
        (e.g., PrintSetupPage.navigate_to_page).
        """
        self._navigate_to(index)

    # ────────────────────────────────────────────────────────────────
    #  v7.4.0-a: Loading banner
    # ────────────────────────────────────────────────────────────────

    def show_loading(self, message: str):
        """Show the global loading banner with the given message."""
        if hasattr(self, '_loading_banner'):
            self._loading_banner.show_for(message)

    def hide_loading(self):
        """Hide the global loading banner."""
        if hasattr(self, '_loading_banner'):
            self._loading_banner.hide()

    # ────────────────────────────────────────────────────────────────
    #  v7.4.0-c: Help mode + FormRow registry
    # ────────────────────────────────────────────────────────────────

    def _on_help_toggled(self, on: bool):
        """Top-bar Help toggle clicked. Propagate to registered FormRows."""
        self._help_mode = on
        for row in list(self._registered_form_rows):
            try:
                row.set_help_visible(on)
            except Exception as e:
                logger.debug(f"FormRow.set_help_visible failed: {e}")
        self.help_mode_changed.emit(on)

    # ── Tutorial action recorder ─────────────────────────────────

    def _rec_button_qss(self, active: bool) -> str:
        """Idle it reads as a quiet secondary control; armed it is
        unmistakably red, because leaving it running by accident wastes a
        take and fills the disk with screenshots."""
        if active:
            return (f"QPushButton#recToggle {{ background: {COLORS['red']};"
                    f" color: {COLORS['crust']}; border: none;"
                    f" border-radius: {s(6)}px; padding: {s(5)}px {s(12)}px;"
                    f" font-size: {scaled_font_size(9)}pt; font-weight: 700; }}")
        return (f"QPushButton#recToggle {{ background: transparent;"
                f" color: {COLORS['overlay0']};"
                f" border: 1px solid {COLORS['surface1']};"
                f" border-radius: {s(6)}px; padding: {s(5)}px {s(12)}px;"
                f" font-size: {scaled_font_size(9)}pt; }}"
                f"QPushButton#recToggle:hover {{ color: {COLORS['red']};"
                f" border-color: {COLORS['red']}; }}")

    def _on_toggle_recording(self):
        """Arm/disarm the walkthrough recorder from the top bar."""
        try:
            if self._action_recorder is None:
                from gui.action_recorder import ActionRecorder
                self._action_recorder = ActionRecorder(self)
                # Never record the stop-click on the REC button itself — the
                # press arrives while still armed (stop fires on release), so
                # without this every tape ends with a junk final step.
                self._action_recorder.ignore_widget(self._rec_btn)
                self._action_recorder.step_recorded.connect(
                    self._on_recorder_step)
                self._action_recorder.step_marked.connect(
                    self._on_recorder_mark)

            rec = self._action_recorder
            if rec.armed:
                path = rec.stop()
                self._rec_btn.setChecked(False)
                self._rec_btn.setText("● REC")
                # Release the fixed width reserved while armed.
                self._rec_btn.setMinimumWidth(0)
                self._rec_btn.setMaximumWidth(16777215)  # QWIDGETSIZE_MAX
                self._rec_btn.setStyleSheet(self._rec_button_qss(False))
                self.console.log(
                    f"Recording stopped — {rec.step_count} step(s), "
                    f"{rec.mark_count} marked → {path}", "success")
                self.console.log(
                    "Build a tutorial from it:  python "
                    "tools_build_tour_from_tape.py \"%s\"" % path, "info")
            else:
                session = rec.start()
                self._rec_btn.setChecked(True)
                # Fix the width for the whole take: the counter must never
                # relayout the top bar mid-recording (geometry shifts would
                # land in the captured frames and desync top-bar rects).
                fm = self._rec_btn.fontMetrics()
                self._rec_btn.setFixedWidth(
                    fm.horizontalAdvance("● REC 999 ★99") + s(28))
                self._rec_btn.setText("● REC 0")
                self._rec_btn.setStyleSheet(self._rec_button_qss(True))
                self.console.log(
                    f"Recording walkthrough → {session}  "
                    f"(F9 marks a step that matters)", "warning")
        except Exception as e:
            logger.error("recorder toggle failed: %s", e, exc_info=True)
            self._rec_btn.setChecked(False)
            self._rec_btn.setMinimumWidth(0)
            self._rec_btn.setMaximumWidth(16777215)
            self._rec_btn.setStyleSheet(self._rec_button_qss(False))
            # Silent-grey-button failure is how an operator loses a 10-minute
            # take believing it was recorded — say it where they look.
            try:
                self.console.log(f"⚠ Recorder failed: {e}", "error")
            except Exception:
                pass

    def _on_recorder_step(self, steps: int, marks: int):
        self._rec_btn.setText(
            f"● REC {steps} ★{marks}" if marks else f"● REC {steps}")

    def _on_recorder_mark(self, step_index: int):
        """F9 feedback — the button already re-renders via _on_recorder_step's
        next tick; this makes the mark itself visible immediately."""
        rec = self._action_recorder
        if rec is not None:
            self._rec_btn.setText(f"● REC {rec.step_count} ★{rec.mark_count}")
        try:
            self.console.log(f"★ marked step {step_index}", "success")
        except Exception:
            pass

    def register_form_row(self, row):
        """Register a FormRow so the top-bar Help toggle controls it.

        Pages call this for each FormRow they construct; the toggle is
        applied immediately so newly-registered rows match current state.
        """
        if row not in self._registered_form_rows:
            self._registered_form_rows.append(row)
        # Apply current state immediately
        try:
            row.set_help_visible(self._help_mode)
        except Exception:
            pass

    @property
    def help_mode(self) -> bool:
        return self._help_mode

    # ════════════════════════════════════════════════════════════════
    #  TIMERS & STATUS UPDATES
    # ════════════════════════════════════════════════════════════════

    def _setup_timers(self):
        """Start the position/status polling timer using single-shot reschedule.

        Single-shot prevents Qt from queuing up back-to-back timer callbacks
        when _update_status() occasionally runs over the interval — the next
        tick is only scheduled after the current one fully completes.
        """
        self._closing = False
        self._tick()

        # v7.5.x: fast display-only animation tick (~30 fps). It does real work
        # ONLY while a jog/travel motion estimate is live (see _motion_anim_tick),
        # so it is a cheap boolean check at idle. Repeating QTimer — the work is
        # light (label/needle refresh), so no single-shot reschedule is needed.
        self._motion_anim_timer = QTimer(self)
        self._motion_anim_timer.setInterval(33)
        self._motion_anim_timer.timeout.connect(self._motion_anim_tick)
        self._motion_anim_timer.start()

    def _tick(self):
        if getattr(self, "_closing", False):
            return
        import time as _time
        _t0 = _time.monotonic()
        try:
            self._update_status()
        finally:
            elapsed_ms = (_time.monotonic() - _t0) * 1000
            # v7.5.x perf diag: on a slow tick, OR once per ~60 s, log the tick
            # cost + the live Qt widget count. A "slow over time" regression
            # then shows up in logs/app.log as a widget count that CLIMBS across
            # a session (a leak) vs a roughly constant count (uniform slowness).
            # The allWidgets() scan only runs on those infrequent samples.
            self._tick_count = getattr(self, "_tick_count", 0) + 1
            if elapsed_ms > 20 or (self._tick_count % 200 == 0):
                try:
                    from PySide6.QtWidgets import QApplication as _QApp
                    _nw = len(_QApp.allWidgets())
                except Exception:
                    _nw = -1
                logger.debug(
                    f"[Tick] _update_status {elapsed_ms:.1f}ms | "
                    f"widgets={_nw} | tick#{self._tick_count}")
            if not getattr(self, "_closing", False):
                interval = self.settings.get("polling.position_interval_ms", 300)
                QTimer.singleShot(interval, self._tick)

    def _emit_stage_disconnected(self, stage_name: str) -> None:
        """v7.5.x: StageController.on_disconnect callback. Runs on the
        watchdog/poller thread — only emit the signal here; never touch
        widgets directly (the queued connection hops to the GUI thread)."""
        try:
            self.stage_disconnected.emit(str(stage_name))
        except Exception:
            pass

    def _on_stage_disconnected(self, stage_name: str) -> None:
        """v7.5.x: GUI-thread slot — refresh connection state immediately
        when a stage drops instead of waiting for the next ~300ms poll tick.
        The hardened is_*_connected properties make this reflect reality."""
        logger.warning(f"{stage_name} stage disconnected — refreshing status")
        try:
            self._update_status()
        except Exception as e:
            logger.debug(f"status refresh after disconnect failed: {e}")

    def _emit_stage_connected(self, stage_name: str) -> None:
        """v7.5.x: StageController.on_connect callback. May run on a worker
        thread (onboarding) — only emit the signal here; the queued slot
        hops to the GUI thread before touching widgets."""
        try:
            self.stage_connected.emit(str(stage_name))
        except Exception:
            pass

    def _on_stage_connected(self, stage_name: str) -> None:
        """v7.5.x GUI-thread slot for a successful (re)connect."""
        if stage_name == "ZP":
            # Defer so the prompt never opens mid-connect-callstack (the
            # signal is a direct call when connect ran on the GUI thread).
            QTimer.singleShot(0, self._maybe_prompt_zp_position_restore)

    def _maybe_prompt_zp_position_restore(self) -> None:
        """v7.5.x: on the first real ZP connect this session, offer to
        restore the last-known ZP position saved at the previous clean
        shutdown. Marlin has no absolute encoder and powers up at 0, so
        the saved snapshot (plus the operator confirming nothing moved by
        hand) is the only way to recover the prior position. Accepting
        re-stamps the firmware counter per axis via override_zp_position
        (G92, no motion)."""
        if self._zp_restore_prompted:
            return
        zp = self.controller.zp_stage
        if zp is None:
            return
        if getattr(zp, "simulate", False):
            self._zp_restore_prompted = True  # never nag in simulation
            return
        snap = self.settings.get_section("zp_last_position")
        if not snap:
            return
        axes = [(ax, snap.get(ax)) for ax in ("Z", "P1", "P2", "P3")
                if isinstance(snap.get(ax), (int, float))]
        if not axes:
            return
        self._zp_restore_prompted = True

        from PySide6.QtWidgets import QMessageBox
        ts = snap.get("timestamp", "an earlier session")
        lines = "\n".join(f"    {ax} = {float(v):.3f} mm" for ax, v in axes)
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Question)
        box.setWindowTitle("Restore ZP position?")
        box.setText(
            "The ZP stage (Marlin) has no absolute position memory and "
            "powers up reporting 0.\n\n"
            f"Last known position, saved {ts}:\n{lines}\n\n"
            "If the stage has NOT been moved by hand since then, assign "
            "these as the current position for all ZP axes?")
        box.setInformativeText(
            "This re-stamps the firmware position counter (G92) — no motion "
            "occurs. Choose Skip if unsure; you can set positions manually "
            "in Hardware Setup → Stage → Override / Sync Axis Position.")
        assign_btn = box.addButton(
            "Assign these values", QMessageBox.ButtonRole.AcceptRole)
        box.addButton("Skip", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(assign_btn)
        box.exec()
        if box.clickedButton() is not assign_btn:
            logger.info("ZP position restore skipped by user")
            return
        applied = []
        for ax, v in axes:
            res = self.controller.override_zp_position(ax, float(v))
            if res.get("ok"):
                applied.append(f"{ax}={float(v):.3f}")
            else:
                logger.warning(
                    f"ZP restore {ax} failed: {res.get('error', 'unknown')}")
        logger.info(f"ZP position restored: {', '.join(applied) or 'none'}")
        # v7.5.x: lock the limits into the live controller + persist to disk now
        # (the same Apply+Save the operator otherwise had to do by hand).
        self._apply_and_save_after_restore("ZP position restore")
        try:
            self._update_status()
        except Exception:
            pass

    def _maybe_show_calibration_status(self) -> None:
        """v7.5.x: once per session, show the calibration-usability pop-up —
        but only when attention is needed (a recalibration threshold crossed,
        or a type never calibrated). A clean setup is never interrupted; the
        operator can still open it on demand via Hardware Setup → Device →
        "Calibration status…". See gui/dialogs/calibration_status_dialog.py."""
        if self._usability_prompted:
            return
        self._usability_prompted = True
        try:
            from gui.dialogs.calibration_status_dialog import (
                show_calibration_status)
            show_calibration_status(self, force=False)
        except Exception as e:
            logger.debug(f"calibration status pop-up skipped: {e}")

    def _maybe_prompt_calibration_restore(self) -> None:
        """v7.5.x: once per session, if the live calibration came up empty but
        a last-known-good snapshot exists, offer to restore it (needle zero +
        plate wells/warp + Z plane/heights) as a unit. Mirrors
        _maybe_prompt_zp_position_restore.

        Only fires when the plate calibration is actually missing, so a healthy
        restart (calibration auto-loaded fine) is never interrupted. Warns when
        the hardware fingerprint shows the setup changed since the snapshot was
        saved (applying it anyway may be inaccurate)."""
        if self._calibration_restore_prompted:
            return
        self._calibration_restore_prompted = True
        cal_page = self._page_widgets[1] if len(self._page_widgets) > 1 else None
        if cal_page is None or not hasattr(cal_page, '_has_plate_calibration'):
            return
        # Calibration survived this restart → nothing to recover, don't nag.
        if cal_page._has_plate_calibration():
            return
        try:
            from SupportClasses.CalibrationSnapshotStore import (
                CalibrationSnapshotStore)
            store = CalibrationSnapshotStore()
        except Exception as e:
            logger.debug(f"calibration snapshot load failed: {e}")
            return
        snap = store.load_snapshot()
        if not snap or not snap.get("calibration"):
            return

        saved_at = snap.get("saved_at", "an earlier session")
        diffs = store.fingerprint_diff(
            snap.get("fingerprint"), store.build_fingerprint(self.settings))

        from PySide6.QtWidgets import QMessageBox
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning if diffs
                    else QMessageBox.Icon.Question)
        box.setWindowTitle("Restore last calibration?")
        box.setText(
            f"A saved calibration from {saved_at} was found, and this session "
            "has no plate calibration loaded.\n\n"
            "Restore it (needle zero + plate wells + Z plane / heights)?")
        info = ("This re-applies the needle zero reference (no motion) and the "
                "plate / Z calibration. Choose Skip to calibrate fresh.")
        if diffs:
            info = ("⚠ The hardware setup looks DIFFERENT from when this "
                    "calibration was saved:\n    "
                    + "\n    ".join(diffs)
                    + "\n\nRestoring may be inaccurate — recalibrate if unsure."
                    + "\n\n" + info)
        box.setInformativeText(info)
        restore_btn = box.addButton(
            "Restore", QMessageBox.ButtonRole.AcceptRole)
        box.addButton("Skip", QMessageBox.ButtonRole.RejectRole)
        if not diffs:
            box.setDefaultButton(restore_btn)
        box.exec()
        if box.clickedButton() is not restore_btn:
            logger.info("Calibration restore skipped by user")
            return

        # Apply needle zero — a software reference only (no motion).
        zp = snap.get("zero_position") or {}
        if isinstance(zp, dict) and zp:
            try:
                self.controller.zero_position.update(
                    {k: v for k, v in zp.items()
                     if isinstance(v, (int, float))})
                self.settings.set_section(
                    "zero_position", dict(self.controller.zero_position))
            except Exception as e:
                logger.warning(f"Failed to apply restored zero_position: {e}")

        # Restore plate + Z into the live section and reload the page through
        # its normal load path (recomputes predicted/calibrated positions); the
        # emit re-pushes to the Jog page / Workflows mode.
        try:
            self.settings.set_section("calibration", dict(snap["calibration"]))
            cal_page._load_calibration()
            if hasattr(cal_page, '_emit_calibration_data_changed'):
                cal_page._emit_calibration_data_changed()
            logger.info(f"Calibration restored from snapshot ({saved_at})")
        except Exception as e:
            logger.error(f"Failed to restore calibration snapshot: {e}")

        # v7.5.x: do the Hardware Setup → Device "Apply + Save" automatically so
        # the restored setup is locked into the live limits and persisted to
        # disk right now — previously the restore only mutated in-memory state,
        # so the operator had to go to the Device page and Apply+Save by hand
        # (and the restored values were lost until the next clean shutdown).
        self._apply_and_save_after_restore("calibration restore")

        try:
            self._update_status()
        except Exception:
            pass

    def _apply_and_save_after_restore(self, what: str) -> None:
        """After accepting a last-known restore, replicate the Device-page
        "Apply + Save" so the operator doesn't have to:

        * **Apply** — mirror the persisted ``safety_limits`` into the LIVE
          envelope object (mutating its fields in place; never rebinding it, so
          the jog handlers' held reference stays valid). Idempotent — it just
          guarantees the live limits match what's on disk after the restore /
          ZP reconnect.
        * **Save** — persist everything to disk now via ``save_settings`` (the
          same path a clean shutdown uses: window geo, hardware config,
          ``zero_position``, ``zp_last_position``, and the calibration
          snapshot), instead of waiting for a clean shutdown.

        Then refresh the left-panel limit bars so the change is visible.
        """
        # Apply: mirror persisted limits into the live envelope (in place).
        try:
            saved = self.settings.get_section("safety_limits") or {}
            sl = getattr(self.controller, "safety_limits", None)
            if sl is not None and saved:
                for key, val in saved.items():
                    if hasattr(sl, key) and isinstance(val, (int, float, bool)):
                        setattr(sl, key, val)
        except Exception as e:
            logger.warning(f"{what}: applying safety limits failed: {e}")
        # v7.5.x: a CALIBRATED pump's soft-limit envelope is owned by the
        # plunger calibration (``pump_setup``), NOT the persisted safety_limits
        # mirror — the same way ``z_up_sign`` owns Z (CLAUDE.md principle #4).
        # The mirror above can be stale/sign-flipped (e.g. ME3B V3's on-disk
        # ``p2 [0, 30]`` vs the captured raw ``0 → -30 ⇒ [-30, 0]``); blindly
        # mirroring it here re-broke the envelope that ``apply_pump_convention``
        # had correctly re-derived at boot, so every aspirate clamped to 0 and
        # ink pickup did nothing. Re-derive each calibrated pump's envelope from
        # the authoritative captured extremes so calibration wins over the
        # mirror (exact extremes, no margin — matches ``apply_pump_setup`` /
        # ``apply_pump_convention``). Uncalibrated pumps keep the mirror.
        try:
            pump_setup = self.controller.get_pump_setup()
            if sl is not None and pump_setup:
                for pump, s in pump_setup.items():
                    rd, ra = s.get("raw_dispensed"), s.get("raw_aspirated")
                    if rd is None or ra is None:
                        continue
                    lo, hi = min(float(rd), float(ra)), max(float(rd), float(ra))
                    setattr(sl, f"{pump.lower()}_min", lo)
                    setattr(sl, f"{pump.lower()}_max", hi)
        except Exception as e:
            logger.warning(f"{what}: re-deriving pump envelope failed: {e}")
        # Save: persist to disk now (not just on clean shutdown).
        try:
            self.save_settings()
            logger.info(f"{what}: applied limits + saved configuration to disk")
        except Exception as e:
            logger.warning(f"{what}: save_settings failed: {e}")
        # Refresh the persistent left-panel position/limit bars so the new
        # envelope extents show immediately (best-effort).
        try:
            hw_page = self._page_widgets[0] if self._page_widgets else None
            cp = getattr(hw_page, "_control_panel", None) if hw_page else None
            if cp is not None and hasattr(cp, "refresh_safety_limits"):
                cp.refresh_safety_limits()
        except Exception:
            pass

    def _refresh_all_speed_limits(self):
        """v7.5.x: re-apply the per-axis max anchors to the jog handlers and
        fan a ``refresh_speed_limits()`` refresh out to every page / jog-context
        panel that has one, so the single common speed source is reflected
        everywhere live. Best-effort; runs on the GUI thread."""
        try:
            if self.controller is not None and hasattr(
                    self.controller, "refresh_jog_speed_limits"):
                self.controller.refresh_jog_speed_limits()
        except Exception as e:
            logger.debug(f"refresh_jog_speed_limits failed: {e}")
        seen: set[int] = set()

        def _refresh(obj):
            if obj is None or id(obj) in seen:
                return
            seen.add(id(obj))
            fn = getattr(obj, "refresh_speed_limits", None)
            if callable(fn):
                try:
                    fn()
                except Exception:
                    pass
        for page in (getattr(self, "_page_widgets", None) or []):
            _refresh(page)
            for attr in ("_control_panel", "_context_widget",
                         "_context_panel", "_hw_panel"):
                _refresh(getattr(page, attr, None))

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
            if proto_val is not None and proto_val != self._xy_position_scale:
                self.xy_position_scale = proto_val

        # Position readouts (display getters → animate during a move)
        self._update_status_bar_positions()

        # Safety status — only re-style when state changes
        sl = self.controller.safety_limits
        _safety_on = sl.enabled
        if getattr(self, "_safety_state", None) != _safety_on:
            self._safety_state = _safety_on
            if _safety_on:
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

        # v7.5.x: the left-context host lives outside the page stack — tick its
        # Custom view's live sections (camera / syringe / positions) directly.
        host = getattr(self, "_context_host", None)
        if host is not None:
            try:
                host.on_status_update()
            except Exception:
                pass

    def _update_status_bar_positions(self) -> None:
        """v7.5.x: status-bar XY/Z/pump readouts, sourced from the DISPLAY
        getters so they ANIMATE toward the destination during a jog/travel move
        and snap to the measured value on arrival. Estimated axes are marked
        with a leading '~' (see MEBP_v75x_JOG_MOTION_INTERPOLATION)."""
        try:
            c = self.controller
            xy = (c.get_display_xy_position()
                  if hasattr(c, "get_display_xy_position")
                  else c.get_xy_position(cached=True))
            zp = (c.get_display_zp_position()
                  if hasattr(c, "get_display_zp_position")
                  else c.get_zp_position(cached=True))
            speeds = c.get_speed_info()
            est = c.motion_estimating() if hasattr(c, "motion_estimating") else set()

            def _mark(axis: str, text: str) -> str:
                return ("~" + text) if axis in est else text

            # XY in µm
            if xy[0] is not None:
                zx = xy[0] - c.zero_position["x"]
                zy = xy[1] - c.zero_position["y"]
                ux = stage_to_um(zx, self._xy_position_scale)
                uy = stage_to_um(zy, self._xy_position_scale)
                self.sb_xy.setText(
                    f"XY: {_mark('X', f'{ux:,.1f}')} , "
                    f"{_mark('Y', f'{uy:,.1f}')} µm")
            else:
                self.sb_xy.setText("XY: — , — µm")

            # Z in mm, Pumps in µL when syringe configured else mm.
            # v7.4.2 hotfix: route ZP reads through logical axis (axis_map).
            z_val = c.zp_logical_value(zp, "Z")
            if z_val is not None:
                zz = c.zero_position.get("Z", 0)
                self.sb_z.setText(f"Z: {_mark('Z', f'{z_val - zz:.2f}')}")

                for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
                    pos_mm = c.zp_logical_value(zp, pid)
                    lbl = getattr(self, f"sb_p{idx}", None)
                    if lbl is None:
                        continue
                    zero_ref = c.zero_position.get(pid, 0)
                    if pos_mm is not None and self._hardware_config:
                        pump_cfg = self._hardware_config.pumps.get(pid)
                        if pump_cfg and pump_cfg.is_configured:
                            try:
                                pos_uL = pump_cfg.mm_to_uL(pos_mm - zero_ref)
                                lbl.setText(
                                    f"{pid}: {_mark(pid, f'{pos_uL:.2f}')} µL")
                                continue
                            except ValueError:
                                pass
                    if pos_mm is not None:
                        lbl.setText(
                            f"{pid}: {_mark(pid, f'{pos_mm - zero_ref:.2f}')} mm")
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

    def _motion_anim_tick(self) -> None:
        """v7.5.x: fast (~33 ms) display-only animation tick. Does work ONLY
        while a jog/travel motion estimate is live — refreshes the status-bar
        positions and the active page's needle/readout so they glide toward the
        destination between the ~300 ms hardware polls, then idle (cheap)."""
        if getattr(self, "_closing", False):
            return
        try:
            c = self.controller
            if c is None or not hasattr(c, "motion_estimate_active"):
                return
            if not c.motion_estimate_active():
                return
            self._update_status_bar_positions()
            page = self._page_widgets[self._current_page_index]
            fn = getattr(page, "on_motion_tick", None)
            if callable(fn):
                fn()
            host = getattr(self, "_context_host", None)
            if host is not None:
                host.on_motion_tick()
        except Exception:
            pass

    def _update_conn_dot(self, name: str, state: str):
        """Update a connection status dot — only re-styles when state changes.

        unpolish()/polish() re-evaluate the entire Qt stylesheet; calling them
        every tick even when nothing changed caused severe per-tick overhead.
        """
        if getattr(self, f"_conn_state_{name}", None) == state:
            return
        setattr(self, f"_conn_state_{name}", state)

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
        # v7.5.x: persist the left-context pill + collapse state (the custom
        # layout itself lives in ContextPanelLayoutStore's own JSON).
        try:
            host = getattr(self, "_context_host", None)
            if host is not None:
                self.settings.set("context_panel.active_view",
                                  host.requested_view())
            self.settings.set("context_panel.collapsed",
                              bool(getattr(self, "_left_context_user_collapsed",
                                           False)))
        except Exception as e:
            logger.debug("persist context_panel state failed: %s", e)
        if self._hardware_config:
            self._save_hardware_config(self._hardware_config)
        # v7.5.x: persist the live zero reference on clean shutdown so it
        # survives restart. The explicit "Set Zero" buttons already save
        # zero_position on demand, but non-button paths (the Xbox
        # "zero_needle_pos" → _calibrate_zero, auto-calibration) mutate it
        # only in RAM. Without this catch-all the reference reverts to the
        # last button-saved value on restart, and since the ProScan keeps
        # its absolute position while powered, displayed = raw − zero_position
        # then reads wrong (the reported XY-restart bug).
        try:
            self.settings.set_section(
                "zero_position", dict(self.controller.zero_position))
        except Exception as e:
            logger.warning(f"Failed to persist zero_position on shutdown: {e}")
        # v7.5.x: snapshot the last-known ZP position (zero-ref mm) + timestamp
        # so the next startup can offer to restore it (Marlin loses its
        # position on power cycle). Only overwrite when ZP is connected and we
        # got a real reading — never clobber a good saved snapshot with a
        # disconnected-stage blank.
        try:
            if self.controller.is_zp_connected:
                zr = self.controller.get_zp_position_zero_ref(cached=True)
                if any(v is not None for v in zr.values()):
                    from datetime import datetime
                    snap = {ax: (round(float(v), 4) if v is not None else None)
                            for ax, v in zr.items()}
                    snap["timestamp"] = datetime.now().isoformat(
                        timespec="seconds")
                    self.settings.set_section("zp_last_position", snap)
        except Exception as e:
            logger.warning(f"Failed to persist zp_last_position on shutdown: {e}")
        # v7.5.x: flush a final last-known-good calibration snapshot so an
        # in-progress teach (not yet written by the 500 ms debounced auto-save)
        # is captured on clean exit. _save_calibration writes both the live
        # section and the durable snapshot; the snapshot write is itself gated
        # on a real plate calibration, so this is a no-op when nothing's taught.
        try:
            cal_page = (self._page_widgets[1]
                        if len(self._page_widgets) > 1 else None)
            if (cal_page is not None
                    and getattr(cal_page, '_has_plate_calibration', None)
                    and cal_page._has_plate_calibration()):
                cal_page._save_calibration()
        except Exception as e:
            logger.debug(f"calibration snapshot flush on shutdown failed: {e}")
        # v7.5.x: persist any throttled XY-travel odometer accumulation.
        try:
            from SupportClasses.CalibrationStatusStore import get_store
            get_store().flush()
        except Exception as e:
            logger.debug(f"calibration status flush on shutdown failed: {e}")
        self.settings.save()

    # ════════════════════════════════════════════════════════════════
    #  CLEANUP
    # ════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        """Clean shutdown — stop timers, save settings, stop recording."""
        # v7.18: ask about the incubator heaters BEFORE anything is torn down
        # (the prompt needs a live event loop, and the answer decides what
        # shutdown_incubator() commands). Only when a zone is actually
        # heating — a cold, idle session closes silently. NOTE closing the
        # app never stops the heaters by itself: the firmware owns the
        # control loop and holds its last setpoint (see
        # SupportClasses/incubator/README heritage) — which is exactly why
        # the question is worth asking rather than silently choosing.
        incubator_heaters_off = True
        try:
            from PySide6.QtWidgets import QMessageBox
            from SupportClasses.incubator.service import peek_incubator
            from SupportClasses.incubator.config_store import get_store as \
                _incu_store
            _inc = peek_incubator()
            if _inc is not None and _inc.connected:
                from SupportClasses.incubator.zones import ALL_ZONES as _IZ
                _heating = any(
                    _inc.zone_runtime(z.zone_id).requested_c > 0 for z in _IZ)
                if _heating:
                    _default_off = bool(
                        _incu_store().get("heaters_off_on_app_exit", True))
                    _ans = QMessageBox.question(
                        self, "Incubator is heating",
                        "The incubator is holding a temperature. Closing the "
                        "app does NOT stop the board — Marlin keeps its own "
                        "control loop running and will hold the current "
                        "setpoint.\n\n"
                        "Turn both heaters OFF before closing?\n\n"
                        "Yes = turn heaters off\n"
                        "No  = leave them running for a long soak",
                        QMessageBox.Yes | QMessageBox.No,
                        QMessageBox.Yes if _default_off else QMessageBox.No,
                    )
                    incubator_heaters_off = (_ans == QMessageBox.Yes)
        except Exception as e:
            logger.debug(f"incubator close prompt skipped: {e}")

        self._closing = True  # stops the self-rescheduling _tick loop
        _at = getattr(self, "_motion_anim_timer", None)
        if _at is not None:
            _at.stop()
        self.save_settings()
        if self.recorder and self.recorder.is_recording:
            self.recorder.stop_recording()

        # Flush an armed walkthrough tape — otherwise closing mid-recording
        # loses the whole take.
        try:
            if self._action_recorder is not None and self._action_recorder.armed:
                self._action_recorder.stop()
        except Exception as e:
            logger.debug("action recorder shutdown failed: %s", e)

        # Shut down any background threads owned by pages
        for page in self._page_widgets:
            if hasattr(page, '_shutdown_detection_worker'):
                page._shutdown_detection_worker()
            # v7.5.x: also stop the Plate Location mosaic scan thread so we
            # don't tear down the controller/page under a running QThread.
            if hasattr(page, '_shutdown_mosaic_worker'):
                page._shutdown_mosaic_worker()

        # v7.3.3: Stop all cameras
        if hasattr(self, '_camera_manager'):
            self._camera_manager.shutdown()

        # v7.5.x: release the microscope body (COM / Micro-Manager) and stop its
        # worker thread. Best-effort — a wedged turret must not block the close.
        try:
            from SupportClasses.MicroscopeControl import shutdown_microscope
            shutdown_microscope()
        except Exception as e:
            logger.debug(f"microscope shutdown failed: {e}")

        # v7.18: close the incubator session, honouring the prompt answer.
        # ORDERING: before controller.shutdown() — in shared-transport mode
        # the heater-off commands ride the ZP link, which disconnect_stages()
        # is about to close. peek-only, so an untouched incubator costs
        # nothing here.
        try:
            from SupportClasses.incubator.service import shutdown_incubator
            shutdown_incubator(heaters_off=incubator_heaters_off)
        except Exception as e:
            logger.debug(f"incubator shutdown failed: {e}")

        # v7.17: stop the LabLink upload worker and STATE what was not sent —
        # the operator chose in-session-only retry, which is honest only if
        # the loss is named at the moment it happens. WorkflowsModePage has no
        # shutdown fan-out, so the service is stopped here directly.
        try:
            from SupportClasses.LabLinkService import peek_service
            _svc = peek_service()
            if _svc is not None:
                _report = _svc.stop(timeout=2.0)
                if _report.get("unsent"):
                    logger.warning(
                        "LabLink: %d queued upload(s) were dropped by this "
                        "close and will not be retried", _report["unsent"])
        except Exception as e:
            logger.debug(f"LabLink shutdown failed: {e}")

        self.controller.shutdown()
        event.accept()
