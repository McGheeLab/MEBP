"""
app_modifications.py — Changes required to gui/app.py for v7.2 Hardware Setup gating.

This file documents the modifications needed. It provides the new/changed methods
that should be integrated into the existing MainWindow class in gui/app.py.

KEY CHANGES:
1. Import HardwareSetupPage
2. Add it as page index 0 (shift all other page indices +1)
3. Disable non-setup pages until hardware config is valid
4. Wire config_changed / config_validated signals
5. Propagate HardwareConfig to all pages that need it
6. Store/restore HardwareConfig from settings.json
"""

# ═══════════════════════════════════════════════════════════════════
# ADD TO IMPORTS (at top of app.py)
# ═══════════════════════════════════════════════════════════════════

# Add this import alongside the existing page imports:
# from gui.pages.hardware_setup import HardwareSetupPage
# from SupportClasses.HardwareConfig import HardwareConfig


# ═══════════════════════════════════════════════════════════════════
# REPLACE _create_pages() method
# ═══════════════════════════════════════════════════════════════════

def _create_pages_v72(self):
    """
    Create all application pages.
    Page 0 is Hardware Setup (always enabled).
    Pages 1-6 are disabled until hardware config is valid.
    """
    # Hardware config — restored from settings or fresh
    self._hardware_config = self._restore_hardware_config()

    pages = [
        HardwareSetupPage(),                                          # 0: Hardware Setup
        DashboardPage(self.controller, self.print_history),           # 1: Dashboard
        JogControlPage(self.controller),                              # 2: Jog Control
        CalibrationPage(self.controller, settings=self.settings),     # 3: Calibration
        PrintSetupPage(self.controller),                              # 4: Print Setup
        PrintMonitorPage(self.controller, self.settings),             # 5: Print Monitor
        SettingsPage(self.controller, self.settings),                 # 6: Settings
    ]

    # Wire Hardware Setup signals
    hw_page = pages[0]
    hw_page.config_changed.connect(self._on_hardware_config_changed)
    hw_page.config_validated.connect(self._on_hardware_validated)

    # Restore saved config if available
    if self._hardware_config:
        hw_page.set_config(self._hardware_config)

    # Wire recorder
    if self.recorder:
        monitor = pages[5]
        if hasattr(monitor, 'set_recorder'):
            monitor.set_recorder(self.recorder)
        setup = pages[4]
        if hasattr(setup, 'print_manager') and setup.print_manager:
            setup.print_manager.recorder = self.recorder

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

        # Propagate microsteps
        if hasattr(page, 'set_microsteps_per_micron'):
            page.set_microsteps_per_micron(self._microsteps_per_micron)

    # Initial page gating
    self._update_page_gating(self._hardware_config is not None and self._hardware_config.is_valid)


# ═══════════════════════════════════════════════════════════════════
# NEW METHODS TO ADD TO MainWindow
# ═══════════════════════════════════════════════════════════════════

def _on_hardware_config_changed(self, config):
    """Called when hardware setup changes. Propagates to all pages."""
    self._hardware_config = config

    # Propagate to pages that need it
    for page in self._page_widgets:
        if hasattr(page, 'set_hardware_config'):
            page.set_hardware_config(config)

    # Save to settings for persistence
    self._save_hardware_config(config)

    logger.info(f"Hardware config updated: {config}")


def _on_hardware_validated(self, is_valid: bool):
    """Called when hardware setup validity changes. Gates other pages."""
    self._update_page_gating(is_valid)
    if is_valid:
        logger.info("Hardware setup valid — all pages unlocked")
    else:
        logger.info("Hardware setup incomplete — pages locked")


def _update_page_gating(self, hardware_valid: bool):
    """Enable/disable navigation buttons for pages that require hardware setup."""
    # Page 0 (Hardware Setup) and Page 6 (Settings) are always available
    # Pages 1-5 require valid hardware config
    for i, btn in enumerate(self._menu_buttons):
        if i == 0 or i == 6:
            btn.setEnabled(True)
        else:
            btn.setEnabled(hardware_valid)
            if not hardware_valid:
                btn.setToolTip("Complete Hardware Setup first")
            else:
                btn.setToolTip("")


def _restore_hardware_config(self):
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


def _save_hardware_config(self, config):
    """Persist hardware config to settings.json."""
    try:
        self.settings.set("hardware_config", config.to_dict())
        self.settings.save()
    except Exception as e:
        logger.warning(f"Failed to save hardware config: {e}")


# ═══════════════════════════════════════════════════════════════════
# UPDATED NAVIGATION MAP (in _on_menu_click)
# ═══════════════════════════════════════════════════════════════════

# Update btn_map in _on_menu_click:
UPDATED_BTN_MAP = {
    "btn_hardware": 0,   # NEW
    "btn_dashboard": 1,  # was 0
    "btn_jog": 2,        # was 1
    "btn_calibrate": 3,  # was 2
    "btn_print": 4,      # was 3
    "btn_monitor": 5,    # was 4 (was 5 in v7.1)
    "btn_settings": 6,   # was 5 (was 4 in v7.1)
}


# ═══════════════════════════════════════════════════════════════════
# UPDATED MENU BUTTON CREATION (in _build_sidebar or equivalent)
# ═══════════════════════════════════════════════════════════════════

# Add this as the FIRST button in the sidebar:
# btn_hardware = self._make_menu_btn("btn_hardware", "🔧", "Hardware Setup")
# This goes BEFORE btn_dashboard in the sidebar layout.


# ═══════════════════════════════════════════════════════════════════
# UPDATED BOTTOM BAR — Pump positions in µL
# ═══════════════════════════════════════════════════════════════════

def _update_bottom_bar_pump_positions(self, zp_pos):
    """
    Update bottom bar pump readouts in µL instead of mm.

    Called from _update_status(). Replace the existing pump position
    display logic with this.
    """
    if not zp_pos:
        return

    for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
        pos_mm = zp_pos[idx] if idx < len(zp_pos) else None
        lbl = getattr(self, f"_bottom_{pid.lower()}_label", None)
        if lbl is None:
            continue

        if pos_mm is not None and self._hardware_config:
            pump_cfg = self._hardware_config.pumps.get(pid)
            if pump_cfg and pump_cfg.is_configured:
                try:
                    pos_uL = pump_cfg.mm_to_uL(pos_mm)
                    lbl.setText(f"{pid}: {pos_uL:.2f} µL")
                    continue
                except ValueError:
                    pass
        if pos_mm is not None:
            lbl.setText(f"{pid}: {pos_mm:.3f} mm")
        else:
            lbl.setText(f"{pid}: —")
