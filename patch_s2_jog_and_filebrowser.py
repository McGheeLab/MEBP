#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 2 Patch: Jog Step Display Fix + HW Setup File Browser

Issues addressed:
    #2  XY jog step size display does not match actual motion
    #3  Hardware Setup context panel needs a config file browser

Changes:
    gui/pages/jog_control.py     — Step verification display, conversion factor
                                   label, warning if factor is default
    gui/pages/hardware_setup.py  — Context panel with config file browser list,
                                   load/delete from list, auto-refresh on save

Depends on: Session 1 (styles)

Usage:
    python patch_s2_jog_and_filebrowser.py [project_root]
"""

import os
import sys
import re
from pathlib import Path

# ── Terminal colors ───────────────────────────────────────────────
BOLD = "\033[1m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

_applied = 0
_skipped = 0
_failed = 0


def find_project_root() -> Path:
    candidates = [Path("."), Path(".."),
                  Path("MEBP-Version-7.0"), Path("MEBP-Version-7.1"),
                  Path("MEBP-Version-7.2"), Path("MEBP-Version-7.2.3")]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{RED}ERROR{RESET}: Cannot find project root")
    sys.exit(1)


def read_file(path: Path) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path: Path, content: str):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def patch_replace(content: str, old: str, new: str, label: str, fpath: str = "") -> str:
    global _applied, _skipped, _failed
    if old in content:
        if new in content:
            print(f"  {YELLOW}SKIP{RESET}: {label} (already applied)")
            _skipped += 1
            return content
        result = content.replace(old, new, 1)
        print(f"  {GREEN}OK{RESET}:   {label}")
        _applied += 1
        return result
    else:
        print(f"  {RED}MISS{RESET}: {label} — old text not found in {fpath}")
        _failed += 1
        return content


def patch_insert_after(content: str, anchor: str, insertion: str, label: str, fpath: str = "") -> str:
    global _applied, _skipped, _failed
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {label} — anchor not found in {fpath}")
        _failed += 1
        return content
    if insertion.strip() in content:
        print(f"  {YELLOW}SKIP{RESET}: {label} (already applied)")
        _skipped += 1
        return content
    result = content.replace(anchor, anchor + insertion, 1)
    print(f"  {GREEN}OK{RESET}:   {label}")
    _applied += 1
    return result


# ═══════════════════════════════════════════════════════════════════
#  PATCH A: gui/pages/jog_control.py — Step verification + factor display
# ═══════════════════════════════════════════════════════════════════

def patch_jog_control(root: Path):
    filepath = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)
    fp = str(filepath)

    # ── A1: Add tracking attributes to __init__ ──────────────────
    old_init_end = (
        '        self._setup_ui()\n'
        '        self._setup_shortcuts()'
    )
    new_init_end = (
        '        # v7.2.4: Step verification tracking\n'
        '        self._last_jog_step_um: float = 0.0\n'
        '        self._last_xy_before: tuple = (None, None)\n'
        '        self._conversion_factor_set: bool = False  # True once protocol loads\n'
        '\n'
        '        self._setup_ui()\n'
        '        self._setup_shortcuts()'
    )
    content = patch_replace(content, old_init_end, new_init_end,
                            "A1: Add step verification tracking attrs to __init__", fp)

    # ── A2: Add conversion factor label + warning to context panel ─
    # Insert after the pump step combo row in the context panel
    old_ctx_speed = (
        '        # ── Speed Multipliers ────────────────────────────────\n'
        '        speed_label = QLabel("Speed (Xbox Jog)")'
    )
    new_ctx_speed = (
        '        # ── v7.2.4: Conversion Factor Display ──────────────\n'
        '        factor_label = QLabel("Unit Conversion")\n'
        '        factor_label.setObjectName("contextSectionLabel")\n'
        '        layout.addWidget(factor_label)\n'
        '\n'
        '        self._lbl_conversion_factor = QLabel(\n'
        '            f"Scale: {self._microsteps_per_micron:.1f} steps/µm")\n'
        '        self._lbl_conversion_factor.setStyleSheet(\n'
        '            f"color: {COLORS[\'subtext0\']}; font-size: 9pt;")\n'
        '        layout.addWidget(self._lbl_conversion_factor)\n'
        '\n'
        '        self._lbl_factor_warning = QLabel(\n'
        '            "⚠ Using default factor — verify in Settings")\n'
        '        self._lbl_factor_warning.setStyleSheet(\n'
        '            f"color: {COLORS[\'yellow\']}; font-size: 8pt;")\n'
        '        self._lbl_factor_warning.setWordWrap(True)\n'
        '        self._lbl_factor_warning.setVisible(not self._conversion_factor_set)\n'
        '        layout.addWidget(self._lbl_factor_warning)\n'
        '\n'
        '        # ── v7.2.4: Last Jog Step Verification ─────────────\n'
        '        self._lbl_last_jog = QLabel("")\n'
        '        self._lbl_last_jog.setStyleSheet(\n'
        '            f"color: {COLORS[\'subtext0\']}; font-size: 8pt;")\n'
        '        self._lbl_last_jog.setWordWrap(True)\n'
        '        layout.addWidget(self._lbl_last_jog)\n'
        '\n'
        '        # ── Speed Multipliers ────────────────────────────────\n'
        '        speed_label = QLabel("Speed (Xbox Jog)")'
    )
    content = patch_replace(content, old_ctx_speed, new_ctx_speed,
                            "A2: Add conversion factor label + warning + last jog label", fp)

    # ── A3: Update set_microsteps_per_micron to update UI ─────────
    old_set_um = (
        '    def set_microsteps_per_micron(self, value: float):\n'
        '        """Called by MainWindow when the conversion factor changes."""\n'
        '        self._microsteps_per_micron = max(0.001, value)'
    )
    new_set_um = (
        '    def set_microsteps_per_micron(self, value: float):\n'
        '        """Called by MainWindow when the conversion factor changes.\n'
        '\n'
        '        v7.2.4: Also updates the conversion factor display and hides\n'
        '        the warning banner once a real value is set.\n'
        '        """\n'
        '        self._microsteps_per_micron = max(0.001, value)\n'
        '        self._conversion_factor_set = True\n'
        '        # Update context panel labels if they exist\n'
        '        if hasattr(self, \'_lbl_conversion_factor\'):\n'
        '            self._lbl_conversion_factor.setText(\n'
        '                f"Scale: {self._microsteps_per_micron:.1f} steps/µm")\n'
        '        if hasattr(self, \'_lbl_factor_warning\'):\n'
        '            self._lbl_factor_warning.setVisible(False)\n'
        '        logger.info(f"Jog: microsteps_per_micron set to {value}")'
    )
    content = patch_replace(content, old_set_um, new_set_um,
                            "A3: Enhanced set_microsteps_per_micron with UI update", fp)

    # ── A4: Enhance _jog_xy to track step + record pre-jog position ─
    old_jog_body = (
        '        if not self.controller.is_xy_connected:\n'
        '            return\n'
        '\n'
        '        # Get step size in microns from combo box\n'
        '        step_um = self.xy_step_combo.currentData()\n'
        '        # Convert µm → microsteps\n'
        '        step_steps = step_um * self._microsteps_per_micron\n'
        '\n'
        '        # BUG-1 FIX: Use relative move — no dependency on cached position\n'
        '        self.controller.move_xy_relative(dx * step_steps, dy * step_steps)'
    )
    new_jog_body = (
        '        if not self.controller.is_xy_connected:\n'
        '            return\n'
        '\n'
        '        # Get step size in microns from combo box\n'
        '        step_um = self.xy_step_combo.currentData()\n'
        '        # Convert µm → microsteps\n'
        '        step_steps = step_um * self._microsteps_per_micron\n'
        '\n'
        '        # v7.2.4: Record pre-jog position for verification display\n'
        '        self._last_xy_before = self.controller.get_xy_position(cached=True)\n'
        '        self._last_jog_step_um = step_um\n'
        '\n'
        '        # BUG-1 FIX: Use relative move — no dependency on cached position\n'
        '        self.controller.move_xy_relative(dx * step_steps, dy * step_steps)\n'
        '\n'
        '        # v7.2.4: Update step verification display\n'
        '        direction = ""\n'
        '        if dx > 0: direction = "X+"\n'
        '        elif dx < 0: direction = "X−"\n'
        '        if dy > 0: direction += "Y+"\n'
        '        elif dy < 0: direction += "Y−"\n'
        '        cmd_steps = round(step_steps)\n'
        '        if hasattr(self, \'_lbl_last_jog\'):\n'
        '            self._lbl_last_jog.setText(\n'
        '                f"Last: {direction} {step_um:g} µm "\n'
        '                f"({cmd_steps} steps)")\n'
        '        logger.debug(f"Jog {direction}: {step_um:g} µm = "\n'
        '                     f"{cmd_steps} microsteps "\n'
        '                     f"(factor={self._microsteps_per_micron})")'
    )
    content = patch_replace(content, old_jog_body, new_jog_body,
                            "A4: Enhanced _jog_xy with step tracking + verification display", fp)

    # ── A5: Add step verification in _update_position ─────────────
    # After the position readout, show the measured delta vs requested
    old_update_end = (
        '        else:\n'
        '            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:\n'
        '                lbl.setText("—")'
    )
    new_update_end = (
        '        else:\n'
        '            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:\n'
        '                lbl.setText("—")\n'
        '\n'
        '        # v7.2.4: Step verification — show measured delta after jog\n'
        '        if (self._last_jog_step_um > 0 and\n'
        '                self._last_xy_before[0] is not None and\n'
        '                xy[0] is not None):\n'
        '            dx_steps = abs(xy[0] - self._last_xy_before[0]) + \\\n'
        '                       abs(xy[1] - self._last_xy_before[1])\n'
        '            dx_um = dx_steps / self._microsteps_per_micron\n'
        '            # Only show verification if stage has settled (delta > 0)\n'
        '            if dx_um > 0.01 and hasattr(self, \'_lbl_last_jog\'):\n'
        '                current_text = self._lbl_last_jog.text()\n'
        '                if "→" not in current_text:  # Don\'t keep appending\n'
        '                    self._lbl_last_jog.setText(\n'
        '                        f"{current_text}\\n→ Moved: {dx_um:.1f} µm")'
    )
    content = patch_replace(content, old_update_end, new_update_end,
                            "A5: Step verification delta display in _update_position", fp)

    # ── A6: Add set_hardware_config to jog page ──────────────────
    # The jog page should get the hardware config for future pump µL display
    if "def set_hardware_config(self, config" not in content:
        anchor = '    def on_status_update(self):'
        if anchor not in content:
            anchor = '    def _update_position(self):'
        if anchor in content:
            new_method = (
                '    def set_hardware_config(self, config):\n'
                '        """v7.2.4: Receive hardware config (for future pump µL display)."""\n'
                '        self._hardware_config = config\n'
                '\n'
            )
            content = content.replace(anchor, new_method + anchor, 1)
            print(f"  {GREEN}OK{RESET}:   A6: Added set_hardware_config to JogControlPage")
            global _applied
            _applied += 1
        else:
            print(f"  {RED}MISS{RESET}: A6: Could not find anchor for set_hardware_config")
    else:
        print(f"  {YELLOW}SKIP{RESET}: A6: set_hardware_config already exists")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH B: gui/pages/hardware_setup.py — Config file browser context panel
# ═══════════════════════════════════════════════════════════════════

def patch_hardware_setup(root: Path):
    filepath = root / "gui" / "pages" / "hardware_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)
    fp = str(filepath)

    # ── B1: Add QListWidget to imports if missing ─────────────────
    if "QListWidget" not in content:
        old_imports = "from PySide6.QtWidgets import ("
        # Find the full import block and add QListWidget
        import_match = re.search(
            r'from PySide6\.QtWidgets import \((.*?)\)',
            content, re.DOTALL
        )
        if import_match:
            old_block = import_match.group(0)
            if "QListWidget" not in old_block:
                # Insert QListWidget before the closing paren
                new_block = old_block.replace(
                    "QAbstractItemView,",
                    "QAbstractItemView, QListWidget, QListWidgetItem,",
                    1
                )
                if new_block == old_block:
                    # Try another insertion point
                    new_block = old_block.replace(
                        "QScrollArea,",
                        "QScrollArea, QListWidget, QListWidgetItem,",
                        1
                    )
                if new_block != old_block:
                    content = content.replace(old_block, new_block, 1)
                    print(f"  {GREEN}OK{RESET}:   B1: Added QListWidget to imports")
                else:
                    print(f"  {YELLOW}NOTE{RESET}: B1: Could not auto-add QListWidget import — add manually")
        else:
            print(f"  {YELLOW}NOTE{RESET}: B1: Import block pattern not found")

    # ── B2: Add config directory constant ─────────────────────────
    if "CONFIG_HARDWARE_DIR" not in content:
        anchor = "logger = logging.getLogger(__name__)"
        insertion = (
            '\n\n# v7.2.4: Default directory for hardware config files\n'
            'CONFIG_HARDWARE_DIR = Path(__file__).resolve().parent.parent.parent / "config" / "hardware"\n'
        )
        content = patch_insert_after(content, anchor, insertion,
                                     "B2: Added CONFIG_HARDWARE_DIR constant", fp)

    # ── B3: Replace get_context_widget to return a file browser ───
    old_ctx = (
        '    def get_context_widget(self) -> QWidget | None:\n'
        '        """Hardware Setup has no context panel."""\n'
        '        return self._context_widget'
    )
    new_ctx = (
        '    def get_context_widget(self) -> QWidget | None:\n'
        '        """v7.2.4: Context panel with saved config file browser."""\n'
        '        if self._context_widget is not None:\n'
        '            return self._context_widget\n'
        '\n'
        '        ctx = QWidget()\n'
        '        layout = QVBoxLayout(ctx)\n'
        '        layout.setContentsMargins(10, 8, 10, 8)\n'
        '        layout.setSpacing(6)\n'
        '\n'
        '        # ── Saved Configurations ────────────────────────────\n'
        '        configs_label = QLabel("Saved Configurations")\n'
        '        configs_label.setObjectName("contextSectionLabel")\n'
        '        layout.addWidget(configs_label)\n'
        '\n'
        '        self._config_list = QListWidget()\n'
        '        self._config_list.setAlternatingRowColors(True)\n'
        '        self._config_list.setMaximumHeight(260)\n'
        '        self._config_list.itemDoubleClicked.connect(\n'
        '            self._on_config_list_double_click)\n'
        '        self._config_list.currentItemChanged.connect(\n'
        '            self._on_config_list_selection)\n'
        '        layout.addWidget(self._config_list)\n'
        '\n'
        '        # Active config indicator\n'
        '        self._lbl_active_config = QLabel("Active: (unsaved)")\n'
        '        self._lbl_active_config.setStyleSheet(\n'
        '            f"color: {COLORS.get(\'subtext0\', \'#a6adc8\')}; "\n'
        '            f"font-size: 9pt; font-style: italic;")\n'
        '        layout.addWidget(self._lbl_active_config)\n'
        '\n'
        '        # Buttons row\n'
        '        btn_row = QHBoxLayout()\n'
        '        btn_load = QPushButton("Load")\n'
        '        btn_load.setMaximumHeight(26)\n'
        '        btn_load.clicked.connect(self._on_config_list_load)\n'
        '        btn_row.addWidget(btn_load)\n'
        '\n'
        '        btn_del = QPushButton("Delete")\n'
        '        btn_del.setMaximumHeight(26)\n'
        '        btn_del.setObjectName("dangerBtn")\n'
        '        btn_del.clicked.connect(self._on_config_list_delete)\n'
        '        btn_row.addWidget(btn_del)\n'
        '\n'
        '        btn_refresh = QPushButton("🔄")\n'
        '        btn_refresh.setMaximumHeight(26)\n'
        '        btn_refresh.setMaximumWidth(32)\n'
        '        btn_refresh.setToolTip("Refresh config file list")\n'
        '        btn_refresh.clicked.connect(self._scan_config_directory)\n'
        '        btn_row.addWidget(btn_refresh)\n'
        '        layout.addLayout(btn_row)\n'
        '\n'
        '        # ── Validity Status ─────────────────────────────────\n'
        '        status_label = QLabel("Setup Status")\n'
        '        status_label.setObjectName("contextSectionLabel")\n'
        '        layout.addWidget(status_label)\n'
        '\n'
        '        self._ctx_validity_label = QLabel("⚠ Setup incomplete")\n'
        '        self._ctx_validity_label.setStyleSheet(\n'
        '            f"color: {COLORS.get(\'yellow\', \'#f9e2af\')}; font-size: 9pt;")\n'
        '        self._ctx_validity_label.setWordWrap(True)\n'
        '        layout.addWidget(self._ctx_validity_label)\n'
        '\n'
        '        layout.addStretch()\n'
        '\n'
        '        self._context_widget = ctx\n'
        '\n'
        '        # Initial scan\n'
        '        self._scan_config_directory()\n'
        '\n'
        '        return ctx\n'
        '\n'
        '    # ════════════════════════════════════════════════════════════════\n'
        '    #  CONFIG FILE BROWSER (v7.2.4)\n'
        '    # ════════════════════════════════════════════════════════════════\n'
        '\n'
        '    def _scan_config_directory(self):\n'
        '        """Scan config/hardware/ for saved .json config files."""\n'
        '        if not hasattr(self, \'_config_list\'):\n'
        '            return\n'
        '        self._config_list.clear()\n'
        '        self._config_file_paths = {}  # name → path\n'
        '\n'
        '        config_dir = CONFIG_HARDWARE_DIR\n'
        '        if not config_dir.is_dir():\n'
        '            logger.warning(f"Config directory not found: {config_dir}")\n'
        '            return\n'
        '\n'
        '        json_files = sorted(config_dir.glob("*.json"))\n'
        '        for jf in json_files:\n'
        '            try:\n'
        '                import json\n'
        '                with open(jf, "r") as f:\n'
        '                    data = json.load(f)\n'
        '                cfg_name = data.get("config_name", jf.stem)\n'
        '                display = f"{cfg_name}  ({jf.name})"\n'
        '                self._config_list.addItem(display)\n'
        '                self._config_file_paths[display] = jf\n'
        '            except Exception as e:\n'
        '                logger.debug(f"Skipping {jf.name}: {e}")\n'
        '                self._config_list.addItem(f"⚠ {jf.name} (invalid)")\n'
        '\n'
        '        # Highlight active config if it matches\n'
        '        self._highlight_active_config()\n'
        '        logger.debug(f"Config browser: found {len(json_files)} files in {config_dir}")\n'
        '\n'
        '    def _highlight_active_config(self):\n'
        '        """Highlight the currently active config in the list."""\n'
        '        if not hasattr(self, \'_config_list\'):\n'
        '            return\n'
        '        active_name = self._config.config_name\n'
        '        for i in range(self._config_list.count()):\n'
        '            item = self._config_list.item(i)\n'
        '            text = item.text()\n'
        '            if text.startswith(active_name):\n'
        '                item.setSelected(True)\n'
        '                self._config_list.setCurrentItem(item)\n'
        '                break\n'
        '        if hasattr(self, \'_lbl_active_config\'):\n'
        '            self._lbl_active_config.setText(f"Active: {active_name}")\n'
        '\n'
        '    def _on_config_list_selection(self, current, previous):\n'
        '        """Handle selection change in config list."""\n'
        '        pass  # Selection visual is handled by QListWidget\n'
        '\n'
        '    def _on_config_list_double_click(self, item):\n'
        '        """Double-click loads the config."""\n'
        '        self._on_config_list_load()\n'
        '\n'
        '    def _on_config_list_load(self):\n'
        '        """Load the selected config from the file browser."""\n'
        '        if not hasattr(self, \'_config_list\'):\n'
        '            return\n'
        '        current = self._config_list.currentItem()\n'
        '        if current is None:\n'
        '            return\n'
        '        display = current.text()\n'
        '        path = self._config_file_paths.get(display)\n'
        '        if path and path.exists():\n'
        '            try:\n'
        '                self._config = HardwareConfig.load(str(path))\n'
        '                self._apply_config_to_ui()\n'
        '                self._highlight_active_config()\n'
        '                logger.info(f"Loaded config from browser: {path.name}")\n'
        '            except Exception as e:\n'
        '                QMessageBox.critical(\n'
        '                    self, "Load Error", f"Failed to load:\\n{e}")\n'
        '\n'
        '    def _on_config_list_delete(self):\n'
        '        """Delete the selected config file (with confirmation)."""\n'
        '        if not hasattr(self, \'_config_list\'):\n'
        '            return\n'
        '        current = self._config_list.currentItem()\n'
        '        if current is None:\n'
        '            return\n'
        '        display = current.text()\n'
        '        path = self._config_file_paths.get(display)\n'
        '        if not path or not path.exists():\n'
        '            return\n'
        '        reply = QMessageBox.question(\n'
        '            self, "Delete Config",\n'
        '            f"Delete config file?\\n\\n{path.name}\\n\\n"\n'
        '            f"This cannot be undone.",\n'
        '            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)\n'
        '        if reply == QMessageBox.Yes:\n'
        '            try:\n'
        '                path.unlink()\n'
        '                self._scan_config_directory()\n'
        '                logger.info(f"Deleted config file: {path.name}")\n'
        '            except Exception as e:\n'
        '                QMessageBox.critical(\n'
        '                    self, "Delete Error", f"Failed to delete:\\n{e}")'
    )
    content = patch_replace(content, old_ctx, new_ctx,
                            "B3: Replaced get_context_widget with file browser", fp)

    # ── B4: Wire _save_config to auto-refresh the file list ───────
    old_save_success = (
        '                QMessageBox.information(\n'
        '                    self, "Saved", f"Configuration saved to:\\n{path}")'
    )
    new_save_success = (
        '                QMessageBox.information(\n'
        '                    self, "Saved", f"Configuration saved to:\\n{path}")\n'
        '                # v7.2.4: Refresh the config file browser\n'
        '                self._scan_config_directory()'
    )
    content = patch_replace(content, old_save_success, new_save_success,
                            "B4: Auto-refresh config list after save", fp)

    # ── B5: Wire _load_config to refresh the file list + highlight ─
    old_load_success = (
        '                QMessageBox.information(\n'
        '                    self, "Loaded", f"Configuration loaded from:\\n{path}")'
    )
    new_load_success = (
        '                QMessageBox.information(\n'
        '                    self, "Loaded", f"Configuration loaded from:\\n{path}")\n'
        '                # v7.2.4: Refresh and highlight in file browser\n'
        '                self._scan_config_directory()'
    )
    content = patch_replace(content, old_load_success, new_load_success,
                            "B5: Auto-refresh + highlight after file dialog load", fp)

    # ── B6: Wire _on_config_changed to update context validity ────
    old_valid_green = (
        '        if valid:\n'
        '            self.validity_label.setText("✓ Setup complete")\n'
        '            self.validity_label.setStyleSheet(\n'
        '                f"color: {COLORS.get(\'green\', \'#a6e3a1\')};")'
    )
    new_valid_green = (
        '        if valid:\n'
        '            self.validity_label.setText("✓ Setup complete")\n'
        '            self.validity_label.setStyleSheet(\n'
        '                f"color: {COLORS.get(\'green\', \'#a6e3a1\')};")\n'
        '            # v7.2.4: Also update context panel validity\n'
        '            if hasattr(self, \'_ctx_validity_label\'):\n'
        '                self._ctx_validity_label.setText("✓ Setup complete")\n'
        '                self._ctx_validity_label.setStyleSheet(\n'
        '                    f"color: {COLORS.get(\'green\', \'#a6e3a1\')};")'
    )
    content = patch_replace(content, old_valid_green, new_valid_green,
                            "B6a: Update context validity label (valid)", fp)

    old_valid_yellow = (
        '        else:\n'
        '            _, issues = self._config.validate()\n'
        '            self.validity_label.setText(\n'
        '                f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")\n'
        '            self.validity_label.setStyleSheet(\n'
        '                f"color: {COLORS.get(\'yellow\', \'#f9e2af\')};")'
    )
    new_valid_yellow = (
        '        else:\n'
        '            _, issues = self._config.validate()\n'
        '            self.validity_label.setText(\n'
        '                f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")\n'
        '            self.validity_label.setStyleSheet(\n'
        '                f"color: {COLORS.get(\'yellow\', \'#f9e2af\')};")\n'
        '            # v7.2.4: Also update context panel validity\n'
        '            if hasattr(self, \'_ctx_validity_label\'):\n'
        '                issue_text = issues[0] if issues else "Setup incomplete"\n'
        '                self._ctx_validity_label.setText(f"⚠ {issue_text}")\n'
        '                self._ctx_validity_label.setStyleSheet(\n'
        '                    f"color: {COLORS.get(\'yellow\', \'#f9e2af\')};")'
    )
    content = patch_replace(content, old_valid_yellow, new_valid_yellow,
                            "B6b: Update context validity label (invalid)", fp)

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'═' * 60}")
    print(f" MEBP v7.2.4 — Session 2 Patch")
    print(f" Jog Step Fix + Hardware Setup File Browser")
    print(f"{'═' * 60}{RESET}")
    print(f"Project root: {root}")

    for check_dir in ["gui/pages", "gui", "SupportClasses"]:
        if not (root / check_dir).is_dir():
            print(f"\n{RED}ERROR{RESET}: Expected directory not found: {check_dir}")
            sys.exit(1)

    # Ensure config/hardware/ exists
    config_hw_dir = root / "config" / "hardware"
    if not config_hw_dir.is_dir():
        config_hw_dir.mkdir(parents=True, exist_ok=True)
        print(f"  {CYAN}INFO{RESET}: Created {config_hw_dir}")

    patch_jog_control(root)       # A: Jog step verification
    patch_hardware_setup(root)    # B: Config file browser

    # Summary
    print(f"\n{'═' * 60}")
    print(f"{BOLD}Session 2 Patch Summary{RESET}")
    print(f"{'═' * 60}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped} (already applied)")
    print(f"  {RED}Failed{RESET}:   {_failed}")

    if _failed > 0:
        print(f"\n{YELLOW}WARNING{RESET}: {_failed} patches could not be applied.")
        print(f"Review the MISS messages above and apply manually.")

    print(f"\n{BOLD}Files modified:{RESET}")
    print(f"  gui/pages/jog_control.py    — Step verification + factor display + warning")
    print(f"  gui/pages/hardware_setup.py — Config file browser context panel")

    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
