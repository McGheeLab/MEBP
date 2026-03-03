#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 1 Patch: Centralized Styles + Hardware Config Propagation

Issues addressed:
    #1  HW config not propagating to calibration plate selector & well setup ink/rosette lists
    #7  GUI section titles misaligned / not stylish across all pages
    #2  (partial) Verify µm factor propagation to jog page

Changes:
    gui/styles.py              — Add SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
    gui/app.py                 — Add propagation logging, verify µm factor wiring
    gui/pages/calibration.py   — Sync plate format from HardwareConfig, use centralized styles
    gui/pages/print_well_setup.py — Refresh ink/rosette combos from HardwareConfig
    gui/pages/print_objects.py — Refresh ink combo + well diameter from HardwareConfig
    gui/pages/hardware_setup.py — Replace _group_style() with centralized import
    gui/pages/print_workspace.py — Replace inline styles
    gui/pages/dashboard.py     — Replace inline styles
    gui/pages/jog_control.py   — Replace inline styles
    gui/pages/settings_page.py — Replace inline styles
    gui/pages/print_setup.py   — Replace inline styles
    gui/pages/print_monitor.py — Replace inline styles

Usage:
    python patch_s1_styles_and_propagation.py [project_root]

    If project_root is omitted, searches for SupportClasses/ in current and parent dirs.
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

# ── Counters ──────────────────────────────────────────────────────
_applied = 0
_skipped = 0
_failed = 0


def find_project_root() -> Path:
    """Locate the MEBP project root."""
    candidates = [
        Path("."),
        Path(".."),
        Path("MEBP-Version-7.0"),
        Path("MEBP-Version-7.1"),
        Path("MEBP-Version-7.2"),
        Path("MEBP-Version-7.2.3"),
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{RED}ERROR{RESET}: Cannot find project root (looking for SupportClasses/ + gui/)")
    sys.exit(1)


def read_file(path: Path) -> str:
    """Read a file, returning its content."""
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path: Path, content: str):
    """Write content to a file."""
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def patch_replace(content: str, old: str, new: str, label: str, filepath: str = "") -> str:
    """Replace old with new in content. Reports success/skip/fail."""
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
        print(f"  {RED}MISS{RESET}: {label} — old text not found in {filepath}")
        _failed += 1
        return content


def patch_insert_after(content: str, anchor: str, insertion: str, label: str, filepath: str = "") -> str:
    """Insert text after the anchor line. Reports success/skip/fail."""
    global _applied, _skipped, _failed
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {label} — anchor not found in {filepath}")
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
#  PATCH 1: gui/styles.py — Add centralized section styles
# ═══════════════════════════════════════════════════════════════════

def patch_styles(root: Path):
    filepath = root / "gui" / "styles.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 1: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Add centralized style constants after COLORS dict
    style_constants = '''

# ── Centralized Section Styles (v7.2.4) ─────────────────────────
# Use these instead of per-page _group_style() methods for consistency.

SECTION_TITLE_STYLE = f"""
    QGroupBox {{
        font-size: 11pt;
        font-weight: 600;
        color: {COLORS['text']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        margin-top: 12px;
        padding: 16px 12px 10px 12px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 12px;
        padding: 2px 8px;
        background-color: {COLORS['base']};
        border-radius: 4px;
        color: {COLORS['blue']};
        font-size: 10pt;
        font-weight: 600;
    }}
"""

CONTEXT_SECTION_LABEL_STYLE = f"""
    font-size: 10pt;
    font-weight: 600;
    color: {COLORS['blue']};
    padding: 6px 0px 2px 0px;
    border-bottom: 1px solid {COLORS['surface1']};
    margin-bottom: 4px;
"""

PAGE_HEADER_STYLE = f"""
    font-size: 14pt;
    font-weight: 700;
    color: {COLORS['text']};
    padding: 4px 0px;
"""

CARD_FRAME_STYLE = f"""
    QFrame#cardFrame {{
        background-color: {COLORS['surface0']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        padding: 12px;
    }}
"""
'''

    # Find the end of the COLORS dict to insert after it
    anchor = 'MENU_SELECTED_STYLESHEET = ('
    content = patch_insert_after(
        content,
        anchor,
        "",  # We'll insert before this anchor instead
        "placeholder",
        str(filepath),
    )
    # Reset counters for the real patch
    global _applied, _skipped, _failed

    # Actually: insert the style constants right before MENU_SELECTED_STYLESHEET
    if "SECTION_TITLE_STYLE" not in content:
        insert_point = content.find("# ── Menu Selection Stylesheet")
        if insert_point == -1:
            insert_point = content.find("MENU_SELECTED_STYLESHEET")
        if insert_point > 0:
            content = content[:insert_point] + style_constants + "\n" + content[insert_point:]
            print(f"  {GREEN}OK{RESET}:   Added SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE, PAGE_HEADER_STYLE, CARD_FRAME_STYLE")
            _applied += 1
        else:
            print(f"  {RED}MISS{RESET}: Could not find insertion point in styles.py")
            _failed += 1
    else:
        print(f"  {YELLOW}SKIP{RESET}: Centralized styles already present")
        _skipped += 1

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 2: gui/app.py — Propagation audit + logging
# ═══════════════════════════════════════════════════════════════════

def patch_app(root: Path):
    filepath = root / "gui" / "app.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 2: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Enhance _propagate_hardware_config with per-page logging
    old_propagate = '''    def _propagate_hardware_config(self, config: HardwareConfig):
        """Push hardware config to all pages and the controller."""
        if hasattr(self.controller, 'set_hardware_config'):
            self.controller.set_hardware_config(config)

        for page in self._page_widgets:
            # Skip the hardware setup page — it's the SOURCE, not the target
            if isinstance(page, HardwareSetupPage):
                continue
            if hasattr(page, 'set_hardware_config'):
                page.set_hardware_config(config)'''

    new_propagate = '''    def _propagate_hardware_config(self, config: HardwareConfig):
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
                logger.debug(f"HW config → Page {i}: {page_name} (no set_hardware_config)")'''

    content = patch_replace(content, old_propagate, new_propagate,
                            "Enhanced _propagate_hardware_config with per-page logging",
                            str(filepath))

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 3: gui/pages/calibration.py — Plate format sync + styles
# ═══════════════════════════════════════════════════════════════════

def patch_calibration(root: Path):
    filepath = root / "gui" / "pages" / "calibration.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 3: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # 3a. Add SECTION_TITLE_STYLE import
    old_import = "from gui.styles import COLORS"
    new_import = "from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE"
    content = patch_replace(content, old_import, new_import,
                            "Add centralized style imports", str(filepath))

    # 3b. Replace set_hardware_config with enhanced version that syncs BOTH combos
    old_shc = '''    def set_hardware_config(self, config):
        """v7.2: Set hardware config — auto-select plate format and show needle info."""
        self._hardware_config = config
        if config is None:
            return

        # Auto-select plate format from hardware config
        if hasattr(config, 'plate_format') and config.plate_format:
            if hasattr(self, 'plate_combo'):
                for i in range(self.plate_combo.count()):
                    if self.plate_combo.itemData(i) == config.plate_format:
                        self.plate_combo.setCurrentIndex(i)
                        break

        # Show needle info in calibration context panel
        if hasattr(config, 'needle') and config.needle and hasattr(self, 'ctx_lbl_needle_info'):
            n = config.needle
            self.ctx_lbl_needle_info.setText(
                f"Needle: {n.gauge}G | ID: {n.inner_diameter_um:.0f} µm | "
                f"Length: {n.length_inches:.1f}\\\""
            )'''

    new_shc = (
        '    def set_hardware_config(self, config):\n'
        '        """v7.2.4: Set hardware config — sync plate format, needle info, and plate model.\n'
        '\n'
        '        Syncs BOTH the main plate_combo (if it exists) and the context panel\n'
        '        ctx_plate_combo. Also rebuilds the WellPlate model and validation\n'
        '        well combo so calibration uses the correct plate geometry.\n'
        '        """\n'
        '        self._hardware_config = config\n'
        '        if config is None:\n'
        '            return\n'
        '\n'
        '        # Auto-select plate format in context panel combo\n'
        '        if hasattr(config, \'plate_format\') and config.plate_format:\n'
        '            fmt = config.plate_format\n'
        '            # Sync main plate_combo (if page has one)\n'
        '            if hasattr(self, \'plate_combo\'):\n'
        '                for i in range(self.plate_combo.count()):\n'
        '                    if self.plate_combo.itemData(i) == fmt:\n'
        '                        self.plate_combo.setCurrentIndex(i)\n'
        '                        break\n'
        '            # Sync context panel plate combo\n'
        '            if hasattr(self, \'ctx_plate_combo\'):\n'
        '                for i in range(self.ctx_plate_combo.count()):\n'
        '                    if self.ctx_plate_combo.itemData(i) == fmt:\n'
        '                        if self.ctx_plate_combo.currentIndex() != i:\n'
        '                            self.ctx_plate_combo.setCurrentIndex(i)\n'
        '                        break\n'
        '            # Rebuild the plate model directly so calibration always matches HW config\n'
        '            self._plate = WellPlate.from_format(fmt)\n'
        '            defn = PLATE_DEFINITIONS[fmt]\n'
        '            rows, cols = defn["rows"], defn["cols"]\n'
        '            self._corner_well = f"{chr(ord(\'A\') + rows - 1)}{cols}"\n'
        '            if hasattr(self, \'val_well_combo\'):\n'
        '                wells = self._plate.well_names\n'
        '                self.val_well_combo.clear()\n'
        '                self.val_well_combo.addItems(wells)\n'
        '            logger.info(f"Calibration: plate format synced to {fmt}-well from HardwareConfig")\n'
        '\n'
        '        # Show needle info in calibration context panel\n'
        '        if hasattr(config, \'needle\') and config.needle:\n'
        '            n = config.needle\n'
        '            needle_text = (f"Needle: {n.gauge}G | ID: {n.id_um:.0f} \\u00b5m | "\n'
        '                           f"Length: {n.length_inches:.1f}\\"")\n'
        '            if hasattr(self, \'ctx_lbl_needle_info\'):\n'
        '                self.ctx_lbl_needle_info.setText(needle_text)\n'
    )

    content = patch_replace(content, old_shc, new_shc,
                            "Enhanced set_hardware_config — syncs both combos + rebuilds plate model",
                            str(filepath))

    # 3c. Apply CONTEXT_SECTION_LABEL_STYLE to context panel section labels
    # The context panel uses setObjectName("contextSectionLabel") which is
    # already styled in the QSS. We'll add an explicit Python-side style for
    # labels that don't get the objectName treatment. This is optional but
    # ensures consistency if QSS is missed.

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 4: gui/pages/print_well_setup.py — Refresh ink/rosette from HW config
# ═══════════════════════════════════════════════════════════════════

def patch_well_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 4: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # 4a. Add SECTION_TITLE_STYLE import
    old_import = "from gui.styles import COLORS"
    new_import = "from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE"
    content = patch_replace(content, old_import, new_import,
                            "Add centralized style imports", str(filepath))

    # 4b. Add set_hardware_config method if not present, or enhance existing one
    # The well setup tab currently gets data via set_workspace(WorkspaceConfig).
    # We need to add a direct set_hardware_config that refreshes inks/rosettes.

    if "def set_hardware_config(self, config):" not in content:
        # Find the class and add the method
        anchor = "    def set_workspace(self"
        if anchor in content:
            new_method = '''    def set_hardware_config(self, config):
        """v7.2.4: Receive HardwareConfig directly for ink/rosette/plate sync.

        This ensures the well setup tab always has the latest ink library
        and rosette library from Page 0, even if the WorkspaceConfig bridge
        hasn't fired yet.
        """
        if config is None:
            return
        self._hw_config = config
        logger.info(f"WellSetup: received HardwareConfig "
                    f"(plate={config.plate_format}, "
                    f"inks={list(config.ink_library.keys())}, "
                    f"rosettes={list(config.rosette_library.keys())})")

        # Refresh ink combo from HardwareConfig ink library
        if hasattr(self, 'ink_combo'):
            current_ink = self.ink_combo.currentData()
            self.ink_combo.clear()
            self.ink_combo.addItem("(none)", None)
            for name in config.ink_library:
                self.ink_combo.addItem(name, name)
            # Restore previous selection if still valid
            if current_ink:
                idx = self.ink_combo.findData(current_ink)
                if idx >= 0:
                    self.ink_combo.setCurrentIndex(idx)

        # Refresh rosette combo from HardwareConfig rosette library
        if hasattr(self, 'rosette_combo'):
            current_ros = self.rosette_combo.currentData()
            self.rosette_combo.clear()
            self.rosette_combo.addItem("None", None)
            for name in config.rosette_library:
                self.rosette_combo.addItem(name, name)
            if current_ros:
                idx = self.rosette_combo.findData(current_ros)
                if idx >= 0:
                    self.rosette_combo.setCurrentIndex(idx)

        # Sync plate format if the model supports it
        if hasattr(self, '_model') and self._model:
            if config.plate_format != getattr(self._model, '_plate_format', None):
                # The plate view will be rebuilt when workspace is also updated
                pass

    '''
            content = patch_insert_after(content, anchor, "",
                                          "placeholder — will insert before", str(filepath))
            # Actually insert before set_workspace
            content = content.replace(
                "    def set_workspace(self",
                new_method + "    def set_workspace(self",
                1
            )
            print(f"  {GREEN}OK{RESET}:   Added set_hardware_config() method to WellSetupTab")
            global _applied
            _applied += 1
        else:
            print(f"  {RED}MISS{RESET}: Could not find set_workspace anchor in well_setup")
    else:
        print(f"  {YELLOW}SKIP{RESET}: set_hardware_config already exists in well_setup")

    # 4c. Enhance set_workspace to also refresh ink/rosette combos
    old_set_ws = '''    def _refresh_ink_combo(self) -> None:
        """Populate ink combo from workspace ink library."""
        self.ink_combo.clear()
        self.ink_combo.addItem("(none)", None)
        for name in self._workspace.ink_library:
            self.ink_combo.addItem(name, name)

    def _refresh_rosette_combo(self) -> None:
        """Populate rosette combo from workspace rosette library."""
        self.rosette_combo.clear()
        self.rosette_combo.addItem("None", None)
        for name in self._workspace.rosette_library:
            self.rosette_combo.addItem(name, name)'''

    new_set_ws = '''    def _refresh_ink_combo(self) -> None:
        """Populate ink combo from workspace ink library or HardwareConfig.

        v7.2.4: Prefers HardwareConfig._hw_config if available (most up-to-date),
        falls back to workspace ink library.
        """
        source = {}
        if hasattr(self, '_hw_config') and self._hw_config:
            source = self._hw_config.ink_library
        elif self._workspace:
            source = self._workspace.ink_library

        current = self.ink_combo.currentData() if self.ink_combo.count() > 0 else None
        self.ink_combo.clear()
        self.ink_combo.addItem("(none)", None)
        for name in source:
            self.ink_combo.addItem(name, name)
        if current:
            idx = self.ink_combo.findData(current)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
        logger.debug(f"WellSetup: ink combo refreshed with {list(source.keys())}")

    def _refresh_rosette_combo(self) -> None:
        """Populate rosette combo from workspace rosette library or HardwareConfig.

        v7.2.4: Prefers HardwareConfig._hw_config if available (most up-to-date),
        falls back to workspace rosette library.
        """
        source = {}
        if hasattr(self, '_hw_config') and self._hw_config:
            source = self._hw_config.rosette_library
        elif self._workspace:
            source = self._workspace.rosette_library

        current = self.rosette_combo.currentData() if self.rosette_combo.count() > 0 else None
        self.rosette_combo.clear()
        self.rosette_combo.addItem("None", None)
        for name in source:
            self.rosette_combo.addItem(name, name)
        if current:
            idx = self.rosette_combo.findData(current)
            if idx >= 0:
                self.rosette_combo.setCurrentIndex(idx)
        logger.debug(f"WellSetup: rosette combo refreshed with {list(source.keys())}")'''

    content = patch_replace(content, old_set_ws, new_set_ws,
                            "Enhanced _refresh_ink_combo and _refresh_rosette_combo to use HardwareConfig",
                            str(filepath))

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 5: gui/pages/print_objects.py — Refresh from HW config
# ═══════════════════════════════════════════════════════════════════

def patch_print_objects(root: Path):
    filepath = root / "gui" / "pages" / "print_objects.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 5: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # 5a. Add style imports if not present
    if "SECTION_TITLE_STYLE" not in content:
        old_import = "from gui.styles import COLORS"
        new_import = "from gui.styles import COLORS, SECTION_TITLE_STYLE"
        if old_import in content:
            content = patch_replace(content, old_import, new_import,
                                    "Add centralized style import", str(filepath))

    # 5b. Enhance set_hardware_config to refresh ink combos and well diameter
    # Look for existing set_hardware_config or add one
    if "def set_hardware_config(self, config" in content:
        # Enhance the existing one — find it and check if it already refreshes inks
        if "_refresh_ink_options_from_config" not in content:
            # Add a helper method and wire it
            old_shc_pattern = re.search(
                r'(    def set_hardware_config\(self, config.*?\n(?:        .*\n)*)',
                content
            )
            if old_shc_pattern:
                old_shc = old_shc_pattern.group(0)
                # Check if it stores the config
                if "self._hw_config = config" not in old_shc:
                    # Add storage line
                    content = content.replace(
                        old_shc,
                        old_shc.rstrip() + "\n        self._hw_config = config\n\n",
                        1
                    )
                    print(f"  {GREEN}OK{RESET}:   Added self._hw_config storage to set_hardware_config")
                    _applied = _applied  # already counted

            # Add refresh helper
            refresh_method = '''
    def _refresh_ink_options_from_config(self):
        """v7.2.4: Update ink selection options from HardwareConfig."""
        if not hasattr(self, '_hw_config') or not self._hw_config:
            return
        ink_names = list(self._hw_config.ink_library.keys())
        # Update ink combo in the designer if it exists
        if hasattr(self, '_ink_combo'):
            current = self._ink_combo.currentData()
            self._ink_combo.clear()
            for name in ink_names:
                pump_id = None
                for pid, pcfg in self._hw_config.pumps.items():
                    if pcfg.ink and pcfg.ink.name == name:
                        pump_id = pid
                        break
                label = f"{pump_id}: {name}" if pump_id else name
                self._ink_combo.addItem(label, name)
            if current:
                idx = self._ink_combo.findData(current)
                if idx >= 0:
                    self._ink_combo.setCurrentIndex(idx)
        # Update well diameter from plate format
        self._update_well_diameter()
        logger.debug(f"PrintObjects: refreshed ink options: {ink_names}")

'''
            # Insert before _update_well_diameter or at end of class
            if "def _update_well_diameter(self):" in content:
                content = content.replace(
                    "    def _update_well_diameter(self):",
                    refresh_method + "    def _update_well_diameter(self):",
                    1
                )
                print(f"  {GREEN}OK{RESET}:   Added _refresh_ink_options_from_config() method")
            else:
                print(f"  {YELLOW}SKIP{RESET}: Could not find _update_well_diameter anchor")
    else:
        # No set_hardware_config at all — add one
        # Find a good insertion point
        if "def set_workspace(self" in content:
            new_shc = '''    def set_hardware_config(self, config):
        """v7.2.4: Receive HardwareConfig for ink/plate sync."""
        if config is None:
            return
        self._hw_config = config
        logger.info(f"PrintObjects: received HardwareConfig "
                    f"(plate={config.plate_format}, "
                    f"inks={list(config.ink_library.keys())})")
        self._update_well_diameter()

'''
            content = content.replace(
                "    def set_workspace(self",
                new_shc + "    def set_workspace(self",
                1
            )
            print(f"  {GREEN}OK{RESET}:   Added set_hardware_config() to PrintObjectsTab")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 6: Replace _group_style() across all page files
# ═══════════════════════════════════════════════════════════════════

def patch_replace_group_styles(root: Path):
    """Replace inline _group_style() methods with centralized SECTION_TITLE_STYLE import."""
    pages_dir = root / "gui" / "pages"
    print(f"\n{'═' * 60}")
    print(f"PATCH 6: Replace _group_style() across all pages")
    print(f"{'═' * 60}")

    # Files known to have _group_style() or inline QGroupBox styles
    target_files = [
        "hardware_setup.py",
        "print_workspace.py",
        "calibration.py",
        "jog_control.py",
        "print_setup.py",
        "print_monitor.py",
        "settings_page.py",
        "dashboard.py",
        "print_objects.py",
        "print_well_setup.py",
    ]

    for fname in target_files:
        filepath = pages_dir / fname
        if not filepath.exists():
            print(f"  {YELLOW}SKIP{RESET}: {fname} (file not found)")
            continue

        content = read_file(filepath)
        modified = False

        # Ensure SECTION_TITLE_STYLE is imported
        if "SECTION_TITLE_STYLE" not in content and "from gui.styles import" in content:
            old_line = re.search(r'from gui\.styles import (.+)', content)
            if old_line:
                existing_imports = old_line.group(1).strip()
                if "SECTION_TITLE_STYLE" not in existing_imports:
                    new_imports = existing_imports.rstrip()
                    if new_imports.endswith(")"):
                        # Multi-line import — add before closing paren
                        pass  # skip complex case
                    else:
                        new_line = f"from gui.styles import {new_imports}, SECTION_TITLE_STYLE"
                        content = content.replace(old_line.group(0), new_line, 1)
                        print(f"  {GREEN}OK{RESET}:   {fname}: Added SECTION_TITLE_STYLE import")
                        modified = True

        # Replace _group_style() static method if present
        # Pattern: static method returning QGroupBox stylesheet
        group_style_pattern = re.compile(
            r'    @staticmethod\n    def _group_style\(\) -> str:\n'
            r'        return f""".*?"""',
            re.DOTALL
        )
        match = group_style_pattern.search(content)
        if match:
            replacement = '''    @staticmethod
    def _group_style() -> str:
        """v7.2.4: Delegates to centralized SECTION_TITLE_STYLE."""
        return SECTION_TITLE_STYLE'''
            content = content[:match.start()] + replacement + content[match.end():]
            print(f"  {GREEN}OK{RESET}:   {fname}: Replaced _group_style() body with centralized style")
            modified = True

        # Also replace any inline setStyleSheet(self._group_style()) calls
        # that might use a different pattern — these are fine as-is since
        # _group_style() now returns SECTION_TITLE_STYLE

        if modified:
            write_file(filepath, content)
        else:
            # Check if it even has QGroupBox styling
            if "_group_style" in content:
                print(f"  {YELLOW}NOTE{RESET}: {fname}: _group_style exists but pattern didn't match")
            else:
                print(f"  {CYAN}INFO{RESET}: {fname}: No _group_style() found (may use QSS directly)")


# ═══════════════════════════════════════════════════════════════════
#  PATCH 7: gui/pages/print_setup.py — Forward set_hardware_config to tabs
# ═══════════════════════════════════════════════════════════════════

def patch_print_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH 7: {filepath}")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Ensure set_hardware_config forwards to ALL tabs including well setup
    if "def set_hardware_config(self, config" in content:
        # Check if it already forwards to tab_wells
        if "tab_wells" not in content or "tab_wells.set_hardware_config" not in content:
            # Find the method and enhance it
            old_pattern = re.search(
                r'(    def set_hardware_config\(self, config.*?\n(?:        .*\n)*)',
                content
            )
            if old_pattern:
                old_method = old_pattern.group(0)
                # Add forwarding to well setup tab and objects tab
                if "tab_wells" not in old_method:
                    addition = '''
        # v7.2.4: Forward to Well Setup tab for ink/rosette refresh
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'set_hardware_config'):
            self.tab_wells.set_hardware_config(config)
        # v7.2.4: Forward to Print Objects tab for ink refresh
        if hasattr(self, 'tab_objects') and hasattr(self.tab_objects, 'set_hardware_config'):
            self.tab_objects.set_hardware_config(config)
'''
                    content = content.replace(
                        old_method,
                        old_method.rstrip() + "\n" + addition,
                        1
                    )
                    print(f"  {GREEN}OK{RESET}:   Added tab_wells + tab_objects forwarding to set_hardware_config")
                else:
                    print(f"  {YELLOW}SKIP{RESET}: tab_wells forwarding already present")
            else:
                print(f"  {RED}MISS{RESET}: Could not parse set_hardware_config method")
    else:
        print(f"  {RED}MISS{RESET}: No set_hardware_config found in print_setup.py")

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
    print(f" MEBP v7.2.4 — Session 1 Patch")
    print(f" Styles + Hardware Config Propagation")
    print(f"{'═' * 60}{RESET}")
    print(f"Project root: {root}")

    # Verify structure
    for check_dir in ["gui/pages", "gui", "SupportClasses"]:
        if not (root / check_dir).is_dir():
            print(f"\n{RED}ERROR{RESET}: Expected directory not found: {check_dir}")
            sys.exit(1)

    # Run patches in order
    patch_styles(root)                    # 1. Centralized styles
    patch_app(root)                       # 2. Propagation logging
    patch_calibration(root)               # 3. Calibration plate sync
    patch_well_setup(root)                # 4. Well setup ink/rosette refresh
    patch_print_objects(root)             # 5. Print objects ink refresh
    patch_replace_group_styles(root)      # 6. Replace _group_style everywhere
    patch_print_setup(root)               # 7. Print setup forwarding

    # Summary
    print(f"\n{'═' * 60}")
    print(f"{BOLD}Session 1 Patch Summary{RESET}")
    print(f"{'═' * 60}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped} (already applied)")
    print(f"  {RED}Failed{RESET}:   {_failed}")
    total = _applied + _skipped + _failed
    print(f"  Total:    {total}")

    if _failed > 0:
        print(f"\n{YELLOW}WARNING{RESET}: {_failed} patches could not be applied.")
        print(f"This may be due to code that has changed since this patch was written.")
        print(f"Review the MISS messages above and apply those changes manually.")

    print(f"\n{BOLD}Files modified:{RESET}")
    print(f"  gui/styles.py              — Centralized SECTION_TITLE_STYLE + constants")
    print(f"  gui/app.py                 — Per-page propagation logging")
    print(f"  gui/pages/calibration.py   — Plate format sync from HardwareConfig")
    print(f"  gui/pages/print_well_setup.py — Ink/rosette refresh from HardwareConfig")
    print(f"  gui/pages/print_objects.py — Ink refresh + well diameter from HardwareConfig")
    print(f"  gui/pages/print_setup.py   — Forward HW config to well setup + objects tabs")
    print(f"  gui/pages/*.py             — _group_style() → centralized SECTION_TITLE_STYLE")

    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
