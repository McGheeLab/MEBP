#!/usr/bin/env python3
"""
apply_um_patches.py — Apply microsteps-to-microns (µm) conversion patches.

Run from the MEBP root directory:
    python apply_um_patches.py

This script applies targeted patches to:
    1. gui/pages/calibration.py   — position display + taught points in µm
    2. gui/pages/settings_page.py — µsteps/µm spinbox + safety limits in µm
    3. gui/pages/print_monitor.py — set_microsteps_per_micron method
    4. gui/pages/print_setup.py   — set_microsteps_per_micron stub

Each patch is a (old_str, new_str) replacement applied with str.replace().
If the old_str is not found, the patch is skipped with a warning.
"""

import os
import sys

# ── Color output ─────────────────────────────────────────────────
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"


def apply_patch(filepath: str, old_str: str, new_str: str, description: str) -> bool:
    """Apply a single str.replace() patch to a file."""
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    if old_str not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description}  (pattern not found)")
        return False

    count = content.count(old_str)
    if count > 1:
        print(f"  {YELLOW}WARN{RESET}: {description}  (pattern found {count}x, replacing first)")
        content = content.replace(old_str, new_str, 1)
    else:
        content = content.replace(old_str, new_str)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"  {GREEN}OK{RESET}: {description}")
    return True


def insert_after(filepath: str, anchor: str, new_text: str, description: str) -> bool:
    """Insert new_text immediately after anchor line."""
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description}  (anchor not found)")
        return False

    if new_text.strip() in content:
        print(f"  {YELLOW}SKIP{RESET}: {description}  (already applied)")
        return False

    content = content.replace(anchor, anchor + new_text, 1)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"  {GREEN}OK{RESET}: {description}")
    return True


def insert_before(filepath: str, anchor: str, new_text: str, description: str) -> bool:
    """Insert new_text immediately before anchor line."""
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()

    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description}  (anchor not found)")
        return False

    if new_text.strip() in content:
        print(f"  {YELLOW}SKIP{RESET}: {description}  (already applied)")
        return False

    content = content.replace(anchor, new_text + anchor, 1)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"  {GREEN}OK{RESET}: {description}")
    return True


# ══════════════════════════════════════════════════════════════════
#  CALIBRATION.PY PATCHES
# ══════════════════════════════════════════════════════════════════

def patch_calibration(filepath: str):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    # 1) Add import for unit_helpers
    insert_after(
        filepath,
        "from gui.styles import COLORS",
        "\nfrom gui.unit_helpers import steps_to_um, format_um, DEFAULT_MICROSTEPS_PER_MICRON",
        "Add unit_helpers import",
    )

    # 2) Add _microsteps_per_micron attribute in __init__
    insert_after(
        filepath,
        "self._context_widget = None",
        "\n\n        # v7.1.1: Microsteps-to-microns conversion factor\n"
        "        self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON",
        "Add _microsteps_per_micron to __init__",
    )

    # 3) Add set_microsteps_per_micron method (after get_page_title)
    insert_after(
        filepath,
        '        return "Calibration"',
        """

    def set_microsteps_per_micron(self, value: float):
        \"\"\"Update the microsteps-per-micron conversion factor.\"\"\"
        self._microsteps_per_micron = value""",
        "Add set_microsteps_per_micron method",
    )

    # 4) Update position readout labels in _setup_ui
    apply_patch(
        filepath,
        'for name, attr in [("X:", "lbl_x"), ("Y:", "lbl_y"), ("Z:", "lbl_z")]:',
        'for name, attr in [("X (µm):", "lbl_x"), ("Y (µm):", "lbl_y"), ("Z (mm):", "lbl_z")]:',
        "Label position readouts with units",
    )

    # 5) Convert on_status_update XY position display to µm
    apply_patch(
        filepath,
        '''            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            self.lbl_x.setText(f"{zx:.0f}")
            self.lbl_y.setText(f"{zy:.0f}")''',
        '''            zx = xy[0] - ctrl.zero_position["x"]
            zy = xy[1] - ctrl.zero_position["y"]
            ux = steps_to_um(zx, self._microsteps_per_micron)
            uy = steps_to_um(zy, self._microsteps_per_micron)
            self.lbl_x.setText(f"{ux:,.1f}")
            self.lbl_y.setText(f"{uy:,.1f}")''',
        "Convert on_status_update XY display to µm",
    )

    # 6) Convert _set_zero display to µm
    apply_patch(
        filepath,
        '''        self.lbl_zero_status.setText(
            f"Set: X={z['x']:.0f} Y={z['y']:.0f} Z={z['Z']:.2f}")''',
        '''        zx_um = steps_to_um(z['x'], self._microsteps_per_micron)
        zy_um = steps_to_um(z['y'], self._microsteps_per_micron)
        self.lbl_zero_status.setText(
            f"Set: X={zx_um:,.1f} µm  Y={zy_um:,.1f} µm  Z={z['Z']:.2f} mm")''',
        "Convert _set_zero display to µm",
    )

    # 7) Convert _record_a1 display to µm
    apply_patch(
        filepath,
        '''        self._taught_a1 = (xy[0], xy[1])
        self.lbl_a1.setText(f"({xy[0]:.0f}, {xy[1]:.0f})")''',
        '''        self._taught_a1 = (xy[0], xy[1])
        ax = steps_to_um(xy[0], self._microsteps_per_micron)
        ay = steps_to_um(xy[1], self._microsteps_per_micron)
        self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) µm")''',
        "Convert _record_a1 display to µm",
    )

    # 8) Convert _record_corner display to µm
    apply_patch(
        filepath,
        '''        self._taught_corner = (xy[0], xy[1])
        self.lbl_corner.setText(f"({xy[0]:.0f}, {xy[1]:.0f})")''',
        '''        self._taught_corner = (xy[0], xy[1])
        cx = steps_to_um(xy[0], self._microsteps_per_micron)
        cy = steps_to_um(xy[1], self._microsteps_per_micron)
        self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) µm")''',
        "Convert _record_corner display to µm",
    )

    # 9) Convert _calculate_alignment offset display to µm
    apply_patch(
        filepath,
        '''        self.lbl_alignment.setText(
            f"✅ Scale: {self._scale:.4f} | "
            f"Rot: {self._rotation:.2f}° | "
            f"Off: ({self._offset_x:.0f}, {self._offset_y:.0f})"
        )''',
        '''        off_x_um = steps_to_um(self._offset_x, self._microsteps_per_micron)
        off_y_um = steps_to_um(self._offset_y, self._microsteps_per_micron)
        self.lbl_alignment.setText(
            f"✅ Scale: {self._scale:.4f} | "
            f"Rot: {self._rotation:.2f}° | "
            f"Off: ({off_x_um:,.1f}, {off_y_um:,.1f}) µm"
        )''',
        "Convert _calculate_alignment offset display to µm",
    )

    # 10) Convert _goto_well target display to µm
    apply_patch(
        filepath,
        '''        self.lbl_val_result.setText(
            f"Moving to {well} → ({target_x:.0f}, {target_y:.0f})"
        )''',
        '''        tx_um = steps_to_um(target_x, self._microsteps_per_micron)
        ty_um = steps_to_um(target_y, self._microsteps_per_micron)
        self.lbl_val_result.setText(
            f"Moving to {well} → ({tx_um:,.1f}, {ty_um:,.1f}) µm"
        )''',
        "Convert _goto_well target display to µm",
    )

    # 11) Convert _load_calibration taught_a1 display to µm
    apply_patch(
        filepath,
        '''            self.lbl_a1.setText(
                f"({self._taught_a1[0]:.0f}, {self._taught_a1[1]:.0f})"
            )''',
        '''            a1x = steps_to_um(self._taught_a1[0], self._microsteps_per_micron)
            a1y = steps_to_um(self._taught_a1[1], self._microsteps_per_micron)
            self.lbl_a1.setText(f"({a1x:,.1f}, {a1y:,.1f}) µm")''',
        "Convert _load_calibration taught_a1 display to µm",
    )

    # 12) Convert _load_calibration taught_corner display to µm
    apply_patch(
        filepath,
        '''            self.lbl_corner.setText(
                f"({self._taught_corner[0]:.0f}, {self._taught_corner[1]:.0f})"
            )''',
        '''            cx = steps_to_um(self._taught_corner[0], self._microsteps_per_micron)
            cy = steps_to_um(self._taught_corner[1], self._microsteps_per_micron)
            self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) µm")''',
        "Convert _load_calibration taught_corner display to µm",
    )


# ══════════════════════════════════════════════════════════════════
#  SETTINGS_PAGE.PY PATCHES
# ══════════════════════════════════════════════════════════════════

def patch_settings_page(filepath: str):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    # 1) Add imports
    insert_after(
        filepath,
        "from gui.styles import COLORS",
        "\nfrom gui.unit_helpers import (\n"
        "    steps_to_um, um_to_steps, convert_safety_xy_text,\n"
        "    DEFAULT_MICROSTEPS_PER_MICRON,\n"
        ")",
        "Add unit_helpers imports",
    )

    # 2) Add _microsteps_per_micron attribute in __init__
    insert_after(
        filepath,
        "self._context_widget = None",
        "\n        self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON",
        "Add _microsteps_per_micron to __init__",
    )

    # 3) Add set_microsteps_per_micron method after on_status_update
    insert_after(
        filepath,
        '        """Called by MainWindow timer. No periodic refresh needed."""',
        """

    def set_microsteps_per_micron(self, value: float):
        \"\"\"Update the conversion factor and refresh related UI.\"\"\"
        self._microsteps_per_micron = value
        if hasattr(self, 'spin_um_factor'):
            self.spin_um_factor.blockSignals(True)
            self.spin_um_factor.setValue(value)
            self.spin_um_factor.blockSignals(False)
        if hasattr(self, 'lbl_safety_um'):
            self._update_safety_um_label()""",
        "Add set_microsteps_per_micron method",
    )

    # 4) Add unit conversion card builder method (before _build_safety_card)
    insert_before(
        filepath,
        "    def _build_safety_card(self, parent_layout):",
        """    def _build_unit_conversion_card(self, parent_layout):
        \"\"\"Build the XY unit conversion settings card.\"\"\"
        card = QFrame()
        card.setObjectName("cardFrame")
        layout = QVBoxLayout(card)
        layout.setSpacing(6)

        title = QLabel("XY Unit Conversion")
        title.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; "
            f"color: {COLORS['text']};")
        layout.addWidget(title)

        desc = QLabel(
            "The XY stage reports positions in microsteps.\\n"
            "Set the conversion factor to display in microns (µm).\\n"
            "ProScan III default: 10.0 microsteps/µm (0.1 µm resolution)")
        desc.setWordWrap(True)
        desc.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 9pt;")
        layout.addWidget(desc)

        row = QHBoxLayout()
        row.addWidget(QLabel("Microsteps per µm:"))
        self.spin_um_factor = QDoubleSpinBox()
        self.spin_um_factor.setRange(0.001, 10000.0)
        self.spin_um_factor.setDecimals(3)
        self.spin_um_factor.setSingleStep(0.1)
        self.spin_um_factor.setValue(self._microsteps_per_micron)
        self.spin_um_factor.setToolTip(
            "Conversion factor: microsteps_per_micron\\n"
            "ProScan III: 10.0 (0.1 µm/step)\\n"
            "ProScan II: 10.0 (0.1 µm/step)")
        row.addWidget(self.spin_um_factor)
        layout.addLayout(row)

        parent_layout.addWidget(card)

""",
        "Add _build_unit_conversion_card method",
    )

    # 5) Call _build_unit_conversion_card from _setup_ui (before safety card)
    insert_before(
        filepath,
        "        self._build_safety_card(content_layout)",
        "        self._build_unit_conversion_card(content_layout)\n",
        "Call _build_unit_conversion_card in _setup_ui",
    )

    # 6) Add µm summary label inside _build_safety_card (after enabled checkbox)
    insert_after(
        filepath,
        "        self.chk_safety_enabled.toggled.connect(self._safety_main_toggled)\n        layout.addWidget(self.chk_safety_enabled)",
        """

        # XY limits summary in µm
        self.lbl_safety_um = QLabel("")
        self.lbl_safety_um.setWordWrap(True)
        self.lbl_safety_um.setStyleSheet(
            f"color: {COLORS['overlay0']}; font-size: 9pt; padding: 2px;")
        layout.addWidget(self.lbl_safety_um)""",
        "Add µm safety summary label",
    )

    # 7) Add _update_safety_um_label helper
    insert_before(
        filepath,
        "    def _refresh_ports(self):",
        """    def _update_safety_um_label(self):
        \"\"\"Update the safety limits µm summary label.\"\"\"
        if not hasattr(self, 'lbl_safety_um'):
            return
        try:
            sl = self.controller.safety_limits
            self.lbl_safety_um.setText(
                convert_safety_xy_text(sl, self._microsteps_per_micron))
        except Exception:
            self.lbl_safety_um.setText("")

""",
        "Add _update_safety_um_label helper",
    )

    # 8) Save microsteps_per_micron in _apply_settings (before settings.save())
    apply_patch(
        filepath,
        '        self.settings.save()\n        logger.info("Settings applied and saved")',
        '        # Unit conversion factor\n'
        '        if hasattr(self, \'spin_um_factor\'):\n'
        '            um_val = self.spin_um_factor.value()\n'
        '            self.settings.set("stage.microsteps_per_micron", um_val)\n'
        '            self._microsteps_per_micron = um_val\n'
        '\n'
        '        self.settings.save()\n'
        '        logger.info("Settings applied and saved")\n'
        '\n'
        '        # Update µm summary\n'
        '        self._update_safety_um_label()',
        "Save microsteps_per_micron in _apply_settings",
    )

    # 9) Load microsteps_per_micron in _load_from_controller (after verbose checkbox)
    insert_after(
        filepath,
        "        self.chk_verbose.setChecked(\n"
        '            self.settings.get("logging.verbose", False))',
        """

        # Unit conversion factor
        um_val = self.settings.get("stage.microsteps_per_micron",
                                   DEFAULT_MICROSTEPS_PER_MICRON)
        self._microsteps_per_micron = float(um_val)
        if hasattr(self, 'spin_um_factor'):
            self.spin_um_factor.setValue(self._microsteps_per_micron)""",
        "Load microsteps_per_micron in _load_from_controller",
    )

    # 10) Convert _set_xy_from_current display to µm
    apply_patch(
        filepath,
        '''            logger.info(
                f"XY {'max' if as_max else 'min'} set to "
                f"({zero_x:.0f}, {zero_y:.0f})")''',
        '''            ux = steps_to_um(zero_x, self._microsteps_per_micron)
            uy = steps_to_um(zero_y, self._microsteps_per_micron)
            logger.info(
                f"XY {'max' if as_max else 'min'} set to "
                f"({ux:,.1f}, {uy:,.1f}) µm")''',
        "Convert _set_xy_from_current display to µm",
    )


# ══════════════════════════════════════════════════════════════════
#  PRINT_MONITOR.PY PATCHES
# ══════════════════════════════════════════════════════════════════

def patch_print_monitor(filepath: str):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    # 1) Add import for unit_helpers
    insert_after(
        filepath,
        "from gui.styles import COLORS",
        "\nfrom gui.unit_helpers import steps_to_um, DEFAULT_MICROSTEPS_PER_MICRON",
        "Add unit_helpers import",
    )

    # 2) Add _microsteps_per_micron attribute in __init__
    #    Look for the recorder init line
    insert_after(
        filepath,
        "        self._recorder = None",
        "\n        self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON",
        "Add _microsteps_per_micron to __init__",
    )

    # 3) Add set_microsteps_per_micron method (after set_recorder)
    insert_after(
        filepath,
        "        self._recorder = recorder",
        """

    def set_microsteps_per_micron(self, value: float):
        \"\"\"Update the microsteps-per-micron conversion factor.\"\"\"
        self._microsteps_per_micron = value""",
        "Add set_microsteps_per_micron method",
    )


# ══════════════════════════════════════════════════════════════════
#  PRINT_SETUP.PY PATCHES
# ══════════════════════════════════════════════════════════════════

def patch_print_setup(filepath: str):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    # 1) Add set_microsteps_per_micron stub (after get_page_title)
    insert_after(
        filepath,
        '        return "Print Setup"',
        """

    def set_microsteps_per_micron(self, value: float):
        \"\"\"Update the microsteps-per-micron conversion factor.\"\"\"
        self._microsteps_per_micron = value""",
        "Add set_microsteps_per_micron method",
    )

    # 2) Add _microsteps_per_micron attribute init
    insert_after(
        filepath,
        "        self._context_widget = None",
        "\n        self._microsteps_per_micron = 10.0  # Default, updated by MainWindow",
        "Add _microsteps_per_micron to __init__",
    )


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    # Detect project root
    root = os.getcwd()
    gui_pages = os.path.join(root, "gui", "pages")

    if not os.path.isdir(gui_pages):
        # Try from script location
        root = os.path.dirname(os.path.abspath(__file__))
        gui_pages = os.path.join(root, "gui", "pages")

    if not os.path.isdir(gui_pages):
        print(f"{RED}ERROR{RESET}: Cannot find gui/pages/ directory.")
        print(f"  Searched in: {root}")
        print(f"  Run this script from the MEBP root directory.")
        sys.exit(1)

    print(f"MEBP root: {root}")
    print(f"Applying µm conversion patches...\n")

    patch_calibration(os.path.join(gui_pages, "calibration.py"))
    patch_settings_page(os.path.join(gui_pages, "settings_page.py"))
    patch_print_monitor(os.path.join(gui_pages, "print_monitor.py"))
    patch_print_setup(os.path.join(gui_pages, "print_setup.py"))

    print(f"\n{'='*60}")
    print(f"{GREEN}Done!{RESET} Patches applied.")
    print(f"{'='*60}")
    print(f"\nNext steps:")
    print(f"  1. Copy gui/unit_helpers.py to your project")
    print(f"  2. Copy the updated gui/app.py to your project")
    print(f"  3. Run the app to verify: python main.py")
    print(f"  4. Adjust microsteps_per_micron in Settings → XY Unit Conversion")


if __name__ == "__main__":
    main()
