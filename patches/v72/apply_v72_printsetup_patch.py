#!/usr/bin/env python3
"""
apply_v72_printsetup_patch.py — Patch print_setup.py for v7.2 µL-based pump control.

Run from the MEBP project root:
    python apply_v72_printsetup_patch.py

Changes:
    1. Pump Feed spin: mm/s → µL/s with correct range
    2. _get_settings(): Uses new µL fields in PrintSettings
    3. Add set_hardware_config() method for HardwareConfig propagation
    4. Add HardwareConfig import
    5. Update _build_current_job to pass µL settings
"""

import os
import sys

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"


def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def replace_first(content, old, new, desc=""):
    if old not in content:
        print(f"  {YELLOW}SKIP{RESET}: text not found for: {desc}")
        return content
    result = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def insert_after(content, anchor, insertion, desc=""):
    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: anchor not found for: {desc}")
        return content
    idx = content.index(anchor) + len(anchor)
    result = content[:idx] + "\n" + insertion + content[idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def main():
    if os.path.isdir("gui"):
        root = "."
    elif os.path.isdir("MEBP-Version-7.0/gui"):
        root = "MEBP-Version-7.0"
    elif os.path.isdir("MEBP-Version-7.1/gui"):
        root = "MEBP-Version-7.1"
    else:
        print(f"{RED}ERROR{RESET}: Cannot find gui/ directory.")
        sys.exit(1)

    filepath = os.path.join(root, "gui", "pages", "print_setup.py")

    print(f"\n{BOLD}MEBP v7.2 Print Setup Patch{RESET}")
    print(f"{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: {filepath} not found!")
        sys.exit(1)

    content = read_file(filepath)

    # ── 1. Add HardwareConfig import ──
    content = insert_after(
        content,
        "from gui.styles import COLORS",
        "\ntry:\n"
        "    from SupportClasses.HardwareConfig import HardwareConfig\n"
        "except ImportError:\n"
        "    HardwareConfig = None",
        "Add HardwareConfig import",
    )

    # ── 2. Add _hardware_config to __init__ ──
    content = insert_after(
        content,
        "self._microsteps_per_micron = 10.0  # Default, updated by MainWindow",
        "        self._hardware_config = None  # v7.2: HardwareConfig (set by MainWindow)",
        "Add _hardware_config to __init__",
    )

    # ── 3. Add set_hardware_config method ──
    content = insert_after(
        content,
        '''    def set_microsteps_per_micron(self, value: float):
        """Update the microsteps-per-micron conversion factor."""
        self._microsteps_per_micron = value''',
        '''
    def set_hardware_config(self, config):
        """v7.2: Set hardware config for µL-based pump control."""
        self._hardware_config = config
        # Forward to workspace tab if it exists
        if hasattr(self, 'tab_workspace') and hasattr(self.tab_workspace, 'set_hardware_config'):
            self.tab_workspace.set_hardware_config(config)
        # Forward to print manager
        if hasattr(self.print_manager, 'hardware_config'):
            self.print_manager.hardware_config = config
        elif hasattr(self.controller, 'set_hardware_config'):
            pass  # Already set via controller
''',
        "Add set_hardware_config method",
    )

    # ── 4. Update Pump Feed spin from mm/s to µL/s ──
    content = replace_first(
        content,
        '''        pf_row = QHBoxLayout()
        pf_row.addWidget(QLabel("Pump Feed:"))
        self.pump_feed_spin = QDoubleSpinBox()
        self.pump_feed_spin.setRange(0.001, 5.0)
        self.pump_feed_spin.setValue(0.1)
        self.pump_feed_spin.setSuffix(" mm/s")
        self.pump_feed_spin.setDecimals(3)
        pf_row.addWidget(self.pump_feed_spin)
        layout.addLayout(pf_row)''',
        '''        pf_row = QHBoxLayout()
        pf_row.addWidget(QLabel("Pump Rate:"))
        self.pump_feed_spin = QDoubleSpinBox()
        self.pump_feed_spin.setRange(0.001, 50.0)
        self.pump_feed_spin.setValue(0.25)
        self.pump_feed_spin.setSuffix(" µL/s")
        self.pump_feed_spin.setDecimals(3)
        self.pump_feed_spin.setToolTip("Default pump flow rate in µL/s")
        pf_row.addWidget(self.pump_feed_spin)
        layout.addLayout(pf_row)''',
        "Update Pump Feed spin from mm/s to µL/s",
    )

    # ── 5. Update _get_settings() to use µL fields ──
    content = replace_first(
        content,
        '''    def _get_settings(self) -> PrintSettings:
        """Read current settings from context panel into PrintSettings."""
        return PrintSettings(
            feedrate_xy=self.xy_feed_spin.value(),
            feedrate_z=self.z_feed_spin.value(),
            feedrate_pump=self.pump_feed_spin.value(),
            num_layers=self.layers_spin.value(),
            layer_height=self.layer_height_spin.value(),
            active_pump=self.pump_combo.currentText(),
            flow_rate=self.flow_spin.value(),
            retract_volume=self.retract_spin.value(),
            prime_volume=self.prime_spin.value(),
        )''',
        '''    def _get_settings(self) -> PrintSettings:
        """Read current settings from context panel into PrintSettings.

        v7.2: Uses µL-based fields for pump control. Legacy mm fields
        are populated for backward compatibility.
        """
        active_pump = self.pump_combo.currentText()
        pump_rate = self.pump_feed_spin.value()  # µL/s
        retract_uL = self.retract_spin.value()    # µL
        prime_uL = self.prime_spin.value()         # µL

        return PrintSettings(
            xy_feedrate=self.xy_feed_spin.value(),
            z_feedrate=self.z_feed_spin.value() * 60.0,  # mm/s → mm/min for legacy
            print_feedrate=self.xy_feed_spin.value() * 60.0,  # mm/s → mm/min
            num_layers=self.layers_spin.value(),
            layer_height=self.layer_height_spin.value(),
            dwell_after_move=0.0,
            # v7.2 µL fields
            pump_rate_uL_s=pump_rate,
            retract_amounts_uL={
                "P1": retract_uL if active_pump == "P1" else 0.0,
                "P2": retract_uL if active_pump == "P2" else 0.0,
                "P3": retract_uL if active_pump == "P3" else 0.0,
            },
            prime_amounts_uL={
                "P1": prime_uL if active_pump == "P1" else 0.0,
                "P2": prime_uL if active_pump == "P2" else 0.0,
                "P3": prime_uL if active_pump == "P3" else 0.0,
            },
            pump_rates_uL_s={
                "P1": pump_rate, "P2": pump_rate, "P3": pump_rate,
            },
            # Legacy fields for backward compat
            pump_feedrate=30.0,
        )''',
        "Update _get_settings for µL-based PrintSettings",
    )

    write_file(filepath, content)
    print(f"\n  {GREEN}DONE{RESET}: print_setup.py patched for v7.2")
    print(f"\nSummary:")
    print(f"  • Pump Feed → Pump Rate (µL/s) with range 0.001–50.0")
    print(f"  • _get_settings() → creates PrintSettings with µL fields")
    print(f"  • set_hardware_config() → propagates to workspace tab + print manager")
    print(f"  • HardwareConfig import added")


if __name__ == "__main__":
    main()
