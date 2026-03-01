#!/usr/bin/env python3
"""
apply_v72_dashboard_patch.py — Patch dashboard.py for v7.2 µL pump display.

Changes:
    1. Add HardwareConfig import
    2. Add _hardware_config attribute to __init__
    3. Add set_hardware_config() method
    4. Update update_data() to show pump positions in µL
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

    filepath = os.path.join(root, "gui", "pages", "dashboard.py")

    print(f"\n{BOLD}MEBP v7.2 Dashboard Patch{RESET}")
    print(f"{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: {filepath} not found!")
        sys.exit(1)

    content = read_file(filepath)

    # 1. Add HardwareConfig import
    content = insert_after(
        content,
        "from gui.styles import COLORS",
        "\ntry:\n"
        "    from SupportClasses.HardwareConfig import HardwareConfig\n"
        "except ImportError:\n"
        "    HardwareConfig = None",
        "Add HardwareConfig import",
    )

    # 2. Add _hardware_config to __init__
    content = insert_after(
        content,
        "self._microsteps_per_micron: float = 10.0",
        "        self._hardware_config = None  # v7.2: HardwareConfig for µL display",
        "Add _hardware_config to __init__",
    )

    # 3. Add set_hardware_config method
    content = insert_after(
        content,
        '''    def set_microsteps_per_micron(self, value: float):
        """Called by MainWindow when the conversion factor changes."""
        self._microsteps_per_micron = max(0.001, value)''',
        '''
    def set_hardware_config(self, config):
        """v7.2: Set hardware config for µL pump display."""
        self._hardware_config = config
''',
        "Add set_hardware_config method",
    )

    # 4. Replace the ZP position display block in update_data()
    old_zp_block = """        # ZP position (already in mm)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position['P1']:.2f}")
            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position['P2']:.2f}")
            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position['P3']:.2f}")"""

    new_zp_block = """        # ZP position — Z in mm, pumps in µL (v7.2) or mm (fallback)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            for pid, lbl in [("P1", self.lbl_p1), ("P2", self.lbl_p2), ("P3", self.lbl_p3)]:
                idx = {"P1": 1, "P2": 2, "P3": 3}[pid]
                pos_mm = zp[idx] if idx < len(zp) else None
                zero_ref = ctrl.zero_position.get(pid, 0)
                if pos_mm is not None:
                    rel_mm = pos_mm - zero_ref
                    if self._hardware_config:
                        pump_cfg = self._hardware_config.pumps.get(pid)
                        if pump_cfg and pump_cfg.is_configured:
                            try:
                                pos_uL = pump_cfg.mm_to_uL(rel_mm)
                                lbl.setText(f"{pos_uL:.2f} µL")
                                continue
                            except (ValueError, AttributeError):
                                pass
                    lbl.setText(f"{rel_mm:.2f}")
                else:
                    lbl.setText("—")"""

    content = replace_first(content, old_zp_block, new_zp_block,
                            "Update ZP position display for µL")

    write_file(filepath, content)
    print(f"\n  {GREEN}DONE{RESET}: dashboard.py patched for v7.2 µL display")


if __name__ == "__main__":
    main()
