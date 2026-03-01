#!/usr/bin/env python3
"""
apply_v72_patches.py — Apply v7.2 Hardware Setup & µL patches to existing MEBP files.

Run from the MEBP project root:
    python apply_v72_patches.py

This script modifies:
    1. SupportClasses/StageController.py — adds µL methods + HardwareConfig support
    2. SupportClasses/__init__.py — adds HardwareConfig exports
    3. SupportClasses/Settings.py — adds hardware_config to DEFAULTS

It does NOT modify app.py — use the replacement app.py provided instead.

Pre-requisites (files must exist in project):
    - SupportClasses/HardwareConfig.py (new file, already delivered)
    - gui/pages/hardware_setup.py (new file, already delivered)
    - gui/pages/jog_control.py (replacement file, already delivered)
"""

import os
import re
import sys

# Colors for terminal output
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


def insert_after(content, anchor, insertion, desc=""):
    """Insert text after the first occurrence of anchor."""
    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: anchor not found for: {desc}")
        return content
    idx = content.index(anchor) + len(anchor)
    result = content[:idx] + "\n" + insertion + content[idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def insert_before(content, anchor, insertion, desc=""):
    """Insert text before the first occurrence of anchor."""
    if anchor not in content:
        print(f"  {YELLOW}SKIP{RESET}: anchor not found for: {desc}")
        return content
    idx = content.index(anchor)
    result = content[:idx] + insertion + "\n" + content[idx:]
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def replace_text(content, old, new, desc=""):
    """Replace first occurrence of old with new."""
    if old not in content:
        print(f"  {YELLOW}SKIP{RESET}: text not found for: {desc}")
        return content
    result = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


def append_to_file(content, text, desc=""):
    """Append text to end of file."""
    result = content.rstrip() + "\n\n" + text + "\n"
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


# ═══════════════════════════════════════════════════════════════════
# PATCH 1: StageController.py
# ═══════════════════════════════════════════════════════════════════

def patch_stage_controller(filepath):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # 1) Add import for HardwareConfig
    content = insert_after(
        content,
        "from SupportClasses.SafetyLimits import SafetyLimits",
        "from SupportClasses.HardwareConfig import HardwareConfig",
        "Add HardwareConfig import",
    )

    # 2) Add _hardware_config to __init__
    content = insert_after(
        content,
        '        self.on_disconnect: Callable | None = None',
        '''
        # v7.2: Hardware configuration (set by GUI when hardware setup completes)
        self._hardware_config: HardwareConfig | None = None''',
        "Add _hardware_config to __init__",
    )

    # 3) Append µL methods to the class
    ul_methods = '''
    # ══════════════════════════════════════════════════════════════
    #  v7.2: µL-Based Pump Control
    # ══════════════════════════════════════════════════════════════

    def set_hardware_config(self, config: HardwareConfig) -> None:
        """
        Set the hardware configuration. Called by MainWindow when
        hardware setup is completed or modified.

        Enables µL-based pump methods and updates safety limits.
        """
        self._hardware_config = config
        if config:
            for pid in ["P1", "P2", "P3"]:
                pump_cfg = config.pumps.get(pid)
                if pump_cfg and pump_cfg.is_configured:
                    logger.debug(f"{pid}: syringe={pump_cfg.syringe.volume_uL}µL")
        logger.info("StageController: hardware config updated")

    @property
    def hardware_config(self) -> HardwareConfig | None:
        """Get the current hardware configuration."""
        return self._hardware_config

    def move_pump_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None
    ) -> None:
        """
        Move a pump by a specified volume in µL.

        This is the primary pump movement method for v7.2+.
        Converts µL → mm using the syringe spec, and µL/s → mm/min
        for the feedrate.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
            volume_uL: Volume to dispense (+) or aspirate (-) in µL
            rate_uL_s: Flow rate in µL/s. If None, uses default feedrate.

        Raises:
            ValueError: If pump has no syringe configured
        """
        if not self._hardware_config:
            raise ValueError("No hardware config — complete Hardware Setup first")

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            raise ValueError(f"{pump}: No syringe configured")

        # Convert µL → mm
        distance_mm = pump_cfg.uL_to_mm(volume_uL)

        # Convert rate µL/s → mm/min
        feedrate_mm_min = None
        if rate_uL_s is not None:
            feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(rate_uL_s))
            # Clamp flow rate via safety limits
            if self.safety_limits.enabled:
                clamped = self.safety_limits.clamp_flow_rate(rate_uL_s, pump)
                if abs(clamped) != abs(rate_uL_s):
                    feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(clamped))

        logger.debug(
            f"move_pump_uL({pump}, {volume_uL:+.3f} µL"
            + (f", {rate_uL_s:.3f} µL/s" if rate_uL_s else "")
            + f") → {distance_mm:+.5f} mm"
            + (f", {feedrate_mm_min:.1f} mm/min" if feedrate_mm_min else "")
        )
        self.move_pump_relative(pump, distance_mm, feedrate_mm_min)

    def get_pump_position_uL(self, pump: str) -> float | None:
        """
        Get the current pump position in µL (relative to zero reference).

        Returns None if unavailable or no syringe configured.
        """
        if not self._hardware_config:
            return None

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            return None

        pos = self.get_zp_position(cached=True)
        if pos is None or pos[0] is None:
            return None

        idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
        if idx >= len(pos):
            return None

        pos_mm = pos[idx]
        zero_ref = self.zero_position.get(pump, 0)
        relative_mm = pos_mm - zero_ref

        try:
            return pump_cfg.mm_to_uL(relative_mm)
        except ValueError:
            return None

    def get_all_pump_positions_uL(self) -> dict[str, float | None]:
        """Get all pump positions in µL as a dict."""
        return {pid: self.get_pump_position_uL(pid) for pid in ["P1", "P2", "P3"]}

    def extrude_uL(
        self, pump: str, volume_uL: float, rate_uL_s: float | None = None
    ) -> bool:
        """
        Extrude (dispense) a specific volume with fluid column tracking.

        Positive volume_uL = dispense, negative = aspirate.
        Updates FluidColumn in HardwareConfig if available.

        Returns True if executed successfully.
        """
        if not self._hardware_config:
            logger.error("No hardware config — cannot extrude")
            return False

        pump_cfg = self._hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            logger.error(f"{pump}: not configured — cannot extrude")
            return False

        # Track fluid column
        if volume_uL > 0:
            if not pump_cfg.fluid_column.can_dispense(volume_uL):
                logger.warning(
                    f"{pump}: Requested {volume_uL:.2f} µL but only "
                    f"{pump_cfg.fluid_column.ink_volume_uL:.2f} µL available"
                )
            pump_cfg.fluid_column.dispense(volume_uL)
        elif volume_uL < 0 and pump_cfg.ink:
            pump_cfg.fluid_column.aspirate_ink(abs(volume_uL), pump_cfg.ink)

        self.move_pump_uL(pump, volume_uL, rate_uL_s)
        return True'''

    # Find the end of the StageController class to append methods
    # Look for the shutdown method which is typically near the end
    if "def shutdown(self)" in content:
        # Find the next class definition or end of file after shutdown
        shutdown_idx = content.index("def shutdown(self)")
        # Find the end of the shutdown method (next def at same indent or end of class)
        # We'll insert before any non-class code after the class
        # Simplest: append after the last method in the class

        # Strategy: find 'def shutdown' and insert after its body
        lines = content.split('\n')
        in_shutdown = False
        insert_line = len(lines)
        for i, line in enumerate(lines):
            if 'def shutdown(self)' in line:
                in_shutdown = True
                continue
            if in_shutdown:
                # Find next top-level class/function definition or end
                if line.strip() and not line.startswith(' ') and not line.startswith('\t'):
                    insert_line = i
                    break
                # Or next class-level method at the correct indent
                if line.startswith('class ') or (line.startswith('# ═') and i > shutdown_idx + 5):
                    insert_line = i
                    break

        lines.insert(insert_line, ul_methods)
        content = '\n'.join(lines)
        print(f"  {GREEN}OK{RESET}: Added µL pump methods (move_pump_uL, get_pump_position_uL, extrude_uL)")
    else:
        # Fallback: append to end of file
        content = append_to_file(content, ul_methods, "Added µL pump methods (appended to end)")

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: StageController.py patched")


# ═══════════════════════════════════════════════════════════════════
# PATCH 2: SupportClasses/__init__.py
# ═══════════════════════════════════════════════════════════════════

def patch_init(filepath):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Add import
    content = insert_after(
        content,
        "from SupportClasses.SafetyLimits import SafetyLimits",
        """from SupportClasses.HardwareConfig import HardwareConfig, PumpChannelConfig""",
        "Add HardwareConfig import",
    )

    # Add to __all__
    content = replace_text(
        content,
        '"SafetyLimits",',
        '"SafetyLimits",\n    "HardwareConfig", "PumpChannelConfig",',
        "Add HardwareConfig to __all__",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: __init__.py patched")


# ═══════════════════════════════════════════════════════════════════
# PATCH 3: Settings.py — add hardware_config to DEFAULTS
# ═══════════════════════════════════════════════════════════════════

def patch_settings(filepath):
    print(f"\n{'='*60}")
    print(f"Patching: {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Add hardware_config section to DEFAULTS dict
    # Find the "ui" section which is typically near the end of DEFAULTS
    content = insert_before(
        content,
        '"ui": {',
        '''    "hardware_config": None,  # v7.2: HardwareConfig dict (set by Hardware Setup page)
    ''',
        "Add hardware_config to DEFAULTS",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: Settings.py patched")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    # Detect project root
    if os.path.isdir("SupportClasses"):
        root = "."
    elif os.path.isdir("MEBP-Version-7.0/SupportClasses"):
        root = "MEBP-Version-7.0"
    elif os.path.isdir("MEBP-Version-7.1/SupportClasses"):
        root = "MEBP-Version-7.1"
    else:
        print(f"{RED}ERROR{RESET}: Cannot find SupportClasses/ directory.")
        print("Run this script from the MEBP project root.")
        sys.exit(1)

    print(f"\n{BOLD}MEBP v7.2 Patch Script{RESET}")
    print(f"Project root: {os.path.abspath(root)}")

    # Check prerequisites
    hw_config = os.path.join(root, "SupportClasses", "HardwareConfig.py")
    hw_setup = os.path.join(root, "gui", "pages", "hardware_setup.py")
    if not os.path.isfile(hw_config):
        print(f"\n{YELLOW}WARNING{RESET}: SupportClasses/HardwareConfig.py not found.")
        print("Copy it from the delivered files first.")
    if not os.path.isfile(hw_setup):
        print(f"\n{YELLOW}WARNING{RESET}: gui/pages/hardware_setup.py not found.")
        print("Copy it from the delivered files first.")

    # Apply patches
    patch_stage_controller(os.path.join(root, "SupportClasses", "StageController.py"))
    patch_init(os.path.join(root, "SupportClasses", "__init__.py"))
    patch_settings(os.path.join(root, "SupportClasses", "Settings.py"))

    print(f"\n{BOLD}Patches applied.{RESET}")
    print(f"\nRemaining manual steps:")
    print(f"  1. Replace gui/app.py with the new version (see app_v72.py)")
    print(f"  2. Replace gui/pages/jog_control.py with the new version")
    print(f"  3. Copy SupportClasses/HardwareConfig.py (new file)")
    print(f"  4. Copy gui/pages/hardware_setup.py (new file)")
    print(f"  5. Copy config/hardware/sample_setup.json (new file)")


if __name__ == "__main__":
    main()
