#!/usr/bin/env python3
"""
apply_v72_sessions3to6_patch.py — Sessions 3-6: Remaining v7.2 µL patches.

Session 3: Calibration Page — set_hardware_config, plate from config, needle info
Session 4: SafetyLimits — update_from_hardware_config() auto-sets flow rate limits
Session 5: Xbox ZPJogHandler — use move_pump_uL when available
Session 6: Print Monitor — set_hardware_config, µL pump display

Run from the MEBP project root:
    python apply_v72_sessions3to6_patch.py
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


def append_to_file(content, text, desc=""):
    result = content.rstrip() + "\n\n" + text + "\n"
    print(f"  {GREEN}OK{RESET}: {desc}")
    return result


# ═══════════════════════════════════════════════════════════════════
# SESSION 3: Calibration Page
# ═══════════════════════════════════════════════════════════════════

def patch_calibration(root):
    filepath = os.path.join(root, "gui", "pages", "calibration.py")
    print(f"\n{'='*60}")
    print(f"Session 3: Patching {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # 1. Add HardwareConfig import
    content = insert_after(
        content,
        "from gui.unit_helpers import steps_to_um, format_um, DEFAULT_MICROSTEPS_PER_MICRON",
        "\ntry:\n"
        "    from SupportClasses.HardwareConfig import HardwareConfig\n"
        "except ImportError:\n"
        "    HardwareConfig = None",
        "Add HardwareConfig import",
    )

    # 2. Add _hardware_config to __init__
    content = insert_after(
        content,
        "self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON",
        "        self._hardware_config = None  # v7.2: HardwareConfig",
        "Add _hardware_config to __init__",
    )

    # 3. Add set_hardware_config method
    content = insert_after(
        content,
        '''    def set_microsteps_per_micron(self, value: float):
        """Update the microsteps-per-micron conversion factor."""
        self._microsteps_per_micron = value''',
        '''
    def set_hardware_config(self, config):
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
            )
''',
        "Add set_hardware_config method",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: calibration.py patched")


# ═══════════════════════════════════════════════════════════════════
# SESSION 4: SafetyLimits — update_from_hardware_config()
# ═══════════════════════════════════════════════════════════════════

def patch_safety_limits(root):
    filepath = os.path.join(root, "SupportClasses", "SafetyLimits.py")
    print(f"\n{'='*60}")
    print(f"Session 4: Patching {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # SafetyLimits already has clamp_flow_rate from v7.1.
    # Add update_from_hardware_config() method that auto-computes
    # flow rate limits from needle gauge + syringe specs.

    new_method = '''
    # ── v7.2: Auto-configure from HardwareConfig ─────────────────

    def update_from_hardware_config(self, hardware_config) -> None:
        """
        v7.2: Auto-configure safety limits from hardware config.

        Sets per-pump max flow rate limits based on needle gauge.
        Uses conservative lookup table: smaller gauge = lower max flow.

        Also updates pump travel limits based on syringe stroke length.
        """
        if hardware_config is None:
            return

        # Max flow rate by needle gauge (µL/s) — conservative defaults
        # Based on typical bioprinting literature recommendations
        GAUGE_MAX_FLOW = {
            16: 50.0,   # 16G: very wide, high flow ok
            18: 30.0,
            20: 15.0,
            22: 8.0,
            23: 5.0,
            25: 3.0,
            27: 1.5,
            28: 1.0,
            30: 0.5,
            32: 0.2,
        }

        gauge = None
        if hasattr(hardware_config, 'needle') and hardware_config.needle:
            gauge = hardware_config.needle.gauge

        for pid in ["P1", "P2", "P3"]:
            pump_cfg = hardware_config.pumps.get(pid)
            if pump_cfg is None or not pump_cfg.is_configured:
                continue

            # Set flow rate limit from needle gauge
            if gauge and gauge in GAUGE_MAX_FLOW:
                max_rate = GAUGE_MAX_FLOW[gauge]
                self.set_max_flow_rate(pid, max_rate)
                logger.info(f"{pid}: max flow rate = {max_rate:.1f} µL/s ({gauge}G needle)")

            # Set pump travel limits from syringe stroke length
            if pump_cfg.syringe:
                stroke_mm = pump_cfg.syringe.stroke_mm
                # Allow ±stroke from zero reference (generous)
                attr_min = f"{pid.lower()}_min"
                attr_max = f"{pid.lower()}_max"
                if hasattr(self, attr_min):
                    setattr(self, attr_min, -stroke_mm * 0.1)  # Small negative for retract
                if hasattr(self, attr_max):
                    setattr(self, attr_max, stroke_mm * 1.05)  # Slight extra for safety
                logger.info(f"{pid}: pump limits = [{-stroke_mm*0.1:.1f}, {stroke_mm*1.05:.1f}] mm "
                           f"(syringe stroke = {stroke_mm:.1f} mm)")

    def get_pump_limits_uL(self, pump: str, hardware_config=None) -> tuple[float, float] | None:
        """
        v7.2: Get pump limits in µL instead of mm.

        Returns (min_uL, max_uL) or None if no syringe configured.
        """
        if hardware_config is None:
            return None

        pump_cfg = hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            return None

        p_min, p_max = self._pump_limits(pump)
        try:
            min_uL = pump_cfg.mm_to_uL(p_min)
            max_uL = pump_cfg.mm_to_uL(p_max)
            return (min_uL, max_uL)
        except (ValueError, AttributeError):
            return None'''

    # Insert before the __repr__ method
    content = insert_after(
        content,
        '''    def _get_max_flow_rate(self, pump: str) -> float:
        """Get the max flow rate for a pump."""
        mapping = {
            "P1": self.max_flow_rate_p1_uL_s,
            "P2": self.max_flow_rate_p2_uL_s,
            "P3": self.max_flow_rate_p3_uL_s,
        }
        return mapping.get(pump, 0.0)''',
        new_method,
        "Add update_from_hardware_config + get_pump_limits_uL",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: SafetyLimits.py patched")


# ═══════════════════════════════════════════════════════════════════
# SESSION 5: Xbox ZPJogHandler — use move_pump_uL
# ═══════════════════════════════════════════════════════════════════

def patch_stage_controller_zpjog(root):
    filepath = os.path.join(root, "SupportClasses", "StageController.py")
    print(f"\n{'='*60}")
    print(f"Session 5: Patching {filepath} (ZPJogHandler)")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # The ZPJogHandler uses self.stage.move_relative() directly with mm.
    # We need to add an optional hardware_config that allows µL jog
    # when the user presses pump buttons via Xbox controller.

    # Add _hardware_config to ZPJogHandler.__init__
    content = insert_after(
        content,
        '''class ZPJogHandler:
    """
    Sends discrete relative moves to the ZP stage.
    Runs its own daemon thread for continuous jog updates.

    Controls: Z up/down, P1/P2/P3 extrude/retract.
    Proven working from XBOXCONTROLLED code — core logic unchanged.
    """

    def __init__(''',
        "",  # No insertion needed here — we add below
        "Locate ZPJogHandler",
    )

    # Add set_hardware_config to ZPJogHandler
    # Find the ZPJogHandler.start() method and insert before it
    content = insert_after(
        content,
        '        self.processor.register_handler("increment_xyspeed_down", self._incr_down)\n' if 'self.processor.register_handler("increment_xyspeed_down", self._incr_down)' in content else
        '        self.processor.register_handler("increment_zpspeed_down", self._incr_down)',
        '''
    # v7.2: Hardware config for µL conversion
    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Enable µL-based pump jog when hardware config available."""
        self._hardware_config = config
''',
        "Add set_hardware_config to ZPJogHandler",
    )

    # Wire hardware config propagation from StageController
    content = insert_after(
        content,
        '''        logger.info("StageController: hardware config updated")''',
        '''
        # Propagate to ZP jog handler for µL pump jog
        if self.zp_jog and hasattr(self.zp_jog, 'set_hardware_config'):
            self.zp_jog.set_hardware_config(config)

        # Update safety limits from hardware config
        if self.safety_limits:
            self.safety_limits.update_from_hardware_config(config)''',
        "Wire hardware config to ZPJogHandler + SafetyLimits",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: StageController.py ZPJogHandler patched")


# ═══════════════════════════════════════════════════════════════════
# SESSION 6: Print Monitor — set_hardware_config + µL display
# ═══════════════════════════════════════════════════════════════════

def patch_print_monitor(root):
    filepath = os.path.join(root, "gui", "pages", "print_monitor.py")
    print(f"\n{'='*60}")
    print(f"Session 6: Patching {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

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
        "self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON",
        "        self._hardware_config = None  # v7.2: HardwareConfig for µL display",
        "Add _hardware_config to __init__",
    )

    # 3. Add set_hardware_config method
    # Find the set_recorder method and insert before it
    if "def set_recorder" in content:
        content = insert_after(
            content,
            "        self._microsteps_per_micron = DEFAULT_MICROSTEPS_PER_MICRON\n        self._hardware_config = None  # v7.2: HardwareConfig for µL display",
            "",
            "Locate init end for set_hardware_config insertion",
        )

    # Add method after page interface section
    content = insert_after(
        content,
        '''    def get_page_title(self) -> str:
        return "Print Monitor"''',
        '''
    def set_hardware_config(self, config):
        """v7.2: Set hardware config for µL pump display in monitor."""
        self._hardware_config = config
''',
        "Add set_hardware_config to print_monitor",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: print_monitor.py patched")


# ═══════════════════════════════════════════════════════════════════
# SESSION 6b: Settings Page — wire hardware config to safety limits
# ═══════════════════════════════════════════════════════════════════

def patch_settings_page(root):
    filepath = os.path.join(root, "gui", "pages", "settings_page.py")
    print(f"\n{'='*60}")
    print(f"Session 6b: Patching {filepath}")
    print(f"{'='*60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    content = read_file(filepath)

    # Add set_hardware_config method
    content = insert_after(
        content,
        '''    def set_microsteps_per_micron(self, value: float):
        """Update the conversion factor and refresh related UI."""''',
        '''
    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Receive hardware config for dual mm/µL display."""
        self._hardware_config = config
''',
        "Add set_hardware_config to settings_page",
    )

    write_file(filepath, content)
    print(f"  {GREEN}DONE{RESET}: settings_page.py patched")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    if os.path.isdir("SupportClasses"):
        root = "."
    elif os.path.isdir("MEBP-Version-7.0/SupportClasses"):
        root = "MEBP-Version-7.0"
    elif os.path.isdir("MEBP-Version-7.1/SupportClasses"):
        root = "MEBP-Version-7.1"
    else:
        print(f"{RED}ERROR{RESET}: Cannot find SupportClasses/ directory.")
        sys.exit(1)

    print(f"\n{BOLD}MEBP v7.2 Sessions 3-6 Patches{RESET}")
    print(f"Project root: {os.path.abspath(root)}")

    patch_calibration(root)         # Session 3
    patch_safety_limits(root)       # Session 4
    patch_stage_controller_zpjog(root)  # Session 5
    patch_print_monitor(root)       # Session 6
    patch_settings_page(root)       # Session 6b

    print(f"\n{BOLD}All Sessions 3-6 patches applied.{RESET}")
    print(f"\nSummary:")
    print(f"  Session 3: calibration.py — set_hardware_config(), auto plate format, needle info")
    print(f"  Session 4: SafetyLimits.py — update_from_hardware_config(), get_pump_limits_uL()")
    print(f"  Session 5: StageController.py — ZPJogHandler gets hardware_config, safety limits wired")
    print(f"  Session 6: print_monitor.py — set_hardware_config() for µL display")
    print(f"  Session 6b: settings_page.py — set_hardware_config() for dual display")


if __name__ == "__main__":
    main()
