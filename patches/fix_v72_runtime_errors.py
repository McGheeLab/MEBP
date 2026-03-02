#!/usr/bin/env python3
"""
fix_v72_runtime_errors.py — Fix three runtime errors after v7.2 patching.

1. SafetyLimits: stroke_mm → stroke_length_mm (SyringeSpec attribute name)
2. StageController: add connect_xy/connect_zp/disconnect_xy/disconnect_zp
3. print_monitor: date_str → timestamp (RecordingInfo field name)

Run from MEBP project root:
    python fix_v72_runtime_errors.py
"""

import os
import sys

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"

fixes_applied = 0
fixes_skipped = 0


def fix_file(path, replacements, label):
    """Apply text replacements to a file."""
    global fixes_applied, fixes_skipped

    if not os.path.isfile(path):
        print(f"  {RED}SKIP{RESET}: {path} not found")
        fixes_skipped += 1
        return

    with open(path) as f:
        content = f.read()

    changed = False
    for old, new, desc in replacements:
        if old in content:
            content = content.replace(old, new, 1)
            print(f"  {GREEN}OK{RESET}: {desc}")
            changed = True
        else:
            if new in content:
                print(f"  {YELLOW}SKIP{RESET}: {desc} (already applied)")
            else:
                print(f"  {YELLOW}SKIP{RESET}: {desc} (text not found)")
                fixes_skipped += 1

    if changed:
        with open(path) as f:
            pass  # verify readable
        with open(path, "w") as f:
            f.write(content)
        fixes_applied += 1


def main():
    global fixes_applied, fixes_skipped

    print(f"\n{BOLD}MEBP v7.2 Runtime Error Fixes{RESET}\n")

    # ── Fix 1: SafetyLimits stroke_mm → stroke_length_mm ─────────
    print(f"{BOLD}Fix 1: SafetyLimits.py — SyringeSpec attribute name{RESET}")
    fix_file("SupportClasses/SafetyLimits.py", [
        (
            "stroke_mm = pump_cfg.syringe.stroke_mm",
            "stroke_mm = pump_cfg.syringe.stroke_length_mm",
            "stroke_mm → stroke_length_mm",
        ),
    ], "SafetyLimits")

    # ── Fix 2: StageController connect_xy/zp methods ─────────────
    print(f"\n{BOLD}Fix 2: StageController.py — connect_xy/zp convenience methods{RESET}")

    path = "SupportClasses/StageController.py"
    if not os.path.isfile(path):
        print(f"  {RED}SKIP{RESET}: {path} not found")
        fixes_skipped += 1
    else:
        with open(path) as f:
            content = f.read()

        if "def connect_xy(self)" in content:
            print(f"  {YELLOW}SKIP{RESET}: connect_xy already exists")
        else:
            # Find the end of connect_stages method
            anchor = "self._pos_poller.set_stages(self.xy_stage, self.zp_stage)"
            if anchor in content:
                # Find end of line after anchor
                idx = content.index(anchor) + len(anchor)
                while idx < len(content) and content[idx] != '\n':
                    idx += 1

                methods = '''

    # ── Convenience connection methods (used by Dashboard) ────────

    def connect_xy(self) -> None:
        """Connect only the XY stage."""
        self.connect_stages(xy=True, zp=False)

    def connect_zp(self) -> None:
        """Connect only the ZP stage."""
        self.connect_stages(xy=False, zp=True)

    def disconnect_xy(self) -> None:
        """Disconnect only the XY stage."""
        if self.xy_jog:
            self.xy_jog.stop()
            self.xy_jog = None
        if self.xy_stage:
            self.xy_stage.stop()
            self.xy_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        logger.info("XY stage disconnected")

    def disconnect_zp(self) -> None:
        """Disconnect only the ZP stage."""
        if self.zp_jog:
            self.zp_jog.stop()
            self.zp_jog = None
        if self.zp_stage:
            self.zp_stage.stop()
            self.zp_stage = None
        self._pos_poller.set_stages(self.xy_stage, self.zp_stage)
        logger.info("ZP stage disconnected")'''

                # Also add is_xy_connected etc. if missing
                if "is_xy_connected" not in content:
                    methods += '''

    @property
    def is_xy_connected(self) -> bool:
        return self.xy_stage is not None

    @property
    def is_zp_connected(self) -> bool:
        return self.zp_stage is not None

    @property
    def is_xbox_connected(self) -> bool:
        return self.xbox_process is not None and self.xbox_process.is_alive()'''

                content = content[:idx] + methods + content[idx:]
                with open(path, "w") as f:
                    f.write(content)
                print(f"  {GREEN}OK{RESET}: Added connect_xy/zp + disconnect_xy/zp")
                fixes_applied += 1
            else:
                print(f"  {RED}FAIL{RESET}: Could not find connect_stages anchor")
                fixes_skipped += 1

    # ── Fix 3: print_monitor date_str → timestamp ─────────────────
    print(f"\n{BOLD}Fix 3: print_monitor.py — RecordingInfo field name{RESET}")
    fix_file("gui/pages/print_monitor.py", [
        (
            ".date_str",
            ".timestamp",
            "date_str → timestamp (RecordingInfo field)",
        ),
    ], "print_monitor")

    # ── Summary ───────────────────────────────────────────────────
    print(f"\n{BOLD}Summary: {fixes_applied} fixes applied, {fixes_skipped} skipped{RESET}")


if __name__ == "__main__":
    main()
