#!/usr/bin/env python3
"""Fix ZPJogHandler._clamp_pump_flow AttributeError for missing _hardware_config."""

import ast, re, shutil, sys
from datetime import datetime
from pathlib import Path

def find_root():
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent,
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit("Cannot find MEBP root")

def main():
    root = find_root()
    path = root / "SupportClasses" / "StageController.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: safe _hardware_config access"
    if marker in content:
        print("Already applied — skipping.")
        return 0

    # Fix 1: Make _clamp_pump_flow use getattr instead of direct access
    old = "    def _clamp_pump_flow(self, vel, pump_id):"
    if old not in content:
        # Try to find it with different formatting
        m = re.search(r'(    def _clamp_pump_flow\(self.*?\):.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
                       content, re.DOTALL)
        if m:
            # Replace entire method with safe version
            new_method = (
                f'    def _clamp_pump_flow(self, vel, pump_id):  # {marker}\n'
                '        """Clamp pump velocity to max safe flow rate if hw config available."""\n'
                '        hw = getattr(self, "_hardware_config", None)\n'
                '        if hw is None:\n'
                '            return vel\n'
                '        sl = getattr(self, "safety_limits", None)\n'
                '        if sl is None:\n'
                '            return vel\n'
                '        try:\n'
                '            max_rate = sl.get_max_flow_rate(pump_id)\n'
                '            if max_rate is not None and max_rate > 0:\n'
                '                # Convert vel to flow rate, clamp, convert back\n'
                '                pump_cfg = hw.pumps.get(pump_id)\n'
                '                if pump_cfg and pump_cfg.is_configured:\n'
                '                    rate = abs(pump_cfg.mm_to_uL(abs(vel)))\n'
                '                    if rate > max_rate:\n'
                '                        clamped_mm = pump_cfg.uL_to_mm(max_rate)\n'
                '                        vel = clamped_mm if vel > 0 else -clamped_mm\n'
                '        except Exception:\n'
                '            pass  # Safe fallback — no clamping if anything fails\n'
                '        return vel\n'
                '\n'
            )
            content = content[:m.start()] + new_method + content[m.end():]
            print("✓ Replaced entire _clamp_pump_flow with safe version")
        else:
            print("✗ _clamp_pump_flow not found")
            return 1
    else:
        # Simple fix: replace just the first line of the method body
        # Find "hw = self._hardware_config" and replace with getattr
        old_hw = "        hw = self._hardware_config"
        if old_hw in content:
            content = content.replace(
                old_hw,
                f"        hw = getattr(self, '_hardware_config', None)  # {marker}",
                1
            )
            print("✓ Fixed _clamp_pump_flow: self._hardware_config → getattr")
        else:
            print("✗ Could not find hw = self._hardware_config")
            return 1

    # Fix 2: Also ensure ZPJogHandler.__init__ initializes _hardware_config = None
    # Find ZPJogHandler class and its __init__
    if "class ZPJogHandler" in content:
        # Check if _hardware_config is already set in ZPJogHandler
        zp_init = re.search(
            r'(class ZPJogHandler.*?def __init__\(self.*?\):.*?\n)(.*?)(?=\n    def )',
            content, re.DOTALL
        )
        if zp_init:
            init_body = zp_init.group(2)
            if '_hardware_config' not in init_body:
                # Find last assignment in __init__ to inject after
                last_assign = re.search(
                    r'(        self\._registered_handlers.*?\])',
                    init_body, re.DOTALL
                )
                if last_assign:
                    inject_pos = zp_init.start(2) + last_assign.end()
                    inject = '\n\n        # v7.2.7: init _hardware_config for _clamp_pump_flow\n        self._hardware_config = None\n'
                    content = content[:inject_pos] + inject + content[inject_pos:]
                    print("✓ Added self._hardware_config = None to ZPJogHandler.__init__")
                else:
                    # Try injecting after the last self. line
                    lines = init_body.split('\n')
                    for i in range(len(lines)-1, -1, -1):
                        if lines[i].strip().startswith('self.'):
                            inject_pos = zp_init.start(2) + sum(len(l)+1 for l in lines[:i+1])
                            inject = '\n        # v7.2.7: init _hardware_config\n        self._hardware_config = None\n'
                            content = content[:inject_pos] + inject + content[inject_pos:]
                            print("✓ Added self._hardware_config = None to ZPJogHandler.__init__")
                            break
            else:
                print("○ ZPJogHandler already has _hardware_config in __init__")

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"✗ AST FAIL: {e}")
        return 1

    # Backup + write
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727clamp_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"→ StageController.py written successfully")
    return 0

if __name__ == "__main__":
    sys.exit(main())
