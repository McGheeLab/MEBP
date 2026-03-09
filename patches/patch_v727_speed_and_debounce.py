#!/usr/bin/env python3
"""
MEBP v7.2.7 — Speed decade increments + button debounce.

Changes:
  1) XYJogHandler: speed ×10/÷10, range 1–10000
  2) ZPJogHandler: z_speed and p_speed ×10/÷10, range 0.01–100
  3) XboxController: 300ms per-button debounce
"""

import ast, re, shutil, sys
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"

def find_root():
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent,
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit("Cannot find MEBP root")


def find_method(content, name, indent=4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def patch_stage_controller(root):
    print(f"\n{GREEN}=== StageController.py — Decade Speed Increments ==={RESET}")
    path = root / "SupportClasses" / "StageController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: decade speed"

    if marker in content:
        print(f"  {YELLOW}○ Already applied{RESET}")
        return

    # ── Fix 1: XYJogHandler._incr_up/down → ×10/÷10 ─────────────

    m = find_method(content, "_incr_up")
    if m:
        # Check this is inside XYJogHandler (look backwards for class name)
        before = content[:m.start()]
        if "class XYJogHandler" in before[before.rfind("class "):]:
            new_method = (
                f'    def _incr_up(self, *a, **kw):  # {marker}\n'
                '        self.xy_speed = min(self.xy_speed * 10, 10000)\n'
                '        logger.info(f"XY speed: {self.xy_speed}")\n'
                '\n'
            )
            content = content[:m.start()] + new_method + content[m.end():]
            print(f"  {GREEN}✓ XY _incr_up: ×2 → ×10{RESET}")
        else:
            print(f"  {YELLOW}○ _incr_up found but not in XYJogHandler{RESET}")
    else:
        print(f"  {RED}✗ _incr_up not found{RESET}")

    # Re-find after content changed
    m = find_method(content, "_incr_down")
    if m:
        before = content[:m.start()]
        if "class XYJogHandler" in before[before.rfind("class "):]:
            new_method = (
                f'    def _incr_down(self, *a, **kw):  # {marker}\n'
                '        self.xy_speed = max(self.xy_speed / 10, 1)\n'
                '        logger.info(f"XY speed: {self.xy_speed}")\n'
                '\n'
            )
            content = content[:m.start()] + new_method + content[m.end():]
            print(f"  {GREEN}✓ XY _incr_down: ÷2 → ÷10{RESET}")
    else:
        print(f"  {RED}✗ _incr_down not found{RESET}")

    # ── Fix 2: ZPJogHandler speed increments → ×10/÷10 ───────────

    # Z speed up
    m = find_method(content, "_incr_z_up")
    if m:
        new_method = (
            f'    def _incr_z_up(self, *a, **kw):  # {marker}\n'
            '        self.z_speed = min(self.z_speed * 10, 100)\n'
            '        logger.info(f"Z speed: {self.z_speed}")\n'
            '\n'
        )
        content = content[:m.start()] + new_method + content[m.end():]
        print(f"  {GREEN}✓ Z _incr_z_up: ×2 → ×10, max=100{RESET}")

    # Z speed down
    m = find_method(content, "_incr_z_down")
    if m:
        new_method = (
            f'    def _incr_z_down(self, *a, **kw):  # {marker}\n'
            '        self.z_speed = max(self.z_speed / 10, 0.01)\n'
            '        logger.info(f"Z speed: {self.z_speed}")\n'
            '\n'
        )
        content = content[:m.start()] + new_method + content[m.end():]
        print(f"  {GREEN}✓ Z _incr_z_down: ÷2 → ÷10, min=0.01{RESET}")

    # P speed up
    m = find_method(content, "_incr_p_up")
    if m:
        new_method = (
            f'    def _incr_p_up(self, *a, **kw):  # {marker}\n'
            '        self.p_speed = min(self.p_speed * 10, 100)\n'
            '        logger.info(f"P speed: {self.p_speed}")\n'
            '\n'
        )
        content = content[:m.start()] + new_method + content[m.end():]
        print(f"  {GREEN}✓ P _incr_p_up: ×2 → ×10, max=100{RESET}")

    # P speed down
    m = find_method(content, "_incr_p_down")
    if m:
        new_method = (
            f'    def _incr_p_down(self, *a, **kw):  # {marker}\n'
            '        self.p_speed = max(self.p_speed / 10, 0.01)\n'
            '        logger.info(f"P speed: {self.p_speed}")\n'
            '\n'
        )
        content = content[:m.start()] + new_method + content[m.end():]
        print(f"  {GREEN}✓ P _incr_p_down: ÷2 → ÷10, min=0.01{RESET}")

    # ── Fix 3: Set sensible default speeds ────────────────────────
    # XY default: 100 (stick full = 100 µm/s — fine positioning range)
    # Z default: 1.0 (1 mm/s at full stick)
    # P default: 0.5 (moderate pump speed)

    # XY default is already 100, good.

    # Z default — find and replace
    z_default = re.search(r'(self\.z_speed:\s*float\s*=\s*)[\d.]+', content)
    if z_default:
        old_val = z_default.group(0)
        new_val = z_default.group(1) + '1.0'
        if old_val != new_val:
            content = content.replace(old_val, new_val, 1)
            print(f"  {GREEN}✓ Z default speed: → 1.0{RESET}")

    # P default
    p_default = re.search(r'(self\.p_speed:\s*float\s*=\s*)[\d.]+', content)
    if p_default:
        old_val = p_default.group(0)
        new_val = p_default.group(1) + '0.5'
        if old_val != new_val:
            content = content.replace(old_val, new_val, 1)
            print(f"  {GREEN}✓ P default speed: → 0.5{RESET}")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}✗ AST FAIL: {e}{RESET}")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727spd_{ts}"))
        path.write_text(content, encoding="utf-8")
        print(f"  {GREEN}→ StageController.py written{RESET}")
    else:
        print(f"  {YELLOW}→ No changes needed{RESET}")


def patch_xbox_debounce(root):
    print(f"\n{GREEN}=== XboxController.py — Button Debounce ==={RESET}")
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: button debounce"

    if marker in content:
        print(f"  {YELLOW}○ Already applied{RESET}")
        return

    # Strategy: Find the button press section in the main loop.
    # Currently it's:
    #     for i in range(joystick.get_numbuttons()):
    #         if joystick.get_button(i):
    #             mapped_func = mapping.get("buttons", {}).get(str(i))
    #             if mapped_func and mapped_func != "None":
    #                 out_queue.put({"button": i, "command": mapped_func})
    #
    # Add a debounce dict + 300ms cooldown per button.

    # Find the button loop
    btn_pattern = re.search(
        r'(            for i in range\(joystick\.get_numbuttons\(\)\):\s*\n)'
        r'(                if joystick\.get_button\(i\):)',
        content
    )

    if btn_pattern:
        # We need to:
        # 1. Add debounce state dict before the main loop
        # 2. Replace the button section with debounced version

        # Step 1: Add debounce state near last_hat or last_sent
        # Find "last_hat = (0, 0)" and add after it
        last_hat_line = re.search(r'(    last_hat = \(0, 0\)\n)', content)
        if last_hat_line:
            inject = f'    _btn_debounce = {{}}  # {marker}: last-fire time per button\n'
            content = (content[:last_hat_line.end()] + inject +
                      content[last_hat_line.end():])
            print(f"  {GREEN}✓ Added _btn_debounce dict{RESET}")
        else:
            # Try alternate location - before main loop
            main_loop = re.search(r'(    # ── Main Loop)', content)
            if main_loop:
                inject = f'\n    _btn_debounce = {{}}  # {marker}\n'
                content = content[:main_loop.start()] + inject + content[main_loop.start():]
                print(f"  {GREEN}✓ Added _btn_debounce dict (before main loop){RESET}")
            else:
                print(f"  {RED}✗ Cannot find injection point for debounce dict{RESET}")
                return

        # Step 2: Replace the button handling section
        # Re-find pattern after injection shifted positions
        btn_pattern2 = re.search(
            r'(            # ── Button Presses.*?\n)?'
            r'(            for i in range\(joystick\.get_numbuttons\(\)\):\s*\n'
            r'                if joystick\.get_button\(i\):\s*\n'
            r'                    mapped_func = mapping\.get\("buttons?".*?\n'
            r'                    if mapped_func and mapped_func != "None":\s*\n'
            r'                        out_queue\.put\(\{"button": i, "command": mapped_func\}\))',
            content, re.DOTALL
        )

        if btn_pattern2:
            new_btn_section = (
                '            # ── Button Presses (debounced) ─────────────────\n'
                '            for i in range(joystick.get_numbuttons()):\n'
                '                if joystick.get_button(i):\n'
                '                    # 300ms debounce per button\n'
                '                    _last = _btn_debounce.get(i, 0)\n'
                '                    if current_time - _last < 0.3:\n'
                '                        continue\n'
                '                    mapped_func = mapping.get("buttons", {}).get(str(i))\n'
                '                    if mapped_func and mapped_func != "None":\n'
                '                        out_queue.put({"button": i, "command": mapped_func})\n'
                '                        _btn_debounce[i] = current_time'
            )
            content = content[:btn_pattern2.start()] + new_btn_section + content[btn_pattern2.end():]
            print(f"  {GREEN}✓ Button loop replaced with 300ms debounce{RESET}")
        else:
            print(f"  {RED}✗ Could not match full button loop for replacement{RESET}")
            # Fallback: simple string replacement
            old_check = '                if joystick.get_button(i):'
            new_check = (
                '                if joystick.get_button(i):\n'
                '                    _last = _btn_debounce.get(i, 0)\n'
                '                    if current_time - _last < 0.3:\n'
                '                        continue'
            )
            if old_check in content:
                content = content.replace(old_check, new_check, 1)
                # Also add the timestamp update after the put
                old_put = '                        out_queue.put({"button": i, "command": mapped_func})'
                new_put = (
                    '                        out_queue.put({"button": i, "command": mapped_func})\n'
                    '                        _btn_debounce[i] = current_time'
                )
                content = content.replace(old_put, new_put, 1)
                print(f"  {GREEN}✓ Button debounce injected (fallback method){RESET}")
            else:
                print(f"  {RED}✗ Could not inject debounce{RESET}")

    else:
        print(f"  {RED}✗ Button press loop not found{RESET}")

    # ── Also debounce D-pad-as-buttons ────────────────────────────
    # The dpad-as-buttons code fires on every poll cycle while held.
    # Add the same debounce — but dpad is directional, so debounce per direction.
    dpad_marker = "v7.2.7: dpad debounce"
    if dpad_marker not in content and "v7.2.7: dpad-as-buttons" in content:
        # Find where dpad events are put on queue and add debounce
        old_dpad_put = "out_queue.put({\"dpad\": _dir, \"command\": _cmd})"
        if old_dpad_put in content:
            new_dpad_put = (
                f'_dlast = _btn_debounce.get(("dpad", _dir), 0)  # {dpad_marker}\n'
                '                                if current_time - _dlast >= 0.3:\n'
                '                                    out_queue.put({"dpad": _dir, "command": _cmd})\n'
                '                                    _btn_debounce[("dpad", _dir)] = current_time'
            )
            content = content.replace(old_dpad_put, new_dpad_put)
            print(f"  {GREEN}✓ D-pad debounce added{RESET}")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}✗ AST FAIL: {e}{RESET}")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727dbnc_{ts}"))
        path.write_text(content, encoding="utf-8")
        print(f"  {GREEN}→ XboxController.py written{RESET}")
    else:
        print(f"  {YELLOW}→ No changes needed{RESET}")


def main():
    root = find_root()
    print(f"Project root: {root}")

    patch_stage_controller(root)
    patch_xbox_debounce(root)

    print(f"\n{GREEN}Summary:{RESET}")
    print(f"  Speed increments now ×10/÷10 (was ×2/÷2)")
    print(f"  XY speed range: 1 → 10 → 100 → 1000 → 10000 µm/s")
    print(f"  Z  speed range: 0.01 → 0.1 → 1.0 → 10 → 100 mm/s")
    print(f"  P  speed range: 0.01 → 0.1 → 0.5 → 5.0 → 50 → 100")
    print(f"  Stick 0→1 gives fine control within each decade")
    print(f"  Buttons debounced at 300ms (no more rapid-fire)")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
