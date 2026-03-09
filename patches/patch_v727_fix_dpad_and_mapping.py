#!/usr/bin/env python3
"""
Fix Xbox Series X Bluetooth button mapping and D-pad-as-buttons support.

1) Updates current_button_mapping.json with correct Bluetooth button indices
2) Patches XboxController.py to detect D-pad-as-buttons when hats=0
"""

import ast, json, re, shutil, sys
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


def fix_mapping(root):
    """Update current_button_mapping.json for Xbox Series X Bluetooth layout."""
    print(f"\n{GREEN}=== Fix 1: Button Mapping ==={RESET}")
    path = root / "current_button_mapping.json"

    # Xbox Series X via Bluetooth on macOS (pygame 2.6.1, SDL 2.28.4):
    #   A=0, B=1, X=2, Y=3, Back=4, Start=6, LS_down=7, RS_down=8,
    #   LB=9, RB=10, DpadUp=11, DpadDown=12, DpadLeft=13, DpadRight=14, Enter=15
    #
    # Axes: 0=LStickX, 1=LStickY, 2=RStickX, 3=RStickY, 4=LTrigger, 5=RTrigger

    new_mapping = {
        "buttons": {
            "0": "zero_needle_pos",
            "1": "None",
            "2": "None",
            "3": "None",
            "4": "None",
            "5": "None",
            "6": "None",
            "7": "None",
            "8": "None",
            "9": "increment_xyspeed_down",
            "10": "increment_xyspeed_up",
            "11": "increment_zspeed_up",
            "12": "increment_zspeed_down",
            "13": "increment_pspeed_down",
            "14": "increment_pspeed_up",
            "15": "None"
        },
        "axes": {
            "0-1": "move_stage_at_velocity",
            "2-3": "move_z_at_velocity",
            "4": "move_p3_at_velocity",
            "5": "move_p3_at_velocity"
        },
        "dpad": {
            "up": "increment_zspeed_up",
            "down": "increment_zspeed_down",
            "left": "increment_pspeed_down",
            "right": "increment_pspeed_up"
        }
    }

    # Backup existing
    if path.exists():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727map_{ts}"))
        print(f"  Backed up existing mapping")

    path.write_text(json.dumps(new_mapping, indent=4) + "\n", encoding="utf-8")
    print(f"  {GREEN}✓ Written new mapping with Bluetooth button indices{RESET}")
    print(f"    A(0)=zero, LB(9)=XY speed down, RB(10)=XY speed up")
    print(f"    DpadUp(11)=Z speed up, DpadDown(12)=Z speed down")
    print(f"    DpadLeft(13)=P speed down, DpadRight(14)=P speed up")


def fix_dpad_as_buttons(root):
    """Patch XboxController.py to handle D-pad reported as buttons."""
    print(f"\n{GREEN}=== Fix 2: D-pad as Buttons ==={RESET}")
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: dpad-as-buttons"
    if marker in content:
        print(f"  {YELLOW}○ Already applied{RESET}")
        return

    # The current code has a hat-polling section like:
    #   for i in range(joystick.get_numhats()):
    #       hat = joystick.get_hat(i)
    #       ...
    #
    # When hats=0, this loop never executes. We need to add button-based
    # D-pad detection AFTER the button press section.
    #
    # Strategy: Find the button press loop and inject D-pad-as-buttons
    # handling right after it. The D-pad buttons (11-14) will be checked
    # separately from the regular button mapping so they generate dpad
    # events with the correct format.

    # Find the hat polling section and add a fallback before it
    hat_pattern = re.search(
        r'(            # ── (?:D-?[Pp]ad|Hat).*?\n)'
        r'(.*?for i in range\(joystick\.get_numhats\(\)\):)',
        content, re.DOTALL
    )

    if not hat_pattern:
        # Try simpler pattern
        hat_pattern = re.search(
            r'(            for i in range\(joystick\.get_numhats\(\)\):)',
            content
        )

    if hat_pattern:
        inject_pos = hat_pattern.start()
        dpad_code = (
            f'            # {marker}\n'
            '            # Xbox Series X via Bluetooth reports D-pad as buttons, not hat.\n'
            '            # Detect this when numhats==0 and emit dpad events from buttons.\n'
            '            if joystick.get_numhats() == 0:\n'
            '                _dpad_map = mapping.get("_dpad_buttons", {\n'
            '                    "11": "up", "12": "down", "13": "left", "14": "right"\n'
            '                })\n'
            '                _current_dpad = set()\n'
            '                for _btn_str, _dir in _dpad_map.items():\n'
            '                    _btn_idx = int(_btn_str)\n'
            '                    if _btn_idx < joystick.get_numbuttons() and joystick.get_button(_btn_idx):\n'
            '                        _current_dpad.add(_dir)\n'
            '                # Convert to hat-style tuple for compatibility\n'
            '                _hx = (1 if "right" in _current_dpad else 0) - (1 if "left" in _current_dpad else 0)\n'
            '                _hy = (1 if "up" in _current_dpad else 0) - (1 if "down" in _current_dpad else 0)\n'
            '                _hat_now = (_hx, _hy)\n'
            '                if _hat_now != last_hat:\n'
            '                    if _hat_now != (0, 0):\n'
            '                        if _hy > 0:\n'
            '                            _cmd = mapping.get("dpad", {}).get("up")\n'
            '                            if _cmd and _cmd != "None":\n'
            '                                out_queue.put({"dpad": "up", "command": _cmd})\n'
            '                        elif _hy < 0:\n'
            '                            _cmd = mapping.get("dpad", {}).get("down")\n'
            '                            if _cmd and _cmd != "None":\n'
            '                                out_queue.put({"dpad": "down", "command": _cmd})\n'
            '                        if _hx < 0:\n'
            '                            _cmd = mapping.get("dpad", {}).get("left")\n'
            '                            if _cmd and _cmd != "None":\n'
            '                                out_queue.put({"dpad": "left", "command": _cmd})\n'
            '                        elif _hx > 0:\n'
            '                            _cmd = mapping.get("dpad", {}).get("right")\n'
            '                            if _cmd and _cmd != "None":\n'
            '                                out_queue.put({"dpad": "right", "command": _cmd})\n'
            '                    last_hat = _hat_now\n'
            '\n'
        )
        content = content[:inject_pos] + dpad_code + content[inject_pos:]
        print(f"  {GREEN}✓ Injected D-pad-as-buttons handler before hat polling{RESET}")
    else:
        # Alternative: inject at end of main try block, before sleep
        sleep_pattern = re.search(
            r'(            time\.sleep\(0\.02\))',
            content
        )
        if sleep_pattern:
            dpad_code = (
                f'\n            # {marker}\n'
                '            # D-pad as buttons fallback (Xbox Series X Bluetooth: hats=0)\n'
                '            if joystick.get_numhats() == 0:\n'
                '                _dpad_btns = {"11": "up", "12": "down", "13": "left", "14": "right"}\n'
                '                _hx = (1 if 14 < joystick.get_numbuttons() and joystick.get_button(14) else 0) - \\\n'
                '                      (1 if 13 < joystick.get_numbuttons() and joystick.get_button(13) else 0)\n'
                '                _hy = (1 if 11 < joystick.get_numbuttons() and joystick.get_button(11) else 0) - \\\n'
                '                      (1 if 12 < joystick.get_numbuttons() and joystick.get_button(12) else 0)\n'
                '                _hat_now = (_hx, _hy)\n'
                '                if _hat_now != last_hat:\n'
                '                    if _hat_now != (0, 0):\n'
                '                        for _dir, _cond in [("up", _hy>0), ("down", _hy<0), ("left", _hx<0), ("right", _hx>0)]:\n'
                '                            if _cond:\n'
                '                                _cmd = mapping.get("dpad", {}).get(_dir)\n'
                '                                if _cmd and _cmd != "None":\n'
                '                                    out_queue.put({"dpad": _dir, "command": _cmd})\n'
                '                    last_hat = _hat_now\n'
            )
            content = content[:sleep_pattern.start()] + dpad_code + '\n' + content[sleep_pattern.start():]
            print(f"  {GREEN}✓ Injected D-pad-as-buttons handler before sleep{RESET}")
        else:
            print(f"  {RED}✗ Could not find injection point{RESET}")
            return

    # Also: prevent D-pad buttons from firing as regular button events
    # The button loop checks mapping["buttons"]["11"] etc. Since we now handle
    # buttons 11-14 as D-pad, we should skip them in the button loop OR
    # simply set them to "None" in the mapping (which we already do above
    # by not mapping them in buttons... actually we DO map them to speed commands).
    #
    # Wait - the user might want BOTH: D-pad as dpad events AND as button events.
    # Since the mapping already has buttons 11-14 mapped to speed commands,
    # and the dpad section also maps to the same commands, we'd get DOUBLE fires.
    #
    # Solution: Set buttons 11-14 to "None" in the mapping and let D-pad
    # handling take care of them. We already did this in fix_mapping() above
    # ... actually no, we set them to speed commands in buttons too.
    # Let me fix the mapping to NOT have speed commands on buttons 11-14.

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727dpad_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}→ XboxController.py written{RESET}")


def fix_mapping_no_double(root):
    """Remove D-pad buttons from the buttons section to prevent double-firing."""
    print(f"\n{GREEN}=== Fix 3: Prevent Double-Fire ==={RESET}")
    path = root / "current_button_mapping.json"

    # D-pad is handled by the dpad section, so buttons 11-14 should be "None"
    # But we ALSO want them as buttons in case someone uses a USB controller
    # where D-pad IS a hat. So the dpad-as-buttons code only fires when hats=0.
    # When hats>0, buttons 11-14 might not even exist or mean something different.
    #
    # For Bluetooth Xbox Series X (hats=0, 16 buttons):
    #   - Buttons 11-14 ARE the D-pad → set to "None" in buttons, use dpad section
    #
    # The mapping file is hot-reloaded, so this is the user's config.
    # Set buttons 11-14 to "None" since dpad section handles them.

    mapping = json.loads(path.read_text(encoding="utf-8"))
    changed = False
    for btn in ["11", "12", "13", "14"]:
        if mapping.get("buttons", {}).get(btn, "None") != "None":
            mapping["buttons"][btn] = "None"
            changed = True

    if changed:
        path.write_text(json.dumps(mapping, indent=4) + "\n", encoding="utf-8")
        print(f"  {GREEN}✓ Buttons 11-14 set to None (handled by dpad section){RESET}")
    else:
        print(f"  {YELLOW}○ Already correct{RESET}")


def main():
    root = find_root()
    print(f"Project root: {root}")

    fix_mapping(root)
    fix_dpad_as_buttons(root)
    fix_mapping_no_double(root)

    print(f"\n{GREEN}All done!{RESET}")
    print(f"\nButton mapping summary:")
    print(f"  A  (0)  = Zero needle position")
    print(f"  LB (9)  = XY speed DOWN")
    print(f"  RB (10) = XY speed UP")
    print(f"  D-Up    = Z speed UP    (via dpad handler)")
    print(f"  D-Down  = Z speed DOWN  (via dpad handler)")
    print(f"  D-Left  = P speed DOWN  (via dpad handler)")
    print(f"  D-Right = P speed UP    (via dpad handler)")
    print(f"\nAxis mapping (unchanged):")
    print(f"  Left stick  (0-1) = XY stage jog")
    print(f"  Right stick (2-3) = Z needle jog")
    print(f"  L Trigger   (4)   = Pump 3 jog")
    print(f"  R Trigger   (5)   = Pump 3 jog")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
