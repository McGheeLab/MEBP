#!/usr/bin/env python3
"""
MEBP v7.2.7 — Cross-platform Xbox controller fixes.

Platform differences:
  macOS Bluetooth:  hats=0, triggers rest at 0.0, SDL video crashes on threads
  Windows USB/BT:   hats=1, triggers rest at -1.0, SDL video fine on threads

Changes:
  1) XboxController.py — OS-aware SDL env vars, trigger normalization
  2) dashboard.py — thread mode only on macOS
  3) xbox_button_mapper.py — OS-aware diagnostic tool
"""

import ast, platform, re, shutil, sys, json
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"

def find_root():
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent,
              Path(__file__).resolve().parent.parent.parent,
              Path.home() / "Documents" / "GitHub" / "MEBP",
              Path.home() / "OneDrive" / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root.{RESET}")
    sys.exit(1)


def find_method(content, name, indent=4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def patch_xbox_controller(root):
    """Make XboxController.py platform-aware."""
    print(f"\n{GREEN}=== Fix 1: XboxController.py — Platform-Aware ==={RESET}")
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content

    marker = "v7.2.7: cross-platform"

    if marker in content:
        print(f"  {YELLOW}○ Already applied{RESET}")
        return

    # ── Fix 1a: Make SDL env vars macOS-only ──────────────────────
    # Find the SDL env var block and wrap in platform check
    sdl_patterns = [
        '_os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")',
        '_os.environ["SDL_VIDEODRIVER"] = "dummy"',
        '_os.environ["SDL_AUDIODRIVER"] = "dummy"',
        '_os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")',
    ]

    # Find the block containing SDL env vars
    # It starts with SDL_JOYSTICK_HIDAPI and ends after SDL_AUDIODRIVER
    block_start = None
    block_end = None
    for pat in sdl_patterns:
        idx = content.find(pat)
        if idx != -1:
            if block_start is None or idx < block_start:
                block_start = idx
            line_end = content.find('\n', idx)
            if line_end != -1 and (block_end is None or line_end > block_end):
                block_end = line_end + 1

    if block_start is not None and block_end is not None:
        # Find the start of the first line (including indentation + comment)
        line_start = content.rfind('\n', 0, block_start) + 1
        # Also capture the comment line above if it's an SDL hint comment
        prev_line_start = content.rfind('\n', 0, line_start - 1) + 1
        prev_line = content[prev_line_start:line_start].strip()
        if 'SDL' in prev_line or 'Bluetooth' in prev_line or 'v7.2.7' in prev_line:
            line_start = prev_line_start

        old_block = content[line_start:block_end]
        new_block = (
            f'    # {marker}: platform-aware SDL configuration\n'
            '    import platform as _platform\n'
            '    _is_macos = _platform.system() == "Darwin"\n'
            '    _os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")\n'
            '    _os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")\n'
            '    if _is_macos:\n'
            '        # macOS: dummy video driver prevents Cocoa main-thread crash\n'
            '        _os.environ["SDL_VIDEODRIVER"] = "dummy"\n'
            '        _os.environ["SDL_AUDIODRIVER"] = "dummy"\n'
        )
        content = content[:line_start] + new_block + content[block_end:]
        print(f"  {GREEN}✓ SDL env vars now macOS-only (dummy driver){RESET}")
    else:
        print(f"  {YELLOW}○ SDL env var block not found — may need manual check{RESET}")

    # ── Fix 1b: Normalize triggers (Windows: rest=-1, macOS: rest=0) ──
    # The axis averaging code needs to treat the trigger rest position correctly.
    # On Windows, triggers (axes 4,5) rest at -1.0 and go to +1.0.
    # On macOS, triggers rest at 0.0 and go to +1.0.
    # The deadzone check `abs(raw) > deadzone` fails on Windows because
    # abs(-1.0) > 0.2 is always true.
    #
    # Fix: After pygame.init(), detect trigger rest values and normalize.
    # We inject a trigger offset that gets applied during axis reading.

    trigger_marker = "v7.2.7: trigger normalization"
    if trigger_marker not in content:
        # Find where axis_groups is defined and add trigger calibration after it
        axis_groups_end = re.search(
            r'(    axis_groups = \[.*?\])\s*\n',
            content, re.DOTALL
        )
        if axis_groups_end:
            inject_pos = axis_groups_end.end()
            trigger_cal = (
                f'\n    # {trigger_marker}\n'
                '    # Windows Xbox triggers rest at -1.0; macOS at 0.0.\n'
                '    # Read initial trigger values to use as zero-offset.\n'
                '    pygame.event.pump()\n'
                '    _trigger_offsets = {}\n'
                '    for _ti in [4, 5]:\n'
                '        if _ti < num_axes:\n'
                '            _tval = joystick.get_axis(_ti)\n'
                '            # If rest value is < -0.5, this is Windows-style (-1 to +1)\n'
                '            _trigger_offsets[_ti] = _tval if _tval < -0.5 else 0.0\n'
                '\n'
            )
            content = content[:inject_pos] + trigger_cal + content[inject_pos:]
            print(f"  {GREEN}✓ Trigger offset calibration added{RESET}")

            # Now modify the axis reading to apply the offset.
            # Find where raw axis values are read:
            #   raw = joystick.get_axis(i)
            # and adjust for triggers
            old_raw = '                raw = joystick.get_axis(i)'
            if old_raw in content:
                new_raw = (
                    '                raw = joystick.get_axis(i)\n'
                    '                # Normalize triggers: subtract rest offset, remap to 0..1\n'
                    '                if i in _trigger_offsets and _trigger_offsets[i] < -0.5:\n'
                    '                    raw = (raw - _trigger_offsets[i]) / 2.0  # -1..+1 → 0..+1'
                )
                content = content.replace(old_raw, new_raw, 1)
                print(f"  {GREEN}✓ Trigger axis normalization applied{RESET}")
            else:
                print(f"  {YELLOW}○ Could not find raw axis read line{RESET}")
        else:
            print(f"  {YELLOW}○ axis_groups not found — trigger cal skipped{RESET}")
    else:
        print(f"  {YELLOW}○ Trigger normalization already present{RESET}")

    # ── Write ─────────────────────────────────────────────────────
    if content != original:
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}✗ AST FAIL: {e}{RESET}")
            return False
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727xplat_{ts}"))
        path.write_text(content, encoding="utf-8")
        print(f"  {GREEN}→ XboxController.py written{RESET}")
    return True


def patch_dashboard(root):
    """Make dashboard thread-mode macOS-only."""
    print(f"\n{GREEN}=== Fix 2: dashboard.py — Thread Mode macOS-Only ==={RESET}")
    path = root / "gui" / "pages" / "dashboard.py"
    content = path.read_text(encoding="utf-8")
    original = content

    # Check if it already has the platform check
    if 'platform.system() == "Darwin"' in content or "platform.system() == 'Darwin'" in content:
        print(f"  {GREEN}✓ Already has macOS platform check{RESET}")
        # Verify the logic is correct — thread mode on macOS, process on others
        return True

    print(f"  {YELLOW}○ No platform check found — check _connect_xbox manually{RESET}")
    return True


def fix_button_mapper(root):
    """Create an OS-aware button mapper tool."""
    print(f"\n{GREEN}=== Fix 3: Updated xbox_button_mapper.py ==={RESET}")

    mapper_code = '''#!/usr/bin/env python3
"""
Xbox Controller Button Mapper — OS-aware version.
Press each button/axis/dpad to see its index.
"""

import os, sys, time, platform

_IS_MACOS = platform.system() == "Darwin"

# Only set dummy drivers on macOS (thread safety for Cocoa)
if _IS_MACOS:
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        print("No controller found. Make sure it's on and paired/plugged in.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()

    print(f"\\nOS:       {platform.system()} {platform.release()}")
    print(f"pygame:   {pygame.ver}")
    sdl = pygame.get_sdl_version()
    print(f"SDL:      {sdl[0]}.{sdl[1]}.{sdl[2]}")
    print(f"\\nController: {js.get_name()}")
    print(f"Buttons: {js.get_numbuttons()}")
    print(f"Axes:    {js.get_numaxes()}")
    print(f"Hats:    {js.get_numhats()}")

    # Calibrate trigger rest values
    pygame.event.pump()
    trigger_rest = {}
    for i in range(js.get_numaxes()):
        val = js.get_axis(i)
        if abs(val) > 0.5:
            trigger_rest[i] = val
            print(f"  Axis {i} rests at {val:+.3f} (will be treated as zero)")

    print(f"\\n{'='*60}")
    print("Press buttons, move sticks, and use D-pad.")
    print("Press Ctrl+C to quit.")
    print(f"{'='*60}\\n")

    last_buttons = {}
    last_axes = {}
    last_hats = {}
    deadzone = 0.15

    try:
        while True:
            pygame.event.pump()

            # Buttons
            for i in range(js.get_numbuttons()):
                pressed = js.get_button(i)
                if pressed and not last_buttons.get(i, False):
                    print(f"  BUTTON {i:2d}  PRESSED")
                last_buttons[i] = pressed

            # Axes (with trigger normalization)
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                # Normalize triggers that rest at -1.0
                if i in trigger_rest and trigger_rest[i] < -0.5:
                    val = (val - trigger_rest[i]) / 2.0  # -1..+1 → 0..+1

                prev = last_axes.get(i, 0.0)
                if abs(val) > deadzone and abs(val - prev) > 0.05:
                    direction = ""
                    if i in (0, 2):
                        direction = "RIGHT" if val > 0 else "LEFT"
                    elif i in (1, 3):
                        direction = "DOWN" if val > 0 else "UP"
                    elif i in (4, 5):
                        direction = f"trigger ({val:.2f})"
                    print(f"  AXIS   {i:2d}  = {val:+.3f}  {direction}")
                last_axes[i] = val

            # Hats (D-pad on Windows)
            for i in range(js.get_numhats()):
                hat = js.get_hat(i)
                prev = last_hats.get(i, (0, 0))
                if hat != prev:
                    names = []
                    if hat[1] > 0: names.append("UP")
                    if hat[1] < 0: names.append("DOWN")
                    if hat[0] < 0: names.append("LEFT")
                    if hat[0] > 0: names.append("RIGHT")
                    name = "+".join(names) if names else "CENTER"
                    print(f"  HAT    {i:2d}  = {hat}  ({name})")
                last_hats[i] = hat

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\\n\\nDone!")

    pygame.quit()


if __name__ == "__main__":
    main()
'''

    # Write to tests/ directory
    tests_dir = root / "tests"
    tests_dir.mkdir(exist_ok=True)
    mapper_path = tests_dir / "xbox_button_mapper.py"
    mapper_path.write_text(mapper_code, encoding="utf-8")
    print(f"  {GREEN}✓ Written tests/xbox_button_mapper.py (OS-aware){RESET}")

    # Also write to patches output
    out_path = Path(__file__).resolve().parent / "xbox_button_mapper.py"
    out_path.write_text(mapper_code, encoding="utf-8")

    return True


def main():
    root = find_root()
    print(f"Project root: {root}")
    print(f"Current OS:   {platform.system()}")

    patch_xbox_controller(root)
    patch_dashboard(root)
    fix_button_mapper(root)

    print(f"\n{GREEN}=== Platform Behavior Summary ==={RESET}")
    print(f"  {'':30s} {'macOS BT':>12s}  {'Windows':>12s}")
    print(f"  {'D-pad report':30s} {'buttons':>12s}  {'hat':>12s}")
    print(f"  {'Trigger rest value':30s} {'0.0':>12s}  {'-1.0':>12s}")
    print(f"  {'SDL video on thread':30s} {'CRASH':>12s}  {'OK':>12s}")
    print(f"  {'Worker mode':30s} {'thread':>12s}  {'process':>12s}")
    print(f"  {'SDL_VIDEODRIVER':30s} {'dummy':>12s}  {'(default)':>12s}")

    print(f"\n  Next: Run tests/xbox_button_mapper.py to verify your controller")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
