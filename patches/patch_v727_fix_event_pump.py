#!/usr/bin/env python3
"""
Fix: Revert to pygame.init() but WITH SDL_VIDEODRIVER=dummy set beforehand.
The dummy video driver doesn't use Cocoa, so it's safe on background threads.
This restores the event system that joystick polling needs.
"""

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
    path = root / "SupportClasses" / "XboxController.py"
    content = path.read_text(encoding="utf-8")
    original = content
    changed = False

    # Step 1: Ensure SDL_VIDEODRIVER=dummy and SDL_AUDIODRIVER=dummy are set
    # BEFORE any pygame init. Check they exist.
    if 'SDL_VIDEODRIVER' not in content:
        # Find the SDL hints block or the import pygame area
        hints = '_os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")'
        if hints in content:
            content = content.replace(hints, 
                hints + '\n'
                '    _os.environ["SDL_VIDEODRIVER"] = "dummy"  # v7.2.7: safe for threads\n'
                '    _os.environ["SDL_AUDIODRIVER"] = "dummy"')
            changed = True
            print("✓ Added SDL_VIDEODRIVER=dummy + SDL_AUDIODRIVER=dummy")
        else:
            print("○ SDL hints block not found")

    # Step 2: Replace pygame.joystick.init() back to pygame.init()
    # The joystick-only init was wrong — we need the event system
    marker_old = "pygame.joystick.init()"
    # Find the line that replaced pygame.init() (inside the function, not in _find_controller)
    # Look for pattern: comment about joystick-only + pygame.joystick.init()
    joystick_only_pattern = re.compile(
        r'(    # v7\.2\.7: joystick-only init.*?\n'
        r'    # SDL_VideoInit triggers.*?\n'
        r'    pygame\.joystick\.init\(\))',
        re.DOTALL
    )
    m = joystick_only_pattern.search(content)
    if m:
        replacement = (
            '    # v7.2.7: Use pygame.init() with SDL_VIDEODRIVER=dummy (set above)\n'
            '    # to get event system without Cocoa main-thread requirement.\n'
            '    pygame.init()'
        )
        content = content[:m.start()] + replacement + content[m.end():]
        changed = True
        print("✓ Reverted pygame.joystick.init() → pygame.init() (with dummy drivers)")
    elif '    pygame.joystick.init()' in content and '    pygame.init()' not in content:
        # Simpler case — just the bare replacement
        # Only replace the first occurrence (the one in the function body, not in _find_controller)
        idx = content.find('    pygame.joystick.init()')
        if idx != -1:
            # Check this isn't inside _find_controller (which legitimately uses joystick.init)
            # The function-level one is the first occurrence
            content = content[:idx] + '    pygame.init()  # v7.2.7: with SDL_VIDEODRIVER=dummy' + content[idx + len('    pygame.joystick.init()'):]
            changed = True
            print("✓ Replaced pygame.joystick.init() → pygame.init()")
    else:
        print("○ pygame.init() already present or pattern not matched")

    if not changed:
        print("Nothing to change")
        return 0

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"✗ AST FAIL: {e}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727evpump_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"→ XboxController.py written successfully")
    return 0

if __name__ == "__main__":
    sys.exit(main())
