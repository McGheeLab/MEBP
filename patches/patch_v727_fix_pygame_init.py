#!/usr/bin/env python3
"""
Fix SIGTRAP crash: pygame.init() calls SDL_VideoInit on a background thread,
which crashes on macOS because Cocoa APIs require the main thread.

Solution: Replace pygame.init() with pygame.joystick.init() in the worker.
The worker only needs joystick — not video, display, or keyboard.
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

    marker = "v7.2.7: joystick-only init"
    changed = False

    if marker not in content:
        # Replace ALL occurrences of pygame.init() with joystick-only init
        # inside xbox_polling_worker function
        # Pattern 1: bare pygame.init()
        old1 = "    pygame.init()"
        new1 = (
            f"    # {marker} — never call pygame.init() from a thread on macOS;\n"
            "    # SDL_VideoInit triggers Cocoa APIs that require the main thread.\n"
            "    pygame.joystick.init()"
        )
        if old1 in content:
            content = content.replace(old1, new1)
            changed = True
            print("✓ Replaced pygame.init() with pygame.joystick.init()")

        # Also fix inside _find_controller if it re-inits
        # The _find_controller already does pygame.joystick.quit()/init() which is fine
    else:
        print("○ Already applied")

    # Also fix the duplicate SDL library warning — set env var to prefer pygame's SDL
    sdl_marker = "v7.2.7: suppress cv2 SDL conflict"
    if sdl_marker not in content and "SDL_VIDEODRIVER" not in content:
        # Find the SDL hints block we added earlier
        hints_marker = "v7.2.7: SDL Bluetooth hints"
        if hints_marker in content:
            # Add after existing hints
            old_hints = '_os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")'
            if old_hints in content:
                new_hints = (
                    old_hints + '\n'
                    f'    _os.environ["SDL_VIDEODRIVER"] = "dummy"  # {sdl_marker}\n'
                    '    _os.environ["SDL_AUDIODRIVER"] = "dummy"'
                )
                content = content.replace(old_hints, new_hints)
                changed = True
                print("✓ Added SDL_VIDEODRIVER=dummy to suppress video init")
        else:
            print("○ SDL hints block not found — skipping SDL_VIDEODRIVER")

    if not changed:
        print("Nothing to change")
        return 0

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"✗ AST FAIL: {e}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727init_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"→ XboxController.py written successfully")
    return 0

if __name__ == "__main__":
    sys.exit(main())
