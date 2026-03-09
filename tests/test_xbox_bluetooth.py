#!/usr/bin/env python3
"""
MEBP Xbox Bluetooth Diagnostic — Run this FIRST to verify pygame can see your controller.

Usage:
    python test_xbox_bluetooth.py

This script tests multiple detection strategies to determine why the Xbox
controller might not be visible to pygame on macOS via Bluetooth.
"""

import os
import sys
import time
import platform


def banner(msg):
    print(f"\n{'='*60}")
    print(f"  {msg}")
    print(f"{'='*60}")


def test_system_info():
    banner("System Information")
    print(f"  Python:   {sys.version}")
    print(f"  Platform: {platform.platform()}")
    print(f"  Machine:  {platform.machine()}")
    print(f"  macOS:    {platform.mac_ver()[0] if platform.system() == 'Darwin' else 'N/A'}")

    # Check multiprocessing start method
    import multiprocessing
    print(f"  MP start: {multiprocessing.get_start_method()}")


def test_pygame_import():
    banner("Pygame Import")
    try:
        import pygame
        print(f"  pygame version: {pygame.ver}")
        sdl_ver = pygame.get_sdl_version()
        print(f"  SDL version:    {sdl_ver[0]}.{sdl_ver[1]}.{sdl_ver[2]}")
        return True
    except ImportError as e:
        print(f"  FAIL: pygame not installed — {e}")
        print(f"  Fix:  pip install pygame")
        return False


def test_detection_basic():
    """Test 1: Basic detection without any SDL hints."""
    banner("Test 1: Basic Detection (no SDL hints)")
    import pygame
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    print(f"  Joystick count: {count}")
    if count > 0:
        for i in range(count):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(f"  [{i}] Name:    {js.get_name()}")
            print(f"      Axes:    {js.get_numaxes()}")
            print(f"      Buttons: {js.get_numbuttons()}")
            print(f"      Hats:    {js.get_numhats()}")
    pygame.joystick.quit()
    pygame.quit()
    return count


def test_detection_with_hints():
    """Test 2: Detection with SDL Bluetooth hints."""
    banner("Test 2: Detection with SDL Bluetooth Hints")
    os.environ['SDL_JOYSTICK_HIDAPI'] = '1'
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    print(f"  SDL_JOYSTICK_HIDAPI = {os.environ.get('SDL_JOYSTICK_HIDAPI')}")
    print(f"  SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS = {os.environ.get('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS')}")

    import pygame
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    print(f"  Joystick count: {count}")
    if count > 0:
        for i in range(count):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(f"  [{i}] Name:    {js.get_name()}")
            print(f"      Axes:    {js.get_numaxes()}")
            print(f"      Buttons: {js.get_numbuttons()}")
            print(f"      Hats:    {js.get_numhats()}")
    pygame.joystick.quit()
    pygame.quit()
    return count


def test_detection_with_event_pump():
    """Test 3: Detection with event pump (critical for Bluetooth on macOS)."""
    banner("Test 3: Detection with Event Pump + SDL Hints")
    os.environ['SDL_JOYSTICK_HIDAPI'] = '1'
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'

    import pygame
    pygame.init()

    print("  Pumping events for 3 seconds...")
    found = False
    for attempt in range(15):  # 15 attempts × 0.2s = 3 seconds
        pygame.event.pump()
        pygame.joystick.quit()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count > 0:
            print(f"  Found controller on attempt {attempt + 1}!")
            for i in range(count):
                js = pygame.joystick.Joystick(i)
                js.init()
                print(f"  [{i}] Name:    {js.get_name()}")
                print(f"      Axes:    {js.get_numaxes()}")
                print(f"      Buttons: {js.get_numbuttons()}")
                print(f"      Hats:    {js.get_numhats()}")
            found = True
            break
        time.sleep(0.2)

    if not found:
        print(f"  No controllers found after 3 seconds of event pumping.")

    pygame.joystick.quit()
    pygame.quit()
    return 1 if found else 0


def test_detection_in_subprocess():
    """Test 4: Detection inside a subprocess (simulates the actual MEBP architecture)."""
    banner("Test 4: Detection in Subprocess (like MEBP)")
    from multiprocessing import Process, Queue

    def _worker(q):
        import os as _os
        _os.environ['SDL_JOYSTICK_HIDAPI'] = '1'
        _os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
        try:
            import pygame
            pygame.init()
            for attempt in range(15):
                pygame.event.pump()
                pygame.joystick.quit()
                pygame.joystick.init()
                count = pygame.joystick.get_count()
                if count > 0:
                    js = pygame.joystick.Joystick(0)
                    js.init()
                    q.put({"found": True, "attempt": attempt + 1,
                           "name": js.get_name(),
                           "axes": js.get_numaxes(),
                           "buttons": js.get_numbuttons()})
                    pygame.quit()
                    return
                import time
                time.sleep(0.2)
            q.put({"found": False})
            pygame.quit()
        except Exception as e:
            q.put({"found": False, "error": str(e)})

    q = Queue()
    p = Process(target=_worker, args=(q,), daemon=True)
    p.start()
    p.join(timeout=5.0)

    if not q.empty():
        result = q.get()
        if result.get("found"):
            print(f"  SUCCESS in subprocess on attempt {result['attempt']}!")
            print(f"  Name:    {result['name']}")
            print(f"  Axes:    {result['axes']}")
            print(f"  Buttons: {result['buttons']}")
            return 1
        else:
            err = result.get("error", "")
            print(f"  FAILED: Controller not found in subprocess.{f' Error: {err}' if err else ''}")
            return 0
    else:
        print(f"  FAILED: Worker timed out or crashed.")
        if p.is_alive():
            p.terminate()
        return 0


def test_detection_in_thread():
    """Test 5: Detection inside a thread (fallback strategy)."""
    banner("Test 5: Detection in Thread (fallback)")
    import threading
    from queue import Queue

    def _worker(q):
        import os as _os
        _os.environ['SDL_JOYSTICK_HIDAPI'] = '1'
        _os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
        try:
            import pygame
            pygame.init()
            for attempt in range(15):
                pygame.event.pump()
                pygame.joystick.quit()
                pygame.joystick.init()
                count = pygame.joystick.get_count()
                if count > 0:
                    js = pygame.joystick.Joystick(0)
                    js.init()
                    q.put({"found": True, "attempt": attempt + 1,
                           "name": js.get_name()})
                    # Don't quit pygame here — thread shares process
                    return
                import time
                time.sleep(0.2)
            q.put({"found": False})
        except Exception as e:
            q.put({"found": False, "error": str(e)})

    q = Queue()
    t = threading.Thread(target=_worker, args=(q,), daemon=True)
    t.start()
    t.join(timeout=5.0)

    if not q.empty():
        result = q.get()
        if result.get("found"):
            print(f"  SUCCESS in thread on attempt {result['attempt']}!")
            print(f"  Name: {result['name']}")
            return 1
        else:
            err = result.get("error", "")
            print(f"  FAILED in thread.{f' Error: {err}' if err else ''}")
            return 0
    else:
        print(f"  FAILED: Thread timed out.")
        return 0


def main():
    print("\n" + "=" * 60)
    print("  MEBP Xbox Bluetooth Controller Diagnostic")
    print("  Make sure your controller is ON and paired via Bluetooth")
    print("=" * 60)

    test_system_info()

    if not test_pygame_import():
        return

    results = {}
    results["basic"] = test_detection_basic()
    results["hints"] = test_detection_with_hints()
    results["pump"]  = test_detection_with_event_pump()
    results["subprocess"] = test_detection_in_subprocess()
    results["thread"] = test_detection_in_thread()

    banner("SUMMARY")
    for name, count in results.items():
        status = "PASS" if count > 0 else "FAIL"
        icon = "✓" if count > 0 else "✗"
        print(f"  {icon} {name:12s}: {status} (found {count} controller(s))")

    print()
    if results["subprocess"] > 0:
        print("  → Subprocess detection works! The MEBP patch should fix your issue.")
    elif results["thread"] > 0:
        print("  → Thread detection works but subprocess does NOT.")
        print("    The patch includes a threading fallback that will fix this.")
    elif results["pump"] > 0:
        print("  → Main process detection works but neither subprocess nor thread.")
        print("    The patch includes a threading fallback that should fix this.")
    elif results["hints"] > 0 or results["basic"] > 0:
        print("  → Basic detection works. Apply the patch — it adds the missing pieces.")
    else:
        print("  → pygame cannot detect your controller at all.")
        print("    Possible causes:")
        print("    1. Controller is not paired in macOS Bluetooth settings")
        print("    2. Controller is in pairing mode but not connected")
        print("    3. pygame version doesn't support this controller")
        print("    Try: pip install --upgrade pygame")
        print("    Also verify: System Settings → Bluetooth → controller shows 'Connected'")
    print()


if __name__ == "__main__":
    main()
