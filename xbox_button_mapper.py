#!/usr/bin/env python3
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

    print(f"\nOS:       {platform.system()} {platform.release()}")
    print(f"pygame:   {pygame.ver}")
    sdl = pygame.get_sdl_version()
    print(f"SDL:      {sdl[0]}.{sdl[1]}.{sdl[2]}")
    print(f"\nController: {js.get_name()}")
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

    print(f"\n{'='*60}")
    print("Press buttons, move sticks, and use D-pad.")
    print("Press Ctrl+C to quit.")
    print(f"{'='*60}\n")

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
        print("\n\nDone!")

    pygame.quit()


if __name__ == "__main__":
    main()
