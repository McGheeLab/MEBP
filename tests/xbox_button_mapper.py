#!/usr/bin/env python3
"""
Xbox Controller Button Mapper — Press each button/axis to see its index.
Run this to identify the correct mapping for your Xbox Series X via Bluetooth.

Usage: python xbox_button_mapper.py
"""

import os, sys, time

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
        print("No controller found. Make sure it's on and paired.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"\nController: {js.get_name()}")
    print(f"Buttons: {js.get_numbuttons()}")
    print(f"Axes:    {js.get_numaxes()}")
    print(f"Hats:    {js.get_numhats()}")
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

            # Axes
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                prev = last_axes.get(i, 0.0)
                if abs(val) > deadzone and abs(val - prev) > 0.1:
                    direction = ""
                    if i in (0, 2):  # horizontal
                        direction = "RIGHT" if val > 0 else "LEFT"
                    elif i in (1, 3):  # vertical
                        direction = "DOWN" if val > 0 else "UP"
                    elif i in (4, 5):  # triggers
                        direction = f"({val:+.2f})"
                    print(f"  AXIS   {i:2d}  = {val:+.3f}  {direction}")
                last_axes[i] = val

            # Hats (D-pad on some controllers)
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
