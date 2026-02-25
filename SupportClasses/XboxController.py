"""
Xbox Controller — Pygame-based Xbox controller polling in a separate process.

Runs as a ``multiprocessing.Process`` target for latency isolation.
Communicates with the main process via a ``multiprocessing.Queue``.

**This code is proven and its core logic should not be changed.**
Only structural cleanup and logging improvements have been applied.

Queue message formats:
    Button press:   {"button": int, "command": str}
    Axis update:    {"axis": str, "average": float|tuple, "command": str}
    D-pad change:   {"dpad": str, "command": str}
    Debug info:     {"debug": str}
"""

from __future__ import annotations

import json
import logging
import time
from multiprocessing import Queue
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


def load_xbox_mapping(mapping_file: str = "current_button_mapping.json") -> dict:
    """
    Load button/axis/dpad mapping from a JSON file.

    Returns:
        Dict with keys "buttons", "axes", "dpad".
        Falls back to empty mapping on error.
    """
    try:
        with open(mapping_file, "r") as f:
            mapping = json.load(f)
        return mapping
    except Exception as e:
        logger.warning(f"Failed to load Xbox mapping from {mapping_file}: {e}")
        return {"buttons": {}, "axes": {}, "dpad": {}}


def xbox_polling_worker(
    out_queue: Queue,
    mapping_file: str = "current_button_mapping.json",
    avg_interval: float = 0.5,
    deadzone: float = 0.2,
) -> None:
    """
    Main polling loop — runs in a separate process.

    Continuously reads the Xbox controller via Pygame and sends events
    to *out_queue*.  The mapping file is hot-reloaded every 5 seconds.

    Args:
        out_queue:     Multiprocessing queue for outbound messages.
        mapping_file:  Path to the button mapping JSON file.
        avg_interval:  Seconds between averaged axis updates.
        deadzone:      Axis deadzone threshold (0–1).
    """
    try:
        import pygame
    except ImportError:
        out_queue.put({"debug": "pygame not installed — Xbox controller unavailable"})
        return

    pygame.init()
    pygame.joystick.init()

    # Load initial mapping
    mapping = load_xbox_mapping(mapping_file)

    # Check for connected controllers
    count = pygame.joystick.get_count()
    out_queue.put({"debug": f"Found {count} joystick(s)."})

    if count == 0:
        out_queue.put({"debug": "No controller connected."})
        return

    # Initialise the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    out_queue.put({
        "debug": f"Controller connected: {joystick.get_name()}",
        "joystick_info": {
            "numbuttons": joystick.get_numbuttons(),
            "numaxes": joystick.get_numaxes(),
            "numhats": joystick.get_numhats(),
        },
    })

    # Axis averaging accumulators
    num_axes = joystick.get_numaxes()
    axis_accum = {i: 0.0 for i in range(num_axes)}
    axis_count = {i: 0 for i in range(num_axes)}
    last_axis_time = time.time()
    last_mapping_time = time.time()
    last_hat = (0, 0)
    last_sent: dict = {}

    # Axis groups define how physical axes are combined and labelled
    axis_groups = [
        {"name": "0-1", "axes": [0, 1], "type": "axis"},      # Left stick
        {"name": "2-3", "axes": [2, 3], "type": "axis"},      # Right stick
        {"name": "4",   "axes": [4],    "type": "trigger"},   # Left trigger
        {"name": "5",   "axes": [5],    "type": "trigger"},   # Right trigger
    ]

    # ── Main Loop ─────────────────────────────────────────────────
    while True:
        current_time = time.time()
        pygame.event.pump()

        # Hot-reload mapping every 5 seconds
        if current_time - last_mapping_time >= 5:
            mapping = load_xbox_mapping(mapping_file)
            last_mapping_time = current_time

        # ── Button Presses ────────────────────────────────────────
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                mapped_cmd = mapping.get("buttons", {}).get(str(i))
                if mapped_cmd and mapped_cmd != "None":
                    out_queue.put({"button": i, "command": mapped_cmd})
                    time.sleep(0.2)  # Debounce

        # ── Accumulate Axis Readings ──────────────────────────────
        for axis_id in range(num_axes):
            val = joystick.get_axis(axis_id)
            axis_accum[axis_id] += val
            axis_count[axis_id] += 1

        # ── Process Averaged Axes ─────────────────────────────────
        if current_time - last_axis_time >= avg_interval:
            for group in axis_groups:
                averages = []
                for axis_id in group["axes"]:
                    if axis_count[axis_id] > 0:
                        avg_val = axis_accum[axis_id] / axis_count[axis_id]
                    else:
                        avg_val = 0.0

                    # Triggers: remap from [-1, 1] to [0, 2] range
                    if group["type"] == "trigger":
                        avg_val += 1.0
                        # Invert left trigger (axis 4)
                        if group["axes"][0] == 4:
                            avg_val = -avg_val

                    averages.append(avg_val)

                # Determine if the movement exceeds deadzone
                is_active = any(abs(v) > deadzone for v in averages)

                mapped_cmd = mapping.get("axes", {}).get(group["name"])
                if not mapped_cmd or mapped_cmd == "None":
                    # Reset accumulators even if no command mapped
                    for axis_id in group["axes"]:
                        axis_accum[axis_id] = 0.0
                        axis_count[axis_id] = 0
                    continue

                # Build the value to send
                if len(averages) == 1:
                    current_value = averages[0]
                    zero_value = 0
                else:
                    current_value = tuple(round(v, 2) for v in averages)
                    zero_value = tuple(0 for _ in group["axes"])

                if is_active:
                    out_queue.put({
                        "axis": group["name"],
                        "average": current_value,
                        "command": mapped_cmd,
                    })
                    last_sent[group["name"]] = current_value
                else:
                    # Send zero when axis returns to neutral
                    prev = last_sent.get(group["name"])
                    if prev is not None and prev != zero_value:
                        out_queue.put({
                            "axis": group["name"],
                            "average": zero_value,
                            "command": mapped_cmd,
                        })
                        last_sent[group["name"]] = zero_value
                    elif prev is None:
                        last_sent[group["name"]] = zero_value

                # Reset accumulators
                for axis_id in group["axes"]:
                    axis_accum[axis_id] = 0.0
                    axis_count[axis_id] = 0

            last_axis_time = current_time

        # ── D-Pad (Hat) ───────────────────────────────────────────
        if joystick.get_numhats() > 0:
            current_hat = joystick.get_hat(0)
            if current_hat != last_hat:
                dpad_map = mapping.get("dpad", {})

                # Vertical: up/down
                if current_hat[1] == 1:
                    cmd = dpad_map.get("up")
                    if cmd and cmd != "None":
                        out_queue.put({"dpad": "up", "command": cmd})
                elif current_hat[1] == -1:
                    cmd = dpad_map.get("down")
                    if cmd and cmd != "None":
                        out_queue.put({"dpad": "down", "command": cmd})

                # Horizontal: left/right
                if current_hat[0] == 1:
                    cmd = dpad_map.get("right")
                    if cmd and cmd != "None":
                        out_queue.put({"dpad": "right", "command": cmd})
                elif current_hat[0] == -1:
                    cmd = dpad_map.get("left")
                    if cmd and cmd != "None":
                        out_queue.put({"dpad": "left", "command": cmd})

                last_hat = current_hat

        # Prevent busy-spin
        time.sleep(0.02)
