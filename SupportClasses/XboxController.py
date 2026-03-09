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
    v7.2.6: S4 resilient worker — retry loop on startup, crash recovery,
    periodic heartbeat. Worker never exits; reconnects automatically.

    Args:
        out_queue:     Multiprocessing queue for outbound messages.
        mapping_file:  Path to the button mapping JSON file.
        avg_interval:  Seconds between averaged axis updates.
        deadzone:      Axis deadzone threshold (0–1).
    """
    try:
        import pygame
    except ImportError:
        out_queue.put({"debug": "pygame not installed -- Xbox controller unavailable"})
        return

    pygame.init()

    # Load initial mapping
    mapping = load_xbox_mapping(mapping_file)

    def _init_accumulators(js):
        """Build fresh axis accumulators for a joystick."""
        n = js.get_numaxes()
        return (
            {i: 0.0 for i in range(n)},
            {i: 0   for i in range(n)},
        )

    def _find_controller():
        """Retry until a controller is found. Sends status messages."""
        while True:
            pygame.joystick.quit()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                out_queue.put({"debug": f"Controller connected: {js.get_name()}"})
                out_queue.put({"status": "connected"})
                return js
            else:
                out_queue.put({"debug": "No controller found, retrying in 2s..."})
                out_queue.put({"status": "waiting"})
                time.sleep(2.0)

    # ── Initial controller acquisition ────────────────────────────
    joystick = _find_controller()

    axis_accum, axis_count = _init_accumulators(joystick)
    num_axes = joystick.get_numaxes()
    last_axis_time   = time.time()
    last_mapping_time = time.time()
    last_heartbeat   = time.time()
    last_hat = (0, 0)
    last_sent: dict = {}

    axis_groups = [
        {"name": "0-1", "axes": [0, 1], "type": "axis"},
        {"name": "2-3", "axes": [2, 3], "type": "axis"},
        {"name": "4",   "axes": [4],    "type": "trigger"},
        {"name": "5",   "axes": [5],    "type": "trigger"},
    ]

    # ── Main Loop ─────────────────────────────────────────────────
    while True:
        current_time = time.time()

        try:
            pygame.event.pump()

            # Hot-reload mapping every 5 seconds
            if current_time - last_mapping_time >= 5:
                mapping = load_xbox_mapping(mapping_file)
                last_mapping_time = current_time

            # Heartbeat every 3 seconds
            if current_time - last_heartbeat >= 3.0:
                out_queue.put({"status": "alive"})
                last_heartbeat = current_time

            # ── Button Presses ─────────────────────────────────────
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    mapped_func = mapping.get("buttons", {}).get(str(i))
                    if mapped_func and mapped_func != "None":
                        out_queue.put({"button": i, "command": mapped_func})

            # ── Axis Accumulation ──────────────────────────────────
            for i in range(num_axes):
                raw = joystick.get_axis(i)
                if abs(raw) > deadzone:
                    axis_accum[i] += raw
                    axis_count[i] += 1

            if current_time - last_axis_time >= avg_interval:
                for group in axis_groups:
                    axes = group["axes"]
                    cmd = mapping.get("axes", {}).get(group["name"])
                    if not cmd or cmd == "None":
                        for a in axes:
                            axis_accum[a] = 0.0; axis_count[a] = 0
                        continue

                    if group["type"] == "axis":
                        counts = [axis_count[a] for a in axes]
                        total = sum(counts)
                        if total > 0:
                            avgs = [axis_accum[a] / axis_count[a] if axis_count[a] else 0.0
                                    for a in axes]
                        else:
                            avgs = [0.0] * len(axes)
                        avg_val = tuple(avgs)
                    else:
                        a = axes[0]
                        avg_val = axis_accum[a] / axis_count[a] if axis_count[a] else 0.0

                    zero_value = (0.0, 0.0) if group["type"] == "axis" else 0.0
                    prev = last_sent.get(group["name"], zero_value)

                    # Only send if changed or non-zero
                    if avg_val != zero_value or prev != zero_value:
                        out_queue.put({
                            "axis":    group["name"],
                            "average": avg_val,
                            "command": cmd,
                        })
                        last_sent[group["name"]] = avg_val

                    for a in axes:
                        axis_accum[a] = 0.0; axis_count[a] = 0
                last_axis_time = current_time

            # ── D-Pad (Hat) ────────────────────────────────────────
            if joystick.get_numhats() > 0:
                current_hat = joystick.get_hat(0)
                if current_hat != last_hat:
                    dpad_map = mapping.get("dpad", {})
                    if current_hat[1] == 1:
                        cmd = dpad_map.get("up")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "up", "command": cmd})
                    elif current_hat[1] == -1:
                        cmd = dpad_map.get("down")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "down", "command": cmd})
                    if current_hat[0] == 1:
                        cmd = dpad_map.get("right")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "right", "command": cmd})
                    elif current_hat[0] == -1:
                        cmd = dpad_map.get("left")
                        if cmd and cmd != "None":
                            out_queue.put({"dpad": "left", "command": cmd})
                    last_hat = current_hat

            time.sleep(0.02)

        except Exception as e:
            # v7.2.6: Crash recovery — reconnect instead of dying
            out_queue.put({"debug": f"Controller error: {e}"})
            out_queue.put({"status": "disconnected"})
            joystick = _find_controller()
            # Re-init accumulators for the new joystick
            axis_accum, axis_count = _init_accumulators(joystick)
            num_axes = joystick.get_numaxes()
            last_axis_time   = time.time()
            last_mapping_time = time.time()
            last_heartbeat   = time.time()
            last_hat = (0, 0)
            last_sent = {}

