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



def calibrate_sticks(
    duration: float = 2.0,
    deadzone: float = 0.05,
) -> dict[str, float]:
    """
    Sample stick axes at rest for *duration* seconds and return center offsets.

    Run this with sticks untouched. Returns a dict like
    ``{0: 0.012, 1: -0.003, 2: 0.008, 3: -0.015}`` mapping axis index to
    its measured center offset.

    v7.3.2
    """
    import os as _os
    import platform as _platform
    _is_macos = _platform.system() == "Darwin"
    _os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")
    _os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    if _is_macos:
        _os.environ["SDL_VIDEODRIVER"] = "dummy"
        _os.environ["SDL_AUDIODRIVER"] = "dummy"

    try:
        import pygame
    except ImportError:
        raise RuntimeError("pygame not installed")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        pygame.quit()
        raise RuntimeError("No controller found")

    js = pygame.joystick.Joystick(0)
    js.init()
    num_axes = js.get_numaxes()
    stick_axes = [i for i in range(min(num_axes, 4))]  # axes 0-3 (sticks)

    sums: dict[int, float] = {a: 0.0 for a in stick_axes}
    counts: dict[int, int] = {a: 0 for a in stick_axes}

    start = time.time()
    while time.time() - start < duration:
        pygame.event.pump()
        for a in stick_axes:
            val = js.get_axis(a)
            sums[a] += val
            counts[a] += 1
        time.sleep(0.02)

    offsets = {}
    for a in stick_axes:
        avg = sums[a] / counts[a] if counts[a] else 0.0
        offsets[a] = avg if abs(avg) > deadzone else 0.0

    pygame.quit()
    return offsets


def xbox_polling_worker(
    out_queue: Queue,
    mapping_file: str = "current_button_mapping.json",
    avg_interval: float = 0.1,
    deadzone: float = 0.2,
    reconnect_timeout: float = 30.0,
    stick_offsets: dict | None = None,
    axis_deadzones: dict | None = None,
    debug_mode: bool = False,
) -> None:
    """
    Main polling loop — runs in a separate process.
    v7.2.6: S4 resilient worker — retry loop on startup, crash recovery,
    periodic heartbeat. Worker never exits; reconnects automatically.
    v7.3.2: stick_offsets — per-axis center offsets to subtract before deadzone.
    v7.3.4: axis_deadzones — per-axis deadzone thresholds (0–1). Keys are
            axis indices. Falls back to ``deadzone`` for unmapped axes.
            Sticks are axes 0–3; triggers are axes 4–5.

    Args:
        out_queue:          Multiprocessing queue for outbound messages.
        mapping_file:       Path to the button mapping JSON file.
        avg_interval:       Seconds between averaged axis updates.
        deadzone:           Fallback deadzone threshold (0–1) for axes not in
                            axis_deadzones.
        reconnect_timeout:  Seconds to attempt reconnection before giving up.
        stick_offsets:      Dict mapping axis index → center offset (v7.3.2).
        axis_deadzones:     Dict mapping axis index → deadzone threshold (v7.3.4).
                            e.g. {0: 0.15, 1: 0.15, 2: 0.15, 3: 0.15,
                                  4: 0.05, 5: 0.05}
    """
    # v7.2.7: SDL Bluetooth hints
    import os as _os
    # v7.2.7: cross-platform: platform-aware SDL configuration
    import platform as _platform
    _is_macos = _platform.system() == "Darwin"
    _os.environ.setdefault("SDL_JOYSTICK_HIDAPI", "1")
    _os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    if _is_macos:
        # macOS: dummy video driver prevents Cocoa main-thread crash
        _os.environ["SDL_VIDEODRIVER"] = "dummy"
        _os.environ["SDL_AUDIODRIVER"] = "dummy"

    try:
        import pygame
    except ImportError:
        out_queue.put({"debug": "pygame not installed -- Xbox controller unavailable"})
        return

    # v7.2.7: Use pygame.init() with SDL_VIDEODRIVER=dummy (set above)
    # to get event system without Cocoa main-thread requirement.
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

    def _find_controller(timeout: float = 0.0, status_key: str = "waiting"):
        """Retry until a controller is found or timeout expires.

        Args:
            timeout:    Max seconds to search. 0 = no limit (initial connect).
            status_key: Status string to send while searching
                        ("waiting" for initial, "reconnecting" after loss).
        Returns:
            Joystick object on success, None if timeout expired.
        """
        _attempt = 0
        _start = time.time()
        while True:
            _attempt += 1
            if timeout > 0 and (time.time() - _start) >= timeout:
                out_queue.put({"debug": f"Reconnect timeout ({timeout:.0f}s) expired"})
                out_queue.put({"status": "disconnected"})
                return None
            pygame.event.pump()  # Critical for Bluetooth on macOS
            pygame.joystick.quit()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                out_queue.put({"debug": f"Controller connected: {js.get_name()} "
                               f"(axes={js.get_numaxes()}, btns={js.get_numbuttons()}, "
                               f"hats={js.get_numhats()}, attempt={_attempt})"})
                out_queue.put({"status": "connected"})
                return js
            else:
                if _attempt <= 3 or _attempt % 10 == 0:
                    out_queue.put({"debug": f"No controller found "
                                   f"(attempt {_attempt}), retrying in 2s..."})
                out_queue.put({"status": status_key})
                time.sleep(2.0)

    # ── Initial controller acquisition ────────────────────────────
    joystick = _find_controller()

    axis_accum, axis_count = _init_accumulators(joystick)
    num_axes = joystick.get_numaxes()
    last_axis_time   = time.time()
    last_mapping_time = time.time()
    last_heartbeat   = time.time()
    last_hat = (0, 0)
    _btn_debounce = {}  # v7.2.7: button debounce: last-fire time per button
    _accum_suppress_until = 0.0  # v7.3.4: suppress axis accumulation after heartbeat reinit
    last_sent: dict = {}

    axis_groups = [
        {"name": "0-1", "axes": [0, 1], "type": "axis"},
        {"name": "2-3", "axes": [2, 3], "type": "axis"},
        {"name": "4",   "axes": [4],    "type": "trigger"},
        {"name": "5",   "axes": [5],    "type": "trigger"},
    ]


    # v7.2.7: trigger normalization
    # Windows Xbox triggers (XInput) rest at -1.0 and travel to +1.0.
    # macOS triggers rest at 0.0 and travel to +1.0 — no normalization needed.
    # v7.3.4: Use platform-based normalization instead of sampling.
    # Sampling is unreliable at startup due to Bluetooth settle latency —
    # early reads often return 0.0 before the driver delivers the true -1.0,
    # causing a partial/wrong offset that lets the rest value pass the deadzone.
    if _is_macos:
        _trigger_offsets = {}  # macOS: already 0..1, no normalization needed
    else:
        # Windows (and Linux XInput): triggers always rest at -1.0
        _trigger_offsets = {ti: -1.0 for ti in [4, 5] if ti < num_axes}

    # v7.3.4: Diagnostic — report actual trigger rest values so we can
    # confirm whether the -1.0 assumption holds on this controller/driver.
    pygame.event.pump()
    time.sleep(0.2)  # brief settle
    pygame.event.pump()
    _diag_vals = {}
    for _ti in [4, 5]:
        if _ti < num_axes:
            _diag_vals[_ti] = round(joystick.get_axis(_ti), 4)
    if debug_mode:
        out_queue.put({"debug":
            f"Trigger rest values (raw, at-rest, BEFORE normalization): {_diag_vals} | "
            f"offsets applied: {_trigger_offsets} | platform: {'macOS' if _is_macos else 'Windows/Linux'}"
        })

    # ── Main Loop ─────────────────────────────────────────────────
    _last_trigger_diag = time.time()  # v7.3.4: periodic trigger diagnostics

    while True:
        current_time = time.time()

        try:
            pygame.event.pump()

            # v7.3.4: Periodic trigger diagnostic (debug mode only).
            if debug_mode and current_time - _last_trigger_diag >= 2.0:
                _per_axis_dz_diag = axis_deadzones or {}
                _trig_report = []
                for _ti in [4, 5]:
                    if _ti < num_axes:
                        _raw = joystick.get_axis(_ti)
                        _norm = _raw
                        if _ti in _trigger_offsets and _trigger_offsets[_ti] < -0.1:
                            _off = _trigger_offsets[_ti]
                            _norm = (_raw - _off) / (1.0 - _off)
                        _dz = _per_axis_dz_diag.get(_ti, deadzone)
                        _passes = abs(_norm) > _dz
                        _trig_report.append(
                            f"ax{_ti}: raw={_raw:.3f} norm={_norm:.3f} "
                            f"dz={_dz:.2f} passes={_passes}"
                        )
                out_queue.put({"debug": "TRIG DIAG | " + " | ".join(_trig_report)})
                _last_trigger_diag = current_time

            # Hot-reload mapping every 5 seconds
            if current_time - last_mapping_time >= 5:
                mapping = load_xbox_mapping(mapping_file)
                last_mapping_time = current_time

            # Heartbeat every 3 seconds — with active presence check
            if current_time - last_heartbeat >= 3.0:
                # Re-enumerate joysticks to detect silent Bluetooth disconnection
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() == 0:
                    raise RuntimeError("Controller lost (no joysticks detected)")
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                out_queue.put({"status": "alive"})
                last_heartbeat = current_time
                # v7.3.4: Suppress accumulation after reinit.
                # pygame.joystick.quit/init causes triggers to transiently
                # report 0.0. With offset=-1.0 applied, 0.0 normalises to 0.5
                # and passes the deadzone. We flush the accumulator AND block
                # axis accumulation for a full window + buffer so every transient
                # read in this iteration AND the settling period is discarded.
                for _a in range(num_axes):
                    axis_accum[_a] = 0.0
                    axis_count[_a] = 0
                last_axis_time = current_time
                _accum_suppress_until = current_time + avg_interval + 0.15

            # ── Button Presses (debounced) ─────────────────
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    # 300ms debounce per button
                    _last = _btn_debounce.get(i, 0)
                    if current_time - _last < 0.3:
                        continue
                    mapped_func = mapping.get("buttons", {}).get(str(i))
                    if mapped_func and mapped_func != "None":
                        out_queue.put({"button": i, "command": mapped_func})
                        _btn_debounce[i] = current_time

            # ── Axis Accumulation ──────────────────────────────────
            # v7.3.4: Skip accumulation during heartbeat-reinit settle window.
            if current_time < _accum_suppress_until:
                time.sleep(0.02)
                continue
            _stick_off = stick_offsets or {}
            _per_axis_dz = axis_deadzones or {}
            for i in range(num_axes):
                raw = joystick.get_axis(i)
                # Normalize triggers: subtract rest offset, remap to 0..1
                # v7.3.4: threshold consistent with detection (< -0.1);
                # span = 1 - offset maps any rest position correctly to 0..1.
                if i in _trigger_offsets and _trigger_offsets[i] < -0.1:
                    _off = _trigger_offsets[i]
                    raw = (raw - _off) / (1.0 - _off)  # rest→0, full-press→1
                # v7.3.2: Subtract stick center offset
                if i in _stick_off:
                    raw -= _stick_off[i]
                # v7.3.4: Per-axis deadzone (falls back to global deadzone)
                _dz = _per_axis_dz.get(i, deadzone)
                if abs(raw) > _dz:
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
                        # v7.2.7: LT retract: LT (axis 4) = retract (negative)
                        if a == 4:
                            avg_val = -avg_val
                        # v7.3.4: Output-level deadzone — clamp averaged trigger
                        # to zero if it doesn't clear the per-axis threshold.
                        _out_dz = _per_axis_dz.get(a, deadzone)
                        if abs(avg_val) <= _out_dz:
                            avg_val = 0.0

                    zero_value = (0.0, 0.0) if group["type"] == "axis" else 0.0
                    prev = last_sent.get(group["name"], zero_value)

                    # Only send if changed or non-zero
                    if avg_val != zero_value or prev != zero_value:
                        # v7.3.4: Log every dispatch so we can trace the source
                        if debug_mode and avg_val != zero_value:
                            out_queue.put({"debug":
                                f"DISPATCH axis={group['name']} avg={avg_val} cmd={cmd}"
                            })
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


            # v7.2.7: dpad-as-buttons
            # D-pad as buttons fallback (Xbox Series X Bluetooth: hats=0)
            if joystick.get_numhats() == 0:
                _dpad_btns = {"11": "up", "12": "down", "13": "left", "14": "right"}
                _hx = (1 if 14 < joystick.get_numbuttons() and joystick.get_button(14) else 0) - \
                      (1 if 13 < joystick.get_numbuttons() and joystick.get_button(13) else 0)
                _hy = (1 if 11 < joystick.get_numbuttons() and joystick.get_button(11) else 0) - \
                      (1 if 12 < joystick.get_numbuttons() and joystick.get_button(12) else 0)
                _hat_now = (_hx, _hy)
                if _hat_now != last_hat:
                    if _hat_now != (0, 0):
                        for _dir, _cond in [("up", _hy>0), ("down", _hy<0), ("left", _hx<0), ("right", _hx>0)]:
                            if _cond:
                                _cmd = mapping.get("dpad", {}).get(_dir)
                                if _cmd and _cmd != "None":
                                    _dlast = _btn_debounce.get(("dpad", _dir), 0)  # v7.2.7: dpad debounce
                                if current_time - _dlast >= 0.3:
                                    out_queue.put({"dpad": _dir, "command": _cmd})
                                    _btn_debounce[("dpad", _dir)] = current_time
                    last_hat = _hat_now

            time.sleep(0.02)

        except Exception as e:
            # v7.2.8: Crash recovery — reconnect with timeout
            out_queue.put({"debug": f"Controller error: {e}"})
            out_queue.put({"status": "reconnecting"})
            joystick = _find_controller(
                timeout=reconnect_timeout, status_key="reconnecting"
            )
            if joystick is None:
                # Timeout expired — give up
                out_queue.put({"debug": "Controller lost — reconnect timed out, stopping worker"})
                break
            # Re-init accumulators for the new joystick
            axis_accum, axis_count = _init_accumulators(joystick)
            num_axes = joystick.get_numaxes()
            last_axis_time   = time.time()
            last_mapping_time = time.time()
            last_heartbeat   = time.time()
            last_hat = (0, 0)
            last_sent = {}

