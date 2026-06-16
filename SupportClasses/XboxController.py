"""
Xbox Controller — Pygame-based Xbox controller polling in a separate process.

Runs as a ``multiprocessing.Process`` target for latency isolation.
Communicates with the main process via a ``multiprocessing.Queue``.

v7.5.x input-pipeline overhaul (MEBP_v75x_XBOX_INPUT_PIPELINE):
  - The steady-state heartbeat no longer tears down the SDL joystick
    subsystem. The v7.2.8 ``pygame.joystick.quit()/init()`` re-enumeration
    wiped per-device axis state every 3 s; an idle device never re-reports,
    so triggers read 0.0 indefinitely and the Windows -1.0 rest offset
    manufactured a phantom half-press (verified empirically on real HW).
    Device loss is now detected via SDL2 hot-plug events (JOYDEVICEREMOVED)
    plus a non-destructive ``get_count()`` check at each heartbeat.
  - Trigger axes are "armed" only after their first JOYAXISMOTION event for
    the current joystick object; until then they are treated as at-rest
    (their zero-initialized SDL state is not trustworthy).
  - On any controller loss/exception, zero commands are dispatched for every
    active axis group before reconnecting, so the stages stop immediately.
  - Axis groups of the same type mapped to the same command dispatch ONE
    combined value (e.g. triggers: RT − LT) instead of two competing
    messages; a group that goes inactive mid-hold gets a final zero-send.
  - Raw per-axis state for the GUI live monitor is published via "monitor"
    messages, decoupled from control dispatch.
  - Optional ``ctrl_queue`` accepts live tuning updates (deadzones, stick
    offsets, debug mode) without reconnecting; optional ``stop_event``
    allows a clean shutdown in both process and thread mode.

The ``_find_controller`` quit/init retry loop is the proven macOS-Bluetooth
detection idiom (v7.2.7) and is intentionally retained for initial connect
and reconnect — only the steady-state path stopped re-initializing.

Queue message formats:
    Button press:   {"button": int, "command": str}
    Axis update:    {"axis": str, "average": float|tuple, "command": str}
    D-pad change:   {"dpad": str, "command": str}
    Monitor state:  {"monitor": {"sticks": {0..3: raw [-1,1]},
                                 "triggers": {4,5: pull [0,1]}}}
    Debug info:     {"debug": str}
    Status:         {"status": "waiting"|"connected"|"alive"|
                               "reconnecting"|"disconnected"}
"""

from __future__ import annotations

import json
import logging
import time
from multiprocessing import Queue
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


def load_xbox_mapping(mapping_file: str = "current_button_mapping.json",
                      fallback: dict | None = None) -> dict:
    """
    Load button/axis/dpad mapping from a JSON file.

    Args:
        mapping_file: Path to the mapping JSON.
        fallback:     v7.5.x — value returned on read/parse failure. The
                      worker passes its current mapping so a transient
                      file-write race with the editor's save (hot-reload
                      every 5 s) can no longer silently unmap every input.

    Returns:
        Dict with keys "buttons", "axes", "dpad".
        Falls back to *fallback* (or an empty mapping) on error.
    """
    try:
        with open(mapping_file, "r") as f:
            mapping = json.load(f)
        return mapping
    except Exception as e:
        logger.warning(f"Failed to load Xbox mapping from {mapping_file}: {e}")
        if fallback is not None:
            return fallback
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

    NOTE (v7.5.x): this opens its own pygame instance, so it must NOT be
    called while the polling worker is connected. The GUI panel snapshots
    the live "monitor" state instead.

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
    ctrl_queue=None,
    stop_event=None,
    mapping_reload_interval: float = 5.0,
) -> None:
    """
    Main polling loop — runs in a separate process.
    v7.2.6: S4 resilient worker — retry loop on startup, crash recovery,
    periodic heartbeat. Worker never exits; reconnects automatically.
    v7.3.2: stick_offsets — per-axis center offsets to subtract before deadzone.
    v7.3.4: axis_deadzones — per-axis deadzone thresholds (0–1). Keys are
            axis indices. Falls back to ``deadzone`` for unmapped axes.
            Sticks are axes 0–3; triggers are axes 4–5.
    v7.5.x: non-destructive heartbeat, trigger arming, zero-on-loss,
            combined same-command dispatch, monitor messages, ctrl_queue,
            stop_event (see module docstring).

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
        ctrl_queue:         Optional multiprocessing.Queue of tuning updates:
                            {"axis_deadzones": {...}} / {"stick_offsets": {...}}
                            / {"debug_mode": bool}. Applied live (v7.5.x).
        stop_event:         Optional multiprocessing.Event — set to stop the
                            worker cleanly (works in thread mode too, v7.5.x).
        mapping_reload_interval: Seconds between mapping-file hot-reloads.
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

        Proven macOS-Bluetooth detection idiom (v7.2.7) — the quit/init
        re-enumeration here is intentional and only runs while *searching*.

        Args:
            timeout:    Max seconds to search. 0 = no limit (initial connect).
            status_key: Status string to send while searching
                        ("waiting" for initial, "reconnecting" after loss).
        Returns:
            Joystick object on success, None if timeout expired or the
            stop_event was set.
        """
        _attempt = 0
        _start = time.time()
        while True:
            if stop_event is not None and stop_event.is_set():
                return None
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
    if joystick is None:
        return  # stop_event set while searching

    axis_accum, axis_count = _init_accumulators(joystick)
    num_axes = joystick.get_numaxes()
    last_axis_time   = time.time()
    last_mapping_time = time.time()
    last_heartbeat   = time.time()
    last_hat = (0, 0)
    _btn_debounce = {}  # v7.2.7: button debounce: last-fire time per button
    # v7.5.x: last_sent maps dispatch-key → (value, command) so an inactive
    # key (unmapped / remapped / controller lost) can be sent a final zero.
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

    # v7.5.x: trigger arming. SDL zero-initializes per-device axis state and
    # an idle controller sends no input reports, so get_axis() on a trigger
    # reads 0.0 INDEFINITELY until the user first moves it (verified on real
    # HW). With the -1.0 rest offset that 0.0 would normalize to a phantom
    # half-press. A trigger axis is therefore trusted only after its first
    # JOYAXISMOTION event for the current joystick object; until then it is
    # treated as at-rest. Cleared on every (re)connect.
    _armed_triggers: set[int] = set()

    def _trigger_pull(ti: int) -> float:
        """Normalized pull amount for trigger axis *ti* in [0, 1]."""
        if ti in _trigger_offsets and ti not in _armed_triggers:
            return 0.0  # state unknown — at rest by definition
        raw = joystick.get_axis(ti)
        _off = _trigger_offsets.get(ti)
        if _off is not None and _off < -0.1:
            raw = (raw - _off) / (1.0 - _off)  # rest→0, full-press→1
        return max(0.0, min(1.0, raw))

    def _dispatch_zeroes(reason: str) -> None:
        """v7.5.x: send a zero to every dispatch-key that last sent a
        non-zero value. Called on controller loss/shutdown so the jog
        handlers stop the stages immediately instead of holding the last
        velocity until reconnection."""
        for _key in list(last_sent):
            _val, _cmd = last_sent.pop(_key)
            _zero = (0.0, 0.0) if isinstance(_val, tuple) else 0.0
            if _val != _zero:
                out_queue.put({"axis": _key, "average": _zero, "command": _cmd})
        if debug_mode:
            out_queue.put({"debug": f"Dispatched zero velocities ({reason})"})

    # v7.3.4 diagnostic retained: report trigger state at startup.
    pygame.event.pump()
    time.sleep(0.2)  # brief settle
    pygame.event.pump()
    if debug_mode:
        _diag_vals = {ti: round(joystick.get_axis(ti), 4)
                      for ti in [4, 5] if ti < num_axes}
        out_queue.put({"debug":
            f"Trigger rest values (raw, at-rest): {_diag_vals} | "
            f"offsets: {_trigger_offsets} | armed: {sorted(_armed_triggers)} | "
            f"platform: {'macOS' if _is_macos else 'Windows/Linux'}"
        })

    # ── Main Loop ─────────────────────────────────────────────────
    _last_trigger_diag = time.time()  # v7.3.4: periodic trigger diagnostics

    while True:
        if stop_event is not None and stop_event.is_set():
            _dispatch_zeroes("worker stopped")
            out_queue.put({"status": "disconnected"})
            break

        current_time = time.time()

        try:
            # v7.5.x: drain SDL events (replaces the bare event.pump()).
            # JOYAXISMOTION arms trigger axes; JOYDEVICEREMOVED detects
            # loss immediately (incl. silent Bluetooth loss) without the
            # destructive subsystem re-init the old heartbeat used.
            _device_removed = False
            for _ev in pygame.event.get():
                if _ev.type == pygame.JOYAXISMOTION:
                    _ax = getattr(_ev, "axis", None)
                    if _ax in _trigger_offsets:
                        _armed_triggers.add(_ax)
                elif _ev.type == pygame.JOYDEVICEREMOVED:
                    try:
                        _ours = (_ev.instance_id == joystick.get_instance_id())
                    except Exception:
                        _ours = True  # can't tell — assume it was ours
                    if _ours:
                        _device_removed = True
            if _device_removed:
                raise RuntimeError("Controller removed (JOYDEVICEREMOVED)")

            # v7.5.x: live tuning updates from the GUI (no reconnect needed)
            if ctrl_queue is not None:
                try:
                    while True:
                        _upd = ctrl_queue.get_nowait()
                        if "axis_deadzones" in _upd:
                            axis_deadzones = {int(k): float(v) for k, v
                                              in (_upd["axis_deadzones"] or {}).items()}
                        if "stick_offsets" in _upd:
                            stick_offsets = {int(k): float(v) for k, v
                                             in (_upd["stick_offsets"] or {}).items()}
                        if "debug_mode" in _upd:
                            debug_mode = bool(_upd["debug_mode"])
                        if debug_mode:
                            out_queue.put({"debug": f"Tuning updated live: "
                                           f"{sorted(_upd.keys())}"})
                except Exception:
                    pass  # queue drained (Empty) or malformed update

            # v7.3.4: Periodic trigger diagnostic (debug mode only).
            if debug_mode and current_time - _last_trigger_diag >= 2.0:
                _per_axis_dz_diag = axis_deadzones or {}
                _trig_report = []
                for _ti in [4, 5]:
                    if _ti < num_axes:
                        _raw = joystick.get_axis(_ti)
                        _pull = _trigger_pull(_ti)
                        _dz = _per_axis_dz_diag.get(_ti, deadzone)
                        _trig_report.append(
                            f"ax{_ti}: raw={_raw:.3f} pull={_pull:.3f} "
                            f"armed={_ti in _armed_triggers} dz={_dz:.2f} "
                            f"passes={abs(_pull) > _dz}"
                        )
                out_queue.put({"debug": "TRIG DIAG | " + " | ".join(_trig_report)})
                _last_trigger_diag = current_time

            # Hot-reload mapping (default every 5 seconds).
            # v7.5.x: keeps the current mapping on read/parse failure.
            if current_time - last_mapping_time >= mapping_reload_interval:
                mapping = load_xbox_mapping(mapping_file, fallback=mapping)
                last_mapping_time = current_time

            # Heartbeat every 3 seconds — v7.5.x: NON-destructive.
            # The old quit()/init() re-enumeration wiped axis state every
            # 3 s (the root cause of the periodic-reset / phantom-trigger
            # field failures). get_count() is hot-plug-aware in SDL2 once
            # events are pumped, and JOYDEVICEREMOVED above catches loss
            # immediately, so no subsystem re-init is needed.
            if current_time - last_heartbeat >= 3.0:
                if pygame.joystick.get_count() == 0:
                    raise RuntimeError("Controller lost (no joysticks detected)")
                out_queue.put({"status": "alive"})
                last_heartbeat = current_time

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
            _stick_off = stick_offsets or {}
            _per_axis_dz = axis_deadzones or {}
            for i in range(num_axes):
                if i in _trigger_offsets:
                    # v7.5.x: triggers go through the armed/normalized path
                    raw = _trigger_pull(i)
                else:
                    raw = joystick.get_axis(i)
                    # v7.3.2: Subtract stick center offset
                    if i in _stick_off:
                        raw -= _stick_off[i]
                # v7.3.4: Per-axis deadzone (falls back to global deadzone)
                _dz = _per_axis_dz.get(i, deadzone)
                if abs(raw) > _dz:
                    axis_accum[i] += raw
                    axis_count[i] += 1

            if current_time - last_axis_time >= avg_interval:
                # 1) Per-group averaged values (only actively mapped groups)
                group_vals: dict = {}
                group_cmds: dict = {}
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

                    # v7.5.x: soft per-group axis invert (mapping "axes_invert").
                    # A controller-input-side direction flip, independent of the
                    # hardware axis_flip (device profile): lets a user correct a
                    # backwards stick/trigger without inverting the motor for the
                    # GUI jog or prints. Applied after the LT-retract negation and
                    # deadzone clamp, and before same-command combination.
                    if mapping.get("axes_invert", {}).get(group["name"]):
                        if group["type"] == "axis":
                            avg_val = tuple(-v for v in avg_val)
                        else:
                            avg_val = -avg_val

                    group_vals[group["name"]] = (group["type"], avg_val)
                    group_cmds[group["name"]] = cmd
                    for a in axes:
                        axis_accum[a] = 0.0; axis_count[a] = 0

                # 2) v7.5.x: combine same-type groups sharing one command.
                # Both triggers mapped to move_z_at_velocity used to race
                # (deterministic RT-wins overwrite each cycle); now they
                # dispatch a single combined value (RT − LT). Same for two
                # stick groups sharing a command (element-wise sum).
                dispatch: dict = {}
                _handled: set = set()
                _names = list(group_vals)
                for _name in _names:
                    if _name in _handled:
                        continue
                    _cmd = group_cmds[_name]
                    _gtype, _val = group_vals[_name]
                    _partners = [n for n in _names
                                 if n != _name and n not in _handled
                                 and group_cmds[n] == _cmd
                                 and group_vals[n][0] == _gtype]
                    if _partners:
                        _members = [_name] + _partners
                        _key = "+".join(_members)
                        if _gtype == "axis":
                            _vals = [group_vals[n][1] for n in _members]
                            _comb = tuple(
                                max(-1.0, min(1.0, sum(v[i] for v in _vals)))
                                for i in range(2))
                        else:
                            _comb = max(-1.0, min(1.0, sum(
                                group_vals[n][1] for n in _members)))
                        dispatch[_key] = (_cmd, _gtype, _comb)
                        _handled.update(_members)
                    else:
                        dispatch[_name] = (_cmd, _gtype, _val)
                        _handled.add(_name)

                # 3) Send changed-or-non-zero values
                for _key, (_cmd, _gtype, _val) in dispatch.items():
                    zero_value = (0.0, 0.0) if _gtype == "axis" else 0.0
                    _prev_entry = last_sent.get(_key)
                    prev = _prev_entry[0] if _prev_entry else zero_value

                    if _val != zero_value or prev != zero_value:
                        # v7.3.4: Log every dispatch so we can trace the source
                        if debug_mode and _val != zero_value:
                            out_queue.put({"debug":
                                f"DISPATCH axis={_key} avg={_val} cmd={_cmd}"
                            })
                        out_queue.put({
                            "axis":    _key,
                            "average": _val,
                            "command": _cmd,
                        })
                        last_sent[_key] = (_val, _cmd)

                # 4) v7.5.x: keys that went inactive (unmapped / remapped /
                # regrouped mid-hold) get one final zero to their previous
                # command so a velocity can't stay stuck in a jog handler.
                for _key in list(last_sent):
                    if _key not in dispatch:
                        _val, _cmd = last_sent.pop(_key)
                        _zero = (0.0, 0.0) if isinstance(_val, tuple) else 0.0
                        if _val != _zero:
                            out_queue.put(
                                {"axis": _key, "average": _zero, "command": _cmd})

                # 5) v7.5.x: publish raw monitor state for the GUI (sticks
                # raw pre-offset so Calibrate Sticks can snapshot true rest
                # values; triggers as gated normalized pull in [0,1]).
                _mon_sticks = {a: round(joystick.get_axis(a), 4)
                               for a in range(min(4, num_axes))}
                _mon_trigs = {ti: round(_trigger_pull(ti), 4)
                              for ti in (4, 5) if ti < num_axes}
                out_queue.put({"monitor": {"sticks": _mon_sticks,
                                           "triggers": _mon_trigs}})

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
            # v7.5.x: fixed misindented debounce check — '_dlast' was
            # referenced outside its 'if _cmd' guard, so an UNMAPPED
            # direction raised NameError -> outer except -> full reconnect.
            if joystick.get_numhats() == 0:
                _hx = (1 if 14 < joystick.get_numbuttons() and joystick.get_button(14) else 0) - \
                      (1 if 13 < joystick.get_numbuttons() and joystick.get_button(13) else 0)
                _hy = (1 if 11 < joystick.get_numbuttons() and joystick.get_button(11) else 0) - \
                      (1 if 12 < joystick.get_numbuttons() and joystick.get_button(12) else 0)
                _hat_now = (_hx, _hy)
                if _hat_now != last_hat:
                    if _hat_now != (0, 0):
                        for _dir, _cond in [("up", _hy>0), ("down", _hy<0), ("left", _hx<0), ("right", _hx>0)]:
                            if not _cond:
                                continue
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
            # v7.5.x: stop the stages FIRST — zero every active dispatch key
            # so the jog handlers don't hold the last velocity while we
            # search for the controller.
            out_queue.put({"debug": f"Controller error: {e}"})
            _dispatch_zeroes("controller lost")
            out_queue.put({"status": "reconnecting"})
            joystick = _find_controller(
                timeout=reconnect_timeout, status_key="reconnecting"
            )
            if joystick is None:
                # Timeout expired (or stop requested) — give up
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
            # v7.5.x: new joystick object — trigger state unknown again
            _armed_triggers.clear()
