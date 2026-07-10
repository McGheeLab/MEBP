"""Display-only motion interpolation ("dead reckoning") for a live-polling feel.

When the GUI issues a jog/travel move the command is *fire-and-forget* and the
real position **cannot be polled while the axis is moving** (the background
``PositionPoller`` is ~300 ms and is *suspended* during ``safe_travel_to`` /
prints), so a readout would sit frozen and then snap to the destination. This
estimator predicts where the axis *should* be as a function of elapsed time and
the known move speed, so the display can ANIMATE start→target "as if it were
live-polling," then hand back to the real poller cache on arrival.

PURE PREDICTION — this never commands hardware and never affects motion,
clamping, or safety. The GUI position readouts consult it through the
``StageController.get_display_*`` getters; the real ``PositionPoller`` cache
remains the single source of truth for every other consumer. On any error a
getter yields ``None`` and the caller falls back to the raw cache.

Frames match the ``StageController`` caches exactly:

* ``"XY"``                  — 2-tuple ``(x, y)`` absolute stage µm
* ``"Z" / "P1" / "P2" / "P3"`` — scalar raw Marlin mm (the value at that logical
  axis's physical slot of the ZP tuple)

Two registration verbs:

* :meth:`note_target` — a discrete move with a known destination + scalar speed.
  Interpolated LINEARLY over ``duration = |target - start| / speed``; once the
  window elapses the channel goes idle (the display defers to the real cache).
* :meth:`advance` — one segment of a *continuous* jog (Xbox stick). The estimate
  accumulates the commanded delta from a base captured (from the live cache) at
  jog start, and stays live until ``settle_s`` after the last advance, then idle.

Thread-safe (one lock); every public method is guarded to never raise.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Iterable

# Logical scalar channels (everything except the 2-D "XY" channel).
SCALAR_CHANNELS: tuple[str, ...] = ("Z", "P1", "P2", "P3")
ALL_CHANNELS: tuple[str, ...] = ("XY",) + SCALAR_CHANNELS


class MotionEstimator:
    """Display-only dead-reckoning of stage position. See module docstring."""

    def __init__(self, *, settle_s: float = 0.4,
                 clock: Callable[[], float] | None = None) -> None:
        self._lock = threading.Lock()
        # How long an "advance" (continuous-jog) estimate stays live after the
        # last commanded segment before it expires and defers to the real cache.
        # A bit longer than one poll interval so the poller has caught up.
        self._settle_s = max(0.0, float(settle_s))
        self._clock = clock or time.monotonic
        # channel -> state dict
        self._ch: dict[str, dict] = {}

    # ── registration ────────────────────────────────────────────────

    def note_target(self, channel: str, start, target, speed: float) -> None:
        """Register a discrete move to ``target`` from ``start`` at ``speed``.

        ``channel`` is ``"XY"`` (``start``/``target`` are ``(x, y)`` µm, ``speed``
        µm/s) or a scalar channel (``start``/``target`` raw mm, ``speed`` mm/s).
        The estimate interpolates linearly over ``|target-start|/speed`` seconds.
        A non-positive/NaN speed or malformed input simply clears the channel
        (the display then shows the real cache).
        """
        try:
            speed = float(speed)
            if channel == "XY":
                sx, sy = float(start[0]), float(start[1])
                tx, ty = float(target[0]), float(target[1])
                dist = math.hypot(tx - sx, ty - sy)
                payload: dict = {"start": (sx, sy), "target": (tx, ty)}
            elif channel in SCALAR_CHANNELS:
                s = float(start)
                t = float(target)
                dist = abs(t - s)
                payload = {"start": s, "target": t}
            else:
                return
            if not math.isfinite(dist) or not math.isfinite(speed) or speed <= 0:
                self.clear(channel)
                return
            dur = 0.0 if dist <= 1e-9 else dist / speed
            now = self._clock()
            with self._lock:
                self._ch[channel] = {
                    "mode": "target", "t0": now, "dur": dur, **payload}
        except Exception:
            self.clear(channel)

    def advance(self, channel: str, delta) -> None:
        """Accumulate one continuous-jog segment's commanded ``delta``.

        ``delta`` is ``(dx, dy)`` µm for ``"XY"`` or a scalar raw-mm delta for a
        scalar channel. The first :meth:`estimate` after this locks the base to
        the live cache; subsequent advances accumulate onto it. The estimate
        expires ``settle_s`` after the last advance.
        """
        try:
            if channel == "XY":
                dx, dy = float(delta[0]), float(delta[1])
                if not (math.isfinite(dx) and math.isfinite(dy)):
                    return
            elif channel in SCALAR_CHANNELS:
                dx = float(delta)
                if not math.isfinite(dx):
                    return
            else:
                return
            now = self._clock()
            with self._lock:
                st = self._ch.get(channel)
                if st is None or st.get("mode") != "advance":
                    st = {"mode": "advance", "base": None,
                          "acc": (0.0, 0.0) if channel == "XY" else 0.0}
                    self._ch[channel] = st
                if channel == "XY":
                    ax, ay = st["acc"]
                    st["acc"] = (ax + dx, ay + dy)
                else:
                    st["acc"] = float(st["acc"]) + dx
                st["t_last"] = now
        except Exception:
            self.clear(channel)

    def clear(self, channel: str) -> None:
        with self._lock:
            self._ch.pop(channel, None)

    def clear_all(self) -> None:
        with self._lock:
            self._ch.clear()

    # ── evaluation ──────────────────────────────────────────────────

    def estimate(self, channel: str, cache_value):
        """Predicted value for ``channel`` (same shape as ``cache_value``), or
        ``None`` to defer to the real cache.

        ``cache_value`` is the current poller value for that channel: ``(x, y)``
        (or longer) for ``"XY"``, else the scalar raw mm. It is used to lock the
        base for an ``advance`` estimate; ``target`` estimates ignore it.
        """
        try:
            now = self._clock()
            with self._lock:
                st = self._ch.get(channel)
                if st is None:
                    return None
                mode = st.get("mode")
                if mode == "target":
                    return self._eval_target(channel, st, now)
                if mode == "advance":
                    return self._eval_advance(channel, st, now, cache_value)
        except Exception:
            return None
        return None

    def _eval_target(self, channel, st, now):
        dur = st["dur"]
        frac = 1.0 if dur <= 0 else (now - st["t0"]) / dur
        if frac >= 1.0:
            # Arrived: show the exact target this frame, then go idle so the
            # next frame defers to the (by-now-real) cache.
            self._ch.pop(channel, None)
            return st["target"]
        if frac < 0.0:
            frac = 0.0
        if channel == "XY":
            sx, sy = st["start"]
            tx, ty = st["target"]
            return (sx + (tx - sx) * frac, sy + (ty - sy) * frac)
        s, t = st["start"], st["target"]
        return s + (t - s) * frac

    def _eval_advance(self, channel, st, now, cache_value):
        if (now - st.get("t_last", now)) > self._settle_s:
            self._ch.pop(channel, None)
            return None
        base = st.get("base")
        if base is None:
            # Lock the base to the live cache and discard the (tiny) pre-base
            # accumulation, so we don't double-count motion already reflected.
            if channel == "XY":
                if not cache_value or cache_value[0] is None:
                    return None
                base = (float(cache_value[0]), float(cache_value[1]))
                st["base"] = base
                st["acc"] = (0.0, 0.0)
            else:
                if cache_value is None:
                    return None
                base = float(cache_value)
                st["base"] = base
                st["acc"] = 0.0
        if channel == "XY":
            ax, ay = st["acc"]
            return (base[0] + ax, base[1] + ay)
        return base + st["acc"]

    # ── status ──────────────────────────────────────────────────────

    def active_channels(self) -> set[str]:
        """The set of channels currently producing a live estimate (for the
        'estimated/moving' cue). A completed/expired estimate is not counted."""
        out: set[str] = set()
        try:
            now = self._clock()
            with self._lock:
                for ch, st in self._ch.items():
                    mode = st.get("mode")
                    if mode == "target":
                        dur = st.get("dur", 0.0)
                        if dur > 0 and (now - st["t0"]) < dur:
                            out.add(ch)
                    elif mode == "advance":
                        if (now - st.get("t_last", now)) <= self._settle_s:
                            out.add(ch)
        except Exception:
            return set()
        return out

    def is_active(self) -> bool:
        return bool(self.active_channels())

    @staticmethod
    def display_axes(channels: Iterable[str]) -> set[str]:
        """Map estimator channels to per-axis display tokens
        (``"XY"`` → ``{"X", "Y"}``; scalar channels pass through)."""
        out: set[str] = set()
        for ch in channels:
            if ch == "XY":
                out.update(("X", "Y"))
            else:
                out.add(ch)
        return out
