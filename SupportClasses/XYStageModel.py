"""XYStageModel.py — a saved-characteristics motion model of the XY stage.

The 2026-07-28 hardware session measured, on ME3B V1, everything a velocity
follower's closed loop actually experiences:

    loop period      32.0 ms   (rate the command→read cycle really runs at)
    dead time        67.1 ms   (command → first OBSERVED motion, through the
                                same serial read path the follower polls; the
                                transport share is ~40 ms, the rest is the read)
    rise time τ      27.1 ms   (first-order velocity rise after the dead time)
    top speed        5945.6 µm/s (measured; the JSON's declared 50000 is wrong)
    encoder quantum  ~1 µm     (rest jitter 0–1 µm/poll)

Those numbers are persisted by the calibration flow
(``PrintTimingCalibrationStore``: ``velocity_dead_time_s`` + its meta's
``tau_s``, ``control_loop_ms``, ``xy_max_speed_um_s``), which makes the stage's
dynamics a *saved artefact* — so its likely motion can be simulated offline,
with different tunings, without touching hardware. :class:`StageCharacteristics`
is that artefact; :class:`XYStageModel` integrates it.

The model is deliberately simple — FOPDT (first-order plus dead time) per axis
with a shared magnitude clamp and a quantised readout — because that is exactly
the model the calibration math already assumes (``plant_ultimate_gain``,
``derive_settings``), and the hardware session showed it holds: the relay's
measured Tu matched the FOPDT prediction to ~1 %, and the dead-time probe's five
trials agreed to 0.5 ms. Validation against real Circle/Star traces lives in
``tests/test_v75x_xy_stage_model.py`` + the update plan.

Pure Python (``math`` only): importable from GUI worker threads and tests alike.

⚠ A command pipeline, not a resettable timer: each velocity command takes
effect ``dead_time_s`` after it was ISSUED. An earlier ad-hoc sim restarted a
single delay timer on every command, so re-commanding faster than the dead time
meant the stage never moved at all — wrong, and the kind of wrong that quietly
invalidates every conclusion drawn on top of it.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, asdict


@dataclass
class StageCharacteristics:
    """The saved dynamics of one machine's XY stage. Zeros = not measured."""
    dead_time_s: float = 0.0        # command → observed motion (incl. read path)
    tau_s: float = 0.0              # first-order velocity rise time
    top_speed_um_s: float = 0.0     # measured true maximum |v|
    control_loop_ms: float = 0.0    # measured closed-loop command→read period
    quant_um: float = 1.0           # position readout quantum
    name: str = ""                  # e.g. "ME3B V1"
    measured_at: str = ""           # ISO timestamp of the measurement

    def is_complete(self) -> bool:
        return (self.dead_time_s > 0 and self.top_speed_um_s > 0
                and self.control_loop_ms > 0)

    def missing(self) -> list:
        out = []
        if self.control_loop_ms <= 0:
            out.append("comms rate")
        if self.dead_time_s <= 0:
            out.append("dead time")
        if self.top_speed_um_s <= 0:
            out.append("top speed")
        return out

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "StageCharacteristics":
        d = d or {}
        kw = {}
        for f in ("dead_time_s", "tau_s", "top_speed_um_s", "control_loop_ms",
                  "quant_um"):
            try:
                kw[f] = float(d.get(f, 0.0) or 0.0)
            except (TypeError, ValueError):
                kw[f] = 0.0
        kw["quant_um"] = kw["quant_um"] or 1.0
        kw["name"] = str(d.get("name", "") or "")
        kw["measured_at"] = str(d.get("measured_at", "") or "")
        return cls(**kw)

    @classmethod
    def from_store(cls, store, *, name: str = "") -> "StageCharacteristics":
        """Build from the shared timing-calibration store — the persisted result
        of the one-click calibration / "Measure dead time" flow."""
        dead = 0.0
        try:
            dead, _src = store.effective_dead_time_s()
        except Exception:
            pass
        meta = {}
        try:
            meta = store.get_velocity_dead_time_meta() or {}
        except Exception:
            pass
        try:
            top = float(store.get_xy_max_speed_um_s() or 0.0)
        except Exception:
            top = 0.0
        try:
            loop = float(store.get_control_loop_ms() or 0.0)
        except Exception:
            loop = 0.0
        return cls(
            dead_time_s=float(dead or 0.0),
            tau_s=float(meta.get("tau_s", 0.0) or 0.0),
            top_speed_um_s=top,
            control_loop_ms=loop,
            name=name,
            measured_at=str(meta.get("last_updated", "") or ""),
        )

    @classmethod
    def from_machine(cls, m, *, name: str = "") -> "StageCharacteristics":
        """From an ``XYAutoCalibration.MeasuredMachine`` (fresh probe results)."""
        return cls(
            dead_time_s=float(getattr(m, "dead_time_s", 0.0) or 0.0),
            tau_s=float(getattr(m, "tau_s", 0.0) or 0.0),
            top_speed_um_s=float(getattr(m, "top_speed_um_s", 0.0) or 0.0),
            control_loop_ms=float(getattr(m, "control_loop_ms", 0.0) or 0.0),
            name=name,
        )


class XYStageModel:
    """Integrates :class:`StageCharacteristics`: velocity commands enter a
    dead-time pipeline, the actual velocity first-order-lags toward the active
    command, position integrates, and the readout is quantised.

    Time is the model's own (seconds from construction); the caller advances it
    explicitly with :meth:`advance`, so simulations are deterministic and run at
    machine speed regardless of wall clock.
    """

    #: Integration substep. 1 ms resolves a 27 ms τ and a 32 ms tick cleanly.
    SUBSTEP_S = 0.001

    def __init__(self, char: StageCharacteristics, *, x_um: float = 0.0,
                 y_um: float = 0.0):
        self.char = char
        self._x = float(x_um)
        self._y = float(y_um)
        self._t = 0.0
        self._pending = deque()          # (t_effective, vx, vy) — a pipeline
        self._cmd = (0.0, 0.0)           # command currently in effect
        self._v = (0.0, 0.0)             # actual velocity (after the lag)
        self.travel_um = 0.0             # odometer, diagnostic

    # ── inputs ────────────────────────────────────────────────────────
    def command_velocity(self, vx_um_s: float, vy_um_s: float) -> None:
        """Issue a velocity command NOW; it takes effect ``dead_time_s`` later.
        Magnitude is clamped to the measured top speed (the stage cannot exceed
        it however hard it is commanded)."""
        vx, vy = float(vx_um_s), float(vy_um_s)
        top = self.char.top_speed_um_s
        if top > 0:
            mag = math.hypot(vx, vy)
            if mag > top:
                vx *= top / mag
                vy *= top / mag
        self._pending.append((self._t + max(0.0, self.char.dead_time_s), vx, vy))

    def teleport(self, x_um: float, y_um: float) -> None:
        """Place the stage (models a completed point-to-point positioning move
        that the follower does not observe). Clears in-flight commands."""
        self._x, self._y = float(x_um), float(y_um)
        self._pending.clear()
        self._cmd = (0.0, 0.0)
        self._v = (0.0, 0.0)

    # ── time ──────────────────────────────────────────────────────────
    @property
    def t(self) -> float:
        return self._t

    def advance(self, dt_s: float) -> None:
        """Integrate the model forward by ``dt_s`` seconds."""
        remaining = max(0.0, float(dt_s))
        tau = max(0.0, self.char.tau_s)
        while remaining > 1e-12:
            h = min(self.SUBSTEP_S, remaining)
            t_next = self._t + h
            # every pipelined command whose effective time has arrived
            while self._pending and self._pending[0][0] <= t_next:
                _, vx, vy = self._pending.popleft()
                self._cmd = (vx, vy)
            if tau > 1e-6:
                a = 1.0 - math.exp(-h / tau)
                self._v = (self._v[0] + (self._cmd[0] - self._v[0]) * a,
                           self._v[1] + (self._cmd[1] - self._v[1]) * a)
            else:
                self._v = self._cmd
            dx = self._v[0] * h
            dy = self._v[1] * h
            self._x += dx
            self._y += dy
            self.travel_um += math.hypot(dx, dy)
            self._t = t_next
            remaining -= h

    # ── outputs ───────────────────────────────────────────────────────
    def read_position(self) -> tuple:
        """(x, y) in µm, quantised like the real readout."""
        q = self.char.quant_um or 1.0
        return (round(self._x / q) * q, round(self._y / q) * q)

    def position_exact(self) -> tuple:
        return (self._x, self._y)

    def velocity(self) -> tuple:
        return self._v
