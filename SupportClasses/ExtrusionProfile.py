"""
ExtrusionProfile.py — the deposition along ONE print path, per segment.

v7.21.5. Every print path used to carry a single scalar flow: one
``flow_rate_uL_s`` for the whole ``PRINT_PATH``, so the deposited
volume-per-mm was constant from the first waypoint to the last. That is fine
for a single-shape print and wrong for a sketch, whose shapes can each declare
their own line width (v7.21.4/v7.21.5: an outline prints as ONE pass and its
width comes from flow) and any of which may be marked *no extrude*.

This module is the one place that describes "how much per mm, WHERE" and the
one place that looks it up. It is deliberately pure — stdlib only, no numpy, no
Qt, no repo imports — because it is consumed on the print thread by
``PrintManager`` and produced by the Sketch compiler and Quick Print.

**Absolute microlitres per mm is the transferable quantity.** An extrusion
*modifier* is bore-relative and therefore only means something next to the
needle it was computed against; µL/mm is what the pump actually has to deliver.
Producers convert (``bore area × modifier``) and hand over µL/mm.

**A malformed profile is REFUSED, never repaired.** :meth:`ExtrusionProfile.build`
returns ``None`` when the entry count does not match the path, because a profile
that is off by one mis-assigns flow to the wrong part of the path — silently
laying a thick bead where a thin one was designed. The caller then falls back to
the scalar flow, which is merely the old behaviour.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass

__all__ = ["ExtrusionProfile", "modifier_to_vol_per_mm", "rate_for"]


def modifier_to_vol_per_mm(modifiers, bore_area_mm2: float,
                           trim: float = 1.0) -> list[float]:
    """Convert a per-segment extrusion MODIFIER profile to µL/mm.

    ``bore_area_mm2`` is the needle orifice cross-section — the deposition at
    modifier 1.0, the canonical F-1 bead model shared by ``FlowPhysics`` /
    ``GeometryEngine`` / the Sketch compiler / Quick Print. ``trim`` is an
    extra global scale (Quick Print's own Extrusion ×), applied on top so the
    operator keeps one knob over a sketch's own numbers.

    Recomputing from the modifier — rather than shipping the producer's µL/mm
    verbatim — is deliberate: it prints the declared WIDTHS with the needle
    actually fitted now, instead of reproducing a volume that was correct for
    whatever needle was fitted when the sketch was baked.
    """
    area = max(0.0, float(bore_area_mm2 or 0.0))
    k = area * max(0.0, float(trim if trim is not None else 1.0))
    out = []
    for m in modifiers or ():
        try:
            out.append(max(0.0, float(m)) * k)
        except (TypeError, ValueError):
            out.append(0.0)
    return out


def rate_for(vol_per_mm: float, speed_mm_s: float, fallback: float) -> float:
    """Pump rate (µL/s) that lays ``vol_per_mm`` while travelling at
    ``speed_mm_s``.

    The rate has to track the volume, or a thicker segment would simply take
    longer to dispense: the discrete executor paces each segment by
    ``max(xy_move_time, pump_move_time)``, so holding the rate fixed would slow
    the stage down over a wide line and lay the extra volume over a longer time
    — i.e. quietly cancel the width it was meant to produce. ``fallback`` (the
    path's scalar flow) is used when either input is unusable, and a
    zero-deposition segment returns 0.0 so the caller can skip the pump move.
    """
    try:
        v = float(vol_per_mm)
        s = float(speed_mm_s)
    except (TypeError, ValueError):
        return max(0.0, float(fallback or 0.0))
    if v <= 0.0:
        return 0.0
    if not (s > 0.0) or not math.isfinite(v) or not math.isfinite(s):
        return max(0.0, float(fallback or 0.0))
    return v * s


@dataclass(frozen=True)
class ExtrusionProfile:
    """Per-segment deposition (µL/mm) for one path, indexable two ways.

    ``vol_per_mm[i]`` applies to the segment from waypoint *i* to *i+1*, so
    ``len(vol_per_mm) == len(points) - 1``. The discrete executor walks segments
    and looks up :meth:`at_index`; the velocity followers advance an arc-length
    cursor and look up :meth:`at_arclen`. Both must agree, which is why they
    share one object and one cumulative-length table.
    """

    vol_per_mm: tuple[float, ...]
    cum_mm: tuple[float, ...]          # cumulative arc length, len = n_points

    # ── construction ────────────────────────────────────────────────

    @classmethod
    def build(cls, points, vol_per_mm) -> "ExtrusionProfile | None":
        """Build from a path and its per-segment µL/mm, or ``None``.

        Returns ``None`` — meaning "use the scalar flow" — when the profile is
        absent, empty, the wrong length, or not finite. Refusing beats guessing:
        an off-by-one profile would deposit the wrong amount in the wrong place.
        """
        if vol_per_mm is None or points is None:
            return None
        try:
            vals = [float(v) for v in vol_per_mm]
            pts = [(float(p[0]), float(p[1])) for p in points]
        except (TypeError, ValueError, IndexError):
            return None
        if len(pts) < 2 or len(vals) != len(pts) - 1:
            return None
        if any((not math.isfinite(v)) or v < 0.0 for v in vals):
            return None
        cum = [0.0]
        acc = 0.0
        for i in range(len(pts) - 1):
            acc += math.dist(pts[i], pts[i + 1])
            cum.append(acc)
        return cls(vol_per_mm=tuple(vals), cum_mm=tuple(cum))

    # ── lookup ──────────────────────────────────────────────────────

    def at_index(self, i: int) -> float:
        """µL/mm for the segment leaving waypoint ``i`` (clamped at the ends)."""
        if not self.vol_per_mm:
            return 0.0
        if i < 0:
            i = 0
        elif i >= len(self.vol_per_mm):
            i = len(self.vol_per_mm) - 1
        return self.vol_per_mm[i]

    def at_arclen(self, s: float) -> float:
        """µL/mm at arc length ``s`` along the path.

        ``s`` is measured from the path start (what every velocity follower
        already tracks). A position exactly on a waypoint takes the value of the
        segment it is ENTERING, so the deposition changes at the boundary rather
        than one segment late.
        """
        if not self.vol_per_mm:
            return 0.0
        try:
            s = float(s)
        except (TypeError, ValueError):
            return self.vol_per_mm[0]
        if s <= 0.0:
            return self.vol_per_mm[0]
        if s >= self.cum_mm[-1]:
            return self.vol_per_mm[-1]
        # cum_mm[k] <= s < cum_mm[k+1] → segment k.
        k = bisect_right(self.cum_mm, s) - 1
        return self.at_index(k)

    # ── reporting ───────────────────────────────────────────────────

    @property
    def total_uL(self) -> float:
        return sum(self.vol_per_mm[i] * (self.cum_mm[i + 1] - self.cum_mm[i])
                   for i in range(len(self.vol_per_mm)))

    @property
    def total_mm(self) -> float:
        return self.cum_mm[-1] if self.cum_mm else 0.0

    @property
    def is_uniform(self) -> bool:
        """True when every PRINTING segment deposits the same amount — i.e. the
        profile carries no information a scalar flow could not."""
        vals = [v for v in self.vol_per_mm if v > 0.0]
        if not vals:
            return True
        return (max(vals) - min(vals)) <= 1e-12 * max(1.0, max(vals))

    def printing_span(self) -> tuple[float, float]:
        """``(min, max)`` µL/mm over the PRINTING segments (``(0, 0)`` if none).

        Zeros are excluded because a travel / no-extrude segment is an absence
        of deposition, not a thin bead — including them would report every
        sketch with a pen-up as spanning down to zero.
        """
        vals = [v for v in self.vol_per_mm if v > 0.0]
        return (min(vals), max(vals)) if vals else (0.0, 0.0)

    def slice_segments(self, i0: int, i1: int) -> list[float]:
        """The µL/mm entries for segments ``[i0, i1)`` — used when a path is
        split into sub-paths at its travel moves and each sub-path needs the
        matching slice of the profile."""
        i0 = max(0, int(i0))
        i1 = min(len(self.vol_per_mm), int(i1))
        return list(self.vol_per_mm[i0:i1]) if i1 > i0 else []
