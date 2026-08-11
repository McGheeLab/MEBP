"""
plate_layout — ONE mm → pixel fit for every widget that draws a well plate.

v7.12.

Why this exists
---------------
A plate's wells carry their real geometry on ``WellInfo`` (``x``, ``y``,
``diameter``, all mm relative to A1). The plate-level aggregates —
``rows``/``cols``, ``well_spacing_x/y``, ``well_diameter`` — describe a regular
ANSI grid and are deliberately meaningless on a parametric plate:
``WellPlate.from_wells`` stores ``0.0`` spacing and ``0.0`` diameter, and
synthesises ``rows``/``cols`` as a *pseudo*-grid in which a whole ring of wells
can share one cell.

Widgets that laid wells out on that pseudo-grid therefore drew a 3-well ring as
a single circle. Widgets that read the per-well geometry were already correct.
This module is the shared version of what the correct ones do, so the layout
has one implementation rather than one per widget.

The transform is pure arithmetic — no Qt — so it is testable without a
``QApplication``, and a widget only has to decide *where* to draw, never *how
to fit*.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

logger = logging.getLogger(__name__)

__all__ = ["PlateTransform", "wells_from_plate", "fit_wells"]


@dataclass(frozen=True)
class PlateTransform:
    """Immutable mm → px mapping for one plate rendering.

    ``scale`` is pixels per mm; ``ox``/``oy`` place plate-local mm ``(0, 0)``
    (the A1 centre) in widget pixels.
    """

    scale: float
    ox: float
    oy: float
    min_radius_px: float = 0.0

    def to_px(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        """Plate-local mm → widget pixels."""
        return (self.ox + x_mm * self.scale, self.oy + y_mm * self.scale)

    def to_mm(self, px: float, py: float) -> tuple[float, float]:
        """Widget pixels → plate-local mm (inverse of :meth:`to_px`)."""
        if self.scale == 0:                                # pragma: no cover
            return (0.0, 0.0)
        return ((px - self.ox) / self.scale, (py - self.oy) / self.scale)

    def radius_px(self, diameter_mm: float) -> float:
        """Radius in pixels for a well of *diameter_mm*.

        Floored at ``min_radius_px`` so a small well on a mixed-diameter plate
        (a 5.5 mm rosette bore beside a 28 mm insert) stays visible and
        clickable in a small widget instead of collapsing to sub-pixel.
        """
        return max(self.min_radius_px, float(diameter_mm) * self.scale / 2.0)


def wells_from_plate(plate) -> list[tuple[str, float, float, float]]:
    """``[(name, x_mm, y_mm, diameter_mm), …]`` for every well on *plate*.

    Resolves each diameter through ``WellPlate.well_diameter_of`` so a plate
    with no per-well diameter still reports something drawable.

    A well carrying no ``x``/``y`` is SKIPPED rather than defaulted to the
    origin: this feeds ``paintEvent``, which must not raise, and defaulting
    would silently stack wells at one point — the very failure this module
    exists to prevent. A plate whose wells have no geometry therefore returns
    ``[]`` and the caller draws nothing, which is the honest outcome.
    """
    out: list[tuple[str, float, float, float]] = []
    if plate is None:
        return out
    try:
        wells = plate.get_all_wells()
    except Exception:                                      # pragma: no cover
        return out
    skipped = 0
    for well in wells:
        x = getattr(well, "x", None)
        y = getattr(well, "y", None)
        if x is None or y is None:
            skipped += 1
            continue
        try:
            diameter = float(plate.well_diameter_of(well.name))
        except Exception:
            diameter = float(getattr(well, "diameter", 0.0) or 0.0)
        out.append((getattr(well, "name", ""), float(x), float(y), diameter))
    if skipped:
        logger.debug("plate_layout: skipped %d well(s) with no x/y", skipped)
    return out


def fit_wells(
    wells: Iterable[Sequence],
    width_px: float,
    height_px: float,
    margins: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0),
    footprint: Optional[tuple[float, float, float, float]] = None,
    min_radius_px: float = 0.0,
) -> Optional[PlateTransform]:
    """Aspect-preserving fit of a plate into a widget rect.

    Args:
        wells: ``(name, x_mm, y_mm, diameter_mm)`` tuples — the output of
            :func:`wells_from_plate`. Also accepts bare
            ``(x_mm, y_mm, diameter_mm)`` triples.
        width_px / height_px: the widget's size.
        margins: ``(left, top, right, bottom)`` pixels to reserve, e.g. for
            row/column header labels.
        footprint: the plate outline as ``(x0, y0, x1, y1)`` mm in the same
            A1-relative frame (see
            ``PlateDocumentStore.plate_footprint_extent_mm``). When given, the
            fit frames the real plate; otherwise it frames the wells.
        min_radius_px: floor passed through to
            :meth:`PlateTransform.radius_px`.

    Returns:
        A :class:`PlateTransform`, or None when there is nothing to draw or no
        room to draw it. **Each well's own radius is included in the extent**,
        so wells are never clipped at the edges — the failure mode of a fit
        computed from centres alone.
    """
    items = list(wells)
    if not items:
        return None

    left, top, right, bottom = margins
    avail_w = float(width_px) - left - right
    avail_h = float(height_px) - top - bottom
    if avail_w <= 0 or avail_h <= 0:
        return None

    if footprint is not None:
        x0, y0, x1, y1 = (float(v) for v in footprint)
    else:
        xs0, ys0, xs1, ys1 = [], [], [], []
        for item in items:
            x, y, d = (item[1], item[2], item[3]) if len(item) >= 4 else item[:3]
            r = float(d) / 2.0
            xs0.append(float(x) - r)
            ys0.append(float(y) - r)
            xs1.append(float(x) + r)
            ys1.append(float(y) + r)
        x0, y0, x1, y1 = min(xs0), min(ys0), max(xs1), max(ys1)

    span_x = x1 - x0
    span_y = y1 - y0
    # A single well (or a degenerate footprint) has no span; give it one so the
    # scale stays finite and the well lands in the middle of the widget.
    if span_x <= 0 and span_y <= 0:
        span_x = span_y = max(
            (float(item[3] if len(item) >= 4 else item[2]) for item in items),
            default=1.0,
        ) or 1.0
    span_x = max(span_x, 1e-6)
    span_y = max(span_y, 1e-6)

    scale = min(avail_w / span_x, avail_h / span_y)
    if not math.isfinite(scale) or scale <= 0:             # pragma: no cover
        return None

    ox = left + (avail_w - span_x * scale) / 2.0 - x0 * scale
    oy = top + (avail_h - span_y * scale) / 2.0 - y0 * scale
    return PlateTransform(scale=scale, ox=ox, oy=oy,
                          min_radius_px=float(min_radius_px))
