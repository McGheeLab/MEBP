"""
responsive.py — container-width-driven adaptive sizing (a "CSS clamp()" for Qt).

v7.5.x: The left context panel content is meant to be *sized to fit* — text and
buttons grow on a wide panel and shrink (to a readable floor) on a narrow one,
rather than forcing the panel to a fixed width. This module provides the small
primitive that makes that possible: a scale factor derived from a widget's
actual width relative to a design width, clamped to a readable range — the Qt
analogue of a CSS container query / ``clamp()``.

Widgets call ``container_scale(self.width(), design_px)`` on resize and multiply
their base (design-time, DPI-scaled) dimensions and font sizes by it. Combined
with aligned grids whose buttons/fields *expand to fill* the width, this lets the
content fit any panel width without horizontal scrolling: the controls scrunch
and their text shrinks together as the panel narrows.

The DPI scale (``gui.scaling.scale_factor``) still applies underneath — this is
a *second*, per-container factor on top of it, so a value scales as
``base_px * scale_factor() * container_scale(...)``.
"""

from __future__ import annotations


# Readable floor / generous ceiling for the per-container factor. At the design
# width the factor is exactly 1.0 (identical to the pre-responsive appearance).
DEFAULT_MIN_SCALE = 0.72
DEFAULT_MAX_SCALE = 1.30


def container_scale(width_px: float, design_px: float,
                    lo: float = DEFAULT_MIN_SCALE,
                    hi: float = DEFAULT_MAX_SCALE) -> float:
    """Return a scale factor for a container of ``width_px`` whose content is
    laid out for ``design_px``.

    ``1.0`` at the design width; below it shrinks toward ``lo``; above it grows
    toward ``hi``. Degrades to ``1.0`` for non-positive inputs (e.g. a widget
    that has not been sized yet), so callers can apply it unconditionally.
    """
    if width_px <= 0 or design_px <= 0:
        return 1.0
    return max(lo, min(hi, width_px / design_px))


def quantize(scale: float, step: float = 0.02) -> float:
    """Round a scale to a coarse step so resize handlers can cheaply skip
    re-applying identical sizing (and avoid resize→relayout→resize churn)."""
    if step <= 0:
        return scale
    return round(scale / step) * step


def scale_descendant_fonts(root, factor: float, *, skip_subtrees=(),
                           min_pt: float = 4.5, default_pt: float = 9.0) -> None:
    """Scale the point size of every descendant widget's font by ``factor``,
    relative to each widget's *base* size (recorded once on the widget itself).

    This is the "shrink the text by a %% of the width" pass used to make a whole
    panel fit a narrow container: call it from the container's ``resizeEvent``
    with ``factor = container_scale(width, design)``. Every label / button /
    spin / field font then shrinks (or grows) together with the width.

    Notes:
      * A widget whose stylesheet hard-codes ``font-size`` is unaffected (QSS
        wins over ``setFont``); strip ``font-size`` from those stylesheets if
        you want them to scale.
      * Subtrees in ``skip_subtrees`` are left untouched — pass any child that
        scales itself (e.g. a nested JogButtonArray) so it isn't double-scaled.
    """
    from PySide6.QtWidgets import QWidget
    skip = set()
    for st in skip_subtrees:
        if st is None:
            continue
        skip.add(st)
        for child in st.findChildren(QWidget):
            skip.add(child)
    for w in root.findChildren(QWidget):
        if w in skip:
            continue
        base = w.property("_rf_base_pt")
        if base is None or float(base) <= 0:
            base = w.font().pointSizeF()
            if base <= 0:
                base = default_pt
            w.setProperty("_rf_base_pt", float(base))
        else:
            base = float(base)
        f = w.font()
        f.setPointSizeF(max(min_pt, base * factor))
        w.setFont(f)
