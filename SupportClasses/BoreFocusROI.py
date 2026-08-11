"""
BoreFocusROI.py — per-bore focus regions for the microscope bore calibration.

v7.10. Measuring each bore's own Z means scoring each bore's own sharpness, so
every bore needs a region of interest around the pixel where its tip was
clicked. ``NeedleDetector.compute_focus_score`` already takes an ``roi_rect``,
so one frame scores every bore at once — which is the whole reason the operator
can jog Z and watch all bores respond together.

THE ONE THING THAT MATTERS HERE: the ROIs must not overlap. Fused bores sit
100-500 µm apart (decision D7), which at a typical 0.5 µm/px is 200-1000 px —
comparable to a generously sized ROI. If bore 1's box reaches into bore 2's tip,
both boxes peak at whichever tip is sharper and every bore reports the SAME
best-focus Z. That reproduces the exact bug this work exists to fix (all bores
in one Z plane) by a different route, and it would look like a real measurement.

So the boxes shrink to stay clear of their nearest neighbour, and the shrink is
reported so the UI can tell the operator when the bores are too close to resolve
at this magnification.

Zero GUI dependencies (math only).
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

#: An ROI smaller than this is dominated by noise rather than tip structure.
MIN_ROI_PX = 24
#: Default box side as a multiple of the bore's outer diameter — enough to hold
#: the tip plus the defocus halo that carries most of the focus signal.
DEFAULT_ROI_OD_MULTIPLE = 3.0
#: Keep this fraction of the gap between neighbours as clear space, so two
#: boxes never touch even after rounding.
NEIGHBOUR_CLEARANCE = 0.9


def _finite(value, default: float = 0.0) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


def nearest_neighbour_px(point, others: Sequence) -> Optional[float]:
    """Chebyshev distance to the closest of ``others`` (px), or None if none.

    ⚠ CHEBYSHEV — ``max(|dx|, |dy|)`` — not Euclidean, because the ROIs are
    AXIS-ALIGNED SQUARES. Two squares of side ``s`` centred ``d`` apart are
    disjoint only when ``max(|dx|, |dy|) >= s``; the Euclidean distance can be
    comfortably larger than ``s`` while both axis separations are smaller, so
    sizing against it lets diagonal neighbours overlap.

    Found by ``test_a_triple_never_overlaps``: a triple at (900,500), (1100,500)
    and (1000,660) has a Euclidean gap of 189 px between bores 0 and 2, which
    admitted a 170 px box whose x and y separations are only 100 and 160 px —
    overlapping, which would have made those two bores report the same focus
    peak and silently recreate the all-bores-in-one-Z-plane bug.
    """
    px, py = _finite(point[0]), _finite(point[1])
    best = None
    for o in others or ():
        if o is None:
            continue
        d = max(abs(_finite(o[0]) - px), abs(_finite(o[1]) - py))
        if d <= 0.0:
            continue
        best = d if best is None else min(best, d)
    return best


def bore_roi_rect(click_px,
                  frame_wh,
                  um_per_px: float,
                  bore_od_um: float,
                  others: Sequence = (),
                  od_multiple: float = DEFAULT_ROI_OD_MULTIPLE,
                  min_px: int = MIN_ROI_PX) -> tuple[int, int, int, int]:
    """``(x, y, w, h)`` focus ROI centred on a bore-tip click, in RAW frame px.

    Sized from the bore's own outer diameter so it scales with magnification,
    then shrunk if a neighbouring bore's click is closer than half the box — see
    the module docstring for why an overlapping box silently destroys the
    measurement. Always clamped inside the frame, and never below ``min_px``.

    ``others`` are the other bores' click pixels, in the same frame.
    """
    fw = max(1, int(_finite(frame_wh[0], 1.0)))
    fh = max(1, int(_finite(frame_wh[1], 1.0)))
    cx = _finite(click_px[0])
    cy = _finite(click_px[1])

    upp = _finite(um_per_px)
    od = _finite(bore_od_um)
    side = (od_multiple * od / upp) if (upp > 0 and od > 0) else float(min_px)

    # Never let a box reach its neighbour's tip.
    gap = nearest_neighbour_px((cx, cy), others)
    if gap is not None:
        side = min(side, NEIGHBOUR_CLEARANCE * gap)

    side = max(float(min_px), side)
    side = min(side, float(min(fw, fh)))
    half = side / 2.0

    x = int(round(cx - half))
    y = int(round(cy - half))
    w = h = int(round(side))
    # Clamp inside the frame without changing the size, so the score stays
    # comparable between bores.
    x = max(0, min(x, fw - w))
    y = max(0, min(y, fh - h))
    return (x, y, max(1, w), max(1, h))


def rois_for_clicks(clicks,
                    frame_wh,
                    um_per_px: float,
                    bore_od_um: float,
                    od_multiple: float = DEFAULT_ROI_OD_MULTIPLE,
                    min_px: int = MIN_ROI_PX) -> dict:
    """``{bore_index: (x, y, w, h)}`` for every recorded click.

    ``clicks`` maps bore index → pixel. Each box is sized against all the
    OTHERS, so adding a bore can tighten its neighbours — recompute the whole
    set rather than caching one box at a time.
    """
    out = {}
    items = [(k, p) for k, p in dict(clicks or {}).items() if p is not None]
    for k, p in items:
        others = [q for j, q in items if j != k]
        out[k] = bore_roi_rect(p, frame_wh, um_per_px, bore_od_um,
                               others=others, od_multiple=od_multiple,
                               min_px=min_px)
    return out


def rois_overlap(a, b) -> bool:
    """True when two ``(x, y, w, h)`` rectangles intersect at all."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not (ax + aw <= bx or bx + bw <= ax
                or ay + ah <= by or by + bh <= ay)


def view_px_to_frame_px(click_px, view_wh, frame_wh) -> tuple[float, float]:
    """Rescale a click from the view's image frame into RAW frame pixels.

    ``CameraFeedView`` reports clicks in the coordinates of the image it is
    displaying, which is normally the raw frame but need not be; the focus score
    runs on ``get_current_frame()``, which always is. Mirrors the rescale
    ``_ploc_save_reanchor_feature`` does for the same reason.
    """
    vw = _finite(view_wh[0], 0.0)
    vh = _finite(view_wh[1], 0.0)
    fw = _finite(frame_wh[0], 0.0)
    fh = _finite(frame_wh[1], 0.0)
    if vw <= 0 or vh <= 0:
        return (_finite(click_px[0]), _finite(click_px[1]))
    return (_finite(click_px[0]) * (fw / vw), _finite(click_px[1]) * (fh / vh))
