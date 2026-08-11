"""
MosaicWellRemap — derive well NAME → absolute-stage-µm from a ground-truth mosaic.

The full-plate mosaic is stitched at TRUSTED absolute stage positions (Prior
encoders), so every detected well CENTRE is ground truth in the stage frame.
Which NAME binds to which centre is purely the per-machine plate ORIENTATION
(``StageController.plate_axis_sign``): A1 sits at the corner the convention
calls "top-left". With ``plate_axis_sign == (-1, -1)`` (ME3B V1: plate mounted
180° to the stage — origin bottom-right, well A1 top-left) A1 is the
MAX-stageX / MAX-stageY well and column/row indices increase toward DECREASING
stage X/Y. With ``(1, 1)`` the plate axes are aligned (A1 at the min corner).

This module is GUI-/hardware-free and commands NO motion — it only reads an
image + plate geometry and returns a ``{name: (x_um, y_um)}`` dict in absolute
stage µm. Used by:
  * the Calibration "Re-derive wells from saved mosaic (ground truth)" action,
  * one-shot regeneration of a stale calibration after an orientation change.

The matching (nearest-unused detection to the orientation-anchored predicted
grid) mirrors ``CalibrationPage._ploc_fit_from_mosaic_detections`` so the
behaviour is identical to the live mosaic fit — only the well NAMES change with
the orientation sign; the detected POSITIONS are fixed ground truth.
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)


def _plate_tolerance_um(plate) -> float:
    """Match tolerance (µm), mirroring CalibrationPage._ploc_mosaic_metrics:
    max(0.6·well_diameter, 0.4·min pitch)."""
    try:
        well_d_um = float(plate.well_diameter) * 1000.0
    except Exception:
        well_d_um = 0.0
    if well_d_um <= 0:
        well_d_um = 6000.0
    try:
        pitch_um = min(float(plate.well_spacing_x),
                       float(plate.well_spacing_y)) * 1000.0
    except Exception:
        pitch_um = well_d_um
    if pitch_um <= 0:
        pitch_um = well_d_um
    return max(well_d_um * 0.6, pitch_um * 0.4)


def orient_lattice_index(row, col, rows, cols, plate_axis_sign=(1.0, 1.0)):
    """Detector pixel-lattice (row, col) → PLATE (row, col).

    v7.9.1. ``PlateWellDetector.fit_lattice`` canonicalises its solution to the
    near-0° branch, so its **col increases with +pixel-x and row with +pixel-y**
    — and a mosaic canvas is built with pixel (0,0) at MIN stage X / MIN stage Y
    (``MosaicBuilder`` applies no Y inversion). Its own comment is explicit that
    *which corner is really A1 stays the caller's decision, made from the
    plate-orientation convention, not from pixels*: a 180°-rotated lattice fits
    the image exactly as well, and nothing in the picture can break the tie.

    That decision is this rule, and it is the same one
    :func:`label_positions` makes — A1 sits at the MAX coordinate of any axis
    whose ``plate_axis_sign`` is negative, because a plate mounted 180° to the
    stage has ``+col → −stage X`` and ``+row → −stage Y``. The mapping dialog's
    auto-detect used the raw detector indices instead, so on this rig
    (``plate_flip_180=True`` ⇒ sign ``(-1,-1)``) every name landed on the
    diagonally opposite well and A1 appeared bottom-right.

    Per-axis, not a single 180° rotation: each axis flips independently on its
    own sign, and only both-negative reduces to a rotation.
    """
    sx = -1.0 if float(plate_axis_sign[0]) < 0 else 1.0
    sy = -1.0 if float(plate_axis_sign[1]) < 0 else 1.0
    out_col = (int(cols) - 1 - int(col)) if sx < 0 else int(col)
    out_row = (int(rows) - 1 - int(row)) if sy < 0 else int(row)
    return out_row, out_col


def label_positions(positions, plate, plate_axis_sign=(1.0, 1.0)) -> dict:
    """Assign well names to ground-truth absolute-µm detected centres.

    ``positions``: iterable of ``(x_um, y_um)`` absolute stage µm — the detected
    well centres (any prior names are IGNORED; they are re-derived here).
    ``plate``: a ``WellPlate``. ``plate_axis_sign``: the per-machine
    plate-local→stage sign (``StageController.plate_axis_sign()``).

    Returns ``{name: (x_um, y_um)}`` for every plate well that matched a
    detection within tolerance (nearest-unused match against the
    orientation-anchored predicted grid). The 24 positions are unchanged; only
    which NAME binds to which position depends on ``plate_axis_sign``.
    """
    pts = [(float(x), float(y)) for (x, y) in positions]
    if not pts or plate is None:
        return {}
    sx = -1.0 if float(plate_axis_sign[0]) < 0 else 1.0
    sy = -1.0 if float(plate_axis_sign[1]) < 0 else 1.0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    # Anchor A1 (col 0, row 0) at the bbox corner the orientation calls A1: for
    # a NEGATIVE sign that axis's A1 end is the MAX coordinate, else the MIN.
    a1x = max(xs) if sx < 0 else min(xs)
    a1y = max(ys) if sy < 0 else min(ys)
    predicted = plate.get_all_positions_from_a1(a1x, a1y, (sx, sy))
    tol = _plate_tolerance_um(plate)
    results: dict[str, tuple[float, float]] = {}
    used: set[int] = set()
    for name, (px, py) in predicted.items():
        best, best_d = None, tol
        for i, (dx, dy) in enumerate(pts):
            if i in used:
                continue
            d = math.hypot(dx - px, dy - py)
            if d < best_d:
                best_d, best = d, i
        if best is not None:
            used.add(best)
            results[name] = pts[best]
    return results


def detect_raw_positions(image_bgr, extent_um, mosaic_scale, plate,
                         global_shift=(0.0, 0.0)) -> list:
    """Detect well centres in a saved mosaic and back-project to absolute µm.

    Returns a list of ``(x_um, y_um)`` ABSOLUTE stage µm — UNLABELED (the
    ground-truth detected centres). Detection mirrors
    ``CalibrationPage._ploc_detect_and_fit_from_mosaic`` (filled-disc blobs →
    grid-cull → Hough fallback); back-projection mirrors
    ``_ploc_fit_from_mosaic_detections`` (``extent[0:2] - global_shift +
    px/scale``). ``global_shift`` is ``(0, 0)`` for a saved mosaic.
    """
    if image_bgr is None or extent_um is None or not mosaic_scale:
        return []
    try:
        from SupportClasses.VisionDetector import WellDetector
    except Exception as e:  # pragma: no cover - import guard
        logger.warning("WellDetector unavailable: %s", e)
        return []
    rows = int(getattr(plate, "rows", 0) or 0)
    cols = int(getattr(plate, "cols", 0) or 0)
    dets: list = []
    try:
        filled = WellDetector.detect_filled_wells(image_bgr)
        if filled and rows and cols and len(filled) >= 4:
            centers = [(d.center_px[0], d.center_px[1]) for d in filled]
            _aff, assign = WellDetector.fit_well_grid(centers, rows, cols)
            if assign:
                filled = [filled[i] for i in sorted(set(assign.values()))]
        dets = filled
    except Exception as e:
        logger.debug("detect_filled_wells skipped: %s", e)
    if len(dets) < 3:
        try:
            well_d_um = float(getattr(plate, "well_diameter", 0) or 0) * 1000.0
            pitch_um = min(float(plate.well_spacing_x),
                           float(plate.well_spacing_y)) * 1000.0
            dets = WellDetector.detect_wells(
                image_bgr, max(1.0, well_d_um * mosaic_scale),
                min_dist_px=max(1.0, pitch_um * mosaic_scale * 0.7))
        except Exception as e:
            logger.warning("mosaic well detect failed: %s", e)
            return []
    gx = float(global_shift[0])
    gy = float(global_shift[1])
    ox = float(extent_um[0]) - gx
    oy = float(extent_um[1]) - gy
    return [(ox + float(d.center_px[0]) / mosaic_scale,
             oy + float(d.center_px[1]) / mosaic_scale) for d in dets]


def detect_well_positions(image_bgr, extent_um, mosaic_scale, plate,
                          plate_axis_sign=(1.0, 1.0),
                          global_shift=(0.0, 0.0)) -> dict:
    """Detect wells in a saved mosaic image and label them per orientation.

    Convenience: ``label_positions(detect_raw_positions(...), plate, sign)``.
    Returns ``{name: (x_um, y_um)}`` absolute stage µm.
    """
    pts = detect_raw_positions(image_bgr, extent_um, mosaic_scale, plate,
                               global_shift)
    return label_positions(pts, plate, plate_axis_sign)
