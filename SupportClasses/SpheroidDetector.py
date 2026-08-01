"""
SpheroidDetector.py — find + measure spheroids on a stitched well mosaic.

v7.8: the Spheroid Pick & Place workflow can survey a whole well from one
fluorescence mosaic instead of the operator eyeballing every target. This module
is the pure half: given a mosaic image and the geometry that positions it in the
stage frame, return every roughly-circular object whose FITTED diameter falls in
an operator-set **absolute µm** window, with its centre back-projected to
absolute stage µm.

GUI-free, hardware-free, commands no motion. The headless contract (and the
``global_shift`` kwarg on the positional prefix) mirrors ``MosaicWellRemap`` so
the two read the same way.

Why a sibling of ``WellDetector.detect_filled_wells`` and not an extension of it
-------------------------------------------------------------------------------
``detect_filled_wells`` gates blobs *relatively* — ``min_area_frac`` of the
LARGEST blob, floor 50 px², with no maximum. That is right for "find the N
biggest discs on a plate mosaic" and wrong here for two reasons: the operator's
rule is an absolute µm band, and the biggest bright blob in a single well is
usually the **well wall / meniscus ring**, which both survives the filter and
raises the floor high enough to erase every real spheroid. It is also on the
plate-location calibration path (``MosaicWellRemap.detect_raw_positions``) whose
well centres command motion, so changing its semantics is a needle-crash-adjacent
edit. We reuse its *primitives* (``VisionDetector.fit_circle_robust``) instead.

Two shape gates, deliberately named apart
-----------------------------------------
``detect_filled_wells``' ``circularity_min`` is **not** circularity — it is the
fraction of contour points explained by the fitted circle (see
``VisionDetector.py`` ``_fit_circle_robust``), and its 0.55 default is tuned to
accept a *partial well arc*. Copying that name or that number gives a detector
that happily accepts two merged spheroids. So:

    * ``fit_fraction_min``  — that inlier fraction.
    * ``circularity_min``   — genuine isoperimetric ``4πA/P²``. This is what a
      biologist means by circularity, and what rejects a doublet: a peanut's
      min-enclosing-circle inlier fraction can look fine while its compactness
      cannot.

Back-projection
---------------
``back_project_px`` is the ONE px → stage-µm implementation in this feature —
shared by detection, the mosaic view's click handling and the radius-drag
write-back::

    x_um = (extent[0] - shift[0]) + px_x / mosaic_scale
    y_um = (extent[1] - shift[1]) + px_y / mosaic_scale

The ``- shift`` is not optional. ``MosaicBuilder.canvas_extent_um`` ADDS the
global registration shift to the extent while the tile pixels stay in the
trusted raw stage frame (``MosaicStore.save`` documents the same contract), and
that shift is bounded by 20 % of the FOV width — hundreds of µm at 10×, i.e.
larger than a spheroid. Callers that drive the stage must also confirm the shift
was actually RECORDED (``FluorescenceMosaicStore.has_shift``); a legacy mosaic
reports (0, 0) whether or not that is true.

Deliberate v1 limit: touching spheroids are NOT split. A doublet exceeds
``max_diameter_um`` and is dropped — literally the operator's stated rule — and
``DetectionReport``'s per-reason counters make that visible rather than
mysterious. Watershed splitting on fluorescent spheroids over-segments and would
ship confidently-wrong diameters.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

logger = logging.getLogger(__name__)

try:
    import cv2
    import numpy as np
    _CV2 = True
except ImportError:   # pragma: no cover - cv2/numpy always present here
    cv2 = None
    np = None
    _CV2 = False


# Below this the fitted radius is quantisation noise, not a measurement: a disc
# only a handful of px across cannot yield a diameter worth aspirating from.
MIN_RESOLVABLE_PX = 6.0

# Default absolute size window (µm). Spheroids much under ~50 µm are usually
# debris or single cells; much over ~500 µm rarely clears a needle bore.
DEFAULT_MIN_DIAMETER_UM = 80.0
DEFAULT_MAX_DIAMETER_UM = 400.0

DEFAULT_CIRCULARITY_MIN = 0.80
DEFAULT_FIT_FRACTION_MIN = 0.60

THRESHOLD_MODES = ("triangle", "otsu", "fixed")


# ── result types ───────────────────────────────────────────────────

@dataclass
class SpheroidDetection:
    """One measured spheroid.

    ``center_px`` / ``radius_px`` are mosaic pixels (scene coords 1:1, so a GUI
    can draw the circle straight onto the mosaic). ``center_um`` is ABSOLUTE
    stage µm with the registration shift already removed. ``diameter_um`` is the
    measurement of record — the operator may overwrite it by redrawing, which
    sets ``user_edited``.
    """
    center_px: tuple[float, float]
    radius_px: float
    diameter_um: float
    center_um: tuple[float, float]
    circularity: float = 0.0
    fit_fraction: float = 0.0
    area_px: float = 0.0
    confidence: float = 0.0
    det_id: str = ""
    source: str = "auto"          # "auto" | "user"
    user_edited: bool = False

    def to_dict(self) -> dict:
        return {
            "center_px": [float(self.center_px[0]), float(self.center_px[1])],
            "radius_px": float(self.radius_px),
            "diameter_um": float(self.diameter_um),
            "center_um": [float(self.center_um[0]), float(self.center_um[1])],
            "circularity": float(self.circularity),
            "fit_fraction": float(self.fit_fraction),
            "area_px": float(self.area_px),
            "confidence": float(self.confidence),
            "det_id": str(self.det_id),
            "source": str(self.source),
            "user_edited": bool(self.user_edited),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SpheroidDetection":
        cp = d.get("center_px") or (0.0, 0.0)
        cu = d.get("center_um") or (0.0, 0.0)
        return cls(
            center_px=(float(cp[0]), float(cp[1])),
            radius_px=float(d.get("radius_px", 0.0)),
            diameter_um=float(d.get("diameter_um", 0.0)),
            center_um=(float(cu[0]), float(cu[1])),
            circularity=float(d.get("circularity", 0.0)),
            fit_fraction=float(d.get("fit_fraction", 0.0)),
            area_px=float(d.get("area_px", 0.0)),
            confidence=float(d.get("confidence", 0.0)),
            det_id=str(d.get("det_id", "")),
            source=str(d.get("source", "auto")),
            user_edited=bool(d.get("user_edited", False)),
        )


@dataclass
class DetectionReport:
    """Detections PLUS why everything else was rejected.

    The counters are not diagnostics padding: on a nuclear-stain channel a
    spheroid is a cluster of puncta rather than a filled disc, so the two
    realistic failures are "0 found" and "hundreds found". A bare count cannot
    tell the operator whether to widen the band, switch channel, or add targets
    by hand — ``summary()`` can.
    """
    detections: list[SpheroidDetection] = field(default_factory=list)
    n_blobs: int = 0
    n_too_small: int = 0
    n_too_large: int = 0
    n_shape: int = 0
    n_outside_well: int = 0
    n_unfittable: int = 0
    truncated: bool = False
    refused: str = ""

    def __len__(self) -> int:
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)

    def summary(self) -> str:
        if self.refused:
            return self.refused
        parts = [f"saw {self.n_blobs} blob(s) → {len(self.detections)} in band"]
        for n, label in ((self.n_too_small, "too small"),
                         (self.n_too_large, "too large"),
                         (self.n_shape, "not round"),
                         (self.n_outside_well, "outside well"),
                         (self.n_unfittable, "unfittable")):
            if n:
                parts.append(f"{n} {label}")
        out = " · ".join(parts)
        if self.truncated:
            out += " (list truncated)"
        return out


# ── projection (the single implementation) ─────────────────────────

def back_project_px(px_xy, extent_um, mosaic_scale: float,
                    global_shift=(0.0, 0.0)) -> tuple[float, float]:
    """Mosaic pixel → ABSOLUTE stage µm.

    ``extent_um`` is the mosaic's stored world extent (which INCLUDES the
    registration shift) and ``mosaic_scale`` is px per µm. Subtracting the shift
    recovers the trusted raw stage frame the tile pixels actually sit in — the
    same correction ``calibration._ploc_fit_from_mosaic_detections`` and
    ``MosaicWellRemap.detect_raw_positions`` apply.
    """
    scale = float(mosaic_scale)
    if scale <= 0:
        raise ValueError("mosaic_scale must be > 0 to back-project")
    ox = float(extent_um[0]) - float(global_shift[0])
    oy = float(extent_um[1]) - float(global_shift[1])
    return (ox + float(px_xy[0]) / scale, oy + float(px_xy[1]) / scale)


def forward_project_um(xy_um, extent_um, mosaic_scale: float,
                       global_shift=(0.0, 0.0)) -> tuple[float, float]:
    """Absolute stage µm → mosaic pixel (exact inverse of :func:`back_project_px`)."""
    scale = float(mosaic_scale)
    if scale <= 0:
        raise ValueError("mosaic_scale must be > 0 to project")
    ox = float(extent_um[0]) - float(global_shift[0])
    oy = float(extent_um[1]) - float(global_shift[1])
    return ((float(xy_um[0]) - ox) * scale, (float(xy_um[1]) - oy) * scale)


def px_window_for_um(min_diameter_um: float, max_diameter_um: float,
                     mosaic_scale: float) -> tuple[float, float]:
    """The absolute µm size window in mosaic pixels (``px = µm × scale``)."""
    scale = float(mosaic_scale)
    return (float(min_diameter_um) * scale, float(max_diameter_um) * scale)


def diameter_um_for_radius_px(radius_px: float, mosaic_scale: float) -> float:
    """Fitted mosaic radius → diameter in µm."""
    scale = float(mosaic_scale)
    if scale <= 0:
        return 0.0
    return 2.0 * float(radius_px) / scale


def radius_px_for_diameter_um(diameter_um: float, mosaic_scale: float) -> float:
    """Diameter in µm → mosaic radius in px (inverse of the above)."""
    return (float(diameter_um) / 2.0) * float(mosaic_scale)


def refuse_reason(mosaic_scale: float, min_diameter_um: float,
                  max_diameter_um: float) -> Optional[str]:
    """Why detection cannot run, in operator language — or None when it can.

    Refusing with a reason beats returning an unexplained zero: the commonest
    real failure is a mosaic whose resolution simply cannot resolve the requested
    smallest spheroid.
    """
    if not _CV2:
        return "OpenCV is not available, so detection cannot run."
    try:
        scale = float(mosaic_scale)
    except (TypeError, ValueError):
        scale = 0.0
    if scale <= 0:
        return ("This mosaic has no pixel scale recorded, so a pixel cannot be "
                "converted to µm. Re-scan the well.")
    try:
        lo = float(min_diameter_um)
        hi = float(max_diameter_um)
    except (TypeError, ValueError):
        return "The spheroid size range is not a pair of numbers."
    if lo <= 0 or hi <= 0:
        return "The spheroid size range must be positive."
    if lo >= hi:
        return (f"The size range is inverted: minimum {lo:.0f} µm is not below "
                f"maximum {hi:.0f} µm.")
    min_d_px = lo * scale
    if min_d_px < MIN_RESOLVABLE_PX:
        return (f"A {lo:.0f} µm spheroid is only {min_d_px:.1f} px across in "
                f"this mosaic — raise the mosaic resolution, or use a higher "
                f"objective, before detecting.")
    return None


# ── detection ──────────────────────────────────────────────────────

def _odd(n: int) -> int:
    n = int(n)
    return n if n % 2 else n + 1


def _threshold(bright, mode: str, abs_threshold):
    """Binary mask of "bright enough to be an object".

    Default TRIANGLE, not Otsu. Otsu assumes a bimodal histogram; a single-well
    fluorescence mosaic is ~95 % background, i.e. unimodal, and Otsu then picks a
    level *inside* the background and floods the mask. A ``mean + 3σ`` floor over
    the darker half guards the same failure for every mode.
    """
    mode = str(mode or "triangle").lower()
    if mode == "fixed" and abs_threshold is not None:
        thr = float(abs_threshold)
    elif mode == "otsu":
        thr, _ = cv2.threshold(bright, 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        thr, _ = cv2.threshold(bright, 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_TRIANGLE)
    vals = bright.reshape(-1)
    med = float(np.median(vals))
    bg = vals[vals <= med]
    if bg.size:
        floor = float(bg.mean()) + 3.0 * float(bg.std())
        thr = max(float(thr), floor)
    thr = min(max(float(thr), 1.0), 254.0)
    _, mask = cv2.threshold(bright, thr, 255, cv2.THRESH_BINARY)
    return mask, thr


def _circularity(contour) -> float:
    """Isoperimetric compactness ``4πA/P²`` — 1.0 for a circle, lower for a
    peanut. The metric that separates one spheroid from two touching ones."""
    per = float(cv2.arcLength(contour, True))
    if per <= 0:
        return 0.0
    area = float(cv2.contourArea(contour))
    return float(4.0 * math.pi * area / (per * per))


def detect_spheroids_px(
    image_bgr,
    *,
    min_diameter_px: float,
    max_diameter_px: float,
    circularity_min: float = DEFAULT_CIRCULARITY_MIN,
    fit_fraction_min: float = DEFAULT_FIT_FRACTION_MIN,
    threshold_mode: str = "triangle",
    abs_threshold=None,
    edge_margin_px: float = 0.0,
    max_results: int = 2048,
) -> DetectionReport:
    """Find roughly-circular blobs whose FITTED diameter is in a pixel window.

    Pure pixel space — no stage geometry. The gate is on the *fitted* diameter,
    not the blob area, because the fit is what becomes the measurement; the area
    test is only a loose prefilter (slack both ways, so morphology erosion or a
    slightly merged blob still reaches the fit and gets judged there).
    """
    report = DetectionReport()
    if not _CV2 or image_bgr is None or getattr(image_bgr, "size", 0) == 0:
        report.refused = "No image to detect on."
        return report
    min_d_px = float(min_diameter_px)
    max_d_px = float(max_diameter_px)
    if min_d_px <= 0 or max_d_px <= min_d_px:
        report.refused = "Invalid pixel size window."
        return report

    bright = (image_bgr.max(axis=2) if image_bgr.ndim == 3 else image_bgr)
    bright = cv2.GaussianBlur(bright, (5, 5), 0)
    mask, _thr = _threshold(bright, threshold_mode, abs_threshold)

    # Kernel scaled to the smallest thing we are looking for. A fixed 7×7 (what
    # detect_filled_wells uses) stops meaning anything once the mosaic scale
    # changes: it is a whole spheroid at one resolution and a speck at another.
    k_size = _odd(max(3, int(round(min_d_px * 0.3))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # The canvas carries half-FOV padding beyond the scanned bounds, so anything
    # touching the un-stitched margin is a clipped arc, not a spheroid. Restrict
    # to pixels that actually received image data, eroded by the margin.
    if edge_margin_px > 0:
        valid = (bright > 0).astype(np.uint8) * 255
        er = _odd(max(3, int(round(edge_margin_px))))
        valid = cv2.erode(
            valid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (er, er)))
        mask = cv2.bitwise_and(mask, valid)

    n, lbl, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return report

    area_lo = (math.pi / 4.0) * min_d_px * min_d_px * 0.5
    area_hi = (math.pi / 4.0) * max_d_px * max_d_px * 1.6

    for i in range(1, n):
        area = float(stats[i, cv2.CC_STAT_AREA])
        if area < max(4.0, area_lo):
            report.n_blobs += 1
            report.n_too_small += 1
            continue
        if area > area_hi:
            report.n_blobs += 1
            report.n_too_large += 1
            continue
        report.n_blobs += 1

        comp = (lbl == i).astype(np.uint8)
        cnts, _h = cv2.findContours(comp, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_NONE)
        if not cnts:
            report.n_unfittable += 1
            continue
        cnt = max(cnts, key=cv2.contourArea)
        circ = _circularity(cnt)

        rfit = _fit_circle(cnt)
        if rfit is None:
            report.n_unfittable += 1
            continue
        cx, cy, r, frac = rfit
        d_px = 2.0 * r

        # Shape first, so a merged doublet is reported as "not round" rather than
        # "too large" — the two call for different operator responses.
        if circ < float(circularity_min) or frac < float(fit_fraction_min):
            report.n_shape += 1
            continue
        if d_px < min_d_px:
            report.n_too_small += 1
            continue
        if d_px > max_d_px:
            report.n_too_large += 1
            continue

        report.detections.append(SpheroidDetection(
            center_px=(float(cx), float(cy)),
            radius_px=float(r),
            diameter_um=0.0,        # filled in by detect_spheroids
            center_um=(0.0, 0.0),
            circularity=float(circ),
            fit_fraction=float(frac),
            area_px=area,
            confidence=float(max(0.0, min(1.0, circ * frac))),
        ))
        if len(report.detections) >= int(max_results):
            report.truncated = True
            break
    return report


def _fit_circle(contour):
    """Chord-robust fit → ``(cx, cy, r, inlier_fraction)`` or None."""
    from SupportClasses.VisionDetector import fit_circle_robust
    pts = contour.reshape(-1, 2)
    fit = fit_circle_robust(pts)
    if fit is not None:
        return fit
    try:
        (cx, cy), r = cv2.minEnclosingCircle(contour)
        return float(cx), float(cy), float(r), 1.0
    except Exception:
        return None


def detect_spheroids(
    image_bgr,
    extent_um,
    mosaic_scale: float,
    *,
    min_diameter_um: float = DEFAULT_MIN_DIAMETER_UM,
    max_diameter_um: float = DEFAULT_MAX_DIAMETER_UM,
    global_shift=(0.0, 0.0),
    well_center_um=None,
    well_radius_um=None,
    well_margin_frac: float = 0.98,
    id_prefix: str = "S",
    **kw,
) -> DetectionReport:
    """Detect + measure + back-project, in one call.

    ``extent_um`` / ``mosaic_scale`` / ``global_shift`` position the mosaic in
    the stage frame (see :func:`back_project_px`). Supplying ``well_center_um``
    and ``well_radius_um`` masks detections outside the well — recommended,
    because the well wall / meniscus ring is the single largest false-positive
    source and its geometry is already known from the plate map.

    Results are sorted largest-first and given stable ``det_id``s.
    """
    refusal = refuse_reason(mosaic_scale, min_diameter_um, max_diameter_um)
    if refusal:
        rep = DetectionReport()
        rep.refused = refusal
        return rep

    scale = float(mosaic_scale)
    min_d_px, max_d_px = px_window_for_um(min_diameter_um, max_diameter_um, scale)
    report = detect_spheroids_px(
        image_bgr, min_diameter_px=min_d_px, max_diameter_px=max_d_px, **kw)
    if report.refused:
        return report

    kept: list[SpheroidDetection] = []
    r_limit = None
    if well_center_um is not None and well_radius_um:
        r_limit = float(well_radius_um) * float(well_margin_frac)
    for det in report.detections:
        det.diameter_um = diameter_um_for_radius_px(det.radius_px, scale)
        det.center_um = back_project_px(
            det.center_px, extent_um, scale, global_shift)
        if r_limit is not None:
            dx = det.center_um[0] - float(well_center_um[0])
            dy = det.center_um[1] - float(well_center_um[1])
            if math.hypot(dx, dy) > r_limit:
                report.n_outside_well += 1
                continue
        kept.append(det)

    kept.sort(key=lambda d: d.diameter_um, reverse=True)
    for n, det in enumerate(kept, start=1):
        det.det_id = f"{id_prefix}{n:03d}"
    report.detections = kept
    return report


def manual_detection(center_px, radius_px, extent_um, mosaic_scale: float,
                     global_shift=(0.0, 0.0), det_id: str = "") -> SpheroidDetection:
    """Build a hand-placed / hand-redrawn detection from mosaic geometry.

    Detection can legitimately find nothing (a nuclear stain is puncta, not a
    disc), so adding and sizing a spheroid by hand has to be a first-class path,
    not a fallback — this is the constructor for it.
    """
    scale = float(mosaic_scale)
    return SpheroidDetection(
        center_px=(float(center_px[0]), float(center_px[1])),
        radius_px=float(radius_px),
        diameter_um=diameter_um_for_radius_px(radius_px, scale),
        center_um=back_project_px(center_px, extent_um, scale, global_shift),
        circularity=1.0,
        fit_fraction=1.0,
        area_px=math.pi * float(radius_px) ** 2,
        confidence=1.0,
        det_id=str(det_id),
        source="user",
        user_edited=True,
    )


def resize_detection(det: SpheroidDetection, radius_px: float,
                     mosaic_scale: float) -> SpheroidDetection:
    """Apply an operator's redrawn radius (mosaic px) to a detection, in place."""
    det.radius_px = float(radius_px)
    det.diameter_um = diameter_um_for_radius_px(radius_px, mosaic_scale)
    det.user_edited = True
    return det


def recentre_detection(det: SpheroidDetection, center_px, extent_um,
                       mosaic_scale: float,
                       global_shift=(0.0, 0.0)) -> SpheroidDetection:
    """Apply an operator's dragged centre (mosaic px) to a detection, in place."""
    det.center_px = (float(center_px[0]), float(center_px[1]))
    det.center_um = back_project_px(center_px, extent_um, mosaic_scale,
                                    global_shift)
    det.user_edited = True
    return det


def sort_detections(dets: Sequence[SpheroidDetection], *, ascending: bool = False
                    ) -> list[SpheroidDetection]:
    """Detections by diameter — the operator's requested ordering."""
    return sorted(dets, key=lambda d: d.diameter_um, reverse=not ascending)
