"""
FluorescencePostProcess.py — optional denoise + background subtraction for
fluorescence channel stitches (v7.13).

Applied AT SAVE TIME to a COPY of the stitched channel image: the raw stitch
stays on disk as the canonical record, a ``*_proc.png`` sibling carries the
processed version, and the applied settings are recorded in the store metadata
so a processed image can always say what was done to it.

Background subtraction is the rolling-ball approximation via a grayscale
morphological OPENING with an elliptical kernel: the opening erases every
structure smaller than the ball, leaving the smooth background (vignetting,
haze, illumination gradient), which is then subtracted. For the large radii
this implies on a 3000-px stitch, the opening runs on a downscaled copy and
the background is upscaled before subtraction — the error is far below the
structures being removed, and it turns a multi-second operation into tens of
milliseconds.

Pure: numpy + cv2 only, no Qt.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

try:
    import numpy as np
    import cv2
    _CV_OK = True
except ImportError:                      # pragma: no cover
    _CV_OK = False


DENOISE_MODES = ("off", "median", "gaussian")

DEFAULTS = {
    "denoise": "off",
    "denoise_strength": 1,       # 1..3
    "bg_subtract": False,
    "bg_radius_um": 100.0,       # rolling-ball radius, sample-side µm
}

#: Openings with kernels larger than this run on a downscaled copy.
_MAX_FULLRES_KERNEL_PX = 31
_DOWNSCALE = 8


def normalize_settings(d: dict | None) -> dict:
    """Coerce + clamp a settings dict to the documented shape."""
    d = dict(d or {})
    out = dict(DEFAULTS)
    mode = str(d.get("denoise", out["denoise"])).lower()
    out["denoise"] = mode if mode in DENOISE_MODES else "off"
    try:
        out["denoise_strength"] = max(1, min(3, int(d.get(
            "denoise_strength", out["denoise_strength"]))))
    except (TypeError, ValueError):
        pass
    out["bg_subtract"] = bool(d.get("bg_subtract", out["bg_subtract"]))
    try:
        out["bg_radius_um"] = max(1.0, float(d.get(
            "bg_radius_um", out["bg_radius_um"])))
    except (TypeError, ValueError):
        pass
    return out


def is_active(settings: dict | None) -> bool:
    s = normalize_settings(settings)
    return s["denoise"] != "off" or s["bg_subtract"]


def denoise(gray, mode: str, strength: int = 1):
    """Mild denoise of a 2-D image. Preserves dtype and shape.

    median: kernel 3/5/7 for strength 1/2/3 (edge-preserving, kills shot-noise
    speckle). gaussian: σ = 0.5·strength (gentler, slightly blurs edges).
    """
    if not _CV_OK or gray is None or mode not in ("median", "gaussian"):
        return gray
    strength = max(1, min(3, int(strength)))
    if mode == "median":
        k = {1: 3, 2: 5, 3: 7}[strength]
        # cv2.medianBlur supports uint8 at any kernel; uint16 only at k=3/5.
        if gray.dtype != np.uint8 and k > 5:
            k = 5
        return cv2.medianBlur(gray, k)
    sigma = 0.5 * strength
    return cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma, sigmaY=sigma)


def subtract_background(gray, radius_px: float):
    """Rolling-ball-style background flattening via morphological opening.

    Removes smooth background (vignetting/haze) while preserving structures
    smaller than ``radius_px``. Preserves dtype and shape.
    """
    if not _CV_OK or gray is None:
        return gray
    r = max(3, int(round(float(radius_px))))
    k = 2 * r + 1
    if k <= _MAX_FULLRES_KERNEL_PX:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        background = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
    else:
        # Large ball: open a downscaled copy, upscale the background. The
        # background is smooth BY DEFINITION (that is what the opening keeps),
        # so the interpolation error is negligible against the removed haze.
        h, w = gray.shape[:2]
        sw = max(8, w // _DOWNSCALE)
        sh = max(8, h // _DOWNSCALE)
        small = cv2.resize(gray, (sw, sh), interpolation=cv2.INTER_AREA)
        rs = max(1, int(round(r * sw / float(w))))
        ks = 2 * rs + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
        bg_small = cv2.morphologyEx(small, cv2.MORPH_OPEN, kernel)
        background = cv2.resize(bg_small, (w, h),
                                interpolation=cv2.INTER_LINEAR)
    return cv2.subtract(gray, background)


def process(gray, settings: dict | None, um_per_px: float | None = None
            ) -> "tuple[object, dict]":
    """Apply the enabled steps to a 2-D image; ``(processed, applied)``.

    ``applied`` records exactly what ran (including the resolved pixel
    radius) — it is what the store persists next to the processed image.
    Denoise runs FIRST so the background estimate isn't biased by speckle.
    """
    s = normalize_settings(settings)
    applied: dict = {}
    out = gray
    if out is None or not _CV_OK:
        return gray, applied
    if s["denoise"] != "off":
        out = denoise(out, s["denoise"], s["denoise_strength"])
        applied["denoise"] = s["denoise"]
        applied["denoise_strength"] = s["denoise_strength"]
    if s["bg_subtract"]:
        upp = float(um_per_px or 0.0)
        radius_px = (s["bg_radius_um"] / upp) if upp > 0 else 50.0
        radius_px = max(3.0, radius_px)
        out = subtract_background(out, radius_px)
        applied["bg_subtract"] = True
        applied["bg_radius_um"] = s["bg_radius_um"]
        applied["bg_radius_px"] = round(radius_px, 2)
    return out, applied
