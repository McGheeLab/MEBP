"""
mono_display.py — Shared mono→BGR8 display conversion for scientific cameras.

v7.9.x: Extracted verbatim from ``andor_backend`` (where it shipped and was
hardware-verified on the ANDOR Zyla) so that EVERY mono scientific camera in
the app renders through the SAME display math.

Why this is shared rather than copied
-------------------------------------
The rest of the app consumes 8-bit BGR frames, but a scientific sensor delivers
mono 12/16-bit. Turning those counts into something visible is a *display*
decision (per-frame percentile auto-scale, or fixed black/white levels), and it
materially changes how bright/contrasty a camera looks.

The operator evaluates one mono camera AGAINST another (Tucsen Libra vs. ANDOR
Zyla) in the same MICROSCOPE role. If each backend carried its own copy of this
conversion, the two could drift apart and the A/B comparison would be measuring
our display math instead of the sensors. One implementation makes the comparison
about the cameras.

This module is pure (numpy + optional cv2), GUI-free, and imports no SDK.

``andor_backend`` re-exports ``_auto_levels`` / ``_mono_to_bgr8`` from here, so
existing references to ``andor_backend._mono_to_bgr8`` keep resolving.
"""

from __future__ import annotations

try:
    import numpy as np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False


# Full-scale raw count for a 16-bit sensor readout — the upper bound offered for
# manual black/white display levels.
LEVEL_MAX = 65535


def _auto_levels(arr) -> "tuple[float, float]":
    """1–99 percentile (lo, hi) of a 2-D plane, from a DECIMATED sample.

    Percentile sorts, so doing it on the full 4.2M-px frame every tick is
    costly; the sample bounds the cost while the scale is applied full-frame.
    """
    step = max(1, int(max(arr.shape) // 512))
    sample = arr[::step, ::step]
    try:
        lo, hi = np.percentile(sample, (1.0, 99.0))
    except Exception:
        lo, hi = float(arr.min()), float(arr.max())
    return float(lo), float(hi)


def _mono_to_bgr8(frame, levels=None) -> "np.ndarray | None":
    """Convert a mono (2-D) uint16/uint8 frame to an 8-bit BGR image.

    ``levels=(lo, hi)`` maps those raw counts to display 0..255 (fixed manual
    scaling); ``levels=None`` keeps the per-frame 1–99 percentile auto-scale so
    dim (e.g. fluorescence) scenes remain visible. Already-BGR frames pass
    through unchanged.
    """
    if not _NP_AVAILABLE or frame is None:
        return None
    try:
        arr = np.asarray(frame)
    except Exception:
        return None

    # Already a 3-channel 8-bit image → pass through.
    if arr.ndim == 3 and arr.shape[2] == 3:
        return arr.astype(np.uint8, copy=False)

    # Reduce anything else to a single 2-D plane.
    if arr.ndim == 3:
        arr = arr[..., 0]
    if arr.ndim != 2:
        return None

    if arr.dtype == np.uint8:
        gray8 = arr
    else:
        if levels is not None:
            lo, hi = float(levels[0]), float(levels[1])
        else:
            lo, hi = _auto_levels(arr)
        if not (hi > lo):
            hi = lo + 1.0
        f = arr.astype(np.float32)
        gray8 = np.clip((f - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    try:
        import cv2
        return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
    except Exception:
        # numpy fallback: stack the plane into 3 channels.
        return np.repeat(gray8[:, :, None], 3, axis=2)
