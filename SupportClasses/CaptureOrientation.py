"""
CaptureOrientation.py — apply the camera's display orientation to an ndarray.

v7.14. Frames leave the camera RAW: `CameraFeedView._orient_qimage` applies the
measured mirror / flip / rotation at DISPLAY time only. So a capture that saves
what the operator is looking at must re-apply exactly that transform — from a
worker thread, without Qt.

This is the numpy twin of the Qt path, deliberately derived from the SAME
convention (`A = R(θ)·diag(mx,my)`) that `camera_feed_view.view_transform_coeffs`
anchors. A test composes the two pixel-for-pixel across every cardinal
combination, because a saved image that is silently mirrored relative to the
screen is the most likely — and least visible — defect in the capture feature.

Cardinal angles take a lossless `np.rot90`/`np.flip` fast path; only an
arbitrary angle resamples.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP = True
except ImportError:      # pragma: no cover
    _NP = False


def _cardinal(deg: float) -> "int | None":
    """The quarter-turn count for an angle within 0.01° of a multiple of 90."""
    try:
        d = float(deg) % 360.0
    except (TypeError, ValueError):
        return None
    for k in range(4):
        if abs(d - k * 90.0) < 0.01 or abs(d - k * 90.0 - 360.0) < 0.01:
            return k
    return None


def orient_array(arr, *, mirrored: bool = False, flip_y: bool = False,
                 rotation_deg: float = 0.0):
    """Return ``arr`` as the operator sees it on screen.

    Order matches the display path: mirror (horizontal), then flip_y
    (vertical), then rotate counter-clockwise by ``rotation_deg``. Works for
    2-D (mono 16-bit) and 3-D (BGR8) arrays alike, so a raw capture and a
    display capture are oriented by the same code.

    Never raises for an ORIENTATION failure: the array is returned unchanged
    and logged — a saved image with the wrong rotation is recoverable; a
    crashed capture loses the moment.

    ⚠ But an input that is not an image at all returns **None**, loudly.
    v7.15: this used to accept anything and hand it back, so when the video
    recorder passed a QImage the failure travelled on and detonated in
    ``_open_writer``'s ``h, w = img.shape[:2]`` — in a daemon thread with no
    ``try``, which stranded the whole recording. Defensive catching that moves
    a fault away from its cause is worse than no catching at all; refuse here,
    where the type is known.
    """
    if not _NP or arr is None:
        return None if arr is None else arr
    if not isinstance(arr, np.ndarray):
        logger.error(
            "orient_array got %s, not a numpy array — refusing rather than "
            "passing a non-image downstream", type(arr).__name__)
        return None
    if arr.ndim < 2:
        logger.error("orient_array got a %d-D array — not an image", arr.ndim)
        return None
    try:
        out = np.asarray(arr)
        if mirrored:
            out = np.flip(out, axis=1)
        if flip_y:
            out = np.flip(out, axis=0)
        k = _cardinal(rotation_deg)
        if k:
            # NEGATIVE k: Qt's y-axis points DOWN, so the display transform
            # A = R(θ)·diag(mx,my) renders CLOCKWISE on screen while np.rot90
            # turns counter-clockwise. Measured against the real
            # _orient_qimage — a saved image rotated the other way is the
            # defect the parity test exists to catch.
            out = np.rot90(out, -k)
        elif k is None:
            out = _rotate_arbitrary(out, -float(rotation_deg))
        return np.ascontiguousarray(out)
    except Exception as exc:      # pragma: no cover — defensive
        logger.warning(f"capture orientation failed ({exc}); saving raw")
        return arr


def _rotate_arbitrary(arr, deg: float):
    """Resample by a non-cardinal angle, expanding the canvas to fit.

    Only reachable on a camera whose mount was never squared up; the
    square-up tool exists precisely so this path is rare.
    """
    try:
        import cv2
    except ImportError:      # pragma: no cover
        logger.warning("cv2 unavailable — saving without the residual rotation")
        return arr
    h, w = arr.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), float(deg), 1.0)
    cos, sin = abs(m[0, 0]), abs(m[0, 1])
    nw, nh = int(h * sin + w * cos), int(h * cos + w * sin)
    m[0, 2] += nw / 2.0 - w / 2.0
    m[1, 2] += nh / 2.0 - h / 2.0
    return cv2.warpAffine(arr, m, (nw, nh), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
