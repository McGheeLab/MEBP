"""
CaptureResolution.py — momentary full-resolution capture (v7.14).

A scientific camera runs its LIVE preview binned (the Zyla defaults to
1024x1024 2x2) because a full 2048x2048 mono-16 stream is 8.4 MB/frame and
saturates the USB link. But a mosaic tile, or a single "capture this field"
image, wants every pixel the sensor has.

Binning does not change the FIELD OF VIEW — only how many pixels cover it — so
switching up for the duration of a capture costs transfer time, never coverage:
a full-resolution mosaic visits exactly the same tiles.

This module is the ONE place that switch is expressed, shared by the mosaic
scans and the capture button, so they cannot drift about how the previous
resolution is remembered or restored.

Pure: no Qt, no SDK. The camera manager is duck-typed (anything exposing
``get_hw_settings(cam_idx)`` and ``set_capture_resolution(cam_idx, w, h)``),
which is what makes every path here unit-testable against a fake.

⚠ Two facts the callers must respect:

* ``set_capture_resolution`` on an SDK camera STOPS the stream, joins the
  reader thread, fails any pending averaged capture, re-ROIs and restarts —
  hundreds of milliseconds. Never call it from the GUI thread.
* It deliberately emits no signal and writes no config/store, so a temporary
  switch cannot leak into the microscope's ``active_resolution``. The flip side
  is that if the operator (re)starts the camera mid-operation, Hardware Setup's
  ``_maybe_apply_resolution_to_device`` will drive it back to
  ``active_resolution`` underneath you — hence :func:`describe_switch` logging
  what was done rather than assuming it holds.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)

Res = tuple[int, int]

# Bytes per canvas pixel held by MosaicBuilder while stitching: a float64
# composite (3 channels x 8) + a float64 weight sum (8) + the uint8 display
# cache (3). Measured against SupportClasses/MosaicBuilder._init_composite —
# this is why the canvas cannot simply be doubled with the capture resolution.
CANVAS_BYTES_PER_PX = 3 * 8 + 8 + 3

# Ceiling on the auto-raised mosaic canvas. 4500 px is ~709 MB of stitch
# accumulators; beyond that a scan risks thrashing the machine that is also
# driving the stage.
DEFAULT_CANVAS_CAP_PX = 4500

# Canvas footprint past which the operator is warned before scanning.
CANVAS_WARN_MB = 700.0


def _as_res(value: Any) -> Optional[Res]:
    """Coerce ``(w, h)``-ish input to a positive int pair, else None."""
    try:
        w, h = int(value[0]), int(value[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    return (w, h) if w > 0 and h > 0 else None


def available_resolutions(mgr, cam_idx: int) -> list[Res]:
    """Resolutions this camera advertises, largest last. ``[]`` if unknown."""
    if mgr is None or cam_idx is None:
        return []
    try:
        st = mgr.get_hw_settings(cam_idx) or {}
    except Exception:
        return []
    out: list[Res] = []
    for entry in (st.get("resolutions") or []):
        res = _as_res(entry)
        if res is not None and res not in out:
            out.append(res)
    out.sort(key=lambda r: r[0] * r[1])
    return out


def current_resolution(mgr, cam_idx: int) -> Optional[Res]:
    """The camera's CURRENT capture resolution, or None if unreadable."""
    if mgr is None or cam_idx is None:
        return None
    try:
        st = mgr.get_hw_settings(cam_idx) or {}
    except Exception:
        return None
    return _as_res(st.get("resolution"))


def max_resolution(mgr, cam_idx: int) -> Optional[Res]:
    """Largest advertised resolution, or None when the camera lists none."""
    res = available_resolutions(mgr, cam_idx)
    return res[-1] if res else None


def switch_to_max(mgr, cam_idx: int) -> Optional[Res]:
    """Switch to full sensor resolution; return the PREVIOUS one to restore.

    Returns ``None`` — meaning "nothing to restore" — when the camera is
    already at its maximum, advertises a single resolution, cannot report its
    resolutions, or refuses the switch. Callers pass the result straight to
    :func:`restore` in a ``finally``, so a None result is a no-op there too.

    Never raises: a capture must not die because a camera would not switch.
    """
    prev = current_resolution(mgr, cam_idx)
    target = max_resolution(mgr, cam_idx)
    if prev is None or target is None or prev == target:
        return None
    try:
        actual = mgr.set_capture_resolution(cam_idx, target[0], target[1])
    except Exception as exc:
        logger.info(f"Full-res switch refused by cam {cam_idx}: {exc}")
        return None
    got = _as_res(actual) or current_resolution(mgr, cam_idx)
    if got is None or got == prev:
        # The device did not move — nothing to restore, and the caller should
        # not believe it is capturing at full resolution.
        logger.info(f"Full-res switch had no effect on cam {cam_idx} "
                    f"(still {prev[0]}x{prev[1]})")
        return None
    logger.info(f"Cam {cam_idx}: capture resolution {prev[0]}x{prev[1]} -> "
                f"{got[0]}x{got[1]} (temporary; will be restored)")
    return prev


def restore(mgr, cam_idx: int, prev: Optional[Res]) -> bool:
    """Put the camera back to ``prev``. No-op on None. Never raises.

    Called from ``finally`` blocks and teardown paths, so it must survive a
    half-torn-down manager: every failure is logged and swallowed.
    """
    res = _as_res(prev) if prev is not None else None
    if mgr is None or cam_idx is None or res is None:
        return False
    try:
        mgr.set_capture_resolution(cam_idx, res[0], res[1])
        logger.info(f"Cam {cam_idx}: capture resolution restored to "
                    f"{res[0]}x{res[1]}")
        return True
    except Exception as exc:
        logger.warning(f"Cam {cam_idx}: failed to restore capture resolution "
                       f"{res[0]}x{res[1]}: {exc}")
        return False


# ── Mosaic canvas sizing ─────────────────────────────────────────────────────
#
# Capturing at full resolution only reaches the operator if the stitched CANVAS
# can hold the extra detail: MosaicBuilder resizes every tile into a canvas
# whose long edge is ``target_mosaic_px``. Over one 15 mm well a 3000 px canvas
# is ~5 µm/px while a 10x objective at 2048 delivers ~0.65 µm/px — so without
# raising the canvas the extra pixels are thrown away at the resize.

def canvas_px_for_full_res(target_px: int, prev: Optional[Res],
                           new: Optional[Res],
                           cap_px: int = DEFAULT_CANVAS_CAP_PX) -> int:
    """Canvas long edge that preserves the resolution gain, capped.

    Scales ``target_px`` by the linear pixel ratio of the switch (2048/1024 =
    2x), then clamps to ``cap_px`` — memory grows with the SQUARE of this
    number, so an uncapped doubling is a 4x allocation.
    """
    base = max(1, int(target_px))
    a, b = _as_res(prev), _as_res(new)
    if a is None or b is None or a[0] <= 0:
        return min(base, int(cap_px))
    ratio = float(b[0]) / float(a[0])
    if ratio <= 1.0:
        return min(base, int(cap_px))
    return int(min(round(base * ratio), int(cap_px)))


def estimated_canvas_mb(canvas_px: int) -> float:
    """Approximate stitch memory for a square canvas of this long edge (MB).

    A square canvas is the worst case for a given long edge, which is the
    right side to err on for a pre-scan warning.
    """
    px = max(1, int(canvas_px))
    return (px * px * CANVAS_BYTES_PER_PX) / (1024.0 * 1024.0)


def describe_switch(prev: Optional[Res], new: Optional[Res],
                    canvas_px: Optional[int] = None) -> str:
    """One human-readable line for the log and the operator-facing status."""
    a, b = _as_res(prev), _as_res(new)
    if a is None or b is None:
        return "capture resolution unchanged"
    txt = (f"capturing at {b[0]}x{b[1]} (preview {a[0]}x{a[1]}); "
           f"restored when finished")
    if canvas_px:
        txt += (f" · canvas {int(canvas_px)} px "
                f"(~{estimated_canvas_mb(canvas_px):.0f} MB)")
    return txt
