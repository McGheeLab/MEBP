"""Capture timing math — how long to wait for a genuinely-new camera frame.

v7.14. Pure, GUI-free and SDK-free so it is testable without a camera.

WHY THIS EXISTS
---------------
A mosaic tile is only valid if the frame it stitches was EXPOSED after the
stage stopped. The scan worker enforces that by waiting for N genuinely-new
frames after the move, which needs two numbers this module supplies:

* the camera's **frame period** — how long one new frame actually takes; and
* a **timeout** sized from it.

Both were previously constants (a flat 2.5 s), which is fine while the camera
outruns the 15 Hz display timer and badly wrong once it does not. At
2048x2048 with a 200 ms exposure the Zyla delivers ~5 fps: three frames take
~600 ms, and at a 1 s exposure they take over 3 s — past the old flat timeout,
so the wait was abandoned and a STALE (motion-blurred) tile was stitched
silently. Full-resolution scanning is exactly the condition that triggers it.

THE ARGUMENT FOR WAITING TWO FRAMES, NOT ONE
--------------------------------------------
At the instant the settle ends the sensor may already be mid-integration on a
frame that STARTED before the stage stopped. That frame completes and is
delivered — it is "new", and it is smeared. Only the frame after it is
guaranteed to have begun integrating with the stage at rest. So the first new
frame must be discarded: ``MIN_FRESH_FRAMES = 2`` is the floor below which no
amount of settle time can make the result trustworthy.
"""

from __future__ import annotations

# A frame period is never shorter than the exposure; a camera also needs to
# read the sensor out. Used only when the backend cannot report a frame rate.
READOUT_FLOOR_S = 0.030

# Multiplier on the exposure-derived estimate. The estimate is a LOWER bound
# (a camera can always be slower than its exposure implies, never faster), so
# the margin has to point one way: generous.
EXPOSURE_PERIOD_MARGIN = 1.25

# Believable bounds for a resolved frame period. Outside these the reported
# value is not describing frames (a mis-mapped SDK property id, a stale
# reading) and is discarded rather than used to compute a nonsense timeout.
MIN_PERIOD_S = 1e-4      # 10 kHz — faster than any camera here
MAX_PERIOD_S = 60.0      # a 60 s exposure is the practical ceiling

# Waiting costs seconds; giving up costs a dropped tile and a gap in the
# mosaic. So the timeout is deliberately loose.
TIMEOUT_FRAME_MARGIN = 2.5
TIMEOUT_FIXED_PAD_S = 1.0
TIMEOUT_CAP_S = 45.0

# See the module docstring: one new frame may straddle the move.
MIN_FRESH_FRAMES = 2


def frame_period_s(settings) -> float | None:
    """Seconds per new camera frame, or None when it cannot be established.

    ``settings`` is a mapping as returned by a backend's ``get_settings()``.
    Two sources, in order of authority:

    1. ``frame_rate`` (Hz) — what the camera itself reports it is running at.
       The Andor publishes this (v7.13); it already accounts for exposure,
       readout and the interface transfer-rate limit.
    2. ``exposure_us`` — every backend reports this. It is a LOWER bound on
       the period, so it is padded by the readout floor and a margin.

    Returns None rather than a guess when neither is usable — callers then
    fall back to their configured timeout, which is the pre-v7.14 behaviour.
    """
    if not settings:
        return None
    try:
        get = settings.get
    except AttributeError:
        return None

    rate = get("frame_rate")
    try:
        if rate is not None:
            r = float(rate)
            if r > 0:
                p = 1.0 / r
                if MIN_PERIOD_S <= p <= MAX_PERIOD_S:
                    return p
    except (TypeError, ValueError):
        pass

    exp_us = get("exposure_us")
    try:
        if exp_us is not None:
            e = float(exp_us)
            if e > 0:
                p = (e / 1e6 + READOUT_FLOOR_S) * EXPOSURE_PERIOD_MARGIN
                if MIN_PERIOD_S <= p <= MAX_PERIOD_S:
                    return p
    except (TypeError, ValueError):
        pass

    return None


def fresh_frame_timeout_s(n_frames: int, period_s: float | None,
                          configured_s: float) -> float:
    """How long to wait for ``n_frames`` genuinely-new frames.

    Never SHORTER than the operator's configured value — this only ever
    lengthens the wait for a slow camera, so a rig that works today cannot
    start dropping tiles because of this function.
    """
    try:
        base = float(configured_s)
    except (TypeError, ValueError):
        base = 0.0
    if base < 0:
        base = 0.0

    if period_s is None:
        return base
    try:
        p = float(period_s)
    except (TypeError, ValueError):
        return base
    if not (MIN_PERIOD_S <= p <= MAX_PERIOD_S):
        return base

    n = max(1, int(n_frames))
    need = n * p * TIMEOUT_FRAME_MARGIN + TIMEOUT_FIXED_PAD_S
    return min(TIMEOUT_CAP_S, max(base, need))


def resolve_grab_timing(cam, n_frames: int, configured_s: float):
    """``(n_frames, timeout_s, period_s)`` for a duck-typed camera widget.

    Tolerates a camera that raises, reports nothing, or predates the frame
    accounting entirely — every failure degrades to the configured timeout.
    """
    period = None
    try:
        getter = getattr(cam, "get_hw_settings", None)
        if callable(getter):
            period = frame_period_s(getter())
    except Exception:
        period = None

    n = max(MIN_FRESH_FRAMES, int(n_frames or 0))
    return n, fresh_frame_timeout_s(n, period, configured_s), period
