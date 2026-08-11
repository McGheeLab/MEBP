"""Averaging several frames per tile — and deciding when that is safe.

v7.14. Pure numpy; no Qt, no SDK, so the statistic is testable without a camera.

WHY AVERAGE
-----------
Averaging N frames of the same scene divides read/shot noise by sqrt(N): three
frames is a 1.7x improvement, free apart from the time to collect them. On a
mosaic tile that is a real quality win, especially in fluorescence where the
signal is dim.

WHY IT NEEDS A GUARD (the operator's own condition: "as long as the three
frames look like each other")
------------------------------------------------------------------------
If anything moved between the frames — stage still settling, vibration, a
drifting sample, a lamp flicker, an auto-exposure step — averaging BLURS
instead of denoising. That is worse than a single frame, and it looks
plausible, so nothing downstream would catch it.

WHY "ARE THEY THE SAME?" IS THE WRONG QUESTION
----------------------------------------------
Consecutive frames NEVER look identical: shot noise guarantees a difference,
and at low signal the difference is ENTIRELY noise — which is exactly the
situation averaging is for. A similarity test with an absolute threshold
therefore rejects hardest where averaging helps most.

Measured, not assumed. A simple normalised difference was tried first and does
not work: on a dim, low-contrast scene noise alone scored 0.0707 while a real
3 px shift on a bright scene scored only 0.0236 — the noise-only case scored
HIGHER than genuine motion, so no fixed threshold can separate them.

THE STATISTIC THAT DOES WORK
----------------------------
Noise is spatially uncorrelated; structure is not. Block-average the difference
image by B: noise falls as 1/B, structure does not fall at all. So

    ratio = B * mean|blockmean(A - REF)| / mean|A - REF|

is ~1.0 for pure noise and rises toward B as the difference becomes structural
— and it is independent of both signal level and contrast, which is what the
first attempt lacked.

MEASURED over scene brightness 200..20000 counts, contrast 0.3..1.0 and frame
sizes 64x64..2048x2048 (two independent runs):

    noise only ................ 1.00 - 1.05   (2048x2048: 1.004 - 1.015)
    1 px shift ................ 1.6  - 4.8
    3 px shift ................ 2.8  - 7.5
    10 % brightness change .... 3.5  - 8.0
    three different scenes .... = B  (the theoretical maximum)
    flat featureless field .... 1.00 either way

Hence ``DEFAULT_AGREEMENT_RATIO = 1.5``: a 1.43x margin over the worst
noise-only case, while still catching a single-pixel shift.

Two honest limits, both benign:

* On a DIM, LOW-CONTRAST scene a 1 px shift scores ~1.05 and is not caught.
  It is also barely present — the blur it adds is small next to the noise
  being removed, so accepting it is the right trade.
* On a FLAT featureless field motion is undetectable (nothing to correlate)
  and equally harmless: there is no structure to smear.

The block size is chosen so the statistic always has ~32x32 blocks whatever
the camera resolution, which is what keeps its spread consistent (a fixed B
gave 1.44 on a small frame — too close to the threshold).
"""

from __future__ import annotations

import numpy as np

# Target block grid. The statistic's SPREAD depends on how many blocks it
# averages over, so holding the grid constant holds the noise floor constant
# from a 64x64 thumbnail to a 2048x2048 full-resolution tile.
BLOCK_GRID = 32
MIN_BLOCK = 2

# See the measurements in the module docstring.
DEFAULT_AGREEMENT_RATIO = 1.5

# Below this the difference is numerically empty — identical buffers. Averaging
# is pointless (no independent noise to cancel) but not harmful.
_EMPTY_DIFF = 1e-12


def block_size_for(shape) -> int:
    """Block factor giving ~BLOCK_GRID blocks across the shorter axis."""
    try:
        m = int(min(int(shape[0]), int(shape[1])))
    except (TypeError, ValueError, IndexError):
        return MIN_BLOCK
    return max(MIN_BLOCK, m // BLOCK_GRID)


def block_mean(a, block: int):
    """Non-overlapping block average; trailing partial blocks are dropped."""
    b = max(1, int(block))
    h = (a.shape[0] // b) * b
    w = (a.shape[1] // b) * b
    if h < b or w < b:
        return a.astype(np.float64, copy=False)
    return a[:h, :w].reshape(h // b, b, w // b, b).mean(axis=(1, 3))


def structure_ratio(ref, other) -> float:
    """How STRUCTURED the difference between two frames is.

    ~1.0  the frames differ only by noise  -> averaging is the right thing
    >1    the difference survives block averaging, so something moved
    = B   the two frames share no structure at all

    Scale-free: independent of signal level and contrast. Colour frames are
    reduced to a single plane first — a mirrored/rotated tile is oriented
    downstream, so the channels carry no extra information here.
    """
    a = np.asarray(ref)
    b = np.asarray(other)
    if a.shape != b.shape:
        # Different shapes cannot be averaged at all; report maximal
        # disagreement rather than raising into a scan loop.
        return float(BLOCK_GRID)
    if a.ndim == 3:
        a = a.mean(axis=2)
        b = b.mean(axis=2)
    d = b.astype(np.float64) - a.astype(np.float64)
    full = float(np.mean(np.abs(d)))
    if full <= _EMPTY_DIFF:
        # Byte-identical frames. Not disagreement — but see average_frames'
        # note: there is no independent noise to average away.
        return 1.0
    block = block_size_for(d.shape)
    coarse = float(np.mean(np.abs(block_mean(d, block))))
    return block * coarse / full


def frames_agree(frames, threshold: float = DEFAULT_AGREEMENT_RATIO):
    """``(agree, worst_ratio, reason)`` for a list of frames.

    Every frame is compared against the FIRST, not against its predecessor, so
    a slow drift across the sequence is caught rather than passing as three
    individually-small steps.
    """
    if frames is None or len(frames) < 2:
        return True, 1.0, ""
    try:
        thr = float(threshold)
    except (TypeError, ValueError):
        thr = DEFAULT_AGREEMENT_RATIO
    ref = frames[0]
    worst = 1.0
    for f in frames[1:]:
        if getattr(f, "shape", None) != getattr(ref, "shape", None):
            return False, float(BLOCK_GRID), "frame size changed mid-capture"
        worst = max(worst, structure_ratio(ref, f))
    if worst > thr:
        return (False, worst,
                f"frames disagree (structure ratio {worst:.2f} > {thr:.2f}) — "
                f"something moved between them")
    return True, worst, ""


def average_frames(frames):
    """Per-pixel mean, returned in the input dtype.

    Accumulates in float64: a uint16 accumulator overflows after two bright
    frames, and a uint8 one after two of anything.
    """
    if not frames:
        return None
    if len(frames) == 1:
        return frames[0]
    ref = frames[0]
    acc = np.zeros(ref.shape, dtype=np.float64)
    for f in frames:
        acc += f
    acc /= float(len(frames))
    if np.issubdtype(ref.dtype, np.integer):
        info = np.iinfo(ref.dtype)
        return np.clip(np.rint(acc), info.min, info.max).astype(ref.dtype)
    return acc.astype(ref.dtype)


def average_if_agreeing(frames, threshold: float = DEFAULT_AGREEMENT_RATIO):
    """``(frame, n_used, note)``.

    On disagreement this returns the FIRST frame rather than nothing: a single
    post-move frame is exactly what the scan produced before averaging existed,
    so falling back is never worse than the previous behaviour. Refusing the
    tile outright would make scans fail where they currently succeed.
    """
    if not frames:
        return None, 0, "no frames"
    if len(frames) == 1:
        return frames[0], 1, ""
    ok, worst, reason = frames_agree(frames, threshold)
    if not ok:
        return frames[0], 1, reason
    return average_frames(frames), len(frames), ""
