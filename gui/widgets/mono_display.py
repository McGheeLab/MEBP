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


def compute_raw_frame_stats(arr, clip_level: int, bins: int = 256) -> "dict | None":
    """Cheap per-frame statistics of a RAW mono plane, before display scaling.

    The display auto-scale (1–99 percentile) actively HIDES clipping — a
    saturated frame still renders with headroom — so exposure decisions need
    numbers taken from the raw counts. ``clip_level`` is the sensor's true
    full-scale for the CURRENT gain mode (a Zyla in a 12-bit mode clips at
    ~4095, far below the uint16 container's 65535); pixels at or above it are
    counted as clipped.

    Decimates with the same stride policy as :func:`_auto_levels` so the cost
    is bounded (~1–2 ms at 2048²) and the two views of the frame agree about
    which pixels were consulted. Returns None on any failure — statistics must
    never be able to take down a camera feed.
    """
    if not _NP_AVAILABLE or arr is None:
        return None
    try:
        a = np.asarray(arr)
        if a.ndim == 3:
            a = a[..., 0]
        if a.ndim != 2 or a.size == 0:
            return None
        clip = max(1, int(clip_level))
        step = max(1, int(max(a.shape) // 512))
        sample = a[::step, ::step]
        n = int(sample.size)
        if n == 0:
            return None
        hist, _edges = np.histogram(sample, bins=int(bins), range=(0, clip + 1))
        return {
            "min": float(sample.min()),
            "max": float(sample.max()),
            "mean": float(sample.mean()),
            "clipped_frac": float(np.count_nonzero(sample >= clip) / n),
            "clip_level": clip,
            "hist": hist,
            "hist_range": (0, clip + 1),
            "sample_count": n,
        }
    except Exception:
        return None


class RawAverageRequest:
    """Reader-thread-serviced accumulator behind ``capture_raw_average()``.

    Shared by BOTH mono scientific backends (Zyla, Tucsen): each backend's
    reader thread is the single frame waiter, so a caller publishes this
    request and blocks on ``done`` while the reader adds each NEW raw plane.
    Accumulation is float64 — a uint16 accumulator overflows after two bright
    frames.
    """

    def __init__(self, n: int, agree_threshold: "float | None" = None):
        import threading
        from SupportClasses.FrameAveraging import DEFAULT_AGREEMENT_RATIO
        self.n = int(n)
        self.count = 0
        self.acc = None
        self.shape = None
        self.error: "str | None" = None
        self.done = threading.Event()
        # v7.14: averaging frames that do NOT show the same thing blurs rather
        # than denoises — the stage still settling, a drifting sample, a lamp
        # flicker. Held to the operator's own condition: average "as long as
        # the three frames look like each other". None disables the check.
        self.agree_threshold = (DEFAULT_AGREEMENT_RATIO
                                if agree_threshold is None
                                else agree_threshold)
        self.worst_ratio = 1.0
        self._ref = None

    def add(self, plane) -> None:
        if self.done.is_set():
            return
        if self.acc is None:
            self.shape = tuple(plane.shape)
            self.acc = plane.astype(np.float64)
            self.count = 1
            if self.agree_threshold is not None:
                # Keep the first plane as the agreement reference. Comparing
                # against the FIRST rather than the previous frame catches a
                # slow drift that would pass as three individually-small steps.
                self._ref = plane.copy()
        elif tuple(plane.shape) != self.shape:
            self.fail("frame shape changed mid-capture")
            return
        else:
            if self._ref is not None:
                from SupportClasses.FrameAveraging import structure_ratio
                try:
                    self.worst_ratio = max(
                        self.worst_ratio, structure_ratio(self._ref, plane))
                except Exception:
                    pass
            self.acc += plane
            self.count += 1
        if self.count >= self.n:
            self.done.set()

    def frames_agree(self) -> bool:
        """True when the collected frames differ only by noise."""
        if self.agree_threshold is None:
            return True
        return self.worst_ratio <= float(self.agree_threshold)

    def fail(self, reason: str) -> None:
        self.error = str(reason)
        self.done.set()

    def result(self) -> "np.ndarray | None":
        if self.error or self.acc is None or self.count < self.n:
            return None
        if not self.frames_agree():
            # v7.14: refuse rather than return a blurred mean. The caller
            # falls back to a SINGLE post-move frame, which is exactly what
            # the scan produced before averaging existed — so this can only
            # ever match the old behaviour, never do worse than it.
            self.error = (
                f"frames disagree (structure ratio {self.worst_ratio:.2f} > "
                f"{float(self.agree_threshold):.2f}) — something moved between "
                f"them, so averaging would blur rather than denoise")
            return None
        return np.clip(np.rint(self.acc / float(self.count)),
                       0, LEVEL_MAX).astype(np.uint16)


# ════════════════════════════════════════════════════════════════════
#  One-shot signal optimizer (v7.13.x)
# ════════════════════════════════════════════════════════════════════
# The operator's ask: the software computes the camera parameters for the best
# histogram, sets them ONCE, and nothing changes per frame afterwards. Pure
# planning math here (testable without a camera); the blocking driver below is
# duck-typed against CameraManager and works for any backend advertising the
# raw-stats capability (Zyla, Tucsen).

def freeze_levels_from_frame(frame, level_cap: int = LEVEL_MAX) -> "tuple[int, int]":
    """Fixed display (black, white) levels from a raw frame.

    The mosaic-proven recipe: lo = P0.5, hi = lo + (P99.9 − lo) × 1.2 (20 %
    headroom), degenerate spans (<16 counts) widened so a flat/dark frame
    never yields an all-white display."""
    lo = float(np.percentile(frame, 0.5))
    hi = float(np.percentile(frame, 99.9))
    hi = min(float(level_cap), lo + (hi - lo) * 1.2)
    if hi - lo < 16.0:
        hi = lo + 256.0
    lo_i = max(0, int(round(lo)))
    hi_i = min(int(level_cap), int(round(hi)))
    if hi_i <= lo_i:
        hi_i = min(int(level_cap), lo_i + 256)
    return lo_i, hi_i


def plan_auto_exposure_step(p_signal, clipped_frac, clip_level, exposure_us,
                            exp_min_us, exp_max_us,
                            target_frac: float = 0.70, tol: float = 0.08
                            ) -> "tuple[int | None, str]":
    """One step of the one-shot exposure optimizer.

    Returns ``(new_exposure_us, note)`` — or ``(None, note)`` when done
    (converged or pinned at a range end). ``p_signal`` is the P99.9 of a fresh
    RAW frame (full-precision percentile, not the 256-bin histogram).

    A clipped frame is CENSORED — the true signal is unknowable above the
    clip — so the response is to halve, never to scale linearly (which would
    undershoot and leave the frame still clipped).
    """
    clip = max(1.0, float(clip_level))
    exp = max(1.0, float(exposure_us))
    lo = max(1.0, float(exp_min_us))
    hi = max(lo, float(exp_max_us))
    target = float(target_frac) * clip

    if float(clipped_frac) > 0.001 or float(p_signal) >= clip * 0.999:
        if exp <= lo * 1.001:
            return None, "saturated at minimum exposure — reduce illumination"
        return int(max(lo, exp * 0.5)), "clipped — halving exposure"

    p = float(p_signal)
    if p <= 0.0:
        if exp >= hi * 0.999:
            return None, "no signal at maximum exposure"
        return int(min(hi, exp * 4.0)), "no signal — quadrupling exposure"

    ratio = target / p
    if abs(ratio - 1.0) <= float(tol):
        return None, "converged"
    new = exp * ratio
    if new >= hi:
        if exp >= hi * 0.999:
            return None, "signal weak at maximum exposure"
        return int(hi), "raising to maximum exposure"
    if new <= lo:
        if exp <= lo * 1.001:
            return None, "at minimum exposure"
        return int(lo), "lowering to minimum exposure"
    new_i = int(round(new))
    if new_i == int(round(exp)):
        return None, "converged"
    return new_i, f"scaling exposure x{ratio:.2f}"


def run_signal_optimize(mgr, cam_idx: int, *, freeze_display: bool = True,
                        max_iters: int = 6, target_frac: float = 0.70,
                        avg_frames: int = 2) -> "tuple[int | None, str]":
    """One-shot optimize: auto-expose to the target histogram, then freeze.

    Iterates set-exposure → fresh raw capture → :func:`plan_auto_exposure_step`
    until converged (≤ ``max_iters``), then — when ``freeze_display`` — turns
    the per-frame display auto-scale OFF and pins fixed black/white levels
    from the final frame, so nothing changes per frame afterwards.

    BLOCKING (waits for fresh frames between steps) — worker-thread-only, the
    same contract as ``capture_raw_average``. Returns
    ``(achieved_exposure_us | None, human-readable note)``; None means the
    routine refused or could not run (the note says why) and nothing beyond
    already-applied exposure steps was changed.
    """
    import time
    if not _NP_AVAILABLE:
        return None, "numpy unavailable"
    try:
        st = mgr.get_hw_settings(cam_idx) or {}
    except Exception:
        st = {}
    if st.get("auto_exposure") is True:
        return None, ("disable the camera's auto-exposure first — "
                      "it would fight the optimizer")
    rng = st.get("exposure_range_us")
    if rng and len(rng) >= 2 and rng[0] is not None and rng[1] is not None:
        exp_lo, exp_hi = float(rng[0]), float(rng[1])
    else:
        exp_lo, exp_hi = 100.0, 30_000_000.0
    exp = st.get("exposure_us")
    if not exp or float(exp) <= 0:
        exp = 30_000.0
    exp = float(min(max(float(exp), exp_lo), exp_hi))

    clip = None
    frame = None
    p999 = None
    note = "no adjustment"
    for _ in range(max(1, int(max_iters))):
        try:
            mgr.set_hw_exposure_us(cam_idx, int(round(exp)))
        except Exception:
            pass
        # Let a frame already exposing at the OLD setting drain before the
        # fresh capture (capture_raw_average only guarantees frames that
        # ARRIVE after the request, not frames that STARTED after the set).
        time.sleep(min(2.0, max(0.05, 1.5 * exp / 1e6)))
        timeout = max(5.0, (avg_frames + 2) * 2.0 * exp / 1e6 + 2.0)
        try:
            frame = mgr.capture_raw_average(cam_idx, avg_frames,
                                            timeout_s=timeout)
        except Exception:
            frame = None
        if frame is None:
            return None, "no raw frames — is the camera streaming?"
        try:
            stats = mgr.get_raw_frame_stats(cam_idx) or {}
        except Exception:
            stats = {}
        clip = int(stats.get("clip_level") or clip or LEVEL_MAX)
        p999 = float(np.percentile(frame, 99.9))
        clipped = float(np.count_nonzero(frame >= clip)) / float(frame.size)
        new, note = plan_auto_exposure_step(
            p999, clipped, clip, exp, exp_lo, exp_hi, target_frac=target_frac)
        if new is None:
            break
        exp = float(new)

    achieved = None
    try:
        achieved = (mgr.get_hw_settings(cam_idx) or {}).get("exposure_us")
    except Exception:
        pass
    if achieved is None:
        achieved = int(round(exp))

    levels_txt = ""
    if freeze_display and frame is not None:
        lo_l, hi_l = freeze_levels_from_frame(frame, clip or LEVEL_MAX)
        # Auto-scale OFF first, explicit levels after — same ordering
        # invariant as the persisted-controls restore (auto-off seeds levels
        # from the last auto frame; the explicit values must win).
        for name, val in (("set_hw_andor_auto_scale", False),
                          ("set_hw_andor_scale_lo", lo_l),
                          ("set_hw_andor_scale_hi", hi_l)):
            fn = getattr(mgr, name, None)
            if fn is not None:
                try:
                    fn(cam_idx, val)
                except Exception:
                    pass
        levels_txt = f" · display levels frozen {lo_l}–{hi_l}"

    head = f"exposure {float(achieved) / 1000.0:.1f} ms"
    if p999 is not None and clip:
        head += f" · P99.9 at {100.0 * p999 / clip:.0f}% of clip"
    return int(achieved), head + levels_txt + f" · {note}"


# Public aliases — consumers outside the backend pair (mosaic workers, tests)
# should not have to import underscore names.
auto_levels = _auto_levels
mono_to_bgr8 = _mono_to_bgr8
