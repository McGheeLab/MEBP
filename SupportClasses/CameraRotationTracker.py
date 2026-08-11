"""Live camera-rotation tracking for physically squaring up a camera mount.

GUI-free and dependency-light: importing this module must NOT pull in PySide6,
scipy or skimage. The Fourier-Mellin estimator is lazy-imported from
``MosaicBuilder`` inside :func:`estimate_rotation` (that import costs ~0.5 s
because of skimage, and this module sits in the camera path).

WHAT THIS IS FOR
----------------
The app already MEASURES a camera's rotation vs the stage and corrects for it
in software. This module supports the opposite operation: helping the operator
REMOVE the rotation physically, by turning the camera in its mount until the
measured angle lands on a cardinal (0/90/180/270).

Two independent jobs:

* :func:`target_image_rotation_deg` — how far the IMAGE must rotate, computed
  from the stored calibration. This is the one and only sign site.
* :func:`estimate_rotation` / :class:`RotationTracker` — how far the image HAS
  rotated so far, measured live against a frozen reference frame, with no stage
  motion at all.

THE SIGN, DERIVED ONCE
----------------------
``CameraManager.pixel_to_stage_offset`` maps a raw centred pixel ``p`` to a
stage offset as ``s = R(θ)·F·u·p`` with ``F = diag(mx, my)``, ``mx = −1`` iff
mirrored (flip_x), ``my = −1`` iff flip_y, and ``R(t) = [[c,−s],[s,c]]`` in
(x, y-down) image coordinates. ``camera_feed_view.view_transform_coeffs`` and
``MosaicBuilder._orient_tile`` share that convention.

Turn the camera so that the scene point which sat at ``p`` now appears at
``R(φ)·p``. Its stage offset is a property of the plate, not the camera, so it
is unchanged::

    R(θ)·F = R(θ')·F·R(φ)
    R(θ') = R(θ)·[F·R(−φ)·F⁻¹]

Conjugating a rotation by ``F`` reverses it exactly when ``det F = −1``, so::

    θ' = θ − φ   when  det F = +1   (flip_x == flip_y)
    θ' = θ + φ   when  det F = −1   (flip_x != flip_y)

Wanting ``θ' = nominal`` with ``θ = nominal + Δ`` therefore gives

    **φ_target = +Δ  unmirrored,  −Δ  net-mirrored.**

Only ``det F`` is read, so it does not matter that
``derive_camera_stage_orientation`` always parks handedness on ``flip_y``.
Pinned by a closed-loop test that never mentions φ's sign: it drives the two
pixel positions through the real ``pixel_to_stage_offset`` and asserts the
stage offsets match.

WHAT IS DELIBERATELY *NOT* DERIVED
----------------------------------
Which way the operator must physically turn the camera. Physical-turn direction
maps to image-rotation direction through the optical path (an odd number of
mirrors reverses it), which software cannot know. :class:`RotationTracker`
latches that empirically instead — see :meth:`RotationTracker.direction_hint`.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    cv2 = None

logger = logging.getLogger(__name__)

# Working size for the estimator. Measured on this repo's own Fourier-Mellin:
# 256² costs 10.5 ms and recovers a true −8.000° as −8.094°; 512² costs 40.4 ms
# for −7.969°. The extra resolution buys nothing — the angular precision comes
# from phaseCorrelate's sub-pixel peak, not the row count.
TRACK_SIZE = 256

# Minimum Fourier-Mellin confidence to accept a sample.
#
# ⚠ This is NOT a rotation-quality metric: FM returns the *translation*
# response measured AFTER de-rotating by the estimated angle, so it is really
# "did the whole registration close". That makes it an excellent end-to-end
# validity check AND a free detector for the 180° log-polar branch ambiguity —
# a wrong branch de-rotates wrongly and the response collapses. Measured on
# synthetic scenes: 80° → 0.83, 100° → 0.01, 180° → 0.015. That is why this
# module folds to (−90, 90] and gates, instead of trying to unwrap: a wrapped
# sample can never pass.
MIN_CONF = 0.15

# Below this the reference frame is too flat to register against. Matches
# MosaicBuilder._REGISTER_MIN_STD, which rejects the same crops upstream.
MIN_REF_STD = 6.0

# The four squared-up positions. A camera's ``rotation_deg`` is the camera→stage
# rotation, whose square positions are always multiples of 90° — for EVERY role.
# (Do not reach for ``hardware_setup.role_nominal_rotations`` here: for a needle
# cam that returns the ±45° MOUNT nominals, and aiming a needle camera's roll at
# 45° would roll it out of level.)
SQUARE_NOMINALS = (0.0, 90.0, 180.0, -90.0)

# Motion that must accumulate before the turn direction can be latched.
MOTION_MIN_DEG = 2.0
# An accepted sample older than this is stale — blank the readout, don't hold it.
STALE_S = 0.5
# Rolling median window. 3 at ~120 ms is ~240 ms of lag, which a hand can track;
# 5 would be ~600 ms and drives overshoot oscillation.
SMOOTH_N = 3


def wrap_deg(angle: float) -> float:
    """Wrap an angle to (-180, 180]. The canonical copy.

    ``gui.pages.hardware_setup`` re-exports this rather than keeping its own —
    a duplicated sign-carrying helper is how this codebase accumulated its
    sign-bug history.
    """
    a = (float(angle) + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def fold_parallel_deg(angle_deg: float) -> float:
    """Fold an angle to (-90, 90] — its deviation from PARALLEL.

    A displacement vector and its negation lie on the same line, so both fold
    to the same roll (e.g. 179° → -1°, -135° → 45°). Re-exported by
    ``gui.dialogs.pixel_calibration_dialog``, which is where it used to live.
    """
    r = float(angle_deg) % 180.0  # Python % is non-negative for float rhs>0
    return r - 180.0 if r > 90.0 else r


def nearest_nominal(theta_deg: float,
                    nominals: "tuple[float, ...]") -> "tuple[float, float]":
    """``(nearest_nominal, signed_delta)`` with ``theta = nominal + delta``.

    Seeded at ``(0.0, wrap(theta))`` and compared with a strict ``<``, exactly
    reproducing ``hardware_setup.nominal_rotation_delta`` — including its
    behaviour when 0 is not in ``nominals`` and nothing beats the seed, and its
    deterministic choice at the ±45° knife-edge (45.0 keeps nominal 0; 45.01
    flips to 90).
    """
    best_nom, best_delta = 0.0, wrap_deg(theta_deg)
    for nom in nominals:
        d = wrap_deg(float(theta_deg) - float(nom))
        if abs(d) < abs(best_delta):
            best_nom, best_delta = float(nom), d
    return best_nom, best_delta


def nearest_square_rotation(theta_deg: float) -> "tuple[float, float]":
    """``(nominal, delta)`` against the four cardinals. ``|delta| <= 45``."""
    return nearest_nominal(theta_deg, SQUARE_NOMINALS)


def target_image_rotation_deg(theta_deg: float, mirrored_net: bool,
                              nominal_deg: float) -> float:
    """How far the IMAGE CONTENT must rotate to bring ``theta`` to ``nominal``.

    ``+`` follows ``R(t) = [[c,-s],[s,c]]`` in (x, y-down) raw frame coords —
    the same convention as ``view_transform_coeffs`` and as the angle
    :func:`estimate_rotation` returns, so target and measurement are directly
    comparable.

    ``mirrored_net`` is ``flip_x XOR flip_y`` (i.e. ``det F == -1``). See the
    module docstring for the derivation; this is the only place the sign is
    written down.
    """
    delta = wrap_deg(float(theta_deg) - float(nominal_deg))
    return wrap_deg(-delta if bool(mirrored_net) else delta)


def prepare_frame(img: "np.ndarray", size: int = TRACK_SIZE) -> "np.ndarray":
    """Centre-crop to a square, downscale to ``size``, return uint8 grayscale.

    The square crop is load-bearing, not tidiness: Fourier-Mellin's log-polar
    resample uses ``maxRadius = min(w, h) / 2``, so a non-square input wastes
    most of the frame and the confidence collapses (measured 0.004 vs 0.83 on
    the same scene). Rotating about the optical axis also preserves only the
    inscribed disc, which a square crop centres on.

    ``INTER_AREA`` matters at the ~14x decimation a 3664x2748 sensor needs —
    the default bilinear aliases badly, and the log-polar transform weights all
    radii equally so that aliasing lands straight in the angle estimate.
    """
    if cv2 is None or img is None:
        raise ValueError("prepare_frame requires cv2 and a frame")
    a = np.asarray(img)
    if a.ndim == 3:
        a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    elif a.ndim != 2:
        raise ValueError(f"expected a 2-D or 3-D frame, got shape {a.shape}")
    h, w = a.shape[:2]
    side = int(min(h, w))
    if side < 8:
        raise ValueError(f"frame too small to track: {w}x{h}")
    y0 = (h - side) // 2
    x0 = (w - side) // 2
    a = a[y0:y0 + side, x0:x0 + side]
    n = max(8, int(size))
    if side != n:
        interp = cv2.INTER_AREA if side > n else cv2.INTER_LINEAR
        a = cv2.resize(a, (n, n), interpolation=interp)
    if a.dtype != np.uint8:
        # CLAHE inside the estimator needs CV_8UC1; a 16-bit scientific frame
        # would otherwise throw and silently fall back to un-normalised input.
        a = cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    assert a.shape[0] == a.shape[1], "prepare_frame must return a square"
    return np.ascontiguousarray(a)


def frame_is_trackable(gray: "np.ndarray") -> bool:
    """Does this prepared frame have enough texture to register against?"""
    if gray is None:
        return False
    return float(np.asarray(gray, dtype=np.float64).std()) >= MIN_REF_STD


def _rot_matrix(shape, phi_deg: float):
    h, w = shape[:2]
    # Match _register_overlap_fourier_mellin's centre exactly ((w/2, h/2), not
    # ((w-1)/2, ...)) so make_ghost and the estimator agree to sub-pixel.
    #
    # cv2.getRotationMatrix2D(c, a) has linear part [[cos a, sin a],
    # [-sin a, cos a]] == R(-a), and warpAffine maps src -> dst by that matrix.
    # So to move content by R(phi) we pass a = -phi.
    return cv2.getRotationMatrix2D((w / 2.0, h / 2.0), -float(phi_deg), 1.0)


def make_ghost(img: "np.ndarray", phi_deg: float) -> "np.ndarray":
    """Rotate ``img``'s content by ``R(phi)``, same shape and dtype.

    Sign pinned two ways: a round-trip against :func:`estimate_rotation`, and
    an absolute anchor test that rotates a synthetic scene with a hand-written
    ``getRotationMatrix2D`` call and checks the recovered angle.
    """
    if cv2 is None or img is None:
        raise ValueError("make_ghost requires cv2 and an image")
    a = np.asarray(img)
    h, w = a.shape[:2]
    return cv2.warpAffine(a, _rot_matrix(a.shape, phi_deg), (w, h),
                          flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)


def edge_rgba(img: "np.ndarray", color_rgb=(166, 227, 161),
              low: int = 50, high: int = 150,
              thickness: int = 1) -> "np.ndarray":
    """Canny edges of ``img`` as an RGBA overlay (transparent off the edges).

    The alternative to the photo ghost: crisper to align by eye on a textured
    scene, but it can come out nearly empty on a low-contrast plate view, which
    is why the dialog offers both rather than picking one.
    """
    if cv2 is None or img is None:
        raise ValueError("edge_rgba requires cv2 and an image")
    a = np.asarray(img)
    if a.ndim == 3:
        a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    if a.dtype != np.uint8:
        a = cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    edges = cv2.Canny(a, int(low), int(high))
    if int(thickness) > 1:
        k = np.ones((int(thickness), int(thickness)), np.uint8)
        edges = cv2.dilate(edges, k)
    h, w = edges.shape[:2]
    out = np.zeros((h, w, 4), dtype=np.uint8)
    r, g, b = (int(c) for c in color_rgb)
    out[..., 0] = r
    out[..., 1] = g
    out[..., 2] = b
    out[..., 3] = edges
    return out


@dataclass(frozen=True)
class RotationSample:
    """One accepted measurement of the live frame against the reference."""
    deg: float          # image rotation, folded to (-90, 90]
    dx: float           # residual translation, prepared-frame px
    dy: float
    conf: float


def estimate_rotation(ref_gray: "np.ndarray", live_gray: "np.ndarray"
                      ) -> "RotationSample | None":
    """Rotation of ``live`` relative to ``ref``, or ``None`` if not trustworthy.

    Reuses ``MosaicBuilder._register_overlap_fourier_mellin`` rather than
    reimplementing the log-polar transform, so there is exactly ONE place in
    the repo where that sign convention is written. Imported lazily — see the
    module docstring.

    Folds to (-90, 90]: the estimator works on the FFT magnitude, which is
    centro-symmetric, so its output is only defined mod 180. Folding is safe
    because the tool only ever targets the NEAREST cardinal, bounding the true
    angle at 45°; and a genuinely wrapped sample is rejected by ``MIN_CONF``
    anyway (see that constant).
    """
    if cv2 is None or ref_gray is None or live_gray is None:
        return None
    try:
        from SupportClasses.MosaicBuilder import (
            _register_overlap_fourier_mellin as _fm)
    except Exception as e:                                # pragma: no cover
        logger.debug(f"rotation estimator unavailable: {e}")
        return None
    try:
        out = _fm(ref_gray, live_gray)
    except Exception as e:                                # pragma: no cover
        logger.debug(f"rotation estimate failed: {e}")
        return None
    if out is None:
        return None
    dx, dy, rot, _scale, conf = out
    if float(conf) < MIN_CONF:
        return None
    return RotationSample(deg=fold_parallel_deg(float(rot)),
                          dx=float(dx), dy=float(dy), conf=float(conf))


class RotationTracker:
    """Tracks how far the image has rotated since the reference was frozen.

    Owns no Qt and no camera — the caller hands it prepared frames. Times are
    injectable so the staleness logic is testable without sleeping.
    """

    def __init__(self, target_deg: float = 0.0):
        self._ref: "np.ndarray | None" = None
        self._target = float(target_deg)
        self._hist: deque = deque(maxlen=SMOOTH_N)
        self._track: deque = deque(maxlen=64)   # (phi, residual) for direction
        self._last: "RotationSample | None" = None
        self._last_t: float = -1e9
        self._smoothed: "float | None" = None

    # -- configuration ----------------------------------------------------
    @property
    def target_deg(self) -> float:
        return self._target

    def set_target(self, target_deg: float) -> None:
        """Retarget without dropping the reference (e.g. the operator toggled
        the mirror flag on the card behind the dialog, inverting phi_target)."""
        t = float(target_deg)
        if abs(t - self._target) < 1e-9:
            return
        self._target = t
        # Residual history is measured against the old target; keep the angle
        # samples but restart the direction latch.
        self._track.clear()

    def set_reference(self, gray: "np.ndarray | None") -> bool:
        """Freeze a new reference. Returns False if it is too flat to use."""
        self._hist.clear()
        self._track.clear()
        self._last = None
        self._smoothed = None
        self._last_t = -1e9
        if gray is None or not frame_is_trackable(gray):
            self._ref = None
            return False
        self._ref = np.array(gray, copy=True)
        return True

    @property
    def has_reference(self) -> bool:
        return self._ref is not None

    @property
    def reference_shape(self):
        return None if self._ref is None else tuple(self._ref.shape[:2])

    # -- measurement ------------------------------------------------------
    def update(self, gray: "np.ndarray", now: float
               ) -> "RotationSample | None":
        """Measure one frame. Returns the accepted sample, or None."""
        if self._ref is None or gray is None:
            return None
        if np.asarray(gray).shape != self._ref.shape:
            # A resolution change makes the frozen reference a different scene
            # scale; prepare_frame would hide that, so refuse rather than
            # report a confident wrong angle.
            return None
        s = estimate_rotation(self._ref, gray)
        if s is None:
            return None
        self._hist.append(s.deg)
        self._smoothed = float(np.median(list(self._hist)))
        self._last = s
        self._last_t = float(now)
        self._track.append((self._smoothed, self.residual_deg))
        return s

    # -- readout ----------------------------------------------------------
    def is_stale(self, now: float) -> bool:
        return (float(now) - self._last_t) > STALE_S

    @property
    def rotated_deg(self) -> "float | None":
        """Smoothed image rotation since the reference was frozen."""
        return self._smoothed

    @property
    def instantaneous_deg(self) -> "float | None":
        return None if self._last is None else self._last.deg

    @property
    def confidence(self) -> "float | None":
        return None if self._last is None else self._last.conf

    @property
    def translation_px(self) -> "tuple[float, float]":
        """Reference→live translation in prepared-frame px.

        FM returns the correction to ADD to the live frame's position to put it
        back on the reference, so moving the REFERENCE onto the live frame is
        the negation. Used to keep the ghost overlaid when the mount axis is
        not the optical axis, so the operator judges angle only.
        """
        if self._last is None:
            return (0.0, 0.0)
        return (-self._last.dx, -self._last.dy)

    @property
    def residual_deg(self) -> "float | None":
        """Degrees of image rotation still to go. Zero means squared up."""
        if self._smoothed is None:
            return None
        return wrap_deg(self._target - self._smoothed)

    def direction_hint(self) -> str:
        """``"unknown"`` | ``"good"`` | ``"reverse"``.

        MEASURED, never derived. Whether turning the camera clockwise rotates
        the image clockwise depends on how many mirrors sit in the optical
        path, which software cannot know — so the operator is told to start
        turning either way, and this latches once real motion has been seen.
        """
        if len(self._track) < 2:
            return "unknown"
        phi_now, res_now = self._track[-1]
        for phi_old, res_old in self._track:
            if abs(wrap_deg(phi_now - phi_old)) >= MOTION_MIN_DEG:
                return "good" if abs(res_now) < abs(res_old) else "reverse"
        return "unknown"
