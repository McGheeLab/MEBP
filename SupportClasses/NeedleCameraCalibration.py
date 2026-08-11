"""
NeedleCameraCalibration.py — solve a needle side camera's axes from TWO
measured stage moves, using the needle itself as the reference object.

v7.10. Operator: *"The needle cameras need the needle as a reference object to
be able to calibrate stage motion to its rotation etc, so the needle must move
on the needle camera's axis to be able to judge rotation. We should check z
motion, and a 45 degree stage motion to ensure it stays within the camera
frame."*

Pure: no Qt, no hardware, no cv2 at import time.

────────────────────────────────────────────────────────────────────────────
Why one move is not enough
────────────────────────────────────────────────────────────────────────────

The needle cameras are bolted to the **XY stage** and look sideways at a needle
that hangs from the Z axis. So:

* an **XY move** moves the CAMERA — the stationary needle (and everything else
  in view) appears to translate. ``plus_column_direction_deg``'s own docstring
  says as much: *"Moving the cameras by +m … makes stationary content shift"*.
* a **Z move** moves the NEEDLE and nothing else. The background does not move.

Until now only the XY leg was measured, and that is **rank one**: one image
vector for one stage direction. Everything else had to be assumed. Three
consequences, all of which this module measures instead:

**1. µm/px was systematically over-estimated, undetectably.** A side camera only
sees the component of a move that is perpendicular to its optical axis. If the
commanded direction is off the true lateral by an angle ``β``, the image moves
by ``D·cos β`` worth of pixels, so ``u = D / |p|`` comes out too LARGE by
``1/cos β`` — 6 % at 20°, 41 % at 45°. Nothing in the single-leg flow can see
this; the operator compensates by hand, clicking presets and keeping "the
longest arrow", which is exactly a manual search for ``cos β → 1``.

A **Z** move has no such problem: lab-vertical is perpendicular to a horizontal
optical axis whatever the camera's azimuth. So ``u_z`` is the trustworthy scale,
and ``u_lat / u_z = 1/cos β`` recovers the very error that used to be invisible.

**2. The sensor roll was inferred from the lateral leg, which requires knowing
where lateral IS** — circular. The Z leg gives it directly: lab-vertical is a
known direction, so the angle at which it lands on the sensor IS the roll.

**3. "Image rows map to stage Z" was an unverified assumption.**
``TwoCameraNeedleAligner``'s docstring asserts it and the whole Z-centring path
depends on it, but nothing ever checked it, and the Z SIGN was a manual
*Invert Z* checkbox — a guess the operator had to get right. Two legs give the
angle between the lateral and vertical responses, whose deviation from 90° is a
real quality figure, and the sign falls out of the measurement.

────────────────────────────────────────────────────────────────────────────
Model
────────────────────────────────────────────────────────────────────────────

A side camera = a pinhole with a roughly horizontal optical axis plus a roll
``ψ`` about it. Under that model, with image ``+x`` = right and ``+y`` = DOWN:

* lab **up** (+Z in the height frame) images along ``(sin ψ, −cos ψ)``
* lab **horizontal ⟂ optical axis** images along ``±(cos ψ, sin ψ)``

Two unknowns in plane (the roll and the optical azimuth); the Z leg pins the
first without touching the second, which is what makes the pair well-posed
where one move was not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# A tracked feature must move at least this far to count as having moved at all.
# Below it the two frames are indistinguishable and any angle read off them is
# noise — on the Z leg it also means the tracker locked onto the stationary
# BACKGROUND instead of the needle, which is the failure this catches.
MIN_TRAVEL_PX = 6.0

# The measured lateral and vertical image responses should be perpendicular
# (square pixels, a rigid scene). Past this the model does not hold — most
# often because one leg tracked a different object.
MAX_ORTHOGONALITY_ERR_DEG = 12.0

# u_lat/u_z is 1/cos(beta) >= 1 by construction. A ratio far above 1 means the
# lateral preset is badly off the camera's true lateral direction; far BELOW 1
# is not physical and indicates a bad track.
MAX_SCALE_RATIO = 1.45          # ~ beta > 46 deg — refuse, the µm/px is junk
WARN_SCALE_RATIO = 1.06         # ~ beta > 19 deg — usable, but say so
MIN_SCALE_RATIO = 0.85

# Fraction of the SHORTER frame dimension a tracked feature may traverse and
# still be expected to stay in view. select_trackable_patch keeps its patch
# within the central region (margin_frac 0.22), so roughly (0.5 - 0.22) of the
# frame is available; 0.25 leaves headroom for the patch's own half-width.
IN_FRAME_TRAVEL_FRAC = 0.25


def _norm(dx: float, dy: float) -> float:
    return math.hypot(float(dx), float(dy))


def wrap_deg(angle: float) -> float:
    """Normalize to (-180, 180]."""
    a = (float(angle) + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def fold_perpendicular_deg(angle: float) -> float:
    """Fold an angle into (-90, 90] — i.e. modulo a 180° ambiguity."""
    a = (float(angle) + 90.0) % 180.0 - 90.0
    return 90.0 if a == -90.0 else a


@dataclass(frozen=True)
class NeedleCameraAxes:
    """The solved geometry of one needle side camera.

    ``um_per_px`` is the value that should be COMMITTED: it comes from the Z
    leg, which cannot be foreshortened. ``um_per_px_lateral`` is the legacy
    single-leg figure, kept so the two can be compared and the difference
    explained rather than silently absorbed.

    **The Z leg stands alone.** It fully determines the scale, the roll and the
    Z direction — the XY leg adds only the aligner's mount direction and the two
    cross-checks that need a second vector. So the lateral fields are optional,
    and ``has_lateral`` says whether they mean anything. Requiring an XY leg for
    a rotation correction would be requiring the less trustworthy measurement to
    validate the more trustworthy one.
    """

    um_per_px: float                 # from the Z leg — the trustworthy one
    roll_deg: float                  # sensor roll, from lab-vertical
    z_row_sign: float                # +1: moving the needle UP decreases the row
    z_travel_px: float
    # Lateral leg — all None when the Z leg was measured on its own.
    um_per_px_lateral: Optional[float] = None   # foreshortened by 1/cos B
    orthogonality_err_deg: Optional[float] = None  # |angle between legs| - 90
    scale_ratio: Optional[float] = None         # um_per_px_lateral / um_per_px
    off_lateral_deg: Optional[float] = None     # beta: preset vs true lateral
    lateral_travel_px: Optional[float] = None

    @property
    def has_lateral(self) -> bool:
        """True when an XY leg was measured alongside the Z leg."""
        return self.lateral_travel_px is not None

    @property
    def is_trustworthy(self) -> bool:
        return not self.refusal()

    def refusal(self) -> Optional[str]:
        """Why this solve must not be committed, or None.

        Only checks what was actually measured: a Z-only solve is judged on the
        Z leg alone.
        """
        if self.has_lateral and self.lateral_travel_px < MIN_TRAVEL_PX:
            return (f"The XY leg moved the tracked feature only "
                    f"{self.lateral_travel_px:.1f} px. Either the move went "
                    f"along this camera's optical axis (it just changes focus) "
                    f"or the feature left the frame — try another direction.")
        if self.z_travel_px < MIN_TRAVEL_PX:
            return (f"The Z leg moved the tracked feature only "
                    f"{self.z_travel_px:.1f} px. A Z move moves the NEEDLE and "
                    f"nothing else, so a near-zero reading means the tracker "
                    f"locked onto the stationary background. Re-frame so the "
                    f"needle fills more of the view, or use a larger Z move.")
        if not self.has_lateral:
            return None
        if abs(self.orthogonality_err_deg) > MAX_ORTHOGONALITY_ERR_DEG:
            return (f"The XY and Z responses are {90 + self.orthogonality_err_deg:.1f}° "
                    f"apart, not 90°. For square pixels viewing a rigid scene "
                    f"they must be perpendicular, so the two legs are probably "
                    f"tracking different objects.")
        if not (MIN_SCALE_RATIO <= self.scale_ratio <= MAX_SCALE_RATIO):
            return (f"The two legs disagree about scale by "
                    f"{self.scale_ratio:.2f}x. Above ~1.45 the XY move is more "
                    f"than 45° off this camera's lateral direction, which makes "
                    f"its um/px meaningless; below 1.0 is not physical at all.")
        return None

    def advisories(self) -> list[str]:
        """Non-blocking notes worth showing the operator."""
        out: list[str] = []
        if not self.has_lateral:
            out.append(
                "Measured from the Z leg alone. That fully determines the "
                "um/px, the sensor roll and the Z direction — only the "
                "needle aligner's column->stage MOUNT direction needs an XY "
                "move, so any previously measured one is left as it was.")
        elif self.scale_ratio > WARN_SCALE_RATIO:
            out.append(
                f"The XY move was about {self.off_lateral_deg:.0f}deg off this "
                f"camera's lateral direction, so the single-move um/px would "
                f"have read {self.um_per_px_lateral:.4f} instead of "
                f"{self.um_per_px:.4f} um/px — {(self.scale_ratio - 1) * 100:.0f}% "
                f"high. The Z leg's value is the one being stored.")
        if abs(self.roll_deg) > 2.0:
            out.append(
                f"Sensor roll is {self.roll_deg:+.1f}deg. The aligner reads the "
                f"needle's column offset and assumes image rows are stage Z, so "
                f"a roll leaks Z error into the XY solve; squaring the mount up "
                f"removes it.")
        if self.z_row_sign < 0:
            out.append(
                "Moving the needle UP increases the image row on this camera "
                "(the view is vertically inverted). That is now measured, so "
                "the Z-centring sign no longer depends on the Invert Z "
                "checkbox.")
        return out


def solve_needle_camera_axes(*, z_um: float, z_dx_px: float, z_dy_px: float,
                             lateral_um: Optional[float] = None,
                             lateral_dx_px: Optional[float] = None,
                             lateral_dy_px: Optional[float] = None
                             ) -> NeedleCameraAxes:
    """Solve one needle camera's axes. The Z leg is required; XY is optional.

    ``z_um`` is the commanded Z move in the HEIGHT frame (positive = up) and
    ``(z_dx_px, z_dy_px)`` its image displacement. Image convention: +x right,
    +y DOWN.

    Supplying the XY leg (``lateral_um`` + its displacement) adds the aligner's
    mount direction and two cross-checks — orthogonality and the foreshortening
    ratio. Omitting it is a first-class mode, not a degraded one: the Z leg is
    the trustworthy measurement, so a rotation correction must not be gated on
    the lateral one being available or good.

    Never raises on a degenerate input — it returns an object whose
    :meth:`NeedleCameraAxes.refusal` explains what went wrong. A calibration
    that quietly returns a plausible number for an unusable measurement is
    worse than one that refuses.
    """
    have_lat = (lateral_um is not None and lateral_dx_px is not None
                and lateral_dy_px is not None)
    lat_px = _norm(lateral_dx_px, lateral_dy_px) if have_lat else None
    z_px = _norm(z_dx_px, z_dy_px)

    u_lat = None
    if have_lat:
        u_lat = (abs(float(lateral_um)) / lat_px) if lat_px > 1e-9 else 0.0
    u_z = (abs(float(z_um)) / z_px) if z_px > 1e-9 else 0.0

    # Roll: lab-vertical is a KNOWN direction, so where it lands on the sensor
    # is the roll, with no assumption about the camera's azimuth. Express the Z
    # response as "up in the image" (negate dy, which points down) and take its
    # deviation from straight up. Fold by 180° so a vertically-inverted mount
    # reports a small roll rather than ~180°; the inversion is carried by
    # z_row_sign instead, where it can be acted on.
    roll = 0.0
    z_row_sign = 1.0
    if z_px > 1e-9:
        up_x, up_y = float(z_dx_px), -float(z_dy_px)
        if float(z_um) < 0:                      # commanded DOWN — flip to up
            up_x, up_y = -up_x, -up_y
        # atan2(x, y) measured from the +up axis, clockwise positive.
        roll = fold_perpendicular_deg(math.degrees(math.atan2(up_x, up_y)))
        z_row_sign = 1.0 if (up_y > 0) else -1.0

    if not have_lat:
        return NeedleCameraAxes(
            um_per_px=u_z, roll_deg=roll, z_row_sign=z_row_sign,
            z_travel_px=z_px)

    # Orthogonality: the lateral response should be perpendicular to the
    # vertical one. Fold by 180° — which of the two perpendicular senses the
    # lateral leg landed on is the camera's azimuth, not an error.
    ortho_err = 0.0
    if lat_px > 1e-9 and z_px > 1e-9:
        a_lat = math.degrees(math.atan2(lateral_dy_px, lateral_dx_px))
        a_z = math.degrees(math.atan2(z_dy_px, z_dx_px))
        ortho_err = fold_perpendicular_deg(wrap_deg(a_lat - a_z) - 90.0)

    ratio = (u_lat / u_z) if u_z > 1e-12 else 0.0
    # u_lat/u_z = 1/cos(beta) exactly, so beta = acos(u_z/u_lat). Clamped:
    # measurement noise can push the ratio a hair below 1, where acos is fine,
    # and well below 1 is unphysical and already refused on the ratio itself.
    off_lateral = 0.0
    if ratio > 1e-9:
        off_lateral = math.degrees(math.acos(max(0.0, min(1.0, 1.0 / ratio))))

    return NeedleCameraAxes(
        um_per_px=u_z,
        um_per_px_lateral=u_lat,
        roll_deg=roll,
        z_row_sign=z_row_sign,
        orthogonality_err_deg=ortho_err,
        scale_ratio=ratio,
        off_lateral_deg=off_lateral,
        lateral_travel_px=lat_px,
        z_travel_px=z_px,
    )


def max_in_frame_move_um(um_per_px: float, frame_w: int, frame_h: int,
                         travel_frac: float = IN_FRAME_TRAVEL_FRAC) -> float:
    """Largest move (µm) whose tracked feature should still be in view.

    Returns 0.0 when the scale or frame size is unknown, which callers treat as
    "cannot check" rather than "refuse".

    This check exists because the two ways a leg can fail are **indistinguishable
    after the fact**: a move along the optical axis and a move that pushed the
    feature off the sensor both come back as "almost no displacement, low
    confidence". Bounding the move up front means a small reading can only mean
    the first, which is the one the operator can fix by choosing another
    direction.
    """
    try:
        u = float(um_per_px)
        short = min(int(frame_w), int(frame_h))
    except (TypeError, ValueError):
        return 0.0
    if u <= 0 or short <= 0:
        return 0.0
    return float(short) * float(travel_frac) * u


def in_frame_refusal(move_um: float, um_per_px: float, frame_w: int,
                     frame_h: int, *, what: str = "move") -> Optional[str]:
    """Why ``move_um`` would push the tracked feature out of view, or None.

    Silent when the scale is unknown — an uncalibrated camera has to make its
    first measurement somehow, and refusing it would be a deadlock.
    """
    limit = max_in_frame_move_um(um_per_px, frame_w, frame_h)
    if limit <= 0:
        return None
    if abs(float(move_um)) <= limit:
        return None
    return (f"A {abs(float(move_um)):.0f} um {what} is about "
            f"{abs(float(move_um)) / float(um_per_px):.0f} px at this camera's "
            f"scale, which would carry the tracked feature out of the frame. "
            f"Keep it under about {limit:.0f} um.")
