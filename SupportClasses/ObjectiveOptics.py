"""
Objective optics — the numbers that SIZE a focus sweep.

An autofocus sweep has two questions that only the optics can answer:

* **How finely must I step?** Too coarse and a real focus peak hides between
  samples; too fine and you sweep the encoder rather than the optics. The
  natural scale is the **depth of field**.
* **How far may I travel?** The microscope focus drive moves the objective
  *toward the specimen*. The bound is the objective's **working distance** —
  a 20x with ~1 mm WD physically cannot sweep ±1 mm, and asking it to is a
  collision with the plate.

Both answers therefore come from the objective itself, never from a constant.
``MountedOptic`` carries ``numerical_aperture`` / ``working_distance_mm`` /
``magnification`` as floats for exactly this reason; re-parsing them out of the
human-readable ``detail`` string would fail silently into a wrong step size or a
wrong collision bound, which is the worst possible failure mode here.

This module is pure: no Qt, no hardware, no I/O, and numpy is never needed.
Every value it returns is a number a test can check by hand.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ── constants, each with its physical justification ───────────────────────

#: Illumination wavelength (µm) for the diffraction term. 0.55 µm is the green
#: peak of both the photopic response and a typical sensor's sensitivity, and is
#: the value Nikon's own published depth-of-field figures assume.
DEFAULT_WAVELENGTH_UM = 0.55

#: Refractive index between front lens and specimen. 1.0 = dry. Every objective
#: on this rig is dry; an oil/water objective must pass its own value or the
#: diffraction term is wrong by that factor.
DEFAULT_IMMERSION_N = 1.0

#: Fraction of the working distance a sweep may span on either side of centre.
#: THE PRIMARY COLLISION GUARD. The focus drive raises the objective toward the
#: plate, so the front lens has WD millimetres of room and no more. A quarter of
#: it leaves three quarters as margin against the plate not being where the
#: model thinks — which is precisely the thing being measured, so the margin
#: cannot be argued down by appealing to the measurement.
WD_SWEEP_FRACTION = 0.25

#: Half-range (µm) when the body reports no working distance. Deliberately tiny:
#: an unknown WD is not a licence to sweep far, and a caller that needs more must
#: supply the number rather than inherit a guess. Callers MUST surface that this
#: fallback is in force — a silently narrow sweep looks like "no peak found".
NO_WD_FALLBACK_HALF_RANGE_UM = 50.0

#: Best achievable 1-sigma focus repeatability, as a fraction of depth of field.
#: DOF/6 is the standard rule of thumb for a well-sampled through-focus curve on
#: a high-contrast feature; it is a FLOOR, not a promise.
FOCUS_SIGMA_DOF_FRACTION = 1.0 / 6.0

#: Expected width of the focus response at half maximum, as a multiple of DOF.
#: The DOF *is* the axial range over which the image looks sharp, so the metric's
#: response is necessarily of that order — somewhat broader, because the metric
#: integrates broadband spatial frequencies whose contrast decays over more than
#: one DOF.
EXPECTED_FWHM_DOF_BAND = (1.0, 3.0)

#: Outside this the curve is not a focus response at all.
#:  * below 0.5x DOF is PHYSICALLY IMPOSSIBLE — you cannot get a focus response
#:    narrower than the diffraction-limited depth of field; it means aliasing
#:    (step too coarse, a spurious 3-sample spike fitted) or one hot frame.
#:  * above 6x DOF means a thick feature, drifting illumination (auto-exposure!),
#:    or a metric dominated by fixed pattern. The peak is not a surface.
HARD_FWHM_DOF_BAND = (0.5, 6.0)

#: Tolerance on the declared field number before the geometry is called a
#: contradiction. Generous, because a demagnifying coupler is legitimate and the
#: declared value is often nominal.
FIELD_NUMBER_TOLERANCE = 0.15


@dataclass(frozen=True, kw_only=True)
class ObjectiveOptics:
    """Everything needed to size a sweep through one objective.

    ``um_per_px_sample`` is sample-side µm per pixel **at the resolution the
    sweep will actually run at**. If the sweep is binned, pass the binned value —
    every derived quantity then re-derives consistently rather than silently
    mixing resolutions.
    """

    label: str = ""
    position: int = 0

    magnification: float | None = None
    numerical_aperture: float | None = None
    working_distance_mm: float | None = None

    um_per_px_sample: float | None = None
    frame_wh: tuple[int, int] | None = None

    immersion_n: float = DEFAULT_IMMERSION_N
    wavelength_um: float = DEFAULT_WAVELENGTH_UM
    #: Camera-port field number in mm, if the operator declared one. None skips
    #: the geometry cross-check entirely.
    field_number_mm: float | None = None
    #: Binning in force for the sweep. Recorded for provenance only —
    #: ``um_per_px_sample`` is expected to already reflect it.
    binning: int = 1

    def describe(self) -> str:
        bits = [self.label or f"position {self.position}"]
        if self.numerical_aperture:
            bits.append(f"NA {self.numerical_aperture:g}")
        if self.working_distance_mm:
            bits.append(f"WD {self.working_distance_mm:g} mm")
        if self.um_per_px_sample:
            bits.append(f"{self.um_per_px_sample:.4g} µm/px")
        return " · ".join(bits)


def refuse_if_incomplete(o: ObjectiveOptics) -> str:
    """"" if this objective can size a sweep, else an operator-actionable why.

    Refusing is the point. Substituting a default NA or a default working
    distance produces a plan that looks entirely reasonable and is wrong by
    whatever the real optic differs by — and the working-distance one is a
    collision.
    """
    who = o.label or f"position {o.position}"
    if not o.numerical_aperture or o.numerical_aperture <= 0:
        return (f"{who} reports no numerical aperture, so the focus step cannot "
                f"be sized from its depth of field. Read the objectives from the "
                f"microscope on Hardware Setup → Microscope, or drop it from the "
                f"ladder.")
    if not o.um_per_px_sample or o.um_per_px_sample <= 0:
        return (f"{who} has no µm/px calibration for this camera, so its depth "
                f"of field and field of view are unknown. Run the objective "
                f"calibration on Hardware Setup → Cameras.")
    return ""


def depth_of_field_um(o: ObjectiveOptics) -> float | None:
    """Berek's two-term depth of field, in µm. None if NA or µm/px is unknown.

        DOF = λ·n/NA²  +  n·(µm per px at the sample)/NA

    First term is wave-optical (diffraction), second geometric — the circle of
    confusion set by how finely the DETECTOR samples the image.

    Inoué & Spring, *Video Microscopy: The Fundamentals* 2e, Plenum 1997, p.31;
    Nikon MicroscopyU, "Depth of Field and Depth of Focus" (Berek 1927).

    The textbook geometric term is ``n·e/(M·NA)`` for detector-resolved distance
    ``e`` and total magnification ``M``. But ``e/M`` IS the sample-side µm/px,
    which this repo already stores per camera+objective as a MEASURED quantity
    (``ObjectiveCalibration.measured_um_per_px``). Using it directly removes the
    sensor-pitch and coupler-magnification bookkeeping — and, more importantly,
    drives the formula from a measured number rather than a nameplate one.

    ``e`` is taken as ONE pixel, not the Nyquist two. That halves the geometric
    term, giving a smaller DOF, a finer step and a conservative plan. This is
    deliberate — do not "fix" it to 2 px.
    """
    na = o.numerical_aperture
    upp = o.um_per_px_sample
    if not na or na <= 0 or not upp or upp <= 0:
        return None
    n = float(o.immersion_n or DEFAULT_IMMERSION_N)
    lam = float(o.wavelength_um or DEFAULT_WAVELENGTH_UM)
    diffraction = lam * n / (na * na)
    geometric = n * float(upp) / na
    return float(diffraction + geometric)


def fov_um(o: ObjectiveOptics) -> tuple[float, float] | None:
    """Field of view (width, height) in sample-side µm, or None."""
    if not o.um_per_px_sample or not o.frame_wh:
        return None
    try:
        w, h = int(o.frame_wh[0]), int(o.frame_wh[1])
    except (TypeError, ValueError, IndexError):
        return None
    if w <= 0 or h <= 0:
        return None
    return (w * float(o.um_per_px_sample), h * float(o.um_per_px_sample))


def achievable_focus_sigma_um(o: ObjectiveOptics) -> float | None:
    """Best 1-sigma focus repeatability this objective can deliver (µm)."""
    dof = depth_of_field_um(o)
    return None if dof is None else dof * FOCUS_SIGMA_DOF_FRACTION


def expected_fwhm_band_um(o: ObjectiveOptics) -> tuple[float, float] | None:
    """Focus-curve FWHM band that indicates a healthy measurement (µm)."""
    dof = depth_of_field_um(o)
    if dof is None:
        return None
    lo, hi = EXPECTED_FWHM_DOF_BAND
    return (dof * lo, dof * hi)


def hard_fwhm_band_um(o: ObjectiveOptics) -> tuple[float, float] | None:
    """FWHM band outside which the curve is refused outright (µm)."""
    dof = depth_of_field_um(o)
    if dof is None:
        return None
    lo, hi = HARD_FWHM_DOF_BAND
    return (dof * lo, dof * hi)


def wd_bounded_half_range_um(o: ObjectiveOptics) -> tuple[float, bool]:
    """``(half_range_um, wd_known)`` — how far a sweep may go from centre.

    This is the collision guard, expressed as a pure number so it can be unit
    tested without a microscope. ``wd_known`` False means the fallback is in
    force and the caller MUST say so: a silently narrow sweep is reported by the
    peak finder as "no peak in range", which reads like an optics problem rather
    than a missing datum.
    """
    wd = o.working_distance_mm
    if not wd or wd <= 0:
        return (NO_WD_FALLBACK_HALF_RANGE_UM, False)
    return (float(wd) * 1000.0 * WD_SWEEP_FRACTION, True)


def check_field_number(o: ObjectiveOptics) -> str:
    """"" unless the declared field number contradicts the measured geometry.

    The illuminated field at the intermediate image plane is the field number;
    a sensor cannot see more of the specimen than ``FN / magnification``. If the
    measured µm/px and the frame size together imply a larger field than the
    operator declared, one of the three is wrong — and a wrong µm/px propagates
    straight into the geometric term of the depth of field, hence into every step
    size in the ladder.

    Skipped entirely when no field number is declared (the common case), because
    a demagnifying coupler makes the naive comparison meaningless.
    """
    if not o.field_number_mm or o.field_number_mm <= 0:
        return ""
    if not o.magnification or o.magnification <= 0:
        return ""
    f = fov_um(o)
    if f is None:
        return ""
    diag_um = math.hypot(f[0], f[1])
    required_fn_mm = diag_um * float(o.magnification) / 1000.0
    limit = float(o.field_number_mm) * (1.0 + FIELD_NUMBER_TOLERANCE)
    if required_fn_mm <= limit:
        return ""
    who = o.label or f"position {o.position}"
    return (f"{who}: the measured µm/px and frame size imply a "
            f"{required_fn_mm:.1f} mm field at the camera port, but the declared "
            f"field number is {o.field_number_mm:g} mm. The µm/px calibration, "
            f"the frame size or the declared field number is wrong — and a wrong "
            f"µm/px mis-sizes every focus step.")


def from_mounted_optic(optic, *, um_per_px_sample: float | None,
                       frame_wh: tuple[int, int] | None = None,
                       field_number_mm: float | None = None,
                       immersion_n: float = DEFAULT_IMMERSION_N,
                       wavelength_um: float = DEFAULT_WAVELENGTH_UM,
                       binning: int = 1,
                       ) -> tuple[ObjectiveOptics | None, str]:
    """Build from a :class:`MicroscopeControl.MountedOptic`. ``(optics, why)``.

    Reads the NUMERIC fields only. It must never fall back to parsing ``detail``:
    that string is formatted for humans, and a parse that quietly fails would
    produce a plausible plan built on a wrong NA or a wrong working distance.
    """
    if optic is None:
        return (None, "no objective in that nosepiece position")
    if not getattr(optic, "present", False):
        pos = getattr(optic, "position", 0)
        return (None, f"nosepiece position {pos} is empty")
    o = ObjectiveOptics(
        label=str(getattr(optic, "label", "") or ""),
        position=int(getattr(optic, "position", 0) or 0),
        magnification=getattr(optic, "magnification", None),
        numerical_aperture=getattr(optic, "numerical_aperture", None),
        working_distance_mm=getattr(optic, "working_distance_mm", None),
        um_per_px_sample=(None if um_per_px_sample is None
                          else float(um_per_px_sample)),
        frame_wh=frame_wh,
        immersion_n=immersion_n,
        wavelength_um=wavelength_um,
        field_number_mm=field_number_mm,
        binning=int(binning or 1),
    )
    why = refuse_if_incomplete(o)
    if why:
        return (None, why)
    return (o, "")
