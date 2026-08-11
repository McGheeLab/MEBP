"""Non-contact plate-bottom measurement — the arithmetic, with no hardware.

THE GESTURE
-----------
The operator focuses the microscope on the glass (``f0``). The focus then moves
UP by a margin ``X``, and the needle descends to roughly that plane. The focus
is swept through the STATIONARY tip, and the peak of that curve says where the
tip actually is::

    h = (f_tip - f0) * focus_up_sign          # tip height above the glass, µm

``h`` is MEASURED, not commanded. The needle does not have to arrive at exactly
``X`` — it only has to arrive somewhere the objective can see it. That is the
whole point: the answer does not inherit the error in the geometric guess that
positioned the needle, and the tip never comes closer to the glass than ``X``.

WHY A LADDER OF MARGINS
-----------------------
Each ``X`` yields an INDEPENDENT estimate of one number, the plate bottom. Their
agreement is the verification — there is nothing else to check them against. A
single margin reproduces exactly the weakness of
``NeedleFocusTemplateStore.focus_to_needle_z_mm``: one point fits an offset and
must assume the scale.

Run them LARGEST FIRST. Only the first descent trusts the geometric guess; every
later one is bounded by a measurement.

THE SCALE MUST BE 1
-------------------
Both axes measure the displacement of the same rigid tip, so the ratio of the
needle's height change to the focal plane's height change is exactly 1 by
physics. A fitted value that is not 1 is a BUG REPORT — a units slip, a wrong
``focus_units_per_um``, a wrong ``steps_per_mm`` — and applying it would bake the
error in while hiding the fault the measurement just found. So: gate it, then
snap to exactly ±1 and apply sign and offset only. This module never returns a
scale to apply; :func:`focal_sign_and_offset` returns the ±1 form the
``PlateFocusDatumStore`` consumes.

...but the gate is only meaningful if the data can support it. Slope precision
is ``sigma * sqrt(12) / (span * sqrt(n))``, so with the default ladder (900 µm
span, 4 rungs) it is ~0.3 % at 10x and ~1.8 % at 4x, where the achievable
per-point sigma is DOF/6 ≈ 9.5 µm. When the span cannot support the tolerance,
:func:`scale_verifiable` says so and the caller must report the scale as
UNVERIFIED rather than as passed. Claiming a check that could not have failed is
worse than not running it.

FRAMES
------
``z`` is zero-ref mm; ``zdir`` is ``StageController.print_z_dir()`` (+1 or -1);
heights above the plate bottom follow ``plate_relative_to_zref``. ``f`` is the
microscope focus axis in µm and ``focus_up_sign`` is +1 when a larger focus
reading raises the focal plane (``MicroscopeConfigStore.focus_up_is_positive``).

Pure: no Qt, no cv2, numpy imported lazily inside the one function that needs it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# The scale band is owned by the store that consumes it — a second copy here
# would drift the day one of them is retuned.
from SupportClasses.PlateFocusDatumStore import SCALE_TOLERANCE, check_scale

__all__ = [
    "MIN_TIP_CLEARANCE_UM", "DEFAULT_OFFSETS_UM", "DEFAULT_SPREAD_TOL_UM",
    "MIN_RUNGS_FOR_SLOPE", "MAX_OFFSET_UM",
    "RungMeasurement", "Reconciliation",
    "ladder_gate", "tip_height_um", "plate_bottom_from_rung",
    "needle_target_zref", "reconcile_rungs", "measured_scale",
    "slope_precision_frac", "scale_verifiable", "focal_sign_and_offset",
    "focus_needle_offset_mm",
]

# ── constants ────────────────────────────────────────────────────────

#: Smallest gap we will ever leave between the LOWEST-reaching bore and the
#: glass. Not a comfort margin — below this a stage settle or a plate that is
#: not quite flat closes the remaining distance.
MIN_TIP_CLEARANCE_UM = 50.0

#: Largest-first. The 900 µm span is what makes the scale checkable at 10x;
#: shortening it silently costs the verification, not just precision.
DEFAULT_OFFSETS_UM = (1000.0, 500.0, 200.0, 100.0)

#: Rung-to-rung agreement. Tighter than PlateZPlane's 50 µm residual tolerance
#: because this is one point measured repeatedly, not a plane fitted across one.
DEFAULT_SPREAD_TOL_UM = 25.0

#: Two points define a slope; the third is the first one that can disagree.
MIN_RUNGS_FOR_SLOPE = 2

#: A margin larger than this is almost certainly a units mistake (mm typed as
#: µm). Refuse rather than command a 5 mm descent from a guess.
MAX_OFFSET_UM = 3000.0


# ── data ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class RungMeasurement:
    """One margin's worth of evidence.

    ``needle_z_zref_mm`` is the READ-BACK needle Z, never the commanded target —
    the needle may have been clamped by the print floor or the soft limits, and
    a commanded value would silently absorb that as plate-bottom error.
    """
    margin_um: float
    needle_z_zref_mm: float
    focus_tip_um: float
    focus_sigma_um: float = 0.0
    fwhm_um: float = 0.0
    adopted_by: str = "auto"            # "auto" | "operator"
    refusal: str = ""

    @property
    def usable(self) -> bool:
        return not self.refusal


@dataclass(frozen=True)
class Reconciliation:
    """What the ladder collectively says, and whether to believe it."""
    plate_bottom_zref_mm: float | None = None
    spread_um: float = 0.0
    per_rung_zref_mm: tuple = ()
    n_used: int = 0
    span_um: float = 0.0
    scale: float | None = None
    scale_ok: bool = False
    scale_verified: bool = False
    scale_note: str = ""
    refusal: str = ""

    @property
    def ok(self) -> bool:
        return not self.refusal and self.plate_bottom_zref_mm is not None


# ── the gate that is not optional ────────────────────────────────────

def ladder_gate(offsets_um, longest_bore_mm: float,
                min_clearance_um: float = MIN_TIP_CLEARANCE_UM) -> tuple:
    """``(ok, why)`` — is every margin in this ladder physically safe?

    THE ROI SCORES BORE 0, THE DATUM — but a bore protruding further reaches the
    glass first. A needle whose longest bore stands 200 µm past the datum,
    measured at a 100 µm margin, puts that bore 100 µm THROUGH the plate while
    every number on screen reads as a comfortable clearance. Same failure
    ``PickAndPlaceManager._descend_z_mm`` and ``_touch_target_zref`` already
    exist to prevent, arriving by a new route.

    Refuses rather than clamping: a clamped margin is a measurement of a
    different height than the one reported, which is worse than no measurement.
    """
    try:
        vals = [float(v) for v in offsets_um]
    except (TypeError, ValueError):
        return False, "The margin list contains a value that is not a number."
    if not vals:
        return False, "Add at least one margin to measure at."
    if any(not math.isfinite(v) for v in vals):
        return False, "The margin list contains a non-finite value."
    if any(v <= 0 for v in vals):
        return False, "Every margin must be greater than zero."
    if any(v > MAX_OFFSET_UM for v in vals):
        return (False,
                f"A margin above {MAX_OFFSET_UM:.0f} µm is almost certainly a "
                f"units mistake (mm typed as µm). Largest given: "
                f"{max(vals):.0f} µm.")
    if len(set(vals)) != len(vals):
        return False, "The same margin is listed twice — each must be distinct."
    if vals != sorted(vals, reverse=True):
        return (False,
                "List the margins largest first. Only the first descent trusts "
                "the geometric guess; each later one is bounded by the "
                "measurement before it.")

    try:
        bore = max(0.0, float(longest_bore_mm)) * 1000.0
    except (TypeError, ValueError):
        bore = 0.0
    floor = bore + max(0.0, float(min_clearance_um))
    bad = [v for v in vals if v < floor]
    if bad:
        return (False,
                f"Margin {min(bad):.0f} µm is below the {floor:.0f} µm floor for "
                f"this needle. The focus is scored on bore 1, but the longest "
                f"bore reaches {bore:.0f} µm further down — at that margin it "
                f"would be {floor - min(bad):.0f} µm into the glass. Raise the "
                f"margin or fit a needle whose bores are closer to coplanar.")
    return True, ""


# ── per-rung arithmetic ──────────────────────────────────────────────

def tip_height_um(focus_tip_um: float, focus_zero_um: float,
                  focus_up_sign: float) -> float:
    """Measured height of the tip above the glass (µm).

    ``focus_zero_um`` is the operator's on-glass focus. An error in it is a
    COMMON-MODE error: it shifts every rung by the same amount, so the ladder's
    agreement check cannot see it. It is checked once, at the source, by sweeping
    the glass itself — not here.
    """
    return (float(focus_tip_um) - float(focus_zero_um)) * float(focus_up_sign)


def plate_bottom_from_rung(needle_z_zref_mm: float, height_um: float,
                           zdir: float) -> float:
    """Plate bottom (zero-ref mm) implied by one rung.

    Inverse of ``plate_relative_to_zref``: the needle sits ``height_um`` above
    the bottom, so the bottom is that far below it in the height frame.
    """
    return float(needle_z_zref_mm) - float(zdir) * (float(height_um) / 1000.0)


def needle_target_zref(plate_bottom_zref_mm: float, margin_um: float,
                       zdir: float) -> float:
    """Where to send the needle for a given margin (zero-ref mm)."""
    return float(plate_bottom_zref_mm) + float(zdir) * (float(margin_um) / 1000.0)


# ── the ladder, collectively ─────────────────────────────────────────

def reconcile_rungs(rungs, *, focus_zero_um: float, focus_up_sign: float,
                    zdir: float, spread_tol_um: float = DEFAULT_SPREAD_TOL_UM,
                    scale_tol: float = SCALE_TOLERANCE) -> Reconciliation:
    """Fold a ladder into one plate bottom, or refuse and say why.

    The median, not the mean: one rung that locked onto a reflection should not
    drag the answer a third of the way toward itself.
    """
    used = [r for r in rungs if getattr(r, "usable", False)]
    if not used:
        return Reconciliation(refusal="No margin produced a usable focus peak.")

    heights = [tip_height_um(r.focus_tip_um, focus_zero_um, focus_up_sign)
               for r in used]
    per_rung = tuple(plate_bottom_from_rung(r.needle_z_zref_mm, h, zdir)
                     for r, h in zip(used, heights))

    ordered = sorted(per_rung)
    n = len(ordered)
    median = (ordered[n // 2] if n % 2
              else 0.5 * (ordered[n // 2 - 1] + ordered[n // 2]))
    spread_um = (max(ordered) - min(ordered)) * 1000.0

    span_um = (max(heights) - min(heights)) if len(heights) > 1 else 0.0
    scale, scale_ok, scale_note, verified = None, False, "", False
    if len(used) >= MIN_RUNGS_FOR_SLOPE:
        scale = measured_scale(used, focus_zero_um=focus_zero_um,
                               focus_up_sign=focus_up_sign, zdir=zdir)
        if scale is not None:
            scale_ok, why = check_scale(scale)
            sigma = max((float(r.focus_sigma_um) for r in used), default=0.0)
            verified = scale_verifiable(sigma, span_um, len(used), tol=scale_tol)
            if not verified:
                prec = slope_precision_frac(sigma, span_um, len(used))
                scale_note = (
                    f"Scale measured {scale:.4f}, but this ladder cannot verify "
                    f"it: {len(used)} rungs over {span_um:.0f} µm at ±{sigma:.1f} "
                    f"µm give a slope precision of {prec * 100:.1f} %, wider than "
                    f"the {scale_tol * 100:.0f} % band. Report it as UNVERIFIED — "
                    f"widen the margins or use a higher-power objective.")
            elif not scale_ok:
                scale_note = why
            else:
                scale_note = f"Scale {scale:.4f} — within ±{scale_tol * 100:.0f} %."
    else:
        scale_note = (f"Only {len(used)} usable rung — a slope needs at least "
                      f"{MIN_RUNGS_FOR_SLOPE}. The offset is measured; the scale "
                      f"is assumed.")

    refusal = ""
    if spread_um > float(spread_tol_um):
        lo_i = per_rung.index(min(per_rung))
        hi_i = per_rung.index(max(per_rung))
        refusal = (
            f"The margins disagree by {spread_um:.0f} µm, above the "
            f"{spread_tol_um:.0f} µm tolerance: the {used[lo_i].margin_um:.0f} µm "
            f"and {used[hi_i].margin_um:.0f} µm margins put the plate bottom in "
            f"different places. They measure one number, so a spread is a real "
            f"fault — backlash in the Z lead screw, a drifting tip focus datum, "
            f"or a units slip. Re-run before accepting.")
    elif scale is not None and verified and not scale_ok:
        refusal = (
            f"{scale_note} Both axes measure the same rigid tip, so this ratio "
            f"must be 1. A different value is a bug — check focus_units_per_um "
            f"and the Z steps/mm — not a calibration to apply.")

    return Reconciliation(
        plate_bottom_zref_mm=(None if refusal else median),
        spread_um=spread_um, per_rung_zref_mm=per_rung, n_used=len(used),
        span_um=span_um, scale=scale, scale_ok=scale_ok,
        scale_verified=verified, scale_note=scale_note, refusal=refusal)


def measured_scale(rungs, *, focus_zero_um: float, focus_up_sign: float,
                   zdir: float) -> float | None:
    """Needle height change ÷ focal-plane height change. Physics says 1.0.

    Least-squares slope so every rung contributes; ``None`` when the focal plane
    barely moved (the ratio is then meaningless, not merely imprecise).
    """
    used = [r for r in rungs if getattr(r, "usable", False)]
    if len(used) < MIN_RUNGS_FOR_SLOPE:
        return None
    # Focal-plane height (mm) and needle height (mm), both above the glass.
    xs = [tip_height_um(r.focus_tip_um, focus_zero_um, focus_up_sign) / 1000.0
          for r in used]
    ys = [float(zdir) * float(r.needle_z_zref_mm) for r in used]
    n = float(len(xs))
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-12:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return sxy / sxx


def slope_precision_frac(sigma_um: float, span_um: float, n: int) -> float:
    """Fractional 1-sigma uncertainty on the fitted slope.

    ``sigma * sqrt(12) / (span * sqrt(n))`` for roughly even spacing. Returns
    ``inf`` when the span is degenerate — a slope over no baseline has no
    precision, and a caller must not read that as "perfect".
    """
    try:
        span, sig, cnt = float(span_um), abs(float(sigma_um)), int(n)
    except (TypeError, ValueError):
        return float("inf")
    if span <= 0.0 or cnt < MIN_RUNGS_FOR_SLOPE:
        return float("inf")
    if sig <= 0.0:
        return 0.0
    return sig * math.sqrt(12.0) / (span * math.sqrt(float(cnt)))


def scale_verifiable(sigma_um: float, span_um: float, n: int,
                     tol: float = SCALE_TOLERANCE) -> bool:
    """Can this ladder actually resolve a ``tol``-sized scale error?

    If not, the gate would pass for any data at all, which reads on screen as a
    check that succeeded. Say UNVERIFIED instead.
    """
    return slope_precision_frac(sigma_um, span_um, n) <= float(tol)


def focal_sign_and_offset(plate_bottom_zref_mm: float, focus_zero_um: float,
                          focus_up_sign: float, zdir: float) -> tuple:
    """``(focal_sign, offset_mm)`` for ``PlateFocusDatumStore``.

    Satisfying its contract ``needle_z_zref = focal_sign * (focus_um/1000) +
    offset_mm``. Derived from the on-glass anchor, not fitted — the fit supplies
    only the ±1 that ``focal_sign`` already is, and taking the offset from the
    anchor keeps it tied to the one plane both axes were measured at.
    """
    sign = 1.0 if float(zdir) * float(focus_up_sign) >= 0 else -1.0
    offset = float(plate_bottom_zref_mm) - sign * (float(focus_zero_um) / 1000.0)
    return sign, offset


def focus_needle_offset_mm(rungs, focal_sign: float) -> tuple:
    """``(offset_mm, spread_mm, n)`` — the constant focal-plane ↔ needle-tip
    offset, estimated once per rung.

    Every usable rung pairs a focus reading taken ON the tip (``focus_tip_um``
    — the focal plane was swept until the tip was sharpest, so the two axes are
    looking at the same physical plane) with the needle Z read back at that
    moment (``needle_z_zref_mm``). Each pair is therefore an independent
    estimate of the one constant in the datum contract::

        needle_z_zref_mm = focal_sign * (focus_um / 1000) + offset_mm
        offset_k         = needle_z_k - focal_sign * f_tip_k / 1000

    The MEDIAN over rungs is returned (same outlier reasoning as
    :func:`reconcile_rungs`), with the max−min spread in mm. Constancy across
    rungs IS the verification: as the needle steps down the ladder the focal
    plane follows it, so a spread beyond noise means the two axes do not track
    1:1 — backlash, a units slip, or a drifting focus drive — and the ladder's
    spread/scale gates will name it.

    A strictly better estimator of the same constant than the single on-glass
    pair :func:`focal_sign_and_offset` uses, because the tip pairs are fitted
    peaks with a σ while ``f0`` is one hand-focused reading. Returns
    ``(None, 0.0, 0)`` when no rung is usable.
    """
    used = [r for r in rungs if getattr(r, "usable", False)]
    if not used:
        return None, 0.0, 0
    sign = 1.0 if float(focal_sign) >= 0 else -1.0
    offsets = sorted(
        float(r.needle_z_zref_mm) - sign * float(r.focus_tip_um) / 1000.0
        for r in used)
    n = len(offsets)
    median = (offsets[n // 2] if n % 2
              else 0.5 * (offsets[n // 2 - 1] + offsets[n // 2]))
    spread = offsets[-1] - offsets[0]
    return median, spread, n
