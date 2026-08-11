"""
Through-focus curve → best-focus Z, or a refusal.

A sweep produces (focus position, focus score) samples. Turning them into one
number is where a plate-leveling measurement is won or lost: the *location* of
the peak is the measurement, and a peak that is confidently wrong looks exactly
like a peak that is right.

So this module returns ``(FocusPeak, "")`` or ``(None, reason)`` — **never a
number with a caveat attached.** Every refusal below corresponds to a real way
the curve stops meaning "where the surface is".

THE ESTIMATOR
-------------
Weighted Gaussian fit to ``log(F - B)``: a linear least-squares parabola in
``ln(F - B)`` over the half-maximum set, seeded and cross-checked by the
closed-form 3-point log-parabola vertex.

* Naidu & Fisher, *A comparative analysis of algorithms for determining the peak
  position of a stripe to sub-pixel accuracy*, Proc. BMVC 1991 — the head-to-head
  that finds the Gaussian (log-parabola) estimator the best sub-sample peak
  locator on a bell-shaped response.
* Yeo, Ong, Jayasooriah & Sinniah, *Autofocusing for tissue microscopy*, Image
  and Vision Computing 11(10):629-639, 1993 — establishes that the through-focus
  response of gradient/Laplacian metrics is Gaussian-like.
* Groen, Young & Ligthart, *A comparison of different focus functions for use in
  autofocus algorithms*, Cytometry 6:81-91, 1985 — the canonical source for the
  unimodality / monotonicity / false-maximum criteria that became the refusals.
* Santos et al., *Evaluation of autofocus functions in molecular cytogenetic
  analysis*, J. Microscopy 188:264-272, 1997 — defines the width of the focus
  function at half maximum as a figure of merit. That is the FWHM below.
* Guo, *A simple algorithm for fitting a Gaussian function*, IEEE Signal
  Processing Magazine 28(5):134-137, 2011 — the intensity weighting that removes
  the log transform's bias toward low-SNR tail samples.

WHY NOT HILL-CLIMBING
---------------------
Fibonacci / hill-climbing search (Krotkov, *Focusing*, IJCV 1(3), 1987) converges
in fewer frames, but it never produces the CURVE — and the curve's shape is the
only thing that detects a wrong answer. A bracketed sweep costs the same order of
frames and yields the diagnostics. Take the sweep.

Pure: no Qt, no hardware, no I/O. numpy is imported lazily inside the fit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


# ── refusal codes ─────────────────────────────────────────────────────────

PEAK_AT_EDGE = "PEAK_AT_EDGE"
MONOTONIC = "MONOTONIC"
LOW_PROMINENCE = "LOW_PROMINENCE"
MULTIMODAL = "MULTIMODAL"
SATURATED = "SATURATED"
NON_CONCAVE = "NON_CONCAVE"
VERTEX_OUTSIDE_BRACKET = "VERTEX_OUTSIDE_BRACKET"
FWHM_MISMATCH = "FWHM_MISMATCH"
INSUFFICIENT_SAMPLES = "INSUFFICIENT_SAMPLES"
DRIFT = "DRIFT"
MIXED_EXPOSURE = "MIXED_EXPOSURE"
BAD_BASELINE = "BAD_BASELINE"


# ── tunables ──────────────────────────────────────────────────────────────

#: Fewer than this and neither a fit nor an FWHM is meaningful.
MIN_SAMPLES = 5

#: Minimum samples at or above half maximum for the weighted fit.
MIN_HALFMAX_SAMPLES = 3

#: Peak must rise this far above the baseline, as a fraction of the baseline.
#: THE MOST IMPORTANT REFUSAL HERE. Clean empty glass has almost no contrast, so
#: without this the estimator happily fits sensor noise and reports a confident
#: surface height. This is what forces the operator onto a real feature.
MIN_PROMINENCE_FRAC = 0.30

#: A second local maximum this large relative to the global prominence means the
#: sweep crossed TWO surfaces (a #1.5 coverslip's two faces are ~170 µm apart).
SECOND_PEAK_FRAC = 0.50

#: Any frame with this fraction of ROI pixels clipped invalidates the curve.
#: Clipping removes exactly the gradients the metric measures, and it clips FIRST
#: at best focus — producing a local MINIMUM at true focus flanked by two false
#: maxima. That reads as a clean bimodal curve whose "peak" is ~1 DOF off.
MAX_SATURATED_FRAC = 0.001

#: Spearman |rho| above this across the bracket means focus is outside the range.
MONOTONIC_RHO = 0.90

#: Lateral wander of the tracked feature, as a fraction of the ROI width, beyond
#: which successive samples are scoring different objects.
MAX_DRIFT_FRAC = 0.25

#: Sub-step vertex must lie inside half a step of the discrete argmax.
VERTEX_TOLERANCE_STEPS = 0.5

_FWHM_PER_SIGMA = 2.0 * math.sqrt(2.0 * math.log(2.0))   # 2.3548


@dataclass(frozen=True, kw_only=True)
class FocusSample:
    """One frame of a sweep."""
    z_um: float
    score: float
    laplacian_var: float = 0.0
    tenengrad: float = 0.0
    mean_intensity: float = 0.0
    saturated_frac: float = 0.0
    black_frac: float = 0.0
    roi_rect: tuple[int, int, int, int] | None = None
    #: Identifies the camera settings in force. Any change within one sweep makes
    #: the scores incomparable — the metric is UNNORMALISED.
    settings_fingerprint: str = ""
    matched_conf: float | None = None
    drift_px: float | None = None
    #: What the focus axis actually reported after the move. set_focus_um clamps
    #: silently, so the commanded value must never be assumed.
    focus_readback_um: float | None = None


@dataclass(frozen=True, kw_only=True)
class FocusPeak:
    z_um: float
    sigma_z_um: float
    fwhm_um: float
    prominence: float
    baseline: float
    amplitude: float
    snr: float
    n_samples: int
    step_um: float
    #: Closed-form 3-point vertex, kept for the cross-check.
    vertex_3pt_um: float
    #: Centroid estimate — biased by asymmetry, so a cross-check only.
    centroid_um: float | None = None
    warnings: tuple[str, ...] = field(default_factory=tuple)

    def describe(self) -> str:
        return (f"{self.z_um:.2f} µm ±{self.sigma_z_um:.2f} "
                f"(FWHM {self.fwhm_um:.1f} µm, SNR {self.snr:.0f})")


# ── helpers ───────────────────────────────────────────────────────────────

def _sorted_unique(samples):
    """Sorted by z with duplicate z collapsed to their mean score."""
    buckets: dict[float, list] = {}
    for s in samples:
        buckets.setdefault(round(float(s.z_um), 6), []).append(s)
    out = []
    for z in sorted(buckets):
        group = buckets[z]
        if len(group) == 1:
            out.append(group[0])
        else:
            avg = sum(float(g.score) for g in group) / len(group)
            out.append(type(group[0])(**{**group[0].__dict__, "score": avg}))
    return out


def _percentile(values, q):
    if not values:
        return 0.0
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * float(q)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(xs) - 1)
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def _spearman_abs(zs, fs) -> float:
    """|Spearman rho| — rank correlation, so a monotone curve of any shape."""
    n = len(zs)
    if n < 3:
        return 0.0

    def ranks(v):
        order = sorted(range(n), key=lambda i: v[i])
        r = [0.0] * n
        i = 0
        while i < n:
            j = i
            while j + 1 < n and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r

    rz, rf = ranks(zs), ranks(fs)
    mz = sum(rz) / n
    mf = sum(rf) / n
    num = sum((a - mz) * (b - mf) for a, b in zip(rz, rf))
    dz = math.sqrt(sum((a - mz) ** 2 for a in rz))
    df = math.sqrt(sum((b - mf) ** 2 for b in rf))
    if dz <= 0 or df <= 0:
        return 0.0
    return abs(num / (dz * df))


def _local_maxima(fs):
    return [j for j in range(1, len(fs) - 1)
            if fs[j] > fs[j - 1] and fs[j] >= fs[j + 1]]


def centroid_peak(samples, threshold_frac: float = 0.5) -> float | None:
    """Centre of mass of the curve above a threshold — the cross-check.

    Uses every sample and cannot fail to converge, but it is BIASED BY ASYMMETRY
    and the through-focus response genuinely is asymmetric (spherical aberration
    lengthens the below-focus tail). So it is never the primary estimate; a
    disagreement with the Gaussian fit larger than half a step flags a truncated
    bracket that the edge test alone would miss.
    """
    ss = _sorted_unique(samples)
    if len(ss) < 3:
        return None
    fs = [float(s.score) for s in ss]
    zs = [float(s.z_um) for s in ss]
    base = min(fs)
    peak = max(fs)
    if peak <= base:
        return None
    thr = base + threshold_frac * (peak - base)
    num = den = 0.0
    for z, f in zip(zs, fs):
        w = f - thr
        if w > 0:
            num += w * z
            den += w
    return (num / den) if den > 0 else None


def peak_focus_um(samples, *, dof_um: float | None = None,
                  check_fwhm: bool = True,
                  ) -> tuple[FocusPeak | None, str]:
    """Best-focus Z from a through-focus sweep. ``(peak, reason)``.

    ``dof_um`` enables the FWHM sanity band — the fault detector that catches
    what no other check sees. Omit it only when the objective's optics are
    genuinely unknown.
    """
    ss = _sorted_unique(samples)
    n = len(ss)
    if n < MIN_SAMPLES:
        return (None, f"{INSUFFICIENT_SAMPLES}: {n} usable samples, need "
                      f"{MIN_SAMPLES}")

    # -- settings must be constant: the metric is UNNORMALISED, so scores taken
    #    under different exposure/gain are not comparable, full stop.
    prints = {(s.settings_fingerprint or "") for s in ss}
    if len(prints) > 1:
        return (None, f"{MIXED_EXPOSURE}: the camera settings changed during "
                      f"the sweep ({len(prints)} distinct), so the focus scores "
                      f"are not comparable. Turn auto-exposure off.")

    if any(float(s.saturated_frac or 0.0) > MAX_SATURATED_FRAC for s in ss):
        worst = max(float(s.saturated_frac or 0.0) for s in ss)
        return (None, f"{SATURATED}: {worst * 100:.2f}% of the ROI is clipped. "
                      f"Clipping destroys the gradients the focus metric "
                      f"measures, and it clips first AT focus — the curve can "
                      f"show a dip exactly where the surface is. Lower the "
                      f"exposure or the illumination.")

    # -- lateral wander: a moving ROI scores a different object at each Z.
    if ss[0].roi_rect:
        roi_w = float(ss[0].roi_rect[2] or 0) or 0.0
        if roi_w > 0:
            drifts = [float(s.drift_px) for s in ss if s.drift_px is not None]
            if drifts and max(drifts) > MAX_DRIFT_FRAC * roi_w:
                return (None, f"{DRIFT}: the tracked feature moved "
                              f"{max(drifts):.0f} px across the sweep (ROI is "
                              f"{roi_w:.0f} px). Successive samples scored "
                              f"different things.")

    zs = [float(s.z_um) for s in ss]
    fs = [float(s.score) for s in ss]
    steps = [zs[i + 1] - zs[i] for i in range(n - 1)]
    step = sum(steps) / len(steps) if steps else 0.0
    if step <= 0:
        return (None, f"{INSUFFICIENT_SAMPLES}: the sweep has no Z extent")

    i = max(range(n), key=lambda k: fs[k])

    # -- baseline. compute_focus_score has a nonzero floor even on a fully
    #    blurred frame (sensor noise contributes to both terms); fitting ln(F)
    #    without removing it fits the wrong function.
    baseline = min(fs[0], fs[-1], _percentile(fs, 0.10))
    amplitude = fs[i] - baseline
    if amplitude <= 0:
        return (None, f"{BAD_BASELINE}: the curve never rises above its own "
                      f"baseline")

    prominence = amplitude / baseline if baseline > 0 else float("inf")
    if prominence < MIN_PROMINENCE_FRAC:
        return (None, f"{LOW_PROMINENCE}: the sharpest frame is only "
                      f"{prominence * 100:.0f}% above the blurred baseline "
                      f"(need {MIN_PROMINENCE_FRAC * 100:.0f}%). There is not "
                      f"enough contrast here to focus on — pick a feature with "
                      f"visible structure.")

    # Monotonic is checked BEFORE the edge test. A strictly monotone curve always
    # has its argmax at an edge, so testing the edge first would make MONOTONIC
    # unreachable — and "sharpness rises across the whole sweep, focus is outside
    # this range" is a strictly more informative diagnosis than "the peak is at
    # the end", because it says the range is wrong rather than merely off-centre.
    if _spearman_abs(zs, fs) > MONOTONIC_RHO:
        rising = fs[-1] > fs[0]
        where = "above" if rising else "below"
        return (None, f"{MONOTONIC}: sharpness increases steadily across the "
                      f"whole sweep with no peak — focus is outside this range, "
                      f"{where} it. Move the centre rather than widening.")

    if i == 0 or i == n - 1:
        where = "below" if i == 0 else "above"
        return (None, f"{PEAK_AT_EDGE}: the sharpest frame is at the {where} "
                      f"end of the sweep, so the true focus is probably outside "
                      f"it. Shift the range {where}ward rather than widening it.")

    # -- two surfaces? Only the curve SHAPE can see this. A systematic common to
    #    every site passes every numeric gate downstream.
    others = [j for j in _local_maxima(fs) if abs(j - i) > 1]
    for j in sorted(others, key=lambda k: -fs[k]):
        lo, hi = (j, i) if j < i else (i, j)
        valley = min(fs[lo:hi + 1])
        if (fs[j] - valley) > SECOND_PEAK_FRAC * amplitude:
            sep = abs(zs[j] - zs[i])
            return (None, f"{MULTIMODAL}: two focus peaks {sep:.0f} µm apart. "
                          f"The sweep crossed two surfaces — typically the two "
                          f"faces of the coverslip. Confirm which one the needle "
                          f"touches (the inner well bottom, where cells sit).")

    # -- closed-form 3-point log-parabola vertex: the seed and the cross-check.
    try:
        y0 = math.log(fs[i - 1] - baseline)
        y1 = math.log(fs[i] - baseline)
        y2 = math.log(fs[i + 1] - baseline)
    except ValueError:
        return (None, f"{BAD_BASELINE}: a sample at the peak sits at or below "
                      f"the baseline, so log-domain fitting is impossible")
    d2 = y0 - 2.0 * y1 + y2
    if d2 >= -1e-12:
        return (None, f"{NON_CONCAVE}: the three samples around the maximum do "
                      f"not form a peak, so there is no vertex to interpolate.")
    vertex_3pt = zs[i] + 0.5 * step * (y0 - y2) / d2
    if abs(vertex_3pt - zs[i]) > VERTEX_TOLERANCE_STEPS * step * 1.001:
        return (None, f"{VERTEX_OUTSIDE_BRACKET}: the interpolated peak falls "
                      f"outside the samples that bracket it — the baseline or "
                      f"the noise is wrong. Refusing rather than falling back "
                      f"to the nearest sample.")

    # -- weighted linear least squares over the half-max set. Linear, so it
    #    cannot fail to converge on exactly the pathological curves we refuse.
    half = baseline + 0.5 * amplitude
    idx = [k for k in range(n) if fs[k] >= half]
    z_fit = sigma = fwhm = None
    warnings: list[str] = []
    if len(idx) >= MIN_HALFMAX_SAMPLES:
        import numpy as np
        z0 = zs[i]                              # centre for conditioning
        zc = np.array([zs[k] - z0 for k in idx], dtype=float)
        yv = np.log(np.array([fs[k] - baseline for k in idx], dtype=float))
        w = np.array([fs[k] - baseline for k in idx], dtype=float)
        A = np.column_stack([zc ** 2, zc, np.ones_like(zc)])
        W = np.sqrt(w)
        try:
            sol, *_ = np.linalg.lstsq(A * W[:, None], yv * W, rcond=None)
            a, b, _c = (float(sol[0]), float(sol[1]), float(sol[2]))
            if a < 0:
                z_fit = z0 - b / (2.0 * a)
                sigma = math.sqrt(-1.0 / (2.0 * a))
                fwhm = _FWHM_PER_SIGMA * sigma
                resid = (A @ sol) - yv
                rms = float(np.sqrt(float(np.mean(resid ** 2))))
            else:
                warnings.append("half-max fit was not concave; used the "
                                "3-point vertex")
                rms = 0.0
        except Exception:
            warnings.append("half-max fit failed; used the 3-point vertex")
            rms = 0.0
    else:
        warnings.append(f"only {len(idx)} samples above half maximum; used the "
                        f"3-point vertex")
        rms = 0.0

    if z_fit is None:
        z_fit = vertex_3pt
        sigma = step / math.sqrt(-d2)
        fwhm = _FWHM_PER_SIGMA * sigma
        rms = 0.0

    if abs(z_fit - vertex_3pt) > 0.5 * step:
        warnings.append(f"the half-max fit and the 3-point vertex disagree by "
                        f"{abs(z_fit - vertex_3pt):.1f} µm — the bracket may be "
                        f"truncated on one side")

    # -- FWHM is the operator's "point spread" and the shape fault detector.
    if check_fwhm and dof_um:
        from SupportClasses.ObjectiveOptics import HARD_FWHM_DOF_BAND
        lo, hi = HARD_FWHM_DOF_BAND
        if fwhm < lo * dof_um:
            return (None, f"{FWHM_MISMATCH}: the focus peak is {fwhm:.1f} µm "
                          f"wide but this objective's depth of field is "
                          f"{dof_um:.1f} µm. A response narrower than the depth "
                          f"of field is physically impossible — the sweep step "
                          f"is too coarse, or one bright frame was fitted as a "
                          f"peak.")
        if fwhm > hi * dof_um:
            return (None, f"{FWHM_MISMATCH}: the focus peak is {fwhm:.1f} µm "
                          f"wide against a {dof_um:.1f} µm depth of field. That "
                          f"is not a surface — a thick or layered feature, "
                          f"drifting illumination, or a fixed-pattern-dominated "
                          f"score.")

    snr = (amplitude / rms) if rms > 1e-12 else float("inf")
    # No amount of sub-step interpolation beats the optics, so DOF/10 is a floor.
    sigma_z = (fwhm / (_FWHM_PER_SIGMA * snr)) if snr not in (0, float("inf")) \
        else 0.0
    if dof_um:
        sigma_z = max(sigma_z, dof_um / 10.0)
    else:
        sigma_z = max(sigma_z, step / 10.0)

    cent = centroid_peak(ss)
    if cent is not None and abs(cent - z_fit) > 0.5 * step:
        warnings.append(f"the centroid estimate differs by "
                        f"{abs(cent - z_fit):.1f} µm — the curve is asymmetric "
                        f"or the bracket is truncated")

    return (FocusPeak(z_um=float(z_fit), sigma_z_um=float(sigma_z),
                      fwhm_um=float(fwhm), prominence=float(prominence),
                      baseline=float(baseline), amplitude=float(amplitude),
                      snr=float(snr), n_samples=n, step_um=float(step),
                      vertex_3pt_um=float(vertex_3pt), centroid_um=cent,
                      warnings=tuple(warnings)), "")
