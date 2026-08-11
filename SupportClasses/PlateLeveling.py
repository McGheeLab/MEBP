"""
Plate-bed leveling — site geometry, the plate-to-plate prior, and the solve.

The plate bottom is flat but tilted. A survey focuses on a trained feature at
several XY sites, and the *differences* between those focus heights are the tilt.
The absolute datum comes from elsewhere (a needle touch-off, or the stored
focus↔needle datum) — this module never invents one.

TWO PROPERTIES THIS MODULE EXISTS TO GUARANTEE
----------------------------------------------
**1. The site gate and the fitter agree.** ``sites_gate`` applies the SAME
collinearity constant that :func:`PlateZPlane.from_focal_readings` re-applies
afterwards, so the UI can never let an operator start a twenty-minute survey that
the fitter will refuse at the end of it.

**2. The prior only ever narrows a SEARCH.** A stored per-plate-type prior makes
site 1 fast and sites 2..N nearly instant, but it contributes *nothing* to the
fitted plane. Every run re-measures the tilt from scratch, so a plate that seats
differently is measured correctly rather than assumed to match the last one. That
is what makes "slight plate-to-plate differences" safe rather than silent.

Pure: no Qt, no hardware, no I/O. numpy is imported lazily.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from SupportClasses.PlateZPlane import (
    MIN_TRIANGLE_AREA_MM2, triangle_max_area_mm2,
)


# ── tunables ──────────────────────────────────────────────────────────────

#: Three sites fit a tilt EXACTLY — R² is identically 1.0 and the hold-out is
#: undefined — so three can never be checked. The fourth is what proves the fit.
MIN_SITES = 4

#: Sites must span at least this fraction of the plate footprint's diagonal. A
#: tilt measured over a small patch is extrapolated across everything else.
MIN_SPAN_FRAC = 0.50

#: Two sites closer than this many fields of view measure the same point.
MIN_SEPARATION_FOV = 1.0

#: Default acceptance tolerance (mm). Tighter than PlateZPlane's own 0.05 mm
#: because the operator asked for better than 50 µm across the plate; the wizard
#: reports the ACHIEVED figure so the claim is verified rather than hoped for.
DEFAULT_TOLERANCE_MM = 0.025

#: Extra margin (µm) added to the prior window on top of 3 sigma.
PRIOR_MARGIN_UM = 50.0

#: With no history at all, how wide the first site's search should be (µm).
NO_PRIOR_HALF_RANGE_UM = 1000.0


@dataclass(frozen=True, kw_only=True)
class SiteGeometry:
    count: int
    triangle_max_area_mm2: float
    bbox_um: tuple[float, float, float, float]
    span_mm: float
    min_pair_sep_um: float
    footprint_cover_frac: float | None = None


def sites_geometry(points_xy_um, footprint_um=None) -> SiteGeometry:
    pts = [(float(x), float(y)) for x, y in (points_xy_um or ())]
    if not pts:
        return SiteGeometry(count=0, triangle_max_area_mm2=0.0,
                            bbox_um=(0.0, 0.0, 0.0, 0.0), span_mm=0.0,
                            min_pair_sep_um=0.0)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    bbox = (min(xs), min(ys), max(xs), max(ys))
    span_mm = math.hypot(bbox[2] - bbox[0], bbox[3] - bbox[1]) / 1000.0
    sep = float("inf")
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            sep = min(sep, math.hypot(pts[i][0] - pts[j][0],
                                      pts[i][1] - pts[j][1]))
    if sep == float("inf"):
        sep = 0.0
    cover = None
    if footprint_um:
        try:
            fdiag = math.hypot(float(footprint_um[2]) - float(footprint_um[0]),
                               float(footprint_um[3]) - float(footprint_um[1]))
            cover = (span_mm * 1000.0 / fdiag) if fdiag > 0 else None
        except (TypeError, ValueError, IndexError):
            cover = None
    return SiteGeometry(count=len(pts),
                        triangle_max_area_mm2=triangle_max_area_mm2(pts),
                        bbox_um=bbox, span_mm=span_mm, min_pair_sep_um=sep,
                        footprint_cover_frac=cover)


def sites_gate(points_xy_um, *, footprint_um=None, fov_um=None
               ) -> tuple[bool, str]:
    """Are these sites fit to measure a plane? ``(ok, why)``.

    Uses ``MIN_TRIANGLE_AREA_MM2`` imported from :mod:`PlateZPlane` — not a copy
    — so this gate and the fitter's own refusal cannot drift apart.
    """
    g = sites_geometry(points_xy_um, footprint_um=footprint_um)
    if g.count < MIN_SITES:
        return (False,
                f"Pick at least {MIN_SITES} sites (you have {g.count}). Three "
                f"sites fit a tilt exactly and can never be checked; the fourth "
                f"is what proves the fit.")
    if g.triangle_max_area_mm2 < MIN_TRIANGLE_AREA_MM2:
        return (False,
                f"These sites are nearly collinear (largest triangle "
                f"{g.triangle_max_area_mm2:.1f} mm², need "
                f"{MIN_TRIANGLE_AREA_MM2:.0f} mm²). Points in a line fit a plane "
                f"perfectly and define a garbage tilt — spread them toward "
                f"opposite corners.")
    if g.footprint_cover_frac is not None and \
            g.footprint_cover_frac < MIN_SPAN_FRAC:
        return (False,
                f"The sites span only {g.span_mm:.0f} mm of the plate "
                f"({g.footprint_cover_frac * 100:.0f}% of its diagonal). A tilt "
                f"measured over a small patch gets extrapolated across the rest "
                f"— move sites toward the far corners.")
    if fov_um:
        try:
            fov = float(fov_um[0]) if isinstance(fov_um, (list, tuple)) \
                else float(fov_um)
        except (TypeError, ValueError):
            fov = 0.0
        if fov > 0 and g.min_pair_sep_um < MIN_SEPARATION_FOV * fov:
            return (False,
                    f"Two sites are only {g.min_pair_sep_um:.0f} µm apart — less "
                    f"than one {fov:.0f} µm field of view. They measure the same "
                    f"point and add nothing.")
    return (True, "")


def propose_sites(*, well_positions_um: dict, anchor_xy_um=None, n: int = 5,
                  footprint_um=None) -> list[str]:
    """Farthest-point sampling, SEEDED AT THE ANCHOR well.

    Seeding at the well nearest the taught plate-bottom anchor matters: it makes
    site 0 *be* the anchor, which is what binds the fit to the taught datum and
    lets an anchor-drift check exist at all. The rest maximise spread, which is
    what the collinearity and span gates want.
    """
    items = [(str(k), (float(v[0]), float(v[1])))
             for k, v in (well_positions_um or {}).items()
             if v is not None and len(v) >= 2]
    if not items:
        return []
    n = max(MIN_SITES, min(int(n), len(items)))

    if anchor_xy_um:
        ax, ay = float(anchor_xy_um[0]), float(anchor_xy_um[1])
        start = min(items, key=lambda it: math.hypot(it[1][0] - ax,
                                                     it[1][1] - ay))
    else:
        cx = sum(p[1][0] for p in items) / len(items)
        cy = sum(p[1][1] for p in items) / len(items)
        start = min(items, key=lambda it: math.hypot(it[1][0] - cx,
                                                     it[1][1] - cy))

    chosen = [start]
    remaining = [it for it in items if it[0] != start[0]]
    while len(chosen) < n and remaining:
        best = max(remaining, key=lambda it: min(
            math.hypot(it[1][0] - c[1][0], it[1][1] - c[1][1]) for c in chosen))
        chosen.append(best)
        remaining = [it for it in remaining if it[0] != best[0]]
    return [c[0] for c in chosen]


# ── the plate-to-plate prior ──────────────────────────────────────────────

@dataclass(frozen=True, kw_only=True)
class PlatePrior:
    """What previous plates of this type looked like.

    ``offset_mean_um`` / ``offset_sigma_um`` describe how much the whole focal
    surface shifts from plate to plate (glass thickness, how the plate seats in
    the holder). ``sx``/``sy`` are the last measured tilt — used ONLY to predict
    where to look next, never as a result.
    """
    n_runs: int = 0
    offset_mean_um: float = 0.0
    offset_sigma_um: float = 0.0
    sx_mm_per_mm: float = 0.0
    sy_mm_per_mm: float = 0.0
    ref_xy_um: tuple[float, float] | None = None
    ref_focus_um: float | None = None

    def predict_focus_um(self, x_um: float, y_um: float,
                         run_offset_um: float = 0.0) -> float | None:
        """Where the focus should be at this XY, given this run's own offset."""
        if self.ref_focus_um is None or self.ref_xy_um is None:
            return None
        dx = (float(x_um) - self.ref_xy_um[0]) / 1000.0
        dy = (float(y_um) - self.ref_xy_um[1]) / 1000.0
        return (self.ref_focus_um + self.sx_mm_per_mm * dx * 1000.0
                + self.sy_mm_per_mm * dy * 1000.0 + float(run_offset_um))

    def first_site_half_range_um(self) -> float:
        """Search half-range for site 1, from how much plates actually vary.

        Degrades to the full range with no history, and self-tightens as more
        plates are seen. It never tightens below one margin, because a prior
        built from two plates is not evidence about the third.
        """
        if self.n_runs < 2 or self.ref_focus_um is None:
            return NO_PRIOR_HALF_RANGE_UM
        return max(PRIOR_MARGIN_UM,
                   3.0 * float(self.offset_sigma_um) + PRIOR_MARGIN_UM)


def update_prior(prior: PlatePrior | None, *, run_offset_um: float,
                 sx: float, sy: float, ref_xy_um, ref_focus_um: float
                 ) -> PlatePrior:
    """Fold one accepted run into the running prior (Welford-style mean/σ)."""
    p = prior or PlatePrior()
    n = int(p.n_runs) + 1
    mean = float(p.offset_mean_um)
    delta = float(run_offset_um) - mean
    mean += delta / n
    # Population sigma from the running sum of squares kept implicitly: with
    # only the previous sigma available, recompute conservatively.
    if n == 1:
        sigma = 0.0
    else:
        prev_var = float(p.offset_sigma_um) ** 2 * max(1, p.n_runs)
        var = (prev_var + delta * (float(run_offset_um) - mean)) / n
        sigma = math.sqrt(max(0.0, var))
    return PlatePrior(n_runs=n, offset_mean_um=mean, offset_sigma_um=sigma,
                      sx_mm_per_mm=float(sx), sy_mm_per_mm=float(sy),
                      ref_xy_um=(float(ref_xy_um[0]), float(ref_xy_um[1])),
                      ref_focus_um=float(ref_focus_um))


# ── measurements → a plane ────────────────────────────────────────────────

@dataclass(frozen=True, kw_only=True)
class SiteMeasurement:
    """One surveyed site, in the frames the fitter wants."""
    label: str
    x_stage_um: float
    y_stage_um: float
    focus_um: float | None = None
    focus_sigma_um: float | None = None
    fwhm_um: float | None = None
    prominence: float | None = None
    objective_name: str = ""
    match_conf: float | None = None
    clamped: bool = False
    peak_at_edge: bool = False
    refusal: str = ""
    surface: str = "well_glass_bottom"

    @property
    def usable(self) -> bool:
        return (self.focus_um is not None and not self.refusal
                and not self.clamped and not self.peak_at_edge)


@dataclass(frozen=True, kw_only=True)
class LevelSolution:
    plane: object | None = None
    reason: str = ""
    holdout_um: float | None = None
    residual_max_um: float | None = None
    span_mm: float | None = None
    blockers: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()
    sign_identifiable: bool = True
    holdout_site: str = ""

    @property
    def ok(self) -> bool:
        return self.plane is not None and not self.blockers


def solve(measurements, *, anchor_xy_um, anchor_z_zref_mm: float,
          focal_sign: int = 1, tolerance_mm: float = DEFAULT_TOLERANCE_MM,
          provenance: dict | None = None,
          plate_key: str | None = None) -> LevelSolution:
    """Fit the plate plane from a survey. Never raises; refuses instead."""
    from SupportClasses.PlateZPlane import (
        ZPlanePoint, from_focal_readings, holdout_focal_error_mm)

    ms = list(measurements or ())
    usable = [m for m in ms if m.usable]
    blockers: list[str] = []
    warns: list[str] = []

    for m in ms:
        if m.refusal:
            blockers.append(f"Site {m.label}: {m.refusal}")
        elif m.clamped:
            blockers.append(
                f"Site {m.label} hit the end of the focus range before a peak, "
                f"so its Z is a guess rather than a measurement. Redo it or drop "
                f"it.")
        elif m.peak_at_edge:
            blockers.append(
                f"Site {m.label}'s sharpest frame was at the edge of its sweep. "
                f"Redo it with the range shifted.")

    if len(usable) < MIN_SITES:
        blockers.append(
            f"Only {len(usable)} sites measured successfully; {MIN_SITES} are "
            f"needed so the fit can be checked against a site it never saw.")
        return LevelSolution(reason="too few usable sites",
                             blockers=tuple(blockers))

    pts = [ZPlanePoint(label=m.label, kind="free",
                       x_stage_um=m.x_stage_um, y_stage_um=m.y_stage_um,
                       source="focal_readout", surface=m.surface,
                       focal_raw=m.focus_um, focal_unit="um",
                       focal_mm=float(m.focus_um) / 1000.0,
                       focus_sigma_um=m.focus_sigma_um,
                       focus_score=m.prominence)
           for m in usable]

    anchor = ZPlanePoint(label="anchor", kind="well",
                         x_stage_um=float(anchor_xy_um[0]),
                         y_stage_um=float(anchor_xy_um[1]),
                         z_zref_mm=float(anchor_z_zref_mm),
                         source="needle_touch",
                         surface=usable[0].surface)

    prov = dict(provenance or {})
    if plate_key:
        prov.setdefault("plate_key", plate_key)
    plane, why = from_focal_readings(points=pts, anchor=anchor,
                                     focal_sign=focal_sign, provenance=prov)
    if plane is None:
        blockers.append(why)
        return LevelSolution(reason=why, blockers=tuple(blockers))

    hold = holdout_focal_error_mm(pts, focal_sign)
    g = sites_geometry([(m.x_stage_um, m.y_stage_um) for m in usable])

    tol_um = float(tolerance_mm) * 1000.0
    if hold is not None and hold * 1000.0 > tol_um:
        blockers.append(
            f"Hold-out error {hold * 1000.0:.0f} µm exceeds the {tol_um:.0f} µm "
            f"target — the plane predicts a site it never saw that far wrong, so "
            f"one site is inconsistent with the others.")
    if plane.residual_max_mm * 1000.0 > tol_um:
        blockers.append(
            f"Largest residual {plane.residual_max_mm * 1000.0:.0f} µm exceeds "
            f"the {tol_um:.0f} µm target.")

    # The sign is only PROVABLE when there is enough tilt to distinguish it.
    span_z_um = abs(plane.span_mm(g.bbox_um)) * 1000.0 \
        if hasattr(plane, "span_mm") else 0.0
    sign_ok = span_z_um >= 2.0 * tol_um
    if not sign_ok:
        warns.append(
            f"This plate is nearly level (total variation {span_z_um:.0f} µm), "
            f"so the focus-direction sign cannot be proved by this survey. It "
            f"barely matters here — but the sign is assumed, not verified.")

    # The most demanding hold-out site is the one farthest from the anchor.
    far = max(usable, key=lambda m: math.hypot(m.x_stage_um - anchor.x_stage_um,
                                               m.y_stage_um - anchor.y_stage_um))
    return LevelSolution(
        plane=plane, reason="", holdout_um=(None if hold is None
                                            else hold * 1000.0),
        residual_max_um=plane.residual_max_mm * 1000.0,
        span_mm=g.span_mm, blockers=tuple(blockers), warnings=tuple(warns),
        sign_identifiable=sign_ok, holdout_site=far.label)
