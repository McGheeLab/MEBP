"""
SampleSurface.py — the per-well CRITICAL SAMPLE SURFACE from a mosaic focus
survey (v7.13).

What this is — and is NOT
-------------------------
The fluorescence mosaic's per-tile autofocus measures where the SAMPLE is in
focus: the surface the cells sit on. That is frequently NOT the well bottom —
cells grow on hydrogel pads, coatings, or inserts that hold them a substantial
distance above the glass — and its topology is usually flat but does not have
to be. It is therefore its own artifact, evaluated per well, and it is NEVER
installed as the plate-bottom datum (the plate bottom / whole-plate tilt is
owned by the v7.11 optical leveling wizard).

Models
------
The survey's raw (x_um, y_um, focus_um) samples are always persisted, so any
model can be re-fit later. Three evaluation models are offered:

* ``plane``  — least-squares plane. Robust with few samples; residual reported
  so a domed surface shows up as residual instead of being silently flattened.
* ``linear`` — Delaunay linear interpolation (scipy griddata). Follows real
  topology inside the sample convex hull.
* ``spline`` — smoothed RBF (thin-plate) fit. Smooth topology; tolerant of
  measurement noise.

Confidence: evaluation is "high" only INSIDE both the well boundary and the
sample convex hull — the raster's corner tiles sit outside the circular well
and AF refuses on their empty glass, so the periphery is data-free and any
value there is extrapolation (per the operator: "the edges may be outside the
well, the highest confidence is within the well boundary region"). Outside
the hull the plane fit is used as fallback and flagged "low".

Pure: numpy + (lazily) scipy. No Qt, no hardware, no I/O.
"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP = True
except ImportError:                      # pragma: no cover
    _NP = False

SURFACE_MODELS = ("plane", "linear", "spline")

#: Minimum accepted samples for any surface at all.
MIN_SAMPLES = 4
#: Minimum samples before the interpolating models are offered.
MIN_INTERP_SAMPLES = 8

CONF_HIGH = "high"
CONF_LOW = "low"


class SampleSurfaceModel:
    """Evaluate a per-well sample surface from persisted survey samples."""

    def __init__(self, samples, *, well_center_um, well_radius_um: float,
                 model: str = "plane"):
        """``samples``: iterable of dicts/objects with x_um, y_um, focus_um."""
        if not _NP:
            raise RuntimeError("numpy required")
        pts = []
        for s in samples or ():
            get = s.get if isinstance(s, dict) else (
                lambda k, _s=s: getattr(_s, k, None))
            x, y, f = get("x_um"), get("y_um"), get("focus_um")
            if x is None or y is None or f is None:
                continue
            pts.append((float(x), float(y), float(f)))
        if len(pts) < MIN_SAMPLES:
            raise ValueError(
                f"{len(pts)} usable focus samples; {MIN_SAMPLES} needed for a "
                f"surface")
        self._pts = np.array(pts, dtype=np.float64)
        self._cx = float(well_center_um[0])
        self._cy = float(well_center_um[1])
        self._r = float(well_radius_um)
        model = str(model or "plane").lower()
        if model not in SURFACE_MODELS:
            model = "plane"
        if model != "plane" and len(pts) < MIN_INTERP_SAMPLES:
            logger.info(
                f"SampleSurface: {model} needs ≥{MIN_INTERP_SAMPLES} samples "
                f"(have {len(pts)}) — falling back to plane")
            model = "plane"
        self._model = model
        self._plane = self._fit_plane()
        self._interp = None
        self._hull = None
        if model in ("linear", "spline"):
            self._build_interp()

    # ── fitting ───────────────────────────────────────────────────

    def _fit_plane(self):
        xs, ys, fs = self._pts[:, 0], self._pts[:, 1], self._pts[:, 2]
        x0, y0 = xs.mean(), ys.mean()
        A = np.column_stack([xs - x0, ys - y0, np.ones_like(xs)])
        sol, *_ = np.linalg.lstsq(A, fs, rcond=None)
        a, b, c = float(sol[0]), float(sol[1]), float(sol[2])
        return (a, b, c - a * x0 - b * y0)

    def _build_interp(self):
        try:
            from scipy.spatial import Delaunay, QhullError
        except ImportError:              # pragma: no cover
            logger.info("SampleSurface: scipy unavailable — plane only")
            self._model = "plane"
            return
        xy = self._pts[:, :2]
        try:
            self._hull = Delaunay(xy)
        except (QhullError, Exception):
            # Collinear / degenerate geometry — no hull, no interpolation.
            logger.info("SampleSurface: degenerate sample geometry — plane only")
            self._model = "plane"
            return
        if self._model == "linear":
            from scipy.interpolate import LinearNDInterpolator
            self._interp = LinearNDInterpolator(
                self._hull, self._pts[:, 2])
        else:
            from scipy.interpolate import RBFInterpolator
            # Thin-plate spline with mild smoothing — measurement sigma is of
            # order a DOF, so exact interpolation would chase noise.
            self._interp = RBFInterpolator(
                xy, self._pts[:, 2], kernel="thin_plate_spline", smoothing=1.0)

    # ── evaluation ────────────────────────────────────────────────

    def model(self) -> str:
        return self._model

    def plane(self) -> tuple:
        return tuple(self._plane)

    def in_well(self, x_um: float, y_um: float) -> bool:
        return math.hypot(float(x_um) - self._cx,
                          float(y_um) - self._cy) <= self._r

    def in_hull(self, x_um: float, y_um: float) -> bool:
        if self._hull is None:
            return False
        try:
            return bool(self._hull.find_simplex(
                np.array([[float(x_um), float(y_um)]])) >= 0)
        except Exception:
            return False

    def _plane_at(self, x_um: float, y_um: float) -> float:
        a, b, c = self._plane
        return a * float(x_um) + b * float(y_um) + c

    def evaluate(self, x_um: float, y_um: float) -> "tuple[float, str]":
        """``(focus_um, confidence)`` at a point.

        Confidence is "high" only inside BOTH the well boundary and (for the
        interpolating models) the sample convex hull; everywhere else the
        plane fallback answers with "low" — a value is always produced, but
        the caller must treat "low" as extrapolation.
        """
        inside_well = self.in_well(x_um, y_um)
        if self._model == "plane":
            # The plane extrapolates smoothly; confidence still keys on the
            # sampled region (well boundary as proxy).
            conf = CONF_HIGH if inside_well else CONF_LOW
            return self._plane_at(x_um, y_um), conf
        if inside_well and self.in_hull(x_um, y_um) and self._interp is not None:
            try:
                v = float(self._interp(
                    np.array([[float(x_um), float(y_um)]]))[0])
                if math.isfinite(v):
                    return v, CONF_HIGH
            except Exception:
                pass
        return self._plane_at(x_um, y_um), CONF_LOW

    # ── reporting ─────────────────────────────────────────────────

    def rms_plane_residual_um(self) -> float:
        a, b, c = self._plane
        errs = self._pts[:, 2] - (a * self._pts[:, 0]
                                  + b * self._pts[:, 1] + c)
        return float(np.sqrt(np.mean(errs ** 2)))

    def tilt_mm_per_mm(self) -> "tuple[float, float]":
        """Within-well tilt of the plane component (µm/µm ≡ mm/mm)."""
        a, b, _c = self._plane
        return (float(a), float(b))

    def span_across_well_um(self) -> float:
        """Plane-component focus variation across the well diameter."""
        a, b, _c = self._plane
        return float(math.hypot(a, b) * 2.0 * self._r)

    def n_samples(self) -> int:
        return int(len(self._pts))
