"""
MosaicFocusTracker.py — checkerboard AF scheduling + running surface fit for
mosaic scans (v7.13).

During a fluorescence single-well mosaic the focus drive follows the sample's
CRITICAL SURFACE (where the cells are — often above the well bottom, e.g. on
hydrogel). Sweeping every tile is too slow and photobleaches, so autofocus
micro-sweeps run on a spatially uniform **checkerboard lattice** of in-well
tiles, and EVERY tile still gets its focus set from the running plane fit.

Why a lattice and not every-Nth-in-sequence: the raster is a serpentine, so
"every 4th tile" collapses onto stripes — whole bands of the well would never
be sampled in one axis. The lattice keys on the tile's (col, row) GRID index,
staggering alternate lattice rows by half a cell, so coverage is 2-D uniform
by construction (the operator's requirement: "ensure this hits as a
checkerboard on the mosaic").

Refused sweeps (LOW_PROMINENCE on empty glass, etc.) never enter the fit —
an empty tile must not drag the surface — and accepted samples are outlier-
gated against the current fit before they join it.

Pure: no Qt, no hardware. numpy for the least-squares fit only.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP = True
except ImportError:                      # pragma: no cover
    _NP = False

#: Residual (vs the current fit) beyond which an accepted-looking sample is
#: treated as an outlier, in multiples of the objective DOF.
OUTLIER_DOF_MULT = 6.0

#: Minimum accepted samples before the plane fit replaces the running mean.
MIN_PLANE_SAMPLES = 3


@dataclass(frozen=True, kw_only=True)
class FocusSampleXY:
    """One accepted AF measurement at a tile."""
    x_um: float
    y_um: float
    focus_um: float
    prominence: float = 0.0
    sigma_um: float = 0.0
    tile_index: int = -1

    def to_dict(self) -> dict:
        return {"x_um": self.x_um, "y_um": self.y_um,
                "focus_um": self.focus_um, "prominence": self.prominence,
                "sigma_um": self.sigma_um, "tile_index": self.tile_index}


def grid_indices(positions) -> list:
    """Map raster tile centres to integer (col, row) grid indices.

    The raster generator emits tiles on a regular grid (possibly serpentine-
    ordered); cols/rows are recovered by ranking the distinct (rounded) x and
    y coordinates — the same distinct-coordinate trick the workflow's plan
    summary uses. Returns a list of (col, row) aligned with ``positions``.
    """
    xs = sorted({round(float(x), 1) for x, _y in positions})
    ys = sorted({round(float(y), 1) for _x, y in positions})
    xi = {v: i for i, v in enumerate(xs)}
    yi = {v: i for i, v in enumerate(ys)}
    return [(xi[round(float(x), 1)], yi[round(float(y), 1)])
            for x, y in positions]


class MosaicFocusTracker:
    """Schedules checkerboard AF sweeps + fits the sample surface as it goes."""

    def __init__(self, *, well_center_um, well_radius_um: float,
                 lattice_spacing: int = 2, dof_um: float = 10.0,
                 positions=None):
        self._cx = float(well_center_um[0])
        self._cy = float(well_center_um[1])
        self._r = float(well_radius_um)
        self._a = max(1, int(lattice_spacing))
        self._dof = max(0.1, float(dof_um))
        self._samples: list[FocusSampleXY] = []
        self._n_refused = 0
        self._n_outlier = 0
        self._plane: "tuple[float, float, float] | None" = None
        # Grid indices per tile (col,row), for lattice scheduling.
        self._grid = grid_indices(positions) if positions else []

    # ── scheduling ────────────────────────────────────────────────

    def in_well(self, x_um: float, y_um: float) -> bool:
        return math.hypot(float(x_um) - self._cx,
                          float(y_um) - self._cy) <= self._r

    def should_af(self, tile_index: int, x_um: float, y_um: float) -> bool:
        """Sweep this tile? In-well AND on the staggered checkerboard lattice.

        Lattice: sweep on every ``a``-th grid ROW; within a lattice row, every
        ``a``-th COLUMN, with alternate lattice rows offset by ``a // 2`` (for
        a >= 2) so the sampled tiles form a staggered checkerboard rather than
        aligned columns. a=1 degenerates to every in-well tile.
        """
        if not self.in_well(x_um, y_um):
            return False
        a = self._a
        if a <= 1:
            return True
        if not (0 <= int(tile_index) < len(self._grid)):
            return False
        col, row = self._grid[int(tile_index)]
        if row % a != 0:
            return False
        offset = ((row // a) % 2) * (a // 2)
        return (col + offset) % a == 0

    # ── accumulation ──────────────────────────────────────────────

    def note_refused(self):
        """A sweep ran but the curve was refused (empty glass, etc.)."""
        self._n_refused += 1

    def add(self, sample: FocusSampleXY) -> bool:
        """Accept a measured sample into the fit; False if outlier-rejected.

        The outlier gate compares against the CURRENT fit (when one exists):
        a sample more than OUTLIER_DOF_MULT × DOF from the predicted surface
        is a mis-lock (a floating speck, the coverslip's other face), not
        topology.
        """
        pred = self.predict(sample.x_um, sample.y_um)
        if pred is not None and len(self._samples) >= MIN_PLANE_SAMPLES:
            if abs(float(sample.focus_um) - pred) > OUTLIER_DOF_MULT * self._dof:
                self._n_outlier += 1
                logger.info(
                    "MosaicFocusTracker: outlier rejected at "
                    f"({sample.x_um:.0f},{sample.y_um:.0f}): "
                    f"{sample.focus_um:.1f} µm vs predicted {pred:.1f} µm")
                return False
        self._samples.append(sample)
        self._refit()
        return True

    def _refit(self):
        """Least-squares plane f = a·x + b·y + c over accepted samples."""
        self._plane = None
        if not _NP or len(self._samples) < MIN_PLANE_SAMPLES:
            return
        try:
            xs = np.array([s.x_um for s in self._samples], dtype=np.float64)
            ys = np.array([s.y_um for s in self._samples], dtype=np.float64)
            fs = np.array([s.focus_um for s in self._samples], dtype=np.float64)
            # Centre the coordinates for conditioning (µm magnitudes ~1e5).
            x0, y0 = xs.mean(), ys.mean()
            A = np.column_stack([xs - x0, ys - y0, np.ones_like(xs)])
            sol, *_rest = np.linalg.lstsq(A, fs, rcond=None)
            a, b, c = (float(sol[0]), float(sol[1]), float(sol[2]))
            # Collinear samples leave the fit rank-deficient; lstsq still
            # returns a minimum-norm solution, which is fine for prediction.
            self._plane = (a, b, c - a * x0 - b * y0)
        except Exception as exc:                     # pragma: no cover
            logger.debug(f"MosaicFocusTracker refit failed: {exc}")
            self._plane = None

    # ── prediction ────────────────────────────────────────────────

    def predict(self, x_um: float, y_um: float) -> "float | None":
        """Predicted focus at (x, y): plane fit (n≥3), mean (n≥1), else None."""
        if self._plane is not None:
            a, b, c = self._plane
            return a * float(x_um) + b * float(y_um) + c
        if self._samples:
            return float(sum(s.focus_um for s in self._samples)
                         / len(self._samples))
        return None

    # ── reporting ─────────────────────────────────────────────────

    def samples(self) -> list:
        return list(self._samples)

    def sample_dicts(self) -> list:
        return [s.to_dict() for s in self._samples]

    def rms_residual_um(self) -> "float | None":
        if self._plane is None or not self._samples:
            return None
        a, b, c = self._plane
        errs = [(s.focus_um - (a * s.x_um + b * s.y_um + c))
                for s in self._samples]
        return math.sqrt(sum(e * e for e in errs) / len(errs))

    def summary(self) -> dict:
        xs = [s.x_um for s in self._samples]
        ys = [s.y_um for s in self._samples]
        return {
            "n_accepted": len(self._samples),
            "n_refused": self._n_refused,
            "n_outlier": self._n_outlier,
            "plane": (list(self._plane) if self._plane is not None else None),
            "rms_residual_um": self.rms_residual_um(),
            "span_x_um": (max(xs) - min(xs)) if xs else 0.0,
            "span_y_um": (max(ys) - min(ys)) if ys else 0.0,
            "lattice_spacing": self._a,
            "dof_um": self._dof,
            "well_center_um": (self._cx, self._cy),
            "well_radius_um": self._r,
        }


class PlanePredictor:
    """Replay-mode predictor for later channels: a frozen plane (or mean)."""

    def __init__(self, samples):
        tracker = MosaicFocusTracker(
            well_center_um=(0.0, 0.0), well_radius_um=float("inf"))
        for s in samples or ():
            if isinstance(s, dict):
                s = FocusSampleXY(**{k: s.get(k, 0.0) for k in
                                     ("x_um", "y_um", "focus_um",
                                      "prominence", "sigma_um", "tile_index")})
            tracker._samples.append(s)
        tracker._refit()
        self._tracker = tracker

    def predict(self, x_um: float, y_um: float) -> "float | None":
        return self._tracker.predict(x_um, y_um)

    def n_samples(self) -> int:
        return len(self._tracker.samples())
