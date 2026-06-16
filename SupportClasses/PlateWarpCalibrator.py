"""
PlateWarpCalibrator — interpolating XY warp for well-plate calibration.

Unlike :class:`MosaicCalibrator` (a single global similarity transform —
rotation + *uniform* scale + translation, least-squares over all points),
this calibrator builds a warp from the per-control-point *deltas* so that
the corrected plate **passes exactly through every control point** and
interpolates smoothly between them.

The transform is ``base_affine`` + ``TPS_residual``:

    warp(p) = A·[p, 1]          (linear base — handles rotation, non-uniform
                                 X/Y scale, shear and translation)
            + tps(p)            (thin-plate-spline of the leftover residual,
                                 interpolates it to zero at every control
                                 point so the total is exact there)

Graceful degradation by control-point count:

    0 points → identity
    1 point  → translation only
    2 points → similarity (rotation + uniform scale + translation, exact)
    3 points → full affine (exact when non-collinear)
    ≥4 points→ full affine (least-squares) + TPS residual → exact at all pts

All coordinates are in **absolute stage µm** (same frame the calibration
page pairs against), so the warp is frame-agnostic: it maps predicted
absolute µm → measured absolute µm.

Usage::

    warp = PlateWarpCalibrator()
    warp.add_point(pred_x, pred_y, meas_x, meas_y)   # one per control point
    ...
    warp.solve()
    corrected = warp.correct_positions(predicted_dict)
    print(warp.mode, warp.mean_correction_um, warp.rms_error_um)

Persistence is by raw control-point pairs (:meth:`to_dict` /
:meth:`from_dict`); the warp is re-solved deterministically on load.
"""

from __future__ import annotations

import logging
import math
from typing import Dict, List, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ── Thin-plate-spline helpers ─────────────────────────────────────────

def _pdist2(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise squared Euclidean distances, ``a`` (m×2) vs ``b`` (n×2) → m×n."""
    return np.sum((a[:, None, :] - b[None, :, :]) ** 2, axis=2)


def _tps_kernel(r2: np.ndarray) -> np.ndarray:
    """2-D TPS basis ``U(r) = r²·log(r)`` evaluated from squared radii.

    ``r²·log(r) = r²·½·log(r²)`` — computed in the squared domain to avoid a
    sqrt, with the ``r=0`` singularity pinned to 0.
    """
    out = np.zeros_like(r2)
    mask = r2 > 1e-12
    out[mask] = r2[mask] * 0.5 * np.log(r2[mask])
    return out


class _TPS:
    """Thin-plate spline mapping 2-D control points → 2-D values.

    Fits one TPS per output component (sharing the kernel) that interpolates
    ``values`` exactly at ``src``. Control-point coordinates are centered and
    RMS-normalized before fitting for numerical conditioning (stage µm span
    ~10⁵, so the raw kernel matrix is badly scaled).
    """

    def __init__(self, src: np.ndarray, values: np.ndarray) -> None:
        src = np.asarray(src, dtype=float)
        values = np.asarray(values, dtype=float)
        n = src.shape[0]

        self.mu = src.mean(axis=0)
        rms = float(np.sqrt(np.mean(np.sum((src - self.mu) ** 2, axis=1))))
        self.scale = rms if rms > 1e-9 else 1.0
        u = (src - self.mu) / self.scale          # normalized control pts (n×2)

        k = _tps_kernel(_pdist2(u, u))            # n×n
        p = np.hstack([np.ones((n, 1)), u])       # n×3 (affine block)

        # Solve  [K  P][W]   [values]
        #        [Pᵀ 0][A] = [  0   ]
        size = n + 3
        lhs = np.zeros((size, size))
        lhs[:n, :n] = k
        lhs[:n, n:] = p
        lhs[n:, :n] = p.T
        rhs = np.zeros((size, 2))
        rhs[:n, :] = values
        try:
            sol = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            sol = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

        self._u = u
        self._w = sol[:n, :]                      # n×2 nonlinear weights
        self._a = sol[n:, :]                      # 3×2 affine part of the TPS

    def eval(self, pts: np.ndarray) -> np.ndarray:
        """Evaluate the spline at ``pts`` (m×2) → m×2 residual correction."""
        q = (np.asarray(pts, dtype=float) - self.mu) / self.scale
        k = _tps_kernel(_pdist2(q, self._u))      # m×n
        p = np.hstack([np.ones((q.shape[0], 1)), q])
        return k @ self._w + p @ self._a


# ── Similarity (Procrustes) helper ────────────────────────────────────

def _similarity(p: np.ndarray, m: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
    """Best-fit similarity ``m ≈ s·R·p + t`` via SVD Procrustes.

    Returns ``(R 2×2, scale, t 2,)``. Exact for 2 points.
    """
    p_c = p.mean(axis=0)
    m_c = m.mean(axis=0)
    pc = p - p_c
    mc = m - m_c
    h = pc.T @ mc
    u, s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    denom = float(np.sum(pc ** 2))
    scale = float(np.sum(s) / denom) if denom > 0 else 1.0
    t = m_c - scale * (r @ p_c)
    return r, scale, t


class PlateWarpCalibrator:
    """Interpolating warp calibrator for well-plate XY positions."""

    def __init__(self) -> None:
        self._pred: List[Tuple[float, float]] = []
        self._meas: List[Tuple[float, float]] = []
        # ``out = [x, y, 1] @ _Aaug`` (3×2 augmented affine), identity to start.
        self._Aaug: np.ndarray = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
        self._tps: _TPS | None = None
        self._solved: bool = False

        self.mode: str = "identity"
        self.rms_error_um: float = 0.0       # RMS of post-fit residuals (fit quality)
        self.mean_correction_um: float = 0.0  # RMS magnitude of raw (meas−pred) deltas
        self.max_correction_um: float = 0.0   # largest raw delta magnitude
        # Raw per-control-point deltas (meas − pred), in the input order added.
        self.point_deltas: List[Tuple[float, float]] = []

    # ── Data collection ───────────────────────────────────────────────

    def add_point(self, pred_x: float, pred_y: float,
                  meas_x: float, meas_y: float) -> None:
        """Add one matched pair (predicted vs measured, absolute stage µm)."""
        self._pred.append((float(pred_x), float(pred_y)))
        self._meas.append((float(meas_x), float(meas_y)))

    @property
    def n_points(self) -> int:
        return len(self._pred)

    # ── Solver ────────────────────────────────────────────────────────

    def solve(self) -> None:
        """Fit the warp from the collected control points.

        Chooses the richest model the point count supports (see module
        docstring). Never raises on a degenerate point layout — falls back to
        the affine/least-squares solution and skips the TPS term.
        """
        p = np.array(self._pred, dtype=float)
        m = np.array(self._meas, dtype=float)
        n = p.shape[0]

        # Raw deltas (what the user "was off by" at each control point).
        if n:
            deltas = m - p
            self.point_deltas = [(float(deltas[i, 0]), float(deltas[i, 1]))
                                 for i in range(n)]
            norms = np.linalg.norm(deltas, axis=1)
            self.mean_correction_um = float(np.sqrt(np.mean(norms ** 2)))
            self.max_correction_um = float(np.max(norms))
        else:
            self.point_deltas = []
            self.mean_correction_um = 0.0
            self.max_correction_um = 0.0

        self._tps = None
        if n == 0:
            self._Aaug = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
            self.mode = "identity"
        elif n == 1:
            t = m[0] - p[0]
            self._Aaug = np.array([[1.0, 0.0], [0.0, 1.0], [t[0], t[1]]])
            self.mode = "translation"
        elif n == 2:
            r, s, t = _similarity(p, m)
            self._Aaug = np.array([
                [s * r[0, 0], s * r[1, 0]],
                [s * r[0, 1], s * r[1, 1]],
                [t[0], t[1]],
            ])
            self.mode = "similarity"
        else:
            # ≥3 points → least-squares full affine ([x, y, 1] @ X = m).
            p_aug = np.hstack([p, np.ones((n, 1))])
            x, _res, _rank, _sv = np.linalg.lstsq(p_aug, m, rcond=None)
            self._Aaug = x
            self.mode = "affine"
            if n >= 4:
                residual = m - p_aug @ x
                try:
                    self._tps = _TPS(p, residual)
                    self.mode = "affine_tps"
                except Exception as exc:   # degenerate layout — keep affine
                    logger.warning(
                        "PlateWarp: TPS residual fit failed (%s) — "
                        "falling back to affine-only.", exc)
                    self._tps = None

        self._solved = True

        # Post-fit residual RMS (≈0 for the exact-interpolation modes).
        if n:
            fitted = self._transform_many(p)
            res = np.linalg.norm(fitted - m, axis=1)
            self.rms_error_um = float(np.sqrt(np.mean(res ** 2)))
        else:
            self.rms_error_um = 0.0

        logger.debug(
            "PlateWarp solved: mode=%s n=%d mean_corr=%.1fµm max_corr=%.1fµm "
            "rms_resid=%.2fµm", self.mode, n, self.mean_correction_um,
            self.max_correction_um, self.rms_error_um)

    # ── Application ───────────────────────────────────────────────────

    def _transform_many(self, pts: np.ndarray) -> np.ndarray:
        """Apply the warp to an m×2 array of points → m×2."""
        pts = np.asarray(pts, dtype=float)
        out = np.hstack([pts, np.ones((pts.shape[0], 1))]) @ self._Aaug
        if self._tps is not None:
            out = out + self._tps.eval(pts)
        return out

    def transform(self, x: float, y: float) -> Tuple[float, float]:
        """Warp a single point (absolute stage µm) → (x', y')."""
        out = self._transform_many(np.array([[x, y]]))[0]
        return float(out[0]), float(out[1])

    def correct_positions(
        self, predicted_positions: Dict[str, Tuple[float, float]],
    ) -> Dict[str, Tuple[float, float]]:
        """Apply the warp to every entry in *predicted_positions*.

        Returns a new dict with the same keys. If :meth:`solve` has not run,
        the input is returned unchanged.
        """
        if not self._solved or not predicted_positions:
            return dict(predicted_positions)
        names = list(predicted_positions.keys())
        pts = np.array([predicted_positions[k] for k in names], dtype=float)
        out = self._transform_many(pts)
        return {names[i]: (float(out[i, 0]), float(out[i, 1]))
                for i in range(len(names))}

    # ── Linear-part summary (for display / legacy AffineCalibration) ──

    def similarity_approx(self) -> Tuple[float, float, Tuple[float, float]]:
        """Approximate the linear base as rotation + uniform scale + shift.

        Drops shear / non-uniform scale (via polar/SVD decomposition) and the
        TPS term — for the on-screen "scale / rotation" readout and the
        backward-compatible ``mosaic_affine`` (similarity) persistence only.
        Returns ``(rotation_deg, scale, (tx, ty))``.
        """
        lin = self._Aaug[:2, :].T           # 2×2 map: out = lin @ [x, y]
        u, s, vt = np.linalg.svd(lin)
        r = u @ vt
        if np.linalg.det(r) < 0:
            u[:, -1] *= -1
            r = u @ vt
        rot = math.degrees(math.atan2(r[1, 0], r[0, 0]))
        scale = float(np.mean(s))
        t = self._Aaug[2, :]
        return rot, scale, (float(t[0]), float(t[1]))

    # ── Persistence ───────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Serialize as raw control-point pairs (re-solved on load)."""
        return {
            "version": 1,
            "mode": self.mode,
            "points": [
                [self._pred[i][0], self._pred[i][1],
                 self._meas[i][0], self._meas[i][1]]
                for i in range(len(self._pred))
            ],
            "mean_correction_um": self.mean_correction_um,
            "max_correction_um": self.max_correction_um,
            "rms_error_um": self.rms_error_um,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PlateWarpCalibrator":
        """Rebuild and re-solve a warp from :meth:`to_dict` output."""
        warp = cls()
        for row in data.get("points", []):
            if len(row) >= 4:
                warp.add_point(row[0], row[1], row[2], row[3])
        if warp.n_points:
            warp.solve()
        return warp
