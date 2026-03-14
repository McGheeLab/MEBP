"""
MosaicCalibrator — 2-D point-set registration for well-plate calibration.

Fits a similarity transform (uniform scale + rotation + translation) that
maps predicted well positions to measured stage positions using SVD-based
Procrustes analysis.

Usage:
    cal = MosaicCalibrator()
    cal.add_point(pred_x, pred_y, meas_x, meas_y)   # ≥2 points
    ...
    cal.solve()
    corrected = cal.correct_positions(predicted_dict)
    print(f"RMS error: {cal.rms_error_um:.1f} µm")
"""

from __future__ import annotations

import logging
from typing import Dict, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class MosaicCalibrator:
    """
    Similarity-transform calibrator for well-plate XY positions.

    Computes the best-fit scale, rotation, and translation that maps
    a set of predicted (geometry-derived) well coordinates to a set of
    measured (stage) coordinates, using SVD Procrustes analysis.

    Works with as few as 2 matched point pairs.  With 1 pair only a
    translation can be recovered (no rotation/scale).
    """

    def __init__(self) -> None:
        self._pred: list[tuple[float, float]] = []
        self._meas: list[tuple[float, float]] = []
        self._R: np.ndarray = np.eye(2)
        self._scale: float = 1.0
        self._tx: float = 0.0
        self._ty: float = 0.0
        self._solved: bool = False
        self.rms_error_um: float = 0.0

    # ── Data collection ───────────────────────────────────────────────

    def add_point(
        self,
        pred_x: float,
        pred_y: float,
        meas_x: float,
        meas_y: float,
    ) -> None:
        """Add one matched point pair (predicted vs measured in stage µm)."""
        self._pred.append((float(pred_x), float(pred_y)))
        self._meas.append((float(meas_x), float(meas_y)))

    @property
    def n_points(self) -> int:
        return len(self._pred)

    # ── Solver ────────────────────────────────────────────────────────

    def solve(self) -> None:
        """
        Compute the similarity transform P_meas ≈ s·R·P_pred + t.

        Raises ValueError if fewer than 2 point pairs have been added.
        """
        n = len(self._pred)
        if n < 2:
            raise ValueError(
                f"MosaicCalibrator.solve() requires ≥2 point pairs (have {n})."
            )

        P = np.array(self._pred, dtype=float)   # N×2 predicted
        M = np.array(self._meas,  dtype=float)  # N×2 measured

        # Centroids
        P_c = P.mean(axis=0)
        M_c = M.mean(axis=0)
        Pc = P - P_c
        Mc = M - M_c

        # SVD of the cross-covariance matrix
        H = Pc.T @ Mc          # 2×2
        U, S, Vt = np.linalg.svd(H)

        # Rotation (handle reflection)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Uniform scale: s = Σσ / Σ||Pc||²
        denom = float(np.sum(Pc ** 2))
        scale = float(np.sum(S) / denom) if denom > 0 else 1.0

        # Translation
        t = M_c - scale * (R @ P_c)

        self._R = R
        self._scale = scale
        self._tx = float(t[0])
        self._ty = float(t[1])
        self._solved = True

        # RMS residual
        P_tf = scale * (Pc @ R.T) + M_c
        residuals = np.linalg.norm(P_tf - M, axis=1)
        self.rms_error_um = float(np.sqrt(np.mean(residuals ** 2)))

        logger.debug(
            f"MosaicCalibrator solved: n={n}, scale={scale:.6f}, "
            f"angle={float(np.degrees(np.arctan2(R[1,0], R[0,0]))):.3f}°, "
            f"tx={self._tx:.1f} µm, ty={self._ty:.1f} µm, "
            f"RMS={self.rms_error_um:.1f} µm"
        )

    # ── Application ───────────────────────────────────────────────────

    def correct_positions(
        self,
        predicted_positions: Dict[str, Tuple[float, float]],
    ) -> Dict[str, Tuple[float, float]]:
        """
        Apply the fitted transform to every entry in *predicted_positions*.

        Returns a new dict with the same keys and corrected (x, y) values.
        If :meth:`solve` has not been called the input is returned unchanged.
        """
        if not self._solved:
            return dict(predicted_positions)

        R = self._R
        s = self._scale
        tx, ty = self._tx, self._ty

        result: Dict[str, Tuple[float, float]] = {}
        for name, (x, y) in predicted_positions.items():
            cx = s * (R[0, 0] * x + R[0, 1] * y) + tx
            cy = s * (R[1, 0] * x + R[1, 1] * y) + ty
            result[name] = (float(cx), float(cy))
        return result
