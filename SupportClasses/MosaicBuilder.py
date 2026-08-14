"""
MosaicBuilder — Full-plate mosaic stitching and affine calibration.

Collects camera frames captured during a serpentine raster scan,
stitches them into a seamless composite using industry-standard
microscopy techniques, and fits an affine transform between
predicted and detected well centres.

v7.3.1 — Phase 2 (rewrite)

Stitching pipeline:
  1. Acquire tiles in serpentine raster with configurable overlap (default 10%)
  2. Pairwise registration via phase cross-correlation on overlap regions
  3. Global least-squares optimization of all tile positions
  4. Linear feathered blending in overlap zones
  5. Progressive composite update (call stitch_incremental after each frame)

Usage::

    builder = MosaicBuilder(frame_size_px=(916, 686), micron_per_pixel=3.34)
    positions = builder.generate_raster_positions(bounds, overlap=0.10)
    for i, (x, y) in enumerate(positions):
        stage.move_to(x, y)
        frame = camera.capture_fresh_frame()
        builder.add_raster_frame(frame, x, y, index=i)
        composite = builder.stitch_incremental()   # updated composite
    result = builder.build(predicted_positions)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

try:
    from skimage.registration import phase_cross_correlation
    SKIMAGE_AVAILABLE = True
except ImportError:
    phase_cross_correlation = None
    SKIMAGE_AVAILABLE = False

# v7.5.x: minimum grayscale std-dev (0–255) for an overlap to be considered
# "textured" enough to register. Featureless tiles (between wells, or a flat
# in-well region with no edge) fall below this and are EXCLUDED from the global
# alignment estimate — they're still placed by the (trusted) stage + given the
# global shift like every other tile.
_REGISTER_MIN_STD = 6.0


# ═══════════════════════════════════════════════════════════════════
# Data Structures
# ═══════════════════════════════════════════════════════════════════

@dataclass
class FrameRecord:
    """A single captured frame with its metadata."""
    well_name: str
    frame: np.ndarray                           # BGR image
    stage_x_um: float                           # Absolute stage X (µm)
    stage_y_um: float                           # Absolute stage Y (µm)
    detected_offset_um: tuple[float, float] | None = None  # (dx, dy) from frame center
    confidence: float = 0.0
    # Refined position after registration (pixels on composite canvas)
    refined_x_px: float | None = None
    refined_y_px: float | None = None


@dataclass
class AffineCalibration:
    """Result of affine fitting between predicted and detected positions.

    The transform maps predicted → corrected coordinates:
        corrected = R @ (predicted - center) + center + translation

    where R is a rotation+scale matrix.
    """
    rotation_deg: float = 0.0       # Plate rotation (degrees, CCW positive)
    scale: float = 1.0              # Uniform scale factor
    translation_um: tuple[float, float] = (0.0, 0.0)  # (tx, ty) shift in µm
    center_um: tuple[float, float] = (0.0, 0.0)       # Rotation center (µm)
    num_points: int = 0             # Points used in fit
    residual_um: float = 0.0        # RMS residual after fit (µm)

    def correct_position(self, x_um: float, y_um: float) -> tuple[float, float]:
        """Apply affine correction to a single predicted position."""
        rad = math.radians(self.rotation_deg)
        cos_r = math.cos(rad) * self.scale
        sin_r = math.sin(rad) * self.scale
        cx, cy = self.center_um
        tx, ty = self.translation_um
        # Translate to center, rotate+scale, translate back + shift
        dx = x_um - cx
        dy = y_um - cy
        rx = cos_r * dx - sin_r * dy + cx + tx
        ry = sin_r * dx + cos_r * dy + cy + ty
        return (rx, ry)

    def correct_positions(
        self, predicted: dict[str, tuple[float, float]]
    ) -> dict[str, tuple[float, float]]:
        """Apply affine correction to all predicted positions."""
        return {
            name: self.correct_position(x, y)
            for name, (x, y) in predicted.items()
        }

    @property
    def is_identity(self) -> bool:
        """True if this calibration is essentially a no-op."""
        return (abs(self.rotation_deg) < 0.01
                and abs(self.scale - 1.0) < 0.001
                and abs(self.translation_um[0]) < 1.0
                and abs(self.translation_um[1]) < 1.0)


@dataclass
class MosaicResult:
    """Complete result from a mosaic scan."""
    mosaic_image: np.ndarray | None = None      # Composite stitched image (BGR)
    calibration: AffineCalibration = field(default_factory=AffineCalibration)
    frames_captured: int = 0
    frames_detected: int = 0                    # Frames where well was detected
    detected_positions: dict[str, tuple[float, float]] = field(default_factory=dict)

    def correct_positions(
        self, predicted: dict[str, tuple[float, float]]
    ) -> dict[str, tuple[float, float]]:
        """Convenience: apply calibration to predicted positions."""
        return self.calibration.correct_positions(predicted)


# ═══════════════════════════════════════════════════════════════════
# Stitching helpers
# ═══════════════════════════════════════════════════════════════════

def _phase_correlate_overlap(
    tile_a: np.ndarray,
    tile_b: np.ndarray,
) -> tuple[float, float, float]:
    """Compute sub-pixel translation between two overlap regions.

    Uses phase cross-correlation (frequency domain) with Hanning window
    for robust, sub-pixel registration.

    Args:
        tile_a: Overlap region from tile A (grayscale uint8).
        tile_b: Overlap region from tile B (grayscale uint8).

    Returns:
        (dy, dx, confidence) — sub-pixel shift and peak confidence.
    """
    if not SKIMAGE_AVAILABLE:
        return 0.0, 0.0, 0.0

    # Apply Hanning window to reduce edge artifacts
    h, w = tile_a.shape[:2]
    if h < 4 or w < 4:
        return 0.0, 0.0, 0.0

    win_y = np.hanning(h)
    win_x = np.hanning(w)
    window = np.outer(win_y, win_x)

    a = tile_a.astype(np.float64) * window
    b = tile_b.astype(np.float64) * window

    # Phase cross-correlation with sub-pixel refinement
    shift, error, phase_diff = phase_cross_correlation(
        a, b, upsample_factor=10, normalization="phase"
    )

    # Confidence: inverse of the error (higher = better match)
    # error from phase_cross_correlation is already a quality metric
    confidence = max(0.0, 1.0 - abs(error)) if error is not None else 0.5

    return float(shift[0]), float(shift[1]), confidence


def _register_overlap_cv2(
    a_gray: np.ndarray,
    b_gray: np.ndarray,
) -> "tuple[float, float, float] | None":
    """Sub-pixel shift to ADD to tile B's position so its overlap aligns onto
    tile A's, plus a [0, 1] confidence (phase-correlation peak).

    v7.5.x: the stronger pairwise estimator behind ``optimize_registration``.
    Improvements over ``_phase_correlate_overlap``:
      * CLAHE-normalises both crops first, so illumination gradients /
        vignetting between tiles don't bias the FFT peak (a real failure mode
        when the stage moves under uneven lighting);
      * uses OpenCV ``phaseCorrelate``'s actual cross-power response peak as the
        confidence (a true match-sharpness metric, unlike skimage's
        ``1 - |error|``), so the weighted global solve trusts good overlaps and
        ignores ambiguous ones.
    Returns None when either crop is too small or too flat to register. cv2 is a
    hard dependency of the builder, so this always runs (skimage may be absent).
    """
    if cv2 is None or a_gray is None or b_gray is None:
        return None
    if a_gray.shape != b_gray.shape:
        return None
    h, w = a_gray.shape[:2]
    if h < 12 or w < 12:
        return None
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        a = clahe.apply(a_gray)
        b = clahe.apply(b_gray)
    except Exception:
        a, b = a_gray, b_gray
    af = a.astype(np.float64)
    bf = b.astype(np.float64)
    # A flat overlap (uniform colour) can't be registered — phase correlation on
    # it returns noise. Skip so it neither fails nor injects a bogus shift.
    if af.std() < _REGISTER_MIN_STD or bf.std() < _REGISTER_MIN_STD:
        return None
    try:
        win = cv2.createHanningWindow((w, h), cv2.CV_64F)
        (dx, dy), resp = cv2.phaseCorrelate(af, bf, win)
    except Exception:
        return None
    conf = float(max(0.0, min(1.0, resp)))
    # cv2.phaseCorrelate(a, b) returns B's displacement w.r.t. A; the correction
    # to ADD to B's position to bring its content back onto A is the NEGATIVE of
    # that. (Sign locked by test ``test_optimize_registration_corrects_injected_error``.)
    return (-float(dx), -float(dy), conf)


def _register_overlap_fourier_mellin(
    a_gray: np.ndarray,
    b_gray: np.ndarray,
) -> "tuple[float, float, float, float, float] | None":
    """Recover ``(dx, dy, rotation_deg, scale, conf)`` to align B onto A using
    the **Fourier-Mellin transform** — a stronger registration than plain
    translation-only phase correlation (operator: "the registration to detect a
    frame's translation is very bad … explore better methods, e.g. Fourier-
    Mellin").

    The FFT magnitude is translation-INVARIANT, so resampling it into log-polar
    coordinates turns a **rotation** into a row shift and a **scale** into a
    column shift, which a phase correlation recovers. B is then de-rotated /
    de-scaled and phase-correlated with A for the residual **translation**. This
    tolerates rotation + modest scale drift (and, with the CLAHE pre-pass,
    illumination gradients) — exactly the cases where the plain estimator loses
    lock. For adjacent mosaic tiles (same camera → rotation≈0, scale≈1) it
    reduces to a robust translation estimate.

    ``(dx, dy)`` is the correction to ADD to B's position (same convention as
    ``_register_overlap_cv2``). Returns None when the crops are too small/flat.
    Signs locked by ``test_fourier_mellin_recovers_rotation_scale``.
    """
    if cv2 is None or a_gray is None or b_gray is None:
        return None
    if a_gray.shape != b_gray.shape:
        return None
    h, w = a_gray.shape[:2]
    if h < 32 or w < 32:
        return None
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        A = clahe.apply(a_gray)
        B = clahe.apply(b_gray)
    except Exception:
        A, B = a_gray, b_gray
    Af = A.astype(np.float64)
    Bf = B.astype(np.float64)
    if Af.std() < _REGISTER_MIN_STD or Bf.std() < _REGISTER_MIN_STD:
        return None
    try:
        win = cv2.createHanningWindow((w, h), cv2.CV_64F)

        def _logmag(x):
            f = np.fft.fftshift(np.fft.fft2(x * win))
            return np.log1p(np.abs(f))

        ma, mb = _logmag(Af), _logmag(Bf)
        center = (w / 2.0, h / 2.0)
        maxr = float(min(center))
        if maxr < 8:
            return None
        flags = cv2.INTER_LINEAR + cv2.WARP_POLAR_LOG
        lpa = cv2.warpPolar(ma, (w, h), center, maxr, flags).astype(np.float64)
        lpb = cv2.warpPolar(mb, (w, h), center, maxr, flags).astype(np.float64)
        # Log-polar phase correlation: angle → rows (y), log-radius → cols (x).
        (d_col, d_row), rs_resp = cv2.phaseCorrelate(lpa, lpb, win)
        # rows span [0, 360°) over h; scale s shifts col by (w/ln(maxr))·ln(s).
        rotation_deg = ((d_row * 360.0 / float(h) + 180.0) % 360.0) - 180.0
        log_base = math.log(maxr) if maxr > 1.0 else 1.0
        scale = math.exp(d_col * log_base / float(w))
        if not (0.5 < scale < 2.0):        # implausible for a mosaic → ignore
            scale = 1.0
        # De-rotate + de-scale B toward A, then translation-correlate.
        m_aff = cv2.getRotationMatrix2D(center, rotation_deg, scale)
        b_corr = cv2.warpAffine(Bf, m_aff, (w, h), flags=cv2.INTER_LINEAR)
        (dx, dy), t_resp = cv2.phaseCorrelate(Af, b_corr * win, win)
    except Exception:
        return None
    conf = float(max(0.0, min(1.0, t_resp)))
    return (-float(dx), -float(dy), float(rotation_deg), float(scale), conf)


def _build_neighbor_graph(
    n_tiles: int,
    grid_cols: int,
    grid_rows: int,
) -> list[tuple[int, int, str]]:
    """Build adjacency graph for a serpentine raster grid.

    Returns list of (tile_i, tile_j, direction) edges where direction
    is 'h' (horizontal neighbor) or 'v' (vertical neighbor).
    """
    edges = []
    for row in range(grid_rows):
        for col in range(grid_cols):
            idx = row * grid_cols + col
            if idx >= n_tiles:
                break
            # Serpentine: even rows L→R, odd rows R→L
            if row % 2 == 0:
                actual_col = col
            else:
                actual_col = grid_cols - 1 - col
            # Horizontal neighbor (next in same row)
            if col < grid_cols - 1:
                neighbor_idx = idx + 1
                if neighbor_idx < n_tiles:
                    edges.append((idx, neighbor_idx, 'h'))
            # Vertical neighbor (same column position in next row)
            if row < grid_rows - 1:
                # Find tile in next row at same column position
                next_row_start = (row + 1) * grid_cols
                if (row + 1) % 2 == 0:
                    # Next row is L→R, column `actual_col` is at position actual_col
                    v_neighbor = next_row_start + actual_col
                else:
                    # Next row is R→L, column `actual_col` is at position (cols-1-actual_col)
                    v_neighbor = next_row_start + (grid_cols - 1 - actual_col)
                if v_neighbor < n_tiles:
                    edges.append((idx, v_neighbor, 'v'))
    return edges


def _global_optimize_positions(
    n_tiles: int,
    nominal_positions_px: np.ndarray,
    edges: list[tuple[int, int, str]],
    measured_offsets: list[tuple[float, float]],
    confidences: list[float],
    min_confidence: float = 0.1,
) -> np.ndarray:
    """Globally optimize tile positions using least-squares.

    Solves for tile positions that minimize the weighted sum of squared
    differences between measured pairwise offsets and the implied offsets
    from the solved positions.

    minimize  Σ  w_ij * || (pos_j - pos_i) - measured_offset_ij ||²

    The first tile is anchored at its nominal position.

    Args:
        n_tiles: Number of tiles.
        nominal_positions_px: (N, 2) array of nominal [x, y] positions in pixels.
        edges: List of (i, j, direction) adjacency pairs.
        measured_offsets: List of (dx, dy) measured pixel offsets for each edge.
        confidences: Confidence weight for each edge.
        min_confidence: Minimum confidence to include an edge.

    Returns:
        (N, 2) array of optimized [x, y] positions in pixels.
    """
    if n_tiles <= 1:
        return nominal_positions_px.copy()

    # Filter edges by confidence
    valid_edges = []
    valid_offsets = []
    valid_weights = []
    for k, (i, j, d) in enumerate(edges):
        if confidences[k] >= min_confidence:
            valid_edges.append((i, j))
            valid_offsets.append(measured_offsets[k])
            valid_weights.append(confidences[k])

    if not valid_edges:
        logger.warning("No valid edges for global optimization; using nominal positions")
        return nominal_positions_px.copy()

    # Build overdetermined linear system: A @ pos = b
    # For each edge (i, j): pos_j - pos_i = offset_ij
    # → -pos_i + pos_j = offset_ij
    # Solve for x and y independently
    n_edges = len(valid_edges)

    # We anchor tile 0 to its nominal position
    # For the remaining tiles, solve via least-squares
    # System: for each edge (i,j) and each axis:
    #   w * (pos[j] - pos[i]) = w * measured_offset

    A = np.zeros((n_edges + 1, n_tiles))
    bx = np.zeros(n_edges + 1)
    by = np.zeros(n_edges + 1)
    w_diag = np.zeros(n_edges + 1)

    for k, (i, j) in enumerate(valid_edges):
        w = valid_weights[k]
        A[k, i] = -w
        A[k, j] = w
        bx[k] = w * valid_offsets[k][0]
        by[k] = w * valid_offsets[k][1]
        w_diag[k] = w

    # Anchor constraint: tile 0 at nominal position (high weight)
    anchor_w = 10.0 * max(valid_weights) if valid_weights else 1.0
    A[n_edges, 0] = anchor_w
    bx[n_edges] = anchor_w * nominal_positions_px[0, 0]
    by[n_edges] = anchor_w * nominal_positions_px[0, 1]

    # Solve
    opt_x, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
    opt_y, _, _, _ = np.linalg.lstsq(A, by, rcond=None)

    result = np.column_stack([opt_x, opt_y])
    return result


def _compute_feather_weights(tile_h: int, tile_w: int, margin_px: int) -> np.ndarray:
    """Create a 2D feathering weight map for linear blending.

    Uses separable 1D ramps multiplied together. Each ramp goes from
    0 at the tile edge to 1 at `margin_px` inward, linearly. The
    product of the horizontal and vertical ramps gives the 2D weight.

    This ensures that in overlap zones, the sum of weights from
    adjacent tiles equals 1.0 along each axis. At corners where
    four tiles meet, the weights are lower but still sum correctly
    because each axis contribution sums independently.

    Returns:
        float32 array of shape (tile_h, tile_w) with values in [0, 1].
    """
    if margin_px < 1:
        return np.ones((tile_h, tile_w), dtype=np.float32)

    # Build 1D ramp for horizontal axis
    ramp_x = np.ones(tile_w, dtype=np.float32)
    m = min(margin_px, tile_w // 2)
    for i in range(m):
        alpha = (i + 0.5) / margin_px  # half-pixel offset for symmetry
        ramp_x[i] = alpha
        ramp_x[tile_w - 1 - i] = alpha

    # Build 1D ramp for vertical axis
    ramp_y = np.ones(tile_h, dtype=np.float32)
    m = min(margin_px, tile_h // 2)
    for i in range(m):
        alpha = (i + 0.5) / margin_px
        ramp_y[i] = alpha
        ramp_y[tile_h - 1 - i] = alpha

    # 2D weight = product of separable ramps
    weights = np.outer(ramp_y, ramp_x)
    return weights


# ═══════════════════════════════════════════════════════════════════
# MosaicBuilder
# ═══════════════════════════════════════════════════════════════════

class MosaicBuilder:
    """Collects frames during plate scan, stitches mosaic, fits affine transform.

    Uses an industry-standard microscopy stitching pipeline:
    - Phase cross-correlation for pairwise tile registration
    - Global least-squares optimization of tile positions
    - Linear feathered blending in overlap zones

    Args:
        frame_size_px: (width, height) of each camera frame in pixels.
        micron_per_pixel: Camera scale factor (µm per pixel).
        overlap: Expected fractional overlap between tiles (0.0–0.5).
        target_mosaic_px: Target size (longest edge) for the mosaic image.
    """

    def __init__(
        self,
        frame_size_px: tuple[int, int] = (916, 686),
        micron_per_pixel: float = 3.34,
        overlap: float = 0.10,
        target_mosaic_px: int = 2000,
        register: bool = False,
        max_shift_um: float = 0.0,
        register_mode: str = "global",
        initial_shift_um: tuple[float, float] = (0.0, 0.0),
        retain_frames: bool = True,
        frame_rotation_deg: float = 0.0,
        frame_mirrored: bool = False,
        frame_flip_y: bool = False,
        retain_for_reorient: bool = False,
        registration_method: str = "fourier_mellin",
        tile_center_offset_um: tuple[float, float] = (0.0, 0.0),
    ):
        # v7.5.x: ``frame_rotation_deg`` / ``frame_mirrored`` — the camera's
        # calibrated orientation vs the stage axes. Each tile is oriented
        # (mirror then ``R(θ)``) into the STAGE frame before it is placed, so the
        # finished composite's pixel axes equal the stage axes and a click on the
        # mosaic back-projects to the correct stage XY (the raw ``extent+px/scale``
        # map every consumer uses becomes geometrically correct). A mirror
        # reverses handedness — rotation alone can't fix it. Defaults (0°, False)
        # → the tile is placed unchanged (byte-identical to legacy). See
        # ``_orient_tile``. Frames reach the builder RAW (orientation is applied
        # per consumer), so the builder owns the mosaic orientation.
        #   ``retain_for_reorient`` keeps the small canvas-res *pre-orient* tiles
        # so ``set_frame_orientation`` + ``reblend_reoriented`` can re-render the
        # whole mosaic with a NEW orientation (the interactive fix-orientation
        # tool) without re-scanning.
        self._frame_rotation_deg = float(frame_rotation_deg or 0.0)
        self._frame_mirrored = bool(frame_mirrored)   # flip X
        self._frame_flip_y = bool(frame_flip_y)        # flip Y (vertical)
        self._retain_for_reorient = bool(retain_for_reorient)
        # Each: (resized_raw_bgr, px, py, tile_w, tile_h) at canvas resolution.
        self._reorient_tiles: list = []
        # v7.16: STAGE-frame displacement of a tile's centre from the stage
        # position it was captured at. Non-zero only when the camera's crop has
        # been moved off-centre (see CameraCrop.center_offset_px): the delivered
        # frame's middle then shows a point that is NOT where the stage is
        # pointing, so every tile must be placed there instead. Added at the two
        # points a stage position enters the builder, which is what lets every
        # caller stay unchanged. (0, 0) → byte-identical placement.
        try:
            self._tile_center_offset_um = (
                float(tile_center_offset_um[0]),
                float(tile_center_offset_um[1]))
        except (TypeError, ValueError, IndexError):
            self._tile_center_offset_um = (0.0, 0.0)
        # v7.16: intensity regularization state. None = no correction, and the
        # blend is then byte-identical to before regularize_intensity() ran.
        self._flat_field = None
        self._tile_gains = None
        # v7.5.x: ``retain_frames`` — when False, each tile's raw frame is freed
        # immediately after it is blended into the incremental composite
        # (``stitch_incremental``). A long full-plate scan (hundreds of tiles ×
        # ~2 MB/frame) otherwise accumulates ~½ GB of dead image data in
        # ``_records`` → swap thrash → seconds-per-tile slowdown. Set False ONLY
        # for scans that consume the live ``.composite`` (display cache) and
        # never call ``build_mosaic`` / ``tile_images_px`` / ``_ensure_composite``
        # (which re-blend the raw frames). Default True preserves those paths.
        self._retain_frames = bool(retain_frames)
        self._frame_size_px = frame_size_px
        self._um_per_px = micron_per_pixel
        self._overlap = overlap
        self._target_mosaic_px = target_mosaic_px
        # v7.5.x: GLOBAL registration. The stage is accurate, so relative tile
        # placement (by stage coordinates) is trusted and tiles are NEVER
        # shifted individually (a single bad match must not move one image).
        # When True, the per-overlap misalignment is *measured* during stitching
        # and aggregated into ONE robust (median) shift applied uniformly to the
        # whole mosaic — correcting a systematic stage↔image offset only.
        # max_shift_um bounds that global shift (0 → auto = 20% of the FOV).
        #
        # register_mode = "global" (default — one shift for all tiles, the
        # right choice for an accurate stage) | "per_tile" (each tile nudged by
        # its own bounded measured shift). "per_tile" is RETAINED for future
        # applications (e.g. a drifting/open-loop stage) but is NOT used by the
        # plate mosaic scan.
        self._register = bool(register)
        # v7.5.x: pairwise registration method for optimize_registration:
        # "fourier_mellin" (rotation+scale+translation, most robust) |
        # "phase" (translation-only phase correlation) | "off" (manual/stage-
        # only — no auto-registration; the operator aligns by hand).
        self._registration_method = str(registration_method or "fourier_mellin")
        self._max_shift_um = float(max_shift_um)
        self._register_mode = (
            "per_tile" if str(register_mode) == "per_tile" else "global")
        # Per-overlap measurements (dx_px, dy_px) + the finalized global shift.
        # The global shift is pre-seeded with any learned correction
        # (initial_shift_um, from a prior calibration for this camera+objective)
        # so the mosaic is registered from the first tile; finalize replaces it
        # with the freshly-measured median when measurements were collected.
        self._measured_shifts: list[tuple[float, float]] = []
        try:
            self._global_shift_um: tuple[float, float] = (
                float(initial_shift_um[0]), float(initial_shift_um[1]))
        except Exception:
            self._global_shift_um = (0.0, 0.0)
        self._records: list[FrameRecord] = []

        # Grid shape (set during raster generation)
        self._grid_cols: int = 0
        self._grid_rows: int = 0

        # Composite state for incremental stitching
        self._composite: np.ndarray | None = None        # float64 accumulator
        self._weight_sum: np.ndarray | None = None       # float64 weight accumulator
        self._display_cache: np.ndarray | None = None    # uint8 cached normalized output
        self._mosaic_scale: float = 1.0  # µm → mosaic pixels
        self._canvas_origin_um: tuple[float, float] = (0.0, 0.0)
        self._feather_weights: np.ndarray | None = None

    @property
    def frame_count(self) -> int:
        return len(self._records)

    @property
    def canvas_extent_um(self) -> tuple[float, float, float, float] | None:
        """World extent of the composite canvas (min_x, min_y, max_x, max_y) in µm.

        Includes half-FOV padding beyond the scan bounds on each side.
        Returns None if canvas not yet initialized. v7.5.x: shifted by the
        finalized global alignment offset (0 until ``finalize_global_shift``
        runs), so detection + overlay get the single systematic correction
        without the composite pixels being re-placed.
        """
        ext = getattr(self, '_canvas_extent_um', None)
        if ext is None:
            return None
        gx, gy = getattr(self, '_global_shift_um', (0.0, 0.0))
        if gx or gy:
            return (ext[0] + gx, ext[1] + gy, ext[2] + gx, ext[3] + gy)
        return ext

    @property
    def composite(self) -> np.ndarray | None:
        """Current composite image (may be partially built during scan).

        Returns cached display image. Updated incrementally by _blend_tile_to_composite.
        """
        return self._display_cache

    def tile_rects_px(self) -> list[tuple[float, float, float, float]]:
        """Footprint of every captured tile on the composite canvas, as
        ``(x, y, w, h)`` in mosaic pixels — the stage-commanded placement (same
        math as ``_blend_tile_to_composite``). Used to outline each image taken
        in the manual-registration mosaic view. Empty until the canvas exists.
        """
        origin = getattr(self, "_canvas_origin_um", None)
        scale = getattr(self, "_mosaic_scale", None)
        if origin is None or not scale:
            return []
        fov_w_um, fov_h_um = self._oriented_fov_um()
        ox, oy = origin
        w = fov_w_um * scale
        h = fov_h_um * scale
        rects: list[tuple[float, float, float, float]] = []
        for rec in self._records:
            left = (rec.stage_x_um - fov_w_um / 2.0 - ox) * scale
            top = (rec.stage_y_um - fov_h_um / 2.0 - oy) * scale
            rects.append((left, top, w, h))
        return rects

    def tile_images_px(self) -> list:
        """Each captured tile as ``(frame_bgr, left, top, w, h)`` — the raw frame
        plus its footprint on the composite canvas (mosaic px). Lets a viewer
        render tiles INDIVIDUALLY (e.g. to adjust the inter-tile spacing and align
        features in the overlaps). Same placement as ``tile_rects_px``. Empty until
        the canvas exists.
        """
        origin = getattr(self, "_canvas_origin_um", None)
        scale = getattr(self, "_mosaic_scale", None)
        if origin is None or not scale:
            return []
        fov_w_um, fov_h_um = self._oriented_fov_um()
        ox, oy = origin
        w = max(1, int(fov_w_um * scale))
        h = max(1, int(fov_h_um * scale))
        out: list = []
        for rec in self._records:
            if rec.frame is None:    # freed in frame-light mode
                continue
            left = (rec.stage_x_um - fov_w_um / 2.0 - ox) * scale
            top = (rec.stage_y_um - fov_h_um / 2.0 - oy) * scale
            # v7.5.x: orient each tile (calibrated mirror + rotation) so a
            # per-tile registration view shows what the mosaic will look like.
            # No-op at (0°, unmirrored). Re-call after set_frame_orientation to
            # re-render for the interactive fix-orientation tool.
            out.append((self._orient_tile(rec.frame), left, top, w, h))
        return out

    def _normalize_full(self):
        """Normalize the entire composite accumulator into the display cache."""
        if self._composite is None or self._weight_sum is None:
            return
        ch, cw = self._composite.shape[:2]
        self._display_cache = np.zeros((ch, cw, 3), dtype=np.uint8)
        mask = self._weight_sum > 0
        for c in range(3):
            channel = self._composite[:, :, c].copy()
            channel[mask] = (channel[mask] / self._weight_sum[mask]).clip(0, 255)
            self._display_cache[:, :, c] = channel.astype(np.uint8)

    def reset(self):
        """Clear all collected frames and composite state."""
        self._records.clear()
        self._composite = None
        self._weight_sum = None
        self._display_cache = None
        self._feather_weights = None

    # ── Raster grid generation ────────────────────────────────────

    def generate_raster_positions(
        self,
        bounds_um: tuple[float, float, float, float],
        overlap: float = 0.1,
        step_x_um: float | None = None,
        step_y_um: float | None = None,
    ) -> list[tuple[float, float]]:
        """Generate a serpentine raster grid of scan positions covering the given area.

        The grid is based on the camera's field of view, not per-well positions.
        Each position is the stage (X, Y) where the camera center should be placed.

        Args:
            bounds_um: (min_x, min_y, max_x, max_y) of the scan area in µm.
            overlap: Fractional overlap between adjacent frames (0.0–0.5).
                     Default 10% ensures seamless stitching.
            step_x_um / step_y_um: v7.5.x — explicit grid spacing (µm) between
                tile centres. When given (> 0) these OVERRIDE the FOV×(1−overlap)
                computation, so the operator can dial spacing in directly when
                the FOV-derived value doesn't match reality.

        Returns:
            List of (x_um, y_um) stage positions in serpentine (meander) order.
        """
        self._overlap = overlap
        min_x, min_y, max_x, max_y = bounds_um

        # FOV in µm, along the STAGE axes (v7.16). A camera mounted at ±90°
        # covers the sensor's HEIGHT in stage-X and its WIDTH in stage-Y; using
        # the raw camera-axis FOV stepped too far on one axis (near-zero real
        # overlap) and too little on the other, and only min(w,h) of each frame
        # survived — the sensor behaved as a square.
        fov_w, fov_h = self._oriented_fov_um()

        # Step size with overlap — or the explicit override when provided.
        step_x = (float(step_x_um) if step_x_um and step_x_um > 0
                  else fov_w * (1.0 - overlap))
        step_y = (float(step_y_um) if step_y_um and step_y_um > 0
                  else fov_h * (1.0 - overlap))

        # Inset by half-FOV so the camera center stays within bounds
        # while the frame edges still cover the boundary
        start_x = min_x + fov_w / 2.0
        start_y = min_y + fov_h / 2.0
        end_x = max_x - fov_w / 2.0
        end_y = max_y - fov_h / 2.0

        # Handle case where scan area is smaller than one FOV
        if start_x > end_x:
            start_x = end_x = (min_x + max_x) / 2.0
        if start_y > end_y:
            start_y = end_y = (min_y + max_y) / 2.0

        # Generate grid
        cols = max(1, int(math.ceil((end_x - start_x) / step_x)) + 1) if step_x > 0 else 1
        rows = max(1, int(math.ceil((end_y - start_y) / step_y)) + 1) if step_y > 0 else 1

        self._grid_cols = cols
        self._grid_rows = rows

        positions: list[tuple[float, float]] = []
        for row in range(rows):
            y = start_y + row * step_y if rows > 1 else start_y
            y = min(y, end_y)  # clamp last row

            col_range = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
            for col in col_range:
                x = start_x + col * step_x if cols > 1 else start_x
                x = min(x, end_x)  # clamp last col
                positions.append((x, y))

        # Pre-allocate composite canvas
        self._init_composite(bounds_um)

        logger.info(
            f"Raster grid: {cols}×{rows} = {len(positions)} positions, "
            f"FOV {fov_w:.0f}×{fov_h:.0f} µm, step {step_x:.0f}×{step_y:.0f} µm"
        )
        return positions

    def generate_spiral_scan_positions(
        self,
        center_um: tuple[float, float],
        max_radius_um: float,
        bounds_um: tuple[float, float, float, float],
        overlap: float = 0.12,
    ) -> list[tuple[float, float]]:
        """Generate an Archimedean spiral of scan positions from center outward.

        Used for discovery scanning: spiral from the assumed plate center
        to detect wells and determine plate orientation before committing
        to a full raster scan.

        Args:
            center_um: (x, y) spiral center in µm (typically plate center).
            max_radius_um: Maximum spiral radius in µm.
            bounds_um: (min_x, min_y, max_x, max_y) — positions outside
                       this box are skipped (stage travel limits).
            overlap: Fractional overlap between adjacent passes (0.0–0.5).

        Returns:
            List of (x_um, y_um) stage positions in spiral order.
        """
        self._overlap = overlap
        cx, cy = center_um
        min_x, min_y, max_x, max_y = bounds_um

        fov_w, fov_h = self._oriented_fov_um()

        # Radial pitch: distance between rings (use smaller FOV dim for overlap)
        pitch = min(fov_w, fov_h) * (1.0 - overlap)
        # Arc-length step: distance between frames along a ring
        arc_step = max(fov_w, fov_h) * (1.0 - overlap)

        # Spiral has no grid structure — disable neighbor-graph in build_mosaic
        self._grid_cols = 0
        self._grid_rows = 0

        positions: list[tuple[float, float]] = []

        # First position: center itself
        if min_x <= cx <= max_x and min_y <= cy <= max_y:
            positions.append((cx, cy))

        # Walk the Archimedean spiral: r(theta) = pitch * theta / (2*pi)
        theta = 0.0
        while True:
            # Advance theta by arc_step / r (clamped to avoid tiny r near center)
            r = pitch * theta / (2.0 * math.pi)
            if r > max_radius_um:
                break

            effective_r = max(r, pitch * 0.5)
            theta += arc_step / effective_r

            r = pitch * theta / (2.0 * math.pi)
            if r > max_radius_um:
                break

            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)

            # Skip positions outside bounds
            if x < min_x or x > max_x or y < min_y or y > max_y:
                continue

            positions.append((x, y))

        # Pre-allocate composite canvas
        self._init_composite(bounds_um)

        logger.info(
            f"Spiral scan: {len(positions)} positions from center "
            f"({cx:.0f}, {cy:.0f}) µm, max_radius={max_radius_um:.0f} µm, "
            f"pitch={pitch:.0f} µm"
        )
        return positions

    # ── Composite canvas management ────────────────────────────────

    def _init_composite(self, bounds_um: tuple[float, float, float, float]):
        """Pre-allocate the composite canvas based on scan bounds."""
        if not CV2_AVAILABLE:
            return

        min_x, min_y, max_x, max_y = bounds_um
        fov_w, fov_h = self._oriented_fov_um()
        # v7.16: an off-centre crop places every tile displaced from the position
        # it was commanded to, so the padding has to cover that too. At a small
        # crop the displacement can EXCEED half a FOV, and the tile would then be
        # clipped at the canvas edge — the bounds are the operator's scan region,
        # not the tiles' true footprint. Only the canvas grows; the origin is
        # unchanged, so extent→stage back-projection is untouched.
        toff_x, toff_y = getattr(self, "_tile_center_offset_um", (0.0, 0.0))

        # Canvas covers scan bounds + half-FOV padding on each side
        canvas_w_um = (max_x - min_x) + fov_w + 2.0 * abs(toff_x)
        canvas_h_um = (max_y - min_y) + fov_h + 2.0 * abs(toff_y)

        # Scale factor: µm → mosaic pixels
        self._mosaic_scale = self._target_mosaic_px / max(canvas_w_um, canvas_h_um, 1.0)

        canvas_w = max(1, int(canvas_w_um * self._mosaic_scale))
        canvas_h = max(1, int(canvas_h_um * self._mosaic_scale))

        # Origin: top-left of canvas in µm (min position minus half-FOV, minus
        # any crop displacement so a negative offset also stays on the canvas).
        self._canvas_origin_um = (min_x - fov_w / 2.0 - abs(toff_x),
                                  min_y - fov_h / 2.0 - abs(toff_y))
        # Store the full canvas world extent for display coordinate mapping
        self._canvas_extent_um = (
            self._canvas_origin_um[0],
            self._canvas_origin_um[1],
            self._canvas_origin_um[0] + canvas_w_um,
            self._canvas_origin_um[1] + canvas_h_um,
        )

        # Float accumulator for weighted blending
        self._composite = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
        self._weight_sum = np.zeros((canvas_h, canvas_w), dtype=np.float64)
        self._display_cache = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        # Pre-compute feather weights for one tile at mosaic resolution
        tile_w = max(1, int(fov_w * self._mosaic_scale))
        tile_h = max(1, int(fov_h * self._mosaic_scale))
        overlap_px = int(tile_w * self._overlap)
        margin = max(1, overlap_px)
        self._feather_weights = _compute_feather_weights(tile_h, tile_w, margin)

        logger.info(
            f"Composite canvas: {canvas_w}×{canvas_h} px, "
            f"scale={self._mosaic_scale:.4f} px/µm"
        )

    # ── Frame collection ───────────────────────────────────────────

    def add_frame(
        self,
        well_name: str,
        frame: np.ndarray,
        stage_x_um: float,
        stage_y_um: float,
        detection: Any | None = None,
    ) -> FrameRecord:
        """Add a captured frame with its stage position and optional detection result.

        Args:
            well_name: Well being imaged (e.g. "A1").
            frame: BGR camera frame.
            stage_x_um: Absolute stage X position (µm) when frame was captured.
            stage_y_um: Absolute stage Y position (µm) when frame was captured.
            detection: Optional DetectionResult from WellDetector.detect_well().

        Returns:
            The created FrameRecord.
        """
        offset_um = None
        confidence = 0.0
        # v7.16: an off-centre crop displaces the tile's content from the stage
        # position it was captured at — see ``_tile_center_offset_um``.
        toff_x, toff_y = getattr(self, "_tile_center_offset_um", (0.0, 0.0))

        if detection is not None:
            cx_px, cy_px = detection.center_px
            fw, fh = self._frame_size_px
            dx_px = cx_px - fw / 2.0
            dy_px = cy_px - fh / 2.0
            # The detection's offset is measured from the frame centre too, so
            # it carries the same displacement.
            offset_um = (dx_px * self._um_per_px + toff_x,
                         dy_px * self._um_per_px + toff_y)
            confidence = detection.confidence

        record = FrameRecord(
            well_name=well_name,
            frame=frame,
            stage_x_um=stage_x_um + toff_x,
            stage_y_um=stage_y_um + toff_y,
            detected_offset_um=offset_um,
            confidence=confidence,
        )
        self._records.append(record)
        return record

    def add_raster_frame(
        self,
        frame: np.ndarray,
        stage_x_um: float,
        stage_y_um: float,
        index: int = 0,
    ) -> FrameRecord:
        """Add a raster-scan frame (no well association).

        Args:
            frame: BGR camera frame.
            stage_x_um: Absolute stage X position (µm).
            stage_y_um: Absolute stage Y position (µm).
            index: Frame index (used as label).

        Returns:
            The created FrameRecord.
        """
        toff_x, toff_y = getattr(self, "_tile_center_offset_um", (0.0, 0.0))
        record = FrameRecord(
            well_name=f"R{index}",
            frame=frame,
            stage_x_um=stage_x_um + toff_x,
            stage_y_um=stage_y_um + toff_y,
        )
        self._records.append(record)
        return record

    # ── Incremental stitching (called after each frame) ────────────

    def stitch_incremental(self) -> np.ndarray | None:
        """Add the latest frame to the composite using feathered blending.

        Called after each add_frame/add_raster_frame to progressively
        build the mosaic. Uses stage coordinates for placement.

        Returns:
            Current composite image (BGR uint8) or None.
        """
        if not CV2_AVAILABLE or not self._records:
            return None
        if self._composite is None or self._weight_sum is None:
            return None

        rec = self._records[-1]
        self._blend_tile_to_composite(rec)
        # v7.5.x: in frame-light mode, drop the raw frame now that it's blended
        # into the composite — it is never needed again (this builder doesn't
        # re-blend via build_mosaic). Keeps RAM bounded over a long scan.
        if not self._retain_frames:
            rec.frame = None
        return self.composite

    def free_accumulators(self) -> None:
        """v7.5.x: release the float64 accumulators (``_composite`` +
        ``_weight_sum`` — together ~190 MB on a full-plate canvas) once the scan
        is complete, keeping only the uint8 ``_display_cache`` (the finished
        mosaic). Call after the final tile + ``finalize_global_shift``; the
        display cache is already complete (updated incrementally per tile), so
        the global shift (applied via ``canvas_extent_um``) is unaffected. After
        this, ``stitch_incremental`` / ``build_mosaic`` are no-ops until re-init.

        v7.5.x: skipped when ``retain_for_reorient`` is set — the interactive
        fix-orientation tool needs the accumulators to ``reblend_reoriented``.
        Call ``free_reorient`` to release them once the adjustment is done.
        """
        if self._retain_for_reorient:
            return
        self._composite = None
        self._weight_sum = None

    def free_reorient(self) -> None:
        """v7.5.x: release the re-orientation buffers (retained tiles + the
        float64 accumulators) once the fix-orientation adjustment is done."""
        self._reorient_tiles = []
        self._composite = None
        self._weight_sum = None

    def _max_shift_px(self) -> float:
        """Per-tile registration bound in mosaic px (0 µm → 20% of the FOV)."""
        if self._max_shift_um and self._max_shift_um > 0:
            return float(self._max_shift_um) * self._mosaic_scale
        fov_w_px = self._oriented_fov_um()[0] * self._mosaic_scale
        return 0.2 * fov_w_px

    def _measure_tile_shift(self, resized, px, py, tw, th):
        """MEASURE this tile's overlap misalignment vs the already-stitched
        composite (phase correlation). Returns (dx_px, dy_px) sub-pixel, or
        None when there isn't enough prior content or the match is weak. The
        tile is NOT moved — the measurements are aggregated globally.
        """
        if (not SKIMAGE_AVAILABLE or self._display_cache is None
                or self._weight_sum is None):
            return None
        ch, cw = self._weight_sum.shape[:2]
        x1 = max(0, px)
        y1 = max(0, py)
        x2 = min(cw, px + tw)
        y2 = min(ch, py + th)
        if (x2 - x1) < 8 or (y2 - y1) < 8:
            return None
        covered = self._weight_sum[y1:y2, x1:x2] > 0
        # Need enough already-stitched content in the overlap to register to.
        if float(covered.mean()) < 0.25:
            return None
        try:
            comp_region = self._display_cache[y1:y2, x1:x2]
            comp_gray = cv2.cvtColor(comp_region, cv2.COLOR_BGR2GRAY)
            sx1 = x1 - px
            sy1 = y1 - py
            tile_region = resized[sy1:sy1 + (y2 - y1), sx1:sx1 + (x2 - x1)]
            tile_gray = cv2.cvtColor(tile_region, cv2.COLOR_BGR2GRAY)
            # Zero out the not-yet-stitched pixels in BOTH so they don't bias
            # the correlation toward the empty canvas.
            mask = ~covered
            comp_gray = comp_gray.copy()
            tile_gray = tile_gray.copy()
            comp_gray[mask] = 0
            tile_gray[mask] = 0
            # Featureless overlap (uniform colour — between wells, or an in-well
            # region with no edge in view) can't be registered: phase
            # correlation on a flat patch returns noise. Skip it so it neither
            # fails nor injects a bogus shift; it still gets the global shift.
            cov = covered
            if (float(comp_gray[cov].std()) < _REGISTER_MIN_STD
                    or float(tile_gray[cov].std()) < _REGISTER_MIN_STD):
                return None
            # Shift to bring the new tile (b) onto the existing composite (a).
            dy, dx, conf = _phase_correlate_overlap(comp_gray, tile_gray)
        except Exception:
            return None
        if conf < 0.05:
            return None
        return float(dx), float(dy)

    def finalize_global_shift(self) -> tuple[float, float]:
        """Aggregate the per-overlap measurements into ONE global alignment
        shift (robust median, bounded by max_shift) applied uniformly to the
        whole mosaic via ``canvas_extent_um``. Trusts the accurate stage for
        relative placement — a single bad overlap match can't move one tile,
        and per-tile stage noise is averaged out. Returns the shift in µm.
        """
        if not self._register or not self._measured_shifts:
            # No fresh measurements — keep any pre-seeded learned correction.
            return self._global_shift_um
        arr = np.array(self._measured_shifts, dtype=float)   # (N, 2) = (dx, dy) px
        gdx = float(np.median(arr[:, 0]))
        gdy = float(np.median(arr[:, 1]))
        m = self._max_shift_px()
        gdx = max(-m, min(m, gdx))
        gdy = max(-m, min(m, gdy))
        scale = self._mosaic_scale if self._mosaic_scale else 1.0
        self._global_shift_um = (gdx / scale, gdy / scale)
        logger.info(
            "Mosaic global alignment: shift "
            f"({self._global_shift_um[0]:.1f}, {self._global_shift_um[1]:.1f}) "
            f"µm (median of {len(self._measured_shifts)} overlaps)")
        return self._global_shift_um

    def _fov_um(self) -> tuple[float, float]:
        """One frame's extent in µm along the CAMERA's own pixel axes."""
        return (self._frame_size_px[0] * self._um_per_px,
                self._frame_size_px[1] * self._um_per_px)

    def _oriented_fov_um(self) -> tuple[float, float]:
        """One frame's extent in µm along the STAGE axes, after orientation.

        v7.16. The camera can be mounted rotated; ``_orient_tile`` turns each
        tile into stage axes, so the stage-frame footprint of a tile is the
        rotated rectangle's bounding box — NOT the raw (w, h).

        For a rectangular sensor at ±90° the two differ by the aspect ratio,
        and using the raw values (as every caller used to) is what made the
        mosaic behave as if the sensor were **square**:

        * ``_orient_tile`` rendered the rotated content back into a w×h canvas,
          so on a 2600×2048 frame at −90° the 2600-px axis was clipped to 2048
          — **21 % of every frame discarded** — and black bars were blended in
          on the other axis;
        * the raster stepped by the *unrotated* FOV, so the real coverage per
          tile collapsed to min(w,h)² — a square — leaving ~0 % overlap on one
          axis while over-scanning the other.

        Exact at 0°/±90°/180°; for a non-axis angle this is the bounding box, so
        the corners are empty — that is the pre-existing, documented trade (the
        scan overlap covers it), unchanged here.
        """
        fw, fh = self._fov_um()
        theta = float(getattr(self, "_frame_rotation_deg", 0.0) or 0.0)
        if abs(theta) < 0.05:
            return (fw, fh)
        t = math.radians(theta)
        c, s = abs(math.cos(t)), abs(math.sin(t))
        return (fw * c + fh * s, fw * s + fh * c)

    def _oriented_size_px(self, w: int, h: int) -> tuple[int, int]:
        """Canvas size a ``w×h`` tile occupies once oriented into stage axes."""
        theta = float(getattr(self, "_frame_rotation_deg", 0.0) or 0.0)
        if abs(theta) < 0.05:
            return (int(w), int(h))
        t = math.radians(theta)
        c, s = abs(math.cos(t)), abs(math.sin(t))
        return (max(1, int(round(w * c + h * s))),
                max(1, int(round(w * s + h * c))))

    def _orient_tile(self, tile: np.ndarray) -> np.ndarray:
        """v7.5.x: orient a tile from CAMERA-pixel axes into STAGE axes.

        Applies the camera's calibrated mirror (horizontal flip) then rotation
        θ about the tile centre, matching ``CameraManager.pixel_to_stage_offset``
        (mirror ``dx→−dx`` then ``R(θ)``) and ``CameraFeedView.set_view_orientation``.
        The tile centre is invariant, so the tile stays pinned at its
        stage-derived canvas position while its content is oriented to align
        with the stage axes — making the composite a stage-aligned orthophoto.

        No-op fast path when unmirrored and |θ| < 0.05° (byte-identical to the
        legacy raw placement).

        v7.16: the output is the ROTATED BOUNDING BOX (``_oriented_size_px``),
        not the input W×H. Rendering back into W×H clipped a rectangular sensor
        to its short axis at ±90° — 21 % of a 2600×2048 frame — and padded the
        other axis with black, which is the "the mosaic treats the view as a
        square" the operator reported. The tile CENTRE is still invariant, so
        callers place it centred on the same stage point.

        NOTE: the rotation SIGN / mirror axis match ``pixel_to_stage_offset`` by
        construction; verify on real hardware (a mis-signed θ would orient the
        mosaic the wrong way).
        """
        theta = self._frame_rotation_deg
        mir = self._frame_mirrored                 # flip X (horizontal)
        fy = getattr(self, "_frame_flip_y", False)  # flip Y (vertical)
        if (not mir and not fy and abs(theta) < 0.05) or cv2 is None:
            return tile
        h, w = tile.shape[:2]
        ow, oh = self._oriented_size_px(w, h)
        cx = (w - 1) / 2.0
        cy = (h - 1) / 2.0
        dcx = (ow - 1) / 2.0
        dcy = (oh - 1) / 2.0
        t = math.radians(theta)
        c, s = math.cos(t), math.sin(t)
        mx = -1.0 if mir else 1.0
        my = -1.0 if fy else 1.0
        # Linear part A = R(θ)·diag(mx, my); forward (src→dst) affine that holds
        # the centre fixed. warpAffine (no INVERSE flag) maps src→dst. Matches
        # pixel_to_stage_offset (flip X on dx, flip Y on dy, then R(θ)) and the
        # CameraFeedView display transform — ONE unified orientation.
        a00, a01 = c * mx, -s * my
        a10, a11 = s * mx, c * my
        M = np.array([
            [a00, a01, dcx - (a00 * cx + a01 * cy)],
            [a10, a11, dcy - (a10 * cx + a11 * cy)],
        ], dtype=np.float64)
        try:
            return cv2.warpAffine(
                tile, M, (ow, oh), flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        except Exception:
            return tile

    def _blend_tile_to_composite(self, rec: FrameRecord):
        """Blend a single tile into the composite canvas with feathering."""
        if self._composite is None or self._weight_sum is None:
            return

        fov_w_um, fov_h_um = self._fov_um()
        # v7.16: the stage-frame footprint AFTER orientation. Identical to the
        # camera-axis FOV at 0°; transposed at ±90°. Placing an oriented tile in
        # the unrotated box is what squashed a rectangular sensor into a square.
        ofov_w_um, ofov_h_um = self._oriented_fov_um()
        scale = self._mosaic_scale
        ox, oy = self._canvas_origin_um

        # Camera-axis tile box. The resize preserves the frame's aspect, which
        # is what keeps px→µm ISOTROPIC — and that isotropy is precisely what
        # makes rotating inside this box geometrically correct.
        tile_w = max(1, int(fov_w_um * scale))
        tile_h = max(1, int(fov_h_um * scale))

        # Resize the frame to canvas resolution
        try:
            resized = cv2.resize(rec.frame, (tile_w, tile_h),
                                 interpolation=cv2.INTER_AREA)
        except Exception:
            return

        # Top-left of the ORIENTED footprint on the canvas (µm → px). The tile
        # centre is orientation-invariant, so this stays pinned to the stage
        # position the frame was captured at.
        px = int((rec.stage_x_um - ofov_w_um / 2.0 - ox) * scale)
        py = int((rec.stage_y_um - ofov_h_um / 2.0 - oy) * scale)
        out_w, out_h = self._oriented_size_px(tile_w, tile_h)

        # v7.5.x: retain the small PRE-orient canvas-res tile so the interactive
        # fix-orientation tool (set_frame_orientation + reblend_reoriented) can
        # re-render the whole mosaic with a NEW orientation without re-scanning.
        # The box recorded alongside it is the ORIENTED one, so its centre is
        # what a later re-blend re-centres on when the orientation changes.
        if self._retain_for_reorient:
            try:
                self._reorient_tiles.append(
                    (resized.copy(), int(px), int(py),
                     int(out_w), int(out_h)))
            except Exception:
                pass

        # v7.5.x: orient the tile from camera-pixel axes into stage axes
        # (calibrated mirror + rotation) so the composite is stage-aligned and
        # mosaic clicks back-project to the correct XY. No-op at (0°, unmirrored).
        oriented = self._orient_tile(resized)

        # v7.5.x: registration. Measure this tile's overlap misalignment, then
        # either (global, default) record it for ONE uniform shift finalized
        # later — trusting the accurate stage for relative placement — or
        # (per_tile, retained for future apps) nudge THIS tile by its own
        # bounded shift now.
        if self._register:
            try:
                shift = self._measure_tile_shift(oriented, px, py,
                                                 out_w, out_h)
                if shift is not None:
                    if self._register_mode == "per_tile":
                        m = self._max_shift_px()
                        px += int(round(max(-m, min(m, shift[0]))))
                        py += int(round(max(-m, min(m, shift[1]))))
                        rec.refined_x_px = float(px + out_w / 2.0)
                        rec.refined_y_px = float(py + out_h / 2.0)
                    else:
                        self._measured_shifts.append(shift)
            except Exception as e:
                logger.debug(f"Overlap registration skipped: {e}")

        self._accumulate_oriented_tile(oriented, px, py, out_w, out_h)

        # Store refined position on canvas
        rec.refined_x_px = float(px + out_w / 2.0)
        rec.refined_y_px = float(py + out_h / 2.0)

    # ── v7.16: intensity regularization (flat field + per-tile gain) ──

    def _intensity_correct(self, tile, gain=None):
        """Apply the estimated flat field + per-tile gain to a PRE-orient tile.

        No-op (and byte-identical) until ``regularize_intensity`` has run.
        Applied BEFORE orientation because both corrections live in CAMERA
        pixel coordinates — vignetting is a property of the optical path and
        the sensor, so it is fixed in the frame, not on the plate.
        """
        ff = getattr(self, "_flat_field", None)
        if ff is None and (gain is None or abs(gain - 1.0) < 1e-9):
            return tile
        try:
            out = tile.astype(np.float32)
            if ff is not None and ff.shape[:2] == out.shape[:2]:
                out = out / ff[:, :, None]
            if gain is not None:
                out = out * float(gain)
            return np.clip(out, 0.0, 255.0).astype(tile.dtype)
        except Exception as exc:
            logger.debug(f"intensity correction skipped: {exc}")
            return tile

    def _tile_luma(self, tile):
        """Robust per-tile brightness (median over a subsample)."""
        try:
            g = tile[::4, ::4]
            g = g.mean(axis=2) if g.ndim == 3 else g
            return float(np.median(g))
        except Exception:
            return 0.0

    def estimate_flat_field(self, *, blur_frac: float = 0.25,
                            min_tiles: int = 6):
        """Estimate the illumination/vignetting profile FROM THE TILES.

        Returns a float32 (h, w) gain field normalised to a mean of 1.0, or
        None when there is not enough data.

        Each retained tile is divided by its own median, so specimen content —
        which moves from tile to tile — averages out under a per-pixel MEDIAN
        while the illumination pattern, which is fixed in the frame, survives.
        The median (not the mean) is what makes this robust to the minority of
        tiles containing a big bright or dark object.

        The result is heavily blurred: real vignetting is a smooth, low-order
        function of position, so anything sharp in the estimate is residual
        specimen structure — and dividing by that would BURN a ghost of one
        tile's content into every tile. ``min_tiles`` exists for the same
        reason; with only a handful of frames the median cannot separate the
        two and the honest answer is "don't correct".
        """
        tiles = [t[0] for t in getattr(self, "_reorient_tiles", []) or []]
        if cv2 is None or len(tiles) < int(min_tiles):
            return None
        try:
            h, w = tiles[0].shape[:2]
            stack = []
            for t in tiles:
                if t.shape[:2] != (h, w):
                    continue
                g = t.mean(axis=2).astype(np.float32) if t.ndim == 3 else \
                    t.astype(np.float32)
                m = float(np.median(g))
                if m <= 1e-3:
                    continue
                stack.append(g / m)
            if len(stack) < int(min_tiles):
                return None
            ff = np.median(np.stack(stack, axis=0), axis=0).astype(np.float32)
            k = max(3, int(min(h, w) * float(blur_frac)) | 1)
            ff = cv2.GaussianBlur(ff, (k, k), 0, borderType=cv2.BORDER_REPLICATE)
            mean = float(ff.mean())
            if mean <= 1e-6:
                return None
            ff /= mean
            # A degenerate field (near-flat or wildly out of range) means the
            # estimate failed; correcting by it would only add noise.
            np.clip(ff, 0.2, 5.0, out=ff)
            return ff
        except Exception as exc:
            logger.debug(f"flat-field estimate failed: {exc}")
            return None

    def regularize_intensity(self, *, flat_field: bool = True,
                             match_gain: bool = True,
                             positions=None):
        """Even out illumination across the mosaic, then re-blend.

        Two independent effects, both visible as a tile grid in a large mosaic:

        * **vignetting** — each frame is darker at its edges, so every tile
          boundary shows as a soft dark seam. Corrected by dividing each tile
          by :meth:`estimate_flat_field`.
        * **tile-to-tile level** — lamp drift, auto-exposure, or simply a
          brighter region of the specimen leave neighbouring tiles at
          different levels. Corrected by scaling each tile's median to the
          median of all tiles.

        Requires ``retain_for_reorient=True`` (the canvas-res tiles are the
        input). Returns ``(composite, applied_flat_field, n_gain_corrected)``;
        ``composite`` is unchanged when there was nothing to do, so a caller
        can always call this unconditionally.
        """
        tiles = getattr(self, "_reorient_tiles", None)
        if not tiles or self._composite is None or self._weight_sum is None:
            return self.composite, False, 0

        ff = self.estimate_flat_field() if flat_field else None
        self._flat_field = ff

        gains = None
        if match_gain:
            lumas = [self._tile_luma(t[0]) for t in tiles]
            valid = [x for x in lumas if x > 1e-3]
            if len(valid) >= 2:
                target = float(np.median(valid))
                # Bounded: a tile legitimately dominated by a bright object
                # must not be dragged to the plate average, which would erase
                # the very signal the scan is for.
                gains = [
                    (min(2.0, max(0.5, target / x)) if x > 1e-3 else 1.0)
                    for x in lumas]
        self._tile_gains = gains

        if ff is None and gains is None:
            return self.composite, False, 0

        if positions is None:
            positions = getattr(self, "_optimized_positions", None)
        self._composite[...] = 0.0
        self._weight_sum[...] = 0.0
        if self._display_cache is not None:
            self._display_cache[...] = 0
        for i, (resized, px, py, tw, th) in enumerate(tiles):
            try:
                if positions is not None and i < len(positions):
                    px = int(round(float(positions[i][0])))
                    py = int(round(float(positions[i][1])))
                self._place_from_box(resized, px, py, tw, th,
                                     gain=(gains[i] if gains else None))
            except Exception as e:
                logger.debug(f"regularize tile skipped: {e}")
        n_gain = len(gains) if gains else 0
        logger.info(
            "Mosaic intensity regularized: flat-field=%s, gain-matched %d tiles",
            "yes" if ff is not None else "no", n_gain)
        return self.composite, ff is not None, n_gain

    def _place_from_box(self, resized, px, py, tw, th, gain=None):
        """Orient a retained PRE-orient tile and blend it centred on the box.

        v7.16. The three re-blend paths used to pass the STORED box size
        straight to ``_accumulate_oriented_tile``. That was fine while
        ``_orient_tile`` returned the input size, but it now returns the rotated
        bounding box — and ``reblend_reoriented`` exists precisely to re-render
        at a DIFFERENT orientation, where the stored box no longer describes the
        result. Re-centring on the stored box's centre keeps every tile pinned
        to the same stage point across an orientation change.
        """
        oriented = self._orient_tile(self._intensity_correct(resized, gain))
        oh, ow = oriented.shape[:2]
        cx = float(px) + float(tw) / 2.0
        cy = float(py) + float(th) / 2.0
        self._accumulate_oriented_tile(
            oriented, int(round(cx - ow / 2.0)), int(round(cy - oh / 2.0)),
            ow, oh)

    def _accumulate_oriented_tile(self, oriented, px, py, tile_w, tile_h):
        """Feather-blend an already-oriented, canvas-res tile into the composite
        at (px, py). Shared by ``_blend_tile_to_composite`` and
        ``reblend_reoriented``."""
        if self._composite is None or self._weight_sum is None:
            return
        # v7.16: trust the ARRAY, not the caller's idea of its size — the
        # oriented size is derived from the rotation and a stale (tile_w,
        # tile_h) would silently crop or over-read the blend region.
        try:
            tile_h, tile_w = oriented.shape[:2]
        except Exception:
            return
        # Get feather weights (resize if dimensions don't match)
        if (self._feather_weights is not None
                and self._feather_weights.shape == (tile_h, tile_w)):
            weights = self._feather_weights
        else:
            overlap_px = max(1, int(tile_w * self._overlap))
            weights = _compute_feather_weights(tile_h, tile_w, overlap_px)

        # Clip to canvas bounds
        ch, cw = self._composite.shape[:2]
        x1 = max(0, px)
        y1 = max(0, py)
        x2 = min(cw, px + tile_w)
        y2 = min(ch, py + tile_h)

        # Source region within the tile
        sx1 = x1 - px
        sy1 = y1 - py
        sx2 = sx1 + (x2 - x1)
        sy2 = sy1 + (y2 - y1)

        if x2 <= x1 or y2 <= y1:
            return

        # Accumulate weighted pixel values
        tile_crop = oriented[sy1:sy2, sx1:sx2].astype(np.float64)
        w_crop = weights[sy1:sy2, sx1:sx2]

        for c in range(3):
            self._composite[y1:y2, x1:x2, c] += tile_crop[:, :, c] * w_crop
        self._weight_sum[y1:y2, x1:x2] += w_crop

        # Update display cache for only the affected region (fast)
        if self._display_cache is not None:
            region_ws = self._weight_sum[y1:y2, x1:x2]
            mask = region_ws > 0
            for c in range(3):
                ch2 = self._composite[y1:y2, x1:x2, c].copy()
                ch2[mask] = (ch2[mask] / region_ws[mask]).clip(0, 255)
                self._display_cache[y1:y2, x1:x2, c] = ch2.astype(np.uint8)

    # ── v7.5.x: interactive re-orientation ─────────────────────────

    def set_frame_orientation(self, rotation_deg: float = 0.0,
                              mirrored: bool = False,
                              flip_y: bool = False) -> None:
        """Set the per-tile orientation used by ``_orient_tile`` /
        ``reblend_reoriented`` (rotation, flip X = ``mirrored``, flip Y)."""
        self._frame_rotation_deg = float(rotation_deg or 0.0)
        self._frame_mirrored = bool(mirrored)
        self._frame_flip_y = bool(flip_y)

    def reblend_reoriented(self):
        """Re-render the whole composite from the retained pre-orient tiles with
        the CURRENT orientation (``set_frame_orientation``). Returns the new
        composite (BGR uint8) or None. Requires ``retain_for_reorient=True``."""
        if (self._composite is None or self._weight_sum is None
                or not self._reorient_tiles):
            return self.composite
        # Zero the accumulators + display cache (same canvas), then re-blend.
        self._composite[...] = 0.0
        self._weight_sum[...] = 0.0
        if self._display_cache is not None:
            self._display_cache[...] = 0
        for (resized, px, py, tw, th) in self._reorient_tiles:
            try:
                self._place_from_box(resized, px, py, tw, th)
            except Exception as e:
                logger.debug(f"reblend tile skipped: {e}")
        return self.composite

    def has_reorient_tiles(self) -> bool:
        return bool(self._reorient_tiles)

    # ── v7.5.x: full pairwise + global-optimize registration ───────

    def optimize_registration(self, *, min_overlap_frac: float = 0.10,
                              min_conf: float = 0.12, min_edges: int = 1,
                              method: str = None):
        """Register EVERY overlapping tile pair, solve globally-consistent tile
        positions (least-squares), and re-blend at the optimized positions.

        v7.5.x (operator: "we need a better image registration algorithm — the
        current feature detection doesn't work great when moving the stage").
        The open-loop stitch places tiles purely by stage position and applies
        only ONE global median shift; that drifts when the stage backlashes or
        the FOV is slightly off. This measures each overlap with a CLAHE +
        Hanning phase correlation (illumination-gradient tolerant, real
        cross-power peak as confidence — the weak point of the plain estimator)
        and feeds a weighted global least-squares solve
        (``_global_optimize_positions``) so residuals are shared consistently
        across the whole grid rather than accumulating.

        Safety: each tile's correction is BOUNDED to the trusted stage placement
        (``_max_shift_px``) — a single bad match cannot fling a tile — and a
        degenerate solve (too few confident overlaps) is a NO-OP that keeps the
        open-loop composite. Operates on the retained canvas-res tiles
        (``retain_for_reorient``), so memory stays bounded and no re-scan or raw
        frames are needed. Returns ``(composite, n_edges, max_correction_px)``.
        """
        method = str(method or getattr(self, "_registration_method",
                                        "fourier_mellin"))
        if method == "off":
            # Manual / stage-only mode — no auto-registration; the operator
            # aligns the mosaic by hand (the trusted stage placement stands).
            return self.composite, 0, 0.0
        tiles = self._reorient_tiles
        if (cv2 is None or not tiles or self._composite is None
                or self._weight_sum is None):
            return self.composite, 0, 0.0
        n = len(tiles)
        if n < 2:
            return self.composite, 0, 0.0
        nominal = np.array([[float(t[1]), float(t[2])] for t in tiles],
                           dtype=float)
        grays: list = []
        for (tile, _px, _py, _tw, _th) in tiles:
            try:
                grays.append(
                    cv2.cvtColor(self._orient_tile(tile), cv2.COLOR_BGR2GRAY))
            except Exception:
                grays.append(None)
        edges: list = []
        offsets: list = []
        confs: list = []
        rot_samples: list = []
        scale_samples: list = []
        use_fm = (str(method) == "fourier_mellin")
        for i in range(n):
            gi = grays[i]
            if gi is None:
                continue
            xi, yi, wi, hi = (int(tiles[i][1]), int(tiles[i][2]),
                              int(tiles[i][3]), int(tiles[i][4]))
            for j in range(i + 1, n):
                gj = grays[j]
                if gj is None:
                    continue
                xj, yj, wj, hj = (int(tiles[j][1]), int(tiles[j][2]),
                                  int(tiles[j][3]), int(tiles[j][4]))
                ox1, oy1 = max(xi, xj), max(yi, yj)
                ox2, oy2 = min(xi + wi, xj + wj), min(yi + hi, yj + hj)
                ow, oh = ox2 - ox1, oy2 - oy1
                if ow < 12 or oh < 12:
                    continue
                if ow * oh < min_overlap_frac * min(wi * hi, wj * hj):
                    continue
                a = gi[oy1 - yi:oy2 - yi, ox1 - xi:ox2 - xi]
                b = gj[oy1 - yj:oy2 - yj, ox1 - xj:ox2 - xj]
                if a.shape != b.shape or a.size == 0:
                    continue
                dxr = dyr = conf = None
                if use_fm:
                    fm = _register_overlap_fourier_mellin(a, b)
                    if fm is not None:
                        dxr, dyr, rot_ij, scale_ij, conf = fm
                        rot_samples.append(rot_ij)
                        scale_samples.append(scale_ij)
                if conf is None:            # FM off / failed → plain phase corr
                    pc = _register_overlap_cv2(a, b)
                    if pc is not None:
                        dxr, dyr, conf = pc
                if conf is None or conf < min_conf:
                    continue
                offsets.append(((xj - xi) + dxr, (yj - yi) + dyr))
                edges.append((i, j, 'p'))
                confs.append(conf)
        # Diagnostic: the median tile-to-tile rotation/scale should be ~0°/~1;
        # a large value flags a camera µm/px / rotation calibration that is off
        # (the operator can then recalibrate or correct it manually).
        if rot_samples:
            self._measured_rotation_deg = float(np.median(rot_samples))
            self._measured_scale = float(np.median(scale_samples))
            if (abs(self._measured_rotation_deg) > 1.0
                    or abs(self._measured_scale - 1.0) > 0.03):
                logger.info(
                    "Mosaic registration residual (Fourier-Mellin): median "
                    f"rotation {self._measured_rotation_deg:.2f}°, scale "
                    f"{self._measured_scale:.4f} — a large value means the "
                    "camera rotation/µm-px calibration is off.")
        if len(edges) < max(1, int(min_edges)):
            return self.composite, len(edges), 0.0
        opt = _global_optimize_positions(
            n, nominal, edges, offsets, confs, min_confidence=min_conf)
        # Bound each tile to its trusted stage placement — a bad solve degrades
        # to (at worst) the open-loop position, never a wild throw.
        bound = max(4.0, self._max_shift_px())
        opt = np.clip(opt, nominal - bound, nominal + bound)
        max_corr = float(np.abs(opt - nominal).max()) if opt.size else 0.0
        self._reblend_at_positions(opt)
        self._optimized_positions = opt
        logger.info(
            f"Mosaic global optimize: {len(edges)} overlaps registered, "
            f"max per-tile correction {max_corr:.1f}px")
        return self.composite, len(edges), max_corr

    def apply_registration_from(self, reference) -> bool:
        """Adopt ``reference``'s solved tile positions and global shift.

        v7.19 — the reason tile-major multi-channel acquisition is worth its
        cost. Registering each channel INDEPENDENTLY would give every channel
        its own solved positions and its own global shift, so the channels
        would not overlay each other any better than they do when captured
        minutes apart. Solving once on a reference channel and replaying it
        here is what puts them in register.

        Safe because the channels share everything the solve depends on: one
        raster grid, one field of view and one mosaic scale, all frozen once
        per run — so tile ``i`` covers the same ground in every channel and the
        ``(tw, th)`` boxes are identical. Refuses (returns False, changing
        nothing) if the tile counts disagree, which is the only way that
        assumption could be violated.

        Deliberately public: the workflow must not reach into
        ``_reblend_at_positions``.
        """
        opt = getattr(reference, "_optimized_positions", None)
        if opt is None:
            # The reference's own solve was a no-op (too few confident
            # overlaps). Nothing to share — both keep the trusted open-loop
            # stage placement, which is the correct degradation.
            self._global_shift_um = getattr(reference, "_global_shift_um",
                                            self._global_shift_um)
            return False
        mine = getattr(self, "_reorient_tiles", None)
        if not mine or len(mine) != len(opt):
            logger.warning(
                "Mosaic registration not shared: %d tiles here vs %d solved — "
                "this channel keeps its own stage placement.",
                len(mine or []), len(opt))
            return False
        self._reblend_at_positions(opt)
        self._optimized_positions = opt
        # The global shift is applied to the EXTENT, not the pixels, so it must
        # travel with the positions or the channels would be offset in world
        # coordinates even though their pixels line up.
        self._global_shift_um = getattr(reference, "_global_shift_um",
                                        self._global_shift_um)
        return True

    def _reblend_at_positions(self, positions) -> np.ndarray | None:
        """Re-blend every retained canvas-res tile at the given (N, 2) top-left
        pixel positions. Shares the zero-then-accumulate pattern with
        ``reblend_reoriented`` (which re-blends at the ORIGINAL positions)."""
        if self._composite is None or self._weight_sum is None:
            return self.composite
        self._composite[...] = 0.0
        self._weight_sum[...] = 0.0
        if self._display_cache is not None:
            self._display_cache[...] = 0
        for (resized, _px, _py, tw, th), pos in zip(
                self._reorient_tiles, positions):
            try:
                # ``pos`` is the solver's top-left for the SAME (tw, th) box.
                self._place_from_box(
                    resized, int(round(float(pos[0]))),
                    int(round(float(pos[1]))), int(tw), int(th))
            except Exception as e:
                logger.debug(f"reblend-at tile skipped: {e}")
        return self.composite

    # ── Full stitching with registration + global optimization ─────

    def _ensure_composite(self):
        """Ensure composite canvas exists; auto-initialize from frame records if needed.

        Called by build_mosaic when frames were added via add_frame() without
        a prior generate_raster_positions() call.
        """
        if self._composite is not None:
            return

        if not self._records or not CV2_AVAILABLE:
            return

        # Derive bounds from stage positions
        xs = [r.stage_x_um for r in self._records]
        ys = [r.stage_y_um for r in self._records]
        fov_w, fov_h = self._oriented_fov_um()
        bounds = (
            min(xs) - fov_w / 2.0,
            min(ys) - fov_h / 2.0,
            max(xs) + fov_w / 2.0,
            max(ys) + fov_h / 2.0,
        )
        self._init_composite(bounds)

        # Blend all existing frames into the fresh canvas (skip any freed in
        # frame-light mode — defensive; the frame-light scan never re-blends).
        for rec in self._records:
            if rec.frame is None:
                continue
            self._blend_tile_to_composite(rec)

    def build_mosaic(self) -> np.ndarray | None:
        """Build the final mosaic with pairwise registration and global optimization.

        Pipeline:
          1. Compute nominal tile positions from stage coordinates
          2. Pairwise phase correlation on overlap regions between neighbors
          3. Global least-squares optimization of all tile positions
          4. Re-blend all tiles at optimized positions with feathering

        Returns:
            BGR numpy array of the composite, or None if no frames.
        """
        if not self._records or not CV2_AVAILABLE:
            return None

        n_tiles = len(self._records)

        # If only 1 tile or no skimage, fall back to stage-coordinate blending
        if n_tiles == 1 or not SKIMAGE_AVAILABLE:
            self._ensure_composite()
            return self.composite

        # For very large grids (>200 tiles), phase correlation is too slow.
        # Motorized stage coordinates are reliable enough — use feathered
        # blending from stitch_incremental which already looks good.
        MAX_TILES_FOR_REGISTRATION = 200
        if n_tiles > MAX_TILES_FOR_REGISTRATION:
            logger.info(
                f"Skipping registration for {n_tiles} tiles (>{MAX_TILES_FOR_REGISTRATION}); "
                f"using stage-coordinate placement with feathered blending"
            )
            return self.composite

        fov_w_um, fov_h_um = self._oriented_fov_um()

        # Compute stage bounds for canvas
        xs = [r.stage_x_um for r in self._records]
        ys = [r.stage_y_um for r in self._records]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        canvas_w_um = (max_x - min_x) + fov_w_um
        canvas_h_um = (max_y - min_y) + fov_h_um
        scale = self._target_mosaic_px / max(canvas_w_um, canvas_h_um, 1.0)
        origin_x = min_x - fov_w_um / 2.0
        origin_y = min_y - fov_h_um / 2.0

        tile_w = max(1, int(fov_w_um * scale))
        tile_h = max(1, int(fov_h_um * scale))

        # Step 1: Nominal positions in pixels
        nominal = np.zeros((n_tiles, 2), dtype=np.float64)
        for i, rec in enumerate(self._records):
            nominal[i, 0] = (rec.stage_x_um - fov_w_um / 2.0 - origin_x) * scale
            nominal[i, 1] = (rec.stage_y_um - fov_h_um / 2.0 - origin_y) * scale

        # Step 2: Resize all tiles to mosaic resolution + convert to gray
        tiles_resized = []
        tiles_gray = []
        for rec in self._records:
            try:
                resized = cv2.resize(rec.frame, (tile_w, tile_h),
                                     interpolation=cv2.INTER_AREA)
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            except Exception:
                resized = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
                gray = np.zeros((tile_h, tile_w), dtype=np.uint8)
            tiles_resized.append(resized)
            tiles_gray.append(gray)

        # Step 3: Build neighbor graph
        # If we have fewer tiles than the full grid (e.g. partial scan),
        # recompute grid dims from actual tile count
        full_grid = self._grid_cols * self._grid_rows
        if self._grid_cols > 0 and n_tiles == full_grid:
            grid_cols = self._grid_cols
            grid_rows = self._grid_rows
        elif self._grid_cols > 0 and n_tiles < full_grid:
            grid_cols = min(self._grid_cols, n_tiles)
            grid_rows = max(1, (n_tiles + grid_cols - 1) // grid_cols)
        else:
            grid_cols = n_tiles
            grid_rows = 1
        edges = _build_neighbor_graph(n_tiles, grid_cols, grid_rows)

        # Step 4: Pairwise registration via phase correlation
        measured_offsets = []
        confidences = []
        overlap_frac = self._overlap

        for i, j, direction in edges:
            if i >= n_tiles or j >= n_tiles:
                measured_offsets.append((nominal[j, 0] - nominal[i, 0],
                                        nominal[j, 1] - nominal[i, 1]))
                confidences.append(0.0)
                continue

            # Expected offset from nominal positions
            nom_dx = nominal[j, 0] - nominal[i, 0]
            nom_dy = nominal[j, 1] - nominal[i, 1]

            # Extract overlap regions based on direction
            gray_a = tiles_gray[i]
            gray_b = tiles_gray[j]

            overlap_w = max(4, int(tile_w * overlap_frac * 1.5))
            overlap_h = max(4, int(tile_h * overlap_frac * 1.5))

            try:
                if direction == 'h':
                    # Horizontal neighbor: overlap on right edge of A, left edge of B
                    if nom_dx > 0:  # B is to the right
                        region_a = gray_a[:, -overlap_w:]
                        region_b = gray_b[:, :overlap_w]
                    else:  # B is to the left
                        region_a = gray_a[:, :overlap_w]
                        region_b = gray_b[:, -overlap_w:]
                else:  # direction == 'v'
                    # Vertical neighbor: overlap on bottom of A, top of B
                    if nom_dy > 0:  # B is below
                        region_a = gray_a[-overlap_h:, :]
                        region_b = gray_b[:overlap_h, :]
                    else:  # B is above
                        region_a = gray_a[:overlap_h, :]
                        region_b = gray_b[-overlap_h:, :]

                dy, dx, conf = _phase_correlate_overlap(region_a, region_b)

                # Convert correlation shift to absolute offset
                # The measured offset = nominal offset + correction
                measured_offsets.append((nom_dx + dx, nom_dy + dy))
                confidences.append(conf)

            except Exception as e:
                logger.debug(f"Phase correlation failed for edge ({i},{j}): {e}")
                measured_offsets.append((nom_dx, nom_dy))
                confidences.append(0.0)

        # Step 5: Global optimization
        if any(c > 0.1 for c in confidences):
            optimized = _global_optimize_positions(
                n_tiles, nominal, edges, measured_offsets, confidences,
                min_confidence=0.1,
            )
        else:
            # No confident registrations — the incremental composite
            # (stage-coordinate placement with feathering) is already optimal.
            logger.info("No confident registrations; returning incremental composite")
            self._ensure_composite()
            return self.composite

        # Step 6: Re-blend at optimized positions with feathering
        canvas_w = max(1, int(canvas_w_um * scale))
        canvas_h = max(1, int(canvas_h_um * scale))

        composite = np.zeros((canvas_h, canvas_w, 3), dtype=np.float64)
        weight_sum = np.zeros((canvas_h, canvas_w), dtype=np.float64)

        overlap_px = max(1, int(tile_w * overlap_frac))
        feather = _compute_feather_weights(tile_h, tile_w, overlap_px)

        for i, rec in enumerate(self._records):
            px = int(optimized[i, 0])
            py = int(optimized[i, 1])

            x1 = max(0, px)
            y1 = max(0, py)
            x2 = min(canvas_w, px + tile_w)
            y2 = min(canvas_h, py + tile_h)

            sx1 = x1 - px
            sy1 = y1 - py
            sx2 = sx1 + (x2 - x1)
            sy2 = sy1 + (y2 - y1)

            if x2 <= x1 or y2 <= y1:
                continue

            tile_crop = tiles_resized[i][sy1:sy2, sx1:sx2].astype(np.float64)
            w_crop = feather[sy1:sy2, sx1:sx2]

            for c in range(3):
                composite[y1:y2, x1:x2, c] += tile_crop[:, :, c] * w_crop
            weight_sum[y1:y2, x1:x2] += w_crop

            # Update record with optimized position
            rec.refined_x_px = float(px + tile_w / 2.0)
            rec.refined_y_px = float(py + tile_h / 2.0)

        # Normalize
        mask = weight_sum > 0
        result = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        for c in range(3):
            ch = composite[:, :, c].copy()
            ch[mask] = (ch[mask] / weight_sum[mask]).clip(0, 255)
            result[:, :, c] = ch.astype(np.uint8)

        # Update display cache with the registered result
        self._display_cache = result

        n_registered = sum(1 for c in confidences if c > 0.1)
        logger.info(
            f"Mosaic built: {canvas_w}×{canvas_h} px from {n_tiles} tiles, "
            f"{n_registered}/{len(edges)} edges registered"
        )
        return result

    # ── Affine fitting ─────────────────────────────────────────────

    def fit_affine(
        self,
        predicted_positions: dict[str, tuple[float, float]],
        min_confidence: float = 0.3,
    ) -> AffineCalibration:
        """Fit rotation + scale + translation from predicted vs detected positions.

        Uses only frames where well detection succeeded (above *min_confidence*).
        Fits a rigid similarity transform (4 DOF: rotation, uniform scale, tx, ty)
        using least-squares.

        Args:
            predicted_positions: Dict mapping well_name → (x_um, y_um) predicted.
            min_confidence: Minimum detection confidence to include a point.

        Returns:
            AffineCalibration with fitted parameters.
        """
        # Collect matched point pairs: (predicted, detected_absolute)
        pred_pts = []
        det_pts = []

        for rec in self._records:
            if rec.detected_offset_um is None or rec.confidence < min_confidence:
                continue
            if rec.well_name not in predicted_positions:
                continue

            px, py = predicted_positions[rec.well_name]
            # Detected absolute = stage position + detection offset
            dx, dy = rec.detected_offset_um
            det_x = rec.stage_x_um + dx
            det_y = rec.stage_y_um + dy

            pred_pts.append((px, py))
            det_pts.append((det_x, det_y))

        n = len(pred_pts)
        if n < 2:
            logger.warning(f"Affine fit: only {n} matched points, need ≥2. Returning identity.")
            return AffineCalibration(num_points=n)

        pred_arr = np.array(pred_pts)  # (N, 2)
        det_arr = np.array(det_pts)    # (N, 2)

        # Compute centroids
        pred_center = pred_arr.mean(axis=0)
        det_center = det_arr.mean(axis=0)

        # Center both point sets
        p = pred_arr - pred_center
        d = det_arr - det_center

        # Solve for rotation + scale using Procrustes
        A_rows = []
        b_rows = []
        for i in range(n):
            px_i, py_i = p[i]
            dx_i, dy_i = d[i]
            A_rows.append([px_i, -py_i, 1.0, 0.0])
            A_rows.append([py_i,  px_i, 0.0, 1.0])
            b_rows.append(dx_i)
            b_rows.append(dy_i)

        A = np.array(A_rows)
        b_vec = np.array(b_rows)

        result, residuals, rank, sv = np.linalg.lstsq(A, b_vec, rcond=None)
        a, b, tx, ty = result

        scale = math.sqrt(a * a + b * b)
        rotation_deg = math.degrees(math.atan2(b, a))

        total_tx = det_center[0] - pred_center[0] + tx
        total_ty = det_center[1] - pred_center[1] + ty

        # Compute RMS residual
        corrected = np.column_stack([
            a * p[:, 0] - b * p[:, 1] + tx,
            b * p[:, 0] + a * p[:, 1] + ty,
        ])
        residual_rms = float(np.sqrt(np.mean((corrected - d) ** 2)))

        cal = AffineCalibration(
            rotation_deg=rotation_deg,
            scale=scale,
            translation_um=(total_tx, total_ty),
            center_um=(float(pred_center[0]), float(pred_center[1])),
            num_points=n,
            residual_um=residual_rms,
        )
        logger.info(
            f"Affine fit: {n} points, rotation={rotation_deg:.3f}°, "
            f"scale={scale:.5f}, translation=({total_tx:.1f}, {total_ty:.1f}) µm, "
            f"RMS residual={residual_rms:.1f} µm"
        )
        return cal

    # ── High-level build ───────────────────────────────────────────

    def build(
        self,
        predicted_positions: dict[str, tuple[float, float]] | None = None,
        min_confidence: float = 0.3,
    ) -> MosaicResult:
        """Build final mosaic with registration + optimization, and fit affine.

        Args:
            predicted_positions: Dict of predicted well positions for affine fitting.
                                 If None, only the mosaic image is built.
            min_confidence: Minimum detection confidence for affine fitting.

        Returns:
            MosaicResult with mosaic image, calibration, and statistics.
        """
        mosaic_img = self.build_mosaic()

        # Collect detected absolute positions
        detected = {}
        for rec in self._records:
            if rec.detected_offset_um is not None and rec.confidence >= min_confidence:
                dx, dy = rec.detected_offset_um
                detected[rec.well_name] = (
                    rec.stage_x_um + dx,
                    rec.stage_y_um + dy,
                )

        # Fit affine if we have predictions
        cal = AffineCalibration()
        if predicted_positions is not None and len(detected) >= 2:
            cal = self.fit_affine(predicted_positions, min_confidence)

        return MosaicResult(
            mosaic_image=mosaic_img,
            calibration=cal,
            frames_captured=len(self._records),
            frames_detected=len(detected),
            detected_positions=detected,
        )
