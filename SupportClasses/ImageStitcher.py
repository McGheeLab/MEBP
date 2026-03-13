"""
ImageStitcher.py — Stage-coordinate-based image stitcher.

v7.3.3: Builds large composite images of wells by placing camera frames
at their known stage positions. No feature matching needed — stage
coordinates provide exact alignment.

Coordinate system:
    - Each tile's position = stage (x_um, y_um) at frame center
    - Composite origin = top-left of bounding box of all tiles
    - Pixel scale = camera's micron_per_pixel
    - pixel_to_stage() and stage_to_pixel() enable click → coordinate mapping
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class Tile:
    """A single camera frame with its stage position."""
    frame: np.ndarray              # BGR image (H, W, 3)
    stage_x_um: float              # Stage X at frame center (µm)
    stage_y_um: float              # Stage Y at frame center (µm)
    timestamp: float = 0.0         # Capture time (monotonic)
    index: int = 0                 # Order of capture


class StitchedImage:
    """A composite image built from camera frames at known stage positions.

    Usage:
        stitcher = StitchedImage(micron_per_pixel=1.67)
        stitcher.add_tile(frame, stage_x_um=5000.0, stage_y_um=3000.0)
        stitcher.add_tile(frame2, stage_x_um=5500.0, stage_y_um=3000.0)
        composite = stitcher.get_composite()
        x_um, y_um = stitcher.pixel_to_stage(px=100, py=200)
    """

    def __init__(self, micron_per_pixel: float):
        if micron_per_pixel <= 0:
            raise ValueError(f"micron_per_pixel must be positive, got {micron_per_pixel}")

        self.um_per_px = micron_per_pixel
        self._tiles: list[Tile] = []
        self._composite: Optional[np.ndarray] = None
        self._origin_x_um: float = 0.0   # Stage X of composite top-left corner
        self._origin_y_um: float = 0.0   # Stage Y of composite top-left corner
        self._dirty: bool = True
        self._tile_counter: int = 0

    # ── Tile management ──────────────────────────────────────────

    def add_tile(self, frame: np.ndarray, stage_x_um: float, stage_y_um: float,
                 timestamp: float = 0.0):
        """Add a camera frame captured at the given stage position.

        Args:
            frame: BGR numpy array (H, W, 3) from camera.
            stage_x_um: Stage X coordinate at frame center (µm).
            stage_y_um: Stage Y coordinate at frame center (µm).
            timestamp: Optional capture timestamp.
        """
        if frame is None or frame.ndim < 2:
            logger.warning("add_tile: invalid frame, skipping")
            return

        tile = Tile(
            frame=frame.copy(),
            stage_x_um=stage_x_um,
            stage_y_um=stage_y_um,
            timestamp=timestamp,
            index=self._tile_counter,
        )
        self._tiles.append(tile)
        self._tile_counter += 1
        self._dirty = True
        logger.debug(
            f"Tile {tile.index} added at ({stage_x_um:.1f}, {stage_y_um:.1f}) µm, "
            f"shape={frame.shape}")

    @property
    def tile_count(self) -> int:
        return len(self._tiles)

    # ── Composite generation ─────────────────────────────────────

    def get_composite(self) -> Optional[np.ndarray]:
        """Return the stitched composite image.

        Lazy rebuild: only recomputes if tiles have changed since last call.
        Returns None if no tiles have been added.
        """
        if not self._tiles:
            return None
        if self._dirty:
            self._rebuild_composite()
        return self._composite

    def _rebuild_composite(self):
        """Rebuild the composite image from all tiles."""
        if not self._tiles:
            self._composite = None
            self._dirty = False
            return

        # Compute bounding box in stage coordinates (µm)
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for tile in self._tiles:
            h, w = tile.frame.shape[:2]
            half_w_um = (w / 2) * self.um_per_px
            half_h_um = (h / 2) * self.um_per_px

            tile_left = tile.stage_x_um - half_w_um
            tile_right = tile.stage_x_um + half_w_um
            tile_top = tile.stage_y_um - half_h_um
            tile_bottom = tile.stage_y_um + half_h_um

            min_x = min(min_x, tile_left)
            max_x = max(max_x, tile_right)
            min_y = min(min_y, tile_top)
            max_y = max(max_y, tile_bottom)

        self._origin_x_um = min_x
        self._origin_y_um = min_y

        # Composite dimensions in pixels
        comp_w = int(math.ceil((max_x - min_x) / self.um_per_px))
        comp_h = int(math.ceil((max_y - min_y) / self.um_per_px))

        # Clamp to reasonable size (max 16384 x 16384)
        max_dim = 16384
        if comp_w > max_dim or comp_h > max_dim:
            logger.warning(
                f"Composite too large ({comp_w}x{comp_h}), clamping to {max_dim}")
            comp_w = min(comp_w, max_dim)
            comp_h = min(comp_h, max_dim)

        # Allocate composite (black background)
        self._composite = np.zeros((comp_h, comp_w, 3), dtype=np.uint8)

        # Place tiles in order (later tiles overwrite earlier ones in overlap)
        for tile in self._tiles:
            h, w = tile.frame.shape[:2]
            half_w_um = (w / 2) * self.um_per_px
            half_h_um = (h / 2) * self.um_per_px

            # Pixel offset of tile top-left in composite
            px_x = int(round((tile.stage_x_um - half_w_um - self._origin_x_um)
                             / self.um_per_px))
            px_y = int(round((tile.stage_y_um - half_h_um - self._origin_y_um)
                             / self.um_per_px))

            # Clip to composite bounds
            src_x0 = max(0, -px_x)
            src_y0 = max(0, -px_y)
            dst_x0 = max(0, px_x)
            dst_y0 = max(0, px_y)
            src_x1 = min(w, comp_w - px_x)
            src_y1 = min(h, comp_h - px_y)
            dst_x1 = dst_x0 + (src_x1 - src_x0)
            dst_y1 = dst_y0 + (src_y1 - src_y0)

            if src_x1 > src_x0 and src_y1 > src_y0:
                self._composite[dst_y0:dst_y1, dst_x0:dst_x1] = \
                    tile.frame[src_y0:src_y1, src_x0:src_x1]

        self._dirty = False
        logger.info(
            f"Composite rebuilt: {comp_w}x{comp_h} px from {len(self._tiles)} tiles")

    # ── Coordinate conversion ────────────────────────────────────

    def stage_to_pixel(self, x_um: float, y_um: float) -> tuple[int, int]:
        """Convert stage coordinates (µm) to pixel coordinates in composite.

        Returns:
            (px, py) — pixel coordinates, may be outside composite bounds.
        """
        px = int(round((x_um - self._origin_x_um) / self.um_per_px))
        py = int(round((y_um - self._origin_y_um) / self.um_per_px))
        return (px, py)

    def pixel_to_stage(self, px: int, py: int) -> tuple[float, float]:
        """Convert composite pixel coordinates to stage coordinates (µm).

        Returns:
            (x_um, y_um) — stage coordinates.
        """
        x_um = self._origin_x_um + px * self.um_per_px
        y_um = self._origin_y_um + py * self.um_per_px
        return (x_um, y_um)

    def get_origin_um(self) -> tuple[float, float]:
        """Return the stage coordinates of the composite origin (top-left)."""
        return (self._origin_x_um, self._origin_y_um)

    def get_extent_um(self) -> tuple[float, float, float, float]:
        """Return the stage-coordinate bounding box of the composite.

        Returns:
            (min_x_um, min_y_um, max_x_um, max_y_um)
        """
        if self._composite is None:
            return (0.0, 0.0, 0.0, 0.0)
        h, w = self._composite.shape[:2]
        return (
            self._origin_x_um,
            self._origin_y_um,
            self._origin_x_um + w * self.um_per_px,
            self._origin_y_um + h * self.um_per_px,
        )

    # ── Utility ──────────────────────────────────────────────────

    def clear(self):
        """Remove all tiles and reset the stitcher."""
        self._tiles.clear()
        self._composite = None
        self._origin_x_um = 0.0
        self._origin_y_um = 0.0
        self._dirty = True
        self._tile_counter = 0
        logger.info("StitchedImage cleared")

    def get_composite_size_px(self) -> tuple[int, int]:
        """Return (width, height) of the composite in pixels, or (0,0)."""
        if self._composite is None:
            return (0, 0)
        h, w = self._composite.shape[:2]
        return (w, h)


# ── Scan pattern generation ──────────────────────────────────────

def generate_scan_pattern(
    well_center_x_um: float,
    well_center_y_um: float,
    well_diameter_um: float,
    frame_width_px: int,
    frame_height_px: int,
    um_per_px: float,
    overlap_fraction: float = 0.2,
) -> list[tuple[float, float]]:
    """Generate a serpentine raster scan pattern to cover a circular well.

    Args:
        well_center_x_um: Stage X of well center (µm).
        well_center_y_um: Stage Y of well center (µm).
        well_diameter_um: Well diameter (µm).
        frame_width_px: Camera frame width in pixels.
        frame_height_px: Camera frame height in pixels.
        um_per_px: Microns per pixel.
        overlap_fraction: Fraction of frame overlap between adjacent tiles
                          (0.0 = no overlap, 0.5 = 50% overlap).

    Returns:
        List of (stage_x_um, stage_y_um) positions for the scan.
    """
    frame_w_um = frame_width_px * um_per_px
    frame_h_um = frame_height_px * um_per_px

    step_x = frame_w_um * (1.0 - overlap_fraction)
    step_y = frame_h_um * (1.0 - overlap_fraction)

    if step_x <= 0 or step_y <= 0:
        logger.warning("Scan step size is zero or negative, returning center only")
        return [(well_center_x_um, well_center_y_um)]

    well_radius = well_diameter_um / 2.0

    # Grid bounds (centered on well)
    n_cols = max(1, int(math.ceil(well_diameter_um / step_x)))
    n_rows = max(1, int(math.ceil(well_diameter_um / step_y)))

    # Grid origin (top-left)
    grid_x0 = well_center_x_um - (n_cols - 1) * step_x / 2
    grid_y0 = well_center_y_um - (n_rows - 1) * step_y / 2

    positions = []
    for row in range(n_rows):
        y = grid_y0 + row * step_y

        # Serpentine: alternate direction each row
        cols = range(n_cols) if row % 2 == 0 else range(n_cols - 1, -1, -1)

        for col in cols:
            x = grid_x0 + col * step_x

            # Only include positions where center is inside the well circle
            dx = x - well_center_x_um
            dy = y - well_center_y_um
            if math.sqrt(dx * dx + dy * dy) <= well_radius:
                positions.append((x, y))

    if not positions:
        # Fallback: at least capture the center
        positions.append((well_center_x_um, well_center_y_um))

    logger.info(
        f"Scan pattern: {len(positions)} positions, "
        f"{n_cols}x{n_rows} grid, step=({step_x:.0f}, {step_y:.0f}) µm")
    return positions
