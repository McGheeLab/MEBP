"""
ImagePathPlanner.py — Image-stack-to-toolpath generator for MEBP.

Adapted from pathplan3.py (Toolpath_From_ImageStack). Core algorithm:
  1. Combine all pump images for a layer → binary mask
  2. For each raster row, find black-pixel edge segments in the mask
  3. Walk the path between edges; at each pixel check each pump image
  4. Emit waypoints at pump-state-change boundaries
  5. Repeat for each layer (alternating direction, closest-start reorder)

Input modes:
  A) TIFF stack — multi-frame TIFF; RGB split across pumps per frame
  B) Numbered sequences — e.g. color1_01.png, color1_02.png per pump
  C) Single image — one image per pump, repeated for N layers

Pump flow is proportional to greyscale intensity (within threshold):
  invert=True  (default): pixel 0 → 100%, pixel=threshold → 0%, above threshold → 0%
  invert=False:           pixel 0 → 0%, pixel=threshold → 100%, above threshold → 0%

Output: Nx7 array [x, y, z, p1, p2, p3, t] — microns and seconds.
"""

from __future__ import annotations

import csv
import logging
import math
import os
import re as _re
from pathlib import Path
from typing import Any

import numpy as np

try:
    from PIL import Image
except ImportError:
    Image = None

logger = logging.getLogger(__name__)

MAX_PUMPS = 3

# Waypoint metadata: is_travel flag stored alongside data
# Travel waypoints have pumps OFF regardless of pixel state
TRAVEL_MARKER = -1.0  # Sentinel flow value marking a travel move


class ImagePathPlanner:
    """Generate raster toolpaths from greyscale pump image stacks."""

    def __init__(
        self,
        threshold: int = 128,
        tool_diameter: float = 300.0,
        path_overlap: float = 0.8,
        pixels_per_micron: float = 0.09,
        base_feedrate: float = 1000.0,
        corner_slowdown_factor: float = 0.5,
        corner_angle_threshold: float = 120.0,
        flow_factor: float = 0.01,
        initial_z: float = 0.0,
        z_increment: float = 10.0,
        invert: bool = True,
    ):
        self.threshold = threshold
        self.tool_diameter = tool_diameter
        self.path_overlap = path_overlap
        self.pixels_per_micron = pixels_per_micron
        self.base_feedrate = base_feedrate
        self.corner_slowdown_factor = corner_slowdown_factor
        self.corner_angle_threshold = corner_angle_threshold
        self.flow_factor = flow_factor
        self.initial_z = initial_z
        self.z_increment = z_increment
        self.invert = invert

        # Per-pump layer stacks: pump_layers[pump_idx] = [layer0, layer1, ...]
        self._pump_layers: list[list[np.ndarray]] = [[] for _ in range(MAX_PUMPS)]
        self._height: int = 0
        self._width: int = 0
        self._num_layers: int = 0

        # Output
        self.waypoints: list[tuple] = []
        self.pump_states_all: list[list[float]] = []
        self.pump_colors: list[tuple] = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]

    # ══════════════════════════════════════════════════════════════
    #  IMAGE LOADING — THREE MODES
    # ══════════════════════════════════════════════════════════════

    def clear(self):
        """Reset all loaded images and output."""
        self._pump_layers = [[] for _ in range(MAX_PUMPS)]
        self._height = 0
        self._width = 0
        self._num_layers = 0
        self.waypoints.clear()
        self.pump_states_all.clear()

    def _validate_image(self, pixels: np.ndarray, label: str = "image"):
        if self._height == 0:
            self._height, self._width = pixels.shape
        elif pixels.shape != (self._height, self._width):
            raise ValueError(
                f"{label} size {pixels.shape[1]}x{pixels.shape[0]} "
                f"doesn't match {self._width}x{self._height}"
            )

    # ── Mode A: TIFF Stack ────────────────────────────────────────

    def load_tiff_stack(self, tiff_path: str, pump_index: int | None = None):
        """
        Load a multi-frame TIFF.

        If pump_index is None and TIFF has RGB frames, channels are
        split across pumps (R→P1, G→P2, B→P3), each frame = one layer.
        If pump_index is specified (0-2), all frames become greyscale
        layers for that single pump.
        """
        if Image is None:
            raise RuntimeError("Pillow required: pip install Pillow")
        path = Path(tiff_path)
        if not path.exists():
            raise FileNotFoundError(f"TIFF not found: {path}")

        img = Image.open(path)
        frame_count = getattr(img, 'n_frames', 1)
        logger.info(f"TIFF: {path.name}, {frame_count} frame(s), mode={img.mode}")

        for frame_idx in range(frame_count):
            img.seek(frame_idx)
            frame = img.copy()

            if pump_index is not None:
                grey = np.array(frame.convert("L"), dtype=np.uint8)
                self._validate_image(grey, f"TIFF frame {frame_idx}")
                self._pump_layers[pump_index].append(grey)
            else:
                if frame.mode in ("RGB", "RGBA"):
                    rgb = np.array(frame.convert("RGB"), dtype=np.uint8)
                    self._validate_image(rgb[:, :, 0], f"TIFF frame {frame_idx}")
                    for ch in range(min(3, rgb.shape[2])):
                        self._pump_layers[ch].append(rgb[:, :, ch])
                else:
                    grey = np.array(frame.convert("L"), dtype=np.uint8)
                    self._validate_image(grey, f"TIFF frame {frame_idx}")
                    self._pump_layers[0].append(grey)

        self._sync_num_layers()
        logger.info(
            f"Loaded TIFF: {frame_count} frames, "
            f"layers/pump: {[len(s) for s in self._pump_layers]}"
        )

    # ── Mode B: Numbered Sequence ─────────────────────────────────

    def load_image_sequence(self, paths: list[str], pump_index: int):
        """Load ordered image files as layers for one pump."""
        if Image is None:
            raise RuntimeError("Pillow required: pip install Pillow")
        if pump_index < 0 or pump_index >= MAX_PUMPS:
            raise ValueError(f"pump_index must be 0-{MAX_PUMPS - 1}")
        for i, p in enumerate(paths):
            path = Path(p)
            if not path.exists():
                raise FileNotFoundError(f"Image not found: {path}")
            grey = np.array(Image.open(path).convert("L"), dtype=np.uint8)
            self._validate_image(grey, f"P{pump_index + 1} layer {i}")
            self._pump_layers[pump_index].append(grey)
        self._sync_num_layers()
        logger.info(f"Loaded {len(paths)} layers for pump {pump_index + 1}")

    # ── Mode C: Single Image × N Layers ──────────────────────────

    def load_single_image(self, path: str, pump_index: int, num_layers: int = 1):
        """Load one image, replicate for N layers on one pump."""
        if Image is None:
            raise RuntimeError("Pillow required: pip install Pillow")
        if pump_index < 0 or pump_index >= MAX_PUMPS:
            raise ValueError(f"pump_index must be 0-{MAX_PUMPS - 1}")
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Image not found: {p}")
        grey = np.array(Image.open(p).convert("L"), dtype=np.uint8)
        self._validate_image(grey, f"P{pump_index + 1}")
        for _ in range(max(1, num_layers)):
            self._pump_layers[pump_index].append(grey.copy())
        self._sync_num_layers()
        logger.info(
            f"Loaded single image for pump {pump_index + 1}, "
            f"replicated to {num_layers} layers"
        )

    def _sync_num_layers(self):
        counts = [len(stack) for stack in self._pump_layers]
        self._num_layers = max(counts) if any(counts) else 0

    def _get_pump_image(self, pump_idx: int, layer_idx: int) -> np.ndarray | None:
        stack = self._pump_layers[pump_idx]
        if not stack:
            return None
        return stack[min(layer_idx, len(stack) - 1)]

    # ══════════════════════════════════════════════════════════════
    #  HELPERS
    # ══════════════════════════════════════════════════════════════

    def _microns_per_pixel(self) -> float:
        return 1.0 / max(self.pixels_per_micron, 1e-9)

    @staticmethod
    def _distance(p1, p2) -> float:
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def _angle_between_vectors(v1, v2) -> float:
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
        if mag1 * mag2 == 0:
            return 0.0
        cos_a = max(-1.0, min(1.0, dot / (mag1 * mag2)))
        return math.degrees(math.acos(cos_a))

    def _pixel_to_flow(self, value: int) -> float:
        """
        Convert pixel intensity to flow fraction 0.0-1.0.

        THRESHOLD-ALIGNED: pixels ABOVE threshold always return 0.0.
        Pixels at or below threshold are scaled proportionally.
        This ensures flow agrees with the binary state detection.
        """
        if value > self.threshold:
            return 0.0  # Above threshold = pump OFF, zero flow

        # At or below threshold: proportional flow
        if self.threshold == 0:
            return 1.0  # Edge case: threshold=0, pixel=0 → full flow

        if self.invert:
            # pixel=0 → 1.0 (max flow), pixel=threshold → 0.0
            return 1.0 - (value / self.threshold)
        else:
            # pixel=0 → 0.0, pixel=threshold → 1.0
            return value / self.threshold

    def _get_pump_flows_at_pixel(
        self, pump_images: list[np.ndarray | None], x_px: int, y_px: int,
    ) -> list[float]:
        """Get flow fraction for each pump at a pixel position."""
        x_px = max(0, min(x_px, self._width - 1))
        y_px = max(0, min(y_px, self._height - 1))
        flows = []
        for img in pump_images:
            if img is not None:
                flows.append(self._pixel_to_flow(int(img[y_px, x_px])))
            else:
                flows.append(0.0)
        return flows

    def _get_pump_states_at_pixel(
        self, pump_images: list[np.ndarray | None], x_px: int, y_px: int,
    ) -> list[bool]:
        """Binary on/off for each pump (for state-change detection)."""
        x_px = max(0, min(x_px, self._width - 1))
        y_px = max(0, min(y_px, self._height - 1))
        states = []
        for img in pump_images:
            if img is not None:
                states.append(int(img[y_px, x_px]) <= self.threshold)
            else:
                states.append(False)
        return states

    def _get_pump_flows_mic(self, pump_images, x_mic, y_mic):
        um_per_px = self._microns_per_pixel()
        x_px = int(round(x_mic / um_per_px))
        y_px = int(round(y_mic / um_per_px))
        return self._get_pump_flows_at_pixel(pump_images, x_px, y_px)

    # ══════════════════════════════════════════════════════════════
    #  COMBINED MASK (Step 1)
    # ══════════════════════════════════════════════════════════════

    def _build_combined_mask(self, pump_images: list[np.ndarray | None]) -> np.ndarray:
        """
        Combine all pump images into a single binary mask.
        Any pixel <= threshold in ANY pump image → active in mask.
        """
        mask = np.full((self._height, self._width), 255, dtype=np.uint8)
        for img in pump_images:
            if img is not None:
                mask = np.minimum(mask, img)
        return mask

    # ══════════════════════════════════════════════════════════════
    #  LAYER TOOLPATH (Steps 2-4)
    # ══════════════════════════════════════════════════════════════

    def _generate_layer_segments(
        self, mask: np.ndarray, pump_images: list[np.ndarray | None],
    ) -> list[list[tuple]]:
        """
        Generate toolpath segments for one layer.

        Returns a list of SEGMENTS, where each segment is a list of
        (x_um, y_um) points. Segments are separate print paths;
        travel moves happen BETWEEN segments.

        Step 2: find edge pixels per raster row
        Step 3: create path between edges
        Step 4: emit waypoints at pump-state-change boundaries
        """
        step_over = self.tool_diameter * self.path_overlap
        px_per_pass = max(1, int(math.ceil(step_over * self.pixels_per_micron)))
        um_per_px = self._microns_per_pixel()

        all_segments: list[list[tuple]] = []
        direction = 1
        y_px = 0

        while y_px < self._height:
            row = mask[y_px, :]
            black_idx = np.where(row <= self.threshold)[0]

            if black_idx.size == 0:
                y_px += px_per_pass
                direction *= -1
                continue

            # Find contiguous black segments (left→right)
            row_segments = []
            start = black_idx[0]
            for i in range(1, len(black_idx)):
                if black_idx[i] > black_idx[i - 1] + 1:
                    row_segments.append((start, black_idx[i - 1]))
                    start = black_idx[i]
            row_segments.append((start, black_idx[-1]))

            # Reverse for right-to-left passes
            if direction == -1:
                row_segments = row_segments[::-1]
                row_segments = [(e, s) for (s, e) in row_segments]

            y_um = y_px * um_per_px

            # Walk each segment, detect pump state changes
            for (seg_start, seg_end) in row_segments:
                if direction == 1:
                    px_range = list(range(seg_start, seg_end + 1))
                else:
                    px_range = list(range(seg_start, seg_end - 1, -1))

                if not px_range:
                    continue

                segment_points: list[tuple] = []

                # First pixel in segment
                prev_states = self._get_pump_states_at_pixel(
                    pump_images, px_range[0], y_px
                )
                segment_points.append((px_range[0] * um_per_px, y_um))

                # Walk pixels — emit waypoint at every state change
                for idx in range(1, len(px_range)):
                    cur_px = px_range[idx]
                    cur_states = self._get_pump_states_at_pixel(
                        pump_images, cur_px, y_px
                    )

                    if cur_states != prev_states:
                        # End of previous state region
                        prev_px = px_range[idx - 1]
                        prev_x = prev_px * um_per_px
                        if segment_points[-1] != (prev_x, y_um):
                            segment_points.append((prev_x, y_um))

                        # Start of new state region
                        cur_x = cur_px * um_per_px
                        segment_points.append((cur_x, y_um))
                        prev_states = cur_states

                # Always emit segment endpoint
                last_x = px_range[-1] * um_per_px
                if segment_points[-1] != (last_x, y_um):
                    segment_points.append((last_x, y_um))

                if len(segment_points) >= 2:
                    all_segments.append(segment_points)

            y_px += px_per_pass
            direction *= -1

        return all_segments

    # ══════════════════════════════════════════════════════════════
    #  WAYPOINT COMPUTATION (with travel-move awareness)
    # ══════════════════════════════════════════════════════════════

    def _compute_waypoints_from_segments(
        self,
        segments: list[list[tuple]],
        pump_images: list[np.ndarray | None],
        z_pos: float,
        time_offset: float = 0.0,
    ) -> tuple[list[tuple], list[list[float]]]:
        """
        Convert path segments into timed waypoints with pump displacements.

        TRAVEL MOVES between segments have all pump flows set to 0.0.
        Print moves within segments use proportional flow from greyscale.
        """
        if not segments:
            return [], []

        waypoints: list[tuple] = []
        pump_states: list[list[float]] = []
        pump_disp = [0.0] * MAX_PUMPS
        current_time = time_offset
        last_point = None

        for seg_idx, segment in enumerate(segments):
            if not segment:
                continue

            # ── Travel move to segment start ──────────────────────
            if last_point is not None:
                seg_start = segment[0]
                travel_dist = self._distance(last_point, seg_start)

                if travel_dist > 0:
                    # Emit travel endpoint with ZERO flow
                    travel_dt = travel_dist / max(self.base_feedrate, 1e-9)
                    current_time += travel_dt

                    # Travel waypoint: pumps OFF
                    waypoints.append((
                        seg_start[0], seg_start[1], z_pos,
                        *pump_disp, current_time,
                    ))
                    pump_states.append([0.0] * MAX_PUMPS)

            # ── Print moves within segment ────────────────────────
            for pt_idx, point in enumerate(segment):
                cur_flows = self._get_pump_flows_mic(
                    pump_images, point[0], point[1]
                )

                if pt_idx == 0 and last_point is None:
                    # Very first waypoint ever
                    waypoints.append((
                        point[0], point[1], z_pos,
                        *pump_disp, current_time,
                    ))
                    pump_states.append(cur_flows)
                elif pt_idx == 0:
                    # Segment start (travel waypoint already emitted above)
                    # Update flow state for this point
                    if waypoints:
                        pump_states[-1] = cur_flows
                else:
                    # Within-segment point
                    prev_point = segment[pt_idx - 1]
                    dist = self._distance(prev_point, point)

                    # Corner slowdown
                    if pt_idx < len(segment) - 1:
                        next_pt = segment[pt_idx + 1]
                        v_in = (point[0] - prev_point[0], point[1] - prev_point[1])
                        v_out = (next_pt[0] - point[0], next_pt[1] - point[1])
                        angle = self._angle_between_vectors(v_in, v_out)
                        if angle < self.corner_angle_threshold:
                            feedrate = self.base_feedrate * self.corner_slowdown_factor
                        else:
                            feedrate = self.base_feedrate
                    else:
                        feedrate = self.base_feedrate

                    dt = dist / max(feedrate, 1e-9)
                    current_time += dt

                    # Accumulate pump displacement (proportional flow)
                    for j in range(MAX_PUMPS):
                        if cur_flows[j] > 0:
                            pump_disp[j] += cur_flows[j] * dist * self.flow_factor

                    waypoints.append((
                        point[0], point[1], z_pos,
                        *pump_disp, current_time,
                    ))
                    pump_states.append(cur_flows)

                last_point = point

        return waypoints, pump_states

    def _reorder_segments_to_closest(
        self, segments: list[list[tuple]], last_endpoint: tuple,
    ) -> list[list[tuple]]:
        """Reorder segments so that first one starts closest to last_endpoint."""
        if not segments or last_endpoint is None:
            return segments
        # Only reorder the first segment's start; keep overall raster order
        # (full reorder would break raster logic)
        dists = []
        for seg in segments:
            if seg:
                dists.append(self._distance(last_endpoint, seg[0]))
            else:
                dists.append(float('inf'))
        min_idx = int(np.argmin(dists))
        return segments[min_idx:] + segments[:min_idx]

    # ══════════════════════════════════════════════════════════════
    #  MAIN ENTRY: GENERATE ALL LAYERS
    # ══════════════════════════════════════════════════════════════

    def generate_toolpath(self) -> np.ndarray:
        """
        Generate toolpaths for all loaded layers.

        Returns Nx7 numpy array [x, y, z, p1, p2, p3, t].
        """
        if self._num_layers == 0:
            raise ValueError("No images loaded.")

        self.waypoints.clear()
        self.pump_states_all.clear()
        z_pos = self.initial_z
        last_endpoint = None

        for layer_idx in range(self._num_layers):
            pump_images = [
                self._get_pump_image(j, layer_idx) for j in range(MAX_PUMPS)
            ]

            # Step 1: combined mask
            mask = self._build_combined_mask(pump_images)

            # Steps 2-4: raster within mask → list of segments
            segments = self._generate_layer_segments(mask, pump_images)

            # Alternate direction on odd layers
            if layer_idx % 2 == 1:
                # Reverse segment order and reverse points within each segment
                segments = [seg[::-1] for seg in reversed(segments)]

            # Reorder to closest point from previous layer
            if layer_idx > 0 and last_endpoint is not None and segments:
                segments = self._reorder_segments_to_closest(
                    segments, last_endpoint
                )

            # Time offset
            time_offset = 0.0
            if self.waypoints:
                time_offset = self.waypoints[-1][-1]

            layer_wp, layer_states = self._compute_waypoints_from_segments(
                segments, pump_images, z_pos, time_offset
            )

            self.waypoints.extend(layer_wp)
            self.pump_states_all.extend(layer_states)

            if self.waypoints:
                last_endpoint = (self.waypoints[-1][0], self.waypoints[-1][1])

            z_pos += self.z_increment

        traj = np.array(self.waypoints, dtype=np.float64) if self.waypoints else np.zeros((0, 7))
        logger.info(
            f"Generated {len(self.waypoints)} waypoints across "
            f"{self._num_layers} layers"
            + (f", time={self.waypoints[-1][-1]:.1f}s" if self.waypoints else "")
        )
        return traj

    # ══════════════════════════════════════════════════════════════
    #  OUTPUT
    # ══════════════════════════════════════════════════════════════

    @property
    def trajectory(self) -> np.ndarray:
        if not self.waypoints:
            return np.zeros((0, 7), dtype=np.float64)
        return np.array(self.waypoints, dtype=np.float64)

    def save_csv(self, output_path: str) -> None:
        """Save waypoints as CSV: x,y,z,p1,p2,p3,t."""
        if not self.waypoints:
            raise ValueError("No waypoints. Call generate_toolpath() first.")
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z", "p1", "p2", "p3", "t"])
            for wp in self.waypoints:
                writer.writerow([f"{v:.6f}" for v in wp])
        logger.info(f"Saved {len(self.waypoints)} waypoints -> {path}")

    def get_summary(self) -> dict[str, Any]:
        pump_disps = [0.0, 0.0, 0.0]
        if self.waypoints:
            last = self.waypoints[-1]
            pump_disps = [last[3], last[4], last[5]]
        return {
            "num_layers": self._num_layers,
            "layers_per_pump": [len(s) for s in self._pump_layers],
            "image_width_px": self._width,
            "image_height_px": self._height,
            "image_width_um": self._width * self._microns_per_pixel(),
            "image_height_um": self._height * self._microns_per_pixel(),
            "num_waypoints": len(self.waypoints),
            "estimated_time_s": self.waypoints[-1][-1] if self.waypoints else 0.0,
            "pump_displacements": pump_disps,
            "tool_diameter": self.tool_diameter,
            "line_spacing_um": self.tool_diameter * self.path_overlap,
            "z_increment": self.z_increment,
        }
