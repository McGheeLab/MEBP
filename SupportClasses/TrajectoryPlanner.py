"""
TrajectoryPlanner.py — Trajectory planning engine for MEBP v7.1.

Converts parametric PrintObject trajectories into executable,
time-interpolated waypoint sequences for the motion controller.

Responsibilities:
1. Merge multiple PrintObject trajectories into a single timeline
2. Insert travel moves (Z-up → XY travel → Z-down) between objects
3. Handle retract/prime sequences at segment boundaries
4. Interpolate at fixed timestep via cubic spline for smooth motion
5. Detect sharp corners and insert deceleration zones
6. Import/validate CSV trajectories: (x,y,z,p1,p2,p3,t)

All XY coordinates are in mm (converted to µm at command time).
Z and pump positions are in mm.
"""

from __future__ import annotations

import csv
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.interpolate import CubicSpline

from .PhysicalModels import WorkspaceConfig, SyringeSpec
from .GeometryEngine import (
    PrintObject, PrintCollection,
    COL_X, COL_Y, COL_Z, COL_P1, COL_P2, COL_P3, COL_T, NUM_COLS,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_INTERP_DT = 0.05       # 50 ms interpolation timestep
CORNER_ANGLE_THRESHOLD = 30.0  # degrees — sharper = deceleration zone
DECEL_ZONE_TIME = 0.3          # seconds of decel/accel around corners
MIN_SEGMENT_LENGTH = 0.001     # mm — skip degenerate segments


# ---------------------------------------------------------------------------
# Waypoint Dataclass
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    """
    A single point in the trajectory with all axis positions and timestamp.

    Coordinates:
        x, y: mm (XY stage) — converted to µm at command time
        z: mm (Z axis)
        p1, p2, p3: mm (pump plunger positions)
        t: seconds from print start
    """
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    p3: float = 0.0

    # Metadata (not part of the kinematic state)
    is_travel: bool = False     # True for travel moves (not printing)
    is_retract: bool = False    # True during retract/prime
    segment_id: int = 0         # Which object/segment this belongs to

    def to_array(self) -> np.ndarray:
        """Convert to 7-element array [x,y,z,p1,p2,p3,t]."""
        return np.array([self.x, self.y, self.z, self.p1, self.p2, self.p3, self.t])

    @classmethod
    def from_array(cls, arr: np.ndarray, **kwargs) -> Waypoint:
        """Create from 7-element array [x,y,z,p1,p2,p3,t]."""
        return cls(
            x=float(arr[COL_X]), y=float(arr[COL_Y]), z=float(arr[COL_Z]),
            p1=float(arr[COL_P1]), p2=float(arr[COL_P2]), p3=float(arr[COL_P3]),
            t=float(arr[COL_T]),
            **kwargs,
        )


def waypoints_to_array(waypoints: list[Waypoint]) -> np.ndarray:
    """Convert list of Waypoints to Nx7 array."""
    if not waypoints:
        return np.empty((0, NUM_COLS))
    return np.array([w.to_array() for w in waypoints])


def array_to_waypoints(arr: np.ndarray, **kwargs) -> list[Waypoint]:
    """Convert Nx7 array to list of Waypoints."""
    return [Waypoint.from_array(row, **kwargs) for row in arr]


# ---------------------------------------------------------------------------
# CSV Import / Validation
# ---------------------------------------------------------------------------

REQUIRED_COLUMNS = ["x", "y", "z", "p1", "p2", "p3", "t"]


def import_csv_trajectory(filepath: str | Path) -> np.ndarray:
    """
    Load and validate a (x,y,z,p1,p2,p3,t) trajectory from CSV.

    Accepts files with or without a header row. Rows with fewer than
    7 numeric columns are skipped. Time column must be monotonically
    non-decreasing.

    Args:
        filepath: Path to CSV file

    Returns:
        Nx7 numpy array [x,y,z,p1,p2,p3,t]

    Raises:
        ValueError: If file has no valid data or time is not monotonic
        FileNotFoundError: If file doesn't exist
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"CSV file not found: {filepath}")

    rows = []
    with open(filepath, 'r', newline='') as f:
        reader = csv.reader(f)
        for line_num, row in enumerate(reader, 1):
            if len(row) < 7:
                continue
            try:
                vals = [float(v.strip()) for v in row[:7]]
                rows.append(vals)
            except ValueError:
                # Skip header or non-numeric rows
                if line_num == 1:
                    continue
                logger.debug(f"Skipping non-numeric row {line_num}")
                continue

    if not rows:
        raise ValueError(f"No valid trajectory data in {filepath}")

    data = np.array(rows, dtype=np.float64)

    # Validate time monotonicity
    times = data[:, COL_T]
    if len(times) > 1:
        dt = np.diff(times)
        if np.any(dt < -1e-9):
            raise ValueError(
                f"Time column is not monotonically non-decreasing. "
                f"First violation at row {np.argmin(dt < 0) + 2}"
            )

    logger.info(f"Imported CSV: {len(data)} waypoints, "
                f"t=[{times[0]:.3f}, {times[-1]:.3f}]s")
    return data


def validate_trajectory(data: np.ndarray) -> list[str]:
    """
    Validate a trajectory array and return a list of warnings.

    Checks:
    - Shape is Nx7
    - Time is monotonically non-decreasing
    - No NaN/Inf values
    - Reasonable coordinate ranges
    """
    warnings = []

    if data.ndim != 2 or data.shape[1] != NUM_COLS:
        warnings.append(f"Expected Nx{NUM_COLS} array, got shape {data.shape}")
        return warnings

    if np.any(np.isnan(data)):
        warnings.append("Trajectory contains NaN values")
    if np.any(np.isinf(data)):
        warnings.append("Trajectory contains Inf values")

    times = data[:, COL_T]
    if len(times) > 1 and np.any(np.diff(times) < -1e-9):
        warnings.append("Time column is not monotonically non-decreasing")

    # Coordinate range checks
    xy_range = max(
        np.ptp(data[:, COL_X]) if len(data) > 0 else 0,
        np.ptp(data[:, COL_Y]) if len(data) > 0 else 0,
    )
    if xy_range > 200:  # > 200mm seems excessive
        warnings.append(f"XY range is {xy_range:.1f} mm — very large")

    z_range = np.ptp(data[:, COL_Z]) if len(data) > 0 else 0
    if z_range > 50:
        warnings.append(f"Z range is {z_range:.1f} mm — very large")

    return warnings


# ---------------------------------------------------------------------------
# Corner Detection
# ---------------------------------------------------------------------------

def detect_corners(
    xy_points: np.ndarray,
    angle_threshold_deg: float = CORNER_ANGLE_THRESHOLD,
) -> np.ndarray:
    """
    Detect sharp corners in an XY path.

    A corner is defined where the angle between consecutive segments
    exceeds `angle_threshold_deg`.

    Args:
        xy_points: Nx2 array of (x, y) coordinates
        angle_threshold_deg: Minimum angle change to count as corner

    Returns:
        1D boolean array of length N, True at corner vertices
    """
    n = len(xy_points)
    is_corner = np.zeros(n, dtype=bool)

    if n < 3:
        return is_corner

    # Compute segment vectors
    segments = np.diff(xy_points, axis=0)  # (N-1)x2
    seg_lengths = np.sqrt(np.sum(segments ** 2, axis=1))

    for i in range(1, len(segments)):
        l1, l2 = seg_lengths[i - 1], seg_lengths[i]
        if l1 < MIN_SEGMENT_LENGTH or l2 < MIN_SEGMENT_LENGTH:
            continue

        # Angle between consecutive segments via dot product
        v1 = segments[i - 1] / l1
        v2 = segments[i] / l2
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_deg = math.degrees(math.acos(cos_angle))

        if angle_deg > angle_threshold_deg:
            is_corner[i] = True  # vertex index i (0-indexed in xy_points)

    return is_corner


# ---------------------------------------------------------------------------
# Deceleration Zone Insertion
# ---------------------------------------------------------------------------

def insert_deceleration_zones(
    trajectory: np.ndarray,
    corner_indices: np.ndarray,
    decel_time_s: float = DECEL_ZONE_TIME,
    min_speed_fraction: float = 0.1,
) -> np.ndarray:
    """
    Modify trajectory timing around corners to slow down.

    At each corner, the trajectory is re-timed so that:
    - Speed decreases to `min_speed_fraction` of original speed
      approaching the corner
    - Speed increases back to original after the corner

    This is done by stretching time in the vicinity of corners.

    Args:
        trajectory: Nx7 array
        corner_indices: Boolean array from detect_corners
        decel_time_s: Total time to add per corner (split before/after)
        min_speed_fraction: Speed at corner as fraction of original

    Returns:
        Modified Nx7 array with adjusted time column
    """
    if len(trajectory) < 3 or not np.any(corner_indices):
        return trajectory.copy()

    result = trajectory.copy()
    times = result[:, COL_T].copy()
    n = len(times)

    # For each corner, add extra time
    time_additions = np.zeros(n)
    indices = np.where(corner_indices[:n])[0]

    for idx in indices:
        # Add time before and after the corner
        half_add = decel_time_s / 2.0

        # Spread the deceleration over nearby points
        radius = max(3, int(decel_time_s / DEFAULT_INTERP_DT))
        start = max(0, idx - radius)
        end = min(n, idx + radius + 1)

        for j in range(start, end):
            dist = abs(j - idx) / max(1, radius)
            # Gaussian-ish profile: most time added at corner
            weight = math.exp(-2.0 * dist * dist)
            time_additions[j] += half_add * weight

    # Apply cumulative time additions
    cumulative_add = np.cumsum(time_additions)
    result[:, COL_T] = times + cumulative_add

    return result


# ---------------------------------------------------------------------------
# Travel Move Insertion
# ---------------------------------------------------------------------------

def create_travel_move(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    travel_z: float,
    current_z: float,
    t_start: float,
    z_speed_mm_s: float = 2.0,
    xy_speed_mm_s: float = 20.0,
    pump_positions: tuple[float, float, float] = (0, 0, 0),
) -> np.ndarray:
    """
    Create a travel move sequence: Z-up → XY travel → Z-down.

    Args:
        start_xy: (x, y) in mm, starting position
        end_xy: (x, y) in mm, target position
        travel_z: Z height for safe travel (mm)
        current_z: Current Z height (mm)
        t_start: Start time (s)
        z_speed_mm_s: Z axis speed
        xy_speed_mm_s: XY travel speed
        pump_positions: Current (p1, p2, p3) positions

    Returns:
        Nx7 array for the travel move
    """
    p1, p2, p3 = pump_positions
    waypoints = []

    t = t_start

    # 1. Z-up to travel height
    if current_z < travel_z:
        dz = travel_z - current_z
        dt_z = dz / z_speed_mm_s
        waypoints.append([start_xy[0], start_xy[1], current_z, p1, p2, p3, t])
        t += dt_z
        waypoints.append([start_xy[0], start_xy[1], travel_z, p1, p2, p3, t])

    # 2. XY travel at travel_z
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    dist_xy = math.sqrt(dx * dx + dy * dy)
    if dist_xy > MIN_SEGMENT_LENGTH:
        dt_xy = dist_xy / xy_speed_mm_s
        if not waypoints:
            waypoints.append([start_xy[0], start_xy[1], travel_z, p1, p2, p3, t])
        t += dt_xy
        waypoints.append([end_xy[0], end_xy[1], travel_z, p1, p2, p3, t])

    # 3. Z-down to target Z (current_z or the next segment's start Z)
    if travel_z > current_z:
        dz = travel_z - current_z
        dt_z = dz / z_speed_mm_s
        if not waypoints:
            waypoints.append([end_xy[0], end_xy[1], travel_z, p1, p2, p3, t])
        t += dt_z
        waypoints.append([end_xy[0], end_xy[1], current_z, p1, p2, p3, t])

    if not waypoints:
        # No movement needed
        return np.empty((0, NUM_COLS))

    return np.array(waypoints, dtype=np.float64)


# ---------------------------------------------------------------------------
# Retract / Prime Sequences
# ---------------------------------------------------------------------------

def create_retract_sequence(
    position: np.ndarray,
    pump_col: int,
    retract_uL: float,
    retract_speed_uL_s: float,
    syringe: SyringeSpec | None,
    t_start: float,
) -> np.ndarray:
    """
    Create a pump retract (pull-back) sequence to prevent oozing.

    Args:
        position: Current 7-element position array
        pump_col: Column index (COL_P1, COL_P2, COL_P3)
        retract_uL: Volume to retract in µL
        retract_speed_uL_s: Speed in µL/s
        syringe: Syringe spec for µL→mm conversion
        t_start: Start time

    Returns:
        Nx7 array (2 points: start + end of retract)
    """
    if syringe is None or retract_uL <= 0:
        return np.empty((0, NUM_COLS))

    retract_mm = retract_uL * syringe.mm_per_uL
    dt = retract_uL / retract_speed_uL_s if retract_speed_uL_s > 0 else 0.1

    start = position.copy()
    start[COL_T] = t_start

    end = position.copy()
    end[pump_col] -= retract_mm  # Pull back
    end[COL_T] = t_start + dt

    return np.vstack([start, end])


def create_prime_sequence(
    position: np.ndarray,
    pump_col: int,
    prime_uL: float,
    prime_speed_uL_s: float,
    syringe: SyringeSpec | None,
    t_start: float,
) -> np.ndarray:
    """
    Create a pump prime (push-forward) sequence to refill the tip.

    Returns Nx7 array (2 points: start + end of prime).
    """
    if syringe is None or prime_uL <= 0:
        return np.empty((0, NUM_COLS))

    prime_mm = prime_uL * syringe.mm_per_uL
    dt = prime_uL / prime_speed_uL_s if prime_speed_uL_s > 0 else 0.1

    start = position.copy()
    start[COL_T] = t_start

    end = position.copy()
    end[pump_col] += prime_mm  # Push forward
    end[COL_T] = t_start + dt

    return np.vstack([start, end])


# ---------------------------------------------------------------------------
# Cubic Spline Interpolation
# ---------------------------------------------------------------------------

def interpolate_trajectory(
    raw: np.ndarray,
    dt: float = DEFAULT_INTERP_DT,
    smooth_corners: bool = True,
    corner_angle_deg: float = CORNER_ANGLE_THRESHOLD,
) -> np.ndarray:
    """
    Resample trajectory at fixed timestep using cubic spline interpolation.

    Optionally detects sharp corners and inserts deceleration zones
    before interpolating, ensuring smooth motion through turns.

    Args:
        raw: Nx7 array [x,y,z,p1,p2,p3,t]
        dt: Interpolation timestep in seconds
        smooth_corners: If True, detect and decelerate at corners
        corner_angle_deg: Angle threshold for corner detection

    Returns:
        Mx7 interpolated trajectory array
    """
    if len(raw) < 2:
        return raw.copy()

    data = raw.copy()

    # Corner detection and deceleration
    if smooth_corners and len(data) > 2:
        xy = data[:, [COL_X, COL_Y]]
        corners = detect_corners(xy, corner_angle_deg)
        if np.any(corners):
            data = insert_deceleration_zones(data, corners)

    times = data[:, COL_T]
    t_start, t_end = times[0], times[-1]

    if t_end - t_start < dt:
        return data.copy()

    # Remove duplicate timestamps (can cause spline issues)
    unique_mask = np.concatenate([[True], np.diff(times) > 1e-9])
    if not np.all(unique_mask):
        data = data[unique_mask]
        times = data[:, COL_T]

    if len(data) < 2:
        return data.copy()

    # Build cubic splines for each axis
    # Use 'clamped' boundary for smoother endpoints
    new_times = np.arange(t_start, t_end, dt)
    if len(new_times) == 0:
        return data.copy()
    # Ensure we include the final point
    if new_times[-1] < t_end - dt * 0.1:
        new_times = np.append(new_times, t_end)

    result = np.zeros((len(new_times), NUM_COLS))
    result[:, COL_T] = new_times

    for col in [COL_X, COL_Y, COL_Z, COL_P1, COL_P2, COL_P3]:
        try:
            cs = CubicSpline(times, data[:, col], bc_type='clamped')
            result[:, col] = cs(new_times)
        except ValueError:
            # Fallback to linear interpolation if spline fails
            result[:, col] = np.interp(new_times, times, data[:, col])

    return result


# ---------------------------------------------------------------------------
# Trajectory Planner
# ---------------------------------------------------------------------------

class TrajectoryPlanner:
    """
    Converts PrintObject trajectories into executable waypoint sequences.

    The planner handles the full pipeline:
    1. Take a PrintCollection (set of objects for one well)
    2. For each object: extract trajectory, add retract/prime
    3. Insert travel moves between objects
    4. Merge into single timeline
    5. Interpolate at fixed timestep
    6. Return list of Waypoints for the motion controller
    """

    def __init__(self, workspace: WorkspaceConfig | None = None):
        self.workspace = workspace or WorkspaceConfig()

    @property
    def _settings(self) -> dict:
        return self.workspace.print_settings

    def plan_well_print(
        self,
        collection: PrintCollection,
        well_center_xy: tuple[float, float] = (0.0, 0.0),
        well_bottom_z: float = 0.0,
    ) -> list[Waypoint]:
        """
        Generate complete waypoint list for printing one well.

        Args:
            collection: PrintCollection with objects to print
            well_center_xy: (x, y) offset for the well center in mm
            well_bottom_z: Z offset for this well's bottom

        Returns:
            Ordered list of Waypoints ready for the motion controller
        """
        if collection.num_objects == 0:
            return []

        travel_z = self._settings.get("travel_z_mm", 5.0)
        z_speed = self._settings.get("z_feed_rate_mm_s", 2.0)
        retract_uL = self._settings.get("retract_distance_uL", 0.5)
        prime_uL = self._settings.get("prime_distance_uL", 0.5)
        retract_speed = self._settings.get("retract_speed_uL_s", 2.0)
        prime_speed = self._settings.get("prime_speed_uL_s", 1.0)

        # Determine which pump column each object uses
        segments: list[np.ndarray] = []
        t_current = 0.0
        prev_end_xy = well_center_xy
        prev_z = travel_z

        for seg_idx, obj in enumerate(collection.objects):
            if not obj.has_trajectory or obj.num_waypoints < 1:
                continue

            traj = obj.trajectory.copy()

            # Offset by well center and well bottom Z
            traj[:, COL_X] += well_center_xy[0]
            traj[:, COL_Y] += well_center_xy[1]
            traj[:, COL_Z] += well_bottom_z

            # Get pump info
            pump_id = "P1"
            if obj.ink_assignments:
                pump_id = list(obj.ink_assignments.keys())[0]
            pump_col = {"P1": COL_P1, "P2": COL_P2, "P3": COL_P3}.get(pump_id, COL_P1)
            syringe = self._get_syringe(pump_id)

            # Current pump positions from trajectory start
            p_start = traj[0, [COL_P1, COL_P2, COL_P3]]

            # 1. Travel to object start
            obj_start_xy = (float(traj[0, COL_X]), float(traj[0, COL_Y]))
            obj_start_z = float(traj[0, COL_Z])
            travel = create_travel_move(
                start_xy=prev_end_xy,
                end_xy=obj_start_xy,
                travel_z=travel_z + well_bottom_z,
                current_z=prev_z,
                t_start=t_current,
                z_speed_mm_s=z_speed,
                pump_positions=tuple(p_start),
            )
            if len(travel) > 0:
                segments.append(travel)
                t_current = float(travel[-1, COL_T]) + 0.05

            # 2. Prime before printing
            prime_pos = traj[0].copy()
            prime_pos[COL_T] = t_current
            prime_seq = create_prime_sequence(
                prime_pos, pump_col, prime_uL, prime_speed, syringe, t_current,
            )
            if len(prime_seq) > 0:
                segments.append(prime_seq)
                t_current = float(prime_seq[-1, COL_T]) + 0.02

            # 3. Print trajectory (time-shifted to current time)
            t_offset = t_current - float(traj[0, COL_T])
            traj[:, COL_T] += t_offset
            segments.append(traj)
            t_current = float(traj[-1, COL_T]) + 0.02

            # 4. Retract after printing
            retract_pos = traj[-1].copy()
            retract_seq = create_retract_sequence(
                retract_pos, pump_col, retract_uL, retract_speed, syringe, t_current,
            )
            if len(retract_seq) > 0:
                segments.append(retract_seq)
                t_current = float(retract_seq[-1, COL_T]) + 0.02

            # Update positions for next object
            prev_end_xy = (float(traj[-1, COL_X]), float(traj[-1, COL_Y]))
            prev_z = float(traj[-1, COL_Z])

        if not segments:
            return []

        # Merge all segments
        merged = np.vstack(segments)

        # Ensure time monotonicity after merging
        merged = _enforce_time_monotonicity(merged)

        # Convert to Waypoints
        return array_to_waypoints(merged)

    def plan_and_interpolate(
        self,
        collection: PrintCollection,
        well_center_xy: tuple[float, float] = (0.0, 0.0),
        well_bottom_z: float = 0.0,
        dt: float = DEFAULT_INTERP_DT,
        smooth_corners: bool = True,
    ) -> list[Waypoint]:
        """
        Plan a well print and interpolate at fixed timestep.

        Convenience method combining planning + interpolation.
        """
        waypoints = self.plan_well_print(collection, well_center_xy, well_bottom_z)
        if not waypoints:
            return []

        raw = waypoints_to_array(waypoints)
        interpolated = interpolate_trajectory(raw, dt=dt, smooth_corners=smooth_corners)
        return array_to_waypoints(interpolated)

    def plan_multi_well(
        self,
        collection: PrintCollection,
        well_positions: list[tuple[float, float]],
        well_z_offsets: list[float] | None = None,
    ) -> list[Waypoint]:
        """
        Plan a full multi-well print job.

        Args:
            collection: Same print collection applied to each well
            well_positions: List of (x, y) well center positions in mm
            well_z_offsets: Optional Z offsets per well (from plane fitting)

        Returns:
            Complete waypoint list for all wells in order
        """
        if well_z_offsets is None:
            well_z_offsets = [0.0] * len(well_positions)

        all_waypoints: list[Waypoint] = []
        t_offset = 0.0

        for i, (xy, z_off) in enumerate(zip(well_positions, well_z_offsets)):
            well_wps = self.plan_well_print(collection, xy, z_off)
            if not well_wps:
                continue

            # Time-shift this well's waypoints
            for wp in well_wps:
                wp.t += t_offset
                wp.segment_id = i

            all_waypoints.extend(well_wps)
            t_offset = all_waypoints[-1].t + 0.5  # Gap between wells

        return all_waypoints

    def _get_syringe(self, pump_id: str) -> SyringeSpec | None:
        """Get syringe spec for a pump from workspace."""
        pump = self.workspace.pumps.get(pump_id)
        if pump is None:
            return None
        return pump.syringe


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _enforce_time_monotonicity(data: np.ndarray) -> np.ndarray:
    """
    Ensure time column is strictly non-decreasing.

    If any time steps go backward, adjust them to be at least
    epsilon after the previous point.
    """
    times = data[:, COL_T]
    eps = 1e-6
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            times[i] = times[i - 1] + eps
    data[:, COL_T] = times
    return data


def compute_path_speeds(trajectory: np.ndarray) -> np.ndarray:
    """
    Compute instantaneous XY speed at each waypoint.

    Returns:
        N-length array of speeds in mm/s
    """
    n = len(trajectory)
    if n < 2:
        return np.zeros(n)

    dx = np.diff(trajectory[:, COL_X])
    dy = np.diff(trajectory[:, COL_Y])
    dt = np.diff(trajectory[:, COL_T])

    # Avoid division by zero
    dt = np.maximum(dt, 1e-9)

    dist = np.sqrt(dx ** 2 + dy ** 2)
    speeds = dist / dt

    # Pad to match length (repeat last speed)
    return np.append(speeds, speeds[-1] if len(speeds) > 0 else 0)


def estimate_print_time(waypoints: list[Waypoint]) -> float:
    """Estimate total print time from waypoint list."""
    if not waypoints:
        return 0.0
    return waypoints[-1].t - waypoints[0].t


def trajectory_summary(waypoints: list[Waypoint]) -> dict:
    """Compute summary statistics for a waypoint list."""
    if not waypoints:
        return {"num_waypoints": 0, "total_time_s": 0, "path_length_mm": 0}

    arr = waypoints_to_array(waypoints)
    diffs = np.diff(arr[:, :3], axis=0)
    path_len = float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))
    speeds = compute_path_speeds(arr)

    return {
        "num_waypoints": len(waypoints),
        "total_time_s": float(arr[-1, COL_T] - arr[0, COL_T]),
        "path_length_mm": path_len,
        "avg_speed_mm_s": float(np.mean(speeds)),
        "max_speed_mm_s": float(np.max(speeds)),
        "x_range_mm": float(np.ptp(arr[:, COL_X])),
        "y_range_mm": float(np.ptp(arr[:, COL_Y])),
        "z_range_mm": float(np.ptp(arr[:, COL_Z])),
    }
