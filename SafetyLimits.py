"""
Safety Limits - Software endstops for all axes.

Prevents crashing hardware by clamping movement commands within
configured bounds. Limits are persisted via Settings and can be
disabled temporarily for calibration.

Usage:
    limits = SafetyLimits.from_dict(saved_dict)
    clamped_x, clamped_y = limits.clamp_xy(requested_x, requested_y)
    clamped_z = limits.clamp_z(requested_z)
    clamped_p = limits.clamp_pump(requested_p, "P1")
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, asdict
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class SafetyLimits:
    """
    Software endstops for XY, Z, and pump axes.
    
    All positions are in the same units as the stage controller:
      - XY: steps (Prior ProScan III)
      - Z / Pumps: mm (Marlin G-code)
    
    When `enabled` is True, all movement commands are clamped to
    these bounds. Violations are logged as warnings.
    """

    # XY limits (in steps, relative to zero reference)
    xy_min_x: float = -100000.0
    xy_min_y: float = -100000.0
    xy_max_x: float = 100000.0
    xy_max_y: float = 100000.0

    # Z needle limits (in mm, relative to zero reference)
    z_min: float = -10.0
    z_max: float = 50.0

    # Pump limits (in mm, relative to zero reference)
    p1_min: float = -50.0
    p1_max: float = 50.0
    p2_min: float = -50.0
    p2_max: float = 50.0
    p3_min: float = -50.0
    p3_max: float = 50.0

    # Speed limits
    max_xy_speed: float = 10000.0     # steps/sec
    max_z_feedrate: float = 500.0     # mm/min
    max_pump_feedrate: float = 200.0  # mm/min

    # Master enable
    enabled: bool = True

    # ── Clamping Methods ───────────────────────────────────────────

    def clamp_xy(self, x: float, y: float) -> tuple[float, float]:
        """
        Clamp XY position to limits.
        
        Args:
            x, y: Requested position (relative to zero reference)
            
        Returns:
            (clamped_x, clamped_y) — unchanged if limits disabled
        """
        if not self.enabled:
            return x, y

        clamped_x = max(self.xy_min_x, min(self.xy_max_x, x))
        clamped_y = max(self.xy_min_y, min(self.xy_max_y, y))

        if clamped_x != x or clamped_y != y:
            logger.warning(
                f"XY position clamped: ({x:.1f}, {y:.1f}) → ({clamped_x:.1f}, {clamped_y:.1f})"
            )

        return clamped_x, clamped_y

    def clamp_z(self, z: float) -> float:
        """
        Clamp Z position to limits.
        
        Args:
            z: Requested Z position (relative to zero reference, in mm)
            
        Returns:
            Clamped Z value
        """
        if not self.enabled:
            return z

        clamped = max(self.z_min, min(self.z_max, z))
        if clamped != z:
            logger.warning(f"Z position clamped: {z:.3f} → {clamped:.3f} mm")
        return clamped

    def clamp_pump(self, position: float, pump: str = "P1") -> float:
        """
        Clamp pump position to limits.
        
        Args:
            position: Requested pump position (relative to zero ref, in mm)
            pump: Pump identifier ("P1", "P2", "P3")
            
        Returns:
            Clamped position value
        """
        if not self.enabled:
            return position

        pump_limits = {
            "P1": (self.p1_min, self.p1_max),
            "P2": (self.p2_min, self.p2_max),
            "P3": (self.p3_min, self.p3_max),
        }
        p_min, p_max = pump_limits.get(pump, (-50.0, 50.0))
        clamped = max(p_min, min(p_max, position))
        if clamped != position:
            logger.warning(f"{pump} position clamped: {position:.3f} → {clamped:.3f} mm")
        return clamped

    def clamp_z_feedrate(self, feedrate: float) -> float:
        """Clamp Z feedrate to maximum allowed."""
        if not self.enabled or feedrate is None:
            return feedrate
        clamped = min(feedrate, self.max_z_feedrate)
        if clamped != feedrate:
            logger.warning(f"Z feedrate clamped: {feedrate:.0f} → {clamped:.0f} mm/min")
        return clamped

    def clamp_pump_feedrate(self, feedrate: float) -> float:
        """Clamp pump feedrate to maximum allowed."""
        if not self.enabled or feedrate is None:
            return feedrate
        clamped = min(feedrate, self.max_pump_feedrate)
        if clamped != feedrate:
            logger.warning(f"Pump feedrate clamped: {feedrate:.0f} → {clamped:.0f} mm/min")
        return clamped

    def check_xy_near_limit(self, x: float, y: float, margin: float = 500.0) -> dict:
        """
        Check if a position is near the XY limits.
        
        Returns:
            dict with keys 'x_near_min', 'x_near_max', 'y_near_min', 'y_near_max'
        """
        if not self.enabled:
            return {k: False for k in ("x_near_min", "x_near_max", "y_near_min", "y_near_max")}
        return {
            "x_near_min": x - self.xy_min_x < margin,
            "x_near_max": self.xy_max_x - x < margin,
            "y_near_min": y - self.xy_min_y < margin,
            "y_near_max": self.xy_max_y - y < margin,
        }

    def check_z_near_limit(self, z: float, margin: float = 1.0) -> dict:
        """Check if Z is near limits."""
        if not self.enabled:
            return {"z_near_min": False, "z_near_max": False}
        return {
            "z_near_min": z - self.z_min < margin,
            "z_near_max": self.z_max - z < margin,
        }

    # ── Serialization ──────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Serialize to dictionary for persistence."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyLimits":
        """Create from dictionary, ignoring unknown keys."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in data.items() if k in valid_keys}
        return cls(**filtered)

    # ── Convenience ────────────────────────────────────────────────

    def set_xy_from_current(self, x: float, y: float, as_max: bool = True):
        """Set XY limits from the current position (convenience for UI)."""
        if as_max:
            self.xy_max_x = x
            self.xy_max_y = y
            logger.info(f"XY max set to ({x:.1f}, {y:.1f})")
        else:
            self.xy_min_x = x
            self.xy_min_y = y
            logger.info(f"XY min set to ({x:.1f}, {y:.1f})")

    def set_z_from_current(self, z: float, as_max: bool = True):
        """Set Z limit from current position."""
        if as_max:
            self.z_max = z
            logger.info(f"Z max set to {z:.3f} mm")
        else:
            self.z_min = z
            logger.info(f"Z min set to {z:.3f} mm")

    def __repr__(self):
        state = "ON" if self.enabled else "OFF"
        return (
            f"SafetyLimits({state}: "
            f"XY=[{self.xy_min_x:.0f}..{self.xy_max_x:.0f}, {self.xy_min_y:.0f}..{self.xy_max_y:.0f}], "
            f"Z=[{self.z_min:.1f}..{self.z_max:.1f}], "
            f"P=[{self.p1_min:.1f}..{self.p1_max:.1f}])"
        )
