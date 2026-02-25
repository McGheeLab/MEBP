"""
Safety Limits — Software endstops for all axes.

Prevents hardware damage by clamping movement commands within configured
bounds.  Limits are persisted via :class:`Settings` and can be toggled
at runtime (e.g. disabled during calibration).

All positions use the same units as their respective controllers:
    - XY: steps (Prior ProScan III)
    - Z / Pumps: mm (Marlin G-code)

Usage::

    limits = SafetyLimits.from_dict(saved_dict)
    clamped_x, clamped_y = limits.clamp_xy(req_x, req_y)
    clamped_z = limits.clamp_z(req_z)
    clamped_p = limits.clamp_pump(req_p, "P1")
"""

from __future__ import annotations

import logging
from dataclasses import asdict, dataclass

logger = logging.getLogger(__name__)


@dataclass
class SafetyLimits:
    """Software endstops for XY, Z, and pump axes."""

    # XY limits (steps, relative to zero reference)
    xy_min_x: float = -100_000.0
    xy_min_y: float = -100_000.0
    xy_max_x: float = 100_000.0
    xy_max_y: float = 100_000.0

    # Z needle limits (mm, relative to zero reference)
    z_min: float = -10.0
    z_max: float = 50.0

    # Pump limits (mm, relative to zero reference)
    p1_min: float = -50.0
    p1_max: float = 50.0
    p2_min: float = -50.0
    p2_max: float = 50.0
    p3_min: float = -50.0
    p3_max: float = 50.0

    # Speed/feedrate limits
    max_xy_speed: float = 10_000.0     # steps/s
    max_z_feedrate: float = 500.0      # mm/min
    max_pump_feedrate: float = 200.0   # mm/min

    # Master enable switch
    enabled: bool = True

    # ── Clamping ──────────────────────────────────────────────────

    def clamp_xy(self, x: float, y: float) -> tuple[float, float]:
        """Clamp XY to limits. Returns (clamped_x, clamped_y)."""
        if not self.enabled:
            return x, y
        cx = max(self.xy_min_x, min(self.xy_max_x, x))
        cy = max(self.xy_min_y, min(self.xy_max_y, y))
        if cx != x or cy != y:
            logger.warning(f"XY clamped: ({x:.1f},{y:.1f}) → ({cx:.1f},{cy:.1f})")
        return cx, cy

    def clamp_z(self, z: float) -> float:
        """Clamp Z position to limits."""
        if not self.enabled:
            return z
        cz = max(self.z_min, min(self.z_max, z))
        if cz != z:
            logger.warning(f"Z clamped: {z:.3f} → {cz:.3f} mm")
        return cz

    def clamp_pump(self, position: float, pump: str = "P1") -> float:
        """Clamp pump position to limits."""
        if not self.enabled:
            return position
        p_min, p_max = self._pump_limits(pump)
        cp = max(p_min, min(p_max, position))
        if cp != position:
            logger.warning(f"{pump} clamped: {position:.3f} → {cp:.3f} mm")
        return cp

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

    # ── Proximity Checks ──────────────────────────────────────────

    def check_xy_near_limit(
        self, x: float, y: float, margin: float = 500.0
    ) -> dict[str, bool]:
        """Check if XY is within *margin* of any limit."""
        if not self.enabled:
            return {k: False for k in ("x_near_min", "x_near_max", "y_near_min", "y_near_max")}
        return {
            "x_near_min": (x - self.xy_min_x) < margin,
            "x_near_max": (self.xy_max_x - x) < margin,
            "y_near_min": (y - self.xy_min_y) < margin,
            "y_near_max": (self.xy_max_y - y) < margin,
        }

    def check_z_near_limit(self, z: float, margin: float = 1.0) -> dict[str, bool]:
        """Check if Z is within *margin* of any limit."""
        if not self.enabled:
            return {"z_near_min": False, "z_near_max": False}
        return {
            "z_near_min": (z - self.z_min) < margin,
            "z_near_max": (self.z_max - z) < margin,
        }

    # ── Convenience Setters ───────────────────────────────────────

    def set_xy_from_current(self, x: float, y: float, as_max: bool = True) -> None:
        """Set XY limits from a live position."""
        if as_max:
            self.xy_max_x, self.xy_max_y = x, y
            logger.info(f"XY max set to ({x:.1f}, {y:.1f})")
        else:
            self.xy_min_x, self.xy_min_y = x, y
            logger.info(f"XY min set to ({x:.1f}, {y:.1f})")

    def set_z_from_current(self, z: float, as_max: bool = True) -> None:
        """Set Z limit from a live position."""
        if as_max:
            self.z_max = z
            logger.info(f"Z max = {z:.3f} mm")
        else:
            self.z_min = z
            logger.info(f"Z min = {z:.3f} mm")

    # ── Serialisation ─────────────────────────────────────────────

    def to_dict(self) -> dict:
        """Serialise to a plain dict for JSON persistence."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> SafetyLimits:
        """Construct from a dict, silently ignoring unknown keys."""
        valid = {f.name for f in cls.__dataclass_fields__.values()}
        return cls(**{k: v for k, v in data.items() if k in valid})

    # ── Internal ──────────────────────────────────────────────────

    def _pump_limits(self, pump: str) -> tuple[float, float]:
        """Return (min, max) for the given pump."""
        mapping = {
            "P1": (self.p1_min, self.p1_max),
            "P2": (self.p2_min, self.p2_max),
            "P3": (self.p3_min, self.p3_max),
        }
        return mapping.get(pump, (-50.0, 50.0))

    def __repr__(self) -> str:
        state = "ON" if self.enabled else "OFF"
        return (
            f"SafetyLimits({state}: "
            f"XY=[{self.xy_min_x:.0f}..{self.xy_max_x:.0f}, "
            f"{self.xy_min_y:.0f}..{self.xy_max_y:.0f}], "
            f"Z=[{self.z_min:.1f}..{self.z_max:.1f}], "
            f"P1=[{self.p1_min:.1f}..{self.p1_max:.1f}])"
        )
