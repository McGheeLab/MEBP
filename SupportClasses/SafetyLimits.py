"""
Safety Limits — Software endstops for all axes.

Prevents hardware damage by clamping movement commands within configured
bounds.  Limits are persisted via :class:`Settings` and can be toggled
at runtime (e.g. disabled during calibration).

All positions use the same units as their respective controllers:
    - XY: µm (Prior ProScan speaks µm natively)
    - Z / Pumps: mm (Marlin G-code)

v7.1 additions:
    - Per-pump max flow rate limits in µL/s (P8.28)
    - clamp_flow_rate() method (P8.29)

Usage::

    limits = SafetyLimits.from_dict(saved_dict)
    clamped_x, clamped_y = limits.clamp_xy(req_x, req_y)
    clamped_z = limits.clamp_z(req_z)
    clamped_p = limits.clamp_pump(req_p, "P1")
    clamped_flow = limits.clamp_flow_rate(5.0, "P1")
"""

from __future__ import annotations

import logging
from dataclasses import asdict, dataclass, field

logger = logging.getLogger(__name__)

# v7.5.x: the per-pump max FLOW RATE ceiling is derived from needle geometry
# (inner diameter + length) via FlowPhysics' Hagen–Poiseuille calc, using a
# FIXED reference viscosity so the ceiling is a HARDWARE limit independent of
# whichever ink is loaded. Water (≈1 cP) is the least-viscous realistic fluid,
# giving the highest Q_max for the pressure budget — i.e. the most permissive
# geometry-only ceiling. Per-ink / per-move flow safety is still applied
# elsewhere (FlowPhysics.calculate_flow_safety, clamp_flow_rate). Lives here
# (not in FlowPhysics) to keep FlowPhysics ink-agnostic and avoid a circular
# import.
REFERENCE_VISCOSITY_CP = 1.0


def _reference_flow_ceiling(bore_like) -> float | None:
    """Hagen–Poiseuille flow ceiling (µL/s) for ONE bore, at the fixed
    :data:`REFERENCE_VISCOSITY_CP`.

    ``bore_like`` may be a whole ``NeedleSpec`` (which resolves bore 0) or a
    single ``NeedleBore`` — the two are duck-compatible, and a ``NeedleBore``
    hits ``max_safe_flow_rate_uL_s``'s bit-exact single-cylinder fast path with
    the SAME operands (``id_m`` / ``length_mm``), so a single-bore needle's
    ceiling is unchanged to the last ULP. That matters: the ceiling is stored
    and compared downstream (Quick Print's speed chain, print records).

    Returns ``None`` — never 0.0 — when there is nothing valid to derive from,
    so the caller LEAVES an existing ceiling alone rather than widening it to
    0 = "no limit enforced".
    """
    if bore_like is None:
        return None
    # A duck-typed stub / MagicMock has no usable bore, and `id_m > 0` on one
    # raises; require a real number so the gate is total.
    d = getattr(bore_like, "id_m", None)
    if not isinstance(d, (int, float)) or isinstance(d, bool) or d <= 0:
        return None
    try:
        # Imported lazily: FlowPhysics imports PhysicalModels, and keeping this
        # module free of SupportClasses imports at load time is deliberate.
        from .FlowPhysics import (
            max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
        )
        from .PhysicalModels import InkSpec
        ref_ink = InkSpec(name="__reference__",
                          viscosity_cP=REFERENCE_VISCOSITY_CP)
        c = max_safe_flow_rate_uL_s(bore_like, ref_ink,
                                    DEFAULT_PRESSURE_LIMIT_PA)
    except Exception as e:
        logger.warning(f"needle-derived flow ceiling failed: {e}")
        return None
    return float(c) if c and c > 0 else None


@dataclass
class SafetyLimits:
    """Software endstops for XY, Z, and pump axes."""

    # XY limits (µm, ABSOLUTE Prior stage frame — fixed mechanical extents,
    # unaffected by Set Zero / zero_position; clamps compare the absolute
    # destination against these bounds).
    xy_min_x: float = -130_000.0
    xy_min_y: float = -85_000.0
    xy_max_x: float = 130_000.0
    xy_max_y: float = 85_000.0

    # Z needle limits (mm, ABSOLUTE Marlin raw frame — fixed mechanical
    # extents, unaffected by Set Z Zero / needle-zero calibration; clamps
    # compare the absolute destination against these bounds. v7.5.x: was
    # zero-referenced, which went stale whenever calibration re-anchored the
    # needle zero and wrongly clamped valid jogs).
    z_min: float = -10.0
    z_max: float = 50.0

    # Pump limits (mm, ABSOLUTE Marlin raw frame — see Z note above)
    p1_min: float = -50.0
    p1_max: float = 50.0
    p2_min: float = -50.0
    p2_max: float = 50.0
    p3_min: float = -50.0
    p3_max: float = 50.0

    # Speed/feedrate limits
    max_xy_speed: float = 10_000.0     # µm/s
    max_z_feedrate: float = 500.0      # mm/min
    max_pump_feedrate: float = 200.0   # mm/min (global default / fallback)

    # v7.5.x: per-pump max plunger feedrate (mm/min). Each pump may hold a
    # different syringe, so the same mm/min plunger speed maps to a DIFFERENT
    # volumetric rate (µL/s). ``0.0`` means "inherit the global
    # ``max_pump_feedrate``" (back-compat: a config saved before this change has
    # no per-pump keys → all fall back to the single value). Auto-persisted via
    # ``to_dict``/``from_dict``.
    max_pump_feedrate_p1: float = 0.0
    max_pump_feedrate_p2: float = 0.0
    max_pump_feedrate_p3: float = 0.0

    # v7.1: Per-pump max flow rate limits (µL/s) — P8.28
    # Computed from FlowPhysics based on needle/syringe/ink combo.
    # Default of 0.0 means "not yet computed / no limit enforced".
    max_flow_rate_p1_uL_s: float = 0.0
    max_flow_rate_p2_uL_s: float = 0.0
    max_flow_rate_p3_uL_s: float = 0.0

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

    def xy_center(self) -> tuple[float, float]:
        """Midpoint of the XY travel envelope (ABSOLUTE stage µm).

        Used to centre the default (uncalibrated) well plate inside the
        configured envelope regardless of whether the envelope is symmetric.
        Since the envelope is now absolute, this is the absolute stage µm at
        the centre of the physical travel — the plate is seeded here directly
        (see ``StageController.default_plate_center_um``). For the symmetric
        default (±130000 / ±85000) this is (0, 0); for an asymmetric envelope
        (e.g. 0..114332) it is the true centre, not the corner.
        """
        return (
            (self.xy_min_x + self.xy_max_x) / 2.0,
            (self.xy_min_y + self.xy_max_y) / 2.0,
        )

    def clamp_z(self, z: float) -> float:
        """Clamp Z position to limits. v7.5.x: ``z`` is absolute Marlin raw mm."""
        if not self.enabled:
            return z
        cz = max(self.z_min, min(self.z_max, z))
        if cz != z:
            logger.warning(f"Z clamped: {z:.3f} → {cz:.3f} mm")
        return cz

    def clamp_pump(self, position: float, pump: str = "P1") -> float:
        """Clamp pump position to limits. v7.5.x: ``position`` is absolute Marlin raw mm."""
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

    def clamp_pump_feedrate(self, feedrate: float, pump: str = "P1") -> float:
        """Clamp a pump plunger feedrate (mm/min) to the per-pump maximum.

        v7.5.x: the ceiling is per-pump (``pump_feedrate_max``) because each pump
        may hold a different syringe; falls back to the global
        ``max_pump_feedrate`` when no per-pump override is set.
        """
        if not self.enabled or feedrate is None:
            return feedrate
        max_fr = self.pump_feedrate_max(pump)
        clamped = min(feedrate, max_fr)
        if clamped != feedrate:
            logger.warning(
                f"{pump} feedrate clamped: {feedrate:.0f} → {clamped:.0f} mm/min")
        return clamped

    # ── v7.5.x: per-pump max plunger feedrate (mm/min) ───────────

    def pump_feedrate_max(self, pump: str = "P1") -> float:
        """Per-pump max plunger feedrate (mm/min).

        Returns the pump's own override when set (> 0), else the global
        ``max_pump_feedrate``.
        """
        attr = self._pump_feedrate_attr(pump)
        if attr:
            try:
                v = float(getattr(self, attr, 0.0) or 0.0)
            except (TypeError, ValueError):
                v = 0.0
            if v > 0:
                return v
        return float(self.max_pump_feedrate)

    def set_pump_feedrate_max(self, pump: str, feedrate_mm_min: float) -> None:
        """Set a pump's per-pump max plunger feedrate (mm/min). ``0`` clears the
        override so the pump inherits the global ``max_pump_feedrate``."""
        attr = self._pump_feedrate_attr(pump)
        if attr:
            setattr(self, attr, float(feedrate_mm_min))
            logger.info(f"{pump} max plunger feedrate = {feedrate_mm_min:.0f} mm/min")

    def _pump_feedrate_attr(self, pump: str) -> str | None:
        """Attribute name for a pump's per-pump max feedrate."""
        return {
            "P1": "max_pump_feedrate_p1",
            "P2": "max_pump_feedrate_p2",
            "P3": "max_pump_feedrate_p3",
        }.get(pump)

    # ── v7.1: Flow Rate Clamping (P8.29) ─────────────────────────

    def clamp_flow_rate(self, flow_rate_uL_s: float, pump: str = "P1") -> float:
        """
        Clamp a volumetric flow rate to the safe maximum for a pump.

        The max flow rate per pump is computed from FlowPhysics based on
        the current needle/syringe/ink configuration. A value of 0.0 means
        no limit has been configured yet (passes through unclamped).

        Args:
            flow_rate_uL_s: Requested flow rate (µL/s, positive = dispense)
            pump: Pump identifier ("P1", "P2", "P3")

        Returns:
            Clamped flow rate in µL/s (preserves sign)
        """
        if not self.enabled:
            return flow_rate_uL_s

        max_rate = self._get_max_flow_rate(pump)
        if max_rate <= 0.0:
            # No limit configured — pass through
            return flow_rate_uL_s

        # Clamp magnitude, preserve sign (positive = dispense, negative = aspirate)
        sign = 1.0 if flow_rate_uL_s >= 0 else -1.0
        magnitude = abs(flow_rate_uL_s)
        if magnitude > max_rate:
            logger.warning(
                f"{pump} flow rate clamped: {flow_rate_uL_s:.3f} → "
                f"{sign * max_rate:.3f} µL/s (max={max_rate:.3f})"
            )
            return sign * max_rate
        return flow_rate_uL_s

    def set_max_flow_rate(self, pump: str, max_rate_uL_s: float) -> None:
        """
        Set the maximum safe flow rate for a pump.

        Typically called after FlowPhysics.max_safe_flow_rate_uL_s()
        computes the limit based on current needle/syringe/ink config.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
            max_rate_uL_s: Maximum safe flow rate in µL/s
        """
        attr = self._flow_rate_attr(pump)
        if attr:
            setattr(self, attr, max_rate_uL_s)
            logger.info(f"{pump} max flow rate set to {max_rate_uL_s:.3f} µL/s")

    def get_max_flow_rate(self, pump: str) -> float:
        """Get the configured max flow rate for a pump (µL/s). 0 = no limit."""
        return self._get_max_flow_rate(pump)

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
        if self.z_min > self.z_max:
            logger.warning(
                f"Z envelope is INVERTED: z_min={self.z_min:.3f} > "
                f"z_max={self.z_max:.3f} mm — every Z move will clamp to "
                f"{self.z_min:.3f}. Re-record so min < max."
            )

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

    def _flow_rate_attr(self, pump: str) -> str | None:
        """Return the attribute name for a pump's max flow rate."""
        mapping = {
            "P1": "max_flow_rate_p1_uL_s",
            "P2": "max_flow_rate_p2_uL_s",
            "P3": "max_flow_rate_p3_uL_s",
        }
        return mapping.get(pump)

    def _get_max_flow_rate(self, pump: str) -> float:
        """Get the max flow rate for a pump."""
        mapping = {
            "P1": self.max_flow_rate_p1_uL_s,
            "P2": self.max_flow_rate_p2_uL_s,
            "P3": self.max_flow_rate_p3_uL_s,
        }
        return mapping.get(pump, 0.0)

    # ── v7.2: Auto-configure from HardwareConfig ─────────────────

    def update_from_hardware_config(self, hardware_config, skip_pumps=()) -> None:
        """
        v7.2: Auto-configure safety limits from hardware config.

        Sets per-pump max flow rate limits based on needle gauge.
        Uses conservative lookup table: smaller gauge = lower max flow.

        Also updates pump travel limits based on syringe stroke length.

        v7.5.x: ``skip_pumps`` — pumps whose travel-limit overwrite should be
        SKIPPED because their envelope was set by the plunger calibration
        (:meth:`StageController.apply_pump_setup`), which must win over the
        coarse syringe-stroke estimate. Flow-rate limits are still updated.
        """
        if hardware_config is None:
            return
        skip_pumps = set(skip_pumps or ())

        # v7.5.x: per-pump max flow ceiling = a NEEDLE-DERIVED flow rate (µL/s)
        # from needle inner-diameter + length via Hagen–Poiseuille, using the
        # fixed REFERENCE_VISCOSITY_CP (ink-independent). Replaces the old coarse
        # gauge→flow lookup table. A missing/degenerate needle leaves the
        # existing ceilings UNTOUCHED (never widened to 0 = "no limit").
        #
        # v7.9: the ceiling is derived PER PUMP from THAT PUMP'S OWN BORE. Each
        # bore of a multi-bore assembly has its own pump, i.e. N independent
        # pressure sources — a 22G bore and a 30 µm pulled tip on the same
        # backpack differ by ~2000× in Q_max, so one shared ceiling would let the
        # narrow bore be driven far over-pressure and shatter a glass tip.
        # (Deliberately NOT 1/R = Σ1/R_i: that is one source feeding N parallel
        # branches, which is not this plumbing.)
        #
        # Resolving "which bore feeds this pump" MUST use the same authority the
        # rest of the config uses — `HardwareConfig.resolved_bore_pump_map`,
        # which prefers the bores' own `pump_id` and falls back to the stored
        # map. Asking only the bores would leave the exact hole this change
        # exists to close: a backpack whose bores are not wired up yet but whose
        # MAP says P2 feeds a 30 µm pulled tip validates clean, and P2 would
        # inherit the 22 G bore's ceiling — measured 7071× over-pressure.
        #
        # A pump that NEITHER authority claims falls back to the NARROWEST bore
        # on a multi-bore assembly (the fail-safe direction: too slow is
        # recoverable, over-pressure shatters glass) and to the assembly value
        # otherwise — bit-identical to pre-v7.9 for every single-bore needle,
        # since bore 0 mirrors the flat fields, and unchanged for a synthesized
        # multi-bore needle, whose bores are all identical.
        needle = getattr(hardware_config, "needle", None)
        gauge = getattr(needle, "gauge", None) if needle else None
        ceiling = _reference_flow_ceiling(needle)

        # Only an EXPLICIT bore list is heterogeneous; a pre-v7.9 needle
        # synthesizes identical bores, so neither branch below can change it.
        # Require a real sequence: a duck-typed stub hands back something
        # unindexable (or, for a plain Mock, uniterable), and this must degrade
        # to "single-bore" rather than raise — it is called on every config push.
        from .PhysicalModels import needle_bore_for_pump
        raw_bores = getattr(needle, "bores", None) if needle is not None else None
        explicit_bores = list(raw_bores) if isinstance(raw_bores, (list, tuple)) else []
        if len(explicit_bores) >= 2:
            per_bore = [c for c in (_reference_flow_ceiling(b)
                                    for b in explicit_bores) if c is not None]
            if per_bore:
                narrowest = min(per_bore)
                if ceiling is None or narrowest < ceiling:
                    ceiling = narrowest
            bore_map = {}
            try:
                bore_map = hardware_config.resolved_bore_pump_map()
            except Exception as e:                      # duck-typed stub
                logger.debug(f"resolved_bore_pump_map unavailable: {e}")
        else:
            bore_map = {}

        if ceiling is None:
            logger.warning(
                "Pump max flow ceiling NOT updated (no needle / invalid bore / "
                "calc failed) — keeping existing per-pump limits.")

        for pid in ["P1", "P2", "P3"]:
            pump_cfg = hardware_config.pumps.get(pid)
            if pump_cfg is None or not pump_cfg.is_configured:
                continue

            # Resolve this pump's own bore; `ceiling` is the fallback for a pump
            # neither authority claims — the assembly value on every pre-v7.9
            # setup, the narrowest bore on a heterogeneous assembly.
            pump_ceiling, source = ceiling, f"{gauge}G"
            bore = needle_bore_for_pump(needle, pid) if needle is not None else None
            if bore is None and bore_map:
                # The bores did not claim it, but the config's map did.
                for k, mapped in bore_map.items():
                    try:
                        idx, want = int(k), str(mapped).strip().upper()
                    except (TypeError, ValueError):
                        continue
                    if want == pid and 0 <= idx < len(explicit_bores):
                        bore = explicit_bores[idx]
                        break
            if bore is not None:
                bore_ceiling = _reference_flow_ceiling(bore)
                if bore_ceiling is not None:
                    pump_ceiling = bore_ceiling
                    source = (getattr(bore, "summary_line", lambda: "")()
                              or f"{getattr(bore, 'gauge', None)}G")
                else:
                    logger.warning(
                        f"{pid}: bore is unusable for a flow ceiling "
                        f"(invalid geometry) — using the fallback value.")

            # Skip when no valid ceiling so an existing limit isn't clobbered.
            if pump_ceiling is not None:
                self.set_max_flow_rate(pid, pump_ceiling)
                # A duck-typed needle may hand back a non-numeric length; a
                # format crash here would abort the whole limits derivation.
                length_mm = getattr(bore if bore is not None else needle,
                                    "length_mm", 0.0)
                if not isinstance(length_mm, (int, float)):
                    length_mm = 0.0
                logger.info(
                    f"{pid}: max flow rate = {pump_ceiling:.3f} µL/s "
                    f"({source}, L={length_mm:.1f} mm, "
                    f"ref µ={REFERENCE_VISCOSITY_CP:g} cP)")

            # Set pump travel limits from syringe stroke length — UNLESS the
            # plunger calibration already set a (more accurate) envelope.
            if pump_cfg.syringe and pid not in skip_pumps:
                stroke_mm = pump_cfg.syringe.stroke_length_mm
                # Allow ±stroke from zero reference (generous)
                attr_min = f"{pid.lower()}_min"
                attr_max = f"{pid.lower()}_max"
                if hasattr(self, attr_min):
                    setattr(self, attr_min, -stroke_mm * 0.1)  # Small negative for retract
                if hasattr(self, attr_max):
                    setattr(self, attr_max, stroke_mm * 1.05)  # Slight extra for safety
                logger.info(f"{pid}: pump limits = [{-stroke_mm*0.1:.1f}, {stroke_mm*1.05:.1f}] mm "
                           f"(syringe stroke = {stroke_mm:.1f} mm)")

    def get_pump_limits_uL(self, pump: str, hardware_config=None) -> tuple[float, float] | None:
        """
        v7.2: Get pump limits in µL instead of mm.

        Returns (min_uL, max_uL) or None if no syringe configured.
        """
        if hardware_config is None:
            return None

        pump_cfg = hardware_config.pumps.get(pump)
        if not pump_cfg or not pump_cfg.is_configured:
            return None

        p_min, p_max = self._pump_limits(pump)
        try:
            min_uL = pump_cfg.mm_to_uL(p_min)
            max_uL = pump_cfg.mm_to_uL(p_max)
            return (min_uL, max_uL)
        except (ValueError, AttributeError):
            return None

    def __repr__(self) -> str:
        state = "ON" if self.enabled else "OFF"
        flow_info = ""
        for pump in ("P1", "P2", "P3"):
            rate = self._get_max_flow_rate(pump)
            if rate > 0:
                flow_info += f", {pump}_flow≤{rate:.2f}µL/s"
        return (
            f"SafetyLimits({state}: "
            f"XY=[{self.xy_min_x:.0f}..{self.xy_max_x:.0f}, "
            f"{self.xy_min_y:.0f}..{self.xy_max_y:.0f}], "
            f"Z=[{self.z_min:.1f}..{self.z_max:.1f}], "
            f"P1=[{self.p1_min:.1f}..{self.p1_max:.1f}]"
            f"{flow_info})"
        )
