"""
print_manager_modifications.py — Changes to PrintManager for v7.2 µL-based pumps.

KEY CHANGES:
1. PrintSettings: pump amounts (retract, prime, flow_rate) in µL / µL/s
2. EXTRUDE command: amount field is now in µL (was mm)
3. PRINT_PATH command: flow_rate is in µL/s (was dimensionless)
4. _execute_command(): Uses controller.move_pump_uL() instead of move_pump_relative()
5. PrintJob stores reference to HardwareConfig for conversion
6. build_well_plate_job() generates µL-based commands

All existing print files with mm-based amounts need a migration path
(detect version and convert on load).
"""


# ═══════════════════════════════════════════════════════════════════
# MODIFIED PrintSettings
# ═══════════════════════════════════════════════════════════════════

# Replace the existing PrintSettings with:

from dataclasses import dataclass, field


@dataclass
class PrintSettings:
    """
    Print job settings — v7.2: all pump values in µL.

    Feedrate/speed values:
        xy_feedrate:     XY travel speed (stage units/sec) — unchanged
        z_feedrate:      Z feedrate (mm/min) — unchanged
        print_feedrate:  XY speed during printing — unchanged
        pump_rate_uL_s:  Default pump flow rate (µL/s) — NEW (was pump_feedrate mm/min)

    Pump amounts:
        retract_amount_uL:  Default retract volume (µL) — was retract_amount (mm)
        prime_amount_uL:    Default prime volume (µL) — was prime_amount (mm)
    """
    xy_feedrate: float = 1000.0          # XY travel speed (stage units/sec)
    z_feedrate: float = 60.0             # Z feedrate (mm/min)
    print_feedrate: float = 200.0        # XY speed during printing
    pump_rate_uL_s: float = 0.25         # Default pump flow rate (µL/s) — NEW
    travel_z_height: float = 5.0         # Z height for travel moves (mm above zero)
    print_z_height: float = 0.1          # Z height for printing (mm above zero)
    layer_height: float = 0.1            # Height increment per layer
    num_layers: int = 1                  # Number of layers
    dwell_after_move: float = 0.0        # Seconds to wait after travel moves

    # Per-pump retract/prime in µL
    retract_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})

    # Per-pump flow rates in µL/s (if different from default)
    pump_rates_uL_s: dict = field(default_factory=lambda: {"P1": 0.25, "P2": 0.25, "P3": 0.25})

    # DEPRECATED — kept for backward compatibility during migration
    pump_feedrate: float = 30.0          # Legacy mm/min — only used for old file import
    retract_amount: float = 0.0          # Legacy mm
    prime_amount: float = 0.0            # Legacy mm
    retract_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})

    def get_retract_uL(self, pump: str) -> float:
        return self.retract_amounts_uL.get(pump, 0.0)

    def get_prime_uL(self, pump: str) -> float:
        return self.prime_amounts_uL.get(pump, 0.0)

    def get_pump_rate(self, pump: str) -> float:
        """Get flow rate for a specific pump in µL/s."""
        return self.pump_rates_uL_s.get(pump, self.pump_rate_uL_s)

    @classmethod
    def from_dict(cls, d: dict) -> "PrintSettings":
        """Create from dictionary, handling both v7.1 and v7.2 formats."""
        known_fields = set(cls.__dataclass_fields__.keys())
        filtered = {k: v for k, v in d.items() if k in known_fields}
        settings = cls(**filtered)

        # Migration: if old format (has pump_feedrate but no pump_rate_uL_s)
        # We can't auto-convert without knowing the syringe, so just flag it
        if "pump_rate_uL_s" not in d and "pump_feedrate" in d:
            settings._needs_migration = True

        return settings


# ═══════════════════════════════════════════════════════════════════
# MODIFIED EXTRUDE COMMAND FORMAT
# ═══════════════════════════════════════════════════════════════════

# Print file format change for EXTRUDE commands:
#
# OLD (v7.1):
#   {"type": "extrude", "pump": "P1", "amount": 0.5, "feedrate": 30}
#   amount = mm of plunger travel, feedrate = mm/min
#
# NEW (v7.2):
#   {"type": "extrude", "pump": "P1", "amount_uL": 2.5, "rate_uL_s": 0.25}
#   amount_uL = microliters, rate_uL_s = microliters per second
#
# Both formats are supported on load — the v7.1 format requires a
# HardwareConfig to convert.


# ═══════════════════════════════════════════════════════════════════
# MODIFIED _execute_command() — EXTRUDE section
# ═══════════════════════════════════════════════════════════════════

def _execute_extrude_v72(self, cmd, ctrl, settings):
    """
    Execute an EXTRUDE command using µL-based amounts.

    Supports both v7.1 (mm) and v7.2 (µL) command formats.
    """
    p = cmd.params
    pump = p.get("pump", self._active_pump)

    # v7.2 format: amount_uL and rate_uL_s
    if "amount_uL" in p:
        amount_uL = p["amount_uL"]
        rate_uL_s = p.get("rate_uL_s", settings.get_pump_rate(pump))
        ctrl.move_pump_uL(pump, amount_uL, rate_uL_s)

        # Estimate wait time from volume and rate
        if rate_uL_s > 0:
            wait = abs(amount_uL) / rate_uL_s + 0.1
        else:
            wait = 1.0
        import time
        time.sleep(min(wait, 10.0))

    # v7.1 legacy format: amount (mm) and feedrate (mm/min)
    elif "amount" in p:
        amount_mm = p["amount"]
        feedrate = p.get("feedrate", settings.pump_feedrate)
        ctrl.move_pump_relative(pump, amount_mm, feedrate)
        wait = abs(amount_mm) / (feedrate / 60.0) + 0.1 if feedrate > 0 else 1.0
        import time
        time.sleep(min(wait, 5.0))


# ═══════════════════════════════════════════════════════════════════
# MODIFIED _execute_print_path() — flow_rate is now µL/s
# ═══════════════════════════════════════════════════════════════════

def _execute_print_path_v72(self, cmd, ctrl, settings):
    """
    Execute PRINT_PATH with µL/s flow rate.

    The flow_rate parameter is now in µL/s (was a dimensionless ratio).
    Controller handles the µL→mm conversion internally.
    """
    p = cmd.params
    points = p.get("points", [])
    pump = p.get("pump", self._active_pump)
    flow_rate_uL_s = p.get("flow_rate_uL_s", p.get("flow_rate", 0.0))

    if len(points) < 2:
        return

    import math
    import time

    # Move to first point
    ctrl.move_xy_absolute(points[0][0], points[0][1], from_zero_ref=True)
    time.sleep(0.3)

    # Print segments
    for i in range(1, len(points)):
        x1, y1 = points[i - 1][0], points[i - 1][1]
        x2, y2 = points[i][0], points[i][1]
        seg_len_mm = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Calculate volume for this segment
        # flow_rate_uL_s × segment_time = volume
        xy_speed_mm_s = settings.print_feedrate / 60.0 if settings.print_feedrate > 0 else 1.0
        seg_time = seg_len_mm / xy_speed_mm_s if xy_speed_mm_s > 0 else 0
        volume_uL = flow_rate_uL_s * seg_time

        # Command coordinated XY + pump move
        ctrl.move_xy_absolute(x2, y2, from_zero_ref=True)
        if volume_uL > 0:
            ctrl.move_pump_uL(pump, volume_uL, flow_rate_uL_s)

        # Wait for segment completion
        time.sleep(max(seg_time, 0.05))


# ═══════════════════════════════════════════════════════════════════
# MODIFIED build_well_plate_job() — all amounts in µL
# ═══════════════════════════════════════════════════════════════════

def build_well_plate_job_v72_example():
    """
    Example showing how build_well_plate_job generates µL-based commands.

    Key changes from v7.1:
    - Prime/retract amounts are in µL
    - EXTRUDE commands use amount_uL and rate_uL_s
    - PRINT_PATH uses flow_rate_uL_s
    """
    # Prime command (v7.2):
    prime_cmd_example = {
        "type": "extrude",
        "pump": "P1",
        "amount_uL": 0.5,           # Was: "amount": 0.015 (mm)
        "rate_uL_s": 0.25,          # Was: "feedrate": 30 (mm/min)
    }

    # Retract command (v7.2):
    retract_cmd_example = {
        "type": "extrude",
        "pump": "P1",
        "amount_uL": -0.3,          # Negative = aspirate/retract
        "rate_uL_s": 0.5,
    }

    # Print path command (v7.2):
    print_path_example = {
        "type": "print_path",
        "points": [[0, 0], [10, 0], [10, 10]],
        "pump": "P1",
        "flow_rate_uL_s": 0.1,      # Was: "flow_rate": 0.01 (dimensionless)
    }

    return prime_cmd_example, retract_cmd_example, print_path_example


# ═══════════════════════════════════════════════════════════════════
# PRINT FILE VERSION MIGRATION
# ═══════════════════════════════════════════════════════════════════

def migrate_print_file_v71_to_v72(data: dict, hardware_config) -> dict:
    """
    Migrate a v7.1 print file to v7.2 format.

    Converts mm-based pump amounts to µL using the hardware config.
    If no hardware config available, leaves amounts in mm with a warning.
    """
    if data.get("version", "7.1") >= "7.2":
        return data  # Already current

    import logging
    logger = logging.getLogger(__name__)
    logger.info("Migrating print file from v7.1 to v7.2 (mm → µL)")

    migrated = dict(data)
    migrated["version"] = "7.2"
    migrated["migrated_from"] = data.get("version", "7.1")

    if not hardware_config:
        logger.warning("No hardware config — cannot migrate mm to µL, keeping mm values")
        return migrated

    # Migrate commands
    for cmd in migrated.get("commands", []):
        if cmd.get("type") == "extrude" and "amount" in cmd and "amount_uL" not in cmd:
            pump = cmd.get("pump", "P1")
            amount_mm = cmd["amount"]
            pump_cfg = hardware_config.pumps.get(pump)
            if pump_cfg and pump_cfg.is_configured:
                cmd["amount_uL"] = pump_cfg.mm_to_uL(amount_mm)
                if "feedrate" in cmd:
                    cmd["rate_uL_s"] = pump_cfg.feedrate_mm_min_to_uL_s(cmd["feedrate"])
                # Keep originals for reference
                cmd["_legacy_amount_mm"] = amount_mm
                cmd.pop("amount", None)
                cmd.pop("feedrate", None)

        elif cmd.get("type") == "print_path" and "flow_rate" in cmd and "flow_rate_uL_s" not in cmd:
            # Legacy flow_rate was dimensionless — needs context to convert
            # For now, keep as-is and add flag
            cmd["flow_rate_uL_s"] = cmd.get("flow_rate", 0.01)
            cmd["_legacy_flow_rate"] = cmd.pop("flow_rate", 0.01)

    return migrated
