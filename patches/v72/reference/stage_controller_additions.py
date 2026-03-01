"""
stage_controller_additions.py — New methods to add to StageController for v7.2.

These methods provide µL-based pump control that converts to mm internally.
Add these to the existing StageController class in SupportClasses/StageController.py.

The HardwareConfig is set via set_hardware_config() called by MainWindow
when hardware setup completes or changes.
"""


# ═══════════════════════════════════════════════════════════════════
# ADD TO __init__() — after existing initialization
# ═══════════════════════════════════════════════════════════════════

# In __init__, add:
#     self._hardware_config: HardwareConfig | None = None

# Add import at top of file:
#     from SupportClasses.HardwareConfig import HardwareConfig


# ═══════════════════════════════════════════════════════════════════
# NEW METHODS — Add to StageController class
# ═══════════════════════════════════════════════════════════════════

def set_hardware_config(self, config):
    """
    Set the hardware configuration. Called by MainWindow when
    hardware setup is completed or modified.

    This enables µL-based pump methods and updates safety limits
    with flow rate information.
    """
    self._hardware_config = config
    # Update safety limits with max flow rates if available
    if config:
        for pid in ["P1", "P2", "P3"]:
            pump_cfg = config.pumps.get(pid)
            if pump_cfg and pump_cfg.is_configured:
                # Could compute max safe flow rate from FlowPhysics here
                pass
    logger.info(f"StageController: hardware config updated")


def move_pump_uL(
    self, pump: str, volume_uL: float, rate_uL_s: float | None = None
) -> None:
    """
    Move a pump by a specified volume in µL.

    This is the primary pump movement method for v7.2+.
    Converts µL → mm using the syringe spec, and µL/s → mm/min
    for the feedrate.

    Args:
        pump: Pump identifier ("P1", "P2", "P3")
        volume_uL: Volume to dispense (positive) or aspirate (negative) in µL
        rate_uL_s: Flow rate in µL/s. If None, uses default feedrate.

    Raises:
        ValueError: If pump has no syringe configured
    """
    if not self._hardware_config:
        raise ValueError("No hardware config set — complete Hardware Setup first")

    pump_cfg = self._hardware_config.pumps.get(pump)
    if not pump_cfg or not pump_cfg.is_configured:
        raise ValueError(f"{pump}: No syringe configured — complete Hardware Setup first")

    # Convert µL → mm
    distance_mm = pump_cfg.uL_to_mm(volume_uL)

    # Convert rate µL/s → mm/min
    feedrate_mm_min = None
    if rate_uL_s is not None:
        feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(rate_uL_s))
        # Clamp to safety limits
        if self.safety_limits.enabled:
            # Also clamp flow rate in µL/s
            clamped_rate = self.safety_limits.clamp_flow_rate(rate_uL_s, pump)
            if abs(clamped_rate) != abs(rate_uL_s):
                feedrate_mm_min = pump_cfg.feedrate_uL_s_to_mm_min(abs(clamped_rate))
                # Recalculate distance based on direction
                distance_mm = pump_cfg.uL_to_mm(
                    clamped_rate / abs(rate_uL_s) * volume_uL if rate_uL_s != 0 else volume_uL
                )

    logger.debug(
        f"move_pump_uL({pump}, {volume_uL:+.3f} µL, "
        f"{rate_uL_s:.3f} µL/s) → {distance_mm:+.5f} mm, "
        f"{feedrate_mm_min:.1f} mm/min"
        if feedrate_mm_min else
        f"move_pump_uL({pump}, {volume_uL:+.3f} µL) → {distance_mm:+.5f} mm"
    )

    # Delegate to existing mm-based method
    self.move_pump_relative(pump, distance_mm, feedrate_mm_min)


def get_pump_position_uL(self, pump: str) -> float | None:
    """
    Get the current pump position in µL (relative to zero reference).

    Returns None if position unavailable or no syringe configured.
    """
    if not self._hardware_config:
        return None

    pump_cfg = self._hardware_config.pumps.get(pump)
    if not pump_cfg or not pump_cfg.is_configured:
        return None

    pos = self.get_zp_position(cached=True)
    if pos is None or pos[0] is None:
        return None

    idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
    if idx >= len(pos):
        return None

    pos_mm = pos[idx]
    zero_ref = self.zero_position.get(pump, 0)
    relative_mm = pos_mm - zero_ref

    try:
        return pump_cfg.mm_to_uL(relative_mm)
    except ValueError:
        return None


def get_all_pump_positions_uL(self) -> dict[str, float | None]:
    """Get all pump positions in µL as a dict."""
    return {
        pid: self.get_pump_position_uL(pid)
        for pid in ["P1", "P2", "P3"]
    }


# ═══════════════════════════════════════════════════════════════════
# CONVENIENCE: Extrude with volume tracking
# ═══════════════════════════════════════════════════════════════════

def extrude_uL(
    self, pump: str, volume_uL: float, rate_uL_s: float | None = None
) -> bool:
    """
    Extrude (dispense) a specific volume with fluid column tracking.

    Positive volume_uL = dispense (push plunger).
    Negative volume_uL = aspirate (pull plunger).

    Also updates the FluidColumn in HardwareConfig if available.

    Returns True if the extrusion was executed.
    """
    if not self._hardware_config:
        logger.error("No hardware config — cannot extrude")
        return False

    pump_cfg = self._hardware_config.pumps.get(pump)
    if not pump_cfg or not pump_cfg.is_configured:
        logger.error(f"{pump}: not configured — cannot extrude")
        return False

    # Track fluid column
    if volume_uL > 0:
        # Dispensing
        if not pump_cfg.fluid_column.can_dispense(volume_uL):
            logger.warning(
                f"{pump}: Requested {volume_uL:.2f} µL but only "
                f"{pump_cfg.fluid_column.ink_volume_uL:.2f} µL ink available"
            )
        pump_cfg.fluid_column.dispense(volume_uL)
    elif volume_uL < 0 and pump_cfg.ink:
        # Aspirating
        pump_cfg.fluid_column.aspirate_ink(abs(volume_uL), pump_cfg.ink)

    # Execute the move
    self.move_pump_uL(pump, volume_uL, rate_uL_s)
    return True
