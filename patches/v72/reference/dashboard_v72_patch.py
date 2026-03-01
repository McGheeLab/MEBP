"""
dashboard_v72_patch.py — Modifications for dashboard.py to show pump positions in µL.

Apply these changes to the existing gui/pages/dashboard.py:

1. Add HardwareConfig import and instance variable
2. Add set_hardware_config() method
3. Modify update_data() to show pump positions in µL

CHANGES TO MAKE:
"""

# ═══════════════════════════════════════════════════════════════════
# CHANGE 1: Add import (after the existing StageController import)
# ═══════════════════════════════════════════════════════════════════
# Add this line:
#   from SupportClasses.HardwareConfig import HardwareConfig


# ═══════════════════════════════════════════════════════════════════
# CHANGE 2: Add to __init__ (after self._microsteps_per_micron = 10.0)
# ═══════════════════════════════════════════════════════════════════
# Add this line:
#   self._hardware_config: HardwareConfig | None = None


# ═══════════════════════════════════════════════════════════════════
# CHANGE 3: Add method (after set_microsteps_per_micron)
# ═══════════════════════════════════════════════════════════════════
"""
    def set_hardware_config(self, config: HardwareConfig):
        \"\"\"v7.2: Set hardware config for µL pump display.\"\"\"
        self._hardware_config = config
"""


# ═══════════════════════════════════════════════════════════════════
# CHANGE 4: Replace the ZP position display block in update_data()
# ═══════════════════════════════════════════════════════════════════
# Find this block:
"""
        # ZP position (already in mm)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position['P1']:.2f}")
            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position['P2']:.2f}")
            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position['P3']:.2f}")
"""

# Replace with:
"""
        # ZP position — Z in mm, pumps in µL (v7.2) or mm (fallback)
        zp = ctrl.get_zp_position(cached=True)
        if zp[0] is not None:
            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position['Z']:.2f}")
            for pid, lbl in [("P1", self.lbl_p1), ("P2", self.lbl_p2), ("P3", self.lbl_p3)]:
                pos_mm = zp[{"P1": 1, "P2": 2, "P3": 3}[pid]]
                zero_ref = ctrl.zero_position.get(pid, 0)
                rel_mm = pos_mm - zero_ref
                if self._hardware_config:
                    pump_cfg = self._hardware_config.pumps.get(pid)
                    if pump_cfg and pump_cfg.is_configured:
                        try:
                            pos_uL = pump_cfg.mm_to_uL(rel_mm)
                            lbl.setText(f"{pos_uL:.2f} µL")
                            continue
                        except ValueError:
                            pass
                lbl.setText(f"{rel_mm:.2f} mm")
"""


# ═══════════════════════════════════════════════════════════════════
# CHANGE 5: Update the position labels in _setup_ui()
# ═══════════════════════════════════════════════════════════════════
# Where it creates lbl_p1, lbl_p2, lbl_p3, change the unit labels from "mm" to "µL"
# For example, if there are unit labels next to P1/P2/P3, change them to "µL"
# This is cosmetic — the actual value is set in update_data()
