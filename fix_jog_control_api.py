#!/usr/bin/env python3
"""
fix_jog_control_api.py — Fix jog_control.py method calls to match actual StageController API.

Issues:
1. on_status_update(xy_pos, zp_pos) → on_status_update() [fetch positions internally]
2. move_xy_relative(dx, dy) → move_xy_absolute with offset from current position
3. set_zero_xy() → manually set zero_position from current XY
4. set_zero_z() → manually set zero_position["Z"] from current Z
5. set_zero_pump(pump) → manually set zero_position[pump] from current pump pos

Run from MEBP project root:
    python fix_jog_control_api.py
"""

import os, sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

path = "gui/pages/jog_control.py"
if not os.path.isfile(path):
    print(f"{RED}ERROR{RESET}: {path} not found"); sys.exit(1)

with open(path) as f:
    content = f.read()

count = 0

# Fix 1: on_status_update signature — remove args, fetch internally
old = '''    def on_status_update(self, xy_pos, zp_pos):
        """
        Update position readouts.
        xy_pos: (x_steps, y_steps) or (None, None)
        zp_pos: (z_mm, p1_mm, p2_mm, p3_mm) or (None, None, None, None)
        """
        # XY in µm
        if xy_pos and xy_pos[0] is not None:
            x_um = xy_pos[0] / self._microsteps_per_micron
            y_um = xy_pos[1] / self._microsteps_per_micron
            self.x_readout.setText(f"{x_um:.1f} µm")
            self.y_readout.setText(f"{y_um:.1f} µm")

        # Z in mm
        if zp_pos and zp_pos[0] is not None:
            self.z_readout.setText(f"{zp_pos[0]:.3f} mm")

        # Pumps in µL
        if zp_pos:
            for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
                pos_mm = zp_pos[idx] if idx < len(zp_pos) else None
                lbl = self.pump_readouts.get(pid)
                if lbl is None:
                    continue

                if pos_mm is not None and self._hw_config:
                    pump_cfg = self._hw_config.pumps.get(pid)
                    if pump_cfg and pump_cfg.is_configured:
                        try:
                            pos_uL = pump_cfg.mm_to_uL(pos_mm)
                            lbl.setText(f"{pos_uL:.2f} µL")
                            continue
                        except ValueError:
                            pass
                    lbl.setText(f"{pos_mm:.3f} mm")
                elif pos_mm is not None:
                    lbl.setText(f"{pos_mm:.3f} mm")
                else:
                    lbl.setText("— µL")'''

new = '''    def on_status_update(self):
        """
        Update position readouts. Fetches positions from controller.
        Called by MainWindow timer with no arguments.
        """
        ctrl = self.controller

        # XY in µm (relative to zero)
        xy_pos = ctrl.get_xy_position(cached=True)
        if xy_pos[0] is not None:
            x_rel = xy_pos[0] - ctrl.zero_position["x"]
            y_rel = xy_pos[1] - ctrl.zero_position["y"]
            x_um = x_rel / self._microsteps_per_micron
            y_um = y_rel / self._microsteps_per_micron
            self.x_readout.setText(f"{x_um:.1f} µm")
            self.y_readout.setText(f"{y_um:.1f} µm")

        # Z in mm (relative to zero)
        zp_pos = ctrl.get_zp_position(cached=True)
        if isinstance(zp_pos, (list, tuple)) and len(zp_pos) >= 1 and zp_pos[0] is not None:
            z_rel = zp_pos[0] - ctrl.zero_position.get("Z", 0)
            self.z_readout.setText(f"{z_rel:.3f} mm")

        # Pumps in µL (relative to zero)
        if isinstance(zp_pos, (list, tuple)) and len(zp_pos) >= 4:
            for idx, pid in enumerate(["P1", "P2", "P3"], start=1):
                pos_mm = zp_pos[idx] if idx < len(zp_pos) else None
                lbl = self.pump_readouts.get(pid)
                if lbl is None:
                    continue

                if pos_mm is not None:
                    pos_rel = pos_mm - ctrl.zero_position.get(pid, 0)
                    if self._hw_config:
                        pump_cfg = self._hw_config.pumps.get(pid)
                        if pump_cfg and pump_cfg.is_configured:
                            try:
                                pos_uL = pump_cfg.mm_to_uL(pos_rel)
                                lbl.setText(f"{pos_uL:.2f} µL")
                                continue
                            except (ValueError, AttributeError):
                                pass
                    lbl.setText(f"{pos_rel:.3f} mm")
                else:
                    lbl.setText("— µL")'''

if old in content:
    content = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}: Fixed on_status_update() signature and body")
    count += 1
else:
    print(f"  {RED}SKIP{RESET}: on_status_update pattern not found")


# Fix 2: move_xy_relative → move_xy_absolute with offset
old2 = '        self.controller.move_xy_relative(dx_steps, dy_steps)'
new2 = '''        # XY relative move: get current pos, compute absolute target
        xy = self.controller.get_xy_position(cached=True)
        if xy[0] is None:
            logger.warning("Cannot jog XY: no position available")
            return
        target_x = xy[0] + dx_steps - self.controller.zero_position["x"]
        target_y = xy[1] + dy_steps - self.controller.zero_position["y"]
        self.controller.move_xy_absolute(target_x, target_y, from_zero_ref=True)'''

if old2 in content:
    content = content.replace(old2, new2, 1)
    print(f"  {GREEN}OK{RESET}: Fixed move_xy_relative → move_xy_absolute")
    count += 1
else:
    print(f"  {RED}SKIP{RESET}: move_xy_relative not found")


# Fix 3: set_zero_xy, set_zero_z, set_zero_pump
old3 = '''    def _set_zero_xy(self):
        self.controller.set_zero_xy()

    def _set_zero_z(self):
        self.controller.set_zero_z()

    def _set_zero_pump(self, pump: str):
        self.controller.set_zero_pump(pump)'''

new3 = '''    def _set_zero_xy(self):
        """Set current XY position as zero reference."""
        xy = self.controller.get_xy_position(cached=False)
        if xy[0] is not None:
            self.controller.zero_position["x"] = xy[0]
            self.controller.zero_position["y"] = xy[1]
            logger.info(f"XY zero set: ({xy[0]:.1f}, {xy[1]:.1f})")

    def _set_zero_z(self):
        """Set current Z position as zero reference."""
        zp = self.controller.get_zp_position(cached=False)
        if isinstance(zp, (list, tuple)) and len(zp) >= 1 and zp[0] is not None:
            self.controller.zero_position["Z"] = zp[0]
            logger.info(f"Z zero set: {zp[0]:.3f} mm")

    def _set_zero_pump(self, pump: str):
        """Set current pump position as zero reference."""
        zp = self.controller.get_zp_position(cached=False)
        idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
        if isinstance(zp, (list, tuple)) and len(zp) > idx and zp[idx] is not None:
            self.controller.zero_position[pump] = zp[idx]
            logger.info(f"{pump} zero set: {zp[idx]:.3f} mm")'''

if old3 in content:
    content = content.replace(old3, new3, 1)
    print(f"  {GREEN}OK{RESET}: Fixed set_zero_xy/z/pump to use actual API")
    count += 1
else:
    print(f"  {RED}SKIP{RESET}: set_zero pattern not found")


with open(path, "w") as f:
    f.write(content)

print(f"\n{count} fixes applied to jog_control.py")
