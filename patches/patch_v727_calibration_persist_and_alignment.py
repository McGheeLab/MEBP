#!/usr/bin/env python3
"""
v7.2.7: Fix _calculate_alignment, _goto_well, _save/_load_calibration.

Fixes:
  1. _calculate_alignment — unit mismatch (µm taught vs mm plate geometry)
  2. _calculate_alignment — display offset without steps_to_um
  3. _goto_well — fix coordinate transform + display
  4. _save_calibration — add new workflow fields
  5. _load_calibration — restore new fields, remove steps_to_um

Prerequisites: Run patch_v727_calibration_all_fixes.py first.
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN, RED, YELLOW, CYAN, RESET, BOLD = (
    "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[0m", "\033[1m")
ok_count = skip_count = miss_count = 0

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit(f"{RED}Cannot find MEBP root{RESET}")

def report(s, msg):
    global ok_count, skip_count, miss_count
    if s == "OK": ok_count += 1; print(f"  {GREEN}✓ {msg}{RESET}")
    elif s == "SKIP": skip_count += 1; print(f"  {YELLOW}○ {msg}{RESET}")
    else: miss_count += 1; print(f"  {RED}✗ {msg}{RESET}")

def find_method(content, name):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE)
    return pat.search(content)


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Fix alignment, goto_well, save/load calibration")
    print(f"{'='*60}{RESET}")

    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    # ── Fix 1: Replace _calculate_alignment ──────────────────────
    print(f"\n{CYAN}[1] _calculate_alignment{RESET}")
    marker1 = "v7.2.7: alignment with µm"
    if marker1 in content:
        report("SKIP", "Already patched")
    else:
        m = find_method(content, "_calculate_alignment")
        if m:
            new_method = '''    def _calculate_alignment(self):
        """v7.2.7: alignment with µm — compute scale + rotation from taught points.

        taught_a1/corner are in absolute µm. Plate geometry is in mm.
        Convert taught positions to mm relative to A1 before comparing.
        """
        if not self._taught_a1 or not self._taught_corner or not self._plate:
            self.lbl_alignment.setText(
                "\\u26a0 Record A1 and corner first, and select plate format.")
            return

        a1_expected = self._plate.get_well_position("A1")
        corner_expected = self._plate.get_well_position(self._corner_well)

        if a1_expected is None or corner_expected is None:
            self.lbl_alignment.setText(
                "\\u26a0 Cannot compute alignment for this plate format.")
            return

        # Expected vector in mm (plate geometry)
        ex = corner_expected[0] - a1_expected[0]
        ey = corner_expected[1] - a1_expected[1]

        # Measured vector: taught positions are absolute µm, convert to mm
        mx_um = self._taught_corner[0] - self._taught_a1[0]
        my_um = self._taught_corner[1] - self._taught_a1[1]
        mx_mm = mx_um / 1000.0
        my_mm = my_um / 1000.0

        expected_dist = math.sqrt(ex**2 + ey**2)
        measured_dist = math.sqrt(mx_mm**2 + my_mm**2)

        if expected_dist < 0.001:
            return

        self._scale = measured_dist / expected_dist
        expected_angle = math.atan2(ey, ex)
        measured_angle = math.atan2(my_mm, mx_mm)
        self._rotation = math.degrees(measured_angle - expected_angle)

        # Offset: A1 position relative to zero reference, in µm
        self._offset_x = self._taught_a1[0] - self.controller.zero_position["x"]
        self._offset_y = self._taught_a1[1] - self.controller.zero_position["y"]

        self.lbl_alignment.setText(
            f"\\u2705 Scale: {self._scale:.4f} | "
            f"Rot: {self._rotation:.2f}\\u00b0 | "
            f"A1 off: ({self._offset_x:,.1f}, {self._offset_y:,.1f}) \\u00b5m")
        self.lbl_alignment.setStyleSheet(f"color: {COLORS['green']};")

        if hasattr(self, 'ctx_lbl_cal_status'):
            self.ctx_lbl_cal_status.setText("\\u2705 Calibrated")
            self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")

'''
            content = content[:m.start()] + new_method + content[m.end():]
            report("OK", "Replaced _calculate_alignment with µm-aware version")
        else:
            report("MISS", "_calculate_alignment not found")

    # ── Fix 2: Replace _goto_well ────────────────────────────────
    print(f"\n{CYAN}[2] _goto_well{RESET}")
    marker2 = "v7.2.7: goto_well µm"
    if marker2 in content:
        report("SKIP", "Already patched")
    else:
        m = find_method(content, "_goto_well")
        if m:
            new_goto = '''    def _goto_well(self):
        """v7.2.7: goto_well µm — navigate to computed well position.

        Uses plate geometry (mm) + taught A1 (µm) + scale/rotation
        to compute the absolute stage position for any well.
        """
        well = self.val_well_combo.currentText()
        if not well or not self._plate or not self._taught_a1:
            return

        try:
            pos = self._plate.get_well_position(well)
        except KeyError:
            return
        if pos is None:
            return

        a1_expected = self._plate.get_well_position("A1")
        if a1_expected is None:
            return

        # Vector from A1 to target well in plate mm
        dx_mm = pos[0] - a1_expected[0]
        dy_mm = pos[1] - a1_expected[1]

        # Apply rotation
        rad = math.radians(self._rotation)
        rx = dx_mm * math.cos(rad) - dy_mm * math.sin(rad)
        ry = dx_mm * math.sin(rad) + dy_mm * math.cos(rad)

        # Apply scale, convert to µm, add A1 absolute position
        target_x_um = self._taught_a1[0] + rx * self._scale * 1000.0
        target_y_um = self._taught_a1[1] + ry * self._scale * 1000.0

        # Safe travel if safe_z is available
        if hasattr(self, '_safe_z') and self._safe_z is not None:
            if hasattr(self, '_safe_navigate_to'):
                self._safe_navigate_to(target_x_um, target_y_um)
            else:
                self.controller.move_xy_absolute(
                    target_x_um, target_y_um, from_zero_ref=False)
        else:
            self.controller.move_xy_absolute(
                target_x_um, target_y_um, from_zero_ref=False)

        # Display
        rel_x = target_x_um - self.controller.zero_position["x"]
        rel_y = target_y_um - self.controller.zero_position["y"]
        if hasattr(self, 'lbl_val_result'):
            self.lbl_val_result.setText(
                f"Moving to {well} \\u2192 ({rel_x:,.1f}, {rel_y:,.1f}) \\u00b5m")

'''
            content = content[:m.start()] + new_goto + content[m.end():]
            report("OK", "Replaced _goto_well with µm-aware version")
        else:
            report("MISS", "_goto_well not found")

    # ── Fix 3: Replace _save_calibration ─────────────────────────
    print(f"\n{CYAN}[3] _save_calibration{RESET}")
    marker3 = "v7.2.7: save new fields"
    if marker3 in content:
        report("SKIP", "Already patched")
    else:
        m = find_method(content, "_save_calibration")
        if m:
            new_save = '''    def _save_calibration(self):
        """v7.2.7: save new fields — safe_z, top_z, Z plane, third point."""
        if self.settings is None:
            return
        cal_data = {
            "plate_format": self._plate.format if self._plate else None,
            "taught_a1": list(self._taught_a1) if self._taught_a1 else None,
            "taught_corner": list(self._taught_corner) if self._taught_corner else None,
            "offset_x": self._offset_x,
            "offset_y": self._offset_y,
            "rotation": self._rotation,
            "scale": self._scale,
            # v7.2.7 additions
            "safe_z": self._safe_z if hasattr(self, '_safe_z') else None,
            "top_z": self._top_z if hasattr(self, '_top_z') else None,
            "taught_a1_z": getattr(self, '_taught_a1_z', None),
            "taught_corner_z": getattr(self, '_taught_corner_z', None),
            "taught_third": list(self._taught_third) if getattr(self, '_taught_third', None) else None,
            "taught_third_z": getattr(self, '_taught_third_z', None),
            "third_well": getattr(self, '_third_well', None),
            "corner_well": getattr(self, '_corner_well', "H12"),
        }
        # Save Z plane coefficients if fitted
        zp = getattr(self, '_z_plane_result', None)
        if zp is not None:
            cal_data["z_plane"] = {
                "a": zp.a, "b": zp.b, "c": zp.c,
                "r_squared": zp.r_squared,
            }
        self.settings.set_section("calibration", cal_data)
        self.settings.save()
        logger.info("Calibration saved to settings (v7.2.7)")
        if hasattr(self, 'ctx_lbl_cal_status'):
            self.ctx_lbl_cal_status.setText("\\u2705 Saved")
            self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")

'''
            content = content[:m.start()] + new_save + content[m.end():]
            report("OK", "Replaced _save_calibration with v7.2.7 fields")
        else:
            report("MISS", "_save_calibration not found")

    # ── Fix 4: Replace _load_calibration ─────────────────────────
    print(f"\n{CYAN}[4] _load_calibration{RESET}")
    marker4 = "v7.2.7: load new fields"
    if marker4 in content:
        report("SKIP", "Already patched")
    else:
        m = find_method(content, "_load_calibration")
        if m:
            new_load = '''    def _load_calibration(self):
        """v7.2.7: load new fields — safe_z, top_z, Z plane, third point."""
        if self.settings is None:
            return
        cal = self.settings.get_section("calibration")
        if not cal:
            return

        if cal.get("plate_format"):
            try:
                self._plate = WellPlate.from_format(cal["plate_format"])
                wells = self._plate.well_names
                if hasattr(self, 'val_well_combo'):
                    self.val_well_combo.clear()
                    self.val_well_combo.addItems(wells)
            except (ValueError, KeyError):
                pass

        if cal.get("taught_a1"):
            self._taught_a1 = tuple(cal["taught_a1"])
            # Display zero-ref position directly (positions are µm)
            zero_x = self.controller.zero_position.get('x', 0)
            zero_y = self.controller.zero_position.get('y', 0)
            ax = self._taught_a1[0] - zero_x
            ay = self._taught_a1[1] - zero_y
            if hasattr(self, 'lbl_a1'):
                z_str = ""
                if cal.get("taught_a1_z") is not None:
                    z_str = f"  Z: {cal['taught_a1_z']:.2f} mm"
                self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) \\u00b5m{z_str}")
                self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")

        if cal.get("taught_corner"):
            self._taught_corner = tuple(cal["taught_corner"])
            zero_x = self.controller.zero_position.get('x', 0)
            zero_y = self.controller.zero_position.get('y', 0)
            cx = self._taught_corner[0] - zero_x
            cy = self._taught_corner[1] - zero_y
            if hasattr(self, 'lbl_corner'):
                z_str = ""
                if cal.get("taught_corner_z") is not None:
                    z_str = f"  Z: {cal['taught_corner_z']:.2f} mm"
                self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) \\u00b5m{z_str}")
                self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")

        self._offset_x = cal.get("offset_x", 0)
        self._offset_y = cal.get("offset_y", 0)
        self._rotation = cal.get("rotation", 0)
        self._scale = cal.get("scale", 1.0)

        # v7.2.7 fields
        if hasattr(self, '_safe_z'):
            self._safe_z = cal.get("safe_z")
            if self._safe_z is not None and hasattr(self, 'lbl_safe_z'):
                self.lbl_safe_z.setText(f"Safe Z: {self._safe_z:.2f} mm")
                self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")

        if hasattr(self, '_top_z'):
            self._top_z = cal.get("top_z")
            if self._top_z is not None and hasattr(self, 'lbl_top_z'):
                self.lbl_top_z.setText(f"Top Z: {self._top_z:.2f} mm")
                self.lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")

        self._taught_a1_z = cal.get("taught_a1_z")
        self._taught_corner_z = cal.get("taught_corner_z")

        if cal.get("taught_third"):
            self._taught_third = tuple(cal["taught_third"])
            self._taught_third_z = cal.get("taught_third_z")
            self._third_well = cal.get("third_well")

        self._corner_well = cal.get("corner_well", getattr(self, '_corner_well', 'H12'))

        # Restore Z plane
        zp_data = cal.get("z_plane")
        if zp_data:
            try:
                from SupportClasses.WellSetup import PlaneResult
                self._z_plane_result = PlaneResult(
                    a=zp_data["a"], b=zp_data["b"], c=zp_data["c"],
                    r_squared=zp_data.get("r_squared", 0),
                    num_points=3,
                )
                if hasattr(self, 'lbl_zplane'):
                    self.lbl_zplane.setText(f"Z plane: {self._z_plane_result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS['green']};")
            except (ImportError, KeyError) as e:
                logger.debug(f"Could not restore Z plane: {e}")

        if self._scale != 1.0 or self._rotation != 0:
            if hasattr(self, 'lbl_alignment'):
                self.lbl_alignment.setText(
                    f"Loaded | Scale: {self._scale:.4f} | "
                    f"Rotation: {self._rotation:.2f}\\u00b0")
            if hasattr(self, 'ctx_lbl_cal_status'):
                self.ctx_lbl_cal_status.setText("\\u2705 Loaded from settings")
                self.ctx_lbl_cal_status.setStyleSheet(f"color: {COLORS['green']};")

        logger.info("Calibration loaded from settings (v7.2.7)")

'''
            content = content[:m.start()] + new_load + content[m.end():]
            report("OK", "Replaced _load_calibration with v7.2.7 version")
        else:
            report("MISS", "_load_calibration not found")

    # ── AST verify + write ───────────────────────────────────────
    print(f"\n{CYAN}AST verification...{RESET}")
    try:
        ast.parse(content)
        print(f"  {GREEN}✓ AST OK{RESET}")
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0, e.lineno-5), min(len(lines), e.lineno+3)):
            mk = ">>>" if i == e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727pa_{ts}"))
    path.write_text(content, encoding="utf-8")

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention{RESET}")
    else:
        print(f"\n{GREEN}✓ All persistence + alignment fixes applied!{RESET}")
    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
