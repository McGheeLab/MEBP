#!/usr/bin/env python3
"""
v7.2.7 Calibration Hotfix — fix all runtime AttributeErrors + nav bugs.

Fixes:
  1. Missing attrs: _top_z, _taught_third, _third_well, _z_plane_result, etc.
  2. Missing lbl_alignment — add hasattr guards or create it
  3. _safe_navigate_to — wait for Z settle, actually move XY properly
  4. _estimate_well_position_um — fix math for correct corner estimation
  5. Plate view black — ensure set_plate called on plate change
  6. _on_plate_changed — guard all widget references
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
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir(): return c
    sys.exit(f"{RED}Cannot find MEBP root{RESET}")

def report(s, msg):
    global ok_count, skip_count, miss_count
    if s=="OK": ok_count+=1; print(f"  {GREEN}✓ {msg}{RESET}")
    elif s=="SKIP": skip_count+=1; print(f"  {YELLOW}○ {msg}{RESET}")
    else: miss_count+=1; print(f"  {RED}✗ {msg}{RESET}")

def find_method(content, name):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE)
    return pat.search(content)


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Calibration Hotfix — Runtime Error Fixes")
    print(f"{'='*60}{RESET}")

    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    # ── Fix 1: Ensure ALL required attributes exist in __init__ ──
    print(f"\n{CYAN}[1] Ensure __init__ attributes{RESET}")
    # Find __init__ method and inject a block that sets all attrs safely
    # Strategy: find "self._setup_ui()" call in __init__ and inject BEFORE it
    setup_ui_pat = re.compile(r'^(\s+)(self\._setup_ui\(\))', re.MULTILINE)
    m_setup = setup_ui_pat.search(content)
    
    marker_init = "# v7.2.7-hotfix: ensure all calibration attrs"
    if marker_init in content:
        report("SKIP", "Init attrs already hotfixed")
    elif m_setup:
        ind = m_setup.group(1)
        inject = (
            f"{ind}{marker_init}\n"
            f"{ind}if not hasattr(self, '_safe_z'): self._safe_z = None\n"
            f"{ind}if not hasattr(self, '_top_z'): self._top_z = None\n"
            f"{ind}if not hasattr(self, '_taught_a1_z'): self._taught_a1_z = None\n"
            f"{ind}if not hasattr(self, '_taught_corner_z'): self._taught_corner_z = None\n"
            f"{ind}if not hasattr(self, '_taught_third'): self._taught_third = None\n"
            f"{ind}if not hasattr(self, '_taught_third_z'): self._taught_third_z = None\n"
            f"{ind}if not hasattr(self, '_third_well'): self._third_well = None\n"
            f"{ind}if not hasattr(self, '_z_plane_result'): self._z_plane_result = None\n"
            f"{ind}if not hasattr(self, '_z_buffer_mm'): self._z_buffer_mm = 0.5\n"
            f"{ind}if not hasattr(self, '_corner_well'): self._corner_well = 'H12'\n"
            f"\n"
        )
        content = content[:m_setup.start()] + inject + content[m_setup.start():]
        report("OK", "Added all missing attributes before _setup_ui()")
    else:
        report("MISS", "Could not find self._setup_ui() in __init__")

    # ── Fix 2: Replace _safe_navigate_to with working version ────
    print(f"\n{CYAN}[2] Fix _safe_navigate_to{RESET}")
    marker_nav = "v7.2.7-hotfix: safe_navigate"
    if marker_nav in content:
        report("SKIP", "Already hotfixed")
    else:
        m_nav = find_method(content, "_safe_navigate_to")
        if m_nav:
            new_nav = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-hotfix: safe_navigate — raise Z, wait, fast XY, lower Z.

        1. Raise to Safe Z (or 0 if not set) and WAIT for completion
        2. Fast XY travel to target (absolute µm, from_zero_ref=False)
        3. Lower to target_z or (Top Z + buffer) if available
        """
        import time
        ctrl = self.controller
        safe_z = getattr(self, '_safe_z', None)
        if safe_z is None:
            safe_z = 0.0

        # Step 1: Raise to safe Z and wait for Z to reach position
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)
            # Wait for Z to actually reach safe height
            for _ in range(30):  # up to 3 seconds
                time.sleep(0.1)
                zp = ctrl.get_zp_position(cached=False)
                if zp is not None and zp[0] is not None:
                    current_z = zp[0] - ctrl.zero_position.get("Z", 0)
                    if abs(current_z - safe_z) < 0.1:  # within 0.1mm
                        break

        # Step 2: Set fast XY speed and travel
        if ctrl.is_xy_connected:
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                try:
                    ctrl.xy_stage.set_velocity(int(min(10000, 50000)))
                except Exception:
                    pass
            # target is absolute µm — use from_zero_ref=False
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)
            # Wait for XY to settle
            for _ in range(40):  # up to 4 seconds
                time.sleep(0.1)
                xy = ctrl.get_xy_position(cached=False)
                if xy[0] is not None:
                    if (abs(xy[0] - target_x_um) < 50 and
                            abs(xy[1] - target_y_um) < 50):  # within 50µm
                        break

        # Step 3: Lower Z if target provided
        if target_z_mm is not None and ctrl.is_zp_connected:
            ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
        elif getattr(self, '_top_z', None) is not None and ctrl.is_zp_connected:
            approach_z = self._top_z + getattr(self, '_z_buffer_mm', 0.5)
            ctrl.move_z_absolute(approach_z, from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

'''
            content = content[:m_nav.start()] + new_nav + content[m_nav.end():]
            report("OK", "Replaced _safe_navigate_to with waiting version")
        else:
            report("MISS", "_safe_navigate_to not found")

    # ── Fix 3: Replace _estimate_well_position_um ────────────────
    print(f"\n{CYAN}[3] Fix _estimate_well_position_um{RESET}")
    marker_est = "v7.2.7-hotfix: estimate"
    if marker_est in content:
        report("SKIP", "Already hotfixed")
    else:
        m_est = find_method(content, "_estimate_well_position_um")
        if m_est:
            new_est = '''    def _estimate_well_position_um(self, well_name):
        """v7.2.7-hotfix: estimate absolute stage position for a well.

        Uses taught A1 absolute position + plate geometry offset.
        Plate geometry: well positions in mm relative to A1 center.
        A1 is at (0,0) in plate coordinates.

        Returns (x_um, y_um) in absolute stage µm, or None.
        """
        if self._taught_a1 is None or self._plate is None:
            logger.warning("Cannot estimate: A1 not taught or no plate")
            return None
        try:
            # Well position relative to A1 in mm (A1 itself is 0,0)
            well_x_mm, well_y_mm = self._plate.get_well_position(well_name)
            a1_x_mm, a1_y_mm = self._plate.get_well_position("A1")
            # Offset from A1 in mm
            dx_mm = well_x_mm - a1_x_mm
            dy_mm = well_y_mm - a1_y_mm
            # Apply rotation if calibrated
            rot = getattr(self, '_rotation', 0)
            if abs(rot) > 0.001:
                import math
                rad = math.radians(rot)
                rx = dx_mm * math.cos(rad) - dy_mm * math.sin(rad)
                ry = dx_mm * math.sin(rad) + dy_mm * math.cos(rad)
                dx_mm, dy_mm = rx, ry
            # Apply scale if calibrated
            scale = getattr(self, '_scale', 1.0)
            dx_mm *= scale
            dy_mm *= scale
            # Convert mm offset to µm and add to taught A1 absolute position
            target_x = self._taught_a1[0] + dx_mm * 1000.0
            target_y = self._taught_a1[1] + dy_mm * 1000.0
            logger.info(f"Estimated {well_name}: plate offset ({dx_mm:.2f}, {dy_mm:.2f}) mm "
                        f"-> stage ({target_x:.0f}, {target_y:.0f}) µm")
            return (target_x, target_y)
        except (KeyError, TypeError) as e:
            logger.warning(f"Cannot estimate position for {well_name}: {e}")
            return None

'''
            content = content[:m_est.start()] + new_est + content[m_est.end():]
            report("OK", "Replaced _estimate_well_position_um with logged version")
        else:
            report("MISS", "_estimate_well_position_um not found")

    # ── Fix 4: Guard _on_plate_changed widget references ─────────
    print(f"\n{CYAN}[4] Guard _on_plate_changed{RESET}")
    marker_plate = "v7.2.7-hotfix: plate_changed"
    if marker_plate in content:
        report("SKIP", "Already hotfixed")
    else:
        m_plate = find_method(content, "_on_plate_changed")
        if m_plate:
            new_plate = '''    def _on_plate_changed(self, idx):
        """v7.2.7-hotfix: plate_changed — handle plate format change with widget guards."""
        sender = self.sender()
        if sender is None:
            return

        fmt = sender.currentData()
        if fmt is None:
            return

        self._plate = WellPlate.from_format(fmt)
        defn = PLATE_DEFINITIONS[fmt]
        rows, cols = defn["rows"], defn["cols"]

        row_letter = chr(ord('A') + rows - 1)
        self._corner_well = f"{row_letter}{cols}"

        wells = self._plate.well_names
        if hasattr(self, 'val_well_combo'):
            self.val_well_combo.clear()
            self.val_well_combo.addItems(wells)

        # Reset taught positions
        self._taught_a1 = None
        self._taught_corner = None
        if hasattr(self, '_taught_third'):
            self._taught_third = None
        if hasattr(self, 'lbl_a1'):
            self.lbl_a1.setText("—")
        if hasattr(self, 'lbl_corner'):
            self.lbl_corner.setText("—")
        if hasattr(self, 'lbl_alignment'):
            self.lbl_alignment.setText("")
        if hasattr(self, 'lbl_third'):
            self.lbl_third.setText("—")

        # Update plate view
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:
            self._cal_plate_view.set_plate(self._plate)

'''
            content = content[:m_plate.start()] + new_plate + content[m_plate.end():]
            report("OK", "Replaced _on_plate_changed with guarded version + plate view update")
        else:
            report("MISS", "_on_plate_changed not found")

    # ── Fix 5: _try_fit_z_plane — guard all attr access ──────────
    print(f"\n{CYAN}[5] Guard _try_fit_z_plane{RESET}")
    marker_fit = "v7.2.7-hotfix: try_fit"
    if marker_fit in content:
        report("SKIP", "Already hotfixed")
    else:
        m_fit = find_method(content, "_try_fit_z_plane")
        if m_fit:
            new_fit = '''    def _try_fit_z_plane(self):
        """v7.2.7-hotfix: try_fit — attempt Z-plane fit if 3 points available."""
        points = []
        if getattr(self, '_taught_a1', None) is not None and getattr(self, '_taught_a1_z', None) is not None:
            points.append(("A1", self._taught_a1_z))
        if getattr(self, '_taught_corner', None) is not None and getattr(self, '_taught_corner_z', None) is not None:
            points.append((getattr(self, '_corner_well', 'corner'), self._taught_corner_z))
        if getattr(self, '_taught_third', None) is not None and getattr(self, '_taught_third_z', None) is not None:
            points.append((getattr(self, '_third_well', '3rd'), self._taught_third_z))

        if len(points) < 3:
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"{len(points)}/3 points — need {3-len(points)} more")
            return

        try:
            from SupportClasses.WellSetup import WellBottomDetector
            detector = WellBottomDetector(self._plate)
            for name, z in points:
                detector.add_point(name, z)
            result = detector.fit_plane()
            if result:
                self._z_plane_result = result
                if hasattr(self, 'lbl_zplane'):
                    self.lbl_zplane.setText(f"Z plane: {result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS['green']};")
                logger.info(f"Z plane fitted: {result.describe()}")
        except Exception as e:
            logger.error(f"Z plane fit failed: {e}")
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"Fit failed: {e}")
                self.lbl_zplane.setStyleSheet(f"color: {COLORS['red']};")

'''
            content = content[:m_fit.start()] + new_fit + content[m_fit.end():]
            report("OK", "Replaced _try_fit_z_plane with guarded version")
        else:
            report("MISS", "_try_fit_z_plane not found")

    # ── Fix 6: Ensure plate view gets plate on load_calibration ──
    print(f"\n{CYAN}[6] Plate view init on load{RESET}")
    marker_pv = "v7.2.7-hotfix: plate view on load"
    if marker_pv in content:
        report("SKIP", "Already hotfixed")
    else:
        # Find _load_calibration's plate format block and add plate view update
        load_plate_pat = re.compile(
            r"(self\._plate = WellPlate\.from_format\(cal\[\"plate_format\"\]\))"
        )
        m_lp = load_plate_pat.search(content)
        if m_lp:
            inject = (
                "\n            # v7.2.7-hotfix: plate view on load\n"
                "            if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:\n"
                "                self._cal_plate_view.set_plate(self._plate)\n"
            )
            content = content[:m_lp.end()] + inject + content[m_lp.end():]
            report("OK", "Added plate view update on calibration load")
        else:
            report("MISS", "Could not find plate loading in _load_calibration")

    # ── Fix 7: _goto_corner_auto / _goto_third_auto — guard attrs
    print(f"\n{CYAN}[7] Guard auto-navigate methods{RESET}")
    for method_name in ['_goto_corner_auto', '_goto_third_auto', '_record_corner_xyz']:
        m = find_method(content, method_name)
        if m:
            body = m.group(0)
            # Replace bare self._taught_third with getattr
            if 'self._taught_third' in body and 'getattr' not in body:
                body = body.replace(
                    'self._taught_third is not None',
                    'getattr(self, "_taught_third", None) is not None'
                )
            # Replace bare self._top_z with getattr
            if 'self._top_z' in body and 'getattr' not in body:
                body = body.replace(
                    'self._top_z is not None',
                    'getattr(self, "_top_z", None) is not None'
                )
            if body != m.group(0):
                content = content[:m.start()] + body + content[m.end():]
                report("OK", f"Guarded attrs in {method_name}")

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
    shutil.copy2(path, path.with_suffix(f".bak_v727hf_{ts}"))
    path.write_text(content, encoding="utf-8")

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")
    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention{RESET}")
    else:
        print(f"\n{GREEN}✓ All hotfixes applied!{RESET}")
    return 0 if miss_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
