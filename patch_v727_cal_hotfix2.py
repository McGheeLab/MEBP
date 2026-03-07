#!/usr/bin/env python3
"""Hotfix retry — same as cal_hotfix but Fix 6 uses a smarter anchor."""

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
    print(f"\n{BOLD}  Calibration Hotfix v2{RESET}")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    # ── Fix 1: attrs (same as before, idempotent) ────────────────
    print(f"\n{CYAN}[1] __init__ attributes{RESET}")
    marker = "# v7.2.7-hotfix: ensure all calibration attrs"
    if marker in content:
        report("SKIP", "Already present")
    else:
        m = re.search(r'^(\s+)(self\._setup_ui\(\))', content, re.MULTILINE)
        if m:
            ind = m.group(1)
            inject = (
                f"{ind}{marker}\n"
                f"{ind}for _a in ['_safe_z','_top_z','_taught_a1_z','_taught_corner_z',\n"
                f"{ind}          '_taught_third','_taught_third_z','_third_well',\n"
                f"{ind}          '_z_plane_result']:\n"
                f"{ind}    if not hasattr(self, _a): setattr(self, _a, None)\n"
                f"{ind}if not hasattr(self, '_z_buffer_mm'): self._z_buffer_mm = 0.5\n"
                f"{ind}if not hasattr(self, '_corner_well'): self._corner_well = 'H12'\n\n"
            )
            content = content[:m.start()] + inject + content[m.start():]
            report("OK", "Added attrs")
        else:
            report("MISS", "self._setup_ui() not found")

    # ── Fix 2: _safe_navigate_to ─────────────────────────────────
    print(f"\n{CYAN}[2] _safe_navigate_to{RESET}")
    if "v7.2.7-hotfix: safe_navigate" in content:
        report("SKIP", "Already present")
    else:
        m = find_method(content, "_safe_navigate_to")
        if m:
            new = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-hotfix: safe_navigate — raise Z, wait, fast XY, lower Z."""
        import time
        ctrl = self.controller
        safe_z = getattr(self, '_safe_z', None) or 0.0

        # Step 1: Raise to safe Z and wait
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)
            for _ in range(30):
                time.sleep(0.1)
                zp = ctrl.get_zp_position(cached=False)
                if zp and zp[0] is not None:
                    if abs(zp[0] - ctrl.zero_position.get("Z", 0) - safe_z) < 0.1:
                        break

        # Step 2: Fast XY travel
        if ctrl.is_xy_connected:
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                try:
                    ctrl.xy_stage.set_velocity(int(min(10000, 50000)))
                except Exception:
                    pass
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)
            for _ in range(40):
                time.sleep(0.1)
                xy = ctrl.get_xy_position(cached=False)
                if xy[0] is not None:
                    if abs(xy[0] - target_x_um) < 50 and abs(xy[1] - target_y_um) < 50:
                        break

        # Step 3: Lower Z
        if target_z_mm is not None and ctrl.is_zp_connected:
            ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
        elif getattr(self, '_top_z', None) is not None and ctrl.is_zp_connected:
            ctrl.move_z_absolute(self._top_z + getattr(self, '_z_buffer_mm', 0.5),
                                 from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

'''
            content = content[:m.start()] + new + content[m.end():]
            report("OK", "Replaced")
        else:
            report("MISS", "Not found")

    # ── Fix 3: _estimate_well_position_um ────────────────────────
    print(f"\n{CYAN}[3] _estimate_well_position_um{RESET}")
    if "v7.2.7-hotfix: estimate" in content:
        report("SKIP", "Already present")
    else:
        m = find_method(content, "_estimate_well_position_um")
        if m:
            new = '''    def _estimate_well_position_um(self, well_name):
        """v7.2.7-hotfix: estimate absolute stage pos for a well from A1 + plate."""
        if self._taught_a1 is None or self._plate is None:
            return None
        try:
            wx, wy = self._plate.get_well_position(well_name)
            a1x, a1y = self._plate.get_well_position("A1")
            dx_mm = (wx - a1x) * getattr(self, '_scale', 1.0)
            dy_mm = (wy - a1y) * getattr(self, '_scale', 1.0)
            rot = getattr(self, '_rotation', 0)
            if abs(rot) > 0.001:
                rad = math.radians(rot)
                dx_mm, dy_mm = (dx_mm*math.cos(rad) - dy_mm*math.sin(rad),
                                dx_mm*math.sin(rad) + dy_mm*math.cos(rad))
            tx = self._taught_a1[0] + dx_mm * 1000.0
            ty = self._taught_a1[1] + dy_mm * 1000.0
            logger.info(f"Est {well_name}: offset ({dx_mm:.2f},{dy_mm:.2f})mm "
                        f"-> ({tx:.0f},{ty:.0f})µm")
            return (tx, ty)
        except Exception as e:
            logger.warning(f"Cannot estimate {well_name}: {e}")
            return None

'''
            content = content[:m.start()] + new + content[m.end():]
            report("OK", "Replaced")
        else:
            report("MISS", "Not found")

    # ── Fix 4: _on_plate_changed ─────────────────────────────────
    print(f"\n{CYAN}[4] _on_plate_changed{RESET}")
    if "v7.2.7-hotfix: plate_changed" in content:
        report("SKIP", "Already present")
    else:
        m = find_method(content, "_on_plate_changed")
        if m:
            new = '''    def _on_plate_changed(self, idx):
        """v7.2.7-hotfix: plate_changed — guards all widget refs."""
        sender = self.sender()
        if sender is None:
            return
        fmt = sender.currentData()
        if fmt is None:
            return
        self._plate = WellPlate.from_format(fmt)
        defn = PLATE_DEFINITIONS[fmt]
        rows, cols = defn["rows"], defn["cols"]
        self._corner_well = f"{chr(ord(\'A\') + rows - 1)}{cols}"
        if hasattr(self, 'val_well_combo'):
            self.val_well_combo.clear()
            self.val_well_combo.addItems(self._plate.well_names)
        self._taught_a1 = None
        self._taught_corner = None
        if hasattr(self, '_taught_third'): self._taught_third = None
        for attr in ['lbl_a1','lbl_corner','lbl_third']:
            if hasattr(self, attr): getattr(self, attr).setText("—")
        if hasattr(self, 'lbl_alignment'): self.lbl_alignment.setText("")
        if hasattr(self, '_cal_plate_view') and self._cal_plate_view:
            self._cal_plate_view.set_plate(self._plate)

'''
            content = content[:m.start()] + new + content[m.end():]
            report("OK", "Replaced")
        else:
            report("MISS", "Not found")

    # ── Fix 5: _try_fit_z_plane ──────────────────────────────────
    print(f"\n{CYAN}[5] _try_fit_z_plane{RESET}")
    if "v7.2.7-hotfix: try_fit" in content:
        report("SKIP", "Already present")
    else:
        m = find_method(content, "_try_fit_z_plane")
        if m:
            new = '''    def _try_fit_z_plane(self):
        """v7.2.7-hotfix: try_fit — Z-plane fit with 3 points."""
        points = []
        if getattr(self, '_taught_a1', None) and getattr(self, '_taught_a1_z', None) is not None:
            points.append(("A1", self._taught_a1_z))
        if getattr(self, '_taught_corner', None) and getattr(self, '_taught_corner_z', None) is not None:
            points.append((getattr(self, '_corner_well', 'corner'), self._taught_corner_z))
        if getattr(self, '_taught_third', None) and getattr(self, '_taught_third_z', None) is not None:
            points.append((getattr(self, '_third_well', '3rd'), self._taught_third_z))
        if len(points) < 3:
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"{len(points)}/3 points — need {3-len(points)} more")
            return
        try:
            from SupportClasses.WellSetup import WellBottomDetector
            det = WellBottomDetector(self._plate)
            for name, z in points:
                det.add_point(name, z)
            result = det.fit_plane()
            if result:
                self._z_plane_result = result
                if hasattr(self, 'lbl_zplane'):
                    self.lbl_zplane.setText(f"Z plane: {result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS[\'green\']};")
                logger.info(f"Z plane: {result.describe()}")
        except Exception as e:
            logger.error(f"Z plane fit failed: {e}")
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"Fit failed: {e}")

'''
            content = content[:m.start()] + new + content[m.end():]
            report("OK", "Replaced")
        else:
            report("MISS", "Not found")

    # ── Fix 6: Plate view on load — FIXED anchor ─────────────────
    print(f"\n{CYAN}[6] Plate view on load{RESET}")
    marker_pv = "v7.2.7-hotfix: plate view on load"
    if marker_pv in content:
        report("SKIP", "Already present")
    else:
        # Find the COMPLETE try/except block around WellPlate.from_format in _load_calibration
        # Inject AFTER the except clause, not inside the try
        load_block = re.compile(
            r'(self\._plate = WellPlate\.from_format\(cal\["plate_format"\]\).*?'
            r'(?:except.*?(?:pass|continue|logger\.\w+\([^\)]*\)))\s*\n)',
            re.DOTALL
        )
        m = load_block.search(content)
        if m:
            # Get indentation from the line before the match
            line_start = content.rfind('\n', 0, m.start()) + 1
            indent_match = re.match(r'^(\s+)', content[line_start:])
            ind = indent_match.group(1) if indent_match else "            "
            inject = (
                f"\n{ind}# {marker_pv}\n"
                f"{ind}if hasattr(self, '_cal_plate_view') and self._cal_plate_view is not None:\n"
                f"{ind}    self._cal_plate_view.set_plate(self._plate)\n"
            )
            content = content[:m.end()] + inject + content[m.end():]
            report("OK", "Added plate view update AFTER try/except")
        else:
            # Simpler: just find _load_calibration and add at the end
            m_load = find_method(content, "_load_calibration")
            if m_load:
                # Append before the method ends
                body_end = m_load.end()
                inject = (
                    "        # v7.2.7-hotfix: plate view on load\n"
                    "        if hasattr(self, '_cal_plate_view') and self._cal_plate_view and self._plate:\n"
                    "            self._cal_plate_view.set_plate(self._plate)\n\n"
                )
                content = content[:body_end] + inject + content[body_end:]
                report("OK", "Appended plate view update at end of _load_calibration")
            else:
                report("MISS", "Could not find safe injection point for plate view")

    # ── AST + write ──────────────────────────────────────────────
    print(f"\n{CYAN}AST verification...{RESET}")
    try:
        ast.parse(content)
        print(f"  {GREEN}✓ AST OK{RESET}")
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0, e.lineno-5), min(len(lines), e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727hf2_{ts}"))
    path.write_text(content, encoding="utf-8")

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")
    if miss_count == 0:
        print(f"\n{GREEN}✓ All hotfixes applied!{RESET}")
    return 0 if miss_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
