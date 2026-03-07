#!/usr/bin/env python3
"""
v7.2.7: New calibration workflow — Safe Z, Top Z, 3-point Z-plane,
auto-navigate, and clickable plate view.

Prerequisites: Run patch_v727_calibration_all_fixes.py FIRST.

Changes:
  1. Add _safe_z, _top_z, _taught_third, Z-plane state attributes
  2. Replace Step 2 context panel UI with new guided workflow
  3. Add _safe_navigate_to() helper method
  4. Add auto-estimate corner/third well positions from A1 + plate geometry
  5. Make _CalibrationPlateView clickable with well_clicked signal
  6. Add taught-point markers and live needle position to plate view
  7. Integrate WellBottomDetector for Z-plane fitting
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


# ═══════════════════════════════════════════════════════════════════
# PATCH 1: Add state attributes to __init__
# ═══════════════════════════════════════════════════════════════════

def patch_1_init_attrs(content):
    """Add Safe Z / Top Z / third point / Z-plane attributes."""
    print(f"\n{CYAN}[1] __init__ state attributes{RESET}")
    marker = "v7.2.7: calibration workflow state"
    if marker in content:
        report("SKIP", "Already present")
        return content

    # Find the existing taught attrs initialization
    pat = re.compile(
        r'(self\._taught_a1\s*=\s*None\s*\n'
        r'\s+self\._taught_corner\s*=\s*None)'
    )
    m = pat.search(content)
    if not m:
        report("MISS", "Could not find _taught_a1/_taught_corner init")
        return content

    inject = (
        "\n"
        "        # v7.2.7: calibration workflow state\n"
        "        self._safe_z = None          # mm (zero-ref) — safe travel height\n"
        "        self._top_z = None           # mm (zero-ref) — plate top surface\n"
        "        self._taught_a1_z = None     # mm (zero-ref) — Z at A1 glass bottom\n"
        "        self._taught_corner_z = None # mm (zero-ref) — Z at corner\n"
        "        self._taught_third = None    # (x_um, y_um) absolute — third teach point\n"
        "        self._taught_third_z = None  # mm (zero-ref)\n"
        "        self._third_well = None      # well name for third point\n"
        "        self._z_plane_result = None  # PlaneResult from WellBottomDetector\n"
        "        self._z_buffer_mm = 0.5      # approach buffer above Top Z\n"
    )
    content = content[:m.end()] + inject + content[m.end():]
    report("OK", "Added workflow state attributes")
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH 2: Add _safe_navigate_to() helper
# ═══════════════════════════════════════════════════════════════════

def patch_2_safe_navigate(content):
    """Add the safe navigation helper method."""
    print(f"\n{CYAN}[2] _safe_navigate_to() method{RESET}")
    marker = "def _safe_navigate_to"
    if marker in content:
        report("SKIP", "Already present")
        return content

    # Insert before _set_zero
    insert_pat = re.compile(r'^    def _set_zero\(self\)', re.MULTILINE)
    m = insert_pat.search(content)
    if not m:
        report("MISS", "Could not find _set_zero for insertion")
        return content

    method = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7: Safe travel — raise Z, fast XY move, lower Z.

        Pattern:
          1. Raise to Safe Z (or zero if not set)
          2. Fast XY travel to target
          3. Lower to target_z or (Top Z - buffer)
        """
        ctrl = self.controller
        safe_z = self._safe_z if self._safe_z is not None else 0.0

        # Step 1: Raise to safe Z
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)
            import time; time.sleep(0.5)

        # Step 2: Set fast XY speed and travel
        if ctrl.is_xy_connected and hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
            try:
                ctrl.xy_stage.set_velocity(int(min(10000, 50000)))
            except Exception:
                pass
            # target is absolute µm — use from_zero_ref=False
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)
            # Wait for settle
            import time
            time.sleep(1.0)

        # Step 3: Lower Z
        if target_z_mm is not None and ctrl.is_zp_connected:
            ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
        elif self._top_z is not None and ctrl.is_zp_connected:
            approach_z = self._top_z + self._z_buffer_mm
            ctrl.move_z_absolute(approach_z, from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

    def _estimate_well_position_um(self, well_name):
        """v7.2.7: Estimate absolute stage position for a well from A1 + plate geometry.

        Returns (x_um, y_um) or None if A1 not taught.
        """
        if self._taught_a1 is None or self._plate is None:
            return None
        try:
            # Plate gives well position in mm relative to A1
            rel_x_mm, rel_y_mm = self._plate.get_well_position(well_name)
            # A1 position in mm relative to A1 is (0, 0)
            a1_rel_x, a1_rel_y = self._plate.get_well_position("A1")
            # Offset from A1 in mm
            dx_mm = rel_x_mm - a1_rel_x
            dy_mm = rel_y_mm - a1_rel_y
            # Convert to µm and add to taught A1 absolute position
            target_x = self._taught_a1[0] + dx_mm * 1000.0
            target_y = self._taught_a1[1] + dy_mm * 1000.0
            return (target_x, target_y)
        except (KeyError, TypeError) as e:
            logger.warning(f"Cannot estimate position for {well_name}: {e}")
            return None

    def _pick_third_well(self):
        """v7.2.7: Choose a third teach point that maximizes triangle area.

        Given A1 and the diagonal corner, pick a well on the other diagonal.
        For 96-well H12 corner: third = A12 or H1.
        """
        if not self._plate:
            return None
        rows = self._plate.rows
        cols = self._plate.cols
        corner_letter = chr(ord('A') + rows - 1)
        # If corner is bottom-right (e.g., H12), pick top-right (A12)
        third = f"A{cols}"
        # If that's A1, pick bottom-left instead
        if third == "A1":
            third = f"{corner_letter}1"
        return third

'''
    content = content[:m.start()] + method + content[m.start():]
    report("OK", "Added _safe_navigate_to + helpers")
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH 3: Add Z teaching methods
# ═══════════════════════════════════════════════════════════════════

def patch_3_z_teaching(content):
    """Add _set_safe_z, _set_top_z, _record_a1_xyz, auto-navigate, Z-plane fit."""
    print(f"\n{CYAN}[3] Z teaching methods{RESET}")
    marker = "def _set_safe_z"
    if marker in content:
        report("SKIP", "Already present")
        return content

    # Insert before _on_plate_changed
    insert_pat = re.compile(r'^    def _on_plate_changed\(self', re.MULTILINE)
    m = insert_pat.search(content)
    if not m:
        report("MISS", "Could not find _on_plate_changed for insertion")
        return content

    methods = '''    def _set_safe_z(self):
        """v7.2.7: Record current Z as safe travel height."""
        zp = self.controller.get_zp_position(cached=False)
        if zp is None or zp[0] is None:
            return
        self._safe_z = zp[0] - self.controller.zero_position.get("Z", 0)
        if hasattr(self, 'lbl_safe_z'):
            self.lbl_safe_z.setText(f"Safe Z: {self._safe_z:.2f} mm")
            self.lbl_safe_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Safe Z set: {self._safe_z:.2f} mm")

    def _set_top_z(self):
        """v7.2.7: Record current Z as plate top surface."""
        zp = self.controller.get_zp_position(cached=False)
        if zp is None or zp[0] is None:
            return
        self._top_z = zp[0] - self.controller.zero_position.get("Z", 0)
        if hasattr(self, 'lbl_top_z'):
            self.lbl_top_z.setText(f"Top Z: {self._top_z:.2f} mm")
            self.lbl_top_z.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Top Z set: {self._top_z:.2f} mm")

    def _record_a1_xyz(self):
        """v7.2.7: Record A1 with full XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_a1 = (xy[0], xy[1])
        ax = xy[0] - self.controller.zero_position["x"]
        ay = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_a1_z = zp[0] - self.controller.zero_position.get("Z", 0)
        z_str = f"  Z: {self._taught_a1_z:.2f} mm" if self._taught_a1_z is not None else ""
        if hasattr(self, 'lbl_a1'):
            self.lbl_a1.setText(f"({ax:,.1f}, {ay:,.1f}) µm{z_str}")
            self.lbl_a1.setStyleSheet(f"color: {COLORS['green']};")
        # Update plate view
        if hasattr(self, '_cal_plate_view'):
            self._cal_plate_view.set_taught_a1((ax / 1000.0, ay / 1000.0))
            self._cal_plate_view._rebuild()
        logger.info(f"Taught A1 XYZ: {self._taught_a1}, Z={self._taught_a1_z}")

    def _goto_corner_auto(self):
        """v7.2.7: Auto-navigate to estimated corner position with safe travel."""
        if self._taught_a1 is None or self._plate is None:
            QMessageBox.warning(self, "Cannot Navigate",
                                "Record A1 and select plate format first.")
            return
        est = self._estimate_well_position_um(self._corner_well)
        if est is None:
            return
        self._safe_navigate_to(est[0], est[1])
        if hasattr(self, '_gen_status'):
            self._gen_status.setText(f"Navigated to {self._corner_well} estimate")

    def _record_corner_xyz(self):
        """v7.2.7: Record corner with full XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_corner = (xy[0], xy[1])
        cx = xy[0] - self.controller.zero_position["x"]
        cy = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_corner_z = zp[0] - self.controller.zero_position.get("Z", 0)
        z_str = f"  Z: {self._taught_corner_z:.2f} mm" if self._taught_corner_z is not None else ""
        if hasattr(self, 'lbl_corner'):
            self.lbl_corner.setText(f"({cx:,.1f}, {cy:,.1f}) µm{z_str}")
            self.lbl_corner.setStyleSheet(f"color: {COLORS['green']};")
        if hasattr(self, '_cal_plate_view'):
            self._cal_plate_view.set_taught_corner(
                (cx / 1000.0, cy / 1000.0), self._corner_well)
            self._cal_plate_view._rebuild()
        logger.info(f"Taught corner XYZ: {self._taught_corner}, Z={self._taught_corner_z}")
        # Auto-try Z plane fit
        self._try_fit_z_plane()

    def _goto_third_auto(self):
        """v7.2.7: Auto-navigate to estimated third teach point."""
        if self._taught_a1 is None or self._plate is None:
            QMessageBox.warning(self, "Cannot Navigate",
                                "Record A1 and select plate format first.")
            return
        self._third_well = self._pick_third_well()
        if self._third_well is None:
            return
        est = self._estimate_well_position_um(self._third_well)
        if est is None:
            return
        self._safe_navigate_to(est[0], est[1])
        if hasattr(self, 'lbl_third'):
            self.lbl_third.setText(f"At {self._third_well} estimate — fine-tune & record")
            self.lbl_third.setStyleSheet(f"color: {COLORS['yellow']};")

    def _record_third_xyz(self):
        """v7.2.7: Record third teach point with XYZ."""
        xy = self.controller.get_xy_position(cached=False)
        zp = self.controller.get_zp_position(cached=False)
        if xy[0] is None:
            return
        self._taught_third = (xy[0], xy[1])
        tx = xy[0] - self.controller.zero_position["x"]
        ty = xy[1] - self.controller.zero_position["y"]
        if zp is not None and zp[0] is not None:
            self._taught_third_z = zp[0] - self.controller.zero_position.get("Z", 0)
        well_name = self._third_well or "3rd"
        z_str = f"  Z: {self._taught_third_z:.2f} mm" if self._taught_third_z is not None else ""
        if hasattr(self, 'lbl_third'):
            self.lbl_third.setText(f"{well_name}: ({tx:,.1f}, {ty:,.1f}) µm{z_str}")
            self.lbl_third.setStyleSheet(f"color: {COLORS['green']};")
        logger.info(f"Taught third XYZ: {self._taught_third}, Z={self._taught_third_z}")
        self._try_fit_z_plane()

    def _try_fit_z_plane(self):
        """v7.2.7: Attempt Z-plane fit if 3 points available."""
        points = []
        if self._taught_a1 is not None and self._taught_a1_z is not None:
            points.append(("A1", self._taught_a1_z))
        if self._taught_corner is not None and self._taught_corner_z is not None:
            points.append((self._corner_well, self._taught_corner_z))
        if self._taught_third is not None and self._taught_third_z is not None:
            points.append((self._third_well or "3rd", self._taught_third_z))

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
                    self.lbl_zplane.setText(
                        f"Z plane: {result.describe()}")
                    self.lbl_zplane.setStyleSheet(f"color: {COLORS['green']};")
                logger.info(f"Z plane fitted: {result.describe()}")
        except Exception as e:
            logger.error(f"Z plane fit failed: {e}")
            if hasattr(self, 'lbl_zplane'):
                self.lbl_zplane.setText(f"Fit failed: {e}")
                self.lbl_zplane.setStyleSheet(f"color: {COLORS['red']};")

    def _navigate_to_well(self, well_name):
        """v7.2.7: Navigate to any well via plate view click."""
        if self._safe_z is None:
            QMessageBox.warning(self, "No Safe Z",
                                "Set Safe Z height before navigating.")
            return
        if self._taught_a1 is None:
            QMessageBox.warning(self, "No A1",
                                "Teach A1 position before navigating to wells.")
            return
        est = self._estimate_well_position_um(well_name)
        if est is None:
            return
        # Estimate Z from plane if available
        target_z = None
        if self._z_plane_result:
            try:
                rel_x, rel_y = self._plate.get_well_position(well_name)
                target_z = self._z_plane_result.z_at(rel_x, rel_y) + self._z_buffer_mm
            except Exception:
                pass
        self._safe_navigate_to(est[0], est[1], target_z)
        logger.info(f"Navigated to well {well_name}")

'''
    content = content[:m.start()] + methods + content[m.start():]
    report("OK", "Added Z teaching + auto-navigate methods")
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH 4: Make _CalibrationPlateView clickable
# ═══════════════════════════════════════════════════════════════════

def patch_4_clickable_plate_view(content):
    """Add click signal and taught-point markers to _CalibrationPlateView."""
    print(f"\n{CYAN}[4] Make plate view clickable{RESET}")
    marker = "well_clicked = Signal"
    if marker in content:
        report("SKIP", "Already has click signal")
        return content

    # Find the class definition
    class_pat = re.compile(
        r'^(class _CalibrationPlateView\(QGraphicsView\):)\s*\n'
        r'(\s+""".*?""")',
        re.DOTALL | re.MULTILINE
    )
    m = class_pat.search(content)
    if not m:
        report("MISS", "Could not find _CalibrationPlateView class")
        return content

    # Add Signal import check and well_clicked signal after the docstring
    insert_pos = m.end()
    inject = (
        "\n\n"
        "    # v7.2.7: Click signal for well navigation\n"
        "    try:\n"
        "        from PySide6.QtCore import Signal as _Sig\n"
        "        well_clicked = _Sig(str)\n"
        "    except Exception:\n"
        "        pass\n"
    )
    content = content[:insert_pos] + inject + content[insert_pos:]
    report("OK", "Added well_clicked signal")

    # Add mousePressEvent to _CalibrationPlateView
    # Find the resizeEvent method of _CalibrationPlateView to insert before it
    resize_pat = re.compile(
        r'^(    def resizeEvent\(self, event\):\s*\n'
        r'\s+super\(\)\.resizeEvent\(event\)\s*\n'
        r'\s+if self\._plate:)',
        re.MULTILINE
    )
    m_resize = resize_pat.search(content)
    if m_resize:
        click_method = '''    def mousePressEvent(self, event):
        """v7.2.7: Click a well to navigate to it."""
        if self._plate is None:
            super().mousePressEvent(event)
            return
        scene_pos = self.mapToScene(event.pos())
        S = self.SCALE
        # Hit-test wells
        best_well = None
        best_dist = float('inf')
        for well in self._plate.get_all_wells():
            cx, cy = well.x * S, well.y * S
            dx = scene_pos.x() - cx
            dy = scene_pos.y() - cy
            dist = (dx*dx + dy*dy) ** 0.5
            if dist < self.WELL_R + 2 and dist < best_dist:
                best_dist = dist
                best_well = well.name
        if best_well:
            try:
                self.well_clicked.emit(best_well)
            except AttributeError:
                pass
            logger.debug(f"Plate view clicked: {best_well}")
        else:
            super().mousePressEvent(event)

'''
        content = content[:m_resize.start()] + click_method + content[m_resize.start():]
        report("OK", "Added mousePressEvent to plate view")
    else:
        report("MISS", "Could not find resizeEvent for plate view click insertion")

    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH 5: Connect plate view click and update on_status_update
# ═══════════════════════════════════════════════════════════════════

def patch_5_wire_plate_click(content):
    """Connect plate view well_clicked to _navigate_to_well."""
    print(f"\n{CYAN}[5] Wire plate view click signal{RESET}")
    marker = "well_clicked.connect"
    if marker in content:
        report("SKIP", "Already wired")
        return content

    # Find where the plate view is created: self._cal_plate_view or plate_view
    # Look for set_plate call on the calibration plate view
    view_pat = re.compile(r'(self\._cal_plate_view\s*=\s*_CalibrationPlateView\(.*?\))')
    m = view_pat.search(content)
    if m:
        insert_pos = m.end()
        inject = (
            "\n        # v7.2.7: Connect click-to-navigate\n"
            "        if hasattr(self._cal_plate_view, 'well_clicked'):\n"
            "            self._cal_plate_view.well_clicked.connect(self._navigate_to_well)\n"
        )
        content = content[:insert_pos] + inject + content[insert_pos:]
        report("OK", "Connected plate view click signal")
    else:
        # Try alternate name: self.plate_view
        view_pat2 = re.compile(r'(self\.plate_view\s*=\s*_CalibrationPlateView\(.*?\))')
        m2 = view_pat2.search(content)
        if m2:
            insert_pos = m2.end()
            inject = (
                "\n        if hasattr(self.plate_view, 'well_clicked'):\n"
                "            self.plate_view.well_clicked.connect(self._navigate_to_well)\n"
            )
            content = content[:insert_pos] + inject + content[insert_pos:]
            report("OK", "Connected plate view click (alt name)")
        else:
            report("MISS", "Could not find plate view instantiation")

    # Update on_status_update to feed needle position to plate view
    needle_marker = "v7.2.7: feed needle to plate view"
    if needle_marker not in content:
        # Find the end of on_status_update's position display block
        # Look for lbl_z.setText in on_status_update
        z_lbl_pat = re.compile(
            r'(self\.lbl_z\.setText\(f"\{zp\[0\].*?\))\s*\n'
        )
        m_z = z_lbl_pat.search(content)
        if m_z:
            inject_needle = (
                "\n"
                "            # v7.2.7: feed needle to plate view\n"
                "            if hasattr(self, '_cal_plate_view') and self._taught_a1 is not None:\n"
                "                zero_x = ctrl.zero_position.get('x', 0)\n"
                "                zero_y = ctrl.zero_position.get('y', 0)\n"
                "                nx_mm = (xy[0] - self._taught_a1[0]) / 1000.0\n"
                "                ny_mm = (xy[1] - self._taught_a1[1]) / 1000.0\n"
                "                self._cal_plate_view.set_needle_xy(nx_mm, ny_mm)\n"
            )
            content = content[:m_z.end()] + inject_needle + content[m_z.end():]
            report("OK", "Added needle position feed to plate view")
        else:
            report("MISS", "Could not find Z label update for needle feed")
    else:
        report("SKIP", "Needle feed already present")

    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH 6: New Step 2 UI in context panel
# ═══════════════════════════════════════════════════════════════════

def patch_6_step2_ui(content):
    """Replace Step 2 context panel with new guided workflow UI."""
    print(f"\n{CYAN}[6] Step 2 context panel UI{RESET}")
    marker = "Step 2A — Safe Z"
    if marker in content:
        report("SKIP", "New Step 2 UI already present")
        return content

    # Find the old Step 2 UI block. Look for the section label.
    # Pattern: "Step 2 — Teach Plate"
    old_step2_start = re.compile(
        r'^(\s+)s2_label = QLabel\("Step 2.*?Teach Plate"\)',
        re.MULTILINE
    )
    m_start = old_step2_start.search(content)

    # Find the end: Step 3 label or validate section
    old_step3_start = re.compile(
        r'^(\s+)s3_label = QLabel\("Step 3',
        re.MULTILINE
    )
    m_end = old_step3_start.search(content)

    if not m_start or not m_end:
        report("MISS", f"Could not find Step 2 boundaries "
               f"(start={'Y' if m_start else 'N'}, end={'Y' if m_end else 'N'})")
        return content

    ind = m_start.group(1)
    new_step2_ui = f'''{ind}# ── Step 2A — Safe Z ─────────────────────────────
{ind}s2a_label = QLabel("Step 2A — Safe Z")
{ind}s2a_label.setObjectName("contextSectionLabel")
{ind}layout.addWidget(s2a_label)
{ind}layout.addWidget(QLabel("Raise needle to safe travel height, then set."))
{ind}s2a_row = QHBoxLayout()
{ind}btn_safe_z = QPushButton("Set Safe Z")
{ind}btn_safe_z.setObjectName("successBtn")
{ind}btn_safe_z.setMaximumHeight(26)
{ind}btn_safe_z.clicked.connect(self._set_safe_z)
{ind}s2a_row.addWidget(btn_safe_z)
{ind}self.lbl_safe_z = QLabel("Not set")
{ind}self.lbl_safe_z.setStyleSheet(f"color: {{COLORS['yellow']}};")
{ind}s2a_row.addWidget(self.lbl_safe_z, stretch=1)
{ind}layout.addLayout(s2a_row)

{ind}# ── Step 2B — Top Z ──────────────────────────────
{ind}s2b_label = QLabel("Step 2B — Top Z (Plate Surface)")
{ind}s2b_label.setObjectName("contextSectionLabel")
{ind}layout.addWidget(s2b_label)
{ind}layout.addWidget(QLabel("Lower needle to plate top surface, then set."))
{ind}s2b_row = QHBoxLayout()
{ind}btn_top_z = QPushButton("Set Top Z")
{ind}btn_top_z.setMaximumHeight(26)
{ind}btn_top_z.clicked.connect(self._set_top_z)
{ind}s2b_row.addWidget(btn_top_z)
{ind}self.lbl_top_z = QLabel("Not set")
{ind}self.lbl_top_z.setStyleSheet(f"color: {{COLORS['yellow']}};")
{ind}s2b_row.addWidget(self.lbl_top_z, stretch=1)
{ind}layout.addLayout(s2b_row)

{ind}# ── Step 2C — Teach A1 XYZ ──────────────────────
{ind}s2c_label = QLabel("Step 2C — Teach A1 (Center Bottom)")
{ind}s2c_label.setObjectName("contextSectionLabel")
{ind}layout.addWidget(s2c_label)
{ind}layout.addWidget(QLabel("Jog to center bottom of well A1."))
{ind}a1_row = QHBoxLayout()
{ind}a1_row.addWidget(QLabel("A1:"))
{ind}self.lbl_a1 = QLabel("—")
{ind}self.lbl_a1.setStyleSheet(f"color: {{COLORS['overlay0']}};")
{ind}a1_row.addWidget(self.lbl_a1, stretch=1)
{ind}btn_rec_a1 = QPushButton("Rec XYZ")
{ind}btn_rec_a1.setMaximumHeight(24)
{ind}btn_rec_a1.setMaximumWidth(60)
{ind}btn_rec_a1.clicked.connect(self._record_a1_xyz)
{ind}a1_row.addWidget(btn_rec_a1)
{ind}btn_go_a1 = QPushButton("Go")
{ind}btn_go_a1.setMaximumHeight(24)
{ind}btn_go_a1.setMaximumWidth(30)
{ind}btn_go_a1.clicked.connect(self._goto_a1)
{ind}a1_row.addWidget(btn_go_a1)
{ind}layout.addLayout(a1_row)

{ind}# ── Step 2D — Teach Corner (auto-navigate) ──────
{ind}s2d_label = QLabel("Step 2D — Teach Corner (Auto-Navigate)")
{ind}s2d_label.setObjectName("contextSectionLabel")
{ind}layout.addWidget(s2d_label)
{ind}cr_row = QHBoxLayout()
{ind}cr_row.addWidget(QLabel("Corner:"))
{ind}self.lbl_corner = QLabel("—")
{ind}self.lbl_corner.setStyleSheet(f"color: {{COLORS['overlay0']}};")
{ind}cr_row.addWidget(self.lbl_corner, stretch=1)
{ind}btn_go_corner_auto = QPushButton("Go ▶")
{ind}btn_go_corner_auto.setMaximumHeight(24)
{ind}btn_go_corner_auto.setMaximumWidth(40)
{ind}btn_go_corner_auto.setToolTip("Safe-travel to estimated corner position")
{ind}btn_go_corner_auto.clicked.connect(self._goto_corner_auto)
{ind}cr_row.addWidget(btn_go_corner_auto)
{ind}btn_rec_corner = QPushButton("Rec XYZ")
{ind}btn_rec_corner.setMaximumHeight(24)
{ind}btn_rec_corner.setMaximumWidth(60)
{ind}btn_rec_corner.clicked.connect(self._record_corner_xyz)
{ind}cr_row.addWidget(btn_rec_corner)
{ind}layout.addLayout(cr_row)

{ind}# ── Step 2E — Teach Third Point ──────────────────
{ind}s2e_label = QLabel("Step 2E — Third Point (Z Plane)")
{ind}s2e_label.setObjectName("contextSectionLabel")
{ind}layout.addWidget(s2e_label)
{ind}th_row = QHBoxLayout()
{ind}self.lbl_third = QLabel("—")
{ind}self.lbl_third.setStyleSheet(f"color: {{COLORS['overlay0']}};")
{ind}th_row.addWidget(self.lbl_third, stretch=1)
{ind}btn_go_third = QPushButton("Go ▶")
{ind}btn_go_third.setMaximumHeight(24)
{ind}btn_go_third.setMaximumWidth(40)
{ind}btn_go_third.setToolTip("Safe-travel to estimated third point")
{ind}btn_go_third.clicked.connect(self._goto_third_auto)
{ind}th_row.addWidget(btn_go_third)
{ind}btn_rec_third = QPushButton("Rec XYZ")
{ind}btn_rec_third.setMaximumHeight(24)
{ind}btn_rec_third.setMaximumWidth(60)
{ind}btn_rec_third.clicked.connect(self._record_third_xyz)
{ind}th_row.addWidget(btn_rec_third)
{ind}layout.addLayout(th_row)

{ind}# ── Z Plane Status ────────────────────────────────
{ind}self.lbl_zplane = QLabel("0/3 teach points")
{ind}self.lbl_zplane.setWordWrap(True)
{ind}self.lbl_zplane.setStyleSheet(f"color: {{COLORS['overlay0']}}; font-size: 9pt;")
{ind}layout.addWidget(self.lbl_zplane)

{ind}self._gen_status = QLabel("")
{ind}self._gen_status.setStyleSheet(f"color: {{COLORS['subtext0']}}; font-size: 9pt;")
{ind}layout.addWidget(self._gen_status)

'''
    content = content[:m_start.start()] + new_step2_ui + content[m_end.start():]
    report("OK", "Replaced Step 2 UI with new guided workflow")
    return content


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Calibration Workflow: Safe Z + 3-Point + Click Navigate")
    print(f"{'='*60}{RESET}")

    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    # Check prereq
    if "v7.2.7: positions are µm" not in content and \
       "v7.2.7: controller reports µm" not in content:
        print(f"\n{RED}WARNING: Run patch_v727_calibration_all_fixes.py first!{RESET}")
        resp = input("Continue anyway? [y/N]: ").strip().lower()
        if resp != 'y':
            return 1

    # Apply patches in order
    content = patch_1_init_attrs(content)
    content = patch_2_safe_navigate(content)
    content = patch_3_z_teaching(content)
    content = patch_4_clickable_plate_view(content)
    content = patch_5_wire_plate_click(content)
    content = patch_6_step2_ui(content)

    # AST verify
    print(f"\n{CYAN}AST verification...{RESET}")
    try:
        ast.parse(content)
        print(f"  {GREEN}✓ AST OK{RESET}")
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0, e.lineno-5), min(len(lines), e.lineno+3)):
            m = ">>>" if i == e.lineno-1 else "   "
            print(f"    {m} {i+1:4d} | {lines[i]}")
        return 1

    # Write
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727wf_{ts}"))
    path.write_text(content, encoding="utf-8")

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention{RESET}")
    else:
        print(f"\n{GREEN}✓ All workflow changes applied!{RESET}")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
