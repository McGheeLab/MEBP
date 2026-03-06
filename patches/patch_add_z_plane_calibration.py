#!/usr/bin/env python3
"""
patch_add_z_plane_calibration.py
Bug 2: Add Z-plane calibration (well bottom teach + plane fit) to
CalibrationPage context panel as "Step 4 — Z Plane".

Adds to calibration.py:
  State:   _z_detector (WellBottomDetector | None)
  UI:      "Step 4 — Z Plane Calibration" section in context panel
  Methods: _init_z_detector(), _teach_z_point(), _remove_z_point(),
           _fit_z_plane(), _clear_z_points(), _refresh_z_table(),
           _save_z_plane(), _load_z_plane(),
           _propagate_z_plane_to_well_setup()
  Hook:    set_hardware_config → re-init detector on plate change
           _on_plate_format_changed → re-init detector
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN  = "\033[96m"; RESET = "\033[0m"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def find_method(content: str, name: str):
    pat = re.compile(
        r'^(    def ' + re.escape(name) + r'\(self.*?\n)'
        r'(.*?)'
        r'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

# ── New methods block ─────────────────────────────────────────────
NEW_Z_METHODS = '''
    # ════════════════════════════════════════════════════════════════
    #  Z PLANE CALIBRATION  (v7.3.1)
    # ════════════════════════════════════════════════════════════════

    def _init_z_detector(self) -> None:
        """Create / recreate WellBottomDetector for current plate."""
        if self._plate is None:
            self._z_detector = None
            return
        try:
            from SupportClasses.WellSetup import WellBottomDetector
            self._z_detector = WellBottomDetector(self._plate)
        except ImportError:
            self._z_detector = None
            logger.warning("WellBottomDetector unavailable — Z plane disabled")

    def _teach_z_point(self) -> None:
        """Record current Z position as a teach point for selected well."""
        if self._z_detector is None:
            return
        well = getattr(self, "_z_well_combo", None)
        if well is None:
            return
        well_name = well.currentText()
        if not well_name:
            return

        try:
            pos = self.controller.get_zp_position()
            z_raw = pos.get("Z", None) if isinstance(pos, dict) else (
                pos[0] if pos and len(pos) > 0 else None)
        except Exception:
            z_raw = None

        if z_raw is None:
            if hasattr(self, "_z_status_lbl"):
                self._z_status_lbl.setText("⚠ Cannot read Z position")
                self._z_status_lbl.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')};")
            return

        # Convert steps → mm using microsteps factor if needed
        # Z controller returns mm directly in Marlin mode
        z_mm = float(z_raw)

        self._z_detector.add_point(well_name, z_mm)
        logger.info(f"Z teach: {well_name} = {z_mm:.4f} mm")
        self._refresh_z_table()

        n = self._z_detector.num_points
        can_fit = self._z_detector.can_fit
        if hasattr(self, "_z_fit_btn"):
            self._z_fit_btn.setEnabled(can_fit)
        if hasattr(self, "_z_status_lbl"):
            self._z_status_lbl.setText(
                f"{n} point(s) taught" +
                (" — ready to fit" if can_fit else " — need ≥3 to fit"))
            self._z_status_lbl.setStyleSheet(
                f"color: {COLORS.get('green' if can_fit else 'subtext0', '#a6e3a1')};")

    def _remove_z_point(self) -> None:
        """Remove the selected teach point from the table."""
        if self._z_detector is None:
            return
        tbl = getattr(self, "_z_points_table", None)
        if tbl is None:
            return
        row = tbl.currentRow()
        if row < 0:
            return
        well_item = tbl.item(row, 0)
        if well_item:
            self._z_detector.remove_point(well_item.text())
        self._refresh_z_table()
        if hasattr(self, "_z_fit_btn"):
            self._z_fit_btn.setEnabled(
                self._z_detector.can_fit if self._z_detector else False)

    def _fit_z_plane(self) -> None:
        """Fit least-squares plane to teach points and display result."""
        if self._z_detector is None or not self._z_detector.can_fit:
            return
        result = self._z_detector.fit_plane()
        if result is None:
            if hasattr(self, "_z_status_lbl"):
                self._z_status_lbl.setText("⚠ Plane fit failed")
            return
        desc = result.describe()
        if hasattr(self, "_z_status_lbl"):
            self._z_status_lbl.setText(f"✓ {desc}")
            self._z_status_lbl.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')};")
        self._save_z_plane()
        self._propagate_z_plane_to_well_setup()
        logger.info(f"Z plane fit: {desc}")

    def _clear_z_points(self) -> None:
        """Remove all teach points."""
        if self._z_detector:
            self._z_detector.clear()
        self._refresh_z_table()
        if hasattr(self, "_z_fit_btn"):
            self._z_fit_btn.setEnabled(False)
        if hasattr(self, "_z_status_lbl"):
            self._z_status_lbl.setText("Points cleared")
            self._z_status_lbl.setStyleSheet(
                f"color: {COLORS.get('subtext0', '#a6adc8')};")

    def _refresh_z_table(self) -> None:
        """Rebuild the teach-points table from detector state."""
        tbl = getattr(self, "_z_points_table", None)
        if tbl is None or self._z_detector is None:
            return
        from PySide6.QtWidgets import QTableWidgetItem
        points = self._z_detector.points
        tbl.setRowCount(len(points))
        for r, pt in enumerate(points):
            tbl.setItem(r, 0, QTableWidgetItem(pt.well_name))
            tbl.setItem(r, 1, QTableWidgetItem(f"{pt.z_mm:.4f}"))

    def _save_z_plane(self) -> None:
        """Persist teach points and plane result to settings."""
        if self.settings is None or self._z_detector is None:
            return
        try:
            data = self._z_detector.to_dict()
            self.settings.set("calibration", "z_plane", data)
            self.settings.save()
            logger.info("Z plane saved to settings")
        except Exception as exc:
            logger.error(f"Z plane save failed: {exc}")

    def _load_z_plane(self) -> None:
        """Restore teach points and plane result from settings."""
        if self.settings is None or self._z_detector is None:
            return
        try:
            cal = self.settings.get_section("calibration") or {}
            data = cal.get("z_plane")
            if not data:
                return
            self._z_detector.load_dict(data)
            self._refresh_z_table()
            result = self._z_detector.result
            if hasattr(self, "_z_fit_btn"):
                self._z_fit_btn.setEnabled(self._z_detector.can_fit)
            if hasattr(self, "_z_status_lbl") and result:
                self._z_status_lbl.setText(f"✓ {result.describe()}")
                self._z_status_lbl.setStyleSheet(
                    f"color: {COLORS.get('green', '#a6e3a1')};")
            logger.info("Z plane loaded from settings")
        except Exception as exc:
            logger.error(f"Z plane load failed: {exc}")

    def _propagate_z_plane_to_well_setup(self) -> None:
        """Push fitted plane offsets to WellSetupTab if accessible."""
        if self._z_detector is None or self._z_detector.result is None:
            return
        try:
            # Walk up to main window and find print_setup page → tab_wells
            mw = self.window()
            setup_page = None
            if hasattr(mw, "_page_widgets"):
                for page in mw._page_widgets:
                    if hasattr(page, "tab_wells"):
                        setup_page = page
                        break
            if setup_page is None:
                return
            well_tab = setup_page.tab_wells
            model = getattr(well_tab, "_model", None)
            if model is None:
                return
            offsets = self._z_detector.get_all_offsets()
            for name, z in offsets.items():
                if name in model.assignments:
                    model.assignments[name].z_offset = z
            logger.info(f"Z offsets propagated to {len(offsets)} wells")
        except Exception as exc:
            logger.debug(f"Z plane propagation skipped: {exc}")

'''

# ── Z plane context UI section (inserted into get_context_widget) ─
Z_UI_SECTION = '''
        # ── Step 4: Z Plane Calibration (v7.3.1) ────────────────
        s4_label = QLabel("Step 4 — Z Plane Calibration")
        s4_label.setObjectName("contextSectionLabel")
        layout.addWidget(s4_label)

        z_info = QLabel(
            "Jog needle to well glass surface, then click Teach Z.\n"
            "Repeat for ≥3 wells, then Fit Plane.")
        z_info.setWordWrap(True)
        z_info.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        layout.addWidget(z_info)

        # Well selector + Teach button
        z_well_row = QHBoxLayout()
        z_well_row.addWidget(QLabel("Well:"))
        self._z_well_combo = QComboBox()
        if self._plate:
            wells = [w.name for w in self._plate.get_all_wells()]
            self._z_well_combo.addItems(wells)
        z_well_row.addWidget(self._z_well_combo, stretch=1)
        btn_teach_z = QPushButton("Teach Z")
        btn_teach_z.setMaximumHeight(24)
        btn_teach_z.setStyleSheet(
            f"background-color: {COLORS.get('blue', '#89b4fa')}; "
            f"color: #1e1e2e; font-weight: bold;")
        btn_teach_z.clicked.connect(self._teach_z_point)
        z_well_row.addWidget(btn_teach_z)
        layout.addLayout(z_well_row)

        # Teach points table
        from PySide6.QtWidgets import QTableWidget, QHeaderView, QAbstractItemView
        self._z_points_table = QTableWidget(0, 2)
        self._z_points_table.setHorizontalHeaderLabels(["Well", "Z (mm)"])
        self._z_points_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self._z_points_table.setMaximumHeight(100)
        self._z_points_table.setSelectionBehavior(
            QAbstractItemView.SelectRows)
        self._z_points_table.setEditTriggers(
            QAbstractItemView.NoEditTriggers)
        layout.addWidget(self._z_points_table)

        # Action buttons row
        z_btn_row = QHBoxLayout()
        btn_remove_z = QPushButton("Remove")
        btn_remove_z.setMaximumHeight(24)
        btn_remove_z.clicked.connect(self._remove_z_point)
        z_btn_row.addWidget(btn_remove_z)

        self._z_fit_btn = QPushButton("Fit Plane")
        self._z_fit_btn.setMaximumHeight(24)
        self._z_fit_btn.setEnabled(False)
        self._z_fit_btn.setStyleSheet(
            f"QPushButton:enabled {{ background-color: "
            f"{COLORS.get('green', '#a6e3a1')}; color: #1e1e2e; "
            f"font-weight: bold; }}")
        self._z_fit_btn.clicked.connect(self._fit_z_plane)
        z_btn_row.addWidget(self._z_fit_btn)

        btn_clear_z = QPushButton("Clear")
        btn_clear_z.setMaximumHeight(24)
        btn_clear_z.clicked.connect(self._clear_z_points)
        z_btn_row.addWidget(btn_clear_z)
        layout.addLayout(z_btn_row)

        # Status / result label
        self._z_status_lbl = QLabel("No Z plane calibrated")
        self._z_status_lbl.setWordWrap(True)
        self._z_status_lbl.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        layout.addWidget(self._z_status_lbl)

        # Init detector and restore any saved plane
        self._init_z_detector()
        self._load_z_plane()

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "calibration.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")
    changed = False

    # ── 1. Add _z_detector to __init__ ───────────────────────────
    guard1 = "self._z_detector = None"
    if guard1 not in content:
        # Insert after _hardware_config = None
        old = "        self._hardware_config = None  # v7.2: HardwareConfig"
        new = (old + "\n"
               "        self._z_detector = None  # v7.3.1: WellBottomDetector")
        if old in content:
            content = content.replace(old, new, 1)
            print(f"  {GREEN}✓ Added _z_detector to __init__{RESET}")
            changed = True
        else:
            # Fallback: after _context_widget = None
            old2 = "        self._context_widget = None"
            new2 = (old2 + "\n"
                    "        self._z_detector = None  # v7.3.1: WellBottomDetector")
            if old2 in content:
                content = content.replace(old2, new2, 1)
                print(f"  {GREEN}✓ Added _z_detector to __init__ (fallback){RESET}")
                changed = True
            else:
                print(f"  {YELLOW}⚠ Could not add _z_detector to __init__ — continuing{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: _z_detector already in __init__{RESET}")

    # ── 2. Inject new Z methods before _save_calibration ─────────
    guard2 = "v7.3.1: WellBottomDetector for current plate"
    if guard2 not in content:
        m = find_method(content, "_save_calibration")
        if m:
            content = content[:m.start()] + NEW_Z_METHODS + content[m.start():]
            print(f"  {GREEN}✓ Z plane methods injected{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ MISS: _save_calibration anchor not found{RESET}")
            sys.exit(1)
    else:
        print(f"  {YELLOW}○ SKIP: Z methods already present{RESET}")

    # ── 3. Add Z UI section to get_context_widget ─────────────────
    guard3 = "Step 4 — Z Plane Calibration"
    if guard3 not in content:
        # Insert before the Calibration Persistence section
        anchor = "        # ── Calibration Persistence ──────────────────────────────"
        if anchor in content:
            content = content.replace(anchor, Z_UI_SECTION + "\n" + anchor, 1)
            print(f"  {GREEN}✓ Z plane UI section added to context panel{RESET}")
            changed = True
        else:
            # Try alternative anchor
            anchor2 = "        persist_label = QLabel(\"Calibration Data\")"
            if anchor2 in content:
                content = content.replace(anchor2,
                    Z_UI_SECTION + "\n" + anchor2, 1)
                print(f"  {GREEN}✓ Z plane UI section added (fallback anchor){RESET}")
                changed = True
            else:
                print(f"  {RED}✗ MISS: context panel anchor not found{RESET}")
                sys.exit(1)
    else:
        print(f"  {YELLOW}○ SKIP: Z plane UI already present{RESET}")

    # ── 4. Re-init detector when plate format changes ─────────────
    guard4 = "self._init_z_detector()  # v7.3.1"
    if guard4 not in content:
        # Hook into _on_plate_format_changed after _plate is set
        old = "        self._plate = WellPlate.from_format(fmt)"
        new = (old + "\n"
               "        self._init_z_detector()  # v7.3.1: rebuild detector for new plate")
        if old in content:
            content = content.replace(old, new, 1)
            print(f"  {GREEN}✓ Detector re-init hooked into plate format change{RESET}")
            changed = True
        else:
            print(f"  {YELLOW}⚠ Plate format change hook not found — skipping step 4{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: detector re-init already hooked{RESET}")

    if not changed:
        print(f"  {YELLOW}○ Nothing to do{RESET}")
        return

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731zp_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ calibration.py patched — Z plane calibration added{RESET}")
    print(f"\n  New workflow:")
    print(f"    1. On Calibration page → context panel → Step 4")
    print(f"    2. Select well, jog needle to glass surface, click Teach Z")
    print(f"    3. Repeat for ≥3 wells (corners + center recommended)")
    print(f"    4. Click Fit Plane — R² and equation shown, offsets saved")
    print(f"    5. Offsets auto-propagate to Well Setup tab")

if __name__ == "__main__":
    main()
