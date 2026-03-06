#!/usr/bin/env python3
"""
patch_fix_calibration_views_v2.py  (v7.3.1)
Adds CalibrationPlateView + YZ view to calibration.py.
v2: fixes indentation in plate-hook step.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; CYAN = "\033[96m"; RESET = "\033[0m"
GUARD = "v7.3.1-calviews"

def find_root():
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP root")

def ast_ok(text, tag):
    try:
        ast.parse(text); return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL after {tag}: {e}{RESET}")
        lines = text.splitlines()
        ln = e.lineno or 0
        for i in range(max(0, ln - 3), min(len(lines), ln + 3)):
            print(f"    {i+1:4d}: {repr(lines[i])}")
        return False

CAL_WIDGETS = '''\

# ═══════════════════════════════════════════════════════════════════
# Calibration helper widgets  (v7.3.1-calviews)
# ═══════════════════════════════════════════════════════════════════

class _CalibrationPlateView(QGraphicsView):
    """Top-down plate view for calibration page — shows needle XY position."""
    SCALE  = 4.0
    WELL_R = 3.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._plate        = None
        self._taught_a1    = None
        self._taught_corner = None
        self._corner_well  = "H12"
        self._needle_xy    = None
        self._needle_item  = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumHeight(130)

    def set_plate(self, plate):
        self._plate = plate
        self._rebuild()

    def set_taught_a1(self, pt):
        self._taught_a1 = pt

    def set_taught_corner(self, pt, corner_well):
        self._taught_corner = pt
        self._corner_well   = corner_well

    def set_needle_xy(self, x_mm, y_mm):
        self._needle_xy = (x_mm, y_mm)
        self._update_needle()

    def _rebuild(self):
        self._scene.clear()
        self._needle_item = None
        if self._plate is None:
            return
        S = self.SCALE
        wells = list(self._plate.get_all_wells())
        for well in wells:
            cx, cy, r = well.x * S, well.y * S, self.WELL_R
            self._scene.addEllipse(
                cx - r, cy - r, 2 * r, 2 * r,
                QPen(QColor("#585b70"), 0.5),
                QBrush(QColor("#313244")),
            ).setToolTip(well.name)

        # Highlight A1 and corner well
        wmap = {w.name: w for w in wells}
        for wname, color in [("A1", "#89b4fa"), (self._corner_well, "#74c7ec")]:
            w = wmap.get(wname)
            if w:
                cx, cy = w.x * S, w.y * S
                r2 = self.WELL_R + 1.5
                self._scene.addEllipse(
                    cx - r2, cy - r2, 2 * r2, 2 * r2,
                    QPen(QColor(color), 1.5),
                    QBrush(QColor("#00000000")),
                )

        # Needle placeholder
        self._needle_item = self._scene.addEllipse(
            -6, -6, 12, 12,
            QPen(QColor("#f38ba8"), 1.5),
            QBrush(QColor("#f38ba880")),
        )
        self._needle_item.setVisible(False)
        self._needle_item.setZValue(10)

        rect = self._scene.itemsBoundingRect().adjusted(-8, -8, 8, 8)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    def _update_needle(self):
        if self._needle_item is None:
            return
        if self._needle_xy is None:
            self._needle_item.setVisible(False)
            return
        x_mm, y_mm = self._needle_xy
        if self._taught_a1 is not None:
            x_mm = x_mm - self._taught_a1[0]
            y_mm = y_mm - self._taught_a1[1]
        S = self.SCALE
        self._needle_item.setRect(x_mm * S - 6, y_mm * S - 6, 12, 12)
        self._needle_item.setVisible(True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._plate:
            rect = self._scene.itemsBoundingRect().adjusted(-8, -8, 8, 8)
            if not rect.isEmpty():
                self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)


class _CalibrationYZView(QGraphicsView):
    """YZ side view for calibration — shows live needle Z and well-bottom offsets."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene    = QGraphicsScene(self)
        self.setScene(self._scene)
        self._z_min    = -2.0
        self._z_max    =  8.0
        self._needle_z = None
        self._offsets: dict = {}
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setFixedWidth(90)
        self.setMinimumHeight(130)

    def set_well_z_offsets(self, offsets: dict):
        self._offsets = offsets
        self._rebuild()

    def set_needle_z(self, z_mm):
        self._needle_z = z_mm
        self._rebuild()

    def _z_to_y(self, z, h=200):
        frac = (z - self._z_min) / max(self._z_max - self._z_min, 0.001)
        return h * (1.0 - frac)

    def _rebuild(self):
        self._scene.clear()
        h, w = 200, 70
        self._scene.addRect(0, 0, w, h,
                            QPen(QColor("#45475a"), 0),
                            QBrush(QColor("#11111b")))
        # Axis label
        from PySide6.QtWidgets import QGraphicsTextItem
        t = self._scene.addText("Z (mm)")
        t.setDefaultTextColor(QColor("#a6adc8"))
        t.setPos(2, 2)

        # Well-bottom tick lines
        for z in self._offsets.values():
            y = self._z_to_y(z, h)
            self._scene.addLine(10, y, w - 4, y, QPen(QColor("#585b70"), 0.8))

        # Z = 0 baseline
        y0 = self._z_to_y(0.0, h)
        self._scene.addLine(10, y0, w - 4, y0, QPen(QColor("#6c7086"), 1.0))

        # Live needle Z
        if self._needle_z is not None:
            yn = self._z_to_y(self._needle_z, h)
            self._scene.addLine(0, yn, w, yn, QPen(QColor("#f38ba8"), 2.0))
            zt = self._scene.addText(f"{self._needle_z:.2f}")
            zt.setDefaultTextColor(QColor("#f38ba8"))
            zt.setPos(2, max(0, yn - 14))

        self.setSceneRect(0, 0, w, h)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.IgnoreAspectRatio)

'''

CAL_PAGE_METHODS = '''\
    # ── Plate + YZ views (v7.3.1-calviews) ──────────────────────

    def _build_cal_views(self, parent_layout) -> None:
        """Add plate-view + YZ-view row below camera card."""
        from PySide6.QtWidgets import QGroupBox
        grp = QGroupBox("Plate Position View")
        grp.setStyleSheet(
            "QGroupBox { color: #cdd6f4; font-weight: bold; "
            "border: 1px solid #45475a; border-radius: 4px; "
            "margin-top: 6px; padding-top: 12px; }")
        grp_layout = QHBoxLayout(grp)
        grp_layout.setSpacing(4)
        grp_layout.setContentsMargins(4, 4, 4, 4)

        self._cal_plate_view = _CalibrationPlateView()
        grp_layout.addWidget(self._cal_plate_view, stretch=1)

        self._cal_yz_view = _CalibrationYZView()
        grp_layout.addWidget(self._cal_yz_view)

        parent_layout.addWidget(grp)

    def _start_cal_position_poll(self) -> None:
        """Start 250 ms timer to poll needle position."""
        if not hasattr(self, "_cal_pos_timer"):
            self._cal_pos_timer = QTimer(self)
            self._cal_pos_timer.setInterval(250)
            self._cal_pos_timer.timeout.connect(self._poll_cal_position)
        self._cal_pos_timer.start()

    def _poll_cal_position(self) -> None:
        """Poll XY+Z and push to plate + YZ views."""
        ctrl = self.controller
        if ctrl is None:
            return
        try:
            xy = ctrl.get_xy_position()
            zp = ctrl.get_zp_position()
        except Exception:
            return
        mpm = self._microsteps_per_micron or 1.0
        try:
            if isinstance(xy, dict):
                x_s, y_s = xy.get("x", 0) or 0, xy.get("y", 0) or 0
            elif xy and len(xy) >= 2:
                x_s, y_s = xy[0], xy[1]
            else:
                return
            x_mm = float(x_s) / (mpm * 1000.0)
            y_mm = float(y_s) / (mpm * 1000.0)
        except Exception:
            return
        try:
            z_mm = float(zp.get("Z", 0) or 0) if isinstance(zp, dict) \
                   else (float(zp[0]) if zp and len(zp) >= 1 else 0.0)
        except Exception:
            z_mm = 0.0

        if hasattr(self, "_cal_plate_view"):
            self._cal_plate_view.set_needle_xy(x_mm, y_mm)
        if hasattr(self, "_cal_yz_view"):
            self._cal_yz_view.set_needle_z(z_mm)

    def _update_cal_view_plate(self) -> None:
        """Push current plate model to cal views."""
        if not hasattr(self, "_cal_plate_view"):
            return
        if self._plate is not None:
            self._cal_plate_view.set_plate(self._plate)
            self._cal_plate_view.set_taught_a1(self._taught_a1)
            self._cal_plate_view.set_taught_corner(
                self._taught_corner, self._corner_well)
        if hasattr(self, "_cal_yz_view") and self._plate is not None:
            offsets = {}
            if hasattr(self._plate, "get_all_z_offsets"):
                offsets = self._plate.get_all_z_offsets()
            self._cal_yz_view.set_well_z_offsets(offsets)

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "calibration.py"
    print(f"{CYAN}Target: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); sys.exit(1)

    content = target.read_text(encoding="utf-8")
    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return

    # ── A. Imports ────────────────────────────────────────────────
    adds = []
    if "QGraphicsView" not in content:
        adds.append("from PySide6.QtWidgets import QGraphicsView, QGraphicsScene")
    if "QPainter" not in content:
        adds.append("from PySide6.QtGui import QPainter, QPen, QBrush, QColor")
    if "QTimer" not in content:
        adds.append("from PySide6.QtCore import QTimer")

    if adds:
        lines = content.splitlines(keepends=True)
        last_ps = max(
            (i for i, l in enumerate(lines)
             if l.startswith("from PySide6") or l.startswith("import PySide6")),
            default=0)
        lines.insert(last_ps + 1, "".join(a + "\n" for a in adds))
        content = "".join(lines)
        if not ast_ok(content, "imports"): sys.exit(1)
        print(f"  {GREEN}Imports added{RESET}")

    # ── B. Helper classes before CalibrationPage ──────────────────
    anchor_cls = "\nclass CalibrationPage(QWidget):"
    if anchor_cls not in content:
        print(f"  {RED}MISS: CalibrationPage not found{RESET}"); sys.exit(1)
    idx = content.index(anchor_cls)
    content = content[:idx] + CAL_WIDGETS + content[idx:]
    if not ast_ok(content, "helper classes"): sys.exit(1)
    print(f"  {GREEN}Helper classes injected{RESET}")

    # ── C. New methods inside CalibrationPage ─────────────────────
    anchor_m = "\n    def on_status_update(self):"
    if anchor_m not in content:
        anchor_m = "\n    def get_context_widget(self):"
    if anchor_m in content:
        idx2 = content.index(anchor_m)
        content = content[:idx2] + "\n" + CAL_PAGE_METHODS + content[idx2:]
    else:
        content += "\n" + CAL_PAGE_METHODS
    if not ast_ok(content, "page methods"): sys.exit(1)
    print(f"  {GREEN}Page methods added{RESET}")

    # ── D. Call _build_cal_views from _setup_ui ───────────────────
    cam_anchor = "layout.addWidget(cam_card, stretch=1)"
    if cam_anchor in content:
        content = content.replace(
            cam_anchor,
            cam_anchor
            + "\n\n        # v7.3.1-calviews: plate + YZ views\n"
            + "        self._build_cal_views(layout)",
            1,
        )
        if not ast_ok(content, "_build_cal_views call"): sys.exit(1)
        print(f"  {GREEN}_build_cal_views wired into _setup_ui{RESET}")
    else:
        print(f"  {YELLOW}WARN: cam_card anchor missing — skipping layout wire{RESET}")

    # ── E. Start poll after _setup_ui() in __init__ ───────────────
    setup_call = "        self._setup_ui()\n"
    if "_start_cal_position_poll" not in content and setup_call in content:
        content = content.replace(
            setup_call,
            setup_call + "        self._start_cal_position_poll()  # " + GUARD + "\n",
            1,
        )
        if not ast_ok(content, "poll start"): sys.exit(1)
        print(f"  {GREEN}Position poll started in __init__{RESET}")

    # ── F. Hook _update_cal_view_plate when plate is assigned ─────
    # Find: `        self._plate = WellPlate.from_format(` on its own line
    # and append the call AT THE SAME indentation level (8 spaces)
    old_plate_line = "        self._plate = WellPlate.from_format("
    # Find first occurrence
    idx_p = content.find(old_plate_line)
    if idx_p != -1 and "_update_cal_view_plate" not in content:
        # Find end of this statement (the closing paren + \n)
        end_of_line = content.index("\n", idx_p) + 1
        insert = "        self._update_cal_view_plate()  # " + GUARD + "\n"
        content = content[:end_of_line] + insert + content[end_of_line:]
        if not ast_ok(content, "plate hook"): sys.exit(1)
        print(f"  {GREEN}_update_cal_view_plate hooked{RESET}")
    elif "_update_cal_view_plate" in content:
        print(f"  {YELLOW}SKIP: plate hook already present{RESET}")
    else:
        print(f"  {YELLOW}WARN: plate assignment line not found — skipping hook{RESET}")

    # ── Write ─────────────────────────────────────────────────────
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}calibration.py written successfully{RESET}")

if __name__ == "__main__":
    main()
