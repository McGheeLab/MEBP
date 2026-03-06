#!/usr/bin/env python3
"""
patch_fix_calibration_views.py  (v7.3.1)

Adds to gui/pages/calibration.py:
  - Top-down WellPlate view (XY) below camera feeds, shows needle position
  - YZ side-view canvas to the right, shows Z position
  - 250ms QTimer polls controller position and updates both views
  - Views update when plate is taught (A1 / corner wells)
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
        for i in range(max(0, ln-3), min(len(lines), ln+2)):
            print(f"    {i+1:4d}: {repr(lines[i])}")
        return False

# ── New imports to add ────────────────────────────────────────────
NEW_IMPORTS = """\
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QSplitter
from PySide6.QtGui import QPainter, QPen, QBrush, QColor
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from PySide6.QtCore import Qt as Qt_
"""

# ── New class: CalibrationPlateView ──────────────────────────────
CAL_WIDGETS = '''\

# ═══════════════════════════════════════════════════════════════════
# Calibration helper widgets  (v7.3.1-calviews)
# ═══════════════════════════════════════════════════════════════════

class _CalibrationPlateView(QGraphicsView):
    """
    Compact top-down well plate view for the calibration page.
    Shows:
      - All wells as circles (gray fill)
      - Taught A1 well (blue)
      - Taught corner well (blue)
      - Live needle XY position (red crosshair)
    """
    SCALE = 4.0          # pixels per mm
    WELL_R = 3.5         # radius of well circles in scene units (px)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._plate = None
        self._taught_a1 = None        # (x_mm, y_mm) in plate coords
        self._taught_corner = None
        self._corner_well = "H12"
        self._needle_xy = None        # (x_mm, y_mm) in plate coords (may be None)
        self._needle_item = None

        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt_.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt_.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumHeight(130)

    def set_plate(self, plate):
        self._plate = plate
        self._rebuild()

    def set_taught_a1(self, pt):
        self._taught_a1 = pt
        self._update_needle()

    def set_taught_corner(self, pt, corner_well):
        self._taught_corner = pt
        self._corner_well = corner_well
        self._update_needle()

    def set_needle_xy(self, x_mm, y_mm):
        self._needle_xy = (x_mm, y_mm)
        self._update_needle()

    def _rebuild(self):
        self._scene.clear()
        self._needle_item = None
        if self._plate is None:
            return
        S = self.SCALE
        for well in self._plate.get_all_wells():
            cx = well.x * S
            cy = well.y * S
            r  = self.WELL_R
            ellipse = self._scene.addEllipse(
                cx - r, cy - r, 2 * r, 2 * r,
                QPen(QColor("#585b70"), 0.5),
                QBrush(QColor("#313244")),
            )
            ellipse.setToolTip(well.name)

        # Taught wells
        for well_name, color in [
            (getattr(self._plate, "first_well", "A1") if self._plate else "A1", "#89b4fa"),
            (self._corner_well, "#74c7ec"),
        ]:
            wells = {w.name: w for w in self._plate.get_all_wells()}
            w = wells.get(well_name)
            if w:
                cx, cy, r = w.x * S, w.y * S, self.WELL_R + 1.5
                self._scene.addEllipse(
                    cx - r, cy - r, 2 * r, 2 * r,
                    QPen(QColor(color), 1.5),
                    QBrush(QColor("#00000000")),  # transparent fill
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
            self.fitInView(rect, Qt_.AspectRatioMode.KeepAspectRatio)

    def _update_needle(self):
        if self._needle_item is None:
            return
        if self._needle_xy is None:
            self._needle_item.setVisible(False)
            return
        x_mm, y_mm = self._needle_xy
        # Offset by A1 position if known
        if self._taught_a1 is not None:
            # plate coords: A1 is origin
            # actual stage coords relative to A1
            pass
        S = self.SCALE
        self._needle_item.setRect(
            x_mm * S - 6, y_mm * S - 6, 12, 12)
        self._needle_item.setVisible(True)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._plate:
            rect = self._scene.itemsBoundingRect().adjusted(-8, -8, 8, 8)
            if not rect.isEmpty():
                self.fitInView(rect, Qt_.AspectRatioMode.KeepAspectRatio)


class _CalibrationYZView(QGraphicsView):
    """
    YZ side view for calibration page.
    Shows:
      - Horizontal lines at well-bottom Z offsets
      - Live needle Z position (red line + label)
      - Travel Z height (dashed line)
    """
    WELL_ROWS = 8    # max rows (A–H for 96-well)
    WIDTH_PX  = 80
    HEIGHT_PX = 200

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._z_min   = -2.0
        self._z_max   =  8.0
        self._needle_z = None
        self._well_z_offsets: dict = {}  # well_name → z_mm

        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet("background: #11111b; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt_.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt_.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setFixedWidth(90)
        self.setMinimumHeight(130)

    def set_z_range(self, z_min, z_max):
        self._z_min = z_min
        self._z_max = z_max
        self._rebuild()

    def set_well_z_offsets(self, offsets: dict):
        self._well_z_offsets = offsets
        self._rebuild()

    def set_needle_z(self, z_mm):
        self._needle_z = z_mm
        self._rebuild()

    def _z_to_y(self, z, h=200):
        """Map Z (mm) to scene Y (px). Higher Z = lower on screen."""
        frac = (z - self._z_min) / max(self._z_max - self._z_min, 0.001)
        return h * (1.0 - frac)

    def _rebuild(self):
        self._scene.clear()
        h = 200
        w = 70
        # Background
        self._scene.addRect(0, 0, w, h,
                            QPen(QColor("#45475a"), 0),
                            QBrush(QColor("#11111b")))

        # Axis label
        from PySide6.QtWidgets import QGraphicsTextItem
        t = self._scene.addText("Z")
        t.setDefaultTextColor(QColor("#a6adc8"))
        t.setPos(2, 2)

        # Well bottom lines
        for name, z in self._well_z_offsets.items():
            y = self._z_to_y(z, h)
            self._scene.addLine(10, y, w - 4, y,
                                QPen(QColor("#585b70"), 0.8))

        # Z=0 line
        y0 = self._z_to_y(0.0, h)
        self._scene.addLine(10, y0, w - 4, y0,
                            QPen(QColor("#6c7086"), 1.0))

        # Needle Z
        if self._needle_z is not None:
            yn = self._z_to_y(self._needle_z, h)
            self._scene.addLine(0, yn, w, yn,
                                QPen(QColor("#f38ba8"), 2.0))
            # Label
            t2 = self._scene.addText(f"{self._needle_z:.2f}")
            t2.setDefaultTextColor(QColor("#f38ba8"))
            t2.setPos(2, max(0, yn - 14))

        self.setSceneRect(0, 0, w, h)
        self.fitInView(self.sceneRect(), Qt_.AspectRatioMode.IgnoreAspectRatio)

'''

# ── New methods to add inside CalibrationPage ─────────────────────
CAL_PAGE_METHODS = '''\
    # ── Plate + YZ views (v7.3.1-calviews) ──────────────────────

    def _build_cal_views(self, parent_layout) -> None:
        """Add plate-view + YZ-view row below camera card."""
        from PySide6.QtWidgets import QGroupBox, QSplitter
        grp = QGroupBox("Plate Position View")
        grp.setStyleSheet(
            f"QGroupBox {{ color: {COLORS.get('text','#cdd6f4')}; font-weight: bold; "
            f"border: 1px solid {COLORS.get('surface1','#45475a')}; "
            f"border-radius: 4px; margin-top: 6px; padding-top: 12px; }}")
        grp_layout = QHBoxLayout(grp)
        grp_layout.setSpacing(4)
        grp_layout.setContentsMargins(4, 4, 4, 4)

        self._cal_plate_view = _CalibrationPlateView()
        grp_layout.addWidget(self._cal_plate_view, stretch=1)

        self._cal_yz_view = _CalibrationYZView()
        grp_layout.addWidget(self._cal_yz_view)

        parent_layout.addWidget(grp)

    def _start_cal_position_poll(self) -> None:
        """Start 250ms timer to poll needle position."""
        if not hasattr(self, "_cal_pos_timer"):
            self._cal_pos_timer = QTimer(self)
            self._cal_pos_timer.setInterval(250)
            self._cal_pos_timer.timeout.connect(self._poll_cal_position)
        self._cal_pos_timer.start()

    def _poll_cal_position(self) -> None:
        """Poll XY+Z and update plate and YZ views."""
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
            if isinstance(zp, dict):
                z_mm = float(zp.get("Z", 0) or 0)
            elif zp and len(zp) >= 1:
                z_mm = float(zp[0])
            else:
                z_mm = 0.0
        except Exception:
            z_mm = 0.0

        # Convert stage coords → plate coords if calibrated
        px, py = x_mm, y_mm
        if self._taught_a1 is not None:
            px = x_mm - self._taught_a1[0]
            py = y_mm - self._taught_a1[1]

        if hasattr(self, "_cal_plate_view"):
            self._cal_plate_view.set_needle_xy(px, py)
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
        # Update YZ z-offsets
        if hasattr(self, "_cal_yz_view") and self._plate is not None:
            offsets = self._plate.get_all_z_offsets() if hasattr(self._plate, "get_all_z_offsets") else {}
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

    changed = False

    # ── A. Inject imports after existing imports ──────────────────
    # Add QTimer / QSplitter if not present
    imports_to_add = []
    if "from PySide6.QtCore import QTimer" not in content:
        imports_to_add.append("from PySide6.QtCore import QTimer")
    if "QGraphicsView" not in content:
        imports_to_add.append(
            "from PySide6.QtWidgets import QGraphicsView, QGraphicsScene, QSplitter")
    if "QPainter" not in content:
        imports_to_add.append(
            "from PySide6.QtGui import QPainter, QPen, QBrush, QColor")

    if imports_to_add:
        # Insert after last `from PySide6` line
        lines = content.splitlines(keepends=True)
        last_pyside = max(
            (i for i, l in enumerate(lines)
             if l.startswith("from PySide6") or l.startswith("import PySide6")),
            default=0)
        insert_text = "".join(l + "\n" for l in imports_to_add)
        lines.insert(last_pyside + 1, insert_text)
        content = "".join(lines)
        if not ast_ok(content, "imports"):
            sys.exit(1)
        print(f"  {GREEN}Imports added: {imports_to_add}{RESET}")
        changed = True

    # ── B. Inject helper classes before CalibrationPage class ─────
    class_anchor = "\nclass CalibrationPage(QWidget):"
    if class_anchor not in content:
        print(f"  {RED}MISS: CalibrationPage class not found{RESET}"); sys.exit(1)
    idx = content.index(class_anchor)
    content = content[:idx] + CAL_WIDGETS + content[idx:]
    if not ast_ok(content, "helper classes"):
        sys.exit(1)
    print(f"  {GREEN}Helper classes injected{RESET}")
    changed = True

    # ── C. Inject new methods into CalibrationPage ────────────────
    # Insert before on_status_update
    anchor_method = "\n    def on_status_update(self):"
    if anchor_method not in content:
        anchor_method = "\n    def get_context_widget(self):"
    if anchor_method not in content:
        # Append to end of class
        content += "\n" + CAL_PAGE_METHODS
    else:
        idx2 = content.index(anchor_method)
        content = content[:idx2] + "\n" + CAL_PAGE_METHODS + content[idx2:]

    if not ast_ok(content, "page methods"):
        sys.exit(1)
    print(f"  {GREEN}CalibrationPage methods added{RESET}")
    changed = True

    # ── D. Call _build_cal_views from _setup_ui ───────────────────
    # Insert after `layout.addWidget(cam_card, stretch=1)`
    cam_anchor = "layout.addWidget(cam_card, stretch=1)"
    if cam_anchor in content:
        content = content.replace(
            cam_anchor,
            cam_anchor + "\n\n        # Plate + YZ position views (v7.3.1-calviews)\n"
            "        self._build_cal_views(layout)",
            1,
        )
        if not ast_ok(content, "_build_cal_views call"):
            sys.exit(1)
        print(f"  {GREEN}_build_cal_views() wired into _setup_ui{RESET}")
        changed = True
    else:
        print(f"  {YELLOW}WARN: cam_card anchor not found — views not wired into layout{RESET}")

    # ── E. Start position poll in __init__ ────────────────────────
    setup_call = "        self._setup_ui()"
    if "_start_cal_position_poll" not in content and setup_call in content:
        content = content.replace(
            setup_call,
            setup_call + "\n        self._start_cal_position_poll()  # " + GUARD,
            1,
        )
        if not ast_ok(content, "poll start"):
            sys.exit(1)
        print(f"  {GREEN}Position poll started in __init__{RESET}")
        changed = True

    # ── F. Hook _update_cal_view_plate whenever plate is set ──────
    # Look for where self._plate = WellPlate.from_format(...)
    plate_set = re.compile(r'(self\._plate\s*=\s*WellPlate\.from_format\([^)]+\))')
    def _add_cal_update(m):
        return m.group(0) + "\n            self._update_cal_view_plate()  # " + GUARD
    new_content = plate_set.sub(_add_cal_update, content)
    if new_content != content:
        content = new_content
        if not ast_ok(content, "plate hook"):
            sys.exit(1)
        print(f"  {GREEN}_update_cal_view_plate hooked to plate set{RESET}")
        changed = True

    if not changed:
        print(f"  {YELLOW}Nothing changed{RESET}"); return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}calibration.py patched successfully{RESET}")

if __name__ == "__main__":
    main()
