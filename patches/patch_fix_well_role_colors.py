#!/usr/bin/env python3
"""
patch_fix_well_role_colors.py  (v7.3.1)

Two files patched:

A. gui/widgets/well_plate_view.py
   1. Add update_all_wells(appearances) — sets fill from ROLE_COLORS, label text
   2. Add set_well_rosette(name, subwells) — draws tiny circles for each subwell
   3. Add update_well_rosettes(rosette_map) — bulk rosette update

B. gui/pages/print_well_setup.py
   1. Replace _refresh_well_status_colors() to use role-fill + selection border
   2. Add _refresh_well_rosettes() called from _refresh_plate / _attach_rosette
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; CYAN = "\033[96m"; RESET = "\033[0m"
GUARD = "v7.3.1-rolecolors"

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
        print(f"  {RED}AST FAIL after {tag}: {e}{RESET}"); return False

# ── WellPlateView additions ───────────────────────────────────────
WPV_NEW_METHODS = '''\

    # ── Role-based well appearance (v7.3.1-rolecolors) ───────────

    def update_all_wells(
        self,
        appearances: "dict[str, tuple]",
    ) -> None:
        """
        Set fill colour for every well from its role.

        appearances: {well_name: (WellRole, display_label)}
        Falls back gracefully when ROLE_COLORS is unavailable.
        """
        try:
            from SupportClasses.PhysicalModels import ROLE_COLORS
        except ImportError:
            ROLE_COLORS = {}

        for name, (role, label) in appearances.items():
            item = self._well_items.get(name)
            if item is None:
                continue
            hex_color = ROLE_COLORS.get(role, "#585b70")
            from PySide6.QtGui import QBrush, QColor
            item.setBrush(QBrush(QColor(hex_color)))
            item.setToolTip(f"{name}: {label or role.value if hasattr(role, 'value') else str(role)}")

    def set_well_color(self, name: str, hex_color: str) -> None:
        """Set fill colour for a single well (hex string)."""
        from PySide6.QtGui import QBrush, QColor
        item = self._well_items.get(name)
        if item:
            item.setBrush(QBrush(QColor(hex_color)))

    def set_well_border_color(self, name: str, hex_color: str) -> None:
        """Set border (pen) colour for a single well (hex string)."""
        from PySide6.QtGui import QPen, QColor
        item = self._well_items.get(name)
        if item:
            pen = item.pen()
            pen.setColor(QColor(hex_color))
            pen.setWidthF(2.0 if hex_color not in ("#585b70", "#45475a") else 0.5)
            item.setPen(pen)

    def update_well_rosettes(
        self,
        rosette_map: "dict[str, object]",
        plate: "object | None" = None,
    ) -> None:
        """
        Draw rosette subwells inside wells that have a rosette attached.

        rosette_map: {well_name: RosetteInsert | None}
        plate: WellPlate used to look up well diameter (optional)
        """
        # Remove any existing rosette child items
        for name, items in list(getattr(self, "_rosette_items", {}).items()):
            for it in items:
                self._scene.removeItem(it)
        self._rosette_items = {}

        if not rosette_map:
            return

        try:
            from SupportClasses.PhysicalModels import ROLE_COLORS, WellRole
        except ImportError:
            return
        from PySide6.QtGui import QBrush, QColor, QPen

        # Determine scale from existing items
        if not self._well_items or not self._plate:
            return

        # Use first well to estimate pixel-to-mm ratio
        sample_well = next(iter(self._plate.get_all_wells()), None)
        if sample_well is None:
            return
        sample_item = self._well_items.get(sample_well.name)
        if sample_item is None:
            return

        # pixel radius of a well in scene coordinates
        well_r_px = sample_item.rect().width() / 2.0
        # mm radius of a well
        from SupportClasses.WellPlate import PLATE_DEFINITIONS
        defn = PLATE_DEFINITIONS.get(self._plate.format_key, {})
        well_diam_mm = defn.get("well_diameter_mm", 6.4) if defn else 6.4
        well_r_mm = well_diam_mm / 2.0
        px_per_mm = well_r_px / max(well_r_mm, 0.01)

        for well_name, rosette in rosette_map.items():
            if rosette is None:
                continue
            parent_item = self._well_items.get(well_name)
            if parent_item is None:
                continue
            cx = parent_item.rect().center().x()
            cy = parent_item.rect().center().y()

            child_items = []
            for sw in getattr(rosette, "subwells", []):
                # Sub-well offset in mm → pixels
                try:
                    dx_mm, dy_mm = rosette.get_subwell_xy(sw.index)
                except Exception:
                    import math
                    angle_r = math.radians(getattr(sw, "angle_deg", 0))
                    r_mm    = getattr(sw, "radial_offset_mm", 0)
                    dx_mm   = r_mm * math.sin(angle_r)
                    dy_mm   = -r_mm * math.cos(angle_r)

                sw_diam_mm = getattr(sw, "diameter_mm", 1.5)
                sw_r_px    = max((sw_diam_mm / 2.0) * px_per_mm, 1.5)
                scx        = cx + dx_mm * px_per_mm
                scy        = cy + dy_mm * px_per_mm

                # Pick colour from subwell role if available
                sw_role = None
                sw_color = "#a6adc8"
                it = self._scene.addEllipse(
                    scx - sw_r_px, scy - sw_r_px,
                    2 * sw_r_px, 2 * sw_r_px,
                    QPen(QColor("#1e1e2e"), 0.5),
                    QBrush(QColor(sw_color)),
                )
                it.setZValue(2)
                child_items.append(it)

            self._rosette_items[well_name] = child_items

'''

# ── print_well_setup.py replacements ─────────────────────────────

NEW_REFRESH_COLORS = '''\
    def _refresh_well_colors(self) -> None:
        """Update all well fill colors from role (v7.3.1-rolecolors)."""
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return
        appearances: dict = {}
        for name, wa in self._model.assignments.items():
            appearances[name] = (wa.role, wa.get_display_label()
                                 if hasattr(wa, "get_display_label")
                                 else wa.role.value)
        if hasattr(pv, "update_all_wells"):
            try:
                pv.update_all_wells(appearances)
            except Exception:
                pass
        # Apply selection border on top
        self._apply_selection_borders()

    def _apply_selection_borders(self) -> None:
        """Purple border for selected wells; bright white for taught points."""
        pv = getattr(self, "plate_view", None)
        if pv is None or not hasattr(pv, "set_well_border_color"):
            return
        selected = set()
        if hasattr(pv, "get_selected_wells"):
            try:
                selected = set(pv.get_selected_wells())
            except Exception:
                pass
        COLOR_PURPLE = COLORS.get("mauve", "#cba6f7")
        COLOR_DEFAULT = "#585b70"
        for name in self._model.assignments:
            color = COLOR_PURPLE if name in selected else COLOR_DEFAULT
            try:
                pv.set_well_border_color(name, color)
            except Exception:
                pass

    def _refresh_well_status_colors(self) -> None:
        """Alias kept for compatibility — delegates to _refresh_well_colors."""
        self._refresh_well_colors()

    def _refresh_well_rosettes(self) -> None:
        """Push rosette data to plate view for subwell rendering."""
        pv = getattr(self, "plate_view", None)
        if pv is None or not hasattr(pv, "update_well_rosettes"):
            return
        rosette_map: dict = {}
        for name, wa in self._model.assignments.items():
            rn = getattr(wa, "rosette_name", None)
            if rn and self._workspace:
                rosette = self._workspace.get_rosette(rn) if hasattr(self._workspace, "get_rosette") else None
                if rosette:
                    rosette_map[name] = rosette
        try:
            pv.update_well_rosettes(rosette_map, self._model.plate)
        except Exception:
            pass

'''

def patch_well_plate_view(root: Path):
    target = root / "gui" / "widgets" / "well_plate_view.py"
    print(f"{CYAN}  WellPlateView: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); return False

    content = target.read_text(encoding="utf-8")
    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return True

    # Find insertion point: just before `def resizeEvent` or end of class
    anchor = "\n    def resizeEvent"
    if anchor not in content:
        anchor = "\n    def mousePressEvent"
    if anchor not in content:
        print(f"  {YELLOW}WARN: cannot find anchor in WellPlateView — injecting before EOF{RESET}")
        content += WPV_NEW_METHODS
    else:
        idx = content.index(anchor)
        content = content[:idx] + WPV_NEW_METHODS + content[idx:]

    if not ast_ok(content, "WellPlateView methods"):
        return False

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}WellPlateView methods added{RESET}")
    return True


def patch_well_setup(root: Path):
    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"{CYAN}  WellSetupTab: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); return False

    content = target.read_text(encoding="utf-8")
    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return True

    changed = False

    # Replace _refresh_well_status_colors (or _refresh_well_colors if present)
    # Find either method and replace up to the next `def `
    for method_name in ("_refresh_well_status_colors", "_refresh_well_colors"):
        pattern = re.compile(
            r'\n    def ' + method_name + r'\(self\).*?(?=\n    def )',
            re.DOTALL)
        m = pattern.search(content)
        if m:
            content = content[:m.start()] + "\n" + NEW_REFRESH_COLORS + content[m.end():]
            if not ast_ok(content, f"replace {method_name}"):
                return False
            print(f"  {GREEN}Replaced {method_name}() with role-color version{RESET}")
            changed = True
            break
    else:
        # No existing method — inject before _check_well_ready
        anchor = "\n    def _check_well_ready"
        if anchor in content:
            content = content.replace(anchor, "\n" + NEW_REFRESH_COLORS + anchor, 1)
            if not ast_ok(content, "inject color methods"):
                return False
            print(f"  {GREEN}Injected color methods{RESET}")
            changed = True
        else:
            print(f"  {YELLOW}WARN: no suitable anchor for color methods{RESET}")

    # Wire _refresh_well_rosettes into _refresh_plate
    if "_refresh_well_rosettes" not in content:
        old = "        self._refresh_well_status_colors()\n        self._refresh_summary_table()"
        if old not in content:
            old = "        self._refresh_well_colors()\n        self._refresh_summary_table()"
        if old in content:
            content = content.replace(
                old,
                old + "\n        self._refresh_well_rosettes()",
                1)
            if not ast_ok(content, "rosette wire"):
                return False
            print(f"  {GREEN}_refresh_well_rosettes() wired into _refresh_plate{RESET}")
            changed = True
    else:
        print(f"  {YELLOW}SKIP: _refresh_well_rosettes already present{RESET}")

    if not changed:
        print(f"  {YELLOW}Nothing to do{RESET}"); return True

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}print_well_setup.py patched{RESET}")
    return True


def main():
    root = find_root()
    ok1 = patch_well_plate_view(root)
    ok2 = patch_well_setup(root)
    if not (ok1 and ok2):
        sys.exit(1)
    print(f"{GREEN}All well color patches applied.{RESET}")

if __name__ == "__main__":
    main()
