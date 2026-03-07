#!/usr/bin/env python3
"""
Fix the mm/µm unit system throughout the execution chain.

The Prior ProScan II works in µm (manual page 36: "1µm per number").
WellPlate positions are in mm. These must be converted.

This patch makes StageController the conversion boundary:
  - move_xy_absolute(x_mm, y_mm) — NEW: accepts mm, converts to µm internally
  - get_xy_position() — still returns µm (raw from stage, backward compat)

The conversion happens inside StageController so:
  - All callers (trajectory, print manager, jog, calibration) can use mm
  - XYStage/simulator always gets µm
  - zero_position stays in µm (it's what the stage reports)
  - Safety limits stay in µm

Also fixes:
  - print_monitor.py position display (µm → mm conversion)
  - send_velocity_xy docstring (VS command uses µm/s per manual)
"""

import ast, re, sys, shutil, os
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"; B = "\033[1m"
_ok = 0; _skip = 0; _miss = 0
def ok(m): global _ok; _ok += 1; print(f"  {G}✓{X} {m}")
def skip(m): global _skip; _skip += 1; print(f"  {Y}○{X} SKIP: {m}")
def miss(m): global _miss; _miss += 1; print(f"  {R}✗{X} MISS: {m}")

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "SupportClasses").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "SupportClasses").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def safe_write(path, content, label):
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL {path.name}: {e}{X}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if path.exists():
        shutil.copy2(path, path.with_suffix(f".bak_units_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})")
    return True


def patch_stage_controller(root):
    """Patch StageController: move_xy_absolute accepts mm, converts to µm."""
    print(f"\n{B}1. StageController.py — mm→µm conversion{X}")
    path = root / "SupportClasses" / "StageController.py"
    content = path.read_text("utf-8")
    changed = False

    marker = "v7.3: Accept mm, convert to µm internally"
    if marker in content:
        skip("StageController already patched")
        return

    # Replace move_xy_absolute body using exact string matching
    # Match the core pattern: safety clamp → add zero → send to stage
    old_body = """        if not self.xy_stage:
            return
        if self.safety_limits.enabled and from_zero_ref:
            x, y = self.safety_limits.clamp_xy(x, y)
        if from_zero_ref:
            x += self.zero_position["x"]
            y += self.zero_position["y"]
        self.xy_stage.move_stage_to_position(x, y, fast)"""

    new_body = """        \"\"\"v7.3: Accept mm, convert to µm internally.

        When from_zero_ref=True, x/y are in mm (from WellPlate/trajectory).
        Converts to µm, applies safety limits, adds zero reference (µm),
        then sends to stage which expects µm (Prior manual page 36).
        \"\"\"
        if not self.xy_stage:
            return

        if from_zero_ref:
            # Convert mm → µm, then add zero reference (which is in µm)
            x_um = x * 1000.0
            y_um = y * 1000.0
            if self.safety_limits.enabled:
                x_um, y_um = self.safety_limits.clamp_xy(x_um, y_um)
            x_um += self.zero_position["x"]
            y_um += self.zero_position["y"]
        else:
            # Legacy: raw values passed directly (assumed µm already)
            x_um = x
            y_um = y

        self.xy_stage.move_stage_to_position(x_um, y_um, fast)"""

    if old_body in content:
        content = content.replace(old_body, new_body)
        ok("move_xy_absolute: mm→µm conversion added")
        changed = True
    else:
        miss("move_xy_absolute body not found (exact match)")

    # Replace move_xy_relative — keep in µm (jog uses µm)
    # But add clear docstring
    old_rel_doc = '        Move XY stage by a relative offset (microsteps).'
    new_rel_doc = '        Move XY stage by a relative offset in µm.'
    if old_rel_doc in content:
        content = content.replace(old_rel_doc, new_rel_doc)
        ok("move_xy_relative docstring: microsteps → µm")
        changed = True

    # Fix send_velocity_xy docstring
    old_vel_doc = "typically µsteps/s for Prior"
    new_vel_doc = "µm/s for Prior (VS command default unit per manual)"
    if old_vel_doc in content:
        content = content.replace(old_vel_doc, new_vel_doc)
        ok("send_velocity_xy docstring: µsteps/s → µm/s")
        changed = True

    # Add get_xy_position_mm helper
    helper_marker = "def get_xy_position_mm("
    if helper_marker not in content:
        # Find get_xy_position and insert after it
        pos_m = re.search(
            r'^(    def get_xy_position\(self.*?\n)(.*?)(?=\n    def )',
            content, re.DOTALL | re.MULTILINE)
        if pos_m:
            helper = '''
    def get_xy_position_mm(self, cached: bool = True) -> tuple:
        """Get XY position in mm (relative to zero reference).

        v7.3: Convenience method for callers that need mm.
        Returns (x_mm, y_mm) or (None, None) if not connected.
        """
        pos = self.get_xy_position(cached)
        if pos[0] is not None:
            zx = self.zero_position.get("x", 0)
            zy = self.zero_position.get("y", 0)
            return ((pos[0] - zx) / 1000.0, (pos[1] - zy) / 1000.0)
        return (None, None)

'''
            insert_at = pos_m.end()
            content = content[:insert_at] + helper + content[insert_at:]
            ok("Added get_xy_position_mm() helper")
            changed = True
        else:
            miss("Could not find get_xy_position to insert helper after")

    if changed:
        safe_write(path, content, "mm/µm conversion")


def patch_print_monitor(root):
    """Fix print_monitor.py position display — convert µm → mm."""
    print(f"\n{B}2. print_monitor.py — position display{X}")
    path = root / "gui" / "pages" / "print_monitor.py"
    content = path.read_text("utf-8")
    changed = False

    marker = "v7.3: Convert µm → mm for display"
    if marker in content:
        skip("Monitor already patched")
        return

    # Fix on_status_update: divide by 1000 for mm
    old_pos = '''            px = xy[0] - zero.get('x', 0)
            py = xy[1] - zero.get('y', 0)'''
    new_pos = '''            # v7.3: Convert µm → mm for display
            px = (xy[0] - zero.get('x', 0)) / 1000.0
            py = (xy[1] - zero.get('y', 0)) / 1000.0'''
    if old_pos in content:
        content = content.replace(old_pos, new_pos)
        ok("on_status_update: µm → mm conversion")
        changed = True
    else:
        # Try alternate pattern
        miss("Position conversion pattern not found in on_status_update")

    if changed:
        safe_write(path, content, "µm→mm display")


def patch_velocity_executor(root):
    """Fix VelocityExecutor position reading — convert µm → mm."""
    print(f"\n{B}3. VelocityExecutor.py — position reading{X}")
    path = root / "SupportClasses" / "VelocityExecutor.py"
    if not path.exists():
        skip("VelocityExecutor.py not found")
        return
    content = path.read_text("utf-8")

    marker = "v7.3: Convert µm → mm"
    if marker in content:
        skip("VelocityExecutor already patched")
        return

    old_read = '''                        if xy[0] is not None:
                            measured_x = xy[0] - zero.get('x', 0)
                            measured_y = xy[1] - zero.get('y', 0)'''
    new_read = '''                        if xy[0] is not None:
                            # v7.3: Convert µm → mm for controller
                            measured_x = (xy[0] - zero.get('x', 0)) / 1000.0
                            measured_y = (xy[1] - zero.get('y', 0)) / 1000.0'''
    if old_read in content:
        content = content.replace(old_read, new_read)
        ok("Position reading: µm → mm")
        safe_write(path, content, "µm→mm")
    else:
        miss("Position reading pattern not found")


def patch_proscan_json(root):
    """Fix proscan_ii.json position_units."""
    print(f"\n{B}4. proscan_ii.json — position_units{X}")
    import json
    path = root / "config" / "controllers" / "proscan_ii.json"
    if not path.exists():
        skip("proscan_ii.json not found")
        return
    data = json.loads(path.read_text())
    params = data.get("parameters", {})
    old_units = params.get("position_units", "")
    if old_units == "microns":
        skip("Already set to microns")
        return
    params["position_units"] = "microns"
    data["parameters"] = params
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_units_{ts}"))
    path.write_text(json.dumps(data, indent=4))
    ok(f"position_units: '{old_units}' → 'microns'")


def main():
    print(f"\n{B}{'='*60}")
    print(f"  v7.3 Unit System Fix — mm/µm alignment")
    print(f"{'='*60}{X}")
    root = find_root()
    print(f"  Root: {root}\n")

    patch_stage_controller(root)
    patch_print_monitor(root)
    patch_velocity_executor(root)
    patch_proscan_json(root)

    # Clean cache
    for dp, dn, _ in os.walk(root):
        if "__pycache__" in dn:
            shutil.rmtree(Path(dp) / "__pycache__")
    ok("Cleaned __pycache__")

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK: {_ok}{X}  {Y}SKIP: {_skip}{X}  {R}MISS: {_miss}{X}")
    if _miss:
        print(f"\n  {R}⚠ Some patches need manual review{X}")
    else:
        print(f"\n  {G}✓ Done! python main.py{X}")

    print(f"""
  Unit chain after fix:
    WellPlate (mm) → Planner (mm) → Executor → StageController → Stage
                                                    ↓
                                         move_xy_absolute(x_mm, y_mm)
                                         x_um = x_mm × 1000
                                         x_um += zero_position (µm)
                                         → G {{x_um}},{{y_um}}
                                                    ↓
                                              Prior (µm)
""")

if __name__ == "__main__":
    main()
