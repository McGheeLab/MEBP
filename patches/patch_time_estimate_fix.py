#!/usr/bin/env python3
"""
MEBP v7.2.6 — Time Estimate Fix
Fixes feedrate unit mismatches in PrintTrajectoryPlanner and _get_settings()
that cause Z and travel moves to be computed at wrong (clamped) speeds,
producing grossly incorrect print time estimates.

Root causes:
  A) _get_settings(): z_feedrate stored in mm/s but planner treats it as mm/min
     → Z at 0.1 mm/s (clamped) instead of 1.0 mm/s
  B) _travel_to_well(): settings.xy_feedrate (mm/s) used as mm/min
     → Travel at 0.1 mm/s (clamped) instead of 10-100 mm/s
  C) generate() RETURN_HOME: same xy_feedrate bug
  D) _get_settings(): pump_feedrate never set from GUI pump_rate_uL_s

Changes:
  1. print_setup.py  — _get_settings(): fix z_feedrate units + pump_feedrate
  2. PrintTrajectoryPlanner.py — _travel_to_well(): use travel_speed_mm_s
  3. PrintTrajectoryPlanner.py — generate() RETURN_HOME: use travel_speed_mm_s
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colours ────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"

ok_count   = 0
skip_count = 0
fail_count = 0

def log_ok(msg):   global ok_count;   ok_count   += 1; print(f"  {GREEN}✓ OK{RESET}   {msg}")
def log_skip(msg): global skip_count; skip_count += 1; print(f"  {YELLOW}○ SKIP{RESET} {msg}")
def log_fail(msg): global fail_count; fail_count += 1; print(f"  {RED}✗ FAIL{RESET} {msg}")

# ── Project root detection ─────────────────────────────────────────────────
def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for candidate in [p, p.parent, p.parent.parent]:
            if (candidate / "SupportClasses").is_dir() and (candidate / "gui").is_dir():
                return candidate
    raise RuntimeError("Cannot find MEBP project root (needs SupportClasses/ + gui/)")

# ── File helpers ───────────────────────────────────────────────────────────
def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")

def safe_write(path: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        log_fail(f"AST error in {label}: {e}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bak = path.with_suffix(f".bak_v726_tef_{ts}")
    shutil.copy2(path, bak)
    path.write_text(content, encoding="utf-8")
    return True

def find_method(content: str, name: str):
    """Find class method boundaries. Returns re.Match or None."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self[^)]*\)[^:]*:[ \t]*\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)

# ══════════════════════════════════════════════════════════════════════════
# PATCH A — print_setup.py: fix _get_settings()
#   A1: z_feedrate: store mm/min (× 60) not mm/s
#   A2: pump_feedrate: derive from pump_rate_uL_s
# ══════════════════════════════════════════════════════════════════════════
def patch_print_setup(root: Path):
    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        log_fail("print_setup.py not found")
        return

    changed = False

    # ── A1: z_feedrate unit fix ──────────────────────────────────────────
    MARKER_A1 = "v7.2.6-tef-A1: z_feedrate mm/min"
    if MARKER_A1 not in content:
        # Match the exact assignment line (allow leading spaces to vary)
        pat = re.compile(
            r'([ \t]+)(s\.z_feedrate\s*=\s*self\.z_feed_spin\.value\(\)[ \t]*\n)',
            re.MULTILINE
        )
        m = pat.search(content)
        if m:
            indent = m.group(1)
            old_line = m.group(0)
            new_line = (
                f"{indent}# {MARKER_A1}\n"
                f"{indent}# z_feed_spin is in mm/s; planner _move_z divides by 60 expecting mm/min\n"
                f"{indent}s.z_feedrate = self.z_feed_spin.value() * 60.0  # mm/s → mm/min\n"
            )
            content = content[:m.start()] + new_line + content[m.end():]
            log_ok("A1: z_feedrate converted mm/s → mm/min in _get_settings()")
            changed = True
        else:
            log_fail("A1: Could not find 's.z_feedrate = self.z_feed_spin.value()' in print_setup.py")
    else:
        log_skip("A1: z_feedrate fix already applied")

    # ── A2: pump_feedrate derived from pump_rate_uL_s ─────────────────────
    MARKER_A2 = "v7.2.6-tef-A2: pump_feedrate from uL_s"
    if MARKER_A2 not in content:
        # Insert after the pump_rate_uL_s assignment
        pat = re.compile(
            r'([ \t]+)(s\.pump_rate_uL_s\s*=\s*self\.pump_feed_spin\.value\(\)[ \t]*\n)',
            re.MULTILINE
        )
        m = pat.search(content)
        if m:
            indent = m.group(1)
            after_pos = m.end()
            inject = (
                f"{indent}# {MARKER_A2}\n"
                f"{indent}# Convert µL/s → mm/min for service pump moves (default Hamilton 250µL = 3.378 µL/mm)\n"
                f"{indent}_uL_per_mm_default = 3.378\n"
                f"{indent}s.pump_feedrate = max(self.pump_feed_spin.value() * 60.0 / _uL_per_mm_default, 0.5)\n"
            )
            content = content[:after_pos] + inject + content[after_pos:]
            log_ok("A2: pump_feedrate derived from pump_rate_uL_s in _get_settings()")
            changed = True
        else:
            log_fail("A2: Could not find 's.pump_rate_uL_s = self.pump_feed_spin.value()' in print_setup.py")
    else:
        log_skip("A2: pump_feedrate fix already applied")

    if changed:
        if safe_write(path, content, "print_setup.py"):
            log_ok("print_setup.py written successfully")
        else:
            log_fail("print_setup.py write failed (AST error)")

# ══════════════════════════════════════════════════════════════════════════
# PATCH B — PrintTrajectoryPlanner.py
#   B1: _travel_to_well(): use travel_speed_mm_s * 60 instead of xy_feedrate
#   B2: generate() RETURN_HOME: same fix
# ══════════════════════════════════════════════════════════════════════════
def patch_trajectory_planner(root: Path):
    path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    content = safe_read(path)
    if not content:
        log_fail("PrintTrajectoryPlanner.py not found")
        return

    changed = False

    # ── B1: Replace _travel_to_well method ───────────────────────────────
    MARKER_B1 = "v7.2.6-tef-B1: travel_speed_mm_s"
    if MARKER_B1 not in content:
        m = find_method(content, "_travel_to_well")
        if m:
            new_method = '''    def _travel_to_well(self, wx: float, wy: float, settings, well=""):
        """Raise Z → travel XY → (stay at travel height).
        # v7.2.6-tef-B1: travel_speed_mm_s
        Uses settings.travel_speed_mm_s (mm/s) × 60 for correct mm/min feedrate.
        Previously used settings.xy_feedrate which is stored in mm/s but was
        passed directly to _move_xy() which divides by 60 (expecting mm/min),
        causing travel to run at 0.1 mm/s (clamped minimum) instead of 10+ mm/s.
        """
        self._next_segment()
        # Raise to travel height if not there
        tz = settings.travel_z_height
        if self._z < tz - 0.01:
            self._move_z(tz, settings.z_feedrate, segment="travel", well=well)
        # XY travel — use travel_speed_mm_s (mm/s) × 60 → mm/min
        _travel_fr = getattr(settings, 'travel_speed_mm_s', 10.0) * 60.0
        self._move_xy(wx, wy, _travel_fr, segment="travel", well=well)
        # Dwell after travel
        if settings.dwell_after_move > 0:
            self._dwell(settings.dwell_after_move, well=well)

'''
            content = content[:m.start()] + new_method + content[m.end():]
            log_ok("B1: _travel_to_well() fixed to use travel_speed_mm_s * 60")
            changed = True
        else:
            log_fail("B1: Could not find _travel_to_well method in PrintTrajectoryPlanner.py")
    else:
        log_skip("B1: _travel_to_well fix already applied")

    # ── B2: RETURN_HOME _move_xy in generate() ───────────────────────────
    MARKER_B2 = "v7.2.6-tef-B2: return_home travel"
    if MARKER_B2 not in content:
        # Match the specific _move_xy(0, 0, settings.xy_feedrate ...) in generate()
        pat = re.compile(
            r'([ \t]+)(self\._move_xy\(0,\s*0,\s*settings\.xy_feedrate,\s*segment=["\']travel["\']\))',
            re.MULTILINE
        )
        m = pat.search(content)
        if m:
            indent = m.group(1)
            old_call = m.group(0)
            new_call = (
                f"{indent}# {MARKER_B2}: use travel_speed_mm_s * 60 (mm/min)\n"
                f"{indent}_rh_travel_fr = getattr(settings, 'travel_speed_mm_s', 10.0) * 60.0\n"
                f"{indent}self._move_xy(0, 0, _rh_travel_fr, segment='travel')"
            )
            content = content[:m.start()] + new_call + content[m.end():]
            log_ok("B2: RETURN_HOME _move_xy fixed to use travel_speed_mm_s * 60")
            changed = True
        else:
            log_fail("B2: Could not find RETURN_HOME _move_xy(0, 0, settings.xy_feedrate ...) in PrintTrajectoryPlanner.py")
    else:
        log_skip("B2: RETURN_HOME travel fix already applied")

    if changed:
        if safe_write(path, content, "PrintTrajectoryPlanner.py"):
            log_ok("PrintTrajectoryPlanner.py written successfully")
        else:
            log_fail("PrintTrajectoryPlanner.py write failed (AST error)")

# ══════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════
def main():
    print(f"\n{CYAN}══════════════════════════════════════════════════════════{RESET}")
    print(f"{CYAN}  MEBP v7.2.6 — Time Estimate Fix (feedrate unit mismatches){RESET}")
    print(f"{CYAN}══════════════════════════════════════════════════════════{RESET}\n")

    try:
        root = find_root()
        print(f"Project root: {root}\n")
    except RuntimeError as e:
        print(f"{RED}ERROR: {e}{RESET}")
        sys.exit(1)

    print(f"{CYAN}── PATCH A: print_setup.py ──{RESET}")
    patch_print_setup(root)
    print()

    print(f"{CYAN}── PATCH B: PrintTrajectoryPlanner.py ──{RESET}")
    patch_trajectory_planner(root)
    print()

    total = ok_count + skip_count + fail_count
    print(f"{CYAN}══ Summary: {ok_count} OK  {skip_count} SKIP  {fail_count} FAIL  (of {total} steps) ══{RESET}\n")

    if fail_count > 0:
        print(f"{RED}⚠  Some patches failed — check output above. Backups created before any writes.{RESET}\n")
        sys.exit(1)
    elif skip_count == total:
        print(f"{YELLOW}All patches already applied — no changes made.{RESET}\n")
    else:
        print(f"{GREEN}All patches applied. Run AST checks + smoke test.{RESET}\n")
        print("Verification commands:")
        print('  python3 -c "import ast; ast.parse(open(\'gui/pages/print_setup.py\').read())"')
        print('  python3 -c "import ast; ast.parse(open(\'SupportClasses/PrintTrajectoryPlanner.py\').read())"')
        print()

if __name__ == "__main__":
    main()
