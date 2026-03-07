#!/usr/bin/env python3
"""
MEBP v7.2.6 — Auto-Settings & Smart Z Approach
Automatically derives all motion parameters from hardware config + calibration.

Changes:
  A  — PrintSettings: add top_z_height, fast_z_feedrate_mm_min,
       entry_z_feedrate_mm_min, service_xy_speed_mm_s,
       auto_pump_rate_uL_s, service_pump_rate_uL_s
  B1 — PrintTrajectoryPlanner._travel_to_well(): 4-phase smart Z approach
  B2 — PrintTrajectoryPlanner._lower_to_print(): 2-phase slow entry
  B3 — PrintTrajectoryPlanner._raise_from_print(): 2-phase fast exit
  B4 — PrintTrajectoryPlanner._print_path_coordinated(): auto pump rate
  C1 — PrintSetupPage._compute_auto_settings(): derive all from hw+cal
  C2 — PrintSetupPage._generate_print(): call _compute_auto_settings()
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"

ok_count = skip_count = fail_count = 0

def log_ok(m):   global ok_count;   ok_count   += 1; print(f"  {GREEN}✓ OK{RESET}   {m}")
def log_skip(m): global skip_count; skip_count += 1; print(f"  {YELLOW}○ SKIP{RESET} {m}")
def log_fail(m): global fail_count; fail_count += 1; print(f"  {RED}✗ FAIL{RESET} {m}")

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c
    raise RuntimeError("Cannot find MEBP project root")

def safe_read(path: Path) -> str:
    return path.read_text(encoding="utf-8") if path.exists() else ""

def safe_write(path: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        log_fail(f"AST error in {label}: {e}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v726_as_{ts}"))
    path.write_text(content, encoding="utf-8")
    return True

def find_method(content: str, name: str):
    """Find class method boundaries (4-space indent). Returns re.Match or None."""
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self[^)]*\)[^:]*:[ \t]*\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

# ══════════════════════════════════════════════════════════════════════════
# PATCH A — PrintManager.py: add new PrintSettings fields
# ══════════════════════════════════════════════════════════════════════════
def patch_print_settings(root: Path):
    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        log_fail("PrintManager.py not found")
        return

    MARKER = "v7.2.6-auto: top_z_height"
    if MARKER in content:
        log_skip("A: PrintSettings new fields already added")
        return

    # Insert after pump_rates_uL_s field definition
    pat = re.compile(
        r'(    pump_rates_uL_s:\s*dict\s*=\s*field\([^\n]*\)\n)',
        re.MULTILINE
    )
    m = pat.search(content)
    if not m:
        log_fail("A: Cannot find pump_rates_uL_s field anchor in PrintSettings")
        return

    inject = (
        "\n"
        "    # v7.2.6-auto: top_z_height — calibration-derived Z heights and motion tiers\n"
        "    top_z_height: float = 0.0           # plate top surface (from calibration top_z)\n"
        "    fast_z_feedrate_mm_min: float = 120.0   # Z speed above well top (service/travel)\n"
        "    entry_z_feedrate_mm_min: float = 12.0   # Z speed entering well (slow, safe)\n"
        "    service_xy_speed_mm_s: float = 50.0     # XY speed for service moves (waste/wash/ink)\n"
        "    auto_pump_rate_uL_s: float = 0.0        # print pump rate from extrusion physics\n"
        "    service_pump_rate_uL_s: float = 5.0     # service pump rate (max for gauge)\n"
    )
    content = content[:m.end()] + inject + content[m.end():]

    if safe_write(path, content, "PrintManager.py"):
        log_ok("A: PrintSettings new fields added to PrintManager.py")

# ══════════════════════════════════════════════════════════════════════════
# PATCH B1 — PrintTrajectoryPlanner._travel_to_well(): 4-phase smart Z
# ══════════════════════════════════════════════════════════════════════════
def patch_travel_to_well(content: str) -> str:
    MARKER = "v7.2.6-auto-B1"
    if MARKER in content:
        log_skip("B1: _travel_to_well already patched")
        return content

    m = find_method(content, "_travel_to_well")
    if not m:
        log_fail("B1: _travel_to_well not found")
        return content

    new_method = '''    def _travel_to_well(self, wx: float, wy: float, settings, well=""):
        """4-phase smart Z approach. # v7.2.6-auto-B1

        Phase 1: Fast Z up to safe_z (travel_z_height) if not already there.
        Phase 2: Fast XY to destination at service_xy_speed_mm_s.
        Phase 3: Fast Z down to (top_z + 0.5mm buffer) — above well opening.
        Phase 4: Slow Z entry to print_z_height — controlled needle insertion.

        Falls back to single-phase if top_z_height == 0 (calibration not done).
        """
        self._next_segment()
        safe_z   = settings.travel_z_height
        top_z    = getattr(settings, 'top_z_height', 0.0)
        fast_z   = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z  = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        svc_xy   = getattr(settings, 'service_xy_speed_mm_s', 10.0) * 60.0  # → mm/min

        # Phase 1: raise to safe_z at fast speed
        if self._z < safe_z - 0.01:
            self._move_z(safe_z, fast_z, segment="travel", well=well)

        # Phase 2: fast XY travel
        self._move_xy(wx, wy, svc_xy, segment="travel", well=well)

        # Phase 3+4: lower into well if calibration data available
        if top_z > 0:
            approach_z = top_z + 0.5  # 0.5 mm buffer above well top
            # Phase 3: fast Z to just above well
            if self._z > approach_z + 0.01:
                self._move_z(approach_z, fast_z, segment="travel", well=well)
            # Phase 4: slow Z entry to print height
            self._move_z(settings.print_z_height, entry_z, segment="travel", well=well)
        else:
            # Fallback: single-phase lower (no calibration)
            self._move_z(settings.print_z_height, settings.z_feedrate,
                         segment="travel", well=well)

        # Dwell after arrival
        if settings.dwell_after_move > 0:
            self._dwell(settings.dwell_after_move, well=well)

'''
    content = content[:m.start()] + new_method + content[m.end():]
    log_ok("B1: _travel_to_well() replaced with 4-phase smart Z approach")
    return content

# ══════════════════════════════════════════════════════════════════════════
# PATCH B2 — PrintTrajectoryPlanner._lower_to_print(): 2-phase
# ══════════════════════════════════════════════════════════════════════════
def patch_lower_to_print(content: str) -> str:
    MARKER = "v7.2.6-auto-B2"
    if MARKER in content:
        log_skip("B2: _lower_to_print already patched")
        return content

    m = find_method(content, "_lower_to_print")
    if not m:
        log_fail("B2: _lower_to_print not found")
        return content

    new_method = '''    def _lower_to_print(self, settings, well=""):
        """Lower Z to print height. 2-phase if calibration available. # v7.2.6-auto-B2

        Phase 1: Fast Z to (top_z + 0.5mm) if not already below that.
        Phase 2: Slow entry to print_z_height.
        Falls back to single-phase (z_feedrate) if top_z_height == 0.
        """
        top_z   = getattr(settings, 'top_z_height', 0.0)
        fast_z  = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        target  = settings.print_z_height

        if self._z <= target + 0.001:
            return  # Already at or below print height

        if top_z > 0:
            approach_z = top_z + 0.5
            if self._z > approach_z + 0.01:
                # Fast lower to just above well top
                self._move_z(approach_z, fast_z, segment="travel", well=well)
            # Slow entry into well
            self._move_z(target, entry_z, segment="travel", well=well)
        else:
            self._move_z(target, settings.z_feedrate, segment="travel", well=well)

'''
    content = content[:m.start()] + new_method + content[m.end():]
    log_ok("B2: _lower_to_print() replaced with 2-phase entry")
    return content

# ══════════════════════════════════════════════════════════════════════════
# PATCH B3 — PrintTrajectoryPlanner._raise_from_print(): 2-phase fast exit
# ══════════════════════════════════════════════════════════════════════════
def patch_raise_from_print(content: str) -> str:
    MARKER = "v7.2.6-auto-B3"
    if MARKER in content:
        log_skip("B3: _raise_from_print already patched")
        return content

    m = find_method(content, "_raise_from_print")
    if not m:
        log_fail("B3: _raise_from_print not found")
        return content

    new_method = '''    def _raise_from_print(self, settings, well=""):
        """Raise Z from print height to safe travel height. # v7.2.6-auto-B3

        Phase 1: Slow exit from well (entry_z_feedrate) to top_z + 0.5mm.
        Phase 2: Fast raise to safe_z (fast_z_feedrate) to clear obstacles.
        Falls back to single-phase if top_z_height == 0.
        """
        top_z   = getattr(settings, 'top_z_height', 0.0)
        fast_z  = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        safe_z  = settings.travel_z_height

        if top_z > 0:
            clear_z = top_z + 0.5
            # Phase 1: slow exit from well
            if self._z < clear_z - 0.01:
                self._move_z(clear_z, entry_z, segment="travel", well=well)
            # Phase 2: fast raise to safe travel height
            if self._z < safe_z - 0.01:
                self._move_z(safe_z, fast_z, segment="travel", well=well)
        else:
            self._move_z(safe_z, settings.z_feedrate, segment="travel", well=well)

'''
    content = content[:m.start()] + new_method + content[m.end():]
    log_ok("B3: _raise_from_print() replaced with 2-phase exit")
    return content

# ══════════════════════════════════════════════════════════════════════════
# PATCH B4 — PrintTrajectoryPlanner._print_path_coordinated(): auto pump
# ══════════════════════════════════════════════════════════════════════════
def patch_print_path_coordinated(content: str) -> str:
    MARKER = "v7.2.6-auto-B4"
    if MARKER in content:
        log_skip("B4: _print_path_coordinated already patched")
        return content

    m = find_method(content, "_print_path_coordinated")
    if not m:
        log_fail("B4: _print_path_coordinated not found")
        return content

    new_method = '''    def _print_path_coordinated(self, path_points, well_x, well_y,
                                pump_id, flow_rate, feedrate_mm_min,
                                settings, well_name=""):
        """Coordinated XY + pump waypoints. # v7.2.6-auto-B4

        Auto-derives pump flow rate from extrusion physics if
        settings.auto_pump_rate_uL_s > 0 (set by _compute_auto_settings).

        flow_rate here is pump-mm per XY-mm (dimensionless ratio).
        If auto_pump_rate_uL_s is set, it overrides the passed flow_rate.
        """
        if len(path_points) < 2:
            return

        print_speed = max(feedrate_mm_min / 60.0, 0.1)

        # Auto pump rate: derive flow_rate from physics if available
        auto_rate = getattr(settings, 'auto_pump_rate_uL_s', 0.0)
        if auto_rate > 0 and print_speed > 0:
            uL_per_mm_pump = _get_uL_per_mm(settings, pump_id)
            if uL_per_mm_pump > 0:
                # flow_rate (mm-pump / mm-XY) = (uL/s) / (mm/s * uL/mm)
                flow_rate = auto_rate / (print_speed * uL_per_mm_pump)

        # Move to first point
        first_x = well_x + path_points[0][0]
        first_y = well_y + path_points[0][1]
        self._move_xy(first_x, first_y, feedrate_mm_min,
                      segment="print", well=well_name)

        # Print segments: coordinated XY + pump
        for i in range(1, len(path_points)):
            seg_x = well_x + path_points[i][0]
            seg_y = well_y + path_points[i][1]
            dist = _distance_2d(self._x, self._y, seg_x, seg_y)
            if dist < 0.001:
                continue

            duration = dist / print_speed
            n_steps = max(int(duration / self.DT_PRINT), 2)
            pump_delta = dist * flow_rate

            x_start, y_start = self._x, self._y
            p_start = self._pumps[pump_id]

            for j in range(1, n_steps + 1):
                frac = j / n_steps
                self._x = _interp(x_start, seg_x, frac)
                self._y = _interp(y_start, seg_y, frac)
                self._pumps[pump_id] = _interp(p_start, p_start + pump_delta, frac)
                self._t += duration / n_steps
                self._wp(segment="print", well=well_name)

            self._fluid_balance[pump_id] -= abs(pump_delta)

'''
    content = content[:m.start()] + new_method + content[m.end():]
    log_ok("B4: _print_path_coordinated() updated with auto pump rate")
    return content

def patch_trajectory_planner(root: Path):
    path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    content = safe_read(path)
    if not content:
        log_fail("PrintTrajectoryPlanner.py not found")
        return
    orig = content
    content = patch_travel_to_well(content)
    content = patch_lower_to_print(content)
    content = patch_raise_from_print(content)
    content = patch_print_path_coordinated(content)
    if content != orig:
        safe_write(path, content, "PrintTrajectoryPlanner.py")

# ══════════════════════════════════════════════════════════════════════════
# PATCH C1 — print_setup.py: add _compute_auto_settings()
# ══════════════════════════════════════════════════════════════════════════
def patch_compute_auto_settings(root: Path):
    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        log_fail("print_setup.py not found")
        return

    MARKER = "v7.2.6-auto-C1: _compute_auto_settings"
    if MARKER in content:
        log_skip("C1: _compute_auto_settings already added")
        return

    # Find _get_settings method to insert the new method just before it
    m = find_method(content, "_get_settings")
    if not m:
        log_fail("C1: _get_settings not found in print_setup.py")
        return

    new_method = '''    def _compute_auto_settings(self, s, hw_config=None):
        """# v7.2.6-auto-C1: _compute_auto_settings
        Auto-derive all motion parameters from hardware config + calibration.

        Mutates PrintSettings s in-place. Called after _get_settings().

        Derives:
          - travel_z_height      ← calibration safe_z
          - top_z_height         ← calibration top_z
          - fast_z_feedrate_mm_min ← max Z speed
          - entry_z_feedrate_mm_min ← z_feedrate (already set from GUI × 60)
          - service_xy_speed_mm_s  ← stage max XY
          - auto_pump_rate_uL_s    ← extrusion_flow_rate(print_speed, needle, layer)
          - service_pump_rate_uL_s ← GAUGE_MAX_FLOW[gauge]
          - pump_feedrate           ← service_pump_rate converted to mm/min
        """
        # ── 1. Calibration data ──────────────────────────────────────────
        try:
            from SupportClasses.Settings import Settings as _Settings
            _app_settings = getattr(self, '_app_settings', None)
            if _app_settings is None:
                # Try to find from parent chain
                _p = self.parent()
                while _p is not None:
                    if hasattr(_p, 'settings') and hasattr(_p.settings, 'get_section'):
                        _app_settings = _p.settings
                        break
                    _p = _p.parent() if hasattr(_p, 'parent') else None
            if _app_settings:
                cal = _app_settings.get_section("calibration") or {}
                safe_z = cal.get("safe_z")
                top_z  = cal.get("top_z")
                if safe_z is not None and safe_z > 0:
                    s.travel_z_height = float(safe_z)
                if top_z is not None and top_z >= 0:
                    s.top_z_height = float(top_z)
        except Exception as _e:
            import logging as _log
            _log.getLogger(__name__).debug(f"Auto-settings: cal load: {_e}")

        # ── 2. Z speed tiers ─────────────────────────────────────────────
        # fast_z: max Z axis speed — use 2 mm/s = 120 mm/min as safe default
        s.fast_z_feedrate_mm_min = 120.0
        # entry_z: already set by _get_settings (z_feed_spin × 60)
        s.entry_z_feedrate_mm_min = s.z_feedrate

        # ── 3. Service XY speed ──────────────────────────────────────────
        s.service_xy_speed_mm_s = 50.0  # mm/s — max comfortable service speed

        # ── 4. Pump rates from needle + syringe ─────────────────────────
        GAUGE_MAX_FLOW = {
            16: 50.0, 18: 30.0, 20: 15.0, 22: 8.0, 23: 5.0,
            25: 3.0, 27: 1.5, 28: 1.0, 30: 0.5, 32: 0.2,
        }
        gauge = None
        needle = None
        if hw_config is not None:
            needle = getattr(hw_config, 'needle', None)
            if needle:
                gauge = getattr(needle, 'gauge', None)

        # Service pump rate (max safe continuous for gauge)
        if gauge and gauge in GAUGE_MAX_FLOW:
            s.service_pump_rate_uL_s = GAUGE_MAX_FLOW[gauge]
        else:
            s.service_pump_rate_uL_s = 5.0  # conservative default

        # Pump feedrate for service moves: mm/min from service rate
        # Default syringe: 250µL Hamilton = 3.378 µL/mm
        _uL_per_mm = 3.378
        if hw_config is not None:
            try:
                _active = getattr(s, 'active_pump', 'P1') or 'P1'
                _pcfg = hw_config.pumps.get(_active)
                if _pcfg and _pcfg.syringe:
                    _uL_per_mm = _pcfg.syringe.uL_per_mm
            except Exception:
                pass
        s.pump_feedrate = max(s.service_pump_rate_uL_s * 60.0 / _uL_per_mm, 0.5)

        # Print pump rate: derived from extrusion physics
        # flow = print_speed(mm/s) × needle_OD(mm) × layer_height(mm) µL/s
        if needle is not None:
            try:
                from SupportClasses.FlowPhysics import extrusion_flow_rate
                od_mm = getattr(needle, 'od_mm', 0.0)
                if od_mm <= 0:
                    od_um = getattr(needle, 'od_um', 0)
                    od_mm = od_um / 1000.0
                if od_mm > 0:
                    flow = extrusion_flow_rate(
                        s.print_speed_mm_s, needle, s.layer_height)
                    s.auto_pump_rate_uL_s = max(flow, 0.001)
            except Exception as _e:
                import logging as _log
                _log.getLogger(__name__).debug(f"Auto-settings: extrusion calc: {_e}")

    '''
    content = content[:m.start()] + new_method + content[m.start():]
    if safe_write(path, content, "print_setup.py (C1)"):
        log_ok("C1: _compute_auto_settings() added to PrintSetupPage")

# ══════════════════════════════════════════════════════════════════════════
# PATCH C2 — print_setup.py: call _compute_auto_settings in _generate_print
# ══════════════════════════════════════════════════════════════════════════
def patch_generate_print_call(root: Path):
    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        log_fail("print_setup.py not found")
        return

    MARKER = "v7.2.6-auto-C2"
    if MARKER in content:
        log_skip("C2: _compute_auto_settings call already in _generate_print")
        return

    # Find the line: settings = self._get_settings() inside _generate_print
    # Then inject the call right after it
    pat = re.compile(
        r'([ \t]+)(settings\s*=\s*self\._get_settings\(\)[ \t]*\n)',
        re.MULTILINE
    )
    matches = list(pat.finditer(content))
    if not matches:
        log_fail("C2: Cannot find 'settings = self._get_settings()' in print_setup.py")
        return

    # Use the last match (inside _generate_print, not _build_current_job)
    m = matches[-1]
    indent = m.group(1)
    after = m.end()
    inject = (
        f"{indent}# {MARKER}: auto-derive speeds + pump rates from HW + calibration\n"
        f"{indent}_hw_for_auto = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)\n"
        f"{indent}if hasattr(self, '_compute_auto_settings'):\n"
        f"{indent}    self._compute_auto_settings(settings, _hw_for_auto)\n"
    )
    content = content[:after] + inject + content[after:]
    if safe_write(path, content, "print_setup.py (C2)"):
        log_ok("C2: _compute_auto_settings() call injected in _generate_print()")

# ══════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════
def main():
    print(f"\n{CYAN}═══════════════════════════════════════════════════════════════{RESET}")
    print(f"{CYAN}  MEBP v7.2.6 — Auto-Settings & Smart Z Approach{RESET}")
    print(f"{CYAN}═══════════════════════════════════════════════════════════════{RESET}\n")

    try:
        root = find_root()
        print(f"Project root: {root}\n")
    except RuntimeError as e:
        print(f"{RED}ERROR: {e}{RESET}"); sys.exit(1)

    print(f"{CYAN}── A: PrintSettings new fields ──{RESET}")
    patch_print_settings(root)

    print(f"\n{CYAN}── B: PrintTrajectoryPlanner smart Z motion ──{RESET}")
    patch_trajectory_planner(root)

    print(f"\n{CYAN}── C: print_setup.py auto-compute settings ──{RESET}")
    patch_compute_auto_settings(root)
    patch_generate_print_call(root)

    print()
    total = ok_count + skip_count + fail_count
    print(f"{CYAN}══ Summary: {ok_count} OK  {skip_count} SKIP  {fail_count} FAIL  (of {total} steps) ══{RESET}\n")
    if fail_count:
        print(f"{RED}⚠  Failures above — check output. Backups created.{RESET}\n")
        sys.exit(1)
    elif skip_count == total:
        print(f"{YELLOW}All already applied — no changes.{RESET}\n")
    else:
        print(f"{GREEN}Done. Verify:{RESET}")
        print('  python3 -c "import ast; ast.parse(open(\'SupportClasses/PrintManager.py\').read())"')
        print('  python3 -c "import ast; ast.parse(open(\'SupportClasses/PrintTrajectoryPlanner.py\').read())"')
        print('  python3 -c "import ast; ast.parse(open(\'gui/pages/print_setup.py\').read())"')
        print()

if __name__ == "__main__":
    main()
