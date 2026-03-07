#!/usr/bin/env python3
"""
MEBP v7.2.6 — Fix for patch_auto_settings C1 failure.

C1 failed because new_method ended with "\\n    " (trailing 4 spaces,
no newline), which concatenated directly to "    def _get_settings"
producing "        def _get_settings" (doubly-indented nested function).

This script adds _compute_auto_settings() correctly.
C2 already applied successfully, so only C1 is needed here.
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
    shutil.copy2(path, path.with_suffix(f".bak_v726_c1fix_{ts}"))
    path.write_text(content, encoding="utf-8")
    return True

def find_method(content: str, name: str):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self[^)]*\)[^:]*:[ \t]*\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

def patch_c1(root: Path):
    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        log_fail("print_setup.py not found"); return

    MARKER = "v7.2.6-auto-C1: _compute_auto_settings"
    if MARKER in content:
        log_skip("C1: _compute_auto_settings already present"); return

    # Find insertion point: just before _get_settings method
    m = find_method(content, "_get_settings")
    if not m:
        log_fail("C1: _get_settings not found"); return

    # Build new method as a list of lines to avoid any trailing-space issues
    lines = [
        "    def _compute_auto_settings(self, s, hw_config=None):",
        '        """# v7.2.6-auto-C1: _compute_auto_settings',
        "        Auto-derive all motion parameters from hardware config + calibration.",
        "",
        "        Mutates PrintSettings s in-place. Called after _get_settings().",
        "",
        "        Derives:",
        "          - travel_z_height         from calibration safe_z",
        "          - top_z_height            from calibration top_z",
        "          - fast_z_feedrate_mm_min  max Z speed (above plate surface)",
        "          - entry_z_feedrate_mm_min z_feedrate (from GUI spinbox * 60)",
        "          - service_xy_speed_mm_s   fast XY for wash/waste/ink moves",
        "          - auto_pump_rate_uL_s     extrusion physics: v*OD*layer",
        "          - service_pump_rate_uL_s  GAUGE_MAX_FLOW[gauge]",
        "          - pump_feedrate           service rate in mm/min",
        '        """',
        "        # ── 1. Calibration data ──────────────────────────────────────",
        "        try:",
        "            _app_settings = getattr(self, '_app_settings', None)",
        "            if _app_settings is None:",
        "                _p = self.parent() if hasattr(self, 'parent') else None",
        "                while _p is not None:",
        "                    if hasattr(_p, 'settings') and hasattr(_p.settings, 'get_section'):",
        "                        _app_settings = _p.settings",
        "                        break",
        "                    _p = _p.parent() if hasattr(_p, 'parent') else None",
        "            if _app_settings:",
        "                cal = _app_settings.get_section('calibration') or {}",
        "                safe_z = cal.get('safe_z')",
        "                top_z  = cal.get('top_z')",
        "                if safe_z is not None and safe_z > 0:",
        "                    s.travel_z_height = float(safe_z)",
        "                if top_z is not None and top_z >= 0:",
        "                    s.top_z_height = float(top_z)",
        "        except Exception as _e:",
        "            import logging as _log",
        "            _log.getLogger(__name__).debug(f'Auto-settings: cal load: {_e}')",
        "",
        "        # ── 2. Z speed tiers ─────────────────────────────────────────",
        "        # fast_z: safe max Z travel speed (above plate surface)",
        "        s.fast_z_feedrate_mm_min = 120.0  # 2 mm/s",
        "        # entry_z: from GUI z_feed_spin (already stored as mm/min via prior patch)",
        "        s.entry_z_feedrate_mm_min = s.z_feedrate",
        "",
        "        # ── 3. Service XY speed ──────────────────────────────────────",
        "        s.service_xy_speed_mm_s = 50.0",
        "",
        "        # ── 4. Pump rates from needle + syringe ─────────────────────",
        "        GAUGE_MAX_FLOW = {",
        "            16: 50.0, 18: 30.0, 20: 15.0, 22: 8.0, 23: 5.0,",
        "            25: 3.0, 27: 1.5, 28: 1.0, 30: 0.5, 32: 0.2,",
        "        }",
        "        gauge = None",
        "        needle = None",
        "        if hw_config is not None:",
        "            needle = getattr(hw_config, 'needle', None)",
        "            if needle:",
        "                gauge = getattr(needle, 'gauge', None)",
        "",
        "        # Service pump rate (max safe for gauge)",
        "        if gauge and gauge in GAUGE_MAX_FLOW:",
        "            s.service_pump_rate_uL_s = GAUGE_MAX_FLOW[gauge]",
        "        else:",
        "            s.service_pump_rate_uL_s = 5.0",
        "",
        "        # Pump feedrate for service: convert uL/s to mm/min",
        "        _uL_per_mm = 3.378  # 250uL Hamilton default",
        "        if hw_config is not None:",
        "            try:",
        "                _active = getattr(s, 'active_pump', 'P1') or 'P1'",
        "                _pcfg = hw_config.pumps.get(_active)",
        "                if _pcfg and _pcfg.syringe:",
        "                    _uL_per_mm = _pcfg.syringe.uL_per_mm",
        "            except Exception:",
        "                pass",
        "        s.pump_feedrate = max(s.service_pump_rate_uL_s * 60.0 / _uL_per_mm, 0.5)",
        "",
        "        # Print pump rate: extrusion_flow_rate(v, needle_OD, layer_height)",
        "        if needle is not None:",
        "            try:",
        "                from SupportClasses.FlowPhysics import extrusion_flow_rate",
        "                od_mm = getattr(needle, 'od_mm', 0.0)",
        "                if od_mm <= 0:",
        "                    od_um = getattr(needle, 'od_um', 0)",
        "                    od_mm = od_um / 1000.0",
        "                if od_mm > 0:",
        "                    flow = extrusion_flow_rate(",
        "                        s.print_speed_mm_s, needle, s.layer_height)",
        "                    s.auto_pump_rate_uL_s = max(flow, 0.001)",
        "            except Exception as _e:",
        "                import logging as _log",
        "                _log.getLogger(__name__).debug(",
        "                    f'Auto-settings: extrusion calc: {_e}')",
        "",
    ]
    # Join with newlines — no trailing spaces, ends with exactly one blank line + newline
    new_method = "\n".join(lines) + "\n"

    # Verify the new method parses cleanly by itself (wrap in a dummy class)
    test_src = "class _T:\n" + new_method + "    pass\n"
    try:
        ast.parse(test_src)
    except SyntaxError as e:
        log_fail(f"C1: new method self-AST failed: {e}")
        return

    # Insert before _get_settings
    content = content[:m.start()] + new_method + content[m.start():]

    if safe_write(path, content, "print_setup.py (C1-fix)"):
        log_ok("C1: _compute_auto_settings() added to PrintSetupPage")

def main():
    print(f"\n{CYAN}═══════════════════════════════════════════════════════{RESET}")
    print(f"{CYAN}  MEBP v7.2.6 — C1 Fix: _compute_auto_settings(){RESET}")
    print(f"{CYAN}═══════════════════════════════════════════════════════{RESET}\n")
    try:
        root = find_root()
        print(f"Project root: {root}\n")
    except RuntimeError as e:
        print(f"{RED}ERROR: {e}{RESET}"); sys.exit(1)

    patch_c1(root)
    print()
    total = ok_count + skip_count + fail_count
    print(f"{CYAN}══ Summary: {ok_count} OK  {skip_count} SKIP  {fail_count} FAIL ══{RESET}\n")
    if fail_count:
        print(f"{RED}⚠  Fix failed — check output above.{RESET}\n"); sys.exit(1)
    elif skip_count == total:
        print(f"{YELLOW}Already applied.{RESET}\n")
    else:
        print(f"{GREEN}Done. Verify:{RESET}")
        print('  python3 -c "import ast; ast.parse(open(\'gui/pages/print_setup.py\').read())"')
        print()

if __name__ == "__main__":
    main()
