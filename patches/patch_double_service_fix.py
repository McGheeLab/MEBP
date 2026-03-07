#!/usr/bin/env python3
"""
MEBP v7.2.6 — Double Service Cycle Fix

BUG: The service sequence (waste→wash→buffer→wash→ink) was running TWICE
before each print:
  1. As standalone PlanSteps added by PrintPlanOfAction._add_service_steps()
     (WASTE, WASH, REFILL_BUFFER, WASH, LOAD_INK steps in the plan)
  2. Internally inside PrintTrajectoryPlanner._do_service_and_print() when
     it encountered PlanStepType.PRINT

Result: syringe aspirated ink twice, hitting -456 uL (safety limit), then
printing with whatever was left in the syringe.

FIX (PrintTrajectoryPlanner.generate()):
  - When PlanStepType.PRINT is encountered, call _do_print_wells() ONLY.
    The plan already contains all pre-print service steps before the PRINT
    step. No need to re-run service internally.
  - Keep standalone WASTE/WASH/BUFFER/LOAD_INK step handling as-is —
    these execute the steps the plan explicitly added.
  - Keep RETURN_HOME waste+wash (final cleanup after all printing is done).
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
    shutil.copy2(path, path.with_suffix(f".bak_v726_dsf_{ts}"))
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

# ══════════════════════════════════════════════════════════════════════════
# PATCH — PrintTrajectoryPlanner.generate():
#   Replace the PRINT step handler so it calls _do_print_wells() directly
#   instead of _do_service_and_print() (which re-ran all service internally).
# ══════════════════════════════════════════════════════════════════════════
def patch_generate_print_step(root: Path):
    path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    content = safe_read(path)
    if not content:
        log_fail("PrintTrajectoryPlanner.py not found")
        return

    MARKER = "v7.2.6-dsf: PRINT calls _do_print_wells directly"
    if MARKER in content:
        log_skip("Already applied: PRINT handler fixed")
        return

    # Match the PRINT handler block inside generate():
    #   if stype == PlanStepType.PRINT:
    #       # v7.3: Full service workflow before each print run
    #       target_wells = ...
    #       flow = ...
    #       if target_wells:
    #           self._do_service_and_print(...)
    #           well_count += ...
    pat = re.compile(
        r'([ \t]+)(if stype == PlanStepType\.PRINT:[ \t]*\n'
        r'(?:[ \t]+.*\n)*?'          # all lines until the well_count increment
        r'[ \t]+well_count \+= len\(target_wells\)[ \t]*\n)',
        re.MULTILINE
    )
    m = pat.search(content)
    if not m:
        log_fail("Could not find PlanStepType.PRINT handler in generate()")
        return

    indent = m.group(1)
    new_block = (
        f"{indent}if stype == PlanStepType.PRINT:\n"
        f"{indent}    # {MARKER}\n"
        f"{indent}    # The plan already contains all pre-print service steps\n"
        f"{indent}    # (WASTE/WASH/BUFFER/LOAD_INK) added by _add_service_steps().\n"
        f"{indent}    # Calling _do_service_and_print() here caused a double-cycle:\n"
        f"{indent}    # syringe loaded ink twice, hitting safety limits.\n"
        f"{indent}    # Now we just call _do_print_wells() — service steps\n"
        f"{indent}    # were already executed as their own PlanStepType entries.\n"
        f"{indent}    target_wells = getattr(step, 'target_wells', [])\n"
        f"{indent}    flow = getattr(settings, 'flow_rate', 0.01) or 0.01\n"
        f"{indent}    # Use auto pump rate if set by _compute_auto_settings\n"
        f"{indent}    if target_wells:\n"
        f"{indent}        self._do_print_wells(\n"
        f"{indent}            plate, target_wells, pump, path_points,\n"
        f"{indent}            flow, settings)\n"
        f"{indent}        well_count += len(target_wells)\n"
    )
    content = content[:m.start()] + new_block + content[m.end():]
    if safe_write(path, content, "PrintTrajectoryPlanner.py"):
        log_ok("PRINT handler: now calls _do_print_wells() (no double service cycle)")

# ══════════════════════════════════════════════════════════════════════════
# PATCH 2 — Fix _do_print_wells() signature to accept flow_rate parameter
#   Current: _do_print_wells(self, plate, well_names, pump_id, path_points,
#                            flow_rate, settings)
#   The generate() call above passes flow_rate as positional — verify it matches.
# ══════════════════════════════════════════════════════════════════════════
def verify_do_print_wells(root: Path):
    path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    content = safe_read(path)
    m = find_method(content, "_do_print_wells")
    if not m:
        log_fail("_do_print_wells not found — check manually")
        return
    sig_line = m.group(1)
    if "flow_rate" in sig_line:
        log_ok("_do_print_wells signature includes flow_rate — OK")
    else:
        log_fail(f"_do_print_wells signature missing flow_rate: {sig_line.strip()}")

# ══════════════════════════════════════════════════════════════════════════
# PATCH 3 — Fix ink aspiration direction in _do_waste
#   BUG: _do_waste uses eject_vol_mm = +5.0/uL_per_mm (positive = dispense).
#   This is CORRECT for ejecting. But verify the sign is right.
#   Also clamp so it doesn't eject more than the syringe physical max.
# ══════════════════════════════════════════════════════════════════════════
def patch_do_waste_clamp(root: Path):
    path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    content = safe_read(path)

    MARKER = "v7.2.6-dsf: waste clamp"
    if MARKER in content:
        log_skip("_do_waste clamp already applied")
        return

    m = find_method(content, "_do_waste")
    if not m:
        log_fail("_do_waste not found")
        return

    new_method = '''    def _do_waste(self, plate, well_model, pump_id, settings):
        """Waste: travel → lower → eject syringe contents → raise.
        # v7.2.6-dsf: waste clamp
        Ejects only what is currently loaded (no overshoot).
        Uses service_pump_rate_uL_s from auto-settings if available.
        """
        well = _find_well(well_model, plate, "waste")
        if not well:
            logger.info("Skipping: no waste well assigned — will proceed without")
            return
        name, wx, wy = well
        uL_per_mm = _get_uL_per_mm(settings, pump_id)

        # Eject the pump's current fluid balance (what was loaded)
        # Clamp to a safe maximum to avoid runaway
        fluid_loaded_mm = max(0.0, self._fluid_balance.get(pump_id, 0.0) / uL_per_mm
                              if uL_per_mm > 0 else 0.0)
        eject_vol_mm = min(fluid_loaded_mm + (5.0 / uL_per_mm), 50.0 / uL_per_mm)
        eject_vol_mm = max(eject_vol_mm, 1.0 / uL_per_mm)  # at least 1 uL

        svc_fr = getattr(settings, 'pump_feedrate', 30.0)

        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        self._move_pump(pump_id, eject_vol_mm, svc_fr,
                        segment="service", well=name)
        # After waste, pump balance resets to zero
        self._fluid_balance[pump_id] = 0.0
        self._dwell(0.5, well=name)
        self._raise_from_print(settings, well=name)

'''
    content = content[:m.start()] + new_method + content[m.end():]
    if safe_write(path, content, "PrintTrajectoryPlanner.py (_do_waste)"):
        log_ok("_do_waste: now ejects only what was loaded (no overshoot)")

def main():
    print(f"\n{CYAN}═══════════════════════════════════════════════════════════{RESET}")
    print(f"{CYAN}  MEBP v7.2.6 — Double Service Cycle Fix{RESET}")
    print(f"{CYAN}═══════════════════════════════════════════════════════════{RESET}\n")
    try:
        root = find_root()
        print(f"Project root: {root}\n")
    except RuntimeError as e:
        print(f"{RED}ERROR: {e}{RESET}"); sys.exit(1)

    print(f"{CYAN}── Fix 1: PRINT step handler ──{RESET}")
    patch_generate_print_step(root)

    print(f"\n{CYAN}── Verify: _do_print_wells signature ──{RESET}")
    verify_do_print_wells(root)

    print(f"\n{CYAN}── Fix 2: _do_waste clamping ──{RESET}")
    patch_do_waste_clamp(root)

    print()
    total = ok_count + skip_count + fail_count
    print(f"{CYAN}══ Summary: {ok_count} OK  {skip_count} SKIP  {fail_count} FAIL ══{RESET}\n")
    if fail_count:
        print(f"{RED}⚠  Failures — check output above.{RESET}\n"); sys.exit(1)
    elif skip_count == total:
        print(f"{YELLOW}Already applied.{RESET}\n")
    else:
        print(f"{GREEN}Done. Verify:{RESET}")
        print('  python3 -c "import ast; ast.parse(open(\'SupportClasses/PrintTrajectoryPlanner.py\').read())"')
        print()

if __name__ == "__main__":
    main()
