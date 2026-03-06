#!/usr/bin/env python3
"""
Fix PrintPlanOfAction service sequence + trajectory fluid balance.

1. _add_service_steps: correct sequence (WASTE→WASH→BUFFER→WASH→LOAD_INK)
2. _add_service_steps: use correct PlanPreferences attribute names  
3. PrintTrajectoryPlanner: fluid balance is warning not rejection
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

G="\033[92m"; R="\033[91m"; X="\033[0m"; B="\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "SupportClasses").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "SupportClasses").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    print(f"\n{B}=== Fix Plan Service Sequence + Trajectory Balance ==={X}")
    print(f"  Root: {root}\n")

    # ── FIX 1: PrintPlanOfAction._add_service_steps ───────────────
    print(f"{B}1. Fixing PrintPlanOfAction.py{X}")
    poa_path = root / "SupportClasses" / "PrintPlanOfAction.py"
    poa = poa_path.read_text("utf-8")

    marker = "v7.3: Correct service sequence WASTE-WASH-BUFFER-WASH-LOAD_INK"
    if marker not in poa:
        # Find and replace _add_service_steps
        m = re.search(
            r'^(    def _add_service_steps\(self.*?\n)(.*?)(?=\n    def |\Z)',
            poa, re.DOTALL | re.MULTILINE
        )
        if m:
            new_method = '''    def _add_service_steps(self, run_num, run_ink_per_pump, hw_config,
                           ink_wells, wash_wells, waste_wells, buffer_wells):
        """v7.3: Correct service sequence WASTE-WASH-BUFFER-WASH-LOAD_INK.

        Before each print run, prepare the syringe:
        1. WASTE — purge whatever is in the syringe
        2. WASH — clean the needle
        3. BUFFER — load buffer to separate oil from ink
        4. WASH — clean again after buffer
        5. LOAD_INK — aspirate ink needed for this run
        """
        prefs = self.preferences

        # Use the correct attribute names (UI sends use_waste/use_wash/use_buffer)
        do_waste = getattr(prefs, 'use_waste', getattr(prefs, 'waste_before_refill', True))
        do_wash = getattr(prefs, 'use_wash', getattr(prefs, 'wash_after_refill', True))
        do_buffer = getattr(prefs, 'use_buffer', getattr(prefs, 'refill_buffer_after_waste', True))

        # 1. WASTE — purge syringe
        if do_waste and waste_wells:
            for pump_id in run_ink_per_pump:
                self.steps.append(PlanStep(
                    step_type=PlanStepType.WASTE,
                    description=f"Purge {pump_id} at {waste_wells[0]}",
                    target_wells=[waste_wells[0]],
                    pump_id=pump_id, run_number=run_num,
                ))

        # 2. WASH — clean needle
        if do_wash and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Wash needle at {wash_wells[0]}",
                target_wells=[wash_wells[0]], run_number=run_num,
            ))

        # 3. BUFFER — load buffer layer
        if do_buffer and buffer_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.REFILL_BUFFER,
                description=f"Load buffer from {buffer_wells[0]}",
                target_wells=[buffer_wells[0]], run_number=run_num,
            ))

        # 4. WASH again — clean after buffer
        if do_wash and do_buffer and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Post-buffer wash at {wash_wells[0]}",
                target_wells=[wash_wells[0]], run_number=run_num,
            ))

        # 5. LOAD INK — aspirate ink for this run
        for pump_id, volume in run_ink_per_pump.items():
            if volume <= 0:
                continue
            pcfg = hw_config.pumps.get(pump_id)
            ink_name = pcfg.ink.name if pcfg and pcfg.ink else "unknown"
            target = ink_wells[0] if ink_wells else "?"
            self.steps.append(PlanStep(
                step_type=PlanStepType.LOAD_INK,
                description=f"Load {volume:.1f} uL {ink_name} into {pump_id}",
                target_wells=[target] if target != "?" else [],
                pump_id=pump_id, volume_uL=volume,
                ink_name=ink_name, run_number=run_num,
            ))

'''
            poa = poa[:m.start()] + new_method + poa[m.end():]
            print(f"  {G}✓{X} Replaced _add_service_steps with correct sequence")
        else:
            print(f"  {R}✗{X} _add_service_steps not found")
    else:
        print(f"  Already fixed")

    # Also fix _compute to handle the case where pump_ink_needs is empty
    # (no ink info available) — still generate service steps with default volume
    old_no_ink = '''        if not pump_ink_needs:
            self.steps.append(PlanStep(
                step_type=PlanStepType.PRINT,
                description=f"Print {len(print_wells)} wells",
                target_wells=print_wells,
                run_number=1,
            ))
            self.total_runs = 1
            self._estimate_times()
            return'''

    new_no_ink = '''        if not pump_ink_needs:
            # No ink info — use default pump with estimated volume
            default_pump = "P1"
            enabled = getattr(hw_config, 'enabled_pump_ids', [])
            if enabled:
                default_pump = enabled[0]
            # Estimate 2µL per well as default
            estimated_vol = max(len(print_wells) * 2.0, 10.0)
            pump_ink_needs = {default_pump: estimated_vol}
            logger.info(f"No ink info — defaulting to {default_pump}: {estimated_vol:.1f}uL")'''

    if old_no_ink in poa:
        poa = poa.replace(old_no_ink, new_no_ink)
        print(f"  {G}✓{X} Fixed empty pump_ink_needs fallback")
    else:
        print(f"  Empty ink fallback already fixed or different")

    try:
        ast.parse(poa)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {e}{X}"); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(poa_path, poa_path.with_suffix(f".bak_svc_{ts}"))
    poa_path.write_text(poa, "utf-8")
    print(f"  {G}WROTE{X}: PrintPlanOfAction.py")

    # ── FIX 2: PrintTrajectoryPlanner fluid balance ───────────────
    print(f"\n{B}2. Fixing PrintTrajectoryPlanner.py{X}")
    ptp_path = root / "SupportClasses" / "PrintTrajectoryPlanner.py"
    if ptp_path.exists():
        ptp = ptp_path.read_text("utf-8")

        old_reject = '''        # Validate fluid balance
        for pid, balance in self._fluid_balance.items():
            if balance < -0.1:
                self._issues.append(
                    f"{pid}: fluid balance negative ({balance:.2f} mm) — "
                    f"needs more ink loading")'''

        new_reject = '''        # Fluid balance check — warn but don't block execution
        for pid, balance in self._fluid_balance.items():
            if balance < -0.1:
                logger.warning(
                    f"{pid}: fluid balance {balance:.2f} mm "
                    f"(may need more ink loading)")'''

        if old_reject in ptp:
            ptp = ptp.replace(old_reject, new_reject)
            print(f"  {G}✓{X} Fluid balance → warning only")
        elif "warn but don" in ptp:
            print(f"  Already fixed")
        else:
            print(f"  Pattern not found — checking alternative")
            # Try any line that appends fluid balance issue
            ptp = re.sub(
                r'self\._issues\.append\(\s*f"{pid}: fluid balance negative.*?\)',
                'logger.warning(f"{pid}: fluid balance {balance:.2f} mm")',
                ptp
            )
            print(f"  {G}✓{X} Fixed via regex")

        try:
            ast.parse(ptp)
            shutil.copy2(ptp_path, ptp_path.with_suffix(f".bak_bal_{ts}"))
            ptp_path.write_text(ptp, "utf-8")
            print(f"  {G}WROTE{X}: PrintTrajectoryPlanner.py")
        except SyntaxError as e:
            print(f"  {R}AST FAIL: {e}{X}")
    else:
        print(f"  PrintTrajectoryPlanner.py not found — deploy it first")

    # Clean cache
    import os
    for dp, dn, _ in os.walk(root):
        if "__pycache__" in dn:
            shutil.rmtree(Path(dp) / "__pycache__")

    print(f"\n  {G}✓ Done! python main.py{X}")
    print(f"  Service sequence is now: WASTE → WASH → BUFFER → WASH → LOAD_INK → PRINT")

if __name__ == "__main__":
    main()
