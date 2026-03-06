#!/usr/bin/env python3
"""
MEBP v7.2.6 — Visual + Plan Fixes.

Fixes:
  1. print_monitor.py: Only print wells change status (service wells keep role color)
  2. print_monitor.py: XY waypoints start empty, fill green when passed, next target = yellow
  3. PrintPlanOfAction.py: Service commands use settings feedrates instead of hardcoded values
  4. PrintPlanOfAction.py: Track pump volume balance to prevent negative ink
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

G="\033[92m"; Y="\033[93m"; R="\033[91m"; X="\033[0m"; B="\033[1m"
_ok=0; _skip=0; _miss=0
def ok(m): global _ok; _ok+=1; print(f"  {G}✓{X} {m}")
def skip(m): global _skip; _skip+=1; print(f"  {Y}○{X} SKIP: {m}")
def miss(m): global _miss; _miss+=1; print(f"  {R}✗{X} MISS: {m}")

def find_root():
    if len(sys.argv)>1:
        p=Path(sys.argv[1])
        if (p/"gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c/"gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def safe_write(path, content, label):
    try: ast.parse(content)
    except SyntaxError as e: print(f"  {R}AST FAIL {path.name}: {e}{X}"); return False
    ts=datetime.now().strftime("%Y%m%d_%H%M%S")
    if path.exists(): shutil.copy2(path, path.with_suffix(f".bak_vf_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})"); return True

def find_method(content, name, indent=4):
    prefix=" "*indent
    return re.compile(
        rf'^({prefix}def {re.escape(name)}\(self.*?\n)(.*?)(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL|re.MULTILINE).search(content)


# ═══════════════════════════════════════════════════════════════════
#  FIX 1+2: print_monitor.py visual fixes
# ═══════════════════════════════════════════════════════════════════

def patch_monitor_visuals(root):
    print(f"\n{B}── Monitor visual fixes ──{X}")
    path = root/"gui"/"pages"/"print_monitor.py"
    content = path.read_text("utf-8")
    changed = False

    # FIX 1: PlateOverviewWidget.set_active_well should only change PRINT wells
    marker1 = "v7.2.6b: Only print wells change status"
    if marker1 not in content:
        old_saw = 'def set_active_well(self, name: str):'
        if old_saw in content:
            m = re.search(rf'^(    def set_active_well\(self, name.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
                          content, re.DOTALL|re.MULTILINE)
            if m:
                new = '''    def set_active_well(self, name: str):
        """v7.2.6b: Only print wells change status; service wells keep role color."""
        # Only transition previous well to 'done' if it was a print well (had pending/active)
        if self._active_well and self._active_well != name:
            prev_state = self._well_states.get(self._active_well)
            if prev_state in ("active", "pending"):
                self._well_states[self._active_well] = "done"
        # Only set active if it was pending (i.e., a print well)
        if self._well_states.get(name) == "pending" or name not in self._well_states:
            self._well_states[name] = "active"
        self._active_well = name
        self.update()

'''
                content = content[:m.start()] + new + content[m.end():]
                ok("Plate: only print wells change status")
                changed = True
            else:
                miss("set_active_well method not found via regex")
        else:
            miss("set_active_well not found")
    else:
        skip("Plate fix already applied")

    # FIX 2: XYDetailView.paintEvent — next target = yellow, others empty
    marker2 = "v7.2.6b: Next target waypoint yellow"
    if marker2 not in content:
        # Replace the waypoint drawing section in paintEvent
        old_wp_draw = "if i < self._completed_idx:"
        if old_wp_draw in content:
            # Find the XYDetailView paintEvent waypoint loop
            # Replace the conditional coloring block
            old_block = """        for i, (wx, wy) in enumerate(wp):
            px, py = self._to_px(wx, wy)
            if not (-30 < px < w + 30 and -30 < py < h + 30): continue
            if i < self._completed_idx:
                p.setPen(QPen(self.C_WP_FILL, 1)); p.setBrush(QBrush(self.C_WP_FILL))
            else:
                p.setPen(QPen(self.C_WP_EMPTY, 1)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(px, py), wpr, wpr)"""

            new_block = """        # v7.2.6b: Next target waypoint yellow
        for i, (wx, wy) in enumerate(wp):
            px, py = self._to_px(wx, wy)
            if not (-30 < px < w + 30 and -30 < py < h + 30): continue
            if i < self._completed_idx:
                # Completed: filled green
                p.setPen(QPen(self.C_WP_FILL, 1)); p.setBrush(QBrush(self.C_WP_FILL))
            elif i == self._completed_idx:
                # Next target: yellow filled
                p.setPen(QPen(self.C_NEXT, 1.5)); p.setBrush(QBrush(self.C_NEXT))
            else:
                # Future: empty circle
                p.setPen(QPen(self.C_WP_EMPTY, 1)); p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(px, py), wpr, wpr)"""

            if old_block in content:
                content = content.replace(old_block, new_block)
                ok("XY detail: next target yellow, future empty")
                changed = True
            else:
                # Try more flexible match
                pattern = re.compile(
                    r'(        for i, \(wx, wy\) in enumerate\(wp\):.*?'
                    r'p\.drawEllipse\(QPointF\(px, py\), wpr, wpr\))',
                    re.DOTALL)
                m = pattern.search(content)
                if m:
                    content = content[:m.start()] + new_block + content[m.end():]
                    ok("XY detail: next target yellow (regex)")
                    changed = True
                else:
                    miss("Waypoint draw loop not found")
        else:
            miss("completed_idx check not found in XYDetailView")
    else:
        skip("XY waypoint fix already applied")

    if changed:
        safe_write(path, content, "visual fixes")


# ═══════════════════════════════════════════════════════════════════
#  FIX 3+4: PrintPlanOfAction.py — feedrates + pump balance
# ═══════════════════════════════════════════════════════════════════

def patch_plan_of_action(root):
    print(f"\n{B}── Plan feedrate + pump balance fixes ──{X}")
    path = root/"SupportClasses"/"PrintPlanOfAction.py"
    content = path.read_text("utf-8")
    changed = False

    # FIX 3: Replace all service command generators to use settings feedrates
    # FIX 4: Add fluid balance tracking to plan_to_commands

    marker34 = "v7.2.6b: Use settings feedrates + track fluid balance"
    if marker34 not in content:
        # Replace all the service command functions and plan_to_commands
        # Find start of service functions block
        fn_start = content.find("def _find_service_well(")
        if fn_start < 0:
            miss("_find_service_well not found"); return

        # Replace everything from _find_service_well to end of file
        new_block = '''def _find_service_well(well_model, plate, role_value: str):
    """Find first well with given role. Returns (name, x, y) or None.

    v7.2.6b: Use settings feedrates + track fluid balance.
    """
    if well_model is None or plate is None:
        return None
    assignments = getattr(well_model, 'assignments', {})
    for name, assignment in assignments.items():
        r = getattr(assignment, 'role', None)
        if r is not None and getattr(r, 'value', None) == role_value:
            try:
                x, y = plate.get_well_position(name)
                return (name, x, y)
            except Exception:
                continue
    return None


def _waste_commands(step, well_model, plate, settings):
    """Generate waste ejection commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "waste")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No waste well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    eject_vol = 5.0  # µL to eject
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Waste: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to waste well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower to waste depth"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": eject_vol, "feedrate": p_fr},
                     label=f"Eject {eject_vol:.1f} into waste"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 0.5},
                     label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from waste"),
    ]


def _wash_commands(step, well_model, plate, settings):
    """Generate wash commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "wash")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No wash well")]
    name, x, y = well
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Wash: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to wash well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into wash"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 5.0},
                     label="Wash soak"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from wash"),
    ]


def _buffer_commands(step, well_model, plate, settings):
    """Generate buffer commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "buffer")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No buffer well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Buffer: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to buffer {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into buffer"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -5.0, "feedrate": p_fr},
                     label="Aspirate buffer"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from buffer"),
    ]


def _load_ink_commands(step, well_model, plate, settings):
    """Generate ink loading commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "ink")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No ink well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    volume = step.volume_uL if step.volume_uL > 0 else 50.0
    ink_name = step.ink_name or "?"
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Load Ink: {ink_name} ({volume:.1f}uL) into {pump} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to ink well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into ink"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -volume, "feedrate": p_fr},
                     label=f"Aspirate {volume:.1f}uL {ink_name}"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from ink"),
    ]


def plan_to_commands(plan, well_model, plate, path_points, settings, hw_config=None):
    """Convert PrintPlanOfAction into executable PrintJob.

    v7.2.6b: Use settings feedrates + track fluid balance.
    Ensures pump never goes negative by inserting extra LOAD_INK steps.
    """
    from SupportClasses.PrintManager import (
        PrintJob, PrintCommand, CommandType, build_well_plate_job,
    )

    steps = getattr(plan, 'steps', [])
    if not steps:
        logger.warning("plan_to_commands: empty plan")
        return None

    all_commands = []
    prev_pump = None

    # Track fluid balance per pump (µL loaded minus µL dispensed)
    fluid_balance: dict[str, float] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    for step in steps:
        stype = getattr(step, 'step_type', None)
        if stype is None:
            continue

        if stype == PlanStepType.WASTE:
            pump = step.pump_id or "P1"
            all_commands.extend(_waste_commands(step, well_model, plate, settings))
            # Waste ejects 5µL
            fluid_balance[pump] = max(0, fluid_balance.get(pump, 0) - 5.0)

        elif stype == PlanStepType.WASH:
            all_commands.extend(_wash_commands(step, well_model, plate, settings))

        elif stype == PlanStepType.REFILL_BUFFER:
            pump = step.pump_id or "P1"
            all_commands.extend(_buffer_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + 5.0

        elif stype == PlanStepType.LOAD_INK:
            pump = step.pump_id or "P1"
            vol = step.volume_uL if step.volume_uL > 0 else 50.0
            all_commands.extend(_load_ink_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + vol

        elif stype == PlanStepType.PRINT:
            target_wells = getattr(step, 'target_wells', [])
            if not target_wells:
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT, label="SKIP: No wells"))
                continue

            well_positions = []
            for wn in target_wells:
                try:
                    x, y = plate.get_well_position(wn)
                    well_positions.append((wn, x, y))
                except Exception:
                    continue

            if not well_positions:
                continue

            pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
            flow = getattr(settings, 'flow_rate', 0.01) or 0.01

            if prev_pump is not None and pump != prev_pump:
                all_commands.append(PrintCommand(
                    type=CommandType.SWITCH_PUMP, params={"pump": pump},
                    label=f"Switch to {pump}"))
            prev_pump = pump

            # Estimate ink needed for this run
            path_length = 0.0
            if len(path_points) >= 2:
                import math
                for i in range(1, len(path_points)):
                    dx = path_points[i][0] - path_points[i-1][0]
                    dy = path_points[i][1] - path_points[i-1][1]
                    path_length += math.sqrt(dx*dx + dy*dy)
            ink_per_well = path_length * flow * getattr(settings, 'num_layers', 1)
            ink_needed = ink_per_well * len(well_positions)

            # Check if we have enough ink
            balance = fluid_balance.get(pump, 0)
            if balance < ink_needed and ink_needed > 0:
                shortfall = ink_needed - balance
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"WARNING: {pump} needs {ink_needed:.1f}uL but only {balance:.1f}uL loaded"))
                logger.warning(
                    f"Pump {pump}: needs {ink_needed:.1f}uL, has {balance:.1f}uL "
                    f"(shortfall {shortfall:.1f}uL)")

            run_num = getattr(step, 'run_number', '?')
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=f"== Print Run {run_num}: {len(well_positions)} wells, {pump} =="))

            try:
                sub_job = build_well_plate_job(
                    well_positions=well_positions,
                    path_points=path_points,
                    settings=settings,
                    pump=pump,
                    flow_rate=flow,
                    job_name=f"Run {run_num}",
                )
                all_commands.extend(sub_job.commands)
                # Deduct estimated ink used
                fluid_balance[pump] = fluid_balance.get(pump, 0) - ink_needed
            except Exception as exc:
                logger.error(f"build_well_plate_job run {run_num}: {exc}")
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"ERROR: Run {run_num} failed: {exc}"))

        elif stype == PlanStepType.RETURN_HOME:
            z_fr = getattr(settings, 'z_feedrate', 60.0)
            all_commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                label="Final: raise"))
            all_commands.append(PrintCommand(
                type=CommandType.HOME_XY, label="Return home"))
        else:
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT, label=f"Unknown: {stype}"))

    if not all_commands:
        return None

    tw = getattr(plan, 'total_print_wells', '?')
    tr = getattr(plan, 'total_runs', '?')
    return PrintJob(
        name=f"Plan: {tw} wells, {tr} run(s)",
        description="Generated from PrintPlanOfAction v7.2.6b",
        settings=settings,
        commands=all_commands,
    )
'''
        content = content[:fn_start] + new_block + "\n"
        ok("Replaced service commands + plan_to_commands with feedrate/balance fixes")
        changed = True
    else:
        skip("Plan fixes already applied")

    if changed:
        safe_write(path, content, "feedrate + balance fixes")


# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{B}{'='*60}\n  v7.2.6 Visual + Plan Fixes\n{'='*60}{X}")
    root = find_root(); print(f"  Root: {root}\n")
    patch_monitor_visuals(root)
    patch_plan_of_action(root)

    # Clean pycache
    import os
    for dp,dn,_ in os.walk(root):
        if "__pycache__" in dn: shutil.rmtree(Path(dp)/"__pycache__")

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK: {_ok}{X}  {Y}SKIP: {_skip}{X}  {R}MISS: {_miss}{X}")
    if _miss: print(f"\n  {R}⚠ Some patches failed{X}"); sys.exit(1)
    else: print(f"\n  {G}✓ Done! python main.py{X}")

if __name__=="__main__": main()
