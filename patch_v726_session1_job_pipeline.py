#!/usr/bin/env python3
"""
MEBP v7.2.6 Session 1 — Fix the Job Building Pipeline.

Patches:
  1. print_setup.py: Rewrite _build_current_job() with correct API
  2. print_setup.py: Add _get_path_from_objects() geometry extraction
  3. print_setup.py: Add _single_object_to_points() converter
  4. print_setup.py: Update _generate_print() to store _generated_job
  5. print_setup.py: Update _send_to_monitor() to prefer _generated_job
  6. print_setup.py: Ensure _generated_job initialised in _setup_ui
  7. PrintPlanOfAction.py: Add plan_to_commands() bridge + service helpers

Follows PATCHING_BEST_PRACTICES.md:
  - Method-level regex replacement (no exact-string anchors)
  - Idempotency guards with v7.2.6 markers
  - AST verification before write
  - Timestamped backups
  - Safe widget access via getattr/hasattr
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ═══════════════════════════════════════════════════════════════════
#  Terminal colours
# ═══════════════════════════════════════════════════════════════════
G = "\033[92m"  # green
Y = "\033[93m"  # yellow
R = "\033[91m"  # red
X = "\033[0m"   # reset
B = "\033[1m"   # bold

# ═══════════════════════════════════════════════════════════════════
#  Counters
# ═══════════════════════════════════════════════════════════════════
_ok = 0
_skip = 0
_miss = 0


def ok(msg):
    global _ok; _ok += 1; print(f"  {G}✓{X} {msg}")

def skip(msg):
    global _skip; _skip += 1; print(f"  {Y}○{X} SKIP: {msg}")

def miss(msg):
    global _miss; _miss += 1; print(f"  {R}✗{X} MISS: {msg}")


# ═══════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════

def find_root(start: Path = None) -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    if start is None:
        # Check command-line arg first
        if len(sys.argv) > 1:
            p = Path(sys.argv[1])
            if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
                return p
        # Check cwd
        for candidate in [Path.cwd(), Path(__file__).resolve().parent.parent]:
            if (candidate / "SupportClasses").is_dir() and (candidate / "gui").is_dir():
                return candidate
    else:
        if (start / "SupportClasses").is_dir() and (start / "gui").is_dir():
            return start
    print(f"{R}ERROR: Cannot find MEBP project root.{X}")
    print(f"  Usage: python {Path(__file__).name} [/path/to/MEBP]")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → backup → write."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL on {path.name} ({label}): {e}{X}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v726s1_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})")
    return True


def find_method(content: str, name: str, indent: int = 4):
    """Find a method's start and end using regex."""
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 1: print_setup.py
# ═══════════════════════════════════════════════════════════════════

def patch_print_setup(root: Path):
    print(f"\n{B}── Patching gui/pages/print_setup.py ──{X}")
    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        miss("print_setup.py not found")
        return
    changed = False

    # ── 1A. Ensure _generated_job attribute exists in _setup_ui ──
    marker_1a = "v7.2.6: pre-generated job"
    if marker_1a not in content:
        # Find where _setup_ui adds tabs and wire signals
        # Insert after tabs are added, before signal wiring
        m = re.search(
            r'(self\.tabs\.addTab\(self\.tab_wells.*?\n)',
            content
        )
        if m:
            inject = f"\n        self._generated_job = None  # {marker_1a}\n"
            # Only add if not already present with slightly different wording
            if "_generated_job" not in content[:m.end() + 200]:
                content = content[:m.end()] + inject + content[m.end():]
                ok("Added _generated_job attribute init")
                changed = True
            else:
                skip("_generated_job already initialised")
        else:
            miss("Could not find tab_wells addTab to insert _generated_job")
    else:
        skip("_generated_job marker already present")

    # ── 1B. Replace _build_current_job() ─────────────────────────
    marker_1b = "v7.2.6: Fixed job builder with correct build_well_plate_job API"
    if marker_1b not in content:
        m = find_method(content, "_build_current_job")
        if m:
            new_method = '''    def _build_current_job(self):
        """Build a PrintJob from current well setup + print objects.

        v7.2.6: Fixed job builder with correct build_well_plate_job API.
        Extracts geometry from Tab 2 objects, well positions from Tab 3 model.
        """
        # 1. Get well model from Tab 3
        if not (hasattr(self, 'tab_wells') and hasattr(self.tab_wells, '_model')
                and self.tab_wells._model):
            logger.warning("No well model available for job building")
            return None

        model = self.tab_wells._model
        plate = getattr(model, 'plate', None)
        if plate is None:
            logger.warning("No plate geometry available")
            return None

        # 2. Get print wells
        print_wells = []
        for name, assignment in model.assignments.items():
            role = getattr(assignment, 'role', None)
            if role is not None and getattr(role, 'value', None) == 'print':
                print_wells.append(name)

        if not print_wells:
            logger.warning("No print wells assigned")
            return None

        # 3. Build well_positions: list of (name, x_mm, y_mm)
        well_positions = []
        for name in print_wells:
            try:
                x, y = plate.get_well_position(name)
            except Exception:
                x, y = 0.0, 0.0
            well_positions.append((name, x, y))

        # 4. Get path points from Tab 2 objects
        path_points = self._get_path_from_objects()
        if not path_points:
            # Fallback: default meander pattern sized to well
            try:
                from SupportClasses.WellPlate import generate_meander_path
                well_diam = getattr(plate, 'well_diameter', 6.0)
                if well_diam <= 0:
                    well_diam = 6.0
                fill_frac = 0.7
                path_points = generate_meander_path(
                    well_diam * fill_frac,
                    well_diam * fill_frac,
                    0.5,
                )
            except Exception:
                path_points = [(0.0, 0.0)]
            logger.info(f"Using fallback meander pattern ({len(path_points)} pts)")

        # 5. Get settings from context panel
        settings = self._get_settings()

        # 6. Get pump/flow info
        pump = getattr(settings, 'active_pump', 'P1') or 'P1'
        flow_rate = getattr(settings, 'flow_rate', 0.01) or 0.01

        # 7. Build multi-material params from HardwareConfig
        mm_params = {}
        hw = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)
        if hw and hasattr(hw, 'pumps'):
            active_pumps = [
                pid for pid, pcfg in hw.pumps.items()
                if getattr(pcfg, 'enabled', False)
            ]
            if len(active_pumps) > 1:
                mm_params['pump_sequence'] = active_pumps

        # 8. Build job with correct v7.0 API
        try:
            from SupportClasses.PrintManager import build_well_plate_job
            job = build_well_plate_job(
                well_positions=well_positions,
                path_points=path_points,
                settings=settings,
                pump=pump,
                flow_rate=flow_rate,
                job_name=f"Well Plate {getattr(plate, 'format', '?')}-well",
                **mm_params,
            )
            logger.info(
                f"Built job: {job.name}, {job.total_steps} commands, "
                f"{len(well_positions)} wells, {len(path_points)} path pts")
            return job
        except Exception as exc:
            logger.error(f"build_well_plate_job failed: {exc}", exc_info=True)
            return None

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Replaced _build_current_job()")
            changed = True
        else:
            miss("_build_current_job() method not found")
    else:
        skip("_build_current_job() already patched")

    # ── 1C. Add _get_path_from_objects() + helpers ───────────────
    marker_1c = "v7.2.6: Bridge between PrintObjects tab geometry and job builder"
    if marker_1c not in content:
        # Find insertion point: right before _generate_print or _send_to_monitor
        insert_before = None
        for method_name in ['_generate_print', '_send_to_monitor', '_export_gcode']:
            m = find_method(content, method_name)
            if m:
                insert_before = m.start()
                break

        if insert_before is None:
            # Fallback: insert before the last method in the class
            miss("Cannot find insertion point for path extraction methods")
        else:
            new_methods = '''    def _get_path_from_objects(self):
        """Extract path points from Tab 2 print objects.

        v7.2.6: Bridge between PrintObjects tab geometry and job builder.
        Returns list of (x, y) tuples or empty list.
        """
        if not hasattr(self, 'tab_objects'):
            return []

        tab = self.tab_objects

        # Primary: read from _objects list (list of dicts)
        objects_list = getattr(tab, '_objects', None)
        if objects_list and isinstance(objects_list, list) and len(objects_list) > 0:
            all_points = []
            for obj in objects_list:
                if not isinstance(obj, dict):
                    continue
                pts = self._single_object_to_points(obj)
                pos = obj.get('position', (0, 0, 0))
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
                else:
                    all_points.extend(pts)
            if all_points:
                logger.info(f"Extracted {len(all_points)} path points from {len(objects_list)} objects")
                return all_points

        # Secondary: try PrintFileData objects dict
        current_file = getattr(tab, '_current_file', None)
        if current_file and hasattr(current_file, 'objects') and current_file.objects:
            all_points = []
            for obj_name, obj_data in current_file.objects.items():
                if isinstance(obj_data, dict):
                    pts = self._single_object_to_points(obj_data)
                    pos = obj_data.get('position', [0, 0, 0])
                    if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                        all_points.extend([(x + pos[0], y + pos[1]) for x, y in pts])
                    else:
                        all_points.extend(pts)
            if all_points:
                logger.info(f"Extracted {len(all_points)} path points from file data")
                return all_points

        return []

    def _single_object_to_points(self, obj_data) -> list:
        """Convert a single print object dict to path point list.

        v7.2.6: Supports line, meander, spiral, grid, and raw points.
        """
        if isinstance(obj_data, dict):
            obj_type = obj_data.get('object_type', '')
            params = obj_data.get('params', {})
        else:
            obj_type = getattr(obj_data, 'object_type', '')
            params = getattr(obj_data, 'params', {})

        if not isinstance(params, dict):
            params = {}

        try:
            from SupportClasses.WellPlate import (
                generate_line_path, generate_meander_path,
                generate_spiral_path, generate_grid_path,
            )
        except ImportError:
            logger.warning("WellPlate path generators not available")
            return []

        try:
            if obj_type == 'line':
                return generate_line_path(
                    params.get('length', 5.0),
                    params.get('angle', 0.0),
                )
            elif obj_type == 'meander':
                return generate_meander_path(
                    params.get('width', 5.0),
                    params.get('height', 5.0),
                    params.get('spacing', 0.5),
                )
            elif obj_type == 'spiral':
                return generate_spiral_path(
                    params.get('radius', 3.0),
                    params.get('spacing', 0.5),
                )
            elif obj_type == 'grid':
                return generate_grid_path(
                    params.get('width', 5.0),
                    params.get('height', 5.0),
                    params.get('spacing_x', params.get('spacing', 1.0)),
                    params.get('spacing_y', params.get('spacing', 1.0)),
                )
            elif obj_type == 'point' or obj_type == 'dot':
                return [(0.0, 0.0)]
            elif 'points' in params:
                raw = params['points']
                return [(p[0], p[1]) for p in raw if len(p) >= 2]
            else:
                logger.debug(f"Unknown object type '{obj_type}', using center point")
                return [(0.0, 0.0)]
        except Exception as exc:
            logger.warning(f"Failed to generate path for {obj_type}: {exc}")
            return [(0.0, 0.0)]

'''
            content = content[:insert_before] + new_methods + content[insert_before:]
            ok("Added _get_path_from_objects() + _single_object_to_points()")
            changed = True
    else:
        skip("Path extraction methods already present")

    # ── 1D. Update _generate_print() to store _generated_job ─────
    marker_1d = "v7.2.6: Store generated job for _send_to_monitor"
    if marker_1d not in content:
        m = find_method(content, "_generate_print")
        if m:
            new_method = '''    def _generate_print(self):
        """v7.2.6: Store generated job for _send_to_monitor.

        Validate well setup, generate execution plan, and build the
        complete PrintJob. Stores as self._generated_job.
        """
        # Step 1: Validate
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'validate'):
            try:
                is_valid, issues = self.tab_wells.validate()
            except Exception as exc:
                is_valid, issues = False, [f"Validation error: {exc}"]
        else:
            is_valid, issues = False, ["Well setup tab missing validate() method"]

        gen_label = getattr(self, '_gen_status_label', None)

        if not is_valid:
            text = (f"\\u26a0 {len(issues)} issue(s):\\n"
                    + "\\n".join(f"  \\u2022 {i}" for i in issues[:5]))
            if gen_label:
                gen_label.setText(text)
                gen_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            if hasattr(self, 'btn_send_to_monitor'):
                self.btn_send_to_monitor.setEnabled(False)
            logger.warning(f"Generate Print: {len(issues)} validation issues")
            return

        # Step 2: Generate plan
        plan = None
        if hasattr(self, 'tab_wells'):
            if hasattr(self.tab_wells, '_generate_plan'):
                try:
                    self.tab_wells._generate_plan()
                except Exception as e:
                    if gen_label:
                        gen_label.setText(f"\\u26a0 Plan generation failed: {e}")
                        gen_label.setStyleSheet(
                            f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                    if hasattr(self, 'btn_send_to_monitor'):
                        self.btn_send_to_monitor.setEnabled(False)
                    return
            if hasattr(self.tab_wells, 'get_plan'):
                plan = self.tab_wells.get_plan()

        # Step 3: Build job — try plan-based first, fallback to simple
        job = None
        if plan is not None and hasattr(plan, 'steps') and len(plan.steps) > 0:
            try:
                from SupportClasses.PrintPlanOfAction import plan_to_commands
                path_points = self._get_path_from_objects()
                if not path_points:
                    try:
                        from SupportClasses.WellPlate import generate_meander_path
                        model = self.tab_wells._model
                        plate = getattr(model, 'plate', None)
                        d = getattr(plate, 'well_diameter', 6.0) if plate else 6.0
                        path_points = generate_meander_path(d * 0.7, d * 0.7, 0.5)
                    except Exception:
                        path_points = [(0.0, 0.0)]

                settings = self._get_settings()
                model = self.tab_wells._model
                plate = getattr(model, 'plate', None)
                hw = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)

                job = plan_to_commands(
                    plan=plan,
                    well_model=model,
                    plate=plate,
                    path_points=path_points,
                    settings=settings,
                    hw_config=hw,
                )
                if job:
                    job.plan_of_action = plan
                    logger.info(f"Plan-based job: {job.total_steps} commands")
            except ImportError:
                logger.warning("plan_to_commands not available, falling back to simple build")
            except Exception as exc:
                logger.error(f"plan_to_commands failed: {exc}", exc_info=True)

        # Fallback to simple build
        if job is None:
            job = self._build_current_job()

        if job is None:
            if gen_label:
                gen_label.setText("\\u26a0 Could not build print job")
                gen_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            if hasattr(self, 'btn_send_to_monitor'):
                self.btn_send_to_monitor.setEnabled(False)
            return

        # Store for _send_to_monitor
        self._generated_job = job

        # Step 4: Show success
        summary_parts = [f"\\u2713 Job ready: {job.total_steps} commands"]
        if plan:
            summary_parts.append(f"{getattr(plan, 'total_runs', '?')} run(s)")
            est = getattr(plan, 'estimated_total_seconds', 0)
            if est > 0:
                summary_parts.append(f"~{est / 60:.1f} min")
        if gen_label:
            gen_label.setText(" | ".join(summary_parts))
            gen_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        if hasattr(self, 'btn_send_to_monitor'):
            self.btn_send_to_monitor.setEnabled(True)
        logger.info(f"Generate Print complete: {job.total_steps} commands ready")

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Replaced _generate_print()")
            changed = True
        else:
            miss("_generate_print() method not found — may not exist in v7.2.3 base")
    else:
        skip("_generate_print() already patched")

    # ── 1E. Update _send_to_monitor() to prefer _generated_job ───
    marker_1e = "v7.2.6: Prefer pre-generated job"
    if marker_1e not in content:
        m = find_method(content, "_send_to_monitor")
        if m:
            new_method = '''    def _send_to_monitor(self):
        """v7.2.6: Prefer pre-generated job from _generate_print().

        Build a print job and emit job_ready for Print Monitor.
        Validates well setup before sending. Shows issues dialog on failure.
        Includes PrintPlanOfAction in the job when valid.
        """
        # Run validation
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'validate'):
            try:
                is_valid, issues = self.tab_wells.validate()
            except Exception as exc:
                is_valid, issues = False, [f"Validation error: {exc}"]

            if not is_valid:
                try:
                    from PySide6.QtWidgets import QMessageBox
                    msg = QMessageBox(self)
                    msg.setWindowTitle("Setup Validation Failed")
                    msg.setIcon(QMessageBox.Icon.Warning)
                    msg.setText(
                        f"Cannot send to monitor: {len(issues)} issue(s) found.")
                    msg.setDetailedText("\\n".join(f"\\u2022 {i}" for i in issues))
                    msg.exec()
                except Exception:
                    pass
                if hasattr(self, 'status_label'):
                    self.status_label.setText(
                        f"\\u26a0 {len(issues)} validation issue(s)")
                    self.status_label.setStyleSheet(
                        f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                logger.warning(f"Send-to-monitor blocked: {issues}")
                return

        # Use pre-generated job if available, otherwise build fresh
        job = getattr(self, '_generated_job', None) or self._build_current_job()
        if job is None:
            if hasattr(self, 'status_label'):
                self.status_label.setText("\\u26a0 No job \\u2014 configure wells first")
                self.status_label.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 10px;")
            return

        # Attach plan of action to job if available
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'get_plan'):
            plan = self.tab_wells.get_plan()
            if plan is not None:
                job.plan_of_action = plan

        # Show confirmation dialog
        summary_parts = [f"{job.total_steps} commands"]
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, '_model'):
            model = self.tab_wells._model
            print_wells = [
                n for n, a in model.assignments.items()
                if getattr(getattr(a, 'role', None), 'value', None) == 'print'
            ]
            summary_parts.insert(0, f"{len(print_wells)} print wells")
        plan = getattr(job, 'plan_of_action', None)
        if plan:
            summary_parts.append(f"{getattr(plan, 'total_runs', '?')} run(s)")
            est = getattr(plan, 'estimated_total_seconds', 0)
            if est > 0:
                summary_parts.append(f"~{est / 60:.1f} min")

        try:
            from PySide6.QtWidgets import QMessageBox
            confirm = QMessageBox.question(
                self,
                "Send to Monitor",
                f"Send job to Print Monitor?\\n\\n" + "\\n".join(summary_parts),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if confirm != QMessageBox.StandardButton.Yes:
                return
        except Exception:
            pass  # If dialog fails, proceed anyway

        self.job_ready.emit(job)
        self._generated_job = None  # Clear after sending
        if hasattr(self, 'status_label'):
            self.status_label.setText(f"\\u2713 Job sent: {job.name}")
            self.status_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        logger.info(f"Print job sent to Monitor: {job.name}")

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Replaced _send_to_monitor()")
            changed = True
        else:
            miss("_send_to_monitor() method not found")
    else:
        skip("_send_to_monitor() already patched")

    # ── Write ─────────────────────────────────────────────────────
    if changed:
        safe_write(path, content, "Session 1 job pipeline fixes")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  PATCH 2: PrintPlanOfAction.py — Add plan_to_commands() bridge
# ═══════════════════════════════════════════════════════════════════

def patch_plan_of_action(root: Path):
    print(f"\n{B}── Patching SupportClasses/PrintPlanOfAction.py ──{X}")
    path = root / "SupportClasses" / "PrintPlanOfAction.py"
    content = safe_read(path)
    if not content:
        miss("PrintPlanOfAction.py not found")
        return

    changed = False
    marker = "v7.2.6: Bridge between high-level plan and PrintManager commands"

    if marker not in content:
        # Add plan_to_commands and helpers at the END of the file
        bridge_code = '''

# ═══════════════════════════════════════════════════════════════════
# v7.2.6: Plan → PrintCommand Execution Bridge
# ═══════════════════════════════════════════════════════════════════

def _find_service_well(well_model, plate, role_value: str):
    """Find the first well with the given role. Returns (name, x, y) or None.

    v7.2.6: Bridge between high-level plan and PrintManager commands.
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
    """Generate PrintCommands for a waste ejection step."""
    from SupportClasses.PrintManager import PrintCommand, CommandType

    well_info = _find_service_well(well_model, plate, "waste")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No waste well assigned")]
    name, x, y = well_info
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Waste: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to waste well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower to waste depth"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": 5.0, "feedrate": 60},
                     label="Eject waste"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 0.5},
                     label="Settle after waste"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from waste"),
    ]


def _wash_commands(step, well_model, plate, settings):
    """Generate PrintCommands for a wash step."""
    from SupportClasses.PrintManager import PrintCommand, CommandType

    well_info = _find_service_well(well_model, plate, "wash")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No wash well assigned")]
    name, x, y = well_info
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Wash: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to wash well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into wash"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 5.0},
                     label="Wash soak"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from wash"),
    ]


def _buffer_commands(step, well_model, plate, settings):
    """Generate PrintCommands for a buffer refill step."""
    from SupportClasses.PrintManager import PrintCommand, CommandType

    well_info = _find_service_well(well_model, plate, "buffer")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No buffer well assigned")]
    name, x, y = well_info
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Buffer: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to buffer well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into buffer"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -5.0, "feedrate": 30},
                     label="Aspirate buffer"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0},
                     label="Settle after buffer"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from buffer"),
    ]


def _load_ink_commands(step, well_model, plate, settings):
    """Generate PrintCommands for an ink loading step."""
    from SupportClasses.PrintManager import PrintCommand, CommandType

    well_info = _find_service_well(well_model, plate, "ink")
    if not well_info:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No ink well assigned")]
    name, x, y = well_info
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    volume = step.volume_uL if step.volume_uL > 0 else 50.0
    ink_name = step.ink_name or "?"
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Load Ink: {ink_name} ({volume:.1f}uL) into {pump} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to ink well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, label="Lower into ink"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -volume, "feedrate": 30},
                     label=f"Aspirate {volume:.1f}uL {ink_name}"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0},
                     label="Settle after ink load"),
        PrintCommand(type=CommandType.TRAVEL_UP, label="Raise from ink well"),
    ]


def plan_to_commands(
    plan,           # PrintPlanOfAction
    well_model,     # WellSetupModel
    plate,          # WellPlate
    path_points,    # list[(float, float)] - print geometry per well
    settings,       # PrintSettings
    hw_config=None, # HardwareConfig (optional)
):
    """Convert a PrintPlanOfAction into an executable PrintJob.

    v7.2.6: Bridge between high-level plan and PrintManager commands.

    Each PlanStep becomes a sequence of PrintCommands:
        WASTE        -> travel to waste well, lower, eject, raise
        WASH         -> travel to wash well, lower, soak, raise
        REFILL_BUFFER -> travel to buffer well, lower, aspirate, raise
        LOAD_INK     -> travel to ink well, lower, aspirate, raise
        PRINT        -> for each well: travel, lower, print path, raise
        RETURN_HOME  -> home XY
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

    for step in steps:
        step_type = getattr(step, 'step_type', None)
        if step_type is None:
            continue

        if step_type == PlanStepType.WASTE:
            all_commands.extend(_waste_commands(step, well_model, plate, settings))

        elif step_type == PlanStepType.WASH:
            all_commands.extend(_wash_commands(step, well_model, plate, settings))

        elif step_type == PlanStepType.REFILL_BUFFER:
            all_commands.extend(_buffer_commands(step, well_model, plate, settings))

        elif step_type == PlanStepType.LOAD_INK:
            all_commands.extend(_load_ink_commands(step, well_model, plate, settings))

        elif step_type == PlanStepType.PRINT:
            target_wells = getattr(step, 'target_wells', [])
            if not target_wells:
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT, label="SKIP: No wells in print step"))
                continue

            # Build well positions for this run
            well_positions = []
            for well_name in target_wells:
                try:
                    x, y = plate.get_well_position(well_name)
                    well_positions.append((well_name, x, y))
                except Exception:
                    logger.warning(f"Skipping well {well_name}: position lookup failed")
                    continue

            if not well_positions:
                continue

            # Determine pump for this step
            pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
            flow = getattr(settings, 'flow_rate', 0.01) or 0.01

            # Insert SWITCH_PUMP if pump changed
            if prev_pump is not None and pump != prev_pump:
                all_commands.append(PrintCommand(
                    type=CommandType.SWITCH_PUMP,
                    params={"pump": pump},
                    label=f"Switch to {pump}",
                ))
            prev_pump = pump

            # Add run header comment
            run_num = getattr(step, 'run_number', '?')
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=f"== Print Run {run_num}: {len(well_positions)} wells with {pump} ==",
            ))

            # Build sub-job for this run's wells
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
            except Exception as exc:
                logger.error(f"build_well_plate_job for run {run_num} failed: {exc}")
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"ERROR: Failed to build run {run_num}: {exc}",
                ))

        elif step_type == PlanStepType.RETURN_HOME:
            all_commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP, label="Final: raise to travel height"))
            all_commands.append(PrintCommand(
                type=CommandType.HOME_XY, label="Return to home position"))

        else:
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=f"Unknown step type: {step_type}",
            ))

    if not all_commands:
        logger.warning("plan_to_commands produced no commands")
        return None

    total_wells = getattr(plan, 'total_print_wells', '?')
    total_runs = getattr(plan, 'total_runs', '?')
    return PrintJob(
        name=f"Plan: {total_wells} wells, {total_runs} run(s)",
        description="Generated from PrintPlanOfAction v7.2.6",
        settings=settings,
        commands=all_commands,
    )
'''
        content = content.rstrip() + "\n" + bridge_code + "\n"
        ok("Added plan_to_commands() + service step helpers")
        changed = True
    else:
        skip("plan_to_commands() already present")

    if changed:
        safe_write(path, content, "Session 1 plan-to-commands bridge")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{B}{'=' * 60}")
    print(f"  MEBP v7.2.6 Session 1 — Job Building Pipeline Fix")
    print(f"{'=' * 60}{X}\n")

    root = find_root()
    print(f"  Project root: {root}\n")

    patch_print_setup(root)
    patch_plan_of_action(root)

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK:   {_ok}{X}")
    print(f"  {Y}SKIP: {_skip}{X}")
    print(f"  {R}MISS: {_miss}{X}")

    if _miss > 0:
        print(f"\n  {R}⚠ Some patches did not apply. Review output above.{X}")
        sys.exit(1)
    else:
        print(f"\n  {G}✓ Session 1 complete. Ready for Session 2.{X}")


if __name__ == "__main__":
    main()
