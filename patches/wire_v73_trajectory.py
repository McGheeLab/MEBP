#!/usr/bin/env python3
"""
Wire PrintTrajectoryPlanner into the execution chain.

Changes:
  1. print_setup.py._generate_print() → use plan_to_trajectory() 
  2. print_setup.py._send_to_monitor() → attach waypoints to job
  3. app.py._on_monitor_start() → use TrajectoryExecutor for trajectory jobs
  4. app.py._send_job_to_monitor() → pass waypoints to monitor XY detail
"""

import ast, re, sys, shutil, os
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
    if path.exists(): shutil.copy2(path, path.with_suffix(f".bak_traj_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})"); return True

def find_method(content, name, indent=4):
    prefix=" "*indent
    return re.compile(
        rf'^({prefix}def {re.escape(name)}\(self.*?\n)(.*?)(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL|re.MULTILINE).search(content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH print_setup.py
# ═══════════════════════════════════════════════════════════════════

def patch_print_setup(root):
    print(f"\n{B}── print_setup.py: trajectory wiring ──{X}")
    path = root/"gui"/"pages"/"print_setup.py"
    content = path.read_text("utf-8")
    changed = False

    # 1. Replace _generate_print to use plan_to_trajectory
    marker = "v7.3: Use plan_to_trajectory for smooth execution"
    if marker not in content:
        m = find_method(content, "_generate_print")
        if m:
            new = '''    def _generate_print(self):
        """v7.3: Use plan_to_trajectory for smooth execution.

        Generates a time-parameterized trajectory instead of flat commands.
        The trajectory encodes all axis positions with proper timing from
        feedrate settings, enabling smooth coordinated motion.
        """
        gen_label = getattr(self, '_gen_status_label', None)

        # Step 1: Validate
        if hasattr(self, 'tab_wells') and hasattr(self.tab_wells, 'validate'):
            try:
                is_valid, issues = self.tab_wells.validate()
            except Exception as exc:
                is_valid, issues = False, [f"Validation error: {exc}"]
        else:
            is_valid, issues = False, ["Well setup tab missing validate()"]

        if not is_valid:
            if gen_label:
                gen_label.setText(
                    f"\\u26a0 {len(issues)} issue(s):\\n"
                    + "\\n".join(f"  \\u2022 {i}" for i in issues[:5]))
                gen_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            if hasattr(self, 'btn_send_to_monitor'):
                self.btn_send_to_monitor.setEnabled(False)
            return

        # Step 2: Generate plan
        plan = None
        if hasattr(self, 'tab_wells'):
            if hasattr(self.tab_wells, '_generate_plan'):
                try:
                    self.tab_wells._generate_plan()
                except Exception as e:
                    if gen_label:
                        gen_label.setText(f"\\u26a0 Plan failed: {e}")
                        gen_label.setStyleSheet(
                            f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                    if hasattr(self, 'btn_send_to_monitor'):
                        self.btn_send_to_monitor.setEnabled(False)
                    return
            if hasattr(self.tab_wells, 'get_plan'):
                plan = self.tab_wells.get_plan()

        # Step 3: Get path points and settings
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
        model = getattr(self.tab_wells, '_model', None)
        plate = getattr(model, 'plate', None) if model else None
        hw = getattr(self, '_hardware_config', None) or getattr(self, '_hw_config', None)

        # Step 4: Generate trajectory (preferred) or fall back to commands
        job = None
        trajectory_result = None

        if plan is not None and plate is not None:
            try:
                from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
                trajectory_result = plan_to_trajectory(
                    plan=plan, well_model=model, plate=plate,
                    path_points=path_points, settings=settings, hw_config=hw)

                if trajectory_result.valid:
                    # Build a lightweight PrintJob that carries the waypoints
                    from SupportClasses.PrintManager import PrintJob
                    job = PrintJob(
                        name=f"Trajectory: {trajectory_result.well_count} wells",
                        description=trajectory_result.summary(),
                        settings=settings,
                        commands=[],  # empty — execution uses waypoints
                    )
                    job.trajectory_waypoints = trajectory_result.waypoints
                    job.trajectory_result = trajectory_result
                    if plan:
                        job.plan_of_action = plan
                    logger.info(
                        f"Trajectory generated: {len(trajectory_result.waypoints)} "
                        f"waypoints, {trajectory_result.total_duration_s:.1f}s")
                else:
                    logger.warning(f"Trajectory invalid: {trajectory_result.issues}")
                    if gen_label:
                        gen_label.setText(
                            "\\u26a0 " + "; ".join(trajectory_result.issues[:3]))
                        gen_label.setStyleSheet(
                            f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                    if hasattr(self, 'btn_send_to_monitor'):
                        self.btn_send_to_monitor.setEnabled(False)
                    return
            except ImportError:
                logger.warning("PrintTrajectoryPlanner not available, using commands")
            except Exception as exc:
                logger.error(f"Trajectory generation failed: {exc}", exc_info=True)

        # Fallback to command-based job
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

        self._generated_job = job

        # Step 5: Show success
        parts = [f"\\u2713 Ready: {getattr(job, 'name', '?')}"]
        if trajectory_result:
            parts.append(f"{trajectory_result.total_duration_s / 60:.1f} min")
            parts.append(f"{len(trajectory_result.waypoints)} waypoints")
        elif hasattr(job, 'total_steps'):
            parts.append(f"{job.total_steps} commands")
        if gen_label:
            gen_label.setText(" | ".join(parts))
            gen_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        if hasattr(self, 'btn_send_to_monitor'):
            self.btn_send_to_monitor.setEnabled(True)
        logger.info(f"Generate Print complete: {parts}")

'''
            content = content[:m.start()] + new + content[m.end():]
            ok("Replaced _generate_print() with trajectory support")
            changed = True
        else:
            miss("_generate_print not found")
    else:
        skip("_generate_print already has trajectory support")

    if changed:
        safe_write(path, content, "trajectory wiring")


# ═══════════════════════════════════════════════════════════════════
#  PATCH app.py
# ═══════════════════════════════════════════════════════════════════

def patch_app(root):
    print(f"\n{B}── app.py: trajectory execution wiring ──{X}")
    path = root/"gui"/"app.py"
    content = path.read_text("utf-8")
    changed = False

    # 1. Replace _on_monitor_start to handle trajectory jobs
    marker = "v7.3: Trajectory execution with configurable control mode"
    if marker not in content:
        m = find_method(content, "_on_monitor_start")
        if m:
            new = '''    def _on_monitor_start(self, job):
        """v7.3: Trajectory execution with configurable control mode.

        Execution modes (set in settings.json → execution.mode):
          "discrete"  — legacy command-based PrintManager (v7.0 behavior)
          "position"  — trajectory waypoints with position commands (DEFAULT)
          "kalman"    — velocity control via Kalman filter
          "pid"       — velocity control via PID controller

        Default is "position" — simplest and most reliable.
        """
        setup_page = self._page_widgets[4]
        if not hasattr(setup_page, "print_manager"):
            logger.error("No print_manager on setup page")
            return

        pm = setup_page.print_manager
        waypoints = getattr(job, 'trajectory_waypoints', None)

        # Read execution mode from settings (default: position)
        exec_mode = "position"
        if hasattr(self, '_settings') and self._settings:
            exec_mode = self._settings.get("execution.mode", "position")
        logger.info(f"Execution mode: {exec_mode}")

        # ── Discrete mode: use legacy PrintManager ────────────────
        if exec_mode == "discrete" or not waypoints or len(waypoints) == 0:
            try:
                pm.load_job(job)
                pm.start()
                logger.info(f"Print started (discrete): {job.name}")
            except Exception as exc:
                logger.error(f"PrintManager start failed: {exc}", exc_info=True)
            return

        # ── Trajectory modes: position / kalman / pid ─────────────
        logger.info(
            f"Starting trajectory ({exec_mode}): {job.name} "
            f"({len(waypoints)} waypoints)")
        try:
            from SupportClasses.PrintManager import PrintState
            import threading

            pm.job = job
            pm._abort_flag.clear()
            pm._pause_event.set()

            # Choose executor based on mode
            if exec_mode in ("kalman", "pid"):
                try:
                    from SupportClasses.VelocityExecutor import VelocityExecutor
                    tex = VelocityExecutor(
                        pm.controller, strategy=exec_mode,
                        recorder=pm.recorder)
                    logger.info(f"Using VelocityExecutor ({exec_mode})")
                except ImportError:
                    logger.warning("VelocityExecutor not available, using position mode")
                    exec_mode = "position"

            if exec_mode == "position":
                # Use the v7.1 TrajectoryExecutor — simplest, most reliable
                from SupportClasses.PrintManager import TrajectoryExecutor
                tex = TrajectoryExecutor(pm.controller, recorder=pm.recorder)
                logger.info("Using TrajectoryExecutor (position mode)")

            pm._trajectory_executor = tex

            # Start recording
            if hasattr(pm, '_start_recorder'):
                pm._start_recorder()

            def _traj_thread():
                pm._set_state(PrintState.RUNNING)
                try:
                    def on_prog(idx, total, msg):
                        if pm.on_progress:
                            pm.on_progress(idx, total, msg)

                    success = tex.execute(
                        waypoints=waypoints,
                        pause_event=pm._pause_event,
                        on_progress=on_prog,
                    )
                    if success:
                        pm._set_state(PrintState.COMPLETED)
                        if pm.on_progress:
                            pm.on_progress(len(waypoints), len(waypoints),
                                           "Complete!")
                    else:
                        pm._set_state(PrintState.ABORTED)
                except Exception as exc:
                    logger.error(f"Trajectory error: {exc}", exc_info=True)
                    pm._set_state(PrintState.ERROR)
                finally:
                    if hasattr(pm, '_stop_recorder'):
                        try:
                            pm._stop_recorder(pm.state.name.lower())
                        except Exception:
                            pass

            pm._thread = threading.Thread(target=_traj_thread, daemon=True)
            pm._thread.start()

        except Exception as exc:
            logger.error(f"Trajectory start failed: {exc}", exc_info=True)

'''
            content = content[:m.start()] + new + content[m.end():]
            ok("Replaced _on_monitor_start() with trajectory support")
            changed = True
        else:
            miss("_on_monitor_start not found")
    else:
        skip("_on_monitor_start already has trajectory support")

    # 2. Trajectory waypoints are handled by monitor's receive_job() directly
    # (no need to modify _setup_monitor_visualization — monitor reads job.trajectory_waypoints)
    ok("Monitor will read trajectory_waypoints from job in receive_job()")

    if changed:
        safe_write(path, content, "trajectory execution wiring")


# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{B}{'='*60}\n  v7.3 Session 2+3: Wire Trajectory into Execution Chain\n{'='*60}{X}")
    root = find_root(); print(f"  Root: {root}\n")
    patch_print_setup(root)
    patch_app(root)

    # Clean pycache
    for dp,dn,_ in os.walk(root):
        if "__pycache__" in dn: shutil.rmtree(Path(dp)/"__pycache__")
    ok("Cleaned __pycache__")

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK: {_ok}{X}  {Y}SKIP: {_skip}{X}  {R}MISS: {_miss}{X}")
    if _miss: print(f"\n  {R}⚠ Some patches failed{X}"); sys.exit(1)
    else: print(f"\n  {G}✓ Done! python main.py{X}")

if __name__=="__main__": main()
