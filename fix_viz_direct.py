#!/usr/bin/env python3
"""
Direct fix: Replace _send_job_to_monitor in app.py with visualization wiring,
and nuke all __pycache__ directories.
"""

import ast, sys, shutil, re, os
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"; B = "\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    print(f"\n{B}=== Direct Fix: app.py visualization + cache cleanup ==={X}")
    print(f"  Root: {root}\n")

    # ── Step 1: Nuke all __pycache__ ──────────────────────────────
    print(f"{B}Step 1: Cleaning __pycache__{X}")
    count = 0
    for dirpath, dirnames, filenames in os.walk(root):
        if "__pycache__" in dirnames:
            cache_dir = Path(dirpath) / "__pycache__"
            shutil.rmtree(cache_dir)
            print(f"  {G}Deleted{X}: {cache_dir.relative_to(root)}")
            count += 1
    print(f"  Cleaned {count} __pycache__ directories\n")

    # ── Step 2: Fix app.py ────────────────────────────────────────
    print(f"{B}Step 2: Patching gui/app.py{X}")
    app_path = root / "gui" / "app.py"
    content = app_path.read_text(encoding="utf-8")

    # Find the EXACT old _send_job_to_monitor text
    OLD_BODY = '''    def _send_job_to_monitor(self, job):
        """
        Receive a PrintJob from PrintSetupPage, forward it to
        PrintMonitorPage's job queue, and switch to the monitor page.
        """
        monitor_page = self._page_widgets[5]

        if hasattr(monitor_page, 'receive_job'):
            monitor_page.receive_job(job)

        # Auto-switch to Print Monitor page
        self._switch_page(5)'''

    NEW_BODY = '''    def _send_job_to_monitor(self, job):
        """
        v7.2.6: Forward job to monitor WITH visualization setup.

        Initializes plate overview, trajectory view, and syringe displays
        before the job is received by the monitor page.
        """
        monitor_page = self._page_widgets[5]
        setup_page = self._page_widgets[4]

        # ── Initialize monitor visualization ──────────────────────
        try:
            self._setup_monitor_visualization(setup_page, monitor_page, job)
        except Exception as exc:
            logger.error(f"Monitor viz setup error: {exc}", exc_info=True)

        if hasattr(monitor_page, 'receive_job'):
            monitor_page.receive_job(job)

        # Auto-switch to Print Monitor page
        self._switch_page(5)'''

    HELPER_METHOD = '''
    def _setup_monitor_visualization(self, setup_page, monitor_page, job):
        """v7.2.6: Feed plate/trajectory/syringe data to monitor widgets."""
        try:
            # ── 1. Plate Overview ─────────────────────────────────
            plate = None
            well_roles = {}
            print_wells = []

            tab_wells = getattr(setup_page, 'tab_wells', None)
            model = getattr(tab_wells, '_model', None) if tab_wells else None

            if model is not None:
                plate = getattr(model, 'plate', None)
                assignments = getattr(model, 'assignments', {})

                for name, assignment in assignments.items():
                    role = getattr(assignment, 'role', None)
                    if role is not None:
                        well_roles[name] = role
                        if getattr(role, 'value', '') == 'print':
                            print_wells.append(name)

            if plate is not None and hasattr(monitor_page, 'setup_plate'):
                well_diam = getattr(plate, 'well_diameter', 0.0)
                monitor_page.setup_plate(
                    plate=plate,
                    well_roles=well_roles if well_roles else None,
                    print_wells=print_wells if print_wells else None,
                    well_diameter_mm=well_diam,
                )
                logger.info(
                    f"Monitor plate: {len(print_wells)} print wells, "
                    f"diam={well_diam:.1f}mm")

            # ── 2. Trajectory View — path segments from job ───────
            if (hasattr(monitor_page, 'trajectory_view')
                    and hasattr(job, 'get_path_segments')):
                try:
                    segments = job.get_path_segments()
                    tv = monitor_page.trajectory_view
                    if segments:
                        if hasattr(tv, 'set_path_segments'):
                            tv.set_path_segments(segments)
                        elif hasattr(tv, 'load_path_segments'):
                            tv.load_path_segments(segments)
                        logger.info(f"Monitor trajectory: {len(segments)} segments")
                except Exception as exc:
                    logger.debug(f"Trajectory load skipped: {exc}")

            # ── 3. Syringe Display from HardwareConfig ────────────
            hw = getattr(self, '_hardware_config', None)
            if hw is not None and hasattr(monitor_page, 'syringe_panel'):
                try:
                    sp = monitor_page.syringe_panel
                    # Try different syringe panel APIs
                    workspace = getattr(setup_page, '_workspace', None)
                    if workspace:
                        pump_loadouts = getattr(workspace, 'pumps', None)
                        if pump_loadouts and hasattr(monitor_page, 'update_syringe_state'):
                            active = getattr(
                                getattr(job, 'settings', None), 'active_pump', 'P1')
                            monitor_page.update_syringe_state(pump_loadouts, active)
                except Exception as exc:
                    logger.debug(f"Syringe setup skipped: {exc}")

            # ── 4. Needle info label ──────────────────────────────
            if hw and hasattr(monitor_page, 'needle_label'):
                try:
                    needle = getattr(hw, 'needle', None)
                    if needle:
                        g = getattr(needle, 'gauge', '?')
                        l = getattr(needle, 'length_inches', '?')
                        d = getattr(needle, 'id_um', 0)
                        monitor_page.needle_label.setText(
                            f"Needle: {g}G x {l}\\"  ID: {d:.0f} um")
                except Exception:
                    pass

            # ── 5. Push workspace to monitor ──────────────────────
            workspace = getattr(setup_page, '_workspace', None)
            if workspace and hasattr(monitor_page, '_workspace'):
                monitor_page._workspace = workspace
                if hasattr(monitor_page, '_update_needle_info'):
                    try:
                        monitor_page._update_needle_info()
                    except Exception:
                        pass

        except Exception as exc:
            logger.error(f"Monitor viz setup failed: {exc}", exc_info=True)
'''

    if "_setup_monitor_visualization" in content:
        print(f"  {Y}○ SKIP: _setup_monitor_visualization already in app.py{X}")
    elif OLD_BODY in content:
        # Exact match — replace directly
        content = content.replace(OLD_BODY, NEW_BODY)

        # Insert helper after the new _send_job_to_monitor
        insert_pos = content.find(NEW_BODY) + len(NEW_BODY)
        content = content[:insert_pos] + HELPER_METHOD + content[insert_pos:]

        # Verify AST
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {R}AST FAIL: {e}{X}")
            sys.exit(1)

        # Backup + write
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(app_path, app_path.with_suffix(f".bak_vizfix_{ts}"))
        app_path.write_text(content, encoding="utf-8")
        print(f"  {G}✓ Replaced _send_job_to_monitor + added _setup_monitor_visualization{X}")
    else:
        # Exact match failed — try regex
        print(f"  {Y}Exact match failed, trying regex...{X}")
        m = re.search(
            r'^(    def _send_job_to_monitor\(self, job\):\n)'
            r'(.*?)'
            r'(?=\n    def |\nclass |\Z)',
            content, re.DOTALL | re.MULTILINE
        )
        if m:
            content = content[:m.start()] + NEW_BODY + HELPER_METHOD + content[m.end():]
            try:
                ast.parse(content)
            except SyntaxError as e:
                print(f"  {R}AST FAIL: {e}{X}")
                sys.exit(1)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(app_path, app_path.with_suffix(f".bak_vizfix_{ts}"))
            app_path.write_text(content, encoding="utf-8")
            print(f"  {G}✓ Replaced via regex + added _setup_monitor_visualization{X}")
        else:
            print(f"  {R}✗ Cannot find _send_job_to_monitor in app.py!{X}")
            sys.exit(1)

    # ── Step 3: Verify ────────────────────────────────────────────
    print(f"\n{B}Step 3: Verification{X}")
    final = app_path.read_text()
    ok = True
    for check, label in [
        ("_setup_monitor_visualization" in final, "_setup_monitor_visualization exists"),
        ("setup_plate" in final, "setup_plate called"),
        ("trajectory_view" in final, "trajectory_view referenced"),
        ("syringe_panel" in final, "syringe_panel referenced"),
    ]:
        status = f"{G}✓{X}" if check else f"{R}✗{X}"
        print(f"  {status} {label}")
        if not check: ok = False

    if ok:
        print(f"\n  {G}✓ All good! Launch with: python main.py{X}")
    else:
        print(f"\n  {R}Something went wrong — check output above{X}")


if __name__ == "__main__":
    main()
