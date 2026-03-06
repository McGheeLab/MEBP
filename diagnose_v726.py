#!/usr/bin/env python3
"""
Diagnose the ACTUAL state of all v7.2.6-patched files.
Run this to see exactly what's present and what's missing.
"""

import sys, re
from pathlib import Path

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"; B = "\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def check(label, result):
    print(f"  {'✅' if result else '❌'} {label}")
    return result

def main():
    root = find_root()

    print(f"\n{B}=== v7.2.6 Patch Status Diagnostic ==={X}")
    print(f"  Root: {root}\n")

    # ── Check __pycache__ ─────────────────────────────────────────
    print(f"{B}0. __pycache__ status:{X}")
    for d in ["gui/__pycache__", "gui/pages/__pycache__", "SupportClasses/__pycache__"]:
        p = root / d
        if p.exists():
            pyc_files = list(p.glob("*.pyc"))
            print(f"  {R}⚠ {d}/ exists ({len(pyc_files)} .pyc files) — DELETE THIS{X}")
        else:
            print(f"  {G}✓ {d}/ clean{X}")

    # ── app.py ────────────────────────────────────────────────────
    print(f"\n{B}1. gui/app.py:{X}")
    app = (root / "gui" / "app.py").read_text()

    check("_send_job_to_monitor exists", "def _send_job_to_monitor" in app)
    check("_setup_monitor_visualization exists", "def _setup_monitor_visualization" in app)
    check("_setup_monitor_visualization CALLED in _send_job_to_monitor",
          "_setup_monitor_visualization" in app.split("def _on_monitor_start")[0] if "def _on_monitor_start" in app else False)
    check("setup_plate called", "setup_plate(" in app)
    check("_on_monitor_start has load_job", "load_job" in app)
    check("_wire has QueuedConnection", "QueuedConnection" in app)

    # Show actual _send_job_to_monitor
    m = re.search(r'(    def _send_job_to_monitor\(self.*?\n)(.*?)(?=\n    def )', app, re.DOTALL)
    if m:
        print(f"\n  {B}Actual _send_job_to_monitor body:{X}")
        for line in m.group(0).split("\n")[:20]:
            print(f"    {line}")
        if len(m.group(0).split("\n")) > 20:
            print(f"    ... ({len(m.group(0).split(chr(10)))} total lines)")
    else:
        print(f"  {R}_send_job_to_monitor not found!{X}")

    # ── print_monitor.py ──────────────────────────────────────────
    print(f"\n{B}2. gui/pages/print_monitor.py:{X}")
    mon = (root / "gui" / "pages" / "print_monitor.py").read_text()

    check("PrintMonitorPage class exists", "class PrintMonitorPage" in mon)
    check("setup_plate method exists", "def setup_plate" in mon)
    check("mini_plate attribute exists", "self.mini_plate" in mon)
    check("trajectory_view attribute exists", "self.trajectory_view" in mon)
    check("syringe_panel attribute exists", "self.syringe_panel" in mon)
    check("on_status_update exists", "def on_status_update" in mon)
    check("_get_controller exists", "def _get_controller" in mon)

    # Check _get_controller is inside PrintMonitorPage
    import ast
    tree = ast.parse(mon)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "PrintMonitorPage":
            methods = [n.name for n in node.body if isinstance(n, ast.FunctionDef)]
            check("_get_controller INSIDE PrintMonitorPage class", "_get_controller" in methods)
            check("on_status_update INSIDE PrintMonitorPage class", "on_status_update" in methods)
            check("setup_plate INSIDE PrintMonitorPage class", "setup_plate" in methods)
            check("receive_job INSIDE PrintMonitorPage class", "receive_job" in methods)
            break

    # ── print_setup.py ────────────────────────────────────────────
    print(f"\n{B}3. gui/pages/print_setup.py:{X}")
    ps = (root / "gui" / "pages" / "print_setup.py").read_text()

    check("_build_current_job has v7.2.6 marker",
          "v7.2.6: Fixed job builder" in ps)
    check("_get_path_from_objects exists",
          "def _get_path_from_objects" in ps)
    check("_generate_print has v7.2.6 marker",
          "v7.2.6: Store generated job" in ps)
    check("_send_to_monitor has v7.2.6 marker",
          "v7.2.6: Prefer pre-generated job" in ps)
    check("_generated_job attribute",
          "_generated_job" in ps)

    # ── PrintPlanOfAction.py ──────────────────────────────────────
    print(f"\n{B}4. SupportClasses/PrintPlanOfAction.py:{X}")
    poa = (root / "SupportClasses" / "PrintPlanOfAction.py").read_text()

    check("plan_to_commands exists", "def plan_to_commands" in poa)
    check("_find_service_well exists", "def _find_service_well" in poa)

    # ── Summary ───────────────────────────────────────────────────
    print(f"\n{B}5. Action items:{X}")
    
    has_pycache = any((root / d).exists() for d in [
        "gui/__pycache__", "gui/pages/__pycache__", "SupportClasses/__pycache__"])
    
    if has_pycache:
        print(f"  {R}→ DELETE __pycache__ directories:{X}")
        print(f"    rm -rf gui/__pycache__ gui/pages/__pycache__ SupportClasses/__pycache__")

    if "_setup_monitor_visualization" not in app:
        print(f"  {R}→ Visualization patch NOT applied to app.py — re-run fix_monitor_visualization.py{X}")
    
    if "v7.2.6: Fixed job builder" not in ps:
        print(f"  {R}→ Session 1 NOT applied to print_setup.py — re-run patch_v726_session1{X}")

    if not has_pycache and "_setup_monitor_visualization" in app and "v7.2.6: Fixed job builder" in ps:
        print(f"  {G}All patches appear applied and __pycache__ is clean.{X}")
        print(f"  If visualization still doesn't work, the issue is in")
        print(f"  the monitor widgets themselves (mini_plate/trajectory_view/syringe_panel).")
        print(f"  Run: python main.py and check console for 'Monitor plate initialized' log line.")


if __name__ == "__main__":
    main()
