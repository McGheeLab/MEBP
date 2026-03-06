#!/usr/bin/env python3
"""
MEBP v7.2.6 — Batch 2 Deployment: Medium File Rewrites

Files modified:
    1. gui/pages/hardware_setup.py  — _restoring guard on config restore
    2. gui/pages/calibration.py     — Defer camera detect to button click
    3. gui/pages/print_setup.py     — Execution pipeline (_generated_job)

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v726/deploy_v726_batch2.py
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; B = "\033[1m"; C = "\033[96m"; X = "\033[0m"
_ok = 0; _skip = 0; _fail = 0
_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

def find_root() -> Path:
    for c in [Path("."), Path(".."), Path(__file__).parent.parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{R}Cannot find project root{X}"); sys.exit(1)

def safe_read(path: Path) -> str:
    if not path.exists():
        print(f"  {R}FILE NOT FOUND: {path}{X}"); return ""
    return path.read_text(encoding="utf-8")

def safe_write(path: Path, content: str, label: str) -> bool:
    global _ok, _fail
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {label} — {e}{X}"); _fail += 1; return False
    shutil.copy2(path, path.with_suffix(f".bak_v726_{_ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})"); _ok += 1; return True

def rc(applied, label):
    global _skip
    if applied:
        print(f"    {G}✓{X} {label}")
    else:
        print(f"    {Y}○{X} {label} (already applied)"); _skip += 1


# ═══════════════════════════════════════════════════════════════════
#  FILE 1: gui/pages/hardware_setup.py — _restoring guard
# ═══════════════════════════════════════════════════════════════════

def fix_hardware_setup(root: Path):
    path = root / "gui" / "pages" / "hardware_setup.py"
    print(f"\n{C}[1/3] {path.name}{X}")
    content = safe_read(path)
    if not content: return

    changed = False

    # 1a. Add _restoring flag to __init__
    if "_restoring" not in content:
        match = re.search(r'(self\._last_valid\s*=\s*False)', content)
        if match:
            content = content[:match.end()] + \
                "\n        self._restoring = False  # v7.2.6: guard for config restore" + \
                content[match.end():]
            changed = True
            rc(True, "Added _restoring flag to __init__")
        else:
            rc(False, "_last_valid not found")
    else:
        rc(False, "_restoring flag already exists")

    # 1b. Guard _on_config_changed
    if "_restoring" in content and "getattr(self, '_restoring'" not in content:
        match = re.search(
            r'(    def _on_config_changed\(self\):\s*\n'
            r'        """Called whenever any config widget changes\."""\s*\n)'
            r'(        self\._rebuild_config\(\))',
            content
        )
        if match:
            content = content[:match.start(2)] + \
                "        if getattr(self, '_restoring', False):\n" \
                "            return  # v7.2.6: skip during config restore\n" + \
                content[match.start(2):]
            changed = True
            rc(True, "Guarded _on_config_changed with _restoring check")
        else:
            rc(False, "_on_config_changed pattern mismatch")
    else:
        rc(False, "_restoring guard already in _on_config_changed")

    # 1c. Add _restoring = True at start of _apply_config_to_ui
    if "self._restoring = True" not in content:
        # Find the logger.info line that starts the restore
        match = re.search(
            r'(    def _apply_config_to_ui\(self\):.*?""")\s*\n(        logger\.info)',
            content, re.DOTALL
        )
        if match:
            content = content[:match.end(1)] + \
                "\n        self._restoring = True  # v7.2.6\n" + \
                content[match.end(1):]
            changed = True
            rc(True, "Set _restoring = True at start of _apply_config_to_ui")
        else:
            rc(False, "_apply_config_to_ui logger.info pattern mismatch")
    else:
        rc(False, "_restoring = True already present")

    # 1d. Add _restoring = False before the final _on_config_changed call
    if "self._restoring = False" not in content:
        # Find the emit signals section: "# ── N. Emit signals" followed by _on_config_changed
        match = re.search(
            r'(        # ── \d+\. Emit signals ─+\n)(        self\._on_config_changed\(\))',
            content
        )
        if match:
            content = content[:match.start(2)] + \
                "        self._restoring = False  # v7.2.6: re-enable\n" + \
                content[match.start(2):]
            changed = True
            rc(True, "Set _restoring = False before emit")
        else:
            rc(False, "Emit signals section pattern mismatch")
    else:
        rc(False, "_restoring = False already present")

    # 1e. Block signals during pump restore loop
    if "v7.2.6: Block signals during pump restore" not in content:
        match = re.search(
            r'(                pw\.set_config\(pcfg, ink_names=ink_names\))',
            content
        )
        if match:
            content = content[:match.start()] + \
                "                # v7.2.6: Block signals during pump restore\n" \
                "                pw.blockSignals(True)\n" \
                "                pw.set_config(pcfg, ink_names=ink_names)\n" \
                "                pw.blockSignals(False)" + \
                content[match.end():]
            changed = True
            rc(True, "Blocked signals during pump restore")
        else:
            rc(False, "pw.set_config pattern not found")
    else:
        rc(False, "Pump signal blocking already present")

    if changed:
        safe_write(path, content, "config restore guard")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 2: gui/pages/calibration.py — Defer camera detection
# ═══════════════════════════════════════════════════════════════════

def fix_calibration(root: Path):
    path = root / "gui" / "pages" / "calibration.py"
    print(f"\n{C}[2/3] {path.name}{X}")
    content = safe_read(path)
    if not content: return

    changed = False

    # 2a. Comment out detect_cameras() calls in init/build
    patterns = [
        (r'(\s*)(self\._available_cameras\s*=\s*detect_cameras\([^)]*\))',
         r'\1# v7.2.6: Deferred to manual button\n\1self._available_cameras = []'),
        (r'(\s*)(available\s*=\s*detect_cameras\([^)]*\))',
         r'\1# v7.2.6: Deferred\n\1available = []'),
        (r'(\s*)(cameras\s*=\s*detect_cameras\([^)]*\))',
         r'\1# v7.2.6: Deferred\n\1cameras = []'),
    ]
    for pat, repl in patterns:
        if re.search(pat, content) and 'v7.2.6: Deferred' not in content:
            content, n = re.subn(pat, repl, content, count=1)
            if n > 0:
                changed = True
                rc(True, "Deferred detect_cameras() from startup")
                break
    else:
        if 'v7.2.6: Deferred' in content or 'detect_cameras(' not in content:
            rc(False, "detect_cameras already deferred or not found")

    # 2b. Add _on_detect_cameras method if not present
    if '_on_detect_cameras' not in content:
        method = '''
    def _on_detect_cameras(self):
        """v7.2.6: Manually trigger camera detection in background."""
        try:
            from gui.widgets.camera_widget import detect_cameras_async, CV2_AVAILABLE
        except ImportError:
            logger.warning("camera_widget not available")
            return

        if not CV2_AVAILABLE:
            logger.warning("OpenCV not installed — camera detection unavailable")
            return

        if hasattr(self, '_btn_detect_cameras'):
            self._btn_detect_cameras.setEnabled(False)
            self._btn_detect_cameras.setText("🔄 Detecting...")

        def _on_found(indices):
            from PySide6.QtCore import QTimer
            def _apply():
                self._available_cameras = indices
                if hasattr(self, '_btn_detect_cameras'):
                    self._btn_detect_cameras.setText(f"🔍 {len(indices)} camera(s)")
                    self._btn_detect_cameras.setEnabled(True)
                logger.info(f"Camera detection: {indices}")
            QTimer.singleShot(0, _apply)

        detect_cameras_async(_on_found, max_index=4)

'''
        # Insert before on_status_update
        match = re.search(r'\n(    def on_status_update\(self\):)', content)
        if match:
            content = content[:match.start()] + method + "\n" + content[match.start():]
            changed = True
            rc(True, "Added _on_detect_cameras method")
        else:
            rc(False, "Could not find on_status_update anchor")
    else:
        rc(False, "_on_detect_cameras already exists")

    if changed:
        safe_write(path, content, "camera detection deferred")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 3: gui/pages/print_setup.py — Execution pipeline
# ═══════════════════════════════════════════════════════════════════

def fix_print_setup(root: Path):
    path = root / "gui" / "pages" / "print_setup.py"
    print(f"\n{C}[3/3] {path.name}{X}")
    content = safe_read(path)
    if not content: return

    changed = False

    # 3a. Add _generated_job attribute
    if "_generated_job" not in content:
        match = re.search(r'(self\.tabs\.addTab\(self\.tab_wells[^\n]*\n)', content)
        if match:
            content = content[:match.end()] + \
                "\n        self._generated_job = None  # v7.2.6: pre-generated job\n" + \
                content[match.end():]
            changed = True
            rc(True, "Added _generated_job attribute")
        else:
            rc(False, "tab_wells addTab not found")
    else:
        rc(False, "_generated_job already exists")

    # 3b. Ensure set_hardware_config forwards to all sub-tabs
    if "def set_hardware_config" in content:
        # Find the method
        match = re.search(
            r'(    def set_hardware_config\(self, config.*?\):\s*\n(?:        """.*?"""\s*\n)?)'
            r'(.*?)(?=\n    def )',
            content, re.DOTALL
        )
        if match:
            body = match.group(2)
            needs_wells = "tab_wells.set_hardware_config" not in body
            needs_objects = "tab_objects.set_hardware_config" not in body
            needs_workspace = "tab_workspace.set_hardware_config" not in body

            if needs_wells or needs_objects or needs_workspace:
                new_body = '''
        # v7.2.6: Forward to ALL sub-tabs with error handling
        self._hw_config = config
        for tab_name in ['tab_workspace', 'tab_objects', 'tab_wells']:
            tab = getattr(self, tab_name, None)
            if tab and hasattr(tab, 'set_hardware_config'):
                try:
                    tab.set_hardware_config(config)
                except Exception as e:
                    logger.error(f"HW config forward to {tab_name} failed: {e}")
'''
                content = content[:match.start(2)] + new_body + content[match.end(2):]
                changed = True
                rc(True, "Enhanced set_hardware_config to forward to all tabs")
            else:
                rc(False, "All tab forwarding already present")
        else:
            rc(False, "set_hardware_config method pattern mismatch")
    else:
        rc(False, "set_hardware_config not found")

    # 3c. Make _send_to_monitor use _generated_job
    if "_generated_job" in content and "def _send_to_monitor" in content:
        match = re.search(r'(def _send_to_monitor\(self\):)', content)
        if match:
            method_start = match.start()
            next_def = content.find('\n    def ', method_start + 20)
            method_body = content[method_start:next_def] if next_def > 0 else content[method_start:]

            if "self._generated_job" not in method_body:
                # Find "job = self._build_current_job()" and replace
                build_match = re.search(r'job = self\._build_current_job\(\)', method_body)
                if build_match:
                    abs_pos = method_start + build_match.start()
                    old = "job = self._build_current_job()"
                    new = "# v7.2.6: Use pre-generated job if available\n        job = self._generated_job or self._build_current_job()"
                    content = content[:abs_pos] + new + content[abs_pos + len(old):]
                    changed = True
                    rc(True, "_send_to_monitor uses _generated_job")

                    # Clear after emit
                    emit_match = re.search(r'(self\.job_ready\.emit\(job\)[^\n]*\n)', content[abs_pos:])
                    if emit_match:
                        insert_pos = abs_pos + emit_match.end()
                        content = content[:insert_pos] + \
                            "        self._generated_job = None  # v7.2.6: clear after sending\n" + \
                            content[insert_pos:]
                        rc(True, "Clear _generated_job after emit")
                else:
                    rc(False, "_build_current_job() not in _send_to_monitor")
            else:
                rc(False, "_generated_job already in _send_to_monitor")
        else:
            rc(False, "_send_to_monitor not found")

    if changed:
        safe_write(path, content, "execution pipeline")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else find_root()

    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}MEBP v7.2.6 — Batch 2: Medium File Rewrites{X}")
    print(f"{B}{'═' * 60}{X}")
    print(f"Project root: {root}")

    fix_hardware_setup(root)
    fix_calibration(root)
    fix_print_setup(root)

    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}SUMMARY{X}")
    print(f"  {G}Written{X}: {_ok}  {Y}Skipped{X}: {_skip}  {R}Failed{X}: {_fail}")
    print(f"{B}{'═' * 60}{X}")
    if _fail > 0:
        sys.exit(1)
    print(f"{G}✓ Batch 2 complete.{X}")


if __name__ == "__main__":
    main()
