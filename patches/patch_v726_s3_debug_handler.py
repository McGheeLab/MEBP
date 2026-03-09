#!/usr/bin/env python3
"""
patch_v726_s3_debug_handler.py — v7.2.6 Session 3
Adds 'debug' command handler to StageController so Xbox debug messages
always have a handler. Adds connect-order warning in connect_xbox().
Optionally disables/enables Xbox connect button based on stage state (dashboard).
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; SKIP = f"{YELLOW}  o{RESET}"; MISS = f"{RED}  x{RESET}"
applied = 0; skipped = 0; failed = 0

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

def safe_read(path):
    if not path.exists():
        print(f"{MISS} File not found: {path}")
        return ""
    return path.read_text(encoding="utf-8")

def safe_write(path, content, label):
    global applied, failed
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"{MISS} AST FAIL in {label}: {e}")
        failed += 1
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v726_{ts}"))
    path.write_text(content, encoding="utf-8")
    applied += 1
    print(f"{OK} Written: {label}")
    return True

ROOT = find_root()

# ══ S3-A: Add debug handler to StageController ═══════════════════
print("\n=== S3-A: StageController debug command handler ===")
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

if "# v7.2.6: debug handler" not in content:
    # 1. Register the handler in __init__ after zero_needle_pos registration
    new_c, n = re.subn(
        r'(        self\.processor\.register_handler\("zero_needle_pos", self\._calibrate_zero\))',
        (r'\1\n'
         r'        # v7.2.6: debug handler — ensures Xbox debug messages always dispatch\n'
         r'        self.processor.register_handler("debug", self._handle_debug)'),
        content
    )
    if n:
        content = new_c
        print(f"{OK} Registered debug handler in StageController.__init__")
    else:
        print(f"{MISS} Could not find zero_needle_pos registration")
        failed += 1

    # 2. Add _handle_debug method — inject before connect_stages
    if "def _handle_debug" not in content:
        debug_method = '''\
    def _handle_debug(self, *args, **kwargs) -> None:
        """v7.2.6: debug handler — logs Xbox worker debug messages."""
        msg = kwargs.get("message", "") or (args[0] if args else "")
        logger.info(f"[Debug] {msg}")

'''
        # Insert before connect_stages (the first public method after __init__)
        m = re.search(r'^    def connect_stages\(self', content, re.MULTILINE)
        if m:
            content = content[:m.start()] + debug_method + content[m.start():]
            print(f"{OK} Added _handle_debug method")
        else:
            print(f"{MISS} connect_stages not found for method injection")
            failed += 1
else:
    print(f"{SKIP} S3-A already applied"); skipped += 1


# ══ S3-B: connect_xbox() — warn if stages not connected ══════════
print("\n=== S3-B: connect_xbox() connect-order warning ===")

if "# v7.2.6: connect order warning" not in content:
    # Find connect_xbox and add warning at the start of the method body
    m = re.search(
        r'(    def connect_xbox\(self.*?\n)(        )',
        content, re.DOTALL
    )
    if m:
        # Insert warning after the def line + any docstring
        # Safer: look for the first non-docstring line inside connect_xbox
        cx_m = re.search(r'    def connect_xbox\(self[^)]*\)[^:]*:\n', content)
        if cx_m:
            # Find end of docstring or start of code
            after_def = cx_m.end()
            # Check for docstring
            doc_m = re.match(r'(\s+""".*?""")\n', content[after_def:], re.DOTALL)
            if doc_m:
                inject_at = after_def + doc_m.end()
            else:
                inject_at = after_def
            warning_code = (
                '        # v7.2.6: connect order warning\n'
                '        if self.xy_stage is None and self.zp_stage is None:\n'
                '            logger.warning(\n'
                '                "Xbox connected but no stages are connected -- "\n'
                '                "controller input will have no effect until stages connect"\n'
                '            )\n'
            )
            content = content[:inject_at] + warning_code + content[inject_at:]
            print(f"{OK} Added connect-order warning in connect_xbox()")
        else:
            print(f"{MISS} connect_xbox def line not matched precisely")
            failed += 1
    else:
        print(f"{MISS} connect_xbox method not found")
        failed += 1
else:
    print(f"{SKIP} S3-B already applied"); skipped += 1

safe_write(sc_path, content, "StageController.py S3")


# ══ S3-C: Dashboard — Xbox connect button awareness ══════════════
print("\n=== S3-C: Dashboard Xbox connect button state ===")
dash_path = ROOT / "gui" / "pages" / "dashboard.py"
content_d = safe_read(dash_path)
if not content_d:
    print(f"{MISS} dashboard.py not found — skipping S3-C")
    skipped += 1
elif "# v7.2.6: xbox button awareness" not in content_d:
    # Find the status update timer callback and add Xbox button enable/disable
    # We look for the on_status_update method and add button state logic
    # This is optional/nice-to-have — inject safely

    # Find _update_conn_status calls and add xbox button enable logic after them
    # Look for where we already update xy and zp connection status
    btn_code = '''
    def _update_xbox_btn_state(self) -> None:
        """v7.2.6: xbox button awareness — grey out Xbox connect when no stages connected."""
        if not hasattr(self, "_btn_connect_xbox"):
            return
        xy_ok = getattr(self.controller, "xy_stage", None) is not None
        zp_ok = getattr(self.controller, "zp_stage", None) is not None
        enabled = xy_ok or zp_ok
        self._btn_connect_xbox.setEnabled(enabled)
        tip = "" if enabled else "Connect XY or ZP stage first"
        self._btn_connect_xbox.setToolTip(tip)

'''
    # Inject before on_status_update or near end of class
    m = re.search(r'^    def on_status_update\(self', content_d, re.MULTILINE)
    if m:
        content_d = content_d[:m.start()] + btn_code + content_d[m.start():]

        # Also call _update_xbox_btn_state from on_status_update
        # Add call at end of on_status_update method
        # Find on_status_update and inject at end
        m2 = re.search(
            r'    def on_status_update\(self.*?(?=\n    def |\nclass |\Z)',
            content_d, re.DOTALL | re.MULTILINE
        )
        if m2:
            old_body = m2.group(0)
            # Add call before the closing of the method
            if "_update_xbox_btn_state" not in old_body:
                new_body = old_body.rstrip() + "\n        self._update_xbox_btn_state()\n"
                content_d = content_d[:m2.start()] + new_body + content_d[m2.end():]
                print(f"{OK} Added _update_xbox_btn_state method and call in dashboard")
            else:
                print(f"{SKIP} _update_xbox_btn_state call already in on_status_update")
        safe_write(dash_path, content_d, "dashboard.py S3-C")
    else:
        print(f"{MISS} on_status_update not found in dashboard.py")
        skipped += 1
else:
    print(f"{SKIP} S3-C already applied"); skipped += 1

print(f"\n{'='*50}")
print(f"S3 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
