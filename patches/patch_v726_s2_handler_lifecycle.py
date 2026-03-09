#!/usr/bin/env python3
"""
patch_v726_s2_handler_lifecycle.py — v7.2.6 Session 2
Fixes handler registration leaks: ZPJogHandler and XYJogHandler now unregister
their Processor handlers on stop(). Adds try/except around serial calls in jog
loops. Guards connect_stages() against duplicate handler registration.
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

def find_method(content, name):
    """Find a class method's full extent (from def to next def/class/EOF)."""
    pat = re.compile(
        r'^    def ' + re.escape(name) + r'\(self.*?(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

ROOT = find_root()
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

# ══ S2-A: ZPJogHandler — store & unregister handlers on stop() ════
print("\n=== S2-A: ZPJogHandler handler lifecycle ===")

if "# v7.2.6: ZP registered handlers" not in content:
    # Find ZPJogHandler.__init__ and add self._registered_handlers = []
    # after the existing register_handler calls
    # Strategy: find the block of register_handler calls in ZPJogHandler and
    # replace it with a version that also appends to self._registered_handlers

    # First, add self._registered_handlers = [] near end of ZPJogHandler.__init__
    # We look for the last register_handler call in ZPJogHandler's __init__
    # and inject after it.

    # Pattern: find ZPJogHandler class block and its __init__
    zp_init_m = re.search(
        r'(class ZPJogHandler.*?def __init__\(self.*?)(self\.processor\.register_handler\("move_p3_at_velocity".*?\n)',
        content, re.DOTALL
    )
    if zp_init_m:
        insert_at = zp_init_m.end()
        inject = (
            "        # v7.2.6: ZP registered handlers — track for unregister on stop()\n"
            "        self._registered_handlers: list = [\n"
            '            ("move_z_at_velocity",    self._handle_z_vel),\n'
            '            ("incr_z_up",             self._incr_z_up),\n'
            '            ("incr_z_down",           self._incr_z_down),\n'
            '            ("move_p1_at_velocity",   self._handle_p1_vel),\n'
            '            ("move_p2_at_velocity",   self._handle_p2_vel),\n'
            '            ("move_p3_at_velocity",   self._handle_p3_vel),\n'
            '            ("incr_p_up",             self._incr_p_up),\n'
            '            ("incr_p_down",           self._incr_p_down),\n'
            "        ]\n"
        )
        content = content[:insert_at] + inject + content[insert_at:]
        print(f"{OK} Added _registered_handlers list in ZPJogHandler.__init__")
    else:
        print(f"{MISS} Could not find last register_handler in ZPJogHandler.__init__")
        failed += 1

    # Now replace ZPJogHandler.stop() with unregistering version
    new_zp_stop = '''\
    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: Unregister prevents stale callbacks on reconnect.
        """
        self._running = False
        # v7.2.6: Unregister all handlers to prevent accumulation on reconnect
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_relative({}, None)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

'''
    m = find_method(content, "stop")
    # We need the ZPJogHandler.stop, not XYJogHandler.stop
    # Find ZPJogHandler class start and then find stop within it
    zp_class_m = re.search(r'^class ZPJogHandler\b', content, re.MULTILINE)
    xy_class_m = re.search(r'^class XYJogHandler\b', content, re.MULTILINE)

    if zp_class_m and xy_class_m:
        zp_section = content[zp_class_m.start():xy_class_m.start()]
        # find stop in that section
        stop_m = re.search(
            r'    def stop\(self\).*?(?=\n    def |\nclass |\Z)',
            zp_section, re.DOTALL | re.MULTILINE
        )
        if stop_m:
            abs_start = zp_class_m.start() + stop_m.start()
            abs_end   = zp_class_m.start() + stop_m.end()
            content = content[:abs_start] + new_zp_stop + content[abs_end:]
            print(f"{OK} Replaced ZPJogHandler.stop() with unregister version")
        else:
            print(f"{MISS} ZPJogHandler.stop() not found")
            failed += 1
    else:
        print(f"{MISS} ZPJogHandler or XYJogHandler class boundary not found")
        failed += 1
else:
    print(f"{SKIP} S2-A already applied"); skipped += 1


# ══ S2-B: XYJogHandler — store & unregister handlers on stop() ════
print("\n=== S2-B: XYJogHandler handler lifecycle ===")

if "# v7.2.6: XY registered handlers" not in content:
    # Add _registered_handlers after last register_handler in XYJogHandler.__init__
    xy_class_m = re.search(r'^class XYJogHandler\b', content, re.MULTILINE)
    pos_class_m = re.search(r'^class PositionPoller\b', content, re.MULTILINE)

    if xy_class_m and pos_class_m:
        xy_section = content[xy_class_m.start():pos_class_m.start()]

        # Find last register_handler call in XY __init__
        last_reg = None
        for m in re.finditer(r'self\.processor\.register_handler\(.*?\n', xy_section):
            last_reg = m

        if last_reg:
            abs_end = xy_class_m.start() + last_reg.end()
            inject = (
                "        # v7.2.6: XY registered handlers\n"
                "        self._registered_handlers: list = [\n"
                '            ("move_xy_at_velocity", self._handle_vel),\n'
                '            ("incr_xy_up",          self._incr_up),\n'
                '            ("incr_xy_down",        self._incr_down),\n'
                "        ]\n"
            )
            content = content[:abs_end] + inject + content[abs_end:]
            print(f"{OK} Added _registered_handlers list in XYJogHandler.__init__")
        else:
            print(f"{MISS} No register_handler calls found in XYJogHandler.__init__")
            failed += 1

        # Now replace XYJogHandler.stop()
        # Recalculate class boundaries after content modification
        xy_class_m2 = re.search(r'^class XYJogHandler\b', content, re.MULTILINE)
        pos_class_m2 = re.search(r'^class PositionPoller\b', content, re.MULTILINE)
        if xy_class_m2 and pos_class_m2:
            xy_sec2 = content[xy_class_m2.start():pos_class_m2.start()]
            stop_m2 = re.search(
                r'    def stop\(self\).*?(?=\n    def |\nclass |\Z)',
                xy_sec2, re.DOTALL | re.MULTILINE
            )
            if stop_m2:
                abs_s = xy_class_m2.start() + stop_m2.start()
                abs_e = xy_class_m2.start() + stop_m2.end()
                new_xy_stop = '''\
    def stop(self) -> None:
        """Stop jog loop and unregister all Processor handlers.
        v7.2.6: Unregister prevents stale callbacks on reconnect.
        """
        self._running = False
        # v7.2.6: Unregister all handlers
        for cmd, handler in getattr(self, "_registered_handlers", []):
            try:
                self.processor.unregister_handler(cmd, handler)
            except Exception:
                pass
        try:
            self.stage.move_stage_at_velocity(0, 0)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.0)

'''
                content = content[:abs_s] + new_xy_stop + content[abs_e:]
                print(f"{OK} Replaced XYJogHandler.stop() with unregister version")
            else:
                print(f"{MISS} XYJogHandler.stop() not found after injection")
                failed += 1
    else:
        print(f"{MISS} Class boundaries not found for XYJogHandler/PositionPoller")
        failed += 1
else:
    print(f"{SKIP} S2-B already applied"); skipped += 1


# ══ S2-C: try/except around serial calls in jog loops ════════════
print("\n=== S2-C: try/except in XY and ZP jog loops ===")

if "# v7.2.6: XY jog serial guard" not in content:
    # XYJogHandler._jog_loop: wrap move_stage_at_velocity call
    # Find the line: self.stage.move_stage_at_velocity(vx, vy)
    # It's currently unprotected inside the loop
    new_c, n = re.subn(
        r'(            )(self\.stage\.move_stage_at_velocity\(vx, vy\))',
        (r'\1# v7.2.6: XY jog serial guard\n'
         r'\1try:\n'
         r'\1    \2\n'
         r'\1except Exception as e:\n'
         r'\1    logger.warning(f"[XY] Jog move failed: {e}")\n'
         r'\1    with self._lock:\n'
         r'\1        self.vel_x = self.vel_y = 0.0\n'
         r'\1    break'),
        content
    )
    if n:
        content = new_c
        print(f"{OK} Added try/except around XY jog move")
    else:
        print(f"{MISS} Could not find move_stage_at_velocity(vx, vy) in jog loop")
        failed += 1
else:
    print(f"{SKIP} S2-C XY guard already applied"); skipped += 1

if "# v7.2.6: ZP jog serial guard" not in content:
    # ZPJogHandler._jog_loop: wrap self.stage.move_relative(...)
    new_c, n = re.subn(
        r'(            )(self\.stage\.move_relative\(\s*\{[^}]+\},\s*feedrate\s*\))',
        (r'\1# v7.2.6: ZP jog serial guard\n'
         r'\1try:\n'
         r'\1    \2\n'
         r'\1except Exception as e:\n'
         r'\1    logger.warning(f"[ZP] Jog move failed: {e}")\n'
         r'\1    with self._lock:\n'
         r'\1        self.vel_z = self.vel_p1 = self.vel_p2 = self.vel_p3 = 0.0\n'
         r'\1    break'),
        content, flags=re.DOTALL
    )
    if n:
        content = new_c
        print(f"{OK} Added try/except around ZP jog move")
    else:
        print(f"{MISS} Could not find stage.move_relative in ZP jog loop")
        failed += 1
else:
    print(f"{SKIP} S2-C ZP guard already applied"); skipped += 1


# ══ S2-D: connect_stages() — stop old jog handlers before creating new ═
print("\n=== S2-D: connect_stages() duplicate handler guard ===")

if "# v7.2.6: stop old jog handlers" not in content:
    # Find connect_stages and add cleanup before jog handler creation
    # Look for the pattern where xy_jog is assigned
    new_c, n = re.subn(
        r'(            self\.xy_jog = XYJogHandler\()',
        ('            # v7.2.6: stop old jog handlers before creating new ones\n'
         '            if self.xy_jog is not None:\n'
         '                self.xy_jog.stop()\n'
         '                self.xy_jog = None\n'
         r'            \1'),
        content
    )
    if n:
        content = new_c
        print(f"{OK} Added xy_jog cleanup guard in connect_stages()")
    else:
        print(f"{MISS} Could not find 'self.xy_jog = XYJogHandler(' in connect_stages")
        failed += 1

    new_c2, n2 = re.subn(
        r'(            self\.zp_jog = ZPJogHandler\()',
        ('            # v7.2.6: stop old ZP jog handler\n'
         '            if self.zp_jog is not None:\n'
         '                self.zp_jog.stop()\n'
         '                self.zp_jog = None\n'
         r'            \1'),
        content
    )
    if n2:
        content = new_c2
        print(f"{OK} Added zp_jog cleanup guard in connect_stages()")
    else:
        print(f"{MISS} Could not find 'self.zp_jog = ZPJogHandler(' in connect_stages")
        failed += 1
else:
    print(f"{SKIP} S2-D already applied"); skipped += 1

# Write StageController
safe_write(sc_path, content, "StageController.py S2")

print(f"\n{'='*50}")
print(f"S2 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
