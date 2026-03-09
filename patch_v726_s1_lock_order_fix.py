#!/usr/bin/env python3
"""
patch_v726_s1_lock_order_fix.py — Fix XYStage.py + ZPStage.py lock init order.

Root cause: S1 placed self._serial_lock = threading.RLock() AFTER
self.spo = self._initialise_serial(). But _initialise_serial() calls
send_command() during port detection, which references self._serial_lock
before it exists → AttributeError on every connect attempt.

Fix: move lock init to BEFORE _initialise_serial() in both files.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; SKIP = f"{YELLOW}  o{RESET}"; MISS = f"{RED}  x{RESET}"
applied = 0; failed = 0

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

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

# ══ XYStage.py ════════════════════════════════════════════════════
print("\n=== XYStage.py: move _serial_lock init before _initialise_serial() ===")
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = xy_path.read_text(encoding="utf-8")

if "# v7.2.6: lock before init" not in content:
    # Remove the lock line from wherever S1 placed it (after spo = ...)
    content, n = re.subn(
        r'\n        self\._serial_lock = threading\.RLock\(\)  # v7\.2\.6: XY serial lock\n',
        '\n',
        content
    )
    if n == 0:
        # Try without the comment suffix
        content, n = re.subn(
            r'\n        self\._serial_lock = threading\.RLock\(\)\n',
            '\n',
            content, count=1
        )
    if n == 0:
        print(f"{MISS} Could not remove old lock line from XYStage.__init__")
        failed += 1
    else:
        print(f"{OK} Removed misplaced _serial_lock from after _initialise_serial()")

    # Now insert BEFORE self.spo = self._initialise_serial()
    old = "        self.spo = self._initialise_serial()"
    new = (
        "        # v7.2.6: lock before init — must exist before send_command() is called\n"
        "        self._serial_lock = threading.RLock()  # v7.2.6: XY serial lock\n"
        "        self.spo = self._initialise_serial()"
    )
    if old in content:
        content = content.replace(old, new, 1)
        print(f"{OK} Inserted _serial_lock BEFORE _initialise_serial() in XYStage")
        safe_write(xy_path, content, "XYStage.py")
    else:
        print(f"{MISS} 'self.spo = self._initialise_serial()' not found in XYStage.__init__")
        failed += 1
else:
    print(f"{SKIP} XYStage.py already fixed"); 

# ══ ZPStage.py ════════════════════════════════════════════════════
print("\n=== ZPStage.py: move _serial_lock init before serial open ===")
zp_path = ROOT / "SupportClasses" / "ZPStage.py"
content_z = zp_path.read_text(encoding="utf-8")

if "# v7.2.6: lock before init" not in content_z:
    # Remove the lock line from wherever S1 placed it
    content_z, n = re.subn(
        r'\n        self\._serial_lock = threading\.RLock\(\)  # v7\.2\.6: ZP serial lock\n',
        '\n',
        content_z
    )
    if n == 0:
        content_z, n = re.subn(
            r'\n        self\._serial_lock = threading\.RLock\(\)\n',
            '\n',
            content_z, count=1
        )
    if n == 0:
        print(f"{MISS} Could not remove old ZP lock line"); failed += 1
    else:
        print(f"{OK} Removed misplaced _serial_lock from ZPStage.__init__")

    # ZPStage opens serial inline. Find the serial.Serial(...) assignment.
    # Insert lock before self.serial = serial.Serial(...)
    # Pattern varies — search for the serial open line
    m = re.search(r'^( +)(self\.serial\s*=\s*serial\.Serial\()', content_z, re.MULTILINE)
    if m:
        indent = m.group(1)
        insert_pos = m.start()
        lock_line = (
            f"{indent}# v7.2.6: lock before init — must exist before send_data() is called\n"
            f"{indent}self._serial_lock = threading.RLock()  # v7.2.6: ZP serial lock\n"
        )
        content_z = content_z[:insert_pos] + lock_line + content_z[insert_pos:]
        print(f"{OK} Inserted _serial_lock BEFORE serial.Serial() in ZPStage")
        safe_write(zp_path, content_z, "ZPStage.py")
    else:
        # Try alternative: insert before self.simulate assignment or first use of send_data
        m2 = re.search(r'^( +)(self\.simulate\s*=)', content_z, re.MULTILINE)
        if m2:
            indent = m2.group(1)
            insert_pos = m2.start()
            lock_line = (
                f"{indent}# v7.2.6: lock before init\n"
                f"{indent}self._serial_lock = threading.RLock()  # v7.2.6: ZP serial lock\n"
            )
            content_z = content_z[:insert_pos] + lock_line + content_z[insert_pos:]
            print(f"{OK} Inserted _serial_lock at start of ZPStage.__init__ (fallback)")
            safe_write(zp_path, content_z, "ZPStage.py")
        else:
            print(f"{MISS} Cannot find insertion point in ZPStage.__init__"); failed += 1
else:
    print(f"{SKIP} ZPStage.py already fixed")

print(f"\n{'='*50}")
print(f"Results: {GREEN}{applied} fixed{RESET}  {RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
