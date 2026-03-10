#!/usr/bin/env python3
"""
Quick diagnostic: checks for known serial communication bugs.
Works by reading source files directly — no module imports needed.

Run from project root: python tests/check_serial_bugs.py
"""

import re
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent

# Find project root — works whether script is in tests/ or project root
def _find_root():
    for p in [_SCRIPT_DIR, _SCRIPT_DIR.parent, _SCRIPT_DIR.parent.parent]:
        if (p / "SupportClasses").is_dir():
            return p
    # Last resort: current working directory
    cwd = Path.cwd()
    if (cwd / "SupportClasses").is_dir():
        return cwd
    print(f"ERROR: Cannot find project root (SupportClasses/ not found)")
    print(f"  Script dir: {_SCRIPT_DIR}")
    print(f"  CWD: {cwd}")
    sys.exit(1)

_ROOT = _find_root()

PASS = 0
WARN = 0
FAIL = 0

def ok(msg):
    global PASS; PASS += 1; print(f"  \u2705 {msg}")
def warn(msg):
    global WARN; WARN += 1; print(f"  \u26a0\ufe0f  {msg}")
def fail(msg):
    global FAIL; FAIL += 1; print(f"  \u274c {msg}")


print("=" * 65)
print("  Serial Communication Bug Checker (file-based)")
print("=" * 65)
print(f"  Project root: {_ROOT}")

xy_path = _ROOT / "SupportClasses" / "XYStage.py"
zp_path = _ROOT / "SupportClasses" / "ZPStage.py"
utils_path = _ROOT / "SupportClasses" / "SerialUtils.py"

# ── 1. Check _read_response_cr for double-escape bug ─────────────
print("\n[1] XYStage._read_response_cr \u2014 CR byte comparison")

if not xy_path.exists():
    fail(f"XYStage.py not found at {xy_path}")
else:
    raw = xy_path.read_bytes()

    # The BUGGY pattern is literal bytes: b"\\r" (backslash backslash r)
    buggy_cr = b'b"\\\\r"'
    buggy_lf = b'b"\\\\n"'

    cr_count = raw.count(buggy_cr)
    lf_count = raw.count(buggy_lf)

    if cr_count > 0 or lf_count > 0:
        fail(f"DOUBLE-ESCAPE BUG: found {cr_count} b\"\\\\r\" and {lf_count} b\"\\\\n\"")
        fail("  _read_response_cr compares against 2-byte backslash+r, not 1-byte CR")
        fail("  Every position query blocks for ~500ms until timeout")
        fail("  Fix: python patches/patch_fix_cr_escape.py")
    else:
        # Check correct pattern exists
        content = raw.decode("utf-8", errors="replace")
        if '_read_response_cr' in content:
            if 'b"\\r"' in content or "b'\\r'" in content:
                ok("_read_response_cr uses correct single-escape b\"\\r\"")
            else:
                warn("_read_response_cr found but could not verify byte comparison")
        else:
            warn("_read_response_cr function not found")

# ── 2. Check XY get_current_position read method ─────────────────
print("\n[2] XYStage.get_current_position \u2014 read method")

if xy_path.exists():
    content = xy_path.read_text(encoding="utf-8", errors="replace")

    # Find get_current_position method body
    gcp_match = re.search(r'def get_current_position\(self\).*?(?=\n    def |\nclass |\Z)',
                           content, re.DOTALL)
    if gcp_match:
        gcp_body = gcp_match.group()
        if "readline" in gcp_body:
            fail("Uses readline() \u2014 blocks on ProScan II CR terminator")
        elif "_read_response_cr" in gcp_body:
            ok("Uses _read_response_cr (CR-aware)")
        elif "read_until" in gcp_body:
            ok("Uses read_until (explicit terminator)")
        else:
            warn("Could not determine read method \u2014 check manually")
    else:
        warn("Could not find get_current_position method")

# ── 3. Check XY _find_with_protocol detection read ───────────────
print("\n[3] XYStage._find_with_protocol \u2014 detection read method")

if xy_path.exists():
    fwp_match = re.search(r'def _find_with_protocol\(self.*?(?=\n    def |\nclass |\Z)',
                           content, re.DOTALL)
    if fwp_match:
        fwp_body = fwp_match.group()
        readline_count = fwp_body.count("readline")
        cr_read_count = fwp_body.count("_read_response_cr")

        if readline_count > 0:
            fail(f"Still has {readline_count} readline() call(s) \u2014 detection slow")
        else:
            ok("No readline() calls in detection")

        if cr_read_count >= 2:
            ok(f"Uses _read_response_cr {cr_read_count}x (wake + query)")
        elif cr_read_count == 1:
            warn("Only 1 _read_response_cr call \u2014 expected 2")
        elif cr_read_count == 0 and readline_count == 0:
            warn("No read calls found in detection \u2014 check manually")
    else:
        warn("Could not find _find_with_protocol method")

# ── 4. Check ZP get_current_position ──────────────────────────────
print("\n[4] ZPStage.get_current_position \u2014 read method")

if not zp_path.exists():
    fail(f"ZPStage.py not found at {zp_path}")
else:
    zp_content = zp_path.read_text(encoding="utf-8", errors="replace")

    gcp_match = re.search(r'def get_current_position\(self\).*?(?=\n    def |\nclass |\Z)',
                           zp_content, re.DOTALL)
    if gcp_match:
        gcp_body = gcp_match.group()
        if "readline" in gcp_body:
            fail("Uses readline() \u2014 may block")
        elif "receive_data" in gcp_body:
            ok("Uses receive_data (read_all-based, non-blocking)")
        elif "read_all" in gcp_body:
            ok("Uses read_all (non-blocking)")
        else:
            warn("Could not determine read method")
    else:
        warn("Could not find ZP get_current_position")

# ── 5. Check ZP receive_data ─────────────────────────────────────
print("\n[5] ZPStage.receive_data \u2014 sleep timing")

if zp_path.exists():
    rd_match = re.search(r'def receive_data\(self\).*?(?=\n    def |\nclass |\Z)',
                          zp_content, re.DOTALL)
    if rd_match:
        rd_body = rd_match.group()
        if "readline" in rd_body:
            fail("Uses readline() \u2014 should use read_all()")
        elif "read_all" in rd_body:
            ok("Uses read_all() (non-blocking)")

        sleep_match = re.search(r'sleep\((\d+\.?\d*)\)', rd_body)
        if sleep_match:
            sleep_s = float(sleep_match.group(1))
            if sleep_s < 0.005:
                warn(f"Pre-read sleep is only {sleep_s*1000:.0f}ms \u2014 may miss slow responses")
            elif sleep_s > 0.1:
                warn(f"Pre-read sleep is {sleep_s*1000:.0f}ms \u2014 unnecessarily slow")
            else:
                ok(f"Pre-read sleep is {sleep_s*1000:.0f}ms (reasonable)")
    else:
        warn("Could not find receive_data method")

# ── 6. Check ZP _is_marlin_printer read_until ─────────────────────
print("\n[6] ZPStage._is_marlin_printer \u2014 read_until terminator")

if zp_path.exists():
    zp_raw = zp_path.read_bytes()
    # Check for double-escaped read_until
    if b'read_until(b"\\\\n")' in zp_raw or b"read_until(b'\\\\n')" in zp_raw:
        fail("read_until uses double-escaped b\"\\\\n\" \u2014 will timeout every probe")
        fail("  Fix: change to read_until(b\"\\n\")")
    elif b'read_until(b"\\n")' in zp_raw or b"read_until(b'\\n')" in zp_raw:
        ok("read_until uses correct b\"\\n\" (LF byte) for Marlin")
    else:
        imp_match = re.search(r'read_until', zp_content)
        if imp_match:
            warn("Found read_until but could not verify terminator bytes")
        else:
            ok("No read_until calls in ZPStage.py")

# ── 7. Check SerialUtils.safe_readline not used by stages ─────────
print("\n[7] SerialUtils.safe_readline \u2014 usage in stages")

if xy_path.exists():
    if "safe_readline" in content:
        fail("XYStageManager references safe_readline \u2014 will block on ProScan II")
    else:
        ok("XYStageManager does not use safe_readline")

if zp_path.exists():
    if "safe_readline" in zp_content:
        warn("ZPStageManager references safe_readline \u2014 verify it's LF-only")
    else:
        ok("ZPStageManager does not use safe_readline")

# ── 8. Live hardware test ─────────────────────────────────────────
print("\n[8] Live hardware latency test")

try:
    sys.path.insert(0, str(_ROOT))
    import time
    from SupportClasses.XYStage import XYStageManager

    xy = XYStageManager(simulate=False, controller_json="auto")
    if xy.spo is not None:
        times = []
        for _ in range(10):
            t0 = time.monotonic()
            pos = xy.get_current_position()
            times.append((time.monotonic() - t0) * 1000)
            time.sleep(0.02)

        avg = sum(times) / len(times)
        print(f"      XY: 10 queries avg={avg:.1f}ms min={min(times):.1f}ms max={max(times):.1f}ms")

        if avg > 100:
            fail(f"XY latency {avg:.0f}ms \u2014 CR matching bug STILL PRESENT")
        elif avg > 30:
            warn(f"XY latency {avg:.0f}ms \u2014 higher than expected (~12ms)")
        else:
            ok(f"XY latency {avg:.1f}ms \u2014 matches expected ~12ms")
        xy.stop()
    else:
        print("      No real XY hardware \u2014 skipping")
except Exception as e:
    print(f"      XY import/connect failed: {e}")
    print("      (This is expected if XYStageSimulator has import issues)")
    print("      File-based checks above are still valid.")

try:
    from SupportClasses.ZPStage import ZPStageManager
    zp = ZPStageManager(simulate=False)
    if zp.serial is not None and hasattr(zp.serial, 'is_open') and zp.serial.is_open:
        times = []
        for _ in range(10):
            t0 = time.monotonic()
            zp.get_current_position()
            times.append((time.monotonic() - t0) * 1000)
            time.sleep(0.1)

        avg = sum(times) / len(times)
        print(f"      ZP: 10 queries avg={avg:.1f}ms min={min(times):.1f}ms max={max(times):.1f}ms")

        if avg > 200:
            fail(f"ZP latency {avg:.0f}ms \u2014 possible read issue")
        elif avg > 50:
            warn(f"ZP latency {avg:.0f}ms \u2014 higher than expected")
        else:
            ok(f"ZP latency {avg:.1f}ms \u2014 healthy")
        zp.stop()
    else:
        print("      No real ZP hardware \u2014 skipping")
except Exception as e:
    print(f"      ZP import/connect failed: {e}")

# ── Summary ───────────────────────────────────────────────────────
print(f"\n{'='*65}")
print(f"  Results: {PASS} passed, {WARN} warnings, {FAIL} failures")
if FAIL > 0:
    print(f"  \u274c SERIAL BUGS FOUND \u2014 fix before trusting calibration data")
    print(f"  Run: python patches/patch_fix_cr_escape.py")
elif WARN > 0:
    print(f"  \u26a0\ufe0f  Minor concerns \u2014 review warnings above")
else:
    print(f"  \u2705 All serial paths look clean")
print(f"{'='*65}")

sys.exit(1 if FAIL > 0 else 0)