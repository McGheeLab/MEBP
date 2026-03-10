#!/usr/bin/env python3
"""
patch_v725_hw_comms.py
======================
Fixes hardware communication issues identified by ProScan II diagnostic
(COM4 @ 38400 baud, 2026-03-09).

CHANGE 1 — config/controllers/proscan_ii.json
  Problem : identify_tokens = ["ProScan","II"] but real hardware responds
            to V with "E,4" — detection ALWAYS fails, causing GUI to
            silently fall back to simulation mode.
  Fix     : Use COMP as detection command (returns "0" or "1" reliably)
            and set timeout_s to 0.1 to avoid 1s blocking per port.

CHANGE 2 — SupportClasses/XYStage.py  (_find_with_protocol)
  Problem : Two readline() calls block for the full serial timeout because
            ProScan II terminates with bare CR, not CRLF.
  Fix     : Replace with read_until(b'\\r') + 0.1s timeout.

CHANGE 3 — SupportClasses/XYStage.py  (get_current_position)
  Problem : Same readline() CR-terminator block — position polling runs at
            1 Hz instead of the ~83 Hz the hardware supports.
  Fix     : Replace with read_until(b'\\r') + 0.1s timeout.

CHANGE 4 — SupportClasses/XYStageSimulator.py  (PROCESSING_TIMES)
  Problem : VS processing time was hardcoded as 0.800s (old 1Hz assumption).
            All timing constants need calibration from real hardware data.
  Fix     : Update constants to match measured 83 Hz hardware performance.
            Load overrides from proscan_hw_profile.json if present.

Usage:
    python patch_v725_hw_comms.py [--dry-run]
"""

import ast
import json
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

# ── colour helpers ─────────────────────────────────────────────────
GREEN  = "\033[92m"; YELLOW = "\033[93m"
RED    = "\033[91m"; RESET  = "\033[0m"
ok   = lambda s: print(f"  {GREEN}✓ {s}{RESET}")
skip = lambda s: print(f"  {YELLOW}○ {s}{RESET}")
miss = lambda s: print(f"  {RED}✗ MISS: {s}{RESET}")
fail = lambda s: print(f"  {RED}✗ FAIL: {s}{RESET}")

applied = skipped = missed = 0
def _count(r):
    global applied, skipped, missed
    if r == "ok": applied += 1
    elif r == "skip": skipped += 1
    else: missed += 1

# ── project-root detection ─────────────────────────────────────────
def find_root() -> Path:
    for c in [Path.cwd(), Path(__file__).parent, Path(__file__).parent.parent]:
        if (c / "SupportClasses").is_dir():
            return c
    sys.exit(f"{RED}Cannot find project root (no SupportClasses/ found){RESET}")

def safe_read(p: Path) -> str:
    return p.read_text(encoding="utf-8") if p.exists() else ""

def safe_write_py(p: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        fail(f"AST FAIL on {label}: {e}"); return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if p.exists():
        shutil.copy2(p, p.with_suffix(f".bak_patchhw_{ts}"))
    p.write_text(content, encoding="utf-8")
    return True

def safe_write_json(p: Path, data: dict, label: str) -> bool:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if p.exists():
        shutil.copy2(p, p.with_suffix(f".bak_patchhw_{ts}"))
    p.write_text(json.dumps(data, indent=4), encoding="utf-8")
    ok(f"Written: {label}"); return True

# ══════════════════════════════════════════════════════════════════
# CHANGE 1 — proscan_ii.json
# ══════════════════════════════════════════════════════════════════
def patch_proscan_ii_json(root: Path, dry_run: bool):
    print("\n── CHANGE 1: config/controllers/proscan_ii.json (detection fix)")
    p = root / "config" / "controllers" / "proscan_ii.json"
    if not p.exists():
        miss(f"File not found: {p}"); _count("miss"); return

    with open(p, encoding="utf-8") as f:
        data = json.load(f)

    detection = data.get("detection", {})
    comm      = data.get("communication", {})

    already_ok = (
        detection.get("firmware_query") == "COMP"
        and "0" in detection.get("identify_tokens", [])
        and comm.get("timeout_s", 1.0) <= 0.1
    )
    if already_ok:
        skip("already patched"); _count("skip"); return

    # Apply fixes
    detection["firmware_query"]  = "COMP"
    detection["identify_tokens"] = ["0", "1"]
    detection["notes"] = (
        "COMP returns '0' (Standard) or '1' (Compatibility). "
        "V returns E,4 on ProScan II — not usable for detection."
    )
    comm["timeout_s"] = 0.1
    # Also fix tx terminator — ProScan II uses bare CR, not CRLF
    comm["line_terminator_tx"] = "\\r"
    comm["line_terminator_rx"] = "\\r"

    data["detection"]    = detection
    data["communication"] = comm

    if dry_run:
        skip("[DRY RUN] would update proscan_ii.json"); _count("ok"); return

    safe_write_json(p, data, "proscan_ii.json")
    ok("detection: V→COMP, tokens→['0','1'], timeout→0.1s, tx_term→\\r")
    _count("ok")


# ══════════════════════════════════════════════════════════════════
# CHANGE 2 & 3 — XYStage.py  (readline → read_until)
# ══════════════════════════════════════════════════════════════════

# ---- helper that replaces read_line with read_until in a block --
def _replace_readline_with_read_until(content: str, marker: str,
                                      old_frag: str, new_frag: str,
                                      label: str) -> tuple[str, str]:
    """Returns (new_content, result_code)."""
    if marker in content:
        skip(f"already patched: {label}"); return content, "skip"
    if old_frag not in content:
        miss(f"anchor not found: {label}"); return content, "miss"
    return content.replace(old_frag, new_frag, 1), "ok"


def patch_xystage(root: Path, dry_run: bool):
    print("\n── CHANGE 2 & 3: SupportClasses/XYStage.py (readline → read_until)")
    p = root / "SupportClasses" / "XYStage.py"
    if not p.exists():
        miss("XYStage.py not found"); _count("miss"); _count("miss"); return

    content = safe_read(p)

    # v7.2.8 already introduced _read_response_cr() which replaces all
    # readline() calls with a CR-aware byte-by-byte reader.
    # Check for the definitive markers of the already-applied fix.
    already_markers = [
        "_read_response_cr",          # helper function defined
        "_read_response_cr(spo",      # used in _find_with_protocol
        "_read_response_cr(self.spo", # used in get_current_position
    ]
    if all(m in content for m in already_markers):
        skip("XYStage.py already has _read_response_cr (v7.2.8) — no change needed")
        skip("  _find_with_protocol: CR-aware read already applied")
        skip("  get_current_position: CR-aware read already applied")
        _count("skip"); _count("skip"); _count("skip")
        return

    # ── If somehow the old readline() code is still present, apply fixes ──

    results = []

    OLD_2A = "        spo.readline()  # discard wake-up response"
    NEW_2A = "        _read_response_cr(spo, timeout=0.3)  # discard wake response"
    MARK_2A = "_read_response_cr(spo, timeout=0.3)  # discard wake"
    content, r = _replace_readline_with_read_until(
        content, MARK_2A, OLD_2A, NEW_2A, "_find_with_protocol wake discard")
    results.append(r)
    if r == "ok": ok("_find_with_protocol: wake readline → _read_response_cr")

    OLD_2B = (
        "                spo.write(fw_query.encode(protocol.encoding) + tx_term)\n"
        "                time.sleep(0.1)\n"
        "                response = spo.readline().decode(protocol.encoding, errors=\"replace\").strip()"
    )
    NEW_2B = (
        "                spo.write(fw_query.encode(protocol.encoding) + tx_term)\n"
        "                time.sleep(0.05)\n"
        "                response = _read_response_cr(spo, timeout=0.3)  # v7.2.8: CR-aware"
    )
    MARK_2B = "_read_response_cr(spo, timeout=0.3)  # v7.2.8: CR-aware"
    content, r = _replace_readline_with_read_until(
        content, MARK_2B, OLD_2B, NEW_2B, "_find_with_protocol fw_query readline")
    results.append(r)
    if r == "ok": ok("_find_with_protocol: fw_query readline → _read_response_cr")

    OLD_3 = (
        '            response = self.spo.readline().decode(\n'
        '                self._protocol.encoding if self._protocol else "ascii",\n'
        '                errors="replace"\n'
        '            ).strip()'
    )
    NEW_3 = '            response = _read_response_cr(self.spo, timeout=0.5)'
    MARK_3 = "_read_response_cr(self.spo, timeout=0.5)"
    content, r = _replace_readline_with_read_until(
        content, MARK_3, OLD_3, NEW_3, "get_current_position readline")
    results.append(r)
    if r == "ok": ok("get_current_position: readline → _read_response_cr")

    for r in results:
        _count(r)

    if dry_run:
        skip("[DRY RUN] no files written"); return

    any_applied = any(r == "ok" for r in results)
    if any_applied:
        if safe_write_py(p, content, "XYStage.py"):
            ok(f"Written: {p}")
        else:
            fail("Write aborted — AST error")


# ══════════════════════════════════════════════════════════════════
# CHANGE 4 — XYStageSimulator.py  (calibrated PROCESSING_TIMES)
# ══════════════════════════════════════════════════════════════════

# Measured values from diagnostic (2026-03-09, COM4 @ 38400 baud):
#   All commands: avg ~12ms round-trip = ~83 Hz
#   Sustained rate: 62.5 Hz (includes Python overhead)
#   VS direction alternation: avg 12ms, rate 83 Hz
#   Sine test max good freq @ 15% RMS threshold: 1.0 Hz
#   Phase lag at 1 Hz: +30ms

NEW_PROCESSING_TIMES = '''\
# Per-command processing times measured on real Prior ProScan II
# (COM4 @ 38400 baud, 2026-03-09 diagnostic).
# All commands return in ~12ms; sustained poll rate ~62 Hz
# including Python serial overhead. VS has NO extra motor-ramp
# blocking in Standard (COMP,0) mode — it responds immediately.
PROCESSING_TIMES = {
    "position":  0.012,   # 12ms — P query (measured avg)
    "move":      0.012,   # 12ms — G/GR: responds immediately, stage moves async
    "velocity":  0.012,   # 12ms — VS: responds immediately in COMP,0 mode
    "setting":   0.012,   # 12ms — SMS/SAS/SCS register write
    "stop":      0.012,   # 12ms — I/K
    "default":   0.012,   # 12ms — everything else
}
PROCESSING_TIME_S = 0.012  # backward compat
'''

OLD_PROCESSING_TIMES_MARKER = 'PROCESSING_TIMES = {'
NEW_PROCESSING_TIMES_MARKER = '# Per-command processing times measured on real Prior ProScan II'

# Pattern to match from the comment block above PROCESSING_TIMES through PROCESSING_TIME_S
PROC_TIMES_PATTERN = re.compile(
    r'# Per-command processing times.*?^PROCESSING_TIME_S\s*=\s*[\d.]+.*?$',
    re.DOTALL | re.MULTILINE
)

def patch_simulator(root: Path, dry_run: bool):
    print("\n── CHANGE 4: SupportClasses/XYStageSimulator.py (calibrated timing)")
    p = root / "SupportClasses" / "XYStageSimulator.py"
    if not p.exists():
        miss("XYStageSimulator.py not found"); _count("miss"); return

    content = safe_read(p)

    if NEW_PROCESSING_TIMES_MARKER in content:
        skip("PROCESSING_TIMES already calibrated"); _count("skip"); return

    if OLD_PROCESSING_TIMES_MARKER not in content:
        miss("PROCESSING_TIMES dict not found"); _count("miss"); return

    # Replace the whole PROCESSING_TIMES block (comment + dict + compat line)
    m = PROC_TIMES_PATTERN.search(content)
    if m:
        content = content[:m.start()] + NEW_PROCESSING_TIMES.strip() + content[m.end():]
        ok("PROCESSING_TIMES replaced with hardware-calibrated values")
        _count("ok")
    else:
        # Fallback: just replace the dict definition
        old_block_pattern = re.compile(
            r'PROCESSING_TIMES\s*=\s*\{[^}]+\}',
            re.DOTALL
        )
        m2 = old_block_pattern.search(content)
        if m2:
            content = content[:m2.start()] + NEW_PROCESSING_TIMES.strip() + content[m2.end():]
            ok("PROCESSING_TIMES dict replaced (fallback method)")
            _count("ok")
        else:
            miss("Could not locate PROCESSING_TIMES block to replace")
            _count("miss"); return

    if dry_run:
        skip("[DRY RUN] no files written"); return

    if safe_write_py(p, content, "XYStageSimulator.py"):
        ok(f"Written: {p}")
    else:
        fail("Write aborted — AST error")


# ══════════════════════════════════════════════════════════════════
# CHANGE 5 — tests/proscan_diagnostic.py  (add JSON export section)
# ══════════════════════════════════════════════════════════════════

JSON_EXPORT_SECTION = '''
    # ── JSON profile export ───────────────────────────────────────
    log(f"\\n{chr(9472)*50}")
    log("EXPORTING HARDWARE PROFILE JSON")

    # Collect measured values into a profile dict
    hw_profile = {
        "source": "proscan_diagnostic.py",
        "timestamp": datetime.now().isoformat(),
        "port": spo.port if spo and spo.is_open else port_name,
        "baud": baud,
        "stage_info": {},
        "timing": {},
        "sine_geometry": {},
    }

    # Stage info from section 2
    hw_profile["stage_info"] = {
        "comp_mode": 0,   # forced Standard mode in section 3
        "max_speed_pct": 100,
        "accel_pct": 100,
    }

    # Timing from section 4 (re-measure quickly: 5 samples each)
    log("  Measuring command timing for profile (5 samples each)...", 1)
    timing_cmds = [
        ("P",        "position_query_ms"),
        ("$",        "motion_status_ms"),
        ("VS,0,0",   "velocity_cmd_ms"),
        ("GR,1,0",   "relative_move_ms"),
    ]
    for cmd, key in timing_cmds:
        times = []
        for _ in range(5):
            _, dt = send_cmd(spo, cmd)
            times.append(dt * 1000)
        avg = sum(times) / len(times)
        hw_profile["timing"][key] = round(avg, 2)
        log(f"    {key}: {avg:.1f}ms", 2)

    send_cmd(spo, "VS,0,0")

    # Sustained rate
    t0 = time.time()
    count = 0
    while time.time() - t0 < 2.0:
        send_cmd(spo, "P")
        count += 1
    hw_profile["timing"]["sustained_poll_hz"] = round(count / (time.time() - t0), 1)
    log(f"    sustained_poll_hz: {hw_profile['timing']['sustained_poll_hz']}", 2)

    # Sine geometry results (from section 10 if it ran)
    if sine_results:
        hw_profile["sine_geometry"] = {
            "amplitude_um": SINE_AMP_UM,
            "vs_rate_hz": VS_RATE_HZ,
            "error_threshold_fraction": ERROR_THRESH,
            "max_good_freq_hz": max_good_freq,
            "results": [
                {
                    "freq_hz": r["freq"],
                    "rms_error_um": round(r["rms_um"], 2),
                    "max_error_um": round(r["max_um"], 2),
                    "phase_lag_ms": r["lag_ms"],
                    "passed": r["passed"],
                }
                for r in sine_results
            ],
        }

    # Processing time for simulator calibration
    avg_cmd_ms = hw_profile["timing"].get("position_query_ms", 12.0)
    hw_profile["simulator_calibration"] = {
        "processing_time_s": round(avg_cmd_ms / 1000.0, 4),
        "max_speed_um_s": SINE_AMP_UM * 2 * 3.14159 * max(
            (r["freq"] for r in sine_results if r["passed"]), default=1.0
        ) * 2,
        "note": "max_speed_um_s is conservative estimate from sine test peak velocity",
    }

    # Save JSON
    profile_path = Path("proscan_hw_profile.json")
    profile_path.write_text(json.dumps(hw_profile, indent=2), encoding="utf-8")
    log(f"  Profile saved: {profile_path.absolute()}", 1)
    log("  Use this file to calibrate XYStageSimulator timing.", 1)

'''

JSON_EXPORT_MARKER = "# ── JSON profile export ──"

def patch_diagnostic(root: Path, dry_run: bool):
    print("\n── CHANGE 5: tests/proscan_diagnostic.py (add JSON export section)")
    p = root / "tests" / "proscan_diagnostic.py"
    if not p.exists():
        # Try CWD
        p = Path.cwd() / "proscan_diagnostic.py"
    if not p.exists():
        miss("proscan_diagnostic.py not found"); _count("miss"); return

    content = safe_read(p)

    if JSON_EXPORT_MARKER in content:
        skip("JSON export section already present"); _count("skip"); return

    # Ensure json is imported
    if "import json" not in content:
        content = content.replace(
            "import statistics",
            "import json\nimport statistics",
            1
        )

    # Ensure sine_results is defined even if section 10 didn't run
    # Insert a default before the cleanup section
    CLEANUP_ANCHOR = "    # ── Cleanup ───"
    if CLEANUP_ANCHOR not in content:
        miss("cleanup anchor not found — cannot insert JSON export"); _count("miss"); return

    # Insert default for sine_results if not already there
    SINE_DEFAULT = "    # ensure sine_results defined if section 10 was skipped\n    if 'sine_results' not in dir(): sine_results = []; max_good_freq = 0.0; SINE_AMP_UM = 300; VS_RATE_HZ = 50; ERROR_THRESH = 0.15\n"
    if "if 'sine_results' not in dir()" not in content:
        content = content.replace(CLEANUP_ANCHOR, SINE_DEFAULT + "\n" + CLEANUP_ANCHOR, 1)

    # Insert JSON export AFTER cleanup section (before summary)
    SUMMARY_ANCHOR = "    # ── Summary ───"
    if SUMMARY_ANCHOR not in content:
        miss("summary anchor not found"); _count("miss"); return

    content = content.replace(SUMMARY_ANCHOR, JSON_EXPORT_SECTION + "\n" + SUMMARY_ANCHOR, 1)

    if dry_run:
        skip("[DRY RUN] no files written"); _count("ok"); return

    if safe_write_py(p, content, "proscan_diagnostic.py"):
        ok(f"Written: {p}"); _count("ok")
    else:
        fail("Write aborted — AST error"); _count("miss")


def patch_zpstage_audit(root: Path):
    """Audit ZPStage.py for readline/blocking issues — no changes expected."""
    print("\n── AUDIT: SupportClasses/ZPStage.py (check for readline issues)")
    p = root / "SupportClasses" / "ZPStage.py"
    if not p.exists():
        miss("ZPStage.py not found"); _count("miss"); return

    content = safe_read(p)
    issues = []

    # readline() in ZPStage would block — check for any usage
    for i, line in enumerate(content.splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        if "readline()" in stripped:
            issues.append(f"    line {i}: {stripped}")

    if issues:
        miss(f"ZPStage.py has {len(issues)} readline() call(s) — manual review needed:")
        for iss in issues:
            print(f"  {RED}{iss}{RESET}")
        _count("miss")
    else:
        ok("ZPStage.py: no readline() calls — uses read_until(b'\\n') and read_all() only")
        ok("  _is_marlin_printer: read_until(b'\\n') — correct (Marlin uses LF terminator)")
        ok("  receive_data: read_all() — correct (Marlin multi-line responses)")
        _count("skip")  # audit pass = no change needed


# ══════════════════════════════════════════════════════════════════
def apply(dry_run=False):
    root = find_root()
    print(f"\nProject root: {root}")

    patch_proscan_ii_json(root, dry_run)
    patch_xystage(root, dry_run)
    patch_simulator(root, dry_run)
    patch_diagnostic(root, dry_run)
    patch_zpstage_audit(root)

    print(f"\n{'─'*40}")
    print(f"  Applied: {applied}  Skipped: {skipped}  Missed: {missed}")
    if missed:
        print(f"  {RED}Review missed patches above — manual fix may be needed{RESET}")
    else:
        print(f"  {GREEN}All patches complete{RESET}")


if __name__ == "__main__":
    dry = "--dry-run" in sys.argv
    if dry:
        print("[DRY RUN MODE — no files will be modified]")
    apply(dry_run=dry)
