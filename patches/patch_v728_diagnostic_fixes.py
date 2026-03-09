#!/usr/bin/env python3
"""
MEBP v7.2.8 — Diagnostic-based communication fixes.

Patches:
  A. XYStageSimulator.py — Load timing from config/xy_diagnostic_profile.json
  B. config/xy_diagnostic_profile.json — Verify exists (created separately)
  C. config/controllers/proscan_ii.json — Fix detection tokens
  D. config/controllers/proscan_iii.json — Fix overly-broad "E" token
  E. XYStage.py — CR-aware readline for detection + response_pattern support
  F. main.py — Pass controller_json from settings
  G. StageController.py — Error handling + poll interval fix

Usage:
  python patch_v728_diagnostic_fixes.py
  python patch_v728_diagnostic_fixes.py /path/to/MEBP
"""

import ast
import json
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

# ── Terminal Colors ──────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

# ── Counters ─────────────────────────────────────────────────────
applied = 0
skipped = 0
failed  = 0

def ok(msg):
    global applied; applied += 1
    print(f"  {GREEN}✓ OK{RESET}   {msg}")

def skip(msg):
    global skipped; skipped += 1
    print(f"  {YELLOW}○ SKIP{RESET} {msg}")

def miss(msg):
    global failed; failed += 1
    print(f"  {RED}✗ MISS{RESET} {msg}")


# ═══════════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════════

def find_root(hint=None):
    """Find MEBP project root."""
    if hint:
        p = Path(hint)
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    for d in [Path(__file__).resolve().parent] + list(Path(__file__).resolve().parents):
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    for loc in [
        Path.home() / "Documents" / "GitHub" / "MEBP",
        Path(r"C:\Users\mcghe\OneDrive\Documents\GitHub\MEBP"),
    ]:
        if loc.is_dir() and (loc / "SupportClasses").is_dir():
            return loc
    print(f"{RED}ERROR: Cannot find MEBP root. Pass path as argument.{RESET}")
    sys.exit(1)


def safe_read(path):
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""


def safe_write(path, content, label):
    if path.suffix == ".py":
        try:
            ast.parse(content)
        except SyntaxError as e:
            miss(f"{label}: AST FAIL — {e}")
            return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v728_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    return True


def find_method(content, name):
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
# CHANGE A: XYStageSimulator — Load timing from diagnostic profile
# ═══════════════════════════════════════════════════════════════════

def patch_A_simulator_timing(root):
    """Make simulator load timing from config/xy_diagnostic_profile.json."""
    print(f"\n{CYAN}[A] XYStageSimulator.py — Load timing from diagnostic profile{RESET}")
    path = root / "SupportClasses" / "XYStageSimulator.py"
    content = safe_read(path)
    if not content:
        miss("XYStageSimulator.py not found"); return

    marker = "v7.2.8: load timing from diagnostic profile"
    if marker in content:
        skip("XYStageSimulator already loads from diagnostic profile")
        return

    # ── A1: Add json + pathlib import at top ──
    if "import json" not in content:
        # Insert after "import time"
        content = content.replace(
            "import time\n",
            "import json\nimport time\n",
            1
        )
    if "from pathlib import Path" not in content:
        content = content.replace(
            "import time\n",
            "import time\nfrom pathlib import Path\n",
            1
        )

    # ── A2: Replace the hardcoded PROCESSING_TIMES + add loader function ──
    # Find the PROCESSING_TIMES dict
    pt_pattern = re.compile(
        r'(# Per-command processing times.*?'   # comment before
        r'PROCESSING_TIMES\s*=\s*\{[^}]+\}\s*'  # the dict
        r'PROCESSING_TIME_S\s*=\s*[\d.]+[^\n]*)',  # backward compat line
        re.DOTALL
    )
    pt_match = pt_pattern.search(content)
    if not pt_match:
        # Try just the dict itself
        pt_pattern2 = re.compile(
            r'(PROCESSING_TIMES\s*=\s*\{[^}]+\}\s*'
            r'PROCESSING_TIME_S\s*=\s*[\d.]+[^\n]*)',
            re.DOTALL
        )
        pt_match = pt_pattern2.search(content)

    if not pt_match:
        miss("PROCESSING_TIMES dict not found in XYStageSimulator.py")
        return

    new_timing_block = '''# v7.2.8: load timing from diagnostic profile
# Hardcoded fallback — used only if config/xy_diagnostic_profile.json is missing.
# These defaults are from the 2026-03-09 diagnostic run at 38400 baud.
_DEFAULT_PROCESSING_TIMES = {
    "position":  0.008,
    "move":      0.008,
    "velocity":  0.008,
    "setting":   0.008,
    "stop":      0.005,
    "default":   0.008,
}

# Diagnostic profile path (relative to project root)
_DIAGNOSTIC_PROFILE_PATH = "config/xy_diagnostic_profile.json"


def _load_diagnostic_profile():
    """Load timing constants from xy_diagnostic_profile.json.

    Searches for the profile JSON relative to this file's location,
    walking up to find the project root (directory containing config/).

    Returns:
        (processing_times_dict, baud_rate, stage_info_dict) or defaults.
    """
    # Walk up from this file to find project root
    here = Path(__file__).resolve().parent
    for ancestor in [here] + list(here.parents):
        candidate = ancestor / _DIAGNOSTIC_PROFILE_PATH
        if candidate.exists():
            try:
                with open(candidate) as f:
                    profile = json.load(f)

                # Extract processing times
                derived = profile.get("simulator_derived_constants", {})
                proc_times = {
                    "position": derived.get("processing_time_position_s", 0.008),
                    "move":     derived.get("processing_time_move_s", 0.008),
                    "velocity": derived.get("processing_time_velocity_s", 0.008),
                    "setting":  derived.get("processing_time_setting_s", 0.008),
                    "stop":     derived.get("processing_time_stop_s", 0.005),
                    "default":  derived.get("processing_time_default_s", 0.008),
                }

                # Extract baud rate
                comm = profile.get("communication", {})
                baud = comm.get("baud_rate", DEFAULT_BAUD)

                # Extract stage info
                stage_info = profile.get("stage_info", {})

                logger.info(
                    f"Loaded XY diagnostic profile from {candidate} "
                    f"(baud={baud}, VS={proc_times['velocity']*1000:.0f}ms)"
                )
                return proc_times, baud, stage_info

            except Exception as e:
                logger.warning(f"Failed to load diagnostic profile {candidate}: {e}")
                break

    logger.info("No diagnostic profile found — using default timing constants")
    return dict(_DEFAULT_PROCESSING_TIMES), DEFAULT_BAUD, {}


# Load at module import time so all instances share the same profile
PROCESSING_TIMES, _PROFILE_BAUD, _PROFILE_STAGE_INFO = _load_diagnostic_profile()
PROCESSING_TIME_S = PROCESSING_TIMES.get("default", 0.008)'''

    content = content[:pt_match.start()] + new_timing_block + content[pt_match.end():]

    # ── A3: Update __init__ to use profile baud + stage info ──
    # Find the __init__ and add profile-derived overrides after stage info section
    # We want to override _stage_name, _size_x_mm, _size_y_mm from profile
    stage_info_pattern = re.compile(
        r'(        # ── Stage info .*?\n'
        r'        self\._stage_name = "[^"]+"\n'
        r'        self\._size_x_mm = \d+\n'
        r'        self\._size_y_mm = \d+\n)',
        re.DOTALL
    )
    si_match = stage_info_pattern.search(content)
    if si_match:
        old_si = si_match.group(0)
        new_si = (
            '        # ── Stage info (v7.2.8: from diagnostic profile) ──────────\n'
            '        self._stage_name = _PROFILE_STAGE_INFO.get("stage_name", "H101 Simulator")\n'
            '        self._size_x_mm = _PROFILE_STAGE_INFO.get("size_x_mm", 108)\n'
            '        self._size_y_mm = _PROFILE_STAGE_INFO.get("size_y_mm", 71)\n'
        )
        content = content.replace(old_si, new_si, 1)

    # ── A4: Update __init__ default baud_rate param to use profile ──
    # Change: baud_rate: int = DEFAULT_BAUD → baud_rate: int = _PROFILE_BAUD
    content = re.sub(
        r'(baud_rate:\s*int\s*=\s*)DEFAULT_BAUD',
        r'\g<1>_PROFILE_BAUD',
        content,
        count=1
    )

    # ── A5: Update module docstring to note profile loading ──
    old_docstring_line = 'XY Stage Simulator v5'
    if old_docstring_line in content:
        content = content.replace(
            old_docstring_line,
            'XY Stage Simulator v6 — Diagnostic-profile-driven timing',
            1
        )

    old_timing_comment = (
        'Serial timing at 38400 baud (8N1 = 10 bits/byte = 3840 bytes/s):\n'
        '  TX "G 19300,0\\\\r" (13 bytes)  → 3.4ms\n'
        '  RX "R\\\\r"          (2 bytes)  → 0.5ms\n'
        '  RX "19300.0,0.0,0.0\\\\r" (20 bytes) → 5.2ms\n'
        '  Controller processing: ~2ms\n'
        '  Round-trip position query: ~8ms → max ~125 Hz poll rate'
    )
    new_timing_comment = (
        'Timing loaded from config/xy_diagnostic_profile.json at import time.\n'
        'Diagnostic results (38400 baud, 2026-03-09):\n'
        '  ALL commands: ~12ms round-trip, ~83 Hz burst, ~62.5 Hz sustained\n'
        '  Processing (excl baud delay): ~8ms uniform across P, VS, G, GR, $\n'
        '  VS is NOT slower than P — the old 800ms value was incorrect\n'
        '  Sine-wave tracking: clean to 1.0 Hz (1885 µm/s peak) at 50 Hz VS rate'
    )
    content = content.replace(old_timing_comment, new_timing_comment, 1)

    if safe_write(path, content, "A: simulator timing"):
        ok("XYStageSimulator now loads timing from config/xy_diagnostic_profile.json")


# ═══════════════════════════════════════════════════════════════════
# CHANGE B: Verify diagnostic profile exists
# ═══════════════════════════════════════════════════════════════════

def patch_B_diagnostic_profile(root):
    print(f"\n{CYAN}[B] config/xy_diagnostic_profile.json — Create/Verify{RESET}")
    path = root / "config" / "xy_diagnostic_profile.json"
    if path.exists():
        skip("xy_diagnostic_profile.json already exists")
        return

    # Create the file with diagnostic data from 2026-03-09
    path.parent.mkdir(parents=True, exist_ok=True)
    profile = {
        "_description": "ProScan II communication profile measured 2026-03-09 via proscan_diagnostic.py",
        "_controller": "Prior ProScan II — H117(2MM)PHY stage, TYPE 8",
        "_port": "COM4",
        "_baud": 38400,
        "stage_info": {
            "stage_name": "H117(2MM)PHY",
            "stage_type": 8,
            "size_x_mm": 114,
            "size_y_mm": 75,
            "microsteps_per_micron": 25,
            "limits": "normally_closed"
        },
        "communication": {
            "baud_rate": 38400,
            "tx_terminator": "CR",
            "mode": "standard",
            "firmware_response_to_V": "E,4"
        },
        "round_trip_timing_ms": {
            "_note": "20 samples each at 38400 baud, all commands",
            "position_query_P": {"avg": 12.0, "min": 11.5, "max": 12.2, "stdev": 0.2, "rate_hz": 83.1},
            "stage_pos_PS":     {"avg": 12.0, "min": 11.5, "max": 12.3, "stdev": 0.2, "rate_hz": 83.2},
            "motion_status_$":  {"avg": 12.0, "min": 11.7, "max": 12.2, "stdev": 0.1, "rate_hz": 83.2},
            "stage_status_$S":  {"avg": 12.0, "min": 11.4, "max": 12.4, "stdev": 0.3, "rate_hz": 83.0},
            "query_speed_SMS":  {"avg": 12.1, "min": 11.6, "max": 12.3, "stdev": 0.1, "rate_hz": 82.5},
            "query_accel_SAS":  {"avg": 12.1, "min": 12.0, "max": 12.3, "stdev": 0.1, "rate_hz": 82.4},
            "firmware_V":       {"avg": 12.1, "min": 11.5, "max": 12.3, "stdev": 0.2, "rate_hz": 82.8},
            "query_mode_COMP":  {"avg": 12.0, "min": 11.3, "max": 12.2, "stdev": 0.2, "rate_hz": 83.0}
        },
        "movement_timing_ms": {
            "GR_plus_1um":  {"avg": 12.1, "rate_hz": 82.8},
            "GR_minus_1um": {"avg": 12.1, "rate_hz": 82.6},
            "G_same_pos":   {"avg": 12.2, "rate_hz": 82.0}
        },
        "vs_velocity_timing_ms": {
            "VS_stop":       {"avg": 11.9, "rate_hz": 83.8},
            "VS_100":        {"avg": 12.0, "rate_hz": 83.7},
            "VS_500_diag":   {"avg": 12.0, "rate_hz": 83.1},
            "VS_1000":       {"avg": 11.9, "rate_hz": 84.2},
            "VS_5000":       {"avg": 12.0, "rate_hz": 83.5},
            "direction_alternation": {"avg": 12.0, "min": 9.7, "max": 12.4, "rate_hz": 83.2}
        },
        "timing_breakdown_ms": {
            "_note": "write / controller_wait / total",
            "P":        {"write": 0.1, "wait": 11.8, "total": 11.9},
            "VS_100_0": {"write": 0.1, "wait": 11.4, "total": 11.6},
            "GR_1_0":   {"write": 0.1, "wait": 11.8, "total": 11.9},
            "$":        {"write": 0.1, "wait": 11.9, "total": 12.0}
        },
        "sustained_rate_hz": {
            "_note": "5 second tests",
            "position":  62.5,
            "status":    62.5,
            "vs_update": 62.5
        },
        "sine_wave_geometry": {
            "amplitude_um": 300,
            "vs_rate_hz": 50,
            "fail_threshold_pct": 15,
            "max_pass_freq_hz": 1.0,
            "peak_velocity_um_s": 1885,
            "results": [
                {"freq_hz": 0.10, "rms_um": 4.9,  "max_um": 9.0,   "lag_ms": 30, "pass": True},
                {"freq_hz": 0.20, "rms_um": 9.6,  "max_um": 17.3,  "lag_ms": 30, "pass": True},
                {"freq_hz": 0.30, "rms_um": 14.0, "max_um": 24.6,  "lag_ms": 30, "pass": True},
                {"freq_hz": 0.50, "rms_um": 22.9, "max_um": 40.1,  "lag_ms": 30, "pass": True},
                {"freq_hz": 0.75, "rms_um": 33.6, "max_um": 57.9,  "lag_ms": 30, "pass": True},
                {"freq_hz": 1.00, "rms_um": 44.7, "max_um": 76.2,  "lag_ms": 30, "pass": True},
                {"freq_hz": 1.50, "rms_um": 67.8, "max_um": 111.0, "lag_ms": 35, "pass": False}
            ]
        },
        "simulator_derived_constants": {
            "_note": "Processing time = total_roundtrip - TX_baud_delay - RX_baud_delay. "
                     "At 38400 baud (3840 bytes/s): TX 'P\\r' = 0.52ms, "
                     "RX '23223,20169,0\\r' = 3.6ms. So processing ~8ms.",
            "processing_time_position_s":  0.008,
            "processing_time_move_s":      0.008,
            "processing_time_velocity_s":  0.008,
            "processing_time_setting_s":   0.008,
            "processing_time_stop_s":      0.005,
            "processing_time_default_s":   0.008
        }
    }
    path.write_text(json.dumps(profile, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")
    ok("Created config/xy_diagnostic_profile.json from diagnostic data")


# ═══════════════════════════════════════════════════════════════════
# CHANGE C: Fix ProScan II detection tokens
# ═══════════════════════════════════════════════════════════════════

def patch_C_proscan_ii_detection(root):
    """Fix ProScan II detection: V returns 'E,4', not 'ProScan II'."""
    print(f"\n{CYAN}[C] proscan_ii.json — Fix detection tokens{RESET}")
    path = root / "config" / "controllers" / "proscan_ii.json"
    content = safe_read(path)
    if not content:
        miss("proscan_ii.json not found"); return

    try:
        config = json.loads(content)
    except json.JSONDecodeError as e:
        miss(f"proscan_ii.json JSON parse error: {e}"); return

    detection = config.get("detection", {})
    current_query = detection.get("firmware_query", "V")

    if current_query == "P" and detection.get("response_pattern"):
        skip("proscan_ii.json detection already uses P command with pattern")
        return

    config["detection"] = {
        "wake_command": None,
        "wake_delay_ms": 0,
        "firmware_query": "P",
        "identify_tokens": [","],
        "response_pattern": "^-?\\d+,-?\\d+",
        "notes": "v7.2.8: V returns E,4 on some firmware; P always works. "
                 "Response is x,y,z. Pattern ensures comma-separated numbers."
    }

    new_content = json.dumps(config, indent=4, ensure_ascii=False) + "\n"
    path.write_text(new_content, encoding="utf-8")
    ok("proscan_ii.json: firmware_query V→P, tokens→[','], pattern→'^-?\\d+,-?\\d+'")


# ═══════════════════════════════════════════════════════════════════
# CHANGE D: Fix ProScan III detection tokens
# ═══════════════════════════════════════════════════════════════════

def patch_D_proscan_iii_detection(root):
    print(f"\n{CYAN}[D] proscan_iii.json — Fix overly-broad tokens{RESET}")
    path = root / "config" / "controllers" / "proscan_iii.json"
    content = safe_read(path)
    if not content:
        miss("proscan_iii.json not found"); return

    try:
        config = json.loads(content)
    except json.JSONDecodeError as e:
        miss(f"proscan_iii.json JSON parse error: {e}"); return

    detection = config.get("detection", {})
    current_tokens = detection.get("identify_tokens", [])

    if "E" not in current_tokens:
        skip("proscan_iii.json: bare 'E' token already removed")
        return

    detection["identify_tokens"] = ["ProScan", "III", "R"]
    detection["response_pattern"] = "^(?!E,\\d).*(?:ProScan|\\d+\\.\\d+)"
    detection["notes"] = (
        "v7.2.8: Removed bare 'E' token (matched ProScan II error codes). "
        "Send STAGE first to wake, then V to get firmware string. "
        "response_pattern excludes E,N error format."
    )
    config["detection"] = detection

    new_content = json.dumps(config, indent=4, ensure_ascii=False) + "\n"
    path.write_text(new_content, encoding="utf-8")
    ok("proscan_iii.json: removed bare 'E' from identify_tokens")


# ═══════════════════════════════════════════════════════════════════
# CHANGE E: Fix XYStage.py — CR-aware readline + response_pattern
# ═══════════════════════════════════════════════════════════════════

def patch_E_xystage_detection(root):
    print(f"\n{CYAN}[E] XYStage.py — CR-aware detection + response_pattern{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = safe_read(path)
    if not content:
        miss("XYStage.py not found"); return

    marker = "v7.2.8: CR-aware detection read"
    if marker in content:
        skip("XYStage.py already patched (v7.2.8)")
        return

    # ── E1: Add `import re` if missing ──
    if "\nimport re" not in content and "\nimport re " not in content:
        # Find end of ControllerProtocol import block (handles multi-line parens)
        cp_import = re.search(
            r'from SupportClasses\.ControllerProtocol import \(.*?\)\n',
            content, re.DOTALL
        )
        if not cp_import:
            # Single-line import
            cp_import = re.search(
                r'^from SupportClasses\.ControllerProtocol import [^\n]+\n',
                content, re.MULTILINE
            )
        if cp_import:
            insert_pos = cp_import.end()
            content = content[:insert_pos] + "import re  # v7.2.8: response_pattern matching\n" + content[insert_pos:]
        else:
            # Fallback: after "import time"
            content = content.replace("import time\n", "import time\nimport re  # v7.2.8\n", 1)

    # ── E2: Add _read_response_cr helper before class XYStageManager ──
    helper_func = '''
# v7.2.8: CR-aware detection read
def _read_response_cr(spo, timeout=0.3):
    """Read bytes until CR or LF, with short timeout for detection probes.

    pyserial's readline() reads until LF (\\n), but Prior ProScan II
    terminates responses with CR (\\r) only. This causes readline() to
    block until the full timeout (1s+) on every detection probe.
    """
    old_timeout = spo.timeout
    spo.timeout = timeout
    buf = b""
    try:
        while True:
            ch = spo.read(1)
            if not ch:  # timeout
                break
            if ch in (b"\\r", b"\\n"):
                if buf:  # got data before terminator
                    break
                continue  # skip leading CR/LF
            buf += ch
    except Exception:
        pass
    finally:
        try:
            spo.timeout = old_timeout
        except Exception:
            pass
    return buf.decode("ascii", errors="replace").strip()


'''

    class_match = re.search(r'^class XYStageManager', content, re.MULTILINE)
    if not class_match:
        miss("class XYStageManager not found")
        return

    content = content[:class_match.start()] + helper_func + content[class_match.start():]

    # ── E3: Replace _find_with_protocol method ──
    m = find_method(content, "_find_with_protocol")
    if not m:
        miss("_find_with_protocol method not found")
        return

    new_method = '''    def _find_with_protocol(self, protocol: ControllerProtocol) -> "Optional[serial.Serial]":
        """Try to find a controller matching the given protocol.
        v7.2.8: CR-aware detection read + response_pattern support.
        """
        detection = protocol.get_detection_info()
        wake_cmd = detection.get("wake_command")
        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0
        fw_query = detection.get("firmware_query", "V")
        tokens = detection.get("identify_tokens", [])
        resp_pattern = detection.get("response_pattern")
        baud = protocol.baud_rate

        ports = serial.tools.list_ports.comports()
        for port_info in ports:
            try:
                logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")
                spo = serial.Serial(
                    port_info.device,
                    baudrate=baud,
                    bytesize=protocol.byte_size,
                    timeout=0.5,  # v7.2.8: short timeout for detection
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.1)  # let port settle
                spo.reset_input_buffer()
                spo.reset_output_buffer()

                tx_term = protocol.tx_terminator

                # Wake command (if defined)
                if wake_cmd:
                    spo.write(wake_cmd.encode(protocol.encoding) + tx_term)
                    time.sleep(wake_delay)
                    _read_response_cr(spo, timeout=0.3)  # discard wake response
                    spo.reset_input_buffer()

                # Detection query
                spo.write(fw_query.encode(protocol.encoding) + tx_term)
                time.sleep(0.05)
                response = _read_response_cr(spo, timeout=0.3)  # v7.2.8: CR-aware
                logger.debug(f"Detection response from {port_info.device}: {response!r}")

                if not response:
                    spo.close()
                    continue

                # v7.2.8: Check response_pattern first (more specific)
                if resp_pattern:
                    if re.match(resp_pattern, response):
                        logger.info(
                            f"{protocol.controller_name} found on "
                            f"{port_info.device} @ {baud} baud (pattern match: {response!r})"
                        )
                        spo.timeout = protocol.timeout
                        self._detected_controller = protocol.controller_name
                        return spo
                    else:
                        # Pattern didn't match — skip token check for this protocol
                        spo.close()
                        continue

                # Fallback: token-based check (only if no response_pattern)
                if tokens and any(tok in response for tok in tokens):
                    logger.info(
                        f"{protocol.controller_name} found on "
                        f"{port_info.device} @ {baud} baud (token match: {response!r})"
                    )
                    spo.timeout = protocol.timeout
                    self._detected_controller = protocol.controller_name
                    return spo

                spo.close()
            except (serial.SerialException, UnicodeDecodeError, OSError) as e:
                logger.debug(f"Error on {port_info.device} @ {baud}: {e}")
                continue

        logger.debug(f"{protocol.controller_name} not found on any port")
        return None

'''

    content = content[:m.start()] + new_method + content[m.end():]

    # ── E4: Fix get_current_position readline to use _read_response_cr ──
    gcp = find_method(content, "get_current_position")
    if gcp:
        gcp_text = gcp.group(0)
        if "spo.readline()" in gcp_text and "_read_response_cr" not in gcp_text:
            # Replace multi-line readline().decode().strip() pattern
            new_gcp = re.sub(
                r'response\s*=\s*self\.spo\.readline\(\)\.decode\(\s*'
                r'self\._protocol\.encoding if self\._protocol else "ascii"\s*,\s*'
                r'errors="replace"\s*'
                r'\)\.strip\(\)',
                '# v7.2.8: CR-aware read (Prior sends \\r not \\n)\n'
                '            response = _read_response_cr(self.spo, timeout=0.5)',
                gcp_text
            )
            if new_gcp == gcp_text:
                # Try single-line variant
                new_gcp = re.sub(
                    r'response\s*=\s*self\.spo\.readline\(\)[^\n]+\.strip\(\)',
                    '# v7.2.8: CR-aware read\n'
                    '            response = _read_response_cr(self.spo, timeout=0.5)',
                    gcp_text
                )
            if new_gcp != gcp_text:
                content = content[:gcp.start()] + new_gcp + content[gcp.end():]

    if safe_write(path, content, "E: XYStage detection"):
        ok("XYStage.py: CR-aware detection + response_pattern + readline fix")


# ═══════════════════════════════════════════════════════════════════
# CHANGE F: main.py — Pass controller_json from settings
# ═══════════════════════════════════════════════════════════════════

def patch_F_main_controller_json(root):
    print(f"\n{CYAN}[F] main.py — Pass controller_json from settings{RESET}")
    path = root / "main.py"
    content = safe_read(path)
    if not content:
        miss("main.py not found"); return

    marker = "v7.2.8: Pass controller_json"
    if marker in content:
        skip("main.py already patched (v7.2.8)")
        return

    # Find StageController constructor
    sc_pattern = re.compile(
        r'(controller\s*=\s*StageController\(\s*\n'
        r'\s*simulate_xy\s*=\s*simulate_xy\s*,\s*\n'
        r'\s*simulate_zp\s*=\s*simulate_zp\s*,?\s*\n'
        r'\s*\))',
        re.MULTILINE
    )
    match = sc_pattern.search(content)
    if not match:
        sc_pattern2 = re.compile(
            r'(controller\s*=\s*StageController\([^)]*simulate_xy=simulate_xy[^)]*\))',
            re.DOTALL
        )
        match = sc_pattern2.search(content)

    if not match:
        miss("StageController() constructor call not found in main.py")
        return

    old_text = match.group(0)
    insert_before = match.start()

    controller_json_line = (
        '    # v7.2.8: Pass controller_json from settings for hardware auto-detect\n'
        '    controller_json = settings.get("controller.controller_json", "auto")\n\n'
    )

    # Build new constructor
    new_constructor = old_text.rstrip().rstrip(')')
    if new_constructor.rstrip().endswith(','):
        new_constructor += '\n        controller_json=controller_json,  # v7.2.8: Pass controller_json\n    )'
    else:
        new_constructor += ',\n        controller_json=controller_json,  # v7.2.8: Pass controller_json\n    )'

    content = (
        content[:insert_before]
        + controller_json_line
        + new_constructor
        + content[match.end():]
    )

    if safe_write(path, content, "F: main.py controller_json"):
        ok("main.py: controller_json='auto' passed from settings")


# ═══════════════════════════════════════════════════════════════════
# CHANGE G: StageController.py — Error handling + poll interval
# ═══════════════════════════════════════════════════════════════════

def patch_G_stage_controller(root):
    print(f"\n{CYAN}[G] StageController.py — Error handling + poll interval{RESET}")
    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        miss("StageController.py not found"); return

    changed = False

    # ── G1: Fix poll interval 1.0 → 0.3 ──
    marker_g1 = "v7.2.8: poll interval"
    if marker_g1 not in content:
        if re.search(r'PositionPoller\(poll_interval\s*=\s*1\.0\s*\)', content):
            content = re.sub(
                r'PositionPoller\(poll_interval\s*=\s*1\.0\s*\)',
                'PositionPoller(poll_interval=0.3)  # v7.2.8: poll interval restored',
                content
            )
            changed = True
            ok("PositionPoller poll_interval: 1.0 → 0.3")
        elif "PositionPoller(poll_interval=0.3)" in content:
            skip("G1: poll_interval already 0.3")
        else:
            skip("G1: PositionPoller constructor pattern not matched")
    else:
        skip("G1: poll interval already patched")

    # ── G2: Wrap XY stage connection in try/except ──
    marker_g2 = "v7.2.8: connection error handling"
    if marker_g2 not in content:
        xy_construct = re.search(
            r'(            self\.xy_stage = XYStageManager\(\n'
            r'                simulate=self\.simulate_xy,\n'
            r'                controller_json=self\.controller_json,\n'
            r'            \))',
            content
        )
        if xy_construct:
            old_text = xy_construct.group(0)
            new_text = (
                '            # v7.2.8: connection error handling\n'
                '            try:\n'
                '                self.xy_stage = XYStageManager(\n'
                '                    simulate=self.simulate_xy,\n'
                '                    controller_json=self.controller_json,\n'
                '                )\n'
                '            except (ConnectionError, ImportError, OSError) as e:\n'
                '                logger.error(f"XY stage connection failed: {e}")\n'
                '                self.xy_stage = None\n'
                '                if self.on_disconnect:\n'
                '                    self.on_disconnect("XY")\n'
                '                return'
            )
            content = content.replace(old_text, new_text, 1)
            changed = True
            ok("connect_stages: XY wrapped in try/except")
        else:
            skip("G2: XYStageManager constructor exact pattern not found — review manually")
    else:
        skip("G2: already patched")

    # ── G3: Wrap ZP stage connection in try/except ──
    marker_g3 = "v7.2.8: ZP connection error handling"
    if marker_g3 not in content:
        zp_construct = re.search(
            r'(            self\.zp_stage = ZPStageManager\(simulate=self\.simulate_zp\))',
            content
        )
        if zp_construct:
            old_text = zp_construct.group(0)
            new_text = (
                '            # v7.2.8: ZP connection error handling\n'
                '            try:\n'
                '                self.zp_stage = ZPStageManager(simulate=self.simulate_zp)\n'
                '            except (ConnectionError, ImportError, OSError) as e:\n'
                '                logger.error(f"ZP stage connection failed: {e}")\n'
                '                self.zp_stage = None\n'
                '                if self.on_disconnect:\n'
                '                    self.on_disconnect("ZP")\n'
                '                return'
            )
            content = content.replace(old_text, new_text, 1)
            changed = True
            ok("connect_stages: ZP wrapped in try/except")
        else:
            skip("G3: ZPStageManager constructor pattern not found")
    else:
        skip("G3: already patched")

    if changed:
        safe_write(path, content, "G: StageController")


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{BOLD}{'═' * 60}{RESET}")
    print(f"{BOLD} MEBP v7.2.8 — Diagnostic-Based Communication Fixes{RESET}")
    print(f"{BOLD}{'═' * 60}{RESET}")

    hint = sys.argv[1] if len(sys.argv) > 1 else None
    root = find_root(hint)
    print(f"\nProject root: {root}")

    patch_A_simulator_timing(root)
    patch_B_diagnostic_profile(root)
    patch_C_proscan_ii_detection(root)
    patch_D_proscan_iii_detection(root)
    patch_E_xystage_detection(root)
    patch_F_main_controller_json(root)
    patch_G_stage_controller(root)

    print(f"\n{'═' * 60}")
    print(f"  {GREEN}Applied: {applied}{RESET}  |  "
          f"{YELLOW}Skipped: {skipped}{RESET}  |  "
          f"{RED}Failed: {failed}{RESET}")
    print(f"{'═' * 60}\n")

    if failed > 0:
        print(f"{RED}⚠ Some patches failed — review output above.{RESET}")
        sys.exit(1)
    else:
        print(f"{GREEN}✓ All patches applied successfully.{RESET}")


if __name__ == "__main__":
    main()
