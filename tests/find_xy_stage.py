#!/usr/bin/env python3
"""
Quick XY stage finder — bypasses protocol detection to diagnose why
auto-detect fails. Probes every COM port with P and V commands.

Run: python tests/find_xy_stage.py
"""

import sys
import time
from pathlib import Path

# Find project root
_SCRIPT_DIR = Path(__file__).resolve().parent
for p in [_SCRIPT_DIR, _SCRIPT_DIR.parent, _SCRIPT_DIR.parent.parent, Path.cwd()]:
    if (p / "SupportClasses").is_dir():
        _ROOT = p
        break
else:
    _ROOT = Path.cwd()

sys.path.insert(0, str(_ROOT))

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed")
    sys.exit(1)

import json


def read_cr(spo, timeout=0.3):
    """Read until CR or LF with short timeout."""
    old = spo.timeout
    spo.timeout = timeout
    buf = b""
    try:
        while True:
            ch = spo.read(1)
            if not ch:
                break
            if ch in (b"\r", b"\n"):
                if buf:
                    break
                continue
            buf += ch
    except Exception:
        pass
    finally:
        spo.timeout = old
    return buf.decode("ascii", errors="replace").strip()


print("=" * 65)
print("  XY Stage Finder — Manual Port Probe")
print("=" * 65)

# 1. Show current proscan_ii.json detection config
print("\n[1] Current proscan_ii.json detection config:")
json_path = _ROOT / "config" / "controllers" / "proscan_ii.json"
if json_path.exists():
    with open(json_path) as f:
        cfg = json.load(f)
    det = cfg.get("detection", {})
    print(f"    firmware_query:   {det.get('firmware_query', '???')}")
    print(f"    identify_tokens:  {det.get('identify_tokens', '???')}")
    print(f"    response_pattern: {det.get('response_pattern', '???')}")
    print(f"    wake_command:     {det.get('wake_command', '???')}")
    
    fw_query = det.get("firmware_query", "V")
    if fw_query == "V":
        print("    ⚠️  firmware_query is 'V' — ProScan II returns 'E,4' to V!")
        print("       Should be 'P' (position query, always works)")
else:
    print(f"    ⚠️  File not found: {json_path}")
    fw_query = "V"

# Also check proscan_iii.json
json3_path = _ROOT / "config" / "controllers" / "proscan_iii.json"
if json3_path.exists():
    with open(json3_path) as f:
        cfg3 = json.load(f)
    det3 = cfg3.get("detection", {})
    print(f"\n    proscan_iii.json firmware_query: {det3.get('firmware_query', '???')}")
    print(f"    proscan_iii.json response_pattern: {det3.get('response_pattern', '???')}")

# 2. List all COM ports
print("\n[2] Available COM ports:")
ports = list(serial.tools.list_ports.comports())
if not ports:
    print("    No COM ports found!")
    sys.exit(1)

for p in ports:
    print(f"    {p.device:10s}  {p.description}  [{p.hwid}]")

# 3. Probe each port
print("\n[3] Probing each port (38400 baud):")

for port_info in ports:
    dev = port_info.device
    print(f"\n  --- {dev} ({port_info.description}) ---")
    
    try:
        spo = serial.Serial(dev, baudrate=38400, timeout=0.5)
        time.sleep(0.1)
        spo.reset_input_buffer()
        spo.reset_output_buffer()
    except Exception as e:
        print(f"    Cannot open: {e}")
        continue

    # Try P command (position query — works on all ProScan)
    try:
        spo.write(b"P\r")
        time.sleep(0.05)
        resp_p = read_cr(spo, timeout=0.3)
        print(f"    P  → {resp_p!r}")
    except Exception as e:
        resp_p = ""
        print(f"    P  → ERROR: {e}")

    time.sleep(0.05)
    spo.reset_input_buffer()

    # Try V command (firmware — ProScan II returns E,4)
    try:
        spo.write(b"V\r")
        time.sleep(0.05)
        resp_v = read_cr(spo, timeout=0.3)
        print(f"    V  → {resp_v!r}")
    except Exception as e:
        resp_v = ""
        print(f"    V  → ERROR: {e}")

    time.sleep(0.05)
    spo.reset_input_buffer()

    # Try COMP command (compatibility mode — reliable on ProScan II)
    try:
        spo.write(b"COMP\r")
        time.sleep(0.05)
        resp_comp = read_cr(spo, timeout=0.3)
        print(f"    COMP → {resp_comp!r}")
    except Exception as e:
        resp_comp = ""
        print(f"    COMP → ERROR: {e}")

    time.sleep(0.05)
    spo.reset_input_buffer()

    # Try M115 (Marlin firmware query)
    try:
        spo.write(b"\nM115\n")
        time.sleep(0.2)
        resp_m = spo.read_all().decode("utf-8", errors="replace").strip()[:100]
        print(f"    M115 → {resp_m!r}")
    except Exception as e:
        resp_m = ""
        print(f"    M115 → ERROR: {e}")

    # Classify
    is_proscan = False
    if resp_p and "," in resp_p:
        # Check if it looks like comma-separated numbers
        import re
        if re.match(r'^-?\d+,-?\d+', resp_p):
            is_proscan = True
            print(f"    ✅ PROSCAN DETECTED (P response: {resp_p})")
            
            # Check if auto-detect pattern would match
            pattern = det.get("response_pattern", "")
            if pattern:
                if re.match(pattern, resp_p):
                    print(f"    ✅ Pattern '{pattern}' matches response")
                else:
                    print(f"    ❌ Pattern '{pattern}' does NOT match '{resp_p}'")
                    print(f"       This is why auto-detect fails!")
    
    if "FIRMWARE_NAME" in resp_m:
        print(f"    ✅ MARLIN DETECTED")

    if not is_proscan and "FIRMWARE_NAME" not in resp_m:
        print(f"    ⬜ Not ProScan or Marlin")

    spo.close()

# 4. Summary
print(f"\n{'='*65}")
print("  If ProScan was detected but auto-detect still fails,")
print("  check that proscan_ii.json detection config matches above.")
print(f"{'='*65}")
