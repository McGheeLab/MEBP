#!/usr/bin/env python3
"""
Replicate _auto_detect_controller step-by-step with verbose logging.
Shows exactly what happens at each stage of detection.
"""

import re
import sys
import time
from pathlib import Path

for p in [Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent, Path.cwd()]:
    if (p / "SupportClasses").is_dir():
        ROOT = p; break
else:
    ROOT = Path.cwd()
sys.path.insert(0, str(ROOT))

import serial
import serial.tools.list_ports

# Import the REAL _read_response_cr from XYStage
from SupportClasses.XYStage import _read_response_cr
from SupportClasses.ControllerProtocol import (
    ControllerProtocol, discover_controller_files, DEFAULT_CONTROLLERS_DIR,
)

print("=" * 65)
print("  Auto-Detect Step-by-Step Debug")
print("=" * 65)
print(f"  ROOT: {ROOT}")
print(f"  DEFAULT_CONTROLLERS_DIR: {DEFAULT_CONTROLLERS_DIR}")

# 1. Find JSON files
ctrl_dir = Path(DEFAULT_CONTROLLERS_DIR)
print(f"\n[1] Controller directory: {ctrl_dir.resolve()}")
print(f"    Exists: {ctrl_dir.exists()}")

json_files = discover_controller_files()
print(f"    JSON files found: {len(json_files)}")
for f in json_files:
    print(f"      {f.name}")

if not json_files:
    # Try resolving from ROOT
    alt_dir = ROOT / "config" / "controllers"
    print(f"\n    Trying alternate: {alt_dir}")
    if alt_dir.exists():
        alt_files = sorted(alt_dir.glob("*.json"))
        print(f"    Found {len(alt_files)} files there")
        json_files = alt_files
    else:
        print("    Not found either!")
        sys.exit(1)

# 2. List ports
ports = list(serial.tools.list_ports.comports())
print(f"\n[2] COM ports: {len(ports)}")
for p in ports:
    print(f"    {p.device:8s}  {p.description}")

# 3. Try each protocol on each port
for json_path in json_files:
    print(f"\n{'─'*65}")
    protocol = ControllerProtocol.load(json_path)
    det = protocol.get_detection_info()

    wake_cmd = det.get("wake_command")
    wake_delay = det.get("wake_delay_ms", 100) / 1000.0
    fw_query = det.get("firmware_query", "V")
    tokens = det.get("identify_tokens", [])
    resp_pattern = det.get("response_pattern")
    baud = protocol.baud_rate
    tx_term = protocol.tx_terminator

    print(f"  Protocol: {protocol.controller_name}")
    print(f"    fw_query={fw_query!r}  wake={wake_cmd!r}  baud={baud}")
    print(f"    tx_term={tx_term!r}  tokens={tokens}  pattern={resp_pattern!r}")

    for port_info in ports:
        dev = port_info.device
        print(f"\n    [{dev}] Opening at {baud} baud...", end=" ")

        try:
            spo = serial.Serial(
                dev, baudrate=baud, bytesize=protocol.byte_size,
                timeout=0.5, stopbits=serial.STOPBITS_ONE,
            )
            print("OK")
        except Exception as e:
            print(f"FAILED: {e}")
            continue

        time.sleep(0.1)
        spo.reset_input_buffer()
        spo.reset_output_buffer()

        # Wake
        if wake_cmd:
            cmd_bytes = wake_cmd.encode(protocol.encoding) + tx_term
            print(f"    [{dev}] Wake: sending {cmd_bytes!r}")
            spo.write(cmd_bytes)
            time.sleep(wake_delay)
            wake_resp = _read_response_cr(spo, timeout=0.3)
            print(f"    [{dev}] Wake response (discarded): {wake_resp!r}")
            spo.reset_input_buffer()

        # Detection query
        cmd_bytes = fw_query.encode(protocol.encoding) + tx_term
        print(f"    [{dev}] Query: sending {cmd_bytes!r}")
        spo.write(cmd_bytes)
        time.sleep(0.05)
        response = _read_response_cr(spo, timeout=0.3)
        print(f"    [{dev}] Response: {response!r} (len={len(response)})")

        if not response:
            print(f"    [{dev}] → SKIP: empty response")
            spo.close()
            continue

        # Pattern check
        if resp_pattern:
            match = re.match(resp_pattern, response)
            if match:
                print(f"    [{dev}] → MATCH! Pattern '{resp_pattern}' matched")
                print(f"    [{dev}] ✅ {protocol.controller_name} DETECTED")
                spo.close()
            else:
                print(f"    [{dev}] → NO MATCH: '{response}' vs pattern '{resp_pattern}'")
                spo.close()
            continue

        # Token check
        if tokens:
            matched_tokens = [t for t in tokens if t in response]
            if matched_tokens:
                print(f"    [{dev}] → TOKEN MATCH: {matched_tokens}")
                print(f"    [{dev}] ✅ {protocol.controller_name} DETECTED")
            else:
                print(f"    [{dev}] → NO TOKEN MATCH: {response!r} vs {tokens}")
            spo.close()
            continue

        print(f"    [{dev}] → No pattern or tokens defined")
        spo.close()

print(f"\n{'='*65}")
print("  Done. Check above for MATCH/NO MATCH to see why detection fails.")
print(f"{'='*65}")
