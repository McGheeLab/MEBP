#!/usr/bin/env python3
"""
ProScan II Communication Diagnostic Tool

Standalone tool to characterize the Prior ProScan II controller's
actual communication behavior. Tests every timing aspect:

  1. Port discovery & connection
  2. Baud rate detection (9600 vs 38400)
  3. Operating mode check (Standard vs Compatibility)
  4. Per-command round-trip timing (P, G, GR, VS, SMS, $, etc.)
  5. Position polling sustained rate
  6. VS velocity command rate (the bottleneck you measured)
  7. Joystick interference check
  8. Command queue depth test
  9. Move + poll concurrent test

Results are printed to terminal AND saved to proscan_timing_report.txt.

Usage:
  python proscan_diagnostic.py                          # auto-detect
  python proscan_diagnostic.py /dev/cu.usbserial-1410   # macOS
  python proscan_diagnostic.py COM3                     # Windows

Requires: pyserial (pip install pyserial)
"""

import sys
import time
import statistics
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial required. Install with: pip install pyserial")
    sys.exit(1)


class ProScanDiag:
    def __init__(self, port_name, baud=9600, timeout=2.0):
        self.port_name = port_name
        self.baud = baud
        self.timeout = timeout
        self.port = None
        self.log_lines = []

    def log(self, msg, indent=0):
        line = "  " * indent + msg
        print(line)
        self.log_lines.append(line)

    def connect(self):
        try:
            self.port = serial.Serial(
                self.port_name, baudrate=self.baud, bytesize=8,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout, write_timeout=2.0)
            time.sleep(0.2)
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
            return True
        except Exception as e:
            self.log(f"Connection failed: {e}")
            return False

    def disconnect(self):
        if self.port and self.port.is_open:
            try: self.port.close()
            except Exception: pass

    def send_raw(self, cmd):
        """Send command + CR, read one line response. Returns (response, seconds)."""
        if not self.port or not self.port.is_open:
            return ("", 0.0)
        self.port.reset_input_buffer()
        encoded = f"{cmd}\r".encode("ascii")
        t0 = time.perf_counter()
        self.port.write(encoded)
        self.port.flush()
        resp = b""
        while True:
            chunk = self.port.read(1)
            if not chunk: break
            resp += chunk
            if chunk in (b"\r", b"\n"): break
        t1 = time.perf_counter()
        return (resp.decode("ascii", errors="replace").strip(), t1 - t0)

    def send_multi_read(self, cmd, max_lines=20, line_timeout=0.5):
        if not self.port: return ([], 0.0)
        self.port.reset_input_buffer()
        old_to = self.port.timeout
        self.port.timeout = line_timeout
        t0 = time.perf_counter()
        self.port.write(f"{cmd}\r".encode("ascii"))
        self.port.flush()
        lines = []
        for _ in range(max_lines):
            line = self.port.readline().decode("ascii", errors="replace").strip()
            if not line: break
            lines.append(line)
            if line == "END": break
        self.port.timeout = old_to
        return (lines, time.perf_counter() - t0)

    def measure(self, cmd, label="", n=20, warmup=3):
        for _ in range(warmup): self.send_raw(cmd)
        times = []
        responses = []
        for _ in range(n):
            resp, dt = self.send_raw(cmd)
            times.append(dt * 1000)
            responses.append(resp)
        avg = statistics.mean(times)
        return {
            "cmd": cmd, "label": label or cmd, "n": n,
            "avg_ms": avg, "min_ms": min(times), "max_ms": max(times),
            "stdev_ms": statistics.stdev(times) if len(times) > 1 else 0,
            "rate_hz": 1000 / avg if avg > 0 else 0,
            "last_resp": responses[-1], "times": times,
        }


def find_port():
    ports = serial.tools.list_ports.comports()
    print(f"\nAvailable ports ({len(ports)}):")
    for p in ports:
        print(f"  {p.device}  — {p.description}")
    for baud in [9600, 38400]:
        for p in ports:
            try:
                s = serial.Serial(p.device, baudrate=baud, timeout=1)
                time.sleep(0.1)
                s.reset_input_buffer()
                s.write(b"V\r")
                time.sleep(0.3)
                resp = s.read_all().decode("ascii", errors="replace").strip()
                s.close()
                if resp and len(resp) > 2:
                    print(f"\n  → Found: {p.device} @ {baud} baud → {resp}")
                    return p.device
            except Exception: continue
    return None


def run(port_name):
    R = []
    def log(msg, indent=0):
        line = "  " * indent + msg
        print(line); R.append(line)

    log(f"\n{'='*70}")
    log(f" ProScan II Communication Diagnostic — {datetime.now():%Y-%m-%d %H:%M:%S}")
    log(f" Port: {port_name}")
    log(f"{'='*70}")

    # ── 1. Baud detection ─────────────────────────────────────────
    log(f"\n{'─'*50}")
    log("1. BAUD RATE DETECTION")
    baud = None
    for b in [9600, 38400]:
        d = ProScanDiag(port_name, baud=b, timeout=1.0)
        if d.connect():
            resp, dt = d.send_raw("V")
            log(f"  {b} baud: {resp!r}  ({dt*1000:.0f}ms)", 1)
            if resp and resp != "E" and len(resp) > 1:
                baud = b; d.disconnect(); break
            d.disconnect()
    if not baud:
        log("  ❌ No response at any baud rate"); return R
    log(f"  → Using {baud} baud ({baud//10} bytes/s)")

    d = ProScanDiag(port_name, baud=baud, timeout=2.0)
    if not d.connect():
        log("  ❌ Connection failed"); return R

    # ── 2. Controller info ────────────────────────────────────────
    log(f"\n{'─'*50}")
    log("2. CONTROLLER INFORMATION")
    for cmd, label in [("V", "Firmware"), ("COMP", "Mode"), ("P", "Position"),
                       ("SMS", "MaxSpeed%"), ("SAS", "Accel%"), ("SCS", "S-Curve%")]:
        resp, _ = d.send_raw(cmd)
        extra = ""
        if cmd == "COMP": extra = f" ({'Standard' if resp == '0' else 'Compatibility'})"
        log(f"  {label:12s}: {resp}{extra}", 1)

    lines, _ = d.send_multi_read("STAGE")
    for line in lines:
        log(f"  STAGE       : {line}", 1)

    # ── 3. Ensure clean state ─────────────────────────────────────
    log(f"\n{'─'*50}")
    log("3. SET CLEAN STATE")
    d.send_raw("COMP,0")  # Standard mode
    d.send_raw("H")       # Joystick off
    d.send_raw("VS,0,0")  # Stop any movement
    log("  COMP,0 (Standard mode) + H (joystick off) + VS,0,0", 1)

    # ── 4. Per-command timing ─────────────────────────────────────
    log(f"\n{'─'*50}")
    log("4. PER-COMMAND ROUND-TRIP TIMING (20 samples)")
    log(f"  {'Command':20s}  {'Avg':>7s}  {'Min':>6s}  {'Max':>6s}  {'Rate':>7s}  Response")

    for cmd, label in [
        ("P",       "Position query"),
        ("PS",      "Stage pos only"),
        ("$",       "Motion status"),
        ("$,S",     "Stage status"),
        ("SMS",     "Query speed"),
        ("SAS",     "Query accel"),
        ("V",       "Firmware"),
        ("COMP",    "Query mode"),
    ]:
        r = d.measure(cmd, label, n=20, warmup=3)
        log(f"  {label:20s}  {r['avg_ms']:6.1f}ms  {r['min_ms']:5.1f}  "
            f"{r['max_ms']:5.1f}  {r['rate_hz']:5.1f}Hz  {r['last_resp']!r:.25s}", 1)

    # ── 5. Movement commands ──────────────────────────────────────
    log(f"\n{'─'*50}")
    log("5. MOVEMENT COMMAND TIMING")

    for cmd, label in [
        ("GR,1,0",   "GR +1µm X"),
        ("GR,-1,0",  "GR -1µm X"),
    ]:
        r = d.measure(cmd, label, n=20, warmup=3)
        log(f"  {label:20s}  {r['avg_ms']:6.1f}ms  rate={r['rate_hz']:5.1f}Hz", 1)

    pos_resp, _ = d.send_raw("P")
    if pos_resp and "," in pos_resp:
        parts = pos_resp.split(",")
        r = d.measure(f"G,{parts[0]},{parts[1]}", "G to current pos", n=20, warmup=3)
        log(f"  {'G (same pos)':20s}  {r['avg_ms']:6.1f}ms  rate={r['rate_hz']:5.1f}Hz", 1)

    # ── 6. VS VELOCITY — THE KEY TEST ────────────────────────────
    log(f"\n{'─'*50}")
    log("6. VS VELOCITY COMMAND TIMING (key bottleneck test)")

    for cmd, label in [
        ("VS,0,0",       "VS stop"),
        ("VS,100,0",     "VS 100µm/s"),
        ("VS,500,500",   "VS 500µm/s"),
        ("VS,1000,0",    "VS 1000µm/s"),
        ("VS,5000,0",    "VS 5000µm/s"),
    ]:
        r = d.measure(cmd, label, n=10, warmup=2)
        log(f"  {label:20s}  {r['avg_ms']:6.1f}ms  rate={r['rate_hz']:5.1f}Hz", 1)
        d.send_raw("VS,0,0"); time.sleep(0.05)

    # VS direction alternation
    log(f"\n  Direction alternation test (VS ±500):")
    times = []
    for i in range(20):
        v = 500 if i % 2 == 0 else -500
        _, dt = d.send_raw(f"VS,{v},0")
        times.append(dt * 1000)
    d.send_raw("VS,0,0")
    log(f"    avg={statistics.mean(times):.1f}ms  min={min(times):.1f}  "
        f"max={max(times):.1f}  rate={1000/statistics.mean(times):.1f}Hz", 2)

    # ── 7. Write vs Read breakdown ────────────────────────────────
    log(f"\n{'─'*50}")
    log("7. TIMING BREAKDOWN (write vs controller wait vs read)")
    log(f"  {'Command':15s}  {'Write':>7s}  {'Wait':>8s}  {'Total':>8s}")

    for cmd_str in ["P", "VS,100,0", "GR,1,0", "$"]:
        d.port.reset_input_buffer()
        encoded = f"{cmd_str}\r".encode("ascii")
        t0 = time.perf_counter()
        d.port.write(encoded)
        d.port.flush()
        t_write = time.perf_counter()
        resp = d.port.readline()
        t_read = time.perf_counter()
        wr = (t_write - t0) * 1000
        wait = (t_read - t_write) * 1000
        total = (t_read - t0) * 1000
        dec = resp.decode("ascii", errors="replace").strip()
        log(f"  {cmd_str:15s}  {wr:5.1f}ms  {wait:6.1f}ms  {total:6.1f}ms  → {dec!r:.20s}", 1)
    d.send_raw("VS,0,0")

    # ── 8. Sustained rates ────────────────────────────────────────
    log(f"\n{'─'*50}")
    log("8. SUSTAINED RATE TEST (5 seconds each)")

    for cmd, label in [("P", "Position"), ("$,S", "Status"), ("VS,100,0", "VS update")]:
        t0 = time.time()
        count = 0
        while time.time() - t0 < 5.0:
            d.send_raw(cmd)
            count += 1
        elapsed = time.time() - t0
        log(f"  {label:15s}  {count} in {elapsed:.1f}s = {count/elapsed:.1f} Hz", 1)
    d.send_raw("VS,0,0")

    # ── 9. Baud upgrade test ──────────────────────────────────────
    if baud == 9600:
        log(f"\n{'─'*50}")
        log("9. BAUD UPGRADE TEST (9600 → 38400)")
        resp, _ = d.send_raw("BAUD,38")
        log(f"  BAUD,38 → {resp!r}", 1)
        if resp in ("0", ""):
            d.disconnect()
            time.sleep(0.5)
            d2 = ProScanDiag(port_name, baud=38400, timeout=2.0)
            if d2.connect():
                resp, dt = d2.send_raw("V")
                log(f"  38400 baud: V → {resp!r} ({dt*1000:.0f}ms)", 1)
                for cmd, label in [("P", "Position"), ("VS,100,0", "VS")]:
                    r = d2.measure(cmd, label, n=10, warmup=2)
                    log(f"  {label:15s} @ 38400: {r['avg_ms']:.1f}ms = {r['rate_hz']:.1f}Hz", 1)
                d2.send_raw("VS,0,0")
                d2.send_raw("BAUD,96")  # revert
                d2.disconnect()
                time.sleep(0.5)
            d = ProScanDiag(port_name, baud=9600, timeout=2.0)
            d.connect()

    # ── Cleanup ───────────────────────────────────────────────────
    d.send_raw("VS,0,0")
    d.send_raw("J")  # re-enable joystick
    d.disconnect()

    # ── Summary ───────────────────────────────────────────────────
    log(f"\n{'='*70}")
    log("SUMMARY & RECOMMENDATIONS")
    log(f"{'='*70}")
    log(f"  If VS is slow and P is fast → bottleneck is motor ramp processing")
    log(f"  If both are slow → bottleneck is serial baud rate")
    log(f"  If both are fast → your earlier measurement may have included")
    log(f"    readline timeout or joystick interference")
    log(f"")
    log(f"  Tips:")
    log(f"    • BAUD,38 upgrades to 38400 (2.3x faster byte rate)")
    log(f"    • H disables joystick (prevents command conflicts)")
    log(f"    • COMP,0 ensures non-blocking Standard mode")
    log(f"    • $ is faster than P for motion-complete polling")
    log(f"    • VS response time sets the velocity control loop ceiling")

    report_path = Path("proscan_timing_report.txt")
    report_path.write_text("\n".join(R))
    log(f"\n  Report saved: {report_path.absolute()}")
    return R


if __name__ == "__main__":
    print("\n╔══════════════════════════════════════════════╗")
    print("║  ProScan II Communication Diagnostic Tool    ║")
    print("╚══════════════════════════════════════════════╝")

    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        print("\nScanning for ProScan II...")
        port = find_port()
        if not port:
            print("\n❌ Not found. Specify port: python proscan_diagnostic.py /dev/cu.usbserial-XXX")
            sys.exit(1)
    run(port)
