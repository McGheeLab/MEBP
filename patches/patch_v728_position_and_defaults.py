#!/usr/bin/env python3
"""
MEBP v7.2.8 Session 2 — Fix position display + settings defaults.

Fixes:
  1. XYStage.get_current_position() — atomic lock around write+read
  2. XYStage.get_firmware_version() — same atomic fix
  3. Settings.py — defaults simulate_xy/zp = False
  4. main.py — defaults simulate = False
  5. settings.json — update if simulate still True
"""

import ast, json, re, shutil, sys
from datetime import datetime
from pathlib import Path

GREEN="\033[92m"; RED="\033[91m"; YELLOW="\033[93m"; CYAN="\033[96m"; RESET="\033[0m"; BOLD="\033[1m"
applied=0; skipped=0; failed=0
def ok(m): global applied; applied+=1; print(f"  {GREEN}✓ OK{RESET}   {m}")
def skip(m): global skipped; skipped+=1; print(f"  {YELLOW}○ SKIP{RESET} {m}")
def miss(m): global failed; failed+=1; print(f"  {RED}✗ MISS{RESET} {m}")

def find_root(hint=None):
    if hint:
        p=Path(hint)
        if (p/"SupportClasses").is_dir(): return p
    for d in [Path(__file__).resolve().parent]+list(Path(__file__).resolve().parents):
        if (d/"SupportClasses").is_dir() and (d/"gui").is_dir(): return d
    for loc in [Path.home()/"Documents"/"GitHub"/"MEBP", Path(r"C:\Users\mcghe\OneDrive\Documents\GitHub\MEBP")]:
        if loc.is_dir() and (loc/"SupportClasses").is_dir(): return loc
    print(f"{RED}Cannot find root.{RESET}"); sys.exit(1)

def read_n(path): return path.read_text(encoding="utf-8").replace("\r\n","\n")
def write_c(path,content,label):
    if path.suffix==".py":
        try: ast.parse(content)
        except SyntaxError as e:
            miss(f"{label}: AST FAIL — {e}")
            path.with_suffix(".debug_v728s2.py").write_text(content,encoding="utf-8")
            return False
    ts=datetime.now().strftime("%Y%m%d_%H%M%S")
    bk=path.with_suffix(f".bak_v728s2_{ts}")
    if path.exists(): shutil.copy2(path,bk)
    path.write_bytes(content.replace("\n","\r\n").encode("utf-8"))
    return True


def fix_1_position_display(root):
    """Make get_current_position atomic: lock → write → read → unlock."""
    print(f"\n{CYAN}[1] XYStage.py — Atomic position query{RESET}")
    path = root/"SupportClasses"/"XYStage.py"
    content = read_n(path)

    marker = "v7.2.8s2: atomic position query"
    if marker in content:
        skip("Already patched"); return

    # Verify _read_response_cr exists (from patch E)
    if "_read_response_cr" not in content:
        miss("_read_response_cr not found — run patch_v728_fix_EF.py first")
        return

    # Find and replace get_current_position
    # The method should use the serial lock for the entire write+read cycle
    old_gcp_start = '    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":'
    if old_gcp_start not in content:
        # Try unquoted version
        old_gcp_start = '    def get_current_position(self) -> tuple[float | None, float | None, float | None]:'
    if old_gcp_start not in content:
        miss("get_current_position signature not found")
        return

    # Find the full method (until next def at same indent)
    start_idx = content.index(old_gcp_start)
    after = start_idx + 10
    next_def = re.search(r'\n    @|\n    def ', content[after:])
    if next_def:
        end_idx = after + next_def.start()
    else:
        miss("Could not find end of get_current_position")
        return

    new_gcp = '''    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":
        """Query stage position.
        v7.2.8s2: atomic position query — lock protects write+read from
        concurrent JogHandler/PositionPoller contention.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)
        try:
            with self._serial_lock:
                self._send_protocol_command("position_query", fallback_cmd="P")
                response = _read_response_cr(self.spo, timeout=0.5)
            return self._parse_position_response(response)
        except Exception as e:
            logger.debug(f"XY position query error: {e}")
            return (None, None, None)

'''
    content = content[:start_idx] + new_gcp + content[end_idx:]

    # Also fix get_firmware_version the same way
    old_fw_start = '    def get_firmware_version(self) -> Optional[str]:'
    if old_fw_start in content:
        fw_idx = content.index(old_fw_start)
        after_fw = fw_idx + 10
        next_fw_def = re.search(r'\n    def ', content[after_fw:])
        if next_fw_def:
            fw_end = after_fw + next_fw_def.start()
            new_fw = '''    def get_firmware_version(self) -> Optional[str]:
        """Query controller firmware version.
        v7.2.8s2: atomic with serial lock + CR-aware read.
        """
        if self.simulate:
            return "Simulator v1.0"
        try:
            with self._serial_lock:
                self._send_protocol_command("firmware_version", fallback_cmd="V")
                response = _read_response_cr(self.spo, timeout=0.5)
            return response if response else None
        except Exception as e:
            logger.debug(f"Firmware version query error: {e}")
            return None

'''
            content = content[:fw_idx] + new_fw + content[fw_end:]

    # Also fix send_command to use lock for hardware write
    old_send = '''        try:
            if self._protocol:
                encoded = command.encode(self._protocol.encoding) + self._protocol.tx_terminator
            else:
                encoded = f"{command}\\r\\n".encode("ascii")
            self.spo.write(encoded)
            return None
        except (Exception,) as e:
            logger.error(f"XY send_command error: {e}")
            return None'''

    new_send = '''        try:
            if self._protocol:
                encoded = command.encode(self._protocol.encoding) + self._protocol.tx_terminator
            else:
                encoded = f"{command}\\r\\n".encode("ascii")
            # v7.2.8s2: lock is acquired by caller (get_current_position etc.)
            # for atomic write+read. Bare writes (VS, G) don't need response.
            self.spo.write(encoded)
            return None
        except (Exception,) as e:
            logger.error(f"XY send_command error: {e}")
            return None'''

    if old_send in content:
        content = content.replace(old_send, new_send, 1)

    # Also wrap move_stage_at_velocity in lock (VS commands sent from jog handler)
    old_vs = '''    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        """Set XY velocity (continuous jog mode)."""
        self._send_protocol_command(
            "set_velocity",
            fallback_cmd=f"VS,{vx},{vy}",
            vx=vx, vy=vy,
        )'''
    new_vs = '''    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        """Set XY velocity (continuous jog mode).
        v7.2.8s2: serial lock prevents collision with position polling.
        """
        with self._serial_lock:
            self._send_protocol_command(
                "set_velocity",
                fallback_cmd=f"VS,{vx},{vy}",
                vx=vx, vy=vy,
            )'''
    if old_vs in content:
        content = content.replace(old_vs, new_vs, 1)

    if write_c(path, content, "XYStage position"):
        ok("get_current_position + move_stage_at_velocity + get_firmware_version: atomic with serial lock")


def fix_2_settings_defaults(root):
    """Change Settings.py defaults: simulate = False."""
    print(f"\n{CYAN}[2] Settings.py — Default simulate = False{RESET}")
    path = root/"SupportClasses"/"Settings.py"
    content = read_n(path)

    marker = "v7.2.8s2: default simulate False"
    if marker in content:
        skip("Already patched"); return

    old = '''    "simulation": {
        "simulate_xy": True,
        "simulate_zp": True,
    },'''
    new = '''    "simulation": {
        "simulate_xy": False,  # v7.2.8s2: default simulate False (lab instrument)
        "simulate_zp": False,
    },'''

    if old in content:
        content = content.replace(old, new, 1)
        if write_c(path, content, "Settings defaults"):
            ok("DEFAULTS: simulate_xy/zp = False")
    else:
        skip("Defaults block not found or already False")


def fix_3_main_defaults(root):
    """Change main.py: default simulate = False, add --simulate flag."""
    print(f"\n{CYAN}[3] main.py — Default simulate = False + --simulate flag{RESET}")
    path = root/"main.py"
    content = read_n(path)

    marker = "v7.2.8s2: default simulate False"
    if marker in content:
        skip("Already patched"); return

    # Add --simulate flags
    old_args = '    parser.add_argument("--real-zp", action="store_true",\n                        help="Use real ZP stage hardware (default: simulate)")'
    new_args = (
        '    parser.add_argument("--real-zp", action="store_true",\n'
        '                        help="Use real ZP stage hardware")\n'
        '    # v7.2.8s2: default simulate False — add explicit simulate flags\n'
        '    parser.add_argument("--simulate-xy", action="store_true",\n'
        '                        help="Force XY stage simulation")\n'
        '    parser.add_argument("--simulate-zp", action="store_true",\n'
        '                        help="Force ZP stage simulation")'
    )

    if old_args in content:
        content = content.replace(old_args, new_args, 1)
    elif "--simulate-xy" in content:
        pass  # already has simulate flags
    else:
        miss("argparse block not found")
        return

    # Fix the simulate logic
    old_sim_logic = (
        '    simulate_xy = not args.real_xy and settings.get("simulation.simulate_xy", True)\n'
        '    simulate_zp = not args.real_zp and settings.get("simulation.simulate_zp", True)\n'
        '\n'
        '    if args.real_xy:\n'
        '        simulate_xy = False\n'
        '    if args.real_zp:\n'
        '        simulate_zp = False'
    )
    new_sim_logic = (
        '    # v7.2.8s2: default simulate False — real hardware is the default\n'
        '    simulate_xy = settings.get("simulation.simulate_xy", False)\n'
        '    simulate_zp = settings.get("simulation.simulate_zp", False)\n'
        '\n'
        '    # CLI overrides\n'
        '    if args.real_xy:\n'
        '        simulate_xy = False\n'
        '    if args.real_zp:\n'
        '        simulate_zp = False\n'
        '    if getattr(args, "simulate_xy", False):\n'
        '        simulate_xy = True\n'
        '    if getattr(args, "simulate_zp", False):\n'
        '        simulate_zp = True'
    )
    if old_sim_logic in content:
        content = content.replace(old_sim_logic, new_sim_logic, 1)
    elif "v7.2.8s2" not in content:
        miss("simulate logic block not found")
        return

    if write_c(path, content, "main.py defaults"):
        ok("main.py: simulate defaults to False, added --simulate-xy/zp flags")


def fix_4_settings_json(root):
    """Update settings.json: simulate = False."""
    print(f"\n{CYAN}[4] settings.json — Set simulate = false{RESET}")
    path = root/"settings.json"
    if not path.exists():
        skip("settings.json not found"); return

    raw = path.read_text(encoding="utf-8")
    try:
        data = json.loads(raw)
    except:
        miss("settings.json parse error"); return

    sim = data.get("simulation", {})
    changed = False
    if sim.get("simulate_xy", True) is True:
        sim["simulate_xy"] = False; changed = True
    if sim.get("simulate_zp", True) is True:
        sim["simulate_zp"] = False; changed = True
    data["simulation"] = sim

    if changed:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v728s2_{ts}"))
        # Preserve CRLF if present
        eol = "\r\n" if "\r\n" in raw else "\n"
        new_json = json.dumps(data, indent=2, ensure_ascii=False)
        if eol == "\r\n":
            new_json = new_json.replace("\n", "\r\n")
        path.write_text(new_json + eol, encoding="utf-8")
        ok("settings.json: simulate_xy/zp = false")
    else:
        skip("settings.json already has simulate = false")


def main():
    print(f"\n{BOLD}{'═'*60}{RESET}")
    print(f"{BOLD} MEBP v7.2.8 Session 2 — Position fix + defaults{RESET}")
    print(f"{BOLD}{'═'*60}{RESET}")
    hint = sys.argv[1] if len(sys.argv)>1 else None
    root = find_root(hint)
    print(f"\nProject root: {root}")

    fix_1_position_display(root)
    fix_2_settings_defaults(root)
    fix_3_main_defaults(root)
    fix_4_settings_json(root)

    print(f"\n{'═'*60}")
    print(f"  {GREEN}Applied: {applied}{RESET}  |  {YELLOW}Skipped: {skipped}{RESET}  |  {RED}Failed: {failed}{RESET}")
    print(f"{'═'*60}")

    if failed:
        print(f"\n{RED}⚠ Some patches failed.{RESET}")
        sys.exit(1)
    else:
        print(f"\n{GREEN}✓ All applied.{RESET}")
        print(f"\n{BOLD}NEXT CHAT TASKS:{RESET}")
        print(f"  1. Settings page Apply should toggle simulation at runtime")
        print(f"     (disconnect → update simulate flag → reconnect, no restart)")
        print(f"  2. Dashboard Connect buttons should respect current sim state")
        print(f"  3. ZP stage get_current_position may need same atomic lock fix")

if __name__ == "__main__":
    main()
