#!/usr/bin/env python3
"""
patch_v726_s1_serial_locks.py — v7.2.6 Session 1
Adds threading.RLock to XYStage and ZPStage for serial thread safety.
Increases PositionPoller interval from 0.3 to 1.0s.
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

# ══ S1-A: XYStage serial lock ═════════════════════════════════════
print("\n=== S1-A: XYStage serial RLock ===")
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = safe_read(xy_path)
if not content:
    sys.exit(1)

if "# v7.2.6: XY serial lock" not in content:
    changed = False

    # 1. Add import threading if missing
    if "import threading" not in content:
        content = re.sub(r'^(import re\b)', r'import threading\n\1',
                         content, count=1, flags=re.MULTILINE)
        if "import threading" not in content:
            content = "import threading\n" + content
        print(f"{OK} Added import threading (XYStage)")

    # 2. Inject _serial_lock after self.spo = self._initialise_serial()
    lock_code = (
        "\n        # v7.2.6: XY serial lock — RLock allows re-entry from move->send chain\n"
        "        self._serial_lock = threading.RLock()\n"
    )
    m = re.search(r'(        self\.spo = self\._initialise_serial\(\))', content)
    if m:
        content = content[:m.end()] + lock_code + content[m.end():]
        print(f"{OK} Added _serial_lock in XYStage.__init__")
        changed = True
    else:
        print(f"{MISS} Could not locate spo = _initialise_serial() in XYStage")

    # 3. Replace send_command with RLock version
    new_send = '''\
    def send_command(self, command: str) -> "Optional[str]":
        """Send a raw command to the stage.
        v7.2.6: XY serial lock — prevents PositionPoller/JogHandler contention.
        """
        if self.spo is None:
            logger.error("XY stage not initialised -- command ignored")
            return None
        if self.simulate:
            return self.spo.send_command(command)
        with self._serial_lock:
            try:
                if self._protocol:
                    encoded = command.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{command}\\r\\n".encode("ascii")
                self.spo.write(encoded)
                return None
            except (Exception,) as e:
                logger.error(f"XY send_command error: {e}")
                return None

'''
    m = re.search(r'^    def send_command\(self, command: str\).*?(?=\n    def |\nclass |\Z)',
                  content, re.DOTALL | re.MULTILINE)
    if m:
        content = content[:m.start()] + new_send + content[m.end():]
        print(f"{OK} Replaced send_command (XYStage)")
        changed = True
    else:
        print(f"{MISS} send_command not found in XYStage")

    # 4. Replace get_current_position with RLock version
    new_gcp = '''\
    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":
        """Query stage position.
        v7.2.6: XY serial lock wraps hardware send+readline.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)
        with self._serial_lock:
            try:
                self._send_protocol_command("position_query", fallback_cmd="P")
                response = self.spo.readline().decode(
                    self._protocol.encoding if self._protocol else "ascii",
                    errors="replace"
                ).strip()
                return self._parse_position_response(response)
            except Exception as e:
                logger.debug(f"XY position query error: {e}")
                return (None, None, None)

'''
    m = re.search(r'^    def get_current_position\(self\).*?(?=\n    def |\nclass |\Z)',
                  content, re.DOTALL | re.MULTILINE)
    if m:
        content = content[:m.start()] + new_gcp + content[m.end():]
        print(f"{OK} Replaced get_current_position (XYStage)")
        changed = True
    else:
        print(f"{MISS} get_current_position not found in XYStage")

    if changed:
        safe_write(xy_path, content, "XYStage.py S1-A")
    else:
        print(f"{MISS} No XYStage changes applied"); failed += 1
else:
    print(f"{SKIP} S1-A already applied"); skipped += 1

# ══ S1-B: ZPStage serial lock ═════════════════════════════════════
print("\n=== S1-B: ZPStage serial RLock ===")
zp_path = ROOT / "SupportClasses" / "ZPStage.py"
content = safe_read(zp_path)
if not content:
    sys.exit(1)

if "# v7.2.6: ZP serial lock" not in content:
    changed = False

    if "import threading" not in content:
        content = re.sub(r'^(import re\b)', r'import threading\n\1',
                         content, count=1, flags=re.MULTILINE)
        if "import threading" not in content:
            content = "import threading\n" + content
        print(f"{OK} Added import threading (ZPStage)")

    # Inject _serial_lock after self.simulate = ... in __init__
    zp_lock = (
        "        # v7.2.6: ZP serial lock — thread-safe serial access\n"
        "        self._serial_lock = threading.RLock()\n"
    )
    m = re.search(r'(        self\.simulate = .*?\n)', content)
    if m:
        content = content[:m.end()] + zp_lock + content[m.end():]
        print(f"{OK} Added _serial_lock in ZPStage.__init__")
        changed = True
    else:
        print(f"{MISS} Could not find self.simulate in ZPStage.__init__")

    # Replace send_data
    new_sd = '''\
    def send_data(self, data: str) -> None:
        """Send G-code command to printer.
        v7.2.6: ZP serial lock protects write+flush.
        """
        if self.serial is None:
            logger.error("ZP serial not initialised -- command ignored")
            return
        encoded = data.encode("utf-8") + b"\\n"
        for attempt in range(5):
            if hasattr(self.serial, "is_open") and not self.serial.is_open:
                if attempt < 4:
                    import time as _t; _t.sleep(0.01)
                    continue
                logger.error("ZP serial still not open after retries")
                return
            break
        with self._serial_lock:
            try:
                self.serial.write(encoded)
                self.serial.flush()
            except Exception as e:
                logger.error(f"ZP send_data error: {e}")

'''
    m = re.search(r'^    def send_data\(self, data: str\).*?(?=\n    def |\nclass |\Z)',
                  content, re.DOTALL | re.MULTILINE)
    if m:
        content = content[:m.start()] + new_sd + content[m.end():]
        print(f"{OK} Replaced send_data (ZPStage)")
        changed = True
    else:
        print(f"{MISS} send_data not found in ZPStage")

    # Replace receive_data
    new_rd = '''\
    def receive_data(self) -> str:
        """Read all available response data.
        v7.2.6: ZP serial lock protects read_all.
        """
        import time as _t; _t.sleep(0.01)
        with self._serial_lock:
            try:
                return self.serial.read_all().decode("utf-8", errors="replace").strip()
            except Exception as e:
                logger.debug(f"ZP receive_data error: {e}")
                return ""

'''
    m = re.search(r'^    def receive_data\(self\).*?(?=\n    def |\nclass |\Z)',
                  content, re.DOTALL | re.MULTILINE)
    if m:
        content = content[:m.start()] + new_rd + content[m.end():]
        print(f"{OK} Replaced receive_data (ZPStage)")
        changed = True
    else:
        print(f"{MISS} receive_data not found in ZPStage")

    if changed:
        safe_write(zp_path, content, "ZPStage.py S1-B")
    else:
        print(f"{MISS} No ZPStage changes applied"); failed += 1
else:
    print(f"{SKIP} S1-B already applied"); skipped += 1

# ══ S1-C: PositionPoller 0.3 → 1.0s ══════════════════════════════
print("\n=== S1-C: PositionPoller interval 0.3 → 1.0s ===")
sc_path = ROOT / "SupportClasses" / "StageController.py"
content = safe_read(sc_path)
if not content:
    sys.exit(1)

if "# v7.2.6: poll interval 1.0s" not in content:
    new_c, n = re.subn(
        r'PositionPoller\(poll_interval=0\.3\)',
        'PositionPoller(poll_interval=1.0)  # v7.2.6: poll interval 1.0s',
        content
    )
    if n:
        content = new_c
        safe_write(sc_path, content, "StageController.py S1-C")
        print(f"{OK} PositionPoller interval 0.3 -> 1.0s")
    else:
        print(f"{MISS} PositionPoller(poll_interval=0.3) not found"); failed += 1
else:
    print(f"{SKIP} S1-C already applied"); skipped += 1

print(f"\n{'='*50}")
print(f"S1 Results: {GREEN}{applied} applied{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
