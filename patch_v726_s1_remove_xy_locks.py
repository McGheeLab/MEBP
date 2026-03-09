#!/usr/bin/env python3
"""
patch_v726_s1_remove_xy_locks.py
Remove 'with self._serial_lock:' wrapping from XYStage methods.

Root cause: S1 wrapped get_current_position() hardware path with the lock,
including the readline() call. PositionPoller holds the lock for the full
readline() duration (up to the serial timeout). XYJogHandler (different thread)
tries send_command() → blocks waiting for lock → all jog commands dropped.

Fix: Remove lock wrapping from send_command(), get_current_position(), and
move_stage_at_velocity(). Keep self._serial_lock = threading.RLock() in
__init__ so no AttributeError if anything still references it.
The poll-interval increase to 1.0s (S1-C) already reduces contention enough.
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
print("\n=== XYStage.py: remove serial lock wrapping from methods ===")
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = xy_path.read_text(encoding="utf-8")

# Replace the entire send_command method with a clean unlocked version
NEW_SEND_COMMAND = '''\
    def send_command(self, command: str) -> Optional[str]:
        """
        Send a raw command string to the stage.

        P8.18: In hardware mode, uses protocol.tx_terminator instead
        of hardcoded \\r\\n.

        In simulation mode, returns the simulator's response directly.
        In hardware mode, writes to serial (response must be read separately).
        Note: v7.2.6 — serial lock removed from method body; lock exists on
        the object but holding it across a blocking readline() caused jog
        commands to be starved. Poll interval (1.0s) reduces contention.
        """
        if self.spo is None:
            logger.error("XY stage not initialised — command ignored")
            return None

        if self.simulate:
            return self.spo.send_command(command)

        try:
            # P8.18: Use protocol terminator
            if self._protocol:
                encoded = command.encode(self._protocol.encoding) + self._protocol.tx_terminator
            else:
                # Fallback: default terminator
                encoded = f"{command}\\r\\n".encode("ascii")

            self.spo.write(encoded)
            return None  # caller reads response via get_current_position etc.
        except (serial.SerialException, OSError) as e:
            logger.error(f"XY send_command error: {e}")
            return None

'''

# Replace the entire get_current_position method with a clean unlocked version
NEW_GET_POS = '''\
    def get_current_position(self) -> tuple[float | None, float | None, float | None]:
        """
        Query the stage for its current position.

        Returns:
            (x, y, z) tuple, or (None, None, None) on failure.
        Note: v7.2.6 — serial lock removed; holding lock across readline()
        blocked jog commands (different thread) for full read duration.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)

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

changes = 0

# Replace send_command
m = re.search(
    r'^    def send_command\(self, command: str\).*?(?=\n    def )',
    content, re.DOTALL | re.MULTILINE
)
if m:
    content = content[:m.start()] + NEW_SEND_COMMAND + content[m.end():]
    print(f"{OK} Replaced send_command (lock removed)")
    changes += 1
else:
    print(f"{MISS} send_command not found"); failed += 1

# Replace get_current_position
m = re.search(
    r'^    def get_current_position\(self\).*?(?=\n    @staticmethod)',
    content, re.DOTALL | re.MULTILINE
)
if m:
    content = content[:m.start()] + NEW_GET_POS + content[m.end():]
    print(f"{OK} Replaced get_current_position (lock removed)")
    changes += 1
else:
    print(f"{MISS} get_current_position not found"); failed += 1

# Also remove any 'with self._serial_lock:' blocks that may wrap move_stage_at_velocity
# Strategy: find the pattern and de-indent the body
def remove_lock_wrap(text, method_name):
    """Find 'with self._serial_lock:' inside a named method and unwrap it."""
    # Find the method
    m_method = re.search(
        rf'^    def {re.escape(method_name)}\(.*?(?=\n    def |\nclass |\Z)',
        text, re.DOTALL | re.MULTILINE
    )
    if not m_method:
        return text, False
    
    method_src = m_method.group(0)
    
    # Check if there's a with self._serial_lock: block
    m_lock = re.search(
        r'(\n        )with self\._serial_lock:\n((?:\n|.+?\n)*?)(?=\n        [^\s]|\Z)',
        method_src, re.DOTALL
    )
    if not m_lock:
        return text, False
    
    # De-indent the body (remove 4 spaces from each line in the with block)
    lock_body = m_lock.group(2)
    dedented = re.sub(r'^            ', '        ', lock_body, flags=re.MULTILINE)
    new_method = method_src[:m_lock.start()] + '\n' + dedented + method_src[m_lock.end():]
    return text[:m_method.start()] + new_method + text[m_method.end():], True

content, ok = remove_lock_wrap(content, 'move_stage_at_velocity')
if ok:
    print(f"{OK} Unwrapped move_stage_at_velocity lock")
    changes += 1
else:
    print(f"{SKIP} move_stage_at_velocity: no lock wrap found (ok)")

if changes > 0 and failed == 0:
    safe_write(xy_path, content, "XYStage.py")

# ══ ZPStage.py ════════════════════════════════════════════════════
print("\n=== ZPStage.py: remove serial lock wrapping from methods ===")
zp_path = ROOT / "SupportClasses" / "ZPStage.py"
content_z = zp_path.read_text(encoding="utf-8")

zp_changes = 0
for mname in ['send_data', 'receive_data', 'get_current_position', 'move_relative']:
    content_z, ok = remove_lock_wrap(content_z, mname)
    if ok:
        print(f"{OK} Unwrapped {mname} lock in ZPStage")
        zp_changes += 1
    else:
        print(f"{SKIP} {mname}: no lock wrap found (ok)")

if zp_changes > 0 and failed == 0:
    safe_write(zp_path, content_z, "ZPStage.py")
elif zp_changes == 0:
    print(f"{SKIP} ZPStage.py: no lock wraps to remove")

print(f"\n{'='*50}")
print(f"Results: {GREEN}{applied} written{RESET}  "
      f"{YELLOW}{skipped} skipped{RESET}  "
      f"{RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)
