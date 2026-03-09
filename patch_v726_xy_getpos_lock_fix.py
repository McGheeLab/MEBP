#!/usr/bin/env python3
"""
patch_v726_xy_getpos_lock_fix.py
Remove the serial lock wrap from XYStage.get_current_position.
Uses direct str.replace on all known lock-wrap patterns.
"""
import ast, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; SKIP = f"{YELLOW}  o{RESET}"; MISS = f"{RED}  x{RESET}"

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

ROOT = find_root()
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = xy_path.read_text(encoding="utf-8")
original = content

# ── Check if there's actually a lock wrap in get_current_position ─
if "with self._serial_lock:" not in content:
    print(f"{SKIP} No 'with self._serial_lock:' found anywhere in XYStage.py — nothing to do")
    sys.exit(0)

# ── Try all known variants of the wrapped get_current_position ────
# Variant A: lock wraps the entire hardware try block
OLD_A = (
    "        try:\n"
    "            with self._serial_lock:\n"
    "                self._send_protocol_command(\"position_query\", fallback_cmd=\"P\")\n"
    "                response = self.spo.readline().decode(\n"
    "                    self._protocol.encoding if self._protocol else \"ascii\",\n"
    "                    errors=\"replace\"\n"
    "                ).strip()\n"
    "                return self._parse_position_response(response)\n"
    "        except Exception as e:\n"
    "            logger.debug(f\"XY position query error: {e}\")\n"
    "            return (None, None, None)"
)
NEW_A = (
    "        try:\n"
    "            self._send_protocol_command(\"position_query\", fallback_cmd=\"P\")\n"
    "            response = self.spo.readline().decode(\n"
    "                self._protocol.encoding if self._protocol else \"ascii\",\n"
    "                errors=\"replace\"\n"
    "            ).strip()\n"
    "            return self._parse_position_response(response)\n"
    "        except Exception as e:\n"
    "            logger.debug(f\"XY position query error: {e}\")\n"
    "            return (None, None, None)"
)

# Variant B: lock wraps just the send, readline is outside
OLD_B = (
    "        try:\n"
    "            with self._serial_lock:\n"
    "                self._send_protocol_command(\"position_query\", fallback_cmd=\"P\")\n"
    "            response = self.spo.readline().decode(\n"
    "                self._protocol.encoding if self._protocol else \"ascii\",\n"
    "                errors=\"replace\"\n"
    "            ).strip()\n"
    "            return self._parse_position_response(response)\n"
    "        except Exception as e:\n"
    "            logger.debug(f\"XY position query error: {e}\")\n"
    "            return (None, None, None)"
)
NEW_B = NEW_A  # same target

fixed = False
for OLD, NEW, label in [(OLD_A, NEW_A, "Variant A (full wrap)"),
                         (OLD_B, NEW_B, "Variant B (send-only wrap)")]:
    if OLD in content:
        content = content.replace(OLD, NEW, 1)
        print(f"{OK} Fixed get_current_position ({label})")
        fixed = True
        break

if not fixed:
    # Variant C: lock wraps the entire method body differently — scan for any
    # 'with self._serial_lock:' remaining and report its context
    import re
    matches = [(m.start(), content[max(0,m.start()-100):m.start()+200])
               for m in re.finditer(r'with self\._serial_lock:', content)]
    if matches:
        print(f"{MISS} Could not pattern-match the lock wrap. Remaining occurrences:")
        for pos, ctx in matches:
            print(f"  line ~{content[:pos].count(chr(10))+1}: ...{ctx.strip()[:120]}...")
        sys.exit(1)
    else:
        print(f"{SKIP} No lock wrap found in get_current_position (already clean)")
        sys.exit(0)

# ── AST verify + write ────────────────────────────────────────────
try:
    ast.parse(content)
except SyntaxError as e:
    print(f"{MISS} AST FAIL: {e}")
    sys.exit(1)

ts = datetime.now().strftime("%Y%m%d_%H%M%S")
shutil.copy2(xy_path, xy_path.with_suffix(f".bak_v726_{ts}"))
xy_path.write_text(content, encoding="utf-8")
print(f"{OK} XYStage.py written — get_current_position lock removed")
