#!/usr/bin/env python3
"""
MEBP v7.2.6 Fix — BUG 5b only (ZPJogHandler zero_position plumbing)

The main patch applied BUG 5 clamping logic correctly, but failed to:
1. Add zero_position parameter to ZPJogHandler.__init__
2. Add self._zero_position attribute
3. Pass zero_position at the constructor call site

Without this fix, getattr(self, '_zero_position', {}) returns {} and
the zero-offset correction is effectively a no-op.
"""

import ast
import sys
from pathlib import Path

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

ok_count = 0
skip_count = 0
miss_count = 0


def ok(msg):
    global ok_count; ok_count += 1
    print(f"  {GREEN}✓ OK{RESET}   {msg}")

def skip(msg):
    global skip_count; skip_count += 1
    print(f"  {YELLOW}○ SKIP{RESET} {msg}")

def miss(msg):
    global miss_count; miss_count += 1
    print(f"  {RED}✗ MISS{RESET} {msg}")


def find_root() -> Path:
    candidates = [
        Path("/Users/alexmcghee/Documents/GitHub/MEBP"),
        Path.cwd(),
        Path(__file__).resolve().parent.parent.parent,
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def main():
    print(f"\n{BOLD}{'═' * 60}")
    print(f"  MEBP v7.2.6 Fix — BUG 5b (ZPJogHandler zero_position)")
    print(f"{'═' * 60}{RESET}\n")

    root = find_root()
    path = root / "SupportClasses" / "StageController.py"
    print(f"File: {path}\n")

    content = path.read_text(encoding="utf-8")

    # ── Fix 1: Add zero_position parameter to __init__ ──────────

    marker = "v7.2.6: zero ref for clamping"

    if marker in content:
        skip("zero_position parameter already in ZPJogHandler.__init__")
    else:
        old_sig = "        get_zp_position: Callable | None = None,\n    ):"
        new_sig = (
            "        get_zp_position: Callable | None = None,\n"
            "        zero_position: dict | None = None,  # v7.2.6: zero ref for clamping\n"
            "    ):"
        )

        if old_sig in content:
            content = content.replace(old_sig, new_sig, 1)
            ok("Added zero_position parameter to ZPJogHandler.__init__")
        else:
            miss("Cannot find ZPJogHandler.__init__ signature")

    # ── Fix 2: Add self._zero_position attribute ─────────────────

    attr_marker = "self._zero_position"

    if attr_marker in content:
        skip("_zero_position attribute already exists")
    else:
        old_assign = "        self._get_zp_position = get_zp_position\n"
        new_assign = (
            "        self._get_zp_position = get_zp_position\n"
            "        self._zero_position = zero_position or {}  "
            "# v7.2.6: zero ref for jog clamping\n"
        )

        if old_assign in content:
            content = content.replace(old_assign, new_assign, 1)
            ok("Added self._zero_position attribute")
        else:
            miss("Cannot find self._get_zp_position = get_zp_position line")

    # ── Fix 3: Pass zero_position at creation site ───────────────

    create_marker = "zero_position=self.zero_position"

    if create_marker in content:
        skip("zero_position already passed at creation site")
    else:
        # The exact creation code (from the uploaded file):
        old_create = (
            "            self.zp_jog = ZPJogHandler(\n"
            "                self.processor, self.zp_stage,\n"
            "                safety_limits=self.safety_limits,\n"
            "                get_zp_position=lambda: self._pos_poller.zp_position,\n"
            "            )"
        )
        new_create = (
            "            self.zp_jog = ZPJogHandler(\n"
            "                self.processor, self.zp_stage,\n"
            "                safety_limits=self.safety_limits,\n"
            "                get_zp_position=lambda: self._pos_poller.zp_position,\n"
            "                zero_position=self.zero_position,\n"
            "            )"
        )

        if old_create in content:
            content = content.replace(old_create, new_create, 1)
            ok("Passed zero_position=self.zero_position at creation site")
        else:
            miss("Cannot find ZPJogHandler creation site (may already be modified)")

    # ── AST verify and write ─────────────────────────────────────

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"\n  {RED}AST FAIL{RESET}: {e}")
        print(f"  {RED}File NOT written.{RESET}")
        sys.exit(1)

    path.write_text(content, encoding="utf-8")
    print(f"\n  {GREEN}AST OK — file written successfully{RESET}")

    # Summary
    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'─' * 60}")
    print(f"  Summary: {GREEN}{ok_count} OK{RESET}  {YELLOW}{skip_count} SKIP{RESET}  {RED}{miss_count} MISS{RESET}  ({total} total)")
    print(f"{'─' * 60}{RESET}\n")

    if miss_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
