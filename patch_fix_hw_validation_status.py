#!/usr/bin/env python3
"""
patch_fix_hw_validation_status.py
Bug 1 fix: hardware validity label in context panel stays stale on startup.

Root cause: _ctx_validity_label is created lazily in get_context_widget().
            By the time it is created, _on_config_changed() has already run
            (during config restore at startup) but _ctx_validity_label did
            not exist yet, so hasattr() returned False and the update skipped.

Fix: inject a _sync_validity_display() helper, call it from:
  1. _on_config_changed() — already happens for runtime changes (no change needed)
  2. get_context_widget() — right after building _ctx_validity_label, sync once
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN  = "\033[96m"; RESET = "\033[0m"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def find_method(content: str, name: str):
    pat = re.compile(
        r'^(    def ' + re.escape(name) + r'\(self.*?\n)'
        r'(.*?)'
        r'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

# ── New helper method ─────────────────────────────────────────────
SYNC_METHOD = '''    def _sync_validity_display(self) -> None:
        """v7.3.1: Sync all validity labels to current config state.
        Safe to call at any time — no-ops if widgets not yet created."""
        valid = getattr(self._config, "is_valid", False)
        _, issues = self._config.validate() if hasattr(self._config, "validate") else (valid, [])

        # Main page label
        if hasattr(self, "validity_label") and self.validity_label is not None:
            if valid:
                self.validity_label.setText("✓ Setup complete")
                self.validity_label.setStyleSheet(
                    f"color: {COLORS.get('green', '#a6e3a1')};")
            else:
                msg = issues[0] if issues else "Setup incomplete"
                self.validity_label.setText(f"⚠ {msg}")
                self.validity_label.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')};")

        # Context panel label (lazily created — may not exist yet)
        if hasattr(self, "_ctx_validity_label") and self._ctx_validity_label is not None:
            if valid:
                self._ctx_validity_label.setText("✓ Setup complete")
                self._ctx_validity_label.setStyleSheet(
                    f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 9pt;")
            else:
                msg = issues[0] if issues else "Setup incomplete"
                self._ctx_validity_label.setText(f"⚠ {msg}")
                self._ctx_validity_label.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 9pt;")

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "hardware_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")
    changed = False

    # ── 1. Add _sync_validity_display if missing ──────────────────
    guard1 = "v7.3.1: Sync all validity labels"
    if guard1 not in content:
        m = find_method(content, "_on_config_changed")
        if not m:
            print(f"  {RED}✗ MISS: _on_config_changed not found{RESET}")
            sys.exit(1)
        # Insert before _on_config_changed
        content = content[:m.start()] + SYNC_METHOD + content[m.start():]
        print(f"  {GREEN}✓ Added _sync_validity_display(){RESET}")
        changed = True
    else:
        print(f"  {YELLOW}○ SKIP: _sync_validity_display already present{RESET}")

    # ── 2. Call _sync_validity_display at end of _on_config_changed ─
    guard2 = "self._sync_validity_display()"
    if guard2 not in content:
        # Find the config_changed.emit line and add sync after it
        old = "        self.config_changed.emit(self._config)"
        new = ("        self.config_changed.emit(self._config)\n"
               "        self._sync_validity_display()  # v7.3.1: keep labels in sync")
        if old in content:
            content = content.replace(old, new, 1)
            print(f"  {GREEN}✓ _sync_validity_display() called from _on_config_changed{RESET}")
            changed = True
        else:
            print(f"  {YELLOW}⚠ Could not find config_changed.emit anchor — skipping step 2{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: sync call already present{RESET}")

    # ── 3. Call _sync_validity_display at end of get_context_widget ─
    guard3 = "_sync_validity_display()  # v7.3.1: sync on first show"
    if guard3 not in content:
        old = "        # Initial scan\n        self._scan_config_directory()"
        new = ("        # Sync validity label now that _ctx_validity_label exists\n"
               "        self._sync_validity_display()  # v7.3.1: sync on first show\n\n"
               "        # Initial scan\n        self._scan_config_directory()")
        if old in content:
            content = content.replace(old, new, 1)
            print(f"  {GREEN}✓ Sync called in get_context_widget(){RESET}")
            changed = True
        else:
            # Try fallback anchor
            old2 = "        self._scan_config_directory()\n\n        return ctx"
            new2 = ("        self._sync_validity_display()  # v7.3.1: sync on first show\n"
                    "        self._scan_config_directory()\n\n        return ctx")
            if old2 in content:
                content = content.replace(old2, new2, 1)
                print(f"  {GREEN}✓ Sync called in get_context_widget() (fallback anchor){RESET}")
                changed = True
            else:
                print(f"  {YELLOW}⚠ Could not find get_context_widget anchor — skipping step 3{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: sync-on-first-show already present{RESET}")

    if not changed:
        print(f"  {YELLOW}○ Nothing to do{RESET}")
        return

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731hvs_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ hardware_setup.py patched{RESET}")

if __name__ == "__main__":
    main()
