#!/usr/bin/env python3
"""
patch_add_validate_v731.py
Add validate() and get_plan() methods to WellSetupTab.
These are called by print_setup.py before sending a job to the monitor.
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

# Insert before _save_layout (end of class, before Save/Load section)
NEW_METHODS = '''
    def validate(self) -> tuple:
        """v7.3.1: Called by print_setup.py before sending job to monitor."""
        if self._hw_config is None:
            return False, ["No hardware configuration — complete Hardware Setup first"]
        if self._plan is None:
            try:
                self._generate_plan()
            except Exception as exc:
                return False, [f"Plan generation error: {exc}"]
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            return validate_well_setup(
                hw_config=self._hw_config,
                well_model=self._model,
                plan=self._plan,
            )
        except ImportError:
            return False, ["PrintPlanOfAction module not available"]
        except Exception as exc:
            return False, [f"Validation error: {exc}"]

    def get_plan(self):
        """v7.3.1: Return the current PrintPlanOfAction (may be None)."""
        return self._plan

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    guard = "v7.3.1: Called by print_setup.py before sending job"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: validate() already present{RESET}")
        return

    # Insert before _save_layout
    anchor = re.search(r'\n    def _save_layout\(self\)', content)
    if not anchor:
        print(f"  {RED}✗ MISS: _save_layout anchor not found{RESET}")
        sys.exit(1)

    new_content = (
        content[:anchor.start()]
        + NEW_METHODS
        + content[anchor.start():]
    )

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731val_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ validate() and get_plan() added to WellSetupTab{RESET}")

if __name__ == "__main__":
    main()
