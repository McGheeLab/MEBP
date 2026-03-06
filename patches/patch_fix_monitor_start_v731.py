#!/usr/bin/env python3
"""
patch_fix_monitor_start_v731.py
Fix app.py._on_monitor_start: PrintManager.start() takes no args.
Must call load_job(job) first, then start().
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

NEW_ON_MONITOR_START = '''    def _on_monitor_start(self, job):
        """Monitor requested start — load job then start. v7.3.1 fix."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, "print_manager"):
            pm = setup_page.print_manager
            try:
                pm.load_job(job)   # load first — start() takes no args
                pm.start()
            except Exception as exc:
                logger.error(f"PrintManager start failed: {exc}", exc_info=True)

'''

def main():
    root   = find_root()
    target = root / "gui" / "app.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    guard = "load_job(job)   # load first"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}")
        return

    m = find_method(content, "_on_monitor_start")
    if not m:
        print(f"  {RED}✗ MISS: _on_monitor_start not found in app.py{RESET}")
        sys.exit(1)

    new_content = content[:m.start()] + NEW_ON_MONITOR_START + content[m.end():]

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731ms_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ _on_monitor_start fixed — now calls load_job(job) then start(){RESET}")

if __name__ == "__main__":
    main()
