#!/usr/bin/env python3
"""
patch_fix_generate_plan_v731.py
Fix _generate_plan to use the correct classmethod API:
    PrintPlanOfAction.generate_plan(hw_config, well_model, preferences)
instead of the wrong instance method:
    plan = PrintPlanOfAction(); plan.generate(...)
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"

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

NEW_GENERATE_PLAN = '''    def _generate_plan(self) -> None:
        """Generate print plan — v7.3.1b: use classmethod generate_plan()."""
        lbl = getattr(self, "_plan_label", None)
        if self._hw_config is None:
            if lbl:
                lbl.setText("⚠ Hardware config required to generate plan.")
            return
        try:
            from SupportClasses.PrintPlanOfAction import PrintPlanOfAction
        except ImportError:
            if lbl:
                lbl.setText("PrintPlanOfAction module not available.")
            return
        try:
            prefs = self._get_plan_preferences()
            self._plan = PrintPlanOfAction.generate_plan(
                self._hw_config, self._model, prefs
            )
            summary_fn = getattr(self._plan, "summary", None)
            if summary_fn and callable(summary_fn):
                text = summary_fn()
            else:
                steps = getattr(self._plan, "steps", [])
                runs  = getattr(self._plan, "total_runs", "?")
                text  = f"Plan: {len(steps)} steps, {runs} run(s)."
            if lbl:
                lbl.setText(text)
        except Exception as exc:
            if lbl:
                lbl.setText(f"Plan generation error: {exc}")
            logger.error(f"_generate_plan error: {exc}", exc_info=True)

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    guard = "v7.3.1b: use classmethod generate_plan()"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}")
        return

    m = find_method(content, "_generate_plan")
    if not m:
        print(f"  {RED}✗ MISS: _generate_plan not found{RESET}")
        sys.exit(1)

    new_content = content[:m.start()] + NEW_GENERATE_PLAN + content[m.end():]

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731gp_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ _generate_plan fixed — now uses generate_plan() classmethod{RESET}")

if __name__ == "__main__":
    main()
