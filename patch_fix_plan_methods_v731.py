#!/usr/bin/env python3
"""
patch_fix_plan_methods_v731.py
Fix _generate_plan and _run_validation in WellSetupTab to use
the correct PrintPlanOfAction API:

  Correct:  plan = PrintPlanOfAction()
            plan.generate(hw_config, well_model)

  Wrong:    plan = PrintPlanOfAction(well_model, prefs)   ← what we wrote

  Correct:  validate_well_setup(hw_config=..., well_model=..., plan=...)
  Wrong:    validate_well_setup(self._model)              ← missing hw_config
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"

ok_count = skip_count = fail_count = 0

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def find_method(content: str, name: str):
    """Return (start, end) of a class method body, or None."""
    pat = re.compile(
        r'^(    def ' + re.escape(name) + r'\(self.*?\n)'
        r'(.*?)'
        r'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

def safe_write(path: Path, content: str) -> bool:
    global ok_count, fail_count
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL — not writing: {e}{RESET}")
        fail_count += 1
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v731plan_{ts}"))
    path.write_text(content, encoding="utf-8")
    ok_count += 1
    return True

# ── Replacement methods ───────────────────────────────────────────

NEW_GENERATE_PLAN = '''    def _generate_plan(self) -> None:
        """Generate print plan of action — v7.3.1 fix: correct API usage."""
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
            plan = PrintPlanOfAction()
            if prefs is not None:
                plan.preferences = prefs
            plan.generate(self._hw_config, self._model)
            self._plan = plan
            summary_fn = getattr(plan, "summary", None)
            if summary_fn and callable(summary_fn):
                text = summary_fn()
            else:
                steps = getattr(plan, "steps", [])
                text = f"Plan generated: {len(steps)} step(s)."
            if lbl:
                lbl.setText(text)
        except Exception as exc:
            if lbl:
                lbl.setText(f"Plan generation error: {exc}")
            logger.error(f"_generate_plan error: {exc}", exc_info=True)

'''

NEW_RUN_VALIDATION = '''    def _run_validation(self) -> None:
        """Run well setup validation — v7.3.1 fix: pass hw_config + well_model."""
        lbl = getattr(self, "_plan_label", None)
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            ok, messages = validate_well_setup(
                hw_config=self._hw_config,
                well_model=self._model,
                plan=self._plan,
            )
            text = ("✓ Valid" if ok else "✗ Issues: " + "; ".join(messages))
            if lbl:
                lbl.setText(text)
        except ImportError:
            if lbl:
                lbl.setText("Validation module not available.")
        except Exception as exc:
            if lbl:
                lbl.setText(f"Validation error: {exc}")
            logger.error(f"_run_validation error: {exc}", exc_info=True)

'''

def main():
    global ok_count, skip_count, fail_count

    root   = find_root()
    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")
    changed = False

    # ── 1. Replace _generate_plan ─────────────────────────────────
    guard1 = "v7.3.1 fix: correct API usage"
    if guard1 not in content:
        m = find_method(content, "_generate_plan")
        if m:
            content = content[:m.start()] + NEW_GENERATE_PLAN + content[m.end():]
            print(f"  {GREEN}✓ Replaced _generate_plan{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ MISS: _generate_plan not found{RESET}")
            fail_count += 1
    else:
        print(f"  {YELLOW}○ SKIP: _generate_plan already fixed{RESET}")
        skip_count += 1

    # ── 2. Replace _run_validation ────────────────────────────────
    guard2 = "v7.3.1 fix: pass hw_config + well_model"
    if guard2 not in content:
        m = find_method(content, "_run_validation")
        if m:
            content = content[:m.start()] + NEW_RUN_VALIDATION + content[m.end():]
            print(f"  {GREEN}✓ Replaced _run_validation{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ MISS: _run_validation not found{RESET}")
            fail_count += 1
    else:
        print(f"  {YELLOW}○ SKIP: _run_validation already fixed{RESET}")
        skip_count += 1

    if changed:
        safe_write(target, content)

    print(f"\n{CYAN}{'─'*40}")
    print(f"Results:  {GREEN}{ok_count} applied{RESET}  "
          f"{YELLOW}{skip_count} skipped{RESET}  "
          f"{RED}{fail_count} failed{RESET}")
    if fail_count:
        sys.exit(1)

if __name__ == "__main__":
    main()
