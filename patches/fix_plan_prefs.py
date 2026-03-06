#!/usr/bin/env python3
"""
Fix _get_plan_preferences — references UI widgets that don't exist.
Makes it use safe getattr defaults instead of direct attribute access.

Also fixes validate() to catch errors gracefully.

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v726/fix_plan_prefs.py
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; B = "\033[1m"; X = "\033[0m"

def find_root():
    for c in [Path("."), Path(".."), Path(__file__).parent.parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{R}Cannot find project root{X}"); sys.exit(1)

def main():
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else find_root()
    path = root / "gui" / "pages" / "print_well_setup.py"

    print(f"\n{B}Fix: _get_plan_preferences + validate error handling{X}")

    if not path.exists():
        print(f"{R}File not found: {path}{X}"); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v726fix_{ts}"))
    content = path.read_text(encoding="utf-8")
    changed = False

    # ── Fix 1: Replace _get_plan_preferences with safe version ────
    m = re.search(
        r'(    def _get_plan_preferences\(self\).*?\n)(.*?)(?=\n    def )',
        content, re.DOTALL
    )
    if m:
        new_method = '''    def _get_plan_preferences(self):
        """v7.2.6: Build PlanPreferences with safe defaults for missing UI widgets."""
        try:
            from SupportClasses.PrintPlanOfAction import PlanPreferences
        except ImportError:
            return None

        return PlanPreferences(
            max_ink_per_run_uL=getattr(getattr(self, '_max_ink_spin', None), 'value', lambda: 50.0)(),
            wash_between_inks=getattr(getattr(self, '_wash_check', None), 'isChecked', lambda: True)(),
            wash_after_refill=getattr(getattr(self, '_wash_after_check', None), 'isChecked', lambda: True)(),
            prime_after_refill=getattr(getattr(self, '_prime_check', None), 'isChecked', lambda: True)(),
            waste_before_wash=getattr(getattr(self, '_waste_check', None), 'isChecked', lambda: False)(),
            buffer_after_wash=getattr(getattr(self, '_buffer_check', None), 'isChecked', lambda: False)(),
        )

'''
        content = content[:m.start()] + new_method + content[m.end():]
        changed = True
        print(f"  {G}✓{X} Replaced _get_plan_preferences with safe getattr version")
    else:
        print(f"  {Y}○{X} _get_plan_preferences not found")

    # ── Fix 2: Wrap validate's _generate_plan call in try/except ──
    m2 = re.search(
        r'(    def validate\(self\).*?\n)(.*?)(?=\n    def )',
        content, re.DOTALL
    )
    if m2 and 'try:' not in m2.group(2)[:100]:
        new_validate = '''    def validate(self) -> tuple[bool, list[str]]:
        """v7.2.6: Validation with error handling."""
        # Auto-generate plan if missing
        if getattr(self, '_plan', None) is None and getattr(self, '_hw_config', None) is not None:
            try:
                if hasattr(self, '_generate_plan'):
                    self._generate_plan()
            except Exception as e:
                import logging
                logging.getLogger(__name__).error(f"Plan generation failed: {e}")
                return False, [f"Plan generation error: {e}"]

        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            return validate_well_setup(
                hw_config=self._hw_config,
                well_model=self._model,
                plan=getattr(self, '_plan', None),
            )
        except ImportError:
            return False, ["PrintPlanOfAction module not available"]
        except Exception as e:
            return False, [f"Validation error: {e}"]

'''
        content = content[:m2.start()] + new_validate + content[m2.end():]
        changed = True
        print(f"  {G}✓{X} Wrapped validate() with error handling")
    elif m2:
        print(f"  {Y}○{X} validate already has try/except")
    else:
        print(f"  {Y}○{X} validate method not found")

    # ── Fix 3: Also wrap _generate_plan itself ────────────────────
    m3 = re.search(
        r'(    def _generate_plan\(self\).*?\n)(.*?)(?=\n    def )',
        content, re.DOTALL
    )
    if m3 and 'try:' not in m3.group(2)[:80]:
        new_gen = '''    def _generate_plan(self):
        """v7.2.6: Generate execution plan with error handling."""
        if self._hw_config is None:
            import logging
            logging.getLogger(__name__).warning("Cannot generate plan: no hardware config")
            return

        try:
            from SupportClasses.PrintPlanOfAction import PrintPlanOfAction
            prefs = self._get_plan_preferences()
            self._plan = PrintPlanOfAction()
            if prefs:
                self._plan.preferences = prefs
            self._plan.generate(self._hw_config, self._model)

            import logging
            logging.getLogger(__name__).info(
                f"Plan generated: {len(self._plan.steps)} steps, "
                f"{self._plan.total_runs} runs")

            if hasattr(self, '_plan_summary'):
                lines = self._plan.step_summary_lines()
                self._plan_summary.setText("\\n".join(lines[:10]))
            if hasattr(self, '_run_validation'):
                self._run_validation()
        except Exception as e:
            import logging
            logging.getLogger(__name__).error(f"Plan generation failed: {e}")
            self._plan = None

'''
        content = content[:m3.start()] + new_gen + content[m3.end():]
        changed = True
        print(f"  {G}✓{X} Wrapped _generate_plan with error handling")
    elif m3:
        print(f"  {Y}○{X} _generate_plan already has try/except")
    else:
        print(f"  {Y}○{X} _generate_plan not found")

    if changed:
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {R}AST FAIL: {e}{X}"); sys.exit(1)
        path.write_text(content, encoding="utf-8")
        print(f"  {G}WROTE{X}: {path.name}")
    else:
        print(f"  {Y}No changes needed{X}")

    print(f"\n{G}✓ Done. Re-run: python main.py{X}")

if __name__ == "__main__":
    main()
