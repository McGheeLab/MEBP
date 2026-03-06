#!/usr/bin/env python3
"""
patch_fix_build_job_v731.py
Fix _build_current_job in print_setup.py.

build_well_plate_job() real signature:
    build_well_plate_job(well_positions, path_points, settings, pump, ...)

Current (broken) call:
    build_well_plate_job(plate=plate, selected_wells=print_wells, settings=settings)

Fix: build well_positions list from plate + model, pass empty path_points.
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

NEW_BUILD_JOB = '''    def _build_current_job(self):
        """Build a PrintJob from current well setup. v7.3.1 fix: correct API."""
        if not (hasattr(self, 'tab_wells') and self.tab_wells
                and hasattr(self.tab_wells, '_model') and self.tab_wells._model):
            logger.warning("No printable job could be built from current setup")
            return None

        model    = self.tab_wells._model
        plate    = model.plate
        settings = self._get_settings()

        # Collect print wells with their XY centre positions
        well_positions = []
        for name, assignment in model.assignments.items():
            if assignment.role.value != "print":
                continue
            try:
                well_info = plate.get_well_info(name)
                well_positions.append((name, well_info.x_mm, well_info.y_mm))
            except Exception:
                well_positions.append((name, 0.0, 0.0))

        if not well_positions:
            logger.warning("No print wells assigned — cannot build job")
            return None

        try:
            return build_well_plate_job(
                well_positions=well_positions,
                path_points=[],        # no custom path — printer uses defaults
                settings=settings,
                job_name="Well Plate Print",
            )
        except Exception as exc:
            logger.error(f"build_well_plate_job failed: {exc}", exc_info=True)
            return None

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    guard = "v7.3.1 fix: correct API"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}")
        return

    m = find_method(content, "_build_current_job")
    if not m:
        print(f"  {RED}✗ MISS: _build_current_job not found in {target.name}{RESET}")
        sys.exit(1)

    new_content = content[:m.start()] + NEW_BUILD_JOB + content[m.end():]

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731bj_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ _build_current_job fixed in print_setup.py{RESET}")

if __name__ == "__main__":
    main()
