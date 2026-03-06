#!/usr/bin/env python3
"""
patch_fix_build_job_v731.py
Fix _build_current_job in print_setup.py.

The existing call:
    build_well_plate_job(plate=plate, selected_wells=print_wells, settings=settings)

is wrong — build_well_plate_job's real signature is:
    build_well_plate_job(well_positions, path_points, settings, ...)

where well_positions = list of (name, x_mm, y_mm) tuples.
The plate object provides coordinates via plate.get_well_position(name) -> (x, y).
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
        """Build a print job — v7.3.1: use correct build_well_plate_job API."""
        if not (hasattr(self, "tab_wells") and self.tab_wells._model):
            logger.warning("No well model available")
            return None

        model = self.tab_wells._model
        plate = model.plate
        settings = self._get_settings()

        # Collect print wells
        print_wells = [
            name for name, a in model.assignments.items()
            if getattr(getattr(a, "role", None), "value", None) == "print"
        ]

        if not print_wells:
            logger.warning("No print wells assigned")
            return None

        if plate is None:
            logger.warning("No plate geometry available")
            return None

        # Build well_positions: list of (name, x_mm, y_mm)
        try:
            well_positions = []
            for name in print_wells:
                try:
                    x, y = plate.get_well_position(name)
                except Exception:
                    x, y = 0.0, 0.0
                well_positions.append((name, x, y))

            # Default path: single center point per well
            path_points = [(0.0, 0.0)]

            return build_well_plate_job(
                well_positions=well_positions,
                path_points=path_points,
                settings=settings,
            )
        except TypeError:
            # Older API may use positional args only
            try:
                well_positions = []
                for name in print_wells:
                    try:
                        x, y = plate.get_well_position(name)
                    except Exception:
                        x, y = 0.0, 0.0
                    well_positions.append((name, x, y))
                return build_well_plate_job(well_positions, [(0.0, 0.0)], settings)
            except Exception as exc:
                logger.error(f"build_well_plate_job failed: {exc}", exc_info=True)
                return None
        except Exception as exc:
            logger.error(f"_build_current_job error: {exc}", exc_info=True)
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

    guard = "v7.3.1: use correct build_well_plate_job API"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}")
        return

    m = find_method(content, "_build_current_job")
    if not m:
        print(f"  {RED}✗ MISS: _build_current_job not found in print_setup.py{RESET}")
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
    print(f"    Now builds well_positions list from plate geometry")

if __name__ == "__main__":
    main()
