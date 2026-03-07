#!/usr/bin/env python3
"""Fix remaining on_status_update and _set_zero in calibration.py.

Uses flexible regex that tolerates whitespace variations.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN, RED, YELLOW, CYAN, RESET, BOLD = "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[0m", "\033[1m"
ok = skip = miss = 0

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir(): return c
    sys.exit("Cannot find MEBP root")

def rpt(s, m):
    global ok, skip, miss
    if s=="OK": ok+=1; print(f"  {GREEN}✓ {m}{RESET}")
    elif s=="SKIP": skip+=1; print(f"  {YELLOW}○ {m}{RESET}")
    else: miss+=1; print(f"  {RED}✗ {m}{RESET}")

def main():
    global ok, skip, miss
    print(f"\n{BOLD}  Cal display: on_status_update + _set_zero{RESET}")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")
    changed = False

    # ── Fix 1: on_status_update ──────────────────────────────────
    # Match any variant of: steps_to_um(zx, ...) or zx / self._microsteps
    print(f"\n{CYAN}[1] on_status_update{RESET}")
    if "v7.2.7: positions are µm" in content or "v7.2.7: controller reports µm" in content:
        rpt("SKIP", "Already has v7.2.7 marker")
    else:
        # Flexible: find the block that has steps_to_um OR /microsteps in position display
        # within on_status_update context (near xy[0] and lbl_x)
        pat = re.compile(
            r'^(\s+)(\w+)\s*=\s*xy\[0\]\s*-\s*ctrl\.zero_position\["x"\]\s*\n'
            r'\s+(\w+)\s*=\s*xy\[1\]\s*-\s*ctrl\.zero_position\["y"\]\s*\n'
            r'(?:\s*#[^\n]*\n)*'  # optional comments
            r'\s+ux\s*=\s*(?:steps_to_um\(\2,\s*self\._microsteps_per_micron\)|\2\s*/\s*self\._microsteps_per_micron)\s*\n'
            r'\s+uy\s*=\s*(?:steps_to_um\(\3,\s*self\._microsteps_per_micron\)|\3\s*/\s*self\._microsteps_per_micron)\s*\n',
            re.MULTILINE
        )
        m = pat.search(content)
        if m:
            ind = m.group(1)
            content = content[:m.start()] + (
                f"{ind}# v7.2.7: positions are µm directly from controller\n"
                f"{ind}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
                f"{ind}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
            ) + content[m.end():]
            changed = True
            rpt("OK", "Fixed on_status_update")
        else:
            # Check if already displaying raw values (no conversion)
            if re.search(r'ux\s*=\s*xy\[0\]\s*-\s*ctrl\.zero_position', content):
                rpt("SKIP", "Already displays raw µm")
            else:
                # Debug: show what's near lbl_x
                idx = content.find('self.lbl_x.setText')
                if idx > 0:
                    snippet = content[max(0,idx-300):idx+100]
                    print(f"  Context near lbl_x.setText:")
                    for i, ln in enumerate(snippet.split('\n')[-8:]):
                        print(f"    {repr(ln)}")
                rpt("MISS", "Could not find on_status_update conversion")

    # ── Fix 2: _set_zero ─────────────────────────────────────────
    print(f"\n{CYAN}[2] _set_zero{RESET}")
    if "v7.2.7: zero positions already" in content:
        rpt("SKIP", "Already fixed")
    else:
        # Match: zx_um = steps_to_um(z['x'], ...) or z['x'] / ...
        pat2 = re.compile(
            r"^(\s+)zx_um\s*=\s*steps_to_um\(z\[.x.\],\s*self\._microsteps_per_micron\)\s*\n"
            r"\s+zy_um\s*=\s*steps_to_um\(z\[.y.\],\s*self\._microsteps_per_micron\)",
            re.MULTILINE
        )
        m2 = pat2.search(content)
        if m2:
            ind = m2.group(1)
            content = content[:m2.start()] + (
                f"{ind}# v7.2.7: zero positions already in µm\n"
                f"{ind}zx_um = z['x']\n"
                f"{ind}zy_um = z['y']"
            ) + content[m2.end():]
            changed = True
            rpt("OK", "Fixed _set_zero")
        else:
            # Check if already raw
            if re.search(r"zx_um\s*=\s*z\['x'\]", content):
                rpt("SKIP", "Already uses raw values")
            else:
                # Debug
                m_zero = re.search(r'def _set_zero\(self\)', content)
                if m_zero:
                    snippet = content[m_zero.start():m_zero.start()+400]
                    print(f"  _set_zero body:")
                    for ln in snippet.split('\n')[:12]:
                        print(f"    {repr(ln)}")
                rpt("MISS", "Could not find _set_zero conversion")

    if not changed:
        print(f"\n  {YELLOW}No changes needed{RESET}")
        return 0

    # AST + write
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727df_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"\n  {GREEN}✓ Written + AST verified{RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
