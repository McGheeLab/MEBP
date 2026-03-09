#!/usr/bin/env python3
"""
MEBP v7.2.8 fix — Make auto-detect the default when no controller is specified.

Root cause: _load_protocol(None) loads ProScan III and ONLY searches with that
protocol. If the hardware is a ProScan II, it never gets found.

Fix: When controller_json is None, use auto-detect (try ALL protocol JSONs)
instead of assuming ProScan III.
"""

import ast
import shutil
import sys
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def find_root(hint=None):
    if hint:
        p = Path(hint)
        if (p / "SupportClasses").is_dir():
            return p
    for d in [Path(__file__).resolve().parent] + list(Path(__file__).resolve().parents):
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    for loc in [
        Path.home() / "Documents" / "GitHub" / "MEBP",
        Path(r"C:\Users\mcghe\OneDrive\Documents\GitHub\MEBP"),
    ]:
        if loc.is_dir() and (loc / "SupportClasses").is_dir():
            return loc
    print(f"{RED}Cannot find MEBP root.{RESET}")
    sys.exit(1)

def read_normalized(path):
    return path.read_text(encoding="utf-8").replace("\r\n", "\n")

def write_crlf(path, content, label):
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL{RESET} {label}: {e}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v728ad_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_bytes(content.replace("\n", "\r\n").encode("utf-8"))
    return True

def main():
    print(f"\n{BOLD}{'═' * 60}{RESET}")
    print(f"{BOLD} MEBP v7.2.8 — Auto-detect as default{RESET}")
    print(f"{BOLD}{'═' * 60}{RESET}")

    hint = sys.argv[1] if len(sys.argv) > 1 else None
    root = find_root(hint)
    print(f"\nProject root: {root}")

    # ── Fix 1: XYStage._load_protocol — auto-detect when None ────
    print(f"\n{CYAN}[1] XYStage.py — Auto-detect when controller_json=None{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = read_normalized(path)

    marker = "v7.2.8: auto-detect as default"
    if marker in content:
        print(f"  {YELLOW}○ SKIP{RESET} Already patched")
    else:
        # The old code when controller_json is None: load ProScan III as default
        old_block = (
            '    def _load_protocol(self, controller_json: Optional[str]) -> None:\n'
            '        """Load controller protocol from JSON file or auto-detect."""\n'
            '        if controller_json is None:\n'
            '            # Default: ProScan III\n'
            '            default_path = Path(DEFAULT_CONTROLLERS_DIR) / "proscan_iii.json"\n'
            '            if default_path.exists():\n'
            '                try:\n'
            '                    self._protocol = ControllerProtocol.load(default_path)\n'
            '                    logger.info(f"Loaded default protocol: {self._protocol.controller_name}")\n'
            '                    return\n'
            '                except Exception as e:\n'
            '                    logger.warning(f"Failed to load default protocol: {e}")\n'
            '            # Fallback: no protocol loaded (will use hardcoded commands)\n'
            '            logger.warning("No controller protocol loaded — using hardcoded defaults")\n'
        )

        new_block = (
            '    def _load_protocol(self, controller_json: Optional[str]) -> None:\n'
            '        """Load controller protocol from JSON file or auto-detect.\n'
            '        v7.2.8: auto-detect as default when controller_json is None.\n'
            '        """\n'
            '        if controller_json is None or controller_json.lower() == "auto":\n'
            '            # v7.2.8: Auto-detect by default — try all protocols\n'
            '            # instead of assuming ProScan III. The actual detection\n'
            '            # happens in _auto_detect_controller() during _find_controller().\n'
            '            logger.info("Controller protocol set to auto-detect")\n'
            '            return\n'
        )

        if old_block in content:
            content = content.replace(old_block, new_block, 1)
            if write_crlf(path, content, "XYStage._load_protocol"):
                print(f"  {GREEN}✓ OK{RESET}   _load_protocol: None now triggers auto-detect")
            # Re-read for next fix
            content = read_normalized(path)
        else:
            print(f"  {RED}✗ MISS{RESET} _load_protocol old block not found")
            # Try to see what's there
            if "def _load_protocol" in content:
                import re
                m = re.search(r'def _load_protocol\(self.*?\n(.*?)(?=\n    def )', content, re.DOTALL)
                if m:
                    print(f"    Found method at char {m.start()}, first 200 chars:")
                    print(f"    {m.group(0)[:200]!r}")

    # ── Fix 2: Also fix the __init__ path for controller_json=None ──
    print(f"\n{CYAN}[2] XYStage.__init__ — Load protocol for sim mode too{RESET}")
    content = read_normalized(path)

    marker2 = "v7.2.8: always load protocol"
    if marker2 in content:
        print(f"  {YELLOW}○ SKIP{RESET} Already patched")
    else:
        # Current code only loads protocol when controller_json is not None
        # or when not simulating. We need it to ALWAYS load (for auto-detect)
        old_init = (
            '        if controller_json is not None:\n'
            '            self._load_protocol(controller_json)\n'
            '            self._apply_protocol_parameters()\n'
            '        elif not simulate:\n'
            '            # Real hardware without explicit protocol → try default/auto-detect\n'
            '            self._load_protocol(controller_json)\n'
            '            self._apply_protocol_parameters()\n'
        )
        new_init = (
            '        # v7.2.8: always load protocol (auto-detect for None/auto,\n'
            '        # explicit path otherwise). Sim mode also needs protocol\n'
            '        # for parameters like microsteps_per_micron.\n'
            '        self._load_protocol(controller_json)\n'
            '        self._apply_protocol_parameters()\n'
        )

        if old_init in content:
            content = content.replace(old_init, new_init, 1)
            if write_crlf(path, content, "XYStage.__init__"):
                print(f"  {GREEN}✓ OK{RESET}   __init__: always calls _load_protocol")
        else:
            print(f"  {YELLOW}○ SKIP{RESET} __init__ protocol loading pattern not found (may differ)")

    # ── Fix 3: Ensure main.py passes controller_json ──
    print(f"\n{CYAN}[3] main.py — Verify controller_json is passed{RESET}")
    main_path = root / "main.py"
    main_content = read_normalized(main_path)

    if "controller_json=" in main_content and "v7.2.8" in main_content:
        print(f"  {YELLOW}○ SKIP{RESET} Already passes controller_json")
    elif "controller_json=" in main_content:
        print(f"  {YELLOW}○ SKIP{RESET} controller_json already present (from prior patch)")
    else:
        old_ctor = (
            '    controller = StageController(\n'
            '        simulate_xy=simulate_xy,\n'
            '        simulate_zp=simulate_zp,\n'
            '    )'
        )
        new_ctor = (
            '    # v7.2.8: Pass controller_json for hardware auto-detect\n'
            '    controller_json = settings.get("controller.controller_json", "auto")\n'
            '\n'
            '    controller = StageController(\n'
            '        simulate_xy=simulate_xy,\n'
            '        simulate_zp=simulate_zp,\n'
            '        controller_json=controller_json,\n'
            '    )'
        )
        if old_ctor in main_content:
            main_content = main_content.replace(old_ctor, new_ctor, 1)
            if write_crlf(main_path, main_content, "main.py"):
                print(f"  {GREEN}✓ OK{RESET}   main.py: controller_json='auto' passed")
        else:
            print(f"  {RED}✗ MISS{RESET} StageController constructor not found in main.py")

    print(f"\n{'═' * 60}")
    print(f"{GREEN}Done.{RESET} Restart the app to test auto-detection.")
    print(f"Expected log: 'Controller protocol set to auto-detect'")
    print(f"Then: 'Auto-detecting controller from N protocol files'")
    print(f"Then: 'Prior ProScan II found on COM4 @ 38400 baud'")

if __name__ == "__main__":
    main()
