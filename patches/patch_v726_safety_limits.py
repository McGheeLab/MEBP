#!/usr/bin/env python3
"""
MEBP v7.2.6 Patch — Safety Limits Bug Fixes

Fixes 6 bugs in the Z/pump safety limit system:

  BUG 1: Safety limits not loaded from settings.json on startup (PERSISTENCE)
         → main.py: Load safety_limits section and apply to controller

  BUG 2: Single pump spinbox forces P1=P2=P3 limits
         → settings_page.py: Add per-pump limit spinboxes (P1/P2/P3 min/max)

  BUG 3: No per-pump zero reset
         → settings_page.py: Add per-pump "Reset Zero" buttons
         → StageController.py: Add reset_pump_zero() method

  BUG 4: No "Set Pump from Current" buttons
         → settings_page.py: Add quick-set buttons for each pump

  BUG 5: ZPJogHandler clamping ignores zero reference
         → StageController.py: Subtract zero_position before clamping in _jog_loop

  BUG 6: Dashboard only shows P1 safety limits
         → dashboard.py: Show P1/P2/P3 limits when they differ

Files modified:
  - main.py
  - SupportClasses/StageController.py
  - gui/pages/settings_page.py
  - gui/pages/dashboard.py
"""

import ast
import re
import sys
from pathlib import Path


# ═══════════════════════════════════════════════════════════════════
# Terminal Colors
# ═══════════════════════════════════════════════════════════════════

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

ok_count = 0
skip_count = 0
miss_count = 0


def ok(msg):
    global ok_count
    ok_count += 1
    print(f"  {GREEN}✓ OK{RESET}   {msg}")


def skip(msg):
    global skip_count
    skip_count += 1
    print(f"  {YELLOW}○ SKIP{RESET} {msg}")


def miss(msg):
    global miss_count
    miss_count += 1
    print(f"  {RED}✗ MISS{RESET} {msg}")


# ═══════════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════════

def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path("/Users/alexmcghee/Documents/GitHub/MEBP"),
        Path.cwd(),
        Path(__file__).resolve().parent.parent.parent,
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def safe_read(path: Path) -> str:
    """Read file, return empty string if missing."""
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → write. Returns False on AST failure."""
    if path.suffix == ".py":
        try:
            ast.parse(content)
        except SyntaxError as e:
            print(f"  {RED}AST FAIL{RESET} for {label}: {e}")
            return False
    path.write_text(content, encoding="utf-8")
    return True


def find_method(content: str, name: str):
    """Find method boundaries using regex. Returns match or None."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
# BUG 1: main.py — Load safety_limits from settings on startup
# ═══════════════════════════════════════════════════════════════════

def fix_bug1_main_persistence(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 1: Safety limits persistence in main.py{RESET}")

    path = root / "main.py"
    content = safe_read(path)
    if not content:
        miss("main.py not found")
        return

    marker = "v7.2.6: Load safety_limits from settings"

    if marker in content:
        skip("Safety limits loading already applied")
        return

    # Find the zero_position loading block and add safety_limits loading after it
    # Pattern: look for the saved_zero block
    pattern = re.compile(
        r'(    saved_zero = settings\.get_section\("zero_position"\)\n'
        r'    if saved_zero:\n'
        r'        controller\.zero_position\.update\(saved_zero\))\n',
        re.MULTILINE
    )

    match = pattern.search(content)
    if not match:
        miss("Cannot find saved_zero block in main.py")
        return

    inject = (
        "\n"
        "    # v7.2.6: Load safety_limits from settings\n"
        "    saved_limits = settings.get_section(\"safety_limits\")\n"
        "    if saved_limits:\n"
        "        from SupportClasses.SafetyLimits import SafetyLimits\n"
        "        controller.safety_limits = SafetyLimits.from_dict(saved_limits)\n"
        "        import logging as _log\n"
        "        _log.getLogger(__name__).info(\n"
        "            f\"Safety limits loaded from settings \"\n"
        "            f\"(enabled={controller.safety_limits.enabled})\")\n"
    )

    content = content[:match.end()] + inject + content[match.end():]

    if safe_write(path, content, "main.py"):
        ok("Added safety_limits loading after zero_position in main.py")
    else:
        miss("AST verification failed for main.py")


# ═══════════════════════════════════════════════════════════════════
# BUG 5: StageController.py — ZPJogHandler clamping with zero ref
# ═══════════════════════════════════════════════════════════════════

def fix_bug5_jog_handler_zero_ref(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 5: ZPJogHandler clamping zero reference{RESET}")

    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        miss("StageController.py not found")
        return

    marker = "v7.2.6: Subtract zero_position before clamping"

    if marker in content:
        skip("ZPJogHandler zero-ref fix already applied")
        return

    # Find the broken clamping block inside _jog_loop
    # Pattern: the block that does cz, cp1, cp2, cp3 = pos then clamp_z(cz + dz)
    # without subtracting zero reference
    old_pattern = re.compile(
        r'(if\s+self\.safety_limits\s+and\s+self\.safety_limits\.enabled\s+and\s+self\._get_zp_position:\s*\n'
        r'\s+try:\s*\n'
        r'\s+pos\s*=\s*self\._get_zp_position\(\)\s*\n'
        r'\s+if\s+pos\[0\]\s+is\s+not\s+None:\s*\n'
        r'\s+cz,\s*cp1,\s*cp2,\s*cp3\s*=\s*pos\s*\n'
        r'\s+dz\s*=\s*self\.safety_limits\.clamp_z\(cz\s*\+\s*dz\)\s*-\s*cz\s*\n'
        r'\s+dp1\s*=\s*self\.safety_limits\.clamp_pump\(cp1\s*\+\s*dp1,\s*"P1"\)\s*-\s*cp1\s*\n'
        r'\s+dp2\s*=\s*self\.safety_limits\.clamp_pump\(cp2\s*\+\s*dp2,\s*"P2"\)\s*-\s*cp2\s*\n'
        r'\s+dp3\s*=\s*self\.safety_limits\.clamp_pump\(cp3\s*\+\s*dp3,\s*"P3"\)\s*-\s*cp3\s*\n'
        r'\s+except\s+Exception:\s*\n'
        r'\s+pass)',
        re.DOTALL
    )

    match = old_pattern.search(content)
    if not match:
        miss("Cannot find ZPJogHandler clamping block (may have different formatting)")
        return

    new_block = (
        "if self.safety_limits and self.safety_limits.enabled and self._get_zp_position:\n"
        "                try:\n"
        "                    pos = self._get_zp_position()\n"
        "                    if pos[0] is not None:\n"
        "                        cz, cp1, cp2, cp3 = pos\n"
        "                        # v7.2.6: Subtract zero_position before clamping\n"
        "                        # clamp_z/clamp_pump expect zero-referenced values\n"
        "                        _zp = getattr(self, '_zero_position', {})\n"
        "                        z0 = _zp.get('Z', 0.0)\n"
        "                        # target_zr = (abs_pos + delta) - zero; clamp; new_delta = clamped_abs - abs_pos\n"
        "                        dz = self.safety_limits.clamp_z((cz + dz) - z0) + z0 - cz\n"
        "                        p10 = _zp.get('P1', 0.0)\n"
        "                        dp1 = self.safety_limits.clamp_pump((cp1 + dp1) - p10, \"P1\") + p10 - cp1\n"
        "                        p20 = _zp.get('P2', 0.0)\n"
        "                        dp2 = self.safety_limits.clamp_pump((cp2 + dp2) - p20, \"P2\") + p20 - cp2\n"
        "                        p30 = _zp.get('P3', 0.0)\n"
        "                        dp3 = self.safety_limits.clamp_pump((cp3 + dp3) - p30, \"P3\") + p30 - cp3\n"
        "                except Exception:\n"
        "                    pass"
    )

    content = content[:match.start()] + new_block + content[match.end():]

    if safe_write(path, content, "StageController.py [jog zero-ref]"):
        ok("Fixed ZPJogHandler clamping to use zero-referenced positions")
    else:
        miss("AST verification failed for StageController.py")


# ═══════════════════════════════════════════════════════════════════
# BUG 5b: Pass zero_position to ZPJogHandler
# ═══════════════════════════════════════════════════════════════════

def fix_bug5b_jog_handler_zero_position_attr(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 5b: Pass zero_position to ZPJogHandler{RESET}")

    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        miss("StageController.py not found")
        return

    marker = "v7.2.6: Pass zero_position ref to ZPJogHandler"

    if marker in content:
        skip("ZPJogHandler zero_position attr already applied")
        return

    # Find ZPJogHandler __init__ and add _zero_position attribute
    # Look for the line that sets self._get_zp_position = get_zp_position in ZPJogHandler.__init__
    pattern = re.compile(
        r'(class ZPJogHandler:.*?def __init__\(\s*self,\s*\n\s*processor.*?\n'
        r'\s*zp_stage.*?\n'
        r'\s*safety_limits.*?\n'
        r'\s*get_zp_position.*?\n'
        r'\s*\):)',
        re.DOTALL
    )
    match = pattern.search(content)
    if not match:
        miss("Cannot find ZPJogHandler.__init__ signature")
        return

    # Replace the __init__ signature to add zero_position parameter
    old_sig = match.group(0)
    if "zero_position" in old_sig:
        skip("ZPJogHandler already has zero_position parameter")
        return

    new_sig = old_sig.replace(
        "get_zp_position: Callable | None = None,\n    ):",
        "get_zp_position: Callable | None = None,\n"
        "        zero_position: dict | None = None,  # v7.2.6: Pass zero_position ref to ZPJogHandler\n"
        "    ):"
    )

    if new_sig == old_sig:
        # Try alternate formatting
        new_sig = old_sig.replace(
            "get_zp_position: Callable | None = None,",
            "get_zp_position: Callable | None = None,\n"
            "        zero_position: dict | None = None,  # v7.2.6: zero ref for clamping"
        )

    content = content.replace(old_sig, new_sig)

    # Now add the attribute assignment after self._get_zp_position = get_zp_position
    assign_pattern = re.compile(
        r'(self\._get_zp_position\s*=\s*get_zp_position)\n',
    )
    assign_match = assign_pattern.search(content, match.start())
    if assign_match:
        inject = (
            "\n        self._zero_position = zero_position or {}  "
            "# v7.2.6: Pass zero_position ref to ZPJogHandler\n"
        )
        content = content[:assign_match.end()] + inject + content[assign_match.end():]
    else:
        miss("Cannot find self._get_zp_position assignment in ZPJogHandler")
        return

    # Now find where ZPJogHandler is created in connect_stages / _create_zp_jog and pass zero_position
    # Pattern: ZPJogHandler( ... get_zp_position=...
    create_pattern = re.compile(
        r'(self\.zp_jog\s*=\s*ZPJogHandler\(\s*\n'
        r'.*?get_zp_position\s*=.*?\n'
        r'\s*\))',
        re.DOTALL
    )
    create_match = create_pattern.search(content)
    if create_match:
        old_create = create_match.group(0)
        if "zero_position=" not in old_create:
            # Insert zero_position= before the closing paren
            new_create = old_create.rstrip(")")
            new_create += ",\n            zero_position=self.zero_position,\n        )"
            content = content.replace(old_create, new_create)
            ok("Passed zero_position to ZPJogHandler constructor")
        else:
            skip("zero_position already passed to ZPJogHandler")
    else:
        # Try a simpler pattern - sometimes it's on fewer lines
        create_pattern2 = re.compile(
            r'(ZPJogHandler\([^)]*get_zp_position\s*=[^)]*\))',
            re.DOTALL
        )
        create_match2 = create_pattern2.search(content)
        if create_match2:
            old_create = create_match2.group(0)
            if "zero_position=" not in old_create:
                new_create = old_create.rstrip(")")
                new_create += ", zero_position=self.zero_position)"
                content = content.replace(old_create, new_create)
                ok("Passed zero_position to ZPJogHandler constructor (single-line)")
            else:
                skip("zero_position already passed")
        else:
            miss("Cannot find ZPJogHandler creation site")

    if safe_write(path, content, "StageController.py [jog zero_position attr]"):
        ok("Added _zero_position attribute to ZPJogHandler")
    else:
        miss("AST verification failed for StageController.py")


# ═══════════════════════════════════════════════════════════════════
# BUG 3: StageController.py — Add reset_pump_zero() method
# ═══════════════════════════════════════════════════════════════════

def fix_bug3_pump_zero_reset(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 3: Add reset_pump_zero() to StageController{RESET}")

    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        miss("StageController.py not found")
        return

    marker = "def reset_pump_zero("

    if marker in content:
        skip("reset_pump_zero() already exists")
        return

    # Find a good insertion point — after move_pump_relative or after _calibrate_zero
    # Look for _calibrate_zero method
    cal_method = find_method(content, "_calibrate_zero")
    if cal_method:
        insert_pos = cal_method.end()
    else:
        # Fallback: find end of move_pump_relative
        mpr = find_method(content, "move_pump_relative")
        if mpr:
            insert_pos = mpr.end()
        else:
            miss("Cannot find insertion point for reset_pump_zero")
            return

    new_method = '''
    def reset_pump_zero(self, pump: str) -> None:
        """
        v7.2.6: Reset zero reference for a single pump axis.

        Sets the current absolute position as the new zero point
        for the specified pump, without affecting other axes.

        Args:
            pump: Pump identifier ("P1", "P2", "P3")
        """
        if pump not in ("P1", "P2", "P3"):
            logger.warning(f"Invalid pump ID for zero reset: {pump}")
            return

        pos = self.get_zp_position(cached=True)
        if pos[0] is None:
            logger.warning(f"Cannot reset {pump} zero — ZP stage not connected")
            return

        idx = {"P1": 1, "P2": 2, "P3": 3}[pump]
        if idx < len(pos) and pos[idx] is not None:
            self.zero_position[pump] = pos[idx]
            logger.info(f"{pump} zero set to {pos[idx]:.3f} mm (absolute)")

            # Log the event
            self.position_logger.record(
                f"zero_reset_{pump.lower()}",
                zp_pos=pos,
                metadata={"pump": pump, "new_zero": pos[idx]},
            )

    def reset_z_zero(self) -> None:
        """
        v7.2.6: Reset zero reference for Z axis only.

        Sets the current absolute position as the new zero point
        for Z, without affecting XY or pump axes.
        """
        pos = self.get_zp_position(cached=True)
        if pos[0] is None:
            logger.warning("Cannot reset Z zero — ZP stage not connected")
            return

        self.zero_position["Z"] = pos[0]
        logger.info(f"Z zero set to {pos[0]:.3f} mm (absolute)")

        self.position_logger.record(
            "zero_reset_z",
            zp_pos=pos,
            metadata={"new_zero": pos[0]},
        )

'''

    content = content[:insert_pos] + new_method + content[insert_pos:]

    if safe_write(path, content, "StageController.py [reset_pump_zero]"):
        ok("Added reset_pump_zero() and reset_z_zero() methods")
    else:
        miss("AST verification failed for StageController.py")


# ═══════════════════════════════════════════════════════════════════
# BUG 2 + BUG 4: settings_page.py — Per-pump limits & quick-set
# ═══════════════════════════════════════════════════════════════════

def fix_bug2_bug4_settings_page(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 2+4: Per-pump limits & quick-set in settings_page.py{RESET}")

    path = root / "gui" / "pages" / "settings_page.py"
    content = safe_read(path)
    if not content:
        miss("settings_page.py not found")
        return

    marker_ui = "v7.2.6: Per-pump limit spinboxes"

    if marker_ui in content:
        skip("Per-pump limit spinboxes already applied")
        return

    # ── Step A: Replace the single pump limit UI with per-pump spinboxes ──
    # Find the pump limits section:  "Pump Min:" label + spin_p_min + "Pump Max:" + spin_p_max
    pump_ui_pattern = re.compile(
        r'(        # Pump limits\n'
        r'        grid\.addWidget\(QLabel\("Pump Min:"\).*?'
        r'grid\.addWidget\(self\.spin_p_max.*?\n)',
        re.DOTALL
    )

    match_ui = pump_ui_pattern.search(content)
    if not match_ui:
        # Try alternate: look for spin_p_min creation
        pump_ui_pattern2 = re.compile(
            r'(grid\.addWidget\(QLabel\("Pump Min:"\),\s*row,\s*0\).*?'
            r'grid\.addWidget\(self\.spin_p_max,\s*row,\s*3\)\s*\n'
            r'\s*row\s*\+=\s*1\n)',
            re.DOTALL
        )
        match_ui = pump_ui_pattern2.search(content)

    if not match_ui:
        miss("Cannot find single pump limit UI block")
        return

    new_pump_ui = (
        "        # v7.2.6: Per-pump limit spinboxes\n"
        "        self._pump_min_spins = {}\n"
        "        self._pump_max_spins = {}\n"
        "        for pid in ['P1', 'P2', 'P3']:\n"
        "            grid.addWidget(QLabel(f'{pid} Min:'), row, 0)\n"
        "            spin_min = QDoubleSpinBox()\n"
        "            spin_min.setRange(-200, 200)\n"
        "            spin_min.setDecimals(1)\n"
        "            spin_min.setSuffix(' mm')\n"
        "            grid.addWidget(spin_min, row, 1)\n"
        "            grid.addWidget(QLabel(f'{pid} Max:'), row, 2)\n"
        "            spin_max = QDoubleSpinBox()\n"
        "            spin_max.setRange(-200, 200)\n"
        "            spin_max.setDecimals(1)\n"
        "            spin_max.setSuffix(' mm')\n"
        "            grid.addWidget(spin_max, row, 3)\n"
        "            self._pump_min_spins[pid] = spin_min\n"
        "            self._pump_max_spins[pid] = spin_max\n"
        "            row += 1\n"
        "\n"
        "        # v7.2.6: Per-pump quick-set and zero-reset buttons\n"
        "        for pid in ['P1', 'P2', 'P3']:\n"
        "            btn_row_p = QHBoxLayout()\n"
        "            btn_p_min = QPushButton(f'Set {pid} Min from Current')\n"
        "            btn_p_min.setMaximumHeight(26)\n"
        "            btn_p_min.clicked.connect(\n"
        "                lambda checked, p=pid: self._set_pump_from_current(p, as_max=False))\n"
        "            btn_row_p.addWidget(btn_p_min)\n"
        "            btn_p_max = QPushButton(f'Set {pid} Max from Current')\n"
        "            btn_p_max.setMaximumHeight(26)\n"
        "            btn_p_max.clicked.connect(\n"
        "                lambda checked, p=pid: self._set_pump_from_current(p, as_max=True))\n"
        "            btn_row_p.addWidget(btn_p_max)\n"
        "            btn_p_zero = QPushButton(f'Reset {pid} Zero')\n"
        "            btn_p_zero.setMaximumHeight(26)\n"
        "            btn_p_zero.clicked.connect(\n"
        "                lambda checked, p=pid: self._reset_pump_zero(p))\n"
        "            btn_row_p.addWidget(btn_p_zero)\n"
        "            grid.addLayout(btn_row_p, row, 0, 1, 4)\n"
        "            row += 1\n"
    )

    content = content[:match_ui.start()] + new_pump_ui + content[match_ui.end():]

    # ── Step B: Fix _load_from_controller to load per-pump values ──
    # Replace: self.spin_p_min.setValue(sl.p1_min)  /  self.spin_p_max.setValue(sl.p1_max)
    old_load_pump = re.compile(
        r'self\.spin_p_min\.setValue\(sl\.p1_min\)\n'
        r'\s*self\.spin_p_max\.setValue\(sl\.p1_max\)'
    )
    match_load = old_load_pump.search(content)
    if match_load:
        new_load_pump = (
            "# v7.2.6: Load per-pump limits\n"
            "        for pid, attr_min, attr_max in [\n"
            "            ('P1', 'p1_min', 'p1_max'),\n"
            "            ('P2', 'p2_min', 'p2_max'),\n"
            "            ('P3', 'p3_min', 'p3_max'),\n"
            "        ]:\n"
            "            if pid in self._pump_min_spins:\n"
            "                self._pump_min_spins[pid].setValue(getattr(sl, attr_min))\n"
            "                self._pump_max_spins[pid].setValue(getattr(sl, attr_max))"
        )
        content = content[:match_load.start()] + new_load_pump + content[match_load.end():]
    else:
        miss("Cannot find spin_p_min.setValue(sl.p1_min) in _load_from_controller")

    # ── Step C: Fix _apply_settings to read per-pump values ──
    # Replace the block:
    #   sl.p1_min = sl.p2_min = sl.p3_min = self.spin_p_min.value()
    #   sl.p1_max = sl.p2_max = sl.p3_max = self.spin_p_max.value()
    old_apply_pump = re.compile(
        r'sl\.p1_min\s*=\s*sl\.p2_min\s*=\s*sl\.p3_min\s*=\s*self\.spin_p_min\.value\(\)\n'
        r'\s*sl\.p1_max\s*=\s*sl\.p2_max\s*=\s*sl\.p3_max\s*=\s*self\.spin_p_max\.value\(\)'
    )
    match_apply = old_apply_pump.search(content)
    if match_apply:
        new_apply_pump = (
            "# v7.2.6: Apply per-pump limits\n"
            "        for pid, attr_min, attr_max in [\n"
            "            ('P1', 'p1_min', 'p1_max'),\n"
            "            ('P2', 'p2_min', 'p2_max'),\n"
            "            ('P3', 'p3_min', 'p3_max'),\n"
            "        ]:\n"
            "            if pid in self._pump_min_spins:\n"
            "                setattr(sl, attr_min, self._pump_min_spins[pid].value())\n"
            "                setattr(sl, attr_max, self._pump_max_spins[pid].value())"
        )
        content = content[:match_apply.start()] + new_apply_pump + content[match_apply.end():]
    else:
        # There might be a second occurrence (the code is duplicated in the search results)
        miss("Cannot find sl.p1_min = sl.p2_min = sl.p3_min = self.spin_p_min.value()")

    # ── Step D: Add _set_pump_from_current and _reset_pump_zero helper methods ──
    if "def _set_pump_from_current" not in content:
        # Find _set_z_from_current and insert after it
        z_from_current = find_method(content, "_set_z_from_current")
        if z_from_current:
            insert_pos = z_from_current.end()
        else:
            # Fallback: insert before the _update_safety_um_label method
            um_method = find_method(content, "_update_safety_um_label")
            insert_pos = um_method.start() if um_method else len(content)

        new_methods = '''
    def _set_pump_from_current(self, pump: str, as_max: bool = True):
        """v7.2.6: Set pump limit from the current position."""
        pos = self.controller.get_zp_position(cached=True)
        if pos[0] is not None:
            idx = {"P1": 1, "P2": 2, "P3": 3}.get(pump, 1)
            if idx < len(pos) and pos[idx] is not None:
                zero_ref = self.controller.zero_position.get(pump, 0)
                rel_mm = pos[idx] - zero_ref
                if as_max:
                    self._pump_max_spins[pump].setValue(rel_mm)
                else:
                    self._pump_min_spins[pump].setValue(rel_mm)
                logger.info(
                    f"{pump} {'max' if as_max else 'min'} set to "
                    f"{rel_mm:.2f} mm (from current)")

    def _reset_pump_zero(self, pump: str):
        """v7.2.6: Reset the zero reference for a single pump."""
        if hasattr(self.controller, 'reset_pump_zero'):
            self.controller.reset_pump_zero(pump)
            # Save the updated zero_position to settings
            self.settings.set_section(
                "zero_position", dict(self.controller.zero_position))
            self.settings.save()
            logger.info(f"{pump} zero reference reset and saved")
            if self._context_widget:
                self.ctx_status_label.setText(f"{pump} zero reset ✓")
                self.ctx_status_label.setStyleSheet(
                    f"color: {COLORS['green']};")

'''
        content = content[:insert_pos] + new_methods + content[insert_pos:]
        ok("Added _set_pump_from_current() and _reset_pump_zero() methods")
    else:
        skip("_set_pump_from_current already exists")

    if safe_write(path, content, "settings_page.py"):
        ok("Per-pump limit UI and helpers applied")
    else:
        miss("AST verification failed for settings_page.py")


# ═══════════════════════════════════════════════════════════════════
# BUG 6: dashboard.py — Show per-pump limits when they differ
# ═══════════════════════════════════════════════════════════════════

def fix_bug6_dashboard_pump_display(root: Path):
    print(f"\n{CYAN}{BOLD}BUG 6: Dashboard per-pump safety info display{RESET}")

    path = root / "gui" / "pages" / "dashboard.py"
    content = safe_read(path)
    if not content:
        miss("dashboard.py not found")
        return

    marker = "v7.2.6: Show per-pump limits"

    if marker in content:
        skip("Dashboard per-pump display already applied")
        return

    # Find the old single-pump display line
    old_display = re.compile(
        r'self\.lbl_safety_info\.setText\(\s*\n?'
        r'\s*f"XY:.*?"P:\s*\[{sl\.p1_min.*?sl\.p1_max.*?\]"\s*\n?\s*\)'
    )

    match_display = old_display.search(content)
    if not match_display:
        # Try simpler pattern
        old_display2 = re.compile(
            r'(self\.lbl_safety_info\.setText\([^)]*P:\s*\[\{sl\.p1_min[^)]*\))'
        )
        match_display = old_display2.search(content)

    if not match_display:
        miss("Cannot find dashboard safety info display line")
        return

    new_display = (
        "# v7.2.6: Show per-pump limits\n"
        "            pump_parts = []\n"
        "            for pid, pmin, pmax in [\n"
        "                ('P1', sl.p1_min, sl.p1_max),\n"
        "                ('P2', sl.p2_min, sl.p2_max),\n"
        "                ('P3', sl.p3_min, sl.p3_max),\n"
        "            ]:\n"
        "                pump_parts.append(f'{pid}:[{pmin:.1f}..{pmax:.1f}]')\n"
        "            all_same = (sl.p1_min == sl.p2_min == sl.p3_min\n"
        "                        and sl.p1_max == sl.p2_max == sl.p3_max)\n"
        "            pump_str = (f'P:[{sl.p1_min:.1f}..{sl.p1_max:.1f}]'\n"
        "                        if all_same else '  '.join(pump_parts))\n"
        "            self.lbl_safety_info.setText(\n"
        "                f\"XY: [{sl.xy_min_x:.0f}..{sl.xy_max_x:.0f}] × \"\n"
        "                f\"[{sl.xy_min_y:.0f}..{sl.xy_max_y:.0f}]  \"\n"
        "                f\"Z: [{sl.z_min:.1f}..{sl.z_max:.1f}]  \"\n"
        "                f\"{pump_str}\"\n"
        "            )"
    )

    content = content[:match_display.start()] + new_display + content[match_display.end():]

    if safe_write(path, content, "dashboard.py"):
        ok("Dashboard now shows per-pump limits when they differ")
    else:
        miss("AST verification failed for dashboard.py")


# ═══════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{BOLD}{'═' * 60}")
    print(f"  MEBP v7.2.6 Patch — Safety Limits Bug Fixes")
    print(f"{'═' * 60}{RESET}\n")

    root = find_root()
    print(f"Project root: {root}\n")

    # Apply fixes in dependency order
    fix_bug1_main_persistence(root)
    fix_bug5_jog_handler_zero_ref(root)
    fix_bug5b_jog_handler_zero_position_attr(root)
    fix_bug3_pump_zero_reset(root)
    fix_bug2_bug4_settings_page(root)
    fix_bug6_dashboard_pump_display(root)

    # Summary
    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'─' * 60}")
    print(f"  Summary: {GREEN}{ok_count} OK{RESET}  {YELLOW}{skip_count} SKIP{RESET}  {RED}{miss_count} MISS{RESET}  ({total} total)")
    print(f"{'─' * 60}{RESET}\n")

    if miss_count > 0:
        print(f"{YELLOW}⚠ Some patches had MISS results — review output above.{RESET}")
        print(f"  MISS usually means file formatting differs from expected.")
        print(f"  Check the specific file manually.\n")
        sys.exit(1)
    else:
        print(f"{GREEN}All patches applied successfully!{RESET}\n")


if __name__ == "__main__":
    main()
