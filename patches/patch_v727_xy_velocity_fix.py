#!/usr/bin/env python3
"""
v7.2.7 Patch: Fix XY velocity being way too slow during prints.

Root causes fixed:
  1. print_feedrate never set from GUI (GUI value ignored)
  2. print_feedrate treated as mm/min but GUI shows mm/s
  3. Prior stage SMS (max speed) never set before printing
  4. TrajectoryExecutor serial overhead from redundant axis commands
  5. _wait_for_xy_settle can stall on slow Prior moves

All changes use v7.2.7 markers for idempotency.
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colors ──────────────────────────────────────────────
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"
BOLD = "\033[1m"

ok_count = 0
skip_count = 0
miss_count = 0


def find_root() -> Path:
    """Find MEBP project root."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent,
        Path.cwd(),
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP root (need SupportClasses/ + gui/){RESET}")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → backup → write."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL ({label}): {e}{RESET}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v727_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    return True


def find_method(content: str, name: str):
    """Find a class method by name. Returns match with full body."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def report(status: str, msg: str):
    global ok_count, skip_count, miss_count
    if status == "OK":
        ok_count += 1
        print(f"  {GREEN}✓ {msg}{RESET}")
    elif status == "SKIP":
        skip_count += 1
        print(f"  {YELLOW}○ SKIP: {msg}{RESET}")
    else:
        miss_count += 1
        print(f"  {RED}✗ MISS: {msg}{RESET}")


# ═══════════════════════════════════════════════════════════════════
#  PATCH A: PrintSettings — add print_speed_mm_s / travel_speed_mm_s
# ═══════════════════════════════════════════════════════════════════

def patch_A_print_settings(root: Path):
    """Add mm/s speed fields to PrintSettings dataclass."""
    print(f"\n{CYAN}[A] PrintManager.py — PrintSettings mm/s fields{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: print_speed_mm_s"

    if marker in content:
        report("SKIP", "print_speed_mm_s already present")
        return content

    # Find the pump_rate_uL_s line in PrintSettings to inject after
    # We look for the pump_rate_uL_s field definition
    pattern = re.compile(
        r'^(\s+pump_rate_uL_s:\s*float\s*=\s*[\d.]+\s*#.*?)$',
        re.MULTILINE
    )
    match = pattern.search(content)

    if not match:
        # Fallback: look for any line with "pump_rate_uL_s" in the dataclass
        pattern2 = re.compile(
            r'^(\s+pump_rate_uL_s:\s*float\s*=\s*[\d.]+).*?$',
            re.MULTILINE
        )
        match = pattern2.search(content)

    if not match:
        report("MISS", "Could not find pump_rate_uL_s field in PrintSettings")
        return content

    new_fields = (
        "\n"
        "    # v7.2.7: print_speed_mm_s — explicit mm/s speed from GUI\n"
        "    print_speed_mm_s: float = 5.0        # XY speed during printing (mm/s)\n"
        "    travel_speed_mm_s: float = 10.0       # XY speed during travel (mm/s)\n"
    )

    insert_pos = match.end()
    content = content[:insert_pos] + new_fields + content[insert_pos:]
    report("OK", "Added print_speed_mm_s and travel_speed_mm_s to PrintSettings")

    if not safe_write(path, content, "PrintSettings fields"):
        return safe_read(path)  # re-read original on failure
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH B: Fix _execute_print_path — use mm/s, set Prior speed
# ═══════════════════════════════════════════════════════════════════

def patch_B_execute_print_path(root: Path):
    """Fix _execute_print_path to use mm/s and set Prior SMS."""
    print(f"\n{CYAN}[B] PrintManager.py — Fix _execute_print_path speed{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: speed_mm_s"

    if marker in content:
        report("SKIP", "_execute_print_path already patched")
        return content

    # --- Sub-patch B1: Replace the speed calculation in _execute_print_path ---
    # Find the old pattern: move_time = seg_length / max(settings.print_feedrate, 1) * 60
    old_sleep = re.compile(
        r'move_time\s*=\s*seg_length\s*/\s*max\(settings\.print_feedrate,\s*1\)\s*\*\s*60'
    )
    match_sleep = old_sleep.search(content)

    if match_sleep:
        new_sleep = (
            "# v7.2.7: speed_mm_s — use explicit mm/s, fallback to legacy mm/min\n"
            "            _speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)\n"
            "            if _speed_mm_s <= 0:\n"
            "                _speed_mm_s = max(settings.print_feedrate, 1) / 60.0\n"
            "            move_time = seg_length / max(_speed_mm_s, 0.01)"
        )
        content = content[:match_sleep.start()] + new_sleep + content[match_sleep.end():]
        report("OK", "Fixed sleep time calculation to use mm/s")
    else:
        report("MISS", "Could not find move_time formula in _execute_print_path")

    # --- Sub-patch B2: Replace the extrusion time calc ---
    # Find: xy_speed = max(settings.print_feedrate, 1.0)
    #        seg_time = seg_length / (xy_speed / 60.0) if xy_speed > 0 else 0
    old_ext_speed = re.compile(
        r'xy_speed\s*=\s*max\(settings\.print_feedrate,\s*1\.0\)\s*\n'
        r'\s*seg_time\s*=\s*seg_length\s*/\s*\(xy_speed\s*/\s*60\.0\)',
        re.DOTALL
    )
    match_ext = old_ext_speed.search(content)

    if match_ext:
        new_ext = (
            "# v7.2.7: Use mm/s for extrusion timing\n"
            "                _ext_speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)\n"
            "                if _ext_speed_mm_s <= 0:\n"
            "                    _ext_speed_mm_s = max(settings.print_feedrate, 1.0) / 60.0\n"
            "                seg_time = seg_length / max(_ext_speed_mm_s, 0.01)"
        )
        content = content[:match_ext.start()] + new_ext + content[match_ext.end():]
        report("OK", "Fixed extrusion timing to use mm/s")
    else:
        report("MISS", "Could not find extrusion speed formula")

    # --- Sub-patch B3: Add _set_xy_speed_for_print method ---
    marker_method = "def _set_xy_speed_for_print"
    if marker_method not in content:
        # Find _execute_print_path to insert before it
        match_method = re.search(
            r'^    def _execute_print_path\(self',
            content,
            re.MULTILINE
        )
        if match_method:
            speed_method = '''    def _set_xy_speed_for_print(self, speed_mm_s: float = 0):
        """v7.2.7: Set Prior XY stage max speed before print execution.

        Converts mm/s to the stage's native speed parameter and sends SMS.
        Must be called before any print path or trajectory execution.
        """
        settings = self.job.settings if self.job else None
        if speed_mm_s <= 0 and settings:
            speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)
        if speed_mm_s <= 0 and settings:
            speed_mm_s = max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        if speed_mm_s <= 0:
            speed_mm_s = 5.0  # safe default

        ctrl = self.controller
        if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
            # Prior SMS expects speed in µm/s (stage native units)
            speed_um_s = speed_mm_s * 1000.0
            # Add 50% headroom so stage can reach target before next command
            target = int(min(speed_um_s * 1.5, 50000))
            try:
                ctrl.xy_stage.set_velocity(target)
                logger.info(f"v7.2.7: Set XY speed: {speed_mm_s:.1f} mm/s "
                           f"→ SMS {target} µm/s (with 50% headroom)")
            except Exception as e:
                logger.warning(f"Failed to set XY speed: {e}")

'''
            content = content[:match_method.start()] + speed_method + content[match_method.start():]
            report("OK", "Added _set_xy_speed_for_print method")
        else:
            report("MISS", "Could not find _execute_print_path for insertion")
    else:
        report("SKIP", "_set_xy_speed_for_print already exists")

    # --- Sub-patch B4: Call _set_xy_speed at start of _execute_print_path ---
    # Find "Move to start of path" comment to inject speed set before it
    if "_set_xy_speed_for_print" not in content.split("def _execute_print_path")[1] if "def _execute_print_path" in content else "":
        # Look for the pattern where we move to start of path
        start_pattern = re.compile(
            r'(# Move to start of path\n\s+start_x, start_y = points\[0\]\[0\], points\[0\]\[1\])'
        )
        match_start = start_pattern.search(content)
        if match_start:
            inject = (
                "# v7.2.7: Set stage speed before print path\n"
                "        self._set_xy_speed_for_print()\n\n"
                "        "
            )
            content = content[:match_start.start()] + inject + content[match_start.start():]
            report("OK", "Injected speed set call at start of _execute_print_path")
        else:
            report("MISS", "Could not find 'Move to start of path' anchor")
    else:
        report("SKIP", "Speed set call already in _execute_print_path")

    if not safe_write(path, content, "_execute_print_path fixes"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH C: Fix TrajectoryExecutor — set speed, reduce overhead
# ═══════════════════════════════════════════════════════════════════

def patch_C_trajectory_executor(root: Path):
    """Fix TrajectoryExecutor: set Prior speed and reduce serial overhead."""
    print(f"\n{CYAN}[C] PrintManager.py — TrajectoryExecutor fixes{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: Set stage speed for trajectory"

    if marker in content:
        report("SKIP", "TrajectoryExecutor already patched")
        return content

    # Find the TrajectoryExecutor.execute method's waypoint loop start
    # We look for the line: logger.info(f"TrajectoryExecutor: starting {total} waypoints,
    traj_start = re.compile(
        r'(logger\.info\(f"TrajectoryExecutor: starting \{total\} waypoints,.*?\n)',
        re.DOTALL
    )
    match_traj = traj_start.search(content)

    if match_traj:
        speed_setup = (
            "\n"
            "        # v7.2.7: Set stage speed for trajectory\n"
            "        # Compute max speed from waypoint spacing and set Prior SMS\n"
            "        try:\n"
            "            max_speed_mm_s = 0.0\n"
            "            for j in range(1, min(len(waypoints), 100)):\n"
            "                dt_wp = waypoints[j].t - waypoints[j-1].t\n"
            "                if dt_wp > 1e-6:\n"
            "                    dx = waypoints[j].x - waypoints[j-1].x\n"
            "                    dy = waypoints[j].y - waypoints[j-1].y\n"
            "                    import math as _math\n"
            "                    spd = _math.sqrt(dx*dx + dy*dy) / dt_wp\n"
            "                    max_speed_mm_s = max(max_speed_mm_s, spd)\n"
            "            if max_speed_mm_s > 0 and ctrl.is_xy_connected:\n"
            "                headroom = max_speed_mm_s * 1.5 * 1000.0  # mm/s → µm/s + 50%\n"
            "                speed_val = int(min(headroom, 50000))\n"
            "                if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:\n"
            "                    ctrl.xy_stage.set_velocity(speed_val)\n"
            "                    logger.info(f'v7.2.7: Trajectory max speed: '\n"
            "                               f'{max_speed_mm_s:.1f} mm/s, SMS set to {speed_val}')\n"
            "        except Exception as e:\n"
            "            logger.warning(f'v7.2.7: Could not set trajectory speed: {e}')\n"
            "\n"
            "        # v7.2.7: Track previous axis values to skip unchanged commands\n"
            "        _prev_z = None\n"
            "        _prev_p = [None, None, None]\n"
            "\n"
        )
        insert_pos = match_traj.end()
        content = content[:insert_pos] + speed_setup + content[insert_pos:]
        report("OK", "Added speed setup and axis tracking to TrajectoryExecutor")
    else:
        report("MISS", "Could not find TrajectoryExecutor log line anchor")

    # --- Sub-patch C2: Add Z/pump skip logic in the waypoint loop ---
    # Find the Z axis movement block and wrap it with a change check
    # Look for: # Z axis
    #           if ctrl.is_zp_connected:
    #               ctrl.move_z_absolute(wp.z, from_zero_ref=True)
    z_block = re.compile(
        r'(# Z axis\n\s+if ctrl\.is_zp_connected:\n'
        r'\s+ctrl\.move_z_absolute\(wp\.z, from_zero_ref=True\))',
        re.DOTALL
    )
    match_z = z_block.search(content)

    if match_z:
        new_z = (
            "# Z axis — v7.2.7: skip if unchanged\n"
            "            if ctrl.is_zp_connected:\n"
            "                if _prev_z is None or abs(wp.z - _prev_z) > 0.001:\n"
            "                    ctrl.move_z_absolute(wp.z, from_zero_ref=True)\n"
            "                    _prev_z = wp.z"
        )
        content = content[:match_z.start()] + new_z + content[match_z.end():]
        report("OK", "Added Z-axis skip-if-unchanged logic")
    else:
        # Try without the "# Z axis" comment (might not be there exactly)
        report("MISS", "Could not find Z axis block in TrajectoryExecutor (non-critical)")

    # --- Sub-patch C3: Wrap pump moves with change-check ---
    # Find the pump loop: for pump_id, wp_val in [("P1", wp.p1), ("P2", wp.p2), ("P3", wp.p3)]:
    pump_loop = re.compile(
        r'for pump_id, wp_val in \[\("P1", wp\.p1\), \("P2", wp\.p2\), \("P3", wp\.p3\)\]:\n'
        r'\s+mapped = AXIS_MAP\.get\(pump_id\)\n'
        r'\s+if mapped and wp_val != 0\.0:',
        re.DOTALL
    )
    match_pump = pump_loop.search(content)

    if match_pump:
        new_pump = (
            'for _pidx, (pump_id, wp_val) in enumerate([("P1", wp.p1), ("P2", wp.p2), ("P3", wp.p3)]):\n'
            '                    mapped = AXIS_MAP.get(pump_id)\n'
            '                    # v7.2.7: skip if pump value unchanged\n'
            '                    if mapped and wp_val != 0.0 and (_prev_p[_pidx] is None or abs(wp_val - _prev_p[_pidx]) > 0.001):'
        )
        content = content[:match_pump.start()] + new_pump + content[match_pump.end():]
        # Also need to update _prev_p after the pump move
        # Find the pump absolute move line and add tracking after it
        pump_move = re.compile(
            r'(ctrl\.zp_stage\.move_absolute\(\n'
            r'\s+\{mapped: wp_val \+ ctrl\.zero_position\.get\(pump_id, 0\)\},\n'
            r'\s+fast=False,\n'
            r'\s+\))'
        )
        match_pmove = pump_move.search(content)
        if match_pmove:
            content = (content[:match_pmove.end()] + 
                      "\n                        _prev_p[_pidx] = wp_val" + 
                      content[match_pmove.end():])
            report("OK", "Added pump skip-if-unchanged logic with tracking")
        else:
            report("MISS", "Could not find pump move_absolute to add tracking")
    else:
        report("MISS", "Could not find pump loop in TrajectoryExecutor (non-critical)")

    if not safe_write(path, content, "TrajectoryExecutor fixes"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH D: Fix _execute_command — set speed for MOVE_XY
# ═══════════════════════════════════════════════════════════════════

def patch_D_move_xy_speed(root: Path):
    """Set Prior stage speed before MOVE_XY travel commands."""
    print(f"\n{CYAN}[D] PrintManager.py — Set speed for MOVE_XY travel{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: Set travel speed"

    if marker in content:
        report("SKIP", "MOVE_XY speed set already applied")
        return content

    # Find the MOVE_XY handler. It should be in _execute_command dispatch.
    # Look for pattern: if cmd.type == CommandType.MOVE_XY:
    #    or: elif cmd.type == CommandType.MOVE_XY:
    move_xy_handler = re.compile(
        r'((?:el)?if\s+cmd\.type\s*==\s*CommandType\.MOVE_XY:\s*\n)'
        r'(\s+.*?)(?=\n\s+(?:el)?if\s+cmd\.type\s*==\s*CommandType\.)',
        re.DOTALL
    )
    match_mxy = move_xy_handler.search(content)

    if match_mxy:
        handler_start = match_mxy.start(2)
        # Find the first line of the handler body
        body_lines = match_mxy.group(2)
        
        # Insert speed set at the start of the handler body
        speed_inject = (
            "            # v7.2.7: Set travel speed before XY move\n"
            "            _travel_spd = getattr(self.job.settings, 'travel_speed_mm_s', 10.0) if self.job else 10.0\n"
            "            if self.controller.xy_stage:\n"
            "                try:\n"
            "                    self.controller.xy_stage.set_velocity(int(min(_travel_spd * 1000, 50000)))\n"
            "                except Exception:\n"
            "                    pass\n"
        )
        content = content[:handler_start] + speed_inject + content[handler_start:]
        report("OK", "Added travel speed set in MOVE_XY handler")
    else:
        report("MISS", "Could not find MOVE_XY command handler")

    if not safe_write(path, content, "MOVE_XY speed"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH E: print_setup.py — _get_settings sets print_speed_mm_s
# ═══════════════════════════════════════════════════════════════════

def patch_E_get_settings(root: Path):
    """Fix _get_settings to set print_speed_mm_s from GUI."""
    print(f"\n{CYAN}[E] print_setup.py — _get_settings print_speed_mm_s{RESET}")

    path = root / "gui" / "pages" / "print_setup.py"
    content = safe_read(path)
    if not content:
        report("MISS", "print_setup.py not found")
        return content

    marker = "v7.2.7: print_speed_mm_s"

    if marker in content:
        report("SKIP", "print_speed_mm_s already set in _get_settings")
        return content

    # Find _get_settings method and the line: s.xy_feedrate = self.xy_feed_spin.value()
    xy_feed_line = re.compile(
        r'^(\s+s\.xy_feedrate\s*=\s*self\.xy_feed_spin\.value\(\))(\s*(?:#.*)?)$',
        re.MULTILINE
    )
    match_xy = xy_feed_line.search(content)

    if match_xy:
        inject = (
            "\n"
            "        # v7.2.7: print_speed_mm_s — explicit mm/s for print execution\n"
            "        s.print_speed_mm_s = self.xy_feed_spin.value()  # mm/s from GUI\n"
            "        s.travel_speed_mm_s = self.xy_feed_spin.value() * 2.0  # travel 2x faster\n"
            "        # Also set legacy print_feedrate (mm/min) for backward compat\n"
            "        s.print_feedrate = self.xy_feed_spin.value() * 60.0  # mm/s → mm/min"
        )
        insert_pos = match_xy.end()
        content = content[:insert_pos] + inject + content[insert_pos:]
        report("OK", "Added print_speed_mm_s and legacy print_feedrate conversion")
    else:
        report("MISS", "Could not find s.xy_feedrate line in _get_settings")

    if not safe_write(path, content, "_get_settings"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH F: PrintSettings.from_dict — handle mm/s fields
# ═══════════════════════════════════════════════════════════════════

def patch_F_from_dict(root: Path):
    """Ensure from_dict handles new speed fields for loaded print files."""
    print(f"\n{CYAN}[F] PrintManager.py — from_dict backward compat{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: infer mm/s from legacy"

    if marker in content:
        report("SKIP", "from_dict already patched")
        return content

    # Find from_dict method and add post-processing
    match_fd = find_method(content, "from_dict")
    if not match_fd:
        report("MISS", "from_dict method not found")
        return content

    # Find the return statement in from_dict
    return_pattern = re.compile(
        r'(\s+return cls\(\*\*filtered\))',
        re.MULTILINE
    )
    # Search only within from_dict body
    fd_body = match_fd.group(0)
    match_ret = return_pattern.search(fd_body)

    if match_ret:
        # Replace the simple return with one that does post-processing
        old_return = match_ret.group(1)
        new_return = (
            "        instance = cls(**filtered)\n"
            "        # v7.2.7: infer mm/s from legacy print_feedrate if not set\n"
            "        if instance.print_speed_mm_s <= 0 and instance.print_feedrate > 0:\n"
            "            instance.print_speed_mm_s = instance.print_feedrate / 60.0\n"
            "        if instance.travel_speed_mm_s <= 0 and instance.xy_feedrate > 0:\n"
            "            # xy_feedrate could be mm/s (new) or stage-units/s (legacy)\n"
            "            if instance.xy_feedrate < 100:  # likely mm/s\n"
            "                instance.travel_speed_mm_s = instance.xy_feedrate\n"
            "            else:  # likely stage units, convert\n"
            "                instance.travel_speed_mm_s = instance.xy_feedrate / 1000.0\n"
            "        return instance"
        )
        # Replace within the full content
        abs_start = match_fd.start() + match_ret.start()
        abs_end = match_fd.start() + match_ret.end()
        content = content[:abs_start] + new_return + content[abs_end:]
        report("OK", "Added mm/s inference to from_dict")
    else:
        report("MISS", "Could not find return statement in from_dict")

    if not safe_write(path, content, "from_dict"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  PATCH G: Fix _wait_for_xy_settle tolerance and timeout
# ═══════════════════════════════════════════════════════════════════

def patch_G_settle(root: Path):
    """Improve _wait_for_xy_settle to use mm-based tolerance and shorter timeout."""
    print(f"\n{CYAN}[G] PrintManager.py — Fix _wait_for_xy_settle{RESET}")

    path = root / "SupportClasses" / "PrintManager.py"
    content = safe_read(path)
    if not content:
        report("MISS", "PrintManager.py not found")
        return content

    marker = "v7.2.7: mm-based settle"

    if marker in content:
        report("SKIP", "_wait_for_xy_settle already patched")
        return content

    # Find _wait_for_xy_settle method
    match_settle = find_method(content, "_wait_for_xy_settle")
    if not match_settle:
        report("MISS", "_wait_for_xy_settle not found")
        return content

    new_settle = '''    def _wait_for_xy_settle(self, target_x, target_y, timeout=3.0, tolerance=50):
        """
        Wait for XY stage to reach target position.

        v7.2.7: mm-based settle — shorter timeout, better logging.
        target_x/y are in mm (zero-ref). tolerance in µm.
        """
        ctrl = self.controller
        if not hasattr(ctrl, 'xy_stage') or not ctrl.xy_stage:
            time.sleep(0.1)
            return

        # Convert target mm → µm for comparison with stage position
        target_x_um = target_x * 1000.0 + ctrl.zero_position.get("x", 0)
        target_y_um = target_y * 1000.0 + ctrl.zero_position.get("y", 0)

        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._abort_flag.is_set():
                return
            pos = ctrl.get_xy_position(cached=False)
            if pos[0] is not None:
                dx = abs(pos[0] - target_x_um)
                dy = abs(pos[1] - target_y_um)
                if dx < tolerance and dy < tolerance:
                    return
            time.sleep(0.05)

        logger.debug(f"v7.2.7: Settle timeout after {timeout}s "
                     f"(target={target_x:.2f},{target_y:.2f}mm)")

'''
    content = content[:match_settle.start()] + new_settle + content[match_settle.end():]
    report("OK", "Replaced _wait_for_xy_settle with mm-based version")

    if not safe_write(path, content, "_wait_for_xy_settle"):
        return safe_read(path)
    return content


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count

    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7 Patch: XY Velocity Fix")
    print(f"{'='*60}{RESET}")

    root = find_root()
    print(f"Project root: {root}")

    # Apply patches in dependency order
    patch_A_print_settings(root)
    patch_B_execute_print_path(root)
    patch_C_trajectory_executor(root)
    patch_D_move_xy_speed(root)
    patch_E_get_settings(root)
    patch_F_from_dict(root)
    patch_G_settle(root)

    # Summary
    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"  Total changes attempted: {total}")
    print(f"{'='*60}{RESET}")

    if miss_count > 0:
        print(f"\n{RED}⚠ {miss_count} changes could not be applied!{RESET}")
        print(f"  Review MISS messages above.")
        print(f"  Some MISS items (Z-axis, pump) are non-critical optimizations.")
    else:
        print(f"\n{GREEN}✓ All changes applied successfully.{RESET}")

    # Final AST check on both files
    print(f"\n{CYAN}Final AST verification:{RESET}")
    for rel in ["SupportClasses/PrintManager.py", "gui/pages/print_setup.py"]:
        fpath = root / rel
        if fpath.exists():
            try:
                ast.parse(fpath.read_text(encoding="utf-8"))
                print(f"  {GREEN}✓ {rel}{RESET}")
            except SyntaxError as e:
                print(f"  {RED}✗ {rel}: {e}{RESET}")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
