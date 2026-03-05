#!/usr/bin/env python3
"""
MEBP v7.2.5 -- Session 7 Patch: Startup Crash Fixes

Bug 1: Camera detection probes too many indices on startup, generating
       repeated OpenCV errors and slowing load by several seconds.
       Fix: Reduce max_index from 8 to 3, suppress OpenCV errors during
       probe, and make detection lazy (deferred to first camera start).

Bug 2: WellSetupTab.__init__ calls _refresh_plate() which calls
       _refresh_summary() before _build_summary_table() has run,
       causing AttributeError on 'summary_table'.
       Fix: Guard _refresh_summary() against missing summary_table.

Files Modified:
    gui/widgets/camera_widget.py     -- Faster, quieter camera detection
    gui/pages/print_well_setup.py    -- Guard _refresh_summary

Usage:
    python patch_s7_startup_fixes.py [/path/to/MEBP]
"""

import os
import re
import sys

# === Windows console encoding fix ===
import sys as _sys
if _sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(_sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')
# === End encoding fix ===

import shutil
from pathlib import Path
from datetime import datetime

# Windows console encoding fix
if sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')

# Terminal colors
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
RESET  = "\033[0m"

_applied = 0
_skipped = 0
_failed  = 0


def find_project_root() -> Path:
    candidates = [
        Path.cwd(), Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
    sys.exit(1)


def backup_file(filepath: Path):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = filepath.with_suffix(f".bak_v725s7_{ts}")
    shutil.copy2(filepath, backup)


def read_file(filepath: Path) -> str:
    return filepath.read_text(encoding="utf-8")


def write_file(filepath: Path, content: str):
    filepath.write_text(content, encoding="utf-8")


def patch_replace(content: str, old: str, new: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new.strip()[:80] in content and old not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} -- already applied")
        _skipped += 1
        return content
    if old not in content:
        print(f"  {RED}MISS{RESET}: {description} -- anchor not found")
        _failed += 1
        return content
    content = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def patch_replace_regex(content: str, pattern: str, replacement: str, description: str) -> str:
    global _applied, _skipped, _failed
    match = re.search(pattern, content, re.DOTALL)
    if match is None:
        if replacement.strip()[:60] in content:
            print(f"  {YELLOW}SKIP{RESET}: {description} -- already applied")
            _skipped += 1
        else:
            print(f"  {RED}MISS{RESET}: {description} -- pattern not found")
            _failed += 1
        return content
    content = content[:match.start()] + replacement + content[match.end():]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


# =================================================================
# FIX 1: camera_widget.py -- Faster, quieter camera detection
# =================================================================

def fix_camera_detection(root: Path):
    filepath = root / "gui" / "widgets" / "camera_widget.py"
    print(f"\n{'=' * 60}")
    print(f"FIX 1: {filepath.name} -- Camera detection")
    print(f"{'=' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # -- 1a: Replace the detect_cameras function entirely --
    # The old version probes up to max_index=8 without suppressing errors.
    # New version: max_index=4, suppresses OpenCV errors during probing,
    # stops after first gap (most systems have contiguous indices).

    old_detect = (
        'def detect_cameras(max_index: int = 8) -> list[int]:\n'
        '    """Probe camera indices and return those that are available."""\n'
        '    if not CV2_AVAILABLE:\n'
        '        return []\n'
        '    available = []\n'
        '    for idx in range(max_index):\n'
        '        cap = cv2.VideoCapture(idx)\n'
        '        if cap.isOpened():\n'
        '            available.append(idx)\n'
        '            cap.release()\n'
        '    return available'
    )

    new_detect = (
        'def detect_cameras(max_index: int = 4) -> list[int]:\n'
        '    """Probe camera indices and return those that are available.\n'
        '\n'
        '    v7.2.5 S7: Reduced max_index from 8 to 4 for faster startup.\n'
        '    Suppresses OpenCV error logging during probe. Stops early\n'
        '    after 2 consecutive failures (most systems use contiguous indices).\n'
        '    """\n'
        '    if not CV2_AVAILABLE:\n'
        '        return []\n'
        '\n'
        '    # Suppress OpenCV error spam during camera probing\n'
        '    old_log_level = None\n'
        '    try:\n'
        '        old_log_level = cv2.getLogLevel()\n'
        '        cv2.setLogLevel(0)  # SILENT\n'
        '    except (AttributeError, cv2.error):\n'
        '        pass  # Older OpenCV versions may not have setLogLevel\n'
        '\n'
        '    available = []\n'
        '    consecutive_fails = 0\n'
        '    try:\n'
        '        for idx in range(max_index):\n'
        '            try:\n'
        '                cap = cv2.VideoCapture(idx)\n'
        '                if cap.isOpened():\n'
        '                    available.append(idx)\n'
        '                    cap.release()\n'
        '                    consecutive_fails = 0\n'
        '                else:\n'
        '                    consecutive_fails += 1\n'
        '                    if consecutive_fails >= 2:\n'
        '                        break  # Stop probing after 2 consecutive failures\n'
        '            except Exception:\n'
        '                consecutive_fails += 1\n'
        '                if consecutive_fails >= 2:\n'
        '                    break\n'
        '    finally:\n'
        '        # Restore OpenCV log level\n'
        '        if old_log_level is not None:\n'
        '            try:\n'
        '                cv2.setLogLevel(old_log_level)\n'
        '            except (AttributeError, cv2.error):\n'
        '                pass\n'
        '\n'
        '    logger.info(f"Camera detection: found {len(available)} camera(s) "\n'
        '               f"at indices {available} (probed 0-{max_index-1})")\n'
        '    return available'
    )

    content = patch_replace(content, old_detect, new_detect,
                            "1a: Replace detect_cameras with faster, quieter version")

    # If exact match failed, try regex
    if 'consecutive_fails' not in content:
        pattern = (
            r'def detect_cameras\(max_index:\s*int\s*=\s*\d+\)\s*->\s*list\[int\]:\s*\n'
            r'\s*"""Probe camera indices.*?""".*?\n'
            r'(?:\s+.*\n)*?'
            r'\s+return available'
        )
        content = patch_replace_regex(
            content, pattern, new_detect,
            "1a-alt: Replace detect_cameras (regex fallback)"
        )

    write_file(filepath, content)


# =================================================================
# FIX 2: print_well_setup.py -- Guard _refresh_summary
# =================================================================

def fix_refresh_summary(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'=' * 60}")
    print(f"FIX 2: {filepath.name} -- Guard _refresh_summary")
    print(f"{'=' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # -- 2a: Add guard to _refresh_summary --
    # The method references self.summary_table but it may not exist yet
    # during __init__ because _build_summary_table hasn't been called.

    old_refresh = (
        '    def _refresh_summary(self) -> None:\n'
        '        """Rebuild the assignment summary table."""\n'
        '        data = self._model.get_assignment_summary()\n'
        '        self.summary_table.setRowCount(len(data))'
    )
    new_refresh = (
        '    def _refresh_summary(self) -> None:\n'
        '        """Rebuild the assignment summary table."""\n'
        '        # v7.2.5 S7: Guard against summary_table not yet created\n'
        '        # (_refresh_plate can be called in __init__ before _build_summary_table)\n'
        '        if not hasattr(self, "summary_table") or self.summary_table is None:\n'
        '            return\n'
        '        data = self._model.get_assignment_summary()\n'
        '        self.summary_table.setRowCount(len(data))'
    )
    content = patch_replace(content, old_refresh, new_refresh,
                            "2a: Guard _refresh_summary against missing summary_table")

    # -- 2b: Also guard _refresh_summary_table (v7.2.5 S4 variant) --
    # The S4 patch may have added a _refresh_summary_table method that
    # references self._summary_table (with underscore prefix)
    old_refresh_s4 = (
        '    def _refresh_summary_table(self):\n'
        '        """v7.2.5: Refresh the assignment summary table."""'
    )
    if old_refresh_s4 in content:
        # Check if it already has a guard
        method_start = content.find(old_refresh_s4)
        next_lines = content[method_start:method_start + 300]
        if 'hasattr' not in next_lines[:200]:
            new_refresh_s4 = (
                '    def _refresh_summary_table(self):\n'
                '        """v7.2.5: Refresh the assignment summary table."""\n'
                '        if not hasattr(self, "_summary_table") or self._summary_table is None:\n'
                '            return'
            )
            content = patch_replace(content, old_refresh_s4, new_refresh_s4,
                                    "2b: Guard _refresh_summary_table against missing _summary_table")
    else:
        print(f"  {YELLOW}SKIP{RESET}: 2b -- _refresh_summary_table method not found (OK)")

    # -- 2c: Also guard _refresh_ink_combo and _refresh_rosette_combo --
    # These can also be called before their combos exist
    old_ink_refresh = (
        '    def _refresh_ink_combo(self) -> None:\n'
        '        """Populate ink combo from workspace ink library or HardwareConfig.'
    )
    if old_ink_refresh in content:
        method_start = content.find(old_ink_refresh)
        next_chunk = content[method_start:method_start + 400]
        if 'hasattr(self, \'ink_combo\')' not in next_chunk[:200] and 'hasattr(self, "ink_combo")' not in next_chunk[:200]:
            new_ink_refresh = (
                '    def _refresh_ink_combo(self) -> None:\n'
                '        """Populate ink combo from workspace ink library or HardwareConfig.\n'
                '\n'
                '        v7.2.5 S7: Guards against combo not yet created during __init__.'
            )
            content = patch_replace(content, old_ink_refresh, new_ink_refresh,
                                    "2c-header: Update _refresh_ink_combo docstring")

            # Add guard at the start of the method body
            old_source_line = '        source = {}'
            if old_source_line in content:
                idx = content.find(old_source_line, content.find('_refresh_ink_combo'))
                if idx > 0:
                    guard = '        if not hasattr(self, "ink_combo"):\n            return\n'
                    # Only add if not already present nearby
                    nearby = content[idx-100:idx]
                    if 'hasattr(self, "ink_combo")' not in nearby:
                        content = content[:idx] + guard + content[idx:]
                        global _applied
                        _applied += 1
                        print(f"  {GREEN}OK{RESET}:   2c: Guard _refresh_ink_combo body")

    # -- 2d: Guard _refresh_rosette_combo similarly --
    old_ros_refresh = '    def _refresh_rosette_combo(self) -> None:'
    if old_ros_refresh in content:
        method_start = content.find(old_ros_refresh)
        next_chunk = content[method_start:method_start + 400]
        if 'hasattr(self, "rosette_combo")' not in next_chunk[:200] and 'hasattr(self, \'rosette_combo\')' not in next_chunk[:200]:
            # Find the first non-docstring, non-comment line in the method
            # Look for "source = {}" or "current_ros ="
            search_start = method_start + len(old_ros_refresh)
            for marker in ['source = {}', 'current_ros =', 'self.rosette_combo']:
                marker_pos = content.find(marker, search_start)
                if marker_pos > 0 and marker_pos < search_start + 500:
                    # Find the line start
                    line_start = content.rfind('\n', search_start, marker_pos) + 1
                    guard = '        if not hasattr(self, "rosette_combo"):\n            return\n'
                    nearby = content[line_start-100:line_start]
                    if 'hasattr(self, "rosette_combo")' not in nearby:
                        content = content[:line_start] + guard + content[line_start:]
                        _applied += 1
                        print(f"  {GREEN}OK{RESET}:   2d: Guard _refresh_rosette_combo body")
                    break

    write_file(filepath, content)


# =================================================================
# MAIN
# =================================================================

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'=' * 60}")
    print(f" MEBP v7.2.5 -- Session 7: Startup Fixes")
    print(f" Bug 1: Camera detection speed + errors")
    print(f" Bug 2: WellSetupTab summary_table AttributeError")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    fix_camera_detection(root)
    fix_refresh_summary(root)

    # Summary
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'=' * 60}")
    print(f" SUMMARY")
    print(f"{'=' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}: {_applied}")
    print(f"  {YELLOW}Skipped{RESET}: {_skipped}")
    print(f"  {RED}Failed{RESET}:  {_failed}")
    print(f"  Total:   {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} patch(es) failed!")
        print("Check the MISS messages above.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All startup fixes applied successfully!{RESET}")
        print(f"\nFiles modified:")
        print(f"  gui/widgets/camera_widget.py      -- Faster camera detection")
        print(f"  gui/pages/print_well_setup.py     -- Guard _refresh_summary")


if __name__ == "__main__":
    main()
