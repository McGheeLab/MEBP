#!/usr/bin/env python3
"""
MEBP v7.2.5 -- Session 7 Patch: Startup Crash Fixes

Bug 1: Camera detection runs at startup (during CameraWidget.__init__),
       probing 8 indices via cv2.VideoCapture, generating repeated
       OpenCV errors and adding seconds to load time.
       Fix: Make detection fully lazy -- skip in _populate_cameras()
       during init, only detect when user clicks Detect or Start.

Bug 2: WellSetupTab.__init__ calls _refresh_plate() which calls
       _refresh_summary() before _build_summary_table() has run,
       causing AttributeError on 'summary_table'.
       Fix: Guard _refresh_summary() against missing summary_table.

Files Modified:
    gui/widgets/camera_widget.py     -- Lazy camera detection
    gui/pages/print_well_setup.py    -- Guard _refresh_summary

Usage:
    python patch_s7_startup_fixes.py [/path/to/MEBP]
"""

import os
import re
import sys
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
# FIX 1: camera_widget.py -- Fully lazy camera detection
# =================================================================

def fix_camera_detection(root: Path):
    filepath = root / "gui" / "widgets" / "camera_widget.py"
    print(f"\n{'=' * 60}")
    print(f"FIX 1: {filepath.name} -- Lazy camera detection")
    print(f"{'=' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # -- 1a: Replace detect_cameras with quieter, faster version --
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
        '    v7.2.5 S7: Reduced max_index 8->4, suppresses OpenCV errors,\n'
        '    stops after 2 consecutive failures for speed.\n'
        '    """\n'
        '    if not CV2_AVAILABLE:\n'
        '        return []\n'
        '\n'
        '    # Suppress OpenCV error spam during probing\n'
        '    old_log_level = None\n'
        '    try:\n'
        '        old_log_level = cv2.getLogLevel()\n'
        '        cv2.setLogLevel(0)  # SILENT\n'
        '    except (AttributeError, cv2.error):\n'
        '        pass\n'
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
        '                        break\n'
        '            except Exception:\n'
        '                consecutive_fails += 1\n'
        '                if consecutive_fails >= 2:\n'
        '                    break\n'
        '    finally:\n'
        '        if old_log_level is not None:\n'
        '            try:\n'
        '                cv2.setLogLevel(old_log_level)\n'
        '            except (AttributeError, cv2.error):\n'
        '                pass\n'
        '\n'
        '    logger.info(f"Camera detection: found {len(available)} camera(s) "\n'
        '               f"at indices {available}")\n'
        '    return available'
    )
    content = patch_replace(content, old_detect, new_detect,
                            "1a: Replace detect_cameras (quieter, faster, capped at 4)")

    # If exact match missed, try regex
    if 'consecutive_fails' not in content and '_cameras_detected' not in content:
        pattern = (
            r'def detect_cameras\(max_index:\s*int\s*=\s*\d+\)\s*->\s*list\[int\]:\s*\n'
            r'\s*""".*?"""\s*\n'
            r'(?:\s+.*\n)*?'
            r'\s+return available'
        )
        content = patch_replace_regex(
            content, pattern, new_detect,
            "1a-alt: Replace detect_cameras (regex)"
        )

    # -- 1b: Make _populate_cameras lazy -- don't probe on init --
    # Replace _populate_cameras to just show a placeholder.
    # Actual detection deferred to refresh_cameras() or start().
    old_populate = (
        '    def _populate_cameras(self):\n'
        '        """Detect available cameras and populate the source combo."""\n'
        '        self.camera_combo.clear()\n'
        '        if not CV2_AVAILABLE:\n'
        '            return\n'
        '        for idx in detect_cameras():\n'
        '            self.camera_combo.addItem(f"Camera {idx}", idx)\n'
        '        if self.camera_combo.count() == 0:\n'
        '            self.camera_combo.addItem("No cameras found", -1)'
    )
    new_populate = (
        '    def _populate_cameras(self):\n'
        '        """Show placeholder in camera combo -- no hardware probe.\n'
        '\n'
        '        v7.2.5 S7: Camera detection is now lazy. This method just\n'
        '        sets a placeholder. Call refresh_cameras() or start() to\n'
        '        actually probe hardware.\n'
        '        """\n'
        '        self.camera_combo.clear()\n'
        '        if not CV2_AVAILABLE:\n'
        '            return\n'
        '        self._cameras_detected = False\n'
        '        self.camera_combo.addItem("Click Detect or Start", -1)'
    )
    content = patch_replace(content, old_populate, new_populate,
                            "1b: Make _populate_cameras lazy (no probe on init)")

    # -- 1c: Make refresh_cameras actually probe --
    old_refresh = (
        '    def refresh_cameras(self):\n'
        '        """Re-scan for cameras (can be called externally)."""\n'
        '        self._populate_cameras()'
    )
    new_refresh = (
        '    def refresh_cameras(self):\n'
        '        """Detect cameras and populate the combo.\n'
        '\n'
        '        v7.2.5 S7: This is now the only path that probes hardware.\n'
        '        Called by user clicking Detect, or automatically on first start().\n'
        '        """\n'
        '        self.camera_combo.clear()\n'
        '        if not CV2_AVAILABLE:\n'
        '            return\n'
        '        for idx in detect_cameras():\n'
        '            self.camera_combo.addItem(f"Camera {idx}", idx)\n'
        '        if self.camera_combo.count() == 0:\n'
        '            self.camera_combo.addItem("No cameras found", -1)\n'
        '        self._cameras_detected = True'
    )
    content = patch_replace(content, old_refresh, new_refresh,
                            "1c: Make refresh_cameras do the actual detection")

    # -- 1d: Make start() trigger lazy detection if not yet done --
    old_start = (
        '    def start(self):\n'
        '        """Start the camera feed using the currently selected combo index."""\n'
        '        if not CV2_AVAILABLE or self._running:\n'
        '            return\n'
        '\n'
        '        idx = self.camera_combo.currentData()\n'
        '        if idx is None or idx < 0:\n'
        '            self.video_label.setText("No camera available")\n'
        '            return\n'
        '        self.start_with_index(idx)'
    )
    new_start = (
        '    def start(self):\n'
        '        """Start the camera feed using the currently selected combo index.\n'
        '\n'
        '        v7.2.5 S7: Triggers lazy camera detection on first start.\n'
        '        """\n'
        '        if not CV2_AVAILABLE or self._running:\n'
        '            return\n'
        '\n'
        '        # Lazy detection: probe hardware on first start\n'
        '        if not getattr(self, "_cameras_detected", False):\n'
        '            self.refresh_cameras()\n'
        '\n'
        '        idx = self.camera_combo.currentData()\n'
        '        if idx is None or idx < 0:\n'
        '            self.video_label.setText("No camera available")\n'
        '            return\n'
        '        self.start_with_index(idx)'
    )
    content = patch_replace(content, old_start, new_start,
                            "1d: Trigger lazy detection on first start()")

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

    # -- 2a: Guard _refresh_summary (the pre-v7.2.5 version using summary_table) --
    old_refresh = (
        '    def _refresh_summary(self) -> None:\n'
        '        """Rebuild the assignment summary table."""\n'
        '        data = self._model.get_assignment_summary()\n'
        '        self.summary_table.setRowCount(len(data))'
    )
    new_refresh = (
        '    def _refresh_summary(self) -> None:\n'
        '        """Rebuild the assignment summary table."""\n'
        '        # v7.2.5 S7: Guard -- summary_table may not exist yet during __init__\n'
        '        if not hasattr(self, "summary_table") or self.summary_table is None:\n'
        '            return\n'
        '        data = self._model.get_assignment_summary()\n'
        '        self.summary_table.setRowCount(len(data))'
    )
    content = patch_replace(content, old_refresh, new_refresh,
                            "2a: Guard _refresh_summary against missing summary_table")

    # -- 2b: Guard _refresh_summary_table (v7.2.5 S4 variant with _summary_table) --
    old_refresh_s4 = '    def _refresh_summary_table(self):\n        """v7.2.5: Refresh the assignment summary table."""'
    if old_refresh_s4 in content:
        method_start = content.find(old_refresh_s4)
        next_200 = content[method_start:method_start + 200]
        if 'hasattr' not in next_200:
            new_refresh_s4 = (
                '    def _refresh_summary_table(self):\n'
                '        """v7.2.5: Refresh the assignment summary table."""\n'
                '        # v7.2.5 S7: Guard -- _summary_table may not exist yet during __init__\n'
                '        if not hasattr(self, "_summary_table") or self._summary_table is None:\n'
                '            return'
            )
            content = patch_replace(content, old_refresh_s4, new_refresh_s4,
                                    "2b: Guard _refresh_summary_table")
    else:
        print(f"  {YELLOW}SKIP{RESET}: 2b -- _refresh_summary_table not found (OK if S4 not applied)")

    # -- 2c: Guard _refresh_ink_combo --
    # This can also be called early before ink_combo exists
    old_ink = (
        '    def _refresh_ink_combo(self) -> None:\n'
        '        """Populate ink combo from workspace ink library or HardwareConfig.'
    )
    if old_ink in content:
        method_pos = content.find(old_ink)
        next_chunk = content[method_pos:method_pos + 400]
        # Check if a guard already exists
        if 'hasattr(self, "ink_combo")' not in next_chunk[:250] and 'hasattr(self, \'ink_combo\')' not in next_chunk[:250]:
            # Find the first real code line after the docstring
            # Look for 'source = {}' which is the first executable line
            source_line_pos = content.find('        source = {}', method_pos)
            if source_line_pos > 0 and source_line_pos < method_pos + 600:
                guard = '        # v7.2.5 S7: Guard -- ink_combo may not exist during __init__\n        if not hasattr(self, "ink_combo"):\n            return\n'
                content = content[:source_line_pos] + guard + content[source_line_pos:]
                global _applied
                _applied += 1
                print(f"  {GREEN}OK{RESET}:   2c: Guard _refresh_ink_combo")
            else:
                print(f"  {YELLOW}SKIP{RESET}: 2c -- could not find insertion point in _refresh_ink_combo")
        else:
            print(f"  {YELLOW}SKIP{RESET}: 2c -- _refresh_ink_combo already guarded")
    else:
        print(f"  {YELLOW}SKIP{RESET}: 2c -- _refresh_ink_combo not found")

    # -- 2d: Guard _refresh_rosette_combo similarly --
    ros_method = '    def _refresh_rosette_combo(self) -> None:'
    if ros_method in content:
        method_pos = content.find(ros_method)
        next_chunk = content[method_pos:method_pos + 400]
        if 'hasattr(self, "rosette_combo")' not in next_chunk[:250] and 'hasattr(self, \'rosette_combo\')' not in next_chunk[:250]:
            # Find first real code line -- look for 'source = {}' or 'current_ros'
            for marker in ['        source = {}', '        current_ros', '        self.rosette_combo']:
                marker_pos = content.find(marker, method_pos + len(ros_method))
                if marker_pos > 0 and marker_pos < method_pos + 600:
                    guard = '        # v7.2.5 S7: Guard -- rosette_combo may not exist during __init__\n        if not hasattr(self, "rosette_combo"):\n            return\n'
                    content = content[:marker_pos] + guard + content[marker_pos:]
                    _applied += 1
                    print(f"  {GREEN}OK{RESET}:   2d: Guard _refresh_rosette_combo")
                    break
            else:
                print(f"  {YELLOW}SKIP{RESET}: 2d -- could not find insertion point")
        else:
            print(f"  {YELLOW}SKIP{RESET}: 2d -- _refresh_rosette_combo already guarded")
    else:
        print(f"  {YELLOW}SKIP{RESET}: 2d -- _refresh_rosette_combo not found")

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
    print(f" Bug 1: Lazy camera detection (no probe on startup)")
    print(f" Bug 2: Guard _refresh_summary in WellSetupTab")
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
        print(f"  gui/widgets/camera_widget.py      -- Lazy camera detection")
        print(f"  gui/pages/print_well_setup.py     -- Guard _refresh_summary")


if __name__ == "__main__":
    main()
