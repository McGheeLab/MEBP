#!/usr/bin/env python3
"""
patch_camera_fix.py — Fix ToupCam integration issues found in testing.

Fixes:
  1. toupcam_backend.py:
     - Add missing argtypes for Toupcam_StartPullModeWithCallback
     - Use WINFUNCTYPE (stdcall) on Windows instead of CFUNCTYPE (cdecl)
     - Wrap handle in c_void_p() when calling SDK functions (Python 3.12+ safety)

  2. camera_widget.py:
     - Force-apply refresh_cameras() dual-backend (was false-SKIPped)
     - Force-apply _grab_frame() dual-backend (was false-SKIPped)

Run from MEBP root:
    python patches/v726_camera/patch_camera_fix.py
"""

from __future__ import annotations
import os
import re
import sys
import ast
import shutil
from pathlib import Path

GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'

ok_count = 0
skip_count = 0
miss_count = 0


def find_project_root() -> Path:
    for candidate in [Path.cwd(), Path(__file__).resolve().parent.parent.parent]:
        if (candidate / 'main.py').exists() and (candidate / 'gui').exists():
            return candidate
    print(f"{RED}FAIL: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def report(status, label, detail=""):
    global ok_count, skip_count, miss_count
    color = {'+': GREEN, '~': YELLOW, '!': RED}[status]
    tag = {'+': 'OK', '~': 'SKIP', '!': 'MISS'}[status]
    print(f"  {color}{tag:4s}{RESET} {label}")
    if detail:
        print(f"        {detail}")
    if status == '+': ok_count += 1
    elif status == '~': skip_count += 1
    else: miss_count += 1


def backup_and_write(filepath: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        report('!', f"AST FAIL for {label}", str(e))
        return False
    bak = filepath.with_suffix(filepath.suffix + '.bak_camfix')
    if not bak.exists():
        shutil.copy2(filepath, bak)
    filepath.write_text(content, encoding='utf-8')
    report('+', f"Written {filepath.name}")
    return True


# ════════════════════════════════════════════════════════════════════
#  FIX 1: toupcam_backend.py — callback type + argtypes + handle wrapping
# ════════════════════════════════════════════════════════════════════

def fix_toupcam_backend(root: Path):
    print(f"\n{CYAN}{BOLD}── Fix 1: toupcam_backend.py ──{RESET}")

    fpath = root / 'gui' / 'widgets' / 'toupcam_backend.py'
    if not fpath.exists():
        report('!', "toupcam_backend.py not found")
        return

    content = fpath.read_text(encoding='utf-8')

    # ── 1A: Fix callback type — use WINFUNCTYPE on Windows ──
    old_callback = '_EVENT_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)'
    new_callback = '''# v7.3-camera fix: Use WINFUNCTYPE (stdcall) on Windows, CFUNCTYPE (cdecl) elsewhere
import platform as _platform
if _platform.system() == 'Windows':
    _EVENT_CALLBACK = ctypes.WINFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)
else:
    _EVENT_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)'''

    if 'WINFUNCTYPE' in content:
        report('~', "1A: WINFUNCTYPE callback (already applied)")
    elif old_callback in content:
        content = content.replace(old_callback, new_callback, 1)
        report('+', "1A: WINFUNCTYPE callback on Windows")
    else:
        # Try regex for slight variations
        pat = r'_EVENT_CALLBACK\s*=\s*ctypes\.CFUNCTYPE\(None,\s*ctypes\.c_uint,\s*ctypes\.c_void_p\)'
        if re.search(pat, content):
            content = re.sub(pat, new_callback, content, count=1)
            report('+', "1A: WINFUNCTYPE callback on Windows (regex)")
        else:
            report('!', "1A: Cannot find _EVENT_CALLBACK definition")

    # ── 1B: Add StartPullModeWithCallback argtypes in _load_library ──
    if 'Toupcam_StartPullModeWithCallback.argtypes' in content:
        report('~', "1B: StartPullModeWithCallback argtypes (already present)")
    else:
        # Find where we set Toupcam_Snap argtypes (last signature in _load_library)
        anchor = "lib.Toupcam_Snap.restype = ctypes.c_int"
        if anchor in content:
            insert_text = '''

        # StartPullModeWithCallback(handle, callback, ctx) -> HRESULT
        lib.Toupcam_StartPullModeWithCallback.argtypes = [
            ctypes.c_void_p, _EVENT_CALLBACK, ctypes.c_void_p,
        ]
        lib.Toupcam_StartPullModeWithCallback.restype = ctypes.c_int'''

            content = content.replace(anchor, anchor + insert_text, 1)
            report('+', "1B: Added StartPullModeWithCallback argtypes")
        else:
            report('!', "1B: Cannot find Snap argtypes anchor")

    # ── 1C: Also add Stop argtypes with restype if missing ──
    if 'Toupcam_Stop.argtypes' not in content:
        anchor2 = "lib.Toupcam_Close.restype = None"
        if anchor2 in content:
            insert_stop = '''

        # Stop(handle) -> HRESULT
        lib.Toupcam_Stop.argtypes = [ctypes.c_void_p]
        lib.Toupcam_Stop.restype = ctypes.c_int'''
            content = content.replace(anchor2, anchor2 + insert_stop, 1)
            report('+', "1C: Added Stop argtypes")
        else:
            report('~', "1C: Stop argtypes (already present or anchor missing)")
    else:
        report('~', "1C: Stop argtypes (already present)")

    # ── 1D: Wrap handle in c_void_p() in open() method for Python 3.12+ safety ──
    # Replace: self._handle = lib.Toupcam_Open(device_id)
    # With:    raw = lib.Toupcam_Open(device_id); self._handle = ctypes.c_void_p(raw)
    old_open = 'self._handle = lib.Toupcam_Open(device_id)'
    new_open = '''# v7.3-camera fix: Wrap handle as c_void_p for Python 3.12+ ctypes safety
        _raw_handle = lib.Toupcam_Open(device_id)
        self._handle = ctypes.c_void_p(_raw_handle)'''

    if 'c_void_p(_raw_handle)' in content:
        report('~', "1D: Handle wrapping (already applied)")
    elif old_open in content:
        content = content.replace(old_open, new_open, 1)
        report('+', "1D: Wrap handle in c_void_p()")
    else:
        report('!', "1D: Cannot find handle assignment")

    # ── 1E: Fix isOpened to check c_void_p handle correctly ──
    old_is_opened = 'return self._handle is not None and self._running'
    new_is_opened = '''# v7.3-camera fix: c_void_p(0) is falsy, c_void_p(None) is falsy
        handle_ok = self._handle is not None and bool(self._handle)
        return handle_ok and self._running'''

    if 'handle_ok' in content:
        report('~', "1E: isOpened c_void_p check (already applied)")
    elif old_is_opened in content:
        content = content.replace(old_is_opened, new_is_opened, 1)
        report('+', "1E: isOpened c_void_p check")
    else:
        report('!', "1E: Cannot find isOpened return")

    # ── 1F: Fix the open() null-handle check for c_void_p ──
    old_null_check = '''if not self._handle:
            logger.warning(f"ToupCam: failed to open {device_id[:30]}")'''
    new_null_check = '''if self._handle is None or not self._handle:
            logger.warning(f"ToupCam: failed to open {device_id[:30]}")'''

    if 'self._handle is None or not self._handle' in content:
        report('~', "1F: Null handle check (already applied)")
    elif old_null_check in content:
        content = content.replace(old_null_check, new_null_check, 1)
        report('+', "1F: Null handle check for c_void_p")
    else:
        report('~', "1F: Null handle check (pattern not found, may be OK)")

    # ── 1G: Fix _on_event to check handle is valid c_void_p ──
    old_event_check = 'if nEvent == TOUPCAM_EVENT_IMAGE and self._handle and self._lib:'
    new_event_check = 'if nEvent == TOUPCAM_EVENT_IMAGE and self._handle is not None and bool(self._handle) and self._lib:'

    if 'self._handle is not None and bool(self._handle)' in content:
        report('~', "1G: _on_event handle check (already applied)")
    elif old_event_check in content:
        content = content.replace(old_event_check, new_event_check, 1)
        report('+', "1G: _on_event c_void_p handle check")
    else:
        report('~', "1G: _on_event handle check (pattern variant)")

    # ── 1H: Fix release() to handle c_void_p ──
    old_release_check = 'if self._handle and self._lib:'
    new_release_check = 'if self._handle is not None and bool(self._handle) and self._lib:'

    if new_release_check in content:
        report('~', "1H: release() handle check (already applied)")
    elif old_release_check in content:
        content = content.replace(old_release_check, new_release_check, 1)
        report('+', "1H: release() c_void_p handle check")
    else:
        report('~', "1H: release() handle check (pattern variant)")

    # ── Write ──
    backup_and_write(fpath, content, "toupcam_backend.py")


# ════════════════════════════════════════════════════════════════════
#  FIX 2: camera_widget.py — force-apply refresh_cameras + _grab_frame
# ════════════════════════════════════════════════════════════════════

def fix_camera_widget(root: Path):
    print(f"\n{CYAN}{BOLD}── Fix 2: camera_widget.py ──{RESET}")

    fpath = root / 'gui' / 'widgets' / 'camera_widget.py'
    if not fpath.exists():
        report('!', "camera_widget.py not found")
        return

    content = fpath.read_text(encoding='utf-8')

    # ── 2A: Force-replace refresh_cameras method ──
    # Use regex to capture the entire method body (until next def or class at same indent)
    refresh_new = '''    def refresh_cameras(self):
        """Detect cameras and populate the combo.

        v7.3-camera: Detects both OpenCV and ToupCam cameras.
        Combo item data is a tuple: ("opencv", index) or ("toupcam", device_id).
        """
        self.camera_combo.clear()
        if not CAMERA_AVAILABLE:
            return

        # OpenCV cameras
        if CV2_AVAILABLE:
            for idx in detect_cameras():
                self.camera_combo.addItem(f"CV2: Camera {idx}", ("opencv", idx))

        # ToupCam cameras  (v7.3-camera)
        for tc_dev in detect_toupcam_cameras():
            name = tc_dev.get('displayname', 'ToupCam')
            dev_id = tc_dev.get('id', '')
            self.camera_combo.addItem(f"TC: {name}", ("toupcam", dev_id))

        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)
        self._cameras_detected = True
'''

    if 'ToupCam cameras  (v7.3-camera)' in content:
        report('~', "2A: refresh_cameras (already has v7.3-camera marker)")
    else:
        # Match the entire refresh_cameras method
        pat = (
            r'(    def refresh_cameras\(self\):.*?)'
            r'(?=\n    def |\n    # ══|\nclass |\Z)'
        )
        m = re.search(pat, content, re.DOTALL)
        if m:
            content = content[:m.start()] + refresh_new + content[m.end():]
            report('+', "2A: refresh_cameras replaced with dual-backend")
        else:
            report('!', "2A: Cannot find refresh_cameras method boundary")

    # ── 2B: Force-replace _grab_frame method ──
    grab_new = '''    def _grab_frame(self):
        """Capture, adjust, and display one frame.

        v7.3-camera: Reads from OpenCV or ToupCam backend.
        """
        # v7.3-camera: Dual backend read
        backend = getattr(self, '_backend_type', 'opencv')
        if backend == 'toupcam':
            tc = getattr(self, '_toupcam', None)
            if tc is None or not tc.isOpened():
                self.stop()
                return
            ret, frame = tc.read()
        else:
            if not self._capture or not self._capture.isOpened():
                self.stop()
                return
            ret, frame = self._capture.read()

        if not ret or frame is None:
            return

        # Import numpy/cv2 for processing
        try:
            import cv2
            import numpy as np
        except ImportError:
            return

        # Convert BGR -> RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Apply software brightness
        if self._brightness != 0:
            rgb = cv2.convertScaleAbs(rgb, alpha=1.0, beta=self._brightness)

        # Apply software gamma via LUT
        if self._gamma_lut is not None and abs(self._gamma - 1.0) > 0.01:
            rgb = cv2.LUT(rgb, self._gamma_lut)

        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        # Make a copy so the numpy buffer stays valid for QImage
        q_img = QImage(rgb.copy().data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        # Draw crosshair overlay
        pixmap = QPixmap.fromImage(q_img)
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # Scale to fit label
        scaled = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)

        self.frame_captured.emit(q_img)

'''

    if "v7.3-camera: Dual backend read" in content:
        report('~', "2B: _grab_frame (already has v7.3-camera marker)")
    else:
        pat = (
            r'(    def _grab_frame\(self\):.*?)'
            r'(?=\n    def |\n    # ══|\nclass |\Z)'
        )
        m = re.search(pat, content, re.DOTALL)
        if m:
            content = content[:m.start()] + grab_new + content[m.end():]
            report('+', "2B: _grab_frame replaced with dual-backend")
        else:
            report('!', "2B: Cannot find _grab_frame method boundary")

    # ── 2C: Ensure start() routing is correct ──
    # Check that the start() method routes toupcam correctly
    if 'self._start_toupcam(identifier)' in content:
        report('~', "2C: start() routing (already correct)")
    else:
        # Find the start method's body and check for the combo data routing
        if 'cam_data = self.camera_combo.currentData()' in content:
            report('~', "2C: start() routing (cam_data pattern exists)")
        else:
            # Need to patch start() — find where it reads currentData
            old_start_block = (
                '        idx = self.camera_combo.currentData()\n'
                '        if idx is None or idx < 0:\n'
                '            self.video_label.setText("No camera available")\n'
                '            return\n'
                '        self.start_with_index(idx)'
            )
            new_start_block = '''        cam_data = self.camera_combo.currentData()
        if cam_data is None or cam_data == -1:
            self.video_label.setText("No camera available")
            return

        # v7.3-camera: Route by backend type
        if isinstance(cam_data, tuple) and len(cam_data) == 2:
            backend_type, identifier = cam_data
            if backend_type == "toupcam":
                self._start_toupcam(identifier)
                return
            else:
                self.start_with_index(identifier)
                return

        # Legacy: plain integer index
        if isinstance(cam_data, int) and cam_data >= 0:
            self.start_with_index(cam_data)'''

            if old_start_block in content:
                content = content.replace(old_start_block, new_start_block, 1)
                report('+', "2C: start() routing patched")
            else:
                report('!', "2C: Cannot find start() routing block",
                       "Check manually that start() routes ('toupcam', id) tuples")

    # ── Write ──
    backup_and_write(fpath, content, "camera_widget.py")


# ════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count

    print(f"\n{BOLD}{'=' * 60}")
    print(f"  ToupCam Integration Fix Patch")
    print(f"  Fixes: ctypes handle overflow + false-SKIPped methods")
    print(f"{'=' * 60}{RESET}\n")

    root = find_project_root()
    print(f"  Project root: {root}")

    fix_toupcam_backend(root)
    fix_camera_widget(root)

    print(f"\n{BOLD}{'=' * 60}")
    print(f"  SUMMARY")
    print(f"{'=' * 60}{RESET}")
    print(f"  {GREEN}OK:   {ok_count}{RESET}")
    print(f"  {YELLOW}SKIP: {skip_count}{RESET}")
    print(f"  {RED}MISS: {miss_count}{RESET}")

    if miss_count > 0:
        print(f"\n  {YELLOW}Some fixes missed — review MISS items above.{RESET}")
    else:
        print(f"\n  {GREEN}All fixes applied!{RESET}")
        print(f"  Now run:  python test_toupcam_chain.py")
        print(f"  Then:     python main.py")

    return 1 if miss_count > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
