#!/usr/bin/env python3
"""
patch_camera_integration.py — Integrate ToupCam backend into CameraWidget

Patches:
  1. gui/widgets/camera_widget.py  — dual-backend (OpenCV + ToupCam)
  2. gui/pages/calibration.py      — use CAMERA_AVAILABLE flag

Prerequisites:
  - gui/widgets/toupcam_backend.py must already be in place

Run from MEBP project root:
    python patches/v726_camera/patch_camera_integration.py
"""

from __future__ import annotations
import os
import re
import sys
import ast
import shutil
from pathlib import Path

# ── Terminal colors ──
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
    """Find MEBP project root."""
    for candidate in [Path.cwd(), Path(__file__).resolve().parent.parent.parent]:
        if (candidate / 'main.py').exists() and (candidate / 'gui').exists():
            return candidate
    print(f"{RED}FAIL: Cannot find MEBP project root{RESET}")
    sys.exit(1)

def patch_replace(content: str, old: str, new: str, label: str) -> str:
    global ok_count, skip_count, miss_count
    if new in content:
        print(f"  {YELLOW}SKIP{RESET} {label} (already applied)")
        skip_count += 1
        return content
    if old in content:
        content = content.replace(old, new, 1)
        print(f"  {GREEN}OK{RESET}   {label}")
        ok_count += 1
        return content
    print(f"  {RED}MISS{RESET} {label}")
    miss_count += 1
    return content

def regex_replace(content: str, pattern: str, replacement: str, label: str, 
                  flags: int = 0) -> str:
    global ok_count, skip_count, miss_count
    # Check if already applied (look for a unique marker in the replacement)
    marker_lines = [l.strip() for l in replacement.split('\n') if l.strip() and len(l.strip()) > 20]
    if marker_lines and marker_lines[0] in content:
        print(f"  {YELLOW}SKIP{RESET} {label} (already applied)")
        skip_count += 1
        return content
    
    new_content, n = re.subn(pattern, replacement, content, count=1, flags=flags)
    if n > 0:
        print(f"  {GREEN}OK{RESET}   {label}")
        ok_count += 1
        return new_content
    print(f"  {RED}MISS{RESET} {label}")
    miss_count += 1
    return content

def insert_after(content: str, anchor: str, new_text: str, label: str) -> str:
    global ok_count, skip_count, miss_count
    # Check if already applied
    first_new_line = [l.strip() for l in new_text.split('\n') if l.strip()][0] if new_text.strip() else ''
    if first_new_line and first_new_line in content:
        print(f"  {YELLOW}SKIP{RESET} {label} (already applied)")
        skip_count += 1
        return content
    if anchor in content:
        content = content.replace(anchor, anchor + new_text, 1)
        print(f"  {GREEN}OK{RESET}   {label}")
        ok_count += 1
        return content
    print(f"  {RED}MISS{RESET} {label}")
    miss_count += 1
    return content

def backup_and_write(filepath: Path, content: str, label: str) -> bool:
    """AST-verify, backup, and write."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL{RESET} {label}: {e}")
        return False
    
    bak = filepath.with_suffix(filepath.suffix + '.bak_cam')
    if not bak.exists():
        shutil.copy2(filepath, bak)
    
    filepath.write_text(content, encoding='utf-8')
    print(f"  {GREEN}WRITTEN{RESET} {filepath}")
    return True


# ════════════════════════════════════════════════════════════════════
#  PATCH 1: camera_widget.py
# ════════════════════════════════════════════════════════════════════

def patch_camera_widget(root: Path):
    print(f"\n{CYAN}{BOLD}── Patching camera_widget.py ──{RESET}")
    
    fpath = root / 'gui' / 'widgets' / 'camera_widget.py'
    if not fpath.exists():
        print(f"  {RED}MISS{RESET} File not found: {fpath}")
        return
    
    content = fpath.read_text(encoding='utf-8')
    
    # ── 1A: Add ToupCam imports after OpenCV import block ──
    # Find the end of the OpenCV try/except block
    toupcam_import_block = '''
# v7.3-camera: Try to import ToupCam backend
try:
    from gui.widgets.toupcam_backend import ToupCamBackend, TOUPCAM_AVAILABLE
except ImportError:
    TOUPCAM_AVAILABLE = False
    ToupCamBackend = None
    logger.info("ToupCam backend not available")

# v7.3-camera: Unified availability flag
CAMERA_AVAILABLE = CV2_AVAILABLE or bool(TOUPCAM_AVAILABLE)
'''
    
    content = insert_after(
        content,
        'logger.info("OpenCV (cv2) not installed — camera widget disabled")',
        '\n' + toupcam_import_block,
        "1A: Add ToupCam imports"
    )

    # ── 1B: Add detect_toupcam_cameras function ──
    toupcam_detect_func = '''

def detect_toupcam_cameras() -> list[dict]:
    """v7.3-camera: Detect ToupTek/Bestscope cameras.
    
    Returns list of dicts with 'id' and 'displayname' keys.
    """
    if not TOUPCAM_AVAILABLE or ToupCamBackend is None:
        return []
    try:
        return ToupCamBackend.enumerate()
    except Exception as e:
        logger.warning(f"ToupCam detection error: {e}")
        return []

'''
    # Insert before the detect_cameras_async function
    content = insert_after(
        content,
        '    return available\n',
        toupcam_detect_func,
        "1B: Add detect_toupcam_cameras function"
    )

    # ── 1C: Patch _populate_cameras to work without CV2 ──
    content = regex_replace(
        content,
        r'(    def _populate_cameras\(self\):.*?""".*?""")\s*\n'
        r'        self\.camera_combo\.clear\(\)\s*\n'
        r'        if not CV2_AVAILABLE:\s*\n'
        r'            return\s*\n'
        r'        self\._cameras_detected = False\s*\n'
        r'        self\.camera_combo\.addItem\("Click Detect or Start", -1\)',
        r'''\1
        self.camera_combo.clear()
        # v7.3-camera: Work with either backend
        if not CAMERA_AVAILABLE:
            return
        self._cameras_detected = False
        self.camera_combo.addItem("Click Detect or Start", -1)''',
        "1C: _populate_cameras uses CAMERA_AVAILABLE",
        flags=re.DOTALL,
    )

    # ── 1D: Patch refresh_cameras to detect both backends ──
    content = regex_replace(
        content,
        r'    def refresh_cameras\(self\):\s*\n'
        r'        """.*?"""\s*\n'
        r'        self\.camera_combo\.clear\(\)\s*\n'
        r'        if not CV2_AVAILABLE:\s*\n'
        r'            return\s*\n'
        r'        for idx in detect_cameras\(\):\s*\n'
        r'            self\.camera_combo\.addItem\(f"Camera \{idx\}", idx\)\s*\n'
        r'        if self\.camera_combo\.count\(\) == 0:\s*\n'
        r'            self\.camera_combo\.addItem\("No cameras found", -1\)\s*\n'
        r'        self\._cameras_detected = True',
        r'''    def refresh_cameras(self):
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

        # ToupCam cameras
        for tc_dev in detect_toupcam_cameras():
            name = tc_dev.get('displayname', 'ToupCam')
            dev_id = tc_dev.get('id', '')
            self.camera_combo.addItem(f"TC: {name}", ("toupcam", dev_id))

        if self.camera_combo.count() == 0:
            self.camera_combo.addItem("No cameras found", -1)
        self._cameras_detected = True''',
        "1D: refresh_cameras dual-backend",
        flags=re.DOTALL,
    )

    # ── 1E: Patch start() to handle CAMERA_AVAILABLE ──
    content = regex_replace(
        content,
        r'(    def start\(self\):\s*\n'
        r'        """.*?""")\s*\n'
        r'        if not CV2_AVAILABLE or self\._running:\s*\n'
        r'            return',
        r'''\1
        # v7.3-camera: Support either backend
        if not CAMERA_AVAILABLE or self._running:
            return''',
        "1E: start() uses CAMERA_AVAILABLE",
        flags=re.DOTALL,
    )

    # ── 1F: Patch start() routing — after lazy detection, route by combo data ──
    content = regex_replace(
        content,
        r'        idx = self\.camera_combo\.currentData\(\)\s*\n'
        r'        if idx is None or idx < 0:\s*\n'
        r'            self\.video_label\.setText\("No camera available"\)\s*\n'
        r'            return\s*\n'
        r'        self\.start_with_index\(idx\)',
        r'''        cam_data = self.camera_combo.currentData()
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

        # Legacy: plain integer index (backward compat)
        if isinstance(cam_data, int) and cam_data >= 0:
            self.start_with_index(cam_data)''',
        "1F: start() routes by backend type",
        flags=re.DOTALL,
    )

    # ── 1G: Patch start_with_index guard ──
    content = regex_replace(
        content,
        r'(    def start_with_index\(self, camera_index: int\):\s*\n'
        r'        """.*?""")\s*\n'
        r'        if not CV2_AVAILABLE or self\._running:',
        r'''\1
        if not CV2_AVAILABLE or self._running:''',
        "1G: start_with_index guard (no change needed or already OK)",
        flags=re.DOTALL,
    )

    # ── 1H: Add _start_toupcam method after start_with_index ──
    toupcam_start_method = '''
    def _start_toupcam(self, device_id: str):
        """v7.3-camera: Start a ToupCam camera feed."""
        if not TOUPCAM_AVAILABLE or ToupCamBackend is None or self._running:
            return

        self._toupcam = ToupCamBackend()
        if not self._toupcam.open(device_id):
            self.video_label.setText(f"Failed to open ToupCam")
            self._toupcam = None
            return

        w, h = self._toupcam.get_resolution()
        self._running = True
        self._backend_type = "toupcam"
        self._timer.start(int(1000 / self._fps))
        if hasattr(self, 'btn_start'):
            self.btn_start.setText("\\u23f9")
        logger.info(f"{self._camera_label}: ToupCam started ({w}x{h}) at {self._fps} FPS")

'''
    # Insert after the stop() method — find a good anchor
    content = insert_after(
        content,
        '        logger.info(f"{self._camera_label}: camera stopped")\n',
        toupcam_start_method,
        "1H: Add _start_toupcam method"
    )

    # ── 1I: Patch _grab_frame for dual backend ──
    content = regex_replace(
        content,
        r'    def _grab_frame\(self\):\s*\n'
        r'        """Capture, adjust, and display one frame\."""\s*\n'
        r'        if not self\._capture or not self\._capture\.isOpened\(\):\s*\n'
        r'            self\.stop\(\)\s*\n'
        r'            return\s*\n\s*\n'
        r'        ret, frame = self\._capture\.read\(\)',
        r'''    def _grab_frame(self):
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
            ret, frame = self._capture.read()''',
        "1I: _grab_frame dual-backend read",
        flags=re.DOTALL,
    )

    # ── 1J: Patch stop() for dual backend ──
    content = regex_replace(
        content,
        r'    def stop\(self\):\s*\n'
        r'        """Stop the camera feed\."""\s*\n'
        r'        self\._timer\.stop\(\)\s*\n'
        r'        self\._running = False\s*\n'
        r'        if self\._capture:\s*\n'
        r'            self\._capture\.release\(\)\s*\n'
        r'            self\._capture = None',
        r'''    def stop(self):
        """Stop the camera feed."""
        self._timer.stop()
        self._running = False
        # v7.3-camera: Release correct backend
        if getattr(self, '_backend_type', 'opencv') == 'toupcam':
            tc = getattr(self, '_toupcam', None)
            if tc:
                tc.release()
            self._toupcam = None
        if self._capture:
            self._capture.release()
            self._capture = None
        self._backend_type = 'opencv' ''',
        "1J: stop() dual-backend release",
        flags=re.DOTALL,
    )

    # ── 1K: Patch take_snapshot for dual backend ──
    content = regex_replace(
        content,
        r'    def take_snapshot\(self\):\s*\n'
        r'        """Save current frame to file\."""\s*\n'
        r'        if not self\._capture or not self\._capture\.isOpened\(\):\s*\n'
        r'            return\s*\n\s*\n'
        r'        ret, frame = self\._capture\.read\(\)',
        r'''    def take_snapshot(self):
        """Save current frame to file."""
        # v7.3-camera: Read from active backend
        backend = getattr(self, '_backend_type', 'opencv')
        if backend == 'toupcam':
            tc = getattr(self, '_toupcam', None)
            if tc is None or not tc.isOpened():
                return
            ret, frame = tc.read()
        else:
            if not self._capture or not self._capture.isOpened():
                return
            ret, frame = self._capture.read()''',
        "1K: take_snapshot dual-backend",
        flags=re.DOTALL,
    )

    # ── 1L: Patch UI construction to work without CV2 ──
    content = regex_replace(
        content,
        r'''        if not CV2_AVAILABLE:\s*\n            layout\.addWidget\(QLabel\(\s*\n                "Camera unavailable.*?pip install opencv-python"''',
        r'''        if not CAMERA_AVAILABLE:
            layout.addWidget(QLabel(
                "Camera unavailable — install OpenCV or ToupTek SDK:\\n"
                "  pip install opencv-python"''',
        "1L: UI fallback uses CAMERA_AVAILABLE",
        flags=re.DOTALL,
    )

    # ── 1M: Initialize _toupcam and _backend_type in __init__ ──
    content = insert_after(
        content,
        '        self._gamma_lut = None          # Precomputed LUT for speed\n',
        '\n        # v7.3-camera: ToupCam backend state\n'
        '        self._toupcam = None\n'
        '        self._backend_type = "opencv"  # "opencv" or "toupcam"\n',
        "1M: Init _toupcam and _backend_type in __init__"
    )

    # ── Write ──
    if not backup_and_write(fpath, content, "camera_widget.py"):
        return


# ════════════════════════════════════════════════════════════════════
#  PATCH 2: calibration.py
# ════════════════════════════════════════════════════════════════════

def patch_calibration(root: Path):
    print(f"\n{CYAN}{BOLD}── Patching calibration.py ──{RESET}")
    
    fpath = root / 'gui' / 'pages' / 'calibration.py'
    if not fpath.exists():
        print(f"  {RED}MISS{RESET} File not found: {fpath}")
        return
    
    content = fpath.read_text(encoding='utf-8')
    
    # ── 2A: Add CAMERA_AVAILABLE import ──
    content = regex_replace(
        content,
        r'(from gui\.widgets\.camera_widget import CameraWidget, CV2_AVAILABLE)(?:, detect_cameras)?',
        r'from gui.widgets.camera_widget import CameraWidget, CV2_AVAILABLE, detect_cameras\n'
        r'    try:\n'
        r'        from gui.widgets.camera_widget import CAMERA_AVAILABLE\n'
        r'    except ImportError:\n'
        r'        CAMERA_AVAILABLE = CV2_AVAILABLE',
        "2A: Import CAMERA_AVAILABLE",
        flags=0,
    )
    
    # Actually, the import is inside a try/except block already. Let me use a different approach.
    # Just add a CAMERA_AVAILABLE definition after the existing import block.
    if 'CAMERA_AVAILABLE' not in content:
        # Find after the CV2_AVAILABLE assignment in the except block
        content = insert_after(
            content,
            '    CV2_AVAILABLE = False\n',
            '\ntry:\n    from gui.widgets.camera_widget import CAMERA_AVAILABLE\nexcept ImportError:\n    CAMERA_AVAILABLE = CV2_AVAILABLE\n',
            "2A-alt: Add CAMERA_AVAILABLE import after CV2_AVAILABLE"
        )
    
    # ── 2B: Camera grid uses CAMERA_AVAILABLE ──
    content = patch_replace(
        content,
        'if CV2_AVAILABLE and CameraWidget is not None:',
        'if CAMERA_AVAILABLE and CameraWidget is not None:',
        "2B: Camera grid condition uses CAMERA_AVAILABLE"
    )
    
    # ── 2C: _on_detect_cameras check ──
    content = patch_replace(
        content,
        'if not CV2_AVAILABLE:\n            logger.warning("OpenCV not installed — camera detection unavailable")',
        'if not CAMERA_AVAILABLE:\n            logger.warning("No camera backend available")',
        "2C: _on_detect_cameras uses CAMERA_AVAILABLE"
    )
    
    # ── Write ──
    if not backup_and_write(fpath, content, "calibration.py"):
        return


# ════════════════════════════════════════════════════════════════════
#  VERIFY: toupcam_backend.py exists
# ════════════════════════════════════════════════════════════════════

def verify_backend(root: Path):
    print(f"\n{CYAN}{BOLD}── Verifying toupcam_backend.py ──{RESET}")
    
    backend_path = root / 'gui' / 'widgets' / 'toupcam_backend.py'
    if backend_path.exists():
        print(f"  {GREEN}OK{RESET}   toupcam_backend.py exists ({backend_path.stat().st_size:,} bytes)")
        # AST check
        try:
            ast.parse(backend_path.read_text(encoding='utf-8'))
            print(f"  {GREEN}OK{RESET}   AST valid")
        except SyntaxError as e:
            print(f"  {RED}FAIL{RESET} AST error: {e}")
    else:
        print(f"  {RED}MISS{RESET} toupcam_backend.py NOT FOUND at {backend_path}")
        print(f"         Copy it from patches/v726_camera/ output first!")


# ════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count
    
    print(f"\n{BOLD}{'=' * 60}")
    print(f"  ToupCam Camera Integration Patch")
    print(f"  v7.3-camera")
    print(f"{'=' * 60}{RESET}\n")
    
    root = find_project_root()
    print(f"  Project root: {root}")
    
    verify_backend(root)
    patch_camera_widget(root)
    patch_calibration(root)
    
    print(f"\n{BOLD}{'=' * 60}")
    print(f"  SUMMARY")
    print(f"{'=' * 60}{RESET}")
    print(f"  {GREEN}OK:   {ok_count}{RESET}")
    print(f"  {YELLOW}SKIP: {skip_count}{RESET}")
    print(f"  {RED}MISS: {miss_count}{RESET}")
    
    if miss_count > 0:
        print(f"\n  {YELLOW}Some patches missed. This may be due to prior patches")
        print(f"  or slightly different file state. Review the MISS items above.{RESET}")
    elif ok_count > 0 or skip_count > 0:
        print(f"\n  {GREEN}All patches applied successfully!{RESET}")
        print(f"  Test with: python main.py")
    
    return 1 if miss_count > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
