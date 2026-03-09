#!/usr/bin/env python3
"""
test_toupcam_chain.py — Diagnose exactly where the ToupCam integration breaks.

Run from MEBP root:
    python test_toupcam_chain.py
"""

import sys
import os
from pathlib import Path

def check(label, condition, detail=""):
    icon = "PASS" if condition else "FAIL"
    print(f"  [{icon}] {label}")
    if detail:
        print(f"         {detail}")
    return condition

def main():
    print("=" * 60)
    print("  ToupCam Integration Chain Diagnostic")
    print("=" * 60)
    
    root = Path(__file__).resolve().parent
    print(f"\n  Project root: {root}")
    print(f"  Python: {sys.version}")
    
    # ── Step 1: File exists ──
    print("\n-- Step 1: File Existence --")
    backend_path = root / 'gui' / 'widgets' / 'toupcam_backend.py'
    check("toupcam_backend.py exists", backend_path.exists(), str(backend_path))
    
    widget_path = root / 'gui' / 'widgets' / 'camera_widget.py'
    check("camera_widget.py exists", widget_path.exists())
    
    # ── Step 2: Check camera_widget.py was patched ──
    print("\n-- Step 2: Patch Applied --")
    if widget_path.exists():
        content = widget_path.read_text(encoding='utf-8')
        check("TOUPCAM_AVAILABLE import present", 
              'TOUPCAM_AVAILABLE' in content)
        check("CAMERA_AVAILABLE defined",
              'CAMERA_AVAILABLE' in content)
        check("detect_toupcam_cameras function",
              'def detect_toupcam_cameras' in content)
        check("_start_toupcam method",
              'def _start_toupcam' in content)
        check("_backend_type in __init__",
              '_backend_type' in content)
        check("refresh_cameras dual-backend",
              '"toupcam"' in content and 'TC:' in content)
        check("_grab_frame dual-backend",
              "backend == 'toupcam'" in content or 
              'backend == "toupcam"' in content)
    
    # ── Step 3: Import toupcam_backend ──
    print("\n-- Step 3: Import toupcam_backend --")
    try:
        # Add project root to path so gui.widgets works
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        
        from gui.widgets.toupcam_backend import _find_toupcam_dll
        dll_path = _find_toupcam_dll()
        check("_find_toupcam_dll()", dll_path is not None, 
              f"Path: {dll_path}" if dll_path else "RETURNED NONE")
        
        if dll_path:
            p = Path(dll_path)
            check("DLL file exists", p.exists(), f"Size: {p.stat().st_size:,} bytes" if p.exists() else "")
    except Exception as e:
        check("Import toupcam_backend", False, f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    # ── Step 4: Load the library ──
    print("\n-- Step 4: Load Native Library --")
    try:
        from gui.widgets.toupcam_backend import _load_library
        lib = _load_library()
        check("_load_library()", lib is not None)
        
        if lib:
            try:
                func = lib.Toupcam_EnumV2
                check("Toupcam_EnumV2 function exists", True)
            except AttributeError:
                check("Toupcam_EnumV2 function exists", False)
    except Exception as e:
        check("_load_library()", False, f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    # ── Step 5: TOUPCAM_AVAILABLE flag ──
    print("\n-- Step 5: Availability Flags --")
    try:
        from gui.widgets.toupcam_backend import TOUPCAM_AVAILABLE
        avail = bool(TOUPCAM_AVAILABLE)
        check("TOUPCAM_AVAILABLE", avail, f"Value: {avail}")
    except Exception as e:
        check("TOUPCAM_AVAILABLE", False, f"Import error: {e}")
    
    try:
        from gui.widgets.camera_widget import CV2_AVAILABLE
        check("CV2_AVAILABLE", True, f"Value: {CV2_AVAILABLE}")
    except Exception as e:
        check("CV2_AVAILABLE import", False, f"{e}")
    
    try:
        from gui.widgets.camera_widget import CAMERA_AVAILABLE
        check("CAMERA_AVAILABLE", True, f"Value: {CAMERA_AVAILABLE}")
    except ImportError:
        check("CAMERA_AVAILABLE", False, "NOT DEFINED in camera_widget.py — patch missing!")
    except Exception as e:
        check("CAMERA_AVAILABLE", False, f"{e}")
    
    # ── Step 6: Enumerate cameras ──
    print("\n-- Step 6: Camera Enumeration --")
    try:
        from gui.widgets.toupcam_backend import ToupCamBackend
        devices = ToupCamBackend.enumerate()
        check("ToupCamBackend.enumerate()", len(devices) > 0,
              f"Found {len(devices)} device(s)")
        for i, dev in enumerate(devices):
            print(f"           [{i}] {dev.get('displayname', '?')} "
                  f"(id: {dev.get('id', '?')[:40]}...)")
    except Exception as e:
        check("ToupCamBackend.enumerate()", False, f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    try:
        from gui.widgets.camera_widget import detect_toupcam_cameras
        tc_devices = detect_toupcam_cameras()
        check("detect_toupcam_cameras()", len(tc_devices) > 0,
              f"Found {len(tc_devices)} device(s)")
    except ImportError:
        check("detect_toupcam_cameras()", False, "Function not found — patch missing!")
    except Exception as e:
        check("detect_toupcam_cameras()", False, f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    # ── Step 7: Quick capture test ──
    print("\n-- Step 7: Quick Capture Test --")
    try:
        from gui.widgets.toupcam_backend import ToupCamBackend
        devices = ToupCamBackend.enumerate()
        if devices:
            cam = ToupCamBackend()
            opened = cam.open(devices[0]['id'])
            check("ToupCamBackend.open()", opened)
            if opened:
                import time
                time.sleep(1)  # Let auto-exposure settle
                ok, frame = cam.read()
                check("ToupCamBackend.read()", ok and frame is not None,
                      f"Frame shape: {frame.shape}" if frame is not None else "No frame")
                cam.release()
                check("ToupCamBackend.release()", True)
        else:
            check("Quick capture", False, "No devices to test")
    except Exception as e:
        check("Quick capture", False, f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    # ── Summary ──
    print("\n" + "=" * 60)
    print("  If Step 3-4 FAIL: DLL not found or won't load")
    print("  If Step 5 FAIL:   Lazy flag not triggering")
    print("  If Step 6 FAIL:   Camera not detected by SDK")
    print("  If Step 2 FAIL:   Patch didn't apply to camera_widget.py")
    print("=" * 60)


if __name__ == "__main__":
    main()
