"""
ToupCam Backend — ctypes wrapper for ToupTek/Bestscope microscope cameras.

v7.3-camera: Self-contained ToupTek camera interface that provides an
OpenCV-VideoCapture-like API for the CameraWidget dual-backend system.

Finds toupcam.dll automatically from:
  1. ToupView installation (Program Files)
  2. toupcam-master/ in the project root
  3. System PATH

Requires numpy for frame buffers. Does NOT require OpenCV.

Usage::

    from gui.widgets.toupcam_backend import ToupCamBackend, TOUPCAM_AVAILABLE

    if TOUPCAM_AVAILABLE:
        devices = ToupCamBackend.enumerate()
        cam = ToupCamBackend()
        if cam.open(devices[0]['id']):
            ok, frame = cam.read()  # BGR numpy array
            cam.release()
"""

from __future__ import annotations

import os
import sys
import struct
import ctypes
import ctypes.util
import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False

# ════════════════════════════════════════════════════════════════════
#  DLL Discovery
# ════════════════════════════════════════════════════════════════════

def _find_toupcam_dll() -> str | None:
    """Locate toupcam.dll on Windows (or equivalent on other OS).
    
    Search order:
      1. ToupView / ToupLite install dirs (Program Files)
      2. toupcam-master/ in project tree
      3. System PATH
    Returns full path string or None.
    """
    import platform
    is_64bit = struct.calcsize('P') == 8
    
    if platform.system() != 'Windows':
        # macOS / Linux: look for .dylib / .so
        lib_name = 'libtoupcam.dylib' if platform.system() == 'Darwin' else 'libtoupcam.so'
        # Check toupcam-master in project
        project_root = Path(__file__).resolve().parent.parent.parent
        for subdir in ['osx', 'x64', 'x86', '.']:
            candidate = project_root / 'toupcam-master' / subdir / lib_name
            if candidate.exists() and candidate.stat().st_size > 1000:
                return str(candidate)
        # System lib
        path = ctypes.util.find_library('toupcam')
        return path
    
    # ── Windows DLL search ──
    candidates = []
    
    arch_dir = 'x64' if is_64bit else 'x86'
    
    # 1. ToupView / ToupLite install directories
    prog_dirs = [
        os.environ.get('PROGRAMFILES', r'C:\Program Files'),
        os.environ.get('PROGRAMFILES(X86)', r'C:\Program Files (x86)'),
        os.environ.get('LOCALAPPDATA', ''),
    ]
    
    app_names = ['ToupTek', 'ToupView', 'ToupLite', 'Bestscope']
    
    for prog in prog_dirs:
        if not prog:
            continue
        for app in app_names:
            base = Path(prog) / app
            if not base.exists():
                continue
            # Search recursively for toupcam.dll
            for dll in base.rglob('toupcam.dll'):
                size = dll.stat().st_size
                if size < 1000:
                    continue  # Skip LFS stubs
                # Prefer matching architecture
                dll_str = str(dll).lower()
                is_64_dll = 'x64' in dll_str or '64' in dll_str
                arch_match = (is_64bit and is_64_dll) or (not is_64bit and not is_64_dll)
                candidates.append((dll, size, arch_match))
    
    # 2. toupcam-master/ in project tree
    project_root = Path(__file__).resolve().parent.parent.parent
    for subdir in [arch_dir, 'x64', 'x86']:
        candidate = project_root / 'toupcam-master' / subdir / 'toupcam.dll'
        if candidate.exists():
            size = candidate.stat().st_size
            if size > 1000:  # Not an LFS stub
                is_64_dll = 'x64' in subdir
                arch_match = (is_64bit and is_64_dll) or (not is_64bit and not is_64_dll)
                candidates.append((candidate, size, arch_match))
    
    # 3. System PATH
    for p in os.environ.get('PATH', '').split(os.pathsep):
        dll = Path(p) / 'toupcam.dll'
        if dll.exists() and dll.stat().st_size > 1000:
            candidates.append((dll, dll.stat().st_size, True))
    
    if not candidates:
        return None
    
    # Sort: architecture match first, then by size (larger = more complete)
    candidates.sort(key=lambda x: (x[2], x[1]), reverse=True)
    best = candidates[0][0]
    logger.info(f"ToupCam DLL found: {best} ({candidates[0][1]:,} bytes, "
                f"arch_match={candidates[0][2]})")
    return str(best)


# ════════════════════════════════════════════════════════════════════
#  Library Loading
# ════════════════════════════════════════════════════════════════════

_lib = None
_lib_error = None

def _load_library():
    """Load toupcam.dll and set up function signatures."""
    global _lib, _lib_error
    
    if _lib is not None:
        return _lib
    if _lib_error is not None:
        return None
    
    dll_path = _find_toupcam_dll()
    if dll_path is None:
        _lib_error = "toupcam.dll not found"
        logger.info(f"ToupCam: {_lib_error}")
        return None
    
    try:
        # Add DLL directory for dependency resolution
        dll_dir = str(Path(dll_path).parent)
        if hasattr(os, 'add_dll_directory'):
            try:
                os.add_dll_directory(dll_dir)
            except OSError:
                pass
        
        lib = ctypes.CDLL(dll_path)
        
        # Verify it's a real ToupTek library
        try:
            lib.Toupcam_EnumV2
        except AttributeError:
            _lib_error = f"DLL at {dll_path} has no ToupTek API"
            logger.warning(f"ToupCam: {_lib_error}")
            return None
        
        # ── Set up function signatures ──
        
        # EnumV2(ToupcamDeviceV2* pti) -> unsigned
        lib.Toupcam_EnumV2.restype = ctypes.c_uint
        # argtypes set at call time due to struct definition
        
        # Open(id) -> handle
        lib.Toupcam_Open.restype = ctypes.c_void_p
        lib.Toupcam_Open.argtypes = [ctypes.c_wchar_p]
        
        # Close(handle)
        lib.Toupcam_Close.argtypes = [ctypes.c_void_p]
        lib.Toupcam_Close.restype = None
        
        # Stop(handle)
        lib.Toupcam_Stop.argtypes = [ctypes.c_void_p]
        lib.Toupcam_Stop.restype = ctypes.c_int
        
        # get_Size(handle, *w, *h) -> HRESULT
        lib.Toupcam_get_Size.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]
        lib.Toupcam_get_Size.restype = ctypes.c_int
        
        # put_eSize(handle, index) -> HRESULT
        lib.Toupcam_put_eSize.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.Toupcam_put_eSize.restype = ctypes.c_int
        
        # get_eSize(handle, *index) -> HRESULT
        lib.Toupcam_get_eSize.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint),
        ]
        lib.Toupcam_get_eSize.restype = ctypes.c_int
        
        # PullImageV3(handle, pData, nWaitMS, nBPP, nRowPitch, pInfo) -> HRESULT
        lib.Toupcam_PullImageV3.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p,
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.c_void_p,
        ]
        lib.Toupcam_PullImageV3.restype = ctypes.c_int
        
        # put_AutoExpoEnable(handle, bEnable) -> HRESULT
        lib.Toupcam_put_AutoExpoEnable.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.Toupcam_put_AutoExpoEnable.restype = ctypes.c_int
        
        # Snap(handle, nResolutionIndex) -> HRESULT
        lib.Toupcam_Snap.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        lib.Toupcam_Snap.restype = ctypes.c_int

        # StartPullModeWithCallback(handle, callback, ctx) -> HRESULT
        lib.Toupcam_StartPullModeWithCallback.argtypes = [
            ctypes.c_void_p, _EVENT_CALLBACK, ctypes.c_void_p,
        ]
        lib.Toupcam_StartPullModeWithCallback.restype = ctypes.c_int
        
        _lib = lib
        logger.info(f"ToupCam library loaded: {dll_path}")
        return lib
        
    except OSError as e:
        _lib_error = f"Cannot load {dll_path}: {e}"
        logger.warning(f"ToupCam: {_lib_error}")
        return None


# ════════════════════════════════════════════════════════════════════
#  ctypes Structures
# ════════════════════════════════════════════════════════════════════

class _ToupcamResolution(ctypes.Structure):
    _fields_ = [
        ('width', ctypes.c_uint),
        ('height', ctypes.c_uint),
    ]

class _ToupcamModelV2(ctypes.Structure):
    """Simplified model struct — we only need a few fields."""
    _fields_ = [
        ('name', ctypes.c_wchar * 64),
        ('flag', ctypes.c_ulonglong),
        ('maxspeed', ctypes.c_uint),
        ('preview', ctypes.c_uint),
        ('still', ctypes.c_uint),
        ('maxfanspeed', ctypes.c_uint),
        ('ioctrol', ctypes.c_uint),
        ('xpixsz', ctypes.c_float),
        ('ypixsz', ctypes.c_float),
        ('res', _ToupcamResolution * 16),
    ]

class _ToupcamDeviceV2(ctypes.Structure):
    _fields_ = [
        ('displayname', ctypes.c_wchar * 64),
        ('id', ctypes.c_wchar * 64),
        ('model', ctypes.POINTER(_ToupcamModelV2)),
    ]


# ════════════════════════════════════════════════════════════════════
#  Event Constants
# ════════════════════════════════════════════════════════════════════

TOUPCAM_EVENT_EXPOSURE      = 1
TOUPCAM_EVENT_TEMPTINT      = 2
TOUPCAM_EVENT_IMAGE         = 4
TOUPCAM_EVENT_STILLIMAGE    = 5
TOUPCAM_EVENT_ERROR         = 0x80
TOUPCAM_EVENT_DISCONNECTED  = 0x40

# Callback type: void (__stdcall*)(unsigned nEvent, void* pCtx)
# v7.3-camera fix: Use WINFUNCTYPE (stdcall) on Windows, CFUNCTYPE (cdecl) elsewhere
import platform as _platform
if _platform.system() == 'Windows':
    _EVENT_CALLBACK = ctypes.WINFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)
else:
    _EVENT_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)


# ════════════════════════════════════════════════════════════════════
#  Availability Flag
# ════════════════════════════════════════════════════════════════════

def _check_available() -> bool:
    """Check if ToupCam SDK is usable (DLL loadable, numpy present)."""
    if not _NP_AVAILABLE:
        return False
    return _load_library() is not None

# Lazy evaluation — only load DLL when first needed
class _LazyAvailable:
    """Descriptor that loads the DLL on first access."""
    _value = None
    
    def __bool__(self):
        if self._value is None:
            self._value = _check_available()
        return self._value
    
    def __repr__(self):
        return str(bool(self))

TOUPCAM_AVAILABLE = _LazyAvailable()


# ════════════════════════════════════════════════════════════════════
#  ToupCamBackend — OpenCV-like Camera Interface
# ════════════════════════════════════════════════════════════════════

class ToupCamBackend:
    """
    ToupTek camera with OpenCV VideoCapture-like API.
    
    Thread safety: The ToupTek SDK fires callbacks from its own thread.
    Frame data is copied into a locked buffer. The GUI thread calls
    read() to get the latest frame.
    
    Usage::
    
        devices = ToupCamBackend.enumerate()
        cam = ToupCamBackend()
        cam.open(devices[0]['id'])
        ok, frame = cam.read()   # BGR numpy array
        cam.release()
    """
    
    @staticmethod
    def enumerate() -> list[dict]:
        """Enumerate connected ToupTek cameras.
        
        Returns list of dicts with keys:
            id:           Device ID string (pass to open())
            displayname:  Human-readable camera name
            resolutions:  List of (width, height) tuples
            preview_count: Number of preview resolutions
        """
        lib = _load_library()
        if lib is None:
            return []
        
        MAX_DEV = 16
        dev_array = (_ToupcamDeviceV2 * MAX_DEV)()
        
        try:
            count = lib.Toupcam_EnumV2(dev_array)
        except Exception as e:
            logger.warning(f"ToupCam enumerate failed: {e}")
            return []
        
        devices = []
        for i in range(count):
            dev = dev_array[i]
            info = {
                'id': dev.id,
                'displayname': dev.displayname or f"ToupCam #{i}",
                'resolutions': [],
                'preview_count': 0,
            }
            
            if dev.model:
                try:
                    model = dev.model.contents
                    info['preview_count'] = model.preview
                    for j in range(min(model.preview, 16)):
                        r = model.res[j]
                        if r.width > 0 and r.height > 0:
                            info['resolutions'].append((r.width, r.height))
                except Exception:
                    pass
            
            devices.append(info)
            logger.info(f"ToupCam found: {info['displayname']} ({info['id'][:30]}...)")
        
        return devices
    
    def __init__(self):
        self._handle = None
        self._lib = None
        self._buf: np.ndarray | None = None
        self._buf_ptr = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._w = 0
        self._h = 0
        self._running = False
        self._device_id = ""
        self._callback_ref = None  # prevent GC of callback
    
    def open(self, device_id: str, resolution_index: int | None = None) -> bool:
        """Open a ToupTek camera by device ID.
        
        Args:
            device_id: The 'id' string from enumerate()
            resolution_index: Resolution index (0=max, higher=smaller).
                If None, automatically picks a preview-friendly size.
        
        Returns True on success.
        """
        if self._handle is not None:
            self.release()
        
        lib = _load_library()
        if lib is None:
            return False
        self._lib = lib
        
        # v7.3-camera fix: Wrap handle as c_void_p for Python 3.12+ ctypes safety
        _raw_handle = lib.Toupcam_Open(device_id)
        self._handle = ctypes.c_void_p(_raw_handle)
        if self._handle is None or not self._handle:
            logger.warning(f"ToupCam: failed to open {device_id[:30]}")
            return False
        
        self._device_id = device_id
        
        # Set resolution
        if resolution_index is not None:
            lib.Toupcam_put_eSize(self._handle, resolution_index)
        else:
            # Auto-pick: find a resolution <= 1280 wide for smooth preview
            self._auto_pick_resolution()
        
        # Read actual resolution
        w_val, h_val = ctypes.c_int(), ctypes.c_int()
        lib.Toupcam_get_Size(self._handle, ctypes.byref(w_val), ctypes.byref(h_val))
        self._w = w_val.value
        self._h = h_val.value
        
        if self._w <= 0 or self._h <= 0:
            logger.warning("ToupCam: invalid resolution, using 640x480")
            self._w, self._h = 640, 480
        
        # Allocate buffer
        self._buf = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        self._buf_ptr = self._buf.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
        
        # Enable auto-exposure
        try:
            lib.Toupcam_put_AutoExpoEnable(self._handle, 1)
        except Exception:
            pass
        
        # Start pull mode with callback
        self._callback_ref = _EVENT_CALLBACK(self._on_event)
        
        hr = lib.Toupcam_StartPullModeWithCallback(
            self._handle, self._callback_ref, None
        )
        
        if hr < 0:
            logger.warning(f"ToupCam: StartPullMode failed (0x{hr & 0xFFFFFFFF:08X})")
            lib.Toupcam_Close(self._handle)
            self._handle = None
            return False
        
        self._running = True
        logger.info(f"ToupCam opened: {self._w}x{self._h}")
        return True
    
    def _auto_pick_resolution(self):
        """Pick a resolution suitable for live preview (<=1280 wide)."""
        if not self._handle or not self._lib:
            return
        
        # Get current eSize count by trying indices
        for idx in range(8):
            self._lib.Toupcam_put_eSize(self._handle, idx)
            w_val, h_val = ctypes.c_int(), ctypes.c_int()
            hr = self._lib.Toupcam_get_Size(
                self._handle, ctypes.byref(w_val), ctypes.byref(h_val)
            )
            if hr >= 0 and 0 < w_val.value <= 1280:
                logger.info(f"ToupCam auto-res: index {idx} = {w_val.value}x{h_val.value}")
                return
        
        # Fallback: use index 2 (typically 1/4 resolution)
        self._lib.Toupcam_put_eSize(self._handle, min(2, 0))
    
    def _on_event(self, nEvent, pCtx):
        """Callback from ToupTek SDK thread."""
        if nEvent == TOUPCAM_EVENT_IMAGE and self._handle and self._lib:
            try:
                hr = self._lib.Toupcam_PullImageV3(
                    self._handle, self._buf_ptr, 0, 24, 0, None
                )
                if hr >= 0:
                    with self._lock:
                        self._frame = self._buf.copy()
                    self._frame_ready.set()
            except Exception as e:
                logger.debug(f"ToupCam pull error: {e}")
        
        elif nEvent == TOUPCAM_EVENT_ERROR:
            logger.warning("ToupCam: error event received")
        elif nEvent == TOUPCAM_EVENT_DISCONNECTED:
            logger.warning("ToupCam: camera disconnected")
            self._running = False
    
    def isOpened(self) -> bool:
        """Check if camera is open and running."""
        # v7.3-camera fix: c_void_p(0) is falsy, c_void_p(None) is falsy
        handle_ok = self._handle is not None and bool(self._handle)
        return handle_ok and self._running
    
    def read(self) -> tuple[bool, np.ndarray | None]:
        """Read the latest frame (BGR numpy array).
        
        Returns (success, frame) matching OpenCV's VideoCapture.read() API.
        Non-blocking: returns the most recent frame from the callback buffer.
        """
        if not self.isOpened():
            return False, None
        
        # Wait briefly for first frame
        if self._frame is None:
            self._frame_ready.wait(timeout=0.5)
        
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
        
        return False, None
    
    def release(self):
        """Stop and close the camera."""
        if self._handle is not None and bool(self._handle) and self._lib:
            try:
                self._lib.Toupcam_Stop(self._handle)
            except Exception:
                pass
            try:
                self._lib.Toupcam_Close(self._handle)
            except Exception:
                pass
        
        self._handle = None
        self._running = False
        self._frame = None
        self._buf = None
        self._buf_ptr = None
        self._callback_ref = None
        self._frame_ready.clear()
        logger.info("ToupCam released")
    
    def get_resolution(self) -> tuple[int, int]:
        """Return current (width, height)."""
        return (self._w, self._h)
    
    def set_resolution_index(self, index: int) -> bool:
        """Switch to a different resolution index.
        
        Note: This restarts the capture internally.
        """
        if not self._handle or not self._lib:
            return False
        
        # Stop current capture
        try:
            self._lib.Toupcam_Stop(self._handle)
        except Exception:
            pass
        
        hr = self._lib.Toupcam_put_eSize(self._handle, index)
        if hr < 0:
            return False
        
        # Re-read size
        w_val, h_val = ctypes.c_int(), ctypes.c_int()
        self._lib.Toupcam_get_Size(self._handle, ctypes.byref(w_val), ctypes.byref(h_val))
        self._w = w_val.value
        self._h = h_val.value
        
        # Reallocate buffer
        self._buf = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        self._buf_ptr = self._buf.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
        self._frame = None
        self._frame_ready.clear()
        
        # Restart capture
        hr = self._lib.Toupcam_StartPullModeWithCallback(
            self._handle, self._callback_ref, None
        )
        
        logger.info(f"ToupCam resolution changed: {self._w}x{self._h}")
        return hr >= 0
    
    def set_auto_exposure(self, enabled: bool) -> bool:
        """Enable or disable auto-exposure."""
        if not self._handle or not self._lib:
            return False
        hr = self._lib.Toupcam_put_AutoExpoEnable(self._handle, 1 if enabled else 0)
        return hr >= 0
    
    def __del__(self):
        try:
            self.release()
        except Exception:
            pass
