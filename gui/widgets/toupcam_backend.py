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

        # Set running BEFORE StartPullMode: callbacks begin dispatching the
        # instant the stream starts, and _on_event now gates on self._running,
        # so flipping it first avoids dropping the first frame(s).
        self._running = True
        hr = lib.Toupcam_StartPullModeWithCallback(
            self._handle, self._callback_ref, None
        )

        if hr < 0:
            logger.warning(f"ToupCam: StartPullMode failed (0x{hr & 0xFFFFFFFF:08X})")
            self._running = False
            lib.Toupcam_Close(self._handle)
            self._handle = None
            return False

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
        """Callback from ToupTek SDK thread.

        The native PullImageV3 write AND the buffer copy run under self._lock,
        and the handle/buffer are re-checked inside the lock. Teardown
        (release / set_resolution_index) takes the same lock before closing the
        handle or swapping _buf/_buf_ptr, so the SDK can never write into a
        freed/closed handle or a reallocated buffer. Lock hold time is bounded
        because PullImageV3 is called non-blocking (nWaitMS=0).
        """
        if nEvent == TOUPCAM_EVENT_IMAGE:
            try:
                with self._lock:
                    if not (self._running and self._handle and self._lib
                            and self._buf_ptr is not None
                            and self._buf is not None):
                        return
                    hr = self._lib.Toupcam_PullImageV3(
                        self._handle, self._buf_ptr, 0, 24, 0, None
                    )
                    if hr >= 0:
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
        """Stop and close the camera.

        Serialized against the SDK callback: clear _running first (so a fresh
        callback bails), then null the handle/buffers UNDER self._lock so any
        in-flight _on_event finishes its locked pull before we proceed. The
        native Stop/Close then run on a local handle — by then no callback can
        touch it (they see _handle=None and bail)."""
        self._running = False
        with self._lock:
            handle = self._handle
            lib = self._lib
            self._handle = None
            self._frame = None
            self._buf = None
            self._buf_ptr = None
            self._callback_ref = None
        if handle is not None and bool(handle) and lib:
            try:
                lib.Toupcam_Stop(handle)
            except Exception:
                pass
            try:
                lib.Toupcam_Close(handle)
            except Exception:
                pass
        self._frame_ready.clear()
        # Skip logging if the interpreter is shutting down (e.g. release()
        # invoked from __del__): the logging machinery may be half-torn-down,
        # and datetime/strftime in the console handler raises ImportError.
        if sys.meta_path is not None:
            logger.info("ToupCam released")
    
    def get_resolution(self) -> tuple[int, int]:
        """Return current (width, height)."""
        return (self._w, self._h)
    
    def set_resolution_index(self, index: int) -> bool:
        """Switch to a different resolution index.

        Restarts the pull-mode stream internally. Serialized against the SDK
        callback: _running is cleared before Stop (so dispatched callbacks
        bail), and the _buf/_buf_ptr swap happens UNDER self._lock so a pull
        can never write into a half-reallocated / wrong-sized buffer.
        """
        if not self._handle or not self._lib:
            return False

        # Gate the callback off, then stop dispatch.
        self._running = False
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
        new_w, new_h = w_val.value, h_val.value
        if new_w <= 0 or new_h <= 0:
            logger.warning("ToupCam: invalid size after eSize change")
            return False

        # Allocate the new buffer, then publish it atomically under the lock so
        # an in-flight pull (which holds the lock) can't see a torn state.
        new_buf = np.zeros((new_h, new_w, 3), dtype=np.uint8)
        new_ptr = new_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
        with self._lock:
            self._w, self._h = new_w, new_h
            self._buf = new_buf
            self._buf_ptr = new_ptr
            self._frame = None
        self._frame_ready.clear()

        # Restart capture — set running first so callbacks aren't dropped.
        self._running = True
        hr = self._lib.Toupcam_StartPullModeWithCallback(
            self._handle, self._callback_ref, None
        )
        if hr < 0:
            logger.warning(
                f"ToupCam: StartPullMode failed after resize "
                f"(0x{hr & 0xFFFFFFFF:08X})")
            self._running = False
            return False

        logger.info(f"ToupCam resolution changed: {self._w}x{self._h}")
        return True
    
    def set_auto_exposure(self, enabled: bool) -> bool:
        """Enable or disable auto-exposure."""
        if not self._handle or not self._lib:
            return False
        hr = self._lib.Toupcam_put_AutoExpoEnable(self._handle, 1 if enabled else 0)
        return hr >= 0

    # ── Hardware image controls (v7.5.x) ───────────────────────────
    # These set values ON the camera via the ToupTek SDK (firmware-side),
    # distinct from the post-capture software correction in CameraWidget.
    # Ranges per toupcam.h, validated against the BUC3D-1000C (C3CMOS10000KPA).

    # (min, max, default)
    HW_RANGES = {
        "brightness": (-64, 64, 0),
        "contrast": (-100, 100, 0),
        "gamma": (20, 180, 100),
    }

    def _bind(self, name: str, argtypes, restype=ctypes.c_int):
        """Resolve + cache a DLL function signature; None if the DLL lacks it."""
        lib = self._lib
        if lib is None:
            return None
        try:
            fn = getattr(lib, name)
        except AttributeError:
            return None
        fn.argtypes = argtypes
        fn.restype = restype
        return fn

    def _get_scalar(self, name: str, ctype=ctypes.c_int):
        """Call a ``get_X(handle, *out)`` HRESULT getter; return value or None."""
        if not self._handle or not bool(self._handle):
            return None
        fn = self._bind(name, [ctypes.c_void_p, ctypes.POINTER(ctype)])
        if fn is None:
            return None
        out = ctype()
        try:
            hr = fn(self._handle, ctypes.byref(out))
        except Exception as exc:
            logger.debug(f"ToupCam {name} failed: {exc}")
            return None
        return out.value if hr >= 0 else None

    def _put_scalar(self, name: str, value, ctype=ctypes.c_int) -> bool:
        """Call a ``put_X(handle, value)`` HRESULT setter; True on success."""
        if not self._handle or not bool(self._handle):
            return False
        fn = self._bind(name, [ctypes.c_void_p, ctype])
        if fn is None:
            return False
        try:
            hr = fn(self._handle, ctype(int(value)))
        except Exception as exc:
            logger.debug(f"ToupCam {name} failed: {exc}")
            return False
        return hr >= 0

    def get_brightness(self):
        return self._get_scalar("Toupcam_get_Brightness")

    def put_brightness(self, v) -> bool:
        return self._put_scalar("Toupcam_put_Brightness", v)

    def get_contrast(self):
        return self._get_scalar("Toupcam_get_Contrast")

    def put_contrast(self, v) -> bool:
        return self._put_scalar("Toupcam_put_Contrast", v)

    def get_gamma(self):
        return self._get_scalar("Toupcam_get_Gamma")

    def put_gamma(self, v) -> bool:
        return self._put_scalar("Toupcam_put_Gamma", v)

    def get_exposure_time(self):
        """Current exposure time in microseconds (or None)."""
        return self._get_scalar("Toupcam_get_ExpoTime", ctypes.c_uint)

    def put_exposure_time(self, microseconds) -> bool:
        return self._put_scalar(
            "Toupcam_put_ExpoTime", microseconds, ctypes.c_uint)

    def get_exposure_gain(self):
        """Analog gain in percent (100 = 1.0x), or None."""
        return self._get_scalar("Toupcam_get_ExpoAGain", ctypes.c_ushort)

    def put_exposure_gain(self, percent) -> bool:
        return self._put_scalar(
            "Toupcam_put_ExpoAGain", percent, ctypes.c_ushort)

    def get_auto_exposure(self):
        v = self._get_scalar("Toupcam_get_AutoExpoEnable")
        return None if v is None else bool(v)

    def _get_triple(self, name: str, ctype=ctypes.c_uint):
        if not self._handle or not bool(self._handle):
            return None
        fn = self._bind(name, [ctypes.c_void_p] + [ctypes.POINTER(ctype)] * 3)
        if fn is None:
            return None
        a, b, c = ctype(), ctype(), ctype()
        try:
            hr = fn(self._handle, ctypes.byref(a), ctypes.byref(b),
                    ctypes.byref(c))
        except Exception as exc:
            logger.debug(f"ToupCam {name} failed: {exc}")
            return None
        return (a.value, b.value, c.value) if hr >= 0 else None

    def get_exposure_time_range(self):
        """(min_us, max_us, default_us) or None."""
        return self._get_triple("Toupcam_get_ExpTimeRange", ctypes.c_uint)

    def get_exposure_gain_range(self):
        """(min_pct, max_pct, default_pct) or None."""
        return self._get_triple("Toupcam_get_ExpoAGainRange", ctypes.c_ushort)

    def get_eSize(self):
        """Current resolution index (0 = largest), or None."""
        return self._get_scalar("Toupcam_get_eSize", ctypes.c_uint)

    def get_resolution_list(self) -> list:
        """All supported (width, height) resolutions, queried from the device.

        Works around the empty list returned by EnumV2 on this SDK build by
        reading ResolutionNumber / get_Resolution from the open handle.
        """
        if not self._handle or not bool(self._handle):
            return []
        rn = self._bind("Toupcam_get_ResolutionNumber", [ctypes.c_void_p])
        gr = self._bind(
            "Toupcam_get_Resolution",
            [ctypes.c_void_p, ctypes.c_uint,
             ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)])
        if rn is None or gr is None:
            return []
        try:
            n = rn(self._handle)
        except Exception:
            return []
        out = []
        for i in range(max(int(n), 0)):
            w, h = ctypes.c_int(), ctypes.c_int()
            try:
                hr = gr(self._handle, i, ctypes.byref(w), ctypes.byref(h))
            except Exception:
                continue
            if hr >= 0 and w.value > 0 and h.value > 0:
                out.append((w.value, h.value))
        return out

    def get_settings(self) -> dict:
        """Read EVERY hardware setting back from the device (for the readout).

        All values are fetched via SDK getters against the open handle — never
        cached/echoed — so the caller can prove the values come FROM the camera.
        """
        return {
            "brightness": self.get_brightness(),
            "contrast": self.get_contrast(),
            "gamma": self.get_gamma(),
            "exposure_us": self.get_exposure_time(),
            "exposure_gain_pct": self.get_exposure_gain(),
            "auto_exposure": self.get_auto_exposure(),
            "exposure_range_us": self.get_exposure_time_range(),
            "gain_range_pct": self.get_exposure_gain_range(),
            "resolution": self.get_resolution(),
            "eSize": self.get_eSize(),
            "resolutions": self.get_resolution_list(),
            "device_id": self._device_id,
        }

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass
