"""
TUCam Backend — ctypes wrapper for Tucsen (TUCam SDK) scientific cameras.

v7.9.x: Self-contained Tucsen camera interface exposing the same
OpenCV-VideoCapture-like API the ``CameraWidget`` multi-backend system already
expects from ``ToupCamBackend`` and ``AndorBackend``. Target: the **Tucsen Libra
25**, evaluated as an alternate MICROSCOPE camera alongside the ANDOR Zyla.

The SDK is ``TUCam.dll`` (Xintu Photonics / Fuzhou Tucsen). This backend drives
exactly the function set Tucsen's own Micro-Manager adapter uses, which is the
canonical live-preview flow::

    TUCAM_Api_Init  →  TUCAM_Dev_Open  →  TUCAM_Buf_Alloc  →  TUCAM_Cap_Start
      →  [reader thread: TUCAM_Buf_WaitForFrame]  →
    TUCAM_Cap_Stop  →  TUCAM_Buf_Release  →  TUCAM_Dev_Close  →  TUCAM_Api_Uninit

Everything is lazy + guarded: importing this module never loads the SDK.
``TUCAM_AVAILABLE`` only becomes truthy on first access, when numpy imports AND
TUCam.dll is found AND loads. With no SDK the app behaves as if no Tucsen camera
exists.

✅ HARDWARE-VERIFIED on a real Libra 25 (2026-08-04)
---------------------------------------------------
Confirmed live: SDK reports the camera · open/stream/release · **mono 16-bit**
frames decoded to BGR8 at 2600x2048 · both resolution modes discovered from the
SDK's own labels · exposure control.

**⭐ The property-ID table was WRONG until that bench session, and the error was
not benign.** It had put ``EXPOSURETM`` at 4, which on this camera is
``TEMPERATURE`` — so "set exposure" would have written a cooling setpoint. The
``GetAttr`` gate could NOT catch that, because property 4 genuinely exists; only
reading the real values did. The corrected IDs and the observed ranges are
documented at ``TUIDP_*`` below. This is the same class of error as this repo's
Nikon-Ti ProgID and its 40× focus scale — a plausible-looking mapping that is
simply not what the device implements.

Two design choices are what make the SDK's answers, not our guesses, the
authority — keep them if you extend this:
  * Every property access is gated on **``TUCAM_Prop_GetAttr``** first, so an ID
    a model does not implement degrades to "control unavailable" instead of
    silently touching a neighbouring property. ``hardware_capabilities()``
    therefore only advertises controls this camera actually has (the Libra 25
    has no BRIGHTNESS, and none is offered).
  * **Resolutions are discovered, never assumed**: the capability sweep keeps
    whichever ID's own value-texts parse as ``WxH``. On this camera that found
    capability 0 → ``5200x4096 (Resolution)`` and ``2600x2048 (Sensitive)``,
    while correctly *not* mistaking capability 37's ``1x1Normal / 2x2Bin_Sum``
    binning labels for resolutions.

:meth:`TUCamBackend.diagnostics` dumps every property + capability a camera
actually implements with its real range — run it first on any new Tucsen model
rather than trusting the table.

Two units traps worth naming (this codebase has been bitten by both before):
  * **Exposure is milliseconds in TUCam, microseconds in this app's contract.**
    Converted in one place (:meth:`_exposure_scale_us`). Proven on hardware by a
    physical consequence rather than by readback alone: commanding 10 ms gave a
    0.035 s frame interval, 300 ms gave 0.228 s.
  * **Pixel format is read from the frame, never assumed.** ``ucChannels`` /
    ``ucElemBytes`` / ``uiWidthStep`` decide how the buffer is interpreted, so a
    mono-16, color-24 or BGRA-32 sensor all decode correctly. The Libra 25
    reports ``channels=1, elem_bytes=2``.

Usage::

    from gui.widgets.tucam_backend import TUCamBackend, TUCAM_AVAILABLE

    if TUCAM_AVAILABLE:
        devices = TUCamBackend.enumerate()
        cam = TUCamBackend()
        if cam.open(devices[0]['id']):
            ok, frame = cam.read()   # BGR uint8 numpy array
            cam.release()
"""

from __future__ import annotations

import ctypes
import logging
import os
import re
import sys
import threading
import time
from ctypes import (POINTER, Structure, byref, c_char, c_char_p, c_double,
                    c_int, c_uint, c_ubyte, c_ushort, c_void_p, create_string_buffer)
from pathlib import Path

logger = logging.getLogger(__name__)

try:
    import numpy as np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False

# Shared with the Andor backend so both mono scientific cameras render through
# IDENTICAL display math (see gui/widgets/mono_display.py for why that matters).
from gui.widgets.mono_display import LEVEL_MAX, _mono_to_bgr8, _auto_levels


# ════════════════════════════════════════════════════════════════════
#  SDK constants
# ════════════════════════════════════════════════════════════════════

TUCAMRET_SUCCESS = 0x00000001

# Frame formats requested via TUCAM_FRAME.ucFormatGet.
TUFRM_FMT_RAW = 0x10      # sensor raw (undebayered)
TUFRM_FMT_USUAL = 0x11    # "usual": mono16 for mono, RGB for color
TUFRM_FMT_RGB888 = 0x12   # always 8-bit 3-channel

# ── TUCAM_IDINFO (TUCAM_Dev_GetInfo / _GetInfoEx) ────────────────────
# HARDWARE-CONFIRMED on a Libra 25: these return NUMBERS in `nValue`, and this
# SDK build returns no model text at all (every text ID came back empty, via
# both the open handle and GetInfoEx-by-index). So the model name is resolved
# from the OS instead -- see _read_model.
TUCSEN_USB_VID = 0x5453    # 21587, as the SDK reports it (info id 2)
TUIDI_VENDOR = 2           # -> 21587 = 0x5453, the Tucsen USB VID
TUIDI_PRODUCT = 3          # -> 58423 = 0xE437 on the Libra 25
_INFO_MODEL_CANDIDATES = (8, 7, 6)   # string IDs, if a future model provides one

# ── TUCAM_IDPROP (TUCAM_Prop_*) ──────────────────────────────────────
# ⭐ CONFIRMED ON A REAL Libra 25 (2026-08-04) -- these were WRONG before, and
# the error was not benign: the old table put EXPOSURETM at 4, which is
# TEMPERATURE, so "set exposure" would have written a cooling setpoint. The
# GetAttr gate could not catch it because property 4 *does* exist.
#
# The enum is sequential from 0. Observed on this camera (id: range, default):
#     0  GLOBALGAIN   0..3,      dft 2      (a gain MODE selector, not a %)
#     1  EXPOSURETM   0.0063..5.76e6 ms, dft 5.23   <-- exposure, MILLISECONDS
#     3  BLACKLEVEL   0..255,    dft 8
#     4  TEMPERATURE  500..1000, dft 500    (reads 0.375, i.e. outside its own
#                                            declared range -> not trustworthy)
#     8  GAMMA        1..255,    dft 100
#     9  CONTRAST     0..255,    dft 128
#    10  LFTLEVELS    0..16382,  dft 0      (14-bit black point)
#    11  RGTLEVELS    1..16383,  dft 16383  (14-bit white point)
# BRIGHTNESS (2) is absent on this model, so it is correctly not advertised.
#
# Exposure = property 1 in ms is proven by a physical consequence, not by
# readback alone: commanding 10 ms gave a 0.035 s frame interval and 300 ms gave
# 0.228 s. See the update plan.
TUIDP_GLOBALGAIN = 0
TUIDP_EXPOSURETM = 1
TUIDP_BRIGHTNESS = 2
TUIDP_BLACKLEVEL = 3
TUIDP_TEMPERATURE = 4
TUIDP_SHARPNESS = 5
TUIDP_NOISELEVEL = 6
TUIDP_HDR_KVALUE = 7
TUIDP_GAMMA = 8
TUIDP_CONTRAST = 9
TUIDP_LFTLEVELS = 10
TUIDP_RGTLEVELS = 11

# ── TUCAM_IDCAPA (TUCAM_Capa_*) ──────────────────────────────────────
# ⭐ ALSO CONFIRMED ON THE REAL Libra 25 — and ATEXPOSURE was wrong too (it was
# 8, whose range is 0..3, i.e. not a boolean; that should have been the tell).
# Proven the same way as exposure, by a physical consequence rather than a
# readback: enabling capability **3** made the exposure property self-adjust
# (20.0 -> 44.5 -> 145.8 -> 200.0 ms, converging on a target), while capability 8
# left it pinned at 20.001 ms. Capability 8 is auto-LEVELS, hence its 0..3 range.
#
# Observed on this camera (id: range, default [value texts]):
#     0  RESOLUTION   0..1  [0=5200x4096(Resolution), 1=2600x2048(Sensitive)]
#     1  PIXELCLOCK   0..0  [0=High]
#     2  BITOFDEPTH   16..16
#     3  ATEXPOSURE   0..1  <-- auto-exposure
#     4  HORIZONTAL   0..1  (mirror)
#     5  VERTICAL     0..1  (flip)
#     8  ATLEVELS     0..3
#    37  (binning)    0..1  [0=1x1Normal, 1=2x2Bin_Sum]
TUIDC_RESOLUTION = 0       # discovered empirically anyway — see _discover_resolutions
TUIDC_PIXELCLOCK = 1
TUIDC_BITOFDEPTH = 2
TUIDC_ATEXPOSURE = 3
TUIDC_HORIZONTAL = 4
TUIDC_VERTICAL = 5
TUIDC_ATLEVELS = 8
# Resolution is still discovered from the SDK's own value-texts rather than
# trusting TUIDC_RESOLUTION, so a model that numbers it differently still works.
_CAPA_SWEEP_MAX = 40
_PROP_SWEEP_MAX = 64

# Live-preview defaults.
_PREVIEW_MAX_WIDTH = 1280   # auto-pick the largest resolution at or under this
_WAIT_FRAME_TIMEOUT_MS = 1000
_DEFAULT_EXPOSURE_MS = 30.0


# ════════════════════════════════════════════════════════════════════
#  ctypes structures
# ════════════════════════════════════════════════════════════════════

class TUCAM_INIT(Structure):
    _fields_ = [("uiCamCount", c_uint), ("pstrConfigPath", c_char_p)]


class TUCAM_OPEN(Structure):
    _fields_ = [("uiIdxOpen", c_uint), ("hIdxTUCam", c_void_p)]


# `pText` is the caller-supplied output buffer the SDK writes a string into.
# It is declared POINTER(c_char) rather than c_char_p because the two are
# ABI-identical for a C `char*`, but ctypes coerces a c_char_p *field* to an
# immutable `bytes` on read — which would discard the pointer and make the
# out-parameter unusable/unverifiable. POINTER(c_char) keeps it a real pointer.
class TUCAM_VALUE_INFO(Structure):
    _fields_ = [("nID", c_int), ("nValue", c_int),
                ("pText", POINTER(c_char)), ("nTextSize", c_int)]


class TUCAM_VALUE_TEXT(Structure):
    _fields_ = [("nID", c_int), ("dbValue", c_double),
                ("pText", POINTER(c_char)), ("nTextSize", c_int)]


class TUCAM_PROP_ATTR(Structure):
    _fields_ = [("idProp", c_int), ("nIdxChn", c_int),
                ("dbValMin", c_double), ("dbValMax", c_double),
                ("dbValDft", c_double), ("dbValStep", c_double)]


class TUCAM_CAPA_ATTR(Structure):
    _fields_ = [("idCapa", c_int), ("nValMin", c_int), ("nValMax", c_int),
                ("nValDft", c_int), ("nValStep", c_int)]


class TUCAM_FRAME(Structure):
    """Frame descriptor. The SDK fills in the geometry/format fields — we read
    them rather than assuming a pixel layout."""
    _fields_ = [
        ("szSignature", c_char * 8),
        ("usHeader", c_ushort),
        ("usOffset", c_ushort),
        ("usWidth", c_ushort),
        ("usHeight", c_ushort),
        ("uiWidthStep", c_uint),
        ("ucDepth", c_ubyte),
        ("ucFormat", c_ubyte),
        ("ucChannels", c_ubyte),
        ("ucElemBytes", c_ubyte),
        ("ucFormatGet", c_ubyte),
        ("uiIndex", c_uint),
        ("uiImgSize", c_uint),
        ("uiRsdSize", c_uint),
        ("uiHstSize", c_uint),
        ("pBuffer", POINTER(c_ubyte)),
    ]


# ════════════════════════════════════════════════════════════════════
#  DLL discovery
# ════════════════════════════════════════════════════════════════════

def _find_tucam_dll() -> str | None:
    """Locate ``TUCam.dll``.

    Search order (the repo-vendored copy wins so the app keeps working even if
    the TUCam application is uninstalled):
      1. Repo-bundled ``DLLs/tucsen dlls/`` (checked in).
      2. TUCam / Tucsen / Mosaic installs under Program Files.
      3. System PATH.
    """
    import platform

    if platform.system() != "Windows":
        # Linux SDK ships libTUCam.so; keep the shape so tests can run anywhere.
        project_root = Path(__file__).resolve().parent.parent.parent
        cand = project_root / "DLLs" / "tucsen dlls" / "libTUCam.so"
        return str(cand) if cand.exists() else None

    name = "TUCam.dll"
    candidates: list[Path] = []

    # 1. Repo-vendored (mirrors the `DLLs/zyla dlls` convention).
    project_root = Path(__file__).resolve().parent.parent.parent
    candidates.append(project_root / "DLLs" / "tucsen dlls" / name)

    # 2. Vendor installs.
    prog_dirs = [
        os.environ.get("PROGRAMFILES", r"C:\Program Files"),
        os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)"),
    ]
    app_names = ["TUCam", "Tucsen", "TUCSEN", "Mosaic", "Xintu"]
    for prog in prog_dirs:
        if not prog:
            continue
        for app in app_names:
            base = Path(prog) / app
            candidates.append(base / name)
            try:
                if base.exists():
                    candidates.extend(base.rglob(name))
            except OSError:
                pass

    # 3. System PATH.
    for p in os.environ.get("PATH", "").split(os.pathsep):
        if p:
            candidates.append(Path(p) / name)

    for dll in candidates:
        try:
            if dll.is_file() and dll.stat().st_size > 100_000:
                logger.info("TUCam DLL found: %s (%s bytes)",
                            dll, f"{dll.stat().st_size:,}")
                return str(dll)
        except OSError:
            continue
    return None


def _sdk_config_dir() -> str:
    """Writable directory for the SDK's own per-camera parameter files.

    ``TUCAM_Api_Init`` takes a config path and the SDK **writes into it** — one
    XML per physical camera, named with its serial (e.g.
    ``Libra 25_PIDe437_SBLL17725002.xml``) holding that unit's parameter set.

    So this must NOT be the DLL directory: that is `DLLs/tucsen dlls/`, which is
    tracked in git, and pointing the SDK there drops per-unit machine-local files
    into the repo where they show up as untracked noise and can be committed by
    accident. Use the repo's per-machine config area instead, matching where
    every other hardware store lives (`config/hardware/...`).

    Falls back to the DLL directory only if that cannot be created, since the SDK
    needs *some* writable path.
    """
    root = Path(__file__).resolve().parent.parent.parent
    d = root / "config" / "hardware" / "tucsen"
    try:
        d.mkdir(parents=True, exist_ok=True)
        return str(d)
    except OSError as exc:
        logger.debug("TUCam: cannot create %s (%s); falling back to the DLL dir",
                     d, exc)
        return str(Path(_find_tucam_dll() or ".").parent)


_lib = None
_lib_error: str | None = None
_lib_lock = threading.Lock()

# Cached OS device-name lookup (see _os_tucsen_model_names).
_os_model_cache: "list[str] | None" = None


def _load_library():
    """Load TUCam.dll (idempotent). Returns the CDLL or None."""
    global _lib, _lib_error
    with _lib_lock:
        if _lib is not None:
            return _lib
        if _lib_error is not None:
            return None

        dll_path = _find_tucam_dll()
        if dll_path is None:
            _lib_error = "TUCam.dll not found"
            logger.info("TUCam: %s", _lib_error)
            return None

        dll_dir = str(Path(dll_path).parent)
        # The SDK's companion DLLs (tuimgcv_*, msvc*120) sit beside it.
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(dll_dir)
            except OSError:
                pass
        old_cwd = None
        try:
            # Some builds resolve siblings relative to the process CWD.
            old_cwd = os.getcwd()
            os.chdir(dll_dir)
        except OSError:
            old_cwd = None

        try:
            lib = ctypes.CDLL(dll_path)
            lib.TUCAM_Api_Init  # sanity: is this really the TUCam SDK?
        except (OSError, AttributeError) as exc:
            _lib_error = f"cannot load {dll_path}: {exc}"
            logger.warning("TUCam: %s", _lib_error)
            return None
        finally:
            if old_cwd:
                try:
                    os.chdir(old_cwd)
                except OSError:
                    pass

        lib.TUCAM_Api_Init.restype = c_int
        lib.TUCAM_Api_Uninit.restype = c_int
        lib.TUCAM_Dev_Open.restype = c_int
        lib.TUCAM_Dev_Close.restype = c_int
        _lib = lib
        logger.info("TUCam library loaded: %s", dll_path)
        return lib


# ════════════════════════════════════════════════════════════════════
#  API init refcount
# ════════════════════════════════════════════════════════════════════
# TUCAM_Api_Init is process-global: enumerate() and every open share it, so it is
# refcounted. Uninit'ing while another camera is streaming would kill that
# stream — the same non-refcounted-global hazard this repo hit with the position
# poller's suspend flag.

_api_lock = threading.Lock()
_api_refs = 0
_api_cam_count = 0


def _api_init(config_path: str | None = None) -> int:
    """Init the API (refcounted) and return the reported camera count."""
    global _api_refs, _api_cam_count
    lib = _load_library()
    if lib is None:
        return 0
    with _api_lock:
        if _api_refs > 0:
            _api_refs += 1
            return _api_cam_count
        path = config_path or _sdk_config_dir()
        init = TUCAM_INIT(0, path.encode("utf-8"))
        ret = None
        for args in ((byref(init), c_int(5000)), (byref(init),)):
            try:
                ret = lib.TUCAM_Api_Init(*args)
            except Exception as exc:      # pragma: no cover - defensive
                logger.debug("TUCAM_Api_Init raised: %s", exc)
                continue
            break
        if ret is None:
            return 0
        if ret != TUCAMRET_SUCCESS and init.uiCamCount == 0:
            logger.info("TUCam: Api_Init -> 0x%08X, 0 cameras",
                        ret & 0xFFFFFFFF)
        _api_cam_count = int(init.uiCamCount)
        _api_refs = 1
        logger.info("TUCam Api_Init ok (0x%08X) — %d camera(s)",
                    ret & 0xFFFFFFFF, _api_cam_count)
        return _api_cam_count


def _api_release():
    """Drop one API reference; Uninit on the last one."""
    global _api_refs, _api_cam_count
    lib = _load_library()
    with _api_lock:
        if _api_refs <= 0:
            return
        _api_refs -= 1
        if _api_refs > 0 or lib is None:
            return
        _api_cam_count = 0
        try:
            lib.TUCAM_Api_Uninit()
        except Exception:      # pragma: no cover - defensive
            pass


# ════════════════════════════════════════════════════════════════════
#  Availability flag (lazy — mirrors TOUPCAM_AVAILABLE / ANDOR_AVAILABLE)
# ════════════════════════════════════════════════════════════════════

class _LazyAvailable:
    _value = None

    def __bool__(self):
        if self._value is None:
            self._value = bool(_NP_AVAILABLE) and _load_library() is not None
        return self._value

    def __repr__(self):
        return str(bool(self))


TUCAM_AVAILABLE = _LazyAvailable()


def _dims_from_text(text: str) -> tuple[int, int] | None:
    """Parse ``"2048x2048"`` / ``"2048*2048"`` / ``"2048 X 2048"`` → (w, h)."""
    if not text:
        return None
    m = re.search(r"(\d{2,6})\s*[xX*×]\s*(\d{2,6})", text)
    if not m:
        return None
    w, h = int(m.group(1)), int(m.group(2))
    if w <= 0 or h <= 0:
        return None
    return w, h


# ════════════════════════════════════════════════════════════════════
#  TUCamBackend
# ════════════════════════════════════════════════════════════════════

class TUCamBackend:
    """Tucsen camera with an OpenCV ``VideoCapture``-like API.

    The SDK's ``TUCAM_Buf_WaitForFrame`` blocks, so acquisition runs on a
    dedicated reader thread that publishes the latest converted BGR8 frame under
    a lock; ``read()`` is non-blocking after the first frame. This mirrors the
    Andor backend and keeps the Qt event loop free.
    """

    # Software display-scaling range for a mono sensor (raw counts).
    HW_RANGES = {
        "display_level": (0, LEVEL_MAX, 0),
    }

    # ── Enumeration ───────────────────────────────────────────────
    @staticmethod
    def enumerate() -> list[dict]:
        """Enumerate attached Tucsen cameras.

        Returns ``[{"id", "displayname", "resolutions", "preview_count"}]``.
        ``id`` is the device index as a string (the SDK opens by index).
        Returns ``[]`` when the SDK is absent or no camera is attached.
        """
        if not _NP_AVAILABLE:
            return []
        lib = _load_library()
        if lib is None:
            return []
        count = _api_init()
        try:
            devices = []
            for idx in range(count):
                name = _read_model(lib, idx) or f"Tucsen #{idx}"
                devices.append({
                    "id": str(idx),
                    "displayname": name,
                    # Resolutions need an open handle; filled in on open().
                    "resolutions": [],
                    "preview_count": 0,
                })
                logger.info("TUCam found: %s (index %d)", name, idx)
            return devices
        finally:
            _api_release()

    def __init__(self):
        self._lib = None
        self._handle = None
        self._opened_api = False
        self._device_id = ""
        self._model = ""
        self._w = 0
        self._h = 0
        self._frame_desc: TUCAM_FRAME | None = None
        self._buf_allocated = False
        self._capturing = False
        self._running = False
        self._reader: threading.Thread | None = None
        self._lock = threading.RLock()
        self._frame_ready = threading.Event()
        self._frame = None
        # Resolution capability, discovered on open.
        self._res_capa_id: int | None = None
        self._resolutions: list[tuple[int, int]] = []
        self._res_index = 0
        # mono16 → 8-bit display scaling (shared math with the Andor backend).
        self._display_auto_scale = True
        self._display_lo = 0
        self._display_hi = LEVEL_MAX
        self._last_auto_levels: tuple[float, float] | None = None
        self._last_channels = 0
        self._last_elem_bytes = 0

    # ── Open / close ──────────────────────────────────────────────
    def open(self, device_id: str, resolution_index: int | None = None) -> bool:
        """Open a Tucsen camera by device index (as returned by enumerate())."""
        if self._handle is not None:
            self.release()
        if not _NP_AVAILABLE:
            return False
        lib = _load_library()
        if lib is None:
            return False
        self._lib = lib

        try:
            idx = int(str(device_id).strip())
        except (TypeError, ValueError):
            logger.warning("TUCam: bad device id %r", device_id)
            return False

        count = _api_init()
        self._opened_api = True
        if count <= idx:
            logger.warning("TUCam: index %d out of range (%d camera(s))",
                           idx, count)
            self._cleanup_api()
            return False

        op = TUCAM_OPEN(idx, None)
        try:
            ret = lib.TUCAM_Dev_Open(byref(op))
        except Exception as exc:
            logger.warning("TUCam: Dev_Open raised: %s", exc)
            self._cleanup_api()
            return False
        if not op.hIdxTUCam:
            logger.warning("TUCam: Dev_Open(%d) failed (0x%08X)",
                           idx, (ret or 0) & 0xFFFFFFFF)
            self._cleanup_api()
            return False

        self._handle = c_void_p(op.hIdxTUCam)
        self._device_id = str(idx)
        self._model = (_read_model(lib, idx, handle=self._handle)
                       or f"Tucsen #{idx}")

        # Discover what this camera can actually do (never assumed).
        self._res_capa_id, self._resolutions = self._discover_resolutions()

        target = (resolution_index if resolution_index is not None
                  else self._auto_pick_resolution_index())
        if self._resolutions:
            self._apply_resolution_index(target)

        # A middling exposure so the operator sees *something* on first light.
        if self.get_exposure_time() in (None, 0):
            self._prop_set(TUIDP_EXPOSURETM, _DEFAULT_EXPOSURE_MS)

        if not self._start_stream():
            self.release()
            return False

        logger.info("TUCam opened: %s (%dx%d)", self._model, self._w, self._h)
        return True

    def _cleanup_api(self):
        if self._opened_api:
            _api_release()
            self._opened_api = False

    def _start_stream(self) -> bool:
        """Buf_Alloc + Cap_Start + spawn the reader thread."""
        lib, handle = self._lib, self._handle
        if lib is None or handle is None:
            return False

        frame = TUCAM_FRAME()
        frame.pBuffer = None
        # Ask for the sensor's "usual" format; we decode from the header the SDK
        # fills in, so this choice cannot corrupt the interpretation.
        frame.ucFormatGet = TUFRM_FMT_USUAL
        frame.uiRsdSize = 1
        try:
            ret = lib.TUCAM_Buf_Alloc(handle, byref(frame))
        except Exception as exc:
            logger.warning("TUCam: Buf_Alloc raised: %s", exc)
            return False
        if ret != TUCAMRET_SUCCESS:
            logger.warning("TUCam: Buf_Alloc failed (0x%08X)", ret & 0xFFFFFFFF)
            return False
        self._buf_allocated = True
        self._frame_desc = frame

        # Sequence capture (mode 0 = TUCCM_SEQUENCE) — continuous live preview.
        try:
            ret = lib.TUCAM_Cap_Start(handle, c_uint(0))
        except Exception as exc:
            logger.warning("TUCam: Cap_Start raised: %s", exc)
            return False
        if ret != TUCAMRET_SUCCESS:
            logger.warning("TUCam: Cap_Start failed (0x%08X)",
                           ret & 0xFFFFFFFF)
            return False
        self._capturing = True

        if frame.usWidth and frame.usHeight:
            self._w, self._h = int(frame.usWidth), int(frame.usHeight)

        self._running = True
        self._reader = threading.Thread(
            target=self._reader_loop, daemon=True, name="TUCamReader")
        self._reader.start()
        return True

    def _stop_stream(self):
        """Tear the stream down in the SDK's required order.

        AbortWait first so a reader blocked in WaitForFrame returns, then join it
        BEFORE Cap_Stop / Buf_Release — releasing the buffer under a live reader
        would hand the SDK a freed pointer.
        """
        lib, handle = self._lib, self._handle
        self._running = False
        if lib is not None and handle is not None and self._capturing:
            try:
                lib.TUCAM_Buf_AbortWait(handle)
            except Exception:
                pass
        reader = self._reader
        self._reader = None
        if (reader is not None and reader.is_alive()
                and reader is not threading.current_thread()):
            try:
                reader.join(timeout=2.0)
            except Exception:
                pass
        if lib is None or handle is None:
            return
        if self._capturing:
            try:
                lib.TUCAM_Cap_Stop(handle)
            except Exception:
                pass
            self._capturing = False
        if self._buf_allocated:
            try:
                lib.TUCAM_Buf_Release(handle)
            except Exception:
                pass
            self._buf_allocated = False

    def release(self):
        """Stop acquisition, close the device, drop the API reference."""
        self._stop_stream()
        with self._lock:
            handle = self._handle
            self._handle = None
            self._frame = None
            self._frame_desc = None
        if handle is not None and self._lib is not None:
            try:
                self._lib.TUCAM_Dev_Close(handle)
            except Exception:
                pass
        self._cleanup_api()
        self._frame_ready.clear()
        # Skip logging during interpreter teardown (release() may come from
        # __del__, when the logging machinery is half torn down).
        if sys.meta_path is not None:
            logger.info("TUCam released")

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass

    # ── Frame acquisition ─────────────────────────────────────────
    def _reader_loop(self):
        """Block on WaitForFrame, convert to BGR8, publish the latest frame."""
        lib, handle = self._lib, self._handle
        frame = self._frame_desc
        if lib is None or handle is None or frame is None:
            return
        fails = 0
        while self._running:
            try:
                ret = lib.TUCAM_Buf_WaitForFrame(
                    handle, byref(frame), c_int(_WAIT_FRAME_TIMEOUT_MS))
            except Exception as exc:
                logger.debug("TUCam WaitForFrame raised: %s", exc)
                ret = -1
            if not self._running:
                break
            if ret != TUCAMRET_SUCCESS:
                fails += 1
                # A timeout is normal at long exposures; only complain when the
                # stream looks genuinely dead.
                if fails in (10, 100):
                    logger.warning(
                        "TUCam: %d consecutive WaitForFrame failures "
                        "(last 0x%08X)", fails, ret & 0xFFFFFFFF)
                time.sleep(0.01)
                continue
            fails = 0
            bgr = self._decode_frame(frame)
            if bgr is None:
                continue
            with self._lock:
                self._frame = bgr
            self._frame_ready.set()

    def _decode_frame(self, frame: TUCAM_FRAME):
        """Copy + interpret the SDK buffer using the frame's OWN header fields.

        Nothing about the pixel layout is assumed: channel count, bytes per
        element and row pitch all come from the descriptor the SDK just filled
        in, so mono-16, RGB-24 and BGRA-32 sensors all decode correctly.
        """
        if not _NP_AVAILABLE:
            return None
        try:
            w = int(frame.usWidth)
            h = int(frame.usHeight)
            channels = int(frame.ucChannels) or 1
            elem = int(frame.ucElemBytes) or 1
            size = int(frame.uiImgSize)
            if w <= 0 or h <= 0 or size <= 0 or not frame.pBuffer:
                return None

            # Image data begins after the in-band header.
            base = ctypes.cast(frame.pBuffer, c_void_p).value
            if base is None:
                return None
            src = c_void_p(base + int(frame.usHeader))
            buf = create_string_buffer(size)
            ctypes.memmove(buf, src, size)
        except Exception as exc:
            logger.debug("TUCam frame copy failed: %s", exc)
            return None

        self._w, self._h = w, h
        self._last_channels, self._last_elem_bytes = channels, elem

        dtype = np.uint16 if elem >= 2 else np.uint8
        try:
            flat = np.frombuffer(buf, dtype=dtype, count=size // elem)
        except Exception as exc:
            logger.debug("TUCam frombuffer failed: %s", exc)
            return None

        # Honour the row pitch when the SDK pads rows.
        step_elems = int(frame.uiWidthStep) // elem if frame.uiWidthStep else 0
        row_elems = w * channels
        try:
            if step_elems and step_elems >= row_elems and flat.size >= step_elems * h:
                arr = flat[:step_elems * h].reshape(h, step_elems)
                arr = arr[:, :row_elems]
            else:
                arr = flat[:row_elems * h].reshape(h, row_elems)
            if channels > 1:
                arr = arr.reshape(h, w, channels)
        except Exception as exc:
            logger.debug("TUCam reshape failed (w=%d h=%d c=%d e=%d): %s",
                         w, h, channels, elem, exc)
            return None

        if channels == 1:
            # Mono sensor → shared mono→BGR8 display conversion. Record the
            # auto levels so switching auto OFF freezes the current look
            # (the Andor backend's behaviour, kept identical here).
            levels = None
            if self._display_auto_scale:
                if arr.dtype != np.uint8:
                    try:
                        self._last_auto_levels = _auto_levels(arr)
                    except Exception:
                        pass
            else:
                levels = (self._display_lo, self._display_hi)
            return _mono_to_bgr8(arr, levels=levels)

        if channels >= 4:
            arr = arr[:, :, :3]        # drop alpha
        if arr.dtype != np.uint8:
            # >8-bit colour → shift down by the sensor's declared bit depth so
            # the mapping is exact rather than a guessed divisor.
            depth = int(frame.ucDepth or 16)
            shift = depth - 8 if depth > 8 else 0
            arr = np.right_shift(arr, shift) if shift else arr
            arr = np.clip(arr, 0, 255).astype(np.uint8)
        return np.ascontiguousarray(arr)

    def isOpened(self) -> bool:
        return self._handle is not None and bool(self._handle) and self._running

    def read(self) -> "tuple[bool, np.ndarray | None]":
        """Return ``(ok, latest BGR8 frame)``. Non-blocking after first frame."""
        if not self.isOpened():
            return False, None
        if self._frame is None:
            self._frame_ready.wait(timeout=1.5)
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
        return False, None

    # ── Resolution ────────────────────────────────────────────────
    def _discover_resolutions(self) -> "tuple[int | None, list[tuple[int, int]]]":
        """Find the resolution capability EMPIRICALLY.

        Sweeps capability IDs; for each, reads every selectable value's text via
        the SDK's own ``TUCAM_Capa_GetValueText`` and keeps the ID whose labels
        parse as ``WxH``. That makes the resolution list self-verifying instead
        of resting on a guessed enum value. Returns ``(capa_id, [(w, h), ...])``
        — ``(None, [])`` when nothing qualifies, in which case the single
        frame-reported size is used.
        """
        for capa in range(_CAPA_SWEEP_MAX):
            attr = self._capa_attr(capa)
            if attr is None:
                continue
            lo, hi = int(attr.nValMin), int(attr.nValMax)
            if hi <= lo or (hi - lo) > 32:
                continue
            dims: list[tuple[int, int]] = []
            for v in range(lo, hi + 1):
                text = self._capa_value_text(capa, v)
                d = _dims_from_text(text or "")
                if d is None:
                    dims = []
                    break
                dims.append(d)
            if dims:
                logger.info("TUCam: resolution capability id=%d -> %s",
                            capa, dims)
                return capa, dims
        logger.info("TUCam: no resolution capability found; using the "
                    "frame-reported size only")
        return None, []

    def _auto_pick_resolution_index(self) -> int:
        """Largest resolution at or under the preview width cap."""
        best, best_w = 0, -1
        for i, (w, _h) in enumerate(self._resolutions):
            if w <= _PREVIEW_MAX_WIDTH and w > best_w:
                best, best_w = i, w
        if best_w < 0:
            # Everything is bigger than the cap — take the smallest.
            if self._resolutions:
                best = min(range(len(self._resolutions)),
                           key=lambda i: self._resolutions[i][0])
        return best

    def _apply_resolution_index(self, index: int) -> bool:
        if self._res_capa_id is None or not self._resolutions:
            return False
        index = max(0, min(int(index), len(self._resolutions) - 1))
        if not self._capa_set(self._res_capa_id, index):
            return False
        self._res_index = index
        self._w, self._h = self._resolutions[index]
        return True

    def get_resolution(self) -> tuple[int, int]:
        return (self._w, self._h)

    def get_resolution_list(self) -> list:
        if self._resolutions:
            return list(self._resolutions)
        return [(self._w, self._h)] if self._w and self._h else []

    def get_eSize(self):
        """Current resolution index (naming matches the ToupCam contract)."""
        return self._res_index if self._resolutions else None

    def set_resolution_index(self, index: int) -> bool:
        """Switch resolution, restarting the stream around the change.

        The buffer is sized for the old geometry, so the stream must come down
        before the resolution changes and back up afterwards.
        """
        if self._handle is None or self._res_capa_id is None:
            return False
        self._stop_stream()
        ok = self._apply_resolution_index(index)
        with self._lock:
            self._frame = None
        self._frame_ready.clear()
        if not self._start_stream():
            logger.warning("TUCam: stream restart failed after resolution change")
            return False
        logger.info("TUCam resolution changed: %dx%d", self._w, self._h)
        return ok

    # ── Property / capability plumbing ────────────────────────────
    def _prop_attr(self, prop_id: int, channel: int = 0):
        """``TUCAM_Prop_GetAttr`` → attr struct, or None when unsupported.

        This is the gate that makes an unverified property ID safe: an ID the
        camera does not implement never reaches a get/set.
        """
        lib, handle = self._lib, self._handle
        if lib is None or handle is None or not bool(handle):
            return None
        attr = TUCAM_PROP_ATTR()
        attr.idProp = int(prop_id)
        attr.nIdxChn = int(channel)
        try:
            ret = lib.TUCAM_Prop_GetAttr(handle, byref(attr))
        except Exception:
            return None
        return attr if ret == TUCAMRET_SUCCESS else None

    def _prop_get(self, prop_id: int, channel: int = 0):
        if self._prop_attr(prop_id, channel) is None:
            return None
        lib, handle = self._lib, self._handle
        val = c_double(0.0)
        try:
            ret = lib.TUCAM_Prop_GetValue(
                handle, c_int(int(prop_id)), byref(val), c_int(int(channel)))
        except Exception:
            return None
        return float(val.value) if ret == TUCAMRET_SUCCESS else None

    def _prop_set(self, prop_id: int, value, channel: int = 0) -> bool:
        attr = self._prop_attr(prop_id, channel)
        if attr is None:
            return False
        lib, handle = self._lib, self._handle
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        # Clamp into the SDK's own declared range — a camera property is not a
        # discrete selector, so clamping is the right call here (contrast with
        # the microscope turret, which must REFUSE an out-of-range slot).
        lo, hi = float(attr.dbValMin), float(attr.dbValMax)
        if hi > lo:
            v = max(lo, min(hi, v))
        try:
            ret = lib.TUCAM_Prop_SetValue(
                handle, c_int(int(prop_id)), c_double(v), c_int(int(channel)))
        except Exception:
            return False
        return ret == TUCAMRET_SUCCESS

    def _capa_attr(self, capa_id: int):
        lib, handle = self._lib, self._handle
        if lib is None or handle is None or not bool(handle):
            return None
        attr = TUCAM_CAPA_ATTR()
        attr.idCapa = int(capa_id)
        try:
            ret = lib.TUCAM_Capa_GetAttr(handle, byref(attr))
        except Exception:
            return None
        return attr if ret == TUCAMRET_SUCCESS else None

    def _capa_get(self, capa_id: int):
        if self._capa_attr(capa_id) is None:
            return None
        lib, handle = self._lib, self._handle
        val = c_int(0)
        try:
            ret = lib.TUCAM_Capa_GetValue(
                handle, c_int(int(capa_id)), byref(val))
        except Exception:
            return None
        return int(val.value) if ret == TUCAMRET_SUCCESS else None

    def _capa_set(self, capa_id: int, value: int) -> bool:
        """Set a capability, SKIPPING a write that would not change anything.

        🐞 HARDWARE FINDING (Libra 25, 2026-08-04): writing a capability the
        value it ALREADY holds is not a harmless no-op on this camera. Writing
        ``auto_exposure = 0`` while it was already 0 **reset the exposure to the
        sensor minimum (6.3 µs)** — i.e. the live image goes black. A genuine
        transition (1 -> 0) preserves exposure correctly; only the redundant
        write is destructive.

        That is reachable on a completely ordinary path:
        ``hardware_setup._apply_hw_controls`` restores ``auto_exposure`` from the
        persisted ``hw_controls`` every time a camera starts, and that stored
        value was itself read back FROM the camera — so it normally equals the
        current value, and the restore would have blacked out the preview on
        every startup.

        Skipping a no-change write is semantically correct regardless (setting a
        value to what it already is means nothing), so this guards every
        capability rather than special-casing auto-exposure.
        """
        if self._capa_attr(capa_id) is None:
            return False
        try:
            want = int(value)
        except (TypeError, ValueError):
            return False
        current = self._capa_get(capa_id)
        if current is not None and int(current) == want:
            return True
        lib, handle = self._lib, self._handle
        try:
            ret = lib.TUCAM_Capa_SetValue(
                handle, c_int(int(capa_id)), c_int(want))
        except Exception:
            return False
        return ret == TUCAMRET_SUCCESS

    def _capa_value_text(self, capa_id: int, value: int) -> str | None:
        lib, handle = self._lib, self._handle
        if lib is None or handle is None:
            return None
        buf = create_string_buffer(256)
        vt = TUCAM_VALUE_TEXT()
        vt.nID = int(capa_id)
        vt.dbValue = float(value)
        vt.pText = ctypes.cast(buf, POINTER(c_char))
        vt.nTextSize = 256
        try:
            ret = lib.TUCAM_Capa_GetValueText(handle, byref(vt))
        except Exception:
            return None
        if ret != TUCAMRET_SUCCESS:
            return None
        try:
            return buf.value.decode("utf-8", errors="replace")
        except Exception:
            return None

    # ── Exposure (µs contract ↔ ms SDK) ───────────────────────────
    @staticmethod
    def _exposure_scale_us() -> float:
        """Microseconds per TUCam exposure unit.

        TUCam's exposure property is **milliseconds**; every other camera in this
        app reports microseconds. One place does the conversion so the two frames
        cannot drift — the class of bug that made the Nikon focus drive read 40×
        small until the display scale was resolved from the SDK itself.
        """
        return 1000.0

    def get_exposure_time(self):
        """Exposure in MICROSECONDS (app contract), or None."""
        ms = self._prop_get(TUIDP_EXPOSURETM)
        return None if ms is None else ms * self._exposure_scale_us()

    def put_exposure_time(self, microseconds) -> bool:
        try:
            us = float(microseconds)
        except (TypeError, ValueError):
            return False
        return self._prop_set(TUIDP_EXPOSURETM, us / self._exposure_scale_us())

    def get_exposure_time_range(self):
        """(min_us, max_us, default_us) from the SDK's OWN declared range.

        Reported in µs by converting the SDK's ms range, so a wrong unit
        assumption surfaces as an implausible range instead of a silently wrong
        exposure.
        """
        attr = self._prop_attr(TUIDP_EXPOSURETM)
        if attr is None:
            return None
        k = self._exposure_scale_us()
        return (attr.dbValMin * k, attr.dbValMax * k, attr.dbValDft * k)

    # ── Gain / ISP-style controls ─────────────────────────────────
    def get_exposure_gain(self):
        return self._prop_get(TUIDP_GLOBALGAIN)

    def put_exposure_gain(self, percent) -> bool:
        return self._prop_set(TUIDP_GLOBALGAIN, percent)

    def get_exposure_gain_range(self):
        attr = self._prop_attr(TUIDP_GLOBALGAIN)
        if attr is None:
            return None
        return (attr.dbValMin, attr.dbValMax, attr.dbValDft)

    def get_brightness(self):
        return self._prop_get(TUIDP_BRIGHTNESS)

    def put_brightness(self, v) -> bool:
        return self._prop_set(TUIDP_BRIGHTNESS, v)

    def get_contrast(self):
        return self._prop_get(TUIDP_CONTRAST)

    def put_contrast(self, v) -> bool:
        return self._prop_set(TUIDP_CONTRAST, v)

    def get_gamma(self):
        return self._prop_get(TUIDP_GAMMA)

    def put_gamma(self, v) -> bool:
        return self._prop_set(TUIDP_GAMMA, v)

    def get_temperature(self):
        """Sensor temperature, or None when the camera's answer is unusable.

        On the Libra 25 this property declares a 500..1000 range but READS
        0.375 -- i.e. outside its own declared range, so whatever it means it is
        not a temperature in degrees. Rather than surface a misleading number,
        report None unless the value sits inside the range the SDK itself
        declared. (Un-cooled models simply have nothing to report.)
        """
        attr = self._prop_attr(TUIDP_TEMPERATURE)
        if attr is None:
            return None
        val = self._prop_get(TUIDP_TEMPERATURE)
        if val is None:
            return None
        if attr.dbValMax > attr.dbValMin and not (
                attr.dbValMin <= val <= attr.dbValMax):
            logger.debug(
                "TUCam: temperature %.3f outside declared %.3f..%.3f -> "
                "reporting unknown", val, attr.dbValMin, attr.dbValMax)
            return None
        return val

    def prop_range(self, prop_id: int):
        """(min, max, default) for a property, or None when unsupported."""
        attr = self._prop_attr(prop_id)
        if attr is None:
            return None
        return (attr.dbValMin, attr.dbValMax, attr.dbValDft)

    # Named range accessors so callers never need the raw TUIDP_* constants.
    # Each returns None when this model does not implement the property, which
    # is what lets the settings UI hide a control rather than show a dead one.
    def get_gamma_range(self):
        return self.prop_range(TUIDP_GAMMA)

    def get_brightness_range(self):
        return self.prop_range(TUIDP_BRIGHTNESS)

    def get_contrast_range(self):
        return self.prop_range(TUIDP_CONTRAST)

    def get_auto_exposure(self):
        v = self._capa_get(TUIDC_ATEXPOSURE)
        return None if v is None else bool(v)

    def set_auto_exposure(self, enabled: bool) -> bool:
        return self._capa_set(TUIDC_ATEXPOSURE, 1 if enabled else 0)

    # ── mono16 → 8-bit display scaling (shared with the Andor path) ──
    def get_display_auto_scale(self) -> bool:
        return bool(self._display_auto_scale)

    def set_display_auto_scale(self, enabled: bool) -> bool:
        """Toggle the per-frame percentile auto-scale.

        Turning it OFF freezes the last auto levels, so the image keeps the look
        it currently has instead of jumping — identical to the Andor behaviour.
        """
        enabled = bool(enabled)
        if not enabled and self._last_auto_levels:
            lo, hi = self._last_auto_levels
            self._display_lo = int(max(0, min(LEVEL_MAX, round(lo))))
            self._display_hi = int(max(0, min(LEVEL_MAX, round(hi))))
            if self._display_hi <= self._display_lo:
                self._display_hi = min(LEVEL_MAX, self._display_lo + 1)
        self._display_auto_scale = enabled
        return True

    def get_display_levels(self) -> tuple:
        return (self._display_lo, self._display_hi)

    def put_display_black(self, counts) -> bool:
        try:
            v = int(counts)
        except (TypeError, ValueError):
            return False
        v = max(0, min(LEVEL_MAX, v))
        if v >= self._display_hi:
            v = max(0, self._display_hi - 1)
        self._display_lo = v
        return True

    def put_display_white(self, counts) -> bool:
        try:
            v = int(counts)
        except (TypeError, ValueError):
            return False
        v = max(0, min(LEVEL_MAX, v))
        if v <= self._display_lo:
            v = min(LEVEL_MAX, self._display_lo + 1)
        self._display_hi = v
        return True

    def get_display_level_range(self) -> tuple:
        return (0, LEVEL_MAX, 0)

    # ── Readback ──────────────────────────────────────────────────
    def get_settings(self) -> dict:
        """Read every setting back FROM the device (for the settings readout).

        Values come from SDK getters against the open handle — never cached or
        echoed — so the caller can prove they came from the camera.
        """
        lo, hi = self.get_display_levels()
        return {
            "device_id": self._device_id,
            "model": self._model,
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
            "temperature_c": self.get_temperature(),
            # Populated by the frame decode, so report None (unknown) rather
            # than 0 before the first frame has arrived — "0 ch, 0 byte/px"
            # reads like a real answer in the settings readout.
            "channels": self._last_channels or None,
            "elem_bytes": self._last_elem_bytes or None,
            # Mono→8-bit display scaling. Canonically `mono_*`; the `andor_*`
            # aliases are the HISTORICAL names of this same channel (it shipped
            # first on the Zyla) and are what the settings dialog gates on and
            # what `hw_controls` persists. Emitting both lets the Tucsen camera
            # reuse that proven plumbing verbatim while the readout still carries
            # honestly-named keys. Retiring the aliases is a follow-up, not
            # something worth churning a hardware-verified suite for.
            "mono_auto_scale": self.get_display_auto_scale(),
            "mono_scale_lo": lo,
            "mono_scale_hi": hi,
            "andor_auto_scale": self.get_display_auto_scale(),
            "andor_scale_lo": lo,
            "andor_scale_hi": hi,
        }

    # ── Diagnostics ───────────────────────────────────────────────
    def diagnostics(self) -> str:
        """Dump every property/capability this camera ACTUALLY implements.

        Run this on real hardware to confirm (or correct) the ``TUIDP_*`` /
        ``TUIDC_*`` guesses at the top of this module in one bench session —
        the same role ``MicroscopeControl.diagnostics()`` plays for the Nikon Ti.
        """
        lines = [
            "TUCam diagnostics",
            "=" * 60,
            f"model:      {self._model}",
            f"device id:  {self._device_id}",
            f"resolution: {self._w}x{self._h}  (index {self.get_eSize()})",
            f"res list:   {self.get_resolution_list()}",
            f"res capa:   {self._res_capa_id}",
            (f"frame:      channels={self._last_channels} "
             f"elem_bytes={self._last_elem_bytes}"
             if self._last_channels else
             "frame:      (no frame decoded yet — read() at least once)"),
            "",
            "PROPERTIES (id: min .. max, default = current)",
        ]
        # Names as confirmed on a Libra 25. A DIFFERENT Tucsen model may lay the
        # enum out differently, so treat these as the expected mapping to CHECK
        # against, not as ground truth — that is the whole point of this dump.
        # See docs/TUCSEN_CAMERA_INSTALL.md step 5.
        named = {
            TUIDP_GLOBALGAIN: "GLOBALGAIN (gain mode)",
            TUIDP_EXPOSURETM: "EXPOSURETM  <-- exposure, ms",
            TUIDP_BRIGHTNESS: "BRIGHTNESS",
            TUIDP_BLACKLEVEL: "BLACKLEVEL",
            TUIDP_TEMPERATURE: "TEMPERATURE",
            TUIDP_SHARPNESS: "SHARPNESS",
            TUIDP_NOISELEVEL: "NOISELEVEL",
            TUIDP_HDR_KVALUE: "HDR_KVALUE",
            TUIDP_GAMMA: "GAMMA",
            TUIDP_CONTRAST: "CONTRAST",
            TUIDP_LFTLEVELS: "LFTLEVELS",
            TUIDP_RGTLEVELS: "RGTLEVELS",
        }
        found = 0
        for pid in range(_PROP_SWEEP_MAX):
            attr = self._prop_attr(pid)
            if attr is None:
                continue
            found += 1
            cur = self._prop_get(pid)
            tag = named.get(pid, "")
            lines.append(
                f"  {pid:>3}: {attr.dbValMin:g} .. {attr.dbValMax:g}, "
                f"dft={attr.dbValDft:g} step={attr.dbValStep:g} "
                f"= {cur!r}  {tag}")
        if not found:
            lines.append("  (none — is the camera open?)")

        lines += ["", "CAPABILITIES (id: min .. max, default = current)"]
        capa_named = {
            TUIDC_RESOLUTION: "RESOLUTION",
            TUIDC_PIXELCLOCK: "PIXELCLOCK",
            TUIDC_BITOFDEPTH: "BITOFDEPTH",
            TUIDC_ATEXPOSURE: "ATEXPOSURE  <-- auto-exposure",
            TUIDC_HORIZONTAL: "HORIZONTAL (mirror)",
            TUIDC_VERTICAL: "VERTICAL (flip)",
            TUIDC_ATLEVELS: "ATLEVELS (auto levels, NOT auto-exposure)",
        }
        found = 0
        for cid in range(_CAPA_SWEEP_MAX):
            attr = self._capa_attr(cid)
            if attr is None:
                continue
            found += 1
            cur = self._capa_get(cid)
            texts = []
            if (attr.nValMax - attr.nValMin) <= 16:
                for v in range(int(attr.nValMin), int(attr.nValMax) + 1):
                    t = self._capa_value_text(cid, v)
                    if t:
                        texts.append(f"{v}={t}")
            tag = capa_named.get(cid, "")
            lines.append(
                f"  {cid:>3}: {attr.nValMin} .. {attr.nValMax}, "
                f"dft={attr.nValDft} = {cur!r}"
                + (f"  [{', '.join(texts)}]" if texts else "")
                + (f"  {tag}" if tag else ""))
        if not found:
            lines.append("  (none — is the camera open?)")
        return "\n".join(lines)


def _sdk_info_text(lib, handle, info_id: int) -> str | None:
    """One ``TUCAM_Dev_GetInfo`` text read, or None."""
    buf = create_string_buffer(256)
    vi = TUCAM_VALUE_INFO()
    vi.nID = int(info_id)
    vi.nValue = 0
    vi.pText = ctypes.cast(buf, POINTER(c_char))
    vi.nTextSize = 256
    try:
        ret = lib.TUCAM_Dev_GetInfo(handle, byref(vi))
    except Exception:
        return None
    if ret != TUCAMRET_SUCCESS:
        return None
    try:
        text = buf.value.decode("utf-8", errors="replace").strip()
    except Exception:
        return None
    return text if (text and any(ch.isalnum() for ch in text)) else None


def _clean_inf_string(value: str) -> str:
    """``'@oem81.inf,%vid_5453&pid_e437.devicedesc%;Libra 25'`` -> ``'Libra 25'``.

    Windows stores a driver-supplied device description as an INF string
    reference with the resolved display text after the final ``;``.
    """
    text = (value or "").strip()
    if ";" in text:
        text = text.rsplit(";", 1)[-1].strip()
    return text


def _os_tucsen_model_names() -> list[str]:
    """Model names of attached Tucsen devices, as WINDOWS reports them.

    Needed because this SDK build returns **no model text**: on a real Libra 25
    every ``TUCAM_Dev_GetInfo`` / ``GetInfoEx`` string ID came back empty (only
    numeric fields are populated), while Windows names the device exactly
    "Libra 25". `camera_identity` already reads OS names for the UVC cameras, so
    this follows existing practice.

    Read straight from the registry rather than by shelling out to PowerShell:
    camera detection runs on the GUI thread, and a subprocess there is the
    freeze class this repo has already fixed several times. The registry read
    measures ~0.1 ms against ~1 s for a PowerShell spawn. Cached per process
    since a camera's model cannot change without a re-plug.

    Best-effort and label-only — a camera's *identity* never depends on this.
    Returns [] off Windows or on any failure.
    """
    global _os_model_cache
    if _os_model_cache is not None:
        return _os_model_cache
    names: list[str] = []
    if sys.platform == "win32":
        try:
            import winreg
            vid = f"VID_{TUCSEN_USB_VID:04X}"
            base = r"SYSTEM\CurrentControlSet\Enum\USB"
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as root:
                for i in range(4096):
                    try:
                        dev = winreg.EnumKey(root, i)
                    except OSError:
                        break
                    if vid not in dev.upper():
                        continue
                    with winreg.OpenKey(root, dev) as devkey:
                        for j in range(64):
                            try:
                                inst = winreg.EnumKey(devkey, j)
                            except OSError:
                                break
                            with winreg.OpenKey(devkey, inst) as ik:
                                for field in ("FriendlyName", "DeviceDesc"):
                                    try:
                                        raw = winreg.QueryValueEx(ik, field)[0]
                                    except OSError:
                                        continue
                                    text = _clean_inf_string(str(raw))
                                    if text:
                                        names.append(text)
                                        break
        except Exception as exc:
            logger.debug("OS model-name lookup failed: %s", exc)
            names = []
    _os_model_cache = names
    return names


def _read_model(lib, index: int, handle=None) -> str | None:
    """Best-effort model name for camera ``index``.

    Order: the SDK's own text fields (works if a future model/SDK provides
    them), then the OS device name, then None. A wrong or missing answer costs a
    display label and nothing else — the device is always opened by index.
    """
    if handle is not None:
        for info_id in _INFO_MODEL_CANDIDATES:
            text = _sdk_info_text(lib, handle, info_id)
            if text:
                return text
    names = _os_tucsen_model_names()
    if 0 <= index < len(names):
        return names[index]
    return None
