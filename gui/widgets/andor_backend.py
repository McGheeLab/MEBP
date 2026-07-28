"""
Andor Backend — pylablib wrapper for ANDOR SDK3 scientific cameras.

v7.5.x: Self-contained ANDOR Zyla (SDK3) camera interface that provides the
same OpenCV-VideoCapture-like API the CameraWidget dual/triple-backend system
expects from ``ToupCamBackend`` (see gui/widgets/toupcam_backend.py). Targets
the ANDOR ZYLA-4.2P-USB3, a 2048x2048 mono 16-bit sCMOS camera.

Key differences from the ToupTek path:
  * Driven by the Andor SDK3 runtime (atcore.dll + companion DLLs) via the
    ``pylablib`` package (``Andor.AndorSDK3Camera``) — NOT UVC/DirectShow, NOT
    ToupTek. It never appears in the OpenCV or ToupCam enumerations.
  * The sensor is mono 16-bit; the rest of the app expects 8-bit BGR. This
    backend converts every frame to BGR8 inside ``read()`` (per-frame
    percentile auto-scale + GRAY2BGR), so the CameraWidget pipeline is unchanged.
  * The live feed defaults to a binned readout (2x2 -> 1024x1024) for a smooth
    USB3 preview; full 2048x2048 is selectable via the resolution list.

Everything is lazy + guarded: importing this module never requires pylablib or
loads the SDK DLLs. ``ANDOR_AVAILABLE`` only becomes truthy on first access
when numpy + pylablib import AND the atcore DLL directory is found. When the SDK
is absent the app simply behaves as if no Andor camera exists.

Usage::

    from gui.widgets.andor_backend import AndorBackend, ANDOR_AVAILABLE

    if ANDOR_AVAILABLE:
        devices = AndorBackend.enumerate()
        cam = AndorBackend()
        if cam.open(devices[0]['id']):
            ok, frame = cam.read()   # BGR uint8 numpy array
            cam.release()
"""

from __future__ import annotations

import os
import sys
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
#  Resolution / binning presets
# ════════════════════════════════════════════════════════════════════
# The Zyla exposes resolutions through on-sensor binning of the full
# 2048x2048 array. Each preset is (width, height, hbin, vbin). The live
# preview auto-picks a preset <= 1280 wide (1024x1024, 2x2) for a smooth
# USB3 stream; full-res 1x1 is available via the resolution combo.
_ZYLA_FULL = (2048, 2048)
RESOLUTION_PRESETS = [
    (1024, 1024, 2, 2),   # index 0 — default preview (auto-picked)
    (512, 512, 4, 4),     # index 1
    (256, 256, 8, 8),     # index 2
    (2048, 2048, 1, 1),   # index 3 — full resolution
]

# Default exposure applied on open (seconds) when the camera comes up with an
# unusable value. A middling exposure so the operator sees *something*.
_DEFAULT_EXPOSURE_S = 0.03


# ════════════════════════════════════════════════════════════════════
#  DLL Discovery
# ════════════════════════════════════════════════════════════════════

def _find_andor_dll_dir() -> str | None:
    """Locate the directory containing ``atcore.dll`` (Andor SDK3 runtime).

    Search order:
      1. Repo-bundled ``DLLs/zyla dlls/`` (operator-provided, checked in).
      2. Andor SDK3 developer install (Program Files\\Andor SDK3).
      3. Andor Solis install (bundles the SDK3 DLLs).
      4. System PATH.
    Returns the directory path string, or None if atcore.dll is not found.
    """
    import platform

    core = "atcore.dll" if platform.system() == "Windows" else "libatcore.so"

    candidates: list[Path] = []

    # 1. Repo-bundled DLLs (project_root/DLLs/zyla dlls). Note the space in the
    #    folder name — kept verbatim to match what the operator pasted in.
    project_root = Path(__file__).resolve().parent.parent.parent
    candidates.append(project_root / "DLLs" / "zyla dlls")

    # 2/3. Program Files installs
    if platform.system() == "Windows":
        prog_dirs = [
            os.environ.get("PROGRAMFILES", r"C:\Program Files"),
            os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)"),
        ]
        app_names = ["Andor SDK3", "Andor SOLIS", "Andor Solis", "Andor"]
        for prog in prog_dirs:
            if not prog:
                continue
            for app in app_names:
                candidates.append(Path(prog) / app)

    # 4. System PATH
    for p in os.environ.get("PATH", "").split(os.pathsep):
        if p:
            candidates.append(Path(p))

    for d in candidates:
        try:
            if (d / core).exists():
                logger.info(f"Andor SDK3 DLLs found: {d}")
                return str(d)
        except OSError:
            continue

    # Fall back to a recursive scan of the Program Files app dirs (SDK3 may
    # nest the DLLs a level deep).
    if platform.system() == "Windows":
        for prog in [os.environ.get("PROGRAMFILES", r"C:\Program Files"),
                     os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)")]:
            if not prog:
                continue
            for app in ["Andor SDK3", "Andor SOLIS", "Andor Solis"]:
                base = Path(prog) / app
                if not base.exists():
                    continue
                try:
                    for hit in base.rglob(core):
                        logger.info(f"Andor SDK3 DLLs found (nested): {hit.parent}")
                        return str(hit.parent)
                except OSError:
                    continue

    return None


# ════════════════════════════════════════════════════════════════════
#  pylablib loading (lazy, cached)
# ════════════════════════════════════════════════════════════════════

_andor_mod = None      # pylablib.devices.Andor module, once loaded
_andor_error = None    # cached failure reason


def _load_andor():
    """Import pylablib's Andor module with the SDK3 DLL path configured.

    Returns the ``pylablib.devices.Andor`` module, or None if unavailable.
    Result is cached (success or failure) so the import/DLL probe runs once.
    """
    global _andor_mod, _andor_error

    if _andor_mod is not None:
        return _andor_mod
    if _andor_error is not None:
        return None

    if not _NP_AVAILABLE:
        _andor_error = "numpy not available"
        return None

    dll_dir = _find_andor_dll_dir()
    if dll_dir is None:
        _andor_error = "atcore.dll (Andor SDK3) not found"
        logger.info(f"Andor: {_andor_error}")
        return None

    try:
        # Help Windows resolve the SDK3 companion DLLs.
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(dll_dir)
            except OSError:
                pass

        import pylablib as pll
        pll.par["devices/dlls/andor_sdk3"] = dll_dir
        from pylablib.devices import Andor  # noqa: N814

        _andor_mod = Andor
        logger.info(f"Andor (pylablib) loaded; SDK3 dlls at {dll_dir}")
        return Andor
    except Exception as exc:  # ImportError, OSError, pylablib errors
        _andor_error = f"pylablib/Andor import failed: {exc}"
        logger.info(f"Andor: {_andor_error}")
        return None


def _check_available() -> bool:
    """True if numpy + pylablib + SDK3 DLLs are all present."""
    if not _NP_AVAILABLE:
        return False
    return _load_andor() is not None


class _LazyAvailable:
    """Descriptor that resolves availability on first bool() access.

    Mirrors ToupCamBackend's pattern so merely importing this module never
    touches the SDK.
    """
    _value = None

    def __bool__(self):
        if self._value is None:
            self._value = _check_available()
        return self._value

    def __repr__(self):
        return str(bool(self))


ANDOR_AVAILABLE = _LazyAvailable()


# ════════════════════════════════════════════════════════════════════
#  AndorBackend — OpenCV/ToupCam-like Camera Interface
# ════════════════════════════════════════════════════════════════════

class AndorBackend:
    """ANDOR SDK3 camera (Zyla) with the ToupCam-compatible backend surface.

    Threading: pylablib runs its own SDK acquisition; a daemon reader thread
    pulls the newest mono-16 frame into a lock-protected buffer, and ``read()``
    returns the latest frame converted to BGR8. This keeps the GUI-thread grab
    timer non-blocking (mirrors ToupCamBackend's callback -> buffer -> read()).
    """

    # The Zyla has no ToupTek-style ISP brightness/contrast/gamma; only
    # exposure is exposed as a hardware control (see hardware_capabilities()).
    HW_RANGES: dict = {}

    # ── Discovery ─────────────────────────────────────────────────
    @staticmethod
    def enumerate() -> list[dict]:
        """Enumerate connected Andor SDK3 cameras.

        Returns a list of dicts with keys ``id`` (stable serial string),
        ``displayname`` (model), ``resolutions`` (binning presets), and
        ``preview_count``. Empty list when the SDK is unavailable.
        """
        Andor = _load_andor()
        if Andor is None:
            return []

        try:
            count = int(_andor_cameras_number(Andor))
        except Exception as exc:
            logger.warning(f"Andor enumerate failed: {exc}")
            return []

        if count <= 0:
            # The SDK3 runtime loaded but no camera was found. The usual cause is
            # a missing/failed USB DRIVER (Device Manager shows the camera with a
            # yellow bang / Code 28) — copying the SDK3 DLLs does NOT install the
            # camera's USB driver; run the Andor SDK3 / Solis installer for that.
            # Other causes: camera unpowered / cable, or bound by another process.
            logger.info(
                "Andor SDK3 loaded but 0 cameras enumerated — check the camera "
                "is powered + connected and that the Andor USB driver is "
                "installed (Device Manager must not show it with an error/Code 28)")
            return []

        devices = []
        res = [(w, h) for (w, h, _hb, _vb) in RESOLUTION_PRESETS]
        for idx in range(count):
            serial, model = _read_identity(Andor, idx)
            devices.append({
                "id": serial or f"andor#{idx}",
                "displayname": model or f"Andor Zyla #{idx}",
                "resolutions": res,
                "preview_count": len(res),
                "index": idx,
            })
            logger.info(f"Andor found: {model} (serial {serial}) idx={idx}")
        return devices

    def __init__(self):
        self._Andor = None
        self._cam = None
        self._frame: "np.ndarray | None" = None       # latest BGR8
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._reader: "threading.Thread | None" = None
        self._running = False
        self._w = 0
        self._h = 0
        self._eSize = 0
        self._device_id = ""

    # ── Lifecycle ─────────────────────────────────────────────────
    def open(self, device_id: str, resolution_index: int | None = None) -> bool:
        """Open the Andor camera identified by ``device_id`` (serial string).

        Falls back to opening index 0 if the serial can't be matched. Sets a
        default binning preset (auto-picks <=1280 wide unless resolution_index
        is given), configures Mono16 + a sane exposure, and starts acquisition
        + the reader thread.
        """
        if self._cam is not None:
            self.release()

        Andor = _load_andor()
        if Andor is None:
            return False
        self._Andor = Andor

        idx = _index_for_id(Andor, device_id)
        try:
            cam = Andor.AndorSDK3Camera(idx=idx)
        except Exception as exc:
            logger.warning(f"Andor: failed to open idx={idx}: {exc}")
            return False
        self._cam = cam
        self._device_id = device_id

        # Deliver mono-16 frames.
        try:
            cam.set_attribute_value("PixelEncoding", "Mono16")
        except Exception:
            pass

        # Resolution / binning
        if resolution_index is None:
            resolution_index = self._auto_pick_resolution_index()
        self._apply_resolution_index(resolution_index)

        # A sane default exposure so a fresh camera shows something.
        try:
            cur = cam.get_exposure()
            if not cur or cur <= 0:
                cam.set_exposure(_DEFAULT_EXPOSURE_S)
        except Exception:
            pass

        if not self._start_stream():
            self.release()
            return False

        logger.info(f"Andor opened: {self._w}x{self._h} (eSize {self._eSize})")
        return True

    def _buffer_nframes(self) -> int:
        """Ring-buffer depth sized by BYTES, not a fixed frame count.

        pylablib defaults to 100 frames; at full 2048x2048 mono-16 that is
        ~840 MB (8.4 MB/frame) — a huge allocation that can thrash RAM and stall
        the GUI. Cap the ring to ~256 MB (min 10 frames) so a full-res stream
        stays light."""
        frame_bytes = max(1, int(self._w) * int(self._h) * 2)
        return int(min(100, max(10, (256 * 1024 * 1024) // frame_bytes)))

    def _start_stream(self) -> bool:
        """Start continuous acquisition + the daemon reader thread."""
        cam = self._cam
        if cam is None:
            return False
        try:
            cam.start_acquisition(mode="sequence", nframes=self._buffer_nframes())
        except Exception as exc:
            logger.warning(f"Andor: start_acquisition failed: {exc}")
            return False
        self._running = True
        self._frame_ready.clear()
        self._reader = threading.Thread(
            target=self._reader_loop, daemon=True, name="AndorReader")
        self._reader.start()
        return True

    def _rearm_acquisition(self):
        """Re-arm a stopped acquisition in place (reader-thread self-heal).

        Called only from the reader thread when the SDK acquisition has stopped
        (e.g. a transient buffer overflow) so the live feed recovers instead of
        silently freezing."""
        cam = self._cam
        if cam is None:
            return
        try:
            cam.stop_acquisition()
        except Exception:
            pass
        try:
            cam.start_acquisition(mode="sequence", nframes=self._buffer_nframes())
        except Exception as exc:
            logger.debug(f"Andor re-arm failed: {exc}")

    def _reader_loop(self):
        """Pull the newest frame from pylablib into the BGR8 buffer.

        Resilient: logs (throttled) instead of silently swallowing errors, and
        if the SDK acquisition has actually stopped it re-arms it — so a
        transient stall self-heals rather than freezing the feed forever."""
        cam = self._cam
        fails = 0
        while self._running and cam is not None:
            try:
                cam.wait_for_frame(timeout=0.5)
            except Exception as exc:
                fails += 1
                if fails == 1 or fails % 50 == 0:
                    logger.info(
                        f"Andor reader: wait_for_frame failed x{fails}: {exc}")
                # If acquisition died (e.g. buffer overflow), re-arm it rather
                # than spin silently. Check occasionally to avoid hammering.
                if fails % 10 == 0:
                    try:
                        if not cam.acquisition_in_progress():
                            logger.info("Andor reader: acquisition stopped — re-arming")
                            self._rearm_acquisition()
                            fails = 0
                    except Exception:
                        pass
                continue
            try:
                raw = cam.read_newest_image()
            except Exception as exc:
                logger.debug(f"Andor read_newest_image error: {exc}")
                continue
            if raw is None:
                continue
            bgr = _mono_to_bgr8(raw)
            if bgr is None:
                continue
            with self._lock:
                self._frame = bgr
            self._frame_ready.set()
            fails = 0

    def isOpened(self) -> bool:
        return self._cam is not None and self._running

    def read(self) -> tuple[bool, "np.ndarray | None"]:
        """Return (ok, latest BGR8 frame). Non-blocking after the first frame."""
        if not self.isOpened():
            return False, None
        if self._frame is None:
            self._frame_ready.wait(timeout=1.0)
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
        return False, None

    def release(self):
        """Stop acquisition + close the camera, serialized against the reader."""
        self._running = False
        reader = self._reader
        self._reader = None
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            try:
                reader.join(timeout=1.5)
            except Exception:
                pass
        cam = self._cam
        with self._lock:
            self._cam = None
            self._frame = None
        if cam is not None:
            try:
                cam.stop_acquisition()
            except Exception:
                pass
            try:
                cam.close()
            except Exception:
                pass
        self._frame_ready.clear()
        if sys.meta_path is not None:
            logger.info("Andor released")

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass

    # ── Resolution / binning ──────────────────────────────────────
    def _auto_pick_resolution_index(self) -> int:
        """Pick the first preset <= 1280 wide (else full res)."""
        for i, (w, _h, _hb, _vb) in enumerate(RESOLUTION_PRESETS):
            if 0 < w <= 1280:
                return i
        return len(RESOLUTION_PRESETS) - 1

    def _apply_resolution_index(self, index: int) -> bool:
        """Set full-frame ROI at the preset's binning."""
        cam = self._cam
        if cam is None:
            return False
        index = max(0, min(int(index), len(RESOLUTION_PRESETS) - 1))
        w, h, hbin, vbin = RESOLUTION_PRESETS[index]
        ok = True
        try:
            # Full sensor AOI at the requested binning; pylablib clamps to the
            # detector and reports the achieved ROI.
            cam.set_roi(0, _ZYLA_FULL[0], 0, _ZYLA_FULL[1], hbin, vbin)
        except Exception as exc:
            logger.debug(f"Andor set_roi failed: {exc}")
            ok = False
        # Read back the achieved size.
        aw, ah = self._read_current_size(fallback=(w, h))
        self._w, self._h = aw, ah
        self._eSize = index
        return ok

    def _read_current_size(self, fallback: tuple[int, int]) -> tuple[int, int]:
        cam = self._cam
        if cam is None:
            return fallback
        try:
            roi = cam.get_roi()
            # pylablib get_roi -> (hstart, hend, vstart, vend, hbin, vbin)
            if roi and len(roi) >= 6:
                hstart, hend, vstart, vend, hbin, vbin = roi[:6]
                w = max(1, int((hend - hstart) // max(1, int(hbin))))
                h = max(1, int((vend - vstart) // max(1, int(vbin))))
                return (w, h)
        except Exception:
            pass
        # Fall back to a raw frame's shape if acquisition is live.
        with self._lock:
            f = self._frame
        if f is not None and f.ndim >= 2:
            return (int(f.shape[1]), int(f.shape[0]))
        return fallback

    def get_resolution(self) -> tuple[int, int]:
        return (self._w, self._h)

    def get_eSize(self):
        return self._eSize

    def get_resolution_list(self) -> list:
        return [(w, h) for (w, h, _hb, _vb) in RESOLUTION_PRESETS]

    def set_resolution_index(self, index: int) -> bool:
        """Switch binning preset: stop, re-ROI, restart, all under the lock."""
        cam = self._cam
        if cam is None:
            return False
        self._running = False
        reader = self._reader
        self._reader = None
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            try:
                reader.join(timeout=1.5)
            except Exception:
                pass
        try:
            cam.stop_acquisition()
        except Exception:
            pass
        with self._lock:
            self._frame = None
        ok = self._apply_resolution_index(index)
        started = self._start_stream()
        logger.info(f"Andor resolution -> {self._w}x{self._h} (eSize {self._eSize})"
                    f"{'' if (ok and started) else ' [partial]'}")
        return ok and started

    # ── Hardware controls (exposure only) ─────────────────────────
    def get_exposure_time(self):
        """Current exposure time in microseconds, or None."""
        cam = self._cam
        if cam is None:
            return None
        try:
            s = cam.get_exposure()
            return int(round(float(s) * 1e6)) if s else None
        except Exception:
            return None

    def put_exposure_time(self, microseconds) -> bool:
        cam = self._cam
        if cam is None:
            return False
        try:
            cam.set_exposure(max(0.0, float(microseconds) / 1e6))
            return True
        except Exception as exc:
            logger.debug(f"Andor set_exposure failed: {exc}")
            return False

    def get_exposure_time_range(self):
        """(min_us, max_us, default_us) or None."""
        cam = self._cam
        if cam is None:
            return None
        try:
            attr = cam.get_attribute("ExposureTime")
            lo = getattr(attr, "min", None)
            hi = getattr(attr, "max", None)
            if lo is None or hi is None:
                return None
            cur = cam.get_exposure() or lo
            return (int(round(lo * 1e6)), int(round(hi * 1e6)),
                    int(round(cur * 1e6)))
        except Exception:
            return None

    # The Zyla exposes none of these ToupTek ISP controls. Report None so the
    # settings dialog hides them; software correction still works display-side.
    def get_exposure_gain(self):
        return None

    def put_exposure_gain(self, percent) -> bool:
        return False

    def get_exposure_gain_range(self):
        return None

    def get_auto_exposure(self):
        return None

    def set_auto_exposure(self, enabled: bool) -> bool:
        return False

    def get_brightness(self):
        return None

    def put_brightness(self, v) -> bool:
        return False

    def get_contrast(self):
        return None

    def put_contrast(self, v) -> bool:
        return False

    def get_gamma(self):
        return None

    def put_gamma(self, v) -> bool:
        return False

    def get_settings(self) -> dict:
        """Read every hardware setting back from the device (for the readout).

        Same shape as ToupCamBackend.get_settings(); unsupported controls are
        None so the dialog hides them.
        """
        return {
            "brightness": None,
            "contrast": None,
            "gamma": None,
            "exposure_us": self.get_exposure_time(),
            "exposure_gain_pct": None,
            "auto_exposure": None,
            "exposure_range_us": self.get_exposure_time_range(),
            "gain_range_pct": None,
            "resolution": self.get_resolution(),
            "eSize": self.get_eSize(),
            "resolutions": self.get_resolution_list(),
            "device_id": self._device_id,
        }


# ════════════════════════════════════════════════════════════════════
#  Helpers
# ════════════════════════════════════════════════════════════════════

def _mono_to_bgr8(frame) -> "np.ndarray | None":
    """Convert a mono (2-D) uint16/uint8 frame to an 8-bit BGR image.

    Uses a per-frame 1–99 percentile auto-scale so dim (e.g. fluorescence)
    scenes remain visible. Already-BGR frames pass through unchanged.
    """
    if not _NP_AVAILABLE or frame is None:
        return None
    try:
        arr = np.asarray(frame)
    except Exception:
        return None

    # Already a 3-channel 8-bit image → pass through.
    if arr.ndim == 3 and arr.shape[2] == 3:
        return arr.astype(np.uint8, copy=False)

    # Reduce anything else to a single 2-D plane.
    if arr.ndim == 3:
        arr = arr[..., 0]
    if arr.ndim != 2:
        return None

    if arr.dtype == np.uint8:
        gray8 = arr
    else:
        # Auto-scale range from a DECIMATED sample (percentile sorts, so doing it
        # on the full 4.2M-px frame every tick is costly); the scale itself is
        # then applied to the full frame.
        step = max(1, int(max(arr.shape) // 512))
        sample = arr[::step, ::step]
        try:
            lo, hi = np.percentile(sample, (1.0, 99.0))
        except Exception:
            lo, hi = float(arr.min()), float(arr.max())
        lo, hi = float(lo), float(hi)
        if not (hi > lo):
            hi = lo + 1.0
        f = arr.astype(np.float32)
        gray8 = np.clip((f - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    try:
        import cv2
        return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
    except Exception:
        # numpy fallback: stack the plane into 3 channels.
        return np.repeat(gray8[:, :, None], 3, axis=2)


def _andor_cameras_number(Andor) -> int:
    """Number of SDK3 cameras, tolerating pylablib API-name variation."""
    for name in ("get_cameras_number_SDK3", "get_cameras_number"):
        fn = getattr(Andor, name, None)
        if callable(fn):
            try:
                return int(fn())
            except Exception:
                continue
    return 0


def _read_identity(Andor, idx: int) -> tuple[str, str]:
    """Open camera ``idx`` briefly to read (serial, model); ('','') on failure."""
    cam = None
    try:
        cam = Andor.AndorSDK3Camera(idx=idx)
        info = cam.get_device_info()
        serial = str(getattr(info, "serial_number", "") or "")
        model = str(getattr(info, "camera_model", "")
                    or getattr(info, "camera_name", "") or "")
        return serial, model
    except Exception as exc:
        logger.debug(f"Andor identity read failed for idx={idx}: {exc}")
        return "", ""
    finally:
        if cam is not None:
            try:
                cam.close()
            except Exception:
                pass


def _index_for_id(Andor, device_id: str) -> int:
    """Resolve a serial-string device_id back to a pylablib camera index."""
    if device_id:
        # Direct integer index (e.g. "andor#0" or a bare number).
        digits = "".join(ch for ch in str(device_id) if ch.isdigit())
        try:
            count = _andor_cameras_number(Andor)
        except Exception:
            count = 0
        for idx in range(count):
            serial, _model = _read_identity(Andor, idx)
            if serial and serial == device_id:
                return idx
        if str(device_id).startswith("andor#") and digits:
            return int(digits)
    return 0
